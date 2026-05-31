// Reference-pose calibration tool.
//
// Drives the HAL motor backend through stand mode at the MGTO pose
// (config.yaml::ref_joint_act), letting the operator translate the feet
// forward/back and lateral via arrow keys while keeping the body level
// and feet flat. Writes the calibrated absolute joint pose to
// `bin/ref_pose_calibrated.yaml` on Enter.
//
// See COM_CALIBRATION_SPEC.md for the protocol.
//
// Backends: sim / mujoco / hardware (compile-time selected via -DBACKEND=).
// Tested in mujoco with the viewer for a sim dry-run; the same binary then
// runs on hardware with no flag changes.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include "user/hal/factory.h"
#include "user/leg_ik.h"
#include "utils/config.h"

#ifdef QMINI_HAVE_VIEWER
#include "viewer.h"
#endif

namespace {

std::atomic<bool> g_abort{false};
void handle_sigint(int) { g_abort = true; }

constexpr int kNumJoints = 10;
const char* kJointNames[kNumJoints] = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
};

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\n"
        "Reference-pose calibration. Body stays level, feet stay flat.\n"
        "Operator adjusts (dx_foot, dy_foot) with arrow keys; the tool\n"
        "runs IK and slews the joints to the new pose. On Enter, writes a\n"
        "CALIBRATION RECORD (absolute joint values) to a yaml file.\n"
        "\n"
        "The output is informational only — nothing in the SDK auto-loads\n"
        "it. Copy the `ref_joint_act` list into bin/config.yaml manually if\n"
        "you decide to adopt the calibrated pose.\n"
        "\n"
        "Keys (raw terminal):\n"
        "  ↑ / ↓     dx_foot ±2 mm  (move BOTH feet fwd / back)\n"
        "  → / ←     dy_foot ±2 mm  (widen / narrow stance)\n"
        "  r         reset deltas to zero\n"
        "  s         (re-)apply slew toward current target\n"
        "  Enter     accept and write ref_pose_calibrated.yaml\n"
        "  Esc / q   abort without writing\n"
        "\n"
        "This tool ONLY tunes the MGTO standing pose (CoM). startq is\n"
        "calibrated separately by joint_jog_tool — see 1_calibrate_joints.md.\n"
        "\n"
        "Safety workflow (matches pd_calibration_tool):\n"
        "  1. Pre-motion confirmation — explicit y/Enter before any motion.\n"
        "  2. Smooth ramp from measured pose to MGTO (configurable duration).\n"
        "  3. MGTO confirmation — verify the standing pose looks correct.\n"
        "  4. Operator pose-tuning loop (arrow keys, this is where the IK runs).\n"
        "Use --yes to skip all prompts (sim / scripted runs only).\n"
        "\n"
        "Options:\n"
        "  -y, --yes              skip ALL confirmation prompts (no pre-motion\n"
        "                         gate, no MGTO gate). Sim only.\n"
        "  --ramp-in-s <s>        ramp from measured pose to MGTO (default 2)\n"
        "  --step-mm <int>        ±step in mm per arrow press (default 2)\n"
        "  --max-mm <int>         absolute cap on |dx|, |dy| (default 50)\n"
        "  --slew-ms <int>        slew time per key press (default 200 ms)\n"
        "  --out <path>           output yaml (default bin/ref_pose_calibrated.yaml)\n"
        "  --mjcf <path>          mujoco backend: which MJCF to load.\n"
        "                       Sim dry-run (robot stays put, no falling):\n"
        "                          --mjcf sim_assets/q1_sim_hung.mjcf\n"
        "                         Sim on-floor (robot tips, like hardware):\n"
        "                            --mjcf sim_assets/q1_sim.mjcf\n"
        "  --no-viewer            mujoco backend: skip GLFW viewer\n"
        "  --iface <name>         hardware backend: net iface for DDS\n"
        "  --tick-hz <hz>         control rate (default = 1/cfg.control_dt)\n"
        "  -h, --help             this help\n",
        prog);
}

// ---------------------------------------------------------------------------
// Termios-raw stdin reader. Latches the most-recent operator command as a
// (dx_delta, dy_delta) increment + control flags. Same pattern as
// hal/sim/joystick_keyboard.cpp.
// ---------------------------------------------------------------------------

enum class Cmd { None,
                 IncDx, DecDx, IncDy, DecDy,
                 Reset, Reslew, Accept, Abort };

class KeyboardCmd {
public:
    bool start() {
        const bool is_tty = isatty(STDIN_FILENO);
        if (is_tty) {
            if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
                termios raw = orig_;
                raw.c_lflag &= ~(ICANON | ECHO);
                raw.c_cc[VMIN]  = 0;
                raw.c_cc[VTIME] = 0;
                if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
                    tty_ok_ = true;
                }
            }
        }
        // Non-blocking stdin works for both TTY (raw mode) and piped input
        // (test harness). For piped input we read ANSI sequences directly.
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        running_ = true;
        thread_ = std::thread([this] { loop(); });
        return true;
    }
    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (tty_ok_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }
    // Drain queued commands (FIFO). Returns Cmd::None when empty.
    Cmd pop() {
        std::lock_guard<std::mutex> g(mu_);
        if (queue_.empty()) return Cmd::None;
        Cmd c = queue_.front();
        queue_.erase(queue_.begin());
        return c;
    }

private:
    void loop() {
        while (running_.load()) {
            struct pollfd p{STDIN_FILENO, POLLIN, 0};
            int rv = ::poll(&p, 1, 20);
            if (rv > 0 && (p.revents & POLLIN)) {
                char buf[32];
                ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
                if (n > 0) consume(buf, n);
            }
        }
    }
    void consume(const char* buf, ssize_t n) {
        std::lock_guard<std::mutex> g(mu_);
        for (ssize_t i = 0; i < n; ++i) {
            char c = buf[i];
            // ANSI arrow keys are 3-byte sequences: ESC [ A/B/C/D.
            if (c == 0x1b && i + 2 < n && buf[i + 1] == '[') {
                switch (buf[i + 2]) {
                    case 'A': queue_.push_back(Cmd::IncDx); break; // up
                    case 'B': queue_.push_back(Cmd::DecDx); break; // down
                    case 'C': queue_.push_back(Cmd::IncDy); break; // right
                    case 'D': queue_.push_back(Cmd::DecDy); break; // left
                    default: break;
                }
                i += 2;
                continue;
            }
            // Lone ESC (no bracket) -> abort.
            if (c == 0x1b) { queue_.push_back(Cmd::Abort); continue; }
            switch (c) {
                case '\n': case '\r': queue_.push_back(Cmd::Accept); break;
                case 'q': case 'Q':   queue_.push_back(Cmd::Abort);  break;
                case 'r': case 'R':   queue_.push_back(Cmd::Reset);  break;
                case 's': case 'S':   queue_.push_back(Cmd::Reslew); break;
                default: break;
            }
        }
    }
    std::mutex mu_;
    std::vector<Cmd> queue_;
    termios orig_{};
    bool tty_ok_ = false;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

// ---------------------------------------------------------------------------
// YAML writer for the offset file.
// ---------------------------------------------------------------------------

bool write_ref_pose_yaml(const std::string& path,
                         const std::array<float, kNumJoints>& ref_q_old,
                         const std::array<float, kNumJoints>& dq,
                         double dx_foot_m, double dy_foot_m,
                         const float imu_rpy_mean[3],
                         float imu_rpy_std,
                         const std::string& note) {
    std::ofstream f(path);
    if (!f) return false;
    std::time_t t = std::time(nullptr);
    std::tm tm{}; localtime_r(&t, &tm);
    char tbuf[32]; std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", &tm);

    // Absolute joint angles = old ref_joint_act + IK delta.
    std::array<float, kNumJoints> ref_q_new{};
    for (int i = 0; i < kNumJoints; ++i) ref_q_new[i] = ref_q_old[i] + dq[i];

    f << "# Calibrated reference pose, written by ref_calibration_tool.\n"
      << "#\n"
      << "# This file is a CALIBRATION RECORD, not a runtime config. Nothing\n"
      << "# in the SDK auto-loads it. To use the result, manually copy the\n"
      << "# `ref_joint_act` list below into bin/config.yaml.\n"
      << "#\n"
      << "# Order: HYL HRL HPL KL AL  HYR HRR HPR KR AR\n"
      << "# Computed via inverse kinematics from (dx_foot, dy_foot) with the\n"
      << "# body kept level and feet kept flat. See COM_CALIBRATION_SPEC.md.\n"
      << "ref_joint_act: [ ";
    for (int i = 0; i < kNumJoints; ++i) {
        f << std::fixed << std::setprecision(4) << ref_q_new[i];
        if (i + 1 < kNumJoints) f << ", ";
        if (i == 4) f << "\n                 ";   // line break between L and R
    }
    f << " ]\n"
      << "meta:\n"
      << "  date: " << tbuf << "\n"
      << "  method: foot_translation_ik\n"
      << "  dx_foot_m: " << dx_foot_m << "\n"
      << "  dy_foot_m: " << dy_foot_m << "\n"
      << "  ref_joint_act_baseline: [ ";
    for (int i = 0; i < kNumJoints; ++i) {
        f << std::fixed << std::setprecision(4) << ref_q_old[i];
        if (i + 1 < kNumJoints) f << ", ";
    }
    f << " ]\n"
      << "  delta_from_baseline: [ ";
    for (int i = 0; i < kNumJoints; ++i) {
        f << std::fixed << std::setprecision(4) << dq[i];
        if (i + 1 < kNumJoints) f << ", ";
    }
    f << " ]\n"
      << "  imu_rpy_mean_at_balance: [" << imu_rpy_mean[0] << ", "
      << imu_rpy_mean[1] << ", " << imu_rpy_mean[2] << "]\n"
      << "  imu_rpy_std: " << imu_rpy_std << "\n"
      << "  operator_notes: \"" << note << "\"\n";
    return true;
}

bool prompt_yn(const char* msg, bool default_yes = false) {
    std::printf("%s", msg);
    std::fflush(stdout);
    std::string ans;
    if (!std::getline(std::cin, ans)) return false;
    if (ans.empty()) return default_yes;
    return ans[0] == 'y' || ans[0] == 'Y';
}

}  // namespace

int main(int argc, char** argv) {
    int    step_mm = 2;
    int    max_mm  = 50;
    int    slew_ms = 200;
    std::string out_path = "ref_pose_calibrated.yaml";
    std::string dynzero_path = "dynamic_zero.yaml";
    std::string mjcf_path;
    std::string iface = "eth0";
    double tick_hz = 0.0;
    bool   want_viewer = true;
    bool   assume_yes = false;
    double ramp_in_s = 2.0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "%s requires an argument\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
        else if (a == "--step-mm") step_mm = std::atoi(next("--step-mm"));
        else if (a == "--max-mm")  max_mm  = std::atoi(next("--max-mm"));
        else if (a == "--slew-ms") slew_ms = std::atoi(next("--slew-ms"));
        else if (a == "--out")     out_path = next("--out");
        else if (a == "--dynamic-zero") dynzero_path = next("--dynamic-zero");
        else if (a == "--mjcf")    mjcf_path = next("--mjcf");
        else if (a == "--no-viewer") want_viewer = false;
        else if (a == "--iface")   iface = next("--iface");
        else if (a == "--tick-hz") tick_hz = std::atof(next("--tick-hz"));
        else if (a == "-y" || a == "--yes") assume_yes = true;
        else if (a == "--ramp-in-s") ramp_in_s = std::atof(next("--ramp-in-s"));
        else {
            std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
            print_usage(argv[0]); return 2;
        }
    }
    const double step_m = step_mm * 1e-3;
    const double max_m  = max_mm  * 1e-3;
    const double slew_s = std::max(0.05, slew_ms * 1e-3);

    // Load config.yaml from CWD (same convention as run_interface).
    ConfigParams cfg;
    if (cfg.ref_joint_act.size() < kNumJoints) {
        std::fprintf(stderr, "config.yaml::ref_joint_act has %zu < 10 entries\n",
                     cfg.ref_joint_act.size());
        return 1;
    }
    std::array<float, kNumJoints> ref_q{};
    for (int i = 0; i < kNumJoints; ++i) ref_q[i] = cfg.ref_joint_act[i];

    // dynamic_zero — same persistence file qmini_app reads. The motor backend
    // sees absolute (controller-frame + dynamic_zero); the controller sees
    // (HAL-measured - dynamic_zero). We mirror the qmini_app pattern so the
    // robot ends up at the same physical pose whether we calibrate via this
    // tool or run a deploy.
    std::array<float, kNumJoints> dynamic_zero{};
    {
        std::ifstream f(dynzero_path);
        if (f) {
            try {
                YAML::Node y = YAML::Load(f);
                auto v = y["dynamic_zero"].as<std::vector<float>>();
                for (int i = 0; i < kNumJoints && i < (int)v.size(); ++i) {
                    dynamic_zero[i] = v[i];
                }
                std::printf("[ref-cal] loaded dynamic_zero from %s:",
                            dynzero_path.c_str());
                for (int i = 0; i < kNumJoints; ++i)
                    std::printf(" %+.4f", dynamic_zero[i]);
                std::printf("\n");
            } catch (const std::exception& e) {
                std::fprintf(stderr, "[ref-cal] cannot parse %s: %s\n",
                             dynzero_path.c_str(), e.what());
            }
        } else {
            std::printf("[ref-cal] no %s — assuming zero encoder offset\n",
                        dynzero_path.c_str());
        }
    }

    const double dt = (tick_hz > 0.0) ? (1.0 / tick_hz) : cfg.control_dt;
    if (dt <= 0.0 || dt > 0.05) {
        std::fprintf(stderr, "tick dt=%g out of range\n", dt);
        return 1;
    }

    // HAL setup.
    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    if (!mjcf_path.empty()) hw.mjcf_path = mjcf_path;
    for (int i = 0; i < kNumJoints; ++i) {
        hw.startq[i] = (i < (int)cfg.startq.size()) ? cfg.startq[i] : 0.f;
    }
    auto motor = qmini::hal::make_motor_backend(hw);
    if (!motor || !motor->start()) {
        std::fprintf(stderr, "[ref-cal] motor backend failed to start\n");
        return 1;
    }
    auto imu = qmini::hal::make_imu_backend(hw);
    if (imu && !imu->start()) { imu.reset(); }

    qmini::LegIK ik(ref_q);
    double fxL, fzL, fxR, fzR;
    ik.mgto_foot(0, fxL, fzL); ik.mgto_foot(1, fxR, fzR);
    std::printf("\n"
        "================================================================\n"
        "  REF-POSE CALIBRATION  (foot-translation IK)\n"
        "  MGTO foot L = (%.4f, %.4f) m, R = (%.4f, %.4f) m\n"
        "  step  = %d mm/key   max = ±%d mm   slew = %d ms\n"
        "  output: %s\n"
        "----------------------------------------------------------------\n"
        "  TYPE IN THIS TERMINAL (no Enter needed — raw keys):\n"
        "    ↑/↓  move feet  +/-  dx (sagittal)\n"
        "    →/←  widen/narrow stance (dy)\n"
        "    r    reset to MGTO\n"
        "    Enter   accept & write %s\n"
        "    q / Esc abort without writing\n"
        "================================================================\n",
        fxL, fzL, fxR, fzR, step_mm, max_mm, slew_ms,
        out_path.c_str(), out_path.c_str());

    // Viewer (mujoco only).
    bool viewer_on = false;
#ifdef QMINI_HAVE_VIEWER
    if (want_viewer) {
        viewer_on = qmini::hal::mj::Viewer::instance().start();
        if (viewer_on) std::printf("[ref-cal] viewer window opened.\n");
    }
#else
    (void)want_viewer;
#endif

    std::signal(SIGINT,  handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    // Declared up here so teardown can stop() it whether or not we
    // reached Phase 4 (start() is only called at Phase 4; stop() on a
    // never-started KeyboardCmd is a safe no-op).
    KeyboardCmd keys;

    // State.
    double dx_foot = 0.0, dy_foot = 0.0;
    std::array<float, kNumJoints> last_dq{};      // delta we're currently using
    std::array<float, kNumJoints> target_dq{};    // delta we slew toward
    std::array<float, kNumJoints> live_dq{};      // PD-applied delta (slewing)

    auto compose_q = [&](const std::array<float, kNumJoints>& dq,
                         float out[kNumJoints]) {
        for (int i = 0; i < kNumJoints; ++i) out[i] = ref_q[i] + dq[i];
    };

    // IMU RMS accumulator for the verify phase.
    float imu_rpy_sum[3] = {0, 0, 0};
    float imu_rpy_sq[3]  = {0, 0, 0};
    int   imu_count = 0;

    // PD gains: use config.yaml. q_target is in CONTROLLER frame (centered
    // on the URDF nominal pose); we add dynamic_zero before shipping to the
    // motor backend, matching what qmini_app does in control_tick.
    auto send_cmd = [&](const float q_target[kNumJoints],
                        float kp_scale, float kd_scale) {
        qmini::hal::MotorCmdFrame cmd{};
        for (int i = 0; i < kNumJoints; ++i) {
            cmd.q_target[i] = q_target[i] + dynamic_zero[i];
            cmd.dq_target[i] = 0.f;
            cmd.kp[i] = (i < (int)cfg.kp.size() ? cfg.kp[i] : 0.f) * kp_scale;
            cmd.kd[i] = (i < (int)cfg.kd.size() ? cfg.kd[i] : 0.f) * kd_scale;
            cmd.tau_ff[i] = 0.f;
        }
        motor->send(cmd);
    };

    // ------------------------------------------------------------------
    // Phase 1: pre-motion confirmation. Until this point the motors have
    // been limp. The next step issues PD with config.yaml kp/kd — require
    // explicit y.
    // ------------------------------------------------------------------
    if (!assume_yes) {
        std::printf("\n  About to drive the robot to MGTO and enter the\n"
                    "  pose-tuning loop with full PD gains from config.yaml.\n"
                    "    ramp-in   : %.1f s\n"
                    "    output    : %s\n"
                    "    step / max: %d / %d mm\n"
                    "    Check     : robot on flat ground (or held by operator);\n"
                    "                space for legs to slew; e-stop in reach.\n",
                    ramp_in_s, out_path.c_str(), step_mm, max_mm);
        if (!prompt_yn("\n  Proceed? type 'y' then Enter "
                       "(anything else aborts): ", false)) {
            std::printf("[ref-cal] aborted at pre-motion confirmation — "
                        "no motion commanded.\n");
            motor->stop();
            if (imu) imu->stop();
            return 0;
        }
        std::printf("[ref-cal] confirmed — ramping to MGTO.\n");
        std::fflush(stdout);
    }

    // ------------------------------------------------------------------
    // Phase 2: ramp measured -> MGTO at config.yaml kp/kd.
    // ramp_start is in CONTROLLER frame; subtract dynamic_zero from raw
    // encoder reading (mirror of qmini_app::control_tick).
    // ------------------------------------------------------------------
    auto state0 = motor->read();
    std::array<float, kNumJoints> ramp_start{};
    for (int i = 0; i < kNumJoints; ++i) {
        ramp_start[i] = state0.q[i] - dynamic_zero[i];
    }

    const auto t_start = std::chrono::steady_clock::now();
    auto sleep_until_next = [dt](std::chrono::steady_clock::time_point& t) {
        t += std::chrono::microseconds(static_cast<long>(dt * 1e6));
        std::this_thread::sleep_until(t);
    };
    auto t_tick = t_start;

    std::printf("[ref-cal] ramping to MGTO over %.1f s ...\n", ramp_in_s);
    while (!g_abort.load()) {
        auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - t_start).count();
        if (elapsed >= ramp_in_s) break;
        const float a = static_cast<float>(elapsed / ramp_in_s);
        float qcmd[kNumJoints];
        for (int i = 0; i < kNumJoints; ++i) {
            qcmd[i] = ramp_start[i] + a * (ref_q[i] - ramp_start[i]);
        }
        send_cmd(qcmd, 1.f, 1.f);
        sleep_until_next(t_tick);
    }

    if (g_abort.load()) goto teardown;

    // ------------------------------------------------------------------
    // Phase 3: MGTO confirmation. Robot is now physically holding MGTO
    // via the last sent PD command (hardware backend keeps re-sending;
    // mujoco backend has physics frozen until next send_cmd).
    // ------------------------------------------------------------------
    if (!assume_yes) {
        std::printf("\n  Robot is now holding MGTO.\n"
                    "  Verify the standing pose looks correct — joints at\n"
                    "  ref_joint_act, body level, feet flat.\n");
        if (!prompt_yn("\n  Start pose-tuning loop? type 'y' then Enter "
                       "(anything else aborts): ", false)) {
            std::printf("[ref-cal] aborted at MGTO confirmation.\n");
            goto teardown;
        }
        std::printf("[ref-cal] MGTO confirmed — starting operator loop.\n");
        std::fflush(stdout);
    }

    // ------------------------------------------------------------------
    // Phase 4: operator pose-tuning loop (CoM correction). Arrow keys drive
    // (dx, dy); the IK solves the pose, and we slew live_dq -> target_dq
    // with a first-order alpha. On Enter, write the calibrated pose.
    // ------------------------------------------------------------------
    keys.start();
    {
        std::printf("[ref-cal] operator control. (dx, dy) starts at (0, 0).\n");
        std::fflush(stdout);
        const double alpha_per_tick = std::min(1.0, dt / slew_s);
        int print_throttle = 0;
        bool accepted = false;
        bool aborted  = false;

        while (!g_abort.load()) {
            // Drain keyboard. Echo each press on its own line (above the
            // live status row) so the operator sees they're being heard.
            Cmd c;
            bool dirty = false;
            while ((c = keys.pop()) != Cmd::None) {
                const char* echo = nullptr;
                switch (c) {
                    case Cmd::IncDx:
                        dx_foot = std::min(dx_foot + step_m, max_m);
                        dirty = true; echo = "↑ dx +"; break;
                    case Cmd::DecDx:
                        dx_foot = std::max(dx_foot - step_m, -max_m);
                        dirty = true; echo = "↓ dx -"; break;
                    case Cmd::IncDy:
                        dy_foot = std::min(dy_foot + step_m, max_m);
                        dirty = true; echo = "→ dy +"; break;
                    case Cmd::DecDy:
                        dy_foot = std::max(dy_foot - step_m, -max_m);
                        dirty = true; echo = "← dy -"; break;
                    case Cmd::Reset:
                        dx_foot = 0; dy_foot = 0; dirty = true;
                        echo = "r RESET (dx, dy) → 0"; break;
                    case Cmd::Reslew:
                        dirty = true; echo = "s reslew"; break;
                    case Cmd::Accept:
                        accepted = true; echo = "ENTER accept"; break;
                    case Cmd::Abort:
                        aborted  = true; echo = "ABORT"; break;
                    default: break;
                }
                if (echo) {
                    // Clear the live status line, print the echo on its own
                    // line, then let the status reprint below it on the next
                    // throttled tick.
                    std::printf("\r\x1b[K[key] %-22s → dx=%+5.1f mm  dy=%+5.1f mm\n",
                                echo, dx_foot * 1e3, dy_foot * 1e3);
                    std::fflush(stdout);
                }
                if (accepted || aborted) break;
            }
            if (accepted || aborted) break;

            if (dirty) {
                bool ok = true;
                target_dq = ik.solve(dx_foot, dy_foot, &ok);
                if (!ok) std::printf("[ref-cal] WARN: IK did not converge — clamped.\n");
                // Reset accumulator each time the target changes; the verify
                // phase uses the steady-state IMU.
                imu_rpy_sum[0] = imu_rpy_sum[1] = imu_rpy_sum[2] = 0.f;
                imu_rpy_sq[0]  = imu_rpy_sq[1]  = imu_rpy_sq[2]  = 0.f;
                imu_count = 0;
            }

            // Slew live_dq -> target_dq.
            for (int i = 0; i < kNumJoints; ++i) {
                const float a = static_cast<float>(alpha_per_tick);
                live_dq[i] += a * (target_dq[i] - live_dq[i]);
            }
            float qcmd[kNumJoints];
            compose_q(live_dq, qcmd);
            send_cmd(qcmd, 1.f, 1.f);

            // Read IMU, accumulate.
            if (imu) {
                auto base = imu->read();
                for (int k = 0; k < 3; ++k) {
                    imu_rpy_sum[k] += base.rpy[k];
                    imu_rpy_sq[k]  += base.rpy[k] * base.rpy[k];
                }
                imu_count++;
            }

            // ~5 Hz status line, written with \r so it stays on a single
            // refreshing row. Each keypress prints above it on its own line
            // (see echo block above), pushing the status down — that's the
            // visible feedback the operator needs.
            if (++print_throttle >= static_cast<int>(0.2 / dt)) {
                print_throttle = 0;
                float rpy_mean[3] = {0, 0, 0};
                if (imu_count > 0) {
                    rpy_mean[0] = imu_rpy_sum[0] / imu_count;
                    rpy_mean[1] = imu_rpy_sum[1] / imu_count;
                    rpy_mean[2] = imu_rpy_sum[2] / imu_count;
                }
                std::printf("\r\x1b[K[live] dx=%+5.1f mm  dy=%+5.1f mm  |  "
                            "IMU rpy=(%+5.3f, %+5.3f, %+5.3f) rad   "
                            "n=%d",
                            dx_foot * 1e3, dy_foot * 1e3,
                            rpy_mean[0], rpy_mean[1], rpy_mean[2], imu_count);
                std::fflush(stdout);
            }
            sleep_until_next(t_tick);
        }
        std::printf("\n");

        if (aborted) {
            std::printf("[ref-cal] aborted by user.\n");
        } else if (accepted) {
            // Compute final stats and write yaml.
            float rpy_mean[3] = {0, 0, 0};
            float rpy_std = 0.f;
            if (imu_count > 0) {
                for (int k = 0; k < 3; ++k) rpy_mean[k] = imu_rpy_sum[k] / imu_count;
                float var = 0;
                for (int k = 0; k < 3; ++k) {
                    var += imu_rpy_sq[k] / imu_count - rpy_mean[k] * rpy_mean[k];
                }
                rpy_std = std::sqrt(std::max(0.f, var / 3.f));
            }
            if (write_ref_pose_yaml(out_path, ref_q, target_dq,
                                    dx_foot, dy_foot,
                                    rpy_mean, rpy_std, "")) {
                std::printf("[ref-cal] wrote %s\n", out_path.c_str());
                std::printf("[ref-cal] dx_foot=%+.4f m  dy_foot=%+.4f m\n",
                            dx_foot, dy_foot);
                std::printf("[ref-cal] IMU rpy_mean=(%+.4f, %+.4f, %+.4f) rad  std=%.4f\n",
                            rpy_mean[0], rpy_mean[1], rpy_mean[2], rpy_std);
                std::printf("[ref-cal] joint deltas:\n");
                for (int i = 0; i < kNumJoints; ++i) {
                    std::printf("  %-12s %+8.4f rad\n", kJointNames[i], target_dq[i]);
                }
            } else {
                std::fprintf(stderr, "[ref-cal] FAILED to write %s\n", out_path.c_str());
            }
        }
    }

teardown:
    // Gentle fold: 1 s ramp to limp.
    {
        auto t_fold0 = std::chrono::steady_clock::now();
        auto t_fold  = t_fold0;
        while (true) {
            auto now = std::chrono::steady_clock::now();
            const double e = std::chrono::duration<double>(now - t_fold0).count();
            if (e >= 1.0) break;
            const float scale = static_cast<float>(1.0 - e / 1.0);
            float qcmd[kNumJoints];
            compose_q(live_dq, qcmd);
            send_cmd(qcmd, scale, scale);
            t_fold += std::chrono::microseconds(static_cast<long>(dt * 1e6));
            std::this_thread::sleep_until(t_fold);
        }
    }

    keys.stop();
#ifdef QMINI_HAVE_VIEWER
    if (viewer_on) qmini::hal::mj::Viewer::instance().stop();
#endif
    motor->stop();
    if (imu) imu->stop();
    return 0;
}
