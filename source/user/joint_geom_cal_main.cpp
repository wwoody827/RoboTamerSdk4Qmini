// Geometric per-joint startq calibration tool.
//
// Pins each joint's startq against a *physical landmark* (carpenter's
// square, straightedge, bubble level, top/front view) rather than a
// mechanical hard stop. See GEOMETRIC_JOINT_CALIBRATION_SPEC.md.
//
// Motors are LIMP throughout — the operator hand-poses each joint to its
// landmark, presses SPACE, and the tool sets:
//
//     startq[j] = q_raw[j] − target[j]
//
// where target[j] is the precomputed URDF-frame angle at the landmark.
//
// Symmetric mode (default ON) captures the L/R mirror partner in the same
// shot — if you pose both legs at the landmark simultaneously, one SPACE
// records both. Toggle with `m`.
//
// SAFETY: limp only. The robot stays where you put it. There is no PD, no
// torque, no automatic motion. The capture changes `startq` (the controller
// frame) but the physical joint does not move.
//
// Usage:
//   cd ~/code/RoboTamerSdk4Qmini/bin
//   ./joint_geom_cal_tool
//
//   See docs/images/geom_cal/*.png for the reference poses.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "user/hal/factory.h"
#include "utils/config.h"

#ifdef QMINI_HAVE_VIEWER
#include "viewer.h"
#endif

namespace {

constexpr int kNumJoints = 10;
const char* kJointNames[kNumJoints] = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
};

// Per-joint calibration target q values (URDF / controller frame). See
// GEOMETRIC_JOINT_CALIBRATION_SPEC.md §3.
//
// Note on knee: a strict geometric "thigh ‖ shank" (180°) is at
// q = ∓0.084, but the mechanical stop sits ~3.6° short of that, so the
// joint physically can't reach the colinear pose. We instead capture
// the knee against its q=0 mechanical stop (the URDF lower/upper bound),
// which collapses this one joint to the limit-method math but keeps the
// operator workflow uniform with the other 4 joints. The stop is
// mechanically very reliable, so this is honest and sub-degree on the
// real hardware (Δ in the user's sweep was <1°).
//
//   hip_yaw   : leg straight forward (top + side view)         q_L = +0.400
//   hip_roll  : no lateral tilt (front view)                   q_L =  0.000
//   hip_pitch : thigh horizontal (bubble level, torso ⟂)       q_L = -0.715
//   knee      : leg straight, pushed against stop              q_L =  0.000
//   ankle     : foot ⟂ shank (carpenter's square, 90°)         q_L = -1.569
//
// Right = negate (or 0 for joints whose target is 0).
const float kTarget[kNumJoints] = {
    +0.400f, 0.000f, -0.715f, 0.000f, -1.569f,
    -0.400f, 0.000f, +0.715f, 0.000f, +1.569f,
};

const char* kLandmark[kNumJoints] = {
    "leg fwd (top+side)",   "front: no tilt",        "thigh horizontal",
    "knee straight (stop)", "foot ⟂ shank (90°)",
    "leg fwd (top+side)",   "front: no tilt",        "thigh horizontal",
    "knee straight (stop)", "foot ⟂ shank (90°)",
};

inline int mirror_of(int j) { return (j + 5) % kNumJoints; }

std::atomic<bool> g_quit{false};
std::atomic<bool> g_aborted{false};
void handle_sigint(int) { g_quit = true; g_aborted = true; }

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\n"
        "Calibrates startq per joint by physically posing each joint to a\n"
        "geometric landmark (square / straightedge / bubble level / front+top\n"
        "view) and capturing q_raw. Motors stay LIMP throughout — the robot\n"
        "does not move on its own.\n"
        "\n"
        "Reference images (one per landmark):\n"
        "  docs/images/geom_cal/ankle_perp.png\n"
        "  docs/images/geom_cal/knee_straight.png\n"
        "  docs/images/geom_cal/hip_yaw_forward.png\n"
        "  docs/images/geom_cal/hip_roll_no_tilt.png\n"
        "  docs/images/geom_cal/hip_pitch_thigh_horizontal.png\n"
        "\n"
        "Keys (raw terminal):\n"
        "  0-9      select joint (cursor moves)\n"
        "  [ / ]    select prev / next joint\n"
        "  SPACE    CAPTURE: startq[j] += (q_read - target). Symmetric mode\n"
        "           also captures the mirror partner with target negated.\n"
        "  m        toggle symmetric mode (default ON)\n"
        "  r        revert all session changes (back to original startq)\n"
        "  w        WRITE new startq to config.yaml (+ .bak)\n"
        "  q, Esc   ABORT: revert and exit without saving\n"
        "\n"
        "Options:\n"
        "  --independent     start in independent mode (no L/R mirror capture)\n"
        "  --mjcf <path>     mujoco backend MJCF (sim dry-run)\n"
        "  --no-viewer       skip GLFW viewer (mujoco only)\n"
        "  --iface <name>    hardware net iface\n"
        "  --tick-hz <hz>    control rate (default = 1/cfg.control_dt)\n"
        "  -h, --help        this help\n",
        prog);
}

// ---------------------------------------------------------------------------
// Raw stdin reader. Keys: 0-9, [, ], SPACE, m, r, w, q, Esc.
// ---------------------------------------------------------------------------
enum class Cmd { None, Sel0to9, Prev, Next, Capture, ToggleSym, Revert,
                 Save, Abort };

class KeyboardCmd {
public:
    bool start() {
        const bool is_tty = isatty(STDIN_FILENO);
        if (is_tty) {
            if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
                termios raw = orig_;
                raw.c_lflag &= ~(ICANON | ECHO);
                raw.c_cc[VMIN] = 0;
                raw.c_cc[VTIME] = 0;
                if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) tty_ok_ = true;
            }
        }
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
    // Pop next command; if Sel0to9, the digit is stored in `sel_out`.
    Cmd pop(int* sel_out = nullptr) {
        std::lock_guard<std::mutex> g(mu_);
        if (queue_.empty()) return Cmd::None;
        auto pair = queue_.front();
        queue_.erase(queue_.begin());
        if (sel_out) *sel_out = pair.second;
        return pair.first;
    }
private:
    void loop() {
        while (running_.load()) {
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (::poll(&p, 1, 20) > 0 && (p.revents & POLLIN)) {
                char buf[16];
                ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
                std::lock_guard<std::mutex> g(mu_);
                for (ssize_t i = 0; i < n; ++i) {
                    char c = buf[i];
                    if (c >= '0' && c <= '9')
                        queue_.push_back({Cmd::Sel0to9, c - '0'});
                    else if (c == '[')          queue_.push_back({Cmd::Prev, 0});
                    else if (c == ']')          queue_.push_back({Cmd::Next, 0});
                    else if (c == ' ')          queue_.push_back({Cmd::Capture, 0});
                    else if (c == 'm' || c == 'M') queue_.push_back({Cmd::ToggleSym, 0});
                    else if (c == 'r' || c == 'R') queue_.push_back({Cmd::Revert, 0});
                    else if (c == 'w' || c == 'W') queue_.push_back({Cmd::Save, 0});
                    else if (c == 'q' || c == 'Q' || c == 0x1b)
                        queue_.push_back({Cmd::Abort, 0});
                }
            }
        }
    }
    std::mutex mu_;
    std::vector<std::pair<Cmd, int>> queue_;
    termios orig_{};
    bool tty_ok_ = false;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

// ---------------------------------------------------------------------------
// Rewrite the startq: line in config.yaml, preserving comments/order.
// ---------------------------------------------------------------------------
bool save_startq_to_config(const std::string& path,
                           const std::array<float, kNumJoints>& sq) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "[geom] cannot read %s\n", path.c_str());
        return false;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);
    in.close();
    {
        std::ofstream bak(path + ".bak");
        if (bak) for (auto& l : lines) bak << l << "\n";
    }
    std::ostringstream rep;
    rep << "startq: [";
    for (int i = 0; i < kNumJoints; ++i) {
        rep << std::fixed << std::setprecision(6) << sq[i];
        if (i + 1 < kNumJoints) rep << ", ";
    }
    rep << "]";
    bool found = false;
    for (auto& l : lines) {
        const std::size_t p = l.find_first_not_of(" \t");
        if (p != std::string::npos && l.compare(p, 7, "startq:") == 0) {
            l = rep.str();
            found = true;
            break;
        }
    }
    if (!found) lines.push_back(rep.str());
    std::ofstream out(path);
    if (!out) {
        std::fprintf(stderr, "[geom] cannot write %s\n", path.c_str());
        return false;
    }
    for (auto& l : lines) out << l << "\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    std::string mjcf_path, iface = "eth0";
    double tick_hz = 0.0;
    bool want_viewer = true, symmetric = true;

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
        else if (a == "--mjcf") mjcf_path = next("--mjcf");
        else if (a == "--no-viewer") want_viewer = false;
        else if (a == "--iface") iface = next("--iface");
        else if (a == "--tick-hz") tick_hz = std::atof(next("--tick-hz"));
        else if (a == "--independent") symmetric = false;
        else {
            std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
            return 2;
        }
    }

    // Load config from CWD (same convention as the other tools).
    ConfigParams cfg;
    const double dt = (tick_hz > 0.0) ? (1.0 / tick_hz) : cfg.control_dt;
    if (dt <= 0.0 || dt > 0.05) {
        std::fprintf(stderr, "tick dt=%g out of range\n", dt);
        return 1;
    }
    std::array<float, kNumJoints> startq0{};      // baseline (config.yaml)
    for (int i = 0; i < kNumJoints; ++i) {
        startq0[i] = (i < (int)cfg.startq.size()) ? cfg.startq[i] : 0.f;
    }

    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    if (!mjcf_path.empty()) hw.mjcf_path = mjcf_path;
    for (int i = 0; i < kNumJoints; ++i) hw.startq[i] = startq0[i];
    auto motor = qmini::hal::make_motor_backend(hw);
    if (!motor || !motor->start()) {
        std::fprintf(stderr, "[geom] motor backend failed to start\n");
        return 1;
    }
    auto imu = qmini::hal::make_imu_backend(hw);
    if (imu && !imu->start()) imu.reset();

    bool viewer_on = false;
#ifdef QMINI_HAVE_VIEWER
    if (want_viewer) {
        viewer_on = qmini::hal::mj::Viewer::instance().start();
        if (viewer_on) std::printf("[geom] viewer window opened.\n");
    }
#else
    (void)want_viewer;
#endif

    std::signal(SIGINT,  handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    std::printf("\n"
        "================================================================\n"
        "  GEOMETRIC LANDMARK CALIBRATION  (motors LIMP — hand-pose only)\n"
        "\n"
        "  Reference images: docs/images/geom_cal/*.png\n"
        "  Pose each joint to its landmark, then press SPACE to capture.\n"
        "  Symmetric mode (default ON): one SPACE captures BOTH legs.\n"
        "\n"
        "  Keys:  0-9 select   [/] prev/next   SPACE capture\n"
        "         m sym   r revert   w save   q/Esc abort\n"
        "================================================================\n");
    std::fflush(stdout);

    KeyboardCmd keys;
    keys.start();

    // ----- State -----
    // dsq = cumulative startq delta applied this session.
    std::array<float, kNumJoints> dsq{};
    std::array<bool,  kNumJoints> captured{};   // for display
    int sel = 4;   // start on ankle_l (the most visually distinctive landmark)

    // Always emit zero-torque commands so motors stay limp.
    auto send_limp = [&]() {
        qmini::hal::MotorCmdFrame limp{};   // zero-init: kp=kd=tau=0
        motor->send(limp);
    };

    // Apply the current dsq to the motor backend (changes the HAL's idea of
    // q_offset; the robot doesn't move because PD is off).
    auto apply_startq = [&]() {
        std::array<float, kNumJoints> sq;
        for (int i = 0; i < kNumJoints; ++i) sq[i] = startq0[i] + dsq[i];
        motor->set_zero_offset(sq);
    };

    auto capture = [&](int j) {
        auto st = motor->read();
        // st.q = raw - (startq0 + dsq), so the joint's current error from the
        // landmark is (st.q - target). Correcting startq by that error makes
        // the joint read `target` at this pose. Accumulate it onto the
        // existing delta (`+=`, not `=`) so that re-capturing a joint whose
        // dsq is already non-zero this session stays correct:
        //   new startq = (startq0 + dsq) + (st.q - target) = raw - target.
        dsq[j] += st.q[j] - kTarget[j];
        captured[j] = true;
        if (symmetric) {
            int m = mirror_of(j);
            dsq[m] += st.q[m] - kTarget[m];
            captured[m] = true;
        }
        apply_startq();
    };

    // Reserve display lines: 3 header + 10 joints + 2 footer = 15.
    constexpr int kTableLines = 15;
    for (int i = 0; i < kTableLines; ++i) std::printf("\n");
    std::printf("\033[%dA", kTableLines);
    std::fflush(stdout);

    auto t_next = std::chrono::steady_clock::now();
    auto t_last_paint = std::chrono::steady_clock::now()
                       - std::chrono::seconds(1);
    bool wrote = false;

    while (!g_quit.load()) {
        send_limp();
        auto state = motor->read();

        // Drain keys.
        int sel_digit = 0;
        Cmd c;
        while ((c = keys.pop(&sel_digit)) != Cmd::None) {
            if (c == Cmd::Sel0to9) sel = sel_digit;
            else if (c == Cmd::Prev)  sel = (sel + kNumJoints - 1) % kNumJoints;
            else if (c == Cmd::Next)  sel = (sel + 1) % kNumJoints;
            else if (c == Cmd::Capture) capture(sel);
            else if (c == Cmd::ToggleSym) symmetric = !symmetric;
            else if (c == Cmd::Revert) {
                for (int i = 0; i < kNumJoints; ++i) {
                    dsq[i] = 0; captured[i] = false;
                }
                apply_startq();
            }
            else if (c == Cmd::Save) {
                std::array<float, kNumJoints> sq;
                for (int i = 0; i < kNumJoints; ++i) sq[i] = startq0[i] + dsq[i];
                if (save_startq_to_config("config.yaml", sq)) {
                    wrote = true;
                    // Update baseline so further captures don't double-count.
                    for (int i = 0; i < kNumJoints; ++i) {
                        startq0[i] = sq[i];
                        dsq[i] = 0;
                        captured[i] = false;
                    }
                }
            }
            else if (c == Cmd::Abort) {
                g_aborted = true;
                g_quit = true;
                break;
            }
        }
        if (g_quit.load()) break;

        // Throttled repaint (5 Hz).
        auto now = std::chrono::steady_clock::now();
        if (now - t_last_paint > std::chrono::milliseconds(200)) {
            t_last_paint = now;
            auto base = imu ? imu->read() : qmini::hal::BaseStateFrame{};
            std::printf("\r\x1b[K  sym=%s  IMU rpy=(%+5.3f, %+5.3f, %+5.3f)  "
                        "sel=%s   captured=%d/10%s\n",
                        symmetric ? "ON " : "OFF",
                        base.rpy[0], base.rpy[1], base.rpy[2],
                        kJointNames[sel],
                        [&]{ int n=0; for (bool b : captured) if (b) ++n; return n; }(),
                        wrote ? "   [SAVED]" : "");
            std::printf("\r\x1b[K  %-2s  %-12s  %-22s  %8s  %8s  %8s  %s\n",
                        "j", "joint", "landmark", "target",
                        "q_read", "Δ=q-tgt", "Δstartq");
            std::printf("\r\x1b[K  %s\n",
                "----------------------------------------"
                "----------------------------------------");
            for (int i = 0; i < kNumJoints; ++i) {
                const char marker = (i == sel) ? '>' : ' ';
                const char* check = captured[i] ? "✓" : " ";
                std::printf("\r\x1b[K%c%s %d  %-12s  %-22s  %+8.4f  %+8.4f  "
                            "%+8.4f  %+8.4f\n",
                            marker, check, i, kJointNames[i], kLandmark[i],
                            kTarget[i], state.q[i],
                            state.q[i] - kTarget[i],
                            dsq[i]);
            }
            std::printf("\r\x1b[K\n");
            std::fflush(stdout);
            std::printf("\033[%dA", kTableLines);
        }
        t_next += std::chrono::microseconds(static_cast<long>(dt * 1e6));
        std::this_thread::sleep_until(t_next);
    }

    // Move below the table for the final summary.
    std::printf("\033[%dB\n", kTableLines);

    if (g_aborted.load() || !wrote) {
        // Revert any uncommitted live changes.
        motor->set_zero_offset(startq0);
        if (g_aborted.load()) {
            std::printf("\n[geom] aborted — startq reverted to baseline.\n");
        } else {
            std::printf("\n[geom] exited without saving (press 'w' next time).\n");
        }
    } else {
        std::printf("\n[geom] saved startq to config.yaml "
                    "(backup: config.yaml.bak)\n");
        std::printf("\nNext: re-record the canonical limits so future\n"
                    "limit-based calibrations stay in sync.\n"
                    "  cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_range_tool --out /tmp/c.yaml\n"
                    "  cd .. && python3 tools/apply_limit_calibration.py /tmp/c.yaml --record-canonical\n");
    }

    keys.stop();
#ifdef QMINI_HAVE_VIEWER
    if (viewer_on) qmini::hal::mj::Viewer::instance().stop();
#endif
    motor->stop();
    if (imu) imu->stop();
    return 0;
}
