// Joint jog + startq calibration tool.
//
// Two jobs in one binary:
//
//   1. --gen-poses <file>   (no hardware)
//        Use LegIK to emit a set of reference poses at several body heights,
//        each one body-level / feet-flat / feet-forward in URDF coords. These
//        are the q_ref targets the calibration drives the robot to.
//
//   2. interactive calibration (hardware / sim)
//        Ramp the robot to each reference pose under PD, then let the operator
//        directly jog individual joints (q_target) and trim per-joint startq
//        while watching the live IMU rpy and the feet. At each pose, once the
//        body is level (IMU) and the feet are forward+parallel (eyeball), press
//        SPACE to record {pose, q_ref, q_read, q_cmd, imu_rpy, startq}. The
//        records feed tools/calibration_fit/solve_startq.py, which averages
//        (q_read - q_ref) across poses into a refined startq.
//
// Calibration identity (dynamic_zero = 0):
//        startq_true = startq_at_record + (q_read - q_ref)
//   because q_read = raw/ratio - startq_at_record and, at a physically-correct
//   reference pose, raw/ratio - startq_true == q_ref.
//
// Usage:
//   ./joint_jog_tool --gen-poses bin/cal_poses.yaml
//   ./joint_jog_tool --poses bin/cal_poses.yaml --out /tmp/startq_records.yaml
//   ./joint_jog_tool --poses bin/cal_poses.yaml --mjcf ... --viewer   (dry-run)
//
// SAFETY: motors are LIMP until the pre-motion 'y'. After that PD is live at
// config.yaml gains. Keep the e-stop in reach; Ctrl-C soft-stops.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
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

#include <yaml-cpp/yaml.h>

#include "user/hal/factory.h"
#include "user/leg_ik.h"
#include "utils/config.h"

#ifdef QMINI_HAVE_VIEWER
#include "viewer.h"
#endif

namespace {

using qmini::hal::kNumJoints;

const char* kJointNames[kNumJoints] = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
};

// L/R mirror partner. Indices 0..4 are left joints, 5..9 the matching right
// joints in the same order, so the partner is (j+5)%10. In a symmetric stance
// q_right = -q_left for every pair (see config.yaml::ref_joint_act), so a
// symmetric move applies +Δ to a joint and -Δ to its partner.
inline int mirror_of(int j) { return (j + 5) % kNumJoints; }

// hip_yaw URDF mount bias (rpy.z): +0.4 (L) / -0.4 (R), axis -z. The leg's
// net yaw in the body frame is (mount - q); 0 = foot straight forward. Used
// only for the operator hint readout.
constexpr float kYawMountL = +0.4f;
constexpr float kYawMountR = -0.4f;

std::atomic<bool> g_abort{false};
void handle_sigint(int) { g_abort = true; }

struct Pose {
    std::string name;
    double dz_foot_m = 0.0;
    std::array<float, kNumJoints> q_ref{};
};

struct Record {
    std::string pose;
    std::array<float, kNumJoints> q_ref{};
    std::array<float, kNumJoints> q_read{};
    std::array<float, kNumJoints> q_cmd{};
    std::array<float, kNumJoints> startq{};
    float imu_rpy[3] = {0, 0, 0};
};

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n\n"
        "Modes:\n"
        "  --gen-poses <file>   generate reference poses via LegIK, write, exit\n"
        "                       (no hardware touched)\n"
        "  (default)            interactive jog + startq calibration\n\n"
        "Interactive keys (raw terminal, no Enter):\n"
        "  0-9        select joint to act on\n"
        "  [ / ]      select prev / next joint\n"
        "  up/down    (k/j)  jog selected joint q_target  +/- step\n"
        "  right/left (l/h)  trim selected joint startq   +/- 0.005 rad\n"
        "  = / -      jog step bigger / smaller\n"
        "  z          zero this joint's jog (back to pose q_ref)\n"
        "  r          reset ALL jogs to current pose q_ref (keeps startq)\n"
        "  m          toggle SYMMETRIC mode (mirror L/R, default on)\n"
        "  space      RECORD this pose (q_read, q_cmd, imu, startq)\n"
        "  n / p      next / prev reference pose (ramps there)\n"
        "  w          write current startq to config.yaml (+ .bak)\n"
        "  f          fold (ramp gains to limp) and exit\n"
        "  q / Esc    abort: revert startq, fold, exit\n\n"
        "Options:\n"
        "  --poses <file>        reference-pose yaml (default bin/cal_poses.yaml)\n"
        "  --out <file>          record output yaml (default /tmp/startq_records.yaml)\n"
        "  --heights <a,b,..>    --gen-poses: dz_foot list in m (default 0,0.02,0.04,-0.02)\n"
        "  --independent         start in independent mode (no L/R mirroring)\n"
        "  --jog-step <rad>      initial jog step (default 0.02)\n"
        "  --ramp-in-s <s>       measured->pose0 ramp (default 3)\n"
        "  --slew-s <s>          per-key / pose-change slew (default 0.4)\n"
        "  --no-fold             don't fold at exit\n"
        "  --fold-secs <s>       gain-release ramp (default 2)\n"
        "  -y, --yes             skip the pre-motion confirm (sim/scripts)\n"
        "  --mjcf <path>         mujoco backend MJCF (dry-run)\n"
        "  --no-viewer           skip GLFW viewer\n"
        "  --iface <name>        hardware net iface\n"
        "  --tick-hz <hz>        control rate (default 1/control_dt)\n"
        "  -h, --help\n",
        prog);
}

// --------------------------------------------------------------------------
// Pose file IO.
// --------------------------------------------------------------------------
bool write_poses(const std::string& path, const std::vector<Pose>& poses) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "[jog] cannot open %s for writing\n", path.c_str());
        return false;
    }
    std::fprintf(f,
        "# Reference poses for startq calibration, generated by\n"
        "# joint_jog_tool --gen-poses (via LegIK). Each q_ref is the 10-vector\n"
        "# of URDF joint angles (controller frame) that yields a body-level,\n"
        "# feet-flat, feet-forward stance at the given foot-height offset.\n"
        "# Joint order: hip_yaw_l hip_roll_l hip_pitch_l knee_l ankle_l\n"
        "#              hip_yaw_r hip_roll_r hip_pitch_r knee_r ankle_r\n"
        "poses:\n");
    for (const auto& p : poses) {
        std::fprintf(f, "  - name: %s\n", p.name.c_str());
        std::fprintf(f, "    dz_foot_m: %.4f\n", p.dz_foot_m);
        std::fprintf(f, "    q_ref: [");
        for (int i = 0; i < kNumJoints; ++i) {
            std::fprintf(f, "%+.5f%s", p.q_ref[i], i + 1 < kNumJoints ? ", " : "");
        }
        std::fprintf(f, "]\n");
    }
    std::fclose(f);
    std::printf("[jog] wrote %zu poses -> %s\n", poses.size(), path.c_str());
    return true;
}

bool load_poses(const std::string& path, std::vector<Pose>& out) {
    std::ifstream f(path);
    if (!f) {
        std::fprintf(stderr, "[jog] cannot read poses file %s\n", path.c_str());
        return false;
    }
    try {
        YAML::Node y = YAML::Load(f);
        for (const auto& n : y["poses"]) {
            Pose p;
            p.name = n["name"].as<std::string>("pose");
            p.dz_foot_m = n["dz_foot_m"].as<double>(0.0);
            auto v = n["q_ref"].as<std::vector<float>>();
            if (static_cast<int>(v.size()) < kNumJoints) {
                std::fprintf(stderr, "[jog] pose '%s' has %zu < 10 q_ref\n",
                             p.name.c_str(), v.size());
                return false;
            }
            for (int i = 0; i < kNumJoints; ++i) p.q_ref[i] = v[i];
            out.push_back(p);
        }
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[jog] parse error in %s: %s\n", path.c_str(), e.what());
        return false;
    }
    if (out.empty()) {
        std::fprintf(stderr, "[jog] no poses in %s\n", path.c_str());
        return false;
    }
    return true;
}

void write_records(const std::string& path, const std::vector<Record>& recs) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "[jog] cannot write records to %s\n", path.c_str());
        return;
    }
    std::fprintf(f,
        "# startq calibration records, written by joint_jog_tool.\n"
        "# Feed to tools/calibration_fit/solve_startq.py.\n"
        "# Per record (dynamic_zero assumed 0):\n"
        "#   startq_true[j] = startq[j] + (q_read[j] - q_ref[j])\n"
        "records:\n");
    auto arr = [&](const char* key, const std::array<float, kNumJoints>& a, int ind) {
        std::fprintf(f, "%*s%s: [", ind, "", key);
        for (int i = 0; i < kNumJoints; ++i)
            std::fprintf(f, "%+.5f%s", a[i], i + 1 < kNumJoints ? ", " : "");
        std::fprintf(f, "]\n");
    };
    for (const auto& r : recs) {
        std::fprintf(f, "  - pose: %s\n", r.pose.c_str());
        arr("q_ref", r.q_ref, 4);
        arr("q_read", r.q_read, 4);
        arr("q_cmd", r.q_cmd, 4);
        arr("startq", r.startq, 4);
        std::fprintf(f, "    imu_rpy: [%+.5f, %+.5f, %+.5f]\n",
                     r.imu_rpy[0], r.imu_rpy[1], r.imu_rpy[2]);
    }
    std::fclose(f);
}

// Rewrite config.yaml::startq in place, preserving comments (+ .bak).
bool save_startq_to_config(const std::string& path,
                           const std::array<float, kNumJoints>& sq) {
    std::ifstream in(path);
    if (!in) { std::fprintf(stderr, "[jog] cannot read %s\n", path.c_str()); return false; }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(in, line)) lines.push_back(line);
    in.close();
    { std::ofstream bak(path + ".bak"); if (bak) for (auto& l : lines) bak << l << "\n"; }
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
            l = rep.str(); found = true; break;
        }
    }
    if (!found) lines.push_back(rep.str());
    std::ofstream out(path);
    if (!out) { std::fprintf(stderr, "[jog] cannot write %s\n", path.c_str()); return false; }
    for (auto& l : lines) out << l << "\n";
    return true;
}

bool prompt_yn(const char* msg, bool default_yes) {
    std::printf("%s", msg);
    std::fflush(stdout);
    std::string ans;
    if (!std::getline(std::cin, ans)) return false;
    if (ans.empty()) return default_yes;
    return ans[0] == 'y' || ans[0] == 'Y';
}

std::vector<double> parse_doubles(const std::string& s) {
    std::vector<double> out;
    std::stringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::atof(tok.c_str()));
    }
    return out;
}

// --------------------------------------------------------------------------
// Raw-terminal keyboard reader.
// --------------------------------------------------------------------------
enum class Cmd { None, SelectJoint, NextJoint, PrevJoint, JogUp, JogDown,
                 SqInc, SqDec, StepBigger, StepSmaller, ZeroJog, ResetJogs,
                 ToggleSym, Record, NextPose, PrevPose, SaveStartq,
                 FoldExit, Abort };

struct Event { Cmd cmd; int arg; };

class Keyboard {
public:
    void start() {
        if (isatty(STDIN_FILENO) && tcgetattr(STDIN_FILENO, &orig_) == 0) {
            termios raw = orig_;
            raw.c_lflag &= ~(ICANON | ECHO);
            raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
            if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) tty_ok_ = true;
        }
        int fl = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, fl | O_NONBLOCK);
        running_ = true;
        thread_ = std::thread([this] { loop(); });
    }
    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (tty_ok_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }
    Event pop() {
        std::lock_guard<std::mutex> g(mu_);
        if (q_.empty()) return {Cmd::None, 0};
        Event e = q_.front(); q_.erase(q_.begin()); return e;
    }
private:
    void push(Cmd c, int a = 0) { q_.push_back({c, a}); }
    void loop() {
        while (running_.load()) {
            pollfd p{STDIN_FILENO, POLLIN, 0};
            if (::poll(&p, 1, 20) > 0 && (p.revents & POLLIN)) {
                char buf[32];
                ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
                std::lock_guard<std::mutex> g(mu_);
                for (ssize_t i = 0; i < n; ++i) {
                    char c = buf[i];
                    if (c == 0x1b && i + 2 < n && buf[i + 1] == '[') {
                        switch (buf[i + 2]) {
                            case 'A': push(Cmd::JogUp);   break;
                            case 'B': push(Cmd::JogDown); break;
                            case 'C': push(Cmd::SqInc);   break;
                            case 'D': push(Cmd::SqDec);   break;
                            default: break;
                        }
                        i += 2; continue;
                    }
                    if (c == 0x1b) { push(Cmd::Abort); continue; }
                    if (c >= '0' && c <= '9') { push(Cmd::SelectJoint, c - '0'); continue; }
                    switch (c) {
                        case '[': push(Cmd::PrevJoint); break;
                        case ']': push(Cmd::NextJoint); break;
                        case 'k': case 'K': push(Cmd::JogUp);   break;
                        case 'j': case 'J': push(Cmd::JogDown); break;
                        case 'l': case 'L': push(Cmd::SqInc);   break;
                        case 'h': case 'H': push(Cmd::SqDec);   break;
                        case '=': case '+': push(Cmd::StepBigger);  break;
                        case '-': case '_': push(Cmd::StepSmaller); break;
                        case 'z': case 'Z': push(Cmd::ZeroJog);  break;
                        case 'r': case 'R': push(Cmd::ResetJogs); break;
                        case 'm': case 'M': push(Cmd::ToggleSym); break;
                        case ' ':           push(Cmd::Record);   break;
                        case 'n': case 'N': push(Cmd::NextPose);  break;
                        case 'p': case 'P': push(Cmd::PrevPose);  break;
                        case 'w': case 'W': push(Cmd::SaveStartq); break;
                        case 'f': case 'F': push(Cmd::FoldExit);  break;
                        case 'q': case 'Q': push(Cmd::Abort);     break;
                        default: break;
                    }
                }
            }
        }
    }
    std::mutex mu_;
    std::vector<Event> q_;
    termios orig_{};
    bool tty_ok_ = false;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace

int main(int argc, char** argv) {
    std::string gen_poses_path, poses_path = "cal_poses.yaml";
    std::string out_path = "/tmp/startq_records.yaml";
    std::string heights_str, mjcf_path, iface = "eth0";
    double jog_step = 0.02, ramp_in_s = 3.0, slew_s = 0.4, fold_secs = 2.0;
    double tick_hz = 0.0;
    bool want_viewer = true, assume_yes = false, do_fold = true;
    bool symmetric = true;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&](const char* name) -> const char* {
            if (i + 1 >= argc) { std::fprintf(stderr, "%s needs an arg\n", name); std::exit(2); }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { print_usage(argv[0]); return 0; }
        else if (a == "--gen-poses") gen_poses_path = next("--gen-poses");
        else if (a == "--poses")     poses_path = next("--poses");
        else if (a == "--out")       out_path = next("--out");
        else if (a == "--heights")   heights_str = next("--heights");
        else if (a == "--jog-step")  jog_step = std::atof(next("--jog-step"));
        else if (a == "--ramp-in-s") ramp_in_s = std::atof(next("--ramp-in-s"));
        else if (a == "--slew-s")    slew_s = std::atof(next("--slew-s"));
        else if (a == "--independent") symmetric = false;
        else if (a == "--no-fold")   do_fold = false;
        else if (a == "--fold-secs") fold_secs = std::atof(next("--fold-secs"));
        else if (a == "-y" || a == "--yes") assume_yes = true;
        else if (a == "--mjcf")      mjcf_path = next("--mjcf");
        else if (a == "--no-viewer") want_viewer = false;
        else if (a == "--iface")     iface = next("--iface");
        else if (a == "--tick-hz")   tick_hz = std::atof(next("--tick-hz"));
        else { std::fprintf(stderr, "unknown arg: %s\n", a.c_str()); print_usage(argv[0]); return 2; }
    }

    // Config (CWD), same convention as run_interface / ref_calibration_tool.
    ConfigParams cfg;
    if (cfg.ref_joint_act.size() < kNumJoints) {
        std::fprintf(stderr, "config.yaml::ref_joint_act has %zu < 10\n",
                     cfg.ref_joint_act.size());
        return 1;
    }
    std::array<float, kNumJoints> ref_q{};
    for (int i = 0; i < kNumJoints; ++i) ref_q[i] = cfg.ref_joint_act[i];

    // ----- Mode 1: generate poses and exit (no hardware) -----
    if (!gen_poses_path.empty()) {
        std::vector<double> heights = heights_str.empty()
            ? std::vector<double>{0.0, 0.02, 0.04, -0.02}
            : parse_doubles(heights_str);
        qmini::LegIK ik(ref_q);
        std::vector<Pose> poses;
        for (double dz : heights) {
            bool ok = false;
            auto dq = ik.solve(0.0, 0.0, dz, &ok);
            Pose p;
            p.dz_foot_m = dz;
            char nm[32];
            if (std::abs(dz) < 1e-9) std::snprintf(nm, sizeof(nm), "mgto");
            else std::snprintf(nm, sizeof(nm), "%s_%dmm",
                               dz > 0 ? "crouch" : "tall",
                               static_cast<int>(std::lround(std::abs(dz) * 1000)));
            p.name = nm;
            for (int i = 0; i < kNumJoints; ++i) p.q_ref[i] = ref_q[i] + dq[i];
            if (!ok) std::fprintf(stderr, "[jog] WARNING: IK did not fully "
                                  "converge for dz=%.3f (pose '%s')\n", dz, nm);
            poses.push_back(p);
        }
        return write_poses(gen_poses_path, poses) ? 0 : 1;
    }

    // ----- Mode 2: interactive calibration -----
    std::vector<Pose> poses;
    if (!load_poses(poses_path, poses)) {
        std::fprintf(stderr, "[jog] generate one first: %s --gen-poses %s\n",
                     argv[0], poses_path.c_str());
        return 1;
    }

    const double dt = (tick_hz > 0.0) ? (1.0 / tick_hz) : cfg.control_dt;
    if (dt <= 0.0 || dt > 0.05) { std::fprintf(stderr, "tick dt=%g bad\n", dt); return 1; }

    std::array<float, kNumJoints> lo{}, hi{};
    for (int i = 0; i < kNumJoints; ++i) {
        lo[i] = (i < (int)cfg.act_pos_low.size())  ? cfg.act_pos_low[i]  : -3.f;
        hi[i] = (i < (int)cfg.act_pos_high.size()) ? cfg.act_pos_high[i] : +3.f;
    }

    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    if (!mjcf_path.empty()) hw.mjcf_path = mjcf_path;
    std::array<float, kNumJoints> startq0{};
    for (int i = 0; i < kNumJoints; ++i) {
        startq0[i] = (i < (int)cfg.startq.size()) ? cfg.startq[i] : 0.f;
        hw.startq[i] = startq0[i];
    }
    auto motor = qmini::hal::make_motor_backend(hw);
    if (!motor || !motor->start()) {
        std::fprintf(stderr, "[jog] motor backend failed to start\n");
        return 1;
    }
    auto imu = qmini::hal::make_imu_backend(hw);
    if (imu && !imu->start()) imu.reset();

    bool viewer_on = false;
#ifdef QMINI_HAVE_VIEWER
    if (want_viewer) {
        viewer_on = qmini::hal::mj::Viewer::instance().start();
        if (viewer_on) std::printf("[jog] viewer opened.\n");
    }
#else
    (void)want_viewer;
#endif

    std::signal(SIGINT, handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    std::printf("\n================================================================\n"
                "  JOINT JOG + STARTQ CALIBRATION\n"
                "  poses: %zu (from %s)   records -> %s\n"
                "================================================================\n",
                poses.size(), poses_path.c_str(), out_path.c_str());

    if (!assume_yes) {
        std::printf("\n  About to drive the robot to pose '%s' with PD gains\n"
                    "  from config.yaml (ramp-in %.1f s). Robot on the ground or\n"
                    "  held; legs free to slew; e-stop in reach.\n",
                    poses[0].name.c_str(), ramp_in_s);
        if (!prompt_yn("\n  Proceed? type 'y' then Enter: ", false)) {
            std::printf("[jog] aborted before motion.\n");
            motor->stop(); if (imu) imu->stop();
            return 0;
        }
    }

    auto send_cmd = [&](const std::array<float, kNumJoints>& q, float ks, float ds) {
        qmini::hal::MotorCmdFrame cmd{};
        for (int i = 0; i < kNumJoints; ++i) {
            cmd.q_target[i] = q[i];
            cmd.kp[i] = (i < (int)cfg.kp.size() ? cfg.kp[i] : 0.f) * ks;
            cmd.kd[i] = (i < (int)cfg.kd.size() ? cfg.kd[i] : 0.f) * ds;
        }
        motor->send(cmd);
    };
    auto tick_sleep = [dt](std::chrono::steady_clock::time_point& t) {
        t += std::chrono::microseconds(static_cast<long>(dt * 1e6));
        std::this_thread::sleep_until(t);
    };

    // Ramp measured -> pose[0].
    auto st0 = motor->read();
    std::array<float, kNumJoints> live_q;
    for (int i = 0; i < kNumJoints; ++i) live_q[i] = st0.q[i];
    {
        auto t_tick = std::chrono::steady_clock::now();
        const auto t0 = t_tick;
        std::printf("[jog] ramping to '%s' over %.1f s ...\n", poses[0].name.c_str(), ramp_in_s);
        while (!g_abort.load()) {
            double el = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();
            if (el >= ramp_in_s) break;
            float a = static_cast<float>(el / ramp_in_s);
            std::array<float, kNumJoints> q;
            for (int i = 0; i < kNumJoints; ++i)
                q[i] = live_q[i] + a * (poses[0].q_ref[i] - live_q[i]);
            send_cmd(q, 1.f, 1.f);
            tick_sleep(t_tick);
        }
        for (int i = 0; i < kNumJoints; ++i) live_q[i] = poses[0].q_ref[i];
    }

    // State.
    std::array<float, kNumJoints> dsq{};       // startq deltas (applied live)
    std::array<float, kNumJoints> jog{};       // per-joint q_target delta vs pose
    int cur = 0, sel = 3;
    std::vector<Record> records;
    auto apply_startq = [&]() {
        std::array<float, kNumJoints> sq;
        for (int i = 0; i < kNumJoints; ++i) sq[i] = startq0[i] + dsq[i];
        motor->set_zero_offset(sq);
    };

    Keyboard keys; keys.start();
    const float sq_step = 0.005f;
    bool aborted = false;
    int paint = 0;
    const int paint_period = std::max(1, static_cast<int>(0.2 / dt));
    // Lines emitted per repaint: pose + imu + header + separator + 10 joints
    // + keyhint = kNumJoints + 5. Must match exactly or the cursor-up drifts
    // and the table scrolls instead of redrawing in place.
    constexpr int kLines = kNumJoints + 5;
    for (int i = 0; i < kLines; ++i) std::printf("\n");
    std::printf("\033[%dA", kLines);

    auto t_tick = std::chrono::steady_clock::now();
    const double alpha = std::min(1.0, dt / std::max(0.05, slew_s));

    while (!g_abort.load()) {
        // target = pose q_ref + jog, clamped; slew live -> target.
        std::array<float, kNumJoints> target;
        for (int i = 0; i < kNumJoints; ++i) {
            float t = poses[cur].q_ref[i] + jog[i];
            target[i] = std::min(std::max(t, lo[i]), hi[i]);
            live_q[i] += static_cast<float>(alpha) * (target[i] - live_q[i]);
        }
        send_cmd(live_q, 1.f, 1.f);

        // Keys.
        Event e;
        while ((e = keys.pop()).cmd != Cmd::None) {
            switch (e.cmd) {
                case Cmd::SelectJoint: sel = e.arg; break;
                case Cmd::NextJoint:   sel = (sel + 1) % kNumJoints; break;
                case Cmd::PrevJoint:   sel = (sel + kNumJoints - 1) % kNumJoints; break;
                case Cmd::JogUp: {
                    float d = static_cast<float>(jog_step);
                    jog[sel] += d;
                    if (symmetric) jog[mirror_of(sel)] -= d;
                    break;
                }
                case Cmd::JogDown: {
                    float d = static_cast<float>(jog_step);
                    jog[sel] -= d;
                    if (symmetric) jog[mirror_of(sel)] += d;
                    break;
                }
                case Cmd::SqInc:
                    dsq[sel] += sq_step;
                    if (symmetric) dsq[mirror_of(sel)] -= sq_step;
                    apply_startq(); break;
                case Cmd::SqDec:
                    dsq[sel] -= sq_step;
                    if (symmetric) dsq[mirror_of(sel)] += sq_step;
                    apply_startq(); break;
                case Cmd::StepBigger:  jog_step = std::min(0.2, jog_step * 2.0); break;
                case Cmd::StepSmaller: jog_step = std::max(0.0025, jog_step / 2.0); break;
                case Cmd::ZeroJog:
                    jog[sel] = 0.f;
                    if (symmetric) jog[mirror_of(sel)] = 0.f;
                    break;
                case Cmd::ResetJogs:   jog = {}; break;
                case Cmd::ToggleSym:   symmetric = !symmetric; break;
                case Cmd::NextPose:    cur = (cur + 1) % (int)poses.size(); jog = {}; break;
                case Cmd::PrevPose:    cur = (cur + (int)poses.size() - 1) % (int)poses.size(); jog = {}; break;
                case Cmd::Record: {
                    auto st = motor->read();
                    Record r;
                    r.pose = poses[cur].name;
                    auto base = imu ? imu->read() : qmini::hal::BaseStateFrame{};
                    for (int i = 0; i < kNumJoints; ++i) {
                        r.q_ref[i]  = poses[cur].q_ref[i];
                        r.q_read[i] = st.q[i];
                        r.q_cmd[i]  = live_q[i];
                        r.startq[i] = startq0[i] + dsq[i];
                    }
                    for (int k = 0; k < 3; ++k) r.imu_rpy[k] = base.rpy[k];
                    records.push_back(r);
                    write_records(out_path, records);
                    break;
                }
                case Cmd::SaveStartq: {
                    std::array<float, kNumJoints> sq;
                    for (int i = 0; i < kNumJoints; ++i) sq[i] = startq0[i] + dsq[i];
                    if (save_startq_to_config("config.yaml", sq))
                        std::printf("\n[jog] startq saved to config.yaml (+.bak)\n");
                    break;
                }
                case Cmd::FoldExit: g_abort = true; break;
                case Cmd::Abort:    aborted = true; g_abort = true; break;
                default: break;
            }
            if (g_abort.load()) break;
        }

        if (++paint >= paint_period) {
            paint = 0;
            auto st = motor->read();
            auto base = imu ? imu->read() : qmini::hal::BaseStateFrame{};
            std::printf("\r\033[Kpose [%d/%zu] %-12s   jog_step=%.4f   recorded=%zu   "
                        "mode=%s\n",
                        cur + 1, poses.size(), poses[cur].name.c_str(),
                        jog_step, records.size(),
                        symmetric ? "SYMMETRIC" : "independent");
            std::printf("\r\033[KIMU r=%+.3f p=%+.3f | foot-fwd L=%+.3f R=%+.3f "
                        "(all ->0)\n",
                        base.rpy[0], base.rpy[1],
                        kYawMountL - st.q[0], kYawMountR - st.q[5]);
            std::printf("\r\033[K %-3s %-12s %9s %9s %9s %11s\n",
                        "sel", "joint", "q_cmd", "q_read", "startq", "Δstartq");
            std::printf("\r\033[K%s\n",
                "----------------------------------------------------------------");
            const int mir = symmetric ? mirror_of(sel) : -1;
            for (int i = 0; i < kNumJoints; ++i) {
                const char* mark = (i == sel) ? " > " : (i == mir ? " ~ " : "   ");
                std::printf("\r\033[K %-3s %-12s %+9.4f %+9.4f %+9.5f %+11.5f\n",
                            mark, kJointNames[i],
                            live_q[i], st.q[i], startq0[i] + dsq[i], dsq[i]);
            }
            std::printf("\r\033[K0-9 sel  jk jog  hl startq  m sym  SPC rec  "
                        "np pose  w save  f/q exit\n");
            std::fflush(stdout);
            std::printf("\033[%dA", kLines);
        }
        tick_sleep(t_tick);
    }
    std::printf("\033[%dB\n", kLines);

    // Restore terminal before any blocking work.
    keys.stop();

    if (aborted) {
        std::printf("[jog] aborted — reverting startq to original.\n");
        motor->set_zero_offset(startq0);
    }

    // Fold: ramp gains to zero while holding current live_q, then limp.
    if (do_fold && fold_secs > 0.0) {
        std::printf("[jog] folding (%.1f s) ...\n", fold_secs);
        auto tf = std::chrono::steady_clock::now();
        const auto tf0 = tf;
        g_abort = false;  // let fold complete even after Ctrl-C/abort
        while (true) {
            double el = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - tf0).count();
            if (el >= fold_secs || g_abort.load()) break;
            float s = static_cast<float>(1.0 - el / fold_secs);
            send_cmd(live_q, s, s);
            tick_sleep(tf);
        }
    }
    qmini::hal::MotorCmdFrame limp{};
    motor->send(limp);

    if (!records.empty())
        std::printf("[jog] %zu records -> %s\n"
                    "      solve with: python3 tools/calibration_fit/solve_startq.py %s\n",
                    records.size(), out_path.c_str(), out_path.c_str());

#ifdef QMINI_HAVE_VIEWER
    if (viewer_on) qmini::hal::mj::Viewer::instance().stop();
#endif
    motor->stop();
    if (imu) imu->stop();
    return 0;
}
