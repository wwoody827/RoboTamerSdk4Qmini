// Joint-range measurement tool.
//
// Holds all motors limp (kp = kd = tau = 0) and continuously reads each
// joint's q. Tracks running min / max per joint and renders a live table
// alongside the URDF-declared range. Lets the operator hand-move every
// joint to its mechanical limits and verify whether those match the URDF.
//
// Usage:
//   ./joint_range_tool [--mjcf <path>] [--no-viewer] [--iface <name>]
//
// Keyboard (raw terminal):
//   r       reset all min/max to the current q
//   q,Esc   quit (prints final summary)

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
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

// URDF declared ranges per joint, from assets/q1/urdf/q1.urdf. Order
// matches kJointNames.
const float kUrdfLow[kNumJoints]  = {-0.1f, -0.3f, -2.1f,  0.0f, -2.5f,
                                     -0.7f, -0.6f,  0.0f, -2.1f,  0.0f};
const float kUrdfHigh[kNumJoints] = {+0.7f, +0.6f,  0.0f, +2.1f,  0.0f,
                                     +0.1f, +0.3f, +2.1f,  0.0f, +2.5f};

std::atomic<bool> g_quit{false};
void handle_sigint(int) { g_quit = true; }

void print_usage(const char* prog) {
    std::printf(
        "Usage: %s [options]\n"
        "\n"
        "Holds all motors limp and tracks per-joint min/max q while the\n"
        "operator manually moves each joint to its mechanical limits. Prints\n"
        "a live table comparing the measured range to the URDF range.\n"
        "\n"
        "Keys (raw terminal):\n"
        "  r        reset min/max to current q\n"
        "  q, Esc   quit + print summary\n"
        "\n"
        "Options:\n"
        "  --mjcf <path>     mujoco backend: which MJCF to load\n"
        "  --no-viewer       skip GLFW viewer (mujoco only)\n"
        "  --iface <name>    hardware backend: net iface for DDS\n"
        "  --tick-hz <hz>    control rate (default = 1/cfg.control_dt)\n"
        "  --out <path>      save final ranges as yaml (default: none)\n"
        "  -h, --help        this help\n",
        prog);
}

// ---------------------------------------------------------------------------
// Raw stdin reader. Just listens for 'r' and 'q'/Esc — no fancy queueing.
// ---------------------------------------------------------------------------
class KeyboardCmd {
public:
    enum Cmd { None, Reset, Quit };

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
    Cmd pop() {
        std::lock_guard<std::mutex> g(mu_);
        if (queue_.empty()) return None;
        Cmd c = queue_.front();
        queue_.erase(queue_.begin());
        return c;
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
                    if (c == 'r' || c == 'R') queue_.push_back(Reset);
                    else if (c == 'q' || c == 'Q' || c == 0x1b) queue_.push_back(Quit);
                }
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

void save_yaml(const std::string& path,
               const std::array<float, kNumJoints>& q_min,
               const std::array<float, kNumJoints>& q_max,
               const std::vector<float>& startq) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        std::fprintf(stderr, "[range] cannot open %s for writing\n", path.c_str());
        return;
    }
    std::time_t t = std::time(nullptr);
    std::tm tm{}; localtime_r(&t, &tm);
    char tbuf[32]; std::strftime(tbuf, sizeof(tbuf), "%Y-%m-%dT%H:%M:%S", &tm);

    std::fprintf(f,
        "# Measured per-joint q ranges, written by joint_range_tool.\n"
        "# All values are in CONTROLLER joint space (raw - startq), so\n"
        "# they sit in the same frame as URDF ranges IF startq is\n"
        "# correctly calibrated.\n"
        "#\n"
        "# Compare measured_min/max with URDF range. If they match within\n"
        "# a few degrees, the URDF limits are mechanical hard stops. If\n"
        "# they disagree, the mechanical stop is at the MEASURED value\n"
        "# (the URDF range is just software).\n"
        "#\n"
        "# date: %s\n"
        "# startq_at_capture: [", tbuf);
    for (int i = 0; i < kNumJoints; ++i) {
        std::fprintf(f, "%.4f", i < (int)startq.size() ? startq[i] : 0.0f);
        if (i + 1 < kNumJoints) std::fprintf(f, ", ");
    }
    std::fprintf(f, "]\n\njoint_ranges:\n");
    for (int i = 0; i < kNumJoints; ++i) {
        std::fprintf(f, "  %-12s: { measured_min: %+.4f, measured_max: %+.4f,"
                        " urdf_min: %+.4f, urdf_max: %+.4f }\n",
                     kJointNames[i], q_min[i], q_max[i],
                     kUrdfLow[i], kUrdfHigh[i]);
    }
    std::fclose(f);
    std::printf("[range] wrote %s\n", path.c_str());
}

}  // namespace

int main(int argc, char** argv) {
    std::string mjcf_path, iface = "eth0", out_path;
    double tick_hz = 0.0;
    bool want_viewer = true;

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
        else if (a == "--out") out_path = next("--out");
        else {
            std::fprintf(stderr, "unknown arg: %s\n", a.c_str());
            return 2;
        }
    }

    // Config + HAL setup (same pattern as ref_calibration_tool).
    ConfigParams cfg;
    const double dt = (tick_hz > 0.0) ? (1.0 / tick_hz) : cfg.control_dt;
    if (dt <= 0.0 || dt > 0.05) {
        std::fprintf(stderr, "tick dt=%g out of range\n", dt);
        return 1;
    }

    qmini::hal::HardwareConfig hw;
    hw.network_interface = iface;
    if (!mjcf_path.empty()) hw.mjcf_path = mjcf_path;
    for (int i = 0; i < kNumJoints; ++i) {
        hw.startq[i] = (i < (int)cfg.startq.size()) ? cfg.startq[i] : 0.f;
    }
    auto motor = qmini::hal::make_motor_backend(hw);
    if (!motor || !motor->start()) {
        std::fprintf(stderr, "[range] motor backend failed to start\n");
        return 1;
    }

    // Viewer.
    bool viewer_on = false;
#ifdef QMINI_HAVE_VIEWER
    if (want_viewer) {
        viewer_on = qmini::hal::mj::Viewer::instance().start();
        if (viewer_on) std::printf("[range] viewer window opened.\n");
    }
#else
    (void)want_viewer;
#endif

    std::signal(SIGINT,  handle_sigint);
    std::signal(SIGTERM, handle_sigint);

    std::printf("\n"
        "================================================================\n"
        "  JOINT RANGE MEASUREMENT  (motors LIMP — move by hand)\n"
        "  Move EACH joint slowly to BOTH of its mechanical limits while\n"
        "  watching the table. Compare measured_min/max to the URDF range.\n"
        "  Press 'r' to reset, 'q' or Esc to quit.\n"
        "================================================================\n");
    std::fflush(stdout);

    KeyboardCmd keys;
    keys.start();

    // Initialize min/max with the first reading.
    auto state0 = motor->read();
    std::array<float, kNumJoints> q_min{}, q_max{};
    for (int i = 0; i < kNumJoints; ++i) {
        q_min[i] = state0.q[i];
        q_max[i] = state0.q[i];
    }

    // Reserve display lines: 2 header + 10 joint rows + 1 footer = 13.
    constexpr int kTableLines = 13;
    for (int i = 0; i < kTableLines; ++i) std::printf("\n");
    std::printf("\033[%dA", kTableLines);   // cursor back to top of block
    std::fflush(stdout);

    auto t_next = std::chrono::steady_clock::now();
    auto t_last_paint = std::chrono::steady_clock::now()
                       - std::chrono::seconds(1);

    while (!g_quit.load()) {
        // Send limp command — kp = kd = tau = 0 for every joint.
        qmini::hal::MotorCmdFrame limp{};
        motor->send(limp);

        // Read state, update extremes.
        auto state = motor->read();
        for (int i = 0; i < kNumJoints; ++i) {
            if (state.q[i] < q_min[i]) q_min[i] = state.q[i];
            if (state.q[i] > q_max[i]) q_max[i] = state.q[i];
        }

        // Drain key queue.
        KeyboardCmd::Cmd c;
        while ((c = keys.pop()) != KeyboardCmd::None) {
            if (c == KeyboardCmd::Reset) {
                for (int i = 0; i < kNumJoints; ++i) {
                    q_min[i] = state.q[i];
                    q_max[i] = state.q[i];
                }
            } else if (c == KeyboardCmd::Quit) {
                g_quit = true;
            }
        }
        if (g_quit.load()) break;

        // Repaint at ~5 Hz.
        auto now = std::chrono::steady_clock::now();
        if (now - t_last_paint > std::chrono::milliseconds(200)) {
            t_last_paint = now;
            std::printf("\r\033[K%-13s | %8s | %8s | %8s | %6s | %s\n",
                        "joint", "current", "min", "max", "range", "URDF range");
            std::printf("\r\033[K%s\n",
                "---------------------------------------------------------------------------");
            for (int i = 0; i < kNumJoints; ++i) {
                const float range = q_max[i] - q_min[i];
                std::printf("\r\033[K%-13s | %+8.3f | %+8.3f | %+8.3f | %6.3f | [%+5.2f, %+5.2f]\n",
                            kJointNames[i],
                            state.q[i],
                            q_min[i], q_max[i], range,
                            kUrdfLow[i], kUrdfHigh[i]);
            }
            std::printf("\r\033[K\n");
            std::fflush(stdout);
            // Move cursor back to top of table for the next paint.
            std::printf("\033[%dA", kTableLines);
        }

        t_next += std::chrono::microseconds(static_cast<long>(dt * 1e6));
        std::this_thread::sleep_until(t_next);
    }

    // Move cursor below the table so the summary prints cleanly.
    std::printf("\033[%dB\n", kTableLines);

    std::printf("\n=== final ranges (controller frame: q = raw − startq) ===\n");
    std::printf("%-13s | %10s | %10s | %8s | %20s | match?\n",
                "joint", "meas. min", "meas. max", "range", "URDF range");
    std::printf("--------------------------------------------------------------------------------\n");
    for (int i = 0; i < kNumJoints; ++i) {
        const float urdf_range = kUrdfHigh[i] - kUrdfLow[i];
        const float meas_range = q_max[i] - q_min[i];
        const float min_diff = q_min[i] - kUrdfLow[i];
        const float max_diff = q_max[i] - kUrdfHigh[i];
        const bool match = std::fabs(min_diff) < 0.05f && std::fabs(max_diff) < 0.05f;
        std::printf("%-13s | %+10.4f | %+10.4f | %8.4f | [%+5.2f, %+5.2f] (%5.2f) | %s\n",
                    kJointNames[i],
                    q_min[i], q_max[i], meas_range,
                    kUrdfLow[i], kUrdfHigh[i], urdf_range,
                    match ? "yes"
                          : (meas_range < urdf_range ? "MEAS<URDF" : "MEAS>URDF"));
    }
    std::printf("\nΔ vs URDF (positive = measured is past URDF):\n");
    for (int i = 0; i < kNumJoints; ++i) {
        std::printf("  %-13s  Δmin=%+6.3f rad (%+5.1f°)   Δmax=%+6.3f rad (%+5.1f°)\n",
                    kJointNames[i],
                    q_min[i] - kUrdfLow[i],  (q_min[i] - kUrdfLow[i])  * 180.0f / static_cast<float>(M_PI),
                    q_max[i] - kUrdfHigh[i], (q_max[i] - kUrdfHigh[i]) * 180.0f / static_cast<float>(M_PI));
    }

    if (!out_path.empty()) save_yaml(out_path, q_min, q_max, cfg.startq);

    keys.stop();
#ifdef QMINI_HAVE_VIEWER
    if (viewer_on) qmini::hal::mj::Viewer::instance().stop();
#endif
    motor->stop();
    return 0;
}
