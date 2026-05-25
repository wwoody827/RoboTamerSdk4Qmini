// Probe each FSM mode against the hung MJCF and dump joint angles vs ref.
// Run from a CWD that has config.yaml + sim_assets/.

#include <array>
#include <cmath>
#include <cstdio>
#include <string>

#include "user/qmini_app.h"

using qmini::QminiApp;

namespace {

const std::array<const char*, 10> kJointShort = {
    "HYL", "HRL", "HPL", "KL ", "AL ",
    "HYR", "HRR", "HPR", "KR ", "AR ",
};

template <typename V>
void print_row(const char* tag, const V& v) {
    std::printf("  %-10s ", tag);
    for (int i = 0; i < 10; ++i) {
        std::printf("%+6.2f ", v[i]);
    }
    std::printf("\n");
}

void probe_mode(char mode, double duration_s, float scale, const char* description) {
    QminiApp::Options opts;
    opts.use_real_onnx = false;
    opts.enable_logging = false;
    opts.start_threads = false;
    opts.enable_viewer = false;
    opts.input_from_keyboard = false;
    opts.hw.mjcf_path = "sim_assets/q1_sim_hung.mjcf";
    opts.stand_kp_scale = scale;
    opts.stand_kd_scale = scale;
    opts.initial_mode = mode;

    std::printf("\n=== mode '%c': %s (%.1fs) ===\n", mode, description, duration_s);

    QminiApp app(std::move(opts));

    const double dt = 0.015;  // control_dt from config.yaml
    const int n_ticks = static_cast<int>(duration_s / dt);

    // Sample every ~0.5 s.
    int sample_every = static_cast<int>(0.5 / dt);
    int sample_idx = 0;

    std::array<const char*, 10> header = {
        "HYL", "HRL", "HPL", "KL", "AL", "HYR", "HRR", "HPR", "KR", "AR",
    };
    std::printf("  %-10s ", "joint:");
    for (auto* h : header) std::printf("%6s ", h);
    std::printf("\n");

    for (int i = 0; i < n_ticks; ++i) {
        app.tick();
        if (i % sample_every == 0 || i == n_ticks - 1) {
            char tag[16];
            std::snprintf(tag, sizeof(tag), "t=%.2fs",  i * dt);
            print_row(tag, app.rl().joint_pos());
        }
    }
    print_row("ref", app.rl().ref_joint_act());
    print_row("act", app.rl().joint_act());
}

}  // namespace

int main(int argc, char** argv) {
    // Single-mode invocation so each run has a clean World singleton.
    // Shell loops outside if you want all modes.
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <mode_char> <duration_s> [stand_kp_scale=30]\n", argv[0]);
        return 2;
    }
    char mode = argv[1][0];
    double dur = std::stod(argv[2]);
    float scale = argc >= 4 ? std::stof(argv[3]) : 30.f;
    std::printf("hung-MJCF probe — values in radians.\n");
    std::printf("'ref' = ref_joint_act target. 'joint_pos' = measured. "
                "'act' = commanded q_target.\n");
    const char* desc = "";
    switch (mode) {
        case '1': desc = "fold/idle (kp_soft = 0 → no torque)"; break;
        case '2': desc = "stand (PD → ref over 5 s ramp)"; break;
        case '5': desc = "sin test (sinusoid on joint #sin_joint_idx)"; break;
        case '3': desc = "RL walk (identity policy → zero actions)"; break;
        default:  desc = "(unknown)"; break;
    }
    std::printf("[probe] mode=%c duration=%.1fs stand_scale=%.1f\n", mode, dur, scale);
    probe_mode(mode, dur, scale, desc);
    return 0;
}
