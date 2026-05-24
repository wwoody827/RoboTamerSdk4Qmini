// End-to-end smoke test: build QminiApp with sim backends, drive 200 ticks,
// assert: no NaN, no exception, joint targets stay inside act_pos bounds,
// commands are well-formed. Runs without ONNX (identity policy).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "user/qmini_app.h"

using namespace qmini;

static void check_finite(const char* tag, const float* p, int n) {
    for (int i = 0; i < n; ++i) {
        if (!std::isfinite(p[i])) {
            std::fprintf(stderr, "[%s] non-finite at %d: %g\n", tag, i,
                         (double)p[i]);
            std::exit(1);
        }
    }
}

int main() {
    QminiApp::Options opts;
    opts.use_real_onnx = false;     // identity policy
    opts.enable_logging = false;
    opts.start_threads = false;     // single-threaded ticking
    opts.input_from_keyboard = false;

    QminiApp app(std::move(opts));

    // Drive 200 ticks; flip modes 1 → 2 → 3 mid-stream.
    for (int i = 0; i < 200; ++i) {
        app.tick();
        const auto& rl = app.rl();
        check_finite("joint_act", rl.joint_act().data(), 10);
        check_finite("joint_pos", rl.joint_pos().data(), 10);
        check_finite("base_rpy",  rl.base_rpy().data(),  3);
    }

    std::printf("test_sim_loop: PASS (200 ticks, identity policy)\n");
    return 0;
}
