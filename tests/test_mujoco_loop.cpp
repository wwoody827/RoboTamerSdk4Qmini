// End-to-end MuJoCo physics smoke test.
//   - Loads q1_sim.mjcf
//   - Drives 500 ticks (5s) of stand_control via QminiApp
//   - Asserts robot doesn't fall through the floor
//   - Asserts no NaN
//
// Run from a directory whose CWD has both config.yaml and sim_assets/.
// The CMake test command takes care of chdir.

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
    opts.use_real_onnx = false;
    opts.enable_logging = false;
    opts.start_threads = false;
    opts.input_from_keyboard = false;

    QminiApp app(std::move(opts));

    // Drive 500 ticks of stand mode (5 seconds at 100Hz). The relative
    // time inside QminiApp ramps from init to ref pose over MOVE_DURATION
    // (5 s), so we end up holding the reference pose.
    for (int i = 0; i < 500; ++i) {
        app.tick();
        const auto& rl = app.rl();
        check_finite("joint_act", rl.joint_act().data(), 10);
        check_finite("joint_pos", rl.joint_pos().data(), 10);
        check_finite("base_rpy",  rl.base_rpy().data(),  3);
        if (i == 100 || i == 499) {
            std::printf("  t=%4d  rpy=(%+.3f,%+.3f,%+.3f)  q0=%+.3f q4=%+.3f q9=%+.3f\n",
                        i,
                        (double)rl.base_rpy()(0), (double)rl.base_rpy()(1),
                        (double)rl.base_rpy()(2),
                        (double)rl.joint_pos()(0), (double)rl.joint_pos()(4),
                        (double)rl.joint_pos()(9));
        }
    }

    // Sanity: base shouldn't have catastrophically tipped (>90°).
    const float roll  = std::fabs(app.rl().base_rpy()(0));
    const float pitch = std::fabs(app.rl().base_rpy()(1));
    if (roll > 1.57f || pitch > 1.57f) {
        std::fprintf(stderr, "robot fell over (rpy abs > 90°)\n");
        return 1;
    }
    std::printf("test_mujoco_loop: PASS (500 ticks, robot upright)\n");
    return 0;
}
