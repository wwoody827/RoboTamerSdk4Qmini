// Pure tests for math helpers carried through the HAL refactor.

#include <cassert>
#include <cmath>
#include <cstdio>

#include "user/rl_controller.h"

using qmini::RLController;

#define EXPECT_NEAR(a, b, tol) do { \
    if (std::fabs((a) - (b)) > (tol)) { \
        std::fprintf(stderr, "EXPECT_NEAR fail @ %s:%d:  %g vs %g\n", \
                     __FILE__, __LINE__, (double)(a), (double)(b)); \
        std::exit(1); \
    } \
} while (0)

int main() {
    // smallest_signed_angle_between: wraps to (-pi, pi].
    EXPECT_NEAR(RLController::smallest_signed_angle_between(0.f, 0.5f),  0.5f, 1e-6);
    EXPECT_NEAR(RLController::smallest_signed_angle_between(0.5f, 0.f), -0.5f, 1e-6);
    // crossing +pi -> negative wrap
    EXPECT_NEAR(RLController::smallest_signed_angle_between(
                    -3.0f, 3.0f),
                -2.f * static_cast<float>(M_PI) + 6.f, 1e-5);
    // exp filter: 0.5 weight blends equally
    EXPECT_NEAR(RLController::exp_filter(10.f, 0.f, 0.5f), 5.f, 1e-6);
    EXPECT_NEAR(RLController::exp_filter(0.f, 10.f, 0.5f), 5.f, 1e-6);
    EXPECT_NEAR(RLController::exp_filter(0.f, 1.f, 0.f),   1.f, 1e-6);
    EXPECT_NEAR(RLController::exp_filter(2.f, 1.f, 1.f),   2.f, 1e-6);

    std::printf("test_math: PASS\n");
    return 0;
}
