// LegIK round-trip: for a grid of (dx, dy) targets, run IK to get joint
// deltas, apply them to MGTO, then FK should recover the target foot
// position within tolerance. Foot pitch must remain zero (foot stays flat).

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "user/leg_ik.h"

using qmini::LegIK;

// MGTO crouch pose used by training / SDK (config.yaml::ref_joint_act).
// Joint order: HYL HRL HPL KL AL HYR HRR HPR KR AR
// Source: qmini_lab/source/qmini_lab/configs/base.yaml::action.ref_joint_pos
constexpr std::array<float, 10> kMGTO = {
    0.4f, -0.1f, -1.5f,  1.0f, -1.3f,
   -0.4f,  0.1f,  1.5f, -1.0f,  1.3f,
};

#define EXPECT_NEAR(a, b, tol) do { \
    const double _av = (a), _bv = (b); \
    if (std::fabs(_av - _bv) > (tol)) { \
        std::fprintf(stderr, "EXPECT_NEAR fail @ %s:%d: %g vs %g (tol %g)\n", \
                     __FILE__, __LINE__, _av, _bv, (double)(tol)); \
        std::exit(1); \
    } \
} while (0)

int main() {
    LegIK ik(kMGTO);

    // Sanity: FK at MGTO should agree with cached MGTO foot positions.
    double fxL, fzL, fxR, fzR;
    ik.fk_foot(0, kMGTO, fxL, fzL);
    ik.fk_foot(1, kMGTO, fxR, fzR);
    double mxL, mzL, mxR, mzR;
    ik.mgto_foot(0, mxL, mzL);
    ik.mgto_foot(1, mxR, mzR);
    EXPECT_NEAR(fxL, mxL, 1e-9);
    EXPECT_NEAR(fzL, mzL, 1e-9);
    EXPECT_NEAR(fxR, mxR, 1e-9);
    EXPECT_NEAR(fzR, mzR, 1e-9);

    // Print MGTO foot position once (sanity for the operator).
    std::printf("MGTO foot L = (%.4f, %.4f) m  R = (%.4f, %.4f) m\n",
                fxL, fzL, fxR, fzR);

    // Sweep grid: dx ∈ {-0.05 .. 0.05}, dy ∈ {-0.02 .. 0.02}
    int n_tested = 0;
    double worst_err_xz = 0, worst_err_pitch = 0;
    for (int idx = -5; idx <= 5; ++idx) {
        for (int idy = -2; idy <= 2; ++idy) {
            const double dx = idx * 0.01;
            const double dy = idy * 0.01;
            bool ok = true;
            const auto dq = ik.solve(dx, dy, &ok);

            // Apply deltas to MGTO and FK both feet.
            std::array<float, 10> q;
            for (int i = 0; i < 10; ++i) q[i] = kMGTO[i] + dq[i];

            double fxLn, fzLn, fxRn, fzRn;
            ik.fk_foot(0, q, fxLn, fzLn);
            ik.fk_foot(1, q, fxRn, fzRn);

            // Sagittal target: MGTO + dx.
            EXPECT_NEAR(fxLn, fxL + dx, 1e-4);
            EXPECT_NEAR(fzLn, fzL,      1e-4);
            EXPECT_NEAR(fxRn, fxR + dx, 1e-4);
            EXPECT_NEAR(fzRn, fzR,      1e-4);

            const double err = std::abs(fxLn - (fxL + dx)) + std::abs(fzLn - fzL);
            if (err > worst_err_xz) worst_err_xz = err;

            // Foot-flat in body frame: cumulative pitch about y must = 0.
            // α_foot = THIGH_PRE_RY + sign_hp·q_hp + SHIN_PRE_RY + sign_k·q_k
            //        + ANKLE_PRE_RY + sign_a·q_a
            auto cum_pitch = [&](int side) {
                const int hp = (side == 0) ? 2 : 7;
                const int k  = (side == 0) ? 3 : 8;
                const int a  = (side == 0) ? 4 : 9;
                const double sh = (side == 0) ? +1.0 : -1.0;
                const double sk = (side == 0) ? -1.0 : +1.0;
                const double sa = (side == 0) ? +1.0 : -1.0;
                return 1.5 + sh * q[hp] + 1.05 + sk * q[k]
                       + 1.22 + sa * q[a];
            };
            const double pitch_l = cum_pitch(0);
            const double pitch_r = cum_pitch(1);
            EXPECT_NEAR(pitch_l, 0.0, 1e-6);
            EXPECT_NEAR(pitch_r, 0.0, 1e-6);
            if (std::abs(pitch_l) > worst_err_pitch) worst_err_pitch = std::abs(pitch_l);
            if (std::abs(pitch_r) > worst_err_pitch) worst_err_pitch = std::abs(pitch_r);

            // hip_roll must stay at MGTO (forced 0 delta to keep foot flat).
            EXPECT_NEAR(dq[1], 0.0, 1e-9);
            EXPECT_NEAR(dq[6], 0.0, 1e-9);

            if (!ok) {
                std::fprintf(stderr, "IK did NOT converge for dx=%g dy=%g\n", dx, dy);
                std::exit(1);
            }
            ++n_tested;
        }
    }

    // dy=0 case: hip_yaw must stay 0.
    {
        const auto dq = ik.solve(0.02, 0.0);
        EXPECT_NEAR(dq[0], 0.0, 1e-9);
        EXPECT_NEAR(dq[5], 0.0, 1e-9);
    }

    // dy_foot > 0 widens stance: left hip_yaw < 0, right hip_yaw > 0
    // (URDF axis is -z, so q_yaw_l < 0 swings left leg outward to +y body).
    {
        const auto dq = ik.solve(0.0, 0.01);
        if (!(dq[0] < 0)) { std::fprintf(stderr, "expect dq_hy_l < 0, got %g\n", dq[0]); std::exit(1); }
        if (!(dq[5] > 0)) { std::fprintf(stderr, "expect dq_hy_r > 0, got %g\n", dq[5]); std::exit(1); }
    }

    std::printf("test_leg_ik: PASS (%d targets, worst xz err = %.2e, "
                "worst pitch err = %.2e)\n",
                n_tested, worst_err_xz, worst_err_pitch);
    return 0;
}
