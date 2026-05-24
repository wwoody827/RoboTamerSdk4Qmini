// Pure-function test for the observation builder.
//   - Asserts dimension (44)
//   - Asserts every contributing slice maps to the right index
//   - Asserts static_flag gating at the 0.15 threshold (both sides)
//   - Asserts the ±3 clamp triggers

#include <cassert>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "user/obs_builder.h"

using namespace qmini;

namespace {

void zero(ObsInputs& in) {
    in.target_command.setZero();
    in.base_rpy.setZero();
    in.base_rpy_rate.setZero();
    in.joint_pos.setZero();
    in.joint_vel.setZero();
    in.joint_act.setZero();
    in.ref_joint_act.setZero();
    in.pm_phase.setZero();
    in.pm_f.setZero();
}

#define EXPECT_NEAR(a, b, tol) do { \
    if (std::fabs((a) - (b)) > (tol)) { \
        std::fprintf(stderr, "EXPECT_NEAR fail @ %s:%d:  %g vs %g (tol %g)\n", \
                     __FILE__, __LINE__, (double)(a), (double)(b), (double)(tol)); \
        std::exit(1); \
    } \
} while (0)

}  // namespace

int main() {
    ObsParams p;  // defaults

    // --- 1. zero input → almost-zero observation -----------------------
    {
        ObsInputs in; zero(in);
        in.pm_f.setConstant(0.5f);  // (0.5*0.3 - 1) * 0 = 0 (static_flag=0)
        auto o = build_obs(in, p);
        assert(o.size() == kObsPerStep);
        for (int i = 0; i < o.size(); ++i) EXPECT_NEAR(o(i), 0.f, 1e-6);
    }

    // --- 2. command + flag gating at 0.15 -------------------------------
    {
        ObsInputs in; zero(in);
        in.target_command << 0.10f, 0.0f, 0.0f;  // norm=0.10 < 0.15 → static_flag=0
        in.pm_phase << 0.f, 0.f;  // sin=0, cos=1
        in.pm_f.setConstant(1.0f);
        auto o = build_obs(in, p);
        EXPECT_NEAR(o(0), 0.10f, 1e-6);  // vx
        EXPECT_NEAR(o(38), 0.f, 1e-6);   // sin_L * 0 = 0
        EXPECT_NEAR(o(40), 0.f, 1e-6);   // cos_L * 0 = 0
        EXPECT_NEAR(o(42), 0.f, 1e-6);   // freq * 0 = 0
    }
    {
        ObsInputs in; zero(in);
        in.target_command << 0.16f, 0.0f, 0.0f;  // norm=0.16 ≥ 0.15 → flag=1
        in.pm_phase << 0.f, 0.f;
        in.pm_f.setConstant(1.0f);
        auto o = build_obs(in, p);
        EXPECT_NEAR(o(0), 0.16f, 1e-6);
        EXPECT_NEAR(o(38), 0.f, 1e-6);   // sin(0)=0
        EXPECT_NEAR(o(40), 1.f, 1e-6);   // cos(0)*1=1
        EXPECT_NEAR(o(42), 1.f * 0.3f - 1.f, 1e-6);  // = -0.7
    }

    // --- 3. exact-at-threshold counts as moving -------------------------
    {
        ObsInputs in; zero(in);
        in.target_command << 0.15f, 0.f, 0.f;
        in.pm_phase << 0.f, 0.f;
        in.pm_f.setConstant(0.5f);
        auto o = build_obs(in, p);
        EXPECT_NEAR(o(40), 1.f, 1e-6);   // cos_L * 1
    }

    // --- 4. obs layout: each segment lands at its slot ------------------
    {
        ObsInputs in; zero(in);
        in.target_command << 0.5f, -0.2f, 1.0f;
        in.base_rpy        << 0.1f, -0.2f, 0.3f;
        in.base_rpy_rate   << 0.4f, -0.5f, 0.6f;
        in.joint_pos.setConstant(0.1f);
        in.joint_vel.setConstant(2.0f);
        in.joint_act.setConstant(0.2f);
        in.ref_joint_act.setConstant(0.05f);
        in.pm_phase  << static_cast<float>(M_PI / 2), 0.f;
        in.pm_f      << 2.0f, 3.0f;

        auto o = build_obs(in, p);
        // command 0..2
        EXPECT_NEAR(o(0), 0.5f, 1e-6);
        EXPECT_NEAR(o(1), -0.2f, 1e-6);
        EXPECT_NEAR(o(2), 1.0f, 1e-6);
        // rpy[0..1] at 3..4 (only roll, pitch — yaw is excluded)
        EXPECT_NEAR(o(3), 0.1f, 1e-6);
        EXPECT_NEAR(o(4), -0.2f, 1e-6);
        // rpy_rate * 0.5 at 5..7
        EXPECT_NEAR(o(5),  0.4f * 0.5f, 1e-6);
        EXPECT_NEAR(o(6), -0.5f * 0.5f, 1e-6);
        EXPECT_NEAR(o(7),  0.6f * 0.5f, 1e-6);
        // joint_pos - ref at 8..17
        EXPECT_NEAR(o(8),  0.1f - 0.05f, 1e-6);
        // joint_vel * 0.1 at 18..27
        EXPECT_NEAR(o(18), 2.0f * 0.1f, 1e-6);
        // joint_act - joint_pos at 28..37
        EXPECT_NEAR(o(28), 0.2f - 0.1f, 1e-6);
        // phase_sin_cos at 38..41 (moving → flag=1)
        EXPECT_NEAR(o(38), std::sin(static_cast<float>(M_PI / 2)), 1e-5);  // sin(pi/2)=1
        EXPECT_NEAR(o(39), std::sin(0.f),                          1e-6);
        EXPECT_NEAR(o(40), std::cos(static_cast<float>(M_PI / 2)), 1e-5);
        EXPECT_NEAR(o(41), std::cos(0.f),                          1e-6);
        // freq term at 42..43: (f*0.3 - 1) * flag
        EXPECT_NEAR(o(42), 2.0f * 0.3f - 1.f, 1e-6);
        EXPECT_NEAR(o(43), 3.0f * 0.3f - 1.f, 1e-6);
    }

    // --- 5. clamp at ±3 -------------------------------------------------
    {
        ObsInputs in; zero(in);
        in.joint_vel.setConstant(100.f);  // *0.1=10 → clamped to 3
        in.target_command << 0.f, 0.f, 0.f;
        auto o = build_obs(in, p);
        for (int i = 18; i < 28; ++i) EXPECT_NEAR(o(i), 3.f, 1e-6);
    }
    {
        ObsInputs in; zero(in);
        in.joint_pos.setConstant(-100.f);
        in.ref_joint_act.setConstant(0.f);
        auto o = build_obs(in, p);
        for (int i = 8; i < 18; ++i) EXPECT_NEAR(o(i), -3.f, 1e-6);
    }

    std::printf("test_obs_builder: PASS\n");
    return 0;
}
