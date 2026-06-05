// Pure-function tests for the mjlab v24 observation builder.
//   - Asserts one frame is 39-dim and every term lands at its slot
//   - Asserts projected_gravity_from_rpy(0,0) == [0,0,-1] and small-angle signs
//   - Asserts stack_obs_per_term produces the per-term oldest->newest 117 layout

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "user/obs_builder.h"

using namespace qmini;

namespace {

void zero(ObsInputs& in) {
    in.base_ang_vel.setZero();
    in.projected_gravity.setZero();
    in.joint_pos.setZero();
    in.ref_joint_act.setZero();
    in.joint_vel.setZero();
    in.last_action.setZero();
    in.command.setZero();
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
    // --- 1. dim + zero input -------------------------------------------
    {
        ObsInputs in; zero(in);
        auto o = build_obs(in);
        assert(o.size() == kObsPerStep && kObsPerStep == 39);
        for (int i = 0; i < o.size(); ++i) EXPECT_NEAR(o(i), 0.f, 1e-6);
    }

    // --- 2. projected gravity convention -------------------------------
    {
        auto g = projected_gravity_from_rpy(0.f, 0.f);
        EXPECT_NEAR(g(0), 0.f, 1e-6);
        EXPECT_NEAR(g(1), 0.f, 1e-6);
        EXPECT_NEAR(g(2), -1.f, 1e-6);          // upright
        // pitch forward -> +x gravity component (sin(pitch))
        auto gp = projected_gravity_from_rpy(0.f, 0.2f);
        EXPECT_NEAR(gp(0), std::sin(0.2f), 1e-6);
        // roll -> -sin(roll)*cos(pitch) on y
        auto gr = projected_gravity_from_rpy(0.3f, 0.f);
        EXPECT_NEAR(gr(1), -std::sin(0.3f), 1e-6);
        EXPECT_NEAR(gr(2), -std::cos(0.3f), 1e-6);
        // gravity is a unit vector
        EXPECT_NEAR(gr.norm(), 1.f, 1e-6);
    }

    // --- 3. frame layout: each term at its slot ------------------------
    {
        ObsInputs in; zero(in);
        in.base_ang_vel      << 0.1f, 0.2f, 0.3f;        // 0..2
        in.projected_gravity << 0.0f, 0.0f, -1.0f;       // 3..5
        in.joint_pos.setConstant(0.5f);
        in.ref_joint_act.setConstant(0.2f);              // pos-ref = 0.3 at 6..15
        in.joint_vel.setConstant(1.5f);                  // 16..25
        in.last_action.setConstant(-0.4f);               // 26..35
        in.command           << 0.6f, -0.7f, 0.8f;       // 36..38

        auto o = build_obs(in);
        EXPECT_NEAR(o(0), 0.1f, 1e-6);
        EXPECT_NEAR(o(2), 0.3f, 1e-6);
        EXPECT_NEAR(o(3), 0.0f, 1e-6);
        EXPECT_NEAR(o(5), -1.0f, 1e-6);
        EXPECT_NEAR(o(6), 0.3f, 1e-6);     // joint_pos - ref
        EXPECT_NEAR(o(15), 0.3f, 1e-6);
        EXPECT_NEAR(o(16), 1.5f, 1e-6);    // joint_vel (unscaled)
        EXPECT_NEAR(o(25), 1.5f, 1e-6);
        EXPECT_NEAR(o(26), -0.4f, 1e-6);   // last_action
        EXPECT_NEAR(o(35), -0.4f, 1e-6);
        EXPECT_NEAR(o(36), 0.6f, 1e-6);    // command
        EXPECT_NEAR(o(37), -0.7f, 1e-6);
        EXPECT_NEAR(o(38), 0.8f, 1e-6);
    }

    // --- 4. per-term oldest->newest stacking ---------------------------
    {
        // Build 3 distinguishable frames: frame f has every element == f.
        std::vector<Eigen::Matrix<float, kObsPerStep, 1>> frames(3);
        for (int f = 0; f < 3; ++f)
            frames[f] = Eigen::Matrix<float, kObsPerStep, 1>::Constant(
                static_cast<float>(f));

        auto flat = stack_obs_per_term(frames);
        assert(flat.size() == kObsTotal && kObsTotal == 117);

        // Layout = for each term, 3 frames oldest->newest. With frame value==f,
        // the flattened vector must be blocks of [0,0,0,...(term dim),1,1,1,2,2,2]
        // i.e. within each per-term*frame block the value equals the frame idx.
        int w = 0;
        for (int t = 0; t < kObsNumTerms; ++t) {
            const int d = kObsTermDims[t];
            for (int f = 0; f < 3; ++f)
                for (int k = 0; k < d; ++k)
                    EXPECT_NEAR(flat[w++], static_cast<float>(f), 1e-6);
        }
        assert(w == kObsTotal);

        // Spot-check the documented head: ang_vel(t-2,t-1,t) = first 9 entries
        // = [0,0,0, 1,1,1, 2,2,2] (term dim 3, 3 frames).
        const float head[9] = {0,0,0, 1,1,1, 2,2,2};
        for (int i = 0; i < 9; ++i) EXPECT_NEAR(flat[i], head[i], 1e-6);
    }

    std::printf("test_obs_builder: PASS\n");
    return 0;
}
