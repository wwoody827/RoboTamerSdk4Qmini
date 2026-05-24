#pragma once

#include <Eigen/Dense>

#include "utils/cpp_types.h"

namespace qmini {

// Per-step observation dimension (must match training).
// April 2026 schema: [vx, vy, yaw] command + rpy(2) + rpy_rate(3) +
// (joint_pos - ref)(10) + (joint_vel * scale)(10) + joint_act_err(10) +
// phase_sincos(4) + (freq scale - 1)(2) = 44.
constexpr int kObsPerStep = 44;

// Tunables that MUST stay in sync with training (see CLAUDE.md "Critical
// sync points"). Defaults match the post-April-2026 walk_v2 config.
struct ObsParams {
    float static_threshold   = 0.15f;
    float rpy_rate_scale     = 0.5f;
    float joint_vel_scale    = 0.1f;
    float freq_scale         = 0.3f;
    float clamp_abs          = 3.0f;
};

// All inputs to a single-step observation. Pure: nothing here depends on
// hardware, threads, or global state. Same inputs → same output, always.
struct ObsInputs {
    Vec3<float>  target_command;   // [vx, vy, yaw]
    Vec3<float>  base_rpy;         // [roll, pitch, yaw]
    Vec3<float>  base_rpy_rate;
    Vec10<float> joint_pos;
    Vec10<float> joint_vel;
    Vec10<float> joint_act;        // current target → drives joint_pos_error
    Vec10<float> ref_joint_act;
    Vec2<float>  pm_phase;         // [phase_L, phase_R] in radians
    Vec2<float>  pm_f;             // [freq_L, freq_R] in Hz
};

// 1 if ‖[vx, vy, yaw]‖ ≥ static_threshold, else 0. Matches training's
// gating of the phase-clock terms.
inline float compute_static_flag(const Vec3<float>& cmd, float threshold) {
    return cmd.norm() >= threshold ? 1.f : 0.f;
}

// Build one 44-dim observation. Layout must mirror training's
// _get_observations() exactly — see CLAUDE.md sync point #1.
Eigen::Matrix<float, kObsPerStep, 1> build_obs(
    const ObsInputs& in, const ObsParams& p = {});

}  // namespace qmini
