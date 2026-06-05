#pragma once

#include <vector>

#include <Eigen/Dense>

#include "utils/cpp_types.h"

namespace qmini {

// mjlab v24 deploy contract (supersedes the old 44-dim Isaac schema).
// See models/qmini_walk_v24/MODEL_CARD.md.
//
// One observation frame = 39 dims, concatenated in this order:
//   base_ang_vel(3)  body-frame gyro [wx,wy,wz] rad/s
//   projected_gravity(3)  R_body_world^T*[0,0,-1]; upright ~ [0,0,-1]
//   joint_pos(10)    measured joint pos MINUS the default/ref pose (rad)
//   joint_vel(10)    rad/s, unscaled
//   actions(10)      previous step RAW policy output (pre-scale)
//   command(3)       body-frame twist [vx, vy, yaw_rate]
//
// The policy consumes a 3-frame history flattened PER-TERM, chronological
// oldest->newest (NOT frame-major):
//   [ ang_vel(t-2..t), proj_grav(t-2..t), joint_pos(t-2..t),
//     joint_vel(t-2..t), actions(t-2..t), command(t-2..t) ]
//
// No obs scaling and no obs clamping at deploy (the ONNX graph bakes in the
// EmpiricalNormalization; obs noise is training-only).
constexpr int kObsPerStep = 39;
constexpr int kObsStack   = 3;
constexpr int kNumActions = 10;
constexpr int kObsTotal   = kObsPerStep * kObsStack;  // 117

// Per-term sizes within one frame, in concatenation order.
inline constexpr int kObsTermDims[6] = {3, 3, 10, 10, 10, 3};
inline constexpr int kObsNumTerms = 6;

// All inputs to a single-step observation. Pure: nothing here depends on
// hardware, threads, or global state. Same inputs -> same output, always.
struct ObsInputs {
    Vec3<float>  base_ang_vel;        // body-frame gyro (rad/s)
    Vec3<float>  projected_gravity;   // R^T*[0,0,-1], upright ~ [0,0,-1]
    Vec10<float> joint_pos;           // measured joint position (rad)
    Vec10<float> ref_joint_act;       // default/offset pose (rad)
    Vec10<float> joint_vel;           // rad/s
    Vec10<float> last_action;         // previous RAW policy output (pre-scale)
    Vec3<float>  command;             // body-frame [vx, vy, yaw_rate]
};

// Gravity unit vector in the base frame from roll/pitch (yaw does not affect
// gravity). At (roll=0, pitch=0) returns [0, 0, -1], matching the contract.
//   g_b = R_body_world^T * [0,0,-1]
//       = [ sin(pitch), -sin(roll)cos(pitch), -cos(roll)cos(pitch) ]
Vec3<float> projected_gravity_from_rpy(float roll, float pitch);

// Build one 39-dim observation frame. Layout mirrors mjlab's term order.
Eigen::Matrix<float, kObsPerStep, 1> build_obs(const ObsInputs& in);

// Flatten a history of frames into the per-term oldest->newest layout the
// policy expects. frames.front() is oldest, frames.back() newest. Output
// length is kObsPerStep * frames.size().
Eigen::VectorXf stack_obs_per_term(
    const std::vector<Eigen::Matrix<float, kObsPerStep, 1>>& frames);

}  // namespace qmini
