#pragma once

// Trial schedule for PD calibration (see PD_CALIBRATION_SPEC.md).
// Each trial is a 9-tuple of (joint, test, kp, kd, amp, freq, pose_id, label).
// `compute_offset(t)` returns the per-tick offset that should be added to
// MGTO[joint] for q_target[joint]. All other joints stay at MGTO + hold-gains.

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace qmini {
namespace calib {

constexpr int kNumJoints = 10;

// Joint name table (canonical order, must match config.yaml startq layout).
// Index 0 = HYL = hip_yaw_l, ..., index 9 = AR = ankle_r.
extern const std::array<const char*, kNumJoints> kJointNames;

enum class TestKind { Step = 'A', Sine = 'B', Chirp = 'C' };

struct PoseRef {
    std::string label;                              // "MGTO", "crouch", "tall"
    std::array<float, kNumJoints> q_target{};       // joint targets in controller frame
};

struct Trial {
    int joint;                  // 0..9
    TestKind test;              // A / B / C
    float kp;                   // gain for joint-under-test
    float kd;
    float amp;                  // rad
    float freq_hz;              // for sine; -1 for step; 0 for chirp (swept)
    int pose_id;                // index into pose_table
    std::string label;          // e.g. "A_kp30_kd1.0_step"
    double duration_s;          // total trial length
};

// Step trial uses ±amp at hard-coded segment times (see spec §5.1).
struct StepSchedule {
    static constexpr double seg = 2.0;  // each segment 2 s
    static constexpr double settle = 1.0;
    // Returns multiplier in {-1, 0, +1} to apply to amp.
    static float multiplier(double t);
    static constexpr double total = settle + 4 * seg;  // 9 s
};

// Returns the offset (rad) to add to mgto[trial.joint] at time t in trial.
float trial_offset(const Trial& trial, double t);

// Clip amp to fit within (lo, hi) joint range relative to mgto value, with 70% safety margin.
float safe_amp(float amp, float mgto_val, float lo, float hi);

// Builders -----------------------------------------------------------------

// Default protocol: Tests A+B+C, single pose (MGTO).
// `kp_grid` = {30,50,80}, `kd_grid` = {0.5,1.0,2.0} → 9 step trials per joint.
std::vector<Trial> build_default_plan(
    const std::array<float, kNumJoints>& act_pos_low,
    const std::array<float, kNumJoints>& act_pos_high,
    const PoseRef& mgto_pose);

// Quick smoke-test plan: 1 joint × 1 step × 2 s. Used by --quick.
std::vector<Trial> build_quick_plan(int joint_index = 0);

}  // namespace calib
}  // namespace qmini
