// Leg inverse kinematics for reference-pose calibration.
//
// Solves: given (Δx_foot, Δy_foot) in body frame, find joint deltas Δq[10]
// such that BOTH feet translate by (Δx, ±Δy) while the body stays level
// (roll=pitch=0) and the feet stay flat on the ground (foot pitch=0 in body
// frame, foot roll=0 enforced by holding hip_roll fixed).
//
// Constants are hard-coded from `assets/q1/urdf/q1.urdf` joint origins and
// rpy biases. See COM_CALIBRATION_SPEC.md §3.1 for the kinematic derivation.
//
// This is an approximation valid for |Δfoot| ≲ 0.05 m around MGTO:
//   - Lateral hip offsets ignored (treated as parallel x-z planes per side)
//   - hip_roll forced to 0 (otherwise foot tilts; ankle has no roll axis)
//   - lateral Δy uses small-angle hip_yaw rotation (foot also rotates yaw
//     slightly — acceptable for ±2 cm)

#pragma once

#include <array>

namespace qmini {

class LegIK {
public:
    // ref_joint_act = the 10-vector of MGTO joint angles in canonical order:
    //   [hip_yaw_l, hip_roll_l, hip_pitch_l, knee_l, ankle_l,
    //    hip_yaw_r, hip_roll_r, hip_pitch_r, knee_r, ankle_r]
    explicit LegIK(const std::array<float, 10>& ref_joint_act);

    // Compute joint deltas relative to MGTO. Positive Δx_foot moves both feet
    // forward (body +x). Positive Δy_foot widens the stance (both feet move
    // away from the body midline).
    //
    // Returns the 10-vector of joint deltas. Always returns; on IK failure
    // (target unreachable) the returned deltas correspond to the closest
    // feasible foot position, and `success_out` (if non-null) is set false.
    std::array<float, 10> solve(double dx_foot_m, double dy_foot_m,
                                bool* success_out = nullptr) const;

    // Public for tests: foot position in body frame given the full joint
    // vector. Sagittal only (foot_x, foot_z) -- lateral is approx.
    void fk_foot(int side /*0=L 1=R*/,
                 const std::array<float, 10>& q,
                 double& foot_x, double& foot_z) const;

    // MGTO foot positions cached at construction (for tests + display).
    void mgto_foot(int side, double& foot_x, double& foot_z) const {
        foot_x = (side == 0) ? foot_x_mgto_l_ : foot_x_mgto_r_;
        foot_z = (side == 0) ? foot_z_mgto_l_ : foot_z_mgto_r_;
    }

private:
    std::array<float, 10> ref_;
    double foot_x_mgto_l_{}, foot_z_mgto_l_{};
    double foot_x_mgto_r_{}, foot_z_mgto_r_{};

    // Internal FK with explicit (q_hp, q_k, q_a) — q_a is then constrained
    // outside.
    static void leg_fk(int side, double q_hp, double q_k, double q_a,
                       double& foot_x, double& foot_z);
    static double foot_flat_q_a(int side, double q_hp, double q_k);
    static bool sagittal_ik(int side, double target_x, double target_z,
                            double& q_hp, double& q_k, double& q_a);
};

}  // namespace qmini
