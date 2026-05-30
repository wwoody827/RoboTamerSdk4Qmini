#include "user/leg_ik.h"

#include <cmath>
#include <limits>

namespace qmini {
namespace {

// URDF kinematic constants — see assets/q1/urdf/q1.urdf joint origins.
// All offsets are in the parent link's frame at q=0 (post rpy).
constexpr double HP_OFFSET_X      = 0.0165;      // hip_pitch_l origin in hip_roll_l frame
constexpr double HP_OFFSET_Z      = 0.0;
constexpr double L_THIGH_X        = -0.081317;   // knee_pitch_l origin in hip_pitch_l frame
constexpr double L_THIGH_Z        = -0.081317;
constexpr double L_SHIN_X         = 0.053013;    // ankle_pitch_l origin in knee_pitch_l frame
constexpr double L_SHIN_Z         = -0.14565;
constexpr double L_FOOT           = 0.030;       // ankle joint to foot contact (approx)
constexpr double THIGH_PRE_RY     = 1.5;         // rpy.y at hip_pitch_l joint origin
constexpr double SHIN_PRE_RY      = 1.05;        // rpy.y at knee_pitch_l joint origin
constexpr double ANKLE_PRE_RY     = 1.22;        // rpy.y at ankle_pitch_l joint origin
constexpr double FOOT_FLAT_OFFSET = THIGH_PRE_RY + SHIN_PRE_RY + ANKLE_PRE_RY;

// Joint-axis y-sign per side, per joint (URDF axes for L vs R are mirrored):
//   L:  hp_axis=+y, k_axis=-y, a_axis=+y
//   R:  hp_axis=-y, k_axis=+y, a_axis=-y
// A positive joint angle rotates the child link by q · (axis · ŷ) rad about ŷ.
inline double sign_hp(int side) { return (side == 0) ? +1.0 : -1.0; }
inline double sign_k (int side) { return (side == 0) ? -1.0 : +1.0; }
inline double sign_a (int side) { return (side == 0) ? +1.0 : -1.0; }

// Rotate (x, z) by α about y-axis in-place.
inline void rot_y(double alpha, double& x, double& z) {
    const double c = std::cos(alpha);
    const double s = std::sin(alpha);
    const double xn =  x * c + z * s;
    const double zn = -x * s + z * c;
    x = xn; z = zn;
}

}  // namespace

void LegIK::leg_fk(int side, double q_hp, double q_k, double q_a,
                   double& foot_x, double& foot_z) {
    const double a_thigh = THIGH_PRE_RY + sign_hp(side) * q_hp;
    const double a_shin  = a_thigh      + SHIN_PRE_RY  + sign_k(side)  * q_k;
    const double a_foot  = a_shin       + ANKLE_PRE_RY + sign_a(side)  * q_a;

    double dx, dz;
    dx = L_THIGH_X; dz = L_THIGH_Z; rot_y(a_thigh, dx, dz);
    const double knee_x = HP_OFFSET_X + dx;
    const double knee_z = HP_OFFSET_Z + dz;

    dx = L_SHIN_X;  dz = L_SHIN_Z;  rot_y(a_shin,  dx, dz);
    const double ankle_x = knee_x + dx;
    const double ankle_z = knee_z + dz;

    dx = 0.0;       dz = -L_FOOT;   rot_y(a_foot,  dx, dz);
    foot_x = ankle_x + dx;
    foot_z = ankle_z + dz;
}

double LegIK::foot_flat_q_a(int side, double q_hp, double q_k) {
    // Foot-flat constraint in body frame: α_foot = 0
    //   THIGH_PRE_RY + sign_hp·q_hp + SHIN_PRE_RY + sign_k·q_k
    //                + ANKLE_PRE_RY + sign_a·q_a = 0
    // ⇒ q_a = (-FOOT_FLAT_OFFSET - sign_hp·q_hp - sign_k·q_k) / sign_a
    return (-FOOT_FLAT_OFFSET - sign_hp(side) * q_hp - sign_k(side) * q_k)
           / sign_a(side);
}

bool LegIK::sagittal_ik(int side, double target_x, double target_z,
                        double& q_hp, double& q_k, double& q_a) {
    constexpr int    kMaxIter = 30;
    constexpr double kTol     = 1e-7;
    constexpr double kEps     = 1e-4;
    constexpr double kMaxStep = 0.05;

    for (int iter = 0; iter < kMaxIter; ++iter) {
        q_a = foot_flat_q_a(side, q_hp, q_k);
        double fx, fz;
        leg_fk(side, q_hp, q_k, q_a, fx, fz);
        const double err_x = target_x - fx;
        const double err_z = target_z - fz;
        if (std::abs(err_x) + std::abs(err_z) < kTol) return true;

        // Numerical Jacobian (with foot-flat enforced after each perturbation).
        double fx_h, fz_h, fx_k, fz_k;
        {
            double qa_h = foot_flat_q_a(side, q_hp + kEps, q_k);
            leg_fk(side, q_hp + kEps, q_k, qa_h, fx_h, fz_h);
        }
        {
            double qa_k = foot_flat_q_a(side, q_hp, q_k + kEps);
            leg_fk(side, q_hp, q_k + kEps, qa_k, fx_k, fz_k);
        }
        const double Jx_hp = (fx_h - fx) / kEps;
        const double Jz_hp = (fz_h - fz) / kEps;
        const double Jx_k  = (fx_k - fx) / kEps;
        const double Jz_k  = (fz_k - fz) / kEps;
        const double det = Jx_hp * Jz_k - Jx_k * Jz_hp;
        if (std::abs(det) < 1e-12) return false;

        double dq_hp = ( Jz_k  * err_x - Jx_k  * err_z) / det;
        double dq_k  = (-Jz_hp * err_x + Jx_hp * err_z) / det;

        // Damp the step so we don't leap past the workspace.
        const double mag = std::hypot(dq_hp, dq_k);
        if (mag > kMaxStep) {
            dq_hp *= kMaxStep / mag;
            dq_k  *= kMaxStep / mag;
        }
        q_hp += dq_hp;
        q_k  += dq_k;
    }
    q_a = foot_flat_q_a(side, q_hp, q_k);
    double fx, fz;
    leg_fk(side, q_hp, q_k, q_a, fx, fz);
    return std::abs(target_x - fx) + std::abs(target_z - fz) < 1e-3;
}

LegIK::LegIK(const std::array<float, 10>& ref_joint_act)
    : ref_(ref_joint_act) {
    leg_fk(0, ref_[2], ref_[3], ref_[4], foot_x_mgto_l_, foot_z_mgto_l_);
    leg_fk(1, ref_[7], ref_[8], ref_[9], foot_x_mgto_r_, foot_z_mgto_r_);
}

std::array<float, 10> LegIK::solve(double dx_foot, double dy_foot,
                                   bool* success_out) const {
    std::array<float, 10> dq{};
    bool ok = true;

    for (int side = 0; side < 2; ++side) {
        const int idx_hp = (side == 0) ? 2 : 7;
        const int idx_k  = (side == 0) ? 3 : 8;
        const int idx_a  = (side == 0) ? 4 : 9;
        const double fx_mgto = (side == 0) ? foot_x_mgto_l_ : foot_x_mgto_r_;
        const double fz_mgto = (side == 0) ? foot_z_mgto_l_ : foot_z_mgto_r_;

        double q_hp = ref_[idx_hp];
        double q_k  = ref_[idx_k];
        double q_a  = ref_[idx_a];
        const bool side_ok = sagittal_ik(side,
                                         fx_mgto + dx_foot,
                                         fz_mgto,
                                         q_hp, q_k, q_a);
        if (!side_ok) ok = false;
        dq[idx_hp] = static_cast<float>(q_hp - ref_[idx_hp]);
        dq[idx_k]  = static_cast<float>(q_k  - ref_[idx_k]);
        dq[idx_a]  = static_cast<float>(q_a  - ref_[idx_a]);
    }

    // Lateral: small-angle hip_yaw rotates the leg about base z so the foot
    // translates by ≈ q_yaw · L_leg in body y. URDF axis is (0, 0, -1), so
    // positive joint q rotates the leg toward -y of the body.
    //   To move LEFT foot OUTWARD (+y_body):  q_hy_l = -dy_foot / L_leg
    //   To move RIGHT foot OUTWARD (-y_body): q_hy_r = +dy_foot / L_leg
    constexpr double L_LEG = 0.30;  // approx hip yaw axis to foot
    const double dq_yaw = dy_foot / L_LEG;
    dq[0] = static_cast<float>(-dq_yaw);   // hip_yaw_l
    dq[5] = static_cast<float>(+dq_yaw);   // hip_yaw_r
    // hip_roll forced to MGTO (Δ = 0) so the ankle (no roll axis) keeps the
    // foot flat on the floor.
    dq[1] = 0.0f;  // hip_roll_l
    dq[6] = 0.0f;  // hip_roll_r

    if (success_out) *success_out = ok;
    return dq;
}

void LegIK::fk_foot(int side, const std::array<float, 10>& q,
                    double& foot_x, double& foot_z) const {
    const int idx_hp = (side == 0) ? 2 : 7;
    const int idx_k  = (side == 0) ? 3 : 8;
    const int idx_a  = (side == 0) ? 4 : 9;
    leg_fk(side, q[idx_hp], q[idx_k], q[idx_a], foot_x, foot_z);
}

}  // namespace qmini
