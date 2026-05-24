#include "user/obs_builder.h"

#include <cmath>

namespace qmini {

Eigen::Matrix<float, kObsPerStep, 1> build_obs(
    const ObsInputs& in, const ObsParams& p) {
    Eigen::Matrix<float, kObsPerStep, 1> obs;

    const float static_flag = compute_static_flag(in.target_command,
                                                  p.static_threshold);

    Vec4<float> phase_sin_cos;
    phase_sin_cos(0) = std::sin(in.pm_phase[0]);
    phase_sin_cos(1) = std::sin(in.pm_phase[1]);
    phase_sin_cos(2) = std::cos(in.pm_phase[0]);
    phase_sin_cos(3) = std::cos(in.pm_phase[1]);

    const Vec10<float> joint_pos_err = in.joint_act - in.joint_pos;
    const Vec10<float> joint_pos_devref = in.joint_pos - in.ref_joint_act;
    const Vec2<float> freq_term = (in.pm_f * p.freq_scale).array() - 1.0f;

    obs << in.target_command,
           in.base_rpy.segment(0, 2),
           in.base_rpy_rate * p.rpy_rate_scale,
           joint_pos_devref,
           in.joint_vel * p.joint_vel_scale,
           joint_pos_err,
           phase_sin_cos * static_flag,
           freq_term * static_flag;

    obs = obs.cwiseMax(-p.clamp_abs).cwiseMin(p.clamp_abs);
    return obs;
}

}  // namespace qmini
