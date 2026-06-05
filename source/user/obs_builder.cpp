#include "user/obs_builder.h"

#include <cmath>

namespace qmini {

Vec3<float> projected_gravity_from_rpy(float roll, float pitch) {
    const float cp = std::cos(pitch), sp = std::sin(pitch);
    const float cr = std::cos(roll),  sr = std::sin(roll);
    Vec3<float> g;
    g << sp, -sr * cp, -cr * cp;   // upright (0,0) -> [0, 0, -1]
    return g;
}

Eigen::Matrix<float, kObsPerStep, 1> build_obs(const ObsInputs& in) {
    Eigen::Matrix<float, kObsPerStep, 1> obs;
    obs << in.base_ang_vel,
           in.projected_gravity,
           (in.joint_pos - in.ref_joint_act),
           in.joint_vel,
           in.last_action,
           in.command;
    return obs;
}

Eigen::VectorXf stack_obs_per_term(
    const std::vector<Eigen::Matrix<float, kObsPerStep, 1>>& frames) {
    const int stack = static_cast<int>(frames.size());
    Eigen::VectorXf out(kObsPerStep * stack);
    int w = 0;
    int term_start = 0;
    for (int t = 0; t < kObsNumTerms; ++t) {
        const int d = kObsTermDims[t];
        for (int f = 0; f < stack; ++f) {            // oldest -> newest
            for (int k = 0; k < d; ++k) {
                out[w++] = frames[f][term_start + k];
            }
        }
        term_start += d;
    }
    return out;
}

}  // namespace qmini
