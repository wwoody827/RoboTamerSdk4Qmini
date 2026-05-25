#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "user/hal/types.h"
#include "user/obs_builder.h"
#include "user/policy.h"
#include "utils/config.h"
#include "utils/cpp_types.h"

namespace qmini {

// Above-HAL control logic. Pulls state from HAL frames, builds obs, runs
// inference, integrates joint targets, emits a MotorCmdFrame.
//
// Compared to the pre-HAL version:
//   - No DDS / unitree-SDK types in the interface
//   - No embedded Python (joystick goes through hal::JoystickFrame)
//   - Pure observation builder extracted (see obs_builder.h)
//   - std::mutex instead of uninitialized pthread_mutex_t
//   - No ownership of foreign pointers; destructor is trivial
//   - Inference behind IPolicy so tests can run without onnxruntime
class RLController {
public:
    static constexpr int kNumLegs = 2;
    static constexpr int kNumActuatedJoints = 10;

    explicit RLController(ConfigParams cfg, std::unique_ptr<IPolicy> policy);
    ~RLController() = default;

    RLController(const RLController&) = delete;
    RLController& operator=(const RLController&) = delete;

    void init();
    void reset(bool reset_pose_to_measured);

    // State ingestion (from HAL).
    void update_motor_state(const hal::MotorStateFrame& s);
    void update_base_state(const hal::BaseStateFrame& s);
    void update_joystick(const hal::JoystickFrame& js);

    // Per-tick control entries.
    void rl_control();
    void stand_control(float ratio);
    void sin_control(float amplitude, float f, float motion_time);

    // Output: build a motor command frame from current joint_act + mode.
    hal::MotorCmdFrame to_motor_cmd(char mode) const;

    // Tunables / inputs visible to outer scope.
    void set_task_mode(int m) { task_mode_ = m; }
    int  task_mode() const     { return task_mode_; }
    // Scale applied to kp / kd when the controller emits a "stand" frame
    // (mode '2'). 1.0 = use config.yaml values as-is. >1 useful when
    // observing in sim where the deployed-policy-sized gains can't drive
    // the joints against gravity in a visible way.
    void set_stand_kp_scale(float s) { stand_kp_scale_ = s; }
    void set_stand_kd_scale(float s) { stand_kd_scale_ = s; }
    // Override which joint mode '5' (sin test) wiggles.
    // Valid: 0..9 = single joint by index (HYL HRL HPL KL AL HYR HRR HPR KR AR).
    // Anything else → sin_control no-ops as a safety. The historical
    // "-1 = all joints" mode was removed; commanding every joint in sync
    // is unsafe on the real robot (limits / topple).
    void set_sin_joint_idx(int j) { cfg_.sin_joint_idx = j; }

    // Public telemetry (read-only views — held with the controller's lifetime).
    const Vec10<float>& joint_pos()       const { return joint_pos_; }
    const Vec10<float>& joint_vel()       const { return joint_vel_; }
    const Vec10<float>& joint_tau()       const { return joint_tau_; }
    const Vec10<float>& joint_act()       const { return joint_act_; }
    const Vec10<float>& ref_joint_act()   const { return ref_joint_act_; }
    const Vec3<float>&  base_rpy()        const { return base_rpy_; }
    const Vec3<float>&  base_rpy_rate()   const { return base_rpy_rate_; }
    const Vec3<float>&  base_acc()        const { return base_acc_; }
    const Vec4<float>&  base_quat()       const { return base_quat_; }
    const Vec3<float>&  target_command()  const { return target_command_; }
    const Vec2<float>&  pm_f()            const { return pm_f_; }
    const Vec2<float>&  pm_phase()        const { return pm_phase_; }
    float               static_flag()     const { return static_flag_; }
    int                 counter_rl()      const { return counter_rl_; }
    const Eigen::VectorXf& observation()      const { return observation_; }
    const Eigen::VectorXf& action_increment() const { return action_increment_; }

    // Math helpers (kept as static for testability).
    static float smallest_signed_angle_between(float alpha, float beta);
    static float exp_filter(float history, float present, float weight);

private:
    void joystick_command_process(const hal::JoystickFrame& js);
    void compute_pm_phase(const Vec2<float>& f);
    Eigen::VectorXf build_stacked_obs();
    Eigen::VectorXf transform(const Eigen::VectorXf& net_out);
    void joint_increment_control(const Eigen::VectorXf& increment);
    void smooth_joint_action(float ratio, const Vec10<float>& end_joint_act);

    ConfigParams cfg_;
    std::unique_ptr<IPolicy> policy_;
    ObsParams obs_params_;

    mutable std::mutex state_mutex_;
    int task_mode_ = 0;
    int counter_rl_ = 0;
    bool is_first_run_ = true;
    float rl_time_step_ = 0.01f;
    float record_yaw_ = 0.f;
    float static_flag_ = 0.f;
    float stand_kp_scale_ = 1.f;
    float stand_kd_scale_ = 1.f;

    Vec10<float> joint_pos_, joint_vel_, joint_tau_, joint_acc_;
    Vec10<float> joint_act_, init_joint_act_;
    Vec3<float>  base_rpy_, base_rpy_rate_, base_acc_;
    Vec4<float>  base_quat_;
    Vec3<float>  target_command_;
    Vec2<float>  pm_f_, pm_phase_;

    Vec10<float> ref_joint_act_, act_pos_low_, act_pos_high_;
    Vec10<float> kp_, kd_, kp_soft_, kd_soft_;

    Eigen::VectorXf action_increment_;
    Eigen::VectorXf observation_;
    std::vector<Eigen::VectorXf> obs_stack_;

    hal::JoystickFrame last_joystick_{};
};

}  // namespace qmini
