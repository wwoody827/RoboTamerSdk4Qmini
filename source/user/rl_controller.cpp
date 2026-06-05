#include "user/rl_controller.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace qmini {

RLController::RLController(ConfigParams cfg, std::unique_ptr<IPolicy> policy)
    : cfg_(std::move(cfg)), policy_(std::move(policy)) {
    if (!policy_) {
        throw std::runtime_error("RLController: policy must not be null");
    }
    if (policy_->input_dim() != kObsPerStep) {
        throw std::runtime_error(
            "RLController: policy input_dim must equal kObsPerStep (39)");
    }
    if (policy_->output_dim() != kNumActions) {
        throw std::runtime_error(
            "RLController: policy output_dim must equal kNumActions (10)");
    }
    if (cfg_.num_observations != kObsPerStep) {
        throw std::runtime_error(
            "RLController: config num_observations must equal 39");
    }
    if (cfg_.num_actions != kNumActions) {
        throw std::runtime_error(
            "RLController: config num_actions must equal 10");
    }
}

void RLController::init() {
    joint_pos_.setZero();
    joint_vel_.setZero();
    joint_tau_.setZero();
    joint_acc_.setZero();
    base_acc_.setZero();
    base_quat_ << 1, 0, 0, 0;
    base_rpy_.setZero();
    base_rpy_rate_.setZero();
    target_command_.setZero();
    last_raw_action_.setZero();

    for (int i = 0; i < kNumActuatedJoints; ++i) {
        act_pos_low_[i]  = cfg_.act_pos_low.at(i);
        act_pos_high_[i] = cfg_.act_pos_high.at(i);
        ref_joint_act_[i] = cfg_.ref_joint_act.at(i);
        action_scale_[i] = cfg_.action_scale.at(i);
        kp_[i]      = cfg_.kp.at(i);
        kd_[i]      = cfg_.kd.at(i);
        kp_soft_[i] = cfg_.kp_soft.at(i);
        kd_soft_[i] = cfg_.kd_soft.at(i);
    }
    joint_act_ = ref_joint_act_;

    const int stack = policy_->stack_dim();
    observation_ = Eigen::VectorXf::Zero(kObsPerStep * stack);
    obs_stack_.assign(stack, Eigen::Matrix<float, kObsPerStep, 1>::Zero());
}

void RLController::reset(bool reset_pose_to_measured) {
    std::lock_guard<std::mutex> g(state_mutex_);
    target_command_.setZero();
    last_raw_action_.setZero();
    is_first_run_ = true;
    counter_rl_ = 0;
    if (reset_pose_to_measured) {
        joint_act_ = joint_pos_;
        init_joint_act_ = joint_pos_;
        record_yaw_ = base_rpy_[2];
    } else {
        init_joint_act_ = joint_act_;
        joint_pos_ = joint_act_;
    }
}

void RLController::update_motor_state(const hal::MotorStateFrame& s) {
    std::lock_guard<std::mutex> g(state_mutex_);
    for (int i = 0; i < kNumActuatedJoints; ++i) {
        joint_pos_[i] = exp_filter(joint_pos_[i], s.q[i], 0.2f);
        joint_vel_[i] = exp_filter(joint_vel_[i], s.dq[i], 0.1f);
        joint_tau_[i] = s.tau_est[i];
        joint_acc_[i] = s.ddq[i];
    }
}

void RLController::update_base_state(const hal::BaseStateFrame& s) {
    if (!s.valid) return;
    // HAL backends now emit the canonical robot/training body frame:
    //   - sim (MuJoCo world.cpp): already in body frame
    //   - real (imu_serial.cpp): the IMU mount transform is folded in there
    // so the controller applies NO further sign flips. base_rpy_rate_ holds
    // the body-frame gyro omega; base_acc_ is telemetry only (the policy uses
    // projected_gravity derived from roll/pitch, not acc).
    std::lock_guard<std::mutex> g(state_mutex_);
    for (int i = 0; i < 3; ++i) {
        base_rpy_[i] = exp_filter(base_rpy_[i],
            std::fmod(s.rpy[i], 2 * M_PI), 0.2f);
        base_rpy_rate_[i] = exp_filter(base_rpy_rate_[i], s.omega[i], 0.1f);
        base_acc_[i] = exp_filter(base_acc_[i], s.acc[i], 0.1f);
    }
    for (int i = 0; i < 4; ++i) base_quat_[i] = s.quat[i];
}

void RLController::update_joystick(const hal::JoystickFrame& js) {
    last_joystick_ = js;
}

void RLController::joystick_command_process(const hal::JoystickFrame& js) {
    // Yaw-hold gating: only hold heading while the robot is meant to be
    // moving (matches the old static-threshold behavior).
    static constexpr float kStaticThreshold = 0.15f;
    static_flag_ = target_command_.norm() >= kStaticThreshold ? 1.f : 0.f;

    float vx_cmd = 0, vy_cmd = 0, yr_cmd = 0;
    const float yr_max = cfg_.yr_cmd_range.at(1);
    const float vx_min = cfg_.vx_cmd_range.at(0);
    const float vx_max = cfg_.vx_cmd_range.at(1);

    if (task_mode_ == 3 || task_mode_ == 4) {
        vx_cmd = -vx_max * js.axis[1];
        vy_cmd = -vx_max * js.axis[0];
        yr_cmd = -yr_max * js.axis[2];

        if (std::fabs(yr_cmd) > 0.1f || cfg_.kp_yaw_ctrl < 1e-2f
            || static_flag_ < 0.1f) {
            record_yaw_ = base_rpy_[2];
        } else {
            yr_cmd = cfg_.kp_yaw_ctrl *
                     smallest_signed_angle_between(base_rpy_[2], record_yaw_);
        }
        yr_cmd = std::clamp(yr_cmd, -yr_max, yr_max);
        vx_cmd = std::clamp(vx_cmd, vx_min, vx_max);
        vy_cmd = std::clamp(vy_cmd, vx_min, vx_max);
    }
    target_command_ << vx_cmd, vy_cmd, yr_cmd;
}

Eigen::VectorXf RLController::build_stacked_obs() {
    ObsInputs in;
    {
        std::lock_guard<std::mutex> g(state_mutex_);
        joystick_command_process(last_joystick_);
        in.base_ang_vel      = base_rpy_rate_;   // body-frame gyro
        in.projected_gravity =
            projected_gravity_from_rpy(base_rpy_[0], base_rpy_[1]);
        in.joint_pos      = joint_pos_;
        in.ref_joint_act  = ref_joint_act_;
        in.joint_vel      = joint_vel_;
        in.last_action    = last_raw_action_;
        in.command        = target_command_;
    }

    Eigen::Matrix<float, kObsPerStep, 1> frame = build_obs(in);

    const int stack = policy_->stack_dim();
    if (is_first_run_) {
        obs_stack_.assign(stack, frame);   // pre-fill history with first frame
        is_first_run_ = false;
    } else {
        obs_stack_.erase(obs_stack_.begin());
        obs_stack_.push_back(frame);
    }
    observation_ = stack_obs_per_term(obs_stack_);
    return observation_;
}

void RLController::apply_raw_action(const Eigen::VectorXf& raw) {
    // Absolute joint-position action: target = raw * scale + offset.
    // Offset is the default/ref pose; raw is fed back into the next obs frame
    // BEFORE scaling (matches training's 'actions' term).
    std::lock_guard<std::mutex> g(state_mutex_);
    for (int i = 0; i < kNumActuatedJoints; ++i) {
        last_raw_action_[i] = raw[i];
        joint_act_[i] = raw[i] * action_scale_[i] + ref_joint_act_[i];
    }
    // Training applies no clip, but we clamp to the configured joint range as
    // a hardware safety net (a healthy policy stays well inside it).
    joint_act_ = joint_act_.cwiseMax(act_pos_low_).cwiseMin(act_pos_high_);
}

void RLController::rl_control() {
    counter_rl_++;
    Eigen::VectorXf obs = build_stacked_obs();
    Eigen::VectorXf raw = policy_->infer(obs);
    apply_raw_action(raw);
}

void RLController::smooth_joint_action(float ratio,
                                       const Vec10<float>& end) {
    std::lock_guard<std::mutex> g(state_mutex_);
    joint_act_ = (1.f - ratio) * init_joint_act_ + ratio * end;
    joint_act_ = joint_act_.cwiseMax(act_pos_low_).cwiseMin(act_pos_high_);
}

void RLController::stand_control(float ratio) {
    smooth_joint_action(ratio, ref_joint_act_);
}

void RLController::zero_pose_control(float ratio) {
    // Same ramp as stand_control but the target is the URDF natural pose
    // (q = 0 for every joint). After 'z' captures the operator-held pose
    // as the new zero, mode '0' verifies that capture: PD-holds at zero
    // and the robot should physically stay where it was at the moment of
    // capture.
    Vec10<float> zero = Vec10<float>::Zero();
    smooth_joint_action(ratio, zero);
}

void RLController::sin_control(float amplitude, float f, float t) {
    // Single-joint sin test only — the "wiggle all 10 joints in sync"
    // mode was removed because it's unsafe on the real robot (every
    // joint commanded at the same offset, regardless of pose, can
    // easily exceed limits or topple the robot).
    std::lock_guard<std::mutex> g(state_mutex_);
    const int j = cfg_.sin_joint_idx;
    if (j < 0 || j >= kNumActuatedJoints) return;  // safety no-op
    const float off = amplitude * std::sin(2.f * static_cast<float>(M_PI) * f * t);
    joint_act_[j] = init_joint_act_[j] + off;
    joint_act_ = joint_act_.cwiseMax(act_pos_low_).cwiseMin(act_pos_high_);
}

hal::MotorCmdFrame RLController::to_motor_cmd(char mode) const {
    hal::MotorCmdFrame f;
    std::lock_guard<std::mutex> g(state_mutex_);
    for (int i = 0; i < kNumActuatedJoints; ++i) {
        f.q_target[i] = joint_act_[i];
        f.dq_target[i] = 0.f;
        f.tau_ff[i]    = 0.f;
        if (mode == 'q') {
            f.kp[i] = 0.f;
            f.kd[i] = 0.f;
        } else if (mode == '1') {
            f.kp[i] = kp_soft_[i];
            f.kd[i] = kd_soft_[i];
        } else if (mode == '2') {
            f.kp[i] = kp_[i] * stand_kp_scale_;
            f.kd[i] = kd_[i] * stand_kd_scale_;
        } else {
            f.kp[i] = kp_[i];
            f.kd[i] = kd_[i];
        }
    }
    return f;
}

float RLController::smallest_signed_angle_between(float alpha, float beta) {
    float a = beta - alpha;
    if (a > M_PI) a -= 2 * M_PI;
    else if (a < -M_PI) a += 2 * M_PI;
    return a;
}

float RLController::exp_filter(float history, float present, float weight) {
    return history * weight + present * (1.f - weight);
}

}  // namespace qmini
