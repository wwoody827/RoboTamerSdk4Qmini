#include "user/rl_controller.h"
#include <algorithm>

void RLController::init() {
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "q1");
    Ort::SessionOptions session_options;
    motion_session = new Ort::Session(env, "policy.onnx", session_options);

    // network sees a flat 195-dim input; stacking is done here, so stack_dim = 1
    onnxInference.init(configParams.num_observations, configParams.num_actions, 1);

    jointIndex2Sim << 0, 1, 2, 3, 4, 5, 6, 7, 8, 9;
    base_rpy.setZero();
    base_vel.setZero();
    joint_pos.setZero();
    joint_vel.setZero();
    joint_tau.setZero();
    joint_acc.setZero();
    base_acc.setZero();
    base_quat << 1, 0, 0, 0;
    base_rpy_rate.setZero();
    target_command.setZero();
    projected_gravity << 0, 0, -1;
    joint_pos_error.setZero();
    action_increment.resize(onnxInference.output_dim);
    action_increment.setZero();

    for (int i = 0; i < NUM_JOINTS; ++i) {
        act_pos_low[i] = configParams.act_pos_low.at(i);
        act_pos_high[i] = configParams.act_pos_high.at(i);
        _ref_joint_act[i] = configParams.ref_joint_act.at(i);
        _kp[i] = configParams.kp.at(i);
        _kd[i] = configParams.kd.at(i);
        _kp_soft[i] = configParams.kp_soft.at(i);
        _kd_soft[i] = configParams.kd_soft.at(i);
        _residual_low[i]  = configParams.residual_low.at(i);
        _residual_high[i] = configParams.residual_high.at(i);
    }
    _action_alpha = configParams.action_lowpass_alpha;
    _lp_target.setZero();
    joint_act = _ref_joint_act;

    observation.resize(OBS_HIST * OBS_DIM_PER_STEP);
    observation.setZero();
    _obs_buffer.clear();
    for (int i = 0; i < OBS_BUFFER_LEN; ++i)
        _obs_buffer.push_back(Matrix<float, Dynamic, 1>::Zero(OBS_DIM_PER_STEP));
}

void RLController::reset(bool is_test_local) {
    target_command.setZero();
    _is_first_run = true;
    counter_rl = 0;

    // V2: lp_target and obs buffer reset to zeros at boot/mode switch
    _lp_target.setZero();
    _obs_buffer.clear();
    for (int i = 0; i < OBS_BUFFER_LEN; ++i)
        _obs_buffer.push_back(Matrix<float, Dynamic, 1>::Zero(OBS_DIM_PER_STEP));

    if (is_test_local) {
        init_joint_act = joint_act;
        joint_pos = joint_act;
    } else {
        convert_dds_state2rl_state();
        joint_act = joint_pos;
        init_joint_act = joint_pos;
        _record_yaw = base_rpy[2];
        cout << "Reset done! Rpy: " << base_rpy.transpose() << endl;
    }
    // V2: do NOT prime the buffer here. DEPLOY §5 specifies the first policy
    // step sees buffer = 8 zeros + 1 real frame — that happens naturally on
    // the first rl_control() call.
}


void RLController::rl_control() {
    counter_rl++;
    Matrix<float, Dynamic, 1> net_out;
    net_out = onnxInference.inference(motion_session, get_observation());
    apply_residual_action(net_out);
    _rl_time_step = get_true_loop_period();
}

// V2 residual action mode (DEPLOY.md §3):
//   offset    = clip(net_out, -1, 1) * 0.5            // residual range ±0.5 rad
//   lp_target = alpha * offset + (1 - alpha) * lp_target_prev
//   joint_act = ref_joint_pos + lp_target             // clipped to URDF limits
// _lp_target is initialized to zeros at boot/reset, not to ref_joint_pos.
void RLController::apply_residual_action(const Matrix<float, Dynamic, 1> &net_out) {
    Vec10<float> clipped = net_out.head(NUM_ACTUAT_JOINTS).cwiseMax(-1.f).cwiseMin(1.f);
    // action_increment is preserved as the raw post-clip net output for logging
    // (data_report.h reads it). Old BIRL semantics (joint position increment)
    // no longer apply — it's just the policy's pre-residual-scaling action.
    action_increment.head(NUM_ACTUAT_JOINTS) = clipped;
    Vec10<float> offset;
    for (int i = 0; i < NUM_ACTUAT_JOINTS; ++i) {
        float scale = 0.5f * (_residual_high[i] - _residual_low[i]);
        float bias  = 0.5f * (_residual_high[i] + _residual_low[i]);
        offset[i] = clipped[i] * scale + bias;
    }
    _lp_target = _action_alpha * offset + (1.f - _action_alpha) * _lp_target;
    joint_act = _ref_joint_act + _lp_target;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}


Matrix<float, Dynamic, 1> RLController::get_observation() {
    Matrix<float, Dynamic, 1> obs(OBS_DIM_PER_STEP);
    obs.setZero();

    pthread_mutex_lock(&_rl_state_mutex);

    // Commands: V2 stand task always sees zeros. joystick_command_process still
    // runs so debug telemetry stays live, but its output is discarded for obs.
    joystick_command_process();
    Vec3<float> obs_command = Vec3<float>::Zero();

    // projected_gravity = R_world_to_body * [0, 0, -1] (body-frame gravity).
    // ori::rpy_to_rotMat returns R_world_to_body, so convert_world_frame_to_base_frame
    // is exactly this transformation.
    Vec3<float> gravity_world(0.f, 0.f, -1.f);
    projected_gravity = convert_world_frame_to_base_frame(gravity_world, base_rpy);

    // tracking_err = current_joint_act (commanded target) - joint_pos (measured)
    joint_pos_error = joint_act - joint_pos;

    obs << obs_command,                                     // 3
           base_rpy_rate * 0.5f,                            // 3
           projected_gravity,                               // 3
           (joint_pos - _ref_joint_act),                    // 10
           joint_vel * 0.1f,                                // 10
           joint_pos_error;                                 // 10
    obs = obs.cwiseMax(-3.f).cwiseMin(3.f);

    pthread_mutex_unlock(&_rl_state_mutex);

    // Rolling 9-frame buffer (FIFO). On the very first call after reset() the
    // buffer is already zero-filled, so the first ~9 policy steps include
    // partial zero history — intentional and matches training (DEPLOY §5).
    _obs_buffer.pop_front();
    _obs_buffer.push_back(obs);
    if (_is_first_run) {
        _is_first_run = false;
        cout << endl << "Reset observation history: Done!" << endl;
    }

    // Sample frames at strides of OBS_SKIP, oldest → newest: [0,2,4,6,8].
    for (int i = 0; i < OBS_HIST; ++i) {
        observation.segment(i * OBS_DIM_PER_STEP, OBS_DIM_PER_STEP) =
                _obs_buffer[i * OBS_SKIP];
    }
    return observation;
}


void RLController::joystick_command_process() {
    float vx_cmd = 0, vy_cmd = 0, yr_cmd = 0;
    auto yr_max = configParams.yr_cmd_range.at(1);
    auto vx_min = configParams.vx_cmd_range.at(0);
    auto vx_max = configParams.vx_cmd_range.at(1);
    if (task_mode == 3 or task_mode == 4) {
        ///stand
        vx_cmd = -vx_max * jsreader->Axis[1];  // left stick Y → forward/back
        vy_cmd = -vx_max * jsreader->Axis[0];  // left stick X → left/right
        yr_cmd = -yr_max * jsreader->Axis[2];  // right stick X → yaw

        if (fabs(yr_cmd) > 0.1 or configParams.kp_yaw_ctrl < 1e-2) {
            _record_yaw = base_rpy[2];
        } else {
            yr_cmd = configParams.kp_yaw_ctrl * smallest_signed_angle_between(base_rpy[2], _record_yaw);
        }

        yr_cmd = std::clamp(yr_cmd, -yr_max, yr_max);
        vx_cmd = std::clamp(vx_cmd, vx_min, vx_max);
        vy_cmd = std::clamp(vy_cmd, vx_min, vx_max);
    }
    target_command << vx_cmd, vy_cmd, yr_cmd;
}

void RLController::set_rl_joint_act2dds_motor_command(char mode) {
    MotorCommand motor_command_tmp;
    for (int i = 0; i < NUM_JOINTS; ++i) {
        motor_command_tmp.q_target[i] = joint_act[jointIndex2Sim[i]];
        if (mode=='q') {
            motor_command_tmp.kp[i] = 0.;
            motor_command_tmp.kd[i] = 0.;
        } else if (mode=='1') {
            motor_command_tmp.kp[i] = _kp_soft[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd_soft[jointIndex2Sim[i]];
        }
        else {
            motor_command_tmp.kp[i] = _kp[jointIndex2Sim[i]];
            motor_command_tmp.kd[i] = _kd[jointIndex2Sim[i]];
        }
        motor_command_tmp.tau_ff[i] = 0.;
        motor_command_tmp.dq_target[i] = 0.;
    }
    dds_motor_command->SetData(motor_command_tmp);
}

void RLController::convert_dds_state2rl_state() {
    Vec3<float> trans_axis(-1., 1, -1);
    if (dds_motor_state->GetData()) {
        for (int i = 0; i < NUM_JOINTS; ++i) {
            joint_pos[jointIndex2Sim[i]] = exp_filter(joint_pos[jointIndex2Sim[i]], dds_motor_state->GetData()->q[i], 0.2);
            joint_vel[jointIndex2Sim[i]] = exp_filter(joint_vel[jointIndex2Sim[i]], dds_motor_state->GetData()->dq[i], 0.1);
            joint_tau[jointIndex2Sim[i]] = dds_motor_state->GetData()->tau_est[i];
            joint_acc[jointIndex2Sim[i]] = dds_motor_state->GetData()->ddq[i];
        }
    }
    if (dds_base_state->GetData()) {
        for (int i(0); i < 3; i++) {
            base_rpy(i) = exp_filter(base_rpy(i), fmod(dds_base_state->GetData()->rpy.at(i) * trans_axis(i), 2 * M_PI), 0.2);
            base_rpy_rate(i) = exp_filter(base_rpy_rate(i), dds_base_state->GetData()->omega.at(i) * trans_axis(i), 0.1);
            base_acc(i) = exp_filter(base_acc(i), dds_base_state->GetData()->acc.at(i) * trans_axis(i), 0.1);
        }
    }
    counter_print++;
}


void RLController::smooth_joint_action(float ratio, const Vec10<float> &end_joint_act) {
    joint_act = (1 - ratio) * init_joint_act + ratio * end_joint_act;
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

float RLController::exp_filter(float history, float present, float weight) {
    auto result = history * weight + present * (1. - weight);
    return result;
}


void RLController::sin_control(float amplitude, float f, float motion_time) {
    Vec10<float> sin_joint_act;
    sin_joint_act.setConstant(amplitude * sin(2.f * M_PI * f * motion_time));
    if (configParams.sin_joint_idx == -1)
        joint_act.segment(0, NUM_ACTUAT_JOINTS) = init_joint_act.segment(0, NUM_ACTUAT_JOINTS) + sin_joint_act;
    else
        joint_act[configParams.sin_joint_idx] = init_joint_act[configParams.sin_joint_idx] + sin_joint_act[configParams.sin_joint_idx];
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}


float RLController::get_true_loop_period() {
    static struct timeval last_time;
    static struct timeval now_time;
    static bool first_get_time = true;
    if (first_get_time) {
        gettimeofday(&last_time, nullptr);
        first_get_time = false;
    }
    gettimeofday(&now_time, nullptr);
    auto d_time = (float) (now_time.tv_sec - last_time.tv_sec) +
                  (float) (now_time.tv_usec - last_time.tv_usec) / 1000000;
    last_time = now_time;
    if (fabs(d_time - _rl_time_step) * 1000. > 2.)
        cout << "True period: " << d_time * 1000. << " ms" << endl;
    return d_time;
}

void RLController::stand_control(float ratio) {
    smooth_joint_action(ratio, _ref_joint_act);
}

void RLController::sim_gait_control() {
    static int data_index = 0;
    if (data_index <= sim_gait_data.size() - 2)
        data_index++;
    for (int i(0); i < NUM_ACTUAT_JOINTS; i++) {
        joint_act(i) = sim_gait_data.at(data_index).at(i);
    }
    joint_act = joint_act.cwiseMax(act_pos_low).cwiseMin(act_pos_high);
}

Vec3<float> RLController::convert_world_frame_to_base_frame(const Vec3<float> &world_vec, const Vec3<float> &rpy) {
    return ori::rpy_to_rotMat(rpy) * world_vec;
}

Vec3<float> RLController::quat_rotate_inverse(Vec4<float> q, Vec3<float> v) {
    Vec3<float> a, b, c;
    // q={w,x,y,z}
    // q << 0.99981624, -0.013256095, 0.012793032, -0.005295367; //todo: in isaac gym: x,y,z,w ;here IMU(q): w,x,y,z !!
    // v << -0.13947208, -0.08728597, 0.19939381;
    // result: lin_v(-0.14353749,-0.09399233,0.19336908)
    float q_w = q[0];
    Vec3<float> q_vec(q[1], q[2], q[3]);
    a = v * (2.0 * q_w * q_w - 1.0);
    b = q_vec.cross(v) * q_w * 2.0;
    c = q_vec * q_vec.transpose() * v * 2.0;
    // cout << "quat_rotate_inverse: " << (a - b + c).transpose() << endl << endl;
    return a - b + c;
}

/*!
 * Take the product of two quaternions
 */
Vec4<float> RLController::quat_product(Vec4<float> &q1, Vec4<float> &q2) {
    float r1 = q1[0];
    float r2 = q2[0];

    Vec3<float> v1(q1[1], q1[2], q1[3]);
    Vec3<float> v2(q2[1], q2[2], q2[3]);

    float r = r1 * r2 - v1.dot(v2);
    Vec3<float> v = r1 * v2 + r2 * v1 + v1.cross(v2);
    Vec4<float> q(r, v[0], v[1], v[2]);
    return q;
}

float RLController::smallest_signed_angle_between(float alpha, float beta) {
    auto a = beta - alpha;
    a += (a > M_PI) ? -2 * M_PI : (a < -M_PI) ? 2 * M_PI : 0.;
    return a;
}

Vec4<float> RLController::rpy_to_quat(const Vec3<float> &rpy) {
    auto R = ori::rpy_to_rotMat(rpy);
    auto q = ori::rotMat_to_quat(R);
    return q;
}

Vec4<float> RLController::quat_mul(Vec4<float> a, Vec4<float> b) {
    float x1 = a[1];
    float y1 = a[2];
    float z1 = a[3];
    float w1 = a[0];

    float x2 = b[1];
    float y2 = b[2];
    float z2 = b[3];
    float w2 = b[0];

    float ww = (z1 + x1) * (x2 + y2);
    float yy = (w1 - y1) * (w2 + z2);
    float zz = (w1 + y1) * (w2 - z2);
    float xx = ww + yy + zz;
    float qq = 0.5 * (xx + (z1 - x1) * (x2 - y2));

    float w = qq - ww + (z1 - y1) * (y2 - z2);
    float x = qq - xx + (x1 + w1) * (x2 + w2);
    float y = qq - yy + (w1 - x1) * (y2 + z2);
    float z = qq - zz + (z1 + y1) * (w2 - x2);

    Vec4<float> quat(w, x, y, z);

    return quat;
}