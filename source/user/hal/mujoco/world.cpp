#include "world.h"

#include <cmath>
#include <cstring>
#include <iostream>

#include <mujoco/mujoco.h>

namespace qmini {
namespace hal {
namespace mj {

namespace {

const char* kJointNames[kNumJoints] = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
};

}  // namespace

World& World::instance() {
    static World w;
    return w;
}

World::~World() {
    if (data_)  mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
}

bool World::load(const std::string& mjcf_path) {
    std::lock_guard<std::mutex> g(mu_);
    if (model_) return true;  // idempotent
    char err[1024] = {};
    model_ = mj_loadXML(mjcf_path.c_str(), nullptr, err, sizeof(err));
    if (!model_) {
        std::cerr << "mujoco: failed to load " << mjcf_path << ": " << err
                  << "\n";
        return false;
    }
    data_ = mj_makeData(model_);
    mj_forward(model_, data_);

    // Locate joint qpos/qvel addresses + actuator ids.
    for (int i = 0; i < kNumJoints; ++i) {
        int jid = mj_name2id(model_, mjOBJ_JOINT, kJointNames[i]);
        if (jid < 0) {
            std::cerr << "mujoco: joint not found: " << kJointNames[i] << "\n";
            return false;
        }
        joint_qpos_addr_[i] = model_->jnt_qposadr[jid];
        joint_qvel_addr_[i] = model_->jnt_dofadr[jid];
        std::string aname = std::string("act_") + kJointNames[i];
        int aid = mj_name2id(model_, mjOBJ_ACTUATOR, aname.c_str());
        if (aid < 0) {
            std::cerr << "mujoco: actuator not found: " << aname << "\n";
            return false;
        }
        actuator_id_[i] = aid;
    }
    // Free joint: by convention added at root, type mjJNT_FREE.
    for (int j = 0; j < model_->njnt; ++j) {
        if (model_->jnt_type[j] == mjJNT_FREE) {
            free_joint_qpos_addr_ = model_->jnt_qposadr[j];
            free_joint_qvel_addr_ = model_->jnt_dofadr[j];
            break;
        }
    }
    if (free_joint_qpos_addr_ < 0) {
        std::cerr << "mujoco: no free joint found — robot will be fixed-base\n";
    }
    std::cerr << "mujoco: loaded " << mjcf_path
              << " (nq=" << model_->nq << " nv=" << model_->nv
              << " nu=" << model_->nu << ")\n";
    return true;
}

double World::mj_dt() const { return model_ ? model_->opt.timestep : 0.002; }

bool World::take_render_snapshot(std::vector<double>& qpos,
                                 std::vector<double>& qvel,
                                 double& time) {
    std::lock_guard<std::mutex> g(mu_);
    if (!model_ || !data_) return false;
    qpos.assign(data_->qpos, data_->qpos + model_->nq);
    qvel.assign(data_->qvel, data_->qvel + model_->nv);
    time = data_->time;
    return true;
}

void World::step_with_cmd(const MotorCmdFrame& cmd, int substeps) {
    std::lock_guard<std::mutex> g(mu_);
    if (!model_ || !data_) return;
    for (int s = 0; s < substeps; ++s) {
        // PD in C++ — actuator gain is 1.0, so ctrl is the torque.
        for (int i = 0; i < kNumJoints; ++i) {
            const double q  = data_->qpos[joint_qpos_addr_[i]];
            const double dq = data_->qvel[joint_qvel_addr_[i]];
            double tau = cmd.kp[i] * (cmd.q_target[i] - q)
                       - cmd.kd[i] * dq
                       + cmd.tau_ff[i];
            // Clamp to actuator forcerange (set to ±30 in the MJCF).
            if (tau >  30.0) tau =  30.0;
            if (tau < -30.0) tau = -30.0;
            data_->ctrl[actuator_id_[i]] = tau;
        }
        mj_step(model_, data_);
    }
}

MotorStateFrame World::read_motor_state() {
    MotorStateFrame s;
    std::lock_guard<std::mutex> g(mu_);
    if (!model_ || !data_) return s;
    for (int i = 0; i < kNumJoints; ++i) {
        s.q[i]       = static_cast<float>(data_->qpos[joint_qpos_addr_[i]]);
        s.dq[i]      = static_cast<float>(data_->qvel[joint_qvel_addr_[i]]);
        s.tau_est[i] = static_cast<float>(data_->ctrl[actuator_id_[i]]);
        s.ddq[i]     = 0.f;
    }
    return s;
}

BaseStateFrame World::read_base_state() {
    BaseStateFrame s;
    std::lock_guard<std::mutex> g(mu_);
    if (!model_ || !data_) return s;
    if (free_joint_qpos_addr_ < 0) {
        // Fixed base — return identity / valid so the loop can still run.
        s.valid = true;
        return s;
    }
    // qpos layout for a free joint: [x, y, z, qw, qx, qy, qz]
    const double* qp = data_->qpos + free_joint_qpos_addr_;
    const double  qw = qp[3], qx = qp[4], qy = qp[5], qz = qp[6];
    // qvel layout for a free joint: [vx, vy, vz, wx, wy, wz] in WORLD frame.
    // The IMU expects angular velocity in BASE frame, so we rotate.
    const double* qv = data_->qvel + free_joint_qvel_addr_;
    const double  wx_w = qv[3], wy_w = qv[4], wz_w = qv[5];

    // Quaternion → RPY (mujoco quat is wxyz).
    const double sinr = 2.0 * (qw * qx + qy * qz);
    const double cosr = 1.0 - 2.0 * (qx * qx + qy * qy);
    const double roll = std::atan2(sinr, cosr);
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (sinp > 1.0) sinp = 1.0;
    if (sinp < -1.0) sinp = -1.0;
    const double pitch = std::asin(sinp);
    const double siny = 2.0 * (qw * qz + qx * qy);
    const double cosy = 1.0 - 2.0 * (qy * qy + qz * qz);
    const double yaw  = std::atan2(siny, cosy);

    // Rotate world angular velocity into base frame: w_b = R^T * w_w.
    // R from quaternion (wxyz). Inline 3x3 to avoid pulling Eigen in here.
    auto rotate_inv = [&](double x, double y, double z,
                          double& bx, double& by, double& bz) {
        // R^T = R(-q). For unit quat (w,x,y,z), R^T*v computed directly:
        const double a = 2.0 * qw;
        const double b = 2.0 * (qx * x + qy * y + qz * z);
        bx = (a * qw - 1.0) * x + a * (qy * z - qz * y) + b * qx;
        // Above isn't quite right — use the standard formula:
        // v_b = v - 2 * w * (qxyz × v) + 2 * (qxyz × (qxyz × v))
        // For simplicity, fall back to the explicit Rt:
        const double Rt00 = 1 - 2*(qy*qy + qz*qz);
        const double Rt01 = 2*(qx*qy + qz*qw);
        const double Rt02 = 2*(qx*qz - qy*qw);
        const double Rt10 = 2*(qx*qy - qz*qw);
        const double Rt11 = 1 - 2*(qx*qx + qz*qz);
        const double Rt12 = 2*(qy*qz + qx*qw);
        const double Rt20 = 2*(qx*qz + qy*qw);
        const double Rt21 = 2*(qy*qz - qx*qw);
        const double Rt22 = 1 - 2*(qx*qx + qy*qy);
        bx = Rt00 * x + Rt01 * y + Rt02 * z;
        by = Rt10 * x + Rt11 * y + Rt12 * z;
        bz = Rt20 * x + Rt21 * y + Rt22 * z;
    };
    double wx_b, wy_b, wz_b;
    rotate_inv(wx_w, wy_w, wz_w, wx_b, wy_b, wz_b);

    // Base linear acc in world frame; for now just publish gravity in base
    // frame as a reasonable approximation (matches an IMU at rest).
    double ax_b, ay_b, az_b;
    rotate_inv(0.0, 0.0, -9.81, ax_b, ay_b, az_b);

    s.rpy   = {static_cast<float>(roll),  static_cast<float>(pitch),
               static_cast<float>(yaw)};
    s.omega = {static_cast<float>(wx_b),  static_cast<float>(wy_b),
               static_cast<float>(wz_b)};
    s.acc   = {static_cast<float>(ax_b),  static_cast<float>(ay_b),
               static_cast<float>(az_b)};
    s.quat  = {static_cast<float>(qw), static_cast<float>(qx),
               static_cast<float>(qy), static_cast<float>(qz)};
    s.valid = true;
    return s;
}

}  // namespace mj
}  // namespace hal
}  // namespace qmini
