#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "user/hal/types.h"

struct mjModel_;
struct mjData_;
typedef struct mjModel_ mjModel;
typedef struct mjData_  mjData;

namespace qmini {
namespace hal {
namespace mj {

// Shared MuJoCo world. One instance backs both the motor and IMU backends.
// All physics stepping happens here; backends are thin views over the
// world's mutex-guarded state.
class World {
public:
    static World& instance();

    // Loads the MJCF (idempotent). Returns false on failure.
    bool load(const std::string& mjcf_path);

    // Applies PD torques given the latest cmd + current state, then
    // advances physics by `substeps` MuJoCo steps. Called from motor->send.
    void step_with_cmd(const MotorCmdFrame& cmd, int substeps);

    // Reads joint and base state under the world mutex.
    MotorStateFrame read_motor_state();
    BaseStateFrame  read_base_state();

    bool loaded() const { return model_ != nullptr; }
    double mj_dt() const;

    // Render-side helpers. The viewer thread calls these to snapshot the
    // scene without freezing the control loop.
    //   take_render_snapshot copies qpos/qvel/time + a small bit of context
    //   into the user-supplied buffers under the world mutex. Returns false
    //   if the world isn't loaded yet.
    bool take_render_snapshot(std::vector<double>& qpos,
                              std::vector<double>& qvel,
                              double& time);

    // Public read-only accessors for the viewer.
    const mjModel* model() const { return model_; }

private:
    World() = default;
    ~World();
    World(const World&) = delete;
    World& operator=(const World&) = delete;

    // Joint indices into qpos / qvel for the 10 actuated joints.
    int joint_qpos_addr_[kNumJoints]{};
    int joint_qvel_addr_[kNumJoints]{};
    int actuator_id_[kNumJoints]{};
    int free_joint_qpos_addr_ = -1;  // start of base free joint in qpos (7 entries)
    int free_joint_qvel_addr_ = -1;  // start of base free joint in qvel (6 entries)

    std::mutex mu_;
    mjModel* model_ = nullptr;
    mjData*  data_  = nullptr;
};

}  // namespace mj
}  // namespace hal
}  // namespace qmini
