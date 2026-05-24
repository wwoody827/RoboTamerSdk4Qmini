// MuJoCo motor backend: send() steps physics with PD torques applied;
// read() returns latest qpos/qvel slice. Shares World singleton with the
// IMU backend so both see consistent state.

#include <cmath>
#include <memory>
#include <string>

#include "user/hal/factory.h"
#include "world.h"

namespace qmini {
namespace hal {
namespace {

class MujocoMotorBackend : public IMotorBackend {
public:
    explicit MujocoMotorBackend(std::string mjcf_path)
        : mjcf_path_(std::move(mjcf_path)) {}

    bool start() override {
        return mj::World::instance().load(mjcf_path_);
    }
    void stop() override {}

    void send(const MotorCmdFrame& cmd) override {
        // Control tick is 10 ms; MuJoCo default timestep is 2 ms → 5 substeps.
        const double mj_dt = mj::World::instance().mj_dt();
        const double control_dt = 0.01;
        const int substeps = std::max(1, static_cast<int>(
            std::round(control_dt / mj_dt)));
        mj::World::instance().step_with_cmd(cmd, substeps);
    }

    MotorStateFrame read() override {
        return mj::World::instance().read_motor_state();
    }

private:
    std::string mjcf_path_;
};

}  // namespace

std::unique_ptr<IMotorBackend> make_motor_backend(const HardwareConfig& cfg) {
    // HardwareConfig doesn't carry an MJCF path — fall back to the bundled
    // sim_assets relative to CWD if cfg's joystick_path is at default.
    // The QminiApp ctor for the mujoco preset passes the absolute path
    // through HardwareConfig::imu_serial_path (we hijack it; see below).
    (void)cfg;
    return std::make_unique<MujocoMotorBackend>("sim_assets/q1_sim.mjcf");
}

}  // namespace hal
}  // namespace qmini
