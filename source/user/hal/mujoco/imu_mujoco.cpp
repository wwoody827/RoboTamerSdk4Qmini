// MuJoCo IMU backend: reads base orientation / angular velocity from the
// shared World. No physics stepping — that's owned by the motor backend.

#include <memory>

#include "user/hal/factory.h"
#include "world.h"

namespace qmini {
namespace hal {
namespace {

class MujocoImuBackend : public IImuBackend {
public:
    bool start() override { return true; }
    void stop() override {}
    BaseStateFrame read() override {
        return mj::World::instance().read_base_state();
    }
};

}  // namespace

std::unique_ptr<IImuBackend> make_imu_backend(const HardwareConfig&) {
    return std::make_unique<MujocoImuBackend>();
}

}  // namespace hal
}  // namespace qmini
