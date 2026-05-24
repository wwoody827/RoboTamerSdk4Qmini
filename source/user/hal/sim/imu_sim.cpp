#include <memory>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

// Returns level stance, zero angular velocity, gravity in -Z. Enough
// for the obs builder and the control loop to run end-to-end.
class SimImuBackend : public IImuBackend {
public:
    bool start() override { return true; }
    void stop() override {}

    BaseStateFrame read() override {
        BaseStateFrame s;
        s.rpy = {0.f, 0.f, 0.f};
        s.omega = {0.f, 0.f, 0.f};
        s.acc = {0.f, 0.f, -9.81f};
        s.quat = {1.f, 0.f, 0.f, 0.f};
        s.valid = true;
        return s;
    }
};

}  // namespace

std::unique_ptr<IImuBackend> make_imu_backend(const HardwareConfig&) {
    return std::make_unique<SimImuBackend>();
}

}  // namespace hal
}  // namespace qmini
