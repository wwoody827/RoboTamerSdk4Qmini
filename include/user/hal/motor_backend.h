#pragma once

#include <array>

#include "user/hal/types.h"

namespace qmini {
namespace hal {

// Sends position-PD commands, returns measured state. Implementations:
//   - hardware: wraps Unitree GO-M8010-6 serial drivers (4 parallel threads)
//   - sim:      echoes target to measured with small lag
//   - replay:   reads logged frames at the original tick rate
class IMotorBackend {
public:
    virtual ~IMotorBackend() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    virtual void send(const MotorCmdFrame& cmd) = 0;
    virtual MotorStateFrame read() = 0;

    // Update the per-joint zero offset at runtime. Controller-frame position is
    // measured_raw/ratio - offset, and offset is added to q_target before it is
    // sent. Default no-op; only the hardware backend honors it (sim/mujoco
    // ignore startq entirely).
    virtual void set_zero_offset(const std::array<float, kNumJoints>& /*offset*/) {}
};

}  // namespace hal
}  // namespace qmini
