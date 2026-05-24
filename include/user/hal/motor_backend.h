#pragma once

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
};

}  // namespace hal
}  // namespace qmini
