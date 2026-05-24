#pragma once

#include "user/hal/types.h"

namespace qmini {
namespace hal {

class IJoystickBackend {
public:
    virtual ~IJoystickBackend() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    // Pulls any pending events and returns the current frame.
    virtual JoystickFrame read() = 0;
};

}  // namespace hal
}  // namespace qmini
