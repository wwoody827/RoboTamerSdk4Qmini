#pragma once

#include "user/hal/types.h"

namespace qmini {
namespace hal {

class IImuBackend {
public:
    virtual ~IImuBackend() = default;
    virtual bool start() = 0;
    virtual void stop() = 0;
    // Reads the latest base state. Returns valid=false until the first frame
    // has arrived.
    virtual BaseStateFrame read() = 0;
};

}  // namespace hal
}  // namespace qmini
