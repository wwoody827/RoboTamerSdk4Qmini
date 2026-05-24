// Clock backend that wraps Unitree's CreateRecurrentThreadEx. The sim
// build uses std::thread (clock_std.cpp) and gets identical semantics —
// this exists only because the Unitree SDK's threading also handles CPU
// affinity, real-time priority, and DDS-side bookkeeping in some
// deployment configs.

#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "unitree/common/thread/thread.hpp"
#include "unitree/common/thread/recurrent_thread.hpp"

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

class UnitreeRecurrentThread : public IRecurrentThread {
public:
    UnitreeRecurrentThread(const char* name, long period_us,
                           std::function<void()> body)
        : body_(std::move(body)) {
        thread_ = unitree::common::CreateRecurrentThreadEx(
            name, UT_CPU_ID_NONE, period_us, &UnitreeRecurrentThread::trampoline,
            this);
    }
    ~UnitreeRecurrentThread() override {
        // Unitree SDK's ThreadPtr handles join on destruction; nothing to do.
    }
private:
    static void trampoline(UnitreeRecurrentThread* self) {
        if (self->body_) self->body_();
    }
    std::function<void()> body_;
    unitree::common::ThreadPtr thread_;
};

class UnitreeClock : public IClock {
public:
    std::unique_ptr<IRecurrentThread> create_recurrent(
        const char* name, long period_us,
        std::function<void()> body) override {
        return std::make_unique<UnitreeRecurrentThread>(
            name, period_us, std::move(body));
    }
};

}  // namespace

std::unique_ptr<IClock> make_clock() {
    return std::make_unique<UnitreeClock>();
}

}  // namespace hal
}  // namespace qmini
