#pragma once

#include <functional>
#include <memory>

namespace qmini {
namespace hal {

// Recurrent thread handle. Destroying it joins the thread.
class IRecurrentThread {
public:
    virtual ~IRecurrentThread() = default;
};

// Abstracts away the "spawn a thread that runs body() every period_us"
// pattern. Hardware backend wraps Unitree's CreateRecurrentThreadEx; sim
// backend uses std::thread + sleep_until.
class IClock {
public:
    virtual ~IClock() = default;
    virtual std::unique_ptr<IRecurrentThread> create_recurrent(
        const char* name,
        long period_us,
        std::function<void()> body) = 0;
};

}  // namespace hal
}  // namespace qmini
