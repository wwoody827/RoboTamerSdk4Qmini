#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

class StdRecurrentThread : public IRecurrentThread {
public:
    StdRecurrentThread(const char* name, long period_us,
                       std::function<void()> body)
        : name_(name), period_us_(period_us), body_(std::move(body)),
          running_(true) {
        thread_ = std::thread([this] { run(); });
    }
    ~StdRecurrentThread() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    void run() {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        while (running_) {
            try {
                body_();
            } catch (const std::exception& e) {
                std::cerr << "[" << name_ << "] body threw: " << e.what()
                          << std::endl;
            }
            next += std::chrono::microseconds(period_us_);
            std::this_thread::sleep_until(next);
        }
    }

    std::string name_;
    long period_us_;
    std::function<void()> body_;
    std::atomic<bool> running_;
    std::thread thread_;
};

class StdClock : public IClock {
public:
    std::unique_ptr<IRecurrentThread> create_recurrent(
        const char* name, long period_us,
        std::function<void()> body) override {
        return std::make_unique<StdRecurrentThread>(
            name, period_us, std::move(body));
    }
};

}  // namespace

std::unique_ptr<IClock> make_clock() {
    return std::make_unique<StdClock>();
}

}  // namespace hal
}  // namespace qmini
