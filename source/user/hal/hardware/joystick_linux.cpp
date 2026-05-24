// Linux kernel joystick backend — pure C++, no pygame / no embedded Python.
// Reads /dev/input/jsX via <linux/joystick.h>. Replaces the old
// JoystickReader + bin/joystick.py path.
//
// Button / axis indices match the pygame conventions the pre-HAL SDK was
// using for a PS4 / DualShock-style controller. See PYTHON_REMOVAL.md for
// the full table.

#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <linux/joystick.h>
#include <memory>
#include <mutex>
#include <poll.h>
#include <thread>
#include <unistd.h>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

constexpr float kAxisScale = 1.0f / 32767.0f;

class LinuxJoystickBackend : public IJoystickBackend {
public:
    explicit LinuxJoystickBackend(std::string path) : path_(std::move(path)) {}
    ~LinuxJoystickBackend() override { stop(); }

    bool start() override {
        running_ = true;
        thread_ = std::thread([this] { run(); });
        return true;
    }

    void stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    JoystickFrame read() override {
        std::lock_guard<std::mutex> g(mu_);
        return frame_;
    }

private:
    void run() {
        while (running_) {
            if (fd_ < 0) {
                fd_ = ::open(path_.c_str(), O_RDONLY | O_NONBLOCK);
                if (fd_ < 0) {
                    // No joystick yet — wait and retry. This is the same
                    // behavior as boot.sh's busy-wait for /dev/input/js0.
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    continue;
                }
                std::cerr << "joystick: opened " << path_ << "\n";
            }
            pollfd pfd{fd_, POLLIN, 0};
            int r = poll(&pfd, 1, 50);
            if (r < 0 && errno != EINTR) {
                ::close(fd_);
                fd_ = -1;
                continue;
            }
            if (r <= 0) continue;
            js_event ev;
            ssize_t n = ::read(fd_, &ev, sizeof(ev));
            if (n != sizeof(ev)) {
                if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                    continue;
                ::close(fd_);
                fd_ = -1;
                continue;
            }
            // JS_EVENT_INIT events arrive once at open; treat them like
            // normal events to preseed values.
            const bool is_button = (ev.type & ~JS_EVENT_INIT) == JS_EVENT_BUTTON;
            const bool is_axis   = (ev.type & ~JS_EVENT_INIT) == JS_EVENT_AXIS;
            std::lock_guard<std::mutex> g(mu_);
            if (is_axis && ev.number < frame_.axis.size()) {
                frame_.axis[ev.number] =
                    static_cast<float>(ev.value) * kAxisScale;
            } else if (is_button && ev.number < frame_.button.size()) {
                frame_.button[ev.number] = ev.value ? 1 : 0;
            }
            frame_.valid = true;
        }
    }

    std::string path_;
    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::mutex mu_;
    JoystickFrame frame_{};
};

}  // namespace

std::unique_ptr<IJoystickBackend> make_joystick_backend(
    const HardwareConfig& cfg) {
    return std::make_unique<LinuxJoystickBackend>(cfg.joystick_path);
}

}  // namespace hal
}  // namespace qmini
