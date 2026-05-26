#include <atomic>
#include <chrono>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

using clk = std::chrono::steady_clock;

// Keyboard-driven sim joystick.
//   w/s: cmd_vx +/-     a/d: cmd_vy +/-     q/e: yaw +/-
//   r:   zero command   space: idle
//   1..9, b: button presses to drive the mode FSM:
//     '1' -> START      '2' -> A          '3' -> Y       '4' -> X
//     '5' -> SELECT     '6' -> L1         '7' -> R1      '8' -> L2
//     '9' -> R2         'b' -> B (quit)
//
// One dedicated thread pumps stdin; control_tick / mode_tick just lock the
// mutex and copy. Without the dedicated thread, both threads called
// pump_keys() inside their joystick.read() and raced on stdin — half the
// keystrokes ended up consumed by the wrong reader and the FSM appeared
// "frozen". See the commit log for the history.
//
// Button presses are latched as time-windowed PULSES (default 60 ms). Any
// reader within the window sees button==1; both readers see the same press.
// After the window, button auto-clears even if no one consumed it.
//
// If stdin isn't a TTY (tests, daemon mode) the reader is silently disabled
// — read() still returns an empty frame the controller can consume.
class KeyboardJoystick : public IJoystickBackend {
public:
    bool start() override {
        if (!isatty(STDIN_FILENO)) { tty_ok_ = false; return true; }
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) { tty_ok_ = false; return true; }
        termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            tty_ok_ = false; return true;
        }
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        tty_ok_ = true;
        running_ = true;
        thread_ = std::thread([this] { stdin_loop(); });
        return true;
    }

    void stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (tty_ok_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }

    JoystickFrame read() override {
        std::lock_guard<std::mutex> g(mu_);
        // Time-decay any expired button pulses so the FSM sees the press
        // through both readers but doesn't react to a stuck button forever.
        auto now = clk::now();
        for (int i = 0; i < 10; ++i) {
            if (frame_.button[i] &&
                now - button_set_at_[i] > std::chrono::milliseconds(kPulseMs)) {
                frame_.button[i] = 0;
            }
        }
        for (int i = 0; i < 2; ++i) {
            if (frame_.hat[i] != 0 &&
                now - hat_set_at_[i] > std::chrono::milliseconds(kPulseMs)) {
                frame_.hat[i] = 0;
            }
        }
        JoystickFrame out = frame_;
        out.valid = true;
        return out;
    }

private:
    static constexpr float kStep = 0.1f;
    static constexpr int   kPulseMs = 60;

    void stdin_loop() {
        while (running_.load()) {
            struct pollfd p{STDIN_FILENO, POLLIN, 0};
            int rv = ::poll(&p, 1, 20);  // 20 ms tick
            if (rv > 0 && (p.revents & POLLIN)) {
                char buf[32];
                ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
                if (n > 0) consume(buf, n);
            }
        }
    }

    void consume(const char* buf, ssize_t n) {
        std::lock_guard<std::mutex> g(mu_);
        auto now = clk::now();
        for (ssize_t i = 0; i < n; ++i) {
            char c = buf[i];
            int button_idx = -1;
            const char* label = nullptr;
            switch (c) {
                case 'w': frame_.axis[1] -= kStep; label = "vx+"; break;
                case 's': frame_.axis[1] += kStep; label = "vx-"; break;
                case 'a': frame_.axis[0] -= kStep; label = "vy+"; break;
                case 'd': frame_.axis[0] += kStep; label = "vy-"; break;
                case 'q': frame_.axis[2] -= kStep; label = "yaw+"; break;
                case 'e': frame_.axis[2] += kStep; label = "yaw-"; break;
                case ' ':
                case 'r':
                    for (float& a : frame_.axis) a = 0.f;
                    label = "reset";
                    break;
                case '[': frame_.hat[0] = -1; hat_set_at_[0] = now; label = "sin_joint--"; break;
                case ']': frame_.hat[0] = +1; hat_set_at_[0] = now; label = "sin_joint++"; break;
                case 'z': frame_.hat[1] = +1; hat_set_at_[1] = now; label = "capture_zero"; break;
                case 'h': frame_.hat[1] = -1; hat_set_at_[1] = now; label = "hold_zero";    break;
                case '1': button_idx = 9; label = "fold";        break;
                case '2': button_idx = 0; label = "stand";       break;
                case '3': button_idx = 3; label = "walk";        break;
                case '4': button_idx = 2; label = "rl_stand";    break;
                case '5': button_idx = 8; label = "sin_test";    break;
                case '6': button_idx = 4; label = "L1";          break;
                case '7': button_idx = 5; label = "R1";          break;
                case '8': button_idx = 6; label = "L2";          break;
                case '9': button_idx = 7; label = "R2";          break;
                case 'b': button_idx = 1; label = "quit";        break;
                default: break;
            }
            if (button_idx >= 0) {
                frame_.button[button_idx] = 1;
                button_set_at_[button_idx] = now;
            }
            for (float& a : frame_.axis) {
                if (a >  1.f) a =  1.f;
                if (a < -1.f) a = -1.f;
            }
            if (label) {
                // Echo each consumed key so the user has feedback (ECHO is
                // off in raw mode). Newline + flush so it appears immediately.
                std::printf("\r[key '%c' → %s]\n", c, label);
                std::fflush(stdout);
            }
        }
    }

    std::mutex mu_;
    JoystickFrame frame_;
    clk::time_point button_set_at_[10]{};
    clk::time_point hat_set_at_[2]{};
    termios orig_{};
    bool tty_ok_ = false;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace

std::unique_ptr<IJoystickBackend> make_joystick_backend(const HardwareConfig&) {
    return std::make_unique<KeyboardJoystick>();
}

}  // namespace hal
}  // namespace qmini
