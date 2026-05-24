#include <atomic>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

// Keyboard-driven sim joystick.
//   w/s: cmd_vx +/-     a/d: cmd_vy +/-     q/e: yaw +/-
//   r:   zero command   space: idle
//   1..9, b: mapped to button presses to drive the mode FSM:
//     '1' -> START      '2' -> A          '3' -> Y       '4' -> X
//     '5' -> SELECT     '6' -> L1         '7' -> R1      '8' -> L2
//     '9' -> R2         'b' -> B (quit)
//
// Non-blocking stdin via O_NONBLOCK + termios raw mode. If stdin is not a
// TTY (e.g. tests, daemon mode) the keyboard reader is silently disabled —
// the joystick frame stays all-zeros, which the control loop treats as
// "no command".
class KeyboardJoystick : public IJoystickBackend {
public:
    bool start() override {
        if (!isatty(STDIN_FILENO)) {
            tty_ok_ = false;
            return true;
        }
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) {
            tty_ok_ = false;
            return true;
        }
        termios raw = orig_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
            tty_ok_ = false;
            return true;
        }
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
        tty_ok_ = true;
        return true;
    }

    void stop() override {
        if (tty_ok_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }

    JoystickFrame read() override {
        if (tty_ok_) pump_keys();
        std::lock_guard<std::mutex> g(mu_);
        // Latch button presses: clear after one read so the FSM sees an edge.
        JoystickFrame out = frame_;
        for (int& b : frame_.button) b = 0;
        out.valid = true;
        return out;
    }

private:
    static constexpr float kStep = 0.1f;

    void pump_keys() {
        char buf[32];
        ssize_t n = ::read(STDIN_FILENO, buf, sizeof(buf));
        if (n <= 0) return;
        std::lock_guard<std::mutex> g(mu_);
        for (ssize_t i = 0; i < n; ++i) {
            char c = buf[i];
            switch (c) {
                case 'w': frame_.axis[1] -= kStep; break;
                case 's': frame_.axis[1] += kStep; break;
                case 'a': frame_.axis[0] -= kStep; break;
                case 'd': frame_.axis[0] += kStep; break;
                case 'q': frame_.axis[2] -= kStep; break;
                case 'e': frame_.axis[2] += kStep; break;
                case ' ':
                case 'r':
                    for (float& a : frame_.axis) a = 0.f;
                    break;
                case '1': frame_.button[9] = 1; break;  // START
                case '2': frame_.button[0] = 1; break;  // A
                case '3': frame_.button[3] = 1; break;  // Y
                case '4': frame_.button[2] = 1; break;  // X
                case '5': frame_.button[8] = 1; break;  // SELECT
                case '6': frame_.button[4] = 1; break;  // L1
                case '7': frame_.button[5] = 1; break;  // R1
                case '8': frame_.button[6] = 1; break;  // L2
                case '9': frame_.button[7] = 1; break;  // R2
                case 'b': frame_.button[1] = 1; break;  // B / quit
                default: break;
            }
            for (float& a : frame_.axis) {
                if (a > 1.f) a = 1.f;
                if (a < -1.f) a = -1.f;
            }
        }
    }

    std::mutex mu_;
    JoystickFrame frame_;
    termios orig_{};
    bool tty_ok_ = false;
};

}  // namespace

std::unique_ptr<IJoystickBackend> make_joystick_backend(const HardwareConfig&) {
    return std::make_unique<KeyboardJoystick>();
}

}  // namespace hal
}  // namespace qmini
