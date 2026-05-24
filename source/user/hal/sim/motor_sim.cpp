#include <atomic>
#include <memory>
#include <mutex>

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

// Simulated motors: measured q tracks target with first-order lag,
// dq is the discrete derivative. No physics — just enough to exercise
// the full control-loop path end-to-end.
class SimMotorBackend : public IMotorBackend {
public:
    bool start() override { return true; }
    void stop() override {}

    void send(const MotorCmdFrame& cmd) override {
        std::lock_guard<std::mutex> g(mu_);
        cmd_ = cmd;
    }

    MotorStateFrame read() override {
        std::lock_guard<std::mutex> g(mu_);
        MotorStateFrame s;
        constexpr float alpha = 0.8f;  // measured = alpha*target + (1-alpha)*last_q
        for (int i = 0; i < kNumJoints; ++i) {
            float new_q = alpha * cmd_.q_target[i] + (1.f - alpha) * q_[i];
            s.dq[i] = (new_q - q_[i]) / 0.01f;  // assume 100Hz control
            q_[i] = new_q;
            s.q[i] = new_q;
            s.tau_est[i] = cmd_.kp[i] * (cmd_.q_target[i] - new_q)
                         - cmd_.kd[i] * s.dq[i];
            s.ddq[i] = 0.f;
        }
        last_state_ = s;
        return s;
    }

private:
    std::mutex mu_;
    MotorCmdFrame cmd_;
    MotorStateFrame last_state_;
    std::array<float, kNumJoints> q_{};
};

}  // namespace

std::unique_ptr<IMotorBackend> make_motor_backend(const HardwareConfig&) {
    return std::make_unique<SimMotorBackend>();
}

}  // namespace hal
}  // namespace qmini
