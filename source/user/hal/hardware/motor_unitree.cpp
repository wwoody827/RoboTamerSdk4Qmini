// Motor backend: drives the 10 GO-M8010-6 motors via 4 parallel serial
// threads, exactly as the pre-HAL MotorController did. The serial-grouping
// and gear-ratio logic is preserved bit-for-bit; only the I/O contract
// changed (POD frames in/out instead of DDS LowCmd_).

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#include "user/hal/factory.h"

namespace qmini {
namespace hal {
namespace {

struct SerialGroup {
    const char* port;
    std::vector<int> motor_ids;
};

// FTDI quad-channel port mapping. Keep in sync with MOTOR_PORT_MAP.md.
const std::array<SerialGroup, 4> kSerialGroups = {{
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0",
     {0, 5}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if02-port0",
     {1, 6}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if00-port0",
     {2, 3, 4}},
    {"/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0",
     {7, 8, 9}},
}};

constexpr float kSpeedRatio = 6.33f;
constexpr float kGearRatio  = 3.0f;
constexpr int   kNumMotors  = 10;

inline bool is_special(int motor_id) {
    return motor_id == 1 || motor_id == 6;
}

inline int channel_id(int motor_id) {
    if (motor_id == 1) return 0;
    if (motor_id >= 2 && motor_id <= 4) return motor_id - 2;
    if (motor_id == 5) return 1;
    if (motor_id == 6) return 1;
    if (motor_id >= 7 && motor_id <= 9) return motor_id - 7;
    return motor_id;  // 0 maps to 0
}

class UnitreeMotorBackend : public IMotorBackend {
public:
    explicit UnitreeMotorBackend(const HardwareConfig& cfg) {
        std::memcpy(startq_.data(), cfg.startq, sizeof(startq_));
    }
    ~UnitreeMotorBackend() override { stop(); }

    bool start() override {
        for (const auto& g : kSerialGroups) {
            ports_.emplace_back(std::make_unique<SerialPort>(g.port));
        }
        running_ = true;
        for (size_t i = 0; i < kSerialGroups.size(); ++i) {
            workers_.emplace_back([this, i] { run_group(i); });
        }
        return true;
    }

    void stop() override {
        running_ = false;
        for (auto& t : workers_) if (t.joinable()) t.join();
        workers_.clear();
        ports_.clear();
    }

    void send(const MotorCmdFrame& cmd) override {
        std::lock_guard<std::mutex> g(cmd_mu_);
        cmd_ = cmd;
    }

    MotorStateFrame read() override {
        std::lock_guard<std::mutex> g(state_mu_);
        return state_;
    }

    void set_zero_offset(const std::array<float, kNumJoints>& offset) override {
        std::lock_guard<std::mutex> g(startq_mu_);
        for (int i = 0; i < kNumMotors; ++i) startq_[i] = offset[i];
    }

private:
    void run_group(size_t group_idx) {
        SerialPort& serial = *ports_[group_idx];
        const auto& ids = kSerialGroups[group_idx].motor_ids;
        while (running_) {
            MotorCmdFrame snapshot;
            { std::lock_guard<std::mutex> g(cmd_mu_); snapshot = cmd_; }
            std::array<float, kNumMotors> sq;
            { std::lock_guard<std::mutex> g(startq_mu_); sq = startq_; }
            for (int id : ids) {
                MotorCmd c{};
                MotorData d{};
                c.motorType = MotorType::GO_M8010_6;
                c.mode = queryMotorMode(MotorType::GO_M8010_6, MotorMode::FOC);
                c.id = channel_id(id);
                const float ratio = is_special(id)
                                  ? (kSpeedRatio * kGearRatio) : kSpeedRatio;
                // Joint→motor space PD conversion. The Unitree GO-M8010-6
                // firmware computes PD in MOTOR space:
                //   tau_m = kp_m·(θ_target_m − θ_m) − kd_m·dθ_m
                // and θ_m = G·q_joint, tau_joint = G·tau_m, so the
                // effective joint-space gain is G²·kp_m. To deliver the
                // joint-space PD that the controller requests (matching
                // training-side QMINI_STIFFNESS / QMINI_PD_DAMPING in
                // config.yaml), divide by G² here.
                const float ratio2 = ratio * ratio;
                c.kp  = snapshot.kp[id] / ratio2;
                c.kd  = snapshot.kd[id] / ratio2;
                c.tau = snapshot.tau_ff[id];
                c.q  = (snapshot.q_target[id] + sq[id]) * ratio;
                c.dq = snapshot.dq_target[id] * ratio;
                d.motorType = MotorType::GO_M8010_6;
                try {
                    serial.sendRecv(&c, &d);
                    const float q  = d.q  / ratio - sq[id];
                    const float dq = d.dq / ratio;
                    const float t  = d.tau / ratio;
                    std::lock_guard<std::mutex> g(state_mu_);
                    state_.q[id]       = q;
                    state_.dq[id]      = dq;
                    state_.tau_est[id] = t;
                } catch (const std::exception& e) {
                    std::cerr << "[m" << id << "] serial: " << e.what() << "\n";
                } catch (...) {
                    std::cerr << "[m" << id << "] serial: unknown error\n";
                }
            }
        }
    }

    std::array<float, kNumMotors> startq_{};
    std::mutex startq_mu_;
    std::atomic<bool> running_{false};
    std::vector<std::unique_ptr<SerialPort>> ports_;
    std::vector<std::thread> workers_;
    std::mutex cmd_mu_;
    MotorCmdFrame cmd_{};
    std::mutex state_mu_;
    MotorStateFrame state_{};
};

}  // namespace

std::unique_ptr<IMotorBackend> make_motor_backend(const HardwareConfig& cfg) {
    return std::make_unique<UnitreeMotorBackend>(cfg);
}

}  // namespace hal
}  // namespace qmini
