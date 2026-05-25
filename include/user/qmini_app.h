#pragma once

#include <atomic>
#include <memory>

#include "user/data_report.h"
#include "user/hal/factory.h"
#include "user/mode_switcher.h"
#include "user/rl_controller.h"
#include "utils/config.h"

namespace qmini {

// Top-level orchestrator. Owns HAL backends, the RLController, and the
// recurrent threads. No DDS / Unitree types in its interface.
class QminiApp {
public:
    struct Options {
        hal::HardwareConfig hw;
        std::string policy_path = "policy.onnx";  // ignored when use_real_onnx=false
        bool use_real_onnx = true;
        bool input_from_keyboard = false;         // dev: read mode from stdin
        bool enable_logging = true;
        bool start_threads = true;                // false → caller drives ticks
        bool enable_viewer = true;                // mujoco backend live viewer
        float stand_kp_scale = 1.f;               // see RLController docs
        float stand_kd_scale = 1.f;
        char  initial_mode = '1';                 // FSM mode at startup
        float stand_duration = 2.f;               // seconds to ramp init→ref
        float sin_amplitude = 0.5f;               // mode '5' sin amplitude (rad)
        float sin_frequency = 1.f;                // mode '5' sin frequency (Hz)
        int   sin_joint_idx = -2;                 // -2 = use config.yaml; -1 = all; 0..9 = single
    };

    explicit QminiApp(Options opts);
    ~QminiApp();

    QminiApp(const QminiApp&) = delete;
    QminiApp& operator=(const QminiApp&) = delete;

    void run();      // blocks until stop()
    void stop();
    bool stopped() const { return stop_flag_.load(); }

    // Single-tick API (used by tests; ignores threads).
    void tick();

    // Read-only accessors for tests.
    const RLController& rl() const { return *rl_; }
    char current_mode() const { return current_mode_; }

private:
    void control_tick();
    void mode_tick();
    void report_tick();

    Options opts_;
    ConfigParams cfg_;

    std::unique_ptr<hal::IMotorBackend>    motor_;
    std::unique_ptr<hal::IImuBackend>      imu_;
    std::unique_ptr<hal::IJoystickBackend> joystick_;
    std::unique_ptr<hal::IClock>           clock_;

    std::unique_ptr<RLController> rl_;
    ModeSwitcher mode_switcher_;
    DataReporter reporter_;

    char current_mode_  = '1';
    char selected_mode_ = '1';
    float relative_time_  = 0.f;
    float control_dt_     = 0.01f;

    std::atomic<bool> stop_flag_{false};
    std::unique_ptr<hal::IRecurrentThread> control_thread_;
    std::unique_ptr<hal::IRecurrentThread> mode_thread_;
    std::unique_ptr<hal::IRecurrentThread> report_thread_;
};

}  // namespace qmini
