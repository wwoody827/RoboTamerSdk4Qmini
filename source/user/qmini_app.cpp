#include "user/qmini_app.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#ifdef QMINI_HAVE_VIEWER
#include "viewer.h"  // resolved via MUJOCO_PRIVATE_INC (mujoco backend only)
#endif

namespace qmini {

QminiApp::QminiApp(Options opts)
    : opts_(std::move(opts)) {
    current_mode_  = opts_.initial_mode;
    selected_mode_ = opts_.initial_mode;
    // ConfigParams loads from "config.yaml" in CWD.
    control_dt_ = cfg_.control_dt;

    // Carry startq into hardware config so the motor backend doesn't need
    // its own YAML load. (Sim backend ignores startq.)
    for (size_t i = 0; i < cfg_.startq.size() && i < 10; ++i) {
        opts_.hw.startq[i] = cfg_.startq[i];
    }

    motor_    = hal::make_motor_backend(opts_.hw);
    imu_      = hal::make_imu_backend(opts_.hw);
    joystick_ = hal::make_joystick_backend(opts_.hw);
    clock_    = hal::make_clock();

    motor_->start();
    imu_->start();
    // When --keyboard is set, ModeSwitcher::read_from_keyboard owns stdin.
    // Starting the sim KeyboardJoystick would put the TTY into raw mode and
    // drain stdin out from under the mode switcher → user keystrokes vanish.
    // Leave the joystick instance alive (so .read() still returns an empty
    // frame the controller can consume) but skip start() to avoid the grab.
    if (!opts_.input_from_keyboard) {
        joystick_->start();
    }

    std::unique_ptr<IPolicy> policy;
    if (opts_.use_real_onnx) {
        policy = make_onnx_policy(opts_.policy_path,
                                  cfg_.num_observations,
                                  cfg_.num_stacks,
                                  cfg_.num_actions);
    } else {
        policy = make_identity_policy(cfg_.num_observations,
                                      cfg_.num_stacks,
                                      cfg_.num_actions);
    }
    rl_ = std::make_unique<RLController>(cfg_, std::move(policy));
    rl_->init();
    rl_->set_stand_kp_scale(opts_.stand_kp_scale);
    rl_->set_stand_kd_scale(opts_.stand_kd_scale);

    if (opts_.enable_logging) {
        reporter_.init(true, true);
    }

    // Pull initial state so reset() has something real to anchor to.
    rl_->update_motor_state(motor_->read());
    rl_->update_base_state(imu_->read());
    rl_->reset(/*reset_pose_to_measured=*/true);

    if (opts_.start_threads) {
        control_thread_ = clock_->create_recurrent(
            "control", static_cast<long>(control_dt_ * 1e6),
            [this] { control_tick(); });
        mode_thread_ = clock_->create_recurrent(
            "mode", 20'000, [this] { mode_tick(); });
        if (opts_.enable_logging) {
            report_thread_ = clock_->create_recurrent(
                "report", 10'000, [this] { report_tick(); });
        }
    }

#ifdef QMINI_HAVE_VIEWER
    if (opts_.enable_viewer) {
        hal::mj::Viewer::instance().start();
    }
#endif

    if (opts_.input_from_keyboard) {
        std::printf(
            "[qmini] keyboard mode: type a digit + Enter to switch mode\n"
            "        1=fold  2=stand  3=walk  5=sin  q=e-stop\n");
    } else {
        std::printf(
            "[qmini] stdin joystick: press a key (no Enter), see [key →] echo\n"
            "        1=fold  2=stand  3=walk  5=sin  b=quit\n"
            "        w/s=vx+/-  a/d=vy+/-  q/e=yaw+/-  r/space=reset cmd\n");
    }
    std::fflush(stdout);
}

QminiApp::~QminiApp() {
    stop_flag_ = true;
    control_thread_.reset();
    mode_thread_.reset();
    report_thread_.reset();
#ifdef QMINI_HAVE_VIEWER
    hal::mj::Viewer::instance().stop();
#endif
    if (joystick_) joystick_->stop();
    if (imu_)      imu_->stop();
    if (motor_)    motor_->stop();
    reporter_.close();
}

void QminiApp::run() {
    using namespace std::chrono_literals;
    while (!stop_flag_.load()) {
        std::this_thread::sleep_for(50ms);
#ifdef QMINI_HAVE_VIEWER
        if (hal::mj::Viewer::instance().should_close()) {
            stop_flag_ = true;
            break;
        }
#endif
    }
}

void QminiApp::stop() { stop_flag_ = true; }

void QminiApp::tick() {
    control_tick();
}

void QminiApp::mode_tick() {
    if (opts_.input_from_keyboard) {
        selected_mode_ = mode_switcher_.read_from_keyboard(current_mode_);
    } else {
        selected_mode_ = mode_switcher_.read_from_joystick(
            joystick_->read(), current_mode_);
    }
    if (selected_mode_ != current_mode_) {
        ModeSwitcher::print_selected_mode(selected_mode_);
        relative_time_ = 0.f;
        current_mode_ = selected_mode_;
        rl_->reset(/*reset_pose_to_measured=*/true);
    }
    rl_->set_task_mode(mode_switcher_.rl_task_mode);
    if (current_mode_ == 'q') stop_flag_ = true;
}

void QminiApp::control_tick() {
    relative_time_ += control_dt_;
    const float ratio = std::min(relative_time_ / opts_.stand_duration, 1.f);

    rl_->update_motor_state(motor_->read());
    rl_->update_base_state(imu_->read());
    rl_->update_joystick(joystick_->read());

    switch (current_mode_) {
        case 'q':
            // Stop flag set in mode_tick; emit zero-gain command and let the
            // outer loop wind down.
            break;
        case '2':
            rl_->stand_control(ratio);
            break;
        case '3':
            rl_->rl_control();
            if (rl_->counter_rl() < 2) rl_->stand_control(ratio);
            break;
        case '5':
            rl_->sin_control(opts_.sin_amplitude, opts_.sin_frequency, relative_time_);
            break;
        default:
            rl_->stand_control(ratio);
            break;
    }

    motor_->send(rl_->to_motor_cmd(current_mode_));
}

void QminiApp::report_tick() {
    if (current_mode_ >= '1' && current_mode_ != 'q') {
        reporter_.report_data(*rl_);
    }
}

}  // namespace qmini
