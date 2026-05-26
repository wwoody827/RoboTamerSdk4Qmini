#include "user/qmini_app.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <thread>

#include <yaml-cpp/yaml.h>

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

    // Dynamic zero offset: load from disk if present (sticky between runs),
    // or capture now if --zero-on-start. Must come before the first read in
    // reset() below so the controller's initial joint_pos is already on the
    // adjusted frame.
    load_dynamic_zero();
    if (opts_.zero_on_start) {
        capture_zero();
    }
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
    if (opts_.sin_joint_idx != -2) {   // -2 sentinel → keep config.yaml value
        rl_->set_sin_joint_idx(opts_.sin_joint_idx);
        sin_joint_now_ = opts_.sin_joint_idx;
    } else {
        sin_joint_now_ = cfg_.sin_joint_idx;
    }

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
            "        w/s=vx+/-  a/d=vy+/-  q/e=yaw+/-  r/space=reset cmd\n"
            "        in mode 5:  [ / ] = prev/next sin joint (0..9)\n"
            "        zero:  z=capture current pose as zero (mode 1 only)\n"
            "               h=toggle hold-zero (mode 0, PD targets q=0)\n");
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
    hal::JoystickFrame js;
    if (opts_.input_from_keyboard) {
        selected_mode_ = mode_switcher_.read_from_keyboard(current_mode_);
    } else {
        js = joystick_->read();
        selected_mode_ = mode_switcher_.read_from_joystick(js, current_mode_);
    }
    if (selected_mode_ != current_mode_) {
        ModeSwitcher::print_selected_mode(selected_mode_);
        relative_time_ = 0.f;
        current_mode_ = selected_mode_;
        rl_->reset(/*reset_pose_to_measured=*/true);
    }
    rl_->set_task_mode(mode_switcher_.rl_task_mode);
    if (current_mode_ == 'q') stop_flag_ = true;

    // Live sin-joint cycling: [ / ] (mapped to hat[0]) bump sin_joint_idx
    // when in mode '5'. Edge detect so the 60 ms hat pulse doesn't fire
    // multiple times per press. Range 0..9; "all joints" was removed for
    // safety (would command every joint simultaneously regardless of pose).
    static const char* kJointNames[10] = {
        "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
        "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
    };
    if (js.hat[0] != 0 && last_hat0_ == 0) {
        int j = sin_joint_now_;
        j += (js.hat[0] > 0) ? 1 : -1;
        if (j > 9) j = 0;           // wrap 9 → 0
        if (j < 0) j = 9;           // wrap 0 → 9
        sin_joint_now_ = j;
        rl_->set_sin_joint_idx(j);
        std::printf("[sin] joint = %d (%s)\n", j, kJointNames[j]);
        std::fflush(stdout);
    }
    last_hat0_ = js.hat[0];

    // Dynamic zero ops (hat[1] pulse):
    //   +1 → 'z' key, capture current pose as zero (only safe in mode '1')
    //   -1 → 'h' key, enter hold-zero mode ('0') / toggle back to '1'
    if (js.hat[1] != 0 && last_hat1_ == 0) {
        if (js.hat[1] > 0) {
            if (current_mode_ == '1') {
                capture_zero();
            } else {
                std::printf("[zero] ignored — press 1 (fold) first; "
                            "re-zeroing under PD is unsafe\n");
            }
        } else {
            if (current_mode_ == '0') {
                std::printf("[zero] hold-zero → fold\n");
                current_mode_ = selected_mode_ = '1';
                relative_time_ = 0.f;
                rl_->reset(true);
            } else if (current_mode_ == '1' || current_mode_ == '2') {
                std::printf("[zero] entering hold-zero (target q=0)\n");
                current_mode_ = selected_mode_ = '0';
                relative_time_ = 0.f;
                rl_->reset(true);
            } else {
                std::printf("[zero] hold-zero only from mode 1 or 2\n");
            }
        }
        std::fflush(stdout);
    }
    last_hat1_ = js.hat[1];
}

void QminiApp::capture_zero() {
    auto state = motor_->read();
    for (int i = 0; i < 10; ++i) {
        // state.q is already adjusted by hardware startq AND any previous
        // dynamic_zero (because control_tick subtracts dynamic_zero before
        // publishing). Capturing state.q here REPLACES the offset — next
        // tick state.q will read 0 at this pose.
        dynamic_zero_[i] += state.q[i];
    }
    save_dynamic_zero();
    std::printf("[zero] captured current pose as zero. dynamic_zero:");
    for (int i = 0; i < 10; ++i) std::printf(" %+.4f", dynamic_zero_[i]);
    std::printf("\n");
    std::fflush(stdout);
}

void QminiApp::save_dynamic_zero() {
    std::ofstream f(opts_.dynamic_zero_path);
    if (!f) {
        std::fprintf(stderr, "[zero] cannot write %s\n",
                     opts_.dynamic_zero_path.c_str());
        return;
    }
    f << "# Dynamic per-joint zero offset, captured at runtime.\n"
      << "# Joint space (rad), order: HYL HRL HPL KL AL HYR HRR HPR KR AR.\n"
      << "# Subtracted from measured q before the controller sees it; added\n"
      << "# back to q_target before the motor sees it. Stacks on top of\n"
      << "# config.yaml::startq (which lives in the hardware backend's\n"
      << "# gear-ratio math).\n"
      << "dynamic_zero: [";
    for (int i = 0; i < 10; ++i) {
        f << dynamic_zero_[i];
        if (i < 9) f << ", ";
    }
    f << "]\n";
}

void QminiApp::load_dynamic_zero() {
    std::ifstream f(opts_.dynamic_zero_path);
    if (!f) return;  // no file → all-zeros is fine
    try {
        YAML::Node y = YAML::Load(f);
        auto v = y["dynamic_zero"].as<std::vector<float>>();
        for (int i = 0; i < 10 && i < static_cast<int>(v.size()); ++i) {
            dynamic_zero_[i] = v[i];
        }
        std::printf("[zero] loaded %s:", opts_.dynamic_zero_path.c_str());
        for (int i = 0; i < 10; ++i) std::printf(" %+.4f", dynamic_zero_[i]);
        std::printf("\n");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[zero] failed to parse %s: %s\n",
                     opts_.dynamic_zero_path.c_str(), e.what());
    }
}

void QminiApp::control_tick() {
    relative_time_ += control_dt_;
    const float ratio = std::min(relative_time_ / opts_.stand_duration, 1.f);

    // Apply dynamic zero offset to the measured motor state before the
    // controller sees it (subtract). The matching add-back happens at the
    // outgoing cmd below.
    hal::MotorStateFrame state = motor_->read();
    for (int i = 0; i < 10; ++i) state.q[i] -= dynamic_zero_[i];

    rl_->update_motor_state(state);
    rl_->update_base_state(imu_->read());
    rl_->update_joystick(joystick_->read());

    switch (current_mode_) {
        case 'q':
            // Stop flag set in mode_tick; emit zero-gain command and let the
            // outer loop wind down.
            break;
        case '0':
            // Hold-zero: PD-ramp joint_act to all-zeros over stand_duration.
            // Useful after capturing dynamic_zero with key 'z' — verifies the
            // captured offset is correct (robot should physically stay put).
            rl_->zero_pose_control(ratio);
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

    // Add dynamic zero back to q_target before shipping to the motor (the
    // matching subtraction happened on the incoming state above). The
    // motor sees the same absolute targets it always did; only the
    // controller's coordinate frame slid.
    hal::MotorCmdFrame cmd = rl_->to_motor_cmd(current_mode_);
    for (int i = 0; i < 10; ++i) cmd.q_target[i] += dynamic_zero_[i];
    motor_->send(cmd);
}

void QminiApp::report_tick() {
    if (current_mode_ >= '1' && current_mode_ != 'q') {
        reporter_.report_data(*rl_);
    }
}

}  // namespace qmini
