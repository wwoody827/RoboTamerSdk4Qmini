#pragma once

// PD-calibration inner control loop.
//
// Runs sequential trials at a fixed control rate. Drives one joint at a time
// with the schedule from PD_CALIBRATION_SPEC.md; other 9 joints hold MGTO with
// hold-gains (kp_hold=80, kd_hold=2). Per-tick state captured to an in-memory
// log, then dumped to .npz on trial end.

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "user/calibration/trials.h"
#include "user/hal/imu_backend.h"
#include "user/hal/motor_backend.h"

namespace qmini {
namespace calib {

struct LoopOptions {
    double tick_hz       = 200.0;
    double warm_up_s     = 1.0;
    double rest_between_s = 0.5;
    double cooldown_s    = 2.0;
    float  kp_hold       = 80.f;
    float  kd_hold       = 2.f;
    float  safe_dq_max   = 8.f;    // rad/s
    int    watchdog_n    = 2;      // consecutive ticks
    bool   verbose       = true;
    std::string output_dir;        // run dir (will be created)
};

struct TrialResult {
    std::string file;              // relative path (joint_NN_<name>/<label>.npz)
    std::string result;            // "ok" | "aborted_watchdog" | "aborted_signal"
    double      tick_rate_hz_actual = 0.0;
    int         n_samples = 0;
};

class CalibrationLoop {
public:
    CalibrationLoop(hal::IMotorBackend* motor,
                    hal::IImuBackend*   imu,             // may be nullptr
                    const LoopOptions&  opts,
                    const PoseRef&      mgto_pose);

    // Runs the supplied plan in order. Trials may abort independently.
    // Returns per-trial results aligned with the input plan (same length).
    // Returns early (with partial results filled) when stop() is signalled.
    std::vector<TrialResult> run(const std::vector<Trial>& plan);

    // Signal the loop to abort the current trial and shut down gracefully.
    // Safe to call from a signal handler.
    void stop();

private:
    void send_hold(double settle_s);          // hold MGTO with kp_hold/kd_hold
    void cooldown();

    hal::IMotorBackend* motor_;
    hal::IImuBackend*   imu_;
    LoopOptions         opts_;
    PoseRef             mgto_;
    std::array<float, kNumJoints> hold_kp_{};
    std::array<float, kNumJoints> hold_kd_{};
    volatile bool stop_requested_ = false;
};

}  // namespace calib
}  // namespace qmini
