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
    double ramp_in_s     = 3.0;    // smooth ramp from measured pose -> MGTO
                                   // before warm-up. 0 = skip (only if the
                                   // robot is already held at the stand pose).
    double warm_up_s     = 1.0;
    double rest_between_s = 0.5;
    double cooldown_s    = 2.0;
    bool   fold_at_end   = true;   // after trials, release gains -> limp (fold)
    double fold_s        = 2.0;    // gain-release ramp duration
    // Per-joint hold gains for non-test joints. If left empty, the loop
    // falls back to the scalar defaults below (spec §5 default: 80/2).
    // Recommended (and what pd_calibration_main populates): the
    // training-side QMINI_STIFFNESS / QMINI_PD_DAMPING values from
    // config.yaml::kp/kd, so non-test joints are held with realistic
    // per-joint authority instead of a blanket 80 (which over-stiffens
    // small-inertia joints like ankle and causes integrator instability
    // in MuJoCo dry-runs).
    std::array<float, kNumJoints> hold_kp{};   // 0 → use kp_hold scalar
    std::array<float, kNumJoints> hold_kd{};
    float  kp_hold       = 80.f;   // fallback when hold_kp[i] == 0
    float  kd_hold       = 2.f;
    float  safe_dq_max   = 4.f;    // rad/s (conservative; override via --safe-dq-max)
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
    // If do_ramp is false the caller is responsible for having already brought
    // the robot to MGTO (e.g. via ramp_to_mgto()).
    std::vector<TrialResult> run(const std::vector<Trial>& plan,
                                 bool do_ramp = true);

    // Smoothly ramp from the measured pose to MGTO and leave the robot holding
    // it. Public so the caller can insert a confirmation between the ramp and
    // the trials. Honors stop().
    void ramp_to_mgto(double ramp_s);

    // "Fold": smoothly ramp the PD gains down to zero (q_target held at MGTO)
    // over ramp_s, ending fully limp, so the robot relaxes gracefully instead
    // of being left holding the stand pose at full gain. ramp_s<=0 releases
    // immediately. Called automatically at the end of run() when fold_at_end.
    void fold(double ramp_s);

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
