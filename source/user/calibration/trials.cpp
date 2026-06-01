#include "user/calibration/trials.h"

#include <algorithm>
#include <cstdio>

namespace qmini {
namespace calib {

const std::array<const char*, kNumJoints> kJointNames = {
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
};

float StepSchedule::multiplier(double t) {
    // 0..1   settle
    // 1..3   +1
    // 3..5    0
    // 5..7   -1
    // 7..9    0
    //
    // Transitions are smoothed over `ramp` (default 50 ms) using a
    // half-cosine. The instantaneous step otherwise excites high-freq
    // modes that destabilize explicit-Euler MuJoCo at 2 ms substep —
    // ankle joints can ring to ±12 rad/s. A 50 ms ramp doesn't materially
    // affect the step-response identification (which fits the second-order
    // settling tail over ~500 ms+) and matches what a real motor's
    // bandwidth-limited current loop would deliver anyway.
    const double ramp = 0.05;
    auto smoothstep = [ramp](double dt_into_segment, float lo, float hi) -> float {
        if (dt_into_segment <= 0)    return lo;
        if (dt_into_segment >= ramp) return hi;
        const double u = dt_into_segment / ramp;
        const double s = 0.5 * (1.0 - std::cos(M_PI * u));
        return static_cast<float>(lo + (hi - lo) * s);
    };
    if (t < settle)            return smoothstep(t - settle,            0.f, 0.f);
    double s = t - settle;
    if (s < seg)               return smoothstep(s,                     0.f, +1.f);
    if (s < 2 * seg)           return smoothstep(s - seg,              +1.f,  0.f);
    if (s < 3 * seg)           return smoothstep(s - 2 * seg,           0.f, -1.f);
    return smoothstep(s - 3 * seg, -1.f, 0.f);
}

float trial_offset(const Trial& trial, double t) {
    switch (trial.test) {
        case TestKind::Step:
            return trial.amp * StepSchedule::multiplier(t);
        case TestKind::Sine: {
            return trial.amp * std::sin(2.0 * M_PI * trial.freq_hz * t);
        }
        case TestKind::Chirp: {
            // Log chirp 0.25 → 10 Hz over 30 s.
            const double f0 = 0.25, f1 = 10.0, T = 30.0;
            const double k  = std::log(f1 / f0);
            // ∫₀ᵗ f(τ) dτ = f0 * T / k * ((f1/f0)^(t/T) - 1)
            double phase = 2.0 * M_PI * f0 * T / k *
                           (std::pow(f1 / f0, t / T) - 1.0);
            return trial.amp * std::sin(phase);
        }
        case TestKind::FreeRelease: {
            // Displace to +amp via a half-cosine ramp, then hold there. The
            // RELEASE (kp=kd=0) is applied in loop.cpp by phase, NOT here —
            // q_target stays at amp through the decay window but has no effect
            // once the gains are zero. At re-engage q_target returns to MGTO
            // (offset 0) while the gains ramp back up. Phases (see loop.cpp):
            //   0..1 settle | 1..3 displace | 3..3.5 hold | 3.5..7 decay | 7..8 reengage
            const double settle = 1.0, ramp = 2.0, hold = 0.5, decay = 3.5;
            if (t < settle) return 0.f;
            if (t < settle + ramp) {
                const double u = (t - settle) / ramp;
                return trial.amp * 0.5f *
                       static_cast<float>(1.0 - std::cos(M_PI * u));
            }
            if (t < settle + ramp + hold + decay) return trial.amp;
            return 0.f;
        }
        case TestKind::ConstVel: {
            // Triangle-wave position sweep at constant |velocity| (stored in
            // freq_hz, rad/s) and amplitude `amp`: the joint traverses ±amp at
            // a fixed speed in both directions, under full PD. Friction is read
            // from tau_est at steady velocity (see fit_friction.py).
            const double settle = 1.0;
            if (t < settle) return 0.f;
            const double v = trial.freq_hz;          // rad/s (field reused)
            const double A = trial.amp;
            if (v <= 0.0 || A <= 0.0) return 0.f;
            const double P = 4.0 * A / v;            // triangle period for slope v
            double saw = std::fmod((t - settle) / P, 1.0);
            if (saw < 0.0) saw += 1.0;
            return static_cast<float>(A * (1.0 - 4.0 * std::fabs(saw - 0.5)));
        }
    }
    return 0.f;
}

float safe_amp(float amp, float mgto_val, float lo, float hi) {
    float up = (hi - mgto_val);
    float dn = (mgto_val - lo);
    float lim = 0.7f * std::min(up, dn);
    if (lim < 0.f) lim = 0.f;
    return std::min(amp, lim);
}

namespace {

std::string fmt_label(const char* test, float kp, float kd,
                      const char* tag, float extra = -1.f) {
    char buf[64];
    if (extra >= 0.f) {
        std::snprintf(buf, sizeof(buf),
                      "%s_kp%.0f_kd%.2f_%s_%.2fHz",
                      test, kp, kd, tag, extra);
    } else {
        std::snprintf(buf, sizeof(buf), "%s_kp%.0f_kd%.2f_%s",
                      test, kp, kd, tag);
    }
    return std::string(buf);
}

}  // namespace

std::vector<Trial> build_default_plan(
    const std::array<float, kNumJoints>& act_pos_low,
    const std::array<float, kNumJoints>& act_pos_high,
    const PoseRef& mgto_pose,
    const std::array<float, kNumJoints>& kp,
    const std::array<float, kNumJoints>& kd,
    const std::vector<float>& sine_freqs) {
    std::vector<Trial> out;
    out.reserve(10 * (1 + sine_freqs.size() + 1 + 1 + 3));  // A + B*freqs + C + D + E*3

    for (int j = 0; j < kNumJoints; ++j) {
        float mgto_j = mgto_pose.q_target[j];
        float lo = act_pos_low[j];
        float hi = act_pos_high[j];
        // Every test drives the joint at its deploy gain from config.yaml.
        const float kp_j = kp[j];
        const float kd_j = kd[j];

        // Test A: single step (no kp/kd sweep).
        {
            float amp_step = safe_amp(0.15f, mgto_j, lo, hi);
            Trial t;
            t.joint = j;
            t.test = TestKind::Step;
            t.kp = kp_j;
            t.kd = kd_j;
            t.amp = amp_step;
            t.freq_hz = -1.f;
            t.pose_id = 0;
            t.label = fmt_label("A", kp_j, kd_j, "step");
            t.duration_s = StepSchedule::total;
            out.push_back(t);
        }

        // Test B: sine sweep.
        float amp_sin = safe_amp(0.10f, mgto_j, lo, hi);
        for (float f : sine_freqs) {
            Trial t;
            t.joint = j;
            t.test = TestKind::Sine;
            t.kp = kp_j;
            t.kd = kd_j;
            t.amp = amp_sin;
            t.freq_hz = f;
            t.pose_id = 0;
            t.label = fmt_label("B", kp_j, kd_j, "sine", f);
            // 5 periods + 1 s settle.
            t.duration_s = 5.0 / f + 1.0;
            out.push_back(t);
        }

        // Test C: log chirp.
        float amp_chirp = safe_amp(0.08f, mgto_j, lo, hi);
        {
            Trial t;
            t.joint = j;
            t.test = TestKind::Chirp;
            t.kp = kp_j;
            t.kd = kd_j;
            t.amp = amp_chirp;
            t.freq_hz = 0.f;
            t.pose_id = 0;
            t.label = fmt_label("C", kp_j, kd_j, "chirp");
            t.duration_s = 30.0;
            out.push_back(t);
        }

        // Test D: free-release passive dynamics (viscous b + Coulomb f).
        // Opt-in via --tests D; filtered out of the default A,B,C run.
        {
            float amp_d = safe_amp(0.15f, mgto_j, lo, hi);
            Trial t;
            t.joint = j;
            t.test = TestKind::FreeRelease;
            t.kp = kp_j;
            t.kd = kd_j;
            t.amp = amp_d;
            t.freq_hz = -1.f;
            t.pose_id = 0;
            t.label = fmt_label("D", kp_j, kd_j, "release");
            t.duration_s = 8.0;
            out.push_back(t);
        }

        // Test E: constant-velocity friction sweep (viscous b + Coulomb f),
        // read from tau_est. Works regardless of gravity restoring → the
        // method for high-ratio hip_roll and no-gravity hip_yaw/ankle. Opt-in
        // via --tests E. One trial per velocity (need ≥2 to split b from f).
        {
            const float amp_e = safe_amp(0.12f, mgto_j, lo, hi);
            const float vels[] = {0.2f, 0.5f, 1.0f};   // rad/s
            for (float v : vels) {
                Trial t;
                t.joint = j;
                t.test = TestKind::ConstVel;
                t.kp = kp_j;
                t.kd = kd_j;
                t.amp = amp_e;
                t.freq_hz = v;                 // velocity (rad/s) stored here
                t.pose_id = 0;
                char lbl[64];
                std::snprintf(lbl, sizeof(lbl), "E_kp%.0f_kd%.2f_cv_%.2frad_s",
                              kp_j, kd_j, v);
                t.label = lbl;
                // ~4 traversals (period = 4A/v), clamped to a sane range.
                t.duration_s =
                    std::min(20.0, std::max(4.0, 4.0 * (4.0 * amp_e / v) + 1.0));
                out.push_back(t);
            }
        }
    }
    return out;
}

std::vector<Trial> build_quick_plan(int joint_index) {
    // Quick sanity plan: one big visible sine on the knee.
    // Knee gives the largest visual motion (lower leg swings). 0.3 rad amp
    // at 0.5 Hz is the rough max amplitude that fits inside the knee's
    // [0, 2.1] range from the MGTO crouch (q≈1.4) without saturating.
    // Used for binary-works-in-sim smoke testing — visible in the viewer.
    std::vector<Trial> out;
    Trial t;
    t.joint = (joint_index == 0) ? 3 : joint_index;   // default = knee_l
    t.test = TestKind::Sine;
    t.kp = 45.f;        // matches training-side QMINI_STIFFNESS for knee
    t.kd = 1.5f;
    t.amp = 0.3f;       // ±17° at the knee — unmistakable
    t.freq_hz = 0.5f;   // slow enough to track cleanly
    t.pose_id = 0;
    t.label = "B_kp45_kd1.5_sine_0.50Hz_quick";
    t.duration_s = 4.0; // 2 full periods
    out.push_back(t);
    return out;
}

}  // namespace calib
}  // namespace qmini
