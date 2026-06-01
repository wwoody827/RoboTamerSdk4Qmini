# Body Excitation Calibration Spec

Date: 2026-05-31

Spec for a calibration tool that drives a coordinated multi-joint
**body-excitation** on the real Qmini robot and logs IMU + joint
response. The data lets us tune MuJoCo's physical parameters (joint
damping, armature, contact friction, contact stiffness) until sim
matches real, producing a faithful sim2sim target without RL.

This file is the source of truth for the implementing agent. Read it
once, then build the tool that fully satisfies the "Acceptance
criteria" below. Companion to `PD_CALIBRATION_SPEC.md` and
`FREE_RELEASE_CALIBRATION_SPEC.md`; shares the same SDK code paths
and NPZ schema.

---

## 1. Why

Single-joint PD calibration (`pd_calibration_tool` Test A/B/C/D)
characterized each motor's response. But sim-to-real also depends on
how the **whole body** responds to joint motion: base coupling,
foot-ground contact, leg-chain reflection. The simplest way to
calibrate these is to drive a known coordinated joint pattern on the
real robot, measure the resulting IMU + foot response, replay the
same input in MuJoCo, and tune MJ parameters until the sim trace
matches.

Three excitation protocols cover the three base rotational axes:

| Protocol | Drive pattern (offset added to MGTO)              | Dominant base response |
|----------|---------------------------------------------------|------------------------|
| P1 Pitch | hip_pitch_l = hip_pitch_r = +A·sin(2πft)          | base pitch ω_y         |
| P2 Roll  | hip_roll_l  = hip_roll_r  = +A·sin(2πft)          | base roll  ω_x         |
| P3 Yaw   | hip_yaw_l   = +A·sin(2πft), hip_yaw_r = −A·sin    | base yaw   ω_z         |

(P3 is antiphase because the joint axes are mirrored L/R; antiphase
joint commands produce *same physical direction* foot motion → net
yaw reaction on base.)

## 2. What we measure

For each excitation protocol, log the full body response time series:

- 10 joint encoders (q[t], dq[t]) — both driven and held joints, to
  capture chain reflection
- 10 joint commands (q_target[t]) — verifies the actual excitation
- IMU (rpy[t], ω[t], acc[t]) — base body response
- Foot normal force (Fz_L[t], Fz_R[t]) — contact behaviour
- Measured motor torque (tau_est[t]) — diagnostic only (HAL has a
  known scaling bug, see `friction_findings_2026_05_31.md`)

The fit/compare side (sim repo) computes:

- Time-domain overlay of measured vs sim quantities
- Bode magnitude + phase of `G(jω) = ω_base_axis(jω) / q_drive(jω)`
- Residual cost as a function of MJ tuning parameters

## 3. Protocol per excitation

```
t = 0..2 s     settle    : PD hold at MGTO with cfg gains. Ensure body
                           is stationary (IMU ω < 0.05 rad/s).
t = 2..27 s    sweep     : 5 sine segments × 5 s each, frequencies
                           0.5, 1.0, 1.5, 2.0, 3.0 Hz. Each segment
                           applies its full sin from t_seg_start with
                           initial phase 0. Joint amplitude A = 0.05 rad
                           (auto-clipped to safe_amp per joint).
t = 27..29 s   cooldown  : back to MGTO with full PD; 2 s settle.
```

Total per protocol: 29 s. Three protocols × 29 s = ~90 s + ramp
overhead. Expected full run: ~2 minutes.

### Drive vector per protocol

`pattern[i]` is the sign coefficient applied to joint i (0..9 in
canonical order HYL HRL HPL KL AL HYR HRR HPR KR AR). The per-step
target offset is `A · pattern[i] · sin(2πf·t_seg)`:

```
P1 Pitch : pattern = [0, 0, +1, 0, 0,  0, 0, +1, 0, 0]   # HP_l, HP_r same sign
P2 Roll  : pattern = [0, +1, 0, 0, 0,  0, +1, 0, 0, 0]   # HR_l, HR_r same sign
P3 Yaw   : pattern = [+1, 0, 0, 0, 0,  -1, 0, 0, 0, 0]   # HY_l = -HY_r (antiphase)
```

Other 8 joints stay at MGTO with full cfg PD throughout.

### Safety

- All non-driven joints hold MGTO with full cfg PD — robot remains
  balanced.
- `safe_amp` (existing helper) clips A to fit each driven joint's
  position range.
- Existing `safe_dq_max` watchdog applies (8 rad/s default).
- Frequency cap 3 Hz: high enough to show base coupling, low enough
  to keep dq < safe_dq_max even at amp 0.05.

### Data captured per excitation

Same NPZ schema as Test A/B/C/D (`npz_writer`), but per-trial in this
case is per *protocol* not per *joint*:

- `t_s` (float64, N)
- `q[:, 10]` — all encoder positions
- `dq[:, 10]` — all velocities
- `q_target[:, 10]` — all commanded positions
- `kp[:, 10]` / `kd[:, 10]` — gains in effect each tick
- `tau_est[:, 10]` — measured torques (diagnostic)
- `imu_rpy[:, 3]`, `imu_omega[:, 3]`, `imu_acc[:, 3]`
- `protocol_name` — "P1_pitch" | "P2_roll" | "P3_yaw"
- `drive_pattern` — 10-vec of ±1/0 multipliers (the `pattern[]` above)
- `sweep_freqs` — list of 5 Hz values used in this protocol
- `seg_starts` — list of 5 segment start times (s)
- `amp_rad` — A used (after safe_amp clipping)
- `tick_rate_hz_actual`

NPZ filename: `<run_dir>/protocol_<P1|P2|P3>_<name>.npz`

Manifest entry per protocol:
```
{"protocol": "P1_pitch", "drive_joints": [2, 7], "pattern_signs": [1,1],
 "freqs_hz": [0.5,1.0,1.5,2.0,3.0], "seg_duration_s": 5, "amp_rad": 0.05,
 "duration_s": 29, "tick_rate_hz_actual": 66.39, "n_samples": 1925,
 "file": "protocol_P1_pitch.npz", "result": "ok"}
```

## 4. SDK changes

A new top-level tool `body_excitation_tool`, NOT a Test type
extension of `pd_calibration_tool`. Reason: protocols drive multiple
joints simultaneously and the per-trial schema is per-protocol not
per-joint — fits awkwardly into the existing Test A/B/C/D loop.

### New files

```
include/user/calibration/body_excitation.h
source/user/calibration/body_excitation.cpp
source/user/calibration/body_excitation_main.cpp
```

### `body_excitation.h`

```cpp
namespace qmini::calib::body {

enum class Protocol : char { Pitch = '1', Roll = '2', Yaw = '3' };

struct ProtocolConfig {
    Protocol id;
    std::string name;                            // "P1_pitch" etc.
    std::array<float, kNumJoints> drive_pattern; // sign coefficients
    std::vector<float> sweep_freqs;              // Hz, default {0.5,1,1.5,2,3}
    float amp_rad;                               // default 0.05
    double settle_s;                             // default 2.0
    double seg_duration_s;                       // default 5.0
    double cooldown_s;                           // default 2.0
};

ProtocolConfig make_protocol(Protocol id);

struct ProtocolResult {
    std::string file;
    std::string result;
    double tick_rate_hz_actual;
    int n_samples;
};

class BodyExcitationRunner {
public:
    BodyExcitationRunner(hal::IMotorBackend* motor,
                         hal::IImuBackend* imu,
                         const LoopOptions& opts,
                         const PoseRef& mgto);
    std::vector<ProtocolResult> run(const std::vector<ProtocolConfig>& plan);
    void stop();
    void fold(double s);
    void ramp_to_mgto(double s);
private:
    /* ... members as in CalibrationLoop ... */
};

}  // namespace
```

### `body_excitation_main.cpp`

Same skeleton as `pd_calibration_main.cpp`:

- Same safety gates (`--i-have-checked-the-harness`, MGTO confirmation)
- Same backend factory (sim / mujoco / hardware)
- Same output layout: `data/body_excitation/<run_id>/`
- Same `--label`, `--operator`, `--notes`, `--ramp-in-s`, etc.

New CLI options:
- `--protocols P1,P2,P3` (default all three)
- `--freqs 0.5,1,1.5,2,3` (default; override for narrower sweep)
- `--amp 0.05` (default; clipped via `safe_amp` per driven joint)
- `--seg-duration 5.0` (default seconds per frequency)

### Trial loop core (in `body_excitation.cpp`)

```cpp
// For each protocol:
//   Phase 1: settle (full PD hold MGTO)
//   Phase 2: sweep — for each f in protocol.sweep_freqs:
//     for t in [0, seg_duration_s] within segment:
//       q_target[i] = MGTO[i] + protocol.amp_rad
//                              * protocol.drive_pattern[i]
//                              * sin(2π · f · t)
//       (driven joints only — others stay at MGTO)
//     log every tick (q, dq, q_target, kp, kd, tau_est, imu_*)
//   Phase 3: cooldown
// Write NPZ + manifest entry
```

All joints (driven AND held) use the **full cfg PD gains** throughout
— no `kp/kd = 0` like Test D. Only the q_target changes.

## 5. Fit/compare side (in qmini_lab repo)

The compare workflow lives in `qmini_lab/scripts/`:

1. **`mj_body_excitation.py`** — loads the same protocol definitions,
   replays in MJ. Reads SDK NPZ for the `q_target` time series (the
   exact commands sent to the real robot), drives MJ with the same
   commands, logs the response in the same NPZ schema. Output:
   `<run_dir>/mj_<protocol>.npz` alongside the SDK NPZ.

2. **`compare_body_excitation.py`** — takes a run dir, finds matching
   real + sim NPZ pairs, produces:
   - Time-domain overlay PNG (joint commands, IMU ω, foot Fz)
   - Bode magnitude + phase PNG per protocol axis
   - Summary YAML with residual cost (RMS of difference in IMU ω)

3. **MJ tuning loop** — sweep params manually or via scipy minimize.
   Initial tuning candidates:
   - `dof_armature` (currently env-var-controlled at 0.001)
   - per-joint `dof_damping` (currently from URDF)
   - floor `friction` slide / torsional / rolling
   - `opt.impratio`, `opt.solref` / `opt.solimp` for contact
   - `max_depenetration_velocity`

## 6. Acceptance criteria

1. `./body_excitation_tool --protocols P1 --quick` runs end-to-end
   without abort, producing a 5-sec single-freq smoke trace.
2. Full `--protocols P1,P2,P3` run completes in < 3 min, NPZs land
   in `data/body_excitation/<run_id>/protocol_P*.npz`.
3. For each protocol, the captured NPZ shows:
   - Non-driven joint q within ±0.02 rad of MGTO throughout (the
     PD hold worked).
   - Driven joint q tracks the commanded sine with phase lag < 50°
     even at 3 Hz (joints are following).
   - IMU ω on the protocol's expected axis shows clear sinusoidal
     response > 0.05 rad/s peak at f ≥ 1 Hz.
4. `qmini_lab/scripts/mj_body_excitation.py <run_dir>` produces
   matching `mj_*.npz` files using only the real run's `q_target`
   as input.
5. `qmini_lab/scripts/compare_body_excitation.py <run_dir>` produces
   Bode overlay PNGs and writes a `match_summary.yaml`.

## 7. Risks / open questions

1. **Base coupling at high freq**: at f > 2 Hz, the whole body may
   start to resonate (base pitch ≈ inverted pendulum frequency
   ~1-2 Hz for a 30 cm robot). The driven joints may not be able to
   excite the base cleanly because the whole system moves together.
   Mitigation: limit max freq to 3 Hz initially; can sweep narrower
   range if needed.
2. **Foot lift-off** at large amplitude / high freq: if base pitches
   far enough, one foot leaves the ground. The contact dynamics
   change discontinuously. Mitigation: keep amp ≤ 0.05 rad; abort
   on `in_contact_L + in_contact_R < 1` (one foot up) sustained > 100 ms.
3. **Other-joint hold gain interaction**: when driven joints push
   hard, neighbouring held joints may experience reaction torque
   beyond their PD authority and drift. Acceptance §3 requires the
   held-joint drift to stay < 0.02 rad; if it doesn't, raise the
   hold gains on those specific joints.
4. **Yaw protocol stability**: yaw drift accumulates if the L/R
   antiphase isn't perfectly symmetric (mass / friction asymmetry).
   Mitigation: report base yaw drift in the manifest; abort if it
   exceeds 30° during the trial.
5. **MJ replay timing alignment**: SDK runs at 66.67 Hz, MJ at
   1000 Hz (then downsampled). The replay script must interpolate
   q_target between SDK ticks. Linear interp is sufficient for the
   smooth sine inputs.

## 8. Out of scope

- Single-joint protocols (already covered by Test A/B/C/D).
- Chirp / random multisine excitation (sweep of discrete sines is
  simpler to fit; could add later if needed for system ID).
- Bidirectional protocols (e.g., L hip_pitch + opposite-phase R) —
  could measure asymmetry but adds complexity. Defer.
- Direct MJ parameter optimization in C++ — done in Python in
  qmini_lab (see §5 compare side).
- Closed-loop sim2sim eval — separate workflow (see existing
  `mj_rollout_march.py` for that).
