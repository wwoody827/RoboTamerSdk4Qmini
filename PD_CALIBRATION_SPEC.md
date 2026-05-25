# PD Calibration Spec

Date: 2026-05-25

Spec for a calibration tool that measures the real Qmini robot's per-joint PD
response while fully assembled (cannot disassemble motors from limbs), so
Isaac Lab and MuJoCo can be configured with **measured** PD gains and inertias
instead of nominal-from-URDF values.

This file is the source of truth for the implementing agent. Read it once, then
build a tool that fully satisfies "Goals" and "Acceptance criteria" below.

---

## 1. Why

`docs/sim2sim_trace_findings_2026-05-25.md` (in the `qmini_lab` repo) shows
that with identical action and observation at step 0, Isaac and MuJoCo joints
respond differently within the first 15 ms of physics. The most likely root
cause is that the PD model used in both sims does not match the real robot:

- Isaac Lab uses `ImplicitActuator` with PhysX implicit drive.
- MuJoCo uses explicit substep PD with `kds = QMINI_PD_DAMPING_TUPLE * 5.0`
  (a hand-tuned "× 5" fudge to make MuJoCo settle behave like Isaac).
- The actual hardware uses Unitree GO-M8010-6 motors with a 6.33:1 speed
  ratio plus an extra 3× gear on hip_roll, driven by the SDK's
  `IMotorBackend::send(MotorCmdFrame)` interface.

Until we know `kp_real`, `kd_real`, and effective joint inertia `I_eff` per
joint **for the assembled robot in the operating posture**, we cannot say
either sim is right or pinpoint which one is wrong.

## 2. Goals

1. Per-joint **operating-point effective PD model** `(kp_eff, kd_eff, I_eff)`
   measured around the MGTO standing pose, with the rest of the robot attached
   and gravity loading the chain naturally. See §2.1 for what "effective"
   means and why this is the right quantity for sim2real.
2. A coverage check: redo the measurement at 2-3 additional poses (e.g.
   knee-bent walk pose, crouch pose) so we can quantify how strongly
   `(kp_eff, I_eff)` depend on configuration. This also tells us how much
   the gravity gradient (`g_grad`) varies with pose.
3. A first-cut **frequency response** (Bode-style amplitude/phase) per joint
   from 0.1 Hz to ~10 Hz so the implementor or downstream user can pick the
   best operating regime.
4. Repeatable across runs: same protocol, same data layout, same fitting
   pipeline, so a future agent can rerun calibration on a different robot or
   the same robot after a hardware change.

### 2.1 What "effective PD" Means (Important)

When a joint with PD gain `kp_motor` is suspended in the kinematic chain and
loaded by gravity, the linearized step-response dynamics around the operating
pose `q_0` are:

```text
I_eff · q̈ + kd_eff · q̇ + (kp_motor + g_grad) · (q − q_target) = 0
```

where `g_grad ≡ ∂τ_gravity(q)/∂q | q=q_0`. So a step response measurement
fits `(I_eff, kd_eff, kp_motor + g_grad)`, **not `kp_motor` alone**. The
combined term is what governs joint behaviour at this pose; we call it
`kp_eff` and write it directly into the sim configuration.

This is **the correct quantity to put into Isaac and MuJoCo** as long as both
sims are computing their own gravity (they do, because they use the same
URDF inertials). The sims add their own `g_grad` from forward kinematics on
top of `kp_motor`, so if we put `kp_eff` into them instead of `kp_motor`,
gravity gets double-counted.

Two options to avoid double-counting:

- **Option X (recommended)**: don't try to recover `kp_motor`. Use the fitted
  `kp_eff` directly and **turn off** sim gravity for the joints, OR put the
  `kp_eff` into the actuator PD but tell the sim's link masses to be zero
  (artificial). Bad — breaks other physics.
- **Option Y (recommended)**: use `kp_motor` (the URDF-equivalent gain) in
  sim, let sim compute its own `g_grad` from URDF inertials and gravity. To
  recover `kp_motor` from the measurement, either
  - **(Y1) trust URDF**: compute `g_grad_URDF` analytically from the URDF
    inertials at the calibration pose, then `kp_motor = kp_eff − g_grad_URDF`.
    Cheap; only as good as URDF mass / CoM accuracy.
  - **(Y2) gravity compensation during calibration**: use the SDK's `tau_ff`
    field to inject a real-time gravity torque (computed via MuJoCo's
    `mj_rne` on the URDF). The measurement then sees pure motor + chain
    inertia, no gravity. Fitted `kp_eff` ≈ `kp_motor`. Most rigorous.

This spec defaults to **Y1 (trust URDF + subtract analytically post-fit)**
because it does not require running MuJoCo inside the SDK calibration loop.
Y2 is documented in §5.5 as an optional upgrade.

## 3. Non-goals

- Calibrating the motor alone (not possible without disassembly).
- Identifying nonlinear effects: backlash, stiction, cogging beyond what shows
  up at signal amplitudes ≥ 0.1 rad. A separate spec can cover those.
- Calibrating the IMU. Already covered by `imu_*` HAL backend; this tool only
  uses IMU if available, and only for sanity (drift, sign).
- Closed-loop walking control. This is a static-pose calibration only.

## 4. Physical Setup (assumed before the tool runs)

The operator must complete these steps before invoking the tool:

1. **Suspend the robot in a harness** so the feet are 3-5 cm above the ground.
   The harness should grip the torso, not the legs. The robot must be free to
   move its legs without ground contact for the duration of the measurement.
2. Confirm USB-FTDI serial connections per `MOTOR_PORT_MAP.md`. All four
   FTDI channels visible at `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if*-port0`.
3. Connect a wired e-stop / power-down switch within reach. The tool will
   exercise individual joints with non-trivial amplitudes (≥ 0.1 rad steps).
4. Verify the IMU backend boots cleanly (`imu_backend.h::IIMUBackend`). If
   unreliable, the tool must still run — IMU is optional input only.
5. Confirm enough clearance around the robot: each joint will move ±0.15 rad
   from MGTO during sine sweep. No nearby obstacles.

## 5. Test Protocols

All tests run **one joint at a time**. While joint `j` is moving, all other
joints are held with `kp = kp_hold` (default 80) and `kd = kd_hold` (default 2)
at their MGTO target.

**Tick rate**: the tool MUST default to `1 / control_dt` read from the SDK's
config (currently `control_dt = 0.015 s` → 66.67 Hz). This is the rate the
deployed policy runs at. Calibrating at a different rate produces a
discrete-time PD model that doesn't match what the policy actually
experiences at deploy.

The tool MAY accept a `--tick-hz <Hz>` override for diagnostic comparisons.
When override is used, print a banner-level warning to stdout:

```text
WARNING: tick rate overridden to <X> Hz. Deploy uses <Y> Hz from config.
The fitted PD model will be for <X> Hz; applying it to a <Y> Hz sim/deploy
will introduce a discretization mismatch. Use the default rate unless you
are specifically characterizing rate-dependent behaviour.
```

Verify the actual achieved rate via `IClock::now()` deltas and log it per
run. If the achieved rate drops below 90 % of the requested rate for more
than 500 ms (indicating the hardware backend can't sustain it), abort the
trial and surface a clear error.

### 5.1 Step Response (Test A)

Sequence per joint `j`:

| t (s) | q_target[j] | duration |
|------:|---|---|
| 0.0-1.0 | MGTO[j] | settling, baseline |
| 1.0-3.0 | MGTO[j] + amp_step | step up |
| 3.0-5.0 | MGTO[j] | step down |
| 5.0-7.0 | MGTO[j] − amp_step | step down further |
| 7.0-9.0 | MGTO[j] | return |

Defaults: `amp_step = 0.15 rad`. Run for each `kp` in
`{30, 50, 80}` and each `kd` in `{0.5, 1.0, 2.0}` for the joint under test —
the rest of the chain stays at hold gains. This gives a 3×3 = 9 step traces
per joint × 10 joints = 90 step traces.

Skip joints where the URDF limit is hit before `amp_step` reaches its full
value: clip `amp_step` to `min(amp_step, 0.7 × (limit_hi − MGTO[j]),
0.7 × (MGTO[j] − limit_lo))`.

### 5.2 Sine Sweep (Test B)

Sequence per joint `j`, per frequency `f ∈ {0.25, 0.5, 1, 2, 4, 8} Hz`:

`q_target[j](t) = MGTO[j] + amp_sin · sin(2π·f·t)`

for 5 full periods, then 1 s of MGTO settle between frequencies.

Default `amp_sin = 0.10 rad`. Same fixed `kp = 30, kd = 1.0` (the values used
inside `walk_v2` for hip_roll-class joints — see `QMINI_STIFFNESS` and
`QMINI_DAMPING`). Hold-pose gains for other joints as in 5.1.

Output: per (joint, frequency), the amplitude ratio `|Q_meas(f)| / |Q_cmd(f)|`
and the phase lag in degrees. This is the input to a Bode plot.

### 5.3 Chirp (Test C)

Per joint, run one continuous logarithmic chirp:

`f(t) = 0.25 · (40)^(t/T)`     (0.25 → 10 Hz over `T = 30 s`)

`q_target[j](t) = MGTO[j] + amp_chirp · sin(2π · ∫f(τ)dτ)`

Default `amp_chirp = 0.08 rad`. Same hold-pose convention. Per joint, one
trace, 30 s long.

This gives a continuous frequency-response estimate via FFT cross-spectrum.
It is the cheapest test in operator time. Make it the default if only one
test type can be run; Test A is the next-most-informative.

### 5.4 Multi-pose repeat (Test D)

Repeat Test A (step) at three poses:

1. MGTO at `base_z = 0.40` (default — already covered above).
2. Crouched: `base_z = 0.36`, joint targets from
   `QMINI_REF_JOINT_POSES_BY_Z[1]` in `qmini_lab/source/qmini_lab/assets/q1/constants.py`.
3. Stand-tall: `base_z = 0.44`, joint targets from
   `QMINI_REF_JOINT_POSES_BY_Z[5]`.

Purpose: pose-dependent variation in fitted `kp_eff` and `I_eff` exposes how
sensitive the operating-point dynamics are to chain configuration. The
horizontal-axis joints (`hip_pitch`, `knee`, `ankle_pitch`) carry significant
gravity load that **changes** with pose; if `kp_eff` varies by ≥ 15 % across
the three poses for one of those joints, single-point calibration is not
enough and the sim should either compensate gravity at runtime or use a
pose-interpolated PD table.

For vertical-axis joints (`hip_yaw_l/r`) the gravity gradient is near zero
regardless of pose, so we expect their `kp_eff` to be near-constant across
Test D — a useful sanity check on the rig and the fitting pipeline.

### 5.5 Optional: Real-time Gravity Compensation (Y2)

If the fitting in §8 shows that `kp_eff` is far from `QMINI_STIFFNESS` (e.g.
> 50 % deviation) or varies > 25 % across poses in Test D, the URDF inertials
are likely the dominant source of error. In that case rerun Tests A-C with
real-time gravity compensation:

1. Pre-build a tiny MuJoCo model from the same URDF as the sims:

   ```cpp
   mjModel* m = mj_loadXML("qmini_q1.xml", nullptr, nullptr, 0);
   mjData*  d = mj_makeData(m);
   ```

2. Each control tick, before sending `cmd`:

   ```cpp
   for (int j = 0; j < 10; ++j) {
       d->qpos[7 + j] = state.q[j];      // skip 7 free-base dofs
       d->qvel[6 + j] = state.dq[j];
   }
   d->qvel[6:] = 0;                      // zero accel for static gravity
   mj_inverse(m, d);                      // qfrc_inverse[] = required tau
   for (int j = 0; j < 10; ++j) {
       cmd.tau_ff[j] = d->qfrc_inverse[6 + j];
   }
   ```

3. With `tau_ff` carrying the gravity load, the motor PD only needs to react
   to the test perturbation. Fitted `kp_eff` ≈ `kp_motor`. Compare with the
   no-compensation results to bound URDF inertial error.

This adds a MuJoCo dependency to the SDK build. Keep it behind a CMake flag
(`PD_CALIBRATION_WITH_GRAVITY_COMP=ON`) so the default build stays minimal.

## 6. SDK API Required

The tool runs against the existing HAL. No new C++ HAL methods are needed.
What is needed is one new top-level binary or Python script that does the
following inside a `main()`-style entry point:

```cpp
// Pseudocode — adapt to your language of choice.
using namespace qmini::hal;

auto motors = make_motor_backend(hardware_cfg);
auto imu    = make_imu_backend();       // optional, may be nullptr
auto clock  = make_clock();
motors->start();
if (imu) imu->start();

MotorCmdFrame cmd{};
fill_with_mgto(cmd.q_target);
fill_uniform(cmd.kp, kp_hold);
fill_uniform(cmd.kd, kd_hold);
// dq_target = 0, tau_ff = 0

// 1) Hold MGTO for warm-up_s seconds.
clock->sleep_for(warm_up_s);

// 2) Per joint, per test, per pose, run the trajectory:
for (int j = 0; j < 10; ++j) {
    for (auto& trial : trials_for(j)) {
        record_t0 = clock->now();
        for (each tick at 200 Hz over trial.duration) {
            t = (clock->now() - record_t0);
            update_cmd_for_trial(cmd, j, trial, t);
            motors->send(cmd);
            auto state = motors->read();
            auto base  = imu ? imu->read() : BaseStateFrame{};
            append_sample(trial.log, t, cmd, state, base);
        }
        save_trial_log(trial.log, output_path(joint=j, trial=trial));
    }
    // Return to MGTO and rest before next joint
    cmd.q_target = MGTO;
    motors->send(cmd);
    clock->sleep_for(rest_between_joints_s);
}

motors->stop();
if (imu) imu->stop();
```

### Hard requirements

- Use **only** the existing `IMotorBackend`, `IIMUBackend`, `IClock` factories
  via `user/hal/factory.h`. Do not add new HAL methods.
- Tick rate defaults to `1 / control_dt` read from the SDK's config
  (currently 66.67 Hz). `--tick-hz` override allowed but requires the
  banner warning in §5. Verify via `IClock::now()` deltas, log the actual
  achieved rate per run, and abort if it drops below 90 % of the requested
  rate for >500 ms.
- Always send the **whole 10-joint command frame**; do not partial-update.
  Joints not under test stay at hold gains and MGTO target.
- After all trials, the final command must restore MGTO with hold gains and
  hold for 2 s before `motors->stop()`. Otherwise the robot drops abruptly.
- Implement an emergency-stop watchdog: if `motors->read()` reports any joint
  velocity > `safe_dq_max = 8 rad/s` for two consecutive ticks, abort the
  current trial, return to MGTO, log the abort, and continue with the next
  trial.

### Soft requirements

- Print one line per trial: `[joint=j test=A kp=30 kd=1.0] start`,
  `… stop ok` or `… stop aborted`.
- The tool must run unattended after the operator confirms the harness setup
  with `--i-have-checked-the-harness`. Without that flag, refuse to start.

## 7. Data Storage

Output root: `data/pd_calibration/<run_id>/` where `run_id` is `YYYY-MM-DD_HH-MM-SS_<short_label>`.

Layout:

```text
data/pd_calibration/2026-05-25_15-30-00_initial/
├── manifest.json
├── run_meta.json
├── joint_00_hip_yaw_l/
│   ├── A_kp30_kd0.5_step.npz
│   ├── A_kp30_kd1.0_step.npz
│   ├── ...
│   ├── B_kp30_kd1.0_sine_0.25Hz.npz
│   ├── ...
│   ├── C_chirp.npz
│   └── D_pose0_kp30_kd1.0_step.npz   # multi-pose only if Test D enabled
├── joint_01_hip_roll_l/
│   ...
├── joint_09_ankle_pitch_r/
│   ...
└── log.txt
```

Each `.npz` contains arrays of shape `(N,)` where `N = duration_s × tick_rate`:

| key | dtype | units | description |
|---|---|---|---|
| `t_s` | float64 | s | timestamp relative to trial start |
| `q_target` | float32, (N, 10) | rad | full 10-joint command |
| `dq_target` | float32, (N, 10) | rad/s | full 10-joint vel command |
| `kp` | float32, (N, 10) | N·m/rad | per-joint kp at each tick |
| `kd` | float32, (N, 10) | N·m·s/rad | per-joint kd at each tick |
| `tau_ff` | float32, (N, 10) | N·m | per-joint feedforward torque |
| `q` | float32, (N, 10) | rad | measured joint pos |
| `dq` | float32, (N, 10) | rad/s | measured joint vel |
| `tau_est` | float32, (N, 10) | N·m | estimated joint torque |
| `imu_rpy` | float32, (N, 3) | rad | optional, set to NaN if no IMU |
| `imu_omega` | float32, (N, 3) | rad/s | optional |
| `imu_acc` | float32, (N, 3) | m/s² | optional |
| `joint_under_test` | int (scalar) | — | index 0-9 of `j` for this trial |
| `trial_label` | str (scalar) | — | e.g. `"A_kp30_kd1.0_step"` |
| `mgto_pose` | float32, (10,) | rad | target MGTO joint values used |
| `tick_rate_hz` | float64 (scalar) | Hz | actual measured tick rate |

`manifest.json` lists every `.npz` file and the trial metadata. Schema:

```json
{
  "run_id": "2026-05-25_15-30-00_initial",
  "robot": "Qmini Q1",
  "sdk_commit": "<git sha at run time>",
  "joint_names": ["hip_yaw_l", "hip_roll_l", ...],
  "mgto_pose": [0.2346, -0.0372, ...],
  "trials": [
    {
      "joint": 0,
      "joint_name": "hip_yaw_l",
      "test": "A",
      "pose": "MGTO",
      "kp": 30.0,
      "kd": 1.0,
      "freq_hz": null,
      "amplitude_rad": 0.15,
      "duration_s": 9.0,
      "tick_rate_hz_target": 200.0,
      "file": "joint_00_hip_yaw_l/A_kp30_kd0.5_step.npz",
      "result": "ok"
    },
    ...
  ]
}
```

`run_meta.json` records the operator-supplied context:

```json
{
  "run_id": "2026-05-25_15-30-00_initial",
  "operator": "<name>",
  "harness_checked": true,
  "ambient_temp_c": 24.0,
  "battery_voltage_v": 25.6,
  "notes": "first calibration after foot-box change",
  "tool_version": "1.0.0"
}
```

`log.txt` is the raw stdout of the tool for debugging.

## 8. Fitting / Output

A **separate** offline script (Python, no SDK dependency) reads
`data/pd_calibration/<run_id>/` and produces:

`data/pd_calibration/<run_id>/calibration.yaml`:

```yaml
run_id: "2026-05-25_15-30-00_initial"
joint_names: ["hip_yaw_l", ...]
gravity_comp_used: false  # true if Y2 was active during this run

# Operating-point effective PD at MGTO (see §2.1).
# kp_eff = kp_motor + g_grad(MGTO).
per_joint:
  hip_yaw_l:
    kp_eff: 31.2          # operating-point effective stiffness at MGTO
    kd_eff: 1.18          # operating-point effective damping (incl. URDF damping)
    I_eff:  0.0072        # effective rotational inertia at the joint
    g_grad_urdf: 0.0      # analytical gravity gradient from URDF at MGTO
    kp_motor: 31.2        # = kp_eff - g_grad_urdf (= kp_eff if Y2 was used)
    omega_n_hz: 10.5      # natural frequency
    zeta: 0.42            # damping ratio
    quality_of_fit: 0.93  # R^2 on step response
    bode:                 # frequency response from Test C
      f_hz:   [0.25, 0.5, 1, 2, 4, 8]
      amp_db: [...]
      phase_deg: [...]
  hip_roll_l:
    ...
# Pose variation summary (from Test D, if enabled)
multi_pose:
  hip_yaw_l:
    kp_eff: {pose_MGTO: 31.2, pose_crouch: 31.4, pose_tall: 31.1}
    I_eff:  {pose_MGTO: 0.0072, pose_crouch: 0.0070, pose_tall: 0.0073}
    g_grad_variation_pct: 1.0   # 100*(max-min)/mean across poses
  knee_l:
    kp_eff: {pose_MGTO: 21.5, pose_crouch: 26.8, pose_tall: 18.3}
    I_eff:  {pose_MGTO: 0.0145, pose_crouch: 0.0163, pose_tall: 0.0128}
    g_grad_variation_pct: 39.5  # large variation → pose-dependent
  ...
```

The fitting model is the standard second-order joint dynamics, **with no
gravity term** because §2.1 absorbs the gravity gradient into `kp_eff`:

```text
I_eff · q̈(t) + kd_eff · q̇(t) + kp_eff · (q(t) − q_target(t)) = 0
```

Fit `(I_eff, kd_eff, kp_eff)` per joint by minimizing the L2 error between
measured `q(t)` and the simulated solution under the same `q_target(t)`. Use
Test A (steps) as the primary data and Test B/C as confirmation.

### 8.1 Recovering `kp_motor` from `kp_eff`

To plug into Isaac and MuJoCo (which compute gravity themselves), the offline
script needs `kp_motor`, not `kp_eff`. Two paths:

- **(default, Y1)**: load the same URDF into a one-shot MuJoCo, set joints
  to the calibration pose, run `mj_inverse` with zero acceleration to get
  `g_grad_urdf` per joint (numerically: `(τ(q_0 + ε) − τ(q_0 − ε)) / (2ε)`
  for small `ε ≈ 0.01 rad`). Then `kp_motor = kp_eff − g_grad_urdf`.
- **(Y2 was used)**: `kp_motor = kp_eff` directly. `g_grad_urdf = 0` in the
  yaml.

Either way, **the sim configuration uses `kp_motor`**, not `kp_eff`. The
sims add their own gravity gradient at runtime.

### 8.2 Damping handling

`kd_eff` from the fit is the **total** damping seen at the joint, which is
`kd_motor (PD)` plus URDF `<dynamics damping=...>` plus chain friction
contributions. To decompose:

- For Isaac actuator damping: `QMINI_DAMPING[name] = kd_eff - URDF_damping_in_chain_at_MGTO`.
  Usually `URDF damping ≈ 1 N·m·s/rad` per joint; check the per-joint values
  in `q1.urdf`.
- For MuJoCo PD `kds`: same as Isaac — MuJoCo also respects URDF damping
  natively.

The "× 5" hack currently in `V2MuJoCoCfg.kds` is a pre-calibration fudge.
After calibration, set `MuJoCo kds = QMINI_PD_DAMPING` directly (no × 5).

## 9. Acceptance Criteria

The implementing agent has succeeded when **all** of the following hold:

1. Running the tool with `--i-have-checked-the-harness` on a real Qmini in
   harness completes without operator intervention in under 25 minutes for
   the default protocol (Tests A + B + C, one MGTO pose).
2. The output directory contains exactly the layout in §7, with every `.npz`
   passing the schema check (all required keys, correct dtype, correct shape).
3. The watchdog aborts cleanly and returns the robot to MGTO when forced
   (test by manually wiggling a leg to drive `dq` past `safe_dq_max`).
4. The fitting script produces a `calibration.yaml` with the schema in §8.
   For **vertical-axis joints** (`hip_yaw_l/r`), `kp_motor` (after gravity
   removal) is within ±25 % of `QMINI_STIFFNESS`. For other joints, both
   `kp_eff` and the recovered `kp_motor` are reported; deviation > 50 %
   triggers a warning + manual inspection (see §8.1).
5. Re-running the same protocol twice in succession (under the same harness
   setup) gives `kp_eff` values within ±5 % between the two runs.
6. For vertical-axis joints (`hip_yaw_l/r`), the multi-pose Test D shows
   `kp_eff` variation < 10 % across the three poses (because their `g_grad`
   is near zero regardless of pose). If their variation exceeds 10 %, the
   harness or the fitting pipeline is suspect — investigate before trusting
   the horizontal-axis numbers.
6. The tool refuses to run without the harness flag, and prints a short
   safety reminder including "feet 3-5 cm above ground, e-stop reachable."
7. The README under `data/pd_calibration/README.md` (which the implementing
   agent creates) explains how to consume `calibration.yaml` from
   `qmini_lab/source/qmini_lab/assets/q1/constants.py` to update the
   `QMINI_STIFFNESS` / `QMINI_DAMPING` / `QMINI_PD_DAMPING` dicts.

## 10. Build / Integration

Add a new build target `pd_calibration_tool`:

```text
source/user/calibration/
├── pd_calibration_main.cpp
├── pd_calibration_loop.cpp
├── pd_calibration_loop.h
└── pd_calibration_trials.h
```

Wire it into `CMakeLists.txt` next to the existing user-facing targets. The
binary should link the same HAL libraries as `qmini_app`. Do not add new HAL
methods — the existing `IMotorBackend::send/read` and `IIMUBackend::read` are
sufficient (see §6).

Add the fitting script under `tools/calibration_fit/` (Python, requires
`numpy`, `scipy.optimize.minimize`, and `pyyaml`).

## 11. Things The Implementing Agent Should NOT Do

- Do not change anything in `qmini_lab` automatically. The output is a YAML
  file that the operator reviews before manually editing
  `constants.py`. Auto-editing constants is too easy to get wrong.
- Do not skip the watchdog. The motors are powerful enough to damage cables
  and themselves if a joint runs away.
- Do not run any test that commands `|q_target − q_meas| > 0.5 rad`.
- Do not assume IMU is available. If the IMU backend factory returns null,
  set the corresponding `.npz` fields to NaN and continue.
- Do not infer or guess gear ratios from the data. Use the constants in
  `motor_unitree.cpp` (`kSpeedRatio = 6.33`, `kGearRatio = 3.0` for hip_roll
  only). Those are the canonical values for this hardware.
- Do not commit `.npz` calibration data to git. The data dir lives outside
  git or in a gitignore'd path. Add a `.gitignore` entry for
  `data/pd_calibration/*/` and document in `README.md`.
- **Do not plug `kp_eff` directly into the sim.** Use `kp_motor` (after
  removing the URDF gravity gradient — see §8.1). Both Isaac and MuJoCo
  compute their own gravity from the URDF inertials; using `kp_eff` would
  double-count gravity at the operating pose. The yaml output is structured
  to make this hard to get wrong, but it is worth a banner-level warning in
  the fitting script's stdout summary.

## 12. Suggested Implementation Order

1. Scaffold the directories under `source/user/calibration/` and
   `tools/calibration_fit/`. Wire build target. Make the binary compile and
   print "harness flag missing" without doing anything.
2. Add the inner control loop with hold-MGTO behavior, no perturbations.
   Confirm it can hold the robot in harness at MGTO for 10 s and read back
   stable `q ≈ MGTO`.
3. Implement Test A (step). Run it on **one joint only** (hip_yaw_l)
   end-to-end. Inspect the resulting `.npz`. Confirm the step shape looks
   right and the timestamps make sense.
4. Add the watchdog. Force-trigger it once by commanding a 1 rad step on a
   small-amplitude joint, confirm abort works.
5. Roll out to all 10 joints. Add Test B (sine sweep).
6. Add Test C (chirp). Add Test D (multi-pose) as an optional flag.
7. Implement the offline fitting script. Validate on the step-response data.
8. Run all 10 joints end-to-end. Confirm under-25-minute runtime. Confirm
   reproducibility between two back-to-back runs.

## 13. Pointers

- `MOTOR_PORT_MAP.md` — which motor is on which FTDI port. Required for
  understanding why `IMotorBackend` is implemented as 4 parallel serial
  threads.
- `source/user/hal/hardware/motor_unitree.cpp` — the canonical PD send/recv
  loop. Mirror its update-rate behaviour exactly.
- `include/user/hal/types.h` — the only data types you exchange across the
  HAL boundary.
- `qmini_lab/source/qmini_lab/assets/q1/constants.py` — current nominal
  `QMINI_STIFFNESS / DAMPING / PD_DAMPING` to compare against fitted values.
- `qmini_lab/docs/sim2sim_trace_findings_2026-05-25.md` — the analysis that
  motivates this calibration.

## 14. Open Questions

If any of the following changes how the spec should be implemented, surface
the question back to the operator before writing code:

- Does the harness allow the robot to settle at MGTO with the feet truly
  unloaded? If not, the fitted `kp_eff` will include some residual contact
  load and the gravity-gradient subtraction in §8.1 will be slightly off.
- What is the actual sustained tick rate the hardware backend supports at
  the default 66.67 Hz? It should be trivial since serial loop typically
  runs > 500 Hz. If for some reason the backend can't sustain 66.67 Hz
  (e.g. serial port latency), surface the issue and either degrade
  gracefully (lower rate, longer trials) or stop and report.
- Is the IMU rigidly mounted in the torso? If it has any slop, body-rate
  measurements will be noisy. Spec uses IMU only as optional sanity input.
- How accurate are the URDF inertials? The default fitting path (Y1) treats
  them as ground truth when subtracting `g_grad_urdf`. If a previous
  measurement (e.g. CAD vs. real assembly) shows URDF mass / CoM is off by
  more than ~10 %, switch to Y2 (real-time gravity comp in §5.5) — the cost
  of one MuJoCo build in the SDK is worth it.
