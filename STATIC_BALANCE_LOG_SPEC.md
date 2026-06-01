# Static Balance Log Spec

Date: 2026-05-31

Spec for a tiny SDK tool — `static_balance_log_tool` — that records the
real robot in a stable standing pose and exports the per-joint
equilibrium variables we need to validate URDF mass distribution via
static inverse-dynamics in the sim repo.

This file is the source of truth for the implementing agent. Read it
once, then build the tool that fully satisfies "Acceptance criteria"
below. Companion to `PD_CALIBRATION_SPEC.md`,
`FREE_RELEASE_CALIBRATION_SPEC.md`, and
`BODY_EXCITATION_SPEC.md`; shares the same SDK code paths
(`hal::IMotorBackend`, `hal::IImuBackend`, `npz_writer`).

---

## 1. Why

`ref_calibration_tool` confirmed the real robot stands stably at the
calibrated MGTO under pure PD (motor-side `cfg.kp`/`cfg.kd`, no balance
loop). The same model in MuJoCo (URDF + same MGTO + same PD) falls
forward to +73° within 1 second regardless of which knob is tuned
(armature, damping, friction, foot box size, contact stiffness, kp
scale up to 20×, CoM offset across full range tried).

This points to a structural mismatch between the URDF's mass
distribution and the real robot's mass distribution. A static-dynamics
inverse check on real-robot data isolates exactly where the URDF is
wrong.

**Math:** at the standing equilibrium, each joint satisfies
$$\tau_\text{motor}(j) + \tau_\text{gravity}(j) = 0$$
where the motor torque under PD is
$$\tau_\text{motor}(j) = k_p(j) \cdot (q_\text{target}(j) - q_\text{read}(j)) - k_d(j) \cdot \dot{q}(j)$$
At steady state $\dot{q} \approx 0$, so the position error directly
encodes the gravity-induced joint torque the URDF should predict.

In the sim repo, we'll compute $\tau_\text{gravity,predicted}(j)$ by
running `mj_inverse` on the URDF at $(q_\text{read}, \text{base pose})$
and compare to the measured PD torque. Per-joint mismatch, walked
hip→ankle along the kinematic chain, localizes where the URDF mass
inertia is wrong (a discrepancy at joint $j$ implies the cumulative
mass downstream of $j$ is off).

## 2. What we measure

For a single stable standing pose, log per-tick:

- `t_s` (float64, N)
- `q_target[N, 10]` — commanded MGTO from `cfg.ref_joint_act`
- `q_read[N, 10]` — encoder positions
- `q_dot[N, 10]` — encoder velocities (should be ≈0 at steady state)
- `kp[N, 10]`, `kd[N, 10]` — gains in effect (records `cfg.kp/kd`)
- `tau_est[N, 10]` — motor torque estimate, diagnostic only (HAL has a
  known scaling bug per `friction_findings_2026_05_31.md`; we'll
  cross-check with `kp · err` instead)
- `imu_rpy[N, 3]`, `imu_omega[N, 3]`, `imu_acc[N, 3]` — base
  orientation (used to set the sim's base orientation in `mj_inverse`)
- `dynamic_zero[10]` — startq offsets in effect (`cfg.startq`)

A single trial:

```
t = 0..ramp_s    : ramp from measured pose to MGTO with cfg PD
t = ramp..ramp+settle_s : hold MGTO under full PD until IMU ω < 0.05 rad/s
t = (then)..duration_s  : steady-state logging window (N samples)
```

Defaults: `ramp_s = 3`, `settle_s = 3`, `duration_s = 10`. At
`control_dt = 0.015 s` (66.67 Hz), the steady-state window is ~666
samples.

NPZ filename: `<run_dir>/static_balance_log.npz`.

Manifest entry:
```json
{
  "trial": "static_balance",
  "duration_s": 10.0,
  "settle_s": 3.0,
  "tick_rate_hz_actual": 66.39,
  "n_samples": 663,
  "imu_rpy_mean_deg": [0.5, -1.2, 0.0],
  "imu_omega_rms_rad_s": [0.02, 0.03, 0.01],
  "file": "static_balance_log.npz",
  "result": "ok"
}
```

## 3. SDK changes

A new top-level tool `static_balance_log_tool`, **separate** from
`pd_calibration_tool` because:
- the trial is fundamentally different (passive hold, no perturbation)
- a per-protocol per-tool layout matches the existing pattern
  (`pd_calibration_tool`, `body_excitation_tool`, this one)

### New files

```
include/user/calibration/static_balance.h
source/user/calibration/static_balance.cpp
source/user/calibration/static_balance_main.cpp
```

### `static_balance.h`

```cpp
namespace qmini::calib::staticb {

struct LogOptions {
    double tick_hz       = 66.6667;
    double ramp_in_s     = 3.0;
    double settle_s      = 3.0;
    double duration_s    = 10.0;
    double settle_omega_rms_max = 0.05;   // rad/s; tighter abort if not met
    float  kp_hold[kNumJoints]{};         // populated from cfg.kp
    float  kd_hold[kNumJoints]{};
    bool   verbose = true;
    std::string output_dir;
};

struct LogResult {
    std::string file;
    std::string result;
    double tick_rate_hz_actual = 0.0;
    int    n_samples = 0;
    std::array<double, 3> imu_rpy_mean_deg{};
    std::array<double, 3> imu_omega_rms_rad_s{};
};

class StaticBalanceLogger {
public:
    StaticBalanceLogger(hal::IMotorBackend* motor,
                        hal::IImuBackend*   imu,
                        const LogOptions&   opts,
                        const PoseRef&      mgto_pose);
    LogResult run();
    void stop();
private:
    /* ... */
};

}  // namespace
```

### `static_balance_main.cpp`

Same skeleton as `pd_calibration_main.cpp` / `ref_calibration_main.cpp`:

- Safety gates: `--i-have-checked-the-harness` for hardware backend
- Same backend factory (sim / mujoco / hardware)
- MGTO confirmation prompt (operator verifies pose looks correct
  before logging starts)
- Output: `data/static_balance/<run_id>/` with `static_balance_log.npz`,
  `manifest.json`, `run_meta.json`, `log.txt`

CLI options:
- `--duration <s>` (default 10)
- `--settle <s>` (default 3)
- `--ramp-in-s <s>` (default 3)
- `--label <str>` (default `initial`)
- `--operator <str>`, `--notes <str>` for run_meta.json

### Behaviour during logging

1. Limp prime (200 ms) — read fresh motor state.
2. Smooth cosine ramp from current pose to MGTO over `ramp_in_s`.
3. MGTO confirmation gate (operator types `y`).
4. Settle phase: hold MGTO until **either** (a) `omega_rms` over a
   500 ms window drops below `settle_omega_rms_max`, **or**
   (b) `settle_s` elapses.
5. Log phase: record every tick for `duration_s` to in-memory buffer.
6. Cool-down + fold (smooth gain release to limp).
7. Write NPZ + manifest.

## 4. Fit side (in qmini_lab repo, not part of this PRD)

For reference — the sim agent will implement
`qmini_lab/scripts/static_balance_fit.py`:

1. Load NPZ from SDK run.
2. Average `q_read`, `q_dot`, `q_target`, `imu_rpy` over the
   steady-state window.
3. Compute predicted gravity torque per joint via MuJoCo:
   ```python
   m = mj_load(urdf)
   d = mj_makeData(m)
   d.qpos[7:17] = q_read_avg                    # joint angles
   d.qpos[3:7] = quat_from_rpy(imu_rpy_avg)     # base orientation
   d.qpos[:3]  = (0, 0, settle_z)               # base position (any)
   d.qvel[:]   = 0
   mj_inverse(m, d)
   tau_grav_predicted = d.qfrc_inverse[6:16]
   ```
4. Compute observed PD torque: `tau_pd = kp·(q_target - q_read) - kd·q_dot`.
5. Per-joint discrepancy: `delta[j] = tau_pd[j] - (-tau_grav_predicted[j])`.
6. Walk the chain hip→ankle, the first joint with a large `delta` tells
   us which downstream link's mass/CoM/inertia is wrong.

Output: `static_balance_fit_<run_id>.yaml` with per-joint discrepancy,
plus a recommendation of which URDF link(s) to revise.

## 5. Acceptance criteria

1. `./static_balance_log_tool --i-have-checked-the-harness --duration 10`
   on hardware backend runs end-to-end without abort.
2. NPZ contains exactly the fields listed in §2, sized correctly.
3. `imu_omega_rms_rad_s` in manifest < 0.05 on a healthy stable
   standing pose (logged in `log.txt` if higher).
4. Total runtime ≈ ramp + settle + duration + fold ≈ 18-20 s.
5. With backend=mujoco (using `sim_assets/q1_sim.mjcf`), the tool
   runs the full protocol against the simulated robot and writes a
   matching NPZ — useful for sanity-testing the fitter in sim.

## 6. Risks / open questions

1. **Settle check on small tilt**: if real-robot residual tilt is ~1°
   but omega is small, the log still says "stable". Fine — that's the
   actual operating point. The fitter handles it by reading `imu_rpy`.
2. **`tau_est` HAL bug**: the SDK's reported torque is off by ~`ratio²`
   (see `friction_findings_2026_05_31.md`). The fitter doesn't trust
   `tau_est` — it uses `kp · err`. We log `tau_est` only as a sanity
   cross-check (and for future fix-then-rerun work).
3. **Pose drift during logging**: if joints slowly drift over the
   10 s window, the steady-state assumption breaks. Mitigation:
   compute `q_read_mean` over a tight late-window subset (last 3 s)
   in the fitter rather than the whole 10 s. SDK still logs the full
   window so the fitter can choose.
4. **Base anchor in sim fitter**: `mj_inverse` needs a base position
   that satisfies foot-ground contact. Easiest: set base z = nominal
   stand height and skip the freejoint dofs in the output (just read
   `qfrc_inverse[6:16]` for the 10 actuated joints). The 6 base dofs
   in `qfrc_inverse` should be ≈0 at static equilibrium — if not, the
   pose itself isn't a valid equilibrium and we should warn.
5. **Multiple poses for ID**: a single pose only constrains the
   gravity moment about the foot pivot for the present joint angles.
   To uniquely identify mass+CoM per link, ideally do this in
   2-3 different standing poses (different knee bend / hip angles)
   and stack the constraints. Out of scope for this PRD; extend
   later if a single pose isn't conclusive.

## 7. Out of scope

- Dynamic system ID via this tool — covered by Test B sine / Test C
  chirp / Test D free release / body excitation. This one is
  static-only.
- Foot-contact-force calibration — needs FSR or load cell; not
  measured by this tool.
- Automatic URDF rewriting — the fitter outputs a per-joint
  discrepancy + recommendations; the operator edits the URDF by
  hand per project policy.
