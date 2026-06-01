# Static balance log — first run (2026-05-31)

Run: `bin/data/static_balance/2026-05-31_21-13-02_initial/` (`--static`,
10 s hold at MGTO, standing on the floor). Captured per
`STATIC_BALANCE_LOG_SPEC.md` for the URDF mass-distribution check, and it
doubled as the verification of the `tau_est` scaling bug.

## Health / stability

- IMU ω rms = [0.005, 0.009, 0.007] rad/s ≪ 0.05 → **stable**.
- Temps 30–34 °C, all `merror = 0`. 667 samples, 10.0 s.
- **Standing pose leans +8.9° forward** (pitch), roll −1.6°. Stable lean —
  this is the real standing equilibrium. It's consistent with the spec's
  premise (real robot balances leaning forward while MuJoCo with the URDF
  falls all the way forward → mass distributed differently than the URDF).
  This NPZ + `imu_rpy` is what the `qmini_lab` static-balance fitter
  consumes.

## `tau_est` scaling bug — CONFIRMED and FIXED

At the static equilibrium the motor torque holding each joint equals the
gravity torque, measurable two independent ways: the PD command `kp·err`,
and the motor's own estimate. With the raw `tau_motor` (= `d.tau`) now
logged, `tau_motor × ratio` matches `kp·err` on every loaded joint:

| joint | `kp·err` (gravity τ) | `tau_motor × ratio` | old `tau_est` (= d.tau/ratio) |
|---|---|---|---|
| knee_l | −3.59 | −3.76 | −0.094 (≈ /6.33²) |
| knee_r | +3.49 | +3.62 | +0.090 |
| ankle_l | +2.11 | +2.15 | +0.053 |
| ankle_r | −2.28 | −2.40 | −0.060 |
| hip_roll_l | +1.62 | +1.59 | +0.004 (≈ /18.99²) |
| hip_roll_r | −0.63 | −0.67 | −0.002 |

`tau_motor × ratio` ≈ `kp·err` within ~5% (friction-level); the old
`tau_est = d.tau/ratio` was low by exactly `ratio²` (×40 normal, ×360
hip_roll). **Fix applied:** `motor_unitree.cpp` now reports
`tau_est = d.tau × ratio`. `tau_est` is telemetry-only (→ `joint_tau_`
getter; not in the policy obs or the command path), so this changes only
the reported/logged torque, not control.

### Note on resolution

At these *standing* loads (2–4 N·m) the corrected torque estimate is
**good** (matches `kp·err` to ~5%). It was only too coarse for the tiny
free-release friction torques (~0.05 N·m) — that's why Test E friction ID
failed on SNR, not because the sensor is unusable. For gravity-scale
torques the corrected `tau_est` is reliable.

## Next

- Feed this run to the `qmini_lab` static-balance fitter (`kp·err` vs
  `mj_inverse`, hip→ankle) to localize the URDF mass error.
- Per spec §6.5, a single pose weakly constrains per-link mass — if the
  walk is inconclusive, re-run `--static` at 2–3 tweaked `ref_joint_act`
  poses (different knee bend) and stack the constraints.
