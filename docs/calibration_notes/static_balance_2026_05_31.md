# Static balance log (2026-05-31)

Two `--static` runs (10 s MGTO hold). **Use the hard-floor run for the
URDF mass-distribution check**; the first run was on carpet and is not
valid for it.

| Run | Surface | pitch / roll | Valid for mass check? |
|---|---|---|---|
| `2026-05-31_21-13-02_initial` | **carpet** | +8.9° / −1.6° | ❌ soft, tilting contact |
| `2026-05-31_22-57-48_initial_floor` | **hard floor** | **−2.0° / −0.6°** | ✅ |

## Health / stability

- Both runs stable (IMU ω rms ~0.007 rad/s ≪ 0.05), temps 30–34 °C,
  `merror = 0`, 667 samples / 10 s, no drift.
- **The +8.9° "forward lean" on carpet was a CARPET ARTIFACT** — the
  compliant surface let the feet sink/tilt forward, so the feet weren't
  flat on rigid ground (breaks the `mj_inverse` feet-flat assumption).
  On a **hard floor the same pose holds near-level (−2.0° pitch)**, so the
  9° was not a mass signal and not a calibration error.
- The valid mass-check input is the **hard-floor** NPZ
  (`2026-05-31_22-57-48_initial_floor`) + its `imu_rpy`.

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
