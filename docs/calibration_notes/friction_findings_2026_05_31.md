# Joint friction calibration — findings (2026-05-31)

Goal: per-joint viscous damping `b` and Coulomb friction `f` for the sim
(URDF `<joint damping=… frictionloss=…/>`, Isaac/MuJoCo). Two trial types
were built and run on the harnessed (base-suspended) robot:

- **Test D — free release** (`fit_passive.py`): release the joint, fit the
  passive decay. **Position-based**, no torque needed.
- **Test E — constant-velocity sweep** (`fit_friction.py`): read friction
  from `tau_est` at steady speed. **Torque-based.**

## Headline result

**Torque-based friction ID (Test E) does not work on this hardware** — the
GO-M8010-6's reported torque is unusable for friction. **Position-based
(Test D) works** and is the route.

### Why `tau_est` is unusable

1. **Scaling.** `source/user/hal/hardware/motor_unitree.cpp:135` reports
   `tau_est = d.tau / ratio`. Joint-side torque is motor torque **× ratio**,
   so as reported `tau_est` is under-scaled by ~`ratio²` (≈40× for the
   6.33-ratio joints, ≈360× for hip_roll at 18.99).

   > **CONFIRMED & FIXED (2026-05-31).** The static-balance log showed
   > `tau_motor × ratio` ≈ `kp·err` (gravity torque) on every loaded joint,
   > while old `tau_est = d.tau/ratio` was low by exactly `ratio²` — see
   > `static_balance_2026_05_31.md`. `motor_unitree.cpp` now reports
   > `tau_est = d.tau × ratio`. Safe: `tau_est` is telemetry-only (not in the
   > obs or command path).
2. **Resolution / noise.** Even accounting for scale, the current-derived
   estimate is coarse: hip_pitch_l driving the *whole leg* through 19° read
   `|tau_est|max = 0.12 N·m` (physically ~2 N·m to hold that pose). The
   hip_roll friction signature `tau(+v) − tau(−v)` was ~0.003 N·m, at the
   noise floor (std ~0.002) → `fit_friction` gave R² 0.45–0.61 and
   `f`≈0.0016 N·m (implausibly tiny for an 18.99-ratio joint). Fitting noise.

Even with the scale corrected, the SNR is marginal. `Test E` /
`fit_friction.py` are kept in the tree (they're correct given a good torque
sensor) but **flagged: needs reliable joint-torque feedback this robot lacks.**

## What we use instead

### Normal-ratio motors (the 8 joints at ratio 6.33)

All 8 share the same motor+gearbox, so one clean measurement applies to all.
From **Test D free-release on `hip_pitch_l`** (R² = 0.92, position-based):

| | value (joint-side) |
|---|---|
| viscous `b` | **0.054 N·m·s/rad** |
| Coulomb `f` | **0.023 N·m** |

Apply to all 8 normal-ratio joints (hip_yaw, hip_pitch, knee, ankle ×2).

### hip_roll (ratio 18.99 — extra 3× gear): by gear-ratio scaling

hip_roll's own free-release was rank-deficient (monotonic decay → `b`/`g`
inseparable), and Test E is out. So estimate it by reflecting the normal
motor's friction through hip_roll's higher ratio:

```
f_motor = f_normal / 6.33  = 0.00357 N·m   →  f_hiproll = f_motor · 18.99  ≈ 0.068 N·m
b_motor = b_normal / 6.33² = 0.001348      →  b_hiproll = b_motor · 18.99² ≈ 0.486 N·m·s/rad
```

**Caveats (treat as order-of-magnitude):**
- This is a **lower bound** — it reflects only the *motor's* friction through
  the gear. The extra gear *stage itself* adds Coulomb/viscous friction that
  this scaling ignores, so real hip_roll friction is **higher**.
- Inherits any error in the `hip_pitch_l` `b`/`f` (which depend on the Test A
  inertia estimate).
- To measure hip_roll directly later: a **larger** free-release displacement
  (≈0.25–0.3 rad, away from gravity equilibrium) to try for an *oscillatory*
  swing (makes `b` separable); if it stays overdamped, scaling is the
  fallback.

## Inertia

`I` is **per-joint, from the URDF inertials** (the leg mass each joint
carries) — not a motor property, not measured here.

## Suggested sim values

URDF `<joint>` fields are joint-side, so use the joint-side numbers above:

| Joint group | `damping` (b) | `frictionloss` (f) | confidence |
|---|---|---|---|
| 8 normal-ratio joints | 0.054 | 0.023 | good (R²=0.92, one motor) |
| hip_roll_l / hip_roll_r | ~0.49 | ~0.07 | rough (scaled, lower bound) |

Review before applying to `qmini_lab` — per project policy, the operator
edits `constants.py` / the URDF by hand.
