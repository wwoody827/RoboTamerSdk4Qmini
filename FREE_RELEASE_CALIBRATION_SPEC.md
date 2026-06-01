# Free-Release Calibration Spec (Test D)

Date: 2026-05-31
Status: **Landed 2026-05-31.** Test D is in `pd_calibration_tool`
(`--tests D`, opt-in; default run stays A,B,C) and the fitter is
`tools/calibration_fit/fit_passive.py`. One deliberate deviation from §5:
`g` (gravity stiffness) and `I` are taken from the **same run's Test A
fit** (`g = kp_eff_A − kp_commanded`, `I = I_eff_A`) instead of MuJoCo
`mj_inverse` — so the fitter needs **no MuJoCo** and the run must contain
**both A and D** per joint (`--tests A,D`). Everything else matches.

Spec for extending `pd_calibration_tool` with a passive-dynamics
identification trial — "Test D / Free Release" — that measures per-joint
**viscous damping (b)** and **Coulomb friction (f)** independently of the
PD control loop.

This file is the source of truth for the implementing agent. Read it
once, then build the tool extension that fully satisfies "Acceptance
criteria" below. Companion to `PD_CALIBRATION_SPEC.md` (Test A/B/C);
shares the same SDK code paths and NPZ schema.

---

## 1. Why

The current Test A/B/C fit `(I_eff, kd_eff, kp_eff)` of the *closed-loop*
dynamics. The fitted `kd_eff` mixes commanded PD damping with passive
damping (bearings, gearbox friction, magnetic detent), and cannot
separate them. See `pd_calibration_2026_05_31.md` §"Damping analysis":
most joints have `kd_eff` 1.1-2.3× higher than commanded `kd`,
indicating significant passive damping that we currently can't quantify.

Knowing `b` and `f` separately is needed for:

1. **URDF `damping` and `frictionloss`** fields — propagates to both IL
   (`write_joint_damping_to_sim` + `write_joint_friction_coefficient_to_sim`)
   and MJ (`<joint damping=" " frictionloss=" "/>`). Currently both
   default to URDF nominals which are CAD guesses, not measurements.
2. **Sim2real fidelity**: passive friction dominates joint behavior at
   low speeds (stiction region). Real robot has visible stiction; sim
   without it learns fast-twitch policies that don't transfer.
3. **Decoupled PD calibration**: subtracting passive `b·q̇ + f·sign(q̇)`
   from the closed-loop fit isolates the *actual* PD-loop contribution.

## 2. What we measure

For each of the 10 joints, fit a single-DOF passive equation of motion:

$$I\,\ddot{q} + b\,\dot{q} + f \cdot \text{sign}(\dot{q}) + g\!\cdot\!(q - q_\text{eq}) = 0$$

where:
- `I` — joint moment of inertia (kg·m²), already estimated from Test A
- **`b`** — viscous damping coefficient (N·m·s/rad) ← new output
- **`f`** — Coulomb friction (N·m) ← new output
- `g · (q - q_eq)` — gravity restoring torque (joint pulls toward gravity
  equilibrium). Computed analytically from URDF + MGTO pose using
  MuJoCo's `mj_inverse`, like the existing `g_grad_urdf` in `fit_pd.py`.

## 3. Protocol per joint

```
t = 0..1 s    settle    : PD hold at MGTO (kp=cfg_kp, kd=cfg_kd)
t = 1..3 s    displace  : ramp q_target to (MGTO + 0.15 rad) over 2 s
                          (smooth half-cosine ramp; cfg PD all the way)
t = 3..3.5 s  hold      : maintain displaced position; settle transients
                          (still on full cfg PD)
t = 3.5 s     RELEASE   : instantaneously set kp[j] = 0, kd[j] = 0,
                          tau_ff[j] = 0. Keep q_target[j] = MGTO so the
                          "PD command" becomes zero torque. All other
                          joints continue holding MGTO with cfg gains.
t = 3.5..7 s  decay     : record q[j](t), dq[j](t), tau_est[j](t), IMU
                          while joint freely decays back toward its
                          gravity equilibrium under passive damping +
                          Coulomb friction.
t = 7..8 s    re-engage : smoothly ramp kp[j] / kd[j] back to cfg over
                          1 s (cosine ramp on gains). Joint smoothly
                          recovers to MGTO under PD.
```

Total per-joint duration: 8 s. 10 joints × 8 s = 80 s + per-trial
ramps + cooldown. Expected full run: ~3 minutes.

### Safety

- Other 9 joints always hold MGTO with full cfg gains during the test
  joint's release. This means the released joint can only swing within
  the local kinematic window — won't propagate destabilization.
- Watchdog: keep the existing `safe_dq_max` on the test joint. If decay
  velocity exceeds 8 rad/s the trial aborts (suggests joint flew past
  expected window).
- Joint amplitude bounded by `safe_amp` (existing helper); 0.15 rad
  default but auto-clipped to fit the joint's `act_pos_low/high` range.
- Re-engage ramp is 1 s smooth cosine — no snap back to MGTO.

### Data captured per trial

Same NPZ schema as existing trials (`npz_writer`). Key fields used by
the fit:
- `t_s` (float64, N)
- `q[:, j]` — encoder position of test joint
- `dq[:, j]` — velocity
- `tau_est[:, j]` — measured motor torque (should be near 0 during decay)
- `imu_omega` (N, 3) — base angular velocity for IMU compensation
- `kp[:, j]`, `kd[:, j]` — for verification (should be 0 during decay phase)

Manifest entry: `{"test": "D", "kp": cfg_kp, "kd": cfg_kd, "amp": 0.15,
"hold_s": 0.5, "decay_s": 3.5, "duration_s": 8.0, ...}`. The `kp/kd`
fields are recorded as the HOLD gains used in the pre-release phase.

## 4. SDK changes

### 1. `include/user/calibration/trials.h`

```cpp
enum class TestKind { Step = 'A', Sine = 'B', Chirp = 'C',
                      FreeRelease = 'D' };
```

Add to `Trial` struct nothing new (existing fields cover it):
- `amp` reused as initial displacement
- `duration_s` reused as total trial length
- `freq_hz` set to -1 (unused)

### 2. `source/user/calibration/trials.cpp`

`trial_offset(D, t)` returns the q_target offset:
```cpp
case TestKind::FreeRelease: {
    const double settle = 1.0, ramp = 2.0, hold = 0.5, decay = 3.5,
                 reengage = 1.0;
    if (t < settle) return 0.f;
    if (t < settle + ramp) {
        // Half-cosine ramp 0 → amp over `ramp` s
        const double u = (t - settle) / ramp;
        return trial.amp * 0.5f * (1.f - std::cos(M_PI * u));
    }
    if (t < settle + ramp + hold + decay) return trial.amp;  // q_target stays at amp
    // (after decay, q_target back to 0 — handled by gain ramp not q ramp)
    return 0.f;
}
```

Note: the *release* (kp/kd → 0) is handled in `loop.cpp` based on phase,
not via `q_target`. q_target stays at the displaced position during the
decay phase but kp = 0 means it has no effect.

Add to `build_default_plan`:
```cpp
// Test D: free-release passive dynamics
{
    float amp_d = safe_amp(0.15f, mgto_j, lo, hi);
    Trial t;
    t.joint = j;
    t.test = TestKind::FreeRelease;
    t.kp = kp_j;  t.kd = kd_j;
    t.amp = amp_d;
    t.freq_hz = -1.f;
    t.pose_id = 0;
    t.label = fmt_label("D", kp_j, kd_j, "release");
    t.duration_s = 8.0;
    out.push_back(t);
}
```

### 3. `source/user/calibration/loop.cpp`

In the trial loop, add per-tick gain adjustment for the test joint
based on the current phase within the trial:

```cpp
// Existing: cmd.kp[tr.joint] = tr.kp; cmd.kd[tr.joint] = tr.kd; (set once)
// Replace with per-tick logic for Test D:
float test_kp = tr.kp, test_kd = tr.kd;
if (tr.test == TestKind::FreeRelease) {
    const double settle = 1.0, ramp = 2.0, hold = 0.5, decay = 3.5,
                 reengage = 1.0;
    const double release_t = settle + ramp + hold;   // 3.5 s
    const double reengage_t = release_t + decay;     // 7.0 s
    if (t < release_t) {
        // Hold gain full — leading up to and during pre-release hold
    } else if (t < reengage_t) {
        // DECAY phase: gains = 0 → motor outputs no torque
        test_kp = 0.f;
        test_kd = 0.f;
    } else {
        // Re-engage: smooth cosine ramp 0 → cfg over `reengage` s
        const double u = (t - reengage_t) / reengage;
        const double s = 0.5 * (1.0 - std::cos(M_PI * u));
        test_kp = tr.kp * static_cast<float>(s);
        test_kd = tr.kd * static_cast<float>(s);
    }
}
cmd.kp[tr.joint] = test_kp;
cmd.kd[tr.joint] = test_kd;
```

No other changes needed — NPZ logging already records `kp[t]`/`kd[t]`
per-tick.

### 4. `source/user/calibration/pd_calibration_main.cpp`

`--tests` already accepts a list. Validate that 'D' is recognized:
```cpp
// Already accepts "A,B,C,D" by string membership; no change required.
```

User invocation: `./pd_calibration_tool --tests D --joints 0,...`

## 5. Fit-side changes

### `tools/calibration_fit/fit_passive.py` (new)

Inputs: a run directory containing Test D trials per joint.

Algorithm per joint:

1. Load NPZ trace. Identify decay window: from `release_t` (= 3.5 s) to
   `reengage_t` (= 7.0 s), with a small skip at start to avoid the
   release transient.
2. Subtract gravity equilibrium: compute `q_eq(j)` from `mj_inverse`
   applied to URDF at MGTO. Define `qbar = q - q_eq`. Use `qbar` and
   `dq` for the fit.
3. **Step 1 — Coulomb sign detection**: split decay into half-cycles by
   zero crossings of `dq`. Within each half-cycle `dq` has a constant
   sign, so Coulomb friction is a constant offset.
4. **Step 2 — Linear fit per half-cycle**: discretize
   $I\ddot{q} + b\dot{q} + g\cdot qbar = -f\cdot\text{sign}(\dot{q})$
   with `I`, `g` known (from existing Test A fit + URDF). Solve linear
   least-squares for `(b, f)` over all decay samples:

   $\begin{pmatrix} \dot{q}_0 & \text{sign}(\dot{q}_0) \\ \vdots & \vdots \end{pmatrix} \begin{pmatrix} b \\ f \end{pmatrix} = \begin{pmatrix} -I\ddot{q}_0 - g\cdot qbar_0 \\ \vdots \end{pmatrix}$

   where `q̈` is numerically differentiated from `q̇` (Savitzky-Golay
   smoothing, same as `fit_pd_imu_comp.py`).
5. **Step 3 — IMU base-motion compensation** (optional but cheap):
   subtract `I·α_base_proj` from the LHS, same as `fit_pd_imu_comp.py`.
6. Report `(b, f)` plus 95% CI from the regression covariance.

Output: `bin/data/pd_calibration/<run_id>/passive_dynamics.yaml`

```yaml
per_joint:
  hip_yaw_l:
    b_passive:    0.21   # N·m·s/rad
    b_passive_ci: [0.18, 0.24]
    f_coulomb:    0.08   # N·m
    f_coulomb_ci: [0.06, 0.10]
    I_used:       0.0168
    g_grad_used: -0.012
    r2:           0.91
  ...
```

### Existing `fit_pd_imu_comp.py` enhancement (later)

Subtract measured `b·q̇ + f·sign(q̇)` from the closed-loop residual so
the PD-loop `kp_eff` / `kd_eff` are cleaner. Not blocking for this PRD.

## 6. Acceptance criteria

1. `pd_calibration_tool --tests D --quick` runs end-to-end without
   abort on at least one joint.
2. Full `--tests D` run completes 10 joints in ~3 min, NPZs land in
   `bin/data/pd_calibration/<run_id>/joint_NN_<name>/D_*.npz`.
3. For each joint, the captured NPZ shows `kp[t] == 0` and `kd[t] == 0`
   exclusively during the `[3.5, 7.0]` s decay window.
4. `tools/calibration_fit/fit_passive.py <run_dir>` produces
   `passive_dynamics.yaml` with `b`, `f` per joint and r² > 0.7.
5. Sanity: `f_coulomb > 0` for all joints; `b_passive` within an order
   of magnitude of URDF nominal damping (~1 N·m·s/rad).

## 7. Risks / open questions

1. **Decay window too short?** Joints with high damping might decay
   within 0.5 s, leaving little fittable signal. If observed, lengthen
   `decay` or reduce pre-displacement `amp`.
2. **Other-joint hold interaction**: when test joint releases, gravity
   may slightly rotate the test joint's parent link, which couples
   through other joints under PD hold. This adds noise but shouldn't
   bias `b`/`f` significantly (IMU compensation handles it).
3. **Re-engage jerk**: 1 s cosine ramp should be smooth; if any joint
   shows a velocity spike on re-engagement, lengthen to 2 s.
4. **knee / ankle gravity load**: at MGTO the knee carries significant
   gravity torque (~2-3 N·m). The 0.15 rad displacement is asymmetric
   (one direction relieves gravity, the other adds). The
   `g · (q - q_eq)` term in the fit handles this correctly, but the
   pre-displacement should ideally test BOTH directions (±0.15 rad)
   for a symmetric measurement. Optional v2 feature.
5. **Identifiability of `b` vs `f` near zero velocity**: when `dq → 0`
   (end of decay), Coulomb friction's `sign(dq)` is ill-defined. The
   regression handles this with a small velocity threshold (e.g.,
   ignore samples with `|dq| < 0.05 rad/s`).

## 8. Out of scope

- Bi-directional Test D (release in both + and - directions per joint)
  → optional follow-up if anisotropy suspected.
- Series-elastic gearbox model (separate motor / load DOFs) → would
  give a much more accurate sim2real model but requires a different
  trial protocol with motor-side current measurement.
- Frequency-dependent damping (viscous coefficient varies with speed) →
  same as above.
