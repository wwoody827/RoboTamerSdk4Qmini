# PD Calibration — Half-Side Workflow + Multi-Frequency Identification

Date: 2026-05-29
Owner: SDK
Status: Spec (ready for implementation)
Supersedes nothing — this is an extension to `PD_CALIBRATION_SPEC.md`.

This file is the source of truth for the implementing agent. Read it
once, then build the changes that fully satisfy "Goals" and "Acceptance
criteria" below. **Do not deviate from the CLI / file conventions** —
downstream scripts (`tools/calibration_fit/`) consume them.

---

## 1. Why this addition

The first calibration run (`bin/data/pd_calibration/2026-05-29_16-06-55_initial/`)
used a single 2 Hz sine per joint with the robot **hanging from a strap**.
Analysis (`tools/calibration_fit/fit_pd_sine.py`) found two problems:

1. **Chassis recoil contaminates hip joints.**
   Hip yaw saw 27 % of the apparent joint motion coming from chassis
   rotation; hip roll saw 17 %. The IMU correction recovered usable
   numbers but with reduced confidence. Hip pitch / knee / ankle were
   clean (<11 %).
2. **One frequency is mathematically insufficient.**
   At 2 Hz we are below resonance (`ω_n ≈ 4–5 Hz` for every joint),
   so gain ≈ 1 and only phase carries information. One frequency
   gives 2 numbers; we want 3 parameters (`kp_eff, kd_eff, I_eff`).
   We cannot independently verify `kp_eff = kp_cmd` without exciting
   the response away from low frequency.

The user can mechanically constrain **one side of the robot's torso**
at a time (clamp / fix the left half, run left-leg trials; then flip
to right). This unlocks:

- Clean encoder data (no chassis swing → no IMU compensation needed).
- Ability to run a **step + chirp + multi-sine** plan per joint without
  the robot tipping over.
- L/R symmetry cross-check on the resulting fits.

## 2. Goals

1. Add CLI **half-side plans** to `pd_calibration_tool` so a single
   command runs the full identification (step + chirp + 2 Hz sine) on
   all 5 joints of one side. The user can run twice (one per side)
   without re-typing flags or per-joint loops.
2. Hardware-side: nothing changes — the operator clamps the torso
   themselves, presses go.
3. Fit-side: extend `tools/calibration_fit/fit_pd_sine.py` (or split
   into a new `fit_pd_full.py`) so it consumes step + chirp + sine
   trials and **independently identifies `kp`, `kd`, `I`** per joint.
4. Produce a `calibration.yaml` whose `per_joint.<name>` entries have
   `kp_fit` independent of `kp_cmd` (current spec returns `kp_motor =
   kp_eff − g_grad`; preserve that field, add `kp_independent_fit` for
   the chirp-identified value).

### 2.1 What changes vs the existing spec

| Aspect | Before | After |
|---|---|---|
| Pose | MGTO crouch only | Same (no change) |
| Trials per joint | step + 4-freq sine + chirp (default) | Same (no change) |
| Hardware setup | Robot hung from strap | Robot torso clamped, one side at a time |
| CLI invocation | `--quick` or per-joint manual | New `--plan side_left` / `--plan side_right` |
| IMU usage | Recorded but unused in fit | Recorded as data-quality check only (should be ~0 with torso fixed) |
| kp identification | Trust `kp_cmd` | Identify from chirp Bode |

## 3. CLI changes

### 3.1 New `--plan` values

`pd_calibration_main.cpp` already supports `--plan default` and
`--plan minimal`. Add two new values:

```
--plan side_left        # joints 0..4: hip_yaw_l, hip_roll_l,
                        # hip_pitch_l, knee_l, ankle_l
--plan side_right       # joints 5..9: hip_yaw_r, hip_roll_r,
                        # hip_pitch_r, knee_r, ankle_r
```

For each joint in the side, run **the existing default plan content**:
1× step (Test A, ±0.05 rad, 9 s including settles)
2× sine (Test B, 0.10 rad amp at 1 Hz and 2 Hz, 5 periods each)
1× chirp (Test C, 0.25 → 10 Hz log over 30 s, 0.05 rad amp)

> Amplitudes match `build_default_plan()` defaults; do **not** shrink
> them. The previous "small amp" worry was for hanging robots tipping
> — torso-clamped robots cannot tip.

### 3.2 New `--plan default_full`

Equivalent to `side_left` immediately followed by `side_right` (10
joints back-to-back). Used only if the operator does not need to
re-clamp between sides. **Do not** make this the default; the user
explicitly opts into it.

### 3.3 Preserve existing flags

`--joint <i>`, `--quick`, `--minimal`, `--no-viewer`, `--mjcf`,
`--hold-kp/kd-scale`, `--zero-on-start` all behave unchanged.

### 3.4 Run banner

When `--plan side_left` or `side_right` is selected, the startup banner
**must** print a single safety reminder:

```
======================================================================
   HALF-SIDE PLAN: assumes torso is mechanically constrained.
   IMU motion will be logged for verification. If RMS ω > 0.10 rad/s
   during any trial the run will WARN at exit but not abort.
======================================================================
```

## 4. Data quality check (during run)

After each trial, compute IMU angular velocity RMS over the trial
window (exclude first 0.5 s settle) and write to the per-trial NPZ
as a new scalar `base_omega_rms`. The fit script will use this to
decide if a joint's fit is trustworthy.

Add `imu_omega_rms` to the trial summary in `manifest.json`:

```json
{
  "joint": 3, "joint_name": "knee_l", "test": "B", ...,
  "imu_omega_rms_rad_s": 0.027,
  "result": "ok"
}
```

No abort logic. Display the value on the per-trial console line so
the operator notices if their clamp is loose.

## 5. Fit changes (offline)

Extend `tools/calibration_fit/fit_pd_sine.py` to accept all three trial
types. Rename to `fit_pd_full.py` if cleaner; keep `fit_pd_sine.py`
working for backwards-compat. The new identification:

### 5.1 From step (Test A)

Fit `(I, kd, kp)` jointly by minimizing
`Σ (q_measured(t) − q_sim(t))²` where `q_sim` integrates the 2nd-order
ODE `I·q̈ + kd·q̇ + kp·(q − q_target) = 0` forward-Euler at the
trial's dt. Window: t ∈ [0.6 s, 8.0 s]. Initial guess:
`I = 0.005`, `kd = 1.0`, `kp = kp_cmd`. Use `scipy.optimize.minimize`
Nelder-Mead with `xatol=1e-6, fatol=1e-8, maxiter=2000`.

Output: `kp_step, kd_step, I_step, r2_step, omega_n_step, zeta_step`.

### 5.2 From chirp (Test C)

Window the steady-state portion (t ∈ [0.5 s, end−0.5 s]). Compute
FFT cross-spectrum `H(jω) = Q(jω) / Q_target(jω)`. Bin at
12 log-spaced frequencies in [0.5, 10] Hz. For each bin, compute
amplitude (dB) and phase (deg).

Fit `(kp, kd, I)` to the entire Bode curve via least-squares against
the model `H(jω) = (kp/I) / (jω)² + (kd/I)(jω) + (kp/I))`. Use
`scipy.optimize.least_squares` with bounds `kp ∈ [0.5·kp_cmd, 5·kp_cmd]`,
`kd ∈ [0.01, 50]`, `I ∈ [1e-5, 1.0]`.

Output: `kp_chirp, kd_chirp, I_chirp, r2_chirp, omega_n_chirp, zeta_chirp`.

### 5.3 From 2 Hz sine (Test B)

Already implemented in `fit_pd_sine.py`. Keep as-is for cross-check.

### 5.4 Combination

Per joint, produce the final `(kp_eff, kd_eff, I_eff)`:

- If both step and chirp succeeded (`r2 > 0.85`): **report the chirp
  values** (richer frequency content) and **flag if step ≠ chirp by
  >20 %**.
- If only step succeeded: report step.
- If only chirp succeeded: report chirp.
- If neither: write `null` and `quality_of_fit: failed`.

### 5.5 calibration.yaml output

Extend the existing schema:

```yaml
run_id: 2026-05-30_HH-MM-SS_half_side
method: step+chirp+sine, torso_clamped
joint_names: [...]
per_joint:
  knee_l:
    # Independent identification (new)
    kp_fit: 44.8          # from chirp
    kd_fit: 1.91          # from chirp
    I_fit: 0.0188
    omega_n_hz: 7.78
    zeta: 0.52
    r2: 0.94
    # Commanded for comparison
    kp_cmd: 45.0
    kd_cmd: 0.50
    # Quality flags
    method_used: chirp
    base_omega_rms: 0.018  # rad/s during chirp
    trustworthy: true
    # Per-trial cross-check
    step:  {kp: 45.1, kd: 2.05, I: 0.019, r2: 0.91}
    chirp: {kp: 44.8, kd: 1.91, I: 0.0188, r2: 0.94}
    sine2: {gain_db: +0.40, phase_deg: -30.5}
notes: |
  Identification is independent of kp_cmd assumption. Step and chirp
  agreed within 6 % on all 10 joints.
```

## 6. Acceptance criteria

The implementation is done when **all** of these hold:

1. `pd_calibration_tool --plan side_left` runs all 5 left-side joints
   sequentially through step + chirp + 2 Hz sine without operator
   input. Total runtime: ≤ 5 min including settles.
2. Same for `--plan side_right`.
3. Run output is a single new directory under `bin/data/pd_calibration/`
   with the standard `manifest.json`, per-trial NPZ files, and the
   `base_omega_rms_rad_s` field present in every trial entry.
4. `python3 tools/calibration_fit/fit_pd_full.py <run_dir>` produces a
   `calibration.yaml` matching the schema in §5.5, where `method_used`
   is `chirp` for all 10 joints in a torso-clamped run.
5. For ankle / knee / hip_pitch joints, `kp_fit` agrees with `kp_cmd`
   within ±15 % when the torso is properly clamped (verified by
   `base_omega_rms < 0.05 rad/s` per trial).
6. L/R symmetry: same-name joints have `omega_n_hz` agreeing within
   ±10 %, `zeta` within ±0.10. If not, log a warning.
7. The previous `fit_pd_sine.py` workflow still runs on the old
   2026-05-29 dataset and produces its old output unchanged.
8. New CLI flags documented in `USAGE.md` under §5 (pd_calibration_tool).

## 7. Out of scope

- Multi-pose calibration (only MGTO).
- Automatic torso-fixture detection (assume operator did it right).
- Real-time online identification (offline fit only).
- Anything in `qmini_lab` (training repo).
- Re-running the old 2026-05-29 dataset with the new fitter — keep
  the old dataset+fit as the historical reference.

## 8. Files to touch

```
include/user/calibration/loop.h        # add Plan kind
include/user/calibration/trials.h      # build_side_plan(int side)
source/user/calibration/trials.cpp     # implement build_side_plan
source/user/calibration/loop.cpp       # record imu_omega_rms per trial
source/user/calibration/pd_calibration_main.cpp  # --plan side_left/right/default_full
source/user/calibration/npz_writer.cpp # add base_omega_rms scalar key
tools/calibration_fit/fit_pd_full.py   # new file (combines step+chirp+sine)
USAGE.md                                # add §5.3 half-side workflow
```

The C++ side is ≤200 LOC of additions; the Python side is ~300 LOC.
Total estimate: 1 implementing-agent session.
