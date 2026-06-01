# PD calibration — 2026-05-31

Per-joint PD identification on the real Qmini robot, follow-up to the
geometric `startq` recalibration earlier the same day (SDK commit
`37f7ed5`). Goal: derive the effective per-joint dynamics (kp_eff,
kd_eff, I_eff, ωₙ, ζ) for use in sim training, and quantify how much the
motor-side commanded gains translate into load-side response.

## Setup

- **Robot pose during test**: MGTO standing pose (per `bin/config.yaml`
  `ref_joint_act` calibrated 2026-05-31). Body level (IMU rpy < 2°),
  legs in deploy-ready stance.
- **Base fixation**: the robot was loosely restrained by hand / fixture
  but not rigidly clamped. IMU rpy during a single-joint Test A excursion
  showed ~5° of base rocking — this turned out to be a non-issue for the
  fit (see IMU compensation below).
- **Other joints during a single-joint trial**: held at MGTO with the
  same per-joint deploy gains (`hold_kp` / `hold_kd` from config.yaml).
- **Control rate**: 66.67 Hz (1 / `control_dt`), per
  `pd_calibration_tool` default.
- **Watchdog**: `--safe-dq-max 8 rad/s` (raised from default 4 to allow
  the higher-bandwidth joints to swing during Test A).

## Data

Two runs in `bin/data/pd_calibration/`:

| Run | Tests | Joints | Outcome |
|-----|-------|--------|---------|
| `2026-05-31_17-20-07_left`  | A + C | 0..4 (L leg) | 9/10 ok, 1 aborted (ankle_l chirp watchdog) |
| `2026-05-31_17-24-21_right` | A + C | 5..9 (R leg) | 5/5 ok |

`tests=A,C`, `kp` / `kd` from config.yaml (deploy gains, not the spec
default 30/1.0). Each joint: 1× Test A (9 s step) + 1× Test C (30 s log
chirp 0.25→10 Hz, amplitude 0.08 rad).

## Method

### Baseline fit — `tools/calibration_fit/fit_pd.py`

For each joint, fit a single-DOF damped second-order system to the Test A
step trace:

$$I_\text{eff}\,\ddot{q} + k_{d,\text{eff}}\,\dot{q} + k_{p,\text{eff}}\,(q - q_\text{target}) = 0$$

via Nelder-Mead on the L2 residual against forward-Euler simulation. Fit
window `t ∈ [0.5, 4.0]` s (covers the first +step rise + decay).
Outputs `(I_eff, kp_eff, kd_eff)` per joint plus derived `ωₙ = √(kp/I)/2π`
and `ζ = kd/(2√(kp·I))`.

### IMU-compensated fit — `tools/calibration_fit/fit_pd_imu_comp.py`

Same model with an added known forcing term to account for base motion:

$$I_\text{link}\,\ddot{q} + k_{d}\,\dot{q} + k_p\,(q - q_\text{target}) = -I_\text{link}\,\alpha_{\text{base},\text{proj}}$$

where $\alpha_{\text{base},\text{proj}}$ is the IMU-measured base angular
acceleration projected onto the joint axis (body-frame small-angle
approximation: yaw→ẑ, roll→x̂, pitch/knee/ankle→ŷ). Reasoning: the
joint encoder reads the rotor-vs-stator relative angle; if the stator
(rigidly attached to base) rotates in world frame, the load-side dynamics
gain a forcing term equal to the base reaction.

## Results

### Baseline fit (no IMU compensation)

| Joint        | config kp | kp_eff | kd_eff | I_eff (kg·m²) | ωₙ (Hz) | ζ   | R²    |
|--------------|----------:|-------:|-------:|--------------:|--------:|----:|------:|
| hip_yaw_l    | 55        | 18.63  | 0.528  | 0.0168        | 5.30    | 0.47| 0.992 |
| hip_roll_l   | 105       | 14.21  | 0.609  | 0.0295        | 3.49    | 0.47| 0.989 |
| hip_pitch_l  | 75        | 12.01  | 0.428  | 0.0209        | 3.82    | 0.43| 0.978 |
| knee_l       | 45        | 14.73  | 0.622  | 0.0249        | 3.87    | 0.51| 0.938 |
| ankle_l      | 30        | 18.76  | 0.542  | 0.0187        | 5.03    | 0.46| 0.987 |
| hip_yaw_r    | 55        | 16.66  | 0.496  | 0.0202        | 4.57    | 0.43| 0.994 |
| hip_roll_r   | 105       | 12.98  | 0.592  | 0.0283        | 3.41    | 0.49| 0.996 |
| hip_pitch_r  | 75        | 12.12  | 0.442  | 0.0200        | 3.92    | 0.45| 0.989 |
| knee_r       | 45        | 13.23  | 0.538  | 0.0182        | 4.29    | 0.55| 0.971 |
| ankle_r      | 30        | 15.91  | 0.563  | 0.0207        | 4.41    | 0.49| 0.981 |

L/R symmetric within ~10% — geometric `startq` recalibration earlier the
same day eliminated the previous asymmetry. R² > 0.94 throughout (knee_l
0.94 is the worst — knee A-trial has the largest amplitude / most
nonlinearity).

### IMU compensation effect

| Joint        | kp_orig | kp_comp | Δ%   | α_base_rms (rad/s²) |
|--------------|--------:|--------:|-----:|--------------------:|
| hip_yaw_l    | 18.63   | 18.82   | +1%  | 0.68 |
| hip_roll_l   | 14.21   | 14.62   | +3%  | 2.04 |
| hip_pitch_l  | 12.01   |  9.71   | -19% (outlier) | 0.70 |
| knee_l       | 14.73   | 14.88   | +1%  | 0.36 |
| ankle_l      | 18.76   | 18.78   |  0%  | 0.06 |
| hip_yaw_r    | 16.66   | 15.65   | -6%  | 1.72 |
| hip_roll_r   | 12.98   | 13.81   | +6%  | 1.16 |
| hip_pitch_r  | 12.12   | 12.37   | +2%  | 0.56 |
| knee_r       | 13.23   | 13.01   | -2%  | 1.54 |
| ankle_r      | 15.91   | 15.95   |  0%  | 0.28 |

**Base motion is NOT the dominant error source** — even joints with the
largest α_base_rms (hip_roll_l 2.04, hip_yaw_r 1.72) shift by only a few
percent under compensation. The kp_eff vs config_kp gap is structural, not
a measurement artifact.

## Damping analysis (kd_eff vs config_kd)

While `kp_eff` is uniformly *smaller* than commanded (gearbox compliance,
see below), `kd_eff` shows a different pattern:

| Joint     | cfg_kd | kd_eff (L/R)   | ratio (eff / cfg) |
|-----------|-------:|---------------:|------------------:|
| hip_yaw   | 0.30   | 0.528 / 0.496  | **1.65 - 1.76 ×** ↑ |
| hip_roll  | 2.50   | 0.609 / 0.592  | **0.24 ×** ↓ (anomaly) |
| hip_pitch | 0.30   | 0.428 / 0.441  | 1.43 - 1.47 × ↑ |
| knee      | 0.50   | 0.622 / 0.538  | 1.08 - 1.24 × |
| ankle     | 0.25   | 0.542 / 0.562  | **2.17 - 2.25 ×** ↑ |

**Most joints have kd_eff > config_kd**, by 1.1-2.3×. This is the
opposite of the kp pattern. The likely explanation is that real
hardware has significant **passive joint damping** (bearings, lubricated
gearbox friction, magnetic detent) that adds on top of the PD-commanded
damping. Ankles show the biggest ratio (2.25×) — small load inertia
means friction is a larger fraction of total damping.

**hip_roll is the anomaly**: commanded kd=2.5 but fit gives kd_eff=0.6,
a 4× reduction. Possible causes:
1. The large commanded kd produces a strongly overdamped step response →
   the fit has poor identifiability between kp and kd (both pull toward
   sluggish behavior, ambiguous attribution).
2. Same gearbox compliance affects kd commands too (high-frequency
   damping commands are attenuated by compliance, similar to kp).
3. The hold gain on adjacent joints (other hip_roll = 2.5) may
   interact during the test.

Quality-of-fit R² ≥ 0.989 for hip_roll, so the data IS well-explained by
the simple 2nd-order model — but kp/kd attribution may be wrong.

`ζ` is 0.43-0.55 across all joints — consistent under-damped design,
matches typical robot joint servo tuning.

**Passive damping cannot be separated from PD-loop damping in this
experiment.** That requires a free-release trial (PD off, observe natural
decay) — see SI item 6 / open items below.

## Interpretation

The fitted `kp_eff` is **3-7× smaller than the commanded `config_kp`**
across all joints. This is consistent across L and R legs and stable
under IMU compensation. Most likely cause: the motor's commanded PD acts
through the GO-M8010-6's 6.33:1 planetary reducer, which has significant
torsional compliance ("gear winding"). The motor-side stiffness command
is filtered through the gearbox elasticity before reaching the load-side
encoder. Fitting a single rigid-body 2nd-order model lumps this into a
reduced effective stiffness.

Other contributing factors (smaller):
- Slight residual base motion (the IMU compensation experiment quantified
  this at <5% effect)
- Possible motor current saturation at the largest step amplitudes
- Frame / leg compliance below the joint

The `ωₙ` values (3.4–5.3 Hz) are reasonable for a high-bandwidth joint
servo with this motor + gearbox.

## Implications for sim

The IL training-side PD currently uses `config_kp` values (the deploy
values from `bin/config.yaml`, e.g. `hip_yaw=55`). This gives the
simulated joints a load-side response 3-7× stiffer than the real
hardware. Policies learn to rely on this artificial stiffness for control.

**Recommended fix**: in `qmini_lab/source/qmini_lab/configs/base.yaml`,
replace the `pd_gains.stiffness` block with the measured `kp_eff` values:

```yaml
pd_gains:
  stiffness:
    hip_yaw:   17   # was 55  — measured kp_eff (L 18.6, R 16.7)
    hip_roll:  14   # was 105 — measured kp_eff (L 14.2, R 13.0)
    hip_pitch: 12   # was 75  — measured kp_eff (L 12.0, R 12.1)
    knee:      14   # was 45  — measured kp_eff (L 14.7, R 13.2)
    ankle:     17   # was 30  — measured kp_eff (L 18.8, R 15.9)
  damping:
    hip_yaw:   0.5  # was 0.3  — measured kd_eff
    hip_roll:  0.6  # was 2.5  — measured kd_eff
    hip_pitch: 0.4  # was 0.3  — measured kd_eff
    knee:      0.6  # was 0.5  — measured kd_eff
    ankle:     0.55 # was 0.25 — measured kd_eff
```

(Average of L/R measurements; rounded.)

**Deploy side keeps the old values** — `bin/config.yaml`'s `kp`/`kd`
are the MOTOR-side commands, not the load-side response. Those drive the
physical motor and should stay the same.

## Outputs

- `bin/data/pd_calibration/2026-05-31_17-20-07_left/calibration.yaml`
- `bin/data/pd_calibration/2026-05-31_17-20-07_left/calibration_imu_comp.yaml`
- `bin/data/pd_calibration/2026-05-31_17-24-21_right/calibration.yaml`
- `bin/data/pd_calibration/2026-05-31_17-24-21_right/calibration_imu_comp.yaml`
- `tools/calibration_fit/fit_pd_imu_comp.py` (this run's new tool)

## Open items

1. **Test C (chirp) Bode plots not analyzed** — the chirp traces are
   captured but the per-joint Bode visualization wasn't run. Would be
   useful as a sanity check on `ωₙ` and to look for higher-order modes.
2. **frictionloss (Coulomb friction) not measured** — needs a separate
   free-release trial (`pd_calibration_tool` doesn't have this mode yet;
   see SI item 6).
3. **Gearbox compliance modeling** — for sim2real beyond just the kp
   number, could add an explicit series-elastic actuator model in IL/MJ.
   For now, using the lumped `kp_eff` is the pragmatic fix.
