# 2 — PD & friction calibration

Measures the real per-joint actuator model — PD stiffness/damping, effective
inertia, and passive friction — so the **sim** (Isaac Lab + MuJoCo) matches
the hardware. This is a **sim2real fidelity** task: it is **not** required to
make the robot stand or run a policy. Do it only when you're closing a
sim2real gap or re-tuning the training model.

Prerequisites: `startq` calibrated ([`1_calibrate_joints.md`](1_calibrate_joints.md)
— this tool reads `startq`, never writes it) and a harness.

Design references (the "why"): `PD_CALIBRATION_SPEC.md` (Test A/B/C),
`FREE_RELEASE_CALIBRATION_SPEC.md` (Test D), `PD_CALIBRATION_HALF_SIDE_SPEC.md`,
and the as-run results in `docs/calibration_notes/`.

> **Working-directory gotcha** (same as doc 1): run `pd_calibration_tool` and
> `motor_status` from **`bin/`**; run the Python fitters from the **repo root**.

---

## TL;DR — you need very little

All 10 joints use the **identical** GO-M8010-6 motor+gearbox, so you do **not**
calibrate all of them:

| Quantity | How many joints | Why |
|---|---|---|
| **PD (kp/kd), friction (b, f)** | **one** normal-ratio joint (e.g. hip_pitch) | identical motor → one measurement applies to all 8 normal joints |
| **friction for hip_roll** | **hip_roll** separately | extra 3× gear → ratio 18.99 vs 6.33; friction reflects differently and the extra stage adds its own |
| **inertia `I`** | none — **read per-joint from the URDF** | `I` is the leg mass each joint carries, not a motor property |

So a full friction job is essentially: free-release `hip_pitch` (normal motor)
+ deal with `hip_roll`. PD/inertia: step-test the same representative joint(s).

---

## Physical setup

1. **Suspend the robot in a harness**, feet 3–5 cm off the ground, legs free.
2. **Hold the base RIGIDLY** — clamp it to the stand. A hand introduces
   low-frequency compliance that biases the inertia/PD fit (and shows up as
   base motion in `analyze_run.py`).
3. **Half-side**: calibrate one leg at a time with `--joints`. The tool
   automatically holds the other 9 joints at MGTO with hold gains, so only the
   joint under test moves — clean reaction against a rigid rest-of-robot.
4. e-stop in reach; `ModemManager` masked; confirm the bus with `./motor_status`.

---

## The five tests

| Test | Drives | Measures | Fitter | Works on this robot? |
|---|---|---|---|---|
| **A** step | one joint, ±0.15 rad steps | `kp_eff, kd_eff, I_eff` (closed-loop) | `fit_pd.py`, `fit_pd_imu_comp.py` | ✅ (needs rigid base) |
| **B** sine | sine sweep | frequency response | `fit_pd_sine.py` | ✅ |
| **C** chirp | 0.25→10 Hz | Bode | `fit_pd.py` | ✅ |
| **D** free-release | displace, release, decay | passive `b`, `f` (**position-based**) | `fit_passive.py` | ✅ for gravity-loaded joints |
| **E** const-velocity | triangle at fixed speed | `b`, `f` from `tau_est` | `fit_friction.py` | ❌ **`tau_est` unusable** (see below) |

A/B/C are the original PD/inertia protocol; D/E are friction add-ons. Default
`--tests` is `A,B,C`; **D and E are opt-in.**

---

## Stage 1 — Motor PD + inertia (Test A, optionally B/C)

Half-side, base clamped:
```bash
cd ~/code/RoboTamerSdk4Qmini/bin
./motor_status                                   # confirm bus
# one representative normal joint is enough for the motor (e.g. hip_pitch_l = 2):
./pd_calibration_tool --i-have-checked-the-harness --joints 2 --tests A,C --safe-dq-max 8 --label pd_hp
```
Fit (from repo root):
```bash
python3 tools/calibration_fit/analyze_run.py  bin/data/pd_calibration/<run>/   # was the base still?
python3 tools/calibration_fit/fit_pd.py       bin/data/pd_calibration/<run>/ --mjcf sim_assets/q1_sim.mjcf
# fit_pd_imu_comp.py = same fit with IMU base-motion compensation (use if the
# base rocked); fit_pd_sine.py for Test B sine data.
```
Output `calibration.yaml`: per-joint `kp_eff, kd_eff, I_eff, omega_n, zeta`.
**Run `analyze_run.py` first** — if base ω peaked > 0.3 rad/s the base wasn't
rigid and the fit is biased (re-clamp, or use the IMU-comp fitter).

> `--mjcf` subtracts the analytical URDF gravity gradient so the output gives
> `kp_motor` (what the sim needs), not `kp_eff = kp_motor + g_grad`.

---

## Stage 2 — Friction (Test D free-release)

**Position-based, so it works where torque-based Test E doesn't.** Run Test A
+ D together (the passive fit takes `I` from the A fit):
```bash
cd ~/code/RoboTamerSdk4Qmini/bin
./pd_calibration_tool --i-have-checked-the-harness --joints 2 --tests A,D --safe-dq-max 8 --label fric_hp
cd .. && python3 tools/calibration_fit/fit_passive.py bin/data/pd_calibration/<run>/
```
Gives per-joint `b` (viscous) and `f` (Coulomb) → `passive_dynamics.yaml`.

**Critical: displacement direction.** Free-release decay is driven by the
gravity restoring torque, so the joint only swings if `+0.15 rad` pushes it
*away* from its gravity equilibrium. On a mirrored biped the same `+amp`
excites one leg and not the other. So:
- A joint that "didn't swing" (gate: `< 3°`) was displaced the wrong way →
  re-run that joint with the opposite sign, or use a bigger amplitude.
- **hip_yaw** (vertical axis, no gravity restoring) and **ankle** (foot too
  light) won't swing in either direction — Test D can't do them.

### As-run results (2026-05-31)

- **Normal-ratio motor** (from `hip_pitch_l`, R²=0.92):
  **`b = 0.054 N·m·s/rad`, `f = 0.023 N·m`** → apply to all 8 normal joints.
- **hip_roll** — its own free-release came out rank-deficient (monotonic
  decay, `b`/`g` inseparable), so it's estimated by **gear-ratio scaling**:
  `f ≈ 0.068 N·m`, `b ≈ 0.486 N·m·s/rad` (joint-side) — a **lower bound** (the
  extra gear adds friction the scaling ignores). To measure it directly, try a
  larger free-release (≈0.25–0.3 rad) for an oscillatory swing.

Details: `docs/calibration_notes/friction_findings_2026_05_31.md`.

---

## What does NOT work here: Test E / `tau_est`

Test E reads friction from the motor torque at constant velocity — correct in
principle, but **this robot's `tau_est` is unusable for it**:
1. **Under-scaled** — `motor_unitree.cpp` reports `d.tau / ratio`; joint torque
   is motor torque **× ratio**, so `tau_est` is ~`ratio²` too small (suspected
   HAL bug, left as-is — it's the deploy torque convention).
2. **Too coarse/noisy** — even hip_pitch driving the whole leg reads ≤ 0.12 N·m
   (vs ~2 N·m physical); the friction signal sits at the noise floor (R²~0.5).

So **use Test D (position-based) for friction**, not E. `fit_friction.py` and
Test E stay in the tree (correct given a good torque sensor) but are flagged.

---

## Applying results to `qmini_lab` (by hand)

Per `PD_CALIBRATION_SPEC.md §8` and project policy — **review and edit
`constants.py` / the URDF manually; nothing auto-edits the training repo.**

| Sim field | From | Note |
|---|---|---|
| `QMINI_STIFFNESS[name]` | `kp_motor` (= `kp_eff − g_grad`, from `fit_pd.py --mjcf`) | not `kp_eff` (sim adds its own gravity) |
| `QMINI_DAMPING` / `QMINI_PD_DAMPING` | `kd_eff − URDF damping` | the "×5" MuJoCo hack goes away after this |
| URDF `<joint damping=…>` | `b` (joint-side) | 0.054 normal / ~0.49 hip_roll |
| URDF `<joint frictionloss=…>` | `f` (joint-side) | 0.023 normal / ~0.07 hip_roll |
| inertia | URDF inertials | per-joint, not measured |

---

## Gotchas

- **`startq` first.** This tool reads `startq` and ramps to MGTO at the start;
  it no longer has a zero-cal phase. Calibrate joints (doc 1) before running.
- **Rigid base.** Hand-holding biases inertia/PD; `analyze_run.py` reports it.
- **`--safe-dq-max 8`** for high-freq sine / chirp / free-release (the default 4
  trips on fast trials).
- **Raw NPZ traces** under `bin/data/pd_calibration/` are gitignored (MB each,
  robot-specific). Commit the small fitted `*.yaml` if you want a record, or
  `git add -f` a run deliberately.
- **MGTO pose:** the tool uses the current `config.yaml::ref_joint_act` (your
  CoM-calibrated pose) — fine, the small pose shift barely affects the fit.

---

## CLI quick reference

```
--i-have-checked-the-harness   required (refuses to start otherwise)
--joints N,N,...               subset (default all 10) — use for half-side
--tests A,B,C[,D,E]            default A,B,C; D/E opt-in
--sine-freqs <hz,...>          Test B frequencies (default 0.25,0.5,1,2,4,8)
--safe-dq-max <rad/s>          velocity watchdog (default 4; use 8 for fast trials)
--label / --operator / --notes run metadata
--mjcf / --viewer              sim dry-run (no-op on hardware)
```
Full flag list and per-test detail: `USAGE.md §5`.
