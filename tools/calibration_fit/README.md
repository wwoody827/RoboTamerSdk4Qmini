# Offline PD calibration fitter

Companion to `bin/pd_calibration_tool`. Reads a calibration run directory
(SDK output) and produces `calibration.yaml` with per-joint fitted
`(kp_eff, kd_eff, I_eff)`.

See `PD_CALIBRATION_SPEC.md` §8 for the model and §8.1 for the
`kp_motor = kp_eff − g_grad_urdf` recovery path.

## Three scripts — pick by what you ran

| Script | Input test | Use when |
|---|---|---|
| `analyze_run.py` | any | **run first** — quick-look: tracking quality + whether the base was actually still. PNGs + console table; warns if base ω > 0.3 rad/s. |
| `fit_pd.py` | step (Test A) | the primary second-order fit → `calibration.yaml`. |
| `fit_pd_sine.py` | sine (Test B) | robot hangs/stands **freely** — does IMU chassis-recoil compensation so base swing doesn't bias `I_eff`. Reports encoder-only vs compensated. |

All three take the run dir as the first positional arg and accept `--out`
(default `<run_dir>/calibration.yaml`). `fit_pd.py`/`fit_pd_sine.py` fall
back to `.json` if pyyaml is unavailable.

## Use

```bash
# 0. sanity-check the run (base fixed? joint tracked?):
python3 analyze_run.py data/pd_calibration/<run_id>/

# 1a. STEP fit, bare (no gravity subtraction):
python3 fit_pd.py data/pd_calibration/<run_id>/

# 1b. STEP fit with analytical URDF gravity gradient subtracted:
python3 fit_pd.py data/pd_calibration/<run_id>/ --mjcf sim_assets/q1_sim.mjcf

# 1c. SINE fit with chassis-recoil compensation (free-standing robot):
python3 fit_pd_sine.py data/pd_calibration/<run_id>/
```

`fit_pd_sine.py` and `analyze_run.py` need the IMU channels in the npz —
don't pass `--no-imu` to `pd_calibration_tool` if you intend to use them.

## Dependencies

- `numpy` (required)
- `scipy.optimize.minimize` (required)
- `pyyaml` (preferred; falls back to JSON)
- `matplotlib` (required by `analyze_run.py` and `fit_pd_sine.py` for plots)
- `mujoco` (optional; only needed for `fit_pd.py --mjcf`)

## What the fitter does NOT do

- It does NOT auto-edit `qmini_lab/source/qmini_lab/assets/q1/constants.py`.
  Per spec §11, the operator reviews `calibration.yaml` before updating
  `QMINI_STIFFNESS / QMINI_DAMPING / QMINI_PD_DAMPING` by hand.
