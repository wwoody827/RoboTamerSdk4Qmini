# Offline PD calibration fitter

Companion to `bin/pd_calibration_tool`. Reads a calibration run directory
(SDK output) and produces `calibration.yaml` with per-joint fitted
`(kp_eff, kd_eff, I_eff)` and a chirp-derived Bode estimate.

See `PD_CALIBRATION_SPEC.md` §8 for the model and §8.1 for the
`kp_motor = kp_eff − g_grad_urdf` recovery path.

## Use

```bash
# bare fit (no gravity subtraction):
python3 fit_pd.py data/pd_calibration/2026-05-25_15-30-00_initial/

# with analytical URDF gravity gradient subtracted:
python3 fit_pd.py data/pd_calibration/2026-05-25_15-30-00_initial/ \
    --mjcf sim_assets/q1_sim.mjcf
```

Output: `<run_dir>/calibration.yaml` (or `.json` if pyyaml is unavailable).

## Dependencies

- `numpy` (required)
- `scipy.optimize.minimize` (required)
- `pyyaml` (preferred; falls back to JSON)
- `mujoco` (optional; only needed if `--mjcf` is passed)

## What the fitter does NOT do

- It does NOT auto-edit `qmini_lab/source/qmini_lab/assets/q1/constants.py`.
  Per spec §11, the operator reviews `calibration.yaml` before updating
  `QMINI_STIFFNESS / QMINI_DAMPING / QMINI_PD_DAMPING` by hand.
- It does NOT validate Test D pose-variation; rerun the fitter per pose
  subset for that.
