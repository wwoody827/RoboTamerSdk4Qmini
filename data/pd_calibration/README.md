# PD calibration data

Per-run output of `bin/pd_calibration_tool`. Each run lives under a
timestamped directory (`<YYYY-MM-DD_HH-MM-SS_label>/`) and contains the
NPZ traces, manifest, and run metadata. See `PD_CALIBRATION_SPEC.md` §7
for the full schema.

```
data/pd_calibration/<run_id>/
├── manifest.json
├── run_meta.json
├── log.txt
├── calibration.yaml             # written later by tools/calibration_fit/fit_pd.py
├── joint_00_hip_yaw_l/*.npz
├── joint_01_hip_roll_l/*.npz
├── ...
└── joint_09_ankle_pitch_r/*.npz
```

The `.npz` traces are not committed — they are megabytes per run and
robot-specific. `.gitignore` excludes `data/pd_calibration/*/`.

## How to consume `calibration.yaml` from `qmini_lab`

Once `fit_pd.py` produces `calibration.yaml`, review and apply by hand:

1. Open the YAML; inspect `quality_of_fit` (>0.9 = good), `omega_n_hz`,
   `zeta` per joint. For vertical-axis joints (`hip_yaw_l/r`) check that
   `kp_motor` is within ±25 % of the current `QMINI_STIFFNESS` value.
2. Open `qmini_lab/source/qmini_lab/assets/q1/constants.py`.
3. Update `QMINI_STIFFNESS[name] = kp_motor` (not `kp_eff` — the sim
   computes its own gravity).
4. Update `QMINI_DAMPING[name] = kd_eff − URDF_damping_in_chain`
   (typical URDF `<dynamics damping=...>` ≈ 1 N·m·s/rad).
5. For the MuJoCo sim2sim cfg, set `QMINI_PD_DAMPING[name]` to the same
   `kd_eff`-minus-URDF-damping value. The pre-calibration "× 5" hack in
   `V2MuJoCoCfg.kds` should now be removed.

Why not auto-edit `constants.py`? Per spec §11, calibration values can be
wrong in subtle ways (bad harness contact, drift, etc.) and silently
flowing them into the trained policy is a recipe for an injury. Operator
review is the gate.

## Re-running on a different robot

Same protocol; choose a new run-label:

```bash
cd bin/
./pd_calibration_tool --i-have-checked-the-harness \
    --label after_foot_box_change \
    --operator <name> \
    --notes "hardware change context"
```
