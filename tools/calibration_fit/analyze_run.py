#!/usr/bin/env python3
"""Quick-look analyzer for a pd_calibration run.

Reads every per-joint NPZ in a run directory and answers two questions:

  1. Did the joint under test track its sine command well?
     (amp ratio, phase lag, residual RMS)
  2. Was the base actually fixed during the trial?
     (IMU angular velocity, acc deviation from gravity, off-joint drift)

Produces per-joint PNG plots + a console table.

Usage:
    python3 analyze_run.py path/to/data/pd_calibration/<run_id>/
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


JOINT_NAMES = [
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
]


def fit_sine(t, x, f_hz):
    """LS-fit x ≈ A*sin(2πf t + φ) + c. Returns (amp, phase_rad, dc)."""
    w = 2.0 * np.pi * f_hz
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    a, b, c = coef
    amp = float(np.hypot(a, b))
    phase = float(np.arctan2(b, a))
    return amp, phase, float(c)


def analyze_trial(npz_path: Path, trial: dict) -> dict:
    z = np.load(npz_path, allow_pickle=False)
    t = z["t_s"].astype(np.float64)
    q = z["q"].astype(np.float64)
    qt = z["q_target"].astype(np.float64)
    dq = z["dq"].astype(np.float64)
    omega = z["imu_omega"].astype(np.float64)
    acc = z["imu_acc"].astype(np.float64)
    j = int(z["joint_under_test"])

    # Discard the first 0.5 s (settle / smooth-step ramp).
    mask = t >= 0.5
    t_w = t[mask]
    q_w = q[mask, j]
    qt_w = qt[mask, j]
    dq_w = dq[mask, j]
    omega_w = omega[mask]
    acc_w = acc[mask]

    # Sine fits at the commanded frequency.
    f = float(trial["freq_hz"])
    amp_cmd, phi_cmd, dc_cmd = fit_sine(t_w, qt_w, f)
    amp_q,   phi_q,   dc_q   = fit_sine(t_w, q_w,  f)
    gain_db = 20.0 * np.log10(amp_q / max(amp_cmd, 1e-9))
    phase_lag_deg = float(np.degrees(phi_q - phi_cmd))
    # Wrap to [-180, 180]
    phase_lag_deg = (phase_lag_deg + 180.0) % 360.0 - 180.0

    # Tracking residual (after removing DC + commanded sine).
    sin_only = amp_q * np.sin(2 * np.pi * f * t_w + phi_q) + dc_q
    resid_rms = float(np.sqrt(np.mean((q_w - sin_only) ** 2)))

    # Off-joint drift: how much did the *other* 9 joints move?
    off_drift_max = 0.0
    off_drift_name = ""
    for k in range(10):
        if k == j:
            continue
        drift = float(np.std(q[mask, k]))
        if drift > off_drift_max:
            off_drift_max = drift
            off_drift_name = JOINT_NAMES[k]

    # IMU stats — the base motion we're worried about.
    omega_norm = np.linalg.norm(omega_w, axis=1)
    base_omega_rms = float(np.sqrt(np.mean(omega_norm ** 2)))
    base_omega_max = float(np.max(omega_norm))

    # |acc| should ~ g = 9.81. Deviation reflects shake.
    acc_norm = np.linalg.norm(acc_w, axis=1)
    acc_dev_rms = float(np.sqrt(np.mean((acc_norm - 9.81) ** 2)))

    return dict(
        joint=j,
        joint_name=JOINT_NAMES[j],
        amp_cmd=amp_cmd,
        amp_q=amp_q,
        gain_db=float(gain_db),
        phase_lag_deg=phase_lag_deg,
        resid_rms=resid_rms,
        off_drift_max=off_drift_max,
        off_drift_name=off_drift_name,
        base_omega_rms=base_omega_rms,
        base_omega_max=base_omega_max,
        acc_dev_rms=acc_dev_rms,
        # raw arrays for plotting
        _t=t, _q=q, _qt=qt, _dq=dq, _omega=omega, _acc=acc, _j=j,
    )


def plot_trial(res: dict, out_path: Path):
    j = res["_j"]
    t = res["_t"]; q = res["_q"]; qt = res["_qt"]; dq = res["_dq"]
    omega = res["_omega"]; acc = res["_acc"]

    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)

    # 1) joint under test: q vs q_target
    ax = axes[0]
    ax.plot(t, qt[:, j], "k--", lw=1.2, label="q_target")
    ax.plot(t, q[:, j],  "C0",  lw=1.4, label="q (meas)")
    ax.set_ylabel(f"{JOINT_NAMES[j]}  [rad]")
    ax.set_title(
        f"{JOINT_NAMES[j]}  |  amp_cmd={res['amp_cmd']:.3f}  "
        f"amp_q={res['amp_q']:.3f}  "
        f"gain={res['gain_db']:+.2f} dB  "
        f"phase={res['phase_lag_deg']:+.1f}°  "
        f"resid_rms={res['resid_rms']:.4f}"
    )
    ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.3)

    # 2) off-joints: q drift relative to MGTO offset (subtract initial value)
    ax = axes[1]
    for k in range(10):
        if k == j:
            continue
        ax.plot(t, q[:, k] - q[0, k], lw=0.7, alpha=0.7, label=JOINT_NAMES[k])
    ax.axhline(0, color="k", lw=0.5)
    ax.set_ylabel("Δq off-joints [rad]")
    ax.set_title(f"Off-joint drift (worst: {res['off_drift_name']} σ={res['off_drift_max']:.4f} rad)")
    ax.legend(loc="upper right", fontsize=6, ncol=3); ax.grid(alpha=0.3)

    # 3) IMU angular velocity
    ax = axes[2]
    for i, name in enumerate(["wx", "wy", "wz"]):
        ax.plot(t, omega[:, i], lw=0.9, label=name)
    ax.set_ylabel("IMU ω [rad/s]")
    ax.set_title(
        f"Base angular vel — RMS={res['base_omega_rms']:.3f}, "
        f"max={res['base_omega_max']:.3f} rad/s"
    )
    ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.3)

    # 4) IMU |acc| - g
    ax = axes[3]
    acc_norm = np.linalg.norm(acc, axis=1)
    ax.plot(t, acc_norm - 9.81, "C3", lw=0.9, label="|acc| − g")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("Δ|acc| [m/s²]")
    ax.set_title(f"Base acc deviation — RMS={res['acc_dev_rms']:.3f} m/s²")
    ax.legend(loc="upper right", fontsize=8); ax.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args()

    run_dir = args.run_dir
    manifest = json.loads((run_dir / "manifest.json").read_text())
    plots_dir = run_dir / "plots"
    plots_dir.mkdir(exist_ok=True)

    results = []
    for trial in manifest["trials"]:
        if trial["result"] != "ok":
            continue
        npz = run_dir / trial["file"]
        if not npz.exists():
            print(f"missing: {npz}")
            continue
        res = analyze_trial(npz, trial)
        out_png = plots_dir / f"joint_{res['joint']:02d}_{res['joint_name']}.png"
        plot_trial(res, out_png)
        results.append((trial, res))
        print(f"wrote {out_png.relative_to(run_dir)}")

    # Console summary table.
    print()
    hdr = (
        f"{'joint':<14} {'amp_cmd':>8} {'amp_q':>8} {'gain_dB':>8} "
        f"{'phase°':>8} {'resid':>7} | "
        f"{'off_drift':>9} {'ω_rms':>7} {'ω_max':>7} {'a_dev':>7}"
    )
    print(hdr)
    print("-" * len(hdr))
    for trial, r in results:
        print(
            f"{r['joint_name']:<14} {r['amp_cmd']:>8.3f} {r['amp_q']:>8.3f} "
            f"{r['gain_db']:>+8.2f} {r['phase_lag_deg']:>+8.1f} "
            f"{r['resid_rms']:>7.4f} | "
            f"{r['off_drift_max']:>9.4f} "
            f"{r['base_omega_rms']:>7.3f} {r['base_omega_max']:>7.3f} "
            f"{r['acc_dev_rms']:>7.3f}"
        )

    # Overall verdict on base motion.
    print()
    omega_rms_all = np.mean([r["base_omega_rms"] for _, r in results])
    omega_max_all = max(r["base_omega_max"] for _, r in results)
    off_drift_max = max(r["off_drift_max"] for _, r in results)
    print(f"avg base ω RMS across all trials: {omega_rms_all:.3f} rad/s")
    print(f"peak base ω across all trials:    {omega_max_all:.3f} rad/s")
    print(f"max off-joint drift σ:            {off_drift_max:.4f} rad")
    print()
    if omega_max_all > 0.3:
        print("[WARN] base ω peak > 0.3 rad/s → base was clearly moving during trials.")
    elif omega_max_all > 0.1:
        print("[INFO] base ω peak > 0.1 rad/s → mild base motion; fit may have bias.")
    else:
        print("[OK]   base ω stayed below 0.1 rad/s — base reasonably fixed.")
    if off_drift_max > 0.02:
        print("[WARN] off-joint σ > 0.02 rad → cross-coupling or whole-leg sway, not a single-joint test.")


if __name__ == "__main__":
    main()
