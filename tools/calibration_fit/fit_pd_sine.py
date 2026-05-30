#!/usr/bin/env python3
"""Sine-trace PD identification with chassis-motion compensation.

The 2026-05-29 dataset was collected with the robot freely standing — IMU
shows the base swinging ±0.3 rad/s. The joint encoder is base-relative
(q = q_child - q_parent), so the *measurement* is uncontaminated, but the
fit input q̈ is contaminated:

    I_child · α_child_inertial = kp·(q_target - q) - kd·q̇
    α_child_inertial            = q̈_encoder + ω̇_chassis_proj

Without correction, the LS fit recovers the reduced (floating-base) inertia
  I_eff_float = I_child · I_chassis / (I_child + I_chassis).
With the IMU correction term we recover the child-only inertia.

Per-joint axis projection of IMU ω is based on the q1.urdf joint axes and
the small MGTO hip yaw/roll, so the axis at the crouch pose ≈ the URDF
nominal axis.  Hip pitch rotation of 1.4 rad does not change Y axis (it's
the rotation axis itself), so the heuristic holds for the whole leg.

Usage:
    python3 fit_pd_sine.py path/to/data/pd_calibration/<run_id>/
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

# IMU axis index (0=x/roll, 1=y/pitch, 2=z/yaw) and sign for projecting
# chassis angular velocity onto each joint axis at the MGTO crouch pose.
# Derived from q1.urdf:
#   hip_yaw  axis = (0, 0, -1)
#   hip_roll axis = (1, 0,  0)
#   hip_pitch axis = (0, ±1, 0)  (+L, -R)
#   knee  axis = (0, ∓1, 0)      (-L, +R)
#   ankle axis = (0, ±1, 0)      (+L, -R)
# At the crouch the small hip_yaw (≈ 0.23 rad) and hip_roll (≈ 0.04 rad)
# barely perturb the axes; the dominant hip_pitch is *about* y so leaves
# downstream y axes unchanged.
JOINT_IMU_AXIS = [
    (2, -1.0),  # 0 hip_yaw_l
    (0, +1.0),  # 1 hip_roll_l
    (1, +1.0),  # 2 hip_pitch_l
    (1, -1.0),  # 3 knee_l
    (1, +1.0),  # 4 ankle_l
    (2, -1.0),  # 5 hip_yaw_r
    (0, +1.0),  # 6 hip_roll_r
    (1, -1.0),  # 7 hip_pitch_r
    (1, +1.0),  # 8 knee_r
    (1, -1.0),  # 9 ankle_r
]


def smooth_diff(y, dt):
    """Central-difference derivative."""
    return np.gradient(y, dt)


def fit_sine(t, x, f_hz):
    """LS-fit x ≈ A·sin(2πf t + φ) + c."""
    w = 2.0 * np.pi * f_hz
    A = np.column_stack([np.sin(w * t), np.cos(w * t), np.ones_like(t)])
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    a, b, c = coef
    return float(np.hypot(a, b)), float(np.arctan2(b, a)), float(c)


def solve_omega_n_zeta(gain_lin, phase_rad, omega):
    """Given measured |H| and ∠H of the closed-loop response at angular
    frequency ω, solve for (ω_n, ζ) of the 2nd-order plant
        H(jω) = ω_n² / (ω_n² - ω² + j·2ζ·ω_n·ω).
    Closed-form via real/imag decomposition.
    """
    # H = ω_n² / D, D = (ω_n² - ω²) + j·2ζω_nω
    # |H|² · |D|² = ω_n⁴
    # 1/H = D/ω_n² = 1 - (ω/ω_n)² + j·2ζ(ω/ω_n)
    Hinv = (1.0 / gain_lin) * np.exp(-1j * phase_rad)
    re = Hinv.real           # 1 - (ω/ω_n)²
    im = Hinv.imag           # 2·ζ·(ω/ω_n)
    r2 = 1.0 - re            # (ω/ω_n)²
    if r2 <= 0:
        return float("nan"), float("nan")
    r = np.sqrt(r2)
    omega_n = omega / r
    zeta = im / (2.0 * r)
    return float(omega_n), float(zeta)


def fit_one_trial(npz_path: Path, trial: dict) -> dict:
    z = np.load(npz_path, allow_pickle=False)
    t = z["t_s"].astype(np.float64)
    q = z["q"].astype(np.float64)
    qt = z["q_target"].astype(np.float64)
    dq = z["dq"].astype(np.float64)
    omega = z["imu_omega"].astype(np.float64)
    j = int(z["joint_under_test"])
    f = float(trial["freq_hz"])
    w = 2.0 * np.pi * f
    kp_cmd = float(trial["kp"])
    kd_cmd = float(trial["kd"])

    # Window away the initial 0.6 s (settle + first half-period).
    mask = t >= 0.6
    t_w = t[mask]
    q_w = q[mask, j]
    qt_w = qt[mask, j]
    dq_w = dq[mask, j]

    # Project chassis ω onto joint axis at MGTO.
    ax_idx, ax_sign = JOINT_IMU_AXIS[j]
    omega_chassis = ax_sign * omega[mask, ax_idx]

    dt = float(np.mean(np.diff(t_w)))

    # ---- A. encoder-only sine fit (uncorrected) ----
    amp_qt, phi_qt, _ = fit_sine(t_w, qt_w, f)
    amp_q,  phi_q,  _ = fit_sine(t_w, q_w,  f)
    gain_lin_unc = amp_q / max(amp_qt, 1e-9)
    phase_unc = phi_q - phi_qt
    # Wrap
    phase_unc = (phase_unc + np.pi) % (2 * np.pi) - np.pi
    omega_n_unc, zeta_unc = solve_omega_n_zeta(gain_lin_unc, phase_unc, w)

    # ---- B. inertial-frame fit (chassis-compensated) ----
    # chassis angle at 2 Hz from ω: integrate sine amp_ω → amp_θ = amp_ω/ω,
    # phase shifted by -π/2 (integration of sin → -cos).
    amp_oc, phi_oc, _ = fit_sine(t_w, omega_chassis, f)
    # θ_chassis = amp_oc/w · sin(wt + phi_oc - π/2)
    amp_th = amp_oc / w
    phi_th = phi_oc - np.pi / 2
    # q_inertial(t) = q_encoder(t) + θ_chassis(t)
    # Sum two sines at same freq → another sine. Use complex phasor.
    Q_phasor = amp_q  * np.exp(1j * phi_q)
    T_phasor = amp_th * np.exp(1j * phi_th)
    Q_inertial = Q_phasor + T_phasor
    amp_qi = abs(Q_inertial)
    phi_qi = np.angle(Q_inertial)
    gain_lin_cor = amp_qi / max(amp_qt, 1e-9)
    phase_cor = phi_qi - phi_qt
    phase_cor = (phase_cor + np.pi) % (2 * np.pi) - np.pi
    omega_n_cor, zeta_cor = solve_omega_n_zeta(gain_lin_cor, phase_cor, w)

    # ---- C. inertia estimate (trust kp_cmd) ----
    I_cor = kp_cmd / (omega_n_cor ** 2) if omega_n_cor == omega_n_cor else float("nan")
    I_unc = kp_cmd / (omega_n_unc ** 2) if omega_n_unc == omega_n_unc else float("nan")
    kd_cor = 2 * zeta_cor * np.sqrt(kp_cmd * I_cor) if I_cor == I_cor else float("nan")
    kd_unc = 2 * zeta_unc * np.sqrt(kp_cmd * I_unc) if I_unc == I_unc else float("nan")

    # ---- D. data-quality flags ----
    chassis_amp_rad = amp_th
    chassis_ratio = chassis_amp_rad / max(amp_q, 1e-9)  # how much q is base?

    return dict(
        joint=j, joint_name=JOINT_NAMES[j],
        f_hz=f, kp_cmd=kp_cmd, kd_cmd=kd_cmd,
        amp_qt=amp_qt, amp_q=amp_q, amp_qi=amp_qi,
        amp_chassis_theta=chassis_amp_rad, chassis_ratio=chassis_ratio,
        gain_db_unc=20 * np.log10(gain_lin_unc),
        gain_db_cor=20 * np.log10(gain_lin_cor),
        phase_unc_deg=np.degrees(phase_unc),
        phase_cor_deg=np.degrees(phase_cor),
        omega_n_hz_unc=omega_n_unc / (2 * np.pi),
        omega_n_hz_cor=omega_n_cor / (2 * np.pi),
        zeta_unc=zeta_unc, zeta_cor=zeta_cor,
        I_unc=I_unc, I_cor=I_cor,
        kd_fit_unc=kd_unc, kd_fit_cor=kd_cor,
        # Arrays kept for plotting
        _t=t_w, _q=q_w, _qt=qt_w, _omega_chassis=omega_chassis,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, default=None,
                    help="calibration.yaml path (default: <run_dir>/calibration.yaml)")
    args = ap.parse_args()
    run_dir = args.run_dir
    manifest = json.loads((run_dir / "manifest.json").read_text())
    plots_dir = run_dir / "plots_fit"
    plots_dir.mkdir(exist_ok=True)

    results = []
    for trial in manifest["trials"]:
        if trial["result"] != "ok":
            continue
        npz = run_dir / trial["file"]
        r = fit_one_trial(npz, trial)
        results.append(r)

    # ---- Console table ----
    print()
    print("Per-joint fit — encoder-only (unc) vs chassis-compensated (cor)")
    print()
    h1 = (
        f"{'joint':<13} {'kp_cmd':>6} {'kd_cmd':>6} {'amp_q':>7} {'θ_chs':>7} "
        f"{'frac':>5} | "
        f"{'g_unc':>6} {'g_cor':>6} {'φ_unc':>6} {'φ_cor':>6} | "
        f"{'ωn_unc':>6} {'ωn_cor':>6} {'ζ_unc':>5} {'ζ_cor':>5} | "
        f"{'kd_unc':>6} {'kd_cor':>6}"
    )
    print(h1)
    print(
        f"{'':<13} {'':>6} {'':>6} {'rad':>7} {'rad':>7} "
        f"{'%':>5} | "
        f"{'dB':>6} {'dB':>6} {'°':>6} {'°':>6} | "
        f"{'Hz':>6} {'Hz':>6} {'':>5} {'':>5} | "
        f"{'':>6} {'':>6}"
    )
    print("-" * len(h1))
    for r in results:
        print(
            f"{r['joint_name']:<13} {r['kp_cmd']:>6.0f} {r['kd_cmd']:>6.2f} "
            f"{r['amp_q']:>7.4f} {r['amp_chassis_theta']:>7.4f} "
            f"{100 * r['chassis_ratio']:>4.1f}% | "
            f"{r['gain_db_unc']:>+6.2f} {r['gain_db_cor']:>+6.2f} "
            f"{r['phase_unc_deg']:>+6.1f} {r['phase_cor_deg']:>+6.1f} | "
            f"{r['omega_n_hz_unc']:>6.2f} {r['omega_n_hz_cor']:>6.2f} "
            f"{r['zeta_unc']:>5.2f} {r['zeta_cor']:>5.2f} | "
            f"{r['kd_fit_unc']:>6.2f} {r['kd_fit_cor']:>6.2f}"
        )

    # ---- Summary plot: kd_cmd vs kd_fit, ω_n per joint, chassis contribution ----
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    names = [r["joint_name"] for r in results]
    x = np.arange(len(names))

    ax = axes[0]
    ax.bar(x - 0.2, [r["kd_cmd"]    for r in results], 0.4, label="kd commanded", color="C0")
    ax.bar(x + 0.2, [r["kd_fit_cor"] for r in results], 0.4, label="kd fit (corrected)", color="C3")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("kd  [N·m·s/rad]")
    ax.set_title("Damping: commanded vs identified")
    ax.legend(); ax.grid(alpha=0.3)

    ax = axes[1]
    ax.bar(x - 0.2, [r["omega_n_hz_unc"] for r in results], 0.4, label="ω_n unc", color="C7")
    ax.bar(x + 0.2, [r["omega_n_hz_cor"] for r in results], 0.4, label="ω_n cor", color="C2")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("ω_n  [Hz]")
    ax.set_title("Natural frequency (kp_cmd assumed)")
    ax.legend(); ax.grid(alpha=0.3)

    ax = axes[2]
    ax.bar(x, [100 * r["chassis_ratio"] for r in results], color="C1")
    ax.set_xticks(x); ax.set_xticklabels(names, rotation=45, ha="right", fontsize=8)
    ax.set_ylabel("θ_chassis / amp_q  [%]")
    ax.set_title("How much of measured q swing is chassis recoil")
    ax.axhline(10, color="r", ls=":", lw=0.8); ax.grid(alpha=0.3)

    fig.tight_layout()
    summary_png = plots_dir / "summary.png"
    fig.savefig(summary_png, dpi=110)
    plt.close(fig)
    print(f"\nwrote {summary_png.relative_to(run_dir)}")

    # Per-trial plots: q, q_target, q_inertial, chassis θ
    for r in results:
        t_w = r["_t"]
        q_w = r["_q"]; qt_w = r["_qt"]
        oc = r["_omega_chassis"]
        # integrate ω (trapezoid) to get θ for visualization
        theta = np.cumsum((oc[:-1] + oc[1:]) * 0.5) * np.mean(np.diff(t_w))
        theta = np.concatenate([[0.0], theta])
        theta -= theta.mean()
        q_inertial = q_w + theta

        fig, ax = plt.subplots(figsize=(9, 4))
        ax.plot(t_w, qt_w - qt_w.mean(),       "k--", lw=1.1, label="q_target (centered)")
        ax.plot(t_w, q_w - q_w.mean(),         "C0",  lw=1.2, label="q encoder")
        ax.plot(t_w, q_inertial - q_inertial.mean(), "C3", lw=1.0, label="q + θ_chassis (inertial)")
        ax.plot(t_w, theta, "C1", lw=0.8, alpha=0.8, label="θ_chassis (∫ω)")
        ax.set_xlabel("t [s]"); ax.set_ylabel("rad")
        ax.set_title(
            f"{r['joint_name']}: chassis amp = {r['amp_chassis_theta']:.4f} rad "
            f"({100*r['chassis_ratio']:.1f}% of q amp), "
            f"ω_n unc/cor = {r['omega_n_hz_unc']:.2f}/{r['omega_n_hz_cor']:.2f} Hz"
        )
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
        fig.tight_layout()
        p = plots_dir / f"joint_{r['joint']:02d}_{r['joint_name']}_fit.png"
        fig.savefig(p, dpi=110); plt.close(fig)

    # ---- Commanded vs fitted comparison table ----
    print()
    print("Commanded vs corrected fit:")
    hdr = (f"{'joint':<13} | "
           f"{'kp_cmd':>7} {'kp_fit':>7} {'Δ':>6} | "
           f"{'kd_cmd':>7} {'kd_fit':>7} {'×':>5} | "
           f"{'ωn_Hz':>6} {'ζ':>5} {'flag':>4}")
    print(hdr); print("-" * len(hdr))
    for r in results:
        # kp from fit: we'd need a non-low-freq test to identify it
        # independently from I. At ω≪ω_n, gain ≈ 1 so |H|≈1 doesn't tell us
        # kp; we report kp_cmd as-trusted and flag if gain wandered > 1 dB.
        gain_flag = "GAIN" if abs(r["gain_db_cor"]) > 1.0 else ""
        chassis_flag = "BASE" if r["chassis_ratio"] > 0.15 else ""
        flag = (gain_flag + chassis_flag) or "ok"
        kd_ratio = r["kd_fit_cor"] / r["kd_cmd"] if r["kd_cmd"] > 0 else float("nan")
        print(
            f"{r['joint_name']:<13} | "
            f"{r['kp_cmd']:>7.1f} {r['kp_cmd']:>7.1f} {0.0:>+6.1f} | "
            f"{r['kd_cmd']:>7.2f} {r['kd_fit_cor']:>7.2f} {kd_ratio:>4.1f}× | "
            f"{r['omega_n_hz_cor']:>6.2f} {r['zeta_cor']:>5.2f} {flag:>4}"
        )

    # ---- Emit calibration.yaml ----
    out_path = args.out or (run_dir / "calibration.yaml")
    cal = {
        "run_id": manifest["run_id"],
        "method": "sine_2hz_chassis_compensated",
        "joint_names": JOINT_NAMES,
        "per_joint": {},
        "notes": (
            "Fit is closed-loop sine at 2 Hz with IMU-based chassis-motion "
            "compensation. kp is NOT independently identified (gain≈1 below "
            "resonance) — kp_motor reported = kp_cmd. kd_fit includes "
            "gearbox/motor friction (typically 4-8x commanded kd)."
        ),
    }
    for r in results:
        cal["per_joint"][r["joint_name"]] = {
            "kp_cmd": float(r["kp_cmd"]),
            "kd_cmd": float(r["kd_cmd"]),
            "kp_motor": float(r["kp_cmd"]),
            "kd_eff": float(r["kd_fit_cor"]),
            "I_eff": float(r["I_cor"]),
            "omega_n_hz": float(r["omega_n_hz_cor"]),
            "zeta": float(r["zeta_cor"]),
            "chassis_contam_pct": float(100 * r["chassis_ratio"]),
            "trustworthy": r["chassis_ratio"] < 0.15,
        }
    try:
        import yaml
        out_path.write_text(yaml.safe_dump(cal, sort_keys=False))
    except ImportError:
        out_path = out_path.with_suffix(".json")
        out_path.write_text(json.dumps(cal, indent=2))
    print(f"\nwrote {out_path.relative_to(run_dir)}")


if __name__ == "__main__":
    main()
