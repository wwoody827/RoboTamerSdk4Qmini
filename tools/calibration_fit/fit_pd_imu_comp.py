#!/usr/bin/env python3
"""Fit per-joint PD params from Test A traces with IMU compensation for
non-rigid base motion.

The standard fit_pd.py assumes:
    I_eff · q̈ + kd · q̇ + kp · (q - q_target) = 0

But during calibration the base wasn't perfectly fixed (~5° rocking). The
joint encoder reads the relative angle between rotor and stator; if the
stator (attached to the parent link, which is attached to the base) rotates
in world frame at some α_base, the joint dynamics become:

    I_link · (q̈ + α_base_proj) + kd · q̇ + kp · (q - q_target) = 0

where α_base_proj is the base angular acceleration projected onto the joint
axis. Equivalently:

    I_link · q̈ + kd · q̇ + kp · (q - q_target) = -I_link · α_base_proj

i.e., a known forcing term on the right-hand side.

The fix: measure α_base from IMU, project onto each joint's axis, and add as
a known disturbance during the fit.

Usage:
    python3 fit_pd_imu_comp.py path/to/run_dir/
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize


# Joint axis in the BODY (base) frame. Small-angle approximation around
# MGTO — for our 5° base excursion the axes are accurate to within 1%.
#
#   hip_yaw   — rotates around vertical (body z)
#   hip_roll  — rotates around forward  (body x)
#   hip_pitch — rotates around lateral  (body y)
#   knee      — rotates around lateral  (body y)
#   ankle     — rotates around lateral  (body y)
#
# Index 0..9 in joint_under_test order.
JOINT_AXIS_BODY = np.array([
    [0, 0, 1],  # hip_yaw_l
    [1, 0, 0],  # hip_roll_l
    [0, 1, 0],  # hip_pitch_l
    [0, 1, 0],  # knee_l
    [0, 1, 0],  # ankle_l
    [0, 0, 1],  # hip_yaw_r
    [1, 0, 0],  # hip_roll_r
    [0, 1, 0],  # hip_pitch_r
    [0, 1, 0],  # knee_r
    [0, 1, 0],  # ankle_r
], dtype=float)


def base_angular_accel(imu_omega: np.ndarray, dt: float) -> np.ndarray:
    """Differentiate IMU angular velocity (N,3) to get angular acceleration
    (N,3). Uses central differences; pads endpoints."""
    n = imu_omega.shape[0]
    alpha = np.zeros_like(imu_omega)
    alpha[1:-1] = (imu_omega[2:] - imu_omega[:-2]) / (2 * dt)
    alpha[0]    = (imu_omega[1] - imu_omega[0]) / dt
    alpha[-1]   = (imu_omega[-1] - imu_omega[-2]) / dt
    # Light low-pass to suppress numerical-diff noise
    from scipy.signal import savgol_filter
    if n >= 21:
        alpha = savgol_filter(alpha, window_length=21, polyorder=3, axis=0)
    return alpha


def simulate_step_with_forcing(I, kd, kp, q_target, dt, q0, dq0, alpha_proj):
    """Forward-Euler integration of:
        q̈ = (-kd·q̇ - kp·(q - q_target) - I·alpha_proj) / I
    alpha_proj : np.ndarray (N,) — known forcing per step.
    """
    n = len(q_target)
    q = np.empty(n)
    qd = np.empty(n)
    q[0], qd[0] = q0, dq0
    for k in range(1, n):
        qdd = (-kd * qd[k-1] - kp * (q[k-1] - q_target[k-1]) - I * alpha_proj[k-1]) / I
        qd[k] = qd[k-1] + dt * qdd
        q[k]  = q[k-1] + dt * qd[k]
    return q


def fit_with_compensation(t, q, q_target, imu_omega, joint_axis,
                          init=(0.01, 1.0, 30.0)):
    """Fit (I_link, kd, kp) to step trace with IMU base-motion compensation."""
    dt = float(np.mean(np.diff(t)))
    mask = (t >= 0.5) & (t <= 4.0)
    if mask.sum() < 30:
        return None
    t_w = t[mask]; q_w = q[mask]; qt_w = q_target[mask]
    alpha = base_angular_accel(imu_omega, dt)            # (N,3)
    alpha_proj = alpha @ joint_axis                       # (N,)
    alpha_w = alpha_proj[mask]

    def loss(params):
        I, kd, kp = params
        if I <= 0 or kd < 0 or kp < 0:
            return 1e6
        sim = simulate_step_with_forcing(I, kd, kp, qt_w, dt, q_w[0],
                                          (q_w[1] - q_w[0]) / dt, alpha_w)
        return float(np.sum((sim - q_w) ** 2))

    res = minimize(loss, x0=init, method="Nelder-Mead",
                   options={"xatol": 1e-5, "fatol": 1e-6, "maxiter": 1000})
    I, kd, kp = res.x
    sim = simulate_step_with_forcing(I, kd, kp, qt_w, dt, q_w[0],
                                      (q_w[1] - q_w[0]) / dt, alpha_w)
    ss_res = np.sum((q_w - sim) ** 2)
    ss_tot = np.sum((q_w - np.mean(q_w)) ** 2) + 1e-9
    r2 = 1.0 - ss_res / ss_tot
    omega_n = np.sqrt(kp / I) / (2 * np.pi)
    zeta = kd / (2 * np.sqrt(kp * I))
    return dict(I_link=float(I), kd_eff=float(kd), kp_eff=float(kp),
                r2=float(r2), omega_n_hz=float(omega_n), zeta=float(zeta),
                base_alpha_rms=float(np.sqrt(np.mean(alpha_w ** 2))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args()

    manifest = json.loads((args.run_dir / "manifest.json").read_text())
    joint_names = manifest["joint_names"]

    # Build per-joint dict of step trials
    step_trials = {}
    for trial in manifest["trials"]:
        if trial["result"] != "ok" or trial["test"] != "A":
            continue
        step_trials[trial["joint"]] = trial

    out = {"run_id": manifest["run_id"], "joint_names": joint_names,
           "per_joint": {}}

    print(f"{'joint':<14} {'kp_orig':>8} {'kp_comp':>8} {'kd_orig':>8} {'kd_comp':>8} "
          f"{'I_orig':>8} {'I_comp':>8} {'α_rms':>8} {'R²':>6}")
    # Baseline fit (no IMU compensation) inlined.
    def fit_step_baseline(t, q, q_target, init=(0.01, 1.0, 30.0)):
        dt = float(np.mean(np.diff(t)))
        mask = (t >= 0.5) & (t <= 4.0)
        if mask.sum() < 30:
            return None
        t_w = t[mask]; q_w = q[mask]; qt_w = q_target[mask]
        def loss(params):
            I, kd, kp = params
            if I <= 0 or kd < 0 or kp < 0: return 1e6
            n = len(qt_w)
            q_sim = np.empty(n); qd_sim = np.empty(n)
            q_sim[0] = q_w[0]; qd_sim[0] = (q_w[1] - q_w[0]) / dt
            for k in range(1, n):
                qdd = (-kd * qd_sim[k-1] - kp * (q_sim[k-1] - qt_w[k-1])) / I
                qd_sim[k] = qd_sim[k-1] + dt * qdd
                q_sim[k] = q_sim[k-1] + dt * qd_sim[k]
            return float(np.sum((q_sim - q_w) ** 2))
        res = minimize(loss, x0=init, method="Nelder-Mead",
                       options={"xatol": 1e-5, "fatol": 1e-6, "maxiter": 1000})
        I, kd, kp = res.x
        return dict(I_eff=float(I), kd_eff=float(kd), kp_eff=float(kp))

    for j, trial in step_trials.items():
        z = np.load(args.run_dir / trial["file"], allow_pickle=False)
        t = z["t_s"]
        q  = z["q"][:, j]
        qt = z["q_target"][:, j]
        imu_omega = z["imu_omega"]
        axis = JOINT_AXIS_BODY[j]

        # Baseline (no IMU comp)
        b = fit_step_baseline(t, q, qt)
        # Compensated
        c = fit_with_compensation(t, q, qt, imu_omega, axis)
        if b is None or c is None:
            continue

        name = joint_names[j]
        print(f"{name:<14} {b['kp_eff']:>8.2f} {c['kp_eff']:>8.2f} "
              f"{b['kd_eff']:>8.3f} {c['kd_eff']:>8.3f} "
              f"{b['I_eff']:>8.4f} {c['I_link']:>8.4f} "
              f"{c['base_alpha_rms']:>8.3f} {c['r2']:>6.3f}")

        out["per_joint"][name] = {
            "kp_eff_compensated": c['kp_eff'],
            "kd_eff_compensated": c['kd_eff'],
            "I_link":            c['I_link'],
            "omega_n_hz":        c['omega_n_hz'],
            "zeta":              c['zeta'],
            "r2":                c['r2'],
            "base_alpha_rms":    c['base_alpha_rms'],
            "kp_eff_baseline":   b['kp_eff'],
            "kd_eff_baseline":   b['kd_eff'],
            "I_eff_baseline":    b['I_eff'],
        }

    out_path = args.run_dir / "calibration_imu_comp.yaml"
    out_path.write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
