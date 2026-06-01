#!/usr/bin/env python3
"""Fit per-joint passive dynamics (viscous damping b, Coulomb friction f)
from Test D / free-release traces. See FREE_RELEASE_CALIBRATION_SPEC.md.

During the decay window the joint's gains are zero (released), so it obeys
the single-DOF passive equation of motion:

    I·q̈ + b·q̇ + f·sign(q̇) + g·(q − q_eq) = 0

We take `I` and the gravity stiffness `g` from the SAME run's Test A fit:
Test A gives `kp_eff = kp_motor + g_grad` (spec §2.1), and `kp_motor` is the
commanded PD kp, so `g = kp_eff_A − kp_commanded`, and `I = I_eff_A`. With I
and g known, the decay is a linear least-squares for (b, f) — no MuJoCo
needed (unlike fit_pd.py's --mjcf path).

So the run must contain BOTH Test A and Test D for each joint:
    ./pd_calibration_tool --i-have-checked-the-harness --tests A,D --safe-dq-max 8

Optional IMU base-motion compensation (on by default) subtracts I·α_base
projected onto the joint axis, same as fit_pd_imu_comp.py.

Usage:
    python3 fit_passive.py path/to/run_dir/         # → passive_dynamics.yaml
    python3 fit_passive.py path/to/run_dir/ --no-imu-comp
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import minimize
from scipy.signal import savgol_filter

# Joint axis in the base/body frame (index 0..9), same table as
# fit_pd_imu_comp.py — used to project IMU angular accel onto each joint.
JOINT_AXIS_BODY = np.array([
    [0, 0, 1],  # hip_yaw_l   (vertical)
    [1, 0, 0],  # hip_roll_l  (forward)
    [0, 1, 0],  # hip_pitch_l (lateral)
    [0, 1, 0],  # knee_l
    [0, 1, 0],  # ankle_l
    [0, 0, 1],  # hip_yaw_r
    [1, 0, 0],  # hip_roll_r
    [0, 1, 0],  # hip_pitch_r
    [0, 1, 0],  # knee_r
    [0, 1, 0],  # ankle_r
], dtype=float)

# Free-release phase timing (must match loop.cpp / trials.cpp).
RELEASE_T = 3.5      # gains → 0
REENGAGE_T = 7.0     # gains ramp back
SKIP_AFTER_RELEASE = 0.1   # ignore the release transient
DQ_EPS = 0.05        # rad/s; below this Coulomb sign(q̇) is ill-defined


def smooth_deriv(y: np.ndarray, dt: float) -> np.ndarray:
    """d y / dt via Savitzky-Golay (smooths numerical-diff noise)."""
    n = len(y)
    if n >= 21:
        return savgol_filter(y, window_length=21, polyorder=3, deriv=1, delta=dt)
    g = np.gradient(y, dt)
    return g


def fit_step_baseline(t, q, q_target, init=(0.01, 1.0, 30.0)):
    """Closed-loop second-order fit of Test A → (I_eff, kd_eff, kp_eff).
    Mirrors fit_pd_imu_comp.py's baseline so I/g are consistent."""
    dt = float(np.mean(np.diff(t)))
    mask = (t >= 0.5) & (t <= 4.0)
    if mask.sum() < 30:
        return None
    q_w = q[mask]
    qt_w = q_target[mask]

    def loss(params):
        I, kd, kp = params
        if I <= 0 or kd < 0 or kp < 0:
            return 1e6
        n = len(qt_w)
        qs = np.empty(n)
        qd = np.empty(n)
        qs[0] = q_w[0]
        qd[0] = (q_w[1] - q_w[0]) / dt
        for k in range(1, n):
            qdd = (-kd * qd[k - 1] - kp * (qs[k - 1] - qt_w[k - 1])) / I
            qd[k] = qd[k - 1] + dt * qdd
            qs[k] = qs[k - 1] + dt * qd[k]
        return float(np.sum((qs - q_w) ** 2))

    res = minimize(loss, x0=init, method="Nelder-Mead",
                   options={"xatol": 1e-5, "fatol": 1e-6, "maxiter": 1000})
    I, kd, kp = res.x
    return dict(I_eff=float(I), kd_eff=float(kd), kp_eff=float(kp))


def fit_passive_joint(t, q, dq, imu_omega, axis, I, g, imu_comp):
    """Linear LS for (b, f) over the decay window, with I and g known.

        b·q̇ + f·sign(q̇) − g·q_eq = −I·(q̈ [+ α_proj]) − g·q

    Regressors [q̇, sign(q̇), 1]; the constant absorbs −g·q_eq.
    Returns dict with b, f, their 95% CIs, fitted q_eq, r², n.
    """
    dt = float(np.mean(np.diff(t)))
    qdd = smooth_deriv(dq, dt)                       # q̈ from measured velocity

    alpha_proj = np.zeros_like(t)
    if imu_comp and imu_omega is not None and np.all(np.isfinite(imu_omega)):
        # α_base = d(ω)/dt projected onto the joint axis.
        alpha = np.vstack([smooth_deriv(imu_omega[:, k], dt) for k in range(3)]).T
        alpha_proj = alpha @ axis

    win = (t >= RELEASE_T + SKIP_AFTER_RELEASE) & (t <= REENGAGE_T)
    moving = np.abs(dq) > DQ_EPS
    m = win & moving
    if m.sum() < 20:
        return None

    qd = dq[m]
    sgn = np.sign(qd)
    rhs = -I * (qdd[m] + alpha_proj[m]) - g * q[m]
    X = np.column_stack([qd, sgn, np.ones_like(qd)])   # [b, f, -g*q_eq]

    coef, _res, _rank, _sv = np.linalg.lstsq(X, rhs, rcond=None)
    b, f, c = coef
    resid = rhs - X @ coef
    n, p = X.shape
    dof = max(n - p, 1)
    sigma2 = float(resid @ resid) / dof
    cov = sigma2 * np.linalg.inv(X.T @ X)
    se = np.sqrt(np.diag(cov))
    ss_tot = float(np.sum((rhs - rhs.mean()) ** 2)) + 1e-12
    r2 = 1.0 - float(resid @ resid) / ss_tot
    q_eq = (-c / g) if abs(g) > 1e-6 else float("nan")
    return dict(
        b_passive=float(b), b_ci=[float(b - 1.96 * se[0]), float(b + 1.96 * se[0])],
        f_coulomb=float(f), f_ci=[float(f - 1.96 * se[1]), float(f + 1.96 * se[1])],
        q_eq=float(q_eq), I_used=float(I), g_grad_used=float(g),
        r2=float(r2), n_samples=int(n),
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, default=None,
                    help="output yaml (default <run_dir>/passive_dynamics.yaml)")
    ap.add_argument("--no-imu-comp", action="store_true",
                    help="disable IMU base-motion compensation")
    args = ap.parse_args()

    manifest = json.loads((args.run_dir / "manifest.json").read_text())
    joint_names = manifest["joint_names"]

    a_trials, d_trials = {}, {}
    for tr in manifest["trials"]:
        if tr.get("result") != "ok":
            continue
        if tr["test"] == "A":
            a_trials[tr["joint"]] = tr
        elif tr["test"] == "D":
            d_trials[tr["joint"]] = tr

    if not d_trials:
        sys.exit("no Test D (free-release) trials in this run — run --tests D")

    out = {"run_id": manifest["run_id"], "joint_names": joint_names,
           "imu_compensated": not args.no_imu_comp, "per_joint": {}}

    hdr = (f"{'joint':<13} {'b(N·m·s/rad)':>13} {'f(N·m)':>10} "
           f"{'I_used':>8} {'g_used':>8} {'R²':>6}  flag")
    print(hdr)
    print("-" * len(hdr))
    for j, dtr in sorted(d_trials.items()):
        name = joint_names[j]
        atr = a_trials.get(j)
        if atr is None:
            print(f"{name:<13} {'— no Test A in this run (need I, g) —':>50}")
            continue
        # I and g from Test A.
        za = np.load(args.run_dir / atr["file"], allow_pickle=False)
        a = fit_step_baseline(za["t_s"], za["q"][:, j], za["q_target"][:, j])
        if a is None:
            print(f"{name:<13} {'— Test A fit failed —':>40}")
            continue
        I = a["I_eff"]
        g = a["kp_eff"] - float(atr["kp"])      # g_grad = kp_eff − kp_motor

        zd = np.load(args.run_dir / dtr["file"], allow_pickle=False)
        imu = zd["imu_omega"] if "imu_omega" in zd.files else None
        r = fit_passive_joint(zd["t_s"], zd["q"][:, j], zd["dq"][:, j],
                              imu, JOINT_AXIS_BODY[j], I, g,
                              imu_comp=not args.no_imu_comp)
        if r is None:
            print(f"{name:<13} {'— too little decay signal —':>40}")
            continue

        flag = ""
        if r["f_coulomb"] <= 0:
            flag = "f<=0 (suspect)"
        elif r["r2"] < 0.7:
            flag = "low R²"
        print(f"{name:<13} {r['b_passive']:>13.4f} {r['f_coulomb']:>10.4f} "
              f"{I:>8.4f} {g:>8.3f} {r['r2']:>6.3f}  {flag}")
        out["per_joint"][name] = r

    out_path = args.out or (args.run_dir / "passive_dynamics.yaml")
    out_path.write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nwrote {out_path}")
    print("Decompose closed-loop kd: kd_motor ≈ kd_eff(Test A) − b_passive.")


if __name__ == "__main__":
    main()
