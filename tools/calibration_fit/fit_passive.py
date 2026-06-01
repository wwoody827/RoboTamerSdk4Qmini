#!/usr/bin/env python3
"""Fit per-joint passive dynamics (viscous damping b, Coulomb friction f)
from Test D / free-release traces. See FREE_RELEASE_CALIBRATION_SPEC.md.

During the decay window the joint's gains are zero (released), so it obeys
the single-DOF passive equation of motion:

    I·q̈ + b·q̇ + f·sign(q̇) + g·(q − q_eq) = 0

We take the inertia `I` from the SAME run's Test A fit, then estimate the
gravity stiffness `g`, damping `b`, friction `f`, and equilibrium `q_eq`
directly from the decay by linear least-squares:

    b·q̇ + f·sign(q̇) + g·q + (−g·q_eq) = −I·(q̈ [+ α_base_proj])

(So the run must contain BOTH Test A and Test D per joint: --tests A,D.)
Estimating g from the decay — rather than from Test A's kp_eff — is robust
to a poor Test A stiffness fit, and needs no MuJoCo.

A joint is only fittable if the released joint actually SWINGS: that needs a
gravity restoring torque AND a displacement that pushes *away* from the
gravity equilibrium. Joints that barely move (hip_yaw = no gravity; a leg
displaced toward equilibrium; ankle stuck on stiction) are reported as
"insufficient swing" and skipped — re-run those with the opposite-sign
displacement (bidirectional Test D).

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

JOINT_AXIS_BODY = np.array([
    [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0],
    [0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 1, 0], [0, 1, 0],
], dtype=float)

RELEASE_T = 3.5
REENGAGE_T = 7.0
SKIP_AFTER_RELEASE = 0.1
DQ_EPS = 0.05                 # rad/s; below this Coulomb sign(q̇) is ill-defined
MIN_DECAY_RANGE = np.radians(3.0)   # need >3° of swing to fit b/f/g


def smooth_deriv(y, dt):
    if len(y) >= 21:
        return savgol_filter(y, window_length=21, polyorder=3, deriv=1, delta=dt)
    return np.gradient(y, dt)


def fit_step_baseline(t, q, q_target, init=(0.01, 1.0, 30.0)):
    """Test-A second-order fit → (I_eff, kd_eff, kp_eff). Used only for I."""
    dt = float(np.mean(np.diff(t)))
    mask = (t >= 0.5) & (t <= 4.0)
    if mask.sum() < 30:
        return None
    q_w = q[mask]
    qt_w = q_target[mask]

    def loss(p):
        I, kd, kp = p
        if I <= 0 or kd < 0 or kp < 0:
            return 1e6
        n = len(qt_w)
        qs = np.empty(n); qd = np.empty(n)
        qs[0] = q_w[0]; qd[0] = (q_w[1] - q_w[0]) / dt
        for k in range(1, n):
            qdd = (-kd * qd[k-1] - kp * (qs[k-1] - qt_w[k-1])) / I
            qd[k] = qd[k-1] + dt * qdd
            qs[k] = qs[k-1] + dt * qd[k]
        return float(np.sum((qs - q_w) ** 2))

    res = minimize(loss, x0=init, method="Nelder-Mead",
                   options={"xatol": 1e-5, "fatol": 1e-6, "maxiter": 1000})
    I, kd, kp = res.x
    return dict(I_eff=float(I), kd_eff=float(kd), kp_eff=float(kp))


def fit_passive_joint(t, q, dq, imu_omega, axis, I, imu_comp):
    """Returns (result_dict | None, reason)."""
    dt = float(np.mean(np.diff(t)))
    qdd = smooth_deriv(dq, dt)

    alpha_proj = np.zeros_like(t)
    if imu_comp and imu_omega is not None and np.all(np.isfinite(imu_omega)):
        alpha = np.vstack([smooth_deriv(imu_omega[:, k], dt) for k in range(3)]).T
        alpha_proj = alpha @ axis

    win = (t >= RELEASE_T + SKIP_AFTER_RELEASE) & (t <= REENGAGE_T)
    if win.sum() < 20:
        return None, "no decay window"
    q_range = float(q[win].max() - q[win].min())
    if q_range < MIN_DECAY_RANGE:
        return None, f"insufficient swing ({np.degrees(q_range):.1f}deg)"

    m = win & (np.abs(dq) > DQ_EPS)
    if m.sum() < 20:
        return None, "too few moving samples"

    qd = dq[m]
    X = np.column_stack([qd, np.sign(qd), q[m], np.ones_like(qd)])  # b, f, g, -g*q_eq
    rhs = -I * (qdd[m] + alpha_proj[m])

    # Rank/conditioning guard: if the swing is monotonic (q ∝ q̇), b and g are
    # not separable → ill-conditioned. Flag rather than emit a bogus split.
    cond = float(np.linalg.cond(X))
    coef, _res, rank, _sv = np.linalg.lstsq(X, rhs, rcond=None)
    if rank < X.shape[1]:
        return None, "rank-deficient (monotonic decay, b/g not separable)"
    b, f, g, c = coef
    resid = rhs - X @ coef
    n, p = X.shape
    sigma2 = float(resid @ resid) / max(n - p, 1)
    cov = sigma2 * np.linalg.pinv(X.T @ X)          # pinv: never raises
    se = np.sqrt(np.clip(np.diag(cov), 0.0, None))
    ss_tot = float(np.sum((rhs - rhs.mean()) ** 2)) + 1e-12
    r2 = 1.0 - float(resid @ resid) / ss_tot
    q_eq = (-c / g) if abs(g) > 1e-6 else float("nan")
    return dict(
        b_passive=float(b), b_ci=[float(b - 1.96*se[0]), float(b + 1.96*se[0])],
        f_coulomb=float(f), f_ci=[float(f - 1.96*se[1]), float(f + 1.96*se[1])],
        g_grad_est=float(g), q_eq=float(q_eq), I_used=float(I),
        r2=float(r2), cond=cond, decay_range_deg=float(np.degrees(q_range)),
        n_samples=int(n),
    ), "ok"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--no-imu-comp", action="store_true")
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
        sys.exit("no Test D (free-release) trials in this run — run --tests A,D")

    out = {"run_id": manifest["run_id"], "joint_names": joint_names,
           "imu_compensated": not args.no_imu_comp, "per_joint": {}}

    hdr = (f"{'joint':<13} {'b(N·m·s/rad)':>13} {'f(N·m)':>9} {'g_est':>7} "
           f"{'I':>7} {'swing°':>7} {'R²':>6}  status")
    print(hdr); print("-" * len(hdr))
    for j, dtr in sorted(d_trials.items()):
        name = joint_names[j]
        atr = a_trials.get(j)
        if atr is None:
            print(f"{name:<13} {'':>52} need Test A (for I) — run --tests A,D")
            continue
        za = np.load(args.run_dir / atr["file"], allow_pickle=False)
        a = fit_step_baseline(za["t_s"], za["q"][:, j], za["q_target"][:, j])
        if a is None:
            print(f"{name:<13} {'':>52} Test A fit failed")
            continue
        I = a["I_eff"]
        zd = np.load(args.run_dir / dtr["file"], allow_pickle=False)
        imu = zd["imu_omega"] if "imu_omega" in zd.files else None
        r, reason = fit_passive_joint(zd["t_s"], zd["q"][:, j], zd["dq"][:, j],
                                      imu, JOINT_AXIS_BODY[j], I,
                                      imu_comp=not args.no_imu_comp)
        if r is None:
            print(f"{name:<13} {'':>52} SKIP: {reason}")
            continue
        flag = ""
        if r["b_passive"] < 0 or r["f_coulomb"] < 0:
            flag = "nonphysical (b/f<0)"
        elif r["r2"] < 0.7:
            flag = "low R²"
        print(f"{name:<13} {r['b_passive']:>13.4f} {r['f_coulomb']:>9.4f} "
              f"{r['g_grad_est']:>7.2f} {I:>7.4f} {r['decay_range_deg']:>7.1f} "
              f"{r['r2']:>6.3f}  {flag if flag else 'ok'}")
        out["per_joint"][name] = r

    out_path = args.out or (args.run_dir / "passive_dynamics.yaml")
    out_path.write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nwrote {out_path}  ({len(out['per_joint'])}/{len(d_trials)} joints fit)")


if __name__ == "__main__":
    main()
