#!/usr/bin/env python3
"""Fit per-joint Coulomb friction f and viscous damping b from Test E
(constant-velocity sweep). At steady velocity inertia ≈ 0, so the measured
motor torque is

    tau_est ≈ f·sign(q̇) + b·q̇ + τ_grav(q)

We regress tau_est on [sign(q̇), q̇, q, 1] over the moving samples of all the
joint's Test E trials (the q and constant terms absorb the locally-linear
gravity over the small ±amp sweep). Needs ≥2 speeds to split b (slope) from
f (intercept) — the default plan runs 0.2 / 0.5 / 1.0 rad/s.

Unlike free-release (Test D) this needs NO gravity restoring and NO inertia,
so in principle it's the right method for the high-ratio hip_roll and the
no-gravity hip_yaw / ankle joints.

!! HARDWARE CAVEAT (2026-05-31): on this robot the GO-M8010-6 `tau_est` is
   unusable for friction — under-scaled (motor_unitree.cpp reports
   d.tau/ratio; joint torque is ×ratio) AND too coarse/noisy, so the friction
   signal sits at the noise floor (R²~0.5, implausibly tiny f). This script
   is correct given a good torque sensor, but needs joint-torque feedback this
   robot doesn't provide. Use Test D (position-based) instead. See
   docs/calibration_notes/friction_findings_2026_05_31.md.

Usage:
    python3 fit_friction.py path/to/run_dir/        # → friction.yaml
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import yaml

DQ_MIN = 0.05          # rad/s; only fit clearly-moving samples (skip reversals)


def fit_joint(samples):
    """samples: list of (q, dq, tau) arrays concatenated. Returns dict|None."""
    q = np.concatenate([s[0] for s in samples])
    dq = np.concatenate([s[1] for s in samples])
    tau = np.concatenate([s[2] for s in samples])
    m = np.abs(dq) > DQ_MIN
    if m.sum() < 40:
        return None, "too few moving samples"
    # Need both directions and a spread of speeds to identify f and b.
    if (dq[m] > 0).sum() < 10 or (dq[m] < 0).sum() < 10:
        return None, "need both directions"
    if dq[m].std() < 0.05:
        return None, "single speed (can't split b from f)"

    X = np.column_stack([np.sign(dq[m]), dq[m], q[m], np.ones(m.sum())])  # f,b,g,c
    coef, _r, rank, _s = np.linalg.lstsq(X, tau[m], rcond=None)
    if rank < X.shape[1]:
        return None, "rank-deficient"
    f, b, glin, c = coef
    resid = tau[m] - X @ coef
    n, p = X.shape
    sigma2 = float(resid @ resid) / max(n - p, 1)
    se = np.sqrt(np.clip(np.diag(sigma2 * np.linalg.pinv(X.T @ X)), 0, None))
    ss_tot = float(np.sum((tau[m] - tau[m].mean()) ** 2)) + 1e-12
    r2 = 1.0 - float(resid @ resid) / ss_tot
    return dict(
        f_coulomb=float(f), f_ci=[float(f - 1.96*se[0]), float(f + 1.96*se[0])],
        b_viscous=float(b), b_ci=[float(b - 1.96*se[1]), float(b + 1.96*se[1])],
        grav_slope=float(glin), r2=float(r2),
        n_samples=int(n), speeds=sorted({round(float(np.median(np.abs(s[1]))), 2)
                                         for s in samples}),
    ), "ok"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    manifest = json.loads((args.run_dir / "manifest.json").read_text())
    joint_names = manifest["joint_names"]

    per_joint_trials = {}
    for tr in manifest["trials"]:
        if tr.get("result") == "ok" and tr["test"] == "E":
            per_joint_trials.setdefault(tr["joint"], []).append(tr)
    if not per_joint_trials:
        sys.exit("no Test E (constant-velocity) trials in this run — run --tests E")

    out = {"run_id": manifest["run_id"], "joint_names": joint_names, "per_joint": {}}
    hdr = f"{'joint':<13} {'f(N·m)':>9} {'b(N·m·s/rad)':>13} {'speeds':>18} {'R²':>6}  status"
    print(hdr); print("-" * len(hdr))
    for j, trials in sorted(per_joint_trials.items()):
        name = joint_names[j]
        samples = []
        for tr in trials:
            z = np.load(args.run_dir / tr["file"], allow_pickle=False)
            samples.append((z["q"][:, j], z["dq"][:, j], z["tau_est"][:, j]))
        r, reason = fit_joint(samples)
        if r is None:
            print(f"{name:<13} {'':>42} SKIP: {reason}")
            continue
        flag = "" if (r["f_coulomb"] >= 0 and r["b_viscous"] >= 0 and r["r2"] >= 0.7) \
               else ("nonphysical" if (r["f_coulomb"] < 0 or r["b_viscous"] < 0) else "low R²")
        print(f"{name:<13} {r['f_coulomb']:>9.4f} {r['b_viscous']:>13.4f} "
              f"{str(r['speeds']):>18} {r['r2']:>6.3f}  {flag if flag else 'ok'}")
        out["per_joint"][name] = r

    out_path = args.out or (args.run_dir / "friction.yaml")
    out_path.write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nwrote {out_path}  ({len(out['per_joint'])}/{len(per_joint_trials)} joints fit)")


if __name__ == "__main__":
    main()
