#!/usr/bin/env python3
"""Fit per-joint (kp_eff, kd_eff, I_eff) from PD calibration .npz traces.

Implements PD_CALIBRATION_SPEC.md §8.  This is the offline counterpart to
`pd_calibration_tool` — it consumes a run directory written by the SDK tool
and emits `calibration.yaml`.

Usage:
    python3 fit_pd.py path/to/data/pd_calibration/2026-05-25_15-30-00_initial/

Dependencies: numpy, scipy, pyyaml. Optional: mujoco (for g_grad_urdf).
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List

import numpy as np
from scipy.optimize import minimize


# ---------------------------------------------------------------------------
# Fitting model — second-order damped oscillator at the operating point.
#
#   I_eff · q̈(t) + kd_eff · q̇(t) + kp_eff · (q(t) − q_target(t)) = 0
#
# We discretize with the trapezoid rule and fit (I_eff, kd_eff, kp_eff) by
# minimising L2 between simulated q and measured q under the same q_target.
# ---------------------------------------------------------------------------

def simulate_step(I, kd, kp, q_target, dt, q0, dq0):
    """Forward-Euler ODE integration (good enough for fit; the trial dt is
    small relative to the dynamics)."""
    q = np.empty_like(q_target)
    qd = np.empty_like(q_target)
    q[0], qd[0] = q0, dq0
    for k in range(1, len(q_target)):
        qdd = (-kd * qd[k - 1] - kp * (q[k - 1] - q_target[k - 1])) / I
        qd[k] = qd[k - 1] + dt * qdd
        q[k] = q[k - 1] + dt * qd[k]
    return q


def fit_step_trace(t, q, q_target, init=(0.01, 1.0, 30.0)):
    """Returns dict with I_eff, kd_eff, kp_eff, r2."""
    dt = float(np.mean(np.diff(t)))
    # Take only the first step transient (t in [0.5, 4]) for the fit.
    mask = (t >= 0.5) & (t <= 4.0)
    t_w = t[mask]; q_w = q[mask]; qt_w = q_target[mask]
    if len(t_w) < 30:
        return None

    def loss(params):
        I, kd, kp = params
        if I <= 0 or kd < 0 or kp < 0:
            return 1e6
        sim = simulate_step(I, kd, kp, qt_w, dt, q_w[0], (q_w[1] - q_w[0]) / dt)
        return float(np.sum((sim - q_w) ** 2))

    res = minimize(loss, x0=init, method="Nelder-Mead",
                   options={"xatol": 1e-5, "fatol": 1e-6, "maxiter": 500})
    I, kd, kp = res.x
    sim = simulate_step(I, kd, kp, qt_w, dt, q_w[0], (q_w[1] - q_w[0]) / dt)
    ss_res = np.sum((q_w - sim) ** 2)
    ss_tot = np.sum((q_w - np.mean(q_w)) ** 2) + 1e-9
    r2 = 1.0 - ss_res / ss_tot
    omega_n = np.sqrt(kp / I) / (2 * np.pi)
    zeta = kd / (2 * np.sqrt(kp * I))
    return dict(I_eff=float(I), kd_eff=float(kd), kp_eff=float(kp),
                r2=float(r2), omega_n_hz=float(omega_n), zeta=float(zeta))


# ---------------------------------------------------------------------------
# Frequency response (Test C: chirp → cross-spectrum)
# ---------------------------------------------------------------------------

def bode_from_chirp(t, q, q_target, f_targets=(0.25, 0.5, 1, 2, 4, 8)):
    """Estimate amplitude ratio + phase lag at the requested frequencies.
    Quick & dirty FFT cross-spectrum; replace with welch+coherence if noisy."""
    dt = float(np.mean(np.diff(t)))
    n = len(t)
    Q  = np.fft.rfft(q  - np.mean(q))
    QT = np.fft.rfft(q_target - np.mean(q_target))
    freqs = np.fft.rfftfreq(n, d=dt)
    H = Q / np.where(np.abs(QT) > 1e-9, QT, 1e-9)
    amp = 20 * np.log10(np.abs(H) + 1e-12)
    phase = np.angle(H, deg=True)
    out = {"f_hz": list(f_targets), "amp_db": [], "phase_deg": []}
    for f in f_targets:
        idx = int(np.argmin(np.abs(freqs - f)))
        out["amp_db"].append(float(amp[idx]))
        out["phase_deg"].append(float(phase[idx]))
    return out


# ---------------------------------------------------------------------------
# URDF gravity gradient (Y1 default path from spec §8.1)
# ---------------------------------------------------------------------------

def g_grad_urdf(mjcf_path: str, mgto: np.ndarray, eps: float = 0.01) -> np.ndarray:
    """Returns per-joint analytical gravity gradient at the calibration pose.
    Uses MuJoCo's mj_inverse on the same model the sims use.
    Returns zeros + a warning if mujoco isn't importable.
    """
    try:
        import mujoco
    except ImportError:
        print("[fit_pd] mujoco not importable — skipping g_grad subtraction; "
              "calibration.yaml will report kp_motor = kp_eff (gravity-loaded).")
        return np.zeros_like(mgto)
    if not Path(mjcf_path).exists():
        print(f"[fit_pd] MJCF not found at {mjcf_path}; skipping g_grad subtraction.")
        return np.zeros_like(mgto)
    m = mujoco.MjModel.from_xml_path(mjcf_path)
    d = mujoco.MjData(m)
    # Skip the 7 free-joint DOFs (3 trans + 4 quat) in qpos and 6 in qvel.
    nj = mgto.shape[0]
    out = np.zeros(nj)
    for j in range(nj):
        for sgn, dst in ((+1, "plus"), (-1, "minus")):
            mujoco.mj_resetData(m, d)
            d.qpos[7:7 + nj] = mgto
            d.qpos[7 + j] += sgn * eps
            d.qvel[:] = 0.0
            mujoco.mj_inverse(m, d)
            tau = d.qfrc_inverse[6 + j]
            if sgn == +1: tp = tau
            else:         tm = tau
        out[j] = (tp - tm) / (2.0 * eps)
    return out


# ---------------------------------------------------------------------------
# Orchestration
# ---------------------------------------------------------------------------

@dataclass
class JointFit:
    name: str
    kp_eff: float = float("nan")
    kd_eff: float = float("nan")
    I_eff:  float = float("nan")
    r2:     float = float("nan")
    omega_n_hz: float = float("nan")
    zeta:       float = float("nan")
    bode: Dict | None = None


def fit_run(run_dir: Path, mjcf_path: str | None = None) -> Dict:
    manifest = json.loads((run_dir / "manifest.json").read_text())
    joint_names: List[str] = manifest["joint_names"]
    mgto = np.asarray(manifest["mgto_pose"], dtype=float)
    fits = [JointFit(name=n) for n in joint_names]

    by_joint: Dict[int, list] = {i: [] for i in range(len(joint_names))}
    for trial in manifest["trials"]:
        if trial["result"] != "ok":
            continue
        by_joint[trial["joint"]].append(trial)

    for j, trials in by_joint.items():
        # Use the kp=30/kd=1 step trial as primary fit (per spec §8).
        primary = next(
            (t for t in trials
             if t["test"] == "A" and abs(t["kp"] - 30) < 0.1 and abs(t["kd"] - 1.0) < 0.1),
            None)
        if primary is None and trials:
            primary = next((t for t in trials if t["test"] == "A"), trials[0])
        if primary is not None:
            z = np.load(run_dir / primary["file"], allow_pickle=False)
            t = z["t_s"]
            q  = z["q"][:, j]
            qt = z["q_target"][:, j]
            result = fit_step_trace(t, q, qt)
            if result is not None:
                fits[j].kp_eff = result["kp_eff"]
                fits[j].kd_eff = result["kd_eff"]
                fits[j].I_eff  = result["I_eff"]
                fits[j].r2     = result["r2"]
                fits[j].omega_n_hz = result["omega_n_hz"]
                fits[j].zeta       = result["zeta"]

        # Bode from chirp trial.
        chirp = next((t for t in trials if t["test"] == "C"), None)
        if chirp is not None:
            z = np.load(run_dir / chirp["file"], allow_pickle=False)
            fits[j].bode = bode_from_chirp(z["t_s"], z["q"][:, j], z["q_target"][:, j])

    # Gravity gradient (per spec §8.1 default Y1).
    g_grad = g_grad_urdf(mjcf_path, mgto) if mjcf_path else np.zeros_like(mgto)

    out = {
        "run_id": manifest["run_id"],
        "joint_names": joint_names,
        "gravity_comp_used": False,
        "per_joint": {},
    }
    for j, f in enumerate(fits):
        out["per_joint"][f.name] = {
            "kp_eff": f.kp_eff,
            "kd_eff": f.kd_eff,
            "I_eff":  f.I_eff,
            "g_grad_urdf": float(g_grad[j]),
            "kp_motor": f.kp_eff - float(g_grad[j]),
            "omega_n_hz": f.omega_n_hz,
            "zeta": f.zeta,
            "quality_of_fit": f.r2,
            "bode": f.bode,
        }
    return out


def main():
    ap = argparse.ArgumentParser(description="Fit PD calibration traces.")
    ap.add_argument("run_dir", type=Path,
                    help="data/pd_calibration/<run_id>/")
    ap.add_argument("--mjcf",
                    default=None,
                    help="Path to q1_sim.mjcf for analytical g_grad subtraction.")
    ap.add_argument("--out", type=Path, default=None,
                    help="Output yaml path (default: <run_dir>/calibration.yaml)")
    args = ap.parse_args()

    if not args.run_dir.exists():
        print(f"error: {args.run_dir} not found", file=sys.stderr)
        sys.exit(1)
    out_path = args.out or (args.run_dir / "calibration.yaml")

    result = fit_run(args.run_dir, args.mjcf)
    try:
        import yaml
        out_path.write_text(yaml.safe_dump(result, sort_keys=False))
    except ImportError:
        print("[fit_pd] pyyaml not available; writing JSON to "
              f"{out_path.with_suffix('.json')}")
        out_path = out_path.with_suffix(".json")
        out_path.write_text(json.dumps(result, indent=2))
    print(f"wrote {out_path}")


if __name__ == "__main__":
    main()
