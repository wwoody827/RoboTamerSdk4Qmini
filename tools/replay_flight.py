#!/usr/bin/env python3
"""Parse + diagnose a Qmini flight-recorder log (black box).

A flight log is <name>.bin (raw fixed-size records) + <name>.meta.json (numpy
dtype + config). This tool:

  1. Loads it in one `np.fromfile` and prints a run summary + a fall timeline.
  2. (--replay) re-runs the policy ONNX on the LOGGED obs and diffs against the
     logged raw_action — proves the on-robot ONNX path matches offline.
  3. (--recompute-obs) rebuilds the obs from the logged raw state (the way the
     C++ obs_builder does) and diffs against the LOGGED obs — catches a wrong
     obs construction on the real robot.

Usage:
    python3 replay_flight.py logs/flight/flight_<UTC>.bin
    python3 replay_flight.py <log.bin> --replay models/qmini_walk_v24/policy.onnx
    python3 replay_flight.py <log.bin> --recompute-obs
    python3 replay_flight.py <log.bin> --around-fall   # dump ticks near the fall
"""
import argparse, json, sys
from pathlib import Path
import numpy as np


def load(bin_path):
    bin_path = Path(bin_path)
    meta = json.loads(bin_path.with_suffix(".meta.json").read_text())
    dt = np.dtype([tuple(f) if len(f) == 2 else (f[0], f[1], tuple(f[2]))
                   for f in meta["dtype"]])
    assert dt.itemsize == meta["record_size_bytes"], (
        f"dtype itemsize {dt.itemsize} != record_size_bytes "
        f"{meta['record_size_bytes']} — meta/struct out of sync")
    data = np.fromfile(bin_path, dtype=dt)
    return meta, data


def fall_index(d, roll_lim=0.6, pitch_lim=0.6):
    """First tick where |roll| or |pitch| exceeds limits (rad)."""
    roll = np.abs(d["base_rpy"][:, 0]); pitch = np.abs(d["base_rpy"][:, 1])
    bad = np.where((roll > roll_lim) | (pitch > pitch_lim))[0]
    return int(bad[0]) if len(bad) else -1


def summary(meta, d):
    n = len(d)
    dt = meta["control_dt"]
    print(f"records: {n}  duration: {d['t_mono'][-1]:.2f}s  "
          f"mean dt: {np.mean(np.diff(d['t_mono']))*1000:.2f} ms "
          f"(cfg {dt*1000:.1f} ms)")
    modes, counts = np.unique(d["mode"], return_counts=True)
    print("modes:", ", ".join(f"{chr(m)}={c}" for m, c in zip(modes, counts)))
    # rate jitter — the real-robot timing risk
    ticks_dt = np.diff(d["t_mono"])
    print(f"tick dt: min {ticks_dt.min()*1000:.1f}  max {ticks_dt.max()*1000:.1f}  "
          f"p99 {np.percentile(ticks_dt,99)*1000:.1f} ms")
    # motor health
    merr = d["mot_merror"]
    if np.any(merr != 0):
        bad = np.argwhere(merr != 0)
        print(f"!! motor errors on {len(np.unique(bad[:,1]))} joints, "
              f"first at tick {bad[0,0]}")
    else:
        print("motor errors: none")
    print(f"max motor temp: {d['mot_temp'].max():.0f} C")
    fi = fall_index(d)
    if fi >= 0:
        print(f"!! FALL detected at tick {fi} (t={d['t_mono'][fi]:.2f}s): "
              f"roll={np.degrees(d['base_rpy'][fi,0]):.1f} "
              f"pitch={np.degrees(d['base_rpy'][fi,1]):.1f} deg")
    else:
        print("fall: none (stayed within +/-34 deg)")
    return fi


def around_fall(meta, d, fi, window=15):
    if fi < 0:
        print("no fall to show"); return
    lo, hi = max(0, fi - window), min(len(d), fi + 5)
    print(f"\n--- ticks {lo}..{hi} around fall (tick {fi}) ---")
    print("tick  t     mode roll  pitch  |cmd_tau~kp*err|max  max|raw_act|  max|dq|")
    for i in range(lo, hi):
        err = d["cmd_q"][i] - d["mot_q"][i]
        tau = np.abs(d["cmd_kp"][i] * err)
        flag = "  <== fall" if i == fi else ""
        print(f"{i:4d} {d['t_mono'][i]:5.2f}  {chr(d['mode'][i])}  "
              f"{np.degrees(d['base_rpy'][i,0]):+5.1f} {np.degrees(d['base_rpy'][i,1]):+5.1f}  "
              f"{tau.max():6.1f}            {np.abs(d['raw_action'][i]).max():5.2f}        "
              f"{np.abs(d['mot_dq'][i]).max():5.1f}{flag}")


def replay_onnx(meta, d, onnx_path):
    import onnxruntime as ort
    sess = ort.InferenceSession(onnx_path)
    iname = sess.get_inputs()[0].name
    mask = d["mode"] == ord('3')          # only ticks where the policy ran
    idx = np.where(mask)[0]
    if len(idx) == 0:
        print("\n[replay] no mode-3 (policy) ticks in this log"); return
    obs = d["obs"][idx].astype(np.float32)
    out = sess.run(None, {iname: obs})[0]
    diff = np.abs(out - d["raw_action"][idx])
    print(f"\n[replay] re-ran ONNX on {len(idx)} logged obs")
    print(f"  max |onnx(logged_obs) - logged_action| = {diff.max():.3e}")
    print("  -> ~0 means the on-robot ONNX inference path is faithful; a large"
          " value means the robot ran a different policy/obs than logged.")


def recompute_obs(meta, d):
    """Rebuild the NEWEST per-term frame from logged raw state and diff vs the
    newest frame inside the logged 117-obs. Catches obs-builder bugs on real."""
    ref = np.array(meta["ref_joint"], np.float32)
    TERM = [3, 3, 10, 10, 10, 3]
    def newest(o, ti):
        s = sum(TERM[:ti]) * 3; dd = TERM[ti]; return o[:, s+2*dd:s+3*dd]
    def pg(rpy):
        r, p = rpy[:, 0], rpy[:, 1]
        cp, sp, cr, sr = np.cos(p), np.sin(p), np.cos(r), np.sin(r)
        return np.stack([sp, -sr*cp, -cr*cp], axis=1)
    mask = d["mode"] == ord('3')
    if not np.any(mask):
        print("\n[recompute-obs] no mode-3 ticks"); return
    dd = d[mask]
    terms = {
        "ang_vel": (dd["base_omega"], newest(dd["obs"], 0)),
        "grav":    (pg(dd["base_rpy"]), newest(dd["obs"], 1)),
        "jpos":    (dd["ctrl_q"] - ref, newest(dd["obs"], 2)),
        "jvel":    (dd["ctrl_dq"], newest(dd["obs"], 3)),
        "command": (dd["command"], newest(dd["obs"], 5)),
    }
    print("\n[recompute-obs] logged-obs vs obs rebuilt from logged raw state:")
    for name, (mine, logged) in terms.items():
        print(f"  {name:8s}: max abs diff {np.abs(mine - logged).max():.3e}")
    print("  -> ~0 confirms the C++ obs_builder matches on real data.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="path to flight_<UTC>.bin")
    ap.add_argument("--replay", metavar="ONNX", help="re-run policy on logged obs")
    ap.add_argument("--recompute-obs", action="store_true")
    ap.add_argument("--around-fall", action="store_true")
    args = ap.parse_args()
    meta, d = load(args.log)
    print(f"schema {meta['schema']}  policy {meta['policy_path']}")
    fi = summary(meta, d)
    if args.around_fall or fi >= 0:
        around_fall(meta, d, fi)
    if args.replay:
        replay_onnx(meta, d, args.replay)
    if args.recompute_obs:
        recompute_obs(meta, d)


if __name__ == "__main__":
    main()
