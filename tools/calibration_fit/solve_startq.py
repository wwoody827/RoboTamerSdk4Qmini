#!/usr/bin/env python3
"""Solve startq from joint_jog_tool multi-pose records.

Workflow (see USAGE.md §6):
  1. ./bin/joint_jog_tool --gen-poses bin/cal_poses.yaml
  2. ./bin/joint_jog_tool --poses bin/cal_poses.yaml --out /tmp/recs.yaml
       — at each pose, jog joints + trim startq until the body is level
         (IMU) and the feet are forward/parallel (eyeball), press SPACE.
  3. python3 tools/calibration_fit/solve_startq.py /tmp/recs.yaml --apply

The identity (dynamic_zero = 0):

    startq_true[j] = startq[j] + (q_read[j] - q_ref[j])

holds for every record where the robot was physically at the reference
pose. We average the per-record estimate across all records and report
the spread (std) per joint — small spread = the poses agree = converged.

This refines, not replaces, the joint-limit bootstrap: re-run the jog +
solve loop until the |correction| per joint drops below your threshold
(a few mrad), then lock in the real joint limits with
apply_limit_calibration.py --record-canonical.
"""

import argparse
import re
import shutil
import sys
from pathlib import Path

import yaml

JOINT_NAMES = [
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_r",
]
N = len(JOINT_NAMES)


def mean(xs):
    return sum(xs) / len(xs)


def stdev(xs, mu):
    if len(xs) < 2:
        return 0.0
    return (sum((x - mu) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5


def update_config_yaml(path: Path, new_startq):
    """Rewrite config.yaml::startq in place (preserving everything else),
    after a .bak. Mirrors apply_limit_calibration.py."""
    text = path.read_text()
    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)
    print(f"\nbacked up {path} → {backup}")
    new_line = "startq: [" + ", ".join(f"{v:.6f}" for v in new_startq) + "]"
    pattern = re.compile(r"^startq:\s*\[.*?\]\s*$", re.MULTILINE)
    if not pattern.search(text):
        sys.exit(f"error: no 'startq:' line found in {path}")
    path.write_text(pattern.sub(new_line, text))
    print(f"updated {path} startq line")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("records", type=Path,
                    help="startq records yaml from joint_jog_tool --out")
    ap.add_argument("--apply", action="store_true",
                    help="write the solved startq into config.yaml (+ .bak)")
    ap.add_argument("--config", type=Path, default=Path("bin/config.yaml"),
                    help="config.yaml to update (default bin/config.yaml)")
    ap.add_argument("--max-std", type=float, default=0.02,
                    help="warn if a joint's per-pose spread exceeds this (rad)")
    args = ap.parse_args()

    doc = yaml.safe_load(args.records.read_text())
    recs = doc.get("records") if doc else None
    if not recs:
        sys.exit(f"no records in {args.records}")

    # Per-joint list of startq_true estimates across records.
    est = [[] for _ in range(N)]
    startq_now = None
    for r in recs:
        q_ref = r["q_ref"]
        q_read = r["q_read"]
        startq = r["startq"]
        if startq_now is None:
            startq_now = list(startq)
        for j in range(N):
            est[j].append(startq[j] + (q_read[j] - q_ref[j]))

    new_startq = [mean(est[j]) for j in range(N)]
    stds = [stdev(est[j], new_startq[j]) for j in range(N)]

    print(f"records: {len(recs)}  poses: "
          f"{', '.join(sorted(set(r['pose'] for r in recs)))}\n")
    print(f"{'joint':<13} {'old_startq':>11} {'new_startq':>11} "
          f"{'Δ':>10} {'spread(σ)':>10}  flag")
    print("-" * 72)
    noisy = []
    for j in range(N):
        d = new_startq[j] - startq_now[j]
        flag = ""
        if stds[j] > args.max_std:
            flag = "HIGH σ — poses disagree"
            noisy.append(JOINT_NAMES[j])
        print(f"{JOINT_NAMES[j]:<13} {startq_now[j]:>+11.6f} "
              f"{new_startq[j]:>+11.6f} {d:>+10.4f} {stds[j]:>10.4f}  {flag}")

    max_corr = max(abs(new_startq[j] - startq_now[j]) for j in range(N))
    print(f"\nmax |correction| = {max_corr:.4f} rad "
          f"({max_corr * 180 / 3.14159:.2f}°)")
    if noisy:
        print(f"WARNING: high per-pose spread on: {', '.join(noisy)}\n"
              f"  → those poses weren't reached consistently. Re-record or\n"
              f"    exclude bad poses before trusting their value.")

    print("\nnew startq (paste into config.yaml::startq if not using --apply):")
    print("startq: [" + ", ".join(f"{v:.6f}" for v in new_startq) + "]")

    if args.apply:
        update_config_yaml(args.config, new_startq)
        print("\n✓ startq updated. Re-run the jog+solve loop until max |correction|\n"
              "  is below your threshold (a few mrad), then re-measure limits:\n"
              "    ./bin/joint_range_tool --out /tmp/canonical.yaml\n"
              "    python3 tools/apply_limit_calibration.py /tmp/canonical.yaml "
              "--record-canonical")
    else:
        print("\n(dry run — re-run with --apply to write config.yaml)")


if __name__ == "__main__":
    main()
