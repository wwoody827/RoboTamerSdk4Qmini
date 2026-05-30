#!/usr/bin/env python3
"""Compute new startq from a joint_range_tool yaml and (optionally) write
it back into config.yaml.

Workflow:
  1. Operator runs `joint_range_tool --out joint_ranges.yaml` on the real
     robot, moving each joint to both mechanical limits.
  2. This script reads that yaml, picks the chosen URDF limit per joint
     (hard-coded below, based on the 2026-05-29 sweep), and computes
     per-joint startq corrections so that q at each chosen stop equals
     the corresponding URDF value.
  3. Output: the new startq vector. With --apply, rewrites
     config.yaml::startq in place (with .bak).

Math:
  raw_at_stop = q_chosen_extreme + startq_at_capture   (from yaml)
  new_startq  = raw_at_stop − URDF_chosen_limit
              = startq_at_capture + (q_chosen_extreme − URDF_chosen_limit)
"""

from __future__ import annotations

import argparse
import datetime as dt
import re
import shutil
import sys
from pathlib import Path

import yaml


# Per-joint chosen calibration limit (which end of the URDF range to use).
# Picked from the user's 2026-05-29 joint_range_tool sweep — the end with
# smaller |Δ vs URDF| (most reliable mechanical stop).
#   ("end", urdf_value): "end" is "min" or "max" of the yaml entry.
TARGETS = {
    # ("min" | "max" | "mid", target_q_value)
    # For joints whose URDF range spans both signs (hip_yaw, hip_roll), the
    # "0" position is mid-range — no mechanical reference. We use "mid":
    # average of measured min+max stops vs URDF midpoint. Symmetric errors
    # at both stops cancel, giving better precision than either single end.
    #
    # For pitch-chain joints (hip_pitch, knee, ankle) whose URDF range is
    # entirely one side of 0, q=0 IS the mechanical stop. Single-end is
    # already <1° accurate; midpoint would be worse (uses the far end which
    # may have larger Δ).
    "hip_yaw_l":   ("mid", -0.10, +0.70),
    "hip_roll_l":  ("mid", -0.30, +0.60),
    "hip_pitch_l": ("max",  0.00),
    "knee_l":      ("min",  0.00),
    "ankle_l":     ("max",  0.00),
    "hip_yaw_r":   ("mid", -0.70, +0.10),
    "hip_roll_r":  ("mid", -0.60, +0.30),
    "hip_pitch_r": ("min",  0.00),
    "knee_r":      ("max",  0.00),
    "ankle_r":     ("min",  0.00),
}

JOINT_ORDER = list(TARGETS.keys())


def parse_yaml(path: Path):
    """Extract startq_at_capture + measured_min/max per joint from a
    joint_range_tool yaml. The startq line lives in a comment, so we
    grep for it."""
    text = path.read_text()
    m = re.search(r"#\s*startq_at_capture:\s*\[([^\]]+)\]", text)
    if not m:
        sys.exit(f"error: no 'startq_at_capture' comment in {path}")
    startq = [float(x) for x in m.group(1).split(",")]
    if len(startq) != 10:
        sys.exit(f"error: startq_at_capture has {len(startq)} values, expected 10")
    doc = yaml.safe_load(text)
    if "joint_ranges" not in doc:
        sys.exit(f"error: {path} missing 'joint_ranges' key")
    return startq, doc["joint_ranges"]


def compute_new_startq(old_startq, joint_ranges, canonical=None):
    """Returns (new_startq, per_joint_diagnostic_table).

    If `canonical` is given (a dict from load_canonical), the target column
    becomes the canonical min/max values instead of the URDF range. This
    is the post-bootstrap mode — we trust the measured stop positions in
    URDF coords because the bootstrap pass calibrated startq accurately.
    """
    new_startq = list(old_startq)
    table = []
    for i, name in enumerate(JOINT_ORDER):
        tgt = TARGETS[name]
        if name not in joint_ranges:
            sys.exit(f"error: joint {name} not in yaml")
        meas = joint_ranges[name]
        end = tgt[0]
        # Pick the target URDF value (canonical overrides URDF if present).
        if canonical is not None and name in canonical:
            urdf_lo, urdf_hi = canonical[name]
        elif end == "mid":
            urdf_lo, urdf_hi = tgt[1], tgt[2]
        else:
            urdf_lo = urdf_hi = tgt[1]   # single-end joints
        if end == "max":
            q_target = meas["measured_max"]
            urdf_value = urdf_hi
        elif end == "min":
            q_target = meas["measured_min"]
            urdf_value = urdf_lo
        elif end == "mid":
            q_target = 0.5 * (meas["measured_min"] + meas["measured_max"])
            urdf_value = 0.5 * (urdf_lo + urdf_hi)
        else:
            sys.exit(f"unknown TARGETS end '{end}' for {name}")
        delta = q_target - urdf_value
        new_startq[i] = old_startq[i] + delta
        table.append({
            "name": name, "end": end, "urdf_limit": urdf_value,
            "q_at_extreme": q_target,
            "old_startq": old_startq[i],
            "new_startq": new_startq[i],
            "delta": delta,
        })
    return new_startq, table


def print_table(table):
    print(f"\n{'joint':<13} {'end':<4} {'urdf':>8} {'q_meas':>9} "
          f"{'old_startq':>11} {'new_startq':>11} {'Δstartq':>10}")
    print("-" * 75)
    for r in table:
        print(f"{r['name']:<13} {r['end']:<4} {r['urdf_limit']:>+8.4f} "
              f"{r['q_at_extreme']:>+9.4f} {r['old_startq']:>+11.6f} "
              f"{r['new_startq']:>+11.6f} {r['delta']:>+10.4f}")


def update_config_yaml(path: Path, new_startq):
    """Rewrite the `startq:` line in config.yaml in place, preserving
    every other line. Writes a .bak first."""
    text = path.read_text()
    backup = path.with_suffix(path.suffix + ".bak")
    shutil.copy2(path, backup)
    print(f"\nbacked up {path} → {backup}")

    new_line = "startq: [" + ", ".join(f"{v:.6f}" for v in new_startq) + "]"
    pattern = re.compile(r"^startq:\s*\[.*?\]\s*$", re.MULTILINE)
    if not pattern.search(text):
        sys.exit(f"error: no 'startq:' line found in {path}")
    new_text = pattern.sub(new_line, text)
    path.write_text(new_text)
    print(f"updated {path} startq line")


def load_canonical(path: Path):
    """Load canonical_joint_limits.yaml — per-joint TRUE URDF angles at
    mechanical stops, captured after a one-time bootstrap calibration.
    Returns a dict mapping joint name to (min, max) in URDF coords."""
    if not path.exists():
        return None
    doc = yaml.safe_load(path.read_text())
    if "canonical_limits" not in doc:
        return None
    out = {}
    for name, lim in doc["canonical_limits"].items():
        out[name] = (float(lim["min"]), float(lim["max"]))
    return out


def write_canonical(path: Path, joint_ranges):
    """Save the current measured min/max as the canonical reference. Used
    after bootstrap to lock in the calibrated mechanical-stop URDF angles."""
    import datetime as _dt
    lines = [
        "# Calibrated per-joint mechanical stop positions in URDF coordinates.",
        "# Captured after bootstrap calibration (URDF zero + IMU refinement).",
        "# Used by apply_limit_calibration.py as the target column instead",
        "# of the conservative URDF range values.",
        f"# bootstrap_date: {_dt.datetime.now().isoformat(timespec='seconds')}",
        "canonical_limits:",
    ]
    for name in JOINT_ORDER:
        meas = joint_ranges[name]
        lines.append(f"  {name:<12}: {{ min: {meas['measured_min']:+.4f}, "
                     f"max: {meas['measured_max']:+.4f} }}")
    path.write_text("\n".join(lines) + "\n")
    print(f"wrote canonical limits → {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml", type=Path,
                    help="joint_range_tool yaml file")
    ap.add_argument("--apply", action="store_true",
                    help="rewrite config.yaml::startq with the new values")
    ap.add_argument("--config", type=Path,
                    default=Path("bin/config.yaml"),
                    help="path to config.yaml (only used with --apply)")
    ap.add_argument("--canonical", type=Path,
                    default=Path("bin/canonical_joint_limits.yaml"),
                    help="canonical mechanical-stop reference yaml. If it "
                         "exists, its values override the hard-coded URDF "
                         "TARGETS. Pass /dev/null to ignore.")
    ap.add_argument("--record-canonical", action="store_true",
                    help="after bootstrap: save the current measurement as "
                         "the canonical reference for future runs. Skips the "
                         "startq computation.")
    args = ap.parse_args()

    if not args.yaml.exists():
        sys.exit(f"error: {args.yaml} not found")
    if args.apply and not args.config.exists():
        sys.exit(f"error: {args.config} not found")

    old_startq, joint_ranges = parse_yaml(args.yaml)

    # --record-canonical: bootstrap mode. Trust the input measurement as
    # ground truth and save it for future calibrations to use.
    if args.record_canonical:
        write_canonical(args.canonical, joint_ranges)
        print("\nFrom now on, apply_limit_calibration.py reads this file and\n"
              "uses these per-joint URDF angles as the calibration targets\n"
              "instead of the conservative URDF range values.")
        return

    canonical = load_canonical(args.canonical)
    if canonical is not None:
        print(f"loaded canonical limits from {args.canonical}")
    else:
        print(f"no canonical limits file at {args.canonical} — falling back to "
              f"hard-coded URDF range values. (Run with --record-canonical after "
              f"bootstrap to lock in real stops.)")

    new_startq, table = compute_new_startq(old_startq, joint_ranges, canonical)

    print_table(table)

    print(f"\nnew startq (paste into config.yaml::startq if you want):")
    print("startq: [" + ", ".join(f"{v:.6f}" for v in new_startq) + "]")

    # Quick sanity-check: |Δstartq| should be small (<10°) for every joint
    big_deltas = [(r["name"], r["delta"]) for r in table
                  if abs(r["delta"]) > 0.2]
    if big_deltas:
        print(f"\n⚠ large Δstartq (>0.2 rad ≈ 11°):")
        for name, d in big_deltas:
            print(f"  {name}: {d:+.3f} rad ({d * 180 / 3.14159:+.1f}°)")
        print("  → likely the chosen limit was reached at a different turn,")
        print("    or the URDF limit value is wrong for that joint. Double-check.")

    if args.apply:
        update_config_yaml(args.config, new_startq)
        print(f"\n✓ next time you run `run_interface`, q values will be in URDF coords.")
        print(f"  Δ vs URDF on chosen limit per joint should now be ~0 rad.")
        print(f"  re-run joint_range_tool to confirm.")
    else:
        print(f"\n(dry-run — pass --apply to write back to config.yaml)")


if __name__ == "__main__":
    main()
