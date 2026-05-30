#!/usr/bin/env python3
"""Render Qmini joint calibration target poses.

For each joint, draws the robot with that joint at its chosen URDF
limit and all other joints at MGTO. Visual aid for hand-positioning the
robot during limit-based startq calibration.

Per-joint chosen limits come from the user's joint_range_tool sweep —
the end with smaller |Δ vs URDF| (most reliable mechanical stop).
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import mujoco
import matplotlib.pyplot as plt


# (sdk_joint_name, mjcf_joint_name, chosen_end, urdf_limit_value)
TARGETS = [
    ("hip_yaw_l",   "hip_yaw_l",     "max",  +0.70),
    ("hip_roll_l",  "hip_roll_l",    "max",  +0.60),
    ("hip_pitch_l", "hip_pitch_l",   "max",   0.00),
    ("knee_l",      "knee_pitch_l",  "min",   0.00),
    ("ankle_l",     "ankle_pitch_l", "max",   0.00),
    ("hip_yaw_r",   "hip_yaw_r",     "min",  -0.70),
    ("hip_roll_r",  "hip_roll_r",    "min",  -0.60),
    ("hip_pitch_r", "hip_pitch_r",   "min",   0.00),
    ("knee_r",      "knee_pitch_r",  "max",   0.00),
    ("ankle_r",     "ankle_pitch_r", "min",   0.00),
]

REF_JOINT_ACT = [0.4, -0.1, -1.5, 1.0, -1.3, -0.4, 0.1, 1.5, -1.0, 1.3]
MJCF_JOINT_NAMES = [t[1] for t in TARGETS]


def camera_for_joint(joint_sdk_name: str) -> mujoco.MjvCamera:
    """Pick a viewing angle that highlights the joint's rotation axis."""
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.distance = 0.90
    cam.lookat[:] = [0, 0, -0.15]  # legs are below the body origin
    is_right = joint_sdk_name.endswith("_r")

    if "yaw" in joint_sdk_name:
        # Z-axis rotation — best seen from above
        cam.elevation = -70
        cam.azimuth = 90
        cam.distance = 0.75
    elif "roll" in joint_sdk_name:
        # X-axis rotation — best seen from front
        cam.elevation = -5
        cam.azimuth = 180   # camera on +x side looking toward -x (front of robot)
    else:
        # Y-axis rotation (hip_pitch / knee / ankle) — best seen from side
        cam.elevation = -5
        cam.azimuth = 90 if not is_right else 270
    return cam


def render_at_pose(model, data, renderer, joint_qpos_addr, qpos_vec, cam):
    for nm, val in zip(MJCF_JOINT_NAMES, qpos_vec):
        data.qpos[joint_qpos_addr[nm]] = val
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=cam)
    return renderer.render()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf",
                    default="/home/woody/code/RoboTamerSdk4Qmini/sim_assets/q1_sim_hung.mjcf")
    ap.add_argument("--out",
                    default="/tmp/qmini_calibration_poses.png")
    args = ap.parse_args()

    m = mujoco.MjModel.from_xml_path(args.mjcf)
    d = mujoco.MjData(m)
    renderer = mujoco.Renderer(m, width=480, height=480)

    qadr = {}
    for nm in MJCF_JOINT_NAMES:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
        if jid < 0:
            print(f"joint not found in MJCF: {nm}", file=sys.stderr)
            return 1
        qadr[nm] = m.jnt_qposadr[jid]

    fig, axes = plt.subplots(2, 5, figsize=(22, 9.5))

    for idx, (sdk_name, mjcf_name, end, urdf_val) in enumerate(TARGETS):
        # Start at MGTO
        qpos = list(REF_JOINT_ACT)
        # Override the target joint with its URDF limit
        joint_index = MJCF_JOINT_NAMES.index(mjcf_name)
        qpos[joint_index] = urdf_val

        cam = camera_for_joint(sdk_name)
        img = render_at_pose(m, d, renderer, qadr, qpos, cam)

        ax = axes[idx // 5][idx % 5]
        ax.imshow(img)
        delta = urdf_val - REF_JOINT_ACT[joint_index]
        ax.set_title(
            f"{sdk_name}\n"
            f"{end} = {urdf_val:+.2f} rad  ({np.degrees(urdf_val):+.0f}°)\n"
            f"Δ from MGTO: {delta:+.2f} rad",
            fontsize=10
        )
        ax.axis("off")

    fig.suptitle(
        "Qmini joint calibration target poses\n"
        "Hold the robot so the highlighted joint is at the position shown\n"
        "(other joints can stay close to MGTO, exact pose not critical)",
        fontsize=13
    )
    fig.tight_layout()
    fig.savefig(args.out, dpi=110, bbox_inches="tight")
    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
