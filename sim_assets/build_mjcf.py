#!/usr/bin/env python3
"""Bake a self-contained MJCF + meshes from the qmini URDF.

Output written to ``sim_assets/``:
  - ``q1_sim.mjcf``  — main model (free base, floor, actuators)
  - ``meshes/*.STL`` — copied from qmini_lab so the SDK build is self-contained

Run once whenever the URDF changes. The MuJoCo HAL backend loads
``sim_assets/q1_sim.mjcf`` at runtime; nothing in this script is needed at
runtime.

Usage:
    /home/woody/miniconda3/envs/env_isaaclab/bin/python3 sim_assets/build_mjcf.py
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

import mujoco

URDF_PATH = Path(
    "/home/woody/code/qmini_lab/source/qmini_lab/assets/q1/urdf/q1.urdf"
)
MESH_SRC_DIR = URDF_PATH.parent.parent / "meshes"
OUT_DIR = Path(__file__).resolve().parent
MJCF_OUT = OUT_DIR / "q1_sim.mjcf"
MESH_OUT_DIR = OUT_DIR / "meshes"

JOINT_NAMES = [
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
]
# Kp/Kd as configured for sim (low — the policy provides its own gains via
# the cmd frame; these are just safeties). Order matches JOINT_NAMES.
KP_DEFAULT = [1.0] * 10
KD_DEFAULT = [0.05] * 10


def copy_meshes() -> None:
    MESH_OUT_DIR.mkdir(parents=True, exist_ok=True)
    n = 0
    for stl in MESH_SRC_DIR.glob("*.STL"):
        shutil.copy2(stl, MESH_OUT_DIR / stl.name)
        n += 1
    print(f"copied {n} meshes → {MESH_OUT_DIR}")


def build_mjcf() -> str:
    if not URDF_PATH.exists():
        sys.exit(f"URDF not found: {URDF_PATH}")
    # Compile-time only: makes the MjSpec point mesh files at our copies.
    spec = mujoco.MjSpec.from_file(str(URDF_PATH))
    spec.meshdir = str(MESH_OUT_DIR)

    # Add a free joint to the root link so the robot can fall.
    root = spec.body("base_link")
    if root is None:
        sys.exit("base_link not found")
    root.add_freejoint(name="root")
    # Spawn ~0.85 m above the floor so the legs have room to settle.
    root.pos = [0.0, 0.0, 0.85]

    # Worldbody scaffolding: floor, light.
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5.0, 5.0, 0.1],
        pos=[0.0, 0.0, 0.0],
        rgba=[0.7, 0.7, 0.75, 1.0],
        condim=3,
        friction=[1.0, 0.005, 0.0001],
    )
    # Headless build — no light needed. (`add_light` API varies by mujoco
    # version; renderable scene rigging is a downstream concern.)

    # Torque actuators, one per joint. The HAL motor backend applies PD on
    # top in C++ (kp*(q_target - q) - kd*dq + tau_ff), so MuJoCo only needs
    # plain torque actuators here.
    for jn in JOINT_NAMES:
        gainprm = [0.0] * 10
        gainprm[0] = 1.0
        biasprm = [0.0] * 10
        spec.add_actuator(
            name=f"act_{jn}",
            target=jn,
            trntype=mujoco.mjtTrn.mjTRN_JOINT,
            gainprm=gainprm,
            biasprm=biasprm,
            forcerange=[-30.0, 30.0],
        )

    # Compile to validate, then dump XML.
    spec.compile()
    return spec.to_xml()


def main() -> None:
    copy_meshes()
    xml = build_mjcf()
    MJCF_OUT.write_text(xml)
    print(f"wrote {MJCF_OUT} ({len(xml)} bytes)")
    # Sanity check: load the result back and report shape.
    m = mujoco.MjModel.from_xml_path(str(MJCF_OUT))
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt} nbody={m.nbody}")


if __name__ == "__main__":
    main()
