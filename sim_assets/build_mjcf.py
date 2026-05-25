#!/usr/bin/env python3
"""Bake a self-contained MJCF + meshes from the qmini URDF.

Output written to ``sim_assets/``:
  - ``q1_sim.mjcf``       — main model (free base, floor, actuators)
  - ``q1_sim_hung.mjcf``  — variant with the torso pinned to a fixed world
                            point (``--hang Z``); for observing motion in the
                            viewer without the robot falling
  - ``meshes/*.STL``      — copied from qmini_lab so SDK build is self-contained

Run once whenever the URDF changes. The MuJoCo HAL backend loads the model
at runtime; pass ``--mjcf <path>`` to ``run_interface`` to pick the variant.

Usage:
    python3 sim_assets/build_mjcf.py             # → q1_sim.mjcf
    python3 sim_assets/build_mjcf.py --hang 1.0  # → q1_sim_hung.mjcf
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

import mujoco

URDF_PATH = Path(
    "/home/woody/code/qmini_lab/source/qmini_lab/assets/q1/urdf/q1.urdf"
)
MESH_SRC_DIR = URDF_PATH.parent.parent / "meshes"
OUT_DIR = Path(__file__).resolve().parent
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


def build_mjcf(hang_z: float | None) -> str:
    if not URDF_PATH.exists():
        sys.exit(f"URDF not found: {URDF_PATH}")
    # Compile-time only: makes the MjSpec point mesh files at our copies.
    spec = mujoco.MjSpec.from_file(str(URDF_PATH))
    spec.meshdir = str(MESH_OUT_DIR)

    # Add a free joint to the root link so the robot can fall (or, with a
    # connect equality below, hang from a fixed world point).
    root = spec.body("base_link")
    if root is None:
        sys.exit("base_link not found")
    root.add_freejoint(name="root")
    # Spawn at the hang height if requested, else default for legs-clear.
    spawn_z = hang_z if hang_z is not None else 0.85
    root.pos = [0.0, 0.0, spawn_z]

    # Worldbody scaffolding: floor.
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5.0, 5.0, 0.1],
        pos=[0.0, 0.0, 0.0],
        rgba=[0.7, 0.7, 0.75, 1.0],
        condim=3,
        friction=[1.0, 0.005, 0.0001],
    )

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

    # Optional harness: connect the base_link origin to a world point at
    # (0, 0, hang_z). This is a 3-DOF anchor — the torso can rotate freely
    # about that point (pendulum-style) but can't translate. Good enough to
    # observe leg motion without the robot tipping or falling through floor.
    if hang_z is not None:
        spec.add_equality(
            type=mujoco.mjtEq.mjEQ_CONNECT,
            objtype=mujoco.mjtObj.mjOBJ_BODY,
            name1="base_link",
            name2="world",
            data=[0.0, 0.0, 0.0, 0.0, 0.0, hang_z, 0.0, 0.0, 0.0, 0.0, 0.0],
        )

    # Compile to validate, then dump XML.
    spec.compile()
    return spec.to_xml()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hang", type=float, default=None, metavar="Z",
                    help="Hang base_link from a fixed world point at z=Z (m). "
                         "Emits q1_sim_hung.mjcf instead of q1_sim.mjcf.")
    args = ap.parse_args()

    copy_meshes()
    xml = build_mjcf(args.hang)
    out = OUT_DIR / ("q1_sim_hung.mjcf" if args.hang is not None else "q1_sim.mjcf")
    out.write_text(xml)
    print(f"wrote {out} ({len(xml)} bytes)")
    if args.hang is not None:
        print(f"  harness: base_link anchored to world at z={args.hang} m")
    m = mujoco.MjModel.from_xml_path(str(out))
    print(f"  nq={m.nq} nv={m.nv} nu={m.nu} njnt={m.njnt} nbody={m.nbody}")


if __name__ == "__main__":
    main()
