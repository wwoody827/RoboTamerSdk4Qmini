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

    # Root placement.
    # - Free-fall variant (default): add a free joint so the robot has all
    #   6 DOFs and can fall under gravity. base_link spawns at z=0.85.
    # - Hung variant (--hang Z): DON'T add a freejoint. MjSpec's compile
    #   pass then absorbs base_link into worldbody (no DOFs → fused), so
    #   any base_link.pos override is dropped. Instead we leave base_link
    #   at the origin and move the FLOOR down to z=-Z. From the user's
    #   POV the robot's body sits Z metres above the floor.
    root = spec.body("base_link")
    if root is None:
        sys.exit("base_link not found")
    if hang_z is None:
        root.add_freejoint(name="root")
        root.pos = [0.0, 0.0, 0.85]
        floor_z = 0.0
    else:
        # No freejoint → base_link gets fused to world at body.pos = (0,0,0).
        floor_z = -hang_z

    # Floor geom (we'll post-process the XML to attach a material/texture —
    # MjSpec drops textures added after a URDF-loaded compile, so we inject
    # them as text in finalize_xml() below).
    spec.worldbody.add_geom(
        name="floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5.0, 5.0, 0.1],
        pos=[0.0, 0.0, floor_z],
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

    # (Earlier versions used <equality><weld> to pin the floating base; that
    # produced a solver jolt at t=0 and ejected the robot upward in MuJoCo
    # 3.8.x. The skip-freejoint approach above sidesteps the solver entirely.)

    # Compile to validate, then dump XML.
    spec.compile()
    return finalize_xml(spec.to_xml(), hang_z)


REF_JOINT_ACT = [
    # IK-resolved standing pose at base_z=0.40, mirror-symmetric.
    # Source: qmini_lab/.../constants.py QMINI_REF_JOINT_POSES_BY_Z[3].
    +0.2346, -0.0372, -1.3862, +1.3959, -0.9879,
    -0.2346, +0.0372, +1.3862, -1.3959, +0.9879,
]


def finalize_xml(xml: str, hang_z: float | None) -> str:
    """Inject floor texture/material + light into the generated MJCF.

    MjSpec.from_file(URDF) won't keep textures added via add_texture; the
    compile pass drops them silently. We inject them as text instead.

    For the hung variant we also:
      * pump joint damping × 6 so legs settle quickly under gravity
        (URDF damping is sized for the deployed policy, far too low for a
        passive observation scene).
      * add a <keyframe name="home"> with qpos at the stand reference
        pose, so initial joint angles aren't sitting AT range limits
        (which causes constraint-solver bounce on the first few ticks).
    """
    asset_inject = (
        '    <texture name="floor_tex" type="2d" builtin="checker" '
        'rgb1="0.30 0.32 0.36" rgb2="0.50 0.52 0.56" '
        'width="300" height="300"/>\n'
        '    <material name="floor_mat" texture="floor_tex" '
        'texrepeat="5 5" texuniform="true" reflectance="0.1"/>\n'
    )
    xml = xml.replace("<asset>\n", "<asset>\n" + asset_inject, 1)
    # Reference the material on the floor geom: swap rgba=... → material=...
    # without assuming the position of other attributes (pos/size/type can
    # appear in any order).
    import re
    xml = re.sub(
        r'(<geom name="floor"[^/]*?) rgba="[^"]*"',
        r'\1 material="floor_mat"',
        xml,
    )
    if hang_z is not None:
        # The URDF visual meshes for adjacent links overlap by 5-10cm at
        # the joints (the mesh designer assumed they'd be visual-only).
        # MuJoCo treats every geom as collidable by default and only
        # excludes parent-child pairs — so base_link <-> hip_roll (a
        # grandparent-child pair) fires huge contact impulses on tick 1,
        # ejecting the legs at 40+ rad/s.
        # Strip collision on every robot geom except the floor by setting
        # contype=0 conaffinity=0.
        xml = re.sub(
            r'(<geom (?!name="floor")[^/]*?)/>',
            r'\1 contype="0" conaffinity="0"/>',
            xml,
        )
        # URDF damping (0.4-1.0 N·m·s/rad) is kept as-is. With self-collision
        # disabled (above), zero-ctrl qvel settles to <0.2 rad/s within 0.5 s
        # and full rest by 2 s. The earlier 3× / 30× boosts were masking the
        # self-collision bug; no longer needed.
        # Add a <keyframe> with qpos at the stand reference pose. Loading
        # this keyframe at startup puts joints away from their range limits
        # so the soft limit constraint doesn't fire on the first tick.
        qpos_str = " ".join(f"{v:.3f}" for v in REF_JOINT_ACT)
        kf_block = (f'  <keyframe>\n'
                    f'    <key name="home" qpos="{qpos_str}"/>\n'
                    f'  </keyframe>\n')
        xml = xml.replace("</mujoco>\n", kf_block + "</mujoco>\n", 1)
    # Add a key light at the top of worldbody.
    light_inject = (
        '    <light pos="1 -1 3" dir="-0.3 0.3 -1" '
        'diffuse="0.8 0.8 0.8" specular="0.3 0.3 0.3" castshadow="true"/>\n'
    )
    xml = xml.replace("<worldbody>\n", "<worldbody>\n" + light_inject, 1)
    return xml


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
