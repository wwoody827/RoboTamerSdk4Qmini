#!/usr/bin/env python3
"""End-to-end verification of LegIK against the actual q1_sim.mjcf MuJoCo model.

For a grid of (dx_foot, dy_foot) offsets:
  1. Read MGTO joint angles from config.yaml::ref_joint_act
  2. Compute Δq via LegIK (re-implemented here from leg_ik.cpp constants)
  3. Load q1_sim.mjcf, set qpos to MGTO + Δq, mj_forward
  4. Read ankle_pitch_l/r body positions (= foot positions in world)
  5. Verify foot displacement matches the requested (dx, dy)

If the IK constants are right, mujoco's FK should agree with our IK's FK
to ~mm precision.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import mujoco
import yaml


# Constants — match leg_ik.cpp exactly.
HP_OFFSET_X = 0.0165
HP_OFFSET_Z = 0.0
L_THIGH_X   = -0.081317
L_THIGH_Z   = -0.081317
L_SHIN_X    = 0.053013
L_SHIN_Z    = -0.14565
L_FOOT      = 0.030
THIGH_PRE_RY = 1.5
SHIN_PRE_RY  = 1.05
ANKLE_PRE_RY = 1.22
FOOT_FLAT_OFFSET = THIGH_PRE_RY + SHIN_PRE_RY + ANKLE_PRE_RY


def sign_hp(side): return +1.0 if side == 0 else -1.0
def sign_k (side): return -1.0 if side == 0 else +1.0
def sign_a (side): return +1.0 if side == 0 else -1.0


def rot_y(alpha, x, z):
    c, s = math.cos(alpha), math.sin(alpha)
    return x * c + z * s, -x * s + z * c


def leg_fk(side, q_hp, q_k, q_a):
    a_th = THIGH_PRE_RY + sign_hp(side) * q_hp
    a_sh = a_th         + SHIN_PRE_RY  + sign_k (side) * q_k
    a_ft = a_sh         + ANKLE_PRE_RY + sign_a (side) * q_a
    dx, dz = rot_y(a_th, L_THIGH_X, L_THIGH_Z)
    knee_x, knee_z = HP_OFFSET_X + dx, HP_OFFSET_Z + dz
    dx, dz = rot_y(a_sh, L_SHIN_X, L_SHIN_Z)
    ankle_x, ankle_z = knee_x + dx, knee_z + dz
    dx, dz = rot_y(a_ft, 0.0, -L_FOOT)
    return ankle_x + dx, ankle_z + dz


def foot_flat_q_a(side, q_hp, q_k):
    return (-FOOT_FLAT_OFFSET - sign_hp(side) * q_hp - sign_k(side) * q_k) / sign_a(side)


def sagittal_ik(side, target_x, target_z, q_hp0, q_k0):
    q_hp, q_k = q_hp0, q_k0
    eps = 1e-4
    for _ in range(30):
        q_a = foot_flat_q_a(side, q_hp, q_k)
        fx, fz = leg_fk(side, q_hp, q_k, q_a)
        err_x, err_z = target_x - fx, target_z - fz
        if abs(err_x) + abs(err_z) < 1e-7:
            return q_hp, q_k, q_a
        q_a_p = foot_flat_q_a(side, q_hp + eps, q_k)
        fx_h, fz_h = leg_fk(side, q_hp + eps, q_k, q_a_p)
        q_a_p = foot_flat_q_a(side, q_hp, q_k + eps)
        fx_k, fz_k = leg_fk(side, q_hp, q_k + eps, q_a_p)
        Jx_hp = (fx_h - fx) / eps; Jz_hp = (fz_h - fz) / eps
        Jx_k  = (fx_k - fx) / eps; Jz_k  = (fz_k - fz) / eps
        det = Jx_hp * Jz_k - Jx_k * Jz_hp
        if abs(det) < 1e-12: break
        dq_hp = ( Jz_k * err_x - Jx_k * err_z) / det
        dq_k  = (-Jz_hp * err_x + Jx_hp * err_z) / det
        mag = math.hypot(dq_hp, dq_k)
        if mag > 0.05:
            dq_hp *= 0.05 / mag; dq_k *= 0.05 / mag
        q_hp += dq_hp; q_k += dq_k
    q_a = foot_flat_q_a(side, q_hp, q_k)
    return q_hp, q_k, q_a


def ik_solve(ref, dx_foot, dy_foot):
    """Returns 10-vector of joint deltas."""
    dq = [0.0] * 10
    L_LEG = 0.30
    # Sagittal per leg
    for side in (0, 1):
        idx_hp, idx_k, idx_a = (2, 3, 4) if side == 0 else (7, 8, 9)
        fx0, fz0 = leg_fk(side, ref[idx_hp], ref[idx_k], ref[idx_a])
        q_hp, q_k, q_a = sagittal_ik(side, fx0 + dx_foot, fz0,
                                     ref[idx_hp], ref[idx_k])
        dq[idx_hp] = q_hp - ref[idx_hp]
        dq[idx_k]  = q_k  - ref[idx_k]
        dq[idx_a]  = q_a  - ref[idx_a]
    # Lateral
    dq_yaw = dy_foot / L_LEG
    dq[0] = -dq_yaw  # hip_yaw_l
    dq[5] = +dq_yaw  # hip_yaw_r
    return dq


# MuJoCo joint order is whatever the MJCF lists; we look it up by name.
JOINT_NAMES_SDK = [
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mjcf",   default="tests/fixtures/sim_assets/q1_sim.mjcf")
    ap.add_argument("--config", default="tests/fixtures/config.yaml")
    args = ap.parse_args()

    # ref_joint_act from config.yaml
    cfg = yaml.safe_load(Path(args.config).read_text())
    ref = list(map(float, cfg["ref_joint_act"]))
    print(f"MGTO ref_joint_act = {ref}")

    # Load MJCF
    m = mujoco.MjModel.from_xml_path(args.mjcf)
    d = mujoco.MjData(m)

    # Map SDK joint name -> qposadr
    qadr = {}
    for nm in JOINT_NAMES_SDK:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
        if j < 0:
            print(f"  WARN: joint {nm} not found in MJCF")
            return 1
        qadr[nm] = m.jnt_qposadr[j]

    # Foot bodies = ankle_pitch_{l,r}
    bL = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ankle_pitch_l")
    bR = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ankle_pitch_r")

    def set_joints_and_fk(q_vec):
        for nm, qv in zip(JOINT_NAMES_SDK, q_vec):
            d.qpos[qadr[nm]] = qv
        mujoco.mj_forward(m, d)
        # Body frame: base_link is the second body (after world).
        # base_link world position:
        base_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        base_pos = d.xpos[base_id].copy()
        base_quat = d.xquat[base_id].copy()  # w x y z
        # Foot positions in WORLD.
        fL_w = d.xpos[bL].copy()
        fR_w = d.xpos[bR].copy()
        # Transform to body frame: (foot - base) rotated by base_quat^-1
        def world_to_body(p_w):
            dp = p_w - base_pos
            # mujoco quat is (w x y z); invert by conjugate (w, -x, -y, -z)
            w, x, y, z = base_quat
            # Rotate dp by inverse quat
            qv = np.array([-x, -y, -z])
            t = 2.0 * np.cross(qv, dp)
            return dp + w * t + np.cross(qv, t)
        return world_to_body(fL_w), world_to_body(fR_w)

    # Baseline: MGTO
    fL0, fR0 = set_joints_and_fk(ref)
    print(f"\nMGTO foot positions in body frame:")
    print(f"  L = ({fL0[0]:+.4f}, {fL0[1]:+.4f}, {fL0[2]:+.4f}) m")
    print(f"  R = ({fR0[0]:+.4f}, {fR0[1]:+.4f}, {fR0[2]:+.4f}) m")

    # IK foot (sagittal x, z) at MGTO from our model:
    fxL, fzL = leg_fk(0, ref[2], ref[3], ref[4])
    fxR, fzR = leg_fk(1, ref[7], ref[8], ref[9])
    print(f"\nLegIK FK (sagittal x, z):")
    print(f"  L = ({fxL:+.4f}, {fzL:+.4f}) m  (mujoco said x={fL0[0]:+.4f}, z={fL0[2]:+.4f})")
    print(f"  R = ({fxR:+.4f}, {fzR:+.4f}) m  (mujoco said x={fR0[0]:+.4f}, z={fR0[2]:+.4f})")

    # Sweep grid
    print(f"\n{'dx_mm':>6} {'dy_mm':>6} | "
          f"{'L_dx_actual':>12} {'L_dz_actual':>12} {'L_dy_actual':>12} | "
          f"{'R_dx_actual':>12} {'R_dz_actual':>12} {'R_dy_actual':>12}")
    print("-" * 110)
    worst_err_x = 0; worst_err_y = 0; worst_err_z = 0
    for dx_mm in (-40, -20, 0, +20, +40):
        for dy_mm in (-15, 0, +15):
            dx, dy = dx_mm * 1e-3, dy_mm * 1e-3
            dq = ik_solve(ref, dx, dy)
            q_new = [ref[i] + dq[i] for i in range(10)]
            fL, fR = set_joints_and_fk(q_new)
            L_dx = fL[0] - fL0[0]; L_dy = fL[1] - fL0[1]; L_dz = fL[2] - fL0[2]
            R_dx = fR[0] - fR0[0]; R_dy = fR[1] - fR0[1]; R_dz = fR[2] - fR0[2]
            print(f"{dx_mm:>6} {dy_mm:>6} | "
                  f"{L_dx*1e3:>11.2f}mm {L_dz*1e3:>11.2f}mm {L_dy*1e3:>11.2f}mm | "
                  f"{R_dx*1e3:>11.2f}mm {R_dz*1e3:>11.2f}mm {R_dy*1e3:>11.2f}mm")
            worst_err_x = max(worst_err_x, abs(L_dx - dx), abs(R_dx - dx))
            worst_err_y = max(worst_err_y, abs(L_dy - dy), abs(R_dy + dy))  # L moves +y, R moves -y when dy_foot > 0
            worst_err_z = max(worst_err_z, abs(L_dz), abs(R_dz))

    print()
    print(f"Worst x error: {worst_err_x*1e3:.2f} mm")
    print(f"Worst y error: {worst_err_y*1e3:.2f} mm  (lateral; small angle approx → ~mm OK for ±15 mm)")
    print(f"Worst z error: {worst_err_z*1e3:.2f} mm  (foot should stay at MGTO height)")


if __name__ == "__main__":
    sys.exit(main() or 0)
