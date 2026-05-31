#!/usr/bin/env python3
"""Render the two geometric-landmark reference images.

Only ankle⟂shank and knee-straight (thigh ‖ shank) are used; hip_pitch
is then handled by IMU body-level (joint_jog_tool) once those two are
right. Both landmarks are intra-leg angles → joint values are
independent of the other two joints, so we render a single clean pose
and overlay each check on its own image.

Pose used: (hp_L, kn_L, an_L) = (-0.715, -0.084, -1.569)
  - thigh+shank colinear (knee straight)
  - foot perpendicular to shank (ankle ⟂)

Output: docs/images/geom_cal/{ankle_perp,knee_straight}.png
"""

from __future__ import annotations

import math
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import mujoco

REPO = Path("/home/woody/code/RoboTamerSdk4Qmini")
MJCF = REPO / "sim_assets/q1_sim.mjcf"
OUT  = REPO / "docs/images/geom_cal"

# Single pose for both landmarks (left leg in controller frame). Right
# mirrors per the spec; we set right leg to a neutral hanging pose so the
# left leg reads cleanly.
HP_L, KN_L, AN_L = -0.715, -0.084, -1.569

JOINT_NAMES = [
    "hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
    "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r",
]


def render_pose(m, d, renderer, base_z=0.50):
    qpos10 = [
        0.0, 0.0, HP_L, KN_L, AN_L,
        0.0, 0.0, -HP_L, -KN_L, -AN_L,
    ]
    d.qpos[:3] = [0.0, 0.0, base_z]
    d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
    for nm, q in zip(JOINT_NAMES, qpos10):
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
        d.qpos[m.jnt_qposadr[jid]] = q
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth = 90              # camera at +Y looking toward -Y
    cam.elevation = -5
    cam.distance = 0.80
    cam.lookat[:] = [0.0, 0.0, base_z - 0.18]
    renderer.update_scene(d, camera=cam)
    return renderer.render()


def body_xz(m, d, name):
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    return float(d.xpos[bid][0]), float(d.xpos[bid][2])


def project(x, z, img_w, img_h, distance, lookat_z):
    half_w = distance * math.tan(math.radians(22.5))
    half_h = half_w * (img_h / img_w)
    px = img_w * 0.5 + (x - 0.0) / (2 * half_w) * img_w
    py = img_h * 0.5 - (z - lookat_z) / (2 * half_h) * img_h
    return px, py


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    m = mujoco.MjModel.from_xml_path(str(MJCF))
    d = mujoco.MjData(m)
    W = H = 720
    try:
        renderer = mujoco.Renderer(m, width=W, height=H)
    except ValueError:
        W = H = 480
        renderer = mujoco.Renderer(m, width=W, height=H)

    img = render_pose(m, d, renderer)
    # Need base_z and camera_distance/lookat for projection math.
    base_z = float(d.qpos[2])
    cam_distance = 0.80
    cam_lookat_z = base_z - 0.18

    # Joint world positions (sagittal x, z) for overlay placement.
    hip_xz   = body_xz(m, d, "hip_pitch_l")
    knee_xz  = body_xz(m, d, "knee_pitch_l")
    ankle_xz = body_xz(m, d, "ankle_pitch_l")

    common = dict(img_w=W, img_h=H, distance=cam_distance, lookat_z=cam_lookat_z)
    p_hip    = project(*hip_xz, **common)
    p_knee   = project(*knee_xz, **common)
    p_ankle  = project(*ankle_xz, **common)

    # ------------------------------------------------------------------
    # Image 1: ankle perpendicular to shank
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img)
    ax.axis("off")
    ax.set_title("ankle perpendicular to shank   "
                 "(square at foot/shank, 90°)",
                 fontsize=14, pad=10)
    # Carpenter's square at the ankle: two perpendicular orange lines
    # following the shank direction and the foot direction.
    # Shank direction: from knee to ankle.
    dxs = p_ankle[0] - p_knee[0]
    dys = p_ankle[1] - p_knee[1]
    ns = math.hypot(dxs, dys) or 1
    us = (dxs / ns, dys / ns)
    # Foot direction: perpendicular to shank, anterior (toward foot tip).
    # Pick the direction that points generally forward in image space.
    perp_a = (us[1], -us[0])
    perp_b = (-us[1], us[0])
    # toes-forward: the foot tip body's xz roughly forward-of-ankle.
    foot_geom_id = None
    for gi in range(m.ngeom):
        bid_g = m.geom_bodyid[gi]
        if bid_g == mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY,
                                       "ankle_pitch_l"):
            foot_geom_id = gi
            break
    if foot_geom_id is not None:
        fx, fz = d.geom_xpos[foot_geom_id][0], d.geom_xpos[foot_geom_id][2]
        pf = project(fx, fz, **common)
        df_x, df_y = pf[0] - p_ankle[0], pf[1] - p_ankle[1]
        if df_x * perp_a[0] + df_y * perp_a[1] > 0:
            uf = perp_a
        else:
            uf = perp_b
    else:
        uf = perp_a

    L = 70
    # Shank leg of square (back along the shank from the ankle).
    ax.plot([p_ankle[0], p_ankle[0] - us[0] * L],
            [p_ankle[1], p_ankle[1] - us[1] * L],
            color="#ff5500", linewidth=4, solid_capstyle="round")
    # Foot leg of square (along the foot from the ankle).
    ax.plot([p_ankle[0], p_ankle[0] + uf[0] * L],
            [p_ankle[1], p_ankle[1] + uf[1] * L],
            color="#ff5500", linewidth=4, solid_capstyle="round")
    ax.annotate("90°",
                (p_ankle[0] + uf[0] * 25 - us[0] * 25,
                 p_ankle[1] + uf[1] * 25 - us[1] * 25),
                color="#ff5500", fontsize=18, fontweight="bold",
                ha="center", va="center")

    ax.text(20, H - 20,
            "Carpenter's square between the FOOT long-axis\n"
            "and the SHANK. When the corner is flush at 90°,\n"
            "read q_ankle (left leg).\n\n"
            "target  q_ankle_L = −1.569 rad  (−89.9°)\n"
            "        q_ankle_R = +1.569 rad  (+89.9°)",
            color="white", fontsize=11,
            bbox=dict(boxstyle="round,pad=0.6", facecolor="#222",
                      edgecolor="none", alpha=0.85),
            verticalalignment="bottom")
    fig.tight_layout()
    out = OUT / "ankle_perp.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # ------------------------------------------------------------------
    # Image 2: knee straight (thigh ‖ shank, 180°)
    # ------------------------------------------------------------------
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img)
    ax.axis("off")
    ax.set_title("knee straight   (thigh and shank colinear, 180°)",
                 fontsize=14, pad=10)
    # Straightedge line from above the hip through the leg, extended below
    # the ankle, lying ON the thigh+shank line.
    dxh = p_ankle[0] - p_hip[0]
    dyh = p_ankle[1] - p_hip[1]
    nh = math.hypot(dxh, dyh) or 1
    ux, uy = dxh / nh, dyh / nh
    ax.plot([p_hip[0] - 60 * ux, p_ankle[0] + 60 * ux],
            [p_hip[1] - 60 * uy, p_ankle[1] + 60 * uy],
            color="#ff5500", linewidth=4, solid_capstyle="round", alpha=0.9)
    # Mark the knee as the verified point.
    ax.add_patch(mpatches.Circle(p_knee, 14, fill=False,
                                 edgecolor="#ffd000", linewidth=3))
    ax.annotate("knee",
                (p_knee[0] + 18, p_knee[1] - 8),
                color="#ffd000", fontsize=13, fontweight="bold")
    ax.annotate("straightedge",
                (p_hip[0] - 80 * ux + 10, p_hip[1] - 80 * uy - 8),
                color="#ff5500", fontsize=12, fontweight="bold")

    ax.text(20, H - 20,
            "Lay a straightedge along the THIGH and SHANK.\n"
            "When both links lie flat on it with no gap at\n"
            "the knee, read q_knee (left leg).\n\n"
            "target  q_knee_L = −0.084 rad  (−4.8°)\n"
            "        q_knee_R = +0.084 rad  (+4.8°)\n\n"
            "NOTE: this is ~q=0 mechanical stop. If the\n"
            "knee is at the stop, you'll feel it.",
            color="white", fontsize=11,
            bbox=dict(boxstyle="round,pad=0.6", facecolor="#222",
                      edgecolor="none", alpha=0.85),
            verticalalignment="bottom")
    fig.tight_layout()
    out = OUT / "knee_straight.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # ------------------------------------------------------------------
    # Image 3: hip_yaw — leg straight forward (no splay)
    #
    # URDF hip_yaw mount rpy.z = +0.4 (L) / -0.4 (R), axis = -z. The
    # splay-cancel target is q_hy_L = +0.4 (leg in body sagittal plane).
    # We verify from TWO views in one figure:
    #   left panel:  top view (azimuth/elevation aimed straight down)
    #   right panel: side view (azimuth=90, elevation=-5)
    # The leg should align with the body forward direction in both.
    # ------------------------------------------------------------------
    # Re-pose the L leg for the hip_yaw check. Knee bent + ankle plantar-
    # flexed so the leg has shape (not a single line collapsed into the
    # body silhouette).
    HY_HP_L, HY_KN_L, HY_AN_L = -0.715, 1.0, -1.2
    HY_HY_L = +0.4                    # target

    def render_with_cam(qpos10, az, el, distance, lookat_z):
        d.qpos[:3] = [0.0, 0.0, 0.50]
        d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        for nm, q in zip(JOINT_NAMES, qpos10):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
            d.qpos[m.jnt_qposadr[jid]] = q
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(cam)
        cam.azimuth = az
        cam.elevation = el
        cam.distance = distance
        cam.lookat[:] = [0.0, 0.0, lookat_z]
        renderer.update_scene(d, camera=cam)
        return renderer.render()

    qpos_hy = [
        HY_HY_L, 0.0, HY_HP_L, HY_KN_L, HY_AN_L,
        -HY_HY_L, 0.0, -HY_HP_L, -HY_KN_L, -HY_AN_L,
    ]
    img_top  = render_with_cam(qpos_hy, az=0,  el=-89, distance=1.10, lookat_z=0.45)
    img_side = render_with_cam(qpos_hy, az=90, el=-5,  distance=0.90, lookat_z=0.32)

    fig, (ax_top, ax_side) = plt.subplots(1, 2, figsize=(16, 8))
    ax_top.imshow(img_top);  ax_top.axis("off")
    ax_top.set_title("top view — leg points forward (no splay)",
                     fontsize=13, pad=8)
    ax_side.imshow(img_side); ax_side.axis("off")
    ax_side.set_title("side view — thigh lies in body's sagittal plane",
                      fontsize=13, pad=8)

    # Top-view annotation: arrow along the body forward axis (+x), and a
    # parallel arrow showing the leg's direction. They should coincide.
    H2 = img_top.shape[0]
    W2 = img_top.shape[1]
    cx = W2 / 2
    ax_top.annotate("", xy=(cx, H2 * 0.20), xytext=(cx, H2 * 0.65),
                    arrowprops=dict(arrowstyle="->", color="#ffd000",
                                    linewidth=2.5))
    ax_top.text(cx + 18, H2 * 0.40, "body +x", color="#ffd000",
                fontsize=11, fontweight="bold")

    # Side-view annotation: vertical guideline showing body's sagittal
    # plane, plus a label.
    H3 = img_side.shape[0]; W3 = img_side.shape[1]
    ax_side.axvline(W3 / 2, color="#ffd000", linewidth=1.5,
                    linestyle=":", alpha=0.6)
    ax_side.text(W3 / 2 + 8, 30, "body sagittal plane", color="#ffd000",
                 fontsize=10, fontweight="bold")

    fig.suptitle("hip_yaw: leg straight forward (no splay)",
                 fontsize=15, y=0.99)
    fig.text(0.5, 0.04,
             "Pose the leg so it points straight in the body's +x direction.\n"
             "Verify the same alignment from both views: top (leg forward)\n"
             "and side (thigh in the body's sagittal plane, not splayed out).\n\n"
             "target  q_hip_yaw_L = +0.400 rad  (+22.9°)\n"
             "        q_hip_yaw_R = −0.400 rad  (−22.9°)",
             color="black", fontsize=11, ha="center", va="bottom",
             bbox=dict(boxstyle="round,pad=0.5", facecolor="#fff7c2",
                       edgecolor="#888"))

    fig.tight_layout(rect=[0, 0.16, 1, 0.96])
    out = OUT / "hip_yaw_forward.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # ------------------------------------------------------------------
    # Image 4: hip_roll — no lateral tilt (front view)
    #
    # Target: q_hip_roll = 0 for both legs. URDF hip_roll mount rpy = 0
    # (no offset). The verifier is the front view — looking down the body
    # +x axis, the thigh should fall in the body's frontal plane (no
    # lateral tilt), with knees vertically below the hips.
    # ------------------------------------------------------------------
    HR_HP_L, HR_KN_L, HR_AN_L = -0.715, 1.0, -1.2
    qpos_hr = [
        +0.4, 0.0, HR_HP_L, HR_KN_L, HR_AN_L,
        -0.4, 0.0, -HR_HP_L, -HR_KN_L, -HR_AN_L,
    ]
    img_front = render_with_cam(qpos_hr, az=180, el=-5,
                                distance=0.95, lookat_z=0.30)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img_front); ax.axis("off")
    ax.set_title("hip_roll: no lateral tilt (front view)",
                 fontsize=14, pad=10)
    H4 = img_front.shape[0]; W4 = img_front.shape[1]
    # Vertical guideline showing the body's centerline.
    ax.axvline(W4 / 2, color="#ffd000", linewidth=1.5,
               linestyle=":", alpha=0.6)
    ax.text(W4 / 2 + 8, 30, "body centerline (sagittal plane)",
            color="#ffd000", fontsize=10, fontweight="bold")
    ax.text(20, H4 - 20,
            "Look at the robot from in front (down the body +x axis).\n"
            "Each thigh should fall straight down with NO lateral tilt\n"
            "outward or inward. Knees vertically below the hips.\n\n"
            "target  q_hip_roll_L = 0.000 rad   (0°)\n"
            "        q_hip_roll_R = 0.000 rad   (0°)",
            color="white", fontsize=11,
            bbox=dict(boxstyle="round,pad=0.6", facecolor="#222",
                      edgecolor="none", alpha=0.85),
            verticalalignment="bottom")
    fig.tight_layout()
    out = OUT / "hip_roll_no_tilt.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # ------------------------------------------------------------------
    # Image 5: hip_pitch — thigh horizontal (bubble level on thigh,
    # torso held vertical)
    # ------------------------------------------------------------------
    HP_KN_L, HP_AN_L = 1.0, -1.2          # show a bent knee + ankle
    qpos_hp = [
        +0.4, 0.0, -0.715, HP_KN_L, HP_AN_L,
        -0.4, 0.0, +0.715, -HP_KN_L, -HP_AN_L,
    ]
    img_hp = render_with_cam(qpos_hp, az=90, el=-5,
                             distance=0.90, lookat_z=0.32)
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.imshow(img_hp); ax.axis("off")
    ax.set_title("hip_pitch: thigh horizontal   (bubble level on thigh)",
                 fontsize=14, pad=10)
    H5 = img_hp.shape[0]; W5 = img_hp.shape[1]
    # Bubble level overlay on the thigh.
    hp_xz = body_xz(m, d, "hip_pitch_l")
    kn_xz = body_xz(m, d, "knee_pitch_l")
    common_hp = dict(img_w=W5, img_h=H5, distance=0.90,
                     lookat_z=0.32)
    p_hp_b = project(*hp_xz, **common_hp)
    p_kn_b = project(*kn_xz, **common_hp)
    mx = (p_hp_b[0] + p_kn_b[0]) / 2
    my = (p_hp_b[1] + p_kn_b[1]) / 2 - 26
    ax.add_patch(mpatches.Rectangle((mx - 70, my - 9), 140, 18,
                                    linewidth=2.5, edgecolor="#ff5500",
                                    facecolor="#ffeebb"))
    ax.plot([mx], [my], "o", color="#222", markersize=11,
            markerfacecolor="#000")
    ax.annotate("bubble", (mx + 80, my - 4), color="#ff5500",
                fontsize=11, fontweight="bold")
    ax.plot([p_hp_b[0], p_kn_b[0]], [p_hp_b[1], p_kn_b[1]],
            color="#ff5500", linewidth=2, linestyle=":")

    # Vertical body-line dashed yellow to remind operator to keep torso upright.
    ax.axvline(W5 / 2, color="#ffd000", linewidth=1.5,
               linestyle=":", alpha=0.6)
    ax.text(W5 / 2 + 8, 30, "torso vertical (IMU pitch ≈ 0)",
            color="#ffd000", fontsize=10, fontweight="bold")

    ax.text(20, H5 - 20,
            "Hold the TORSO vertical — verify IMU pitch on the\n"
            "tool's status line (roll, pitch < 0.02 rad).\n"
            "Place a bubble level on the THIGH. When the bubble\n"
            "centers, the thigh is horizontal in the world.\n\n"
            "target  q_hip_pitch_L = −0.715 rad   (−40.9°)\n"
            "        q_hip_pitch_R = +0.715 rad   (+40.9°)",
            color="white", fontsize=11,
            bbox=dict(boxstyle="round,pad=0.6", facecolor="#222",
                      edgecolor="none", alpha=0.85),
            verticalalignment="bottom")
    fig.tight_layout()
    out = OUT / "hip_pitch_thigh_horizontal.png"
    fig.savefig(out, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")

    # Remove the no-longer-used images so the docs folder stays tidy.
    for stale in ("knee_perp.png", "hip_thigh_horizontal.png"):
        sp = OUT / stale
        if sp.exists():
            sp.unlink()
            print(f"removed (no longer in spec subset): {sp}")


if __name__ == "__main__":
    main()
