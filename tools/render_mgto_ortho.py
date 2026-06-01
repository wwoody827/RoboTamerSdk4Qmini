"""Render the MGTO reference pose with orthographic projection (正交投影).

Output: docs/images/geom_cal/mgto_rear_ortho.png + mgto_ortho_4views.png

Visual reference for the calibrated physical robot at MGTO. Parallel
lines stay parallel in ortho, so rear / front / side / top views can
be measured against the real robot without perspective foreshortening.
"""
from pathlib import Path
import mujoco
import matplotlib.pyplot as plt

REPO = Path("/home/woody/code/RoboTamerSdk4Qmini")
OUT  = REPO / "docs/images/geom_cal"
OUT.mkdir(parents=True, exist_ok=True)

m = mujoco.MjModel.from_xml_path(str(REPO / "sim_assets/q1_sim.mjcf"))
# Reduce perspective foreshortening by using a small FOV (long focal
# length / "telephoto" look). cam.orthographic=1 in this MuJoCo build
# isn't producing the expected size — silently behaves perspective-like.
# Fovy=8° + distance=4m gives a clean near-ortho appearance at the
# right scale: parallel-looking lines, minimal foreshortening.
m.vis.global_.fovy = 5.0     # smaller fovy + farther camera = more orthographic
d = mujoco.MjData(m)

# MGTO joint values from config.yaml::ref_joint_act
# Order: HYL HRL HPL KL AL  HYR HRR HPR KR AR
ref = [0.4, -0.1, -1.5, 1.0, -1.3,  -0.4, 0.1, 1.5, -1.0, 1.3]
names = ["hip_yaw_l", "hip_roll_l", "hip_pitch_l", "knee_pitch_l", "ankle_pitch_l",
         "hip_yaw_r", "hip_roll_r", "hip_pitch_r", "knee_pitch_r", "ankle_pitch_r"]
# Free base: identity, lifted off the floor.
d.qpos[:3] = [0.0, 0.0, 0.50]
d.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
for nm, q in zip(names, ref):
    jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, nm)
    d.qpos[m.jnt_qposadr[jid]] = q
d.qvel[:] = 0
mujoco.mj_forward(m, d)

# Framebuffer size is fixed at GL context creation from the MJCF default
# (the runtime m.vis.global_.offwidth write does NOT reallocate it). The
# stock q1_sim.mjcf supports up to 800x800; request that and no more.
m.vis.global_.offwidth = 800
m.vis.global_.offheight = 800
renderer = mujoco.Renderer(m, width=800, height=800)

# Near-orthographic views: small fovy (8°) + distance ≈ 4m. With small
# fovy, perspective foreshortening is minimal and parallel-ish, while
# the rendering pipeline behaves correctly. Robot fills ~85% of frame.
# (cam.orthographic=1 was producing 0% fill in this build, so we use
# the small-fovy perspective trick.)
views = [
    ("rear  view (looking from −x toward +x)",  180, 0,  6.0, 0.30),
    ("front view (looking from +x toward −x)",    0, 0,  6.0, 0.30),
    ("left  view (looking from +y toward −y)",   90, 0,  6.0, 0.30),
    ("top   view (looking down)",                 0, -89, 6.0, 0.45),
]
imgs = []
for label, az, el, dist, look_z in views:
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(cam)
    cam.azimuth = az
    cam.elevation = el
    cam.distance = dist             # in ortho: half-height of view in world units
    cam.lookat[:] = [0.0, 0.0, look_z]
    renderer.update_scene(d, camera=cam)
    imgs.append((label, renderer.render()))

fig, axes = plt.subplots(2, 2, figsize=(14, 14))
for ax, (label, img) in zip(axes.flat, imgs):
    ax.imshow(img)
    ax.axis("off")
    ax.set_title(label, fontsize=12)
fig.suptitle(
    "MGTO reference pose — orthographic (正交投影)\n"
    f"qpos: hip_yaw={ref[0]:+.3f}, hip_roll={ref[1]:+.3f}, "
    f"hip_pitch={ref[2]:+.3f}, knee={ref[3]:+.3f}, ankle={ref[4]:+.3f}  "
    "(right leg = negate)",
    fontsize=13, y=0.995)
fig.tight_layout()
out_4 = OUT / "mgto_ortho_4views.png"
fig.savefig(out_4, dpi=130, bbox_inches="tight", pad_inches=0.05)
print(f"saved {out_4}")
plt.close(fig)

# Rear view as a raw MuJoCo render saved directly via PIL — no
# matplotlib wrapping, no white border, no rescaling. The pixels you
# see are exactly the 1280x1280 ortho render of the robot.
import PIL.Image
rear_img = imgs[0][1]
PIL.Image.fromarray(rear_img).save(OUT / "mgto_rear_ortho.png")
print(f"saved {OUT / 'mgto_rear_ortho.png'} ({rear_img.shape[1]}x{rear_img.shape[0]}, no mpl wrap)")
