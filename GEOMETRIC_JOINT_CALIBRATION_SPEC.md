# Geometric Per-Joint `startq` Calibration — Spec

Date: 2026-05-31
Status: Spec (ready for implementation)
Owner: SDK

This file is the source of truth for the implementing agent. Read it once,
then build the tool + render the reference images that satisfy "Acceptance
criteria" below. It is self-contained — all constants and the verified
landmark angles are derived here.

Related: [`1_calibrate_joints.md`](1_calibrate_joints.md) (the existing
startq runbook: limit bootstrap → jog refine → canonical lock). This spec
adds a **third, independent** calibration/diagnosis method for the sagittal
(pitch) chain.

---

## 1. Why this exists

`startq[j]` maps raw encoder → URDF joint angle (`q = q_raw − startq`). The
existing methods calibrate it two ways, and **both fail to pin the sagittal
(pitch) chain reliably**:

1. **IMU body-leveling** (jog tool / old ref-cal Phase 4.5) can only observe
   the **sum** of the pitch-chain joints. With the feet flat on the ground the
   torso pitch is a pure sum of the joint angles up the leg:

   ```
   θ_body  =  ankle + knee + hip_pitch        (planar chain, foot flat)
   ```

   A constant `startq` error on **any** of the three shifts `θ_body` by the
   same amount, **at every pose/height** (the sensitivity is ±1 regardless of
   configuration). So IMU body-pitch — even sampled at many heights — gives
   one equation in three unknowns. This is exactly why the multi-pose jog
   solver reports **high per-pose σ on hip_pitch / knee / ankle** (the poses
   "disagree"): the chain is under-determined. (Multi-pose *does* pin hip_yaw
   via feet-forward and hip_roll via body-roll, because each of those has a
   dedicated observable. The pitch chain does not.)

2. **Mechanical hard stops** (limit method, `apply_limit_calibration.py`) *do*
   decompose per joint — each joint has its own q=0 stop. But it assumes the
   stop sits exactly at URDF zero. A few-mrad offset there sums into a visible
   body pitch; a stop that wasn't fully reached during the sweep produces a
   **large** body tilt at MGTO. (This is the failure mode that motivated this
   spec: a large, can't-tell-which-joint body-pitch error at MGTO.)

**Geometric per-joint landmarks** give an **independent, off-stop reference
for each joint individually** — letting the operator both *diagnose* which
joint is mis-calibrated and *recalibrate* that joint directly, using a square,
a level, and a straightedge against the physical links. No IMU, no chain
ambiguity.

---

## 2. The kinematic gotcha (read this first)

**`q = 0` is NOT "links colinear" on this robot.** Each pitch joint has a
rotated mount frame in `assets/q1/urdf/q1.urdf` (and the baked
`sim_assets/q1_sim.mjcf`). The body `quat`s are all rotations about **+Y**:

| Joint | MJCF body quat (w,x,y,z) | Mount rotation about +Y |
|---|---|---|
| hip_pitch | `0.731689 0 0.681639 0` | **1.500 rad (85.9°)** |
| knee | `0.865324 0 0.501213 0` | **1.050 rad (60.2°)** |
| ankle | `0.819648 0 0.572867 0` | **1.220 rad (69.9°)** |

So a geometric pose (e.g. "foot perpendicular to shank") corresponds to a
specific **computed, non-zero** q value. Do **not** let the operator
eyeball-to-zero. The verified values are in §3.

---

## 3. Verified landmark angles

All sagittal joints and their mount frames rotate about the same +Y axis, so
the pitch chain is **planar** and solvable in closed form. The reference
computation (pure Python, no deps) is in §3.2; its output is the table below.
It reproduces the MGTO pose as a sanity check (knee bent 62.1°, foot-vs-shank
74.6°, thigh 45° down-back), which validates the model.

### 3.1 Landmark table

LEFT leg; **RIGHT leg = negate each value** (`q_r = −q_l`, the mirror
convention of `ref_joint_act`).

| Geometric pose (how the operator verifies it) | Target q — LEFT | RIGHT | Independent of stop? |
|---|---|---|---|
| **ankle: foot ⟂ shank (90°)** — square between foot long-axis and shank | **−1.569 rad (−89.9°)** | +1.569 | ✅ far from q=0 stop |
| **knee: thigh ⟂ shank (90°)** — square between thigh and shank | **+1.486 rad (+85.2°)** | −1.486 | ✅ far from q=0 stop |
| **hip_pitch: thigh horizontal** — bubble level on thigh, **torso held upright** | **−0.715 rad (−40.9°)** | +0.715 | ✅ (needs torso vertical) |
| knee: leg straight (thigh & shank colinear) — straightedge | −0.084 rad (−4.8°) | +0.084 | ✗ ≈ q=0 stop |

**Use the first two first.** `ankle ⟂ shank` and `knee ⟂ shank` are
**intra-leg** (an angle between two links of the *same* leg), so they need no
torso reference and are completely decoupled from the limit calibration and
from each other. `hip_pitch` references gravity, so the torso must be held
vertical for it.

**Diagnosis flow:** check ankle (−90°) and knee (+85°) first. If both read
their target, the body-pitch error is isolated to **hip_pitch** (check
thigh-horizontal = −0.715). Whichever joint's measured q is far from target is
the mis-calibrated one.

### 3.2 Reference computation (reproduce + basis for rendering)

```python
import math
def yang(w, sy): return 2*math.atan2(sy, w)        # +Y rotation from quat (w,0,sy,0)
m_hp = yang(0.731689,0.681639)   # 1.500
m_kn = yang(0.865324,0.501213)   # 1.050
m_an = yang(0.819648,0.572867)   # 1.220
def RY(phi,v): x,z=v; return (x*math.cos(phi)+z*math.sin(phi), -x*math.sin(phi)+z*math.cos(phi))
v_thigh=(-0.081317,-0.081317)    # knee origin in hip_pitch frame (x,z)
v_shank=( 0.053013,-0.14565)     # ankle origin in knee frame   (x,z)
def dirs(qhp,qkn,qan):           # link direction vectors in the hip_roll frame
    a_hp=m_hp+qhp                 # axes: hip_pitch +Y(+q), knee -Y(-q), ankle +Y(+q)
    a_kn=a_hp+m_kn-qkn
    a_an=a_kn+m_an+qan
    return RY(a_hp,v_thigh), RY(a_kn,v_shank), RY(a_an,(1.0,0.0))  # thigh, shank, foot+x
def ang(u,w):                    # signed angle between two (x,z) vectors, deg
    d=math.atan2(w[0],-w[1])-math.atan2(u[0],-u[1])
    return math.degrees((d+math.pi)%(2*math.pi)-math.pi)
# landmarks (bisect on the relevant joint, others at MGTO):
#   ankle ⟂ shank:  ang(shank, foot) == 90   -> q_ankle = -1.569
#   knee  ⟂ shank:  ang(thigh, shank) == 90  -> q_knee  = +1.486
#   knee  straight: ang(thigh, shank) == 0   -> q_knee  = -0.084
#   hip thigh-horizontal: thigh z-component == 0 -> q_hip = -0.715
```

The bone vectors `v_thigh`, `v_shank` are the child-body `pos` attributes from
the MJCF; the mount angles are from the body `quat`s. If the URDF changes,
re-bake the MJCF and re-run this to regenerate the table.

---

## 4. Manual procedure (works today, no new tooling)

This is the fallback / the behaviour the tool automates.

1. Motors **limp** (zero torque) — joints stay where hand-posed.
2. Pose **one** joint to its landmark, verified physically:
   - ankle ⟂ shank → carpenter's square between the foot long-axis and the shank.
   - knee ⟂ shank → square between thigh and shank.
   - thigh horizontal → bubble level on the thigh, **torso held vertical**.
3. Read that joint's `q_raw` from `./motor_status` (run from `bin/`).
4. **Diagnose:** compare `q = q_raw − startq[j]` to the §3.1 target. Off-by =
   that joint's `startq` error.
5. **Fix:** `startq_new[j] = q_raw[j] − target[j]` (pins the joint directly
   from geometry; no dependence on the old value).
6. Repeat per joint. Do ankle + knee (torso-independent) first, then hip.
7. **Re-record canonical limits** afterward so the limit method stays
   consistent (`joint_range_tool` → `apply_limit_calibration.py
   --record-canonical`), exactly as in `1_calibrate_joints.md` Stage 3.

> Note on units: `motor_status` reports `q_raw` already joint-side
> (`raw/ratio`), and `config.yaml` applies `q = q_raw − startq[j]`. The
> implementing agent must confirm `motor_status` output semantics before
> wiring the arithmetic.

---

## 5. Tool to build: limp geometric-landmark calibrator

A small interactive tool (suggest `bin/joint_geom_cal_tool`, source
`source/user/joint_geom_cal_main.cpp`; or a `--geom` mode on
`joint_jog_tool`). Reuse the jog tool's `Keyboard` class, config loader,
`save_startq_to_config()` pattern, and motor/IMU backend factory.

**Behaviour**
- Motors stay **LIMP** (zero torque) for the whole session — inherently safe,
  operator hand-poses. (No PD ramp, no MGTO drive.)
- Live table, one row per joint: current `q = q_raw − startq`, the selected
  landmark's target, and the delta. Highlight the selected joint.
- Operator selects a joint and a landmark (the four in §3.1), poses the robot
  physically, then presses a **capture** key:
  `startq[j] = q_raw[j] − target[j]`, apply via `set_zero_offset`, mark dirty.
- **Symmetric capture** (default on): also set the mirror joint
  `(j+5)%10` to the negated target from its own `q_raw` (capture both legs in
  one pose if posed symmetrically), matching the jog tool's symmetric mode.
- Show IMU rpy on a status line (operator uses it to confirm the torso is
  vertical for the thigh-horizontal landmark).
- `w` saves `startq` to `config.yaml` (+ `.bak`); `q`/`Esc` aborts and
  reverts; print a clear before/after table.
- After save, **prompt the operator to re-record canonical limits** (or do it
  in-tool: not required).

**CLI** (mirror the other tools): `--config` (default CWD `config.yaml`),
`--mjcf`/`--viewer` (sim dry-run no-op on hardware), `--iface`,
`--independent` (disable symmetric), `-h`.

**Safety:** limp only; still print a one-line reminder and require a confirm
keypress before the first capture. No watchdog needed (zero torque).

**CWD:** C++ tool loads `config.yaml` from the current dir → run from `bin/`
(same gotcha as the other C++ tools — see `1_calibrate_joints.md`).

---

## 6. Reference images to render (deliverable for the rendering agent)

Render the robot from `sim_assets/q1_sim.mjcf` (MuJoCo offscreen or viewer
screenshot), **left leg, sagittal (side, −Y looking) view**, one image per
landmark, with the joint set to its target and an overlaid annotation showing
the square/level placement and the resulting angle. Save under
`docs/images/geom_cal/` and link them from §3.1 of this doc and from
`1_calibrate_joints.md`.

Suggested full `qpos` joint vectors (10-vector, controller frame; set the
free base level). Keep non-subject joints at a clear demonstration pose so the
geometry reads cleanly:

| Image | Set the leg to (hip_pitch, knee, ankle)_L | Shows |
|---|---|---|
| `ankle_perp.png` | (−0.715, +1.486, **−1.569**) | thigh horizontal, knee 90°, **foot ⟂ shank** + square on foot/shank |
| `knee_perp.png` | (−0.715, **+1.486**, −1.569) | **thigh ⟂ shank** + square on thigh/shank |
| `hip_thigh_horizontal.png` | (**−0.715**, ~0, ~−1.0) | torso upright, **thigh level** + bubble level on thigh |
| `knee_straight.png` | (−0.715, **−0.084**, −1.2) | **leg straight** (thigh & shank colinear) + straightedge |

(Render the right leg as a mirror or annotate "right = negate." A combined
annotated diagram per landmark is fine.)

The angle definitions and the exact joint→geometry mapping are in §3.2 — the
renderer can set `qpos`, read the body frames, and draw the square/level along
the actual link axes rather than guessing.

---

## 7. Acceptance criteria

1. Tool runs with motors limp and shows, per joint, live `q`, the selected
   landmark target (from §3.1), and the delta.
2. Capturing a landmark sets `startq[j] = q_raw[j] − target[j]`, applies it
   live, and on save writes `config.yaml` (+ `.bak`). Symmetric capture sets
   the mirror joint to the negated target.
3. After capturing ankle and knee at their 90° landmarks and re-reading,
   `q` at the posed landmark equals the target to within the operator's
   measurement tolerance (≈ ±2°).
4. After calibrating ankle/knee/hip via landmarks and re-recording canonical
   limits, **body pitch at MGTO is < 2°** (verify with the IMU readout in
   `joint_jog_tool` or `ref_calibration_tool` Phase 3).
5. The four reference images in §6 are rendered, annotated, and linked from
   §3.1 and from `1_calibrate_joints.md`.
6. `GEOMETRIC_JOINT_CALIBRATION_SPEC.md` (this file) is referenced from
   `1_calibrate_joints.md` as the per-joint decomposition/diagnosis method,
   and the tool is added to `README.md` / `CLAUDE.md` file indices.

## 8. Things the implementing agent should NOT do

- Do **not** drive the joints under PD in this tool. It is a limp,
  hand-posed measurement tool; PD is unnecessary and adds risk.
- Do **not** assume `q=0` is any geometric landmark — use the §3.1 values.
- Do **not** hard-code the landmark angles without re-deriving from the MJCF
  (§3.2) if the URDF/MJCF has changed since 2026-05-31.
- Do **not** skip the canonical-limits re-record after changing `startq` —
  otherwise `apply_limit_calibration.py` will fight this calibration on the
  next reboot.
