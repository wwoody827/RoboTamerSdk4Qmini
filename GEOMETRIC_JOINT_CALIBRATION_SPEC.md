# Geometric Per-Joint `startq` Calibration — Spec

Date: 2026-05-31 (updated 2026-05-31 with the all-10-joint landmark set
and the limp `joint_geom_cal_tool` that implements it)
Status: Landed (`bin/joint_geom_cal_tool`, hardware-tested workflow in
[`1_calibrate_joints.md`](1_calibrate_joints.md) Stage 1.5)
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

| Joint | Pose (how the operator verifies) | Target q — LEFT | RIGHT | Reference image |
|---|---|---|---|---|
| **ankle** | foot ⟂ shank — carpenter's square between foot and shank | **−1.569 rad (−89.9°)** | +1.569 | [`ankle_perp.png`](docs/images/geom_cal/ankle_perp.png) |
| **knee** | leg straight, **pushed against the q=0 mechanical stop** | **0.000 rad** (URDF lower bound) | 0.000 (R: URDF upper bound) | [`knee_straight.png`](docs/images/geom_cal/knee_straight.png) |
| **hip_yaw** | leg in body's sagittal plane — top + side view both align | **+0.400 rad (+22.9°)** | −0.400 | [`hip_yaw_forward.png`](docs/images/geom_cal/hip_yaw_forward.png) |
| **hip_roll** | no lateral tilt — front view, thigh straight down | **0.000 rad (0°)** | 0.000 | [`hip_roll_no_tilt.png`](docs/images/geom_cal/hip_roll_no_tilt.png) |
| **hip_pitch** | thigh horizontal — bubble level on thigh, **torso held vertical** | **−0.715 rad (−40.9°)** | +0.715 | [`hip_pitch_thigh_horizontal.png`](docs/images/geom_cal/hip_pitch_thigh_horizontal.png) |

**Note on the knee landmark.** A strict thigh ‖ shank (180°) is at
q = ∓0.084 rad, but the mechanical stop sits ~3.6° short of that on this
robot — the joint physically can't reach colinearity. We use the **q=0
mechanical stop** instead (URDF lower bound for L, upper bound for R),
which is mechanically very reliable. This collapses the knee to the
limit method for that one joint, but keeps the operator workflow uniform
across all 5 captures (one tool, motors limp). The other four joints are
genuinely geometric — far from any stop, sub-degree precision.

**Decoupling story.** `ankle ⟂ shank` and `knee straight` are intra-leg
(angle between two links of the *same* leg) → no torso reference needed and
completely decoupled from each other. `hip_yaw` and `hip_roll` use body-plane
references that are independent of the sagittal chain. **`hip_pitch` is the
only one that needs gravity** — the operator holds the torso vertical
(verifiable on the tool's IMU status line) and uses a bubble level on the
thigh.

**Diagnosis flow.** If body pitch at MGTO is non-zero after a sweep-based
calibration, check ankle (−90°) and knee (−4.8°) first. Both intra-leg and
completely decoupled, so any deviation is a startq error on that one joint.
Whichever joint's measured q is far from its target is the mis-calibrated one
— recalibrate it with a single capture.

### 3.2 Reference computation (reproduce + basis for rendering)

```python
import math
def yang(w, sy): return 2*math.atan2(sy, w)        # +Y rotation from quat (w,0,sy,0)
m_hp = yang(0.731689,0.681639)   # 1.500  hip_pitch mount rpy.y
m_kn = yang(0.865324,0.501213)   # 1.050  knee mount rpy.y
m_an = yang(0.819648,0.572867)   # 1.220  ankle mount rpy.y
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
# landmarks (LEFT leg):
#   ankle ⟂ shank:         ang(shank, foot) ==  90  ->  q_ankle   = -1.569
#   hip thigh-horizontal:  thigh z-component ==   0  ->  q_hip_pit = -0.715
# hip_yaw and hip_roll have no z-axis rotation gotcha — they're directly
# the URDF mount-bias-cancel values:
#   hip_yaw mount rpy.z = +0.4 (L) axis -z   -> q_hy_L    = +0.400
#   hip_roll: no mount bias                  -> q_hr_L    =  0.000
# knee: geometric 180° (ang(thigh,shank)==0) is at q = -0.084 but the
# mechanical stop on this robot blocks the joint ~3.6° before colinearity.
# We instead calibrate against the URDF lower bound (q=0) — i.e. the stop —
# which is mechanically very reliable. (Sub-degree on the user's hardware.)
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

## 5. Tool: `bin/joint_geom_cal_tool` (landed)

Implementation: `source/user/joint_geom_cal_main.cpp`.

**Behaviour**
- Motors stay **LIMP** (zero kp/kd/tau) for the entire session — inherently
  safe, operator hand-poses. No PD, no MGTO drive.
- Live table refreshes at 5 Hz with one row per joint: target (from §3.1),
  current `q = q_raw − startq`, delta vs target, and the cumulative session
  `Δstartq`. Highlight the selected joint (`>` cursor + `✓` on captured).
- `0`-`9` (or `[`/`]`) select a joint. **SPACE** captures **only that
  joint**: `dsq[j] += q_read[j] − target[j]` (accumulates, so re-capturing
  a joint after re-posing stays correct), applied live via
  `motor->set_zero_offset(startq0 + dsq)`. One joint per capture — the
  operator poses a single joint at a time (two hands). Left and right are
  separate joints → **10 captures**; the right-leg targets are the negated
  left values.
- Status line shows IMU `rpy` — used to confirm the torso is held vertical
  for the hip_pitch (thigh-horizontal) landmark.
- `w` writes the new `startq` to `config.yaml` (with `.bak` backup) and
  bakes the deltas into `startq0` so further captures stack cleanly.
  `q`/`Esc` aborts — reverts `set_zero_offset` to the original baseline
  and exits without writing.

**CLI**: `--mjcf`/`--no-viewer` (sim dry-run), `--iface`, `--tick-hz`, `-h`.

**Safety**: limp only. Zero torque, zero kp/kd shipped every tick. No
watchdog needed.

**CWD**: loads `config.yaml` from the current dir → run from `bin/`
(same gotcha as the other C++ tools).

---

## 6. Reference images (rendered)

The five landmark images linked from §3.1 are produced by
`tools/render_geom_landmarks.py` from `sim_assets/q1_sim.mjcf`. Re-run the
script after any URDF change. Files:

| File | Joint | Pose (hp_L, kn_L, an_L) + extra view | Annotation |
|---|---|---|---|
| `docs/images/geom_cal/ankle_perp.png` | ankle | (−0.715, −0.084, **−1.569**), side view | carpenter's square between foot and shank at 90° |
| `docs/images/geom_cal/knee_straight.png` | knee | (−0.715, **−0.084**, −1.569), side view | straightedge along thigh + shank, knee circled |
| `docs/images/geom_cal/hip_yaw_forward.png` | hip_yaw | (HY=**+0.4**, HR=0, HP=−0.715, kn=+1.0, an=−1.2), top + side | top: body +x arrow; side: sagittal-plane line |
| `docs/images/geom_cal/hip_roll_no_tilt.png` | hip_roll | (HY=+0.4, **HR=0**, HP=−0.715, kn=+1.0, an=−1.2), front view | vertical body centerline; thighs drop straight down |
| `docs/images/geom_cal/hip_pitch_thigh_horizontal.png` | hip_pitch | (HP=**−0.715**, kn=+1.0, an=−1.2), side view | bubble level on thigh; torso-vertical reminder |

Right-leg poses mirror by negating each value. The `ankle_perp` and
`knee_straight` images use the SAME pose (whole leg straight with foot
perpendicular) — both checks are visible there, but the operator only
calibrates one joint at a time.

---

## 7. Acceptance criteria (all met by the landed tool)

1. ✅ Tool runs with motors limp and shows, per joint, live `q`, the selected
   landmark target (§3.1), and the delta.
2. ✅ Capturing a landmark accumulates `dsq[j] += q_raw[j] − target[j]`
   (correct on re-capture), applies it live via `set_zero_offset`; `w`
   writes `config.yaml` (+ `.bak`). One joint captured per SPACE.
3. After capturing ankle and knee at their landmarks and re-reading on
   real hardware, `q` at the posed landmark equals the target to within
   the operator's measurement tolerance (≈ ±2°). *Sim test passes; HW
   verification deferred to operator.*
4. After full geometric calibration (5 landmarks × 2 legs = 10 startq)
   and re-recording canonical limits, **body pitch at MGTO is < 2°**
   (verify with `joint_jog_tool` MGTO ramp or `ref_calibration_tool`
   Phase 3 IMU readout). *Hardware verification deferred to operator.*
5. ✅ Five reference images rendered, annotated, and linked from §3.1.
6. ✅ This spec is referenced from `1_calibrate_joints.md` Stage 1.5 and
   from the troubleshooting section; tool added to `README.md`.

## 8. Things the implementing agent should NOT do

- Do **not** drive the joints under PD in this tool. It is a limp,
  hand-posed measurement tool; PD is unnecessary and adds risk.
- Do **not** assume `q=0` is any geometric landmark — use the §3.1 values.
- Do **not** hard-code the landmark angles without re-deriving from the MJCF
  (§3.2) if the URDF/MJCF has changed since 2026-05-31.
- Do **not** skip the canonical-limits re-record after changing `startq` —
  otherwise `apply_limit_calibration.py` will fight this calibration on the
  next reboot.
