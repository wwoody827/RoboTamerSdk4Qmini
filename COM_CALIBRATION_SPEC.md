# Center-of-Mass / Reference-Pose Calibration

Date: 2026-05-29
Owner: SDK
Status: Landed (commit on hal-refactor branch). Operator usage docs in
        `USAGE.md §6`. This file is the design rationale.

---

## 1. Why

Symptom: when the real Qmini is placed on the floor in the trained
**MGTO crouch pose** (`bin/config.yaml::ref_joint_act`), it **falls
forward**. In sim (Isaac Lab / MuJoCo at the same pose) it stands.

That means the **real CoM does not lie above the foot center polygon**
at the MGTO joint angles, while the URDF CoM does.

Hypotheses, ordered by likelihood:

1. The head / chest assembly is heavier than URDF (added cabling,
   battery harness, IMU board moved). This shifts CoM forward and up.
2. Battery position differs from URDF.
3. Foot geometry / contact center differs from URDF (e.g. heel pad
   wear).

In all cases the symptom is the same: at trained MGTO, the
center-of-pressure (CoP) the robot needs to achieve static balance
falls outside its support polygon to the rear, so the body torques
forward and the robot tips forward.

This is a **systematic offset** between simulator and hardware that
the policy was not trained against. Two ways to handle it:

- **Hardware-side fix**: physically move mass (move battery back).
  Not always possible.
- **Software-side fix**: shift the **reference pose** at deploy time
  so the *real* robot at the *new* reference has CoM-over-feet, while
  the *observation* still appears to the policy as if it were at the
  *trained* MGTO. This is the same idea as `dynamic_zero.yaml` already
  in the SDK — additive offset subtracted from observed `q` and added
  to commanded `q_target`. We call this `ref_offset`.

We need a **measurement** to pick `ref_offset` correctly, not guess.

## 2. Goals

1. Provide a **measurement protocol** that tells the operator, in
   under 5 minutes, what `ref_offset` (a 10-vector) to add to the
   MGTO reference pose so the robot stands statically stable.
2. Output a YAML file `bin/ref_offset.yaml` that `qmini_app` loads at
   startup (same pattern as `dynamic_zero.yaml`).
3. The protocol must work **on a single robot with only the standard
   sensors** (motor encoders + base IMU). No force plate, no external
   tracker.
4. Result is verifiable: after applying `ref_offset`, the robot stands
   stably in stand mode with no policy for ≥ 10 s, IMU pitch and roll
   within ±2° of zero.

### 2.1 Non-goal

We are not trying to identify per-link mass / inertia separately.
Only the net effect on the stable standing pose.

## 3. Theory (keep this in mind when implementing)

For static balance on flat ground:

```
CoM_world(q) projected onto floor ∈ support_polygon
```

For a biped in symmetric MGTO with both feet flat and a few cm apart,
the support polygon is a rectangle roughly the size of the foot
footprint. Static balance requires CoM lies inside that rectangle.

**Hard constraints during calibration** (operator requirement):

1. Upper body (`base_link`) must stay **level**: roll = 0, pitch = 0.
   The IMU on the body must read ≈ 0 throughout. This is what the
   trained policy expects, and it makes the observation seen by the
   policy identical to MGTO regardless of any offset we apply.
2. Each foot must stay **flat on the ground**: foot roll = foot pitch
   = 0 in body frame.
3. Symmetric: left and right legs always mirror.

**What we vary**: foot position in the body frame, parameterized by
two scalars:

- `Δx_foot` — both feet move forward (+) or backward (−) the same
  amount in the body sagittal plane.
- `Δy_foot` — both feet move laterally outward (+) or inward (−) the
  same amount. (Stretches or narrows the stance.)

The standing height `z_foot` stays at its MGTO value (we don't lower
the body). Foot z is fixed.

**Why this parameterization**: moving the feet relative to the
(level, fixed-height) body is equivalent to moving the body's CoM
relative to the feet, but it leaves the IMU reading at 0 and the
visible body orientation unchanged. This is what the user wants.

**How `Δx_foot` corrects forward-tip**: if the real CoM is too far
forward, the robot tips forward. Moving the feet forward by `Δx_foot`
puts the feet back under the CoM → balanced. The body itself does not
tilt.

### 3.1 Inverse kinematics

Per leg (5 DoF: hip_yaw, hip_roll, hip_pitch, knee, ankle), given
target foot position `(x_f, y_f, z_f)` in body frame and the
foot-flat constraint, solve for the 5 joint angles.

The Qmini leg chain (from `assets/q1/urdf/q1.urdf`):

```
base_link
  └── hip_yaw   (axis Z)
        └── hip_roll  (axis X)
              └── hip_pitch (axis Y)
                    └── knee     (axis Y)
                          └── ankle    (axis Y)   ← only pitch DoF
```

The ankle has **no roll axis**, so to keep the foot flat on the floor
with the body level, the hip_roll must equal 0. That means **lateral
foot translation cannot be done by `hip_roll`** alone without tilting
the foot. Use `hip_yaw` instead:

- Small `Δy_foot` via small `hip_yaw`: foot moves laterally by
  `hip_yaw × leg_length` (≈ 30 cm). For 1 cm lateral, hip_yaw ≈
  0.033 rad. Side effect: the foot rotates about Z by `hip_yaw`,
  toes pointing slightly outward. Acceptable for small offsets.

The sagittal IK (for `Δx_foot`):

```
# Inputs: x_f, z_f (target foot in body frame, sagittal plane)
# Link lengths read from URDF:
#   L1 = |hip_pitch_origin → knee_origin|
#   L2 = |knee_origin → ankle_origin|
# Constraint: foot pitch in body frame = 0 → θ_hp + θ_k + θ_a = 0

r2 = x_f² + z_f²
cos_k = (r2 − L1² − L2²) / (2·L1·L2)
θ_k = −acos(clip(cos_k, −1, 1))         # knee bent (negative)
θ_hp = atan2(x_f, −z_f) − atan2(L2·sin(θ_k), L1 + L2·cos(θ_k))
θ_a = −(θ_hp + θ_k)
```

The lateral IK (for `Δy_foot`):

```
hip_yaw = atan2(Δy_foot, leg_length_nominal)
# hip_roll = 0 (must be zero to keep foot flat on the floor)
```

The full joint vector is built per leg with the above, and the
**deltas** `Δq = q_new − q_MGTO` are saved as `ref_offset`. The
search is therefore effectively 2D over `(Δx_foot, Δy_foot)`, but
the saved output is the full 10-vector of joint deltas.

### 3.2 Practical search dimensions

- **Δx_foot (forward/back)**: this is the dominant fix for the
  "robot tips forward at MGTO" symptom. Expected range ±3 cm.
- **Δy_foot (lateral)**: usually 0 by symmetry. Only used if the
  robot tips sideways. Expected range ±1 cm.

## 4. Protocol

### 4.1 Equipment

- Flat hard floor.
- The Qmini robot, fully assembled, battery installed in its real
  position.
- One operator who can catch the robot if it falls.

### 4.2 Steps

#### Stage 0: Verify the symptom (1 min)

1. Power on robot. Boot SDK with `--no-onnx --keyboard`.
2. Enter stand mode (`2`) at full kp/kd from `config.yaml`.
3. Robot holds MGTO pose. Operator gently sets robot on floor.
4. Operator releases. Observe tipping direction:
   - `forward` (head-down) → CoM is too far forward → need
     `Δx_foot > 0` (feet move forward)
   - `backward` (head-up) → CoM is too far back → need
     `Δx_foot < 0`
   - `left` → CoM is left of center → need `Δy_foot > 0` if our
     convention is "feet step right to catch CoM"; the tool's
     convention is documented in the on-screen banner.
5. Record observation in `bin/ref_offset_notes.txt`.

If robot stands stably at MGTO, skip to §4.4 — no offset needed.

#### Stage 1: Sagittal foot-position search (~2 min)

The new `ref_calibration_tool` accepts arrow-key input from the
operator. While the robot is on the floor in stand mode, the tool
displays a live banner:

```
ref-cal | mode=Δx_foot     Δx_foot = +0.018 m   Δy_foot = +0.000 m
         IMU rpy = (+0.001, -0.015, 0.000) rad
         ↑/↓ move feet forward/back by 0.002 m
         ←/→ move feet outward/inward by 0.002 m
         Space record  Enter accept  Esc cancel
```

- `↑` / `↓` increment `Δx_foot` by ±0.002 m per press. Range capped
  at ±0.05 m. The tool runs the sagittal IK in §3.1, computes new
  `(hip_pitch, knee, ankle)` for both legs, smoothly slews to the
  new pose over 200 ms.
- `←` / `→` increment `Δy_foot` by ±0.002 m per press. Range capped
  at ±0.02 m. The tool computes new `hip_yaw` for both legs
  symmetrically.
- `r` resets all offsets to zero (back to MGTO).
- `Space` records the current `(Δx_foot, Δy_foot, q_new, IMU rpy)`
  to a session CSV for traceability.
- `Enter` accepts the current pose as final and writes
  `bin/ref_offset.yaml`.
- `Esc` aborts without writing.

The IK invariants `body_pitch = 0, body_roll = 0, foot_pitch = 0,
foot_roll = 0` are enforced by construction; the operator does not
need to think about per-joint angles.

Operator workflow:

1. Observe tip direction (Stage 0).
2. Press `↑` 5–10 times to move feet forward (or `↓` for backward).
3. Wait 1 s after each press for the slew to complete.
4. When the robot stops tipping, fine-tune with single presses.
5. Press `Enter`.

#### Stage 2: Lateral foot-position search (~1 min)

If Stage 1 alone is enough (robot is stable, IMU roll ≈ 0), skip.

Otherwise use `←` / `→` to widen / narrow stance until the lateral
tip is also corrected. Press `Enter`.

#### Stage 3: Verify (~1 min)

The tool re-applies the final pose and holds for 10 s. It records:

- `imu_rpy_mean`: average roll/pitch during the 10 s
- `imu_rpy_std`: stability
- `final_q`: the 10-vector of new joint targets
- `final_delta_q`: the 10-vector of joint offsets to write
- `final_dx_foot`, `final_dy_foot`: the operator-space scalars

Acceptance: `|imu_rpy_mean[0]| < 0.035 rad` (~2°),
`|imu_rpy_mean[1]| < 0.035 rad`, `imu_rpy_std < 0.02 rad`.

If acceptance fails, return to Stage 1.

### 4.3 Output

File `bin/ref_offset.yaml` (overwrites previous):

```yaml
ref_offset:
  # All values are joint angle DELTAS to add to ref_joint_act at deploy.
  # Computed via inverse kinematics from (dx_foot, dy_foot) with body
  # level and feet flat constraints; the operator does not type these.
  hip_yaw_l:    0.0
  hip_roll_l:   0.0           # always 0 under the foot-flat constraint
  hip_pitch_l:  0.041         # feet moved forward, hip pitch swings back
  knee_l:      -0.082         # to keep foot at same height
  ankle_l:      0.041         # to keep foot pitch flat
  hip_yaw_r:    0.0
  hip_roll_r:   0.0
  hip_pitch_r:  0.041
  knee_r:      -0.082
  ankle_r:      0.041
meta:
  date: 2026-05-30T14:22:01
  method: foot_translation_ik
  dx_foot_m: 0.018            # operator-readable scalar
  dy_foot_m: 0.000
  imu_rpy_mean_at_balance: [0.001, -0.008, 0.0]
  imu_rpy_std: 0.011
  symptom_pre_calib: forward_tip
  operator_notes: ""
```

The IK relationship between `dx_foot` and the joint deltas above is
documented in §3.1 — the YAML stores both the user-meaningful
scalars (`dx_foot_m`, `dy_foot_m`) and the joint deltas that get
applied at deploy. **Only the joint deltas are used at runtime**; the
scalars are for human inspection / recomputation if the URDF link
lengths change.

Also append a record to `bin/ref_offset_history.csv` for traceability.

### 4.4 Loading at deploy time

In `qmini_app.cpp`:

- At construction, if `bin/ref_offset.yaml` exists, load it as a
  `std::array<float, 10> ref_offset_`.
- In `RLController` or wherever the reference pose is built, add
  `ref_offset_[i]` to `ref_joint_act[i]` **before** sending to motors.
- For observations sent to the policy: **subtract** `ref_offset_[i]`
  from observed `q[i]` before computing `q − ref_joint_act`. This
  keeps the observation identical to what the policy saw in training.

This is the same pattern as `dynamic_zero.yaml`. Reuse the I/O code.

> The two offsets are independent and **stack additively**:
> `dynamic_zero` corrects encoder zero error (electrical/mechanical),
> `ref_offset` corrects URDF CoM error (mass distribution). Apply both.

## 5. CLI

```
./bin/ref_calibration_tool [--smooth-ms 200] \
                           [--step-x-m 0.002] [--step-y-m 0.002] \
                           [--out bin/ref_offset.yaml] \
                           [--mjcf <path>] [--no-viewer]
```

Defaults shown. The tool is keyboard-driven (raw termios; reuse the
`joystick_keyboard.cpp` pattern for stdin handling, and `Viewer` for
optional sim dry-run).

Required dependency: link lengths read from the URDF at
`assets/q1/urdf/q1.urdf`. Specifically `L_thigh` (hip_pitch → knee)
and `L_shin` (knee → ankle) joint origins. Parse once at startup and
print them on the banner so the operator can sanity-check.

For sim dry-run: `--mjcf sim_assets/q1_walking.mjcf` brings up the
MuJoCo viewer with the robot **on the floor** (not hung) so the
operator can practice the workflow before the real robot.

## 6. Acceptance criteria

The implementation is done when **all** of these hold:

1. `./bin/ref_calibration_tool --mjcf sim_assets/q1_walking.mjcf`
   runs in sim; operator can interactively find a stable pose with
   arrow keys and write a yaml.
2. On hardware (`./bin/ref_calibration_tool` with no `--mjcf`), the
   same workflow runs and produces `bin/ref_offset.yaml`.
3. After running, `./bin/run_interface --no-onnx` in stand mode (`2`)
   with the robot on the floor leaves the robot standing for ≥ 10 s
   with `|IMU pitch| < 2°` and `|IMU roll| < 2°`.
4. `dynamic_zero.yaml` and `ref_offset.yaml` can both exist and both
   apply; their effects stack. Verified by a unit test in
   `tests/test_offsets.cpp`.
5. `USAGE.md` has a new section "§4.4 Reference-pose calibration"
   that walks an operator through the procedure end-to-end in under
   1 page.
6. The default `bin/ref_offset.yaml` (committed to the repo) is empty
   `{ref_offset: {...all zeros...}}` so behavior is unchanged for any
   user who has not run the calibration.

## 7. Edge cases / safety

- **Operator catches robot during search**: arrow-key presses keep
  applying even if the robot is in the air. That's fine — the
  reference still slews; the IMU just shows rpy ≠ 0 while held. Ignore
  IMU until the robot is back on the floor.
- **Robot pitches violently after a single key press**: the
  `--smooth-ms 200` ramp should prevent this. If it happens, the
  PD gain is too high → reduce kp by `--hold-kp-scale 0.6` and retry.
- **Asymmetric offset > 0.05 rad**: warn the operator — suggests
  mechanical fault, not CoM drift. Log warning, accept yaml anyway.
- **No `bin/ref_offset.yaml` file**: SDK behaves as today
  (no offset applied). Print one-line INFO at startup: `[ref] no
  ref_offset.yaml — using URDF nominal pose`.

## 8. Out of scope

- Per-link mass identification. We only measure the *net* offset.
- Dynamic balance (during walking). The trained policy handles that;
  this calibration only corrects the static reference.
- Auto-tuning during walking. Manual operator-driven only.
- Anything in `qmini_lab`.

## 9. Files to add / touch

```
include/user/leg_ik.h                  # NEW: sagittal IK + lateral IK
source/user/leg_ik.cpp                 # NEW: clip + atan2 closed-form
include/user/ref_calibration.h         # CLI + state machine
source/user/ref_calibration_main.cpp   # new binary
source/user/ref_offset_loader.{h,cpp}  # yaml I/O, shared with qmini_app
source/user/qmini_app.cpp              # load + apply ref_offset
include/user/qmini_app.h               # add ref_offset_ member
bin/ref_offset.yaml                    # default (all zeros)
tests/test_leg_ik.cpp                  # NEW: IK roundtrip via FK
tests/test_offsets.cpp                 # ref_offset + dynamic_zero stack
USAGE.md                               # §4.4
CMakeLists.txt                         # add ref_calibration_tool target
```

`tests/test_leg_ik.cpp` must check: for a grid of `(Δx, Δy) ∈
[−0.05, 0.05]² m`, running `(Δx, Δy) → joint deltas → forward
kinematics` recovers the requested `(Δx, Δy)` within 1e-4 m, and the
foot pitch/roll in body frame is < 1e-4 rad.

Estimate: ~600 LOC C++, ~80 LOC YAML/CMake, 1 doc page. 1 agent
session.
