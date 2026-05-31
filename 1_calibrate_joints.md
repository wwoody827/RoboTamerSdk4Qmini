# 1 — Calibrate the joints (`startq`)

Calibrates `bin/config.yaml::startq` so the SDK's reported joint angle
`q` matches URDF coordinates. This is the **first** thing to do on a
fresh robot, after any power cycle, and any time a motor loses power
mid-run. No `run_interface` is involved at any step.

Four tools, all robot-side:

| Tool | Lang | Run from | Role |
|---|---|---|---|
| `bin/joint_range_tool` | C++ | `bin/` | limp sweep of mechanical stops → ranges yaml |
| `bin/joint_geom_cal_tool` | C++ | `bin/` | limp per-joint geometric-landmark `startq` pin (square/level/view) |
| `bin/joint_jog_tool` | C++ | `bin/` | multi-pose PD jog refinement of hip_yaw / hip_roll (legacy) |
| `tools/apply_limit_calibration.py` | Python | repo root | ranges → `startq`; records canonical limits |
| `tools/calibration_fit/solve_startq.py` | Python | repo root | jog records → `startq` (per-joint σ) |

> **Working-directory gotcha.** The C++ tools load `config.yaml` from
> the **current directory** → run them from `bin/`. The Python scripts
> default to the relative path `bin/config.yaml` → run them from the
> **repo root**. The typical loop is `cd bin && ./tool …` then
> `cd .. && python3 tools/…`.

---

## TL;DR

**After a normal reboot** (encoder windows intact — the common case):

```bash
cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_range_tool --out /tmp/r.yaml
cd .. && python3 tools/apply_limit_calibration.py /tmp/r.yaml --apply
```

This reads `bin/canonical_joint_limits.yaml` and reproduces your locked
calibration (jog refinement baked in) from a limp sweep alone — no
jogging needed. ~30 s. Done.

**First-time bring-up, after a window slip, or if you don't trust the
calibration** — run the full 3-stage flow below.

---

## Concepts

### The encoder problem

The Unitree GO-M8010-6 motor has a 15-bit **single-turn** absolute
encoder. Multi-turn position does **not** persist across power cycles.
At every boot a motor reports its angle only within whatever single-turn
window the encoder happens to be in. The SDK maps raw → URDF angle by
subtracting a per-joint offset:

```
q[i] = raw[i] − startq[i]
```

`startq` is what we calibrate. The calibration is anchored by the
mechanical hard stops (motors physically cannot pass these).

### Window width — why hip_roll is fragile

A given joint angle maps to `motor_angle = q · ratio`, and the encoder
only knows `motor_angle mod 2π`. So the boot reading is unique only
within one joint-side **window** of width `2π / ratio`:

| Joints | Gear ratio | Window width | Tolerance to boot in the same window |
|---|---|---|---|
| hip_yaw, hip_pitch, knee, ankle | 6.33 | **0.99 rad (~57°)** | ±28° — generous |
| **hip_roll** | 18.99 | **0.33 rad (~19°)** | **±9.5° — tight** |

If a joint boots more than half a window from where it was calibrated —
or a motor stalls/trips and re-seats one revolution off — `startq` for
that joint is wrong by exactly one window (~0.99 rad, or ~0.33 for
hip_roll). hip_roll is the usual victim. See
[Troubleshooting](#troubleshooting).

### The calibration identity

For both tools the math reduces to: at any physical pose the robot
actually reaches,

```
startq_true[j] = startq_now[j] + (q_read[j] − q_target[j])
```

`joint_range_tool` uses the mechanical stops as the known `q_target`;
`joint_jog_tool` uses precomputed reference poses.

### Per-joint method — why two tools

Not every joint can be calibrated the same way:

- **hip_pitch / knee / ankle (6 joints):** their URDF range is entirely
  one side of 0, so **`q=0` is a real mechanical stop**. The limit
  sweep pins them to <1°. They do **not** need jogging.
- **hip_yaw / hip_roll (4 joints):** their range straddles 0, so `q=0`
  is mid-range with **no mechanical reference**. The limit sweep can
  only estimate them by *midpoint* of the two stops (imprecise), and
  hip_yaw in particular has no IMU handle (yaw doesn't tilt the body).
  These are what the **jog tool** fixes — hip_yaw by feet-forward,
  hip_roll by body-roll-level.

So the final `startq` is a **merge**: jog values for the 4 yaw/roll
joints, limit-bootstrap values for the 6 sagittal joints. The canonical
step then bakes the jog-refined zero into the limit reference so future
reboots reproduce it from a sweep alone.

---

## Stage 1 — Bootstrap from mechanical limits

Gets every joint into the correct encoder window and gives precise
`startq` for the sagittal chain.

1. With motors powered but limp, hand-pose the robot to a *rough*
   standing/zero stance (body upright, legs roughly straight, feet
   roughly forward). Within ~half a window is enough — generous for
   most joints, keep hip_roll within ~9°.

2. Sweep the stops (limp, zero-torque, safe):

   ```bash
   cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_range_tool --out /tmp/ranges.yaml
   ```

   Move each joint slowly to **both** hard stops; watch the table latch
   the extremes. **Enter** to save, `q`/`Esc` to abort.

   Sanity check the printed ranges: each joint's measured range should
   match its URDF width (~0.8/0.9 for yaw/roll, ~2.1/2.5/2.7 for the
   rest). A range that's ~1 window short/long means that joint booted in
   the wrong window — re-pose it and re-sweep.

3. Compute + write `startq` (from the **repo root**):

   ```bash
   cd ~/code/RoboTamerSdk4Qmini
   python3 tools/apply_limit_calibration.py /tmp/ranges.yaml          # dry run, inspect
   python3 tools/apply_limit_calibration.py /tmp/ranges.yaml --apply  # write (+ .bak)
   ```

   Corrections of a few degrees are normal. A `>0.2 rad` warning means a
   wrong-window stop or a bad URDF target — investigate before applying.

---

## Stage 1.5 — Geometric per-joint pin (optional, recommended)

Pins each joint's `startq` to a precise URDF angle using a **physical
landmark** (carpenter's square, straightedge, bubble level, top/front
view) rather than the mechanical stop alone. This decouples each joint's
calibration from every other and from the limit-sweep accuracy. See
[`GEOMETRIC_JOINT_CALIBRATION_SPEC.md`](GEOMETRIC_JOINT_CALIBRATION_SPEC.md)
for theory + landmark derivation.

Five landmarks × 2 legs = 10 captures. Motors stay limp throughout.

| Joint | Landmark | q_L target | Reference image |
|---|---|---|---|
| ankle | foot ⟂ shank (square) | −1.569 | [`ankle_perp`](docs/images/geom_cal/ankle_perp.png) |
| knee | thigh ‖ shank, 180° (straightedge) | −0.084 | [`knee_straight`](docs/images/geom_cal/knee_straight.png) |
| hip_yaw | leg in body sagittal plane (top + side view) | +0.400 | [`hip_yaw_forward`](docs/images/geom_cal/hip_yaw_forward.png) |
| hip_roll | no lateral tilt (front view) | 0.000 | [`hip_roll_no_tilt`](docs/images/geom_cal/hip_roll_no_tilt.png) |
| hip_pitch | thigh horizontal, torso vertical (bubble level + IMU) | −0.715 | [`hip_pitch_thigh_horizontal`](docs/images/geom_cal/hip_pitch_thigh_horizontal.png) |

Right leg = negate each value.

### Procedure

```bash
cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_geom_cal_tool
```

The tool ships zero kp/kd/tau every tick; the robot does not move on its
own. Workflow:

1. With symmetric mode on (default), pose **both legs** at one
   landmark simultaneously (e.g. carpenter's square on both ankles).
2. `0`–`9` (or `[` / `]`) select the joint, **SPACE** captures both legs.
3. Repeat for the other four landmarks. The tool's `✓` column tracks
   which joints have been captured this session.
4. `w` writes the new `startq` to `config.yaml` (+ `.bak`). `q` / `Esc`
   aborts and reverts.

### Operator notes

- **hip_pitch needs the torso vertical.** Use the IMU readout on the
  tool's status line (`rpy`, top of the table). Hold the body so
  `pitch < 0.02 rad` while you bubble-level the thigh.
- **ankle and knee are intra-leg** — no external reference needed. Do
  these first; any body-pitch error after that is isolated to the
  remaining joints.
- **hip_yaw and hip_roll** are body-plane checks (top / side / front
  views). Visually verify the leg is in the body's sagittal plane (no
  splay) and drops straight down (no lateral tilt). These don't depend
  on the sagittal chain.

### After Stage 1.5

`startq` is now pinned per joint with sub-degree accuracy. **Skip Stage 2
(jog)** and proceed to Stage 3 to re-record canonical limits in the new
frame, then return to the [TL;DR](#tldr) daily path.

> Stage 2 (PD jog) remains documented below for the legacy workflow
> and for hip_yaw / hip_roll refinement when no square/level is
> available.

---

## Stage 2 — Jog refinement (hip_yaw / hip_roll)

This is what limits can't do. The tool ramps the robot under PD to a
series of body-level, feet-forward/parallel reference poses at several
heights; at each you nudge the robot to *physically* match the ideal
pose, then record.

1. Generate the reference poses (LegIK, no hardware):

   ```bash
   cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_jog_tool --gen-poses cal_poses.yaml
   ```

   Writes 4 poses (`mgto`, `crouch_20mm`, `crouch_40mm`, `tall_20mm`).
   `cal_poses.yaml` is gitignored — regenerable any time.

2. Run the interactive session (**hand on the e-stop**):

   ```bash
   ./joint_jog_tool --poses cal_poses.yaml --out /tmp/recs.yaml
   ```

   **Controls**

   | Key | Action |
   |---|---|
   | `0`–`9` | select joint |
   | `k` / `j`  (or ↑ / ↓) | jog selected joint's command up / down |
   | `l` / `h`  (or → / ←) | trim selected joint's `startq` up / down |
   | `=` / `-` | bigger / smaller jog step |
   | `z` | zero the selected joint's jog |
   | `m` | toggle **symmetric** mode (default **ON** — mirrors L/R, opposite sign, so the stance stays symmetric); `--independent` to start off |
   | **SPACE** | record this pose (`q_read`, `q_target`, IMU) |
   | `n` / `p` | next / previous pose |
   | `w` | save records to `--out` and `startq` to `config.yaml` (+ .bak) |
   | `f` | fold gains down and exit; `q` aborts (reverts `startq`) |

   The bottom line shows `IMU r/p` (body tilt) and `foot-fwd L/R` (yaw),
   all of which should go to **0**.

   **At each of the 4 poses:**
   1. Trim hip_pitch/ankle (and hip_roll) until **IMU r≈0, p≈0** (body level).
   2. Jog **hip_yaw** until **foot-fwd L,R ≈ 0** (feet straight forward). *This is the hip_yaw calibration.*
   3. Eyeball that the feet are flat and parallel.
   4. **SPACE** to record, then `n` for the next pose.

   Record all 4 (re-record a pose if you re-tune it), then `w` to save,
   `f` to exit.

3. Solve and inspect the per-pose agreement (from the **repo root**):

   ```bash
   cd ~/code/RoboTamerSdk4Qmini
   python3 tools/calibration_fit/solve_startq.py /tmp/recs.yaml
   ```

   Read the **spread (σ)** column:
   - **hip_yaw / hip_roll → σ < ~0.005:** trust these. This is the win.
   - **hip_pitch / knee / ankle → σ ~0.02–0.07 (HIGH):** *expected and
     ignore.* The sagittal chain is under-determined by jogging (3 DOF,
     only body-pitch + foot-flat are observable — height is not), so the
     poses disagree. Keep the Stage-1 limit values for these.

4. Apply the **merge** — jog values for yaw/roll, Stage-1 bootstrap
   values for the sagittal chain. `config.yaml.bak` from Stage 1 holds
   the clean bootstrap; combine by hand (or edit the `startq` line
   directly):

   ```
   #            HYL        HRL       HPL       KL        AL        HYR        HRR       HPR       KR        AR
   #            jog        jog       boot      boot      boot      jog        jog       boot      boot      boot
   startq: [<yaw_l>, <roll_l>, <bootstrap 6 sagittal interleaved>, <yaw_r>, <roll_r>, …]
   ```

   The yaw/roll values converge to <3.5 mrad, so a single jog session is
   normally enough — no need to iterate.

---

## Stage 3 — Lock the canonical limits

Bakes the jog-refined zero into the limit reference so future reboots
skip Stages 2 entirely.

1. Re-sweep with the **final merged `startq`** (the Stage-1 sweep was in
   the old frame). This doubles as the final verification:

   ```bash
   cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_range_tool --out /tmp/canonical.yaml
   ```

   Confirm: no window slipped, and **hip_yaw L/R are now mirror-symmetric**
   (e.g. `[0.00, +0.89]` vs `[−0.89, 0.00]`). They will *not* match the
   nominal URDF range — correct, because the jog redefined the yaw zero
   by feet-forward, which is the right reference.

2. Record them (from the **repo root**):

   ```bash
   cd ~/code/RoboTamerSdk4Qmini
   python3 tools/apply_limit_calibration.py /tmp/canonical.yaml --record-canonical
   ```

   Writes `bin/canonical_joint_limits.yaml`. `--record-canonical` only
   writes the limits file; it does **not** touch `startq`.

3. Verify self-consistency — re-running the solver against the same
   sweep should now show **Δstartq ≈ 0 on every joint**:

   ```bash
   python3 tools/apply_limit_calibration.py /tmp/canonical.yaml
   ```

From now on the [TL;DR](#tldr) daily path reproduces this exact `startq`.

---

## What gets written where

| File | Written by | Read by | Tracked in git? |
|---|---|---|---|
| `bin/config.yaml::startq` | `apply_limit_calibration.py --apply`, `solve_startq.py --apply`, jog tool `w` | every SDK boot | yes (robot-specific — commit only for a single-robot repo) |
| `bin/canonical_joint_limits.yaml` | `apply_limit_calibration.py --record-canonical` | `apply_limit_calibration.py` on later runs | yes (robot-specific) |
| `bin/cal_poses.yaml` | `joint_jog_tool --gen-poses` | `joint_jog_tool --poses` | **no** — gitignored, regenerable |
| `*.bak` | every `--apply` / `w` save | — | no — gitignored |

---

## Verify the calibration

Quick: at MGTO the body should sit level and the feet point forward.
Re-run the jog tool, let it ramp to `mgto`, and just watch (don't
record):

```bash
cd ~/code/RoboTamerSdk4Qmini/bin && ./joint_jog_tool --poses cal_poses.yaml --out /tmp/check.yaml
# IMU r,p ≈ 0 ; foot-fwd L,R ≈ 0 ; q_read tracks q_cmd. Then 'f' to fold.
```

---

## Troubleshooting

### A joint is off by ~1 revolution (window slip)

**Symptom:** after a power cycle or a motor briefly losing power, a pose
looks wrong even though `motor_status` reports all motors OK.

**Detect (no hardware motion):** read `motor_status`, compute
`q = q_raw − startq`, and check left/right pairs satisfy `q_r ≈ −q_l` in
a symmetric stance. The shifted motor is off by exactly one window
(~0.99 rad, or ~0.33 for hip_roll). hip_roll is the usual one (tiny
window, often on a flaky FTDI port).

**Fix:** re-run the full flow from Stage 1 (the limp sweep re-seats every
joint into the correct window robustly). Do **not** "check" by commanding
MGTO with a suspect `startq` — driving a window-shifted joint can slam a
stop and re-trip the shift.

### "motor id=N does not reply", set of failures shifts between boots

Not a power/wiring/permissions problem. **ModemManager** grabs the FTDI
`ttyUSB` ports on boot/hotplug and probes them for ~30 s. Mask it once:

```bash
sudo systemctl mask --now ModemManager
```

(The mask survives reboots. `woody` is in `dialout`, so no `sudo` for the
tools afterward.) See `scripts/hardware/README.md`.

### `YAML::BadFile: bad file: config.yaml`

You ran a C++ tool from the wrong directory. Run `joint_range_tool` /
`joint_jog_tool` from `bin/`; run the Python scripts from the repo root.

### Body not level at MGTO / can't tell which pitch joint is wrong

Body-IMU leveling only sees the **sum** of the pitch chain
(`θ_body = ankle + knee + hip_pitch`), so it can't tell you *which* of
hip_pitch / knee / ankle is mis-calibrated — and multi-pose jogging
can't either (same degenerate equation at every height).

Fix: run [Stage 1.5](#stage-15--geometric-per-joint-pin-optional-recommended)
with `./joint_geom_cal_tool`. The ankle ⟂ shank and knee straight
landmarks are **intra-leg** — they decompose the chain and pin those
two joints with sub-degree accuracy independent of the others. After
they're right, any residual body-pitch error is fully attributable to
hip_pitch (capture its thigh-horizontal landmark).

Full method + verified target angles + spec:
[`GEOMETRIC_JOINT_CALIBRATION_SPEC.md`](GEOMETRIC_JOINT_CALIBRATION_SPEC.md).

---

*Related: `7. ref_calibration_tool` in `USAGE.md` handles CoM correction
(foot translation so the robot balances at MGTO) — a separate concern
from `startq`. PD-gain identification is in `PD_CALIBRATION_SPEC.md`.*
