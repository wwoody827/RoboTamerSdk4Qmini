# Using the SDK

How to build the SDK and drive its two binaries:

- `bin/run_interface` — control runtime. Connects to a backend
  (sim / mujoco / hardware), runs the FSM, optionally loads ONNX policy.
- `bin/pd_calibration_tool` — PD calibration sweep. No policy. Drives
  one joint at a time and dumps `.npz` traces. See also
  `PD_CALIBRATION_SPEC.md` for the protocol it implements.

Tested on Ubuntu 24.04 + CMake 3.28 + an `env_isaaclab` conda env with
`mujoco` and `numpy/scipy/pyyaml`. No root needed for the desktop builds
except `sudo apt install libglfw3-dev` once.

---

## 1. Pick a backend

| Goal | Backend | Preset |
|---|---|---|
| Unit / integration tests, fast dev loop | `sim` | `desktop-sim` |
| Watch policy under real physics with live viewer | `mujoco` | `desktop-mujoco` |
| Drive the real Qmini robot | `hardware` | `robot-release` (ARM64, on the Orin) |

Each preset has its own `build/<preset>/` dir. Switching is just
`cmake --preset <name>` + `cmake --build …`. Binaries always go to
`bin/`, so the most-recently-built preset wins when you type
`./bin/run_interface`.

---

## 2. One-time setup

### MuJoCo (desktop-mujoco only)

Point the build at any MuJoCo install (the conda env that trains the
policy already has one):

```bash
cd ~/code/RoboTamerSdk4Qmini
sim_assets/setup_mujoco.sh /home/woody/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/mujoco
```

Creates `lib/mujoco/include` and `lib/mujoco/libmujoco.so*` symlinks.
Rerun whenever you switch conda envs or upgrade mujoco.

### GLFW (desktop-mujoco only, for the viewer)

```bash
sudo apt install libglfw3-dev
```

Without it, the viewer stub compiles in place and the binary still
runs headlessly — useful for SSH / benchmarks.

### MJCFs (regenerate when the URDF changes)

```bash
python3 sim_assets/build_mjcf.py             # → q1_sim.mjcf (free-fall, default)
python3 sim_assets/build_mjcf.py --hang 1.0  # → q1_sim_hung.mjcf (torso pinned at z=1m)
```

---

## 3. Build

```bash
# Desktop, MuJoCo physics + viewer (what you'll normally use):
cmake --preset desktop-mujoco
cmake --build build/desktop-mujoco -j

# Or sim only (no MuJoCo dep):
cmake --preset desktop-sim
cmake --build build/desktop-sim -j

# Robot (only configures on the ARM64 robot, needs unitree_sdk2):
cmake --preset robot-release
cmake --build build/robot-release -j

# Tests for the preset you built:
(cd build/desktop-mujoco && ctest)
```

Sanity-check what the binary actually links to:

```bash
ldd bin/run_interface | grep -E "mujoco|glfw|onnx"
```

---

## 4. `run_interface`

### Quick start (desktop, mujoco + viewer + hung MJCF)

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/run_interface --no-onnx --no-log \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2
```

The viewer window opens, robot stands at the ref pose, keyboard
controls take over.

### CLI flags

| Flag | Purpose |
|---|---|
| `--no-onnx` | Use identity policy (zeros). Default for desktop. |
| `--no-log` | Skip `general.txt` / `rl.txt` and the UDP telemetry broadcast. |
| `--policy <path>` | Load a specific ONNX file (needs `WITH_ONNX=ON`). |
| `--mjcf <path>` | Mujoco-backend MJCF (default `sim_assets/q1_sim.mjcf`). Pass `q1_sim_hung.mjcf` for observation. |
| `--no-viewer` | Headless — skip GLFW window. |
| `--initial-mode <c>` | FSM mode at startup. Default `1` (fold/zero-torque). Pass `2` to boot directly into stand. |
| `--stand-duration <s>` | Stand-mode ramp time (default 2 s). Larger = gentler on hardware. |
| `--stand-kp-scale <N>` / `--stand-kd-scale <N>` | Multiplier on mode-`2` gains. Default `1.0` (= use `config.yaml::kp/kd`). |
| `--sin-joint <0..9>` | In mode `5`, which joint to wiggle. Default = `config.yaml::sin_joint_idx`. |
| `--sin-amp <rad>` `--sin-freq <Hz>` | Sin amplitude/frequency (default 0.5 rad / 1 Hz). |
| `--zero-on-start` | At boot, capture current pose as the joint-space zero. Same as pressing `z` once in mode 1. |
| `--keyboard` | Switch mode FSM to line-mode stdin (digit + Enter). For hardware builds without a paired joystick. **Don't pass on sim/mujoco** — the keyboard joystick already owns stdin. |
| `--iface <name>` | Network iface (hardware backend only). |

### Modes (FSM)

| Mode | What it does | kp/kd source |
|---|---|---|
| `1` | Fold / idle | `kp_soft = kd_soft = 0` (zero torque) |
| `2` | Stand | `kp_ * stand_kp_scale`, `kd_ * stand_kd_scale` |
| `3` | RL walk (runs policy) | `kp_`, `kd_` directly |
| `5` | Sin test (single joint) | `kp_`, `kd_` directly |
| `0` | Hold-zero (PD to `q=[0]*10`) | `kp_`, `kd_` directly |
| `q` | E-stop (zero gains, stop_flag set) | — |

FSM transitions: `1 ↔ 2`, `2 ↔ 3`, `2 → 5` (sin), `3 → 4/5/6/7/8/9` (RL
sub-tasks). `0` is reachable from `1` or `2` via the `h` key.

### Keyboard (stdin joystick — desktop default)

Each keystroke is a button press, raw mode (no Enter, no echo). Every
consumed key echoes `[key 'X' → tag]` to stdout, and mode transitions
print in green: `Current mode: standing…`.

| Key | Action |
|---|---|
| `1` | mode 1 (fold) |
| `2` | mode 2 (stand) |
| `3` | mode 3 (RL walk) |
| `5` | mode 5 (sin test) |
| `b` | quit |
| `w` / `s` | `cmd_vx` ± 0.1 m/s |
| `a` / `d` | `cmd_vy` ± 0.1 m/s |
| `q` / `e` | `cmd_yaw` ± 0.1 rad/s |
| `r` / space | reset all command axes to 0 |
| `[` / `]` | (mode 5 only) cycle to prev / next sin joint, wraps 9 → 0 |
| `z` | (mode 1 only) capture current pose as the new zero. Writes `bin/dynamic_zero.yaml`. |
| `h` | toggle hold-zero (mode `0`). Use right after `z` to verify the capture. |

### Re-zeroing the encoders at runtime

`bin/config.yaml::startq` is the per-joint zero offset measured once at
factory bring-up. On the hardware backend it's baked into the gear-ratio
conversion. After a motor swap / re-cable / wrong-encoder, re-zero
**without editing config.yaml**:

1. Power on (robot in any pose).
2. Press `1` (fold, zero torque).
3. Manually position the robot at URDF natural pose (legs straight,
   body upright — your "true zero" reference).
4. Press `z`. Terminal prints `[zero] captured…` and writes
   `bin/dynamic_zero.yaml`.
5. Optionally press `h` to enter hold-zero. PD targets `q=0`; robot
   should physically stay put. If it doesn't move, the capture was
   correct.
6. Next boot auto-loads from `dynamic_zero.yaml`.

For scripted / unattended bring-up, replace steps 2–4 with the CLI
flag `--zero-on-start`.

The dynamic offset is added on top of `config.yaml::startq` in the
controller layer (above HAL). Works identically across sim / mujoco /
hardware. Delete `bin/dynamic_zero.yaml` to revert.

### Sim observation: the hung MJCF

The default MJCF (`q1_sim.mjcf`) lets the robot fall freely under
gravity. Fine for trained-policy deployment testing; useless for
watching what the legs do under zero or identity-policy actions
(the robot just collapses). Pass the hung variant:

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/run_interface --no-onnx --no-log \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2
```

The hung MJCF (built by `build_mjcf.py --hang Z`):
- Drops the freejoint → torso is rigidly attached to the worldbody
  at the spawn pose. No oscillation, no equality solver.
- Moves the floor down to `z = −hang_z` so the body appears
  `hang_z` metres above the ground.
- Disables self-collisions between robot links (the URDF visual
  meshes overlap at joints; without disabling, contact impulses
  eject joints at 40+ rad/s — see commit history if curious).
- Bakes a `<keyframe name="home">` at the stand reference pose;
  `world.cpp` loads it so joints start where the policy expects.

`--initial-mode 2` is recommended in this mode: mode `1` applies
zero torque, so legs gradually drift under gravity. Booting straight
in mode `2` keeps PD active from t=0.

Remove `--mjcf` (or rebake without `--hang`) when testing a real
policy — the policy was trained against the free-fall MJCF.

### Running with a trained policy

```bash
../../bin/run_interface --policy /path/to/policy.onnx --no-log
```

Requires `WITH_ONNX=ON` at configure time. The default
`desktop-mujoco` preset has `WITH_ONNX=OFF`; use `desktop-sim-onnx`
or reconfigure with `-DWITH_ONNX=ON`. `bin/policy.onnx` is loaded by
default if no `--policy` is given.

---

## 5. `pd_calibration_tool`

Runs the protocol from `PD_CALIBRATION_SPEC.md` — drives one joint at
a time through step / sine / chirp sequences and dumps NPZ traces.
Refuses to start without the harness-acknowledgement flag.

### Run sequence (hardware)

A hardware run steps through these phases, each interruptible with
Ctrl-C (soft stop) — keep the physical e-stop in reach as the hard stop:

1. **`Proceed?`** confirmation before any motion.
2. **Ramp to stand (MGTO)** — smooth ramp from the measured pose to the
   stand pose (no snap), then it holds MGTO.
3. **MGTO confirmation** — while holding the stand pose, verify it looks
   correct (startq right, no joint off) before perturbations start.
4. **Trials** — the step/sine/chirp plan, with the velocity watchdog.
5. **Fold** — ramps `kp`/`kd` down to limp so the robot relaxes
   gracefully instead of being left stiff. Skip with `--no-fold`.

`startq` is **not** calibrated here — do that first with
`joint_geom_cal_tool` (see
[`1_calibrate_joints.md`](1_calibrate_joints.md)); this tool only reads
it. `-y`/`--yes` bypasses the prompts (for sim / scripted runs). Confirm
the motor bus is live first with `./motor_status`.

### Operator-friendly workflow (you're holding the robot)

Every joint is driven at **its per-joint kp/kd from `config.yaml`** (the
deploy gains) — there is no kp/kd sweep. To keep each hold short, do one
frequency at a time.

```bash
cd ~/code/RoboTamerSdk4Qmini/bin     # config.yaml is here

# Confirm power/bus first (zero-torque, can't move the robot):
./motor_status --rounds 5

# Test A — one ~9 s step trial at the joint's deploy gain:
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --tests A

# Test B — ONE frequency per run (each ~5/f + 1 s). Repeat per freq.
# Use --safe-dq-max 8 for freqs >= 4 Hz (fast joints trip the default 4):
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --tests B --sine-freqs 1.0
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --tests B --sine-freqs 8.0 --safe-dq-max 8
```

All 10 joints, one frequency (each joint ~3.5 s at 2 Hz, ~1.5 min total —
one continuous hold):

```bash
./pd_calibration_tool --i-have-checked-the-harness --tests B --sine-freqs 2.0
```

Each run writes its own timestamped dir; the npz label shows the joint's
actual gain and frequency (e.g. `joint_03_knee_l/B_kp45_kd0.50_sine_2.00Hz.npz`),
so runs accumulate without overwriting. The npz also records the real
per-tick kp/kd arrays.

### Dry-run in sim with the viewer (before touching the robot)

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/pd_calibration_tool --i-have-checked-the-harness --quick \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --viewer \
    --output-root /tmp/cal_dryrun --label dryrun
```

`--quick` is the smoke test (single visible knee swing, ~4 s).
For a fuller rehearsal:

```bash
# Joint 3 (knee_l), Test A only — one step trial at the joint's deploy gain, ~9 s
../../bin/pd_calibration_tool --i-have-checked-the-harness \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --viewer \
    --tests A --joints 3 \
    --output-root /tmp/cal_dryrun --label j3_step
```

`--mjcf` and `--viewer` are sim-only — they no-op on hardware builds.

### Real-robot workflow

One joint at a time (recommended for first-ever calibration):

```bash
cd ~/code/RoboTamerSdk4Qmini/bin   # config.yaml is here

# Joint 0 — full A+B+C protocol, ~3 min
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 0 --label j0_hip_yaw_l \
    --output-root data/pd_calibration --operator <your_name>

# inspect, re-run if needed, then joint 1
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 1 --label j1_hip_roll_l \
    --output-root data/pd_calibration --operator <your_name>
# … repeat for joints 2..9
```

All 10 joints, full protocol in one session (~25 min of trials). Note
this is **not** unattended — you confirm the `Proceed?`/MGTO prompts,
then it runs the full A+B+C sweep:

```bash
./pd_calibration_tool --i-have-checked-the-harness \
    --label initial_full --output-root data/pd_calibration --operator you
```

For the full sine sweep (Test B up to 8 Hz) add `--safe-dq-max 8` so the
fast high-freq trials don't trip the velocity watchdog.

### CLI flags

| Flag | Purpose |
|---|---|
| `--i-have-checked-the-harness` | Required. Refuses to start otherwise. |
| `-y`, `--yes` | Skip the `Proceed?`/MGTO prompts (sim / scripted runs). |
| `--joints N,N,...` | Subset of joints (default all 10). |
| `--tests A,B,C` | Subset of tests (default A+B+C). |
| `--sine-freqs <hz,...>` | Test B frequencies (default `0.25,0.5,1,2,4,8`). One value → single freq per run. |
| `--quick` | Single 4 s sine on knee_l. Smoke test. |
| `--ramp-in-s <s>` | Smooth ramp from measured pose to MGTO before trials (default 3; 0 = none). |
| `--safe-dq-max <rad/s>` | Velocity-watchdog trip; aborts a trial if `|dq|` exceeds it for 2 ticks (default 4). |
| `--no-fold` | Don't fold (release gains to limp) at the end. |
| `--fold-secs <s>` | Gain-release ramp at the end (default 2; 0 = instant). |
| `--mjcf <path>` | MuJoCo backend MJCF override (sim dry-runs). |
| `--viewer` | Open GLFW viewer (sim dry-runs). |
| `--no-imu` | Skip IMU backend; set `imu_*` fields to NaN. |
| `--tick-hz <Hz>` | Override control rate. Default = `1/control_dt` from `config.yaml` (≈66.67 Hz). Prints a discretization-mismatch warning when overridden; see `PD_CALIBRATION_SPEC.md` §5. |
| `--output-root <path>` | Output root (default `data/pd_calibration`). |
| `--label <s>` | Run label suffix (default `initial`). |
| `--operator <s>` | Operator name for `run_meta.json`. |
| `--notes <s>` | Free-text notes for `run_meta.json`. |

### Per-joint trial counts

No kp/kd sweep — Test A is one step trial per joint.

| Protocol | Trials / joint | Time / joint |
|---|---|---|
| `A + B + C` (default) | 1 + 6 + 1 = 8 | ~73 s |
| `--tests A` | 1 | ~9 s |
| `--tests B --sine-freqs 2.0` | 1 | ~3.5 s |

### Output layout

```
<output-root>/<timestamp>_<label>/
├── manifest.json          # trial list, achieved rates, results
├── run_meta.json          # operator, notes, harness flag
├── log.txt
├── joint_00_hip_yaw_l/        # gains = config.yaml per joint (here 55 / 0.30)
│   ├── A_kp55_kd0.30_step.npz
│   ├── B_kp55_kd0.30_sine_0.25Hz.npz
│   ├── …
│   └── C_kp55_kd0.30_chirp.npz
└── joint_09_ankle_pitch_r/…
```

Each `.npz` has 16 arrays + 3 scalars; the base schema is in
`PD_CALIBRATION_SPEC.md §7`, plus the raw per-motor returns `tau_motor`
(unscaled motor-side torque), `temp` (°C), and `merror` (error code) —
hardware backend only, logged as-is for offline analysis.

### Fitting

Offline scripts under `tools/calibration_fit/`, picked by which test you
ran and how the robot was held:

```bash
# 1. Quick-look FIRST — did the joint track, and was the base actually
#    still? Writes per-joint PNGs + a console table; warns if base ω
#    peaked > 0.3 rad/s (chassis was swinging → fits will be biased).
python3 tools/calibration_fit/analyze_run.py <run_dir>/

# 2a. STEP data (Test A): second-order fit → calibration.yaml.
python3 tools/calibration_fit/fit_pd.py <run_dir>/ --mjcf sim_assets/q1_sim.mjcf
#     (fit_pd_imu_comp.py is the Test-A variant with IMU base-motion comp.)

# 2b. SINE data (Test B): sine-trace fit with IMU chassis-recoil
#     compensation — use this when the robot hangs/stands freely (the
#     base swings and contaminates q̈). Reports encoder-only vs
#     chassis-compensated per joint; writes plots + calibration.yaml.
python3 tools/calibration_fit/fit_pd_sine.py <run_dir>/

# 2c. FREE-RELEASE data (Test D): passive viscous damping b + Coulomb
#     friction f → passive_dynamics.yaml. Needs a gravity swing (skips
#     no-swing joints). See FREE_RELEASE_CALIBRATION_SPEC.md.
python3 tools/calibration_fit/fit_passive.py <run_dir>/

# 2d. CONST-VELOCITY data (Test E): friction f + viscous b read from
#     tau_est → friction.yaml. No gravity/inertia needed → the method for
#     high-ratio hip_roll and no-gravity hip_yaw/ankle. Needs ≥2 speeds.
python3 tools/calibration_fit/fit_friction.py <run_dir>/
```

Motors are identical, so the per-motor friction only needs **one normal
joint** (e.g. hip_pitch) + **hip_roll** (extra 3× gear → ratio 18.99 vs
6.33, friction reflects differently). Inertia is per-joint and comes from
the URDF, not measured. Test E is the robust friction method:
`./pd_calibration_tool --i-have-checked-the-harness --tests E --joints 1,6 --safe-dq-max 8`.

All three take the run dir as the first arg and accept `--out` for the
yaml path (default `<run_dir>/calibration.yaml`).

`fit_pd.py --mjcf` enables URDF gravity-gradient subtraction (spec §8.1
Y1 path); without it the output has `kp_motor = kp_eff` and a comment
that gravity isn't separated. `fit_pd_sine.py` needs the IMU channels in
the npz (don't run the trials with `--no-imu` if you'll fit with it).

**Don't auto-edit `qmini_lab` constants.** Per spec §11, the
operator reviews `calibration.yaml` and hand-updates
`qmini_lab/.../q1/constants.py`. See `data/pd_calibration/README.md`.

---

## 6. Joint calibration (`startq`)

**Moved to its own runbook: [`1_calibrate_joints.md`](1_calibrate_joints.md).**

It covers the single-turn-encoder background, the
bootstrap → jog-refine → canonical-lock workflow (the current method,
which calibrates hip_yaw/hip_roll with `joint_jog_tool` rather than the
old `ref_calibration_tool` Phase 4.5 path), the 30-second daily re-cal,
what's written where, and troubleshooting (encoder window slips,
ModemManager FTDI grabs, the working-directory gotcha). Start there for
anything touching `startq` or the mechanical limits.

`ref_calibration_tool` (§7 below) is now only for **CoM correction**
(foot translation so the robot balances at MGTO) — a separate concern
from `startq`.

---

## 7. `ref_calibration_tool`

Finds the foot-position offset that puts the robot's CoM over its foot
polygon, so it stands stably at MGTO. Writes the result to
`bin/ref_pose_calibrated.yaml` as **absolute joint values**. This is a
calibration record only — `run_interface` does NOT auto-load it. The
operator inspects the file and manually copies the new `ref_joint_act`
into `bin/config.yaml` if they want to adopt the calibrated pose.

> **CoM-only tool.** This tunes the MGTO standing pose; it does **not**
> touch `startq`. Calibrate `startq` first with `joint_jog_tool` (see
> [`1_calibrate_joints.md`](1_calibrate_joints.md)), then come here to
> make the robot balance.

### Why you need this

The trained policy expects the robot to be balanced at MGTO. The URDF
CoM is wherever the mass distribution in `q1.urdf` says it is. If the
hardware doesn't match the URDF (head heavier, battery moved, cabling
added), MGTO won't balance, and the robot tips in stand mode before
the policy can take over.

This tool lets the operator translate both feet in the body frame to
find a stable pose, while keeping the body level and the feet flat —
all via an inverse kinematics solver. On Enter it writes a calibration
record (`bin/ref_pose_calibrated.yaml`) containing the absolute joint
values at the chosen pose.

**The SDK does not auto-load this file.** Inspect the result, decide
whether to adopt the new pose, and if so, manually copy the
`ref_joint_act` list into `bin/config.yaml`. The calibration record is
purely a measurement artifact.

### Dry-run in sim (recommended first)

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/ref_calibration_tool --mjcf sim_assets/q1_sim_hung.mjcf
```

The hung MJCF welds the chassis to the world so the robot can't fall
while you practice the controls. The on-floor variant
(`--mjcf sim_assets/q1_sim.mjcf`) reproduces the real-robot tip
behaviour — useful if you want to practice catching the tip in sim,
but the robot WILL fall if MGTO is statically unstable.

### Real-robot workflow

```bash
cd ~/code/RoboTamerSdk4Qmini/bin
./ref_calibration_tool
```

Workflow (≤5 minutes). The tool walks 4 phases, every motion-causing
phase gated by an explicit prompt (mirrors `pd_calibration_tool`):

1. **Pre-motion confirmation** (`y`). Until this `y`, motors are limp.
   The next phase issues PD with the gains from `config.yaml`.
2. **Ramp to MGTO** (`--ramp-in-s`, default 2 s). Smooth blend from
   measured pose to MGTO.
3. **MGTO confirmation** (`y`). Robot is now holding MGTO via PD.
   Operator verifies the standing pose looks right (visual + IMU rpy)
   before any further motion.
4. **Operator pose-tuning loop (CoM correction)**. Arrow keys adjust
   `(dx_foot, dy_foot)`; the IK keeps the body level and feet flat;
   `Enter` writes `ref_pose_calibrated.yaml` (absolute joint values).
   Press `Enter` immediately if MGTO is already balanced.

`startq` is **not** touched anywhere in this tool — calibrate it first
with `joint_jog_tool` ([`1_calibrate_joints.md`](1_calibrate_joints.md)).

On real hardware, place the robot on the ground before phase 1, and keep
the e-stop in reach throughout.

### Phase 4 keys (CoM correction)

1. Observe tip direction:
   - Falls forward (head down) → press `↑` (move feet forward to
     catch up to CoM)
   - Falls backward (head up) → press `↓`
   - Falls sideways → use `←` / `→` to widen/narrow stance
2. Each arrow press shifts the feet by 2 mm (default) and the joints
   slew over 200 ms. Keep adjusting until the IMU rpy mean settles
   near zero.
3. Press `Enter` to write `bin/ref_pose_calibrated.yaml`. Inspect the
   file; if you're happy with the result, **manually copy the
   `ref_joint_act` block into `bin/config.yaml`**. The SDK will pick
   up the new pose on the next boot. (Nothing auto-reads this file.)

### Keys (raw terminal — no Enter needed per key)

| Key | Action |
|---|---|
| `↑` / `↓` | Move BOTH feet forward / backward by `--step-mm` |
| `→` / `←` | Widen / narrow stance by `--step-mm` |
| `r` | Reset to MGTO (`dx=dy=0`) |
| `s` | Re-apply slew toward the current target |
| `Enter` | Accept current pose, write yaml |
| `q` / `Esc` | Abort without writing |

Each keypress echoes its own status line; the live `[live]` row at
the bottom keeps refreshing with current `(dx, dy)` and IMU rpy.

### Sign convention

`dx_foot > 0` moves both feet **forward** in body x. This shifts the
foot polygon forward, which is equivalent to shifting the body CoM
**backward** relative to the feet. So if the robot tips **forward**,
the foot polygon is BEHIND the CoM — pressing `↑` (`dx > 0`) moves
feet forward to catch up to the CoM. If the robot tips **backward**,
press `↓`.

### CLI flags

Safety / workflow:
```
-y, --yes               skip ALL confirmation prompts (sim / scripts only)
--ramp-in-s <s>         ramp from measured pose to MGTO (default 2)
```

Operator loop:
```
--step-mm <int>     ±step per arrow press (default 2)
--max-mm <int>      absolute cap on |dx|, |dy| (default 50)
--slew-ms <int>     slew time per key press (default 200 ms)
--out <path>        output yaml (default bin/ref_pose_calibrated.yaml)
--dynamic-zero <p>  encoder zero file to load (default bin/dynamic_zero.yaml)
```

Backend:
```
--mjcf <path>       mujoco backend: which MJCF to load
                    (recommend sim_assets/q1_sim_hung.mjcf for dry-run)
--no-viewer         mujoco backend: skip GLFW viewer
--iface <name>      hardware backend: net iface for DDS
--tick-hz <hz>      control rate (default = 1/cfg.control_dt)
```

### What the tool reads

1. `config.yaml::startq` — factory zero, applied inside the HAL
2. `dynamic_zero.yaml::dynamic_zero` — runtime re-zero, applied
   above the HAL (subtracted from observed q, added to commanded q)
3. `config.yaml::ref_joint_act` — MGTO joint values used as IK seed
   and the baseline the operator's `(dx, dy)` is measured from
4. `config.yaml::kp` / `kd` — PD gains used for ramp and hold

### Output

`bin/ref_pose_calibrated.yaml`:

```yaml
# Calibrated reference pose, written by ref_calibration_tool.
#
# This file is a CALIBRATION RECORD, not a runtime config. Nothing in
# the SDK auto-loads it. To use the result, manually copy the
# `ref_joint_act` list below into bin/config.yaml.
ref_joint_act: [ 0.4000, -0.1000, -1.4727,  1.0177, -1.2248,
                -0.4000,  0.1000,  1.4727, -1.0177,  1.2248 ]
meta:
  date: 2026-05-30T14:22:01
  method: foot_translation_ik
  dx_foot_m: 0.020
  dy_foot_m: 0.000
  ref_joint_act_baseline: [ 0.4000, -0.1000, -1.5000, ... ]
  delta_from_baseline:    [ 0.0000,  0.0000,  0.0273, ... ]
  imu_rpy_mean_at_balance: [0.001, -0.008, 0.0]
  imu_rpy_std: 0.011
```

`ref_joint_act` is the absolute joint pose at the operator's final
key press. `ref_joint_act_baseline` is what `config.yaml` said when
the tool started, and `delta_from_baseline` is the IK adjustment. The
operator inspects the file and decides whether to copy `ref_joint_act`
into `bin/config.yaml`.

### Constraints enforced by the IK

| Invariant | How |
|---|---|
| Body roll = 0 | Operator only adjusts feet, body stays put |
| Body pitch = 0 | Same |
| Foot pitch = 0 (feet flat) | Ankle absorbs hip_pitch + knee deltas |
| Foot roll = 0 | hip_roll forced to MGTO; ankle has no roll DoF |
| L/R symmetric | Same dx applied to both legs |

The IK math + verification against the MJCF kinematic chain is in
`COM_CALIBRATION_SPEC.md` and `tools/verify_leg_ik_mujoco.py`.

---

## 8. Tick rate notes

`pd_calibration_tool` defaults to `1/control_dt` from `config.yaml`
(currently `0.015 s` → 66.67 Hz) — the same rate the deployed policy
runs at. Calibrating at a different rate produces a discrete-time PD
model that doesn't match what the policy actually sees.

Override with `--tick-hz <Hz>` only when characterizing rate-dependent
behaviour (e.g. 200 Hz vs 66.67 Hz comparison). The tool prints a
banner warning whenever override is active.

The rate watchdog aborts a trial if the achieved rate drops below 90%
of the requested rate for >500 ms continuous. If you see
`aborted_slow_rate` in a trial's manifest entry, the backend can't
sustain the requested rate — lower `--tick-hz` and try again.

---

## 9. Live viewer (MuJoCo backend)

Both `run_interface` and `pd_calibration_tool` open the same GLFW
window when built against `desktop-mujoco` and run with `--viewer`
(default-on for `run_interface`, opt-in for the calibration tool).

Renderer runs on a dedicated thread, snapshots `qpos`/`qvel` from
`World` under the world mutex, then renders at ~60 Hz without
holding the lock — physics ticks in the control thread are
unaffected.

| Mouse | Action |
|---|---|
| Left-drag | Rotate camera |
| Right-drag | Pan camera |
| Scroll | Zoom |
| Shift + drag | Horizontal pan |
| Close window | Sets stop_flag, app shuts down cleanly |

Disable with `--no-viewer` (or just don't pass `--viewer` for the
calibration tool). If `libglfw3-dev` was missing at configure time
the viewer stub compiles in place and `--viewer` prints a notice
and continues headless.

---

## 10. Diagnostics: `bin/probe_modes`

Drives `QminiApp` through one FSM mode for N seconds, prints joint
positions every 0.5 s, and dumps the full trace to
`/tmp/probe_mode<char>.npz`. Doesn't run as part of `ctest`.

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/probe_modes <mode_char> <duration_s> [stand_kp_scale=30]

# e.g.
../../bin/probe_modes 5 3 1          # sin test, 3 s, default gains
../../bin/probe_modes 2 7 1          # stand, 7 s
```

Use when "I don't see motion" or "is the FSM transitioning?" — the
NPZ has every tick of `q`, `q_target`, `ref_joint_act` so you can
verify what the SDK actually commanded vs measured, independent of
viewer perception.

---

## 11. Cheat sheet

```bash
# === Build ===
cmake --preset desktop-mujoco && cmake --build build/desktop-mujoco -j

# === Calibration (1_calibrate_joints.md) — C++ tools from bin/, python from root ===

# Daily, ~30 s (reproduces the locked calibration via canonical limits):
(cd bin && ./joint_range_tool --out /tmp/ranges.yaml)
python3 tools/apply_limit_calibration.py /tmp/ranges.yaml --apply

# First-time / full startq:
(cd bin && ./joint_range_tool --out /tmp/bs1.yaml)          # bootstrap windows + sagittal
python3 tools/apply_limit_calibration.py /tmp/bs1.yaml --apply
(cd bin && ./joint_geom_cal_tool)                           # geometric per-joint pin (10 captures)
(cd bin && ./joint_range_tool --out /tmp/canonical.yaml)    # re-record canonical in the new frame
python3 tools/apply_limit_calibration.py /tmp/canonical.yaml --record-canonical

# CoM balance (only if MGTO still tips after startq is correct):
(cd bin && ./ref_calibration_tool)
# → paste ref_joint_act from bin/ref_pose_calibrated.yaml into bin/config.yaml

# === PD calibration (§5) ===

# Dry-run in sim with viewer:
cd tests/fixtures && ../../bin/pd_calibration_tool \
    --i-have-checked-the-harness --quick \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --viewer \
    --output-root /tmp/dryrun --label dr

# Real-robot calibration, one joint, full protocol:
cd bin && ./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --label j3_knee \
    --output-root data/pd_calibration --operator <name>

# === Deployment ===

# Policy + viewer, hung MJCF, stand from t=0:
cd tests/fixtures && ../../bin/run_interface --no-onnx --no-log \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2

# Real-robot deployment:
cd bin && ./run_interface --policy policy.onnx

# === Diagnostics ===
./bin/probe_modes 5 3 1   # FSM mode trace, 3 s
```
