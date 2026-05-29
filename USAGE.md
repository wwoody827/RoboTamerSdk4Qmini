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

1. **startq zero calibration** (prompt, default yes). Motors go limp; a
   10 s countdown lets you hand-pose the robot to the zero pose, then a
   5 s window averages the measured positions. Confirm to **apply +
   save** the new `startq` to `config.yaml` (picked up by future runs
   and `run_interface`). Skip with `--no-zero-cal`.
2. **`Proceed?`** confirmation before any motion.
3. **Ramp to stand (MGTO)** — smooth ramp from the measured pose to the
   stand pose (no snap), then it holds MGTO.
4. **MGTO confirmation** — while holding the stand pose, verify it looks
   correct (startq right, no joint off) before perturbations start.
5. **Trials** — the step/sine/chirp plan, with the velocity watchdog.
6. **Fold** — ramps `kp`/`kd` down to limp so the robot relaxes
   gracefully instead of being left stiff. Skip with `--no-fold`.

`-y`/`--yes` bypasses the prompts and skips zero-cal (for sim / scripted
runs). Confirm the motor bus is live first with `./motor_status`.

### Operator-friendly workflow (you're holding the robot)

Every joint is driven at **its per-joint kp/kd from `config.yaml`** (the
deploy gains) — there is no kp/kd sweep. To keep each hold short, do one
frequency at a time. `startq` only needs calibrating once (it's saved),
so add `--no-zero-cal` after the first run.

```bash
cd ~/code/RoboTamerSdk4Qmini/bin     # config.yaml is here

# Confirm power/bus first (zero-torque, can't move the robot):
./motor_status --rounds 5

# Test A — one ~9 s step trial at the joint's deploy gain:
./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --tests A

# Test B — ONE frequency per run (each ~5/f + 1 s). Repeat per freq.
# Use --safe-dq-max 8 for freqs >= 4 Hz (fast joints trip the default 4):
./pd_calibration_tool --i-have-checked-the-harness --no-zero-cal \
    --joints 3 --tests B --sine-freqs 1.0
./pd_calibration_tool --i-have-checked-the-harness --no-zero-cal \
    --joints 3 --tests B --sine-freqs 8.0 --safe-dq-max 8
```

All 10 joints, one frequency, with zero-cal in front (each joint ~3.5 s
at 2 Hz, ~1.5 min total — one continuous hold):

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
this is **not** unattended — you hand-pose for zero-cal and confirm the
`Proceed?`/MGTO prompts, then it runs the full A+B+C sweep:

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
| `-y`, `--yes` | Skip the `Proceed?`/MGTO prompts and zero-cal (sim / scripted runs). |
| `--joints N,N,...` | Subset of joints (default all 10). |
| `--tests A,B,C` | Subset of tests (default A+B+C). |
| `--sine-freqs <hz,...>` | Test B frequencies (default `0.25,0.5,1,2,4,8`). One value → single freq per run. |
| `--quick` | Single 4 s sine on knee_l. Smoke test. |
| `--no-zero-cal` | Skip the startq zero-calibration step (default = prompt, run). |
| `--zero-cal-countdown <s>` | Seconds to hand-pose to the zero pose before measuring (default 10). |
| `--zero-cal-measure <s>` | Averaging window for zero calibration (default 5). |
| `--ramp-in-s <s>` | Smooth ramp from measured pose to MGTO before trials (default 3; 0 = none). |
| `--safe-dq-max <rad/s>` | Velocity-watchdog trip; aborts a trial if `|dq|` exceeds it for 2 ticks (default 4). |
| `--no-fold` | Don't fold (release gains to limp) at the end. |
| `--fold-secs <s>` | Gain-release ramp at the end (default 2; 0 = instant). |
| `--mjcf <path>` | MuJoCo backend MJCF override (sim dry-runs). |
| `--viewer` | Open GLFW viewer (sim dry-runs). |
| `--no-imu` | Skip IMU backend; set `imu_*` fields to NaN. |
| `--tick-hz <Hz>` | Override control rate. Default = `1/control_dt` from `config.yaml`. See §6. |
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

Each `.npz` has 12 arrays + 4 scalars; schema in
`PD_CALIBRATION_SPEC.md §7`.

### Fitting

```bash
python3 tools/calibration_fit/fit_pd.py /tmp/cal_dryrun/<run_id>/ \
    --mjcf sim_assets/q1_sim.mjcf
# → /tmp/cal_dryrun/<run_id>/calibration.yaml
```

`--mjcf` enables the URDF gravity-gradient subtraction (spec §8.1
Y1 path). Without it, the output has `kp_motor = kp_eff` and an
explicit comment that gravity isn't separated.

**Don't auto-edit `qmini_lab` constants.** Per spec §11, the
operator reviews `calibration.yaml` and hand-updates
`qmini_lab/.../q1/constants.py`. See `data/pd_calibration/README.md`.

---

## 6. Tick rate notes

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

## 7. Live viewer (MuJoCo backend)

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

## 8. Diagnostics: `bin/probe_modes`

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

## 9. Cheat sheet

```bash
# Build desktop with viewer:
cmake --preset desktop-mujoco && cmake --build build/desktop-mujoco -j

# Run policy + viewer, hung MJCF, stand from t=0:
cd tests/fixtures && ../../bin/run_interface --no-onnx --no-log \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2

# Dry-run calibration in sim with viewer:
cd tests/fixtures && ../../bin/pd_calibration_tool \
    --i-have-checked-the-harness --quick \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --viewer \
    --output-root /tmp/dryrun --label dr

# Real-robot calibration, one joint, full protocol:
cd bin && ./pd_calibration_tool --i-have-checked-the-harness \
    --joints 3 --label j3_knee \
    --output-root data/pd_calibration --operator <name>

# Re-zero encoders at runtime (interactive):
# Press 1, manually position robot, press z, optionally press h to verify

# Or at boot:
../../bin/run_interface --no-onnx --no-log --zero-on-start ...

# Diagnostic dump of one FSM mode:
../../bin/probe_modes 5 3 1   # mode 5, 3 s, stand_scale=1
```
