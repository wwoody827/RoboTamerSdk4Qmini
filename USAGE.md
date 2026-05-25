# Using the SDK

How to build the SDK and run the two top-level binaries:

- `bin/run_interface` — policy/control runtime. Connects to a backend
  (sim / mujoco / hardware), runs the FSM, optionally loads ONNX policy.
- `bin/pd_calibration_tool` — PD calibration sweep. No policy. Drives
  one joint at a time and dumps `.npz` traces to disk.

Tested on Ubuntu 24.04 + cmake 3.28, an `env_isaaclab` conda env that
has `mujoco` and `numpy/scipy/pyyaml` installed. No root needed for
the desktop builds.

---

## 1. Pick a backend

| What you want to do | Backend | Preset |
|---|---|---|
| Just run unit tests, exercise the FSM and obs builder | `sim` | `desktop-sim` |
| See the policy stand / walk under real physics on this desktop | `mujoco` | `desktop-mujoco` |
| Drive a real Qmini robot | `hardware` | `robot-release` (arm64) |

Each preset has its own `build/<preset>/` dir; switching between them
is just `cmake --preset <name>` + `cmake --build`. The output binaries
**always go to `bin/`**, so the last preset built is the one that runs
when you type `./bin/run_interface`.

---

## 2. One-time MuJoCo setup (desktop-mujoco only)

The build links against an existing MuJoCo install. The conda env that
trains the policy already has one — point the build script at it:

```bash
cd ~/code/RoboTamerSdk4Qmini
sim_assets/setup_mujoco.sh /home/woody/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/mujoco
```

This creates `lib/mujoco/include` and `lib/mujoco/libmujoco.so*` symlinks
into the conda install. Rerun it whenever you switch conda envs or
upgrade mujoco.

Then bake the MJCF from the URDF (only needed when the URDF changes):

```bash
/home/woody/miniconda3/envs/env_isaaclab/bin/python3 sim_assets/build_mjcf.py
# → sim_assets/q1_sim.mjcf  + sim_assets/meshes/*.STL
```

---

## 3. Build

```bash
cd ~/code/RoboTamerSdk4Qmini

# Desktop, MuJoCo physics (this is what you'll normally use):
cmake --preset desktop-mujoco
cmake --build build/desktop-mujoco -j

# Or just sim (faster, no physics, no MuJoCo dep):
cmake --preset desktop-sim
cmake --build build/desktop-sim -j

# Run tests for the preset you built:
(cd build/desktop-mujoco && ctest)
```

Both `run_interface` and `pd_calibration_tool` end up in `bin/`.
Confirm what the binary actually links to:

```bash
ldd bin/run_interface | grep -E "mujoco|onnx"
```

Expect `libmujoco.so` for `desktop-mujoco`, nothing for `desktop-sim`.

---

## 4. Run the policy in MuJoCo physics

The binary loads `config.yaml` and `sim_assets/q1_sim.mjcf` from CWD,
so run it from a dir that has both. The repo's `tests/fixtures/` is
exactly that:

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/run_interface --no-onnx --no-log
```

| Flag | Why |
|---|---|
| `--no-onnx` | Use identity policy (zeros). Build also defaults to `WITH_ONNX=OFF` for desktop — saves linking onnxruntime. |
| `--no-log` | Don't write `general.txt` / `rl.txt` and don't open the UDP broadcast. |
| `--keyboard` | Switch to line-mode stdin (digit + Enter). Use on hardware builds without a joystick — see note below. |
| `--policy <path>` | Load a different ONNX (only if built with `WITH_ONNX=ON`). |

### Controls (desktop default — stdin joystick, raw mode, no echo)

The sim and mujoco backends ship with a keyboard-driven sim joystick
that the binary attaches to stdin automatically. Each keystroke is a
button press; you won't see what you type (raw mode, no echo).

| Key | Action |
|---|---|
| `1` | folding / default (smooth init → MGTO ref) |
| `2` | stand (PD hold at MGTO) |
| `3` | RL walk (runs policy; needs `--policy` unless you want identity-zero actions) |
| `5` | sin test (0.2 rad amplitude, 2 Hz on a single joint) |
| `b` | quit |
| `w` / `s` | `cmd_vx` ± 0.1 m/s |
| `a` / `d` | `cmd_vy` ± 0.1 m/s |
| `q` / `e` | `cmd_yaw` ± 0.1 rad/s |
| `r` / space | reset command axes to zero |

The mode transition prints in green: `Current mode: standing…`.

### About `--keyboard`

`--keyboard` switches the mode FSM to **line-mode stdin** (type a
digit, press Enter). It exists because hardware builds may not have a
joystick paired and you still need a way to drive modes. **Don't pass
it with the sim or mujoco backend** — the keyboard joystick would
race the line-mode reader for stdin. On desktop, leave it off and use
the direct keys above.

To run with a trained policy:

```bash
../../bin/run_interface --policy /path/to/policy.onnx --no-log --keyboard
# (requires the build to have WITH_ONNX=ON; use cmake preset desktop-sim-onnx
#  or pass -DWITH_ONNX=ON when configuring desktop-mujoco)
```

---

## 5. Run the PD calibration tool

```bash
cd ~/code/RoboTamerSdk4Qmini/tests/fixtures
../../bin/pd_calibration_tool --i-have-checked-the-harness --quick \
    --output-root /tmp/calib --label sanity
```

Quick mode is the smoke-test path — one joint, one 2 s sine. Use it
to verify the binary runs against your backend before committing to
the full sweep.

Full sweep is ~25 min, drives all 10 joints through Tests A (step)
+ B (sine sweep) + C (chirp):

```bash
../../bin/pd_calibration_tool --i-have-checked-the-harness \
    --output-root data/pd_calibration \
    --label initial \
    --operator <your_name> \
    --notes "first calibration after harness setup"
```

Subset runs:

```bash
# Joint 0 only, step test only:
../../bin/pd_calibration_tool --i-have-checked-the-harness \
    --tests A --joints 0 --output-root /tmp/calib --label step_j0

# Override tick rate (prints a warning — see §6 below):
../../bin/pd_calibration_tool --i-have-checked-the-harness \
    --tick-hz 200 --output-root /tmp/calib --label hires
```

Output layout:

```
<output-root>/<run_id>/
├── manifest.json          # trial list, rates, results
├── run_meta.json          # operator, notes, harness flag
├── log.txt                # per-trial start/stop lines
├── joint_00_hip_yaw_l/
│   ├── A_kp30_kd0.5_step.npz
│   ├── A_kp30_kd1.0_step.npz
│   ├── …
│   ├── B_kp30_kd1.0_sine_0.25Hz.npz
│   └── C_kp30_kd1.0_chirp.npz
└── joint_09_ankle_r/…
```

Each `.npz` has 12 arrays (q, dq, tau_est, q_target, kp, kd, tau_ff,
IMU rpy/omega/acc, t_s + 2-row dq_target) plus 4 scalars (joint index,
label, mgto pose, achieved tick rate). See `PD_CALIBRATION_SPEC.md §7`.

### Fitting the traces

```bash
python3 tools/calibration_fit/fit_pd.py /tmp/calib/<run_id>/ \
    --mjcf sim_assets/q1_sim.mjcf
# → /tmp/calib/<run_id>/calibration.yaml
```

The `--mjcf` flag enables the URDF gravity-gradient subtraction
(spec §8.1 Y1 path). Without it, the output has `kp_motor = kp_eff`
and a comment that gravity isn't separated out.

**Don't auto-edit `qmini_lab` constants.** Per spec §11, the operator
reviews `calibration.yaml` and hand-updates
`qmini_lab/source/qmini_lab/assets/q1/constants.py`. See
`data/pd_calibration/README.md` for the recipe.

---

## 6. Tick rate notes

The PD calibration tool defaults to `1/control_dt` from `config.yaml`
(currently `0.015 s` → **66.67 Hz**) — the same rate the deployed
policy runs at. Calibrating at a different rate produces a
discrete-time PD model that doesn't match what the policy actually
sees at deploy.

Override with `--tick-hz <Hz>` only when characterizing rate-dependent
behaviour (e.g. running at 200 Hz to see the "continuous-time limit"
vs the discrete-time-at-66.67-Hz response). The tool prints a banner
warning whenever override is active.

The rate watchdog aborts a trial if the achieved rate stays below 90 %
of the requested rate for more than 500 ms continuous. If you see
`aborted_slow_rate` in a trial's manifest entry, the hardware backend
can't sustain the rate you asked for — lower `--tick-hz` and try again.

---

## 7. Visualization — current state

> **The MuJoCo backend is headless today.** It ticks physics in
> `mjModel`/`mjData` but doesn't open a viewer window. If you want to
> see the robot move in real time, see the two options below.

### Option A — post-hoc replay (works today, no code changes)

Write the policy/calibration trajectory to disk, then replay it in
the standalone Python viewer:

```bash
# Run the binary headlessly, with `--no-log` removed so general.txt/rl.txt
# capture the state stream:
cd tests/fixtures
../../bin/run_interface --no-onnx --keyboard

# Drop a small replay script alongside it:
python3 -m mujoco.viewer --mjcf sim_assets/q1_sim.mjcf
# (this opens an interactive view of the model, not the trajectory —
# for trajectory replay, write a 30-line loader that reads bin/general.txt
# and steps qpos by hand. We don't ship one yet.)
```

For the calibration tool, a replay script that walks an `.npz` file
back through `mujoco.viewer.launch_passive` is the cleanest fit; ~50
LOC of Python. Not built yet.

### Option B — add a viewer thread to the C++ backend (cleaner; a follow-up commit)

The MuJoCo library that's already linked includes `mjrender` and a
GLFW-based viewer (`simulate`). Wiring it in means:

- Adding a `Viewer` class in `source/user/hal/mujoco/` that owns a
  GLFW window, `mjvScene`, `mjrContext`.
- Spawning a viewer thread alongside the existing control thread.
- Mutex on `mjData` between the control thread (writes) and the viewer
  thread (reads `qpos`/`qvel`).
- New CMake dep: `find_package(glfw3)` (apt: `libglfw3-dev`).

Roughly 200 LOC. The Python `mujoco.viewer.launch_passive(model, data)`
shows what the API looks like — the C++ equivalent is in MuJoCo's
[`sample/`](https://github.com/google-deepmind/mujoco/tree/main/sample)
directory.

Say the word and I'll add it.

---

## 8. Cheat sheet

```bash
# Build everything desktop:
cmake --preset desktop-mujoco && cmake --build build/desktop-mujoco -j

# Run policy in mujoco physics, no real ONNX, keyboard joystick (default):
cd tests/fixtures && ../../bin/run_interface --no-onnx --no-log

# Calibration smoke test in mujoco physics:
cd tests/fixtures && ../../bin/pd_calibration_tool \
    --i-have-checked-the-harness --quick --output-root /tmp/c --label smoke

# Calibration full sweep (real robot, hardware backend):
cd bin && ./pd_calibration_tool --i-have-checked-the-harness \
    --output-root data/pd_calibration --label initial --operator you
```

```bash
# Switch back to sim (no physics, no mujoco needed):
cmake --build build/desktop-sim -j --target run_interface
ldd bin/run_interface | grep mujoco   # should be empty
```
