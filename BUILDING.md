# Building

Three CMake presets, picked based on where the binary will run.

| Preset            | Where it runs                  | Extra setup |
|-------------------|--------------------------------|-------------|
| `desktop-sim`     | any Linux dev box              | none        |
| `desktop-mujoco`  | any Linux dev box with mujoco  | `sim_assets/setup_mujoco.sh` once |
| `robot-release`   | the Qmini's onboard ARM64      | `unitree_sdk2` + motor SDK installed |

All builds need a recent CMake (≥ 3.16) and a C++17 compiler. Eigen and
yaml-cpp are pulled via `FetchContent` if not found system-wide, so most
desktop builds start clean with no apt installs needed.

---

## `desktop-sim` — fastest dev loop

Toy "echo target → measured" motor model. Useful for exercising the obs
builder, mode FSM, and threading without dragging in physics.

```bash
cmake --preset desktop-sim
cmake --build build/desktop-sim -j
(cd build/desktop-sim && ctest)
```

Three tests run: `test_obs_builder`, `test_math`, `test_sim_loop`.

Run the binary from a CWD with `config.yaml`:

```bash
cd bin
./run_interface --no-onnx --no-log    # quit with Ctrl-C
```

CLI flags:

| Flag | Meaning |
|---|---|
| `--no-onnx` | Use identity policy (zeros). The build also defaults to `WITH_ONNX=OFF`, so the ONNX runtime is *not* linked at all in desktop-sim. |
| `--no-log` | Don't write `general.txt` / `rl.txt`; don't open the UDP broadcast. |
| `--keyboard` | Read mode from stdin instead of a joystick. |
| `--iface <name>` | Network interface (only used in hardware build). |
| `--policy <path>` | Override `policy.onnx` location. |

---

## `desktop-mujoco` — real physics, no robot

Same code path, but the motor + IMU backends are backed by an MuJoCo
simulation. Use this when you want to see the policy actually move a robot.

### One-time setup

Point the build at any MuJoCo install. Conda envs that already have
`mujoco` installed for training work fine — pass the `site-packages/mujoco`
path:

```bash
sim_assets/setup_mujoco.sh /home/woody/miniconda3/envs/env_isaaclab/lib/python3.11/site-packages/mujoco
```

This creates `lib/mujoco/include` and `lib/mujoco/libmujoco.so*` symlinks.
Re-run it any time you switch envs or pip-upgrade mujoco.

Then bake the MJCF (only when the URDF changes):

```bash
/home/woody/miniconda3/envs/env_isaaclab/bin/python3 sim_assets/build_mjcf.py
# → sim_assets/q1_sim.mjcf, sim_assets/meshes/*.STL
```

### Build + test

```bash
cmake --preset desktop-mujoco
cmake --build build/desktop-mujoco -j
(cd build/desktop-mujoco && ctest)
```

Four tests run: the three from desktop-sim + `test_mujoco_loop` (500-tick
physics run, asserts no NaN + robot doesn't fall through floor).

### Run

The MuJoCo backend loads `sim_assets/q1_sim.mjcf` relative to CWD, so run
from a dir where both `config.yaml` and `sim_assets/` are reachable. The
test fixtures dir works:

```bash
cd tests/fixtures
../../bin/run_interface --no-onnx --no-log
```

To watch state, use `test_mujoco_loop` instead — it prints rpy and joint
positions every 100 ticks.

---

## `robot-release` — production build for the robot

Builds against the real Unitree SDK and motor drivers. **Only configures
on the robot** (or in a cross-compile env with `unitree_sdk2` installed).

```bash
cmake --preset robot-release
cmake --build build/robot-release -j
```

Required system installs on the robot:

- `unitree_sdk2`        (https://github.com/unitreerobotics/unitree_sdk2)
- `unitree_actuator_sdk` (GO-M8010-6 branch)
- `libonnxruntime`      (ARM64 build — vendored under `lib/onnx/`)
- `libyaml-cpp-dev`, `libjsoncpp-dev`, `libeigen3-dev`

The hardware backend is **pure C++** — no Python, no pygame, no pyserial.
It talks to the IMU directly via termios @ 921600 baud and reads the
joystick from `/dev/input/jsX`. See `PYTHON_REMOVAL.md` for the protocol
references.

Reads zero offsets, kp/kd, command ranges from `bin/config.yaml` exactly
like before — no changes to the YAML schema.

---

## Common issues

- **`MuJoCo headers not found at lib/mujoco/include/mujoco/`** — run
  `sim_assets/setup_mujoco.sh <path-to-site-packages/mujoco>`.
- **`runtime error: ONNX policy requested but built with WITH_ONNX=OFF`** —
  pass `--no-onnx`, or reconfigure with `-DWITH_ONNX=ON` (needs
  onnxruntime headers + .so).
- **`bad file: config.yaml`** at start — you're running from the wrong
  CWD. The binary reads `./config.yaml`. Run from `bin/` or
  `tests/fixtures/`.
- **`pthread_mutex_init` / DDS link errors on `robot-release`** — you're
  trying to configure the hardware preset on a desktop without
  `unitree_sdk2`. Switch to `desktop-sim` or `desktop-mujoco`.
