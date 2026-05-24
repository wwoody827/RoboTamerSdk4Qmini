# HAL Plan — Desktop Development & Robot Release

> **Status (May 2026): LANDED.**
> This document is kept as the original architecture proposal. The
> realized design is documented in [ARCHITECTURE.md](ARCHITECTURE.md) and
> the build flow in [BUILDING.md](BUILDING.md).
>
> | Step | Status | Notes |
> |------|--------|-------|
> | 1. HAL types + interfaces + factory | ✓ | `include/user/hal/` |
> | 2. Joystick backend (Linux jsX, no Python) | ✓ | Pure C++ instead of pygame |
> | 3. IMU backend (termios, no Python) | ✓ | Pure C++ instead of pyserial |
> | 4. Motor backend | ✓ | POD I/O at the boundary |
> | 5. Clock abstraction | ✓ | `IClock`, hardware wraps `CreateRecurrentThreadEx` |
> | 6. CMake split + presets | ✓ | `desktop-sim`, `desktop-mujoco`, `robot-release` |
> | 7. Golden-vector test for obs | ✓ | `tests/test_obs_builder.cpp` |
> | 8. Replay backend (file-driven) | ✗ deferred | Superseded by `BACKEND=mujoco` for sim2sim; file-replay still useful for regression on real logs |
> | 9. `policy_meta.yaml` startup gate | ✗ deferred | Recommended next pass |
>
> The python-removal design picked **Option A** (pure C++ everywhere).
> See [PYTHON_REMOVAL.md](PYTHON_REMOVAL.md).

---


## Goal

Separate the development environment from the real-running environment so most
work can happen on a desktop (no motors, no IMU, no Unitree drivers), with a
clean "release" path that pushes a tested build to the robot.

Concretely:
- Desktop build compiles without `unitree_sdk2`, `unitreeMotor`, serial drivers,
  or embedded Python.
- The same `rl_controller.cpp` (obs vector, ONNX, torque formula) runs in both
  environments — no `#ifdef SIM` branches above the HAL line.
- Tag-driven release artifact for the robot.

---

## Hardware seams (where the cuts go)

Only four narrow seams couple the code to hardware:

| Seam | Current location | Call sites |
|---|---|---|
| **Motor I/O** | `Motor_thread.hpp::MotorController` | `custom.cpp:116-117` only |
| **IMU** | `IMUReader.h` (Python+JSON) | `custom.cpp:107-110, 128` |
| **Joystick / gamepad** | `joystick_reader.h` (Python sticks) + `XRockerGamepad` (DDS buttons) | `rl_controller.cpp:147-149`, `mode_switcher.h` |
| **Recurrent threads** | `CreateRecurrentThreadEx` (unitree_sdk2 symbol) | `custom.hpp:68-102` |

Hidden coupling: `include/unitree/g1/motors.hpp` declares POD `MotorCommand` /
`MotorState` but `#include`s the DDS IDL, which transitively forces a link to
`ddsc`/`ddscxx`. We route around it with our own POD types in `hal/types.h`.

---

## New layout

```
include/user/hal/
    types.h               # POD MotorCmdFrame, MotorStateFrame, BaseStateFrame, JoystickFrame
    motor_backend.h       # IMotorBackend interface
    imu_backend.h         # IImuBackend interface
    joystick_backend.h    # IJoystickBackend interface
    clock.h               # IClock::create_recurrent(...) abstraction
    factory.h             # make_motor_backend() / make_imu_backend() / ...

source/user/hal/
    hardware/             # built only when BACKEND=hardware
        motor_unitree.cpp     # ← Motor_thread.hpp body
        imu_unitree.cpp       # ← IMUReader.h body
        joystick_unitree.cpp  # ← JoystickReader + XRockerGamepad
        clock_unitree.cpp     # wraps CreateRecurrentThreadEx
    sim/                  # built only when BACKEND=sim
        motor_sim.cpp         # echoes target → measured with small lag
        imu_sim.cpp           # level-stance state, optional yaml scenario
        joystick_keyboard.cpp # keyboard input or scripted command
        clock_std.cpp         # std::thread + steady_clock
    replay/               # built only when BACKEND=replay
        motor_replay.cpp      # reads logged frames from file
        imu_replay.cpp
        joystick_replay.cpp
        clock_logical.cpp     # step-N-ticks for tests
```

---

## File-by-file changes to existing code

### Surgery only

| File | Change |
|---|---|
| `include/user/custom.hpp` | Replace `MotorController Motor_control`, `IMUReader imuReader`, `XRockerGamepad xRockerGamepad` members with `std::unique_ptr<I…Backend>`. Drop `#include "Motor_thread.hpp"` and `IMUReader.h`. Drop direct `ChannelFactory::Init`. Replace `CreateRecurrentThreadEx` with `clock_->create_recurrent(...)`. |
| `source/user/custom.cpp` | `JointStateReadWriter()` → `motor_->send(frame); motor_->read(state);`. `IMUStateReader()` → `imu_->read(base_state);`. `RecordBaseState()` reads from `BaseStateFrame`. `SetMotorCmd()` returns POD `MotorCmdFrame`, not `LowCmd_`. |
| `include/user/rl_controller.h` | Replace `JoystickReader *jsreader` with `IJoystickBackend *joystick`. Drop `#include "unitree/g1/joystick.hpp"`. Replace `Gamepad *gamepad` with POD `JoystickFrame*` or wrap. |
| `source/user/rl_controller.cpp` | `joystick_command_process()` reads `joystick_->frame().Axis[...]`. Obs vector / ONNX / torque formula untouched. |
| `include/user/mode_switcher.h` | Replace embedded `JoystickReader jsreader` with `IJoystickBackend*` injected. Drop `unitree::common::xKeySwitchUnion*` or wrap. |
| `Motor_thread.hpp`, `IMUReader.h`, `joystick_reader.h` | Bodies move into `hal/hardware/*.cpp`; old headers deleted. |
| `source/run_interface.cpp` | Unchanged; `G1` picks backend via `factory.h` per compile flag. |

### New files

| File | Purpose |
|---|---|
| `hal/types.h` | POD `MotorCmdFrame {q, dq, kp, kd, tau}[10]`, `MotorStateFrame {q, dq, tau}[10]`, `BaseStateFrame {rpy, omega, acc, quat}`, `JoystickFrame {Axis[4], Hat[2], But[10]}`. Replaces `MotorCommand`/`MotorState`/`BaseState` from vendored headers — kills the IDL dependency. |
| `hal/factory.h` | `make_motor_backend()` etc. `#ifdef HAL_HARDWARE` returns Unitree impl, else returns sim/replay impl. Resolved at link time. |
| `tests/test_obs_vector.cpp` | Feeds canned `BaseStateFrame` + joint state, asserts obs matches `golden_obs.txt`. The regression test that would have caught the April 2026 obs-dim change. |
| `tests/test_torque_formula.cpp` | Pins `kp×err + kd_bias − vel + offset − 3.5·sign(vel)·vel_sign` against training repo values. |
| `scripts/release.sh` | Tag → arm64 cross-build → tarball of `run_interface + policy.onnx + config.yaml + policy_meta.yaml`. |

---

## Consistency strategy (sim ↔ real ↔ training)

Three layers, ordered by leverage:

### 1. Structural — free, enforced by compiler
- Both backends implement the same `I*Backend` interface and produce identical
  POD frames. Code above the HAL line is bit-for-bit shared.
- Interfaces are read/write POD only. No callbacks, no flags. Wide interfaces
  invite divergence.

### 2. Replay backend — catches sim/real drift
- Third backend reads recorded `BaseStateFrame`/`MotorStateFrame`/`JoystickFrame`
  from a log file at the original tick rate.
- Workflow: log a short run on the real robot → feed the log through the
  desktop `replay` build → assert emitted actions match bit-for-bit.
- Any divergence = non-determinism, leaked sim branch, or a real bug.
- Almost free once HAL exists. **More valuable than dynamics simulation.**

### 3. Cross-repo sync — catches the April-2026 footgun
- Training repo's policy export step emits `policy_meta.yaml` alongside
  `policy.onnx`:
  ```yaml
  obs_dim: 44
  num_stacks: 3
  static_flag_thresh: 0.15
  action_low: [...]
  action_high: [...]
  joint_order: [HYL, HRL, HPL, KL, AL, HYR, HRR, HPR, KR, AR]
  control_dt: 0.01
  ```
- SDK refuses to start if `policy_meta.yaml` is missing, or any field
  disagrees with `config.yaml` or with compiled-in constants.
- The ONNX runtime exposes the model's input shape — assert it equals
  `obs_dim × num_stacks`. The April 2026 43↔44 mismatch would have died
  at startup.

---

## Test tiers

Build with `desktop-sim` preset only, run via `ctest`. Use `gtest`.

| # | Tier | What | Catches |
|---|---|---|---|
| 1 | Pure-function unit | `quat_*`, `rpy_to_quat`, `smallest_signed_angle_between` (±π wrap), `exp_filter`, `compute_pm_phase` | math regressions |
| 2 | Golden obs vector | Canned input → `ObsBuilder::build()` → diff against `golden_obs.txt`. Cases: static, walking, both sides of 0.15 threshold | obs-vector drift (the April 2026 class of bug) |
| 3 | Policy startup smoke | Load `bin/policy.onnx`, assert `input_dim == 44 × 3`, push zeros, assert finite | dim mismatch, fast fail |
| 4 | End-to-end sim loop | Build `G1` with sim backends, run 100 ticks across modes 1→2→3→q. Assert no NaNs, commands stay in bounds, no deadlock | integration, threading, mode transitions |
| 5 | Replay regression | Logs in `tests/data/*.log`, replay backend → assert action sequence matches `actions.golden` | behavior-preserving refactors verified |
| 6 | Real-robot smoke (manual) | `./run_interface --smoke`: log 3s of stand, compare against desktop-sim replay of same input log. Run before flipping release symlink | sim ↔ real divergence at deploy time |

Tier 5 requires deterministic control loop ordering — see "open questions" below.

---

## CMake changes

- New cache var: `set(BACKEND "hardware" CACHE STRING "hardware | sim | replay")`.
- Guard the unitree-SDK linking block behind
  `if(BACKEND STREQUAL "hardware")`.
- Glob `source/user/hal/${BACKEND}/*.cpp` into the executable.
- Re-enable the commented x86_64 link branch.
- Add `CMakePresets.json` with `desktop-sim` (x86, BACKEND=sim) and
  `robot-release` (aarch64, BACKEND=hardware, Release).

---

## Implementation order (each step keeps robot build working)

1. Add `hal/types.h` + empty `hal/*_backend.h` interfaces + `hal/factory.h`.
   No behavior change.
2. **Joystick backend.** Least entangled — no DDS leak. Move `JoystickReader`
   body to `hal/hardware/joystick_unitree.cpp`. Add `joystick_keyboard.cpp`.
   Update `rl_controller.cpp:147-149` and `mode_switcher.h`.
3. **IMU backend.** Same pattern.
4. **Motor backend.** Hardest because of `LowCmd_`. Translation
   `MotorCmdFrame → LowCmd_` becomes private to `motor_unitree.cpp`.
   `SetMotorCmd()` in `custom.cpp` returns POD.
5. **Clock abstraction.** Wrap `CreateRecurrentThreadEx`. After this,
   `custom.hpp` no longer needs `unitree/common/thread/thread.hpp`.
6. **CMake split + desktop-sim preset.** Desktop build link succeeds.
7. **Golden-vector test** for obs vector. Run in CI.
8. **Replay backend.** Builds on (1)–(6). Implements tier 5 tests.
9. **`policy_meta.yaml`** in training repo + assertion in SDK startup.

---

## Tradeoffs / open questions

- **`LowCmd_` DDS type is the stickiest piece.** Cleanest fix: convert to
  POD at the hardware backend boundary (~15 lines). Cheaper: keep
  `LowCmd_` and ship a stub for sim. Pick clean.
- **Sim fidelity ≠ free.** A "motor echoes target" sim won't tell you the
  robot will stand. It tells you the code path runs end-to-end (obs vector
  well-formed, ONNX loads, no NaNs). That covers most regressions. For
  real dynamics, wire to MuJoCo in the training repo as a later step.
- **Python embedding stays for hardware build only.** Sim backends must not
  use embedded Python or the desktop build still needs Python dev headers.
- **Replay determinism** requires the control loop to be deterministic.
  Today `CreateRecurrentThreadEx` introduces wall-clock-driven ordering
  between control/imu/motor/joystick threads. `clock_logical.cpp` should
  expose a "step N ticks" mode for tests. Design in from step 5.

---

## Release workflow (separate from HAL but enabled by it)

- `git tag v1.x.y` → CI builds arm64 → bundles `run_interface + policy.onnx +
  config.yaml + policy_meta.yaml` into `release-v1.x.y.tar.gz`.
- `scripts/deploy.sh` rsyncs tarball to robot, untars under
  `/opt/qmini/releases/v1.x.y/`, flips `current` symlink.
- Rollback = re-flip the symlink.
