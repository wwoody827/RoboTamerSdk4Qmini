# RoboTamerSdk4Qmini — Claude Code Context

## What this repo is

C++ real-robot deployment SDK for the **Unitree Qmini biped robot**.
Loads an ONNX policy trained in `~/code/RoboTamer4Qmini`, reads IMU + motor state,
runs inference at ~100Hz, and sends torque commands to the motors.

**Related training repo**: `~/code/RoboTamer4Qmini`

---

## Build

```bash
mkdir build && cd build
cmake -DPLATFORM=arm64 ..
make -j4
```

Runs on the robot's onboard ARM64 computer. Cross-compile from x86 or build natively on the robot.

---

## Key files

| File | Purpose |
|------|---------|
| `include/user/rl_controller.h` | RLController class declaration |
| `source/user/rl_controller.cpp` | Observation building, joystick, inference |
| `include/user/custom.hpp` | G1 class (main robot interface, threads) |
| `source/user/custom.cpp` | G1 control loop, mode switching |
| `include/user/Motor_thread.hpp` | Low-level motor serial comms |
| `source/run_interface.cpp` | Entry point |
| `bin/` | Compiled binary + policy ONNX + config |

---

## Runtime modes (custom.cpp)

| Key | Mode |
|-----|------|
| `1` | Position stand (default) |
| `2` | Transition to stand (smooth) |
| `3` | **RL control** (policy running) |
| `5` | Sin test / sim gait |
| `q` | Quit / kill torque |

Mode 3 and 4 activate the joystick and run `rl_control()` each loop.

---

## Observation vector (must exactly mirror training)

Built in `RLController::get_observation()` (`source/user/rl_controller.cpp`).

| Index | Content | Notes |
|-------|---------|-------|
| 0 | cmd_vx | forward velocity |
| 1 | cmd_vy | lateral velocity ← **added April 2026** |
| 2 | cmd_yaw | yaw rate |
| 3–4 | roll, pitch | from IMU (base_rpy[0:2]) |
| 5–7 | angular velocity × 0.5 | base_rpy_rate |
| 8–17 | joint_pos − ref_joint_pos | |
| 18–27 | joint_vel × 0.1 | |
| 28–37 | joint_act − joint_pos | tracking error |
| 38–41 | sin/cos of phases × static_flag | phase modulator |
| 42–43 | (freq × 0.3 − 1.0) × static_flag | |

**Total**: 44 dims per step × 3 stacked = 132 input to policy.

`static_flag` = 1 if `‖[vx, vy, yaw]‖ ≥ 0.15`, else 0.

This changed from 43→44 dims when `cmd_vy` was added in April 2026.
**Policies trained before April 2026 are incompatible** — they expect 43-dim obs.

---

## Commands (joystick mapping)

Set in `RLController::joystick_command_process()`:

| Axis | Stick | Command |
|------|-------|---------|
| `Axis[1]` | Left stick Y | `cmd_vx` (negated: push forward → positive) |
| `Axis[0]` | Left stick X | `cmd_vy` (negated: push left → positive) ← **added April 2026** |
| `Axis[2]` | Right stick X | `cmd_yaw` (negated) |

Max velocities come from `configParams.vx_cmd_range` and `yr_cmd_range`.
`cmd_vy` reuses `vx_max` as its limit (same linear velocity scale).

**Yaw correction**: when joystick yaw is near zero and `kp_yaw_ctrl > 0`, the controller holds the heading recorded at the last non-zero yaw command. This prevents slow drift while walking straight.

---

## `target_command` type history

- **Before April 2026**: `Vec2<float>` = `[vx, yaw]`
- **After April 2026**: `Vec3<float>` = `[vx, vy, yaw]`

This change cascades to:
1. `get_observation()` — `target_command` is the first element of the obs vector
2. `static_flag` check — now uses 3D norm
3. `joystick_command_process()` — now reads 3 axes

---

## Config params (loaded from YAML in `bin/`)

Key fields used by RLController:

| Field | Purpose |
|-------|---------|
| `num_observations` | Per-step obs dim (must be 44) |
| `num_actions` | Policy output dim (12) |
| `num_stacks` | History frames (3) |
| `vx_cmd_range` | `[min, max]` forward velocity |
| `yr_cmd_range` | `[min, max]` yaw rate |
| `kp_yaw_ctrl` | Heading correction gain (0 = disabled) |
| `ref_joint_act` | Standing pose joint positions |
| `kp`, `kd` | PD gains per joint |
| `act_inc_low/high` | Action scaling bounds |

---

## Torque formula (must match training)

```
torque = kp × (target − pos) + kd_bias − vel + joint_offset − 3.5 × sign(vel) × vel_sign
```

`kd` is a **constant bias**, not velocity-proportional. `−vel` provides the actual damping (unit gain). This unusual formulation must match `legged_robot.py` in the training repo exactly.

---

## Critical sync points with training repo

If you change **any** of the following in `~/code/RoboTamer4Qmini`, you must update this SDK too:

1. **Observation vector** (`birl_task.py::pure_observation`) → `rl_controller.cpp::get_observation()`
2. **`num_obs_per_step`** in `qmini_birl.yaml` → `num_observations` in SDK config YAML
3. **`static_flag` threshold** (0.15) → same value in `get_observation()`
4. **Phase modulator formula** → `compute_pm_phase()`
5. **Action scaling bounds** → `act_inc_low/high` in config
6. **Torque formula** → `legged_robot.py` line ~414

---

## Deploying a new policy

1. Train in `~/code/RoboTamer4Qmini` → get `.onnx` from `experiments/<name>/deploy/`
2. Copy `policy_<iter>.onnx` to `bin/policy.onnx` on the robot
3. Ensure `num_observations: 44` in the SDK config YAML
4. Rebuild if any C++ was changed
5. Run `./run_interface`
