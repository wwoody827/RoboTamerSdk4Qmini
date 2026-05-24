# RoboTamerSdk4Qmini — Claude Code Context

## What this repo is

C++ real-robot deployment SDK for the **Unitree Qmini biped robot**.
Loads an ONNX policy trained in `~/code/qmini_lab` (Isaac Lab 2.3 + rsl_rl 5),
reads IMU + motor state, runs inference at ~100Hz, and sends torque commands.

**Current training repo**: `~/code/qmini_lab`  (Isaac Lab, walking + static stand, MuJoCo sim2sim eval)
**Legacy training repo**: `~/code/RoboTamer4Qmini`  (Isaac Gym, V2 stand only — ports of rewards live in qmini_lab now)

### Workspace
Both are accessible via `~/code/qmini-workspace/repos/<name>` symlinks.
See `~/code/qmini-workspace/CLAUDE.md` for cross-repo coupling overview.

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

## Critical sync points with training repo (qmini_lab)

If you change **any** of the following in `~/code/qmini_lab`, you must update this SDK too:

1. **Observation vector layout** (`source/qmini_lab/tasks/walk_v2/walk_v2_env.py::_get_observations`) → `rl_controller.cpp::get_observation()`. Per-step dim **43** (V11+, phase clock added 4 dims: sin/cos φ_L, sin/cos φ_R).
2. **`actor_step_dim`** in `walk_v2_env_cfg.py` (currently `43`) → `num_observations` in SDK config YAML (with `obs_history=5` → 215 total).
3. **`static_threshold`** (0.15 m/s) → same in `get_observation()`.
4. **Phase clock**: `phase_base_freq=2.0` Hz, advances only when cmd_vx > threshold, reset to 0. Mirror in `rl_controller.cpp`.
5. **Action scaling**: residual range ±0.5 rad, LP α=0.55, `action_clamp=True` (V13+). SDK must clamp action to [-1, 1].
6. **`max_depenetration_velocity=0.05`** in `assets/q1/robot_cfg.py` — Isaac/sim only, no SDK effect.
7. **Manifest schema**: `source/qmini_lab/deploy/manifest.py` (Pydantic, `extra="forbid"`). SDK parses YAML — any new field breaks loader.
8. **Joint order**: `assets/q1/constants.py::QMINI_JOINT_NAMES`. URDF + SDK must match.
9. **Foot collision in URDF**: box 0.11×0.05×0.02 m (V35+) at xyz (0.005, ±0.012, -0.084). Sim only.

### Legacy reference (RoboTamer4Qmini, Isaac Gym era)

V2 static-stand rewards were ported from `~/code/RoboTamer4Qmini/envs/v2_stand_env.py` into qmini_lab. SDK's old obs layout (44 dims, no phase clock) used RoboTamer4Qmini policies. Don't deploy those on current SDK without re-aligning obs dim.

---

## Deploying a new policy

1. Train in `~/code/qmini_lab` (see its CLAUDE.md)
2. After training, ONNX + manifest live in:
   `~/code/IsaacLab/logs/rsl_rl/qmini_walk_v2/<run_name>/exported/best/`
   or `.../exported/iter_NNNNNN/`
3. Copy to robot:
   ```
   bin/policy.onnx           ← exported/<.>/policy.onnx
   bin/policy_manifest.yaml  ← exported/<.>/policy_manifest.yaml
   ```
4. Verify `num_observations: 215` (= 43 per-step × 5 stack) in SDK config YAML.
5. Rebuild if C++ obs builder changed.
5. Run `./run_interface`
