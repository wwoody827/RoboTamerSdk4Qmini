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

## Current policy: V2 stand (May 2026)

`bin/policy.onnx` is `deploy/v2_stand_best/policy.onnx` from `RoboTamer4Qmini`
(v2_run25 iter 2800, MuJoCo-verified). Stand-only task. **Commands are forced to
`[0, 0, 0]` regardless of joystick input.** See `deploy/v2_stand_best/DEPLOY.md`
in the training repo for the authoritative spec.

## Observation vector (V2, 39 dims per step)

Built in `RLController::get_observation()` (`source/user/rl_controller.cpp`).

| Index   | Content                    | Scaling | Source |
|---------|----------------------------|---------|--------|
| 0–2     | commands                   | —       | `[0,0,0]` (forced) |
| 3–5     | base_ang_vel × 0.5         | × 0.5   | body-frame ω (stored in `base_rpy_rate`) |
| 6–8     | projected_gravity          | —       | `R_world_to_body @ [0,0,-1]` |
| 9–18    | joint_pos − ref_joint_pos  | —       | 10 joints |
| 19–28   | joint_vel × 0.1            | × 0.1   | 10 joints |
| 29–38   | joint_act − joint_pos      | —       | tracking error |

After concat, clip every element to `[-3.0, +3.0]`.

## Observation history (V2)

- Rolling buffer of **9 frames** (`OBS_BUFFER_LEN`), each 39-dim.
- Initialized to zeros at boot and on every `reset()` (mode switch).
- Each policy step: pop oldest, push newest.
- Network input is **5 frames** sampled at indices `[0, 2, 4, 6, 8]` (skip=2,
  oldest → newest), concatenated to **195 dims**.
- The first ~9 policy steps include zero-padded history — intentional, matches
  training (DEPLOY §5).

## Joint order (10 joints, identity `jointIndex2Sim`)

```
[hip_yaw_L, hip_roll_L, hip_pitch_L, knee_L, ankle_L,
 hip_yaw_R, hip_roll_R, hip_pitch_R, knee_R, ankle_R]
```

---

## Commands (V2 stand: ignored)

`joystick_command_process()` still runs (it updates `target_command` for
telemetry/logging), but **`get_observation()` zeroes the command slot before
inserting it into the obs vector**. The joystick has no effect on the policy in
V2 stand mode.

The joystick mapping itself is unchanged from the BIRL-era code in case a
walking policy is plugged in later.

---

## Config params (loaded from YAML in `bin/`)

Key fields used by RLController for V2:

| Field | Purpose |
|-------|---------|
| `num_observations` | Network input dim (195 = 39 × 5) |
| `num_actions` | Policy output dim (10) |
| `num_stacks` | Must be 1 — stacking is done inside RLController |
| `ref_joint_act` | Standing pose joint positions |
| `kp`, `kd` | Standard PD gains (V2: hip_yaw 55, hip_roll 105, hip_pitch 75, knee 45, ankle 30) |
| `residual_low/high` | Per-joint residual range (V2: ±0.5 rad) |
| `action_lowpass_alpha` | EMA factor on the residual (V2: 0.75) |
| `act_pos_low/high` | Joint position clip (URDF limits) |
| `vx_cmd_range`, `yr_cmd_range`, `kp_yaw_ctrl` | Unused in V2 stand — kept for future walking policies |
| `act_inc_low/high` | Legacy BIRL field — unused by V2 but still loaded |

---

## Action transform (V2 residual mode)

In `RLController::apply_residual_action()`:

```
clipped   = clip(net_out, -1, 1)
offset    = clipped * scale + bias    // scale=(high-low)/2, bias=(high+low)/2
lp_target = alpha * offset + (1-alpha) * lp_target_prev
joint_act = ref_joint_pos + lp_target
joint_act = clip(joint_act, act_pos_low, act_pos_high)
```

`_lp_target` is persisted across policy steps and **reset to zero (not to
ref_joint_pos) on every `reset()`**.

## Torque (delegated to motor driver)

The Unitree GO_M8010_6 motor driver runs `τ = kp*(q_target - q) - kd*dq`
internally at high rate. The SDK passes `q_target`, `kp`, `kd` per joint and
sets `tau_ff = 0`, `dq_target = 0`. No custom torque formula in the SDK loop.

(Earlier BIRL training used a non-standard formula with kd as a constant bias
plus a velocity-sign term — that's **gone** in V2. Standard PD only.)

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
6. Run `./run_interface`
