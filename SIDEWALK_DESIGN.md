# Sideways Walking (cmd_vy) — Design & Changes

## Overview

This document explains the full set of changes required to add lateral (sideways)
walking support to both the training repo and this SDK.

The robot previously only supported:
- Forward/backward walking (`cmd_vx`)
- Turning in place (`cmd_yaw`)

After this change it also supports:
- Sideways walking (`cmd_vy`) — left/right strafing

---

## Why it required coordinated changes in both repos

The ONNX policy is a black box that maps a fixed-size observation vector to actions.
The observation vector is built identically in two places:

```
Training:   env/tasks/birl_task.py  ::pure_observation()    (Python, Isaac Gym)
Deployment: source/user/rl_controller.cpp::get_observation() (C++, real robot)
```

If these diverge by even one element the policy outputs garbage. Adding `cmd_vy`
inserts a new element at index 1 — every downstream system that depends on obs
dimension (YAML configs, SDK config, sim2sim) must be updated simultaneously.

---

## Changes made (April 2026)

### 1. Training repo — `~/code/RoboTamer4Qmini`

#### `config/Base.py`
```python
# Before
lin_vel_y_range = [-0., 0.]   # effectively disabled

# After
lin_vel_y_range = [-0.3, 0.3]  # enables lateral command sampling during training
```

#### `env/tasks/birl_task.py` — observation
```python
# Before: commands[:, [0, 2]] = [vx, yaw]  → 2 elements
# After:  commands[:, [0, 1, 2]] = [vx, vy, yaw]  → 3 elements
self.obs_buf = torch.cat([
    self.commands[:, [0, 1, 2]],   # ← was [0, 2]
    ...
])
```
Same change applied to `pure_critic_observation()`, plus added lateral velocity
error to critic: `commands[:, [1]] - base_lin_vel[:, [1]]`.

#### `env/tasks/birl_task.py` — reward
```python
# Before: penalize ALL lateral velocity (assumes cmd_vy=0 always)
lateral_vel_rew = exp(-k * vy²)
lateral_vel_rew += -0.6/norm * |vy| * static_flag

# After: track cmd_vy (reward matching lateral target)
lateral_vel_rew = exp(-k * (cmd_vy - vy)²)
lateral_vel_rew += -0.6/norm * |cmd_vy - vy| * static_flag
```

Also fixed `vertical_vel_rew` which previously penalized `norm([vy, vz])` —
now penalizes only `vz` so intentional lateral motion isn't double-penalized:
```python
# Before
vertical_vel_rew -= 0.2/norm * norm(base_lin_vel[:, 1:]) * static_flag  # [vy, vz]

# After
vertical_vel_rew -= 0.2/norm * norm(base_lin_vel[:, [2]]) * static_flag  # vz only
```

#### `deploy/sim2sim/sim2sim.py` and `evaluate.py`
- `commands` array changed from `[vx, yaw]` to `[vx, vy, yaw]`
- Added `--cmd_vy` CLI argument to `sim2sim.py`
- `run_episode()` in `evaluate.py` accepts `cmd_vy=0.0` parameter

#### `deploy/sim2sim/configs/qmini_birl.yaml`
```yaml
# Added
cmd_vy: 0.0

# Updated
num_obs_per_step: 44   # was 43
```

---

### 2. SDK repo — `~/code/RoboTamerSdk4Qmini`

#### `include/user/rl_controller.h`
```cpp
// Before
Vec2<float> target_command;   // [vx, yaw]

// After
Vec3<float> target_command;   // [vx, vy, yaw]
```

#### `source/user/rl_controller.cpp` — joystick mapping
```cpp
// Before
float vx_cmd = 0, yr_cmd = 0;
vx_cmd = -vx_max * jsreader->Axis[1];
yr_cmd = -yr_max * jsreader->Axis[2];
...
target_command << vx_cmd, yr_cmd;

// After
float vx_cmd = 0, vy_cmd = 0, yr_cmd = 0;
vx_cmd = -vx_max * jsreader->Axis[1];  // left stick Y → forward/back
vy_cmd = -vx_max * jsreader->Axis[0];  // left stick X → left/right  ← NEW
yr_cmd = -yr_max * jsreader->Axis[2];  // right stick X → yaw
vy_cmd = std::clamp(vy_cmd, vx_min, vx_max);
...
target_command << vx_cmd, vy_cmd, yr_cmd;
```

#### `source/user/rl_controller.cpp` — static_flag
```cpp
// Before: 2D norm of [vx, yaw]
if (sqrt(pow(target_command(0), 2) + pow(target_command(1), 2)) < 0.15)

// After: 3D norm of [vx, vy, yaw]
if (sqrt(pow(target_command(0), 2) + pow(target_command(1), 2) + pow(target_command(2), 2)) < 0.15)
```

---

## Observation index shift

Before adding `cmd_vy`, the obs vector was:
```
[0] vx  [1] yaw  [2] roll  [3] pitch  ...
```

After:
```
[0] vx  [1] vy  [2] yaw  [3] roll  [4] pitch  ...
```

**All policies trained before April 2026 are incompatible.** They expect a 43-dim
input; the current code produces 44-dim. Loading an old policy will cause a
dimension mismatch error at runtime (the SDK already checks this and exits with
a clear message).

---

## SDK config YAML — what to update

When deploying a policy trained with `cmd_vy` support, make sure the SDK config has:

```yaml
num_observations: 44   # was 43
```

The SDK reads this to size the observation buffer. If it does not match the
`onnxInference.input_dim` loaded from the ONNX file, the runtime check in
`get_observation()` will catch it and exit.

---

## Known remaining issues in training

These reward terms may still partially conflict with sideways walking and could
be improved in a future training iteration:

| Term | Issue | Suggested fix |
|------|-------|---------------|
| `foot_slip_rew` (weight 0.5) | Penalizes foot lateral velocity unconditionally — feet need lateral velocity to strafe | Gate with `\|cmd_vy\| < 0.1` |
| `action_constraint_rew` | Penalizes hip_yaw and hip_roll deviating from ref — these joints are used for lateral motion | Reduce weight or gate when `\|cmd_vy\| > 0.1` |
| `leg_width_rew` | Keeps feet at fixed 14cm from body center — lateral body shift during strafing may make this harder | May need to track relative foot position instead |

---

## Testing sideways walking in sim2sim

```bash
cd ~/code/RoboTamer4Qmini

# Walk left at 0.3 m/s
python deploy/sim2sim/sim2sim.py \
    --cmd_vx 0.0 --cmd_vy 0.3 --cmd_yaw 0.0 \
    --policy experiments/<name>/deploy/policy_<iter>.onnx

# Walk diagonally
python deploy/sim2sim/sim2sim.py \
    --cmd_vx 0.3 --cmd_vy 0.3 --cmd_yaw 0.0 \
    --policy experiments/<name>/deploy/policy_<iter>.onnx
```

Note: policies trained before April 2026 will fail with a dimension error.
You need to train a fresh policy with the updated code to test sideways walking.
