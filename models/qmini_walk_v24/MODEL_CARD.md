# Qmini walking policy — `qmini_walk_v24`

mjlab (MuJoCo-Warp) velocity-tracking actor for Qmini. Blind locomotion
(proprioception only — no height/vision). Trained on flat + small-bump terrain
with push/actuator/mass/friction domain randomization.

Checkpoint: `model_2999.pt`  |  ONNX↔torch max abs err on export: `2.15e-06`

> **This supersedes the old 44-dim Isaac contract** (`obs_builder.h` /
> `kObsPerStep=44`). The mjlab contract below is different: 39-dim/frame,
> projected-gravity (not rpy), absolute action (not increment), 3-frame stack,
> and **no phase clock**. Do not reuse the phase modulator.

## Control loop (run at 66.67 Hz)
1kHz physics × decimation 15 ⇒ **66.67 Hz** control. One step:
```
read sensors -> build 1 frame (39) -> push into 3-frame history
                -> flatten to 117 -> ONNX(obs)=raw_action(10)
                -> target = raw_action*scale + offset      (absolute joint angle)
                -> PD: tau = kp*(target-q) + kd*(-dq), clamp |tau|<=effort
                -> store raw_action as 'actions' term for next frame
```

## ONNX I/O
- input `obs`: float32 `[batch, 117]`. **Normalization is baked in — feed RAW obs.**
- output `action`: float32 `[batch, 10]` = **raw** action (apply scale+offset yourself).

## Observation — 117 = 3 frames × 39
Joint order (all 10-vectors): `hip_yaw_l`, `hip_roll_l`, `hip_pitch_l`, `knee_pitch_l`, `ankle_pitch_l`, `hip_yaw_r`, `hip_roll_r`, `hip_pitch_r`, `knee_pitch_r`, `ankle_pitch_r`

One frame (39 dims), concatenated in THIS order:

| idx | term | dim | meaning |
|----|------|-----|---------|
| 0 | `base_ang_vel` | 3 | body-frame angular velocity (gyro), rad/s [wx,wy,wz] |
| 1 | `projected_gravity` | 3 | gravity unit vector in body frame; upright≈[0,0,-1] |
| 2 | `joint_pos` | 10 | joint_pos - default_pose (rad), in joint_order |
| 3 | `joint_vel` | 10 | joint velocity (rad/s), in joint_order |
| 4 | `actions` | 10 | previous step RAW policy output (pre-scale), 10-dim |
| 5 | `command` | 3 | twist command in BODY frame [vx (m/s), vy (m/s), yaw_rate (rad/s)] |

**History layout (critical):** each term is stacked across the 3 frames
**chronologically oldest→newest**, and terms are concatenated term-by-term:
```
[ base_ang_vel(t-2), base_ang_vel(t-1), base_ang_vel(t),
  projected_gravity(t-2..t), joint_pos(t-2..t), joint_vel(t-2..t),
  actions(t-2..t), command(t-2..t) ]
```
i.e. NOT frame-major. Within a term block the 3 frames are oldest→newest.
On the very first control steps (history not yet full), the buffer pre-fills
by repeating the first frame.

Notes:
- `projected_gravity` = R_body_world^T · [0,0,-1]; from IMU orientation. Upright ≈ [0,0,-1].
- `joint_pos` is RELATIVE to the default/ref pose (subtract offset below).
- `joint_vel` is raw rad/s (no scaling).
- `actions` is the **raw** ONNX output of the PREVIOUS step (before scale/offset), not the joint target.
- `command` is in the robot BODY frame: +vx forward, +vy left, +yaw_rate CCW.
- No obs noise / no obs scaling at deploy.

## Action — 10-dim, absolute joint position
`joint_target[i] = raw_action[i] * scale[i] + offset[i]`, then PD to torque.
`offset` == default/ref standing pose. no clip (None).

| joint | scale |
|---|---|
| `hip_yaw_l` | 0.09091 |
| `hip_roll_l` | 0.14286 |
| `hip_pitch_l` | 0.06667 |
| `knee_pitch_l` | 0.11111 |
| `ankle_pitch_l` | 0.16667 |
| `hip_yaw_r` | 0.09091 |
| `hip_roll_r` | 0.14286 |
| `hip_pitch_r` | 0.06667 |
| `knee_pitch_r` | 0.11111 |
| `ankle_pitch_r` | 0.16667 |

| joint | offset (= ref pose, rad) |
|---|---|
| `hip_yaw_l` | 0.40000 |
| `hip_roll_l` | -0.10000 |
| `hip_pitch_l` | -1.50850 |
| `knee_pitch_l` | 1.00650 |
| `ankle_pitch_l` | -1.25510 |
| `hip_yaw_r` | -0.40000 |
| `hip_roll_r` | 0.10000 |
| `hip_pitch_r` | 1.50850 |
| `knee_pitch_r` | -1.00650 |
| `ankle_pitch_r` | 1.25510 |

## PD gains (apply target with these; clamp to effort)
| joint | kp (N·m/rad) |
|---|---|
| `hip_yaw_l` | 55.0 |
| `hip_roll_l` | 105.0 |
| `hip_pitch_l` | 75.0 |
| `knee_pitch_l` | 45.0 |
| `ankle_pitch_l` | 30.0 |
| `hip_yaw_r` | 55.0 |
| `hip_roll_r` | 105.0 |
| `hip_pitch_r` | 75.0 |
| `knee_pitch_r` | 45.0 |
| `ankle_pitch_r` | 30.0 |

| joint | kd (N·m·s/rad) |
|---|---|
| `hip_yaw_l` | 0.30 |
| `hip_roll_l` | 2.50 |
| `hip_pitch_l` | 0.30 |
| `knee_pitch_l` | 0.50 |
| `ankle_pitch_l` | 0.25 |
| `hip_yaw_r` | 0.30 |
| `hip_roll_r` | 2.50 |
| `hip_pitch_r` | 0.30 |
| `knee_pitch_r` | 0.50 |
| `ankle_pitch_r` | 0.25 |

| joint | effort limit (N·m) |
|---|---|
| `hip_yaw_l` | 20.0 |
| `hip_roll_l` | 60.0 |
| `hip_pitch_l` | 20.0 |
| `knee_pitch_l` | 20.0 |
| `ankle_pitch_l` | 20.0 |
| `hip_yaw_r` | 20.0 |
| `hip_roll_r` | 60.0 |
| `hip_pitch_r` | 20.0 |
| `knee_pitch_r` | 20.0 |
| `ankle_pitch_r` | 20.0 |

## Integration checklist (SDK side)
1. Replace the 44-dim `build_obs` with the 39-dim frame above (projected_gravity,
   joint_pos_rel, joint_vel, last_raw_action, command). Drop phase clock + rpy.
2. Keep a 3-deep ring buffer of frames; flatten **per-term oldest→newest** → 117.
3. Load `policy.onnx`; feed raw obs (normalization is inside).
4. Apply `target = raw*scale + offset`; send to PD with kp/kd/effort above.
5. Persist `raw_action` to feed the next frame's `actions` term.
6. Sanity: with the robot standing at ref pose and zero command, obs ≈
   [0 ang_vel, [0,0,-1] grav, 0 joint_pos_rel, 0 joint_vel, last_action, 0 cmd]
   and the action should hold the ref pose.

All values above are auto-extracted from the checkpoint by `export_onnx.py`;
see `policy_manifest.yaml` for the machine-readable form (incl. normalizer stats).
