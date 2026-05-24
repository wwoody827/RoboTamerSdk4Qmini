# RoboTamerSdk4Qmini — Claude Code Context

C++ deployment SDK for the **Unitree Qmini biped**. Loads an ONNX policy
trained in `~/code/qmini_lab` (Isaac Lab 2.3 + rsl_rl 5), reads IMU + motor
state, runs inference at 100 Hz, sends torque commands.

**Training repo:** `~/code/qmini_lab` (Isaac Lab + MuJoCo sim2sim).
**Workspace:** `~/code/qmini-workspace/repos/<name>` symlinks. See the
workspace `CLAUDE.md` for cross-repo coupling.

---

## Architecture (post-HAL, May 2026)

The SDK is split by a thin HAL — above the line is pure C++ control logic;
below the line is one of three swappable backends.

```
┌─────────────────────────────────────────────────────────────────┐
│  QminiApp  →  RLController  →  ObsBuilder  →  IPolicy           │
│              (HAL frames in, MotorCmdFrame out)                 │
└──────────────────────────┬──────────────────────────────────────┘
                           │  POD frames only (types.h)
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│  HAL interfaces: IMotorBackend, IImuBackend, IJoystickBackend,  │
│                  IClock                                          │
└──────────────────────────┬──────────────────────────────────────┘
                           ▼
        ┌──────────────────┼──────────────────┐
        │                  │                  │
   ┌────▼────┐       ┌─────▼─────┐      ┌─────▼──────┐
   │   sim   │       │  mujoco   │      │  hardware  │
   │  echoes │       │  physics  │      │  real robot│
   └─────────┘       └───────────┘      └────────────┘
   no deps           libmujoco           unitree_sdk2
                                          + motor SDK
```

**Build presets** (`CMakePresets.json`):

| Preset | BACKEND | Deps | Run anywhere |
|---|---|---|---|
| `desktop-sim` | sim | Eigen + yaml-cpp (FetchContent) | yes |
| `desktop-mujoco` | mujoco | + libmujoco (any 3.x) | yes |
| `robot-release` | hardware | + unitree_sdk2, motor SDK, jsoncpp | robot only |

See `BUILDING.md` for full commands and `ARCHITECTURE.md` for the
control-loop walkthrough.

---

## Key files

| Path | Purpose |
|------|---------|
| `include/user/hal/{types,*_backend,clock,factory}.h` | POD frames + 4 interfaces |
| `include/user/qmini_app.h` | Top-level orchestrator (formerly `G1`/`custom.hpp`) |
| `include/user/rl_controller.h` | Control loop, above HAL |
| `include/user/obs_builder.h` | Pure 44-dim observation builder |
| `include/user/policy.h` | `IPolicy` (`identity` and `onnx` impls) |
| `include/user/mode_switcher.h` | FSM driven by joystick or keyboard |
| `source/user/hal/{sim,mujoco,hardware}/` | Backend implementations |
| `bin/config.yaml` | Runtime config (kp/kd, ranges, ref pose, startq) |
| `sim_assets/q1_sim.mjcf` | Baked MuJoCo model (run `build_mjcf.py` to regen) |

---

## Critical sync points with training repo (qmini_lab)

If you change **any** of the following in `~/code/qmini_lab`, you must
update this SDK too:

1. **Observation vector layout** — `obs_builder.cpp::build_obs()`. Per-step
   dim **44** (April 2026 schema; `[vx, vy, yaw]` + rpy(2) + rpy_rate(3) +
   `joint_pos - ref`(10) + `joint_vel * 0.1`(10) + `joint_act - joint_pos`(10) +
   phase_sincos(4) + freq_term(2)). Stack ×3 → 132-dim policy input.
2. **`static_threshold`** (0.15 m/s) — `ObsParams::static_threshold`.
3. **Action scaling** — `act_inc_low/high` in `config.yaml`. Phase freq
   `[0.5, 3.5]` Hz, joint increments `[-15, 15]` deg/s.
4. **Torque formula** (hardware backend) — `motor_unitree.cpp` applies the
   gear-ratio-compensated PD. For sim/mujoco, the controller passes
   `kp, kd` through `MotorCmdFrame` and the backend applies `kp×err − kd×dq + tau_ff`.
5. **Joint order** — must match `QMINI_JOINT_NAMES` in qmini_lab:
   `[hip_yaw_l, hip_roll_l, hip_pitch_l, knee_pitch_l, ankle_pitch_l,
     hip_yaw_r, hip_roll_r, hip_pitch_r, knee_pitch_r, ankle_pitch_r]`.
6. **Policy I/O shape** — `IPolicy::input_dim() == 44`, `output_dim() == 12`.
   `OnnxPolicy::infer` clamps to `[-1, 1]`.

The legacy `ConfigParams::num_observations` is still read from
`config.yaml`; mismatch with `kObsPerStep = 44` throws at startup.

---

## Deploying a new policy

1. Train in `~/code/qmini_lab` (see its CLAUDE.md).
2. After training, ONNX + manifest live in
   `~/code/IsaacLab/logs/rsl_rl/qmini_walk_v2/<run>/exported/best/`.
3. Copy to robot:
   ```
   bin/policy.onnx           ← exported/<.>/policy.onnx
   bin/policy_manifest.yaml  ← exported/<.>/policy_manifest.yaml
   ```
4. Verify `num_observations: 44` in `bin/config.yaml`. The runtime check in
   `RLController` constructor will hard-fail otherwise.
5. Rebuild only if the C++ obs builder changed (rare — it's pinned to the
   training-side layout).

---

## Quick reference

- Build desktop sim: `cmake --preset desktop-sim && cmake --build build/desktop-sim -j`
- Build with MuJoCo: `sim_assets/setup_mujoco.sh <path>` then `cmake --preset desktop-mujoco`
- Run tests: `(cd build/<preset> && ctest)`
- Run binary (sim or mujoco): from a CWD with `config.yaml` and `sim_assets/`:
  `./bin/run_interface --no-onnx --no-log`
- Robot: `cmake --preset robot-release` (must be on the ARM64 onboard computer)

For history of why things are this way:
- `HAL_PLAN.md` — original architecture proposal (status block at top)
- `REFACTOR_PLAN.md` — pre-HAL cleanups (status block at top)
- `PYTHON_REMOVAL.md` — why we dropped embedded Python and what protocols we ported
- `SIDEWALK_DESIGN.md` — the April 2026 `cmd_vy` change
- `MOTOR_PORT_MAP.md` — FTDI port → motor ID table (for hardware bring-up)
