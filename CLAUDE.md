# RoboTamerSdk4Qmini — Claude Code Context

C++ deployment SDK for the **Unitree Qmini biped**. Loads an ONNX
policy trained in `~/code/qmini_lab` (Isaac Lab 2.3 + rsl_rl 5),
reads IMU + motor state, runs inference, sends torque commands.

This file is the LLM-facing summary. Humans should read `README.md`,
then `ARCHITECTURE.md` / `BUILDING.md` / `USAGE.md`.

**Training repo:** `~/code/qmini_lab` (Isaac Lab + MuJoCo sim2sim).
**Workspace:** `~/code/qmini-workspace/repos/<name>` symlinks.

---

## Architecture (post-HAL, May 2026)

A thin HAL separates pure-C++ control logic above the line from a
swappable backend below.

```
┌─────────────────────────────────────────────────────────────────┐
│  QminiApp  →  RLController  →  ObsBuilder  →  IPolicy           │
│              (HAL frames in, MotorCmdFrame out)                 │
└──────────────────────────┬──────────────────────────────────────┘
                           │  POD frames only (hal/types.h)
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
   no deps           libmujoco +         unitree_sdk2
                     libglfw (viewer)    + motor SDK
                                         + onnxruntime
```

| Preset (`CMakePresets.json`) | BACKEND | Extra deps |
|---|---|---|
| `desktop-sim` | sim | Eigen + yaml-cpp (FetchContent) |
| `desktop-mujoco` | mujoco | + libmujoco + libglfw3-dev (for viewer) |
| `robot-release` | hardware | + unitree_sdk2, motor SDK, jsoncpp |

---

## File index

| Path | Purpose |
|---|---|
| `include/user/hal/{types,*_backend,clock,factory}.h` | POD frames + 4 interfaces |
| `include/user/qmini_app.h` | Top-level orchestrator (`Options`, FSM, threads) |
| `include/user/rl_controller.h` | Above-HAL control: obs build → policy → action → cmd |
| `include/user/obs_builder.h` | Pure 44-dim observation builder |
| `include/user/policy.h` | `IPolicy` + `make_identity_policy` / `make_onnx_policy` |
| `include/user/mode_switcher.h` | FSM (joystick OR line-mode stdin) |
| `include/user/data_report.h` | CSV + UDP telemetry |
| `include/user/calibration/` | PD calibration trial schedule, loop, NPZ writer |
| `source/user/hal/sim/` | Sim backends (echo motor, identity IMU, kbd joystick, std clock) |
| `source/user/hal/mujoco/` | World singleton + motor / IMU thin views + GLFW viewer |
| `source/user/hal/hardware/` | Unitree SDK motors, serial IMU, jsX joystick, unitree clock |
| `source/run_interface.cpp` | `bin/run_interface` entry |
| `source/user/calibration/pd_calibration_main.cpp` | `bin/pd_calibration_tool` entry |
| `tools/probe_modes.cpp` | Diagnostic: drive FSM modes, dump NPZ (not in ctest) |
| `tools/calibration_fit/fit_pd.py` | Offline fit: NPZ → `calibration.yaml` |
| `sim_assets/build_mjcf.py` | Bake `q1_sim*.mjcf` from the qmini_lab URDF |
| `sim_assets/q1_sim.mjcf` | Free-fall MJCF (free joint + floor at z=0) |
| `sim_assets/q1_sim_hung.mjcf` | Pinned-torso MJCF (no freejoint, floor at z = −hang_z) |
| `bin/config.yaml` | Runtime config (kp, kd, ref_joint_act, ranges, startq) |
| `scripts/hardware/` | Operator scripts: motor scan, joystick pair, IMU sanity |
| `tests/` | `obs_builder`, `math`, `sim_loop`, `mujoco_loop` (ctest) |

---

## Critical sync points with `qmini_lab`

Breaking any of these silently causes sim2real divergence. Recalibrate
whenever you change one.

1. **Observation layout** — `obs_builder.cpp::build_obs()`. 44 dims:
   `[vx, vy, yaw] + rpy[:2] + rpy_rate·0.5 + (q − ref) + dq·0.1 + (act − q) + phase_sincos·static + (pm_f·0.3 − 1)·static`.
   Stack ×3 → 132-dim policy input. Clamped to ±3.
2. **`static_threshold` = 0.15 m/s** — `ObsParams`, must match training.
3. **Action scaling** — `config.yaml::act_inc_low/high`. First 2 outputs (phase freqs) → `[0.5, 3.5]` Hz; next 10 (joint deltas) → `[−15, 15]` deg/s.
4. **Joint order** — `[hip_yaw_l, hip_roll_l, hip_pitch_l, knee_pitch_l, ankle_pitch_l, hip_yaw_r, hip_roll_r, hip_pitch_r, knee_pitch_r, ankle_pitch_r]`. Pinned in `world.cpp::kJointNames`, `config.yaml`, URDF, `qmini_lab/.../constants.py::QMINI_JOINT_NAMES`.
5. **Reference standing pose** — `config.yaml::ref_joint_act` ≡ `QMINI_REF_JOINT_POSES_BY_Z[3]` (z=0.40).
6. **PD gains** — `config.yaml::kp / kd` ≡ `QMINI_STIFFNESS / QMINI_PD_DAMPING`.
7. **Policy I/O shape** — `num_observations × num_stacks = 44 × 3 = 132` input floats, 12 output floats. `OnnxPolicy::infer` clamps to `[−1, 1]`.

The runtime check in `RLController` ctor compares `cfg_.num_observations` (from YAML) against `kObsPerStep = 44` (compiled in) and aborts if they disagree.

---

## Deploying a new policy

1. Train in `~/code/qmini_lab`.
2. Exported ONNX + manifest land in `IsaacLab/logs/rsl_rl/qmini_walk_v2/<run>/exported/best/`.
3. Copy to robot's `bin/`:
   - `policy.onnx`
   - `policy_manifest.yaml`
4. Verify `num_observations: 44` in `bin/config.yaml`.
5. Rebuild only if the obs builder layout changed (rare — the schema is pinned).

---

## Quick reference

```bash
# Desktop
cmake --preset desktop-sim     && cmake --build build/desktop-sim -j
cmake --preset desktop-mujoco  && cmake --build build/desktop-mujoco -j   # needs MuJoCo install

# Robot (must run on the Orin)
cmake --preset robot-release   && cmake --build build/robot-release -j

# Tests
(cd build/desktop-mujoco && ctest)        # all 4 of obs_builder/math/sim_loop/mujoco_loop

# Run
cd tests/fixtures
../../bin/run_interface --no-onnx --no-log --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2

# Calibrate one joint, dry-run in sim with the viewer
../../bin/pd_calibration_tool --i-have-checked-the-harness \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --viewer \
    --tests A --joints 3 --output-root /tmp/dryrun --label j3
```

---

## Background reading order for a new agent

1. `README.md` — high-level pitch + repo layout.
2. `ARCHITECTURE.md` — the HAL line, control flow, observation contract.
3. `BUILDING.md` — preset → deps mapping.
4. `USAGE.md` — every CLI flag and what it does.
5. `1_calibrate_joints.md` — joint (`startq`) calibration: encoder windows, bootstrap → geometric-pin (Stage 1.5) → (or jog-refine, Stage 2) → canonical lock. Read before touching `startq`, `joint_range_tool`, `joint_geom_cal_tool`, or `joint_jog_tool`. The geometric-landmark method is specced in `GEOMETRIC_JOINT_CALIBRATION_SPEC.md` (reference images: `docs/images/geom_cal/`).
6. `2_pd_calibration.md` — PD + friction calibration runbook (sim2real): step/sine/chirp + free-release, one-motor + hip_roll strategy, and what does/doesn't work on this hardware (`tau_est` unusable → Test E out). Specs: `PD_CALIBRATION_SPEC.md`, `FREE_RELEASE_CALIBRATION_SPEC.md`; results in `docs/calibration_notes/`.
7. `PD_CALIBRATION_SPEC.md` — the per-joint PD identification protocol spec (Test A/B/C detail).
8. `PYTHON_REMOVAL.md` — only if you're touching the hardware-backend protocols (jsX, VSISLab IMU framing).
