# Architecture

This is the post-HAL design (landed May 2026). It supersedes a
pre-HAL layout where Python was embedded for joystick + IMU and motor
ownership was tangled through `G1`/`custom.hpp`. The git log of
the `hal-refactor` branch carries the migration commits; this doc
describes the design as it stands now.

---

## Layered view

```
                ┌────────────────────────────────────────────┐
                │  main()  →  QminiApp                       │
                │  CLI parse, signal handling                │
                └──────────────────────┬─────────────────────┘
                                       │
                ┌──────────────────────▼─────────────────────┐
                │  QminiApp  (qmini_app.cpp)                 │
                │    owns:  IMotorBackend                    │
                │           IImuBackend                      │
                │           IJoystickBackend                 │
                │           IClock                           │
                │           RLController                     │
                │           ModeSwitcher                     │
                │           DataReporter                     │
                │  threads: control (100 Hz)                 │
                │           mode    (50 Hz)                  │
                │           report  (100 Hz)                 │
                └──────────────────────┬─────────────────────┘
                                       │
                ┌──────────────────────▼─────────────────────┐
                │  RLController  (rl_controller.cpp)         │
                │  - update_motor_state(MotorStateFrame)     │
                │  - update_base_state(BaseStateFrame)       │
                │  - update_joystick(JoystickFrame)          │
                │  - rl_control() / stand / sin              │
                │  - to_motor_cmd(mode) → MotorCmdFrame      │
                │                                            │
                │  delegates obs build to ObsBuilder         │
                │  delegates inference to IPolicy            │
                └──────────────────────┬─────────────────────┘
                                       │ POD frames only
                                       ▼
═══════════════════════════ HAL LINE ══════════════════════════════
                                       ▼
        ┌──────────────────────────────┴──────────────────────────────┐
        │                                                             │
   ┌────▼─────────┐    ┌─────────────────┐    ┌──────────────────┐
   │ hal/sim/     │    │ hal/mujoco/     │    │ hal/hardware/    │
   │              │    │                 │    │                  │
   │ motor_sim    │    │ motor_mujoco    │    │ motor_unitree    │
   │ imu_sim      │    │ imu_mujoco      │    │ imu_serial       │
   │ joystick_kbd │    │ joystick_kbd*   │    │ joystick_linux   │
   │ clock_std    │    │ clock_std*      │    │ clock_unitree    │
   │              │    │ world.h/cpp     │    │                  │
   │              │    │ (mjModel/mjData)│    │                  │
   └──────────────┘    └─────────────────┘    └──────────────────┘
                       * shared with hal/sim/
```

**The HAL line is bit-for-bit POD** — `MotorCmdFrame`, `MotorStateFrame`,
`BaseStateFrame`, `JoystickFrame`. No DDS types, no Unitree types, no
Eigen, no embedded Python. Crossing this line up or down is a memcpy in
spirit; the controller cannot tell which backend is running.

---

## Per-tick control flow

`QminiApp::control_tick()` runs once per 10 ms:

```
1. relative_time += dt
2. motor_state  = motor_backend->read()        ─┐
3. base_state   = imu_backend->read()           │ pull from HAL
4. joystick     = joystick_backend->read()     ─┘
5. rl->update_motor_state(motor_state)
   rl->update_base_state(base_state)
   rl->update_joystick(joystick)

6. dispatch on mode:
     '1' folding / default → stand_control(ratio)   (smooth init → ref)
     '2' stand              → stand_control(ratio)
     '3' RL walk            → rl_control()
                                ├─ build_stacked_obs()   (44 × 3 = 132)
                                ├─ policy->infer(obs)    (→ 12 actions)
                                ├─ transform(net_out)    (→ inc rad/s)
                                └─ joint_increment_control(inc)
     '5' sin test           → sin_control(0.2, 2Hz, t)
     'q' e-stop             → stop_flag = true (zero-gain command emitted)

7. motor_backend->send(rl->to_motor_cmd(current_mode))
```

`mode_tick()` runs at 50 Hz, reads either the joystick frame or stdin,
and edges the FSM (`ModeSwitcher`). `report_tick()` runs at 100 Hz,
writes CSV rows and broadcasts a UDP packet at 20 Hz.

---

## The observation contract

`ObsBuilder` is the canonical implementation of the 44-dim observation.
It is **pure**: same inputs → same output, no hidden state. The
controller owns the mutable state (`pm_phase`, `pm_f`, `joint_act`) and
passes a snapshot to the builder each tick.

```
                  ┌─────────────── 44 dims ────────────────┐
                  │                                        │
   [0..2]  target_command          (vx, vy, yaw)
   [3..4]  base_rpy[:2]            (roll, pitch)
   [5..7]  base_rpy_rate * 0.5
   [8..17] joint_pos - ref_joint_act
   [18..27] joint_vel * 0.1
   [28..37] joint_act - joint_pos
   [38..41] phase_sin_cos * static_flag
   [42..43] (pm_f * 0.3 - 1)  * static_flag
```

`static_flag = (‖[vx, vy, yaw]‖ ≥ 0.15)`. Whole vector clamped to ±3.

This layout **must** match training-side `_get_observations` exactly.
That's the single biggest source of bugs in deploy; the April 2026
incident was a 43→44 mismatch. A `policy_meta.yaml` startup check
that asserts obs-dim, joint order, and action ranges agree between
the SDK config and the trained policy would catch that class of bug
at boot rather than runtime — see "Deliberately not built" below.

---

## Inference (`IPolicy`)

Two impls:

- **`IdentityPolicy`** — returns zeros. Used for sim builds without
  ONNX and for tests. The action transform maps zeros to the middle of
  each output range.
- **`OnnxPolicy`** — wraps `Ort::Session`, clamps output to `[-1, 1]`.
  Only compiled when `WITH_ONNX=ON`. When `WITH_ONNX=OFF`, a stub
  symbol throws at construction so callers don't need ifdefs.

Action transform (in `RLController::transform`):
- First 2 outputs (phase freqs)  → `[0.5, 3.5]` Hz
- Next 10 outputs (joint deltas) → `[-15, 15]` deg/s

`act_inc_low/high` in `config.yaml` sets the range; the index split
(legs vs joints) is hard-coded since it's pinned by the training schema.

---

## Backend semantics

### `sim`
- Motor: `q_measured ← α·q_target + (1-α)·q_measured_prev`, `dq` is the
  finite difference. No physics — just enough to exercise the
  end-to-end loop in milliseconds.
- IMU: returns `rpy=(0,0,0)`, `acc=(0,0,-g)`, identity quat. Constant.
- Joystick: keyboard via stdin (non-blocking termios raw mode).

### `mujoco`
- Motor: shared `World` singleton with `mjModel*` + `mjData*` and a mutex.
  `send(cmd)` applies PD (`τ = kp·(q_target - q) − kd·dq + τ_ff`) and
  advances `mj_step()` N times (N = control_dt / mj_timestep ≈ 5).
  Read returns latest `qpos`/`qvel` slice for actuated joints.
- IMU: reads the same world. Free-joint quaternion → rpy; world-frame
  angular velocity rotated into base frame. Linear acc = gravity rotated
  into base frame (good-enough resting-IMU approximation).
- Joystick + clock: reused from `sim`.

### `hardware`
- Motor: 4 parallel serial threads driving 10 GO-M8010-6 motors. Gear
  ratio + speed ratio compensation. Per-motor zero offsets from
  `config.yaml::startq`.
- IMU: termios @ 921600 baud, VSISLab framed binary protocol. Single
  reader thread, port held open across reads.
- Joystick: `/dev/input/jsX` via `<linux/joystick.h>`. Reconnects if the
  device disappears (matches the old boot.sh busy-wait).
- Clock: wraps `CreateRecurrentThreadEx` from `unitree_sdk2`.

All four hardware backends are pure C++. The pre-HAL embedded CPython
(`bin/joystick.py`, `bin/imu_*.py`, `JoystickReader`, `IMUReader`) is gone.

---

## Tests

| Test | Coverage |
|------|----------|
| `test_obs_builder` | Layout / clamp / static_flag gating; pinned to the 44-dim schema |
| `test_math`        | `smallest_signed_angle_between`, `exp_filter` |
| `test_sim_loop`    | 200 ticks with sim backends + identity policy; no NaN |
| `test_mujoco_loop` | 500 ticks with MuJoCo physics; robot stays upright |

`ctest` runs all of them in <1 s. None require ONNX, none require the
robot.

---

## Things we deliberately did **not** build

- A file-replay backend (read recorded `StateFrame` from disk and drive
  the controller deterministically). Useful for regression-testing
  against captured real-robot logs, not blocking. Natural slot:
  `source/user/hal/replay/`.
- A `policy_meta.yaml` startup check that asserts obs-dim, joint order,
  action ranges agree between SDK config and the trained policy. Would
  catch class-of-bug like the April 2026 obs-dim mismatch at startup
  instead of at runtime. Worth doing in a follow-up pass.
- Anything in `qmini_lab` — training is owned by a separate agent. The
  SDK only consumes the exported `policy.onnx` + `policy_manifest.yaml`.
