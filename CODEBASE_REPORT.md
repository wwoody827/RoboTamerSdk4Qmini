# RoboTamerSdk4Qmini — Codebase Report

**Project:** RoboTamerSdk4Qmini v1.0
**Type:** C++ Robotic Control SDK with ONNX Runtime Integration
**Authors:** Yanyun Chen, Tiyu Fang, Kaiwen Li, Kunqi Zhang, Wei Zhang
**Organization:** Visual Sensing and Intelligent System Lab (VSISLab), Shandong University
**License:** MIT (2025)

---

## 1. Project Overview

This SDK deploys pre-trained reinforcement learning (RL) walking policies onto the **Unitree Qmini** biped robot. The workflow is: train a gait policy in simulation with PyTorch, export it to ONNX format, then run it on real hardware at 100Hz using this framework.

---

## 2. Directory Structure

```
RoboTamerSdk4Qmini/
├── bin/                    # Executables, ONNX model, config, data logs, Python helpers
│   ├── config.yaml         # Main robot control parameters
│   ├── policy.onnx         # Pre-trained walking policy (805 KB)
│   ├── run_interface       # Production executable (ARM64, 16 MB)
│   ├── test_interface      # Test/debugging executable (22 MB)
│   ├── joystick.py         # Pygame joystick input handler
│   ├── imu_interface.py    # IMU data wrapper
│   └── imu_receiver.py     # Serial IMU parser (921600 baud)
│
├── include/
│   ├── user/               # Main C++ class headers
│   │   ├── custom.hpp      # G1: top-level robot orchestrator
│   │   ├── rl_controller.h # RLController: RL policy execution
│   │   ├── onnx_inference.h# OnnxInference: ONNX Runtime wrapper
│   │   ├── Motor_thread.hpp# MotorController: serial motor I/O
│   │   ├── mode_switcher.h # ModeSwitcher: operation mode management
│   │   ├── joystick_reader.h# JoystickReader: Python/Pygame bridge
│   │   ├── IMUReader.h     # IMUReader: Python IMU bridge
│   │   └── data_report.h   # DataReporter: telemetry logging
│   │
│   ├── utils/              # Math and config utilities
│   │   ├── config.h        # YAML config parser
│   │   ├── cpp_types.h     # Eigen type aliases
│   │   └── orientation_tools.h # RPY/quaternion utilities
│   │
│   ├── onnx/               # ONNX Runtime headers
│   ├── unitree/            # Unitree SDK headers (DDS, G1, robot base)
│   └── unitreeMotor/       # Motor SDK headers
│
├── lib/                    # Pre-compiled libraries
│   ├── aarch64/            # ARM64: libunitree_sdk2.a
│   ├── x86_64/             # x86_64: libunitree_sdk2.a
│   ├── onnx/               # libonnxruntime.so
│   └── m8010motor/         # libUnitreeMotorSDK (ARM64 + x86_64)
│
├── source/                 # C++ implementation files
│   ├── run_interface.cpp   # Main entry point
│   ├── test_interface.cpp  # Test entry point
│   └── user/
│       ├── custom.cpp      # G1 implementation
│       └── rl_controller.cpp # RLController implementation
│
├── thirdparty/             # Cyclone DDS libraries and headers
└── CMakeLists.txt          # Build configuration
```

---

## 3. Architecture

### 3.1 Thread-Based Real-Time Control

The system runs six concurrent fixed-rate threads managed by the `G1` class:

| Thread | Period | Frequency | Role |
|--------|--------|-----------|------|
| `Control` | 10ms | 100Hz | Run RL policy, compute motor commands |
| `JointStateReadWriter` | 2ms | 500Hz | Motor command/state serial I/O |
| `IMUStateReader` | 3ms | ~333Hz | Fetch IMU sensor data |
| `RunJoystick` | 4ms | 250Hz | Poll gamepad input |
| `ReportData` | 10ms | 100Hz | Log telemetry to CSV files |
| `ModeProcess` | 20ms | 50Hz | Handle mode transitions |

### 3.2 DDS Middleware Decoupling

**Cyclone DDS** (Data Distribution Service) decouples control logic from hardware I/O:

```
Control Logic
    │
    ├─ Publishes motor commands  ──→  DDS topic: rt/lowcmd
    │
    └─ Subscribes robot state   ←──  DDS topic: rt/lowstate
                                         │
                                    Motor feedback
                                    Base state (IMU)
```

Thread-safe `DataBuffer<T>` objects synchronize state between threads.

### 3.3 High-Level Data Flow

```
Joystick ──→ velocity command ─────────────────────────┐
                                                        │
IMU ────────→ base_rpy, rpy_rate ──→ observation (43D) ─┤
                                                        │
Motors ─────→ joint_pos, joint_vel ─────────────────────┘
                                                        │
                                               obs_stack (129D)
                                                        │
                                            ONNX Policy Inference
                                                        │
                                               action (12D)
                                                        │
                                         joint position increments
                                                        │
                                            Motor commands (DDS)
                                                        │
                                            Serial → Motors
```

---

## 4. Key Classes

### 4.1 `G1` — [include/user/custom.hpp](include/user/custom.hpp)

Top-level robot orchestrator. Initializes all subsystems and spawns all threads.

**Key members:**
- `RLController* rlController` — RL policy executor
- `ModeSwitcher modeSwitcher` — mode state machine
- `MotorController Motor_control` — motor serial I/O
- `IMUReader imuReader` — IMU sensor interface
- `DataReporter dataReporter` — telemetry logger
- DDS buffers: `motor_state_buffer_`, `motor_command_buffer_`, `base_state_buffer_`

---

### 4.2 `RLController` — [include/user/rl_controller.h](include/user/rl_controller.h) / [source/user/rl_controller.cpp](source/user/rl_controller.cpp)

The RL inference engine. Core responsibilities:

1. Read sensor state from DDS buffers
2. Build 43-dim observation vector
3. Stack 3 frames of history → 129-dim input
4. Run ONNX inference → 12-dim output
5. Map output to motor position increments
6. Publish commands back to DDS

**Control modes:**

| Method | Description |
|--------|-------------|
| `rl_control()` | Run live ONNX policy inference |
| `stand_control()` | Smooth interpolation to standing pose |
| `sin_control()` | Sinusoidal test motion |
| `sim_gait_control()` | Pre-recorded gait playback |

**Key constants:**
- `NUM_LEGS = 2`
- `NUM_JOINTS = 10` (5 per leg)
- `NUM_ACTUAT_JOINTS = 10`

---

### 4.3 `OnnxInference` — [include/user/onnx_inference.h](include/user/onnx_inference.h)

Thin ONNX Runtime wrapper.

```cpp
Matrix<float, Dynamic, 1> inference(Ort::Session* session,
                                     Matrix<float, Dynamic, 1> observation);
```

- Input: flat observation vector (`input_dim × stack_dim`)
- Output: action vector (`output_dim`), clamped to [-1, 1]
- Uses CPU memory allocation

---

### 4.4 `MotorController` — [include/user/Motor_thread.hpp](include/user/Motor_thread.hpp)

Manages 10 motors (5 per leg) across 4 parallel serial threads:

| Thread | Motors | Notes |
|--------|--------|-------|
| 0 | {0, 5} | Hip yaw L+R |
| 1 | {1, 6} | Hip roll L+R — uses combined ratio |
| 2 | {2, 3, 4} | Hip pitch, knee, ankle (left) |
| 3 | {7, 8, 9} | Hip pitch, knee, ankle (right) |

**Motor specs (GO_M8010_6):**
- Speed Ratio: 6.33
- Gear Ratio: 3.0
- Motors 1 & 6: use `Speed_Ratio × Gear_Ratio`
- Others: use `Speed_Ratio` only

---

### 4.5 `ModeSwitcher` — [include/user/mode_switcher.h](include/user/mode_switcher.h)

Manages the robot's operating mode state machine:

| Key | Mode | Description |
|-----|------|-------------|
| `1` | Folding | Initialization / safe rest pose |
| `2` | Standing | Position-controlled stand |
| `3` | RL Walking | Live RL policy with joystick velocity command |
| `4` | RL Standing | RL policy in standing configuration |
| `5` | Sin Test | Sinusoidal joint test motion |
| `q` | E-Stop | Emergency stop |

Input sources: keyboard, Pygame joystick, or Unitree gamepad.

---

### 4.6 `JoystickReader` — [include/user/joystick_reader.h](include/user/joystick_reader.h)

Embeds CPython to call [bin/joystick.py](bin/joystick.py) (Pygame). Returns 16-field JSON (4 axes, 2 hat, 10 buttons). Manages GIL explicitly for thread safety.

### 4.7 `IMUReader` — [include/user/IMUReader.h](include/user/IMUReader.h)

Embeds CPython to call [bin/imu_interface.py](bin/imu_interface.py) → [bin/imu_receiver.py](bin/imu_receiver.py). Returns 12-field JSON (3-axis accel, 3-axis gyro, RPY, quaternion) over serial at 921600 baud.

### 4.8 `ConfigParams` — [include/utils/config.h](include/utils/config.h)

Parses [bin/config.yaml](bin/config.yaml) at startup. Provides all tunable control parameters.

### 4.9 `DataReporter` — [include/user/data_report.h](include/user/data_report.h)

Writes two CSV log files to `bin/`:

| File | Contents |
|------|----------|
| `general.txt` | Joint pos/vel/tau, base RPY, quaternion |
| `rl.txt` | Actions, observations, phase, gait frequency |

---

## 5. Observation & Action Spaces

### Observation Vector (43 dims × 3 frames = 129 total)

| Component | Dims | Notes |
|-----------|------|-------|
| Target command `[vx, yr]` | 2 | Joystick: forward velocity + yaw rate |
| Base roll, pitch | 2 | From IMU |
| Roll/pitch rates | 2 | IMU gyro, scaled × 0.5 |
| Joint position error | 10 | `current_pos − reference_pos` |
| Joint velocity | 10 | Scaled × 0.1 |
| Gait phase `[sin, cos]` | 4 | 2 per leg |
| Gait frequency × static flag | 4 | Policy output fed back as input |
| **Total** | **43** | |

### Action Vector (12 dims)

| Component | Dims | Range | Description |
|-----------|------|-------|-------------|
| Phase frequency `[f_left, f_right]` | 2 | [0.5, 3.5] Hz | Gait clock speed |
| Joint increments `[inc_0 … inc_9]` | 10 | [-15, 15] deg | Position deltas per joint |

---

## 6. Control Cycle Detail (Every 10ms)

```
1.  Poll joystick → update [vx, yr] command
2.  Read DDS buffers → joint_pos, joint_vel, base_rpy, base_rpy_rate
3.  Compute joint_pos_error = joint_pos - reference_pos
4.  Build 43-dim observation vector
5.  Shift obs_stack: [t-2, t-1, t-1] → [t-2, t-1, t]
6.  Flatten obs_stack → 129-dim ONNX input
7.  ONNX inference → 12-dim output ∈ [-1, 1]
8.  Transform output:
      phase_freq  = map(out[0:2], [-1,1] → [0.5, 3.5])
      joint_inc   = map(out[2:12], [-1,1] → [-15, 15])
9.  Update gait phase:
      phase[leg] += 2π × f[leg] × dt
10. Integrate joint position:
      joint_act  += joint_inc × dt
      joint_act   = clamp(joint_act, act_pos_low, act_pos_high)
11. Publish DDS lowcmd:
      q_target = joint_act
      kp, kd   = from config.yaml
      tau_ff   = 0
12. MotorController sends via serial (gear-ratio compensated)
```

---

## 7. Configuration Reference (`bin/config.yaml`)

Key tunable parameters:

| Parameter | Description |
|-----------|-------------|
| `action_dim` | 12 (2 phase + 10 joints) |
| `obs_dim` | 43 |
| `frame_stack` | 3 |
| `control_dt` | 15ms (67Hz — note: threads run at 10ms/100Hz) |
| `vx_range` | [-0.3, 0.5] m/s |
| `yr_range` | [-1.0, 1.0] rad/s |
| `kp`, `kd` | Position and velocity gains per joint group |
| `act_pos_low/high` | Hard joint position bounds |
| `ref_joint_action` | Reference standing pose (10 joints) |

---

## 8. Build System

```bash
mkdir build && cd build
cmake -DPLATFORM=arm64 ..   # or arm64 / x86_64
make
# Output: bin/run_interface
```

**Toolchain:**
- C++17, `-O3` optimization, `-Wall -Wextra`
- Architecture detection: `aarch64` → ARM libs, else → x86_64 libs
- Links: `unitree_sdk2`, `onnxruntime`, `ddsc`, `ddscxx`, `yaml-cpp`, `jsoncpp`, `Python3`, `Eigen3`

---

## 9. Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| ONNX Runtime | 1.17.1+ | Neural network inference |
| Cyclone DDS | — | Publish-subscribe middleware |
| Unitree SDK2 | — | Robot communication protocol |
| Unitree Motor SDK | — | GO_M8010 motor drivers |
| Eigen3 | 3.3.7+ | Linear algebra |
| yaml-cpp | 0.6.0+ | Config file parsing |
| jsoncpp | — | JSON serialization |
| Python3 | 3.8.12+ | Joystick and IMU interfaces |
| pygame | 2.6.1+ | Joystick input |
| pyserial | — | IMU serial communication |

---

## 10. Safety Mechanisms

| Mechanism | Implementation |
|-----------|----------------|
| Emergency stop | Mode `'q'` zeroes all Kp/Kd, closes logs, calls `exit(1)` |
| Soft position limits | `joint_act.cwiseMax(low).cwiseMin(high)` every cycle |
| Thread safety | Mutexes on all shared state (`_rl_state_mutex`, `cmd_mutex_`, `fileMutex`) |
| Timing monitor | Warns if loop period deviates > 2ms from target |
| Soft control mode | Mode `'1'` sets Kp=Kd=0 (compliant/backdrivable) |

---

## 11. Hard-Coded Parameters (Requires Editing for New Hardware)

| File | Parameter | Notes |
|------|-----------|-------|
| [include/user/Motor_thread.hpp](include/user/Motor_thread.hpp) | `Startq[]` | Home joint positions |
| [include/user/Motor_thread.hpp](include/user/Motor_thread.hpp) | USB serial port IDs | FTDI device paths |
| [include/user/Motor_thread.hpp](include/user/Motor_thread.hpp) | Speed/gear ratios | Per motor group |
| [source/run_interface.cpp](source/run_interface.cpp) | `"eth0"` | Network interface |
| [bin/config.yaml](bin/config.yaml) | All gains, limits, dims | Full control config |

---

## 12. Operational Procedure

1. Build for target platform (`arm64` for Qmini hardware)
2. Ensure `policy.onnx` and `config.yaml` are present in `bin/`
3. Run `./run_interface` (may require `sudo` for serial port access)
4. Press `1` → folding/ready mode
5. Press `2` → standing mode
6. Press `3` → RL walking (use left joystick for velocity, right stick for yaw)
7. Press `q` → emergency stop

---

## 13. Summary

| Aspect | Value |
|--------|-------|
| Primary language | C++17 |
| Support language | Python 3 |
| Target robot | Unitree Qmini (biped) |
| Motor count | 10 (5 per leg) |
| Motor type | GO_M8010_6 |
| Control frequency | 100 Hz |
| Observation dims | 43 per frame × 3 frames = 129 |
| Action dims | 12 |
| Policy format | ONNX (policy.onnx, 805 KB) |
| Middleware | Cyclone DDS |
| License | MIT (2025) |
| Maintenance status | No longer maintained (per README) |