# IMU Mounting, Coordinate Frames, and Policy Expectations

## Overview

The RL policy was trained in Isaac Gym using a specific coordinate frame convention.
The physical IMU must be mounted and configured so that its output matches this convention,
or the mismatch must be corrected in software via `trans_axis` in `rl_controller.cpp`.

---

## Isaac Gym Coordinate Frame (Training Convention)

The policy expects all orientation data in this frame:

```
        Z (up)
        |
        |
        +------- Y (left)
       /
      /
     X (forward)
```

| Axis | Direction       |
|------|----------------|
| X    | Forward (robot nose) |
| Y    | Left            |
| Z    | Up              |
| Gravity | [0, 0, -9.81] m/s² (negative Z) |

### Euler Angles (Right-Hand Rule)

| Angle   | Rotation Around | Positive Direction                  |
|---------|----------------|-------------------------------------|
| Roll    | X-axis         | Tilt left (left side goes up)       |
| Pitch   | Y-axis         | Nose up                             |
| Heading | Z-axis         | Counter-clockwise viewed from above |

### What the Policy Observes

From `base_task.py` in the training code:
```python
self.base_euler[:, :2] * 5.    # [roll * 5, pitch * 5]  — heading excluded
self.base_ang_vel * 0.5        # [roll_rate, pitch_rate, yaw_rate] in body frame
```

- Only **Roll and Pitch** enter the observation (heading is excluded)
- Heading is only used for yaw stabilization in the deploy code
- All angles scaled by 5x, angular rates scaled by 0.5x

---

## Physical IMU: CP2102 (Silicon Labs) Sensor Frame

This IMU uses a **NED (North-East-Down)** convention:

```
        Z (down)
        |
        |
     Y (right) ------+
                      \
                       \
                        X (forward)
```

| Axis | Direction |
|------|-----------|
| X    | Forward   |
| Y    | **Right** (opposite to Isaac Gym) |
| Z    | **Down**  (opposite to Isaac Gym) |

### Mismatch vs Isaac Gym

| Axis | IMU    | Isaac Gym | Match? |
|------|--------|-----------|--------|
| X    | Forward | Forward  | ✓      |
| Y    | Right   | Left     | ✗ flipped |
| Z    | Down    | Up       | ✗ flipped |

Because Y and Z are both flipped, the Euler angles are affected:

| Angle   | Effect                                          |
|---------|-------------------------------------------------|
| Roll    | Sign flipped (Y-axis inverted)                  |
| Pitch   | Unchanged (X-axis matches)                      |
| Heading | Sign flipped (Z-axis inverted)                  |

---

## Software Correction: `trans_axis`

In `source/user/rl_controller.cpp`, `convert_dds_state2rl_state()`:

```cpp
Vec3<float> trans_axis(-1., 1, -1);
```

Applied to every RPY component and angular rate:
```cpp
base_rpy(i)      = imu_rpy(i)      * trans_axis(i)
base_rpy_rate(i) = imu_rpy_rate(i) * trans_axis(i)
```

| Component    | trans_axis | Reason                          |
|--------------|------------|---------------------------------|
| Roll  (i=0)  | -1         | IMU Y-axis is right, not left   |
| Pitch (i=1)  | +1         | IMU X-axis matches              |
| Heading (i=2)| -1         | IMU Z-axis is down, not up      |

---

## Physical Mounting Requirements

For `trans_axis(-1, 1, -1)` to be correct, mount the IMU as follows:

- **X-axis arrow on PCB** points toward the **robot's nose (forward)**
- **Z-axis** points **downward** into the robot body (board face-down, or sensor chip facing down)
- **Y-axis** points to the **robot's right**

If your IMU is mounted differently, adjust `trans_axis` accordingly:

| Mounting change         | Required trans_axis adjustment |
|-------------------------|-------------------------------|
| X pointing backward     | flip index 0 and 1 (swap roll/pitch, negate) |
| Y pointing left (not right) | change index 0 from -1 to +1 |
| Z pointing up (not down) | change index 2 from -1 to +1 |

---

## How to Test

### Prerequisites
```bash
sudo apt-get install python3-serial
cd /home/woody/code/RoboTamerSdk4Qmini/bin
```

### Run the IMU test script
```bash
sudo python3 test_imu.py
```

The script shows both the raw sensor values and the corrected **policy frame** values in real time.

### Verification Tests

Perform each physical tilt and check the **POLICY sees** row:

#### 1. Pitch — tilt forward (nose down)
- **Expected**: Policy Pitch → **negative**
- If positive: negate index 1 in `trans_axis`

#### 2. Pitch — tilt backward (nose up)
- **Expected**: Policy Pitch → **positive**

#### 3. Roll — tilt left (left side down)
- **Expected**: Policy Roll → **positive**
- If negative: negate index 0 in `trans_axis`

#### 4. Roll — tilt right (right side down)
- **Expected**: Policy Roll → **negative**

#### 5. Heading — rotate counter-clockwise (viewed from above)
- **Expected**: Policy Heading → **increasing (positive direction)**
- If decreasing: negate index 2 in `trans_axis`

#### 6. Stationary check
- All rates (RollSpeed, PitchSpeed, YawSpeed) should be near **0**
- `|g|` should be close to **9.81 m/s²**
- Roll and Pitch should be near **0** if the robot is on flat ground

### Fixing trans_axis

If any test fails, edit `source/user/rl_controller.cpp`:
```cpp
Vec3<float> trans_axis(-1., 1, -1);  // (roll, pitch, heading)
```

And update `bin/test_imu.py` to match:
```python
TRANS_AXIS = (-1, 1, -1)  # matches C++ trans_axis
```

Then rebuild:
```bash
cd /home/woody/code/RoboTamerSdk4Qmini/build && make -j$(nproc)
```

---

## Summary

| Item | Value |
|------|-------|
| IMU model | CP2102 (Silicon Labs USB-UART bridge) |
| IMU convention | NED (x-forward, y-right, z-down) |
| Isaac Gym convention | x-forward, y-left, z-up |
| Software correction | `trans_axis(-1, 1, -1)` |
| Serial port | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| Baud rate | 921600 |
| Policy uses | Roll, Pitch only (scaled ×5) |
| Heading used for | Yaw stabilization P-controller only |
