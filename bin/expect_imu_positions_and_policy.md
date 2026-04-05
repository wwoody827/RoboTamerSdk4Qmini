# IMU Mounting, Coordinate Frames, and Policy Expectations

## Overview

The RL policy was trained in Isaac Gym using a specific coordinate frame convention.
The physical IMU must be mounted and configured so that its output matches this convention.
The mismatch is corrected in **`bin/imu_receiver.py`** at the data source level,
so the rest of the codebase (including `rl_controller.cpp`) receives already-corrected values.

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

## Software Correction: `bin/imu_receiver.py`

The axis correction is applied at the IMU data source in `bin/imu_receiver.py`,
inside the AHRS parsing block:

```python
# Axis correction: IMU uses NED (x-forward, y-right, z-down),
# policy expects ENU-like (x-forward, y-left, z-up).
# Y and Z are flipped → negate Roll and Heading; Pitch is unchanged.
result["RollSpeed"]    = AHRS_DATA[0] * -1   # Y-axis flipped
result["PitchSpeed"]   = AHRS_DATA[1]         # X-axis matches, no change
result["HeadingSpeed"] = AHRS_DATA[2] * -1    # Z-axis flipped

result["Roll"]    = AHRS_DATA[3] * -1         # Y-axis flipped
result["Pitch"]   = AHRS_DATA[4]              # X-axis matches, no change
result["Heading"] = AHRS_DATA[5] * -1         # Z-axis flipped
```

| Field        | Multiplier | Reason                        |
|--------------|------------|-------------------------------|
| Roll         | -1         | IMU Y-axis is right, not left |
| Pitch        | +1         | IMU X-axis matches            |
| Heading      | -1         | IMU Z-axis is down, not up    |
| RollSpeed    | -1         | same as Roll                  |
| PitchSpeed   | +1         | same as Pitch                 |
| HeadingSpeed | -1         | same as Heading               |

`rl_controller.cpp` uses `trans_axis(1, 1, 1)` — no further correction needed there.

---

## Physical Mounting Requirements

For this correction to be correct, mount the IMU as follows:

- **X-axis arrow on PCB** points toward the **robot's nose (forward)**
- **Z-axis** points **downward** into the robot body (board face-down, or sensor chip facing down)
- **Y-axis** points to the **robot's right**

If your IMU is mounted differently, adjust the multipliers in `bin/imu_receiver.py`:

| Mounting change              | Adjustment in imu_receiver.py             |
|------------------------------|-------------------------------------------|
| Y pointing left (not right)  | change Roll/RollSpeed multiplier to +1    |
| Z pointing up (not down)     | change Heading/HeadingSpeed multiplier to +1 |
| X pointing backward          | swap Roll↔Pitch indices and negate        |

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
- If positive: negate index 1 in `imu_receiver.py`

#### 2. Pitch — tilt backward (nose up)
- **Expected**: Policy Pitch → **positive**

#### 3. Roll — tilt left (left side down)
- **Expected**: Policy Roll → **positive**
- If negative: negate Roll multiplier in `imu_receiver.py`

#### 4. Roll — tilt right (right side down)
- **Expected**: Policy Roll → **negative**

#### 5. Heading — rotate counter-clockwise (viewed from above)
- **Expected**: Policy Heading → **increasing (positive direction)**
- If decreasing: negate Heading multiplier in `imu_receiver.py`

#### 6. Stationary check
- All rates (RollSpeed, PitchSpeed, YawSpeed) should be near **0**
- `|g|` should be close to **9.81 m/s²**
- Roll and Pitch should be near **0** if the robot is on flat ground

### Fixing the correction

If any test fails, edit the multipliers in `bin/imu_receiver.py`:
```python
result["Roll"]    = AHRS_DATA[3] * -1   # change -1 to +1 if roll sign is wrong
result["Pitch"]   = AHRS_DATA[4]        # change to * -1 if pitch sign is wrong
result["Heading"] = AHRS_DATA[5] * -1   # change -1 to +1 if heading sign is wrong
# same for RollSpeed, PitchSpeed, HeadingSpeed
```

Also update `TRANS_AXIS` in `bin/test_imu.py` to match so the test reflects reality:
```python
TRANS_AXIS = (1, 1, 1)  # correction applied in imu_receiver.py
```

No rebuild needed — changes are in Python only.

---

## Summary

| Item | Value |
|------|-------|
| IMU model | CP2102 (Silicon Labs USB-UART bridge) |
| IMU convention | NED (x-forward, y-right, z-down) |
| Isaac Gym convention | x-forward, y-left, z-up |
| Software correction | `imu_receiver.py` (Roll×-1, Pitch×+1, Heading×-1) |
| Serial port | `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` |
| Baud rate | 921600 |
| Policy uses | Roll, Pitch only (scaled ×5) |
| Heading used for | Yaw stabilization P-controller only |
