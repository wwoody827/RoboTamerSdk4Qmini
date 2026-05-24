# Python Removal — Design Note

## Decision

The SDK no longer embeds CPython. Both consumers — joystick and IMU — are
ported to pure C++.

| Before | After |
|---|---|
| `IMUReader.h` → `bin/imu_interface.py` → `bin/imu_receiver.py` (pyserial) | `hal/hardware/imu_serial.cpp` (termios + binary frame parser) |
| `JoystickReader` → `bin/joystick.py` (pygame, SDL dummy driver) | `hal/hardware/joystick_linux.cpp` (`/dev/input/jsX`, `<linux/joystick.h>`) |

## Why

1. **Deployment is simpler.** Robot needs the binary and ONNX. No `pip install`,
   no `requirements.txt`, no Python version pinning, no pygame on a headless
   ARM machine.
2. **Build is simpler.** `Python3::Python` link dep dropped. ARM cross-compile
   no longer needs matching Python dev headers in the sysroot.
3. **Real bugs go away.**
   - `Py_Initialize` / `Py_Finalize` called twice (IMUReader + JoystickReader)
     → second `Py_Finalize` is UB.
   - `imu_receiver.read_imu_data()` opens a fresh 921600-baud serial port on
     **every call** — i.e., every IMU tick at ~333 Hz. C++ port holds the
     port open once.
   - GIL juggling inside a 100 Hz real-time control thread.
4. **Sim build is automatically clean** — desktop-sim never had a reason to
   pull Python in.

## Protocol references (so the C++ ports are auditable)

### Joystick — Linux kernel API

`open("/dev/input/js0", O_RDONLY)` then `read()` returns `struct js_event`:

```c
struct js_event {
    __u32 time;     // event timestamp (ms)
    __s16 value;    // axis: signed range, button: 0/1
    __u8  type;     // JS_EVENT_BUTTON | JS_EVENT_AXIS | JS_EVENT_INIT
    __u8  number;   // axis or button index
};
```

Axis values are normalized to `[-32767, 32767]`; SDK divides by 32767 to match
the old pygame `[-1, 1]` range. Button events are 0 / 1.

Button / axis index map (same as the old pygame code, which is what the
training-side joystick mapping expects):

| pygame index | Function | SDK use |
|---|---|---|
| `axis[0]` | left stick X | `cmd_vy` (negated) |
| `axis[1]` | left stick Y | `cmd_vx` (negated) |
| `axis[2]` | right stick X | `cmd_yaw` (negated) |
| `axis[3]` | right stick Y | unused |
| `button[0]` | A | mode '2' stand |
| `button[1]` | B | quit |
| `button[3]` | X | mode '4' RL stand |
| `button[4]` | Y | mode '3' RL walk |
| `button[6]` | L1 | mode '6' lateral |
| `button[7]` | R1 | mode '7' |
| `button[8]` | L2 | mode '8' |
| `button[9]` | R2 | mode '9' |
| `button[10]` | SELECT | mode '5' sin test |
| `button[11]` | START | mode '1' ready |

These indices are the **kernel jsX indices**, which match pygame's mapping for
a PS4 / DualShock-style controller.

### IMU — VSISLab serial frame protocol (921600 baud)

Frame format, byte by byte:

```
[0xFC] [type] [len] [sn] [crc8] [crc16_H] [crc16_L] [payload …]
```

| Type | Meaning | Payload length |
|---|---|---|
| `0x40` | IMU (accel/gyro) | 0x38 = 56 bytes |
| `0x41` | AHRS (rpy + quat) | 0x30 = 48 bytes |
| `0x42` | INSGPS | 0x48 (unused) |
| `0x5C` | Geodetic pos | 0x20 (unused) |
| `0x50` | System state | 0x64 (unused) |
| others | various | discarded |

CRC and seq bytes are read but **not validated** (preserving current
behavior; can add CRC validation later if desired).

#### TYPE_IMU (0x40) payload — 56 bytes

`struct.unpack('12f ii', ...)`. Used fields:
- `imu[3]`, `imu[4]`, `imu[5]` → accelerometer XYZ (m/s²)

#### TYPE_AHRS (0x41) payload — 48 bytes

`struct.unpack('10f ii', ...)`. Used fields, with the axis remapping the
Python code applies:
- `RollSpeed`  = ahrs[1]
- `PitchSpeed` = -ahrs[0]
- `HeadingSpeed` = ahrs[2]
- `Roll`  = ahrs[4]
- `Pitch` = -ahrs[3]
- `Heading` = ahrs[5]
- `qw, qx, qy, qz` = ahrs[6..9]

The C++ port preserves these axis flips bit-for-bit.

#### Serial config

8N1, no flow control, raw mode, 921600 baud, blocking reads. termios:
`BOTHER` + `c_ispeed/c_ospeed` for arbitrary baud, or
`B921600` on glibc-based Linux (supported since 2.4.x kernels).

## Files removed / repurposed

| Path | Status |
|---|---|
| `include/user/IMUReader.h` | deleted (logic moved to `hal/hardware/imu_serial.cpp`) |
| `include/user/joystick_reader.h` | deleted (logic moved to `hal/hardware/joystick_linux.cpp`) |
| `bin/imu_interface.py` | deleted |
| `bin/imu_receiver.py` | kept as reference under `bin/python_legacy/` for protocol audit; not executed |
| `bin/joystick.py` | deleted |
| `bin/test_imu.py`, `bin/test_joystick.py` | kept (standalone diagnostic tools) |

## Verification

- Compile: hardware build no longer links `Python3::Python`. Verified by
  `ldd run_interface` not showing `libpython*.so`.
- IMU on the bench: capture 10 seconds of frames in both old and new readers,
  diff RPY/quat — should match within float epsilon.
- Joystick on the bench: run `bin/test_joystick_cpp`, wiggle sticks, compare
  values against the old pygame readout.
