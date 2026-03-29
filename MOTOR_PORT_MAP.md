# Motor Port Mapping

FTDI Adapter: `FTB09QAL` (single quad-channel USB-Serial converter)

---

## Port → Motor Mapping

| FTDI Port | Serial Path | Global Motor IDs | Joints |
|-----------|-------------|-----------------|--------|
| if03 | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0` | 0, 5 | Hip Yaw Left, Hip Yaw Right |
| if01 | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0` | 1, 6 | Hip Roll Left, Hip Roll Right |
| if00 | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if00-port0` | 2, 3, 4 | Left Hip Pitch, Left Knee, Left Ankle |
| if02 | `/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if02-port0` | 7, 8, 9 | Right Hip Pitch, Right Knee, Right Ankle |

---

## Motor ID to Program on Each Motor

Each motor must be programmed with its **channel ID** (position within its port group), NOT the global software ID.

| Global ID | Joint | Leg | FTDI Port | Program Motor ID |
|-----------|-------|-----|-----------|-----------------|
| 0 | Hip Yaw | Left | if03 | **0** |
| 5 | Hip Yaw | Right | if03 | **1** |
| 1 | Hip Roll | Left | if01 | **0** |
| 6 | Hip Roll | Right | if01 | **1** |
| 2 | Hip Pitch | Left | if00 | **0** |
| 3 | Knee | Left | if00 | **1** |
| 4 | Ankle | Left | if00 | **2** |
| 7 | Hip Pitch | Right | if02 | **0** |
| 8 | Knee | Right | if02 | **1** |
| 9 | Ankle | Right | if02 | **2** |

---

## Key Rule

Motors share IDs across different ports — this is fine because each port is a separate serial bus.
IDs only need to be unique **within the same port**.

```
if03 → IDs: 0, 1
if01 → IDs: 0, 1
if00 → IDs: 0, 1, 2
if02 → IDs: 0, 1, 2
```

---

## How to Set Motor IDs

Use the Unitree motor tool (ARM64 version on this Pi):

```bash
cd /home/pi/code/unitree_actuator_sdk/motor_tools/Unitree_MotorTools_v1.2.4_arm64_Linux/
sudo ./changeid
```

**Important:** Connect only ONE motor per port when changing IDs to avoid bus conflicts.

---

## Checking Current IDs

```bash
cd /home/pi/code/RoboTamerSdk4Qmini/bin
sudo python3 test_all_motors.py
```
