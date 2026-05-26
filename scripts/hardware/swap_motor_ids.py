"""
Swap motor IDs 0 and 1 on two-motor ports (if03 Hip Yaw, if01 Hip Roll).

Strategy (avoids ID conflict):
  Step 1: ID 0  →  temp ID 5
  Step 2: ID 1  →  ID 0
  Step 3: ID 5  →  ID 1

IMPORTANT: Only connect ONE port at a time when running this script.
Run with: sudo python3 swap_motor_ids.py
"""

import subprocess
import sys
import time

sys.path.append('/home/pi/code/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import *

CHANGEID = "/home/pi/code/unitree_actuator_sdk/motor_tools/Unitree_MotorTools_v1.2.4_arm64_Linux/changeid"
TEMP_ID = 5

TWO_MOTOR_PORTS = [
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0", "Hip Yaw  (if03)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0", "Hip Roll (if01)"),
]


def query_motor(serial, motor_id):
    cmd = MotorCmd()
    data = MotorData()
    cmd.motorType = MotorType.GO_M8010_6
    data.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
    cmd.id = motor_id
    cmd.kp = cmd.kd = cmd.q = cmd.dq = cmd.tau = 0.0
    serial.sendRecv(cmd, data)
    time.sleep(0.05)
    return data


def motor_present(serial, motor_id):
    data = query_motor(serial, motor_id)
    return data.merror == 0 and data.temp > 0


def change_id(port, current_id, new_id):
    print(f"  changeid: {current_id} → {new_id} ...", end=" ", flush=True)
    result = subprocess.run(
        [CHANGEID, port, str(current_id), str(new_id)],
        capture_output=True, text=True, timeout=10
    )
    output = (result.stdout + result.stderr).strip()
    if result.returncode == 0:
        print(f"OK  ({output})")
        return True
    else:
        print(f"FAILED\n    {output}")
        return False


def swap_port(port, label):
    print(f"\n{'='*50}")
    print(f"Port: {label}")
    print(f"{'='*50}")

    serial = SerialPort(port)

    # Verify both motors present
    has_0 = motor_present(serial, 0)
    has_1 = motor_present(serial, 1)
    print(f"  ID 0 present: {has_0}")
    print(f"  ID 1 present: {has_1}")

    if not has_0 or not has_1:
        print("  ERROR: Both motors (ID 0 and ID 1) must be connected. Skipping.")
        return False

    # Check temp ID not already occupied
    if motor_present(serial, TEMP_ID):
        print(f"  ERROR: Temp ID {TEMP_ID} is already in use. Skipping.")
        return False

    del serial  # Close port before changeid uses it

    time.sleep(0.2)

    print(f"  Swapping IDs 0 ↔ 1 via temp ID {TEMP_ID}:")

    if not change_id(port, 0, TEMP_ID):
        return False
    time.sleep(0.5)

    if not change_id(port, 1, 0):
        return False
    time.sleep(0.5)

    if not change_id(port, TEMP_ID, 1):
        return False
    time.sleep(0.5)

    # Verify result
    serial = SerialPort(port)
    ok0 = motor_present(serial, 0)
    ok1 = motor_present(serial, 1)
    ok5 = motor_present(serial, TEMP_ID)
    print(f"  Verify → ID 0: {'OK' if ok0 else 'MISSING'}, ID 1: {'OK' if ok1 else 'MISSING'}, temp ID {TEMP_ID}: {'CLEAN' if not ok5 else 'STILL PRESENT (error)'}")

    return ok0 and ok1 and not ok5


def main():
    print("Motor ID Swap Script — swaps IDs 0 and 1 on two-motor ports")
    print("IMPORTANT: ALL motors on ALL ports should be connected.\n")

    results = {}
    for port, label in TWO_MOTOR_PORTS:
        success = swap_port(port, label)
        results[label] = success

    print(f"\n{'='*50}")
    print("Summary:")
    for label, ok in results.items():
        status = "DONE" if ok else "FAILED"
        print(f"  {label}: {status}")


if __name__ == "__main__":
    main()
