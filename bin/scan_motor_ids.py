"""Scan all IDs 0-7 on all ports to find what's actually programmed."""
import sys
import time
sys.path.append('/home/pi/code/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import *

groups = [
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0", "Hip Yaw    (expect IDs 0,1)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0", "Hip Roll   (expect IDs 0,1)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if00-port0", "Left Leg   (expect IDs 0,1,2)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if02-port0", "Right Leg  (expect IDs 0,1,2)"),
]

for port, description in groups:
    print(f"\n=============================")
    print(f"Port: {port}")
    print(f"Group: {description}")
    print(f"=============================")

    serial = SerialPort(port)
    cmd = MotorCmd()
    data = MotorData()

    for motor_id in range(8):
        data.motorType = MotorType.GO_M8010_6
        cmd.motorType  = MotorType.GO_M8010_6
        cmd.mode = queryMotorMode(MotorType.GO_M8010_6, MotorMode.FOC)
        cmd.id  = motor_id
        cmd.kp  = 0.0
        cmd.kd  = 0.0
        cmd.q   = 0.0
        cmd.dq  = 0.0
        cmd.tau = 0.0

        serial.sendRecv(cmd, data)
        time.sleep(0.01)

        if data.merror == 0 and data.temp > 0:
            print(f"  ID {motor_id}: FOUND  | pos={data.q:.4f}  temp={data.temp}  merror={data.merror}")

print("\nDone.")
