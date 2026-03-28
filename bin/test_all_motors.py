import sys
import time
sys.path.append('/home/pi/code/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import *

groups = [
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if03-port0", [0, 1], "Hip Yaw    (global 0, 5)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if01-port0", [0, 1], "Hip Roll   (global 1, 6)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if00-port0", [0, 1, 2], "Left Leg   (global 2, 3, 4)"),
    ("/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB09QAL-if02-port0", [0, 1, 2], "Right Leg  (global 7, 8, 9)"),
]

for port, ids, description in groups:
    print(f"\n=============================")
    print(f"Port: {port}")
    print(f"Group: {description}")
    print(f"=============================")

    serial = SerialPort(port)
    cmd = MotorCmd()
    data = MotorData()

    for motor_id in ids:
        # Zero torque — motor will NOT move
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
        time.sleep(0.005)

        if data.merror == 0 and (data.q != 0.0 or data.temp != 0):
            print(f"  ID {motor_id}: REPLY OK  | pos={data.q:.4f}  vel={data.dq:.4f}  temp={data.temp}  merror={data.merror}")
        else:
            print(f"  ID {motor_id}: NO REPLY  | pos={data.q:.4f}  temp={data.temp}  merror={data.merror}")

print("\nDone.")
