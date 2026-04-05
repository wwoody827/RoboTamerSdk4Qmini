import sys, time, math
sys.path.insert(0, '.')
from imu_receiver import read_imu_data

PORT = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
TRANS_AXIS = (1, 1, 1)  # correction applied in imu_receiver.py

def clear():
    print("\033[2J\033[H", end="")

while True:
    d = read_imu_data(port=PORT)

    roll_raw   = d["Roll"]
    pitch_raw  = d["Pitch"]
    heading    = d["Heading"]
    roll_rate  = d["RollSpeed"]
    pitch_rate = d["PitchSpeed"]
    yaw_rate   = d["HeadingSpeed"]
    ax, ay, az = d["Accelerometer_X"], d["Accelerometer_Y"], d["Accelerometer_Z"]

    # same as C++ trans_axis(-1, 1, -1)
    roll_policy  = roll_raw  * TRANS_AXIS[0]
    pitch_policy = pitch_raw * TRANS_AXIS[1]

    clear()
    print("=" * 50)
    print("  IMU DEBUG — raw sensor vs policy frame")
    print("=" * 50)
    print(f"\n  RAW (from sensor):")
    print(f"    Roll   : {math.degrees(roll_raw):+7.2f} deg  ({roll_raw:+.4f} rad)")
    print(f"    Pitch  : {math.degrees(pitch_raw):+7.2f} deg  ({pitch_raw:+.4f} rad)")
    print(f"    Heading: {math.degrees(heading):+7.2f} deg  ({heading:+.4f} rad)")
    print(f"\n  POLICY sees (after trans_axis):")
    print(f"    Roll   : {math.degrees(roll_policy):+7.2f} deg  ({roll_policy:+.4f} rad)")
    print(f"    Pitch  : {math.degrees(pitch_policy):+7.2f} deg  ({pitch_policy:+.4f} rad)")
    print(f"\n  Angular rates (raw):")
    print(f"    RollSpeed : {roll_rate:+.4f} rad/s")
    print(f"    PitchSpeed: {pitch_rate:+.4f} rad/s")
    print(f"    YawSpeed  : {yaw_rate:+.4f} rad/s")
    print(f"\n  Accelerometer (m/s²):")
    print(f"    X: {ax:+7.3f}  Y: {ay:+7.3f}  Z: {az:+7.3f}")
    print(f"    |g| = {math.sqrt(ax**2+ay**2+az**2):.3f} (expect ~9.81)")
    print()
    print("  Tilt forward  → policy Pitch should be NEGATIVE")
    print("  Tilt backward → policy Pitch should be POSITIVE")
    print("  Tilt right    → policy Roll  should be NEGATIVE")
    print("  Tilt left     → policy Roll  should be POSITIVE")
    print()
    print("  Ctrl+C to exit")
