# Hardware bring-up scripts

Standalone helpers for the real robot. None of these are invoked by
`run_interface` or `pd_calibration_tool` — they're operator tools for
the first time you wire up a Qmini, or when motors need re-mapping.

| Script | What it does |
|---|---|
| `ps4_pair.sh` | Pair a PS4 controller over Bluetooth. Run once per controller. |
| `scan_motor_ids.py` | Walk the four FTDI serial buses and print every motor that responds. Use to discover unknown wiring. |
| `swap_motor_ids.py` | Reassign a motor's serial-line ID. Needed after replacing a motor or fixing a wiring mistake. |
| `test_all_motors.py` | Quick sweep — send a small command to every joint, verify all 10 respond. |
| `test_imu.py` | Connect to the IMU on its serial port, print framed packets. Confirms wiring + baud. |
| `test_joystick.py` | Dump `/dev/input/js0` events. Use to map your gamepad's buttons before running the FSM. |
| `test_udp_broadcast.py` | Listen for the `data_report` UDP packets that `run_interface --enable-logging` emits. |

These are Python and depend on:

```bash
pip install pyserial pygame  # whatever each script imports
```

They live here, not in `bin/`, because `bin/` is for **compiled outputs**
(`run_interface`, `pd_calibration_tool`, …) that real-time work depends on.

For canonical motor↔FTDI mapping see `../../MOTOR_PORT_MAP.md`.
