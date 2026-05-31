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

## Troubleshooting

### Motors intermittently "do not reply" (shifting set each boot)

Symptom: `motor_status` / any tool reports `motor id=N does not reply`
for a *different* set of motors on each run (e.g. 8/10, then 6/10, then
2/10), and the LEDs are all flashing green (= powered, healthy). Running
with `sudo` seems to "fix" it intermittently.

Cause: **`ModemManager`**. On every boot/hotplug it probes each FTDI
`ttyUSB` port (opens it, sends AT commands for up to ~30 s) looking for a
cellular modem, holding the port so the motor SDK's reads time out. It
grabs different ports at different moments → the failing set shifts run
to run. `sudo` only appears to help because by the time you re-run, MM
has finished probing and released the ports.

This is **not** a power, wiring, or dialout-permission problem. The robot
has no cellular modem, so ModemManager has no purpose here. Mask it:

```bash
sudo systemctl mask --now ModemManager
systemctl is-active ModemManager     # -> inactive
```

Then a fresh boot gives a stable 10/10 with no `sudo` (your user just
needs to be in the `dialout` group: `sudo usermod -aG dialout $USER`,
then re-login). If you must keep ModemManager, instead add a udev rule
tagging the FTDI adapters with `ID_MM_DEVICE_IGNORE=1`.
