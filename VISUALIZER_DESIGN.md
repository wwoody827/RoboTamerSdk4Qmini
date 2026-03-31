# Real-time Robot State Visualizer — Design Doc

## Overview

Two-machine setup:
- **Robot machine** (Pi/Jetson): runs `run_interface`, streams UDP
- **Viewer machine** (training PC): runs `visualizer.py`, displays live plots

```
Robot (Pi/Jetson)                    Viewer Machine (Training PC)
┌─────────────────────────┐          ┌──────────────────────────┐
│  run_interface @ 100Hz  │          │  visualizer.py           │
│                         │  UDP     │                          │
│  DataReporter ──────────┼─────────▶│  UDPReceiver thread      │
│  (every 5 frames=20Hz)  │  :9870   │  matplotlib FuncAnimation│
│  broadcast 255.255.255  │          │  8 subplots live update  │
└─────────────────────────┘          └──────────────────────────┘
```

---

## Part 1 — What Was Done in the SDK (`RoboTamerSdk4Qmini`)

Both changes are committed to branch `claude/study-motor-zero-calibration-7qjpF`.

### 1.1 `include/user/data_report.h` — UDP broadcast added

**New includes** (top of file):
```cpp
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
```

**New private members:**
```cpp
int udp_sock_    = -1;
int udp_counter_ = 0;
struct sockaddr_in udp_addr_{};
```

**`init()` signature change** — added optional UDP params:
```cpp
void init(bool general, bool rl,
          const char* udp_broadcast_ip = "255.255.255.255",
          int udp_port = 9870)
```

Inside `init()`, after file setup:
```cpp
udp_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
int broadcast = 1;
setsockopt(udp_sock_, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));
memset(&udp_addr_, 0, sizeof(udp_addr_));
udp_addr_.sin_family      = AF_INET;
udp_addr_.sin_port        = htons(udp_port);
udp_addr_.sin_addr.s_addr = inet_addr(udp_broadcast_ip);
```

**`report_data()` — UDP send block** appended after file writes:
```cpp
if (udp_sock_ >= 0 && !_general_buffer.empty()) {
    udp_counter_++;
    if (udp_counter_ >= 5) {          // 100Hz / 5 = 20Hz
        udp_counter_ = 0;
        std::ostringstream oss;
        for (size_t i = 0; i < _general_buffer.size(); ++i) {
            if (i > 0) oss << ',';
            oss << _general_buffer[i];
        }
        std::string msg = oss.str();
        sendto(udp_sock_, msg.c_str(), msg.size(), 0,
               (struct sockaddr*)&udp_addr_, sizeof(udp_addr_));
    }
}
```

**`close()` — socket cleanup:**
```cpp
if (udp_sock_ >= 0) {
    ::close(udp_sock_);
    udp_sock_ = -1;
}
```

### 1.2 `source/user/custom.cpp` — all modes now trigger reporting

Removed the `counter_rl >= 2` guard so folding/standing modes also log and broadcast:

```cpp
// BEFORE:
void G1::ReportData() {
    if (current_mode >= '1' and current_mode != 'q') {
        if (rlController->counter_rl >= 2) {
            dataReporter.report_data(rlController);
        }
    }
}

// AFTER:
void G1::ReportData() {
    if (current_mode >= '1' and current_mode != 'q') {
        dataReporter.report_data(rlController);
    }
}
```

Triggers in: folding (`'1'`), standing (`'2'`), RL (`'3'`), sin test (`'5'`).  
Does NOT trigger in: emergency stop (`'q'`).

---

## Part 2 — What Needs to Be Done in the Training Repo (`RoboTamer4Qmini`)

### Task

Create `visualizer.py` in the repo root and commit to branch `claude/study-motor-zero-calibration-7qjpF`.

No other files need to be changed.

### Data Contract

Each UDP packet is a comma-separated string of **53 floats**:

| Slice | Content | Unit | Size |
|-------|---------|------|------|
| `[0:10]` | `joint_act` — commanded joint positions | rad | 10 |
| `[10:20]` | `joint_pos` — actual joint positions | rad | 10 |
| `[20:30]` | `joint_vel` — joint velocities | rad/s | 10 |
| `[30:40]` | `joint_tau` — joint torques | N·m | 10 |
| `[40:43]` | `base_rpy` — roll, pitch, yaw | rad | 3 |
| `[43:46]` | `base_rpy_rate` — angular velocity | rad/s | 3 |
| `[46:49]` | `base_acc` — linear acceleration | m/s² | 3 |
| `[49:53]` | `base_quat` — qw, qx, qy, qz | — | 4 |

Joint index order (same for all 4 joint blocks):

| Index | Joint |
|-------|-------|
| 0 | Left Hip Yaw |
| 1 | Left Hip Roll |
| 2 | Left Hip Pitch |
| 3 | Left Knee |
| 4 | Left Ankle |
| 5 | Right Hip Yaw |
| 6 | Right Hip Roll |
| 7 | Right Hip Pitch |
| 8 | Right Knee |
| 9 | Right Ankle |

### `visualizer.py` — full code

```python
"""
Real-time robot state visualizer for Qmini.
Receives UDP broadcast from RoboTamerSdk4Qmini DataReporter and plots live.

Place this file in: RoboTamer4Qmini/visualizer.py

Usage:
    pip install matplotlib numpy
    python3 visualizer.py [--port 9870] [--history 200]

Data layout (53 floats, comma-separated per UDP packet):
    [0:10]   joint_act   commanded position  (rad)
    [10:20]  joint_pos   actual position     (rad)
    [20:30]  joint_vel   velocity            (rad/s)
    [30:40]  joint_tau   torque              (N·m)
    [40:43]  rpy         roll, pitch, yaw    (rad)
    [43:46]  rpy_rate    angular velocity    (rad/s)
    [46:49]  base_acc    ax, ay, az          (m/s²)
    [49:53]  base_quat   qw, qx, qy, qz

Joint order (indices 0-4 = left leg, 5-9 = right leg):
    hyaw, hrol, hpit, knee, apit
"""

import argparse
import socket
import threading
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ── column indices ──────────────────────────────────────────────────────────
IDX_ACT  = slice(0,  10)
IDX_POS  = slice(10, 20)
IDX_VEL  = slice(20, 30)
IDX_TAU  = slice(30, 40)
IDX_RPY  = slice(40, 43)
IDX_RATE = slice(43, 46)
IDX_ACC  = slice(46, 49)
IDX_QUAT = slice(49, 53)

JOINT_NAMES = [
    'l_hyaw', 'l_hrol', 'l_hpit', 'l_knee', 'l_apit',
    'r_hyaw', 'r_hrol', 'r_hpit', 'r_knee', 'r_apit',
]
RPY_NAMES  = ['roll', 'pitch', 'yaw']
RPY_COLORS = ['tab:red', 'tab:green', 'tab:blue']
LEFT_IDX   = list(range(0, 5))
RIGHT_IDX  = list(range(5, 10))


# ── UDP receiver (background thread) ────────────────────────────────────────
class UDPReceiver:
    def __init__(self, port: int, history: int):
        self.buf   = deque(maxlen=history)
        self.lock  = threading.Lock()
        self.port  = port
        self._stop = threading.Event()

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', self.port))
        sock.settimeout(1.0)
        print(f"Listening on UDP port {self.port} ...")
        while not self._stop.is_set():
            try:
                data, addr = sock.recvfrom(4096)
                values = np.array([float(x) for x in data.decode().split(',')],
                                  dtype=np.float32)
                if len(values) >= 53:
                    with self.lock:
                        self.buf.append(values)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"[UDP] parse error: {e}")
        sock.close()

    def get_array(self):
        """Return (N, 53) numpy array, newest last. None if empty."""
        with self.lock:
            if not self.buf:
                return None
            return np.stack(self.buf, axis=0)

    def stop(self):
        self._stop.set()


# ── Visualizer ───────────────────────────────────────────────────────────────
class Visualizer:
    def __init__(self, receiver: UDPReceiver):
        self.rx = receiver

        # 4 rows x 2 cols
        self.fig, axes = plt.subplots(4, 2, figsize=(14, 10))
        self.fig.suptitle('Qmini Real-time Monitor', fontsize=13)
        self.fig.tight_layout(rect=[0, 0, 1, 0.96])

        self.ax_pos_l, self.ax_pos_r = axes[0]   # row 0: joint position
        self.ax_vel_l, self.ax_vel_r = axes[1]   # row 1: joint velocity
        self.ax_rpy,   self.ax_rrate = axes[2]   # row 2: RPY / RPY rate
        self.ax_tau_l, self.ax_tau_r = axes[3]   # row 3: torque

        self._setup_axes()

    def _setup_axes(self):
        colors5 = plt.cm.tab10(np.linspace(0, 0.5, 5))

        def _make_lines(ax, indices, style='-'):
            return [ax.plot([], [], style, lw=1.2,
                            label=JOINT_NAMES[i], color=colors5[k])[0]
                    for k, i in enumerate(indices)]

        # joint position (solid = actual, dashed = commanded)
        self.ax_pos_l.set_title('Joint Pos — Left Leg (rad)')
        self.lines_pos_act_l = _make_lines(self.ax_pos_l, LEFT_IDX,  '--')
        self.lines_pos_l     = _make_lines(self.ax_pos_l, LEFT_IDX,  '-')
        self.ax_pos_l.legend(fontsize=7, loc='upper left')

        self.ax_pos_r.set_title('Joint Pos — Right Leg (rad)')
        self.lines_pos_act_r = _make_lines(self.ax_pos_r, RIGHT_IDX, '--')
        self.lines_pos_r     = _make_lines(self.ax_pos_r, RIGHT_IDX, '-')
        self.ax_pos_r.legend(fontsize=7, loc='upper left')

        # joint velocity
        self.ax_vel_l.set_title('Joint Vel — Left Leg (rad/s)')
        self.lines_vel_l = _make_lines(self.ax_vel_l, LEFT_IDX)
        self.ax_vel_l.legend(fontsize=7, loc='upper left')

        self.ax_vel_r.set_title('Joint Vel — Right Leg (rad/s)')
        self.lines_vel_r = _make_lines(self.ax_vel_r, RIGHT_IDX)
        self.ax_vel_r.legend(fontsize=7, loc='upper left')

        # RPY
        self.ax_rpy.set_title('Base RPY (rad)')
        self.lines_rpy = [
            self.ax_rpy.plot([], [], lw=1.5, label=n, color=c)[0]
            for n, c in zip(RPY_NAMES, RPY_COLORS)
        ]
        self.ax_rpy.legend(fontsize=8, loc='upper left')
        self.ax_rpy.axhline(0, color='k', lw=0.5, ls=':')

        # RPY rate
        self.ax_rrate.set_title('Base RPY Rate (rad/s)')
        self.lines_rrate = [
            self.ax_rrate.plot([], [], lw=1.5, label=n + '_rate', color=c)[0]
            for n, c in zip(RPY_NAMES, RPY_COLORS)
        ]
        self.ax_rrate.legend(fontsize=8, loc='upper left')
        self.ax_rrate.axhline(0, color='k', lw=0.5, ls=':')

        # torque
        self.ax_tau_l.set_title('Torque — Left Leg (N·m)')
        self.lines_tau_l = _make_lines(self.ax_tau_l, LEFT_IDX)
        self.ax_tau_l.legend(fontsize=7, loc='upper left')

        self.ax_tau_r.set_title('Torque — Right Leg (N·m)')
        self.lines_tau_r = _make_lines(self.ax_tau_r, RIGHT_IDX)
        self.ax_tau_r.legend(fontsize=7, loc='upper left')

        for ax in self.fig.axes:
            ax.grid(True, lw=0.4, alpha=0.5)

    def _update(self, _frame):
        data = self.rx.get_array()
        if data is None or len(data) < 2:
            return []

        t   = np.arange(len(data))
        act = data[:, IDX_ACT]
        pos = data[:, IDX_POS]
        vel = data[:, IDX_VEL]
        tau = data[:, IDX_TAU]
        rpy = data[:, IDX_RPY]
        rr  = data[:, IDX_RATE]

        def _set(lines, ys):
            for line, y in zip(lines, ys.T):
                line.set_data(t, y)

        _set(self.lines_pos_l,     pos[:, LEFT_IDX])
        _set(self.lines_pos_act_l, act[:, LEFT_IDX])
        _set(self.lines_pos_r,     pos[:, RIGHT_IDX])
        _set(self.lines_pos_act_r, act[:, RIGHT_IDX])
        _set(self.lines_vel_l,     vel[:, LEFT_IDX])
        _set(self.lines_vel_r,     vel[:, RIGHT_IDX])
        _set(self.lines_tau_l,     tau[:, LEFT_IDX])
        _set(self.lines_tau_r,     tau[:, RIGHT_IDX])

        for i, line in enumerate(self.lines_rpy):
            line.set_data(t, rpy[:, i])
        for i, line in enumerate(self.lines_rrate):
            line.set_data(t, rr[:, i])

        for ax in self.fig.axes:
            ax.relim()
            ax.autoscale_view()

        last = data[-1]
        self.fig.suptitle(
            f'Qmini Monitor  |  '
            f'roll={last[40]:.3f}  pitch={last[41]:.3f}  yaw={last[42]:.3f}  '
            f'[{len(data)} frames @ 20Hz]',
            fontsize=11
        )
        return []

    def run(self, interval_ms: int = 100):
        ani = animation.FuncAnimation(
            self.fig, self._update,
            interval=interval_ms, blit=False, cache_frame_data=False
        )
        plt.show()
        return ani   # keep reference to prevent GC


# ── main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description='Qmini real-time visualizer')
    parser.add_argument('--port',    type=int, default=9870,
                        help='UDP port (default: 9870)')
    parser.add_argument('--history', type=int, default=200,
                        help='Frames to display (default: 200 = 10s at 20Hz)')
    args = parser.parse_args()

    rx = UDPReceiver(port=args.port, history=args.history)
    rx.start()

    viz = Visualizer(rx)
    try:
        viz.run(interval_ms=100)
    except KeyboardInterrupt:
        pass
    finally:
        rx.stop()
        print("Stopped.")


if __name__ == '__main__':
    main()
```

### Commit instructions for the other session

```
Task: create visualizer.py in the RoboTamer4Qmini repo root with the code above.
Commit message: "Add real-time robot state visualizer (visualizer.py)"
Branch: claude/study-motor-zero-calibration-7qjpF
No other files need to be modified.
Dependencies: matplotlib, numpy (standard, no training framework needed)
```

---

## Part 3 — End-to-End Usage

### Robot machine (Pi/Jetson)

```bash
# Pull the updated SDK branch
cd ~/RoboTamerSdk4Qmini
git fetch origin
git checkout claude/study-motor-zero-calibration-7qjpF

# Rebuild
cd build
cmake -DPLATFORM=arm64 .. && make

# Run — UDP starts broadcasting as soon as mode 1/2/3 is active
cd ../bin
sudo ./run_interface
```

### Viewer machine (Training PC, same LAN)

```bash
cd ~/RoboTamer4Qmini
pip install matplotlib numpy
python3 visualizer.py --port 9870 --history 200
```

Window opens immediately. Plots update at ~10Hz as data arrives at 20Hz.

### Troubleshooting

| Problem | Fix |
|---------|-----|
| No data received | Check both machines are on same subnet; check firewall allows UDP 9870 |
| `TkAgg` error | `sudo apt install python3-tk` or change backend to `Qt5Agg` |
| Plots lag | Reduce `--history` (e.g. `--history 50`) |
| Only works in mode 3 | Make sure you pulled the latest SDK branch (custom.cpp fix) |
