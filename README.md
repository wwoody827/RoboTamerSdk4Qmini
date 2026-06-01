# RoboTamerSdk4Qmini

C++ SDK that deploys an Isaac Lab–trained ONNX policy onto the Unitree
Qmini biped. Three backends share one Hardware Abstraction Layer
(HAL):

| Backend | Where it runs | Used for |
|---|---|---|
| `sim` | Any Linux | unit + integration tests, fast dev loop |
| `mujoco` | Any Linux desktop | physics-accurate visualisation with the live GLFW viewer |
| `hardware` | Qmini's onboard ARM64 (Orin) | production: real motors, real IMU, real joystick |

Above the HAL line, exactly one set of C++ code runs in all three
modes — same controller, same policy I/O, same observation builder.
That's the property that makes sim2sim and sim2real diagnostics
honest.

## Quick links

| Doc | Topic |
|---|---|
| `ARCHITECTURE.md` | Layered design, per-tick control flow, where the HAL boundary lives, how the policy plugs in |
| `BUILDING.md` | The three CMake presets and what each needs |
| `USAGE.md` | How to run `run_interface` and `pd_calibration_tool` day-to-day |
| `1_calibrate_joints.md` | Joint (`startq`) calibration runbook: bootstrap → geometric-pin (Stage 1.5) → (or jog-refine) → canonical lock |
| `2_pd_calibration.md` | PD + friction calibration runbook (sim2real): step/sine/chirp + free-release; one motor + hip_roll; what works on this hardware |
| `GEOMETRIC_JOINT_CALIBRATION_SPEC.md` | Spec: per-joint geometric `startq` calibration via physical landmarks (square / level / view). Implemented by `bin/joint_geom_cal_tool`; reference images in `docs/images/geom_cal/` |
| `PD_CALIBRATION_SPEC.md` | Spec for the per-joint PD identification protocol |
| `PYTHON_REMOVAL.md` | Why the runtime is pure C++ (and the protocol references for the hardware backends) |
| `MOTOR_PORT_MAP.md` | FTDI port → motor ID table (per-robot wiring) |
| `tools/calibration_fit/README.md` | Offline fitter (Python) that consumes calibration `.npz` and emits `calibration.yaml` |
| `scripts/hardware/README.md` | Standalone bring-up scripts (motor scan, joystick pair, IMU sanity) |

## 60-second quick start (desktop dev loop)

```bash
# Build the mujoco backend with the live viewer
sudo apt install -y libglfw3-dev libeigen3-dev libyaml-cpp-dev cmake
sim_assets/setup_mujoco.sh /path/to/your/mujoco/install
cmake --preset desktop-mujoco
cmake --build build/desktop-mujoco -j

# Run with the live viewer; press 2 (stand), 3 (walk), b (quit)
cd tests/fixtures
../../bin/run_interface --no-onnx --no-log \
    --mjcf ../../sim_assets/q1_sim_hung.mjcf --initial-mode 2
```

See `USAGE.md` for the full key map, calibration flow, and viewer
controls.

## Repository layout

```
RoboTamerSdk4Qmini/
├── README.md, ARCHITECTURE.md, BUILDING.md, USAGE.md   ← start here
├── PD_CALIBRATION_SPEC.md, PYTHON_REMOVAL.md           ← deep refs
├── MOTOR_PORT_MAP.md                                   ← wiring table
├── CLAUDE.md                                           ← for LLM agents
│
├── include/user/             headers above the HAL line
│   ├── qmini_app.h           top-level orchestrator (Options struct + FSM)
│   ├── rl_controller.h       control logic (obs build, inference, action transform)
│   ├── obs_builder.h         pure 44-dim observation
│   ├── policy.h              IPolicy interface + IdentityPolicy / OnnxPolicy factories
│   ├── data_report.h         CSV + UDP telemetry
│   ├── mode_switcher.h       joystick/keyboard → FSM mode
│   ├── hal/                  HAL types + interfaces (motor / imu / joystick / clock)
│   └── calibration/          PD calibration trial schedule + loop + NPZ writer
│
├── source/                   matching .cpp; HAL backends under hal/{sim,mujoco,hardware}/
├── sim_assets/               q1_sim.mjcf, q1_sim_hung.mjcf, meshes, build_mjcf.py
├── tools/
│   ├── probe_modes.cpp       FSM-mode tracer (dumps NPZ; not in ctest)
│   └── calibration_fit/      offline Python fitter for calibration.yaml
├── scripts/hardware/         operator scripts: motor scan, joystick pair, IMU sanity
├── tests/                    obs_builder / math / sim_loop / mujoco_loop
├── lib/                      vendored: mujoco (symlinked), unitree_sdk2, motor SDK, onnxruntime
└── bin/                      compiled binaries (run_interface, pd_calibration_tool, …)
                              + config.yaml (loaded by every binary)
```

## Cross-repo coupling

This SDK pairs with the training repo `qmini_lab` (Isaac Lab + rsl_rl).
The contract:

- **Joint name + order**: `qmini_lab/.../q1/constants.py::QMINI_JOINT_NAMES` ≡ `bin/config.yaml::startq` ≡ MJCF joint declaration order.
- **Reference standing pose**: `QMINI_REF_JOINT_POSES_BY_Z[3]` (z=0.40) → `bin/config.yaml::ref_joint_act` (verbatim).
- **PD gains**: `QMINI_STIFFNESS / QMINI_PD_DAMPING` → `bin/config.yaml::kp / kd` (verbatim).
- **Observation vector**: 44 dims, layout pinned in `include/user/obs_builder.h` ≡ `qmini_lab/.../_get_observations`.
- **Policy I/O shape**: 12 actions, `num_observations × num_stacks = 44 × 3 = 132` obs floats.

Any change on either side that breaks one of these contracts will
cause silent sim2real divergence. Calibrate before deploying a new
policy — see `PD_CALIBRATION_SPEC.md`.

## Credits

Original codebase by **Yanyun Chen, Tiyu Fang, Kaiwen Li, Kunqi Zhang,
and Wei Zhang** — Visual Sensing and Intelligent System Lab (VSISLab),
Shandong University. <https://www.vsislab.com>

HAL refactor (May 2026) added the pure-C++ HAL, MuJoCo backend + live
viewer, PD calibration tool, and the obs/policy sync to `qmini_lab`.

## License

MIT — see `LICENSE`.
