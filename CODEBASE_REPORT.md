# CODEBASE_REPORT.md — superseded

This file used to be a snapshot of the pre-HAL codebase. The May 2026
refactor invalidated most of it (threads, file layout, class names,
embedded-Python usage all changed).

For the current design, see:

- **[ARCHITECTURE.md](ARCHITECTURE.md)** — layered view, per-tick control
  flow, backend semantics, tests.
- **[BUILDING.md](BUILDING.md)** — the three presets and their setup.
- **[CLAUDE.md](CLAUDE.md)** — file index, training-sync contract.
- **[PYTHON_REMOVAL.md](PYTHON_REMOVAL.md)** — why CPython was dropped
  from the runtime, with the jsX + VSISLab IMU protocol references.

For history of how we got here:

- **[HAL_PLAN.md](HAL_PLAN.md)** — original architecture proposal +
  status table.
- **[REFACTOR_PLAN.md](REFACTOR_PLAN.md)** — pre-HAL cleanups + status
  table.
- **[SIDEWALK_DESIGN.md](SIDEWALK_DESIGN.md)** — April 2026 `cmd_vy`
  addition.
- **[MOTOR_PORT_MAP.md](MOTOR_PORT_MAP.md)** — FTDI port → motor ID
  table (still accurate; hardware didn't change).
