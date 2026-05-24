# Pre-HAL Refactor Plan

Refactors to do **before** the HAL split (see `HAL_PLAN.md`). Goal: pay back
the HAL work, fix real bugs, and unblock testing — without repainting walls
that are about to be knocked down.

Estimated total: ~1.5 dev-days. Stop here if it stretches further.

---

## A. Must-do — blocks or complicates the HAL refactor

| # | Issue | Where | Why |
|---|---|---|---|
| A1 | **Headers contain implementations.** Full bodies live in `Motor_thread.hpp`, `IMUReader.h`, `joystick_reader.h`, `mode_switcher.h`, `data_report.h`, `onnx_inference.h`. | `include/user/*.h{,pp}` | HAL needs each backend in a `.cpp` behind an interface header. Today `#include "Motor_thread.hpp"` drags Unitree motor SDK symbols into every TU. Split is half the HAL work already. |
| A2 | **RLController owns pointers it didn't allocate.** `~RLController()` calls `delete` on `dds_motor_command`, `dds_motor_state`, `dds_base_state`, `jsreader`, `gamepad` — but these point to `G1`'s members. Never fires today because the process `exit(1)`s before unwinding. | `include/user/rl_controller.h:122-129` | HAL adds proper construction/teardown → double-free immediately. Null out or change to non-owning. |
| A3 | **`exit(1)` from inside the control thread** on `case 'q'`. | `source/user/custom.cpp:35` | A control loop you can't `return` from is one you can't unit-test or run headlessly. Replace with stop flag. Prereq for tier-4 sim-loop test. |
| A4 | **`get_observation()` has 6+ side effects** in 50 lines: computes `joint_pos_error`, calls `joystick_command_process()` (mutates `target_command` + `_record_yaw`), computes `static_flag`, updates `pm_phase_sin_cos`, pushes to obs stack, checks dim, returns stacked obs. | `source/user/rl_controller.cpp:85-137` | Golden-vector test (tier 2) needs a *pure* function `BaseStateFrame + MotorStateFrame + cmd → obs`. Extract `ObsBuilder::build(...)`. |
| A5 | **`pthread_mutex_t _rl_state_mutex` declared but never `pthread_mutex_init`-ed** — locking is UB. | `include/user/rl_controller.h:70`, used `rl_controller.cpp:91,113` | Switch to `std::mutex` + `std::lock_guard`. Unify on one primitive. |

---

## B. Should-do — real bugs you'd otherwise carry forward

| # | Issue | Where |
|---|---|---|
| B1 | **Magic `0.15` `static_flag` threshold inline** — CLAUDE.md flags this as a critical sync point with training. | `rl_controller.cpp:98` |
| B2 | **Magic constants scattered**: `0.5` pm_f init, `0.2`/`0.1` exp-filter weights, `0.3`/`1.0` freq scale, `±3` obs clamp, `0.1` joystick deadzone, action-bound index splits `ii=0/1/2` in `transform()`. | `rl_controller.cpp` various, `transform():213-225` |
| B3 | **`jointIndex2Sim << 0,1,…,9` is the identity** but used as a remap everywhere. Either load-bearing and undocumented, or dead code. | `rl_controller.cpp:10` |
| B4 | **`Py_Initialize/Py_Finalize` called twice** (IMUReader + JoystickReader). Second `Py_Finalize` is UB. | `IMUReader.h:14,19`, `joystick_reader.h:19,24` |
| B5 | **`MonitorThread()` runs once then exits** — no `while(running)` loop. | `Motor_thread.hpp:137-151` |
| B6 | **Hardcoded relative paths** — `policy.onnx`, `config.yaml`, `general.txt`, `rl.txt` open relative to CWD; must `cd bin/` to run. | `rl_controller.cpp:7`, `Motor_thread.hpp:31`, `data_report.h:122-123` |
| B7 | **Hardcoded network interface `"wlP1p1s0"`** in `main`. | `run_interface.cpp:5` |
| B8 | **Hardcoded FTDI device serial-by-id paths** for 4 specific cables. Won't survive a cable swap. | `Motor_thread.hpp:25-28` |

B1 + B2 collapse into one fix: a `constants.h` (or surfaced into `policy_meta.yaml`)
with named symbols. Then the cross-repo consistency check from `HAL_PLAN.md`
becomes a 5-line assert.

---

## C. Nice-to-have — independent hygiene (skip until later)

| # | Issue |
|---|---|
| C1 | No namespacing. `using namespace std;` and `using namespace unitree::common;` injected at file scope in headers. Wrap user code in `qmini::`. |
| C2 | Include-guard inconsistency. `#pragma once` in some, `#ifndef` in others, none in `Motor_thread.hpp`. |
| C3 | No const-correctness. Pure-getter methods aren't const. |
| C4 | No logging facility. `cout` + `printf` + ANSI escapes scattered. ~30 lines for `log::info/warn/error`. |
| C5 | CMakeLists oddities: sets `CMAKE_CXX_FLAGS` three times in a row (lines 64/70/71), hardcoded `/usr/include/python3.13/`, default build type `Debug` for real-time code, commented-out `test_interface` cruft. |
| C6 | `Crc32Core(...)` inline in vendored `motors.hpp`. Not yours, but the IDL include there couples motor types to DDS — flagged as the wedge point. |
| C7 | String-TSV log files in `data_report.h`. If tier-5 replay reads these, version them with a header line. |
| C8 | Refcount leak in `IMUReader::fetchIMUData` — `pModule` not DECREFed on `!pFunc` early-return path. |

**Skip C1–C7 until post-HAL.** Don't pay back the HAL work; just churn.

---

## Recommended order

1. **A1** (header→cpp split) — biggest, but mechanical. Rebuilds get faster.
   No behavior change. ~half a day.
2. **A2 + A3 + A5** together — ownership, exit-flag, mutex. ~2h. Prereq for testing.
3. **A4 (extract `ObsBuilder`) + B1/B2 (named constants)** — same code area.
   ~2h. Unlocks golden-vector testing + cross-repo sync check.
4. **B6/B7** — paths + interface from argv/config. ~30min. Makes desktop
   running painless even before sim backends exist.
5. **B3, B4, B5, B8, C8** — quick wins. ~1h total.
6. Then start `HAL_PLAN.md` step 1.

---

## Caveat

Refactor-without-feature has diminishing returns past ~1.5 dev-days. If A1
stretches, do it in one PR everyone else can rebase off cleanly and move on.
Don't let B/C creep.
