# behav3d_py Python modules

This is the Python API surface for the ROS 2 `behav3d_py` package. The focus is on a FIFO command queue (`Commands`), a YAML runner (`SequenceParser`), and higher-level capture macros (`Macros`).

## Package exports
- `__init__.py`: exports `Commands`, `SequenceParser`, and `__version__`.

## Core API

### `commands.py`
`Commands` is a single-queue orchestrator that wraps motion planning, controller execution, printing, sensing, and reconstruction into a FIFO.

Public methods:
- `home(duration_s=10.0, on_move_done=None)`: enqueue a joint trajectory to the hard-coded HOME.
- `goto(x, y, z, rx=None, ry=None, rz=None, eef=None, vel_scale=None, accel_scale=None, exec=True, on_move_done=None, start_print=None, motion=None)`: plan and optionally execute a PTP/LIN move.
- `printTime(secs, speed, use_previous_speed=False, on_done=None)`: enqueue the `PrintTime` action.
- `printSteps(steps, speed, use_previous_speed=False, on_done=None)`: enqueue the `PrintSteps` action.
- `setPTP()`, `setLIN()`: set default motion mode.
- `setEef(name)`, `setSpd(val)`, `setAcc(val)`: update defaults used by `goto()`.
- `wait(secs, on_done=None)`: delay in the FIFO.
- `getPose(eef, base_frame="world", use_tf=False, on_done=None)`: query link pose (MoveIt or TF).
- `capture(rgb=False, depth=False, ir=False, pose=False, folder=None, on_done=None)`: call `/capture` service.
- `reconstruct(use_latest=True, session_path="", on_done=None)`: call `/reconstruct_mesh` service.
- `input(key=None, prompt=None, on_done=None)`: wait for keyboard input in the FIFO.
- `pause()`, `resume()`, `toggle_pause()`: pause/resume queue processing.

Queue/executor internals (useful for debugging):
- `_enqueue(...)`, `_prepend(...)`, `_process_next()`: FIFO core.
- `_do_follow_traj(...)`: send `FollowJointTrajectory`.
- `_do_plan_motion(...)`: call Pilz PTP/LIN services and optionally execute.
- `_do_print_time(...)`, `_do_print_steps(...)`: action clients.
- `_do_wait(...)`, `_do_get_pose(...)`, `_do_capture(...)`, `_do_reconstruct(...)`, `_do_wait_input(...)`.
- `_finish_move(...)`: common completion and queue advance.
- `_quat_from_euler(...)`: RPY (rad) to quaternion helper.

### `yaml_parser.py`
`SequenceParser` maps YAML items to `Commands` calls.
- `run_file(path)`: parse a YAML list and enqueue commands.
- `_ensure_dict(val)`: validate mapping items.
- `_parse_pose(val)`: convert `pose` or `getPose` YAML entries into `Commands.getPose(...)` arguments.
- `_handle_input(val)`: parse input gating (`input` command).
- `_handle_capture(val)`: parse capture flags and folder rules.
- `_on_pose_log(res)`: default pose logger callback.

### `macros.py`
`Macros` composes higher-level routines on top of `Commands`.
- `fibScan(target, distance, cap_rad, samples, prompt=None, folder=None, settle_s=0.2, debug=False, z_jitter=0.0)`: enqueue a Fibonacci spherical-cap scan with capture at each viewpoint.

Helper functions:
- `_fibonacci_cap_dirs_np(cap_rad, n)`: vectorized directions on a spherical cap.
- `_any_orthonormal(v)`: return an arbitrary orthonormal vector.

## Demo nodes

### `handeye_capture_sequence.py`
`MoveAndPrintTest` drives a fib-scan capture sequence.
- `_run_once()`: builds the scan sequence.
- `_on_move_done(res)`: unified move callback.
- `_on_pose(res)`: pose print helper.
- `_on_quit(res)`: shutdown handler.

### `move_and_print_test.py`
`MoveAndPrintTest` demo for motion + printing.
- `_run_once()`: enqueues sample moves and print actions.
- `_on_move_done(res)`, `_on_pose(res)`, `_on_quit(res)`.

### `run_yaml_test.py`
`RunYamlTest` loads and executes a YAML command file.
- `RunYamlTest.__init__(yaml_path)` creates `Commands` and `SequenceParser`.
- CLI: `--path /path/to/file.yaml`.

### `print_test.py`
Interactive print action tester.
- `KeyReader`: puts the terminal into cbreak mode and reads single keys.
- `OrchestratorNode`: sends a `PrintTime` goal and updates speed on key presses.
  - Action helpers: `_send_goal(...)`, `_on_goal_response(...)`, `_cancel_goal(...)`, `_cancel_goal_and_wait(...)`.
  - Service helpers: `_get_status(...)`, `_set_speed(...)`.
  - UI: `_on_key(...)`, `_on_feedback(...)`, `_on_result(...)`, `_shutdown()`.

### `modbus_test.py`
Minimal Modbus test node.
- `ModbusTest`: connects to Modbus TCP and exposes params for coil + speed register.
- `_on_params(...)`, `_poll()`, `_write_coil(...)`, `_write_speed(...)`.

## Actionable improvements
- Split `Commands` into transport adapters (MoveIt, print, capture) plus a FIFO core.
- Replace magic strings with enums/constants (motion modes, service names, frames).
- Add type hints for callbacks and payloads, and run `mypy` on the package.
- Move user-facing examples out of the module namespace (to `examples/` or `demos/`).
- Add structured result objects (dataclasses) for FIFO responses instead of dicts.
- Add retry/backoff around ROS service/action calls to handle transient failures.
- Enforce timeouts per queue item and propagate them into callbacks.
