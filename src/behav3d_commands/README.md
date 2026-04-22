# behav3d_commands architecture

## Overview
`behav3d_commands` provides the low-level command layer and a queue-driven
orchestration surface for Behav3D systems. The main entry point is `Session`,
which owns a `SessionQueue`, a `CommandRouter`, and subsystem command sets
(`MotionCommands`, `CameraCommands`, `FieldCommands`, `ExtruderCommands`,
`UtilCommands`).

Higher-level orchestration and example sessions live in `behav3d_examples`.

## Core building blocks

### Session
`Session` wires the command sets to a shared queue and routes queue items to
their handlers. It exposes:
- `run_group(items)` to schedule a parallel group.
- `run_sync(item, timeout_s=None)` to block until a single item's `on_done` fires.
- `pause()` / `resume()` and `is_busy` / `is_paused`.
- `queue` for direct access to the `SessionQueue` when needed.

`run_sync` must not be called from the ROS executor thread; use a worker thread
or a multi-threaded executor.

### SessionQueue
`SessionQueue` is a FIFO queue with optional parallel groups:
- `enqueue(item)` adds one item.
- `prepend(item)` pushes one item to the head.
- `enqueue_group([...])` starts a set of items at the same time.

Groups are best-effort concurrency. There is no resource locking yet, so avoid
conflicting actions in the same group unless you know they are safe.

### QueueItem and Command
`QueueItem` is the unit of work:
```
QueueItem(kind, payload, cmd_kind=None, on_done=None)
```

- `kind` maps to a handler registered in `CommandRouter`.
- `cmd_kind` is the user-facing label in callbacks (defaults to `kind`).
- `on_done` receives a result dict when the command finishes.

`Command.finish_flag(...)` enforces the result contract and guarantees the queue
advance even if the callback raises.

## Command sets
Each `*Commands` class:
- Builds a `QueueItem` for its public methods.
- Enqueues by default, or returns a `QueueItem` when `enqueue=False`.
- Registers internal handlers with `CommandRouter`.

Example (build items, run in a group):
```python
session = Session(node)
move = session.motion.goto(x=0.5, y=1.0, z=0.3, enqueue=False)
print_item = session.extruder.print_steps(steps=2000, speed=500, enqueue=False)
session.run_group([move, print_item])
```

Example (blocking call):
```python
plan_item = session.motion.plan(x=0.5, y=1.0, z=0.3, enqueue=False)
result = session.run_sync(plan_item, timeout_s=10.0)
```

### FieldCommands
`FieldCommands` wraps scalar-field services exposed by `fields_node`:
- `/behav3d/init_field_from_scan`
- `/behav3d/generate_print_candidates`

Public methods:
- `session.field.init_field_from_scan(...)`
- `session.field.generate_print_candidates(...)`

Minimal blocking example:
```python
session = Session(node)

init_res = session.run_sync(
    session.field.init_field_from_scan(
        use_latest=False,
        session_path="@session",
        scan_mesh_paths=["@session/field_loop/cycle_0000/scan/reconstruct/tsdf_surface_mesh.stl"],
        field_mesh_path="/home/lab/behav3d_ws/mesh/curved_wall_5mm.obj",
        state_output_dir="@session/field_loop/cycle_0000/field_init",
        enqueue=False,
    ),
    timeout_s=120.0,
)

cand_res = session.run_sync(
    session.field.generate_print_candidates(
        use_latest=False,
        session_path="@session",
        field_state_path=init_res["metrics"]["field_state_path"],
        scan_mesh_paths=["@session/field_loop/cycle_0000/scan/reconstruct/tsdf_surface_mesh.stl"],
        output_dir="@session/field_loop/cycle_0000/candidates",
        candidate_mode="gradient_lift",            # z_lift | gradient_lift
        beads_per_step=7,
        bead_separation_mm=16.0,
        bead_height_mm=12.0,
        orient_with_tangent=True,                  # tangent-based frame orientation
        tangent_sign=1.0,                          # +1 or -1 gradient direction
        clamp_to_cone=True,                        # optional orientation clamp around +Z
        cone_max_tilt_deg=40.0,
        base_to_world_yaw_deg=180.0,               # base->world yaw for YAML output
        target_zx=0.03,
        target_zy=-0.01,
        target_zz=1.0,
        target_position_scale=1000.0,
        enqueue=False,
    ),
    timeout_s=120.0,
)
```

Returned metrics include:
- Init: `field_state_path`, `debug_field_ply_path`, `offset_x/y/z`
- Candidates: `targets_yaml_path`, `debug_field_ply_path`,
  `debug_contour_ply_path`, `debug_candidates_ply_path`, `candidate_count`

Notes:
- If `session_path` is provided, `use_latest` is forced to `False`.
- Candidate generation mode is selected in the service request:
  `candidate_mode="z_lift"` or `candidate_mode="gradient_lift"`.
- YAML target frame reorientation (`base_link -> world`) is handled in
  `fields_node` during target generation (`base_to_world_yaw_deg`).

### CameraCommands World Mesh Preview
For field-debug visualization + optional scan restore in one command:
```python
preview_res = session.run_sync(
    session.camera.preview_field_ply(
        use_latest=False,
        session_path="@session",
        field_ply_path="/tmp/field_masked_cycle.ply",
        restore_mesh_path="/tmp/tsdf_surface_mesh.stl",  # optional
        wait_timeout_s=45.0,
        enqueue=False,
    ),
    timeout_s=55.0,
)
```

## Result contract
`on_done` receives a dict with:
- `ok` (bool), `kind` (str), `phase` (str), `error` (str or None)
- `metrics` (dict) and optional extras for command-specific data

## Extending the system
To add a new subsystem:
1. Create a new `*Commands` class.
2. Implement handlers (`_handle_*`) that accept `(payload, Command)`.
3. Add public methods that return `QueueItem` (and optionally enqueue).
4. Register handlers in `register(...)`, and wire the class in `Session`.

## Examples
`ScanSession` and `PrintSession` are example subclasses in `behav3d_examples`.
Example nodes that use them live in `behav3d_examples`:
- `move_and_print_test`
- `handeye_capture_sequence`

## Related packages
`behav3d_utils` contains geometry helpers (Python port of `target_builder.cpp`).
Import from `behav3d_utils.target_builder` when you need pose/transform helpers.
