# behav3d_commands architecture

## Overview
`behav3d_commands` provides the low-level command layer and a queue-driven
orchestration surface for Behav3D systems. The main entry point is `Session`,
which owns a `SessionQueue`, a `CommandRouter`, and subsystem command sets
(`MotionCommands`, `CameraCommands`, `FieldCommands`, `ExtruderCommands`,
`UtilCommands`).

Higher-level orchestration lives in `behav3d_orchestrator`.

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

The local `Session.pause()`/`resume()` methods only control that instance's
queue. Orchestrator sessions derive from `ControlAwareSession`, which listens
to the shared `/behav3d/control_state` topic and waits before starting the next
internal command.

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
        candidate_mode="gradient_lift",            # z_lift | gradient_lift | gradient_walk
        beads_per_step=7,
        bead_separation_mm=16.0,
        bead_height_mm=12.0,
        walk_distance_mm=12.0,                     # gradient_walk only
        walk_step_mm=1.0,
        walk_max_steps=32,
        walk_start_fraction=0.25,
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
  `candidate_mode="z_lift"`, `candidate_mode="gradient_lift"`, or
  `candidate_mode="gradient_walk"`.
- `gradient_walk` uses the walk parameters above and writes nested
  `segments: [{start, end}]` YAML for line printing. The flat `targets:` YAML
  shape is still used by `z_lift` and `gradient_lift`. Segment endpoints are
  tangent-oriented even if `orient_with_tangent` is left false.
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

### MotionCommands Pilz sequence and TCP retime
`MotionCommands` also exposes Pilz sequence planning for polyline-style paths:
- `session.motion.plan_sequence(...)` plans and stores a sequence trajectory.
- `session.motion.goto_sequence(...)` plans and optionally executes it.

These commands call `/behav3d/plan_pilz_sequence`, which uses MoveIt's Pilz
Industrial Motion Planner (`sequence_move_group`) to generate the LIN sequence.
Optional TCP-speed retiming happens after Pilz planning, without changing the
Pilz path geometry and without calling IK again.

Minimal blocking example:
```python
res = session.run_sync(
    session.motion.plan_sequence(
        poses,
        eef="extruder_tcp",
        vel_scale=0.05,
        accel_scale=0.05,
        blend_radius=0.003,
        frame_id="world",
        target_tcp_speed_m_s=0.080,
        retime_min_dt_s=0.008,
        tcp_sample_spacing_m=0.002,
        tcp_speed_threshold_m_s=0.064,
        enqueue=False,
    ),
    timeout_s=30.0,
)
if res["ok"]:
    session.run_sync(session.motion.exec(enqueue=False), timeout_s=30.0)
```

Key parameters:
- `blend_radius`: Pilz blend radius in meters. The last sequence item is forced
  to `0.0` by the bridge.
- `target_tcp_speed_m_s`: requested TCP speed for constant-speed retime. `0.0`
  disables TCP retime.
- `retime_min_dt_s`: minimum controller timestamp spacing; this is a technical
  guard, not the main smoothing control.
- `tcp_sample_spacing_m`: TCP resampling spacing before retime. Default command
  value is `0.002` m (`2 mm`). The bridge uses
  `max(tcp_sample_spacing_m, target_tcp_speed_m_s * retime_min_dt_s)` as the
  effective spacing so timestamps do not become too dense.
- `tcp_speed_threshold_m_s`: diagnostic threshold only. It counts samples below
  the threshold in metrics; it does not reject or alter the plan.

Returned metrics include:
- `duration_s`, `points`, `interior_zero_velocity_points`
- `tcp_path_length_m`, `tcp_duration_s`
- `tcp_speed_min_mm_s`, `tcp_speed_mean_mm_s`, `tcp_speed_max_mm_s`
- `tcp_speed_low_sample_count`, `tcp_speed_sample_count`
- `tcp_speed_retimed`, `tcp_speed_retime_fallback`
- `tcp_sample_spacing_mm`

Notes:
- Pilz still owns planning. The retime step only rewrites timing and resamples
  the already planned Pilz trajectory by interpolating robot states along the
  Pilz path.
- If TCP retime fails and `retime_fallback_on_failure=true` in
  `behav3d_motion_bridge`, the bridge returns the original Pilz plan.
- The bridge has an internal `max_tcp_speed_m_s` parameter, default `0.350`
  (`350 mm/s`). Requests above that are clamped, not rejected.
- For moving between unrelated polylines, use a separate normal LIN/PTP move to
  the next start target. Do not include long transition moves inside the print
  sequence if constant TCP speed matters.

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

## Related packages
`behav3d_utils` contains geometry helpers (Python port of `target_builder.cpp`).
Import from `behav3d_utils.target_builder` when you need pose/transform helpers.
