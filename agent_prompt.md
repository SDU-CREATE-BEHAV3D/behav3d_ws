# Behav3D Command Surface and Orchestration Guide (AI Agent)

**Purpose**  
This document describes the Behav3D command surface, how commands map to ROS 2 interfaces, and how orchestration is done in the `custom_sequence.py` + `custom_session.py` pattern. It is intended for AI agents that must reason about system dynamics, available commands, and correct usage constraints.

**Context**  
Behav3D is intended to coordinate a reactive 3D printing loop that alternates scanning and printing with a clay extruder. The longer-term goal is to develop autonomous printing behaviors that close the loop between perception and extrusion.

**Scope**  
The primary command surface is `behav3d_commands` and its `Session` API. This guide also lists the underlying ROS 2 services/actions that back those commands and highlights the exact runtime topology expected by `print_move.launch.py`.

---

**System Entry Points**

Launch file: `src/behav3d_bringup/launch/print_move.launch.py`

The launch file brings up these runtime roles:
- UR driver and controllers (UR20 workcell).
- MoveIt `move_group`.
- `behav3d_motion_bridge` (planning + pose services).
- Orbbec camera driver (Femto Bolt).
- `behav3d_print` (extrusion actions + print services).
- `behav3d_sense` (capture service and session storage).
- Reconstruction services via `behav3d_sense/launch/reconstruct_services.launch.py`:
  - `color_to_depth_service`
  - `tsdf_cropped_service`
  - `tsdf_object_extract_service`
- `behav3d_world` (`behav3d_sense/world_node.py`) for:
  - world-state publishing (`/behav3d/world_state`, `/behav3d/get_world_state`)
  - mesh update service (`/behav3d/update_world_mesh`)
  - RViz mesh marker publication (`/visualization_marker`)

---

**Command Architecture (behav3d_commands)**

Primary entry point: `behav3d_commands.Session` in `src/behav3d_commands/behav3d_commands/session.py`.

Key concepts:
- `Session` owns a `SessionQueue`, a `CommandRouter`, and four command sets: motion, camera, extruder, util.
- Each command returns a `QueueItem` when `enqueue=False` and enqueues by default when `enqueue=True`.
- `SessionQueue` supports FIFO execution and best-effort parallel groups (`run_group`).
- `run_sync` blocks the caller until `on_done` fires. It must not be called from the ROS executor thread; use a worker thread or a multi-threaded executor.
- `Command.finish_flag(...)` finalizes results and guarantees the queue advances even if user callbacks raise.

Result contract (dict passed to `on_done`):
- `ok` (bool)
- `kind` (str)
- `phase` (str, typically `plan` or `exec`)
- `error` (str or None)
- `metrics` (dict)
- `extra` (dict, optional, command-specific)

---

**Command Surface: Motion**

Implementation: `src/behav3d_commands/behav3d_commands/motion_commands/motion_commands.py`

Underlying ROS interfaces:
- Services: `/behav3d/plan_pilz_ptp`, `/behav3d/plan_pilz_lin` (`PlanPilzPtp`, `PlanPilzLin`)
- Action: `/scaled_joint_trajectory_controller/follow_joint_trajectory` (`FollowJointTrajectory`)

Default settings:
- `motion_mode = "PTP"`
- `default_eef = "extruder_tcp"`
- `default_vel_scale = 0.1`
- `default_accel_scale = 0.1`
- Planning group is fixed to `ur_arm` in the request.

Motion commands and parameters:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `home()` | Send a fixed home joint trajectory. | `duration_s` | Uses the controller action directly. |
| `plan()` | Plan a motion and store trajectory. | `pose` or `x,y,z`, `eef`, `vel_scale`, `accel_scale`, `motion` | Uses Pilz PTP/LIN service; `preview_only=True`. |
| `exec()` | Execute last planned trajectory. | none | Fails if no plan stored. |
| `goto()` | Plan and optionally execute in one step. | `pose` or `x,y,z`, `rx,ry,rz`, `eef`, `vel_scale`, `accel_scale`, `exec`, `motion` | Default `exec=True`. |
| `setPTP()` | Set default mode to PTP. | none | Updates `motion_mode`. |
| `setLIN()` | Set default mode to LIN. | none | Updates `motion_mode`. |
| `setEef(name)` | Set default end-effector link. | `name` | Used for subsequent plans. |
| `setSpd(val)` | Set default velocity scale. | `val` (0..1) | Clamped to [0,1]. |
| `setAcc(val)` | Set default acceleration scale. | `val` (0..1) | Clamped to [0,1]. |

Pose handling rules:
- If `pose` is a `PoseStamped`, it is used directly.
- Otherwise `x,y,z` must be provided; `frame_id` defaults to `"world"`.
- If any of `rx,ry,rz` is provided, they are treated as RPY (rad) and converted to a quaternion; otherwise orientation is identity.

---

**Command Surface: Extruder**

Implementation: `src/behav3d_commands/behav3d_commands/print_commands/extruder_commands.py`

Underlying ROS interfaces:
- Action `/print` (`PrintTime`)
- Action `/print_steps` (`PrintSteps`)

Extruder commands:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `print_time()` | Extrude for a time duration. | `secs`, `speed`, `offset_s`, `use_previous_speed` | If `offset_s>0`, delays start using a timer. |
| `print_steps()` | Extrude a fixed number of steps. | `steps`, `speed`, `offset_s`, `use_previous_speed` | If `offset_s>0`, delays start using a timer. |
| `setExtruder()` | Turn extrusion ON/OFF indefinitely (optionally set speed). | `on`, `speed` (optional) | Uses `update_print_config` service; if `speed` omitted, keeps current speed. |

The actions are implemented by `behav3d_print`:
- `PrintTime` uses a timed loop and controls Modbus registers/coils.
- `PrintSteps` queues steps via Modbus and waits for completion.

Additional print services (not wrapped by `behav3d_commands`):
- `UpdatePrintConfig` at `update_print_config`
- `GetPrintStatus` at `get_print_status`

Implementation: `src/behav3d_print/behav3d_print/print_node.py`

---

**Command Surface: Camera / Sense**

Implementation: `src/behav3d_commands/behav3d_commands/sense_commands/camera_commands.py`

Underlying ROS interfaces:
- Service `/capture` (`Capture`)
- Service `/behav3d/get_link_pose` (`GetLinkPose`)
- Service `/reconstruct_mesh` (`ReconstructMesh`)
- Service `/reconstruct/color_to_depth` (`ColorToDepth`)
- Service `/reconstruct/tsdf_cropped` (`TsdfCropped`)
- Service `/reconstruct/tsdf_object_extract` (`TsdfObjectExtract`)
- Service `/behav3d/update_world_mesh` (`UpdateWorldMesh`)

Camera commands:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `capture()` | Capture RGB/Depth/IR (+ optional pose). | `rgb`, `depth`, `ir`, `pose`, `folder` | If `folder` is provided, `set_folder=True`. |
| `get_pose()` | Get link pose in a base frame. | `eef`, `base_frame`, `use_tf` | If `use_tf=True`, bypasses MoveIt and reads TF. |
| `reconstruct()` | Run TSDF reconstruction. | `use_latest`, `session_path` | Starts a background reconstruction job. |
| `reconstruct_color_to_depth()` | Run color-to-depth alignment stage. | `use_latest`, `session_path`, `scan_folder`, `visualize` | Calls `/reconstruct/color_to_depth`. |
| `reconstruct_color_to_depth_grid_sweep()` | Color-to-depth for grid sweep captures. | `use_latest`, `session_path`, `scan_folder`, `visualize` | Default `scan_folder="grid_sweep"`. |
| `reconstruct_tsdf_cropped()` | Run TSDF cropped stage. | `use_latest`, `session_path`, `scan_folder`, `visualize`, `device` | Calls `/reconstruct/tsdf_cropped`. |
| `reconstruct_tsdf_grid_sweep()` | TSDF cropped for grid sweep captures. | `use_latest`, `session_path`, `scan_folder`, `visualize`, `device` | Default `scan_folder="grid_sweep"`. |
| `update_world_mesh()` | Update RViz mesh marker from explicit or inferred reconstruction outputs. | `use_latest`, `session_path`, `mesh_path`, `ply_path`, `prefer`, `wait_timeout_s` | Calls `/behav3d/update_world_mesh`. Prefer explicit `mesh_path`/`ply_path` from TSDF response. |

Reconstruction command usage notes:
- Stage order for mesh flow: `reconstruct_color_to_depth*` first, then `reconstruct_tsdf_*`.
- If `session_path` is non-empty, command layer forces `use_latest=False` (explicit path wins).
- If `session_path` is empty, `use_latest=True` resolves the latest session under captures root.
- Reconstruction services return quickly and continue processing in a background thread; watch node logs for completion.
- Outputs are scoped by scan folder under the capture folder:
  - `@session/<scan_folder>/reconstruct/color_in_depth/`
  - `@session/<scan_folder>/reconstruct/tsdf_surface_mesh.stl`
  - `@session/<scan_folder>/reconstruct/tsdf_surface_rgb_colored.ply`
- Current protocol stores only `color_in_depth_*.png` for alignment (heavy debug files are not saved by default).
- No reconstruction history snapshot duplication is used; outputs are kept in the active scan-folder reconstruction path.
- For `update_world_mesh`, if explicit `mesh_path`/`ply_path` is provided, use those; do not rely on fallback discovery unless necessary.
- `tsdf_object_extract` is currently exposed as a ROS service only (`/reconstruct/tsdf_object_extract`); it is launched in bringup but not wrapped as a `CameraCommands` method yet.

Reconstruction + world-mesh protocol (current):
1. Choose a scan folder per cycle (recommended incremental naming: `grid_sweep_00`, `grid_sweep_01`, ...).
2. Capture into that folder (`@session/<scan_folder>`), so each cycle is isolated.
3. Run `reconstruct_color_to_depth*` with the same `scan_folder` and wait for fresh `color_in_depth` outputs.
4. Run `reconstruct_tsdf_*` with the same `scan_folder` and read returned `mesh_path` and `rgb_ply_path`.
5. Call `update_world_mesh` using explicit `mesh_path`/`ply_path` and `prefer` (`mesh` or `ply`).
6. World node stages a timestamped mesh copy in `/tmp/behav3d_world_mesh_cache` before publish, which avoids RViz stale-resource caching on repeated updates.

Capture folder semantics (as implemented in `behav3d_sense`):
- `""` or `"."` uses current capture directory.
- `"@session"` or `"@"` uses the session root.
- `"@session/subdir"` or `"@/subdir"` writes under the session root.
- `"./subdir"` is relative to current capture directory.
- `".."` or `"../subdir"` is clamped within the session root.
- Any other relative path is resolved under the captures root.

Capture session root:
- Default is `~/behav3d_ws/captures`.
- Override with `BEHAV3D_CAPTURES_ROOT`.

Implementation: `src/behav3d_sense/behav3d_sense/sense_node.py`, `src/behav3d_sense/behav3d_sense/reconstruction.py`, and `src/behav3d_sense/behav3d_sense/reconstruct/reconstruct_services.py`

---

**Command Surface: Utility**

Implementation: `src/behav3d_commands/behav3d_commands/util_commands/util_commands.py`

Utility commands:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `wait()` | Delay execution. | `secs` | Uses a ROS timer. |
| `wait_until()` | Poll a predicate until it becomes true. | `predicate`, `period_s`, `timeout_s` | Fails on timeout or predicate exception. |
| `input()` | Wait for user input. | `key`, `prompt` | If no TTY, auto-continues. |
| `publish_targets()` | Publish RViz target markers (axes). | `targets`, `axis_length`, `axis_radius`, `clear_before` | Calls `/behav3d/publish_targets`. |
| `delete_markers()` | Delete all published target markers. | none | Calls `/behav3d/delete_markers`. |

**RViz note:** add a `MarkerArray` display subscribing to `/behav3d/markers/targets` and set the Fixed Frame to match the target `frame_id` (typically `world`).

---

**Underlying ROS Interfaces (Summary)**

Actions in `src/behav3d_interfaces/action`:
- `PrintTime.action` at `/print` (provider: `behav3d_print`)
- `PrintSteps.action` at `/print_steps` (provider: `behav3d_print`)
- `PlanAndExecute.action` at `/behav3d/plan_and_execute` (provider: `behav3d_orchestrator`)

Services in `src/behav3d_interfaces/srv` (key ones used by commands):
- `/behav3d/plan_pilz_ptp` (`PlanPilzPtp`) via `behav3d_motion_bridge`
- `/behav3d/plan_pilz_lin` (`PlanPilzLin`) via `behav3d_motion_bridge`
- `/behav3d/get_link_pose` (`GetLinkPose`) via `behav3d_motion_bridge`
- `/capture` (`Capture`) via `behav3d_sense`
- `/reconstruct_mesh` (`ReconstructMesh`) via `behav3d_sense/reconstruction`
- `/reconstruct/color_to_depth` (`ColorToDepth`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/reconstruct/tsdf_cropped` (`TsdfCropped`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/reconstruct/tsdf_object_extract` (`TsdfObjectExtract`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/behav3d/update_world_mesh` (`UpdateWorldMesh`) via `behav3d_sense/world_node.py`
- `update_print_config` (`UpdatePrintConfig`) via `behav3d_print`
- `get_print_status` (`GetPrintStatus`) via `behav3d_print`
- `/behav3d/publish_targets` (`PublishTargets`) via `behav3d_motion_bridge`
- `/behav3d/delete_markers` (`DeleteMarkers`) via `behav3d_motion_bridge`

Implementation references:
- `src/behav3d_motion_bridge/src/motion_bridge_node.cpp`
- `src/behav3d_orchestrator/behav3d_orchestrator/orchestrator_node.py`
- `src/behav3d_print/behav3d_print/print_node.py`
- `src/behav3d_sense/behav3d_sense/sense_node.py`
- `src/behav3d_sense/behav3d_sense/reconstruction.py`

---

**Ideal Use Case: custom_sequence + custom_session**

The reference pattern is:
- `custom_sequence.py` starts a node, creates a `MySession`, and runs a worker thread that calls `run_sync` and `run_group`.
- `custom_session.py` defines `MySession`, which provides high-level scan and print workflows by composing session commands.

Files:
- `src/behav3d_examples/behav3d_examples/custom_sequence.py`
- `src/behav3d_examples/behav3d_examples/src/custom_session.py`

Execution flow in `custom_sequence.py`:
1. Start ROS and spin a node (`CustomSequenceDemo`).
2. Create `MySession` instance in the node.
3. Run a worker thread (not the ROS executor thread).
4. Run a reconstruction cycle with incremented capture folder naming (`grid_sweep_00`, `grid_sweep_01`, ...):
   - grid sweep capture to `@session/<scan_folder>`
   - `color_to_depth` with `scan_folder=<scan_folder>`
   - `tsdf_cropped` with `scan_folder=<scan_folder>`
   - `update_world_mesh` using returned TSDF output paths
5. Prompt operator: type `c` to capture another cycle, otherwise shutdown.

Orchestration details in `custom_session.py`:
- `run_scan_session`:
  - `home()` then set defaults `setSpd`, `setAcc`, `setEef`, `setLIN`.
  - For each target: `plan()`, then `exec()`, then `capture(...)`.
  - Uses `run_sync` to enforce strict ordering.
- `run_disc_print_session`:
  - `home()` first.
  - For each target: `plan()` then either:
    - First target: `exec()` only.
    - Subsequent targets: `run_group([exec(), print_steps(...)])` for motion + extrusion in parallel.
  - Final `wait()` to provide a synchronization barrier.

Why this pattern is ideal:
- `run_sync` enforces deterministic sequencing.
- `run_group` enables safe, explicit concurrency (move + print).
- The worker thread avoids blocking the ROS executor.

---

**Additional Orchestration Patterns (Current)**

YAML-driven orchestration in `behav3d_orchestrator`:
- Entry points:
  - `ros2 run behav3d_orchestrator yaml_target_sequence`
  - `ros2 run behav3d_orchestrator print_path_sequence`
- Session implementation: `src/behav3d_orchestrator/behav3d_orchestrator/src/yaml_session.py`
- `parse_yaml_targets(...)` supports:
  - `{index: N, xyz: [x, y, z]}`
  - `{index: N, x: ..., y: ..., z: ...}`
  - `{index: N, plane: "O(x,y,z) Z(i,j,k)"}` where `O` is in mm and `Z` is the orientation normal
  - `[x, y, z]`
- Ordering rule: targets are sorted by `index` (fallback: file position).
- `run_yaml_target_sequence(...)` behavior:
  - home, short settle wait, set `setSpd/setAcc/setLIN`, optional marker publishing, optional operator gate (`util.input`) per target.
  - First target uses a safety approach (+0.40 m in Z) before final move.
- `run_yaml_print_path(...)` behavior:
  - Parse ordered targets, approach above first target, go to first target, turn extruder ON, plan+exec LIN through all remaining targets, turn extruder OFF, clear markers.

Fib-cap capture flow in `behav3d_examples`:
- Main session class: `FibCapSession` in `src/behav3d_examples/behav3d_examples/src/fib_cap_session.py`
- Backward-compatibility alias still exported: `ScanSession = FibCapSession`
- Sequence entry points:
  - `ros2 run behav3d_examples handeye_capture_sequence`
  - `ros2 run behav3d_examples fib_cap_sequence` (alias to same node)
- `fib_scan(...)` handles Fibonacci-cap viewpoint generation, ordered motion/capture chaining, optional debug gating, and folder-scoped captures.

---

**Guidance for AI Agents**

Do:
- Use `enqueue=False` when building items for `run_group`.
- Use `run_sync` only from a worker thread.
- Check service/action availability (commands already do this and return an error in `on_done`).
- Prefer `plan()` + `exec()` for explicit control; use `goto(exec=True)` for simpler flows.
- Use `capture(folder=...)` for session organization; use `"@session/..."` when you want consistent session roots.
- Prefer shared geometry/orientation helpers from `behav3d_utils` instead of local duplicates.
- For plane-normal to pose conversion, use `behav3d_utils.target_transforms` (`pose_from_xyz_and_z_axis`, `quat_from_z_axis`, `quat_from_rotmat`).

Avoid:
- Calling `run_sync` directly inside ROS callbacks.
- Scheduling conflicting hardware actions in the same `run_group`.
- Assuming resource locks exist; `SessionQueue` does not enforce them.
- Re-implementing pose/quaternion helper logic inside sessions when equivalent utilities already exist in `behav3d_utils`.

---

**Minimal Example (Pseudo-Flow)**

```python
session = MySession(node)

# Scan
session.run_sync(session.motion.home(enqueue=False))
session.run_sync(session.motion.setLIN(enqueue=False))
session.run_sync(session.motion.plan(x=0.2, y=0.8, z=0.31, enqueue=False))
session.run_sync(session.motion.exec(enqueue=False))
session.run_sync(session.camera.capture(rgb=True, depth=True, ir=True, folder="@session/scan", enqueue=False))

# Print with concurrent motion
session.run_sync(session.motion.plan(x=0.2, y=0.8, z=0.7, enqueue=False))
session.run_group([
    session.motion.exec(enqueue=False),
    session.extruder.print_steps(steps=2000, speed=500, enqueue=False),
])
```

This mirrors the behavior in `custom_session.py` and `custom_sequence.py` while showing the command surface clearly.
