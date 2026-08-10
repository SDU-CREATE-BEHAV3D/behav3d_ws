# Behav3D Command Surface and Orchestration Guide (AI Agent)

**Purpose**  
This document describes the Behav3D command surface, how commands map to ROS 2 interfaces, and how the active orchestration sessions compose them. It is intended for AI agents that must reason about system dynamics, available commands, and correct usage constraints.

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
- `behav3d_motion_bridge` (planning + pose + planning-scene services).
- Orbbec camera driver (Femto Mega).
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
- `behav3d_fields` (`behav3d_sense/fields_node.py`) for:
  - `/behav3d/init_field_from_scan`
  - `/behav3d/generate_print_candidates`

---

**Command Architecture (behav3d_commands)**

Primary entry point: `behav3d_commands.Session` in `src/behav3d_commands/behav3d_commands/session.py`.

Key concepts:
- `Session` owns a `SessionQueue`, a `CommandRouter`, and five command sets: motion, camera, field, extruder, util.
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
- Service: `/behav3d/plan_pilz_sequence` (`PlanPilzSequence`) for Pilz LIN sequences
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
| `plan_sequence()` | Plan and store a Pilz LIN sequence. | `poses`, `eef`, `vel_scale`, `accel_scale`, `blend_radius`, `blend_radii`, `frame_id`, `target_tcp_speed_m_s`, `retime_min_dt_s`, `tcp_sample_spacing_m`, `tcp_speed_threshold_m_s` | Uses `/behav3d/plan_pilz_sequence`; optional constant TCP-speed retime. |
| `exec()` | Execute last planned trajectory. | none | Fails if no plan stored. |
| `goto()` | Plan and optionally execute in one step. | `pose` or `x,y,z`, `rx,ry,rz`, `eef`, `vel_scale`, `accel_scale`, `exec`, `motion` | Default `exec=True`. |
| `goto_sequence()` | Plan and optionally execute a Pilz LIN sequence. | same as `plan_sequence`, plus `exec` | Default `exec=True`. |
| `setPTP()` | Set default mode to PTP. | none | Updates `motion_mode`. |
| `setLIN()` | Set default mode to LIN. | none | Updates `motion_mode`. |
| `setEef(name)` | Set default end-effector link. | `name` | Used for subsequent plans. |
| `setSpd(val)` | Set default velocity scale. | `val` (0..1) | Clamped to [0,1]. |
| `setAcc(val)` | Set default acceleration scale. | `val` (0..1) | Clamped to [0,1]. |

Pose handling rules:
- If `pose` is a `PoseStamped`, it is used directly.
- Otherwise `x,y,z` must be provided; `frame_id` defaults to `"world"`.
- If any of `rx,ry,rz` is provided, they are treated as RPY (rad) and converted to a quaternion; otherwise orientation is identity.

Pilz sequence / TCP retime rules:
- Pilz sequence planning uses MoveIt's Pilz Industrial Motion Planner through
  `sequence_move_group`; the bridge sets each item planner to `LIN`.
- The sequence request accepts poses in `frame_id` and an EEF link, normally
  `extruder_tcp`.
- `blend_radius` is in meters. The final sequence item is forced to zero blend
  radius.
- If `target_tcp_speed_m_s > 0`, the bridge retimes the Pilz trajectory for
  approximately constant TCP speed. The retime does not call IK again; it
  interpolates robot states along the already planned Pilz trajectory.
- The current retime mode first resamples the TCP path using
  `tcp_sample_spacing_m` and then assigns timestamps. The effective sample
  spacing is `max(tcp_sample_spacing_m, target_tcp_speed_m_s * retime_min_dt_s)`.
- `retime_min_dt_s` is only a minimum timestamp guard for the controller.
  Prefer tuning `tcp_sample_spacing_m` for path density and smoothness.
- `tcp_speed_threshold_m_s` is diagnostic only; it counts slow TCP samples in
  returned metrics and does not reject a trajectory.
- `behav3d_motion_bridge` clamps requested TCP retime speed with
  `max_tcp_speed_m_s` (default `0.350`, i.e. `350 mm/s`). Clamp is logged and
  does not fail planning.
- If retime fails and bridge parameter `retime_fallback_on_failure=true`, the
  original Pilz plan is returned with `tcp_speed_retime_fallback=true`.
- For separate polylines, transition to the first target with a normal
  `goto(..., motion="LIN")` or `goto(..., motion="PTP")` outside the sequence;
  do not include long non-print transitions inside a constant-TCP print
  sequence.

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
- Service `/reconstruct/color_to_depth` (`ColorToDepth`)
- Service `/reconstruct/tsdf_cropped` (`TsdfCropped`)
- Service `/reconstruct/tsdf_object_extract` (`TsdfObjectExtract`)
- Service `/behav3d/update_world_mesh` (`UpdateWorldMesh`)
- Service `/behav3d/update_planning_scene_mesh` (`UpdatePlanningSceneMesh`)

Camera commands:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `capture()` | Capture RGB/Depth/IR (+ optional pose). | `rgb`, `depth`, `ir`, `pose`, `folder` | If `folder` is provided, `set_folder=True`. |
| `get_pose()` | Get link pose in a base frame. | `eef`, `base_frame`, `use_tf` | If `use_tf=True`, bypasses MoveIt and reads TF. |
| `reconstruct_color_to_depth()` | Run color-to-depth alignment stage. | `use_latest`, `session_path`, `scan_folder`, `visualize` | Calls `/reconstruct/color_to_depth`. |
| `reconstruct_color_to_depth_grid_sweep()` | Color-to-depth for grid sweep captures. | `use_latest`, `session_path`, `scan_folder`, `visualize` | Default `scan_folder="grid_sweep"`. |
| `reconstruct_tsdf_cropped()` | Run TSDF cropped stage. | `use_latest`, `session_path`, `scan_folder`, `visualize`, `device` | Calls `/reconstruct/tsdf_cropped`. |
| `reconstruct_tsdf_grid_sweep()` | TSDF cropped for grid sweep captures. | `use_latest`, `session_path`, `scan_folder`, `visualize`, `device` | Default `scan_folder="grid_sweep"`. |
| `update_world_mesh()` | Update RViz world geometry from explicit or inferred reconstruction outputs. | `use_latest`, `session_path`, `mesh_path`, `ply_path`, `prefer`, `wait_timeout_s` | Calls `/behav3d/update_world_mesh`. `prefer="mesh"` publishes `MESH_RESOURCE`; `prefer="ply"` publishes colored PLY markers (`POINTS`/`TRIANGLE_LIST`) for debugging when available. Prefer explicit `mesh_path`/`ply_path` from TSDF response. |
| `update_planning_scene_mesh()` | Update MoveIt planning-scene collision geometry from reconstructed mesh outputs. | `use_latest`, `session_path`, `mesh_path`, `object_id`, `frame_id`, `wait_timeout_s` | Calls `/behav3d/update_planning_scene_mesh`. Prefer explicit `mesh_path` from TSDF response. Re-using the same `object_id` replaces the previous obstacle mesh. If `frame_id` is empty, the motion bridge uses the MoveIt planning frame. For reconstructed TSDF meshes, `frame_id` should normally match the RViz world-mesh frame (`behav3d_world.mesh_frame_id`, currently `ur20_base_link`). |

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
- For `update_planning_scene_mesh`, if explicit `mesh_path` is provided, use it; do not rely on fallback discovery unless necessary.
- `update_planning_scene_mesh` applies the mesh as a MoveIt `CollisionObject` for obstacle avoidance; this is separate from RViz visualization in `update_world_mesh`.
- `update_world_mesh` is the frame/alignment reference for reconstructed geometry. The planning-scene mesh should use the same frame interpretation as the RViz world mesh.
- Verified behavior: if `update_world_mesh` publishes in `ur20_base_link` but `update_planning_scene_mesh` is added in `world`, the collision mesh can appear correct in RViz yet be offset in MoveIt. In that case motions may seem to pass through the obstacle.
- Verified behavior: when the planning-scene collision mesh is added in the same frame as the RViz world mesh (`ur20_base_link` in the current bringup), the obstacle blocks planning as expected.
- Current limitation: there is no explicit Behav3D command/service yet to remove a planning-scene collision mesh by `object_id`. Re-using the same `object_id` replaces the obstacle mesh, but clearing it explicitly still requires a future API addition.
- TODO: add a planning-scene removal service in `behav3d_motion_bridge` (expected implementation: `PlanningSceneInterface.removeCollisionObjects([object_id])`) and wrap it in `behav3d_commands`.
- `tsdf_object_extract` is currently exposed as a ROS service only (`/reconstruct/tsdf_object_extract`); it is launched in bringup but not wrapped as a `CameraCommands` method yet.
- TSDF depth bias correction is currently applied internally in `src/behav3d_sense/behav3d_sense/reconstruct/TSDF_cpu_cropped.py` via `DEPTH_BIAS_MM` (current tuned value: `-5.1` mm). This bias is not part of the `/reconstruct/tsdf_cropped` service request contract.

Reconstruction + world-mesh protocol (current):
1. Choose a scan folder per cycle (recommended incremental naming: `grid_sweep_00`, `grid_sweep_01`, ...).
2. Capture into that folder (`@session/<scan_folder>`), so each cycle is isolated.
3. Run `reconstruct_color_to_depth*` with the same `scan_folder` and wait for fresh `color_in_depth` outputs.
4. Run `reconstruct_tsdf_*` with the same `scan_folder` and read returned `mesh_path` and `rgb_ply_path`.
5. Call `update_world_mesh` using explicit `mesh_path`/`ply_path` and `prefer` (`mesh` or `ply`).
6. Call `update_planning_scene_mesh` using explicit `mesh_path` if the reconstructed surface should be used for MoveIt obstacle avoidance.
7. Use the same frame for planning-scene insertion as for RViz world-mesh publication. In the current bringup, `behav3d_world` publishes the reconstructed mesh in `ur20_base_link`, so `update_planning_scene_mesh(frame_id="ur20_base_link")` is the expected default.
8. World node stages a timestamped mesh copy in `/tmp/behav3d_world_mesh_cache` before publish, which avoids RViz stale-resource caching on repeated updates.

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

**Command Surface: Field**

Implementation: `src/behav3d_commands/behav3d_commands/sense_commands/field_commands.py`

Underlying ROS interfaces:
- Service `/behav3d/init_field_from_scan` (`InitFieldFromScan`)
- Service `/behav3d/generate_print_candidates` (`GeneratePrintCandidates`)

Field commands:

| Command | Purpose | Key Parameters | Notes |
| --- | --- | --- | --- |
| `init_field_from_scan()` | Build and position the scalar field once from scan mesh + field mesh. | `use_latest`, `session_path`, `scan_mesh_paths[]`, `field_mesh_path`, `state_output_dir` | Produces `field_state_init.npz` and debug `field_masked_init.ply`; returns positioned offset `(x,y,z)`. |
| `generate_print_candidates()` | Evaluate current scan against initialized field and produce print candidates. | `use_latest`, `session_path`, `field_state_path`, `scan_mesh_paths[]`, `output_dir`, `candidate_mode`, `beads_per_step`, `bead_separation_mm`, `bead_height_mm`, `walk_distance_mm`, `walk_step_mm`, `walk_max_steps`, `walk_start_fraction`, `orient_with_tangent`, `tangent_sign`, `clamp_to_cone`, `cone_max_tilt_deg`, `base_to_world_yaw_deg`, `target_zx/zy/zz`, `target_position_scale` | Produces cycle debug artifacts + final `targets.yaml` using `z_lift`, `gradient_lift`, or `gradient_walk`. |

Field command usage notes:
- If `session_path` is non-empty, command layer forces `use_latest=False` (explicit path wins).
- `scan_mesh_paths` can be omitted; the service resolves default TSDF mesh inside the session.
- `field_state_path` can be omitted in `generate_print_candidates`; the service resolves latest `field_state_init.npz`.
- Targets YAML is written in robot plane format `O(x,y,z) Z(i,j,k)` with position scaled by `target_position_scale` (default mm).
- `candidate_mode` supports `z_lift`, `gradient_lift`, and `gradient_walk`.
- Candidate selection is greedy: choose the valid contour point with minimum
  `heat_norm`, suppress points inside its spacing radius, and repeat. Heat sets
  priority; spacing determines feasibility and is not part of the heat score.
- Heat and optional width are interpolated at contour source points before lift
  or walk. Fixed mode uses `bead_separation_mm`; field-width mode uses
  `(width_a + width_b) / 2 - overlap`. A secondary Euclidean endpoint check
  removes candidates that become too close after displacement.
- Variable-width settings are parameters of `/behav3d_fields`, synchronized by
  the field-loop orchestrators before candidate generation. A width NPZ must
  contain `width_norm` in `[0, 1]` with the scalar field's vertex count/order.
- Variable-width dot and segment targets receive analytic `volume_mm3`;
  fixed-width targets omit it. `candidate_volume_factor` scales analytic volume
  before YAML; DDS voxel occupancy is not used to calculate requested volume.
- `candidate_mode` controls candidate geometry; the orchestrator `print_mode`
  independently selects flat `targets:` dots or nested `segments:` output.
  `print_mode=auto` maps `gradient_walk` to segments and other modes to dots.
- `gradient_walk` uses the `walk_*` parameters and forces tangent orientation.
  `gradient_lift` segment output may move each start toward its end using
  `candidate_segment_start_offset_mm`; the setting is ignored for dots and
  `gradient_walk`.
- If `orient_with_tangent=true`, target orientation is sampled from the scalar tangent field; optional orientation clamp is applied with `clamp_to_cone` and `cone_max_tilt_deg`.
- Base-link to world target reorientation is applied inside `fields_node` at YAML generation time (`base_to_world_yaw_deg`, default falls back to node parameter `target_base_to_world_yaw_deg`).

Fields node implementation:
- `src/behav3d_sense/behav3d_sense/fields_node.py`
- Services exposed: `/behav3d/init_field_from_scan`, `/behav3d/generate_print_candidates`

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
- `PlanAndExecute.action` remains defined, but the workspace currently has no
  `/behav3d/plan_and_execute` provider because the old orchestrator action
  server was removed.

Services in `src/behav3d_interfaces/srv` (key ones used by commands):
- `/behav3d/plan_pilz_ptp` (`PlanPilzPtp`) via `behav3d_motion_bridge`
- `/behav3d/plan_pilz_lin` (`PlanPilzLin`) via `behav3d_motion_bridge`
- `/behav3d/get_link_pose` (`GetLinkPose`) via `behav3d_motion_bridge`
- `/behav3d/update_planning_scene_mesh` (`UpdatePlanningSceneMesh`) via `behav3d_motion_bridge`
- `/capture` (`Capture`) via `behav3d_sense`
- `/reconstruct/color_to_depth` (`ColorToDepth`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/reconstruct/tsdf_cropped` (`TsdfCropped`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/reconstruct/tsdf_object_extract` (`TsdfObjectExtract`) via `behav3d_sense/reconstruct/reconstruct_services.py`
- `/behav3d/update_world_mesh` (`UpdateWorldMesh`) via `behav3d_sense/world_node.py`
- `/behav3d/init_field_from_scan` (`InitFieldFromScan`) via `behav3d_sense/fields_node.py`
- `/behav3d/generate_print_candidates` (`GeneratePrintCandidates`) via `behav3d_sense/fields_node.py`
- `update_print_config` (`UpdatePrintConfig`) via `behav3d_print`
- `get_print_status` (`GetPrintStatus`) via `behav3d_print`
- `/behav3d/publish_targets` (`PublishTargets`) via `behav3d_motion_bridge`
- `/behav3d/delete_markers` (`DeleteMarkers`) via `behav3d_motion_bridge`

Implementation references:
- `src/behav3d_motion_bridge/src/motion_bridge_node.cpp`
- `src/behav3d_orchestrator/behav3d_orchestrator/src/control_session.py`
- `src/behav3d_orchestrator/behav3d_orchestrator/src/scan_session.py`
- `src/behav3d_orchestrator/behav3d_orchestrator/src/print_session.py`
- `src/behav3d_print/behav3d_print/print_node.py`
- `src/behav3d_sense/behav3d_sense/sense_node.py`
- `src/behav3d_sense/behav3d_sense/reconstruction.py`

---

**Current Orchestration Session Pattern**

Active sequence nodes create one or more specialized sessions on the same ROS
node and run blocking orchestration in a worker thread:
- `ScanSession`: grid, Fibonacci, half-cylinder, capture, and reconstruction flows.
- `PrintSession`: YAML targets/segments and extrusion-aware print execution.
- `YamlSession`: YAML target and polyline parsing plus motion helpers.
- `FieldLoopSession`: field-state center helpers used by the oriented loop.

All derive from `ControlAwareSession` in
`src/behav3d_orchestrator/behav3d_orchestrator/src/control_session.py`.
Sessions attached to the same node share one transient-local subscription to
`/behav3d/control_state`. Before each internal `run_sync` command, the session
waits while the state is `paused`; command timeouts start only after resume.
Continuous segment printing keeps `extruder ON -> motion -> extruder OFF`
atomic and honors pause after the extruder is safely off.

Dot-print invariants:
- Configured `dot_steps` is net bead material, never bead plus retract.
- A target `volume_mm3` overrides `dot_steps` and is converted with
  `extrusion_steps_per_mm3`.
- Keep `extrusion_steps_per_mm3` as a physical unit conversion. Variable-width
  extrusion tuning belongs in `candidate_volume_factor` upstream of YAML.
- When retract is enabled, `PrintSession` sends
  `net_bead_steps + post_dot_retract_steps`, then performs the reverse retract.
- The canonical YAML schema is in `command_format.md`; print configuration is
  documented in `src/behav3d_orchestrator/README.md`.

Segment-print invariants:
- `segment_steps` is net material for segments without `volume_mm3`; a supplied
  volume is converted with `extrusion_steps_per_mm3`.
- `segment_steps_per_second` is the extruder pulse rate. `PrintSession` derives
  target duration and TCP speed from forward steps and segment length.
- `post_segment_retract_steps` is added to the forward request before the same
  explicit reverse step command runs. Do not compensate it in `segment_steps`.
- `segment_print_vel_scale` is only the initial trajectory-tuning seed; there is
  no manually configured segment TCP speed.

Global control flow:
1. Publish `std_msgs/String(data="stop")` on `/behav3d/control`.
2. `behav3d_world` toggles and publishes `paused` or `running` on
   `/behav3d/control_state`.
3. Active sessions finish their current command and wait before the next one.

This is cooperative pause, not trajectory cancellation or an emergency stop.

---

**Additional Orchestration Patterns (Current)**

Orchestration entry points in `behav3d_orchestrator`:
- `ros2 run behav3d_orchestrator print_field_oriented_sequence_v2`
- `ros2 run behav3d_orchestrator geometry_representation_scan_print_loop`
- `ros2 run behav3d_orchestrator print_yaml_and_scan_sequence`
- `ros2 run behav3d_orchestrator polyline_motion_sequence`
- `ros2 run behav3d_orchestrator scan_sequence`
- `ros2 run behav3d_orchestrator scan_yaml_targets_sequence`
- YAML-specific session implementation: `src/behav3d_orchestrator/behav3d_orchestrator/src/yaml_session.py`
- `parse_yaml_targets(...)` supports:
  - `{index: N, xyz: [x, y, z]}`
  - `{index: N, x: ..., y: ..., z: ...}`
  - `{index: N, plane: "O(x,y,z) Z(i,j,k)"}` where `O` is in mm and `Z` is the orientation normal
  - `[x, y, z]`
- Ordering rule: targets are sorted by `index` (fallback: file position).

Scan and print flows:
- `scan_sequence` runs one configured grid, Fibonacci, half-cylinder, or
  half-cylinder-with-side-caps scan through `ScanSession`.
- `scan_yaml_targets_sequence` loads explicit plane targets from YAML and runs
  LIN plan/execute, dwell, and capture per reachable target, followed by one
  reconstruction. Its config is reloaded before every target. Plane Z is the
  desired camera optical axis; free roll is matched to the current configured
  camera EEF orientation after the optional home.
- `print_yaml_and_scan_sequence` prints YAML targets in chunks and optionally
  scans/reconstructs after each chunk.
- `print_field_oriented_sequence_v2` runs the scan, reconstruction, scalar-field
  initialization, candidate generation, and dot/segment printing loop.
- Runtime configuration files are installed from
  `src/behav3d_orchestrator/config/`.

Pilz polyline stress/test flow:
- Entry point: `ros2 run behav3d_orchestrator polyline_motion_sequence`
- YAML format is grouped polylines:
  ```yaml
  polylines:
    - index: 0
      planes:
        - "O(-300.00,460.00,300.00) Z(0.00,0.00,1.00)"
        - "O(-200.00,500.00,300.00) Z(0.00,0.00,1.00)"
  ```
- Plane positions are always interpreted as millimeters and converted to meters.
  Do not add a `units` field.
- The node publishes RViz target markers only for the active polyline.
- `move_to_polyline_start=true` moves to the first target of each polyline with
  a normal `goto` using `start_motion` (default `LIN`) and no TCP retime; the
  Pilz sequence is then planned for the remaining points.
- If a polyline fails to move, plan, or execute, the node logs the skipped
  `index` and continues with the next polyline.
- Typical command for the resampled constant-TCP retime:
  ```bash
  ros2 run behav3d_orchestrator polyline_motion_sequence --ros-args \
    -p yaml_path:=/home/lab/behav3d_ws/yaml/pilz_polyline_dummy.yaml \
    -p target_speed_mm_s:=80.0 \
    -p retime_constant_tcp_speed:=true \
    -p retime_min_dt_s:=0.008 \
    -p tcp_sample_spacing_mm:=2.0 \
    -p blend_radius:=0.003 \
    -p start_motion:=LIN
  ```
- Confirm the new retime path by looking for:
  `TCP resample retime: original_points=... sampled_points=...`.

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
- Use the existing `ControlAwareSession`/`ControlPauseGate` implementation for
  global cooperative pause; sessions on one node must share the gate.
- Keep safety-critical continuous-print groups atomic so pause cannot leave the
  extruder running while motion is stopped.

Avoid:
- Calling `run_sync` directly inside ROS callbacks.
- Scheduling conflicting hardware actions in the same `run_group`.
- Assuming resource locks exist; `SessionQueue` does not enforce them.
- Re-implementing pose/quaternion helper logic inside sessions when equivalent utilities already exist in `behav3d_utils`.
- Subscribing separately to `/behav3d/control_state` in each sequence or adding
  sequence-local pause flags; pause belongs in `control_session.py`.

---

**Minimal Example (Pseudo-Flow)**

```python
session = ScanSession(node)

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

`ScanSession`, `PrintSession`, `YamlSession`, and `FieldLoopSession` automatically
honor `/behav3d/control_state` between their internal synchronous commands.
