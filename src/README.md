# src

ROS 2 workspace packages for the Behav3D stack. Each subdirectory is a package (Python or C++). This README focuses on the modules and functions you will touch most often.

## Package index

### `behav3d_bringup`
Launch-only package for wiring the system.
- `launch/print_move.launch.py`: brings up print + motion stack.
- `launch/kinematics_ur20_demo.launch.py`: UR20 kinematics demo launch.

### `behav3d_cpp`
C++ library + nodes for motion, capture, and session workflows. Public APIs are defined in `include/behav3d_cpp/*.hpp`.

Key classes and functions:
- `behav3d::motion_controller::PilzMotionController` (`motion_controller.hpp`)
  - `planTarget(...)`, `planJoints(...)`, `planSequence(...)`
  - `executeTrajectory(...)`, `getCurrentPose(...)`, `getRelativePose(...)`
  - `computeIK(...)`, `computeFK(...)`, `isReachable(...)`, `cancelAllGoals()`
- `behav3d::motion_visualizer::MotionVisualizer` (`motion_visualizer.hpp`)
  - `publishTargetPose(...)`, `publishTrail(...)`, `publishGhost(...)`
  - `deleteAllMarkers()`, `prompt(...)`, `trigger()`
- `behav3d::camera_manager::CameraManager` (`camera_manager.hpp`)
  - `initSession(...)`, `capture(...)`, `captureAsync()`, `waitForIdle()`
  - `getCalibration(...)` (optionally writes intrinsics YAML)
- `behav3d::session_manager::SessionManager` (`session_manager.hpp`)
  - `initSession(...)`, `run(...)`, `finish()`, `getSessionDir()`
- `behav3d::trajectory_builder` (`trajectory_builder.hpp`)
  - `fibonacciSphericalCap(...)`, `sweepZigzag(...)`, `addJitter(...)`, `blend(...)`, `uniformSphere(...)`
- `behav3d::target_builder` (`target_builder.hpp`)
  - `poseStamped(...)`, `worldXY(...)`, `worldXZ(...)`, `worldYZ(...)`
  - `toIso(...)`, `fromIso(...)`, `translate(...)`, `rotateEuler(...)`, `transformRel(...)`
  - `changeBasis(...)`, `rebase(...)`, `xAxis(...)`, `yAxis(...)`, `zAxis(...)`
  - `alignTarget(...)`, `adjustTarget(...)`, `axisAngle(...)`, `poseBetween(...)`
  - `mirrorAcrossPlane(...)`, `average(...)`, `offsetTarget(...)`, `setTargetOrigin(...)`
  - `flipTargetAxes(...)`, `swapTargetAxes(...)`
- `behav3d::util` (`util.hpp`)
  - `deg2rad(...)`, `rad2deg(...)`, `wrapAngle(...)`, `randomUnitVector(...)`, `fromRPY(...)`
  - `readJson(...)`, `writeJson(...)`, `readYaml(...)`, `writeYaml(...)`
  - `indexString(...)`, `timeStringDateTime(...)`, `toJsonPose(...)`, `toJsonJoints(...)`
  - `writeIntrinsicsYAML_OpenCV(...)`
- `behav3d::handeye::HandeyeCalibration` (`handeye_calibration.hpp`)
  - `getSessionParameters()`, `run()` and per-step helpers for manifests + Charuco detection.

Implementation files live under `src/behav3d_cpp/src/*.cpp`.

### `behav3d_demo`
C++ demo nodes and launchers.
- `behav3d_demo/demo.cpp`: motion demo.
- `behav3d_demo/mancap.cpp`: capture/mancap demo.
- `launch/*.launch.py`: UR10e/UR20 demo launches.

### `behav3d_interfaces`
ROS 2 interface definitions.
- Actions: `PlanAndExecute.action`, `PrintSteps.action`, `PrintTime.action`.
- Services: `Capture.srv`, `CaptureRGBD.srv`, `CaptureSnapshot.srv`, `ComputeHandEye.srv`,
  `GetLinkPose.srv`, `GetPrintStatus.srv`, `PlanCartesianPath.srv`, `PlanPilzLin.srv`,
  `PlanPilzPtp.srv`, `PlanPilzSequence.srv`, `PlanWithMoveIt.srv`, `ReconstructMesh.srv`,
  `UpdatePrintConfig.srv`.

### `behav3d_motion_bridge`
MoveIt2 bridge node (`src/motion_bridge_node.cpp`) exposing planning services and optional execution.
- Services: `/behav3d/plan_with_moveit`, `/behav3d/plan_cartesian_path`,
  `/behav3d/plan_pilz_ptp`, `/behav3d/plan_pilz_lin`, `/behav3d/plan_pilz_sequence`,
  `/behav3d/get_link_pose`.
- Publishes: `/move_group/display_planned_path`, `/behav3d/last_plan`, `/behav3d/markers/eef_path`.
- Supports controller execution via `control_msgs/FollowJointTrajectory`.

### `behav3d_orchestrator`
Python action server that wraps `PlanAndExecute` on top of the motion bridge.
- `Orchestrator` methods (`behav3d_orchestrator/orchestrator_node.py`):
  - `goal_cb(...)`, `cancel_cb(...)`, `execute_cb(...)`
  - `_wait_for_bridge(...)`, `_call_bridge(...)`

### `behav3d_print`
Python Modbus extruder control and print actions.
- `PrintNode` (`behav3d_print/print_node.py`):
  - Modbus controls: `set_extrude(...)`, `set_speed(...)`, `enqueue_steps(...)`
  - Low-level helpers: `_write_u32(...)`, `_pulse_coil(...)`, `_read_steps_remaining_now()`
  - ROS callbacks: `_poll()`, `_on_update_config(...)`, `_on_get_status(...)`
  - Actions: `_exec_print_time(...)`, `_exec_print_steps(...)`

### `behav3d_py`
Python utilities + demos used by motion/printing workflows. See `src/behav3d_py/README.md` for API and examples.

### `behav3d_sense`
Capture pipeline and reconstruction service.
- `SenseNode` (`behav3d_sense/sense_node.py`):
  - Session helpers: `_create_new_session_dir()`, `_resolve_folder_arg()`, `_ensure_manifest()`
  - TF helpers: `_lookup_T_base_tool0()`, `_lookup_tool0_to(...)`
  - IO helpers: `_append_capture_entry(...)`, `_save_extrinsics_yaml(...)`
  - Service handler: `handle_capture(...)`
- `FemtoCapture` (`behav3d_sense/femto_capture.py`):
  - Sync + frame handling: `compose_pair_from_raw()`
  - Saving: `save_color(...)`, `save_depth(...)`, `save_ir(...)`
  - Intrinsics export: `save_intrinsics_yaml(...)`
- `ThreeDReconstructor` (`behav3d_sense/reconstruction.py`):
  - `handle_request(...)`, `_resolve_session_path(...)`, `_run_reconstruction(...)`

### `behav3d_vision`
Vision utilities and motion-sequence runner.
- `PoseSequenceRunner` (`behav3d_vision/pose_sequence_runner.py`):
  - `run()` and service helpers `_send_named(...)`, `_send_cartesian(...)`, `_send_pilz_ptp(...)`,
    `_send_pilz_lin(...)`, `_send_pilz_sequence(...)`, `_send_pilz_sequence_from_points(...)`.
- `ImageProcessor` (`behav3d_vision/image_processor.py`):
  - `_store(...)`, `_closest_pair(...)`, `_save_depth(...)`, `_capture_cb(...)`.
- `RGBDCapture` (`behav3d_vision/rgbd_capture.py`):
  - Partial stub showing message_filters sync + `cb_images(...)`.

### `custom_workcell`
UR10e/UR20 workcell URDFs, meshes, calibration YAMLs, and MoveIt configurations.
- `i40_workcell/` and `ur20_workcell/` for robot descriptions.
- `*_moveit_config/` for MoveIt configs, planners, and launch.

### `kinematics_demo_cpp`
C++ kinematics demo with launch files for UR10e and UR20.

### `kinematics_demo_py`
Python demo node built on `behav3d_py`.
- `PilzDemo` (`kinematics_demo_py/demo.py`):
  - Commands: `home()`, `draw_square()`, `draw_square_seq()`, `draw_circle()`, `draw_circle_seq()`, `draw_line()`
  - Helpers: `RobotState_from_joints(...)`, `PoseStamped_from_xyzq(...)`, `PoseStamped_WorldXY(...)`

### `world_visualizer`
Publishes a mesh marker for RViz.
- `MeshVisualizer` (`world_visualizer/mesh_visualizer.py`):
  - `publish_latest_mesh(...)` and file-watcher callbacks in `MeshUpdateHandler`.

## Related docs
- `python_scripts/README.md` for standalone scripts.
- `src/behav3d_py/README.md` for the Python API and demo entry points.

## Actionable improvements
- Add per-package READMEs for all ROS packages (build/run/test instructions + topics/services).
- Standardize package naming and node names across Python and C++ stacks (avoid one-off demos).
- Consolidate repeated capture/session logic between `behav3d_sense` and `behav3d_cpp`.
- Move hard-coded parameters into ROS params or YAML configs (paths, IPs, frames).
- Deprecate or complete partial stubs (for example, `behav3d_vision/rgbd_capture.py`).
- Split core vs demo packages (e.g., `behav3d_core/*`, `behav3d_demos/*`) for clarity.
- Add a top-level `ARCHITECTURE.md` with system diagram, data flow, and dependency map.
- Define a common message/manifest schema document under `docs/` and link to it.
- Add a `colcon` build profile and a standard launch entry point for the full stack.
- Introduce a `ci/` or `.github/workflows/` pipeline for lint, tests, and build. [Oz]
