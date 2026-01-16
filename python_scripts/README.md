# python_scripts

Standalone calibration, capture, and reconstruction scripts. These are not installed as ROS 2 packages and most files assume a local session layout plus hard-coded paths; edit the constants at the top of each script before running.

## Layout
- `3d_reconstruction/`: TSDF integration experiments (CPU and legacy GPU flows).
- `handeye_calibration/`: hand-eye calibration utilities (RGB/IR Charuco).
- `utils/`: shared helpers (session, manifest, intrinsics, extrinsics, transforms, loading).

## 3d_reconstruction

### `3d_reconstruction/o3d_cpu_reconstruction.py`
CPU-only TSDF integration over a session.
- `TSDF_Integration.__init__(session, voxel_size, block_count, block_resolution, depth_max, depth_scale, device)`
  - Loads manifest, robot poses, extrinsics, depth images, and intrinsics.
  - Initializes Open3D `VoxelBlockGrid` on CPU.
- `TSDF_Integration._tensorize_robot_poses()`
  - Converts each `T_base_ir` to an Open3D tensor on CPU.
- `TSDF_Integration._tensorize_intrinsics()`
  - Converts the pinhole intrinsics matrix to an Open3D tensor.
- `TSDF_Integration._tensorize_depth_images()`
  - Loads depth images into Open3D tensors.
- `TSDF_Integration.integrate_depths()`
  - Integrates each depth frame into the TSDF, builds per-frame point clouds, and returns a mesh.

The script runs an integration at import time, visualizes point clouds, and writes a mesh to `/home/lab/robot/meshes/tsdf_mesh.stl`.

### `3d_reconstruction/o3d_gpu_reconstruction_old.py`
Legacy GPU TSDF integration pipeline.
- `load_camera_intrinsics(path)`
  - Reads an OpenCV YAML intrinsics file and returns a `PinholeCameraIntrinsic`.
- `load_camera_extrinsics(path, frame_key="tool0_to_ir")`
  - Loads extrinsics from JSON/YAML and returns `T_cam_to_tool`.
- `load_robot_poses_manifest(path)`
  - Parses `manifest.yaml` into `{depth_path, pose_matrix}` entries.
- `transform_robot_to_camera_pose(captures, T_cam_to_tool)`
  - Adds `camera_pose` per capture (`T_base_tool @ T_cam_to_tool`).
- `sanity_check_motion(captures)`
  - Prints translation deltas; raises if poses are identical.
- `TSDFIntegrationGPU._tensorize_depth(depth_path)`
  - Loads a depth image, optionally resizes, returns Open3D tensor image.
- `TSDFIntegrationGPU._tensorize_extrinsic_cpu(T_cam_in_world)`
  - Returns inverse pose as CPU tensor.
- `TSDFIntegrationGPU.integrate_frame(T_cam_in_world, depth_path)`
  - Integrates one frame into the TSDF volume.
- `TSDFIntegrationGPU.integrate_captures(captures)`
  - Loops over all captures, integrating each.
- `TSDFIntegrationGPU.extract_mesh(out_path=None)`
  - Extracts a mesh and optionally writes it to disk.

## handeye_calibration

### `handeye_calibration/handeye_v4.py`
Session-driven hand-eye calibration that uses `python_scripts/utils` helpers.
- `detect_charuco(img, K, D, board, dictionary, axis_len=0.1, refine_corners_kernel=None, debug=False)`: detect Charuco, optionally visualize.
- `main()`: loads session/manifest, runs IR and color calibration, prints results, optional chain validation.

## utils package

### `utils/session.py`
- `Session.__init__(path, init_scan_folder=None, config_folder="config")`: stores folder layout, resolves intrinsics/extrinsics paths.
- `Session.load_scan_folder(scan_folder)`: loads capture paths.
- `Session._get_image_file_paths(image_type)`: assemble file paths from captures.
- `Session._get_extrinsics_path()`, `Session._get_intrinsics_path()`: resolve config files.

### `utils/manifest.py`
- `read_manifest(session_path, scan_folder)`: read `manifest.yaml`.
- `load_robot_poses(manifest)`: build `T_base_tool0` 4x4 matrices.
- `load_robot_poses_decomposed(manifest)`: returns separate translation/rotation lists.
- `transform_robot_to_camera_pose(robot_poses, extrinsics, type="T_tool0_ir")`: apply tool-to-camera transform.
- `construct_image_paths(manifest, session, image_type="color")`: build absolute image paths.

### `utils/intrinsics.py`
- `load_intrinsics(file_path)`: read `camera_matrix`, `distortion_coefficients`, image size.
- `intrinsics_matrix(width, height, K)`: create Open3D pinhole intrinsics.

### `utils/extrinsics.py`
- `load_extrinsics(file_path, frame_key="T_tool0_color")`: read tool0->camera extrinsics from YAML.

### `utils/image_loader.py`
- `load_images(image_paths, image_type, library)`: load images via `cv2` or `open3d`.
- `load_color_image(path)`: read RGB/BGR, normalize 16-bit to 8-bit.
- `load_ir_image(path)`: read IR, normalize 16-bit to 8-bit grayscale.
- `preprocess_percentile_ir(...)`: percentile stretch + optional gamma + CLAHE.
- `preprocess_threshold_ir(gray, ir_threshold, use_clahe=False)`: clip and optional CLAHE.

### `utils/transforms.py`
- `rotmat_to_quat_xyzw(Rm, reorthonormalize=True)`
- `rotmat_to_rpy(Rm, degrees=False, reorthonormalize=True)`
- `compose_T(rvec, tvec)`

### `utils/integration.py`
- `visualize_camera_poses(T_base_tool0_list, T_base_ir_list)`: draw coordinate frames in Open3D.


## Notes
- Dependencies: `numpy`, `opencv-python`, `open3d`, `scipy`, `pyyaml`.
- Most scripts assume a session layout with `manifest.yaml/json` and `config/` intrinsics/extrinsics.

## Actionable improvements
- Replace hard-coded paths with CLI args and/or a shared config file.
- Promote stable scripts into a proper ROS 2 package (entry points + install).
- Standardize manifest schema across scripts (one YAML schema, one JSON fallback).
- Add a `requirements.txt` or `pyproject.toml` for reproducible installs.
- Mark or remove deprecated scripts (for example, `o3d_gpu_reconstruction_old.py`).
- Add a small smoke-test dataset and scripted validation for calibration/reconstruction.
- Remove `sys.path` hacks by packaging shared utilities and using proper imports.
- Add `argparse` to every script with `--session`, `--config`, and `--output` flags.
- Use Python `logging` instead of `print` for consistent verbosity control.
- Validate input files (manifest, intrinsics, extrinsics) early with clear errors.
- Write outputs into a versioned `results/` folder per run (timestamp + params).
