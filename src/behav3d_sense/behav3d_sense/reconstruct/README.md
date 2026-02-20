# Behav3D Reconstruction Services

This folder contains the ROS 2 services and reconstruction scripts for Behav3D. The services wrap the scripts and run them asynchronously (service returns immediately, processing continues in a background thread).

## What’s Here

- `reconstruct_services.py`
  - ROS 2 service wrapper for:
    - `/reconstruct/color_to_depth`
    - `/reconstruct/tsdf_cropped`
    - `/reconstruct/tsdf_object_extract`
- `color_to_depth.py`
  - Produces aligned color-in-depth images and alignment diagnostics.
- `TSDF_cpu_cropped.py`
  - TSDF integration, cropping, optional plane fitting/slicing, point clouds.
  - Exports a mesh as `tsdf_surface_mesh.stl`.
- `TSDF_cpu_object_extract.py`
  - TSDF integration and object extraction using a precomputed table plane.

## Data Layout Expectations

Each capture session directory looks like:

- `config/` with intrinsics and extrinsics YAML files
- `<scan_folder>/manifest.yaml`
- `<scan_folder>/color_raw`, `depth_raw`, `ir_raw` (paths referenced by the manifest)

Example:

```
captures/260113_170839/
  config/
  manual_caps/
    manifest.yaml
    color_raw/
    depth_raw/
    ir_raw/
```

Reconstruction outputs are now scoped inside each scan folder:

- `captures/<session>/<scan_folder>/reconstruct/color_in_depth/` (from `color_to_depth`)
- `captures/<session>/<scan_folder>/reconstruct/tsdf_surface_*.{stl,ply}` (from TSDF services)

TSDF scripts read aligned images from `<scan_folder>/reconstruct/color_in_depth/`.

## Environment Setup

These scripts are self-contained inside `behav3d_sense` and do not depend on `python_scripts`.

```bash
source /home/lab/behav3d_ws/install/setup.bash
export BEHAV3D_CAPTURES_ROOT=/home/lab/behav3d_ws/captures
```

If ROS log writes fail due to permissions, set:

```bash
export ROS_HOME=/tmp/ros_home
export ROS_LOG_DIR=/tmp/ros_home/log
```

## Service Commands

### 1) Color-to-Depth Alignment

Start the service:

```bash
ros2 run behav3d_sense color_to_depth_service
```

Call it:

```bash
ros2 service call /reconstruct/color_to_depth behav3d_interfaces/srv/ColorToDepth \
"{use_latest: false, session_path: '/home/lab/behav3d_ws/captures/260113_170839', scan_folder: 'manual_caps', visualize: false}"
```

Output:

- `<scan_folder>/reconstruct/color_in_depth/`

### 2) TSDF Cropped (Mesh + Point Clouds)

Start the service:

```bash
ros2 run behav3d_sense tsdf_cropped_service
```

Call it (CPU):

```bash
ros2 service call /reconstruct/tsdf_cropped behav3d_interfaces/srv/TsdfCropped \
"{use_latest: false, session_path: '/home/lab/behav3d_ws/captures/260113_170839', scan_folder: 'manual_caps', visualize: false, device: 'CPU:0'}"
```

Call it (GPU, if Open3D CUDA is available):

```bash
ros2 service call /reconstruct/tsdf_cropped behav3d_interfaces/srv/TsdfCropped \
"{use_latest: false, session_path: '/home/lab/behav3d_ws/captures/260113_170839', scan_folder: 'manual_caps', visualize: false, device: 'CUDA:0'}"
```

Outputs:

- `<scan_folder>/reconstruct/tsdf_surface_mesh.stl`
- `<scan_folder>/reconstruct/tsdf_surface_rgb_colored.ply`
- `<scan_folder>/reconstruct/tsdf_surface_confidence_colored.ply`

### 3) TSDF Object Extract

Start the service:

```bash
ros2 run behav3d_sense tsdf_object_extract_service
```

Call it:

```bash
ros2 service call /reconstruct/tsdf_object_extract behav3d_interfaces/srv/TsdfObjectExtract \
"{use_latest: false, session_path: '/home/lab/behav3d_ws/captures/260113_170839', scan_folder: 'manual_caps', visualize: false}"
```

Device selection for this service uses ROS parameters (request has no `device` field):

```bash
ros2 run behav3d_sense tsdf_object_extract_service --ros-args -p device:=CPU:0
```

Outputs:

- `<scan_folder>/reconstruct/tsdf_surface_rgb_colored.ply`
- `<scan_folder>/reconstruct/tsdf_surface_confidence_colored.ply` (if enabled)

## Notes and Troubleshooting

- Services return immediately while reconstruction runs in a background thread. Watch the console logs for completion or errors.
- `tsdf_cropped` and `tsdf_object_extract` read aligned color images from `<scan_folder>/reconstruct/color_in_depth/`. Run `color_to_depth` first for the same `scan_folder`.
- `tsdf_object_extract` expects `table_plane.json` in `<scan_folder>/reconstruct/`. Generate it with `tsdf_cropped` in plane fit mode for that same folder.
- If you request `CUDA:0` but Open3D CUDA is not available, the code falls back to `CPU:0` with a warning.
