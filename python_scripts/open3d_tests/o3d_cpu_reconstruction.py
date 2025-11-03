import argparse
import datetime
from pathlib import Path
import numpy as np
import yaml
import cv2
import open3d as o3d
import open3d.core as o3c
from scipy.spatial.transform import Rotation as R
import json
import os


# ---------- INTRINSICS / EXTRINSICS ----------
def load_camera_intrinsics(file_path):
    # check file existence
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return None, None

    # read YAML file using OpenCV
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print(f"Failed to open the file {file_path}.")
        return None, None

    # extract parameters
    width  = int(fs.getNode("image_width").real())
    height = int(fs.getNode("image_height").real())
    camera_matrix = fs.getNode("camera_matrix").mat()
    dist_coeffs = fs.getNode("distortion_coefficients").mat()
    fs.release()

    # convert intrinsics to Open3D format
    intrinsic = o3d.camera.PinholeCameraIntrinsic(
        width, height,
        camera_matrix[0, 0],  # fx
        camera_matrix[1, 1],  # fy
        camera_matrix[0, 2],  # cx
        camera_matrix[1, 2],  # cy
    )

    return intrinsic, dist_coeffs
# print(load_camera_intrinsics("/home/lab/behav3d_ws/captures/session-20250905_180510_mancap/calib/depth_intrinsics.yaml"))


def load_camera_extrinsics(file_path, frame_key="tool0_to_color"):
    # check file existence
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return None

    # detect file type
    ext = os.path.splitext(file_path)[1].lower()

    if ext in [".json"]:
        # JSON hand-eye calibration
        with open(file_path, 'r') as f:
            data = json.load(f)
        translation = np.array(data["translation_m"])
        quaternion = np.array(data["quaternion_xyzw"])
        rotation = R.from_quat(quaternion).as_matrix()

        T_cam_to_tool = np.eye(4)
        T_cam_to_tool[:3, :3] = rotation
        T_cam_to_tool[:3, 3] = translation
        return T_cam_to_tool

    elif ext in [".yaml", ".yml"]:
        # YAML calibration with multiple frames
        with open(file_path, 'r') as f:
            data = yaml.safe_load(f)

        frames = data.get("frames", {})
        if frame_key not in frames:
            raise KeyError(f"Frame '{frame_key}' not found in YAML extrinsics file")

        frame = frames[frame_key]
        translation = np.array(frame["xyz"])
        quaternion = np.array(frame["quat_xyzw"])
        rotation = R.from_quat(quaternion).as_matrix()

        # determine direction: if parent is tool and child is camera → invert
        parent = frame.get("parent", "").lower()
        child = frame.get("child", "").lower()

        T_tool_to_cam = np.eye(4)
        T_tool_to_cam[:3, :3] = rotation
        T_tool_to_cam[:3, 3] = translation

        if "tool" in parent and ("color" in child or "camera" in child or "ir" in child):
            # the transform is tool→camera → invert to get camera→tool
            T_cam_to_tool = np.linalg.inv(T_tool_to_cam)
        else:
            T_cam_to_tool = T_tool_to_cam

        return T_cam_to_tool

    else:
        raise ValueError(f"Unsupported file extension: {ext}")
# print(load_camera_extrinsics("/home/lab/behav3d_ws/captures/session-20250905_180510_mancap/calib/extrinsics.yaml"))


def load_robot_poses_manifest(file_path):
    # check file existence
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return None
     
    # load YAML manifest
    with open(file_path, 'r') as f:
        manifest = yaml.safe_load(f)
     
    session_info = manifest.get("session", {})
    base_path = session_info.get("path", None)
    if not base_path or not os.path.isdir(base_path):
        raise ValueError(f"Invalid or missing capture path in manifest: {base_path}")
     
    captures = manifest.get("captures", [])
    parsed_captures = []

    # parse each capture entry
    for capture in captures:
        rgb_rel = capture.get("rgb")
        depth_rel = capture.get("depth")
        ir_rel = capture.get("ir")
        pose = capture.get("pose_tool0", {})
        timestamp = capture.get("timestamp", None)
          
        # build absolute paths
        rgb_path = os.path.join(base_path, rgb_rel) if rgb_rel else None
        depth_path = os.path.join(base_path, depth_rel) if depth_rel else None
        ir_path = os.path.join(base_path, ir_rel) if ir_rel else None

        # convert pose to 4x4 matrix
        translation = np.array(pose.get("position", [0, 0, 0]))
        quaternion = np.array(pose.get("orientation_xyzw", [0, 0, 0, 1]))
        rotation = R.from_quat(quaternion).as_matrix()
        pose_matrix = np.eye(4)
        pose_matrix[:3, :3] = rotation
        pose_matrix[:3, 3] = translation

        # append structured data
        parsed_captures.append({
            "index": capture.get("index", None),
            "timestamp": timestamp,
            "rgb_path": rgb_path,
            "depth_path": depth_path,
            "ir_path": ir_path,
            "pose_matrix": pose_matrix,
        })

    return parsed_captures
# print(load_robot_poses_manifest("/home/lab/behav3d_ws/captures/251021_115853/scan_1/manifest.yaml"))


def transform_robot_to_camera_pose(parsed_captures, T_cam_to_tool):
    # compute camera pose in base/world frame
    transformed_captures = []
    for cap in parsed_captures:
        T_tool_to_base = cap["pose_matrix"]                # from robot
        T_cam_to_base = T_tool_to_base @ T_cam_to_tool     # correct order
        cap["camera_pose"] = T_cam_to_base
        transformed_captures.append(cap)
    return transformed_captures


def load_depth_image(file_path):
    # check file existence
    if not os.path.isfile(file_path):
        print(f"The file {file_path} does not exist.")
        return None
    
    # read and return depth image
    depth_image = o3d.io.read_image(file_path)
    return depth_image


def visualize_camera_frustums(captures, intrinsic, scale=0.1):
    # create camera frustum visualization for all captures
    geometries = []
    for cap in captures:
        if "camera_pose" not in cap:
            continue
        T = cap["camera_pose"]
        frustum = o3d.geometry.LineSet.create_camera_visualization(
            intrinsic,             # pass the PinholeCameraIntrinsic directly
            np.linalg.inv(T),      # Open3D expects world→camera
            scale
        )
        geometries.append(frustum)

    # add world origin frame
    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    geometries.append(origin)
    o3d.visualization.draw(geometries)


# gpu reconstruction function with depth integration
def gpu_reconstruction(captures, intrinsic, voxel_length=0.004, sdf_trunc=0.04):
    # Initialize Open3D TSDF volume (CPU version for debugging)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=voxel_length,
        sdf_trunc=sdf_trunc,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor
    )
    
    expected_width = intrinsic.width
    expected_height = intrinsic.height
    print(f"Expected frame size from intrinsics: {expected_width}x{expected_height}")

    for cap in captures:
        depth_path = cap.get("depth_path", None)
        T = cap.get("camera_pose", None)
        if depth_path is None or T is None:
            continue

        # Load depth image
        depth_image = o3d.io.read_image(depth_path)
        if depth_image is None:
            continue

        # Convert to numpy for sanity check
        depth_np = np.asarray(depth_image)
        h, w = depth_np.shape

        # ---------- TEMPORARY RESIZE TO MATCH INTRINSICS ----------
        if (w, h) != (expected_width, expected_height):
            print(f"Resizing depth {w}x{h} → {expected_width}x{expected_height}")
            depth_np = cv2.resize(depth_np, (expected_width, expected_height), interpolation=cv2.INTER_NEAREST)
            depth_image = o3d.geometry.Image(depth_np)
        # ----------------------------------------------------------

        # Create a dummy gray image (required by Open3D RGBD factory)
        gray_image = o3d.geometry.Image(np.zeros((expected_height, expected_width), dtype=np.uint8))

        # Create RGBD image with no color
        rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
            gray_image,
            depth_image,
            depth_scale=1000.0,   # adjust for your depth units
            depth_trunc=3.0,
            convert_rgb_to_intensity=False
        )

        # Integrate depth frame
        volume.integrate(
            rgbd_image,
            intrinsic,
            np.linalg.inv(T)  # world → camera
        )
        print(f"Integrated capture {cap['index']}")

    # Extract surface mesh from TSDF
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    print("Reconstruction complete.")
    o3d.visualization.draw([mesh, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

    # Save mesh to file
    mesh_path = "reconstructed_mesh_gpu.stl"
    # o3d.io.write_triangle_mesh(mesh_path, mesh)


    return mesh

    





# ---------- MAIN EXECUTION ----------

intrinsic, _ = load_camera_intrinsics("/home/lab/behav3d_ws/captures/251103_170035/config/depth_intrinsics.yaml")
print("Intrinsics loaded")

T_cam_to_tool = load_camera_extrinsics("/home/lab/behav3d_ws/captures/251103_170035/config/extrinsics.yaml")
print("Extrinsics loaded:\n", T_cam_to_tool)

captures = load_robot_poses_manifest("/home/lab/behav3d_ws/captures/251103_170035/config/manifest.yaml")
captures = transform_robot_to_camera_pose(captures, T_cam_to_tool)
print(f"{len(captures)} captures parsed")

visualize_camera_frustums(captures, intrinsic, scale=0.05)
gpu_reconstruction(captures, intrinsic, voxel_length=2/512, sdf_trunc=0.04)



