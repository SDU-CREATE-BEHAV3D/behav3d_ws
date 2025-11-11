import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R


def read_manifest(session_path, scan_folder):
    manifest_path = os.path.join(session_path, scan_folder, "manifest.yaml")
    with open(manifest_path, 'r') as f:
        manifest = yaml.safe_load(f)
    return manifest

### load robot poses from manifest
def load_robot_poses(manifest):
    captures = manifest.get("captures", [])
    for c in captures:
        pose = c.get("T_base_tool0", {})
        translation = np.array(pose.get("position", [0,0,0]), dtype=float)
        quaternion = np.array(pose.get("orientation_xyzw", [0,0,0,1]), dtype=float)
        rotation = R.from_quat(quaternion).as_matrix()
        T_base_tool0 = np.eye(4)
        T_base_tool0[:3, :3] = rotation
        T_base_tool0[:3, 3] = translation
        c["T_base_tool0"] = T_base_tool0
        
        # Collect all T_base_tool0 matrices
    T_base_tool0 = [c["T_base_tool0"] for c in captures]

    return T_base_tool0

### transform from robot to camera pose
def transform_robot_to_camera_pose(robot_poses, extrensics, type ="T_tool0_ir"):
    if type == "T_tool0_ir":
        T_tool0_ir = extrensics
        for T in robot_poses:
            T = robot_poses @ T_tool0_ir
            print("T:", T)
            T_base_ir = T
        return T_base_ir

    elif type == "T_tool0_color":
        T_tool0_color = extrensics
        for T in robot_poses:
            T = robot_poses @ T_tool0_color
            T_base_color = T
        return T_base_color
    else:
        raise ValueError(f"Unsupported type: {type}")
    

### construct images paths from the manifest
def construct_image_paths(manifest, image_type="rgb"):
    captures = manifest.get("captures", [])
    image_paths = []
    for c in captures:
        file_name = c.get(image_type)
        if file_name:
            image_paths.append(file_name)
    return image_paths


