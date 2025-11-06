import open3d as o3d
import os
import yaml
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R


# ### load intrinsics
# def load_intrinsics(file_path, type="depth"):
#     if type == "depth":
#         intrinsics_file = os.path.join(file_path, "config", "depth_intrinsics.yaml")
#     elif type == "color":
#         intrinsics_file = os.path.join(file_path,"config", "color_intrinsics.yaml")
#     elif type == "ir":
#         intrinsics_file = os.path.join(file_path,"config", "ir_intrinsics.yaml")
#     else:
#         raise ValueError(f"Unsupported type: {type}")

#     if not os.path.isfile(intrinsics_file):
#         raise FileNotFoundError(f"Intrinsics file not found: {intrinsics_file}")

#     fs = cv2.FileStorage(intrinsics_file, cv2.FILE_STORAGE_READ)
#     if not fs.isOpened():
#         raise IOError(f"Failed to open: {intrinsics_file}")

#     try:
#         K = fs.getNode("camera_matrix").mat()
#         D = fs.getNode("distortion_coefficients").mat()
#         width = int(fs.getNode("image_width").real() or 0)
#         height = int(fs.getNode("image_height").real() or 0)
#         K = np.asarray(K, dtype=np.float64).reshape(3, 3)
#         D = np.asarray(D, dtype=np.float64).ravel()
#         return width, height, K, D
#     finally:
#         fs.release()

# ### load extrinsics
# def load_extrinsics(file_path, frame_key="T_tool0_color"):
#     extrinsics = os.path.join(file_path, "config", "extrinsics.yaml")
#     if not os.path.isfile(extrinsics):
#         raise FileNotFoundError(f"Extrinsics file not found: {extrinsics}")
#     with open(extrinsics, "r") as f:
#         data = yaml.safe_load(f)

#     ### load color extrinsics
#     if frame_key== "T_tool0_color":
#         frame = data["frames"].get(frame_key, None)
#         if frame is None:
#             raise KeyError(f"Frame '{frame_key}' not found in extrinsics file")
        
#         translation = np.array(frame["xyz"])
#         quaternion = np.array(frame["quat_xyzw"])
#         rotation = R.from_quat(quaternion).as_matrix()

#         T_tool0_color = np.eye(4)
#         T_tool0_color[:3, :3] = rotation
#         T_tool0_color[:3, 3] = translation
#         return T_tool0_color
    
#     ### load ir extrinsics    
#     elif frame_key== "T_tool0_ir" or frame_key== "T_tool0_depth":
#         frame = data["frames"].get(frame_key, None)
#         if frame is None:
#             raise KeyError(f"Frame '{frame_key}' not found in extrinsics file")
        
#         translation = np.array(frame["xyz"])
#         quaternion = np.array(frame["quat_xyzw"])
#         rotation = R.from_quat(quaternion).as_matrix()

#         T_tool0_ir = np.eye(4)
#         T_tool0_ir[:3, :3] = rotation
#         T_tool0_ir[:3, 3] = translation
#         return T_tool0_ir
    
#     else:
#         raise ValueError(f"Unsupported frame_key: {frame_key}")

def load_camera_intrinsics(camera_intrinsics_path): #TODO: check implementation and implement parsing
    print(f"Loading camera intrinsics from: {camera_intrinsics_path}...", end='')
    if not os.path.isfile(camera_intrinsics_path):
        raise FileNotFoundError(f"Intrinsics file not found: {camera_intrinsics_path}")

    fs = cv2.FileStorage(camera_intrinsics_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Failed to open: {camera_intrinsics_path}")

    try:
        W = int(fs.getNode("image_width").real() or 0)
        H = int(fs.getNode("image_height").real() or 0)
        K = np.asarray(fs.getNode("camera_matrix").mat(),
                                                dtype=np.float64).reshape(3, 3)
        D = np.asarray(fs.getNode("distortion_coefficients").mat(),
                                                dtype=np.float64).ravel()
        print("Done.")
        return W, H, K, D
    finally:
        fs.release()

### load manifest
def load_manifest(file_path, scan_folder):
    manifest_file = os.path.join(file_path, scan_folder, "manifest.yaml")
    if not os.path.isfile(manifest_file):
        raise FileNotFoundError(f"Manifest file not found: {manifest_file}")
    with open(manifest_file, "r") as f:
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
        pose = capture.get("T_base_tool0", {})
        timestamp = capture.get("timestamp", None)
          
        # build absolute paths
        rgb_path = os.path.join(base_path, rgb_rel) if rgb_rel else None
        depth_path = os.path.join(base_path, depth_rel) if depth_rel else None
        ir_path = os.path.join(base_path, ir_rel) if ir_rel else None

        # build pose matrix
        translation = np.array(pose.get("position", [0,0,0]), dtype=float)
        quaternion = np.array(pose.get("orientation_xyzw", [0,0,0,1]), dtype=float)
        rotation = R.from_quat(quaternion).as_matrix()
        T_base_tool0 = np.eye(4)
        T_base_tool0[:3, :3] = rotation
        T_base_tool0[:3, 3] = translation

        parsed_captures.append({
            "index": capture.get("index", None),
            "timestamp": timestamp,
            "rgb_path": rgb_path,
            "depth_path": depth_path,
            "ir_path": ir_path,
            "T_base_tool0": T_base_tool0,
        })
    return parsed_captures

### transform from robot to camera pose
def transform_robot_to_camera_pose(captures, extrensics, type ="T_tool0_ir"):
    if type == "T_tool0_ir":
        T_tool0_ir = extrensics
        for c in captures:
            T_base_tool0 = c["T_base_tool0"]
            T_base_ir = T_base_tool0 @ T_tool0_ir
            c["T_base_ir"] = T_base_ir
            transformed_captures = captures
        return transformed_captures
    elif type == "T_tool0_color":
        T_tool0_color = extrensics
        for c in captures:
            T_base_tool0 = c["T_base_tool0"]
            T_base_color = T_base_tool0 @ T_tool0_color
            c["T_base_color"] = T_base_color
            transformed_captures = captures
        return transformed_captures
    else:
        raise ValueError(f"Unsupported type: {type}")
    
### load images
def load_images(captures, image_type, library):
    images = []
    for c in captures:
        if image_type == "rgb":
            img_path = c.get("rgb_path", None)
        elif image_type == "depth":
            img_path = c.get("depth_path", None)
        elif image_type == "ir":
            img_path = c.get("ir_path", None)
        else:
            raise ValueError(f"Unsupported image type: {image_type}")
        
        if img_path is None or not os.path.isfile(img_path):
            raise FileNotFoundError(f"Image file not found: {img_path}")
        
        if library == "cv2":
            if image_type == "depth":
                img = cv2.imread(img_path, cv2.IMREAD_UNCHANGED)
            else:
                img = cv2.imread(img_path, cv2.IMREAD_COLOR)
            images.append(img)
        elif library == "o3d":
            img = o3d.io.read_image(img_path)
            images.append(img)
        else:
            raise ValueError(f"Unsupported library: {library}")
    return images





# intrinsics_path, extrinsics_path, manifest_path = construct_paths("/home/lab/behav3d_ws/captures/251104_140521/config")

# load_intrinsics(intrinsics_path, type)



# print (session_path)