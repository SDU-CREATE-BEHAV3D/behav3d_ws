import os
import yaml
import numpy as np
from scipy.spatial.transform import Rotation as R
import cv2

### load extrinsics
### load extrinsics
def load_extrinsics(file_path, frame_key="T_tool0_color"):
    with open(file_path, "r") as f:
        data = yaml.safe_load(f)

    ### load color extrinsics
    if frame_key== "T_tool0_color":
        frame = data["frames"].get(frame_key, None)
        if frame is None:
            raise KeyError(f"Frame '{frame_key}' not found in extrinsics file")
        
        translation = np.array(frame["xyz"])
        quaternion = np.array(frame["quat_xyzw"])
        rotation = R.from_quat(quaternion).as_matrix()

        T_tool0_color = np.eye(4)
        T_tool0_color[:3, :3] = rotation
        T_tool0_color[:3, 3] = translation
        return T_tool0_color

    ### load ir extrinsics    
    elif frame_key== "T_tool0_ir" or frame_key== "T_tool0_depth":
        frame = data["frames"].get(frame_key, None)
        if frame is None:
            raise KeyError(f"Frame '{frame_key}' not found in extrinsics file")
        
        translation = np.array(frame["xyz"])
        quaternion = np.array(frame["quat_xyzw"])
        rotation = R.from_quat(quaternion).as_matrix()

        T_tool0_ir = np.eye(4)
        T_tool0_ir[:3, :3] = rotation
        T_tool0_ir[:3, 3] = translation
        return T_tool0_ir
    
    else:
        raise ValueError(f"Unsupported frame_key: {frame_key}")