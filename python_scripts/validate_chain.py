#!/usr/bin/env python3
# validate_chain.py
# Validates full transform chain:
#   T_base→cam = T_base→tool0 × T_tool0→cam
#   T_base→board = T_base→cam × T_cam→board
# and checks consistency across captures.

import os
import json
import yaml
import numpy as np
import cv2

# ---------- Paths ----------
SESS_PATH = "/home/lab/behav3d_ws/captures/251104_140521"
YAML_TOOL0_CAM = os.path.join(SESS_PATH, "manual_caps", "tool0_from_cam.yaml")
JSON_CAM_BOARD = os.path.join(SESS_PATH, "manual_caps", "T_cam2board_list.json")
MANIFEST_YAML = os.path.join(SESS_PATH, "manifest.yaml")
MANIFEST_JSON = os.path.join(SESS_PATH, "manifest.json")

# ---------- Helpers ----------
def RT_to_T(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T

def quat_xyzw_to_R(qx, qy, qz, qw):
    q = np.array([qx, qy, qz, qw], dtype=float)
    q /= np.linalg.norm(q)
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ], dtype=float)

def load_tool0_from_cam(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    R = fs.getNode("R").mat()
    t = fs.getNode("t").mat()
    fs.release()
    return R, t

def load_manifest():
    if os.path.isfile(MANIFEST_YAML):
        with open(MANIFEST_YAML, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        captures = data.get("captures", [])
        return [{
            "image_path": c.get("ir") or c.get("rgb"),
            "q": c["pose_tool0"]["orientation_xyzw"],
            "p": c["pose_tool0"]["position"]
        } for c in captures if "pose_tool0" in c]
    elif os.path.isfile(MANIFEST_JSON):
        with open(MANIFEST_JSON, "r", encoding="utf-8") as f:
            data = json.load(f)
        captures = data.get("captures", [])
        return [{
            "image_path": c.get("ir") or c.get("rgb") or c.get("image_ir"),
            "q": c["pose_tool0"]["orientation_xyzw"],
            "p": c["pose_tool0"]["position"]
        } for c in captures if "pose_tool0" in c]
    else:
        raise FileNotFoundError("manifest.yaml or manifest.json not found")

# ---------- Load data ----------
R_t_c, t_t_c = load_tool0_from_cam(YAML_TOOL0_CAM)
T_tool0_cam = RT_to_T(R_t_c, t_t_c)

with open(JSON_CAM_BOARD, "r", encoding="utf-8") as f:
    cam_board_data = json.load(f)

manifest_data = load_manifest()

# ---------- Validation loop ----------
T_base_board_all = []
for m, cb in zip(manifest_data, cam_board_data):
    qx, qy, qz, qw = m["q"]
    px, py, pz = m["p"]
    R_b_t = quat_xyzw_to_R(qx, qy, qz, qw)
    t_b_t = np.array([px, py, pz])

    T_base_tool0 = RT_to_T(R_b_t, t_b_t)
    T_cam_board = np.array(cb["T_cam2board"], dtype=float)

    # Full chain
    T_base_cam = T_base_tool0 @ T_tool0_cam
    T_base_board = T_base_cam @ T_cam_board
    T_base_board_all.append(T_base_board)

# ---------- Check consistency ----------
positions = np.array([T[:3, 3] for T in T_base_board_all])
mean_pos = positions.mean(axis=0)
std_pos = positions.std(axis=0)

print("=== Validation results ===")
print(f"Board position mean [m]: {mean_pos}")
print(f"Board position std [m]:  {std_pos}")
print(f"Average distance spread: {np.linalg.norm(std_pos):.4f} m")

# Optional loop closure check (board as world)
T_board_base = np.linalg.inv(T_base_board_all[0])
closure = T_board_base @ T_base_board_all[-1]
rot_err = np.degrees(np.arccos(np.clip((np.trace(closure[:3,:3]) - 1) / 2, -1, 1)))
trans_err = np.linalg.norm(closure[:3,3])
print(f"Loop closure rotation error: {rot_err:.3f} deg")
print(f"Loop closure translation error: {trans_err*1000:.2f} mm")
