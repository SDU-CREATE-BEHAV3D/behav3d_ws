import open3d as o3d
import numpy as np
import json, random, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# -----------------------------
# CONFIG
# -----------------------------
SESSION_DIR = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
MANIFEST_PATH = SESSION_DIR / "manifest.json"

# Depth intrinsics (from depth_intrinsics.yaml)
INTRINSICS = o3d.camera.PinholeCameraIntrinsic(
    640, 576, 505.085205078125, 505.0267028808594, 337.96063232421875, 338.32763671875
)

# -----------------------------
# HAND–EYE TRANSFORM (tool0 -> camera)
# -----------------------------
r_tool_cam = R.from_euler('xyz', [1.589, -0.00152, 3.111])
T_tool_cam = np.eye(4)
T_tool_cam[:3, :3] = r_tool_cam.as_matrix()
T_tool_cam[:3, 3] = [-0.0445562, 0.17385712, -0.07569992]

T_cam_tool = np.linalg.inv(T_tool_cam)

# Toggle between them for debugging
USE_INVERTED = False
print(f"Using {'INVERTED' if USE_INVERTED else 'DIRECT'} hand–eye transform")

# -----------------------------
# LOAD MANIFEST
# -----------------------------
with open(MANIFEST_PATH, "r") as f:
    manifest = json.load(f)
captures = manifest["captures"]
selected = random.sample(captures, 3)

# -----------------------------
# HELPERS
# -----------------------------
def quat_to_matrix(q):
    return R.from_quat(q).as_matrix()

def make_T_base_tool(pose):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(pose["orientation_xyzw"])
    T[:3, 3] = pose["position"]
    return T

def load_rgbd(color_path, depth_path):
    depth = o3d.io.read_image(str(depth_path))
    depth_np = np.asarray(depth)
    h, w = depth_np.shape
    color_cv = cv2.imread(str(color_path))
    color_cv = cv2.cvtColor(color_cv, cv2.COLOR_BGR2RGB)
    color_cv = cv2.resize(color_cv, (w, h), interpolation=cv2.INTER_AREA)
    color = o3d.geometry.Image(color_cv)
    depth_o3d = o3d.geometry.Image(depth_np)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth_o3d,
        depth_scale=1000.0,
        depth_trunc=2.0,
        convert_rgb_to_intensity=False
    )
    return rgbd

def draw_frame(T, size=0.05):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    frame.transform(T)
    return frame

# -----------------------------
# BUILD CLOUDS
# -----------------------------
clouds, frames = [], []
for cap in selected:
    idx = cap["index"]
    color_path = SESSION_DIR / "color_raw" / f"color_t{idx}.png"
    depth_path = SESSION_DIR / "depth_raw" / f"depth_t{idx}.png"
    T_base_tool = make_T_base_tool(cap["pose_tool0"])
    rgbd = load_rgbd(color_path, depth_path)

    T_base_cam = T_base_tool @ (T_cam_tool if USE_INVERTED else T_tool_cam)

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, INTRINSICS)
    pcd.transform(T_base_cam)
    clouds.append(pcd)
    frames.append(draw_frame(T_base_cam, size=0.05))

# -----------------------------
# VISUALIZE
# -----------------------------
print("Displaying colored point clouds with camera frames...")
o3d.visualization.draw_geometries(clouds + frames)
