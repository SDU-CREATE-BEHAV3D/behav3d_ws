import open3d as o3d
import numpy as np
import json, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# -----------------------------
# CONFIG
# -----------------------------
SESSION_DIR = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
MANIFEST_PATH = SESSION_DIR / "manifest.json"

NUM_SCANS = 5             # number of frames to use
VOXEL_SIZE = 0.0022       # ~1.4 mm grid
SDF_TRUNC = 0.2          # truncation distance (keep wide band for smooth fusion)

# workspace bounding box (in base frame, meters)
X_MIN, X_MAX = -0.3, 1.9
Y_MIN, Y_MAX = -1.7, 0.6
Z_MIN, Z_MAX = -2.0, 0.4

# -----------------------------
# CAMERA + HAND-EYE CALIBRATION
# -----------------------------
INTRINSICS = o3d.camera.PinholeCameraIntrinsic(
    640, 576,
    505.085205078125, 505.0267028808594,
    337.96063232421875, 338.32763671875
)

r_tool_cam = R.from_euler('xyz', [1.589, -0.00152, 3.111])
T_tool_cam = np.eye(4)
T_tool_cam[:3, :3] = r_tool_cam.as_matrix()
T_tool_cam[:3, 3] = [-0.0445562, 0.17385712, -0.07569992]

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
    return color_cv, depth_np

# -----------------------------
# LOAD MANIFEST + SELECT SUBSET
# -----------------------------
with open(MANIFEST_PATH, "r") as f:
    manifest = json.load(f)
captures = manifest["captures"]

total_frames = len(captures)
step = max(1, total_frames // NUM_SCANS)
subset = captures[::step][:NUM_SCANS]
print(f"Loaded {total_frames} frames, using {len(subset)} for TSDF fusion: {[c['index'] for c in subset]}")

# -----------------------------
# TSDF SETUP
# -----------------------------
tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
    voxel_length=VOXEL_SIZE,
    sdf_trunc=SDF_TRUNC,
    color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
)

fx, fy = INTRINSICS.get_focal_length()
cx, cy = INTRINSICS.get_principal_point()

# -----------------------------
# FUSION LOOP
# -----------------------------
for i, cap in enumerate(subset):
    idx = cap["index"]
    color_cv, depth_np = load_rgbd(
        SESSION_DIR / "color_raw" / f"color_t{idx}.png",
        SESSION_DIR / "depth_raw" / f"depth_t{idx}.png"
    )

    T_base_tool = make_T_base_tool(cap["pose_tool0"])
    T_base_cam = T_base_tool @ T_tool_cam

    # Project to base frame for cropping
    h, w = depth_np.shape
    ys, xs = np.meshgrid(np.arange(h), np.arange(w), indexing="ij")
    z = depth_np / 1000.0  # mm → m
    valid = z > 0
    x = (xs - cx) * z / fx
    y = (ys - cy) * z / fy
    pts_cam = np.stack((x, y, z), axis=-1).reshape(-1, 3)
    pts_cam = pts_cam[valid.reshape(-1)]
    pts_base = (T_base_cam[:3, :3] @ pts_cam.T + T_base_cam[:3, [3]]).T

    # Mask out-of-bounds points
    inside = (
        (pts_base[:, 0] > X_MIN) & (pts_base[:, 0] < X_MAX) &
        (pts_base[:, 1] > Y_MIN) & (pts_base[:, 1] < Y_MAX) &
        (pts_base[:, 2] > Z_MIN) & (pts_base[:, 2] < Z_MAX)
    )

    # Rebuild cropped depth
    depth_crop = np.zeros_like(depth_np)
    valid_idx = np.where(valid.reshape(-1))[0][inside]
    depth_flat = depth_np.reshape(-1)
    depth_crop.reshape(-1)[valid_idx] = depth_flat[valid_idx]

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_cv),
        o3d.geometry.Image(depth_crop),
        depth_scale=1000.0, depth_trunc=2.0,
        convert_rgb_to_intensity=False
    )

    tsdf.integrate(rgbd, INTRINSICS, np.linalg.inv(T_base_cam))
    if (i + 1) % 2 == 0 or i == len(subset) - 1:
        print(f"Integrated {i+1}/{len(subset)} scans")

# -----------------------------
# EXTRACT RESULTS
# -----------------------------
print("Extracting fused point cloud...")
pcd = tsdf.extract_point_cloud()
pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=2.0)
print(f"✅ Extracted {len(pcd.points)} points after fusion")

o3d.visualization.draw_geometries([pcd])
