import open3d as o3d
import cv2
import numpy as np
import json
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# -----------------------------
# CONFIG
# -----------------------------
SESSION_DIR = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
MANIFEST_PATH = SESSION_DIR / "manifest.json"
OUTPUT_PCD = SESSION_DIR / "subset_cleaned_cloud.pcd"

NUM_SCANS = 7             # number of frames to use
VOXEL_SIZE = 0.0012       # 1.2 mm detail level
MERGE_SIZE = 0.001        # 1 mm voxel merge to remove duplicates

# workspace crop bounds (meters, base frame)
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
    color = o3d.io.read_image(str(color_path))
    depth = o3d.io.read_image(str(depth_path))
    color_np = np.asarray(color)
    depth_np = np.asarray(depth)

    if color_np.ndim == 2:
        color_np = cv2.cvtColor(color_np, cv2.COLOR_GRAY2RGB)
    elif color_np.shape[2] == 4:
        color_np = color_np[:, :, :3]

    if color_np.shape[0] != depth_np.shape[0] or color_np.shape[1] != depth_np.shape[1]:
        color_np = cv2.resize(color_np, (depth_np.shape[1], depth_np.shape[0]))

    color_o3d = o3d.geometry.Image(color_np)
    depth_o3d = o3d.geometry.Image(depth_np)

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_o3d,
        depth_o3d,
        depth_scale=1000.0,
        depth_trunc=2.0,
        convert_rgb_to_intensity=False
    )
    return rgbd

def crop_cloud(pcd):
    pts = np.asarray(pcd.points)
    mask = (
        (pts[:, 0] > X_MIN) & (pts[:, 0] < X_MAX) &
        (pts[:, 1] > Y_MIN) & (pts[:, 1] < Y_MAX) &
        (pts[:, 2] > Z_MIN) & (pts[:, 2] < Z_MAX)
    )
    return pcd.select_by_index(np.where(mask)[0])

# -----------------------------
# LOAD MANIFEST + SELECT SUBSET
# -----------------------------
with open(MANIFEST_PATH, "r") as f:
    manifest = json.load(f)
captures = manifest["captures"]
print(f"Total frames in session: {len(captures)}")

step = max(1, len(captures) // NUM_SCANS)
subset = captures[::step][:NUM_SCANS]
print(f"Using {len(subset)} frames: {[c['index'] for c in subset]}")

# -----------------------------
# FUSE SELECTED SCANS
# -----------------------------
merged = o3d.geometry.PointCloud()
for i, cap in enumerate(subset):
    idx = cap["index"]
    rgbd = load_rgbd(
        SESSION_DIR / "color_raw" / f"color_t{idx}.png",
        SESSION_DIR / "depth_raw" / f"depth_t{idx}.png"
    )
    T_base_tool = make_T_base_tool(cap["pose_tool0"])
    T_base_cam = T_base_tool @ T_tool_cam

    cloud = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, INTRINSICS)
    cloud.transform(T_base_cam)
    merged += cloud
    print(f"Added scan {i+1}/{len(subset)} index {idx}, {len(cloud.points)} pts")

# -----------------------------
# CLEANING PIPELINE
# -----------------------------
print("Cropping workspace ...")
merged = crop_cloud(merged)
print(f"After crop: {len(merged.points)} points")

print("Voxel downsampling ...")
merged = merged.voxel_down_sample(voxel_size=VOXEL_SIZE)
print(f"After voxel downsampling ({VOXEL_SIZE} m): {len(merged.points)} points")

# Plane segmentation
plane_model, inliers = merged.segment_plane(distance_threshold=0.005,
                                            ransac_n=3,
                                            num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane eq: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")
plane_cloud = merged.select_by_index(inliers)
off_plane = merged.select_by_index(inliers, invert=True)

pts_off = np.asarray(off_plane.points)
dist = np.abs(a * pts_off[:, 0] + b * pts_off[:, 1] + c * pts_off[:, 2] + d)
near_mask = dist < 0.02
near_points = off_plane.select_by_index(np.where(near_mask)[0])
merged = plane_cloud + near_points
print(f"After plane segmentation: {len(merged.points)} points")

# noise filtering
merged, _ = merged.remove_statistical_outlier(nb_neighbors=40, std_ratio=1.0)
print(f"After outlier removal: {len(merged.points)} points")

# Merge duplicates / overlaps
merged = merged.voxel_down_sample(voxel_size=MERGE_SIZE)
print(f"After voxel merging ({MERGE_SIZE} m): {len(merged.points)} points")

# -----------------------------
# Visualize cleaned point cloud
# -----------------------------
print("Displaying cleaned point cloud...")
o3d.visualization.draw_geometries([merged])

# -----------------------------
# Generate surface mesh (Ball Pivoting)
# -----------------------------
print("Reconstructing mesh with Ball Pivoting...")
radii = [VOXEL_SIZE * 3, VOXEL_SIZE * 4, VOXEL_SIZE * 5]
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
    merged,
    o3d.utility.DoubleVector(radii)
)

mesh.remove_unreferenced_vertices()
mesh.remove_degenerate_triangles()
mesh.compute_vertex_normals()

# -----------------------------
# Visualize separately
# -----------------------------
print("Displaying Ball-Pivoting mesh...")
o3d.visualization.draw_geometries([mesh])
