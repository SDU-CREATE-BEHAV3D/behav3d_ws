import open3d as o3d
import numpy as np
from pathlib import Path

# -----------------------------
# CONFIG
# -----------------------------
SESSION_DIR = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
INPUT_FILE = SESSION_DIR / "aligned_fused_cloud.pcd"
OUTPUT_FILE = SESSION_DIR / "aligned_cleaned_cloud.pcd"

# -----------------------------
# LOAD CLOUD
# -----------------------------
pcd = o3d.io.read_point_cloud(str(INPUT_FILE))
print(f"Loaded cloud with {len(pcd.points)} points")

# -----------------------------
# 1️⃣ Rough bounding box crop
# -----------------------------
# Adjust these bounds to your workspace (in meters, base frame)
points = np.asarray(pcd.points)
x_min, x_max = -0.3, 1.9
y_min, y_max = -1.7, 0.6
z_min, z_max = -2.0, 0.4

mask = (
    (points[:, 0] > x_min) & (points[:, 0] < x_max) &
    (points[:, 1] > y_min) & (points[:, 1] < y_max) &
    (points[:, 2] > z_min) & (points[:, 2] < z_max)
)
pcd = pcd.select_by_index(np.where(mask)[0])
print(f"After bounding crop: {len(pcd.points)} points")

# -----------------------------
# 2️⃣ Plane segmentation (keep plane + near points)
# -----------------------------
plane_model, inliers = pcd.segment_plane(distance_threshold=0.005,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Plane eq: {a:.3f}x + {b:.3f}y + {c:.3f}z + {d:.3f} = 0")

plane_cloud = pcd.select_by_index(inliers)
off_plane = pcd.select_by_index(inliers, invert=True)

# Keep only points close to the plane (±2 cm)
points_off = np.asarray(off_plane.points)
dist_to_plane = np.abs(a * points_off[:, 0] + b * points_off[:, 1] + c * points_off[:, 2] + d)
near_mask = dist_to_plane < 0.02
near_points = off_plane.select_by_index(np.where(near_mask)[0])

pcd = plane_cloud + near_points
print(f"After plane segmentation: {len(pcd.points)} points")

# -----------------------------
# 3️⃣ Statistical outlier removal
# -----------------------------
pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=30, std_ratio=1.5)
print(f"After noise filtering: {len(pcd.points)} points")

# -----------------------------
# SAVE + VISUALIZE
# -----------------------------
o3d.io.write_point_cloud(str(OUTPUT_FILE), pcd)
print(f"✅ Cleaned cloud saved: {OUTPUT_FILE}")

o3d.visualization.draw_geometries([pcd])
