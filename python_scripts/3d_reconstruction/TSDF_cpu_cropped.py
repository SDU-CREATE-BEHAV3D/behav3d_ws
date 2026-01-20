# TSDF -> high-confidence surface point cloud with RGB
# PLUS confidence visualization using a conventional colormap gradient (TURBO)
#
# Adds:
# - Per-point confidence = number of valid observations (obs count)
# - Confidence gradient visualization (TURBO) as a separate point cloud
# - Cropping and outlier filtering that also keeps confidence aligned
#
# Outputs:
# - tsdf_surface_rgb_colored.ply
# - tsdf_surface_confidence_colored.ply
#
# Notes:
# - Confidence here is an observation-count proxy (robust and practical).
# - Colormap is OpenCV COLORMAP_TURBO (conventional, perceptually strong).

# set src path for utils import
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
import cv2
import open3d as o3d
import open3d.core as o3c
from scipy.spatial.transform import Rotation as R

from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses, transform_robot_to_camera_pose, construct_image_paths
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images
from utils.integration import visualize_camera_poses

session_folder = "260113_170839"
SESSION_PATH = "C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/" + session_folder
scan_folder = "manual_caps"
output_folder = Path("C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/" + session_folder)
my_session = Session(SESSION_PATH, scan_folder)

# Folder containing your color_in_depth outputs
C2D_DIR = output_folder / "alignment_test"
C2D_GLOB = "color_in_depth*.png"

# ----------------------------
# Final output filtering parameters
# ----------------------------

# Crop box in WORLD coordinates (base frame)
CROP_ENABLE = True
CROP_MIN = np.array([-0.25, -1.1, -1.0], dtype=np.float64)  # meters
CROP_MAX = np.array([ 0.3,  -0.65,  0.5], dtype=np.float64)  # meters

# Outlier removal method: "none", "statistical", "radius"
OUTLIER_METHOD = "statistical"

# Statistical outlier params
OUTLIER_NB_NEIGHBORS = 35
OUTLIER_STD_RATIO = 1.5

# Radius outlier params
OUTLIER_NB_POINTS = 12
OUTLIER_RADIUS = 0.02  # meters

# Confidence visualization params
CONF_CLIP_PERCENTILE = 100  # clip very high obs counts for better contrast
CONF_COLORMAP = cv2.COLORMAP_TURBO  # conventional gradient for confidence


class TSDF_Integration():

    def __init__(
            self,
            session,
            voxel_size=1/512,
            block_count=10000,
            block_resolution=12,
            depth_max=1.0,
            depth_scale=1000.0,
            device='CPU:0',
        ):
        # initialize session and load poses
        self.session = session
        self.manifest = read_manifest(self.session.path, self.session._scan_folder)
        self.T_base_tool0_list = load_robot_poses(self.manifest)
        self.T_tool0_ir = np.asarray(
            load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir"),
            dtype=np.float64
        )
        self.T_base_ir = [T_base_tool0 @ self.T_tool0_ir for T_base_tool0 in self.T_base_tool0_list]

        # load depth images
        self.image_paths = construct_image_paths(self.manifest, self.session, image_type="depth")
        self.images = load_images(self.image_paths, image_type="depth", library="cv2")

        # load intrinsics
        self.device = o3c.Device(device)
        self.width, self.height, self.K, self.D = load_intrinsics(self.session.depth_intrinsics_path)
        self.pinhole_matrix = intrinsics_matrix(self.width, self.height, self.K)

        # TSDF parameters
        self.voxel_size = voxel_size
        self.block_count = block_count
        self.block_resolution = block_resolution
        self.depth_max = float(depth_max)
        self.depth_scale = float(depth_scale)

        # initialize VoxelBlockGrid (CPU)
        self.vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight'),
            attr_dtypes=(o3c.float32, o3c.float32),
            attr_channels=((1), (1)),
            voxel_size=self.voxel_size,
            block_resolution=self.block_resolution,
            block_count=self.block_count,
            device=self.device
        )

        # load all color_in_depth images found in folder
        self.color_in_depth, self.color_in_depth_paths = self._load_color_in_depth_images()
        if len(self.color_in_depth) == 0:
            raise FileNotFoundError(f"No {C2D_GLOB} found in {C2D_DIR}")

    def _load_color_in_depth_images(self):
        if not C2D_DIR.exists():
            raise FileNotFoundError(f"color_in_depth folder not found: {C2D_DIR}")

        paths = sorted(C2D_DIR.glob(C2D_GLOB))
        imgs = []
        for p in paths:
            im = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if im is None:
                raise RuntimeError(f"Failed reading: {p}")
            imgs.append(im)

        print(f"Loaded {len(imgs)} color_in_depth frames from {C2D_DIR}")
        return imgs, paths

    def _tensorize_robot_poses(self):
        cpu = o3c.Device("CPU:0")
        self.T_base_ir_tensors = []
        for T in self.T_base_ir:
            self.T_base_ir_tensors.append(o3c.Tensor(T, o3c.Dtype.Float64, cpu))
        return self.T_base_ir_tensors

    def _tensorize_intrinsics(self):
        self.K_cpu = o3c.Tensor(
            np.asarray(self.pinhole_matrix.intrinsic_matrix),
            o3c.Dtype.Float64,
            self.device
        )
        return self.K_cpu

    def _tensorize_depth_images(self):
        self.depth_tensors = []
        for img in self.images:
            self.depth_tensors.append(o3c.Tensor(img, o3c.Dtype.UInt16, self.device))
        return self.depth_tensors

    def integrate_depths(self):
        self._tensorize_robot_poses()
        self._tensorize_intrinsics()
        self._tensorize_depth_images()

        for i, depth_tensor in enumerate(self.depth_tensors):
            T_base_ir_cpu = self.T_base_ir_tensors[i]
            Tcw_cpu = o3c.Tensor(
                np.linalg.inv(T_base_ir_cpu.numpy()),
                o3c.Dtype.Float64,
                self.device
            )

            depth_img_t = o3d.t.geometry.Image(depth_tensor)
            block_coords = self.vbg.compute_unique_block_coordinates(
                depth_img_t,
                self.K_cpu,
                Tcw_cpu,
                self.depth_scale,
                self.depth_max
            )

            self.vbg.integrate(
                block_coords,
                depth_img_t,
                self.K_cpu,
                Tcw_cpu,
                self.depth_scale,
                self.depth_max
            )
            print(f"Integrated depth image {i+1}/{len(self.depth_tensors)}")

        print("TSDF Integration complete.")
        return True

    def extract_tsdf_surface_point_cloud(self):
        mesh = self.vbg.extract_triangle_mesh().to_legacy()
        mesh.compute_vertex_normals()

        pts = np.asarray(mesh.vertices, dtype=np.float64)
        nrm = np.asarray(mesh.vertex_normals, dtype=np.float64)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts)
        pcd.normals = o3d.utility.Vector3dVector(nrm)
        return pcd

    def colorize_tsdf_surface_points(self, pcd, sample_step=1, z_band_m=0.02, min_obs=3):
        pts_w = np.asarray(pcd.points, dtype=np.float64)
        N = pts_w.shape[0]
        pts_h = np.concatenate([pts_w, np.ones((N, 1), dtype=np.float64)], axis=1)

        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        cx, cy = float(self.K[0, 2]), float(self.K[1, 2])
        H, W = self.images[0].shape[:2]

        n_frames = min(len(self.images), len(self.T_base_ir), len(self.color_in_depth))

        accum = np.zeros((N, 3), dtype=np.float64)
        obs = np.zeros((N,), dtype=np.int32)

        for i in range(0, n_frames, sample_step):
            T_base_ir = self.T_base_ir[i]
            T_ir_base = np.linalg.inv(T_base_ir)

            X_ir = (T_ir_base @ pts_h.T).T
            X = X_ir[:, 0]
            Y = X_ir[:, 1]
            Z = X_ir[:, 2]

            valid = Z > 0
            u = (X / Z) * fx + cx
            v = (Y / Z) * fy + cy

            ui = np.round(u).astype(np.int32)
            vi = np.round(v).astype(np.int32)

            inb = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
            valid = valid & inb
            if not np.any(valid):
                continue

            depth_m = self.images[i].astype(np.float32) / self.depth_scale
            z_obs = depth_m[vi[valid], ui[valid]]

            ok = valid.copy()
            ok[valid] = (z_obs > 0) & (z_obs < self.depth_max) & (np.abs(z_obs - Z[valid]) < z_band_m)

            if not np.any(ok):
                continue

            bgr = self.color_in_depth[i][vi[ok], ui[ok], :].astype(np.float64) / 255.0
            rgb = bgr[:, ::-1]

            accum[ok] += rgb
            obs[ok] += 1

        keep = obs >= int(min_obs)

        colors = np.zeros((N, 3), dtype=np.float64)
        colors[keep] = accum[keep] / obs[keep][:, None]

        pcd_out = o3d.geometry.PointCloud()
        pcd_out.points = o3d.utility.Vector3dVector(pts_w[keep])
        if pcd.has_normals():
            nrm = np.asarray(pcd.normals, dtype=np.float64)
            pcd_out.normals = o3d.utility.Vector3dVector(nrm[keep])
        pcd_out.colors = o3d.utility.Vector3dVector(colors[keep])

        conf = obs[keep].astype(np.float64)

        print(f"High-confidence points kept: {np.count_nonzero(keep)} / {N}  (min_obs={min_obs})")
        return pcd_out, conf

    # ----------------------------
    # Crop + outlier filtering that preserves confidence alignment
    # ----------------------------
    def crop_pcd_aabb_with_conf(self, pcd, conf, aabb_min, aabb_max):
        pts = np.asarray(pcd.points, dtype=np.float64)
        m = (
            (pts[:, 0] >= aabb_min[0]) & (pts[:, 0] <= aabb_max[0]) &
            (pts[:, 1] >= aabb_min[1]) & (pts[:, 1] <= aabb_max[1]) &
            (pts[:, 2] >= aabb_min[2]) & (pts[:, 2] <= aabb_max[2])
        )
        ind = np.where(m)[0].astype(np.int64)
        pcd2 = pcd.select_by_index(ind)
        conf2 = conf[ind]
        return pcd2, conf2

    def filter_outliers_with_conf(self, pcd, conf,
                                  method="statistical",
                                  nb_neighbors=30,
                                  std_ratio=1.5,
                                  nb_points=12,
                                  radius=0.02):
        method = (method or "none").lower().strip()
        if method == "none":
            return pcd, conf

        if method == "statistical":
            _, ind = pcd.remove_statistical_outlier(
                nb_neighbors=int(nb_neighbors),
                std_ratio=float(std_ratio)
            )
            ind = np.asarray(ind, dtype=np.int64)
            return pcd.select_by_index(ind), conf[ind]

        if method == "radius":
            _, ind = pcd.remove_radius_outlier(
                nb_points=int(nb_points),
                radius=float(radius)
            )
            ind = np.asarray(ind, dtype=np.int64)
            return pcd.select_by_index(ind), conf[ind]

        raise ValueError(f"Unknown OUTLIER_METHOD: {method}")

    # ----------------------------
    # Confidence gradient visualization (TURBO colormap)
    # ----------------------------
    def make_confidence_colored_pcd(self, pcd, conf, clip_percentile=95, cv_colormap=cv2.COLORMAP_TURBO):
        conf = conf.astype(np.float64)
        vmax = float(np.percentile(conf, clip_percentile))
        vmax = max(vmax, 1.0)

        norm = np.clip(conf / vmax, 0.0, 1.0)
        img = (norm * 255.0).astype(np.uint8).reshape(-1, 1)  # (N,1)

        bgr = cv2.applyColorMap(img, cv_colormap).reshape(-1, 3).astype(np.float64) / 255.0
        rgb = bgr[:, ::-1]

        pcd_conf = o3d.geometry.PointCloud()
        pcd_conf.points = pcd.points
        if pcd.has_normals():
            pcd_conf.normals = pcd.normals
        pcd_conf.colors = o3d.utility.Vector3dVector(rgb)
        return pcd_conf


# ---- RUN ----
tsdf_integration = TSDF_Integration(my_session)
print(f"Number of depth images loaded: {len(tsdf_integration.images)}")
print(f"Number of robot poses loaded: {len(tsdf_integration.T_base_tool0_list)}")
print(f"Number of color_in_depth loaded: {len(tsdf_integration.color_in_depth)}")

img = tsdf_integration.images[0]
print("Depth range:", np.min(img), np.max(img))

# 1) TSDF integrate (depth only)
tsdf_integration.integrate_depths()

# 2) Extract TSDF surface as point cloud
pcd_surface = tsdf_integration.extract_tsdf_surface_point_cloud()

# 3) RGB color + confidence (obs count)
pcd_rgb, conf = tsdf_integration.colorize_tsdf_surface_points(
    pcd_surface,
    sample_step=1,
    z_band_m=0.02,
    min_obs=3
)

# 4) Crop in WORLD frame (optional), keeps conf aligned
if CROP_ENABLE:
    pcd_rgb, conf = tsdf_integration.crop_pcd_aabb_with_conf(pcd_rgb, conf, CROP_MIN, CROP_MAX)
    print(f"After crop: {len(pcd_rgb.points)} points")

# 5) Outlier removal (optional), keeps conf aligned
pcd_rgb, conf = tsdf_integration.filter_outliers_with_conf(
    pcd_rgb, conf,
    method=OUTLIER_METHOD,
    nb_neighbors=OUTLIER_NB_NEIGHBORS,
    std_ratio=OUTLIER_STD_RATIO,
    nb_points=OUTLIER_NB_POINTS,
    radius=OUTLIER_RADIUS
)
print(f"After outlier filter ({OUTLIER_METHOD}): {len(pcd_rgb.points)} points")

# 6) Build confidence-colored visualization point cloud
pcd_conf = tsdf_integration.make_confidence_colored_pcd(
    pcd_rgb, conf,
    clip_percentile=CONF_CLIP_PERCENTILE,
    cv_colormap=CONF_COLORMAP
)

# 7) Visualize
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

print("Visualizing RGB-colored TSDF surface point cloud")
o3d.visualization.draw([pcd_rgb, axes])

print("Visualizing CONFIDENCE gradient (TURBO) for TSDF surface point cloud")
o3d.visualization.draw([pcd_conf, axes])

# 8) Save outputs
out_rgb = output_folder / "tsdf_surface_rgb_colored.ply"
out_conf = output_folder / "tsdf_surface_confidence_colored.ply"
o3d.io.write_point_cloud(str(out_rgb), pcd_rgb)
# o3d.io.write_point_cloud(str(out_conf), pcd_conf)
print("Saved:", out_rgb)
# print("Saved:", out_conf)

# mesh part intentionally commented out
# mesh = tsdf_integration.vbg.extract_triangle_mesh().to_legacy()
# mesh.compute_vertex_normals()
# o3d.io.write_triangle_mesh(str(output_folder / "tsdf_mesh.stl"), mesh)
