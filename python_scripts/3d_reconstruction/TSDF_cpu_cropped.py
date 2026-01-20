# ADD-ON: cropping box + outlier filtering parameters for the final TSDF surface point cloud
# This is your last script with two extra controllable stages:
#   1) Crop by axis-aligned bounding box in WORLD coordinates
#   2) Remove outliers (statistical or radius) with tunable parameters
#
# Where to change parameters:
#   CROP_ENABLE, CROP_MIN, CROP_MAX
#   OUTLIER_METHOD, OUTLIER_* params

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


SESSION_PATH = "C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/260113_170839"
scan_folder = "manual_caps"
output_folder = Path("C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/260113_170839")
my_session = Session(SESSION_PATH, scan_folder)

# Folder containing your color_in_depth outputs
C2D_DIR = output_folder / "alignment_test"
C2D_GLOB = "color_in_depth*.png"

# ----------------------------
# NEW: Final output filtering parameters
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
        self.T_tool0_ir = np.asarray(load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir"),
                                     dtype=np.float64)
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

        colors = np.zeros((N, 3), dtype=np.float64)
        keep = obs >= int(min_obs)
        colors[keep] = accum[keep] / obs[keep][:, None]

        pcd_out = o3d.geometry.PointCloud()
        pcd_out.points = o3d.utility.Vector3dVector(pts_w[keep])
        if pcd.has_normals():
            nrm = np.asarray(pcd.normals, dtype=np.float64)
            pcd_out.normals = o3d.utility.Vector3dVector(nrm[keep])
        pcd_out.colors = o3d.utility.Vector3dVector(colors[keep])

        print(f"High-confidence points kept: {np.count_nonzero(keep)} / {N}  (min_obs={min_obs})")
        return pcd_out

    # ----------------------------
    # NEW: crop + outlier filtering helpers
    # ----------------------------
    def crop_pcd_aabb(self, pcd, aabb_min, aabb_max):
        aabb = o3d.geometry.AxisAlignedBoundingBox(aabb_min, aabb_max)
        return pcd.crop(aabb)

    def filter_outliers(self, pcd,
                        method="statistical",
                        nb_neighbors=30,
                        std_ratio=1.5,
                        nb_points=12,
                        radius=0.02):
        method = (method or "none").lower().strip()
        if method == "none":
            return pcd

        if method == "statistical":
            cl, ind = pcd.remove_statistical_outlier(nb_neighbors=int(nb_neighbors),
                                                     std_ratio=float(std_ratio))
            return pcd.select_by_index(ind)

        if method == "radius":
            cl, ind = pcd.remove_radius_outlier(nb_points=int(nb_points),
                                                radius=float(radius))
            return pcd.select_by_index(ind)

        raise ValueError(f"Unknown OUTLIER_METHOD: {method}")


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

# 3) Color + confidence filter
pcd_surface_colored = tsdf_integration.colorize_tsdf_surface_points(
    pcd_surface,
    sample_step=1,
    z_band_m=0.02,
    min_obs=3
)

# 4) Crop in WORLD frame (optional)
if CROP_ENABLE:
    pcd_surface_colored = tsdf_integration.crop_pcd_aabb(pcd_surface_colored, CROP_MIN, CROP_MAX)
    print(f"After crop: {len(pcd_surface_colored.points)} points")

# 5) Outlier removal (optional)
pcd_surface_colored = tsdf_integration.filter_outliers(
    pcd_surface_colored,
    method=OUTLIER_METHOD,
    nb_neighbors=OUTLIER_NB_NEIGHBORS,
    std_ratio=OUTLIER_STD_RATIO,
    nb_points=OUTLIER_NB_POINTS,
    radius=OUTLIER_RADIUS
)
print(f"After outlier filter ({OUTLIER_METHOD}): {len(pcd_surface_colored.points)} points")

# 6) Visualize + save
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
o3d.visualization.draw([pcd_surface_colored, axes])

out_ply = output_folder / "tsdf_surface_colored_highconf_cropped_filtered.ply"
o3d.io.write_point_cloud(str(out_ply), pcd_surface_colored)
print("Saved:", out_ply)

# mesh part intentionally commented out
# mesh = tsdf_integration.vbg.extract_triangle_mesh().to_legacy()
# mesh.compute_vertex_normals()
# o3d.io.write_triangle_mesh(str(output_folder / "tsdf_mesh.stl"), mesh)
