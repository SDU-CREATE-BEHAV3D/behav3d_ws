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
import json
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
SESSION_PATH = "/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/" + session_folder
scan_folder = "manual_caps"
output_folder = Path("/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/" + session_folder)
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

# ----------------------------
# Table plane extraction + slicing (for removing the horizontal table)
# ----------------------------

# Plane mode: "fit" (estimate + save), "load" (reuse), "off"
TABLE_PLANE_MODE = "fit"
TABLE_PLANE_FILE = output_folder / "table_plane.json"

# RANSAC params for plane fitting
TABLE_PLANE_DOWNSAMPLE = 0.005  # meters, 0 to disable
TABLE_PLANE_RANSAC_THRESH = 0.004
TABLE_PLANE_RANSAC_N = 3
TABLE_PLANE_RANSAC_ITERS = 2000

# Orientation hint (used to keep "above" consistent)
TABLE_PLANE_UP_AXIS = np.array([0.0, 0.0, 1.0], dtype=np.float64)
TABLE_PLANE_MAX_TILT_DEG = 25.0  # warn if plane is far from up

# Slice points relative to the plane (leave False to keep all TSDF points)
TABLE_SLICE_ENABLE = False
TABLE_SLICE_KEEP_SIDE = "above"  # "above" or "below"
TABLE_SLICE_MARGIN = 0.005  # meters

# Plane visualization
TABLE_PLANE_VIS_ENABLE = True
TABLE_PLANE_VIS_COLOR = (0.2, 0.4, 1.0)
TABLE_PLANE_VIS_SCALE = 1.2
TABLE_PLANE_VIS_GRID = 25 # number of inner grid lines per axis
TABLE_PLANE_NORMAL_COLOR = (1.0, 0.2, 0.2)
TABLE_PLANE_NORMAL_SCALE = 0.25
TABLE_PLANE_NORMAL_RADIUS = 0.002  # meters; 0 uses a thin line


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
        if conf is None or len(conf) == 0 or len(pcd.points) == 0:
            return pcd
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


def _normalize_plane(plane_model):
    plane_model = np.asarray(plane_model, dtype=np.float64).reshape(4)
    n = plane_model[:3]
    norm = np.linalg.norm(n)
    if norm == 0:
        raise ValueError("Plane normal is zero")
    return plane_model / norm


def _fit_table_plane(pcd, distance_threshold, ransac_n, num_iterations,
                     downsample=0.0, up_axis=None, max_tilt_deg=None):
    pcd_fit = pcd
    if downsample and downsample > 0:
        pcd_fit = pcd.voxel_down_sample(float(downsample))

    plane_model, inliers = pcd_fit.segment_plane(
        distance_threshold=float(distance_threshold),
        ransac_n=int(ransac_n),
        num_iterations=int(num_iterations)
    )
    plane_model = _normalize_plane(plane_model)

    n = plane_model[:3]
    if up_axis is not None:
        up_axis = np.asarray(up_axis, dtype=np.float64)
        up_norm = np.linalg.norm(up_axis)
        if up_norm > 0:
            up_axis = up_axis / up_norm
            if np.dot(n, up_axis) < 0:
                plane_model = -plane_model
                n = -n
            if max_tilt_deg is not None:
                ang = np.degrees(np.arccos(np.clip(np.dot(n, up_axis), -1.0, 1.0)))
                if ang > float(max_tilt_deg):
                    print(f"Warning: plane tilt {ang:.1f} deg exceeds {max_tilt_deg} deg")

    print(f"Table plane model (a,b,c,d): {plane_model.tolist()}")
    print(f"Table plane inliers: {len(inliers)} / {len(pcd_fit.points)}")
    return plane_model


def _save_plane(path, plane_model):
    payload = {"plane_model": plane_model.tolist()}
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2))


def _load_plane(path):
    data = json.loads(path.read_text())
    plane = np.asarray(data["plane_model"], dtype=np.float64)
    return _normalize_plane(plane)


def _slice_pcd_by_plane(pcd, conf, plane_model, keep_side="above", margin=0.0):
    plane_model = _normalize_plane(plane_model)
    n = plane_model[:3]
    d = plane_model[3]

    pts = np.asarray(pcd.points, dtype=np.float64)
    dist = pts @ n + d

    keep_side = (keep_side or "above").lower().strip()
    margin = float(margin)
    if keep_side == "above":
        m = dist > margin
    elif keep_side == "below":
        m = dist < -margin
    else:
        raise ValueError(f"Unknown TABLE_SLICE_KEEP_SIDE: {keep_side}")

    ind = np.where(m)[0].astype(np.int64)
    pcd2 = pcd.select_by_index(ind)
    conf2 = conf[ind] if conf is not None else None
    return pcd2, conf2


def _make_plane_line_set(plane_model, pcd_ref, color=(0.85, 0.85, 0.85), scale=1.2,
                         normal_color=(1.0, 0.2, 0.2), normal_scale=0.25,
                         normal_radius=0.0, grid_lines=0):
    plane_model = _normalize_plane(plane_model)
    n = plane_model[:3]
    d = plane_model[3]

    bbox = pcd_ref.get_axis_aligned_bounding_box()
    center = np.asarray(bbox.get_center(), dtype=np.float64)
    center = center - (np.dot(n, center) + d) * n

    ext = np.asarray(bbox.get_extent(), dtype=np.float64)
    size = float(np.max(ext)) * float(scale)

    ref = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    if abs(np.dot(n, ref)) > 0.9:
        ref = np.array([1.0, 0.0, 0.0], dtype=np.float64)

    t1 = np.cross(n, ref)
    t1 = t1 / max(np.linalg.norm(t1), 1e-9)
    t2 = np.cross(n, t1)
    t2 = t2 / max(np.linalg.norm(t2), 1e-9)

    half = 0.5 * size
    corners = np.array([
        center + ( t1 * half) + ( t2 * half),
        center + (-t1 * half) + ( t2 * half),
        center + (-t1 * half) + (-t2 * half),
        center + ( t1 * half) + (-t2 * half),
    ], dtype=np.float64)

    points = [c for c in corners]
    lines = [[0, 1], [1, 2], [2, 3], [3, 0]]

    grid_lines = int(max(0, grid_lines))
    if grid_lines > 0:
        for i in range(1, grid_lines + 1):
            offset = -half + (2.0 * half) * (i / (grid_lines + 1))

            p_a = center + (t1 * offset) + (t2 * half)
            p_b = center + (t1 * offset) - (t2 * half)
            idx = len(points)
            points.extend([p_a, p_b])
            lines.append([idx, idx + 1])

            p_c = center + (t2 * offset) + (t1 * half)
            p_d = center + (t2 * offset) - (t1 * half)
            idx = len(points)
            points.extend([p_c, p_d])
            lines.append([idx, idx + 1])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.vstack(points))
    line_set.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    line_set.colors = o3d.utility.Vector3dVector(
        np.tile(np.array(color, dtype=np.float64), (len(lines), 1))
    )

    normal_len = size * float(normal_scale)
    if float(normal_radius) > 0:
        # Cylinder along +Z, centered at origin; rotate to normal and translate.
        cyl = o3d.geometry.TriangleMesh.create_cylinder(
            radius=float(normal_radius),
            height=float(normal_len),
            resolution=20,
            split=4
        )
        cyl.paint_uniform_color(normal_color)
        z = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        dot = float(np.clip(np.dot(z, n), -1.0, 1.0))
        if dot < 0.999999:
            if dot > -0.999999:
                axis = np.cross(z, n)
                axis = axis / max(np.linalg.norm(axis), 1e-9)
                angle = float(np.arccos(dot))
                R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
                cyl.rotate(R, center=(0.0, 0.0, 0.0))
            else:
                R = o3d.geometry.get_rotation_matrix_from_axis_angle(
                    np.array([1.0, 0.0, 0.0]) * np.pi
                )
                cyl.rotate(R, center=(0.0, 0.0, 0.0))
        cyl.translate(center + n * (0.5 * normal_len))
        return [line_set, cyl]

    p0 = center
    p1 = center + n * normal_len
    normal = o3d.geometry.LineSet()
    normal.points = o3d.utility.Vector3dVector(np.vstack([p0, p1]))
    normal.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
    normal.colors = o3d.utility.Vector3dVector(
        np.array([normal_color], dtype=np.float64)
    )
    return [line_set, normal]


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

# 4.5) Fit/load table plane using cropped points
plane_model = None
plane_vis = None
if TABLE_PLANE_MODE != "off":
    mode = TABLE_PLANE_MODE.lower().strip()
    if mode == "fit":
        plane_model = _fit_table_plane(
            pcd_rgb,
            distance_threshold=TABLE_PLANE_RANSAC_THRESH,
            ransac_n=TABLE_PLANE_RANSAC_N,
            num_iterations=TABLE_PLANE_RANSAC_ITERS,
            downsample=TABLE_PLANE_DOWNSAMPLE,
            up_axis=TABLE_PLANE_UP_AXIS,
            max_tilt_deg=TABLE_PLANE_MAX_TILT_DEG
        )
        _save_plane(TABLE_PLANE_FILE, plane_model)
        print(f"Saved table plane: {TABLE_PLANE_FILE}")
    elif mode == "load":
        if not TABLE_PLANE_FILE.exists():
            raise FileNotFoundError(f"Table plane file not found: {TABLE_PLANE_FILE}")
        plane_model = _load_plane(TABLE_PLANE_FILE)
        print(f"Loaded table plane: {TABLE_PLANE_FILE}")
    else:
        raise ValueError(f"Unknown TABLE_PLANE_MODE: {TABLE_PLANE_MODE}")

    if TABLE_PLANE_VIS_ENABLE:
        plane_vis = _make_plane_line_set(
            plane_model,
            pcd_rgb,
            color=TABLE_PLANE_VIS_COLOR,
            scale=TABLE_PLANE_VIS_SCALE,
            normal_color=TABLE_PLANE_NORMAL_COLOR,
            normal_scale=TABLE_PLANE_NORMAL_SCALE,
            normal_radius=TABLE_PLANE_NORMAL_RADIUS,
            grid_lines=TABLE_PLANE_VIS_GRID
        )

    if TABLE_SLICE_ENABLE:
        pcd_rgb, conf = _slice_pcd_by_plane(
            pcd_rgb, conf, plane_model,
            keep_side=TABLE_SLICE_KEEP_SIDE,
            margin=TABLE_SLICE_MARGIN
        )
        print(f"After table slice ({TABLE_SLICE_KEEP_SIDE}): {len(pcd_rgb.points)} points")

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

def _draw_geoms(geoms):
    # Open3D < 0.18 lacks visualization.draw
    if hasattr(o3d.visualization, "draw"):
        o3d.visualization.draw(geoms)
    else:
        o3d.visualization.draw_geometries(geoms)

print("Visualizing RGB-colored TSDF surface point cloud")
geoms_rgb = [pcd_rgb, axes]
if plane_vis is not None:
    geoms_rgb.extend(plane_vis)
_draw_geoms(geoms_rgb)

print("Visualizing CONFIDENCE gradient (TURBO) for TSDF surface point cloud")
geoms_conf = [pcd_conf, axes]
if plane_vis is not None:
    geoms_conf.extend(plane_vis)
_draw_geoms(geoms_conf)

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
