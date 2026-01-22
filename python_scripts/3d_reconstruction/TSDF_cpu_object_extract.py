# TSDF -> cropped + plane-filtered surface point cloud
# Focus: crop and remove points below a known table plane (with offset).

# set src path for utils import
import sys
import json
from pathlib import Path

import numpy as np
import cv2
import open3d as o3d
import open3d.core as o3c

sys.path.append(str(Path(__file__).resolve().parents[1]))

from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses, construct_image_paths
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images

session_folder = "260114_112631"
SESSION_PATH = "/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/" + session_folder
scan_folder = "manual_caps"
output_folder = Path("/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/" + session_folder)
my_session = Session(SESSION_PATH, scan_folder)

# Folder containing your color_in_depth outputs
C2D_DIR = output_folder / "alignment_test"
C2D_GLOB = "color_in_depth*.png"

# ----------------------------
# Crop (WORLD/base frame)
# ----------------------------
CROP_ENABLE = True
CROP_MIN = np.array([-0.25, -1.1, -1.0], dtype=np.float64)  # meters
CROP_MAX = np.array([0.3, -0.65, 0.5], dtype=np.float64)    # meters

# ----------------------------
# Table plane slicing (WORLD/base frame)
# ----------------------------
TABLE_PLANE_FILE = output_folder / "table_plane.json"
TABLE_PLANE_UP_AXIS = np.array([0.0, 0.0, 1.0], dtype=np.float64)
TABLE_PLANE_OFFSET_M = 0.005  # meters (0.5 cm upward)
TABLE_PLANE_KEEP_SIDE = "above"  # keep points above the plane
TABLE_PLANE_MARGIN = 0.0

# Visualization
PLANE_VIS_ENABLE = True
PLANE_VIS_COLOR = (0.2, 0.4, 1.0)
PLANE_VIS_SCALE = 1.2
PLANE_VIS_GRID = 25
PLANE_NORMAL_COLOR = (1.0, 0.2, 0.2)
PLANE_NORMAL_SCALE = 0.25
PLANE_NORMAL_RADIUS = 0.002

# Confidence visualization
CONF_CLIP_PERCENTILE = 100
CONF_COLORMAP = cv2.COLORMAP_TURBO

# Save options
SAVE_CONF = False


class TSDF_Integration:
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

        # load color images (only used for colorize output)
        self.color_paths = construct_image_paths(self.manifest, self.session, image_type="color")
        self.color_images = load_images(self.color_paths, image_type="color", library="cv2")

        # load color extrinsics/intrinsics (for color->depth mapping)
        self.T_tool0_color = np.asarray(
            load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_color"),
            dtype=np.float64
        )
        self.T_color_from_ir = np.linalg.inv(self.T_tool0_color) @ self.T_tool0_ir

        # load intrinsics
        self.device = o3c.Device(device)
        self.width, self.height, self.K, self.D = load_intrinsics(self.session.depth_intrinsics_path)
        self.pinhole_matrix = intrinsics_matrix(self.width, self.height, self.K)
        self.wc, self.hc, self.Kc, self.Dc = load_intrinsics(self.session.color_intrinsics_path)

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

    def make_confidence_colored_pcd(self, pcd, conf, clip_percentile=95, cv_colormap=cv2.COLORMAP_TURBO):
        if conf is None or len(conf) == 0 or len(pcd.points) == 0:
            return pcd
        if len(conf) != len(pcd.points):
            print(
                f"Warning: confidence length ({len(conf)}) "
                f"does not match points ({len(pcd.points)}); skipping conf coloring"
            )
            return pcd

        vmax = float(np.percentile(conf, clip_percentile)) if clip_percentile < 100 else float(conf.max())
        vmax = max(vmax, 1.0)
        vals = np.clip(conf / vmax, 0.0, 1.0)
        vals_u8 = (vals * 255.0).astype(np.uint8).reshape(-1, 1)
        colored = cv2.applyColorMap(vals_u8, cv_colormap)
        rgb = colored.reshape(-1, 3)[:, ::-1].astype(np.float64) / 255.0

        pcd_conf = o3d.geometry.PointCloud()
        pcd_conf.points = pcd.points
        if pcd.has_normals():
            pcd_conf.normals = pcd.normals
        pcd_conf.colors = o3d.utility.Vector3dVector(rgb)
        return pcd_conf


def _normalize_plane(plane_model):
    plane_model = np.asarray(plane_model, dtype=np.float64).reshape(4)
    n = plane_model[:3]
    norm = float(np.linalg.norm(n))
    if norm < 1e-12:
        raise ValueError("Invalid plane normal")
    return plane_model / norm


def _load_plane(path):
    data = json.loads(Path(path).read_text())
    plane = np.asarray(data["plane_model"], dtype=np.float64)
    return _normalize_plane(plane)


def _offset_plane_model(plane_model, offset_m, up_axis=None):
    plane = _normalize_plane(plane_model)
    if up_axis is not None:
        up = np.asarray(up_axis, dtype=np.float64).reshape(3)
        if np.dot(plane[:3], up) < 0:
            plane = -plane
    plane[3] -= float(offset_m)
    return plane


def _slice_pcd_by_plane(pcd, conf, plane_model, keep_side="above", margin=0.0):
    plane_model = _normalize_plane(plane_model)
    n = plane_model[:3]
    d = plane_model[3]

    pts = np.asarray(pcd.points, dtype=np.float64)
    dist = pts @ n + d

    keep_side = (keep_side or "above").lower().strip()
    margin = float(margin)
    if keep_side == "above":
        keep = dist >= margin
    elif keep_side == "below":
        keep = dist <= -margin
    else:
        keep = np.abs(dist) >= margin

    ind = np.where(keep)[0].astype(np.int64)
    pcd2 = pcd.select_by_index(ind)
    conf2 = conf[ind]
    return pcd2, conf2


def _make_plane_line_set(plane_model, pcd_ref, color=(0.85, 0.85, 0.85), scale=1.2,
                         grid_lines=15, normal_color=(1.0, 0.0, 0.0),
                         normal_scale=0.2, normal_radius=0.0):
    plane_model = _normalize_plane(plane_model)
    n = plane_model[:3]
    d = plane_model[3]

    bbox = pcd_ref.get_axis_aligned_bounding_box()
    center = bbox.get_center()
    extent = bbox.get_extent() * float(scale)

    u = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if abs(np.dot(u, n)) > 0.9:
        u = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    u = u - np.dot(u, n) * n
    u /= np.linalg.norm(u)
    v = np.cross(n, u)

    plane_center = center - (np.dot(center, n) + d) * n

    half_u = 0.5 * extent[0]
    half_v = 0.5 * extent[1]

    corners = [
        plane_center + u * half_u + v * half_v,
        plane_center + u * half_u - v * half_v,
        plane_center - u * half_u - v * half_v,
        plane_center - u * half_u + v * half_v,
    ]

    points = np.array(corners, dtype=np.float64)
    lines = [[0, 1], [1, 2], [2, 3], [3, 0]]

    if int(grid_lines) > 0:
        grid_lines = int(grid_lines)
        for i in range(1, grid_lines):
            t = i / float(grid_lines)
            a = plane_center + u * half_u + v * (half_v * (1 - 2 * t))
            b = plane_center - u * half_u + v * (half_v * (1 - 2 * t))
            c = plane_center + v * half_v + u * (half_u * (1 - 2 * t))
            dpt = plane_center - v * half_v + u * (half_u * (1 - 2 * t))
            points = np.vstack([points, a, b, c, dpt])
            idx = points.shape[0] - 4
            lines.extend([[idx, idx + 1], [idx + 2, idx + 3]])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines, dtype=np.int32))
    line_set.colors = o3d.utility.Vector3dVector(
        np.tile(np.array(color, dtype=np.float64), (len(lines), 1))
    )

    if normal_radius > 0.0:
        cyl = o3d.geometry.TriangleMesh.create_cylinder(radius=normal_radius, height=normal_scale)
        cyl.paint_uniform_color(normal_color)
        z_axis = np.array([0.0, 0.0, 1.0])
        axis = np.cross(z_axis, n)
        if np.linalg.norm(axis) > 1e-6:
            axis = axis / np.linalg.norm(axis)
            angle = np.arccos(np.clip(np.dot(z_axis, n), -1.0, 1.0))
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            cyl.rotate(R, center=(0.0, 0.0, 0.0))
        cyl.translate(plane_center + n * (0.5 * normal_scale))
        return [line_set, cyl]

    p0 = plane_center
    p1 = plane_center + n * normal_scale
    normal = o3d.geometry.LineSet()
    normal.points = o3d.utility.Vector3dVector(np.vstack([p0, p1]))
    normal.lines = o3d.utility.Vector2iVector(np.array([[0, 1]], dtype=np.int32))
    normal.colors = o3d.utility.Vector3dVector(
        np.array([normal_color], dtype=np.float64)
    )
    return [line_set, normal]


def _draw_geoms(geoms):
    # Open3D < 0.18 lacks visualization.draw
    if hasattr(o3d.visualization, "draw"):
        o3d.visualization.draw(geoms)
    else:
        o3d.visualization.draw_geometries(geoms)


# ---- RUN ----
tsdf_integration = TSDF_Integration(my_session)
print(f"Number of depth images loaded: {len(tsdf_integration.images)}")
print(f"Number of robot poses loaded: {len(tsdf_integration.T_base_tool0_list)}")
print(f"Number of color images loaded: {len(tsdf_integration.color_images)}")
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

# 4) Crop in WORLD frame
if CROP_ENABLE:
    pcd_rgb, conf = tsdf_integration.crop_pcd_aabb_with_conf(pcd_rgb, conf, CROP_MIN, CROP_MAX)
    print(f"After crop: {len(pcd_rgb.points)} points")

# 5) Remove points below the offset table plane
if not TABLE_PLANE_FILE.exists():
    raise FileNotFoundError(f"Table plane file not found: {TABLE_PLANE_FILE}")
plane_model = _load_plane(TABLE_PLANE_FILE)
plane_model = _offset_plane_model(plane_model, TABLE_PLANE_OFFSET_M, up_axis=TABLE_PLANE_UP_AXIS)
print(f"Loaded table plane: {TABLE_PLANE_FILE}")
print(f"Using plane offset: {TABLE_PLANE_OFFSET_M:.4f} m")

pcd_rgb, conf = _slice_pcd_by_plane(
    pcd_rgb,
    conf,
    plane_model,
    keep_side=TABLE_PLANE_KEEP_SIDE,
    margin=TABLE_PLANE_MARGIN
)
print(f"After table slice ({TABLE_PLANE_KEEP_SIDE}): {len(pcd_rgb.points)} points")

# 6) Confidence-colored visualization point cloud
pcd_conf = tsdf_integration.make_confidence_colored_pcd(
    pcd_rgb, conf,
    clip_percentile=CONF_CLIP_PERCENTILE,
    cv_colormap=CONF_COLORMAP
)

# 7) Visualize
axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
plane_vis = None
if PLANE_VIS_ENABLE:
    plane_vis = _make_plane_line_set(
        plane_model,
        pcd_rgb,
        color=PLANE_VIS_COLOR,
        scale=PLANE_VIS_SCALE,
        normal_color=PLANE_NORMAL_COLOR,
        normal_scale=PLANE_NORMAL_SCALE,
        normal_radius=PLANE_NORMAL_RADIUS,
        grid_lines=PLANE_VIS_GRID
    )

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
if SAVE_CONF:
    o3d.io.write_point_cloud(str(out_conf), pcd_conf)
print("Saved:", out_rgb)
if SAVE_CONF:
    print("Saved:", out_conf)
