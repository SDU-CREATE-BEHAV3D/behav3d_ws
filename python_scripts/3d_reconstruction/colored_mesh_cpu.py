# CPU TSDF reconstruction + mesh coloring using your precomputed color_in_depth captures
#
# What this adds to your working CPU TSDF script:
# 1) Loads color_in_depth images from disk (the outputs of your working color_to_depth script)
# 2) Colors the extracted TSDF mesh by projecting vertices into each frame's depth camera,
#    sampling color_in_depth at the projected pixel, and averaging across frames.
#
# Assumptions:
# - Your color_to_depth script saved images like:
#     .../alignment_test/color_in_depth_0000.png
#     .../alignment_test/color_in_depth_0001.png
#   (update C2D_DIR and pattern if yours differ)
# - color_in_depth is in depth resolution and aligned to depth intrinsics (Kd = depth_intrinsics)
#
# Output:
# - tsdf_mesh_colored.ply  (mesh with vertex_colors)
# - tsdf_mesh.stl          (optional, unchanged)

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


SESSION_PATH = "C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/260114_112631"
scan_folder = "manual_caps"
output_folder = Path("C:/Users/jomi/Desktop/PhD/BEAM-Resources/captures/260114_112631")
my_session = Session(SESSION_PATH, scan_folder)

# Folder where your color_to_depth script saved depth-aligned RGB images:
# Update if needed.
C2D_DIR = output_folder / "alignment_test"
C2D_PATTERN = "color_in_depth_{:04d}.png"  # expected naming


class TSDF_Integration():

    def __init__(
            self,
            session,
            voxel_size=1/512,
            block_count=10000,
            block_resolution=12,
            depth_max=1.0,
            depth_scale=1000.0,
            device='CPU:0',   # CPU only
        ):
        # initialize session and load poses
        self.session = session
        self.manifest = read_manifest(self.session.path, self.session._scan_folder)
        self.T_base_tool0_list = load_robot_poses(self.manifest)
        self.T_tool0_ir = load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir")
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

        self.mesh = o3d.geometry.TriangleMesh()

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

        # --- NEW: load color_in_depth images (depth-aligned RGB) ---
        self.color_in_depth = self._load_color_in_depth_images()

        # sanity
        if len(self.color_in_depth) != len(self.images):
            print(f"WARNING: color_in_depth frames ({len(self.color_in_depth)}) != depth frames ({len(self.images)}). "
                  "Mesh coloring will use min(N).")

    def _load_color_in_depth_images(self):
        """
        Loads ALL depth-aligned RGB images from disk.
        Files must contain 'color_in_depth' in the filename.
        Returns a list of uint8 BGR images sorted by filename.
        """
        if not C2D_DIR.exists():
            raise FileNotFoundError(f"color_in_depth folder not found: {C2D_DIR}")

        paths = sorted(C2D_DIR.glob("color_in_depth*.png"))

        if len(paths) == 0:
            raise FileNotFoundError(
                f"No color_in_depth images found in {C2D_DIR}"
            )

        imgs = []
        for p in paths:
            img = cv2.imread(str(p), cv2.IMREAD_COLOR)
            if img is None:
                raise RuntimeError(f"Failed to read image: {p}")
            imgs.append(img)

        print(f"Loaded {len(imgs)} color_in_depth images")
        return imgs


    ### tensorize robot poses (CPU)
    def _tensorize_robot_poses(self):
        cpu = o3c.Device("CPU:0")
        self.T_base_ir_tensors = []
        for T in self.T_base_ir:
            T_tensor = o3c.Tensor(T, o3c.Dtype.Float64, cpu)
            self.T_base_ir_tensors.append(T_tensor)
        return self.T_base_ir_tensors

    ### tensorize intrinsics (CPU)
    def _tensorize_intrinsics(self):
        self.K_cpu = o3c.Tensor(
            np.asarray(self.pinhole_matrix.intrinsic_matrix),
            o3c.Dtype.Float64,
            self.device
        )
        return self.K_cpu

    ### tensorize depth images (CPU)
    def _tensorize_depth_images(self):
        self.depth_tensors = []
        for img in self.images:
            depth_tensor = o3c.Tensor(img, o3c.Dtype.UInt16, self.device)
            self.depth_tensors.append(depth_tensor)
        return self.depth_tensors

    ### integrate depth images into TSDF (depth-only)
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

            # ---- per-frame point cloud for visualization ----
            depth_o3d = o3d.geometry.Image(depth_tensor.numpy())
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                self.width, self.height,
                self.K[0, 0], self.K[1, 1],
                self.K[0, 2], self.K[1, 2]
            )

            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d,
                intrinsic,
                extrinsic=np.linalg.inv(T_base_ir_cpu.numpy()),
                depth_scale=self.depth_scale,
                depth_trunc=self.depth_max
            )

            color = np.random.rand(3)
            pcd.paint_uniform_color(color)
            if i == 0:
                self.pcds = []
            self.pcds.append(pcd)

            # TSDF integration (CPU tensor pipeline)
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

        # visualize all per-frame clouds together
        o3d.visualization.draw(self.pcds + [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

        # Extract and visualize mesh
        self.mesh = self.vbg.extract_triangle_mesh().to_legacy()
        self.mesh.compute_vertex_normals()
        print("Vertices:", len(self.mesh.vertices))
        print("TSDF Integration complete.")
        return self.mesh

    # --- NEW: colorize TSDF mesh from depth-aligned color captures ---
    def colorize_mesh_from_c2d(self, mesh, sample_step=1):
        """
        Colors mesh vertices by projecting each vertex into depth camera views and
        sampling the corresponding pixel in color_in_depth (depth-canonical RGB).
        Averages colors across frames.

        sample_step: use every Nth frame to speed up (1 = all frames)
        """
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        V = verts.shape[0]

        accum = np.zeros((V, 3), dtype=np.float64)
        weight = np.zeros((V, 1), dtype=np.float64)

        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        cx, cy = float(self.K[0, 2]), float(self.K[1, 2])

        H, W = self.images[0].shape[:2]
        verts_h = np.concatenate([verts, np.ones((V, 1), dtype=np.float64)], axis=1)

        n_frames = min(len(self.T_base_ir), len(self.color_in_depth))

        for i in range(0, n_frames, sample_step):
            T_base_ir = self.T_base_ir[i]
            T_ir_base = np.linalg.inv(T_base_ir)

            X_ir_h = (T_ir_base @ verts_h.T).T
            X = X_ir_h[:, 0]
            Y = X_ir_h[:, 1]
            Z = X_ir_h[:, 2]

            valid = Z > 0
            u = (X / Z) * fx + cx
            v = (Y / Z) * fy + cy

            u_i = np.round(u).astype(np.int32)
            v_i = np.round(v).astype(np.int32)

            inb = (u_i >= 0) & (u_i < W) & (v_i >= 0) & (v_i < H)
            valid = valid & inb
            if not np.any(valid):
                continue

            # Optionally enforce depth range by comparing to the actual depth image
            # (helps avoid coloring vertices that are behind the observed surface)
            depth = self.images[i].astype(np.float32) / self.depth_scale
            z_obs = depth[v_i[valid], u_i[valid]]
            ok = valid.copy()
            # keep if observed depth exists and vertex is close to observed surface
            ok[valid] = (z_obs > 0) & (z_obs < self.depth_max) & (np.abs(z_obs - Z[valid]) < 0.02)  # 2 cm band

            if not np.any(ok):
                continue

            bgr = self.color_in_depth[i][v_i[ok], u_i[ok], :].astype(np.float64) / 255.0
            rgb = bgr[:, ::-1]

            accum[ok] += rgb
            weight[ok] += 1.0

        colors = np.zeros((V, 3), dtype=np.float64)
        nz = weight[:, 0] > 0
        colors[nz] = accum[nz] / weight[nz]

        mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
        return mesh


### test the tsdf integration class
tsdf_integration = TSDF_Integration(my_session)
print(f"Number of depth images loaded: {len(tsdf_integration.images)}")
print(f"Number of robot poses loaded: {len(tsdf_integration.T_base_tool0_list)}")
print(f"Number of c2d images loaded: {len(tsdf_integration.color_in_depth)}")

img = tsdf_integration.images[0]
print("Depth range:", np.min(img), np.max(img))

mesh = tsdf_integration.integrate_depths()

# Colorize mesh using c2d captures (set sample_step=2 or 3 if slow)
mesh_colored = tsdf_integration.colorize_mesh_from_c2d(mesh, sample_step=1)

o3d.visualization.draw([mesh_colored, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

# save meshes
o3d.io.write_triangle_mesh(str(output_folder / "tsdf_mesh.stl"), mesh)  # geometry only
o3d.io.write_triangle_mesh(str(output_folder / "tsdf_mesh_colored.ply"), mesh_colored)  # colored
