# set src path for utils import
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
import open3d as o3d
import open3d.core as o3c

from utils.session import Session
from utils.manifest import (
    read_manifest,
    load_robot_poses,
    construct_image_paths,
)
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images


SESSION_PATH = "/home/lab/behav3d_ws/captures/260113_170839"
scan_folder = "manual_caps"
my_session = Session(SESSION_PATH, scan_folder)


class TSDFIntegration:
    """
    Dual-mode TSDF integration (CPU / CUDA) using Open3D Tensor pipeline (VoxelBlockGrid).

    IMPORTANT (based on your Open3D build behavior):
      - In CUDA mode, Open3D's compute_unique_block_coordinates() internally calls
        InverseTransformation() which expects CPU tensors for intrinsics/extrinsics.
      - Therefore, in CUDA mode:
          * VoxelBlockGrid lives on CUDA
          * Depth tensors live on CUDA
          * Intrinsics K and extrinsics Tcw live on CPU (Float64)
      - In CPU mode: everything lives on CPU.

    This keeps TSDF integration accelerated while satisfying the CPU-only intrinsic/extrinsic constraints.
    """

    def __init__(
        self,
        session: Session,
        mode: str = "cuda",                 # "cpu" or "cuda"
        voxel_size: float = 1 / 1024,
        block_count: int = 20000,
        block_resolution: int = 12,
        depth_max: float = 1.0,
        depth_scale: float = 1000.0,
        debug_legacy_vis: bool = False,     # CPU-only visualization (slow in cuda mode)
        vis_stride: int = 20,
    ):
        self.session = session
        self.mode = mode.lower().strip()
        if self.mode not in ("cpu", "cuda"):
            raise ValueError(f"mode must be 'cpu' or 'cuda', got: {mode}")

        # TSDF parameters
        self.voxel_size = voxel_size
        self.block_count = block_count
        self.block_resolution = block_resolution
        self.depth_max = depth_max
        self.depth_scale = depth_scale

        # Debug viz
        self.debug_legacy_vis = debug_legacy_vis
        self.vis_stride = max(1, int(vis_stride))

        # Devices
        self.cpu = o3c.Device("CPU:0")
        self.gpu = o3c.Device("CUDA:0")
        self.tsdf_device = self.gpu if self.mode == "cuda" else self.cpu

        # In YOUR Open3D build, these must be CPU tensors (Float64) even in CUDA mode:
        self.calib_device = self.cpu

        # ---- load session data (numpy / python) ----
        self.manifest = read_manifest(self.session.path, self.session._scan_folder)

        self.T_base_tool0_list = load_robot_poses(self.manifest)

        # extrinsics: tool0 -> IR
        self.T_tool0_ir = load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir")

        # base -> IR for each frame
        self.T_base_ir = [T_base_tool0 @ self.T_tool0_ir for T_base_tool0 in self.T_base_tool0_list]

        # depth images
        self.image_paths = construct_image_paths(self.manifest, self.session, image_type="depth")
        self.images = load_images(self.image_paths, image_type="depth", library="cv2")

        # intrinsics
        self.width, self.height, self.K, self.D = load_intrinsics(self.session.depth_intrinsics_path)
        self.pinhole_matrix = intrinsics_matrix(self.width, self.height, self.K)

        # ---- tensors ----
        self.K_tensor = self._make_intrinsics_tensor_cpu_float64()
        self.depth_tensors = self._make_depth_tensors_on_tsdf_device()
        self.Tcw_tensors = self._make_Tcw_tensors_cpu_float64()

        # ---- TSDF structure ----
        self.vbg = self._make_vbg_on_tsdf_device()

        # outputs
        self.mesh = None
        self.debug_pcds = []

        self._assert_device_consistency()

    # -------------------------------------------------------------------------
    # Checks / helpers
    # -------------------------------------------------------------------------

    def _assert_device_consistency(self):
        # VoxelBlockGrid does not expose a .device attribute in Python bindings, so we validate inputs instead.

        # Intrinsics and poses must be CPU Float64 for your build.
        if self.K_tensor.device != self.calib_device:
            raise RuntimeError(f"K_tensor on {self.K_tensor.device}, expected {self.calib_device}")
        if self.K_tensor.dtype != o3c.Dtype.Float64:
            raise RuntimeError(f"K_tensor dtype {self.K_tensor.dtype}, expected Float64")

        for i, T in enumerate(self.Tcw_tensors):
            if T.device != self.calib_device:
                raise RuntimeError(f"Tcw_tensors[{i}] on {T.device}, expected {self.calib_device}")
            if T.dtype != o3c.Dtype.Float64:
                raise RuntimeError(f"Tcw_tensors[{i}] dtype {T.dtype}, expected Float64")

        # Depth tensors should match TSDF device (CPU in cpu mode, CUDA in cuda mode)
        for i, d in enumerate(self.depth_tensors):
            if d.device != self.tsdf_device:
                raise RuntimeError(f"depth_tensors[{i}] on {d.device}, expected {self.tsdf_device}")
            if d.dtype != o3c.Dtype.UInt16:
                raise RuntimeError(f"depth_tensors[{i}] dtype {d.dtype}, expected UInt16")

    def _to_cpu_numpy(self, tensor: o3c.Tensor) -> np.ndarray:
        if tensor.device.get_type() == o3c.Device.DeviceType.CUDA:
            return tensor.cpu().numpy()
        return tensor.numpy()

    # -------------------------------------------------------------------------
    # Tensor creation
    # -------------------------------------------------------------------------

    def _make_intrinsics_tensor_cpu_float64(self) -> o3c.Tensor:
        K_np = np.asarray(self.pinhole_matrix.intrinsic_matrix, dtype=np.float64)
        return o3c.Tensor(K_np, dtype=o3c.Dtype.Float64, device=self.calib_device)

    def _make_depth_tensors_on_tsdf_device(self):
        out = []
        for img in self.images:
            if img.dtype != np.uint16:
                img = img.astype(np.uint16, copy=False)
            out.append(o3c.Tensor(img, dtype=o3c.Dtype.UInt16, device=self.tsdf_device))
        return out

    def _make_Tcw_tensors_cpu_float64(self):
        """
        Keep your original convention:
          Tcw = inv(T_base_ir)
        Precompute once; store on CPU as Float64.
        """
        out = []
        for T_base_ir in self.T_base_ir:
            Tcw_np = np.linalg.inv(T_base_ir).astype(np.float64, copy=False)
            out.append(o3c.Tensor(Tcw_np, dtype=o3c.Dtype.Float64, device=self.calib_device))
        return out

    # -------------------------------------------------------------------------
    # TSDF structure
    # -------------------------------------------------------------------------

    def _make_vbg_on_tsdf_device(self) -> o3d.t.geometry.VoxelBlockGrid:
        return o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight"),
            attr_dtypes=(o3c.float32, o3c.float32),
            attr_channels=((1,), (1,)),
            voxel_size=self.voxel_size,
            block_resolution=self.block_resolution,
            block_count=self.block_count,
            device=self.tsdf_device,
        )

    # -------------------------------------------------------------------------
    # Debug legacy visualization (optional, CPU-bound)
    # -------------------------------------------------------------------------

    def _debug_add_legacy_pcd(self, frame_idx: int, depth_tensor: o3c.Tensor, Tcw_tensor: o3c.Tensor):
        """
        CPU-only legacy point cloud for sanity checks.
        In CUDA mode this forces device->host copies for the depth.
        Use sparingly via vis_stride.
        """
        depth_np = self._to_cpu_numpy(depth_tensor)
        Tcw_np = self._to_cpu_numpy(Tcw_tensor)

        depth_o3d = o3d.geometry.Image(depth_np)

        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            self.width,
            self.height,
            float(self.K[0, 0]),
            float(self.K[1, 1]),
            float(self.K[0, 2]),
            float(self.K[1, 2]),
        )

        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d,
            intrinsic,
            extrinsic=Tcw_np,
            depth_scale=self.depth_scale,
            depth_trunc=self.depth_max,
        )

        pcd.paint_uniform_color(np.random.rand(3))
        self.debug_pcds.append(pcd)

    # -------------------------------------------------------------------------
    # Integration
    # -------------------------------------------------------------------------

    def integrate(self) -> o3d.geometry.TriangleMesh:
        self._assert_device_consistency()

        n = len(self.depth_tensors)
        for i in range(n):
            depth_tensor = self.depth_tensors[i]
            Tcw = self.Tcw_tensors[i]

            depth_img_t = o3d.t.geometry.Image(depth_tensor)

            # K and Tcw are CPU Float64 (required by your build)
            block_coords = self.vbg.compute_unique_block_coordinates(
                depth_img_t,
                self.K_tensor,
                Tcw,
                self.depth_scale,
                self.depth_max,
            )

            self.vbg.integrate(
                block_coords,
                depth_img_t,
                self.K_tensor,
                Tcw,
                self.depth_scale,
                self.depth_max,
            )

            if self.debug_legacy_vis and (i % self.vis_stride == 0):
                self._debug_add_legacy_pcd(i, depth_tensor, Tcw)

            print(f"Integrated depth image {i+1}/{n} | TSDF on {self.tsdf_device} | calib on {self.calib_device}")

        if self.debug_legacy_vis and len(self.debug_pcds) > 0:
            o3d.visualization.draw(
                self.debug_pcds + [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)]
            )

        # Extract mesh (tensor -> legacy once)
        mesh_t = self.vbg.extract_triangle_mesh()
        mesh = mesh_t.to_legacy()
        mesh.compute_vertex_normals()

        self.mesh = mesh
        print("Vertices:", len(mesh.vertices))
        print("TSDF Integration complete.")
        return mesh

    def save_mesh(self, out_path: str):
        if self.mesh is None:
            raise RuntimeError("No mesh to save. Run integrate() first.")
        o3d.io.write_triangle_mesh(out_path, self.mesh)

    # mesh refinement
    def refine_mesh_high_res(
        self,
        mesh: o3d.geometry.TriangleMesh,
        keep_largest_component: bool = True,
        taubin_iters: int = 15,
        laplacian_iters: int = 0,
    ):
        # 1) cleanup (no resolution loss)
        mesh.remove_degenerate_triangles()
        mesh.remove_duplicated_triangles()
        mesh.remove_duplicated_vertices()
        mesh.remove_unreferenced_vertices()

        # 2) remove floaters (noise blobs)
        if keep_largest_component:
            tri_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
            tri_clusters = np.asarray(tri_clusters)
            cluster_n_triangles = np.asarray(cluster_n_triangles)

            if cluster_n_triangles.size > 0:
                keep_id = int(np.argmax(cluster_n_triangles))
                remove_mask = tri_clusters != keep_id
                mesh.remove_triangles_by_mask(remove_mask)
                mesh.remove_unreferenced_vertices()

        # 3) smoothing for flat surfaces
        # Taubin reduces noise with less shrinkage than Laplacian.
        if taubin_iters and taubin_iters > 0:
            mesh = mesh.filter_smooth_taubin(number_of_iterations=int(taubin_iters))

        # Optional extra smoothing (stronger, more shrink risk)
        if laplacian_iters and laplacian_iters > 0:
            mesh = mesh.filter_smooth_laplacian(number_of_iterations=int(laplacian_iters))

        mesh.compute_vertex_normals()
        return mesh



if __name__ == "__main__":
    # Choose mode: "cpu" or "cuda"
    MODE = "cuda"

    tsdf = TSDFIntegration(
        my_session,
        mode=MODE,
        debug_legacy_vis=False,  # set True only for debugging (slow in cuda)
        vis_stride=20,
    )

    print(f"Number of depth images loaded: {len(tsdf.images)}")
    print(f"Number of robot poses loaded: {len(tsdf.T_base_tool0_list)}")

    img0 = tsdf.images[0]
    print("Depth range:", int(np.min(img0)), int(np.max(img0)))

    mesh = tsdf.integrate()
    o3d.visualization.draw([mesh, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])
    
    # refine mesh
    # mesh = tsdf.refine_mesh_high_res(
    #     mesh,
    #     keep_largest_component=True,
    #     taubin_iters=15,
    #     laplacian_iters=0,
    # )
    # o3d.visualization.draw([mesh, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

    # save mesh
    tsdf.save_mesh("/home/lab/robot/meshes/tsdf_mesh.stl")
