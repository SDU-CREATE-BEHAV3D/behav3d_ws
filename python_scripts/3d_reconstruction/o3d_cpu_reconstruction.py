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


SESSION_PATH = "C:/Users/jomi/Desktop/PhD/BEAM-Resources/Behav3d_ws/python_scripts/251111_112516"
scan_folder = "manual_caps"
my_session = Session(SESSION_PATH, scan_folder)

class TSDF_Integration():

    def __init__(
            self, 
            session,
            voxel_size =0.002,
            block_count =300000,
            block_resolution =10,
            depth_max =1.5,
            depth_scale =1000.0,
            device ='CPU:0',
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
        self.depth_max = depth_max
        self.depth_scale = depth_scale

        self.mesh = o3d.geometry.TriangleMesh()

        # initialize VoxelBlockGrid
        self.vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf', 'weight'),
            attr_dtypes=(o3c.float32, o3c.float32),
            attr_channels=((1), (1)),
            voxel_size=self.voxel_size,
            block_resolution=self.block_resolution,
            block_count=self.block_count,
            device=self.device
        )

    ### tensorize robot poses
    def _tensorize_robot_poses(self):
        self.T_base_ir_tensors = []
        for T in self.T_base_ir:
            T_tensor = o3c.Tensor(T, o3c.Dtype.Float64, self.device)
            self.T_base_ir_tensors.append(T_tensor)
        return self.T_base_ir_tensors

    ### tensorize intrinsics
    def _tensorize_intrinsics(self):
        self.K_cpu = o3c.Tensor(
            np.asarray(self.pinhole_matrix.intrinsic_matrix),
            o3c.Dtype.Float64,
            self.device
        )
        return self.K_cpu
    
    ### tensorize depth images
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
            T_base_ir = self.T_base_ir_tensors[i]

            # camera-to-world transform
            Tcw_cpu = o3c.Tensor(np.linalg.inv(T_base_ir.numpy()), o3c.Dtype.Float64, self.device)

            # ---- create per-frame point cloud for visualization ----
            depth_o3d = o3d.geometry.Image(depth_tensor.numpy())
            intrinsic = o3d.camera.PinholeCameraIntrinsic(
                self.width, self.height,
                self.K[0, 0], self.K[1, 1],
                self.K[0, 2], self.K[1, 2]
            )

            # generate point cloud from depth
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d,
                intrinsic,
                extrinsic=np.linalg.inv(T_base_ir.numpy()),
                depth_scale=self.depth_scale,
                depth_trunc=self.depth_max
            )

            # color each frame differently
            color = np.random.rand(3)
            pcd.paint_uniform_color(color)
            if i == 0:
                self.pcds = []
            self.pcds.append(pcd)

            # compute voxel blocks (correct call)
            block_coords = self.vbg.compute_unique_block_coordinates(
                o3d.t.geometry.Image(depth_tensor),
                self.K_cpu,
                Tcw_cpu,
                self.depth_scale,
                self.depth_max
            )

            # integrate frame
            self.vbg.integrate(
                block_coords,
                o3d.t.geometry.Image(depth_tensor),
                self.K_cpu,
                Tcw_cpu,
                self.depth_scale,
                self.depth_max
            )
            print(f"Integrated depth image {i+1}/{len(self.depth_tensors)}")

        # visualize all per-frame clouds together
        o3d.visualization.draw(self.pcds + [o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

        # ---- Extract and visualize mesh ----
        self.mesh = self.vbg.extract_triangle_mesh().to_legacy()
        self.mesh.compute_vertex_normals()
        print("Vertices:", len(self.mesh.vertices))
        print("TSDF Integration complete.")
        return self.mesh


### test the tsdf integration class
tsdf_integration = TSDF_Integration(my_session)
# print("TSDF Integration initialized.")
print(f"Number of depth images loaded: {len(tsdf_integration.images)}")
print(f"Number of robot poses loaded: {len(tsdf_integration.T_base_tool0_list)}")
#Check for NaNs or zeros
img = tsdf_integration.images[0]
print("Depth range:", np.min(img), np.max(img))

mesh = tsdf_integration.integrate_depths()
o3d.visualization.draw([mesh, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])
# save mesh
o3d.io.write_triangle_mesh("tsdf_mesh.stl", mesh)
