# set src path for utils import
import sys
from pathlib import Path
sys.path.append(str(Path(__file__).resolve().parents[1]))

import numpy as np
import cv2
import open3d as o3d
import open3d.core as o3c
from scipy.spatial.transform import Rotation as R
import math

from utils.session import Session
from utils.load_helpers import load_camera_intrinsics
from utils.manifest import read_manifest, load_robot_poses, transform_robot_to_camera_pose, construct_image_paths
from utils.intrinsics import load_intrinsics, intrinsics_matrix
from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images
from utils.integration import visualize_camera_poses

SESSION_PATH = "C:/Users/jomi/Desktop/PhD/BEAM-Resources/Behav3d_ws/python_scripts/251111_112516"
scan_folder = "manual_caps"

my_session = Session(SESSION_PATH, scan_folder)


manifest = read_manifest(my_session.path, scan_folder)
# print ("Manifest:\n", manifest)

T_base_tool0_list = load_robot_poses(manifest)
# print("T_base_tool0:\n", T_base_tool0_list)

# load intrinsics
width, height, K, D = load_intrinsics(my_session.depth_intrinsics_path)
# print (f"Loaded intrinsics: width={width}, height={height}, K=\n{K}, D={D}")

intrinsics = intrinsics_matrix(width, height, K)
# print("Intrinsics matrix:\n", intrinsics.intrinsic_matrix)

### load extrinsics
T_tool0_ir = load_extrinsics(my_session._camera_extrinsics_path, frame_key="T_tool0_ir")
# print("T_tool0_ir:\n", T_tool0_ir)

### transform robot to camera poses
T_base_ir_list = [T_base_tool0 @ T_tool0_ir for T_base_tool0 in T_base_tool0_list]
# print("T_base_ir:\n", T_base_ir_list)


### load image paths
image_paths = construct_image_paths(manifest, my_session, scan_folder, image_type="depth")
# print("Depth image paths:\n", image_paths)

images = load_images(image_paths, image_type="depth", library="cv2")




visualize_camera_poses(T_base_tool0_list, T_base_ir_list)









# # gpu reconstruction function with depth integration
# def gpu_reconstruction(captures, intrinsic, voxel_length=0.008, sdf_trunc=0.04):
#     # Initialize Open3D TSDF volume (CPU version for debugging)
#     volume = o3d.pipelines.integration.ScalableTSDFVolume(
#         voxel_length=voxel_length,
#         sdf_trunc=sdf_trunc,
#         color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor
#     )
    
#     expected_width = intrinsic.width
#     expected_height = intrinsic.height
#     print(f"Expected frame size from intrinsics: {expected_width}x{expected_height}")

#     for cap in captures:
#         depth_path = cap.get("depth_path", None)
#         T = cap.get("camera_pose", None)
#         if depth_path is None or T is None:
#             continue

#         # Load depth image
#         depth_image = o3d.io.read_image(depth_path)
#         if depth_image is None:
#             continue

#         # Convert to numpy for sanity check
#         depth_np = np.asarray(depth_image)
#         h, w = depth_np.shape

#         # ---------- TEMPORARY RESIZE TO MATCH INTRINSICS ----------
#         if (w, h) != (expected_width, expected_height):
#             print(f"Resizing depth {w}x{h} → {expected_width}x{expected_height}")
#             depth_np = cv2.resize(depth_np, (expected_width, expected_height), interpolation=cv2.INTER_NEAREST)
#             depth_image = o3d.geometry.Image(depth_np)
#         # ----------------------------------------------------------

#         # Create a dummy gray image (required by Open3D RGBD factory)
#         gray_image = o3d.geometry.Image(np.zeros((expected_height, expected_width), dtype=np.uint8))

#         # Create RGBD image with no color
#         rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#             gray_image,
#             depth_image,
#             depth_scale=1000.0,   # adjust for your depth units
#             depth_trunc=3.0,
#             convert_rgb_to_intensity=False
#         )

#         # Integrate depth frame
#         volume.integrate(
#             rgbd_image,
#             intrinsic,
#             np.linalg.inv(T)  # world → camera
#         )
#         print(f"Integrated capture {cap['index']}")

#     # Extract surface mesh from TSDF
#     mesh = volume.extract_triangle_mesh()
#     mesh.compute_vertex_normals()
#     print("Reconstruction complete.")
#     o3d.visualization.draw([mesh, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)])

#     # Save mesh to file
#     mesh_path = "reconstructed_mesh_gpu.stl"
#     # o3d.io.write_triangle_mesh(mesh_path, mesh)


#     return mesh

    



