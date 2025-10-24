#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Refined TSDF debug tool — verified alignment chain, with automatic CPU fallback
for systems lacking CUDA TSDFVoxelGrid.
"""

import argparse, datetime
from pathlib import Path
import numpy as np, yaml, cv2, open3d as o3d
import open3d.core as o3c
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# ---------- Camera intrinsics ----------
class CamIntrinsics:
    def __init__(self, w, h, fx, fy, cx, cy):
        self.w, self.h, self.fx, self.fy, self.cx, self.cy = int(w), int(h), float(fx), float(fy), float(cx), float(cy)
    def to_o3d(self):
        return o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.cx, self.cy)
    def K_numpy(self):
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], dtype=np.float64)
    def K_tensor(self, device):
        return o3d.core.Tensor(self.K_numpy(), o3d.core.Dtype.Float64, device)

def load_ir_intrinsics(session_dir: Path):
    d1, d2 = session_dir / "depth_intrinsics.yaml", session_dir / "ir_intrinsics.yaml"
    used = d1 if d1.exists() else (d2 if d2.exists() else None)
    if used is None:
        print("⚠️ No intrinsics YAML found — using built-in defaults (640x576 fx≈505 fy≈505 cx≈338 cy≈338).")
        IR = CamIntrinsics(640, 576, 505.08520507812500, 505.02670288085938, 337.96063232421875, 338.32763671875000)
        return IR, None, None
    with open(used, "r") as f: y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
    dist = np.array(y.get("distortion_coefficients", {}).get("data", []), dtype=np.float32)
    w, h = int(y["image_width"]), int(y["image_height"])
    IR = CamIntrinsics(w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    if dist.size > 0 and np.any(np.abs(dist) > 1e-8):
        m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_32FC1)
        print(f"✓ Loaded intrinsics from {used.name} (with distortion correction).")
        return IR, (m1, m2), used
    print(f"✓ Loaded intrinsics from {used.name} (no distortion).")
    return IR, None, used

# ---------- Hand-eye transform ----------
def make_T_tool_ir():
    r_tool_ir = R.from_euler("xyz", [np.deg2rad(85.78033658),
                                 np.deg2rad(1.05565325),
                                 np.deg2rad(-178.75094858)])
    T_tool_ir = np.array([
        [-0.999592721, -0.016765345, -0.023094539,  0.011802710],
        [-0.021794634, -0.073963478,  0.997022748,  0.148739219],
        [-0.018423581,  0.997120023,  0.073567957, -0.025017651],
        [ 0.000000000,  0.000000000,  0.000000000,  1.000000000],
    ], dtype=np.float32)
    return T_tool_ir

# ---------- Utility ----------
def get_pose_tool0(cap):
    q = cap["pose_tool0"]["orientation_xyzw"]
    t = cap["pose_tool0"]["position"]
    r = R.from_quat([q[0], q[1], q[2], q[3]])
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = r.as_matrix().astype(np.float32)
    T[:3, 3]  = np.array(t, dtype=np.float32)
    return T

def pcd_from_depth(depth, intr, T_cam_to_world, depth_scale=1000.0, depth_trunc=1000.0):
    depth_img = o3d.geometry.Image(depth.astype(np.float32))
    return o3d.geometry.PointCloud.create_from_depth_image(
        depth_img, intr.to_o3d(),
        T_cam_to_world.astype(np.float64),
        depth_scale=depth_scale, depth_trunc=depth_trunc)

def make_axes(scale=0.1):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True)
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--depth-scale", type=float, default=1000.0)
    ap.add_argument("--depth-trunc", type=float, default=1.2)
    ap.add_argument("--voxel", type=float, default=1.0/512)
    ap.add_argument("--sdf-trunc", type=float, default=0.025)
    ap.add_argument("--gpu", action="store_true")
    args = ap.parse_args()

    session_dir = Path(args.session)
    with open(session_dir / "manifest.yaml", "r") as f: manifest = yaml.safe_load(f)
    captures = manifest["captures"]

    IR, remaps, _ = load_ir_intrinsics(session_dir)
    T_tool_ir = make_T_tool_ir()
    print("T_tool_ir translation (m):", T_tool_ir[:3,3])

    idxs = np.linspace(0, len(captures)-1, min(args.frames, len(captures)), dtype=int)
    print(f"[INFO] Using frames {list(idxs)} of {len(captures)}")

    # ---- Multi-frame overlay ----
    geoms = [make_axes(0.1)]
    # centroids = []
    for j in idxs:
        dpath = session_dir / captures[j]["depth"]
        d_raw = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED)
        if d_raw is None: continue
        # if remaps is not None: d_raw = cv2.remap(d_raw, remaps[0], remaps[1], cv2.INTER_NEAREST)
        # d_m = np.clip(d_raw.astype(np.float32) / args.depth_scale, 0.0, args.depth_trunc)
        T_base_tool = get_pose_tool0(captures[j])
        T_base_cam  = T_base_tool @ T_tool_ir
        T_cam_to_world = np.linalg.inv(T_base_cam)
        
        pcd = pcd_from_depth(d_raw, IR, T_cam_to_world)
        pcd.paint_uniform_color(np.random.rand(3))

        geoms.append(pcd)
        pts = np.asarray(pcd.points)
        # if pts.size > 0: centroids.append(np.mean(pts, axis=0))
    o3d.visualization.draw_geometries(geoms, window_name="Multi-frame overlay")

    # if len(centroids) > 1:
    #     dists = [np.linalg.norm(centroids[i]-centroids[i-1]) for i in range(1,len(centroids))]
    #     print(f"[DEBUG] Mean centroid shift (m): {np.mean(dists):.4f}")
    #     print(f"[DEBUG] Max centroid shift (m): {np.max(dists):.4f}")

    # ---- Step 4: TSDF integration (GPU or fallback CPU) ----
    print("[INFO] Initializing TSDF integration...")
    dev = o3d.core.Device("CPU:0")# if args.gpu and o3d.core.cuda.is_available() else o3d.core.Device("CPU:0")

    try:
        tsdf = o3d.t.geometry.VoxelBlockGrid(
				attr_names=['tsdf', 'weight'],
				attr_dtypes=[o3c.float32, o3c.float32],
				attr_channels=[(1), (1)],
				voxel_size=3.0/512,
				block_resolution=16,
				block_count=100000,
				device=dev)
        print("[INFO] Using GPU Tensor VoxelBlockGrid" if args.gpu else "[INFO] Using CPU Tensor VoxelBlockGrid")
    except AttributeError:
        print("[WARN] TSDFVoxelGrid not available — falling back to legacy CPU ScalableTSDFVolume.")
    #     tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
    #         voxel_length=float(args.voxel), sdf_trunc=float(args.sdf_trunc),
    #         color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)

    # ---- Integrate ----
    for j in idxs:
        dpath = session_dir / captures[j]["depth"]
        d_raw = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED)

        if d_raw is None: continue

        # if remaps is not None: d_raw = cv2.remap(d_raw, remaps[0], remaps[1], cv2.INTER_NEAREST)
        # d_m = np.clip(d_raw.astype(np.float32) / args.depth_scale, 0.0, args.depth_trunc)

        T_base_tool = get_pose_tool0(captures[j])
        T_base_cam  = T_base_tool @ T_tool_ir
        T_cam_to_world = np.linalg.inv(T_base_cam)


        if args.gpu and o3d.core.cuda.is_available():
            depth_tensor = o3d.t.geometry.Image(o3d.core.Tensor(d_raw, o3d.core.Dtype.Float32, dev))
            extr = o3d.core.Tensor(T_cam_to_world, o3d.core.Dtype.Float64, dev) # implement -> read_pinhole_camera_trajectory

            frustum_block_coords = tsdf.compute_unique_block_coordinates(depth_tensor,
                                                                        IR.K_tensor(dev),
                                                                        extr,
                                                                        args.depth_scale,
                                                                        args.depth_trunc).to(dev)
            tsdf.integrate(frustum_block_coords,
                           depth_tensor,
                           IR.K_tensor(dev),
                           extr,
                           args.depth_scale,
                           args.depth_trunc)
        else:
            pass
            # depth_legacy = o3d.geometry.Image((d_m * args.depth_scale).astype(np.uint16))
            # rgb_dummy = o3d.geometry.Image(np.zeros((IR.h, IR.w, 3), np.uint8))
            # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            #     rgb_dummy, depth_legacy,
            #     depth_scale=args.depth_scale, depth_trunc=args.depth_trunc,
            #     convert_rgb_to_intensity=False)
            # tsdf.integrate(rgbd, IR.to_o3d(), T_cam_to_world)

    # ---- Extract mesh + visualize ----
    print("[INFO] Extracting surface...")
    if use_gpu_tsdf:
        pcd = tsdf.extract_surface_point_cloud().to_legacy()
        mesh = tsdf.extract_triangle_mesh().to_legacy()
    else:
        pcd = tsdf.extract_point_cloud()
        mesh = tsdf.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    o3d.visualization.draw_geometries([make_axes(0.1), pcd], window_name="TSDF surface points")
    o3d.visualization.draw_geometries([make_axes(0.1), mesh], window_name="TSDF mesh")

    out_dir = session_dir / "outputs" / f"debug_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
    out_dir.mkdir(parents=True, exist_ok=True)
    o3d.io.write_point_cloud(str(out_dir / "mini_tsdf_surface.ply"), pcd)
    o3d.io.write_triangle_mesh(str(out_dir / "mini_tsdf_mesh.ply"), mesh)
    print(f"[DONE] Results saved in {out_dir}")

if __name__ == "__main__":
    main()




### TO DO:

###### 1 - How to read/translate/write trajectory logs?
# 	def trajectory(self):
# 		"""
# 		Computes and logs the robot's trajectory based on the transformations to the camera frame.

# 		:return: A list of transformation matrices representing the robot's trajectory.
# 		"""

# 		data = self.base_T_camera()
# 		Ts = np.asarray(data.get('H')).reshape((-1,4,4))
# 		n = len(Ts)
# 		result = []

# 		path = os.path.join(self.dir, 'trajectory.log')
# 		with open(path, 'w') as f:
# 			for i in range(n):
# 				f.write('{} {} {}\n'.format(i-1, i, n))

# 				T = np.asarray(Ts[i]).reshape((4,4))
				
# 				T[:3, 3] *= 0.001

# 				result.append(T)
# 				s = np.array2string(T)
# 				s = re.sub('[\[\]]', '', s)

# 				f.write('{}\n'.format(s))
# 		return result


##### 2 - use the TSDF Integration class
# class TSDF_Integration():



##### 3 - How do we import intrinsics?
# def import_intrinsic_calib(dir, fn="intrinsic.json"):
# """
# Imports intrinsic calibration data from a JSON file.

# :param dir: Directory where the intrinsic calibration file is located.
# :param fn: Filename of the intrinsic calibration file, default is 'intrinsic.json'.
# :return: A tuple containing the intrinsic matrix and distortion coefficients as numpy arrays.
#             Returns (None, None) if the file does not exist.
# """
# file_path = os.path.join(dir, fn)

# if not os.path.isfile(file_path):
#     print(f"The file {file_path} does not exist.")
#     return None, None

# data = readJSON(file_path)
# intrinsic_matrix = np.array(data.get("intrinsic_matrix", []))
# distortion_coefficients = np.array(data.get("distortion_coefficients", []))

# return np.reshape(intrinsic_matrix, (3, 3)), distortion_coefficients