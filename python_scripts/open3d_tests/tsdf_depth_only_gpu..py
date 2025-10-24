#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import datetime
from pathlib import Path
import numpy as np
import cv2, yaml, open3d as o3d
from scipy.spatial.transform import Rotation as R

# ---------- CONFIGURATION ----------
SESSION_DIR = Path(r"/home/lab/behav3d_ws/captures/251021_115853/scan_1")
VOXEL = 0.002
SDF_TRUNC = 0.01
DEPTH_SCALE = 1000.0
DEPTH_TRUNC = 1.2
SCANS = 4
BILATERAL = 0
# -----------------------------------

# ---------- INTRINSICS / EXTRINSICS ----------
class CamIntrinsics:
    def __init__(self, w, h, fx, fy, cx, cy):
        self.w, self.h, self.fx, self.fy, self.cx, self.cy = int(w), int(h), fx, fy, cx, cy
    def to_o3d(self):
        return o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.cx, self.cy)
    def K_tensor(self, device):
        K_np = np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]], np.float32)
        return o3d.core.Tensor(K_np, device=device)

IR = CamIntrinsics(640, 576, 505.0852, 505.0267, 337.9606, 338.3276)
DEPTH_REMAPS = None

r_tool_ir = R.from_euler("xyz", [1.4796203267, 0.0010471976, -3.1372642370])
T_tool_ir = np.eye(4, dtype=np.float32)
T_tool_ir[:3, :3] = r_tool_ir.as_matrix()
T_tool_ir[:3, 3] = np.array([0.00180271, 0.15873923, -0.03501765], np.float32)

def load_ir_intrinsics(session_dir: Path):
    global IR, DEPTH_REMAPS
    dpath = session_dir / "depth_intrinsics.yaml"
    if not dpath.exists():
        dpath = session_dir / "ir_intrinsics.yaml"
    if not dpath.exists():
        print("⚠️ No intrinsics YAML found — using built-in defaults.")
        DEPTH_REMAPS = None
        return
    with open(dpath, "r") as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"]["data"], np.float32).reshape(3, 3)
    dist = np.array(y.get("distortion_coefficients", {}).get("data", []), np.float32)
    w, h = int(y["image_width"]), int(y["image_height"])
    IR = CamIntrinsics(w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    if dist.size > 0 and np.any(np.abs(dist) > 1e-8):
        m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_32FC1)
        DEPTH_REMAPS = (m1, m2)
    else:
        DEPTH_REMAPS = None

def gpu_tsdf_available():
    return hasattr(o3d.t.geometry, "TSDFVoxelGrid") and o3d.core.cuda.is_available()

# ---------- PIPELINE ----------
def run():
    session_dir = SESSION_DIR
    print(f"[SESSION] {session_dir}")

    manifest_path = session_dir / "manifest.yaml"
    if not manifest_path.exists():
        raise FileNotFoundError(f"manifest.yaml missing in {session_dir}")

    load_ir_intrinsics(session_dir)
    with open(manifest_path, "r") as f:
        manifest = yaml.safe_load(f)
    captures = manifest.get("captures", [])
    if not captures:
        raise RuntimeError("No captures in manifest.yaml")

    n_total = len(captures)
    idxs = np.linspace(0, n_total - 1, min(SCANS, n_total), dtype=int)
    print(f"[INFO] Using frames {list(idxs)} of {n_total}")

    out_dir = session_dir / "outputs" / f"run_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}_debug"
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"[OUT_DIR] {out_dir}")

    use_gpu = gpu_tsdf_available()
    if use_gpu:
        print("[INFO] Using GPU Tensor TSDFVoxelGrid")
        dev = o3d.core.Device("CUDA:0")
        tsdf = o3d.t.geometry.TSDFVoxelGrid(
            voxel_size=VOXEL, sdf_trunc=SDF_TRUNC,
            block_resolution=16, block_count=200000,
            device=dev, dtype=o3d.core.Dtype.Float32,
            color_type=getattr(o3d.t.geometry.TSDFVoxelGrid.ColorType, "None"))
        K_cuda = IR.K_tensor(dev)
    else:
        print("[INFO] Using legacy CPU ScalableTSDFVolume")
        tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=VOXEL, sdf_trunc=SDF_TRUNC,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        intr_o3d = IR.to_o3d()

    for i in idxs:
        cap = captures[i]
        depth_path = session_dir / cap["depth"]
        if not depth_path.exists():
            print(f"⚠️ Missing depth frame: {depth_path.name}")
            continue
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            print(f"⚠️ Could not read {depth_path}")
            continue
        if DEPTH_REMAPS is not None:
            depth_raw = cv2.remap(depth_raw, DEPTH_REMAPS[0], DEPTH_REMAPS[1], cv2.INTER_NEAREST)

        depth_m = depth_raw.astype(np.float32) / DEPTH_SCALE
        if BILATERAL:
            depth_m = cv2.bilateralFilter(depth_m, 5, 0.01, 5)
        depth_m = np.clip(depth_m, 0.0, DEPTH_TRUNC)

        qx, qy, qz, qw = cap["pose_tool0"]["orientation_xyzw"]
        tx, ty, tz = cap["pose_tool0"]["position"]
        T_base_tool = np.eye(4, dtype=np.float32)
        T_base_tool[:3, :3] = R.from_quat([qx, qy, qz, qw]).as_matrix()
        T_base_tool[:3, 3] = np.array([tx, ty, tz], np.float32)
        T_base_camIR = T_base_tool @ T_tool_ir
        T_cam_to_world = np.linalg.inv(T_base_camIR).astype(np.float32)

        # --- Depth->PCD visualization (blocking) ---
        print(f"\n[DEBUG] Visualizing depth frame {i}")
        if use_gpu:
            depth_tensor = o3d.t.geometry.Image(o3d.core.Tensor(depth_m, o3d.core.Dtype.Float32, dev))
            pcd_dbg = o3d.t.geometry.PointCloud.create_from_depth_image(
                depth_tensor, intrinsics=K_cuda,
                extrinsics=o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32, dev),
                depth_scale=1.0, depth_max=DEPTH_TRUNC)
            o3d.visualization.draw_geometries([pcd_dbg.to_legacy()],
                                              window_name=f"Depth→PCD frame {i}")
        else:
            depth_legacy = o3d.geometry.Image(depth_m)
            pcd_dbg = o3d.geometry.PointCloud.create_from_depth_image(
                depth_legacy, IR.to_o3d(),
                np.eye(4, dtype=np.float64), 1.0, DEPTH_TRUNC)
            o3d.visualization.draw_geometries([pcd_dbg],
                                              window_name=f"Depth→PCD frame {i}")

        # --- integrate ---
        if use_gpu:
            extr = o3d.core.Tensor(T_cam_to_world, o3d.core.Dtype.Float32, dev)
            tsdf.integrate(depth=depth_tensor, color=None,
                           depth_intrinsics=K_cuda, extrinsics=extr,
                           depth_scale=1.0, depth_max=DEPTH_TRUNC)
        else:
            d16 = (depth_m * DEPTH_SCALE).astype(np.uint16)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.zeros((IR.h, IR.w, 3), np.uint8)),
                o3d.geometry.Image(d16),
                depth_scale=DEPTH_SCALE, depth_trunc=DEPTH_TRUNC,
                convert_rgb_to_intensity=False)
            tsdf.integrate(rgbd, intr_o3d, T_cam_to_world)

        # --- TSDF stats ---
        if use_gpu:
            vox = tsdf.voxel_grid["tsdf"].cpu().numpy()
            print(f"[TSDF] frame {i} → min {np.nanmin(vox):.3f} max {np.nanmax(vox):.3f}")

        # --- Live TSDF visualization (blocking) ---
        print(f"[DEBUG] Visualizing fused TSDF surface after frame {i}")
        if use_gpu:
            pcd_live = tsdf.extract_surface_point_cloud().to_legacy()
        else:
            pcd_live = tsdf.extract_point_cloud()
        o3d.visualization.draw_geometries([pcd_live],
                                          window_name=f"TSDF after frame {i}")

        print(f"[OK] Integrated frame {i}")

    # ---- Final mesh + PCD visualization ----
    if use_gpu:
        pcd = tsdf.extract_surface_point_cloud().to_legacy()
        mesh = tsdf.extract_triangle_mesh().to_legacy()
    else:
        pcd = tsdf.extract_point_cloud()
        mesh = tsdf.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    print("\n[DEBUG] Visualizing final fused results")
    o3d.visualization.draw_geometries([pcd, mesh], window_name="Final Fused Results")

    pcd_path = out_dir / "integrated_point_cloud.ply"
    mesh_path = out_dir / "integrated_mesh.ply"
    o3d.io.write_point_cloud(str(pcd_path), pcd)
    o3d.io.write_triangle_mesh(str(mesh_path), mesh)
    print(f"[DONE] Saved:\n - {pcd_path}\n - {mesh_path}")

if __name__ == "__main__":
    run()
