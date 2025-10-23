#!/usr/bin/env python3
# verify_depth_distortion.py
# Load RAW depth + YAML intrinsics, build undistortion maps, remap depth,
# create point clouds (raw vs undistorted), push to CUDA, and save both.
# Comments in English only.

import sys
import os
import numpy as np
import cv2
import open3d as o3d

def load_yaml_intrinsics(yaml_path):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open YAML: {yaml_path}")
    width  = int(fs.getNode("image_width").real())
    height = int(fs.getNode("image_height").real())
    K = fs.getNode("camera_matrix").mat()
    D = fs.getNode("distortion_coefficients").mat()
    R = fs.getNode("rectification_matrix").mat()
    P = fs.getNode("projection_matrix").mat()
    fs.release()
    # Ensure shapes
    K = K.astype(np.float64).reshape(3,3)
    if D.size not in (4,5,8):  # plumb_bob can be 5; here you have 8
        raise ValueError(f"Unexpected distortion coeffs size: {D.size}")
    D = D.astype(np.float64).ravel()
    if R is None or R.size == 0:
        R = np.eye(3, dtype=np.float64)
    else:
        R = R.astype(np.float64).reshape(3,3)
    if P is not None and P.size >= 9:
        P = P.astype(np.float64).reshape(3,4)
    return width, height, K, D, R, P

def depth_to_image_tensor(depth_meters_f32):
    # depth_meters_f32: float32 array [H,W] in meters
    T = o3d.core.Tensor(depth_meters_f32, dtype=o3d.core.Dtype.Float32, device=o3d.core.Device("CPU:0"))
    return o3d.t.geometry.Image(T.contiguous())

def create_pcd_from_depth_t(depth_img_t, K_3x3, depth_max_m):
    # K_3x3 is numpy float32 [3,3]
    K_t = o3d.core.Tensor(K_3x3, dtype=o3d.core.Dtype.Float32, device=o3d.core.Device("CPU:0"))
    pcd_t = o3d.t.geometry.PointCloud.create_from_depth_image(
        depth=depth_img_t,
        intrinsics=K_t,
        extrinsics=o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32, o3d.core.Device("CPU:0")),
        depth_scale=1.0,         # already in meters
        depth_max=float(depth_max_m),
        stride=1,
        with_normals=False
    )
    return pcd_t

def main():
    if len(sys.argv) < 3:
        print("Usage: python verify_depth_distortion.py <depth.png> <intrinsics.yaml> [out_dir]")
        print("Notes:")
        print(" - Supports uint16 depth (mm) or float32 depth (m).")
        print(" - Saves RAW and UNDISTORTED point clouds for visual comparison.")
        sys.exit(1)

    depth_path = sys.argv[1]
    yaml_path  = sys.argv[2]
    out_dir    = sys.argv[3] if len(sys.argv) > 3 else os.path.dirname(os.path.abspath(depth_path)) or "."

    os.makedirs(out_dir, exist_ok=True)

    # --- Load intrinsics from YAML ---
    W, H, K, D, R, P = load_yaml_intrinsics(yaml_path)
    print(f"[INFO] YAML: {W}x{H}, fx={K[0,0]:.3f}, fy={K[1,1]:.3f}, cx={K[0,2]:.3f}, cy={K[1,2]:.3f}, D={D.tolist()}")

    # --- Load depth image UNCHANGED ---
    depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise RuntimeError(f"Cannot read depth: {depth_path}")
    h, w = depth.shape[:2]
    print(f"[INFO] Depth file: {w}x{h}, dtype={depth.dtype}")
    if (w, h) != (W, H):
        print("[WARN] Depth resolution does not match YAML. Proceeding anyway, but expect geometry mismatch.")

    # --- Convert to meters (float32) ---
    if depth.dtype == np.uint16 or depth.dtype == np.uint32:
        depth_m = depth.astype(np.float32) / 1000.0  # assume mm -> m
    elif depth.dtype == np.float32:
        depth_m = depth.copy()
    else:
        raise ValueError(f"Unsupported depth dtype: {depth.dtype}")

    # --- Build undistort/rectify maps (R = I), choose new K ---
    alpha = 0.0  # crop black borders; change to ~0.5 to keep more FOV
    K_new, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha, newImgSize=(w, h))
    map1, map2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), K_new, (w, h), cv2.CV_32FC1)

    # --- Remap depth with nearest-neighbor to avoid mixing ranges ---
    # Keep zeros as invalid (borderValue=0)
    depth_m_undist = cv2.remap(depth_m, map1, map2, interpolation=cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT, borderValue=0)

    # --- Make Tensor Images (CPU) ---
    depth_img_raw_t  = depth_to_image_tensor(depth_m)
    depth_img_und_t  = depth_to_image_tensor(depth_m_undist)

    # --- Create PCDs (CPU) using proper K for each ---
    K_raw  = K.astype(np.float32)
    K_rect = K_new.astype(np.float32)

    depth_trunc = 2.0  # meters, adjust for your scene
    pcd_raw_t = create_pcd_from_depth_t(depth_img_raw_t, K_raw,  depth_trunc)
    pcd_und_t = create_pcd_from_depth_t(depth_img_und_t, K_rect, depth_trunc)

    # --- Push to CUDA and do a light voxel for speed/clean-up ---
    dev_cuda = o3d.core.Device("CUDA:0") if o3d.core.cuda.is_available() else o3d.core.Device("CPU:0")
    pcd_raw_cuda = pcd_raw_t.to(dev_cuda).voxel_down_sample(voxel_size=0.01)
    pcd_und_cuda = pcd_und_t.to(dev_cuda).voxel_down_sample(voxel_size=0.01)

    print(f"[INFO] CUDA available: {o3d.core.cuda.is_available()} | Using device: {dev_cuda}")
    print(f"[INFO] RAW   PCD device: {pcd_raw_cuda.device}, points: {int(pcd_raw_cuda.point.positions.shape[0])}")
    print(f"[INFO] UND   PCD device: {pcd_und_cuda.device}, points: {int(pcd_und_cuda.point.positions.shape[0])}")

    # --- Back to CPU for saving/visualization ---
    raw_legacy = pcd_raw_cuda.to(o3d.core.Device("CPU:0")).to_legacy()
    und_legacy = pcd_und_cuda.to(o3d.core.Device("CPU:0")).to_legacy()

    out_raw = os.path.join(out_dir, "pc_raw.ply")
    out_und = os.path.join(out_dir, "pc_undistorted.ply")
    o3d.io.write_point_cloud(out_raw, raw_legacy, write_ascii=False, compressed=False)
    o3d.io.write_point_cloud(out_und, und_legacy, write_ascii=False, compressed=False)
    print(f"[OK] Saved:\n - {out_raw}\n - {out_und}")

    # Quick visual (optional)
    try:
        o3d.visualization.draw_geometries([raw_legacy], window_name="RAW (expect curvature if not rectified)")
        o3d.visualization.draw_geometries([und_legacy], window_name="UNDISTORTED (should look geometrically straighter)")
    except Exception:
        pass

if __name__ == "__main__":
    main()
