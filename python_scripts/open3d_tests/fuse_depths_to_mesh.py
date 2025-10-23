#!/usr/bin/env python3
# fuse_depths_to_mesh.py
# Volumetric fusion (TSDF, legacy CPU) of many RAW depth frames into a clean mesh/point cloud.
# Uses:
#  - Hard-coded T_tool0_depth from URDF (origin xyz + rpy in radians, extrinsic X-Y-Z as in URDF)
#  - Per-capture T_base_tool0 from manifest.yaml (position + orientation_xyzw)
#  - Depth rectification via OpenCV K,D (nearest-neighbor remap; preserves metric depths)
# Outputs:
#  - Fused point cloud (from TSDF)
#  - Fused mesh (from TSDF)
# Comments in English only.

import os
import argparse
import yaml
from typing import Dict, Any, List, Tuple

import numpy as np
import cv2
import open3d as o3d


# ============================
# >>>>>> URDF EXTRINSICS (tool0 -> depth camera) <<<<<<
# From your URDF joint:
# <origin xyz="0.00203272  0.15910973 -0.03529811" rpy=" 1.479059 0.001322 -3.137201 "/>
URDF_XYZ = (0.00203272, 0.15910973, -0.03529811)  # meters
URDF_RPY = (1.479059, 0.001322, -3.137201)        # radians; roll=X, pitch=Y, yaw=Z (extrinsic XYZ)
# ============================


# ---------- Math helpers ----------

def Rx(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, ca, -sa],
                     [0.0, sa,  ca]], dtype=np.float32)

def Ry(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ ca, 0.0, sa],
                     [0.0, 1.0, 0.0],
                     [-sa, 0.0, ca]], dtype=np.float32)

def Rz(a: float) -> np.ndarray:
    ca, sa = np.cos(a), np.sin(a)
    return np.array([[ca, -sa, 0.0],
                     [sa,  ca, 0.0],
                     [0.0, 0.0, 1.0]], dtype=np.float32)

def rpy_extrinsic_xyz_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    URDF uses extrinsic X-Y-Z (roll, pitch, yaw about fixed axes).
    Equivalent rotation matrix: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """
    return (Rz(yaw) @ Ry(pitch) @ Rx(roll)).astype(np.float32)

def quat_xyzw_to_R(qx, qy, qz, qw) -> np.ndarray:
    x, y, z, w = float(qx), float(qy), float(qz), float(qw)
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1.0 - 2.0*(yy + zz),  2.0*(xy - wz),      2.0*(xz + wy)],
        [2.0*(xy + wz),        1.0 - 2.0*(xx + zz), 2.0*(yz - wx)],
        [2.0*(xz - wy),        2.0*(yz + wx),      1.0 - 2.0*(xx + yy)]
    ], dtype=np.float32)

def pose_to_matrix(pos_xyz: List[float], quat_xyzw: List[float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = quat_xyzw_to_R(*quat_xyzw)
    T[:3, 3]  = np.array(pos_xyz, dtype=np.float32)
    return T

def build_T_tool0_depth_from_urdf(xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = rpy_extrinsic_xyz_to_R(*rpy)
    T[:3, 3] = np.array(xyz, dtype=np.float32)
    return T


# ---------- IO helpers ----------

def load_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r") as f:
        return yaml.safe_load(f)

def load_intrinsics_opencv_yaml(depth_yaml_path: str) -> Tuple[int, int, np.ndarray, np.ndarray]:
    fs = cv2.FileStorage(depth_yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open intrinsics YAML: {depth_yaml_path}")
    image_width_node = fs.getNode("image_width")
    image_height_node = fs.getNode("image_height")
    W = int(image_width_node.real())
    H = int(image_height_node.real())
    K = fs.getNode("camera_matrix").mat().astype(np.float64).reshape(3, 3)
    D = fs.getNode("distortion_coefficients").mat().astype(np.float64).ravel()
    fs.release()
    return W, H, K, D

def precompute_rect_maps(W: int, H: int, K: np.ndarray, D: np.ndarray, alpha: float):
    # alpha=0 crops borders; ~0.5 keeps more FOV with black edges
    K_new, _ = cv2.getOptimalNewCameraMatrix(K, D, (W, H), alpha, (W, H))
    map1, map2 = cv2.initUndistortRectifyMap(K, D, np.eye(3), K_new, (W, H), cv2.CV_32FC1)
    return K_new.astype(np.float64), map1, map2  # K_new as float64 for Open3D Pinhole ctor

def ensure_dir(path: str) -> None:
    d = os.path.dirname(path)
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)


# ---------- Main ----------

def main():
    ap = argparse.ArgumentParser(
        description="TSDF fuse RAW depth frames into a global mesh/point cloud (legacy CPU, robust)."
    )
    ap.add_argument("--manifest", required=True, help="Path to manifest.yaml with captures (depth path + pose_tool0).")
    ap.add_argument("--intrinsics", required=True, help="OpenCV intrinsics YAML (K,D) for the depth camera.")
    ap.add_argument("--depth_scale", type=float, default=1000.0,
                    help="If uint16 in mm use 1000.0; if meters use 1.0.")
    ap.add_argument("--depth_max", type=float, default=2.5,
                    help="Depth truncation (meters) during back-projection.")
    ap.add_argument("--alpha_rect", type=float, default=0.0,
                    help="Undistort alpha (0 crop, 0.5 more FOV).")
    ap.add_argument("--voxel_length", type=float, default=0.005,
                    help="TSDF voxel size (meters).")
    ap.add_argument("--sdf_trunc", type=float, default=0.02,
                    help="TSDF truncation distance (meters).")
    ap.add_argument("--invert_tool0_depth", action="store_true",
                    help="Set if URDF matrix is T_cam_tool0 instead of T_tool0_cam (will invert it).")
    ap.add_argument("--out_pcd", required=True, help="Output fused point cloud PLY (extracted from TSDF).")
    ap.add_argument("--out_mesh", required=True, help="Output fused mesh PLY (extracted from TSDF).")
    args = ap.parse_args()

    manifest = load_yaml(args.manifest)
    base_dir = os.path.dirname(os.path.abspath(args.manifest))

    # Intrinsics & rectification maps
    W, H, K_raw, D = load_intrinsics_opencv_yaml(args.intrinsics)
    K_rect, map1, map2 = precompute_rect_maps(W, H, K_raw, D, args.alpha_rect)

    # Pinhole intrinsics for Open3D (rectified)
    fx = float(K_rect[0, 0])
    fy = float(K_rect[1, 1])
    cx = float(K_rect[0, 2])
    cy = float(K_rect[1, 2])
    intrinsic = o3d.camera.PinholeCameraIntrinsic(width=W, height=H, fx=fx, fy=fy, cx=cx, cy=cy)

    # Fixed hand-eye from URDF: tool0 -> depth camera
    T_tool0_depth = build_T_tool0_depth_from_urdf(URDF_XYZ, URDF_RPY).astype(np.float64)
    if args.invert_tool0_depth:
        T_tool0_depth = np.linalg.inv(T_tool0_depth)

    # TSDF volume (legacy, CPU)
    tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=float(args.voxel_length),
        sdf_trunc=float(args.sdf_trunc),
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8

    )

    used = 0
    captures = manifest.get("captures", [])
    total = len(captures)
    if total == 0:
        raise RuntimeError("No captures in manifest.")

    for cap in captures:
        depth_rel = cap.get("depth")
        pose = cap.get("pose_tool0", {})
        if not depth_rel or not pose:
            continue

        # Resolve depth path (manifest base or nested scan_1/)
        depth_path = os.path.join(base_dir, depth_rel)
        if not os.path.isfile(depth_path):
            alt = os.path.join(base_dir, "scan_1", depth_rel)
            if os.path.isfile(alt):
                depth_path = alt
        if not os.path.isfile(depth_path):
            print(f"[WARN] Missing depth: {depth_rel}")
            continue

        # Load depth
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            print(f"[WARN] Cannot read depth: {depth_path}")
            continue
        if depth.shape[:2] != (H, W):
            print(f"[WARN] Depth size {depth.shape[1]}x{depth.shape[0]} != YAML {W}x{H} (skip)")
            continue

        # Convert to meters
        if depth.dtype == np.uint16 or depth.dtype == np.uint32:
            depth_m = depth.astype(np.float32) / float(args.depth_scale)
        elif depth.dtype == np.float32:
            depth_m = depth
            if args.depth_scale != 1.0:
                depth_m = depth_m / float(args.depth_scale)
        else:
            print(f"[WARN] Unsupported depth dtype: {depth.dtype} (skip)")
            continue

        # Rectify depth (nearest to avoid mixing ranges)
        depth_rect = cv2.remap(
            depth_m, map1, map2,
            interpolation=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0
        ).astype(np.float32)

        # Optional truncation before integration
        if args.depth_max > 0.0:
            mask = (depth_rect > 0.0) & (depth_rect <= float(args.depth_max))
            depth_rect = np.where(mask, depth_rect, 0.0).astype(np.float32)

        # Build Open3D RGBD (dummy color)
        depth_o3d = o3d.geometry.Image(depth_rect)
        color_o3d = o3d.geometry.Image(np.zeros((H, W, 3), dtype=np.uint8))
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
             color_o3d,
             depth_o3d,
             depth_scale=1.0,  # already meters
             depth_trunc=float(args.depth_max) if args.depth_max > 0.0 else 10.0,
             convert_rgb_to_intensity=False
        )

        # Compose camera pose in base/world:
        # From manifest: T_base_tool0 (position + orientation_xyzw)
        pos = pose.get("position")
        quat = pose.get("orientation_xyzw")
        if pos is None or quat is None:
            print(f"[WARN] Missing pose_tool0 fields at index {cap.get('index')}")
            continue
        T_base_tool0 = pose_to_matrix(pos, quat).astype(np.float64)

        # camera-to-world (base): T_base_cam = T_base_tool0 · T_tool0_depth
        T_base_cam = T_base_tool0 @ T_tool0_depth

        # Integrate this frame into TSDF
        tsdf.integrate(rgbd, intrinsic, np.linalg.inv(T_base_cam))

        used += 1
        if used % 10 == 0:
            print(f"[INFO] Integrated {used} / {total} frames into TSDF")

    if used == 0:
        raise RuntimeError("No frames integrated. Check paths and manifest.")

    # Extract results from TSDF
    print("[INFO] Extracting mesh and point cloud from TSDF...")
    mesh = tsdf.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    pcd = tsdf.extract_point_cloud()

    # Save
    ensure_dir(args.out_mesh)
    ensure_dir(args.out_pcd)
    o3d.io.write_triangle_mesh(args.out_mesh, mesh, write_triangle_uvs=False)
    o3d.io.write_point_cloud(args.out_pcd, pcd, write_ascii=False, compressed=False)

    print(f"[OK] Fused mesh saved: {args.out_mesh}")
    print(f"[OK] Fused cloud saved: {args.out_pcd}")


if __name__ == "__main__":
    main()
