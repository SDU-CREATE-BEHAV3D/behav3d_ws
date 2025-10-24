#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import datetime
from pathlib import Path
import numpy as np
import yaml, cv2, open3d as o3d
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt


# ---------- INTRINSICS ----------
class CamIntrinsics:
    def __init__(self, w, h, fx, fy, cx, cy):
        self.w, self.h, self.fx, self.fy, self.cx, self.cy = int(w), int(h), fx, fy, cx, cy
    def to_o3d(self):
        return o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.cx, self.cy)


def load_ir_intrinsics(session_dir: Path):
    dpath = session_dir / "depth_intrinsics.yaml"
    if not dpath.exists():
        dpath = session_dir / "ir_intrinsics.yaml"
    if not dpath.exists():
        print("⚠️ No intrinsics YAML found — using built-in defaults (640x576 fx≈505 fy≈505 cx≈338 cy≈338).")
        return CamIntrinsics(640, 576, 505.0852, 505.0267, 337.9606, 338.3276), None
    with open(dpath, "r") as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"]["data"], np.float32).reshape(3, 3)
    dist = np.array(y.get("distortion_coefficients", {}).get("data", []), np.float32)
    w, h = int(y["image_width"]), int(y["image_height"])
    intr = CamIntrinsics(w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    if dist.size > 0 and np.any(np.abs(dist) > 1e-8):
        m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_32FC1)
        print("✓ Loaded intrinsics with distortion correction.")
        return intr, (m1, m2)
    print("✓ Loaded intrinsics (no distortion).")
    return intr, None


# ---------- FIXED T_tool_ir ----------
def make_T_tool_ir():
    r_tool_ir = R.from_euler("xyz", [1.4796203267, 0.0010471976, -3.1372642370])
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = r_tool_ir.as_matrix()
    T[:3, 3] = np.array([0.00180271, 0.15873923, -0.03501765], np.float32)
    return T


# ---------- UTILITIES ----------
def get_pose_tool0(cap):
    qx, qy, qz, qw = cap["pose_tool0"]["orientation_xyzw"]
    tx, ty, tz = cap["pose_tool0"]["position"]
    Rm = R.from_quat([qx, qy, qz, qw]).as_matrix()
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = Rm
    T[:3, 3] = np.array([tx, ty, tz], np.float32)
    return T


def make_axes(scale=0.1):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)


def pcd_from_depth(depth_m, intr, T_cam_to_world):
    depth_img = o3d.geometry.Image(depth_m.astype(np.float32))
    return o3d.geometry.PointCloud.create_from_depth_image(
        depth_img, intr.to_o3d(),
        T_cam_to_world.astype(np.float64),
        depth_scale=1.0, depth_trunc=float(np.max(depth_m))
    )


# ---------- MAIN ----------
def main():
    session_dir = Path("/home/lab/behav3d_ws/captures/251021_115853/scan_1")
    manifest_path = session_dir / "manifest.yaml"
    if not manifest_path.exists():
        raise FileNotFoundError("Missing manifest.yaml")

    with open(manifest_path, "r") as f:
        manifest = yaml.safe_load(f)
    captures = manifest["captures"]
    idxs = np.linspace(0, len(captures) - 1, min(8, len(captures)), dtype=int)
    print(f"[INFO] Using frames {list(idxs)}")

    IR, remaps = load_ir_intrinsics(session_dir)
    T_tool_ir = make_T_tool_ir()
    T_tool_ir_inv = np.linalg.inv(T_tool_ir)

    print("\n[CHECK] T_tool_ir forward translation:", T_tool_ir[:3, 3])
    print("[CHECK] T_tool_ir inverse translation:", T_tool_ir_inv[:3, 3])

    # Visualize both
    geoms_fwd = [make_axes(0.1)]
    geoms_inv = [make_axes(0.1)]
    centroids_fwd, centroids_inv = [], []

    for i in idxs:
        cap = captures[i]
        depth_path = session_dir / cap["depth"]
        depth_raw = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
        if depth_raw is None:
            continue
        if remaps is not None:
            depth_raw = cv2.remap(depth_raw, remaps[0], remaps[1], cv2.INTER_NEAREST)
        depth_m = np.clip(depth_raw.astype(np.float32) / 1000.0, 0.0, 1.2)

        T_base_tool = get_pose_tool0(cap)

        # --- Forward transform ---
        T_base_cam = T_base_tool @ T_tool_ir
        T_cam_to_world = np.linalg.inv(T_base_cam)
        pcd_fwd = pcd_from_depth(depth_m, IR, T_cam_to_world)
        pcd_fwd.paint_uniform_color([0.1, 0.3, 1.0])  # blue
        geoms_fwd.append(pcd_fwd)
        pts = np.asarray(pcd_fwd.points)
        if pts.size > 0: centroids_fwd.append(np.mean(pts, axis=0))

        # --- Inverse transform ---
        T_base_cam_i = T_base_tool @ T_tool_ir_inv
        T_cam_to_world_i = np.linalg.inv(T_base_cam_i)
        pcd_inv = pcd_from_depth(depth_m, IR, T_cam_to_world_i)
        pcd_inv.paint_uniform_color([1.0, 0.1, 0.1])  # red
        geoms_inv.append(pcd_inv)
        pts_i = np.asarray(pcd_inv.points)
        if pts_i.size > 0: centroids_inv.append(np.mean(pts_i, axis=0))

    # --- Show overlays ---
    print("\n[DEBUG] Showing both calibration directions...")
    o3d.visualization.draw_geometries(geoms_fwd + geoms_inv,
        window_name="Blue: Forward | Red: Inverted")

    # --- Quantify centroid shifts ---
    def centroid_drift(cs):
        if len(cs) < 2: return 0, 0
        dists = [np.linalg.norm(cs[i] - cs[i-1]) for i in range(1, len(cs))]
        return np.mean(dists), np.max(dists)

    mean_f, max_f = centroid_drift(centroids_fwd)
    mean_i, max_i = centroid_drift(centroids_inv)
    print(f"\n[RESULTS]")
    print(f"Forward T_tool_ir → Mean shift {mean_f:.4f} m | Max shift {max_f:.4f} m")
    print(f"Inverse T_tool_ir → Mean shift {mean_i:.4f} m | Max shift {max_i:.4f} m")

    # --- Save overlay screenshots manually from GUI if needed ---
    print("\n[INSTRUCTIONS] Inspect visually:")
    print(" • Blue (Forward) should align object correctly if calibration direction is right.")
    print(" • Red (Inverse) should look wrong — or vice versa.")
    print("The version with *tighter overlap and smaller centroid shift* is your correct T_tool_ir direction.")


if __name__ == "__main__":
    main()
