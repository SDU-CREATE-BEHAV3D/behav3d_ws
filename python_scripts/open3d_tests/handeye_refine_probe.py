#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
handeye_refine_probe.py
Grid-search small adjustments around your current T_tool_ir to reduce multi-frame misalignment.

Metric: mean centroid shift between consecutive world-point-clouds (lower is better).
Output:
  - Top-N candidates with (dRx,dRy,dRz in deg, dTx,dTy,dTz in m) and mean/max shift
  - Best transform printed as a 4x4 matrix + Euler (xyz extrinsic) + translation
  - Blocking overlay visualization of the best candidate vs. baseline

Usage example:
  python handeye_refine_probe.py \
    --session /home/lab/behav3d_ws/captures/251021_115853/scan_1 \
    --frames 8 --depth-trunc 1.2 \
    --rot-range 2 --rot-step 1 \
    --trans-range 0.02 --trans-step 0.01
"""

import argparse
from pathlib import Path
import numpy as np
import yaml, cv2, open3d as o3d
from scipy.spatial.transform import Rotation as R

# ---------- Intrinsics ----------
class CamIntrinsics:
    def __init__(self, w, h, fx, fy, cx, cy):
        self.w, self.h, self.fx, self.fy, self.cx, self.cy = int(w), int(h), float(fx), float(fy), float(cx), float(cy)
    def to_o3d(self):
        return o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.cx, self.cy)

def load_ir_intrinsics(session_dir: Path):
    d1, d2 = session_dir / "depth_intrinsics.yaml", session_dir / "ir_intrinsics.yaml"
    used = d1 if d1.exists() else (d2 if d2.exists() else None)
    if used is None:
        print("⚠️ No intrinsics YAML found — using defaults 640x576 fx≈505 fy≈505 cx≈338 cy≈338.")
        return CamIntrinsics(640, 576, 505.0852, 505.0267, 337.9606, 338.3276), None
    with open(used, "r") as f:
        y = yaml.safe_load(f)
    K = np.array(y["camera_matrix"]["data"], np.float32).reshape(3, 3)
    dist = np.array(y.get("distortion_coefficients", {}).get("data", []), np.float32)
    w, h = int(y["image_width"]), int(y["image_height"])
    intr = CamIntrinsics(w, h, K[0, 0], K[1, 1], K[0, 2], K[1, 2])
    if dist.size > 0 and np.any(np.abs(dist) > 1e-8):
        m1, m2 = cv2.initUndistortRectifyMap(K, dist, None, K, (w, h), cv2.CV_32FC1)
        print(f"✓ Loaded intrinsics ({used.name}) with distortion correction.")
        return intr, (m1, m2)
    print(f"✓ Loaded intrinsics ({used.name}) (no distortion).")
    return intr, None

# ---------- Baseline T_tool_ir (tool0 -> IR) ----------
def make_T_tool_ir():
    # Your current hand–eye from earlier messages
    r_tool_ir = R.from_euler("xyz", [1.4796203267, 0.0010471976, -3.1372642370])
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = r_tool_ir.as_matrix().astype(np.float32)
    T[:3, 3]  = np.array([0.00180271, 0.15873923, -0.03501765], np.float32)
    return T

# ---------- IO ----------
def load_manifest(session_dir: Path):
    mpath = session_dir / "manifest.yaml"
    if not mpath.exists():
        raise FileNotFoundError(f"manifest.yaml missing in {session_dir}")
    with open(mpath, "r") as f:
        return yaml.safe_load(f)

def get_pose_tool0(cap):
    qx, qy, qz, qw = cap["pose_tool0"]["orientation_xyzw"]
    tx, ty, tz = cap["pose_tool0"]["position"]
    Rm = R.from_quat([qx, qy, qz, qw]).as_matrix().astype(np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = Rm
    T[:3, 3]  = np.array([tx, ty, tz], np.float32)
    return T

def make_axes(scale=0.1):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale)

def pcd_from_depth(depth_m, intr, T_cam_to_world):
    depth_img = o3d.geometry.Image(depth_m.astype(np.float32))
    return o3d.geometry.PointCloud.create_from_depth_image(
        depth_img, intr.to_o3d(), T_cam_to_world.astype(np.float64),
        depth_scale=1.0, depth_trunc=float(np.max(depth_m) if np.isfinite(depth_m).any() else 10.0), stride=1
    )

# ---------- Metric ----------
def centroid_shift_metric(pcds):
    """Return (mean_shift, max_shift) over consecutive centroids."""
    cents = []
    for p in pcds:
        pts = np.asarray(p.points)
        if pts.size == 0:
            continue
        cents.append(np.mean(pts, axis=0))
    if len(cents) < 2:
        return np.inf, np.inf
    diffs = [np.linalg.norm(cents[i] - cents[i-1]) for i in range(1, len(cents))]
    return float(np.mean(diffs)), float(np.max(diffs))

# ---------- Apply perturbations ----------
def apply_delta_to_T_tool_ir(T_tool_ir, dR_deg_xyz, dT_m_xyz):
    dR = R.from_euler("xyz", np.array(dR_deg_xyz, dtype=np.float64), degrees=True).as_matrix().astype(np.float32)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = dR @ T_tool_ir[:3, :3]
    T[:3, 3]  = T_tool_ir[:3, 3] + np.array(dT_m_xyz, dtype=np.float32)
    return T

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, type=str)
    ap.add_argument("--frames", type=int, default=8)
    ap.add_argument("--depth-scale", type=float, default=1000.0)
    ap.add_argument("--depth-trunc", type=float, default=1.2)
    ap.add_argument("--rot-range", type=float, default=2.0, help="±deg around each axis")
    ap.add_argument("--rot-step", type=float, default=1.0, help="deg step")
    ap.add_argument("--trans-range", type=float, default=0.02, help="±meters around each axis")
    ap.add_argument("--trans-step", type=float, default=0.01, help="meters step")
    ap.add_argument("--topk", type=int, default=6, help="print top-k candidates")
    args = ap.parse_args()

    session_dir = Path(args.session)
    manifest = load_manifest(session_dir)
    captures = manifest.get("captures", [])
    if not captures:
        raise RuntimeError("No captures in manifest.yaml")

    IR, remaps = load_ir_intrinsics(session_dir)
    T_tool_ir_base = make_T_tool_ir()

    n_total = len(captures)
    idxs = np.linspace(0, n_total - 1, min(args.frames, n_total), dtype=int).tolist()
    print(f"[INFO] Using frames {idxs} of {n_total}")
    print("[INFO] Baseline T_tool_ir translation (m):", T_tool_ir_base[:3,3])

    # Preload depths & poses
    frame_data = []
    for i in idxs:
        cap = captures[i]
        dp = session_dir / cap["depth"]
        d_raw = cv2.imread(str(dp), cv2.IMREAD_UNCHANGED)
        if d_raw is None:
            print(f"⚠️ Cannot read {dp}, skipping frame {i}")
            continue
        if remaps is not None:
            d_raw = cv2.remap(d_raw, remaps[0], remaps[1], cv2.INTER_NEAREST)
        d_m = np.clip(d_raw.astype(np.float32) / float(args.depth_scale), 0.0, float(args.depth_trunc))
        T_base_tool = get_pose_tool0(cap)
        frame_data.append((i, d_m, T_base_tool))

    if len(frame_data) < 2:
        raise RuntimeError("Not enough valid frames for metric.")

    # Search grids
    rot_vals = np.arange(-args.rot_range, args.rot_range + 1e-6, args.rot_step)  # degrees
    trans_vals = np.arange(-args.trans_range, args.trans_range + 1e-9, args.trans_step)  # meters

    candidates = []

    # First, evaluate baseline
    pcds = []
    for _, d_m, T_base_tool in frame_data:
        T_base_cam = T_base_tool @ T_tool_ir_base
        T_c2w = np.linalg.inv(T_base_cam)
        pcds.append(pcd_from_depth(d_m, IR, T_c2w))
    mean_shift, max_shift = centroid_shift_metric(pcds)
    candidates.append(((0,0,0),(0,0,0), mean_shift, max_shift, "BASELINE"))

    # Grid search (coarse but effective)
    print(f"[INFO] Sweeping rotations {rot_vals} deg and translations {trans_vals} m ...")
    for rx in rot_vals:
        for ry in rot_vals:
            for rz in rot_vals:
                for tx in trans_vals:
                    for ty in trans_vals:
                        for tz in trans_vals:
                            if rx==0 and ry==0 and rz==0 and tx==0 and ty==0 and tz==0:
                                continue
                            T_candidate = apply_delta_to_T_tool_ir(T_tool_ir_base, (rx,ry,rz), (tx,ty,tz))
                            pcds = []
                            for _, d_m, T_base_tool in frame_data:
                                T_base_cam = T_base_tool @ T_candidate
                                T_c2w = np.linalg.inv(T_base_cam)
                                pcds.append(pcd_from_depth(d_m, IR, T_c2w))
                            mean_shift, max_shift = centroid_shift_metric(pcds)
                            candidates.append(((rx,ry,rz),(tx,ty,tz), mean_shift, max_shift, ""))

    # Rank
    candidates.sort(key=lambda x: (x[2], x[3]))  # by mean, then max
    print("\n===== Top candidates (lowest mean centroid shift) =====")
    for k, c in enumerate(candidates[:args.topk]):
        (rx,ry,rz),(tx,ty,tz), m, M, tag = c
        tag = tag or f"ΔR(deg)=({rx},{ry},{rz}) ΔT(m)=({tx:.3f},{ty:.3f},{tz:.3f})"
        print(f"{k+1:2d}. {tag}  -> mean {m:.4f} m | max {M:.4f} m")

    # Visualize baseline vs best
    best = candidates[0]
    (brx,bry,brz),(btx,bty,btz), m_best, M_best, _ = best
    T_best = apply_delta_to_T_tool_ir(T_tool_ir_base, (brx,bry,brz), (btx,bty,btz))

    print("\n[BEST] ΔR(deg)=({},{},{})  ΔT(m)=({:.3f},{:.3f},{:.3f})  -> mean {:.4f} | max {:.4f}".format(
        brx,bry,brz,btx,bty,btz,m_best,M_best
    ))

    # Build overlay
    geoms = [make_axes(0.1)]
    for idx, d_m, T_base_tool in frame_data:
        # baseline (blue)
        Tb = T_base_tool @ T_tool_ir_base
        pcd_b = pcd_from_depth(d_m, IR, np.linalg.inv(Tb))
        pcd_b.paint_uniform_color([0.1, 0.3, 1.0])
        geoms.append(pcd_b)
        # best (green)
        Tg = T_base_tool @ T_best
        pcd_g = pcd_from_depth(d_m, IR, np.linalg.inv(Tg))
        pcd_g.paint_uniform_color([0.1, 0.8, 0.1])
        geoms.append(pcd_g)

    o3d.visualization.draw_geometries(
        geoms, window_name="Blue = Baseline  |  Green = Best candidate"
    )

    # Print final 4x4 and Euler for copy-paste
    R_best = R.from_matrix(T_best[:3,:3])
    euler_best = R_best.as_euler("xyz", degrees=True)
    print("\n===== Paste this as your corrected T_tool_ir =====")
    print("T_tool_ir = np.array([")
    for r in range(4):
        row = ", ".join([f"{float(T_best[r,c]): .9f}" for c in range(4)])
        print(f"    [{row}],")
    print("], dtype=np.float32)")
    print("Euler xyz (deg):", euler_best)
    print("Translation (m):", T_best[:3,3])

if __name__ == "__main__":
    main()
