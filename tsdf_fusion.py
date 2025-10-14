#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import cv2
import argparse
import numpy as np
import open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# =========================
# CONFIG (edit if needed)
# =========================
SESSION_DIR   = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
MANIFEST_PATH = SESSION_DIR / "manifest.json"

# --- IR intrinsics (depth sensor; 640x576) ---
IR_W, IR_H = 640, 576
IR_FX = 505.085205078125
IR_FY = 505.0267028808594
IR_CX = 337.96063232421875
IR_CY = 338.32763671875
INTR_IR = o3d.camera.PinholeCameraIntrinsic(IR_W, IR_H, IR_FX, IR_FY, IR_CX, IR_CY)

# --- Color intrinsics (AT CALIBRATION RESOLUTION, not necessarily your image size) ---
COL_FX = 505.085205078125
COL_FY = 505.0267028808594
COL_CX = 337.96063232421875
COL_CY = 338.32763671875
COL_REF_W, COL_REF_H = 640, 576   # <- the resolution where the above COL_* were calibrated

# --- Hand-eye (tool0→opt, tool0→ir) from your URDF ---
r_tool_opt = R.from_euler('xyz', [1.599,  0.002,  -3.139])
T_tool_opt = np.eye(4, dtype=np.float32)
T_tool_opt[:3,:3] = r_tool_opt.as_matrix().astype(np.float32)
T_tool_opt[:3, 3] = np.array([-0.03158908, 0.1747814,  -0.07149736], dtype=np.float32)

r_tool_ir  = R.from_euler('xyz', [1.496, -0.005,  -3.140])
T_tool_ir  = np.eye(4, dtype=np.float32)
T_tool_ir[:3,:3] = r_tool_ir.as_matrix().astype(np.float32)
T_tool_ir[:3, 3] = np.array([ 0.00028979, 0.17421966, -0.07240817], dtype=np.float32)

# IR→OPT (both wrt tool0)
T_ir_to_opt = np.linalg.inv(T_tool_ir) @ T_tool_opt

# --- TSDF defaults for quick tests (tiny subset) ---
VOXEL_SIZE  = 0.0015
SDF_TRUNC   = 0.015     # ~10× voxel
DEPTH_SCALE = 1000.0    # mm→m (adjust if your depth exporter uses different units)
DEPTH_TRUNC = 0.7

# --- Filtering ---
USE_MEDIAN_BLUR = False
USE_BILATERAL   = False

# --- Pre-crop (disabled in debug to avoid masking errors) ---
ENABLE_PRE_CROP = False
CROP_BOUNDS = dict(x=(-0.3, 1.9), y=(-1.7, 0.6), z=(-2.0, 0.4))

# =========================
# Helpers
# =========================
def quat_to_R(q, assume_xyzw: bool):
    q = np.asarray(q, dtype=np.float32)
    if assume_xyzw:
        return R.from_quat(q).as_matrix().astype(np.float32)
    else:
        # Interpret stored as wxyz
        qwxyz = np.array([q[3], q[0], q[1], q[2]], dtype=np.float32)
        return R.from_quat(qwxyz).as_matrix().astype(np.float32)

def make_T_base_tool(pose, assume_xyzw: bool):
    T = np.eye(4, dtype=np.float32)
    T[:3,:3] = quat_to_R(pose["orientation_xyzw"], assume_xyzw)
    T[:3, 3] = np.array(pose["position"], dtype=np.float32)
    return T

def load_color_depth(idx):
    cpath = SESSION_DIR / "color_raw" / f"color_t{idx}.png"
    dpath = SESSION_DIR / "depth_raw" / f"depth_t{idx}.png"
    color = cv2.imread(str(cpath), cv2.IMREAD_COLOR)
    if color is None:
        raise FileNotFoundError(cpath)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    depth = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(dpath)
    if depth.dtype != np.uint16:
        depth = depth.astype(np.uint16)
    return color, depth

def reproject_color_to_ir(color_rgb, depth_ir_u16):
    """No RGB resize. Build a remap from IR pixels to native color pixels using IR→OPT extrinsics."""
    depth = depth_ir_u16.copy()
    if USE_MEDIAN_BLUR:
        depth = cv2.medianBlur(depth, 3)
    if USE_BILATERAL:
        depth = cv2.bilateralFilter(depth, d=5, sigmaColor=20, sigmaSpace=20)

    depth_m = depth.astype(np.float32) / DEPTH_SCALE
    h, w = depth_m.shape
    ys, xs = np.meshgrid(np.arange(h, dtype=np.float32),
                         np.arange(w, dtype=np.float32), indexing="ij")
    z = depth_m
    valid = z > 0

    # IR backprojection
    X = (xs - IR_CX) * z / IR_FX
    Y = (ys - IR_CY) * z / IR_FY
    Z = z

    Xv, Yv, Zv = X[valid], Y[valid], Z[valid]
    Pir = np.stack([Xv, Yv, Zv, np.ones_like(Xv)], axis=0)

    # Transform IR→OPT
    Popt = T_ir_to_opt @ Pir
    Xo, Yo, Zo = Popt[0], Popt[1], Popt[2]

    # Scale color intrinsics to native color resolution
    Hc, Wc = color_rgb.shape[:2]
    sx, sy = Wc / float(COL_REF_W), Hc / float(COL_REF_H)
    fx_c, fy_c = COL_FX * sx, COL_FY * sy
    cx_c, cy_c = COL_CX * sx, COL_CY * sy

    # Project into color image
    u = (Xo * fx_c) / Zo + cx_c
    v = (Yo * fy_c) / Zo + cy_c

    # Build remap maps (IR size, indexes into native color)
    map_x = np.full((h, w), -1, dtype=np.float32)
    map_y = np.full((h, w), -1, dtype=np.float32)

    in_front  = Zo > 0
    u, v      = u[in_front], v[in_front]
    valid_idx = np.flatnonzero(valid)[in_front]

    in_bounds = (u >= 0) & (u < Wc) & (v >= 0) & (v < Hc)
    u, v      = u[in_bounds], v[in_bounds]
    valid_idx = valid_idx[in_bounds]

    map_y.flat[valid_idx] = v.astype(np.float32)
    map_x.flat[valid_idx] = u.astype(np.float32)

    color_aligned = cv2.remap(
        color_rgb, map_x, map_y,
        interpolation=cv2.INTER_LANCZOS4,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0,0,0)
    )
    return color_aligned, depth

def debug_overlay(color_aligned, depth_u16, out_png):
    depth_8 = np.zeros_like(depth_u16, dtype=np.uint8)
    nz = depth_u16 > 0
    if np.any(nz):
        depth_norm = np.clip(depth_u16.astype(np.float32) / (DEPTH_TRUNC * DEPTH_SCALE) * 255.0, 0, 255)
        depth_8 = depth_norm.astype(np.uint8)

    d_edges = cv2.Canny(depth_8, 40, 120)
    c_edges = cv2.Canny(cv2.cvtColor(color_aligned, cv2.COLOR_RGB2GRAY), 60, 180)

    d_edges_bgr = np.stack([np.zeros_like(d_edges), d_edges, np.zeros_like(d_edges)], axis=-1)  # green
    c_edges_bgr = np.stack([c_edges, np.zeros_like(c_edges), np.zeros_like(c_edges)], axis=-1)  # red

    overlay = cv2.addWeighted(color_aligned, 0.6, d_edges_bgr, 0.9, 0)
    overlay = cv2.addWeighted(overlay, 1.0, c_edges_bgr, 0.9, 0)
    cv2.imwrite(str(out_png), cv2.cvtColor(overlay, cv2.COLOR_RGB2BGR))

def make_marker(pos, color, r=0.005):
    m = o3d.geometry.TriangleMesh.create_sphere(radius=r)
    m.paint_uniform_color(color)
    m.translate(pos)
    return m

# =========================
# Main debug flow
# =========================
def main():
    ap = argparse.ArgumentParser(description="Step-by-step reprojection/TSDF debugger")
    ap.add_argument("--idx", type=int, default=None, help="Frame index to debug (default: midpoint)")
    ap.add_argument("--num_scans", type=int, default=5, help="Small subset for TSDF A/B tests")
    ap.add_argument("--assume_xyzw", action="store_true", help="Interpret manifest quaternions as [x,y,z,w]")
    ap.add_argument("--use_world_T_cam", action="store_true", help="Pass extrinsic as world_T_cam (not inverted) to TSDF")
    args = ap.parse_args()

    os.environ.setdefault("OMP_NUM_THREADS", "8")

    with open(MANIFEST_PATH, "r") as f:
        manifest = json.load(f)
    captures = manifest["captures"]
    n_total  = len(captures)
    print(f"\n=== MANIFEST ===\nTotal frames: {n_total}")

    # Pick frame
    if args.idx is None:
        mid = captures[len(captures)//2]["index"]
        idx = mid
    else:
        idx = args.idx
    cap = next(c for c in captures if c["index"] == idx)
    print(f"Debug frame index: {idx}")

    # Load images
    color_rgb, depth_ir = load_color_depth(idx)
    Hc, Wc = color_rgb.shape[:2]
    print(f"Color size: {Wc}x{Hc} ; Depth size: {depth_ir.shape[1]}x{depth_ir.shape[0]}")
    print(f"Color intrinsics ref: {COL_REF_W}x{COL_REF_H}")
    print(f"Depth stats (u16): nz_min={depth_ir[depth_ir>0].min() if np.any(depth_ir>0) else 0}, "
          f"median={np.median(depth_ir[depth_ir>0]) if np.any(depth_ir>0) else 0}, "
          f"max={depth_ir.max()}  (DEPTH_SCALE={DEPTH_SCALE})")

    # Build IR camera in base
    T_base_tool  = make_T_base_tool(cap["pose_tool0"], args.assume_xyzw)
    T_base_camIR = (T_base_tool @ T_tool_ir).astype(np.float32)

    # === STEP 1: Reprojection sanity + overlay ===
    print("\n[STEP 1] Reproject color→IR (no RGB resize) and make overlay…")
    color_aligned, depth_filtered = reproject_color_to_ir(color_rgb, depth_ir)
    overlay_png = SESSION_DIR / f"dbg_overlay_t{idx}.png"
    debug_overlay(color_aligned, depth_filtered, overlay_png)
    print(f"Overlay saved: {overlay_png}")
    print("Visually inspect: red (color edges) vs green (depth edges) should coincide.")

    # === STEP 2: Single-frame point cloud in base ===
    print("\n[STEP 2] Single-frame point cloud in BASE frame…")
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        o3d.geometry.Image(color_aligned.astype(np.uint8)),
        o3d.geometry.Image(depth_filtered),
        depth_scale=DEPTH_SCALE,
        depth_trunc=DEPTH_TRUNC,
        convert_rgb_to_intensity=False
    )
    pcd1 = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, INTR_IR)
    pcd1.transform(T_base_camIR)  # camera->base
    pts = np.asarray(pcd1.points)
    center = pts.mean(axis=0) if pts.size else np.array([np.nan, np.nan, np.nan])
    print(f"Single-frame points: {len(pts)} ; centroid_base: {center}")
    pcd1_path = SESSION_DIR / f"dbg_single_frame_t{idx}.ply"
    o3d.io.write_point_cloud(str(pcd1_path), pcd1)
    print(f"Saved: {pcd1_path}")

    # === STEP 3: TSDF A/B test on extrinsic direction ===
    # tiny subset around debug frame
    print("\n[STEP 3] TSDF A/B: extrinsic direction (world_T_cam vs inverse)…")
    # select small subset evenly spaced
    step = max(1, n_total // args.num_scans)
    subset = captures[::step][:args.num_scans]
    subset_idx = [c["index"] for c in subset]
    print(f"Subset ({len(subset_idx)}): {subset_idx}")

    def fuse_subset(use_world_T_cam: bool, tag: str):
        tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=VOXEL_SIZE, sdf_trunc=SDF_TRUNC,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
        )
        for c in subset:
            i = c["index"]
            col, dep = load_color_depth(i)
            colA, depF = reproject_color_to_ir(col, dep)
            Tbt = make_T_base_tool(c["pose_tool0"], args.assume_xyzw) @ T_tool_ir
            extr = Tbt if use_world_T_cam else np.linalg.inv(Tbt)
            rgbd_i = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(colA.astype(np.uint8)),
                o3d.geometry.Image(depF),
                depth_scale=DEPTH_SCALE, depth_trunc=DEPTH_TRUNC,
                convert_rgb_to_intensity=False
            )
            tsdf.integrate(rgbd_i, INTR_IR, extr)
        pcdf = tsdf.extract_point_cloud()
        pts = np.asarray(pcdf.points)
        ctr = pts.mean(axis=0) if pts.size else np.array([np.nan, np.nan, np.nan])
        return pcdf, ctr

    pcd_wTc, ctr_wTc = fuse_subset(True,  "wTc")
    pcd_inv,  ctr_inv = fuse_subset(False, "inv")

    outA = SESSION_DIR / "dbg_tsdf_wTc.ply"
    outB = SESSION_DIR / "dbg_tsdf_inv.ply"
    o3d.io.write_point_cloud(str(outA), pcd_wTc)
    o3d.io.write_point_cloud(str(outB), pcd_inv)
    print(f"wTc  points: {len(pcd_wTc.points)} ; centroid: {ctr_wTc} -> {outA}")
    print(f"inv  points: {len(pcd_inv.points)}  ; centroid: {ctr_inv}  -> {outB}")
    print("Pick the one that looks geometrically correct and sharper.")

    # === STEP 4: Quaternion order A/B (only if still fuzzy) ===
    print("\n[STEP 4] Quaternion order A/B (xyzw vs wxyz) for the single frame…")
    # build base_T_cam with both interpretations and report baseline delta
    T_base_tool_xyzw = make_T_base_tool(cap["pose_tool0"], True)
    T_base_tool_wxyz = make_T_base_tool(cap["pose_tool0"], False)
    dR = T_base_tool_xyzw[:3,:3].T @ T_base_tool_wxyz[:3,:3]
    ang = np.degrees(np.arccos(np.clip((np.trace(dR)-1)/2, -1.0, 1.0)))
    dT  = np.linalg.norm(T_base_tool_xyzw[:3,3] - T_base_tool_wxyz[:3,3])
    print(f"R delta (deg): {ang:.3f} ; t delta (m): {dT:.4f}")
    print("If this is large (>1–2° or >2–3 mm), your quaternion order assumption is likely wrong.")

    print("\n=== DONE ===")
    print("Share:")
    print(" 1) overlay PNG path + a photo of how it looks")
    print(" 2) single-frame centroid + whether it looks crisp in MeshLab")
    print(" 3) which TSDF cloud (wTc vs inv) looks correct")
    print(" 4) quaternion delta numbers")
    print("Then I’ll tell you the exact fix.")

if __name__ == "__main__":
    main()
