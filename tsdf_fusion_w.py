#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import cv2
import numpy as np
import open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# =========================================================
# CONFIG
# =========================================================
SESSION_DIR   = Path("/home/lab/behav3d_ws/captures/session-20251003_113628_mancap")
MANIFEST_PATH = SESSION_DIR / "manifest.json"

# Use IR depth resolution as fusion resolution
IR_W, IR_H = 640, 576
FX = 505.085205078125
FY = 505.0267028808594
CX = 337.96063232421875
CY = 338.32763671875
INTR_IR = o3d.camera.PinholeCameraIntrinsic(IR_W, IR_H, FX, FY, CX, CY)

# Hand-eye from your latest URDF (tool0 -> optical, tool0 -> IR)
r_tool_opt = R.from_euler('xyz', [1.599,  0.002,  -3.139])
T_tool_opt = np.eye(4, dtype=np.float32)
T_tool_opt[:3,:3] = r_tool_opt.as_matrix().astype(np.float32)
T_tool_opt[:3, 3] = np.array([-0.03158908, 0.1747814,  -0.07149736], dtype=np.float32)

r_tool_ir  = R.from_euler('xyz', [1.496, -0.005,  -3.140])
T_tool_ir  = np.eye(4, dtype=np.float32)
T_tool_ir[:3,:3] = r_tool_ir.as_matrix().astype(np.float32)
T_tool_ir[:3, 3] = np.array([ 0.00028979, 0.17421966, -0.07240817], dtype=np.float32)

# IR→OPT (both defined wrt tool0)
T_ir_to_opt = np.linalg.inv(T_tool_ir) @ T_tool_opt

# TSDF params (tuned for better quality without killing CPU)
VOXEL_SIZE = 0.0011   # 1.5 mm
SDF_TRUNC  = 0.007   # 1.5 cm
DEPTH_SCALE = 900.0
DEPTH_TRUNC = 0.6

# Frame selection
NUM_SCANS = 9  # fuse a subset, evenly spaced

# Depth pre-filtering (cheap + effective)
USE_MEDIAN_BLUR   = True   # 3x3 median
USE_BILATERAL     = False  # set True if surfaces are noisy; increases CPU

# Per-frame 3D crop BEFORE fusion (strongly recommended)
ENABLE_PRE_CROP   = True
CROP_BOUNDS = dict(  # base frame AABB
    x=(-0.3, 1.9),
    y=(-1.7, 0.6),
    z=(-2.0, 0.4),
)

# Post visualization
SHOW_POINTCLOUD   = True
SHOW_MESH         = False  # set True to also view a smoothed mesh

# =========================================================
# HELPERS
# =========================================================
def quat_to_R(q):
    return R.from_quat(q).as_matrix().astype(np.float32)

def make_T_base_tool(pose):
    T = np.eye(4, dtype=np.float32)
    T[:3,:3] = quat_to_R(pose["orientation_xyzw"])
    T[:3, 3] = np.array(pose["position"], dtype=np.float32)
    return T

def load_color_depth(idx):
    cpath = SESSION_DIR / "color_raw" / f"color_t{idx}.png"
    dpath = SESSION_DIR / "depth_raw" / f"depth_t{idx}.png"
    color = cv2.imread(str(cpath), cv2.IMREAD_COLOR)  # BGR
    if color is None:
        raise FileNotFoundError(cpath)
    color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)
    depth = cv2.imread(str(dpath), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise FileNotFoundError(dpath)
    if depth.dtype != np.uint16:
        depth = depth.astype(np.uint16)
    return color, depth

def reproject_color_to_ir(color_rgb, depth_ir_u16, T_ir2opt, fx=FX, fy=FY, cx=CX, cy=CY):
    """
    Reproject color (optical) to IR pixel grid using depth in IR + IR->OPT transform.
    Returns color_aligned (HxW x 3, uint8) at IR resolution.
    """
    # Resize color → IR resolution for consistent sampling
    color_res = cv2.resize(color_rgb, (IR_W, IR_H), interpolation=cv2.INTER_AREA)

    # Optional depth pre-filtering
    depth = depth_ir_u16.copy()
    if USE_MEDIAN_BLUR:
        depth = cv2.medianBlur(depth, 3)
    if USE_BILATERAL:
        # Bilateral expects 8/16 bit; use conservative params
        depth = cv2.bilateralFilter(depth, d=5, sigmaColor=20, sigmaSpace=20)

    depth_m = depth.astype(np.float32) / DEPTH_SCALE
    h, w   = depth_m.shape

    # Backproject in IR
    ys, xs = np.meshgrid(np.arange(h, dtype=np.float32), np.arange(w, dtype=np.float32), indexing="ij")
    z = depth_m
    valid = z > 0

    X = (xs - cx) * z / fx
    Y = (ys - cy) * z / fy
    Z = z

    Xv, Yv, Zv = X[valid], Y[valid], Z[valid]
    ones = np.ones_like(Xv, dtype=np.float32)
    Pir = np.stack([Xv, Yv, Zv, ones], axis=0)  # 4xN

    # Transform IR → OPT
    Popt = T_ir2opt @ Pir
    Xo, Yo, Zo = Popt[0, :], Popt[1, :], Popt[2, :]

    # Project into optical pixels (same intrinsics after resize)
    u = (Xo * fx) / Zo + cx
    v = (Yo * fy) / Zo + cy

    # Build remap with bounds checks
    map_x = np.full((h, w), -1, dtype=np.float32)
    map_y = np.full((h, w), -1, dtype=np.float32)

    in_front  = Zo > 0
    u, v      = u[in_front], v[in_front]
    valid_idx = np.flatnonzero(valid)[in_front]

    in_bounds = (u >= 0) & (u < w) & (v >= 0) & (v < h)
    u, v      = u[in_bounds], v[in_bounds]
    valid_idx = valid_idx[in_bounds]

    map_y.flat[valid_idx] = v.astype(np.float32)
    map_x.flat[valid_idx] = u.astype(np.float32)

    # Higher-quality resampling for color (slightly costlier but worth it)
    color_aligned = cv2.remap(
        color_res, map_x, map_y,
        interpolation=cv2.INTER_CUBIC,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0)
    )
    return color_aligned, depth  # return possibly filtered depth (uint16)

def mask_depth_outside_aabb(depth_u16, T_base_cam_ir, bounds=CROP_BOUNDS):
    """
    Per-frame crop: zero out depth pixels outside base-frame AABB (cheap and effective).
    """
    depth_m = depth_u16.astype(np.float32) / DEPTH_SCALE
    h, w = depth_m.shape
    if not np.any(depth_m > 0):
        return depth_u16  # nothing to do

    ys, xs = np.meshgrid(np.arange(h, dtype=np.float32), np.arange(w, dtype=np.float32), indexing="ij")
    z = depth_m
    valid = z > 0

    X = (xs - CX) * z / FX
    Y = (ys - CY) * z / FY
    Z = z

    # stack valid points
    Xv, Yv, Zv = X[valid], Y[valid], Z[valid]
    ones = np.ones_like(Xv, dtype=np.float32)
    P_ir = np.stack([Xv, Yv, Zv, ones], axis=0)

    # IR -> base
    P_base = (T_base_cam_ir @ P_ir).astype(np.float32)
    Xb, Yb, Zb = P_base[0, :], P_base[1, :], P_base[2, :]

    inside = (
        (Xb > bounds["x"][0]) & (Xb < bounds["x"][1]) &
        (Yb > bounds["y"][0]) & (Yb < bounds["y"][1]) &
        (Zb > bounds["z"][0]) & (Zb < bounds["z"][1])
    )

    depth_out = depth_u16.copy()
    flat = depth_out.reshape(-1)
    idx_valid = np.flatnonzero(valid)
    # zero everything outside AABB
    flat[idx_valid[~inside]] = 0
    return depth_out

# =========================================================
# MAIN
# =========================================================
def main():
    # Optional: cap OpenMP threads (keeps CPU civilized)
    os.environ.setdefault("OMP_NUM_THREADS", "8")

    with open(MANIFEST_PATH, "r") as f:
        manifest = json.load(f)
    captures = manifest["captures"]
    n_total  = len(captures)
    step     = max(1, n_total // NUM_SCANS)
    subset   = captures[::step][:NUM_SCANS]

    print(f"Total frames: {n_total}")
    print(f"Using {len(subset)} frames: {[c['index'] for c in subset]}")
    print("TSDF backend: CPU (classic)")

    tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=VOXEL_SIZE,
        sdf_trunc=SDF_TRUNC,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8
    )

    for i, cap in enumerate(subset):
        idx = cap["index"]
        color_rgb, depth_ir = load_color_depth(idx)

        # camera extrinsics in base for IR
        T_base_tool  = make_T_base_tool(cap["pose_tool0"])
        T_base_camIR = (T_base_tool @ T_tool_ir).astype(np.float32)

        # reproject color→IR (also get filtered depth)
        color_aligned, depth_filtered = reproject_color_to_ir(color_rgb, depth_ir, T_ir_to_opt)

        # optional: 3D crop BEFORE integration (zero depth outside workspace)
        if ENABLE_PRE_CROP:
            depth_filtered = mask_depth_outside_aabb(depth_filtered, T_base_camIR)

        # build RGBD
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(color_aligned.astype(np.uint8)),
            o3d.geometry.Image(depth_filtered),
            depth_scale=DEPTH_SCALE,
            depth_trunc=DEPTH_TRUNC,
            convert_rgb_to_intensity=False
        )

        # integrate (classic API expects world_T_cam inverse)
        tsdf.integrate(rgbd, INTR_IR, np.linalg.inv(T_base_camIR))
        print(f"Integrated {i+1}/{len(subset)}")

    # extract
    pcd = tsdf.extract_point_cloud()
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    print(f"Fused points: {len(pcd.points)}")

    geoms = [pcd]

    if SHOW_MESH:
        mesh = tsdf.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        # a tiny amount of smoothing + decimation for nicer display
        mesh = mesh.filter_smooth_simple(number_of_iterations=1)
        mesh = mesh.simplify_vertex_clustering(voxel_size=VOXEL_SIZE*1.5)
        geoms.append(mesh)

    o3d.visualization.draw_geometries(geoms, window_name="TSDF (CPU) — Reprojected RGB→IR, Denoised + Cropped")

    # === Extract refined mesh ===
    mesh = tsdf.extract_triangle_mesh()
    mesh.compute_vertex_normals()

    # Optional: smooth & simplify
    mesh = mesh.filter_smooth_taubin(number_of_iterations=20)
    mesh = mesh.simplify_vertex_clustering(voxel_size=VOXEL_SIZE * 4)
    mesh.compute_vertex_normals()

    # Save
    o3d.io.write_triangle_mesh("tsdf_mesh_refined.ply", mesh, write_ascii=False)
    print(f"✅ Mesh exported: {len(mesh.vertices)} vertices, {len(mesh.triangles)} faces")

    # Extract point cloud
    pcd = tsdf.extract_point_cloud()

    # === Compute geometric center ===
    vertices = np.asarray(mesh.vertices)
    center_geo = vertices.mean(axis=0)
    print(f"🟢 Mesh geometric center: {center_geo}")

    # --- Visualize original mesh + center ---
    def make_marker(pos, color):
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.005)
        sphere.paint_uniform_color(color)
        sphere.translate(pos)
        return sphere

    geo_sphere = make_marker(center_geo, [0, 1, 0])  # green

    o3d.visualization.draw_geometries(
        [mesh, pcd, geo_sphere],
        window_name="Mesh with Center Marker",
        mesh_show_back_face=True
    )
    # save the mesh
    o3d.io.write_triangle_mesh("tsdf_mesh_refined.stl", mesh, write_ascii=False)

    # === Crop mesh around its geometric center ===
    box_size = np.array([0.35, 0.35, 0.3])  # [X, Y, Z] crop dimensions (meters)
    half_size = box_size / 2.0

    min_bound = center_geo - half_size
    max_bound = center_geo + half_size

    crop_box = o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound)
    crop_box.color = (1, 0, 0)

    mesh_cropped = mesh.crop(crop_box)
    mesh_cropped.compute_vertex_normals()
    pcd_cropped = pcd.crop(crop_box)

    print(f"🔻 Cropped mesh: {len(mesh_cropped.vertices)} vertices, {len(mesh_cropped.triangles)} faces")

    # === Compute new center of cropped mesh ===
    if len(mesh_cropped.vertices) > 0:
        cropped_center = np.asarray(mesh_cropped.vertices).mean(axis=0)
        print(f"🟢 Cropped mesh center: {cropped_center}")
    else:
        cropped_center = center_geo
        print("⚠️ Cropped mesh is empty; using original center.")

    # Visualize cropped mesh + new center
    cropped_center_marker = make_marker(cropped_center, [1, 0, 0])  # red

    o3d.visualization.draw_geometries(
        [mesh_cropped, crop_box, cropped_center_marker],
        window_name="Cropped Mesh + New Center",
        mesh_show_back_face=True
    )



if __name__ == "__main__":
    main()

