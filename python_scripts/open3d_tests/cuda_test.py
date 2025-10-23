#!/usr/bin/env python3
# depth_to_pcd_gpu.py
# Minimal Open3D 0.19 script: depth -> point cloud -> GPU ops (voxel + normals) -> save
# Comments in English only.

import sys
import numpy as np
import open3d as o3d

def make_K(fx, fy, cx, cy, *, device):
    import numpy as np
    K_np = np.array([[fx, 0.0, cx],
                     [0.0, fy, cy],
                     [0.0, 0.0, 1.0]], dtype=np.float32)
    return o3d.core.Tensor(K_np, device=device)

def main():
    if len(sys.argv) < 2:
        print("Usage: python depth_to_pcd_gpu.py <depth.png> [out.ply]")
        print("Assumes depth is in millimeters unless you set depth_scale=1.0 below.")
        sys.exit(1)

    depth_path = sys.argv[1]
    out_path   = sys.argv[2] if len(sys.argv) > 2 else "pc_gpu_processed.ply"

    # ---- Camera intrinsics (edit with your real values) ----
    fx, fy = 505.085205078125, 505.0267028808594   # focal lengths in pixels
    cx, cy = 337.9606, 338.32763671875   # principal point in pixels
    depth_scale = 1000.0    # 1000.0 if depth image is in millimeters, 1.0 if in meters
    depth_trunc = 1.0       # meters

    # ---- Choose devices ----
    dev_cpu  = o3d.core.Device("CPU:0")
    dev_cuda = o3d.core.Device("CUDA:0")

    # ---- Read depth as Tensor Image (CPU) ----
    # Read depth as Tensor Image (CPU)
    img = o3d.t.io.read_image(depth_path)        # -> t.geometry.Image
    T = img.as_tensor()                           # -> core.Tensor

    # Convert to meters as a tensor first
    if T.dtype in (o3d.core.Dtype.UInt16, o3d.core.Dtype.UInt32):
        T = T.to(o3d.core.Dtype.Float32) / depth_scale
    else:
        T = T.to(o3d.core.Dtype.Float32)
        if depth_scale != 1.0:
            T = T / depth_scale

    # Wrap back into a Tensor Image (and make it contiguous)
    depth_m = o3d.t.geometry.Image(T.contiguous())

    # ---- Build intrinsics K (CPU) ----
    K_cpu = make_K(fx, fy, cx, cy, device=dev_cpu)  


    # ---- Try Tensor API to create point cloud from depth on CPU, then move to CUDA ----
    # Some Open3D builds expose create_from_depth_image in t.geometry.PointCloud
    try:
        pcd_t = o3d.t.geometry.PointCloud.create_from_depth_image(
            depth=depth_m,
            intrinsics=K_cpu,  # <-- nombre correcto
            extrinsics=o3d.core.Tensor.eye(4, o3d.core.Dtype.Float32, dev_cpu),  # <-- nombre correcto
            depth_scale=1.0,
            depth_max=depth_trunc,
            stride=1,
            with_normals=False
        )

    except Exception:
        # Fallback: use legacy creation on CPU, then convert to Tensor
        arr = np.asarray(depth_m.as_tensor()).astype(np.float32)
        depth_legacy = o3d.geometry.Image(arr)
        pinhole = o3d.camera.PinholeCameraIntrinsic(
            width=int(arr.shape[1]), height=int(arr.shape[0]), fx=fx, fy=fy, cx=cx, cy=cy
        )

        pcd_legacy = o3d.geometry.PointCloud.create_from_depth_image(
            depth_legacy, pinhole,
            extrinsic=np.eye(4, dtype=np.float32),
            depth_scale=1.0, depth_trunc=depth_trunc, project_valid_depth_only=True
        )
        pcd_t = o3d.t.geometry.PointCloud.from_legacy(pcd_legacy)

    # ---- Move point cloud to GPU (CUDA) ----
    pcd_cuda = pcd_t.to(dev_cuda)

    # ---- GPU ops: voxel downsample + normals ----
    # Choose a voxel size in meters (edit this)
    voxel_size = 0.01
    pcd_cuda = pcd_cuda.voxel_down_sample(voxel_size)

    # Estimate normals on GPU (radius/knn parameters are typical; adjust as needed)
    pcd_cuda.estimate_normals(
        max_nn=30,
        radius=0.05
    )

    # ---- Back to CPU (for saving/visualization using legacy viewer) ----
    pcd_cpu = pcd_cuda.to(dev_cpu)
    pcd_legacy_out = pcd_cpu.to_legacy()

    # Save PLY
    o3d.io.write_point_cloud(out_path, pcd_legacy_out, write_ascii=False, compressed=False)
    print(f"Saved: {out_path}")

    # Optional visualize
    try:
        o3d.visualization.draw_geometries([pcd_legacy_out])
    except Exception:
        pass

if __name__ == "__main__":
    main()
