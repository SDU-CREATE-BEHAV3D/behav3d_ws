# o3d_gpu_reconstruction_fixed.py

import os
import json
import yaml
import cv2
import numpy as np
import open3d as o3d
import open3d.core as o3c
from scipy.spatial.transform import Rotation as R


# -------- intrinsics / extrinsics --------
def load_camera_intrinsics(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(file_path)
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise IOError(f"Cannot open {file_path}")
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    K = fs.getNode("camera_matrix").mat()
    fs.release()
    return o3d.camera.PinholeCameraIntrinsic(
        w, h, K[0,0], K[1,1], K[0,2], K[1,2]
    )

def load_camera_extrinsics(file_path, frame_key="tool0_to_ir"):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(file_path)
    ext = os.path.splitext(file_path)[1].lower()
    if ext in [".json"]:
        with open(file_path, "r") as f:
            d = json.load(f)
        t = np.array(d["translation_m"])
        q = np.array(d["quaternion_xyzw"])
        Rm = R.from_quat(q).as_matrix()
        T_cam_to_tool = np.eye(4); T_cam_to_tool[:3,:3]=Rm; T_cam_to_tool[:3,3]=t
        return T_cam_to_tool
    elif ext in [".yaml", ".yml"]:
        with open(file_path, "r") as f:
            d = yaml.safe_load(f)
        frame = d["frames"][frame_key]
        t = np.array(frame["xyz"])
        q = np.array(frame["quat_xyzw"])
        Rm = R.from_quat(q).as_matrix()
        T_tool_to_cam = np.eye(4); T_tool_to_cam[:3,:3]=Rm; T_tool_to_cam[:3,3]=t
        # YAML is tool->camera; we need camera->tool
        return np.linalg.inv(T_tool_to_cam)
    else:
        raise ValueError(f"Unsupported: {ext}")


# -------- manifest parsing --------
def load_robot_poses_manifest(file_path):
    if not os.path.isfile(file_path):
        raise FileNotFoundError(file_path)
    with open(file_path, "r") as f:
        man = yaml.safe_load(f)
    base = man["session"]["path"]
    out = []
    for cap in man["captures"]:
        pose = cap.get("pose_tool0", {})  # << correct key
        t = np.array(pose.get("position", [0,0,0]), dtype=float)
        q = np.array(pose.get("orientation_xyzw", [0,0,0,1]), dtype=float)
        Rm = R.from_quat(q).as_matrix()
        T_tool_to_base = np.eye(4)
        T_tool_to_base[:3,:3] = Rm
        T_tool_to_base[:3, 3] = t
        out.append({
            "index": cap.get("index", None),
            "timestamp": cap.get("timestamp"),
            "depth_path": os.path.join(base, cap.get("depth")),
            "pose_matrix": T_tool_to_base
        })
    print(f"{len(out)} captures parsed.")
    return out

def transform_robot_to_camera_pose(captures, T_cam_to_tool):
    for c in captures:
        c["camera_pose"] = c["pose_matrix"] @ T_cam_to_tool
    return captures

def sanity_check_motion(captures):
    if len(captures) < 2: return
    deltas = []
    for i in range(len(captures)-1):
        a = captures[i]["camera_pose"][:3,3]
        b = captures[i+1]["camera_pose"][:3,3]
        deltas.append(float(np.linalg.norm(b-a)))
    print("Pose translation deltas (first 5):", np.round(deltas[:5], 6))
    if np.max(deltas) < 1e-8:
        raise RuntimeError("All poses are identical (check manifest key / frames).")


# -------- GPU TSDF (depth-only) --------
class TSDFIntegrationGPU:
    def __init__(self, intrinsic,
                 voxel_size=0.002,      # start coarser to tame noise
                 block_count=120000,
                 block_resolution=16,
                 depth_max=3.0,
                 depth_scale=1000.0,    # use 1.0 if your depth is already in meters
                 device='CUDA:0'):
        self.device = o3c.Device(device)
        self.voxel_size = voxel_size
        self.depth_max = depth_max
        self.depth_scale = depth_scale
        self.width = intrinsic.width
        self.height = intrinsic.height
        # K on CPU for transform indexer
        self.K_cpu = o3c.Tensor(np.array(intrinsic.intrinsic_matrix),
                                o3c.Dtype.Float64, o3c.Device("CPU:0"))
        self.vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=('tsdf','weight'),
            attr_dtypes=(o3c.float32, o3c.float32),
            attr_channels=((1),(1)),
            voxel_size=self.voxel_size,
            block_resolution=block_resolution,
            block_count=block_count,
            device=self.device
        )

    def _tensorize_depth(self, depth_path):
        d = o3d.io.read_image(depth_path)
        if d is None: return None
        dn = np.asarray(d)
        if dn.ndim != 2:
            raise RuntimeError(f"Depth not single-channel: {depth_path}")
        # optional denoise to reduce speckle
        # dn = cv2.medianBlur(dn.astype(np.uint16), 5)
        h, w = dn.shape
        if (w, h) != (self.width, self.height):
            dn = cv2.resize(dn, (self.width, self.height), interpolation=cv2.INTER_NEAREST)
        return o3d.t.geometry.Image(o3c.Tensor(dn, o3c.Dtype.UInt16, self.device))

    def _tensorize_extrinsic_cpu(self, T_cam_in_world):
        return o3c.Tensor(np.linalg.inv(T_cam_in_world), o3c.Dtype.Float64, o3c.Device("CPU:0"))

    def integrate_frame(self, T_cam_in_world, depth_path):
        t_depth = self._tensorize_depth(depth_path)
        if t_depth is None: return False
        Tcw_cpu = self._tensorize_extrinsic_cpu(T_cam_in_world)
        blocks = self.vbg.compute_unique_block_coordinates(
            t_depth, self.K_cpu, Tcw_cpu, self.depth_scale, self.depth_max)
        self.vbg.integrate(blocks, t_depth, self.K_cpu, Tcw_cpu, self.depth_scale, self.depth_max)
        return True

    def integrate_captures(self, captures):
        ok = 0
        for i, c in enumerate(captures):
            print(f"Integrating {i+1}/{len(captures)} ...", end="")
            if self.integrate_frame(c["camera_pose"], c["depth_path"]):
                ok += 1
                print("done.")
            else:
                print("skipped.")
        return ok

    def extract_mesh(self, out_path=None):
        m = self.vbg.extract_triangle_mesh().to_legacy()
        m.compute_vertex_normals()
        if out_path:
            o3d.io.write_triangle_mesh(out_path, m)
        return m


# -------- main --------
if __name__ == "__main__":
    # ---- set your dataset root ----
    base_dir = "/home/lab/behav3d_ws/captures/251103_170035"
    intrinsics_path = os.path.join(base_dir, "config/depth_intrinsics.yaml")
    extrinsics_path = os.path.join(base_dir, "config/extrinsics.yaml")
    manifest_path   = os.path.join(base_dir, "scan_charuco/manifest.yaml")

    intrinsic = load_camera_intrinsics(intrinsics_path)
    T_cam_to_tool = load_camera_extrinsics(extrinsics_path, frame_key="tool0_to_ir")
    captures = load_robot_poses_manifest(manifest_path)
    captures = transform_robot_to_camera_pose(captures, T_cam_to_tool)

    # sanity: make sure poses actually move
    sanity_check_motion(captures)

    # depth unit sanity (uncomment to verify)
    # d0 = np.asarray(o3d.io.read_image(captures[0]["depth_path"]))
    # print("Depth range:", int(np.nanmin(d0)), int(np.nanmax(d0)))

    fuser = TSDFIntegrationGPU(
        intrinsic=intrinsic,
        voxel_size=0.008,     # try 0.006→0.010 depending on noise
        depth_scale=1000.0,   # set to 1.0 if your depth is in meters
        depth_max=1.0,
        device='CUDA:0'
    )

    n_ok = fuser.integrate_captures(captures)
    print(f"Integrated {n_ok}/{len(captures)} frames.")

    mesh = fuser.extract_mesh(out_path=os.path.join(base_dir, "mesh_gpu.stl"))
    print("Saved mesh to:", os.path.join(base_dir, "mesh_gpu.ply"))

    origin = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
    o3d.visualization.draw([mesh, origin])
