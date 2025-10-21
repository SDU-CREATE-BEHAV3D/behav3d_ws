#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import cv2
import numpy as np
import open3d as o3d
import threading
from pathlib import Path
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from behav3d_interfaces.srv import ReconstructMesh
import yaml


class ThreeDReconstructor(Node):
    def __init__(self):
        super().__init__('three_d_reconstructor')

        # ROS2 service
        self.srv = self.create_service(ReconstructMesh, '/reconstruct_mesh', self.handle_request)

        # Paths
        env_root = os.environ.get('BEHAV3D_CAPTURES_ROOT', '')
        self.captures_root = Path(env_root).expanduser() if env_root else (Path.home() / 'behav3d_ws' / 'captures')
        self.mesh_dir = Path.home() / 'robot' / 'meshes'
        self.mesh_dir.mkdir(parents=True, exist_ok=True)

        self.running = False
        self.get_logger().info("🧠 3D Reconstruction service active: /reconstruct_mesh")
        self.get_logger().info(f"📁 Capture root: {self.captures_root}")

    # =====================================================================
    # SERVICE HANDLER
    # =====================================================================
    def handle_request(self, request, response):
        if self.running:
            response.success = False
            response.message = "Reconstruction already running."
            return response

        session_dir = self._resolve_session_path(request.session_path.strip() if request.session_path else "")

        if not session_dir.exists():
            response.success = False
            response.message = f"Session directory not found: {session_dir}"
            return response

        self.get_logger().info(f"🔍 Reconstructing from session: {session_dir}")

        self.running = True
        thread = threading.Thread(target=self._run_reconstruction, args=(session_dir,), daemon=True)
        thread.start()

        response.success = True
        response.message = f"Reconstruction started for {session_dir}"
        response.mesh_path = str(self.mesh_dir / "tsdf_mesh_refined.stl")
        return response

    # =====================================================================
    def _get_latest_session(self) -> Path:
        subdirs = [d for d in self.captures_root.iterdir() if d.is_dir()]
        if not subdirs:
            self.get_logger().warn("No capture sessions found.")
            return self.captures_root
        return max(subdirs, key=lambda d: d.stat().st_mtime)

    def _resolve_session_path(self, path_arg: str) -> Path:
        p = path_arg.strip() if path_arg else ""
        if p.startswith("@session"):
            active = os.environ.get("BEHAV3D_ACTIVE_SESSION", "")
            if active and Path(active).exists():
                session_root = Path(active)
            else:
                session_root = self._get_latest_session()
            sub = p.replace("@session", "").lstrip("/")
            return session_root / sub if sub else session_root
        if not os.path.isabs(p):
            return (self.captures_root / p).resolve()
        return Path(p).expanduser().resolve()

    # =====================================================================
    def _run_reconstruction(self, session_dir: Path):
        try:
            self.get_logger().info(f"⚙️ Running full TSDF reconstruction for {session_dir}")

            # ===============================
            # === CONFIGURATION ===
            # ===============================
            IR_W, IR_H = 640, 576
            FX, FY, CX, CY = 505.085205078125, 505.0267028808594, 337.96063232421875, 338.32763671875
            INTR_IR = o3d.camera.PinholeCameraIntrinsic(IR_W, IR_H, FX, FY, CX, CY)

            # --- Extrinsics ---
            r_tool_opt = R.from_euler('xyz', [1.583606, 0.007528, -3.141280])
            T_tool_opt = np.eye(4, dtype=np.float32)
            T_tool_opt[:3, :3] = r_tool_opt.as_matrix()
            T_tool_opt[:3, 3] = np.array([-0.03041263, 0.15675367, -0.03452134])

            r_tool_ir = R.from_euler('xyz', [1.479059, 0.001322, -3.137201])
            T_tool_ir = np.eye(4, dtype=np.float32)
            T_tool_ir[:3, :3] = r_tool_ir.as_matrix()
            T_tool_ir[:3, 3] = np.array([00.00203272, 0.15910973, -0.03529811])

            T_ir_to_opt = np.linalg.inv(T_tool_ir) @ T_tool_opt

            # --- Color camera intrinsics ---
            COLOR_W, COLOR_H = 1920, 1080
            FX_C, FY_C, CX_C, CY_C = 1121.1506, 1120.6919, 950.5190, 539.1617
            DIST_COEFFS = np.array([0.07194096, -0.1016186, -0.0003443, 0.0000462, 0.04160475])

            # --- Reconstruction parameters ---
            VOXEL_SIZE = 0.0011
            SDF_TRUNC = 0.005
            DEPTH_SCALE = 1000.0
            DEPTH_TRUNC = 0.9
            NUM_SCANS = 12
            USE_MEDIAN_BLUR = False

            # ===================================
            # === LOAD MANIFEST & CAPTURES ===
            # ===================================
            manifest_path = session_dir / "manifest.yaml"
            if not manifest_path.exists():
                self.get_logger().error(f"❌ No manifest.yaml found in {session_dir}")
                return

            with open(manifest_path, "r") as f:
                manifest = yaml.safe_load(f)

            captures = manifest.get("captures", [])
            if not captures:
                self.get_logger().error("❌ No captures found in manifest.")
                return

            n_total = len(captures)
            step = max(1, n_total // NUM_SCANS)
            subset = captures[::step][:NUM_SCANS]
            self.get_logger().info(f"Integrating {len(subset)} of {n_total} frames...")

            tsdf = o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=VOXEL_SIZE,
                sdf_trunc=SDF_TRUNC,
                color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8,
            )

            # ===================================================
            # === CORE FUSION LOOP WITH TRUE ALIGNMENT ===
            # ===================================================
            for cap in subset:
                idx = cap["index"]
                color_path = session_dir / cap["rgb"]
                depth_path = session_dir / cap["depth"]

                color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
                depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
                if color is None or depth is None:
                    self.get_logger().warn(f"⚠️ Missing frame {idx}")
                    continue

                # Undistort color image
                K_color = np.array([[FX_C, 0, CX_C], [0, FY_C, CY_C], [0, 0, 1]], dtype=np.float32)
                color = cv2.undistort(color, K_color, DIST_COEFFS)
                color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

                # Depth preprocessing
                if USE_MEDIAN_BLUR:
                    depth = cv2.medianBlur(depth, 3)
                depth_m = depth.astype(np.float32) / DEPTH_SCALE

                # Pose
                if "pose_tool0" not in cap:
                    self.get_logger().warn(f"⚠️ Missing pose for frame {idx}")
                    continue
                q = cap["pose_tool0"]["orientation_xyzw"]
                t = cap["pose_tool0"]["position"]
                T_base_tool = np.eye(4, dtype=np.float32)
                T_base_tool[:3, :3] = R.from_quat(q).as_matrix()
                T_base_tool[:3, 3] = np.array(t)
                T_base_camIR = (T_base_tool @ T_tool_ir).astype(np.float32)

                # === DEPTH → COLOR reprojection ===
                h, w = depth_m.shape
                u, v = np.meshgrid(np.arange(w), np.arange(h))
                z = depth_m
                x = (u - CX) * z / FX
                y = (v - CY) * z / FY
                pts_ir = np.stack((x, y, z, np.ones_like(z)), axis=-1).reshape(-1, 4).T
                pts_color = (T_ir_to_opt @ pts_ir)[:3, :]
                u_c = (FX_C * pts_color[0, :] / pts_color[2, :]) + CX_C
                v_c = (FY_C * pts_color[1, :] / pts_color[2, :]) + CY_C

                aligned_color = np.zeros((h, w, 3), dtype=np.uint8)
                valid = (u_c >= 0) & (u_c < COLOR_W - 1) & (v_c >= 0) & (v_c < COLOR_H - 1)
                u_c_valid = u_c[valid]
                v_c_valid = v_c[valid]
                u0, v0 = np.floor(u_c_valid).astype(int), np.floor(v_c_valid).astype(int)
                du, dv = u_c_valid - u0, v_c_valid - v0

                for c_idx in range(3):
                    I00 = color[v0, u0, c_idx]
                    I01 = color[v0, np.clip(u0 + 1, 0, COLOR_W - 1), c_idx]
                    I10 = color[np.clip(v0 + 1, 0, COLOR_H - 1), u0, c_idx]
                    I11 = color[np.clip(v0 + 1, 0, COLOR_H - 1), np.clip(u0 + 1, 0, COLOR_W - 1), c_idx]
                    aligned = (
                        I00 * (1 - du) * (1 - dv)
                        + I01 * du * (1 - dv)
                        + I10 * (1 - du) * dv
                        + I11 * du * dv
                    )
                    aligned_color.reshape(-1, 3)[valid, c_idx] = aligned.astype(np.uint8)

                # Create Open3D RGBD image
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    o3d.geometry.Image(aligned_color),
                    o3d.geometry.Image((depth_m * DEPTH_SCALE).astype(np.uint16)),
                    depth_scale=DEPTH_SCALE,
                    depth_trunc=DEPTH_TRUNC,
                    convert_rgb_to_intensity=False,
                )

                tsdf.integrate(rgbd, INTR_IR, np.linalg.inv(T_base_camIR))

            # ===================================================
            # === POSTPROCESSING ===
            # ===================================================
            mesh = tsdf.extract_triangle_mesh()
            mesh.compute_vertex_normals()
            mesh = mesh.filter_smooth_taubin(number_of_iterations=20)
            mesh = mesh.simplify_vertex_clustering(voxel_size=VOXEL_SIZE * 4)
            mesh.compute_vertex_normals()

            output_path = self.mesh_dir / "tsdf_mesh_refined.stl"
            o3d.io.write_triangle_mesh(str(output_path), mesh, write_ascii=False)
            self.get_logger().info(f"✅ Mesh saved at: {output_path}")

        except Exception as e:
            self.get_logger().error(f"Reconstruction failed: {e}")
        finally:
            self.running = False


def main(args=None):
    rclpy.init(args=args)
    node = ThreeDReconstructor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
