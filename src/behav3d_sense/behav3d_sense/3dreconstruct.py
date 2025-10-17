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

        if request.session_path.strip():
            session_dir = Path(request.session_path.strip()).expanduser()
        elif request.use_latest:
            session_dir = self._get_latest_session()
        else:
            session_dir = self._get_latest_session()

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

    # =====================================================================
    def _run_reconstruction(self, session_dir: Path):
        try:
            self.get_logger().info(f"⚙️ Running full TSDF reconstruction for {session_dir}")

            # ===============================
            # === CONFIG (copied exactly) ===
            # ===============================
            IR_W, IR_H = 640, 576
            FX, FY, CX, CY = 505.085205078125, 505.0267028808594, 337.96063232421875, 338.32763671875
            INTR_IR = o3d.camera.PinholeCameraIntrinsic(IR_W, IR_H, FX, FY, CX, CY)

            r_tool_opt = R.from_euler('xyz', [1.599, 0.002, -3.139])
            T_tool_opt = np.eye(4, dtype=np.float32)
            T_tool_opt[:3, :3] = r_tool_opt.as_matrix()
            T_tool_opt[:3, 3] = np.array([-0.03158908, 0.1747814, -0.07149736])

            r_tool_ir = R.from_euler('xyz', [1.496, -0.005, -3.140])
            T_tool_ir = np.eye(4, dtype=np.float32)
            T_tool_ir[:3, :3] = r_tool_ir.as_matrix()
            T_tool_ir[:3, 3] = np.array([0.00028979, 0.17421966, -0.07240817])

            T_ir_to_opt = np.linalg.inv(T_tool_ir) @ T_tool_opt

            VOXEL_SIZE = 0.0012
            SDF_TRUNC = 0.008
            DEPTH_SCALE = 1000.0
            DEPTH_TRUNC = 0.7
            NUM_SCANS = 9
            USE_MEDIAN_BLUR = True
            ENABLE_PRE_CROP = True
            CROP_BOUNDS = dict(x=(-0.3, 1.9), y=(-1.7, 0.6), z=(-2.0, 0.4))

            # ===================================
            # === LOAD MANIFEST & CAPTURES ===
            # ===================================
            manifest_path = session_dir / "manifest.json"
            if not manifest_path.exists():
                self.get_logger().error(f"❌ No manifest.json found in {session_dir}")
                return

            with open(manifest_path, "r") as f:
                manifest = json.load(f)
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
            # === CORE FUSION LOOP (exactly like tsdf_fusion) ===
            # ===================================================
            for cap in subset:
                idx = cap["index"]
                color_path = session_dir / "color_raw" / f"color_t{idx}.png"
                depth_path = session_dir / "depth_raw" / f"depth_t{idx}.png"

                color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
                depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)

                if color is None or depth is None:
                    self.get_logger().warn(f"⚠️ Missing frame {idx}")
                    continue

                color = cv2.cvtColor(color, cv2.COLOR_BGR2RGB)

                # === PREPROCESSING ===
                if USE_MEDIAN_BLUR:
                    depth = cv2.medianBlur(depth, 3)

                depth_m = depth.astype(np.float32) / DEPTH_SCALE

                # === Build transformation ===
                q = cap["pose_tool0"]["orientation_xyzw"]
                t = cap["pose_tool0"]["position"]
                T_base_tool = np.eye(4, dtype=np.float32)
                T_base_tool[:3, :3] = R.from_quat(q).as_matrix()
                T_base_tool[:3, 3] = np.array(t)
                T_base_camIR = (T_base_tool @ T_tool_ir).astype(np.float32)

                # === ALIGN COLOR TO IR ===
                color_res = cv2.resize(color, (IR_W, IR_H), interpolation=cv2.INTER_AREA)
                rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                    o3d.geometry.Image(color_res),
                    o3d.geometry.Image((depth_m * DEPTH_SCALE).astype(np.uint16)),
                    depth_scale=DEPTH_SCALE,
                    depth_trunc=DEPTH_TRUNC,
                    convert_rgb_to_intensity=False
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
