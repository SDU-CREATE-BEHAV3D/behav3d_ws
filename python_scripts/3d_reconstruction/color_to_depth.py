# FINAL: Correct remapping + correct colored PLY (no ROS2), built on your last script
#
# What is fixed vs your pasted script:
# 1) Uses the CONFIRMED correct mapping variant:
#       T_color_from_ir = inv(T_tool0_color) @ T_tool0_ir
#    (this matches your "Kd_depth_intrinsics__invTcxTi_overlay.png")
#
# 2) Colors are assigned with a mask that matches the point ordering.
#    - We build the point cloud in WORLD using create_from_depth_image(), then color it by
#      projecting those 3D points into the COLOR camera to sample the original color image.
#    - This avoids all the mask/order mismatch issues.
#
# Outputs for frame 0:
#   - color_in_depth_0000.png
#   - overlay_edges_0000.png
#   - colored_world_pcd_0000.ply
#
# Visualizations:
#   - OpenCV windows: depth, original color, color_in_depth, overlay
#   - Open3D: colored point cloud in world frame

import sys
from pathlib import Path

import numpy as np
import cv2
import open3d as o3d

# set src path for utils import
sys.path.append(str(Path(__file__).resolve().parents[1]))

from utils.session import Session
from utils.manifest import read_manifest, load_robot_poses, construct_image_paths
from utils.intrinsics import load_intrinsics
from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images


SESSION_PATH = "/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/260113_170839"
scan_folder = "manual_caps"
output_folder = Path("/Users/josephnamar/Desktop/SDU/PHD/behav3d/Captures/260113_170839")
my_session = Session(SESSION_PATH, scan_folder)


class ColorToDepthAligner:
    def __init__(self, session, depth_max=1.0, depth_scale=1000.0):
        self.session = session
        self.depth_max = float(depth_max)
        self.depth_scale = float(depth_scale)

        self.manifest = read_manifest(self.session.path, self.session._scan_folder)
        self.T_base_tool0_list = load_robot_poses(self.manifest)

        self.T_tool0_ir = load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir")
        self.T_tool0_color = load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_color")

        self.T_tool0_ir = np.asarray(self.T_tool0_ir, dtype=np.float64)
        self.T_tool0_color = np.asarray(self.T_tool0_color, dtype=np.float64)

        # CONFIRMED correct mapping: "invTcxTi"
        # color <- ir (depth optical)
        self.T_color_from_ir = np.linalg.inv(self.T_tool0_color) @ self.T_tool0_ir

        self.wd, self.hd, self.Kd, self.Dd = load_intrinsics(self.session.depth_intrinsics_path)
        self.wc, self.hc, self.Kc, self.Dc = load_intrinsics(self.session.color_intrinsics_path)

        self.depth_paths = construct_image_paths(self.manifest, self.session, image_type="depth")
        self.color_paths = construct_image_paths(self.manifest, self.session, image_type="color")

        self.depth_images = load_images(self.depth_paths, image_type="depth", library="cv2")
        self.color_images = load_images(self.color_paths, image_type="color", library="cv2")

    # --- Depth->Color projection to build color_in_depth ---
    def _align_color_to_depth(self, depth_u16, color_bgr):
        H, W = depth_u16.shape[:2]
        Hc, Wc = color_bgr.shape[:2]

        z = depth_u16.astype(np.float32) / self.depth_scale
        valid_depth = (z > 0) & (z < self.depth_max)

        u = np.arange(W, dtype=np.float32)
        v = np.arange(H, dtype=np.float32)
        uu, vv = np.meshgrid(u, v)

        fx_d, fy_d = float(self.Kd[0, 0]), float(self.Kd[1, 1])
        cx_d, cy_d = float(self.Kd[0, 2]), float(self.Kd[1, 2])

        Xd = (uu - cx_d) * z / fx_d
        Yd = (vv - cy_d) * z / fy_d
        Zd = z
        pts_d = np.stack([Xd, Yd, Zd], axis=-1).reshape(-1, 3)

        R = self.T_color_from_ir[:3, :3].astype(np.float32)
        t = self.T_color_from_ir[:3, 3].astype(np.float32)

        pts_c = (pts_d @ R.T) + t[None, :]
        Zc = pts_c[:, 2]

        fx_c, fy_c = float(self.Kc[0, 0]), float(self.Kc[1, 1])
        cx_c, cy_c = float(self.Kc[0, 2]), float(self.Kc[1, 2])

        uc = (pts_c[:, 0] / Zc) * fx_c + cx_c
        vc = (pts_c[:, 1] / Zc) * fy_c + cy_c

        uv_map = np.stack([uc, vc], axis=-1).reshape(H, W, 2).astype(np.float32)

        valid_geo = (Zc.reshape(H, W) > 0)
        valid_bounds = (
            (uv_map[..., 0] >= 0) & (uv_map[..., 0] < (Wc - 1)) &
            (uv_map[..., 1] >= 0) & (uv_map[..., 1] < (Hc - 1))
        )
        valid_all = valid_depth & valid_geo & valid_bounds

        map_x = uv_map[..., 0].copy()
        map_y = uv_map[..., 1].copy()
        map_x[~valid_all] = -1
        map_y[~valid_all] = -1

        color_in_depth = cv2.remap(
            color_bgr,
            map_x,
            map_y,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        color_in_depth[~valid_all] = 0
        return color_in_depth, valid_all

    def _depth_edges_overlay(self, depth_u16, color_in_depth_bgr, valid_mask):
        depth_m = depth_u16.astype(np.float32) / self.depth_scale
        depth_m[~valid_mask] = 0.0

        gx = cv2.Sobel(depth_m, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(depth_m, cv2.CV_32F, 0, 1, ksize=3)
        g = np.sqrt(gx * gx + gy * gy)

        edges = (g > 0.01).astype(np.uint8) * 255
        overlay = color_in_depth_bgr.copy()
        overlay[edges > 0] = (0, 0, 255)
        return overlay

    def _visualize_images(self, depth_u16, color_bgr, color_in_depth_bgr, overlay_bgr):
        depth_norm = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        def show(name, img):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            cv2.imshow(name, img)

        show("Depth (colormap)", depth_vis)
        show("Color (original)", color_bgr)
        show("Color aligned to depth", color_in_depth_bgr)
        show("Overlay depth edges (confirmed)", overlay_bgr)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    # --- Robust coloring: project WORLD points into COLOR image (no ordering issues) ---
    def _color_world_pcd_by_projection(self, pcd_world, color_bgr, T_base_ir, T_base_tool0):
        """
        pcd_world: Open3D legacy point cloud in WORLD (base) frame
        color_bgr: original color image
        T_base_ir: base <- ir (same as in your TSDF pose chain)
        T_base_tool0: base <- tool0 for this frame
        """
        # Build base <- color for this frame:
        # base<-color = base<-tool0 @ tool0<-color
        # tool0<-color is inverse of (color<-tool0). But we do not know your convention.
        # We will stay consistent with the alignment convention you validated:
        #   T_color_from_ir = inv(T_tool0_color) @ T_tool0_ir
        #
        # That implies T_tool0_color behaves as (tool0 <- color) or (color <- tool0)?
        # Instead of guessing, we compute color<-ir from the validated expression and
        # then use: base<-color = base<-ir @ inv(color<-ir)
        #
        # base<-ir is T_base_ir, and color<-ir is T_color_from_ir.
        T_color_from_ir = self.T_color_from_ir
        T_base_color = T_base_ir @ np.linalg.inv(T_color_from_ir)

        # Now project each world point into the color camera:
        pts_w = np.asarray(pcd_world.points, dtype=np.float64)
        N = pts_w.shape[0]
        pts_w_h = np.concatenate([pts_w, np.ones((N, 1), dtype=np.float64)], axis=1)

        # color<-base
        T_color_base = np.linalg.inv(T_base_color)
        pts_c_h = (T_color_base @ pts_w_h.T).T
        Xc = pts_c_h[:, 0]
        Yc = pts_c_h[:, 1]
        Zc = pts_c_h[:, 2]

        # valid in front
        valid = Zc > 0

        fx, fy = float(self.Kc[0, 0]), float(self.Kc[1, 1])
        cx, cy = float(self.Kc[0, 2]), float(self.Kc[1, 2])

        u = (Xc / Zc) * fx + cx
        v = (Yc / Zc) * fy + cy

        Hc, Wc = color_bgr.shape[:2]
        valid &= (u >= 0) & (u < Wc) & (v >= 0) & (v < Hc)

        # Sample nearest-neighbor (robust)
        u_i = np.clip(np.round(u).astype(np.int32), 0, Wc - 1)
        v_i = np.clip(np.round(v).astype(np.int32), 0, Hc - 1)

        colors = np.zeros((N, 3), dtype=np.float64)
        # Open3D expects RGB, image is BGR
        sampled_bgr = color_bgr[v_i[valid], u_i[valid], :].astype(np.float64) / 255.0
        sampled_rgb = sampled_bgr[:, ::-1]
        colors[valid] = sampled_rgb

        pcd_world.colors = o3d.utility.Vector3dVector(colors)
        return pcd_world

    def test_one_frame(self, frame_idx=0, visualize=True):
        out_dir = output_folder / "alignment_test"
        out_dir.mkdir(parents=True, exist_ok=True)

        depth0 = self.depth_images[frame_idx]
        color0 = self.color_images[frame_idx]

        if depth0 is None or color0 is None:
            raise RuntimeError(f"Failed to load depth/color for frame {frame_idx}")

        # --- Remap (confirmed correct) ---
        color_in_depth, valid_map = self._align_color_to_depth(depth0, color0)
        overlay = self._depth_edges_overlay(depth0, color_in_depth, valid_map)

        cv2.imwrite(str(out_dir / f"color_in_depth_{frame_idx:04d}.png"), color_in_depth)
        cv2.imwrite(str(out_dir / f"Kd_depth_intrinsics__invTcxTi_overlay_{frame_idx:04d}.png"), overlay)

        # --- Build WORLD point cloud exactly like your TSDF visualization path ---
        T_base_tool0 = self.T_base_tool0_list[frame_idx]
        T_base_ir = T_base_tool0 @ self.T_tool0_ir

        depth_o3d = o3d.geometry.Image(depth0)
        intrinsic_d = o3d.camera.PinholeCameraIntrinsic(
            self.wd, self.hd,
            float(self.Kd[0, 0]), float(self.Kd[1, 1]),
            float(self.Kd[0, 2]), float(self.Kd[1, 2])
        )

        pcd_world = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d,
            intrinsic_d,
            extrinsic=np.linalg.inv(T_base_ir),
            depth_scale=self.depth_scale,
            depth_trunc=self.depth_max,
            project_valid_depth_only=True
        )

        # --- Color the WORLD point cloud by projecting into the ORIGINAL COLOR image ---
        # This guarantees correct correspondence without relying on any pixel-mask ordering.
        pcd_world = self._color_world_pcd_by_projection(pcd_world, color0, T_base_ir, T_base_tool0)

        ply_path = out_dir / f"colored_world_pcd_{frame_idx:04d}.ply"
        o3d.io.write_point_cloud(str(ply_path), pcd_world)

        print("Saved:")
        print(out_dir / f"color_in_depth_{frame_idx:04d}.png")
        print(out_dir / f"Kd_depth_intrinsics__invTcxTi_overlay_{frame_idx:04d}.png")
        print(ply_path)

        if visualize:
            self._visualize_images(depth0, color0, color_in_depth, overlay)
            axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            o3d.visualization.draw([pcd_world, axes])


if __name__ == "__main__":
    aligner = ColorToDepthAligner(my_session, depth_max=1.0, depth_scale=1000.0)

    print(f"Depth frames: {len(aligner.depth_images)}")
    print(f"Color frames: {len(aligner.color_images)}")
    print(f"Robot poses:  {len(aligner.T_base_tool0_list)}")

# aligne all frames
for idx in range(len(aligner.depth_images)):
    aligner.test_one_frame(frame_idx=idx, visualize=False)
    print(f"Processed frame {idx+1}/{len(aligner.depth_images)}")




