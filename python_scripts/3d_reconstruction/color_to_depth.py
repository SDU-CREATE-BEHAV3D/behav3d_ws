#!/usr/bin/env python3
"""
Color->depth alignment utility with diagnostics.

Modes:
- process: write aligned images (and optional overlays/PLY)
- diagnose: compare transform conventions and report which one aligns best
"""

import argparse
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import open3d as o3d

# set src path for utils import
sys.path.append(str(Path(__file__).resolve().parents[1]))

from utils.extrinsics import load_extrinsics
from utils.image_loader import load_images
from utils.intrinsics import load_intrinsics
from utils.manifest import construct_image_paths, load_robot_poses, read_manifest
from utils.session import Session


DEFAULT_SESSION_PATH = "/home/lab/behav3d_ws/captures/260223_112846"
DEFAULT_SCAN_FOLDER = "trial_manual_caps"
DEFAULT_OUTPUT_DIR = None
DEFAULT_CONVENTION = "inv_Tc_Ti"


class ColorToDepthAligner:
    def __init__(
        self,
        session: Session,
        output_dir: Path,
        depth_max: float = 1.0,
        depth_scale: float = 1000.0,
    ):
        self.session = session
        self.output_dir = Path(output_dir)
        self.depth_max = float(depth_max)
        self.depth_scale = float(depth_scale)

        self.manifest = read_manifest(self.session.path, self.session._scan_folder)
        self.T_base_tool0_list = load_robot_poses(self.manifest)

        self.T_tool0_ir = np.asarray(
            load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_ir"),
            dtype=np.float64,
        )
        self.T_tool0_color = np.asarray(
            load_extrinsics(self.session._camera_extrinsics_path, frame_key="T_tool0_color"),
            dtype=np.float64,
        )

        self.wd, self.hd, self.Kd, self.Dd = load_intrinsics(self.session.depth_intrinsics_path)
        self.wc, self.hc, self.Kc, self.Dc = load_intrinsics(self.session.color_intrinsics_path)

        self.depth_paths = construct_image_paths(self.manifest, self.session, image_type="depth")
        self.color_paths = construct_image_paths(self.manifest, self.session, image_type="color")

        self.depth_images = load_images(self.depth_paths, image_type="depth", library="cv2")
        self.color_images = load_images(self.color_paths, image_type="color", library="cv2")

        if len(self.depth_images) != len(self.color_images):
            raise RuntimeError(
                f"Depth/color count mismatch: {len(self.depth_images)} vs {len(self.color_images)}"
            )

    def _make_T_color_from_ir(self, convention: str) -> np.ndarray:
        Ti = self.T_tool0_ir
        Tc = self.T_tool0_color

        if convention == "inv_Tc_Ti":
            return np.linalg.inv(Tc) @ Ti
        if convention == "Tc_inv_Ti":
            return Tc @ np.linalg.inv(Ti)
        if convention == "inv_inv_Tc_Ti":
            return np.linalg.inv(np.linalg.inv(Tc) @ Ti)
        if convention == "inv_Tc_inv_Ti":
            return np.linalg.inv(Tc @ np.linalg.inv(Ti))

        raise ValueError(
            "Unknown convention. Use one of: "
            "inv_Tc_Ti, Tc_inv_Ti, inv_inv_Tc_Ti, inv_Tc_inv_Ti"
        )

    def _align_color_to_depth(
        self,
        depth_u16: np.ndarray,
        color_bgr: np.ndarray,
        T_color_from_ir: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
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

        R = T_color_from_ir[:3, :3].astype(np.float32)
        t = T_color_from_ir[:3, 3].astype(np.float32)
        pts_c = (pts_d @ R.T) + t[None, :]
        Zc = pts_c[:, 2]

        fx_c, fy_c = float(self.Kc[0, 0]), float(self.Kc[1, 1])
        cx_c, cy_c = float(self.Kc[0, 2]), float(self.Kc[1, 2])

        uc = (pts_c[:, 0] / Zc) * fx_c + cx_c
        vc = (pts_c[:, 1] / Zc) * fy_c + cy_c
        uv_map = np.stack([uc, vc], axis=-1).reshape(H, W, 2).astype(np.float32)

        valid_geo = Zc.reshape(H, W) > 0
        valid_bounds = (
            (uv_map[..., 0] >= 0)
            & (uv_map[..., 0] < (Wc - 1))
            & (uv_map[..., 1] >= 0)
            & (uv_map[..., 1] < (Hc - 1))
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

    def _depth_edges_overlay(
        self,
        depth_u16: np.ndarray,
        color_in_depth_bgr: np.ndarray,
        valid_mask: np.ndarray,
    ) -> np.ndarray:
        depth_m = depth_u16.astype(np.float32) / self.depth_scale
        depth_m[~valid_mask] = 0.0

        gx = cv2.Sobel(depth_m, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(depth_m, cv2.CV_32F, 0, 1, ksize=3)
        g = np.sqrt(gx * gx + gy * gy)

        edges = (g > 0.01).astype(np.uint8)
        overlay = color_in_depth_bgr.copy()
        overlay[edges > 0] = (0, 0, 255)
        return overlay

    @staticmethod
    def _color_gradient(gray: np.ndarray) -> np.ndarray:
        gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
        gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
        return np.sqrt(gx * gx + gy * gy)

    def _alignment_metrics(
        self,
        depth_u16: np.ndarray,
        color_in_depth_bgr: np.ndarray,
        valid_mask: np.ndarray,
    ) -> Dict[str, float]:
        depth_m = depth_u16.astype(np.float32) / self.depth_scale
        depth_m[~valid_mask] = 0.0

        depth_grad = self._color_gradient(depth_m)
        edges = depth_grad > 0.01

        gray = cv2.cvtColor(color_in_depth_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
        color_grad = self._color_gradient(gray)

        score = float(np.mean(color_grad[edges])) if np.any(edges) else 0.0
        valid_frac = float(np.mean(valid_mask))
        edge_frac = float(np.mean(edges))

        return {
            "edge_score": score,
            "valid_frac": valid_frac,
            "edge_frac": edge_frac,
        }

    def _save_world_colored_pcd(
        self,
        frame_idx: int,
        color_bgr: np.ndarray,
        convention: str,
        stem_prefix: str,
    ) -> Path:
        T_color_from_ir = self._make_T_color_from_ir(convention)

        T_base_tool0 = self.T_base_tool0_list[frame_idx]
        T_base_ir = T_base_tool0 @ self.T_tool0_ir

        depth_u16 = self.depth_images[frame_idx]
        depth_o3d = o3d.geometry.Image(depth_u16)
        intrinsic_d = o3d.camera.PinholeCameraIntrinsic(
            self.wd,
            self.hd,
            float(self.Kd[0, 0]),
            float(self.Kd[1, 1]),
            float(self.Kd[0, 2]),
            float(self.Kd[1, 2]),
        )

        pcd_world = o3d.geometry.PointCloud.create_from_depth_image(
            depth_o3d,
            intrinsic_d,
            extrinsic=np.linalg.inv(T_base_ir),
            depth_scale=self.depth_scale,
            depth_trunc=self.depth_max,
            project_valid_depth_only=True,
        )

        T_base_color = T_base_ir @ np.linalg.inv(T_color_from_ir)
        pts_w = np.asarray(pcd_world.points, dtype=np.float64)
        N = pts_w.shape[0]
        pts_w_h = np.concatenate([pts_w, np.ones((N, 1), dtype=np.float64)], axis=1)

        T_color_base = np.linalg.inv(T_base_color)
        pts_c_h = (T_color_base @ pts_w_h.T).T
        Xc, Yc, Zc = pts_c_h[:, 0], pts_c_h[:, 1], pts_c_h[:, 2]

        fx, fy = float(self.Kc[0, 0]), float(self.Kc[1, 1])
        cx, cy = float(self.Kc[0, 2]), float(self.Kc[1, 2])
        u = (Xc / Zc) * fx + cx
        v = (Yc / Zc) * fy + cy

        Hc, Wc = color_bgr.shape[:2]
        valid = (Zc > 0) & (u >= 0) & (u < Wc) & (v >= 0) & (v < Hc)
        u_i = np.clip(np.round(u).astype(np.int32), 0, Wc - 1)
        v_i = np.clip(np.round(v).astype(np.int32), 0, Hc - 1)

        colors = np.zeros((N, 3), dtype=np.float64)
        sampled_bgr = color_bgr[v_i[valid], u_i[valid], :].astype(np.float64) / 255.0
        colors[valid] = sampled_bgr[:, ::-1]
        pcd_world.colors = o3d.utility.Vector3dVector(colors)

        out_ply = self.output_dir / f"{stem_prefix}colored_world_pcd_{frame_idx:04d}.ply"
        o3d.io.write_point_cloud(str(out_ply), pcd_world)
        return out_ply

    def process_frame(
        self,
        frame_idx: int,
        *,
        convention: str,
        save_overlay: bool,
        save_ply: bool,
        visualize: bool,
        stem_prefix: str = "",
    ) -> Dict[str, float]:
        depth_u16 = self.depth_images[frame_idx]
        color_bgr = self.color_images[frame_idx]
        if depth_u16 is None or color_bgr is None:
            raise RuntimeError(f"Failed to load depth/color for frame {frame_idx}")

        T_color_from_ir = self._make_T_color_from_ir(convention)
        color_in_depth, valid_mask = self._align_color_to_depth(depth_u16, color_bgr, T_color_from_ir)
        metrics = self._alignment_metrics(depth_u16, color_in_depth, valid_mask)

        out_img = self.output_dir / f"{stem_prefix}color_in_depth_{frame_idx:04d}.png"
        cv2.imwrite(str(out_img), color_in_depth)

        if save_overlay or visualize:
            overlay = self._depth_edges_overlay(depth_u16, color_in_depth, valid_mask)
            out_overlay = self.output_dir / f"{stem_prefix}overlay_edges_{frame_idx:04d}.png"
            if save_overlay:
                cv2.imwrite(str(out_overlay), overlay)
        else:
            overlay = None

        out_ply = None
        if save_ply:
            out_ply = self._save_world_colored_pcd(
                frame_idx,
                color_bgr,
                convention,
                stem_prefix,
            )

        if visualize:
            self._visualize_images(depth_u16, color_bgr, color_in_depth, overlay)
            if save_ply and out_ply is not None:
                pcd = o3d.io.read_point_cloud(str(out_ply))
                axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
                o3d.visualization.draw([pcd, axes])

        return metrics

    def _visualize_images(
        self,
        depth_u16: np.ndarray,
        color_bgr: np.ndarray,
        color_in_depth_bgr: np.ndarray,
        overlay_bgr: Optional[np.ndarray],
    ):
        depth_norm = cv2.normalize(depth_u16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)

        cv2.namedWindow("Depth (colormap)", cv2.WINDOW_NORMAL)
        cv2.imshow("Depth (colormap)", depth_vis)

        cv2.namedWindow("Color (original)", cv2.WINDOW_NORMAL)
        cv2.imshow("Color (original)", color_bgr)

        cv2.namedWindow("Color aligned to depth", cv2.WINDOW_NORMAL)
        cv2.imshow("Color aligned to depth", color_in_depth_bgr)

        if overlay_bgr is not None:
            cv2.namedWindow("Overlay depth edges", cv2.WINDOW_NORMAL)
            cv2.imshow("Overlay depth edges", overlay_bgr)

        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def process_all(
        self,
        *,
        convention: str,
        max_frames: Optional[int],
        save_overlay: bool,
        save_ply: bool,
        visualize: bool,
    ):
        n = len(self.depth_images)
        if max_frames is not None:
            n = min(n, int(max_frames))

        self.output_dir.mkdir(parents=True, exist_ok=True)
        print(f"Processing {n} frames with convention='{convention}'")

        for idx in range(n):
            metrics = self.process_frame(
                idx,
                convention=convention,
                save_overlay=save_overlay,
                save_ply=save_ply,
                visualize=visualize,
            )
            print(
                f"frame {idx:04d}: edge_score={metrics['edge_score']:.3f} "
                f"valid_frac={metrics['valid_frac']:.3f} edge_frac={metrics['edge_frac']:.3f}"
            )

    def diagnose(
        self,
        *,
        max_frames: Optional[int],
        conventions: List[str],
        save_examples: int,
        save_overlay: bool,
    ):
        n = len(self.depth_images)
        if max_frames is not None:
            n = min(n, int(max_frames))

        self.output_dir.mkdir(parents=True, exist_ok=True)

        print("\n=== Dataset Stats ===")
        self._print_depth_stats(n)

        per_conv: Dict[str, List[Dict[str, float]]] = {c: [] for c in conventions}
        winners: Dict[str, int] = {c: 0 for c in conventions}

        print("\n=== Per-frame Best Convention ===")
        for idx in range(n):
            frame_scores: Dict[str, float] = {}

            for c in conventions:
                stem = f"diag_{c}_" if idx < save_examples else ""
                metrics = self.process_frame(
                    idx,
                    convention=c,
                    save_overlay=save_overlay and (idx < save_examples),
                    save_ply=False,
                    visualize=False,
                    stem_prefix=stem,
                )
                per_conv[c].append(metrics)
                frame_scores[c] = metrics["edge_score"]

            best = max(frame_scores.items(), key=lambda kv: kv[1])[0]
            winners[best] += 1
            scores_text = " ".join([f"{k}:{v:.2f}" for k, v in frame_scores.items()])
            print(f"frame {idx:04d} best={best} | {scores_text}")

        print("\n=== Convention Summary ===")
        for c in conventions:
            arr_score = np.array([m["edge_score"] for m in per_conv[c]], dtype=float)
            arr_valid = np.array([m["valid_frac"] for m in per_conv[c]], dtype=float)
            arr_edge = np.array([m["edge_frac"] for m in per_conv[c]], dtype=float)
            print(
                f"{c:16s} "
                f"score_mean={arr_score.mean():.3f} score_med={np.median(arr_score):.3f} "
                f"valid_mean={arr_valid.mean():.3f} edge_mean={arr_edge.mean():.3f}"
            )

        print("\n=== Winner Counts ===")
        for c in conventions:
            print(f"{c:16s} wins={winners[c]:3d}/{n}")

        self._print_diagnosis(per_conv, winners, n, conventions)

    def _print_depth_stats(self, n_frames: int):
        all_vals_m = []
        for idx in range(n_frames):
            d = self.depth_images[idx]
            if d is None:
                continue
            vals = d[d > 0].astype(np.float32) / self.depth_scale
            if vals.size > 0:
                all_vals_m.append(vals)

        if not all_vals_m:
            print("No valid depth values found.")
            return

        vals = np.concatenate(all_vals_m, axis=0)
        q = np.percentile(vals, [1, 5, 25, 50, 75, 95, 99])
        print(
            "depth[m] percentiles "
            f"p1={q[0]:.3f} p5={q[1]:.3f} p25={q[2]:.3f} p50={q[3]:.3f} "
            f"p75={q[4]:.3f} p95={q[5]:.3f} p99={q[6]:.3f}"
        )
        print(
            f"fraction <0.30m={(vals < 0.30).mean():.3f} "
            f"<0.40m={(vals < 0.40).mean():.3f} "
            f"<0.60m={(vals < 0.60).mean():.3f} "
            f"<1.00m={(vals < 1.00).mean():.3f}"
        )

    def _print_diagnosis(
        self,
        per_conv: Dict[str, List[Dict[str, float]]],
        winners: Dict[str, int],
        n_frames: int,
        conventions: List[str],
    ):
        top = max(winners.items(), key=lambda kv: kv[1])
        top_conv, top_wins = top
        dominance = top_wins / max(n_frames, 1)

        default_scores = np.array([m["edge_score"] for m in per_conv.get(DEFAULT_CONVENTION, [])], dtype=float)
        top_scores = np.array([m["edge_score"] for m in per_conv[top_conv]], dtype=float)

        default_mean = float(default_scores.mean()) if default_scores.size else 0.0
        top_mean = float(top_scores.mean()) if top_scores.size else 0.0

        valid_means = {
            c: float(np.mean([m["valid_frac"] for m in per_conv[c]])) for c in conventions
        }
        valid_all_mean = float(np.mean(list(valid_means.values()))) if valid_means else 0.0

        print("\n=== Diagnosis Hint ===")
        if valid_all_mean < 0.45:
            print(
                "Low valid depth-to-color overlap across all conventions. "
                "Likely range/FOV/depth clipping or wrong intrinsics/resolution pairing."
            )

        if dominance >= 0.70 and top_conv != DEFAULT_CONVENTION:
            print(
                f"Convention mismatch likely: '{top_conv}' dominates ({dominance:.0%} wins) "
                f"and outperforms default '{DEFAULT_CONVENTION}' "
                f"(mean {top_mean:.2f} vs {default_mean:.2f})."
            )
            print("This points to transform-direction/configuration issue, not necessarily hand-eye quality.")
        elif dominance < 0.55:
            print(
                f"No convention clearly dominates (best wins={dominance:.0%}). "
                "This can indicate hand-eye/extrinsics quality issues, pose sync errors, "
                "or scene-dependent instability."
            )
        else:
            print(
                f"Default convention is competitive (top='{top_conv}', wins={dominance:.0%}). "
                "If alignment is still visually off, suspect hand-eye accuracy, distortion handling, "
                "or motion/pose noise rather than pure convention mismatch."
            )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Color to depth alignment + diagnostics")
    parser.add_argument("--session-path", default=DEFAULT_SESSION_PATH)
    parser.add_argument("--scan-folder", default=DEFAULT_SCAN_FOLDER)
    parser.add_argument(
        "--output-dir",
        default=DEFAULT_OUTPUT_DIR,
        help="Output directory. Default: <session>/alignment_test",
    )
    parser.add_argument("--mode", choices=["process", "diagnose"], default="process")
    parser.add_argument(
        "--convention",
        choices=["inv_Tc_Ti", "Tc_inv_Ti", "inv_inv_Tc_Ti", "inv_Tc_inv_Ti"],
        default=DEFAULT_CONVENTION,
        help="Transform convention for process mode",
    )
    parser.add_argument(
        "--diagnose-conventions",
        default="inv_Tc_Ti,Tc_inv_Ti,inv_inv_Tc_Ti,inv_Tc_inv_Ti",
        help="Comma-separated conventions for diagnose mode",
    )
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--depth-max", type=float, default=1.0)
    parser.add_argument("--depth-scale", type=float, default=1000.0)
    parser.add_argument("--save-overlay", action="store_true")
    parser.add_argument("--save-ply", action="store_true")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument(
        "--save-examples",
        type=int,
        default=3,
        help="In diagnose mode, save aligned images for first N frames per convention",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    output_dir = Path(args.output_dir) if args.output_dir else Path(args.session_path) / "alignment_test"
    session = Session(args.session_path, args.scan_folder)

    aligner = ColorToDepthAligner(
        session,
        output_dir=output_dir,
        depth_max=args.depth_max,
        depth_scale=args.depth_scale,
    )

    print(f"Depth frames: {len(aligner.depth_images)}")
    print(f"Color frames: {len(aligner.color_images)}")
    print(f"Robot poses:  {len(aligner.T_base_tool0_list)}")
    print(f"Output dir:   {output_dir}")

    if args.mode == "process":
        aligner.process_all(
            convention=args.convention,
            max_frames=args.max_frames,
            save_overlay=bool(args.save_overlay),
            save_ply=bool(args.save_ply),
            visualize=bool(args.visualize),
        )
        return

    conventions = [c.strip() for c in str(args.diagnose_conventions).split(",") if c.strip()]
    aligner.diagnose(
        max_frames=args.max_frames,
        conventions=conventions,
        save_examples=max(0, int(args.save_examples)),
        save_overlay=bool(args.save_overlay),
    )


if __name__ == "__main__":
    main()
