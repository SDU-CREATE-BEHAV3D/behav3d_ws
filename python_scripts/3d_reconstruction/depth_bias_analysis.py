#!/usr/bin/env python3
"""
Depth bias analysis against a measured ground-truth surface (mesh from sparse points).

What this script does
1) Builds a piecewise-linear 2.5D mesh z=f(x,y) from measured TCP points (ground truth).
2) Reads depth captures from a Behav3D session (h1/h2/h3 style folders + manifest.yaml).
3) Back-projects each depth pixel to 3D in camera IR frame using depth intrinsics.
4) Transforms points to world using:
   - T_base_tool0 from each capture manifest entry
   - T_tool0_ir from config/extrinsics.yaml
   - optional base->world rigid transform (default yaw=180 deg, zero translation)
5) Compares z_depth_world vs z_gt_mesh(x,y) and reports bias statistics per height folder.
6) Fits a linear model: error_mm = a + b * range_m.

Definitions
- error = z_depth_world - z_gt_mesh
  Negative error means depth is below ground truth (underestimation).

Usage (example with your latest data)
python3 python_scripts/3d_reconstruction/depth_bias_analysis.py \
  --session-path captures/260217_132316

Optional arguments
- --folders h1,h2,h3              Folders under <session>/depth_bias to analyze.
- --points-mm \"x,y,z;...\"         Ground-truth points in mm (world). Defaults to your 9 points.
- --base-to-world-yaw-deg 180      Rotation used to map base coordinates to world.
- --base-to-world-xyz-m 0,0,0      Translation used to map base coordinates to world.
- --roi-margin-mm 3                XY margin from GT boundary to avoid edge artifacts.
- --min-depth-m / --max-depth-m    Depth validity bounds.
- --output-dir <dir>               Where CSV/JSON report is written.

Outputs
- <output-dir>/depth_bias_report.csv
- <output-dir>/depth_bias_report.json
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import cv2
import numpy as np
import yaml
from scipy.interpolate import LinearNDInterpolator
from scipy.spatial.transform import Rotation as R


DEFAULT_GT_POINTS_MM: List[List[float]] = [
    [-642.0, 1068.0, -6.5],
    [-442.0, 1068.0, -16.0],
    [-242.0, 1068.0, -25.0],
    [-642.0, 868.0, -7.9],
    [-442.0, 868.0, -17.0],
    [-242.0, 868.0, -26.1],
    [-642.0, 668.0, -8.6],
    [-442.0, 668.0, -18.0],
    [-242.0, 668.0, -28.1],
]


def parse_points_mm(text: str) -> np.ndarray:
    """
    Parse format: "x,y,z;x,y,z;..."
    Returns Nx3 float array in mm.
    """
    rows = []
    for chunk in str(text).strip().split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        vals = [float(v.strip()) for v in chunk.split(",")]
        if len(vals) != 3:
            raise ValueError(f"Invalid point '{chunk}'. Expected x,y,z.")
        rows.append(vals)
    if not rows:
        raise ValueError("No valid points parsed from --points-mm.")
    return np.asarray(rows, dtype=np.float64)


def parse_xyz_m(text: str) -> np.ndarray:
    vals = [float(v.strip()) for v in str(text).split(",")]
    if len(vals) != 3:
        raise ValueError(f"Invalid xyz '{text}', expected 3 comma-separated floats.")
    return np.asarray(vals, dtype=np.float64)


def load_intrinsics_k(path: Path) -> np.ndarray:
    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open intrinsics file: {path}")
    try:
        k = fs.getNode("camera_matrix").mat()
    finally:
        fs.release()
    if k is None:
        raise RuntimeError(f"camera_matrix missing in {path}")
    return np.asarray(k, dtype=np.float64).reshape(3, 3)


def load_t_tool0_ir(path: Path) -> np.ndarray:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    frames = data.get("frames", {}) if isinstance(data, dict) else {}
    ir = frames.get("T_tool0_ir", None)
    if ir is None:
        raise RuntimeError(f"T_tool0_ir not found in {path}")
    q = np.asarray(ir["quat_xyzw"], dtype=np.float64)
    t = np.asarray(ir["xyz"], dtype=np.float64)
    tmat = np.eye(4, dtype=np.float64)
    tmat[:3, :3] = R.from_quat(q).as_matrix()
    tmat[:3, 3] = t
    return tmat


def pose_to_tmat(pose: Dict[str, object]) -> np.ndarray:
    q = np.asarray(pose["orientation_xyzw"], dtype=np.float64)
    t = np.asarray(pose["position"], dtype=np.float64)
    tmat = np.eye(4, dtype=np.float64)
    tmat[:3, :3] = R.from_quat(q).as_matrix()
    tmat[:3, 3] = t
    return tmat


def robust_stats(values_m: np.ndarray, q_low: float, q_high: float) -> Dict[str, float]:
    if values_m.size == 0:
        return {
            "n": 0,
            "n_trim": 0,
            "mean_mm": float("nan"),
            "median_mm": float("nan"),
            "std_mm": float("nan"),
            "p_low_mm": float("nan"),
            "p_high_mm": float("nan"),
        }

    p_low, p_high = np.percentile(values_m, [q_low, q_high])
    keep = (values_m >= p_low) & (values_m <= p_high)
    trimmed = values_m[keep]
    if trimmed.size == 0:
        trimmed = values_m

    return {
        "n": int(values_m.size),
        "n_trim": int(trimmed.size),
        "mean_mm": float(np.mean(trimmed) * 1000.0),
        "median_mm": float(np.median(trimmed) * 1000.0),
        "std_mm": float(np.std(trimmed) * 1000.0),
        "p_low_mm": float(p_low * 1000.0),
        "p_high_mm": float(p_high * 1000.0),
    }


def iter_folders(session_path: Path, subdir: str, folders_csv: str) -> Iterable[Path]:
    names = [x.strip() for x in str(folders_csv).split(",") if x.strip()]
    for name in names:
        folder = session_path / subdir / name
        if folder.is_dir():
            yield folder


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze depth bias vs measured GT mesh.")
    parser.add_argument(
        "--session-path",
        required=True,
        help="Session root, e.g. captures/260217_132316",
    )
    parser.add_argument("--subdir", default="depth_bias", help="Subfolder under session path.")
    parser.add_argument("--folders", default="h1,h2,h3", help="Comma-separated folder list.")
    parser.add_argument(
        "--points-mm",
        default="",
        help='Ground-truth points in mm: "x,y,z;x,y,z;...". If empty, built-in defaults are used.',
    )
    parser.add_argument("--base-to-world-yaw-deg", type=float, default=180.0)
    parser.add_argument(
        "--base-to-world-xyz-m",
        default="0,0,0",
        help="Translation from base to world in meters (x,y,z).",
    )
    parser.add_argument("--roi-margin-mm", type=float, default=3.0)
    parser.add_argument("--min-depth-m", type=float, default=0.20)
    parser.add_argument("--max-depth-m", type=float, default=2.20)
    parser.add_argument("--trim-low-q", type=float, default=5.0)
    parser.add_argument("--trim-high-q", type=float, default=95.0)
    parser.add_argument("--output-dir", default="", help="Output directory for CSV/JSON report.")
    args = parser.parse_args()

    session = Path(args.session_path).expanduser().resolve()
    if not session.is_dir():
        raise FileNotFoundError(f"Session path not found: {session}")

    if args.points_mm.strip():
        gt_pts_mm = parse_points_mm(args.points_mm)
    else:
        gt_pts_mm = np.asarray(DEFAULT_GT_POINTS_MM, dtype=np.float64)
    if gt_pts_mm.shape[0] < 3:
        raise ValueError("Need at least 3 GT points.")

    # GT mesh surface z=f(x,y), using measured points in world coordinates.
    gt_pts_m = gt_pts_mm / 1000.0
    gt_xy = gt_pts_m[:, :2]
    gt_z = gt_pts_m[:, 2]
    z_interp = LinearNDInterpolator(gt_xy, gt_z, fill_value=np.nan)

    x_min, x_max = float(np.min(gt_xy[:, 0])), float(np.max(gt_xy[:, 0]))
    y_min, y_max = float(np.min(gt_xy[:, 1])), float(np.max(gt_xy[:, 1]))
    roi_margin_m = float(args.roi_margin_mm) / 1000.0

    # Camera model / extrinsics
    k = load_intrinsics_k(session / "config" / "depth_intrinsics.yaml")
    fx, fy, cx, cy = float(k[0, 0]), float(k[1, 1]), float(k[0, 2]), float(k[1, 2])
    t_tool0_ir = load_t_tool0_ir(session / "config" / "extrinsics.yaml")

    # Optional base->world transform (needed when manifests are in ur20_base_link).
    t_bw = np.asarray(parse_xyz_m(args.base_to_world_xyz_m), dtype=np.float64)
    r_bw = R.from_euler("z", np.deg2rad(float(args.base_to_world_yaw_deg))).as_matrix()

    folders = list(iter_folders(session, args.subdir, args.folders))
    if not folders:
        raise FileNotFoundError(
            f"No valid folders found under {session / args.subdir} for names '{args.folders}'."
        )

    results: List[Dict[str, float]] = []
    u_grid = None
    v_grid = None

    for folder in folders:
        manifest_path = folder / "manifest.yaml"
        if not manifest_path.is_file():
            continue
        manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
        captures = manifest.get("captures", [])

        folder_errors = []
        folder_ranges = []
        frame_count = 0

        for cap in captures:
            depth_rel = cap.get("depth", None)
            pose = cap.get("T_base_tool0", None)
            if depth_rel is None or pose is None:
                continue
            depth_path = folder / str(depth_rel)
            if not depth_path.is_file():
                continue

            depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
            if depth is None:
                continue
            if depth.dtype == np.uint16:
                z = depth.astype(np.float32) / 1000.0
            elif depth.dtype == np.float32:
                z = depth.astype(np.float32)
            else:
                z = depth.astype(np.float32)

            h, w = z.shape
            if u_grid is None or u_grid.shape != z.shape:
                uu = np.arange(w, dtype=np.float32)
                vv = np.arange(h, dtype=np.float32)
                u_grid, v_grid = np.meshgrid(uu, vv)

            valid = (z > float(args.min_depth_m)) & (z < float(args.max_depth_m))
            if not np.any(valid):
                continue

            x = (u_grid - cx) * z / fx
            y = (v_grid - cy) * z / fy
            pts_ir = np.stack((x[valid], y[valid], z[valid]), axis=1)

            t_base_tool0 = pose_to_tmat(pose)
            t_base_ir = t_base_tool0 @ t_tool0_ir
            r_bi = t_base_ir[:3, :3]
            t_bi = t_base_ir[:3, 3]

            pts_base = (r_bi @ pts_ir.T).T + t_bi[None, :]
            pts_world = (r_bw @ pts_base.T).T + t_bw[None, :]

            # Camera center range to GT center (in world), as proxy for distance.
            cam_world = (r_bw @ t_bi) + t_bw
            gt_center = np.mean(gt_pts_m, axis=0)
            folder_ranges.append(float(np.linalg.norm(cam_world - gt_center)))

            roi = (
                (pts_world[:, 0] >= x_min + roi_margin_m)
                & (pts_world[:, 0] <= x_max - roi_margin_m)
                & (pts_world[:, 1] >= y_min + roi_margin_m)
                & (pts_world[:, 1] <= y_max - roi_margin_m)
            )
            if not np.any(roi):
                continue

            xy_roi = pts_world[roi, :2]
            z_obs = pts_world[roi, 2]
            z_ref = z_interp(xy_roi[:, 0], xy_roi[:, 1])
            good = np.isfinite(z_ref)
            if not np.any(good):
                continue

            err = z_obs[good] - z_ref[good]
            folder_errors.append(err)
            frame_count += 1

        if folder_errors:
            errors = np.concatenate(folder_errors, axis=0)
        else:
            errors = np.asarray([], dtype=np.float64)

        stats = robust_stats(errors, q_low=float(args.trim_low_q), q_high=float(args.trim_high_q))
        stats["folder"] = folder.name
        stats["frames_used"] = int(frame_count)
        stats["range_m_mean"] = float(np.mean(folder_ranges)) if folder_ranges else float("nan")
        stats["range_m_std"] = float(np.std(folder_ranges)) if folder_ranges else float("nan")
        results.append(stats)

    # Linear fit of median bias vs range
    fit = None
    valid_rows = [
        r for r in results
        if int(r.get("n", 0)) > 0
        and np.isfinite(float(r.get("range_m_mean", np.nan)))
        and np.isfinite(float(r.get("median_mm", np.nan)))
    ]
    if len(valid_rows) >= 2:
        x = np.asarray([float(r["range_m_mean"]) for r in valid_rows], dtype=np.float64)
        y = np.asarray([float(r["median_mm"]) for r in valid_rows], dtype=np.float64)
        a_mat = np.vstack([np.ones_like(x), x]).T
        coef, *_ = np.linalg.lstsq(a_mat, y, rcond=None)
        a, b = float(coef[0]), float(coef[1])
        y_hat = a_mat @ coef
        ss_res = float(np.sum((y - y_hat) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        r2 = float(1.0 - ss_res / ss_tot) if ss_tot > 1e-12 else float("nan")
        fit = {"a_mm": a, "b_mm_per_m": b, "r2": r2}

    # Console summary
    print("Depth bias summary (error = z_depth_world - z_gt_mesh):")
    for row in results:
        if int(row["n"]) == 0:
            print(f"  {row['folder']}: no valid points")
            continue
        print(
            f"  {row['folder']}: n={int(row['n'])} trim={int(row['n_trim'])} "
            f"mean={row['mean_mm']:.2f} mm median={row['median_mm']:.2f} mm "
            f"std={row['std_mm']:.2f} mm pLow={row['p_low_mm']:.2f} pHigh={row['p_high_mm']:.2f} "
            f"range={row['range_m_mean']:.3f}±{row['range_m_std']:.3f} m"
        )
    if fit is not None:
        print(
            f"Linear fit: error_mm = {fit['a_mm']:.3f} + {fit['b_mm_per_m']:.3f} * range_m "
            f"(R2={fit['r2']:.4f})"
        )
    else:
        print("Linear fit: not enough valid folders.")

    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if str(args.output_dir).strip()
        else (session / args.subdir)
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    # CSV
    csv_path = output_dir / "depth_bias_report.csv"
    fieldnames = [
        "folder", "frames_used", "n", "n_trim",
        "mean_mm", "median_mm", "std_mm", "p_low_mm", "p_high_mm",
        "range_m_mean", "range_m_std",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in results:
            writer.writerow({k: row.get(k, "") for k in fieldnames})

    # JSON
    json_path = output_dir / "depth_bias_report.json"
    payload = {
        "session_path": str(session),
        "subdir": str(args.subdir),
        "folders": [p.name for p in folders],
        "ground_truth_points_mm": gt_pts_mm.tolist(),
        "base_to_world": {
            "yaw_deg": float(args.base_to_world_yaw_deg),
            "xyz_m": t_bw.tolist(),
        },
        "roi_margin_mm": float(args.roi_margin_mm),
        "depth_limits_m": [float(args.min_depth_m), float(args.max_depth_m)],
        "trim_quantiles": [float(args.trim_low_q), float(args.trim_high_q)],
        "results": results,
        "linear_fit_median_vs_range": fit,
    }
    json_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"Saved CSV:  {csv_path}")
    print(f"Saved JSON: {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
