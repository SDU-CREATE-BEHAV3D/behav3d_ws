#!/usr/bin/env python3
"""Measure mesh agreement with saved depth using fixed rays and explicit denominators.

These are consistency measures, not ground-truth accuracy. Both pinhole and
saved-lens-model interpretations are reported to expose camera-model dependence.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import yaml

from experiment import intrinsics, mesh_metrics, write_json


def transform(position, quaternion):
    result = np.eye(4)
    result[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    result[:3, 3] = position
    return result


def camera_poses(source, scan):
    data = yaml.safe_load((source / "config/extrinsics.yaml").read_text())["frames"]["T_tool0_ir"]
    handeye = transform(data["xyz"], data["quat_xyzw"])
    captures = yaml.safe_load((source / scan / "manifest.yaml").read_text())["captures"]
    poses = [transform(c["T_base_tool0"]["position"], c["T_base_tool0"]["orientation_xyzw"]) @ handeye for c in captures]
    return captures, poses


def error_stats(errors, total):
    valid = np.isfinite(errors)
    e = errors[valid] * 1000
    result = {"valid_raw_rays": total, "mesh_hits": int(valid.sum()), "hit_fraction": float(valid.sum() / total) if total else None}
    if len(e):
        result.update(median_signed_mm=float(np.median(e)), median_abs_mm=float(np.median(np.abs(e))),
                      p90_abs_mm=float(np.percentile(np.abs(e), 90)),
                      within_5mm_fraction=float((np.abs(e) <= 5).sum() / total),
                      within_10mm_fraction=float((np.abs(e) <= 10).sum() / total),
                      gross_30mm_fraction=float((np.abs(e) > 30).sum() / total))
    return result


def depth_agreement(mesh, source, scan, crop_width, crop_height, frames=None, stride=4):
    """Ray directions have camera Z=1; ray t is axial depth, not Euclidean range."""
    captures, poses = camera_poses(source, scan)
    plane = np.asarray(json.loads((source / scan / "reconstruct/table_plane.json").read_text())["plane_model"])
    k, d, (w, h) = intrinsics(source / "config/depth_intrinsics.yaml")
    v, u = np.mgrid[0:h:stride, 0:w:stride]
    uv = np.stack([u.ravel(), v.ravel()], axis=-1).astype(np.float64)
    ideal = np.column_stack([(uv[:, 0] - k[0, 2]) / k[0, 0], (uv[:, 1] - k[1, 2]) / k[1, 1]])
    undistorted = cv2.undistortPointsIter(uv[:, None, :], k, d, None, None,
                                        (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10)).reshape(-1, 2)
    rect_uv = undistorted * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]]
    # Restrict to rays available to BOTH preprocessing choices; keep denominator fixed.
    roi = np.ones(len(uv), bool)
    if crop_width:
        x0, y0 = (w - crop_width) // 2, (h - crop_height) // 2
        for points in (uv, rect_uv):
            roi &= (points[:, 0] >= x0 + 2) & (points[:, 0] < x0 + crop_width - 2) & (points[:, 1] >= y0 + 2) & (points[:, 1] < y0 + crop_height - 2)
    scene = o3d.t.geometry.RaycastingScene(nthreads=4)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    results = {}
    frames = list(range(len(captures))) if frames is None else frames
    for model, xy in (("pinhole", ideal), ("saved_distortion", undistorted)):
        errors_all, errors_interior, errors_object, errors_table, rows = [], [], [], [], []
        rays_camera = np.column_stack([xy, np.ones(len(xy))])
        for index in frames:
            raw = cv2.imread(str(source / scan / captures[index]["depth"]), cv2.IMREAD_UNCHANGED)
            values = raw[v.ravel(), u.ravel()].astype(float)
            depth = (values - 5.0) / 1000.0
            valid = roi & (values > 5) & (depth < 0.9)
            # Separate depth-edge pixels so a lower interior residual cannot hide edge loss.
            high = cv2.dilate(raw, np.ones((5, 5), np.uint8)).astype(float)
            low = cv2.erode(raw, np.ones((5, 5), np.uint8)).astype(float)
            interior = (high - low)[v.ravel(), u.ravel()] <= 20
            pose = poses[index]
            direction = rays_camera[valid] @ pose[:3, :3].T
            rays = np.column_stack([np.broadcast_to(pose[:3, 3], direction.shape), direction]).astype(np.float32)
            hits = scene.cast_rays(o3d.core.Tensor(rays), nthreads=4)["t_hit"].numpy()
            error = hits - depth[valid]
            # Shared proxy regions from raw observations, never candidate mesh geometry.
            physical_rays = np.column_stack([undistorted[valid], np.ones(valid.sum())])
            raw_world = (physical_rays * depth[valid, None]) @ pose[:3, :3].T + pose[:3, 3]
            height = raw_world @ plane[:3] + plane[3]
            errors_object.append(error[interior[valid] & (height > .015) & (height < .5)])
            errors_table.append(error[interior[valid] & (np.abs(height) < .01)])
            errors_all.append(error)
            errors_interior.append(error[interior[valid]])
            rows.append(dict(frame=index, **error_stats(error, len(error))))
        all_errors, smooth_errors = np.concatenate(errors_all), np.concatenate(errors_interior)
        object_errors, table_errors = np.concatenate(errors_object), np.concatenate(errors_table)
        results[model] = dict(all=error_stats(all_errors, len(all_errors)),
                              interior=error_stats(smooth_errors, len(smooth_errors)),
                              object_interior=error_stats(object_errors, len(object_errors)),
                              table_interior=error_stats(table_errors, len(table_errors)), per_frame=rows)
    return results


def table_fit(mesh, reference_plane):
    """Robust plane fit near the saved table plane; no claim it is a known flat target."""
    points = np.asarray(mesh.vertices)
    plane = np.asarray(reference_plane, float)
    plane /= np.linalg.norm(plane[:3])
    points = points[np.abs(points @ plane[:3] + plane[3]) < 0.010]
    if len(points) < 50:
        return {"points": len(points)}
    # Freeze the initial 10 mm selection; iteratively downweight residuals rather than delete them.
    a = np.column_stack([points[:, 0], points[:, 1], np.ones(len(points))])
    weights = np.ones(len(points))
    for _ in range(6):
        coeff = np.linalg.lstsq(a * weights[:, None], points[:, 2] * weights, rcond=None)[0]
        residual = points[:, 2] - a @ coeff
        weights = np.sqrt(np.minimum(1, 0.002 / np.maximum(np.abs(residual), 1e-9)))
    normal = np.array([-coeff[0], -coeff[1], 1.0])
    norm = np.linalg.norm(normal)
    residual /= norm
    return dict(points=len(points), median_abs_mm=float(np.median(np.abs(residual)) * 1000),
                p90_abs_mm=float(np.percentile(np.abs(residual), 90) * 1000),
                plane_model=(np.r_[normal, -coeff[2]] / norm).tolist(),
                note="Near-table consistency; real surface warp and low objects can contribute")


def evaluate(run, frames=None):
    run = Path(run)
    record = json.loads((run / "run.json").read_text())
    meshfile = run / "session/scan/reconstruct/tsdf_surface_mesh.ply"
    mesh = o3d.io.read_triangle_mesh(str(meshfile))
    source, scan = Path(record["source_session"]), record["source_scan"]
    saved_plane = json.loads((source / scan / "reconstruct/table_plane.json").read_text())["plane_model"]
    result = dict(case=record["case"], variant=record["variant"], run=str(run),
                  evaluated_frames=frames, trained_frames=record["selected_manifest_positions"],
                  mesh=record["mesh_metrics"], table=table_fit(mesh, saved_plane),
                  evaluation_roi_pixels=[270, 290],
                  depth=depth_agreement(mesh, source, scan, 270, 290, frames=frames),
                  depth_full_frame=depth_agreement(mesh, source, scan, 0, 0, frames=frames, stride=8),
                  runtime={k: record[k] for k in ("stage_seconds", "color_alignment_seconds", "tsdf_seconds")})
    write_json(run / ("evaluation.json" if frames is None else "heldout_evaluation.json"), result)
    return result


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("runs", type=Path, nargs="+")
    p.add_argument("--frames", help="Evaluate only these original manifest positions")
    args = p.parse_args()
    for run in args.runs:
        result = evaluate(run, None if args.frames is None else [int(i) for i in args.frames.split(",")])
        print(json.dumps(dict(case=result["case"], variant=result["variant"], depth=result["depth"]["saved_distortion"]["interior"], table=result["table"])), flush=True)
