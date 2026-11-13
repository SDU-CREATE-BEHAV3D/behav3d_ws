#!/usr/bin/env python3
"""Separate TSDF extraction, color observation, and component losses on saved data.

Writes only beneath --output. Input sessions must be the separate snapshot.
One frozen integrated volume supplies each weight-threshold comparison.
"""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import shutil
import sys
import time

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import yaml

from experiment import digest, intrinsics, mesh_metrics, source_snapshot, write_json
from verify_geometry import compare


def observation_counts(integration, points):
    """Instrument the production projection rule without discarding zero counts."""
    n = len(points)
    depth_count, color_count = np.zeros(n, np.uint16), np.zeros(n, np.uint16)
    h, w = integration.images[0].shape
    k = integration.K
    for depth, color, pose in zip(integration.images, integration.color_in_depth, integration.T_base_ir):
        camera = (points - pose[:3, 3]) @ pose[:3, :3]
        z = camera[:, 2]
        with np.errstate(divide='ignore', invalid='ignore'):
            uv = np.rint(camera[:, :2] / z[:, None] * [k[0, 0], k[1, 1]] + [k[0, 2], k[1, 2]])
        valid = (z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        indices = np.flatnonzero(valid)
        u, v = uv[indices].astype(int).T
        values = depth.astype(np.float32)[v, u] / integration.depth_scale
        ok = (values > 0) & (values < integration.depth_max) & (np.abs(values - z[indices]) < .02)
        depth_count[indices[ok]] += 1
        colored = np.any(color[v, u] > 0, axis=1)
        color_count[indices[ok & colored]] += 1
    return depth_count, color_count


def cloud_equivalence(a, b):
    if not len(a.points) or not len(b.points):
        return dict(a=len(a.points), b=len(b.points), equivalent=False)
    ap, bp = np.asarray(a.points), np.asarray(b.points)
    da, ia = cKDTree(bp).query(ap)
    db, _ = cKDTree(ap).query(bp)
    maximum = float(max(da.max(), db.max()))
    return dict(a=len(ap), b=len(bp), max_point_difference_mm=maximum * 1000,
                equivalent=bool(len(ap) == len(bp) and maximum < 1e-7))


def scan_inventory(source):
    scan = source / 'scan_yaml_targets'
    captures = yaml.safe_load((scan / 'manifest.yaml').read_text())['captures']
    frames = []
    for c in captures:
        p = scan / c['depth']
        d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        if d is None or d.dtype != np.uint16 or d.shape != (576, 640):
            raise ValueError(f'Invalid depth {p}')
        frames.append(dict(index=c['index'], sha256=digest(p), zero_fraction=float((d == 0).mean()),
                           usable_fraction=float(((d > 5) & (d < 905)).mean()),
                           over_range_fraction=float((d >= 905).mean()),
                           nonzero_depth_mm=np.percentile(d[d > 0], [5, 50, 95]).tolist()))
    return dict(captures=len(captures), depth_files=len(list((scan / 'depth_raw').glob('*.png'))),
                color_files=len(list((scan / 'color_raw').glob('*.png'))),
                aligned_files=len(list((scan / 'reconstruct/color_in_depth').glob('*.png'))),
                unique_depth_frames=len(set(x['sha256'] for x in frames)),
                unique_poses=len(set(json.dumps(c['T_base_tool0'], sort_keys=True) for c in captures)),
                configs={p.name: digest(p) for p in sorted((source / 'config').glob('*.yaml'))}, frames=frames)


def analyze(source, output, tsdf, color_to_depth, variant):
    output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(1)
    o3d.utility.random.seed(42)
    record = dict(source=str(source), variant=variant, input_inventory=scan_inventory(source),
                  voxel_size_m=1/512, truncation_m=8/512, effective_bias_mm=-5,
                  depth_max_m=.9, crop=False, thresholds=[3., 2., 1.],
                  min_color_observations=3, observation_tolerance_m=.02)
    active = source
    if variant == 'rectified':
        active = output / 'session'
        shutil.copytree(source / 'config', active / 'config')
        scan = active / 'scan_yaml_targets'
        scan.mkdir()
        shutil.copy2(source / 'scan_yaml_targets/manifest.yaml', scan / 'manifest.yaml')
        for kind in ('depth_raw', 'color_raw'):
            (scan / kind).symlink_to(source / 'scan_yaml_targets' / kind, target_is_directory=True)
        color_to_depth.run(session_path=str(active), scan_folder='scan_yaml_targets', visualize=False, rectify_images=True)
    tsdf.C2D_DIR = active / 'scan_yaml_targets/reconstruct' / ('color_in_depth_rectified' if variant == 'rectified' else 'color_in_depth')
    tsdf.C2D_CENTER_CROP_ENABLE = False
    tsdf.C2D_CENTER_CROP_APPLY_TO_DEPTH = False
    tsdf.C2D_ERODE_ENABLE = False
    tsdf.C2D_REQUIRE_VALID_MASK_FOR_OBS = True
    session = tsdf.Session(str(active), 'scan_yaml_targets')
    start = time.perf_counter()
    integration = tsdf.TSDF_Integration(session, depth_bias_m=-.0051, device='CPU:0', rectify_images=variant == 'rectified')
    integration.integrate_depths()
    record['integration_seconds'] = time.perf_counter() - start
    record['weights'] = {}
    for weight in (3., 2., 1.):
        mesh = integration.vbg.extract_triangle_mesh(weight_threshold=weight).to_legacy()
        mesh.compute_vertex_normals()
        raw_path = output / f'weight{int(weight)}_raw.ply'
        o3d.io.write_triangle_mesh(str(raw_path), mesh)
        row = dict(raw=mesh_metrics(raw_path))
        clean = tsdf._clean_mesh_for_export(copy.deepcopy(mesh))
        clean_path = output / f'weight{int(weight)}.ply'
        o3d.io.write_triangle_mesh(str(clean_path), clean)
        row['clean'] = mesh_metrics(clean_path)
        row['cleanup_triangle_loss'] = len(mesh.triangles) - len(clean.triangles)
        # Every vertex survives here; later thresholds can be measured without re-integrating.
        pts = np.asarray(mesh.vertices)
        dep, col = observation_counts(integration, pts)
        np.savez_compressed(output / f'observations_weight{int(weight)}.npz', depth=dep, color=col)
        row['depth_observation_histogram'] = np.bincount(dep, minlength=len(integration.images)+1).tolist()
        row['color_observation_histogram'] = np.bincount(col, minlength=len(integration.images)+1).tolist()
        row['color_mask_extra_point_loss_at_3'] = int(((dep >= 3) & (col < 3)).sum())
        row['min_obs3_point_loss'] = int((col < 3).sum())
        if weight == 3:
            cloud = o3d.geometry.PointCloud()
            cloud.points = mesh.vertices
            cloud.normals = mesh.vertex_normals
            # Independent instrumentation must agree with actual production outputs.
            prod, conf = integration.colorize_tsdf_surface_points(cloud, min_obs=1)
            assert np.array_equal(np.asarray(prod.points), pts[col >= 1])
            assert np.array_equal(conf, col[col >= 1])
            row['observation_rule_verified_against_production'] = True
            before_sor, conf = integration.colorize_tsdf_surface_points(cloud, min_obs=3)
            after_sor, _ = integration.filter_outliers_with_conf(before_sor, conf, method='statistical', nb_neighbors=35, std_ratio=1.5)
            row['pcd_before_sor'] = len(before_sor.points)
            row['pcd_after_sor'] = len(after_sor.points)
            if variant == 'legacy':
                saved_mesh = source / 'scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply'
                row['saved_mesh_equivalence'] = compare(clean_path, saved_mesh)
                saved_pcd = o3d.io.read_point_cloud(str(source / 'scan_yaml_targets/reconstruct/tsdf_surface_rgb_colored.ply'))
                row['saved_pcd_equivalence'] = cloud_equivalence(after_sor, saved_pcd)
        record['weights'][str(int(weight))] = row
        write_json(output / 'audit.json', record)
        print(json.dumps(dict(session=source.name, variant=variant, weight=weight,
                              triangles=row['clean']['triangles'], confidence_loss=row['min_obs3_point_loss'],
                              color_mask_loss=row['color_mask_extra_point_loss_at_3'])), flush=True)
    return record


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--data', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--variant', choices=['legacy', 'rectified'], required=True)
    p.add_argument('--sessions', nargs='*')
    args = p.parse_args()
    repo = Path(__file__).resolve().parents[2]
    args.output.mkdir(parents=True, exist_ok=True)
    snapshot = args.output / 'source'
    if not snapshot.exists():
        hashes = source_snapshot(repo, snapshot, 'HEAD', working_tree=True)
        write_json(args.output / 'source_hashes.json', hashes)
    sys.path.insert(0, str(snapshot))
    from reconstruct import TSDF_cpu_cropped as tsdf, color_to_depth
    shutil.copy2(__file__, args.output / 'audit_runner.py')
    for manifest in sorted(args.data.glob('*/*/*/scan_yaml_targets/manifest.yaml')):
        source = manifest.parents[1]
        if args.sessions and source.name not in args.sessions:
            continue
        output = args.output / args.variant / source.name
        if (output / 'audit.json').exists() and len(json.loads((output / 'audit.json').read_text())['weights']) == 3:
            continue
        analyze(source, output, tsdf, color_to_depth, args.variant)
