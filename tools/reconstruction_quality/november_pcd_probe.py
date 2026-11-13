#!/usr/bin/env python3
"""Infer which observation rules could explain the separately saved RGB cloud.

This is a diagnostic parameter sweep, not evidence of the original runtime config.
Matching retained vertices is measured against the saved mesh itself.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from evaluate import camera_poses
from experiment import intrinsics, write_json
from november_results import object_mask


def probe(source, output):
    scan = source / 'scan_yaml_targets'
    mesh = o3d.io.read_triangle_mesh(str(scan / 'reconstruct/tsdf_surface_mesh.ply'))
    points = np.asarray(mesh.vertices)
    pcd = o3d.io.read_point_cloud(str(scan / 'reconstruct/tsdf_surface_rgb_colored.ply'))
    nearest, _ = cKDTree(np.asarray(pcd.points)).query(points)
    saved = nearest < 1e-4
    plane = np.array(json.loads((scan / 'reconstruct/table_plane.json').read_text())['plane_model'])
    obj = object_mask(points, plane)
    captures, poses = camera_poses(source, 'scan_yaml_targets')
    k, _, (w, h) = intrinsics(source / 'config/depth_intrinsics.yaml')
    delta = np.full((len(captures), len(points)), np.inf, np.float32)
    crop = np.zeros((len(captures), len(points)), np.uint8)
    color_valid = np.zeros_like(crop, bool)
    for i, (capture, pose) in enumerate(zip(captures, poses)):
        camera = (points - pose[:3, 3]) @ pose[:3, :3]
        z = camera[:, 2]
        uv = np.rint(camera[:, :2]/z[:, None]*[k[0, 0], k[1, 1]]+[k[0, 2], k[1, 2]])
        valid = (z > 0) & (uv[:, 0] >= 0) & (uv[:, 0] < w) & (uv[:, 1] >= 0) & (uv[:, 1] < h)
        ids = np.flatnonzero(valid); u, v = uv[ids].astype(int).T
        raw = cv2.imread(str(scan / capture['depth']), cv2.IMREAD_UNCHANGED)
        raw = np.where(raw > 0, np.maximum(raw.astype(np.int32)-5, 1), 0).astype(np.uint16)
        values = raw.astype(np.float32)[v, u]/1000
        ok = (values > 0) & (values < .9)
        delta[i, ids[ok]] = np.abs(values[ok]-z[ids[ok]])
        color = cv2.imread(str(scan / f'reconstruct/color_in_depth/color_in_depth_{i:04d}.png'))
        if color is None:
            files = sorted((scan/'reconstruct/color_in_depth').glob('*.png'), key=lambda p:int(p.stem.split('_')[-1]))
            color = cv2.imread(str(files[i]))
        color_valid[i, ids] = np.any(color[v,u] > 0, axis=1)
        for bit, (cw, ch) in enumerate([(270, 290), (300, 290), (350, 350)]):
            in_crop = (u >= (w-cw)//2) & (u < (w+cw)//2) & (v >= (h-ch)//2) & (v < (h+ch)//2)
            crop[i, ids[in_crop]] |= 1 << bit
    rows = []
    for band in (.002, .005, .01, .02, .03):
        for mode in ('full', '270x290', '300x290', '350x350'):
            ok = (delta < band) & color_valid
            if mode != 'full':
                bit = ['270x290', '300x290', '350x350'].index(mode)
                ok &= (crop & (1 << bit)) != 0
            for step in (1, 2, 3):
                obs = ok[::step].sum(0)
                for minimum in (2, 3, 4, 5, 6):
                    pred = obs >= minimum
                    intersection = int((pred & saved).sum())
                    rows.append(dict(z_band_mm=band*1000, crop=mode, sample_step=step, min_obs=minimum,
                                     frame_limit=len(captures),
                                     predicted_points=int(pred.sum()), saved_recall=intersection/int(saved.sum()),
                                     jaccard=intersection/int((pred | saved).sum()),
                                     object_predicted_points=int((pred & obj).sum())))
    # The orchestrator historically accepted one fresh aligned image as completion.
    # Test truncated color availability separately from confidence thresholds.
    for limit in range(3, len(captures)):
        obs = ((delta[:limit] < .02) & color_valid[:limit]).sum(0)
        pred = obs >= 3
        intersection = int((pred & saved).sum())
        rows.append(dict(z_band_mm=20., crop='full', sample_step=1, min_obs=3, frame_limit=limit,
                         predicted_points=int(pred.sum()), saved_recall=intersection/int(saved.sum()),
                         jaccard=intersection/int((pred | saved).sum()), object_predicted_points=int((pred & obj).sum())))
    rows.sort(key=lambda x:x['jaccard'], reverse=True)
    # Test actual production SOR after the strongest rule, for a numerical match.
    best = rows[0]
    ok = (delta < best['z_band_mm']/1000) & color_valid
    if best['crop'] != 'full':
        bit = ['270x290', '300x290', '350x350'].index(best['crop']); ok &= (crop & (1 << bit)) != 0
    obs = ok[:best['frame_limit']:best['sample_step']].sum(0)
    pred = obs >= best['min_obs']
    cloud = o3d.geometry.PointCloud(); cloud.points = o3d.utility.Vector3dVector(points[pred])
    after, _ = cloud.remove_statistical_outlier(nb_neighbors=35, std_ratio=1.5)
    dist, _ = cKDTree(np.asarray(after.points)).query(np.asarray(pcd.points))
    rev, _ = cKDTree(np.asarray(pcd.points)).query(np.asarray(after.points))
    result = dict(source=str(source), saved_mesh_vertices=len(points), saved_pcd_points=len(pcd.points),
                  saved_pcd_matched_vertices=int(saved.sum()), object_mesh_vertices=int(obj.sum()),
                  object_saved_points=int((saved & obj).sum()), best_candidates=rows[:12],
                  best_after_sor=dict(points=len(after.points), saved_max_distance_mm=float(dist.max()*1000),
                                      predicted_max_distance_mm=float(rev.max()*1000)),
                  caveat='Parameter inference only; original runtime settings are not in the saved manifest.')
    write_json(output, result)
    print(source.name, json.dumps({k:v for k,v in result.items() if k not in ('source','best_candidates')}), 'best',best,flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('root',type=Path);p.add_argument('--sessions',nargs='*');a=p.parse_args()
    output=a.root/'pcd_probes';output.mkdir(exist_ok=True)
    for manifest in sorted((a.root/'data').glob('*/*/*/scan_yaml_targets/manifest.yaml')):
        source=manifest.parents[1]
        if a.sessions and source.name not in a.sessions:continue
        probe(source,output/(source.name+'.json'))
