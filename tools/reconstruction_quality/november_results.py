#!/usr/bin/env python3
"""Object-region measurements with fixed raw-depth rays and identical mesh views."""
import argparse
import json
from pathlib import Path
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from evaluate import camera_poses, error_stats
from experiment import intrinsics, write_json


def object_mask(points, plane):
    # Fixed workspace region, chosen from saved captures; excludes remote table edges.
    height = points @ plane[:3] + plane[3]
    return ((np.abs(points[:, 0]) < .15) & (points[:, 1] > -1.15)
            & (points[:, 1] < -.65) & (height > .012) & (height < .15))


def rays_for_scan(source, frames=None):
    scan = 'scan_yaml_targets'
    captures, poses = camera_poses(source, scan)
    plane = np.array(json.loads((source / scan / 'reconstruct/table_plane.json').read_text())['plane_model'])
    k, d, (w, h) = intrinsics(source / 'config/depth_intrinsics.yaml')
    v, u = np.mgrid[0:h:2, 0:w:2]
    uv = np.c_[u.ravel(), v.ravel()].astype(float)
    pinhole = (uv - [k[0, 2], k[1, 2]]) / [k[0, 0], k[1, 1]]
    undistorted = cv2.undistortPointsIter(uv[:, None], k, d, None, None,
                (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10)).reshape(-1, 2)
    xy = {'saved_distortion': undistorted, 'pinhole': pinhole}
    rays = {key: [] for key in xy}
    depths, positions, counts, raw_points = [], [], [], []
    for i, (capture, pose) in enumerate(zip(captures, poses)):
        if frames is not None and i not in frames:
            continue
        raw = cv2.imread(str(source / scan / capture['depth']), cv2.IMREAD_UNCHANGED)
        values = raw[v.ravel(), u.ravel()]
        depth = (values.astype(float) - 5) / 1000
        world = (np.c_[undistorted, np.ones(len(uv))] * depth[:, None]) @ pose[:3, :3].T + pose[:3, 3]
        valid = (values > 5) & (depth < .9) & object_mask(world, plane)
        # Remove the immediate discontinuity neighborhood; same mask for every candidate.
        hi = cv2.dilate(raw, np.ones((3, 3), np.uint8)).astype(float)
        lo = cv2.erode(raw, np.ones((3, 3), np.uint8)).astype(float)
        smooth = (hi - lo)[v.ravel(), u.ravel()] <= 20
        sel = valid & smooth
        counts.append(dict(frame=i, object_valid_rays=int(valid.sum()), interior_rays=int(sel.sum())))
        raw_points.append(world[valid])
        depths.append(depth[sel])
        positions.append(np.full(sel.sum(), i))
        for model, xy_model in xy.items():
            directions = np.c_[xy_model[sel], np.ones(sel.sum())] @ pose[:3, :3].T
            rays[model].append(np.c_[np.broadcast_to(pose[:3, 3], directions.shape), directions].astype(np.float32))
    return plane, {k: np.concatenate(v) for k, v in rays.items()}, np.concatenate(depths), np.concatenate(positions), counts, np.concatenate(raw_points)


def evaluate_mesh(mesh, rays, depths, positions):
    scene = o3d.t.geometry.RaycastingScene(nthreads=4)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    result = {}
    for model, r in rays.items():
        hits = scene.cast_rays(o3d.core.Tensor(r), nthreads=4)['t_hit'].numpy()
        error = hits - depths
        result[model] = dict(all=error_stats(error, len(error)), per_frame={str(i): error_stats(error[positions == i], int((positions == i).sum())) for i in np.unique(positions)})
    return result


def render(mesh, center, width=.18, height=.14):
    forward = -np.array([.7, -1., .8]); forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0, 0, 1]); right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    x, y = np.meshgrid(np.linspace(-width/2, width/2, 600), np.linspace(-height/2, height/2, 450))
    origins = np.array(center) - forward + x[..., None] * right + y[..., None] * up
    rays = np.concatenate([origins, np.broadcast_to(forward, origins.shape)], axis=-1).astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene(nthreads=4)
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    hit = scene.cast_rays(o3d.core.Tensor(rays), nthreads=4)
    normal = hit['primitive_normals'].numpy()
    light = np.array([-.4, -.6, .8]); light /= np.linalg.norm(light)
    image = np.array([.58, .72, .77])[None, None] * (.25 + .75 * np.abs(normal @ light))[..., None]
    image[~np.isfinite(hit['t_hit'].numpy())] = .96
    return image


def summarize(root):
    summary = {}
    figs = root / 'figures'; figs.mkdir(exist_ok=True)
    for manifest in sorted((root / 'data').glob('*/*/*/scan_yaml_targets/manifest.yaml')):
        source = manifest.parents[1]; name = source.name
        plane, rays, depths, positions, counts, points = rays_for_scan(source)
        center = np.median(points, axis=0)
        center[2] = -(center[:2] @ plane[:2] + plane[3])/plane[2] + .035
        case = dict(source=str(source), raw_object_frame_counts=counts, camera_center=center.tolist(), variants={})
        paths = [('saved', source / 'scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply')]
        for var in ('legacy', 'rectified'):
            for w in (3, 2, 1):
                f = root / f'experiments/{var}/{name}/weight{w}.ply'
                audit = f.parent / 'audit.json'
                if audit.exists() and str(w) in json.loads(audit.read_text())['weights']:
                    paths.append((f'{var}_w{w}', f))
        fig, axes = plt.subplots(1, len(paths), figsize=(3.1 * len(paths), 3.1), squeeze=False)
        for ax, (variant, path) in zip(axes.flat, paths):
            mesh = o3d.io.read_triangle_mesh(str(path))
            verts = np.asarray(mesh.vertices); tri = np.asarray(mesh.triangles)
            chosen = object_mask(verts, plane)
            t = tri[chosen[tri].all(1)]
            area = np.linalg.norm(np.cross(verts[t[:, 1]] - verts[t[:, 0]], verts[t[:, 2]] - verts[t[:, 0]]), axis=1).sum()/2
            row = dict(object_triangles=len(t), object_area_cm2=float(area * 1e4), depth=evaluate_mesh(mesh, rays, depths, positions))
            if variant != 'saved':
                w = variant[-1]
                raw = o3d.io.read_triangle_mesh(str(path.with_name(f'weight{w}_raw.ply')))
                mask = object_mask(np.asarray(raw.vertices), plane)
                obs = np.load(path.with_name(f'observations_weight{w}.npz'))
                row['object_raw_triangles'] = int(mask[np.asarray(raw.triangles)].all(1).sum())
                row['object_raw_vertices'] = int(mask.sum())
                row['object_color_obs_below3'] = int((obs['color'][mask] < 3).sum())
                row['object_depth_obs_below3'] = int((obs['depth'][mask] < 3).sum())
                row['object_observation_histogram'] = np.bincount(obs['color'][mask], minlength=len(counts)+1).tolist()
            case['variants'][variant] = row
            ax.imshow(render(mesh, center, width=.25 if name.startswith('261104') else .14, height=.17 if name.startswith('261104') else .105), origin='lower')
            ax.set_title(f"{variant}\n{len(t):,} object triangles", fontsize=10); ax.axis('off')
        fig.suptitle(name + ' · identical cameras; raw triangle surfaces, no smoothing or hole filling')
        fig.tight_layout(); fig.savefig(figs / f'{name}_comparison.png', dpi=130); plt.close(fig)
        summary[name] = case
        write_json(root / 'object_summary.json', summary)
        print(name, {k: (v['object_triangles'], round(v['depth']['saved_distortion']['all'].get('p90_abs_mm', 0), 2), round(v['depth']['saved_distortion']['all'].get('within_5mm_fraction', 0)*100, 1)) for k,v in case['variants'].items()}, flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    summarize(args.root)
