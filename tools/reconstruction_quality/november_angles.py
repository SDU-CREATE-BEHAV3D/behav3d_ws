#!/usr/bin/env python3
"""Processing-only view-angle selection with fixed captured depth and poses.

Normals are estimated for selection; surviving depth values are never smoothed
or changed. Unknown normals and discontinuity neighbourhoods are retained.
"""
import argparse
import json
from pathlib import Path
import shutil
import sys
import time

import cv2
import numpy as np
import open3d as o3d

from experiment import digest, write_json
from november_results import rays_for_scan, evaluate_mesh


def view_cosines(depth, k):
    """Estimate incidence using local PCA on a valid 5x5 camera-space patch."""
    z = depth.astype(np.float64) / 1000
    v, u = np.mgrid[:z.shape[0], :z.shape[1]]
    points = np.stack(((u-k[0, 2])*z/k[0, 0],
                       (v-k[1, 2])*z/k[1, 1], z), axis=-1)
    valid = (depth > 0) & (depth < 900)
    # A large one-pixel jump marks unreliable neighbourhood normals. Preserve
    # those pixels instead of allowing a normal fit to erase a real silhouette.
    adjacent = valid.copy()
    for axis in (0, 1):
        for shift in (-1, 1):
            other = np.roll(depth, shift, axis=axis).astype(np.float64)
            adjacent &= (other > 0) & (np.abs(other-depth.astype(float)) <= 20)
    adjacent[[0, -1], :] = False
    adjacent[:, [0, -1]] = False
    reliable = cv2.erode(adjacent.astype(np.uint8), np.ones((3, 3), np.uint8),
                         borderType=cv2.BORDER_CONSTANT, borderValue=0).astype(bool)
    count = cv2.boxFilter(valid.astype(float), -1, (5, 5), normalize=False,
                         borderType=cv2.BORDER_CONSTANT)
    reliable &= count == 25
    mean = cv2.boxFilter(points, -1, (5, 5), normalize=True,
                        borderType=cv2.BORDER_CONSTANT)
    ids = np.flatnonzero(reliable)
    covariance = np.empty((len(ids), 3, 3), dtype=float)
    for a in range(3):
        for b in range(a, 3):
            moment = cv2.boxFilter(points[..., a]*points[..., b], -1, (5, 5),
                                  normalize=True, borderType=cv2.BORDER_CONSTANT)
            values = (moment-mean[..., a]*mean[..., b]).ravel()[ids]
            covariance[:, a, b] = covariance[:, b, a] = values
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    planar = (eigenvalues[:, 1] > 1e-10) & (
        np.maximum(eigenvalues[:, 0], 0) / np.maximum(eigenvalues[:, 1], 1e-12) < .15)
    reliable.ravel()[ids[~planar]] = False
    selected = ids[planar]
    normals = eigenvectors[planar, :, 0]
    rays = points.reshape(-1, 3)[selected]
    rays /= np.linalg.norm(rays, axis=1)[:, None]
    cosine = np.full(depth.shape, np.nan)
    cosine.ravel()[selected] = np.abs(np.sum(normals*rays, axis=1))
    return cosine, reliable


def select_angles(depth, cosine, reliable, angle):
    rejected = reliable & (cosine < np.cos(np.deg2rad(angle)))
    output = depth.copy()
    output[rejected] = 0
    assert np.array_equal(output[~rejected], depth[~rejected])
    return output, rejected


def self_check():
    """Known tilted planes independently check normal direction and units."""
    k = np.array([[500., 0., 31.5], [0., 500., 23.5], [0., 0., 1.]])
    v, u = np.mgrid[:48, :64]
    rays = np.stack(((u-31.5)/500, (v-23.5)/500, np.ones_like(u)), axis=-1)
    for angle in (0, 80):
        normal = np.array([np.sin(np.deg2rad(angle)), 0, np.cos(np.deg2rad(angle))])
        depth = np.rint(1000*.6*normal[2]/(rays@normal)).astype(np.uint16)
        before = depth.copy()
        cosine, reliable = view_cosines(depth, k)
        expected = np.abs(rays@normal)/np.linalg.norm(rays, axis=-1)
        center = np.zeros_like(reliable); center[16:32, 24:40] = True
        assert np.all(reliable[center])
        assert np.max(np.abs(cosine[center]-expected[center])) < .03
        selected, rejected = select_angles(depth, cosine, reliable, 60)
        assert np.all(rejected[center] == (angle == 80))
        assert np.array_equal(depth, before)
    step = np.full((48, 64), 550, dtype=np.uint16); step[:, 32:] = 650
    cosine, reliable = view_cosines(step, k)
    selected, _ = select_angles(step, cosine, reliable, 60)
    assert not reliable[:, 31:33].any()
    assert np.array_equal(selected[:, 31:33], step[:, 31:33])


def main(args):
    self_check()
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copy2(__file__, args.output/'runner.py')
    sys.path.insert(0, str(args.pairs/'source'))
    from reconstruct import TSDF_cpu_cropped as tsdf
    cv2.setNumThreads(1)
    prior = json.loads((args.audit/'object_summary.json').read_text())
    names = args.sessions or sorted(name for name in prior if name.startswith('261113'))
    result = dict(profiles=['baseline']+[f'angle{a}' for a in args.angles],
                  heldout_offset=args.offset, full=args.full,
                  fixed=dict(rectified=True, voxel_size_m=1/512, truncation_m=8/512,
                             extraction_weight=3, depth_bias_mm=-5,
                             poses_unchanged=True, calibration_unchanged=True),
                  normal_method='5x5 PCA, local planarity and 20 mm adjacent-jump guard; uncertain normals retained',
                  analytic_self_check_passed=True, cases={})
    write_json(args.output/'provenance.json', dict(command=sys.argv,
               runner_sha256=digest(Path(__file__)),
               source_runtime=json.loads((args.pairs/'runtime.json').read_text())))
    for name in names:
        active = args.pairs/f'runs/{name}/corrected/session'
        source = Path(prior[name]['source'])
        count = json.loads((args.pairs/f'runs/{name}/corrected/run.json').read_text())['captures']
        heldout = list(range(args.offset, count, 4))
        trained = [i for i in range(count) if i not in heldout]
        if args.full: trained = heldout = list(range(count))
        plane, rays, depths, positions, _, _ = rays_for_scan(source, frames=heldout)
        case = dict(trained=trained, heldout=heldout, variants={}); result['cases'][name] = case
        normals = None
        for angle in [None]+args.angles:
            profile = 'baseline' if angle is None else f'angle{angle}'
            start = time.perf_counter(); o3d.utility.random.seed(42)
            tsdf.C2D_DIR = active/'scan_yaml_targets/reconstruct/color_in_depth_rectified'
            tsdf.C2D_CENTER_CROP_ENABLE = False; tsdf.C2D_CENTER_CROP_APPLY_TO_DEPTH = False
            integration = tsdf.TSDF_Integration(tsdf.Session(str(active), 'scan_yaml_targets'),
                          depth_bias_m=-.005, device='CPU:0', rectify_images=True)
            images = [integration.images[i] for i in trained]
            poses = [integration.T_base_ir[i] for i in trained]
            if normals is None:
                normals = [view_cosines(im, integration.K) for im in images]
            selections = []
            if angle is not None:
                outputs = []
                for im, pose, (cosine, reliable) in zip(images, poses, normals):
                    out, rejected = select_angles(im, cosine, reliable, angle)
                    outputs.append(out)
                    v, u = np.mgrid[:im.shape[0], :im.shape[1]]
                    z = im.astype(float)/1000
                    xyz = np.stack(((u-integration.K[0,2])*z/integration.K[0,0],
                                    (v-integration.K[1,2])*z/integration.K[1,1], z), axis=-1)
                    world = xyz@pose[:3,:3].T+pose[:3,3]
                    height = world@plane[:3]+plane[3]
                    region = (np.linalg.norm(world[...,:2]-np.array(prior[name]['camera_center'])[:2],axis=-1)<.10)
                    region &= (height>.012)&(height<.15)&(im>0)&(im<900)
                    selections.append(dict(valid=int(np.count_nonzero(im)),
                                           rejected=int(rejected.sum()),
                                           object_pixels=int(region.sum()),
                                           object_rejected=int((region&rejected).sum()),
                                           reliable_object_normals=int((region&reliable).sum())))
                images = outputs
            integration.images = images; integration.T_base_ir = poses
            integration.integrate_depths()
            mesh = integration.vbg.extract_triangle_mesh(weight_threshold=3.).to_legacy()
            mesh = tsdf._clean_mesh_for_export(mesh)
            path = args.output/name/profile/'mesh.ply'; path.parent.mkdir(parents=True)
            assert o3d.io.write_triangle_mesh(str(path), mesh)
            metrics = evaluate_mesh(mesh, rays, depths, positions)
            case['variants'][profile] = dict(metrics=metrics, frame_selection=selections,
                  max_incidence_angle_deg=angle, mesh_sha256=digest(path),
                  triangles=len(mesh.triangles), seconds=time.perf_counter()-start)
            write_json(args.output/'summary.json', result)
            print(name, profile, json.dumps(metrics['saved_distortion']['all']), flush=True)
            del integration, mesh


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit', type=Path, required=True)
    p.add_argument('--pairs', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sessions', nargs='+')
    p.add_argument('--angles', type=int, nargs='+', default=[55, 70])
    p.add_argument('--offset', type=int, default=3)
    p.add_argument('--full', action='store_true')
    main(p.parse_args())
