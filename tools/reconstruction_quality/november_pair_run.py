#!/usr/bin/env python3
"""Re-run every November 13 scan with distortion correction off/on, weight 3.

Original input bytes are checked against the earlier read-only Drive snapshot.
Each reconstruction gets a fresh output folder and complete synchronous alignment.
"""
from __future__ import annotations
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import yaml

from experiment import digest, mesh_metrics, source_snapshot, write_json


def verify_inputs(audit_root, output):
    inventory = json.loads((audit_root / 'inventory.json').read_text())
    rows = [r for r in inventory['files'] if r['selected']
            and r['path'].startswith('13112026/Scans/')
            and '/reconstruct/' not in r['path']]
    if len(rows) != 381:
        raise ValueError(f'Unexpected November 13 input inventory: {len(rows)}')

    def check(row):
        for base in (Path(inventory['source']), audit_root / 'data'):
            path = base / row['path']
            if digest(path) != row['sha256']:
                raise ValueError(f'Input changed since original audit: {path}')
        return row

    with ThreadPoolExecutor(max_workers=8) as pool:
        for n, future in enumerate(as_completed(pool.submit(check, r) for r in rows), 1):
            future.result()
            if n % 100 == 0 or n == len(rows):
                print(f'Checked original and snapshot inputs: {n}/{len(rows)}', flush=True)
    write_json(output / 'input_provenance.json', dict(
        drive_folder='https://drive.google.com/drive/folders/16YXI5viCtLaFw5RlyM0jDAW7WdZNkFMb',
        original_root=inventory['source'], snapshot_root=str(audit_root / 'data'),
        original_and_snapshot_match=True, files=rows))
    return rows


def run_pair(source, output, rectify, color_to_depth, tsdf):
    output.mkdir(parents=True, exist_ok=False)
    session = output / 'session'
    shutil.copytree(source / 'config', session / 'config')
    scan = session / 'scan_yaml_targets'
    scan.mkdir()
    shutil.copy2(source / 'scan_yaml_targets/manifest.yaml', scan / 'manifest.yaml')
    for kind in ('depth_raw', 'color_raw'):
        (scan / kind).symlink_to(source / 'scan_yaml_targets' / kind, target_is_directory=True)
    captures = yaml.safe_load((scan / 'manifest.yaml').read_text())['captures']
    cv2.setNumThreads(1)
    o3d.utility.random.seed(42)
    start = time.perf_counter()
    aligned = color_to_depth.run(session_path=str(session), scan_folder='scan_yaml_targets',
                                visualize=False, rectify_images=rectify)
    paths = sorted(aligned.glob('color_in_depth_*.png'))
    expected = [f'color_in_depth_{i:04d}.png' for i in range(len(captures))]
    if [p.name for p in paths] != expected:
        raise ValueError('Incomplete/misindexed aligned image set')
    for p in paths:
        image = cv2.imread(str(p))
        if image is None or image.shape[:2] != (576, 640):
            raise ValueError(f'Incomplete aligned image: {p}')
    alignment_seconds = time.perf_counter() - start
    start = time.perf_counter()
    tsdf.run(session_path=str(session), scan_folder_override='scan_yaml_targets',
             visualize=False, device='CPU:0', rectify_images=rectify,
             center_crop_enable=False, center_crop_apply_to_depth=False,
             aabb_crop_enable=False, auto_object_crop_enable=False)
    products = scan / 'reconstruct'
    record = dict(source_session=str(source), rectify_images=rectify, captures=len(captures),
                  aligned_frames_verified=len(paths), alignment_seconds=alignment_seconds,
                  reconstruction_seconds=time.perf_counter() - start,
                  settings=dict(extraction_weight=3, voxel_size_m=1/512, truncation_m=8/512,
                                depth_max_m=.9, effective_depth_bias_mm=-5, full_frame=True,
                                min_color_observations=3, mesh_min_component_triangles=50),
                  mesh=mesh_metrics(products / 'tsdf_surface_mesh.ply'),
                  output_sha256={p.name: digest(p) for p in products.iterdir() if p.is_file()})
    write_json(output / 'run.json', record)
    print(source.name, 'corrected' if rectify else 'uncorrected', record['mesh']['triangles'], 'triangles', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--audit-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    rows = verify_inputs(args.audit_root, args.output)
    hashes = source_snapshot(repo, args.output / 'source', 'HEAD', working_tree=True)
    write_json(args.output / 'runtime.json', dict(
        revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        source_sha256=hashes, python=sys.version, platform=platform.platform(),
        packages={p: importlib.metadata.version(p) for p in ('numpy', 'scipy', 'open3d', 'opencv-python', 'PyYAML', 'matplotlib')},
        command=sys.argv, runner_sha256=digest(Path(__file__))))
    shutil.copy2(__file__, args.output / 'pair_runner.py')
    sys.path.insert(0, str(args.output / 'source'))
    from reconstruct import color_to_depth, TSDF_cpu_cropped as tsdf
    sessions = sorted((args.audit_root / 'data/13112026/Scans').glob('*/scan_yaml_targets/manifest.yaml'))
    assert len(sessions) == 9
    for manifest in sessions:
        for rectify in (False, True):
            variant = 'corrected' if rectify else 'uncorrected'
            run_pair(manifest.parents[1], args.output / 'runs' / manifest.parents[1].name / variant,
                     rectify, color_to_depth, tsdf)
    # Recheck the immutable local snapshot after all processing.
    assert all(digest(args.audit_root / 'data' / row['path']) == row['sha256'] for row in rows)
    assert all(digest(repo / name) == sha for name, sha in hashes.items())
    write_json(args.output / 'complete.json', dict(scans=9, reconstructions=18,
               input_files=len(rows), snapshot_inputs_unchanged=True, production_source_unchanged=True))
