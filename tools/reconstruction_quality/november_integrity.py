#!/usr/bin/env python3
"""Verify original/snapshot hashes, frozen code, and baseline mesh correspondence."""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from experiment import digest, write_json


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    args = parser.parse_args()
    root = args.root
    repo = Path(__file__).resolve().parents[2]
    inventory = json.loads((root / 'inventory.json').read_text())
    selected = [x for x in inventory['files'] if x['selected']]

    def verify(row):
        changed = []
        for location in (Path(inventory['source']), root / 'data'):
            path = location / row['path']
            if digest(path) != row['sha256']:
                changed.append(str(path))
        return changed

    input_changes = []
    with ThreadPoolExecutor(max_workers=8) as pool:
        for count, future in enumerate(as_completed(pool.submit(verify, row) for row in selected), 1):
            input_changes.extend(future.result())
            if count % 100 == 0 or count == len(selected):
                print(f'Verified originals and copies: {count}/{len(selected)}', flush=True)
    source_hashes = json.loads((root / 'experiments/source_hashes.json').read_text())
    source_changes = [p for p, sha in source_hashes.items() if digest(repo / p) != sha]
    comparisons = {}
    for path in sorted((root / 'experiments/legacy').glob('*/audit.json')):
        record = json.loads(path.read_text())
        replay = o3d.io.read_triangle_mesh(str(path.parent / 'weight3.ply'))
        saved = o3d.io.read_triangle_mesh(str(Path(record['source']) / 'scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply'))
        rv, sv = np.asarray(replay.vertices), np.asarray(saved.vertices)
        a, _ = cKDTree(sv).query(rv)
        b, _ = cKDTree(rv).query(sv)
        comparisons[path.parent.name] = dict(
            replay_vertices=len(rv), saved_vertices=len(sv), replay_triangles=len(replay.triangles), saved_triangles=len(saved.triangles),
            replay_to_saved_mm=np.percentile(a * 1000, [50, 99, 99.9, 100]).tolist(),
            saved_to_replay_mm=np.percentile(b * 1000, [50, 99, 99.9, 100]).tolist(),
            replay_fraction_within_0p1mm=float((a < .0001).mean()), saved_fraction_within_0p1mm=float((b < .0001).mean()))
    record = dict(
        selected_input_files=len(selected), selected_input_bytes=sum(x['size'] for x in selected),
        input_changes=input_changes, source_code_changes=source_changes, source_hashes=source_hashes,
        baseline_mesh_comparisons=comparisons, python=sys.version, executable=sys.executable,
        platform=platform.platform(), packages={p: importlib.metadata.version(p) for p in ['numpy', 'scipy', 'open3d', 'opencv-python', 'PyYAML', 'matplotlib']},
        git_revision=subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=repo, text=True).strip(),
        audit_script_hashes={str(p.relative_to(repo)): digest(p) for p in (repo / 'tools/reconstruction_quality').glob('november*.py')},
        review_file_hashes={p.name: digest(p) for p in (root / 'review_meshes').iterdir() if p.is_file()})
    write_json(root / 'integrity.json', record)
    assert not input_changes and not source_changes
    print(json.dumps(dict(inputs=len(selected), input_changes=input_changes, source_changes=source_changes,
                          minimum_baseline_match_percent=min(x['replay_fraction_within_0p1mm'] for x in comparisons.values()) * 100)), flush=True)
