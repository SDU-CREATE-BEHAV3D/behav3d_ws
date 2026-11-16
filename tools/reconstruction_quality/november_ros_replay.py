#!/usr/bin/env python3
"""Replay actual ROS service runner functions without ROS transport or a robot.

The unavailable ROS imports are stubbed; image alignment, completion validation,
and production reconstruction are real. Raw capture files remain read only.
"""
import argparse
import importlib.util
import inspect
import json
from pathlib import Path
import shutil
import sys
import time
from types import SimpleNamespace

import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree

from experiment import digest, source_snapshot, write_json
from november_pair_run import verify_inputs


def main(args):
    args.output.mkdir(parents=True, exist_ok=False)
    repo = Path(__file__).resolve().parents[2]
    inputs = verify_inputs(args.audit, args.output)
    hashes = source_snapshot(repo, args.output/'source', 'HEAD', working_tree=True)
    sys.path.insert(0, str(args.output/'source'))
    sys.modules['rclpy'] = SimpleNamespace()
    sys.modules['rclpy.node'] = SimpleNamespace(Node=object)
    sys.modules['behav3d_interfaces.srv'] = SimpleNamespace(ColorToDepth=object, TsdfCropped=object, TsdfObjectExtract=object)
    from reconstruct import reconstruct_services as services
    helper = repo/'src/behav3d_orchestrator/behav3d_orchestrator/src/reconstruction_outputs.py'
    spec = importlib.util.spec_from_file_location('completion',helper)
    completion=importlib.util.module_from_spec(spec);spec.loader.exec_module(completion)
    hashes[str(helper.relative_to(repo))] = digest(helper)
    write_json(args.output/'runtime.json', dict(source_sha256=hashes, command=sys.argv,
               ros_transport_tested=False, real_service_runner_functions=True,
               crop_profile='scan_yaml_targets_sequence_config.yaml: full frame'))
    assert inspect.signature(services._run_color_to_depth).parameters['rectify_images'].default is True
    assert inspect.signature(services._run_tsdf_cropped).parameters['rectify_images'].default is True
    records={}
    cv2.setNumThreads(1)
    for manifest in sorted((args.audit/'data/13112026/Scans').glob('*/scan_yaml_targets/manifest.yaml')):
        source=manifest.parents[1]; name=source.name
        session=args.output/'runs'/name/'session'
        shutil.copytree(source/'config',session/'config')
        scan=session/'scan_yaml_targets';scan.mkdir()
        shutil.copy2(manifest,scan/'manifest.yaml')
        for kind in ('depth_raw','color_raw'):
            (scan/kind).symlink_to(source/'scan_yaml_targets'/kind,target_is_directory=True)
        start=time.time();o3d.utility.random.seed(42)
        aligned=services._run_color_to_depth(session,'scan_yaml_targets',False,'CPU:0')
        expected=services._resolve_color_to_depth_outputs(session,'scan_yaml_targets')['output_path']
        assert aligned == expected
        assert completion.alignment_is_complete(aligned,start)
        metadata=json.loads((aligned/'alignment.json').read_text())
        assert metadata['rectified'] and metadata['complete']
        services._run_tsdf_cropped(session,'scan_yaml_targets',False,'CPU:0',
                                  center_crop_enable=False,center_crop_apply_to_depth=False,
                                  aabb_crop_enable=False,auto_object_crop_enable=False)
        products=scan/'reconstruct'
        mesh=o3d.io.read_triangle_mesh(str(products/'tsdf_surface_mesh.ply'))
        reference=args.pairs/f'runs/{name}/corrected/session/scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply'
        old=o3d.io.read_triangle_mesh(str(reference))
        v,w=np.asarray(mesh.vertices),np.asarray(old.vertices)
        distances=np.r_[cKDTree(v).query(w,workers=1)[0],cKDTree(w).query(v,workers=1)[0]]
        fraction=float(np.mean(distances<.0001))
        assert fraction>.999, f'Unexpected geometry change: {name} {fraction}'
        records[name]=dict(aligned_frames=metadata['frame_count'],complete_handoff=True,
                           rectified=True,triangles=len(mesh.triangles),
                           bidirectional_vertices_within_01mm_fraction=fraction,
                           vertex_distance_max_mm=float(distances.max()*1000),
                           files_sha256={p.name:digest(p) for p in products.iterdir() if p.is_file()})
        write_json(args.output/'summary.json',records)
        print(name,json.dumps(records[name]),flush=True)
    assert all(digest(args.audit/'data'/row['path'])==row['sha256'] for row in inputs)
    assert all(digest(repo/path)==sha for path,sha in hashes.items())
    write_json(args.output/'complete.json',dict(scans=len(records),source_and_inputs_unchanged=True,
                                              complete_frame_handoff_verified=True,
                                              geometry_matches_previous_rectified_runs=True,
                                              ros_transport_tested=False))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--pairs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    main(p.parse_args())
