#!/usr/bin/env python3
"""Replay inferred incomplete color prefixes through the actual production filter."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from experiment import write_json


def distance_stats(a, b):
    av, bv = np.asarray(a.points), np.asarray(b.points)
    da, _ = cKDTree(bv).query(av); db, _ = cKDTree(av).query(bv)
    return dict(replayed_points=len(av), saved_points=len(bv),
                replay_to_saved_mm=np.percentile(da*1000, [50, 99, 99.9, 100]).tolist(),
                saved_to_replay_mm=np.percentile(db*1000, [50, 99, 99.9, 100]).tolist(),
                replay_fraction_within_0p1mm=float((da < .0001).mean()),
                saved_fraction_within_0p1mm=float((db < .0001).mean()))


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args()
    sys.path.insert(0,str(a.root/'experiments/source'))
    from reconstruct import TSDF_cpu_cropped as tsdf
    cv2.setNumThreads(1)
    results={}
    for probe_path in sorted((a.root/'pcd_probes').glob('*.json')):
        probe=json.loads(probe_path.read_text());source=Path(probe['source']);name=source.name
        tsdf.C2D_DIR=source/'scan_yaml_targets/reconstruct/color_in_depth'
        tsdf.C2D_CENTER_CROP_ENABLE=False;tsdf.C2D_CENTER_CROP_APPLY_TO_DEPTH=False
        tsdf.C2D_REQUIRE_VALID_MASK_FOR_OBS=True
        integration=tsdf.TSDF_Integration(tsdf.Session(str(source),'scan_yaml_targets'),depth_bias_m=-.0051,device='CPU:0')
        mesh=o3d.io.read_triangle_mesh(str(a.root/f'experiments/legacy/{name}/weight3_raw.ply'))
        cloud=o3d.geometry.PointCloud();cloud.points=mesh.vertices;cloud.normals=mesh.vertex_normals
        best=probe['best_candidates'][0]
        if not (best['crop']=='full' and best['sample_step']==1 and best['min_obs']==3 and best['z_band_mm']==20 and best['jaccard']>.97):
            results[name]=dict(status='No close prefix match; original point-cloud settings or loaded inputs remain unresolved',best_candidate=best)
            write_json(a.root/'prefix_replay.json',results)
            del integration
            continue
        full_color=integration.color_in_depth
        integration.color_in_depth=full_color[:best['frame_limit']]
        replay,conf=integration.colorize_tsdf_surface_points(cloud,min_obs=3)
        replay,_=integration.filter_outliers_with_conf(replay,conf,method='statistical',nb_neighbors=35,std_ratio=1.5)
        saved=o3d.io.read_point_cloud(str(source/'scan_yaml_targets/reconstruct/tsdf_surface_rgb_colored.ply'))
        row=dict(available_frames=len(full_color),inferred_used_frames=best['frame_limit'],**distance_stats(replay,saved))
        integration.color_in_depth=full_color
        full,conf=integration.colorize_tsdf_surface_points(cloud,min_obs=3)
        full,_=integration.filter_outliers_with_conf(full,conf,method='statistical',nb_neighbors=35,std_ratio=1.5)
        row['complete_replay_points']=len(full.points)
        o3d.io.write_point_cloud(str(a.root/f'experiments/legacy/{name}/complete_rgb.ply'),full)
        results[name]=row;write_json(a.root/'prefix_replay.json',results)
        print(name,json.dumps(row),flush=True)
        del integration
