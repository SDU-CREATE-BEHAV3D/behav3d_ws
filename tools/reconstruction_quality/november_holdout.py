#!/usr/bin/env python3
"""Withhold every fourth November 13 view and score it after depth fusion."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import open3d as o3d
from experiment import write_json
from november_results import rays_for_scan,evaluate_mesh


PROFILES = {
    'legacy': (False, 1/512, 8/512),
    'rectified': (True, 1/512, 8/512),
    'rectified_trunc8': (True, 1/512, .008),
    'rectified_trunc4': (True, 1/512, .004),
    'rectified_fine8': (True, .001, .008),
}


class TruncationGrid:
    def __init__(self, grid, multiplier):
        self.grid, self.multiplier = grid, multiplier

    def __getattr__(self, name):
        return getattr(self.grid, name)

    def compute_unique_block_coordinates(self, *args):
        return self.grid.compute_unique_block_coordinates(*args, trunc_voxel_multiplier=self.multiplier)

    def integrate(self, *args):
        return self.grid.integrate(*args, trunc_voxel_multiplier=self.multiplier)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path)
    p.add_argument('--profiles',nargs='+',choices=PROFILES,default=['legacy','rectified'])
    p.add_argument('--sessions',nargs='*');p.add_argument('--experiment',default='heldout')
    p.add_argument('--full',action='store_true',help='Train and evaluate on all views; not an independent holdout')
    a=p.parse_args()
    sys.path.insert(0,str(a.root/'experiments/source'))
    from reconstruct import TSDF_cpu_cropped as tsdf
    cv2.setNumThreads(1);results={}
    for manifest in sorted((a.root/'data/13112026/Scans').glob('*/scan_yaml_targets/manifest.yaml')):
        source=manifest.parents[1];name=source.name
        if a.sessions and name not in a.sessions:continue
        count=len(__import__('yaml').safe_load(manifest.read_text())['captures']);heldout=list(range(3,count,4));trained=[i for i in range(count) if i not in heldout]
        if a.full:heldout=list(range(count));trained=list(range(count))
        _,rays,depths,positions,counts,_=rays_for_scan(source,frames=heldout)
        results[name]=dict(heldout=heldout,trained=trained,independent_holdout=not a.full,raw_object_frame_counts=counts,variants={},profiles={key:PROFILES[key] for key in a.profiles})
        for variant in a.profiles:
            rectify,voxel_size,truncation=PROFILES[variant]
            active=source if not rectify else a.root/f'experiments/rectified/{name}/session'
            tsdf.C2D_DIR=active/'scan_yaml_targets/reconstruct'/('color_in_depth' if not rectify else 'color_in_depth_rectified')
            tsdf.C2D_CENTER_CROP_ENABLE=False;tsdf.C2D_CENTER_CROP_APPLY_TO_DEPTH=False
            integration=tsdf.TSDF_Integration(tsdf.Session(str(active),'scan_yaml_targets'),depth_bias_m=-.0051,device='CPU:0',rectify_images=rectify,voxel_size=voxel_size)
            if truncation/voxel_size != 8:
                integration.vbg=TruncationGrid(integration.vbg,truncation/voxel_size)
            integration.images=[integration.images[i] for i in trained]
            integration.T_base_ir=[integration.T_base_ir[i] for i in trained]
            integration.integrate_depths()
            output=a.root/f'{a.experiment}/{variant}/{name}';output.mkdir(parents=True,exist_ok=False)
            for weight in (3,2,1):
                mesh=integration.vbg.extract_triangle_mesh(weight_threshold=float(weight)).to_legacy()
                mesh=tsdf._clean_mesh_for_export(mesh)
                o3d.io.write_triangle_mesh(str(output/f'weight{weight}.ply'),mesh)
                result=evaluate_mesh(mesh,rays,depths,positions)
                results[name]['variants'][f'{variant}_w{weight}']=result
                write_json(a.root/(a.experiment+'_summary.json'),results)
                print(name,variant,weight,json.dumps(result['saved_distortion']['all']),flush=True)
            del integration
