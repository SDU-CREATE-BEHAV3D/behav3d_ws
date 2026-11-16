#!/usr/bin/env python3
"""Summarize all scan types and render actual full-scan refinement meshes."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from experiment import digest, write_json
from november_results import render
from november_cube_bounds import CASES, measure


LABELS = {
    'baseline': 'Corrected baseline',
    'bias_minus6': 'Depth bias -6 mm',
    'bias_minus7': 'Depth bias -7 mm',
    'support5x3': 'Three-view depth support',
    'pose_edge20': 'Pose + depth-edge rejection',
    'fine8': '1 mm voxels / 8 mm band',
    'pose_edge20_fine8': '1 mm + pose / edge rejection',
}


def main(root, audit, pairs):
    ordinary=json.loads((root/'all_scan_validation/summary.json').read_text())
    fine=json.loads((root/'fine_all_validation/summary.json').read_text())
    prior=json.loads((audit/'object_summary.json').read_text())
    compact=dict(measure='Sensor consistency, not independent physical accuracy',
                 primary_split='Every fourth view starting at index 3; refinements use training views only',
                 aggregation='P90 is the arithmetic mean of nine per-scan P90 values. Within-5mm/coverage pool all valid withheld rays, including misses in denominators.',
                 aggregate={},cases={},cube_bounds={})
    for name in CASES:
        merged={**ordinary['cases'][name]['variants'],**fine['cases'][name]['variants']}
        compact['cases'][name]={p:r['metrics']['saved_distortion']['all'] for p,r in merged.items()}
    for profile,label in LABELS.items():
        rows=[c[profile] for c in compact['cases'].values()]
        n=sum(m['valid_raw_rays'] for m in rows)
        compact['aggregate'][profile]=dict(label=label,valid_raw_rays=n,
            mean_scan_p90_mm=float(np.mean([m['p90_abs_mm'] for m in rows])),
            within_5mm_percent=100*sum(m['within_5mm_fraction']*m['valid_raw_rays'] for m in rows)/n,
            mesh_hit_percent=100*sum(m['mesh_hits'] for m in rows)/n)
    figures=root/'figures';figures.mkdir(exist_ok=True)
    fig=plt.figure(figsize=(18,9.5),facecolor='white')
    outer=fig.add_gridspec(3,3,left=.018,right=.988,bottom=.025,top=.86,hspace=.35,wspace=.08)
    for i,(name,(shape,pattern)) in enumerate(CASES.items()):
        row,col=divmod(i,3);inner=outer[row,col].subgridspec(1,2,wspace=.025)
        bbox=outer[row,col].get_position(fig)
        fig.text((bbox.x0+bbox.x1)/2,bbox.y1+.035,f'{shape} · {pattern}',ha='center',va='bottom',fontsize=13,fontweight='bold')
        fig.text((bbox.x0+bbox.x1)/2,bbox.y1+.017,name,ha='center',va='bottom',fontsize=10,color='#555555')
        center=np.array(prior[name]['camera_center'])
        plane=np.array(json.loads((Path(prior[name]['source'])/'scan_yaml_targets/reconstruct/table_plane.json').read_text())['plane_model'])
        center[2]=-(center[:2]@plane[:2]+plane[3])/plane[2]+(.015 if shape=='Cone' else .025)
        paths=[pairs/f'runs/{name}/corrected/session/scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply',
               root/f'full_fine_review/{name}/pose_edge20_fine8/mesh.ply']
        pair,pair_axes=plt.subplots(1,2,figsize=(10,4.8))
        for j,(path,title) in enumerate(zip(paths,['Corrected baseline','Experimental: 1 mm + pose / edge'])):
            mesh=o3d.io.read_triangle_mesh(str(path));assert len(mesh.triangles)
            picture=render(mesh,center,width=.12,height=.09)
            ax=fig.add_subplot(inner[0,j]);ax.imshow(picture,origin='lower');ax.set_title(title,fontsize=9,pad=4);ax.axis('off')
            pair_axes[j].imshow(picture,origin='lower');pair_axes[j].set_title(title);pair_axes[j].axis('off')
            if shape=='Cube':
                bounds=measure(np.asarray(mesh.vertices),plane,center)
                bounds['mesh_sha256']=digest(path)
                compact['cube_bounds'].setdefault(name,{})['baseline' if j==0 else 'experimental']=bounds
        pair.suptitle(f'{shape} / {pattern} · {name} · distortion correction ON in both')
        pair.text(.5,.02,'All captured views fused for display. Same view and scale; no display smoothing. Experimental profile is not a new default.',ha='center',fontsize=9)
        pair.tight_layout(rect=[0,.04,1,.94]);pair.savefig(figures/f'{name}_refinement.png',dpi=180);plt.close(pair)
    fig.suptitle('Further refinement · all nine November 13 scans',fontsize=21,y=.994,fontweight='bold')
    fig.text(.5,.944,'Distortion correction ON in both · threshold 3 · full-scan meshes · experimental profile has accuracy/coverage tradeoffs',ha='center',fontsize=12)
    fig.savefig(figures/'all_nine_refinement.png',dpi=180);plt.close(fig)
    values=compact['aggregate'];names=list(values);y=np.arange(len(names))
    chart,axes=plt.subplots(1,2,figsize=(12,5.5),sharey=True)
    colors=['#24716f']+['#859cbb']*(len(names)-1)
    for ax,key,title in zip(axes,['mean_scan_p90_mm','within_5mm_percent'],['Mean per-scan P90 error (mm) · lower is better','Within 5 mm of withheld depth (%) · higher is better']):
        data=[values[n][key] for n in names]
        ax.barh(y,data,color=colors);ax.set_title(title,fontsize=11)
        for i,v in enumerate(data):ax.text(v+.25,i,f'{v:.2f}',va='center',fontsize=9)
        ax.set_xlim(0,max(data)*1.13);ax.spines[['top','right']].set_visible(False)
        ax.grid(axis='x',alpha=.2);ax.set_axisbelow(True)
    axes[0].set_yticks(y,[LABELS[n] for n in names]);axes[0].invert_yaxis()
    chart.suptitle('Further refinement: held-out sensor agreement across all nine scans',fontsize=14)
    chart.text(.5,.015,'Fixed withheld rays; missing surfaces count against the within-5-mm rate. These metrics do not establish physical accuracy.',ha='center',fontsize=9)
    chart.tight_layout(rect=[0,.04,1,.94]);chart.savefig(figures/'refinement_metrics.png',dpi=180);plt.close(chart)
    studies=['cube_discovery','cube_depth_support','all_scan_validation','alternate_cube_validation','fine_all_validation']
    compact['heldout_reconstruction_count']=sum(sum(len(c['variants']) for c in json.loads((root/s/'summary.json').read_text())['cases'].values()) for s in studies)
    compact['full_experimental_reconstruction_count']=9
    compact['ros_replay']=json.loads((root/'ros_replay/complete.json').read_text())
    write_json(root/'refinement_summary.json',compact)
    print(json.dumps(compact['aggregate'],indent=2))
    print('Held-out reconstructions:',compact['heldout_reconstruction_count'])


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--root',type=Path,required=True);p.add_argument('--audit',type=Path,required=True)
    p.add_argument('--pairs',type=Path,required=True)
    a=p.parse_args();main(a.root,a.audit,a.pairs)
