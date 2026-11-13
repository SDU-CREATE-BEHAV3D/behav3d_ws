#!/usr/bin/env python3
"""Build compact review figures from completed November experiments."""
import argparse
import json
from pathlib import Path
import shutil
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from november_results import render


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);a=p.parse_args()
    data=json.loads((a.root/'object_summary.json').read_text());out=a.root/'figures';out.mkdir(exist_ok=True)
    cases=[('261113_150432','Cube · planar'),('261113_151435','Cone · planar'),('261113_152345','Sphere · planar'),('261113_150006','Cube · oriented barrel')]
    fig,axes=plt.subplots(len(cases),3,figsize=(12,11))
    for row,(case,label) in enumerate(cases):
        source=Path(data[case]['source']);center=data[case]['camera_center']
        paths=[source/'scan_yaml_targets/reconstruct/tsdf_surface_mesh.ply',a.root/f'experiments/legacy/{case}/weight1.ply',a.root/f'experiments/rectified/{case}/weight3.ply']
        for col,path in enumerate(paths):
            mesh=o3d.io.read_triangle_mesh(str(path))
            axes[row,col].imshow(render(mesh,center,width=.12,height=.09),origin='lower')
            axes[row,col].set_xticks([]);axes[row,col].set_yticks([])
            for spine in axes[row,col].spines.values():spine.set_visible(False)
            if col==0:axes[row,col].set_ylabel(label,fontsize=11,labelpad=10)
    for ax,label in zip(axes[0],['Saved mesh · threshold 3','Threshold lowered to 1','Distortion corrected · threshold 3']):ax.set_title(label,fontsize=11)
    fig.suptitle('November 13: confidence alone does not repair the object geometry',fontsize=15,y=.99)
    fig.text(.5,.01,'Same views and lighting. Full triangle surfaces; no smoothing or hole filling.\nThe cube still has damaged sides after correction.',ha='center',fontsize=10)
    fig.tight_layout(rect=[0,.04,1,.97]);fig.savefig(out/'november_mesh_comparison.png',dpi=160);plt.close(fig)

    replays=json.loads((a.root/'prefix_replay.json').read_text());names=[k for k,v in replays.items() if k.startswith('261113') and 'saved_points' in v]
    saved=np.array([replays[k]['saved_points'] for k in names]);full=np.array([replays[k]['complete_replay_points'] for k in names]);y=np.arange(len(names))
    labels=['Cube / linear','Cube / oriented','Cube / planar','Cone / linear','Cone / oriented','Cone / planar','Sphere / linear','Sphere / oriented','Sphere / planar']
    fig,ax=plt.subplots(figsize=(10,6.5));ax.barh(y-.17,saved/1000,height=.32,color='#aab9c4',label='Saved colored cloud');ax.barh(y+.17,full/1000,height=.32,color='#207c91',label='All aligned images available')
    ax.set_yticks(y,labels);ax.invert_yaxis();ax.set_xlabel('Retained points (thousands; entire scene)');ax.set_xlim(0,290);ax.legend(loc='upper center',bbox_to_anchor=(.5,-.15),ncol=2)
    for i,k in enumerate(names):
        r=replays[k];ax.text(full[i]/1000+3,i+.2,f"{r['inferred_used_frames']}/{r['available_frames']} views inferred",fontsize=8,va='center')
    ax.set_title('All aligned views retain more points\nwith the confidence threshold unchanged',pad=14)
    ax.spines[['top','right']].set_visible(False);fig.subplots_adjust(left=.2,right=.98,top=.87,bottom=.22);fig.savefig(out/'november_color_availability.png',dpi=160);plt.close(fig)

    # Export the full scene, not the figure's tighter viewing window.
    meshes=a.root/'review_meshes';meshes.mkdir(exist_ok=True)
    mesh_labels=['cube_linear','cube_oriented','cube_planar','cone_linear','cone_oriented','cone_planar','sphere_linear','sphere_oriented','sphere_planar']
    for name,label in zip(names,mesh_labels):
        source=a.root/f'experiments/rectified/{name}/weight3.ply';stem=f'{name}_{label}_rectified_weight3'
        shutil.copy2(source,meshes/(stem+'.ply'))
        mesh=o3d.io.read_triangle_mesh(str(source));mesh.compute_triangle_normals()
        if not o3d.io.write_triangle_mesh(str(meshes/(stem+'.stl')),mesh):
            raise OSError(f'Failed STL export: {stem}')
    (meshes/'settings.json').write_text(json.dumps(dict(coordinate_frame='ur20_base_link',units='meters',
        profile='saved calibration rectification, full depth frames, voxel 1/512 m, truncation 8/512 m, TSDF extraction threshold 3, component cleanup <50 triangles',
        mesh_contents='Complete scene; the review figure only changes the viewing window.',
        limitations='Cube sides remain visibly damaged. This is the best tested conservative profile, not a claim of ground-truth accuracy.',
        cases=dict(zip(names,mesh_labels))),indent=2)+'\n')
