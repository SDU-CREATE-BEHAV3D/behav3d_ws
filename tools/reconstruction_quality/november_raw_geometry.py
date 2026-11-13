#!/usr/bin/env python3
"""Fit each raw view's nearby table independently to measure inter-view mismatch."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np
from evaluate import camera_poses
from experiment import intrinsics,write_json


def fit(points):
    a=np.c_[points[:,:2],np.ones(len(points))];w=np.ones(len(points))
    for _ in range(10):
        coefficients=np.linalg.lstsq(a*w[:,None],points[:,2]*w,rcond=None)[0]
        residual=points[:,2]-a@coefficients
        w=np.sqrt(np.minimum(1.,.0015/np.maximum(np.abs(residual),1e-9)))
    normal=np.r_[-coefficients[:2],1.];normal/=np.linalg.norm(normal)
    return coefficients,normal,float(np.percentile(np.abs(residual),90)*1000)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('root',type=Path);args=p.parse_args()
    summaries=json.loads((args.root/'object_summary.json').read_text());results={}
    for name,summary in summaries.items():
        source=Path(summary['source']);captures,poses=camera_poses(source,'scan_yaml_targets')
        center=np.array(summary['camera_center']);plane=np.array(json.loads((source/'scan_yaml_targets/reconstruct/table_plane.json').read_text())['plane_model'])
        k,d,(w,h)=intrinsics(source/'config/depth_intrinsics.yaml');v,u=np.mgrid[0:h:4,0:w:4];uv=np.c_[u.ravel(),v.ravel()].astype(float)
        pinhole=(uv-[k[0,2],k[1,2]])/[k[0,0],k[1,1]]
        undistorted=cv2.undistortPointsIter(uv[:,None],k,d,None,None,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,50,1e-10)).reshape(-1,2)
        models={'pinhole':pinhole,'saved_distortion':undistorted};rows={key:[] for key in models}
        for i,(capture,pose) in enumerate(zip(captures,poses)):
            raw=cv2.imread(str(source/'scan_yaml_targets'/capture['depth']),cv2.IMREAD_UNCHANGED);values=raw[v.ravel(),u.ravel()];depth=(values.astype(float)-5)/1000
            world=(np.c_[undistorted,np.ones(len(uv))]*depth[:,None])@pose[:3,:3].T+pose[:3,3]
            height=world@plane[:3]+plane[3]
            selected=((values>5)&(depth<.9)&(abs(world[:,0])<.2)&(world[:,1]>-1.15)&(world[:,1]<-.65)&(abs(height)<.04)&(np.linalg.norm(world[:,:2]-center[:2],axis=1)>.10))
            if selected.sum()<100:continue
            for model,xy in models.items():
                xyz=(np.c_[xy[selected],np.ones(selected.sum())]*depth[selected,None])@pose[:3,:3].T+pose[:3,3]
                coef,normal,p90=fit(xyz);z=float(np.r_[center[:2],1.]@coef)
                rows[model].append(dict(frame=i,points=int(selected.sum()),center_table_height_m=z,normal=normal.tolist(),within_frame_plane_p90_mm=p90))
        results[name]={}
        for model,rs in rows.items():
            z=np.array([r['center_table_height_m'] for r in rs]);normals=np.array([r['normal'] for r in rs]);cos=np.clip(normals@normals.T,-1,1)
            results[name][model]=dict(frames=rs,center_height_range_mm=float(np.ptp(z)*1000),center_height_p90_p10_mm=float(np.diff(np.percentile(z,[10,90]))[0]*1000),max_normal_difference_deg=float(np.rad2deg(np.arccos(cos)).max()))
        print(name,{key:round(value['center_height_range_mm'],2) for key,value in results[name].items()},flush=True)
    write_json(args.root/'raw_table_consistency.json',results)
