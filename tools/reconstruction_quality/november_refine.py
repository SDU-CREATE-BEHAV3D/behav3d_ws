#!/usr/bin/env python3
"""Compare conservative depth/pose refinements using unchanged held-out rays.

Experimental only: frozen production source, no changes to raw data or ROS
defaults. Pose fitting and cross-view support use training frames exclusively.
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
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from experiment import digest, write_json
from november_results import rays_for_scan, evaluate_mesh


def edge_mask(image, threshold=20):
    kernel = np.ones((3, 3), np.uint8)
    hi = cv2.dilate(image, kernel).astype(float)
    lo = cv2.erode(image, kernel).astype(float)
    return (image > 0) & (hi - lo <= threshold)


def world_points(image, pose, k, step=1):
    v, u = np.mgrid[0:image.shape[0]:step, 0:image.shape[1]:step]
    z = image[v, u].ravel().astype(float) / 1000
    camera = np.c_[(u.ravel()-k[0,2])*z/k[0,0], (v.ravel()-k[1,2])*z/k[1,1], z]
    return camera @ pose[:3,:3].T + pose[:3,3], z, v.ravel(), u.ravel()


def supported_images(images, poses, k, tolerance=.005, minimum_other_views=1):
    result, kept = [], []
    for i,(im,pose) in enumerate(zip(images, poses)):
        xyz,z,v,u = world_points(im, pose, k)
        valid = (z > 0) & (z < .9)
        ids = np.flatnonzero(valid)
        points = xyz[ids]
        support = np.zeros(len(ids), np.uint16)
        for j,(other,other_pose) in enumerate(zip(images, poses)):
            if j == i: continue
            camera = (points-other_pose[:3,3]) @ other_pose[:3,:3]
            depth = camera[:,2]
            with np.errstate(divide='ignore', invalid='ignore'):
                uv = camera[:,:2]/np.maximum(depth[:,None], 1e-9)
                uv = np.rint(uv*[k[0,0],k[1,1]]+[k[0,2],k[1,2]]).astype(int)
            visible = ((depth>0)&(uv[:,0]>=0)&(uv[:,0]<im.shape[1])
                       &(uv[:,1]>=0)&(uv[:,1]<im.shape[0]))
            sel = np.flatnonzero(visible)
            observed = other[uv[sel,1],uv[sel,0]].astype(float)/1000
            support[sel] += (observed>0)&(np.abs(observed-depth[sel])<=tolerance)
        out = np.zeros_like(im)
        chosen = ids[support >= minimum_other_views]
        out[v[chosen],u[chosen]] = im[v[chosen],u[chosen]]
        result.append(out)
        kept.append(dict(before=int(valid.sum()), after=int(len(chosen))))
    return result, kept


def refine_poses(images, poses, k, plane, center):
    """Regularized point-to-plane registration to other training frames.

    Object and table observations are balanced; corrections are bounded to
    3 mm / 1 degree per axis. A fixed anchor preserves the robot reference.
    No correction is accepted solely on an underconstrained tabletop match.
    """
    clouds, normal_sets, object_counts = [], [], []
    for im,pose in zip(images,poses):
        clean = np.where(edge_mask(im,20), im, 0).astype(np.uint16)
        xyz,z,_,_ = world_points(clean,pose,k,step=2)
        h = xyz@plane[:3]+plane[3]
        near = (np.linalg.norm(xyz[:,:2]-center[:2],axis=1)<.18)
        sel = (z>0)&(z<.9)&near&(h>-.015)&(h<.13)
        cloud=o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz[sel]))
        cloud=cloud.voxel_down_sample(.003)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=.012,max_nn=30))
        p,n=np.asarray(cloud.points),np.asarray(cloud.normals)
        h=p@plane[:3]+plane[3]
        obj=np.flatnonzero(h>.012); table=np.flatnonzero(np.abs(h)<.012)
        object_counts.append(len(obj))
        # Deterministic sampling limits domination by the table.
        ids=np.r_[obj[::max(1,len(obj)//1800)],table[::max(1,len(table)//400)]]
        clouds.append(p[ids]);normal_sets.append(n[ids])
    anchor=int(np.argmax(object_counts))
    deltas=[np.eye(4) for _ in poses]
    rows=[]
    for i,(points,normals) in enumerate(zip(clouds,normal_sets)):
        if i==anchor or object_counts[i]<80:
            rows.append(dict(frame=i,anchor=i==anchor,accepted=False,reason='anchor or insufficient object points'))
            continue
        target=np.concatenate([p for j,p in enumerate(clouds) if j!=i])
        target_normals=np.concatenate([n for j,n in enumerate(normal_sets) if j!=i])
        tree=cKDTree(target)
        x=np.zeros(6)
        for _ in range(5):
            rot=Rotation.from_rotvec(np.deg2rad(x[3:])).as_matrix()
            moved=(points-center)@rot.T+center+x[:3]/1000
            distance,indices=tree.query(moved,workers=1)
            match=(distance<.012)&(np.abs(np.sum((normals@rot.T)*target_normals[indices],axis=1))>.6)
            if match.sum()<100: break
            source=points[match];dest=target[indices[match]];ns=target_normals[indices[match]]
            def residual(q):
                r=Rotation.from_rotvec(np.deg2rad(q[3:])).as_matrix()
                p=(source-center)@r.T+center+q[:3]/1000
                values=np.sum((p-dest)*ns,axis=1)*1000
                # Robust data term with a robot-pose prior independent of sample count.
                values=np.clip(values,-3.,3.)/np.sqrt(len(values))
                return np.r_[values, .20*q/np.array([2.,2.,2.,.3,.3,.3])]
            fit=least_squares(residual,x,bounds=([-3,-3,-3,-1,-1,-1],[3,3,3,1,1,1]),max_nfev=30)
            if np.linalg.norm(fit.x-x)<1e-4:
                x=fit.x;break
            x=fit.x
        rot=Rotation.from_rotvec(np.deg2rad(x[3:])).as_matrix()
        delta=np.eye(4);delta[:3,:3]=rot;delta[:3,3]=center-rot@center+x[:3]/1000
        deltas[i]=delta
        rows.append(dict(frame=i,accepted=True,translation_at_object_mm=x[:3].tolist(),rotation_vector_deg=x[3:].tolist(),object_points=object_counts[i]))
    return [d@p for d,p in zip(deltas,poses)],rows


def main(args):
    args.output.mkdir(parents=True,exist_ok=False)
    shutil.copy2(__file__, args.output/'runner.py')
    sys.path.insert(0,str(args.pairs/'source'))
    from reconstruct import TSDF_cpu_cropped as tsdf
    cv2.setNumThreads(1)
    prior=json.loads((args.audit/'object_summary.json').read_text())
    result=dict(profiles=args.profiles,heldout_offset=args.offset,full=args.full,cases={})
    write_json(args.output/'provenance.json',dict(command=sys.argv,runner_sha256=digest(Path(__file__)),source_runtime=json.loads((args.pairs/'runtime.json').read_text())))
    for name in args.sessions:
        active=args.pairs/f'runs/{name}/corrected/session'
        source=Path(prior[name]['source'])
        count=json.loads((args.pairs/f'runs/{name}/corrected/run.json').read_text())['captures']
        heldout=list(range(args.offset,count,4));trained=[i for i in range(count) if i not in heldout]
        if args.full: heldout=list(range(count));trained=list(range(count))
        plane,rays,depths,positions,counts,_=rays_for_scan(source,frames=heldout)
        record=dict(trained=trained,heldout=heldout,variants={});result['cases'][name]=record
        for profile in args.profiles:
            start=time.perf_counter();o3d.utility.random.seed(42)
            tsdf.C2D_DIR=active/'scan_yaml_targets/reconstruct/color_in_depth_rectified'
            tsdf.C2D_CENTER_CROP_ENABLE=False;tsdf.C2D_CENTER_CROP_APPLY_TO_DEPTH=False
            bias_mm = {'bias0':0, 'bias_minus2':-2, 'bias_minus6':-6, 'bias_minus7':-7, 'bias_minus8':-8}.get(profile,-5)
            voxel_size=.001 if 'fine8' in profile else 1/512
            integration=tsdf.TSDF_Integration(tsdf.Session(str(active),'scan_yaml_targets'),depth_bias_m=bias_mm/1000,device='CPU:0',rectify_images=True,voxel_size=voxel_size)
            images=[integration.images[i] for i in trained];poses=[integration.T_base_ir[i] for i in trained]
            info={'depth_bias_mm':bias_mm}
            info['voxel_size_mm']=voxel_size*1000
            info['truncation_mm']=8*voxel_size*1000
            if 'pose' in profile:
                poses,info['pose_refinement']=refine_poses(images,poses,integration.K,plane,np.array(prior[name]['camera_center']))
            if 'edge' in profile:
                threshold=10 if 'edge10' in profile else 20
                images=[np.where(edge_mask(im,threshold),im,0).astype(np.uint16) for im in images]
                info['edge_jump_threshold_mm']=threshold
            if 'support' in profile:
                tolerance=.003 if profile=='support3' else .005
                minimum=2 if profile.startswith('support5x3') else 1
                images,info['support_counts']=supported_images(images,poses,integration.K,tolerance,minimum)
                info['support_tolerance_mm']=tolerance*1000
                info['minimum_other_views']=minimum
            integration.images=images;integration.T_base_ir=poses
            integration.integrate_depths()
            mesh=integration.vbg.extract_triangle_mesh(weight_threshold=3.).to_legacy()
            mesh=tsdf._clean_mesh_for_export(mesh)
            path=args.output/name/profile/'mesh.ply';path.parent.mkdir(parents=True)
            assert o3d.io.write_triangle_mesh(str(path),mesh)
            metrics=evaluate_mesh(mesh,rays,depths,positions)
            record['variants'][profile]=dict(metrics=metrics,processing=info,seconds=time.perf_counter()-start,mesh_sha256=digest(path),triangles=len(mesh.triangles))
            write_json(args.output/'summary.json',result)
            print(name,profile,json.dumps(metrics['saved_distortion']['all']),flush=True)
            del integration,mesh


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--audit',type=Path,required=True);p.add_argument('--pairs',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--sessions',nargs='+',required=True)
    p.add_argument('--profiles',nargs='+',default=['baseline','edge10','edge20','support5','pose','pose_edge20'])
    p.add_argument('--offset',type=int,default=3);p.add_argument('--full',action='store_true')
    main(p.parse_args())
