"""Read-only source adapter for a recorded Agent007 cap-review case.

Run with a Python environment containing numpy and PyYAML. Writes only to specified output directory.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import sys
import math
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/behav3d_orchestrator'))
from behav3d_orchestrator.offline_agent.inspection import generate_patterns


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def quat(q):
    x,y,z,w = np.array(q,dtype=float)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def ply_points(path, limit=10000):
    # Exact Open3D binary vertex layout; fail closed for other layouts.
    with path.open('rb') as f:
        header=[]
        while True:
            line=f.readline().decode('ascii').strip(); header.append(line)
            if line=='end_header': break
            if not line: raise ValueError('invalid PLY header')
        expected=['property double '+k for k in ('x','y','z','nx','ny','nz')]+['property uchar '+k for k in ('red','green','blue')]
        if 'format binary_little_endian 1.0' not in header or [v for v in header if v.startswith('property')] != expected:
            raise ValueError('unsupported PLY vertex layout')
        n=int(next(v.split()[-1] for v in header if v.startswith('element vertex')))
        stride=struct.calcsize('<6d3B'); step=max(1,math.ceil(n/limit)); points=[]
        for i in range(n):
            row=struct.unpack('<6d3B',f.read(stride))
            if i%step==0 and all(math.isfinite(v) for v in row[:3]):
                points.append([round(v,5) for v in row[:3]])
        return points,n


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--agent-root',type=Path,required=True)
    ap.add_argument('--dataset',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args(); a=args.agent_root; d=args.dataset
    review_path=a/'outputs/expected_deposition_review_260728_133013_final/cycle_0006/review.json'
    caps_path=a/'outputs/candidate_caps_260728_133013_prefix14_validated/cycle_0006/candidate_caps.json'
    manifest_path=d/'field_loop/cycle_0006/scan/manifest.yaml'
    scene_path=d/'field_loop/cycle_0006/scan/reconstruct/tsdf_surface_rgb_colored.ply'
    ext_path=d/'config/extrinsics.yaml'; intr_path=d/'config/depth_intrinsics.yaml'
    review=json.loads(review_path.read_text()); caps=json.loads(caps_path.read_text())
    candidate=next(v for v in review['assessed_candidates'] if v['label']=='C5:T1')
    cap=next(v for v in caps['candidates'] if v['label']=='C5:T1')
    manifest=yaml.safe_load(manifest_path.read_text()); ext=yaml.safe_load(ext_path.read_text())
    if candidate['expected_observation_cycle']!=6 or caps['scan_cycle']!=6: raise ValueError('cycle linkage mismatch')
    if any(c['T_base_tool0']['parent']!='ur20_base_link' for c in manifest['captures']): raise ValueError('frame mismatch')
    binding=review['source_bindings']['current_scan']
    if sha(Path(binding['path'])) != binding['sha256']:
        raise ValueError('scan changed since evaluation')
    metrics_path=Path('/Users/josephnamar/Documents/GitHub/3DP-DDS/outputs/field_loop_260728_133013_cycle_history_corrected/cycles/cycle_0005/voxel_comparison_metrics.json')
    metrics=json.loads(metrics_path.read_text())
    source_yaml=Path(candidate['source_yaml'])
    entries=yaml.safe_load(source_yaml.read_text())['targets']
    plane=entries[0]['plane']
    raw=[float(v) for v in re.search(r'O\(([^)]+)\)',plane)[1].split(',')]
    signs=np.array([-1 if metrics['flip_x'] else 1,-1 if metrics['flip_y'] else 1,1])
    if not np.allclose(np.array(raw)*.001*signs,candidate['position_m'],atol=1e-9,rtol=0):
        raise ValueError('issued target transform mismatch')
    plan=generate_patterns(focus_m=candidate['position_m'],frame='ur20_base_link',units='m',
        distance_m=.4,width_m=.12,height_m=.12,nx=3,ny=3,samples=9,cap_deg=60,outward_axis=candidate['axis'])
    points,total=ply_points(scene_path)
    ir=ext['frames']['T_tool0_ir']; rc=quat(ir['quat_xyzw']); tc=np.array(ir['xyz'])
    recorded=[]
    for capture in manifest['captures']:
        tool=capture['T_base_tool0']; rt=quat(tool['orientation_xyzw'])
        r=rt@rc; p=np.array(tool['position'])+rt@tc
        recorded.append({'index':capture['index'],'position_m':p.tolist(),'optical_axes_world':r.T.tolist()})
    text=intr_path.read_text(); matrix=re.search(r'camera_matrix:.*?data:\s*\[([^]]+)\]',text,re.S)
    k=[float(v) for v in matrix.group(1).split(',')]
    calibration={'width':int(re.search(r'image_width:\s*(\d+)',text)[1]),
                 'height':int(re.search(r'image_height:\s*(\d+)',text)[1]),
                 'fx':k[0],'fy':k[4],'cx':k[2],'cy':k[5],
                 'frustum_model':'ideal pinhole; distortion not simulated',
                 'recorded_pose_basis':'T_base_tool0 composed with T_tool0_ir; depth/IR frame assumption matches replay; not independently calibrated'}
    source_yaml=Path(candidate['source_yaml'])
    if sha(source_yaml)!=candidate['source_yaml_sha256']: raise ValueError('issued YAML changed since evaluation')
    plan.update(label='C5:T1',candidate_cycle=5,scan_cycle=6,
                evidence_reasons=cap['cap']['reasons'],focus_basis='issued target; no diagnostic-fit assimilation',
                coordinate_mapping={'yaml_mm_to_base_m_signs':signs.tolist(), 'basis':'saved DDS flip_x/flip_y, checked against replay target', 'scene_basis':'recorded reconstruction base coordinates; no fitted scene transform'},
                settings={'distance_m':.4,'grid_width_m':.12,'grid_height_m':.12,'cap_deg':60,
                          'settings_status':'demonstration assumptions, not approved operating limits'},
                sources={str(p):sha(p) for p in (review_path,caps_path,manifest_path,scene_path,ext_path,intr_path,source_yaml,metrics_path,Path(binding['path']))})
    display={'plan':plan,'scene':points,'scene_vertex_count':total,
             'observed_patch':cap['observed_cap_points_m'],'recorded':recorded,'calibration':calibration}
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'poses.json').write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n')
    (args.output/'scene.json').write_text(json.dumps(display,allow_nan=False)+'\n')
    template=(Path(__file__).parent/'viewer.html').read_text()
    (args.output/'viewer.html').write_text(template.replace('/*DATA*/',json.dumps(display,allow_nan=False)))
    print(json.dumps({'grid_poses':len(plan['grid']),'cap_poses':len(plan['cap']),
                      'recorded_poses':len(recorded),'display_points':len(points),'output':str(args.output)}))


if __name__=='__main__': main()
