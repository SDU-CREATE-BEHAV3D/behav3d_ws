"""Build an auditable local-scene assessment using saved sources only."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from bead_surface import nominal_surface
from local_scene import NominalBead, ObservedScene, assess_views, coverage, positive, select_views
from visibility import load_binary_stl

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src/behav3d_orchestrator'))
from behav3d_orchestrator.offline_agent.inspection import basis, pose


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def directional_pool(target, frame, settings):
    distance = positive(settings['distance_m'])
    tilts = settings['pool_tilts_deg']; n = settings['azimuth_samples']
    if not isinstance(tilts, list) or not tilts or any(isinstance(t,bool) or not isinstance(t,(float,int)) or not math.isfinite(t) or not 0<t<90 for t in tilts):
        raise ValueError('pool tilts must be explicit angles in (0,90)')
    if type(n) is not int or not 3<=n<=32 or len(tilts)*n>128:
        raise ValueError('invalid bounded direction count')
    bx, by, bz = map(np.asarray, basis(target.direction.tolist()))
    poses = []
    for tilt in tilts:
        for i in range(n):
            azimuth = i*2*math.pi/n; theta = math.radians(tilt)
            direction = math.sin(theta)*(math.cos(azimuth)*bx+math.sin(azimuth)*by)+math.cos(theta)*bz
            position = np.asarray(target.top)+distance*direction
            item = pose(position, basis((-direction).tolist()), frame, len(poses))
            item.update(tilt_deg=tilt, azimuth_deg=i*360/n)
            poses.append(item)
    return poses


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--demo',type=Path,required=True)
    parser.add_argument('--settings',type=Path,default=Path(__file__).with_name('local_scene_settings.json'))
    args=parser.parse_args()
    data=json.loads((args.demo/'scene.json').read_text()); plan=data['plan']
    settings=json.loads(args.settings.read_text())
    if settings['schema']!='behav3d.local_scene_settings.v1' or settings['units']!='m' or plan['units']!='m' or settings['frame']!=plan['frame']:
        raise ValueError('settings/plan frame or units mismatch')
    for path, expected in plan['sources'].items():
        if digest(path)!=expected: raise ValueError('source revision changed: '+path)
    comparison=data['comparison']; mesh_path=Path(comparison['mesh_path'])
    if comparison['frame']!=plan['frame'] or comparison['units']!='m' or digest(mesh_path)!=comparison['mesh_sha256']:
        raise ValueError('comparison mesh revision/frame mismatch')
    review_path=next(Path(p) for p in plan['sources'] if p.endswith('/cycle_0006/review.json'))
    review=json.loads(review_path.read_text())
    if review['scan_cycle']!=plan['scan_cycle'] or review['assessed_candidate_cycle']!=plan['candidate_cycle'] or plan['scan_cycle']!=plan['candidate_cycle']+1:
        raise ValueError('invalid cycle linkage')
    candidates=review['assessed_candidates']
    if any(c['candidate_cycle']!=plan['candidate_cycle'] or c['expected_observation_cycle']!=plan['scan_cycle'] for c in candidates):
        raise ValueError('neighbor candidates are from another cycle')
    beads=[NominalBead(c['label'],tuple(c['position_m']),tuple(c['axis']),c['width_mm']*.001,c['height_mm']*.001) for c in candidates]
    if len({b.label for b in beads})!=len(beads): raise ValueError('duplicate bead labels')
    target=next(b for b in beads if b.label==plan['label'])
    if not np.allclose(target.top,plan['focus_m'],rtol=0,atol=1e-9): raise ValueError('target/plan mismatch')
    neighbors=[b for b in beads if b.label!=target.label]
    surface=nominal_surface(target.top,target.axis,target.width_m,target.height_m)
    scene=ObservedScene(load_binary_stl(mesh_path))
    pool=directional_pool(target,plan['frame'],settings)
    options={k:settings[k] for k in ('relation_margin_m','neighbor_margin_m','endpoint_tolerance_m','max_incidence_deg')}
    groups={'grid':plan['grid'],'cap':plan['cap'],'direction_pool':pool,'recorded':data['recorded']}
    patterns={}
    for name,poses in groups.items():
        rows=assess_views(poses,surface,data['calibration'],scene,target,neighbors,settings.get('camera_envelope'),**options)
        patterns[name]={'poses':poses,'assessments':rows,'coverage':coverage(rows,surface)}
        print(name,patterns[name]['coverage']['regions'],flush=True)
    proposal=select_views(pool,patterns['direction_pool']['assessments'],surface,settings['pose_budget'])
    # Relation is geometric only. This mask is used for a separate proximity
    # diagnostic, NEVER to remove mesh triangles from visibility/clearance tests.
    outside=np.all(target.sdf(scene.triangles)>settings['relation_margin_m'],axis=1)
    proximity=[]
    for point in surface['points_m']:
        proximity.append(scene.nearest_surface(point,mask=outside))
    side=np.asarray(surface['regions'])=='side'
    gaps=[r['distance_m'] for i,r in enumerate(proximity) if side[i] and r['distance_m'] is not None]
    baseline=np.array(patterns['recorded']['coverage']['sample_view_counts'])>0
    selected=np.array(proposal['coverage']['sample_view_counts'])>0
    weights=np.asarray(surface['area_weights_m2'])
    proposal['additional_side_opportunity_vs_recorded_pose_model_fraction']=float(weights[side & selected & ~baseline].sum()/weights[side].sum())
    # Neighbor outlines help explain competing spatial associations in the viewer.
    outlines=[dict(b.record(),points_m=nominal_surface(b.top,b.axis,b.width_m,b.height_m)['points_m']) for b in neighbors]
    blockers=Counter()
    for p in patterns.values():
        for row in p['assessments']:
            blockers.update(rel['kind'] for rel in row['blocker_relations'] if rel)
    source_files={**plan['sources'],str(mesh_path):digest(mesh_path),str(args.settings.resolve()):digest(args.settings)}
    result={'schema':'behav3d.local_scene_assessment.v1','frame':plan['frame'],'units':'m',
            'label':plan['label'],'candidate_cycle':plan['candidate_cycle'],'scan_cycle':plan['scan_cycle'],
            'target':target.record(),'neighbors':outlines,'nominal_surface':surface,
            'patterns':patterns,'proposal':proposal,'settings':settings,'sources':source_files,
            'blocker_relation_counts':dict(blockers),'proximity':proximity,
            'side_min_outside_region_surface_distance_m':min(gaps) if gaps else None,
            'neighbor_scope':'six same-cycle intended beads; all other observed geometry retained as unassigned scene',
            'proximity_basis':'nearest triangles whose vertices are outside target plus relation margin; no identity or physical gap assertion',
            'recorded_baseline_basis':'modeled visibility from recorded camera poses, not raw-depth confirmation',
            'unknown_space_policy':'unobserved geometry is unknown; no hit does not prove free space; intended-neighbor envelope hits are uncertain',
            'selection_basis':'greedy new side area, side repeats up to three, then top; geometry-only demonstration objective',
            'execution_authorized':False,'robot_commands':[],
            'unverified':['material identity','registration accuracy','calibrated camera envelope','full tool/arm clearance','IK','trajectory','depth quality']}
    # Immutable provenance digest for this result, excluding no mutable fields.
    result['assessment_sha256']=hashlib.sha256(json.dumps(result,sort_keys=True,allow_nan=False).encode()).hexdigest()
    (args.demo/'local_scene.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    data['local_scene']=result
    data.pop('grid_supplement', None)  # Invalidate proposals tied to an older assessment.
    (args.demo/'scene.json').write_text(json.dumps(data,allow_nan=False)+'\n')
    template=Path(__file__).with_name('viewer.html').read_text()
    (args.demo/'viewer.html').write_text(template.replace('/*DATA*/',json.dumps(data,allow_nan=False)))
    print('proposal',proposal['selected_pool_indices'],proposal['coverage']['regions'],flush=True)


if __name__=='__main__':main()
