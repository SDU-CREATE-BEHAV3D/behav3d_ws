"""Evaluate full-resolution mesh against existing observed samples, no capture."""
import argparse
import hashlib
import json
from pathlib import Path
from visibility import MeshRays, load_binary_stl, evaluate
from bead_surface import nominal_surface, surface_views


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--demo',type=Path,required=True)
    ap.add_argument('--mesh',type=Path,required=True);args=ap.parse_args()
    display=json.loads((args.demo/'scene.json').read_text());plan=display['plan']
    for path,digest in plan['sources'].items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
            raise ValueError('source changed since pose generation: '+path)
    bound_meshes=[Path(p) for p in plan['sources'] if p.endswith('tsdf_surface_mesh.ply')]
    if not bound_meshes or args.mesh.resolve()!=bound_meshes[0].with_suffix('.stl').resolve():
        raise ValueError('mesh must be the bound scan reconstruction sibling')
    triangles=load_binary_stl(args.mesh);rays=MeshRays(triangles)
    # Use every supplied observed cap sample, never infer missing bead surface.
    points=display['observed_patch']
    # Commanded reference axis is not an observed surface normal.
    candidates_path=next(Path(p) for p in plan['sources'] if p.endswith('/cycle_0006/review.json'))
    review=json.loads(candidates_path.read_text())
    candidate=next(c for c in review['assessed_candidates'] if c['label']==plan['label'])
    reference=candidate['axis']
    out={'schema':'behav3d.recorded_visibility_comparison.v1','label':plan['label'],
         'frame':plan['frame'],'units':'m','mesh_path':str(args.mesh.resolve()),
         'mesh_sha256':hashlib.sha256(args.mesh.read_bytes()).hexdigest(),'triangle_count':len(triangles),
         'sample_basis':'all existing raw observed cap samples; not full intended bead coverage',
         'unknown_space_policy':'clear ray without endpoint support remains unknown; recorded mesh support does not certify free space',
         'surface_normal_basis':'commanded candidate axis only; angles are diagnostic, no incidence acceptance threshold',
         'physical_checks':'depth quality, robot reachability, tool clearance and transitions unverified',
         'execution_authorized':False,'patterns':{},'selection':'hold; visibility diagnostics alone do not resolve identity/registration or certify a scan plan'}
    for pattern in ('grid','cap'):
        result=evaluate(plan[pattern],points,display['calibration'],rays,reference_axis=reference)
        # Reclassify stored nearest intersections to check tolerance sensitivity without new rays.
        result['tolerance_sensitivity']=[]
        import math
        for tolerance in (.002,.004,.008):
            counts=[0]*len(points)
            for pose,row in zip(plan[pattern],result['poses']):
                for i,(status,hit) in enumerate(zip(row['statuses'],row['first_hit_m'])):
                    if status=='outside_image' or hit is None:continue
                    length=math.dist(pose['position_m'],points[i])
                    if abs(hit-length)<=tolerance:counts[i]+=1
            result['tolerance_sensitivity'].append({'tolerance_mm':tolerance*1000,'supported_once':sum(c>0 for c in counts),'supported_three_views':sum(c>=3 for c in counts)})
        out['patterns'][pattern]=result
        print(pattern, result['endpoint_supported_in_at_least_one_view'],result['endpoint_supported_in_at_least_three_views'],flush=True)
    surface=nominal_surface(plan['focus_m'],reference,candidate['width_mm']*.001,candidate['height_mm']*.001)
    out['nominal_surface']=surface
    out['nominal_patterns']={}
    for pattern in ('grid','cap'):
        result=surface_views(plan[pattern],surface,display['calibration'],rays,evaluate)
        out['nominal_patterns'][pattern]=result
        print(pattern, 'nominal top/side',result['regions'],flush=True)
    display['comparison']=out
    display.pop('local_scene', None)
    display.pop('grid_supplement', None)
    (args.demo/'comparison.json').write_text(json.dumps(out,indent=2,allow_nan=False)+'\n')
    (args.demo/'scene.json').write_text(json.dumps(display,allow_nan=False)+'\n')
    template=(Path(__file__).parent/'viewer.html').read_text()
    (args.demo/'viewer.html').write_text(template.replace('/*DATA*/',json.dumps(display,allow_nan=False)))


if __name__=='__main__':main()
