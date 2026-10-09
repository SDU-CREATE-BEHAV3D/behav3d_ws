"""Higher inward-lean experiment on recorded scan 19, with contact access study."""
import argparse
from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import yaml

from geometry import adapt_targets,issued_targets,lean,ply_points
from build_experiments import dump,digest,snapshot_sources
from build_grid_supplement import validate_assessment
from build_frontier_scan import load_bundle
from visibility import load_binary_stl
from viability import (ROOT,COLLISION_SOURCE,TOOL_MESH,WORKCELL,Scene,tool_models,paddle_samples,
                       evaluate_candidates,seat_targets,search_supported_targets,propose_contact)

LEAN={'x_start_m':.19,'x_full_m':.21,'y_center_m':-.875,
      'y_inner_m':.065,'y_outer_m':.095,'pivot_z_m':-.075}
ANGLES=[0,4,8,12]
SETTINGS={'support_gap_mm':3.,'minimum_support_rays':6,'tool_margin_mm':2.,
          'minimum_center_spacing_mm':32.,'path_step_m':.003,
          'settings_status':'experimental screens, not calibrated print-validity thresholds',
          'spacing_basis':'36 mm nominal proxy minus 4 mm assumed overlap; recorded width map unavailable'}


def deform(points,angle): return lean(points,-angle,LEAN)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--agent-root',type=Path,required=True)
    parser.add_argument('--local',type=Path,default=ROOT/'outputs/offline_inspection/frontier_batch/c18-t7/local_scene.json')
    parser.add_argument('--output',type=Path,default=ROOT/'outputs/offline_recovery/inward_cycle19')
    args=parser.parse_args()
    local=json.loads(args.local.read_text());validate_assessment(local)
    if (local['label'],local['scan_cycle'],local['frame'],local['units'])!=('C18:T7',19,'ur20_base_link','m'):
        raise ValueError('protocol bound to recorded C18:T7/scan19')
    bundle=args.agent_root/'outputs/integrated_evaluation_260728_133013_final/cycle_0019/evaluation'
    packet,evaluation=load_bundle(bundle)
    if packet['scan_cycle']!=19 or packet['candidate_cycle']!=18: raise ValueError('bundle cycle mismatch')
    review_path=next(Path(p) for p in local['sources'] if p.endswith('/cycle_0019/review.json'))
    if evaluation['source_sha256'].get(str(review_path))!=local['sources'][str(review_path)]: raise ValueError('unbound review')
    review=json.loads(review_path.read_text())
    candidate=next(c for c in review['assessed_candidates'] if c['label']==local['label'])
    metrics_path=Path(candidate['source_metrics']);metrics=json.loads(metrics_path.read_text())
    mesh_path=next(Path(p) for p in local['sources'] if p.endswith('tsdf_surface_mesh.stl'))
    scan=mesh_path.parents[1];manifest_path=scan/'manifest.yaml'
    manifest=yaml.safe_load(manifest_path.read_text())
    if any(c['T_base_tool0']['parent']!=local['frame'] for c in manifest['captures']): raise ValueError('frame mismatch')
    next_dir=scan.parent/'candidates';target_path=next_dir/'targets.yaml';points_path=next_dir/'field_print_candidates_cycle.ply'
    baseline=issued_targets(target_path,19,local['target']['width_m'],local['target']['height_m'],
                            flip_x=metrics['flip_x'],flip_y=metrics['flip_y'])
    if not np.allclose(np.array([r['top_m'] for r in baseline]),ply_points(points_path),rtol=0,atol=.00001):
        raise ValueError('archived YAML/point-cloud mapping mismatch')
    paths=[args.local,review_path,metrics_path,mesh_path,manifest_path,target_path,points_path,
           COLLISION_SOURCE,TOOL_MESH,WORKCELL,*bundle.glob('*.json'),*local['sources'],
           *Path(__file__).parent.glob('*.py'),Path(__file__).with_name('inward_viewer.html')]
    sources=snapshot_sources(paths)
    triangles=load_binary_stl(mesh_path)
    vertices,inverse=np.unique(triangles.reshape(-1,3),axis=0,return_inverse=True)
    faces=inverse.reshape(-1,3).astype(np.int32)
    original=Scene(vertices,faces)
    lib,models,tool_meta=tool_models()
    tool_meta['transforms']={k:{x:v for x,v in m.items() if x!='points_tcp'} for k,m in models.items()}
    nominal=np.array([r['top_m'] for r in baseline])
    affected=[r['label'] for i,r in enumerate(baseline) if np.linalg.norm(deform(nominal,8)[i]-nominal[i])>1e-6]
    output=args.output.resolve();output.mkdir(parents=True,exist_ok=True)
    protocol={'schema':'behav3d.inward_lean_protocol.v1','source_scan':19,'candidate_cycle':18,'next_batch_cycle':19,
              'frame':local['frame'],'units':'m','lean_direction':'-X into the inter-wall gap',
              'lean_parameters':LEAN,'angles_deg':ANGLES,'default_angle_deg':8,'screen_settings':SETTINGS,
              'affected_candidate_labels':affected,'tool_model':tool_meta,'sources_sha256':sources,
              'original_data_unchanged':True,'mechanics_simulated':False,'execution_authorized':False,
              'goal_and_avoidance':'archived full field/avoidance state unavailable; no goal validity claim',
              'contact_scope':'approach geometry study; correction, force and release require physical response validation',
              'next_candidate_dimensions':'C18 nominal dimensions borrowed; C19 YAML has no sizes',
              'zero_control':'same new geometric screens applied to unmodified scan, revealing model/parameter baseline limitations'}
    dump(output/'protocol.json',protocol)
    trials=[]
    for angle in ANGLES:
        print('Checking inward lean',angle,'degrees on scan 19',flush=True)
        moved=deform(vertices,angle);scene=Scene(moved,faces)
        adapted=adapt_targets(baseline,-angle,LEAN)
        seated=seat_targets(adapted,affected,scene,lib)
        supported=search_supported_targets(adapted,affected,scene,lib,SETTINGS)
        proposals=[('continue','Keep archived candidates',baseline),
                   ('defer','Defer the affected region',[r for r in baseline if r['label'] not in affected]),
                   ('adapt','Follow the inward deformation',adapted),
                   ('seat','Adapt and fit bases to the edited surface',seated),
                   ('local','Search nearby supported placements (≤8 mm XY)',supported)]
        plans=[]
        for sid,name,records in proposals:
            checks=evaluate_candidates(records,scene,lib,models,SETTINGS)
            plans.append({'id':sid,'name':name,'targets':records,'checks':checks,
                          'passed_screens':sum(c['geometric_screen']=='passes_declared_screens' for c in checks),
                          'rejected_screens':sum(c['geometric_screen']=='reject' for c in checks),
                          'affected_passed_screens':sum(c['label'] in affected and c['geometric_screen']=='passes_declared_screens' for c in checks),
                          'affected_candidate_count':sum(r['label'] in affected for r in records),
                          'inspection_criterion':'Inspect the same region for geometric change and worsening lean before another batch.',
                          'fully_validated':False,'executable':False,'robot_commands':[]})
        print('  Comparing contact access',flush=True)
        contact=propose_contact(original,scene,deform,angle,lib,models,SETTINGS)
        trials.append({'angle_deg':angle,'plans':plans,'contact':contact,
                       'changed_vertices':int((np.linalg.norm(moved-vertices,axis=1)>1e-12).sum()),
                       'maximum_displacement_mm':float(np.linalg.norm(moved-vertices,axis=1).max()*1000),
                       'all_edited_vertices_move_inward':bool(np.all(moved[:,0]<=vertices[:,0]+1e-12))})
        if angle==8: np.savez_compressed(output/'inward_8deg_mesh.npz',vertices_m=moved,faces=faces)
    np.savez_compressed(output/'recorded_mesh.npz',vertices_m=vertices,faces=faces)
    results={'schema':'behav3d.inward_lean_results.v1','trials':trials,'execution_authorized':False,'robot_commands':[]}
    dump(output/'results.json',results)
    display=vertices[(vertices[:,0]>.005)&(vertices[:,0]<.35)&(vertices[:,1]>-.995)&(vertices[:,1]<-.775)&(vertices[:,2]>-.18)]
    if len(display)>18000: display=display[np.linspace(0,len(display)-1,18000,dtype=int)]
    # Scene extent and contact tool dimensions determine the view; full meshes
    # remain in every query. JSON contains no robot-controller command format.
    data={'protocol':{k:v for k,v in protocol.items() if k!='sources_sha256'},'results':results,
          'recorded':display.tolist(),'scenes':{str(a):deform(display,a).tolist() for a in ANGLES},
          'intended':[local['target'],*local['neighbors']],
          'contact_tool_offsets':{'paddle':paddle_samples()[0].tolist(),
              'extruder_urdf':(models['urdf']['points_tcp'][::3]@lib.rotation_from_z_axis(np.array([-1.,0,0])).T).tolist()}}
    template=Path(__file__).with_name('inward_viewer.html').read_text()
    (output/'viewer.html').write_text(template.replace('/*DATA*/',json.dumps(data,allow_nan=False).replace('<','\\u003c')))
    if sources!=snapshot_sources(paths): raise ValueError('source changed during build')
    dump(output/'manifest.json',{'files_sha256':{p.name:digest(p) for p in output.iterdir() if p.name in
          {'viewer.html','results.json','protocol.json','recorded_mesh.npz','inward_8deg_mesh.npz'}},
          'source_hashes_unchanged':True,'execution_authorized':False})
    print('Wrote',output,flush=True)


if __name__=='__main__': main()
