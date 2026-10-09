"""Offline geometric screens and contact-path proposals on full recorded meshes.

Only numerical geometry is imported. No ROS, sessions, robot clients or force
commands. A screen pass is conditional on its declared tool and geometry models.
"""
import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d

from geometry import bead

ROOT = Path(__file__).resolve().parents[2]
COLLISION_SOURCE = ROOT/'src/behav3d_py/behav3d_py/scalar_field/lib_scalar/extruder_collision.py'
TOOL_MESH = ROOT/'src/custom_workcell/ur20_workcell/meshes/ram_extruder_v2_simplified.dae'
WORKCELL = ROOT/'src/custom_workcell/ur20_workcell/urdf/workcell_macro.xacro'


def collision_library():
    spec = importlib.util.spec_from_file_location('offline_numerical_extruder_collision', COLLISION_SOURCE)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def collada_vertices_faces(path):
    """Restricted reader for this repository's identity-transform triangle mesh."""
    root = ET.parse(path).getroot()
    ns = {'c':'http://www.collada.org/2005/11/COLLADASchema'}
    if float(root.find('c:asset/c:unit', ns).attrib['meter']) != 1:
        raise ValueError('tool mesh must be in metres')
    if root.find('c:asset/c:up_axis', ns).text != 'Z_UP':
        raise ValueError('unsupported tool mesh axes')
    # Match the library's local-geometry convention only when instances and all
    # ancestors are untransformed. Do not silently drop scene transforms.
    def check(node, transformed=False):
        transformed |= any(child.tag.split('}')[-1] in ('matrix','translate','rotate','scale','lookat','skew') for child in node)
        if node.find('c:instance_geometry',ns) is not None and transformed:
            raise ValueError('nonidentity COLLADA instance not supported')
        for child in node.findall('c:node',ns): check(child,transformed)
    for scene in root.findall('c:library_visual_scenes/c:visual_scene',ns): check(scene)
    vertices, faces, offset = [], [], 0
    for mesh in root.findall('c:library_geometries/c:geometry/c:mesh',ns):
        sources={}
        for source in mesh.findall('c:source',ns):
            acc=source.find('c:technique_common/c:accessor',ns)
            if int(acc.attrib['stride'])!=3 or int(acc.attrib.get('offset','0'))!=0:
                raise ValueError('unsupported source layout')
            sources[source.attrib['id']]=np.fromstring(source.find('c:float_array',ns).text,sep=' ').reshape(-1,3)
        v=mesh.find('c:vertices',ns)
        vi=v.find("c:input[@semantic='POSITION']",ns)
        points=sources[vi.attrib['source'][1:]]
        blocks=[]
        for tri in mesh.findall('c:triangles',ns):
            inputs=tri.findall('c:input',ns)
            stride=1+max(int(i.attrib.get('offset','0')) for i in inputs)
            inp=next(i for i in inputs if i.attrib['semantic']=='VERTEX')
            if inp.attrib['source'] != '#'+v.attrib['id']: raise ValueError('unexpected vertex input')
            indices=np.fromstring(tri.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(inp.attrib.get('offset','0'))].reshape(-1,3)
            if len(indices)!=int(tri.attrib['count']): raise ValueError('triangle count mismatch')
            blocks.append(indices+offset)
        if not blocks or mesh.find('c:polylist',ns) is not None: raise ValueError('triangle mesh required')
        vertices.append(points);faces.extend(blocks);offset+=len(points)
    return np.concatenate(vertices),np.concatenate(faces)


def tool_models():
    lib=collision_library()
    vertices,faces=collada_vertices_faces(TOOL_MESH)
    # Add face centroids to the production checker vertex convention. This still
    # remains a sampled surface test, not continuous mesh/arm collision checking.
    samples=np.concatenate([vertices,vertices[faces].mean(axis=1)])
    root=ET.parse(WORKCELL).getroot()
    origin=root.find(".//joint[@name='tool0_to_extruder_tcp']/origin")
    xyz=np.fromstring(origin.attrib['xyz'],sep=' ')
    rpy=np.fromstring(origin.attrib['rpy'],sep=' ')
    models={}
    for name,translation,angles in [('urdf',xyz,rpy),('checker_defaults',lib.TOOL0_TCP_XYZ,lib.TOOL0_TCP_RPY)]:
        transform=lib.transform_from_xyz_rpy(tuple(translation),tuple(angles))
        body=samples[np.linalg.norm(samples-translation,axis=1)>.002]
        models[name]={'points_tcp':lib.transform_points(body,np.linalg.inv(transform)),
                      'xyz_m':list(translation),'rpy_rad':list(angles)}
    return lib,models,{'sample_count_before_tip_exclusion':len(samples),
                       'tip_exclusion_radius_mm':2.,'tool_offset_difference_mm':float(np.linalg.norm(xyz-lib.TOOL0_TCP_XYZ)*1000),
                       'basis':'repository DAE vertices plus face centroids; two repository TCP transforms, neither verified against the recording'}


class Scene:
    def __init__(self,vertices,faces):
        self.vertices=np.asarray(vertices)
        self.faces=np.asarray(faces)
        self.scene=o3d.t.geometry.RaycastingScene()
        mesh=o3d.t.geometry.TriangleMesh(o3d.core.Tensor(self.vertices.astype(np.float32)),
                                       o3d.core.Tensor(self.faces.astype(np.uint32)))
        self.scene.add_triangles(mesh)
        self.z_top=float(self.vertices[:,2].max()+.1)
        self.candidate_cache={}

    def distances(self,points):
        points=np.asarray(points)
        if not len(points): return np.zeros(0)
        return self.scene.compute_distance(o3d.core.Tensor(points.astype(np.float32))).numpy().astype(float)

    def cast(self,origins,directions):
        rays=np.column_stack([np.atleast_2d(origins),np.atleast_2d(directions)]).astype(np.float32)
        result=self.scene.cast_rays(o3d.core.Tensor(rays))
        return result['t_hit'].numpy().astype(float),result['primitive_ids'].numpy().astype(int)

    def vertical_gaps(self,points):
        points=np.asarray(points)
        origins=points.copy();origins[:,2]=self.z_top
        hits,_=self.cast(origins,np.tile([0,0,-1],(len(points),1)))
        return np.where(np.isfinite(hits),points[:,2]-(self.z_top-hits),np.nan)


def interpolate_path(points,step_m=.003):
    if not np.isfinite(step_m) or step_m<=0: raise ValueError('positive path sampling step required')
    output=[]
    for start,end in zip(points[:-1],points[1:]):
        start,end=np.asarray(start),np.asarray(end)
        n=max(1,int(np.ceil(np.linalg.norm(end-start)/step_m)))
        output.extend(start+(end-start)*np.linspace(0,1,n,endpoint=False)[:,None])
    output.append(np.asarray(points[-1]))
    return np.asarray(output)


def swept_clearance(scene,tool_offsets,positions,margin_m=.002):
    positions=np.asarray(positions)
    points=(positions[:,None,:]+np.asarray(tool_offsets)[None,:,:]).reshape(-1,3)
    distances=scene.distances(points).reshape(len(positions),-1)
    mins=distances.min(axis=1)
    worst=int(np.argmin(mins))
    return {'minimum_sampled_clearance_mm':float(mins[worst]*1000),
            'near_surface_samples':int((distances<=margin_m).sum()),
            'waypoint_sample_count':len(positions),'tool_sample_count':len(tool_offsets),
            'margin_mm':margin_m*1000,'screen':'conflict' if np.any(distances<=margin_m) else 'no_sampled_conflict',
            'worst_position_m':positions[worst].tolist(),
            'continuous_collision_verified':False,'inside_outside_verified':False}


def evaluate_candidates(records,scene,lib,models,settings):
    positions=np.array([r['top_m'] for r in records])
    result=[]
    for i,record in enumerate(records):
        b=bead(record);top=np.array(b.top);axis=b.direction;base=top-b.height_m*axis
        rotation=lib.rotation_from_z_axis(axis)
        radius=max(b.width_m/2-b.height_m/2,0)*.7
        phi=np.arange(8)*np.pi/4
        footprint=np.concatenate([base[None,:],base+radius*(np.cos(phi)[:,None]*rotation[:,0]+np.sin(phi)[:,None]*rotation[:,1])])
        gaps=scene.vertical_gaps(footprint)*1000
        close=np.isfinite(gaps)&(np.abs(gaps)<=settings['support_gap_mm'])
        rays_supported=int(close.sum())
        top_gap=float(scene.vertical_gaps(top[None,:])[0]*1000)
        approach=interpolate_path([top+.05*axis,top],settings['path_step_m'])
        key=(id(models),settings['path_step_m'],settings['tool_margin_mm'],*np.round(np.r_[top,axis],10))
        if key not in scene.candidate_cache:
            scene.candidate_cache[key]={name:swept_clearance(scene,m['points_tcp']@rotation.T,approach,settings['tool_margin_mm']/1000)
                                         for name,m in models.items()}
        tools=scene.candidate_cache[key]
        distances=np.linalg.norm(positions-top,axis=1);distances[i]=np.inf
        min_spacing=float(distances.min()*1000) if len(records)>1 else None
        reasons=[]
        if not np.isfinite(top_gap) or top_gap<=0: reasons.append('top_not_above_observed_frontier')
        if rays_supported<settings['minimum_support_rays']: reasons.append('insufficient_nominal_base_support')
        if min_spacing is not None and min_spacing<settings['minimum_center_spacing_mm']: reasons.append('nominal_batch_spacing')
        if any(t['screen']=='conflict' for t in tools.values()): reasons.append('sampled_extruder_approach_conflict')
        finite=gaps[np.isfinite(gaps)]
        result.append({'label':b.label,'base_gap_mm':None if not np.isfinite(gaps[0]) else float(gaps[0]),
                       'top_above_upper_surface_mm':None if not np.isfinite(top_gap) else top_gap,
                       'support_rays_within_gap':rays_supported,'support_ray_count':len(gaps),
                       'footprint_gap_range_mm':None if not len(finite) else [float(finite.min()),float(finite.max())],
                       'minimum_batch_spacing_mm':min_spacing,'tool_models':tools,
                       'tcp_model_disagreement':len({t['screen'] for t in tools.values()})>1,
                       'geometric_screen':'reject' if reasons else 'passes_declared_screens',
                       'rejection_reasons':reasons,'field_goal_validity':'not_replayed',
                       'avoidance_map_validity':'not_replayed','robot_ik':'unverified','executable':False})
    return result


def seat_targets(records,affected_labels,scene,lib):
    """Local Z adjustment from the edited scan, preserving proposed axes and XY."""
    from copy import deepcopy
    result=deepcopy(records)
    for record in result:
        if record['label'] not in affected_labels: continue
        b=bead(record);base=np.array(b.top)-b.height_m*b.direction
        rotation=lib.rotation_from_z_axis(b.direction)
        phi=np.arange(8)*np.pi/4
        radius=max(b.width_m/2-b.height_m/2,0)*.7
        footprint=np.concatenate([base[None,:],base+radius*(np.cos(phi)[:,None]*rotation[:,0]+np.sin(phi)[:,None]*rotation[:,1])])
        gaps=scene.vertical_gaps(footprint)
        finite=gaps[np.isfinite(gaps)]
        if len(finite)<6:
            record['surface_fit_status']='insufficient_ray_hits'
            continue
        shift=-float(np.median(finite))
        record['top_m'][2]+=shift
        record['surface_fit_z_shift_mm']=shift*1000
        record['surface_fit_status']='median_footprint_seated; field goal not regenerated'
    return result


def search_supported_targets(records,affected_labels,scene,lib,settings):
    """Bounded XY alternatives, preserving axes/dimensions and fixed tolerances.

    This proposes geometry near an archived candidate. It does not create a goal
    field or override planner rules. Final body/spacing screens are separate.
    """
    from copy import deepcopy
    result=deepcopy(records)
    for i,record in enumerate(records):
        if record['label'] not in affected_labels:continue
        alternatives=[]
        for dx in [-.008,-.004,0,.004,.008]:
            for dy in [-.008,-.004,0,.004,.008]:
                if np.hypot(dx,dy)>.008+1e-12:continue
                trial=deepcopy(record);trial['top_m'][0]+=dx;trial['top_m'][1]+=dy
                trial=seat_targets([trial],affected_labels,scene,lib)[0]
                if trial.get('surface_fit_status')=='insufficient_ray_hits':continue
                b=bead(trial);base=np.array(b.top)-b.height_m*b.direction
                rotation=lib.rotation_from_z_axis(b.direction);phi=np.arange(8)*np.pi/4
                radius=max(b.width_m/2-b.height_m/2,0)*.7
                footprint=np.concatenate([base[None,:],base+radius*(np.cos(phi)[:,None]*rotation[:,0]+np.sin(phi)[:,None]*rotation[:,1])])
                gaps=scene.vertical_gaps(footprint)*1000
                support=int((np.isfinite(gaps)&(np.abs(gaps)<=settings['support_gap_mm'])).sum())
                shift=float(np.linalg.norm(np.array(trial['top_m'])-record['top_m']))
                alternatives.append((support,-shift,trial))
        if alternatives:
            best=max(alternatives,key=lambda r:r[:2])
            result[i]=best[2]
            result[i]['search_summary']={'tested_placements':13,'placements_with_sufficient_ray_hits':len(alternatives),
                                         'max_xy_change_mm':8,'selection_support_rays':best[0],
                                         'selection_basis':'most base rays within fixed tolerance, then smallest total displacement',
                                         'goal_validity':'not_replayed'}
    return result


def paddle_samples():
    # Concept dimensions only: 12 mm Y x 10 mm Z contact face, 3 mm thick;
    # vertical 6 mm diameter stem rising 80 mm, offset 6 mm behind contact face.
    xs=np.linspace(-.003,0,3);ys=np.linspace(-.006,.006,7);zs=np.linspace(-.005,.005,6)
    grid=np.stack(np.meshgrid(xs,ys,zs,indexing='ij'),axis=-1).reshape(-1,3)
    surface=(np.isclose(grid[:,0],-.003)|np.isclose(grid[:,0],0)|np.isclose(np.abs(grid[:,1]),.006)|np.isclose(np.abs(grid[:,2]),.005))
    ang=np.arange(24)*2*np.pi/24
    stem=np.array([[-.006+.003*np.cos(a),.003*np.sin(a),z] for z in np.linspace(.004,.084,41) for a in ang])
    return np.concatenate([grid[surface],stem]),{'contact_face_yz_mm':[12,10],'plate_thickness_mm':3,
           'stem_diameter_mm':6,'stem_height_mm':80,'mount_and_wrist_modeled':False,
           'design_status':'unbuilt geometric proxy; edge rounding, stiffness and contact compliance unspecified'}


def propose_contact(original_scene,scene,deform,angle,lib,models,settings):
    if angle==0: return {'status':'not_needed_zero_control','proposals':[],'selected_id':None,'executable':False}
    # Contact points come from recorded right-wall triangles, then the prescribed
    # edit. This is not a force/displacement prediction.
    centroids=original_scene.vertices[original_scene.faces].mean(axis=1)
    right=Scene(original_scene.vertices,original_scene.faces[centroids[:,0]>.205])
    paddle,dimensions=paddle_samples()
    proposals=[]
    for yi,y in enumerate([-.90,-.875,-.85]):
        for zi,z in enumerate([-.060,-.050,-.040]):
            origin=np.array([.185,y,z]);hit,_=right.cast(origin,[1,0,0])
            if not np.isfinite(hit[0]): continue
            desired=origin+np.array([hit[0],0,0]);contact=deform(desired[None,:],angle)[0]
            stroke=float(desired[0]-contact[0])
            if stroke<=.0001 or scene.distances(contact[None,:])[0]>.001: continue
            for stand in [.012,.024]:
                standby=contact-np.array([stand,0,0])
                above=standby.copy();above[2]=original_scene.vertices[:,2].max()+.05
                approach_end=contact-np.array([.006,0,0])
                path=interpolate_path([above,standby,approach_end],settings['path_step_m'])
                paddle_test=swept_clearance(scene,paddle,path,settings['tool_margin_mm']/1000)
                rotation=lib.rotation_from_z_axis(np.array([-1.,0,0]))
                nozzle={name:swept_clearance(scene,m['points_tcp']@rotation.T,path,settings['tool_margin_mm']/1000)
                        for name,m in models.items()}
                proposals.append({'id':f'contact-{yi}-{zi}-{int(stand*1000)}',
                    'contact_m':contact.tolist(),'reference_contact_m':desired.tolist(),'requested_contact_shift_mm':stroke*1000,
                    'push_direction':[1,0,0],'tool_reference':'paddle contact-face center; NOT flange or calibrated TCP',
                    'waypoints':[{'phase':'above_gap','point_m':above.tolist()},
                                 {'phase':'standby','point_m':standby.tolist()},
                                 {'phase':'approach_end','point_m':approach_end.tolist()},
                                 {'phase':'contact_search','point_m':contact.tolist()},
                                 {'phase':'desired_correction','point_m':desired.tolist()},
                                 {'phase':'release','point_m':(desired-np.array([.012,0,0])).tolist()},
                                 {'phase':'retreat_above','point_m':[float(desired[0]-.012),float(desired[1]),float(above[2])]}],
                    'paddle_approach':paddle_test,'extruder_approach':nozzle,
                    'contact_and_push_validity':'requires force/contact/material response model; not collision-cleared',
                    'release_validity':'requires post-contact geometry; not evaluated against a fictitious corrected mesh',
                    'restoration_of_whole_artifact':'unproven; one contact shift does not invert regional deformation',
                    'executable':False,'robot_commands':[]})
    eligible=[p for p in proposals if p['paddle_approach']['screen']=='no_sampled_conflict']
    # Prefer greater geometric clearance among fixed contact alternatives. This
    # ranks access only; it is not a recommendation of force or mechanical efficacy.
    best=max(eligible or proposals,key=lambda p:p['paddle_approach']['minimum_sampled_clearance_mm'],default=None)
    return {'status':'approach_candidate_only' if eligible else 'no_sampled_clear_approach',
            'selected_id':None if best is None else best['id'],'selected_is_approach_eligible':bool(eligible),
            'selection_basis':'maximum sampled free-approach clearance; mechanical correction not ranked',
            'proposals':proposals,'paddle':dimensions,'paddle_samples_local_m':paddle.tolist(),
            'sampled_free_space_ends_before_contact_mm':6,'force_N':None,'speed_m_s':None,
            'requires':['tool calibration and mount model','robot IK and full-arm/tool swept collision',
                        'force feedback and material response limits','contact detection and abort criteria','post-contact inspection'],
            'executable':False}
