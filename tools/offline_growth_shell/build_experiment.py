"""Test revised growth shells against recorded scan 19, entirely offline."""
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, data):
    Path(path).write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', required=True, type=Path)
    parser.add_argument('--dependency-path', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/offline_growth_shell/scan19')
    args = parser.parse_args()
    if args.dependency_path:
        sys.path.insert(0, str(args.dependency_path.resolve()))
    sys.path[:0] = [str(ROOT/'tools/offline_planner_replay'), str(ROOT/'tools/offline_recovery')]
    import numpy as np
    from growth import Frontier, fit_shell_offsets, predict_offset, revise_shell, mesh_quality, return_diagnostics
    from comparison import candidate_delta, compare_batches
    from replay import PROFILE, contour_from_mesh, generate, current_targets
    from build_inward_lean import deform, LEAN, SETTINGS
    from viability import Scene, tool_models, evaluate_candidates, COLLISION_SOURCE, TOOL_MESH, WORKCELL

    output = args.output.resolve()
    if output == args.evaluation.resolve().parent or args.evaluation.resolve().parent in output.parents:
        raise ValueError('experiment must not write into the Agent007 source bundle')
    output.mkdir(parents=True, exist_ok=True)
    sources = {}

    def track(path):
        path = Path(path).resolve()
        sources[str(path)] = sha(path)
        return path

    for path in [*Path(__file__).parent.glob('*.py'), Path(__file__).with_name('viewer.html'),
                 ROOT/'tools/offline_planner_replay/replay.py',
                 * (ROOT/'tools/offline_recovery').glob('*.py'),
                 * (ROOT/'src/behav3d_py/behav3d_py/scalar_field/lib_scalar').glob('*.py'),
                 ROOT/'tools/offline_inspection/local_scene.py', ROOT/'tools/offline_inspection/visibility.py',
                 COLLISION_SOURCE, TOOL_MESH, WORKCELL]:
        track(path)
    fixture_path = track(ROOT/'outputs/offline_planner_replay/recorded_260728/baseline_fixture.npz')
    replay_manifest = json.loads(track(fixture_path.with_name('manifest.json')).read_text())
    if sha(fixture_path) != replay_manifest['files_sha256']['baseline_fixture.npz']:
        raise ValueError('baseline fixture changed')
    with np.load(fixture_path, allow_pickle=False) as saved:
        field = {key: saved['field_'+key].copy() for key in ['vertices', 'faces', 'heat', 'offset']}
        evaluation = json.loads(track(args.evaluation).read_text())
        if evaluation['scan_cycle'] != 19 or evaluation['candidate_cycle'] != 18:
            raise ValueError('experiment is bound to scan 19 / candidates 18')
        # Resolve the producer's abstract world/base label by matching every
        # planned top to the archived, base-frame candidate PLY through scan 19.
        for candidate in evaluation['candidates']:
            cycle = candidate['candidate_cycle']
            index = int(candidate['label'].split(':T')[1])-1
            if not np.allclose(candidate['expected_top_m'], saved[f'c{cycle}_archived'][index], atol=.00001, rtol=0):
                raise ValueError('shell-to-recording coordinate/candidate mismatch')
    selected = [c for c in evaluation['candidates'] if c['wall'] == 1
                and c['quantitative_position_evidence_eligible']
                and c['expected_middle_m'][2] >= -.08]
    if len(selected) < 4:
        raise ValueError('insufficient eligible shell pairs in upper right wall')
    top = np.asarray([c['expected_top_m'] for c in selected])
    middle = np.asarray([c['expected_middle_m'] for c in selected])
    cap = np.asarray([c['observed_cap_m'] for c in selected])
    evidence = {'schema': 'behav3d.shell_recovery_evidence.v1', 'source_evaluation_sha256': sha(args.evaluation),
                'frame': 'ur20_base_link', 'units': 'm', 'candidate_labels': [c['label'] for c in selected],
                'observation_cycles': [c['observation_cycle'] for c in selected],
                'current_scan_pair_count': sum(c['observation_cycle'] == 19 for c in selected),
                'expected_top_m': top.tolist(), 'expected_middle_m': middle.tolist(), 'observed_cap_m': cap.tolist(),
                'observed_middle_proxy_m': [c['observed_middle_proxy_m'] for c in selected],
                'expected_shell': evaluation['shell']['expected_mesh'], 'observed_shell': evaluation['shell']['observed_mesh'],
                'registration_status': evaluation['context']['registration_status'],
                'past_observations_remeasured': False, 'physical_failure_confirmed': False,
                'frame_binding': 'all 122 expected tops matched archived candidate PLY in ur20_base_link to 0.01 mm'}
    inward = ROOT/'outputs/offline_recovery/inward_cycle19'
    mesh_path = track(inward/'recorded_mesh.npz')
    mesh_manifest = json.loads(track(inward/'manifest.json').read_text())
    if sha(mesh_path) != mesh_manifest['files_sha256']['recorded_mesh.npz']:
        raise ValueError('recorded mesh changed')
    with np.load(mesh_path, allow_pickle=False) as data:
        scan_vertices, scan_faces = data['vertices_m'], data['faces']
    old_path = track(inward/'results.json')
    if sha(old_path) != mesh_manifest['files_sha256']['results.json']:
        raise ValueError('archived target results changed')
    old = json.loads(old_path.read_text())
    original_batch = old['trials'][0]['plans'][0]['targets']
    reference = next(r for r in original_batch if r['label'] == 'C19:T7')
    peers = [r for r in original_batch if r['label'] != 'C19:T7']
    base_contour, _, _ = contour_from_mesh(scan_vertices, scan_faces, field)
    frontier = Frontier.from_contour(base_contour)
    lib, models, tool_meta = tool_models()
    settings = dict(SETTINGS, minimum_center_spacing_mm=PROFILE['min_spacing_m']*1000,
                    spacing_basis='36 mm endpoint spacing from the inferred replay profile; no assumed overlap')
    profiles = [('original', 'Original goal', None), ('follow', 'Keep the fitted offset', None),
                ('return20', 'Return over 20 mm', .02), ('return40', 'Return over 40 mm', .04),
                ('return60', 'Return over 60 mm', .06)]
    trials, display, meshes = [], {}, {'faces': field['faces'], 'original_vertices_m': field['vertices'],
                                     'transported_scalar': field['heat']}
    for angle in [0, 4, 8, 12]:
        print(f'Building growth shells: {angle} degrees inward lean', flush=True)
        edited = deform(scan_vertices, angle)
        scene = Scene(edited, scan_faces)
        archived_checks = evaluate_candidates(original_batch, scene, lib, models, settings)
        archived_candidates = []
        for record, check in zip(original_batch, archived_checks):
            check['field_goal_validity'] = 'archived_target_not_regenerated_for_edited_scan'
            archived_candidates.append({'id': record['label'], 'target': record, 'check': check,
                'source_m': None, 'source_basis': 'not recovered for archived targets',
                'passes_experimental_geometry': not check['rejection_reasons'],
                'rejection_reasons': check['rejection_reasons'], 'structural_stability': 'unverified',
                'executable': False})
        archived_reference = next(c for c in archived_candidates if c['id'] == reference['label'])
        # Reapply the producer's middle-proxy convention AFTER editing cap
        # positions. Deforming nominal middles directly would add a false
        # height-dependent offset under shear.
        edited_caps = deform(cap, angle)
        edited_middle = edited_caps-(top-middle)
        fit = fit_shell_offsets(top, middle, edited_middle)
        near = edited[(edited[:, 0] > .005) & (edited[:, 0] < .34) & (edited[:, 1] > -.99)
                      & (edited[:, 1] < -.78) & (edited[:, 2] > -.115)]
        if len(near) > 10000:
            near = near[np.linspace(0, len(near)-1, 10000, dtype=int)]
        display[str(angle)] = {'scan_points': np.round(near, 6).tolist(), 'paired_observed_m': np.round(edited_middle, 6).tolist(),
                               'paired_expected_m': np.round(middle, 6).tolist()}
        angle_cases = []
        for key, title, height in profiles:
            mode = key if key in {'original', 'follow'} else 'return'
            revised = revise_shell(field['vertices'], frontier, fit, return_height_m=height, mode=mode)
            quality = mesh_quality(field['vertices'], revised, field['faces'])
            changed = dict(field, vertices=revised)
            contour, lines, _ = contour_from_mesh(edited, scan_faces, changed)
            ordinary = generate(contour, lines, changed)
            batch, batch_stats = current_targets(ordinary, changed)
            batch_records = [dict(reference, label=f'{key}-batch-{i+1}', top_m=position.tolist(),
                                  axis=axis.tolist(), presence='offline_proposal')
                             for i, (position, axis) in enumerate(zip(batch.end_points, batch.end_z_dirs))]
            batch_checks = evaluate_candidates(batch_records, scene, lib, models, settings) if batch_records else []
            batch_candidates = []
            for i, (record, check) in enumerate(zip(batch_records, batch_checks)):
                source_index = int(batch_stats['kept_indices'][i])
                source = ordinary.source_points[source_index]
                direction = np.asarray(record['top_m'])-source
                upward = bool(direction[2] > 1e-9)
                reasons = list(check['rejection_reasons'])
                if not upward:
                    reasons.append('candidate_does_not_advance_upward')
                if quality['degenerate_faces'] or quality['reversed_faces_relative_to_source']:
                    reasons.append('revised_field_face_quality')
                check['field_goal_validity'] = 'generated_from_versioned_shell_proposal'
                batch_candidates.append({'id': record['label'], 'target': record, 'source_m': source.tolist(),
                    'source_selection_index': source_index, 'start_m': batch.start_points[i].tolist(),
                    'start_axis': batch.start_z_dirs[i].tolist(), 'check': check, 'upward_growth': upward,
                    'source_to_top_tilt_deg': float(np.degrees(np.arctan2(np.linalg.norm(direction[:2]), direction[2]))),
                    'passes_experimental_geometry': not reasons, 'rejection_reasons': reasons,
                    'structural_stability': 'unverified', 'executable': False})
            # Test one-at-a-time alternatives in the same local wall region.
            # This 20 mm neighborhood is an explicit new experiment scope, not
            # the prior 8 mm free-space proposal search.
            region = ((contour[:, 0] > .19) & (contour[:, 2] > -.075)
                      & (np.linalg.norm(contour[:, :2]-np.asarray(reference['top_m'])[:2], axis=1) <= .02))
            alternatives, seen = [], set()
            for index in np.flatnonzero(region):
                mask = np.zeros(len(contour), bool)
                mask[index] = True
                candidate = generate(contour, lines, changed, mask=mask, count=1)
                if not len(candidate.points):
                    continue
                targets, _ = current_targets(candidate, changed)
                position, axis = targets.end_points[0], targets.end_z_dirs[0]
                identity = tuple(np.round(np.r_[position, axis], 10))
                if identity in seen:
                    continue
                seen.add(identity)
                record = dict(reference, label=f'{key}-contour-{int(index)}', top_m=position.tolist(),
                              axis=axis.tolist(), presence='offline_proposal')
                check = evaluate_candidates([record, *peers], scene, lib, models, settings)[0]
                check['field_goal_validity'] = 'generated_from_versioned_shell_proposal'
                source = candidate.source_points[0]
                direction = position-source
                upward = bool(direction[2] > 1e-9)
                tilt = float(np.degrees(np.arctan2(np.linalg.norm(direction[:2]), direction[2])))
                reasons = list(check['rejection_reasons'])
                if not upward:
                    reasons.append('candidate_does_not_advance_upward')
                if quality['degenerate_faces'] or quality['reversed_faces_relative_to_source']:
                    reasons.append('revised_field_face_quality')
                alternatives.append({'id': record['label'], 'target': record, 'source_m': source.tolist(),
                    'check': check, 'upward_growth': upward, 'source_to_top_tilt_deg': tilt,
                    'source_distance_to_scan_mm': float(scene.distances(source[None, :])[0]*1000),
                    'top_displacement_from_archived_mm': float(np.linalg.norm(position-reference['top_m'])*1000),
                    'passes_experimental_geometry': not reasons, 'rejection_reasons': reasons,
                    'structural_stability': 'unverified', 'executable': False})
                alternatives[-1]['versus_archived'] = candidate_delta(alternatives[-1], archived_reference)
            ranked = sorted(alternatives, key=lambda c: (not c['passes_experimental_geometry'],
                -c['check']['support_rays_within_gap'], c['top_displacement_from_archived_mm'], c['id']))
            best = ranked[0] if ranked else None
            promising = [c for c in alternatives if c['passes_experimental_geometry']]
            diagnostic = return_diagnostics(field['vertices'], frontier, fit, height) if key != 'original' else None
            anchor_points = base_contour[(base_contour[:, 0] > .19) & (base_contour[:, 2] > -.075)]
            modeled_anchor = revise_shell(anchor_points, frontier, fit, return_height_m=height, mode=mode)
            # Sampled modeled anchor-to-edited scan distances are an independent
            # diagnostic; points are not declared supported just by construction.
            distances = scene.distances(modeled_anchor)*1000
            case = {'id': f'lean{angle}-{key}', 'key': key, 'title': title, 'angle_deg': angle,
                'return_height_mm': None if height is None else height*1000,
                'revised_goal_sha256': hashlib.sha256(revised.astype('<f8').tobytes()+field['faces'].astype('<i4').tobytes()).hexdigest(),
                'geometry_quality': quality, 'return_diagnostics': diagnostic,
                'maximum_goal_change_mm': float(np.linalg.norm(revised-field['vertices'], axis=1).max()*1000),
                'original_z_coordinates_preserved': bool(np.array_equal(revised[:, 2], field['vertices'][:, 2])),
                'anchor_distance_median_mm': float(np.median(distances)), 'anchor_distance_max_mm': float(distances.max()),
                'anchor_distance_basis': 'mapped original recorded frontier versus edited scan, before contour regeneration; not candidate-base support',
                'normal_planner_raw_count': len(ordinary.points), 'normal_planner_current_count': batch.count,
                'generated_batch': {'candidates': batch_candidates,
                    'secondary_rule_stats': {k: v.tolist() if isinstance(v, np.ndarray) else v for k,v in batch_stats.items()},
                    'spacing_context': 'other targets in this generated batch; no deposited peer material added',
                    'passes_experimental_geometry_count': sum(c['passes_experimental_geometry'] for c in batch_candidates),
                    'executable': False},
                'batch_versus_archived': compare_batches(batch_candidates, archived_candidates),
                'alternatives': ranked, 'best_comparison_id': None if best is None else best['id'],
                'promising_count': len(promising), 'selected_for_execution': None,
                'checks_pending': ['structural_stability', 'material_response', 'future_deposition_support',
                                   'calibrated_bead_model', 'avoidance_state', 'robot_ik_and_full_tool_clearance'],
                'decision': 'no_failure_assumed_control' if angle == 0 else 'offline_review_only',
                'executable': False, 'robot_commands': []}
            angle_cases.append(case)
            meshes[case['id'].replace('-', '_')] = revised
            print(f"  {key}: {len(alternatives)} alternatives, {len(promising)} pass geometric screens", flush=True)
        unmodified_candidates = angle_cases[0]['generated_batch']['candidates']
        for case in angle_cases:
            case['batch_versus_unchanged_goal'] = compare_batches(case['generated_batch']['candidates'], unmodified_candidates)
        trials.append({'angle_deg': angle, 'fit': fit, 'cases': angle_cases,
                       'archived_candidates_on_same_scan': archived_candidates,
                       'controlled_edit': angle != 0, 'physical_failure_detected': False})
    limits = ['Shell faces and middles are inferred proxies; older pairs are last-known observations.',
              'Controlled edits move recorded cap proxies and the full scan; Agent007 detection is not rerun or claimed.',
              'Registration and a stable/intact artifact are assumptions for comparing strategies, not measured conclusions.',
              'A fitted offset is assumed uniform laterally at each height; local extrapolation is unvalidated.',
              'Original scalar values are transported with vertex identity; gradients are recomputed on revised geometry. No unknown heat-solver settings are invented.',
              'Below-frontier field collar is a numerical reference change, never movement of already printed material.',
              'Return completeness is sampled, and mesh self-intersections have not been checked.',
              'Only the next bead alternatives are screened against observed material; no future deposition is treated as measured support.',
              'The 36 x 16 mm bead and 6/9 support rule remain experimental; the ordinary control also exposes their limitations.',
              'No material strength, curing, load, buckling or contact simulation establishes that this wall stands.',
              'A geometry-screen pass is not a printable or structurally stable plan. Full frontier scanning remains required.']
    result = {'schema': 'behav3d.growth_shell_experiment.v2', 'frame': 'ur20_base_link', 'units': 'm',
              'recording': '260728_133013', 'scan_cycle': 19, 'evidence': {k:v for k,v in evidence.items() if k not in
                {'expected_shell', 'observed_shell', 'expected_top_m', 'expected_middle_m', 'observed_cap_m', 'observed_middle_proxy_m'}},
              'planner_profile': PROFILE, 'screen_settings': settings, 'lean_parameters': LEAN,
              'goal_revision_limit_mm': 12., 'candidate_region_radius_mm': 20., 'tool_model': tool_meta,
              'trials': trials, 'limitations': limits, 'source_sha256': sources,
              'runtime': {n: importlib.metadata.version(n) for n in ['numpy','scipy','open3d','potpourri3d']},
              'executable': False, 'robot_commands': []}
    if any(sha(path) != value for path,value in sources.items()):
        raise ValueError('source changed during the experiment')
    dump(output/'results.json', result)
    comparison = {'schema': 'behav3d.offline_candidate_comparison.v1', 'frame': result['frame'], 'units': 'm',
        'scan_cycle': 19, 'source_sha256': sources, 'screen_settings': settings,
        'comparison_contract': {
            'archived': 'original C19 targets, unchanged and rescored on each edited scan',
            'unchanged_goal': 'existing planner rerun on the same edited scan and original goal',
            'revised_goal': 'same planner and edited scan, using the proposed growth field',
            'batch_matching': 'one-to-one endpoint association within 36 mm, not target identity or a motion bound',
            'local_alternatives': 'each replaces C19:T7 alone with the same six archived peers reserved; never a combined batch',
            'dimensions': '36 x 16 mm proxy borrowed from C18, not recorded C19 dimensions',
            'axis': 'unit tool Z direction only; not a complete robot pose or trajectory'},
        'trials': [{'angle_deg': t['angle_deg'], 'archived_candidates': t['archived_candidates_on_same_scan'],
                    'cases': [{k:c[k] for k in ['id', 'key', 'title', 'revised_goal_sha256', 'generated_batch',
                         'batch_versus_archived', 'batch_versus_unchanged_goal', 'alternatives', 'checks_pending']}
                              for c in t['cases']]} for t in trials],
        'executable': False, 'robot_commands': []}
    dump(output/'candidate_comparison.json', comparison)
    dump(output/'shell_evidence.json', evidence)
    np.savez_compressed(output/'proposed_fields.npz', **meshes)
    view_faces = field['faces'][np.all(field['vertices'][field['faces'], 2] > -.09, axis=1)]
    data = {'result': result, 'scenes': display, 'field_vertices': np.round(field['vertices'], 6).tolist(),
            'field_faces': view_faces[::2].tolist(), 'proposals': {k:np.round(v, 6).tolist() for k,v in meshes.items() if k.startswith('lean')},
            'reference': reference}
    template = Path(__file__).with_name('viewer.html').read_text()
    (output/'viewer.html').write_text(template.replace('/*DATA*/', json.dumps(data, allow_nan=False).replace('<', '\\u003c')))
    dump(output/'manifest.json', {'files_sha256': {p:sha(output/p) for p in ['results.json','shell_evidence.json','proposed_fields.npz','viewer.html','candidate_comparison.json']},
                                 'source_hashes_unchanged': True, 'executable': False})
    print('Wrote', output, flush=True)


if __name__ == '__main__':
    main()
