"""Fixed-settings, source-bound scan-policy replay across recorded cases.

Uses the existing policy unchanged. Cases are selected by stage and Agent007
finding before pose scoring; results are independent proposals, not one joint scan.
"""
import argparse
from collections import Counter
from copy import deepcopy
from html import escape
import json
from pathlib import Path
import re
import time

import numpy as np
import yaml

from assess_local_scene import directional_pool
from bead_surface import nominal_surface
from build_frontier_scan import (file_digest, inspection_requests, load_bundle,
                                 recorded_baseline, report_html)
from build_grid_supplement import canonical_digest, validate_assessment
from build_recorded_demo import quat
from frontier_scan import plan_frontier_scan
from local_scene import NominalBead, ObservedScene, assess_views, coverage
from visibility import load_binary_stl


CASES = [
    ('C1:T1', 'early cap uncertainty'),
    ('C1:T3', 'early missing-and-extra shape review'),
    ('C1:T4', 'within-tolerance comparison with broad shape-review suggestion'),
    ('C5:T1', 'repeat the original cap case as a regression control'),
    ('C5:T3', 'identity ambiguity'),
    ('C5:T7', 'shape review beside the original case'),
    ('C11:T2', 'middle-stage shape review'),
    ('C11:T3', 'middle-stage identity ambiguity'),
    ('C11:T4', 'cap uncertainty with within-tolerance DDS evidence'),
    ('C18:T2', 'late positional discrepancy'),
    ('C18:T7', 'late shape review'),
    ('C22:T1', 'late insufficient observation'),
]


def issued_bead(candidate, entries, metrics):
    """Verify the recorded DDS sign convention against issued position AND axis."""
    match = re.fullmatch(r'C(\d+):T(\d+)', candidate['label'])
    if match is None or int(match[1]) != candidate['candidate_cycle']:
        raise ValueError('invalid candidate label/cycle')
    rows = [r for r in entries if r['index'] == int(match[2]) - 1]
    if len(rows) != 1:
        raise ValueError('issued target identity missing or duplicated')
    plane = rows[0]['plane']
    origin = re.search(r'O\(([^)]+)\)', plane)
    axis = re.search(r'Z\(([^)]+)\)', plane)
    if origin is None or axis is None:
        raise ValueError('issued plane is missing origin/axis')
    if any(type(metrics[k]) is not bool for k in ('flip_x', 'flip_y')):
        raise ValueError('explicit DDS sign mapping required')
    signs = np.array([-1 if metrics['flip_x'] else 1, -1 if metrics['flip_y'] else 1, 1])
    top = np.array([float(v) for v in origin[1].split(',')]) * .001 * signs
    direction = np.array([float(v) for v in axis[1].split(',')]) * signs
    direction /= np.linalg.norm(direction)
    if (not np.allclose(top, candidate['position_m'], atol=1e-9, rtol=0)
            or not np.allclose(direction, candidate['axis'], atol=1e-9, rtol=0)
            or not np.isclose(metrics['bead_width_m'], candidate['width_mm'] * .001)
            or not np.isclose(metrics['bead_height_m'], candidate['height_mm'] * .001)):
        raise ValueError('issued geometry/size differs from source-bound review')
    return NominalBead(candidate['label'], tuple(top), tuple(direction),
                       candidate['width_mm'] * .001, candidate['height_mm'] * .001)


def load_cycle(agent_root, scan_cycle, settings, settings_path):
    bundle = agent_root / f'outputs/integrated_evaluation_260728_133013_final/cycle_{scan_cycle:04d}/evaluation'
    packet, evaluation = load_bundle(bundle)
    review_path = next(Path(p) for p in evaluation['source_sha256']
                       if p.endswith(f'/cycle_{scan_cycle:04d}/review.json'))
    review = json.loads(review_path.read_text())
    if (review['scan_cycle'] != scan_cycle or review['assessed_candidate_cycle'] != scan_cycle - 1
            or packet['scan_cycle'] != scan_cycle):
        raise ValueError('review cycle mismatch')
    binding = review['source_bindings']['current_scan']
    mesh_ply = Path(binding['path'])
    if file_digest(mesh_ply) != binding['sha256']:
        raise ValueError('reconstruction changed since review')
    mesh = mesh_ply.with_suffix('.stl')
    dataset = mesh_ply.parents[4]
    manifest_path = mesh_ply.parents[1] / 'manifest.yaml'
    extrinsic_path = dataset / 'config/extrinsics.yaml'
    intrinsic_path = dataset / 'config/depth_intrinsics.yaml'
    sources = {str(review_path): file_digest(review_path), str(mesh_ply): binding['sha256'],
               str(mesh): file_digest(mesh), str(settings_path): file_digest(settings_path)}
    for path in (manifest_path, extrinsic_path, intrinsic_path):
        digest = file_digest(path)
        if evaluation['source_sha256'].get(str(path)) != digest:
            raise ValueError('camera source not bound to evaluation: ' + str(path))
        sources[str(path)] = digest
    beads = []
    for c in review['assessed_candidates']:
        if c['candidate_cycle'] != scan_cycle - 1 or c['expected_observation_cycle'] != scan_cycle:
            raise ValueError('mixed candidate cycles')
        for key in ('source_yaml', 'source_metrics'):
            path, digest = c[key], c[key + '_sha256']
            if file_digest(path) != digest or evaluation['source_sha256'].get(path) != digest:
                raise ValueError('unbound issued geometry: ' + path)
            sources[path] = digest
        entries = yaml.safe_load(Path(c['source_yaml']).read_text())['targets']
        metrics = json.loads(Path(c['source_metrics']).read_text())
        beads.append(issued_bead(c, entries, metrics))
    if len({b.label for b in beads}) != len(beads):
        raise ValueError('duplicate current bead identity')
    manifest = yaml.safe_load(manifest_path.read_text())
    ext = yaml.safe_load(extrinsic_path.read_text())['frames']['T_tool0_ir']
    poses = []
    for capture in manifest['captures']:
        tool = capture['T_base_tool0']
        if tool['parent'] != settings['frame']:
            raise ValueError('recorded frame differs from test settings')
        rotation = quat(tool['orientation_xyzw'])
        poses.append({'index': capture['index'],
                      'position_m': (np.asarray(tool['position']) + rotation @ np.asarray(ext['xyz'])).tolist(),
                      'optical_axes_world': (rotation @ quat(ext['quat_xyzw'])).T.tolist(),
                      'frame': settings['frame'], 'pose_kind': 'camera_optical', 'executable': False})
    text = intrinsic_path.read_text()
    matrix = re.search(r'camera_matrix:.*?data:\s*\[([^]]+)\]', text, re.S)
    k = [float(v) for v in matrix[1].split(',')]
    calibration = {'width': int(re.search(r'image_width:\s*(\d+)', text)[1]),
                   'height': int(re.search(r'image_height:\s*(\d+)', text)[1]),
                   'fx': k[0], 'fy': k[4], 'cx': k[2], 'cy': k[5]}
    return dict(bundle=bundle, packet=packet, evaluation=evaluation, beads=beads,
                poses=poses, calibration=calibration, sources=sources, mesh=mesh)


def assess_case(context, label, settings, code_sources):
    target = next(b for b in context['beads'] if b.label == label)
    neighbors = [b for b in context['beads'] if b.label != label]
    surface = nominal_surface(target.top, target.axis, target.width_m, target.height_m)
    pool = directional_pool(target, settings['frame'], settings)
    options = {k: settings[k] for k in ('relation_margin_m', 'neighbor_margin_m',
                                       'endpoint_tolerance_m', 'max_incidence_deg')}
    patterns = {}
    # Reuse the full scene between cases from the same scan; no cropping/decimation.
    if 'scene' not in context:
        context['scene'] = ObservedScene(load_binary_stl(context['mesh']))
    for name, poses in (('recorded', context['poses']), ('direction_pool', pool)):
        rows = assess_views(poses, surface, context['calibration'], context['scene'],
                            target, neighbors, settings['camera_envelope'], **options)
        patterns[name] = {'poses': poses, 'assessments': rows, 'coverage': coverage(rows, surface)}
    local = {'schema': 'behav3d.local_scene_assessment.v1', 'label': label,
             'frame': settings['frame'], 'units': 'm',
             'candidate_cycle': context['packet']['candidate_cycle'],
             'scan_cycle': context['packet']['scan_cycle'], 'target': target.record(),
             'neighbors': [b.record() for b in neighbors], 'nominal_surface': surface,
             'patterns': patterns, 'settings': settings, 'calibration': context['calibration'],
             'sources': {**context['sources'], **code_sources},
             'neighbor_scope': 'same-cycle intended beads; all older recorded geometry remains in whole-mesh ray tests',
             'mesh_basis': 'STL sibling of source-bound PLY; no independent format-equivalence certification',
             'execution_authorized': False, 'robot_commands': []}
    local['assessment_sha256'] = canonical_digest(local)
    return local


def summarize(result, local, candidate, selection_reason):
    rows = local['patterns']['direction_pool']['assessments']
    requests = result['requests']
    return {'label': result['label'], 'scan_cycle': result['scan_cycle'],
            'selection_reason': selection_reason, 'agent_status': candidate['evaluation_status'],
            'dds_status': candidate['dds_status'],
            'request_actions': [r['action'] for r in requests],
            'surface_regions': sorted({region for r in requests for region in r['surface_regions']}),
            'baseline_capture_count': result['baseline_capture_count'],
            'extra_capture_count': result['extra_capture_count'],
            'decision': result['decision'],
            'baseline_preserved': result['baseline_poses'] == [p['pose'] for p in result['capture_order'][:result['baseline_capture_count']]],
            'eligible_pool_views': sum(r['eligible_for_offline_selection'] for r in rows),
            'selected_pool_indices': [p['index'] for p in result['supplemental_poses']],
            'baseline_opportunity_fraction': requests[0]['baseline_opportunity_fraction'] if requests else None,
            'combined_opportunity_fraction': requests[0]['combined_opportunity_fraction'] if requests else None,
            'request_statuses': [r['status'] for r in requests],
            'append_distance_m': result['append_order_metrics']['position_order_length_m'],
            'append_rotation_deg': result['append_order_metrics']['summed_orientation_change_deg'],
            'inspection_resolved': False}


def batch_html(summary):
    rows = []
    for c in summary['cases']:
        if 'error' in c:
            rows.append(f'<tr><td>{escape(c["label"])}</td><td colspan="6">Failed: {escape(c["error"])}</td></tr>')
            continue
        opportunity = ('Evidence review first' if c['baseline_opportunity_fraction'] is None else
                       f"{c['baseline_opportunity_fraction']:.1%} → {c['combined_opportunity_fraction']:.1%}")
        slug = c['label'].lower().replace(':', '-')
        rows.append(f'<tr><td><a href="{slug}/frontier_scan_report.html">{escape(c["label"])}</a><br>scan {c["scan_cycle"]}</td>'
                    f'<td>{escape(c["agent_status"].replace("_", " "))}<br><small>DDS: {escape(c["dds_status"].replace("_", " "))}</small></td>'
                    f'<td>{escape(" + ".join(c["surface_regions"]) or "—")}</td>'
                    f'<td>{c["baseline_capture_count"]} + {c["extra_capture_count"]}</td>'
                    f'<td>{opportunity}</td><td>{c["append_distance_m"]:.2f} m</td>'
                    f'<td>{c["eligible_pool_views"]}/24</td></tr>')
    return '''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Recorded scan-policy cases</title><style>body{font:16px/1.5 system-ui;background:#121c26;color:#edf3f6;max-width:1150px;margin:32px auto;padding:0 20px}
h1{font-size:28px}h2{font-size:20px}table{border-collapse:collapse;width:100%;font-size:14px}td,th{border-bottom:1px solid #405366;padding:12px;text-align:left}small{color:#bed0de}a{color:#88dbe0}.table{overflow-x:auto}.note{padding:18px;background:#213747;border-left:4px solid #78d2cc}</style>
<h1>Testing the scan decision on recorded cases</h1>
<p>Preserve each full recorded frontier sweep. Append at most two inspection views per case, from the same 24-direction pool and unchanged settings.</p>
<p class="note">These are separate case proposals, not a combined scan schedule. Percentages predict opportunity on the requested nominal surface; they are not measured inspection improvement or whole-frontier mesh completeness.</p>
<div class="table"><table><thead><tr><th>Case</th><th>Agent007 finding</th><th>Surface objective</th><th>Sweep + extra</th><th>Modeled opportunity</th><th>Added order length</th><th>Eligible views</th></tr></thead><tbody>''' + ''.join(rows) + '''</tbody></table></div>
<h2>How to interpret this test</h2><ul><li>Cases were chosen by construction stage and recorded findings before scoring. This is a diagnostic sample from one recording, not a representative accuracy benchmark.</li>
<li>Zero additions leaves an inspection open when the current area-based model cannot identify a useful view.</li>
<li>Additional views predict geometric access. Their effects on raw depth, reconstruction, and subsequent printing candidates have not been tested with new captures.</li>
<li>Registration and material identity remain unverified. Camera/tool assumptions and conservative neighbor envelopes affect the results.</li>
<li>A DDS status is provisional recorded evidence; missing or extra does not establish a physical defect.</li></ul><p><a href="batch_results.json">Results and provenance</a> · <a href="case_selection.json">Case selection and fixed settings</a></p></html>'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--agent-root', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--settings', type=Path, default=Path(__file__).with_name('local_scene_settings.json'))
    ap.add_argument('--reuse', action='store_true', help='reuse assessments only after source/code hash validation')
    args = ap.parse_args()
    settings_path = args.settings.resolve()
    settings = json.loads(settings_path.read_text())
    if settings['schema'] != 'behav3d.local_scene_settings.v1' or settings['units'] != 'm':
        raise ValueError('invalid fixed settings')
    code_sources = {str(p.resolve()): file_digest(p) for p in Path(__file__).parent.glob('*.py') if not p.name.startswith('test_')}
    args.output.mkdir(parents=True, exist_ok=True)
    selection = {'cases': [{'label': label, 'reason': reason} for label, reason in CASES],
                 'selection_basis': 'stage and recorded finding, fixed before scoring',
                 'max_extra_views': 2, 'settings': settings, 'code_sources': code_sources}
    (args.output / 'case_selection.json').write_text(json.dumps(selection, indent=2) + '\n')
    summary = {'schema': 'behav3d.frontier_scan_batch.v1', 'cases': [],
               'selection_sha256': canonical_digest(selection), 'max_extra_views_per_case': 2,
               'joint_scan_plan': False, 'execution_authorized': False, 'robot_commands': []}
    context = None
    for label, reason in CASES:
        start = time.monotonic()
        print('Assessing', label, reason, flush=True)
        try:
            cycle = int(re.fullmatch(r'C(\d+):T(\d+)', label)[1]) + 1
            if context is None or context['packet']['scan_cycle'] != cycle:
                context = load_cycle(args.agent_root, cycle, settings, settings_path)
            folder = args.output / label.lower().replace(':', '-')
            folder.mkdir(exist_ok=True)
            path = folder / 'local_scene.json'
            local = None
            if args.reuse and path.exists():
                local = json.loads(path.read_text())
                validate_assessment(local)
                if any(local['sources'].get(p) != h for p, h in {**context['sources'], **code_sources}.items()):
                    raise ValueError('cached assessment sources do not match this run')
                if local['label'] != label or local['settings'] != settings:
                    raise ValueError('cached assessment case/settings mismatch')
            if local is None:
                local = assess_case(context, label, settings, code_sources)
                path.write_text(json.dumps(local, indent=2, allow_nan=False) + '\n')
            requests, deferred, binding = inspection_requests(context['packet'], context['evaluation'], local)
            baseline, provenance = recorded_baseline(local)
            result = plan_frontier_scan(baseline, local['patterns']['direction_pool'], local['nominal_surface'],
                                        requests, frame=local['frame'], units='m', max_extra=2)
            if result['baseline_poses'] != baseline['poses']:
                raise AssertionError('required baseline altered')
            result.update(label=label, candidate_cycle=cycle - 1, scan_cycle=cycle,
                          source_binding=binding, baseline_source=provenance, deferred_reviews=deferred,
                          source_assessment_sha256=local['assessment_sha256'],
                          registration_status=context['packet']['registration_status'],
                          sources={str(path.resolve()): file_digest(path),
                                   **{str((context['bundle'] / name).resolve()): file_digest(context['bundle'] / name)
                                      for name in ('manifest.json', 'evaluation.json', 'robot_review_packet.json')}})
            result['proposal_sha256'] = canonical_digest(result)
            (folder / 'frontier_scan.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
            (folder / 'frontier_scan_report.html').write_text(report_html(result))
            candidate = next(c for c in context['evaluation']['candidates'] if c['label'] == label)
            row = summarize(result, local, candidate, reason)
        except Exception as exc:
            row = {'label': label, 'error': str(exc), 'error_type': type(exc).__name__}
        row['elapsed_s'] = round(time.monotonic() - start, 2)
        summary['cases'].append(row)
        summary['decision_counts'] = dict(Counter(c.get('decision', 'error') for c in summary['cases']))
        summary['complete'] = len(summary['cases']) == len(CASES)
        summary['all_cases_succeeded'] = summary['complete'] and not any('error' in c for c in summary['cases'])
        (args.output / 'batch_results.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
        (args.output / 'report.html').write_text(batch_html(summary))
        print(json.dumps(row), flush=True)
    if not summary['all_cases_succeeded']:
        raise SystemExit('One or more cases failed; see batch_results.json')


if __name__ == '__main__':
    main()
