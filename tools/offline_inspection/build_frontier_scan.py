"""Replay one recorded inspection region without changing the required sweep.

Consumes the actual Agent007 evaluation bundle, not the provisional controller
envelope. This is a stateless, local-file replay; no runtime transport or ROS.
"""
import argparse
from copy import deepcopy
import hashlib
from html import escape
import json
from pathlib import Path

import numpy as np
import yaml

from build_grid_supplement import canonical_digest, validate_assessment
from build_recorded_demo import quat
from frontier_scan import plan_frontier_scan


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_bundle(folder):
    manifest = json.loads((folder / 'manifest.json').read_text())
    if manifest['schema'] != 'agent007.evaluation_bundle.v1':
        raise ValueError('unsupported evaluation bundle')
    for name in ('evaluation.json', 'robot_review_packet.json'):
        if file_digest(folder / name) != manifest['files'][name]:
            raise ValueError('evaluation bundle hash mismatch: ' + name)
    packet = json.loads((folder / 'robot_review_packet.json').read_text())
    evaluation = json.loads((folder / 'evaluation.json').read_text())
    for path, expected in evaluation['source_sha256'].items():
        if file_digest(path) != expected:
            raise ValueError('evaluation source revision changed: ' + path)
    return packet, evaluation


def inspection_requests(packet, evaluation, local):
    """Bind symbolic review IDs to locally assessed geometry via shared sources.

    Agent007's abstract world/base frame is retained as metadata. No coordinate
    transform or frame alias is inferred. All planning coordinates come from the
    independently source-bound local assessment in its explicit robot base frame.
    """
    if packet['schema'] != 'agent007.robot_review_packet.v1':
        raise ValueError('unsupported review packet')
    for key in ('artifact_id', 'evaluation_id', 'scan_job_id', 'candidate_cycle',
                'scan_cycle', 'previous_evaluation_id', 'summary', 'suggestions'):
        if packet[key] != evaluation[key]:
            raise ValueError('packet/evaluation mismatch: ' + key)
    for key in ('candidate_cycle', 'scan_cycle'):
        if type(packet[key]) is not int or packet[key] != local[key]:
            raise ValueError('packet/geometry cycle mismatch')
    if packet['scan_cycle'] != packet['candidate_cycle'] + 1:
        raise ValueError('invalid candidate/scan linkage')
    if (packet['coordinate_units'] != 'm' or local['units'] != 'm'
            or packet['frame_id'] != evaluation['context']['frame_id']):
        raise ValueError('packet units/frame metadata mismatch')
    if not packet['frame_id'] or not local['frame']:
        raise ValueError('explicit frames required')
    for obj in (packet, evaluation):
        for key in ('future_evidence_used', 'fabrication_authorized', 'robot_commands_sent'):
            if obj[key] is not False:
                raise ValueError('offline source may not authorize or use future execution evidence')
    common = set(evaluation['source_sha256']) & set(local['sources'])
    if not any(p.endswith('/review.json') for p in common):
        raise ValueError('missing shared candidate review source')
    for path in common:
        if evaluation['source_sha256'][path] != local['sources'][path]:
            raise ValueError('evaluation and geometry source revisions differ')
    candidates = {c['label']: c for c in evaluation['candidates']}
    if len(candidates) != len(evaluation['candidates']):
        raise ValueError('duplicate candidate identity')
    target = candidates[local['label']]
    if (target['candidate_cycle'] != local['candidate_cycle']
            or target['observation_cycle'] != local['scan_cycle']
            or not np.allclose(target['expected_top_m'], local['target']['top_m'],
                               atol=1e-9, rtol=0)):
        raise ValueError('target identity/anchor differs from bound review')
    suggestions = packet['suggestions']
    if (packet['suggestions_truncated'] is not False
            or packet['suggestion_count'] != len(suggestions)):
        raise ValueError('complete suggestion list required for this replay')
    seen = set()
    requests, deferred = [], []
    region_rules = {
        'review_scan_coverage': {'top'},
        'inspect_geometry': {'top', 'side'},
        'inspect_dds_shape_size': {'top', 'side'},
    }
    allowed = set(region_rules) | {'review_registration', 'review_surface_identity'}
    for suggestion in suggestions:
        sid = suggestion['suggestion_id']
        if not isinstance(sid, str) or not sid.strip() or sid in seen:
            raise ValueError('unique suggestion IDs required')
        seen.add(sid)
        if suggestion['action'] not in allowed:
            raise ValueError('unsupported suggestion action')
        for key, value in (('executable', False), ('fabrication_authorized', False),
                           ('requires_operator_review', True)):
            if suggestion[key] is not value:
                raise ValueError('suggestion execution flags invalid')
        refs, labels = suggestion['evidence_ids'], suggestion['candidate_labels']
        evidence = {'context:registration'} | {c['evidence_id'] for c in candidates.values()}
        if (not refs or len(set(refs)) != len(refs) or not set(refs) <= evidence
                or len(set(labels)) != len(labels) or not set(labels) <= set(candidates)):
            raise ValueError('unbound or duplicate suggestion evidence/candidates')
        if any(candidates[label]['evidence_id'] not in refs for label in labels):
            raise ValueError('candidate suggestion lacks its evidence reference')
        action = suggestion['action']
        if action not in region_rules:
            deferred.append(dict(deepcopy(suggestion), disposition='evidence_review_before_scan_change'))
            continue
        other_labels = [label for label in labels if label != local['label']]
        if other_labels or not labels:
            deferred.append(dict(deepcopy(suggestion), candidate_labels=other_labels,
                                 disposition='outside_this_assessed_region' if other_labels else 'no_target_region'))
        if local['label'] not in labels:
            continue
        indices = [i for i, region in enumerate(local['nominal_surface']['regions'])
                   if region in region_rules[action]]
        requests.append({
            'request_id': sid + ':' + local['label'], 'origin': 'agent007',
            'suggestion_id': sid, 'action': action, 'reason': suggestion['reason'],
            'candidate_labels': [local['label']], 'evidence_ids': list(refs),
            'sample_indices': indices, 'surface_regions': sorted(region_rules[action]),
            'uncertainty_reasons': target['uncertainty_reasons'],
            'mapping_basis': ('cap review targets nominal top; geometric novelty cannot establish cap quality'
                              if action == 'review_scan_coverage' else
                              'position/shape review targets nominal top and sides'),
        })
    binding = {
        'artifact_id': packet['artifact_id'], 'evaluation_id': packet['evaluation_id'],
        'scan_job_id': packet['scan_job_id'], 'packet_frame_metadata': packet['frame_id'],
        'planning_frame': local['frame'], 'coordinate_transform_applied': False,
        'basis': 'symbolic candidate IDs and shared hashed review; planning coordinates exclusively from local assessment',
        'shared_source_paths': sorted(common),
    }
    return requests, deferred, binding


def recorded_baseline(local):
    """Recompose all recorded optical poses and check the cached assessments."""
    manifest_path = next(p for p in local['sources']
                         if p.endswith(f"/cycle_{local['scan_cycle']:04d}/scan/manifest.yaml"))
    extrinsic_path = next(p for p in local['sources'] if p.endswith('/config/extrinsics.yaml'))
    manifest = yaml.safe_load(Path(manifest_path).read_text())
    ext = yaml.safe_load(Path(extrinsic_path).read_text())['frames']['T_tool0_ir']
    group = deepcopy(local['patterns']['recorded'])
    captures = manifest['captures']
    if len(captures) != len(group['poses']):
        raise ValueError('recorded sweep was cropped')
    for capture, pose in zip(captures, group['poses']):
        tool = capture['T_base_tool0']
        rotation = quat(tool['orientation_xyzw'])
        position = np.asarray(tool['position']) + rotation @ np.asarray(ext['xyz'])
        axes = (rotation @ quat(ext['quat_xyzw'])).T
        if (tool['parent'] != local['frame'] or capture['index'] != pose['index']
                or not np.allclose(position, pose['position_m'], rtol=0, atol=1e-10)
                or not np.allclose(axes, pose['optical_axes_world'], rtol=0, atol=1e-10)):
            raise ValueError('recorded sweep order/pose/frame was changed')
        pose.update(frame=local['frame'], pose_kind='camera_optical', executable=False)
    return group, {'manifest_path': manifest_path,
                   'manifest_sha256': local['sources'][manifest_path],
                   'pose_basis': 'recorded T_base_tool0 composed with T_tool0_ir, manifest order unchanged',
                   'scope': 'recorded-cycle replay; not a generated scan for a future print cycle'}


def report_html(result):
    esc = lambda value: escape(str(value))
    decisions = {'keep_frontier_sweep': 'Keep the existing sweep',
                 'keep_frontier_sweep_with_supplement': 'Keep the sweep and append inspection views',
                 'hold_baseline_assessment_rejected': 'Review the baseline assessment before proceeding'}
    statuses = {
        'unresolved_no_new_geometric_opportunity_in_pool': 'Still unresolved; tested views add no new area for this objective',
        'unresolved_within_extra_view_budget': 'Still unresolved within the view budget',
        'unresolved_baseline_assessment_rejected': 'Baseline assessment needs review',
        'additional_geometric_opportunity_inspection_unverified': 'Additional area predicted; inspection outcome unverified',
    }
    dispositions = {'evidence_review_before_scan_change': 'Review evidence before changing the scan',
                    'outside_this_assessed_region': 'Geometry not assessed in this replay',
                    'no_target_region': 'Inspection region needs definition'}
    items = ''.join('<tr><td>' + '</td><td>'.join(map(esc, (
        ', '.join(r['candidate_labels']), r['reason'].replace('_', ' '), ', '.join(r['surface_regions']),
        f"{r['baseline_opportunity_fraction']:.1%} → {r['combined_opportunity_fraction']:.1%}",
        statuses[r['status']]))) + '</td></tr>' for r in result['requests'])
    pending = ''.join('<li>' + esc(', '.join(s['candidate_labels']) or 'Registration')
                      + ': ' + esc(dispositions[s['disposition']]) + '</li>' for s in result['deferred_reviews'])
    return '''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Frontier scan decision</title><style>
body{font:16px/1.55 system-ui;background:#121c26;color:#edf3f6;max-width:1050px;margin:40px auto;padding:0 24px}
h1{font-size:28px}h2{font-size:20px}table{border-collapse:collapse;width:100%;font-size:14px}
td,th{border-bottom:1px solid #405366;padding:12px;text-align:left;overflow-wrap:anywhere}
.table{overflow-x:auto}.decision{padding:18px;background:#213747;border-left:4px solid #78d2cc}
a{color:#89dce0}small{color:#b9cbd4}</style>''' + f'''
<h1>Preserve the upper-frontier sweep</h1>
<p>Recorded replay · C{result['candidate_cycle']} → scan {result['scan_cycle']} · {esc(result['label'])}</p>
<p>The frontier scan supplies the closed mesh used with the goal field and ray intersections to generate printing candidates. Inspection views may only be appended.</p>
<p class="decision"><strong>{esc(decisions[result['decision']])}</strong><br>
{result['baseline_capture_count']} original captures preserved in order · {result['extra_capture_count']} additions · budget {result['max_extra_views']}</p>
<h2>Inspection request</h2><div class="table"><table><thead><tr><th>Region</th><th>Reason</th><th>Surface objective</th><th>Modeled opportunity</th><th>Result</th></tr></thead><tbody>{items}</tbody></table></div>
<p>This percentage describes only the requested nominal surface. It does not measure completeness of the upper-frontier mesh or resolve Agent007's uncertainty. Keeping the sweep preserves its inputs; reconstructed mesh quality and candidate selection still need captured-data validation.</p>
<h2>Other reviews remain open</h2><ul>{pending}</ul>
<p>The report assesses one region. Other labels have not been silently cleared or counted as inspected.</p>
<h2>Development boundary</h2><p>The existing sweep remains authoritative, including its normal height adjustment in the ROS workflow. This replay reuses the recorded cycle's poses. No future-cycle scan, robot trajectory, capture or candidate plan is executed.</p>
<p>Selection adds a view only for new modeled opportunity on an explicitly requested region. Repeated observations, angular diversity and sensor quality remain outside this first selection rule.</p>
<p><a href="frontier_scan.json">Full proposal and provenance</a></p>
<small>Evaluation: {esc(result['source_binding']['evaluation_id'])}</small></html>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', type=Path, required=True)
    parser.add_argument('--evaluation-bundle', type=Path, required=True)
    parser.add_argument('--max-extra', type=int, default=2)
    args = parser.parse_args()
    local = json.loads((args.demo / 'local_scene.json').read_text())
    validate_assessment(local)
    packet, evaluation = load_bundle(args.evaluation_bundle)
    requests, deferred, binding = inspection_requests(packet, evaluation, local)
    baseline, provenance = recorded_baseline(local)
    result = plan_frontier_scan(baseline, local['patterns']['direction_pool'],
                               local['nominal_surface'], requests,
                               frame=local['frame'], units=local['units'],
                               max_extra=args.max_extra)
    result.update(label=local['label'], candidate_cycle=local['candidate_cycle'],
                  scan_cycle=local['scan_cycle'], source_binding=binding,
                  baseline_source=provenance, deferred_reviews=deferred,
                  source_assessment_sha256=local['assessment_sha256'],
                  registration_status=packet['registration_status'],
                  sources={str((args.demo / 'local_scene.json').resolve()): file_digest(args.demo / 'local_scene.json'),
                           **{str((args.evaluation_bundle / name).resolve()): file_digest(args.evaluation_bundle / name)
                              for name in ('manifest.json', 'evaluation.json', 'robot_review_packet.json')}})
    result['proposal_sha256'] = canonical_digest(result)
    (args.demo / 'frontier_scan.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    (args.demo / 'frontier_scan_report.html').write_text(report_html(result))
    print(json.dumps({k: result[k] for k in ('decision', 'baseline_capture_count', 'extra_capture_count')}))
    for request in result['requests']:
        print(request['request_id'], request['status'])


if __name__ == '__main__':
    main()
