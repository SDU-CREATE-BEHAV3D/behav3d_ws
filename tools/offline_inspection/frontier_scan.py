"""Offline supplements to a mandatory upper-frontier capture sequence.

The caller supplies the existing sweep and evidence-backed regions to inspect.
This module never regenerates, crops, reorders or drops that sweep. Cached ray
assessments predict geometric opportunities only, not reconstruction quality.
"""
from copy import deepcopy
from itertools import combinations
import numpy as np

from grid_supplement import (
    OPPORTUNITY, _validated_group, order_metrics, shortest_append_order,
)


def plan_frontier_scan(baseline, pool, surface, requests, *, frame, units,
                       max_extra=2):
    """Select at most four additions, scoring only explicitly requested samples.

    Requests identify a reason, evidence IDs and sample indices in this surface.
    Their union is area-weighted once, so overlapping requests cannot inflate gain.
    A geometry-development caller can supply frontier samples through this same
    interface. Detecting frontier deficiencies is outside this selector.
    """
    if not isinstance(frame, str) or not frame.strip() or units != 'm':
        raise ValueError('explicit frame and metre units required')
    if type(max_extra) is not int or not 0 <= max_extra <= 4:
        raise ValueError('extra-view budget must be between zero and four')
    points = np.asarray(surface['points_m'], dtype=float)
    n = len(points)
    weights = np.asarray(surface['area_weights_m2'], dtype=float)
    if (not n or points.shape != (n, 3) or not np.isfinite(points).all()
            or weights.shape != (n,) or not np.isfinite(weights).all()
            or np.any(weights <= 0)):
        raise ValueError('finite sample positions and positive area weights required')
    bp, br = _validated_group(baseline, n, frame)
    pp, pr = _validated_group(pool, n, frame)
    if not bp or len(pp) > 32:
        raise ValueError('mandatory nonempty sweep and pool of at most 32 required')
    if not isinstance(requests, list):
        raise ValueError('explicit inspection request list required')
    requested = np.zeros(n, dtype=bool)
    ids = set()
    for request in requests:
        for key in ('request_id', 'reason', 'origin'):
            if not isinstance(request.get(key), str) or not request[key].strip():
                raise ValueError('request identity, origin and reason required')
        if request['request_id'] in ids:
            raise ValueError('duplicate inspection request')
        ids.add(request['request_id'])
        refs = request.get('evidence_ids')
        indices = request.get('sample_indices')
        if (not isinstance(refs, list) or not refs
                or any(not isinstance(x, str) or not x.strip() for x in refs)
                or len(set(refs)) != len(refs)):
            raise ValueError('unique evidence references required')
        if (not isinstance(indices, list) or not indices
                or any(type(i) is not int or not 0 <= i < n for i in indices)
                or len(set(indices)) != len(indices)):
            raise ValueError('unique sample indices within the assessed surface required')
        requested[indices] = True

    visible = lambda row: np.array([s == OPPORTUNITY for s in row['outcomes']])
    baseline_visible = np.any([visible(row) for row in br], axis=0)
    selected, metrics = [], order_metrics([])
    baseline_rejected = [r['index'] for r in br if not r['eligible_for_offline_selection']]
    best_key = (0., 0, 0., 0., ())
    if not baseline_rejected and max_extra and requests:
        masks = {}
        for i, row in enumerate(pr):
            new = requested & ~baseline_visible & visible(row)
            if row['eligible_for_offline_selection'] and new.any():
                masks[i] = new
        for size in range(1, min(max_extra, len(masks)) + 1):
            for combo in combinations(masks, size):
                union = np.any([masks[i] for i in combo], axis=0)
                gain = float(weights[union].sum())
                ordered, cost = shortest_append_order(bp[-1], [pp[i] for i in combo])
                key = (round(gain, 15), -size, -cost['position_order_length_m'],
                       -cost['summed_orientation_change_deg'],
                       tuple(-p['index'] for p in ordered))
                if key > best_key:
                    best_key, selected, metrics = key, ordered, cost

    by_id = {r['index']: r for r in pr}
    combined_visible = baseline_visible.copy()
    for pose in selected:
        combined_visible |= visible(by_id[pose['index']])
    all_pool_visible = baseline_visible.copy()
    for row in pr:
        if row['eligible_for_offline_selection']:
            all_pool_visible |= visible(row)
    reviews = []
    for request in requests:
        indices = request['sample_indices']
        area = float(weights[indices].sum())
        before = float(weights[indices][baseline_visible[indices]].sum() / area)
        after = float(weights[indices][combined_visible[indices]].sum() / area)
        possible = float(weights[indices][all_pool_visible[indices]].sum() / area)
        if baseline_rejected:
            status = 'unresolved_baseline_assessment_rejected'
        elif after > before + 1e-12:
            status = 'additional_geometric_opportunity_inspection_unverified'
        elif possible > before + 1e-12:
            status = 'unresolved_within_extra_view_budget'
        else:
            status = 'unresolved_no_new_geometric_opportunity_in_pool'
        reviews.append(dict(deepcopy(request), status=status,
                            baseline_opportunity_fraction=before,
                            combined_opportunity_fraction=after,
                            pool_opportunity_fraction=possible,
                            inspection_resolved=False))
    result = {
        'schema': 'behav3d.frontier_scan_proposal.v1', 'frame': frame, 'units': units,
        'decision': ('hold_baseline_assessment_rejected' if baseline_rejected else
                     'keep_frontier_sweep_with_supplement' if selected else
                     'keep_frontier_sweep'),
        'purpose': 'preserve upper-frontier scanning for mesh/field candidate generation; supplement named inspection needs',
        'baseline_preserved': True, 'baseline_poses': bp,
        'baseline_rejected_indices': baseline_rejected,
        'supplemental_poses': selected,
        'capture_order': ([{'role': 'required_frontier_sweep', 'pose': p} for p in bp]
                          + [{'role': 'inspection_supplement', 'pose': p} for p in selected]),
        'baseline_capture_count': len(bp), 'extra_capture_count': len(selected),
        'total_capture_count': len(bp) + len(selected), 'max_extra_views': max_extra,
        'requests': reviews, 'baseline_order_metrics': order_metrics(bp),
        'append_order_metrics': metrics,
        'objective': 'new opportunity on requested samples only; then fewest additions; then append distance and rotation',
        'frontier_mesh_completeness': 'not established by preserving poses',
        'candidate_planner_effect': 'not evaluated; requires reconstruction from captured evidence',
        'model_limits': [
            'New area is a geometric proxy, not predicted uncertainty reduction.',
            'Repeated views, angular diversity and depth quality are not scored.',
            'Conservative intended-neighbor envelopes can exclude clear recorded-mesh rays.',
            'Costs describe optical-camera position order, not robot travel time.',
            'Camera calibration, full tool clearance, IK and transitions remain unverified.',
        ],
        'execution_authorized': False, 'robot_commands': [],
    }
    return deepcopy(result)
