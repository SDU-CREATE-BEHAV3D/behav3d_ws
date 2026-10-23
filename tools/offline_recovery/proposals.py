"""Bounded recovery proposals on observed surfaces; never planner-approved targets."""
from copy import deepcopy
import math

import numpy as np

from geometry import bead, union_contains
from viability import evaluate_candidates, seat_targets


def local_offsets(radius_m=.008):
    """Thirteen positions with a declared 8 mm default XY bound."""
    if not math.isfinite(radius_m) or radius_m <= 0: raise ValueError('positive search radius required')
    return [(x, y) for x in np.linspace(-radius_m, radius_m, 5)
            for y in np.linspace(-radius_m, radius_m, 5)
            if np.hypot(x, y) <= radius_m + 1e-12]


def axis_options(axis, tilt_deg=4.):
    axis = np.asarray(axis, dtype=float)
    if axis.shape != (3,) or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-12:
        raise ValueError('finite nonzero axis required')
    if not math.isfinite(tilt_deg) or not 0 <= tilt_deg <= 10: raise ValueError('bounded tilt required')
    axis = axis / np.linalg.norm(axis)
    yield axis
    if tilt_deg:
        for a in (-np.radians(tilt_deg), np.radians(tilt_deg)):
            c, s = np.cos(a), np.sin(a)
            yield np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ axis
            yield np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ axis


def nominal_volume_mm3(record):
    # Rounded-cylinder proxy; this is a cost estimate, never a flow command.
    width, height = record['width_m'] * 1000, record['height_m'] * 1000
    radius, halfheight = width / 2, height / 2
    rounding = min(radius, halfheight)
    core = radius - rounding
    return float(2 * math.pi * radius**2 * (halfheight - rounding)
                 + 2 * math.pi * core**2 * rounding + math.pi**2 * core * rounding**2
                 + 4 * math.pi * rounding**3 / 3)


def make_proposal(pid, strategy, record, scene, lib, models, settings, *,
                  peers=(), reference=None, missing=None, baseline_support=0):
    # Peers reserve space but are not issued as part of this one-bead proposal.
    checks = evaluate_candidates([record, *peers], scene, lib, models, settings)
    check = checks[0]
    reasons = check['rejection_reasons']
    screens = {
        'support': 'fail' if any(r in reasons for r in ('insufficient_nominal_base_support', 'top_not_above_observed_frontier')) else 'pass',
        'spacing': 'fail' if 'nominal_batch_spacing' in reasons else 'pass',
        'tool_access': 'fail' if 'sampled_extruder_approach_conflict' in reasons else 'pass',
        'search_bounds': 'pass',
    }
    if missing is not None:
        benefit = int(union_contains(missing, [record]).sum())
        basis = 'missing_reference_samples_reached_not_volume'
    else:
        benefit = max(0, check['support_rays_within_gap'] - baseline_support)
        basis = 'additional_base_rays_within_experimental_gap_not_stability'
    deviation = (0. if reference is None else
                 float(np.linalg.norm(np.array(record['top_m']) - reference['top_m']) * 1000))
    return {'id': pid, 'strategy': strategy, 'targets': [deepcopy(record)], 'screens': screens,
            'checks': [check], 'metrics': {'benefit': benefit, 'benefit_basis': basis,
                                        'deviation_mm': deviation, 'material_mm3': nominal_volume_mm3(record)},
            'reserved_peer_labels': [r['label'] for r in peers],
            'limits': ['geometric screens use uncalibrated experimental thresholds',
                       'goal, avoidance, process and future continuation not validated'],
            'executable': False, 'robot_commands': []}


def adaptation_proposals(seed, reference, scene, lib, models, settings, *, peers=(),
                         radius_m=.008, tilt_deg=4., max_z_fit_m=.012):
    """Evaluate all placements before ranking; a rejected winner cannot hide another option."""
    base_check = evaluate_candidates([reference, *peers], scene, lib, models, settings)[0]
    result = [make_proposal('unchanged', 'adapt', reference, scene, lib, models, settings,
                            peers=peers, reference=reference,
                            baseline_support=base_check['support_rays_within_gap'])]
    for axis_i, axis in enumerate(axis_options(seed['axis'], tilt_deg)):
        for offset_i, (dx, dy) in enumerate(local_offsets(radius_m)):
            trial = deepcopy(seed)
            trial['axis'] = axis.tolist()
            trial['top_m'][0] += float(dx)
            trial['top_m'][1] += float(dy)
            trial = seat_targets([trial], [trial['label']], scene, lib)[0]
            proposal = make_proposal(f'adapt-{axis_i}-{offset_i}', 'adapt', trial, scene, lib, models,
                                     settings, peers=peers, reference=reference,
                                     baseline_support=base_check['support_rays_within_gap'])
            proposal['search_bounds'] = {'xy_radius_mm': radius_m * 1000,
                                         'axis_rotation_deg': tilt_deg, 'z_fit_limit_mm': max_z_fit_m * 1000}
            if abs(trial.get('surface_fit_z_shift_mm', 0)) > max_z_fit_m * 1000:
                proposal['screens']['search_bounds'] = 'fail'
                proposal['limits'].append('required Z fitting exceeds declared search bound')
            result.append(proposal)
    return result


def surface_fill_proposals(missing, reference, scene, lib, models, settings, *, peers=(), radius_m=.008):
    """First attached bead only. Buried deficits do not create buried deposition targets.

    Subsequent beads require a new observed scene or an explicit deposition model;
    no invented support from a previously proposed bead is accepted here.
    """
    missing = np.asarray(missing, dtype=float)
    if missing.ndim != 2 or missing.shape[1] != 3 or not len(missing) or not np.isfinite(missing).all():
        raise ValueError('finite confirmed-or-hypothesized deficit reference samples required')
    center = np.mean(missing, axis=0)
    result = []
    for i, (dx, dy) in enumerate(local_offsets(radius_m)):
        base = center + [dx, dy, 0]
        gap = scene.vertical_gaps(base[None, :])[0]
        if not np.isfinite(gap): continue
        base[2] -= gap
        trial = deepcopy(reference)
        trial['label'] = f'fill-{i}'
        trial['top_m'] = (base + trial['height_m'] * bead(trial).direction).tolist()
        trial = seat_targets([trial], [trial['label']], scene, lib)[0]
        proposal = make_proposal(f'fill-{i}', 'fill', trial, scene, lib, models, settings,
                                 peers=peers, reference=reference, missing=missing)
        proposal['construction'] = 'One bead attached to the observed upper surface, followed by inspection.'
        proposal['search_bounds'] = {'xy_radius_mm': radius_m * 1000, 'multi_bead_sequence': False}
        result.append(proposal)
    return result
