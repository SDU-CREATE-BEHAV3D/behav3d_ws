"""Spatial comparisons of offline candidate proposals, never target identities."""
import numpy as np
from scipy.optimize import linear_sum_assignment


def minimum_clearance(candidate):
    values = [m['minimum_sampled_clearance_mm'] for m in candidate['check']['tool_models'].values()]
    return min(values) if values else None


def candidate_delta(new, old):
    """Signed new-minus-old measurements; support is the declared proxy only."""
    a, b = new['target'], old['target']
    delta = (np.asarray(a['top_m']) - b['top_m'])*1000
    axis_a = np.asarray(a['axis'], dtype=float)
    axis_b = np.asarray(b['axis'], dtype=float)
    axis_a /= np.linalg.norm(axis_a)
    axis_b /= np.linalg.norm(axis_b)
    rotation = np.degrees(np.arctan2(np.linalg.norm(np.cross(axis_a, axis_b)), np.dot(axis_a, axis_b)))
    new_clear, old_clear = minimum_clearance(new), minimum_clearance(old)
    new_spacing, old_spacing = (c['check']['minimum_batch_spacing_mm'] for c in [new, old])
    return {'new_id': new['id'], 'old_id': old['id'], 'top_delta_xyz_mm': delta.tolist(),
            'top_shift_mm': float(np.linalg.norm(delta)), 'axis_change_deg': float(rotation),
            'support_ray_delta': new['check']['support_rays_within_gap']-old['check']['support_rays_within_gap'],
            'sampled_tool_clearance_delta_mm': None if new_clear is None or old_clear is None else new_clear-old_clear,
            'batch_spacing_delta_mm': None if new_spacing is None or old_spacing is None else new_spacing-old_spacing}


def compare_batches(new, old, maximum_pair_distance_mm=36.):
    """Maximize gated one-to-one matches, then minimize total endpoint distance.

    Dummy columns let targets remain unmatched instead of forcing comparisons.
    The gate is an association convention, not a permitted robot movement.
    """
    if not np.isfinite(maximum_pair_distance_mm) or maximum_pair_distance_mm <= 0:
        raise ValueError('positive finite comparison distance required')
    if len({c['id'] for c in new}) != len(new) or len({c['id'] for c in old}) != len(old):
        raise ValueError('candidate IDs must be unique within each batch')
    pairs, used_new, used_old = [], set(), set()
    if new and old:
        a = np.asarray([c['target']['top_m'] for c in new])
        b = np.asarray([c['target']['top_m'] for c in old])
        if not np.isfinite(a).all() or not np.isfinite(b).all():
            raise ValueError('finite candidate endpoints required')
        distances = np.linalg.norm(a[:, None, :]-b[None, :, :], axis=2)*1000
        penalty = (len(new)+1)*maximum_pair_distance_mm
        costs = np.full((len(new), len(old)+len(new)), penalty)
        costs[:, :len(old)] = np.where(distances <= maximum_pair_distance_mm, distances, penalty*2)
        rows, cols = linear_sum_assignment(costs)
        for i, j in zip(rows, cols):
            if j >= len(old) or distances[i, j] > maximum_pair_distance_mm:
                continue
            pairs.append(candidate_delta(new[i], old[j]))
            used_new.add(int(i))
            used_old.add(int(j))
    return {'method': 'maximum-cardinality one-to-one spatial association, then minimum total endpoint distance',
            'identity_confirmed': False, 'maximum_pair_distance_mm': maximum_pair_distance_mm,
            'new_count': len(new), 'old_count': len(old), 'pairs': pairs,
            'unmatched_new_ids': [c['id'] for i,c in enumerate(new) if i not in used_new],
            'unmatched_old_ids': [c['id'] for i,c in enumerate(old) if i not in used_old]}
