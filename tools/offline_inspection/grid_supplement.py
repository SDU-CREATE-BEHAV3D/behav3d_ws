"""Small non-executable side-view supplements to a fixed grid.

Uses cached geometric assessments, not measured visibility. Exhaustive search is
bounded to 32 candidates and four additions. Paths are Euclidean camera-position
orders only, with orientation change reported separately; no robot path is solved.
"""
from itertools import combinations, permutations
import math
import numpy as np
from local_scene import coverage

OPPORTUNITY = 'inspection_opportunity_unknown_space'


def _validated_group(group, sample_count, frame):
    poses, rows = group['poses'], group['assessments']
    if len(poses) != len(rows) or len({p['index'] for p in poses}) != len(poses):
        raise ValueError('duplicate or mismatched pose identities')
    for p, r in zip(poses, rows):
        if type(p['index']) is not int or p['index'] < 0 or p['index'] != r['index']:
            raise ValueError('pose and assessment indices differ')
        if p.get('frame') != frame:
            raise ValueError('camera frame mismatch')
        position = np.asarray(p['position_m'], dtype=float)
        axes = np.asarray(p['optical_axes_world'], dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('finite camera position required')
        if axes.shape != (3,3) or not np.isfinite(axes).all() or not np.allclose(axes@axes.T, np.eye(3), atol=1e-8) or not math.isclose(np.linalg.det(axes), 1, abs_tol=1e-8):
            raise ValueError('right-handed orthonormal axes required')
        if len(r['outcomes']) != sample_count or type(r['eligible_for_offline_selection']) is not bool:
            raise ValueError('invalid assessment samples')
    return poses, rows


def rotation_change_deg(a, b):
    cosine = (np.trace(np.asarray(a['optical_axes_world'])@np.asarray(b['optical_axes_world']).T)-1)/2
    return math.degrees(math.acos(float(np.clip(cosine, -1, 1))))


def order_metrics(poses):
    return {'position_order_length_m': sum(math.dist(a['position_m'], b['position_m']) for a,b in zip(poses,poses[1:])),
            'summed_orientation_change_deg': sum(rotation_change_deg(a,b) for a,b in zip(poses,poses[1:]))}


def shortest_append_order(grid_end, poses):
    """Exact open position order, fixed grid endpoint, no return assumption."""
    if len(poses)>4:
        raise ValueError('append ordering supports at most four poses')
    if not poses:
        return [], order_metrics([])
    best = None
    for order in permutations(poses):
        metrics = order_metrics([grid_end, *order])
        key = (metrics['position_order_length_m'], metrics['summed_orientation_change_deg'], tuple(p['index'] for p in order))
        if best is None or key < best[0]: best = (key, list(order), metrics)
    return best[1], best[2]


def plan_supplement(grid, pool, surface, *, frame, units, max_extra=4):
    if not isinstance(frame,str) or not frame.strip() or units!='m':
        raise ValueError('explicit frame and metre units required')
    if type(max_extra) is not int or not 0<=max_extra<=4:
        raise ValueError('extra budget must be between zero and four')
    n = len(surface['points_m']); weights=np.asarray(surface['area_weights_m2'],dtype=float)
    regions=np.asarray(surface['regions'])
    if weights.shape!=(n,) or regions.shape!=(n,) or not np.isfinite(weights).all() or np.any(weights<=0):
        raise ValueError('positive finite sample area weights required')
    side=regions=='side'; side_area=float(weights[side].sum())
    if not n or side_area<=0: raise ValueError('side region required')
    gp, gr = _validated_group(grid,n,frame); pp, pr = _validated_group(pool,n,frame)
    if not gp or len(pp)>32: raise ValueError('nonempty grid and bounded pool required')
    baseline=coverage(gr,surface)
    result={'schema':'behav3d.grid_supplement.v1','frame':frame,'units':units,
            'baseline_kind':'existing proposed cropped grid; not previously measured coverage',
            'objective':'maximize newly inspectable nominal side area; then fewest additions; then shortest append order; then rotation',
            'baseline_grid_indices':[p['index'] for p in gp], 'baseline_coverage':baseline,
            'baseline_order_metrics':order_metrics(gp), 'max_extra_views':max_extra,
            'curves':[], 'execution_authorized':False,'robot_commands':[],
            'cost_scope':'camera-position polyline and rotation from last grid pose; no approach/return, IK, collision-checked transit, or duration',
            'unknown_space_policy':'all selected opportunities retain unknown space and unverified physical execution'}
    if not all(r['eligible_for_offline_selection'] for r in gr):
        return dict(result,status='hold_baseline_grid_has_rejected_views',recommended=None)
    baseline_counts=np.asarray(baseline['sample_view_counts'])
    masks={}
    for i,row in enumerate(pr):
        if row['eligible_for_offline_selection']:
            mask=sum(1<<j for j,o in enumerate(row['outcomes']) if side[j] and baseline_counts[j]==0 and o==OPPORTUNITY)
            if mask: masks[i]=mask
    result['candidate_views_with_new_side_opportunity']=len(masks)
    score_cache={0:0.}
    def area(mask):
        if mask not in score_cache:
            score_cache[mask]=float(sum(w for j,w in enumerate(weights) if (mask>>j)&1))
        return score_cache[mask]
    # Each item is (score, combo, union, order, metrics). Baseline is always valid.
    empty_metrics=order_metrics([])
    best_by_budget=[((0.,0,0.,0.,()),(),0,[],empty_metrics)]*(max_extra+1)
    for size in range(1,max_extra+1):
        for combo in combinations(masks,size):
            union=0
            for i in combo: union |= masks[i]
            # A supplemental pose must contribute unique new side area to this set.
            redundant=False
            for i in combo:
                others=0
                for j in combo:
                    if j!=i: others |= masks[j]
                if masks[i] & ~others == 0:
                    redundant=True; break
            if redundant: continue
            gain=area(union)
            order, metrics=shortest_append_order(gp[-1],[pp[i] for i in combo])
            key=(round(gain,15),-size,-metrics['position_order_length_m'],
                 -metrics['summed_orientation_change_deg'],tuple(-p['index'] for p in order))
            item=(key,combo,union,order,metrics)
            for budget in range(size,max_extra+1):
                if key>best_by_budget[budget][0]: best_by_budget[budget]=item
    row_by_id={r['index']:r for r in pr}
    for budget,(_,combo,union,ordered,metrics) in enumerate(best_by_budget):
        rows=[row_by_id[p['index']] for p in ordered]
        cumulative=baseline_counts.copy(); decisions=[]
        for p,row in zip(ordered,rows):
            available=np.array([o==OPPORTUNITY for o in row['outcomes']])
            added=side & (cumulative==0) & available
            decisions.append({'pool_index':p['index'],'added_side_area_fraction_in_this_order':float(weights[added].sum()/side_area)})
            cumulative+=available.astype(int)
        combined=coverage([*gr,*rows],surface)
        result['curves'].append({'budget':budget,'extra_views':len(ordered),
            'total_captures':len(gp)+len(ordered), 'selected_pool_indices':[p['index'] for p in ordered],
            'poses':ordered,'assessments':rows,'combined_coverage':combined,
            'new_side_area_fraction':area(union)/side_area,
            'new_side_sample_indices':[j for j in range(n) if (union>>j)&1],
            'append_order_metrics':metrics,
            'total_position_order_length_m':result['baseline_order_metrics']['position_order_length_m']+metrics['position_order_length_m'],
            'decisions':decisions,
            'stop_reason':'no_further_new_side_area_in_pool' if len(ordered)<budget else 'budget_limit'})
    # Same best achievable area: choose smallest view count, then shortest order.
    chosen=result['curves'][-1]
    result['recommended']=chosen
    result['status']='non_executable_grid_supplement' if chosen['extra_views'] else 'no_supplement_adds_new_side_area'
    result['selection_note']='minimal additions achieving best new side area within tested pool/budget; no calibrated cost/benefit threshold'
    result['view_count_is_not_independent_measurement_count']=True
    return result
