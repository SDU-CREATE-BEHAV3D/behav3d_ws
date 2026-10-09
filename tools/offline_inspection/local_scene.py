"""Read-only local scene diagnostics and non-executable inspection proposals.

Proximity to a nominal bead is a spatial association, never material identity.
All observed triangles remain in ray tests. Missing geometry stays unknown.
"""
from collections import Counter
from dataclasses import dataclass
import math
import numpy as np
from visibility import MeshRays, project


def xyz(value):
    result = np.asarray(value, dtype=float)
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('finite xyz required')
    return result


def positive(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError('finite positive dimension required')
    return float(value)


@dataclass(frozen=True)
class NominalBead:
    label: str
    top: tuple
    axis: tuple
    width_m: float
    height_m: float

    def __post_init__(self):
        if not isinstance(self.label, str) or not self.label.strip():
            raise ValueError('bead label required')
        xyz(self.top)
        if np.linalg.norm(xyz(self.axis)) < 1e-12:
            raise ValueError('nonzero bead axis required')
        positive(self.width_m); positive(self.height_m)

    @property
    def direction(self):
        z = xyz(self.axis)
        return z / np.linalg.norm(z)

    @property
    def center(self):
        return xyz(self.top) - self.direction * self.height_m / 2

    def sdf(self, points):
        """Same analytic rounded-cylinder convention as nominal_surface."""
        points = np.asarray(points, dtype=float)
        if points.shape[-1:] != (3,) or not np.isfinite(points).all():
            raise ValueError('finite xyz samples required')
        delta = points-self.center
        axial = delta @ self.direction
        radial = np.linalg.norm(delta-np.expand_dims(axial, -1)*self.direction, axis=-1)
        rounding = min(self.width_m, self.height_m)/2
        q = np.stack([radial-self.width_m/2+rounding,
                      np.abs(axial)-self.height_m/2+rounding], axis=-1)
        return np.minimum(q.max(axis=-1), 0)+np.linalg.norm(np.maximum(q, 0), axis=-1)-rounding

    def envelope(self, margin_m):
        # Conservative enclosing sphere; intentionally overestimates rounded shape.
        return self.center, math.hypot(self.width_m/2, self.height_m/2)+margin_m

    def record(self):
        return {'label': self.label, 'top_m': list(self.top), 'axis': self.direction.tolist(),
                'width_m': self.width_m, 'height_m': self.height_m,
                'presence': 'intended_only_identity_unverified'}


def spatial_relation(point, target, neighbors, margin_m):
    positive(margin_m)
    in_target = bool(target.sdf(xyz(point)) <= margin_m)
    near = [b.label for b in neighbors if b.sdf(point) <= margin_m]
    if in_target and near:
        kind = 'overlapping_target_neighbor_regions'
    elif in_target:
        kind = 'target_region_possible_self_obstruction'
    elif near:
        kind = 'neighbor_region_identity_unverified'
    else:
        kind = 'unassigned_observed_scene'
    return {'kind': kind, 'neighbor_labels': near, 'identity_verified': False}


def segment_envelopes(origin, endpoint, neighbors, margin_m):
    """Potential intersections with intended neighbors; no presence assertion."""
    positive(margin_m)
    origin, endpoint = xyz(origin), xyz(endpoint)
    delta = endpoint-origin
    length2 = float(delta@delta)
    if length2 < 1e-20:
        raise ValueError('zero-length view ray')
    hits = []
    for bead in neighbors:
        center, radius = bead.envelope(margin_m)
        t = float(np.clip((center-origin)@delta/length2, 0, 1))
        if np.linalg.norm(origin+t*delta-center) <= radius:
            hits.append(bead.label)
    return hits


class ObservedScene:
    def __init__(self, triangles):
        self.triangles = np.asarray(triangles, dtype=float)
        self.rays = MeshRays(self.triangles)

    def nearest_surface(self, point, mask=None):
        """Exact distance to observed triangles, including interiors and edges."""
        point = xyz(point)
        indices = np.arange(len(self.triangles)) if mask is None else np.flatnonzero(mask)
        if not len(indices):
            return {'distance_m': None, 'point_m': None, 'triangle_index': None}
        triangles = self.triangles[indices]
        # Any vertex is an upper bound. Bounding boxes reject no closer triangle.
        upper2 = float(np.min(np.sum((triangles[:, 0]-point)**2, axis=1)))
        low = self.rays.low[indices]; high = self.rays.high[indices]
        gap = np.maximum(np.maximum(low-point, point-high), 0)
        keep = np.sum(gap*gap, axis=1) <= upper2+1e-15
        indices = indices[keep]; triangles = triangles[keep]
        a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
        candidates = []
        for start, end in ((a, b), (b, c), (c, a)):
            edge = end-start
            denom = np.sum(edge*edge, axis=1)
            t = np.divide(np.sum((point-start)*edge, axis=1), denom,
                          out=np.zeros_like(denom), where=denom>1e-24)
            candidates.append(start+np.clip(t, 0, 1)[:, None]*edge)
        normal = np.cross(b-a, c-a); n2 = np.sum(normal*normal, axis=1)
        projection = point-np.divide(np.sum((point-a)*normal, axis=1), n2,
                                    out=np.zeros_like(n2), where=n2>1e-24)[:, None]*normal
        inside = n2>1e-24
        for start, end in ((a, b), (b, c), (c, a)):
            inside &= np.sum(np.cross(end-start, projection-start)*normal, axis=1) >= -1e-18
        candidates.append(np.where(inside[:, None], projection, np.inf))
        all_points = np.stack(candidates)
        d2 = np.sum((all_points-point)**2, axis=2)
        k, j = np.unravel_index(np.argmin(d2), d2.shape)
        return {'distance_m': float(math.sqrt(d2[k, j])),
                'point_m': all_points[k, j].tolist(), 'triangle_index': int(indices[j])}

    def envelope_clearance(self, pose, envelope):
        if not isinstance(envelope, dict) or not envelope.get('spheres'):
            return {'status': 'unverified_missing_envelope', 'minimum_surface_gap_m': None,
                    'spheres': [], 'full_tool_clearance_verified': False}
        margin = positive(envelope['margin_m'])
        axes = np.asarray(pose['optical_axes_world'], dtype=float)
        if axes.shape != (3, 3) or not np.allclose(axes@axes.T, np.eye(3), atol=1e-8) or np.linalg.det(axes)<0:
            raise ValueError('invalid camera axes')
        entries = []
        for sphere in envelope['spheres']:
            radius = positive(sphere['radius_m'])
            center = xyz(pose['position_m'])+xyz(sphere['center_optical_m'])@axes
            nearest = self.nearest_surface(center)
            gap = None if nearest['distance_m'] is None else nearest['distance_m']-radius-margin
            entries.append({'name': sphere['name'], 'center_m': center.tolist(),
                            'radius_with_margin_m': radius+margin, 'surface_gap_m': gap,
                            'nearest_observed_point_m': nearest['point_m']})
        gaps = [e['surface_gap_m'] for e in entries if e['surface_gap_m'] is not None]
        status = ('intersects_recorded_surface' if gaps and min(gaps)<=0 else
                  'no_recorded_surface_intersection_unknown_space' if gaps else 'unknown_no_observed_geometry')
        return {'status': status, 'minimum_surface_gap_m': min(gaps) if gaps else None,
                'spheres': entries, 'full_tool_clearance_verified': False,
                'basis': envelope['basis'], 'interior_and_unobserved_obstacles': 'unverified'}


def assess_views(poses, surface, calibration, scene, target, neighbors, envelope,
                 *, relation_margin_m=.008, neighbor_margin_m=.008,
                 endpoint_tolerance_m=.004, max_incidence_deg=75):
    """Generate per-sample explanations without deleting any scene geometry."""
    for v in (relation_margin_m, neighbor_margin_m, endpoint_tolerance_m): positive(v)
    if not 0<max_incidence_deg<90:
        raise ValueError('incidence angle must be in (0,90)')
    points = np.asarray(surface['points_m'], dtype=float)
    normals = np.asarray(surface['normals'], dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or normals.shape!=points.shape or not np.isfinite(points).all() or not np.isfinite(normals).all() or not np.allclose(np.linalg.norm(normals,axis=1),1):
        raise ValueError('finite surface and unit normals required')
    results = []
    for pose in poses:
        origin = xyz(pose['position_m'])
        clearance = scene.envelope_clearance(pose, envelope)
        contacts = []
        for sphere in clearance['spheres']:
            for bead in neighbors:
                center, radius = bead.envelope(neighbor_margin_m)
                if np.linalg.norm(np.asarray(sphere['center_m'])-center) <= radius+sphere['radius_with_margin_m']:
                    contacts.append({'envelope_sphere': sphere['name'], 'neighbor_label': bead.label})
        clearance['possible_intended_neighbor_contacts'] = contacts
        outcomes = []; blocker_relations = []; hit_points = []; uncertain_neighbors = []
        for point, normal in zip(points, normals):
            uvz = project(point, pose, calibration)
            ray = point-origin; length = float(np.linalg.norm(ray))
            if length<1e-9: raise ValueError('camera coincides with target')
            cos_angle = float(normal @ (-ray/length))
            relation = None; hit_point = None; nominal_hits = []
            if uvz is None or not (0<=uvz[0]<calibration['width'] and 0<=uvz[1]<calibration['height']):
                outcome = 'outside_image'
            elif cos_angle<=0:
                outcome = 'back_facing_nominal'
            elif cos_angle<math.cos(math.radians(max_incidence_deg)):
                outcome = 'grazing_nominal'
            else:
                hit = scene.rays.first_hit(origin, ray)
                if hit is not None and hit<length-endpoint_tolerance_m:
                    hit_point = (origin+ray/length*hit).tolist()
                    relation = spatial_relation(hit_point, target, neighbors, relation_margin_m)
                    outcome = relation['kind']
                else:
                    nominal_hits = segment_envelopes(origin, point, neighbors, neighbor_margin_m)
                    outcome = ('possible_intended_neighbor_obstruction' if nominal_hits else
                               'inspection_opportunity_unknown_space')
            outcomes.append(outcome); blocker_relations.append(relation)
            hit_points.append(hit_point); uncertain_neighbors.append(nominal_hits)
        results.append({'index': pose['index'], 'outcomes': outcomes,
                        'counts': dict(Counter(outcomes)), 'blocker_relations': blocker_relations,
                        'blocker_points_m': hit_points, 'possible_neighbor_labels': uncertain_neighbors,
                        'clearance': clearance,
                        'eligible_for_offline_selection': clearance['status']=='no_recorded_surface_intersection_unknown_space' and not contacts})
    return results


def coverage(rows, surface):
    n = len(surface['points_m'])
    counts = np.zeros(n, dtype=int)
    for row in rows:
        if row['eligible_for_offline_selection']:
            counts += np.array([s=='inspection_opportunity_unknown_space' for s in row['outcomes']], dtype=int)
    regions = np.asarray(surface['regions']); weights = np.asarray(surface['area_weights_m2'])
    output = {}
    for region in ('top', 'side'):
        mask = regions==region; total = float(weights[mask].sum())
        output[region] = {'sample_count': int(mask.sum()),
                          'opportunity_area_fraction_once': float(weights[mask & (counts>0)].sum()/total) if total else None,
                          'opportunity_area_fraction_three_views': float(weights[mask & (counts>=3)].sum()/total) if total else None}
    return {'regions': output, 'sample_view_counts': counts.tolist()}


def select_views(poses, rows, surface, budget):
    """Greedy side coverage, then side repeat views, then top; no LLM ranking.

    Repeated opportunity capped at three. Zero-benefit poses are never added.
    This optimizes a requested inspection surface, not measured information gain.
    """
    if type(budget) is not int or budget<1 or len(poses)!=len(rows):
        raise ValueError('invalid budget or pose/assessment linkage')
    if any(p['index']!=r['index'] for p,r in zip(poses,rows)):
        raise ValueError('pose/assessment indices differ')
    weights = np.asarray(surface['area_weights_m2']); regions = np.asarray(surface['regions'])
    counts = np.zeros(len(weights), dtype=int); chosen = []; decisions = []
    remaining = [i for i,r in enumerate(rows) if r['eligible_for_offline_selection']]
    while remaining and len(chosen)<budget:
        scores = []
        for i in remaining:
            available = np.array([s=='inspection_opportunity_unknown_space' for s in rows[i]['outcomes']])
            new = available & (counts==0); repeat = available & (counts<3)
            scores.append((float(weights[new & (regions=='side')].sum()),
                           float(weights[repeat & (regions=='side')].sum()),
                           float(weights[repeat & (regions=='top')].sum()), -i))
        best = max(range(len(remaining)), key=lambda k: scores[k]); score = scores[best]
        if max(score[:3])<=0: break
        i = remaining.pop(best); chosen.append(i)
        counts += np.array([s=='inspection_opportunity_unknown_space' for s in rows[i]['outcomes']], dtype=int)
        decisions.append({'pool_index': poses[i]['index'], 'new_side_area_m2': score[0],
                          'side_repeat_benefit_m2': score[1], 'top_benefit_m2': score[2]})
    return {'selected_pool_indices': [poses[i]['index'] for i in chosen],
            'poses': [poses[i] for i in chosen], 'assessments': [rows[i] for i in chosen],
            'decisions': decisions, 'coverage': coverage([rows[i] for i in chosen], surface),
            'status': 'non_executable_inspection_proposal' if chosen else 'hold_no_supported_proposal',
            'execution_authorized': False, 'robot_commands': [],
            'order_kind': 'selection_priority_only_not_robot_path'}
