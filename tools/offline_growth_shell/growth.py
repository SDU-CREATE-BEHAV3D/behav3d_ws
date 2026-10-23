"""Bounded shell revisions for offline experiments; no execution interfaces."""
from dataclasses import dataclass

import numpy as np


def smoothstep(t):
    t = np.clip(np.asarray(t, float), 0., 1.)
    return t*t*(3.-2.*t)


def fit_shell_offsets(expected_top, expected_middle, observed_middle, *, minimum_span_m=.02):
    """Fit horizontal shell displacement versus planned cap height.

    These are corresponded cap-derived middle proxies, not measured volume
    centres. Height is from planned tops so a query at the printing frontier has
    the same convention. Z discrepancies remain diagnostics, never silently
    interpreted as an instruction to move the remaining design vertically.
    """
    top, expected, observed = map(lambda p: np.asarray(p, float), (expected_top, expected_middle, observed_middle))
    if top.ndim != 2 or top.shape[1:] != (3,) or expected.shape != top.shape or observed.shape != top.shape:
        raise ValueError('corresponded N x 3 arrays required')
    if len(top) < 4 or not all(np.isfinite(p).all() for p in (top, expected, observed)):
        raise ValueError('at least four finite shell pairs required')
    span = np.ptp(top[:, 2])
    if span < minimum_span_m:
        raise ValueError('insufficient observed height span for a regional fit')
    z_ref = float(top[:, 2].mean())
    design = np.column_stack([np.ones(len(top)), top[:, 2]-z_ref])
    delta = observed-expected
    coefficients = np.linalg.lstsq(design, delta[:, :2], rcond=None)[0]
    residual = delta[:, :2] - design @ coefficients
    return {'z_reference_m': z_ref, 'intercept_xy_m': coefficients[0].tolist(),
            'slope_xy_per_m': coefficients[1].tolist(), 'pair_count': len(top),
            'fit_height_range_m': [float(top[:, 2].min()), float(top[:, 2].max())],
            'horizontal_residual_rms_mm': float(np.sqrt(np.mean(np.sum(residual**2, axis=1))) * 1000),
            'vertical_residual_range_mm': [float(delta[:, 2].min()*1000), float(delta[:, 2].max()*1000)],
            'lateral_uniformity': 'same fitted displacement at a given height across this wall patch; experimental assumption'}


def predict_offset(fit, z):
    z = np.asarray(z, float)
    return np.asarray(fit['intercept_xy_m']) + (z[..., None]-fit['z_reference_m']) * np.asarray(fit['slope_xy_per_m'])


@dataclass
class Frontier:
    y: np.ndarray
    z: np.ndarray

    @classmethod
    def from_contour(cls, contour):
        contour = np.asarray(contour, float)
        keep = (contour[:, 0] > .19) & (contour[:, 2] > -.075)
        points = contour[keep]
        if len(points) < 4:
            raise ValueError('recorded right-wall frontier is unavailable')
        # Median per millimetre bin keeps this case-specific chart from using
        # duplicate contour vertices as independent anchors.
        bins = np.round(points[:, 1]/.001).astype(int)
        y, z = [], []
        for b in np.unique(bins):
            p = points[bins == b]
            y.append(float(np.median(p[:, 1])))
            z.append(float(np.median(p[:, 2])))
        return cls(np.asarray(y), np.asarray(z))

    def height(self, y):
        return np.interp(y, self.y, self.z)

    def weight(self, points):
        points = np.asarray(points, float)
        # Protect the opposite wall and taper the lateral edges of the selected
        # patch. Membership is fixed for this recorded experiment, not inferred
        # from nearest points on unrelated walls.
        wx = smoothstep((points[:, 0]-.19)/.02)
        wy = smoothstep((points[:, 1]-(self.y.min()-.008))/.008)
        wy *= smoothstep(((self.y.max()+.008)-points[:, 1])/.008)
        return wx*wy


def revise_shell(vertices, frontier, fit, *, return_height_m, mode='return', maximum_offset_m=.012):
    """Preserve topology/Z and smoothly taper a fitted XY offset back to zero.

    A below-frontier collar connects the numerical field to the proposed anchor;
    this changes a planning reference, never the recorded printed mesh. Above the
    frontier, offset magnitude can only decrease. Kept scalar values travel with
    vertex identity; production gradients are reevaluated on this revised mesh.
    """
    vertices = np.asarray(vertices, float)
    if vertices.ndim != 2 or vertices.shape[1:] != (3,) or not np.isfinite(vertices).all():
        raise ValueError('finite vertices required')
    if mode not in {'return', 'follow', 'original'}:
        raise ValueError('unsupported shell revision')
    if mode == 'return' and (return_height_m is None or not np.isfinite(return_height_m) or return_height_m <= 0):
        raise ValueError('positive finite return height required')
    if not np.isfinite(maximum_offset_m) or maximum_offset_m <= 0:
        raise ValueError('positive offset bound required')
    anchor_z = frontier.height(vertices[:, 1])
    delta = predict_offset(fit, anchor_z)
    weight = frontier.weight(vertices)
    collar = smoothstep((vertices[:, 2]+.075)/np.maximum(anchor_z+.075, 1e-9))
    if mode == 'original':
        scale = np.zeros(len(vertices))
    elif mode == 'follow':
        scale = collar
    else:
        scale = np.where(vertices[:, 2] <= anchor_z, collar,
                         1-smoothstep((vertices[:, 2]-anchor_z)/return_height_m))
    offset = delta * (weight*scale)[:, None]
    if np.linalg.norm(offset, axis=1).max(initial=0.) > maximum_offset_m + 1e-12:
        raise ValueError('estimated displacement exceeds the declared goal-revision bound')
    result = vertices.copy()
    result[:, :2] += offset
    return result


def mesh_quality(original, revised, faces):
    a, b = np.asarray(original)[faces], np.asarray(revised)[faces]
    na = np.cross(a[:, 1]-a[:, 0], a[:, 2]-a[:, 0])
    nb = np.cross(b[:, 1]-b[:, 0], b[:, 2]-b[:, 0])
    aa, ab = np.linalg.norm(na, axis=1), np.linalg.norm(nb, axis=1)
    valid = aa > 1e-14
    if not valid.all():
        raise ValueError('source field contains degenerate faces')
    cosine = np.sum(na*nb, axis=1)/np.maximum(aa*ab, 1e-30)
    return {'degenerate_faces': int(np.sum(ab <= 1e-14)),
            'reversed_faces_relative_to_source': int(np.sum(cosine <= 0)),
            'minimum_area_ratio': float(np.min(ab/aa)),
            'maximum_face_rotation_deg': float(np.degrees(np.arccos(np.clip(cosine, -1, 1))).max()),
            'self_intersections_checked': False}


def return_diagnostics(vertices, frontier, fit, height):
    """Measure return envelope at sampled goal columns, without extending the goal."""
    vertices = np.asarray(vertices)
    columns = []
    for y in np.linspace(frontier.y.min(), frontier.y.max(), 17):
        slice_mask = (vertices[:, 0] > .19) & (np.abs(vertices[:, 1]-y) <= .006)
        if not slice_mask.any():
            continue
        anchor_z = float(frontier.height(y))
        roof_z = float(vertices[slice_mask, 2].max())
        available = max(0., roof_z-anchor_z)
        offset = predict_offset(fit, anchor_z)
        progress = 0. if height is None else float(smoothstep(available/height))
        columns.append({'y_m': float(y), 'anchor_z_m': anchor_z, 'roof_z_m': roof_z,
                        'available_height_mm': available*1000,
                        'anchor_offset_mm': float(np.linalg.norm(offset)*1000),
                        'residual_at_roof_mm': float(np.linalg.norm(offset)*(1-progress)*1000),
                        'return_completed': height is not None and available >= height})
    return {'sampled_columns': columns, 'complete_columns': sum(c['return_completed'] for c in columns),
            'column_count': len(columns), 'maximum_residual_at_roof_mm': max(c['residual_at_roof_mm'] for c in columns),
            'minimum_available_height_mm': min(c['available_height_mm'] for c in columns),
            'basis': '17 sampled Y columns with 6 mm field-vertex neighborhoods; not an exact boundary proof'}
