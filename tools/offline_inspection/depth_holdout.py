"""Raw-depth ablation helpers. No full-scan mesh or fitted patch selects a view."""
import math
import numpy as np


def corrected_depth(raw, scale, bias):
    """Match the recorded DDS integer depth-bias convention; zero stays unknown."""
    raw = np.asarray(raw)
    if raw.dtype != np.uint16 or not np.isfinite([scale, bias]).all() or scale <= 0:
        raise ValueError('uint16 raw depth and finite positive scale required')
    adjusted = np.clip(raw.astype(np.int32) + round(bias * scale), 1, 65535) / scale
    return np.where(raw > 0, adjusted, np.nan)


def crop_mask(shape, meta):
    rows, cols = shape
    if type(meta['center_crop_depth']) is not bool:
        raise ValueError('explicit crop mode required')
    w, h = meta['center_crop_width'], meta['center_crop_height']
    if any(type(x) is not int or x <= 0 for x in (w, h)):
        raise ValueError('positive integer crop dimensions required')
    out = np.ones(shape, dtype=bool)
    if meta['center_crop_depth']:
        w, h = min(w, cols), min(h, rows)
        x, y = (cols - w) // 2, (rows - h) // 2
        out[:] = False
        out[y:y+h, x:x+w] = True
    return out


def backproject_region(raw, pose, calibration, meta, top, chart, width_mm, height_mm, padding_mm):
    if raw.shape != (calibration['height'], calibration['width']):
        raise ValueError('depth dimensions differ from intrinsics')
    depth = corrected_depth(raw, meta['depth_scale'], meta['depth_bias_m'])
    good = crop_mask(raw.shape, meta) & np.isfinite(depth) & (depth <= meta['depth_max'])
    y, x = np.nonzero(good)
    z = depth[good]
    camera = np.column_stack(((x-calibration['cx'])/calibration['fx']*z,
                              (y-calibration['cy'])/calibration['fy']*z, z))
    world = camera @ np.asarray(pose['optical_axes_world']) + np.asarray(pose['position_m'])
    local = (world - np.asarray(top)) @ np.asarray(chart) * 1000
    inside = ((np.linalg.norm(local[:, :2], axis=1) <= width_mm/2 + padding_mm)
              & (local[:, 2] >= -height_mm-padding_mm) & (local[:, 2] <= padding_mm))
    return world[inside], {'valid_used_pixels': int(good.sum()), 'used_pixels': int(crop_mask(raw.shape, meta).sum()),
                           'raw_points_in_candidate_search': int(inside.sum())}


def four_folds(indices):
    if len(indices) < 8 or len(set(indices)) != len(indices):
        raise ValueError('at least eight unique recorded captures required')
    return [{'fold': k, 'training': [v for j, v in enumerate(indices) if j % 4 != k],
             'held_out': [v for j, v in enumerate(indices) if j % 4 == k]} for k in range(4)]


def subset_cloud(points_by_view, indices):
    if not indices or len(set(indices)) != len(indices) or not set(indices) <= set(points_by_view):
        raise ValueError('unique known subset views required')
    return (np.vstack([points_by_view[i] for i in indices]),
            np.concatenate([np.full(len(points_by_view[i]), i, dtype=int) for i in indices]))


def order_heldout_views(poses, training, held_out, top, calibration, meta):
    """Pre-depth ranking: new viewing direction, then shortest camera displacement.

    Only pose metadata and commanded geometry are used. The true held-out depth,
    full-scan fitted cap and reconstructed mesh are deliberately absent.
    """
    if set(training) & set(held_out) or not training:
        raise ValueError('training and held-out views must be disjoint')
    by_id = {p['index']: p for p in poses}
    if len(by_id) != len(poses) or not set(training + held_out) <= set(by_id):
        raise ValueError('unique known capture identities required')
    top = np.asarray(top)
    directions = {}
    for index, pose in by_id.items():
        vector = np.asarray(pose['position_m']) - top
        if np.linalg.norm(vector) < 1e-9:
            raise ValueError('camera coincides with target')
        directions[index] = vector / np.linalg.norm(vector)
    used = crop_mask((calibration['height'], calibration['width']), meta)
    available = []
    for index in held_out:
        p = by_id[index]
        x, y, z = np.asarray(p['optical_axes_world']) @ (top - p['position_m'])
        if not 0 < z <= meta['depth_max']:
            continue
        u, v = int(round(calibration['fx']*x/z + calibration['cx'])), int(round(calibration['fy']*y/z + calibration['cy']))
        if 0 <= v < used.shape[0] and 0 <= u < used.shape[1] and used[v, u]:
            available.append(index)
    selected = []
    while available:
        previous = by_id[(training + selected)[-1]]['position_m']
        def key(index):
            angle = min(math.acos(float(np.clip(directions[index] @ directions[j], -1, 1)))
                        for j in training + selected)
            return (angle, -math.dist(previous, by_id[index]['position_m']), -index)
        best = max(available, key=key)
        selected.append(best)
        available.remove(best)
    return selected
