"""Deterministic camera-optical poses only; no ROS, IK or hardware imports."""
import math
from .review import ContractError


def vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ContractError('expected xyz triplet')
    if any(type(x) not in (float, int) or not math.isfinite(x) for x in value):
        raise ContractError('coordinates must be finite numbers')
    return tuple(float(x) for x in value)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(v):
    v = vector(v)
    n = math.sqrt(dot(v, v))
    if n < 1e-12:
        raise ContractError('direction must be nonzero')
    return tuple(x/n for x in v)


def basis(z):
    z = unit(z)
    guess = min(((1., 0., 0.), (0., 1., 0.), (0., 0., 1.)), key=lambda a: abs(dot(a, z)))
    x = unit(tuple(a-dot(guess, z)*b for a, b in zip(guess, z)))
    return x, cross(z, x), z


def positive(value):
    if type(value) not in (float, int) or not math.isfinite(value) or value <= 0:
        raise ContractError('dimensions must be finite and positive')
    return float(value)


def count(value, minimum):
    if type(value) is not int or not minimum <= value <= 100:
        raise ContractError('sample count outside supported range')
    return value


def pose(position, axes, frame, index):
    return {'index': index, 'position_m': list(position),
            'optical_axes_world': [list(a) for a in axes], 'frame': frame,
            'pose_kind': 'camera_optical', 'executable': False}


def generate_patterns(*, focus_m, frame, units, distance_m, width_m, height_m,
                      nx, ny, samples, cap_deg, outward_axis):
    """Grid uses world XY and fixed downward +Z; cap axis is explicit.

    optical_axes_world stores world vectors for local X,Y,Z (matrix columns).
    These are camera viewpoints, not flange targets or validated trajectories.
    """
    if not isinstance(frame, str) or not frame.strip() or units != 'm':
        raise ContractError('explicit frame and metres required')
    c = vector(focus_m)
    d, w, h = map(positive, (distance_m, width_m, height_m))
    nx, ny, samples = count(nx, 2), count(ny, 2), count(samples, 1)
    if type(cap_deg) not in (int, float) or not math.isfinite(cap_deg) or not 0 < cap_deg <= 90:
        raise ContractError('cap angle must be in (0,90] degrees')
    bx, by, bz = basis(outward_axis)
    grid = []
    for j in range(ny):
        for i in (range(nx) if j % 2 == 0 else reversed(range(nx))):
            p = (c[0]-w/2+i*w/(nx-1), c[1]-h/2+j*h/(ny-1), c[2]+d)
            grid.append(pose(p, ((1.,0.,0.),(0.,-1.,0.),(0.,0.,-1.)), frame, len(grid)))
    cap = []
    cos_cap = math.cos(math.radians(cap_deg))
    golden = (1+math.sqrt(5))/2
    for i in range(samples):
        z = cos_cap+(1-cos_cap)*(i+.5)/samples
        r = math.sqrt(max(0., 1-z*z))
        a = 2*math.pi*(i+.5)/golden
        local = (r*math.cos(a), r*math.sin(a), z)
        direction = tuple(sum(local[k]*axis[t] for k, axis in enumerate((bx,by,bz))) for t in range(3))
        p = tuple(c[t]+d*direction[t] for t in range(3))
        cap.append(pose(p, basis(tuple(-x for x in direction)), frame, i))
    # Same position-only nearest-neighbor ordering concept as the existing cap.
    ordered = [cap.pop(0)]
    while cap:
        previous = ordered[-1]['position_m']
        idx = min(range(len(cap)), key=lambda k: sum((a-b)**2 for a,b in zip(cap[k]['position_m'], previous)))
        ordered.append(cap.pop(idx))
    for i, p in enumerate(ordered):
        p['index'] = i
    return {'schema': 'behav3d.focused_inspection_poses.v1', 'frame': frame, 'units': units,
            'focus_m': list(c), 'grid': grid, 'cap': ordered, 'executable': False,
            'checks': {k: 'unverified' for k in ('visibility','depth_quality','ik','collision','transitions')},
            'robot_commands': []}
