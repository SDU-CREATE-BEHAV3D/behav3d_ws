"""Offline mesh visibility diagnostics. Clear mesh rays do not prove free space."""
import math
import struct
from pathlib import Path
import numpy as np


def load_binary_stl(path):
    raw = Path(path).read_bytes()
    if len(raw) < 84:
        raise ValueError('truncated STL')
    n = struct.unpack_from('<I', raw, 80)[0]
    if n == 0 or len(raw) != 84 + 50*n:
        raise ValueError('expected nonempty binary STL')
    rows = np.frombuffer(raw, dtype=np.dtype([
        ('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')]),offset=84,count=n)
    triangles = rows['vertices'].astype(float)
    if not np.isfinite(triangles).all():
        raise ValueError('nonfinite mesh')
    return triangles


class MeshRays:
    def __init__(self, triangles):
        t = np.asarray(triangles, dtype=float)
        if t.ndim != 3 or t.shape[1:] != (3,3) or not np.isfinite(t).all():
            raise ValueError('expected finite triangles')
        self.low = t.min(axis=1)
        self.high = t.max(axis=1)
        self.a = t[:,0]
        self.e1 = t[:,1]-t[:,0]
        self.e2 = t[:,2]-t[:,0]

    def first_hit(self, origin, direction):
        """Double-sided Moller–Trumbore, metric distance, no backface culling."""
        o = np.asarray(origin, dtype=float); d = np.asarray(direction,dtype=float)
        if o.shape != (3,) or d.shape != (3,) or not np.isfinite(o).all() or not np.isfinite(d).all() or np.linalg.norm(d)<1e-12:
            raise ValueError('invalid ray')
        d = d/np.linalg.norm(d)
        # Ray/AABB broad phase, including parallel slabs, preserves all triangles.
        near = np.full(len(self.a), -np.inf); far = np.full(len(self.a), np.inf)
        eligible = np.ones(len(self.a), dtype=bool)
        for axis in range(3):
            if abs(d[axis]) < 1e-12:
                eligible &= (o[axis]>=self.low[:,axis]-1e-9)&(o[axis]<=self.high[:,axis]+1e-9)
            else:
                lo=(self.low[:,axis]-o[axis])/d[axis]; hi=(self.high[:,axis]-o[axis])/d[axis]
                near=np.maximum(near,np.minimum(lo,hi)); far=np.minimum(far,np.maximum(lo,hi))
        eligible &= (far>=np.maximum(near,0)-1e-9)
        a=self.a[eligible]; e1=self.e1[eligible]; e2=self.e2[eligible]
        p = np.cross(d,e2); det = np.einsum('ij,ij->i',e1,p)
        valid = np.abs(det)>1e-12
        inv = np.zeros(len(det)); inv[valid] = 1/det[valid]
        tv = o-a
        u = np.einsum('ij,ij->i',tv,p)*inv
        q = np.cross(tv,e1)
        v = q@d*inv
        dist = np.einsum('ij,ij->i',e2,q)*inv
        valid &= (u>=-1e-9)&(v>=-1e-9)&(u+v<=1+1e-9)&(dist>1e-6)
        return float(np.min(dist[valid])) if np.any(valid) else None


def project(point, pose, calibration):
    axes = np.asarray(pose['optical_axes_world'],dtype=float)
    origin = np.asarray(pose['position_m'],dtype=float)
    point = np.asarray(point,dtype=float)
    if axes.shape != (3,3) or point.shape != (3,) or origin.shape != (3,) or not np.isfinite(axes).all() or not np.isfinite(point).all() or not np.isfinite(origin).all():
        raise ValueError('invalid point or camera')
    if not np.allclose(axes@axes.T,np.eye(3),atol=1e-8) or np.linalg.det(axes)<0:
        raise ValueError('camera axes must be right-handed orthonormal')
    for key in ('fx','fy','width','height'):
        if not math.isfinite(calibration[key]) or calibration[key]<=0:
            raise ValueError('invalid calibration')
    if not all(math.isfinite(calibration[key]) for key in ('cx','cy')):
        raise ValueError('invalid principal point')
    x,y,z = axes@(point-origin)
    if z<=1e-9:
        return None
    return [float(calibration['fx']*x/z+calibration['cx']),
            float(calibration['fy']*y/z+calibration['cy']),float(z)]


def classify(point, pose, calibration, rays, tolerance_m):
    if not math.isfinite(tolerance_m) or tolerance_m<=0:
        raise ValueError('positive endpoint tolerance required')
    uvz = project(point,pose,calibration)
    if uvz is None or not (0<=uvz[0]<calibration['width'] and 0<=uvz[1]<calibration['height']):
        return 'outside_image', None
    origin = np.asarray(pose['position_m']); delta=np.asarray(point)-origin
    length = float(np.linalg.norm(delta)); hit=rays.first_hit(origin,delta)
    if hit is not None and hit<length-tolerance_m:
        return 'blocked_in_mesh', hit
    if hit is not None and abs(hit-length)<=tolerance_m:
        return 'endpoint_supported_in_mesh', hit
    return 'unknown_mesh_support', hit


def evaluate(poses, points, calibration, rays, *, tolerance_m=.004, reference_axis=None):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or len(points)==0 or not np.isfinite(points).all():
        raise ValueError('nonempty finite target samples required')
    results=[]; support_count=np.zeros(len(points),dtype=int)
    for pose in poses:
        statuses=[]; hits=[]
        for point in points:
            status,hit=classify(point,pose,calibration,rays,tolerance_m)
            statuses.append(status); hits.append(hit)
        counts={key:statuses.count(key) for key in ('outside_image','blocked_in_mesh','endpoint_supported_in_mesh','unknown_mesh_support')}
        supported=np.array([s=='endpoint_supported_in_mesh' for s in statuses]); support_count+=supported
        origin=np.asarray(pose['position_m']); direction=origin-points.mean(axis=0)
        angle=None
        if reference_axis is not None:
            axis=np.asarray(reference_axis,dtype=float); norm=np.linalg.norm(axis)
            if axis.shape!=(3,) or not np.isfinite(axis).all() or norm<1e-12: raise ValueError('invalid reference axis')
            angle=float(np.degrees(np.arccos(np.clip(np.dot(direction,axis)/np.linalg.norm(direction)/norm,-1,1))))
        results.append({'index':pose['index'],'counts':counts,'statuses':statuses,'first_hit_m':hits,
                        'commanded_axis_view_angle_deg':angle})
    travel=sum(math.dist(a['position_m'],b['position_m']) for a,b in zip(poses,poses[1:]))
    return {'poses':results,'sample_count':len(points),
            'endpoint_supported_in_at_least_one_view':int(np.count_nonzero(support_count)),
            'endpoint_supported_in_at_least_three_views':int(np.count_nonzero(support_count>=3)),
            'per_sample_supported_view_count':support_count.tolist(),
            'camera_position_order_length_m':travel,
            'travel_is_not_robot_path':True,'tolerance_m':tolerance_m}
