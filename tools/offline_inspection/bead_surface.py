"""Nominal rounded-cylinder inspection targets, never measured material."""
import math
import numpy as np


def nominal_surface(top, axis, width_m, height_m):
    top=np.asarray(top,dtype=float);z=np.asarray(axis,dtype=float)
    if top.shape!=(3,) or z.shape!=(3,) or not np.isfinite(top).all() or not np.isfinite(z).all() or np.linalg.norm(z)<1e-12:
        raise ValueError('finite top and nonzero axis required')
    if not all(math.isfinite(v) and v>0 for v in (width_m,height_m)):
        raise ValueError('positive dimensions required')
    z=z/np.linalg.norm(z);guess=np.eye(3)[np.argmin(abs(z))]
    x=guess-np.dot(guess,z)*z;x/=np.linalg.norm(x);y=np.cross(z,x)
    r=width_m/2;rounding=min(width_m,height_m)/2;core_r=r-rounding
    core_h=height_m/2-rounding;center=top-z*height_m/2
    points=[];normals=[];regions=[];weights=[]
    def append(radial,axial,normal_r,normal_z,region,weight,theta):
        direction=x*math.cos(theta)+y*math.sin(theta)
        points.append((center+direction*radial+z*axial).tolist())
        normals.append((direction*normal_r+z*normal_z).tolist());regions.append(region);weights.append(weight)
    # Flat top disk: equal-area rings, avoid duplicate pole points.
    if core_r>1e-12:
        for ring in range(2):
            radius=core_r*math.sqrt((ring+.5)/2)
            for i in range(12):append(radius,height_m/2,0,1,'top',math.pi*core_r**2/24,2*math.pi*i/12)
    # Rounded side bands. Midpoint quadrature weights track surface area.
    for j in range(6):
        phi=-math.pi/2+(j+.5)*math.pi/6
        radius=core_r+rounding*math.cos(phi)
        axial=(core_h if phi>=0 else -core_h)+rounding*math.sin(phi)
        for i in range(12):
            append(radius,axial,math.cos(phi),math.sin(phi),'side',
                   radius*rounding*(math.pi/6)*(2*math.pi/12),2*math.pi*i/12)
    if core_h>1e-12:
        for i in range(12):append(r,0,1,0,'side',r*2*core_h*(2*math.pi/12),2*math.pi*i/12)
    return {'points_m':points,'normals':normals,'regions':regions,'area_weights_m2':weights,
            'basis':'nominal DDS rounded cylinder, issued top anchor; not observed material',
            'bottom_excluded':True,'width_m':width_m,'height_m':height_m}


def surface_views(poses,surface,calibration,rays,evaluate):
    result=evaluate(poses,surface['points_m'],calibration,rays)
    points=np.asarray(surface['points_m'],dtype=float);normals=np.asarray(surface['normals'],dtype=float);weights=np.asarray(surface['area_weights_m2'],dtype=float)
    per=[]
    for pose,row in zip(poses,result['poses']):
        directions=np.asarray(pose['position_m'])-points;directions/=np.linalg.norm(directions,axis=1)[:,None]
        facing=np.einsum('ij,ij->i',directions,normals)
        angles=np.degrees(np.arccos(np.clip(facing,-1,1)))
        outcomes=[]
        for status,cosine,angle in zip(row['statuses'],facing,angles):
            if status=='outside_image':outcomes.append(status)
            elif cosine<=0:outcomes.append('back_facing_nominal')
            elif angle>75:outcomes.append('grazing_nominal')
            elif status=='blocked_in_mesh':outcomes.append(status)
            else:outcomes.append('no_modeled_obstruction_unknown_space')
        row['surface_outcomes']=outcomes;row['incidence_deg']=angles.tolist()
        per.append([o=='no_modeled_obstruction_unknown_space' for o in outcomes])
    count=np.sum(per,axis=0);result['regions']={}
    for region in ('top','side'):
        mask=np.array([r==region for r in surface['regions']]);den=float(weights[mask].sum())
        result['regions'][region]={'samples':int(mask.sum()),'opportunity_area_fraction_once':float(weights[mask&(count>0)].sum()/den) if den else None,
                                   'opportunity_area_fraction_three_views':float(weights[mask&(count>=3)].sum()/den) if den else None}
    result['incidence_limit_deg']=75;result['incidence_limit_status']='demonstration criterion, not calibrated sensor limit'
    result['interpretation']='nominal inspection opportunities only; unknown space and nominal material are not verified'
    return result
