"""Pure geometry shared by 240 Hz monitor and offline adversarial checks."""
import numpy as np

def triangles_touch_cylinder(triangles,center,radius,z_low,z_high):
    """Closed finite vertical cylinder; clip each boundary-crossing face first.

    Most cloth faces are entirely inside/outside the height slab. Only faces
    crossing either end cap require polygon clipping, preserving exact XY edges.
    """
    t=np.asarray(triangles,float).reshape(-1,3,3)
    if not np.isfinite(t).all() or not np.isfinite([*center,radius,z_low,z_high]).all():return True
    if z_low>z_high or radius<0:raise ValueError('Invalid cylinder')
    lo=t[:,:,2].min(1);hi=t[:,:,2].max(1)
    eps=1e-12  # Metres: arithmetic guard for mathematically closed boundaries.
    overlap=(hi>=z_low-eps)&(lo<=z_high+eps)
    whole=overlap&(lo>=z_low-eps)&(hi<=z_high+eps)
    if np.any(whole) and triangles_touch_disk(t[whole],center,radius):return True
    for triangle in t[overlap&~whole]:
        polygon=list(triangle)
        for height,sign in ((z_low,1),(z_high,-1)):
            if not polygon:break
            clipped=[];previous=polygon[-1];was_inside=sign*(previous[2]-height)>=-eps
            for point in polygon:
                inside=sign*(point[2]-height)>=-eps
                if inside!=was_inside:
                    fraction=(height-previous[2])/(point[2]-previous[2])
                    intersection=previous+fraction*(point-previous)
                    intersection[2]=height
                    clipped.append(intersection)
                if inside:clipped.append(point)
                previous=point;was_inside=inside
            polygon=clipped
        if not polygon:continue
        if len(polygon)<3:
            polygon=polygon+[polygon[-1]]*(3-len(polygon))
        fan=np.array([[polygon[0],polygon[i],polygon[i+1]] for i in range(1,len(polygon)-1)])
        if triangles_touch_disk(fan,center,radius):return True
    return False

def triangles_touch_disk(triangles,center,radius):
    t=np.asarray(triangles,float)[...,:2]-np.asarray(center,float)
    if not np.isfinite(t).all():return True
    squared_radius=(radius+1e-12)**2  # 1 pm guard, not a physical clearance margin.
    if np.any(np.sum(t*t,axis=-1)<=squared_radius):return True
    a=t;b=np.roll(t,-1,axis=1);d=b-a
    u=np.clip(-np.sum(a*d,axis=-1)/np.maximum(np.sum(d*d,axis=-1),1e-30),0,1)
    p=a+u[...,None]*d
    if np.any(np.sum(p*p,axis=-1)<=squared_radius):return True
    # Disk centre inside a triangle: edges and vertices may all lie outside.
    cross=d[:,:,0]*(-a[:,:,1])-d[:,:,1]*(-a[:,:,0])
    area=np.abs(np.sum(t[:,:,0]*b[:,:,1]-t[:,:,1]*b[:,:,0],axis=1))
    return bool(np.any(((cross>=0).all(1)|(cross<=0).all(1))&(area>1e-15)))

def point_inside_closed_mesh(triangles,point):
    t=np.asarray(triangles,float)-np.asarray(point,float)
    if not np.isfinite(t).all():return True
    lengths=np.linalg.norm(t,axis=-1)
    determinant=np.einsum('ij,ij->i',t[:,0],np.cross(t[:,1],t[:,2]))
    denominator=np.prod(lengths,axis=1)+np.sum(t[:,0]*t[:,1],axis=1)*lengths[:,2]+np.sum(t[:,1]*t[:,2],axis=1)*lengths[:,0]+np.sum(t[:,2]*t[:,0],axis=1)*lengths[:,1]
    return bool(abs(np.sum(2*np.arctan2(determinant,denominator)))>2*np.pi)

def mesh_touches_contact_disk(triangles,center,radius,surface_z,tolerance=.002):
    """Boundary plus solid containment for the closed thin contact volume."""
    t=np.asarray(triangles,float)
    if not np.isfinite(t).all():return True
    if triangles_touch_cylinder(t,center,radius,surface_z-tolerance,surface_z+tolerance):return True
    # If two connected solids overlap without boundary contact, one contains
    # the other. The mesh-inside-cylinder case already hits above; this handles
    # a contact cylinder wholly contained inside a closed object.
    return point_inside_closed_mesh(t,[*center,surface_z])

def surface_disk_distance(triangles,center):
    t=np.asarray(triangles,float)[...,:2]-np.asarray(center,float)
    if not np.isfinite(t).all():return -np.inf
    if triangles_touch_disk(np.asarray(triangles),center,0.):return 0.
    a=t;b=np.roll(t,-1,axis=1);d=b-a
    fraction=np.clip(-np.sum(a*d,axis=-1)/np.maximum(np.sum(d*d,axis=-1),1e-30),0,1)
    return float(np.sqrt(np.sum((a+fraction[...,None]*d)**2,axis=-1).min()))

class Trial:
    def __init__(self):self.reset()
    def reset(self):
        self.failed=False;self.reason='';self.physics_steps=0
        self.wood_s=0.;self.pan_s=0.
    def fail(self,reason):
        if not self.failed:self.failed=True;self.reason=reason
    def monitor(self,wood_triangles,angle,center,surface_z,table_z,pan_min_z,fixture_ok=True,radius=.1):
        self.physics_steps+=1
        t=np.asarray(wood_triangles,float)
        if not np.isfinite(t).all() or not np.isfinite([angle,pan_min_z]).all():
            self.fail('invalid_physics_state');return
        if t[...,2].min()<table_z-.05 or pan_min_z<table_z-.05:self.fail('object_fell')
        if not fixture_ok:self.fail('fixed_fixture_moved')
        if not self.failed and angle>np.radians(30) and mesh_touches_contact_disk(t,center,radius,surface_z):
            self.fail('heated_with_utensil_contact')
