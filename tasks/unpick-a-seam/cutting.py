"""Force/time thread severing and conservative blade-damage detection.

The cut input must come from a PhysX blade-vs-thread contact-force matrix. Blade
pose or distance alone never cuts thread. A true contact event disables one
native joint within the contacted thread loop; the cloth is never detached.
Parameters are an explicitly approximate cutting law, not measured tool data.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class CutParameters:
    minimum_edge_force_n: float=.25
    sustained_seconds: float=.040
    thread_radius_m: float=.0002


class ThreadCutter:
    def __init__(self,link_counts,parameters=CutParameters()):
        self.counts=tuple(map(int,link_counts));self.p=parameters
        if not self.counts or min(self.counts)<3:raise ValueError('Closed thread loops need three or more links')
        self.elapsed=np.zeros(sum(self.counts));self.severed=set();self.events=[];self.t=0.

    def update(self,forces_on_blade,edge_normal_world,dt):
        f,n=np.asarray(forces_on_blade,float),np.asarray(edge_normal_world,float)
        if (f.shape!=(len(self.elapsed),3) or n.shape!=(3,) or not np.isfinite(f).all()
                or not np.isfinite(n).all() or np.linalg.norm(n)<1e-9 or not np.isfinite(dt) or dt<=0):
            raise ValueError('Invalid measured blade contact state')
        normal=n/np.linalg.norm(n)
        force=-f@normal
        active=force>=self.p.minimum_edge_force_n
        self.elapsed=np.where(active,self.elapsed+dt,0.)
        newly=[]
        for k in np.flatnonzero(self.elapsed>=self.p.sustained_seconds):
            if int(k) not in self.severed:
                self.severed.add(int(k));newly.append(int(k))
                self.events.append({'time_s':self.t+dt,'link':int(k),'edge_force_n':float(force[k]),
                                    'continuous_contact_s':float(self.elapsed[k])})
        self.t+=dt
        return newly


def segment_triangle_intersections(a,b,vertices,faces,eps=1e-10):
    """Scale-relative finite-segment test over actual cloth faces.

    ``eps`` is dimensionless. A fixed determinant cutoff has cubic length
    units and would silently miss crossings after submillimetre refinement.
    """
    a,b,p,f=np.asarray(a,float),np.asarray(b,float),np.asarray(vertices,float),np.asarray(faces,int)
    d=b-a;v0,v1,v2=(p[f[:,i]] for i in range(3))
    e1,e2=v1-v0,v2-v0
    h=np.cross(np.broadcast_to(d,e2.shape),e2);det=np.einsum('ij,ij->i',e1,h)
    scale=np.linalg.norm(d)*np.linalg.norm(e1,axis=1)*np.linalg.norm(e2,axis=1)
    nonparallel=(scale>0)&(np.abs(det)>eps*scale)
    inv=np.divide(1.,det,out=np.zeros_like(det),where=nonparallel)
    s=a-v0;u=inv*np.einsum('ij,ij->i',s,h)
    q=np.cross(s,e1);v=inv*(q@d);t=inv*np.einsum('ij,ij->i',e2,q)
    return np.flatnonzero(nonparallel&(u>=-eps)&(v>=-eps)&(u+v<=1+eps)&(t>=-eps)&(t<=1+eps))


class DamageAudit:
    """Latch any blade crossing of fabric outside the pre-existing holes.

    Native FEM cannot tear topology at runtime. This audit conservatively
    invalidates a trial rather than pretend intact fabric survived a blade.
    It does not claim to simulate fracture propagation or inspect tiny fibres.
    """
    def __init__(self):self.crossed_faces=set();self.events=[];self.previous_cloth=None

    def update(self,blade_previous,blade_current,cloth_vertices,cloth_faces,time_s):
        before,now=np.asarray(blade_previous,float),np.asarray(blade_current,float)
        if (before.shape!=now.shape or now.ndim!=2 or now.shape[1]!=3 or
                not np.isfinite(before).all() or not np.isfinite(now).all()):
            raise ValueError('Blade samples must be Nx3')
        blade_low=np.minimum(before.min(0),now.min(0))
        blade_high=np.maximum(before.max(0),now.max(0))
        for layer,(p,f) in enumerate(zip(cloth_vertices,cloth_faces)):
            p=np.asarray(p);f=np.asarray(f,int)
            global_low=p.min(0);global_high=p.max(0)
            if self.previous_cloth is not None:
                previous=np.asarray(self.previous_cloth[layer])
                # This bound contains current/old triangles and their endpoint
                # translations without first expanding all mesh faces.
                travel=np.abs(p-previous).max(0)
                global_low=np.minimum(global_low,previous.min(0))-travel
                global_high=np.maximum(global_high,previous.max(0))+travel
            if np.any((blade_high<global_low-1e-9)|(blade_low>global_high+1e-9)):continue
            current=p[f];low=current.min(1);high=current.max(1)
            old=None;displacement=None
            if self.previous_cloth is not None:
                old=np.asarray(self.previous_cloth[layer])[f]
                displacement=current[:,0]-old[:,0]
                # Bound every triangle used by both translation-relative
                # sweeps, including each translated endpoint shape. Keeping
                # all four shapes also covers rotating/deforming faces.
                for shape in (old,old+displacement[:,None],current-displacement[:,None]):
                    low=np.minimum(low,shape.min(1));high=np.maximum(high,shape.max(1))
            # Conservative broad phase only. The original exact intersection
            # tests and tolerances still decide every candidate; 1 nm padding
            # exceeds their tolerance band at this task's sub-metre scale.
            candidates=np.flatnonzero(np.all((blade_high>=low-1e-9)&(blade_low<=high+1e-9),axis=1))
            if not len(candidates):continue
            subset=f[candidates]
            hits=set()
            for a,b in zip(before,now):hits.update(candidates[segment_triangle_intersections(a,b,p,subset)].tolist())
            # Include the physical blade edge itself, not only its moving tips.
            for a,b in zip(now[:-1],now[1:]):hits.update(candidates[segment_triangle_intersections(a,b,p,subset)].tolist())
            if self.previous_cloth is not None:
                # Cloth may move through a stationary blade. Reuse the same
                # strict, translation-relative sweep guard as thread contact.
                from contact_audit import _hits
                old=old[candidates];current=current[candidates];displacement=displacement[candidates]
                for a,b in zip(before,now):
                    old_start=np.broadcast_to(a,displacement.shape)
                    new_end=np.broadcast_to(b,displacement.shape)
                    hit=_hits(old_start,new_end-displacement,old)|_hits(old_start+displacement,new_end,current)
                    hits.update(candidates[np.flatnonzero(hit)].tolist())
            fresh={(layer,int(i)) for i in hits}-self.crossed_faces
            if fresh:self.events.append({'time_s':float(time_s),'layer':layer,'triangles':sorted(i for _,i in fresh)})
            self.crossed_faces.update(fresh)
        self.previous_cloth=[np.asarray(p).copy() for p in cloth_vertices]
        return bool(self.crossed_faces)
