"""Conservative coarse capillary uptake at actual cloth/lock surface contact."""
import numpy as np
from scipy.spatial import cKDTree

def distances(points,vertices,triangles):
    tri=vertices[triangles];centres=tri.mean(axis=1)
    _,ids=cKDTree(centres).query(points,k=min(12,len(tri)))
    a=tri[ids,0];b=tri[ids,1];c=tri[ids,2];p=np.asarray(points)[:,None,:]
    ab=b-a;ac=c-a;ap=p-a
    aa=np.sum(ab*ab,axis=-1);bb=np.sum(ab*ac,axis=-1);cc=np.sum(ac*ac,axis=-1)
    d=np.sum(ap*ab,axis=-1);e=np.sum(ap*ac,axis=-1);den=np.maximum(aa*cc-bb*bb,1e-18)
    u=(cc*d-bb*e)/den;v=(aa*e-bb*d)/den
    plane=a+u[...,None]*ab+v[...,None]*ac
    best=np.where((u>=0)&(v>=0)&(u+v<=1),np.sum((plane-p)**2,axis=-1),np.inf)
    for x,y in [(a,b),(b,c),(c,a)]:
        edge=y-x;t=np.clip(np.sum((p-x)*edge,axis=-1)/np.maximum(np.sum(edge*edge,axis=-1),1e-18),0,1)
        best=np.minimum(best,np.sum((x+t[...,None]*edge-p)**2,axis=-1))
    return np.sqrt(best.min(axis=1))

def advance(water,uptake,contact,dt=.05):
    # Each grouped lock segment can transfer at most0.05g/s, with60g total
    # cloth capacity. These are uncalibrated engineering approximations.
    delta=np.minimum(np.asarray(water),.05*dt*np.asarray(contact,dtype=float))
    total=float(delta.sum());delta*=min(1.,max(0.,60.-uptake)/max(total,1e-12))
    return np.asarray(water)-delta,uptake+float(delta.sum())
