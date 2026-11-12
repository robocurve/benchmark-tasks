"""Continuous gauge/solid distances from authored convex geometry and LIVE poses.

CPU PhysX scene queries can lag GPU rigid-body transforms in this stack. They are
not the grading authority. No geometry/state from this module enters the policy.
"""
from functools import lru_cache
import numpy as np
from scipy.spatial import ConvexHull
from .geometry import wedges
from . import layout as L

def segment_triangle_distances(start,end,triangles):
    """Exact Euclidean distances (up to float roundoff) to closed triangles.

    Minimum of endpoint/face and segment/edge distances, plus interior crossings.
    Vectorized over triangles. Degenerate faces fall back to their edges.
    """
    tri=np.asarray(triangles);a,b,c=tri[:,0],tri[:,1],tri[:,2]
    v=b-a;w=c-a;n=np.cross(v,w);nn=np.einsum('ij,ij->i',n,n)
    result=np.full(len(tri),np.inf);d=np.asarray(end)-start;dd=float(d@d)
    for p in (np.asarray(start),np.asarray(end)):
        ap=p-a;h=np.einsum('ij,ij->i',ap,n)
        face=nn>1e-30;den=np.where(face,nn,1.)
        # Cross-product barycentrics avoid cancellation on narrow triangles.
        u=np.einsum('ij,ij->i',np.cross(ap,w),n)/den
        t=np.einsum('ij,ij->i',np.cross(v,ap),n)/den
        inside=(u>=-1e-10)&(t>=-1e-10)&(u+t<=1+1e-10)
        result=np.minimum(result,np.where(face&inside,h*h/den,np.inf))
    for e0,e1 in ((a,b),(b,c),(c,a)):
        e=e1-e0;r=np.asarray(start)-e0
        ee=np.einsum('ij,ij->i',e,e);de=e@d;dr=r@d;er=np.einsum('ij,ij->i',e,r)
        det=dd*ee-de*de
        s=np.zeros(len(tri));valid=det>1e-18
        s[valid]=np.clip((de[valid]*er[valid]-ee[valid]*dr[valid])/det[valid],0,1)
        t=(de*s+er)/np.where(ee>0,ee,1.)
        low=t<0;high=t>1
        s[low]=np.clip(-dr[low]/dd,0,1);s[high]=np.clip((de[high]-dr[high])/dd,0,1)
        t=np.clip(t,0,1);delta=r+s[:,None]*d-t[:,None]*e
        result=np.minimum(result,np.einsum('ij,ij->i',delta,delta))
    # Interior segment/triangle intersection (edge distances alone miss it).
    nd=n@d;h=np.einsum('ij,ij->i',a-np.asarray(start),n)
    t=np.full(len(tri),np.inf);valid=np.abs(nd)>1e-15;t[valid]=h[valid]/nd[valid]
    ap=np.asarray(start)+np.where(valid,t,0)[:,None]*d-a
    den=np.where(nn>1e-30,nn,1.)
    u=np.einsum('ij,ij->i',np.cross(ap,w),n)/den
    k=np.einsum('ij,ij->i',np.cross(v,ap),n)/den
    cross=valid&(nn>1e-30)&(t>=0)&(t<=1)&(u>=-1e-10)&(k>=-1e-10)&(u+k<=1+1e-10)
    result[cross]=0
    return np.sqrt(np.maximum(result,0))

@lru_cache(None)
def solid(name):
    triangles=[];planes=[]
    for _,points in wedges(**L.ring(name)):
        p=np.asarray(points);h=ConvexHull(p)
        triangles.extend(p[h.simplices]);planes.append(h.equations)
    return np.asarray(triangles),planes

def matrix(q):
    x,y,z,w=np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def clearance(name,start,end):
    triangles,planes=solid(name)
    distance=float(segment_triangle_distances(start,end,triangles).min())
    # Surface distance alone would be wrong for a segment wholly inside material.
    for eq in planes:
        if np.all(eq[:,:3]@start+eq[:,3]<=1e-10) or np.all(eq[:,:3]@end+eq[:,3]<=1e-10):return 0.
    return distance

def check(pose):
    p=np.asarray(pose,dtype=float)
    if p.shape!=(7,) or not np.isfinite(p).all() or abs(np.linalg.norm(p[3:])-1)>.001:
        return dict(valid=False,clear=False,method='live-pose-convex-solid')
    start=np.asarray(L.HOUSING)+[0,L.HOUSING_OUT+.007,0]
    end=np.asarray(L.HOUSING)+[0,L.R_IN-.005,0]
    housing=clearance('housing',start-np.asarray(L.HOUSING),end-np.asarray(L.HOUSING))
    r=matrix(p[3:]);sleeve=clearance('sleeve',(start-p[:3])@r,(end-p[:3])@r)
    return dict(valid=True,clear=min(housing,sleeve)>=.005-1e-8,diameter_m=.010,tolerance_m=1e-8,
                housing_clearance_m=housing,sleeve_clearance_m=sleeve,
                method='live-pose-convex-solid',note='Continuous capsule distance to convex solids; no fluid model.')
