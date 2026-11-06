"""Actual surface clearance from a narrow finger-pad OBB (not whole-finger/cloth AABBs)."""
import itertools
import numpy as np

_SIGN=np.array(list(itertools.product([-1,1],repeat=3)))
_EDGES=np.array([(i,j) for i in range(8) for j in range(i+1,8) if np.count_nonzero(_SIGN[i]!=_SIGN[j])==1])

def _point_segments(p,a,b):
    d=b-a
    t=np.clip(np.sum((p-a)*d,axis=-1)/np.maximum(np.sum(d*d,axis=-1),1e-30),0,1)
    return np.sum((p-a-t[...,None]*d)**2,axis=-1)

def pad_clear(center,rotation,half,triangles,clearance=.03):
    """Euclidean pad-box/surface clearance, including crossing edges and face interiors.

    Vertex/face and edge/edge features cover disjoint convex-box/triangle distances.
    For intersecting surfaces, a box vertex is within half-diagonal of some
    intersection point, so the half-diagonal < clearance assertion also detects it.
    """
    half=np.asarray(half);t=(np.asarray(triangles)-center)@rotation
    assert np.linalg.norm(half)<clearance
    low=t.min((0,1));high=t.max((0,1))
    if np.linalg.norm(np.maximum(np.maximum(-half-high,low-half),0))>=clearance:return True
    # Any mesh vertex close to the actual oriented pad box.
    if np.any(np.sum(np.maximum(np.abs(t)-half,0)**2,axis=-1)<clearance**2):return False
    corners=_SIGN*half
    # A box corner can be nearest to a triangle's interior (no nearby mesh nodes).
    from trimesh.triangles import closest_point
    for p in corners:
        q=closest_point(t,np.broadcast_to(p,(len(t),3)))
        if np.any(np.sum((q-p)**2,axis=-1)<clearance**2):return False
    # Remaining closest-feature case: an interior point of each of two edges.
    a=t;b=np.roll(t,-1,axis=1);u=b-a
    for i,j in _EDGES:
        c=corners[i];d=corners[j];v=d-c;w=a-c
        if np.any(_point_segments(c,a,b)<clearance**2) or np.any(_point_segments(d,a,b)<clearance**2):return False
        # mesh endpoint to box segment was already covered by point-to-box above.
        aa=np.sum(u*u,axis=-1);bb=np.sum(u*v,axis=-1);cc=np.dot(v,v)
        dd=np.sum(u*w,axis=-1);ee=np.sum(v*w,axis=-1);den=aa*cc-bb*bb
        valid=den>1e-18
        ss=np.where(valid,(bb*ee-cc*dd)/np.maximum(den,1e-30),0)
        tt=np.where(valid,(aa*ee-bb*dd)/np.maximum(den,1e-30),0)
        valid &= (ss>=0)&(ss<=1)&(tt>=0)&(tt<=1)
        distance=np.sum((w+ss[...,None]*u-tt[...,None]*v)**2,axis=-1)
        if np.any(valid&(distance<clearance**2)):return False
    return True

def checks():
    h=np.array([.02,.005,.017]);r=np.eye(3);c=np.zeros(3)
    assert not pad_clear(c,r,h,[[[-2,-2,.025],[2,-2,.025],[0,3,.025]]])
    assert pad_clear(c,r,h,[[[1,1,1],[2,1,1],[1,2,1]]])
    assert not pad_clear(c,r,h,[[[-.2,0,0],[.2,0,0],[0,.2,0]]])
    print('pad/surface clearance checks passed')

if __name__=='__main__':checks()
