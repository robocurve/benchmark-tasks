"""End-state evidence of a released, exposed receiving tray; no trajectory or arm-role conditions."""
import numpy as np
from scipy.spatial import ConvexHull
OUTLINE=np.array([[-.0254,-.0254],[.0254,-.0254],[.0254,.0224],[.0224,.0254],[-.0254,.0254]])
SOLID=np.vstack([np.c_[OUTLINE,np.full(5,z)] for z in [-.0035,0]])
EDGES=[(i+layer,(i+1)%5+layer) for layer in (0,5) for i in range(5)]+[(i,i+5) for i in range(5)]
def overlap(a,b):
    for p in [a,b]:
        dif=np.roll(p,-1,axis=0)-p
        for x,y in dif:
            axis=np.array([-y,x]);aa=a@axis;bb=b@axis
            if aa.max()<=bb.min()+1e-12 or bb.max()<=aa.min()+1e-12:return False
    return True
def exposed(translation,rotation,mouth):
    vertices=SOLID@rotation.T+translation
    kept=list(vertices[vertices[:,2]>=0])
    for i,j in EDGES:
        p,q=vertices[i],vertices[j]
        if p[2]*q[2]<0:kept.append(p+p[2]/(p[2]-q[2])*(q-p))
    if len(kept)<3:return True
    pts=np.array(kept)[:,:2]
    if np.linalg.matrix_rank(pts-pts[0],tol=1e-9)<2:return True
    poly=pts[ConvexHull(pts).vertices]
    square=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*mouth/2
    for i in [-1,0,1]:
        for j in [-1,0,1]:
            if overlap(poly,square+np.array([i,j])*.013):return False
    return True

from itertools import product

def covered(points,offset,rot):
    """Project the part above the receiver datum; use the exact clipped convex outline."""
    v=rot.apply(points)+offset
    if np.max(v[:,2])<0 or (v[:,:2].min(0)>.01845).any() or (v[:,:2].max(0)<-.01845).any():
        return []
    kept=list(v[v[:,2]>=0])
    if len(kept)!=len(v):
        # Intersection of the convex hull with z=0. Interior points do not affect its hull.
        for p in v[v[:,2]>0]:
            for q in v[v[:,2]<0]:
                kept.append(p+(q-p)*(p[2]/(p[2]-q[2])))
    xy=np.asarray(kept)[:,:2]
    if len(xy)<3 or np.linalg.matrix_rank(xy-xy[0],tol=1e-9)<2:
        return []
    poly=xy[ConvexHull(xy).vertices]
    if (poly.min(0)>.01845).any() or (poly.max(0)<-.01845).any():
        return []
    square=np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*.00545
    return [(x,y) for x,y in product(range(-1,2),repeat=2) if overlap(poly,square+.013*np.array([x,y]))]

