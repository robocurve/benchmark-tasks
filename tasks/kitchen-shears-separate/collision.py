"""Exact separating-axis checks for the horizontally extruded convex solids.

This offline validator supports yaw/translation only. Runtime uses PhysX; no
collision result from this module unlocks or attaches any object.
"""
import numpy as np
import geometry as g

def placed(solids,pos,rot):
    if not np.allclose(rot[:,2],[0,0,1],atol=1e-8):
        raise ValueError('offline extrusion checker supports horizontal solids only')
    result=[]
    for s in solids:
        v=g.world(s.vertices,pos,rot); n=len(v)//2
        result.append((v[:n,:2],v[:,2].min(),v[:,2].max(),v[:,:2].min(0),v[:,:2].max(0),s.kind))
    return result

def overlap(a,b,tolerance=1e-7):
    pa,za0,za1,alo,ahi,_=a; pb,zb0,zb1,blo,bhi,_=b
    if min(za1,zb1)-max(za0,zb0)<=tolerance: return False
    if np.any(np.minimum(ahi,bhi)-np.maximum(alo,blo)<=tolerance): return False
    for poly in (pa,pb):
        edges=np.roll(poly,-1,axis=0)-poly
        axes=np.c_[-edges[:,1],edges[:,0]]
        sizes=np.linalg.norm(axes,axis=1); axes=axes[sizes>1e-12]/sizes[sizes>1e-12,None]
        aa=pa@axes.T; bb=pb@axes.T
        if np.any(np.minimum(aa.max(0),bb.max(0))-np.maximum(aa.min(0),bb.min(0))<=tolerance): return False
    return True

def intersections(a,b,tolerance=1e-7):
    return [(x[-1],y[-1]) for x in a for y in b if overlap(x,y,tolerance)]

def half_intersections(positions,rotations,tolerance=1e-7):
    return intersections(placed(g.half(0),positions[0],rotations[0]),
                         placed(g.half(1),positions[1],rotations[1]),tolerance)
