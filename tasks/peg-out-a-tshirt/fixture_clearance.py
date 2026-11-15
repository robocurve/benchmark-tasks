"""CPU triangle/fixture intersection audit; not a dynamic contact check.

Boxes use the full triangle/AABB separating-axis test. A finite cylinder is
tested by clipping the triangle to its axial slab, then measuring the projected
polygon's distance from the cylinder axis. Inflating a solid by ``margin`` is
conservative at its corners, so an empty result establishes at least that
clearance rather than just testing whether mesh vertices are inside a fixture.
"""
import numpy as np


def triangle_box_intersects(triangle,center,size,margin=0.):
    v=np.asarray(triangle,dtype=float)-np.asarray(center,dtype=float)
    half=np.asarray(size,dtype=float)/2+margin
    edges=np.roll(v,-1,axis=0)-v
    axes=[*np.eye(3),np.cross(edges[0],edges[1])]
    axes.extend(np.cross(edge,axis) for edge in edges for axis in np.eye(3))
    for axis in axes:
        if np.dot(axis,axis)<1e-24:continue
        projections=v@axis
        radius=half@np.abs(axis)
        if projections.min()>radius+1e-12 or projections.max()<-radius-1e-12:
            return False
    return True


def _clip_z(poly,height,keep_above):
    result=[]
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        da=(a[2]-height)*(1 if keep_above else -1)
        db=(b[2]-height)*(1 if keep_above else -1)
        a_inside=da>=0.;b_inside=db>=0.
        if a_inside:result.append(a)
        if a_inside!=b_inside:result.append(a+(b-a)*(da/(da-db)))
    return np.asarray(result)


def triangle_cylinder_intersects(triangle,center,radius,height,margin=0.,axis='Z'):
    order={'X':(1,2,0),'Y':(0,2,1),'Z':(0,1,2)}[axis]
    poly=(np.asarray(triangle,dtype=float)-np.asarray(center,dtype=float))[:,order]
    half=height/2+margin
    if poly[:,2].max() < -half or poly[:,2].min() > half:return False
    poly=_clip_z(poly,-half,True)
    if len(poly)==0:return False
    poly=_clip_z(poly,half,False)
    if len(poly)==0:return False
    xy=poly[:,:2];ends=np.roll(xy,-1,axis=0)
    cross=xy[:,0]*ends[:,1]-xy[:,1]*ends[:,0]
    # Convex clipping preserves winding. Degenerate projections are segments.
    if abs(cross.sum())>1e-16 and (np.all(cross>=-1e-12) or np.all(cross<=1e-12)):
        return True
    delta=ends-xy;length2=np.sum(delta*delta,axis=1)
    t=np.divide(-np.sum(xy*delta,axis=1),length2,out=np.zeros_like(length2),where=length2>1e-24)
    closest=xy+np.clip(t,0.,1.)[:,None]*delta
    return bool(np.min(np.sum(closest*closest,axis=1))<=(radius+margin)**2+1e-12)


def fixture_intersections(points,faces,fixtures,margin=.001):
    """Return intersecting face indices per fixture, including touching faces."""
    triangles=np.asarray(points)[np.asarray(faces)]
    hits={}
    for name,spec in fixtures.items():
        center=np.asarray(spec['center'])
        if spec['kind']=='box':size=np.asarray(spec['size'])
        else:
            size=np.full(3,2*spec['radius'])
            size['XYZ'.index(spec.get('axis','Z'))]=spec['height']
        low=center-size/2-margin;high=center+size/2+margin
        candidates=np.flatnonzero(np.all(triangles.max(1)>=low,axis=1)&
                                  np.all(triangles.min(1)<=high,axis=1))
        if spec['kind']=='box':
            hit=lambda t:triangle_box_intersects(t,center,size,margin)
        elif spec['kind']=='cylinder':
            hit=lambda t:triangle_cylinder_intersects(t,center,spec['radius'],spec['height'],margin,spec.get('axis','Z'))
        else:raise ValueError('Unsupported fixture primitive: '+spec['kind'])
        hits[name]=[int(i) for i in candidates if hit(triangles[i])]
    return hits
