"""Non-convex film containment, with a virtual cap across the open mouth.

The cap is an evaluator surface, never a collider or force. Signed solid
angles distinguish exterior folds from the actual bag interior. A 5 mm
surface tolerance accounts for deformable/rigid contact offsets.
"""
import numpy as np

def closed_triangles(vertices, faces, mouth_ids):
    vertices = np.asarray(vertices, dtype=float)
    mouth = vertices[np.asarray(mouth_ids, dtype=int)]
    centre = mouth.mean(axis=0)
    cap = np.stack([mouth, np.roll(mouth, -1, axis=0), np.repeat(centre[None], len(mouth), axis=0)], axis=1)
    return np.concatenate([vertices[np.asarray(faces, dtype=int)], cap])

def surface_distance(points, triangles):
    """Exact point-to-triangle distance, vectorised over one small point batch."""
    p = np.asarray(points)[:, None, :]
    a, b, c = (triangles[None, :, i, :] for i in range(3))
    ab, ac, ap = b-a, c-a, p-a
    d00 = (ab*ab).sum(-1); d01 = (ab*ac).sum(-1); d11 = (ac*ac).sum(-1)
    d20 = (ap*ab).sum(-1); d21 = (ap*ac).sum(-1)
    denom = d00*d11-d01*d01
    safe = np.where(np.abs(denom)>1e-25, denom, 1.)
    u = (d11*d20-d01*d21)/safe; v = (d00*d21-d01*d20)/safe
    projection = a+u[..., None]*ab+v[..., None]*ac
    distance2 = np.where((u>=0)&(v>=0)&(u+v<=1)&(denom>1e-25), ((p-projection)**2).sum(-1), np.inf)
    for start, end in ((a,b),(b,c),(c,a)):
        edge = end-start
        t = np.clip(((p-start)*edge).sum(-1)/np.maximum((edge*edge).sum(-1),1e-25),0,1)
        distance2 = np.minimum(distance2, ((p-start-t[...,None]*edge)**2).sum(-1))
    return np.sqrt(np.maximum(0, distance2.min(axis=1)))

def contains_points(points, triangles, tolerance=.005, signed=False):
    """Return per-point containment; never substitute the convex hull.

    signed=True counts only positive winding (the bag interior side of the
    film), so the gap between the liner and its folded-back cuff is outside.
    """
    points = np.asarray(points, dtype=float)
    if not np.isfinite(points).all() or not np.isfinite(triangles).all():
        return np.zeros(len(points), dtype=bool)
    a,b,c = (triangles[None,:,i,:]-points[:,None,:] for i in range(3))
    la=np.linalg.norm(a,axis=-1);lb=np.linalg.norm(b,axis=-1);lc=np.linalg.norm(c,axis=-1)
    numerator=np.einsum('pti,pti->pt',a,np.cross(b,c))
    denominator=la*lb*lc+(a*b).sum(-1)*lc+(b*c).sum(-1)*la+(c*a).sum(-1)*lb
    winding=(2*np.arctan2(numerator,denominator)).sum(axis=1)/(4*np.pi)
    inside=(winding>.5) if signed else (np.abs(winding)>.5)
    outside=np.flatnonzero(~inside)
    if len(outside):inside[outside]=surface_distance(points[outside],triangles)<=tolerance
    return inside
