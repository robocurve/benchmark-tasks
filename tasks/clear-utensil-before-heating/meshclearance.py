"""Full triangle-surface and closed-component separation; audited CPU geometry.

Boundary distance includes vertex/face, edge/edge and edge/face crossings.
Closed-solid overlap is checked separately with one representative per component.
"""
import numpy as np


def point_segment_squared(p, a, b):
    d = b-a
    u = np.clip(np.sum((p-a)*d, axis=-1)/np.maximum(np.sum(d*d, axis=-1), 1e-30), 0., 1.)
    return np.sum((p-a-u[..., None]*d)**2, axis=-1)


def point_triangle_squared(p, t):
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    u, v = b-a, c-a
    normal = np.cross(u, v)
    normal2 = np.sum(normal*normal, axis=-1)
    height = np.sum((p-a)*normal, axis=-1)
    projected = p-height[:, None]*normal/np.maximum(normal2[:, None], 1e-30)
    w = projected-a
    uu, uv, vv = np.sum(u*u, -1), np.sum(u*v, -1), np.sum(v*v, -1)
    wu, wv = np.sum(w*u, -1), np.sum(w*v, -1)
    denominator = uu*vv-uv*uv
    s = (vv*wu-uv*wv)/np.maximum(denominator, 1e-30)
    q = (uu*wv-uv*wu)/np.maximum(denominator, 1e-30)
    inside = (normal2 > 1e-28) & (s >= 0.) & (q >= 0.) & (s+q <= 1.)
    edge = np.minimum.reduce([point_segment_squared(p, a, b), point_segment_squared(p, b, c),
                              point_segment_squared(p, c, a)])
    return np.where(inside, height*height/np.maximum(normal2, 1e-30), edge)


def segment_segment_squared(a, b, c, d):
    u, v, w = b-a, d-c, a-c
    uu, uv, vv = np.sum(u*u, -1), np.sum(u*v, -1), np.sum(v*v, -1)
    uw, vw = np.sum(u*w, -1), np.sum(v*w, -1)
    determinant = uu*vv-uv*uv
    s = (uv*vw-vv*uw)/np.maximum(determinant, 1e-30)
    t = (uu*vw-uv*uw)/np.maximum(determinant, 1e-30)
    valid = (determinant > 1e-28) & (s >= 0.) & (s <= 1.) & (t >= 0.) & (t <= 1.)
    endpoints = np.minimum.reduce([point_segment_squared(a, c, d), point_segment_squared(b, c, d),
                                   point_segment_squared(c, a, b), point_segment_squared(d, a, b)])
    interior = np.sum((w+s[:, None]*u-t[:, None]*v)**2, axis=-1)
    return np.minimum(endpoints, np.where(valid, interior, np.inf))


def segment_triangle_crosses(a, b, triangle):
    v0, v1, v2 = triangle[:, 0], triangle[:, 1], triangle[:, 2]
    u, v, direction = v1-v0, v2-v0, b-a
    normal = np.cross(u, v)
    denominator = np.sum(normal*direction, -1)
    fraction = np.sum(normal*(v0-a), -1)/np.where(abs(denominator) > 1e-28, denominator, 1.)
    point = a+fraction[:, None]*direction
    return ((abs(denominator) > 1e-28) & (fraction >= 0.) & (fraction <= 1.)
            & (point_triangle_squared(point, triangle) < 1e-24))


def triangle_pair_squared(a, b):
    distance = np.full(len(a), np.inf)
    crossing = np.zeros(len(a), dtype=bool)
    for i in range(3):
        distance = np.minimum(distance, point_triangle_squared(a[:, i], b))
        distance = np.minimum(distance, point_triangle_squared(b[:, i], a))
        crossing |= segment_triangle_crosses(a[:, i], a[:, (i+1) % 3], b)
        crossing |= segment_triangle_crosses(b[:, i], b[:, (i+1) % 3], a)
        for j in range(3):
            distance = np.minimum(distance, segment_segment_squared(a[:, i], a[:, (i+1) % 3],
                                                                     b[:, j], b[:, (j+1) % 3]))
    return np.where(crossing, 0., distance)


def inside_closed_surface(triangles, point):
    t = triangles-point
    lengths = np.linalg.norm(t, axis=-1)
    numerator = np.sum(t[:, 0]*np.cross(t[:, 1], t[:, 2]), axis=-1)
    denominator = (np.prod(lengths, axis=-1) + np.sum(t[:, 0]*t[:, 1], -1)*lengths[:, 2]
                   + np.sum(t[:, 1]*t[:, 2], -1)*lengths[:, 0]
                   + np.sum(t[:, 2]*t[:, 0], -1)*lengths[:, 1])
    return abs(np.sum(2*np.arctan2(numerator, denominator))) > 2*np.pi


def surfaces_separated(a, b, clearance=.002, a_component_points=None, b_component_points=None):
    """True iff boundaries and specified closed-solid components are separated.

    Caller supplies one point on each disconnected closed component for exact
    containment handling. Default points assume each supplied mesh is connected.
    """
    a, b = np.asarray(a, float).reshape(-1, 3, 3), np.asarray(b, float).reshape(-1, 3, 3)
    if not len(a) or not len(b) or not np.isfinite(a).all() or not np.isfinite(b).all():
        return False
    alo, ahi, blo, bhi = a.min(1), a.max(1), b.min(1), b.max(1)
    axis_gap = np.maximum(alo.min(0)-bhi.max(0), blo.min(0)-ahi.max(0))
    if np.any(axis_gap >= clearance):
        return True
    limit2 = max(0., clearance-1e-12)**2
    # Boundary features outside the other complete mesh's expanded bounds
    # cannot be a close pair. Keep the complete meshes below for containment.
    use_a = np.all((ahi >= blo.min(0)-clearance) & (alo <= bhi.max(0)+clearance), axis=1)
    use_b = np.all((bhi >= alo.min(0)-clearance) & (blo <= ahi.max(0)+clearance), axis=1)
    boundary_a, boundary_b = a[use_a], b[use_b]
    al, ah, bl, bh = alo[use_a], ahi[use_a], blo[use_b], bhi[use_b]
    ac, bc = boundary_a.mean(1), boundary_b.mean(1)
    ar = np.linalg.norm(boundary_a-ac[:, None], axis=2).max(1)
    br = np.linalg.norm(boundary_b-bc[:, None], axis=2).max(1)
    for start in range(0, len(boundary_b), 16):
        stop = min(start+16, len(boundary_b))
        overlap = np.all((ah[:, None] >= bl[None, start:stop]-clearance)
                         & (bh[None, start:stop] >= al[:, None]-clearance), axis=-1)
        ai, bi = np.nonzero(overlap)
        if len(ai):
            # Enclosing spheres provide another sound lower bound on the
            # distance; dense spoon triangles need no narrow-phase work when
            # their full spheres already lie outside the clearance band.
            near = np.sum((ac[ai]-bc[start+bi])**2, axis=1) < (clearance+ar[ai]+br[start+bi])**2
            ai, bi = ai[near], bi[near]
        if len(ai) and np.any(triangle_pair_squared(boundary_a[ai], boundary_b[start+bi]) < limit2):
            return False
    if a_component_points is None:
        a_component_points = [a[0, 0]]
    if b_component_points is None:
        b_component_points = [b[0, 0]]
    return not (any(inside_closed_surface(b, p) for p in a_component_points)
                or any(inside_closed_surface(a, p) for p in b_component_points))
