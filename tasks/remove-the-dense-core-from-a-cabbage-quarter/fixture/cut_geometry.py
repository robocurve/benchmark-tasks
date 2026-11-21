"""Where the held blade edge has cut the two core-boundary surfaces (all lengths in metres, quarter frame).

Each boundary surface runs from the root corner (-70 mm, +-25 mm) to the apex (-30 mm, 0), from the board up
through the 58 mm core height. A cut counts at a boundary point when the swept blade edge passes through it
anywhere between BAND_IN inside the core outline and BAND_OUT outside it, in the leaves. A cut farther inside
leaves dense core on the leaves; a cut farther out does not count here and its edible waste is scored separately
(task.py, swept_leaf_damage). The band applies only normal to the boundary: it never widens a cut along the
boundary or through its depth.

coverage: the swept surface, clipped to the band, is projected onto the boundary's (length, height) plane and
marks the grid points it covers.
incision: one physically connected swept surface inside the band must span 20 mm along the boundary and 5 mm
of height; separate cuts are measured separately.
"""
import numpy as np

EPS = 1e-9
BAND_IN, BAND_OUT = .002, .010
Z_LO, Z_HI = .001, .059


def frame(side):
    """Origin, rows (along the boundary, outward normal, up) and length of boundary side -1 or +1."""
    origin = np.array([-.07, side * .025, 0.])
    tangent = np.array([.04, -side * .025, 0.])
    length = float(np.linalg.norm(tangent))
    tangent /= length
    normal = side * np.array([-tangent[1], tangent[0], 0.])  # points away from the core, into the leaves
    return origin, np.array([tangent, normal, [0., 0., 1.]]), length


FRAMES = [frame(-1), frame(1)]


def grid(side, ds=.002, dz=.002):
    """Boundary sample points (quarter frame) and their (length, height) coordinates; 1 mm inset at both ends."""
    origin, basis, length = FRAMES[(side + 1) // 2]
    s = np.arange(.001, length - .001 + 1e-9, ds)
    z = np.arange(.002, .058 + 1e-9, dz)
    ss, zz = np.meshgrid(s, z, indexing='ij')
    sz = np.stack([ss.ravel(), zz.ravel()], axis=1)
    points = origin + sz[:, :1] * basis[0] + sz[:, 1:] * basis[2]
    return points, sz


def clip(poly, axis, value, sign):
    if not len(poly):
        return poly
    out = []
    for a, b in zip(poly, np.roll(poly, -1, axis=0)):
        da = sign * (a[axis] - value)
        db = sign * (b[axis] - value)
        if da >= -EPS:
            out.append(a)
        if (da > EPS and db < -EPS) or (da < -EPS and db > EPS):
            out.append(a + (b - a) * da / (da - db))
        elif da < -EPS and abs(db) <= EPS:
            out.append(b)
    return np.asarray(out)


def band_polygons(before, after, side):
    """Swept-edge triangles between two observations, clipped to the band of one boundary (boundary coordinates)."""
    origin, basis, length = FRAMES[(side + 1) // 2]
    a = (np.asarray(before) - origin) @ basis.T
    b = (np.asarray(after) - origin) @ basis.T
    for i in range(len(a) - 1):
        for triangle in (np.array([a[i], a[i + 1], b[i + 1]]), np.array([a[i], b[i + 1], b[i]])):
            poly = triangle
            for axis, value, sign in ((1, -BAND_IN, 1), (1, BAND_OUT, -1), (0, 0., 1), (0, length, -1),
                                      (2, Z_LO, 1), (2, Z_HI, -1)):
                poly = clip(poly, axis, value, sign)
                if len(poly) < 3:
                    break
            if len(poly) < 3:
                continue
            area = np.linalg.norm(np.sum(np.cross(poly, np.roll(poly, -1, axis=0)), axis=0)) / 2
            if area > 1e-12:
                yield poly


def cover(mask, sz, poly, tol=1e-6):
    """Mark grid points (length, height) inside the polygon's projection onto the boundary plane."""
    p = poly[:, [0, 2]]
    area = np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1]) / 2
    if abs(area) < 1e-10:
        return
    if area < 0:
        p = p[::-1]
    lo = p.min(axis=0) - tol
    hi = p.max(axis=0) + tol
    near = np.flatnonzero(np.all((sz >= lo) & (sz <= hi), axis=1) & ~mask)
    if not len(near):
        return
    q = sz[near]
    inside = np.ones(len(near), bool)
    for a, b in zip(p, np.roll(p, -1, axis=0)):
        e = b - a
        length = np.hypot(*e)
        if length < 1e-12:
            continue
        inside &= (e[0] * (q[:, 1] - a[1]) - e[1] * (q[:, 0] - a[0])) / length >= -tol
    mask[near[inside]] = True


def section(poly, normal, origin):
    out = []
    distance = (poly - origin) @ normal
    for i, (a, b) in enumerate(zip(poly, np.roll(poly, -1, axis=0))):
        da = distance[i]
        db = distance[(i + 1) % len(poly)]
        if abs(da) <= EPS:
            out.append(a)
        if (da > EPS and db < -EPS) or (da < -EPS and db > EPS):
            out.append(a + (b - a) * da / (da - db))
    return np.asarray(out)


def plane(poly):
    origin = poly[0]
    best = np.zeros(3)
    for i in range(1, len(poly) - 1):
        for j in range(i + 1, len(poly)):
            n = np.cross(poly[i] - origin, poly[j] - origin)
            if np.linalg.norm(n) > np.linalg.norm(best):
                best = n
    length = np.linalg.norm(best)
    return origin, best / length if length > 1e-15 else None


def touching2d(a, b):
    if np.any(a.max(axis=0) < b.min(axis=0) - EPS) or np.any(b.max(axis=0) < a.min(axis=0) - EPS):
        return False
    for p in (a, b):
        for edge in np.roll(p, -1, axis=0) - p:
            normal = np.array([-edge[1], edge[0]])
            length = np.linalg.norm(normal)
            if length < EPS:
                continue
            normal /= length
            aa = a @ normal
            bb = b @ normal
            if aa.max() < bb.min() - EPS or bb.max() < aa.min() - EPS:
                return False
    return True


def touching(a, b):
    """True only when two cut polygons physically meet in 3D (parallel cuts at different depths never merge)."""
    if np.any(a.max(axis=0) < b.min(axis=0) - EPS) or np.any(b.max(axis=0) < a.min(axis=0) - EPS):
        return False
    oa, na = plane(a)
    ob, nb = plane(b)
    if na is None or nb is None:
        return False
    line = np.cross(na, nb)
    length = np.linalg.norm(line)
    if length < 1e-10:
        if abs(float((ob - oa) @ na)) > EPS:
            return False
        axes = [i for i in range(3) if i != int(np.argmax(np.abs(na)))]
        return touching2d(a[:, axes], b[:, axes])
    aa = section(a, nb, ob)
    bb = section(b, na, oa)
    if not len(aa) or not len(bb):
        return False
    line /= length
    ta = aa @ line
    tb = bb @ line
    return max(ta.min(), tb.min()) <= min(ta.max(), tb.max()) + EPS


def add_patch(groups, poly):
    lo = poly.min(axis=0)
    hi = poly.max(axis=0)
    matched = []
    for i, g in enumerate(groups):
        if np.any(g['hi'] < lo - EPS) or np.any(hi < g['lo'] - EPS):
            continue
        if any(touching(poly, p) for p in reversed(g['polys'])):
            matched.append(i)
    polys = [poly]
    for i in reversed(matched):
        g = groups.pop(i)
        polys.extend(g['polys'])
        lo = np.minimum(lo, g['lo'])
        hi = np.maximum(hi, g['hi'])
    groups.append(dict(polys=polys, lo=lo, hi=hi))
    return hi[0] - lo[0] >= .020 - EPS and hi[2] - lo[2] >= .005 - EPS


def sweep(state, before, after):
    """Record one held-blade edge step (quarter frame) on both boundaries; returns True if new points were cut."""
    new = False
    for i, side in enumerate((-1, 1)):
        mask = state['cuts'][i]
        count = int(mask.sum())
        for poly in band_polygons(before, after, side):
            cover(mask, SZ[i], poly)
            if not state['incised'] and add_patch(state['incision_groups'][i], poly):
                state['incised'] = True
        new |= int(mask.sum()) > count
    return new


GRIDS = [grid(-1), grid(1)]
POINTS = [g[0] for g in GRIDS]
SZ = [g[1] for g in GRIDS]
