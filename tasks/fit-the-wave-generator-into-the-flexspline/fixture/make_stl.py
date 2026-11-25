"""Write the printable fixture parts as STL (mm): cradle.stl and handle.stl.

Same dimensions as cradle.scad. Each part is a stack of straight extrusions of 2-D outlines (with holes); the stacked
layers share only boundary edges; internal horizontal faces are omitted, giving one watertight solid. Needs numpy and shapely 2.1 or later:

    python make_stl.py
"""
from __future__ import annotations

import math
import struct
from pathlib import Path

import numpy as np
import shapely
from shapely.geometry import Point, box
from shapely.geometry.polygon import orient

HERE = Path(__file__).resolve().parent
SEG = 96  # facets on a full circle


def circle(d, c=(0.0, 0.0), n=SEG):
    return Point(c).buffer(d / 2, quad_segs=n // 4)


def cap(outline, z, up=True):
    """Triangulate only exposed horizontal material (holes are preserved)."""
    if outline.is_empty:
        return []
    tris = []
    for t in shapely.constrained_delaunay_triangles(outline).geoms:
        p = np.asarray(orient(t, 1.0).exterior.coords)[:3]
        tris.append([(x, y, z) for x, y in (p if up else p[::-1])])
    return tris


def stacked(layers):
    """One manifold boundary for stacked, touching polygon extrusions."""
    tris = cap(layers[0][0], layers[0][1], up=False)
    for i, (outline, z0, z1) in enumerate(layers):
        outline = orient(outline, 1.0)
        for ring in [outline.exterior] + list(outline.interiors):
            pts = np.asarray(ring.coords)
            for a, b in zip(pts[:-1], pts[1:]):
                tris.append([(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1)])
                tris.append([(a[0], a[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)])
        if i + 1 < len(layers):
            above = layers[i + 1][0]
            tris += cap(outline.difference(above), z1, up=True)
            tris += cap(above.difference(outline), z1, up=False)
        else:
            tris += cap(outline, z1, up=True)
    return np.asarray(tris, float)


def verify(tris):
    """Every boundary edge has two opposite uses; report the enclosed volume."""
    from collections import Counter
    vertices = np.round(tris, 6)
    edges = Counter()
    for tri in vertices:
        for a, b in zip(tri, np.roll(tri, -1, axis=0)):
            edges[(tuple(a), tuple(b))] += 1
    if any(count != 1 or edges[(b, a)] != 1 for (a, b), count in edges.items()):
        raise ValueError('STL boundary is not manifold with consistently oriented faces')
    volume = float(np.einsum('ij,ij->i', tris[:, 0], np.cross(tris[:, 1], tris[:, 2])).sum() / 6)
    if volume <= 0:
        raise ValueError('non-positive solid volume')
    return volume


def write_stl(path, tris, name):
    with open(path, 'wb') as f:
        f.write(name.encode()[:80].ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1.0)
            f.write(struct.pack('<12fH', *n, *t[0], *t[1], *t[2], 0))


# ---- cradle: base plate and riser, printed in one piece (PLA) -----------------------------------------------------
PLATE = (120.0, 120.0, 10.0)
RISER_OD, RISER_BORE, RISER_TOP = 84.0, 56.5, 53.0  # bore centres the 56 h7 output pilot; flange seat 53 up
BOLT_PCD, TAP_D, TAP_DEPTH, N_BOLTS = 65.0, 3.3, 12.0, 8  # M4 tapped holes under the flange's 8 x 4.5 holes


def cradle():
    plate = box(-PLATE[0] / 2, -PLATE[1] / 2, PLATE[0] / 2, PLATE[1] / 2).difference(circle(RISER_BORE))
    lower = circle(RISER_OD).difference(circle(RISER_BORE))
    upper = lower
    for k in range(N_BOLTS):
        a = 2 * math.pi * (k + 0.5) / N_BOLTS
        upper = upper.difference(circle(TAP_D, (BOLT_PCD / 2 * math.cos(a), BOLT_PCD / 2 * math.sin(a)), 32))
    return stacked([(plate, 0.0, PLATE[2]),
                    (lower, PLATE[2], RISER_TOP - TAP_DEPTH),
                    (upper, RISER_TOP - TAP_DEPTH, RISER_TOP)])


# ---- handle: finger-stop disc and grip knob, glued on the 8 mm shaft (PETG) ---------------------------------------
DISC_D, DISC_H, KNOB_D, KNOB_H, SHAFT_BORE = 30.0, 3.0, 20.0, 16.0, 7.9  # 7.9 bore: press fit on the 8 h6 shaft


def handle():
    disc = circle(DISC_D).difference(circle(SHAFT_BORE, n=48))
    knob = box(-KNOB_D / 2, -KNOB_D / 2, KNOB_D / 2, KNOB_D / 2).difference(circle(SHAFT_BORE, n=48))
    return stacked([(disc, 0.0, DISC_H), (knob, DISC_H, DISC_H + KNOB_H)])


if __name__ == '__main__':
    for name, tris in (('cradle', cradle()), ('handle', handle())):
        volume = verify(tris)
        out = HERE / f'{name}.stl'
        write_stl(out, tris, f'{name} (mm)')
        lo, hi = tris.reshape(-1, 3).min(0), tris.reshape(-1, 3).max(0)
        print(out.name, len(tris), 'triangles; manifold boundary; volume cm3', round(volume / 1000, 3),
              'extent', np.round(lo, 2).tolist(), np.round(hi, 2).tolist())
