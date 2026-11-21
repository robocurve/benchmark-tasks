"""Write printable STL files for the printed parts into ``cad/``, from the same numbers as the simulated scene
(``rdb_sling_guards/layout.py``; ``tools/build_assets.py`` builds the scene from them).

Run from the task folder: ``python tools/build_cad.py``. Standard library only. Units: millimetres. Each file is a set of
closed convex shells that touch; slicers union them (the spark-plug package's way).

- ``edge_guard.stl``: one edge guard (print four): a 3 in (76.2 mm) PETG angle, two 45 mm flanges 3 mm thick and a
  rounded 4.5 mm nose along the outside of the corner, a 4 mm end wall at each end (outside the 2 in web's width: a
  20 x 12 mm tab out along the outside bisector and a 20 x 22 mm lobe out from the side flange), and a 12.9 mm pocket 2.1 mm deep in each flange's inside face, two
  per flange, for a K&J Magnetics D81-N52 disc (1/2 x 1/16 in, N52) glued to the pocket's floor, 0.5 mm below the face.
- ``guard_tray.stl``: the left tray (mirror it in the slicer for the right one): a 110 x 150 mm floor 8 mm thick with a
  12 mm rim and a 21 mm block for the bottom-edge guard.
"""

import math
import struct
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdb_sling_guards import layout as L  # noqa: E402

CAD = ROOT / "cad"
SIDES = 48
POCKET_R = .00645                # 12.9 mm pocket for the 12.7 mm disc
POCKET_DEPTH = L.MAG_RECESS + L.MAGNET_T     # 2.09 mm: the disc's face 0.5 mm below the inside face
CELL = .010                      # half size of the square cell around each pocket


def outward(tri, centre):
    """Order a triangle so its normal points away from ``centre``."""
    a, b, c = tri
    n = [(b[1] - a[1]) * (c[2] - a[2]) - (b[2] - a[2]) * (c[1] - a[1]),
         (b[2] - a[2]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[2] - a[2]),
         (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])]
    m = [(a[i] + b[i] + c[i]) / 3 - centre[i] for i in range(3)]
    return (a, b, c) if sum(n[i] * m[i] for i in range(3)) >= 0 else (a, c, b), n


def prism(bottom, top):
    """Triangles of a convex prism between two polygons with the same number of points (3-D, in order)."""
    n = len(bottom)
    pts = bottom + top
    centre = [sum(p[i] for p in pts) / len(pts) for i in range(3)]
    faces = [bottom[::-1], top] + [[bottom[i], bottom[(i + 1) % n], top[(i + 1) % n], top[i]] for i in range(n)]
    tris = []
    for f in faces:
        for k in range(1, len(f) - 1):
            tri, normal = outward((f[0], f[k], f[k + 1]), centre)
            if sum(v * v for v in normal) > 1e-18:
                tris.append(tri)
    return tris


def box(x, y, z):
    """Shell of an axis-aligned box given by its (low, high) extents."""
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    bottom = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)]
    top = [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    return prism(bottom, top)


def sector(half, r, i, n):
    """The part of the square [-half, half]^2 outside a regular n-gon of radius r between the rays at k and k + 1."""
    a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n

    def to_square(a):
        c, s = math.cos(a), math.sin(a)
        t = half / max(abs(c), abs(s))
        return (t * c, t * s)

    pts = [(r * math.cos(a0), r * math.sin(a0)), (r * math.cos(a1), r * math.sin(a1)), to_square(a1)]
    for corner in ((half, half), (-half, half), (-half, -half), (half, -half)):       # square corners between the rays
        ac = math.atan2(corner[1], corner[0]) % (2 * math.pi)
        if a0 < ac < a1:
            pts.append(corner)
    pts.append(to_square(a0))
    centre = (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))
    return sorted(pts, key=lambda p: math.atan2(p[1] - centre[1], p[0] - centre[0]))


def pocket_cell(to3d, depth_total):
    """A square cell 20 x 20 mm of a flange with a round pocket in its inside face: a ring of convex prisms down to the
    pocket's floor and a solid plate behind it. ``to3d(u, v, w)``: cell coordinates (u, v across the face, w into the
    flange from its inside face) to the guard's frame."""
    shells = []
    for i in range(SIDES):
        poly = sector(CELL, POCKET_R, i, SIDES)
        shells.append(prism([to3d(u, v, 0.) for u, v in poly], [to3d(u, v, POCKET_DEPTH) for u, v in poly]))
    sq = [(-CELL, -CELL), (CELL, -CELL), (CELL, CELL), (-CELL, CELL)]
    shells.append(prism([to3d(u, v, POCKET_DEPTH) for u, v in sq], [to3d(u, v, depth_total) for u, v in sq]))
    return shells


def edge_guard():
    """In the guard's own frame (metres): inside corner on the x axis, the 'face' flange inside face on z = 0 reaching
    along -y, the 'side' flange inside face on y = 0 reaching along -z (as it sits on a top-left edge)."""
    T, F, H = L.GUARD_T, L.GUARD_FL, L.GUARD_LEN / 2
    xs = sorted({-H, H} | {m[0] + s * CELL for m in L.MAG_POS for s in (-1, 1)})
    cells = [(m[0] - CELL, m[0] + CELL) for m in L.MAG_POS]
    out_c = L.MAG_POS[0][1]
    shells = []
    # face flange: y from -F to T, z from 0 to T; pockets centred at y = -out_c
    shells.append(box((-H, H), (-F, -out_c - CELL), (0., T)))
    shells.append(box((-H, H), (-out_c + CELL, T), (0., T)))
    for x0, x1 in zip(xs[:-1], xs[1:]):
        if (x0, x1) in cells:
            xm = (x0 + x1) / 2
            shells += pocket_cell(lambda u, v, w, xm=xm: (xm + u, -out_c + v, w), T)
        else:
            shells.append(box((x0, x1), (-out_c - CELL, -out_c + CELL), (0., T)))
    # side flange: y from 0 to T, z from -F to 0; pockets centred at z = -out_c
    shells.append(box((-H, H), (0., T), (-F, -out_c - CELL)))
    shells.append(box((-H, H), (0., T), (-out_c + CELL, 0.)))
    for x0, x1 in zip(xs[:-1], xs[1:]):
        if (x0, x1) in cells:
            xm = (x0 + x1) / 2
            shells += pocket_cell(lambda u, v, w, xm=xm: (xm + u, w, -out_c + v), T)
        else:
            shells.append(box((x0, x1), (0., T), (-out_c - CELL, -out_c + CELL)))
    # the nose: a round bar of radius R along the outside of the corner, tangent to both inside-face planes
    R = L.GUARD_NOSE_R
    ring_pts = [(R + R * math.cos(2 * math.pi * k / SIDES), R + R * math.sin(2 * math.pi * k / SIDES)) for k in range(SIDES)]
    shells.append(prism([(-H, y, z) for y, z in ring_pts], [(H, y, z) for y, z in ring_pts]))
    # a grip tab across each end, out along the outside bisector, outside the web's width
    b, c = (0., 1 / math.sqrt(2), 1 / math.sqrt(2)), (0., -1 / math.sqrt(2), 1 / math.sqrt(2))
    quad = [(d, w) for d, w in ((L.TAB_FROM, -L.TAB_W / 2), (L.TAB_TO, -L.TAB_W / 2), (L.TAB_TO, L.TAB_W / 2), (L.TAB_FROM, L.TAB_W / 2))]
    yz = [(d * b[1] + w * c[1], d * b[2] + w * c[2]) for d, w in quad]
    for x0, x1 in ((-H, -H + L.TAB_T), (H - L.TAB_T, H)):
        shells.append(prism([(x0, y, z) for y, z in yz], [(x1, y, z) for y, z in yz]))
        shells.append(box((x0, x1), (0., L.LOBE_OUT), (-L.LOBE_DOWN, 0.)))       # the lobe out from the side flange
    return shells


def tray():
    """The left tray (print one mirrored for the right): the floor, the rim, and the block the bottom-edge guard sits on."""
    from rdb_sling_guards import fixed_scene as F

    sx, sy, floor = L.TRAY_SIZE
    wall, rim = L.TRAY_WALL
    yc = L.TRAY_XY["left"][1]
    shells = [box((-sx / 2, sx / 2), (-sy / 2, sy / 2), (0., floor))]
    shells.append(box((-sx / 2, sx / 2), (-sy / 2, -sy / 2 + wall), (floor, floor + rim)))
    shells.append(box((-sx / 2, sx / 2), (sy / 2 - wall, sy / 2), (floor, floor + rim)))
    shells.append(box((-sx / 2, -sx / 2 + wall), (-sy / 2 + wall, sy / 2 - wall), (floor, floor + rim)))
    shells.append(box((sx / 2 - wall, sx / 2), (-sy / 2 + wall, sy / 2 - wall), (floor, floor + rim)))
    (cx, cy, cz), (hx, hy, hz) = F.tray_block("left")
    shells.append(box((-hx, hx), (cy - yc - hy, cy - yc + hy), (cz - hz, cz + hz)))
    return shells


def shell_volume(shells):
    """Volume (m^3) of closed outward-oriented shells (divergence theorem)."""
    v = 0.
    for shell in shells:
        for a, b, c in shell:
            v += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6
    return v


def write_stl(name, shells, *, scale=1000.):
    tris = [tuple(tuple(scale * v for v in p) for p in t) for shell in shells for t in shell]
    low = [min(p[i] for t in tris for p in t) for i in range(3)]
    tris = [tuple((x - low[0], y - low[1], z - low[2]) for x, y, z in t) for t in tris]
    data = bytearray(b"printed part, millimetres".ljust(80, b" ")) + struct.pack("<I", len(tris))
    for a, b, c in tris:
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
        length = math.sqrt(sum(x * x for x in n)) or 1.
        data += struct.pack("<12fH", *(x / length for x in n), *a, *b, *c, 0)
    CAD.mkdir(exist_ok=True)
    (CAD / name).write_bytes(bytes(data))
    size = [max(p[i] for t in tris for p in t) for i in range(3)]
    return name, len(tris), [round(v, 1) for v in size]


def main():
    guard = edge_guard()
    out = [write_stl("edge_guard.stl", guard), write_stl("guard_tray.stl", tray())]
    for name, n, size in out:
        print(f"{name}: {n} triangles, {size[0]} x {size[1]} x {size[2]} mm")
    v = shell_volume(guard)
    print(f"edge guard: {1e6 * v:.2f} cm^3 of PETG (shells overlap at the nose: an upper bound), "
          f"{1000 * (L.PETG_DENSITY * v + 4 * L.MAGNET_MASS):.1f} g with four discs; layout.GUARD_MASS "
          f"{1000 * L.GUARD_MASS:.1f} g")


if __name__ == "__main__":
    main()
