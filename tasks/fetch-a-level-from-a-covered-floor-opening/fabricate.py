"""Build files for the two custom parts, from the same dimensions as the scene (fetch_a_level_from_a_covered_floor_opening.py),
in mm: the hole cover (plate, 2x2 handle, two chamfered 1x2 cleats) and the floor deck (3/4 in plywood on 2x3 framing,
boxed round a 200 mm opening). Both are normally cut from lumber (the cut list below); the STL files are for a printed
or CNC-cut copy and for checking a wooden build against the scene.

    python3 fabricate.py [out_dir]      writes hole_cover.stl, floor_deck.stl (ASCII STL, mm) and prints the cut list

Each file is a union of closed boxes and prisms; slicers merge overlapping shells. Standard library only; nothing is
written on import.
"""

import os
import sys

# scene dimensions (mm)
OPEN = 200.0
PLY_T, FRAME_H, FRAME_W = 19.0, 64.0, 38.0  # 3/4 in plywood; 2x3 on edge (actual 1.5 x 2.5 in)
DECK_X, DECK_Y = (120.0, 420.0), (-145.0, 410.0)  # station frame; the opening is centred at x 270, y 0
OPEN_C = (270.0, 0.0)
COVER_L, PLATE_T = 280.0, 19.0
HANDLE = (150.0, 38.0, 38.0)  # 2x2
CLEAT_L, CLEAT_W, CLEAT_T, CLEAT_OUT, CHAMFER = 180.0, 38.0, 19.0, 184.0, 8.0  # 1x2 laid flat


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _prism_x(outline, x0, x1):
    """A convex outline [(y, z), ...] (counter-clockwise seen from +x) extruded from x0 to x1."""
    n = len(outline)
    tris = []
    for k in range(1, n - 1):
        a, b, c = outline[0], outline[k], outline[k + 1]
        tris.append(((x1, *a), (x1, *b), (x1, *c)))
        tris.append(((x0, *a), (x0, *c), (x0, *b)))
    for k in range(n):
        (ay, az), (by, bz) = outline[k], outline[(k + 1) % n]
        tris += [((x0, ay, az), (x0, by, bz), (x1, by, bz)), ((x0, ay, az), (x1, by, bz), (x1, ay, az))]
    return tris


def hole_cover():
    """Origin at the plate's bottom centre; cleats below z 0, handle on top."""
    h = COVER_L / 2
    t = _box(-h, -h, 0.0, h, h, PLATE_T)
    hx, hy = HANDLE[0] / 2, HANDLE[1] / 2
    t += _box(-hx, -hy, PLATE_T, hx, hy, PLATE_T + HANDLE[2])
    o, i = CLEAT_OUT / 2, CLEAT_OUT / 2 - CLEAT_W
    plus = [(i, -CLEAT_T), (o - CHAMFER, -CLEAT_T), (o, -CLEAT_T + CHAMFER), (o, 0.0), (i, 0.0)]
    minus = [(-u, v) for u, v in reversed(plus)]
    for outline in (plus, minus):
        t += _prism_x(outline, -CLEAT_L / 2, CLEAT_L / 2)
    return t


def floor_deck():
    """Origin on the table at the station frame's origin, as in the scene."""
    ox0, ox1 = OPEN_C[0] - OPEN / 2, OPEN_C[0] + OPEN / 2
    oy0, oy1 = OPEN_C[1] - OPEN / 2, OPEN_C[1] + OPEN / 2
    z0, z1 = FRAME_H, FRAME_H + PLY_T
    t = _box(DECK_X[0], DECK_Y[0], z0, ox0, DECK_Y[1], z1) + _box(ox1, DECK_Y[0], z0, DECK_X[1], DECK_Y[1], z1)
    t += _box(ox0, DECK_Y[0], z0, ox1, oy0, z1) + _box(ox0, oy1, z0, ox1, DECK_Y[1], z1)
    t += _box(ox0 - FRAME_W, DECK_Y[0], 0.0, ox0, DECK_Y[1], FRAME_H)
    t += _box(ox1, DECK_Y[0], 0.0, ox1 + FRAME_W, DECK_Y[1], FRAME_H)
    t += _box(ox0, oy0 - FRAME_W, 0.0, ox1, oy0, FRAME_H) + _box(ox0, oy1, 0.0, ox1, oy1 + FRAME_W, FRAME_H)
    t += _box(ox0, DECK_Y[1] - FRAME_W, 0.0, ox1, DECK_Y[1], FRAME_H)
    return t


CUT_LIST = """Cut list (mm), softwood lumber and 3/4 in (19 mm) sheathing plywood:
  Cover plate     1 x 280 x 280 plywood, painted safety yellow
  Cover handle    1 x 150 of 2x2 (38 x 38), glued and screwed from below
  Cover cleats    2 x 180 of 1x2 (19 x 38) laid flat, 8 mm x 45 deg chamfer on the outer bottom edge, glued 184 mm apart
                  outside to outside, centred on the plate
  Deck framing    2 x 555 of 2x3 (38 x 64) on edge along y; 3 x 200 of 2x3 between them (two headers, one far edge)
  Deck plywood    300 x 555 sheet with a 200 x 200 opening, 50 mm from the near edge, 45 mm from the right edge"""


def _write(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            f.write("  facet normal 0 0 0\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    for name, fn in (("hole_cover", hole_cover), ("floor_deck", floor_deck)):
        _write(os.path.join(out, name + ".stl"), name, fn())
        print("wrote", os.path.join(out, name + ".stl"))
    print(CUT_LIST)
