"""Print files for the low-cost route (no transilluminator bought): a housing at the Accuris myView E3100's outer size,
its UV-blocking cover with the free-edge lip, and the two hinge blocks the friction hinges screw to. Every dimension is
the one the scene uses (close_uv_transilluminator_cover_before_switch_on.py), in mm, and comes from the E3100: 203 x 254
mm footprint, 55 mm housing, 165 x 135 mm filter glass recessed 3 mm, cover hinged on the front edge.

    python3 fabricate.py [out_dir]       writes housing.stl, cover.stl, hinge_block.stl (ASCII STL, mm)

Each file is a union of closed boxes and prisms; slicers merge overlapping shells. Standard library only; nothing is
written on import.
"""

import math
import os
import sys

# E3100 dimensions (mm), the same numbers as the scene
DEPTH, WIDTH, CORNER = 203.0, 254.0, 15.0
BODY_H = 48.0  # solid body up to the glass seat (above the 10 mm feet)
RIM_H = 7.0  # rim round the window: the glass (4 mm) sits 3 mm below its top
WIN_X, WIN_Y = 135.0, 165.0
COVER_L, COVER_W, COVER_T = 186.0, 220.0, 3.0  # 3 mm amber UV-blocking acrylic (the scene's collider is 5 mm)
SKIRT = 6.0
KNUCKLE_Y = (-100.0, 100.0)  # hinge centres from the unit's centre line
HINGE_BLOCK = (14.0, 42.0, 13.0)  # under each Southco E6 hinge leaf: rim top to the hinge axis (78 mm above the table)
ROCKER_CUT = (13.1, 19.4)  # E-Switch RA1113112R panel cut-out, width x height


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _prism(cx, cy, r, z0, z1, n=24):
    tris = []
    ring = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    for k in range(n):
        (ax, ay), (bx, by) = ring[k], ring[(k + 1) % n]
        tris += [((cx, cy, z0), (bx, by, z0), (ax, ay, z0)), ((cx, cy, z1), (ax, ay, z1), (bx, by, z1)),
                 ((ax, ay, z0), (bx, by, z0), (bx, by, z1)), ((ax, ay, z0), (bx, by, z1), (ax, ay, z1))]
    return tris


def _rounded_slab(lx, ly, r, z0, z1):
    ix, iy = lx / 2 - r, ly / 2 - r
    tris = _box(-lx / 2, -iy, z0, lx / 2, iy, z1) + _box(-ix, -ly / 2, z0, ix, ly / 2, z1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            tris += _prism(sx * ix, sy * iy, r, z0, z1)
    return tris


def housing():
    """Body (with a rocker cut-out notch marked by a 2 mm recess) and the rim frame round the glass window."""
    t = _rounded_slab(DEPTH, WIDTH, CORNER, 0.0, BODY_H)
    z0, z1 = BODY_H, BODY_H + RIM_H
    t += _box(-DEPTH / 2, -WIDTH / 2 + CORNER, z0, -WIN_X / 2, WIDTH / 2 - CORNER, z1)
    t += _box(WIN_X / 2, -WIDTH / 2 + CORNER, z0, DEPTH / 2, WIDTH / 2 - CORNER, z1)
    t += _box(-DEPTH / 2 + CORNER, -WIDTH / 2, z0, DEPTH / 2 - CORNER, -WIN_Y / 2, z1)
    t += _box(-DEPTH / 2 + CORNER, WIN_Y / 2, z0, DEPTH / 2 - CORNER, WIDTH / 2, z1)
    return t


def cover():
    """Cover plate, hinge edge at x 0, with the 6 mm lip under its free edge."""
    t = _box(0.0, -COVER_W / 2, 0.0, COVER_L, COVER_W / 2, COVER_T)
    t += _box(COVER_L - 5.0, -COVER_W / 2, -SKIRT, COVER_L, COVER_W / 2, 0.0)
    return t


def hinge_block():
    x, y, z = HINGE_BLOCK
    return _box(-x / 2, -y / 2, 0.0, x / 2, y / 2, z)


def write_stl(path, name, tris):
    with open(path, "w", encoding="ascii") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            m = math.sqrt(sum(k * k for k in n)) or 1.0
            f.write(f" facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n  outer loop\n")
            for p in (a, b, c):
                f.write(f"   vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("  endloop\n endfacet\n")
        f.write(f"endsolid {name}\n")


PARTS = {"housing": housing, "cover": cover, "hinge_block": hinge_block}

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    for name, fn in PARTS.items():
        write_stl(os.path.join(out, f"{name}.stl"), name, fn())
        print(f"wrote {name}.stl")
