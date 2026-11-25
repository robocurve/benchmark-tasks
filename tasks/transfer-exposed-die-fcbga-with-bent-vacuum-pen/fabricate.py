"""Print files for the real-trial rig of the exposed-die FCBGA transfer task (no pen is printed: the Virtual Industries
PEN-VAC V8901-A-ESD is bought).

Run:  python3 fabricate.py [out_dir]   writes ASCII STL files in mm. Nothing is written on import.

Every size comes from the scene (transfer_exposed_die_fcbga_with_bent_vacuum_pen.py), which copies the Entegris
H44-999-62C02 4 in STAT-PRO 3 x 3 waffle pack (101.6 x 101.6 x 6.5 mm, 25.32 mm pockets, 30.8 mm pitch), the AMD/Xilinx
SBG484 lidless FCBGA (19 x 19 x 2.5 mm with a 3.5 mm 45-deg pin-1 chamfer, 14 x 14 mm bare die) and an open-top 19 mm
BGA test-socket nest of the Enplas OTB-484 / Ironwood SG-BGA 19 mm class.

  waffle_pack.stl  ESD (carbon-loaded) PETG: the 3 x 3 pack, 3.2 mm floor, 3.3 mm walls, nine 25.32 mm pockets.
                   Print upright, 0.15 mm layers, 100 % walls. Use it only if a surplus H44-999-62C02 is not at hand.
  chip_dummy.stl   the SBG484 stand-in: a 19 x 19 x 1.75 mm substrate with a 3.5 mm 45-degree pin-1 corner chamfer at
                   (-x, -y) and a 14 x 14 x 0.75 mm die plate on top. Print the substrate green, glue a 14 x 14 x 0.75 mm
                   polished silicon offcut or aluminium shim on top, and add steel shim under the substrate until it
                   weighs 5.0 g (the real chip's mass).
  test_socket.stl  PEEK-look PETG nest: 38 x 38 x 10 mm, 19.5 x 19.5 mm locating pocket (0.25 mm/side clearance), 2.5 mm
                   vertical walls with a 3.65 mm 45-degree pin-1 corner key at (-x, -y), and a 45 degree lead-in funnel
                   to a 26.5 mm mouth.
  pen_cradle.stl   Slate PETG bench cradle: 110 x 40 x 3 mm base, two 10 mm V saddles 100 mm apart (90 mm clear between),
                   barrel axis 41 mm above the mat, rear stop 4 mm behind the barrel end.

Print at 0.2 mm layers, 20 % infill unless noted.
"""

import math
import os
import sys

# ---- scene dimensions, mm ------------------------------------------------------------------------------------------
PACK, POCKET, PITCH, FLOOR_T, RIB_H = 101.6, 25.32, 30.8, 3.2, 3.3
CHIP_A, SUB_T, DIE_A, DIE_T, CHAMFER = 19.0, 1.75, 14.0, 0.75, 3.5
SOCK, OPEN_HALF, MOUTH_HALF, WALL_Z0, WALL_Z1, FUN_Z0 = 38.0, 9.75, 13.25, 4.0, 10.0, 6.5
KEY_LEG = 3.65
BASE_T = 4.0  # socket base block
CRADLE_X, CRADLE_Y, CRADLE_T = 110.0, 40.0, 3.0
SADDLE_U, SADDLE_T, AXIS_H, BARREL_R = 50.0, 10.0, 41.0, 6.35
V_BOT = AXIS_H - BARREL_R * math.sqrt(2.0) - 0.3
PEN_L = 146.0
STOP_T, STOP_W, STOP_H = 4.0, 30.0, 48.0


# ---- meshers -------------------------------------------------------------------------------------------------------
def grid_solid(add, sub=()):
    """Watertight triangles of (union of add boxes) minus (union of sub boxes), on their shared breakpoint grid."""
    xs = sorted({b[0] for b in add + list(sub)} | {b[3] for b in add + list(sub)})
    ys = sorted({b[1] for b in add + list(sub)} | {b[4] for b in add + list(sub)})
    zs = sorted({b[2] for b in add + list(sub)} | {b[5] for b in add + list(sub)})

    def inside(boxes, p):
        return any(b[0] < p[0] < b[3] and b[1] < p[1] < b[4] and b[2] < p[2] < b[5] for b in boxes)

    nx, ny, nz = len(xs) - 1, len(ys) - 1, len(zs) - 1
    full = {}
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                c = ((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2, (zs[k] + zs[k + 1]) / 2)
                full[i, j, k] = inside(add, c) and not inside(sub, c)

    def f(i, j, k):
        return full.get((i, j, k), False)

    tris = []
    for (i, j, k), v in full.items():
        if not v:
            continue
        x0, x1, y0, y1, z0, z1 = xs[i], xs[i + 1], ys[j], ys[j + 1], zs[k], zs[k + 1]
        faces = [
            ((i - 1, j, k), [(x0, y0, z0), (x0, y0, z1), (x0, y1, z1), (x0, y1, z0)]),
            ((i + 1, j, k), [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)]),
            ((i, j - 1, k), [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)]),
            ((i, j + 1, k), [(x0, y1, z0), (x0, y1, z1), (x1, y1, z1), (x1, y1, z0)]),
            ((i, j, k - 1), [(x0, y0, z0), (x0, y1, z0), (x1, y1, z0), (x1, y0, z0)]),
            ((i, j, k + 1), [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]),
        ]
        for nb, q in faces:
            if not f(*nb):
                tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
    return tris


def prism_x(poly_yz, x0, x1):
    """Closed prism: a convex (y, z) polygon, counter-clockwise seen from +x, extruded from x0 to x1."""
    n = len(poly_yz)
    tris = []
    for a in range(1, n - 1):
        p, q, r = poly_yz[0], poly_yz[a], poly_yz[a + 1]
        tris.append(((x1, *p), (x1, *q), (x1, *r)))
        tris.append(((x0, *p), (x0, *r), (x0, *q)))
    for a in range(n):
        p, q = poly_yz[a], poly_yz[(a + 1) % n]
        tris += [((x0, *p), (x0, *q), (x1, *q)), ((x0, *p), (x1, *q), (x1, *p))]
    return tris



def prism_z(poly_xy, z0, z1):
    """Closed prism: a convex (x, y) polygon, counter-clockwise seen from +z, extruded from z0 to z1."""
    n = len(poly_xy)
    tris = []
    for a in range(1, n - 1):
        p, q, r = poly_xy[0], poly_xy[a], poly_xy[a + 1]
        tris.append(((*p, z1), (*q, z1), (*r, z1)))
        tris.append(((*p, z0), (*r, z0), (*q, z0)))
    for a in range(n):
        p, q = poly_xy[a], poly_xy[(a + 1) % n]
        tris += [((*p, z0), (*q, z0), (*q, z1)), ((*p, z0), (*q, z1), (*p, z1))]
    return tris


def swap_xy(tris):
    """Mirror a mesh across the plane x = y (swap the two axes) and keep it outward by reversing each triangle."""
    return [tuple((p[1], p[0], p[2]) for p in (t[0], t[2], t[1])) for t in tris]


# ---- parts ---------------------------------------------------------------------------------------------------------
def waffle_pack():
    add = [(-PACK / 2, -PACK / 2, 0.0, PACK / 2, PACK / 2, FLOOR_T)]
    border = (PACK - (2 * PITCH + POCKET)) / 2
    for c, w in ((PITCH / 2, PITCH - POCKET), (-PITCH / 2, PITCH - POCKET), (PITCH + POCKET / 2 + border / 2, border),
                 (-(PITCH + POCKET / 2 + border / 2), border)):
        add.append((c - w / 2, -PACK / 2, FLOOR_T, c + w / 2, PACK / 2, FLOOR_T + RIB_H))
        add.append((-PACK / 2, c - w / 2, FLOOR_T, PACK / 2, c + w / 2, FLOOR_T + RIB_H))
    return [grid_solid(add)]


def chip_dummy():
    h = CHIP_A / 2
    sub_poly = [(-h + CHAMFER, -h), (h, -h), (h, h), (-h, h), (-h, -h + CHAMFER)]
    die = grid_solid([(-DIE_A / 2, -DIE_A / 2, SUB_T, DIE_A / 2, DIE_A / 2, SUB_T + DIE_T)])
    return [prism_z(sub_poly, 0.0, SUB_T), die]


def test_socket():
    h = SOCK / 2
    base = grid_solid([(-h, -h, 0.0, h, h, BASE_T)])
    # walls on +-y: a (y, z) profile pushed along x; on +-x: the same profile with the axes swapped
    prof = [(OPEN_HALF, WALL_Z0), (h, WALL_Z0), (h, WALL_Z1), (MOUTH_HALF, WALL_Z1), (OPEN_HALF, FUN_Z0)]
    mirror = [(-y, z) for y, z in reversed(prof)]
    key_poly = [(-OPEN_HALF, -OPEN_HALF), (-OPEN_HALF + KEY_LEG, -OPEN_HALF), (-OPEN_HALF, -OPEN_HALF + KEY_LEG)]
    shells = [base, prism_x(prof, -h, h), prism_x(mirror, -h, h)]
    shells += [swap_xy(prism_x(prof, -h, h)), swap_xy(prism_x(mirror, -h, h)), prism_z(key_poly, WALL_Z0, FUN_Z0)]
    return shells


def pen_cradle():
    base = grid_solid([(-CRADLE_X / 2, -CRADLE_Y / 2, 0.0, CRADLE_X / 2, CRADLE_Y / 2, CRADLE_T)])
    top = V_BOT + 20.0
    left = [(-20.0, 0.0), (0.0, 0.0), (0.0, V_BOT), (-20.0, top)]
    right = [(0.0, 0.0), (20.0, 0.0), (20.0, top), (0.0, V_BOT)]
    shells = [base]
    for u in (-SADDLE_U, SADDLE_U):
        shells += [prism_x(left, u - SADDLE_T / 2, u + SADDLE_T / 2), prism_x(right, u - SADDLE_T / 2, u + SADDLE_T / 2)]
    x1 = -PEN_L / 2 - 4.0
    shells.append(grid_solid([(x1 - STOP_T, -STOP_W / 2, 0.0, x1, STOP_W / 2, STOP_H)]))
    return shells


PARTS = {"waffle_pack": waffle_pack, "chip_dummy": chip_dummy, "test_socket": test_socket, "pen_cradle": pen_cradle}


# ---- output --------------------------------------------------------------------------------------------------------
def _normal(t):
    (ax, ay, az), (bx, by, bz), (cx, cy, cz) = t
    ux, uy, uz, vx, vy, vz = bx - ax, by - ay, bz - az, cx - ax, cy - ay, cz - az
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(c * c for c in n)) or 1.0
    return tuple(c / m for c in n)


def to_print(shells):
    """Shift so the part rests on z = 0."""
    zmin = min(p[2] for s in shells for t in s for p in t)
    return [[tuple((p[0], p[1], p[2] - zmin) for p in t) for t in s] for s in shells]


def write_stl(path, name, shells):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for s in shells:
            for t in s:
                n = _normal(t)
                f.write(f"  facet normal {n[0]:.6f} {n[1]:.6f} {n[2]:.6f}\n    outer loop\n")
                for p in t:
                    f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
                f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in PARTS.items():
        path = os.path.join(out_dir, f"{name}.stl")
        write_stl(path, name, to_print(fn()))
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
