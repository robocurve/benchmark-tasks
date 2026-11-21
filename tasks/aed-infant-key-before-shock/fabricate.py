"""Print files for the real-trial stand-in AED (no defibrillator, no energy) and its key holder.

Run:  python3 fabricate.py [out_dir]   writes ASCII STL files in mm. Nothing is written on import.

Every size comes from the scene (aed_infant_key_before_shock.py), which copies the Philips HeartStart FRx 861304
(180 x 140 x 60 mm body, face 75 mm above the table in its case bottom; Infant/Child Key receptacle 10 x 30 mm,
24 mm deep, 20 mm from the top end; shock button 28 mm) and the Philips FRx Infant/Child Key 989803139311
(13 x 44 x 46 mm paddle with chamfered top corners, 4 x 24 x 20 mm blade).

  frx_panel.stl    grey PETG: the FRx body as a 3 mm shell, 180 x 140 x 60 mm, open underneath. Its face carries the
                   key receptacle (a 10 x 30 mm sleeve, 24 mm deep, 2 mm walls, floor with a 5 x 12 mm slot for the
                   microswitch lever), a 30 mm hole for the illuminated shock button and two 5 mm holes for the mode
                   lamps (amber adult at y -40 mm, blue infant/child at y +40 mm, beside the receptacle).
  case_bottom.stl  grey PETG (navy if you have it): the carry case bottom the FRx lies in, 200 x 160 x 15 mm, 2 mm shell, open underneath.
  case_lid.stl     grey PETG (navy if you have it): the open case lid, 200 x 130 x 12 mm, 2 mm shell, open underneath.
  key_holder.stl   grey PETG: 25 x 56 x 22 mm block with a 10 x 30 mm blade slot, 3 mm walls 12 mm tall round a
                   19 x 50 mm recess. Stands on the lid; it only keeps the key upright where the real case keeps it.
  key.stl          pink PETG: the Infant/Child Key. Print flat on a 44 x 46 mm face with 2 walls and about 15 %
                   gyroid infill, then weigh it: the target is 16 g, the real key's mass (solid it would be 35 g),
                   so change the infill by 5 % steps until it is 15 to 17 g. Paint a white teddy bear on each face.

Print at 0.2 mm layers; panel, case and holder at 20 % infill.
"""

import math
import os
import sys

# ---- scene dimensions, mm ------------------------------------------------------------------------------------------
HX, HY, BODY_H = 90.0, 70.0, 60.0  # FRx body half sizes and height (face 75 mm above the table on the 15 mm case)
SHELL = 3.0
SLOT_X, SLOT_HX, SLOT_HY, SLOT_D = 70.0, 5.0, 15.0, 24.0  # key receptacle
SLEEVE = 2.0
FLOOR = 2.0
LEVER_HX, LEVER_HY = 2.5, 6.0  # 5 x 12 mm slot in the receptacle floor for the microswitch lever
BTN_X, BTN_HOLE_R = -20.0, 15.0  # 30 mm panel hole for the 28 mm class illuminated button (scene well 31 mm)
LAMP_X, LAMP_Y, LAMP_HOLE_R = 70.0, 40.0, 2.5  # 5 mm holes: 5 mm LEDs pushed in from behind, flange inside
TRAY = (200.0, 160.0, 15.0)
LID = (200.0, 130.0, 12.0)
CASE_WALL = 2.0  # the case parts carry no mechanism, so thinner walls save filament
# key holder (scene: SLOT 10 x 30, outer 25 x 56, block 22 tall, recess walls 3 mm x 12 mm round 19 x 50)
H_OX, H_OY, H_BLOCK, H_WALL, H_WALL_H, H_IN_HX, H_IN_HY = 12.5, 28.0, 22.0, 3.0, 12.0, 9.5, 25.0
# key (scene: PAD 13 x 44 x 46 with the top corners chamfered from 36 mm to a 28 mm top, BLADE 4 x 24 x 20)
PAD_T, PAD_W, PAD_H, PAD_SHOULDER, PAD_TOP_W = 13.0, 44.0, 46.0, 36.0, 28.0
BLADE_T, BLADE_W, BLADE_L = 4.0, 24.0, 20.0


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


def square_with_hole(cx, cy, z0, z1, half, r, n=48):
    """Closed tube shell: a square plate (half size `half`) with a round hole of radius r, as rings joined by quads."""
    pts_o, pts_i = [], []
    for a in range(n):
        t = 2 * math.pi * a / n
        c, s = math.cos(t), math.sin(t)
        k = half / max(abs(c), abs(s))
        pts_o.append((cx + k * c, cy + k * s))
        pts_i.append((cx + r * c, cy + r * s))
    tris = []
    for a in range(n):
        b = (a + 1) % n
        o0, o1, i0, i1 = pts_o[a], pts_o[b], pts_i[a], pts_i[b]
        tris += [((*o0, z0), (*o1, z0), (*o1, z1)), ((*o0, z0), (*o1, z1), (*o0, z1))]  # outer, facing out
        tris += [((*i0, z0), (*i1, z1), (*i1, z0)), ((*i0, z0), (*i0, z1), (*i1, z1))]  # inner, facing the hole
        tris += [((*o0, z1), (*o1, z1), (*i1, z1)), ((*o0, z1), (*i1, z1), (*i0, z1))]  # top
        tris += [((*o0, z0), (*i1, z0), (*o1, z0)), ((*o0, z0), (*i0, z0), (*i1, z0))]  # bottom
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


def shell_box(sx, sy, sz, wall=SHELL):
    """Open-bottom box: walls and top `wall` thick, outer size sx x sy x sz, base centre at the origin."""
    ox, oy = sx / 2, sy / 2
    return [(-ox, -oy, 0.0, ox, oy, sz)], [(-ox + wall, -oy + wall, 0.0, ox - wall, oy - wall, sz - wall)]


# ---- parts ---------------------------------------------------------------------------------------------------------
def frx_panel():
    add, sub = shell_box(2 * HX, 2 * HY, BODY_H)
    top = BODY_H
    # receptacle sleeve with its floor, standing down from the face
    add.append((SLOT_X - SLOT_HX - SLEEVE, -SLOT_HY - SLEEVE, top - SLOT_D - FLOOR,
                SLOT_X + SLOT_HX + SLEEVE, SLOT_HY + SLEEVE, top))
    sub.append((SLOT_X - SLOT_HX, -SLOT_HY, top - SLOT_D, SLOT_X + SLOT_HX, SLOT_HY, top + 1.0))
    sub.append((SLOT_X - LEVER_HX, -LEVER_HY, top - SLOT_D - FLOOR - 1.0, SLOT_X + LEVER_HX, LEVER_HY, top))
    # square openings in the face, filled by the round-hole plates below (0.5 mm overlap so the slicer unions them)
    rings = [(BTN_X, 0.0, 20.0, BTN_HOLE_R), (LAMP_X, -LAMP_Y, 9.0, LAMP_HOLE_R), (LAMP_X, LAMP_Y, 9.0, LAMP_HOLE_R)]
    for cx, cy, half, _ in rings:
        sub.append((cx - half + 0.5, cy - half + 0.5, top - SHELL - 1.0, cx + half - 0.5, cy + half - 0.5, top + 1.0))
    shells = [grid_solid(add, sub)]
    shells += [square_with_hole(cx, cy, top - SHELL, top, half, r) for cx, cy, half, r in rings]
    return shells


def case_bottom():
    return [grid_solid(*shell_box(*TRAY, wall=CASE_WALL))]


def case_lid():
    return [grid_solid(*shell_box(*LID, wall=CASE_WALL))]


def key_holder():
    s_hx, s_hy = 5.0, 15.0
    top = H_BLOCK + H_WALL_H
    add = [(-H_OX, -H_OY, 0.0, H_OX, H_OY, top)]
    sub = [(-s_hx, -s_hy, -1.0, s_hx, s_hy, top + 1.0), (-H_IN_HX, -H_IN_HY, H_BLOCK, H_IN_HX, H_IN_HY, top + 1.0)]
    return [grid_solid(add, sub)]


def key():
    hw, tw = PAD_W / 2, PAD_TOP_W / 2
    poly = [(-hw, 0.0), (hw, 0.0), (hw, PAD_SHOULDER), (tw, PAD_H), (-tw, PAD_H), (-hw, PAD_SHOULDER)]
    paddle = prism_x(poly, -PAD_T / 2, PAD_T / 2)
    blade = grid_solid([(-BLADE_T / 2, -BLADE_W / 2, -BLADE_L, BLADE_T / 2, BLADE_W / 2, 1.0)])
    return [paddle, blade]


PARTS = {"frx_panel": frx_panel, "case_bottom": case_bottom, "case_lid": case_lid, "key_holder": key_holder,
         "key": key}


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
