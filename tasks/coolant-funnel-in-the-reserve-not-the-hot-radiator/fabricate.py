"""Print files for the real-trial rig, from the same dimensions as the scene
(coolant_funnel_in_the_reserve_not_the_hot_radiator.py), in mm. The cap, neck, tank and funnel are bought parts; these
are the custom pieces round them:

  - radiator_top_tank_a.stl, radiator_top_tank_b.stl: the black top tank of the radiator section, 460 x 50 x 45 mm in
    two 230 mm halves; half b has a 57 mm counterbore, 6 mm deep, centred 120 mm from its outer end, that seats the
    Howe 34301 weld-on filler neck (56.6 mm OD) with epoxy. Print in black PETG; glue to a 460 x 30 x 65 mm core board.
  - recovery_tank_top.stl: printed route only (when no 46 mm pocket can be cut for the Dorman 603-001): the visible top
    of the tank, 165 x 102 x 50 mm with a 30 mm bore 38 mm deep, and its 25 mm filler neck (38 mm OD, 2 mm chamfer).
    Print in natural (off-white) PETG.
  - cap_tether.stl: the strap that ties the recovery cap to the tank: a ring that slips over the 47 mm cap skirt
    (2 mm wall, 6 mm tall), a 24.5 mm tab and a hinge knuckle for a 3 mm steel pin; and tether_clip.stl, the base
    clip that holds the pin, taped to the tank top 48 mm from the neck axis. Print both in black PETG.
  - washer_neck_block.stl: the visible top of the washer bottle, 100 x 100 x 50 mm with a 38 mm bore 42 mm deep and a
    20 mm neck (44 mm OD). Print in white PETG.

    python3 fabricate.py [out_dir]      writes the ASCII STL files (mm); nothing is written on import

Standard library only.
"""

import math
import os
import sys

SIDES = 64

# Radiator section (scene: RAD_* constants)
TANK_LEN, TANK_DEPTH, TANK_H = 460.0, 50.0, 45.0
NECK_OD, NECK_SEAT = 57.0, 6.0
NECK_FROM_END = 120.0  # scene: cap at y -0.10, radiator from y -0.22
# Recovery tank top (scene: RES_* constants)
RES_BODY = (165.0, 102.0, 50.0)
RES_BORE_R, RES_BORE_DEPTH = 15.0, 38.0
RES_NECK_R_OUT, RES_NECK_H, RES_LEAD = 19.0, 25.0, 2.0
RES_NECK_AT = (85.0 - 2.5, 51.0 + 9.0)  # neck axis from the block's -x, -y corner (scene: neck at x 0.250, y 0.080)
# Cap tether (scene: RES_STRAP, RES_CAP_R_OUT)
CAP_SKIRT_R = 23.5
STRAP = 48.0
TETHER_RING_H, TETHER_WALL = 6.0, 2.0
PIN_R = 1.6
# Washer neck block (scene: WASH_* constants)
WASH_BODY = (100.0, 100.0, 50.0)
WASH_BORE_R, WASH_BORE_DEPTH = 19.0, 42.0
WASH_NECK_R_OUT, WASH_NECK_H = 22.0, 20.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _ring(cx, cy, r_in, r_out, z0, z1, sides=SIDES):
    """Closed annulus (or solid disc when r_in is 0) between z0 and z1, centred at (cx, cy)."""
    tris = []
    for i in range(sides):
        a0, a1 = 2 * math.pi * i / sides, 2 * math.pi * (i + 1) / sides
        o0 = (cx + r_out * math.cos(a0), cy + r_out * math.sin(a0))
        o1 = (cx + r_out * math.cos(a1), cy + r_out * math.sin(a1))
        i0 = (cx + r_in * math.cos(a0), cy + r_in * math.sin(a0))
        i1 = (cx + r_in * math.cos(a1), cy + r_in * math.sin(a1))
        c = (cx, cy)
        tris += [((*o0, z0), (*o1, z1), (*o1, z0)), ((*o0, z0), (*o0, z1), (*o1, z1))]
        if r_in > 0:
            tris += [((*o0, z1), (*i0, z1), (*i1, z1)), ((*o0, z1), (*i1, z1), (*o1, z1))]
            tris += [((*o0, z0), (*i1, z0), (*i0, z0)), ((*o0, z0), (*o1, z0), (*i1, z0))]
            tris += [((*i0, z0), (*i1, z0), (*i1, z1)), ((*i0, z0), (*i1, z1), (*i0, z1))]
        else:
            tris += [((*c, z1), (*o0, z1), (*o1, z1)), ((*c, z0), (*o1, z0), (*o0, z0))]
    return tris


def _block_with_hole(sx, sy, z0, z1, cx, cy, r, sides=SIDES):
    """A rectangular block 0..sx, 0..sy, z0..z1 with a round through hole of radius r at (cx, cy): each ray from the
    hole centre meets the hole and the outline once, so the faces pair up into a closed solid."""
    outer, inner = [], []
    for i in range(sides):
        a = 2 * math.pi * i / sides
        dx, dy = math.cos(a), math.sin(a)
        ts = []
        if dx > 1e-12:
            ts.append((sx - cx) / dx)
        if dx < -1e-12:
            ts.append(-cx / dx)
        if dy > 1e-12:
            ts.append((sy - cy) / dy)
        if dy < -1e-12:
            ts.append(-cy / dy)
        t = min(ts)
        outer.append((cx + t * dx, cy + t * dy))
        inner.append((cx + r * dx, cy + r * dy))
    corners = [(sx, sy), (0.0, sy), (0.0, 0.0), (sx, 0.0)]
    tris = []
    for i in range(sides):
        j = (i + 1) % sides
        o0, o1, i0, i1 = outer[i], outer[j], inner[i], inner[j]
        poly_top = [o0]
        a0, a1 = math.atan2(o0[1] - cy, o0[0] - cx), math.atan2(o1[1] - cy, o1[0] - cx)
        for c in corners:  # a corner between two rays belongs to this sector
            ac = math.atan2(c[1] - cy, c[0] - cx)
            span = (a1 - a0) % (2 * math.pi)
            if 1e-9 < (ac - a0) % (2 * math.pi) < span - 1e-9:
                poly_top.append(c)
        poly_top.append(o1)
        for k in range(len(poly_top) - 1):  # fan from i0/i1 over the outline pieces
            p, q = poly_top[k], poly_top[k + 1]
            tris.append(((*i0, z1), (*p, z1), (*q, z1)) if k == 0 else ((*i1, z1), (*p, z1), (*q, z1)))
            tris.append(((*i0, z0), (*q, z0), (*p, z0)) if k == 0 else ((*i1, z0), (*q, z0), (*p, z0)))
            tris += [((*p, z0), (*q, z0), (*q, z1)), ((*p, z0), (*q, z1), (*p, z1))]
        tris += [((*i0, z1), (*poly_top[1], z1), (*i1, z1)), ((*i0, z0), (*i1, z0), (*poly_top[1], z0))]
        tris += [((*i0, z0), (*i0, z1), (*i1, z1)), ((*i0, z0), (*i1, z1), (*i1, z0))]
    return tris


def parts():
    half = TANK_LEN / 2
    tank_a = _box(0.0, 0.0, 0.0, half, TANK_DEPTH, TANK_H)
    tank_b = _box(0.0, 0.0, 0.0, half, TANK_DEPTH, TANK_H - NECK_SEAT)
    tank_b += _block_with_hole(half, TANK_DEPTH, TANK_H - NECK_SEAT, TANK_H, half - NECK_FROM_END, TANK_DEPTH / 2,
                               NECK_OD / 2)
    nx, ny = RES_NECK_AT
    res_top = _box(0.0, 0.0, 0.0, RES_BODY[0], RES_BODY[1], RES_BODY[2] - RES_BORE_DEPTH)
    res_top += _block_with_hole(RES_BODY[0], RES_BODY[1], RES_BODY[2] - RES_BORE_DEPTH, RES_BODY[2], nx, ny, RES_BORE_R)
    z = RES_BODY[2]
    res_top += _ring(nx, ny, RES_BORE_R, RES_NECK_R_OUT, z, z + RES_NECK_H - RES_LEAD)
    res_top += _ring(nx, ny, RES_BORE_R + RES_LEAD, RES_NECK_R_OUT, z + RES_NECK_H - RES_LEAD, z + RES_NECK_H)
    tether = _ring(0.0, 0.0, CAP_SKIRT_R, CAP_SKIRT_R + TETHER_WALL, 0.0, TETHER_RING_H)
    tether += _box(-(STRAP - 2.0), -6.0, 0.0, -(CAP_SKIRT_R + TETHER_WALL) + 0.5, 6.0, 3.0)
    tether += _ring(-STRAP, 0.0, PIN_R + 0.2, PIN_R + 2.5, 0.0, 6.0, 24)  # knuckle (axis vertical in print, laid flat)
    clip = _box(-12.0, -10.0, 0.0, 12.0, 10.0, 3.0) + _ring(0.0, -7.5, PIN_R, PIN_R + 2.5, 3.0, 9.0, 24)
    clip += _ring(0.0, 7.5, PIN_R, PIN_R + 2.5, 3.0, 9.0, 24)
    w = WASH_BODY
    washer = _box(0.0, 0.0, 0.0, w[0], w[1], w[2] - WASH_BORE_DEPTH)
    washer += _block_with_hole(w[0], w[1], w[2] - WASH_BORE_DEPTH, w[2], w[0] / 2, w[1] / 2, WASH_BORE_R)
    washer += _ring(w[0] / 2, w[1] / 2, WASH_BORE_R, WASH_NECK_R_OUT, w[2], w[2] + WASH_NECK_H)
    return {
        "radiator_top_tank_a": tank_a,
        "radiator_top_tank_b": tank_b,
        "recovery_tank_top": res_top,
        "cap_tether": tether,
        "tether_clip": clip,
        "washer_neck_block": washer,
    }


def _write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            u = [b[i] - a[i] for i in range(3)]
            v = [c[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            ln = math.sqrt(sum(x * x for x in n)) or 1.0
            f.write(f"  facet normal {n[0] / ln:.6f} {n[1] / ln:.6f} {n[2] / ln:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, tris in parts().items():
        path = os.path.join(out_dir, name + ".stl")
        _write_stl(path, name, tris)
        print(f"wrote {path} ({len(tris)} triangles)")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
