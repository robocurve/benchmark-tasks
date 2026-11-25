"""Print-ready ASCII STL files (mm) for the custom parts of the motorized-roller bay, from the same dimensions as the
task file (ext_robot_amr_payload_hitch_conveyor_modules_23.py).

    python3 fabricate.py [out_dir]

Writes, only when run as a script (nothing on import):
  side_rail.stl       x2  170 x 75 x 6 mm side rail with the 13 mm open-top hex drop-in slot and V seat (print in PETG
                          or cut from 6 mm aluminium plate; the slot outline is the part that matters)
  torque_plate.stl    x1  34 x 70 x 4 mm anti-rotation plate with the point-up hex notch (laser-cut from 4 mm SUS304
                          from the same outline, or print as a check gauge)
  plate_guide.stl     x1  guide pocket for the torque plate: two posts with front lips and a floor ledge, glued or
                          screwed to the left rail's outer face
  v_block.stl         x2  90 deg V block that stages the roller at the start
  plate_rack.stl      x1  slotted stand that holds the torque plate upright at the start
  drag_hub.stl        x1  inert-roller drag hub: slides onto the 7/16 in hex shaft at its middle (hex bore 11.4 mm
                          across flats) and carries a 27 x 3.5 mm nitrile O-ring that rubs the tube bore (35.6 mm), so
                          the tube turns on its shaft against a set drag, as the task's 0.10 N m tube drag; it stands
                          in for the motor and gearbox housing (the sim's key and sleeve) inside a real Power Moller
"""

from __future__ import annotations

import math
import os
import sys

# mm, matching the task file
HEX_AF = 11.0
HEX_R = HEX_AF / math.sqrt(3.0)
SEAT_H = 50.0
SLOT_HALF, LEAD_HALF, LEAD_D = 6.5, 11.0, 5.0
RAIL_L, RAIL_H, RAIL_T = 170.0, 75.0, 6.0
SLOT_X = 85.0  # slot centre from the rail's front end (rail x 215..385 mm, slot at x 300)
TAN30 = math.tan(math.radians(30.0))
PL_W, PL_T, PL_BOT, PL_TOP = 34.0, 4.0, -10.0, 60.0
PL_LEAD_HALF, PL_LEAD_D = 9.0, 5.0  # notch mouth flared to 18 mm over its lowest 5 mm
LEDGE_TOP, GUIDE_TOP = 28.0, 70.0
POCKET = 10.0  # pocket depth off the rail face (4 mm plate, 3 mm a side)
V_APEX, V_H, V_HX, V_T = 12.0, 28.0, 30.0, 16.0
HUB_BORE_AF = 11.4  # 7/16 in (11.11 mm) hex shaft plus 0.3 mm
HUB_PROFILE = [(0.0, 15.5), (18.0, 15.5), (18.0, 14.5), (22.5, 14.5), (22.5, 15.5), (40.0, 15.5)]  # (z, r) mm, O-ring groove


def _area(poly):
    return 0.5 * sum(poly[i][0] * poly[i - 1][1] - poly[i - 1][0] * poly[i][1] for i in range(len(poly))) * -1


def _ccw(poly):
    return poly if _area(poly) > 0 else list(reversed(poly))


def _inside(p, a, b, c):
    def s(p1, p2, p3):
        return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
    d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
    return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))


def _triangulate(poly):
    """Ear clipping for a simple polygon (CCW)."""
    idx, tris = list(range(len(poly))), []
    while len(idx) > 3:
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) <= 1e-12:
                continue
            if any(_inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError("triangulation failed")
    tris.append(tuple(idx))
    return tris


def prism(outline, depth, place):
    """Extrude a 2-D outline [(u, v)] by depth along w; place(u, v, w) gives the 3-D point. Returns triangles."""
    poly = _ccw([tuple(map(float, p)) for p in outline])
    n, tris = len(poly), []
    for a, b, c in _triangulate(poly):
        tris.append((place(*poly[a], 0.0), place(*poly[c], 0.0), place(*poly[b], 0.0)))
        tris.append((place(*poly[a], depth), place(*poly[b], depth), place(*poly[c], depth)))
    for i in range(n):
        p, q = poly[i], poly[(i + 1) % n]
        tris.append((place(*p, 0.0), place(*q, 0.0), place(*q, depth)))
        tris.append((place(*p, 0.0), place(*q, depth), place(*p, depth)))
    vol = sum(a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])
              for a, b, c in tris)
    return tris if vol > 0 else [(a, c, b) for a, b, c in tris]  # a mirrored placement turns the solid inside out


def box(x0, y0, z0, x1, y1, z1):
    return prism([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], z1 - z0, lambda u, v, w: (u, v, z0 + w))


def side_rail():
    a = SEAT_H - HEX_R
    v = a + SLOT_HALF * TAN30
    c = SLOT_X
    xz = [(0, 0), (RAIL_L, 0), (RAIL_L, RAIL_H), (c + LEAD_HALF, RAIL_H), (c + SLOT_HALF, RAIL_H - LEAD_D),
          (c + SLOT_HALF, v), (c, a), (c - SLOT_HALF, v), (c - SLOT_HALF, RAIL_H - LEAD_D), (c - LEAD_HALF, RAIL_H),
          (0, RAIL_H)]
    return prism(xz, RAIL_T, lambda x, z, y: (x, y, z))


def torque_plate():
    flank = HEX_R - SLOT_HALF * TAN30
    h = PL_W / 2
    xz = [(-h, PL_BOT), (-PL_LEAD_HALF, PL_BOT), (-SLOT_HALF, PL_BOT + PL_LEAD_D), (-SLOT_HALF, flank), (0.0, HEX_R),
          (SLOT_HALF, flank), (SLOT_HALF, PL_BOT + PL_LEAD_D), (PL_LEAD_HALF, PL_BOT), (h, PL_BOT), (h, PL_TOP),
          (-h, PL_TOP)]
    return prism(xz, PL_T, lambda x, z, y: (x, y, z - PL_BOT))


def plate_guide():
    """In rail-face coordinates: x along the rail (0 = slot centre), y off the rail face, z above the table."""
    t = []
    for s in (-1, 1):
        x0, x1 = sorted((s * 20.0, s * 28.0))
        t += box(x0, 0.0, LEDGE_TOP, x1, POCKET + 4.0, GUIDE_TOP)  # post
        x0, x1 = sorted((s * 14.0, s * 20.0))
        t += box(x0, POCKET, LEDGE_TOP, x1, POCKET + 4.0, GUIDE_TOP)  # front lip, 3 mm over the plate edge
    t += box(-28.0, 0.0, LEDGE_TOP - 8.0, 28.0, POCKET + 4.0, LEDGE_TOP)  # floor ledge
    return t


def v_block():
    top_half = V_H - V_APEX
    xz = [(-V_HX, 0), (V_HX, 0), (V_HX, V_H), (top_half, V_H), (0, V_APEX), (-top_half, V_H), (-V_HX, V_H)]
    return prism(xz, V_T, lambda x, z, y: (x, y, z))


def plate_rack():
    gap_y, gap_x = PL_T / 2 + 1.0, PL_W / 2 + 3.0
    t = box(-26.0, -15.0, 0.0, 26.0, 15.0, 4.0)
    for s in (-1, 1):
        y0, y1 = sorted((s * gap_y, s * (gap_y + 6.0)))
        t += box(-26.0, y0, 0.0, 26.0, y1, 30.0)
        x0, x1 = sorted((s * gap_x, s * (gap_x + 6.0)))
        t += box(x0, -gap_y, 0.0, x1, gap_y, 30.0)
    return t


def drag_hub(n=24):
    """A solid of revolution (HUB_PROFILE) with a straight hex bore along z, meshed directly (watertight)."""
    ap = HUB_BORE_AF / 2

    def inner(a):
        k = round((a - math.pi / 6) / (math.pi / 3))
        return ap / math.cos(a - (math.pi / 6 + k * math.pi / 3))

    ang = [2 * math.pi * i / n for i in range(n)]
    ring = [[(r * math.cos(a), r * math.sin(a), z) for a in ang] for z, r in HUB_PROFILE]
    z0, z1 = HUB_PROFILE[0][0], HUB_PROFILE[-1][0]
    i0 = [(inner(a) * math.cos(a), inner(a) * math.sin(a), z0) for a in ang]
    i1 = [(inner(a) * math.cos(a), inner(a) * math.sin(a), z1) for a in ang]
    t = []
    for i in range(n):
        j = (i + 1) % n
        for a, b in zip(ring, ring[1:]):
            t += [(a[i], a[j], b[j]), (a[i], b[j], b[i])]
        t += [(i0[i], i1[j], i0[j]), (i0[i], i1[i], i1[j])]
        o0, o1 = ring[0], ring[-1]
        t += [(o0[i], i0[j], o0[j]), (o0[i], i0[i], i0[j])]
        t += [(o1[i], o1[j], i1[j]), (o1[i], i1[j], i1[i])]
    return t


def write_stl(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            ln = math.sqrt(sum(k * k for k in n)) or 1.0
            f.write(f"  facet normal {n[0] / ln:.6f} {n[1] / ln:.6f} {n[2] / ln:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


PARTS = {"side_rail": side_rail, "torque_plate": torque_plate, "plate_guide": plate_guide, "v_block": v_block,
         "plate_rack": plate_rack, "drag_hub": drag_hub}


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in PARTS.items():
        tris = fn()
        write_stl(os.path.join(out_dir, f"{name}.stl"), name, tris)
        print(f"{name}.stl: {len(tris)} triangles")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
