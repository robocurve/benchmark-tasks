"""Print files for the custom parts of this task: the cam lever's thumb cap, the nonabrasive stand-in wheel and the
guard-angle gauge.

The mechanism is stock: a Makita XAG03 grinder and its Makita 123144-0 tool-less guard, whose band is locked by a
hinged cam lever (Makita XAG03 manual, page 9, "For tool with clamp lever type wheel guard", figures 015084 and
015085). The printed parts only make it reachable, safe and quick to score:

- lever_cap.stl: a thumb cap pushed onto the end of the 123144-0 guard's cam lever. With the grinder upside down the
  lever lies under the wheel's plane; the cap gives it an upright tab 10 x 14 mm, 40 mm tall, standing beside the
  wheel's edge and reaching 17.5 mm above its face, so a gripper coming down from above can push the lever open and
  shut. The lever passes through a slot 8.6 x 6.6 mm (the scene's 8 x 6 mm lever end plus 0.3 mm a side); if your
  lever's end measures differently, set LEVER_SLOT and print again, and bond it with a strip of 3M VHB 4910 tape.
  Print in red PETG, slot horizontal, no supports needed for an 8.6 mm bridge. It adds 4 g at the lever's tip; the
  lever's travel (35 degrees), pivot and over-centre action are the stock guard's.
- stand_in_wheel.stl: reproduces the size of the Makita 741423-B (4-1/2 x 1/4 x 7/8 in, 24 grit general purpose
  Type 27 grinding wheel): 115 mm outside diameter, 6 mm thick, 22.23 mm bore. It is a flat disc (the real wheel's
  depressed centre is not reproduced), mounted on the tool's own inner flange and lock nut like a flat wheel, in brown
  PETG (about 75 g, 100 percent infill). Smooth plastic, no grit: it cannot abrade the pads.
- q_gauge.stl: makes the score quick to read. After the run, set it on the wheel lock nut (its cup fits over the 38 mm
  nut), turn its tall fin to point along the motor housing, and read where the guard's orange tape falls against the
  20 and 60 degree fins. It sits inside the skirt (55 mm radius against the skirt's 62 mm), clear of the wheel.

    python3 fabricate.py [out_dir]      writes the three STL files (ASCII, millimetres) into out_dir (default ./stl)

Standard library only; importing this file writes nothing. Dimensions match the scene in
set_an_angle_grinder_guard_toward_the_operator_and_lock_its_band_clamp.py (wheel 115 x 6 x 22.23 mm, nut radius
19 mm, 8 mm tall; wheel top 83 mm, skirt inner radius 62 mm, lever 8 x 6 mm at 62 to 68 mm and its tab 10 x 14 mm
from 60.5 to 100.5 mm above the table with the grinder upside down).
"""

import math
import os
import sys

# Stand-in wheel (mm), the scene's tube(0.0111, 0.0575, 0.006).
WHEEL_OD, WHEEL_T, WHEEL_BORE = 115.0, 6.0, 22.23

# Lever thumb cap (mm): the scene's tab box (TAB_ALONG x TAB_W, TAB_Z0..TAB_TOP) round its lever box (8 x 6 mm).
TAB_ALONG, TAB_W, TAB_H = 10.0, 14.0, 40.0
LEVER_SLOT = (8.6, 6.6)            # slot width and height for the lever's end
SLOT_Z0 = 62.0 - 0.3 - 60.5        # slot floor above the cap's base (lever bottom 62 mm, cap base 60.5 mm)

# Gauge dimensions (mm).
NUT_R, NUT_H = 19.0, 8.0          # wheel lock nut it sits on
CUP_RIN, CUP_ROUT, CUP_DEPTH = 19.5, 23.0, 6.0
DISC_R, DISC_T = 55.0, 3.0         # under the skirt's 62 mm inner radius
FIN_R0, FIN_R1, FIN_W, FIN_H = 40.0, 54.0, 2.0, 6.0
POINTER_H = 14.0                   # the 0 fin, sighted along the motor housing
MARKS_DEG = (0.0, 20.0, -20.0, 60.0, -60.0)
SIDES = 96


def _ring(r, z, n=SIDES):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z) for k in range(n)]


def prism_shell(outline_bottom, outline_top):
    """A closed prism from two matching convex outlines (counter-clockwise from above). Returns triangles."""
    n = len(outline_bottom)
    tris = []
    cb = tuple(sum(p[i] for p in outline_bottom) / n for i in range(3))
    ct = tuple(sum(p[i] for p in outline_top) / n for i in range(3))
    for k in range(n):
        a, b = outline_bottom[k], outline_bottom[(k + 1) % n]
        c, d = outline_top[k], outline_top[(k + 1) % n]
        tris += [(cb, b, a), (ct, c, d), (a, b, d), (a, d, c)]
    return tris


def tube_shell(r_in, r_out, z0, z1, n=SIDES):
    """A closed annular prism (a cup wall)."""
    ib, ob, it, ot = _ring(r_in, z0, n), _ring(r_out, z0, n), _ring(r_in, z1, n), _ring(r_out, z1, n)
    tris = []
    for k in range(n):
        j = (k + 1) % n
        tris += [(ob[k], ob[j], ot[j]), (ob[k], ot[j], ot[k])]          # outer wall
        tris += [(ib[k], it[j], ib[j]), (ib[k], it[k], it[j])]          # inner wall
        tris += [(it[k], ot[k], ot[j]), (it[k], ot[j], it[j])]          # top
        tris += [(ib[k], ob[j], ob[k]), (ib[k], ib[j], ob[j])]          # bottom
    return tris


def fin_shell(angle_deg, r0, r1, w, z0, z1):
    a = math.radians(angle_deg)
    u, v = (math.cos(a), math.sin(a)), (-math.sin(a), math.cos(a))
    corners = [(r0, -w / 2), (r1, -w / 2), (r1, w / 2), (r0, w / 2)]
    pts = [(p * u[0] + q * v[0], p * u[1] + q * v[1]) for p, q in corners]
    return prism_shell([(x, y, z0) for x, y in pts], [(x, y, z1) for x, y in pts])


def gauge_shells():
    """The gauge as closed shells, z = 0 at the cup's open rim (it hangs over the nut from there)."""
    disc_z0 = CUP_DEPTH
    shells = [
        tube_shell(CUP_RIN, CUP_ROUT, 0.0, CUP_DEPTH + 0.5),                                  # cup over the nut
        prism_shell(_ring(DISC_R, disc_z0), _ring(DISC_R, disc_z0 + DISC_T)),                  # disc (cup floor)
    ]
    for m in MARKS_DEG:
        h = POINTER_H if m == 0.0 else FIN_H
        shells.append(fin_shell(m, FIN_R0, FIN_R1, FIN_W, disc_z0 + DISC_T - 0.5, disc_z0 + DISC_T + h))
    return shells


def check_closed(tris):
    """Every directed edge appears once and its reverse once: a closed, consistently wound shell."""
    edges = {}
    for t in tris:
        for i in range(3):
            e = (tuple(round(c, 6) for c in t[i]), tuple(round(c, 6) for c in t[(i + 1) % 3]))
            edges[e] = edges.get(e, 0) + 1
    return all(n == 1 and edges.get((b, a)) == 1 for (a, b), n in edges.items())


def signed_volume(tris):
    return sum((a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0])
                + a[2] * (b[0] * c[1] - b[1] * c[0])) for a, b, c in tris) / 6.0


def write_stl(path, name, shells):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for tris in shells:
            for a, b, c in tris:
                ux, uy, uz = (b[i] - a[i] for i in range(3))
                vx, vy, vz = (c[i] - a[i] for i in range(3))
                nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
                ln = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
                f.write(f"  facet normal {nx / ln:.6e} {ny / ln:.6e} {nz / ln:.6e}\n    outer loop\n")
                for p in (a, b, c):
                    f.write(f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
                f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def box_shell(x0, x1, y0, y1, z0, z1):
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return prism_shell([(x, y, z0) for x, y in pts], [(x, y, z1) for x, y in pts])


def lever_cap_shells():
    """The cap as four overlapping closed boxes round a through slot along x (the lever's length), z = 0 at its base."""
    sw, sh = LEVER_SLOT
    a, w = TAB_ALONG, TAB_W
    z_lo, z_hi = SLOT_Z0, SLOT_Z0 + sh
    e = 0.2                                                             # overlap so the boxes fuse
    return [
        box_shell(0.0, a, -w / 2, w / 2, 0.0, z_lo),                    # floor under the slot
        box_shell(0.0, a, -w / 2, w / 2, z_hi, TAB_H),                  # the tab above the slot
        box_shell(0.0, a, -w / 2, -sw / 2, z_lo - e, z_hi + e),         # side wall
        box_shell(0.0, a, sw / 2, w / 2, z_lo - e, z_hi + e),           # side wall
    ]


def wheel_shells():
    """The stand-in wheel: one closed annular disc, z = 0 on the flange face."""
    return [tube_shell(WHEEL_BORE / 2, WHEEL_OD / 2, 0.0, WHEEL_T, n=192)]


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    paths = []
    for name, shells, what in (
            ("lever_cap", lever_cap_shells(), f"{TAB_ALONG:.0f} x {TAB_W:.0f} x {TAB_H:.0f} mm, "
                                              f"slot {LEVER_SLOT[0]} x {LEVER_SLOT[1]} mm"),
            ("stand_in_wheel", wheel_shells(), f"{WHEEL_OD:.0f} mm across, {WHEEL_T:.0f} mm thick, "
                                               f"{WHEEL_BORE:.2f} mm bore"),
            ("q_gauge", gauge_shells(), f"{2 * DISC_R:.0f} mm across, {CUP_DEPTH + DISC_T + POINTER_H:.0f} mm tall")):
        for s in shells:
            if not check_closed(s) or signed_volume(s) <= 0:
                raise SystemExit(f"{name}: a shell is open or inside out")
        path = os.path.join(out_dir, name + ".stl")
        write_stl(path, name, shells)
        print(f"wrote {path}: {len(shells)} closed shell(s), {what}")
        paths.append(path)
    return paths


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
