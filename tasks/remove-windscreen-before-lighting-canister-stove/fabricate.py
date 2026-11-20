"""Print files for the real-world preparation fixtures of remove-windscreen-before-lighting-canister-stove:
a 65-degree-per-hinge windscreen wrapping template (for repeatable <30 s reset of the Vargo T420 5-panel aluminum
windscreen around the 90 mm Jetboil JetPower 100 g canister), a valve-angle scoring card (marking 0 deg closed,
14 deg hazard threshold, 57 deg open threshold, and 90 deg full open CCW viewed from the handle end), and a
100 mm radius keep-clear ring stencil for the silicone cook mat. Every dimension is in mm and matches
remove_windscreen_before_lighting_canister_stove.py and the commercial products (Vargo T420 83 x 81 mm panels,
Jetboil JetPower JF100 90 mm diameter canister, MSR PocketRocket Deluxe 10955 valve axis).

    python3 fabricate.py [out_dir]     writes wrap_template.stl, valve_angle_card.stl, keep_clear_stencil.stl (ASCII STL, mm)

Standard library only; nothing is written on import.
"""

import math
import os
import sys

# Exact dimensions (mm) matching remove_windscreen_before_lighting_canister_stove.py
CAN_R = 45.0
PANEL_W = 83.0
WRAP_DEG = 65.0
R_APOTHEM = (PANEL_W / 2.0) / math.tan(math.radians(WRAP_DEG / 2.0))  # 65.14 mm
RING_R_IN = 98.0
RING_R_OUT = 101.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def _ring(r_in, r_out, z0, z1, n=48):
    tris = []
    for k in range(n):
        a0 = 2.0 * math.pi * k / n
        a1 = 2.0 * math.pi * (k + 1) / n
        i0 = (r_in * math.cos(a0), r_in * math.sin(a0))
        i1 = (r_in * math.cos(a1), r_in * math.sin(a1))
        o0 = (r_out * math.cos(a0), r_out * math.sin(a0))
        o1 = (r_out * math.cos(a1), r_out * math.sin(a1))
        tris += [
            ((i0[0], i0[1], z1), (o0[0], o0[1], z1), (o1[0], o1[1], z1)),
            ((i0[0], i0[1], z1), (o1[0], o1[1], z1), (i1[0], i1[1], z1)),
            ((i0[0], i0[1], z0), (o1[0], o1[1], z0), (o0[0], o0[1], z0)),
            ((i0[0], i0[1], z0), (i1[0], i1[1], z0), (o1[0], o1[1], z0)),
            ((o0[0], o0[1], z0), (o1[0], o1[1], z0), (o1[0], o1[1], z1)),
            ((o0[0], o0[1], z0), (o1[0], o1[1], z1), (o0[0], o0[1], z1)),
            ((i0[0], i0[1], z0), (i1[0], i1[1], z1), (i1[0], i1[1], z0)),
            ((i0[0], i0[1], z0), (i0[0], i0[1], z1), (i1[0], i1[1], z1)),
        ]
    return tris


def _seg_bar(p0, p1, width, z0, z1):
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ln = math.hypot(dx, dy)
    nx, ny = -dy / ln * (width / 2.0), dx / ln * (width / 2.0)
    a = (p0[0] + nx, p0[1] + ny)
    b = (p1[0] + nx, p1[1] + ny)
    c = (p1[0] - nx, p1[1] - ny)
    d = (p0[0] - nx, p0[1] - ny)
    pts = [a, b, c, d]
    tris = [
        ((a[0], a[1], z1), (b[0], b[1], z1), (c[0], c[1], z1)),
        ((a[0], a[1], z1), (c[0], c[1], z1), (d[0], d[1], z1)),
        ((a[0], a[1], z0), (c[0], c[1], z0), (b[0], b[1], z0)),
        ((a[0], a[1], z0), (d[0], d[1], z0), (c[0], c[1], z0)),
    ]
    for i in range(4):
        u, w = pts[i], pts[(i + 1) % 4]
        tris += [
            ((u[0], u[1], z0), (w[0], w[1], z0), (w[0], w[1], z1)),
            ((u[0], u[1], z0), (w[0], w[1], z1), (u[0], u[1], z1)),
        ]
    return tris


def wrap_template():
    """Flat 3 mm guide ring with 90 mm inner bore for the canister and 5 outer guide ribs at 65 deg/hinge."""
    t = _ring(CAN_R + 0.5, CAN_R + 6.0, 0.0, 3.0, n=48)
    # 5 panel chord segments around apothem R_APOTHEM (outer guide edge at R_APOTHEM + 2.5 mm)
    r_out = R_APOTHEM + 3.0
    a = math.radians(WRAP_DEG)
    verts = []
    # Start at front panel p3 centered at (-r_out, 0)
    p3_l = (-r_out, +PANEL_W / 2.0)
    p3_r = (-r_out, -PANEL_W / 2.0)
    segs = [(p3_r, p3_l)]
    for sign in (+1.0, -1.0):
        cur = p3_l if sign > 0 else p3_r
        ang = sign * (math.pi / 2.0)
        for _ in range(2):
            ang -= sign * a
            nxt = (cur[0] + PANEL_W * math.cos(ang), cur[1] + PANEL_W * math.sin(ang))
            segs.append((cur, nxt))
            cur = nxt
    for p0, p1 in segs:
        t += _seg_bar(p0, p1, 4.0, 0.0, 5.0)
    # Radial spokes tying inner canister ring to outer 5-panel template
    for deg in (180.0, 115.0, -115.0):
        th = math.radians(deg)
        t += _seg_bar(((CAN_R + 3.0) * math.cos(th), (CAN_R + 3.0) * math.sin(th)),
                      (r_out * math.cos(th), r_out * math.sin(th)), 5.0, 0.0, 3.0)
    return t


def valve_angle_card():
    """Protractor card (60 x 60 x 2 mm) with raised angle ticks at 0, 14, 57, and 90 deg CCW from vertical."""
    t = _box(-30.0, -30.0, 0.0, 30.0, 30.0, 2.0)
    for deg in (0.0, 14.3, 57.3, 90.0):
        th = math.radians(90.0 + deg)  # 0 deg vertical up (+y on card), CCW toward -x
        t += _seg_bar((8.0 * math.cos(th), 8.0 * math.sin(th)), (27.0 * math.cos(th), 27.0 * math.sin(th)),
                      1.8, 2.0, 3.2)
    return t


def keep_clear_stencil():
    """100 mm radius paint/tape ring stencil (98..101 mm ring slot between inner 98 mm and outer 106 mm rings)."""
    t = _ring(90.0, RING_R_IN, 0.0, 2.0, n=64)
    t += _ring(RING_R_OUT, 108.0, 0.0, 2.0, n=64)
    for deg in (0.0, 90.0, 180.0, 270.0):
        th = math.radians(deg)
        t += _seg_bar((91.0 * math.cos(th), 91.0 * math.sin(th)), (107.0 * math.cos(th), 107.0 * math.sin(th)),
                      3.0, 0.0, 2.0)
    return t


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


PARTS = {
    "wrap_template": wrap_template,
    "valve_angle_card": valve_angle_card,
    "keep_clear_stencil": keep_clear_stencil,
}

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out, exist_ok=True)
    for name, fn in PARTS.items():
        write_stl(os.path.join(out, f"{name}.stl"), name, fn())
        print(f"wrote {name}.stl")
