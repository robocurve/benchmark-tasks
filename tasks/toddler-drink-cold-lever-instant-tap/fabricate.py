"""Print files for the real trial's custom parts (ASCII STL, millimetres). Run: python3 fabricate.py [out_dir]

Trial aids (both routes):

- angle_card.stl: a 3 mm card that clips round the faucet body (57 mm) under the handle, with raised ridges at
  10 deg back (hot line) and 30 deg and 45 deg forward (cool lines) from off, read against the handle in the top video.
  (The bench rig below carries the same ridges on its bracket, so it needs no card.)
- outlet_target.stl: a 3 mm disc with a 30 mm hole (the 15 mm axis rule) that lies on the counter under the outlet, so
  the video shows the cup's rim around the outlet; it is lifted away before a trial starts, leaving a pencil ring.
- tray.stl: the toddler's tray, a flat 120 x 160 x 8 mm plate (print in white). The scene's 6 mm rim is look-only (the
  robot's fingers pass through it in the simulator), so it is left off the print and nothing can snag on it.

Bench rig (the cheap route: a printed stand-in for the F-HC3300 at the scene's dimensions, with its press-down lock):

- rig_body.stl: the 57 mm round body, 195 mm tall on a 64 mm base flange, with the handle's lock cap at the scene's
  pivot (38.5 mm off the axis on the +y side): a 43 mm disc 116 to 126 mm up holding a 608-2RS bearing (22 x 7 mm
  pocket, 8.6 mm hole), two lock lugs under it (16 to 75 deg each side of off, 11 to 20 mm out, 112 to 116 mm up), and
  under the handle a bracket plate 66 to 72 mm up with a 7 mm spring guide pin (to 91 mm: the 7 mm press stop), two
  stop pegs (45 deg forward, 50 deg back) and raised angle ridges at 10 deg back and 30 and 45 deg forward.
  Print upright with supports.
- rig_spout.stl: the arched spout (18 mm octagon, 133 mm reach) and aerator; its outlet is 140 mm above the counter
  when glued on top of the body (spout toward -x, the robot).
- rig_handle.stl: hub (17 mm, 92 to 108 mm up) with the lock tooth on top (4 mm wide, 6 to 18 mm out, 8 mm tall),
  a 10.5 mm spring pocket underneath and an 8.2 mm pin hole on top, and the 60 mm paddle (16 mm wide at the hub, 10 mm
  at the tip). Mark the top of its back half red and its front half blue with tape.
- rig_pin.stl: the 7.9 mm pivot pin (40 mm), pushed down through the bearing into the hub and glued there; it turns
  and slides 7 mm in the bearing.

The spring is a Lee Spring LC 026EE 10S (9.53 mm OD, 28.58 mm free, 0.38 N/mm): installed 26 mm long between the
bracket and the hub's pocket it holds the tooth up in its notch between the lugs with about 1 N and takes about 3.6 N
at the 7 mm stop. Up, the tooth meets a lug within about 6 deg either way; pressed 4 mm it clears the lugs and the
handle swings; once swung out, the spring holds the tooth against the lugs' undersides until the handle comes back to
off and pops up into the notch.

Dimensions are the scene's: body radius 28.5 mm, handle pivot 38.5 mm off the body axis and 100 mm up, handle 60 mm
long, outlet 133 mm out and 140 mm up.
"""

import math
import os
import sys

BODY_R = 28.5
PIVOT_Y = 38.5
CARD_R = 75.0
T = 3.0
TRAY_X, TRAY_Y, TRAY_H = 120.0, 160.0, 8.0


def _box(x0, y0, z0, x1, y1, z1):
    v = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    q = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for a, b, c, d in q] + [(v[a], v[c], v[d]) for a, b, c, d in q]


def _prism(poly, z0, z1):
    """Triangles of a straight prism over a convex polygon [(x, y), ...] (counter-clockwise)."""
    tris = []
    n = len(poly)
    for i in range(1, n - 1):
        tris.append(((*poly[0], z0), (*poly[i + 1], z0), (*poly[i], z0)))
        tris.append(((*poly[0], z1), (*poly[i], z1), (*poly[i + 1], z1)))
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        tris.append(((*a, z0), (*b, z0), (*b, z1)))
        tris.append(((*a, z0), (*b, z1), (*a, z1)))
    return tris


def _sector(r0, r1, a0, a1, z0, z1, cx=0.0, cy=0.0):
    """Annular sector built from convex quads (angles from +y, positive toward -x)."""
    tris = []
    steps = max(1, int(abs(a1 - a0) / math.radians(10)))
    for i in range(steps):
        b0 = a0 + (a1 - a0) * i / steps
        b1 = a0 + (a1 - a0) * (i + 1) / steps
        pts = [(cx - r * math.sin(b), cy + r * math.cos(b)) for r, b in ((r0, b0), (r1, b0), (r1, b1), (r0, b1))]
        area = sum(pts[k][0] * pts[(k + 1) % 4][1] - pts[(k + 1) % 4][0] * pts[k][1] for k in range(4))
        tris += _prism(pts if area > 0 else pts[::-1], z0, z1)
    return tris


def angle_card():
    tris = _sector(BODY_R + 0.5, CARD_R, math.radians(-60), math.radians(60), 0.0, T, 0.0, -PIVOT_Y)
    for deg in (-10.0, 30.0, 45.0):  # ridges: hot line 10 deg back; cool lines 30 and 45 deg forward
        a = math.radians(deg)
        tris += _sector(40.0, CARD_R, a - math.radians(0.8), a + math.radians(0.8), T, T + 1.5, 0.0, -PIVOT_Y)
    return tris


def outlet_target():
    tris = []
    for i in range(36):
        a0, a1 = 2 * math.pi * i / 36, 2 * math.pi * (i + 1) / 36
        tris += _sector(15.0, 40.0, a0, a1, 0.0, T)
    return tris


def tray():
    return _box(0.0, 0.0, 0.0, TRAY_X, TRAY_Y, TRAY_H)


# ---- bench rig -------------------------------------------------------------------------------------------------------
# Body frame: origin on the body axis on the counter, z up, spout toward -x, handle pivot at (0, PIVOT_Y).
PIVOT_Z = 100.0
CAP_Z = (116.0, 126.0)  # lock cap plate; the bearing pocket is its top 7 mm
LUG_Z = (112.0, 116.0)
BRACKET_Z = (66.0, 72.0)
FLANGE_R, FLANGE_H, BODY_TOP = 32.0, 6.0, 195.0
REACH, OUTLET_Z, SPOUT_R = 133.0, 140.0, 9.0


def _cyl(cx, cy, r, z0, z1, n=32):
    poly = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
    return _prism(poly, z0, z1)


def _ring(cx, cy, r0, r1, z0, z1, a0=0.0, a1=2 * math.pi):
    return _sector(r0, r1, a0, a1, z0, z1, cx, cy)


def _rod(p0, p1, r, n=8):
    """An n-sided prism of circumradius r from p0 to p1, lengthened by r at both ends so joints overlap."""
    d = [p1[k] - p0[k] for k in range(3)]
    L = math.sqrt(sum(x * x for x in d))
    d = [x / L for x in d]
    p0 = [p0[k] - r * d[k] for k in range(3)]
    p1 = [p1[k] + r * d[k] for k in range(3)]
    a = (0.0, 1.0, 0.0) if abs(d[1]) < 0.9 else (1.0, 0.0, 0.0)
    u = [a[1] * d[2] - a[2] * d[1], a[2] * d[0] - a[0] * d[2], a[0] * d[1] - a[1] * d[0]]
    m = math.sqrt(sum(x * x for x in u))
    u = [x / m for x in u]
    v = [d[1] * u[2] - d[2] * u[1], d[2] * u[0] - d[0] * u[2], d[0] * u[1] - d[1] * u[0]]

    def ring(c):
        return [tuple(c[k] + r * (math.cos(2 * math.pi * i / n) * u[k] + math.sin(2 * math.pi * i / n) * v[k])
                      for k in range(3)) for i in range(n)]

    r0, r1 = ring(p0), ring(p1)
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(r0[i], r0[j], r1[j]), (r0[i], r1[j], r1[i]), (tuple(p1), r1[i], r1[j]), (tuple(p0), r0[j], r0[i])]
    return tris


def rig_body():
    py = PIVOT_Y
    tris = _cyl(0.0, 0.0, FLANGE_R, 0.0, FLANGE_H)
    # the round body: a 3 mm wall, with a gap in the wall where the bearing pocket meets it, and a closed top
    tris += _ring(0.0, 0.0, BODY_R - 3.0, BODY_R, FLANGE_H, CAP_Z[0])
    tris += _ring(0.0, 0.0, BODY_R - 3.0, BODY_R, CAP_Z[0], CAP_Z[1], math.radians(20), math.radians(340))
    tris += _ring(0.0, 0.0, BODY_R - 3.0, BODY_R, CAP_Z[1], BODY_TOP - 6.0)
    tris += _cyl(0.0, 0.0, BODY_R, BODY_TOP - 6.0, BODY_TOP)
    # lock cap: 8.6 mm hole under a 22 x 7 mm bearing pocket, joined to the body
    tris += _ring(0.0, py, 4.3, 21.5, CAP_Z[0], CAP_Z[0] + 3.0)
    tris += _ring(0.0, py, 11.05, 21.5, CAP_Z[0] + 3.0, CAP_Z[1])
    tris += _box(-10.0, 18.0, CAP_Z[0], 10.0, py - 15.0, CAP_Z[1])
    # lock lugs, each side of the notch at off
    for sign in (1.0, -1.0):
        a0, a1 = sorted((sign * math.radians(16.0), sign * math.radians(75.0)))
        tris += _sector(11.0, 20.0, a0, a1, LUG_Z[0], LUG_Z[1] + 0.5, 0.0, py)
    # bracket under the handle: plate, spring guide pin (its top is the 7 mm press stop), stop pegs, angle ridges
    tris += _sector(0.0001, 75.0, math.radians(-75.0), math.radians(75.0), *BRACKET_Z, 0.0, py)
    tris += _box(-10.0, 18.0, BRACKET_Z[0], 10.0, py, BRACKET_Z[1])
    tris += _cyl(0.0, py, 3.5, BRACKET_Z[1] - 0.5, 91.0, 16)
    for deg in (45.0 + 20.7, -(50.0 + 20.7)):  # paddle side (about 15 deg at 30 mm) plus the peg's own 5.7 deg
        a = math.radians(deg)
        tris += _cyl(-30.0 * math.sin(a), py + 30.0 * math.cos(a), 3.0, BRACKET_Z[1] - 0.5, 95.0, 16)
    for deg in (-10.0, 30.0, 45.0):
        a = math.radians(deg)
        tris += _sector(62.0, 75.0, a - math.radians(0.8), a + math.radians(0.8), BRACKET_Z[1] - 0.5,
                        BRACKET_Z[1] + 1.5, 0.0, py)
    return tris


def rig_spout():
    rb = 22.0
    pts = [(0.0, 0.0, BODY_TOP), (0.0, 0.0, 205.0)]
    for k in range(1, 7):
        a = math.radians(15.0 * k)
        pts.append((-rb + rb * math.cos(a), 0.0, 205.0 + rb * math.sin(a)))
    pts.append((-REACH + rb, 0.0, 205.0 + rb))
    for k in range(1, 7):
        b = math.radians(15.0 * k)
        pts.append((-REACH + rb - rb * math.sin(b), 0.0, 205.0 + rb * math.cos(b)))
    pts.append((-REACH, 0.0, OUTLET_Z + 12.0))
    tris = []
    for p0, p1 in zip(pts[:-1], pts[1:]):
        tris += _rod(p0, p1, SPOUT_R)
    tris += _cyl(-REACH, 0.0, 10.5, OUTLET_Z, OUTLET_Z + 12.0, 24)  # aerator
    return tris


def rig_handle():
    """Hub and paddle in the pivot frame (pivot at the origin, 100 mm up when mounted), off along +y."""
    tris = _ring(0.0, 0.0, 5.25, 8.5, -8.0, -2.0)  # spring pocket
    tris += _cyl(0.0, 0.0, 8.5, -2.0, 0.0)
    tris += _ring(0.0, 0.0, 4.1, 8.5, 0.0, 8.0)  # pin hole
    tris += _box(-2.0, 6.0, 8.0, 2.0, 18.0, 16.0)  # lock tooth
    tris += _box(-2.0, 6.0, 6.0, 2.0, 8.6, 8.5)  # its root on the hub
    tris += _box(-8.0, 6.0, -8.0, 8.0, 30.0, 8.0)
    tris += _box(-6.5, 30.0, -6.5, 6.5, 56.0, 6.5)
    tris += _box(-5.0, 56.0, -5.0, 5.0, 60.0, 5.0)
    return tris


def rig_pin():
    return _cyl(0.0, 0.0, 3.95, 0.0, 40.0, 24)


def write_stl(path, name, tris):
    def nrm(a, b, c):
        u = [b[k] - a[k] for k in range(3)]
        v = [c[k] - a[k] for k in range(3)]
        n = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        m = math.sqrt(sum(x * x for x in n)) or 1.0
        return tuple(x / m for x in n)

    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            f.write("  facet normal %.6f %.6f %.6f\n    outer loop\n" % nrm(a, b, c))
            for p in (a, b, c):
                f.write("      vertex %.4f %.4f %.4f\n" % p)
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in (("angle_card", angle_card), ("outlet_target", outlet_target), ("tray", tray),
                     ("rig_body", rig_body), ("rig_spout", rig_spout), ("rig_handle", rig_handle),
                     ("rig_pin", rig_pin)):
        path = os.path.join(out_dir, name + ".stl")
        write_stl(path, name, fn())
        print("wrote", path)


if __name__ == "__main__":
    main(*sys.argv[1:2])
