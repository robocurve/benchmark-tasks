"""Print-ready STL files (ASCII, mm) of the custom parts of the fogger task, from the same dimensions as the scene:

- jacket_arc.stl: the water heater jacket's front 110 degree arc, 508 mm outside diameter, 4 mm wall, 250 mm tall
  (print in sections or use a scrap jacket; it only carries the control and the door plate);
- door_plate.stl: the burner access door, 100 x 70 x 10 mm, with a 28 mm sight-window hole for the tea light;
- bezel_lip.stl: the bezel ring round the WV8840 knob (68 mm outside, 54 mm inside, 19 mm tall) with the interlock
  ledge, for a bench mock-up without a real WV8840.

Run as a script: python3 fabricate.py [out_dir]. Importing this module writes nothing.
"""

import math
import os
import sys

TANK_R, WALL, TANK_H = 254.0, 4.0, 250.0
ARC = math.radians(110.0)
DOOR = (100.0, 70.0, 10.0)
WINDOW_R = 14.0
BEZEL = (27.0, 34.0, 19.0)  # r_in, r_out, height
LIP = (22.5, 27.0, 8.5, 12.0)  # r_in, r_out, height, ledge end (deg) past the tooth centre at PILOT and OFF


def _facet(f, a, b, c):
    ux, uy, uz = (b[i] - a[i] for i in range(3))
    vx, vy, vz = (c[i] - a[i] for i in range(3))
    n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    m = math.sqrt(sum(x * x for x in n)) or 1.0
    f.write(f"facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n outer loop\n")
    for p in (a, b, c):
        f.write(f"  vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
    f.write(" endloop\nendfacet\n")


def _quad(f, a, b, c, d):
    _facet(f, a, b, c)
    _facet(f, a, c, d)


def _ring_sector(f, r0, r1, z0, z1, a0, a1, n):
    """A closed ring sector (watertight): outer, inner, top, bottom and the two end faces."""
    angs = [a0 + (a1 - a0) * i / n for i in range(n + 1)]

    def p(r, a, z):
        return (r * math.cos(a), r * math.sin(a), z)

    for a, b in zip(angs[:-1], angs[1:]):
        _quad(f, p(r1, a, z0), p(r1, b, z0), p(r1, b, z1), p(r1, a, z1))
        _quad(f, p(r0, b, z0), p(r0, a, z0), p(r0, a, z1), p(r0, b, z1))
        _quad(f, p(r0, a, z1), p(r1, a, z1), p(r1, b, z1), p(r0, b, z1))
        _quad(f, p(r0, b, z0), p(r1, b, z0), p(r1, a, z0), p(r0, a, z0))
    if a1 - a0 < 2 * math.pi - 1e-9:
        _quad(f, p(r0, a0, z0), p(r0, a0, z1), p(r1, a0, z1), p(r1, a0, z0))
        _quad(f, p(r1, a1, z0), p(r1, a1, z1), p(r0, a1, z1), p(r0, a1, z0))


def _box(f, x0, x1, y0, y1, z0, z1):
    c = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    for a, b, cc, d in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        _quad(f, c[a], c[b], c[cc], c[d])


def jacket_arc(f):
    _ring_sector(f, TANK_R - WALL, TANK_R, 0.0, TANK_H, math.pi - ARC / 2, math.pi + ARC / 2, 44)


def door_plate(f):
    """Plate with a square-cornered window approximated by four bars round a 28 mm opening (print flat)."""
    w, h, t = DOOR
    s = WINDOW_R
    _box(f, -w / 2, w / 2, -h / 2, -s, 0, t)
    _box(f, -w / 2, w / 2, s, h / 2, 0, t)
    _box(f, -w / 2, -s, -s, s, 0, t)
    _box(f, s, w / 2, -s, s, 0, t)


def bezel_lip(f):
    r0, r1, h = BEZEL
    _ring_sector(f, r0, r1, 0.0, h, 0.0, 2 * math.pi, 48)
    li, lo, lh, gap = LIP
    # the ledge spans from the tooth's OFF position + gap to its PILOT position - gap (tooth at +90 deg at ON)
    a0 = math.radians(90.0 - 90.0 + gap)
    a1 = math.radians(90.0 - 45.0 - gap)
    _ring_sector(f, li, lo + 0.5, h - lh, h, a0, a1, 6)


PARTS = {"jacket_arc": jacket_arc, "door_plate": door_plate, "bezel_lip": bezel_lip}


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in PARTS.items():
        path = os.path.join(out_dir, name + ".stl")
        with open(path, "w") as f:
            f.write(f"solid {name}\n")
            fn(f)
            f.write(f"endsolid {name}\n")
        print("wrote", path)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
