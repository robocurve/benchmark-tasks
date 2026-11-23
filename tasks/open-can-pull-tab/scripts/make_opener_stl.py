"""Writes assets/opener_print.stl: the tab lifter as three overlapping boxes (mm), ready to slice (PLA, 100 % infill, handle
upright, the tongue's flat on the build plate's side is not needed: print it standing on the handle's end)."""
import struct
import sys

BOXES = [(-4.25, 11.75, -8, 8, 30, 120), (-1.5, 1.5, -4, 4, 0, 30), (1.5, 6.5, -4, 4, 0, 1.8)]


def tris(b):
    x0, x1, y0, y1, z0, z1 = b
    v = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    q = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    out = []
    for a, b2, c, d in q:
        out += [(v[a], v[b2], v[c]), (v[a], v[c], v[d])]
    return out


def main(path):
    t = [x for b in BOXES for x in tris(b)]
    with open(path, "wb") as f:
        f.write(b"tab lifter".ljust(80, b" "))
        f.write(struct.pack("<I", len(t)))
        for a, b, c in t:
            f.write(struct.pack("<12fH", 0, 0, 0, *a, *b, *c, 0))


if __name__ == "__main__":
    main(sys.argv[1])
