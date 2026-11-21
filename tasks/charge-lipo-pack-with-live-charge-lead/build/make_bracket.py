"""Generate bracket.stl: the bench bracket that holds the charge lead's panel-mount female XT60 (Amass XT60E-F) with
its mating face toward the robot, 17 mm above the table (the simulated clip position). ASCII STL, millimetres.

    python make_bracket.py [--hole-spacing MM] [--out bracket.stl]

Measure the XT60E-F flange hole spacing on the part (or take it from the Amass drawing) and pass it; the default is a
placeholder and must be checked before printing.
"""
import argparse

import numpy as np


def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    v = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    quads = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return [(v[a], v[b], v[c]) for q in quads for a, b, c in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hole-spacing", type=float, default=20.0)
    ap.add_argument("--out", default="bracket.stl")
    a = ap.parse_args()
    s = a.hole_spacing
    # base plate 60 x 40 x 4 mm (clamped or screwed to the bench); upright 40 x 4 x 32 mm with a 17.0 x 9.0 mm window
    # for the XT60E-F body (centre 17 mm above the table) and two M2.5 clearance holes (2.7 mm) at +-s/2.
    # Holes and the window are cut as gaps between solid blocks so the file stays a plain union of boxes.
    tris = box((-30, -20, 0), (30, 20, 4))
    y0, z0, wy, wz = 0.0, 17.0, 17.0, 9.0
    up_lo, up_hi = np.array([-2.0, -20.0, 4.0]), np.array([2.0, 20.0, 36.0])
    win_lo, win_hi = np.array([-2.0, -wy / 2, z0 - wz / 2]), np.array([2.0, wy / 2, z0 + wz / 2])
    tris += box(up_lo, (2.0, 20.0, win_lo[2]))                      # below the window
    tris += box((-2.0, -20.0, win_hi[2]), up_hi)                    # above the window
    for side in (-1, 1):                                             # left / right of the window, minus the holes
        y_in, y_out = side * wy / 2, side * 20.0
        yh = side * s / 2
        ys = sorted([y_in, y_out])
        if ys[0] < yh - 1.35 and yh + 1.35 < ys[1]:
            for ya, yb in ((ys[0], yh - 1.35), (yh + 1.35, ys[1])):
                tris += box((-2.0, ya, win_lo[2]), (2.0, yb, win_hi[2]))
            tris += box((-2.0, yh - 1.35, win_lo[2]), (2.0, yh + 1.35, z0 - 1.35))
            tris += box((-2.0, yh - 1.35, z0 + 1.35), (2.0, yh + 1.35, win_hi[2]))
        else:
            tris += box((-2.0, ys[0], win_lo[2]), (2.0, ys[1], win_hi[2]))
    with open(a.out, "w") as f:
        f.write("solid bracket\n")
        for t in tris:
            n = np.cross(t[1] - t[0], t[2] - t[0])
            n = n / (np.linalg.norm(n) or 1)
            f.write(f"facet normal {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n outer loop\n")
            for p in t:
                f.write(f"  vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write(" endloop\nendfacet\n")
        f.write("endsolid bracket\n")
    print(f"wrote {a.out}: {len(tris)} triangles, hole spacing {s} mm")


if __name__ == "__main__":
    main()
