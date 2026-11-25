"""Print and cut files for the custom pieces of the rotary-tool task, in millimetres, from the task file's own numbers.

Nothing in this task's mechanism is custom: the tool, plug, strip and vise are commercial parts. Two small helpers are:

  strip_board.stl   the pine board the strip is screwed to (19 x 140 x 350 mm, cut from a 1x6 board)
  reset_gauge.stl   an 18 mm printed spacer: at reset, set it on the housing against the white OFF line and slide the
                    switch cap back onto it, which puts the cap at setting 8 (18 mm from OFF) every trial

Run it as a script (python3 fabricate.py [out_dir]); importing it writes nothing. Standard library only.
"""

import os
import sys

BOARD = (19.0, 350.0, 140.0)       # x (thickness), y (length), z (height), mm: the task's BOARD and box size
SETTING_8 = 18.0                   # mm from the OFF line to the cap's rear face at the start (-Q_OFF)
GAUGE_W, GAUGE_H, TAB_H = 10.0, 4.0, 14.0  # gauge across the housing, its height, and an upright pull tab


def _box(x0, y0, z0, x1, y1, z1):
    """Twelve triangles of an axis-aligned box, outward normals."""
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0), (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
             (2, 7, 6), (3, 0, 4), (3, 4, 7)]
    return [(v[a], v[b], v[c]) for a, b, c in faces]


def _write(path, name, tris):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a, b, c in tris:
            ux, uy, uz = (b[i] - a[i] for i in range(3))
            vx, vy, vz = (c[i] - a[i] for i in range(3))
            n = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
            m = sum(k * k for k in n) ** 0.5 or 1.0
            f.write(f"  facet normal {n[0] / m:.6f} {n[1] / m:.6f} {n[2] / m:.6f}\n    outer loop\n")
            for p in (a, b, c):
                f.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")


def main(out_dir="."):
    os.makedirs(out_dir, exist_ok=True)
    _write(os.path.join(out_dir, "strip_board.stl"), "strip_board", _box(0, 0, 0, *BOARD))
    # the gauge: an 18 mm long bar that sits on the housing between the OFF line and the cap, with a tab to lift it
    gauge = _box(0, 0, 0, SETTING_8, GAUGE_W, GAUGE_H) + _box(0, 3.0, GAUGE_H, 4.0, GAUGE_W - 3.0, GAUGE_H + TAB_H)
    _write(os.path.join(out_dir, "reset_gauge.stl"), "reset_gauge", gauge)
    print(f"wrote strip_board.stl and reset_gauge.stl to {os.path.abspath(out_dir)}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
