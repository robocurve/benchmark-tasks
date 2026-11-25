"""Print-ready (or CNC / cut-list) ASCII STL files for the custom parts of the casement window rig, in mm, from the
same dimensions as the scene in close_casement_window_and_nest_folding_crank.py. Standard library only.

The operator, its nesting cover and the folding crank are store-bought Truth Hardware Encore parts and are not
printed. The custom parts are the timber parts round them:

  stool.stl         window stool board, 132 x 500 x 32 mm, with the operator's 4 screw pilot holes
  frame.stl         window frame: two jambs, head, frame sill and the 9 mm room-side stop lip, 490 x 73 x 377 mm
  sash.stl          sash frame: two 50 mm stiles, 60 and 50 mm rails, a 3 mm pane groove, 432 x 38 x 300 mm
  start_gauge.stl   68 x 30 x 20 mm gauge block that sets the 9 degree start opening at the lock edge

Run:  python3 fabricate.py [out_dir]     (writes nothing on import)
"""

import os
import sys

# -- scene dimensions, mm (the task file's metres x 1000) -------------------------------------------------------------
STOOL = (132.0, 500.0, 32.0)
FRAME_W, FRAME_D, FRAME_H = 490.0, 73.0, 377.0   # on the stool: table z 32 .. 409
JAMB_W, HEAD_H, SILL_H = 25.0, 25.0, 42.0
LIP_D, LIP_W, LIP_TOP_H, LIP_BOT_H = 9.0, 20.0, 16.0, 10.0
SASH_W, SASH_D, SASH_H = 432.0, 38.0, 300.0
STILE_W, BOT_RAIL, TOP_RAIL = 50.0, 60.0, 50.0
PANE_T = 3.0
GAUGE = (68.0, 30.0, 20.0)
PILOT = 3.0                                       # square pilot holes for the operator's #8 screws


def _grid(boxes, cuts):
    """Cells of the breakpoint grid of all boxes; a cell is solid if inside a box and in no cut."""
    xs, ys, zs = (sorted({v for b in boxes + cuts for v in (b[0][i], b[1][i])}) for i in range(3))

    def inside(p, bs):
        return any(all(b[0][i] < p[i] < b[1][i] for i in range(3)) for b in bs)

    solid = set()
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            for k in range(len(zs) - 1):
                c = ((xs[i] + xs[i + 1]) / 2, (ys[j] + ys[j + 1]) / 2, (zs[k] + zs[k + 1]) / 2)
                if inside(c, boxes) and not inside(c, cuts):
                    solid.add((i, j, k))
    return xs, ys, zs, solid


def mesh(boxes, cuts=()):
    """Watertight triangle list of the union of boxes minus cuts (each box ((x0,y0,z0),(x1,y1,z1)))."""
    xs, ys, zs, solid = _grid(list(boxes), list(cuts))
    g = (xs, ys, zs)
    tris = []
    for (i, j, k) in solid:
        idx = (i, j, k)
        for ax in range(3):
            for side in (0, 1):
                n = list(idx)
                n[ax] += 1 if side else -1
                if tuple(n) in solid:
                    continue
                lo = [g[a][idx[a]] for a in range(3)]
                hi = [g[a][idx[a] + 1] for a in range(3)]
                u, v = [a for a in range(3) if a != ax]
                f = hi[ax] if side else lo[ax]

                def pt(a, b):
                    p = [0.0, 0.0, 0.0]
                    p[ax], p[u], p[v] = f, a, b
                    return tuple(p)
                q = [pt(lo[u], lo[v]), pt(hi[u], lo[v]), pt(hi[u], hi[v]), pt(lo[u], hi[v])]
                # outward normal along +ax when side is 1: (u, v) right-handed with ax
                rh = (ax, u, v) in ((0, 1, 2), (1, 2, 0), (2, 0, 1))
                if side != rh:
                    q = q[::-1]
                tris += [(q[0], q[1], q[2]), (q[0], q[2], q[3])]
    return tris


def write_stl(path, name, tris):
    with open(path, "w") as fh:
        fh.write(f"solid {name}\n")
        for a, b, c in tris:
            e1 = [b[i] - a[i] for i in range(3)]
            e2 = [c[i] - a[i] for i in range(3)]
            n = (e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0])
            ln = sum(x * x for x in n) ** 0.5 or 1.0
            fh.write(f"  facet normal {n[0]/ln:.6f} {n[1]/ln:.6f} {n[2]/ln:.6f}\n    outer loop\n")
            for p in (a, b, c):
                fh.write(f"      vertex {p[0]:.3f} {p[1]:.3f} {p[2]:.3f}\n")
            fh.write("    endloop\n  endfacet\n")
        fh.write(f"endsolid {name}\n")


def stool():
    x, y, z = STOOL
    # operator footprint centred 45 mm from the room edge, 30 mm right of centre; 4 pilot holes on 25 x 60 mm
    cx, cy = 45.0, y / 2 - 30.0
    holes = [((cx + dx - PILOT / 2, cy + dy - PILOT / 2, z - 20.0), (cx + dx + PILOT / 2, cy + dy + PILOT / 2, z + 1))
             for dx in (-12.5, 12.5) for dy in (-30.0, 30.0)]
    return mesh([((0, 0, 0), (x, y, z))], holes)


def frame():
    W, D, H = FRAME_W, FRAME_D, FRAME_H
    boxes = [
        ((0, 0, 0), (D, JAMB_W, H)),                          # right jamb
        ((0, W - JAMB_W, 0), (D, W, H)),                      # left jamb
        ((0, 0, H - HEAD_H), (D, W, H)),                      # head
        ((0, 0, 0), (D, W, SILL_H)),                          # frame sill
        # room-side stop lip round the opening, 9 mm deep at the room face (x 0 .. 9)
        ((0, JAMB_W, SILL_H), (LIP_D, JAMB_W + LIP_W, H - HEAD_H)),
        ((0, W - JAMB_W - LIP_W, SILL_H), (LIP_D, W - JAMB_W, H - HEAD_H)),
        ((0, JAMB_W, H - HEAD_H - LIP_TOP_H), (LIP_D, W - JAMB_W, H - HEAD_H)),
        ((0, JAMB_W, SILL_H), (LIP_D, W - JAMB_W, SILL_H + LIP_BOT_H)),
    ]
    return mesh(boxes)


def sash():
    W, D, H = SASH_W, SASH_D, SASH_H
    boxes = [
        ((0, 0, 0), (D, STILE_W, H)),
        ((0, W - STILE_W, 0), (D, W, H)),
        ((0, 0, 0), (D, W, BOT_RAIL)),
        ((0, 0, H - TOP_RAIL), (D, W, H)),
    ]
    # pane groove, 3 mm wide and 8 mm deep, centred in the sash depth, round the opening
    g0, g1 = D / 2 - PANE_T / 2, D / 2 + PANE_T / 2
    cuts = [
        ((g0, STILE_W - 8, BOT_RAIL - 8), (g1, W - STILE_W + 8, H - TOP_RAIL + 8)),
    ]
    return mesh(boxes, cuts)


def gauge():
    x, y, z = GAUGE
    return mesh([((0, 0, 0), (x, y, z))])


PARTS = {"stool": stool, "frame": frame, "sash": sash, "start_gauge": gauge}


def main(out_dir="stl"):
    os.makedirs(out_dir, exist_ok=True)
    for name, fn in PARTS.items():
        tris = fn()
        path = os.path.join(out_dir, f"{name}.stl")
        write_stl(path, name, tris)
        print(f"{path}: {len(tris)} triangles")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "stl")
