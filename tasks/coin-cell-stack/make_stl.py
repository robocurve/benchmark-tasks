"""Write the printed parts as STL files (millimetres) into stl/: the assembly mould, the tweezer stand, the tube rack and
the pipette holder, at the sizes the simulation uses.  python make_stl.py

Each is a closed solid for a slicer. The tweezer stand, rack and holder are unions of touching boxes (a slicer merges
them); the mould is one solid with its round pocket.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def box(size, centre):
    sx, sy, sz = (s / 2 for s in size)
    cx, cy, cz = centre
    v = np.array([[cx + a * sx, cy + b * sy, cz + c * sz] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
    f = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    return v, np.array(f)


def mould(side=60.0, height=20.0, pocket_d=20.3, depth=2.0, n=96):
    """A square block with a round pocket in its top."""
    r, h, z1 = pocket_d / 2, side / 2, height - depth
    v, f = [], []

    def add(p):
        v.append(p)
        return len(v) - 1

    # the top: a square ring round the pocket, fanned from the square's outline to the circle
    ang = [2 * math.pi * k / n for k in range(n)]
    circ_top = [add((r * math.cos(a), r * math.sin(a), height)) for a in ang]
    circ_bot = [add((r * math.cos(a), r * math.sin(a), z1)) for a in ang]
    sq = [add((h * max(-1, min(1, math.cos(a) * math.sqrt(2))), h * max(-1, min(1, math.sin(a) * math.sqrt(2))), height)) for a in ang]
    for k in range(n):
        j = (k + 1) % n
        f += [(sq[k], sq[j], circ_top[j]), (sq[k], circ_top[j], circ_top[k])]
        f += [(circ_top[k], circ_top[j], circ_bot[j]), (circ_top[k], circ_bot[j], circ_bot[k])]  # the pocket's wall
    c = add((0.0, 0.0, z1))
    f += [(c, circ_bot[(k + 1) % n], circ_bot[k]) for k in range(n)]  # the pocket's floor
    base = [add((p[0], p[1], 0.0)) for p in (v[i] for i in sq)]
    for k in range(n):
        j = (k + 1) % n
        f += [(sq[k], base[k], base[j]), (sq[k], base[j], sq[j])]  # the sides
    cb = add((0.0, 0.0, 0.0))
    f += [(cb, base[k], base[(k + 1) % n]) for k in range(n)]  # the bottom
    return np.array(v), np.array(f)


def union(boxes):
    vs, fs, n = [], [], 0
    for size, centre in boxes:
        v, f = box(size, centre)
        vs.append(v)
        fs.append(f + n)
        n += len(v)
    return np.vstack(vs), np.vstack(fs)


def write(path: Path, v, f, name):
    lines = [f"solid {name}"]
    for tri in f:
        a, b, c = v[tri[0]], v[tri[1]], v[tri[2]]
        nrm = np.cross(b - a, c - a)
        nrm = nrm / max(np.linalg.norm(nrm), 1e-12)
        lines.append(f"  facet normal {nrm[0]:.6f} {nrm[1]:.6f} {nrm[2]:.6f}\n    outer loop")
        lines += [f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in (a, b, c)]
        lines.append("    endloop\n  endfacet")
    lines.append(f"endsolid {name}")
    path.write_text("\n".join(lines) + "\n")


def main():
    import cellparts as cp

    out = HERE / "stl"
    out.mkdir(exist_ok=True)
    mm = 1000.0
    write(out / "assembly_mould.stl", *mould(cp.MOULD[0] * mm, cp.MOULD[2] * mm, cp.MOULD_POCKET[0] * mm, cp.MOULD_POCKET[1] * mm), "assembly_mould")
    sx, sy, sz = (s * mm for s in cp.TW_STAND)
    wx, wy, d = (s * mm for s in cp.TW_SLOT)
    fl = sz - d
    stand = [((sx, sy, fl), (0, 0, fl / 2)), (((sx - wx) / 2, sy, d), (-(sx + wx) / 4, 0, fl + d / 2)), (((sx - wx) / 2, sy, d), ((sx + wx) / 4, 0, fl + d / 2)),
             ((wx, (sy - wy) / 2, d), (0, -(sy + wy) / 4, fl + d / 2)), ((wx, (sy - wy) / 2, d), (0, (sy + wy) / 4, fl + d / 2))]
    write(out / "tweezer_stand.stl", *union(stand), "tweezer_stand")
    rx, ry, rz = (s * mm for s in cp.RACK)
    hole, seat = (cp.TUBE_OD / 2 + 0.0003) * mm, cp.TUBE_SEAT * mm
    rack = [((rx, ry, seat), (0, 0, seat / 2)), ((rx, ry / 2 - hole, rz - seat), (0, (ry / 2 + hole) / 2, (rz + seat) / 2)),
            ((rx, ry / 2 - hole, rz - seat), (0, -(ry / 2 + hole) / 2, (rz + seat) / 2)), ((rx / 2 - hole, 2 * hole, rz - seat), ((rx / 2 + hole) / 2, 0, (rz + seat) / 2)),
            ((rx / 2 - hole, 2 * hole, rz - seat), (-(rx / 2 + hole) / 2, 0, (rz + seat) / 2))]
    write(out / "tube_rack.stl", *union(rack), "tube_rack")
    hz = cp.HOLDER_CRADLE_Z * mm
    holder = [((60, 60, 8), (0, 0, 4)), ((10, 16, hz), (25, 0, hz / 2)), ((8, 34, 6), (25, 0, hz - 3)),
              ((40, 6, 6), (5, 14, hz - 3)), ((40, 6, 6), (5, -14, hz - 3))]
    write(out / "pipette_holder.stl", *union(holder), "pipette_holder")
    print("wrote", ", ".join(sorted(p.name for p in out.glob("*.stl"))))


if __name__ == "__main__":
    main()
