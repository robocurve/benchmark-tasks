"""Export the gauge and the six blocks as STL (millimetres) and a dimensioned drawing, from task.py's own shapes.

    pip install numpy matplotlib
    PYTHONPATH=<path to yam-isaac-lab-arena> python build/export_gauge.py      # from the task folder

Writes gauge.stl (base plate and four anvils, base bottom at z 0, centred on the plate), block_a.stl .. block_f.stl,
dimensions.png and dimensions.json (every number printed on the drawing). Isaac Sim is not needed.
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
from yam_arena import task_file

HERE = pathlib.Path(__file__).resolve().parent
TASK = task_file.load(HERE.parent / "task.py")
T = TASK.module
MM = 1000.0
GAP_TOL_MM = 0.05  # gap width tolerance for the real gauge (a sixth of the 0.3 mm drop-in clearance)
BLOCK_TOL_MM = 0.05
GAUGE_NAMES = tuple(sorted(T.GAUGE_NAMES))
ANVILS = ("go_anvil_l", "go_anvil_r", "nogo_anvil_l", "nogo_anvil_r")


def corners(names) -> dict:
    """name -> the part's 8 corners in mm, relative to the gauge base's bottom centre (bevels turned 45 deg about x)."""
    spec = {o.name: o for o in TASK.objects}
    base = spec["gauge_base"]
    origin = np.array([base.pos[0], base.pos[1], base.pos[2] - base.size[2] / 2])
    a = np.pi / 4
    turn = np.array([[1, 0, 0], [0, np.cos(a), -np.sin(a)], [0, np.sin(a), np.cos(a)]])
    out = {}
    for n in names:
        half = np.array(spec[n].size) / 2
        local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
        if n.endswith("_bevel"):
            local = local @ turn.T
        out[n] = np.round((local + np.array(spec[n].pos) - origin) * MM, 4)
    return out


def bounds(names) -> tuple:
    pts = np.concatenate([corners([n])[n] for n in names])
    return pts.min(0), pts.max(0)


def hull(p) -> np.ndarray:
    """Convex hull of 2-D points (monotone chain), for drawing a projected part."""
    p = sorted(map(tuple, np.round(p, 6)))
    def half(pts):
        out = []
        for q in pts:
            while len(out) >= 2 and np.cross(np.subtract(out[-1], out[-2]), np.subtract(q, out[-2])) <= 0:
                out.pop()
            out.append(q)
        return out[:-1]
    return np.array(half(p) + half(p[::-1]))


def box_triangles(v) -> list:
    faces = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4), (2, 6, 7), (2, 7, 3), (0, 4, 6),
             (0, 6, 2), (1, 3, 7), (1, 7, 5)]  # outward normals
    return [v[list(f)] for f in faces]


def write_stl(path, parts) -> None:
    lines = [f"solid {path.stem}"]
    for v in parts:
        for tri in box_triangles(v):
            n = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            n = n / np.linalg.norm(n)
            lines += [f"  facet normal {n[0]:.6g} {n[1]:.6g} {n[2]:.6g}", "    outer loop"]
            lines += [f"      vertex {p[0]:.4f} {p[1]:.4f} {p[2]:.4f}" for p in tri]
            lines += ["    endloop", "  endfacet"]
    path.write_text("\n".join(lines + [f"endsolid {path.stem}", ""]))


def anvil_bounds(name) -> tuple:
    return bounds([name, f"{name}_top", f"{name}_bevel"])


def numbers() -> dict:
    base_lo, base_hi = bounds(["gauge_base"])
    gaps = {}
    for gap in ("go", "nogo"):
        left, right = anvil_bounds(f"{gap}_anvil_l"), anvil_bounds(f"{gap}_anvil_r")
        gaps[gap] = {"width": round(float(left[0][1] - right[1][1]), 4),
                     "centre_y": round(float((left[0][1] + right[1][1]) / 2), 4),
                     "anvil_faces_y": [round(float(v), 4) for v in (right[0][1], right[1][1], left[0][1], left[1][1])]}
    a_lo, a_hi = anvil_bounds("go_anvil_l")
    return {
        "base": [round(float(v), 4) for v in base_hi - base_lo],
        "anvil": [round(float(v), 4) for v in a_hi - a_lo],
        "anvil_top_above_base_bottom": round(float(a_hi[2]), 4),
        "chamfer_45deg": round(T.CHAMFER * MM, 4),
        "gaps": gaps,
        "block_length": T.BLOCK_LEN * MM,
        "block_height": T.BLOCK_H * MM,
        "widths": {k.upper(): round(w * MM, 4) for k, w in T.WIDTHS.items()},
        "in_tolerance": sorted(k.upper() for k, w in T.WIDTHS.items() if T.in_tolerance(w)),
        "drop_clearance_per_side_min": T.DROP_CLEAR * MM,
        "stay_overlap_per_side_min": T.STAY_OVERLAP * MM,
    }


def draw(nums, path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon

    g = corners(GAUGE_NAMES)
    fig, (top, front) = plt.subplots(1, 2, figsize=(15, 5.5), gridspec_kw={"width_ratios": [1, 2]})
    for ax, (i, j), title in ((top, (1, 0), "Top view (mm): y across, x away from the robot"),
                              (front, (1, 2), "Front view (mm), section through the anvils")):
        for n, v in g.items():
            color = "#999999" if n == "gauge_base" else ("#3c8c46" if n.startswith("go") else "#b42a28")
            ax.add_patch(Polygon(hull(v[:, [i, j]]), fc=color, ec=color if "anvil" in n else "k", lw=0.8))
        ax.set_title(title)
        ax.set_aspect("equal")
        ax.autoscale()
        ax.grid(alpha=0.3)
        ax.set_xlabel("y (mm), robot's left to the left")
        ax.set_ylabel("x (mm)" if ax is top else "z (mm) above the plate bottom")
    top.invert_xaxis()
    front.invert_xaxis()
    top_z = nums["anvil_top_above_base_bottom"]
    for gap, d in nums["gaps"].items():
        y0, w = d["centre_y"], d["width"]
        front.annotate("", (y0 - w / 2, top_z + 4), (y0 + w / 2, top_z + 4), arrowprops={"arrowstyle": "<->"})
        name = "GO" if gap == "go" else "NO-GO"
        front.text(y0, top_z + 6, f"{name} {w:.2f} +{GAP_TOL_MM:.2f}/-0 mm\ncentre y {y0:+.1f}", ha="center", va="bottom")
        for f in d["anvil_faces_y"]:
            front.text(f, -3, f"{f:+.2f}", ha="center", va="top", fontsize=7)
    bx, by, bz = nums["base"]
    ax_, ay_, az_ = nums["anvil"]
    front.set_ylim(-9, top_z + 25)
    widths = ", ".join(f"{k} {v:.1f}" for k, v in nums["widths"].items())
    fig.text(0.02, 0.02,
             f"Base plate {bx:.0f} x {by:.0f} x {bz:.0f} (x by y by z). Anvils {ax_:.0f} x {ay_:.0f} x {az_:.0f}, "
             f"anvil tops {top_z:.0f} above the plate bottom, anvils centred on the plate in x; each anvil's inner top edge "
             f"has a {nums['chamfer_45deg']:.1f} x 45 deg lead-in chamfer.\nGap widths are between "
             f"the anvils' inner faces; the numbers under the front view are the anvil faces' y positions.\n"
             f"Blocks {nums['block_length']:.0f} long x {nums['block_height']:.0f} tall, widths (+-{BLOCK_TOL_MM:.2f}): "
             f"{widths}; in tolerance ({T.NOMINAL * MM:.1f} +- {T.TOLERANCE * MM:.1f}): {', '.join(nums['in_tolerance'])}.\n"
             f"A block that must drop in clears its gap by at least {nums['drop_clearance_per_side_min']:.2f} a side; "
             f"one that must stay out overlaps each anvil's inner face by at least {nums['stay_overlap_per_side_min']:.2f}.",
             fontsize=9, va="bottom")
    fig.tight_layout(rect=(0, 0.15, 1, 1))
    fig.savefig(path, dpi=150)


def main() -> None:
    write_stl(HERE / "gauge.stl", list(corners(GAUGE_NAMES).values()))
    for k, w in T.WIDTHS.items():
        half = np.array([T.BLOCK_LEN, w, T.BLOCK_H]) * MM / 2
        unit = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
        write_stl(HERE / f"block_{k}.stl", [np.round(unit * half + [0, 0, half[2]], 4)])
    nums = numbers()
    (HERE / "dimensions.json").write_text(json.dumps(nums, indent=2) + "\n")
    draw(nums, HERE / "dimensions.png")
    print("wrote gauge.stl, block_a..f.stl, dimensions.json, dimensions.png")


if __name__ == "__main__":
    main()
