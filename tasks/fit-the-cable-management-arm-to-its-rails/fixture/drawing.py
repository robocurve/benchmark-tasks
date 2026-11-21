"""Writes geometry.json (every box of the scene, millimetres, world frame) and layout.svg (a dimensioned top view and a
side view of the rack rear) from task.py, so the drawings always match the simulated scene.

    python fixture/drawing.py        (from the task folder, with yam_arena importable)
"""

import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import task  # noqa: E402

MM = 1000.0


def boxes(part, state):
    poses = {"base": (np.asarray(state.pos, float), task._rot(state.quat_xyzw))}
    for name, (p, q) in state.links.items():
        poses[name] = (np.asarray(p, float), task._rot(q))
    out = []
    for link, c, r, s in part.world_boxes(poses):
        yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
        out.append(dict(link=link, centre_mm=[round(v * MM, 1) for v in c], size_mm=[round(v * MM, 1) for v in s],
                        yaw_deg=round(yaw, 2)))
    return out


def main():
    inst = task.CMA.state(pos=tuple(task.P_INST), joints={"elbow": -task.OPEN0})
    geo = {
        "frame": "x away from the robot toward the rack front, y to the robot's left, z up; origin at the arm bases' "
                 "midpoint on the bench top (bench top z = 0)",
        "bench_top_mm": 0.0,
        "tab_top_mm_above_bench": round((task.ZT - task.TOP) * MM, 1),
        "support_bar_top_mm_above_bench": round((task.BAR_TOP - task.TOP) * MM, 1),
        "rack": boxes(task.RACK, task.RACK.state()),
        "cma_installed": boxes(task.CMA, inst),
        "cma_start": boxes(task.CMA, task.CMA.state()),
        "cma_masses_kg_estimate": {"inner_basket_with_hinge_and_clip": task.CMA_MASS, "outer_basket_with_clip":
                                   task.ELBOW.mass},
        "latch_tooth": {"spring_N_m_per_rad": task.RACK.joints[0].spring, "standing_mm_above_tab": 2.5, "thickness_mm": 2.0,
                        "width_mm": 16.0, "hinge_mm_from_stop_face": 10.0},
        "stop_bracket": {"pivot_mm": [round(v * MM, 1) for v in task.GATE_PIVOT], "length_mm": task.GATE_LEN * MM,
                         "open_past_rad": task.GATE_OPEN_AT, "spring_N_m_per_rad": 0.10},
    }
    for k in ("rack", "cma_installed", "cma_start"):
        for b in geo[k]:
            b["centre_mm"][2] = round(b["centre_mm"][2] - task.TOP * MM, 1)
    (HERE / "geometry.json").write_text(json.dumps(geo, indent=1) + "\n")

    # top view (x up the page, y to the left) and side view, 2 px per mm
    s, ox, oy = 2.0, 520.0, 980.0

    def pt(x, y):
        return ox - y * MM * s, oy - x * MM * s

    rects = []
    for name, col in (("rack", "#8a8f96"), ("cma_installed", "#222"), ("cma_start", "#999")):
        for b in geo[name]:
            if b["link"] in ("inner_latch", "outer_latch"):
                col2 = "#e8c000"
            elif b["link"] == "stop_bracket":
                col2 = "#1a5ad0"
            else:
                col2 = col
            cx, cy, _ = (v / MM for v in b["centre_mm"])
            sx, sy, _ = (v / MM for v in b["size_mm"])
            a = math.radians(b["yaw_deg"])
            px, py = pt(cx, cy)
            dash = ' stroke-dasharray="4 3"' if name == "cma_start" else ""
            rects.append(f'<rect x="{px - sy * MM * s / 2:.1f}" y="{py - sx * MM * s / 2:.1f}" width="{sy * MM * s:.1f}" '
                         f'height="{sx * MM * s:.1f}" fill="none" stroke="{col2}" stroke-width="1"{dash} '
                         f'transform="rotate({-math.degrees(a):.2f} {px:.1f} {py:.1f})"/>')
    notes = [
        "Top view, 1:0.5 (2 px = 1 mm). Grey: rack rear (posts, rails, server, brackets, support bar). Black: CMA installed.",
        "Dashed: CMA start. Yellow: latch teeth. Blue: stop bracket (shut). Robot arm bases at x = 0, y = +/-305 mm.",
        f"Tab top faces {geo['tab_top_mm_above_bench']} mm above the bench; support bar top {geo['support_bar_top_mm_above_bench']} mm.",
        "Inner tab: x 407-437 mm, y 178-198 mm. Outer tab: x 357-387 mm, y 208-228 mm. Server rear face x 480 mm.",
        "Every value: geometry.json. Dimensions of the CMA, clips and brackets are estimates sized on the Lenovo kit.",
    ]
    text = "".join(f'<text x="20" y="{30 + 18 * i}" font-family="sans-serif" font-size="13">{n}</text>'
                   for i, n in enumerate(notes))
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="1040" height="1100" viewBox="0 0 1040 1100">'
           f'<rect width="1040" height="1100" fill="white"/>{text}{"".join(rects)}</svg>\n')
    (HERE / "layout.svg").write_text(svg)
    print("wrote", HERE / "geometry.json", HERE / "layout.svg")


if __name__ == "__main__":
    main()
