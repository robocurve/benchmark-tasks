"""Writes the fixture's full-size drawing (frame-fixture-1to1.svg, millimetres, print at 100 %) and its cut list
(cut-list.csv) from the same numbers the simulation uses (../task.py). Run from this folder:

    python make_drawing.py

The drawing shows the frame in its own plane, level (before the 10 degree nose-down tip in the base), with the tube
centrelines and outlines, the three plywood gusset pairs, the two bottle bosses, the cage floor and the bottle seated,
a 100 mm check bar, and a side view of the base (board, bottom-bracket block, head-tube post).
"""

import csv
import math
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
import task as T  # noqa: E402

MM = 1000.0
OX, OY = 270.0, 400.0  # page position of the frame origin (the cage floor's middle), mm


def P(u, v):
    return OX + u * MM, OY - v * MM


def line(a, b, w=0.5, dash=None, col="#000"):
    (x0, y0), (x1, y1) = P(*a), P(*b)
    d = f' stroke-dasharray="{dash}"' if dash else ""
    return f'<line x1="{x0:.2f}" y1="{y0:.2f}" x2="{x1:.2f}" y2="{y1:.2f}" stroke="{col}" stroke-width="{w}"{d}/>'


def text(u, v, s, size=4.0):
    x, y = P(u, v)
    return f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" font-family="sans-serif">{s}</text>'


def tube_outline(a, b, d):
    ux, uy = b[0] - a[0], b[1] - a[1]
    n = math.hypot(ux, uy)
    nx, ny = -uy / n * d / 2, ux / n * d / 2
    pts = [P(a[0] + nx, a[1] + ny), P(b[0] + nx, b[1] + ny), P(b[0] - nx, b[1] - ny), P(a[0] - nx, a[1] - ny)]
    return '<polygon points="' + " ".join(f"{x:.2f},{y:.2f}" for x, y in pts) + '" fill="none" stroke="#000" stroke-width="0.6"/>'


def main():
    out = ['<svg xmlns="http://www.w3.org/2000/svg" width="620mm" height="680mm" viewBox="0 0 620 680">',
           '<rect width="620" height="680" fill="white"/>']
    rows = [("part", "material", "length_mm", "angle_in_frame_deg", "from_u_mm", "from_v_mm", "to_u_mm", "to_v_mm")]
    groups = [("down tube", T.DOWN, T.D_DOWN), ("top tube", T.TOPT, T.D_TOP), ("seat tube", T.SEAT, T.D_SEAT),
              ("head tube", T.HEAD, T.D_HEAD)]
    mat = {T.D_DOWN: "6061 aluminium round tube 2 in OD x 0.065 in wall",
           T.D_TOP: "6061 aluminium round tube 1-3/8 in OD x 0.058 in wall"}
    for name, pts, d in groups:
        for i, (a, b) in enumerate(zip(pts[:-1], pts[1:])):
            out += [tube_outline(a, b, d), line(a, b, 0.25, "4,2")]
            L = math.hypot(b[0] - a[0], b[1] - a[1]) * MM
            ang = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
            label = name if len(pts) == 2 else f"{name} piece {i + 1}"
            rows.append((label, mat[d], f"{L:.0f}", f"{ang:.1f}", f"{a[0] * MM:.0f}", f"{a[1] * MM:.0f}",
                         f"{b[0] * MM:.0f}", f"{b[1] * MM:.0f}"))
            mid = ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2)
            out.append(text(mid[0] + 0.03, mid[1], f"{label}: {L:.0f} mm at {ang:.1f} deg"))
    for (u, v), (lu, lv), ang in T.GUSSETS:
        c, s = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        corners = [(u + c * x - s * y, v + s * x + c * y) for x, y in
                   ((-lu / 2, -lv / 2), (lu / 2, -lv / 2), (lu / 2, lv / 2), (-lu / 2, lv / 2))]
        out.append('<polygon points="' + " ".join("%.2f,%.2f" % P(*q) for q in corners)
                   + '" fill="none" stroke="#a0522d" stroke-width="0.5"/>')
        rows.append(("gusset plate (x2)", "12 mm birch plywood", f"{lu * MM:.0f} x {lv * MM:.0f}", f"{ang:.0f}",
                     f"{u * MM:.0f}", f"{v * MM:.0f}", "", ""))
    for u, v in T.BOSS_AT:
        x, y = P(u, v)
        out.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2.5" fill="none" stroke="#c00" stroke-width="0.5"/>')
        out.append(f'<text x="{x + 4:.1f}" y="{y:.1f}" font-size="3.5" fill="#c00">M5 rivet nut ({u * MM:.0f}, {v * MM:.0f})</text>')
    # the seated bottle and its axis
    a = T.A_L[:2]
    b = T.B_L[:2]
    r = T.R_B
    h = T.BODY_H + T.CAP_H
    poly = [(-r * b[0], -r * b[1]), (-r * b[0] + h * a[0], -r * b[1] + h * a[1]), (r * b[0] + h * a[0], r * b[1] + h * a[1]),
            (r * b[0], r * b[1])]
    out.append('<polygon points="' + " ".join("%.2f,%.2f" % P(*q) for q in poly)
               + '" fill="none" stroke="#1565c0" stroke-width="0.4" stroke-dasharray="2,1"/>')
    out.append(line((0, 0), (0.22 * a[0], 0.22 * a[1]), 0.3, "1,1", "#1565c0"))
    out.append(text(0.01, -0.012, "origin: middle of the cage floor (bottle base seated); cage axis 61.0 deg in the frame", 3.5))
    # 100 mm check bar
    out.append(line((-0.20, -0.29), (-0.10, -0.29), 1.0))
    out.append(text(-0.20, -0.30, "100 mm check bar: must measure 100 mm when printed", 3.5))
    out.append(text(-0.24, 0.40, "Bottle-cage frame fixture, full size (1:1, mm). Frame plane, level. u forward (to the head tube), v up.", 5))
    out.append(text(-0.24, 0.39, f"In the base the frame is tipped {abs(math.degrees(T.PITCH)):.0f} deg nose-down; the cage floor's middle is "
                    f"{(T.FLOOR[2] - T.TOP) * MM:.0f} mm above the bench.", 4))
    out.append("</svg>")
    (HERE / "frame-fixture-1to1.svg").write_text("\n".join(out))
    rows.append(("bottom-bracket stand-in", mat[T.D_DOWN], f"{T.W_BB * MM:.0f}", "across the frame",
                 f"{T.BB[0] * MM:.0f}", f"{T.BB[1] * MM:.0f}", "", ""))
    sup, _ = T._support_shapes(T.FLOOR_Z)
    names = ["base board", "bottom-bracket block", "head-tube post"]
    for n, s in zip(names, sup):
        rows.append((n, "18 mm plywood" if n == "base board" else "plywood / 45 x 45 mm softwood",
                     " x ".join(f"{v * MM:.0f}" for v in s.size), "", "", "", "", ""))
    with open(HERE / "cut-list.csv", "w", newline="") as f:
        csv.writer(f).writerows(rows)


if __name__ == "__main__":
    main()
