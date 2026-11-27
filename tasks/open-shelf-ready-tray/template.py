"""The tray's tear line as a 1:1 cut file (tray_template.svg), made from the same numbers as the sim (task.py).

The front wall and both side walls are drawn as their outside faces, unfolded about the front corners: left side wall
(back edge at the far left), front wall, right side wall. Red is the tear line, cut through the board; the gaps in it
are the 15 ties (9 along the lip line, 3 down each diagonal), at the sim's tie positions. Black ticks mark each tie's
two edges, where a knife cut stops and restarts. Grey and blue are reference only.

Run:  PYTHONPATH=<yam-isaac-lab-arena> python template.py [tie width in mm, diagonal ties] [lip ties]
(defaults below). test_task.py checks that tray_template.svg matches this file and the sim.
"""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402

MM = 1000.0
MARGIN = 10.0  # mm round the drawing
HEAD = 34.0  # mm of notes above the walls
TIE_DIAG_MM = round(0.2 * T.TIE_K, 3)  # starting tie widths (mm), in proportion to the sim's tie strengths (0.6 mm at
TIE_LIP_MM = round(0.4 * T.TIE_K, 3)  # TIE_K 3; the lip's are twice the diagonals'); the batch pull test sets the final
#                                       widths (README)
OUT = pathlib.Path(__file__).resolve().parent / "tray_template.svg"

TD, TW, TH = T.TD * MM, T.TW * MM, T.TH * MM  # side wall length, front wall width, wall height (outside)
STRIP_W = 2 * TD + TW


def _xy(X: float, z_m: float) -> tuple[float, float]:
    """Strip X (mm from the left side wall's back edge) and height above the table (m) -> SVG point (mm)."""
    return MARGIN + X, MARGIN + HEAD + (TH - z_m * MM)


def strip_x(face: str, coord_m: float) -> float:
    """Strip X (mm) of a point on a face. left/right: coord = distance back from the front face; front: coord = y."""
    if face == "left":
        return TD - coord_m * MM
    if face == "front":
        return TD + TW / 2 - coord_m * MM
    return TD + TW + coord_m * MM


def tie_marks() -> list[tuple[str, float, float]]:
    """(tie name, strip X mm, height above the table mm) for every tie in the sim, in the sim's order."""
    out = []
    for name, seg, p_world in T.TIES:
        p = p_world - T.TRAY_POS  # tray frame: x back from the front face, y left, z up
        if seg == "panel":
            out.append((name, strip_x("front", p[1]), p[2] * MM))
        else:
            out.append((name, strip_x("left" if p[1] > 0 else "right", p[0]), p[2] * MM))
    return out


def _pieces() -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """The tear line's three straight pieces, as (strip X mm, height mm) end points."""
    x_lo, z_lo = T.diag_point(0.0)
    x_hi, z_hi = T.diag_point(1.0)
    left = ((strip_x("left", x_hi), z_hi * MM), (strip_x("left", x_lo), z_lo * MM))
    across = ((strip_x("left", x_lo), z_lo * MM), (strip_x("right", x_lo), z_lo * MM))
    right = ((strip_x("right", x_lo), z_lo * MM), (strip_x("right", x_hi), z_hi * MM))
    return [left, across, right]


def tie_edges(w_diag: float = TIE_DIAG_MM, w_lip: float = TIE_LIP_MM) -> list[tuple[float, float, float, float]]:
    """(X, z, unit direction of the line) at both edges of every tie, in mm: where a knife cut stops and restarts."""
    out = []
    for (X0, z0), (X1, z1) in _pieces():
        length = ((X1 - X0) ** 2 + (z1 - z0) ** 2) ** 0.5
        dx, dz = (X1 - X0) / length, (z1 - z0) / length
        for name, X, z in tie_marks():
            off = abs((X - X0) * (z1 - z0) - (z - z0) * (X1 - X0)) / length
            t = ((X - X0) * dx + (z - z0) * dz) / length
            if 0.0 < t < 1.0 and off < 1e-6:
                half = (w_lip if name.startswith("lip") else w_diag) / 2
                out += [(X - dx * half, z - dz * half, dx, dz), (X + dx * half, z + dz * half, dx, dz)]
    return out


def cut_segments(w_diag: float = TIE_DIAG_MM, w_lip: float = TIE_LIP_MM) -> list[tuple[float, float, float, float]]:
    """The cut, as (X0, z0, X1, z1) segments in mm: each piece of the line with a gap the tie's width at every tie."""
    marks = tie_marks()
    segs = []
    for (X0, z0), (X1, z1) in _pieces():
        length = ((X1 - X0) ** 2 + (z1 - z0) ** 2) ** 0.5
        gaps = []
        for name, X, z in marks:
            t = ((X - X0) * (X1 - X0) + (z - z0) * (z1 - z0)) / length ** 2
            off = abs((X - X0) * (z1 - z0) - (z - z0) * (X1 - X0)) / length
            if 0.0 < t < 1.0 and off < 1e-6:
                half = (w_lip if name.startswith("lip") else w_diag) / 2 / length
                gaps.append((t - half, t + half))
        t_at = 0.0
        for g0, g1 in sorted(gaps) + [(1.0, 1.0)]:
            if g0 > t_at:
                segs.append((X0 + (X1 - X0) * t_at, z0 + (z1 - z0) * t_at, X0 + (X1 - X0) * g0, z0 + (z1 - z0) * g0))
            t_at = g1
    return segs


def svg(w_diag: float = TIE_DIAG_MM, w_lip: float = TIE_LIP_MM) -> str:
    W, H = STRIP_W + 2 * MARGIN, TH + HEAD + 2 * MARGIN + 12.0
    f = lambda v: f"{v:.3f}"  # noqa: E731
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{f(W)}mm" height="{f(H)}mm" viewBox="0 0 {f(W)} {f(H)}">',
           f'<rect x="0" y="0" width="{f(W)}" height="{f(H)}" fill="white"/>',
           '<g fill="none" stroke="#9a9a9a" stroke-width="0.3">']
    for X0, X1 in ((0.0, TD), (TD, TD + TW), (TD + TW, STRIP_W)):  # the three walls' outside faces (reference)
        x, y = _xy(X0, T.TH)
        out.append(f'<rect x="{f(x)}" y="{f(y)}" width="{f(X1 - X0)}" height="{f(TH)}"/>')
    out.append('</g><g fill="none" stroke="#9a9a9a" stroke-width="0.3" stroke-dasharray="3 2">')
    for X in (TD, TD + TW):  # the front corners: folds, not cut
        (x, y0), (_, y1) = _xy(X, T.TH), _xy(X, 0.0)
        out.append(f'<line x1="{f(x)}" y1="{f(y0)}" x2="{f(x)}" y2="{f(y1)}"/>')
    out.append('</g><g fill="none" stroke="#0050ff" stroke-width="0.2">')
    for _, X, z in tie_marks():  # tie positions (reference circles, not cut)
        x, y = _xy(X, z / MM)
        out.append(f'<circle cx="{f(x)}" cy="{f(y)}" r="2.5"/>')
    out.append('</g><g fill="none" stroke="#000000" stroke-width="0.1">')
    for X, z, dx, dz in tie_edges(w_diag, w_lip):  # tie edges: 4 mm ticks across the line (reference, not cut)
        (x0, y0), (x1, y1) = _xy(X + 2.0 * dz, (z - 2.0 * dx) / MM), _xy(X - 2.0 * dz, (z + 2.0 * dx) / MM)
        out.append(f'<line x1="{f(x0)}" y1="{f(y0)}" x2="{f(x1)}" y2="{f(y1)}"/>')
    out.append('</g><g fill="none" stroke="#ff0000" stroke-width="0.1">')
    for X0, z0, X1, z1 in cut_segments(w_diag, w_lip):  # CUT THROUGH
        (x0, y0), (x1, y1) = _xy(X0, z0 / MM), _xy(X1, z1 / MM)
        out.append(f'<line x1="{f(x0)}" y1="{f(y0)}" x2="{f(x1)}" y2="{f(y1)}"/>')
    out.append('</g><g fill="#000000" font-family="Helvetica, Arial, sans-serif" font-size="4">')
    notes = [
        "Shelf-ready tray: tear line, 1:1. Outside faces of the left side wall, front wall and right side wall, "
        "unfolded about the front corners.",
        f"Red: cut through the board. The gaps in it are the 15 ties (blue circles: 9 on the lip line, 3 down each "
        f"diagonal). Ties: {w_diag:.2f} mm on the diagonals, {w_lip:.2f} mm on the lip line (starting widths: the "
        "batch pull test sets them). Black ticks: each tie's two edges.",
        f"Grey: wall outlines, and the front-corner folds (dashed); do not cut. Front wall {TW:.0f} x {TH:.0f} mm, "
        f"side walls {TD:.0f} x {TH:.0f} mm; lip line {T.LIP * MM:.0f} mm above the bottom edge; each diagonal from "
        f"{T.DX * MM:.0f} mm back from the front face at the top edge to the lip line {T.B * MM:.0f} mm behind it.",
        "Print or cut at 100 %: the scale bar must measure 100 mm. By hand: tape each wall's printout on the wall and "
        "cut along the red line with a craft knife and a steel rule, stopping and restarting at the black ticks.",
    ]
    for k, line in enumerate(notes):
        out.append(f'<text x="{f(MARGIN)}" y="{f(MARGIN + 5 + 7 * k)}">{line}</text>')
    for X0, X1, label in ((0.0, TD, "left side wall"), (TD, TD + TW, "front wall"), (TD + TW, STRIP_W,
                                                                                   "right side wall")):
        x, y = _xy((X0 + X1) / 2, 0.0)
        out.append(f'<text x="{f(x)}" y="{f(y + 6)}" text-anchor="middle">{label}</text>')
    x0, y0 = _xy(0.0, 0.0)
    out.append('</g><g stroke="#000000" stroke-width="0.4">')
    out.append(f'<line x1="{f(x0)}" y1="{f(y0 + 10)}" x2="{f(x0 + 100)}" y2="{f(y0 + 10)}"/>')
    out.append(f'</g><text x="{f(x0 + 102)}" y="{f(y0 + 11.5)}" font-family="Helvetica, Arial, sans-serif" '
               'font-size="4">100 mm</text>')
    out.append("</svg>")
    return "\n".join(out) + "\n"


if __name__ == "__main__":
    args = [float(a) for a in sys.argv[1:3]]
    OUT.write_text(svg(*args))
    print(OUT, f"{len(cut_segments(*args))} cut segments, {len(tie_marks())} ties")
