"""Draw the real-trial rig, dimensioned, from task.py's own constants: python draw_rig.py  ->  rig_layout.svg

task.py is read, not imported (it needs the simulator): its module-level constants are evaluated with every BELT_* setting at its default and the table
top at z = 0. The case's and the stand-off's outlines are literals inside _case() and _receiver(); they are restated in OUTLINE below, and the script stops
if task.py no longer contains them. The arm bases (x = 0, y = +/-0.305 m) and the 0.60 m reach are the Robocurve station's figures, not task.py's.
"""

from __future__ import annotations

import ast
import collections
import math
import pathlib
import types

HERE = pathlib.Path(__file__).resolve().parent
SRC = (HERE / "task.py").read_text()

ARM_Y, REACH = 0.305, 0.60  # Robocurve station: arm bases at y = +/-30.5 cm, x = 0; reach about 60 cm
BOARD_T = 0.018  # _wall(): t = 0.018

# literals inside _case() and _receiver(), with the text that must still be in task.py
OUTLINE = dict(case_len=0.115, case_h=0.070, case_bat_h=0.090, stand_w=0.089, stand_h=0.130, screws=0.090)
MUST = ["size = (CASE_D, 0.115, 0.070)", "bat = (BATTEN, 0.115, 0.090)", "(WALL_X - xs, 0.089, 0.130)", "t = 0.018", "m.box((E_T, E_L, E_H), ec)",
        "(CLIP_H - 0.090) / 2", "Y_SEAT = Y_RECV + LIP_T - (E_L / 2 + NECK_L)"]


def constants() -> dict:
    env = {"float": float, "int": int, "_f": lambda n, d: float(d), "os": types.SimpleNamespace(environ={}), "TABLE_TOP": collections.defaultdict(float), "__builtins__": {}}
    for node in ast.parse(SRC).body:
        if isinstance(node, ast.Assign):
            try:
                val = eval(compile(ast.Expression(node.value), "task.py", "eval"), env)
            except Exception:
                continue
            for tgt in node.targets:
                if isinstance(tgt, ast.Name):
                    env[tgt.id] = val
                elif isinstance(tgt, ast.Tuple) and isinstance(val, tuple):
                    for el, v in zip(tgt.elts, val):
                        env[el.id] = v
    return env


C = constants()
for s in MUST:
    if s not in SRC:
        raise SystemExit(f"task.py changed: '{s}' not found; update OUTLINE in draw_rig.py")

XB, WALL_X, WALL_W, WALL_H = C["XB"], C["WALL_X"], C["WALL_W"], C["WALL_H"]
ZB, Y_M, Y_R, CASE_D, BATTEN = C["ZB"], C["Y_MOUTH"], C["Y_RECV"], C["CASE_D"], C["BATTEN"]
H_L, H_W, H_T = C["E_L"], C["E_H"], C["E_T"]  # the end fitting
Y_DOCK, Y_SEAT, Y_HAND = C["Y_END0"], C["Y_SEAT"], C["Y_HAND"]
CLIP_H, CLIP_W, CLIP_D, LIP_T, BORE_Y, BORE_X, SLIT_X = (C[k] for k in ("CLIP_H", "CLIP_W", "CLIP_D", "LIP_T", "BORE_Y", "BORE_X", "SLIT_X"))
FOOT, CHAN_H, WIN_H, Z_CLIP, Z_FLOOR, Z_LIPTOP, Z_HEAD = (C[k] for k in ("FOOT", "CHAN_H", "WIN_H", "Z_CLIP", "Z_FLOOR", "Z_LIPTOP", "Z_HEAD"))
NECK_X, NECK_L, BEAD_X, BEAD_L, FIN_H, LIFT, GAP = (C[k] for k in ("NECK_X", "NECK_L", "BEAD_X", "BEAD_L", "FIN_H", "LIFT", "GAP"))
CLIPC = "#2b2b2e"
O = OUTLINE

out: list[str] = []
W_PX, H_PX = 1240, 1240
INK, DIM, BELT, CASE, ZINC, WOOD, HOUS = "#222", "#c0392b", "#d4a017", "#f2c200", "#9aa0a6", "#c8a27a", "#555a60"


def mm(v: float) -> str:
    return f"{v * 1000:.0f}"


def rect(x0, y0, x1, y1, fill="none", stroke=INK, sw=1.2, dash=None, op=1.0):
    xa, xb, ya, yb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<rect x="{xa:.1f}" y="{ya:.1f}" width="{xb - xa:.1f}" height="{yb - ya:.1f}" fill="{fill}" fill-opacity="{op}" stroke="{stroke}" stroke-width="{sw}"{d}/>')


def line(x0, y0, x1, y1, stroke=INK, sw=1.0, dash=None):
    d = f' stroke-dasharray="{dash}"' if dash else ""
    out.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="{stroke}" stroke-width="{sw}"{d}/>')


def text(x, y, s, size=12, anchor="middle", fill=INK, rot=None, weight="normal"):
    r = f' transform="rotate({rot} {x:.1f} {y:.1f})"' if rot is not None else ""
    out.append(f'<text x="{x:.1f}" y="{y:.1f}" font-family="Helvetica, Arial, sans-serif" font-size="{size}" text-anchor="{anchor}" fill="{fill}" font-weight="{weight}"{r}>{s}</text>')


def dim_h(xa, xb, y, label, ext_from=None, size=11):
    """A horizontal dimension at page height y between page x xa and xb; extension lines from ext_from (page y) if given."""
    if ext_from is not None:
        for x in (xa, xb):
            line(x, ext_from, x, y + (4 if y > ext_from else -4), DIM, 0.6)
    line(xa, y, xb, y, DIM, 0.9)
    for x, s in ((xa, 1), (xb, -1)):
        lo, hi = (xa, xb) if xa < xb else (xb, xa)
        sgn = 1 if x == lo else -1
        out.append(f'<path d="M{x:.1f},{y:.1f} l{6 * sgn:.1f},-3 l0,6 z" fill="{DIM}"/>')
    text((xa + xb) / 2, y - 4, label, size, fill=DIM)


def dim_v(x, ya, yb, label, ext_from=None, size=11, side=-1, rot=False):
    if ext_from is not None:
        for y in (ya, yb):
            line(ext_from, y, x + (4 if x > ext_from else -4), y, DIM, 0.6)
    line(x, ya, x, yb, DIM, 0.9)
    lo = min(ya, yb)
    hi = max(ya, yb)
    out.append(f'<path d="M{x:.1f},{lo:.1f} l-3,6 l6,0 z" fill="{DIM}"/>')
    out.append(f'<path d="M{x:.1f},{hi:.1f} l-3,-6 l6,0 z" fill="{DIM}"/>')
    if rot:
        text(x + side * 8, (ya + yb) / 2, label, size, fill=DIM, rot=-90)
    else:
        text(x + side * 5, (ya + yb) / 2 + 4, label, size, anchor="end" if side < 0 else "start", fill=DIM)


# ---------------------------------------------------------------- plan view
S = 0.70  # px per mm
CX = 600
PT = 130  # page y of the board's back face


def P(x: float, y: float):
    """Plan: the robot's left (+y) to the page's left, forward (+x) up the page."""
    return CX - y * 1000 * S, PT + (WALL_X + BOARD_T - x) * 1000 * S


text(30, 30, "Belt barrier relay: real-trial rig, from task.py (mm; table top z = 0)", 17, anchor="start", weight="bold")
text(30, 52, "1. Plan view from above. x forward from the arm bases, y to the robot's left. Robot at the bottom, wall at the top.", 13, anchor="start")

# board
rect(*P(WALL_X, -WALL_W / 2), *P(WALL_X + BOARD_T, WALL_W / 2), fill=WOOD, op=0.5)
text(P(WALL_X + BOARD_T, 0)[0], P(WALL_X + BOARD_T, 0)[1] - 6, f"wall board {mm(WALL_W)} x {mm(WALL_H)} x {mm(BOARD_T)} plywood, standing on the table", 11)
# unit: batten, case, dock
y_case0 = Y_M - O["case_len"]
rect(*P(WALL_X - BATTEN, y_case0), *P(WALL_X, Y_M), fill=WOOD)
rect(*P(XB - 0.02, y_case0), *P(XB - 0.02 + CASE_D, Y_M), fill=CASE)
text(*P(XB + 0.03, (y_case0 + Y_M) / 2), "", 10)
cx_, cy_ = P(XB + 0.02, (y_case0 + Y_M) / 2)
text(cx_, cy_, "896 unit", 10)
text(cx_, cy_ + 12, f"{mm(CASE_D)} x {mm(O['case_len'])} est.", 9)
bx, by = P(WALL_X - BATTEN / 2, y_case0 - 0.01)
# receiver: the 898 clip on its stand-off
XS = XB + CLIP_D / 2
rect(*P(XS, Y_R), *P(WALL_X, Y_R + O["stand_w"]), fill=WOOD)
rect(*P(XB - CLIP_D / 2, Y_R), *P(XS, Y_R + CLIP_W), fill=CLIPC)
rx, ry = P(XB + 0.06, Y_R + O["stand_w"] + 0.006)
text(rx, ry, "898 clip", 10, anchor="end")
text(rx, ry + 12, "on stand-off", 10, anchor="end")
# belt plane and housing positions
line(*P(XB, Y_M), *P(XB, Y_R), BELT, 2.2, "8,4")
for yc, lab, col in ((Y_DOCK, "start (docked)", HOUS), (Y_HAND, "hand-over", "#777"), (Y_SEAT, "seated", HOUS)):
    rect(*P(XB - H_T / 2, yc - H_L / 2), *P(XB + H_T / 2, yc + H_L / 2), fill=col, stroke=INK, sw=0.8, op=0.9 if col == HOUS else 0.5)
    px_, py_ = P(XB - H_T / 2 - 0.016, yc)
    text(px_, py_ + 4, lab, 10)
    text(px_, py_ + 16, f"y = {mm(yc)}", 9)
# arm bases and reach
for ys, nm in ((ARM_Y, "left arm"), (-ARM_Y, "right arm")):
    bx_, by_ = P(0.0, ys)
    out.append(f'<circle cx="{bx_:.1f}" cy="{by_:.1f}" r="9" fill="#444"/>')
    text(bx_, by_ + 26, f"{nm} base", 11)
    text(bx_, by_ + 40, f"(0, {mm(ys)})", 10)
    r = REACH * 1000 * S
    out.append(f'<path d="M{bx_ - r:.1f},{by_:.1f} A{r:.1f},{r:.1f} 0 0 1 {bx_ + r:.1f},{by_:.1f}" fill="none" stroke="#888" stroke-width="0.8" stroke-dasharray="3,4" clip-path="url(#plan)"/>')
text(*P(0.07, 0.0), "dashed arcs: about 600 reach from each base", 10, fill="#666")

# plan dimensions
top_ext = P(WALL_X + BOARD_T, 0)[1]
y_line = PT - 24
dim_h(P(0, WALL_W / 2)[0], P(0, -WALL_W / 2)[0], y_line, f"board {mm(WALL_W)}", ext_from=top_ext)
y2 = P(XB, 0)[1] + 0
row = P(XB - 0.03, 0)[1] + 52
dim_h(P(0, Y_R)[0], P(0, Y_M)[0], row, f"unit mouth to receiver mouth {mm(Y_R - Y_M)}", ext_from=P(XB, 0)[1] + 10)
row2 = row + 30
dim_h(P(0, 0)[0], P(0, Y_M)[0], row2, f"{mm(-Y_M)}", ext_from=P(XB, 0)[1] + 10)
dim_h(P(0, Y_R)[0], P(0, 0)[0], row2, f"{mm(Y_R)}", ext_from=P(XB, 0)[1] + 10)
dim_h(P(0, Y_M)[0], P(0, -WALL_W / 2)[0], row2, f"{mm(WALL_W / 2 + Y_M)}", ext_from=P(WALL_X, 0)[1] + 4)
dim_h(P(0, WALL_W / 2)[0], P(0, Y_R)[0], row2, f"{mm(WALL_W / 2 - Y_R)}", ext_from=P(WALL_X, 0)[1] + 4)
row3 = P(0, 0)[1] + 62
dim_h(P(0, ARM_Y)[0], P(0, -ARM_Y)[0], row3, f"arm bases {mm(2 * ARM_Y)} apart, centred on y = 0", ext_from=P(0, 0)[1] + 12)
line(*P(-0.01, 0), *P(WALL_X + BOARD_T + 0.01, 0), "#999", 0.6, "10,3,2,3")
text(P(0.005, 0)[0] + 4, P(0.005, 0)[1] - 4, "y = 0", 9, anchor="start", fill="#666")
# x dimensions at the right side of the page
xr_ = P(0, -WALL_W / 2)[0] + 40
dim_v(xr_, P(0, 0)[1], P(XB, 0)[1], f"belt plane x = {mm(XB)}", ext_from=P(0, -ARM_Y)[0] + 14, side=1, rot=True)
dim_v(xr_ + 0, P(XB, 0)[1], P(WALL_X, 0)[1], f"{mm(WALL_X - XB)}", side=1)
xl_ = P(0, WALL_W / 2)[0] - 40
dim_v(xl_, P(0, 0)[1], P(WALL_X, 0)[1], f"board face x = {mm(WALL_X)}", side=-1, rot=True)
# case and batten depths, under the unit
xu = P(0, y_case0)[0] + 16
dim_v(xu, P(XB - 0.02, 0)[1], P(XB - 0.02 + CASE_D, 0)[1], f"{mm(CASE_D)}", ext_from=P(0, y_case0)[0], side=1, size=10)
dim_v(xu, P(WALL_X - BATTEN, 0)[1], P(WALL_X, 0)[1], f"{mm(BATTEN)}", ext_from=P(0, y_case0)[0], side=1, size=10)
dim_v(P(0, Y_R)[0] + 12, P(XB - CLIP_D / 2, 0)[1], P(XS, 0)[1], f"{mm(CLIP_D)}", ext_from=P(0, Y_R)[0], side=1, size=10)
dim_v(P(0, Y_R)[0] + 12, P(XS, 0)[1], P(WALL_X, 0)[1], f"stand-off {mm(WALL_X - XS)}", ext_from=P(0, Y_R)[0], side=1, size=10)
dim_h(P(0, Y_R + CLIP_W)[0], P(0, Y_R)[0], P(XB - CLIP_D / 2, 0)[1] + 16, f"{mm(CLIP_W)}", ext_from=P(XB - CLIP_D / 2, 0)[1], size=9)
plan_bottom = row3 + 20

# ---------------------------------------------------------------- front elevation
E0 = plan_bottom + 60
SE = 0.70
text(30, E0 - 20, "2. Front elevation, looking at the wall from the robot (the robot's left on the left). Heights above the table top.", 13, anchor="start")
ZT = E0 + WALL_H * 1000 * SE + 10  # page y of the table top


def E(y: float, z: float):
    return CX - y * 1000 * SE, ZT - z * 1000 * SE


line(E(-WALL_W / 2 - 0.08, 0)[0], ZT, E(WALL_W / 2 + 0.08, 0)[0], ZT, INK, 1.6)
text(E(WALL_W / 2 + 0.06, 0)[0], ZT + 14, "table top, z = 0", 10, anchor="start")
rect(*E(-WALL_W / 2, 0), *E(WALL_W / 2, WALL_H), fill=WOOD, op=0.35)
zb = ZB  # TOP = 0
rect(*E(y_case0, zb - O["case_bat_h"] / 2), *E(Y_M, zb + O["case_bat_h"] / 2), fill=WOOD)
rect(*E(y_case0, zb - O["case_h"] / 2), *E(Y_M, zb + O["case_h"] / 2), fill=CASE)
zs0 = Z_CLIP + CLIP_H / 2 - O["stand_h"] / 2
rect(*E(Y_R, zs0), *E(Y_R + O["stand_w"], zs0 + O["stand_h"]), fill=WOOD)
rect(*E(Y_R, Z_CLIP), *E(Y_R + CLIP_W, Z_CLIP + CLIP_H), fill=CLIPC)
rect(*E(Y_SEAT - H_L / 2, zb - H_W / 2), *E(Y_SEAT + H_L / 2, zb + H_W / 2), fill=HOUS, op=0.3, sw=0.6, dash="3,2")
rect(*E(Y_M, zb - H_W / 2), *E(Y_HAND - H_L / 2, zb + H_W / 2), fill=BELT, op=0.8, sw=0.6)
rect(*E(Y_HAND - H_L / 2, zb - H_W / 2), *E(Y_HAND + H_L / 2, zb + H_W / 2), fill="#777", op=0.7)
text(*E((Y_M + Y_HAND) / 2, zb + 0.004), f"belt, {mm(H_W)} wide, drawn out", 10)
text(*E(Y_HAND, zb + H_W / 2 + 0.01), "end at hand-over", 10)
text(*E(y_case0 + O["case_len"] / 2, zb + O["case_bat_h"] / 2 + 0.012), "896 unit on batten", 10)
text(*E(Y_R + 0.04, zs0 + O["stand_h"] + 0.012), "898 clip on stand-off", 10)
text(*E(Y_SEAT - 0.03, zb - H_W / 2 - 0.016), "end seated (dashed)", 9)
xe = E(WALL_W / 2, 0)[0] - 30
dim_v(xe, ZT, E(0, WALL_H)[1], f"board {mm(WALL_H)}", side=-1)
xe2 = E(y_case0, 0)[0] + 20
dim_v(xe2, ZT, E(0, zb)[1], f"belt centre {mm(zb)}", ext_from=E(y_case0, 0)[0], side=1)
dim_v(E(Y_HAND - H_L / 2 - 0.06, 0)[0], E(0, zb - H_W / 2)[1], E(0, zb + H_W / 2)[1], f"{mm(H_W)}", side=1, size=10)
dim_v(E(Y_R + O["stand_w"], 0)[0] - 14, E(0, Z_CLIP)[1], E(0, Z_CLIP + CLIP_H)[1], f"clip {mm(CLIP_H)}, foot at {mm(Z_CLIP)}", side=-1, size=10)
dim_v(E(y_case0, 0)[0] + 110, E(0, zb - O["case_h"] / 2)[1], E(0, zb + O["case_h"] / 2)[1], f"{mm(O['case_h'])} (est.)", ext_from=E(y_case0, 0)[0], side=1, size=10)
dim_h(E(Y_M, 0)[0], E(y_case0, 0)[0], ZT + 24, f"{mm(O['case_len'])}", ext_from=E(0, zb - O["case_bat_h"] / 2)[1], size=10)
dim_h(E(Y_R + O["stand_w"], 0)[0], E(Y_R, 0)[0], ZT + 24, f"{mm(O['stand_w'])}", ext_from=E(0, zs0)[1], size=10)
elev_bottom = ZT + 40

# ---------------------------------------------------------------- the 898 clip and the end fitting
D0 = elev_bottom + 100
SD = 4.0  # px per mm
text(30, D0 - 66, "3. The 898 wall clip seen from the robot (left; its slot opens toward the unit, on the right) and a plan section across its slot (right), 4x scale.", 13, anchor="start")
gx0 = 300


def G(y: float, z: float):
    """Elevation detail: y to the page's left as in view 2, z up; the clip's bottom at the page's D0 + 470."""
    return gx0 + (Y_R + CLIP_W - y) * 1000 * SD, D0 + 470 - (z - Z_CLIP) * 1000 * SD


rect(*G(Y_R, Z_CLIP), *G(Y_R + CLIP_W, Z_CLIP + CLIP_H), fill="#d9d9dc", sw=1.0)
rect(*G(Y_R, Z_FLOOR), *G(Y_R + LIP_T, Z_LIPTOP), fill="#9aa0a6", sw=0.8)  # the lips (the near one in view)
rect(*G(Y_R, Z_LIPTOP), *G(Y_R + LIP_T + BORE_Y, Z_HEAD), fill="white", sw=0.8)  # the entry window
rect(*G(Y_R + LIP_T, Z_FLOOR), *G(Y_R + LIP_T + BORE_Y, Z_LIPTOP), fill="white", sw=0.8, dash="2,2")  # the channel behind the lips
for zs in (Z_CLIP + (CLIP_H - O["screws"]) / 2, Z_CLIP + (CLIP_H + O["screws"]) / 2):
    cx_, cy_ = G(Y_R + CLIP_W / 2 + 0.002, zs)
    out.append(f'<circle cx="{cx_:.1f}" cy="{cy_:.1f}" r="8" fill="none" stroke="#222" stroke-width="1"/>')
# the bead seated (solid) and at the entry height (dotted), with the drop
for zc, dash, lab in ((zb, None, "bead seated"), (zb + LIFT, "2,2", "bead at entry")):
    rect(*G(Y_R + LIP_T, zc - FIN_H / 2), *G(Y_R + LIP_T + BEAD_L, zc + FIN_H / 2), fill=HOUS, op=0.6, sw=0.8, dash=dash)
    rect(*G(Y_R - GAP, zc - FIN_H / 2), *G(Y_R + LIP_T, zc + FIN_H / 2), fill=HOUS, op=0.3, sw=0.6, dash=dash)
    text(G(Y_R, zc)[0] + 14, G(0, zc)[1] + 4, lab, 10, anchor="start", fill="#555")
ax_, ay0 = G(Y_R, zb + LIFT)[0] + 40, G(Y_R, zb + LIFT)[1]
line(ax_ + 70, ay0, ax_ + 70, G(0, zb)[1], DIM, 1.4)
out.append(f'<path d="M{ax_ + 70:.1f},{G(0, zb)[1]:.1f} l-5,-10 l10,0 z" fill="{DIM}"/>')
text(ax_ + 78, (ay0 + G(0, zb)[1]) / 2, f"lowered {mm(LIFT)}", 10, anchor="start", fill=DIM)
xd = G(Y_R + CLIP_W, 0)[0] - 18
dim_v(xd, G(0, Z_CLIP)[1], G(0, Z_CLIP + CLIP_H)[1], f"{mm(CLIP_H)}", side=-1, rot=True)
xd2 = G(Y_R, 0)[0] + 200
for za, zb_, lab in ((Z_CLIP, Z_FLOOR, f"foot {mm(FOOT)}"), (Z_FLOOR, Z_LIPTOP, f"channel {mm(CHAN_H)}"), (Z_LIPTOP, Z_HEAD, f"window {mm(WIN_H)}"),
                     (Z_HEAD, Z_CLIP + CLIP_H, f"head {mm(Z_CLIP + CLIP_H - Z_HEAD)}")):
    dim_v(xd2, G(0, za)[1], G(0, zb_)[1], lab, ext_from=G(Y_R, 0)[0], side=1, size=10)
dim_v(xd - 34, G(0, Z_CLIP + (CLIP_H - O["screws"]) / 2)[1], G(0, Z_CLIP + (CLIP_H + O["screws"]) / 2)[1], f"screws {mm(O['screws'])} apart", side=-1, rot=True, size=10)
dim_h(G(Y_R + CLIP_W, 0)[0], G(Y_R, 0)[0], G(0, Z_CLIP)[1] + 22, f"{mm(CLIP_W)}", ext_from=G(0, Z_CLIP)[1], size=10)
text(G(Y_R, 0)[0] + 14, G(0, Z_CLIP + 0.004)[1], "to the unit (right)", 10, anchor="start")
# plan section across the slot, at the channel's height
SX = 8.0
hx0, hy0 = 820, D0 + 40


def Hs(x: float, y: float):
    """Plan section, 8x: y (toward the receiver) down the page, x (toward the wall) to the right."""
    return hx0 + (x - XB) * 1000 * SX, hy0 + (y - (Y_R - 0.010)) * 1000 * SX


rect(*Hs(XB - CLIP_D / 2, Y_R), *Hs(XB + CLIP_D / 2, Y_R + CLIP_W), fill="#d9d9dc", sw=1.0)
rect(*Hs(XB - SLIT_X / 2, Y_R), *Hs(XB + SLIT_X / 2, Y_R + LIP_T), fill="white", sw=0.8)
rect(*Hs(XB - BORE_X / 2, Y_R + LIP_T), *Hs(XB + BORE_X / 2, Y_R + LIP_T + BORE_Y), fill="white", sw=0.8)
rect(*Hs(XB - H_T / 2, Y_R - 0.010), *Hs(XB + H_T / 2, Y_R - GAP), fill=HOUS, op=0.35, sw=0.6)
rect(*Hs(XB - NECK_X / 2, Y_R - GAP), *Hs(XB + NECK_X / 2, Y_R + LIP_T), fill=HOUS, op=0.6, sw=0.6)
rect(*Hs(XB - BEAD_X / 2, Y_R + LIP_T), *Hs(XB + BEAD_X / 2, Y_R + LIP_T + BEAD_L), fill=HOUS, op=0.6, sw=0.6)
rect(*Hs(XS, Y_R), *Hs(XS + 0.006, Y_R + CLIP_W), fill=WOOD, sw=0.6)
text(Hs(XS + 0.003, 0)[0], Hs(0, Y_R + CLIP_W)[1] + 14, "stand-off", 9)
text(Hs(XB, 0)[0], Hs(0, Y_R - 0.010)[1] - 6, "end fitting front (belt pull up the page)", 9, fill="#555")
dim_h(Hs(XB - CLIP_D / 2, 0)[0], Hs(XB + CLIP_D / 2, 0)[0], Hs(0, Y_R + CLIP_W)[1] + 34, f"{mm(CLIP_D)}", ext_from=Hs(0, Y_R + CLIP_W)[1], size=10)
dim_h(Hs(XB - SLIT_X / 2, 0)[0], Hs(XB + SLIT_X / 2, 0)[0], Hs(0, Y_R)[1] - 4, f"gap {mm(SLIT_X)}", size=9)
dim_h(Hs(XB - BORE_X / 2, 0)[0], Hs(XB + BORE_X / 2, 0)[0], Hs(0, Y_R + LIP_T + BORE_Y)[1] + 14, f"channel {mm(BORE_X)}", size=9)
dim_v(Hs(XB - CLIP_D / 2, 0)[0] - 50, Hs(0, Y_R)[1], Hs(0, Y_R + CLIP_W)[1], f"{mm(CLIP_W)}", ext_from=Hs(XB - CLIP_D / 2, 0)[0], side=-1, size=10)
dim_v(Hs(XB - CLIP_D / 2, 0)[0] - 16, Hs(0, Y_R)[1], Hs(0, Y_R + LIP_T)[1], f"lips {mm(LIP_T)}", ext_from=Hs(XB - CLIP_D / 2, 0)[0], side=-1, size=9)
dim_v(Hs(XB - CLIP_D / 2, 0)[0] - 16, Hs(0, Y_R + LIP_T)[1], Hs(0, Y_R + LIP_T + BORE_Y)[1], f"{mm(BORE_Y)}", side=-1, size=9)
text(hx0 - 200, Hs(0, Y_R + CLIP_W)[1] + 64, f"bead {BEAD_X * 1000:.0f} x {BEAD_L * 1000:.1f}, neck {NECK_X * 1000:.1f} thick; 1 mm chamfers on the lips' top edges", 10, anchor="start")

# ---------------------------------------------------------------- end housing profile
F0 = D0 + 470 + 140
text(30, F0 - 70, "4. Belt end, side view from the robot (its front, with the end fitting, on the left as in view 2; the belt on the right), 4x scale.", 13, anchor="start")
fx0, SF = 260, 4.0
TOPH = H_W / 2


def F(y: float, z: float):
    return fx0 + (H_L / 2 + NECK_L + BEAD_L - y) * 1000 * SF, F0 + (TOPH - z) * 1000 * SF


rect(*F(-H_L / 2, -H_W / 2), *F(H_L / 2, H_W / 2), fill=HOUS, op=0.45, sw=1.1)
rect(*F(H_L / 2, -FIN_H / 2), *F(H_L / 2 + NECK_L, FIN_H / 2), fill=HOUS, op=0.3, sw=0.8)
rect(*F(H_L / 2 + NECK_L, -FIN_H / 2), *F(H_L / 2 + NECK_L + BEAD_L, FIN_H / 2), fill=HOUS, op=0.7, sw=0.8)
rect(*F(-H_L / 2, -H_W / 2 + 0.001), *F(-H_L / 2 - 0.02, H_W / 2 - 0.001), fill=BELT, op=0.8, sw=0.6)
text(F(-H_L / 2 - 0.01, 0)[0], F(0, 0)[1] + 4, "belt", 10, rot=-90)
dim_h(F(H_L / 2, 0)[0], F(-H_L / 2, 0)[0], F(0, -H_W / 2)[1] + 50, f"{mm(H_L)}", ext_from=F(0, -H_W / 2)[1])
dim_h(F(H_L / 2 + NECK_L + BEAD_L, 0)[0], F(H_L / 2, 0)[0], F(0, -H_W / 2)[1] + 26, f"neck {NECK_L * 1000:.1f} + bead {BEAD_L * 1000:.1f}", ext_from=F(0, -FIN_H / 2)[1], size=10)
dim_v(F(H_L / 2 + NECK_L + BEAD_L, 0)[0] - 20, F(0, -FIN_H / 2)[1], F(0, FIN_H / 2)[1], f"{mm(FIN_H)}", ext_from=F(H_L / 2 + NECK_L + BEAD_L, 0)[0], side=-1)
dim_v(F(-H_L / 2, 0)[0] + 110, F(0, -H_W / 2)[1], F(0, H_W / 2)[1], f"{mm(H_W)}", ext_from=F(-H_L / 2, 0)[0], side=1)
text(F(-H_L / 2, 0)[0] + 150, F(0, 0)[1], f"{mm(H_T)} thick, {C['E_MASS'] * 1000:.0f} g", 11, anchor="start")
D_bottom = F0 + H_W * 1000 * SF + 110
note = ["All numbers come from task.py, except the arm bases and reach (Robocurve station). est. = estimate, no published figure.",
        "Clip outline, screw spacing: Tensabarrier 898 drawing. Slot, lips and end fitting: estimates; measure the real clip and end. Belt pull 8.9 N toward the unit's mouth. No latch."]
for k, s in enumerate(note):
    text(30, D_bottom - 20 + 16 * k, s, 11, anchor="start", fill="#444")
H = D_bottom + 20
svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W_PX}" height="{H:.0f}" viewBox="0 0 {W_PX} {H:.0f}">\n'
       f'<rect width="100%" height="100%" fill="white"/>\n<defs><clipPath id="plan"><rect x="0" y="{PT + BOARD_T * 1000 * S:.1f}" width="{W_PX}" height="{plan_bottom - PT:.1f}"/></clipPath></defs>\n' + "\n".join(out) + "\n</svg>\n")
(HERE / "rig_layout.svg").write_text(svg)
print(f"rig_layout.svg: belt plane x {mm(XB)}, board face x {mm(WALL_X)}, belt centre z {mm(ZB)}, mouths y {mm(Y_M)} and {mm(Y_R)} "
      f"({mm(Y_R - Y_M)} apart), batten {mm(BATTEN)}, clip {mm(CLIP_H)} x {mm(CLIP_W)} x {mm(CLIP_D)}, foot at z {mm(Z_CLIP)}, stand-off {mm(WALL_X - XS)}")
