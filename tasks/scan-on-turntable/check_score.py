"""The score lines and the coverage geometry on hand-built cases, with no simulator: python3 check_score.py.
Runs `judge`, `surface_points`, `hidden`, `view` and `part_distance` from task.py, read from its source."""

import ast
import math
import pathlib

import numpy as np

src = pathlib.Path(__file__).with_name("task.py").read_text()
ns = {"np": np, "math": math, "os": __import__("os")}
FUNCS = {"judge", "surface_points", "hidden", "view", "part_distance", "turntable_distance", "button_pressed"}
SKIP = {"SCENE", "INSTRUCTION", "LOOKS", "OBJECTS", "SCAN_", "TOP", "DT", "PART_Z", "PART_MID", "STROKES", "SCAN_REST", "SCAN_PTS", "LENS_DOWN_R", "LENS_DOWN_AT", "HANDLE_GRIP"}
for node in ast.parse(src).body:
    if isinstance(node, ast.Assign):
        names = {n.id for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)}
        if names & SKIP or "m." in ast.get_source_segment(src, node) or "d." in ast.get_source_segment(src, node):
            continue
        exec(ast.get_source_segment(src, node), ns)
    elif isinstance(node, ast.FunctionDef) and node.name in FUNCS:
        exec("from __future__ import annotations\n" + ast.get_source_segment(src, node), ns)
    if isinstance(node, ast.FunctionDef) and node.name == "surface_points":
        ns["SURFACE_P"], ns["SURFACE_N"] = ns["surface_points"]()
judge, HOLD = ns["judge"], ns["HOLD_STEPS"]
assert HOLD == 20 and ns["SUCCESS_HOLD_S"] == 0.0, "the scorer counts its own 1 s hold (20 steps at 20 Hz); the harness adds none"

bad = 0


def check(name, got, want):
    global bad
    good = got == want
    bad += not good
    print(f"{'ok ' if good else 'BAD'} {name}: {got}")


# (coverage, top coverage, knocked, struck, hands and scanner clear, held steps) -> (full, score)
CASES = {
    "96 % covered, hands clear, held 1 s": ((0.96, 0.97, False, False, True, HOLD), (True, 1.0)),
    "the same, held 0.95 s": ((0.96, 0.97, False, False, True, HOLD - 1), (True, 0.5)),
    "96 % covered, the scanner still within 3 cm of the part": ((0.96, 0.97, False, False, False, HOLD), (False, 0.5)),
    "94 % covered": ((0.94, 0.90, False, False, True, HOLD), (False, 0.5)),
    "70 % covered, half the top": ((0.70, 0.50, False, False, True, HOLD), (False, 0.5)),
    "77 % covered, none of the top (a scanner held still at a low view)": ((0.77, 0.0, False, False, True, HOLD), (False, 0.25)),
    "80 % covered, 49 % of the top": ((0.80, 0.49, False, False, True, HOLD), (False, 0.25)),
    "69 % covered, all the top": ((0.69, 1.0, False, False, True, HOLD), (False, 0.25)),
    "40 % covered": ((0.40, 0.0, False, False, True, HOLD), (False, 0.25)),
    "39 % covered": ((0.39, 0.0, False, False, True, HOLD), (False, 0.0)),
    "95 % covered, but the part was knocked off centre on the way": ((0.95, 0.95, True, False, True, HOLD), (False, 0.0)),
    "95 % covered, but the scanner touched the part on the way": ((0.95, 0.95, False, True, True, HOLD), (False, 0.0)),
}
for name, (args, want) in CASES.items():
    check(name, judge(*args), want)

# Geometry: a scanner square on to one long side of the base and the housing's end face, 25 cm off, captures that side and nothing behind it.
P, N = ns["SURFACE_P"], ns["SURFACE_N"]
SHAFT_H = ns["SHAFT_H"]
cam = np.array([0.0, -0.25, SHAFT_H])
inside, seen = ns["view"](cam, np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))
check("square on to the -y side: every point captured faces -y or is on the top or the round", bool(np.all(N[seen][:, 1] <= 1e-9)), True)
check("square on to the -y side: no point on the +y side captured", int(np.sum(seen & (N[:, 1] > 0.5))), 0)
check("square on to the -y side: the whole -y end face captured", bool(np.all(seen[N[:, 1] < -0.99])), True)
too_near = ns["view"](np.array([0.0, -0.12, SHAFT_H]), np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0]))[1]
check("12 cm off, inside the scanner's 200 mm near limit: nothing captured", int(too_near.sum()), 0)
behind = ns["view"](np.array([0.0, -0.25, SHAFT_H]), np.array([0.0, -1.0, 0.0]), np.array([0.0, 0.0, 1.0]))[1]
check("looking away from the part: nothing captured", int(behind.sum()), 0)
cam = np.array([0.20, 0.0, 0.22])
look = -cam / np.linalg.norm(cam)
steep = ns["view"](cam, look, np.cross(look, np.array([0.0, 1.0, 0.0])))[1]
check("from +x, 45 degrees up: the base's top beyond the housing hidden, the near one captured",
      (int(np.sum(steep & (N[:, 2] > 0.99) & (P[:, 0] < 0))), bool(np.sum(steep & (N[:, 2] > 0.99) & (P[:, 0] > 0)) > 100)), (0, True))
d = ns["part_distance"](np.array([[0.0, 0.0, SHAFT_H + ns["HOUSING_R"] + 0.001], [0.0, -0.05, 0.0], [0.0, 0.0, 0.01]]))
check("distance to the part: 1 mm over the housing's top, 31 mm off its side, inside it", [round(float(v), 4) for v in d], [0.001, 0.031, 0.0])
under = ns["view"](np.array([0.0, -0.20, -0.02]), np.array([0.0, 1.0, 0.0]) / 1.0, np.array([0.0, 0.0, 1.0]))[1]
check("from 2 cm below the platter's top, 20 cm off: the platter hides the whole part", int(under.sum()), 0)
td = ns["turntable_distance"](np.array([[0.0, 0.10, 0.031], [0.0, 0.16, 0.02], [0.0, 0.0, 0.02]]))
check("distance to the turntable: 1 mm over the platter, 1 cm beyond its rim, inside it", [round(float(v), 4) for v in td], [0.001, 0.01, 0.0])

def ring(elev_deg, dist):
    """What a scanner held upright at one elevation sees as the part turns a full turn under it (72 views, 5 degrees apart)."""
    got = np.zeros(len(P), bool)
    e = math.radians(elev_deg)
    mid = np.array([0.0, 0.0, 0.035])
    for k in range(72):
        a = math.radians(5.0 * k)
        look = np.array([-math.cos(e) * math.cos(a), -math.cos(e) * math.sin(a), -math.sin(e)])
        up = np.array([0.0, 0.0, 1.0]) - look[2] * look
        got |= ns["view"](mid - dist * look, look, up / np.linalg.norm(up))[1]
    return got


TOP_PTS = ns["TOP_PTS"]
low, steep = ring(15.0, 0.34), ring(60.0, 0.30)
both = low | steep
for name, g in (("a full turn at 15 degrees up", low), ("a full turn at 60 degrees up", steep), ("both turns", both)):
    print(f"{name}: {100 * g.mean():.1f} % covered, {100 * g[TOP_PTS].mean():.1f} % of the top")
check("a full turn held still at 15 degrees up (a tripod): 0.25", ns["judge"](float(low.mean()), float(low[TOP_PTS].mean()), False, False, True, HOLD)[1], 0.25)
check("a full turn at 15 degrees and one at 60 degrees: full score", ns["judge"](float(both.mean()), float(both[TOP_PTS].mean()), False, False, True, HOLD), (True, 1.0))
# The press: a fingertip point at button 4's face (pressed, it stops there: -0.2 to +0.6 mm logged); hovering 1.6 mm off, off-centre or through, no.
bp = ns["button_pressed"]
for name, along, radial, want in (("pressed, at the face", 0.0003, 0.0024, True), ("pressed hard, 2 mm in", -0.002, 0.001, True),
                                  ("hovering 1.6 mm off (the hover run)", 0.0016, 0.0005, False), ("through the handle, 12 mm in", -0.012, 0.0, False),
                                  ("at the face but 7 mm off the button's middle", 0.0, 0.007, False)):
    check(f"button 4, {name}", bp([along], [radial]), want)

# A scanner held still at one height through a full turn, as on a tripod: at 40 degrees down it sees much of the sides and the top (0.5); no
# single height reaches 95 %.
best = max(((e, d) for e in range(10, 81, 5) for d in (0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60)), key=lambda ed: ring(ed[0], ed[1]).mean())
g = ring(*best)
print(f"best still height: {best[0]} degrees down at {best[1]:.2f} m: {100 * g.mean():.1f} %, {100 * g[TOP_PTS].mean():.1f} % of the top")
check("one still height, the best: at most 0.5", ns["judge"](float(g.mean()), float(g[TOP_PTS].mean()), False, False, True, HOLD)[1] <= 0.5, True)
g40 = ring(40.0, 0.50)
check("still at 40 degrees down, 0.50 m: 0.5", ns["judge"](float(g40.mean()), float(g40[TOP_PTS].mean()), False, False, True, HOLD)[1], 0.5)


def up_of(look):
    u = np.array([0.0, 0.0, 1.0]) - look[2] * look
    return u / np.linalg.norm(u)


# Left on the table and aimed at the part, its lens 2 to 5 cm over the table (the part's frame: the table is 3 cm below z 0), through a full turn.
table = np.zeros(len(P), bool)
for dist in np.arange(0.20, 0.40, 0.02):
    for lz in (-0.006, 0.024):
        got = np.zeros(len(P), bool)
        for k in range(36):
            a = math.radians(10 * k)
            cam = np.array([-dist * math.cos(a), -dist * math.sin(a), lz])
            look = np.array([0.0, 0.0, 0.035]) - cam
            look /= np.linalg.norm(look)
            got |= ns["view"](cam, look, up_of(look))[1]
        table = got if got.mean() > table.mean() else table
check(f"left on the table, aimed at the part, a full turn: 0.25 ({100 * table.mean():.1f} %, {100 * table[TOP_PTS].mean():.1f} % of the top)",
      ns["judge"](float(table.mean()), float(table[TOP_PTS].mean()), False, False, True, HOLD)[1], 0.25)
# No turning: every view from one side, level to 80 degrees down, 25 to 50 cm, along the part's length.
side = np.zeros(len(P), bool)
for e in range(0, 81, 5):
    for dx in np.arange(-0.15, 0.151, 0.03):
        for dist in (0.25, 0.35, 0.50):
            er = math.radians(e)
            look = np.array([0.0, math.cos(er), -math.sin(er)])
            cam = np.array([dx, 0.0, 0.035]) - dist * look
            if cam[2] >= 0.0:
                side |= ns["view"](cam, look, up_of(look))[1]
check(f"no turning, every view from one side: 0.25 ({100 * side.mean():.1f} %)", ns["judge"](float(side.mean()), float(side[TOP_PTS].mean()), False, False, True, HOLD)[1], 0.25)
print(f"{len(P)} surface points, {int(ns['TOP_PTS'].sum())} of them on the top")
print("all as stated" if not bad else f"{bad} case(s) differ from the scoring text")
raise SystemExit(1 if bad else 0)
