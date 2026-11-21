"""The back of a 1U rack server in a 19-inch rack, seen from the hot aisle. The server is fully pushed in on its
tool-less slide rails; the two rear rack posts stand on the bench and the server sits 58 mm above it. Behind the
server, at the rails' rear ends, are the rails' cable-management-arm brackets: on the robot's left rail an inner
bracket (on the rail member that moves with the server) and an outer bracket (on the member fixed to the rack), each
with a short flat tab pointing toward the robot; a yellow latch tooth stands up from each tab. A steel support bar
spans the rack between the rails just below the brackets' height. Near its right end a blue stop bracket stands
upright on the support bar. In front of the rack a black two-arm cable management arm (CMA) lies loose on the bench,
its two cable baskets joined by a hinge at the robot's right; its other arm swings freely on that hinge and lies
partly open. A mounting clip ends each basket at the robot's left.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 0.05  # score() itself needs the end state let go and still for 1 s
EPS = 1e-6

STEEL, ZINC, DARK, BLACK = (0.55, 0.57, 0.60), (0.72, 0.73, 0.74), (0.16, 0.17, 0.18), (0.06, 0.06, 0.07)
BLUE, YELLOW, GREEN = (0.10, 0.35, 0.80), (0.95, 0.78, 0.08), (0.10, 0.55, 0.20)

# -- the rack rear ------------------------------------------------------------------------------------------------------
# World frame: x away from the robot (toward the rack's front), y to the robot's left, z up. The server's rear face is
# at x 0.480, leaving 43 mm behind it for plugged-in cables' plug bodies; the tabs' top faces are 90 mm above the bench.
ZT = TOP + 0.090  # top face of both CMA tabs
X_IN, Y_IN = 0.437, 0.188  # inner bracket: the stop face its clip meets, and the tab's centre line
X_OUT, Y_OUT = 0.387, 0.218  # outer bracket
TAB_L, TAB_W, TAB_T = 0.030, 0.020, 0.002  # each tab: 30 mm long, 20 mm wide, 2 mm sheet steel
BAR_TOP = ZT - 0.033  # support bar's top face (the CMA baskets' bottoms are 3 mm above it when level)
GATE_PIVOT = (0.336, -0.220)  # stop bracket hinge (vertical axis)
GATE_LEN = 0.043
RACK_O = np.array([0.40, 0.0, TOP])  # the rack part's origin


def _wbox(lo, hi, color, origin=RACK_O, collide=True, friction=None):
    """A box given by its world corners, in a part frame whose origin is ``origin``."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in (lo + hi) / 2 - origin),
                     color=color, collide=collide, friction=friction)


def _mirror(lo, hi):
    return (lo[0], -hi[1], lo[2]), (hi[0], -lo[1], hi[2])


RAIL_Z = (ZT - 0.020, ZT + 0.008)
BRACKET_Z = (ZT - 0.013, ZT + 0.011)
RACK_SHAPES = [
    # 1U server, truncated 240 mm in front of its rear face (fixed; never touched)
    _wbox((0.480, -0.2172, ZT - 0.032), (0.720, 0.2172, ZT + 0.011), DARK),
    _wbox((0.4775, -0.2100, ZT - 0.028), (0.480, -0.1300, ZT + 0.007), BLACK),  # power supply 1 (rear face)
    _wbox((0.4775, -0.1250, ZT - 0.028), (0.480, -0.0450, ZT + 0.007), BLACK),  # power supply 2
    _wbox((0.4775, 0.0000, ZT - 0.020), (0.480, 0.1200, ZT + 0.000), (0.25, 0.26, 0.28)),  # I/O panel
]
for side in (1, -1):
    def m(lo, hi, s=side):
        return (lo, hi) if s > 0 else _mirror(lo, hi)
    RACK_SHAPES += [
        _wbox(*m((0.480, 0.2172, RAIL_Z[0]), (0.720, 0.2215, RAIL_Z[1])), ZINC),  # inner rail member
        _wbox(*m((0.455, 0.2215, RAIL_Z[0]), (0.720, 0.2275, RAIL_Z[1])), STEEL),  # outer rail member
        _wbox(*m((0.455, 0.2305, TOP), (0.475, 0.2505, TOP + 0.240)), BLACK),  # rear rack post
        _wbox(*m((0.3875, 0.2245, BRACKET_Z[0]), (0.455, 0.2305, BRACKET_Z[1])), ZINC),  # outer rail rear bracket arm
    ]
RACK_SHAPES += [
    # inner bracket (left rail, inner member): stop block and tab
    _wbox((X_IN, Y_IN - 0.0125, BRACKET_Z[0]), (0.480, 0.2172, BRACKET_Z[1]), ZINC),
    _wbox((X_IN - TAB_L, Y_IN - TAB_W / 2, ZT - TAB_T), (X_IN, Y_IN + TAB_W / 2, ZT), ZINC),
    # outer bracket (left rail, outer member): stop block and tab
    _wbox((X_OUT, Y_OUT - 0.012, BRACKET_Z[0]), (X_OUT + 0.0045, 0.2305, BRACKET_Z[1]), ZINC),
    _wbox((X_OUT - TAB_L, Y_OUT - TAB_W / 2, ZT - TAB_T), (X_OUT, Y_OUT + TAB_W / 2, ZT), ZINC),
    # support bar (installed: pin in the left rail's slot, bracket on the right rail's outer bracket) and the stop
    # bracket's base plate
    _wbox((0.375, -0.2245, BAR_TOP - 0.005), (0.405, 0.2245, BAR_TOP), STEEL),
    _wbox((0.330, -0.2245, BAR_TOP - 0.005), (0.375, -0.168, BAR_TOP), STEEL),
]


def _tooth(name, x_stop, y_mid):
    """A sprung latch tooth on a tab: a 2 mm plate standing 2.5 mm proud of the tab's top, hinged in the tab
    10 mm from the stop face. A clip pushed on folds it flat into the tab; at the clip's window it springs up behind
    the window's front edge, so the clip cannot be pulled off."""
    pivot = np.array([x_stop - 0.010, y_mid, ZT - 0.001])
    return parts.Hinge(
        name, pivot=tuple(float(v) for v in pivot - RACK_O), axis=(0.0, 1.0, 0.0), lower=0.0, upper=1.6, mass=0.02,
        spring=0.005, damping=0.0005, max_effort=5.0,
        shapes=(_wbox(pivot + (-0.001, -0.008, 0.0), pivot + (0.001, 0.008, 0.0035), YELLOW, friction=(0.3, 0.25)),))


GATE_CLOSED_SHAPES = (
    _wbox((GATE_PIVOT[0] - 0.003, GATE_PIVOT[1], BAR_TOP + 0.0005), (GATE_PIVOT[0] + 0.003, GATE_PIVOT[1] + GATE_LEN,
                                                                      ZT + 0.004), BLUE),
)
RACK = parts.Part(
    "rack", pos=tuple(float(v) for v in RACK_O), shapes=RACK_SHAPES, contact_offset=0.002,
    joints=[
        _tooth("inner_latch", X_IN, Y_IN),
        _tooth("outer_latch", X_OUT, Y_OUT),
        parts.Hinge("stop_bracket", pivot=(GATE_PIVOT[0] - RACK_O[0], GATE_PIVOT[1] - RACK_O[1], 0.0),
                    axis=(0.0, 0.0, 1.0), lower=0.0, upper=1.65, mass=0.03, spring=0.10, damping=0.01, max_effort=5.0,
                    shapes=GATE_CLOSED_SHAPES),
    ],
)
GATE_OPEN_AT = 0.8  # rad: past this the stop bracket's detent carries it fully open, short of it fully shut

# -- the cable management arm (CMA) -------------------------------------------------------------------------------------
# CMA frame: origin on the elbow hinge's axis at the tabs' top-face height, axes as the world's when installed. The
# inner basket (the CMA's body) lies on +x of the hinge, the outer basket on -x; both run along +y to their clips.
P_INST = np.array([0.3895, -0.205, ZT])  # where the CMA frame sits when installed
OPEN0 = math.radians(15.0)  # how far the outer arm lies open at the start (it swings freely on its hinge)
B_LO, B_HI = -0.030, 0.010  # basket bottom and top (z, CMA frame)
WALL = 0.0025


def _cbox(lo, hi, color, turn=0.0, collide=True):
    """A box given by CMA-frame corners in the installed (folded) pose, turned ``turn`` rad about the hinge axis."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = (lo + hi) / 2
    q = None
    if turn:
        r = parts._rot((0.0, 0.0, 1.0), turn)
        c = r @ c
        q = (0.0, 0.0, math.sin(turn / 2), math.cos(turn / 2))
    return parts.box(tuple(float(v) for v in hi - lo), at=tuple(float(v) for v in c), color=color, quat=q,
                     collide=collide)


def _flare(p0, p1, y0, y1, thick, away, color, turn=0.0):
    """A lead-in plate whose face toward the tab runs from p0 to p1 (x, z; CMA frame, folded), ``thick`` thick on the
    ``away`` side (+1 up, -1 down), spanning y0..y1; turned ``turn`` rad about the hinge axis."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = (p1 - p0) / np.linalg.norm(p1 - p0)
    n = np.array([-d[1], d[0]]) * (1.0 if d[0] > 0 else -1.0)
    if n[1] * away < 0:
        n = -n
    cx, cz = (p0 + p1) / 2 + n * thick / 2
    phi = math.atan2(-d[1], d[0])  # about +y
    c = np.array([cx, (y0 + y1) / 2, cz])
    qy = (0.0, math.sin(phi / 2), 0.0, math.cos(phi / 2))
    qz = (0.0, 0.0, math.sin(turn / 2), math.cos(turn / 2))
    x1, y1_, z1, w1 = qz
    x2, y2, z2, w2 = qy
    q = (w1 * x2 + x1 * w2 + y1_ * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1_ * w2 + z1 * x2,
         w1 * z2 + x1 * y2 - y1_ * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1_ * y2 - z1 * z2)
    c = parts._rot((0.0, 0.0, 1.0), turn) @ c
    size = (float(np.linalg.norm(p1 - p0)), float(y1 - y0), float(thick))
    return parts.box(size, at=tuple(float(v) for v in c), color=color, quat=tuple(float(v) for v in q))


def _basket(x0, x1, y0, y1, color, turn=0.0):
    return [
        _cbox((x0, y0, B_LO), (x1, y1, B_LO + WALL + 0.0005), color, turn),  # floor
        _cbox((x0, y0, B_LO), (x0 + WALL, y1, B_HI), color, turn),  # robot-side wall
        _cbox((x1 - WALL, y0, B_LO), (x1, y1, B_HI), color, turn),  # far wall
        _cbox((x0, y0, B_LO), (x1, y0 + 0.003, B_HI), color, turn),  # hinge-end wall
        _cbox((x0, y1 - 0.003, B_LO), (x1, y1, B_HI), color, turn),  # clip-end wall
    ]


def _sflare(x0, x1, y_face0, y_face1, thick, away, z0, z1, color, turn=0.0):
    """A sideways lead-in plate: its face toward the tab runs from (x0, y_face0) to (x1, y_face1), ``thick`` thick on
    the ``away`` side (+1: +y), spanning z0..z1; turned ``turn`` rad about the hinge axis."""
    d = np.array([x1 - x0, y_face1 - y_face0])
    ln = float(np.linalg.norm(d))
    d /= ln
    n = np.array([-d[1], d[0]])
    if n[1] * away < 0:
        n = -n
    cx, cy = (np.array([x0, y_face0]) + np.array([x1, y_face1])) / 2 + n * thick / 2
    a = math.atan2(d[1], d[0]) + turn
    c = parts._rot((0.0, 0.0, 1.0), turn) @ np.array([cx, cy, (z0 + z1) / 2])
    return parts.box((ln, float(thick), float(z1 - z0)), at=tuple(float(v) for v in c), color=color,
                     quat=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2)))


def _clip(x_stop, y_tab, y_from, color, turn=0.0, side=0.0006):
    """A mounting clip: a 30 mm slot round its tab (``side`` clearance each side, 0.5 mm above and below the tab),
    whose top wall has a window 12.5 to 6 mm behind the clip's front; the blue release ring is its front end. The
    last 6 mm of its walls flare open (top 2.5 mm, bottom 3 mm, sides 3 mm) as a lead-in. ``x_stop`` is the bracket
    stop face it meets, ``y_tab`` the tab's centre line, ``y_from`` the basket end it grows from."""
    front = x_stop - 0.0005
    s0, s1 = y_tab - TAB_W / 2 - side, y_tab + TAB_W / 2 + side  # slot
    t = 0.0025
    y_out = s1 + t
    zb, zt = -TAB_T - 0.0035 - t, 0.003 + t
    out = [
        _cbox((front - 0.030, y_from, 0.0005), (x_stop - 0.0125, y_out, 0.0005 + t), color, turn),  # top, behind window
        _flare((x_stop - 0.006, 0.0005), (front, 0.003), y_from, y_out, t, 1, BLUE, turn),  # top ahead of window
        _cbox((front - 0.030, y_from, -TAB_T - 0.0005 - t), (front - 0.006, y_out, -TAB_T - 0.0005), color, turn),
        _flare((front - 0.006, -TAB_T - 0.0005), (front, -TAB_T - 0.0035), y_from, y_out, t, -1, color, turn),
        _cbox((front - 0.030, s1, zb), (front - 0.006, y_out, zt), color, turn),  # outer side
        _sflare(front - 0.006, front, s1, s1 + 0.003, t, 1, zb, zt, color, turn),
        _cbox((front - 0.030, min(y_from, s0 - t), zb), (front - 0.006, s0, zt), color, turn),  # inner side
        _sflare(front - 0.006, front, s0, s0 - 0.003, t, -1, zb, zt, color, turn),
    ]
    return out


C_IN = (X_IN - P_INST[0], Y_IN - P_INST[1])  # inner clip stop and tab line (CMA frame)
C_OUT = (X_OUT - P_INST[0], Y_OUT - P_INST[1])
IN_X = (0.0025, 0.0475)
OUT_X = (-0.0475, -0.0025)
IN_Y1, OUT_Y1 = C_IN[1] - TAB_W / 2 - 0.0045, C_OUT[1] - TAB_W / 2 - 0.0045  # basket clip-end walls
CMA_SHAPES = (_basket(IN_X[0], IN_X[1], 0.005, IN_Y1, BLACK)
              + _clip(C_IN[0], C_IN[1], IN_Y1, BLACK, side=0.0006)
              + [parts.cylinder(0.006, B_HI - B_LO, at=(0.0, 0.0, (B_LO + B_HI) / 2), color=DARK)])
OUTER_SHAPES = tuple(_basket(OUT_X[0], OUT_X[1], 0.005, OUT_Y1, BLACK, OPEN0)
                     + _clip(C_OUT[0], C_OUT[1], OUT_Y1, BLACK, OPEN0, side=0.002)
                     + [_cbox((-0.0045, -0.004, B_LO), (0.0, 0.008, B_HI), DARK, OPEN0)])
# The hinge has no spring; its friction is modelled as 1 N m s/rad of damping (an estimate: it holds the outer arm's
# angle while the CMA is carried, and a hand swings it at 0.1 rad/s with about 0.3 N).
ELBOW = parts.Hinge("elbow", pivot=(0.0, 0.0, 0.0), axis=(0.0, 0.0, 1.0), lower=-OPEN0 - 0.03, upper=2.0, mass=0.40,
                    spring=0.0, damping=1.0, friction=0.05, shapes=OUTER_SHAPES)
CMA_MASS = 0.45  # inner basket with its clip and the hinge; the outer basket is 0.40 kg (estimates, 0.85 kg in all)
START_A = (float(P_INST[0] - 0.150), float(P_INST[1]), float(TOP - B_LO + 0.0005))
CMA = parts.Part("cma", pos=START_A, mass=CMA_MASS, shapes=CMA_SHAPES, joints=[ELBOW], friction=(0.5, 0.45),
                 contact_offset=0.002)
OBJECTS = [RACK, CMA]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"cma": START_A}))  # one published layout


# -- geometry -----------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _rz(a) -> np.ndarray:
    return parts._rot((0.0, 0.0, 1.0), a)


def _angles(r) -> tuple[float, float]:
    """(yaw, tilt) in degrees of a rotation: yaw of its x axis about the vertical, tilt of its z axis from vertical."""
    yaw = math.degrees(math.atan2(r[1, 0], r[0, 0]))
    tilt = math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))
    return yaw, tilt


MOUTH_IN = np.array([C_IN[0] - 0.0005, C_IN[1], -TAB_T / 2])  # clip front centre, CMA frame, installed
MOUTH_OUT = np.array([C_OUT[0] - 0.0005, C_OUT[1], -TAB_T / 2])
SEAT_IN, SEAT_OUT = P_INST + MOUTH_IN, P_INST + MOUTH_OUT  # their world positions when fully home
PLAY = 0.0035  # the clip may sit back from fully home by up to this and still have the tooth in its window
SIDE_TOL, HEIGHT_TOL, YAW_TOL, TILT_TOL = 0.0025, 0.002, 3.0, 3.0
TOOTH_UP = 0.25  # rad: a tooth turned back less than this stands in the window
GATE_SHUT = 0.15  # rad
ELBOW_FOOT = np.array([0.025, 0.010, B_LO])  # inner basket's underside at its hinge end (CMA frame)
OUTER_FOOT = np.array([-0.025, 0.010, B_LO])  # outer basket's underside at its hinge end (folded frame)


def outer_pose(cma):
    """World (position, rotation) of the outer arm's folded-frame: a point given in the installed (folded) CMA frame
    on the outer arm maps to pos + rot @ point."""
    r_link = _rot(cma.links["elbow"][1])
    return np.asarray(cma.links["elbow"][0], float), r_link @ _rz(OPEN0)


def clip_errors(objects):
    """For each clip, (back, side, height, yaw, tilt): how far its front sits back from fully home along the tab (m,
    negative = past), sideways and up from the tab's line (m), and its basket's yaw and tilt from installed (deg)."""
    cma = objects["cma"]
    p, r = np.asarray(cma.pos, float), _rot(cma.quat_xyzw)
    po, ro = outer_pose(cma)
    out = {}
    for name, (pp, rr, mouth, seat) in {"inner": (p, r, MOUTH_IN, SEAT_IN),
                                         "outer": (po, ro, MOUTH_OUT, SEAT_OUT)}.items():
        e = pp + rr @ mouth - seat
        yaw, tilt = _angles(rr)
        out[name] = (float(-e[0]), float(e[1]), float(e[2]), yaw, tilt)
    return out


def latched(objects) -> dict:
    """Each clip latched: its tooth standing (under 0.25 rad) in the clip's window, i.e. the clip home on the right
    tab to within 3.5 mm, within 2.5 mm sideways and 2 mm in height of the tab's line, basket square (3 deg yaw,
    3 deg tilt)."""
    rack = objects["rack"]
    out = {}
    for name, (back, side, height, yaw, tilt) in clip_errors(objects).items():
        out[name] = bool(-0.001 - EPS <= back <= PLAY + EPS and abs(side) <= SIDE_TOL + EPS
                         and abs(height) <= HEIGHT_TOL + EPS and abs(yaw) <= YAW_TOL + EPS
                         and tilt <= TILT_TOL + EPS and float(rack.joints[f"{name}_latch"]) <= TOOTH_UP + EPS)
    return out


def on_bar(objects) -> bool:
    """Both baskets' hinge ends rest on the support bar: their undersides within 1.5 mm above its top."""
    cma = objects["cma"]
    p, r = np.asarray(cma.pos, float), _rot(cma.quat_xyzw)
    po, ro = outer_pose(cma)
    zi = float((p + r @ ELBOW_FOOT)[2])
    zo = float((po + ro @ OUTER_FOOT)[2])
    return all(BAR_TOP - 0.001 - EPS <= z <= BAR_TOP + 0.0015 + EPS for z in (zi, zo))


def gate_shut(objects) -> bool:
    return float(objects["rack"].joints["stop_bracket"]) <= GATE_SHUT + EPS


def level_now(objects) -> float:
    """The tier the scene shows now, before the let-go, stillness and whole-trial rules."""
    lat = latched(objects)
    n = int(lat["inner"]) + int(lat["outer"])
    if n == 2 and on_bar(objects):
        return 1.0 if gate_shut(objects) else 2.0 / 3.0
    return 1.0 / 3.0 if n >= 1 else 0.0


# -- the trial record ---------------------------------------------------------------------------------------------------
CLEAR = 0.030  # let go: every sampled point of both grippers more than 30 mm from the CMA and the stop bracket
STILL_M, HOLD_S = 0.002, 1.0
FLOOR_Z = TOP - 0.010  # the CMA's lowest point under the bench top: knocked off the bench
_rec = parts.__dict__.setdefault("cma_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", fell=False, gap=None, snap=None, since=None, level=0.0, now=0.0,
                t=0.0, last_t=None)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    cma = objects["cma"]
    p, r = np.asarray(cma.pos, float), _rot(cma.quat_xyzw)
    po, ro = outer_pose(cma)
    j = objects["rack"].joints
    pts = [p + r @ v for v in ((0.0, 0.0, 0.0), (0.0, 0.4, 0.0), (0.05, 0.0, 0.0))]
    pts += [po + ro @ v for v in ((-0.05, 0.0, 0.0), (0.0, 0.4, 0.0))]
    ang = [0.04 * float(j[k]) for k in ("inner_latch", "outer_latch", "stop_bracket")]  # turned at 40 mm
    return np.concatenate([np.ravel(pts), ang])


def _let_go() -> bool:
    g = _rec["gap"]
    return g is not None and g > CLEAR + EPS


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: the closest sampled gripper point to the CMA or the stop bracket (m), or None when
    the grippers could not be read, which fails the trial."""
    _rec["steps"] += 1
    _rec["t"] = float(t)
    try:
        cma, rack = objects["cma"], objects["rack"]
        if not (np.all(np.isfinite(cma.pos)) and np.all(np.isfinite(cma.quat_xyzw))):
            raise ValueError("CMA state unknown")
        for k in ("inner_latch", "outer_latch", "stop_bracket"):
            if not math.isfinite(float(rack.joints[k])):
                raise ValueError(f"{k} unknown")
        if not math.isfinite(float(cma.joints["elbow"])) or "elbow" not in cma.links:
            raise ValueError("CMA hinge unknown")
        if gap is None or not math.isfinite(gap):
            raise ValueError("gripper state unknown")
    except (KeyError, ValueError, TypeError, AttributeError) as e:
        _rec.update(known=False, reason=str(e), since=None, level=0.0)
        return
    if float(cma.low[2]) < FLOOR_Z:
        _rec["fell"] = True
    _rec["gap"] = float(gap)
    lvl = level_now(objects) if _let_go() else 0.0
    snap = _snap(objects)
    if lvl <= 0.0:
        _rec.update(since=None, level=0.0, snap=None, now=0.0)
        return
    moved = math.inf if _rec["snap"] is None else float(np.abs(snap - _rec["snap"]).max())
    if _rec["since"] is None or moved > STILL_M + EPS or lvl != _rec["now"]:
        _rec.update(since=float(t), snap=snap, level=0.0, now=lvl)
    elif float(t) - _rec["since"] >= HOLD_S - EPS:
        _rec["level"] = lvl


def _ok() -> bool:
    return bool(_rec["known"] and not _rec["fell"] and _rec["steps"] > 0)


def score(objects):
    """0 for the whole trial if the CMA ever fell off the bench, or a state could not be read. Otherwise, once both
    grippers are more than 30 mm from the CMA and the stop bracket and the scene has been still for 1 s: 1/3 for one
    clip latched on its own tab (or both without the hinge end resting on the support bar), 2/3 for both latched with
    the CMA's hinge end resting on the support bar, 1.0 for that with the stop bracket shut."""
    if not _ok():
        return 0.0
    lvl = min(_rec["level"], level_now(objects)) if _let_go() else 0.0
    return float(lvl)


def success(objects):
    return score(objects) >= 1.0 - EPS


def distance(objects):
    e = clip_errors(objects)
    return float(max(0.0, e["inner"][0]) + max(0.0, e["outer"][0]))


# -- grippers: hull faces sampled every 3 mm ----------------------------------------------------------------------------
FACE_STEP = 0.003
_HAND_LINKS = ("gripper", "tip_left", "tip_right")


def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    """Points on every face of a convex hull about ``step`` apart, corners included."""
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_face_points(h) for h in hs])
    return _FACES[name]


def _scored_boxes(objects):
    cma, rack = objects["cma"], objects["rack"]
    poses = {"base": (np.asarray(cma.pos, float), _rot(cma.quat_xyzw))}
    poses["elbow"] = (np.asarray(cma.links["elbow"][0], float), _rot(cma.links["elbow"][1]))
    boxes = CMA.world_boxes(poses)
    g = rack.links["stop_bracket"]
    boxes += [b for b in RACK.world_boxes({"stop_bracket": (np.asarray(g[0], float), _rot(g[1]))})]
    return boxes


def gap_to(objects, pts) -> float:
    """Closest distance (m, negative inside) from any point to any shape of the CMA or the stop bracket."""
    from yam_arena import analysis

    pts = np.asarray(pts, float)
    near = np.linalg.norm(pts - np.asarray(objects["cma"].pos, float), axis=1) < 0.80
    if not near.any():
        return 0.10
    pts = pts[near]
    return float(min(analysis.box_signed_distance(pts, c, r, s).min() for _, c, r, s in _scored_boxes(objects)))


def _hand_points(sim) -> np.ndarray | None:
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                r = _rot(poses[i, 3:])
                out.append(link_points(n) @ r.T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- the stop bracket's detent ------------------------------------------------------------------------------------------
# The real stop bracket is pressed down to unlock and turned between two positions. Here a detent: past 0.8 rad it is
# carried to fully open (1.6 rad), short of it back to shut (0), by its 0.1 N m/rad spring.
_gate = parts.__dict__.setdefault("cma_gate", {"target": 0.0, "t": None})


@parts.on_reset
def _new_trial(sim) -> None:
    reset_record()
    _gate.update(target=0.0, t=None)
    try:
        sim.set_target("rack", "stop_bracket", 0.0)
    except Exception:  # noqa: BLE001
        pass


@parts.on_step
def _step(sim) -> None:
    if _rec.get("last_t") == sim.t:  # a second copy of this module: step once
        return
    _rec["last_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    try:
        q = float(objects["rack"].joints["stop_bracket"])
        want = 1.6 if q >= GATE_OPEN_AT else 0.0
        if want != _gate["target"]:
            sim.set_target("rack", "stop_bracket", want)
            _gate["target"] = want
    except Exception:  # noqa: BLE001
        pass
    try:
        pts = _hand_points(sim)
        g = gap_to(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    if os.environ.get("TRACE_OUT") and _rec["steps"] % 2 == 0 and objects:
        try:
            j, cma = objects["rack"].joints, objects["cma"]
            row = dict(t=round(float(sim.t), 3), score=score(objects), level=_rec["level"], now=_rec["now"],
                       known=_rec["known"], reason=_rec["reason"], fell=_rec["fell"],
                       gap=None if _rec["gap"] is None else round(_rec["gap"], 4),
                       latched=latched(objects), on_bar=on_bar(objects),
                       clips={k: [round(v, 4) for v in e] for k, e in clip_errors(objects).items()},
                       inner_latch=round(float(j["inner_latch"]), 3), outer_latch=round(float(j["outer_latch"]), 3),
                       gate=round(float(j["stop_bracket"]), 3), elbow=round(float(cma.joints["elbow"]), 4),
                       cma=np.round(cma.pos, 4).tolist(), low=round(float(cma.low[2]) - TOP, 4))
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("cma.jsonl").open("a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with pathlib.Path(os.environ["TRACE_OUT"]).with_name("cma.jsonl").open("a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open-loop from the start layout the cameras show -------------------------------------------------------------
# The right hand first turns the blue stop bracket open (it stands where the CMA's hinge end has to go). Then both
# hands take the inner basket, one near each end, lift the CMA level, carry it to the rack and push its inner clip onto
# the inner tab until the tooth clicks up. Both let go (the right hand's finger sits between the baskets and would stop
# the outer arm folding), the left pushes the swung-open outer arm shut about the hinge until its clip snaps onto the
# outer tab, the right swings the stop bracket shut in front of the CMA's hinge end, and both arms move clear.
# Positions come from the published start layout only.
GRIP_SEP_OPEN, GRIP_SEP_CLOSED = 0.056, 0.039  # finger separation round the 45 mm basket
GRIP_C = {"left": np.array([0.025, 0.335, 0.004]), "right": np.array([0.025, 0.10, 0.004])}  # CMA frame
LIFT_Z = 0.003  # carry the CMA this far above its installed height
SAG = {"left": 0.0055, "right": 0.0125}  # how far each hand settles below its target carrying the CMA (traces)
PRE = 0.036  # stand-off before the push
OVER = 0.012  # push this far past fully home (the arms are soft: about 4 N against the stop)
PUSH_R = 0.300  # the left hand pushes the outer basket this far out from the hinge
PUSH_W = 0.0475 + 0.004  # ... on its robot-side wall: fingertips (13 mm thick along the push) 2.4 mm into it
HOVER = 0.070
PARK = {"left": (0.25, 0.20, TOP + 0.15), "right": (0.14, -0.27, TOP + 0.20)}


def _sep(s):
    return -s / 2


def _at(c, pivot, yaw):
    """World position of a CMA-frame point with the CMA frame at ``pivot`` turned ``yaw``."""
    return tuple(float(v) for v in np.asarray(pivot, float) + _rz(yaw) @ np.asarray(c, float))


def _gate_point(alpha, side, w=0.008):
    """A fingertip beside the stop bracket's plate when the plate points at ``alpha`` (rad, world; pi/2 is shut):
    35 mm out along it, its centre ``w`` off the plate's centre line on the ``side`` (+1: the side facing +x when
    shut). The closed fingertips are 13 mm thick across the plate (yaw = alpha), so 8 mm presses about 1.5 mm into it
    and 15 mm stands clear."""
    u = 0.035
    d = np.array([math.cos(alpha), math.sin(alpha)])
    n = np.array([math.sin(alpha), -math.cos(alpha)])  # +x side when shut
    p = np.array(GATE_PIVOT) + u * d + side * w * n
    return float(p[0]), float(p[1])


def demo():
    lot = lots.current()
    start = np.asarray(lot.pos("cma"), float)
    yaw0 = float(lot.yaw_of("cma"))
    lifted = np.array([start[0], start[1], P_INST[2] + LIFT_Z])
    pre = P_INST + (-PRE, 0.0, LIFT_Z)
    home = P_INST + (OVER, 0.0, 0.001)
    relaxed = P_INST + (-0.001, 0.0, 0.001)  # the hands stop pushing, so the CMA is not held turned on its clip
    zg = ZT + 0.001  # fingertips 12 mm below the stop bracket's top
    yawg = {"left": -math.pi / 2, "right": math.pi / 2}  # fingers close along the basket's depth either way

    def grip(arm, pivot, yaw, sep, label, d, dz=0.0, dx=0.0):
        carrying = pivot is not start  # the CMA is in the hands (or just let go of, at the same height)
        p = np.array(_at(GRIP_C[arm], pivot, yaw)) + (dx, 0.0, dz + (SAG[arm] if carrying else 0.0))
        return Waypoint(d, tuple(float(v) for v in p), _sep(sep), label, yawg[arm] + yaw)

    # right hand: open the stop bracket, then take the inner basket by its hinge end
    def gate(d, alpha, side, z, label, w=0.008, turn=0.0):
        return Waypoint(d, (*_gate_point(alpha, side, w), z), FINGER_CLOSED, label, alpha + turn)

    shut = math.pi / 2
    right = [gate(2.0, shut, 1, ZT + HOVER, "above the stop bracket", 0.015, -math.pi),
             gate(1.5, shut, 1, zg, "fingertips behind its plate", 0.015, -math.pi)]
    for k, a in enumerate((105, 125, 150, 175), 1):
        right.append(gate(0.8, math.radians(a), 1, zg, f"turn it open {k}", turn=-math.pi))
    right.append(gate(1.0, math.radians(175), 1, ZT + HOVER, "up", 0.015, -math.pi))
    naive = os.environ.get("TASK_DEMO") == "naive"  # a check only: skip the stop bracket, push the CMA straight in
    if naive:
        right = [Waypoint(sum(w.duration for w in right), PARK["right"], FINGER_OPEN, "leave the stop bracket")]
    right += [grip("right", start, yaw0, GRIP_SEP_OPEN, "above the inner basket's hinge end", 2.0, HOVER),
              grip("right", start, yaw0, GRIP_SEP_OPEN, "fingers either side of it", 1.5)]
    t_right = sum(w.duration for w in right)
    left = [Waypoint(t_right - 3.5, PARK["left"], FINGER_OPEN, "wait", yawg["left"] + yaw0),
            grip("left", start, yaw0, GRIP_SEP_OPEN, "above the inner basket's clip end", 2.0, HOVER),
            grip("left", start, yaw0, GRIP_SEP_OPEN, "fingers either side of it", 1.5)]
    both = [
        ("close on the basket", 0.8, start, yaw0, GRIP_SEP_CLOSED),
        ("lift the CMA level", 1.5, lifted, yaw0, GRIP_SEP_CLOSED),
        ("carry it to the rack", 3.0, pre, 0.0, GRIP_SEP_CLOSED),
        ("settle", 0.4, pre, 0.0, GRIP_SEP_CLOSED),
        ("push the inner clip home", 2.5, home, 0.0, GRIP_SEP_CLOSED),
        ("hold", 0.5, home, 0.0, GRIP_SEP_CLOSED),
        ("ease off to where it sits", 0.6, relaxed, 0.0, GRIP_SEP_CLOSED),
    ]
    for arm, plan in (("left", left), ("right", right)):
        for label, d, piv, yaw, sep in both:
            plan.append(grip(arm, piv, yaw, sep, label, d))
    # left hand: let go, push the outer arm shut about the hinge until its clip snaps home
    left += [grip("left", relaxed, 0.0, GRIP_SEP_OPEN, "let go", 0.6),
             grip("left", relaxed, 0.0, GRIP_SEP_OPEN, "straight up", 0.8, 0.040),
             grip("left", relaxed, 0.0, GRIP_SEP_OPEN, "back", 0.6, 0.040, -0.040)]

    def push_pt(theta, extra=0.0):
        c = _rz(theta) @ np.array([-(PUSH_W + extra), PUSH_R, -0.012])
        return tuple(float(v) for v in P_INST + c)

    a0 = OPEN0 + math.radians(4.0)
    hi = push_pt(a0, 0.010)
    yp = lambda a: a - math.pi / 2  # noqa: E731  fingertips' thin side along the push
    left += [Waypoint(2.0, (hi[0], hi[1], ZT + HOVER), FINGER_CLOSED, "above the outer basket's front", yp(a0)),
             Waypoint(1.2, hi, FINGER_CLOSED, "fingertips in front of it", yp(a0))]
    for k, a in enumerate(np.linspace(a0, math.radians(-4.0), 5)[1:], 1):
        left.append(Waypoint(0.8, push_pt(a), FINGER_CLOSED, f"push the outer arm shut {k}", yp(a)))
    end = math.radians(-4.0)
    left.append(Waypoint(0.5, push_pt(end, 0.010), FINGER_CLOSED, "back off", yp(end)))
    left.append(Waypoint(1.0, push_pt(end, 0.010)[:2] + (ZT + HOVER,), FINGER_CLOSED, "up", yp(end)))
    # right hand lets go at the same time (a finger left between the baskets would stop the outer arm folding), waits,
    # then swings the stop bracket shut
    right += [grip("right", relaxed, 0.0, GRIP_SEP_OPEN, "let go", 0.6),
              grip("right", relaxed, 0.0, GRIP_SEP_OPEN, "straight up", 0.6, 0.018),
              grip("right", relaxed, 0.0, GRIP_SEP_OPEN, "back", 0.8, 0.035, -0.045)]
    t_left = sum(w.duration for w in left)
    t_right = sum(w.duration for w in right)
    right += [gate(max(2.0, t_left - t_right), math.pi, -1, ZT + HOVER, "above the open stop bracket", 0.015,
                   -math.pi),
              gate(1.5, math.pi, -1, zg, "fingertips beside its plate", 0.015, -math.pi)]
    for k, a in enumerate((165, 140, 110), 1):
        right.append(gate(0.8, math.radians(a), -1, zg, f"swing it shut {k}", turn=-math.pi))
    if naive:
        right = right[:-5]
    right += [gate(1.0, math.radians(110), -1, ZT + HOVER, "up", 0.015, -math.pi),
              Waypoint(2.0, PARK["right"], FINGER_OPEN, "park"),
              Waypoint(3.0, PARK["right"], FINGER_OPEN, "hold still")]
    t_right = sum(w.duration for w in right)
    left += [Waypoint(2.0, PARK["left"], FINGER_OPEN, "park", -math.pi / 2),
             Waypoint(max(0.5, t_right - sum(w.duration for w in left) - 2.0), PARK["left"], FINGER_OPEN, "wait",
                      -math.pi / 2)]
    return {"left": left, "right": right}
