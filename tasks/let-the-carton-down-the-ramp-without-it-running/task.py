"""On the station table: a model trailer bed on the robot's left, with a grey steel parcel slide running from its edge
down toward the robot's right onto a green dock plate. A brown carton stands on the trailer bed at the top of the
slide. Two white cartons stand on the table just past the far end of the dock.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402

import parts  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

GREY, DARK, GREEN = (0.62, 0.64, 0.66), (0.30, 0.30, 0.32), (0.20, 0.55, 0.25)
BROWN, WHITE = (0.60, 0.42, 0.24), (0.90, 0.90, 0.88)

SLOPE = math.radians(30.0)
RAMP_LEN, RAMP_W, RAMP_T = 0.32, 0.15, 0.01  # along the slope, across it (x), thickness
XC = 0.27  # centre line of the bed, slide and dock (x, in front of the arm bases)
Y_EDGE = 0.12  # the bed's edge, where the slide is hinged
DOCK_T = 0.012  # dock plate thickness
PIVOT_Z = DOCK_T + RAMP_LEN * math.sin(SLOPE) - RAMP_T / 2 * math.cos(SLOPE)  # hinge (board mid-plane) height
BED_TOP = PIVOT_Z + RAMP_T / 2 * math.cos(SLOPE)  # flush with the slide's top at its high edge
BED_LEN = 0.16  # bed, along y from the edge
Y_FOOT = Y_EDGE - RAMP_LEN * math.cos(SLOPE) - RAMP_T / 2 * math.sin(SLOPE)  # where the slide's top meets the dock
Y_DOCK_END = Y_FOOT - 0.15
DOCK_W = 0.20  # dock plate across (x)
FRICTION = (0.2, 0.2)  # plastic-lined slide, bed top and dock plate; the carton the same

RAMP = parts.Hinge(
    "slide",
    shapes=[parts.box((RAMP_W, RAMP_LEN, RAMP_T), at=(XC, Y_EDGE - RAMP_LEN / 2, PIVOT_Z), color=GREY)],
    pivot=(XC, Y_EDGE, PIVOT_Z),
    axis=(1.0, 0.0, 0.0),  # positive lowers the -y end; it rests on its upper stop
    lower=0.0,
    upper=SLOPE,
    mass=0.4,
    damping=0.05,
)
DOCK = parts.Part(
    "dock",
    pos=(0.0, 0.0, TOP),
    shapes=[
        parts.box((0.18, BED_LEN, BED_TOP), at=(XC, Y_EDGE + BED_LEN / 2, BED_TOP / 2), color=DARK),  # trailer bed
        # dock plate; its uphill end runs 1 cm under the slide's foot (a link does not collide with its own body)
        parts.box((DOCK_W, Y_FOOT + 0.01 - Y_DOCK_END, DOCK_T), at=(XC, (Y_FOOT + 0.01 + Y_DOCK_END) / 2, DOCK_T / 2),
                  color=GREEN),
    ],
    joints=[RAMP],
    friction=FRICTION,
)

CARTON_SIZE = (0.11, 0.12, 0.10)  # across x, along y (down the slide), tall
CARTON_MASS = 0.5
CARTON = parts.Part(
    "carton", pos=(XC, Y_EDGE + 0.005 + CARTON_SIZE[1] / 2, TOP + BED_TOP + CARTON_SIZE[2] / 2 + 0.0005),
    mass=CARTON_MASS, friction=FRICTION,
    shapes=[parts.box(CARTON_SIZE, at=(0.0, 0.0, 0.0), color=BROWN)],
)
W = 0.08  # witness cartons, cubes
Y_WITNESS = Y_DOCK_END - 0.005 - W / 2
WITNESSES = [
    Box("witness_a", pos=(XC - 0.045, Y_WITNESS, TOP + W / 2), size=(W, W, W), mass=0.2, color=WHITE),
    Box("witness_b", pos=(XC + 0.045, Y_WITNESS, TOP + W / 2), size=(W, W, W), mass=0.2, color=WHITE),
]
OBJECTS = [DOCK, CARTON, *WITNESSES]
LOOSE = ("carton", "witness_a", "witness_b")

SUCCESS_HOLD_S = 2.0
EPS = 1e-6
MAX_TILT = math.radians(10.0)
ON_DOCK = 0.001  # carton's lowest point within 1 mm of the dock top
TABLE_TOUCH = 0.002  # carton's lowest point within 2 mm of the table top: it fell off
WITNESS_LIMIT = 0.003  # a witness carton moved more than 3 mm
TELEPORT = 0.10
MAX_STEP = 0.020  # m: the carton's centre moved more than 20 mm in one 1/20 s control step (0.4 m/s): it ran

_history = {"spoiled": False, "worst": {"witness_a": 0.0, "witness_b": 0.0}, "peak": 0.0, "last": None}


def _quat_matrix(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _start_pos(name):
    return np.asarray(CARTON.pos if name == "carton" else next(w.pos for w in WITNESSES if w.name == name), float)


def _at_start(objects) -> bool:
    return all(np.linalg.norm(np.asarray(objects[n].pos) - _start_pos(n)) <= 0.0005 and objects[n].resting()
               for n in LOOSE)


def _update(objects) -> None:
    """Whole-trial record: the worst displacement of each witness carton from its start, the carton's largest move in one control step, and
    whether the carton ever touched the table. Cleared only by a scene reset: every loose object back within 0.5 mm of its start, at rest,
    after a jump of more than 20 mm in one control step (or at the first step)."""
    now = {n: np.asarray(objects[n].pos, float) for n in LOOSE}
    last = _history["last"]
    jump = last is None or max(np.linalg.norm(now[n] - last[n]) for n in LOOSE) > 0.02
    reset = jump and _at_start(objects)
    if reset:
        _history["spoiled"] = False
        _history["worst"] = {"witness_a": 0.0, "witness_b": 0.0}
        _history["peak"] = 0.0
    step = 0.0 if last is None else float(np.linalg.norm(now["carton"] - last["carton"]))
    if not reset and step <= TELEPORT:  # a move over 10 cm in 1/20 s is a scene edit, not motion
        _history["peak"] = max(_history["peak"], step)
    _history["last"] = now
    for n in ("witness_a", "witness_b"):
        _history["worst"][n] = max(_history["worst"][n], float(np.linalg.norm(now[n] - _start_pos(n))))
    if objects["carton"].low[2] <= TOP + TABLE_TOUCH + EPS:
        _history["spoiled"] = True


def _carton_on_dock(objects) -> bool:
    st = objects["carton"]
    rot = _quat_matrix(st.quat_xyzw)
    up = rot[:, 2]
    if up[2] < math.cos(MAX_TILT) - EPS:
        return False
    if abs(st.low[2] - (TOP + DOCK_T)) > ON_DOCK + EPS:
        return False
    hx, hy, hz = (s / 2 for s in CARTON_SIZE)
    centre = np.asarray(st.pos, float)
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = centre + rot @ np.array([sx * hx, sy * hy, -hz])  # bottom corners
            if not (Y_DOCK_END - EPS <= c[1] <= Y_FOOT + EPS and abs(c[0] - XC) <= DOCK_W / 2 + EPS):
                return False
    return st.resting()


def _grade(objects) -> float:
    _update(objects)
    if (_history["spoiled"] or max(_history["worst"].values()) > WITNESS_LIMIT + EPS
            or _history["peak"] > MAX_STEP + EPS):
        return 0.0
    return 1.0 if _carton_on_dock(objects) else 0.0


def success(objects):
    """The carton stands on the dock, still, never fell to the table, and neither white carton was moved."""
    return _grade(objects) == 1.0


def score(objects):
    """Pass/fail."""
    return _grade(objects)


# -- scripted demo: fixed waypoints ------------------------------------------------------------------------------

TIP = 0.013  # finger tips below the TCP
FACE = 0.012  # TCP centre to the closed fingers' face (along y), estimated
LOW = 0.025  # finger tips this high above the surface under them
RELEASE = None  # probes of the hazard: "top", "mid" or "foot" lets the carton go there


def _surface(y: float) -> float:
    """Height above the table of the bed, slide or dock top at y on the centre line."""
    if y >= Y_EDGE:
        return BED_TOP
    if y <= Y_FOOT:
        return DOCK_T
    return BED_TOP - (Y_EDGE - y) * math.tan(SLOPE)


def _at(y: float, low: float = LOW, x: float = XC):
    return (x, y, TOP + _surface(y) + low + TIP)


def demo():
    """Left arm: closed fingers push the carton along the bed until it tips over the edge onto the slide, then back
    off and lift away. Right arm: closed fingers wait low on the slide a few cm below where the carton lands, take
    its downhill face, walk it down the slide and across the dock at about 5 cm/s, then back off and lift. All fixed
    waypoints."""
    c = FINGER_CLOSED
    up_face = CARTON.pos[1] + CARTON_SIZE[1] / 2
    l0 = up_face + FACE + 0.003
    push = 0.08  # carton centre pushed about 1.5 cm past the edge
    r_wait = Y_EDGE - (CARTON_SIZE[1] + 0.025) * math.cos(SLOPE) - FACE  # 2.5 cm below where its face lands
    r_end = Y_FOOT + FACE + 0.003  # behind the carton: its uphill face pushed to the foot (it slides on about 1.5 cm)
    hi = TOP + 0.16
    q = math.pi / 2
    right = [
        Waypoint(1.5, (XC, 0.0, TOP + 0.14), c, "over the slide", q),
        Waypoint(1.5, _at(r_wait), c, "wait low on the slide", q),
        Waypoint(7.0, _at(r_wait), c, "wait for the carton", q),
        Waypoint(1.5, _at((r_wait + Y_FOOT + 0.02) / 2), c, "walk it down", q),
        Waypoint(1.5, _at(Y_FOOT + 0.02), c, "walk it down to the foot", q),
        Waypoint(1.5, _at(Y_FOOT - 0.03), c, "to the foot", q),
        Waypoint(1.0, _at(Y_FOOT - 0.03), c, "it stops where the slide meets the dock", q),
        Waypoint(1.0, (XC, Y_FOOT - 0.04, TOP + 0.10), c, "up", q),
        Waypoint(1.0, (XC - 0.12, Y_FOOT - 0.04, TOP + 0.10), c, "aside", q),
        Waypoint(1.5, (XC - 0.12, Y_FOOT + 0.118, _at(Y_FOOT + 0.118)[2] + 0.006), c, "past its side", q),
        Waypoint(1.0, _at(Y_FOOT + 0.118), c, "in behind it", q),
        Waypoint(3.0, _at(Y_FOOT + 0.033), c, "push it over the foot", q),
        Waypoint(1.5, (XC, r_end, TOP + 0.05), c, "onto the dock", q),
        Waypoint(1.0, (XC, r_end + 0.015, TOP + 0.05), c, "back off", q),
        Waypoint(1.0, (XC, r_end + 0.015, TOP + 0.16), c, "lift away", q),
    ]
    left = [
        Waypoint(1.5, (XC, l0 + 0.03, TOP + 0.215), c, "over the bed", -q),
        Waypoint(1.5, _at(l0 + 0.01, 0.03), c, "down behind the carton", -q),
        Waypoint(1.0, _at(l0, 0.03), c, "against its uphill face", -q),
        Waypoint(1.0, _at(l0, 0.03), c, "ready", -q),
        Waypoint(3.0, _at(l0 - push, 0.03), c, "push it over the edge", -q),
        Waypoint(0.5, _at(l0 - push + 0.04, 0.03), c, "back off", -q),
        Waypoint(1.0, (XC, l0 - push + 0.07, TOP + 0.215), c, "away", -q),
    ]
    if RELEASE:  # a probe: the right arm lifts away at once, leaving the carton free
        keep = {"top": 1, "mid": 4, "foot": 5}[RELEASE]
        last = right[keep - 1].pos
        right = right[:keep] + [Waypoint(0.4, (last[0], last[1], last[2] + 0.08), c, "let go", q),
                                Waypoint(20.0, (0.12, -0.25, TOP + 0.15), c, "out of the way", q)]
    return {"left": left, "right": right}
