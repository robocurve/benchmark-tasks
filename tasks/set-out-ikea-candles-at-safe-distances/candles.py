"""IKEA candles: set out three candles on a stone board following IKEA's candle-safety distances.

On the table:
- a grey stone heat-proof board (24 x 44 cm) in the middle
- a white block candle (5 cm across, 7.5 cm tall) at the left, two tealights in clear glass holders
  (4.5 cm across, 4 cm tall) at the right
- a vase of dried flowers at the back left corner of the table
- a wooden wall shelf at the back right, 30 cm above the table, overhanging the back right of the board

IKEA's candle-safety guide: minimum distance between burning candles 7 cm for block candles and 3 cm for tealights;
candles at least 30 cm from anything that can catch fire (dried flowers); at least 50 cm of open space above a lit
candle (so not under a shelf).
"""

import itertools
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import mech  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CONTROL_HZ = 20.0
DEBUG = False

# The rules (IKEA candle-safety guide)
GAP = {"block": 0.07, "tealight": 0.03}  # minimum gap between candle edges; for a mixed pair, the larger applies
FLAMMABLE_M = 0.30  # from anything that can catch fire
UPRIGHT_DEG = 10.0

# Stone board (where the candles go), vase of dried flowers, shelf
TRAY_X, TRAY_Y, TRAY_H = (0.12, 0.36), (-0.22, 0.22), 0.01  # the board's extent and thickness
TRAY_C = ((TRAY_X[0] + TRAY_X[1]) / 2, (TRAY_Y[0] + TRAY_Y[1]) / 2)
VASE_C, VASE_R, VASE_H = (0.50, 0.44), 0.04, 0.12
SHELF_X, SHELF_Y, SHELF_Z = (0.30, 0.45), (-0.32, -0.07), TOP + 0.30  # the shelf's footprint and underside height
# Candles: (name, kind, radius, height, start xy)
CANDLES = [
    ("block", "block", 0.025, 0.075, (0.22, 0.28)),
    ("tealight_1", "tealight", 0.0225, 0.04, (0.22, -0.28)),  # picked first
    ("tealight_2", "tealight", 0.0225, 0.04, (0.145, -0.27)),  # then this one, nearer the robot
]
KIND = {c[0]: c[1] for c in CANDLES}
RAD = {c[0]: c[2] for c in CANDLES}

WHITE, GLASS, METAL = (0.96, 0.95, 0.9), (0.75, 0.85, 0.9), (0.42, 0.43, 0.45)
WOOD, VASE, DRIED = (0.7, 0.52, 0.32), (0.35, 0.3, 0.45), (0.78, 0.62, 0.35)

OBJECTS = [
    Box("tray", size=(TRAY_X[1] - TRAY_X[0], TRAY_Y[1] - TRAY_Y[0], TRAY_H), pos=(*TRAY_C, TOP + TRAY_H / 2), color=METAL),
    Cylinder("vase", radius=VASE_R, height=VASE_H, pos=(*VASE_C, TOP + VASE_H / 2), color=VASE),
    *[Sphere(f"flower_{i}", radius=0.022, pos=(VASE_C[0] + dx, VASE_C[1] + dy, TOP + VASE_H + 0.03 + dz), color=DRIED)
      for i, (dx, dy, dz) in enumerate([(0.0, 0.0, 0.02), (0.025, 0.02, 0.0), (-0.02, 0.025, -0.005), (0.01, -0.025, 0.0)])],
    Box("shelf", size=(SHELF_X[1] - SHELF_X[0], SHELF_Y[1] - SHELF_Y[0], 0.018),
        pos=((SHELF_X[0] + SHELF_X[1]) / 2, (SHELF_Y[0] + SHELF_Y[1]) / 2, SHELF_Z + 0.009), color=WOOD),
    Box("shelf_post_l", size=(0.02, 0.02, SHELF_Z - TOP), pos=(SHELF_X[1] - 0.01, SHELF_Y[1] - 0.01, (TOP + SHELF_Z) / 2), color=WOOD),
    Box("shelf_post_r", size=(0.02, 0.02, SHELF_Z - TOP), pos=(SHELF_X[1] - 0.01, SHELF_Y[0] + 0.01, (TOP + SHELF_Z) / 2), color=WOOD),
]
for name, kind, r, h, (x, y) in CANDLES:
    if kind == "block":
        OBJECTS.append(Cylinder(name, radius=r, height=h, pos=(x, y, TOP + h / 2), color=WHITE, mass=0.15))
    else:  # a tealight in a clear glass holder: the holder is the body, the tealight's white top inside it
        OBJECTS.append(Cylinder(name, radius=r, height=h, pos=(x, y, TOP + h / 2), color=GLASS, mass=0.12))
        OBJECTS.append(Cylinder(f"{name}_wax", radius=r - 0.006, height=0.004, pos=(x, y, TOP + h + 0.002), color=WHITE, mass=0.002))
        mech.weld(f"{name}_wax", parent=name)


# -- the rules, as measurements --------------------------------------------------------------------


def tilt_deg(q) -> float:
    x, y, _, _ = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def on_tray(name, o) -> bool:
    """Standing upright with its whole base on the stone board."""
    r = RAD[name]
    inside = TRAY_X[0] - 0.002 <= o.pos[0] - r and o.pos[0] + r <= TRAY_X[1] + 0.002 and \
        TRAY_Y[0] - 0.002 <= o.pos[1] - r and o.pos[1] + r <= TRAY_Y[1] + 0.002
    return inside and abs(o.low[2] - (TOP + TRAY_H)) < 0.006 and tilt_deg(o.quat_xyzw) < UPRIGHT_DEG


def gap(a, oa, b, ob) -> float:
    return math.hypot(oa.pos[0] - ob.pos[0], oa.pos[1] - ob.pos[1]) - RAD[a] - RAD[b]


def needed_gap(a, b) -> float:
    return max(GAP[KIND[a]], GAP[KIND[b]])


def flammable_gap(name, o) -> float:
    return math.hypot(o.pos[0] - VASE_C[0], o.pos[1] - VASE_C[1]) - RAD[name] - VASE_R


def under_shelf(name, o) -> bool:
    r = RAD[name]
    return SHELF_X[0] - r < o.pos[0] < SHELF_X[1] + r and SHELF_Y[0] - r < o.pos[1] < SHELF_Y[1] + r


def broken(placed, objects):
    """The rules broken by the candles standing on the tray."""
    why = []
    for n in placed:
        if flammable_gap(n, objects[n]) < FLAMMABLE_M:
            why.append(f"{n} within 30 cm of the dried flowers")
        if under_shelf(n, objects[n]):
            why.append(f"{n} under the shelf")
    for a, b in itertools.combinations(placed, 2):
        if gap(a, objects[a], b, objects[b]) < needed_gap(a, b):
            why.append(f"{a} and {b} too close")
    return why


def score(objects):
    placed = [n for n in KIND if on_tray(n, objects[n])]
    why = broken(placed, objects)
    if DEBUG:
        _state["n"] = _state.get("n", 0) + 1
        if _state["n"] % 20 == 0:
            pos = {n: (round(float(objects[n].pos[0]), 3), round(float(objects[n].pos[1]), 3)) for n in KIND}
            tcp = {a: tuple(round(float(v), 3) for v in objects[f"{a}_tcp"].pos[:2]) for a in ("left", "right")}
            print(f"DBG t={_state['n'] / CONTROL_HZ:.0f} placed={placed} broken={why} pos={pos} tcp={tcp}", flush=True)
    if why:
        return 0.0
    return len(placed) / len(KIND)


_state = {}


@mech.on_tick
def _new_episode(sim):
    if sim.t <= 1.5 / CONTROL_HZ:
        _state.clear()


# -- scripted demo -------------------------------------------------------------------------------

# A safe layout (found by a grid search, checked by the tests: every rule met with at least 2 cm to spare): the block
# candle on the board's left half (left arm), the tealights on its right half (right arm), each within easy reach
# and with the arms' wrists clear of the shelf.
TARGETS = {
    "block": (0.18, 0.165),
    "tealight_1": (0.18, -0.065),
    "tealight_2": (0.19, -0.165),
}


def _pick_place(name, travel, yaw=0.0):
    kind, r, h = KIND[name], RAD[name], next(c[3] for c in CANDLES if c[0] == name)
    sx, sy = next(c[4] for c in CANDLES if c[0] == name)
    tx, ty = TARGETS[name]
    grip = TOP + h - 0.015  # fingers around the candle, 1.5 cm below its top
    place = TOP + TRAY_H + h - 0.015 + 0.012  # let go with the candle 1 cm above the board: no pressing into it
    return [
        Waypoint(1.5, (sx, sy, travel), FINGER_OPEN, f"over {name}", yaw),
        Waypoint(1.2, (sx, sy, grip), FINGER_OPEN, "down around it", yaw),
        Waypoint(0.6, (sx, sy, grip), FINGER_CLOSED, "grip", yaw),
        Waypoint(1.2, (sx, sy, travel), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (tx, ty, travel), FINGER_CLOSED, "over its spot on the board", yaw),
        Waypoint(1.5, (tx, ty, place), FINGER_CLOSED, "lower it just above the board", yaw),
        Waypoint(0.5, (tx, ty, place), FINGER_CLOSED, "steady", yaw),
        Waypoint(0.6, (tx, ty, place), FINGER_OPEN, "let go: it stands on the board", yaw),
        Waypoint(1.0, (tx, ty, travel), FINGER_OPEN, "up", yaw),
    ]


def demo():
    travel = TOP + 0.15
    left = [*_pick_place("block", travel), Waypoint(2.0, (0.15, 0.27, travel + 0.05), FINGER_OPEN, "park")]
    right = [Waypoint(12.0, (0.15, -0.27, travel + 0.05), FINGER_OPEN, "wait for the left arm"),
             *_pick_place("tealight_1", travel), *_pick_place("tealight_2", travel),
             Waypoint(2.0, (0.15, -0.27, travel + 0.05), FINGER_OPEN, "park")]
    return {"left": left, "right": right}
