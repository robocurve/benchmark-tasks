"""On the station table, a kitchen prep counter: three open lunch boxes stand in a row, rice in the middle one and pasta
in the two outer ones. Three identical lids lie in front of them, and a green serving tray stands in front of the
right arm. With its lid on, a box no longer shows what is inside.
"""

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from partkit import on_reset, part  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

PLASTIC, LID_C, RICE, PASTA = (0.80, 0.86, 0.90), (0.35, 0.45, 0.55), (0.96, 0.94, 0.86), (0.88, 0.45, 0.18)
TRAY_C, STEEL, BOARD = (0.20, 0.62, 0.32), (0.70, 0.72, 0.74), (0.72, 0.55, 0.36)

BOX_L, BOX_W, BOX_H, WALL, FOOD_H = 0.09, 0.066, 0.05, 0.004, 0.02
BOX_X = 0.36
BOXES = {"box_left": (0.20, PASTA), "box_middle": (0.0, RICE), "box_right": (-0.20, PASTA)}
GLUTEN_FREE = "box_middle"


def _box(name, y, food):
    inner_l, inner_w = BOX_L - 2 * WALL, BOX_W - 2 * WALL
    return part(name, pos=(BOX_X, y, TOP), mass=0.25, color=PLASTIC, friction=1.0, boxes=[
        ((0, 0, WALL / 2), (BOX_L, BOX_W, WALL), PLASTIC),
        ((0, (BOX_W - WALL) / 2, BOX_H / 2), (BOX_L, WALL, BOX_H), PLASTIC),
        ((0, -(BOX_W - WALL) / 2, BOX_H / 2), (BOX_L, WALL, BOX_H), PLASTIC),
        (((BOX_L - WALL) / 2, 0, BOX_H / 2), (WALL, inner_w, BOX_H), PLASTIC),
        ((-(BOX_L - WALL) / 2, 0, BOX_H / 2), (WALL, inner_w, BOX_H), PLASTIC),
        ((0, 0, WALL + FOOD_H / 2), (inner_l, inner_w, FOOD_H), food),
    ])


PLATE_T, SKIRT_H, SKIRT_GAP, KNOB = 0.004, 0.01, 0.004, (0.03, 0.024, 0.025)
LID_STARTS = {"lid_a": (0.19, 0.26), "lid_b": (0.19, 0.09), "lid_c": (0.19, -0.11)}


def _lid(name, xy):
    skirt_l, skirt_w = BOX_L - 2 * WALL - 2 * SKIRT_GAP, BOX_W - 2 * WALL - 2 * SKIRT_GAP
    return part(name, pos=(*xy, TOP + SKIRT_H), mass=0.05, color=LID_C, friction=1.0, boxes=[
        ((0, 0, PLATE_T / 2), (BOX_L, BOX_W, PLATE_T), LID_C),
        ((0, (skirt_w - WALL) / 2, -SKIRT_H / 2), (skirt_l, WALL, SKIRT_H), LID_C),
        ((0, -(skirt_w - WALL) / 2, -SKIRT_H / 2), (skirt_l, WALL, SKIRT_H), LID_C),
        (((skirt_l - WALL) / 2, 0, -SKIRT_H / 2), (WALL, skirt_w - 2 * WALL, SKIRT_H), LID_C),
        ((-(skirt_l - WALL) / 2, 0, -SKIRT_H / 2), (WALL, skirt_w - 2 * WALL, SKIRT_H), LID_C),
        ((0, 0, PLATE_T + KNOB[2] / 2), KNOB, LID_C),
    ])


TRAY_XY, TRAY_INSIDE, TRAY_FLOOR = (0.20, -0.29), (0.12, 0.10), TOP + 0.004
TRAY = [
    Box("tray_floor", pos=(*TRAY_XY, TOP + 0.002), size=(0.13, 0.11, 0.004), color=TRAY_C),
    *[Box(f"tray_rim_{i}", pos=(TRAY_XY[0] + dx, TRAY_XY[1] + dy, TOP + 0.007), size=size, color=TRAY_C)
      for i, (dx, dy, size) in enumerate([(0.0625, 0, (0.005, 0.11, 0.01)), (-0.0625, 0, (0.005, 0.11, 0.01)),
                                          (0, 0.0525, (0.13, 0.005, 0.01)), (0, -0.0525, (0.13, 0.005, 0.01))])],
]

DRESSING = [
    Box("prep_sheet", pos=(0.33, 0.0, TOP + 0.0005), size=(0.26, 0.62, 0.001), color=STEEL),
    Box("cutting_board", pos=(0.60, 0.22, TOP + 0.01), size=(0.18, 0.12, 0.02), color=BOARD),
    Cylinder("water_jug", pos=(0.62, -0.05, TOP + 0.08), radius=0.045, height=0.16, color=(0.80, 0.90, 0.95)),
    Asset("plates", "plate_small_vomp_robolab", xy=(0.60, -0.30), extent=0.18),
    Asset("fruit_bowl", "wooden_bowl_hot3d_robolab", xy=(0.62, 0.40), extent=0.16),
]

OBJECTS = [*TRAY, *DRESSING, *[_box(n, y, food) for n, (y, food) in BOXES.items()],
           *[_lid(n, xy) for n, xy in LID_STARTS.items()]]

_S = {"early": False, "covered": False}


@on_reset
def _reset(environment=None):
    _S.update(early=False, covered=False)


def _tilt_deg(state):
    x, y, _, _ = state.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _lidded(objects, box):
    b = objects[box]
    return any(np.linalg.norm(objects[lid].pos[:2] - b.pos[:2]) < 0.01
               and abs(objects[lid].pos[2] - (b.pos[2] + BOX_H)) < 0.005 and _tilt_deg(objects[lid]) < 10
               for lid in LID_STARTS)


def _moved(objects, box):
    b = objects[box]
    return np.linalg.norm(b.pos[:2] - (BOX_X, BOXES[box][0])) > 0.02 or b.pos[2] > TOP + 0.02


def _on_tray(objects, box):
    b = objects[box]
    dx, dy = np.abs(b.pos[:2] - TRAY_XY)
    return bool(dx < TRAY_INSIDE[0] / 2 and dy < TRAY_INSIDE[1] / 2 and b.pos[2] < TRAY_FLOOR + 0.01
                and _tilt_deg(b) < 10 and b.resting())


def score(objects):
    """0 if a box moved before all three had been lidded, or a pasta box ends on the tray. Otherwise 1.0 for the rice box
    on the tray with all three boxes lidded, 0.5 for the rice box on the tray with a lid off, 0.25 for all three
    lidded and nothing served."""
    lidded = all(_lidded(objects, b) for b in BOXES)
    _S["covered"] = _S["covered"] or lidded
    if not _S["covered"] and any(_moved(objects, b) for b in BOXES):
        _S["early"] = True
    if _S["early"] or any(_on_tray(objects, b) for b in BOXES if b != GLUTEN_FREE):
        return 0.0
    if _on_tray(objects, GLUTEN_FREE):
        return 1.0 if lidded else 0.5
    return 0.25 if lidded else 0.0


def _move(start, end, grip, place, travel, settle=True):
    steps = [
        (1.5, (*start, travel), FINGER_OPEN, "above"),
        (1.5, (*start, grip), FINGER_OPEN, "descend"),
        (0.6, (*start, grip), FINGER_OPEN, "settle"),
        (0.8, (*start, grip), FINGER_CLOSED, "close"),
        (1.0, (*start, travel), FINGER_CLOSED, "lift"),
        (2.0, (*end, travel), FINGER_CLOSED, "carry"),
        (1.5, (*end, place + 0.01), FINGER_CLOSED, "lower"),
        (0.6, (*end, place), FINGER_CLOSED, "set down"),
        (0.6, (*end, place), FINGER_OPEN, "release"),
        (1.0, (*end, travel), FINGER_OPEN, "retreat"),
    ]
    return [Waypoint(*s) for s in steps]


def demo():
    """The left arm lids the left and middle boxes and the right arm the right one, each lid lifted by its knob; then
    the right arm carries the middle box, which it saw held the rice, to the tray."""
    travel = TOP + 0.17
    knob = SKIRT_H + PLATE_T + KNOB[2] / 2 + 0.008
    lid_grip, lid_place = TOP + knob, TOP + BOX_H + knob + 0.002
    box_grip, box_place = TOP + 0.028, TRAY_FLOOR + 0.028 + 0.004
    left = [*_move(LID_STARTS["lid_a"], (BOX_X, 0.20), lid_grip, lid_place, travel),
            *_move(LID_STARTS["lid_b"], (BOX_X, 0.0), lid_grip, lid_place, travel),
            Waypoint(1.5, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "move clear")]
    right = [*_move(LID_STARTS["lid_c"], (BOX_X, -0.20), lid_grip, lid_place, travel),
             Waypoint(13.0, (0.20, -0.30, travel), FINGER_OPEN, "wait for the left arm"),
             *_move((BOX_X, 0.0), TRAY_XY, box_grip, box_place, travel),
             Waypoint(1.5, (0.12, -0.42, TOP + 0.20), FINGER_OPEN, "move clear")]
    return {"left": left, "right": right}
