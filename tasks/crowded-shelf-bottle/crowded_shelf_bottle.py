"""On the station table: a fixed wooden shelf board (12 x 30 x 2 cm) with five glass bottles (4 cm wide, 12 cm tall)
standing in a row 1.2 cm apart. The middle one is red, the others amber. A plastic tray is fixed in front of the
left arm. A bottle that leans 6 degrees touches its neighbour; it topples at 18.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SHELF_TOP = TOP + 0.02
ROW_X = 0.30
RADIUS, HEIGHT = 0.02, 0.12
AMBER, RED = (0.60, 0.35, 0.10), (0.80, 0.12, 0.12)

SHELF = Box("shelf", pos=(0.31, 0.10, TOP + 0.01), size=(0.12, 0.30, 0.02), color=(0.55, 0.40, 0.25))


def _bottle(name, y, color):
    z = SHELF_TOP + HEIGHT / 2 + 0.001
    return Cylinder(name, pos=(ROW_X, y, z), radius=RADIUS, height=HEIGHT, mass=0.15, color=color)


BOTTLES = [
    _bottle("amber_1", -0.004, AMBER),
    _bottle("amber_2", 0.048, AMBER),
    _bottle("red_bottle", 0.100, RED),
    _bottle("amber_3", 0.152, AMBER),
    _bottle("amber_4", 0.204, AMBER),
]
RED_BOTTLE = BOTTLES[2]
OTHERS = [b.name for b in BOTTLES if b is not RED_BOTTLE]

TRAY_X, TRAY_Y = 0.15, 0.25
TRAY_INSIDE = (0.10, 0.10)
TRAY_FLOOR = TOP + 0.004


def _tray(wall=0.006, height=0.02, color=(0.20, 0.30, 0.60)):
    lx, ly = TRAY_INSIDE
    wz = TOP + height / 2
    return [
        Box("tray_floor", pos=(TRAY_X, TRAY_Y, TOP + 0.002), size=(lx + 2 * wall, ly + 2 * wall, 0.004), color=color),
        Box("tray_front", pos=(TRAY_X - (lx + wall) / 2, TRAY_Y, wz), size=(wall, ly + 2 * wall, height), color=color),
        Box("tray_back", pos=(TRAY_X + (lx + wall) / 2, TRAY_Y, wz), size=(wall, ly + 2 * wall, height), color=color),
        Box("tray_left", pos=(TRAY_X, TRAY_Y + (ly + wall) / 2, wz), size=(lx, wall, height), color=color),
        Box("tray_right", pos=(TRAY_X, TRAY_Y - (ly + wall) / 2, wz), size=(lx, wall, height), color=color),
    ]


OBJECTS = [SHELF, *BOTTLES, *_tray()]

START_XY = {b.name: np.array(b.pos[:2]) for b in BOTTLES}
_fell = False
_disturbed = False
_last = {}


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _moved(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - START_XY[name]))


def _in_tray(obj):
    dx, dy = np.abs(obj.pos[:2] - (TRAY_X, TRAY_Y))
    inside = dx < TRAY_INSIDE[0] / 2 and dy < TRAY_INSIDE[1] / 2
    return bool(inside and obj.low[2] < TRAY_FLOOR + 0.01 and obj.resting())


def _new_trial(objects):
    """The task module lives across trials, so a reset is recognised by every bottle jumping back to its mark
    within one control step. A bottle stood back up by the arm never jumps, so its fall still counts."""
    pose = {n: (objects[n].pos.copy(), _tilt_deg(objects[n])) for n in START_XY}
    jumped = any(
        np.linalg.norm(pose[n][0] - _last[n][0]) > 0.03 or abs(pose[n][1] - _last[n][1]) > 30 for n in _last
    )
    _last.update(pose)
    at_marks = all(
        _moved(objects, n) < 0.005 and _tilt_deg(objects[n]) < 2 and objects[n].resting() for n in START_XY
    )
    return jumped and at_marks


def _track(objects):
    """Remember, for the whole trial, whether any bottle fell or any amber bottle was tilted."""
    global _fell, _disturbed
    if _new_trial(objects):
        _fell = _disturbed = False
    _fell = _fell or any(_tilt_deg(objects[n]) > 45 for n in START_XY)
    _disturbed = _disturbed or any(_tilt_deg(objects[n]) > 10 for n in OTHERS)


def score(objects):
    """0 if any bottle ever tilted past 45 deg. Red bottle upright in the tray: 1.0, or 0.5 if an amber bottle
    ever tilted past 10 deg or ends more than 1 cm from its start. Red bottle upright, 5 cm or more from its
    start but not in the tray: 0.25."""
    _track(objects)
    red = objects[RED_BOTTLE.name]
    if _fell or _tilt_deg(red) > 15:
        return 0.0
    if _in_tray(red):
        pushed = any(_moved(objects, n) > 0.01 for n in OTHERS)
        return 0.5 if _disturbed or pushed else 1.0
    return 0.25 if _moved(objects, RED_BOTTLE.name) >= 0.05 and red.resting() else 0.0


def success(objects):
    return score(objects) == 1.0


def demo():
    """Left arm grips the red bottle near its top, fingers closing front to back, slides it forward out of the
    row, then sets it down in the tray."""
    x, y = RED_BOTTLE.pos[:2]
    grip = SHELF_TOP + HEIGHT - 0.025
    drop = TRAY_FLOOR + 0.005 + HEIGHT - 0.025
    travel = TOP + 0.20
    yaw = -np.pi / 2
    steps = [
        (1.5, (x, y, travel), FINGER_OPEN, "above the red bottle"),
        (1.5, (x, y, grip), FINGER_OPEN, "descend"),
        (0.8, (x, y, grip), FINGER_CLOSED, "close"),
        (0.8, (x, y, grip + 0.02), FINGER_CLOSED, "lift"),
        (1.5, (0.18, y, grip + 0.02), FINGER_CLOSED, "slide out of the row"),
        (1.0, (0.18, y, travel), FINGER_CLOSED, "raise"),
        (1.5, (TRAY_X, TRAY_Y, travel), FINGER_CLOSED, "carry"),
        (1.2, (TRAY_X, TRAY_Y, drop), FINGER_CLOSED, "lower"),
        (0.6, (TRAY_X, TRAY_Y, drop), FINGER_OPEN, "release"),
        (1.0, (TRAY_X, TRAY_Y, travel), FINGER_OPEN, "retreat"),
    ]
    return {"left": [Waypoint(d, p, f, label, yaw) for d, p, f, label in steps]}
