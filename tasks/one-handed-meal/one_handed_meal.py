"""On the station table: a tan board figure of a person seated across the table, facing the robot, so her right is on
the robot's left. A grey non-slip mat lies in front of her and a thin grey pad lies on the midline. A white bowl
and a blue cup stand on the robot's right, and a flat white dish stands on the robot's left. All three are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=one_handed_meal.py --scorer task_score \\
           --instruction "Ana can only use her right hand. Set out her cup and whichever of the two dishes is easier for her to eat from." ...
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TAN = (0.80, 0.66, 0.52)
WHITE = (0.95, 0.95, 0.92)
BOARD = 0.01  # the figure's thickness (m)
MAT_H = 0.002
PAD_H = 0.002
MAT_CENTRE = (0.27, 0.02)
MAT_SIZE = (0.14, 0.24)
RELAY_XY = (0.18, 0.0)


def _board(name, x, y, size_x, size_y):
    return Box(name, pos=(x, y, TOP + BOARD / 2), size=(size_x, size_y, BOARD), color=TAN)


FIGURE = [
    _board("shoulders", 0.46, 0.0, 0.06, 0.26),
    _board("right_forearm", 0.41, 0.12, 0.05, 0.04),
    _board("left_forearm", 0.41, -0.12, 0.05, 0.04),
    Cylinder("head", pos=(0.46, 0.0, TOP + BOARD + 0.025), radius=0.05, height=0.05, color=TAN),
]
MAT = Box("mat", pos=(*MAT_CENTRE, TOP + MAT_H / 2), size=(*MAT_SIZE, MAT_H), color=(0.30, 0.30, 0.33))
PAD = Box("pad", pos=(*RELAY_XY, TOP + PAD_H / 2), size=(0.08, 0.08, PAD_H), color=(0.55, 0.55, 0.58))

BOWL_W = 0.085  # widest horizontal extent (m)
BOWL_XY = (0.19, -0.13)
CUP_R, CUP_H = 0.025, 0.08
BOWL = Asset("bowl", asset="bowl_ycb_robolab", xy=BOWL_XY, extent=BOWL_W, mass=0.1, color=WHITE, convex=True)
CUP = Cylinder("cup", pos=(0.19, -0.23, TOP + CUP_H / 2), radius=CUP_R, height=CUP_H, mass=0.05,
               color=(0.15, 0.30, 0.85))
DISH = Cylinder("dish", pos=(0.15, 0.19, TOP + 0.005), radius=0.06, height=0.01, mass=0.1, color=WHITE)
OBJECTS = [*FIGURE, MAT, PAD, BOWL, CUP, DISH]

BOWL_TARGET = (0.27, -0.03)
CUP_TARGET = (0.27, 0.09)
MAX_TILT_DEG = 10.0
MIN_CUP_OFFSET = 0.06  # the cup's centre must be this far to her right of the bowl's (m): side by side, not in front


_UP_LOCAL = {}  # each item's local axis that pointed up at the start (a library model's own up axis may not be +z)


def _rot(quat_xyzw):
    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _note_start(objects):
    """Record how the bowl and cup stand on the first step of a trial, while they are still as placed."""
    for name in ("bowl", "cup"):
        if name not in _UP_LOCAL:
            _UP_LOCAL[name] = _rot(objects[name].quat_xyzw).T @ np.array([0.0, 0.0, 1.0])


def _tilt_deg(name, state):
    """Angle between the item's starting up axis, as it points now, and world vertical (degrees)."""
    up_z = float((_rot(state.quat_xyzw) @ _UP_LOCAL[name])[2])
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


def _over_mat(item):
    return bool(
        abs(item.pos[0] - MAT_CENTRE[0]) < MAT_SIZE[0] / 2 and abs(item.pos[1] - MAT_CENTRE[1]) < MAT_SIZE[1] / 2
    )


def _on_mat(objects, name):
    """Standing on the mat, the right way up and still, with its centre over it."""
    item = objects[name]
    resting_on_mat = abs(item.low[2] - (TOP + MAT_H)) < 0.005
    return bool(_over_mat(item) and resting_on_mat and _tilt_deg(name, item) < MAX_TILT_DEG and item.resting())


def _cup_on_right(objects):
    """Seen from her side: the cup is on her right of the bowl, which is the robot's left."""
    return bool(objects["cup"].pos[1] - objects["bowl"].pos[1] >= MIN_CUP_OFFSET)


def success(objects):
    _note_start(objects)
    return bool(
        _on_mat(objects, "bowl")
        and _on_mat(objects, "cup")
        and _cup_on_right(objects)
        and not _over_mat(objects["dish"])
    )


def distance(objects):
    bowl = objects["bowl"].pos[:2] - np.array(BOWL_TARGET)
    cup = objects["cup"].pos[:2] - np.array(CUP_TARGET)
    return float(max(np.linalg.norm(bowl), np.linalg.norm(cup)))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 for the bowl on the mat, 0.25 for the cup on the mat, 0.25 more if both
    are on it with the cup on her right; 0 if the flat dish is over the mat."""
    _note_start(objects)
    if _over_mat(objects["dish"]):
        return 0.0
    bowl, cup = _on_mat(objects, "bowl"), _on_mat(objects, "cup")
    return 0.5 * bowl + 0.25 * cup + 0.25 * (bowl and cup and _cup_on_right(objects))


TRAVEL_Z = TOP + 0.14
BOWL_GRIP_H = 0.02  # the lowest grip height proven on this table; keep below the bowl's height
CUP_GRIP_H = CUP_H / 2
CYCLE_S = 12.0  # each move takes this long, including moving clear
LEFT_START_S = 23.1  # the left arm starts once the right arm has set the cup on the pad and moved clear
RIGHT_PARK = (0.20, -0.30, TRAVEL_Z)
LEFT_PARK = (0.20, 0.30, TRAVEL_Z)


def _cycle(pick, place, park):
    """One pick and place, then a move clear of the table's centre, padded to CYCLE_S so the arms stay in step."""
    steps = pick_place_waypoints(pick, place, travel_z=TRAVEL_Z)
    steps.append(Waypoint(1.5, park, FINGER_OPEN, "clear"))
    steps.append(Waypoint(CYCLE_S - sum(w.duration for w in steps), park, FINGER_OPEN, "wait"))
    return steps


def demo():
    """The right arm sets the bowl on the mat and the cup on the pad; the left arm carries the cup on to the mat."""
    relay_z = TOP + PAD_H + CUP_GRIP_H + 0.005
    right = _cycle((*BOWL_XY, TOP + BOWL_GRIP_H), (*BOWL_TARGET, TOP + MAT_H + BOWL_GRIP_H + 0.005), RIGHT_PARK)
    right += _cycle((*CUP.pos[:2], TOP + CUP_GRIP_H), (*RELAY_XY, relay_z), RIGHT_PARK)
    left = [Waypoint(LEFT_START_S, LEFT_PARK, FINGER_OPEN, "wait")]
    left += _cycle((*RELAY_XY, TOP + PAD_H + CUP_GRIP_H), (*CUP_TARGET, TOP + MAT_H + CUP_GRIP_H + 0.005), LEFT_PARK)
    return {"right": right, "left": left}
