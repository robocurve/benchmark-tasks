"""On the station table: a keyboard lying left to right with a mug of coffee touching its left end, and a
smartphone in front of the keyboard with a takeaway coffee cup touching its right side.
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CLEARANCE = 0.15

KEYBOARD = Asset("keyboard", "keyboard_hot3d_robolab", xy=(0.33, 0.0), extent=0.30, mass=0.50, yaw=np.pi / 2)
PHONE = Asset("phone", "smartphone_hot3d_robolab", xy=(0.21, -0.10), extent=0.14, mass=0.17)
MUG = Asset("mug", "mug_ycb_robolab", xy=(0.33, 0.20), extent=0.085, mass=0.30)
CUP = Asset("coffee_cup", "gregorys_coffee_cup_objaverse_robolab", xy=(0.21, -0.215), extent=0.065, mass=0.25)
OBJECTS = [KEYBOARD, PHONE, MUG, CUP]

DRINKS, DEVICES = ("mug", "coffee_cup"), ("keyboard", "phone")
START_XY = {o.name: np.array(o.xy) for o in OBJECTS}
_spilled = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _moved(objects, name):
    return float(np.linalg.norm(objects[name].pos[:2] - START_XY[name]))


def _gap(a, b):
    """Horizontal distance between two objects' outlines (world-aligned bounding boxes)."""
    dx = max(a.low[0] - b.high[0], b.low[0] - a.high[0], 0.0)
    dy = max(a.low[1] - b.high[1], b.low[1] - a.high[1], 0.0)
    return float(np.hypot(dx, dy))


def _track(objects):
    """Remember, for the whole trial, whether a drink tipped far enough to spill."""
    global _spilled
    if all(_moved(objects, n) < 0.005 for n in START_XY) and all(_tilt_deg(objects[n]) < 2 for n in DRINKS):
        _spilled = False
    _spilled = _spilled or any(_tilt_deg(objects[n]) > 30 for n in DRINKS)


def score(objects):
    """0 if a drink ever tilted past 30 deg or a device ends more than 2 cm from its start. Otherwise 0.5 for
    each drink upright and at least 15 cm from both devices."""
    _track(objects)
    if _spilled or any(_moved(objects, n) > 0.02 for n in DEVICES):
        return 0.0
    safe = [
        _tilt_deg(d) < 15 and d.resting() and min(_gap(d, objects[n]) for n in DEVICES) >= CLEARANCE
        for d in (objects[n] for n in DRINKS)
    ]
    return 0.5 * sum(safe)


def success(objects):
    return score(objects) == 1.0


def demo():
    """Both arms at once: the left arm moves the mug out to the left, the right arm the cup out to the right."""
    travel = TOP + 0.16
    return {
        "left": pick_place_waypoints((*MUG.xy, TOP + 0.045), (0.17, 0.36, TOP + 0.05), travel_z=travel),
        "right": pick_place_waypoints((*CUP.xy, TOP + 0.07), (0.17, -0.38, TOP + 0.075), travel_z=travel),
    }
