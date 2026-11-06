"""On the station table: two placemats in front of the robot, a green one on its left and a yellow one on its
right, each marking the seat of a person who sits across the table, facing the robot. A red mug stands beside each
mat, nearer the robot, with its handle pointing at the robot.
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MAT_X, MAT_SIZE = 0.32, (0.14, 0.16)
MAT_TOP = TOP + 0.003
HANDLE_OFFSET = 0.0085  # the handle shifts the mug's bounding-box centre this far from its body centre

# Each person faces the robot, so their left is the robot's right (-90 deg) and their right the robot's left (+90).
MATS = {
    "green_mat": (0.10, (0.20, 0.55, 0.30), -90.0),  # left-handed person
    "yellow_mat": (-0.10, (0.90, 0.80, 0.20), 90.0),  # right-handed person
}
OBJECTS = [
    *(Box(n, pos=(MAT_X, y, TOP + 0.0015), size=(*MAT_SIZE, 0.003), color=c) for n, (y, c, _) in MATS.items()),
    Asset("mug_left", "mug_ycb_robolab", xy=(0.18, 0.20), extent=0.085, mass=0.30, yaw=np.pi),
    Asset("mug_right", "mug_ycb_robolab", xy=(0.18, -0.20), extent=0.085, mass=0.30, yaw=np.pi),
]
MUGS = {o.name: np.array(o.xy) for o in OBJECTS if isinstance(o, Asset)}
_tipped = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def handle_heading_deg(mug):
    """Direction the handle points, in degrees about the vertical (0: away from the robot, 90: robot's left)."""
    x, y, z, w = mug.quat_xyzw
    return float(np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))))


def _on_mat(mug, mat_y):
    dx, dy = abs(mug.pos[0] - MAT_X), abs(mug.pos[1] - mat_y)
    inside = dx < MAT_SIZE[0] / 2 and dy < MAT_SIZE[1] / 2
    return inside and mug.low[2] < MAT_TOP + 0.01 and mug.resting() and _tilt_deg(mug) < 15


def _track(objects):
    """Remember, for the whole trial, whether a mug tipped over."""
    global _tipped
    if all(np.linalg.norm(objects[n].pos[:2] - xy) < 0.005 and _tilt_deg(objects[n]) < 2 for n, xy in MUGS.items()):
        _tipped = False
    _tipped = _tipped or any(_tilt_deg(objects[n]) > 30 for n in MUGS)


def _mat_score(objects, mat_y, goal_deg):
    best = 0.0
    for n in MUGS:
        mug = objects[n]
        if _on_mat(mug, mat_y):
            off = abs((handle_heading_deg(mug) - goal_deg + 180) % 360 - 180)
            best = max(best, 0.5 if off <= 30 else 0.1)
    return best


def score(objects):
    """0 if a mug ever tilted past 30 deg. Per mat: 0.5 for an upright mug on it with the handle within 30 deg of
    the side of the person's dominant hand, 0.1 for a mug on it facing elsewhere."""
    _track(objects)
    if _tipped:
        return 0.0
    return sum(_mat_score(objects, y, goal) for y, _, goal in MATS.values())


def success(objects):
    return score(objects) == 1.0


def demo():
    """Each arm grips its mug across the body and turns it 90 deg on the way: the left arm toward the robot's
    right for the left-handed person, the right arm toward the robot's left for the right-handed person."""

    def serve(xy, mat_y, turn):
        body = (xy[0] + HANDLE_OFFSET, xy[1], TOP + 0.04)
        return pick_place_waypoints(body, (MAT_X, mat_y, MAT_TOP + 0.045), travel_z=TOP + 0.12, yaw=0.0, place_yaw=turn)

    return {
        "left": serve(MUGS["mug_left"], MATS["green_mat"][0], np.pi / 2),
        "right": serve(MUGS["mug_right"], MATS["yellow_mat"][0], -np.pi / 2),
    }
