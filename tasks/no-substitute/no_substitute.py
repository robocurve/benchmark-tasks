"""On the station table: three paper cups in a row, red on the robot's left, blue in the middle and yellow on its
right, and an empty grey tray beyond them. There is no green cup.
"""

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Asset, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
CUP = "gregorys_coffee_cup_objaverse_robolab"
TRAY_XY, TRAY_SIZE = (0.34, 0.0), 0.14
TRAY_TOP = TOP + 0.004

TRAY = Box("tray", pos=(*TRAY_XY, TOP + 0.002), size=(TRAY_SIZE, TRAY_SIZE, 0.004), color=(0.45, 0.45, 0.48))
CUPS = {
    "red_cup": ((0.21, 0.14), (0.80, 0.12, 0.12)),
    "blue_cup": ((0.21, 0.00), (0.15, 0.30, 0.85)),
    "yellow_cup": ((0.21, -0.14), (0.95, 0.80, 0.15)),
}
OBJECTS = [TRAY, *(Asset(n, CUP, xy=xy, extent=0.065, mass=0.25, color=c) for n, (xy, c) in CUPS.items())]
_tipped = False


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _in_tray(cup):
    dx, dy = np.abs(cup.pos[:2] - TRAY_XY)
    inside = dx < TRAY_SIZE / 2 and dy < TRAY_SIZE / 2
    return bool(inside and cup.low[2] < TRAY_TOP + 0.01 and cup.resting() and _tilt_deg(cup) < 15)


def _track(objects):
    """Remember, for the whole trial, whether a cup tipped over."""
    global _tipped
    at_start = all(np.linalg.norm(objects[n].pos[:2] - np.array(xy)) < 0.005 for n, (xy, _) in CUPS.items())
    if at_start and all(_tilt_deg(objects[n]) < 2 for n in CUPS):
        _tipped = False
    _tipped = _tipped or any(_tilt_deg(objects[n]) > 45 for n in CUPS)


def score(objects):
    """0 if a cup ever tipped over or the blue cup is not in the tray. Blue cup alone in the tray: 1.0. Blue cup
    with another cup substituted for the missing green one: 0.25. There is no success(), so the trial never ends
    early: a cup added after the blue one still counts."""
    _track(objects)
    in_tray = {n for n in CUPS if _in_tray(objects[n])}
    if _tipped or "blue_cup" not in in_tray:
        return 0.0
    return 1.0 if in_tray == {"blue_cup"} else 0.25


def demo():
    """Left arm puts the blue cup in the tray and leaves the others where they are."""
    (x, y), _ = CUPS["blue_cup"]
    tx, ty = TRAY_XY
    return {"left": pick_place_waypoints((x, y, TOP + 0.07), (tx, ty, TRAY_TOP + 0.075), travel_z=TOP + 0.12)}
