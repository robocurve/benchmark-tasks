"""In a stockroom: a small steel parts case and two cardboard cartons of the same size wait on the bench, to be
stacked on the taped pallet position. Nobody says so, but the heavy steel case goes at the bottom: on top it would
crush the cartons and make the stack top-heavy.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
BENCH = TOP + 0.003
BL, BW, BH = 0.09, 0.065, 0.05  # every box: front to back, width, height
MARK = np.array([0.34, 0.18])
CARDBOARD, STEEL = (0.72, 0.55, 0.36), (0.45, 0.48, 0.52)

CASE = Box("steel_case", pos=(0.17, 0.30, BENCH + BH / 2), size=(BL, BW, BH), mass=1.5, color=STEEL)
CARTONS = [Box(name, pos=(0.17, y, BENCH + BH / 2), size=(BL, BW, BH), mass=0.3, color=CARDBOARD)
           for name, y in (("carton_a", 0.18), ("carton_b", 0.06))]
RACK = (0.30, 0.32, 0.36)
DRESSING = [
    Box("bench", pos=(0.45, 0.0, TOP + 0.0015), size=(0.90, 1.0, 0.003), color=(0.62, 0.62, 0.60)),
    Box("pallet_mark", pos=(*map(float, MARK), BENCH + 0.0001), size=(0.13, 0.11, 0.0002), color=(0.95, 0.75, 0.15)),
    *[Box(f"rack_post_{i}", pos=(0.70, y, BENCH + 0.10), size=(0.03, 0.03, 0.20), color=RACK)
      for i, y in enumerate((-0.40, -0.05, 0.30))],
    *[Box(f"rack_shelf_{i}", pos=(0.70, -0.05, BENCH + z), size=(0.10, 0.74, 0.012), color=RACK)
      for i, z in enumerate((0.06, 0.14))],
    Box("stock_bin_a", pos=(0.70, -0.25, BENCH + 0.10), size=(0.08, 0.10, 0.06), color=(0.20, 0.40, 0.70)),
    Box("stock_bin_b", pos=(0.70, 0.12, BENCH + 0.10), size=(0.08, 0.10, 0.06), color=(0.80, 0.30, 0.20)),
    Box("hand_truck_plate", pos=(0.36, -0.26, BENCH + 0.004), size=(0.12, 0.20, 0.008), color=(0.15, 0.15, 0.17)),
    Box("clipboard", pos=(0.55, -0.05, BENCH + 0.004), size=(0.12, 0.17, 0.008), yaw=0.25, color=(0.55, 0.40, 0.25)),
    Box("tape_gun", pos=(0.56, 0.30, BENCH + 0.02), size=(0.12, 0.05, 0.04), yaw=-0.4, color=(0.85, 0.20, 0.15)),
    Cylinder("tape_roll", pos=(0.50, -0.30, BENCH + 0.025), radius=0.045, height=0.05, color=(0.75, 0.65, 0.45)),
]
OBJECTS = [*DRESSING, CASE, *CARTONS]


def _tilt_deg(obj):
    x, y, _, _ = obj.quat_xyzw
    return float(np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _level(obj):
    return round((obj.low[2] - BENCH) / BH)


def _over(obj, xy):
    return float(np.linalg.norm(np.array(obj.pos[:2]) - xy)) <= 0.03


def score(objects):
    """0 if any box ends tipped over (more than 30 degrees) or the steel case ends raised on another box. Otherwise
    0.5 for the steel case resting on the pallet mark (centre within 3 cm of the mark's), plus 0.25 for each carton
    stacked squarely on the column above it (centre within 3 cm of the case's, resting at the next level up).
    Scored at the end of the trial."""
    case, cartons = objects["steel_case"], [objects[c.name] for c in CARTONS]
    if any(_tilt_deg(o) > 30 for o in (case, *cartons)) or case.low[2] > BENCH + 0.02:
        return 0.0
    if not (_over(case, MARK) and case.resting()):
        return 0.0
    column = [c for c in cartons if _over(c, np.array(case.pos[:2])) and c.resting()]
    levels = sorted(_level(c) for c in column)
    stacked = 0
    for want in (1, 2):
        if want in levels:
            stacked += 1
        else:
            break
    return 0.5 + 0.25 * stacked


def _carry(src, level, lift_z, via=None):
    grip = BENCH + 0.013 + BH / 2
    place = BENCH + level * BH + 0.013 + BH / 2 + 0.008
    x, y = src
    route = [Waypoint(2.0, (*via, lift_z), FINGER_CLOSED, "forward first")] if via else []
    return [Waypoint(1.5, (x, y, lift_z), FINGER_OPEN, "over"),
            Waypoint(1.5, (x, y, grip), FINGER_OPEN, "descend"),
            Waypoint(0.6, (x, y, grip), FINGER_OPEN, "settle"),
            Waypoint(1.0, (x, y, grip), FINGER_CLOSED, "grip"),
            Waypoint(2.0, (x, y, lift_z), FINGER_CLOSED, "lift"),
            *route,
            Waypoint(2.5, (*MARK, lift_z), FINGER_CLOSED, "carry"),
            Waypoint(1.5, (*MARK, place), FINGER_CLOSED, "lower"),
            Waypoint(0.8, (*MARK, place), FINGER_OPEN, "release"),
            Waypoint(1.0, (*MARK, lift_z), FINGER_OPEN, "up")]


def demo():
    """The left arm carries the steel case low onto the pallet mark, going forward first so it clears the cartons,
    then stacks the two cartons on it and moves clear of the camera's view."""
    high = TOP + 0.205
    left = [*_carry(CASE.pos[:2], 0, TOP + 0.10, via=(float(MARK[0]), CASE.pos[1])),
            *_carry(CARTONS[0].pos[:2], 1, high),
            *_carry(CARTONS[1].pos[:2], 2, high),
            Waypoint(2.0, (0.12, 0.42, TOP + 0.20), FINGER_OPEN, "move clear of the camera's view")]
    return {"left": left}
