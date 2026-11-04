"""On the station table: a short gravity flow lane, sloping down to the left, with a fixed stop at its low end. Three
totes stand in the lane against the stop, nose to tail: a red-tagged tote at the front and two grey totes behind it, which
slide forward the moment nothing holds them. A green bin stands on the table left of the lane. The goal is to take the
red tote out and put it in the bin, keeping the queue from moving more than 2 cm until it is in the bin, then to let the
queue slide forward so the next tote is against the stop.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \\
           --instruction "Hold back the grey totes, put the red tote in the green bin, then let the grey totes slide forward to the stop. They must not move more than 2 cm until the red tote is in the bin." \\
           --scorer task_score ...
"""

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))
from rigidparts import rigid_part  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL, RED, GREY, GREEN = (0.45, 0.47, 0.50), (0.80, 0.15, 0.12), (0.62, 0.64, 0.67), (0.20, 0.55, 0.25)

SLOPE = math.radians(10)  # the lane falls toward +y (left)
LANE_X, LANE_Y, LANE_Z = 0.22, -0.06, TOP + 0.05  # where the lane's frame sits
FRICTION = 0.05  # the lane's contact friction: rollers, in effect
TOTE = (0.06, 0.12, 0.05)  # across the lane, along it, tall
TOTE_MASS = 0.25
LANE_Y0, LANE_Y1 = -0.24, 0.20  # the lane's ends, along its own length
STOP_Y = LANE_Y1 - 0.01  # the stop's face, along the lane
FRONT, SECOND, THIRD = STOP_Y - TOTE[1] / 2, STOP_Y - 1.5 * TOTE[1] - 0.001, STOP_Y - 2.5 * TOTE[1] - 0.002


def lane_point(along, up=0.0):
    """A point of the lane's frame in the world: ``along`` its length toward the stop, ``up`` from its surface."""
    return (LANE_X, LANE_Y + along * math.cos(SLOPE) + up * math.sin(SLOPE), LANE_Z - along * math.sin(SLOPE) + up * math.cos(SLOPE))


LENGTH = LANE_Y1 - LANE_Y0
MID = (LANE_Y0 + LANE_Y1) / 2
HALF_W = TOTE[0] / 2 + 0.003  # 6 mm of play across the lane
LANE = rigid_part(
    "lane",
    pos=lane_point(0.0),
    boxes=[((0.0, MID, -0.005), (2 * HALF_W + 0.02, LENGTH, 0.01), STEEL),
           ((HALF_W + 0.005, MID, 0.01), (0.01, LENGTH, 0.02), STEEL),
           ((-HALF_W - 0.005, MID, 0.01), (0.01, LENGTH, 0.02), STEEL),
           ((0.0, STOP_Y + 0.005, 0.02), (2 * HALF_W, 0.01, 0.04), STEEL)],
    friction=FRICTION,
    slick=True,
    tilt=-SLOPE,
)


def tote(name, along, color):
    h = TOTE[2] / 2
    return rigid_part(name, pos=lane_point(along, h + 0.0005), boxes=[((0.0, 0.0, 0.0), TOTE, color)], mass=TOTE_MASS,
                      tilt=-SLOPE)


RED_TOTE = tote("red_tote", FRONT, RED)
SECOND_TOTE = tote("second_tote", SECOND, GREY)
THIRD_TOTE = tote("third_tote", THIRD, GREY)

# The lane's legs, so it doesn't look as if it floats.
LEGS = [Box(f"leg_{i}", pos=(LANE_X, y, TOP + (z - TOP) / 2), size=(0.06, 0.02, z - TOP - 0.012), color=STEEL)
        for i, (y, z) in enumerate((lane_point(a)[1:] for a in (0.15, -0.20)))]

BIN_XY, BIN_IN, BIN_H, BIN_WALL = (0.17, 0.33), 0.16, 0.035, 0.01
_off = BIN_IN / 2 + BIN_WALL / 2
BIN = [
    Box("bin_0", pos=(BIN_XY[0] + _off, BIN_XY[1], TOP + BIN_H / 2), size=(BIN_WALL, BIN_IN + 2 * BIN_WALL, BIN_H), color=GREEN),
    Box("bin_1", pos=(BIN_XY[0] - _off, BIN_XY[1], TOP + BIN_H / 2), size=(BIN_WALL, BIN_IN + 2 * BIN_WALL, BIN_H), color=GREEN),
    Box("bin_2", pos=(BIN_XY[0], BIN_XY[1] + _off, TOP + BIN_H / 2), size=(BIN_IN, BIN_WALL, BIN_H), color=GREEN),
    Box("bin_3", pos=(BIN_XY[0], BIN_XY[1] - _off, TOP + BIN_H / 2), size=(BIN_IN, BIN_WALL, BIN_H), color=GREEN),
]
OBJECTS = [LANE, RED_TOTE, SECOND_TOTE, THIRD_TOTE, *LEGS, *BIN]

MAX_CREEP = 0.02  # m the queue may move before the red tote is in the bin
AT_STOP = 0.005  # m: the next tote's nose within this of where the red tote's was


def _along(state) -> float:
    """How far along the lane a tote's centre is."""
    return (float(state.pos[1]) - LANE_Y) * math.cos(SLOPE) + (LANE_Z - float(state.pos[2])) * math.sin(SLOPE)


def in_bin(objects) -> bool:
    """The red tote's centre is inside the bin's walls and it rests on the table."""
    red = objects["red_tote"]
    return bool(abs(red.pos[0] - BIN_XY[0]) < BIN_IN / 2 and abs(red.pos[1] - BIN_XY[1]) < BIN_IN / 2 and red.low[2] < TOP + 0.01)


class Watch:
    """How far the queue crept before the red tote reached the bin. A jump of over 3 cm back up the lane in one control step
    is a reset; totes never slide that way on their own, however fast the queue runs forward."""

    def __init__(self):
        self.last, self.creep, self.binned = None, 0.0, False

    def update(self, objects):
        now = _along(objects["second_tote"])
        if self.last is not None and now < self.last - 0.03:
            self.creep, self.binned = 0.0, False
        self.last = now
        self.binned = self.binned or in_bin(objects)
        if not self.binned:
            self.creep = max(self.creep, now - SECOND)


WATCH = Watch()


def forward(objects) -> bool:
    """The next tote stands against the stop, with the last one behind it."""
    second, third = _along(objects["second_tote"]), _along(objects["third_tote"])
    in_lane = all(abs(objects[n].pos[0] - LANE_X) < 0.01 for n in ("second_tote", "third_tote"))
    return bool(abs(second - FRONT) < AT_STOP and third < second and in_lane)


def success(objects):
    """The red tote in the bin, the queue forward, the queue never early by over 2 cm, all still; must hold for 1 s. Runs
    first in every control step, so it also keeps the record of how far the queue crept."""
    WATCH.update(objects)
    return bool(in_bin(objects) and forward(objects) and WATCH.creep <= MAX_CREEP
                and all(objects[n].resting() for n in ("red_tote", "second_tote", "third_tote")))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 if the red tote is in the bin and the queue forward but the queue crept
    over 2 cm early; 0.25 if the red tote is in the bin but the queue isn't forward."""
    if not in_bin(objects):
        return 0.0
    return 0.5 if forward(objects) else 0.25


def demo():
    """Right arm: grip the second tote and hold it, so the queue can't move. Left arm: lift the red tote straight up, carry
    it to the bin and let go. Then the right arm lets go and backs up, and the queue slides forward to the stop."""
    yaw_l, yaw_r = -np.pi / 2, np.pi / 2  # fingers across the lane
    part = FINGER_OPEN * 0.85
    grip_up = TOTE[2] - 0.012  # up from the lane's surface: above the side rails, 12 mm below the tote's top
    # Gripped toward the far ends of the two totes: the hands need about 19 cm between them.
    second = lane_point(SECOND - 0.035, grip_up)
    front = lane_point(FRONT + 0.035, grip_up)
    above = TOP + 0.18
    right = [
        Waypoint(1.5, (second[0], second[1], above), part, "above the second tote", yaw_r),
        Waypoint(1.5, second, part, "down to it", yaw_r),
        Waypoint(0.8, second, FINGER_CLOSED, "grip it", yaw_r),
        Waypoint(9.5, second, FINGER_CLOSED, "hold the queue", yaw_r),
        Waypoint(0.8, second, part, "let go", yaw_r),
        Waypoint(1.0, (second[0], second[1], above), part, "up, out of the way", yaw_r),
        Waypoint(2.5, (second[0], second[1], above), part, "watch the queue slide forward", yaw_r),
    ]
    drop = (BIN_XY[0], BIN_XY[1], TOP + TOTE[2] - 0.012 + 0.006)
    left = [
        Waypoint(2.8, (front[0], front[1], above), part, "above the red tote", yaw_l),
        Waypoint(1.5, front, part, "down to it", yaw_l),
        Waypoint(0.8, front, FINGER_CLOSED, "grip it", yaw_l),
        Waypoint(1.5, (front[0], front[1], front[2] + 0.09), FINGER_CLOSED, "lift it straight out", yaw_l),
        Waypoint(2.0, (drop[0], drop[1], front[2] + 0.09), FINGER_CLOSED, "carry it over the bin", yaw_l),
        Waypoint(1.2, drop, FINGER_CLOSED, "lower it in", yaw_l),
        Waypoint(0.6, drop, part, "let go", yaw_l),
        Waypoint(1.0, (drop[0], drop[1], above), part, "back up", yaw_l),
    ]
    return {"left": left, "right": right}
