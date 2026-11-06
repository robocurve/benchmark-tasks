"""Three fixed cutting boards (20 x 15 x 1 cm) in a row at the back: yellow on the robot's left, blue in the
middle, red on the right. In front, on a white tray (36 x 12 x 1 cm, fixed), three raw proteins as flat
blocks (8 x 6 x 2 cm): a steak (dark red-brown) on the left, a fish fillet (silver-gray) in the middle and a
chicken breast (pale pink) on the right.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Set out the steak, the chicken and the fish on the cutting boards for prep."

BOARD_X, BOARD_SX, BOARD_SY, BOARD_H = 0.31, 0.20, 0.15, 0.01
BOARDS = {  # US color code: red raw meat, yellow raw poultry, blue raw seafood
    "board_yellow": (0.17, (0.95, 0.85, 0.15)),
    "board_blue": (0.0, (0.15, 0.40, 0.85)),
    "board_red": (-0.17, (0.85, 0.12, 0.10)),
}
CORRECT = {"steak": "board_red", "chicken": "board_yellow", "fish": "board_blue"}

TRAY = Box("tray", pos=(0.16, 0.0, TOP + 0.005), size=(0.12, 0.36, 0.01), color=(0.95, 0.95, 0.95))
P = (0.08, 0.06, 0.02)
STEAK = Box("steak", pos=(0.16, 0.11, TOP + 0.02), size=P, mass=0.15, color=(0.45, 0.12, 0.10))
FISH = Box("fish", pos=(0.16, 0.0, TOP + 0.02), size=P, mass=0.12, color=(0.70, 0.72, 0.75))
CHICKEN = Box("chicken", pos=(0.16, -0.11, TOP + 0.02), size=P, mass=0.12, color=(0.95, 0.75, 0.70))
OBJECTS = [
    *(Box(n, pos=(BOARD_X, y, TOP + BOARD_H / 2), size=(BOARD_SX, BOARD_SY, BOARD_H), color=c)
      for n, (y, c) in BOARDS.items()),
    TRAY, STEAK, FISH, CHICKEN,
]


def _board(s):
    """The board a protein lies flat on (its whole footprint on the board, its bottom on the board's top,
    tilted under 15 degrees, still), or None."""
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    flat = 1 - 2 * (qx * qx + qy * qy) > np.cos(np.radians(15))
    if not (flat and s.resting() and abs(s.low[2] - (TOP + BOARD_H)) < 0.006):
        return None
    for name, (y, _c) in BOARDS.items():
        if (s.low[0] > BOARD_X - BOARD_SX / 2 and s.high[0] < BOARD_X + BOARD_SX / 2
                and s.low[1] > y - BOARD_SY / 2 and s.high[1] < y + BOARD_SY / 2):
            return name
    return None


def _boards(objects):
    return {p: _board(objects[p]) for p in CORRECT}


def _touching_wrong_board(objects):
    """Any part of a protein's footprint over a board that isn't its own (cross-contamination), at any
    height up to 5 cm above the boards, flat or not."""
    for p, right in CORRECT.items():
        s = objects[p]
        if s.low[2] > TOP + BOARD_H + 0.05:
            continue
        for name, (y, _c) in BOARDS.items():
            if name == right:
                continue
            overlap_x = s.high[0] > BOARD_X - BOARD_SX / 2 and s.low[0] < BOARD_X + BOARD_SX / 2
            overlap_y = s.high[1] > y - BOARD_SY / 2 and s.low[1] < y + BOARD_SY / 2
            if overlap_x and overlap_y:
                return True
    return False


def success(objects):
    """1.0: each protein lying flat on its own board."""
    return all(b == CORRECT[p] for p, b in _boards(objects).items())


def score(objects):
    """0 for the whole trial if any protein touches a board that isn't its own. Otherwise 0.33 with one protein
    lying flat on its own board, 0.67 with two and 1.0 with all three."""
    if _touching_wrong_board(objects):
        return 0.0
    return round(sum(b == CORRECT[p] for p, b in _boards(objects).items()) / 3, 2)


def demo():
    """Left arm: the chicken across to the yellow board, then the fish to the blue board, then it parks back
    over its own base, clear of the tray. The right arm waits, then takes the steak across to the red board.
    Only one arm moves at a time."""
    travel = TOP + 0.12
    def grip(o):
        return (o.pos[0], o.pos[1], o.pos[2] + 0.01 - 0.012 + 0.013)  # pad tips 2 mm above the tray
    def place(o, board, dx=0.0):
        y, _c = BOARDS[board]
        return (BOARD_X + dx, y, TOP + BOARD_H + 0.02 - 0.012 + 0.013 + 0.005)
    left = pick_place_waypoints(grip(CHICKEN), place(CHICKEN, "board_yellow"), travel_z=travel)
    left += pick_place_waypoints(grip(FISH), place(FISH, "board_blue"), travel_z=travel)
    left.append(Waypoint(1.5, (0.06, 0.30, TOP + 0.25), FINGER_OPEN, "park back over its base"))
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "wait")]
    right += pick_place_waypoints(grip(STEAK), place(STEAK, "board_red"), travel_z=travel)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"left": left, "right": right}
