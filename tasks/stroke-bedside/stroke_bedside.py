"""On the station table: a tan board manikin of a patient lying on his back, head away from the robot, so his
left side is on the robot's right. A red call button, a white tissue box and a black TV remote stand on the table
on the patient's left, and a thin grey pad lies on the midline in front of him. The three items are loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=stroke_bedside.py --scorer task_score \\
           --instruction "Sam had a stroke and can't use his left arm. Put his call button, tissues and TV remote where he can reach them." ...
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

TAN = (0.80, 0.66, 0.52)
BOARD = 0.01  # the manikin's thickness (m)
HAND_XY = (0.25, 0.135)  # the patient's working (right) hand, on the robot's left
REACH = 0.13  # an item counts as reachable with its centre this close to that hand (m)
ON_TABLE = 0.005  # an item's lowest point must be this close to the table (m), so not on the manikin
PAD_H = 0.002


def _board(name, x, y, size_x, size_y):
    return Box(name, pos=(x, y, TOP + BOARD / 2), size=(size_x, size_y, BOARD), color=TAN)


MANIKIN = [
    _board("torso", 0.37, 0.0, 0.18, 0.20),
    _board("right_upper_arm", 0.35, 0.135, 0.16, 0.04),
    _board("left_upper_arm", 0.35, -0.135, 0.16, 0.04),
    _board("right_hand", HAND_XY[0], HAND_XY[1], 0.04, 0.04),
    _board("left_hand", HAND_XY[0], -HAND_XY[1], 0.04, 0.04),
    Cylinder("head", pos=(0.525, 0.0, TOP + 0.025), radius=0.06, height=0.05, color=TAN),
]
RELAY_XY = (0.18, 0.0)
PAD = Box("pad", pos=(*RELAY_XY, TOP + PAD_H / 2), size=(0.08, 0.08, PAD_H), color=(0.55, 0.55, 0.58))

# Sizes keep every item narrow enough for a top-down grip that closes along world y.
BUTTON = Box("call_button", pos=(0.19, -0.10, TOP + 0.02), size=(0.04, 0.04, 0.04), mass=0.05,
             color=(0.85, 0.12, 0.12))
TISSUES = Box("tissues", pos=(0.19, -0.19, TOP + 0.02), size=(0.06, 0.04, 0.04), mass=0.05,
              color=(0.95, 0.95, 0.92))
REMOTE = Box("remote", pos=(0.26, -0.22, TOP + 0.015), size=(0.10, 0.035, 0.03), mass=0.05,
             color=(0.08, 0.08, 0.09))
ITEMS = [BUTTON, TISSUES, REMOTE]
OBJECTS = [*MANIKIN, PAD, *ITEMS]

WEIGHTS = {"call_button": 0.4, "tissues": 0.3, "remote": 0.3}
TARGETS = {"call_button": (0.19, 0.10), "tissues": (0.19, 0.19), "remote": (0.26, 0.22)}


def _reachable(item):
    """On the table, still, and with its centre within REACH of the working hand."""
    near = np.hypot(item.pos[0] - HAND_XY[0], item.pos[1] - HAND_XY[1]) <= REACH
    return bool(near and item.low[2] < TOP + ON_TABLE and item.resting())


def success(objects):
    return all(_reachable(objects[name]) for name in WEIGHTS)


def distance(objects):
    """The farthest item's distance from the working hand (m)."""
    return float(max(np.hypot(objects[n].pos[0] - HAND_XY[0], objects[n].pos[1] - HAND_XY[1]) for n in WEIGHTS))


def score(objects):
    """Partial credit (success scores 1.0): 0.4 for the call button, 0.3 each for the tissues and the remote."""
    return float(sum(w for name, w in WEIGHTS.items() if _reachable(objects[name])))


GRIP_Z = TOP + 0.02  # the lowest grip height proven on this table
TRAVEL_Z = TOP + 0.14
CYCLE_S = 12.0  # each arm spends this long per item
HANDOVER_DELAY_S = 11.1  # the left arm starts once the right arm has set the first item down and moved clear


def _cycle(pick_xy, place_xy, park_xy, place_z):
    """One pick and place, then a move clear of the pad, padded to CYCLE_S so the arms stay in step."""
    steps = pick_place_waypoints((*pick_xy, GRIP_Z), (*place_xy, place_z), travel_z=TRAVEL_Z)
    park = (*park_xy, TRAVEL_Z)
    steps.append(Waypoint(1.5, park, FINGER_OPEN, "clear"))
    steps.append(Waypoint(CYCLE_S - sum(w.duration for w in steps), park, FINGER_OPEN, "wait"))
    return steps


def demo():
    """The right arm sets each item on the pad, and the left arm carries it on to the patient's right hand."""
    relay_z = GRIP_Z + PAD_H + 0.005
    right, left = [], [Waypoint(HANDOVER_DELAY_S, (0.20, 0.30, TRAVEL_Z), FINGER_OPEN, "wait")]
    for item in ITEMS:
        right += _cycle(item.pos[:2], RELAY_XY, (0.20, -0.30), relay_z)
        left += _cycle(RELAY_XY, TARGETS[item.name], (0.20, 0.30), GRIP_Z + 0.005)
    return {"right": right, "left": left}
