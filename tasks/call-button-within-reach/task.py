"""A patient-room mock. A low bed (a fixed white platform, 28 x 24 cm, 4 cm tall) stands on the robot's left of
center, its head end away from the arms. A patient doll lies on it (fixed blocks: a light-blue gown torso
18 x 10 x 6 cm and a skin-colored head 4 x 6 x 6 cm), with its right hand (a skin-colored block,
5 x 4 x 2 cm) resting on the mattress beside the torso, on the side toward the robot's right. On the robot's
right, nearer the arms, stands a bedside table (a fixed dark-brown block, 12 x 12 cm, 8 cm tall) with the nurse-call
button on it: a red box, 4 x 4 x 2.5 cm.
"""

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint, pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Leave the call button for the patient."

BED_X0, BED_X1, BED_Y0, BED_Y1, BED_H = 0.18, 0.46, -0.06, 0.18, 0.04
BED_TOP = TOP + BED_H
SKIN = (0.93, 0.76, 0.62)
HAND = (0.30, 0.0)  # hand center (x, y)
REACH = 0.08  # within this horizontal distance of the hand center = within reach
TABLE_X, TABLE_Y, TABLE_S, TABLE_H = 0.22, -0.18, 0.12, 0.08

BED = Box("bed", pos=((BED_X0 + BED_X1) / 2, (BED_Y0 + BED_Y1) / 2, TOP + BED_H / 2),
          size=(BED_X1 - BED_X0, BED_Y1 - BED_Y0, BED_H), color=(0.96, 0.96, 0.96))
TORSO = Box("patient_torso", pos=(0.31, 0.09, BED_TOP + 0.03), size=(0.18, 0.10, 0.06), color=(0.60, 0.78, 0.92))
HEAD = Box("patient_head", pos=(0.425, 0.09, BED_TOP + 0.03), size=(0.04, 0.06, 0.06), color=SKIN)
HAND_BOX = Box("patient_hand", pos=(HAND[0], HAND[1], BED_TOP + 0.01), size=(0.05, 0.04, 0.02), color=SKIN)
TABLE = Box("bedside_table", pos=(TABLE_X, TABLE_Y, TOP + TABLE_H / 2), size=(TABLE_S, TABLE_S, TABLE_H),
            color=(0.42, 0.30, 0.20))
BUTTON = Box("call_button", pos=(TABLE_X, TABLE_Y, TOP + TABLE_H + 0.0125), size=(0.04, 0.04, 0.025), mass=0.05,
             color=(0.85, 0.08, 0.08))
OBJECTS = [BED, TORSO, HEAD, HAND_BOX, TABLE, BUTTON]


def _on_bed(s):
    """Still, its center over the bed and its bottom on the mattress or on the patient's hand (2 cm), not
    up on the torso or head (6 cm)."""
    return bool(
        s.resting() and BED_X0 < s.pos[0] < BED_X1 and BED_Y0 < s.pos[1] < BED_Y1
        and BED_TOP - 0.01 < s.low[2] < BED_TOP + 0.03
    )


def _near_hand(s):
    return float(np.hypot(s.pos[0] - HAND[0], s.pos[1] - HAND[1])) <= REACH


def success(objects):
    """1.0: the button rests on the mattress (or on the hand) with its center within 8 cm of the center of the patient's hand."""
    b = objects["call_button"]
    return _on_bed(b) and _near_hand(b)


def score(objects):
    """1.0 on the mattress within 8 cm of the hand; 0.5 on the mattress but farther from the hand (at the
    foot, by the head, on the far side); 0 anywhere else: on the bedside table, on the patient, on the floor
    (table top), or held."""
    b = objects["call_button"]
    if not _on_bed(b):
        return 0.0
    return 1.0 if _near_hand(b) else 0.5


def demo():
    """Right arm: lift the button off the bedside table and set it on the mattress just beside the hand, on
    the hand's open side."""
    travel = TOP + 0.16
    grip = (TABLE_X, TABLE_Y, TOP + TABLE_H + 0.025 - 0.012 + 0.013)
    place = (0.30, -0.042, BED_TOP + 0.005 + 0.025 - 0.012 + 0.013)
    right = pick_place_waypoints(grip, place, travel_z=travel)
    right.append(Waypoint(1.5, (0.15, -0.25, TOP + 0.25), FINGER_OPEN, "park"))
    return {"right": right}
