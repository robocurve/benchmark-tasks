"""On the station table: a large, empty, closed cardboard box (50 cm deep, 40 cm wide, 20 cm tall, 1 kg) lies flat
in the middle of the table, between the arms. Its front face is 26 cm from the table's far edge (the edge
straight ahead of the robot, at x = +0.91 m). Slide the box toward that edge, as close as you can, without any
part of it sticking out past the edge.

Scoring (at the end of the episode): the gap is the shortest distance from the box to the plane of the table's
far edge. Any overhang (gap below 0), a box that isn't resting flat on the table top, or a box off the table
scores 0. Otherwise the closer the better: 1.0 within 1 cm, 0.75 within 2 cm, 0.5 within 4 cm, 0.25 within
8 cm, else 0. There is no early end on success: the episode runs to its limit (or until the policy stops), so
the box can still be pushed over the edge after reaching full score.

Run it:  inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=slide_cardboard_box.py \\
           -E object_jitter=0 --scorer task_score --grader none --max-steps 1200 \\
           --instruction "Slide the cardboard box as close as you can to the far edge of the table without letting any part of it hang over the edge."
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

CARDBOARD = (0.62, 0.48, 0.32)

# ---- geometry (m) ----
EDGE_X = 0.91  # the station table's far edge (table spans x -0.19..0.91)
BOX_SIZE = (0.50, 0.40, 0.20)  # x (towards the edge), y (across), z
BOX_MASS = 1.0  # kg: an empty corrugated moving box
BOX_POS = (0.40, 0.0, TOP + BOX_SIZE[2] / 2)  # middle of the table; table centre is x 0.36
START_GAP = EDGE_X - (BOX_POS[0] + BOX_SIZE[0] / 2)  # 0.26

# ---- thresholds ----
TIERS = ((0.01, 1.0), (0.02, 0.75), (0.04, 0.5), (0.08, 0.25))  # gap at most (m) -> score
ON_TABLE_TOL = 0.01  # the box's bottom within 1 cm of the table top
MAX_TILT_DEG = 10.0  # the box's top face within 10 deg of level

OBJECTS = [
    Box("cardboard_box", pos=BOX_POS, size=BOX_SIZE, mass=BOX_MASS, color=CARDBOARD),
]


def gap(objects):
    """Shortest distance (m) from any part of the box to the plane of the table's far edge; negative is
    overhang. The world-aligned bounding box includes the corners of a turned box."""
    return float(EDGE_X - objects["cardboard_box"].high[0])


def _tilt_deg(quat_xyzw):
    x, y, _, _ = quat_xyzw
    up_z = 1.0 - 2.0 * (x * x + y * y)  # z component of the box's up axis
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


def _flat_on_table(box):
    return abs(box.low[2] - TOP) <= ON_TABLE_TOL and _tilt_deg(box.quat_xyzw) <= MAX_TILT_DEG


def score(objects):
    """0 for any overhang or a box not flat on the table top; otherwise by the gap to the edge."""
    box = objects["cardboard_box"]
    g = gap(objects)
    if g < 0.0 or not _flat_on_table(box):
        return 0.0
    for limit, value in TIERS:
        if g <= limit:
            return value
    return 0.0


# No success(): it would end the episode early with 1.0 and spare the robot the risk of pushing on past the
# edge. The score is the box's state at the last step (SCORE_OVER = "end").


def distance(objects):
    return max(gap(objects), 0.0)


# ---- demo ----
PUSH_Z = TOP + 0.013 + 0.05  # TCP height: closed fingertips 5 cm above the table
PUSH_Y = 0.14  # each arm pushes 14 cm off the box's centre line, so the box goes straight
CLEAR_Y = 0.28  # the grippers come down outside the box's sides (y +-0.20)
FINGER_FACE = 0.022  # closed fingers' +x face past the TCP (m); measured in sim (0.015 overshot by 7 mm)
TARGET_GAP = 0.005


def demo():
    """Both arms, fingers closed: come down beside the box, move in behind its back face, push it straight
    ahead until its front face is 5 mm from the far edge, back off and lift."""
    back = BOX_POS[0] - BOX_SIZE[0] / 2
    start_x = back - FINGER_FACE - 0.03
    touch_x = back - FINGER_FACE
    end_x = EDGE_X - TARGET_GAP - BOX_SIZE[0] - FINGER_FACE
    hover = TOP + 0.16
    c = FINGER_CLOSED
    plan = {}
    for arm, s in (("left", 1.0), ("right", -1.0)):
        plan[arm] = [
            Waypoint(2.0, (start_x, s * CLEAR_Y, hover), c, "beside the box"),
            Waypoint(1.5, (start_x, s * CLEAR_Y, PUSH_Z), c, "descend"),
            Waypoint(1.5, (start_x, s * PUSH_Y, PUSH_Z), c, "behind the box"),
            Waypoint(1.0, (touch_x, s * PUSH_Y, PUSH_Z), c, "touch"),
            Waypoint(6.0, (end_x, s * PUSH_Y, PUSH_Z), c, "push"),
            Waypoint(1.0, (end_x - 0.04, s * PUSH_Y, PUSH_Z), c, "back off"),
            Waypoint(1.5, (end_x - 0.04, s * PUSH_Y, hover), c, "lift"),
        ]
    return plan
