"""On the station table: a large, empty, closed cardboard box (50 cm deep, 40 cm wide, 20 cm tall, 1 kg) lies flat
in the middle of the table, between the arms. Its front face is 26 cm from the table's far edge (the edge
straight ahead of the robot, at x = +0.91 m). Slide the box toward that edge, as close as you can, without any
part of it sticking out past the edge.

Scoring (at the end of the episode): 0 if any part of the box is past any edge of the table, or if the box is not
flat on the table top. Otherwise the gap is the average distance from the far edge to the box's two corners
nearest it, and the score is linear in the gap: 0 at the starting gap (26 cm) and 1.0 at 1 cm or less, so each
centimetre is worth 0.04. Run with -E object_jitter=0: the code uses the fixed starting gap. There is no early
end on success, so the box can still be pushed over the edge after it reaches full score.

Run it:  inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=slide_cardboard_box.py \\
           -E object_jitter=0 --scorer task_score --grader none --max-steps 2400 \\
           --instruction "Slide the cardboard box as close as you can to the far edge of the table without letting any part of it hang over the edge."
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.station import TABLE_CENTER, TABLE_SIZE
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

CARDBOARD = (0.62, 0.48, 0.32)

# ---- geometry (m) ----
TABLE_LOW = np.subtract(TABLE_CENTER[:2], np.divide(TABLE_SIZE[:2], 2))  # x -0.19, y -0.65
TABLE_HIGH = np.add(TABLE_CENTER[:2], np.divide(TABLE_SIZE[:2], 2))  # x 0.91, y 0.65
EDGE_X = float(TABLE_HIGH[0])  # the far edge, straight ahead of the arms
BOX_SIZE = (0.50, 0.40, 0.20)  # x (towards the edge), y (across), z
BOX_MASS = 1.0  # kg: an empty corrugated moving box
BOX_POS = (0.40, 0.0, TOP + BOX_SIZE[2] / 2)  # middle of the table; table centre is x 0.36
START_GAP = EDGE_X - (BOX_POS[0] + BOX_SIZE[0] / 2)  # 0.26; score 0 here, so run with -E object_jitter=0

# ---- thresholds ----
FULL_GAP = 0.01  # score 1.0 at an average gap of 1 cm or less; 0 at START_GAP or more, linear in between
ON_TABLE_TOL = 0.01  # the box's bottom within 1 cm of the table top
MAX_TILT_DEG = 10.0  # the box's top face within 10 deg of level

OBJECTS = [
    Box("cardboard_box", pos=BOX_POS, size=BOX_SIZE, mass=BOX_MASS, color=CARDBOARD),
]


def _rotation(quat_xyzw) -> np.ndarray:
    x, y, z, w = quat_xyzw
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def overhang(objects) -> bool:
    """Any part of the box is past any of the table's four edges. The world-aligned bounding box includes the
    corners of a turned box."""
    box = objects["cardboard_box"]
    return bool(np.any(box.low[:2] < TABLE_LOW) or np.any(box.high[:2] > TABLE_HIGH))


def gap(objects) -> float:
    """Average distance (m) from the plane of the table's far edge to the box's two corners nearest it, seen from
    above. Negative values mean the corners are past the edge."""
    box = objects["cardboard_box"]
    hx, hy = BOX_SIZE[0] / 2, BOX_SIZE[1] / 2
    corners = np.array([[sx * hx, sy * hy, 0.0] for sx in (-1, 1) for sy in (-1, 1)])
    xs = box.pos[0] + corners @ _rotation(box.quat_xyzw)[0]
    return float(EDGE_X - np.sort(xs)[-2:].mean())


def _tilt_deg(quat_xyzw):
    x, y, _, _ = quat_xyzw
    up_z = 1.0 - 2.0 * (x * x + y * y)  # z component of the box's up axis
    return float(np.degrees(np.arccos(np.clip(up_z, -1.0, 1.0))))


def _flat_on_table(box):
    return abs(box.low[2] - TOP) <= ON_TABLE_TOL and _tilt_deg(box.quat_xyzw) <= MAX_TILT_DEG


def score(objects):
    """0 for any overhang or a box not flat on the table top. Otherwise linear in the gap, clipped to 0..1:
    0 at START_GAP, 1.0 at FULL_GAP or less."""
    if overhang(objects) or not _flat_on_table(objects["cardboard_box"]):
        return 0.0
    return float(np.clip((START_GAP - gap(objects)) / (START_GAP - FULL_GAP), 0.0, 1.0))


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
