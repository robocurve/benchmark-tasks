"""On the station table, in front of the left arm (positions in cm from the midpoint between the arm bases, x forward, y
to the robot's left): a dark grey electric motor (10 cm long, 6 cm wide, 5 cm tall) fixed with its centre at (13.7,
12.3), turned 30 degrees anticlockwise from straight out, so its shaft points forward and to the left. An orange shaft
stub (1.8 x 1.8 cm, 3 cm long) sticks out of the centre of its front face, which is at (18.0, 14.8), 2.5 cm above the
table. Two blue pillow-block bearings (3 cm thick, 6 cm wide, 5 cm tall; the bore runs through the 3 cm thickness, 2.5
cm up) stand on the table further left: one centred at (23, 30), turned 10 degrees, and one at (36, 33), turned 55
degrees. The motor and stub are fixed; the bearings are loose. The job: set the bearings in line with the motor's shaft,
one about 10 cm and one about 20 cm out from the motor's front face, each centred on the shaft's line (within 3 mm) and
square to it (within 2 degrees), so a straight shaft would pass through the motor and both bores.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]

SHAFT_YAW = math.radians(30.0)
ALONG = np.array([math.cos(SHAFT_YAW), math.sin(SHAFT_YAW)])  # the shaft's direction, out of the motor
ACROSS = np.array([-ALONG[1], ALONG[0]])
MOTOR = (0.10, 0.06, 0.05)  # length along the shaft, width, height: clearly longer than wide, so it shows the axis
SHAFT_Z = TOP + 0.025
FACE_XY = np.array([0.15, 0.13]) + 0.035 * ALONG  # the motor's front face, where the shaft comes out
MOTOR_XY = FACE_XY - MOTOR[0] / 2 * ALONG
# 1.8 cm across and 3 cm long, so the stub shows from the top camera (at 1.2 x 2.5 cm it was 2 pixels at 320 px)
STUB = (0.030, 0.018, 0.018)
FIXED = [  # no mass: fixed
    Box("motor", pos=(*MOTOR_XY, TOP + MOTOR[2] / 2), size=MOTOR, color=(0.18, 0.19, 0.21), yaw=SHAFT_YAW),
    Box("shaft_stub", pos=(*(FACE_XY + STUB[0] / 2 * ALONG), SHAFT_Z), size=STUB, color=(0.95, 0.50, 0.05),
        yaw=SHAFT_YAW),
]

BEARING, BEARING_KG = (0.03, 0.06, 0.05), 0.20  # thickness (the bore's direction), width, height; a hollow cast housing
BEARING_BLUE = (0.12, 0.35, 0.80)
START = {  # centre (m) and turn (degrees, anticlockwise from straight out)
    "bearing_a": ((0.23, 0.30), 10.0),
    "bearing_b": ((0.36, 0.33), 55.0),
}
NAMES = tuple(START)
BEARINGS = [Box(name, pos=(*xy, TOP + BEARING[2] / 2 + 0.001), size=BEARING, mass=BEARING_KG, color=BEARING_BLUE,
                yaw=math.radians(turn)) for name, (xy, turn) in START.items()]  # each starts 1 mm up, and settles
OBJECTS = [*FIXED, *BEARINGS]

SLOTS = {"near": (0.08, 0.12), "far": (0.18, 0.22)}  # from the motor's face along the shaft: 10 and 20 cm, +-2 cm
ALIGN_MM, ALIGN_DEG = 0.003, 2.0  # in line: centre within 3 mm of the shaft's line, square within 2 degrees
LINE_DEG = 1.0  # full score also needs the line through both bearings within 1 degree of the shaft, so a straight
# shaft through both bores would meet the motor's (review: two 3 mm errors in opposite directions, 3 cm apart, tilted
# that line 11 degrees)
EPS = 1e-9  # so a limit reads the same both ways (+3.00 mm scored 0.75 and -3.00 mm 1.0 through rounding)
ROUGH_MM, ROUGH_DEG = 0.010, 6.0  # roughly in line: half credit
MAX_TILT_DEG = 5.0  # standing on its base
TABLE_TOL = 0.003  # on the table: its lowest point within 3 mm of it
ALIGNED_CREDIT, ROUGH_CREDIT = 0.5, 0.25  # per slot

# Where a bearing is comes from its centre and rotation, not from ``low``/``high``: Arena builds those from the
# block's corners, turned with it, so they grow when a block turns about the vertical.


def _rot(quat_xyzw) -> np.ndarray:
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _corners(block) -> np.ndarray:
    half = np.array(BEARING) / 2
    signs = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    return np.asarray(block.pos) + (signs * half) @ _rot(block.quat_xyzw).T


def _tilt_deg(block) -> float:
    return float(np.degrees(np.arccos(min(1.0, abs(_rot(block.quat_xyzw)[2, 2])))))


def along(block) -> float:
    """How far the bearing's centre is out along the shaft from the motor's face (m)."""
    return float((np.asarray(block.pos[:2]) - FACE_XY) @ ALONG)


def off_line(block) -> float:
    """How far the bearing's centre is from the shaft's line, seen from above (m)."""
    return abs(float((np.asarray(block.pos[:2]) - FACE_XY) @ ACROSS))


def squareness_deg(block) -> float:
    """Angle between the bearing's bore (its 3 cm axis) and the shaft, seen from above (0 to 90 degrees)."""
    bore = _rot(block.quat_xyzw)[:2, 0]
    norm = float(np.linalg.norm(bore))
    if norm < 1e-9:  # bore pointing straight up: a bearing on its face
        return 90.0
    return float(np.degrees(np.arccos(min(1.0, abs(float(bore @ ALONG)) / norm))))


def _standing(block) -> bool:
    return _tilt_deg(block) <= MAX_TILT_DEG and abs(float(_corners(block)[:, 2].min()) - TOP) <= TABLE_TOL


def _off_table(block) -> bool:
    return float(_corners(block)[:, 2].min()) < TOP - 0.01


def _slot_credit(block, slot) -> float:
    lo, hi = SLOTS[slot]
    if not (_standing(block) and block.resting() and lo - EPS <= along(block) <= hi + EPS):
        return 0.0
    if off_line(block) <= ALIGN_MM + EPS and squareness_deg(block) <= ALIGN_DEG + EPS:
        return ALIGNED_CREDIT
    if off_line(block) <= ROUGH_MM + EPS and squareness_deg(block) <= ROUGH_DEG + EPS:
        return ROUGH_CREDIT
    return 0.0


def line_deg(near, far) -> float:
    """Angle between the shaft and the line through the two bearings' centres, seen from above (degrees)."""
    d = np.asarray(far.pos[:2], dtype=float) - np.asarray(near.pos[:2], dtype=float)
    return float(np.degrees(np.arctan2(abs(float(d @ ACROSS)), abs(float(d @ ALONG)))))


def _grade(objects) -> float:
    a, b = (objects[n] for n in NAMES)
    if _off_table(a) or _off_table(b):
        return 0.0
    # Either bearing may go in either place: score both ways round and keep the better (review: judging the nearer
    # one in the near place scored the same half job 0 or 0.5 depending on which bearing was moved).
    best, pair = max(((_slot_credit(n, "near") + _slot_credit(f, "far"), (n, f)) for n, f in ((a, b), (b, a))),
                     key=lambda c: c[0])
    if best == 2 * ALIGNED_CREDIT and line_deg(*pair) > LINE_DEG + EPS:
        return 0.75  # both in line on their own, but a straight shaft through both would miss the motor's
    return round(best, 6)


def success(objects):
    """Both bearings in line: one 8-12 cm and one 18-22 cm out from the motor's face along the shaft, each centred on
    the shaft's line within 3 mm and square to it within 2 degrees, the line through both within 1 degree of the
    shaft, standing and still. Must hold for 1 s."""
    return _grade(objects) == 1.0


def score(objects):
    """Partial credit (success scores 1.0): for each of the two places along the shaft (8-12 and 18-22 cm out; either
    bearing can go in either), 0.5 if a bearing stands there in line (within 3 mm and 2 degrees), 0.25 if roughly in
    line (within 10 mm and 6 degrees). Both in line but the line through them more than 1 degree off the shaft: 0.75.
    0 if a bearing ends off the table."""
    return _grade(objects)


GRIP_ABOVE = 0.030  # grip point above a bearing's bottom: the pad tips 1.7 cm above the table
TRAVEL_Z = TOP + 0.12  # a carried bearing's bottom 4 cm above the motor and the other bearing
DROP = 0.001  # each bearing is let go 1 mm above the table
DEMO_OUT = {"bearing_a": 0.10, "bearing_b": 0.20}  # where the demo puts each, out along the shaft from the face
# Where each bearing lands against where it is let go, in mm along and across the shaft and degrees of turn, from the
# first official run (2026-11-03, replayed): the arm stops 3.6-4.6 mm short and 2.7-5.1 mm to the shaft's left, and
# each bearing comes out turned 0.7-0.8 degrees anticlockwise, which left bearing_a 5.1 mm off the line (score 0.75).
# The demo lets go of each that much the other way.
DEMO_LANDING = {"bearing_a": (-3.56, 5.06, 0.74), "bearing_b": (-4.55, 2.66, 0.76)}
# With these, the next official run (replayed) left the bearings 0.36 and 0.06 mm off the line and square within 0.01
# degree. After the motor and stub were enlarged (same front face), the official demo scored 1.0 again.
# Every bearing's aim moved this far across the shaft (mm, + to its left). 0 for the demo; the margin runs set it to
# +1 and -1 to check the bearings stay in line if every one lands a millimetre off. Official runs (2026-11-04,
# replayed), bearings off the line (mm) and the line through both (degrees): demo 0.36, 0.06, 0.24; +1 mm 1.17, 0.91,
# 0.15; -1 mm 0.61, 1.64, 0.58. All three 1.0, every bearing square within 0.2 degree.
DEMO_AIM_SHIFT_MM = 0.0
# Grip across the bearing's 3 cm thickness with the fingers aimed 22 mm apart, 4 mm inside each face: each finger
# presses about 20 N, under its 50 N limit, so the pair centres the block (fully closed, both push at the limit and a
# part can slide across the grip: 17 mm on the official stack, in the snug-tray task).
GRIP_FINGER = FINGER_OPEN * 22 / 94
RELEASE_FINGER = FINGER_OPEN * 46 / 94  # let go gently: open slowly to 46 mm, 8 mm clear each side


def _grip_yaw(turn_deg) -> float:
    """The grip's turn (rad) that closes the fingers across a block turned ``turn_deg``: the fingers close along the
    grip's y, the block's thickness runs along its own x."""
    return math.radians(turn_deg - 90.0)


def _move(pick, pick_yaw, place, place_yaw) -> list[Waypoint]:
    """A top-down pick and place that turns the grip from ``pick_yaw`` to ``place_yaw`` while carrying: wait over the
    block before gripping it, set it down slowly, open the fingers slowly to RELEASE_FINGER, then lift slowly."""
    (px, py, pz), (qx, qy, qz) = pick, place
    return [
        Waypoint(1.5, (px, py, TRAVEL_Z), FINGER_OPEN, "to the object", pick_yaw),
        Waypoint(1.5, (px, py, pz), FINGER_OPEN, "descend", pick_yaw),
        Waypoint(0.5, (px, py, pz), FINGER_OPEN, "steady", pick_yaw),
        Waypoint(0.8, (px, py, pz), GRIP_FINGER, "close", pick_yaw),
        Waypoint(1.2, (px, py, TRAVEL_Z), GRIP_FINGER, "lift", pick_yaw),
        Waypoint(2.0, (qx, qy, TRAVEL_Z), GRIP_FINGER, "carry", place_yaw),
        Waypoint(0.5, (qx, qy, TRAVEL_Z), GRIP_FINGER, "hover", place_yaw),
        Waypoint(2.0, (qx, qy, qz), GRIP_FINGER, "set down", place_yaw),
        Waypoint(0.3, (qx, qy, qz), GRIP_FINGER, "hold", place_yaw),
        Waypoint(1.5, (qx, qy, qz), RELEASE_FINGER, "release", place_yaw),
        Waypoint(0.3, (qx, qy, qz), RELEASE_FINGER, "settle", place_yaw),
        Waypoint(1.5, (qx, qy, TRAVEL_Z), RELEASE_FINGER, "retreat", place_yaw),
    ]


def demo_targets() -> dict[str, np.ndarray]:
    """Where the demo aims each bearing's centre: on the shaft's line, DEMO_OUT from the face."""
    return {n: FACE_XY + DEMO_OUT[n] * ALONG for n in NAMES}


def demo():
    """Left arm: bearing_a, then bearing_b, each gripped across its 3 cm thickness (the grip turned to match it),
    lifted 4 cm above every top, turned square to the shaft while carried, and set down slowly on the shaft's line
    (10 and 20 cm out from the motor's face, allowing for DEMO_LANDING), let go gently 1 mm above the table."""
    waypoints = []
    for name, target in demo_targets().items():
        (sx, sy), turn = START[name]
        d_along, d_across, d_turn = DEMO_LANDING[name]
        aim = target - d_along / 1000 * ALONG + (DEMO_AIM_SHIFT_MM - d_across) / 1000 * ACROSS
        waypoints += _move((sx, sy, TOP + GRIP_ABOVE), _grip_yaw(turn),
                           (*aim, TOP + DROP + GRIP_ABOVE), _grip_yaw(math.degrees(SHAFT_YAW) - d_turn))
    return {"left": waypoints}
