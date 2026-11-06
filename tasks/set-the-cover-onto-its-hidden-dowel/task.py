"""On the station table, between the arms: the top of a gearbox housing, a dark grey block fixed to the table, with
one small square dowel hole in its top. A grey inspection cover, a square block too wide to grip, rests on the
housing top. A steel locating dowel sticks out of the cover's underside, out of sight; the cover rests tilted on it.
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
import parts  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

HOUSING_C, COVER_C, STEEL_C = (0.20, 0.21, 0.23), (0.62, 0.64, 0.66), (0.72, 0.73, 0.75)
FRICTION = (0.4, 0.3)

# Housing (fixed): 320 (x) x 260 (y) x 40 mm, its top 40 mm above the table. One 12 mm square dowel hole, 10 mm deep,
# in its top. World frame: x away from the arm bases, y to the robot's left, z up (m).
HOUSING_XY = (0.330, -0.010)
HOUSING = (0.320, 0.260, 0.040)
HOLE, HOLE_DEPTH = 0.012, 0.010
HOLE_XY = (0.325, 0.023)  # off both of the housing's centre lines
H_TOP = TOP + HOUSING[2]


def _housing() -> parts.Part:
    lx, ly, h = HOUSING[0] / 2, HOUSING[1] / 2, HOUSING[2]
    hx, hy = HOLE_XY[0] - HOUSING_XY[0], HOLE_XY[1] - HOUSING_XY[1]  # hole centre in the part frame
    r, d = HOLE / 2, HOLE_DEPTH
    z = h - d / 2
    return parts.Part("housing", pos=(*HOUSING_XY, TOP), friction=FRICTION, shapes=[
        parts.box((2 * lx, 2 * ly, h - d), at=(0.0, 0.0, (h - d) / 2), color=HOUSING_C),
        # Top layer, 10 mm thick, round the hole: two strips beside it running the full length (x), then the strips
        # behind and beyond it, as wide as the hole. Every seam runs along x, the way the cover slides.
        parts.box((2 * lx, hy - r + ly, d), at=(0.0, (-ly + hy - r) / 2, z), color=HOUSING_C),
        parts.box((2 * lx, ly - hy - r, d), at=(0.0, (ly + hy + r) / 2, z), color=HOUSING_C),
        parts.box((hx - r + lx, HOLE, d), at=((-lx + hx - r) / 2, hy, z), color=HOUSING_C),
        parts.box((lx - hx - r, HOLE, d), at=((lx + hx + r) / 2, hy, z), color=HOUSING_C),
    ])


# Cover (loose): 130 x 130 x 30 mm, 0.5 kg, plain top. Body origin at the centre of its underside. The dowel, a 10 mm
# steel pin 8 mm long, stands out of the underside 40 mm from the centre toward the robot (-x).
COVER = (0.130, 0.130, 0.030)
COVER_MASS = 0.5
DOWEL_R, DOWEL_L = 0.005, 0.008
DOWEL_AT = (-0.040, 0.0)
SEATED_C = (HOLE_XY[0] - DOWEL_AT[0], HOLE_XY[1] - DOWEL_AT[1])  # cover centre when seated: (0.365, 0.023), off centre
START_OFF = (-0.018, -0.016)  # at the start the cover is 18 mm short of seated (x) and 16 mm to the robot's right
COVER_START = (SEATED_C[0] + START_OFF[0], SEATED_C[1] + START_OFF[1])  # (0.347, 0.007): over the hole

COVER_PART = parts.Part(
    "cover", pos=(*COVER_START, H_TOP + DOWEL_L + 0.0005), mass=COVER_MASS, friction=FRICTION, shapes=[
        parts.box(COVER, at=(0.0, 0.0, COVER[2] / 2), color=COVER_C),
        parts.cylinder(DOWEL_R, DOWEL_L, at=(*DOWEL_AT, -DOWEL_L / 2), color=STEEL_C, axis="z"),
    ])

OBJECTS = [_housing(), COVER_PART]
SUCCESS_HOLD_S = 4.0  # seated and still for 4 s, so a demo arm that only pushed has backed away

# -- scoring --------------------------------------------------------------------------------------------------------
FLAT_DEG = 1.5  # the cover's top within 1.5 degrees of level (resting tilted on its dowel: 4.58 degrees, measured)
FLAT_TOL = 0.0015  # every corner of the underside within 1.5 mm of the housing top
IN_HOLE = (HOLE - 2 * DOWEL_R) / 2 + 0.0005  # dowel axis within 1.5 mm of the hole centre, in x and in y
DOWN_BY = 0.005  # dowel tip more than 5 mm below the housing top
_HALF = np.array(COVER) / 2
_UNDER = np.array([[sx * _HALF[0], sy * _HALF[1], 0.0] for sx in (1, -1) for sy in (1, -1)])
_DOWEL_TIP = np.array([*DOWEL_AT, -DOWEL_L])


def _rot(q) -> np.ndarray:
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_deg(st) -> float:
    """Angle between the cover's up axis and the vertical (degrees); 180 for upside down."""
    return float(np.degrees(np.arccos(np.clip(_rot(st.quat_xyzw)[2, 2], -1.0, 1.0))))


def dowel_tip(st) -> np.ndarray:
    return np.asarray(st.pos, float) + _rot(st.quat_xyzw) @ _DOWEL_TIP


def seated(st) -> bool:
    """Flat on the housing top with the dowel down in its hole, and still."""
    r = _rot(st.quat_xyzw)
    under = np.asarray(st.pos, float)[:, None] + r @ _UNDER.T
    tip = dowel_tip(st)
    return bool(
        tilt_deg(st) < FLAT_DEG
        and np.all(np.abs(under[2] - H_TOP) <= FLAT_TOL + 1e-6)
        and abs(tip[0] - HOLE_XY[0]) <= IN_HOLE + 1e-6
        and abs(tip[1] - HOLE_XY[1]) <= IN_HOLE + 1e-6
        and tip[2] < H_TOP - DOWN_BY
        and st.resting()
    )


def success(objects):
    return seated(objects["cover"])


def score(objects):
    """Pass/fail: 1.0 when the cover lies flat on the housing with its dowel in the hole, at rest; 0 otherwise."""
    return 1.0 if seated(objects["cover"]) else 0.0


def distance(objects):
    tip = dowel_tip(objects["cover"])
    return float(np.hypot(tip[0] - HOLE_XY[0], tip[1] - HOLE_XY[1]))


# -- scripted demo --------------------------------------------------------------------------------------------------
# A fixed raster search, reading no state. The cover is slid sideways (y) across the housing in rows, alternately by
# the right arm (pushing its right face, +y) and the left arm (pushing its left face, -y); between rows the left arm
# pushes its near face 9 mm further from the robot (+x). The rows cover x offsets from 18 mm short of seated to 9 mm
# past it and y offsets from 16 mm right to 18 mm left. Once the dowel drops into the hole the cover cannot slide,
# and every later push only presses against it (the soft arm gives way); without the hole the pattern would carry the
# cover 27 mm further from the robot.
TIP_PAST_TCP = 0.013
PUSH_TIP_Z = 0.020  # finger tips 20 mm above the housing top: on a face whether the cover is tilted or flat
PUSH_Z = H_TOP + PUSH_TIP_Z + TIP_PAST_TCP
OVER_Z = H_TOP + 0.075  # TCP height that clears the cover
FINGER_FRONT = 0.008  # TCP to the pushing face of an open finger (measured: about 8 mm)
CLEAR = 0.015  # gap left before a push
ROW_DX = 0.009
ROWS = 4
Y_LO, Y_HI = -0.016, 0.018  # cover y offsets from seated at the ends of each row
PARK = {"left": (0.22, 0.20, H_TOP + 0.10), "right": (0.22, -0.20, H_TOP + 0.10)}
YAW = {"x": 0.0, "+y": math.pi / 2, "-y": -math.pi / 2}  # the open gripper is symmetric front to back


def _plan():
    """Both arms' waypoints, phase by phase (each phase has the same duration on both arms)."""
    hx, hy = SEATED_C[0] + START_OFF[0], SEATED_C[1] + Y_LO
    arms = {"left": [], "right": []}
    pose = {a: (PARK[a], 0.0) for a in arms}

    def phase(dur, moves, label):
        for a in arms:
            if a in moves:
                pose[a] = moves[a]
            p, yaw = pose[a]
            arms[a].append(Waypoint(dur, p, FINGER_OPEN, label, yaw))

    def push(arm, start, end, yaw, dur, label):
        """Over the start point, down, push to the end, back off 15 mm, up, back to its park; the other arm waits."""
        back = np.asarray(start) - np.asarray(end)
        back = back / np.linalg.norm(back) * 0.015
        e = (end[0] + back[0], end[1] + back[1])
        phase(0.9, {arm: ((start[0], start[1], OVER_Z), yaw)}, f"{label}: over")
        phase(0.6, {arm: ((start[0], start[1], PUSH_Z), yaw)}, f"{label}: down")
        phase(dur, {arm: ((end[0], end[1], PUSH_Z), yaw)}, f"{label}: push")
        phase(0.4, {arm: ((e[0], e[1], PUSH_Z), yaw)}, f"{label}: back off")
        phase(0.5, {arm: ((e[0], e[1], OVER_Z), yaw)}, f"{label}: up")
        phase(0.9, {arm: (PARK[arm], 0.0)}, f"{label}: clear")

    phase(1.5, {}, "start")
    cy_from, cy_to = SEATED_C[1] + Y_LO, SEATED_C[1] + Y_HI
    for r in range(ROWS):
        cx = hx + r * ROW_DX
        if r % 2 == 0:  # right arm pushes the right face (-y side) toward +y
            f0, f1 = cy_from - COVER[1] / 2, cy_to - COVER[1] / 2
            push("right", (cx, f0 - FINGER_FRONT - CLEAR), (cx, f1 - FINGER_FRONT), YAW["+y"], 3.0, f"row {r + 1}")
        else:  # left arm pushes the left face (+y side) toward -y
            f0, f1 = cy_to + COVER[1] / 2, cy_from + COVER[1] / 2
            push("left", (cx, f0 + FINGER_FRONT + CLEAR), (cx, f1 + FINGER_FRONT), YAW["-y"], 3.0, f"row {r + 1}")
        cy = cy_to if r % 2 == 0 else cy_from
        if r < ROWS - 1:
            n0 = cx - COVER[0] / 2
            push("left", (n0 - FINGER_FRONT - CLEAR, cy), (n0 + ROW_DX - FINGER_FRONT, cy), YAW["x"], 1.2,
                 f"step {r + 1}")
    return arms


def demo():
    return _plan()
