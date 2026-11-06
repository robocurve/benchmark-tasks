"""Two identical grey four-pocket tube racks (pockets 30 mm square inside, 3.5 cm deep, fixed walls, the
table as the floor) stand one behind the other. The near one (the holder) holds four capped blood tubes
(16 mm x 10 cm, each coloured by its cap) in mixed order: lavender, green, light blue, gold from the robot's
left to right. The far one (the draw rack) is empty; a black marker block at its left end marks pocket 1.
"""

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Set the blood tubes in the rack in the order you'll draw them."

# Pocket geometry, shared by both racks.
POCKET = 0.030  # inside width
WALL = 0.004
PITCH = POCKET + WALL  # 3.4 cm between pocket centres
DEPTH = 0.035  # wall height
POCKET_Y = [1.5 * PITCH, 0.5 * PITCH, -0.5 * PITCH, -1.5 * PITCH]  # pocket 1 (robot's left) .. pocket 4
HOLDER_X, RACK_X = 0.20, 0.31
GREY = (0.6, 0.6, 0.62)

R_TUBE, H_TUBE = 0.008, 0.10
CAPS = {  # CLSI order of draw: citrate, serum, heparin, EDTA
    "tube_citrate": (0.55, 0.80, 0.95),  # light blue
    "tube_serum": (0.90, 0.75, 0.20),  # gold
    "tube_heparin": (0.20, 0.65, 0.30),  # green
    "tube_edta": (0.70, 0.55, 0.85),  # lavender
}
ORDER = ["tube_citrate", "tube_serum", "tube_heparin", "tube_edta"]  # pocket 1 .. 4
START = ["tube_edta", "tube_heparin", "tube_citrate", "tube_serum"]  # holder pocket 1 .. 4


def _rack(prefix, cx):
    """Front and back rails along y, five dividers along x: four pockets in a row."""
    span = 4 * PITCH + WALL
    off = POCKET / 2 + WALL / 2
    parts = [
        Box(f"{prefix}_front", pos=(cx - off, 0.0, TOP + DEPTH / 2), size=(WALL, span, DEPTH), color=GREY),
        Box(f"{prefix}_back", pos=(cx + off, 0.0, TOP + DEPTH / 2), size=(WALL, span, DEPTH), color=GREY),
    ]
    for i in range(5):
        y = 2 * PITCH - i * PITCH
        parts.append(Box(f"{prefix}_div{i}", pos=(cx, y, TOP + DEPTH / 2), size=(POCKET, WALL, DEPTH), color=GREY))
    return parts


MARKER = Box("pocket1_marker", pos=(RACK_X, 2 * PITCH + 0.015, TOP + 0.01), size=(0.02, 0.02, 0.02),
             color=(0.05, 0.05, 0.05))
TUBES = [
    Cylinder(name, pos=(HOLDER_X, POCKET_Y[i], TOP + H_TUBE / 2), radius=R_TUBE, height=H_TUBE, mass=0.012,
             color=CAPS[name])
    for i, name in enumerate(START)
]
OBJECTS = [*_rack("holder", HOLDER_X), *_rack("rack", RACK_X), MARKER, *TUBES]


def _bottom(s):
    """Centre of the tube's bottom face: the body centre minus half its length along its own axis."""
    qx, qy, qz, qw = s.quat_xyzw
    axis = np.array([2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy)])
    return s.pos - axis * H_TUBE / 2, axis[2]


def _pocket(s, cx):
    """Which pocket (0..3) of the rack at cx the tube is seated in, or None. Seated: its bottom on the table
    inside the pocket, tilted under 30 degrees, and still."""
    b, cos_tilt = _bottom(s)
    if not (s.resting() and cos_tilt > np.cos(np.radians(30)) and b[2] < TOP + 0.01):
        return None
    if abs(b[0] - cx) > POCKET / 2:
        return None
    for i, y in enumerate(POCKET_Y):
        if abs(b[1] - y) < POCKET / 2:
            return i
    return None


def _seated(objects):
    return {name: _pocket(objects[name], RACK_X) for name in ORDER}


def success(objects):
    """1.0: each tube seated in the draw rack in order of draw, pocket 1 (robot's left) to 4."""
    seated = _seated(objects)
    return all(seated[name] == i for i, name in enumerate(ORDER))


def score(objects):
    """0 whenever any tube is seated in the wrong pocket of the draw rack (a wrong order is what causes
    carryover). Otherwise 0.25 for each tube seated in its own pocket; a tube still in the holder, lying
    down, on the table or held earns nothing. All four in their own pockets is 1.0."""
    seated = _seated(objects)
    if any(seated[name] is not None and seated[name] != i for i, name in enumerate(ORDER)):
        return 0.0
    return 0.25 * sum(seated[name] == i for i, name in enumerate(ORDER))


# -- demo -------------------------------------------------------------------------------------------------

OPEN_PART = -0.02  # pads 4 cm apart: enough for a 16 mm tube, clear of the neighbours
GRIP_Z = TOP + H_TUBE - 0.025 + 0.013  # pad tips 2.5 cm below the cap
TRAVEL_Z = TOP + 0.15
SET_Z = GRIP_Z + 0.003  # tube bottom 3 mm above the table
HOVER_Z = GRIP_Z + DEPTH + 0.015  # tube bottom 1.5 cm above the rack walls
YAW = np.pi / 2  # fingers close along x, so open pads stay clear of the tubes either side


def _move(src_y, dst_y, k=0.7):
    a, b = (HOLDER_X, src_y), (RACK_X, dst_y)
    return [
        Waypoint(1.2 * k, (*a, TRAVEL_Z), OPEN_PART, "over the tube", YAW),
        Waypoint(1.2 * k, (*a, GRIP_Z), OPEN_PART, "descend", YAW),
        Waypoint(0.8, (*a, GRIP_Z), FINGER_CLOSED, "close", YAW),
        Waypoint(1.2 * k, (*a, TRAVEL_Z), FINGER_CLOSED, "lift", YAW),
        Waypoint(1.5 * k, (*b, TRAVEL_Z), FINGER_CLOSED, "carry", YAW),
        Waypoint(1.0 * k, (*b, HOVER_Z), FINGER_CLOSED, "down to just above the rack", YAW),
        Waypoint(0.6, (*b, HOVER_Z), FINGER_CLOSED, "settle", YAW),
        Waypoint(1.5, (*b, SET_Z), FINGER_CLOSED, "lower into the pocket", YAW),
        Waypoint(0.6, (*b, SET_Z), OPEN_PART, "release", YAW),
        Waypoint(1.0 * k, (*b, TRAVEL_Z), OPEN_PART, "up", YAW),
    ]


def demo():
    """Left arm fills pockets 1 and 2, then parks; the right arm waits, then fills pockets 3 and 4."""
    src = {name: POCKET_Y[i] for i, name in enumerate(START)}
    left = _move(src["tube_citrate"], POCKET_Y[0]) + _move(src["tube_serum"], POCKET_Y[1])
    left.append(Waypoint(1.5, (0.15, 0.25, TOP + 0.25), OPEN_PART, "park"))
    wait = sum(w.duration for w in left)
    right = [Waypoint(wait, (0.15, -0.25, TOP + 0.25), OPEN_PART, "wait")]
    right += _move(src["tube_heparin"], POCKET_Y[2]) + _move(src["tube_edta"], POCKET_Y[3])
    return {"left": left, "right": right}
