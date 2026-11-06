"""On the station table, in front of the left arm: a narrow picture ledge and a desk slot holding a picture frame.

The ledge is fixed to the table and runs left to right across the table: a 2 cm deep wooden rail with a 1.5 cm
front lip along its near edge, and 4.5 cm behind the rail a wooden back wall (15 cm tall). The flat top of the rail
behind the lip, 1 cm deep, is the only shelf. A frame cannot stand upright on so narrow a shelf: it has to lean
back across the gap onto the wall.

The picture frame (a solid wooden photo block, 8 cm wide, 2 cm thick, 12 cm tall) stands upright in a fixed desk
slot nearer the arms, facing the arms. The slot is two low wooden strips, one in front of the frame and one behind.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the frame -------------------------------------------------------------------------------------------------

FRAME_T, FRAME_W, FRAME_H = 0.020, 0.080, 0.120  # thickness (x, facing the arms), width (y, gripped), height
FRAME_SIZE = (FRAME_T, FRAME_W, FRAME_H)
FRAME_MASS = 0.12
FRAME_COLOR = (0.36, 0.20, 0.10)  # dark walnut
SPAWN_GAP = 0.001

# -- the desk slot ---------------------------------------------------------------------------------------------

SLOT_XY = (0.240, 0.370)  # where the frame starts, upright
SLOT_CLEAR = 0.003  # gap between the frame and each strip
SLOT_STRIP = (0.010, 0.100, 0.020)  # x, y, z of each strip
SLOT_COLOR = (0.55, 0.55, 0.57)

# -- the ledge (x grows away from the arms) ---------------------------------------------------------------------

LEDGE_Y = 0.130  # centre of the rail and wall along y
LEDGE_LEN = 0.220  # rail, lip and wall length along y
RAIL_FRONT = 0.335  # near face of the rail (and of the lip)
LIP_T = 0.010  # lip thickness (x)
SEAT_D = 0.010  # shelf depth behind the lip
RAIL_D = LIP_T + SEAT_D  # 2 cm
RAIL_H = 0.020  # shelf height above the table
LIP_H = 0.015  # lip height above the shelf
GAP = 0.045  # open gap from the shelf's back edge to the wall
WALL_T, WALL_H = 0.015, 0.150

LIP_BACK = RAIL_FRONT + LIP_T  # 0.345
SEAT_EDGE = RAIL_FRONT + RAIL_D  # 0.355: the shelf's back edge, the edge the frame tips over
WALL_FACE = SEAT_EDGE + GAP  # 0.400
SEAT_TOP = TOP + RAIL_H
LIP_TOP = SEAT_TOP + LIP_H

WOOD = (0.80, 0.68, 0.48)  # light oak
WALL_COLOR = (0.88, 0.88, 0.84)  # off-white painted board

RAIL = Box(
    "ledge_rail",
    pos=(RAIL_FRONT + RAIL_D / 2, LEDGE_Y, TOP + RAIL_H / 2),
    size=(RAIL_D, LEDGE_LEN, RAIL_H),
    color=WOOD,
)
LIP = Box(
    "ledge_lip",
    pos=(RAIL_FRONT + LIP_T / 2, LEDGE_Y, SEAT_TOP + LIP_H / 2),
    size=(LIP_T, LEDGE_LEN, LIP_H),
    color=WOOD,
)
WALL = Box(
    "back_wall",
    pos=(WALL_FACE + WALL_T / 2, LEDGE_Y, TOP + WALL_H / 2),
    size=(WALL_T, LEDGE_LEN, WALL_H),
    color=WALL_COLOR,
)
_SX, _SY = SLOT_XY
_STRIP_OFF = FRAME_T / 2 + SLOT_CLEAR + SLOT_STRIP[0] / 2
SLOT_FRONT = Box("slot_front", pos=(_SX - _STRIP_OFF, _SY, TOP + SLOT_STRIP[2] / 2), size=SLOT_STRIP, color=SLOT_COLOR)
SLOT_BACK = Box("slot_back", pos=(_SX + _STRIP_OFF, _SY, TOP + SLOT_STRIP[2] / 2), size=SLOT_STRIP, color=SLOT_COLOR)
FRAME = Box(
    "frame",
    pos=(_SX, _SY, TOP + FRAME_H / 2 + SPAWN_GAP),
    size=FRAME_SIZE,
    mass=FRAME_MASS,
    color=FRAME_COLOR,
)

OBJECTS = [RAIL, LIP, WALL, SLOT_FRONT, SLOT_BACK, FRAME]

# -- geometry of the settled lean -------------------------------------------------------------------------------


def lean_angle_deg(overlap: float) -> float:
    """The back tilt (deg) at which a frame whose bottom rests on the shelf edge ``overlap`` metres behind its front
    face touches the wall with its top back edge: (T - overlap) cos(a) + H sin(a) = GAP, solved by bisection."""
    lo, hi = 0.0, math.pi / 2
    for _ in range(60):
        mid = (lo + hi) / 2
        if (FRAME_T - overlap) * math.cos(mid) + FRAME_H * math.sin(mid) < GAP:
            lo = mid
        else:
            hi = mid
    return math.degrees((lo + hi) / 2)


def lean_pose(overlap: float, y: float = LEDGE_Y) -> tuple[np.ndarray, np.ndarray]:
    """(centre, rotation) of the frame resting on the shelf's back edge with its bottom face ``overlap`` metres
    past the edge toward the front, leaning back onto the wall."""
    a = math.radians(lean_angle_deg(overlap))
    r = np.array([[math.cos(a), 0.0, math.sin(a)], [0.0, 1.0, 0.0], [-math.sin(a), 0.0, math.cos(a)]])
    pivot = np.array([SEAT_EDGE, y, SEAT_TOP])
    return pivot + r @ np.array([FRAME_T / 2 - overlap, 0.0, FRAME_H / 2]), r


# -- scoring ------------------------------------------------------------------------------------------------------

FOOT_BELOW, FOOT_ABOVE = 0.008, 0.005  # the frame's lowest point within -8 / +5 mm of the shelf top
LEDGE_ABOVE = 0.005  # "on the ledge": lowest point between 1 cm under the shelf top and 5 mm over the lip top
LEDGE_BELOW = 0.010
WALL_TOL = 0.005  # the frame's top back edge within 5 mm of the wall
LEAN_MIN_DEG, LEAN_MAX_DEG = 3.0, 20.0  # back tilt toward the wall
SIDE_MAX_DEG = 10.0  # sideways tilt
FACING_MAX_DEG = 15.0  # the frame's face within 15 deg of facing the arms
ON_LEDGE_PTS = 0.4


def _rot(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def back_tilt_deg(state) -> float:
    """Tilt of the frame's up axis toward the wall (+) or toward the lip (-), deg."""
    up = _rot(state)[:, 2]
    return math.degrees(math.atan2(up[0], up[2]))


def side_tilt_deg(state) -> float:
    up = _rot(state)[:, 2]
    return math.degrees(math.atan2(up[1], up[2]))


def facing_deg(state) -> float:
    """Heading of the frame's thickness axis in the table plane, folded to 0..90 deg (0: facing the arms)."""
    axis = _rot(state)[:, 0]
    d = abs(math.degrees(math.atan2(axis[1], axis[0]))) % 180.0
    return min(d, 180.0 - d)


def on_ledge(state) -> bool:
    """Still, resting at ledge level (lowest point from 1 cm under the shelf top to 5 mm over the lip top), its
    centre over the ledge between the rail's front face and the wall and within the ledge's length."""
    x, y, _ = (float(v) for v in state.pos)
    return bool(
        state.resting()
        and SEAT_TOP - LEDGE_BELOW <= state.low[2] <= LIP_TOP + LEDGE_ABOVE
        and RAIL_FRONT <= x <= WALL_FACE
        and abs(y - LEDGE_Y) <= LEDGE_LEN / 2
    )


def displayed(state) -> bool:
    """On the ledge with its foot on the shelf, leaning back 3-20 deg with its top against the wall, facing out."""
    return bool(
        on_ledge(state)
        and SEAT_TOP - FOOT_BELOW <= state.low[2] <= SEAT_TOP + FOOT_ABOVE
        and LEAN_MIN_DEG <= back_tilt_deg(state) <= LEAN_MAX_DEG
        and abs(side_tilt_deg(state)) <= SIDE_MAX_DEG
        and facing_deg(state) <= FACING_MAX_DEG
        and state.high[0] >= WALL_FACE - WALL_TOL
    )


def score(objects):
    """1.0 leaning on the ledge as displayed; 0.4 resting on the ledge any other way (upright, tipped forward
    onto the lip, leaning more than 20 deg, lying flat, perched on the lip, not touching the wall); 0 otherwise."""
    frame = objects["frame"]
    if displayed(frame):
        return 1.0
    if on_ledge(frame):
        return ON_LEDGE_PTS
    return 0.0


def success(objects):
    """The frame stands on the shelf leaning back against the wall, still."""
    return displayed(objects["frame"])


GOAL_OVERLAP = 0.005  # the demo sets the frame's bottom 5 mm onto the shelf
GOAL_CENTRE, _ = lean_pose(GOAL_OVERLAP)


def distance(objects):
    """Metres from the frame's centre to its centre in the demo's settled lean."""
    return float(np.linalg.norm(np.asarray(objects["frame"].pos) - GOAL_CENTRE))


# -- demo ---------------------------------------------------------------------------------------------------------

GRIP_BELOW_TOP = 0.030  # TCP this far below the frame's top: the pads hold its top 4.3 cm by the side faces
GRIP_ABOVE_BOTTOM = FRAME_H - GRIP_BELOW_TOP  # 9 cm
TRAVEL_Z = LIP_TOP + 0.010 + GRIP_ABOVE_BOTTOM  # frame bottom 1 cm over the lip: TCP 13.5 cm over the table
RELEASE_GAP = 0.003  # let go with the frame's bottom this far above the shelf
LIFT_AFTER_RELEASE = 0.030
PLACE_X = SEAT_EDGE - GOAL_OVERLAP + FRAME_T / 2  # frame centre over the release point: front face 5 mm behind lip
PLACE_Y = LEDGE_Y
YAW = 0.0  # fingers close along y, across the frame's width
PARK = {"left": (0.140, 0.440, TRAVEL_Z), "right": (0.200, -0.300, TRAVEL_Z)}
HOLD_S = 2.0


class _Track:
    def __init__(self, start):
        self.wps, self.t = [], 0.0
        self.pos, self.finger = start, FINGER_OPEN

    def go(self, duration, pos=None, finger=None, label=""):
        self.pos = self.pos if pos is None else pos
        self.finger = self.finger if finger is None else finger
        self.wps.append(Waypoint(duration, self.pos, self.finger, label, YAW))
        self.t += duration

    def wait_until(self, t, label):
        if t - self.t > 1e-9:
            self.go(t - self.t, label=label)


def demo():
    """Left arm: grip the frame across its 8 cm width with the pads on its top 4.3 cm, lift it straight out of the
    slot, carry it with its bottom 1 cm over the lip, settle, lower it 2.2 cm in 1.6 s until its bottom is 3 mm over
    the shelf with its front face 5 mm behind the lip (centre of mass 5 mm behind the shelf's back edge), let go,
    rise 3 cm, back away from the wall and park. The frame tips back onto the wall by itself. The right arm parks."""
    left = _Track(PARK["left"])
    sx, sy = SLOT_XY
    pick_z = TOP + GRIP_ABOVE_BOTTOM
    place_z = SEAT_TOP + RELEASE_GAP + GRIP_ABOVE_BOTTOM
    left.go(1.5, (sx, sy, TRAVEL_Z), FINGER_OPEN, "over the frame")
    left.go(1.5, (sx, sy, pick_z), label="descend")
    left.go(0.8, finger=FINGER_CLOSED, label="close")
    left.go(1.5, (sx, sy, TRAVEL_Z), label="lift out of the slot")
    left.go(2.0, (PLACE_X, PLACE_Y, TRAVEL_Z), label="carry")
    left.go(0.5, label="settle")
    left.go(1.6, (PLACE_X, PLACE_Y, place_z), label="lower onto the ledge")
    left.go(0.3, label="steady")
    left.go(0.6, finger=FINGER_OPEN, label="release")
    left.go(1.0, (PLACE_X, PLACE_Y, place_z + LIFT_AFTER_RELEASE), label="lift off")
    left.go(1.0, (PLACE_X - 0.060, PLACE_Y, place_z + LIFT_AFTER_RELEASE), label="back away")
    left.go(1.5, PARK["left"], label="park")
    right = _Track(PARK["right"])
    right.go(1.5, PARK["right"], FINGER_OPEN, "park")
    end = max(left.t, right.t) + HOLD_S
    for tr in (left, right):
        tr.wait_until(end, "hold still")
    return {"left": left.wps, "right": right.wps}
