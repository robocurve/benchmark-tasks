"""On the station table, on a green cutting mat: a 45 x 45 mm aluminium T-slot profile, 30 cm long, lies along y at
x = 0.30, fixed, its slot facing up, black end caps closing both ends. The slot opens 16 mm at the top, between two
5 mm lips, onto a 30 mm wide, 9 mm high channel. A red tape mark crosses the profile's top at y = -0.12. In front of
the profile, on the mat at (0.21, -0.14), a yellow-zinc drop-in T-nut (24 x 10 x 6 mm, 24 mm along x) stands with an
8 mm stud 40 mm tall screwed into it, its handle. A speed square and a marker lie on the robot's left.

The task: drop the T-nut into the slot, turn it a quarter turn so it locks under the lips, slide it along the slot to
the red mark, and let go.

The T-nut is 10 mm wide and 24 mm long: it goes through the 16 mm opening only lengthwise (along the slot), and once
it is in the channel and turned crosswise, its ends sit 4 mm under each lip and it can't come out. The end caps close
the channel, so the nut can't be slid in from an end. ``compound.py`` makes the T-nut and its stud one rigid body of
two box colliders. ``fingers.py`` tells the scorer whether a finger still touches the nut.

Run it:  inspect-robots run --embodiment yam_arena -E task=t_nut_quarter_turn.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from bonds import on_reset, on_step  # noqa: E402
from compound import Compound, compound  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the cutting mat -------------------------------------------------------------------------------------------------
MAT_T = 0.002
MAT_TOP = TOP + MAT_T
MAT_XY, MAT_SIZE = (0.28, -0.08), (0.28, 0.36)
MAT = Box("mat", pos=(*MAT_XY, TOP + MAT_T / 2), size=(*MAT_SIZE, MAT_T), color=(0.10, 0.36, 0.22))

# -- the profile: along y, slot up. Its cross-section (x, z), from the slot line and the mat --------------------------
PROFILE_X = 0.30  # the slot's line
PROFILE_Y0, PROFILE_LEN = -0.21, 0.30  # its right end (y) and length, end caps outside that
PROFILE_Y1 = PROFILE_Y0 + PROFILE_LEN
PROFILE_W = 0.045  # 45 x 45 mm
SLOT_OPEN = 0.016  # m: the opening between the lips
SLOT_IN = 0.030  # m: the channel's width under the lips
LIP_T = 0.005  # m: the lips' thickness
CHANNEL_H = 0.009  # m: the channel's height under the lips
PROFILE_TOP = MAT_TOP + PROFILE_W
LIP_BOTTOM = PROFILE_TOP - LIP_T
FLOOR = LIP_BOTTOM - CHANNEL_H  # the channel's floor
CAP_T = 0.004  # m: the end caps' thickness
_py = PROFILE_Y0 + PROFILE_LEN / 2
_lip_w = PROFILE_W / 2 - SLOT_OPEN / 2
_wall_w = PROFILE_W / 2 - SLOT_IN / 2
ALU = (0.72, 0.74, 0.76)
BLACK = (0.05, 0.05, 0.05)
PROFILE = Box("profile", pos=(PROFILE_X, _py, (MAT_TOP + FLOOR) / 2), size=(PROFILE_W, PROFILE_LEN, FLOOR - MAT_TOP),
              color=ALU)  # the solid core under the channel: it carries the profile's mesh
PROFILE_PARTS = [
    Box(f"wall_{side}", pos=(PROFILE_X + s * (SLOT_IN / 2 + _wall_w / 2), _py, (FLOOR + LIP_BOTTOM) / 2),
        size=(_wall_w, PROFILE_LEN, CHANNEL_H), color=ALU)
    for side, s in (("near", -1), ("far", 1))
] + [
    Box(f"lip_{side}", pos=(PROFILE_X + s * (SLOT_OPEN / 2 + _lip_w / 2), _py, (LIP_BOTTOM + PROFILE_TOP) / 2),
        size=(_lip_w, PROFILE_LEN, LIP_T), color=ALU)
    for side, s in (("near", -1), ("far", 1))
] + [
    Box(f"cap_{side}", pos=(PROFILE_X, y, MAT_TOP + PROFILE_W / 2), size=(PROFILE_W, CAP_T, PROFILE_W), color=BLACK)
    for side, y in (("right", PROFILE_Y0 - CAP_T / 2), ("left", PROFILE_Y1 + CAP_T / 2))
]
MARK_Y = -0.12  # the red tape mark across the profile's top
MARK_W = 0.006  # m: the tape's width (drawn only)

# -- the T-nut: a drop-in nut with its stud, one rigid body -----------------------------------------------------------
NUT_SIZE = (0.024, 0.010, 0.006)  # long (its own x), wide, thick
STUD_W, STUD_H = 0.008, 0.040  # the stud: 8 mm across, 40 mm above the nut
TNUT_SIZE = (NUT_SIZE[0], NUT_SIZE[1], NUT_SIZE[2] + STUD_H)  # the whole part's box
NUT_BELOW = TNUT_SIZE[2] / 2 - NUT_SIZE[2] / 2  # m from the part's centre down to the nut's centre
TNUT_MASS = 0.025
TNUT_XY = (0.21, -0.14)
YELLOW_ZINC = (0.86, 0.66, 0.16)
TNUT = compound(Compound(
    "tnut",
    pos=(*TNUT_XY, MAT_TOP + TNUT_SIZE[2] / 2),
    size=TNUT_SIZE,
    mass=TNUT_MASS,
    yaw=0.0,  # crosswise to the slot: it must be turned to go in
    color=YELLOW_ZINC,
    parts=(
        ("nut", (0.0, 0.0, -NUT_BELOW), NUT_SIZE),
        ("stud", (0.0, 0.0, NUT_SIZE[2] / 2), (STUD_W, STUD_W, STUD_H)),
    ),
    contact=0.002,
))

# -- the bench's other things: SimReady props, fixed, on the robot's left ---------------------------------------------
SQUARE = SimReady("square", pos=(0.25, 0.20, TOP), size=(0.13, 0.17, 0.005), yaw=math.pi / 2,
                  package="measurement_ruler_speedsquare_a02", usd="sm_measurement_ruler_speedsquare_a02_01.usd",
                  scale=0.6)
MARKER = SimReady("marker", pos=(0.20, 0.06, TOP), size=(0.12, 0.013, 0.013), yaw=0.35,
                  package="stationery_marker_alcohol_prussianBlue_a01",
                  usd="sm_stationery_marker_alcohol_prussianBlue_a01_01.usd")

OBJECTS = [MAT, PROFILE, *PROFILE_PARTS, TNUT, SQUARE, MARKER]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged. The profile's
# mesh also draws its walls, lips, end caps and the red mark.
LOOKS = {"mat": "mat.usdc", "profile": "profile.usdc", "tnut": "tnut.usdc"}
dress(LOOKS, hide=[b.name for b in PROFILE_PARTS])

# -- scoring ---------------------------------------------------------------------------------------------------------
UPRIGHT_DEG = 20.0  # the stud within this of vertical
CROSS_DEG = 10.0  # turned: the nut's long side within this of perpendicular to the slot
AT_MARK = 0.01  # m: the nut's centre this close to the mark, along the slot
TOUCH = 0.003  # m: a finger this close to the T-nut's box still holds it
OFF_TABLE_Z = -0.02  # m above the table: lower, the T-nut has fallen off the table

# What the scorer knows besides the objects: whether a finger touched the T-nut at the last control step.
STATE = {"held": False}


@on_reset
def reset_memory(env=None) -> None:
    STATE["held"] = False


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _finite(objects) -> bool:
    s = objects["tnut"]
    return bool(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.quat_xyzw)) and np.all(np.isfinite(s.lin_vel)))


def nut_centre(objects) -> np.ndarray:
    """The nut's own centre (under the stud)."""
    s = objects["tnut"]
    return np.asarray(s.pos) + _rot(s.quat_xyzw) @ np.array([0.0, 0.0, -NUT_BELOW])


def tilt_deg(objects) -> float:
    """How far the stud leans from vertical."""
    return float(np.degrees(np.arccos(np.clip(_rot(objects["tnut"].quat_xyzw)[2, 2], -1.0, 1.0))))


def cross_error_deg(objects) -> float:
    """How far the nut's long side is from perpendicular to the slot (0: crosswise, 90: lengthwise)."""
    r = _rot(objects["tnut"].quat_xyzw)
    a = abs(math.degrees(math.atan2(r[1, 0], r[0, 0])))  # the long side's heading from x, 0..180
    return float(min(a, 180.0 - a))


def in_slot(objects) -> bool:
    """The nut is in the channel: the stud upright in the opening, the nut's centre under the lips and between the
    end caps."""
    c = nut_centre(objects)
    return bool(
        tilt_deg(objects) <= UPRIGHT_DEG
        and abs(c[0] - PROFILE_X) < SLOT_OPEN / 2
        and FLOOR - 0.002 < c[2] < LIP_BOTTOM
        and PROFILE_Y0 < c[1] < PROFILE_Y1
    )


def locked(objects) -> bool:
    """In the channel and turned crosswise, so its ends are under the lips."""
    return in_slot(objects) and cross_error_deg(objects) <= CROSS_DEG


def at_mark(objects) -> bool:
    return bool(abs(nut_centre(objects)[1] - MARK_Y) < AT_MARK)


def touching(points, objects) -> bool:
    """Whether any of the finger ``points`` (n, 3) is within 3 mm of the T-nut's box."""
    if points is None or len(points) == 0:
        return False
    s = objects["tnut"]
    local = (np.asarray(points) - np.asarray(s.pos)) @ _rot(s.quat_xyzw)
    return bool(np.any(np.all(np.abs(local) <= np.asarray(TNUT_SIZE) / 2 + TOUCH, axis=1)))


@on_step
def watch_fingers(env) -> None:
    """Every control step: does a finger still touch the T-nut?"""
    from fingers import finger_points
    from yam_arena.scene import object_states

    objects = object_states(env, ["tnut"])
    STATE["held"] = _finite(objects) and touching(finger_points(env), objects)


def _violation(objects) -> bool:
    return not _finite(objects) or bool(objects["tnut"].pos[2] < TOP + OFF_TABLE_Z)


def success(objects) -> bool:
    """Locked at the mark, let go, and still."""
    if _violation(objects):
        return False
    return bool(locked(objects) and at_mark(objects) and not STATE["held"] and objects["tnut"].resting())


def score(objects) -> float:
    """1.0: success. 0.5: locked in the channel, away from the mark or still held. 0.25: in the channel, not turned.
    0 otherwise, and 0 if the T-nut fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if locked(objects):
        return 0.5
    return 0.25 if in_slot(objects) else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# The right arm, straight down, closes on the top of the stud with the fingers along y (yaw 0), across the nut's
# 10 mm width. It lifts the T-nut, turns it a quarter turn on the way so the nut lies along the slot, lowers it through
# the opening into the channel, turns it back a quarter turn (crosswise: locked), slides it along the slot to the mark,
# lets go and rises. Numbers are TCP positions.
GRIP_DEPTH = 0.014  # m: the pad tips this far below the stud's top
GRIP_Z = MAT_TOP + TNUT_SIZE[2] - GRIP_DEPTH + PAD_TIP_PAST_TCP
TRAVEL_Z = TOP + 0.13  # the nut's bottom 4 cm over the profile
HOVER_NUT = 0.001  # m: the nut's bottom over the channel's floor while it is turned and slid
SLOT_Z = FLOOR + HOVER_NUT + TNUT_SIZE[2] - GRIP_DEPTH + PAD_TIP_PAST_TCP
ENTRY_Z = SLOT_Z + (PROFILE_TOP - FLOOR) + 0.006  # the nut's bottom 6 mm over the profile, lined up to go in
DROP_Y = -0.04  # where the nut goes in
ALONG = math.pi / 2  # the grip's yaw that lays the nut along the slot
CROSS = 0.0  # and across it
TURN = 1.0  # set 0 to leave the nut lengthwise in the slot (the ablation of the quarter turn)
PULL = 0.0  # set 1 to pull the nut straight up after the turn instead of sliding it (the lips' check)


def demo():
    w = Waypoint
    nx, ny = TNUT_XY
    x = PROFILE_X
    lock_yaw = CROSS if TURN else ALONG
    if PULL:
        tail = [
            w(2.0, (x, DROP_Y, TRAVEL_Z), FINGER_CLOSED, "pull straight up", lock_yaw),
            w(3.0, (x, DROP_Y, TRAVEL_Z), FINGER_CLOSED, "hold up", lock_yaw),
        ]
    else:
        tail = [
            w(3.0, (x, MARK_Y, SLOT_Z), FINGER_CLOSED, "slide to the mark", lock_yaw),
            w(0.8, (x, MARK_Y, SLOT_Z), FINGER_OPEN, "let go", lock_yaw),
            w(1.2, (x, MARK_Y, TRAVEL_Z), FINGER_OPEN, "rise", lock_yaw),
        ]
    return {
        "right": [
            w(1.5, (nx, ny, TRAVEL_Z), FINGER_OPEN, "over the T-nut", CROSS),
            w(1.5, (nx, ny, GRIP_Z), FINGER_OPEN, "down around the stud", CROSS),
            w(0.8, (nx, ny, GRIP_Z), FINGER_CLOSED, "close", CROSS),
            w(1.2, (nx, ny, TRAVEL_Z), FINGER_CLOSED, "lift", CROSS),
            w(2.0, (x, DROP_Y, TRAVEL_Z), FINGER_CLOSED, "carry and turn along the slot", ALONG),
            w(1.2, (x, DROP_Y, ENTRY_Z), FINGER_CLOSED, "down to the opening", ALONG),
            w(0.6, (x, DROP_Y, ENTRY_Z), FINGER_CLOSED, "line up", ALONG),
            w(1.5, (x, DROP_Y, SLOT_Z), FINGER_CLOSED, "lower into the channel", ALONG),
            w(2.0, (x, DROP_Y, SLOT_Z), FINGER_CLOSED, "quarter turn", lock_yaw),
            *tail,
        ],
    }
