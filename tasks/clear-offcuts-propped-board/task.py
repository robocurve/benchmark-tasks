"""On the station table: a finished pine batten (36 x 4 x 2 cm) lies across the table, its left end on a
pine bearer (14 x 4 x 4 cm) and its right end on a dark plywood offcut (14 x 4 x 5 cm) that someone used as a
prop, so the board slopes slightly. The second pine bearer lies unused on the table beyond the board's right
end. A second, smaller dark plywood offcut lies loose at the front left, and an open scrap bin stands at the
front on the midline. Both offcuts are scrap, but one of them is holding the board up. A mug, a roll of tape
and a screwdriver lie behind the board as distractors.
"""

import pathlib
import sys
from itertools import product

import numpy as np

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from mechkit import fingertips, on_reset, on_step  # noqa: E402
from simready import SimReady  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

X = 0.32  # the board's centre line across the table; the bearers and the prop lie crosswise under it
BOARD_Y = 0.03  # the board's centre, a little left of the midline
BEARER_Y, PROP_Y = 0.16, -0.10  # the left bearer and the propping offcut, 26 cm apart
BOARD_SIZE = (0.04, 0.36, 0.02)
BEARER_SIZE = (0.14, 0.04, 0.04)
PROP_SIZE = (0.14, 0.04, 0.05)
LOOSE_SIZE = (0.07, 0.04, 0.03)
SPARE_Y = -0.27

PINE = (0.86, 0.72, 0.48)
PLY = (0.30, 0.20, 0.13)
VARNISH = (0.62, 0.36, 0.14)
BIN_GREY = (0.35, 0.37, 0.40)

BEARER = Box("left_bearer", pos=(X, BEARER_Y, TOP + 0.02), size=BEARER_SIZE, mass=0.11, color=PINE)
SPARE = Box("spare_bearer", pos=(X, SPARE_Y, TOP + 0.02), size=BEARER_SIZE, mass=0.11, color=PINE)
PROP = Box("prop_offcut", pos=(X, PROP_Y, TOP + 0.025), size=PROP_SIZE, mass=0.18, color=PLY)
LOOSE_POS = (0.17, 0.20)
LOOSE = Box("loose_offcut", pos=(*LOOSE_POS, TOP + 0.015), size=LOOSE_SIZE, mass=0.05, color=PLY)
# Spawned level just above the prop; its left end drops 1 cm onto the lower bearer while the scene settles.
BOARD = Box("board", pos=(X, BOARD_Y, TOP + 0.062), size=BOARD_SIZE, mass=0.18, color=VARNISH)

# The scrap bin: a fixed open tray, 18 x 16 cm inside, 3 cm walls.
BIN_X, BIN_Y, BIN_INX, BIN_IN, WALL, WALL_H = 0.14, 0.0, 0.18, 0.16, 0.01, 0.03
_o = BIN_IN / 2 + WALL / 2
_ox = BIN_INX / 2 + WALL / 2
BIN = [
    Box("bin_base", pos=(BIN_X, BIN_Y, TOP + 0.0025), size=(BIN_INX + 2 * WALL, BIN_IN + 2 * WALL, 0.005), color=BIN_GREY),
    Box("bin_front", pos=(BIN_X - _ox, BIN_Y, TOP + WALL_H / 2), size=(WALL, BIN_IN + 2 * WALL, WALL_H), color=BIN_GREY),
    Box("bin_back", pos=(BIN_X + _ox, BIN_Y, TOP + WALL_H / 2), size=(WALL, BIN_IN + 2 * WALL, WALL_H), color=BIN_GREY),
    Box("bin_left", pos=(BIN_X, BIN_Y + _o, TOP + WALL_H / 2), size=(BIN_INX, WALL, WALL_H), color=BIN_GREY),
    Box("bin_right", pos=(BIN_X, BIN_Y - _o, TOP + WALL_H / 2), size=(BIN_INX, WALL, WALL_H), color=BIN_GREY),
]
# Distractors behind the board (SimReady props), which scoring ignores. pos is each one's origin, on the table top.
MUG = SimReady(
    "mug", pos=(0.52, 0.26, TOP), size=(0.114, 0.09, 0.08), mass=0.30, yaw=np.pi,
    package="dishware_mug_a01", usd="sm_dishware_mug_a01_01.usda",
)
TAPE = SimReady(
    "tape_roll", pos=(0.50, -0.22, TOP), size=(0.10, 0.10, 0.05), mass=0.06,
    package="tape_clear_a01", usd="sm_tape_clear_a01_01.usd",
)
SCREWDRIVER = SimReady(
    "screwdriver", pos=(0.47, 0.03, TOP), size=(0.188, 0.034, 0.032), mass=0.10, yaw=np.pi / 2, convex=True,
    package="handTool_screwdriver_phillips_ph2_a01", usd="sm_handTool_screwdriver_phillips_ph2_a01_01.usd",
)
OBJECTS = [BEARER, SPARE, PROP, LOOSE, BOARD, *BIN, MUG, TAPE, SCREWDRIVER]

SUCCESS_HOLD_S = 0.0  # the task scorer counts its own continuous 3 s dwell
SUCCESS_HOLD_STEPS = 60  # 3 s at the task's 20 Hz control rate
SEAT_Z = TOP + BEARER_SIZE[2] + BOARD_SIZE[2] / 2  # the board's centre height when it lies on two bearers
CONTACT_Z = TOP + 0.001  # allow 1 mm for contact-solver tolerance when detecting bench contact
CLEAR = 0.02  # "released": no point on either gripper's fingers is closer to the board than this
_harm = False
_tips = None  # points on both grippers' fingers at the latest step
_success_hold_steps = 0
_full_awarded = False


@on_reset
def _new_trial(environment=None):
    """The harm record is cleared only by a reset of the scene, never by anything that happens in a trial."""
    global _harm, _tips, _success_hold_steps, _full_awarded
    _harm, _tips, _success_hold_steps, _full_awarded = False, None, 0, False


@on_step
def _see_grippers(environment):
    global _tips, _success_hold_steps, _full_awarded
    _tips = fingertips(environment)
    from yam_arena.scene import object_states

    objects = object_states(environment, ("board", "left_bearer", "spare_bearer", "prop_offcut", "loose_offcut"))
    _track(objects)
    full_state = not _harm and all(_in_bin(name, objects[name]) for name in ("prop_offcut", "loose_offcut")) and _board_on_bearers(objects)
    _success_hold_steps = _success_hold_steps + 1 if full_state else 0
    if _success_hold_steps >= SUCCESS_HOLD_STEPS:
        _full_awarded = True


def _rotate(quat_xyzw, v):
    x, y, z, w = quat_xyzw
    u = np.array([x, y, z])
    return v + 2 * np.cross(u, np.cross(u, v) + w * np.asarray(v, dtype=float))


def _tilt_deg(obj):
    up = _rotate(obj.quat_xyzw, np.array([0.0, 0.0, 1.0]))
    return float(np.degrees(np.arccos(np.clip(up[2], -1.0, 1.0))))


def _track(objects):
    """Remember whether any part of the board contacted the bench during the trial."""
    global _harm
    _harm = bool(_harm or objects["board"].low[2] <= CONTACT_Z)


def _released(obj, size=BOARD_SIZE, clear=CLEAR):
    """No fingertip point within `clear` of the box's surface."""
    if _tips is None:
        return False
    x, y, z, w = obj.quat_xyzw
    local = np.array([_rotate((-x, -y, -z, w), p - obj.pos) for p in _tips])
    gap = np.linalg.norm(np.maximum(np.abs(local) - np.array(size) / 2, 0.0), axis=1)
    return bool(gap.min() > clear)


OFFCUT_SIZE = {"prop_offcut": PROP_SIZE, "loose_offcut": LOOSE_SIZE}
OFFCUT_CLEAR = 0.01  # an offcut counts as lying in the bin only once no fingertip point is within 1 cm of it


def _in_bin(name, obj):
    inside = abs(obj.pos[0] - BIN_X) < BIN_INX / 2 and abs(obj.pos[1] - BIN_Y) < BIN_IN / 2
    return bool(inside and obj.pos[2] < TOP + 0.07 and obj.resting() and _released(obj, OFFCUT_SIZE[name], OFFCUT_CLEAR))


def _board_on_bearers(objects):
    """Verify the released board is physically supported by two flat, crosswise bearers."""
    board = objects["board"]
    if not (board.resting() and _released(board) and _tilt_deg(board) < 3.0 and abs(board.pos[2] - SEAT_Z) < 0.005):
        return False
    board_x = _rotate(board.quat_xyzw, np.array([1.0, 0.0, 0.0]))
    board_y = _rotate(board.quat_xyzw, np.array([0.0, 1.0, 0.0]))
    ys = []
    for name in ("left_bearer", "spare_bearer"):
        b = objects[name]
        bx = _rotate(b.quat_xyzw, np.array([1.0, 0.0, 0.0]))
        bz = _rotate(b.quat_xyzw, np.array([0.0, 0.0, 1.0]))
        flat = abs(b.pos[2] - (TOP + BEARER_SIZE[2] / 2)) < 0.005 and abs(bz[2]) >= np.cos(np.radians(3.0))
        crosswise = abs(float(np.dot(bx, board_x))) >= np.cos(np.radians(5.0))
        released = _released(b, BEARER_SIZE)

        # Project the bearer's actual oriented footprint onto the board's width and length axes.
        corners = [
            b.pos + _rotate(b.quat_xyzw, np.array([sx * BEARER_SIZE[0] / 2, sy * BEARER_SIZE[1] / 2, sz * BEARER_SIZE[2] / 2]))
            for sx, sy, sz in product((-1.0, 1.0), repeat=3)
        ]
        relative = np.asarray(corners) - board.pos
        x_projection = relative @ board_x
        y_projection = relative @ board_y
        overlap_x = max(0.0, min(BOARD_SIZE[0] / 2, float(x_projection.max())) - max(-BOARD_SIZE[0] / 2, float(x_projection.min())))
        overlap_y = max(0.0, min(BOARD_SIZE[1] / 2, float(y_projection.max())) - max(-BOARD_SIZE[1] / 2, float(y_projection.min())))
        overlaps = overlap_x >= 0.03 and overlap_y >= 0.03
        contact_gap = (board.pos[2] - BOARD_SIZE[2] / 2) - (b.pos[2] + BEARER_SIZE[2] / 2)
        contacts_board = abs(contact_gap) <= 0.002

        if not (b.resting() and flat and crosswise and released and overlaps and contacts_board):
            return False
        ys.append(float(b.pos[1]))
    return bool(max(ys) - min(ys) >= 0.15 and min(ys) < board.pos[1] < max(ys))


def score(objects):
    """Latch 0 on bench contact; award 1 only after all full-score conditions hold for 3 s."""
    _track(objects)
    if _harm:
        return 0.0
    n = sum(_in_bin(name, objects[name]) for name in ("prop_offcut", "loose_offcut"))
    if _full_awarded:
        return 1.0
    if n == 2 and _board_on_bearers(objects):
        return 0.75
    return float(0.25 * n)


def success(objects):
    return bool(_full_awarded)


TRAVEL = TOP + 0.16
BOARD_GRIP_Z = TOP + 0.055  # the sloping board's centre height at the start (measured in the simulator)
# A crosswise bearer or the prop is gripped by its far end, 3.5 cm behind the board's edge: the gripper's body
# overhangs on the far side, so from there it stays clear of the board.
END_X = X + 0.055
ASIDE_Y = -0.20  # the prop is slid here, clear of the board's right end, before it is drawn forward
DRAWN_X = 0.225  # gripper x after drawing the prop forward
DRAWN_MID = (0.17, ASIDE_Y)  # where the prop's middle then lies (measured in the simulator)


def _left_plan():
    """Bin the loose offcut, then take the board's weight: grip it across its width at its middle and raise it
    2 cm, hold while the right arm swaps the prop for the bearer, then set it down."""
    yaw = -np.pi / 2
    lx, ly = LOOSE_POS
    mid = (X, BOARD_Y)
    return [
        Waypoint(2.0, (lx, ly, TRAVEL), FINGER_OPEN, "above the loose offcut"),
        Waypoint(1.5, (lx, ly, TOP + 0.02), FINGER_OPEN, "descend"),
        Waypoint(1.0, (lx, ly, TOP + 0.02), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (lx, ly, TRAVEL), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (BIN_X, 0.04, TRAVEL), FINGER_CLOSED, "over the bin"),
        Waypoint(1.0, (BIN_X, 0.04, TOP + 0.07), FINGER_CLOSED, "lower"),
        Waypoint(0.8, (BIN_X, 0.04, TOP + 0.07), FINGER_OPEN, "drop"),
        Waypoint(1.2, (BIN_X, 0.04, TRAVEL), FINGER_OPEN, "up"),
        Waypoint(2.0, (*mid, TOP + 0.13), FINGER_OPEN, "above the board", yaw),
        Waypoint(1.5, (*mid, BOARD_GRIP_Z), FINGER_OPEN, "descend", yaw),
        Waypoint(1.0, (*mid, BOARD_GRIP_Z), FINGER_CLOSED, "grip the board", yaw),
        Waypoint(1.5, (*mid, BOARD_GRIP_Z + 0.02), FINGER_CLOSED, "take its weight", yaw),  # t = 17.0
        Waypoint(28.0, (*mid, BOARD_GRIP_Z + 0.02), FINGER_CLOSED, "hold", yaw),  # t = 45.0
        Waypoint(2.0, (*mid, SEAT_Z + 0.001), FINGER_CLOSED, "set down on the bearers", yaw),
        Waypoint(1.0, (*mid, SEAT_Z + 0.001), FINGER_OPEN, "let go", yaw),
        Waypoint(1.5, (*mid, TOP + 0.13), FINGER_OPEN, "retreat", yaw),
    ]


def _right_plan():
    """Once the left arm holds the board: slide the prop out sideways by its far end, past the board's right
    end, draw it forward, regrip it at its middle and bin it; then slide the spare bearer in by its far end to
    where the prop stood."""
    prop_z = TOP + 0.035
    bz = TOP + 0.025
    mx, my = DRAWN_MID
    return [
        Waypoint(13.0, (END_X, PROP_Y, TRAVEL), FINGER_OPEN, "above the prop's far end"),
        Waypoint(2.0, (END_X, PROP_Y, prop_z), FINGER_OPEN, "descend"),
        Waypoint(2.5, (END_X, PROP_Y, prop_z), FINGER_OPEN, "wait for the board to be held"),  # t = 17.5
        Waypoint(1.0, (END_X, PROP_Y, prop_z), FINGER_CLOSED, "grip the prop"),
        Waypoint(1.5, (END_X, ASIDE_Y, prop_z), FINGER_CLOSED, "slide it out past the board's end"),
        Waypoint(2.5, (DRAWN_X, ASIDE_Y, prop_z), FINGER_CLOSED, "draw it forward"),
        Waypoint(0.8, (DRAWN_X, ASIDE_Y, prop_z), FINGER_OPEN, "release"),
        Waypoint(1.0, (DRAWN_X, ASIDE_Y, TOP + 0.10), FINGER_OPEN, "up"),
        Waypoint(1.0, (mx, my, TOP + 0.10), FINGER_OPEN, "over its middle"),
        Waypoint(1.0, (mx, my, prop_z), FINGER_OPEN, "descend"),
        Waypoint(1.0, (mx, my, prop_z), FINGER_CLOSED, "grip"),
        Waypoint(1.5, (mx, my, TRAVEL), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (BIN_X, -0.04, TRAVEL), FINGER_CLOSED, "over the bin"),
        Waypoint(1.0, (BIN_X, -0.04, TOP + 0.09), FINGER_CLOSED, "lower"),
        Waypoint(0.8, (BIN_X, -0.04, TOP + 0.09), FINGER_OPEN, "drop"),
        Waypoint(1.0, (BIN_X, -0.04, TRAVEL), FINGER_OPEN, "up"),
        Waypoint(2.0, (END_X, SPARE_Y, TRAVEL), FINGER_OPEN, "above the spare bearer's far end"),
        Waypoint(1.5, (END_X, SPARE_Y, bz), FINGER_OPEN, "descend"),
        Waypoint(1.0, (END_X, SPARE_Y, bz), FINGER_CLOSED, "grip"),
        Waypoint(0.5, (END_X, SPARE_Y, bz + 0.006), FINGER_CLOSED, "raise it off the bench"),
        Waypoint(2.5, (END_X, PROP_Y, bz + 0.006), FINGER_CLOSED, "carry it under the board"),
        Waypoint(0.5, (END_X, PROP_Y, bz), FINGER_CLOSED, "set it down"),
        Waypoint(0.8, (END_X, PROP_Y, bz), FINGER_OPEN, "let go"),
        Waypoint(1.5, (END_X, PROP_Y, TRAVEL), FINGER_OPEN, "retreat"),  # t = 43.9
    ]


def demo():
    return {"left": _left_plan(), "right": _right_plan()}
