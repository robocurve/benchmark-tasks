"""On the station table: a fixed camera on a post at the far side, a fixed board holder (a slot between two rails)
in front of it, and a camera-calibration board (a thin double-sided checkerboard, 16 x 10 cm) standing edge-on
in a storage slot in front of the right arm. The robot must move the board into the holder so that it stands
upright and faces the camera. Setting up camera calibration is routine maintenance for every robot in a fleet.
The camera, the holder and the storage slot are fixed; the board is loose.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py \
           --instruction "stand the calibration board in its holder, facing the camera" \
           --scorer task_score ...
"""

import math

import numpy as np

from yam_arena.demo import pick_place_waypoints
from yam_arena.task_file import TABLE_TOP, Box, Cylinder

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# Positions on the table from the midpoint between the arm bases: x forward, y to the left (m).
DARK, STEEL = (0.15, 0.15, 0.17), (0.55, 0.57, 0.6)

# -- the camera (fixed), looking back toward the arms -------------------------------------------------------------
CAMERA_XY = (0.55, 0.0)
CAMERA_POST = Cylinder("camera_post", pos=(*CAMERA_XY, TOP + 0.11), radius=0.015, height=0.22, color=STEEL)
CAMERA = Box("camera", pos=(*CAMERA_XY, TOP + 0.24), size=(0.06, 0.09, 0.04), color=DARK)

# -- the board holder (fixed): a 2 cm slot running left to right, so a board in it faces the camera -----------------
HOLDER_XY = (0.32, 0.0)
SLOT = 0.02  # gap between the rails
RAIL_SIZE = (0.01, 0.12, 0.03)
_rail_dx = SLOT / 2 + RAIL_SIZE[0] / 2
HOLDER = [
    Box("holder_front", pos=(HOLDER_XY[0] - _rail_dx, HOLDER_XY[1], TOP + RAIL_SIZE[2] / 2), size=RAIL_SIZE, color=DARK),
    Box("holder_back", pos=(HOLDER_XY[0] + _rail_dx, HOLDER_XY[1], TOP + RAIL_SIZE[2] / 2), size=RAIL_SIZE, color=DARK),
]

# -- the storage slot (fixed): runs front to back, so the board starts edge-on to the camera ------------------------
STORE_XY = (0.18, -0.22)
_store_rail = (RAIL_SIZE[1], RAIL_SIZE[0], RAIL_SIZE[2])
STORE = [
    Box("store_left", pos=(STORE_XY[0], STORE_XY[1] + _rail_dx, TOP + RAIL_SIZE[2] / 2), size=_store_rail, color=STEEL),
    Box("store_right", pos=(STORE_XY[0], STORE_XY[1] - _rail_dx, TOP + RAIL_SIZE[2] / 2), size=_store_rail, color=STEEL),
]

# -- the calibration board (loose) ---------------------------------------------------------------------------------
BOARD_W, BOARD_T, BOARD_H = 0.16, 0.008, 0.10  # the arms reach straight down only ~20 cm up
BOARD = Box("board", pos=(*STORE_XY, TOP + BOARD_H / 2), size=(BOARD_W, BOARD_T, BOARD_H), mass=0.10,
            color=(0.92, 0.92, 0.9))

OBJECTS = [CAMERA_POST, CAMERA, *HOLDER, *STORE, BOARD]

# -- stillness ----------------------------------------------------------------------------------------------------
_STEP = {"objects": None, "prev": {}, "cur": {}}


def still(objects, name) -> bool:
    """At rest: yam_arena's resting() (under 1 cm/s and 0.5 rad/s), or not moved at all since the previous control
    step (under 0.5 mm, and under 0.01 in each quaternion component). The second test is needed because PhysX can
    put a light, settled body to sleep while it still reports the small velocity it had just before, so resting()
    would never become true for it. The hooks run once per control step on the same ``objects``, so a new
    ``objects`` dict marks a new step."""
    if objects is not _STEP["objects"]:
        _STEP["objects"], _STEP["prev"] = objects, _STEP["cur"]
        _STEP["cur"] = {n: np.r_[np.asarray(o.pos, dtype=float), np.asarray(o.quat_xyzw, dtype=float)]
                        for n, o in objects.items()}
    now, last = _STEP["cur"][name], _STEP["prev"].get(name)
    unmoved = (last is not None and float(np.linalg.norm(now[:3] - last[:3])) < 0.0005
               and float(np.abs(now[3:] - last[3:]).max()) < 0.01)
    return bool(objects[name].resting() or unmoved)


# -- scoring ------------------------------------------------------------------------------------------------------
UPRIGHT_H = 0.085  # standing: its top at least 8.5 cm above its bottom (within about 30 degrees of vertical)
FACING_MAX_X = 0.05  # facing the camera: thin front to back (it may lean back in the slot)...
FACING_MIN_Y = 0.145  # ...and wide left to right (within about 25 degrees of square-on)
NEAR = 0.10  # "near the holder": centre within 10 cm of the slot's centre


def _extent(board):
    return np.asarray(board.high) - np.asarray(board.low)


def upright(objects) -> bool:
    board = objects["board"]
    return _extent(board)[2] >= UPRIGHT_H and board.low[2] <= TOP + 0.01 and still(objects, "board")


def facing_camera(board) -> bool:
    size = _extent(board)
    return size[0] <= FACING_MAX_X and size[1] >= FACING_MIN_Y


def in_holder(board) -> bool:
    x, y = board.pos[:2]
    return abs(x - HOLDER_XY[0]) <= SLOT / 2 + 0.005 and abs(y - HOLDER_XY[1]) <= RAIL_SIZE[1] / 2


def in_store(board) -> bool:
    x, y = board.pos[:2]
    return abs(x - STORE_XY[0]) <= RAIL_SIZE[1] / 2 and abs(y - STORE_XY[1]) <= SLOT / 2 + 0.005


def outcome(objects) -> str:
    """'in_holder' (standing in the holder, facing the camera: success), 'facing_near' (standing and facing the
    camera within 10 cm of the holder, but not in it), 'standing' (out of storage and standing, but not facing the
    camera or not near the holder), or 'other' (still in storage, lying flat, dropped, held)."""
    board = objects["board"]
    if not upright(objects) or in_store(board):
        return "other"
    near = float(np.hypot(*(np.asarray(board.pos[:2]) - np.asarray(HOLDER_XY)))) <= NEAR
    if facing_camera(board) and in_holder(board):
        return "in_holder"
    if facing_camera(board) and near:
        return "facing_near"
    return "standing"


def success(objects):
    """The board stands in the holder's slot, facing the camera."""
    return outcome(objects) == "in_holder"


def distance(objects):
    """Metres (horizontal) from the board's centre to the holder's slot."""
    return float(np.hypot(*(np.asarray(objects["board"].pos[:2]) - np.asarray(HOLDER_XY))))


def score(objects):
    """Partial credit (success scores 1.0): 0.5 standing and facing the camera near the holder but not in its
    slot; 0.25 out of storage and standing anywhere else, or turned the wrong way; 0 for anything else."""
    return {"in_holder": 1.0, "facing_near": 0.5, "standing": 0.25}.get(outcome(objects), 0.0)


# -- scripted demo ------------------------------------------------------------------------------------------------
# Waypoint.yaw turns the straight-down grip about the vertical; 0 closes the fingers along world y, across the
# board's thickness while it stands in storage. A quarter turn on the way leaves it facing the camera.
PICK_YAW, PLACE_YAW = 0.0, math.pi / 2
GRIP_Z = TOP + BOARD_H - 0.015  # grip 1.5 cm below the board's top edge


def demo():
    """The right arm grips the board's top edge, lifts it 4.5 cm straight out of storage (clear of the 3 cm rails),
    turns it a quarter turn while carrying it to the holder, and lowers it into the slot, letting go 5 mm above
    where it rests."""
    pick = (*STORE_XY, GRIP_Z)
    place = (*HOLDER_XY, GRIP_Z + 0.005)
    return {"right": pick_place_waypoints(pick, place, travel_z=GRIP_Z + 0.045, yaw=PICK_YAW, place_yaw=PLACE_YAW)}
