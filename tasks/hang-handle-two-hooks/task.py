"""On the station table: a wall rack with two J-hooks, and a long square wooden handle lying flat on the table.

The rack stands at the far side of the table in front of the right arm. Its back board is an upright light grey
panel, 380 mm wide (left to right, y), 18 mm thick and 145 mm tall, standing on the table with its front face
34.6 cm out from the arm bases (x) and centred 20 cm right of the midline. Two dark grey J-hooks stick out of the
board toward the arms, 200 mm apart (centred 10 cm and 30 cm right of the midline). Each hook is a flat arm 20 mm
wide whose top is 80 mm above the table, running 87 mm out from the board, and ending in an upturned lip 10 mm thick
whose top is 15 mm above the arm. The open slot between each lip and the board is 87 mm wide.

The handle is a square orange bar, 320 x 22 x 22 mm, about 100 g, lying flat on the table with its long side
running away from the arms (x). Its centre is 24 cm out from the arm bases and 4 cm left of the midline. It is
loose; the rack and hooks are fixed.

A handle is hung when it lies across both hooks at once, in both slots between lip and board, resting on both
hook arms. "Left" and "right" are as the arms face the table: left is toward the left arm.
"""

import math

import numpy as np

from yam_arena.constants import FINGER_CLOSED
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- the handle: a square hardwood bar, 320 x 22 x 22 mm, lying along x ----------------------------------------------

BAR_LEN = 0.320
BAR_W = 0.022  # square cross-section
BAR_SIZE = (BAR_LEN, BAR_W, BAR_W)  # yaw 0 puts the long side along world x
BAR_MASS = 0.100  # kg: 1.55e-4 m^3 of hardwood at about 650 kg/m^3
BAR_START = (0.24, 0.04)  # centre: clear of the rack, and near enough for joint4 at the carry height
SPAWN_GAP = 0.001  # spawns this far above the table and settles onto it

# -- the rack: a back board and two J-hooks, all fixed boxes ------------------------------------------------------

RACK_Y = -0.20  # rack centre, in front of the right arm
HOOK_DY = 0.10  # hooks 200 mm apart, centre to centre
HOOK_YS = (RACK_Y + HOOK_DY, RACK_Y - HOOK_DY)  # (-0.10, -0.30)
HOOK_W = 0.020  # each hook's width along y
GOAL_X = 0.30  # the handle's centre-line when hung in the middle of both slots
LIP_GAP = 0.030  # bar face to lip, hung in the middle of the slot (room for a 15 mm grip offset and 5 deg skew)
WALL_GAP = 0.035  # bar face to board: also clears the fingers (pads 21 mm, upper fingers 35 mm out from the TCP)
LIP_IN_X = GOAL_X - BAR_W / 2 - LIP_GAP  # 0.259: the lip's inner face
WALL_X = GOAL_X + BAR_W / 2 + WALL_GAP  # 0.346: the board's front face
SLOT = WALL_X - LIP_IN_X  # 87 mm
LIP_T = 0.010
HOOK_T = 0.010  # hook arm thickness
ARM_H = 0.080  # hook arm top above the table
LIP_H = 0.015  # lip top above the hook arm top
ARM_TOP = TOP + ARM_H
BOARD_T = 0.018
BOARD_W = 0.380
BOARD_H = ARM_H + 0.065  # 145 mm: 50 mm above the lips, low enough for the gripper body to pass over it
RACK_GREY = (0.78, 0.78, 0.76)
HOOK_GREY = (0.22, 0.22, 0.24)

BAR = Box(
    "handle",
    pos=(BAR_START[0], BAR_START[1], TOP + BAR_W / 2 + SPAWN_GAP),
    size=BAR_SIZE,
    mass=BAR_MASS,
    color=(0.90, 0.45, 0.08),
)
BOARD = Box(
    "back_board",
    pos=(WALL_X + BOARD_T / 2, RACK_Y, TOP + BOARD_H / 2),
    size=(BOARD_T, BOARD_W, BOARD_H),
    color=RACK_GREY,
)


def _hook(tag: str, y: float) -> list[Box]:
    arm = Box(
        f"hook_{tag}_arm",
        pos=((LIP_IN_X + WALL_X) / 2, y, ARM_TOP - HOOK_T / 2),
        size=(SLOT, HOOK_W, HOOK_T),
        color=HOOK_GREY,
    )
    lip = Box(
        f"hook_{tag}_lip",
        pos=(LIP_IN_X - LIP_T / 2, y, ARM_TOP + (LIP_H - HOOK_T) / 2),
        size=(LIP_T, HOOK_W, HOOK_T + LIP_H),
        color=HOOK_GREY,
    )
    return [arm, lip]


HOOKS = {"inner": HOOK_YS[0], "outer": HOOK_YS[1]}  # inner: nearer the midline
RACK = [BOARD] + [b for tag, y in HOOKS.items() for b in _hook(tag, y)]
OBJECTS = [BAR] + RACK

# -- scoring -------------------------------------------------------------------------------------------------------

SEAT_TOL = 0.005  # underside within 5 mm of the hook arm top
OVERHANG = 0.030  # the end beyond a hook reaches at least 30 mm past the hook's outer edge
FULL, ONE_HOOK = 1.0, 0.3


def _rotation(state) -> np.ndarray:
    x, y, z, w = (float(v) for v in state.quat_xyzw)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def long_axis(state) -> np.ndarray:
    """The handle's long axis (its body x) in the world frame, pointing toward +y (or +x if level with x)."""
    a = _rotation(state)[:, 0]
    return -a if (a[1], a[0]) < (0.0, 0.0) else a


def at_hook(state, hook_y: float):
    """Where the handle's centre-line crosses the vertical plane through a hook (y = hook_y), and the height of
    its underside there: (point, underside z), or None if the centre-line does not reach that plane within the
    handle's length, or the handle points more than 60 deg away from y (it does not lie across the rack)."""
    r = _rotation(state)
    a = r[:, 0]
    if abs(a[1]) < 0.5:  # long axis more than 60 deg from y: it does not lie across the hooks
        return None
    t = (hook_y - float(state.pos[1])) / a[1]
    if abs(t) > BAR_LEN / 2:
        return None
    point = np.asarray(state.pos, dtype=float) + t * a
    underside = float(point[2]) - BAR_W / 2 * (abs(r[2, 1]) + abs(r[2, 2]))
    return point, underside


def end_past_hook(state, hook_y: float) -> float:
    """How far the handle's end on the far side of this hook (away from the rack centre) reaches past the hook's
    outer edge, along y (m); negative if it stops short."""
    out = 1.0 if hook_y > RACK_Y else -1.0
    a = long_axis(state)
    end_y = float(state.pos[1]) + out * abs(a[1]) * BAR_LEN / 2
    return out * (end_y - hook_y) - HOOK_W / 2


def seated(state, hook_y: float) -> bool:
    """Resting in this hook: where the centre-line crosses the hook, it lies between the lip's inner face and the
    board's face, the underside is within 5 mm of the hook arm's top, and the end beyond this hook reaches at least
    30 mm past the hook's outer edge."""
    hit = at_hook(state, hook_y)
    if hit is None:
        return False
    point, underside = hit
    in_slot = LIP_IN_X < float(point[0]) < WALL_X
    on_arm = abs(underside - ARM_TOP) <= SEAT_TOL
    return bool(in_slot and on_arm and end_past_hook(state, hook_y) >= OVERHANG)


def hooks_seated(objects) -> int:
    s = objects["handle"]
    return sum(seated(s, y) for y in HOOK_YS)


def score(objects):
    """1.0 resting in both hooks, 0.3 in one hook only, 0 otherwise; the handle must be still."""
    s = objects["handle"]
    if not s.resting():
        return 0.0
    n = hooks_seated(objects)
    return FULL if n == 2 else ONE_HOOK if n == 1 else 0.0


def success(objects):
    """Hung in both hooks and still; must hold for 1 s."""
    return bool(score(objects) == 1.0)


GOAL_POS = np.array([GOAL_X, RACK_Y, ARM_TOP + BAR_W / 2])


def distance(objects):
    """Metres: the handle's centre to its hung position (middle of both slots, on the hook arms), plus 0.1 m times
    how far its long axis is from lying along y (0 along y, 0.1 across), plus 0.05 m for each hook it is not
    resting in."""
    s = objects["handle"]
    d = float(np.linalg.norm(np.asarray(s.pos, dtype=float) - GOAL_POS))
    d += 0.1 * (1.0 - abs(float(_rotation(s)[1, 0])))
    d += 0.05 * (2 - hooks_seated(objects))
    return d


# -- demo ------------------------------------------------------------------------------------------------------------
# One arm (the right, which stands in front of the rack) grips the handle at its middle, lifts it, turns it a
# quarter turn anticlockwise seen from above while it is still over the start (the turning ends sweep away from
# the rack), carries it to the rack above the lips, lowers it slowly into both slots at once, lets go 3 mm above
# the hook arms, rises 3 cm, backs away and parks clear of the top camera's view of the rack. The left arm stays in
# its ready pose. Why one arm: a 100 g bar held at its middle balances, the grip sits between the hooks where
# nothing is under or beside the fingers but the board, and the two-arm version would have to swing two wrists
# round each other through the quarter turn.

GRIP_BELOW = 0.017  # TCP this far above the handle's underside: pad tips 4 mm above it, pads hold 18 of 22 mm
GRIP_Z = TOP + GRIP_BELOW
RELEASE_GAP = 0.003  # let go with the underside this far above the hook arms
RELEASE_Z = ARM_TOP + RELEASE_GAP + GRIP_BELOW
CARRY_CLEAR = 0.015  # carried with the underside this far above the lip tops
CARRY_Z = ARM_TOP + LIP_H + CARRY_CLEAR + GRIP_BELOW  # TCP 12.7 cm above the table (joint4 limit ~14 cm)
HOVER_Z = TOP + 0.100
CLEAR_RISE = 0.030
APPROACH_CLEARANCE = 0.010  # each pad this far off the bar on the way down (little travel, little grip offset)
LOOSEN = 0.004  # each pad backs off this far to let go (keeps the outer finger 9 mm off the board)
PICK_YAW = 0.0  # fingers close along y, across the handle lying along x
PLACE_YAW = math.pi / 2  # a quarter turn anticlockwise: fingers close along x, handle along y
BACK_X = 0.20  # after letting go, back away to here before parking
PARK = (0.14, -0.42, TOP + 0.14)  # parked low and off to the right, clear of the rack in the top camera
PARK_YAW = 0.0
FINAL_HOLD_S = 2.0
DEMO_ARM = "right"


def _finger(clearance: float) -> float:
    """Finger joint target leaving ``clearance`` between each pad and the 22 mm bar."""
    return -(BAR_W / 2 + clearance)


def hang_waypoints() -> list[Waypoint]:
    px, py = BAR_START
    gx, gy = GOAL_X, RACK_Y
    approach, loose = _finger(APPROACH_CLEARANCE), _finger(LOOSEN)
    rise_z = RELEASE_Z + CLEAR_RISE
    return [
        Waypoint(1.5, (px, py, HOVER_Z), approach, "to the handle", PICK_YAW),
        Waypoint(1.5, (px, py, GRIP_Z), approach, "descend", PICK_YAW),
        Waypoint(0.8, (px, py, GRIP_Z), FINGER_CLOSED, "close", PICK_YAW),
        Waypoint(1.5, (px, py, CARRY_Z), FINGER_CLOSED, "lift", PICK_YAW),
        Waypoint(2.5, (px, py, CARRY_Z), FINGER_CLOSED, "turn", PLACE_YAW),
        Waypoint(2.5, (gx, gy, CARRY_Z), FINGER_CLOSED, "carry", PLACE_YAW),
        Waypoint(2.0, (gx, gy, RELEASE_Z), FINGER_CLOSED, "lower", PLACE_YAW),
        Waypoint(0.6, (gx, gy, RELEASE_Z), loose, "release", PLACE_YAW),
        Waypoint(1.0, (gx, gy, rise_z), loose, "rise clear", PLACE_YAW),
        Waypoint(1.2, (BACK_X, gy, rise_z), loose, "back away", PLACE_YAW),
        Waypoint(1.5, PARK, loose, "park", PARK_YAW),
        Waypoint(FINAL_HOLD_S, PARK, loose, "hold still", PARK_YAW),
    ]


def demo():
    """Right arm only, 18.6 s: pick the handle at its middle, turn it a quarter turn, hang it on both hooks."""
    return {DEMO_ARM: hang_waypoints()}
