"""On the station table, a slick dark grey rubber mat lies diagonally, from front right to back left. On it, end to
end along the mat: a bare green circuit board (6.5 x 14 cm, 3.6 mm thick with its parts, 45 g) at the front right,
and a pink antistatic bag (7.7 x 15.2 cm, 1.2 cm thick, 15 g) at the back left, its mouth facing the board 5 cm
away. The bag's cavity is 7.3 cm wide, 14.8 cm deep and 8 mm high, with a thumb notch (3 x 1.5 cm) in its top layer
at the mouth. The board's near end and the bag's closed end overhang the mat's ends by 3 cm. A grey plastic crate
(25.4 x 17 x 7.2 cm) stands on the robot's left, behind the bag.

The task: hold the bag, slide the board flat into it until it is fully inside, then lay the bag in the crate.

The bag and the board slide on the mat (friction 0.25), and the board grips the bag's inside (friction 1.0): pushing
the board in drags the bag along unless the other hand holds it. The bag is a rigid stand-in for a soft one:
``envelope.py`` builds it as one rigid body of thin walls.

Run it:  inspect-robots run --embodiment yam_arena -E task=board_into_antistatic_bag.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from envelope import Envelope, envelope, surface  # noqa: E402
from looks import dress  # noqa: E402
from simready import SimReady  # noqa: E402
from yam_arena.constants import PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the line: the board and the bag lie end to end along one line, diagonal on the table ---------------------------
# Everything is placed along the line from the board to the bag, which points back and to the robot's left (120
# degrees from x): the bag's mouth faces the board, front right, and its closed end lies back left. Along this line
# both grippers reach a grip across the board and the bag, and the right one's wrist camera points away from the left
# gripper (along y, the two cameras face each other and the grippers meet).
LINE_ANGLE = math.radians(120)
LINE = (math.cos(LINE_ANGLE), math.sin(LINE_ANGLE))  # unit vector, board -> bag
ACROSS = (-LINE[1], LINE[0])  # unit vector across the line, to its left
TURN = LINE_ANGLE - math.pi / 2  # the yaw of the boxes along the line (their own +y is along the line)
MOUTH = (0.28, -0.02)  # the bag's mouth, on the line


def on_line(s: float, t: float = 0.0) -> tuple[float, float]:
    """The table point ``s`` metres along the line from the bag's mouth (negative toward the board) and ``t`` to
    its left."""
    return (MOUTH[0] + s * LINE[0] + t * ACROSS[0], MOUTH[1] + s * LINE[1] + t * ACROSS[1])


MAT_T = 0.005  # the rubber mat's thickness
MAT_TOP = TOP + MAT_T

# -- the bag: its mouth toward the board, on the mat ----------------------------------------------------------------
BAG_SIZE = (0.077, 0.152, 0.012)  # width, length, thickness
BAG_MASS = 0.015
BAG = envelope(Envelope(
    "bag",
    pos=(*on_line(BAG_SIZE[1] / 2), MAT_TOP + BAG_SIZE[2] / 2),
    size=BAG_SIZE,
    mass=BAG_MASS,
    yaw=TURN,
    color=(0.95, 0.42, 0.66),
    wall=0.002,
    seam=0.002,
    end=0.004,
    notch=(0.03, 0.015),
))
CAVITY = BAG.cavity  # 7.3 cm wide, 14.8 cm deep, 8 mm high
BAG_FLOOR = MAT_TOP + BAG.wall  # the cavity's floor, as the bag lies at the start

# -- the board: on the line, before the bag's mouth ------------------------------------------------------------------
BOARD_SIZE = (0.065, 0.14, 0.0036)  # width, length, thickness with its parts (1.6 mm board, 2 mm parts)
BOARD_MASS = 0.045
BOARD_GAP = 0.05  # m from the board's far end to the bag's mouth
BOARD_S = -BOARD_GAP - BOARD_SIZE[1] / 2  # the board's centre along the line
BOARD = Box("board", pos=(*on_line(BOARD_S), MAT_TOP + BOARD_SIZE[2] / 2), size=BOARD_SIZE, mass=BOARD_MASS,
            yaw=TURN, color=(0.08, 0.36, 0.17))

# -- the mat: the board's near end and the bag's closed end overhang its two ends, so fingers reach under them ----
OVERHANG = 0.03  # m
MAT_S0 = BOARD_S - BOARD_SIZE[1] / 2 + OVERHANG
MAT_S1 = BAG_SIZE[1] - OVERHANG
MAT_SIZE = (0.24, MAT_S1 - MAT_S0, MAT_T)
MAT = Box("mat", pos=(*on_line((MAT_S0 + MAT_S1) / 2), TOP + MAT_T / 2), size=MAT_SIZE, yaw=TURN,
          color=(0.13, 0.14, 0.16))
MAT_FRICTION = 0.25  # slick: the bag and the board slide on it, while they grip each other and the fingers at 1.0
surface("mat", MAT_FRICTION, "min")

# -- the crate: a SimReady plastic crate, fixed, on the left behind the bag -----------------------------------------
CRATE_XY = (0.30, 0.28)
CRATE_SCALE = 0.42  # the 60.5 x 40.5 x 17 cm model to 25.4 x 17 x 7.2 cm
CRATE = SimReady("crate", pos=(*CRATE_XY, TOP), size=(0.17, 0.254, 0.0715), yaw=math.pi / 2,
                 package="crate_gray_a08", usd="sm_crate_gray_a08_01.usd", scale=CRATE_SCALE)
CRATE_RIM = TOP + 0.0715
CRATE_INNER = (0.16, 0.244)  # m: the inside's x and y extents (5 mm walls, measured on the model)
CRATE_FLOOR = TOP + 0.0025

OBJECTS = [MAT, BAG, BOARD, CRATE]

# What the boxes look like: meshes modelled in Blender (looks/make_looks.py), the colliders unchanged.
LOOKS = {"bag": "bag.usdc", "board": "board.usdc", "mat": "mat.usdc"}
dress(LOOKS)

# -- scoring ---------------------------------------------------------------------------------------------------------
TOL = 0.002  # m of slack on the cavity's walls (contacts let shapes overlap a little)
NEAR_END = 0.01  # m: the board's far edge within this of the bag's closed end
IN_CRATE_XY = (CRATE_INNER[0] / 2 - 0.01, CRATE_INNER[1] / 2 - 0.01)  # the bag's centre this close to the crate's
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _finite(objects) -> bool:
    return all(np.all(np.isfinite(objects[n].pos)) and np.all(np.isfinite(objects[n].quat_xyzw))
               for n in ("bag", "board"))


def board_in_bag_frame(objects) -> tuple[np.ndarray, np.ndarray]:
    """The board's centre and its four corners in the bag's own frame: x across, y from the bag's centre toward
    its closed end (the mouth is at y = -length/2), z up through the bag."""
    bag, board = objects["bag"], objects["board"]
    to_bag = _rot(bag.quat_xyzw).T
    centre = to_bag @ (board.pos - bag.pos)
    rb = _rot(board.quat_xyzw)
    hx, hy = BOARD_SIZE[0] / 2, BOARD_SIZE[1] / 2
    corners = np.array([to_bag @ (board.pos + rb @ np.array([sx * hx, sy * hy, 0.0]) - bag.pos)
                        for sx in (-1, 1) for sy in (-1, 1)])
    return centre, corners


def _in_cavity_xz(p) -> bool:
    return bool(abs(p[0]) <= CAVITY[0] / 2 + TOL and abs(p[2]) <= CAVITY[2] / 2 + TOL)


def depth_in(objects) -> float:
    """How far the board's far edge is past the mouth (m), in the bag's frame."""
    _, corners = board_in_bag_frame(objects)
    return float(corners[:, 1].max() + BAG_SIZE[1] / 2)


def halfway_in(objects) -> bool:
    """The board's centre is inside the cavity: past the mouth, between the seams, between the walls."""
    c, _ = board_in_bag_frame(objects)
    mouth, end = -BAG_SIZE[1] / 2, -BAG_SIZE[1] / 2 + CAVITY[1]
    return bool(_in_cavity_xz(c) and mouth <= c[1] <= end + TOL)


def fully_in(objects) -> bool:
    """The whole board is inside the cavity: its centre between the walls and every corner between the seams,
    past the mouth and short of the closed end."""
    c, corners = board_in_bag_frame(objects)
    mouth, end = -BAG_SIZE[1] / 2, -BAG_SIZE[1] / 2 + CAVITY[1]
    return bool(
        _in_cavity_xz(c)
        and all(abs(p[0]) <= CAVITY[0] / 2 + TOL for p in corners)
        and corners[:, 1].min() >= mouth
        and corners[:, 1].max() <= end + TOL
    )


def far_edge_at_end(objects) -> bool:
    """The board's far edge is within 1 cm of the closed end."""
    return bool(depth_in(objects) >= CAVITY[1] - NEAR_END)


def bag_in_crate(objects) -> bool:
    """The bag's centre is over the crate's floor (within 7 x 11.2 cm of its centre) and below its rim."""
    c = objects["bag"].pos
    return bool(
        abs(c[0] - CRATE_XY[0]) <= IN_CRATE_XY[0]
        and abs(c[1] - CRATE_XY[1]) <= IN_CRATE_XY[1]
        and c[2] < CRATE_RIM
    )


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[n].pos[2] < TOP + OFF_TABLE_Z for n in ("bag", "board"))


def success(objects) -> bool:
    """The board is fully in the bag, its far edge within 1 cm of the closed end, and the bag lies still in the
    crate."""
    if _violation(objects):
        return False
    return bool(fully_in(objects) and far_edge_at_end(objects) and bag_in_crate(objects)
                and objects["bag"].resting())


def score(objects) -> float:
    """1.0: success. 0.5: the board fully in the bag (its far edge within 1 cm of the closed end), the bag anywhere
    else. 0.25: the board's centre inside the bag. 0 otherwise, and 0 if the board or the bag fell off the table."""
    if _violation(objects):
        return 0.0
    if success(objects):
        return 1.0
    if fully_in(objects) and far_edge_at_end(objects):
        return 0.5
    return 0.25 if halfway_in(objects) else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# All straight-down grips, the fingers closing across the line. Left arm: closes across the bag's width at its closed
# end and holds it there. Right arm: closes across the board's width at its near end (pad tips 3 mm below it, past
# the mat's end), lifts it so it rides in the middle of the cavity, slides it into the mouth until the pads are 3 mm
# short of it, lets go, closes its fingers behind the board and pushes it the last 3.5 cm through the thumb notch. Then
# the left arm lifts the bag, turns it along the crate and lays it in. Numbers are TCP positions, ``on_line`` points.
GRIP_YAW = LINE_ANGLE - math.pi  # -60 degrees: fingers across the line, the right wrist camera away from the left
PLACE_YAW = -math.pi / 2  # the left arm turns the bag along the crate (along y) on the way
PAD_HALF = 0.0132  # m: half a finger pad's width along the line
PAD_TIP_HALF = 0.0077  # m: the same in the pad's lowest 5 mm, which is rounded off
HOVER_Z = TOP + 0.10
# The fingers are sent a few mm past each side of what they hold, not fully closed: their drive (5000 N/m) then
# squeezes with 20 N (board) and 12.5 N (bag). Closed with the full 50 N, the 45 g board and the 15 g bag shoot out.
BOARD_GRIP = -(BOARD_SIZE[0] / 2 - 0.004)  # finger joint target (m)
BAG_GRIP = -(BAG_SIZE[0] / 2 - 0.0025)

GRIP_BACK = 0.012  # m: the right arm's pads close this far from the board's near end
PICK_S = BOARD_S - BOARD_SIZE[1] / 2 + GRIP_BACK
PICK_Z = MAT_TOP - 0.003 + PAD_TIP_PAST_TCP  # pad tips 3 mm below the board, past the mat's end
LIFT = BAG_FLOOR + (CAVITY[2] - BOARD_SIZE[2]) / 2 - MAT_TOP  # m: the board's bottom up to mid-cavity (4.2 mm)
DROOP = 0.0015  # m higher still, in case the board, held by its near end, droops at its far end
INSERT_Z = PICK_Z + LIFT + DROOP
INSERT_S = -PAD_HALF - 0.003  # the pads 3 mm short of the mouth
BOARD_OUT = INSERT_S - GRIP_BACK  # where the board's near end is then (2.8 cm out of the mouth)
PUSH_FROM_S = BOARD_OUT - PAD_TIP_HALF - 0.008
PUSH_Z = BAG_FLOOR + 0.001 + PAD_TIP_PAST_TCP  # closed pad tips 1 mm above the cavity's floor
PUSH_PAST = 0.007  # m: the board's near end is pushed this far past the mouth
PUSH_S = PUSH_PAST - PAD_TIP_HALF
AWAY = (0.30, -0.20, TOP + 0.12)  # where the right arm waits, out of the left arm's way

HOLD_BACK = 0.012  # m: the left arm's pads close this far from the bag's closed end
HOLD_S = BAG_SIZE[1] - HOLD_BACK
HOLD_Z = MAT_TOP - 0.003 + PAD_TIP_PAST_TCP  # pad tips 3 mm below the bag, past the mat's end
CARRY_Z = TOP + 0.13
BAG_BELOW_TCP = HOLD_Z - MAT_TOP  # the bag's bottom below the TCP while it is held
# Over the crate the bag lies along y, its closed end (held) toward +y: its centre is then over the crate's.
PLACE_XY = (CRATE_XY[0], CRATE_XY[1] + (HOLD_S - BAG_SIZE[1] / 2))
PLACE_Z = CRATE_FLOOR + 0.02 + BAG_BELOW_TCP  # the bag's bottom 2 cm above the crate's floor
RIGHT_DONE = 14.5  # s: the right arm has pushed the board in and left by then


def demo():
    w = Waypoint
    yaw = GRIP_YAW

    def at(s, z):
        return (*on_line(s), z)

    right = [
        w(1.5, at(PICK_S, HOVER_Z), FINGER_OPEN, "over the board", yaw),
        w(1.5, at(PICK_S, PICK_Z), FINGER_OPEN, "down past the mat", yaw),
        w(0.8, at(PICK_S, PICK_Z), BOARD_GRIP, "close on the board", yaw),
        w(0.6, at(PICK_S, INSERT_Z), BOARD_GRIP, "lift to the mouth", yaw),
        w(3.5, at(INSERT_S, INSERT_Z), BOARD_GRIP, "slide in", yaw),
        w(0.6, at(INSERT_S, INSERT_Z), FINGER_OPEN, "let go", yaw),
        w(0.6, at(INSERT_S, INSERT_Z + 0.015), FINGER_OPEN, "up", yaw),
        w(0.6, at(PUSH_FROM_S, INSERT_Z + 0.015), FINGER_OPEN, "back", yaw),
        w(0.5, at(PUSH_FROM_S, INSERT_Z + 0.015), FINGER_CLOSED, "close", yaw),
        w(0.8, at(PUSH_FROM_S, PUSH_Z), FINGER_CLOSED, "down behind the board", yaw),
        w(1.5, at(PUSH_S, PUSH_Z), FINGER_CLOSED, "push it in", yaw),
        w(0.6, at(PUSH_S, TOP + 0.06), FINGER_CLOSED, "up", yaw),
        w(1.2, AWAY, FINGER_CLOSED, "away", yaw),
    ]
    held = 1.5 + 1.5 + 0.8
    left = [
        w(1.5, at(HOLD_S, HOVER_Z), FINGER_OPEN, "over the bag's end", yaw),
        w(1.5, at(HOLD_S, HOLD_Z), FINGER_OPEN, "down past the mat", yaw),
        w(0.8, at(HOLD_S, HOLD_Z), BAG_GRIP, "hold the bag", yaw),
        w(RIGHT_DONE - held, at(HOLD_S, HOLD_Z), BAG_GRIP, "keep holding", yaw),
        w(1.5, at(HOLD_S, CARRY_Z), BAG_GRIP, "lift", yaw),
        w(2.0, (*PLACE_XY, CARRY_Z), BAG_GRIP, "over the crate", PLACE_YAW),
        w(1.2, (*PLACE_XY, PLACE_Z), BAG_GRIP, "lower", PLACE_YAW),
        w(0.6, (*PLACE_XY, PLACE_Z), FINGER_OPEN, "let go", PLACE_YAW),
        w(1.0, (*PLACE_XY, CARRY_Z), FINGER_OPEN, "up", PLACE_YAW),
    ]
    return {"right": right, "left": left}
