"""On the station table: a 90 x 40 cm pegboard stands along the back, in front of both arms, its face 48 cm in front
of the arms' bases. Twelve hand tools hang on it in one row, 15 cm above the table, from the robot's right to left:
two Phillips screwdrivers, the diagonal cutters (in front of the right arm), combination pliers, long-nose pliers,
curved long-nose pliers, three slotted screwdrivers and three Torx screwdrivers. Each pair of pliers hangs jaws up, its
handles straddling a 6 mm peg that ends in an upturned tip, so a tool has to be lifted before it can slide off. Each
screwdriver hangs blade down between two pegs. A rubber mat lies on the table in front of the right arm.

The task: take the diagonal cutters off the board and lay them on the mat, without knocking the other tools down.

The tools are NVIDIA SimReady props (CC-BY 4.0), loaded by ``simready.py``. The pliers are hinged in the files and
are made rigid here. The scripted demo tilts the grip forward with ``wrist.py``.

Run it:  inspect-robots run --embodiment yam_arena -E task=cutters_off_pegboard.py --scorer task_score ...
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from simready import SimReady  # noqa: E402
from wrist import TiltWaypoint  # noqa: E402
from yam_arena.demo import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0

# -- the pegboard: 90 x 46 cm hardboard, 12 mm thick, standing on the table ------------------------------------------
BOARD_FACE_X = 0.48  # its front face, 16 cm past the arms' ready pose
BOARD_T = 0.012
BOARD_Y = (-0.46, 0.44)  # its right and left ends (y): in front of both arms (bases at y = -0.305 and 0.305)
BOARD_H = 0.46
BOARD = Box(
    "pegboard",
    pos=(BOARD_FACE_X + BOARD_T / 2, sum(BOARD_Y) / 2, TOP + BOARD_H / 2),
    size=(BOARD_T, BOARD_Y[1] - BOARD_Y[0], BOARD_H),
    color=(0.55, 0.42, 0.28),
)

# -- pegs: 6 mm square steel, 6 cm long, each ending in a 1.5 cm upturned tip ------------------------------------------
PEG = 0.006
PEG_LEN = 0.06
PEG_Z = TOP + 0.15  # the main row's centre line (the cutters' peg)
TIP_H = 0.015  # the tip rises from the peg's underside to 9 mm above its top
PEG_TIP_X = BOARD_FACE_X - PEG_LEN  # where a peg ends
STEEL = (0.62, 0.63, 0.66)


def _peg(name: str, y: float, z: float) -> list[Box]:
    return [
        Box(f"{name}_peg", pos=(BOARD_FACE_X - PEG_LEN / 2, y, z), size=(PEG_LEN, PEG, PEG), color=STEEL),
        Box(
            f"{name}_peg_tip",
            pos=(PEG_TIP_X + PEG / 2, y, z - PEG / 2 + TIP_H / 2),
            size=(PEG, PEG, TIP_H),
            color=STEEL,
        ),
    ]


# -- the tools -------------------------------------------------------------------------------------------------------
# A SimReady hand tool's file lies flat: long axis x (jaws, blade or head toward +x), width y, thickness z from z = 0
# up. Hanging, the long axis is vertical and the thickness points away from the board:
HANG_JAWS_UP = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))  # -90 deg about y: file x -> up, file z -> toward robot
HANG_BLADE_DOWN = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))  # +90 deg about y: file x -> down, file z -> board
SPAWN_GAP = 0.002  # m between a tool's back and the board at spawn
TOOLS: list[SimReady] = []
PEGS: list[Box] = []


def _add(tool: SimReady, pegs: list[tuple[float, float]]) -> SimReady:
    """Hang ``tool`` on pegs at (y, z): the tool and its pegs join the scene."""
    TOOLS.append(tool)
    for i, (y, z) in enumerate(pegs):
        PEGS.extend(_peg(f"{tool.name}_{i}", y, z))
    return tool


HANG_DAMPING = 2.0  # 1/s: angular damping for the tools, standing in for friction at the pegs (see README.md)


def _sim(name, package, pos, mass, quat, usd=None) -> SimReady:
    return SimReady(name, pos=pos, size=(0.03, 0.07, 0.25), mass=mass, package=package,
                    usd=usd or f"sm_{package}_01.usd", quat_xyzw=quat, angular_damping=HANG_DAMPING)


def pliers(name: str, package: str, y: float, mass: float, peg_z: float = PEG_Z, spawn_x: float = -0.02):
    """Jaws up, one peg between the handles. It spawns with the peg ``spawn_x`` from the file's origin (toward the
    handles, where they are well apart) and settles down onto the peg where the handles close in."""
    pos = (BOARD_FACE_X - SPAWN_GAP, y, peg_z - spawn_x)
    return _add(_sim(name, package, pos, mass, HANG_JAWS_UP), [(y, peg_z)])


SCREW_PEG_DY = 0.008  # a screwdriver's two pegs, either side of its axis: 1 cm between them
SCREW_AXIS_OUT = 0.025  # its axis this far in front of the board: a 3.8 cm handle clears the board by 6 mm


def screwdriver(name: str, package: str, y: float, shank_x: float, axis_z: float, mass: float, peg_z: float = PEG_Z):
    """Blade down between two pegs, the bolster (17 to 26 mm) resting on them, the axis 2.5 cm in front of the board. It
    spawns with its shank (5 mm or thinner) between the pegs, ``shank_x`` from the file's origin, and settles onto
    them. ``axis_z`` is the file's shank axis above its z = 0."""
    pos = (BOARD_FACE_X - SCREW_AXIS_OUT - axis_z, y, peg_z + shank_x)
    return _add(_sim(name, package, pos, mass, HANG_BLADE_DOWN), [(y - SCREW_PEG_DY, peg_z), (y + SCREW_PEG_DY, peg_z)])


def ring_tool(name: str, package: str, y: float, hole_xy: tuple[float, float], thick: float, mass: float, peg_z: float):
    """A wrench hung by the ring at its box end, the ring up, a peg through the hole at ``hole_xy`` (in the file)."""
    pos = (BOARD_FACE_X - SPAWN_GAP - thick, y - hole_xy[1], peg_z + hole_xy[0])
    return _add(_sim(name, package, pos, mass, HANG_BLADE_DOWN), [(y, peg_z)])


def head_up(name: str, package: str, y: float, axis_y: float, rest_x: float, mass: float, peg_z: float):
    """A hammer head up, its handle between two pegs 3.8 cm apart, the head resting on them. ``axis_y`` is the handle's
    centre line and ``rest_x`` the head's underside in the file. It spawns 3 mm above the pegs."""
    pos = (BOARD_FACE_X - SPAWN_GAP, y - axis_y, peg_z + PEG / 2 + 0.003 - rest_x)
    return _add(_sim(name, package, pos, mass, HANG_JAWS_UP), [(y - 0.022, peg_z), (y + 0.022, peg_z)])


CUTTERS_Y = -0.26  # in front of the right arm's base (y = -0.305)
SD, PL = "handTool_screwdriver_", "handTool_pliers_"
# From the robot's right to left. Masses: 15 cm screwdrivers 60 g, 19 cm 100 g.
screwdriver("phillips_ph1", f"{SD}phillips_ph1_a01", -0.40, 0.05, 0.0131, 0.06)
screwdriver("phillips_ph2", f"{SD}phillips_ph2_a01", -0.34, 0.05, 0.0164, 0.10)
CUTTERS = pliers("cutters", f"{PL}diagonal_cutting_a01", CUTTERS_Y, 0.20)
screwdriver("torx_t10", f"{SD}torx_t10_a01", CUTTERS_Y, 0.05, 0.0131, 0.06, peg_z=TOP + 0.35)  # above the cutters
pliers("combination_pliers", f"{PL}combination_a01", -0.18, 0.25)
pliers("long_nose_pliers", f"{PL}long_nose_a01", -0.10, 0.18)
pliers("curved_long_nose_pliers", f"{PL}long_nose_curved_a01", -0.02, 0.18)
ring_tool("combination_wrench", "handTool_wrench_combination_a01", 0.06, (-0.106, -0.001), 0.0176, 0.15, TOP + 0.30)
pliers("groove_joint_pliers", f"{PL}groove_joint_10in_b01", 0.13, 0.30, peg_z=TOP + 0.21, spawn_x=0.035)
head_up("claw_hammer", "tool_hammer_a01", 0.205, 0.002, 0.118, 0.45, TOP + 0.32)
screwdriver("slotted_5_5mm", f"{SD}slotted_55mm_a01", 0.31, 0.07, 0.0163, 0.10)
screwdriver("torx_t20", f"{SD}torx_t20_a01", 0.37, 0.07, 0.0163, 0.10)
OTHER_TOOLS = [t for t in TOOLS if t is not CUTTERS]

# -- the mat: 24 x 14 cm rubber, 3 mm thick ---------------------------------------------------------------------------
MAT_XY = (0.20, CUTTERS_Y)
MAT_HALF = (0.12, 0.07)
MAT_T = 0.003
MAT = Box(
    "mat",
    pos=(MAT_XY[0], MAT_XY[1], TOP + MAT_T / 2),
    size=(2 * MAT_HALF[0], 2 * MAT_HALF[1], MAT_T),
    color=(0.16, 0.30, 0.22),
)

OBJECTS = [BOARD, *PEGS, MAT, *TOOLS]

# -- scoring ---------------------------------------------------------------------------------------------------------
FALLEN_DROP = 0.05  # m: a tool whose centre is this much below where it hangs has fallen off its pegs
# Where each tool's centre hangs, measured in the simulator after the scene settles (README.md, "Scoring").
HANGING_Z = {  # m, world (the table top is at 0.7475)
    "phillips_ph1": 0.904,
    "phillips_ph2": 0.901,
    "cutters": 0.885,
    "torx_t10": 1.103,
    "combination_pliers": 0.883,
    "long_nose_pliers": 0.906,
    "curved_long_nose_pliers": 0.906,
    "combination_wrench": 0.930,
    "groove_joint_pliers": 0.906,
    "claw_hammer": 0.942,
    "slotted_5_5mm": 0.900,
    "torx_t20": 0.901,
}
ON_MAT_Z = 0.03  # m above the table: the cutters' centre is lower when they lie on the mat
OFF_BOARD_X = PEG_TIP_X - 0.02  # m: the cutters' centre nearer the robot than this is clear of the pegs
OFF_TABLE_Z = -0.02  # m above the table: an object whose centre is lower has fallen off the table

def _finite(objects) -> bool:
    return all(np.all(np.isfinite(s.pos)) and np.all(np.isfinite(s.lin_vel)) for s in objects.values())


def fallen(objects) -> list[str]:
    """The other tools that no longer hang: their centre is more than 5 cm below where it hangs."""
    return [t.name for t in OTHER_TOOLS if objects[t.name].pos[2] < HANGING_Z[t.name] - FALLEN_DROP]


def on_mat(objects) -> bool:
    """The cutters lie on the mat: their centre is over it and less than 3 cm above the table."""
    c = objects["cutters"].pos
    return bool(
        abs(c[0] - MAT_XY[0]) < MAT_HALF[0] and abs(c[1] - MAT_XY[1]) < MAT_HALF[1] and c[2] < TOP + ON_MAT_Z
    )


def off_board(objects) -> bool:
    """The cutters are off their peg: their centre is at least 2 cm nearer the robot than the pegs' tips."""
    return bool(objects["cutters"].pos[0] < OFF_BOARD_X)


def _violation(objects) -> bool:
    if not _finite(objects):
        return True
    return any(objects[t.name].pos[2] < TOP + OFF_TABLE_Z for t in TOOLS)


def success(objects) -> bool:
    """The cutters lie still on the mat and every other tool still hangs."""
    return bool(not _violation(objects) and on_mat(objects) and objects["cutters"].resting() and not fallen(objects))


def score(objects) -> float:
    """1.0: success. 0.5: the cutters lie still on the mat, one other tool fell. 0.25: the cutters are off the board
    (held, or put down off the mat) and no other tool fell. 0 otherwise, and 0 for any tool off the table."""
    if _violation(objects):
        return 0.0
    down = len(fallen(objects))
    if on_mat(objects) and objects["cutters"].resting():
        return 1.0 if down == 0 else 0.5 if down == 1 else 0.0
    return 0.25 if down == 0 and off_board(objects) else 0.0


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Right arm. Grip the cutters' jaws from the front (fingers closing across their 2.2 cm width), lift them 3 cm off the
# peg, pull them 8 cm clear, turn the grip straight down (the cutters swing flat, handles toward the robot) and lay
# them on the mat. Numbers are TCP positions. Tuned from the settle run.
GRIP_X = BOARD_FACE_X - 0.019  # the TCP 1.9 cm in front of the board: pad tips stop 6 mm short of it
GRIP_Z = PEG_Z + 0.055  # on the jaws, 5.5 cm above the peg
LIFT = 0.03
PULL_X = GRIP_X - 0.08
PLACE_X = MAT_XY[0] + 0.07  # the jaws over the mat's far half, so the handles lie on its near half
PLACE_Z = TOP + MAT_T + 0.03
FINGER_HALF = -0.025  # open 5 cm: enough for the jaws, clear of the neighbours
FWD = math.pi / 2


def demo():
    w = TiltWaypoint
    y = CUTTERS_Y
    return {
        "right": [
            w(2.0, (GRIP_X - 0.08, y, GRIP_Z), FINGER_HALF, "in front of the jaws", pitch=FWD),
            w(1.5, (GRIP_X, y, GRIP_Z), FINGER_HALF, "around the jaws", pitch=FWD),
            w(0.8, (GRIP_X, y, GRIP_Z), FINGER_CLOSED, "close", pitch=FWD),
            w(1.2, (GRIP_X, y, GRIP_Z + LIFT), FINGER_CLOSED, "lift off the peg", pitch=FWD),
            w(1.5, (PULL_X, y, GRIP_Z + LIFT), FINGER_CLOSED, "pull clear", pitch=FWD),
            w(2.0, (PLACE_X, y, PLACE_Z + 0.08), FINGER_CLOSED, "turn down", pitch=0.0),
            w(1.2, (PLACE_X, y, PLACE_Z), FINGER_CLOSED, "lower", pitch=0.0),
            w(0.6, (PLACE_X, y, PLACE_Z), FINGER_OPEN, "release", pitch=0.0),
            w(1.0, (PLACE_X, y, PLACE_Z + 0.08), FINGER_OPEN, "retreat", pitch=0.0),
        ]
    }
