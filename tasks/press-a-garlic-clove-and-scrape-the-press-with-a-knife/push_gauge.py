"""A check, not the task: the push the right arm gives on the garlic press's upper handle, read against a spring.

The board (30 mm) and, in the press's place, a stand-in for its upper handle: a bar hinged to the world on the press's
hinge axis, held open by a stiff spring (``lever``, 200 N m/rad) at the angle under test. A hand that pushes the bar
down turns it against the spring: torque = stiffness x the turn. Without the lower handle, so that the hand's
fingertips touch nothing but the bar. Used by ``push_check.py``.
"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
from mechkit import body, box, hinge  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
INSTRUCTION = "Push the handle down."
TOP = TABLE_TOP[SCENE]
BEECH, ALUMINIUM = (0.80, 0.62, 0.40), (0.78, 0.79, 0.80)

# The layout under test (bench frame, m): the press along y, its tip (hinge end) at y TIP_Y, its grip end towards the
# right arm; u runs along the press from the tip, w up from the underside of its lower handle (on the board's top).
BOARD = ((0.205, 0.355), (-0.285, -0.035), 0.030)  # x span, y span, thickness
PRESS_X, TIP_Y = 0.28, 0.015
HINGE_UW = (0.004, 0.014)
PUSH_UW = (0.130, 0.018)  # where the hand pinches the upper handle (closed): 25 mm from its end, at its middle
K, DAMPING = 200.0, 0.5  # N m/rad, N m s/rad
TEST_DEG = [25.0]  # the angle the bar stands open at; set by the runner before a reset


def world(u: float, w: float) -> np.ndarray:
    return np.array([PRESS_X, TIP_Y - u, TOP + BOARD[2] + w])


def opened(u: float, w: float, phi: float) -> tuple[float, float]:
    """(u, w) of a point of the upper handle when it is opened by ``phi`` rad (the grip end rises)."""
    du, dw = u - HINGE_UW[0], w - HINGE_UW[1]
    c, s = math.cos(phi), math.sin(phi)
    return HINGE_UW[0] + c * du - s * dw, HINGE_UW[1] + s * du + c * dw


(bx0, bx1), (by0, by1), bt = BOARD
body("board", [box((bx1 - bx0, by1 - by0, bt))], pos=((bx0 + bx1) / 2, (by0 + by1) / 2, TOP + bt / 2), color=BEECH)
HINGE_AT = world(*HINGE_UW)
# The bar in its frame (origin on the hinge axis, closed): u maps to -y, w to +z. 26 mm wide, 12 mm thick, from the
# hinge to the grip end (155 mm from the tip).
_len = 0.155 - HINGE_UW[0]
LEVER = body("lever", [box((0.026, _len, 0.012), pos=(0.0, -_len / 2, 0.018 - HINGE_UW[1]))], pos=tuple(HINGE_AT), mass=0.07, color=ALUMINIUM)
hinge("lever", "lever", None, anchor=tuple(HINGE_AT), axis=(-1.0, 0.0, 0.0), limits=(-0.05, 1.2), spring=K, rest=0.0, damping=DAMPING)
OBJECTS = list(mk.MECH.bodies.values())


@mk.on_reset
def start() -> None:
    if mk.MECH.environment is not None:
        phi = math.radians(TEST_DEG[0])
        mk.set_spring("lever", rest=phi)
        mk.set_joint("lever", phi)


def grasp_point(phi: float) -> np.ndarray:
    return world(*opened(*PUSH_UW, phi))


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
