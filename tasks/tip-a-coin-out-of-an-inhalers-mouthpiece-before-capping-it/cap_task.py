"""Kit check for the inhaler task, not for review: the cap's push fit (capfit.py) under known loads. The inhaler is fixed
where task.py lays it, as if a hand held it still, with the dime lodged as at the start; the cap is pushed with a known
force alone. Run by trials.py ``cap``."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import capfit as F  # noqa: E402
import lodge as L  # noqa: E402
import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, matrix_to_quat, weld  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = "Kit check: the inhaler's cap pushed on."
TOP = TABLE_TOP[SCENE]
c, s = math.cos(math.radians(-60.0)), math.sin(math.radians(-60.0))
INHALER_ROT = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # as task.py
COIN_IN_I = P.coin_lodged(0.45)
INHALER_POS = np.array([*(np.array([0.40, -0.29]) - (INHALER_ROT @ COIN_IN_I)[:2]), TOP + P.BARREL_R + 0.001])

_body_pos, _body_rot = F.body_pose(INHALER_POS, INHALER_ROT)
body("inhaler", F.inhaler_body_shapes(), pos=tuple(_body_pos), quat=matrix_to_quat(_body_rot), mass=P.INHALER_MASS, contact_offset=P.THIN)
body("coin", P.lodged_shapes(), pos=tuple(INHALER_POS + INHALER_ROT @ COIN_IN_I), mass=P.LODGED_MASS, friction=(P.COIN_FRICTION, P.COIN_FRICTION),
     friction_combine="min", contact_offset=P.THIN, contacts=True)  # fmt: skip
body("cap", P.cap_shapes(), pos=(0.16, 0.06, TOP + 0.5 * P.CAP_DEPTH + 0.001), quat=matrix_to_quat(np.array([[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [-1.0, 0.0, 0.0]])),
     mass=P.CAP_MASS, contact_offset=P.THIN)  # fmt: skip
F.declare()
L.declare()
weld("hold", "inhaler", None)  # held still where it lies, as by a hand (a free body, so that the fit's joints are those of task.py)

OBJECTS = list(mk.MECH.bodies.values())
EVENTS: list = []
rel, gap, seated, coin_in = F.rel, F.gap, F.seated, F.coin_in


@mk.on_reset
def start() -> None:
    EVENTS.clear()
    F.reset()
    L.reset()


@mk.on_step
def step() -> None:
    L.watch(EVENTS)
    F.watch(EVENTS)


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
