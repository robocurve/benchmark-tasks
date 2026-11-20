"""Kit check for the inhaler task, not for review: a dime lodged in the mouthpiece of the training inhaler, the inhaler
held in the air on a turning joint about its barrel (lying on its side, mouthpiece towards +y), so that a check can
tip the mouthpiece down at a known rate or shake it, with no arm. Run by tube_trial.py."""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import mechkit as mk  # noqa: E402
import parts as P  # noqa: E402
from mechkit import body, hinge  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
INSTRUCTION = "Kit check: a coin in an inhaler's mouthpiece."
TOP = TABLE_TOP[SCENE]
HELD_AT = np.array([0.30, -0.06, TOP + 0.10])  # the barrel's axis meets the mouthpiece's here, 10 cm over the bench
TILT_K = 2.0  # N m/rad: the joint's spring, whose rest the check moves (the boot's own weight sags it 0.1 deg)
TILT_C = 0.012  # N m s/rad: about critical for the boot's 1.5e-5 kg m2 about the barrel

body("inhaler", P.inhaler_shapes(), pos=tuple(HELD_AT), quat=P.LYING_QUAT, mass=P.INHALER_MASS, contact_offset=P.THIN)
COIN_START = HELD_AT + P.LYING @ P.coin_lodged(0.46)
body("coin", P.lodged_shapes(), pos=tuple(COIN_START), mass=P.LODGED_MASS, friction=(P.COIN_FRICTION, P.COIN_FRICTION), friction_combine="min",
     contact_offset=P.THIN)  # fmt: skip
hinge("tilt", "inhaler", None, anchor=tuple(HELD_AT), axis=(1.0, 0.0, 0.0), limits=(-math.radians(100.0), math.radians(30.0)), spring=TILT_K,
      rest=0.0, damping=TILT_C)  # fmt: skip
mk.sense_contact("coin", "inhaler")

OBJECTS = list(mk.MECH.bodies.values())


@mk.on_reset
def start() -> None:
    if mk.MECH.environment is not None:
        mk.set_spring("tilt", rest=0.0)


def coin_in_inhaler() -> np.ndarray:
    """The coin's middle in the inhaler's frame I (m)."""
    p, r = mk.pose("inhaler")
    return r.T @ (mk.pose("coin")[0] - p)


def score(objects) -> float:
    return 0.0


def success(objects) -> bool:
    return False
