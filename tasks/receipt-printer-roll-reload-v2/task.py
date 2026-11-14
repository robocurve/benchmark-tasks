"""Reload a receipt printer v2: hold its hinged lid open, turn the paper roll so paper feeds out the front slot, seat it, let the lid close."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from geometry import (
    COVER_DAMPING,
    COVER_HIGH_RAD,
    COVER_LOW_RAD,
    COVER_MASS_KG,
    COVER_NAME,
    COVER_SHAPES,
    COVER_SPRING_N_M_PER_RAD,
    HINGE_AT,
    PAPER_ROLL_MASS_KG,
    PRINTER_FRICTION,
    PRINTER_NAME,
    PRINTER_SHAPES,
    PRINTER_XY,
    ROLL_FRICTION,
    ROLL_NAME,
    ROLL_SHAPES,
    ROLL_START_ORIGIN_Z,
    ROLL_START_XY,
    ROLL_START_YAW,
    STAND_FRICTION,
    STAND_NAME,
    STAND_SHAPES,
    STAND_XY,
    SUCCESS_HOLD_TIME_S,
    TABLE_TOP,
)
from mechkit import Joint, Mech
import demo_plan
import scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = SUCCESS_HOLD_TIME_S
SCORE_OVER = "end"

# The cover: a lid on a hinge at the back top edge of the printer body with a small spring toward closed.
COVER_JOINT = Joint(
    COVER_NAME,
    kind="hinge",
    axis="y",
    at=HINGE_AT,
    low=COVER_LOW_RAD,
    high=COVER_HIGH_RAD,
    spring=COVER_SPRING_N_M_PER_RAD,
    damping=COVER_DAMPING,
    mass=COVER_MASS_KG,
    shapes=COVER_SHAPES,
)

PRINTER = Mech(
    PRINTER_NAME,
    pos=(PRINTER_XY[0], PRINTER_XY[1], TOP),
    mass=None,
    friction=PRINTER_FRICTION,
    shapes=PRINTER_SHAPES,
    joints=[COVER_JOINT],
)

# The roll: loose Mech body carrying paper cylinder, core, and paper tail strip.
ROLL = Mech(
    ROLL_NAME,
    pos=(ROLL_START_XY[0], ROLL_START_XY[1], TOP + ROLL_START_ORIGIN_Z),
    mass=PAPER_ROLL_MASS_KG,
    yaw=ROLL_START_YAW,
    friction=ROLL_FRICTION,
    shapes=ROLL_SHAPES,
)

# Stand: fixed rubber plate with chock.
STAND = Mech(
    STAND_NAME,
    pos=(STAND_XY[0], STAND_XY[1], TOP),
    mass=None,
    friction=STAND_FRICTION,
    shapes=STAND_SHAPES,
)

OBJECTS = [PRINTER, ROLL, STAND]

success = scoring.success
score = scoring.score
distance = scoring.distance
demo = demo_plan.plans
