"""Take the lid off a stovetop pressure pan whose lock a sprung vent button releases, keeping out of the steam, and set it on the trivet."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from geometry import (
    COLOR_HANDLE,
    COLOR_KNOB,
    COLOR_LID,
    COLOR_LOCK,
    COLOR_PAN,
    COLOR_POST,
    COLOR_TRIVET,
    COLOR_VALVE,
    COLOR_VENT,
    LID_DISC,
    LID_FRICTION,
    LID_KNOB,
    LID_LUG,
    LID_MASS_KG,
    LID_NAME,
    LID_REST_Z_M,
    LID_VALVE,
    PAN_CYL,
    PAN_FRICTION,
    PAN_HANDLE,
    PAN_KEY_PIN,
    PAN_MASS_KG,
    PAN_NAME,
    PAN_POSTS,
    PAN_XY,
    SUCCESS_HOLD_TIME_S,
    TABLE_TOP,
    TRIVET_FRICTION,
    TRIVET_MASS_KG,
    TRIVET_NAME,
    TRIVET_SIZE,
    TRIVET_XY,
    VENT_ARM,
    VENT_BUTTON,
    VENT_CROSS,
    VENT_DAMPING,
    VENT_MASS_KG,
    VENT_NAME,
    VENT_POST,
    VENT_REST,
    VENT_SPRING_N_PER_M,
    VENT_STEM,
    VENT_TRAVEL_M,
)
import tracekit  # noqa: F401
from parts import Slider, part
import demo_plan
import scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = SUCCESS_HOLD_TIME_S
SCORE_OVER = "end"

# The vent assembly (button, stem, cross bar, post, lock arm) is one slide along +x carried by the pan part; it moves
# toward +x when pressed, so its joint position is 0 at rest and positive when pressed.
VENT = Slider(
    VENT_NAME,
    rest=VENT_REST,
    boxes=[
        (VENT_BUTTON[0], VENT_BUTTON[1], COLOR_VENT),
        (VENT_STEM[0], VENT_STEM[1], COLOR_LOCK),
        (VENT_CROSS[0], VENT_CROSS[1], COLOR_LOCK),
        (VENT_POST[0], VENT_POST[1], COLOR_LOCK),
        (VENT_ARM[0], VENT_ARM[1], COLOR_LOCK),
    ],
    travel=VENT_TRAVEL_M,
    rate=VENT_SPRING_N_PER_M,
    damping=VENT_DAMPING,
    mass=VENT_MASS_KG,
    axis="+X",
)

PAN = part(
    PAN_NAME,
    (PAN_XY[0], PAN_XY[1], TOP),
    boxes=[(PAN_HANDLE[0], PAN_HANDLE[1], COLOR_HANDLE)]
    + [(c, s, COLOR_POST) for c, s in PAN_POSTS]
    + [(PAN_KEY_PIN[0], PAN_KEY_PIN[1], COLOR_POST)],
    cylinders=[(PAN_CYL[0], PAN_CYL[1], PAN_CYL[2], COLOR_PAN, "Z")],
    sliders=[VENT],
    mass=PAN_MASS_KG,
    friction=PAN_FRICTION,
)

LID = part(
    LID_NAME,
    (PAN_XY[0], PAN_XY[1], TOP + LID_REST_Z_M),
    boxes=[(LID_LUG[0], LID_LUG[1], COLOR_LID)],
    cylinders=[
        (LID_DISC[0], LID_DISC[1], LID_DISC[2], COLOR_LID, "Z"),
        (LID_KNOB[0], LID_KNOB[1], LID_KNOB[2], COLOR_KNOB, "Z"),
        (LID_VALVE[0], LID_VALVE[1], LID_VALVE[2], COLOR_VALVE, "Z"),
    ],
    mass=LID_MASS_KG,
    friction=LID_FRICTION,
)

TRIVET = part(
    TRIVET_NAME,
    (TRIVET_XY[0], TRIVET_XY[1], TOP),
    boxes=[((0.0, 0.0, TRIVET_SIZE[2] / 2), TRIVET_SIZE, COLOR_TRIVET)],
    mass=TRIVET_MASS_KG,
    friction=TRIVET_FRICTION,
)

OBJECTS = [PAN, LID, TRIVET]

success = scoring.success
score = scoring.score
distance = scoring.distance
demo = demo_plan.plans
