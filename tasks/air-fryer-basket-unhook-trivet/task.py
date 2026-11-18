"""Air fryer fry-basket task: on a heat-resistant trivet, slide the handle's button guard forward, press the release
button on top of the handle while lifting the hot fry basket out of its outer basket by the grip, and set it on a mat."""

from __future__ import annotations

import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from geometry import (
    BASKET_BODY_BOXES,
    BASKET_FRICTION,
    BASKET_HANDLE_BOXES,
    BASKET_MASS,
    BASKET_START_POS,
    COLOR_BASKET,
    COLOR_BUTTON,
    COLOR_GUARD,
    COLOR_HANDLE,
    COLOR_HOUSING,
    COLOR_MAT,
    COLOR_PAN,
    COLOR_TRIVET,
    GUARD_AT,
    GUARD_BOXES,
    GUARD_DAMPING_NS_M,
    GUARD_MASS,
    GUARD_TRAVEL_M,
    HOUSING_BOXES,
    HOUSING_FRICTION,
    HOUSING_POS,
    LEVER_AT,
    LEVER_BOXES,
    LEVER_DAMPING,
    LEVER_FRICTION,
    LEVER_MASS,
    LEVER_RATE_NM_RAD,
    LEVER_TRAVEL_RAD,
    MAT_BOXES,
    MAT_CENTRE,
    PAN_BOXES,
    PAN_FRICTION,
    PAN_MASS,
    PAN_START_POS,
    STAND_BOX,
    TABLE_TOP,
    TRIVET_BOXES,
    TRIVET_POS,
)
import demo_plan
from mechkit import Joint, Mech, box
import scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# 1. The appliance body (fixed, hot), its drawer bay empty
HOUSING = Mech(
    "af3_housing",
    pos=HOUSING_POS,
    mass=None,
    friction=HOUSING_FRICTION,
    shapes=[box(c, s, COLOR_HOUSING) for c, s, _ in HOUSING_BOXES] + [box(STAND_BOX[0], STAND_BOX[1], COLOR_TRIVET)],
)

# 2. Heat-resistant trivet (fixed) under the outer basket
TRIVET = Mech("af3_trivet", pos=TRIVET_POS, mass=None, friction=1.0,
              shapes=[box(c, s, COLOR_TRIVET) for c, s, _ in TRIVET_BOXES])

# 3. Outer basket (loose 600 g steel pan) with the catch lip on its front wall
PAN = Mech("af3_pan", pos=PAN_START_POS, mass=PAN_MASS, friction=PAN_FRICTION,
           shapes=[box(c, s, COLOR_PAN) for c, s, _ in PAN_BOXES])

# 4. Fry basket (loose 300 g) with its inverted-L handle, the sprung release lever and the sliding button guard
lever_joint = Joint(
    "lever",
    "hinge",
    axis="y",
    at=LEVER_AT,
    low=0.0,
    high=LEVER_TRAVEL_RAD,
    spring=LEVER_RATE_NM_RAD,
    damping=LEVER_DAMPING,
    mass=LEVER_MASS,
    friction=LEVER_FRICTION,
    shapes=[box(c, s, COLOR_BUTTON) for c, s, _ in LEVER_BOXES],
)
guard_joint = Joint(
    "guard",
    "slide",
    axis="x",
    at=GUARD_AT,
    low=0.0,
    high=GUARD_TRAVEL_M,
    spring=0.0,
    damping=GUARD_DAMPING_NS_M,
    mass=GUARD_MASS,
    shapes=[box(c, s, COLOR_GUARD) for c, s, _ in GUARD_BOXES],
)
BASKET = Mech(
    "af3_basket",
    pos=BASKET_START_POS,
    mass=BASKET_MASS,
    friction=BASKET_FRICTION,
    shapes=[box(c, s, COLOR_BASKET) for c, s, _ in BASKET_BODY_BOXES]
    + [box(c, s, COLOR_HANDLE) for c, s, _ in BASKET_HANDLE_BOXES],
    joints=[lever_joint, guard_joint],
)

# 5. Heat-proof mat (fixed) for the fry basket
MAT = Mech("af3_mat", pos=MAT_CENTRE, mass=None, friction=1.2,
           shapes=[box(c, s, COLOR_MAT) for c, s, _ in MAT_BOXES])

OBJECTS = [HOUSING, TRIVET, PAN, BASKET, MAT]

score = scoring.score
success = scoring.success
distance = scoring.distance
demo = demo_plan.plans
