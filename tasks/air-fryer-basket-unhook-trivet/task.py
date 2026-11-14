"""Air fryer basket release button task: release and remove a hot air fryer basket onto a heat-proof mat."""

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
    BUTTON_AT,
    BUTTON_BOXES,
    BUTTON_DAMPING_NS_M,
    BUTTON_MASS,
    BUTTON_RATE_N_M,
    BUTTON_TRAVEL_M,
    COLOR_BASKET,
    COLOR_BUTTON,
    COLOR_DRAWER,
    COLOR_HANDLE,
    COLOR_HOUSING,
    COLOR_MAT,
    DRAWER_AT,
    DRAWER_BOXES,
    DRAWER_DAMPING_NS_M,
    DRAWER_MASS,
    DRAWER_RATE_N_M,
    DRAWER_TRAVEL_M,
    HOUSING_BOXES,
    STAND_BOX,
    COLOR_STAND,
    HOUSING_FRICTION,
    HOUSING_POS,
    MAT_BOXES,
    MAT_CENTRE,
    TABLE_TOP,
)
import demo_plan
from mechkit import Joint, Mech, box
import scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# 1. Housing (fixed to the bench) carrying the drawer on a prismatic slide
housing_shapes = [box(c, s, COLOR_HOUSING) for c, s, _ in HOUSING_BOXES] + [box(STAND_BOX[0], STAND_BOX[1], COLOR_STAND)]
drawer_shapes = [box(c, s, COLOR_DRAWER) for c, s, _ in DRAWER_BOXES]

drawer_joint = Joint(
    "drawer",
    "slide",
    axis="x",
    at=DRAWER_AT,
    low=0.0,
    high=DRAWER_TRAVEL_M,
    spring=DRAWER_RATE_N_M,
    damping=DRAWER_DAMPING_NS_M,
    mass=DRAWER_MASS,
    shapes=drawer_shapes,
)

HOUSING = Mech(
    "af2_housing",
    pos=HOUSING_POS,
    mass=None,
    friction=HOUSING_FRICTION,
    shapes=housing_shapes,
    joints=[drawer_joint],
)

# 2. Basket (loose 350 g body) carrying the release button on a prismatic slide
basket_shapes = (
    [box(c, s, COLOR_BASKET) for c, s, _ in BASKET_BODY_BOXES]
    + [box(c, s, COLOR_HANDLE) for c, s, _ in BASKET_HANDLE_BOXES]
)
button_shapes = [box(c, s, COLOR_BUTTON) for c, s, _ in BUTTON_BOXES]

button_joint = Joint(
    "button",
    "slide",
    axis="x",
    at=BUTTON_AT,
    low=0.0,
    high=BUTTON_TRAVEL_M,
    spring=BUTTON_RATE_N_M,
    damping=BUTTON_DAMPING_NS_M,
    mass=BUTTON_MASS,
    shapes=button_shapes,
)

BASKET = Mech(
    "af2_basket",
    pos=BASKET_START_POS,
    mass=BASKET_MASS,
    friction=BASKET_FRICTION,
    shapes=basket_shapes,
    joints=[button_joint],
)

# 3. Heat-proof mat (fixed to the bench)
mat_shapes = [box(c, s, COLOR_MAT) for c, s, _ in MAT_BOXES]
MAT = Mech(
    "af2_mat",
    pos=MAT_CENTRE,
    mass=None,
    friction=1.2,
    shapes=mat_shapes,
)

OBJECTS = [HOUSING, BASKET, MAT]

score = scoring.score
success = scoring.success
distance = scoring.distance
demo = demo_plan.plans
