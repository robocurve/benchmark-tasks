from __future__ import annotations
import pathlib, sys
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import tracekit  # noqa: F401
from geometry import (
    COLOR_LATCH,
    COLOR_TRAY,
    DESK_BOXES,
    DESK_FRICTION,
    DESK_MASS_KG,
    DESK_NAME,
    DESK_POS_WORLD,
    LATCH_DIMS_M,
    SUCCESS_HOLD_TIME_S,
    TABLE_TOP,
    TRAY_DIMS_M,
)
from parts import Slider, part
import demo_plan, scoring

SCENE = "station"
TOP = TABLE_TOP
SUCCESS_HOLD_S = SUCCESS_HOLD_TIME_S
SCORE_OVER = "end"

# Desk with 2 sliders:
# 1. Keyboard tray (+X slide, travel 280 mm, damping 20)
# 2. Purple latch catch (-Z slider, travel 8 mm, spring 100 N/m)
DESK = part(
    DESK_NAME,
    DESK_POS_WORLD,
    boxes=DESK_BOXES,
    mass=DESK_MASS_KG,
    friction=DESK_FRICTION,
    sliders=[
        Slider("tray", rest=(-0.140, 0.0, 0.020), boxes=[
            ((-0.140, 0.0, 0.020), TRAY_DIMS_M, COLOR_TRAY),
            ((-0.140, -0.185, 0.020), (0.040, 0.070, 0.020), COLOR_TRAY),
            ((-0.140, -0.220, 0.020), (0.040, 0.040, 0.040), COLOR_TRAY),
        ], axis="+X", travel=0.280, rate=0.001, damping=20.0),
        Slider("latch", rest=(0.140, 0.0, 0.040), boxes=[((0.140, 0.0, 0.040), LATCH_DIMS_M, COLOR_LATCH)], axis="Z", travel=0.008, rate=100.0, damping=10.0),
    ],
)

OBJECTS = [DESK]

success = scoring.success
score = scoring.score
distance = scoring.distance
demo = demo_plan.plans

# Aliases for backward compatibility
SCORER = score
SUCCESS = success
DEMO_PLANS = demo
DEMO_PLAN = demo
