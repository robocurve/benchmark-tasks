"""Return an instrument module to the empty one of three covered cases.

The left arm removes each centred-handle lid and holds it to the left. The
right arm carries the incoming module and inspects the exposed cases with its
wrist camera. An occupied case is reclosed before moving to the next one.
"""

from __future__ import annotations

import sys
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from yam_arena.task_file import Box
from scenekit import ContactBox, UsdBody
from robocurve_case.design import make_design, MODULE_SIZE, MODULE_MASS
from robocurve_case import bridge

SCENE = "station"
SUCCESS_HOLD_S = 0.0  # The score bridge applies the full one-second stability hold.
SCORE_OVER = "end"

_LAYOUT = make_design("covered", 2)
OBJECTS = [
    (ContactBox if p.name.startswith("case_") else Box)(
        p.name, pos=p.position, size=p.size, color=p.color)
    for p in _LAYOUT.fixed
]
OBJECTS.extend(
    Box(name, pos=pos, size=MODULE_SIZE, mass=MODULE_MASS, color=(0.70, 0.48, 0.20))
    for name, pos in {
        "incoming": _LAYOUT.intake,
        "stored_0": (_LAYOUT.cradle_origins[0][0], _LAYOUT.cradle_origins[0][1],
                     _LAYOUT.cradle_origins[0][2] + MODULE_SIZE[2] / 2),
        "stored_1": (_LAYOUT.cradle_origins[1][0], _LAYOUT.cradle_origins[1][1],
                     _LAYOUT.cradle_origins[1][2] + MODULE_SIZE[2] / 2),
    }.items()
)
OBJECTS.extend(
    UsdBody(m.name, pos=m.origin, size=(0.106, 0.106, 0.040),
            mass=m.mass, usd=f"assets/{m.name}.usda")
    for m in _LAYOUT.mechanisms
)


def success(objects):
    return bridge.evaluate(objects).success_ready


def score(objects):
    return bridge.current_score()


def demo():
    from yam_arena.demo import Waypoint
    data = json.loads((HERE / "demo_waypoints.json").read_text())
    return {side: [Waypoint(**point) for point in points]
            for side, points in data.items()}
