"""A shoe-fitting foot measuring device lies across the table, heel cup on the left, toe end on the right. A rigid
right-foot form (a model foot with a 60 mm round ankle post on top) stands on the device's plate with its heel 20 to
30 mm forward of the heel cup. The device has three printed scales: a length scale under the toes, an arch-length
scale on the raised guide along the far (inner) side of the foot with a sliding pointer whose curved nose meets the
ball of the foot, and a width chart on the near side read at the end of a sliding width bar. The pointer starts slid
back toward the heel and the width bar starts at its widest. Measure the foot as the device's maker instructs: heel
seated against the back of the heel cup, the pointer slid forward until its curve fits the ball joint, the width bar
slid in against the side of the foot, then let go of everything.

Layout: trial k uses layout k mod 3 (environment TRIAL=k; BRANNOCK_LAYOUT=0, 1 or 2 overrides); the three layouts use
three different forms.
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import layout as LY  # noqa: E402
import scoring as SC  # noqa: E402
import simhook as H  # noqa: E402
from sizing import BAR_START, DEVICE, PTR_START  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
ASSETS = HERE / "assets"

if not (ASSETS / "device.usda").exists():
    import build_assets

    build_assets.write_all(ASSETS)

def build_assets_friction() -> tuple[float, float]:
    from build_assets import SLIDER_FRICTION_N

    return SLIDER_FRICTION_N


K = LY.trial_layout()
FORM, FORM_POS, FORM_Q = LY.form_start(K)
GEO = SC.FormGeometry(FORM)
DEVICE_Q = (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))  # device u -> world -y

OBJECTS = [
    H.UsdObject("device", pos=tuple(LY.to_world(0.0, 0.0, 0.0)), size=(0.22, 0.34, 0.05),
                usd=str(ASSETS / "device.usda"), articulated=True, rot_xyzw=DEVICE_Q,
                joint_pos0=(("pointer_slide", 0.0), ("bar_slide", 0.0)),
                joint_friction=build_assets_friction()),
    H.UsdObject("foot_form", pos=tuple(FORM_POS + np.array([0.0, 0.0, 0.0005])), size=(0.11, 0.28, 0.13),
                mass=0.27, usd=str(ASSETS / f"form_{FORM['name']}.usda"), rot_xyzw=FORM_Q),
]

S: dict = {}


@H.on_reset
def _reset():
    S.clear()
    S.update(latched=False, faults=[], last=None, armed=False, was_seated=False)


_reset()


def sliders_mm() -> tuple[float, float]:
    """(pointer index position u, width-bar distance d from the guide face), mm. The joints read 0 at the start."""
    pos, _ = H.joints("device")
    return PTR_START + pos["pointer_slide"] * 1000.0, BAR_START - pos["bar_slide"] * 1000.0


def readings(objects) -> dict:
    f = objects["foot_form"]
    _, vel = H.joints("device")
    index, d = sliders_mm()
    fingers = H.robot_points(("tip_left", "tip_right"))
    return SC.measure(GEO, f.pos, f.quat_xyzw, f.lin_vel, f.ang_vel, index / 1000.0, -d / 1000.0,
                      vel["pointer_slide"], vel["bar_slide"], fingers)


def score(objects) -> float:
    m = readings(objects)
    index, d = sliders_mm()
    if m["heel_gap_mm"] < SC.FEELER:
        S["was_seated"] = True
    moved = abs(index - PTR_START) >= SC.SLIDER_MOVED_MM or abs(d - BAR_START) >= SC.SLIDER_MOVED_MM
    if S["was_seated"] and moved:
        S["armed"] = True
    fl = SC.faults(m, S["armed"])
    if fl and not S["latched"]:
        S["latched"], S["faults"] = True, fl
    S["last"] = m
    return SC.score(m, S["latched"])


def success(objects) -> bool:
    m = S["last"] if S["last"] is not None else readings(objects)
    return (not S["latched"]) and SC.levels(m)["full"]


assert DEVICE["ptr_travel"][0] <= PTR_START <= DEVICE["ptr_travel"][1]
