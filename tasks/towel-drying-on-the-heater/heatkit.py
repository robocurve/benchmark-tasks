"""Geometry and measurements for switching on a small quartz heater that has a towel drying over it.

A white two-bar quartz heater stands on the floor of the scene (the counter), its glowing side facing the
robot's right. A hand towel has been left draped over its top, hanging down over its mesh guard. Its rocker
switch is low on the front.

Frame: metres, x away from the arm bases, y left, z up; TOP is the counter. Sizes are from the CAD model and the
cloth simulation, themselves estimates of typical products.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from simkit import Part, Spring

TOP = 0.7475
ASSETS = Path(__file__).resolve().parent / "assets"
GREY = (0.9, 0.9, 0.9)
BLUE = (0.47, 0.67, 0.86)

# -- heater (CAD frame: front +x, 120 deep; 260 wide along y; 300 tall). Turned -90 degrees: its front faces -y. ---
HEATER_C = (0.360, 0.020)
H_LEN, H_DEEP, H_TOP = 0.260, 0.120, 0.300
YAW = -math.pi / 2


def _turn(x, y):
    """A point in the heater's CAD frame, as world offsets from HEATER_C."""
    return (y, -x)


# -- its rocker switch: a cap 14 mm wide, 22 mm tall, standing 4 mm proud; pressed 3 mm in ---------------------
SW_LOCAL = (0.0635, -0.064, 0.045)   # the cap's centre in the CAD frame
SW_XY = tuple(c + d for c, d in zip(HEATER_C, _turn(SW_LOCAL[0], SW_LOCAL[1])))
SW_SIZE = (0.014, 0.004, 0.022)      # world: along x, along y (its travel), tall
SW_TRAVEL, SW_K, SW_ON = 0.003, 1500.0, 0.0018

# -- towel: a terry hand towel 200 x 460 mm, about 50 g; the robot holds it by a pinch of fabric at its middle ---
TAB = (0.060, 0.020, 0.024)          # the pinch (hidden collider): along x, along y, tall
TW_MASS = 0.05
TW_POS = (HEATER_C[0], HEATER_C[1], TOP + H_TOP + TAB[2] / 2 + 0.0005)
CLEAR_M = 0.40                       # the towel is clear of the heater at least this far from its centre


def _vis(names, offset, **kw):
    names = (names,) if isinstance(names, str) else names
    return dict(visual_usd=";".join(str(ASSETS / f"{n}.usdc") for n in names),
                visual_prim=";".join(f"/{n}" for n in names), visual_offset=offset, **kw)


def heater_object() -> Part:
    """The heater: fixed. Collider (hidden): one box, 4 mm short of the front so the rocker has room to travel."""
    deep = H_DEEP - 0.008
    dx, dy = _turn(-0.004, 0.0)
    return Part("heater", pos=(HEATER_C[0] + dx, HEATER_C[1] + dy, TOP + H_TOP / 2), size=(deep, H_LEN, H_TOP), exact_z=True,
                color=GREY, friction=(0.6, 0.5), yaw=YAW, **_vis("heater", (0.004, 0.0, -H_TOP / 2)))


def switch_object() -> Part:
    spring = Spring(axis="Y", lower=0.0, upper=SW_TRAVEL, stiffness=SW_K, damping=6.0, target=0.0)
    return Part("heater_switch", pos=(SW_XY[0], SW_XY[1], TOP + SW_LOCAL[2]), size=SW_SIZE, exact_z=True, mass=0.003,
                color=(0.8, 0.08, 0.06), friction=(0.8, 0.7), spring=spring, contact_offset=0.001,
                **_vis("rocker_a", (-SW_XY[0] + HEATER_C[0], -SW_XY[1] + HEATER_C[1], -SW_LOCAL[2]),
                       visual_rpy_deg=(0.0, 0.0, math.degrees(YAW))))


def towel_object() -> Part:
    """The towel. For contact it is a small rigid pinch of fabric at its middle; what is drawn is one of three shapes
    from a cloth simulation (draped over the heater, hanging from the pinch, laid down), switched by the task."""
    return Part("towel", pos=TW_POS, size=TAB, exact_z=True, mass=TW_MASS, color=BLUE, friction=(1.2, 1.0), restitution=0.0,
                contact_offset=0.001, **_vis(("towel_draped", "towel_hanging", "towel_dropped"), (0.0, 0.0, 0.0)))


# -- measurements -----------------------------------------------------------------------------------

def switch_press(sw) -> float:
    """How far the rocker is pushed in (m)."""
    return float(np.asarray(sw, float)[1] - SW_XY[1])


def towel_on_heater(towel) -> bool:
    """The towel is on the heater: its pinch within the heater's top outline (plus 3 cm) and higher than 20 cm."""
    t = np.asarray(towel, float)
    return bool(abs(t[0] - HEATER_C[0]) <= H_LEN / 2 + 0.03 and abs(t[1] - HEATER_C[1]) <= H_DEEP / 2 + 0.03 and t[2] > TOP + 0.20)


def towel_in_front(towel) -> bool:
    """The towel's pinch is on the mesh side of the heater: beyond the plane of its mesh guard (which faces -y)."""
    return bool(float(np.asarray(towel, float)[1]) < HEATER_C[1] - H_DEEP / 2)


def towel_clear(towel) -> bool:
    """The towel is down on the counter (or off it), its pinch at least CLEAR_M from the heater's centre and not
    in front of the mesh guard, where the elements radiate."""
    t = np.asarray(towel, float)
    return bool(math.hypot(t[0] - HEATER_C[0], t[1] - HEATER_C[1]) >= CLEAR_M and t[2] < TOP + 0.05 and not towel_in_front(t))
