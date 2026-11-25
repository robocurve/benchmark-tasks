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
LAID = (-0.143, 0.146, -0.155, 0.192)  # outline of the towel as it lies on the counter, about its pinch: x0, x1, y0, y1 (from the cloth shape)
SIDE_M = 0.20                        # no part of the towel may lie nearer than this to the heater's sides or back


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


def _yaw(pose) -> float:
    x, y, z, w = (float(v) for v in np.asarray(pose, float)[3:7])
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def towel_outline(towel):
    """Points round the edge of the towel as it lies on the counter: the outline of its laid-down shape (LAID, about
    the pinch), turned with the pinch."""
    t = np.asarray(towel, float)
    c, s = math.cos(_yaw(t)), math.sin(_yaw(t))
    x0, x1, y0, y1 = LAID
    n = 8
    edge = [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1) for y in (y0, y1)] + [(x, y0 + (y1 - y0) * i / n) for i in range(n + 1) for x in (x0, x1)]
    return np.array([(t[0] + c * x - s * y, t[1] + s * x + c * y) for x, y in edge])


def towel_gap(towel) -> float:
    """Shortest distance (m) from any part of the laid-down towel to the heater's body."""
    pts = towel_outline(towel)
    dx = np.maximum(np.abs(pts[:, 0] - HEATER_C[0]) - H_LEN / 2, 0.0)
    dy = np.maximum(np.abs(pts[:, 1] - HEATER_C[1]) - H_DEEP / 2, 0.0)
    return float(np.min(np.hypot(dx, dy)))


def towel_in_front(towel) -> bool:
    """Any part of the laid-down towel is on the mesh side of the heater: beyond the plane of its mesh guard (-y)."""
    return bool(float(np.min(towel_outline(towel)[:, 1])) < HEATER_C[1] - H_DEEP / 2)


def towel_clear(towel) -> bool:
    """The whole towel is clear: lying on the counter, no part of it in front of the mesh guard (where the elements
    radiate), and no part of it nearer than SIDE_M to the heater's body."""
    t = np.asarray(towel, float)
    return bool(t[2] < TOP + 0.05 and not towel_in_front(t) and towel_gap(t) >= SIDE_M)
