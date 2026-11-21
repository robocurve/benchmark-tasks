"""Geometry and measurements for putting raw chicken on the grill and bread on a plate with clean hands.

A kitchen counter. On the right: a foam tray with two raw chicken thigh fillets, and beside it a bread basket
with two rolls. In the middle: a cast-iron grill pan and a dinner plate. A gripper that has held raw chicken
carries its juices to whatever it touches next.

Frame: metres, x away from the arm bases, y left, z up; TOP is the counter. Sizes are from the CAD models,
themselves estimates of typical products.
"""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from simkit import Part, Piece

TOP = 0.7475
ASSETS = Path(__file__).resolve().parent / "assets"
GREY = (0.7, 0.7, 0.7)
PINK = (0.93, 0.66, 0.62)
CRUST = (0.80, 0.56, 0.25)

# -- raw chicken thigh fillets (CAD 110 x 70 x 22 mm at 0.9) ------------------------------------------
CH_S = 0.90
CH = (0.099, 0.063, 0.020)
CH_MASS = 0.12
TRAY_C = (0.270, -0.360)
TRAY = (0.230, 0.150, 0.022)         # outside; its floor is 3 mm up
TRAY_FLOOR = 0.003
CHICKEN_XY = ((0.215, -0.360), (0.325, -0.360))

# -- bread rolls (CAD 95 mm across at 0.85) and their basket ---------------------------------------------
BUN_S = 0.85
BUN_R, BUN_H = 0.040, 0.047
BUN_MASS = 0.05
BASKET_C = (0.220, -0.130)
BASKET_IN = (0.099, 0.064)           # inside half-sizes
BASKET_FLOOR, BASKET_RIM = 0.005, 0.070
BUN_XY = ((0.165, -0.130), (0.275, -0.130))

# -- grill pan (CAD 240 mm square at 0.8) -----------------------------------------------------------------
PAN_S = 0.80
PAN_C = (0.460, -0.100)
PAN_IN = 0.107 * PAN_S               # inside half-size at the floor
PAN_RIDGE, PAN_RIM = 0.013 * PAN_S, 0.045 * PAN_S
PAN_SPOTS = ((0.420, -0.145), (0.420, -0.055))

# -- dinner plate ----------------------------------------------------------------------------------------
PLATE_C = (0.220, 0.130)
PLATE_R, PLATE_FLAT, PLATE_SURF = 0.125, 0.088, 0.008
PLATE_SPOTS = ((0.170, 0.110), (0.270, 0.110))


def _vis(names, offset, **kw):
    names = (names,) if isinstance(names, str) else names
    return dict(visual_usd=";".join(str(ASSETS / f"{n}.usdc") for n in names),
                visual_prim=";".join(f"/{n}" for n in names), visual_offset=offset, **kw)


def _walls(hx, hy, z0, z1, zc, t=0.004):
    h = z1 - z0
    return (Piece("box", (t, 2 * hy + 2 * t, h), (-hx - t / 2, 0.0, (z0 + z1) / 2 - zc), color=GREY),
            Piece("box", (t, 2 * hy + 2 * t, h), (hx + t / 2, 0.0, (z0 + z1) / 2 - zc), color=GREY),
            Piece("box", (2 * hx, t, h), (0.0, -hy - t / 2, (z0 + z1) / 2 - zc), color=GREY),
            Piece("box", (2 * hx, t, h), (0.0, hy + t / 2, (z0 + z1) / 2 - zc), color=GREY))


def tray_object() -> Part:
    zc = TRAY_FLOOR / 2
    return Part("tray", pos=(TRAY_C[0], TRAY_C[1], TOP + zc), size=(TRAY[0] - 0.02, TRAY[1] - 0.02, TRAY_FLOOR), exact_z=True,
                color=GREY, friction=(0.6, 0.5), pieces=_walls(TRAY[0] / 2 - 0.008, TRAY[1] / 2 - 0.008, 0.0, TRAY[2], zc),
                **_vis("meat_tray", (0.0, 0.0, -zc)))


def basket_object() -> Part:
    zc = BASKET_FLOOR / 2
    return Part("basket", pos=(BASKET_C[0], BASKET_C[1], TOP + zc), size=(2 * BASKET_IN[0], 2 * BASKET_IN[1], BASKET_FLOOR),
                exact_z=True, color=GREY, friction=(0.6, 0.5),
                pieces=_walls(BASKET_IN[0] + 0.012, BASKET_IN[1] + 0.012, 0.0, BASKET_RIM, zc, t=0.006),
                **_vis("bread_basket", (0.0, 0.0, -zc)))


def pan_object() -> Part:
    """The grill pan: fixed; its long handle points away from the arms."""
    zc = PAN_RIDGE / 2
    return Part("pan", pos=(PAN_C[0], PAN_C[1], TOP + zc), size=(2 * PAN_IN, 2 * PAN_IN, PAN_RIDGE), exact_z=True, color=GREY,
                friction=(0.6, 0.5), pieces=_walls(PAN_IN + 0.004, PAN_IN + 0.004, 0.0, PAN_RIM, zc), yaw=math.pi,
                **_vis("grill_pan", (-0.050 * PAN_S, 0.0, -zc), visual_scale=PAN_S))


def plate_object() -> Part:
    zc = PLATE_SURF / 2
    return Part("plate", pos=(PLATE_C[0], PLATE_C[1], TOP + zc), size=(PLATE_R - 0.01, PLATE_SURF, PLATE_R - 0.01), shape="cylinder",
                exact_z=True, color=GREY, friction=(0.6, 0.5), **_vis("dinner_plate", (0.0, 0.0, -zc)))


def chicken_object(i: int) -> Part:
    x, y = CHICKEN_XY[i]
    return Part(f"chicken{i + 1}", pos=(x, y, TOP + TRAY_FLOOR + CH[2] / 2 + 0.001), size=CH, exact_z=True, mass=CH_MASS,
                color=PINK, friction=(1.0, 0.9), restitution=0.01, contact_offset=0.002,
                **_vis("chicken_a" if i == 0 else "chicken_b", (0.0, 0.0, -CH[2] / 2), visual_scale=CH_S))


def bun_object(i: int) -> Part:
    x, y = BUN_XY[i]
    return Part(f"bun{i + 1}", pos=(x, y, TOP + BASKET_FLOOR + BUN_H / 2 + 0.001), size=(BUN_R, BUN_H, BUN_R), shape="cylinder",
                exact_z=True, mass=BUN_MASS, color=CRUST, friction=(1.0, 0.9), restitution=0.01, contact_offset=0.002,
                **_vis("bun_a" if i == 0 else "bun_b", (0.0, 0.0, -BUN_H / 2), visual_scale=BUN_S))


# -- measurements -----------------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _local(pose, p):
    pose = np.asarray(pose, float)
    return quat_matrix(pose[3:7]).T @ (np.asarray(p, float)[:3] - pose[:3])


def grip_on_chicken(pose, tcp) -> bool:
    """A gripper's grasp point within a chicken piece's outline (plus 1 cm): it is holding or pressing on it."""
    q = _local(pose, tcp)
    return bool(abs(q[0]) <= CH[0] / 2 + 0.010 and abs(q[1]) <= CH[1] / 2 + 0.010 and -0.012 <= q[2] <= CH[2] / 2 + 0.020)


def grip_on_bun(pose, tcp) -> bool:
    q = _local(pose, tcp)
    return bool(math.hypot(q[0], q[1]) <= BUN_R + 0.010 and -BUN_H / 2 - 0.005 <= q[2] <= BUN_H / 2 + 0.020)


def on_pan(p) -> bool:
    p = np.asarray(p, float)
    return bool(abs(p[0] - PAN_C[0]) <= PAN_IN and abs(p[1] - PAN_C[1]) <= PAN_IN and p[2] <= TOP + PAN_RIM + 0.02)


def on_plate(p) -> bool:
    p = np.asarray(p, float)
    return bool(math.hypot(p[0] - PLATE_C[0], p[1] - PLATE_C[1]) <= PLATE_R and p[2] <= TOP + 0.08)


def in_tray(p) -> bool:
    p = np.asarray(p, float)
    return bool(abs(p[0] - TRAY_C[0]) <= TRAY[0] / 2 and abs(p[1] - TRAY_C[1]) <= TRAY[1] / 2 and p[2] <= TOP + 0.08)


def in_basket(p) -> bool:
    p = np.asarray(p, float)
    return bool(abs(p[0] - BASKET_C[0]) <= BASKET_IN[0] + 0.012 and abs(p[1] - BASKET_C[1]) <= BASKET_IN[1] + 0.012
                and p[2] <= TOP + BASKET_RIM + 0.03)


def foods_touch(chicken, bun) -> bool:
    """A fillet and a roll in contact: the roll's axis within the fillet's outline plus the roll's radius, at its height."""
    q = _local(chicken, np.asarray(bun, float)[:3])
    dx = max(abs(q[0]) - CH[0] / 2, 0.0)
    dy = max(abs(q[1]) - CH[1] / 2, 0.0)
    return bool(math.hypot(dx, dy) <= BUN_R + 0.004 and abs(q[2]) <= CH[2] / 2 + BUN_H / 2 + 0.004)


def grip_on_bread_surface(tcp) -> bool:
    """A gripper's grasp point down at the plate, or down inside the bread basket (anywhere below its rim): the
    surfaces the bread lies on or is served from."""
    p = np.asarray(tcp, float)
    at_plate = math.hypot(p[0] - PLATE_C[0], p[1] - PLATE_C[1]) <= PLATE_R + 0.010 and p[2] <= TOP + PLATE_SURF + 0.020
    in_basket_low = (abs(p[0] - BASKET_C[0]) <= BASKET_IN[0] + 0.012 and abs(p[1] - BASKET_C[1]) <= BASKET_IN[1] + 0.012
                     and p[2] <= TOP + BASKET_RIM)
    return bool(at_plate or in_basket_low)


def grip_in_tray(tcp) -> bool:
    """A gripper's grasp point down inside the raw-meat tray (within its outline, at or below 1 cm over its rim):
    the tray's inside is wet with raw juices whether or not a fillet is still in it."""
    p = np.asarray(tcp, float)
    return bool(abs(p[0] - TRAY_C[0]) <= TRAY[0] / 2 and abs(p[1] - TRAY_C[1]) <= TRAY[1] / 2 and p[2] <= TOP + TRAY[2] + 0.010)


def grip_in_pan(tcp) -> bool:
    """A gripper's grasp point down inside the grill pan, below its rim."""
    p = np.asarray(tcp, float)
    return bool(abs(p[0] - PAN_C[0]) <= PAN_IN + 0.004 and abs(p[1] - PAN_C[1]) <= PAN_IN + 0.004 and p[2] <= TOP + PAN_RIM)


HANDS_TOUCH = 0.080                  # two grasp points this close: the grippers (each about 9 cm wide) are touching


def hands_touch(tcp_a, tcp_b) -> bool:
    return bool(np.linalg.norm(np.asarray(tcp_a, float)[:3] - np.asarray(tcp_b, float)[:3]) <= HANDS_TOUCH)
