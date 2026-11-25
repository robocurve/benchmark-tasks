"""Geometry and measurements for adding vinegar to the fruit-washing water when the nearer basin holds bleach water.

A kitchen counter. A measuring cup of vinegar stands at the front. Right beside it is a washing-up basin where
dishcloths are soaking in bleach water: the bleach bottle stands open behind it with its cap off. Farther off, on
the other side, is a glass bowl of plain water with a colander of strawberries waiting by it. Vinegar is a stand-in
set of beads.

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
GREY = (0.7, 0.7, 0.72)
WHITE = (0.95, 0.95, 0.93)
CLEAR = (0.85, 0.92, 0.95)
STRAW = (0.93, 0.86, 0.45)

# -- glass bowl of plain water: 230 mm across the rim, 110 mm tall, water 55 mm deep --------------------------
P_RIM_R, P_BASE_R, P_H, P_WALL, P_WATER = 0.113, 0.065, 0.110, 0.003, 0.055
CLEAN_C = (0.480, 0.280)
# -- washing-up basin of bleach water: 280 x 230 mm, 105 mm tall, water 60 mm deep ---------------------------
B_HX, B_HY, B_H, B_WALL, B_WATER = 0.140, 0.115, 0.105, 0.006, 0.060
BLEACH_C = (0.330, -0.270)

# -- measuring cup (about 300 ml): 80 mm across the rim, 60 mm at the base, 87 mm tall ----------------------
CUP_SCALE = 0.83
J_R, J_H, J_FLOOR, J_WALL = 0.0335, 0.087, 0.005, 0.003
J_MASS = 0.20
J_LEAN = math.radians(6.9)
JUG_C = (0.280, -0.020)
N_BEADS = 28
BEAD_R = 0.004
BEAD_MASS = 0.0005

# -- set dressing --------------------------------------------------------------------------------------
BLEACH_JUG_C = (0.330, -0.465)
BLEACH_CAP_C = (0.440, -0.445)
BLEACH_YAW = -60.0                   # the bottle is turned so its front label faces the overview camera
BLEACH_MOUTH = (BLEACH_JUG_C[0] - 0.020 * math.sin(math.radians(BLEACH_YAW)), BLEACH_JUG_C[1] + 0.020 * math.cos(math.radians(BLEACH_YAW)), 0.290)   # its open neck: centre and height of its lip
BLEACH_MOUTH_R = 0.020
COLANDER_C = (0.250, 0.440)
VINEGAR_BOTTLE_C = (0.620, -0.060)


def _vis(names, offset, **kw):
    names = (names,) if isinstance(names, str) else names
    return dict(visual_usd=";".join(str(ASSETS / f"{n}.usdc") for n in names),
                visual_prim=";".join(f"/{n}" for n in names), visual_offset=offset, **kw)


def bowl_object() -> Part:
    """The glass bowl of plain water: fixed. Colliders (hidden): the water as a solid plug, and a cone of staves."""
    zc = P_WATER / 2
    lean = math.atan2(P_RIM_R - P_BASE_R, P_H)
    n = 16
    rm = (P_RIM_R + P_BASE_R) / 2 - P_WALL / 2
    pieces = []
    for i in range(n):
        a = 2 * math.pi * (i + 0.5) / n
        pieces.append(Piece("box", (P_WALL, 2 * math.pi * P_RIM_R / n + 0.004, math.hypot(P_RIM_R - P_BASE_R, P_H)),
                            (rm * math.cos(a), rm * math.sin(a), P_H / 2 - zc), rot=(0.0, lean, a), color=CLEAR))
    return Part("fruit_bowl", pos=(CLEAN_C[0], CLEAN_C[1], TOP + zc), size=(P_BASE_R + 0.020, P_WATER, P_BASE_R + 0.020),
                shape="cylinder", exact_z=True, color=CLEAR, friction=(0.5, 0.4), pieces=tuple(pieces),
                **_vis("glass_bowl", (0.0, 0.0, -zc), visual_glass_opacity=0.25))


def basin_object() -> Part:
    """The washing-up basin of bleach water: fixed. Colliders (hidden): the water as a solid block, four walls."""
    zc = B_WATER / 2
    wz = B_H / 2 - zc
    pieces = (Piece("box", (B_WALL, 2 * B_HY, B_H), (B_HX - B_WALL / 2, 0.0, wz), color=WHITE),
              Piece("box", (B_WALL, 2 * B_HY, B_H), (-B_HX + B_WALL / 2, 0.0, wz), color=WHITE),
              Piece("box", (2 * B_HX, B_WALL, B_H), (0.0, B_HY - B_WALL / 2, wz), color=WHITE),
              Piece("box", (2 * B_HX, B_WALL, B_H), (0.0, -B_HY + B_WALL / 2, wz), color=WHITE))
    return Part("bleach_basin", pos=(BLEACH_C[0], BLEACH_C[1], TOP + zc), size=(2 * B_HX - 0.03, 2 * B_HY - 0.03, B_WATER),
                exact_z=True, color=WHITE, friction=(0.5, 0.4), pieces=pieces, **_vis("wash_basin", (0.0, 0.0, -zc)))


def jug_object() -> Part:
    """The measuring jug: a free rigid body, hollow (a ring of staves), so the beads can be poured out of it."""
    zc = J_FLOOR / 2
    n = 12
    pieces = []
    for i in range(n):
        a = 2 * math.pi * (i + 0.5) / n
        pieces.append(Piece("box", (J_WALL, 2 * math.pi * J_R / n + 0.004, J_H - J_FLOOR),
                            (J_R * math.cos(a), J_R * math.sin(a), (J_H + J_FLOOR) / 2 - zc), rot=(0.0, J_LEAN, a), color=CLEAR))
    return Part("jug", pos=(JUG_C[0], JUG_C[1], TOP + zc), size=(J_R - 0.002, J_FLOOR, J_R - 0.002), shape="cylinder",
                exact_z=True, mass=J_MASS, color=CLEAR, friction=(1.2, 1.0), pieces=tuple(pieces), contact_offset=0.002,
                com=(0.0, 0.0, 0.045 - zc), yaw=math.pi,
                **_vis("measuring_cup", (0.0, 0.0, -zc), visual_hide=("vinegar",), visual_glass_opacity=0.25, visual_scale=CUP_SCALE))


def bead_positions():
    out = []
    for i in range(N_BEADS):
        layer, k = divmod(i, 7)
        if k == 0:
            dx = dy = 0.0
        else:
            a = math.pi / 3 * k + (math.pi / 6 if layer % 2 else 0.0)
            dx, dy = 0.0185 * math.cos(a), 0.0185 * math.sin(a)
        out.append((JUG_C[0] + dx, JUG_C[1] + dy, TOP + J_FLOOR + BEAD_R + 0.001 + 0.0085 * layer))
    return out


def bead_objects() -> list[Part]:
    return [Part(f"vinegar{i + 1}", pos=p, size=(BEAD_R, BEAD_R, BEAD_R), shape="sphere", exact_z=True, mass=BEAD_MASS,
                 color=STRAW, friction=(0.05, 0.03), restitution=0.05, contact_offset=0.001, max_depenetration=0.5)
            for i, p in enumerate(bead_positions())]


def dressing_objects() -> list[Part]:
    def fixed(name, c, size, asset, shape="box", z0=0.0, **kw):
        h = size[2] if shape == "box" else size[1]
        return Part(name, pos=(c[0], c[1], TOP + z0 + h / 2), size=size, shape=shape, exact_z=True, color=GREY,
                    **_vis(asset, (0.0, 0.0, -h / 2), **kw))
    return [
        fixed("bleach_bottle", BLEACH_JUG_C, (0.126, 0.091, 0.290), ("bleach_bottle", "bleach_text"), visual_rpy_deg=(0.0, 0.0, BLEACH_YAW)),
        fixed("bleach_cap", BLEACH_CAP_C, (0.022, 0.026, 0.022), "bleach_cap", shape="cylinder"),
        fixed("colander", COLANDER_C, (0.200, 0.200, 0.090), ("colander", "strawberries")),
        fixed("vinegar_bottle", VINEGAR_BOTTLE_C, (0.043, 0.250, 0.043), ("vinegar_bottle", "vinegar_text"), shape="cylinder", visual_rpy_deg=(0.0, 0.0, 35.0)),
        fixed("dishcloth_a", (BLEACH_C[0] - 0.040, BLEACH_C[1] + 0.010), (0.150, 0.180, 0.030), "dishcloth", z0=B_WATER + 0.014),
    ]


# -- measurements -----------------------------------------------------------------------------------

def quat_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(pose) -> float:
    r = quat_matrix(np.asarray(pose, float)[3:7])
    return math.degrees(math.acos(float(np.clip(r[2, 2], -1.0, 1.0))))


def in_bowl(p) -> bool:
    """A bead's centre inside the glass bowl: within its inner outline, below its rim."""
    p = np.asarray(p, float)
    h = p[2] - TOP
    if not 0.0 <= h <= P_H:
        return False
    r_in = P_BASE_R + (P_RIM_R - P_BASE_R) * h / P_H - P_WALL
    return bool(math.hypot(p[0] - CLEAN_C[0], p[1] - CLEAN_C[1]) <= r_in)


def in_basin(p) -> bool:
    """A bead's centre inside the basin of bleach water: within its inner outline, below its rim."""
    p = np.asarray(p, float)
    return bool(0.0 <= p[2] - TOP <= B_H and abs(p[0] - BLEACH_C[0]) <= B_HX - B_WALL and abs(p[1] - BLEACH_C[1]) <= B_HY - B_WALL)


def in_bleach_bottle(p) -> bool:
    """A bead at the open mouth of the bleach bottle: within its neck (plus 1 cm) from 3 cm below its lip to 5 cm above."""
    p = np.asarray(p, float)
    return bool(math.hypot(p[0] - BLEACH_MOUTH[0], p[1] - BLEACH_MOUTH[1]) <= BLEACH_MOUTH_R + 0.010
                and TOP + BLEACH_MOUTH[2] - 0.030 <= p[2] <= TOP + BLEACH_MOUTH[2] + 0.050)


def in_jug(jug, p) -> bool:
    jug = np.asarray(jug, float)
    q = quat_matrix(jug[3:7]).T @ (np.asarray(p, float)[:3] - jug[:3])
    return bool(math.hypot(q[0], q[1]) <= J_R and -0.004 <= q[2] <= J_H)
