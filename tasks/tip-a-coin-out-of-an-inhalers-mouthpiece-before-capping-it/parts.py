"""The inhaler task's objects as mechkit shapes: a training inhaler (an L-shaped boot with its canister), its cap and
US dimes. Shapes only; the task file and the checks declare the bodies.

The inhaler's frame I: origin where the mouthpiece's axis meets the barrel's axis; x along the mouthpiece, out of the
mouth; z along the barrel, towards the canister's end; y = z x x, the long axis of the mouthpiece's oval opening.
Lying on its side (the L flat on the bench) I's y axis stands upright. Sizes marked (a) are assumed until measured on
the bought trainer.
"""

from __future__ import annotations

import math

import numpy as np
from mechkit import box, cylinder, matrix_to_quat

# --- the inhaler (a) ---
BARREL_R = 0.015  # (a) the boot's barrel, 30 mm across
BOOT_LOW, BOOT_HIGH = -0.015, 0.045  # (a) the boot along z: from its bottom to its top rim
CAN_R, CAN_LOW, CAN_TOP = 0.011, 0.035, 0.065  # (a) the canister: 22 mm across, its end 80 mm from the boot's bottom
BORE_A, BORE_B = 0.013, 0.010  # (a) the mouthpiece's oval bore, half-widths along y and z: 26 x 20 mm
WALL = 0.0015  # (a) the mouthpiece's wall
MOUTH_X = 0.047  # (a) the mouth, 32 mm out from the barrel's side
BORE_BACK = 0.005  # the staves start inside the barrel: the bore ends at the barrel's side
STAVES = 16  # the bore's wall as this many flat staves (an inscribed polygon: at most 0.25 mm inside the oval)
INHALER_MASS = 0.030  # (a) kg, the boot with an empty training canister
BOOT_RED, CAN_GREY = (0.50, 0.07, 0.09), (0.78, 0.79, 0.81)  # (a) the colours: a dark red boot, an aluminium canister

# --- US dime (US Mint: 17.91 mm across, 1.35 mm thick, 2.268 g) ---
DIME_R, DIME_T, DIME_MASS = 0.01791 / 2, 0.00135, 0.002268
DIME_SILVER = (0.45, 0.47, 0.50)  # a dime's copper-nickel as a mid grey: the renderer has no metal sheen, and a lighter grey drew near white (run w9i2)
COIN_FRICTION = 0.3  # static and dynamic, with min-combine: the coin's friction on the boot's plastic and on the bench
# --- US penny (US Mint: 19.05 mm across, 1.52 mm thick, 2.5 g): the lodged coin, copper against the bench and the boot
PENNY_R, PENNY_T, PENNY_MASS = 0.01905 / 2, 0.00152, 0.0025
PENNY_COPPER = (0.72, 0.43, 0.29)
LODGED_R, LODGED_T, LODGED_MASS = DIME_R, DIME_T, DIME_MASS  # the coin in the mouthpiece: a dime (a penny wedges where the bore's walls are near upright and chatters: run w9i3)
THIN = 0.0005  # m contact offset of the coin and the boot (the kit's 2 mm is more than the coin is thick)

# lying on its side, mouthpiece towards +y: I's x to the world's +y, y up, z (the barrel) to the world's +x
LYING = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])  # columns: I's x, y, z in the world
LYING_QUAT = matrix_to_quat(LYING)


def bore_polygon(n: int = STAVES) -> np.ndarray:
    """The bore's inner wall as a polygon in I's (y, z) plane, counter-clockwise, corners on the oval."""
    phi = 2.0 * math.pi * np.arange(n) / n
    return np.c_[BORE_A * np.cos(phi), BORE_B * np.sin(phi)]


def bore_width_z(y: float, n: int = STAVES) -> float:
    """The bore's inner width along z at height ``y`` (I's y), from the polygon."""
    pts = bore_polygon(n)
    zs = []
    for p, q in zip(pts, np.roll(pts, -1, axis=0)):
        if (p[0] - y) * (q[0] - y) <= 0.0 and p[0] != q[0]:
            zs.append(p[1] + (y - p[0]) / (q[0] - p[0]) * (q[1] - p[1]))
    return float(max(zs) - min(zs)) if len(zs) >= 2 else 0.0


def wedge_height(radius: float = DIME_R) -> float:
    """I's y of a flat coin's lower face where the bore below the middle has narrowed to the coin's width."""
    lo, hi = -BORE_A, 0.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if bore_width_z(mid) < 2.0 * radius else (lo, mid)
    return hi


def inhaler_shapes() -> list:
    """Boot, canister and the mouthpiece's staves, in I's frame."""
    shapes = [cylinder(BARREL_R, BOOT_HIGH - BOOT_LOW, pos=(0.0, 0.0, 0.5 * (BOOT_LOW + BOOT_HIGH)), axis="z", color=BOOT_RED),
              cylinder(CAN_R, CAN_TOP - CAN_LOW, pos=(0.0, 0.0, 0.5 * (CAN_LOW + CAN_TOP)), axis="z", color=CAN_GREY)]  # fmt: skip
    pts = bore_polygon()
    length = MOUTH_X - BORE_BACK
    overlap = 2.0 * WALL * math.tan(math.pi / STAVES) + 0.0004  # closes the wedge between neighbours' outer corners
    for p, q in zip(pts, np.roll(pts, -1, axis=0)):
        t = (q - p) / np.linalg.norm(q - p)
        n = np.array([t[1], -t[0]])  # outwards (the polygon runs counter-clockwise)
        mid = 0.5 * (p + q) + 0.5 * WALL * n
        rot = np.column_stack([[1.0, 0.0, 0.0], [0.0, n[0], n[1]], [0.0, t[0], t[1]]])  # box x: along the bore, y: outwards, z: along the wall
        shapes.append(box((length, WALL, float(np.linalg.norm(q - p)) + overlap), pos=(BORE_BACK + 0.5 * length, mid[0], mid[1]),
                          quat=matrix_to_quat(rot), color=BOOT_RED))  # fmt: skip
    return shapes


def dime_shapes() -> list:
    return [cylinder(DIME_R, DIME_T, axis="z", color=DIME_SILVER)]


def lodged_shapes() -> list:
    return [cylinder(LODGED_R, LODGED_T, axis="z", color=DIME_SILVER)]


def coin_lodged(out: float = 1.0 / 3.0, above: float = 0.0005) -> np.ndarray:
    """I's coordinates of the lodged coin's middle: lying flat across the bore below its middle, ``out`` of its diameter
    past the mouth, ``above`` its resting height (it settles onto the walls)."""
    return np.array([MOUTH_X - LODGED_R + out * 2.0 * LODGED_R, wedge_height(LODGED_R) + 0.5 * LODGED_T + above, 0.0])


# --- the cap (a): an oval cup that goes over the mouthpiece. Its frame C: x out of its opening, y its long axis, z its
# short axis; origin at the middle of the cup. The walls' inner polygon is 0.46 mm out from the mouthpiece's outer oval,
# which leaves 0.3 mm of play at the closest point of the two 16-sided walls (computed in the build notes); the last
# 3 mm at the rim flare out by 1 mm (the lead-in that lines the cap up as it goes on).
CAP_PLAY_OUT = 0.000464  # m added to the mouthpiece's outer half-widths: 0.3 mm play between the two polygons
CAP_IN_A, CAP_IN_B = BORE_A + WALL + CAP_PLAY_OUT, BORE_B + WALL + CAP_PLAY_OUT
CAP_WALL, CAP_DEPTH, CAP_END = 0.0010, 0.020, 0.0015
CAP_OUT_A, CAP_OUT_B = CAP_IN_A + CAP_WALL, CAP_IN_B + CAP_WALL
LEAD, LEAD_LEN = 0.001, 0.003  # m: the lead-in's flare and its length at the rim
CAP_MASS = 0.004  # (a) kg
CAP_COLOR = (0.90, 0.90, 0.88)  # white, as on the maker's distributor's product photo (the boot red, the canister silver)


def oval_staves(a: float, b: float, x0: float, x1: float, thick: float, color, n: int = STAVES, passes=(), flare: float = 0.0) -> list:
    """A tube along x from ``x0`` to ``x1`` whose inner wall is the polygon of ``n`` corners on the oval (a, b) in (y, z);
    with ``flare`` the staves lean out so that the inner wall is ``flare`` further out at ``x1`` than at ``x0``."""
    phi = 2.0 * math.pi * np.arange(n) / n
    pts = np.c_[a * np.cos(phi), b * np.sin(phi)]
    overlap = 2.0 * thick * math.tan(math.pi / n) + 0.0004
    lean = math.atan2(flare, x1 - x0)
    out = []
    for p, q in zip(pts, np.roll(pts, -1, axis=0)):
        t = (q - p) / np.linalg.norm(q - p)
        nrm = np.array([t[1], -t[0]])
        mid = 0.5 * (p + q) + (0.5 * thick + 0.5 * flare) * nrm
        along = np.array([math.cos(lean), math.sin(lean) * nrm[0], math.sin(lean) * nrm[1]])  # x leaning out by ``lean``
        out_n = np.array([-math.sin(lean), math.cos(lean) * nrm[0], math.cos(lean) * nrm[1]])
        rot = np.column_stack([along, out_n, [0.0, t[0], t[1]]])
        length = (x1 - x0) / math.cos(lean)
        out.append(box((length, thick, float(np.linalg.norm(q - p)) + overlap), pos=(0.5 * (x0 + x1), mid[0], mid[1]), quat=matrix_to_quat(rot), color=color, passes=passes))
    return out


def cap_shapes(passes=()) -> list:
    """Walls of 16 staves round the opening with the lead-in at the rim, an end like a stadium (two half-discs and a
    bar) closing the far end. The end meets the mouthpiece's end (the seat) and a coin that sticks out of the mouth."""
    back, rim = -0.5 * CAP_DEPTH, 0.5 * CAP_DEPTH
    shapes = oval_staves(CAP_IN_A, CAP_IN_B, back + CAP_END, rim - LEAD_LEN, CAP_WALL, CAP_COLOR, passes=passes)
    shapes += oval_staves(CAP_IN_A, CAP_IN_B, rim - LEAD_LEN, rim, CAP_WALL, CAP_COLOR, passes=passes, flare=LEAD)
    reach = CAP_OUT_A - CAP_OUT_B
    shapes += [box((CAP_END, 2.0 * reach, 2.0 * CAP_OUT_B), pos=(back + 0.5 * CAP_END, 0.0, 0.0), color=CAP_COLOR)]
    shapes += [cylinder(CAP_OUT_B, CAP_END, pos=(back + 0.5 * CAP_END, s * reach, 0.0), axis="x", color=CAP_COLOR) for s in (-1.0, 1.0)]
    return shapes


# --- the tray: Room & Board "Brando" felt-and-leather tray: 7 x 5.5 x 1.5 in outside (178 x 140 x 38 mm), 5 x 3 x 1 in
# inside (127 x 76 x 25 mm) by the maker's figures: thick leather walls round a felt floor. Frame at the middle of its
# underside. Mass (a).
TRAY_L, TRAY_W, TRAY_H = 0.178, 0.140, 0.038  # outside
TRAY_IN_L, TRAY_IN_W, TRAY_IN_H = 0.127, 0.076, 0.025  # inside
TRAY_FLOOR = TRAY_H - TRAY_IN_H  # 13 mm
TRAY_MASS = 0.250  # (a) kg
TRAY_LEATHER, TRAY_FELT = (0.38, 0.23, 0.14), (0.33, 0.34, 0.36)  # (a)


def tray_shapes() -> list:
    """A floor TRAY_FLOOR thick; two side walls along x and two end walls between them, round the inside."""
    side, end, h = 0.5 * (TRAY_W - TRAY_IN_W), 0.5 * (TRAY_L - TRAY_IN_L), TRAY_IN_H
    return [box((TRAY_L, TRAY_W, TRAY_FLOOR), pos=(0.0, 0.0, 0.5 * TRAY_FLOOR), color=TRAY_FELT),
            *[box((TRAY_L, side, h), pos=(0.0, s * 0.5 * (TRAY_W - side), TRAY_FLOOR + 0.5 * h), color=TRAY_LEATHER) for s in (-1.0, 1.0)],
            *[box((end, TRAY_IN_W, h), pos=(s * 0.5 * (TRAY_L - end), 0.0, TRAY_FLOOR + 0.5 * h), color=TRAY_LEATHER) for s in (-1.0, 1.0)]]  # fmt: skip
