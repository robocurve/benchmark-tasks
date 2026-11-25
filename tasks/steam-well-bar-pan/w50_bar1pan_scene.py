"""Scene for the bar + one quarter pan steam-well task: stand, warmer, adaptor bar and ONE quarter pan.

Import after labkit.floor_bay(). Geometry is copied from task_w50_4pan_phase_ready.py (bar uses the darker render
colour of the bar-only task); only the first quarter pan exists.
"""
from __future__ import annotations

import os

import numpy as np

os.environ.setdefault("SW_WCX", "0.48")
from labkit import part  # noqa: E402
import scene_w50 as SW  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402

SCENE = "station"
STEEL = SW.STEEL
BAR_STEEL = (0.42, 0.43, 0.46)  # darker brushed stainless for the bar, render colour only
PAN_BEAD = (0.79, 0.80, 0.82)   # rolled pan rim, visual only
# v2 food: macaroni and cheese, a rigid proxy filling the pan to FOOD_FILL of its depth (no spill physics)
FOOD = os.environ.get("W50_FOOD", "1") == "1"
FOOD_FILL = float(os.environ.get("W50_FOOD_FILL", "0.012"))     # 12 mm layer (a 42 mm two-thirds fill is too heavy for the slide)
FOOD_KG = float(os.environ.get("W50_FOOD_KG", "0.28"))          # 12 mm x 128 x 230 mm = 0.35 L at ~0.8 kg/L
CHEESE = (0.93, 0.70, 0.22)
MACARONI = (0.99, 0.83, 0.38)
import labkit as _lk  # noqa: E402
_lk.METAL[BAR_STEEL] = (1.0, 0.35)
_lk.METAL[PAN_BEAD] = (1.0, 0.25)
_lk.VISUAL_ONLY.add(PAN_BEAD)
_lk.VISUAL_ONLY.add(MACARONI)
STAND, WARMER, RIM, WC, ST = SW.STAND, SW.WARMER, SW.RIM, SW.WC, SW.STAND_TOP
SHEET = 0.0020
BAR_W, BAR_L, BAR_H = 0.0254, 0.5310, 0.0080
Q_W, Q_L, Q_D, Q_FL = 0.1618, 0.2635, 0.0635, 0.0150
BODY_W, BODY_L = Q_W - 2.0 * Q_FL, Q_L - 2.0 * Q_FL
BODY_HALF_X = BODY_W / 2.0

X_STRIP = 0.2130
Y_STAGE_ABS = 0.4675
X_FAR_SEAT = 0.5625
Y_POS_SEAT = +0.1330


def _pan_boxes(top_w, top_l, depth, flange):
    bw, bl = top_w - 2 * flange, top_l - 2 * flange
    b = [((0, 0, SHEET / 2), (bw, bl, SHEET), STEEL)]
    for sx in (-1, 1):
        b.append(((sx * (bw / 2 - SHEET / 2), 0, depth / 2), (SHEET, bl, depth), STEEL))
    for sy in (-1, 1):
        b.append(((0, sy * (bl / 2 - SHEET / 2), depth / 2), (bw, SHEET, depth), STEEL))
    for sx in (-1, 1):
        b.append(((sx * (top_w / 2 - flange / 2), 0, depth - SHEET / 2), (flange, top_l, SHEET), STEEL))
    for sy in (-1, 1):
        b.append(((0, sy * (top_l / 2 - flange / 2), depth - SHEET / 2), (top_w - 2 * flange, flange, SHEET), STEEL))
    return tuple(b)


def _food_boxes(top_w, top_l, flange):
    """the cheese-sauce block: fills the pan body inside the walls up to FOOD_FILL (collider, part of the pan body)"""
    if not FOOD:
        return ()
    bw, bl = top_w - 2 * flange - 2 * SHEET - 0.001, top_l - 2 * flange - 2 * SHEET - 0.001
    return (((0, 0, SHEET + FOOD_FILL / 2), (bw, bl, FOOD_FILL), CHEESE),)


def _macaroni(top_w, top_l, flange):
    """visual-only elbow pieces half sunk in the surface (deterministic layout)"""
    if not FOOD:
        return ()
    bw, bl = top_w - 2 * flange - 2 * SHEET - 0.012, top_l - 2 * flange - 2 * SHEET - 0.012
    z = SHEET + FOOD_FILL
    out = []
    for i in range(7):
        for j in range(5):
            x = -bw / 2 + (j + 0.5 + 0.3 * ((i % 2) - 0.5)) * bw / 5
            y = -bl / 2 + (i + 0.5) * bl / 7
            out.append(((x, y, z), 0.0035, 0.012, MACARONI, "X" if (i + j) % 2 else "Y"))
    return tuple(out)


def _pan_bead(top_w, top_l, depth):
    """visual-only rolled rim: 1.5 mm round bead just inside the flange's outer edge (inside the collider extent)"""
    r = 0.0015
    z = depth - r
    return ((( top_w / 2 - r, 0, z), r, top_l - 2 * r, PAN_BEAD, "Y"), ((-top_w / 2 + r, 0, z), r, top_l - 2 * r, PAN_BEAD, "Y"),
            ((0, top_l / 2 - r, z), r, top_w - 2 * r, PAN_BEAD, "X"), ((0, -top_l / 2 + r, z), r, top_w - 2 * r, PAN_BEAD, "X"))


BAR = part(
    "adaptor_bar",
    pos=(X_STRIP, 0.0000, ST + 0.0005),
    mass=0.136,
    friction=0.5,
    color=BAR_STEEL,
    boxes=(((0.0, 0.0, BAR_H / 2), (BAR_W, BAR_L, BAR_H), BAR_STEEL),),
)
PAN_FRONT = os.environ.get("W50_PAN_START", "deck") == "front"
PAN_Y0 = float(os.environ.get("W50_PAN_Y0", "0.410")) if PAN_FRONT else Y_STAGE_ABS
Q1 = part("quarter_1", pos=(X_STRIP, +PAN_Y0, ST + 0.0005), mass=0.453 + (FOOD_KG if FOOD else 0.0), friction=0.4,
          color=STEEL, boxes=_pan_boxes(Q_W, Q_L, Q_D, Q_FL) + _food_boxes(Q_W, Q_L, Q_FL),
          cylinders=_pan_bead(Q_W, Q_L, Q_D) + _macaroni(Q_W, Q_L, Q_FL))

_down_grip = _demo.grip_rotation
if not getattr(_down_grip, "_orient", False):
    def _grip_rotation(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _down_grip(yaw)
    _grip_rotation._orient = True
    _demo.grip_rotation = _grip_rotation


def hand_pts_from_link_poses(lposes_L: dict, lposes_R: dict) -> np.ndarray:
    """Gripper + camera + both fingertip point covers of both hands, in env-local world coordinates."""
    from hand_cover_data import GRIPPER_D405_COVER_LOCAL, TIP_LEFT_COVER_NATIVE, TIP_RIGHT_COVER_NATIVE

    pts = []
    for lp in (lposes_L, lposes_R):
        for lname, cloud in (("gripper", GRIPPER_D405_COVER_LOCAL), ("tip_left", TIP_LEFT_COVER_NATIVE),
                             ("tip_right", TIP_RIGHT_COVER_NATIVE)):
            T = lp[lname]
            pts.append((T[:3, :3] @ cloud.T).T + T[:3, 3])
    return np.vstack(pts)
