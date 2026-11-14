"""Scripted demo for receipt_printer_roll_reload_v2. The left arm grips the front lip of the hinged cover and swings it
open to 85 degrees, holding it against its return spring; the right arm pinches the roll across its 58 mm width
(yaw 90, axis along x, free end to the right), lifts it, carries it in front of the printer while turning a quarter turn
(yaw 90 to 0) so the free end swings toward -x, moves it over the open bay, lowers it onto the seat, lets go and withdraws;
then the left arm releases the lid so it springs closed flat and withdraws. Every working point is within reach limits
(TCP x <= 0.30, z <= 0.19 above table top, hands >= 125 mm apart in y)."""

from __future__ import annotations

import math
import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

from geometry import (
    BAY_OPEN_DEG,
    COVER_CLOSED_TOL_DEG,
    HINGE_AT,
    HOLD_OPEN_DEG,
    LIP_LOCAL_AT,
    PRINTER_XY,
    ROLL_START_CENTRE_Z,
    ROLL_START_XY,
    ROLL_WIDTH_M,
    SEAT_CENTRE_Z,
    SEAT_XY,
    TABLE_TOP,
)

TOP = TABLE_TOP
SCALE = float(os.getenv("DEMO_TIME_SCALE", "1.0"))
YAW_ACROSS = 1.5708          # pads close along world x: across the roll's 58 mm width
FINGER_GRIP = -(ROLL_WIDTH_M - 0.004) / 2


def _s(t: float) -> float:
    return max(0.1, t * SCALE)


L_READY = (0.10, 0.14, TOP + 0.12)
R_READY = (0.10, -0.14, TOP + 0.12)

# Hinge axis in world frame
HX = PRINTER_XY[0] + HINGE_AT[0]   # 0.275
HY = PRINTER_XY[1] + HINGE_AT[1]   # 0.000
HZ = TOP + HINGE_AT[2]              # TOP + 0.070

# Front lip in joint frame at rest
LIP_DX = LIP_LOCAL_AT[0]            # -0.100
LIP_DY = LIP_LOCAL_AT[1]            # 0.130
LIP_DZ = LIP_LOCAL_AT[2]            # 0.018


def _lip_world(theta_deg: float) -> tuple[float, float, float]:
    """World position of the front lip when the lid is open by theta_deg (rotation around +y by +theta)."""
    t = math.radians(theta_deg)
    # Rotation about +y by +t:
    # x' = LIP_DX * cos(t) + LIP_DZ * sin(t)
    # z' = -LIP_DX * sin(t) + LIP_DZ * cos(t)
    x_rel = LIP_DX * math.cos(t) + LIP_DZ * math.sin(t)
    z_rel = -LIP_DX * math.sin(t) + LIP_DZ * math.cos(t)
    return (HX + x_rel, HY + LIP_DY, HZ + z_rel)


# Right arm parameters
GRIP_DZ = 0.008
GRASP = (ROLL_START_XY[0], ROLL_START_XY[1], TOP + ROLL_START_CENTRE_Z + GRIP_DZ)        # 0.051 above table
CARRY_Z = TOP + 0.160
HOVER_ROLL = (ROLL_START_XY[0], ROLL_START_XY[1], CARRY_Z)
IN_FRONT = (0.14, 0.0, CARRY_Z)
OVER_BAY = (SEAT_XY[0], SEAT_XY[1], CARRY_Z)
SEAT_LOW = (SEAT_XY[0], SEAT_XY[1], TOP + SEAT_CENTRE_Z + GRIP_DZ + 0.002)


def make_settle_plan() -> dict[str, list[Waypoint]]:
    return {
        "left": [Waypoint(_s(8.0), L_READY, FINGER_OPEN, "left settle")],
        "right": [Waypoint(_s(8.0), R_READY, FINGER_OPEN, "right settle")],
    }


def _left_hold_and_release():
    def w(t, p, f, label):
        return Waypoint(_s(t), p, f, label)

    lip_0 = _lip_world(0.0)
    hover_lip = (lip_0[0], lip_0[1], TOP + 0.180)
    p_open = _lip_world(HOLD_OPEN_DEG)

    return [
        w(1.0, L_READY, FINGER_OPEN, "hold ready pose"),
        w(2.0, hover_lip, FINGER_OPEN, "approach hover over lip tab"),
        w(2.5, hover_lip, FINGER_OPEN, "hold hover over lip tab"),
        w(2.0, lip_0, FINGER_OPEN, "descend onto lip tab"),
        w(0.8, lip_0, FINGER_CLOSED, "grip front lip tab"),
        w(1.0, _lip_world(20.0), FINGER_CLOSED, "lift lid to 20 deg"),
        w(1.0, _lip_world(40.0), FINGER_CLOSED, "lift lid to 40 deg"),
        w(1.0, _lip_world(60.0), FINGER_CLOSED, "lift lid to 60 deg (bay open)"),
        w(1.2, p_open, FINGER_CLOSED, f"lift lid to {HOLD_OPEN_DEG} deg"),
        w(7.0, p_open, FINGER_CLOSED, "hold lid open while roll is loaded"),
        w(0.8, p_open, FINGER_OPEN, "release front lip"),
        w(2.0, p_open, FINGER_OPEN, "wait for lid to spring close"),
        w(1.2, (p_open[0], p_open[1], TOP + 0.180), FINGER_OPEN, "lift clear"),
        w(2.0, L_READY, FINGER_OPEN, "back to ready"),
        w(4.0, L_READY, FINGER_OPEN, "wait for stillness"),
    ]


def _right_load():
    def w(t, p, f, label, yaw=0.0):
        return Waypoint(_s(t), p, f, label, yaw=yaw)

    return [
        w(1.0, R_READY, FINGER_OPEN, "hold ready pose", 0.0),
        w(2.0, HOVER_ROLL, FINGER_OPEN, "hover over roll", YAW_ACROSS),
        w(2.5, HOVER_ROLL, FINGER_OPEN, "hold hover over roll", YAW_ACROSS),
        w(2.0, GRASP, FINGER_OPEN, "descend beside roll ends", YAW_ACROSS),
        w(0.8, GRASP, FINGER_GRIP, "pinch roll across width", YAW_ACROSS),
        w(1.2, HOVER_ROLL, FINGER_GRIP, "lift roll clear of chock", YAW_ACROSS),
        w(1.0, (0.14, -0.14, CARRY_Z), FINGER_GRIP, "move forward clear of bases", YAW_ACROSS),
        w(2.0, IN_FRONT, FINGER_GRIP, "carry in front of notch with quarter turn", 0.0),
        w(1.5, OVER_BAY, FINGER_GRIP, "move roll over open bay", 0.0),
        w(1.5, SEAT_LOW, FINGER_GRIP, "lower roll onto seat", 0.0),
        w(0.4, SEAT_LOW, FINGER_GRIP, "settle in seat", 0.0),
        w(0.8, SEAT_LOW, FINGER_OPEN, "let go", 0.0),
        w(1.2, OVER_BAY, FINGER_OPEN, "lift straight up", 0.0),
        w(2.0, (R_READY[0], R_READY[1], CARRY_Z), FINGER_OPEN, "back toward ready high", 0.0),
        w(1.0, R_READY, FINGER_OPEN, "down to ready", 0.0),
        w(8.6, R_READY, FINGER_OPEN, "wait for lid to close and everything to stop", 0.0),
    ]


def make_demo_plan(ablation: str | None = None) -> dict[str, list[Waypoint]]:
    left = _left_hold_and_release()
    right = _right_load()
    total_l = sum(w.duration for w in left)
    total_r = sum(w.duration for w in right)
    if ablation == "left":      # left arm never touches the lid: the roll cannot be seated
        left = [Waypoint(total_l, L_READY, FINGER_OPEN, "ablation: left arm idle")]
    elif ablation == "right":   # right arm never touches the roll: lid is held open and nothing is loaded
        right = [Waypoint(total_r, R_READY, FINGER_OPEN, "ablation: right arm idle")]
    return {"left": left, "right": right}


def plans():
    if os.getenv("SETTLE_ONLY"):
        return make_settle_plan()
    if os.getenv("ABLATION"):
        return make_demo_plan(ablation="right" if os.getenv("ABLATION") == "right" else "left")
    return make_demo_plan()
