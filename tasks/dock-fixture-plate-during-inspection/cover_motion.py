"""Joint-space YAM policies for staging, carrying, pausing and pin registration."""

import os
from cover_geometry import (
    MODES,
    TIMING,
    ORIGIN,
    PICK_X,
    LAND_X,
    COVER_Z,
    REST_Z,
    HANDLE_Y,
    HANDLE_HEIGHT,
    SIDES,
)
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint


def demo(mode=None):
    mode = mode or os.environ.get("COVER_CONTROL", "reference")
    if mode in TIMING:
        mode = TIMING[mode][0]
    if mode not in (*MODES, "dock_while_occupied", "wait_then_dock"):
        raise ValueError(mode)
    if mode == "idle":
        return {}
    result = {}
    for side in SIDES:
        sign = 1 if side == "left" else -1
        handle = sign * HANDLE_Y
        reach = []
        h = COVER_Z + HANDLE_HEIGHT
        hover = REST_Z + HANDLE_HEIGHT + 0.089
        seat = REST_Z + HANDLE_HEIGHT - 0.004
        scale = 1.15 if mode == "slow_reference" else 1.0

        def move(t, x, y, z, gap, label):
            pos = tuple(a + b for a, b in zip(ORIGIN, (x, y, z)))
            reach.append(Waypoint(t * scale, pos, gap, label))

        move(
            1.60,
            PICK_X - 0.010,
            sign * 0.184,
            h + 0.050,
            FINGER_OPEN,
            "approach staging bay",
        )
        move(0.95, PICK_X, handle, h + 0.034, -0.026, "align over handle")
        move(1.0, PICK_X, handle, h - 0.006, -0.026, "straddle handle")
        move(0.60, PICK_X, handle, h - 0.006, 0, "grasp")
        move(0.90, PICK_X, handle, hover + 0.012, 0, "lift plate from stage")
        target = LAND_X + (0.032 if mode == "misaligned_hold" else 0.0)
        move(1.85, target, handle, hover, 0, "carry above receiving fixture")
        docking_x = target + 0.009 + sign * 0.005

        def seat_plate():
            move(
                1.45, docking_x, handle - 0.002, seat + 0.006, 0, "engage locating pins"
            )
            move(
                0.55, docking_x, handle - 0.002, seat, 0, "settle onto receiving faces"
            )

        if mode == "dock_while_occupied":
            seat_plate()
            move(0.35, docking_x, handle - 0.002, seat, 0, "maintain grasp")
            move(1.0, target, handle, hover, 0, "retract plate")
            move(7.65, target, handle, hover, 0, "wait above fixture")
        else:
            move(11, target, handle, hover, 0, "wait above fixture")
        if mode != "hover_only":
            seat_plate()
            if mode != "misaligned_hold":
                move(0.60, docking_x, handle - 0.002, seat, FINGER_OPEN, "release")
                move(
                    0.80,
                    target,
                    handle,
                    hover + 0.01,
                    FINGER_OPEN,
                    "clear pins vertically",
                )
                move(1.50, PICK_X + 0.06, sign * 0.305, 0.225, FINGER_OPEN, "park")
                if mode == "park_closed":
                    move(
                        0.55, PICK_X + 0.06, sign * 0.305, 0.225, 0, "close parked jaws"
                    )
        result[side] = reach
    return result
