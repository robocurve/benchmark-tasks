"""Bimanual demonstration plans and ablations for lockout-tagout-before-service.

Uses yam_arena.demo.Waypoint.
"""

from __future__ import annotations

import os
from dataclasses import replace

import numpy as np

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_GRASP = -0.005          # firm grip on 16 mm pin
FINGER_HANDLE = -0.014         # firm grip on the 45 mm cover handle
FINGER_OPEN_VAL = FINGER_OPEN  # fully open (-0.04695)
FINGER_PRESS = 0.0             # closed fingers for button/lever


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_COVER_FIRST") == "1":
        return "cover_first"
    if os.environ.get("DEMO_NO_PIN") == "1":
        return "no_pin"
    if os.environ.get("DEMO_NO_TEST") == "1":
        return "no_test"
    if os.environ.get("DEMO_NO_HOLD") == "1":
        return "no_hold"
    if os.environ.get("DEMO_ORDER_SWAPPED") == "1":
        return "order_swapped"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    insert_dz = _f("DEMO_INSERT_DZ", 0.0)
    insert_dy = _f("DEMO_INSERT_DY", 0.0)
    overshoot_x = _f("DEMO_OVERSHOOT_X", 0.012)  # 12 mm overshoot for lever push

    hover_z = g.HOVER_Z
    press_z = g.PRESS_Z
    pin_grasp_z = g.GRASP_PIN_Z + g.PIN_GRASP_DZ
    pin_seated_z = g.PIN_SEATED_Z + g.PIN_GRASP_DZ + insert_dz

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    lever_start = (g.LEVER_START_X, g.LEVER_Y, g.LEVER_Z)
    lever_off = (g.LEVER_OFF_X + overshoot_x, g.LEVER_Y, g.LEVER_Z)

    pin_stand = (g.PIN_STAND_POS[0], g.PIN_STAND_POS[1], pin_grasp_z)
    pin_hole = (g.HOLE_POS[0], g.HOLE_POS[1] + insert_dy, pin_seated_z)

    test_btn = (g.TEST_BTN_POS[0], g.TEST_BTN_POS[1], press_z)

    cover_handle_closed = (g.COVER_HANDLE_X, g.COVER_Y, g.COVER_HANDLE_Z)
    cover_handle_open = (g.COVER_HANDLE_X - g.COVER_TRAVEL, g.COVER_Y, g.COVER_HANDLE_Z)
    service_btn = (g.SERVICE_BTN_POS[0], g.SERVICE_BTN_POS[1], press_z)

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def sync_arms():
        t_l = sum(w.duration for w in left)
        t_r = sum(w.duration for w in right)
        if t_l < t_r:
            left.append(Waypoint(t_r - t_l, left[-1].pos, left[-1].finger, "l: sync wait", left[-1].yaw))
        elif t_r < t_l:
            right.append(Waypoint(t_l - t_r, right[-1].pos, right[-1].finger, "r: sync wait", right[-1].yaw))

    if mode == "settle":
        left.append(Waypoint(14.4, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        right.append(Waypoint(14.4, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # === STEP 1: INITIAL STANCE ===
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    # If cover_first ablation: open cover immediately -> FAULT!
    if mode == "cover_first":
        left.append(Waypoint(0.6, (cover_handle_closed[0], cover_handle_closed[1], hover_z), FINGER_OPEN_VAL, "l: hover cover", 0.0))
        left.append(Waypoint(0.5, cover_handle_closed, FINGER_OPEN_VAL, "l: align cover", 0.0))
        left.append(Waypoint(0.4, cover_handle_closed, FINGER_HANDLE, "l: grasp cover", 0.0))
        left.append(Waypoint(0.8, cover_handle_open, FINGER_HANDLE, "l: pull cover open fault", 0.0))
        left.append(Waypoint(0.5, (cover_handle_open[0], cover_handle_open[1], hover_z), FINGER_OPEN_VAL, "l: lift cover", 0.0))
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 2: SWITCH BREAKER OFF (Left arm) ===
    if mode != "order_swapped":
        left.append(Waypoint(0.6, (lever_start[0] - 0.040, lever_start[1], hover_z), FINGER_PRESS, "l: hover lever", 0.0))
        left.append(Waypoint(0.5, (lever_start[0] - 0.040, lever_start[1], lever_start[2]), FINGER_PRESS, "l: align lever", 0.0))
        # Push forward along +X to OFF
        left.append(Waypoint(0.8, lever_off, FINGER_PRESS, "l: push lever off", 0.0))
        left.append(Waypoint(0.5, (lever_off[0], lever_off[1], hover_z), FINGER_OPEN_VAL, "l: lift lever", 0.0))
    sync_arms()

    # === STEP 3: PICK PIN & SEAT IN BREAKER (Left arm) ===
    if mode != "no_pin":
        left.append(Waypoint(0.6, (pin_stand[0], pin_stand[1], hover_z), FINGER_OPEN_VAL, "l: hover pin", 0.0))
        left.append(Waypoint(0.5, pin_stand, FINGER_OPEN_VAL, "l: descend pin", 0.0))
        left.append(Waypoint(0.4, pin_stand, FINGER_GRASP, "l: grasp pin", 0.0))
        left.append(Waypoint(0.5, (pin_stand[0], pin_stand[1], hover_z), FINGER_GRASP, "l: lift pin", 0.0))
        # Move to hole
        left.append(Waypoint(0.6, (pin_hole[0], pin_hole[1], hover_z), FINGER_GRASP, "l: hover hole", 0.0))
        left.append(Waypoint(0.6, pin_hole, FINGER_GRASP, "l: insert pin", 0.0))
        left.append(Waypoint(0.4, pin_hole, FINGER_OPEN_VAL, "l: release pin", 0.0))
        left.append(Waypoint(0.5, (pin_hole[0], pin_hole[1], hover_z), FINGER_OPEN_VAL, "l: clear pin", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
    sync_arms()

    # If order_swapped: attempted pin before breaker OFF -> hole was blocked
    if mode == "order_swapped":
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 4: PRESS TEST BUTTON (Right arm) ===
    if mode != "no_test":
        right.append(Waypoint(0.6, (test_btn[0], test_btn[1], hover_z), FINGER_PRESS, "r: hover test", 0.0))
        right.append(Waypoint(0.5, test_btn, FINGER_PRESS, "r: press test", 0.0))
        right.append(Waypoint(0.5, (test_btn[0], test_btn[1], hover_z), FINGER_OPEN_VAL, "r: release test", 0.0))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))
    sync_arms()

    # === STEP 5: HOLD COVER & PRESS SERVICE BUTTON ===
    # Left arm hooks and pulls cover open 90 mm (total 1.9 s)
    left.append(Waypoint(0.6, (cover_handle_closed[0], cover_handle_closed[1], hover_z), FINGER_OPEN_VAL, "l: hover cover", 0.0))
    left.append(Waypoint(0.5, cover_handle_closed, FINGER_OPEN_VAL, "l: align cover", 0.0))
    left.append(Waypoint(0.4, cover_handle_closed, FINGER_HANDLE, "l: grasp cover", 0.0))
    left.append(Waypoint(0.8, cover_handle_open, FINGER_HANDLE, "l: pull cover open", 0.0))

    # Right arm waits for cover to be fully open (1.9 s)
    right.append(Waypoint(2.3, home_r, FINGER_OPEN_VAL, "r: wait cover open", 0.0))

    hold_duration = 0.2 if mode == "no_hold" else 2.8
    left.append(Waypoint(hold_duration, cover_handle_open, FINGER_HANDLE, "l: hold cover open", 0.0))

    if mode == "no_hold":
        left.append(Waypoint(0.5, (cover_handle_open[0], cover_handle_open[1], hover_z), FINGER_OPEN_VAL, "l: release cover early", 0.0))
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # Right arm presses SERVICE button under open cover
    right.append(Waypoint(0.6, (service_btn[0], service_btn[1], hover_z), FINGER_PRESS, "r: hover service", 0.0))
    right.append(Waypoint(0.5, service_btn, FINGER_PRESS, "r: press service", 0.0))
    right.append(Waypoint(1.2, service_btn, FINGER_PRESS, "r: hold service 1s", 0.0))
    right.append(Waypoint(0.5, (service_btn[0], service_btn[1], hover_z), FINGER_OPEN_VAL, "r: release service", 0.0))
    right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))

    # Left arm guides cover closed and returns home
    left.append(Waypoint(0.8, cover_handle_closed, FINGER_HANDLE, "l: close cover", 0.0))
    left.append(Waypoint(0.5, (cover_handle_closed[0], cover_handle_closed[1], hover_z), FINGER_OPEN_VAL, "l: release cover", 0.0))
    left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
    sync_arms()

    # === STEP 6: FINAL SETTLE ===
    left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))

    return {"left": left, "right": right}


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` with every waypoint duration stretched by DEMO_TIME_SCALE (default 1.8) so the arms keep up."""
    if mode is None:
        mode = mode_from_env()
    result = _plans(mode)
    if mode in ("settle", "dream"):
        return result
    scale = _f("DEMO_TIME_SCALE", 1.8)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
