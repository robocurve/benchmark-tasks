"""Bimanual demonstration plans and ablations for estop-before-clearing-jam.

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
FINGER_GRASP = -0.012          # firm grip on 40 mm block
FINGER_OPEN_VAL = FINGER_OPEN  # fully open (-0.04695)
FINGER_PRE_BLOCK = -0.030      # 60 mm jaw gap: pre-shape to clear 40 mm block
FINGER_PRESS = 0.0             # closed fingers for button pressing


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_ESTOP") == "1":
        return "no_estop"
    if os.environ.get("DEMO_START_EARLY") == "1":
        return "start_early"
    if os.environ.get("DEMO_SKIP_RESET") == "1":
        return "skip_reset"
    if os.environ.get("DEMO_LEAVE_STOPPED") == "1":
        return "leave_stopped"
    if os.environ.get("DEMO_DREAM") == "1":
        return "dream"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    hover_z = g.HOVER_Z
    lift_clear_z = g.LIFT_CLEAR_Z
    press_tcp_z = g.BUTTON_PRESS_TCP_Z
    grasp_tcp_z = g.GRASP_BLOCK_TCP_Z
    tray_place_tcp_z = g.TRAY_PLACE_TCP_Z

    home_l = g.HOME_L
    home_r = g.HOME_R

    estop_hover = (g.ESTOP_BTN_POS_XY[0], g.ESTOP_BTN_POS_XY[1], hover_z)
    estop_press = (g.ESTOP_BTN_POS_XY[0], g.ESTOP_BTN_POS_XY[1], press_tcp_z)

    reset_hover = (g.RESET_BTN_POS_XY[0], g.RESET_BTN_POS_XY[1], hover_z)
    reset_press = (g.RESET_BTN_POS_XY[0], g.RESET_BTN_POS_XY[1], press_tcp_z)

    start_hover = (g.START_BTN_POS_XY[0], g.START_BTN_POS_XY[1], hover_z)
    start_press = (g.START_BTN_POS_XY[0], g.START_BTN_POS_XY[1], press_tcp_z)

    block_hover = (g.BLOCK_INIT_POS[0], g.BLOCK_INIT_POS[1], hover_z)
    block_grasp = (g.BLOCK_INIT_POS[0], g.BLOCK_INIT_POS[1], grasp_tcp_z)

    tray_hover = (g.TRAY_POS[0], g.TRAY_POS[1], lift_clear_z)
    tray_place = (g.TRAY_POS[0], g.TRAY_POS[1], tray_place_tcp_z)

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def sync_arms():
        t_l = sum(w.duration for w in left)
        t_r = sum(w.duration for w in right)
        if t_l < t_r:
            left.append(Waypoint(t_r - t_l, left[-1].pos, left[-1].finger, "l: sync wait", left[-1].yaw))
        elif t_r < t_l:
            right.append(Waypoint(t_l - t_r, right[-1].pos, right[-1].finger, "r: sync wait", right[-1].yaw))

    if mode in ("settle", "dream"):
        left.append(Waypoint(14.4, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        right.append(Waypoint(14.4, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # === STEP 1: INITIAL STANCE ===
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    # If no_estop ablation: left arm reaches into lane danger zone while pusher is moving -> FAULT!
    if mode == "no_estop":
        left.append(Waypoint(0.6, block_hover, FINGER_PRE_BLOCK, "l: hover over moving lane", 0.0))
        left.append(Waypoint(0.6, block_grasp, FINGER_PRE_BLOCK, "l: reach in while moving FAULT", 0.0))
        sync_arms()
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: settle", 0.0))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 2: RIGHT ARM PRESSES E-STOP BUTTON ===
    right.append(Waypoint(0.8, estop_hover, FINGER_PRESS, "r: hover estop", 0.0))
    right.append(Waypoint(0.5, estop_press, FINGER_PRESS, "r: press estop", 0.0))
    right.append(Waypoint(0.4, estop_press, FINGER_PRESS, "r: hold estop", 0.0))
    right.append(Waypoint(0.5, estop_hover, FINGER_OPEN_VAL, "r: release estop", 0.0))
    # Wait for pusher to arrest (0.3 s stopping time + 0.5 s margin = 0.8 s)
    right.append(Waypoint(0.8, estop_hover, FINGER_OPEN_VAL, "r: wait pusher stop", 0.0))
    sync_arms()

    # === STEP 3: START EARLY ABLATION ===
    # If start_early: RESET then START pressed without removing the stuck block -> FAULT!
    if mode == "start_early":
        # Right arm moves to RESET and presses
        right.append(Waypoint(0.6, reset_hover, FINGER_PRESS, "r: hover reset", 0.0))
        right.append(Waypoint(0.4, reset_press, FINGER_PRESS, "r: press reset", 0.0))
        right.append(Waypoint(0.4, reset_hover, FINGER_OPEN_VAL, "r: release reset", 0.0))
        # Right arm moves to START and presses (block still in lane!) -> FAULT
        right.append(Waypoint(0.6, start_hover, FINGER_PRESS, "r: hover start", 0.0))
        right.append(Waypoint(0.4, start_press, FINGER_PRESS, "r: press start early FAULT", 0.0))
        right.append(Waypoint(0.4, start_hover, FINGER_OPEN_VAL, "r: release start", 0.0))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
        sync_arms()
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: settle", 0.0))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 4: LEFT ARM EXTRACTS STUCK BLOCK FROM LANE ===
    # Left arm hovers over lane at stuck block
    left.append(Waypoint(0.8, block_hover, FINGER_PRE_BLOCK, "l: hover block", 0.0))
    # Descend over 20 mm wall to grasp block
    left.append(Waypoint(0.6, block_grasp, FINGER_PRE_BLOCK, "l: descend block", 0.0))
    left.append(Waypoint(0.4, block_grasp, FINGER_GRASP, "l: grasp block", 0.0))
    # Lift block clear over walls
    left.append(Waypoint(0.6, (block_grasp[0], block_grasp[1], lift_clear_z), FINGER_GRASP, "l: lift block clear", 0.0))
    # Transfer to green tray
    left.append(Waypoint(0.8, tray_hover, FINGER_GRASP, "l: transfer to tray", 0.0))
    left.append(Waypoint(0.5, tray_place, FINGER_GRASP, "l: place in tray", 0.0))
    left.append(Waypoint(0.4, tray_place, FINGER_OPEN_VAL, "l: release in tray", 0.0))
    # Lift clear and return home
    left.append(Waypoint(0.5, tray_hover, FINGER_OPEN_VAL, "l: lift from tray", 0.0))
    left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
    sync_arms()

    # === STEP 5: LEAVE STOPPED ABLATION ===
    if mode == "leave_stopped":
        sync_arms()
        left.append(Waypoint(3.0, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(3.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 6: RESET THEN START SEQUENCE (Right arm) ===
    if mode != "skip_reset":
        # Press RESET button
        right.append(Waypoint(0.6, reset_hover, FINGER_PRESS, "r: hover reset", 0.0))
        right.append(Waypoint(0.4, reset_press, FINGER_PRESS, "r: press reset", 0.0))
        right.append(Waypoint(0.4, reset_hover, FINGER_OPEN_VAL, "r: release reset", 0.0))

    # Press START button (in skip_reset, START pressed without RESET -> no effect)
    right.append(Waypoint(0.6, start_hover, FINGER_PRESS, "r: hover start", 0.0))
    right.append(Waypoint(0.4, start_press, FINGER_PRESS, "r: press start", 0.0))
    right.append(Waypoint(0.4, start_hover, FINGER_OPEN_VAL, "r: release start", 0.0))
    right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))
    sync_arms()

    # === STEP 7: FINAL SUCCESS HOLD OUTSIDE DANGER ZONE ===
    # Hold for 2.2 s (44 steps at 20 Hz, satisfying the 2.0 s requirement with margin)
    left.append(Waypoint(2.2, home_l, FINGER_OPEN_VAL, "l: final success hold", 0.0))
    right.append(Waypoint(2.2, home_r, FINGER_OPEN_VAL, "r: final success hold", 0.0))

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
