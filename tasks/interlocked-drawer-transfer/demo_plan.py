"""Bimanual demonstration plans and ablations for interlocked-drawer-transfer.

Uses yam_arena.demo.Waypoint.

Modes:
- demo: Carry solution. Left arm opens left drawer, retrieves block, holds outside
        while right arm closes left drawer, right arm opens right drawer,
        block placed in right drawer, closed. Score = 1.00.
- park: Staging pad solution. Left arm retrieves block, parks on cabinet top staging
        pad, closes left drawer, right arm opens right drawer, picks block from pad,
        places in right drawer, closed. Score = 1.00.
- both_open: Right drawer opened while left drawer is still open -> TIP fault 0.25.
- leave_open: Right drawer left open at end -> Score 0.75.
- wrong_drawer: Block returned to left drawer instead of right -> Score 0.50.
- block_dropped: Block dropped off table -> Score 0.00.
- settle: Arms hold at home for 8 s -> Score 0.00.
- dream: Dream-1 baseline -> Score 0.00.
"""

from __future__ import annotations

import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_GRASP = -0.012          # Firm grip on 40 mm block
FINGER_OPEN_VAL = FINGER_OPEN  # Fully open (-0.04695)
FINGER_HANDLE = -0.022         # Grip on 45 mm wide handle
FINGER_PRE_BLOCK = -0.030      # Pre-grasp clearing drawer interior


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_BOTH_OPEN") == "1":
        return "both_open"
    if os.environ.get("DEMO_LEAVE_OPEN") == "1":
        return "leave_open"
    if os.environ.get("DEMO_WRONG_DRAWER") == "1":
        return "wrong_drawer"
    if os.environ.get("DEMO_BLOCK_DROPPED") == "1":
        return "block_dropped"
    if os.environ.get("DEMO_PARK") == "1":
        return "park"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    hover_z = g.HOVER_Z
    grasp_handle_z = g.GRASP_HANDLE_Z
    grasp_block_z = g.GRASP_BLOCK_DRAWER_Z
    lift_clear_z = g.LIFT_BLOCK_CLEAR_Z
    staging_spot = g.STAGING_SPOT

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    stroke = _f("DEMO_DRAWER_STROKE", 0.095)  # 95 mm stroke (> 70 mm goal by 25 mm)
    drawer_hold_s = _f("DEMO_DRAWER_HOLD_S", 0.6)
    staging_grasp_dz = _f("DEMO_STAGING_GRASP_DZ", 0.012)
    close_overshoot = _f("DEMO_DRAWER_CLOSE_OVERSHOOT", 0.030)

    handle_closed_x = g.HANDLE_REST_X
    handle_open_x = handle_closed_x - stroke
    handle_pushed_closed_x = handle_closed_x + close_overshoot

    left_handle_closed = (handle_closed_x, g.DRAWER_LEFT_Y, grasp_handle_z)
    left_handle_pushed_closed = (handle_pushed_closed_x, g.DRAWER_LEFT_Y, grasp_handle_z)
    left_handle_open = (handle_open_x, g.DRAWER_LEFT_Y, grasp_handle_z)

    right_handle_closed = (handle_closed_x, g.DRAWER_RIGHT_Y, grasp_handle_z)
    right_handle_pushed_closed = (handle_pushed_closed_x, g.DRAWER_RIGHT_Y, grasp_handle_z)
    right_handle_open = (handle_open_x, g.DRAWER_RIGHT_Y, grasp_handle_z)

    block_start_open_x = g.BLOCK_START_X - stroke
    block_drawer_left_open = (block_start_open_x, g.DRAWER_LEFT_Y, grasp_block_z)
    block_drawer_right_open = (block_start_open_x, g.DRAWER_RIGHT_Y, grasp_block_z)

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

    # === STEP 2: LEFT ARM OPENS LEFT DRAWER ===
    left.append(Waypoint(0.6, (handle_closed_x - 0.040, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: hover left handle", 0.0))
    left.append(Waypoint(0.5, (handle_closed_x - 0.040, g.DRAWER_LEFT_Y, grasp_handle_z), FINGER_OPEN_VAL, "l: align left handle", 0.0))
    left.append(Waypoint(0.4, left_handle_closed, FINGER_OPEN_VAL, "l: approach left handle", 0.0))
    left.append(Waypoint(0.4, left_handle_closed, FINGER_HANDLE, "l: grasp left handle", 0.0))
    left.append(Waypoint(1.0, left_handle_open, FINGER_HANDLE, "l: pull left drawer open", 0.0))
    left.append(Waypoint(drawer_hold_s, left_handle_open, FINGER_HANDLE, "l: hold left drawer open", 0.0))
    left.append(Waypoint(0.3, left_handle_open, FINGER_OPEN_VAL, "l: release left handle", 0.0))
    left.append(Waypoint(0.5, (handle_open_x, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: lift from handle", 0.0))
    sync_arms()

    # === STEP 3: LEFT ARM RETRIEVES WHITE BLOCK ===
    left.append(Waypoint(0.6, (block_start_open_x, g.DRAWER_LEFT_Y, hover_z), FINGER_PRE_BLOCK, "l: hover left block", 0.0))
    left.append(Waypoint(0.5, block_drawer_left_open, FINGER_PRE_BLOCK, "l: descend to block", 0.0))
    left.append(Waypoint(0.4, block_drawer_left_open, FINGER_GRASP, "l: grasp block", 0.0))
    left.append(Waypoint(0.6, (block_start_open_x, g.DRAWER_LEFT_Y, lift_clear_z), FINGER_GRASP, "l: lift block clear", 0.0))
    sync_arms()

    if mode == "block_dropped":
        # Drop block off table
        drop_pos = (0.100, +0.450, g.TOP - 0.100)
        left.append(Waypoint(0.8, (0.100, +0.450, hover_z), FINGER_GRASP, "l: carry to drop", 0.0))
        left.append(Waypoint(0.6, drop_pos, FINGER_GRASP, "l: lower off table", 0.0))
        left.append(Waypoint(0.3, drop_pos, FINGER_OPEN_VAL, "l: release drop block", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
        sync_arms()
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    if mode == "both_open":
        # Right arm opens right drawer WHILE left drawer is still open -> TIP fault!
        right.append(Waypoint(0.6, (handle_closed_x - 0.040, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: hover right handle", 0.0))
        right.append(Waypoint(0.5, right_handle_closed, FINGER_OPEN_VAL, "r: approach right handle", 0.0))
        right.append(Waypoint(0.4, right_handle_closed, FINGER_HANDLE, "r: grasp right handle", 0.0))
        right.append(Waypoint(0.8, right_handle_open, FINGER_HANDLE, "r: open right drawer while left open", 0.0))
        # Hold both open for >= 0.3 s to trigger TIP fault
        right.append(Waypoint(0.6, right_handle_open, FINGER_HANDLE, "r: hold both open tip", 0.0))
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_GRASP, "l: hold block tip", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: return home tip", 0.0))
        return {"left": left, "right": right}

    if mode in ("demo", "park", "leave_open"):
        # === STAGING PAD TRANSFER SOLUTION ===
        # Step 4: Left parks block on staging pad
        left.append(Waypoint(0.7, (staging_spot[0], staging_spot[1], hover_z), FINGER_GRASP, "l: hover staging", 0.0))
        left.append(Waypoint(0.5, (staging_spot[0], staging_spot[1], staging_spot[2] + 0.005), FINGER_GRASP, "l: place staging pad", 0.0))
        left.append(Waypoint(0.4, (staging_spot[0], staging_spot[1], staging_spot[2] + 0.005), FINGER_OPEN_VAL, "l: release staging pad", 0.0))
        left.append(Waypoint(0.5, (staging_spot[0], staging_spot[1], hover_z), FINGER_OPEN_VAL, "l: lift from staging", 0.0))

        # Step 5: Left closes left drawer
        left.append(Waypoint(0.5, (handle_open_x, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: hover open handle", 0.0))
        left.append(Waypoint(0.6, left_handle_open, FINGER_OPEN_VAL, "l: align open handle", 0.0))
        left.append(Waypoint(0.4, left_handle_open, FINGER_HANDLE, "l: grasp open handle", 0.0))
        left.append(Waypoint(1.0, left_handle_pushed_closed, FINGER_HANDLE, "l: push left drawer closed", 0.0))
        left.append(Waypoint(0.4, left_handle_pushed_closed, FINGER_HANDLE, "l: hold left drawer closed", 0.0))
        left.append(Waypoint(0.3, left_handle_closed, FINGER_OPEN_VAL, "l: release closed handle", 0.0))
        left.append(Waypoint(0.5, (handle_closed_x - 0.050, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: back off left handle", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
        sync_arms()

        # Step 6: Right opens right drawer
        right.append(Waypoint(0.6, (handle_closed_x - 0.040, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: hover right handle", 0.0))
        right.append(Waypoint(0.5, right_handle_closed, FINGER_OPEN_VAL, "r: approach right handle", 0.0))
        right.append(Waypoint(0.4, right_handle_closed, FINGER_HANDLE, "r: grasp right handle", 0.0))
        right.append(Waypoint(1.0, right_handle_open, FINGER_HANDLE, "r: pull right drawer open", 0.0))
        right.append(Waypoint(drawer_hold_s, right_handle_open, FINGER_HANDLE, "r: hold right drawer open", 0.0))
        right.append(Waypoint(0.3, right_handle_open, FINGER_OPEN_VAL, "r: release right handle", 0.0))
        right.append(Waypoint(0.5, (handle_open_x, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: lift from handle", 0.0))

        # Step 7: Right picks block from staging pad (grip height moved up by staging_grasp_dz so palm never hits staging pad/cabinet lip per Lesson 2)
        right_staging_z = staging_spot[2] + staging_grasp_dz
        right.append(Waypoint(0.6, (staging_spot[0], staging_spot[1], hover_z), FINGER_PRE_BLOCK, "r: hover staging", 0.0))
        right.append(Waypoint(0.6, (staging_spot[0], staging_spot[1], right_staging_z), FINGER_PRE_BLOCK, "r: descend staging", 0.0))
        right.append(Waypoint(0.5, (staging_spot[0], staging_spot[1], right_staging_z), FINGER_GRASP, "r: grasp from staging", 0.0))
        right.append(Waypoint(0.3, (staging_spot[0], staging_spot[1], right_staging_z), FINGER_GRASP, "r: hold grasp staging", 0.0))
        right.append(Waypoint(0.6, (staging_spot[0], staging_spot[1], hover_z), FINGER_GRASP, "r: lift from staging", 0.0))

        # Step 8: Right places block in right drawer
        right.append(Waypoint(0.6, (block_start_open_x, g.DRAWER_RIGHT_Y, hover_z), FINGER_GRASP, "r: hover right drawer", 0.0))
        right.append(Waypoint(0.5, block_drawer_right_open, FINGER_GRASP, "r: place right drawer", 0.0))
        right.append(Waypoint(0.3, block_drawer_right_open, FINGER_OPEN_VAL, "r: release in right drawer", 0.0))
        right.append(Waypoint(0.5, (block_start_open_x, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: lift clear right drawer", 0.0))

        if mode == "leave_open":
            right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home leave open", 0.0))
            sync_arms()
            left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
            right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
            return {"left": left, "right": right}

        # Step 9: Right closes right drawer
        # measured: the drawer stalls ~77 mm open (not the commanded stroke); aim at where the handle really is
        right_handle_actual = (handle_closed_x - _f("DEMO_DRAWER_ACTUAL_OPEN", 0.077), g.DRAWER_RIGHT_Y, grasp_handle_z)
        right.append(Waypoint(0.5, (right_handle_actual[0] - 0.030, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: hover behind right handle", 0.0))
        right.append(Waypoint(0.5, (right_handle_actual[0] - 0.030, g.DRAWER_RIGHT_Y, grasp_handle_z), FINGER_OPEN_VAL, "r: descend behind right handle", 0.0))
        right.append(Waypoint(0.4, right_handle_actual, FINGER_OPEN_VAL, "r: align right handle", 0.0))
        right.append(Waypoint(0.5, right_handle_actual, FINGER_HANDLE, "r: grasp right handle", 0.0))
        right.append(Waypoint(1.0, right_handle_pushed_closed, FINGER_HANDLE, "r: push right drawer closed", 0.0))
        right.append(Waypoint(_f("DEMO_CLOSE_HOLD_S", 1.0), right_handle_pushed_closed, FINGER_HANDLE, "r: hold right drawer closed", 0.0))
        right.append(Waypoint(0.4, right_handle_pushed_closed, FINGER_OPEN_VAL, "r: release right handle closed", 0.0))
        right.append(Waypoint(0.5, (handle_pushed_closed_x, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: lift off right handle", 0.0))
        right.append(Waypoint(0.5, (handle_closed_x - 0.050, g.DRAWER_RIGHT_Y, hover_z), FINGER_OPEN_VAL, "r: back off right handle", 0.0))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))
        sync_arms()

        # Step 10: Settle
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    if mode == "wrong_drawer":
        # Left puts block back into left drawer
        left.append(Waypoint(0.8, (block_start_open_x, g.DRAWER_LEFT_Y, hover_z), FINGER_GRASP, "l: return over left", 0.0))
        left.append(Waypoint(0.5, block_drawer_left_open, FINGER_GRASP, "l: place left drawer", 0.0))
        left.append(Waypoint(0.3, block_drawer_left_open, FINGER_OPEN_VAL, "l: release back in left", 0.0))
        left.append(Waypoint(0.5, (block_start_open_x, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: lift clear left drawer", 0.0))
        left.append(Waypoint(0.5, left_handle_open, FINGER_HANDLE, "l: grasp open handle", 0.0))
        left.append(Waypoint(0.8, left_handle_closed, FINGER_HANDLE, "l: push left drawer closed", 0.0))
        left.append(Waypoint(0.3, left_handle_closed, FINGER_OPEN_VAL, "l: release closed handle", 0.0))
        left.append(Waypoint(0.5, (handle_closed_x - 0.050, g.DRAWER_LEFT_Y, hover_z), FINGER_OPEN_VAL, "l: back off left handle", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
        sync_arms()
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` with every waypoint duration stretched by DEMO_TIME_SCALE (default 1.8)."""
    if mode is None:
        mode = mode_from_env()
    result = _plans(mode)
    if mode in ("settle", "dream"):
        return result
    scale = _f("DEMO_TIME_SCALE", 1.8)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
