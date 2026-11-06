"""Bimanual demonstration plans and ablations for spring-drawer-hold-and-retrieve.

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
FINGER_PRESS = 0.0             # closed fingers (unused)
FINGER_HANDLE = -0.025         # 50 mm jaw gap on the 60 mm wide handle (5 mm past each face)
FINGER_PRE_BLOCK = -0.030      # 60 mm gap: clears the 100 mm wide drawer interior and the 40 mm block


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_HOLD") == "1":
        return "no_hold"
    if os.environ.get("DEMO_PINCH") == "1":
        return "pinch"
    if os.environ.get("DEMO_LEAVE_OPEN") == "1":
        return "leave_open"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    insert_dz = _f("DEMO_INSERT_DZ", 0.0)
    overshoot_x = _f("DEMO_OVERSHOOT_X", 0.015)  # 15 mm overshoot to cover arm compliance vs 18 N/m spring

    hover_z = g.HOVER_Z
    grasp_handle_z = g.GRASP_HANDLE_Z
    grasp_block_z = g.GRASP_BLOCK_Z + insert_dz
    shelf_z = g.SHELF_SPOT[2]

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    handle_closed_x = g.HANDLE_REST_X
    handle_open_x = g.HANDLE_REST_X - g.DRAWER_TRAVEL - overshoot_x

    block_open_x = g.BLOCK_REST_X - g.DRAWER_TRAVEL
    shelf_spot = (g.SHELF_SPOT[0], g.SHELF_SPOT[1], shelf_z)

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

    # === STEP 2: LEFT ARMS HOOKS HANDLE AND PULLS DRAWER OPEN ===
    left.append(Waypoint(0.6, (handle_closed_x - 0.040, 0.000, hover_z), FINGER_OPEN_VAL, "l: hover handle", 0.0))
    left.append(Waypoint(0.5, (handle_closed_x - 0.040, 0.000, grasp_handle_z), FINGER_OPEN_VAL, "l: align handle", 0.0))
    left.append(Waypoint(0.4, (handle_closed_x, 0.000, grasp_handle_z), FINGER_OPEN_VAL, "l: approach handle", 0.0))
    left.append(Waypoint(0.4, (handle_closed_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: grasp handle", 0.0))

    # Pull drawer open to 140 mm
    left.append(Waypoint(1.0, (handle_open_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: pull drawer open", 0.0))
    # Hold open >= 1.0 s
    hold_duration = 0.2 if mode == "no_hold" else 4.0
    left.append(Waypoint(hold_duration, (handle_open_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: hold drawer open", 0.0))

    if mode == "no_hold":
        # Let go immediately: drawer snaps closed, block never retrievable
        left.append(Waypoint(0.5, (handle_closed_x, 0.000, grasp_handle_z), FINGER_OPEN_VAL, "l: release handle early", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # Right waits for drawer to be fully open
    sync_arms()

    # === STEP 3: RIGHT REACHES IN AND RETRIEVES WHITE BLOCK ===
    target_block_x = block_open_x

    right.append(Waypoint(0.6, (target_block_x, 0.000, hover_z), FINGER_PRE_BLOCK, "r: hover block", 0.0))
    right.append(Waypoint(0.5, (target_block_x, 0.000, grasp_block_z), FINGER_PRE_BLOCK, "r: descend block", 0.0))
    right.append(Waypoint(0.4, (target_block_x, 0.000, grasp_block_z), FINGER_GRASP, "r: grasp block", 0.0))

    if mode == "pinch":
        # The right hand now holds the block at the lip: the left lets the drawer snap shut on it -> PINCH fault!
        sync_arms()
        left.append(Waypoint(0.3, (handle_closed_x, 0.000, grasp_handle_z), FINGER_OPEN_VAL, "l: snap close pinch", 0.0))
        sync_arms()
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # Normal mode: lift block clear of drawer
    right.append(Waypoint(0.6, (block_open_x, 0.000, hover_z), FINGER_GRASP, "r: lift block clear", 0.0))
    # Place on green shelf
    right.append(Waypoint(0.7, (shelf_spot[0], shelf_spot[1], hover_z), FINGER_GRASP, "r: hover shelf", 0.0))
    right.append(Waypoint(0.5, shelf_spot, FINGER_GRASP, "r: place shelf", 0.0))
    right.append(Waypoint(0.4, shelf_spot, FINGER_OPEN_VAL, "r: release shelf", 0.0))
    right.append(Waypoint(0.5, (shelf_spot[0], shelf_spot[1], hover_z), FINGER_OPEN_VAL, "r: lift shelf", 0.0))
    right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))
    sync_arms()

    # === STEP 4: LEFT CLOSES DRAWER (unless leave_open ablation) ===
    if mode != "leave_open":
        # Guide drawer back to closed position smoothly
        left.append(Waypoint(1.0, (handle_closed_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: close drawer smooth", 0.0))
        left.append(Waypoint(0.5, (handle_closed_x - 0.050, 0.000, hover_z), FINGER_OPEN_VAL, "l: release handle closed", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))
    else:
        # Keep holding open
        left.append(Waypoint(2.0, (handle_open_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: stay open hold", 0.0))
    sync_arms()

    # === STEP 5: FINAL SETTLE ===
    if mode == "leave_open":
        # Keep the drawer held open to the very end (it would spring shut as soon as the hand lets go).
        left.append(Waypoint(3.0, (handle_open_x, 0.000, grasp_handle_z), FINGER_HANDLE, "l: final hold open", 0.0))
    else:
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(1.5 if mode != "leave_open" else 3.0, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))

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
