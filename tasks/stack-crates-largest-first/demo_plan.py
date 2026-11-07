"""Bimanual demonstration plans and ablations for stack-crates-largest-first.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.
"""

from __future__ import annotations

import os
import numpy as np

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_OPEN_VAL = FINGER_OPEN  # -0.04695
FINGER_GRASP_LARGE = -0.038    # firm grip on 80 mm crate
FINGER_GRASP_MED = -0.028      # firm grip on 60 mm crate
FINGER_GRASP_SMALL = -0.020    # firm grip on 45 mm crate


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_WRONG_ORDER") == "1":
        return "wrong_order"
    if os.environ.get("DEMO_OFFSET") == "1":
        return "offset"
    if os.environ.get("DEMO_LEAVE_SMALL") == "1":
        return "leave_small"
    if os.environ.get("DEMO_KNOCK") == "1":
        return "knock"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    # DEMO_TIME_SCALE applies only to active manipulation demos
    time_scale = _f("DEMO_TIME_SCALE", 1.8) if mode not in ("settle", "dream") else 1.0

    hover_z = g.HOVER_Z
    grasp_z = g.TOP + 0.030

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    pallet_pos = g.PALLET_POS
    large_start = g.LARGE_START_POS
    medium_start = g.MEDIUM_START_POS
    small_start = g.SMALL_START_POS

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

    # Stance
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    # === ABLATION: WRONG_ORDER (Medium on pallet first, then Large on Medium) ===
    if mode == "wrong_order":
        # Right arm brings Medium to pallet
        right.append(Waypoint(0.6 * time_scale, (medium_start[0], medium_start[1], hover_z), FINGER_OPEN_VAL, "r: hover medium", 0.0))
        right.append(Waypoint(0.5 * time_scale, (medium_start[0], medium_start[1], grasp_z), FINGER_OPEN_VAL, "r: descend medium", 0.0))
        right.append(Waypoint(0.4 * time_scale, (medium_start[0], medium_start[1], grasp_z), FINGER_GRASP_MED, "r: grasp medium", 0.0))
        right.append(Waypoint(0.5 * time_scale, (medium_start[0], medium_start[1], hover_z), FINGER_GRASP_MED, "r: lift medium", 0.0))
        right.append(Waypoint(0.6 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_GRASP_MED, "r: transit pallet", 0.0))
        right.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_1_TARGET_Z + 0.005), FINGER_GRASP_MED, "r: place pallet", 0.0))
        right.append(Waypoint(0.4 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_1_TARGET_Z + 0.005), FINGER_OPEN_VAL, "r: release medium", 0.0))
        right.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_OPEN_VAL, "r: clear medium", 0.0))
        right.append(Waypoint(0.6 * time_scale, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
        sync_arms()

        # Left arm brings Large on top of Medium (invalid: large on smaller)
        left.append(Waypoint(0.6 * time_scale, (large_start[0], large_start[1], hover_z), FINGER_OPEN_VAL, "l: hover large", 0.0))
        left.append(Waypoint(0.5 * time_scale, (large_start[0], large_start[1], grasp_z), FINGER_OPEN_VAL, "l: descend large", 0.0))
        left.append(Waypoint(0.4 * time_scale, (large_start[0], large_start[1], grasp_z), FINGER_GRASP_LARGE, "l: grasp large", 0.0))
        left.append(Waypoint(0.5 * time_scale, (large_start[0], large_start[1], hover_z), FINGER_GRASP_LARGE, "l: lift large", 0.0))
        left.append(Waypoint(0.6 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_GRASP_LARGE, "l: transit pallet", 0.0))
        left.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_2_TARGET_Z + 0.005), FINGER_GRASP_LARGE, "l: place large on medium", 0.0))
        left.append(Waypoint(0.4 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_2_TARGET_Z + 0.005), FINGER_OPEN_VAL, "l: release large", 0.0))
        left.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_OPEN_VAL, "l: clear large", 0.0))
        left.append(Waypoint(0.6 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()

        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 1: LEFT ARM BRINGS LARGE CRATE TO PALLET ===
    left.append(Waypoint(0.6 * time_scale, (large_start[0], large_start[1], hover_z), FINGER_OPEN_VAL, "l: hover large", 0.0))
    left.append(Waypoint(0.5 * time_scale, (large_start[0], large_start[1], grasp_z), FINGER_OPEN_VAL, "l: descend large", 0.0))
    left.append(Waypoint(0.4 * time_scale, (large_start[0], large_start[1], grasp_z), FINGER_GRASP_LARGE, "l: grasp large", 0.0))
    left.append(Waypoint(0.5 * time_scale, (large_start[0], large_start[1], hover_z), FINGER_GRASP_LARGE, "l: lift large", 0.0))
    left.append(Waypoint(0.6 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_GRASP_LARGE, "l: transit pallet", 0.0))
    left.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_1_TARGET_Z + 0.005), FINGER_GRASP_LARGE, "l: place pallet", 0.0))
    left.append(Waypoint(0.4 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_1_TARGET_Z + 0.005), FINGER_OPEN_VAL, "l: release large", 0.0))
    left.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_OPEN_VAL, "l: clear large", 0.0))
    left.append(Waypoint(0.6 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
    sync_arms()

    # === STEP 2: RIGHT ARM BRINGS MEDIUM CRATE TO LARGE CRATE ===
    right.append(Waypoint(0.6 * time_scale, (medium_start[0], medium_start[1], hover_z), FINGER_OPEN_VAL, "r: hover medium", 0.0))
    right.append(Waypoint(0.5 * time_scale, (medium_start[0], medium_start[1], grasp_z), FINGER_OPEN_VAL, "r: descend medium", 0.0))
    right.append(Waypoint(0.4 * time_scale, (medium_start[0], medium_start[1], grasp_z), FINGER_GRASP_MED, "r: grasp medium", 0.0))
    right.append(Waypoint(0.5 * time_scale, (medium_start[0], medium_start[1], hover_z), FINGER_GRASP_MED, "r: lift medium", 0.0))
    right.append(Waypoint(0.6 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_GRASP_MED, "r: transit pallet", 0.0))
    right.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_2_TARGET_Z + 0.005), FINGER_GRASP_MED, "r: place medium", 0.0))
    right.append(Waypoint(0.4 * time_scale, (pallet_pos[0], pallet_pos[1], g.LEVEL_2_TARGET_Z + 0.005), FINGER_OPEN_VAL, "r: release medium", 0.0))
    right.append(Waypoint(0.5 * time_scale, (pallet_pos[0], pallet_pos[1], hover_z), FINGER_OPEN_VAL, "r: clear medium", 0.0))
    right.append(Waypoint(0.6 * time_scale, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
    sync_arms()

    if mode == "leave_small":
        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 3: RIGHT ARM BRINGS SMALL CRATE TO MEDIUM CRATE ===
    small_target_xy = (pallet_pos[0] + 0.025, pallet_pos[1]) if mode == "offset" else (pallet_pos[0], pallet_pos[1])

    right.append(Waypoint(0.6 * time_scale, (small_start[0], small_start[1], hover_z), FINGER_OPEN_VAL, "r: hover small", 0.0))
    right.append(Waypoint(0.5 * time_scale, (small_start[0], small_start[1], grasp_z), FINGER_OPEN_VAL, "r: descend small", 0.0))
    right.append(Waypoint(0.4 * time_scale, (small_start[0], small_start[1], grasp_z), FINGER_GRASP_SMALL, "r: grasp small", 0.0))
    right.append(Waypoint(0.5 * time_scale, (small_start[0], small_start[1], hover_z), FINGER_GRASP_SMALL, "r: lift small", 0.0))
    right.append(Waypoint(0.6 * time_scale, (small_target_xy[0], small_target_xy[1], hover_z), FINGER_GRASP_SMALL, "r: transit small", 0.0))
    right.append(Waypoint(0.5 * time_scale, (small_target_xy[0], small_target_xy[1], g.LEVEL_3_TARGET_Z + 0.005), FINGER_GRASP_SMALL, "r: place small", 0.0))
    right.append(Waypoint(0.4 * time_scale, (small_target_xy[0], small_target_xy[1], g.LEVEL_3_TARGET_Z + 0.005), FINGER_OPEN_VAL, "r: release small", 0.0))
    right.append(Waypoint(0.5 * time_scale, (small_target_xy[0], small_target_xy[1], hover_z), FINGER_OPEN_VAL, "r: clear small", 0.0))
    right.append(Waypoint(0.6 * time_scale, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
    sync_arms()

    # === ABLATION: KNOCK (sweep across stack to topple it) ===
    if mode == "knock":
        right.append(Waypoint(0.6 * time_scale, (pallet_pos[0], pallet_pos[1] - 0.080, g.LEVEL_2_TARGET_Z), FINGER_OPEN_VAL, "r: approach knock", 0.0))
        right.append(Waypoint(0.4 * time_scale, (pallet_pos[0], pallet_pos[1] + 0.080, g.LEVEL_2_TARGET_Z), FINGER_OPEN_VAL, "r: knock stack", 0.0))
        right.append(Waypoint(0.6 * time_scale, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
        sync_arms()

    # === STEP 4: FINAL SETTLE & CLEARANCE HOLD ===
    left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))

    return {"left": left, "right": right}

