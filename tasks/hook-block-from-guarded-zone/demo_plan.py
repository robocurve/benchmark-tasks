"""Bimanual demonstration plans and ablations for hook-block-from-guarded-zone.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.
Demo: Left arm grasps rod, hooks block, pulls back 0.20 m, parks rod.
Right arm grasps retrieved block and sets it inside the green tray.
"""

from __future__ import annotations

import os
from typing import Sequence
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_OPEN_VAL = g.FINGER_OPEN_VAL
import os as _os
FINGER_GRASP_ROD = float(_os.environ.get('DEMO_ROD_FINGER', g.FINGER_GRASP_ROD))
FINGER_GRASP_BLOCK = float(_os.environ.get('DEMO_BLOCK_FINGER', g.FINGER_GRASP_BLOCK))


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_REACH_OVER") == "1":
        return "reach_over"
    if os.environ.get("DEMO_DROP_ROD") == "1":
        return "drop_rod_in_zone"
    if os.environ.get("DEMO_LEAVE_OUTSIDE") == "1":
        return "leave_block_outside"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", 1.5) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.HOVER_Z)
    rel_hold = _f("DEMO_RELEASE_HOLD_S", 0.5)

    hook_x = _f("DEMO_HOOK_TCP_X", 0.360)      # left TCP x while the hook sits behind the block (limit 0.41)
    shaft_y = _f("DEMO_SHAFT_Y", 0.045)        # left TCP y: shaft passes beside the block, cross-piece behind it
    pull_x = _f("DEMO_PULL_X", 0.100)          # left TCP x at the end of the pull (block ends ~ TCP + 0.24)
    pull_hold = _f("DEMO_PULL_HOLD_S", 1.0)
    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

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
        left.append(Waypoint(10.0, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        right.append(Waypoint(10.0, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Stance
    left.append(Waypoint(0.6 * time_scale, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.6 * time_scale, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    # === ABLATION: REACH_OVER (gripper reaches x=0.44 into guarded zone -> FAULT 0.0) ===
    if mode == "reach_over":
        # Left arm moves directly across yellow line into guarded zone without tool
        left.append(Waypoint(1.2 * time_scale, (0.440, +0.180, 0.820), FINGER_OPEN_VAL, "l: breach boundary", 0.0))
        left.append(Waypoint(0.8 * time_scale, (0.440, +0.180, g.TOP + 0.040), FINGER_OPEN_VAL, "l: descend in zone", 0.0))
        left.append(Waypoint(1.0 * time_scale, (0.440, +0.180, 0.820), FINGER_OPEN_VAL, "l: lift in zone", 0.0))
        left.append(Waypoint(1.2 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()
        return {"left": left, "right": right}

    # === SHARED PHASE 1: Left arm grasps rod and lifts ===
    rod_grasp_xy = (g.ROD_GRASP_START_POS[0], g.ROD_GRASP_START_POS[1])
    rod_grasp_z = g.ROD_GRASP_START_POS[2]

    # 1. Hover above rod grasp point (60 mm from handle end)
    left.append(Waypoint(0.8 * time_scale, (rod_grasp_xy[0], rod_grasp_xy[1], hover_z), FINGER_OPEN_VAL, "l: hover rod", 0.0))
    # 2. Descend to rod
    left.append(Waypoint(0.6 * time_scale, (rod_grasp_xy[0], rod_grasp_xy[1], rod_grasp_z), FINGER_OPEN_VAL, "l: descend rod", 0.0))
    # 3. Grasp rod
    left.append(Waypoint(0.4 * time_scale, (rod_grasp_xy[0], rod_grasp_xy[1], rod_grasp_z), FINGER_GRASP_ROD, "l: grasp rod", 0.0))
    # 4. Lift rod
    left.append(Waypoint(0.6 * time_scale, (rod_grasp_xy[0], rod_grasp_xy[1], hover_z), FINGER_GRASP_ROD, "l: lift rod", 0.0))

    # === ABLATION: DROP_ROD_IN_ZONE (drops rod forward into guarded zone -> 0.25) ===
    if mode == "drop_rod_in_zone":
        left.append(Waypoint(1.2 * time_scale, (0.360, 0.000, hover_z), FINGER_GRASP_ROD, "l: carry to line", 0.0))
        left.append(Waypoint(0.8 * time_scale, (0.360, 0.000, hover_z), FINGER_OPEN_VAL, "l: drop rod", 0.0))
        left.append(Waypoint(1.2 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()
        # Hold to end
        left.append(Waypoint(4.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: settle", 0.0))
        sync_arms()
        return {"left": left, "right": right}

    # === SHARED PHASE 2: Carry rod with tip past block, hook block, pull back 0.20 m ===
    # Left TCP moves to x = 0.360, y = 0.000 at hover height (tip reaches x = 0.600, past block at 0.580)
    left.append(Waypoint(1.4 * time_scale, (hook_x, shaft_y, hover_z), FINGER_GRASP_ROD, "l: transit hook over block", 0.0))
    left.append(Waypoint(0.8 * time_scale, (hook_x, shaft_y, rod_grasp_z), FINGER_GRASP_ROD, "l: lower hook behind block", 0.0))
    left.append(Waypoint(0.4 * time_scale, (hook_x, shaft_y, rod_grasp_z), FINGER_GRASP_ROD, "l: settle hook", 0.0))
    left.append(Waypoint(1.8 * time_scale, (pull_x, shaft_y, rod_grasp_z), FINGER_GRASP_ROD, "l: pull block out of zone", 0.0))
    left.append(Waypoint(pull_hold * time_scale, (pull_x, shaft_y, rod_grasp_z), FINGER_GRASP_ROD, "l: hold pull", 0.0))
    left.append(Waypoint(0.6 * time_scale, (pull_x, shaft_y, hover_z), FINGER_GRASP_ROD, "l: lift rod clear", 0.0))

    # Park rod safely outside guarded zone
    park_xy = (g.ROD_PARK_POS[0], g.ROD_PARK_POS[1])
    park_z = g.ROD_PARK_POS[2]
    left.append(Waypoint(1.0 * time_scale, (park_xy[0], park_xy[1], hover_z), FINGER_GRASP_ROD, "l: transit rod to park", 0.0))
    left.append(Waypoint(0.6 * time_scale, (park_xy[0], park_xy[1], park_z), FINGER_GRASP_ROD, "l: descend rod to park", 0.0))
    left.append(Waypoint(rel_hold * time_scale, (park_xy[0], park_xy[1], park_z), FINGER_OPEN_VAL, "l: release rod", 0.0))
    left.append(Waypoint(0.6 * time_scale, (park_xy[0], park_xy[1], hover_z), FINGER_OPEN_VAL, "l: clear rod", 0.0))
    left.append(Waypoint(0.8 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
    sync_arms()

    # === ABLATION: LEAVE_BLOCK_OUTSIDE (block outside zone resting -> 0.75) ===
    if mode == "leave_block_outside":
        left.append(Waypoint(4.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: settle", 0.0))
        sync_arms()
        return {"left": left, "right": right}

    # === PHASE 3 (DEMO): Right arm picks block from x <= 0.36 and places in tray ===
    # Block retrieved position is at x ~ 0.360, y ~ 0.000, resting on table
    block_pick_xy = (0.360, 0.000)
    block_pick_z = g.TOP + g.BLOCK_HEIGHT / 2

    tray_place_xy = (g.TRAY_POS[0], g.TRAY_POS[1])
    tray_place_z = g.TRAY_SEAT_Z

    # 1. Right arm hovers above retrieved block
    right.append(Waypoint(1.0 * time_scale, (block_pick_xy[0], block_pick_xy[1], hover_z), FINGER_OPEN_VAL, "r: hover block", 0.0))
    # 2. Descend to grasp block
    right.append(Waypoint(0.6 * time_scale, (block_pick_xy[0], block_pick_xy[1], block_pick_z), FINGER_OPEN_VAL, "r: descend block", 0.0))
    # 3. Grasp block
    right.append(Waypoint(0.4 * time_scale, (block_pick_xy[0], block_pick_xy[1], block_pick_z), FINGER_GRASP_BLOCK, "r: grasp block", 0.0))
    # 4. Lift block
    right.append(Waypoint(0.6 * time_scale, (block_pick_xy[0], block_pick_xy[1], hover_z), FINGER_GRASP_BLOCK, "r: lift block", 0.0))
    # 5. Transit block to green tray
    right.append(Waypoint(1.2 * time_scale, (tray_place_xy[0], tray_place_xy[1], hover_z), FINGER_GRASP_BLOCK, "r: transit tray", 0.0))
    # 6. Descend block into tray
    right.append(Waypoint(0.6 * time_scale, (tray_place_xy[0], tray_place_xy[1], tray_place_z), FINGER_GRASP_BLOCK, "r: place tray", 0.0))
    # 7. Release block
    right.append(Waypoint(rel_hold * time_scale, (tray_place_xy[0], tray_place_xy[1], tray_place_z), FINGER_OPEN_VAL, "r: release tray", 0.0))
    # 8. Lift clear
    right.append(Waypoint(0.6 * time_scale, (tray_place_xy[0], tray_place_xy[1], hover_z), FINGER_OPEN_VAL, "r: clear tray", 0.0))
    # 9. Return to home
    right.append(Waypoint(0.8 * time_scale, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
    sync_arms()

    # Final settle hold to satisfy 2.0 s hold time for 1.00
    left.append(Waypoint(3.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: final hold", 0.0))
    right.append(Waypoint(3.0 * time_scale, home_r, FINGER_OPEN_VAL, "r: final hold", 0.0))
    sync_arms()

    return {"left": left, "right": right}
