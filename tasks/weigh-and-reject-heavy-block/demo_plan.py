"""Bimanual demonstration plans and ablations for weigh-and-reject-heavy-block.

Uses yam_arena.demo.Waypoint.
"""

from __future__ import annotations

import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_GRASP = -0.012          # 40 mm block grasped firmly
FINGER_OPEN_VAL = FINGER_OPEN  # fully open grippers (-0.04695)


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_WEIGH") == "1":
        return "no_weigh"
    if os.environ.get("DEMO_WRONG_PICK") == "1":
        return "wrong_pick"
    if os.environ.get("DEMO_WEIGH_TOO_SHORT") == "1":
        return "weigh_too_short"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)
    time_scale = _f("DEMO_TIME_SCALE", 1.8)

    hover_z = g.HOVER_Z
    home_l = g.HOME_L
    home_r = g.HOME_R

    # Settle mode: explicit 8.0 s hold at home with open grippers
    if mode in ("settle", "dream"):
        return {
            "left": [Waypoint(8.0, home_l, FINGER_OPEN_VAL, "l: settle hold", yaw_l)],
            "right": [Waypoint(8.0, home_r, FINGER_OPEN_VAL, "r: settle hold", yaw_r)],
        }

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    # Initial ready pose
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: ready", yaw_l))
    right.append(Waypoint(0.5, home_r, FINGER_OPEN_VAL, "r: ready", yaw_r))

    def sync_arms():
        dur_l = sum(w.duration for w in left)
        dur_r = sum(w.duration for w in right)
        if dur_l < dur_r:
            left.append(Waypoint(dur_r - dur_l, left[-1].pos, left[-1].finger, "l: sync wait", yaw_l))
        elif dur_r < dur_l:
            right.append(Waypoint(dur_l - dur_r, right[-1].pos, right[-1].finger, "r: sync wait", yaw_r))

    scale_xy = g.SCALE_CENTER_XY
    scale_place_z = g.SCALE_PLACE_Z
    scale_hover_z = g.SCALE_HOVER_Z

    if mode == "demo":
        # === BLOCK 0 (Light) ===
        # Left picks block_0 from supply
        p0 = g.BLOCK_SUPPLY_POS["block_0"]
        left.append(Waypoint(0.6, (p0[0], p0[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], p0[2]), FINGER_OPEN_VAL, "l: descend supply b0", yaw_l))
        left.append(Waypoint(0.4, (p0[0], p0[1], p0[2]), FINGER_GRASP, "l: grasp supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], hover_z), FINGER_GRASP, "l: lift supply b0", yaw_l))

        # Left places on scale
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b0", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b0", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b0", yaw_l))

        # Weigh hold: 1.5 s clear (required >= 1.0 s)
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b0", yaw_l))
        sync_arms()

        # Left re-grasps block_0 from scale and places on green shelf
        left.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: hover scale b0 regrasp", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: descend scale b0 regrasp", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: grasp scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "l: lift scale b0", yaw_l))

        sh0 = g.SHELF_SPOTS["block_0"]
        left.append(Waypoint(0.6, (sh0[0], sh0[1], hover_z), FINGER_GRASP, "l: hover shelf b0", yaw_l))
        left.append(Waypoint(0.5, (sh0[0], sh0[1], sh0[2]), FINGER_GRASP, "l: place shelf b0", yaw_l))
        left.append(Waypoint(0.4, (sh0[0], sh0[1], sh0[2]), FINGER_OPEN_VAL, "l: release shelf b0", yaw_l))
        left.append(Waypoint(0.5, (sh0[0], sh0[1], hover_z), FINGER_OPEN_VAL, "l: lift shelf b0", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home b0", yaw_l))
        sync_arms()

        # === BLOCK 1 (Heavy) ===
        # Left picks block_1 from supply
        p1 = g.BLOCK_SUPPLY_POS["block_1"]
        left.append(Waypoint(0.6, (p1[0], p1[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b1", yaw_l))
        left.append(Waypoint(0.5, (p1[0], p1[1], p1[2]), FINGER_OPEN_VAL, "l: descend supply b1", yaw_l))
        left.append(Waypoint(0.4, (p1[0], p1[1], p1[2]), FINGER_GRASP, "l: grasp supply b1", yaw_l))
        left.append(Waypoint(0.5, (p1[0], p1[1], hover_z), FINGER_GRASP, "l: lift supply b1", yaw_l))

        # Left places on scale
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b1", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b1", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b1", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b1", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b1", yaw_l))

        # Weigh hold: 1.5 s clear (needle in red band!)
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b1", yaw_l))
        sync_arms()

        # Right arm relays heavy block_1 from scale to reject tray
        right.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "r: hover scale b1", yaw_r))
        right.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "r: descend scale b1", yaw_r))
        right.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "r: grasp scale b1", yaw_r))
        right.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "r: lift scale b1", yaw_r))

        tr = g.TRAY_SPOT
        right.append(Waypoint(0.7, (tr[0], tr[1], hover_z), FINGER_GRASP, "r: hover tray b1", yaw_r))
        right.append(Waypoint(0.5, (tr[0], tr[1], tr[2]), FINGER_GRASP, "r: place tray b1", yaw_r))
        right.append(Waypoint(0.4, (tr[0], tr[1], tr[2]), FINGER_OPEN_VAL, "r: release tray b1", yaw_r))
        right.append(Waypoint(0.5, (tr[0], tr[1], hover_z), FINGER_OPEN_VAL, "r: lift tray b1", yaw_r))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home b1", yaw_r))
        sync_arms()

        # === BLOCK 2 (Light) ===
        # Left picks block_2 from supply
        p2 = g.BLOCK_SUPPLY_POS["block_2"]
        left.append(Waypoint(0.6, (p2[0], p2[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b2", yaw_l))
        left.append(Waypoint(0.5, (p2[0], p2[1], p2[2]), FINGER_OPEN_VAL, "l: descend supply b2", yaw_l))
        left.append(Waypoint(0.4, (p2[0], p2[1], p2[2]), FINGER_GRASP, "l: grasp supply b2", yaw_l))
        left.append(Waypoint(0.5, (p2[0], p2[1], hover_z), FINGER_GRASP, "l: lift supply b2", yaw_l))

        # Left places on scale
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b2", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b2", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b2", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b2", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b2", yaw_l))

        # Weigh hold: 1.5 s clear
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b2", yaw_l))
        sync_arms()

        # Left re-grasps block_2 from scale and places on green shelf
        left.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: hover scale b2 regrasp", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: descend scale b2 regrasp", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: grasp scale b2", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "l: lift scale b2", yaw_l))

        sh2 = g.SHELF_SPOTS["block_2"]
        left.append(Waypoint(0.6, (sh2[0], sh2[1], hover_z), FINGER_GRASP, "l: hover shelf b2", yaw_l))
        left.append(Waypoint(0.5, (sh2[0], sh2[1], sh2[2]), FINGER_GRASP, "l: place shelf b2", yaw_l))
        left.append(Waypoint(0.4, (sh2[0], sh2[1], sh2[2]), FINGER_OPEN_VAL, "l: release shelf b2", yaw_l))
        left.append(Waypoint(0.5, (sh2[0], sh2[1], hover_z), FINGER_OPEN_VAL, "l: lift shelf b2", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home b2", yaw_l))
        sync_arms()

        # Final settle: 2.0 s at home with open fingers
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", yaw_l))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", yaw_r))

    elif mode == "no_weigh":
        # All three blocks weighed, but right arm places heavy block without it having been weighed
        # Simulate shortcut: right grabs block_1 before weigh completes
        p0 = g.BLOCK_SUPPLY_POS["block_0"]
        left.append(Waypoint(0.6, (p0[0], p0[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], p0[2]), FINGER_OPEN_VAL, "l: descend supply b0", yaw_l))
        left.append(Waypoint(0.4, (p0[0], p0[1], p0[2]), FINGER_GRASP, "l: grasp supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], hover_z), FINGER_GRASP, "l: lift supply b0", yaw_l))
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b0", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b0", yaw_l))
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b0", yaw_l))
        sync_arms()

    elif mode == "wrong_pick":
        # Weigh block 0, block 1, block 2, but then right arm places light block 0 in reject tray
        # 1. Weigh block 0
        p0 = g.BLOCK_SUPPLY_POS["block_0"]
        left.append(Waypoint(0.6, (p0[0], p0[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], p0[2]), FINGER_OPEN_VAL, "l: descend supply b0", yaw_l))
        left.append(Waypoint(0.4, (p0[0], p0[1], p0[2]), FINGER_GRASP, "l: grasp supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], hover_z), FINGER_GRASP, "l: lift supply b0", yaw_l))
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b0", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b0", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b0", yaw_l))
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b0", yaw_l))
        sync_arms()

        # Place block 0 on shelf temporarily
        left.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: hover scale b0 r", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: descend scale b0 r", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: grasp scale b0 r", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "l: lift scale b0 r", yaw_l))
        sh0 = g.SHELF_SPOTS["block_0"]
        left.append(Waypoint(0.6, (sh0[0], sh0[1], hover_z), FINGER_GRASP, "l: hover shelf b0", yaw_l))
        left.append(Waypoint(0.5, (sh0[0], sh0[1], sh0[2]), FINGER_GRASP, "l: place shelf b0", yaw_l))
        left.append(Waypoint(0.4, (sh0[0], sh0[1], sh0[2]), FINGER_OPEN_VAL, "l: release shelf b0", yaw_l))
        left.append(Waypoint(0.5, (sh0[0], sh0[1], hover_z), FINGER_OPEN_VAL, "l: lift shelf b0", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear shelf b0", yaw_l))
        sync_arms()

        # 2. Weigh block 1
        p1 = g.BLOCK_SUPPLY_POS["block_1"]
        left.append(Waypoint(0.6, (p1[0], p1[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b1", yaw_l))
        left.append(Waypoint(0.5, (p1[0], p1[1], p1[2]), FINGER_OPEN_VAL, "l: descend supply b1", yaw_l))
        left.append(Waypoint(0.4, (p1[0], p1[1], p1[2]), FINGER_GRASP, "l: grasp supply b1", yaw_l))
        left.append(Waypoint(0.5, (p1[0], p1[1], hover_z), FINGER_GRASP, "l: lift supply b1", yaw_l))
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b1", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b1", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b1", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b1", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b1", yaw_l))
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b1", yaw_l))
        sync_arms()

        # Place block 1 on green shelf (wrong shelf)
        left.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: hover scale b1 r", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: descend scale b1 r", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: grasp scale b1 r", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "l: lift scale b1 r", yaw_l))
        sh2 = g.SHELF_SPOTS["block_2"]
        left.append(Waypoint(0.6, (sh2[0], sh2[1], hover_z), FINGER_GRASP, "l: hover shelf b1", yaw_l))
        left.append(Waypoint(0.5, (sh2[0], sh2[1], sh2[2]), FINGER_GRASP, "l: place shelf b1", yaw_l))
        left.append(Waypoint(0.4, (sh2[0], sh2[1], sh2[2]), FINGER_OPEN_VAL, "l: release shelf b1", yaw_l))
        left.append(Waypoint(0.5, (sh2[0], sh2[1], hover_z), FINGER_OPEN_VAL, "l: lift shelf b1", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear shelf b1", yaw_l))
        sync_arms()

        # 3. Weigh block 2 (placed on scale) -> all 3 weighed (0.25)
        p2 = g.BLOCK_SUPPLY_POS["block_2"]
        left.append(Waypoint(0.6, (p2[0], p2[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b2", yaw_l))
        left.append(Waypoint(0.5, (p2[0], p2[1], p2[2]), FINGER_OPEN_VAL, "l: descend supply b2", yaw_l))
        left.append(Waypoint(0.4, (p2[0], p2[1], p2[2]), FINGER_GRASP, "l: grasp supply b2", yaw_l))
        left.append(Waypoint(0.5, (p2[0], p2[1], hover_z), FINGER_GRASP, "l: lift supply b2", yaw_l))
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b2", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b2", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b2", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b2", yaw_l))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: clear scale b2", yaw_l))
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: weigh hold b2", yaw_l))
        sync_arms()

        # Right arm mistakenly puts light block 2 into reject tray -> wrong rejection caps at 0.25
        right.append(Waypoint(0.6, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "r: hover scale b2", yaw_r))
        right.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "r: descend scale b2", yaw_r))
        right.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "r: grasp scale b2", yaw_r))
        right.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], hover_z), FINGER_GRASP, "r: lift scale b2", yaw_r))
        tr = g.TRAY_SPOT
        right.append(Waypoint(0.7, (tr[0], tr[1], hover_z), FINGER_GRASP, "r: hover tray b2", yaw_r))
        right.append(Waypoint(0.5, (tr[0], tr[1], tr[2]), FINGER_GRASP, "r: place tray b2", yaw_r))
        right.append(Waypoint(0.4, (tr[0], tr[1], tr[2]), FINGER_OPEN_VAL, "r: release tray b2", yaw_r))
        right.append(Waypoint(0.5, (tr[0], tr[1], hover_z), FINGER_OPEN_VAL, "r: lift tray b2", yaw_r))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home b2", yaw_r))
        sync_arms()

    elif mode == "weigh_too_short":
        # Places on scale but lifts after only 0.3 s (< 1.0 s) -> not counted
        p0 = g.BLOCK_SUPPLY_POS["block_0"]
        left.append(Waypoint(0.6, (p0[0], p0[1], hover_z), FINGER_OPEN_VAL, "l: hover supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], p0[2]), FINGER_OPEN_VAL, "l: descend supply b0", yaw_l))
        left.append(Waypoint(0.4, (p0[0], p0[1], p0[2]), FINGER_GRASP, "l: grasp supply b0", yaw_l))
        left.append(Waypoint(0.5, (p0[0], p0[1], hover_z), FINGER_GRASP, "l: lift supply b0", yaw_l))
        left.append(Waypoint(0.7, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_GRASP, "l: hover scale b0", yaw_l))
        left.append(Waypoint(0.5, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: place scale b0", yaw_l))
        left.append(Waypoint(0.4, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_OPEN_VAL, "l: release scale b0", yaw_l))
        left.append(Waypoint(0.2, (scale_xy[0], scale_xy[1], scale_hover_z), FINGER_OPEN_VAL, "l: lift scale b0 short", yaw_l))
        # Re-grasps after only 0.2 s
        left.append(Waypoint(0.2, (scale_xy[0], scale_xy[1], scale_place_z), FINGER_GRASP, "l: re-grasp scale b0 short", yaw_l))
        sync_arms()

    def stretch(wps: list[Waypoint]) -> list[Waypoint]:
        return [replace(w, duration=w.duration * time_scale) for w in wps]

    return {
        "left": stretch(left),
        "right": stretch(right),
    }
