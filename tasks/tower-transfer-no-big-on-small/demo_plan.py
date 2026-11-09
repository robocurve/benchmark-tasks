"""Bimanual demonstration plans and ablations for tower-transfer-no-big-on-small.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.
Optimal demo: 7-move Tower of Hanoi.
"""

from __future__ import annotations

import os
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_OPEN_VAL = g.FINGER_OPEN_VAL
FINGER_GRASP_LARGE = g.FINGER_GRASP_LARGE
FINGER_GRASP_MED = g.FINGER_GRASP_MED
FINGER_GRASP_SMALL = g.FINGER_GRASP_SMALL


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_BIG_ON_SMALL") == "1":
        return "big_on_small"
    if os.environ.get("DEMO_LEAVE_ONE") == "1":
        return "leave_one"
    if os.environ.get("DEMO_WRONG_TARGET") == "1":
        return "wrong_target"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", 1.8) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.HOVER_Z)
    place_dz = _f("DEMO_PLACE_DZ", 0.005)
    rel_hold = _f("DEMO_RELEASE_HOLD_S", 0.35)
    # measured: blocks set on the RIGHT mark came to rest ~10 mm short in x; these knobs shift only the RIGHT placements
    r_dx = _f("DEMO_R_PLACE_DX", 0.008)
    r_dy = _f("DEMO_R_PLACE_DY", 0.0)

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
        left.append(Waypoint(14.4, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        right.append(Waypoint(14.4, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Stance
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    def do_pick_and_place(arm_name: str, pick_xy: tuple[float, float], pick_z: float,
                          place_xy: tuple[float, float], place_z: float,
                          grasp_val: float, tag: str):
        if abs(place_xy[0] - g.RIGHT_MARK_POS[0]) < 1e-6 and abs(place_xy[1] - g.RIGHT_MARK_POS[1]) < 1e-6:
            place_xy = (place_xy[0] + r_dx, place_xy[1] + r_dy)
        arm_list = left if arm_name == "left" else right
        p_char = "l" if arm_name == "left" else "r"
        h_pos = home_l if arm_name == "left" else home_r

        # 1. Hover above pick
        arm_list.append(Waypoint(0.5 * time_scale, (pick_xy[0], pick_xy[1], hover_z), FINGER_OPEN_VAL, f"{p_char}: hover {tag}", 0.0))
        # 2. Descend to grasp
        arm_list.append(Waypoint(0.4 * time_scale, (pick_xy[0], pick_xy[1], pick_z), FINGER_OPEN_VAL, f"{p_char}: descend {tag}", 0.0))
        # 3. Grasp
        arm_list.append(Waypoint(0.3 * time_scale, (pick_xy[0], pick_xy[1], pick_z), grasp_val, f"{p_char}: grasp {tag}", 0.0))
        # 4. Lift straight up
        arm_list.append(Waypoint(0.4 * time_scale, (pick_xy[0], pick_xy[1], hover_z), grasp_val, f"{p_char}: lift {tag}", 0.0))
        # 5. Lateral transit to place
        arm_list.append(Waypoint(0.5 * time_scale, (place_xy[0], place_xy[1], hover_z), grasp_val, f"{p_char}: transit {tag}", 0.0))
        # 6. Descend to place
        arm_list.append(Waypoint(0.4 * time_scale, (place_xy[0], place_xy[1], place_z + place_dz), grasp_val, f"{p_char}: place {tag}", 0.0))
        # 7. Release
        arm_list.append(Waypoint(rel_hold * time_scale, (place_xy[0], place_xy[1], place_z + place_dz), FINGER_OPEN_VAL, f"{p_char}: release {tag}", 0.0))
        # 8. Lift straight up before lateral motion (lesson 6)
        arm_list.append(Waypoint(0.4 * time_scale, (place_xy[0], place_xy[1], hover_z), FINGER_OPEN_VAL, f"{p_char}: clear {tag}", 0.0))
        # 9. Return home
        arm_list.append(Waypoint(0.5 * time_scale, h_pos, FINGER_OPEN_VAL, f"{p_char}: return home", 0.0))
        sync_arms()

    # Targets
    xy_l = (g.LEFT_MARK_POS[0], g.LEFT_MARK_POS[1])
    xy_m = (g.MID_MARK_POS[0], g.MID_MARK_POS[1])
    xy_r = (g.RIGHT_MARK_POS[0], g.RIGHT_MARK_POS[1])

    z_l1 = g.LEVEL_1_TARGET_Z
    z_l2 = g.LEVEL_2_TARGET_Z
    z_l3 = g.LEVEL_3_TARGET_Z

    # === ABLATION: BIG_ON_SMALL ===
    if mode == "big_on_small":
        # Move 1: S -> MID
        do_pick_and_place("left", xy_l, z_l3, xy_m, z_l1, FINGER_GRASP_SMALL, "m1_s_to_mid")
        # Move 2: M -> MID (Medium placed on Small on MID! Illegal!)
        do_pick_and_place("left", xy_l, z_l2, xy_m, z_l2, FINGER_GRASP_MED, "m2_m_on_s_illegal")
        # Hold
        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === ABLATION: WRONG_TARGET ===
    if mode == "wrong_target":
        # Move tower to MIDDLE instead of RIGHT
        # 1. S -> R
        do_pick_and_place("left", xy_l, z_l3, xy_r, z_l1, FINGER_GRASP_SMALL, "wt_s_to_r")
        # 2. M -> MID
        do_pick_and_place("left", xy_l, z_l2, xy_m, z_l1, FINGER_GRASP_MED, "wt_m_to_mid")
        # 3. S -> MID
        do_pick_and_place("right", xy_r, z_l1, xy_m, z_l2, FINGER_GRASP_SMALL, "wt_s_to_mid")
        # 4. L -> R
        do_pick_and_place("left", xy_l, z_l1, xy_r, z_l1, FINGER_GRASP_LARGE, "wt_l_to_r")
        # Settle here (Large at R, but tower on MID) -> 0.50
        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === OPTIMAL 7-MOVE SEQUENCE ===
    # Move 1: S -> RIGHT
    do_pick_and_place("left", xy_l, z_l3, xy_r, z_l1, FINGER_GRASP_SMALL, "m1_s_to_r")

    # Move 2: M -> MIDDLE
    do_pick_and_place("left", xy_l, z_l2, xy_m, z_l1, FINGER_GRASP_MED, "m2_m_to_mid")

    # Move 3: S -> MIDDLE (on M)
    do_pick_and_place("right", xy_r, z_l1, xy_m, z_l2, FINGER_GRASP_SMALL, "m3_s_to_mid")

    # Move 4: L -> RIGHT
    do_pick_and_place("left", xy_l, z_l1, xy_r, z_l1, FINGER_GRASP_LARGE, "m4_l_to_r")

    # Move 5: S -> LEFT
    do_pick_and_place("left", xy_m, z_l2, xy_l, z_l1, FINGER_GRASP_SMALL, "m5_s_to_l")

    # Move 6: M -> RIGHT (on L)
    do_pick_and_place("right", xy_m, z_l1, xy_r, z_l2, FINGER_GRASP_MED, "m6_m_to_r")

    # === ABLATION: LEAVE_ONE (Stop after Move 6) ===
    if mode == "leave_one":
        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # Move 7: S -> RIGHT (on M)
    do_pick_and_place("left", xy_l, z_l1, xy_r, z_l3, FINGER_GRASP_SMALL, "m7_s_to_r")

    # Final settle hold (2.5 s)
    left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(2.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))

    return {"left": left, "right": right}

