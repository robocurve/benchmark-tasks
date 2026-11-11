"""Bimanual demonstration plans and ablations for domino-chain-set-and-tip.

Sequence:
1. Left arm places first 2 dominoes (Domino 0 on Mark 0, Domino 1 on Mark 1).
2. Right arm places second 2 dominoes (Domino 2 on Mark 2, Domino 3 on Mark 3).
3. Straight up release (lesson 6) after each placement.
4. Nearest arm (Left) tips the first domino at 60 mm height with closed finger push.
5. All four dominoes topple in chain reaction toward +x.

Ablations:
- demo: full sequence (predicted 1.00)
- gap_too_wide: domino placed with gap too wide (predicted 0.75)
- tip_too_early: tips first domino before second arm places (predicted 0.25)
- no_tip: places all 4 dominoes standing but does not tip (predicted 0.50)
- settle: arms hold at home (predicted 0.00)
- dream: baseline policy (predicted 0.00)
"""

from __future__ import annotations

import os
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint
import geometry as g

FINGER_OPEN_VAL = FINGER_OPEN          # -0.04695
FINGER_GRASP_DOMINO = g.FINGER_GRASP_DOMINO  # -0.020
FINGER_CLOSED_VAL = g.FINGER_CLOSED_VAL      # 0.0


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_GAP_TOO_WIDE") == "1":
        return "gap_too_wide"
    if os.environ.get("DEMO_TIP_TOO_EARLY") == "1":
        return "tip_too_early"
    if os.environ.get("DEMO_NO_TIP") == "1":
        return "no_tip"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", 1.8) if mode not in ("settle", "dream") else 1.0

    hover_z = _f("DEMO_HOVER_Z", 0.840)
    grasp_z = _f("DEMO_GRASP_Z", 0.785)
    place_z = _f("DEMO_PLACE_Z", 0.785)
    push_z = _f("DEMO_PUSH_Z", g.PUSH_Z)
    rel_hold = _f("DEMO_RELEASE_HOLD_S", 0.35)

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

    # === SETTLE MODE ===
    if mode == "settle":
        left.append(Waypoint(14.4, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        right.append(Waypoint(14.4, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Initial ready stance
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    def do_pick_and_place(arm_name: str, pick_xy: tuple[float, float], place_xy: tuple[float, float], tag: str):
        arm_list = left if arm_name == "left" else right
        p_char = "l" if arm_name == "left" else "r"
        h_pos = home_l if arm_name == "left" else home_r

        # For the right arm reaching the far domino (mark 3), use an intermediate waypoint
        is_far_right = (arm_name == "right" and place_xy[0] >= 0.400)
        t_h = 0.800 if is_far_right else hover_z

        # 1. Hover above pick
        arm_list.append(Waypoint(0.5 * time_scale, (pick_xy[0], pick_xy[1], hover_z), FINGER_OPEN_VAL, f"{p_char}: hover {tag}", 0.0))
        # 2. Descend to grasp
        arm_list.append(Waypoint(0.4 * time_scale, (pick_xy[0], pick_xy[1], grasp_z), FINGER_OPEN_VAL, f"{p_char}: descend {tag}", 0.0))
        # 3. Grasp domino
        arm_list.append(Waypoint(0.3 * time_scale, (pick_xy[0], pick_xy[1], grasp_z), FINGER_GRASP_DOMINO, f"{p_char}: grasp {tag}", 0.0))
        # 4. Lift straight up
        arm_list.append(Waypoint(0.4 * time_scale, (pick_xy[0], pick_xy[1], hover_z if not is_far_right else 0.820), FINGER_GRASP_DOMINO, f"{p_char}: lift {tag}", 0.0))
        # 5. Lateral transit to mark
        if is_far_right:
            arm_list.append(Waypoint(0.5 * time_scale, (_f("DEMO_FAR_X", 0.410), -0.100, t_h), FINGER_GRASP_DOMINO, f"{p_char}: transit {tag}", 0.0))
        else:
            arm_list.append(Waypoint(0.5 * time_scale, (place_xy[0], place_xy[1], t_h), FINGER_GRASP_DOMINO, f"{p_char}: transit {tag}", 0.0))
        # 6. Descend to place on mark
        arm_list.append(Waypoint(0.4 * time_scale, (place_xy[0], place_xy[1], place_z), FINGER_GRASP_DOMINO, f"{p_char}: place {tag}", 0.0))
        # 7. Release
        arm_list.append(Waypoint(rel_hold * time_scale, (place_xy[0], place_xy[1], place_z), FINGER_OPEN_VAL, f"{p_char}: release {tag}", 0.0))
        # 8. Lift straight up before lateral motion (lesson 6)
        if is_far_right:
            arm_list.append(Waypoint(0.4 * time_scale, (_f("DEMO_FAR_X", 0.410), -0.100, t_h), FINGER_OPEN_VAL, f"{p_char}: clear {tag}", 0.0))
        else:
            arm_list.append(Waypoint(0.4 * time_scale, (place_xy[0], place_xy[1], t_h), FINGER_OPEN_VAL, f"{p_char}: clear {tag}", 0.0))
        # 9. Return home
        arm_list.append(Waypoint(0.5 * time_scale, h_pos, FINGER_OPEN_VAL, f"{p_char}: return home", 0.0))
        sync_arms()

    def do_tip_first_domino():
        # Nearest arm (left) tips first domino (Mark 0 at x=0.26) at 60 mm height with closed finger push
        # 1. Hover above push start (behind domino at x=0.230)
        left.append(Waypoint(0.5 * time_scale, (0.230, 0.000, hover_z), FINGER_CLOSED_VAL, "l: hover push", 0.0))
        # 2. Descend to push height (60 mm height)
        left.append(Waypoint(0.4 * time_scale, (0.230, 0.000, push_z), FINGER_CLOSED_VAL, "l: descend push", 0.0))
        # 3. Closed finger push forward into first domino
        left.append(Waypoint(_f("DEMO_PUSH_DUR_S", 0.12) * time_scale, (_f("DEMO_PUSH_END_X", 0.290), 0.000, push_z), FINGER_CLOSED_VAL, "l: push domino 0", 0.0))
        # 4. Lift straight up
        left.append(Waypoint(0.4 * time_scale, (_f("DEMO_PUSH_END_X", 0.290), 0.000, hover_z), FINGER_CLOSED_VAL, "l: clear push", 0.0))
        # 5. Return home (clear arms)
        left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home after push", 0.0))
        sync_arms()

    # Domino start positions
    p0 = (g.DOMINO_0_START_POS[0], g.DOMINO_0_START_POS[1])
    p1 = (g.DOMINO_1_START_POS[0], g.DOMINO_1_START_POS[1])
    p2 = (g.DOMINO_2_START_POS[0], g.DOMINO_2_START_POS[1])
    p3 = (g.DOMINO_3_START_POS[0], g.DOMINO_3_START_POS[1])

    # Mark positions
    m0 = (g.MARK_0_POS[0], g.MARK_0_POS[1])
    m1 = (g.MARK_1_POS[0], g.MARK_1_POS[1])
    m2 = (g.MARK_2_POS[0], g.MARK_2_POS[1])
    m3 = (g.MARK_3_POS[0], g.MARK_3_POS[1])

    # === STEP 1: LEFT ARM PLACES FIRST 2 DOMINOES ===
    do_pick_and_place("left", p0, m0, "domino_0_to_mark_0")
    do_pick_and_place("left", p1, m1, "domino_1_to_mark_1")

    # === ABLATION: TIP_TOO_EARLY ===
    if mode == "tip_too_early":
        # Left arm prematurely tips domino 0 before Right arm places dominoes 2 & 3
        do_tip_first_domino()
        left.append(Waypoint(3.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(3.0 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 2: RIGHT ARM PLACES SECOND 2 DOMINOES ===
    if mode == "gap_too_wide":
        # Place domino 2 with gap too wide (x=0.400 instead of 0.360, gap 70 mm > 30 mm)
        m2_wide = (0.400, 0.000)
        do_pick_and_place("right", p2, m2_wide, "domino_2_wide_gap")
    else:
        do_pick_and_place("right", p2, m2, "domino_2_to_mark_2")

    do_pick_and_place("right", p3, m3, "domino_3_to_mark_3")

    # === ABLATION: NO_TIP ===
    if mode == "no_tip":
        # Settle without tipping
        left.append(Waypoint(3.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(3.0 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === STEP 3: NEAREST ARM TIPS FIRST DOMINO ===
    do_tip_first_domino()

    # Final hold: wait for chain to topple and settle with arms clear
    left.append(Waypoint(3.0 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(3.0 * time_scale, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))

    return {"left": left, "right": right}
