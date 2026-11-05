"""Scripted demo for the star-pattern cold-plate bolt torque task.

Bimanual coordination:
- The left arm manages bolts on the left side: L_near (0.225, +0.105) and L_far (0.335, +0.105).
- The right arm manages bolts on the right side: R_near (0.225, -0.105) and R_far (0.335, -0.105).
- Diagonal partner pairs: (L_near, R_far) and (L_far, R_near).
- One press at a time: while the active arm hovers, presses, holds, and lifts, the idle arm waits at its
  home park pose. Hands maintain >= 100 mm clearance at all times (actual distance >= 325 mm).

Waypoint structure: (duration, pos, finger, label, yaw) played open-loop by yam_scripted.
All tuning knobs exposed as environment variables (DEMO_*):
- DEMO_TIME_SCALE: stretch factor for waypoint durations (default 1.8x)
- DEMO_PRESS_DZ: vertical offset adjustment on press TCP z (m, default 0.0)
- DEMO_YAW_SHIFT_LEFT: yaw adjustment for left arm (rad, default 0.0)
- DEMO_YAW_SHIFT_RIGHT: yaw adjustment for right arm (rad, default 0.0)
- DEMO_SETTLE_ONLY: if 1, returns empty plan so arms hold initial ready pose (scores 0.0)
- DEMO_ADJACENT: if 1, presses neighbouring corners in a row (L_near then L_far) -> latches WARP 0.0
- DEMO_ONE_ARM: if 1, left arm presses only its 2 bolts twice -> latches WARP by LEVEL 0.0
- DEMO_NO_STAR: if 1, right stage levels but wrong within-pass order -> latches WARP 0.0
"""

from __future__ import annotations

import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TIP_PAST_TCP = g.PAD_TIP_PAST_TCP
FINGER_BTN = -0.015    # closed fist (30 mm between pads) for pressing buttons
FINGER_OPEN_VAL = FINGER_OPEN  # 0.047 m


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_ADJACENT") == "1":
        return "adjacent"
    if os.environ.get("DEMO_ONE_ARM") == "1":
        return "one_arm"
    if os.environ.get("DEMO_NO_STAR") == "1":
        return "no_star"
    return "demo"


def bolt_press_sequence(mode: str) -> list[str]:
    """Return the ordered list of corner names to press for the given mode."""
    # Standard star pattern: 4 corners per pass, 3 passes = 12 presses
    # Pass: L_near -> R_far (diagonal 1), R_near -> L_far (diagonal 2)
    star_pass = ["L_near", "R_far", "R_near", "L_far"]
    if mode == "demo":
        return star_pass * 3
    if mode == "adjacent":
        # Press L_near (pass 1, p1), then press neighbouring L_far (p2, not diagonal) -> WARP
        return ["L_near", "L_far"]
    if mode == "one_arm":
        # Left arm presses only its 2 bolts twice: L_near (1), L_far (WARP on p2 by star order, or by level on repeat)
        return ["L_near", "L_far", "L_near", "L_far"]
    if mode == "no_star":
        # Right stage levels (each bolt pressed once per pass), but non-star within-pass order:
        # e.g. L_near, R_near (adjacent), L_far, R_far (repeated 3 times)
        no_star_pass = ["L_near", "R_near", "L_far", "R_far"]
        return no_star_pass * 3
    if mode == "settle":
        return []
    raise ValueError(f"Unknown mode: {mode}")


def demo_plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    if mode == "settle":
        return {}

    press_dz = _f("DEMO_PRESS_DZ", 0.0)
    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)

    hover_z = g.HOVER_Z
    press_z = g.BUTTON_PRESS_TCP_Z + press_dz

    home_l = g.HOME_L
    home_r = g.HOME_R

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    # Initial ready waypoint
    left.append(Waypoint(0.5, home_l, FINGER_BTN, "l: ready", yaw_l))
    right.append(Waypoint(0.5, home_r, FINGER_BTN, "r: ready", yaw_r))

    corners = bolt_press_sequence(mode)
    for idx, corner in enumerate(corners):
        pass_idx = idx // 4 + 1
        pos_xy = g.BOLT_POS_XY[corner]
        arm = g.BOLT_ARM[corner]
        is_left = (arm == "left")

        act_wps = left if is_left else right
        idl_wps = right if is_left else left
        act_yaw = yaw_l if is_left else yaw_r
        idl_yaw = yaw_r if is_left else yaw_l
        idl_home = home_r if is_left else home_l

        pfx = "l" if is_left else "r"
        idl_pfx = "r" if is_left else "l"

        # Active arm: hover -> press -> hold -> lift
        # Durations unstretched: 0.7s hover, 1.1s press, 0.6s hold, 0.5s lift
        act_wps.append(Waypoint(0.7, (pos_xy[0], pos_xy[1], hover_z), FINGER_BTN, f"{pfx}: hover {corner} #{idx+1} p{pass_idx}", act_yaw))
        act_wps.append(Waypoint(1.1, (pos_xy[0], pos_xy[1], press_z), FINGER_BTN, f"{pfx}: press {corner} #{idx+1} p{pass_idx}", act_yaw))
        act_wps.append(Waypoint(0.6, (pos_xy[0], pos_xy[1], press_z), FINGER_BTN, f"{pfx}: hold {corner} #{idx+1} p{pass_idx}", act_yaw))
        act_wps.append(Waypoint(0.5, (pos_xy[0], pos_xy[1], hover_z), FINGER_BTN, f"{pfx}: lift {corner} #{idx+1} p{pass_idx}", act_yaw))

        # Idle arm: waits at home during the 2.9 s active cycle
        cycle_time = 0.7 + 1.1 + 0.6 + 0.5
        idl_wps.append(Waypoint(cycle_time, idl_home, FINGER_BTN, f"{idl_pfx}: wait for {corner} #{idx+1}", idl_yaw))

    # Final hold with open grippers
    hold_s = 1.5
    left.append(Waypoint(hold_s, home_l, FINGER_OPEN_VAL, "l: final hold", yaw_l))
    right.append(Waypoint(hold_s, home_r, FINGER_OPEN_VAL, "r: final hold", yaw_r))

    # Stretch 1.8x
    scale = _f("DEMO_TIME_SCALE", 1.8)
    if scale != 1.0:
        left[:] = [replace(w, duration=w.duration * scale) for w in left]
        right[:] = [replace(w, duration=w.duration * scale) for w in right]

    return {"left": left, "right": right}


def plans() -> dict[str, list[Waypoint]]:
    return demo_plans()
