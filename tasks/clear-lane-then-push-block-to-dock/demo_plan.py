"""Demo waypoint plans and ablation variations for clear-lane-then-push-block-to-dock."""

from __future__ import annotations

import os
from dataclasses import dataclass
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint


MODES = ("demo", "push_first", "hit_post", "stop_halfway", "settle", "dream")


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, str(default)))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_PUSH_FIRST") == "1":
        return "push_first"
    if os.environ.get("DEMO_HIT_POST") == "1":
        return "hit_post"
    if os.environ.get("DEMO_STOP_HALFWAY") == "1":
        return "stop_halfway"
    if os.environ.get("DEMO_DREAM") == "1":
        return "dream"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", g.DEMO_TIME_SCALE) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.DEMO_HOVER_Z)
    push_z = _f("DEMO_PUSH_Z", g.PUSH_Z)
    push_x = _f("DEMO_PUSH_X", g.PUSH_END_X)
    push_hold_s = _f("DEMO_PUSH_HOLD_S", g.DEMO_PUSH_HOLD_S)
    retract_x = _f("DEMO_RETRACT_X", 0.300)
    final_hold_s = _f("DEMO_FINAL_HOLD_S", g.DEMO_FINAL_HOLD_S)

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

    if mode in ("settle", "dream"):
        left.append(Waypoint(15.0, home_l, g.FINGER_OPEN, "l: settle hold", 0.0))
        right.append(Waypoint(15.0, home_r, g.FINGER_OPEN, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Ready pose
    left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, g.FINGER_OPEN, "r: ready", 0.0))

    if mode == "push_first":
        # Skip clearing loose blocks: Left arm pushes directly into blockage
        left.append(Waypoint(0.5 * time_scale, (0.090, 0.000, hover_z), g.FINGER_CLOSED, "l: push transit", 0.0))
        left.append(Waypoint(0.4 * time_scale, (0.090, 0.000, push_z), g.FINGER_CLOSED, "l: push descend", 0.0))
        left.append(Waypoint(0.6 * time_scale, (0.180, 0.000, push_z), g.FINGER_CLOSED, "l: push stall", 0.0))
        left.append(Waypoint(0.4 * time_scale, (0.180, 0.000, hover_z), g.FINGER_CLOSED, "l: push lift", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: return home", 0.0))
        sync_arms()
        left.append(Waypoint(final_hold_s * time_scale, home_l, g.FINGER_OPEN, "l: final hold", 0.0))
        right.append(Waypoint(final_hold_s * time_scale, home_r, g.FINGER_OPEN, "r: final hold", 0.0))
        return {"left": left, "right": right}

    # Phase 1: Left arm lifts Blue block out of lane to left
    b_start = g.BLUE_START_POS
    b_stage = g.BLUE_STAGING_POS
    left.append(Waypoint(0.5 * time_scale, (b_start[0], b_start[1], hover_z), g.FINGER_OPEN, "l: blue hover", 0.0))
    left.append(Waypoint(0.4 * time_scale, (b_start[0], b_start[1], b_start[2]), g.FINGER_OPEN, "l: blue descend", 0.0))
    left.append(Waypoint(0.3 * time_scale, (b_start[0], b_start[1], b_start[2]), g.FINGER_GRASP_BLOCK, "l: blue grasp", 0.0))
    left.append(Waypoint(0.4 * time_scale, (b_start[0], b_start[1], hover_z), g.FINGER_GRASP_BLOCK, "l: blue lift", 0.0))
    left.append(Waypoint(0.5 * time_scale, (b_stage[0], b_stage[1], hover_z), g.FINGER_GRASP_BLOCK, "l: blue carry", 0.0))
    left.append(Waypoint(0.4 * time_scale, (b_stage[0], b_stage[1], b_stage[2]), g.FINGER_GRASP_BLOCK, "l: blue lower", 0.0))
    left.append(Waypoint(0.3 * time_scale, (b_stage[0], b_stage[1], b_stage[2]), g.FINGER_OPEN, "l: blue release", 0.0))
    left.append(Waypoint(0.4 * time_scale, (b_stage[0], b_stage[1], hover_z), g.FINGER_OPEN, "l: blue lift clear", 0.0))
    left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: return home 1", 0.0))

    # Phase 2: Right arm lifts Yellow block out of lane to right (runs sequentially for max clearance)
    sync_arms()
    y_start = g.YELLOW_START_POS
    y_stage = g.YELLOW_STAGING_POS
    right.append(Waypoint(0.5 * time_scale, (y_start[0], y_start[1], hover_z), g.FINGER_OPEN, "r: yellow hover", 0.0))
    right.append(Waypoint(0.4 * time_scale, (y_start[0], y_start[1], y_start[2]), g.FINGER_OPEN, "r: yellow descend", 0.0))
    right.append(Waypoint(0.3 * time_scale, (y_start[0], y_start[1], y_start[2]), g.FINGER_GRASP_BLOCK, "r: yellow grasp", 0.0))
    right.append(Waypoint(0.4 * time_scale, (y_start[0], y_start[1], hover_z), g.FINGER_GRASP_BLOCK, "r: yellow lift", 0.0))
    right.append(Waypoint(0.5 * time_scale, (y_stage[0], y_stage[1], hover_z), g.FINGER_GRASP_BLOCK, "r: yellow carry", 0.0))
    right.append(Waypoint(0.4 * time_scale, (y_stage[0], y_stage[1], y_stage[2]), g.FINGER_GRASP_BLOCK, "r: yellow lower", 0.0))
    right.append(Waypoint(0.3 * time_scale, (y_stage[0], y_stage[1], y_stage[2]), g.FINGER_OPEN, "r: yellow release", 0.0))
    right.append(Waypoint(0.4 * time_scale, (y_stage[0], y_stage[1], hover_z), g.FINGER_OPEN, "r: yellow lift clear", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, g.FINGER_OPEN, "r: return home 2", 0.0))

    # Phase 3: Left arm pushes Cargo block along lane to dock
    sync_arms()
    left.append(Waypoint(0.5 * time_scale, (0.090, 0.000, hover_z), g.FINGER_CLOSED, "l: push transit", 0.0))
    left.append(Waypoint(0.4 * time_scale, (0.090, 0.000, push_z), g.FINGER_CLOSED, "l: push descend", 0.0))

    if mode == "stop_halfway":
        # Push 80 mm past start (to x=0.240) then stop
        left.append(Waypoint(0.8 * time_scale, (0.240, 0.000, push_z), g.FINGER_CLOSED, "l: push halfway", 0.0))
        left.append(Waypoint(0.4 * time_scale, (0.240, 0.000, hover_z), g.FINGER_CLOSED, "l: push lift halfway", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: return home 3", 0.0))
    elif mode == "hit_post":
        # Push toward dock and veer into red post at (0.340, +0.090)
        # cl_5: a veer at push height never reached the post (the rail is in the way and the pad rides over it). Lift to the
        # post's mid-height (above the 20 mm rail, below the 80 mm post top), cross over the rail and sweep through the post.
        post_hit_z = _f("DEMO_POST_HIT_Z", 0.800)
        post_sweep_x = _f("DEMO_POST_SWEEP_X", g.POST_START_POS[0] + 0.030)
        left.append(Waypoint(1.0 * time_scale, (0.320, 0.000, push_z), g.FINGER_CLOSED, "l: push dock", 0.0))
        left.append(Waypoint(0.4 * time_scale, (0.320, 0.000, post_hit_z), g.FINGER_CLOSED, "l: lift to post height", 0.0))
        left.append(Waypoint(0.5 * time_scale, (g.POST_START_POS[0] - 0.040, g.POST_START_POS[1], post_hit_z), g.FINGER_CLOSED, "l: cross rail", 0.0))
        left.append(Waypoint(0.5 * time_scale, (post_sweep_x, g.POST_START_POS[1], post_hit_z), g.FINGER_CLOSED, "l: sweep hit post", 0.0))
        left.append(Waypoint(0.4 * time_scale, (post_sweep_x, g.POST_START_POS[1], hover_z), g.FINGER_CLOSED, "l: lift after post", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: return home 3", 0.0))
    else:
        # Standard demo push with 25 mm overshoot past mark center
        left.append(Waypoint(1.2 * time_scale, (push_x, 0.000, push_z), g.FINGER_CLOSED, "l: push dock", 0.0))
        left.append(Waypoint(push_hold_s * time_scale, (push_x, 0.000, push_z), g.FINGER_CLOSED, "l: push hold", 0.0))
        left.append(Waypoint(0.4 * time_scale, (retract_x, 0.000, push_z), g.FINGER_CLOSED, "l: push retract", 0.0))
        left.append(Waypoint(0.4 * time_scale, (retract_x, 0.000, hover_z), g.FINGER_CLOSED, "l: push lift", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, g.FINGER_OPEN, "l: return home 3", 0.0))

    # Phase 4: Final hold
    sync_arms()
    left.append(Waypoint(final_hold_s * time_scale, home_l, g.FINGER_OPEN, "l: final hold", 0.0))
    right.append(Waypoint(final_hold_s * time_scale, home_r, g.FINGER_OPEN, "r: final hold", 0.0))

    return {"left": left, "right": right}
