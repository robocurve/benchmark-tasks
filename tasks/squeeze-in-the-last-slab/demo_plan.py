"""Demonstration plans and ablations for squeeze-in-the-last-slab.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.

Demo sequence (the fifth slab starts standing on its edge at (0.190, +0.220); the arms grip straight
down and cannot turn a flat slab on end):
1. Left arm descends over the fifth slab and grips it across its 60 mm width (jaw gap ~42 mm).
2. Lifts it to hover height, carries it above the slot (0.325, +0.040), descends centrally into the
   38 mm clear gap between the neighbours without contact, releases, lifts straight up, returns home.
3. Both arms hold at home for >= 2.6 s (scoring 2.0 s stable hold).

Ablations:
- knock_neighbour: descends off-centre by DEMO_KNOCK_DX (15 mm), pushing a neighbour slab.
- stop_over_gap: leaves the slab hovering above the gap -> score 0.00.
- settle: both arms hold home idle -> 0.00.
- dream: baseline policy -> 0.00.
"""

from __future__ import annotations

import os
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint

FINGER_OPEN = g.FINGER_OPEN
FINGER_GRASP_WIDTH = g.FINGER_GRASP_WIDTH


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_KNOCK_NEIGHBOUR") == "1":
        return "knock_neighbour"
    if os.environ.get("DEMO_STOP_OVER_GAP") == "1":
        return "stop_over_gap"
    if os.environ.get("DEMO_DREAM") == "1":
        return "dream"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", g.DEMO_TIME_SCALE) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.DEMO_HOVER_Z)
    final_hold_s = _f("DEMO_FINAL_HOLD_S", g.DEMO_FINAL_HOLD_S)

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    insert_z = g.SLAB_STANDING_Z + g.DEMO_INSERT_DZ  # 0.7905 m

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
        left.append(Waypoint(15.0, home_l, FINGER_OPEN, "l: settle hold", 0.0))
        right.append(Waypoint(15.0, home_r, FINGER_OPEN, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Ready pose
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_OPEN, "r: ready", 0.0))

    # Phase 1: left arm grips the standing slab across its 60 mm width
    x5, y5 = g.SLAB_5_START[0], g.SLAB_5_START[1]
    grip_z = g.SLAB_STANDING_Z + _f("DEMO_GRIP_DZ", 0.0)
    grasp_s = _f("DEMO_GRASP_S", 0.8)
    left.append(Waypoint(0.5 * time_scale, (x5, y5, hover_z), FINGER_OPEN, "l: hover slab", 0.0))
    left.append(Waypoint(0.6 * time_scale, (x5, y5, grip_z), FINGER_OPEN, "l: descend slab", 0.0))
    left.append(Waypoint(grasp_s * time_scale, (x5, y5, grip_z), FINGER_GRASP_WIDTH, "l: grip slab", 0.0))
    left.append(Waypoint(0.5 * time_scale, (x5, y5, hover_z), FINGER_GRASP_WIDTH, "l: lift slab", 0.0))

    # Phase 4: Carry above gap and insert
    target_x = g.SLOT_TARGET_POS[0] - g.DEMO_KNOCK_DX if mode == "knock_neighbour" else g.SLOT_TARGET_POS[0]
    target_desc = "l: transit gap (knock)" if mode == "knock_neighbour" else "l: transit gap"

    left.append(Waypoint(0.5 * time_scale, (target_x, g.SLOT_TARGET_POS[1], hover_z), FINGER_GRASP_WIDTH, target_desc, 0.0))

    if mode == "stop_over_gap":
        left.append(Waypoint(final_hold_s * time_scale, (target_x, g.SLOT_TARGET_POS[1], hover_z), FINGER_GRASP_WIDTH, "l: hover idle", 0.0))
        sync_arms()
        right.append(Waypoint(final_hold_s * time_scale, home_r, FINGER_OPEN, "r: final hold", 0.0))
        return {"left": left, "right": right}

    insert_s = _f("DEMO_INSERT_S", 1.2)
    left.append(Waypoint(insert_s * time_scale, (target_x, g.SLOT_TARGET_POS[1], insert_z), FINGER_GRASP_WIDTH, "l: descend insert", 0.0))
    left.append(Waypoint(0.5 * time_scale, (target_x, g.SLOT_TARGET_POS[1], insert_z), FINGER_GRASP_WIDTH, "l: hold insert", 0.0))
    left.append(Waypoint(0.6 * time_scale, (target_x, g.SLOT_TARGET_POS[1], insert_z), FINGER_OPEN, "l: release insert", 0.0))
    left.append(Waypoint(0.4 * time_scale, (target_x, g.SLOT_TARGET_POS[1], hover_z), FINGER_OPEN, "l: lift after insert", 0.0))
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN, "l: return home", 0.0))

    # Phase 5: Final hold
    sync_arms()
    left.append(Waypoint(final_hold_s * time_scale, home_l, FINGER_OPEN, "l: final hold", 0.0))
    right.append(Waypoint(final_hold_s * time_scale, home_r, FINGER_OPEN, "r: final hold", 0.0))

    return {"left": left, "right": right}
