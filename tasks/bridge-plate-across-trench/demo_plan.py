"""Bimanual demonstration plans and ablations for bridge-plate-across-trench.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.

Modes:
- demo: Left arm places bridge across trench (y=0, 20 mm overlap on both lips).
        Right arm pushes block across bridge onto far platform target mark with 25 mm overshoot.
- no_bridge_push: Right arm pushes block into trench without laying bridge -> falls (0.00).
- bridge_short_overlap: Left arm places bridge with insufficient overlap -> tips into trench (0.00).
- skip_target: Left arm places bridge, Right arm pushes block onto bridge and leaves it there (0.50).
- settle: Arms hold at home stances (0.00).
- dream: Dream policy baseline (0.00).
"""

from __future__ import annotations

import os
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_OPEN_VAL = g.FINGER_OPEN_VAL
FINGER_GRASP_BRIDGE = float(os.environ.get("DEMO_GRASP_FINGER", g.FINGER_GRASP_BRIDGE))
FINGER_CLOSED_VAL = g.FINGER_CLOSED_VAL


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_BRIDGE_PUSH") == "1":
        return "no_bridge_push"
    if os.environ.get("DEMO_BRIDGE_SHORT_OVERLAP") == "1":
        return "bridge_short_overlap"
    if os.environ.get("DEMO_SKIP_TARGET") == "1":
        return "skip_target"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", 1.8) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.HOVER_Z)
    place_dz = _f("DEMO_PLACE_DZ", 0.005)
    rel_hold = _f("DEMO_RELEASE_HOLD_S", 0.35)

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, 0.8500)

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
        right.append(Waypoint(14.4, home_r, FINGER_CLOSED_VAL, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Initial ready stance
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: ready", 0.0))
    sync_arms()

    # Bridge placement parameters
    z_bridge_top = g.PLATFORM_TOP_Z + g.BRIDGE_SIZE[2]
    z_grasp_bridge = g.HANDLE_CENTER_Z + PAD_TO_TCP + _f("DEMO_GRASP_DZ", 0.0)
    hx = g.BRIDGE_START_POS[0] + g.HANDLE_OFFSET_X   # grasp the handle, not the plate
    bridge_pick_xy = (hx, g.BRIDGE_START_POS[1])


    # Block pushes. Relay (measured reach): the left arm can reach the block's start side (y > +0.05) but not the far
    # platform; the right arm can reach y <= ~+0.10 and the target. So the left arm pushes the block over the near
    # lip onto the bridge centre, then the right arm pushes it on to the target mark.
    z_transit = _f("DEMO_TRANSIT_Z", 0.880)   # must clear the block top (0.8145 on the bridge) by > 50 mm
    z_push = g.PLATFORM_TOP_Z + _f("DEMO_PUSH_DZ", 0.028)   # pad centre = TCP - 13 mm: 0.028 puts it at block mid-height
    bx = g.BLOCK_START_POS[0]
    mid_tcp_y = _f("DEMO_MID_PUSH_Y", 0.010)       # left pusher TCP y that leaves the block centred on the bridge
    far_tcp_y = _f("DEMO_FAR_PUSH_Y", g.BLOCK_TARGET_POS[1] + 0.033 - 0.005)  # right pusher TCP y (5 mm overshoot)

    behind = _f("DEMO_RIGHT_BEHIND", 0.060)   # right pusher start y behind the block's resting y

    def left_push_to_bridge():
        left.append(Waypoint(0.5 * time_scale, (bx, 0.215, z_transit), FINGER_CLOSED_VAL, "l: hover behind block", 0.0))
        left.append(Waypoint(0.4 * time_scale, (bx, 0.195, z_push), FINGER_CLOSED_VAL, "l: descend behind block", 0.0))
        left.append(Waypoint(1.0 * time_scale, (bx, mid_tcp_y, z_push), FINGER_CLOSED_VAL, "l: push block onto bridge", 0.0))
        left.append(Waypoint(0.3 * time_scale, (bx, mid_tcp_y, z_push), FINGER_CLOSED_VAL, "l: hold on bridge", 0.0))
        left.append(Waypoint(0.4 * time_scale, (bx, mid_tcp_y, z_transit), FINGER_CLOSED_VAL, "l: lift off block", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))

    def right_push_to_target():
        right.append(Waypoint(0.5 * time_scale, (0.220, -0.100, hover_z), FINGER_CLOSED_VAL, "r: approach", 0.0))
        right.append(Waypoint(0.5 * time_scale, (bx, mid_tcp_y + behind + 0.005, hover_z), FINGER_CLOSED_VAL, "r: hover behind block", 0.0))
        right.append(Waypoint(0.4 * time_scale, (bx, mid_tcp_y + behind, z_push), FINGER_CLOSED_VAL, "r: descend behind block", 0.0))
        # fast to the creep start, then a slow creep so the block never carries momentum past the mark
        creep_y = _f("DEMO_FAR_CREEP_Y", -0.060)
        right.append(Waypoint(0.5 * time_scale, (bx, creep_y, z_push), FINGER_CLOSED_VAL, "r: push block over bridge", 0.0))
        right.append(Waypoint(_f("DEMO_FAR_CREEP_S", 2.5) * time_scale, (bx, far_tcp_y, z_push), FINGER_CLOSED_VAL, "r: creep block to target", 0.0))
        right.append(Waypoint(0.8 * time_scale, (bx, far_tcp_y, z_push), FINGER_CLOSED_VAL, "r: hold at target", 0.0))
        right.append(Waypoint(0.4 * time_scale, (bx, far_tcp_y + _f("DEMO_FAR_BACKOFF", 0.012), z_push), FINGER_CLOSED_VAL, "r: back off block", 0.0))
        right.append(Waypoint(0.3 * time_scale, (bx, far_tcp_y + _f("DEMO_FAR_BACKOFF", 0.012), z_transit), FINGER_CLOSED_VAL, "r: lift clear", 0.0))
        right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: return home", 0.0))

    # === ABLATION: NO_BRIDGE_PUSH ===
    if mode == "no_bridge_push":
        # no bridge: the left arm pushes the block at the trench, it falls in
        left_push_to_bridge()
        sync_arms()

        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === ABLATION: BRIDGE_SHORT_OVERLAP ===
    if mode == "bridge_short_overlap":
        # Left arm places bridge at y = +0.035 (doesn't span far lip, tips into trench)
        place_y = 0.035
        left.append(Waypoint(0.5 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], hover_z), FINGER_OPEN_VAL, "l: hover bridge", 0.0))
        left.append(Waypoint(0.4 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], z_grasp_bridge), FINGER_OPEN_VAL, "l: descend bridge", 0.0))
        left.append(Waypoint(0.3 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], z_grasp_bridge), FINGER_GRASP_BRIDGE, "l: grasp bridge", 0.0))
        left.append(Waypoint(0.4 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], hover_z), FINGER_GRASP_BRIDGE, "l: lift bridge", 0.0))
        left.append(Waypoint(0.5 * time_scale, (hx, place_y, hover_z), FINGER_GRASP_BRIDGE, "l: transit bridge short", 0.0))
        left.append(Waypoint(0.4 * time_scale, (hx, place_y, z_grasp_bridge + place_dz), FINGER_GRASP_BRIDGE, "l: place bridge short", 0.0))
        left.append(Waypoint(rel_hold * time_scale, (hx, place_y, z_grasp_bridge + place_dz), FINGER_OPEN_VAL, "l: release bridge", 0.0))
        left.append(Waypoint(0.4 * time_scale, (hx, place_y, hover_z), FINGER_OPEN_VAL, "l: clear bridge", 0.0))
        left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()

        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === NORMAL BRIDGE PLACEMENT (Left Arm) ===
    # 1. Hover above bridge plate
    left.append(Waypoint(0.5 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], hover_z), FINGER_OPEN_VAL, "l: hover bridge", 0.0))
    # 2. Descend to grasp bridge
    left.append(Waypoint(0.4 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], z_grasp_bridge), FINGER_OPEN_VAL, "l: descend bridge", 0.0))
    # 3. Grasp narrow 80 mm side
    left.append(Waypoint(0.3 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], z_grasp_bridge), FINGER_GRASP_BRIDGE, "l: grasp bridge", 0.0))
    # 4. Lift straight up
    left.append(Waypoint(0.4 * time_scale, (bridge_pick_xy[0], bridge_pick_xy[1], hover_z), FINGER_GRASP_BRIDGE, "l: lift bridge", 0.0))
    # 5. Lateral transit to trench center (0.300, 0.000)
    left.append(Waypoint(0.5 * time_scale, (hx, 0.000, hover_z), FINGER_GRASP_BRIDGE, "l: transit bridge", 0.0))
    # 6. Descend to place across trench
    left.append(Waypoint(0.4 * time_scale, (hx, 0.000, z_grasp_bridge + place_dz), FINGER_GRASP_BRIDGE, "l: place bridge", 0.0))
    # 7. Release
    left.append(Waypoint(rel_hold * time_scale, (hx, 0.000, z_grasp_bridge + place_dz), FINGER_OPEN_VAL, "l: release bridge", 0.0))
    # 8. Lift straight up
    left.append(Waypoint(0.4 * time_scale, (hx, 0.000, hover_z), FINGER_OPEN_VAL, "l: clear bridge", 0.0))
    # 9. Return home
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
    sync_arms()

    # === LEFT ARM PUSHES THE BLOCK ONTO THE BRIDGE ===
    left_push_to_bridge()
    sync_arms()

    # === ABLATION: SKIP_TARGET ===
    if mode == "skip_target":
        left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        right.append(Waypoint(2.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: final settle", 0.0))
        return {"left": left, "right": right}

    # === RIGHT ARM PUSHES THE BLOCK TO THE TARGET MARK ===
    right_push_to_target()
    sync_arms()

    # Final settle hold (2.5 s)
    left.append(Waypoint(2.5 * time_scale, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
    right.append(Waypoint(2.5 * time_scale, home_r, FINGER_CLOSED_VAL, "r: final settle", 0.0))

    return {"left": left, "right": right}
