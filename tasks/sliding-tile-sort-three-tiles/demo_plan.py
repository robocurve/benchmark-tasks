"""Demonstration plans and ablations for sliding-tile-sort-three-tiles.

Uses yam_arena.demo.Waypoint with DEMO_TIME_SCALE stretching.

Demo sequence (5-move sliding puzzle solution):
Move 1: Right arm pushes Tile B from cell 3 to cell 1 (+x push).
Move 2: Left arm pushes Tile O from cell 2 to cell 3 (-y push).
Move 3: Left arm pushes Tile Y from cell 0 to cell 2 (-x push, Y in goal).
Move 4: Right arm pushes Tile B from cell 1 to cell 0 (+y push, B in goal).
Move 5: Right arm pushes Tile O from cell 3 to cell 1 (+x push, O in goal).
Final: Both arms hold at home for >= 2.6 s (scoring 2.0 s stable hold).

Ablations:
- wrong_order: Sequence stops after Move 4 (2 tiles in goal cells) -> score 0.50.
- lift_tile: Left arm lifts a tile > 12 mm above frame -> escaped zero-gate latches, score 0.00.
- settle: Both arms hold home idle -> 0.00.
- dream: Baseline policy -> 0.00.
"""

from __future__ import annotations

import os
import numpy as np

import geometry as g
from yam_arena.demo import Waypoint

FINGER_OPEN = g.FINGER_OPEN
FINGER_CLOSED = g.FINGER_CLOSED
FINGER_GRASP = g.FINGER_GRASP


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_WRONG_ORDER") == "1":
        return "wrong_order"
    if os.environ.get("DEMO_LIFT_TILE") == "1":
        return "lift_tile"
    if os.environ.get("DEMO_DREAM") == "1":
        return "dream"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    time_scale = _f("DEMO_TIME_SCALE", g.DEMO_TIME_SCALE) if mode not in ("settle", "dream") else 1.0
    hover_z = _f("DEMO_HOVER_Z", g.DEMO_HOVER_Z)
    push_z = g.DEMO_PUSH_Z
    push_hold_s = _f("DEMO_PUSH_HOLD_S", g.DEMO_PUSH_HOLD_S)
    final_hold_s = _f("DEMO_FINAL_HOLD_S", g.DEMO_FINAL_HOLD_S)
    push_s = _f("DEMO_PUSH_S", 1.2)
    # Push end points, calibrated on the simulator. Runs with the original +20 mm overshoot moved
    # tiles 2, 3 and 4 into their goal cells but drove the blue tile onto the frame rim in the +x push
    # (tile z 0.777 > 0.7745, "escaped" latched, score 0). The +x pushes therefore stop earlier.
    ov = _f("DEMO_PUSH_OVERSHOOT", g.DEMO_PUSH_OVERSHOOT)
    xe1 = _f("DEMO_PUSH_XEND", 0.305)   # moves 1 and 5 (+x, right arm): tile lands ~0.31, inside cell 1
    ye2 = _f("DEMO_PUSH_Y2END", -0.040)  # move 2 (-y): tile follows the tip 1:1; -0.058 rode the -y rim (z 0.779)
    xe3 = _f("DEMO_PUSH_X3END", 0.240)   # move 3 (-x): tile follows the tip; 0.222 rode the -x rim (z 0.776)
    ye4 = _f("DEMO_PUSH_Y4END", 0.072)   # move 4 (+y, right arm near reach): a 0.050 command left the tile at y 0.004

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
        left.append(Waypoint(15.0, home_l, FINGER_OPEN, "l: settle hold", 0.0))
        right.append(Waypoint(15.0, home_r, FINGER_OPEN, "r: settle hold", 0.0))
        return {"left": left, "right": right}

    # Ready pose
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_CLOSED, "l: ready", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED, "r: ready", 0.0))

    if mode == "lift_tile":
        # Left arm grasps and lifts Yellow tile out of frame
        left.append(Waypoint(0.5 * time_scale, (g.CELL_0[0], g.CELL_0[1], hover_z), FINGER_OPEN, "l: hover tile", 0.0))
        left.append(Waypoint(0.4 * time_scale, (g.CELL_0[0], g.CELL_0[1], g.TILE_REST_Z), FINGER_OPEN, "l: descend tile", 0.0))
        left.append(Waypoint(0.3 * time_scale, (g.CELL_0[0], g.CELL_0[1], g.TILE_REST_Z), FINGER_GRASP, "l: grasp tile", 0.0))
        left.append(Waypoint(0.5 * time_scale, (g.CELL_0[0], g.CELL_0[1], hover_z), FINGER_GRASP, "l: lift tile out", 0.0))
        left.append(Waypoint(final_hold_s * time_scale, (g.CELL_0[0], g.CELL_0[1], hover_z), FINGER_GRASP, "l: hold lifted", 0.0))
        sync_arms()
        right.append(Waypoint(final_hold_s * time_scale, home_r, FINGER_CLOSED, "r: final hold", 0.0))
        return {"left": left, "right": right}

    # --- Move 1: Right arm pushes Tile B from cell 3 to cell 1 along +x ---
    # Start: x = 0.185, y = -0.038. End (tip command): x = xe1 = 0.305, y = -0.038
    right.append(Waypoint(0.5 * time_scale, (0.185, -0.038, hover_z), FINGER_CLOSED, "r: hover m1", 0.0))
    right.append(Waypoint(0.3 * time_scale, (0.185, -0.038, push_z), FINGER_CLOSED, "r: descend m1", 0.0))
    right.append(Waypoint(push_s * time_scale, (xe1, -0.038, push_z), FINGER_CLOSED, "r: push m1 B to 1", 0.0))
    right.append(Waypoint(push_hold_s * time_scale, (xe1, -0.038, push_z), FINGER_CLOSED, "r: hold m1", 0.0))
    right.append(Waypoint(0.3 * time_scale, (xe1, -0.038, hover_z), FINGER_CLOSED, "r: lift m1", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED, "r: return home m1", 0.0))
    sync_arms()

    # --- Move 2: Left arm pushes Tile O from cell 2 to cell 3 along -y ---
    # Start: x = 0.242, y = +0.095. End (tip command): x = 0.242, y = ye2 = -0.040
    left.append(Waypoint(0.5 * time_scale, (0.242, +0.095, hover_z), FINGER_CLOSED, "l: hover m2", 0.0))
    left.append(Waypoint(0.3 * time_scale, (0.242, +0.095, push_z), FINGER_CLOSED, "l: descend m2", 0.0))
    left.append(Waypoint(push_s * time_scale, (0.242, ye2, push_z), FINGER_CLOSED, "l: push m2 O to 3", 0.0))
    left.append(Waypoint(push_hold_s * time_scale, (0.242, ye2, push_z), FINGER_CLOSED, "l: hold m2", 0.0))
    left.append(Waypoint(0.3 * time_scale, (0.242, ye2, hover_z), FINGER_CLOSED, "l: lift m2", 0.0))
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_CLOSED, "l: return home m2", 0.0))
    sync_arms()

    # --- Move 3: Left arm pushes Tile Y from cell 0 to cell 2 along -x ---
    # Start: x = 0.375, y = +0.038. End (tip command): x = xe3 = 0.240, y = +0.038
    left.append(Waypoint(0.5 * time_scale, (0.375, +0.038, hover_z), FINGER_CLOSED, "l: hover m3", 0.0))
    left.append(Waypoint(0.3 * time_scale, (0.375, +0.038, push_z), FINGER_CLOSED, "l: descend m3", 0.0))
    left.append(Waypoint(push_s * time_scale, (xe3, +0.038, push_z), FINGER_CLOSED, "l: push m3 Y to 2", 0.0))
    left.append(Waypoint(push_hold_s * time_scale, (xe3, +0.038, push_z), FINGER_CLOSED, "l: hold m3", 0.0))
    left.append(Waypoint(0.3 * time_scale, (xe3, +0.038, hover_z), FINGER_CLOSED, "l: lift m3", 0.0))
    left.append(Waypoint(0.5 * time_scale, home_l, FINGER_CLOSED, "l: return home m3", 0.0))
    sync_arms()

    # --- Move 4: Right arm pushes Tile B from cell 1 to cell 0 along +y ---
    # Start: x = 0.318, y = -0.095. End (tip command): x = 0.318, y = ye4 = 0.072
    right.append(Waypoint(0.5 * time_scale, (0.318, -0.095, hover_z), FINGER_CLOSED, "r: hover m4", 0.0))
    right.append(Waypoint(0.3 * time_scale, (0.318, -0.095, push_z), FINGER_CLOSED, "r: descend m4", 0.0))
    right.append(Waypoint(push_s * time_scale, (0.318, ye4, push_z), FINGER_CLOSED, "r: push m4 B to 0", 0.0))
    right.append(Waypoint(push_hold_s * time_scale, (0.318, ye4, push_z), FINGER_CLOSED, "r: hold m4", 0.0))
    right.append(Waypoint(0.4 * time_scale, (0.318, -0.038, push_z + 0.010), FINGER_CLOSED, "r: retract m4", 0.0))
    right.append(Waypoint(0.3 * time_scale, (0.318, -0.038, hover_z), FINGER_CLOSED, "r: lift m4", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED, "r: return home m4", 0.0))
    sync_arms()

    if mode == "wrong_order":
        left.append(Waypoint(final_hold_s * time_scale, home_l, FINGER_CLOSED, "l: final hold", 0.0))
        right.append(Waypoint(final_hold_s * time_scale, home_r, FINGER_CLOSED, "r: final hold", 0.0))
        return {"left": left, "right": right}

    # --- Move 5: Right arm pushes Tile O from cell 3 to cell 1 along +x ---
    # Start: x = 0.185, y = -0.038. End (tip command): x = xe1 = 0.305, y = -0.038
    right.append(Waypoint(0.5 * time_scale, (0.185, -0.038, hover_z), FINGER_CLOSED, "r: hover m5", 0.0))
    right.append(Waypoint(0.3 * time_scale, (0.185, -0.038, push_z), FINGER_CLOSED, "r: descend m5", 0.0))
    right.append(Waypoint(push_s * time_scale, (xe1, -0.038, push_z), FINGER_CLOSED, "r: push m5 O to 1", 0.0))
    right.append(Waypoint(push_hold_s * time_scale, (xe1, -0.038, push_z), FINGER_CLOSED, "r: hold m5", 0.0))
    right.append(Waypoint(0.3 * time_scale, (xe1, -0.038, hover_z), FINGER_CLOSED, "r: lift m5", 0.0))
    right.append(Waypoint(0.5 * time_scale, home_r, FINGER_CLOSED, "r: return home m5", 0.0))
    sync_arms()

    # Final hold at home
    left.append(Waypoint(final_hold_s * time_scale, home_l, FINGER_CLOSED, "l: final hold", 0.0))
    right.append(Waypoint(final_hold_s * time_scale, home_r, FINGER_CLOSED, "r: final hold", 0.0))

    return {"left": left, "right": right}
