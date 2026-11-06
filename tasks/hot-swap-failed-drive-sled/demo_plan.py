"""Bimanual demonstration plans and ablations for hot-swap-failed-drive-sled.

Uses yam_arena.demo.Waypoint.
"""

from __future__ import annotations

import os
from dataclasses import replace
import numpy as np

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_GRASP = -0.006          # firm grip on the 45 mm handle (commanded well past contact for a strong squeeze)
FINGER_OPEN_VAL = FINGER_OPEN  # fully open (-0.04695)
FINGER_PRESS = 0.0             # closed fingers for button pressing


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NO_LATCH") == "1":
        return "no_latch"
    if os.environ.get("DEMO_WRONG_BAY") == "1":
        return "wrong_bay"
    if os.environ.get("DEMO_WRONG_SPARE") == "1":
        return "wrong_spare"
    if os.environ.get("DEMO_RELEASE_EARLY") == "1":
        return "release_early"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    insert_dz = _f("DEMO_INSERT_DZ", 0.0)
    insert_dy = _f("DEMO_INSERT_DY", -0.005)   # the held spare sits ~5 mm to the left of the fingertips

    hover_z = g.HOVER_Z
    grasp_z = g.GRASP_Z + insert_dz
    press_z = g.LATCH_PRESS_Z

    home_l = (0.200, +0.280, hover_z)
    home_r = (0.200, -0.280, hover_z)

    latch_xy = g.latch_xy(g.FAILED_BAY)
    handle_seated = (g.HANDLE_SEATED_X, g.BAY_Y[g.FAILED_BAY], grasp_z)
    handle_pulled = (g.HANDLE_SEATED_X - 0.080, g.BAY_Y[g.FAILED_BAY], grasp_z)

    spare_color = "orange" if mode == "wrong_spare" else "blue"
    spare_pos = g.SPARE_POS[spare_color]
    spare_handle = (spare_pos[0] + g.HANDLE_OFFSET_X, spare_pos[1] + insert_dy, spare_pos[2] + _f("DEMO_SPARE_GRASP_DZ", 0.0))  # handle centre height on the stand

    # the handle sits 90 mm in front of the sled centre, so the fingertip target is offset to put the sled centre on the pad
    discard_spot = (g.DISCARD_SPOT[0] + g.HANDLE_OFFSET_X, g.DISCARD_SPOT[1], grasp_z)

    right: list[Waypoint] = []
    left: list[Waypoint] = []

    def sync_arms():
        t_l = sum(w.duration for w in right)
        t_r = sum(w.duration for w in left)
        if t_l < t_r:
            right.append(Waypoint(t_r - t_l, right[-1].pos, right[-1].finger, "r: sync wait", right[-1].yaw))
        elif t_r < t_l:
            left.append(Waypoint(t_l - t_r, left[-1].pos, left[-1].finger, "l: sync wait", left[-1].yaw))

    if mode == "settle":
        right.append(Waypoint(14.4, home_r, FINGER_OPEN_VAL, "r: settle hold", 0.0))
        left.append(Waypoint(14.4, home_l, FINGER_OPEN_VAL, "l: settle hold", 0.0))
        return {"right": right, "left": left}

    # === STEP 1: INITIAL STANCE ===
    right.append(Waypoint(0.5, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    sync_arms()

    # === STEP 2: LEFT HOLDS THE LATCH, RIGHT GRIPS THE HANDLE AND PULLS THE FAILED SLED ===
    target_handle = (g.HANDLE_SEATED_X, g.BAY_Y["bay_2"], grasp_z) if mode == "wrong_bay" else handle_seated
    pull_len = 0.080 if mode == "wrong_bay" else _f("DEMO_PULL_LEN", 0.160)
    pulled_x = target_handle[0] - pull_len
    pull_time = 0.8 if mode == "wrong_bay" else 1.2

    # Left presses the latch first and keeps it down (it stays out of the way while the right hand works below it)
    if mode != "no_latch":
        left.append(Waypoint(0.6, (latch_xy[0], latch_xy[1], hover_z), FINGER_PRESS, "l: hover latch", 0.0))
        left.append(Waypoint(0.5, (latch_xy[0], latch_xy[1], press_z), FINGER_PRESS, "l: press latch", 0.0))
        if mode == "release_early":
            left.append(Waypoint(0.3, (latch_xy[0], latch_xy[1], press_z), FINGER_PRESS, "l: hold latch", 0.0))
            left.append(Waypoint(0.5, (latch_xy[0], latch_xy[1], hover_z), FINGER_OPEN_VAL, "l: release latch early", 0.0))
    sync_arms()

    # Right takes the handle
    right.append(Waypoint(0.6, (target_handle[0] - 0.050, target_handle[1], hover_z), FINGER_OPEN_VAL, "r: hover handle", 0.0))
    right.append(Waypoint(0.5, (target_handle[0] - 0.050, target_handle[1], grasp_z), FINGER_OPEN_VAL, "r: align handle", 0.0))
    # the palm stops about 12 mm short of the handle centre against the rack front, so grip there and never push the sled back
    grip = (target_handle[0] - 0.012, target_handle[1], grasp_z)
    right.append(Waypoint(0.4, grip, FINGER_OPEN_VAL, "r: approach handle", 0.0))
    right.append(Waypoint(0.4, grip, FINGER_GRASP, "r: grasp handle", 0.0))
    if mode not in ("no_latch", "release_early"):
        left.append(Waypoint(1.9, (latch_xy[0], latch_xy[1], press_z), FINGER_PRESS, "l: hold latch", 0.0))
    sync_arms()

    # Right pulls the sled forward (fully out of its bay; 80 mm only in the wrong_bay ablation)
    right.append(Waypoint(pull_time, (pulled_x, target_handle[1], grasp_z), FINGER_GRASP, "r: pull sled", 0.0))
    if mode not in ("no_latch", "release_early"):
        left.append(Waypoint(pull_time + 0.4, (latch_xy[0], latch_xy[1], press_z), FINGER_PRESS, "l: hold latch while pulling", 0.0))
    sync_arms()

    if mode == "wrong_bay":
        right.append(Waypoint(0.5, (pulled_x - 0.050, target_handle[1], hover_z), FINGER_OPEN_VAL, "r: clear fault", 0.0))
        right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
        left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()
        right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
        left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))
        return {"right": right, "left": left}

    # Right releases latch if held and retreats to home_l
    if mode != "no_latch" and mode != "release_early":
        left.append(Waypoint(0.5, (latch_xy[0], latch_xy[1], hover_z), FINGER_OPEN_VAL, "l: lift latch", 0.0))
    left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home left", 0.0))

    # === STEP 3: RIGHT PLACES THE OLD SLED ON THE DISCARD PAD ===
    right.append(Waypoint(0.5, (pulled_x, target_handle[1], hover_z), FINGER_GRASP, "r: lift old sled", 0.0))
    right.append(Waypoint(0.7, (discard_spot[0], discard_spot[1], hover_z), FINGER_GRASP, "r: hover discard", 0.0))
    right.append(Waypoint(0.5, discard_spot, FINGER_GRASP, "r: place discard", 0.0))
    right.append(Waypoint(0.4, discard_spot, FINGER_OPEN_VAL, "r: release discard", 0.0))
    right.append(Waypoint(0.5, (discard_spot[0], discard_spot[1], hover_z), FINGER_OPEN_VAL, "r: lift discard", 0.0))
    right.append(Waypoint(0.6, home_r, FINGER_OPEN_VAL, "r: return home right", 0.0))
    sync_arms()

    # === STEP 4: LEFT FETCHES & INSERTS THE SPARE ===
    left.append(Waypoint(0.6, (spare_handle[0], spare_handle[1], hover_z), FINGER_OPEN_VAL, "l: hover spare", 0.0))
    left.append(Waypoint(0.5, spare_handle, FINGER_OPEN_VAL, "l: descend spare", 0.0))
    left.append(Waypoint(0.4, spare_handle, FINGER_GRASP, "l: grasp spare", 0.0))
    left.append(Waypoint(0.5, (spare_handle[0], spare_handle[1], hover_z), FINGER_GRASP, "l: lift spare", 0.0))

    mouth_x = g.RACK_FRONT_X - 0.160   # spare still clear of the bay mouth, aligned and lowered to bay height
    seated_x = g.HANDLE_SEATED_X
    bay_y = g.BAY_Y[g.FAILED_BAY] + insert_dy

    left.append(Waypoint(0.7, (mouth_x, bay_y, hover_z), FINGER_GRASP, "l: hover mouth", 0.0))
    # the spare hangs ~5 mm below the fingertip line and the bay floor is 10 mm above the table, so carry it at fingertip height of the bay
    ins_z = grasp_z + _f("DEMO_INSERT_LIFT", 0.0)
    left.append(Waypoint(0.5, (mouth_x, bay_y, ins_z), FINGER_GRASP, "l: align mouth", 0.0))
    left.append(Waypoint(1.0, (seated_x, bay_y, ins_z), FINGER_GRASP, "l: insert spare", 0.0))
    left.append(Waypoint(0.4, (seated_x, bay_y, ins_z), FINGER_OPEN_VAL, "l: release spare", 0.0))
    left.append(Waypoint(0.5, (seated_x - 0.060, bay_y, hover_z), FINGER_OPEN_VAL, "l: lift spare release", 0.0))
    left.append(Waypoint(0.6, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
    sync_arms()

    # === STEP 5: FINAL SETTLE ===
    right.append(Waypoint(1.5, home_r, FINGER_OPEN_VAL, "r: final settle", 0.0))
    left.append(Waypoint(1.5, home_l, FINGER_OPEN_VAL, "l: final settle", 0.0))

    return {"right": right, "left": left}


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` with every waypoint duration stretched by DEMO_TIME_SCALE (default 1.8) so the arms keep up."""
    if mode is None:
        mode = mode_from_env()
    result = _plans(mode)
    if mode in ("settle", "dream"):
        return result
    scale = _f("DEMO_TIME_SCALE", 1.8)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
