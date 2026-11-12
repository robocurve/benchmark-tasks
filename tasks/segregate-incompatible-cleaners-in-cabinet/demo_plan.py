"""Demonstration motion plans and ablations for segregate-incompatible-cleaners-in-cabinet.

Constructs open-loop ArmPlan waypoints for dual YAM arms.
"""

from __future__ import annotations

import os
from yam_arena.demo import Waypoint
import geometry as g

FINGER_OPEN = g.FINGER_OPEN
FINGER_GRASP = g.FINGER_GRASP

HOME_L = g.HOME_LEFT
HOME_R = g.HOME_RIGHT


def _pad_sync(wps_a: list[Waypoint], wps_b: list[Waypoint], label: str = "sync_wait") -> None:
    t_a = sum(w.duration for w in wps_a)
    t_b = sum(w.duration for w in wps_b)
    if t_a < t_b:
        wps_a.append(Waypoint(duration=t_b - t_a, pos=wps_a[-1].pos, finger=wps_a[-1].finger, label=label))
    elif t_b < t_a:
        wps_b.append(Waypoint(duration=t_a - t_b, pos=wps_b[-1].pos, finger=wps_b[-1].finger, label=label))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_SAME_BAY") == "1":
        return "same_bay_violation"
    if os.environ.get("DEMO_ONE_BOTTLE") == "1":
        return "one_bottle_only"
    return "demo"


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    variant = os.environ.get("TASK_VARIANT", "A").upper()
    if variant not in ("A", "B"):
        variant = "A"

    left_wps: list[Waypoint] = [Waypoint(duration=1.0, pos=HOME_L, finger=FINGER_OPEN, label="home_ready")]
    right_wps: list[Waypoint] = [Waypoint(duration=1.0, pos=HOME_R, finger=FINGER_OPEN, label="home_ready")]

    if mode == "settle":
        left_wps.append(Waypoint(duration=10.0, pos=HOME_L, finger=FINGER_OPEN, label="settle_home"))
        right_wps.append(Waypoint(duration=10.0, pos=HOME_R, finger=FINGER_OPEN, label="settle_home"))
        return {"left": left_wps, "right": right_wps}

    # Staging picks:
    # Left arm picks staging +Y bottle at (0.220, +0.290)
    # Right arm picks staging -Y bottle at (0.220, -0.290)
    pick_l = (g.STAGING_X, g.STAGING_LEFT_Y, g.GRASP_Z)
    pick_r = (g.STAGING_X, g.STAGING_RIGHT_Y, g.GRASP_Z)

    if mode == "same_bay_violation":
        # Hazardous proximity: bottles placed 110 mm apart (< 120 mm violation threshold)
        place_l = (g.CABINET_X, 0.055, g.GRASP_Z)
        place_r = (g.CABINET_X, -0.055, g.GRASP_Z)

        # Sequential motion to ensure huge collision clearance
        # Step 1: Left arm places at +0.055
        left_wps.append(Waypoint(duration=1.0, pos=(pick_l[0], pick_l[1], g.HOVER_Z), finger=FINGER_OPEN, label="l: hover_pick"))
        left_wps.append(Waypoint(duration=0.8, pos=pick_l, finger=FINGER_OPEN, label="l: descend_pick"))
        left_wps.append(Waypoint(duration=0.5, pos=pick_l, finger=FINGER_GRASP, label="l: grasp"))
        left_wps.append(Waypoint(duration=0.8, pos=(pick_l[0], pick_l[1], g.HOVER_Z), finger=FINGER_GRASP, label="l: lift"))
        left_wps.append(Waypoint(duration=1.2, pos=(place_l[0], place_l[1], g.HOVER_Z), finger=FINGER_GRASP, label="l: carry_place"))
        left_wps.append(Waypoint(duration=0.8, pos=place_l, finger=FINGER_GRASP, label="l: seat_well"))
        left_wps.append(Waypoint(duration=0.5, pos=place_l, finger=FINGER_OPEN, label="l: release"))
        left_wps.append(Waypoint(duration=0.8, pos=(place_l[0], place_l[1], g.HOVER_Z), finger=FINGER_OPEN, label="l: retract"))
        left_wps.append(Waypoint(duration=1.2, pos=HOME_L, finger=FINGER_OPEN, label="l: return_home"))

        _pad_sync(left_wps, right_wps, "r: wait_left")

        # Step 2: Right arm places at -0.055
        right_wps.append(Waypoint(duration=1.0, pos=(pick_r[0], pick_r[1], g.HOVER_Z), finger=FINGER_OPEN, label="r: hover_pick"))
        right_wps.append(Waypoint(duration=0.8, pos=pick_r, finger=FINGER_OPEN, label="r: descend_pick"))
        right_wps.append(Waypoint(duration=0.5, pos=pick_r, finger=FINGER_GRASP, label="r: grasp"))
        right_wps.append(Waypoint(duration=0.8, pos=(pick_r[0], pick_r[1], g.HOVER_Z), finger=FINGER_GRASP, label="r: lift"))
        right_wps.append(Waypoint(duration=1.2, pos=(place_r[0], place_r[1], g.HOVER_Z), finger=FINGER_GRASP, label="r: carry_place"))
        right_wps.append(Waypoint(duration=0.8, pos=place_r, finger=FINGER_GRASP, label="r: seat_well"))
        right_wps.append(Waypoint(duration=0.5, pos=place_r, finger=FINGER_OPEN, label="r: release"))
        right_wps.append(Waypoint(duration=0.8, pos=(place_r[0], place_r[1], g.HOVER_Z), finger=FINGER_OPEN, label="r: retract"))
        right_wps.append(Waypoint(duration=1.2, pos=HOME_R, finger=FINGER_OPEN, label="r: return_home"))

        _pad_sync(left_wps, right_wps, "l: wait_right")

        left_wps.append(Waypoint(duration=2.6, pos=HOME_L, finger=FINGER_OPEN, label="l: final_hold"))
        right_wps.append(Waypoint(duration=2.6, pos=HOME_R, finger=FINGER_OPEN, label="r: final_hold"))
        _pad_sync(left_wps, right_wps, "done")
        return {"left": left_wps, "right": right_wps}

    # Normal demo & one_bottle_only:
    # Left Bay target (+Y): (0.250, +0.130)
    # Right Bay target (-Y): (0.250, -0.130)
    # Seat height: bottle centre is 27 mm below the TCP, well floor top is 0.7535 m, bottle half-height 105 mm,
    # so the base touches the floor at TCP z = 0.7535 + 0.105 + 0.027 = 0.8855 m (0.8865 leaves 1 mm to drop)
    place_z = g.WELL_SEAT_TCP_Z
    place_l = (g.CABINET_X, g.BAY_LEFT_Y, place_z)
    place_r = (g.CABINET_X, g.BAY_RIGHT_Y, place_z)

    # 1. Hover above staging
    left_wps.append(Waypoint(duration=1.0, pos=(pick_l[0], pick_l[1], g.HOVER_Z), finger=FINGER_OPEN, label="l: hover_pick"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=1.0, pos=(pick_r[0], pick_r[1], g.HOVER_Z), finger=FINGER_OPEN, label="r: hover_pick"))

    # 2. Descend to grasp contoured bottle neck
    left_wps.append(Waypoint(duration=0.8, pos=pick_l, finger=FINGER_OPEN, label="l: descend_pick"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.8, pos=pick_r, finger=FINGER_OPEN, label="r: descend_pick"))

    # 3. Firm grip
    left_wps.append(Waypoint(duration=0.5, pos=pick_l, finger=FINGER_GRASP, label="l: grasp"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.5, pos=pick_r, finger=FINGER_GRASP, label="r: grasp"))

    # 4. Lift clear of table surface
    left_wps.append(Waypoint(duration=0.8, pos=(pick_l[0], pick_l[1], g.HOVER_Z), finger=FINGER_GRASP, label="l: lift"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.8, pos=(pick_r[0], pick_r[1], g.HOVER_Z), finger=FINGER_GRASP, label="r: lift"))

    # 5. Carry over cabinet well
    left_wps.append(Waypoint(duration=1.2, pos=(place_l[0], place_l[1], g.HOVER_Z), finger=FINGER_GRASP, label="l: carry_place"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=1.2, pos=(place_r[0], place_r[1], g.HOVER_Z), finger=FINGER_GRASP, label="r: carry_place"))

    # 6. Lower into recessed well
    left_wps.append(Waypoint(duration=1.0, pos=(place_l[0], place_l[1], g.HOVER_Z), finger=FINGER_GRASP, label="l: hover_settle"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=1.0, pos=(place_r[0], place_r[1], g.HOVER_Z), finger=FINGER_GRASP, label="r: hover_settle"))
    left_wps.append(Waypoint(duration=0.8, pos=place_l, finger=FINGER_GRASP, label="l: seat_well"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.8, pos=place_r, finger=FINGER_GRASP, label="r: seat_well"))

    # 7. Release grip
    left_wps.append(Waypoint(duration=0.5, pos=place_l, finger=FINGER_OPEN, label="l: release"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.5, pos=place_r, finger=FINGER_OPEN, label="r: release"))

    # 8. Retract vertically above well
    left_wps.append(Waypoint(duration=0.8, pos=(place_l[0], place_l[1], g.HOVER_Z), finger=FINGER_OPEN, label="l: retract"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.8, pos=(place_r[0], place_r[1], g.HOVER_Z), finger=FINGER_OPEN, label="r: retract"))

    # 8b. Back out along X (the axis the fingers are open on) before going sideways, so the open fingers
    #     never push the bottle neck along the closing axis (Y)
    left_wps.append(Waypoint(duration=0.8, pos=(HOME_L[0], place_l[1], g.HOVER_Z), finger=FINGER_OPEN, label="l: clear_x"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=0.8, pos=(HOME_R[0], place_r[1], g.HOVER_Z), finger=FINGER_OPEN, label="r: clear_x"))

    # 9. Return to standby home
    left_wps.append(Waypoint(duration=1.2, pos=HOME_L, finger=FINGER_OPEN, label="l: return_home"))
    if mode != "one_bottle_only":
        right_wps.append(Waypoint(duration=1.2, pos=HOME_R, finger=FINGER_OPEN, label="r: return_home"))

    _pad_sync(left_wps, right_wps, "sync_settle")

    # 10. Hold at standby home for >= 2.5 s (scoring 2.0 s hold)
    left_wps.append(Waypoint(duration=2.6, pos=HOME_L, finger=FINGER_OPEN, label="l: final_hold"))
    right_wps.append(Waypoint(duration=2.6, pos=HOME_R, finger=FINGER_OPEN, label="r: final_hold"))

    _pad_sync(left_wps, right_wps, "done")
    return {"left": left_wps, "right": right_wps}


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` (from the env flags when None), every duration stretched by DEMO_TIME_SCALE (default 1.8)
    so the soft arms keep up with the commanded waypoints. ``_plans(mode)`` is the unscaled plan used by the checks."""
    from dataclasses import replace

    if mode is None:
        mode = mode_from_env()
    result = _plans(mode)
    if mode == "settle":
        return result
    scale = _f("DEMO_TIME_SCALE", 1.8)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
