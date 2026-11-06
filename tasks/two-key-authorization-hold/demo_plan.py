"""Bimanual demonstration plans and ablations for two-key-authorization-hold.

Uses yam_arena.demo.Waypoint.
"""

from __future__ import annotations

import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TIP_PAST_TCP = g.PAD_TIP_PAST_TCP
FINGER_BTN = -0.015            # closed grip for pushing guard / pressing button
FINGER_OPEN_VAL = FINGER_OPEN  # fully open grippers


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_SEQUENTIAL") == "1":
        return "sequential"
    if os.environ.get("DEMO_ONE_KEY") == "1":
        return "one_key"
    if os.environ.get("DEMO_RELEASE_EARLY") == "1":
        return "release_early"
    if os.environ.get("DEMO_NO_CONFIRM") == "1":
        return "no_confirm"
    return "demo"


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)
    press_dz = _f("DEMO_PRESS_DZ", 0.0)
    time_scale = _f("DEMO_TIME_SCALE", 1.8)

    hover_z = g.HOVER_Z
    press_z = g.BUTTON_PRESS_TCP_Z + press_dz
    guard_z = g.GUARD_REST_Z + 0.005  # push height centered on guard
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
    left.append(Waypoint(0.5, home_l, FINGER_BTN, "l: ready", yaw_l))
    right.append(Waypoint(0.5, home_r, FINGER_BTN, "r: ready", yaw_r))

    def sync_arms():
        dur_l = sum(w.duration for w in left)
        dur_r = sum(w.duration for w in right)
        if dur_l < dur_r:
            left.append(Waypoint(dur_r - dur_l, left[-1].pos, left[-1].finger, "l: sync wait", yaw_l))
        elif dur_r < dur_l:
            right.append(Waypoint(dur_l - dur_r, right[-1].pos, right[-1].finger, "r: sync wait", yaw_r))

    if mode == "demo":
        # 1. Open both guards together (push outward)
        # Left arm pushes +Y: approach inside at y = +0.210, push to +0.365
        left.append(Waypoint(0.6, (0.300, +0.210, hover_z), FINGER_BTN, "l: hover guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.210, guard_z), FINGER_BTN, "l: descend guard_left", yaw_l))
        left.append(Waypoint(1.3, (0.300, +0.365, guard_z), FINGER_BTN, "l: push guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.365, hover_z), FINGER_BTN, "l: lift guard_left", yaw_l))

        # Right arm pushes -Y: approach inside at y = -0.210, push to -0.365
        right.append(Waypoint(0.6, (0.300, -0.210, hover_z), FINGER_BTN, "r: hover guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.210, guard_z), FINGER_BTN, "r: descend guard_right", yaw_r))
        right.append(Waypoint(1.3, (0.300, -0.365, guard_z), FINGER_BTN, "r: push guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.365, hover_z), FINGER_BTN, "r: lift guard_right", yaw_r))
        sync_arms()

        # 2. Both move to hover over keys
        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: hover key_left", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right", yaw_r))

        # 3. Both descend and press keys together
        left.append(Waypoint(0.7, (0.300, +0.280, press_z), FINGER_BTN, "l: press key_left", yaw_l))
        right.append(Waypoint(0.7, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right", yaw_r))

        # 4. Hold both keys together for 4.5 s (required >= 3.0 s; margin +1.5 s)
        left.append(Waypoint(4.5, (0.300, +0.280, press_z), FINGER_BTN, "l: hold key_left", yaw_l))
        right.append(Waypoint(4.5, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right", yaw_r))

        # 5. Both release keys and lift to hover
        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: lift key_left", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right", yaw_r))
        sync_arms()

        # 6. Left returns home and opens gripper; Right presses CONFIRM button
        left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home", yaw_l))

        right.append(Waypoint(0.7, (0.300, 0.000, hover_z), FINGER_BTN, "r: hover confirm", yaw_r))
        right.append(Waypoint(0.6, (0.300, 0.000, press_z), FINGER_BTN, "r: press confirm", yaw_r))
        right.append(Waypoint(0.5, (0.300, 0.000, press_z), FINGER_BTN, "r: hold confirm", yaw_r))
        right.append(Waypoint(0.5, (0.300, 0.000, hover_z), FINGER_BTN, "r: lift confirm", yaw_r))
        right.append(Waypoint(0.7, home_r, FINGER_OPEN_VAL, "r: return home", yaw_r))
        sync_arms()

        # 7. Final settle: hold at home still with open fingers for 2.0 s (40 steps)
        left.append(Waypoint(2.0, home_l, FINGER_OPEN_VAL, "l: final settle", yaw_l))
        right.append(Waypoint(2.0, home_r, FINGER_OPEN_VAL, "r: final settle", yaw_r))

    elif mode == "sequential":
        # Left opens guard and presses left key for 2.0 s, releases
        left.append(Waypoint(0.6, (0.300, +0.210, hover_z), FINGER_BTN, "l: hover guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.210, guard_z), FINGER_BTN, "l: descend guard_left", yaw_l))
        left.append(Waypoint(1.3, (0.300, +0.365, guard_z), FINGER_BTN, "l: push guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.365, hover_z), FINGER_BTN, "l: lift guard_left", yaw_l))
        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: hover key_left", yaw_l))
        left.append(Waypoint(0.7, (0.300, +0.280, press_z), FINGER_BTN, "l: press key_left", yaw_l))
        left.append(Waypoint(2.0, (0.300, +0.280, press_z), FINGER_BTN, "l: hold key_left", yaw_l))
        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: lift key_left", yaw_l))
        left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home", yaw_l))

        # Right opens guard and presses right key for 2.0 s, releases (sequential, not simultaneous)
        right.append(Waypoint(4.5, home_r, FINGER_BTN, "r: wait", yaw_r))
        right.append(Waypoint(0.6, (0.300, -0.210, hover_z), FINGER_BTN, "r: hover guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.210, guard_z), FINGER_BTN, "r: descend guard_right", yaw_r))
        right.append(Waypoint(1.3, (0.300, -0.365, guard_z), FINGER_BTN, "r: push guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.365, hover_z), FINGER_BTN, "r: lift guard_right", yaw_r))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right", yaw_r))
        right.append(Waypoint(0.7, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right", yaw_r))
        right.append(Waypoint(2.0, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right", yaw_r))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right", yaw_r))
        right.append(Waypoint(0.8, home_r, FINGER_OPEN_VAL, "r: return home", yaw_r))
        sync_arms()

    elif mode == "one_key":
        # Both open guards, but only right arm presses right key
        left.append(Waypoint(0.6, (0.300, +0.210, hover_z), FINGER_BTN, "l: hover guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.210, guard_z), FINGER_BTN, "l: descend guard_left", yaw_l))
        left.append(Waypoint(1.3, (0.300, +0.365, guard_z), FINGER_BTN, "l: push guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.365, hover_z), FINGER_BTN, "l: lift guard_left", yaw_l))
        left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home", yaw_l))

        right.append(Waypoint(0.6, (0.300, -0.210, hover_z), FINGER_BTN, "r: hover guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.210, guard_z), FINGER_BTN, "r: descend guard_right", yaw_r))
        right.append(Waypoint(1.3, (0.300, -0.365, guard_z), FINGER_BTN, "r: push guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.365, hover_z), FINGER_BTN, "r: lift guard_right", yaw_r))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right", yaw_r))
        right.append(Waypoint(0.7, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right", yaw_r))
        right.append(Waypoint(4.5, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right", yaw_r))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right", yaw_r))
        right.append(Waypoint(0.8, home_r, FINGER_OPEN_VAL, "r: return home", yaw_r))
        sync_arms()

    elif mode == "release_early":
        # Both open guards, then press keys for ~2.0 s (1.1 s * 1.8 stretch = 1.98 s), release (strike 1),
        # then press again for ~2.0 s, release (strike 2 -> locked out 0.25)
        left.append(Waypoint(0.6, (0.300, +0.210, hover_z), FINGER_BTN, "l: hover guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.210, guard_z), FINGER_BTN, "l: descend guard_left", yaw_l))
        left.append(Waypoint(1.3, (0.300, +0.365, guard_z), FINGER_BTN, "l: push guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.365, hover_z), FINGER_BTN, "l: lift guard_left", yaw_l))

        right.append(Waypoint(0.6, (0.300, -0.210, hover_z), FINGER_BTN, "r: hover guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.210, guard_z), FINGER_BTN, "r: descend guard_right", yaw_r))
        right.append(Waypoint(1.3, (0.300, -0.365, guard_z), FINGER_BTN, "r: push guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.365, hover_z), FINGER_BTN, "r: lift guard_right", yaw_r))
        sync_arms()

        # Press 1 (< 3.0 s hold -> strike 1)
        left.append(Waypoint(0.5, (0.300, +0.280, hover_z), FINGER_BTN, "l: hover key_left", yaw_l))
        right.append(Waypoint(0.5, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right", yaw_r))
        left.append(Waypoint(0.4, (0.300, +0.280, press_z), FINGER_BTN, "l: press key_left", yaw_l))
        right.append(Waypoint(0.4, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right", yaw_r))
        left.append(Waypoint(0.6, (0.300, +0.280, press_z), FINGER_BTN, "l: hold key_left", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right", yaw_r))
        left.append(Waypoint(0.5, (0.300, +0.280, hover_z), FINGER_BTN, "l: lift key_left", yaw_l))
        right.append(Waypoint(0.5, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right", yaw_r))

        # Press 2 (< 3.0 s hold -> strike 2 -> locked)
        left.append(Waypoint(0.5, (0.300, +0.280, hover_z), FINGER_BTN, "l: hover key_left 2", yaw_l))
        right.append(Waypoint(0.5, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right 2", yaw_r))
        left.append(Waypoint(0.4, (0.300, +0.280, press_z), FINGER_BTN, "l: press key_left 2", yaw_l))
        right.append(Waypoint(0.4, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right 2", yaw_r))
        left.append(Waypoint(0.6, (0.300, +0.280, press_z), FINGER_BTN, "l: hold key_left 2", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right 2", yaw_r))
        left.append(Waypoint(0.5, (0.300, +0.280, hover_z), FINGER_BTN, "l: lift key_left 2", yaw_l))
        right.append(Waypoint(0.5, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right 2", yaw_r))
        left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home", yaw_l))
        right.append(Waypoint(0.8, home_r, FINGER_OPEN_VAL, "r: return home", yaw_r))
        sync_arms()

    elif mode == "no_confirm":
        # Both open guards, press and hold keys for 4.5 s (ARMED), but CONFIRM is not pressed
        left.append(Waypoint(0.6, (0.300, +0.210, hover_z), FINGER_BTN, "l: hover guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.210, guard_z), FINGER_BTN, "l: descend guard_left", yaw_l))
        left.append(Waypoint(1.3, (0.300, +0.365, guard_z), FINGER_BTN, "l: push guard_left", yaw_l))
        left.append(Waypoint(0.5, (0.300, +0.365, hover_z), FINGER_BTN, "l: lift guard_left", yaw_l))

        right.append(Waypoint(0.6, (0.300, -0.210, hover_z), FINGER_BTN, "r: hover guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.210, guard_z), FINGER_BTN, "r: descend guard_right", yaw_r))
        right.append(Waypoint(1.3, (0.300, -0.365, guard_z), FINGER_BTN, "r: push guard_right", yaw_r))
        right.append(Waypoint(0.5, (0.300, -0.365, hover_z), FINGER_BTN, "r: lift guard_right", yaw_r))
        sync_arms()

        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: hover key_left", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: hover key_right", yaw_r))
        left.append(Waypoint(0.7, (0.300, +0.280, press_z), FINGER_BTN, "l: press key_left", yaw_l))
        right.append(Waypoint(0.7, (0.300, -0.280, press_z), FINGER_BTN, "r: press key_right", yaw_r))
        left.append(Waypoint(4.5, (0.300, +0.280, press_z), FINGER_BTN, "l: hold key_left", yaw_l))
        right.append(Waypoint(4.5, (0.300, -0.280, press_z), FINGER_BTN, "r: hold key_right", yaw_r))
        left.append(Waypoint(0.6, (0.300, +0.280, hover_z), FINGER_BTN, "l: lift key_left", yaw_l))
        right.append(Waypoint(0.6, (0.300, -0.280, hover_z), FINGER_BTN, "r: lift key_right", yaw_r))
        left.append(Waypoint(0.8, home_l, FINGER_OPEN_VAL, "l: return home", yaw_l))
        right.append(Waypoint(0.8, home_r, FINGER_OPEN_VAL, "r: return home", yaw_r))
        sync_arms()

    # Apply time scaling to stretched waypoints
    def stretch(wps: list[Waypoint]) -> list[Waypoint]:
        return [replace(w, duration=w.duration * time_scale) for w in wps]

    return {
        "left": stretch(left),
        "right": stretch(right),
    }
