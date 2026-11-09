"""Bimanual demonstration plans and ablations for compress-and-lock-spring-cartridge.

Uses yam_arena.demo.Waypoint.
Left arm (+y) compresses the orange plunger along +X.
Right arm (-y) slides the lock bar across along +Y into the notch.
"""

from __future__ import annotations

import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_OPEN_VAL = FINGER_OPEN  # fully open (-0.04695)
FINGER_CLOSED = -0.015         # firm closed wedge for pushing knob / handle


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_ONE_ARM") == "1":
        return "one_arm"
    if os.environ.get("DEMO_RELEASE_EARLY") == "1":
        return "release_early"
    if os.environ.get("DEMO_BAR_FIRST") == "1":
        return "bar_first"
    if os.environ.get("DEMO_NO_RELEASE") == "1":
        return "no_release"
    if os.environ.get("DEMO_ORDER_BAR_FIRST_ARM") == "1":
        return "order_bar_first_arm"
    return os.environ.get("DEMO_MODE", "demo")


def _plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    if mode is None:
        mode = mode_from_env()

    hover_z = g.HOVER_Z
    plunger_z = g.PLUNGER_REST_Z
    bar_z = g.LOCK_BAR_REST_Z

    overshoot_x = _f("DEMO_PLUNGER_OVERSHOOT", g.PLUNGER_PUSH_OVERSHOOT)
    overshoot_y = _f("DEMO_BAR_OVERSHOOT", 0.010)
    plunger_hold_s = _f("DEMO_PLUNGER_HOLD_S", 1.5)
    bar_hold_s = _f("DEMO_BAR_HOLD_S", 1.5)
    release_time_s = _f("DEMO_RELEASE_TIME_S", 1.5)

    # Home positions
    home_l = (0.200, +0.305, hover_z)
    home_r = (0.200, -0.305, hover_z)

    # Plunger positions (Left arm)
    plunger_hover = (g.PLUNGER_REST_X - 0.050, g.PLUNGER_REST_Y, hover_z)
    plunger_align = (g.PLUNGER_REST_X - 0.025 - PAD_TO_TCP, g.PLUNGER_REST_Y, plunger_z)
    plunger_contact = (g.PLUNGER_REST_X - PAD_TO_TCP, g.PLUNGER_REST_Y, plunger_z)
    # Full compression push command along +X
    plunger_compressed_x = g.PLUNGER_REST_X + g.PLUNGER_TRAVEL + overshoot_x - PAD_TO_TCP
    plunger_pushed = (plunger_compressed_x, g.PLUNGER_REST_Y, plunger_z)

    # Lock bar positions (Right arm)
    bar_hover = (g.LOCK_BAR_REST_X, g.LOCK_BAR_REST_Y - 0.050, hover_z)
    bar_align = (g.LOCK_BAR_REST_X, g.LOCK_BAR_REST_Y - 0.025 - PAD_TO_TCP, bar_z)
    bar_contact = (g.LOCK_BAR_REST_X, g.LOCK_BAR_REST_Y - PAD_TO_TCP, bar_z)
    # Locked position along +Y (travel 60 mm >= 55 mm threshold, plus overshoot)
    bar_locked_y = g.LOCK_BAR_REST_Y + 0.060 + overshoot_y - PAD_TO_TCP
    bar_pushed = (g.LOCK_BAR_REST_X, bar_locked_y, bar_z)
    # Blocked position (stops at 15 mm)
    bar_blocked_y = g.LOCK_BAR_REST_Y + g.BAR_STOP_EARLY_LIMIT - PAD_TO_TCP
    bar_blocked_pos = (g.LOCK_BAR_REST_X, bar_blocked_y, bar_z)

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def sync_arms(lbl: str = "sync"):
        t_l = sum(w.duration for w in left)
        t_r = sum(w.duration for w in right)
        if t_l < t_r:
            prefix = "l: hold plunger sync" if ("plunger" in left[-1].label or "compress" in left[-1].label or "hold" in left[-1].label) else f"l: {lbl}"
            left.append(Waypoint(t_r - t_l, left[-1].pos, left[-1].finger, prefix, left[-1].yaw))
        elif t_r < t_l:
            prefix = "r: hold bar sync" if ("bar" in right[-1].label or "slide" in right[-1].label or "hold" in right[-1].label) else f"r: {lbl}"
            right.append(Waypoint(t_l - t_r, right[-1].pos, right[-1].finger, prefix, right[-1].yaw))

    if mode in ("settle", "dream"):
        left.append(Waypoint(14.0, home_l, FINGER_OPEN_VAL, "l: settle idle", 0.0))
        right.append(Waypoint(14.0, home_r, FINGER_OPEN_VAL, "r: settle idle", 0.0))
        return {"left": left, "right": right}

    # === STEP 1: INITIAL READY ===
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: ready", 0.0))
    right.append(Waypoint(0.5, home_r, FINGER_OPEN_VAL, "r: ready", 0.0))
    sync_arms()

    if mode == "bar_first":
        # Right arm pushes bar before plunger is compressed -> bar hits physical stop at 15 mm
        right.append(Waypoint(0.6, bar_hover, FINGER_OPEN_VAL, "r: hover bar", 0.0))
        right.append(Waypoint(0.5, bar_align, FINGER_CLOSED, "r: align bar", 0.0))
        right.append(Waypoint(0.4, bar_contact, FINGER_CLOSED, "r: contact bar", 0.0))
        right.append(Waypoint(0.8, bar_blocked_pos, FINGER_CLOSED, "r: push bar (blocked)", 0.0))
        right.append(Waypoint(2.0, bar_blocked_pos, FINGER_CLOSED, "r: stalled at stop box", 0.0))
        sync_arms()
        left.append(Waypoint(3.0, home_l, FINGER_OPEN_VAL, "l: idle home", 0.0))
        right.append(Waypoint(1.0, bar_hover, FINGER_OPEN_VAL, "r: retract hover", 0.0))
        right.append(Waypoint(1.0, home_r, FINGER_OPEN_VAL, "r: return home", 0.0))
        sync_arms()
        return {"left": left, "right": right}

    if mode == "order_bar_first_arm":
        # Valid order 2: Right arm moves to standby near bar handle first, then left arm compresses plunger
        right.append(Waypoint(0.6, bar_hover, FINGER_OPEN_VAL, "r: hover bar standby", 0.0))
        right.append(Waypoint(0.5, bar_align, FINGER_CLOSED, "r: align bar standby", 0.0))

    # Left arm approaches and compresses plunger
    left.append(Waypoint(0.6, plunger_hover, FINGER_OPEN_VAL, "l: hover plunger", 0.0))
    left.append(Waypoint(0.5, plunger_align, FINGER_CLOSED, "l: align plunger", 0.0))
    left.append(Waypoint(0.4, plunger_contact, FINGER_CLOSED, "l: contact plunger", 0.0))
    left.append(Waypoint(1.0, plunger_pushed, FINGER_CLOSED, "l: compress plunger", 0.0))
    # Hold compressed >= 1.0 s
    left.append(Waypoint(plunger_hold_s, plunger_pushed, FINGER_CLOSED, "l: hold compressed 1s", 0.0))
    sync_arms()

    if mode == "release_early":
        # Let go before bar is across: spring returns plunger to 0
        left.append(Waypoint(0.5, plunger_align, FINGER_OPEN_VAL, "l: release early", 0.0))
        left.append(Waypoint(0.8, plunger_hover, FINGER_OPEN_VAL, "l: retract hover", 0.0))
        left.append(Waypoint(1.0, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        sync_arms()
        left.append(Waypoint(3.0, home_l, FINGER_OPEN_VAL, "l: wait home", 0.0))
        right.append(Waypoint(3.0, home_r, FINGER_OPEN_VAL, "r: idle", 0.0))
        sync_arms()
        return {"left": left, "right": right}

    if mode == "one_arm":
        # Left arm holds plunger; right arm cannot reach, stays at home
        left.append(Waypoint(4.0, plunger_pushed, FINGER_CLOSED, "l: continue hold plunger", 0.0))
        sync_arms()
        left.append(Waypoint(1.0, home_l, FINGER_OPEN_VAL, "l: return home", 0.0))
        right.append(Waypoint(1.0, home_r, FINGER_OPEN_VAL, "r: idle home", 0.0))
        return {"left": left, "right": right}

    # === STEP 2: RIGHT ARM HOOKS AND SLIDES LOCK BAR ACROSS ===
    if mode != "order_bar_first_arm":
        right.append(Waypoint(0.6, bar_hover, FINGER_OPEN_VAL, "r: hover bar", 0.0))
        right.append(Waypoint(0.5, bar_align, FINGER_CLOSED, "r: align bar", 0.0))

    right.append(Waypoint(0.4, bar_contact, FINGER_CLOSED, "r: contact bar", 0.0))
    # Slide bar across through the aligned notch to 60 mm (>= 55 mm)
    right.append(Waypoint(1.0, bar_pushed, FINGER_CLOSED, "r: slide bar locked", 0.0))
    # Hold bar locked
    right.append(Waypoint(bar_hold_s, bar_pushed, FINGER_CLOSED, "r: hold bar locked", 0.0))
    # Left arm still holding plunger during bar slide
    left.append(Waypoint(1.0 + bar_hold_s + (0.4 if mode != "order_bar_first_arm" else 0.0), plunger_pushed, FINGER_CLOSED, "l: maintain hold during bar slide", 0.0))
    sync_arms()

    if mode == "no_release":
        # Never release plunger: keep holding until end
        left.append(Waypoint(5.0, plunger_pushed, FINGER_CLOSED, "l: continuous hold no release", 0.0))
        right.append(Waypoint(5.0, bar_pushed, FINGER_CLOSED, "r: stay at bar", 0.0))
        return {"left": left, "right": right}

    # === STEP 3: LEFT ARM EASES OFF OVER 1.5 S AND RELEASES ===
    # Right arm holds bar position
    right.append(Waypoint(release_time_s + 1.0, bar_pushed, FINGER_CLOSED, "r: keep bar steady", 0.0))
    # Left arm slowly retracts back over release_time_s, releasing knob
    left.append(Waypoint(release_time_s, plunger_align, FINGER_OPEN_VAL, f"l: ease off knob {release_time_s:.1f}s", 0.0))
    left.append(Waypoint(0.5, plunger_hover, FINGER_OPEN_VAL, "l: clear plunger knob", 0.0))
    left.append(Waypoint(0.5, home_l, FINGER_OPEN_VAL, "l: retract to home", 0.0))
    sync_arms()

    # === STEP 4: RIGHT ARM RETRACTS, BOTH ARMS CLEAR (held >= 2.0 s) ===
    right.append(Waypoint(0.6, bar_hover, FINGER_OPEN_VAL, "r: lift clear bar", 0.0))
    right.append(Waypoint(0.8, home_r, FINGER_OPEN_VAL, "r: retract to home", 0.0))
    sync_arms()

    # Final hold: both arms home, grippers open, green lamp lit (held >= 2.5 s > 2.0 s required)
    left.append(Waypoint(3.0, home_l, FINGER_OPEN_VAL, "l: final clear hold", 0.0))
    right.append(Waypoint(3.0, home_r, FINGER_OPEN_VAL, "r: final clear hold", 0.0))

    return {"left": left, "right": right}


def plans(mode: str | None = None) -> dict[str, list[Waypoint]]:
    """The plan for ``mode`` with waypoint durations scaled by DEMO_TIME_SCALE (default 1.8)."""
    if mode is None:
        mode = mode_from_env()
    result = _plans(mode)
    if mode in ("settle", "dream"):
        return result
    scale = _f("DEMO_TIME_SCALE", g.DEMO_TIME_SCALE)
    return {arm: [replace(w, duration=w.duration * scale) for w in wps] for arm, wps in result.items()}
