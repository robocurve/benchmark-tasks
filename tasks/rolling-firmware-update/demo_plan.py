"""Scripted bimanual demo and ablations for rolling firmware update. Pure Python + NumPy.

Sequence:
1. Left arm holds Unit B's orange service door open (closed-finger push on handle, >= 12 mm deflection)
   while Right arm presses Unit B's yellow UPDATE button.
2. Hold door open for 4.0+ s until Unit B completes update to v1.
3. Left arm releases Unit B door.
4. Right arm presses the white SWITCH button on the shared panel to transfer active load from A to B.
5. Right arm holds Unit A's orange service door open (closed-finger push on handle)
   while Left arm presses Unit A's yellow UPDATE button.
6. Hold door open for 4.0+ s until Unit A completes update to v1.
7. Right arm releases Unit A door.
8. Both arms return home, open grippers, and hold still for 1.0+ s.

Modes (selected by environment variables, see run.sh):
- demo: complete rolling update procedure -> predicted score 1.00
- no_switch: attempts to update A while A is still active -> OUTAGE latches, predicted score 0.00
- release_early: releases door after 2.0 s (< 4.0 s) -> update cancelled, predicted score 0.25
- door_only: holds a door open >= 18 mm for 5 s with one arm, never presses UPDATE -> predicted score 0.25
- settle: holds at home with open grippers -> predicted score 0.00
"""

from __future__ import annotations

import os
import sys
from dataclasses import replace
import numpy as np

if os.environ.get("YAM_ARENA_SRC"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA_SRC"]))

import geometry as g
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
Z_HOVER = 0.880  # hover height above fixtures
HOME = {
    "l": (0.200, 0.240, Z_HOVER),
    "r": (0.200, -0.240, Z_HOVER),
}

# Closed fist command for pushing door handle and pressing buttons
FINGER_BTN = -0.015  # 30 mm between pads (compact fist)
# Fully closed fingers: the 20 mm door handle fits between the pads of the 30 mm 'fist', so the hand slid past it (GPU run 1, door never moved).
FINGER_PUSH = FINGER_CLOSED

# Height of finger pad during door handle push:
# Handle rest top is at g.DOOR_HANDLE_REST_CZ + g.DOOR_HANDLE_LZ / 2
# Contact around center of handle:
Z_PUSH_DOOR = g.DOOR_HANDLE_REST_CZ + PAD_TO_TCP

# Height of TCP for pressing buttons (14 mm below rest cap top):
Z_PRESS_UPDATE = g.UPDATE_BTN_CAP_TOP_REST_Z - 0.014 + PAD_TO_TCP
# GPU run 1: the switch cap only reached -5.2 mm (needs -6.4 mm); command 8 mm deeper (end stop sets the travel).
Z_PRESS_SWITCH = g.SWITCH_BTN_CAP_TOP_REST_Z - 0.014 + PAD_TO_TCP - float(os.environ.get("DEMO_SWITCH_EXTRA_DZ", 0.008))


def _f(name: str, default: float) -> float:
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def _b(name: str, default: bool = False) -> bool:
    return os.environ.get(name, "1" if default else "0") in ("1", "true", "True")


def mode_from_env() -> str:
    for flag, mode in (
        ("DEMO_SETTLE_ONLY", "settle"),
        ("DEMO_NO_SWITCH", "no_switch"),
        ("DEMO_RELEASE_EARLY", "release_early"),
        ("DEMO_DOOR_ONLY", "door_only"),
    ):
        if _b(flag):
            return mode
    return "demo"


def plans(mode: str | None = None, with_meta: bool = False):
    """Generate Waypoint plans for both arms."""
    mode = mode_from_env() if mode is None else mode

    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)

    seqs = {"left": [], "right": []}
    meta = {}

    def add(arm: str, duration: float, pos, finger: float, label: str, op: int, kind: str, **kwargs):
        yaw = yaw_l if arm == "left" else yaw_r
        wp = Waypoint(duration, tuple(float(v) for v in pos), finger, label, yaw)
        seqs[arm].append(wp)
        meta[label] = dict(arm=arm, op=op, kind=kind, **kwargs)

    home_l = HOME["l"]
    home_r = HOME["r"]

    if mode == "settle":
        hold_s = _f("DEMO_SETTLE_HOLD_S", 8.0)
        add("left", hold_s, home_l, FINGER_OPEN, "l: settle hold", 0, "hold")
        add("right", hold_s, home_r, FINGER_OPEN, "r: settle hold", 0, "hold")
        out = {"left": seqs["left"], "right": seqs["right"]}
        return (out, meta) if with_meta else out

    # Handle coordinates:
    # Rest x is 0.255. Push is in -x by 25 mm (to 0.230) with DEMO_DOOR_OVERSHOOT (default 8 mm, to 0.222).
    DEMO_DOOR_OVERSHOOT = _f("DEMO_DOOR_OVERSHOOT", 0.012)
    x_handle_rest = g.DOOR_HANDLE_REST_CX
    # GPU run 2: the old approach point (handle centre + 15 mm) put the 25 mm-deep closed finger ON TOP of the 24 mm handle.
    # The TCP is the finger centre; the finger is ~12.6 mm half-depth, so descend beside the handle's +x face with clearance,
    # and push so the finger's front face travels DOOR_TRAVEL + overshoot (same contact geometry as the knob pushes in G).
    PAD_HALF_DEPTH = 0.0126
    handle_face = x_handle_rest + g.DOOR_HANDLE_LX / 2
    x_push_approach = handle_face + PAD_HALF_DEPTH + 0.008
    x_push_hold = handle_face - g.DOOR_TRAVEL + PAD_HALF_DEPTH - DEMO_DOOR_OVERSHOOT  # presses the 25 mm stop firmly
    # Lift-off point: the finger's front face just clear of the stopped handle (the overshoot point above is inside it),
    # so the hand backs off in +x while it rises.
    x_release = handle_face - g.DOOR_TRAVEL + PAD_HALF_DEPTH + 0.003

    y_handle_b = g.DOOR_HANDLE_REST_CY["B"]
    y_handle_a = g.DOOR_HANDLE_REST_CY["A"]

    # UPDATE button coordinates:
    x_btn_u = g.UPDATE_BTN_CX
    y_btn_u_b = g.UPDATE_BTN_CY["B"]
    y_btn_u_a = g.UPDATE_BTN_CY["A"]

    # SWITCH button coordinates:
    x_btn_sw = g.SWITCH_BTN_CX
    y_btn_sw = g.SWITCH_BTN_CY

    # Helpers to sync arm durations
    def sync_arms(op_idx: int):
        t_l = sum(w.duration for w in seqs["left"])
        t_r = sum(w.duration for w in seqs["right"])
        if t_l > t_r + 1e-6:
            add("right", t_l - t_r, home_r, FINGER_OPEN, f"r: sync op {op_idx}", op_idx, "wait")
        elif t_r > t_l + 1e-6:
            add("left", t_r - t_l, home_l, FINGER_OPEN, f"l: sync op {op_idx}", op_idx, "wait")

    op = 0

    # =========================================================================
    # ABLATION: no_switch (attempt to update A while A is active -> OUTAGE)
    # =========================================================================
    if mode == "no_switch":
        # Right arm holds A's door
        add("right", 0.8, (x_push_approach, y_handle_a, Z_HOVER), FINGER_PUSH, "r: hover door A", op, "door_hover")
        add("right", 0.6, (x_push_approach, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: descend door A", op, "door_descend")
        add("right", 0.8, (x_push_hold, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: push door A", op, "door_push")
        add("right", 3.0, (x_push_hold, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: hold door A", op, "door_hold")

        # Left arm presses UPDATE A (A is active -> triggers OUTAGE!)
        add("left", 1.8, home_l, FINGER_OPEN, "l: wait door", op, "wait")
        add("left", 0.8, (x_btn_u, y_btn_u_a, Z_HOVER), FINGER_BTN, "l: hover update A", op, "btn_hover")
        add("left", 0.6, (x_btn_u, y_btn_u_a, Z_PRESS_UPDATE), FINGER_BTN, "l: press update A", op, "btn_press")
        add("left", 0.6, (x_btn_u, y_btn_u_a, Z_PRESS_UPDATE), FINGER_BTN, "l: hold update A", op, "btn_press")
        add("left", 0.6, (x_btn_u, y_btn_u_a, Z_HOVER), FINGER_BTN, "l: release update A", op, "btn_hover")
        add("left", 0.8, home_l, FINGER_OPEN, "l: return home", op, "transit")
        sync_arms(op)

        # Park
        add("right", 0.6, (x_release, y_handle_a, Z_HOVER), FINGER_PUSH, "r: lift door A", op + 1, "door_hover")
        add("right", 0.8, home_r, FINGER_OPEN, "r: return home", op + 1, "transit")
        sync_arms(op + 1)
        add("left", 2.0, home_l, FINGER_OPEN, "l: final hold", op + 2, "hold")
        add("right", 2.0, home_r, FINGER_OPEN, "r: final hold", op + 2, "hold")

        out = {"left": seqs["left"], "right": seqs["right"]}
        _scale_plans(out)
        return (out, meta) if with_meta else out

    # =========================================================================
    # ABLATION: door_only (hold door open >= 18 mm for 5 s, no update press -> 0.25)
    # =========================================================================
    if mode == "door_only":
        # Left arm opens door B and holds for 5 s
        add("left", 0.8, (x_push_approach, y_handle_b, Z_HOVER), FINGER_PUSH, "l: hover door B", op, "door_hover")
        add("left", 0.6, (x_push_approach, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: descend door B", op, "door_descend")
        add("left", 0.8, (x_push_hold, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: push door B", op, "door_push")
        add("left", 5.0, (x_push_hold, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: hold door B", op, "door_hold")
        add("left", 0.6, (x_release, y_handle_b, Z_HOVER), FINGER_PUSH, "l: lift door B", op, "door_hover")
        add("left", 0.8, home_l, FINGER_OPEN, "l: return home_l", op, "transit")

        # Right arm stays home, never presses UPDATE
        add("right", 8.6, home_r, FINGER_OPEN, "r: stay home", op, "hold")
        sync_arms(op)

        add("left", 2.0, home_l, FINGER_OPEN, "l: final hold", op + 1, "hold")
        add("right", 2.0, home_r, FINGER_OPEN, "r: final hold", op + 1, "hold")

        out = {"left": seqs["left"], "right": seqs["right"]}
        _scale_plans(out)
        return (out, meta) if with_meta else out

    # =========================================================================
    # STANDARD DEMO / RELEASE_EARLY
    # =========================================================================
    # Duration for holding door open: 4.8 s for full demo (>= 4.0 s required), 2.0 s for release_early
    door_hold_s = 2.0 if mode == "release_early" else 4.8

    # --- STEP 1: Update Standby Unit B ---
    # Left arm: approach, descend, push door B open and hold
    add("left", 0.8, (x_push_approach, y_handle_b, Z_HOVER), FINGER_PUSH, "l: hover door B", op, "door_hover")
    add("left", 0.6, (x_push_approach, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: descend door B", op, "door_descend")
    add("left", 0.8, (x_push_hold, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: push door B", op, "door_push")
    add("left", door_hold_s, (x_push_hold, y_handle_b, Z_PUSH_DOOR), FINGER_PUSH, "l: hold door B", op, "door_hold")

    # Right arm: wait until door is pushed open, then press UPDATE B
    add("right", 1.8, home_r, FINGER_OPEN, "r: wait door push", op, "wait")
    add("right", 0.8, (x_btn_u, y_btn_u_b, Z_HOVER), FINGER_BTN, "r: hover update B", op, "btn_hover")
    add("right", 0.6, (x_btn_u, y_btn_u_b, Z_PRESS_UPDATE), FINGER_BTN, "r: press update B", op, "btn_press")
    add("right", 0.5, (x_btn_u, y_btn_u_b, Z_PRESS_UPDATE), FINGER_BTN, "r: hold update B", op, "btn_press")
    add("right", 0.6, (x_btn_u, y_btn_u_b, Z_HOVER), FINGER_BTN, "r: release update B", op, "btn_hover")
    add("right", 0.8, home_r, FINGER_OPEN, "r: return home_r", op, "transit")
    sync_arms(op)
    op += 1

    # Left arm releases door B and returns home
    add("left", 0.6, (x_release, y_handle_b, Z_HOVER), FINGER_PUSH, "l: lift door B", op, "door_hover")
    add("left", 0.8, home_l, FINGER_OPEN, "l: return home_l", op, "transit")
    sync_arms(op)
    op += 1

    if mode == "release_early":
        # Settle and finish: B never completed, stays v0 -> score 0.25
        add("left", 2.0, home_l, FINGER_OPEN, "l: final hold", op, "hold")
        add("right", 2.0, home_r, FINGER_OPEN, "r: final hold", op, "hold")
        out = {"left": seqs["left"], "right": seqs["right"]}
        _scale_plans(out)
        return (out, meta) if with_meta else out

    # --- STEP 2: Press SWITCH button to transfer load to Unit B ---
    add("right", 0.8, (x_btn_sw, y_btn_sw, Z_HOVER), FINGER_BTN, "r: hover switch", op, "btn_hover")
    add("right", 0.6, (x_btn_sw, y_btn_sw, Z_PRESS_SWITCH), FINGER_BTN, "r: press switch", op, "btn_press")
    add("right", 0.5, (x_btn_sw, y_btn_sw, Z_PRESS_SWITCH), FINGER_BTN, "r: hold switch", op, "btn_press")
    add("right", 0.6, (x_btn_sw, y_btn_sw, Z_HOVER), FINGER_BTN, "r: release switch", op, "btn_hover")
    add("right", 0.8, home_r, FINGER_OPEN, "r: return home_r after switch", op, "transit")
    sync_arms(op)
    op += 1

    # --- STEP 3: Update Standby Unit A ---
    # Right arm: approach, descend, push door A open and hold
    add("right", 0.8, (x_push_approach, y_handle_a, Z_HOVER), FINGER_PUSH, "r: hover door A", op, "door_hover")
    add("right", 0.6, (x_push_approach, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: descend door A", op, "door_descend")
    add("right", 0.8, (x_push_hold, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: push door A", op, "door_push")
    add("right", door_hold_s, (x_push_hold, y_handle_a, Z_PUSH_DOOR), FINGER_PUSH, "r: hold door A", op, "door_hold")

    # Left arm: wait until door A is open, then press UPDATE A
    add("left", 1.8, home_l, FINGER_OPEN, "l: wait door A push", op, "wait")
    add("left", 0.8, (x_btn_u, y_btn_u_a, Z_HOVER), FINGER_BTN, "l: hover update A", op, "btn_hover")
    add("left", 0.6, (x_btn_u, y_btn_u_a, Z_PRESS_UPDATE), FINGER_BTN, "l: press update A", op, "btn_press")
    add("left", 0.5, (x_btn_u, y_btn_u_a, Z_PRESS_UPDATE), FINGER_BTN, "l: hold update A", op, "btn_press")
    add("left", 0.6, (x_btn_u, y_btn_u_a, Z_HOVER), FINGER_BTN, "l: release update A", op, "btn_hover")
    add("left", 0.8, home_l, FINGER_OPEN, "l: return home_l after update A", op, "transit")
    sync_arms(op)
    op += 1

    # Right arm releases door A and returns home
    add("right", 0.6, (x_release, y_handle_a, Z_HOVER), FINGER_PUSH, "r: lift door A", op, "door_hover")
    add("right", 0.8, home_r, FINGER_OPEN, "r: return home_r after door A", op, "transit")
    sync_arms(op)
    op += 1

    # --- STEP 4: Success Hold (Parked at home, grippers open, doors closed) ---
    add("left", 2.5, home_l, FINGER_OPEN, "l: final success hold", op, "hold")
    add("right", 2.5, home_r, FINGER_OPEN, "r: final success hold", op, "hold")

    out = {"left": seqs["left"], "right": seqs["right"]}
    _scale_plans(out)
    return (out, meta) if with_meta else out


def _scale_plans(seqs: dict[str, list[Waypoint]]) -> None:
    """Scale all waypoint durations by DEMO_TIME_SCALE (default 1.8)."""
    scale = _f("DEMO_TIME_SCALE", 1.8)
    if scale != 1.0:
        for arm in ("left", "right"):
            seqs[arm][:] = [replace(w, duration=w.duration * scale) for w in seqs[arm]]
