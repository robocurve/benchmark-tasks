"""The scripted demo for the thermal soak task. Pure Python.

Bimanual coordination:
- The left arm manages picks from staging (y > 0) and placement onto the hot plate.
- The right arm operates the buttons on fixture front and moves samples from hot plate to cool pad (y < 0).
- Arms maintain >= 20 mm clearance at all times by parking the idle arm at home during operations.

Waypoint structure: (duration, pos, finger, label, yaw) played open-loop by yam_scripted.
All tuning knobs are exposed as environment variables (DEMO_*):
- DEMO_TIME_SCALE: stretch every motion duration (default 1.8)
- DEMO_SETTLE_ONLY: if 1, returns empty plan so arms hold initial ready pose (scores 0.0)
- DEMO_NAIVE: if 1, lifts sample 2 when bar shows 6 discs (scores 0.75, core short)
- DEMO_OVERSHOOT: if 1, leaves sample 2 on plate until core > 90 C (scores 0.0, FIRE)
- DEMO_NO_SAMPLE1: if 1, skips sample 1 and heats sample 2 directly (scores 0.75)
- DEMO_YAW_SHIFT_LEFT: yaw shift for left arm (rad, default 0.0)
- DEMO_YAW_SHIFT_RIGHT: yaw shift for right arm (rad, default 0.0)
- DEMO_INSERT_DZ: vertical offset on placement (m, default 0.0007)
- DEMO_GRIP_DX: grasp offset along x (m, default 0.0)
- DEMO_GRIP_DY: grasp offset along y (m, default 0.0)
"""

from __future__ import annotations

import math
import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = g.PAD_TO_TCP
FINGER_GRIP = -0.025  # 50 mm jaw opening (5 mm past 60 mm block face)
FINGER_BTN = -0.015   # closed fist for pressing buttons


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def demo_plans() -> dict[str, list[Waypoint]]:
    if os.environ.get("DEMO_HOLD_ONLY") == "1":
        # GPU settle run: an empty plan makes the scripted policy fall back to its built-in demo (it pressed HEAT by accident),
        # so the settle run uses an explicit hold at the home poses instead.
        hz = g.TOP + 0.160
        hold = _f("DEMO_SETTLE_HOLD_S", 8.0)
        return {"left": [Waypoint(hold, (0.200, +0.220, hz), FINGER_OPEN, "l: settle hold", 0.0)],
                "right": [Waypoint(hold, (0.200, -0.220, hz), FINGER_OPEN, "r: settle hold", 0.0)]}
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return {}

    mode = "demo"
    if os.environ.get("DEMO_NAIVE") == "1":
        mode = "naive"
    elif os.environ.get("DEMO_OVERSHOOT") == "1":
        mode = "overshoot"
    elif os.environ.get("DEMO_NO_SAMPLE1") == "1":
        mode = "no_sample1"

    scale = _f("DEMO_TIME_SCALE", 1.8)
    grip_dx = _f("DEMO_GRIP_DX", 0.0)
    grip_dy = _f("DEMO_GRIP_DY", 0.0)
    insert_dz = _f("DEMO_INSERT_DZ", 0.0007)

    s1_dwell = _f("DEMO_S1_DWELL", 14.0)
    s2_dwell_extra = _f("DEMO_S2_DWELL_EXTRA_S", 0.0)
    s2_dwell = s1_dwell + s2_dwell_extra

    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", 0.0)
    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", 0.0)

    hover_z = g.TOP + 0.160
    home_l = (0.200, +0.220, hover_z)
    home_r = (0.200, -0.220, hover_z)

    # Grasp TCP z heights
    stage_z = g.STAGE_MAT_TOP_Z + g.SAMPLE_SIZE[2] / 2.0 + PAD_TO_TCP
    plate_z = g.PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0 + PAD_TO_TCP + insert_dz
    cool_z = g.COOL_PAD_TOP_Z + g.SAMPLE_SIZE[2] / 2.0 + PAD_TO_TCP + insert_dz

    press_z = g.BUTTON_PRESS_TCP_Z

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def dur(wps: list[Waypoint]) -> float:
        return sum(w.duration for w in wps)

    def sync(tag: str) -> None:
        """Keep both arms synchronized in duration by padding with waits at home."""
        tl, tr = dur(left), dur(right)
        if tl > tr + 1e-6:
            right.append(Waypoint(tl - tr, home_r, FINGER_OPEN, f"r: wait {tag}", yaw_r))
        elif tr > tl + 1e-6:
            left.append(Waypoint(tr - tl, home_l, FINGER_OPEN, f"l: wait {tag}", yaw_l))

    # Helper motion primitives (scaled by DEMO_TIME_SCALE; descents >= 1.2s, dwells >= 0.5s)
    def left_pick_and_place_to_plate(pick_pos: tuple[float, float, float], tag: str) -> None:
        px, py = pick_pos[0] + grip_dx, pick_pos[1] + grip_dy
        dest_x, dest_y = g.PAD_POS[0] + grip_dx, g.PAD_POS[1] + grip_dy

        # Approach and descend onto staging block with fully open hand
        left.extend([
            Waypoint(0.6 * scale, (px, py, hover_z), FINGER_OPEN, f"l: hover {tag} stage", yaw_l),
            Waypoint(0.7 * scale, (px, py, stage_z), FINGER_OPEN, f"l: descend {tag} stage", yaw_l),
            Waypoint(0.3 * scale, (px, py, stage_z), FINGER_GRIP, f"l: grip {tag}", yaw_l),
            Waypoint(0.6 * scale, (px, py, hover_z), FINGER_GRIP, f"l: lift {tag}", yaw_l),
            Waypoint(0.6 * scale, (dest_x, dest_y, hover_z), FINGER_GRIP, f"l: carry {tag} to plate", yaw_l),
            Waypoint(0.7 * scale, (dest_x, dest_y, plate_z), FINGER_GRIP, f"l: lower {tag} to plate", yaw_l),
            Waypoint(0.3 * scale, (dest_x, dest_y, plate_z), FINGER_OPEN, f"l: release {tag} on plate", yaw_l),
            Waypoint(0.5 * scale, (dest_x, dest_y, hover_z), FINGER_OPEN, f"l: lift from plate after {tag}", yaw_l),
            Waypoint(0.6 * scale, home_l, FINGER_OPEN, f"l: park home after {tag}", yaw_l),
        ])

    def right_press_button(btn_pos: tuple[float, float, float], name: str) -> None:
        bx, by = btn_pos[0], btn_pos[1]
        right.extend([
            Waypoint(0.5 * scale, (bx, by, hover_z), FINGER_BTN, f"r: approach {name}", yaw_r),
            Waypoint(0.7 * scale, (bx, by, press_z), FINGER_BTN, f"r: press {name}", yaw_r),
            Waypoint(0.3 * scale, (bx, by, press_z), FINGER_BTN, f"r: hold {name}", yaw_r),
            Waypoint(0.5 * scale, (bx, by, hover_z), FINGER_BTN, f"r: release {name}", yaw_r),
            Waypoint(0.5 * scale, home_r, FINGER_OPEN, f"r: park home after {name}", yaw_r),
        ])

    def right_pick_from_plate_and_place_to_cool(dest_pos: tuple[float, float], tag: str) -> None:
        px, py = g.PAD_POS[0] + grip_dx, g.PAD_POS[1] + grip_dy
        dx, dy = dest_pos[0] + grip_dx, dest_pos[1] + grip_dy

        right.extend([
            Waypoint(0.6 * scale, (px, py, hover_z), FINGER_OPEN, f"r: hover plate for {tag}", yaw_r),
            Waypoint(0.7 * scale, (px, py, plate_z), FINGER_OPEN, f"r: descend to plate for {tag}", yaw_r),
            Waypoint(0.3 * scale, (px, py, plate_z), FINGER_GRIP, f"r: grip {tag} on plate", yaw_r),
            Waypoint(0.6 * scale, (px, py, hover_z), FINGER_GRIP, f"r: lift {tag} from plate", yaw_r),
            Waypoint(0.6 * scale, (dx, dy, hover_z), FINGER_GRIP, f"r: carry {tag} to cool pad", yaw_r),
            Waypoint(0.7 * scale, (dx, dy, cool_z), FINGER_GRIP, f"r: lower {tag} to cool pad", yaw_r),
            Waypoint(0.3 * scale, (dx, dy, cool_z), FINGER_OPEN, f"r: release {tag} on cool pad", yaw_r),
            Waypoint(0.5 * scale, (dx, dy, hover_z), FINGER_OPEN, f"r: lift from cool pad after {tag}", yaw_r),
            Waypoint(0.6 * scale, home_r, FINGER_OPEN, f"r: park home after {tag}", yaw_r),
        ])

    def wait_both(duration_s: float, tag: str) -> None:
        """Physical wait in place while ODE advances in simulation."""
        left.append(Waypoint(duration_s, home_l, FINGER_OPEN, f"l: wait {tag}", yaw_l))
        right.append(Waypoint(duration_s, home_r, FINGER_OPEN, f"r: wait {tag}", yaw_r))

    # Transit overhead between placement release and pickup grasp:
    # (0.5 + 0.6 + 0.6 + 0.7) * scale = 2.4 * scale
    transit_overhead = 2.4 * scale
    s1_wait_s = max(0.1, s1_dwell - transit_overhead)

    # --- Episode Execution Sequence ---

    # Phase 0: Right arm presses HEAT button to begin thermostat heating to 110 C
    right_press_button(g.HEAT_BTN_POS, "HEAT")
    sync("heat on")

    if mode != "no_sample1":
        # Phase 1: Left arm picks Sample 1 from staging and places on hot plate
        left_pick_and_place_to_plate(g.START_POS_1, "s1")
        sync("s1 placed")

        # Phase 2: Dwell on hot plate for s1_dwell
        wait_both(s1_wait_s, f"dwell s1 ({s1_dwell:.1f}s)")

        # Phase 3: Right arm moves Sample 1 to Cool Pad spot 1
        right_pick_from_plate_and_place_to_cool(g.COOL_SPOT_1, "s1")
        sync("s1 on cool pad")

    # Phase 4: Left arm picks Sample 2 from staging and places on hot plate
    left_pick_and_place_to_plate(g.START_POS_2, "s2")
    sync("s2 placed")

    if mode == "naive":
        # Naive ablation: lift early after 9 s, core reaches ~53 C (< 60 C band) -> score 0.75
        s2_wait_s = max(0.1, 9.0 - transit_overhead)
        wait_both(s2_wait_s, "dwell s2 naive early lift")
    elif mode == "overshoot":
        # Overshoot ablation: leave on plate 55 s until core > 90 C (trips FIRE zero-gate)
        wait_both(55.0, "dwell s2 overshoot FIRE")
    else:
        # Demo and no_sample1: dwell derived from s1_dwell + extra
        s2_wait_s = max(0.1, s2_dwell - transit_overhead)
        wait_both(s2_wait_s, f"dwell s2 ({s2_dwell:.1f}s)")

    if mode != "overshoot":
        # Phase 5: Right arm moves Sample 2 to Cool Pad spot 2
        right_pick_from_plate_and_place_to_cool(g.COOL_SPOT_2, "s2")
        sync("s2 on cool pad")

        # Phase 6: Right arm presses OFF button
        right_press_button(g.OFF_BTN_POS, "OFF")
        sync("heat off")

        # Phase 7: Final settling hold
        wait_both(15.0, "settle and success hold")

    sync("end")

    return {"left": left, "right": right}


def _mode_from_env() -> str:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return "settle"
    if os.environ.get("DEMO_NAIVE") == "1":
        return "naive"
    if os.environ.get("DEMO_OVERSHOOT") == "1":
        return "overshoot"
    if os.environ.get("DEMO_NO_SAMPLE1") == "1":
        return "no_sample1"
    return "demo"


def plans(mode: str | None = None, variant: int = 0, with_meta: bool = False):
    """Entry point for check_reach, check_collisions, the evaluator and the scripted policy.

    GPU run: the policy calls plans() with no argument, and the old default mode="demo" wiped the DEMO_* flags, so every
    ablation ran the plain demo. With no argument the mode now comes from the environment (run.sh sets the flag).
    """
    if mode is None:
        mode = _mode_from_env()
    old_env = {}
    knobs = {
        "demo": {},
        "naive": {"DEMO_NAIVE": "1"},
        "overshoot": {"DEMO_OVERSHOOT": "1"},
        "no_sample1": {"DEMO_NO_SAMPLE1": "1"},
        "settle": {"DEMO_SETTLE_ONLY": "1"},
    }

    for k in ("DEMO_NAIVE", "DEMO_OVERSHOOT", "DEMO_NO_SAMPLE1", "DEMO_SETTLE_ONLY"):
        old_env[k] = os.environ.get(k)
        if k in os.environ:
            del os.environ[k]

    for k, v in knobs.get(mode, {}).items():
        os.environ[k] = v

    p = demo_plans()

    for k, v in old_env.items():
        if v is not None:
            os.environ[k] = v
        elif k in os.environ:
            del os.environ[k]

    if with_meta:
        meta = {}
        for arm in ("left", "right"):
            for wp in p.get(arm, []):
                kind = "transit"
                lbl = wp.label.lower()
                if "hover" in lbl:
                    kind = "hover"
                elif "descend" in lbl or "lower" in lbl:
                    kind = "descend"
                elif "grip" in lbl or "release" in lbl:
                    kind = "grip"
                elif "lift" in lbl:
                    kind = "lift"
                elif "press" in lbl or "hold" in lbl or "approach" in lbl:
                    kind = "btn_press"
                elif "wait" in lbl:
                    kind = "wait"
                meta[wp.label] = {"kind": kind}
        return p, meta

    return p
