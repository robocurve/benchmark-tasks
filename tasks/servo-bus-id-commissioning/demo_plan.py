"""The scripted demo for the servo bus commissioning task.

Bimanual coordination:
- The right arm manages module manipulation on the bus strip (unplugging, staging, replugging).
- The left arm operates the pushbuttons on the controller box (ASSIGN, RESET).
- Arms maintain >= 20 mm clearance at all times by parking the idle right arm during button operations.

Waypoint structure: (duration, pos, finger, label, yaw) played open-loop by yam_scripted.
All tuning knobs are exposed as environment variables (DEMO_*):
- DEMO_OVERSHOOT: socket insertion overshoot along +x (m, default 0.0025)
- DEMO_PUSH_S: push insertion duration (s, default 1.0)
- DEMO_INSERT_DZ: raise the insertion height (m, default 0.0007, set from GPU run 3)
- DEMO_TIME_SCALE: stretch every waypoint duration (default 1.8; 1.0 = the original plan)
- DEMO_GRIP_DX: right gripper grasp offset along x (m, default 0.0)
- DEMO_GRIP_DY: right gripper grasp offset along y (m, default 0.0)
- DEMO_SETTLE_ONLY: if 1, returns empty plan so arms hold initial ready pose (scores 0.0)
- DEMO_ALL_PLUGGED: if 1, presses ASSIGN while all 3 are plugged (scores 0.25)
- DEMO_WRONG_ORDER: if 1, commissions green first, then red, then blue without reset (scores 0.25)
- DEMO_RECOVER: if 1, commissions in the wrong order (green 1, red 2, blue 3), presses RESET (all numbers cleared,
  lamp red again), then commissions in the correct order (scores 1.0; RESET is essential: without it red stays 2)
"""

from __future__ import annotations

import math
import os
from dataclasses import replace

import geometry as g
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import Waypoint

PAD_TO_TCP = 0.007
GRIP_PAST_CONTACT = 0.005
HOUSING_TCP_OFFSET_X = -0.009  # centre of housing along world x (origin is nose root)
SHIFT_DEFAULT = {"left": 0.0, "right": 0.0}


def _f(name: str, default: float) -> float:
    return float(os.environ.get(name, default))


def right_pick_pose(pos, dx=0.0, dy=0.0, dz=0.0):
    grip_dx = _f("DEMO_GRIP_DX", 0.0)
    grip_dy = _f("DEMO_GRIP_DY", 0.0)
    return (pos[0] + HOUSING_TCP_OFFSET_X + dx + grip_dx, pos[1] + dy + grip_dy, pos[2] + PAD_TO_TCP + dz)


def _plan_steps(mode: str):
    """The ordered list of procedure steps for a mode. Steps: ("unplug", [colours]), ("plug", colour), ("press", button, n)."""
    correct = [("unplug", ["green", "blue"]), ("press", "assign", 1), ("plug", "green"), ("press", "assign", 2),
               ("plug", "blue"), ("press", "assign", 3)]
    wrong = [("unplug", ["blue", "red"]), ("press", "assign", 1), ("plug", "red"), ("press", "assign", 2),
             ("plug", "blue"), ("press", "assign", 3)]
    if mode == "all_plugged":
        return [("press", "assign", 1)]
    if mode == "wrong_order":
        return wrong
    if mode == "recover":
        return wrong + [("press", "reset", 0)] + correct
    return correct


def demo_plans() -> dict[str, list[Waypoint]]:
    if os.environ.get("DEMO_SETTLE_ONLY") == "1":
        return {}
    if os.environ.get("DEMO_RECOVER") == "1":
        mode = "recover"
    elif os.environ.get("DEMO_WRONG_ORDER") == "1":
        mode = "wrong_order"
    elif os.environ.get("DEMO_ALL_PLUGGED") == "1":
        mode = "all_plugged"
    else:
        mode = "demo"

    yaw_r = _f("DEMO_YAW_SHIFT_RIGHT", SHIFT_DEFAULT["right"])
    yaw_l = _f("DEMO_YAW_SHIFT_LEFT", SHIFT_DEFAULT["left"])

    finger_open = FINGER_OPEN
    finger_release = -0.027  # 54 mm opening: clears 45 mm housing by 4.5 mm/side while leaving > 15 mm to adjacent socket
    finger_grip = -g.HOUSING[0] / 2 + GRIP_PAST_CONTACT
    finger_btn = -0.015  # closed fist (30 mm between pads) for pressing buttons

    hover_z = g.TOP + 0.12
    # The fingertip is commanded g.BUTTON_PRESS_DEPTH_CMD (14 mm) below the REST cap top: 6 mm beyond the cap's 8 mm end
    # stop, so the stop (not the soft arm) sets the travel; the fingertip stays 6 mm above the controller box.
    press_z = g.BUTTON_PRESS_TCP_Z

    overshoot = _f("DEMO_OVERSHOOT", 0.0025)
    push_s = _f("DEMO_PUSH_S", 1.0)

    home_l = (0.180, 0.220, hover_z)
    home_r = (0.180, -0.220, hover_z)
    btn = {
        "assign": (g.ASSIGN_BTN_POS[0], g.ASSIGN_BTN_POS[1]),
        "reset": (g.RESET_BTN_POS[0], g.RESET_BTN_POS[1]),
    }

    left: list[Waypoint] = []
    right: list[Waypoint] = []

    def dur(wps):
        return sum(w.duration for w in wps)

    def sync(tag: str) -> None:
        """Make both arms equally long by holding the shorter one at its home pose."""
        tl, tr = dur(left), dur(right)
        if tl > tr + 1e-9:
            right.append(Waypoint(tl - tr, home_r, finger_open, f"r: wait {tag}", yaw_r))
        elif tr > tl + 1e-9:
            left.append(Waypoint(tr - tl, home_l, finger_btn, f"l: wait {tag}", yaw_l))

    def unplug(colour: str, first: bool) -> None:
        p0 = right_pick_pose(g.START_POS[colour])
        ps = right_pick_pose(g.STAGE_POS[colour])
        right.extend([
            Waypoint(1.0 if first else 0.8, (p0[0], p0[1], hover_z), finger_open, f"r: to {colour} start hover", yaw_r),
            Waypoint(0.8, p0, finger_release, f"r: descend to {colour}", yaw_r),
            Waypoint(0.5, p0, finger_grip, f"r: grip {colour}", yaw_r),
            Waypoint(0.8, (ps[0], ps[1], p0[2]), finger_grip, f"r: pull {colour} out", yaw_r),
            Waypoint(0.6, ps, finger_grip, f"r: lower {colour} to stage", yaw_r),
            Waypoint(0.5, ps, finger_release, f"r: release {colour}", yaw_r),
            Waypoint(0.6, (ps[0], ps[1], hover_z), finger_release, f"r: lift from {colour}", yaw_r),
        ])

    def plug(colour: str) -> None:
        # GPU run 3: a servo gripped at the staging spot slips about 4.5 mm down in the pads, so the nose met the socket
        # front 4.5 mm low; the socket-side waypoints of the insertion are raised by DEMO_INSERT_DZ to compensate.
        p0 = right_pick_pose(g.START_POS[colour], dz=_f("DEMO_INSERT_DZ", 0.0007))
        ps = right_pick_pose(g.STAGE_POS[colour])
        right.extend([
            # GPU run 2: with only 54 mm open (2.5 mm to spare on one side) a pad landed on the staged servo and knocked it
            # over, so the descent onto the staging spot uses the fully open hand (94 mm); the clamp then centres the servo.
            Waypoint(0.8, (ps[0], ps[1], hover_z), finger_open, f"r: hover over {colour} stage", yaw_r),
            Waypoint(0.6, ps, finger_open, f"r: descend to {colour} stage", yaw_r),
            Waypoint(0.5, ps, finger_grip, f"r: grip {colour} stage", yaw_r),
            Waypoint(0.6, (ps[0], ps[1], p0[2]), finger_grip, f"r: lift {colour} to axis", yaw_r),
            Waypoint(push_s, (p0[0] + overshoot, p0[1], p0[2]), finger_grip, f"r: push {colour} into socket", yaw_r),
            Waypoint(0.4, p0, finger_grip, f"r: hold {colour} seated", yaw_r),
            Waypoint(0.5, p0, finger_release, f"r: release {colour} seated", yaw_r),
            Waypoint(0.6, (p0[0], p0[1], hover_z), finger_release, f"r: lift clear of {colour}", yaw_r),
            Waypoint(0.6, home_r, finger_open, f"r: park home_r after {colour}", yaw_r),
        ])

    def press(which: str, n: int, last: bool) -> None:
        x, y = btn[which]
        end_finger = finger_open if last else finger_btn
        tag = f"{which} {n}" if which == "assign" else which
        left.extend([
            Waypoint(0.5, (x, y, hover_z), finger_btn, f"l: approach {tag}", yaw_l),
            Waypoint(0.5, (x, y, press_z), finger_btn, f"l: press {tag}", yaw_l),
            Waypoint(0.4, (x, y, press_z), finger_btn, f"l: hold {tag}", yaw_l),
            Waypoint(0.5, (x, y, hover_z), finger_btn, f"l: release {tag}", yaw_l),
            Waypoint(0.5, home_l, end_finger, f"l: back to home after {tag}", yaw_l),
        ])

    steps = _plan_steps(mode)
    if mode == "all_plugged":
        left.append(Waypoint(0.8, home_l, finger_btn, "l: ready", yaw_l))
    first_unplug = True
    for i, step in enumerate(steps):
        last = i == len(steps) - 1
        if step[0] == "unplug":
            for j, colour in enumerate(step[1]):
                unplug(colour, first_unplug and j == 0)
            first_unplug = False
            right.append(Waypoint(0.6, home_r, finger_open, f"r: park home_r after unplug", yaw_r))
            sync(f"step {i}")
        elif step[0] == "press":
            sync(f"step {i}")
            press(step[1], step[2], last)
        else:
            sync(f"step {i}")
            plug(step[1])
    sync("end")

    # Make labels unique when a procedure repeats (recover mode): "... (2nd)"
    for lst in (left, right):
        seen: dict[str, int] = {}
        for i, w in enumerate(lst):
            seen[w.label] = seen.get(w.label, 0) + 1
            if seen[w.label] > 1:
                lst[i] = replace(w, label=f"{w.label} ({seen[w.label]}{'nd' if seen[w.label] == 2 else 'th'})")

    # Pad both plans identically so the arms hold still for the success hold, then finish at home with open grippers.
    hold_s = 1.5 if mode != "all_plugged" else 2.0
    right.append(Waypoint(hold_s, home_r, finger_open, "r: final hold", yaw_r))
    left.append(Waypoint(hold_s, home_l, finger_open, "l: final hold", yaw_l))
    # GPU run 1: the soft arm lags the open-loop plan (fingers closed while the hand was still descending), so every
    # duration is stretched by the same factor (both arms, so the hand-offs stay in step). DEMO_TIME_SCALE=1 restores it.
    scale = _f("DEMO_TIME_SCALE", 1.8)
    if scale != 1.0:
        right[:] = [replace(w, duration=w.duration * scale) for w in right]
        left[:] = [replace(w, duration=w.duration * scale) for w in left]
    return {"left": left, "right": right}


def plans() -> dict[str, list[Waypoint]]:
    return demo_plans()
