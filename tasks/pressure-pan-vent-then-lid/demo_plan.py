"""Scripted demo for pressure_pan_vent_then_lid. The left arm presses the red vent button and holds it; the right arm
waits clear of the steam until the plume has gone, lifts the lid by its knob, carries it level to the trivet and sets it
down; the left arm lets the vent go once the lid is up. Positions are commanded gripper centres (open loop, times only)."""

from __future__ import annotations

import os

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TABLE_HEIGHT
from yam_arena.demo import Waypoint

TOP = TABLE_HEIGHT
SCALE = float(os.getenv("DEMO_TIME_SCALE", "1.0"))


def _s(t: float) -> float:
    return max(0.1, t * SCALE)


L_READY = (0.10, 0.14, TOP + 0.12)
R_READY = (0.10, -0.14, TOP + 0.12)

# Left arm: closed fingers push the button's face (x 0.2505 at rest, z 0.045 to 0.105) 28 mm toward +x. The finger front
# is about 10 mm ahead of the commanded centre, so contact starts at x 0.2405 and the final command presses 3 mm into the stop.
BTN_Y = 0.165
BTN_Z = TOP + 0.090
BTN_X_NEAR = 0.215
BTN_X_PUSH = 0.272

# Right arm: the knob (4 cm across, 5 cm tall) stands on the pan's centre (0.28, 0); the lid's underside rests at 0.0802.
KNOB_X, KNOB_Y = 0.28, 0.0
GRIP_Z = TOP + 0.0802 + 0.032          # low on the knob (pads reach 13 mm below the centre; knob base at 0.0842)
LIFT_Z = GRIP_Z + 0.045                # lid 4.5 cm above the rim (the arm's upper reach is 0.16)
TRIVET_Y = -0.205
SET_Z = TOP + 0.0102 + 0.032           # lid underside on the trivet top (1.0 cm)
WAIT_POS = (0.20, -0.10, TOP + 0.13)   # 15 cm from the valve's axis, outside the steam zone

PLUME_WAIT_S = 10.5                    # right arm waits: the vent is passed at about 6.4 s, the plume lasts 5 s


def make_settle_plan() -> dict[str, list[Waypoint]]:
    return {
        "left": [Waypoint(_s(8.0), L_READY, FINGER_OPEN, "left settle")],
        "right": [Waypoint(_s(8.0), R_READY, FINGER_OPEN, "right settle")],
    }


def _left_press():
    return [
        Waypoint(_s(2.5), (BTN_X_NEAR, BTN_Y, TOP + 0.13), FINGER_OPEN, "over the red button, behind it"),
        Waypoint(_s(1.5), (BTN_X_NEAR, BTN_Y, BTN_Z), FINGER_OPEN, "down to the button's height"),
        Waypoint(_s(0.8), (BTN_X_NEAR, BTN_Y, BTN_Z), FINGER_CLOSED, "close the fingers"),
        Waypoint(_s(3.0), (BTN_X_PUSH, BTN_Y, BTN_Z), FINGER_CLOSED, "push the button in through its stroke"),
    ]


def _left_clear():
    return [
        Waypoint(_s(0.8), (BTN_X_PUSH, BTN_Y, BTN_Z), FINGER_OPEN, "let the vent go"),
        Waypoint(_s(1.0), (BTN_X_PUSH, BTN_Y, TOP + 0.13), FINGER_OPEN, "rise clear of the button"),
        Waypoint(_s(2.0), L_READY, FINGER_OPEN, "back to ready"),
    ]


def make_demo_plan(ablation: bool = False) -> dict[str, list[Waypoint]]:
    if ablation:
        # left arm only: press, hold 12 s, let go; the right arm stays parked
        left = _left_press() + [Waypoint(_s(12.0), (BTN_X_PUSH, BTN_Y, BTN_Z), FINGER_CLOSED, "hold the vent in")]
        left += _left_clear() + [Waypoint(_s(3.0), L_READY, FINGER_OPEN, "wait")]
        return {"left": left, "right": [Waypoint(_s(30.0), R_READY, FINGER_OPEN, "ablation: right arm parked")]}
    left = _left_press()
    left_t = sum(w.duration for w in left)

    right = [
        Waypoint(_s(2.5), WAIT_POS, FINGER_OPEN, "wait clear of the steam"),
        Waypoint(_s(PLUME_WAIT_S), WAIT_POS, FINGER_OPEN, "keep waiting until the steam has stopped"),
        Waypoint(_s(1.5), (KNOB_X, KNOB_Y, TOP + 0.15), FINGER_OPEN, "over the knob"),
        Waypoint(_s(1.5), (KNOB_X, KNOB_Y, GRIP_Z), FINGER_OPEN, "down to the knob"),
        Waypoint(_s(0.8), (KNOB_X, KNOB_Y, GRIP_Z), FINGER_CLOSED, "pinch the knob"),
        Waypoint(_s(1.5), (KNOB_X, KNOB_Y, LIFT_Z), FINGER_CLOSED, "lift the lid straight up, level"),
    ]
    right.append(Waypoint(_s(1.0), (KNOB_X, KNOB_Y, LIFT_Z), FINGER_CLOSED, "steady at the top"))
    right.append(Waypoint(_s(3.0), (KNOB_X, TRIVET_Y, LIFT_Z), FINGER_CLOSED, "carry the lid level over the trivet"))
    right_t = sum(w.duration for w in right)
    # the left arm keeps the vent pressed until the lid (and its lug) has been carried clear of the lock arm, then lets go
    # and clears while the right arm lowers the lid
    left.append(Waypoint(max(0.1, right_t - left_t), (BTN_X_PUSH, BTN_Y, BTN_Z), FINGER_CLOSED, "hold the vent in"))
    left += _left_clear()
    right.append(Waypoint(_s(1.5), (KNOB_X, TRIVET_Y, SET_Z), FINGER_CLOSED, "lower the lid onto the trivet"))
    right.append(Waypoint(_s(0.5), (KNOB_X, TRIVET_Y, SET_Z), FINGER_CLOSED, "settle"))
    right.append(Waypoint(_s(0.8), (KNOB_X, TRIVET_Y, SET_Z), FINGER_OPEN, "release the knob"))
    right.append(Waypoint(_s(1.0), (KNOB_X, TRIVET_Y, TOP + 0.12), FINGER_OPEN, "lift straight up"))
    right.append(Waypoint(_s(2.0), R_READY, FINGER_OPEN, "back to ready"))
    right.append(Waypoint(_s(3.0), R_READY, FINGER_OPEN, "wait for stillness"))
    return {"left": left, "right": right}


def plans():
    if os.getenv("SETTLE_ONLY"):
        return make_settle_plan()
    if os.getenv("ABLATION"):
        return make_demo_plan(ablation=True)
    return make_demo_plan(ablation=False)
