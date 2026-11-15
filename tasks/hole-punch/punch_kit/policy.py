"""Scripted demo for the hole punch, from the robot's own joint angles and the setup's fixed positions only.

It reads no simulator state. The right arm, gripper open, pushes the card's right edge with both fingertips until the
card's middle is on the punch's centre line, then pushes its back edge until its front edge is against the back stop. Then the left
arm presses the lever's handle end down until the head bottoms out, holds, and lets it back up.

Modes (``-P mode=...``) for the failure checks:
  demo    the full demo;
  nocard  the left arm presses the lever with the card left where it starts (expected 0.0);
  offset  the card is aimed 4 mm off its marks sideways (it lands 2-3 mm off), then the press (expected 0.0).
The environment variable DEMO_AIM_SHIFT_MM="dx,dy" shifts where the right arm aims the card (margin runs).
"""

from __future__ import annotations

import os

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, READY
from yam_arena import ik
from yam_arena.demo import grip_rotation
from yam_arena.scripted import YamScriptedPolicy

from . import geometry as g
from .motion import Arm, Move

CLOSED = 0.0
OPEN = FINGER_OPEN
DOWN = grip_rotation(0.0)
PRESS_EXTRA = 0.030  # the lever end is commanded 30 mm below where it bottoms out: the compliant arm presses firmly


SIDEWAYS = grip_rotation(np.pi / 2)  # fingers open along world x


def _at(x, y, tip_z, rot=DOWN):
    """TCP target for fingertips at (x, y, tip_z), gripper straight down (fingers open along world y unless turned)."""
    pos = np.array([x, y, tip_z + PAD_TIP_PAST_TCP])
    return lambda sim: (pos, rot)


class PunchPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        if mode not in ("demo", "nocard", "offset", "air"):
            raise ValueError("mode is demo, nocard, offset or air")
        self.mode = mode

    def reset(self, scene):
        self._built = False

    def _build(self):
        shift = np.zeros(2)
        if os.environ.get("DEMO_AIM_SHIFT_MM"):
            shift = np.array([float(v) for v in os.environ["DEMO_AIM_SHIFT_MM"].split(",")]) / 1000.0
        if self.mode == "offset":
            shift = shift + np.array([0.0, 0.004])  # lands the card about 2-3 mm off its marks
        self.right = Arm(self._bases["right"])
        self.left = Arm(self._bases["left"])
        low = g.BED + 0.0005  # fingertips 0.5 mm above the stand: they meet the card's 1.5 mm edge
        # pushing from behind, the compliant arm lifts its tips 1-3 mm and they ride over the card's edge; aimed 4 mm
        # into the foam stand, they rest on its surface instead and keep pushing the edge
        sink = g.BED - 0.004
        if self.mode == "air":  # diagnostics: the same push 3 cm up, touching nothing
            sink = g.BED + 0.03
        cx, cy = g.CARD_START[:2]
        right_edge = cy - g.CARD_Y / 2
        tail_edge = cx - g.CARD_X / 2
        # 1) with the gripper open and turned 90 degrees, its two fingertips (94 mm apart along x) push the card's right
        #    edge until the card's middle is on the centre line; two contacts keep it from turning
        y0 = right_edge - g.TIP_HALF_X - 0.010
        y1 = -g.CARD_Y / 2 - g.TIP_HALF_X + 0.0012 + shift[1]  # +1.2 mm: the arm stops about 1 mm short
        # 2) gripper open along y, its two fingertips push the card's back edge until its front edge is on the stop,
        #    then ease off
        x0 = tail_edge - g.TIP_HALF_X - 0.010
        x1 = g.STOP_X - g.CARD_X - g.TIP_HALF_X + 0.008 + shift[0]  # aimed 8 mm past: the arm lags ~4 mm, the stop holds
        y_mid = shift[1]
        if self.mode != "nocard":
            self.right.push(
                Move(2.5, _at(cx, y0, low + 0.03, SIDEWAYS), OPEN, "beside card"),
                Move(1.0, _at(cx, y0, low, SIDEWAYS), OPEN, "down beside card"),
                Move(2.0, _at(cx, y1, low, SIDEWAYS), OPEN, "push card sideways"),
                Move(0.8, _at(cx, y1 - 0.003, low + 0.03, SIDEWAYS), OPEN, "lift"),
                Move(2.0, _at(x0, y_mid, low + 0.03), OPEN, "behind card"),
                Move(0.8, _at(x0, y_mid, sink), OPEN, "down behind card"),
                Move(3.5, _at(x1, y_mid, sink), OPEN, "push card home"),
                Move(0.6, _at(x1 - 0.010, y_mid, sink), OPEN, "ease off"),
                Move(0.8, _at(x1 - 0.004, y_mid, low + 0.04), OPEN, "lift off"),
            )
        self.right_end = sum(m.duration for m in self.right.queue)
        lever_top = g.LEVER_PIVOT[2] + g.LEVER_THICK / 2
        bottom = lever_top - (g.LEVER_PIVOT[0] - g.PRESS_POINT[0]) * g.STROKE / (g.LEVER_PIVOT[0] - g.PIN_X)
        px, py = g.PRESS_POINT
        self.left.push(
            # the two grippers collide near the centre line: the left arm waits until the right arm is home
            Move(self.right_end, None, OPEN, "wait"),
            Move(4.0, _at(px, py, lever_top + 0.04), CLOSED, "above lever"),
            Move(2.0, None, CLOSED, "settle above lever"),
            Move(1.5, _at(px, py, lever_top + 0.002), CLOSED, "on lever"),
            Move(4.0, _at(px, py, bottom - PRESS_EXTRA), CLOSED, "press"),
            Move(2.0, None, CLOSED, "hold"),
            Move(2.0, _at(px, py, lever_top + 0.04), CLOSED, "release"),
        )
        self._built = True

    def act(self, observation):
        if not self._built:
            self._build()
        t = float(observation.state_time) + self._dt
        left = self.left.act(None, t)
        right = self.right.act(None, t)  # with nothing queued an arm holds where its last move ended
        meta = {"request_stop": True} if self.left.idle and self.right.idle else {}
        return ActionChunk(actions=[Action(data=pack(left, right), meta=meta)], control_hz=1.0 / self._dt)
