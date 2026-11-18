"""Scripted demo for the hole punch, from the top camera's first image, the robot's own joint angles and the punch's
fixed position only.

It reads no simulator state. Before moving, it finds the card in the top camera's image (punch_kit/vision.py: yellow
pixels back-projected with the station's calibrated camera), which gives the card's centre and yaw. The right arm,
gripper part-open and turned so its two fingertips lie along x, comes down beside the card's right edge and pushes it
until the card is square and its middle is on the punch's centre line; then, gripper open along y, it pushes the
card's back edge until its front edge is against the back stop. Then the left arm presses the lever's handle knob
down until the punches have gone through the card, holds, and lets the lever back up.

Modes (``-P mode=...``) for the failure checks:
  demo    the full demo;
  nocard  the left arm presses the lever with the card left where it starts (expected 0.0);
  offset  the card is aimed 4 mm off its marks sideways (it lands 2-3 mm off), then the press (expected 0.0);
  short   the card is put on its marks, but the press stops early: the tips come down onto the card and are held
          there without the force to cut it (expected 0.0; the old release-at-the-card's-top-face rule scored this);
  air     diagnostics: the push behind the card runs 3 cm up, touching nothing.
The environment variable DEMO_AIM_SHIFT_MM="dx,dy" shifts where the right arm aims the card (margin runs), and
DEMO_TOP_CAM names the top camera variant if the embodiment was started with ``-E top_cam=demo``.
"""

from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.packing import pack
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import grip_rotation
from yam_arena.scripted import YamScriptedPolicy

from . import geometry as g
from .motion import Arm, Move
from .vision import TopCamera, find_card

CLOSED = 0.0
OPEN = FINGER_OPEN
SPREAD = 0.64 * FINGER_OPEN  # fingertips about 60 mm apart: both stay on the card's 105 mm side at up to 10 deg yaw
DOWN = grip_rotation(0.0)
PRESS_EXTRA = 0.090  # the knob is commanded 90 mm below where the lever bottoms out (the compliant arm presses harder the further its target is)
SHORT_EXTRA = 0.004  # short mode: the knob commanded 4 mm below where the lever bottoms out (the demo uses 90 mm):
# the compliant arm brings the tips down onto the card but presses too softly to cut it
XF_FINAL = 0.215     # step 3: fingertips centred 14.5 mm behind the card's middle, clear of the housing (x 26.2 cm)
BEHIND_GAP = 0.012   # fingertips come down this far behind the card's back edge

SIDEWAYS = grip_rotation(np.pi / 2)  # fingers open along world x


def _at(x, y, tip_z, rot=DOWN):
    """TCP target for fingertips at (x, y, tip_z), gripper straight down (fingers open along world y unless turned)."""
    pos = np.array([x, y, tip_z + PAD_TIP_PAST_TCP])
    return lambda sim: (pos, rot)


class PunchPolicy(YamScriptedPolicy):
    def __init__(self, mode: str = "demo", **kwargs):
        super().__init__(**kwargs)
        if mode not in ("demo", "nocard", "offset", "short", "air"):
            raise ValueError("mode is demo, nocard, offset, short or air")
        self.mode = mode

    def reset(self, scene):
        self._built = False

    def _locate(self, observation):
        """The card's centre (x, y) and yaw from the top camera's image."""
        rgb = np.asarray(observation.images["top_cam"])
        cam = TopCamera.station(rgb.shape[1], rgb.shape[0], os.environ.get("DEMO_TOP_CAM", "calibrated"))
        found = find_card(rgb, cam)
        if found is None:
            raise RuntimeError("hole_punch_scripted: no card seen in the top camera's image")
        x, y, yaw, area = found
        print(json.dumps({"card_seen": {"x": round(x, 4), "y": round(y, 4), "yaw_deg": round(math.degrees(yaw), 2),
                                        "area_cm2": round(area * 1e4, 1)}}), file=sys.stderr, flush=True)
        return x, y, yaw

    def _build(self, observation):
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
        if self.mode != "nocard":
            cx, cy, yaw = self._locate(observation)
            s, c = abs(math.sin(yaw)), abs(math.cos(yaw))
            half_y = g.CARD_Y / 2 * c + g.CARD_X / 2 * s  # the card's half-extent across y, as it lies
            # 1) gripper turned 90 degrees, its two fingertips (about 60 mm apart along x) push the card's right edge
            #    until the card's middle is on the centre line. They are centred halfway between the card's middle
            #    and its right edge's middle, so both stay on the edge while it turns square: the first touch is at
            #    the edge's low end, which turns the card toward square, and the second tip then holds it square
            xf = cx + 0.5 * (g.CARD_Y / 2) * math.sin(yaw)
            y0 = cy - half_y - g.TIP_HALF_X - 0.010
            y1 = -g.CARD_Y / 2 - g.TIP_HALF_X + 0.0034 + shift[1]  # +3.4 mm: the arm stops about 2.2 mm short
            # 2) gripper open along y, its two fingertips push the card's back edge until its front edge is on the
            #    stop, then ease off. The card is square now and its centre still near cx
            x0 = max(cx - g.CARD_X / 2 - g.TIP_HALF_X - BEHIND_GAP, g.STAND_X0 + g.TIP_HALF_X + 0.002)
            x1 = g.STOP_X - g.CARD_X - g.TIP_HALF_X + 0.008 + shift[0]  # aimed 8 mm past: the arm lags, the stop holds
            y_mid = shift[1]
            # the first sideways push aims 16 mm short of the centre line: squaring a card turned near -10 degrees in
            # the back push slides it 2-8 mm toward +y, so the last push (step 3) only ever has to push +y
            ya = y1 - 0.016
            self.right.push(
                Move(2.5, _at(xf, y0, low + 0.03, SIDEWAYS), SPREAD, "beside card"),
                Move(1.0, _at(xf, y0, low, SIDEWAYS), SPREAD, "down beside card"),
                Move(2.5, _at(xf, ya, low, SIDEWAYS), SPREAD, "push card sideways"),
                Move(0.8, _at(xf, ya - 0.003, low + 0.03, SIDEWAYS), SPREAD, "lift"),
                Move(2.0, _at(x0, y_mid, low + 0.03), OPEN, "behind card"),
                Move(0.8, _at(x0, y_mid, sink), OPEN, "down behind card"),
                Move(3.5, _at(x1, y_mid, sink), OPEN, "push card home"),
                Move(0.6, _at(x1 - 0.010, y_mid, sink), OPEN, "ease off"),
                Move(0.8, _at(x1 - 0.004, y_mid, low + 0.04), OPEN, "lift off"),
                # 3) gripper turned again, a last push on the right edge, the card now square against the stop, puts
                #    its middle on the centre line (the first push stops short, so this one only pushes +y)
                Move(1.5, _at(XF_FINAL, ya - 0.008, low + 0.03, SIDEWAYS), SPREAD, "beside card again"),
                Move(0.6, _at(XF_FINAL, ya - 0.008, low, SIDEWAYS), SPREAD, "down beside card again"),
                # (8 mm beyond where the first push left the edge: the back push only ever moves it toward +y)
                Move(3.0, _at(XF_FINAL, y1, low, SIDEWAYS), SPREAD, "push card onto the centre line"),
                Move(1.0, _at(XF_FINAL, y1 - 0.004, low + 0.06, SIDEWAYS), SPREAD, "lift clear"),
            )
        self.right_end = sum(m.duration for m in self.right.queue)
        lever_top = g.LEVER_PIVOT[2] + g.LEVER_THICK / 2
        ratio = (g.LEVER_PIVOT[0] - g.PRESS_POINT[0]) / (g.LEVER_PIVOT[0] - g.PIN_X)
        bottom = lever_top - ratio * g.STROKE
        target = bottom - PRESS_EXTRA
        if self.mode == "short":  # a softer press: the tips reach the card but do not cut it
            target = bottom - SHORT_EXTRA
        px, py = g.PRESS_POINT
        self.left.push(
            # the two grippers collide near the centre line: the left arm waits until the right arm is home
            Move(self.right_end, None, OPEN, "wait"),
            Move(4.0, _at(px, py, lever_top + 0.04), CLOSED, "above lever"),
            Move(2.0, None, CLOSED, "settle above lever"),
            Move(1.5, _at(px, py, lever_top + 0.002), CLOSED, "on lever"),
            Move(4.0, _at(px, py, target), CLOSED, "press"),
            Move(2.0, None, CLOSED, "hold"),
            Move(2.0, _at(px, py, lever_top + 0.04), CLOSED, "release"),
        )
        self._built = True

    def act(self, observation):
        if not self._built:
            self._build(observation)
        t = float(observation.state_time) + self._dt
        left = self.left.act(None, t)
        right = self.right.act(None, t)  # with nothing queued an arm holds where its last move ended
        meta = {"request_stop": True} if self.left.idle and self.right.idle else {}
        return ActionChunk(actions=[Action(data=pack(left, right), meta=meta)], control_hz=1.0 / self._dt)
