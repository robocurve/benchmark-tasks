"""Stateful scorer for "Give the steel plate a coat of primer." The brush's measured pose, the hands'
measured TCPs and the coat reported by task.Dynamics."""
from __future__ import annotations

import math

import numpy as np

import paint as P

from yam_arena.constants import PAD_TIP_PAST_TCP as PAD_TIP

HZ = 20
HOLD_S = 1.0
DRIP_LIMIT = 1          # primer dripped on the bench (outside plate and tray)


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("coat measurements missing")
        self.step = step
        brush = np.asarray(root_poses["brush"], float)
        grip = P.grip_point(brush)
        holding = [s for s in ("left", "right")
                   if float(np.linalg.norm(np.asarray(robot[s]["tcp"], float)[:3] - grip)) < PAD_TIP + 0.025
                   and float(np.mean(robot[s]["fingers"])) > -0.043]
        tips, _, tilt = P.bristle_tips(brush)
        laid_down = (not holding and tilt < 15 and tips[2] < P.TOP + P.TRAY_WALL + 0.01
                     and not P.on_plate(brush[:3]) and float(np.linalg.norm(objects["brush"].lin_vel)) < 0.02)
        cov = float(mechanics["coverage"])
        score = 0.0
        if cov >= 0.30:
            score = 0.25
        if cov >= 0.60:
            score = 0.5
        drips = int(mechanics["drips"])
        smeared = int(mechanics["bench_smear_steps"]) > 0      # primer brushed onto the bench
        if cov >= 0.95 and drips <= DRIP_LIMIT and not smeared:
            score = 0.75
        full = score == 0.75 and laid_down
        self.held = self.held + 1 if full else 0
        if full and self.held >= HOLD_S * HZ:
            score = 1.0
        return {"score": score, "success": score == 1.0, "coverage": cov, "dips": mechanics["dips"],
                "brush_load_ml": mechanics["load_ml"], "drips": drips, "bench_smeared": smeared, "brush_in_hand": holding, "brush_laid_down": bool(laid_down),
                "held_frames": self.held}
