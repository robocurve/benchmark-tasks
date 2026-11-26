"""Stateful scorer for "Land the stripped wire in the empty terminal and tighten it." From the
conductor's and screwdriver's measured poses, the robot's measured TCPs and the clamp-screw turns
reported by task.Dynamics."""
from __future__ import annotations

import math

import numpy as np

import terminal as T

HZ = 20
HOLD_S = 1.0
INTO_MM = 8.0                         # bare end at least this far into the clamp: started in


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0
        self.bad_clamp = False

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("clamp measurements missing")
        self.step = step
        depth, gap, lined = T.in_clamp(root_poses["wire"])
        turns = float(mechanics["screw_turns"])
        full = lined and depth >= T.STRIP - 0.003 and gap <= T.FULL_MM / 1000
        if turns >= T.TURNS_CLAMPED and not full:
            self.bad_clamp = True             # clamped on nothing, on a short-inserted end (copper showing) or askew
        tip, _ = T.driver_tip(root_poses["driver"])
        head = T.head_centre()
        tool_off = float(np.linalg.norm(tip - head)) > 0.02
        clear = all(math.dist(np.asarray(robot[s]["tcp"])[:3], head) > 0.06 for s in ("left", "right"))
        score = 0.0
        if lined and depth >= INTO_MM / 1000:
            score = 0.25
        if full:
            score = 0.5
        if full and turns >= T.TURNS_CLAMPED:
            score = 0.75
        done = score == 0.75 and turns >= T.TURNS_TIGHT and tool_off and clear
        self.held = self.held + 1 if done else 0
        if done and self.held >= HOLD_S * HZ:
            score = 1.0
        if self.bad_clamp:
            score = min(score, 0.5)
        return {"score": score, "success": score == 1.0, "bare_in_mm": round(1000 * depth, 1),
                "insulation_gap_mm": round(1000 * gap, 1), "lined_up": bool(lined), "fully_in": bool(full),
                "screw_turns": round(turns, 2), "bad_clamp": self.bad_clamp, "driver_off": bool(tool_off),
                "hands_clear": bool(clear), "held_frames": self.held}
