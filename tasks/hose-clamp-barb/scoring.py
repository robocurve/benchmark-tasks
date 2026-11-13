"""Stateful scorer for "Connect the hose to the pump's outlet and clamp it." From the hose's and clamp's
measured poses, the robot's measured TCPs and the screw turns reported by task.Dynamics."""
from __future__ import annotations

import math

import numpy as np

import hose as H

HZ = 20
HOLD_S = 1.0
HOME_MM = 3.0                         # mouth within this of the outlet boss: pushed fully home


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0
        self.misplaced = False

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("clamp measurements missing")
        self.step = step
        hose = np.asarray(root_poses["hose"], float)
        clamp = np.asarray(root_poses["clamp"], float)
        depth, lined = H.on_barb(hose)
        back = H.clamp_back_from_mouth(hose, clamp)
        on_shank = H.clamp_on_shank(back)
        turns = float(mechanics["screw_turns"])
        home = lined and depth >= H.INSERT - HOME_MM / 1000
        if turns >= H.TURNS_SNUG and not (home and on_shank):
            self.misplaced = True             # clamped down off the barb's shank, or on a hose not home
        driver_tip, _ = H.driver_tip(root_poses["driver"])
        screw = np.asarray(root_poses["screw"], float)[:3]
        tool_off = float(np.linalg.norm(driver_tip - screw)) > 0.02
        clear = all(math.dist(np.asarray(robot[s]["tcp"])[:3], screw) > 0.06 for s in ("left", "right"))
        score = 0.0
        if lined and depth >= H.BEAD_Y - H.TIP_Y + 0.002:
            score = 0.25                      # over the bead
        if home:
            score = 0.5
        if home and on_shank and turns >= H.TURNS_SNUG:
            score = 0.75
        full = score == 0.75 and turns >= H.TURNS_TIGHT and tool_off and clear
        self.held = self.held + 1 if full else 0
        if full and self.held >= HOLD_S * HZ:
            score = 1.0
        if self.misplaced:
            score = min(score, 0.5)
        return {"score": score, "success": score == 1.0, "hose_depth_mm": round(1000 * depth, 1), "hose_home": bool(home),
                "clamp_back_mm": round(1000 * back, 1), "clamp_on_shank": bool(on_shank), "screw_turns": round(turns, 2),
                "clamped_off_shank": self.misplaced, "driver_off": bool(tool_off), "hands_clear": bool(clear),
                "held_frames": self.held}
