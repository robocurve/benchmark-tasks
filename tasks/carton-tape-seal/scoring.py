"""Stateful scorer for "Tape the carton shut." From the tape cells reported by task.Dynamics, the gun's
measured pose and the robot's measured TCPs."""
from __future__ import annotations

import math

import numpy as np

import tape as T

HZ = 20
HOLD_S = 1.0
CENTRE_MM = 10.0                     # the strip's centre within this of the seam all along
FLUSH_DEG = 3.0                      # a flap standing open more than this under the tape is a gap
TOP_FULL = 1.0                       # every cell along the top stuck (8 mm cells)


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("tape measurements missing")
        self.step = step
        m = mechanics
        frac = m["top_stuck"] / m["top_cells"]
        centred = m["max_offset_mm"] <= CENTRE_MM
        tabs = min(m["near_tab_mm"], m["far_tab_mm"]) >= T.TAB_MIN * 1000
        now_open = max(T.flap_angle(n, root_poses[n]) for n in T.FLAPS)
        flush = m["max_gap_deg"] <= FLUSH_DEG and now_open <= FLUSH_DEG   # flat where taped, and flat now
        gun = np.asarray(root_poses["gun"], float)
        roller, _, _, up = T.gun_points(gun)
        grip = gun[:3]
        free = all(math.dist(np.asarray(robot[s]["tcp"])[:3], grip) > 0.10 or float(np.mean(robot[s]["fingers"])) < -0.040
                   for s in ("left", "right"))                    # no closed hand on the gun
        carton = np.asarray(root_poses["carton"], float)
        clear = all(math.dist(np.asarray(robot[s]["tcp"])[:2], carton[:2]) > 0.12 or
                    float(np.asarray(robot[s]["tcp"])[2]) > T.TOP_SURF + 0.08 for s in ("left", "right"))
        score = 0.0
        if m["started"] and frac >= 0.5:
            score = 0.25
        if frac >= TOP_FULL and centred and m["cut"]:
            score = 0.5
        if score == 0.5 and tabs:
            score = 0.75
        full = score == 0.75 and flush and free and clear
        self.held = self.held + 1 if full else 0
        if full and self.held >= HOLD_S * HZ:
            score = 1.0
        return {"score": score, "success": score == 1.0, "top_covered": round(frac, 3), "centred": bool(centred),
                "cut": bool(m["cut"]), "near_tab_mm": m["near_tab_mm"], "far_tab_mm": m["far_tab_mm"],
                "flaps_flush": bool(flush), "gun_released": bool(free), "hands_clear": bool(clear), "held_frames": self.held}
