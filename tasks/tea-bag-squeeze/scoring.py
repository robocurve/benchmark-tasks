"""Stateful scorer for taking a tea bag out of a mug of tea, from measured poses, the robot's measured
TCPs and the squeeze, drip and touch records reported by task.Dynamics."""
from __future__ import annotations

import math

import numpy as np

import tea as T

HZ = 20
HOLD_S = 1.0
CLEAR = 0.08                          # m: a hand counts as let go this far from the bag, its tag and the spoon
MUG_MOVED = 0.010                     # m
MUG_TILT = 10.0                       # degrees
SPILL_MOVED = 0.030
SPILL_TILT = 30.0


class TrialScorer:
    def __init__(self):
        self.held = 0
        self.step = 0
        self.mug0 = None
        self.moved = 0.0
        self.tilt = 0.0

    def update(self, step, objects, root_poses, robot, mechanics=None, **_):
        if step != self.step + 1:
            raise ValueError("scorer needs one update per control step")
        if mechanics is None:
            raise ValueError("squeeze and drip records missing")
        self.step = step
        mug = np.asarray(root_poses["mug"], float)
        bag = np.asarray(root_poses["bag"], float)
        tag = np.asarray(root_poses["tag"], float)
        spoon = np.asarray(root_poses["spoon"], float)
        if self.mug0 is None:
            self.mug0 = mug[:2].copy()
        self.moved = max(self.moved, float(np.linalg.norm(mug[:2] - self.mug0)))
        self.tilt = max(self.tilt, T.tilt_deg(mug))
        spilled = self.moved > SPILL_MOVED or self.tilt > SPILL_TILT
        still = lambda n: float(np.linalg.norm(objects[n].lin_vel)) < 0.02
        bag_out = bool(T.bag_on_plate(bag) and still("bag"))
        squeezed = float(mechanics["squeezed_s"]) >= T.SQUEEZE_S and not mechanics["fingers_on_bag"]
        clean = int(mechanics["drops_on_mat"]) == 0 and self.moved < MUG_MOVED and self.tilt < MUG_TILT
        spoon_down = bool(T.spoon_on_plate(spoon) and still("spoon"))
        things = [bag[:3], tag[:3], T.spoon_at(spoon, T.GRASP_S), T.spoon_at(spoon, T.BOWL_S)]
        clear = all(float(np.linalg.norm(np.asarray(robot[s]["tcp"], float)[:3] - p)) > CLEAR
                    for s in ("left", "right") for p in things)
        score = 0.0
        if bag_out:
            score = 0.25
        if bag_out and squeezed:
            score = 0.5
        if bag_out and squeezed and clean:
            score = 0.75
        full = score == 0.75 and spoon_down and clear
        self.held = self.held + 1 if full else 0
        if full and self.held >= HOLD_S * HZ:
            score = 1.0
        if spilled:
            score = 0.0
        return {"score": score, "success": score == 1.0, "bag_on_plate": bag_out, "squeezed": bool(squeezed),
                "squeezed_s": round(float(mechanics["squeezed_s"]), 2), "drops_on_mat": int(mechanics["drops_on_mat"]),
                "mug_moved_mm": round(1000 * self.moved, 1), "mug_tilt_deg": round(self.tilt, 1), "spilled": bool(spilled),
                "spoon_on_plate": spoon_down, "hands_clear": bool(clear), "held_frames": self.held}
