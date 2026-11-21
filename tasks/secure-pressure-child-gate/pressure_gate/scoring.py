"""Measure seating of the independently loaded rubber bumpers."""

import math

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.released = 0.

    def evaluate(self, state, elapsed):
        g = state["objects"]["child_gate"]
        joints, links = g["joints"], g["links"]
        angle = math.degrees(joints["Lock"])
        pads = {}
        for side, sign in (("Left", 1), ("Right", -1)):
            for i, height in enumerate(C.CUP_HEIGHTS):
                p = np.asarray(links[f"{side}Pad{i}"][:3])
                compression = -sign*joints[f"{side}Compression{i}"]
                seated = (
                    abs(p[0]-C.GATE_X) <= .006
                    and abs(p[2]-(C.FLOOR_Z+height)) <= .006
                    and .001 <= compression <= .0085
                )
                pads[f"{side}{i}"] = {"position": p.tolist(), "compression_m": compression,
                                        "seated": bool(seated)}
        captured = sum(item["seated"] for item in pads.values())
        clear = all(p[0] < C.GATE_X-.075 or abs(p[1]) > .36 or p[2] > C.FLOOR_Z+C.GATE_HEIGHT+.06
                    for h in state["hands"].values() for p in h["points"])
        locked = C.LEVER_STOP-7. <= angle <= C.LEVER_STOP+1.
        stable = np.linalg.norm(g["velocity"][:3]) <= .015
        secure = captured == 4 and locked and stable
        self.released = self.released + elapsed if secure and clear else 0.
        success = self.released >= .99
        return {"score": 1. if success else .5 if secure else 0.,
                "success": bool(success), "seated_pads": captured, "pads": pads,
                "lock_deg": angle, "extension_m": joints["Width"],
                "hands_clear": bool(clear), "released_seconds": self.released}
