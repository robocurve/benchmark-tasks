"""Track air transport under a rolling nip on the compliant stack."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pockets = np.array([y for _, y, _ in C.GAPS])
        self.travel = np.zeros(len(C.GAPS))
        self.cleared = np.zeros(len(C.GAPS), dtype=bool)
        self.vents = [None] * len(C.GAPS)
        self.previous = None
        self.hold = 0
        self.overload_steps = 0
        self.damaged = False

    def evaluate(self, state):
        cassette = state["objects"]["cassette"]
        roller = state["objects"]["roller"]
        shear = cassette["joints"]["Shear"]
        compression = max(0., -cassette["joints"]["Compression"])
        closed_angle = C.LID_OPEN_DEG + np.degrees(cassette["joints"]["Closing"])
        axle = np.asarray(roller["links"]["Drum"][:3])
        layer = np.asarray(cassette["links"]["Layer"][:3])
        relative = axle - layer
        spin = roller["joints"]["Spin"]
        if self.previous is None:
            previous_y, previous_spin = relative[1], spin
        else:
            previous_y, previous_spin = self.previous
        self.previous = relative[1], spin
        dy = relative[1] - previous_y
        surface = -C.ROLLER_R * (spin - previous_spin)
        direction = np.sign(dy)
        overloaded = compression > C.GEL_DAMAGE_DEPTH or abs(shear) > C.GEL_DAMAGE_SHIFT
        self.overload_steps = self.overload_steps + 1 if overloaded else 0
        self.damaged |= self.overload_steps >= 3
        half_width = C.SHEET_WIDTH / 2
        rolling = (
            not self.damaged
            and .00055 <= compression
            and abs(relative[2] - C.ROLLER_R - .007) <= .0018
            and abs(shear) <= .004
            and .00015 <= abs(dy) <= .004
            and abs(relative[1]) <= half_width + .0015
            and 0.18 * abs(dy) <= direction * surface <= 2.8 * abs(dy) + .00035
        )
        if rolling:
            start = direction * previous_y + C.ROLLER_NIP
            end = direction * relative[1] + C.ROLLER_NIP
            for k, (gx, _, radius) in enumerate(C.GAPS):
                if self.cleared[k] or abs(relative[0] - gx) >= C.ROLLER_WIDTH / 2 - radius:
                    continue
                pocket = direction * self.pockets[k]
                if start - C.ROLLER_NIP <= pocket < end:
                    distance = min(abs(dy), end - pocket)
                    self.travel[k] += distance
                    self.pockets[k] += direction * distance
                    if direction * self.pockets[k] >= half_width:
                        self.pockets[k] = direction * half_width
                        self.cleared[k] = True
                        self.vents[k] = round(1000 * self.pockets[k], 3)
        aligned = abs(shear) <= .003
        closed = abs(closed_angle) <= 2.
        ready = closed and aligned and all(self.cleared) and not self.damaged
        self.hold = self.hold + 1 if ready else 0
        complete = self.hold >= 10
        score = 1. if complete else .25 * int(self.cleared.sum())
        if self.damaged:
            score = 0.
        return {
            "score": score, "success": complete, "gaps_cleared": self.cleared.tolist(),
            "pockets_y_mm": (1000 * self.pockets).round(4).tolist(),
            "pocket_travel_mm": (1000 * self.travel).round(4).tolist(),
            "vent_edge_y_mm": self.vents.copy(), "damaged": bool(self.damaged),
            "laminate_shear_mm": 1000 * shear, "compression_mm": 1000 * compression,
            "cover_angle_deg": closed_angle,
            "roller_xyz": axle.tolist(), "roll_rad": spin, "rolling": bool(rolling),
        }
