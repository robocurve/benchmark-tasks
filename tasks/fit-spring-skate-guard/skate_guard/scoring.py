"""Functional guard fit and physical delivery into the gear bag."""


class Scorer:
    def reset(self):
        self.unsafe = False
        self.settled = 0

    def evaluate(self, s):
        fit = bool(s["guard_engaged"])
        packed = bool(s["boot_in_bag"])
        if packed and not fit:
            self.unsafe = True
        if fit and packed and s["hands_clear"] and s["boot_speed_m_s"] < 0.035:
            self.settled += 1
        else:
            self.settled = 0
        partial = fit and s["hands_clear"] and s["boot_speed_m_s"] < 0.035
        score = 0.0 if self.unsafe else 1.0 if self.settled >= 10 else 0.5 if partial else 0.0
        return {
            "score": score, "success": score == 1.0, "guard_engaged": fit,
            "boot_in_bag": packed, "unsafe_packing": self.unsafe,
            "guard_extension_mm": round(1000 * s["extension_m"], 3),
            "toe_gap_mm": round(1000 * s["toe_gap_m"], 3),
            "heel_gap_mm": round(1000 * s["heel_gap_m"], 3),
            "guard_z_mm": round(1000 * s["guard_z_m"], 3),
            "hands_clear": s["hands_clear"],
        }
