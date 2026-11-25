"""Judge working height and native latch geometry in the released scene."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def bounds(pose, local_center, size):
    rotation = quat_xyzw_to_matrix(pose[3:])
    center = np.asarray(pose[:3]) + rotation @ local_center
    radius = np.abs(rotation) @ (size / 2)
    return center - radius, center + radius


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.dwell = 0.
        self.last_height = None

    def evaluate(self, state, *, elapsed):
        obj = state["objects"]["board"]
        links = obj["links"]
        joints = obj["joints"]
        height = links["Worktop"][2]
        slider_y = links["UpperRunner"][1]
        lever_deg = np.degrees(joints["HeightLever"])
        hands_clear = all(
            all(p[0] < .244 or p[2] > height + .155 for p in hand["points"])
            for hand in state["hands"].values()
        )
        lower = abs(height - C.LOWER_Z) <= .012 and abs(slider_y - C.LOWER_Y) <= .012
        shoulder_lo, shoulder_hi = bounds(
            links["UpperRunner"], C.SHOULDER_CENTER, C.SHOULDER_SIZE
        )
        tooth_lo, tooth_hi = bounds(links["ReleaseLever"], C.LOWER_CATCH, C.CATCH_SIZE)
        overlap = np.minimum(shoulder_hi, tooth_hi) - np.maximum(shoulder_lo, tooth_lo)
        catch_gap = shoulder_lo[1] - tooth_hi[1]
        latched = (
            abs(lever_deg) <= 12.
            and -.002 <= catch_gap <= .004
            and min(overlap[0], overlap[2]) >= .004
        )
        stationary = self.last_height is not None and abs(height - self.last_height) < .001
        self.dwell = self.dwell + elapsed if lower and latched and hands_clear and stationary else 0.
        self.last_height = height
        success = self.dwell >= 1.0 - 1e-8
        score = 1.0 if success else .65 if lower and latched else .40 if lower else 0.
        return {
            "score": score, "success": bool(success), "deck_height_m": height+.051,
            "hinge_height_m": height, "upper_slider_y_m": slider_y,
            "handle_deg": float(lever_deg), "locked": bool(latched),
            "catch_gap_m": float(catch_gap),
            "catch_overlap_xz_m": overlap[[0, 2]].tolist(),
            "hands_clear": hands_clear, "released_s": self.dwell,
        }
