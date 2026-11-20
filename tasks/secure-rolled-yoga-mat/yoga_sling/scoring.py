"""Score attached bands, an unsupported sling lift, and released retention."""

import numpy as np
from yam_arena.ik import quat_xyzw_to_matrix

from . import config as c

MIN_END_MARGIN = .040
MAX_END_MARGIN = .145
MIN_LIFT = .035
MAX_SLIP = .012
LOAD_ACTIONS = 12
RELEASE_ACTIONS = 14
FOAM_RADIUS = c.MAT_RADIUS + .0025


def distance_to_roll(points, center, axis):
    relative = np.asarray(points) - center
    axial = relative @ axis
    radial = np.linalg.norm(relative - axial[:, None] * axis, axis=1)
    end_clearance = np.maximum(np.abs(axial) - c.MAT_LENGTH / 2, 0.)
    side_clearance = np.maximum(radial - FOAM_RADIUS, 0.)
    return np.hypot(end_clearance, side_clearance)


class Scorer:
    def reset(self):
        self.hold = 0
        self.released = 0
        self.reference = None
        self.proved = False

    def evaluate(self, state):
        mat = state["objects"]["mat"]
        sling = state["objects"]["sling"]
        center = np.asarray(mat["pose"][:3])
        axis = quat_xyzw_to_matrix(np.asarray(mat["pose"][3:]))[:, 1]
        springback = mat["joints"]["Springback"]
        wraps = {}
        for name in ("Left", "Right"):
            top = np.asarray(sling["links"][name + "BandTop"])
            bottom = np.asarray(sling["links"][name + "BandBottom"])
            frame = quat_xyzw_to_matrix(top[3:])
            # Midpoint of the open aperture based on the two crown profiles.
            upper_crown = top[:3] + frame[:, 2] * c.BAND_UPPER_HEIGHT
            lower_crown = bottom[:3] - frame[:, 2] * c.BAND_LOWER_HEIGHT
            displacement = (upper_crown + lower_crown) * .5 - center
            y = float(displacement @ axis)
            end_margin = c.MAT_LENGTH / 2 - abs(y)
            transverse = float(np.linalg.norm(displacement - axis * y))
            alignment = float(abs(frame[:, 1] @ axis))
            captures = (MIN_END_MARGIN <= end_margin <= MAX_END_MARGIN
                        and transverse <= .020 and alignment >= .94)
            wraps[name] = {"axial_m": y, "margin_m": end_margin, "radial_m": transverse,
                           "parallel": alignment, "springback_m": springback, "retains": bool(captures)}

        axial = np.array([wraps[name]["axial_m"] for name in ("Left", "Right")])
        n = sum(d["retains"] for d in wraps.values())
        if n == 2 and axial.prod() >= 0.:
            n = 1

        support_top = c.TABLE_TOP + .017
        lowest = float(center[2] - c.MAT_LENGTH * .5 * abs(axis[2]) - FOAM_RADIUS)
        hands = [np.asarray(hand["points"]) for hand in state["hands"].values()]
        hand_distances = [float(distance_to_roll(points, center, axis).min()) for points in hands]
        lift = n == 2 and lowest >= support_top + MIN_LIFT and min(hand_distances) >= .017
        if lift:
            if self.reference is None or float(np.max(np.abs(axial - self.reference))) > MAX_SLIP:
                self.reference = axial.copy()
                self.hold = 0
            self.hold += 1
        else:
            self.reference = None
            self.hold = 0
        self.proved = self.proved or self.hold >= LOAD_ACTIONS

        web = np.array([link[:3] for link in sling["links"].values()])
        hands_clear = all(float(np.min(np.linalg.norm(points[:, None] - web, axis=2))) >= .052
                          for points in hands) and min(hand_distances) >= .050
        sitting = (c.TABLE_TOP - .004 <= lowest <= support_top + .006
                   and np.linalg.norm(np.asarray(mat["velocity"][:3])) < .018)
        valid_release = self.proved and n == 2 and springback <= .008 and sitting and hands_clear
        self.released = self.released + 1 if valid_release else 0
        success = self.released >= RELEASE_ACTIONS
        score = 1. if success else .75 if self.proved and n == 2 else n * .25
        return {"score": score, "success": success, "load_lift_proved": self.proved,
                "band_states": wraps, "mat_height_m": float(center[2]), "lowest_m": lowest,
                "lift_actions": self.hold, "released_actions": self.released,
                "gripper_clearance_m": hand_distances}
