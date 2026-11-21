"""Score physical microfiber coverage and stable return of the finished glass."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.angles = np.pi + np.deg2rad(C.CLOTH_YAW + np.asarray(C.MARK_DEGREES))
        self.directions = np.stack((np.cos(self.angles), np.sin(self.angles),
                                    np.zeros(len(self.angles))), axis=1)
        heights = np.asarray(C.MARK_HEIGHTS)
        radii = np.interp(heights, C.PROFILE_Z, C.PROFILE_R)
        self.spots = radii[:, None] * self.directions
        self.spots[:, 2] = heights
        self.reset()

    def reset(self):
        self.exposure = np.zeros(len(self.angles))
        self.cleared = np.zeros(len(self.angles), dtype=bool)
        self.previous_contact = None
        self.prior_time = None
        self.returned_at = None
        self.damage = False

    def evaluate(self, state):
        t = state["time_s"]
        dt = 0. if self.prior_time is None else max(0., t - self.prior_time)
        self.prior_time = t
        glass = state["objects"]["glass"]
        cloth = state["objects"]["cloth"]
        gp = np.asarray(glass["pose"])
        cp = np.asarray(cloth["pose"])
        pad = np.asarray(cloth["links"]["WorkingFold"])
        glass_R = quat_xyzw_to_matrix(gp[3:])
        pad_R = quat_xyzw_to_matrix(pad[3:])
        glass_inv = glass_R.T

        center = pad[:3] + pad_R @ C.PAD_FACE
        contact = glass_inv @ (center - gp[:3])
        pad_front = glass_inv @ pad_R[:, 0]
        flex = float(cloth["joints"]["FoldCompression"])
        load = C.FLEX_K * max(0., -flex)
        previous = self.previous_contact
        self.previous_contact = contact.copy()
        normal = contact[:2] / max(.001, np.linalg.norm(contact[:2]))
        tangent = np.asarray([-normal[1], normal[0], 0.])
        if previous is None:
            transport = 0.
        else:
            delta = contact - previous
            transport = float(np.hypot(delta @ tangent, delta[2]))
        steady_grasp = gp[2] > C.TABLE + .018 and glass_R[2, 2] > .75
        normal_load = C.LOAD_BAND[0] <= load <= C.LOAD_BAND[1]
        if steady_grasp and normal_load and .00023 <= transport <= .013:
            local = (self.spots @ glass_R.T + gp[:3] - center) @ pad_R
            overlap = ((np.abs(local[:, 0]) < .013) &
                       (np.abs(local[:, 1]) < C.PAD_WIDTH / 2 - .001) &
                       (np.abs(local[:, 2]) < C.PAD_HEIGHT / 2 + .001) &
                       (self.directions @ pad_front < -.58))
            self.exposure[overlap] += dt
            self.cleared |= self.exposure >= .085
        self.damage |= bool(
            gp[2] < C.TABLE - .010 or
            (glass_R[2, 2] < .56 and gp[2] < C.TABLE + .05) or
            (load > 15.0 and steady_grasp and gp[2] + C.MARK_Z - .024 <= center[2]
             <= gp[2] + C.MARK_Z + .024)
        )
        n = int(self.cleared.sum())
        upright = (abs(gp[2] - C.TABLE) < .009 and glass_R[2, 2] > .988 and
                   np.linalg.norm(glass["velocity"][:3]) < .021)
        cloth_set = abs(cp[2] - C.TABLE) < .035 and np.linalg.norm(cloth["velocity"][:3]) < .025
        hands_clear = all(np.linalg.norm(np.asarray(p) - gp[:3] - [.0, .0, .08]) > .060
                          and np.linalg.norm(np.asarray(p) - cp[:3]) > .050
                          for hand in state["hands"].values() for p in hand["points"])
        ready = n == len(self.angles) and upright and cloth_set and hands_clear and not self.damage
        self.returned_at = (t if self.returned_at is None else self.returned_at) if ready else None
        stable = 0. if self.returned_at is None else t - self.returned_at
        success = stable >= .75
        score = 1. if success else (0. if self.damage else round(.8 * n / len(self.angles), 2))
        return dict(score=score, success=success, restored_marks=n, contact_n=round(load, 4),
                    pad_in_glass=contact.tolist(), glass_supported=steady_grasp,
                    wipe_motion_m=transport, upright=bool(upright), cloth_set=bool(cloth_set),
                    hands_clear=bool(hands_clear), damage=bool(self.damage), stable_s=stable)
