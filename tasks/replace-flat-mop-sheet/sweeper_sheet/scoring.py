"""Measure actual strip overlap under two released latches."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as c


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.lift_checks = self.hold = 0
        self.lifted = False
        self.wipe_origin = None
        self.wipe_distance = 0.

    def evaluate(self, state):
        head = np.asarray(state["head"]["Head"])
        pad = np.asarray(state["cloth"]["Pad"])
        rot = quat_xyzw_to_matrix(head[3:])
        retained = {}
        strip_local = {}
        strip_to_bite = {}
        for tag, side in c.END_POINTS.items():
            strip = np.asarray(state["cloth"][tag + "Strip"])
            orient = quat_xyzw_to_matrix(strip[3:])
            tip = strip[:3] + orient @ [0.,side*c.STRIP_LENGTH*.94,0.]
            local = rot.T @ (tip - head[:3])
            strip_local[tag] = local.tolist()
            clip = np.asarray(state["head"][tag+"Clip"])
            clip_rot = quat_xyzw_to_matrix(clip[3:])
            points = strip[:3] + np.outer(np.linspace(.18,.96,12)*side*c.STRIP_LENGTH, orient[:,1])
            under_jaw = (points - clip[:3]) @ clip_rot
            bite = under_jaw[(np.abs(under_jaw[:,0]) < .023)
                             & (np.abs(side*under_jaw[:,1] - .036) < .008)]
            strip_to_bite[tag] = [] if not len(bite) else bite[:,2].tolist()
            grip_confirmed = bool(len(bite) and np.any((-.0085 < bite[:,2]) & (bite[:,2] < -.0016)))
            tilt = abs(state["head_joints"][tag+"ClipPivot"])
            retained[tag] = bool(abs(local[0]) < .022 and .150 < local[1]*side < .186
                                 and .014 < local[2] < .025 and tilt < .15
                                 and (rot.T @ orient[:,1])[1] < -.85 and grip_confirmed)
        pad_local = rot.T @ (pad[:3] - head[:3])
        flat = bool(abs(pad_local[0]) < .009 and abs(pad_local[1]) < .010
                    and -.009 < pad_local[2] < -.0003
                    and np.dot(rot[:,2], quat_xyzw_to_matrix(pad[3:])[:,2]) > .983)
        height = float(head[2] - c.HEAD_Z)
        handle = np.asarray(state["head"]["Socket"])
        handle_grip = handle[:3] + quat_xyzw_to_matrix(handle[3:]) @ [0.,0.,.036]
        hands = np.array(list(state["hands"].values()))
        near_handle = bool(np.min(np.linalg.norm(hands - handle_grip,axis=1)) < .041)
        cloth_clear = all(h[2] - head[2] > .044 or abs(h[1] - head[1]) > .25
                          or abs(h[0] - head[0]) > .12 for h in hands)
        if all(retained.values()) and flat and height > c.LIFT_HEIGHT and near_handle and cloth_clear:
            self.lift_checks += 1
        else:
            self.lift_checks = 0
        self.lifted |= self.lift_checks >= 5
        hands_clear = all(abs(hand[0] - head[0]) > .11 or abs(hand[1] - head[1]) > .246
                          or hand[2] - head[2] > .14 for hand in state["hands"].values())
        level = bool(abs(height) < .008 and rot[2,2] >= .993)
        on_tile = bool(rot[2,2] >= .978 and c.WIPE_MIN_X-.023 <= head[0] <= c.WIPE_MAX_X
                       and abs(head[1]) < .037 and abs(rot[0,0]) > .96)
        pad_axes = quat_xyzw_to_matrix(pad[3:])
        lowest = pad[2] - np.abs(pad_axes[2]) @ [.0635, c.WRAP_Y, .00225]
        pad_grounded = bool(abs(lowest - c.PLATFORM) < .0022)
        if self.lifted and near_handle and on_tile and pad_grounded and flat and all(retained.values()):
            if self.wipe_origin is None:
                self.wipe_origin = float(head[0])
            self.wipe_distance = max(self.wipe_distance, abs(float(head[0]) - self.wipe_origin))
        else:
            self.wipe_origin = None
        on_fixture = bool(level and np.linalg.norm(head[:2] - [c.X,0.]) < .026)
        down = on_tile or on_fixture
        ready = (all(retained.values()) and self.lifted and flat and on_tile and pad_grounded and hands_clear
                 and self.wipe_distance >= c.WIPE_DISTANCE)
        self.hold = self.hold + 1 if ready else 0
        full = self.hold >= 10
        return {
            "score": 1. if full else .4 * sum(retained.values()) if flat and down and hands_clear else 0.,
            "success": full, "captured": retained, "flat": flat, "set_down": down,
            "lift_verified": self.lifted, "height_m": height, "wipe_distance_m": self.wipe_distance,
            "pad_grounded": pad_grounded,
            "hands_clear": bool(hands_clear), "strip_local_m": strip_local,
            "strip_height_in_bite_m": strip_to_bite, "check_grip_near_handle": near_handle,
        }
