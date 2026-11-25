"""Acceptance uses accessible mouths, preserved fills and a contained cap strip."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .materials import CAP_ATTACHMENTS, TUBE_RIMS


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.settled_frames = 0

    def evaluate(self, state):
        objects = state["objects"]
        rack = np.asarray(objects["rack"]["pose"])
        caps = np.asarray(objects["caps"]["pose"])
        rack_axes = quat_xyzw_to_matrix(rack[3:])
        cap_axes = quat_xyzw_to_matrix(caps[3:])
        rims = rack[:3] + TUBE_RIMS @ rack_axes.T
        plugs = caps[:3] + CAP_ATTACHMENTS @ cap_axes.T
        diffs = (plugs[None, :, :] - rims[:, None, :]) @ rack_axes
        blocking = (
            (np.linalg.norm(diffs[..., :2], axis=2) <= 0.0055)
            & (diffs[..., 2] >= -0.002)
            & (diffs[..., 2] <= C.FIT_DEPTH + 0.0020)
        )
        open_tubes = int(np.sum(~np.any(blocking, axis=1)))

        cap_ends = plugs[[0, -1]]
        cap_ends_inside = np.all(
            np.abs(cap_ends[:, :2] - C.DISH[:2])
            < np.asarray(C.DISH_SIZE[:2]) / 2 - 0.006
        )
        in_dish = (
            bool(cap_ends_inside)
            and C.DISH[2] + 0.003 < caps[2] < C.DISH[2] + 0.016
            and np.linalg.norm(objects["caps"]["velocity"][:3]) < 0.025
            and np.linalg.norm(objects["caps"]["velocity"][3:]) < 1.5
        )
        data = state["material"]
        retained = min(data["remaining_ul"]) >= 90.0
        rack_steady = (
            data["max_rack_lift_m"] <= 0.004
            and data["max_rack_shift_m"] <= 0.015
            and data["max_rack_tilt_deg"] <= 8.0
        )
        if open_tubes == 8 and in_dish and retained and rack_steady:
            self.settled_frames += 1
        else:
            self.settled_frames = 0
        success = self.settled_frames >= 10
        score = (
            0.0
            if not (retained and rack_steady)
            else 1.0
            if success
            else 0.5
            if open_tubes == 8
            else 0.25
            if open_tubes >= 4
            else 0.0
        )
        return {
            "score": score,
            "success": success,
            "open_tubes": open_tubes,
            "caps_in_dish": in_dish,
            "liquid_retained": retained,
            "rack_steady": rack_steady,
            "settled_frames": self.settled_frames,
            **data,
        }
