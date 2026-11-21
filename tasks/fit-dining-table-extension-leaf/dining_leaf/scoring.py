"""Score seam fit and positive mechanical capture after both grippers leave."""
import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


def bounds(position, matrix, center, size):
    midpoint = position + matrix @ center
    extent = np.abs(matrix) @ (size / 2)
    return midpoint - extent, midpoint + extent


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0.0

    def evaluate(self, state, elapsed):
        leaf = state["objects"]["leaf"]
        pos, rotation = np.split(np.asarray(leaf["pose"]), [3])
        matrix = Rotation.from_quat(rotation).as_matrix()
        corners = pos + np.array([[.2, -.325, C.THICKNESS/2],
                                  [.2, .325, C.THICKNESS/2]]) @ matrix.T
        gaps = C.SEAM_X - corners[:, 0]
        steps = corners[:, 2] - C.SURFACE_Z
        sides = corners[:, 1] - [-.325, .325]
        joint = state["objects"]["table"]["joints"]["FlipLever"]
        latch = float(np.degrees(joint))
        aligned = np.max(np.abs(gaps)) <= .006 and np.max(np.abs(sides)) <= .008
        level = np.max(np.abs(steps)) <= .004
        near = np.max(np.abs(gaps)) <= .028 and np.max(np.abs(sides)) <= .018
        receiver_min, receiver_max = bounds(pos, matrix, C.RECEIVER, C.RECEIVER_SIZE)
        lever_rot = Rotation.from_rotvec([0., joint, 0.]).as_matrix()
        catch_min, catch_max = bounds(C.LATCH_PIVOT, lever_rot, C.CATCH_END, C.CATCH_SIZE)
        clearance = receiver_min[0] - catch_max[0]
        overlap = np.minimum(receiver_max, catch_max) - np.maximum(receiver_min, catch_min)
        engaged = (76. <= latch <= 86. and -.001 <= clearance <= .008
                   and overlap[1] >= .012 and overlap[2] >= .006)
        clear = all(point[0] <= pos[0] - .24 or abs(point[1]) >= .375
                    or point[2] >= .837
                    for hand in state["hands"].values() for point in hand["points"])
        static = float(np.linalg.norm(leaf["velocity"][:3])) < .008
        retained = aligned and level and engaged and clear and static
        self.hold = self.hold + elapsed if retained else 0.
        score = 1. if self.hold >= .999 else .65 if aligned and level else .30 if near else 0.
        return {
            "score": score, "success": score == 1., "gaps_mm": (1000*gaps).tolist(),
            "steps_mm": (1000*steps).tolist(), "side_offsets_mm": (1000*sides).tolist(),
            "latch_deg": latch, "latch_engaged": engaged, "capture_play_mm": 1000*clearance,
            "capture_overlap_mm": (1000*overlap[1:]).tolist(),
            "leaf_center_x": float(pos[0]), "hands_clear": clear,
            "hold_s": self.hold,
        }
