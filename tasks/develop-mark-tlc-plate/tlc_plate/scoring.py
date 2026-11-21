"""Trial history and simple physical checks for a developed TLC plate."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

PLATE_CORNERS = np.array([
    [x, y, z]
    for x in (-C.PLATE_WIDTH / 2, C.PLATE_WIDTH / 2)
    for y in (-C.PLATE_LENGTH / 2, C.PLATE_LENGTH / 2)
    for z in (-C.PLATE_THICK / 2, C.PLATE_THICK / 2)
])


def rigid_pose(state, name):
    pose = state["objects"][name]["pose"]
    return np.array(pose[:3]), quat_xyzw_to_matrix(np.array(pose[3:]))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.front = C.INITIAL_FRONT
        self.extraction = None
        self.front_at_extraction = None
        self.mark_points = []
        self.hold = 0.
        self.strokes = []
        self.last_contact = None
        self.invalid = False
        self.inserted_again = False

    def evaluate(self, state, elapsed=.05):
        t = state["time_s"]
        plate, rotation = rigid_pose(state, "plate")
        lid, _ = rigid_pose(state, "cap")
        bottom = plate - C.PLATE_LENGTH / 2 * rotation[:, 1]
        in_tank = bool(
            np.all(np.abs(bottom[:2] - C.TANK[:2]) < np.asarray(C.TANK_SIZE[:2]) / 2 - C.WALL)
        )
        liquid_top = C.TANK[2] + C.WALL + C.SOLVENT_DEPTH
        dipped = bool(in_tank and bottom[2] <= liquid_top + .0015)
        lid_open = bool(np.linalg.norm(lid - C.LID) > .017)
        if dipped:
            if self.extraction is None:
                self.front = min(C.PLATE_LENGTH, np.sqrt(C.INITIAL_FRONT**2 + C.CAPILLARY_K * t))
            else:
                self.inserted_again = True
        if self.extraction is None and not dipped and lid_open:
            self.extraction = float(t)
            self.front_at_extraction = float(self.front)
        on_time = self.extraction is not None and C.FRONT_MIN <= self.front <= C.FRONT_MAX
        pencil = state["objects"]["pencil"]
        tip = np.asarray(pencil["links"]["Graphite"][:3])
        pressure = max(0., 285. * pencil["joints"]["LeadTravel"])
        local_tip = rotation.T @ (tip - plate)
        # The graphite cushion deflects under native axial contact. A hand
        # squeezing the pencil barrel does not drive this joint.
        front_error = float(local_tip[1] + C.PLATE_LENGTH / 2 - self.front)
        valid_tip = bool(
            abs(local_tip[0]) <= C.PLATE_WIDTH / 2
            and .00075 <= local_tip[2] <= .0020
            and abs(front_error) <= C.MARK_TOLERANCE
        )
        fresh = self.extraction is not None and t - self.extraction <= C.MARK_DEADLINE
        if on_time and fresh and valid_tip and .07 <= pressure <= .68:
            x, y = map(float, local_tip[:2])
            gap = (
                np.linalg.norm(np.asarray(self.mark_points[-1]) - (x, y))
                if self.mark_points else np.inf
            )
            start = self.last_contact is None or t - self.last_contact > .17 or gap > .004
            if start:
                self.strokes.append([])
            self.last_contact = t
            if start or gap >= .001:
                self.mark_points.append((x, y))
                self.strokes[-1].append(x)
        spread = max((float(np.ptp(stroke)) for stroke in self.strokes), default=0.)
        two_ticks = len(self.strokes) > 1 and np.ptp(
            [float(np.median(stroke)) for stroke in self.strokes]
        ) >= .032
        mark = spread >= .011 or two_ticks
        lowest_corner = float(np.min(plate[2] + PLATE_CORNERS @ rotation[2]))
        self.invalid |= self.extraction is not None and lowest_corner <= C.TOP + .0012
        velocity = np.linalg.norm(state["objects"]["plate"]["velocity"][:3])
        face_up = bool(
            np.linalg.norm(plate[:2] - C.PLACEMENT[:2]) < .021
            and abs(plate[2] - C.PLACEMENT[2]) < .009
            and rotation[0, 2] <= -.83
            and velocity <= .012
        )
        clear = all(
            np.linalg.norm(np.asarray(point) - plate) > .070
            for hand in state["hands"].values() for point in hand["points"]
        )
        if on_time:
            score = .55
        elif self.extraction is not None:
            score = .25
        else:
            score = 0.
        if mark:
            score = .80
        complete = on_time and mark and face_up and clear
        self.hold = self.hold + elapsed if complete else 0.
        if self.inserted_again or self.invalid:
            score = 0.
        elif self.hold >= .75:
            score = 1.
        return {
            "score": score,
            "success": score == 1.,
            "front_mm": float(self.front * 1000.),
            "extraction_s": self.extraction,
            "timely_extraction": bool(on_time),
            "mark_span_mm": spread * 1000.,
            "mark_points": len(self.mark_points),
            "graphite_load_N": float(pressure),
            "pencil_local_mm": (local_tip * 1000.).tolist(),
            "pencil_front_error_mm": front_error * 1000.,
            "dipped": dipped,
            "dropped_or_smeared": bool(self.invalid),
            "lid_open": lid_open,
            "returned_to_solvent": self.inserted_again,
            "plate_in_drying_rest": face_up,
            "hands_clear": clear,
            "hold_s": self.hold,
        }
