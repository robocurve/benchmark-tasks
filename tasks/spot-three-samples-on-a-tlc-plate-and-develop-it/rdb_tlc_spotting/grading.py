"""Score the trial: twelve checks, one twelfth each.

For each of the three lanes: (a) one spot of its own sample covers its tick, (b) the spot fits a printed 3 mm circle,
(c) no bare glass longer than 1 mm in the lane; (b) and (c) count only with (a). For the plate: (d) it stands in the
tank, right way up, its bottom edge in the solvent and its baseline at least 3 mm above the solvent; (e) it leans back
against the back wall, spotted face away from it, at most 35 degrees from vertical; (f) the lid rests on the tank,
centred, level and square; (e) and (f) count only with (d). 1.0 also needs all three spotters standing back in the
rack and both grippers clear of the tank and its lid; otherwise at most 11/12.

Latched 0 for the rest of the trial: a spotter broken (more than 3 N on it from anything but the pads) or dropped
(lying more than 60 degrees from upright with no gripper holding it, or its tip on the table outside the rack and
vials); the plate dropped (lying face down on the bench, where the spots smear, or fallen off the bench; lying face up
is not a zero); the baseline under the solvent while the plate is in the tank (the spots wash out); the tank tipped
past 10 degrees or pushed more than 5 mm. A real trial judges the same checks from a video of the plate on its bar,
a loupe, a ruler and a protractor (task.json). Pure Python.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from . import layout as L
from . import tlc

CONTROL_HZ = L.CONTROL_HZ
ELEMENTS = 12
CAP = 11 / 12                      # all twelve checks but spotters out of the rack or a gripper close: at most this
END_TICKS = round(L.END_S * CONTROL_HZ)
SCORES = tuple(round(k / ELEMENTS, 4) for k in range(ELEMENTS + 1))


def box_distance(point, centre, half):
    """Distance from a point to an upright box around ``centre`` (its base's middle) with half sizes ``half`` (0 inside)."""
    d = (max(0., abs(point[0] - centre[0]) - half[0]), max(0., abs(point[1] - centre[1]) - half[1]),
         max(0., centre[2] - point[2], point[2] - (centre[2] + 2 * half[2])))
    return math.sqrt(sum(v * v for v in d))


@dataclass
class TLCGrade:
    loads: tlc.Loads = field(default_factory=tlc.Loads)
    spots: tlc.Spots = field(default_factory=tlc.Spots)
    void: str | None = None
    success_ticks: int = 0
    tick: int = 0
    tank_start: tuple = (L.TANK_X, L.TANK_Y, 0.)

    def update(self, poses, plate_force, other_force, tool_points, pad_force=None):
        """One 20 Hz step.

        poses: {name: (position, quaternion xyzw)} of the plate, the spotters, the tank and the lid,
        tabletop frame. plate_force: each spotter's contact force on the plate (N). other_force: each spotter's
        contact force from anything but the grippers' pads and the plate (N). tool_points: both grippers' centre
        points (tabletop frame) or None. pad_force: each spotter's total contact force from the grippers' pads (N),
        or None (no spotter held).
        """
        self.tick += 1
        names = ("spotter_1", "spotter_2", "spotter_3")
        tips = [tlc.spotter_tip(poses[n]) for n in names]
        self.loads.update(tips)
        plate = poses["plate"]
        for k in range(3):
            self.spots.update(k, float(plate_force[k]), tlc.to_local(plate, tips[k]), self.loads.sample(k))
        self._voids(poses, tips, plate_force, other_force, pad_force)
        lanes = [self.spots.lane(k) for k in range(3)]
        lane_points = sum(int(x["covered"]) + int(x["small"]) + int(x["clean"]) for x in lanes)
        state = tlc.plate_in_tank(plate, poses["tank"])
        standing = state["inside"] and state["right_way_up"] and state["bottom_wet"] and \
            state["baseline_above_mm"] >= 1000 * L.BASELINE_CLEAR
        leaning = standing and state["on_wall"] and state["face_out"] and state["lean_deg"] <= L.LEAN_MAX_DEG
        lid = tlc.lid_on(poses["lid"], poses["tank"])
        covered = standing and lid["on"]
        points = lane_points + int(standing) + int(leaning) + int(covered)
        racked = [tlc.in_rack(poses[n]) for n in names]
        tank = poses["tank"][0]
        half = (max(L.TANK[0], L.LID[0]) / 2, max(L.TANK[1], L.LID[1]) / 2, (L.TANK[2] + L.LID[2]) / 2)
        lid_pos = poses["lid"][0]
        gaps = None if tool_points is None else [
            min(box_distance(p, tank, half),
                box_distance(p, (lid_pos[0], lid_pos[1], lid_pos[2] - L.LID[2] / 2), (L.LID[0] / 2, L.LID[1] / 2, L.LID[2] / 2)))
            for p in tool_points]
        clear = gaps is not None and all(g >= L.CLEAR_M for g in gaps)
        tidy = all(r is not None for r in racked) and len(set(racked)) == 3
        score = points / ELEMENTS
        if points == ELEMENTS and not (tidy and clear):
            score = CAP
        if self.void:
            score = 0.
        score = round(score, 4)
        self.success_ticks = self.success_ticks + 1 if score == 1. else 0
        return {"score": score, "points": points, "lanes": lanes, "standing": bool(standing),
                "leaning": bool(leaning), "covered": bool(covered), "plate": state, "lid": lid,
                "racked": racked, "tidy": bool(tidy), "grippers_clear": bool(clear),
                "clearance_mm": None if gaps is None else [round(1000 * g, 1) for g in gaps],
                "loads": [self.loads.sample(k) for k in range(3)],
                "spots": [{**{k: (round(1000 * v, 2) if k in ("x", "y") else v) for k, v in s.items()},
                           "diameter_mm": round(1000 * tlc.Spots.diameter(s), 2), "seconds": round(s["seconds"], 3)}
                          for s in self.spots.spots],
                "gouges": [{k: (round(1000 * v, 2) if k in ("x", "y", "length") else v) for k, v in g.items()}
                           for g in self.spots.gouges],
                "touches": self.spots.touches, "void": self.void, "success_ticks": self.success_ticks}

    def _voids(self, poses, tips, plate_force, other_force, pad_force=None):
        if self.void:
            return
        for k, name in enumerate(("spotter_1", "spotter_2", "spotter_3")):
            if max(float(plate_force[k]), float(other_force[k])) > L.BREAK_N:
                self.void = f"spotter {k + 1} broke ({max(float(plate_force[k]), float(other_force[k])):.1f} N)"
                return
            held = pad_force is not None and float(pad_force[k]) >= L.HELD_N
            inside = any(tlc.Loads.in_bore(tips[k], j) for j in range(3)) or tlc.in_rack(poses[name]) is not None
            if (tlc.tilt_deg(poses[name]) > L.DROP_TILT_DEG and not held) or (tips[k][2] < .003 and not inside):
                self.void = f"spotter {k + 1} dropped"
                return
        plate = poses["plate"]
        corners = tlc.plate_corners(plate)
        low = min(c[2] for c in corners)
        state = tlc.plate_in_tank(plate, poses["tank"])
        lying = tlc.tilt_deg(plate, 0) > 60.                     # flat-ish, not standing on an edge in a hand
        face_down = tlc.axis(plate, 2)[2] < 0.                   # the silica face toward the bench
        if low < -.01:
            self.void = "the plate was dropped (it fell off the bench)"
        elif low < .002 and lying and face_down and not state["inside"]:
            self.void = "the plate was dropped (it lies face down on the bench: the spots smear)"
        elif state["inside"] and state["baseline_above_mm"] < 0.:
            self.void = "the baseline is under the solvent: the spots wash out"
        elif tlc.tilt_deg(poses["tank"]) > L.TANK_TIP_DEG or math.hypot(
                poses["tank"][0][0] - self.tank_start[0], poses["tank"][0][1] - self.tank_start[1]) > L.TANK_SHIFT:
            self.void = "the tank was tipped or pushed"

    @property
    def finished(self):
        """1.0 has held for a second, or a zero condition stopped the trial."""
        return self.success_ticks > END_TICKS or self.void is not None


@dataclass(frozen=True)
class TLCScorer:
    name: str = "tlc_spotting_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["tlc_grade"]
        lanes = "; ".join(f"lane {k + 1}: {'covered' if x['covered'] else 'not covered'}, "
                          f"{x['diameter_mm']} mm, {'gouged' if x['gouged'] else 'clean'}"
                          for k, x in enumerate(grade["lanes"]))
        return Score(value=grade["score"],
                     explanation=f"{grade['points']}/12 checks; {lanes}; plate standing {grade['standing']}, "
                                 f"leaning {grade['leaning']} ({grade['plate']['lean_deg']} deg), covered "
                                 f"{grade['covered']}; spotters in the rack {grade['racked']}; grippers clear "
                                 f"{grade['grippers_clear']}; void: {grade['void']}")


def make_scorer():
    return TLCScorer()
