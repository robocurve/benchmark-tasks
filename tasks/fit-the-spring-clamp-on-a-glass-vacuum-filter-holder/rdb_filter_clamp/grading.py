"""Grade the assembly and a proof lift by the upper funnel, at 20 Hz."""

from dataclasses import dataclass

import numpy as np

from . import layout as L

CONTROL_HZ = 20
LIFT_HEIGHT = .020       # Upper edge of the blue stripe, relative to the resting surface
MAX_LIFT_HEIGHT = .026   # Upper edge of the stationary gauge
HOLD_TICKS = 20
RELEASE_TICKS = 10
TABLE_TOL = .002
MARK_TOL = .004  # Horizontal distance from the flask's fixed start, based on the reset allowance
CONTACT_N = .05  # numerical noise floor for normal contact forces
SEAT_GAP, SEAT_OFFSET = .003, .004
CLEAR_AXIS = .100
FAIL_TILT, FAIL_TRAVEL = 10., .030


def flask_clearance(position, rotation):
    """Lowest vertex on the flask foot's two collision rings, relative to the table."""
    z_axis = np.asarray(rotation)[2]
    # The two flat foot polygons have 24 vertices each; use their true rotated heights.
    rings = ((L.FLASK_R - .004, 0.), (L.FLASK_R, .006))
    return float(position[2] + min((L.ring(r, z, 24) @ z_axis).min() for r, z in rings))


def upper_support(s, fp, fr):
    """Identify the sole supporting gripper from contact samples and its measured pose."""
    contacts = s.get("contacts")
    if contacts is None:
        return None
    for i, (arm, other) in enumerate((("left", "right"), ("right", "left"))):
        pads = contacts[f"{arm}_tips_on_funnel"]
        if len(pads) != 2 or min(pads) <= CONTACT_N:
            continue
        tcp = fr.T @ (np.asarray(s["tcp"][i]) - fp)
        if np.linalg.norm(tcp[:2]) > .018 or not L.TAPER_TOP <= tcp[2] <= L.FUNNEL_H:
            continue
        if contacts[f"{other}_arm_on_funnel"] > CONTACT_N:
            continue
        if any(contacts[f"{side}_arm_on_{body}"] > CONTACT_N for side in (arm, other)
               for body in ("holder", "knuckle", "jawL", "jawR")):
            continue
        return arm
    return None


@dataclass
class FilterClampGrade:
    holder_start: tuple
    clamp_start: tuple
    failed: str | None = None
    assembled_on_table: bool = False
    lifting_arm: str | None = None
    lift_ticks: int = 0
    lift_verified: bool = False
    release_ticks: int = 0
    last: dict | None = None

    def update(self, s):
        hp, hr = np.asarray(s["holder_pos"]), np.asarray(s["holder_rot"])
        fp, fr = np.asarray(s["funnel_pos"]), np.asarray(s["funnel_rot"])
        cp, cr = np.asarray(s["clamp_pos"]), np.asarray(s["clamp_rot"])
        seat = L.funnel_on_base(hp, hr, fp, fr)
        ring = L.clamp_on_joint(hp, hr, cp, cr, s["jaw_open_deg"])
        tilt = L.tilt_deg(hr)
        foot = flask_clearance(hp, hr)
        on_table = abs(foot) <= TABLE_TOL
        holder_offset = float(np.linalg.norm(hp[:2] - self.holder_start[:2]))
        on_mark = holder_offset <= MARK_TOL
        seated = abs(seat["gap"]) <= SEAT_GAP and seat["offset"] <= SEAT_OFFSET and seat["tilt_deg"] <= 5.
        clamp_on = (ring["jaws_closed"] and ring["off_axis"] <= .005 and abs(ring["height"]) <= .004
                    and ring["tilt_deg"] <= 7.)
        assembled = seated and clamp_on
        if self.failed is None:
            if tilt > FAIL_TILT or holder_offset > FAIL_TRAVEL:
                self.failed = "flask tipped or displaced"
            elif foot > MAX_LIFT_HEIGHT:
                self.failed = "flask foot above the top of the lift gauge"
            elif L.tilt_deg(fr) > 45. or fp[2] < -.05:
                self.failed = "funnel dropped"
            elif (L.tilt_deg(cr) > 60. or cp[2] < -.05
                  or (cp[2] < .020 and np.linalg.norm(cp[:2] - self.clamp_start[:2]) > .050)):
                self.failed = "clamp dropped"
        support = upper_support(s, fp, fr) if assembled else None
        if not assembled or (on_table and not on_mark):
            self.assembled_on_table = False
            self.lift_verified = False
        elif on_table:
            self.assembled_on_table = True
        # Establish sole support while the assembled flask rests on the mark.
        # Setting it down off the mark invalidates the proof.
        if assembled and on_table:
            self.lifting_arm = support if on_mark else None
        elif not assembled or support != self.lifting_arm:
            self.lifting_arm = None
        lifting = (self.assembled_on_table and self.lifting_arm is not None
                   and LIFT_HEIGHT <= foot <= MAX_LIFT_HEIGHT)
        self.lift_ticks = self.lift_ticks + 1 if lifting else 0
        if self.lift_ticks >= HOLD_TICKS:
            self.lift_verified = True
        contacts = s.get("contacts")
        clear = contacts is not None and all(
            contacts[f"{arm}_arm_on_{body}"] <= CONTACT_N
            for arm in ("left", "right") for body in ("holder", "funnel", "knuckle", "jawL", "jawR"))
        clear = clear and all(np.linalg.norm(np.asarray(t)[:2] - hp[:2]) >= CLEAR_AXIS
                              or t[2] >= fp[2] + L.FUNNEL_H + .05 for t in s["tcp"])
        released = self.lift_verified and assembled and on_table and on_mark and tilt <= 3. and clear
        self.release_ticks = self.release_ticks + 1 if released else 0
        score = 0. if self.failed or not assembled else (1. if self.release_ticks >= RELEASE_TICKS else .5)
        self.last = {
            "score": score, "failed": self.failed, "seated": bool(seated), "clamp_on": bool(clamp_on),
            "assembled": bool(assembled), "on_table": bool(on_table), "upper_support": support,
            "on_mark": bool(on_mark), "holder_offset_mm": round(1000 * holder_offset, 3),
            "foot_clearance_mm": round(1000 * foot, 3), "lift_ticks": self.lift_ticks,
            "lift_verified": self.lift_verified, "release_ticks": self.release_ticks, "clear": bool(clear),
            "holder_tilt_deg": round(tilt, 2), "funnel_offset_mm": round(1000 * seat["offset"], 2),
            "funnel_gap_mm": round(1000 * seat["gap"], 2), "funnel_tilt_deg": round(seat["tilt_deg"], 2),
            "clamp_off_axis_mm": round(1000 * ring["off_axis"], 2), "clamp_height_mm": round(1000 * ring["height"], 2),
            "clamp_tilt_deg": round(ring["tilt_deg"], 2), "jaw_open_deg": list(s["jaw_open_deg"]),
        }
        return self.last

    @property
    def finished(self):
        return self.last is not None and self.last["score"] == 1.


@dataclass(frozen=True)
class FilterClampScorer:
    name: str = "filter_clamp_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        g = record.steps[-1].result.info["filter_clamp_grade"]
        return Score(value=g["score"], explanation=(
            f"assembled {g['assembled']}, proof lift {g['lift_verified']}, "
            f"released on table for {g['release_ticks']} ticks, failure {g['failed']}"))


def make_scorer():
    return FilterClampScorer()
