"""Score the tuning, as task.json's scoring says. Within 5 cents of G4: 1.0 if the G string was plucked after the G
button's last turn and both grippers' centre points are at least 50 mm from the ukulele and tuner at the end,
otherwise 0.75. Over 5 and up to 15 cents off: 0.75; up to 30: 0.5; up to 50: 0.25; over 50 either way: 0. Whatever
the G string reads, 0 if the C, E or A string is more than 15 cents from its start.

On the bench, after the trial, the judge plucks each string once over the soundhole and reads a tuner that shows
cents (a phone tuner app), checks on the trial video that the robot plucked the G string after its last turn of the
G button, and measures the grippers' distance in the video's last frame. In simulation the same bands are applied to
each string's computed pitch (``tuning.Pitch``) at the last step; a pluck is one the stated rule counts
(``tuning.PluckDetector``); the G button counts as turned after a pluck once it is more than the gear's free play
(4 degrees) from its angle at that pluck (``tuning.FinalCheck``); and the distance is measured from each gripper's
centre point to boxes around the ukulele's parts and the tuner (``layout.clear_boxes``). The trial ends at the run
limit or once 1.0 has held for 1 s. Pure Python, so the rules can be unit-tested without the simulator.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math

from . import layout as L
from .tuning import FinalCheck

CONTROL_HZ = 20
FULL_CENTS = 5.                  # 1.0 within this of G4 (with both grippers clear and the final check)
BANDS = ((15., .75), (30., .5), (50., .25))     # otherwise this score up to this many cents from G4; beyond: 0
MOVED_CENTS = 15.                # a C, E or A string further than this from its start: 0
CLEAR_M = L.CLEAR_M              # both grippers' centre points at least this far from the ukulele and tuner
CHECK_TURN_DEG = L.FREE_PLAY_DEG # the G button further than this from its angle at the last pluck: turned since
END_TICKS = CONTROL_HZ           # the trial ends once 1.0 has held for 1 s
SCORES = (0., .25, .5, .75, 1.)
OTHERS = L.OTHERS
BOXES = L.clear_boxes()


def box_distance(point, box):
    """Distance from ``point`` to an axis-aligned box ((x0, x1), (y0, y1), (z0, z1)); 0 inside it."""
    return math.sqrt(sum(max(lo - p, 0., p - hi) ** 2 for p, (lo, hi) in zip(point, box)))


def clearance(point, boxes=BOXES):
    """Distance from a gripper's centre point (instrument frame) to the nearest part of the ukulele or the tuner."""
    return min(box_distance(point, box) for box in boxes)


def band(cents, clear, checked):
    """The score from the G string's final pitch (cents from G4), whether both grippers are clear and whether the G
    string was plucked after its button last turned."""
    error = abs(cents)
    if error <= FULL_CENTS and clear and checked:
        return 1.
    for limit, score in BANDS:
        if error <= limit:
            return score
    return 0.


@dataclass
class TuningGrade:
    start: dict = field(default_factory=lambda: dict(L.START_CENTS))
    success_ticks: int = 0           # consecutive steps at 1.0
    check: FinalCheck = field(default_factory=lambda: FinalCheck(CHECK_TURN_DEG))
    tick: int = 0

    def update(self, cents, tool_points, plucked, button_deg):
        """One 20 Hz step.

        cents: each string's pitch, cents from its target note ({"g", "c", "e", "a"}; G from G4).
        tool_points: the two grippers' centre points (from the joint readings), instrument frame, metres; or None
        when they cannot be read (then they do not count as clear).
        plucked: a pluck of the G string counted on this step (``tuning.PluckDetector``).
        button_deg: the G button's angle, degrees, unwrapped, any fixed zero (``tuning.Pitch.button``).
        """
        self.tick += 1
        checked = self.check.update(self.tick, plucked, button_deg)
        moved = {s: float(cents[s]) - float(self.start[s]) for s in OTHERS}
        worst = max(OTHERS, key=lambda s: abs(moved[s]))
        void = (f"the {worst.upper()} string moved {abs(moved[worst]):.1f} cents from its start"
                if abs(moved[worst]) > MOVED_CENTS else None)
        gaps = None if tool_points is None else [clearance(point) for point in tool_points]
        clear = gaps is not None and all(gap >= CLEAR_M for gap in gaps)
        g = float(cents[L.TUNED])
        score = 0. if void else band(g, clear, checked)
        self.success_ticks = self.success_ticks + 1 if score == 1. else 0
        turned = self.check.turned_since_pluck(button_deg)
        return {"score": score, "g_cents": round(g, 3), "g_start_cents": float(self.start[L.TUNED]),
                "moved_cents": {s: round(v, 3) for s, v in moved.items()},
                "grippers_clear": bool(clear),
                "clearance_mm": None if gaps is None else [round(1000 * g, 1) for g in gaps],
                "final_check": bool(checked), "last_pluck_step": self.check.pluck_tick,
                "g_turned_since_pluck_deg": None if turned is None else round(turned, 2),
                "success_ticks": self.success_ticks, "void": void}

    @property
    def finished(self):
        """1.0 has held for 1 s (21 steps in a row, the last 20 steps after the first): the trial ends (only after the
        final check: 1.0 needs it)."""
        return self.success_ticks > END_TICKS


@dataclass(frozen=True)
class TuningScorer:
    name: str = "ukulele_tuning_score"

    def __call__(self, record, target):
        from inspect_robots.scorer import Score

        if not record.steps:
            return Score(value=0., explanation="No completed control steps")
        grade = record.steps[-1].result.info["tuning_grade"]
        return Score(value=grade["score"],
                     explanation=f"G string {grade['g_cents']} cents from G4 (start {grade['g_start_cents']}); "
                                 f"plucked after the last turn of its button: {grade['final_check']}; "
                                 f"C, E, A moved {grade['moved_cents']} cents; grippers clear: "
                                 f"{grade['grippers_clear']} {grade['clearance_mm']} mm; void: {grade['void']}")


def make_scorer():
    return TuningScorer()
