"""Score ladder for building a two-card A-frame; pure Python so it can be re-run on a recorded trace."""
from dataclasses import dataclass
import json
import math
import sys
import spec


@dataclass
class Evidence:
    near_out: bool              # the card is out of its stand
    far_out: bool
    near_held: bool             # the robot touched the card during the step
    far_held: bool
    near_on_stand: bool         # the card touched either stand during the step
    far_on_stand: bool
    near_lean_deg: float        # from vertical
    far_lean_deg: float
    near_low_m: float           # lowest point above the felt
    far_low_m: float
    apex_gap_m: float           # between the two top edges
    feet_gap_m: float           # between the two bottom edges
    facing: bool                # each card leans towards the other's bottom edge
    inside: bool                # every card and stand in the workspace, no card fallen off the table
    intact: bool                # the table, mat and pedestals unmoved


def is_frame(e):
    """Both cards on the felt, leaning towards each other with their top edges together and their bottom edges
    apart, touching no stand."""
    lo, hi = spec.LEAN_RANGE
    return (e.near_low_m <= spec.ON_FELT and e.far_low_m <= spec.ON_FELT and e.apex_gap_m <= spec.APEX_GAP
            and e.facing and e.feet_gap_m >= spec.FEET_GAP
            and lo <= e.near_lean_deg <= hi and lo <= e.far_lean_deg <= hi
            and not e.near_on_stand and not e.far_on_stand)


class Scorer:
    def __init__(self):
        self.score = 0.
        self.invalid = False
        self.standing = 0

    def update(self, e):
        if self.invalid:
            return 0.
        numbers = [e.near_lean_deg, e.far_lean_deg, e.near_low_m, e.far_low_m, e.apex_gap_m, e.feet_gap_m]
        if not all(math.isfinite(x) for x in numbers) or not e.inside or not e.intact:
            self.invalid = True
            self.score = 0.
            return 0.
        if e.near_out and e.far_out and e.near_held and e.far_held:
            self.score = max(self.score, .25)
        frame = is_frame(e)
        if frame:
            self.score = max(self.score, .5)
        self.standing = self.standing + 1 if frame and not e.near_held and not e.far_held else 0
        if self.standing >= spec.STAND_STEPS:
            self.score = 1.
        return self.score


def regrade(path):
    scorer = Scorer()
    score = 0.
    for line in open(path):
        row = json.loads(line)
        score = scorer.update(Evidence(**row['evidence']))
    return score, scorer.invalid


if __name__ == '__main__':
    score, invalid = regrade(sys.argv[1])
    print(json.dumps({'score': score, 'invalid': invalid}))
