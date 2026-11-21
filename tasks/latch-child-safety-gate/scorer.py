"""Numeric geometric checks for the child-safety-gate close/lock outcome."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    door_yaw_rad: float
    lever_rad: float
    base_lock_rad: float
    latch_overlap_m: float
    door_closed: bool
    lever_locked: bool
    base_locked: bool
    latch_engaged: bool
    hands_clear: bool
    stable: bool
    inside_workspace: bool
    intact: bool


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        # Set once the door is closed (within 7°) with both hands clear; a later reopen past
        # REOPEN_LIMIT (12° from fully closed) then zeros the trial permanently.
        self._released_closed = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) if isinstance(v, float) else True for v in asdict(e).values())
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact)
        if e.door_closed and e.hands_clear:
            self._released_closed = True
        if self._released_closed and abs(e.door_yaw_rad - spec.DOOR_CLOSED_YAW) > spec.REOPEN_LIMIT:
            self.invalid = True
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        closed = e.door_closed
        top = closed and e.lever_locked and e.latch_engaged
        full = top and e.base_locked and e.hands_clear and e.stable
        self.dwell = self.dwell + 1 if full else 0
        if self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif top:
            self.score = .5
        elif closed:
            self.score = .25
        else:
            self.score = 0.
        return self.score


def regrade(path):
    import json
    from pathlib import Path
    scorer = Scorer()
    count = 0
    peak = 0.
    for line in Path(path).read_text().splitlines():
        row = json.loads(line)
        score = scorer.update(Evidence(**row['evidence']))
        if score != row['score'] or scorer.invalid != row['invalid'] or scorer.dwell != row['dwell']:
            raise ValueError(f"Recorded score mismatch at step {row['step']}")
        count += 1
        peak = max(peak, score)
    if not count:
        raise ValueError('Empty evidence trace')
    return {
        'steps': count,
        'terminal_score': scorer.score,
        'peak_score': peak,
        'dwell': scorer.dwell,
        'invalid': scorer.invalid,
        'recorded_scores_match': True,
    }


if __name__ == '__main__':
    import argparse
    import json
    parser = argparse.ArgumentParser()
    parser.add_argument('trace')
    print(json.dumps(regrade(parser.parse_args().trace), indent=2))
