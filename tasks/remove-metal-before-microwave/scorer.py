"""Numeric checks for fork removal before microwave start."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    fork_in_cavity: bool
    fork_on_safe_spot: bool
    dish_in_cavity: bool
    button_latched: bool
    hands_clear: bool
    stable: bool
    inside_workspace: bool
    intact: bool
    fork_xy_err_m: float
    fork_tilt_rad: float
    button_travel_m: float


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False
        self.metal_start = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) if isinstance(v, float) else True for v in asdict(e).values())
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact)
        # Starting with metal still in the cavity permanently fails.
        if e.button_latched and e.fork_in_cavity:
            self.metal_start = True
        self.invalid |= self.metal_start
        # Dish must not leave the cavity before the fork is clear (instruction is heat leftovers).
        if (not e.dish_in_cavity) and e.fork_in_cavity:
            self.invalid = True
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        partial = e.fork_on_safe_spot and (not e.fork_in_cavity)
        half = partial and e.dish_in_cavity
        full = half and e.button_latched and e.hands_clear and e.stable
        self.dwell = self.dwell + 1 if full else 0
        if self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif half:
            self.score = .5
        elif partial:
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
    return {'steps': count, 'terminal_score': scorer.score, 'peak_score': peak,
            'dwell': scorer.dwell, 'invalid': scorer.invalid, 'recorded_scores_match': True}


if __name__ == '__main__':
    import argparse, json
    p = argparse.ArgumentParser()
    p.add_argument('trace')
    print(json.dumps(regrade(p.parse_args().trace), indent=2))
