"""Numeric checks for leash bolt-snap on D-ring."""
from dataclasses import dataclass, asdict
import math
import spec


@dataclass
class Evidence:
    hooked: bool
    gate_closed: bool
    hands_clear: bool
    stable: bool
    inside_workspace: bool
    intact: bool
    hook_xy_err_m: float
    hook_z_err_m: float
    gate_open_m: float


class Scorer:
    def __init__(self):
        self.score = 0.
        self.dwell = 0
        self.invalid = False

    def update(self, e: Evidence):
        finite = all(math.isfinite(v) if isinstance(v, float) else True for v in asdict(e).values())
        self.invalid |= (not finite) or (not e.inside_workspace) or (not e.intact)
        if self.invalid:
            self.score = 0.
            self.dwell = 0
            return 0.
        hooked = e.hooked
        latched = hooked and e.gate_closed
        full = latched and e.hands_clear and e.stable
        self.dwell = self.dwell + 1 if full else 0
        if self.dwell >= spec.DWELL_STEPS:
            self.score = 1.
        elif latched:
            self.score = .5
        elif hooked:
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
    p = argparse.ArgumentParser(); p.add_argument('trace')
    print(json.dumps(regrade(p.parse_args().trace), indent=2))
