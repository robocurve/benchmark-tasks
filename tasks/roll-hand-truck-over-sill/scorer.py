"""Photo/ruler-equivalent geometric checks, separated from policy observations."""
from dataclasses import dataclass,asdict
import math
import spec

@dataclass
class Evidence:
    axle_x:float
    wheels_max_x:float
    cart_max_x:float
    pitch_deg:float
    roll_deg:float
    yaw_deg:float
    plate_floor_gap_m:float
    case_on_plate:bool
    case_aligned:bool
    hands_clear:bool
    stable:bool
    inside_workspace:bool
    intact:bool
    wheel_support_gap_m:float

class Scorer:
    def __init__(self):self.score=0.;self.dwell=0;self.invalid=False
    def update(self,e):
        finite=all(math.isfinite(v) for v in asdict(e).values())
        self.invalid |= not finite or not e.inside_workspace or not e.intact or not e.case_on_plate or e.wheel_support_gap_m>spec.WHEEL_SUPPORT_GAP or abs(e.roll_deg)>15 or abs(e.pitch_deg)>35
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        wheels=e.wheels_max_x<=spec.FINISH_X
        finish=(wheels and e.cart_max_x<=spec.FINISH_X and abs(e.pitch_deg)<=3 and abs(e.roll_deg)<=3
                and abs(e.yaw_deg)<=5 and -.002<=e.plate_floor_gap_m<=.002
                and e.hands_clear and e.stable)
        self.dwell=self.dwell+1 if finish else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if wheels else .25 if e.axle_x<=spec.AXLE[0]-.040 else 0.
        return self.score

def regrade(path):
    """Recompute every recorded score from evidence; fail on disagreement."""
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
    import argparse
    import json
    parser = argparse.ArgumentParser(description='Regrade a completed episode evidence trace')
    parser.add_argument('trace')
    print(json.dumps(regrade(parser.parse_args().trace), indent=2))
