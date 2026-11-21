"""Visible band/root coverage and an unobstructed finished band."""
from dataclasses import dataclass,asdict
import math
import spec
@dataclass
class Evidence:
    coverage:float
    root_coverage:float
    head_clean:bool
    applicator_clear:bool
    band_unobstructed:bool
    stable:bool
    inside_workspace:bool
    intact:bool
    chassis_tilt_deg:float
class Scorer:
    def __init__(self):self.score=0.;self.dwell=0;self.invalid=False
    def update(self,e):
        self.invalid|=not all(math.isfinite(v) for v in asdict(e).values()) or not (0<=e.coverage<=1 and 0<=e.root_coverage<=1) or not e.inside_workspace or not e.intact or not e.head_clean or abs(e.chassis_tilt_deg)>15
        if self.invalid:self.score=0.;self.dwell=0;return 0.
        coverage=min(e.coverage,e.root_coverage)
        complete=coverage>=.95 and e.applicator_clear and e.band_unobstructed and e.stable
        self.dwell=self.dwell+1 if complete else 0
        self.score=1. if self.dwell>=spec.DWELL_STEPS else .5 if coverage>=2/3 else .25 if coverage>=1/3 else 0.
        return self.score
def regrade(path):
    import json
    from pathlib import Path
    s=Scorer();rows=[json.loads(x) for x in Path(path).read_text().splitlines()]
    for r in rows:assert s.update(Evidence(**r['evidence']))==r['score'] and s.dwell==r['dwell'] and s.invalid==r['invalid']
    assert rows
    return dict(steps=len(rows),terminal_score=s.score,peak_score=max(r['score'] for r in rows),dwell=s.dwell,invalid=s.invalid,recorded_scores_match=True)
if __name__=='__main__':
    import argparse,json
    p=argparse.ArgumentParser();p.add_argument('trace');print(json.dumps(regrade(p.parse_args().trace),indent=2))
