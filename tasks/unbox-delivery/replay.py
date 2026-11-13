"""Replay measured unboxing telemetry without the simulator."""
import argparse
import json
from scoring import UnboxScorer
from config import CUT_SEGMENTS,CUT_CELLS

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trajectory');args=p.parse_args()
    tracker=UnboxScorer()
    previous=-1.;diagnostic=None
    with open(args.trajectory) as f:
        for line in f:
            s=json.loads(line)
            if s['time']<=previous: raise ValueError('timestamps must strictly increase')
            mode=bool(s.get('diagnostic',False))
            if diagnostic is None:
                diagnostic=mode
                if diagnostic:
                    tracker.cut=[True]*CUT_SEGMENTS;tracker.covered.update(range(CUT_CELLS))
            elif diagnostic!=mode: raise ValueError('mixed diagnostic and full-task telemetry')
            previous=s['time'];tracker.update(s)
    result=tracker.result()
    if diagnostic: result=dict(score=0.,diagnostic_precut=True,diagnostic_milestones=result)
    print(json.dumps(result,indent=2))

if __name__=='__main__': main()
