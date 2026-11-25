"""Replay a canonical 20 Hz primary trajectory without Isaac Sim or a policy."""
import argparse
import json
from scoring import TrialScorer

def replay(path):
    scorer=TrialScorer();last=None
    with open(path) as f:
        for line in f:
            row=json.loads(line)
            last=scorer.update(step=row['step'],objects=row['objects'],root_poses=row['root_poses'],
                               robot=row['robot'],mechanics=row['mechanics'])
            if last!=row['metrics'] or last['score']!=row['score']:
                raise ValueError('Recorded and replayed primary metrics or score differ at step '+str(row['step']))
    if last is None:raise ValueError('Empty primary trajectory')
    return last

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('trajectory');a=p.parse_args()
    print(json.dumps(replay(a.trajectory),indent=2))
