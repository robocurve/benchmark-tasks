"""Replay read-only native liquid evidence without altering original receipts."""
import argparse
import gzip
import json
from pathlib import Path

import numpy as np
from scoring import BeerScore


def replay(directory, rescore=False):
    scorer=BeerScore()
    samples=0
    last=None
    changed=0
    score_changes=0
    with gzip.open(directory/'native-liquid.npy.gz','rb') as particles, (directory/'native-liquid.jsonl').open() as frames:
        for line in frames:
            frame=json.loads(line)
            points=np.load(particles,allow_pickle=False)
            if len(points)!=frame['particles']:raise ValueError('Native particle evidence count mismatch')
            last=scorer.update(points,frame['glass_position'],frame['glass_rotation'])
            differences=[key for key,value in last.items() if value!=frame[key]]
            if differences and not rescore:
                raise ValueError(f'Native replay mismatch at sample {samples+1}: {differences}; use --rescore for an intentional evaluator revision')
            changed+=bool(differences)
            score_changes+=last['score']!=frame['score']
            samples+=1
        if particles.peek(1):raise ValueError('Unmatched native particle frames')
    if not samples:raise ValueError('No native liquid samples')
    return dict(scope='Read-only replay of recorded native fluid',samples=samples,changed_samples=changed,changed_primary_scores=score_changes,final_metrics=last)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--rescore',action='store_true',help='Report intentional evaluator changes without altering original evidence')
    args=parser.parse_args()
    print(json.dumps(replay(args.recording,args.rescore),indent=2))
