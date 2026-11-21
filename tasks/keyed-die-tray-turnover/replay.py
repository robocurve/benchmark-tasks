"""Recompute every score from a native trace, independently of the policy and RGB."""
import argparse
import json
from pathlib import Path
import numpy as np
from grading import Grader


def replay(source):
    with np.load(source,allow_pickle=False) as f:
        grader=Grader(list(f['names']),list(f['filters']))
        scores=np.array([grader.update(p,c) for p,c in zip(f['poses'],f['counts'],strict=True)])
        if 'scores' in f:
            np.testing.assert_array_equal(scores,f['scores'])
    full=np.flatnonzero(scores==1.)
    return dict(actions=len(scores)-1,final=float(scores[-1]),first_full=int(full[0]) if len(full) else None,
                full_windows=len(full),ending_credit=grader.good_cells)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path)
    print(json.dumps(replay(parser.parse_args().source),indent=2))
