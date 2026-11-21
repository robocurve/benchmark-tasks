"""Verify a controller on both fixed spare assignments and report its paired score."""
import argparse
import json
from pathlib import Path

from verify import load, verify


def verify_pair(first, second, *, rescore=False):
    runs = (first.resolve(), second.resolve())
    configs = [load(run / 'configuration.json') for run in runs]
    assignments = [config['stock_order'] for config in configs]
    if set(assignments) != {'intact-first', 'pierced-first'}:
        raise ValueError('Two distinct spare assignments are required.')
    if configs[0]['mode'] not in ('demo', 'dream') or configs[0]['mode'] != configs[1]['mode']:
        raise ValueError('Both episodes must evaluate the same controller.')
    if configs[0]['canonical_instruction'] != configs[1]['canonical_instruction']:
        raise ValueError('Both episodes require the same policy instruction.')
    manifests = [load(run / 'source-manifest.json') for run in runs]
    for name in set(manifests[0]) | set(manifests[1]):
        if name in ('verify.py', 'verify_pair.py', 'media.py'):
            continue
        if name.endswith(('.py','.npz','.json')) and manifests[0].get(name) != manifests[1].get(name):
            raise ValueError(f'Episode sources differ: {name}')
    scores = {assignment: verify(run, rescore=rescore)['score'] for assignment, run in zip(assignments, runs)}
    return dict(mode=configs[0]['mode'], episode_scores=scores, mean_score=sum(scores.values())/2)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('first', type=Path)
    parser.add_argument('second', type=Path)
    parser.add_argument('--rescore', action='store_true', help='Apply the current rubric to both earlier same-scene recordings.')
    args = parser.parse_args()
    print(json.dumps(verify_pair(args.first,args.second,rescore=args.rescore),indent=2))
