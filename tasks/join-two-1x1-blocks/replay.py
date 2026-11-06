"""Rescore a measured trajectory without a GPU."""
import argparse
import gzip
import json
from pathlib import Path
from config import CONFIG
from scoring import score_samples

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trajectory', type=Path)
    args = parser.parse_args()
    opener = gzip.open if args.trajectory.suffix == '.gz' else open
    with opener(args.trajectory, 'rt') as stream:
        print(json.dumps(score_samples((json.loads(line) for line in stream if line.strip()), CONFIG), indent=2))
