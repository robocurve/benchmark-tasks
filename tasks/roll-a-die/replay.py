"""Recompute a die-roll score from the retained motion audit."""

import argparse
import gzip
import json
from pathlib import Path

from scoring import RollTracker


def replay(path):
    tracker = RollTracker()
    count = 0
    path = Path(path)
    content = gzip.decompress(path.read_bytes()).decode() if path.suffix == ".gz" else path.read_text()
    for line in content.splitlines():
        if line.strip():
            tracker.update(json.loads(line))
            count += 1
    if not count:
        raise ValueError("empty trajectory")
    return tracker.summary()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory")
    args = parser.parse_args()
    print(json.dumps(replay(args.trajectory), indent=2))
