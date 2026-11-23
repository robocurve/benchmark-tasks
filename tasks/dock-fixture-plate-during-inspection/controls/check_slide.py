"""Check a camera-annotated dry cycle. CSV columns: frame,input_mm (0..600 at 20 Hz)."""

import argparse
import csv
import math
from pathlib import Path


def check(rows, profile):
    points = {int(row["frame"]): float(row["input_mm"]) for row in rows}
    if (
        len(rows) != 601
        or set(points) != set(range(601))
        or not all(map(math.isfinite, points.values()))
    ):
        raise ValueError(
            "Supply each frame 0..600 exactly once with a finite ruler reading"
        )
    p = [points[i] for i in range(601)]
    if profile == "continuing":
        if max(map(abs, p)) > 2:
            raise ValueError("Locked input moved beyond 2 mm")
        return
    start = 0 if profile == "early" else 170
    if (
        max(map(abs, p[: start + 1])) > 2
        or max(abs(x - 200) for x in p[start + 160 :]) > 2
    ):
        raise ValueError("Start or end stop out of tolerance")
    farthest = 0.0
    for i, x in enumerate(p):
        u = min(1.0, max(0.0, (i - start) / 160))
        target = 200 * u * u * (3 - 2 * u)
        if abs(x - target) > 5 or x < farthest - 2:
            raise ValueError(f"Frame {i}: input {x:g} mm, target {target:.1f} mm")
        farthest = max(farthest, x)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("profile", choices=("early", "nominal", "continuing"))
    parser.add_argument("csv", type=Path)
    args = parser.parse_args()
    with args.csv.open(newline="") as f:
        check(list(csv.DictReader(f)), args.profile)
    print(
        "Input trajectory passed. Inspect cue timing and wrist/limb footage separately."
    )
