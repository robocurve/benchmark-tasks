"""The scoring table, the inside test and the front-gap rule, on hand-built values, with no simulator and no GPU.
Run with the harness's Python, from this folder: python check_score.py"""

from __future__ import annotations

import sys

import numpy as np

import task as t


def main() -> int:
    bad = 0

    def check(name, ok, got=""):
        nonlocal bad
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}{': ' + str(got) if got != '' else ''}")

    for name, args, want in (  # judge(lid_open, food_in, caught, flat, released, dead, best)
        ("latched, food in, flat, released and still 2 s", (0.3, True, True, True, True, False, 0.7), 1.0),
        ("latched, but a hand still on the box", (0.3, True, True, True, False, False, 0.7), 0.7),
        ("latched, but a leaf across the seam", (0.3, False, True, True, True, False, 0.3), 0.3),
        ("latched, but the box tipped earlier", (0.3, True, True, True, True, True, 0.7), 0.0),
        ("not latched, lid sprung back open, after a clean shut", (25.0, True, False, True, True, False, 0.7), 0.7),
        ("not latched, lid sprung back open, after a shut over the leaf", (25.0, True, False, True, True, False, 0.3), 0.3),
        ("nothing reached", (73.0, False, False, True, True, False, 0.0), 0.0),
    ):
        got = t.judge(*args)
        check(name, got == want, got)

    for name, args, want in (  # reached(lid_open, food_in)
        ("lid shut (2 degrees) with all food in", (2.0, True), 0.7),
        ("lid 3.5 degrees: front 9.6 mm up, all food in", (3.5, True), 0.7),
        ("lid 3.5 degrees over the leaf", (3.5, False), 0.3),
        ("lid 4 degrees: front 11 mm up", (4.0, True), 0.0),
        ("lid 14 degrees with all food in", (14.0, True), 0.0),
        ("lid shut over the leaf", (1.0, False), 0.3),
    ):
        got = t.reached(*args)
        check(name, got == want, got)

    check("front gap at 3.6 degrees is under 10 mm", t.front_gap(3.6) <= 0.010, round(t.front_gap(3.6) * 1000, 1))
    leaf_out = t.LEAF_PTS["tip"]
    check("the leaf tip at reset crosses the rim", not t.inside(leaf_out), round(float(leaf_out[:, 0].min()) * 1000, 1))
    check("the leaf base at reset is inside", t.inside(t.LEAF_PTS["frame"]))
    check("every food piece at reset is inside", all(t.inside(p) for p in t.FOOD_PTS.values()), [n for n, p in t.FOOD_PTS.items() if not t.inside(p)])
    p = t.FOOD_PTS["tomato0"] + np.array([0.0, 0.0, 0.0])
    check("a slice lying on the front flange is not inside", not t.inside(p + np.array([t.FX0 + 0.01 - p[:, 0].mean(), 0.0, t.TZ - p[:, 2].min()])))
    check("a slice on the counter beside the box is not inside", not t.inside(p + np.array([0.0, 0.2, -p[:, 2].min()])))

    trial = t.Trial()
    trial.reset()
    check("a fresh trial scores 0", trial.score == 0.0 and not trial.full)
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
