"""The scorer on hand-built states, with no simulator and no GPU. Run it with the harness's Python, from this folder: python check_scorer.py

It checks the scoring table line by line, then works out, for a rod of each kind, how long ten cycles take with the bob at each place, from the compound pendulum's formula, to show that one fixed place cannot suit both rods.
"""

from __future__ import annotations

import math
import sys

import task as t


def cycle_s(rod_mass: float, s: float) -> float:
    """A uniform rod hanging from its end with a bob `s` below the pivot: 2 pi sqrt(I / (M g d))."""
    inertia = rod_mass * t.ROD_L**2 / 3 + t.BOB_MASS * s**2
    moment = rod_mass * t.ROD_L / 2 + t.BOB_MASS * s
    return 2 * math.pi * math.sqrt(inertia / (moment * 9.81))


def main() -> int:
    bad = 0

    def check(name: str, got, want) -> None:
        nonlocal bad
        ok = got == want
        bad += not ok
        print(f"{'ok ' if ok else 'BAD'} {name}: {got}")

    print("The table: (lost, ten cycles' seconds, swinging, free)")
    check("10.00 s", t.task_score(False, 10.0, True, True), (1.0, True))
    check("10.09 s", t.task_score(False, 10.09, True, True), (1.0, True))
    check("9.91 s", t.task_score(False, 9.91, True, True), (1.0, True))
    check("10.11 s", t.task_score(False, 10.11, True, True), (0.5, False))
    check("10.99 s", t.task_score(False, 10.99, True, True), (0.5, False))
    check("11.01 s", t.task_score(False, 11.01, True, True), (0.0, False))
    check("8.97 s, the bob as it starts", t.task_score(False, 8.97, True, True), (0.0, False))
    check("fewer than ten free cycles", t.task_score(False, None, True, True), (0.0, False))
    check("not swinging at the end", t.task_score(False, 10.0, False, True), (0.0, False))
    check("a finger near it now", t.task_score(False, 10.0, True, False), (0.0, False))
    check("the bob at the rod's end", t.task_score(True, 10.0, True, True), (0.0, False))

    print("One fixed place, by the formula")
    wins = {}
    for kind, mass in t.ROD_MASS.items():
        places = [s / 1000 for s in range(120, 281) if abs(10 * cycle_s(mass, s / 1000) - 10) <= t.FULL_WITHIN]
        wins[kind] = (min(places), max(places))
        print(f"    {kind}: a full score needs the bob from {min(places) * 100:.1f} to {max(places) * 100:.1f} cm below the pivot")
    check("the two windows do not overlap", wins["wood"][1] < wins["steel"][0], True)
    start = {kind: round(10 * cycle_s(mass, t.BOB_START), 2) for kind, mass in t.ROD_MASS.items()}
    print(f"    ten cycles with the bob as it starts: {start}")
    check("the start scores 0 on both rods", all(abs(v - 10) > t.PART_WITHIN for v in start.values()), True)
    print("All agree" if not bad else f"{bad} DISAGREE")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
