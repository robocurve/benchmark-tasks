"""Scoring rules (paint_score.py), without the simulator: python -m pytest tests/ from the task folder."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from paint_score import flake_off, removed, stripped, tier  # noqa: E402

ALL = [[True] * 6 for _ in range(3)]
NONE = [[False] * 6 for _ in range(3)]


def test_a_flake_still_stuck_is_not_off():
    assert not flake_off(0.0005, 1.0)  # a stuck flake jiggles well under 1.5 mm and 8 degrees
    assert flake_off(0.004, 0.0)  # pushed along the rail
    assert flake_off(0.0, 30.0)  # curled up


def test_hovering_takes_nothing_off():
    # a blade held above the paint touches no flake: none moves, nothing is removed, the score is 0
    assert removed([False] * 6) == 0.0
    assert tier(NONE, False, True, True, True) == 0.0


def test_patch_stripped_from_area_removed():
    assert stripped([True] * 5 + [False])  # 5 of 6 flakes: 83 percent bare wood
    assert not stripped([True] * 4 + [False] * 2)  # 67 percent


def test_tiers():
    assert tier(ALL, False, True, True, True) == 1.0
    assert tier(ALL, False, False, True, True) == 0.5  # heat gun not back on its stand
    assert tier(ALL, False, True, True, False) == 0.5  # arms not home
    assert tier([[True] * 6, [False] * 6, [False] * 6], False, True, True, True) == 0.25
    assert tier(ALL, True, True, True, True) == 0.0  # cracked glass
