"""Offline checks of the scoring rule and the hanging geometry (no simulator): python -m pytest test_scoring.py"""

from __future__ import annotations

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import ornkit as K  # noqa: E402
import task as T  # noqa: E402

UPRIGHT = (0.0, 0.0, 0.0, 1.0)


@pytest.fixture(autouse=True)
def fresh():
    T._clear()
    yield
    T._clear()


def test_nothing_done_scores_zero():
    assert T.score({}) == 0.0


def test_half_while_not_both_hung_for_two_seconds():
    T._STATE[T.NAMES[0]]["hung"] = True
    assert T.score({}) == 0.5
    T._STATE[T.NAMES[1]]["hung"] = True
    assert T.score({}) == 0.5 and not T.success({})


def test_both_held_for_two_seconds_is_full_marks():
    T._STATE["done"] = True
    assert T.score({}) == 1.0
    assert T.success({})


def test_broken_glass_zeroes_the_trial():
    T._STATE.update(done=True, broken=True)
    assert T.score({}) == 0.0


@pytest.mark.parametrize("branch", ["left", "right"])
def test_a_bauble_at_its_hanging_pose_is_on_its_branch(branch):
    assert T.on_branch(T.hang_point(branch), UPRIGHT) == branch


def test_a_bauble_beside_or_below_a_branch_is_not_on_it():
    p = T.hang_point("left")
    assert T.on_branch(p + [0.0, 0.0, -0.02], UPRIGHT) is None      # below the branch: the crook is not round it
    assert T.on_branch(p + [0.012, 0.0, 0.0], UPRIGHT) is None      # beside it: the branch is outside the crook
    assert T.on_branch(p + [0.0, 0.13, 0.0], UPRIGHT) is None       # off the branch's end
    turned = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert T.on_branch(p, turned) is None                            # crook along the branch, not across it


def test_baubles_start_in_their_boxes_off_the_stand():
    for i in range(len(T.NAMES)):
        p = T.bauble_start(i)
        assert T.in_box(p) and T.on_branch(p, UPRIGHT) is None


def test_geometry():
    assert K.CROOK_R * 2 - 2 * K.WIRE_R > T.BRANCH_D                  # the crook fits round a branch
    hung = T.hang_point("left")
    assert hung[2] - K.BALL_R > T.TOP + 0.015                        # a hanging bauble clears the table
    root, tip, _ = T.branch_axis("left")
    assert root[1] < hung[1] < tip[1]                                # it hangs within the branch's length
