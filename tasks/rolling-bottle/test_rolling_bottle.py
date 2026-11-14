"""Scoring and demo checks for rolling-bottle that run without the simulator: python -m pytest -q test_rolling_bottle.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

import rolling_bottle as M

HERE = Path(__file__).resolve().parent
LYING_Z = M.WORKTOP_TOP + M.BOTTLE_R  # the bottle's axis, lying on the worktop


STANDING = np.array([0.0, -math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])  # its axis (local x) turned up to z


def bottle(y, z=LYING_Z, speed=0.0, x=None, quat=None) -> ObjectState:
    """As the simulator reports it: the box's centre on the body's axis; the box's bottom about 1 cm low."""
    p = np.array([M.BOTTLE_START[0] if x is None else x, y, z])
    half = np.array([0.12, M.BOTTLE_R, M.BOTTLE_R + 0.01])
    q = np.array([0, 0, 0, 1.0]) if quat is None else quat
    return ObjectState(pos=p, quat_xyzw=q, lin_vel=np.array([0, -speed, 0]),
                       ang_vel=np.array([speed / M.BOTTLE_R, 0, 0]), low=p - half, high=p + half)


def standing(y, on=None) -> ObjectState:
    """Stood up, neck up: the box's centre half the bottle's length over the surface."""
    surface = M.WORKTOP_TOP if on is None else on
    return bottle(y, z=surface + M.SPAN_HALF, quat=STANDING)


def run(*states) -> float:
    """Feed a trial's states in order through a fresh reset; return the last score."""
    M._new_trial()
    s = 0.0
    for b in states:
        s = M.score({"bottle": b})
    return s


def test_the_task_file_loads():
    task = load(HERE / "rolling_bottle.py")
    assert set(task.free_objects) == {"drainboard", "bottle"}


def test_the_bottle_starts_on_the_drainboard_rolling_and_scores_0():
    start = bottle(M.BOTTLE_START[1], z=M.drain_top(M.BOTTLE_START[1]) + M.BOTTLE_R, speed=0.15)
    assert run(start) == 0.0


def test_stopped_on_the_worktop_succeeds_and_scores_0_9_until_held():
    b = bottle(-0.154)
    M._new_trial()
    assert M.success({"bottle": b}) and M.score({"bottle": b}) == 0.9


def test_still_rolling_on_the_worktop_scores_0():
    assert run(bottle(-0.05, speed=0.3)) == 0.0


def test_over_the_edge_scores_0_and_stays_0_even_if_put_back():
    fallen = bottle(-0.30, z=M.TOP + M.BOTTLE_R)
    assert run(bottle(-0.10, speed=0.3), fallen, bottle(-0.10)) == 0.0


def test_a_reset_clears_the_record():
    run(bottle(-0.30, z=M.TOP + M.BOTTLE_R))
    assert run(bottle(-0.10)) == 0.9


def test_held_up_off_the_worktop_does_not_count():
    assert run(bottle(-0.10, z=LYING_Z + 0.02)) == 0.0


def test_caught_still_on_the_drainboard_counts():
    assert run(bottle(0.25, z=M.drain_top(0.25) + M.BOTTLE_R)) == 0.9


def test_stood_up_on_the_worktop_counts():
    assert M.lowest_point(standing(-0.10)) == pytest.approx(M.WORKTOP_TOP, abs=1e-6)
    assert run(standing(-0.10)) == 0.9


def test_stood_up_on_the_drainboard_counts():
    assert run(standing(0.25, on=M.drain_top(0.25))) == 0.9


def test_held_up_standing_does_not_count():
    assert run(standing(-0.10, on=M.WORKTOP_TOP + 0.02)) == 0.0


def test_fallen_and_landed_standing_on_the_table_then_put_back_scores_0():
    landed = standing(-0.30, on=M.TOP)
    assert run(bottle(-0.20, speed=0.3), landed, bottle(-0.10)) == 0.0


def test_hanging_past_the_front_or_back_edge_does_not_count():
    assert run(bottle(-0.10, x=M.WORKTOP_XY[0] - M.WORKTOP[0] / 2000 - 0.01)) == 0.0


def test_the_span_matches_the_bottle():
    neck_tip = M.BODY_LEN / 2 + M.BOTTLE_R + M.NECK_LEN - 0.004 + M.NECK_R
    tail = M.BODY_LEN / 2 + M.BOTTLE_R
    assert M.SPAN_HALF == pytest.approx((neck_tip + tail) / 2, abs=1e-4)


def test_the_bounds_sitting_low_do_not_read_as_fallen():
    b = bottle(-0.10)
    assert float(b.low[2]) < M.WORKTOP_TOP - 0.009  # the reported box bottom sits about 1 cm low
    assert run(b) == 0.9


# -- the scene ------------------------------------------------------------------------------------------------------

def test_the_drainboard_slopes_gently_down_to_the_worktop():
    assert 0.5 <= M.DRAIN_DEG <= 2.0
    assert M.drain_top(M.DRAIN_LOW_Y) == pytest.approx(M.WORKTOP_TOP + M.DRAIN_T / 1000, abs=1e-4)
    assert M.drain_top(M.DRAIN_HIGH_Y) > M.drain_top(M.DRAIN_LOW_Y)


def test_the_bottle_is_a_real_330_ml_bottle():
    assert 2 * M.BOTTLE_R == pytest.approx(0.061) and M.BOTTLE_KG == pytest.approx(0.53)
    length = M.BODY_LEN + 2 * M.BOTTLE_R + M.NECK_LEN
    assert 0.21 < length < 0.25


def test_the_open_end_is_within_the_right_arms_reach():
    assert -0.26 < M.OPEN_END_Y < -0.18


# -- the demo -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_fingers_stand_in_the_bottles_path_on_the_worktop_before_the_edge():
    x, y, z = M.block_tcp()
    assert M.OPEN_END_Y + 0.03 < y < M.DRAIN_LOW_Y
    assert z - 0.013 == pytest.approx(M.WORKTOP_TOP + M.TIPS_ABOVE)
    body = M.BODY_LEN / 2 + M.BOTTLE_R
    assert M.BOTTLE_START[0] - body < x < M.BOTTLE_START[0] + body


def test_the_fingers_arrive_quickly():
    right = M.demo()["right"]
    assert right[0].duration + right[1].duration <= 1.7


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
