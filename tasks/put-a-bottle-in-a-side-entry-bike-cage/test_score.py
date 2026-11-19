"""Unit tests of the scoring in task.py: every band, each shortcut, and which criteria are judged at the end and which at
any time. Plain Python (no Isaac Sim): run with  python -m pytest test_score.py  from this folder (the yam_arena package
must be importable for task.py's imports). Created 11_16_26 (America/New_York)."""

import math

import numpy as np
import pytest

import task as T
from bike_cage import geom as G
from yam_arena.task_file import ObjectState

DT = 0.05
STILL = np.array((0.0, 0.0, 0.0, 1.0))


def _quat_from_rot(r):
    from bike_cage.mech import quat_wxyz

    w, x, y, z = quat_wxyz(r)
    return np.array((x, y, z, w))


def bottle_at(s=0.0, n=0.0, a=0.0, tilt_deg=0.0, v=0.0):
    """Bottle with its base's middle at cage point (s, n, a), its axis tipped tilt_deg toward the near side (+s)."""
    pos = T.cage_world(s, n, a)
    c, sn = math.cos(math.radians(tilt_deg)), math.sin(math.radians(tilt_deg))
    ax_c = np.array((sn, 0.0, c))  # in cage axes
    e1_c = np.array((c, 0.0, -sn))
    r_c = np.column_stack((e1_c, np.cross(ax_c, e1_c), ax_c))
    r = T.RC @ r_c
    return ObjectState(pos=pos, quat_xyzw=_quat_from_rot(r), lin_vel=np.array((v, 0.0, 0.0)), ang_vel=np.zeros(3), low=pos, high=pos)


def bottle_on_table(v=0.0):
    p = np.array((T.BOTTLE_START[0], T.BOTTLE_START[1], T.TOP))
    return ObjectState(pos=p, quat_xyzw=STILL, lin_vel=np.array((v, 0.0, 0.0)), ang_vel=np.zeros(3), low=p, high=p)


def point(p):
    p = np.asarray(p, float)
    return ObjectState(pos=p, quat_xyzw=STILL, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)


def state(bottle, arm=0.0, hand=None):
    far = np.array((0.15, -0.36, T.TOP + 0.25))
    right = far if hand is None else np.asarray(hand, float)
    a = np.array((arm, 0.0, 0.0))
    return {"bottle": bottle, "cage_arm": point(a), "left_tcp": point((0.12, 0.34, T.TOP + 0.18)), "right_tcp": point(right)}


def run(states):
    """Feed states one control step each (as the embodiment does) and return the score of the last."""
    T.reset_latch()
    for st in states:
        T.note_step(st, DT)
    return T.score(states[-1])


def hold(st, seconds):
    return [st] * int(round(seconds / DT))


HELD = T.held_tcp(T.cage_world(0.0, 0.0, 0.0), T.RC[:, 2], T.R_SEAT)  # the hand still holding the seated bottle


# -- bands ------------------------------------------------------------------------------------------------------------
def test_full_score_seated_enclosed_released():
    assert run(hold(state(bottle_at()), 4.0)) == 1.0


def test_success_matches_full_score():
    st = state(bottle_at())
    run(hold(st, 4.0))
    assert T.success(st)


def test_nothing_done_scores_zero():
    assert run(hold(state(bottle_on_table()), 30.0)) == 0.0


def test_enclosed_not_seated_half():
    # base 14 mm up the cage (sitting on the near lip), still inside the arm's ring
    assert run(hold(state(bottle_at(a=0.014)), 4.0)) == 0.5


def test_enclosed_tilted_half():
    # the axis tips 12 degrees but still passes within 15 mm of the cage's axis at the band (base shifted to match)
    a = T.BAND_A
    off = a * math.tan(math.radians(12.0))
    assert run(hold(state(bottle_at(s=-off, tilt_deg=12.0)), 4.0)) == 0.5


def test_seated_but_still_held_quarter():
    assert run(hold(state(bottle_at(), hand=HELD), 10.0)) == 0.25


def test_resting_against_the_cage_quarter():
    # pushed only part way: base on the floor's near edge, the arm's tip open against the bottle's side
    assert run(hold(state(bottle_at(s=0.045), arm=14.0), 4.0)) == 0.25


def test_dropped_scores_zero():
    p = T.cage_world(0.10, 0.0, -0.30)
    b = ObjectState(pos=p, quat_xyzw=STILL, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p)
    assert run(hold(state(b), 4.0)) == 0.0


def test_arm_held_open_not_enclosed():
    # the bottle sits in the cage but the arm is propped open (by the hand or wedged): not enclosed
    st = state(bottle_at(), arm=10.0)
    assert not T.enclosed(st)
    assert run(hold(st, 4.0)) == 0.25


def test_far_off_axis_not_enclosed():
    st = state(bottle_at(s=0.030))
    assert not T.enclosed(st)


# -- hands clear for the last 3 s -------------------------------------------------------------------------------------
def test_needs_three_seconds_clear():
    assert run(hold(state(bottle_at()), 2.5)) == 0.25


def test_clear_time_restarts_after_touching():
    st, held = state(bottle_at()), state(bottle_at(), hand=HELD)
    assert run(hold(st, 5.0) + hold(held, 0.5) + hold(st, 2.0)) == 0.25
    assert run(hold(st, 5.0) + hold(held, 0.5) + hold(st, 3.0)) == 1.0


def test_bottle_moving_at_end_not_retained():
    assert run(hold(state(bottle_at(v=0.05)), 4.0)) == 0.25


# -- over-bending the arm: judged at any time --------------------------------------------------------------------------
def test_overbent_caps_at_quarter_even_if_later_seated():
    st = state(bottle_at())
    bent = state(bottle_at(s=0.03), arm=G.OVERBENT_DEG + 2.0)
    assert run(hold(bent, 0.2) + hold(st, 4.0)) == 0.25


def test_arm_opened_within_its_flex_no_cap():
    st = state(bottle_at())
    flexed = state(bottle_at(s=0.03), arm=G.OVERBENT_DEG - 2.0)
    assert run(hold(flexed, 0.5) + hold(st, 4.0)) == 1.0


# -- end-state criteria are judged at the end only ----------------------------------------------------------------------
def test_transient_unseat_that_settles_scores_full():
    # a bump lifts the base 15 mm part way through; the end state is seated and released
    st, bumped = state(bottle_at()), state(bottle_at(a=0.015, v=0.2))
    assert run(hold(st, 2.0) + hold(bumped, 0.2) + hold(st, 4.0)) == 1.0


def test_seated_earlier_but_fallen_at_end_scores_zero():
    st = state(bottle_at())
    p = T.cage_world(0.10, 0.0, -0.30)
    fallen = state(ObjectState(pos=p, quat_xyzw=STILL, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p, high=p))
    assert run(hold(st, 5.0) + hold(fallen, 4.0)) == 0.0


# -- thresholds ---------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("da,expect", [(0.009, 1.0), (0.011, 0.5)])
def test_seat_threshold(da, expect):
    assert run(hold(state(bottle_at(a=da)), 4.0)) == expect


def test_hand_clear_distance():
    st = state(bottle_at())
    b = T.cage_world(0.0, 0.0, 0.09)
    away = b + T.RC @ np.array((T.CLEAR_R + 0.005, 0.0, 0.0))
    near = b + T.RC @ np.array((T.CLEAR_R - 0.005, 0.0, 0.0))
    assert T.hand_clear(state(st["bottle"], hand=away), "right_tcp")
    assert not T.hand_clear(state(st["bottle"], hand=near), "right_tcp")


def test_geometry_forces_side_entry():
    # straight up the cage there is not room to lift the bottle's base past the front band's top
    assert G.headroom_along_axis(G.BOTTLE_R) < G.ARM_A[1]
    assert G.tip_force(15.0) < 5.0
