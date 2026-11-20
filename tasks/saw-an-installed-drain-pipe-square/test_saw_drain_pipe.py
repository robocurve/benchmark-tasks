"""Offline checks (no simulator): the cut model, the scoring, the layout and the scripted demo's reach.

    python -m pytest -q test_saw_drain_pipe.py
"""

import importlib.util
import math
import pathlib

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
_spec = importlib.util.spec_from_file_location("saw_drain_pipe", HERE / "saw_drain_pipe.py")
T = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(T)
CM = T.CM

from yam_parts import PartState  # noqa: E402


def _quat_z(a):
    return np.array((0.0, 0.0, math.sin(a / 2), math.cos(a / 2)))


def saw_state(origin, quat=None):
    quat = _quat_z(math.pi / 2) if quat is None else np.asarray(quat, float)
    R = T._rotation(type("S", (), {"quat_xyzw": quat})())
    pts = np.array([np.asarray(origin) + R @ c for c in
                    [(-0.32, -0.03, -0.031), (0.125, 0.03, 0.025), (-0.32, 0.03, 0.025), (0.125, -0.03, -0.031)]])
    return PartState(pos=pts.mean(0), quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pts.min(0),
                     high=pts.max(0), origin=np.asarray(origin, float))


def objects(origin=None, quat=None):
    origin = T.SAW_CUT + np.array((0.0, 0.0, 0.08)) if origin is None else origin
    return {"saw": saw_state(origin, quat)}


def drive(feed=10.0, trigger=True, tilt_deg=0.0, x_off=0.0, seconds=8.0, dt=1 / 240, turn_after=None):
    """A level blade lowered onto the pipe's top and pushed down with a steady ``feed`` (N), resting on the model's
    pad wherever there is one (which then pushes back with the whole feed) and dropping at 50 mm/s where there is
    none; the edge runs from the shoe to the blade's tip across the pipe, its plane square to the pipe unless tilted
    about the vertical. Returns the time the cut went through, or None."""
    CM.MODEL.clear()
    h = T.PIPE_Z + T.PIPE_R + 0.001
    t = 0.0
    while t < seconds:
        a = math.radians(tilt_deg if (turn_after is None or t >= turn_after) else 0.0)
        n = np.array((math.cos(a), math.sin(a), 0.0))
        along = np.array((-math.sin(a), math.cos(a), 0.0))
        start = np.array((T.MARK_X + x_off, T.Y_O_CUT + 0.006, h))
        edge = (start, start + along * (T.BLADE_OUT - 0.006))
        plate = CM.MODEL.plate
        resting = plate is not None and h <= plate[0][2] + 1e-6
        CM.MODEL.step(edge, n, feed if resting else 0.0, trigger, dt)
        plate = CM.MODEL.plate
        h = max(h - 0.05 * dt, plate[0][2]) if plate is not None else h - 0.05 * dt
        t += dt
        if CM.MODEL.kerfs and CM.MODEL.kerfs[0].through():
            return t
    return None


# -- the cut model -----------------------------------------------------------------------------------------------------
def test_a_light_push_cuts_through_in_about_four_seconds():
    t = drive(feed=10.0)
    assert t is not None and 3.0 < t < 4.6
    k = CM.MODEL.main_kerf()
    assert abs(k.x() - T.MARK_X) < 1e-3 and k.squareness() < 1e-6 and k.round_share() == 1.0


def test_a_lighter_push_cuts_slower():
    t = drive(feed=6.0, seconds=15.0)
    assert t is not None and t > 7.0


def test_under_the_minimum_feed_it_only_scratches():
    assert drive(feed=2.5, seconds=6.0) is None
    k = CM.MODEL.main_kerf()
    assert k is None or k.round_share() < 0.1


def test_without_the_trigger_nothing_is_cut():
    assert drive(feed=10.0, trigger=False, seconds=4.0) is None
    assert not CM.MODEL.kerfs


def test_a_blade_twisted_in_its_kerf_binds():
    assert drive(feed=10.0, tilt_deg=6.0, turn_after=1.0, seconds=6.0) is None
    k = CM.MODEL.kerfs[0]
    assert k.round_share() < 0.6


def test_a_crooked_start_makes_a_crooked_kerf():
    drive(feed=10.0, tilt_deg=5.0)
    assert math.degrees(CM.MODEL.main_kerf().squareness()) == pytest.approx(5.0, abs=0.05)


# -- scoring -----------------------------------------------------------------------------------------------------------
def _cut(x=None, tilt_deg=0.0, through=True, share=None):
    CM.MODEL.clear()
    a = math.radians(tilt_deg)
    k = CM.Kerf(np.array((T.MARK_X if x is None else x, T.PIPE_Y, T.PIPE_Z + T.PIPE_R)),
                np.array((math.cos(a), math.sin(a), 0.0)))
    if through:
        for j, y in enumerate(CM._strips()):
            top_i, bottom_i = CM._walls(y)
            if top_i is not None:
                k.floor[j] = (bottom_i or top_i)[0] - CM.THROUGH
    elif share is not None:  # down to a height: the floor level across
        for j, y in enumerate(CM._strips()):
            top_i, _ = CM._walls(y)
            if top_i is not None:
                k.floor[j] = min(k.floor[j], share)
    CM.MODEL.kerfs.append(k)
    return k


@pytest.fixture(autouse=True)
def _released():
    T.TEST_TRIGGER = False
    T.TEST_PEX = False
    yield
    T.TEST_TRIGGER = None
    T.TEST_PEX = None
    CM.MODEL.clear()


def test_nothing_cut_scores_zero():
    CM.MODEL.clear()
    assert T.score(objects()) == 0.0


def test_through_square_on_the_mark_released_and_lifted_is_full_score():
    _cut()
    assert T.score(objects()) == pytest.approx(1.0)
    assert T.success(objects())


def test_through_but_the_trigger_still_pulled():
    _cut()
    T.TEST_TRIGGER = True
    assert T.score(objects()) == pytest.approx(0.8)


def test_through_but_the_blade_left_in_the_cut():
    _cut()
    assert T.score(objects(T.SAW_CUT - np.array((0.0, 0.0, 0.03)))) == pytest.approx(0.8)


def test_through_but_the_saw_put_down_on_the_table():
    _cut()
    low = T.SAW_CUT.copy()
    low[2] = T.TOP + 0.031  # its teeth on the table
    low[1] = -0.2  # off to the side of the pipe
    assert T.score(objects(low)) == pytest.approx(0.8)


def test_through_but_the_blade_touched_the_water_line_scores_zero():
    _cut()
    T.TEST_PEX = True
    assert T.score(objects()) == 0.0
    T.TEST_PEX = None  # on the simulator: the largest push of the blade on it, from contacts
    T.CR.STATE["pex"] = 0.5
    assert T.score(objects()) == 0.0
    T.CR.STATE["pex"] = 0.0
    assert T.score(objects()) == pytest.approx(1.0)


def test_half_way_round_is_partial_credit():
    _cut(through=False, share=0.0)  # cut down to the pipe's axis
    s = T.score(objects())
    assert 0.3 < s < 0.5


def test_off_the_mark_scores_zero():
    _cut(x=T.MARK_X + 0.004)
    assert T.score(objects()) == 0.0
    _cut(x=T.MARK_X - 0.0025)
    assert T.score(objects()) == pytest.approx(1.0)


def test_out_of_square_scores_zero():
    _cut(tilt_deg=2.5)
    assert T.score(objects()) == 0.0
    _cut(tilt_deg=1.5)
    assert T.score(objects()) == pytest.approx(1.0)


def test_a_second_cut_scores_zero():
    _cut()
    extra = CM.Kerf(np.array((T.MARK_X + 0.02, T.PIPE_Y, T.PIPE_Z + T.PIPE_R)), np.array((1.0, 0.0, 0.0)))
    extra.floor -= 0.002
    CM.MODEL.kerfs.append(extra)
    assert T.score(objects()) == 0.0


def test_a_broken_state_scores_zero():
    _cut()
    bad = objects()
    bad["saw"].quat_xyzw[:] = (0.0, 0.0, 0.0, 2.0)
    assert T.score(bad) == 0.0


# -- the layout ------------------------------------------------------------------------------------------------------
def _aabb(origin, shapes, R):
    pts = []
    for sh in shapes:
        lo, hi = T.yam_parts._shape_bounds(sh)
        for c in T.yam_parts._corners(lo, hi):
            pts.append(np.asarray(origin) + R @ c)
    pts = np.array(pts)
    return pts.min(0), pts.max(0)


def _boxes_overlap(a, b):
    return bool(np.all(a[0] < b[1]) and np.all(b[0] < a[1]))


def test_the_saw_over_the_pipe_is_clear_of_its_rest():
    saw = _aabb(T.SAW_CUT, T.SAW_SHAPES, T.YAW)
    for post in [o for o in T.OBJECTS if o.name.startswith("rest_")]:
        lo, hi = np.subtract(post.pos, np.divide(post.size, 2)), np.add(post.pos, np.divide(post.size, 2))
        assert not _boxes_overlap(saw, (lo, hi))


def test_the_blade_spans_the_pipe_with_the_shoe_beside_it():
    a, b = (T.SAW_CUT + T.YAW @ e for e in T.CR.EDGE_LOCAL)
    assert a[1] < T.PIPE_Y - T.PIPE_R and b[1] > T.PIPE_Y + T.PIPE_R + 0.03
    shoe_front = T.SAW_CUT[1] + T.SHOE_FRONT
    assert T.PIPE_Y - T.PIPE_R - shoe_front == pytest.approx(0.003)


def test_the_water_line_runs_below_the_drain_under_the_blade_through_the_blocks():
    assert (T.PIPE_Z - T.PIPE_R) - (T.PEX_Z + T.PEX_R) == pytest.approx(1.25 * 0.0254)  # 32 mm
    assert T.PEX_LINE.pos[1] == T.PEX_Y
    tip = (T.SAW_CUT + T.YAW @ T.CR.EDGE_LOCAL[1])[1]
    assert T.PIPE_Y + T.PIPE_R < T.PEX_Y - T.PEX_R  # beside the drain
    assert T.PEX_Y + T.PEX_R < tip - 0.015  # and under the blade
    assert T.TOP < T.PEX_Z - T.PEX_R and T.PEX_Z + T.PEX_R < T.TOP + T.BLOCK[2]  # through the blocks' bored holes
    assert abs(T.PEX_Y - T.PIPE_Y) + T.PEX_R < T.BLOCK[1] / 2
    # the demo catches the saw just under the drain, well above the water line
    assert (T.PIPE_Z - T.PIPE_R - 0.004) - (T.PEX_Z + T.PEX_R) > 0.02


def test_the_saw_on_its_rest_is_clear_of_the_pipe():
    lo, hi = _aabb(T.SAW_REST, T.SAW_SHAPES, T.YAW)
    assert hi[1] < T.PIPE_Y - T.PIPE_R


# -- the demo --------------------------------------------------------------------------------------------------------
def test_the_demo_is_within_reach_and_the_run_guards():
    from yam_arena import station
    from yam_arena.demo import ArmPlan

    limits = np.array([0.288, 0.183, 0.157, 0.163, 0.157, 0.209])
    for arm, wps in T.demo().items():
        plan = ArmPlan(wps, station.BASES[arm])
        prev = None
        for k in range(int(plan.duration / 0.05) + 1):
            a, _ = plan.action7(k * 0.05)
            if prev is not None:
                assert np.all(np.abs(a[:6] - prev[:6]) < 0.95 * limits)
                assert abs(a[6] - prev[6]) < 0.95 * 0.05 + 1e-9
            prev = a
        assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 1.0
