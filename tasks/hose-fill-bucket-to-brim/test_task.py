"""Offline checks, no simulator: the water model and every scoring outcome.

    python -m pytest -q tasks/hose-fill-bucket-to-brim/test_task.py
"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import task as T  # noqa: E402
import water as w  # noqa: E402
from parts import BUCKET, FLOOR_Z, GROUND  # noqa: E402

UP = np.eye(3)
FLOOR = np.array([BUCKET.pos[0], BUCKET.pos[1], BUCKET.pos[2] + FLOOR_Z])


# -- the pail and the valve ----------------------------------------------------------------------------------------


def test_pail_volume():
    assert w.BRIM_L == pytest.approx(6.30, abs=0.01)
    assert w.volume_at(0.0) == 0.0
    for litres in (0.5, 2.0, 4.0, 6.0):
        assert w.volume_at(w.level_at(litres)) == pytest.approx(litres, abs=0.005)
    full_from = w.volume_at(w.DEPTH - T.FULL_GAP)  # the "full" band: the top 15 mm, about 0.56 L
    assert 5.6 < full_from < 5.8


def test_valve():
    assert w.flow(0.0) == 0.0 and w.flow(w.VALVE_OPENS_DEG) == 0.0
    assert w.flow(13.0) == pytest.approx(w.FLOW_MAX / 2)
    assert w.flow(22.0) == w.FLOW_MAX == pytest.approx(0.2)
    assert w.exit_speed(w.FLOW_MAX) == pytest.approx(2.55, abs=0.01)


# -- where the stream lands ----------------------------------------------------------------------------------------


def _trace(outlet, direction, level=0.0, speed=2.5):
    return w.trace(outlet, direction, speed, FLOOR, UP, level, GROUND)


def test_stream_straight_down_into_the_pail():
    top = FLOOR + np.array([0.0, 0.0, w.DEPTH + 0.06])
    hit = _trace(top, (0, 0, -1))
    assert hit.into_bucket and hit.what == "water"
    assert hit.point[2] == pytest.approx(FLOOR[2], abs=0.01)
    hit = _trace(top, (0, 0, -1), level=0.10)
    assert hit.into_bucket and hit.point[2] == pytest.approx(FLOOR[2] + 0.10, abs=0.01)


def test_stream_aimed_across_strikes_the_inner_wall():
    start = FLOOR + np.array([-0.06, 0.0, w.DEPTH + 0.06])
    hit = _trace(start, (1, 0, -0.6), speed=2.5)
    assert hit.into_bucket and hit.what == "inner wall"


def test_stream_over_the_far_rim_is_spilled():
    start = FLOOR + np.array([-0.06, 0.0, w.DEPTH + 0.06])
    hit = _trace(start, (1, 0, 0.3), speed=2.5)
    assert not hit.into_bucket and hit.what == "ground"


def test_stream_outside_the_pail_is_spilled():
    beside = FLOOR + np.array([0.30, 0.0, w.DEPTH])
    assert _trace(beside, (0, 0, -1)).what == "ground"
    at_wall = FLOOR + np.array([w.R_TOP + 0.002, 0.0, w.DEPTH - 0.05])
    hit = _trace(at_wall + np.array([-0.0, 0, 0.0]) + np.array([0.06, 0, 0.0]), (-1, 0, 0), speed=1.0)
    assert not hit.into_bucket and hit.what in ("outer wall", "ground")


def test_submerged_outlet_fills():
    inside = FLOOR + np.array([0.0, 0.0, 0.05])
    hit = _trace(inside, (1, 0, 0), level=0.10)
    assert hit.into_bucket and hit.what == "water"


# -- scoring -------------------------------------------------------------------------------------------------------


def _episode(litres=0.0, missed=0.0, overflow=0.0, off_s=5.0, tipped=False):
    ep = T.Episode()
    ep.tank.in_bucket, ep.tank.missed, ep.tank.overflow, ep.tank.off_s = litres, missed, overflow, off_s
    ep.tipped = tipped
    return ep


@pytest.mark.parametrize("name, kwargs, expected", [
    ("empty", dict(), 0.0),
    ("a fifth", dict(litres=0.2 * w.BRIM_L), 0.0),
    ("a quarter", dict(litres=0.26 * w.BRIM_L), 0.25),
    ("half", dict(litres=0.55 * w.BRIM_L), 0.5),
    ("most", dict(litres=0.86 * w.BRIM_L), 0.75),
    ("full, water off 1 s", dict(litres=w.BRIM_L - 0.3), 1.0),
    ("brimful exactly", dict(litres=w.BRIM_L), 1.0),
    ("full but still running", dict(litres=w.BRIM_L - 0.3, off_s=0.0), 0.75),
    ("full, off only 0.5 s", dict(litres=w.BRIM_L - 0.3, off_s=0.5), 0.75),
    ("full, 0.2 L ran over", dict(litres=w.BRIM_L, overflow=0.2), 1.0),
    ("full, 0.4 L ran over", dict(litres=w.BRIM_L, overflow=0.4), 0.75),
    ("full, 0.3 L missed", dict(litres=w.BRIM_L - 0.2, missed=0.3), 0.75),
    ("1.1 L spilled in all", dict(litres=w.BRIM_L, missed=0.6, overflow=0.5), 0.0),
    ("just short of full", dict(litres=w.volume_at(w.DEPTH - 0.016)), 0.75),
    ("tipped", dict(litres=w.BRIM_L - 0.3, tipped=True), 0.0),
])
def test_score(name, kwargs, expected):
    assert T.score_of(_episode(**kwargs)) == expected, name


def test_scores_never_rise_with_more_spill():
    for litres in np.linspace(0, w.BRIM_L, 8):
        last = 1.0
        for spill in np.linspace(0, 1.5, 16):
            s = T.score_of(_episode(litres=litres, missed=spill))
            assert s <= last + 1e-12
            last = s


# -- the per-step water update, as the simulator calls it ----------------------------------------------------------


def _hold(pitch=45.0):
    pos, rot = T.hold_pose(pitch)
    return pos, rot


def _trigger(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def test_fill_then_overflow_and_puddle():
    ep = T.Episode()
    pos, rot = _hold()
    bucket_pos = np.array(BUCKET.pos)
    dt = 0.05
    steps = 0
    while ep.tank.level < w.DEPTH - 0.010:
        T.advance(ep, dt, pos, rot, rot @ _trigger(22.0), bucket_pos, UP)
        steps += 1
    assert ep.tank.missed == 0.0 and ep.tank.overflow == 0.0
    assert steps * dt == pytest.approx(w.volume_at(w.DEPTH - 0.010) / w.FLOW_MAX, abs=0.1)
    assert T.score_of(ep) == 0.75  # still running
    for _ in range(20):  # let go: the water stops
        T.advance(ep, dt, pos, rot, rot, bucket_pos, UP)
    assert T.score_of(ep) == 1.0
    for _ in range(80):  # squeeze again for 4 s: about 0.4 L runs over the brim
        T.advance(ep, dt, pos, rot, rot @ _trigger(22.0), bucket_pos, UP)
    assert ep.tank.overflow > 0.3 and ep.puddles.spots
    for _ in range(20):
        T.advance(ep, dt, pos, rot, rot, bucket_pos, UP)
    assert T.score_of(ep) == 0.75


def test_aiming_off_the_pail_spills():
    ep = T.Episode()
    pos, rot = _hold()
    pos = pos + np.array([0.0, 0.30, 0.0])  # 30 cm to the left: beside the pail
    for _ in range(40):
        T.advance(ep, 0.05, pos, rot, rot @ _trigger(22.0), np.array(BUCKET.pos), UP)
    assert ep.tank.in_bucket == 0.0 and ep.tank.missed == pytest.approx(0.4, abs=1e-6)
    assert len(ep.puddles.spots) >= 1


def test_tipping_the_pail_fails():
    ep = T.Episode()
    pos, rot = _hold()
    tipped = np.array([[1, 0, 0], [0, math.cos(0.3), -math.sin(0.3)], [0, math.sin(0.3), math.cos(0.3)]])
    T.advance(ep, 0.05, pos, rot, rot, np.array(BUCKET.pos), tipped)
    assert ep.tipped and T.score_of(ep) == 0.0


def test_trigger_angle():
    _, rot = _hold()
    for deg in (0.0, 7.5, 22.0):
        assert T.trigger_angle(rot, rot @ _trigger(deg)) == pytest.approx(deg)
