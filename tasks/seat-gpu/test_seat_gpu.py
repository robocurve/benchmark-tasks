"""Checks of seat_gpu.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_seat_gpu.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("seat_gpu.py"))
M = TASK.module
MM = 0.001


def quat(yaw_deg=0.0, tilt_deg=0.0):
    """Turned yaw about z, then tilted about the card's long axis (x)."""
    a, b = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    qz = np.array([0, 0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(b), 0, 0, math.cos(b)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def card(dx=0.0, dy=0.0, dz=0.0, yaw_deg=0.0, tilt_deg=0.0, moving=False, origin=None, q=None, spinning=False):
    """The card with its frame origin ``(dx, dy, dz)`` from the seated place (or at ``origin``); ``q`` overrides the
    turn and tilt."""
    q = quat(yaw_deg, tilt_deg) if q is None else q
    o = (M.SEATED_O if origin is None else np.asarray(origin)) + np.array([dx, dy, dz])
    pos = o + M._rot(q) @ M.POS_OFF
    half = np.array([0.129, 0.0195, 0.0556])
    return ObjectState(pos=pos, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.array([0, 0, 1.0]) if spinning else np.zeros(3), low=pos - half, high=pos + half)


def objs(c):
    return {"card": c}


def test_start_scores_zero():
    o = objs(card(origin=M.CARD_O))
    assert M.score(o) == 0 and not M.success(o)


def test_seated_still_is_success():
    assert M.score(objs(card())) == 1.0 and M.success(objs(card()))


def test_seated_but_moving_is_not_success():
    o = objs(card(moving=True))
    assert M.score(o) == 0.8 and not M.success(o)


def test_seated_but_turning_is_not_success():
    o = objs(card(spinning=True))
    assert M.score(o) == 0.8 and not M.success(o)


def test_top_score_only_with_success():
    for c in (card(), card(moving=True), card(spinning=True), card(dz=5 * MM), card(dz=20 * MM)):
        assert (M.score(objs(c)) == 1.0) == M.success(objs(c))


@pytest.mark.parametrize("dz,want", [(0.9 * MM, 1.0), (-0.9 * MM, 1.0), (1.1 * MM, 0.5), (7.9 * MM, 0.5),
                                     (8.1 * MM, 0.2), (39 * MM, 0.2), (41 * MM, 0.0), (-1.1 * MM, 0.0)])
def test_height_tiers(dz, want):
    assert M.score(objs(card(dz=dz))) == want


@pytest.mark.parametrize("dx,want", [(1.4 * MM, 1.0), (-1.4 * MM, 1.0), (1.6 * MM, 0.2), (3.1 * MM, 0.0)])
def test_seated_along_the_slot(dx, want):
    assert M.score(objs(card(dx=dx))) == want


@pytest.mark.parametrize("dy,want", [(0.9 * MM, 1.0), (1.1 * MM, 0.5), (1.6 * MM, 0.2), (3.1 * MM, 0.0)])
def test_seated_across_the_slot(dy, want):
    assert M.score(objs(card(dy=dy))) == want


@pytest.mark.parametrize("tilt,want", [(1.9, 1.0), (2.1, 0.5), (5.1, 0.0)])
def test_tilt(tilt, want):
    assert M.score(objs(card(tilt_deg=tilt))) == want


@pytest.mark.parametrize("tilt,want", [(4.9, 0.2), (5.1, 0.0)])
def test_lined_up_tilt_limit(tilt, want):
    # 20 mm above seated, over the slot: only the 0.2 tier's own 5 degree limit decides
    assert M.score(objs(card(dz=20 * MM, tilt_deg=tilt))) == want


def tilt_along(deg):
    """Tilted about the card's short horizontal axis (y): one end down."""
    b = math.radians(deg) / 2
    return np.array([0.0, math.sin(b), 0.0, math.cos(b)])


def pitched(deg, lift=0.0, **kw):
    """Pitched along the card (positive: far end down), its lower connector end ``lift`` above the slot floor."""
    s = math.sin(math.radians(deg))
    dz = max(float(p[0]) * s for p in M.CONNECTOR_ENDS) + lift
    return card(dz=dz, q=tilt_along(deg), **kw)


@pytest.mark.parametrize("deg", [0.6, -0.6])
def test_pitched_within_1_mm_at_both_connector_ends_is_seated(deg):
    assert M.score(objs(pitched(deg))) == 1.0


@pytest.mark.parametrize("deg,lift", [(0.7, 0.0), (-0.7, 0.0), (1.0, 0.9 * MM), (-1.0, 0.9 * MM), (1.9, 0.0),
                                      (-1.9, 0.9 * MM)])
def test_pitched_with_one_connector_end_out_is_not_seated(deg, lift):
    # e.g. far end up 1.9 deg: fingers 2.2 to 5.1 mm above the floor; only started into the slot
    assert M.score(objs(pitched(deg, lift))) == 0.5


def test_one_connector_end_above_the_slot_is_not_started():
    # pitched 3.5 deg far end up: near end 3 mm up, far end of the connector 8.2 mm up, so not in along its length
    assert M.score(objs(pitched(-3.5, 3.0 * MM))) == 0.2
    assert M.score(objs(pitched(-3.5, 2.5 * MM))) == 0.5


def test_square_card_on_the_key_rib_scores_as_started():
    assert M.score(objs(card(dz=3.5 * MM))) == 0.5


@pytest.mark.parametrize("tilt,dz,want", [(4.9, 20 * MM, 0.2), (5.1, 20 * MM, 0.0), (-4.9, 20 * MM, 0.2)])
def test_tilt_along_the_card(tilt, dz, want):
    assert M.score(objs(card(dz=dz, q=tilt_along(tilt)))) == want


def test_far_end_above_the_support_channel_scores_nothing():
    # far end up 4.9 deg, 35 mm above seated: the body's bottom at the channel is above the channel's top
    c = card(dz=35 * MM, q=tilt_along(-4.9))
    assert not M.far_end_in_channel(c) and M.score(objs(c)) == 0.0
    assert M.score(objs(card(dz=39 * MM))) == 0.2 and M.far_end_in_channel(card(dz=39 * MM))


@pytest.mark.parametrize("yaw,want", [(1.9, 1.0), (2.1, 0.5), (3.1, 0.0), (-1.9, 1.0), (-2.1, 0.5), (-3.1, 0.0)])
def test_yaw(yaw, want):
    assert M.score(objs(card(yaw_deg=yaw))) == want


def test_reversed_card_scores_nothing():
    for dz in (0.0, 5 * MM, 20 * MM):
        for yaw in (180.0, 179.0, -179.0):
            assert M.score(objs(card(dz=dz, yaw_deg=yaw))) == 0.0


def test_lined_up_needs_both_axes_within_3_mm():
    assert M.score(objs(card(dx=2.9 * MM, dz=20 * MM))) == 0.2
    assert M.score(objs(card(dy=-3.1 * MM, dz=20 * MM))) == 0.0


def test_card_lying_on_the_riser_scores_nothing():
    assert M.score(objs(card(dz=10 * MM, tilt_deg=90.0))) == 0.0


def test_pos_offset_roundtrip():
    c = card(dx=0.7 * MM, dy=-0.3 * MM, dz=4 * MM, yaw_deg=1.0, tilt_deg=1.0)
    e, tilt, yaw = M.card_pose(c)
    assert np.allclose(e, [0.7 * MM, -0.3 * MM, 4 * MM], atol=1e-6)
    assert abs(tilt - 1.0) < 1e-6 and abs(yaw - 1.0) < 1e-6


def test_demo_reaches():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert plan.duration < 2400 / 20
