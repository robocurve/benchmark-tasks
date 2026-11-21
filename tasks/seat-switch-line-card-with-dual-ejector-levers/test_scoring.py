"""Offline test suite for seat_switch_line_card_with_dual_ejector_levers."""

from __future__ import annotations

import importlib
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from mechkit import PartState  # noqa: E402

T = importlib.import_module("seat_switch_line_card_with_dual_ejector_levers")
D = importlib.import_module("demo")


def _card(gap=T.START_OUT, cock=0.0, lever_l=-T.OPEN_DEG, lever_r=T.OPEN_DEG):
    yaw = math.atan2(-cock, 2 * T.END_Y)
    yaw = -yaw
    pos = np.array([T.SEAT_X - gap, 0.0, T.TOP + T.FLOOR])
    quat = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    joints = {"lever_l_joint": math.radians(lever_l), "lever_r_joint": math.radians(lever_r)}
    return PartState(pos=pos, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=pos, high=pos, joints=joints)


def _objects(**kw):
    return {"chassis": None, "card": _card(**kw)}


def _run(frames, clear=True):
    T._reset_state()
    T._STATE["clear"] = clear
    out = (0.0, False)
    for objs in frames:
        out = (T.score(objs), T.success(objs))
    return out


def test_end_gaps_follow_the_yaw():
    g = T.end_gaps(_card(gap=0.005, cock=0.002))
    assert abs(g.mean() - 0.005) < 1e-6
    assert abs((g[0] - g[1]) - 0.002) < 1e-5


def test_start_scores_zero():
    assert _run([_objects()]) == (0.0, False)


def test_pushed_to_the_pins():
    assert _run([_objects(gap=0.021, lever_l=-T.OPEN_LIMIT, lever_r=T.OPEN_LIMIT)])[0] == 0.25


def test_not_yet_at_the_pins():
    assert _run([_objects(gap=0.030)])[0] == 0.0


def test_connectors_mating():
    assert _run([_objects(gap=0.006, lever_l=-20, lever_r=20)])[0] == 0.5


def test_seated_levers_open_cannot_award_075():
    # Reviewer requirement: levers open (> 2.5 deg) cannot award 0.75
    assert _run([_objects(gap=0.0005, lever_l=-5, lever_r=5)]) == (0.50, False)


def test_seated_and_shut_awards_075():
    # Both levers shut <= 2.5 deg and seated <= 1.0 mm awards 0.75
    assert _run([_objects(gap=0.0005, lever_l=-2.0, lever_r=2.0)]) == (0.75, False)


def test_seated_and_shut():
    assert _run([_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4) for _ in range(20)]) == (1.0, True)


def test_full_score_only_after_one_second_held():
    held = [_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4) for _ in range(20)]
    assert _run(held[:19]) == (0.75, False)
    assert _run(held) == (1.0, True)


def test_seated_and_shut_but_still_held_is_not_success():
    assert _run([_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4)], clear=False) == (0.75, False)


def test_tab_points_follow_the_levers():
    shut = T.tab_points(_card(gap=0.0, lever_l=0.0, lever_r=0.0))
    assert np.allclose(shut[:, :2], [[T.SEAT_X + T.PIVOT_X, T.PIVOT_Y - T.HANDLE_L - 0.001],
                                     [T.SEAT_X + T.PIVOT_X, -T.PIVOT_Y + T.HANDLE_L + 0.001]], atol=1e-6)
    opened = T.tab_points(_card(gap=0.0))
    assert np.all(opened[:, 0] < shut[:, 0] - 0.09)


def test_one_lever_shut_is_not_success():
    assert _run([_objects(gap=0.0003, lever_l=-0.4, lever_r=5.0)]) == (0.50, False)


def test_seated_then_backed_out_keeps_progress():
    frames = [_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4), _objects(gap=0.015)]
    assert _run(frames) == (0.75, False)


def test_one_end_seated_is_not_seated():
    assert _run([_objects(gap=0.0012, cock=0.001)])[0] == 0.5


def test_cocked_while_mating_is_sticky_zero():
    frames = [_objects(gap=0.004, cock=0.002), _objects(gap=0.0003, lever_l=-0.4, lever_r=0.4)]
    assert _run(frames) == (0.0, False)


def test_cocked_before_mating_is_allowed():
    frames = [_objects(gap=0.030, cock=0.003)] + [_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4) for _ in range(20)]
    assert _run(frames) == (1.0, True)


def test_slightly_uneven_while_mating_is_allowed():
    frames = [_objects(gap=0.004, cock=0.0012)] + [_objects(gap=0.0003, lever_l=-0.4, lever_r=0.4) for _ in range(20)]
    assert _run(frames) == (1.0, True)


def test_card_home_meets_the_bottomed_connectors():
    rear_at_home = T.SEAT_X - T.X_FACE + T.CARD_D
    assert abs(rear_at_home - (T.CONN_X0 + T.CONN_TRAVEL)) < 1e-9


def test_levers_shut_where_the_card_is_home():
    assert abs(D.card_x(T, -1.07) - T.SEAT_X) < 2e-4
    assert T.SEAT_X - D.card_x(T, T.LATCH_DEG) < T.HOME


def test_claws_engage_from_where_the_card_is_pushed():
    for gap in (0.025, 0.015, 0.008):
        rel = T.X_FACE + T.PIN_X - (T.SEAT_X - gap + T.PIVOT_X)
        a = next(a for a in np.linspace(T.OPEN_DEG, 0, 2000) if D.card_x(T, a) >= T.SEAT_X - gap)
        assert 5 < a < T.OPEN_DEG - 3
        assert math.hypot(rel, T.CLAW_R) + T.PIN_R < T.CLAW_L + 0.003


def test_leverage():
    assert 50.0 * T.CLAW_R / T.HANDLE_L < 10.0
    assert 2 * T.CONN_RATE * T.CONN_TRAVEL > 2 * 20.0


def test_card_clears_the_guides_and_pins():
    assert abs(T.SLOT_W / 2 - T.CARD_W / 2 - 0.0015) < 1e-9
    assert T.PIN_Y - T.PIN_R > T.FACE_HALF_W
    jam = math.degrees((T.SLOT_W - T.CARD_W) / T.CARD_D)
    assert jam < 0.5
    span = 2 * T.END_Y * math.radians(jam)
    assert span > T.COCK_LIMIT


def _claw_corners(gap, side, a_deg):
    pivot = np.array([T.SEAT_X - gap + T.PIVOT_X, side * T.PIVOT_Y])
    a = math.radians(a_deg)
    d = np.array([math.sin(a), side * math.cos(a)])
    n = np.array([-d[1], d[0]])
    return [pivot + r * d + w * n for r in (0.006, T.CLAW_L) for w in (-0.003, 0.003)]


def test_open_claws_pass_the_pins_and_the_guides():
    for side in (1, -1):
        pin = np.array([T.X_FACE + T.PIN_X, side * T.PIN_Y])
        for gap in np.linspace(T.START_OUT, 0.0, 61):
            corners = _claw_corners(gap, side, T.OPEN_DEG)
            assert all(abs(c[1]) < T.PIN_Y - T.PIN_R - 0.003 for c in corners)
            assert all(abs(c[1]) < T.SLOT_W / 2 - 0.002 for c in corners)
            assert np.linalg.norm(corners[0] - pin) > T.PIN_R


def test_claws_start_clear_of_the_faceplate_ends():
    assert T.PIVOT_Y - 0.0035 > T.FACE_HALF_W


def test_demo_waypoints_reachable():
    from yam_arena import ik, station
    from yam_arena.constants import READY

    for arm, pos, rot in D.waypoints(T):
        _, pe, _ = ik.solve(pos, ik.matrix_to_quat_xyzw(rot), np.array(READY), base_pos=station.BASES[arm], iters=300)
        assert pe < 0.002, (arm, pos)
