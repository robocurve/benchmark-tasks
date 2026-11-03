"""Checks of warning_triangles.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_warning_triangles.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("warning_triangles.py"))
M = TASK.module
NAMES = [t.name for t in M.TRIANGLES]
H = M.CASE[2]


def case(x, y, lift_mm=0.0, roll_deg=0.0, moving=False):
    """A case centred at (x, y), lying on the road (or ``lift_mm`` above it), rolled ``roll_deg`` about x."""
    r = math.radians(roll_deg)
    half = np.array(M.CASE) / 2
    if roll_deg:  # rolled onto its side: the 24 mm side upright
        half = np.array([half[0], abs(half[1] * math.cos(r)) + abs(half[2] * math.sin(r)),
                         abs(half[1] * math.sin(r)) + abs(half[2] * math.cos(r))])
    centre = np.array([x, y, M.ROAD_TOP + half[2] + lift_mm / 1000])
    return ObjectState(pos=centre, quat_xyzw=np.array([math.sin(r / 2), 0.0, 0.0, math.cos(r / 2)]),
                       lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def start():
    return {n: ObjectState(pos=np.array([x, y, M.TOP + H / 2]), quat_xyzw=np.array([0.0, 0, 0, 1]),
                           lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                           low=np.array([x, y, M.TOP]) - np.array(M.CASE) / 2 * [1, 1, 0],
                           high=np.array([x, y, M.TOP + H]) + np.array(M.CASE) / 2 * [1, 1, 0])
            for n, (x, y) in M.CASE_START.items()}


NEAR_X = M.TRAFFIC_FACE_X - M.CASE[0] / 2  # beside the truck's traffic side
behind = lambda ft: M.REAR_Y - ft * M.FT  # noqa: E731
ahead = lambda ft: M.FRONT_Y + ft * M.FT  # noqa: E731


def placed(*cases):
    """``cases`` in order; any case not given stays where it started."""
    o = start()
    for name, c in zip(NAMES, cases):
        o[name] = c
    return o


LEGAL = [case(M.SHOULDER_X, behind(100)), case(NEAR_X, behind(10)), case(M.SHOULDER_X, ahead(100))]


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0.0 and not M.success(o)


def test_the_legal_layout_succeeds():
    o = placed(*LEGAL)
    assert M.score(o) == 1.0 and M.success(o)


@pytest.mark.parametrize("order", [(2, 0, 1), (1, 2, 0)])
def test_any_case_can_go_anywhere(order):
    assert M.success(placed(*(LEGAL[i] for i in order)))


@pytest.mark.parametrize("i", [0, 1, 2])
def test_each_spot_scores_a_third(i):
    assert M.score(placed(LEGAL[i])) == pytest.approx(1 / 3)


def test_two_cases_on_one_spot_count_once():
    assert M.score(placed(LEGAL[0], case(M.SHOULDER_X, behind(110)))) == pytest.approx(1 / 3)


def test_the_divided_road_layout_is_not_full_marks():
    """10, 100 and 200 ft behind is the rule for a divided or one-way road; this road is two-way."""
    o = placed(case(NEAR_X, behind(10)), case(M.SHOULDER_X, behind(100)), case(M.SHOULDER_X, behind(200)))
    assert M.score(o) == pytest.approx(2 / 3) and not M.success(o)


def test_traffic_from_the_wrong_way_is_not_full_marks():
    """The layout mirrored, as if traffic came from the cab's end."""
    o = placed(case(NEAR_X, ahead(10)), case(M.SHOULDER_X, ahead(100)), case(M.SHOULDER_X, behind(100)))
    assert M.score(o) == pytest.approx(2 / 3)


@pytest.mark.parametrize(("ft", "ok"), [(4.5, False), (5.5, True), (14.5, True), (15.5, False)])
def test_the_near_distance(ft, ok):
    assert M.score(placed(case(NEAR_X, behind(ft)))) == pytest.approx(1 / 3 if ok else 0)


@pytest.mark.parametrize(("ft", "ok"), [(74, False), (76, True), (124, True), (126, False)])
@pytest.mark.parametrize("side", ["behind", "ahead"])
def test_the_far_distance(ft, ok, side):
    y = behind(ft) if side == "behind" else ahead(ft)
    assert M.score(placed(case(M.SHOULDER_X, y))) == pytest.approx(1 / 3 if ok else 0)


@pytest.mark.parametrize(("x", "ok"), [
    (NEAR_X, True), (M.TRAFFIC_SIDE_X[0] + 0.001, True), (M.TRAFFIC_SIDE_X[0] - 0.001, False),
    (M.TRAFFIC_FACE_X - 0.001, True), (M.TRAFFIC_FACE_X + 0.001, False), (M.SHOULDER_X, False), (0.198, False),
])
def test_the_near_one_is_on_the_traffic_side(x, ok):
    assert M.score(placed(case(x, behind(10)))) == pytest.approx(1 / 3 if ok else 0)


@pytest.mark.parametrize(("dx_mm", "ok"), [(9, True), (-9, True), (11, False), (-11, False)])
@pytest.mark.parametrize("side", ["behind", "ahead"])
def test_the_far_ones_are_on_the_shoulders_centre(dx_mm, ok, side):
    y = behind(100) if side == "behind" else ahead(100)
    assert M.score(placed(case(M.SHOULDER_X + dx_mm / 1000, y))) == pytest.approx(1 / 3 if ok else 0)


def test_a_case_held_above_the_road_does_not_count():
    assert M.score(placed(case(M.SHOULDER_X, ahead(100), lift_mm=3))) == 0.0
    assert M.score(placed(case(M.SHOULDER_X, ahead(100), lift_mm=1.5))) == pytest.approx(1 / 3)


@pytest.mark.parametrize(("roll_deg", "ok"), [(0, True), (180, True), (90, True), (8, True), (12, False), (45, False)])
def test_a_case_on_any_face_counts_tipped_does_not(roll_deg, ok):
    """The case is the same upside down or on its edge; propped at an angle it isn't lying on the road."""
    assert M.score(placed(case(M.SHOULDER_X, ahead(100), roll_deg=roll_deg))) == pytest.approx(1 / 3 if ok else 0)


def test_moving_is_not_yet_success():
    o = placed(*LEGAL[:2], case(M.SHOULDER_X, ahead(100), moving=True))
    assert M.score(o) == 1.0 and not M.success(o)


def test_the_demo_places_each_case_on_its_spot():
    o = placed(*(case(*M.PLACE[n]) for n in NAMES))
    assert M.success(o)


def test_the_road_ends_are_past_the_far_spots():
    assert M.ROAD_LEN / 2 > max(abs(behind(125)), ahead(125)) + M.CASE[1]


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
