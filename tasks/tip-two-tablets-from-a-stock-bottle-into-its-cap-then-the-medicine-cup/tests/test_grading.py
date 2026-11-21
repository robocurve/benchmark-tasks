"""The scorer on hand-built end states and on the cheap strategies it has to refuse."""

import numpy as np
import pytest

from rdb_tablets_cap import fixed_scene as fixture
from rdb_tablets_cap import grading
from rdb_tablets_cap import layout as L

BOTTLE = (np.array((*fixture.BOTTLE_XY, 0.)), np.eye(3))
CAP = (np.array((*fixture.CAP_XY, .070)), np.eye(3))          # held level, 70 mm up
CAP_ON_TABLE = (np.array((*fixture.CAP_XY, 0.)), np.eye(3))
CUP = (np.array((*fixture.CUP_XY, 0.)), np.eye(3))
AWAY = [np.array((.14, .34, .20)), np.array((.14, -.34, .20))]
LYING = (np.array((*fixture.BOTTLE_XY, L.BOTTLE_R)), np.array(((0., 0., 1.), (0., 1., 0.), (-1., 0., 0.))))


def tablets(cup=0, cap=0, table=0):
    points = np.array([np.asarray(BOTTLE[0]) + L.tablet_start(i) for i in range(L.N_TABLETS)])
    i = 0
    for _ in range(cup):
        points[i] = CUP[0] + (.003 * i, 0., .005 + .004 * i); i += 1
    for k in range(cap):
        points[i] = CAP[0] + (.009 * np.cos(2.1 * k), .009 * np.sin(2.1 * k), L.CAP_FLOOR + .003); i += 1
    for k in range(table):
        points[i] = (.30 + .02 * k, .08, L.TABLET_T / 2); i += 1
    return points


def opening_sample(grade, press_mm, yaw_deg, *, bimanual=True, bottle=BOTTLE):
    angle = np.radians(yaw_deg)
    twist = np.array(((np.cos(angle), -np.sin(angle), 0.), (np.sin(angle), np.cos(angle), 0.), (0., 0., 1.)))
    pos, rot = bottle
    cap = (pos + rot @ (0., 0., L.CLOSED_Z-press_mm/1000), rot @ twist @ np.diag((1., -1., -1.)))
    points = pos + np.array([L.tablet_start(i) for i in range(L.N_TABLETS)]) @ rot.T
    return grade.update(points, bottle, cap, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY,
                        cap_held=bimanual, opening_bimanual=bimanual)


def opened_grade():
    grade = grading.TabletGrade()
    for press, turn in ((0., 0.), (2.8, 0.), (2.8, 23.), (2.8, 42.), (-18., 42.)):
        opening_sample(grade, press, turn)
    assert grade.opening.stage == "opened"
    return grade


def run(states, *, bottle=BOTTLE, cup=CUP, grippers=AWAY, touching=None, repeat=30, cap=None, held=True,
        supported=False):
    """Feed a list of tablet layouts, the last one ``repeat`` times, and return the final grade."""
    grade = opened_grade()
    out = None
    for k, points in enumerate(states):
        for _ in range(repeat if k == len(states) - 1 else 8):
            touch = np.zeros(L.N_TABLETS, dtype=bool) if touching is None else touching
            out = grade.update(points, bottle, CAP if cap is None else cap, cup, touch, grippers, cap_held=held,
                               cap_supported=supported)
    return out


VIA_CAP = [tablets(), tablets(cap=2), tablets(cup=2)]


def test_start_state_scores_nothing():
    out = run([tablets()])
    assert out["score"] == 0. and out["in_bottle"] == L.N_TABLETS and out["outcome"] == "nothing"


def test_full_marks_need_two_in_the_cup_through_the_cap_and_the_rest_in_the_bottle():
    out = run(VIA_CAP)
    assert out["score"] == 1. and out["in_cup"] == 2 and out["in_bottle"] == 28 and out["void"] is None


@pytest.mark.parametrize("states, score", [
    ([tablets(), tablets(cap=4), tablets(cup=2, cap=2)], .7),            # two extra left in the cap
    ([tablets(), tablets(cap=3), tablets(cup=2, table=1)], .7),          # one dropped on the table
    ([tablets(), tablets(cap=6), tablets(cup=2, cap=4)], 0.),            # four not put back: a spill
    ([tablets(), tablets(cap=1), tablets(cup=1)], .4),
    ([tablets(), tablets(cap=3), tablets(cup=3)], .4),
    ([tablets(), tablets(cap=4), tablets(cup=4)], 0.),
    ([tablets(), tablets(cap=2)], .2),
    ([tablets(), tablets(cap=2, table=1)], 0.),
    ([tablets(), tablets(cap=3)], 0.),
])
def test_partial_credit(states, score):
    assert run(states)["score"] == score


def test_pouring_straight_into_the_cup_is_void_even_if_two_land():
    out = run([tablets(), tablets(cup=2)])
    assert out["score"] == 0. and out["void"] == "tablet_reached_cup_without_lying_in_cap"


def test_a_tablet_that_only_bounces_through_the_cap_has_not_lain_in_it():
    grade = opened_grade()
    for points, n in ((tablets(), 5), (tablets(cap=2), 2), (tablets(cup=2), 30)):
        for _ in range(n):
            out = grade.update(points, BOTTLE, CAP, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert out["void"] == "tablet_reached_cup_without_lying_in_cap"


def test_handling_a_tablet_with_a_gripper_is_void_but_a_glancing_touch_is_not():
    touch = np.zeros(L.N_TABLETS, dtype=bool)
    touch[0] = True
    assert run(VIA_CAP, touching=touch)["void"] == "gripper_handled_a_tablet"
    grade = opened_grade()
    for k, points in enumerate(VIA_CAP):
        for step in range(30):
            out = grade.update(points, BOTTLE, CAP, CUP, touch if (k == 1 and step < 3) else ~touch & False, AWAY, cap_held=True)
    assert out["score"] == 1. and out["max_touch_ticks"] == 3


def test_tablets_returned_to_the_bottle_must_pass_through_the_cap_again():
    grade = opened_grade()
    for points in (tablets(), tablets(cap=2), tablets(), tablets(cup=2)):
        for _ in range(10):
            out = grade.update(points, BOTTLE, CAP, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert out["void"] == "tablet_reached_cup_without_lying_in_cap"


def test_bottle_left_lying_or_cup_knocked_over():
    lying_tablets = tablets(cup=2)
    out = run([tablets(), tablets(cap=2), lying_tablets], bottle=LYING)
    assert out["score"] in (0., .7) and out["bottle_standing"] is False
    tipped = (CUP[0] + (0., 0., .02), LYING[1])
    assert run(VIA_CAP, cup=tipped)["score"] == 0.


def test_still_holding_or_still_moving_caps_the_score():
    holding = [np.array((*fixture.BOTTLE_XY, .03)) + (0., -.01, 0.), AWAY[1]]
    assert run(VIA_CAP, grippers=holding)["score"] == grading.CAP_NOT_DONE
    assert run(VIA_CAP, grippers=None)["score"] == grading.CAP_NOT_DONE
    assert run(VIA_CAP, repeat=10)["score"] == grading.CAP_NOT_DONE          # not yet still for a second


def test_regions_do_not_overlap_at_the_start():
    points = tablets()
    assert grading.in_bottle(BOTTLE, points).all()
    assert not grading.in_cap(CAP, points).any() and not grading.in_cup(CUP, points).any()


def test_a_cap_held_just_over_the_bottle_counts_its_own_tablets():
    mouth = BOTTLE[0] + (0., 0., L.BOTTLE_H)
    cap = (mouth + (-.020, 0., .016 - L.CAP_H), np.eye(3))       # inner rim 16 mm above the lip, over the mouth
    points = tablets()
    points[0] = cap[0] + (.0, .0, L.CAP_FLOOR + .003)
    points[1] = cap[0] + (.008, .0, L.CAP_FLOOR + .003)
    bottled, capped, cupped = grading.regions(BOTTLE, cap, CUP, points)
    assert capped[:2].all() and not bottled[:2].any() and bottled[2:].all() and not cupped.any()


def test_a_tablet_in_the_cap_while_it_rests_on_the_table_is_void():
    grade = opened_grade()
    on_table = tablets()
    on_table[0] = CAP_ON_TABLE[0] + (0., 0., L.CAP_FLOOR + .003)
    on_table[1] = CAP_ON_TABLE[0] + (.009, 0., L.CAP_FLOOR + .003)
    for _ in range(10):
        out = grade.update(on_table, BOTTLE, CAP_ON_TABLE, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert out["void"] == "tablet_in_cap_resting_on_table" and out["score"] == 0.
    for _ in range(30):                                          # lifting it afterwards does not undo it
        out = grade.update(tablets(cup=2), BOTTLE, CAP, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert out["score"] == 0.


def test_an_empty_cap_may_lie_on_the_table_and_a_held_cap_may_tilt():
    grade = opened_grade()
    for _ in range(10):
        grade.update(tablets(), BOTTLE, CAP_ON_TABLE, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert grade.void is None
    tilted = (CAP[0], np.array(((1., 0., 0.), (0., np.cos(.6), -np.sin(.6)), (0., np.sin(.6), np.cos(.6)))))
    assert not grading.cap_resting(tilted) and grading.cap_resting(CAP_ON_TABLE)


def test_a_cap_balanced_on_the_upturned_cup_is_not_held():
    """The loophole a review found: the cap set on an overturned cup is 35 mm up, well clear of the height test."""
    upturned_cup = (np.array((*fixture.CUP_XY, L.CUP_H)), np.diag((1., -1., -1.)))
    cap_on_cup = (np.array((*fixture.CUP_XY, L.CUP_H)), np.eye(3))
    assert not grading.cap_resting(cap_on_cup)
    grade = opened_grade()
    points = tablets()
    points[0] = cap_on_cup[0] + (0., 0., L.CAP_FLOOR + .003)
    for _ in range(10):                                       # no gripper on it; the cup holds it up
        out = grade.update(points, BOTTLE, cap_on_cup, upturned_cup, np.zeros(L.N_TABLETS, dtype=bool), AWAY,
                           cap_held=False, cap_supported=True)
    assert out["void"] == "tablet_in_cap_resting_on_bottle_or_cup" and out["score"] == 0.
    grade = opened_grade()                             # a light touch of the fingers does not make it held
    for _ in range(10):
        out = grade.update(points, BOTTLE, cap_on_cup, upturned_cup, np.zeros(L.N_TABLETS, dtype=bool), AWAY,
                           cap_held=True, cap_supported=True)
    assert out["void"] == "tablet_in_cap_resting_on_bottle_or_cup"


def test_a_tablet_in_a_cap_no_gripper_is_pinching_is_void():
    assert run([tablets(), tablets(cap=2), tablets(cup=2)], held=False)["void"] == "tablet_in_cap_not_held"
    assert run([tablets(), tablets(cap=2), tablets(cup=2)], held=None)["void"] == "tablet_in_cap_not_held"


def test_a_held_cap_scores_and_a_moment_of_lost_contact_is_tolerated():
    assert run([tablets(), tablets(cap=2), tablets(cup=2)], held=True)["score"] == 1.
    grade = opened_grade()
    for k, points in enumerate([tablets(), tablets(cap=2), tablets(cup=2)]):
        for step in range(30):
            held = not (k == 1 and step < grading.UNHELD_TICKS - 1)   # under 0.25 s without both fingertips
            out = grade.update(points, BOTTLE, CAP, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=held)
    assert out["void"] is None and out["score"] == 1.


def test_an_empty_cap_needs_no_hand():
    grade = opened_grade()
    for _ in range(30):
        out = grade.update(tablets(), BOTTLE, CAP_ON_TABLE, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY,
                           cap_held=False, cap_supported=True)
    assert out["void"] is None


def test_opening_is_measured_in_the_bottle_frame():
    grade = grading.TabletGrade()
    for (press, turn), stage in zip(((0, 0), (3, 0), (3, 23), (3, 42), (-18, 42)),
                                    ("locked", "pressed", "past_stop", "turned", "opened"), strict=True):
        frame = opening_sample(grade, press, turn, bottle=LYING)
        assert frame["opening_stage"] == stage and frame["score"] == 0


@pytest.mark.parametrize("trajectory", [
    [(0, 0, True), (-30, 0, True)],                         # pull straight off
    [(0, 0, True), (0, 45, True), (-30, 45, True)],         # turn without pressing
    [(3, 0, True), (0, 10, True), (0, 45, True), (-30, 45, True)], # release too soon
    [(3, 0, True), (3, -23, True), (3, -45, True), (-30, -45, True)],
    [(3, 0, False), (3, 23, False), (3, 42, False), (-30, 42, False)],
    [(3, 0, True), (3, 23, True), (3, 42, True), (-18, 42, False)],
])
def test_unverified_opening_cannot_earn_dispensing_credit(trajectory):
    grade = grading.TabletGrade()
    for press, turn, bimanual in trajectory:
        opening_sample(grade, press, turn, bimanual=bimanual)
    assert grade.opening.stage != "opened"
    for points in VIA_CAP:
        for _ in range(30):
            frame = grade.update(points, BOTTLE, CAP, CUP, np.zeros(L.N_TABLETS, dtype=bool), AWAY, cap_held=True)
    assert frame["score"] == 0 and frame["void"] == "tablet_left_before_robot_opened_safety_cap"
