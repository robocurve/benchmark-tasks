"""Checks of dig_out_box.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_dig_out_box.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("dig_out_box.py"))
M = TASK.module
LOWER, UPPER = M.OTHERS


def box_state(x, y, bottom_z, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """A box with its centre at x, y, turned ``yaw_deg`` about the vertical then tilted ``tilt_deg`` about world y,
    its lowest corner at ``bottom_z``. The bounding box is built as Arena 0.3.1 builds it (turned local corners)."""
    s, t = math.radians(yaw_deg), math.radians(tilt_deg)
    rz = np.array([[math.cos(s), -math.sin(s), 0], [math.sin(s), math.cos(s), 0], [0, 0, 1]])
    ry = np.array([[math.cos(t), 0, math.sin(t)], [0, 1, 0], [-math.sin(t), 0, math.cos(t)]])
    rot = ry @ rz
    half = np.array([M.BOX_X / 2, M.BOX_Y / 2, M.BOX_H / 2])
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half @ rot.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + centre
    q = np.array([math.sin(t / 2) * math.sin(s / 2), math.sin(t / 2) * math.cos(s / 2),
                  math.cos(t / 2) * math.sin(s / 2), math.cos(t / 2) * math.cos(s / 2)])
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=world.min(axis=0), high=world.max(axis=0))


def on_table(x, y, level=0, **kw):
    return box_state(x, y, M.TOP + level * M.BOX_H, **kw)


def in_square(dx=0.0, dy=0.0, **kw):
    return box_state(M.ZONE_XY[0] + dx, M.ZONE_XY[1] + dy, M.TOP + M.ZONE_T, **kw)


def scene(target=None, lower=None, upper=None):
    """The start, except for the boxes given."""
    sx, sy = M.STACK_XY
    return {M.TARGET: target or on_table(sx, sy, 0), LOWER: lower or on_table(sx, sy, 1),
            UPPER: upper or on_table(sx, sy, 2)}


def restacked_at(x, y, upper_dx=0.0):
    return {"lower": on_table(x, y, 0), "upper": on_table(x + upper_dx, y, 1)}


def test_the_start_scores_zero():
    assert M.score(scene()) == 0.0 and not M.success(scene())


def test_green_in_the_square_and_the_rest_restacked_on_the_spot_is_success():
    objects = scene(in_square(), **restacked_at(*M.STACK_XY))
    assert M.success(objects) and M.score(objects) == 1.0


def test_either_brown_box_may_be_the_lower_one():
    r = restacked_at(*M.STACK_XY)
    assert M.score(scene(in_square(), lower=r["upper"], upper=r["lower"])) == 1.0


@pytest.mark.parametrize(("offset_mm", "expected"), [(8, 1.0), (12, 0.5)])
def test_the_restack_must_be_within_10_mm_of_the_spot(offset_mm, expected):
    sx, sy = M.STACK_XY
    assert M.score(scene(in_square(), **restacked_at(sx + offset_mm / 1000, sy))) == expected


def test_restacked_elsewhere_scores_half():
    assert M.score(scene(in_square(), **restacked_at(0.34, 0.20))) == 0.5


def test_brown_boxes_left_scattered_score_zero():
    """Sweeping them off the stack to get at the green box earns nothing."""
    assert M.score(scene(in_square(), on_table(0.34, 0.20), on_table(0.30, 0.10))) == 0.0


def test_a_brown_box_on_its_side_scores_zero():
    sx, sy = M.STACK_XY
    knocked = on_table(0.34, 0.20, tilt_deg=90)
    assert M.score(scene(in_square(), on_table(sx, sy, 0), knocked)) == 0.0


def test_upside_down_boxes_count():
    """A plain box looks the same either way up, from above and in the photo check."""
    sx, sy = M.STACK_XY
    objects = scene(in_square(tilt_deg=180), on_table(sx, sy, 0, tilt_deg=180), on_table(sx, sy, 1, tilt_deg=180))
    assert M.success(objects) and M.score(objects) == 1.0
    leaning = scene(in_square(tilt_deg=168), **restacked_at(sx, sy))  # 12 degrees from upside down
    assert M.score(leaning) == 0.0


def test_the_upper_box_must_sit_squarely():
    sx, sy = M.STACK_XY
    assert M.score(scene(in_square(), **restacked_at(sx, sy, upper_dx=0.012))) == 1.0
    assert M.score(scene(in_square(), **restacked_at(sx, sy, upper_dx=0.020))) == 0.0


def test_green_not_in_the_square_scores_zero():
    assert M.score(scene(on_table(0.34, 0.20), **restacked_at(*M.STACK_XY))) == 0.0


@pytest.mark.parametrize(("dx_mm", "inside"), [(14, True), (16, False)])
def test_green_must_be_wholly_inside_the_square(dx_mm, inside):
    """The square is 9 cm and the box 6 cm, so its centre can be up to 15 mm off."""
    objects = scene(in_square(dx=dx_mm / 1000), **restacked_at(*M.STACK_XY))
    assert M.score(objects) == (1.0 if inside else 0.0)


def test_a_turned_green_box_counts_by_its_corners():
    assert M.score(scene(in_square(yaw_deg=45), **restacked_at(*M.STACK_XY))) == 1.0  # 4.2 cm half-extent < 4.5
    assert M.score(scene(in_square(dx=0.01, yaw_deg=45), **restacked_at(*M.STACK_XY))) == 0.0


def test_still_moving_is_not_yet_success():
    r = restacked_at(*M.STACK_XY)
    objects = scene(in_square(moving=True), **r)
    assert M.score(objects) == 1.0 and not M.success(objects)


def test_the_demo_ends_with_everything_in_place():
    places = [w.pos[:2] for w in M.demo()["left"] if w.label.endswith("let go")]
    assert places == [pytest.approx(M.ASIDE_XY), pytest.approx(M.ASIDE_XY), pytest.approx(M.ZONE_XY),
                      pytest.approx(M.STACK_XY), pytest.approx(M.STACK_XY)]


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["left"], station.BASES["left"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
