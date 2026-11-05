"""Checks of align_bearings.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_align_bearings.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("align_bearings.py"))
M = TASK.module
A, B = M.NAMES
SQUARE = math.degrees(M.SHAFT_YAW)


def _rot_x(t):
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def bearing_state(xy, turn_deg, bottom_z=None, tilt_deg=0.0, moving=False):
    """A bearing with its centre over ``xy``, turned ``turn_deg`` about the vertical, then tilted ``tilt_deg`` about
    world x, with its lowest point at ``bottom_z`` (the table by default). Its bounding box is built the way Arena
    0.3.1 builds it: the 8 corners of the block, turned with it."""
    bottom_z = M.TOP if bottom_z is None else bottom_z
    t, s = math.radians(tilt_deg), math.radians(turn_deg)
    rot = _rot_x(t) @ _rot_z(s)
    half = np.array(M.BEARING) / 2
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half @ rot.T
    centre = np.array([xy[0], xy[1], bottom_z - corners[:, 2].min()])
    world = corners + centre
    st, ct, ss, cs = math.sin(t / 2), math.cos(t / 2), math.sin(s / 2), math.cos(s / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([st * cs, -st * ss, ct * ss, ct * cs]),  # the tilt about x times the turn about z
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=world.min(axis=0),
        high=world.max(axis=0),
    )


def fixed_state(box):
    centre, half = np.array(box.pos), np.array(box.size) / 2
    return ObjectState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def on_shaft(out_m, across_m=0.0, turn_off_deg=0.0, **kw):
    """A bearing ``out_m`` along the shaft from the motor's face, ``across_m`` off its line, turned ``turn_off_deg``
    from square."""
    xy = M.FACE_XY + out_m * M.ALONG + across_m * M.ACROSS
    return bearing_state(tuple(xy), SQUARE + turn_off_deg, **kw)


def scene(**bearings):
    """The start, except the bearings given as name=ObjectState."""
    out = {box.name: fixed_state(box) for box in M.FIXED}
    for name, (xy, turn) in M.START.items():
        out[name] = bearings.get(name, bearing_state(xy, turn))
    return out


def lined_up(**overrides):
    """Both bearings on the shaft at 10 and 20 cm out, square; then the overrides."""
    return scene(**{**{A: on_shaft(0.10), B: on_shaft(0.20)}, **overrides})


def test_the_shaft_runs_out_of_the_motor_at_30_degrees_and_bore_height():
    assert SQUARE == pytest.approx(30.0)
    assert M.SHAFT_Z - M.TOP == pytest.approx(M.BEARING[2] / 2)  # a bearing's bore centre is at shaft height
    stub = next(b for b in M.FIXED if b.name == "shaft_stub")
    assert np.dot(np.array(stub.pos[:2]) - M.FACE_XY, M.ACROSS) == pytest.approx(0.0, abs=1e-12)


def test_the_bearings_start_off_the_line_and_turned():
    s = scene()
    for name in M.NAMES:
        assert M.off_line(s[name]) > 0.05 and M.squareness_deg(s[name]) >= 20 - 1e-6


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_both_in_line_is_success():
    assert M.success(lined_up()) and M.score(lined_up()) == 1.0


def test_either_bearing_can_go_in_either_place():
    s = lined_up(**{A: on_shaft(0.20), B: on_shaft(0.10)})
    assert M.score(s) == 1.0


@pytest.mark.parametrize(("across_mm", "turn_deg", "credit"), [
    (2.5, 0.0, 0.5), (0.0, 1.8, 0.5), (4.0, 0.0, 0.25), (0.0, 3.0, 0.25), (9.0, 5.0, 0.25), (11.0, 0.0, 0.0),
    (0.0, 7.0, 0.0), (-2.5, -1.8, 0.5),
])
def test_in_line_means_within_3_mm_and_2_degrees(across_mm, turn_deg, credit):
    assert M._slot_credit(on_shaft(0.20, across_mm / 1000, turn_deg), "far") == credit


@pytest.mark.parametrize("across_mm", [3.0, -3.0])
def test_the_3_mm_limit_reads_the_same_both_ways(across_mm):
    # Review: +3.00 mm scored 0.75 and -3.00 mm 1.0 through rounding.
    assert M._slot_credit(on_shaft(0.20, across_mm / 1000), "far") == 0.5


@pytest.mark.parametrize(("near_mm", "far_mm", "expected"), [
    (2.9, -2.9, 0.75), (0.8, -0.8, 1.0), (2.0, 2.0, 1.0), (0.0, 2.5, 0.75),
])
def test_full_score_needs_the_line_through_both_within_1_degree(near_mm, far_mm, expected):
    # Review: each bearing within 3 mm let the line through them run 11 degrees off the shaft. Two 2.9 mm errors in
    # opposite directions 10 cm apart are 3.3 degrees off; the same offset on both keeps the line parallel.
    s = lined_up(**{A: on_shaft(0.10, near_mm / 1000), B: on_shaft(0.20, far_mm / 1000)})
    assert M.score(s) == expected


def test_a_bearing_turned_180_degrees_is_still_square():
    assert M.score(lined_up(**{A: on_shaft(0.10, turn_off_deg=180.0)})) == 1.0


def test_a_bearing_turned_90_degrees_is_across_the_shaft():
    s = lined_up(**{A: on_shaft(0.10, turn_off_deg=90.0)})
    assert M.squareness_deg(s[A]) == pytest.approx(90.0) and M.score(s) == 0.5


@pytest.mark.parametrize(("out_a", "out_b", "expected"), [
    (0.08, 0.22, 1.0), (0.07, 0.20, 0.5), (0.10, 0.23, 0.5), (0.15, 0.20, 0.5), (0.19, 0.21, 0.5),
])
def test_the_bearings_go_about_10_and_20_cm_out(out_a, out_b, expected):
    # The places are 8-12 and 18-22 cm out; both in the same place earn only that place.
    assert M.score(lined_up(**{A: on_shaft(out_a), B: on_shaft(out_b)})) == expected


@pytest.mark.parametrize(("placed", "where", "other_out"), [
    (B, 0.20, None), (A, 0.20, None), (A, 0.10, 0.05),
])
def test_the_same_half_job_scores_the_same_whichever_bearing_is_moved(placed, where, other_out):
    # Review: with the nearer bearing always judged in the near place, these scored 0.5, 0.0 and 0.0.
    other = B if placed == A else A
    others = {} if other_out is None else {other: on_shaft(other_out, 0.06)}
    assert M.score(scene(**{placed: on_shaft(where), **others})) == 0.5


def test_a_bearing_behind_the_motor_earns_nothing():
    assert M.score(lined_up(**{A: on_shaft(-0.12)})) == 0.5


def test_a_bearing_on_its_side_earns_nothing():
    s = lined_up(**{B: on_shaft(0.20, tilt_deg=90.0)})
    assert M.score(s) == 0.5


def test_a_moving_bearing_is_not_yet_in_line():
    s = lined_up(**{B: on_shaft(0.20, moving=True)})
    assert not M.success(s) and M.score(s) == 0.5


def test_a_bearing_off_the_table_scores_zero():
    assert M.score(lined_up(**{B: bearing_state((0.3, 0.6), 0.0, bottom_z=M.TOP - 0.3)})) == 0.0


def _landed():
    """Each bearing let go where the demo aims, then shifted and turned as measured on the official stack."""
    out = {}
    for w, name in zip([w for w in M.demo()["left"] if w.label == "release"], M.NAMES, strict=True):
        d_along, d_across, d_turn = M.DEMO_LANDING[name]
        xy = np.array(w.pos[:2]) + d_along / 1000 * M.ALONG + d_across / 1000 * M.ACROSS
        out[name] = bearing_state(tuple(xy), math.degrees(w.yaw) + 90.0 + d_turn,
                                  bottom_z=w.pos[2] - M.DROP - M.GRIP_ABOVE)
    return out


def test_the_demo_lines_both_up():
    s = scene(**_landed())
    assert M.success(s) and M.score(s) == 1.0
    for name in M.NAMES:
        assert M.off_line(s[name]) < 1e-9 and M.squareness_deg(s[name]) < 1e-6


def test_the_demo_grips_each_bearing_across_its_thickness():
    picks = [w for w in M.demo()["left"] if w.label == "close"]
    for w, (name, (_, turn)) in zip(picks, M.START.items(), strict=True):
        grip_y = np.array([-math.sin(w.yaw), math.cos(w.yaw)])  # the direction the fingers close along
        bore = np.array([math.cos(math.radians(turn)), math.sin(math.radians(turn))])
        assert abs(abs(float(grip_y @ bore)) - 1.0) < 1e-9, name


def test_the_grip_centres_the_bearing():
    from yam_arena.constants import GRIPPER

    squeeze = (M.BEARING[0] - 0.094 * M.GRIP_FINGER / FINGER_OPEN) / 2
    force = GRIPPER["stiffness"] * squeeze
    assert 0.002 <= squeeze and force < 0.5 * GRIPPER["effort"]
    assert 2 * force * 1.0 > 10 * M.BEARING_KG * 9.81


def test_the_demo_lets_go_gently_with_room_either_side():
    apart_mm = 94 * M.RELEASE_FINGER / FINGER_OPEN
    assert apart_mm - M.BEARING[0] * 1000 >= 15 and apart_mm < 94


def test_a_carried_bearing_clears_every_top():
    assert M.TRAVEL_Z - M.GRIP_ABOVE - (M.TOP + max(M.MOTOR[2], M.BEARING[2])) >= 0.03


@pytest.mark.parametrize("arm", list(M.demo()))
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 2400 / 20 / 2
