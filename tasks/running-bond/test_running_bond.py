"""Checks of running_bond.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_running_bond.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("running_bond.py"))
M = TASK.module
SIZES = {"brick": np.array(M.BRICK), "half_1": np.array(M.HALF), "half_2": np.array(M.HALF)}


def _rot(yaw, roll=0.0):
    """The spin about the vertical times a roll about the piece's own length."""
    cz, sz, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    return np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]) @ np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])


def piece(name, x, y=0.0, yaw_deg=0.0, bottom_z=None, roll_deg=0.0, moving=False):
    """``name`` with its centre at x, y, turned ``yaw_deg``, rolled ``roll_deg`` about its length, its lowest corner
    at ``bottom_z`` (the laid course's top by default). A box's bounding box from its turned corners is exact."""
    bottom_z = M.COURSE_1_TOP if bottom_z is None else bottom_z
    psi, rho = math.radians(yaw_deg), math.radians(roll_deg)
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * SIZES[name] / 2
    corners = corners @ _rot(psi, rho).T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    sz, cz, sx, cx = math.sin(psi / 2), math.cos(psi / 2), math.sin(rho / 2), math.cos(rho / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([cz * sx, sz * sx, sz * cx, cz * cx]),
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=(corners + centre).min(axis=0),
        high=(corners + centre).max(axis=0),
    )


def at_start(name):
    return piece(name, *M.STARTS[name], bottom_z=M.TOP + 0.001)


def course(near=0.0, middle=0.0, far=0.0, **kw):
    """The new course laid right, each piece moved along the wall by the given mm (+ = away from the arms)."""
    return {
        "half_1": piece("half_1", M.HOMES["half_1"] + near / 1000, **kw.get("half_1", {})),
        "brick": piece("brick", M.HOMES["brick"] + middle / 1000, **kw.get("brick", {})),
        "half_2": piece("half_2", M.HOMES["half_2"] + far / 1000, **kw.get("half_2", {})),
    }


def scene(**placed):
    s = {p.name: at_start(p.name) for p in M.PIECES}
    s.update(placed)
    return s


def test_the_quaternion_matches_the_rotation():
    p = piece("brick", 0.3, yaw_deg=25, roll_deg=3)
    assert np.allclose(M._rotation(p.quat_xyzw), _rot(math.radians(25), math.radians(3)))
    assert M._tilt_deg(p) == pytest.approx(3, abs=1e-9)
    assert M._off_line_deg(p) == pytest.approx(25, abs=1e-9)


def test_the_pieces_make_a_course_as_long_as_the_one_below():
    """Half, joint, brick, joint, half = brick, joint, brick."""
    assert 2 * M.HALF[0] + 2 * M.JOINT + M.BRICK[0] == pytest.approx(M.WALL_FAR - M.WALL_NEAR)


def test_the_start_scores_zero():
    s = scene()
    assert not M.success(s) and M.score(s) == 0.0


def test_the_demo_aims_at_a_running_bond():
    s = scene(**course())
    assert M.success(s) and M.score(s) == 0.75
    assert M._joints_mm(s) == pytest.approx((10.0, 10.0))


def test_the_demo_aims_lay_their_piece():
    """A piece that landed exactly where the demo aims it, AIM offset included, is still in its place."""
    for name in ("half_1", "brick", "half_2"):
        dx, dy = M.AIM[name]
        assert M.score(scene(**{name: piece(name, M.HOMES[name] + dx, y=dy)})) == 0.25, name


def test_the_halves_may_swap():
    c = course()
    c["half_1"], c["half_2"] = (piece("half_1", M.HOMES["half_2"]), piece("half_2", M.HOMES["half_1"]))
    assert M.success(scene(**c))


@pytest.mark.parametrize("laid", [["half_1"], ["half_1", "brick"], ["half_1", "brick", "half_2"]])
def test_each_piece_in_place_scores_a_quarter(laid):
    s = scene(**{k: v for k, v in course().items() if k in laid})
    assert M.score(s) == 0.25 * len(laid)


def test_the_brick_over_a_brick_below_is_not_running_bond():
    """Stack bond: the full brick flush with the near end, over the near brick below. It scores nothing there."""
    s = scene(brick=piece("brick", M.WALL_NEAR + M.BRICK[0] / 2))
    assert M.score(s) == 0.0


def test_a_half_over_the_joint_does_not_count_as_the_brick():
    assert M.score(scene(half_1=piece("half_1", M.WALL_X))) == 0.0


@pytest.mark.parametrize("name", ["half_1", "half_2"])
@pytest.mark.parametrize(("mm", "ok"), [(2.9, True), (-2.9, True), (3.5, False), (-3.5, False)])
def test_a_half_flush_with_the_wall_end_within_3_mm(name, mm, ok):
    assert M.score(scene(**{name: piece(name, M.HOMES[name] + mm / 1000)})) == (0.25 if ok else 0.0)


@pytest.mark.parametrize(("mm", "ok"), [(4.9, True), (-4.9, True), (5.5, False)])
def test_the_brick_within_5_mm_of_the_joint_below(mm, ok):
    assert M.score(scene(brick=piece("brick", M.WALL_X + mm / 1000))) == (0.25 if ok else 0.0)


@pytest.mark.parametrize(("near", "middle", "far", "ok"), [
    (0.0, 2.9, 0.0, True),  # joints 12.9 and 7.1 mm
    (2.0, -2.0, 0.0, False),  # near joint 6.0 mm
    (-2.9, 2.9, 2.9, False),  # near joint 15.8 mm
])
def test_joints_from_the_pieces(near, middle, far, ok):
    """Each piece within its own limit, so 0.75 either way; success also needs both joints 7 to 13 mm."""
    s = scene(**course(near, middle, far))
    near_j, far_j = M._joints_mm(s)
    assert near_j == pytest.approx(10.0 + middle - near) and far_j == pytest.approx(10.0 + far - middle)
    assert M.success(s) == ok and M.score(s) == 0.75


def test_every_piece_in_place_but_a_joint_too_wide_is_not_success():
    """Each piece within its own limit (half 2.9 mm past the end, brick 2.7 mm off), the near joint 15.6 mm."""
    s = scene(**course(near=-2.9, middle=2.7))
    assert M.score(s) == 0.75 and not M.success(s)


@pytest.mark.parametrize(("y_mm", "ok"), [(2.9, True), (-2.9, True), (3.5, False), (-3.5, False)])
def test_in_line_with_the_wall_within_3_mm(y_mm, ok):
    assert M.score(scene(brick=piece("brick", M.WALL_X, y=y_mm / 1000))) == (0.25 if ok else 0.0)


@pytest.mark.parametrize(("yaw_deg", "ok"), [(2.5, True), (-2.5, True), (3.5, False), (90.0, False)])
def test_along_the_wall_within_3_degrees(yaw_deg, ok):
    assert M.score(scene(brick=piece("brick", M.WALL_X, yaw_deg=yaw_deg))) == (0.25 if ok else 0.0)


def test_end_for_end_counts():
    assert M.score(scene(brick=piece("brick", M.WALL_X, yaw_deg=180.0))) == 0.25


@pytest.mark.parametrize(("roll_deg", "ok"), [(2.9, True), (3.1, False), (90, False)])
def test_flat_within_3_degrees(roll_deg, ok):
    assert M.score(scene(brick=piece("brick", M.WALL_X, roll_deg=roll_deg))) == (0.25 if ok else 0.0)


def test_upside_down_counts():
    assert M.score(scene(brick=piece("brick", M.WALL_X, roll_deg=180))) == 0.25


@pytest.mark.parametrize(("bottom_mm", "ok"), [(0.0, True), (0.9, True), (-0.9, True), (1.1, False), (-1.1, False),
                                               (25.0, False)])
def test_held_above_or_on_another_piece_does_not_count(bottom_mm, ok):
    p = piece("brick", M.WALL_X, bottom_z=M.COURSE_1_TOP + bottom_mm / 1000)
    assert M.score(scene(brick=p)) == (0.25 if ok else 0.0)


def test_on_the_table_beside_the_wall_does_not_count():
    assert M.score(scene(half_2=piece("half_2", M.HOMES["half_2"], y=0.06, bottom_z=M.TOP))) == 0.0


def test_turned_pieces_are_measured_along_the_wall():
    """Turned 2.9 degrees, within the limit: the joints are still 10 mm along the wall, as a ruler reads them."""
    s = scene(**course(**{n: {"yaw_deg": 2.9} for n in ("half_1", "brick", "half_2")}))
    assert M._joints_mm(s) == pytest.approx((10.0, 10.0)) and M.success(s)


def test_a_turned_course_with_a_wide_joint_is_not_success():
    """Each piece within its own limit, the near joint 14.5 mm along the wall."""
    s = scene(**course(middle=4.5, far=2.8, half_1={"yaw_deg": 2.9}, brick={"yaw_deg": 2.9}))
    assert M._joints_mm(s)[0] == pytest.approx(14.5) and not M.success(s)


def test_a_moving_piece_is_not_yet_success():
    c = course()
    c["brick"] = piece("brick", M.WALL_X, moving=True)
    s = scene(**c)
    assert not M.success(s) and M.score(s) == 0.75


def test_distance_is_zero_when_laid():
    assert M.distance(scene(**course())) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
