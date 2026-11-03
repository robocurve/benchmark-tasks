"""Checks of ladder_angle.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_ladder_angle.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("ladder_angle.py"))
M = TASK.module
SIZE = np.array([M.THICK, M.WIDTH, M.LENGTH])


def _state(centre, rot, moving=False):
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * SIZE / 2 @ rot.T + centre
    return ObjectState(pos=np.asarray(centre, dtype=float), quat_xyzw=_quat(rot),
                       lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=corners.min(axis=0), high=corners.max(axis=0))


def _quat(rot):
    """xyzw quaternion of a rotation matrix, from its largest diagonal term (safe at half turns)."""
    t = np.trace(rot)
    if t > 0:
        w = math.sqrt(1 + t) / 2
        return np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1], 4 * w * w]) / (4 * w)
    i = int(np.argmax(np.diag(rot)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = math.sqrt(1 + rot[i, i] - rot[j, j] - rot[k, k]) / 2
    q = np.zeros(4)
    q[i] = v
    q[j] = (rot[j, i] + rot[i, j]) / (4 * v)
    q[k] = (rot[k, i] + rot[i, k]) / (4 * v)
    q[3] = (rot[k, j] - rot[j, k]) / (4 * v)
    return q


def _rot_y(beta):
    c, s = math.cos(beta), math.sin(beta)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(psi):
    c, s = math.cos(psi), math.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def place(rot, lift_mm=0.0, off_edge_mm=0.0, dy=0.0, moving=False):
    """The ladder turned by ``rot``, its lowest corner on the table (or ``lift_mm`` above it), the nearer corner of
    its face toward the wall on the wall's top edge (or ``off_edge_mm`` short of it), ``dy`` along the wall."""
    n = rot[:, 0] * math.copysign(1.0, rot[0, 0])  # the normal of the face toward the wall
    cz = M.TOP + float(np.abs(rot[2]) @ SIZE) / 2 + lift_mm / 1000
    reach = M.THICK / 2 + M.WIDTH / 2 * abs(n[1]) + off_edge_mm / 1000
    cx = M.WALL_X - (reach - (M.WALL_TOP - cz) * n[2]) / n[0]
    return _state(np.array([cx, M.WALL_Y + dy, cz]), rot, moving)


def pitch(angle_deg):
    """The ladder's length at ``angle_deg`` from the table, its top toward the wall."""
    return _rot_y(math.pi / 2 - math.radians(angle_deg))


def leaning(angle_deg, **kw):
    return place(pitch(angle_deg), **kw)


def _rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def standing():
    return _state(np.array(M.LADDER.pos), np.eye(3))


@pytest.mark.parametrize("rot", [_rot_z(0.3) @ _rot_y(0.4), _rot_x(math.pi), _rot_z(math.pi) @ _rot_y(1.0)])
def test_the_quaternion_matches_the_rotation(rot):
    assert np.allclose(M._rotation(_state(np.zeros(3), rot).quat_xyzw), rot)


def test_the_start_scores_zero():
    o = {"ladder": standing()}
    assert M.score(o) == 0.0 and not M.success(o)


@pytest.mark.parametrize(("angle_deg", "want"), [
    (75.5, 1.0), (73.0, 1.0), (78.0, 1.0), (71.0, 0.5), (80.0, 0.5), (69.0, 0.0), (82.0, 0.0), (85.0, 0.0),
])
def test_the_angle_bands(angle_deg, want):
    o = {"ladder": leaning(angle_deg)}
    assert M.score(o) == want and M.success(o) == (want == 1.0)


def test_the_demo_gap_lands_at_a_quarter():
    """Tipped over its foot's wall-side edge from GAP_AIM out, the ladder meets the wall's edge at cos = 1/4."""
    lad = leaning(math.degrees(math.acos(0.25)))
    th, rot = math.acos(0.25), M._rotation(lad.quat_xyzw)
    pivot = np.asarray(lad.pos) - M.LENGTH / 2 * rot[:, 2] + M.THICK / 2 * rot[:, 0]  # the foot's wall-side edge
    assert M.foot_share(lad) == pytest.approx(0.25)
    assert M.WALL_X - pivot[0] == pytest.approx(M.GAP_AIM) and pivot[2] == pytest.approx(M.TOP)
    assert th == pytest.approx(math.radians(75.52), abs=1e-3)


def test_lying_on_the_table_does_not_count():
    lad = _state(np.array([0.22, M.WALL_Y, M.TOP + M.THICK / 2]), _rot_y(math.pi / 2))
    assert M.score({"ladder": lad}) == 0.0


@pytest.mark.parametrize(("lift_mm", "ok"), [(1.5, True), (3.0, False)])
def test_its_foot_on_the_table(lift_mm, ok):
    assert M.score({"ladder": leaning(75.5, lift_mm=lift_mm)}) == (1.0 if ok else 0.0)


@pytest.mark.parametrize(("off_mm", "ok"), [(2.5, True), (4.0, False)])
def test_resting_on_the_walls_edge(off_mm, ok):
    """Held at the right angle but clear of the wall's edge doesn't count."""
    assert M.score({"ladder": leaning(75.5, off_edge_mm=off_mm)}) == (1.0 if ok else 0.0)


@pytest.mark.parametrize(("dy", "ok"), [(0.075, True), (0.085, False)])
def test_over_the_wall(dy, ok):
    """Its centre over the wall counts, its side overhanging; past the wall's end it is beside the wall."""
    assert M.score({"ladder": leaning(75.5, dy=dy)}) == (1.0 if ok else 0.0)


def test_the_far_face_on_the_edge_does_not_count():
    """Sunk into the wall until its face away from the wall is on the edge."""
    assert M.score({"ladder": leaning(75.5, off_edge_mm=-M.THICK * 1000)}) == 0.0


@pytest.mark.parametrize("flip", ["end_for_end", "face_for_face"])
def test_turned_round_it_still_leans(flip):
    """The bar is the same either way up and either face toward the wall."""
    turn = _rot_x(math.pi) if flip == "end_for_end" else _rot_z(math.pi)
    assert M.score({"ladder": place(pitch(75.5) @ turn)}) == 1.0


@pytest.mark.parametrize(("yaw_deg", "ok"), [(9.5, True), (12.0, False)])
def test_square_to_the_wall(yaw_deg, ok):
    assert M._leaning(place(_rot_z(math.radians(yaw_deg)) @ pitch(75.5))) == ok


@pytest.mark.parametrize(("roll_deg", "ok"), [(8.0, True), (12.0, False)])
def test_no_sideways_lean(roll_deg, ok):
    assert M._leaning(place(_rot_x(math.radians(roll_deg)) @ pitch(75.5))) == ok


@pytest.mark.parametrize(("twist_deg", "ok"), [(8.0, True), (12.0, False)])
def test_twisted_about_its_length(twist_deg, ok):
    """One rail nearer the wall than the other."""
    assert M._leaning(place(pitch(75.5) @ _rot_z(math.radians(twist_deg)))) == ok


def test_past_the_wall_does_not_count():
    """Its centre beyond the wall's face: on the wall's far side, not leaning on it from the arms' side."""
    lad = leaning(75.5)
    far = np.asarray(lad.pos).copy()
    far[0] = M.WALL_X + 0.01
    assert not M._leaning(_state(far, M._rotation(lad.quat_xyzw)))


def test_a_moving_ladder_is_not_yet_success():
    o = {"ladder": leaning(75.5, moving=True)}
    assert M.score(o) == 1.0 and not M.success(o)


def test_distance_is_zero_at_a_quarter():
    assert M.distance({"ladder": leaning(math.degrees(math.acos(0.25)))}) == pytest.approx(0.0, abs=1e-9)
    assert M.distance({"ladder": standing()}) == pytest.approx(0.25)


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
