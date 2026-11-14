from types import SimpleNamespace
import numpy as np
from flange_assembly.geometry import holes, score


def state(pos, q, resting=True):
    return SimpleNamespace(pos=np.array(pos, dtype=float), quat_xyzw=np.array(q, dtype=float),
                           lin_vel=np.zeros(3) if resting else np.array([.02, 0., 0.]),
                           ang_vel=np.zeros(3), resting=lambda: resting)


def assembly():
    s = np.sqrt(.5)
    fixed = state((.32, 0, .95), (0, s, 0, s))
    loose = state((.2295, .02, .95), (0, -s, 0, s))
    pins = [state(holes(fixed)[i] + (.004, 0, 0), (0, -s, 0, s)) for i in (0, 2)]
    return fixed, loose, pins


def test_full_and_partial():
    f, l, p = assembly()
    assert abs(score(f, l, p) - 1) < 1e-9
    assert abs(score(f, l, p[:1]) - .7) < 1e-9


def test_misalignment_and_motion():
    f, l, p = assembly()
    l.pos[2] += .004
    assert score(f, l, p) == 0
    f, l, p = assembly()
    l.resting = lambda: False
    assert abs(score(f, l, p) - 1.) < 1e-9


def test_pin_depth_orientation_and_duplicate():
    f, l, p = assembly()
    p[0].pos[0] -= .02
    assert abs(score(f, l, p) - .7) < 1e-9
    f, l, p = assembly()
    p[0].quat_xyzw = np.array([0, 0, 0, 1])
    assert abs(score(f, l, p) - .7) < 1e-9
    f, l, p = assembly()
    p[1].pos = p[0].pos.copy()
    assert abs(score(f, l, p) - .7) < 1e-9


def test_rotated_flange_cannot_score():
    f, l, p = assembly()
    l.quat_xyzw = np.array([0., 0., 0., 1.])
    assert score(f, l, p) == 0.


def test_motion_does_not_change_final_position_score():
    f, l, p = assembly()
    p[0].lin_vel = np.array([.02, 0., 0.])
    assert abs(score(f, l, p) - 1.) < 1e-9


def test_spin_about_pin_axis_does_not_affect_seating():
    f, l, p = assembly()
    p[0].ang_vel = np.array([2., 0., 0.])
    assert abs(score(f, l, p) - 1.) < 1e-9
    p[0].ang_vel = np.array([0., 2., 0.])
    assert abs(score(f, l, p) - 1.) < 1e-9


def test_side_holes_do_not_count():
    f, l, p = assembly()
    p[0].pos = holes(f)[1] + (.004, 0, 0)
    assert abs(score(f, l, p) - .7) < 1e-9


def test_pin_not_through_joint_is_not_counted():
    f, l, p = assembly()
    p[0].pos[0] -= .03
    assert abs(score(f, l, p) - .7) < 1e-9
