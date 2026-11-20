"""Scorer tests for iron.py, run without the simulator:  python test_iron.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import iron as I  # noqa: E402
import mech  # noqa: E402

AWAY_L, AWAY_R = (0.18, 0.27, I.TOP + 0.20), (0.18, -0.27, I.TOP + 0.20)


def _rx(deg):
    t = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


FLAT = I.R_YAW  # lying on its soleplate, nose to the robot's right (as at the start)
UPRIGHT = _rx(-90.0) @ I.R_YAW  # rolled up about its heel's edge: nose up, standing on its heel
ON_SIDE = I.R_YAW @ _rx(90.0)  # rolled onto its side about its own long axis
FACE_UP = I.R_YAW @ np.diag([1.0, -1.0, -1.0])  # upside down, soleplate up


def _obj(pos, q=(0.0, 0.0, 0.0, 1.0), half=0.01):
    pos = np.asarray(pos, float)
    return ObjectState(pos, np.asarray(q, float), np.zeros(3), np.zeros(3), pos - half, pos + half)


def frame(origin, r=FLAT, left=AWAY_L, right=AWAY_R):
    """A frame with the iron's origin (soleplate's bottom centre) at ``origin`` and its rotation ``r``."""
    origin, r = np.asarray(origin, float), np.asarray(r, float)
    q = mech.matrix_to_quat_xyzw(r)
    return {
        "iron": _obj(origin + r @ I.SHELL_C, q, 0.08),
        "power_light": _obj(origin + r @ np.asarray(I.LIGHT_C), q),
        "left_tcp": _obj(left),
        "right_tcp": _obj(right),
    }


def standing_origin(heel_xy, surface_z):
    """The origin of the iron standing on its heel, the heel's centre at heel_xy on a surface at surface_z."""
    h = UPRIGHT @ np.asarray(I.HEEL_CENTRE)
    bottom = (UPRIGHT @ np.array([I.HEEL_X, 0.0, 0.0]))[2]
    return np.array([heel_xy[0] - h[0], heel_xy[1] - h[1], surface_z - bottom])


def run(frames):
    I._state.clear()
    I._state.update(I._fresh())
    s = 0.0
    for f in frames:
        s = I.score(f)
    return s


START = frame(I.IRON_O)
LIFTED = frame(np.asarray(I.IRON_O) + np.array([0.0, 0.0, 0.05]), right=np.asarray(I._world(I.GRASP_C)) + [0, 0, 0.05])
REST = 25


def stand(heel_xy, surface=None):
    surface = I.surface_z(heel_xy) if surface is None else surface
    return frame(standing_origin(heel_xy, surface), UPRIGHT)


def test_start_is_face_down_on_the_shirt():
    assert I.face_down(START) and not I.upright(START)


def test_doing_nothing_scores_zero():
    assert run([START] * 40) == 0.0


def test_stood_on_its_heel_off_the_shirt_scores_one():
    f = [START] * 3 + [LIFTED] * 5 + [stand((0.38, -0.27))] * REST
    assert I.upright(f[-1]) and not I.on_shirt(I.heel_xy(f[-1]))
    assert run(f) == 1.0


def test_stood_on_its_heel_on_the_shirt_scores_half():
    f = [START] * 3 + [LIFTED] * 5 + [stand((0.29, 0.0))] * REST
    assert I.on_shirt(I.heel_xy(f[-1]))
    assert run(f) == 0.5


def test_set_down_face_down_on_the_table_scores_zero():
    f = [START] * 3 + [LIFTED] * 5 + [frame((0.45, -0.30, I.TOP))] * REST
    assert I.face_down(f[-1])
    assert run(f) == 0.0


def test_set_down_face_down_on_the_mat_scores_zero():
    f = [START] * 3 + [LIFTED] * 5 + [frame((0.30, -0.16, I.MAT_TOP))] * REST
    assert run(f) == 0.0


def test_put_back_face_down_on_the_shirt_scores_zero():
    assert run([START] * 3 + [LIFTED] * 5 + [START] * REST) == 0.0


def test_face_down_only_briefly_then_stood_up_is_still_zero():
    """Setting it down face down for half a second is setting it down; standing it up afterwards doesn't undo it."""
    f = [START] * 3 + [LIFTED] * 5 + [frame((0.45, -0.30, I.TOP))] * 12 + [LIFTED] * 5
    f += [stand((0.38, -0.27))] * REST
    assert run(f) == 0.0


def test_touching_down_face_down_for_a_moment_is_not_setting_it_down():
    f = [START] * 3 + [LIFTED] * 5 + [frame((0.45, -0.30, I.TOP))] * 5 + [LIFTED] * 5
    f += [stand((0.38, -0.27))] * REST
    assert run(f) == 1.0


def test_left_on_its_side_scores_zero():
    o = np.array([0.40, -0.27, I.TOP + 0.055])
    assert run([START] * 3 + [LIFTED] * 5 + [frame(o, ON_SIDE)] * REST) == 0.0


def test_upside_down_scores_zero():
    o = np.array([0.40, -0.27, I.TOP + 0.14])
    assert run([START] * 3 + [LIFTED] * 5 + [frame(o, FACE_UP)] * REST) == 0.0


def test_still_held_scores_zero():
    s = stand((0.38, -0.27))
    held = frame(standing_origin((0.38, -0.27), I.TOP), UPRIGHT, right=I.iron_point(s, I.HANDLE_C))
    assert run([START] * 3 + [LIFTED] * 5 + [held] * REST) == 0.0


def test_needs_a_second_at_rest():
    assert run([START] * 3 + [LIFTED] * 5 + [stand((0.38, -0.27))] * 10) == 0.0


def test_off_the_table_scores_zero():
    assert run([START] * 3 + [LIFTED] * 5 + [frame((0.30, -0.80, I.TOP - 0.40), UPRIGHT)] * REST) == 0.0


def test_laid_flat_on_the_rest_pad_scores_one():
    o = (I.PAD_C[0], I.PAD_C[1], I.PAD_TOP + 0.003)
    f = [START] * 3 + [LIFTED] * 5 + [frame(o)] * REST
    assert I.on_rest_pad(f[-1]) and not I.face_down(f[-1])
    assert run(f) == 1.0


def test_laid_flat_on_the_table_beside_the_pad_scores_zero():
    o = (I.PAD_C[0] + I.PAD_SIZE[0] / 2 + 0.08, I.PAD_C[1], I.TOP)
    f = [START] * 3 + [LIFTED] * 5 + [frame(o)] * REST
    assert I.face_down(f[-1]) and not I.on_rest_pad(f[-1])
    assert run(f) == 0.0


def test_demo_lays_the_whole_soleplate_on_the_pad():
    """At the demo's drop point the soleplate (with its nose) lies within the pad's outline."""
    w = [w for w in I.demo()["right"] if w.label == "let go"][0].pos
    origin = np.asarray(w) - FLAT @ np.asarray(I.GRASP_C)
    for p in ((I.HEEL_X, -0.055, 0.0), (I.HEEL_X, 0.055, 0.0), (0.135, 0.0, 0.0), (0.06, 0.055, 0.0), (0.06, -0.055, 0.0)):
        c = origin + FLAT @ np.asarray(p)
        assert I.on_pad(c[:2]), c
    assert abs(origin[2] - (I.PAD_TOP + 0.003) - 0.004) < 1e-9


def _pad_frames(dx=0.0, dy=0.0):
    o = (I.PAD_C[0] + dx, I.PAD_C[1] + dy, I.PAD_TOP + 0.003)
    return [START] * 3 + [LIFTED] * 5 + [frame(o)] * REST


def test_centre_over_pad_but_nose_over_the_table_scores_zero():
    """Nose tip hangs 4 cm off the pad's far-right end (the nose points to -y): the origin is still over the pad."""
    f = _pad_frames(dy=-(I.PAD_SIZE[1] / 2 - 0.1345 + 0.04))
    assert I.on_pad(I.origin(f[-1])[:2]) and not I.on_rest_pad(f[-1]) and I.face_down(f[-1])
    assert run(f) == 0.0


def test_centre_over_pad_but_heel_over_the_table_scores_zero():
    f = _pad_frames(dy=I.PAD_SIZE[1] / 2 - 0.11 + 0.04)
    assert I.on_pad(I.origin(f[-1])[:2]) and not I.on_rest_pad(f[-1])
    assert run(f) == 0.0


def test_centre_over_pad_but_side_over_the_table_scores_zero():
    f = _pad_frames(dx=I.PAD_SIZE[0] / 2 - 0.055 + 0.03)
    assert I.on_pad(I.origin(f[-1])[:2]) and not I.on_rest_pad(f[-1])
    assert run(f) == 0.0


def test_hanging_off_the_pad_onto_the_mat_scores_zero():
    """Pad at y -0.38; its mat-side edge is at y -0.23; the heel overhangs onto the mat's far edge region is table, so
    check the mat directly: partly on the mat counts as face down too."""
    f = [START] * 3 + [LIFTED] * 5 + [frame((0.30, -0.16, I.MAT_TOP))] * REST
    assert not I.on_rest_pad(f[-1]) and run(f) == 0.0


def test_whole_soleplate_just_inside_the_pad_scores_one():
    f = _pad_frames(dx=I.PAD_SIZE[0] / 2 - 0.055 - 0.005, dy=I.PAD_SIZE[1] / 2 - 0.11 - 0.005)
    assert I.on_rest_pad(f[-1]) and run(f) == 1.0


def test_standing_in_the_wire_holder_scores_one():
    heel = (I.HOLDER_C[0], I.HOLDER_C[1])
    f = [START] * 3 + [LIFTED] * 5 + [stand(heel, I.TOP + I.HOLDER_BASE[2])] * REST
    assert I.upright(f[-1]) and not I.on_shirt(I.heel_xy(f[-1]))
    assert run(f) == 1.0


def test_demo_plan_reaches():
    worst = mech.check_plan(I.demo())
    assert worst["right"][0] < 10.0 and worst["right"][1] < 3.0, worst


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
