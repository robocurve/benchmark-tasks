"""Scorer tests for guards.py, run without the simulator:  python test_guards.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import guards as g  # noqa: E402

AWAY = (0.10, 0.0, g.TOP + 0.30)  # a fingertip far from the wheel
NEAR = (g.WHEEL_C[0], g.WHEEL_C[1] + g.WHEEL_R + 0.01, g.AXLE_Z)  # a fingertip at the rest, next to the wheel


def _state(pos, size, w=0.0):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.zeros(3), np.array([w, 0.0, 0.0]), pos - half, pos + half)


def frame(rest_gap=g.START_GAP_M, tongue_gap=g.START_GAP_M, w=0.0, pressed=False, left=AWAY, right=AWAY):
    rest_y = g.WHEEL_C[1] + g.WHEEL_R + rest_gap + g.REST_SIZE[1] / 2
    tongue_z = g.AXLE_Z + g.WHEEL_R + tongue_gap + g.TONGUE_SIZE[2] / 2
    btn_z = g.BTN_Z - (0.004 if pressed else 0.0)
    return {
        "wheel": _state(g.WHEEL_C, (0.025, 0.11, 0.11), w=w),
        "wheel_mark": _state((g.WHEEL_C[0], g.WHEEL_C[1], g.AXLE_Z + 0.048), (0.028, 0.02, 0.02), w=w),
        "wheel_mark2": _state((g.WHEEL_C[0], g.WHEEL_C[1], g.AXLE_Z - 0.048), (0.028, 0.02, 0.02), w=w),
        "tool_rest": _state((g.REST_C[0], rest_y, g.REST_C[2]), g.REST_SIZE),
        "tongue": _state((g.TONGUE_C[0], g.TONGUE_C[1], tongue_z), g.TONGUE_SIZE),
        "start_button": _state((*g.CTRL_XY, btn_z), (0.035, 0.035, 0.02)),
        "left_tcp": _state(left, (0.02, 0.02, 0.02)),
        "right_tcp": _state(right, (0.02, 0.02, 0.02)),
    }


class FakeSim:
    def __init__(self, t, states):
        self.t, self.states = t, states

    def set_velocity(self, *args, **kwargs):
        pass


def run(frames):
    """Each control step: the tick hook (as the simulator calls it), then score()."""
    out = None
    for i, f in enumerate(frames):
        g._motor(FakeSim((i + 1) / g.CONTROL_HZ, f))
        out = g.score(f)
    return out


SET = dict(rest_gap=0.002, tongue_gap=0.004)  # both within the OSHA gaps


def test_nothing_done_scores_zero():
    assert run([frame()] * 10) == 0.0


def test_start_straight_away_scores_zero():
    assert run([frame(), frame(pressed=True), frame(w=g.WHEEL_W)]) == 0.0


def test_one_guard_set_scores_quarter():
    assert run([frame(), frame(rest_gap=0.002)]) == 0.25
    assert run([frame(), frame(tongue_gap=0.005)]) == 0.25


def test_both_set_not_started_scores_half():
    assert run([frame(), frame(**SET)]) == 0.5


def test_full_procedure_scores_one():
    assert run([frame(), frame(**SET), frame(**SET, pressed=True), frame(**SET, w=g.WHEEL_W)] + [frame(**SET, w=g.WHEEL_W)] * 5) == 1.0


def test_rest_too_far_at_start_scores_zero():
    f = dict(rest_gap=0.004, tongue_gap=0.004)  # 4 mm > 1/8 in
    assert run([frame(), frame(**f, pressed=True), frame(**f, w=g.WHEEL_W)]) == 0.0


def test_tongue_too_far_at_start_scores_zero():
    f = dict(rest_gap=0.002, tongue_gap=0.007)  # 7 mm > 1/4 in
    assert run([frame(), frame(**f, pressed=True), frame(**f, w=g.WHEEL_W)]) == 0.0


def test_touching_the_wheel_at_start_scores_zero():
    f = dict(rest_gap=0.0002, tongue_gap=0.004)
    assert run([frame(), frame(**f, pressed=True), frame(**f, w=g.WHEEL_W)]) == 0.0


def test_reaching_near_the_turning_wheel_scores_zero():
    frames = [frame(), frame(**SET, pressed=True), frame(**SET, w=g.WHEEL_W), frame(**SET, w=g.WHEEL_W, left=NEAR)]
    assert run(frames + [frame(**SET, w=g.WHEEL_W)] * 3) == 0.0


def test_hands_near_the_wheel_before_start_are_fine():
    frames = [frame(left=NEAR, right=NEAR), frame(**SET, left=NEAR), frame(**SET, pressed=True), frame(**SET, w=g.WHEEL_W)]
    assert run(frames) == 1.0


def test_violation_does_not_carry_over_to_the_next_episode():
    run([frame(), frame(pressed=True), frame(w=g.WHEEL_W)])
    assert run([frame(), frame(**SET)]) == 0.5


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
