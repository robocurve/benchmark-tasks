"""Scorer tests for grinder.py, run without the simulator:  python test_grinder.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import grinder as g  # noqa: E402

AWAY = (0.10, 0.0, g.TOP + 0.30)  # a fingertip far from the wheel


def _state(pos, size=(0.02, 0.02, 0.02), w=0.0, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.array([w, 0.0, 0.0]),
                       pos - half, pos + half)


def frame(w=g.WHEEL_W, pressed=False, part=g.PART_C, left=AWAY, right=AWAY):
    btn_z = g.BTN_Z - (0.004 if pressed else 0.0)
    return {
        "wheel": _state(g.WHEEL_C, (0.025, 0.12, 0.12), w=w),
        "wheel_mark": _state((g.WHEEL_C[0], g.WHEEL_C[1], g.AXLE_Z + g.WHEEL_R - 0.012), w=w),
        "wheel_mark2": _state((g.WHEEL_C[0], g.WHEEL_C[1], g.AXLE_Z - g.WHEEL_R + 0.012), w=w),
        "stop_button": _state((*g.CTRL_XY, btn_z), (0.035, 0.035, 0.02)),
        "part": _state(part, g.PART_SIZE),
        "left_tcp": _state(left),
        "right_tcp": _state(right),
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
        g._drive(FakeSim((i + 1) / g.CONTROL_HZ, f))
        out = g.score(f)
    return out


IN_TRAY = (g.TRAY_C[0], g.TRAY_C[1], g.TOP + 0.005 + g.PART_SIZE[2] / 2)
ON_TABLE = (0.20, 0.0, g.TOP + g.PART_SIZE[2] / 2)
NEAR_WHEEL = (g.PART_C[0], g.PART_C[1], g.PART_C[2] + 0.01)  # fingertip at the part, next to the wheel


def coast(n=5):
    """STOP pressed, then the wheel slows to a stop."""
    return [frame(pressed=True)] + [frame(w=g.WHEEL_W * (1 - k / n)) for k in range(1, n)] + [frame(w=0.0)]


def test_nothing_done_scores_zero():
    assert run([frame()] * 10) == 0.0


def test_stop_and_wait_scores_quarter():
    assert run([frame()] + coast()) == 0.25


def test_full_procedure_scores_one():
    frames = [frame()] + coast() + [frame(w=0.0, left=NEAR_WHEEL), frame(w=0.0, part=IN_TRAY)]
    assert run(frames) == 1.0


def test_part_elsewhere_scores_half():
    frames = [frame()] + coast() + [frame(w=0.0, left=NEAR_WHEEL), frame(w=0.0, part=ON_TABLE)]
    assert run(frames) == 0.5


def test_grabbing_the_part_while_the_wheel_coasts_scores_zero():
    frames = [frame(), frame(pressed=True), frame(w=5.0, left=NEAR_WHEEL), frame(w=0.0), frame(w=0.0, part=IN_TRAY)]
    assert run(frames) == 0.0


def test_reaching_near_the_running_wheel_scores_zero_even_without_touching_the_part():
    frames = [frame(), frame(right=(g.WHEEL_C[0] - 0.03, g.WHEEL_C[1], g.WHEEL_C[2]))] + coast() + [
        frame(w=0.0, part=IN_TRAY)
    ]
    assert run(frames) == 0.0


def test_part_moved_while_spinning_scores_zero():
    frames = [frame(), frame(part=(g.PART_C[0] - 0.02, g.PART_C[1], g.PART_C[2]))] + coast() + [
        frame(w=0.0, part=IN_TRAY)
    ]
    assert run(frames) == 0.0


def test_part_dropped_on_the_floor_keeps_only_the_safe_stop_credit():
    floor = (0.20, 0.40, g.TOP - 0.30)
    frames = [frame()] + coast() + [frame(w=0.0, part=floor)]
    assert run(frames) == 0.25  # STOP pressed and wheel stopped; the dropped part earns nothing more


def test_wheel_stopped_without_pressing_stop_is_not_progress():
    assert run([frame(w=0.0)] * 5) == 0.0


def test_new_episode_resets_violation():
    run([frame(), frame(w=5.0, left=NEAR_WHEEL)])
    assert g._state["violated"]
    assert run([frame()] + coast()) == 0.25


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
