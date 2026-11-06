"""Scorer tests for pvc.py, run without the simulator:  python test_pvc.py"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import pvc as v  # noqa: E402

HZ = v.CONTROL_HZ
TEE_POS = (v.TEE_C[0], v.TEE_C[1], (v.TOP + v.SOCKET_TOP) / 2)  # reported centre: the tee's bounding box
AWAY_L, AWAY_R = (0.15, 0.27, v.TOP + 0.17), (0.15, -0.27, v.TOP + 0.17)
ON_TEE = (v.TEE_C[0], 0.10, v.TEE_C[2] + 0.005)


def _state(pos, size, yaw=0.0):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    q = (0.0, 0.0, math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2))
    return ObjectState(pos, np.asarray(q), np.zeros(3), np.zeros(3), pos - half, pos + half)


def frame(pipe_bottom=None, pipe_yaw=0.0, holding=False, left=AWAY_L, tee_dx=0.0, tee_yaw=0.0):
    """pipe_bottom None: the pipe in its holder; otherwise its bottom's height, in the socket."""
    if pipe_bottom is None:
        pipe = v.PIPE_C
    else:
        pipe = (v.TEE_C[0], v.TEE_C[1], pipe_bottom + v.PIPE_L / 2)
    right = (pipe[0], pipe[1], pipe[2] + 0.035) if holding else AWAY_R
    return {
        "tee": _state((TEE_POS[0] + tee_dx, TEE_POS[1], TEE_POS[2]), (0.033, 0.16, v.SOCKET_TOP - v.TOP), yaw=tee_yaw),
        "pipe": _state(pipe, (2 * v.PIPE_R, 2 * v.PIPE_R, v.PIPE_L), yaw=pipe_yaw),
        "left_tcp": _state(left, (0.02, 0.02, 0.02)),
        "right_tcp": _state(right, (0.02, 0.02, 0.02)),
    }


class FakeSim:
    def __init__(self, t, states):
        self.t, self.states = t, states

    def set_velocity(self, *args, **kwargs):
        pass


def run(frames):
    out = None
    for i, f in enumerate(frames):
        v._joint(FakeSim((i + 1) / HZ, f))
        out = v.score(f)
    return out


def secs(s, **kw):
    return [frame(**kw)] * int(round(s * HZ))


IN = dict(pipe_bottom=v.SEAT_Z, pipe_yaw=90.0, holding=True, left=ON_TEE)


def joint(hold_s=31.0, start_s=5.0):
    return secs(start_s, left=ON_TEE) + secs(hold_s, **IN)


def test_nothing_done_scores_zero():
    assert run(secs(5)) == 0.0


def test_full_joint_scores_one():
    assert run(joint() + secs(2, pipe_bottom=v.SEAT_Z, pipe_yaw=90.0, left=ON_TEE)) == 1.0


def test_no_quarter_turn_scores_quarter():
    assert run(secs(5, left=ON_TEE) + secs(31, pipe_bottom=v.SEAT_Z, holding=True, left=ON_TEE)) == 0.25


def test_letting_go_too_soon_scores_quarter():
    f = joint(hold_s=10.0) + secs(25, pipe_bottom=v.SEAT_Z, pipe_yaw=90.0, left=ON_TEE)
    assert run(f) == 0.25


def test_joined_after_the_cement_dried_scores_quarter():
    assert run(joint(start_s=20.0)) == 0.25


def test_still_holding_scores_half():
    assert run(joint(hold_s=10.0)) == 0.5


def test_not_pushed_fully_in_scores_zero():
    assert run(secs(5, left=ON_TEE) + secs(31, pipe_bottom=v.SEAT_Z + 0.01, pipe_yaw=90.0, holding=True, left=ON_TEE)) == 0.0


def test_tee_twisted_scores_zero():
    assert run(joint(hold_s=5.0) + secs(1, **IN | dict(left=AWAY_L), tee_yaw=30.0)) == 0.0


def test_tee_settling_into_the_grip_before_the_pipe_goes_in_is_fine():
    assert run(secs(1, left=ON_TEE, tee_yaw=12.0, tee_dx=0.004) + secs(3, left=ON_TEE, tee_yaw=12.0, tee_dx=0.004)
               + secs(31, **IN, tee_yaw=12.0, tee_dx=0.004)
               + secs(1, pipe_bottom=v.SEAT_Z, pipe_yaw=90.0, tee_yaw=12.0, tee_dx=0.004)) == 1.0


def test_tee_pushed_away_scores_zero():
    assert run(joint(hold_s=5.0) + secs(1, **IN, tee_dx=0.03)) == 0.0


def test_new_episode_resets():
    run(secs(2, tee_dx=0.03))
    assert run(joint()) == 1.0


if __name__ == "__main__":
    tests = [t for k, t in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
