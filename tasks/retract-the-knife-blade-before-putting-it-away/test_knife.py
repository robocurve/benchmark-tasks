"""Scorer tests for knife.py, run without the simulator:  python test_knife.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import knife as k  # noqa: E402

AWAY_L, AWAY_R = (0.15, 0.30, k.TOP + 0.14), (0.15, -0.28, k.TOP + 0.14)
IN_TRAY = (k.TRAY_C[0], k.TRAY_C[1], k.TOP + 0.004 + k.HANDLE_SIZE[2] / 2)


def _state(pos, size, moving=False):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    v = np.array([0.0, 0.1, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), v, np.zeros(3), pos - half, pos + half)


def frame(knife=k.KNIFE_C, slid=0.0, left=AWAY_L, right=AWAY_R, moving=False):
    """slid: how far the button has been slid back (0 = blade out 2 cm; 0.024 = fully in)."""
    kp = np.asarray(knife, float)
    bp = kp + (np.asarray(k.BLADE_C) - np.asarray(k.KNIFE_C)) + np.array([0.0, slid, 0.0])
    return {"knife": _state(kp, k.HANDLE_SIZE, moving), "blade": _state(bp, k.BLADE_SIZE, moving),
            "left_tcp": _state(left, (0.02, 0.02, 0.02)), "right_tcp": _state(right, (0.02, 0.02, 0.02))}


class FakeSim:
    def __init__(self, t, states):
        self.t, self.states = t, states


def run(frames):
    out = None
    for i, f in enumerate(frames):
        k._new_episode(FakeSim((i + 1) / k.CONTROL_HZ, f))
        out = k.score(f)
    return out


HOLD = (k.KNIFE_C[0], k.KNIFE_C[1] + 0.07, k.KNIFE_C[2])
TIP = (k.KNIFE_C[0], k.FRONT_Y - 0.01, k.KNIFE_C[2])  # at the exposed blade


def test_nothing_done_scores_zero():
    assert run([frame()] * 5) == 0.0


def test_retracted_on_the_mat_scores_half():
    assert run([frame(), frame(left=HOLD, slid=0.012), frame(left=HOLD, slid=0.024), frame(slid=0.024)]) == 0.5


def test_retracted_and_in_the_tray_scores_one():
    f = [frame(), frame(left=HOLD, slid=0.024), frame(knife=IN_TRAY, slid=0.024, left=IN_TRAY, moving=True),
         frame(knife=IN_TRAY, slid=0.024)]
    assert run(f) == 1.0


def test_in_the_tray_with_the_blade_out_scores_zero():
    f = [frame(), frame(knife=IN_TRAY, left=IN_TRAY, moving=True), frame(knife=IN_TRAY), frame(knife=IN_TRAY, slid=0.024)]
    assert run(f) == 0.0


def test_put_down_elsewhere_with_the_blade_out_scores_zero():
    moved = (0.20, -0.10, k.KNIFE_C[2])
    f = [frame(), frame(knife=moved, left=(0.20, -0.07, k.KNIFE_C[2]), moving=True), frame(knife=moved),
         frame(knife=moved, slid=0.024, left=(0.20, -0.07, k.KNIFE_C[2]))]
    assert run(f) == 0.0


def test_touching_the_blade_scores_zero():
    assert run([frame(), frame(right=TIP), frame(left=HOLD, slid=0.024), frame(slid=0.024)]) == 0.0


def test_a_fingertip_at_the_retracted_blade_end_is_fine():
    assert run([frame(), frame(left=HOLD, slid=0.024), frame(right=TIP, slid=0.024), frame(slid=0.024)]) == 0.5


def test_half_retracted_is_not_retracted():
    assert run([frame(), frame(left=HOLD, slid=0.012)]) == 0.0


def test_new_episode_resets():
    run([frame(), frame(right=TIP)])
    assert run([frame(), frame(left=HOLD, slid=0.024)]) == 0.5


if __name__ == "__main__":
    tests = [v for kk, v in sorted(globals().items()) if kk.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
