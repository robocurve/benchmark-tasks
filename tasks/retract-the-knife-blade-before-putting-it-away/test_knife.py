"""Scorer tests for knife.py, run without the simulator:  python test_knife.py"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from yam_arena.task_file import ObjectState  # noqa: E402

import knife as k  # noqa: E402

AWAY_L, AWAY_R = (0.15, 0.30, k.TOP + 0.14), (0.15, -0.28, k.TOP + 0.14)
IN_TRAY = (k.TRAY_C[0], k.TRAY_C[1], k.TOP + 0.004 + k.HANDLE_SIZE[2] / 2)
FINGER_HALF = 0.015  # each finger's centre line this far either side of the TCP (along x here)
DOWN = np.array([1.0, 0.0, 0.0, 0.0])  # gripper pointing down: its +Z (approach) along world -Z
FULL = 0.024  # the button's full travel: the tip 4 mm inside the handle


def _state(pos, size, moving=False, quat=(0.0, 0.0, 0.0, 1.0)):
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    v = np.array([0.0, 0.1, 0.0]) if moving else np.zeros(3)
    return ObjectState(pos, np.asarray(quat, float), v, np.zeros(3), pos - half, pos + half)


def _arm(name, tcp, out, finger_at=None):
    tcp = np.asarray(tcp, float)
    out[f"{name}_tcp"] = _state(tcp, (0.02, 0.02, 0.02), quat=DOWN)
    for side, s in (("l", 1.0), ("r", -1.0)):
        p = tcp + np.array([s * FINGER_HALF, 0.0, 0.0])
        if finger_at is not None and side == "l":
            p = np.asarray(finger_at, float)
        out[f"{name}_finger_{side}"] = _state(p, (0.02, 0.02, 0.02), quat=DOWN)


def frame(knife=k.KNIFE_C, slid=0.0, left=AWAY_L, right=AWAY_R, moving=False, right_finger=None):
    """slid: how far the button has been slid back (0 = blade out 2 cm; 0.020 = tip flush; 0.024 = fully in)."""
    kp = np.asarray(knife, float)
    if moving:
        kp = kp + np.random.uniform(-0.01, 0.01, 3)
    bp = kp + (np.asarray(k.BLADE_C) - np.asarray(k.KNIFE_C)) + np.array([0.0, slid, 0.0])
    out = {"knife": _state(kp, k.HANDLE_SIZE, moving), "blade": _state(bp, k.BLADE_SIZE, moving)}
    _arm("left", left, out)
    _arm("right", right, out, finger_at=right_finger)
    return out


class FakeSim:
    def __init__(self, t, states):
        self.t, self.states = t, states


def run(frames):
    out = None
    for i, f in enumerate(frames):
        k._new_episode(FakeSim((i + 1) / k.CONTROL_HZ, f))
        out = k.score(f)
    return out


def still(n=25, **kw):
    return [frame(**kw)] * n


HOLD = (k.KNIFE_C[0], k.KNIFE_C[1] + 0.07, k.KNIFE_C[2])
TIP = (k.KNIFE_C[0], k.FRONT_Y - 0.01, k.KNIFE_C[2])  # at the exposed blade (2 cm out)
FRONT = (k.KNIFE_C[0], k.FRONT_Y - 0.004, k.KNIFE_C[2])  # 4 mm in front of the handle's blade end


def stowed(slid=FULL):
    """Retract, carry to the tray, let go, and leave it there for a second."""
    return ([frame()] + [frame(left=HOLD, slid=slid)] + [frame(knife=IN_TRAY, slid=slid, left=IN_TRAY, moving=True)] * 3
            + still(knife=IN_TRAY, slid=slid))


def test_nothing_done_scores_zero():
    assert run(still()) == 0.0


def test_retracted_and_in_the_tray_scores_one():
    assert run(stowed()) == 1.0


def test_in_the_tray_but_not_yet_still_for_a_second_scores_half():
    f = stowed()
    assert run(f[:-25] + f[-5:]) == 0.5


def test_in_the_tray_but_still_held_scores_half():
    f = stowed()[:-25] + still(knife=IN_TRAY, slid=FULL, left=IN_TRAY)
    assert run(f) == 0.5


def test_retracted_on_the_mat_scores_half():
    assert run([frame(), frame(left=HOLD, slid=0.012)] + still(slid=FULL)) == 0.5


def test_in_the_tray_with_the_blade_out_scores_zero():
    assert run([frame(), frame(knife=IN_TRAY, left=IN_TRAY, moving=True)] + still(knife=IN_TRAY)
               + still(knife=IN_TRAY, slid=FULL)) == 0.0


def test_in_the_tray_with_2_mm_of_blade_out_scores_zero():
    assert run(stowed(slid=0.018)) == 0.0


def test_in_the_tray_with_the_tip_flush_scores_zero():
    assert run(stowed(slid=0.020)) == 0.0


def test_tip_just_inside_counts_as_retracted():
    assert run(stowed(slid=0.0206)) == 1.0


def test_put_down_elsewhere_with_the_blade_out_scores_zero():
    moved = (0.20, -0.10, k.KNIFE_C[2])
    f = [frame(), frame(knife=moved, left=(0.20, -0.07, k.KNIFE_C[2]), moving=True)] + still(knife=moved)
    assert run(f + still(knife=moved, slid=FULL)) == 0.0


def test_touching_the_blade_scores_zero():
    assert run([frame(), frame(right=TIP), frame(left=HOLD, slid=FULL)] + still(slid=FULL)) == 0.0


def test_one_finger_at_the_blade_scores_zero_even_with_the_midpoint_away():
    assert run([frame(), frame(right_finger=TIP)] + stowed()[1:]) == 0.0


def test_a_finger_at_a_partly_exposed_tip_scores_zero():
    tip_2mm = (k.KNIFE_C[0], k.FRONT_Y - 0.002, k.KNIFE_C[2])
    assert run([frame(), frame(slid=0.018, right_finger=tip_2mm)] + stowed()[1:]) == 0.0


def test_a_finger_at_the_flush_tip_scores_zero():
    assert run([frame(), frame(slid=0.020, right_finger=FRONT)] + stowed()[1:]) == 0.0


def test_a_finger_at_the_handle_front_after_retracting_is_fine():
    assert run([frame(), frame(left=HOLD, slid=FULL), frame(slid=FULL, right_finger=FRONT)] + stowed(slid=FULL)[1:]) == 1.0


def test_half_retracted_is_not_retracted():
    assert run([frame(), frame(left=HOLD, slid=0.012)]) == 0.0


def test_new_episode_resets():
    run([frame(), frame(right=TIP)])
    assert run(stowed()) == 1.0


if __name__ == "__main__":
    tests = [v for kk, v in sorted(globals().items()) if kk.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print(f"{len(tests)} passed")
