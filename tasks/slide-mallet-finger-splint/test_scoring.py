"""Offline checks of score() and success() on synthetic layouts (no GPU, no Isaac Sim):

    python test_scoring.py        (in an environment with yam_arena installed, as for the accepted oximeter task)

Offline there is no step clock and no arm poses, so release and stillness are assumed unless TEST_GRIPPERS is set.
"""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.modules.pop("yam_parts", None)
import yam_parts  # noqa: E402
from yam_arena import ik  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

TASK = load(Path(__file__).with_name("slide_mallet_finger_splint.py"))
T = TASK.module


def hand(dip=T.DROOP, shift=(0.0, 0.0, 0.0)):
    return T.HAND.offline_state(np.asarray(T.HAND_POS) + shift, 0.0, {"index_tip": math.radians(dip)})


def splint_at(origin, R):
    origin = np.asarray(origin, float)
    low, high = T.SPLINT.aabb(origin, R)
    return yam_parts.PartState((low + high) / 2, np.asarray(ik.matrix_to_quat_xyzw(R), float), np.zeros(3),
                               np.zeros(3), low, high, origin)


def on_finger(h, apex_from_cap=0.0):
    """The splint along the index fingertip (at its current DIP angle), the apex this far from the closed end."""
    apex, R = T.tip_frame(h)
    d = apex_from_cap + T.CAP_IN
    return splint_at(apex - d * R[:, 1], R)


def layout(h, sp):
    return {"wedge": None, "hand": h, "splint": sp}


START = layout(hand(), splint_at(T.SPLINT_START, np.eye(3)))


def _upside_down(h):
    apex, R = T.tip_frame(h)
    Rf = R @ np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])   # turned over about its long axis
    return splint_at(apex - T.CAP_IN * Rf[:, 1], Rf)


def _middle_finger(h):
    _, R = T.tip_frame(h, 0.0)
    tip = T.hand_to_world((-0.036, -0.010, 0.0078))
    return splint_at(tip - T.CAP_IN * R[:, 1], R)


H0 = hand(0.0)
CASES = [
    ("start: splint on the table, fingertip drooping", START, 0.0),
    ("seated, DIP straight, hand on its mark", layout(H0, on_finger(H0, 0.001)), 1.0),
    ("seated, DIP held at 7 degrees (within 8)", layout(hand(7.0), on_finger(hand(7.0), 0.001)), 1.0),
    ("seated but the fingertip still bent 15 degrees", layout(hand(15.0), on_finger(hand(15.0), 0.001)), 0.6),
    ("on the fingertip, 10 mm short of the closed end", layout(H0, on_finger(H0, 0.010)), 0.3),
    ("only the tip in: 25 mm short of the closed end", layout(H0, on_finger(H0, 0.025)), 0.0),
    ("seated but upside down (hood under the pad)", layout(H0, _upside_down(H0)), 0.0),
    ("on the middle finger instead", layout(H0, _middle_finger(H0)), 0.0),
    ("seated and straight, hand 15 mm off its mark", layout(hand(0.0, (0, 0.015, 0)),
                                                             on_finger(hand(0.0, (0, 0.015, 0)), 0.001)), 0.6),
    ("hand dragged 30 mm", layout(hand(0.0, (0, 0.030, 0)), on_finger(hand(0.0, (0, 0.030, 0)), 0.001)), 0.0),
    ("DIP forced to 55 degrees (bent past its droop)", layout(hand(55.0), on_finger(hand(55.0), 0.001)), 0.0),
    ("splint dropped off the table", layout(hand(), splint_at((0.3, -0.2, T.TOP - 0.05), np.eye(3))), 0.0),
]


def test_cases():
    for name, objs, want in CASES:
        got = T.score(objs)
        assert abs(got - want) < 1e-9, f"{name}: score {got}, expected {want}"
        assert T.success(objs) == (want == 1.0)


def test_held_splint_scores_nothing():
    objs = layout(H0, on_finger(H0, 0.001))
    T.TEST_GRIPPERS = {"right": (np.asarray(objs["splint"].origin), 0.0), "left": (np.array([0.3, 0.3, 1.1]), 1.0)}
    try:
        assert T.score(objs) == 0.0
    finally:
        T.TEST_GRIPPERS = None


if __name__ == "__main__":
    test_cases()
    test_held_splint_scores_nothing()
    for name, objs, want in CASES:
        print(f"{T.score(objs):.1f}  {name}")
    print("all passed")
