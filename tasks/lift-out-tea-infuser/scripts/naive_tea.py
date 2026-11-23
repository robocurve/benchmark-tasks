"""The task with a wrong-strategy demo: the right hand sweeps sideways through the teapot's belly at 8 cm and shoves the
whole pot over towards the saucer to get the infuser out. It should score 0 (the pot was shifted).
  cd tasks/lift-out-tea-infuser && inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=scripts/naive_tea.py ..."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import task  # noqa: E402
from task import *  # noqa: E402,F401,F403


def demo():
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    px, py = task.POT_XY
    T = task.T
    right = [W(2.0, (0.20, -0.17, T + 0.22), O, "start"),
             W(2.0, (px, py - 0.17, T + 0.10), C, "beside the pot"),
             W(1.0, (px, py - 0.11, T + 0.08), C, "against its belly"),
             W(4.0, (px, py + 0.12, T + 0.08), C, "shove it over"),
             W(1.5, (px, py + 0.12, T + 0.20), C, "up")]
    left = [W(2.0, (0.20, 0.20, T + 0.22), O, "start"),
            W(sum(w.duration for w in right) - 2.0, (0.20, 0.20, T + 0.22), O, "wait")]
    return {"left": left, "right": right}
