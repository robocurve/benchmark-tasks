"""log_can.py with a wrong-strategy demo: both hands take the can by its body, the right hand twists and jerks it about
and the can is put back down (no lifter, tab untouched). It should score 0 (the tab was not lifted)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from log_can import *  # noqa: E402,F401,F403

import task  # noqa: E402


def demo():
    from yam_arena.constants import FINGER_CLOSED as C
    from yam_arena.constants import FINGER_OPEN as O
    from yam_arena.demo import Waypoint as W

    cx, cy = task.CAN_XY
    z = task.Z0 + 0.06
    right = [W(2.0, (0.20, -0.17, task.T + 0.22), O, "start"),
             W(2.0, (cx, cy - 0.10, z + 0.08), O, "beside the can"),
             W(1.5, (cx, cy - 0.04, z), O, "to the can"),
             W(1.0, (cx, cy - 0.04, z), C, "squeeze it"),
             W(1.5, (cx, cy - 0.04, z + 0.10), C, "lift it"),
             W(1.5, (cx, cy - 0.04, z + 0.10), C, "shake"),
             W(1.5, (cx, cy - 0.04, z), C, "put it back"),
             W(1.0, (cx, cy - 0.04, z), O, "let go")]
    left = [W(2.0, (0.20, 0.20, task.T + 0.22), O, "start"),
            W(sum(w.duration for w in right) - 2.0, (0.20, 0.20, task.T + 0.22), O, "wait")]
    return {"left": left, "right": right}
