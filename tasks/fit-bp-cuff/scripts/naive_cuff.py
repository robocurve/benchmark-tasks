"""The task with a wrong-order demo: the hook end is wrapped first and the bladder end laid over it, so the hook tab
ends up under the cuff, against the arm, and cannot fasten. It should score at most 0.25 (wrapped, not fastened)."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import task  # noqa: E402
from task import *  # noqa: E402,F401,F403

_demo = task.demo


def demo():
    from yam_arena.demo import Waypoint as W

    plan = _demo()
    L = [w for w in plan["left"] if w.label != "wait"]
    R = [w for w in plan["right"] if w.label != "wait"]
    tl = sum(w.duration for w in L)
    right = [R[0], W(tl - R[0].duration, R[0].pos, R[0].finger, "wait for the hook end", R[0].yaw)] + R[1:]
    tr = sum(w.duration for w in right)
    left = L + [W(tr - tl, L[-1].pos, L[-1].finger, "wait", L[-1].yaw)]
    return {"left": left, "right": right}
