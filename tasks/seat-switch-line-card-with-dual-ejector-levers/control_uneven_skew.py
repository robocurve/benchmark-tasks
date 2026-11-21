"""Negative control: left lever operated alone, causing excessive skew (>1.5 mm) while mating."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from demo import Arm, CAL, CONTROL_HZ, TIPS, both, card_x, run, script, tab, wait


def script_skew(task, ctx):
    from yam_arena.demo import grip_rotation

    T = task
    L, R = ctx.left, ctx.right
    top = T.TOP
    face_z = top + T.FLOOR + 0.008 + TIPS
    tab_z = top + T.FLOOR + 0.047 + TIPS
    high = top + 0.13
    push_l = grip_rotation(-math.pi / 2)
    push_r = grip_rotation(math.pi / 2)
    pad = 0.0064
    start_face = T.SEAT_X - T.START_OUT

    # 1. Push in faceplate
    ys = CAL["push_y"]
    x_push = T.SEAT_X - CAL["push_gap"] - pad
    yield from both(L.move(np.array([start_face - 0.04, ys, high]), push_l, 2.0, grip=0.0),
                    R.move(np.array([start_face - 0.04, -ys, high]), push_r, 2.0, grip=0.0))
    yield from both(L.move(np.array([start_face - 0.02 - pad, ys, face_z]), push_l, 1.5),
                    R.move(np.array([start_face - 0.02 - pad, -ys, face_z]), push_r, 1.5))
    yield from both(L.move(np.array([x_push, ys, face_z]), push_l, 3.0),
                    R.move(np.array([x_push, -ys, face_z]), push_r, 3.0))
    yield from wait(0.5)
    yield from both(L.move(np.array([x_push - 0.04, ys, high]), push_l, 1.2),
                    R.move(np.array([x_push - 0.04, -ys, high]), push_r, 1.2))

    # 2. Only left arm approaches and grips left lever
    a0 = T.OPEN_DEG
    at = T.SEAT_X - CAL["gap_after_push"]
    (pl, rl, _), _ = tab(T, 1, a0, tab_z, math.pi, at), tab(T, -1, a0, tab_z, 0.0, at)
    up = np.array([0, 0, high - tab_z])
    yield from L.move(pl + up, rl, 1.5, grip=0.45)
    yield from L.move(pl, rl, 1.2)
    yield from L.set_grip(0.0, 0.8)
    yield from wait(0.3)

    # 3. Only left lever swung shut (causes cocking and trips skew latch)
    angles = np.linspace(a0, -CAL["shut_past"], 80)
    path_l = [tab(T, 1, a, tab_z, math.pi, at)[:2] for a in angles]
    yield from L.path(path_l, 6.0)
    yield from wait(1.0)


if __name__ == "__main__":
    import demo
    demo.script = script_skew
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    demo.run(a.task, a.out, a.max_steps, record=not a.no_record)
