"""Negative control: card pushed to pins and partially engaged, but levers not shut into catches."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
from demo import Arm, CAL, CONTROL_HZ, TIPS, both, card_x, run, script, tab, wait


def script_unlatched(task, ctx):
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

    # 1. Push in faceplate to pins
    ys = CAL["push_y"]
    x_push = T.SEAT_X - CAL["push_gap"] - pad
    yield from both(L.move(np.array([start_face - 0.04, ys, high]), push_l, 2.0, grip=0.0),
                    R.move(np.array([start_face - 0.04, -ys, high]), push_r, 2.0, grip=0.0))
    yield from both(L.move(np.array([start_face - 0.02 - pad, ys, face_z]), push_l, 1.5),
                    R.move(np.array([start_face - 0.02 - pad, -ys, face_z]), push_r, 1.5))
    yield from both(L.move(np.array([x_push, ys, face_z]), push_l, 3.0),
                    R.move(np.array([x_push, -ys, face_z]), push_r, 3.0))
    yield from wait(0.5)
    # Stop and back away without engaging levers shut
    yield from both(L.move(np.array([x_push - 0.04, ys, high]), push_l, 1.2),
                    R.move(np.array([x_push - 0.04, -ys, high]), push_r, 1.2))
    yield from wait(3.0)


if __name__ == "__main__":
    import demo
    demo.script = script_unlatched
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=1200)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    demo.run(a.task, a.out, a.max_steps, record=not a.no_record)
