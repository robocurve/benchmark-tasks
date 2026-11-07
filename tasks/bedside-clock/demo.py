"""Scripted full-score demo: the left arm presses HOUR as many times as the analog clock's time needs and goes back to
its start pose; then the right arm does the same with MIN. One arm at a time, so the two stay clear of each other over
the clock. Then the count is checked against the clock, and any press that didn't count is made again.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--misread]

``--misread`` reads the analog clock's hour hand as 5 (one HOUR press too many): the clock ends at 5:40, which scores 0.5.
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import PAD_TIP, run, wait  # noqa: E402

ARM = {"hour": "left", "min": "right"}
YAW = {"left": -math.pi / 2, "right": math.pi / 2}      # fingers along x: the two grippers stay clear of each other


def presses(ctx, name: str, n: int):
    """Close the gripper, then press a button n times with the fingertips: down past its travel, and back up."""
    from yam_arena.demo import grip_rotation

    arm = getattr(ctx, ARM[name])
    rot = grip_rotation(YAW[ARM[name]])
    x, y = g.BUTTON_X, g.BUTTON_Y[name]
    above = np.array([x, y, g.TOP + g.CAP_TOP + PAD_TIP + 0.03])
    down = np.array([x, y, g.TOP + g.CAP_TOP - g.TRAVEL - 0.004 + PAD_TIP])    # 4 mm past the travel: the wrist is soft
    if np.linalg.norm(arm.pos - above) > 0.005:
        yield from arm.move(above, rot, 1.5, grip=0.0)
    for _ in range(n):
        yield from arm.move(down, rot, 0.3)
        yield from wait(0.15)
        yield from arm.move(above, rot, 0.3)
        yield from wait(0.1)


def presses_then_home(ctx, name: str, n: int):
    """Press a button n times, then take that arm back to its start pose."""
    if n:
        yield from presses(ctx, name, n)
        yield from getattr(ctx, ARM[name]).go_home(2.0)


def script(ctx, hour_target: int = g.TARGET[0]):
    S = ctx.task._STATE
    for _ in range(4):                  # the presses, then up to 3 checks for any press that didn't count
        need_h = (hour_target - S["hour"]) % 12
        need_m = (g.TARGET[1] - S["minute"]) % 60
        if not need_h and not need_m:
            break
        yield from presses_then_home(ctx, "hour", need_h)     # one arm at a time: they stay clear of each other
        yield from presses_then_home(ctx, "min", need_m)
        yield from wait(0.3)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return f"clock {S['hour']}:{S['minute']:02d} presses {S['presses']}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--misread", action="store_true", help="read the hour hand as 5")
    a = ap.parse_args()
    run(a.task, a.out, (lambda ctx: script(ctx, 5)) if a.misread else script, max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=40)
