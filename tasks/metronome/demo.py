"""Scripted full-score demo: the right arm grips the weight from the front and pushes it down the rod to the 120 mark,
closing the loop on the weight's measured height, then lets go and backs off. The left arm takes the rod above the
weight between its fingers, swings it out 25 degrees and backs straight off, so the pendulum swings on its own.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-set]

``--no-set`` starts the metronome the same way without moving the weight: it ticks at 60 (0).
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
from demokit import PAD_TIP, run, turn, wait  # noqa: E402

TILT = math.radians(15)                 # both grippers come in from the front, tilted down, fingers either side (y)
I_TARGET = g.MARKS.index(g.TARGET)
SWING_TO = math.radians(-25)            # how far the left fingers swing the rod out
LAG = 0.005                             # the rod lags the trailing finger by this much
PINCH = 0.15                            # grip with the fingers 7 mm either side of the rod (1 open .. 0 closed)


def reach() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return turn([0, 1, 0], -math.pi / 2 + TILT) @ grip_rotation(0.0)


def rod_point(ctx, d: float, theta: float | None = None) -> np.ndarray:
    """The point on the rod ``d`` above the pivot, at swing ``theta`` (default: where it is now), in the world."""
    s = ctx.states()["metronome"]
    th = s.joints["pendulum"] if theta is None else theta
    local = np.array([g.PIVOT_X, 0.0, g.PIVOT_Z]) + [0.0, -d * math.sin(th), d * math.cos(th)]
    return np.asarray(s.pos, float) + s.rotation() @ local


def set_weight(ctx):
    """Right arm: grip the weight from the front and push it down to the 120 mark."""
    arm, rot = ctx.right, reach()
    a = rot[:, 2]                                               # the approach direction
    reach_in = PAD_TIP - g.WEIGHT_T / 2 + 0.001                 # fingertips 1 mm short of the weight's back
    at = lambda: rod_point(ctx, ctx.states()["metronome"].joints["weight"] - g.WEIGHT_H / 2) - a * reach_in  # noqa: E731
    yield from arm.move(at() - a * 0.08, rot, 2.0, grip=1.0)
    yield from arm.move(at(), rot, 1.0)
    yield from arm.set_grip(0.0, 0.6)
    target = g.MARK_X[I_TARGET]
    for k in range(5):                                          # the wrist is soft: push, measure, correct
        err = ctx.states()["metronome"].joints["weight"] - target
        if abs(err) < 0.0005:
            break
        yield from arm.move(arm.pos - [0, 0, err], None, 1.5 if k == 0 else 0.5)
        yield from wait(0.3)
    yield from arm.set_grip(1.0, 0.5)
    yield from arm.move(arm.pos - a * 0.08, None, 1.0)
    yield from arm.go_home(2.0)


def start(ctx):
    """Left arm: take the rod between the fingers clear of the weight (above it when it is low on the rod, below it
    when it is high), swing it out, and back straight off."""
    arm, rot = ctx.left, reach()
    a = rot[:, 2]
    tip = PAD_TIP - 0.0035                                      # fingertips 3.5 mm past the rod, clear of the case
    top = ctx.states()["metronome"].joints["weight"]
    d = 0.105 if top < 0.09 else top - g.WEIGHT_H - 0.025
    swing = SWING_TO - math.copysign(math.atan(LAG / d), SWING_TO)
    yield from arm.move(rod_point(ctx, d) - a * (tip + 0.08), rot, 2.0, grip=1.0)
    yield from arm.move(rod_point(ctx, d) - a * tip, rot, 1.0)
    yield from arm.set_grip(PINCH, 0.5)
    n = 16
    for k in range(1, n + 1):                                   # along the rod's arc, about 1.2 s
        yield from arm.move(rod_point(ctx, d, swing * k / n) - a * tip, rot, 0.075)
    yield from wait(0.3)
    yield from arm.move(arm.pos - a * 0.06, rot, 0.4)           # off the rod's front: it swings on its own
    yield from arm.go_home(2.0)


def script(ctx, set_tempo: bool = True):
    if set_tempo:
        yield from set_weight(ctx)
    yield from start(ctx)
    yield from wait(6.0)


def log(ctx, step, info) -> str:
    S, task = ctx.task._STATE, ctx.task
    x = ctx.states()["metronome"].joints["weight"]
    t = task.tempo()
    return (f"notch {g.MARKS[S['notch']]:3d}  weight {x * 1000:6.2f} mm  swing {math.degrees(S['swing']):4.1f}  "
            f"theta {math.degrees(S['theta']):5.1f}  running {S["running"]}  ticking {S["ticking"]}  tempo {'-' if t is None else f'{t:.1f}'}  "
            f"tilt {math.degrees(S['tilt']):4.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-set", action="store_true", help="start it without moving the weight")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, set_tempo=not a.no_set), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=10)
