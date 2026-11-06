"""Scripted full-score demo: the left arm grips the vial low on its body, from the front, and holds it. The right
arm grips the cap from above, pushes it down and, still pushing, turns it anticlockwise until it comes free (the
wrist can turn 200 degrees in one go). It then lifts the cap off and holds it.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-press]

``--no-press`` grips and turns the cap the same way, 200 degrees, without pushing it down: it spins and stays on (0).
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

HOLD_TILT = math.radians(15)           # the left gripper comes in from the front, tilted down, below the cap
TURN_FROM, TURN_TO = math.radians(-65), math.radians(135)     # the right wrist's range for a top grip, here
TURN_STEP = math.radians(10)
PRESS = g.TRAVEL + 0.007               # pushed 7 mm past the cap's 3 mm travel: the wrist is soft


def hold_rotation() -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return turn([0, 1, 0], -math.pi / 2 + HOLD_TILT) @ grip_rotation(0.0)


def hold_bottle(ctx, base):
    """Left arm: come in from the front and grip the vial's body 3 cm up, fingers either side of it."""
    rot = hold_rotation()
    at = base + [0, 0, 0.030]
    yield from ctx.left.move(at - rot[:, 2] * 0.10, rot, 2.0, grip=1.0)
    yield from ctx.left.move(at, rot, 1.0)
    yield from ctx.left.set_grip(0.0, 0.8)


def cap_point(ctx) -> np.ndarray:
    """Where the right tool point grips the cap: fingertips 9 mm below its centre, over the vial where it stands now."""
    x, y, z = ctx.states()["pill_bottle"].pos
    return np.array([x, y, z + g.CAP_C + PAD_TIP - 0.009])


def unscrew(ctx, press: bool = True):
    """Right arm: grip the cap from above, push it down and turn it anticlockwise until it is free."""
    from yam_arena.demo import grip_rotation

    S = ctx.task._STATE
    arm = ctx.right
    cap = cap_point(ctx)
    yield from arm.move(cap + [0, 0, 0.04], grip_rotation(TURN_FROM), 2.0, grip=1.0)
    cap = cap_point(ctx)                                        # the vial settles in the left grip meanwhile
    yield from arm.move(cap, grip_rotation(TURN_FROM), 1.0)
    yield from arm.set_grip(0.0, 0.6)
    down = cap - [0, 0, PRESS if press else 0.0]
    yield from arm.move(down, grip_rotation(TURN_FROM), 0.5)
    yaw = TURN_FROM
    while yaw < TURN_TO and not S["free"]:
        yaw = min(TURN_TO, yaw + TURN_STEP)
        yield from arm.move(down, grip_rotation(yaw), 0.15)
    yield from arm.move(cap, grip_rotation(yaw), 0.3)


def script(ctx, press: bool = True):
    base = np.asarray(ctx.states()["pill_bottle"].pos, float)
    yield from hold_bottle(ctx, base)
    yield from unscrew(ctx, press)
    yield from ctx.right.move(ctx.right.pos + [0, 0, 0.04], None, 1.0)      # lift the cap off
    yield from wait(3.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"unscrewed {math.degrees(S['unscrewed']):5.1f} deg  pressed {S['depth'] * 1000:4.1f} mm  free {S['free']}  "
            f"lift {S['lift'] * 1000:5.1f} mm  off {S['off']}  tilt {math.degrees(S['tilt']):4.1f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-press", action="store_true", help="turn the cap without pressing it")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, press=not a.no_press), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=20)
