"""Scripted full-score demo, OPEN LOOP (targets from geometry.py only; object_jitter=0).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--no-button] [--hold]

1. Right arm pinches the flat pack from above across its height at its front end, one pad on the catch's button,
   which the squeeze presses flush; it slides the pack forward off the rail, pushes it 10 cm aside and lets go.
2. Right arm pinches the charged pack the same way and slides it 26 cm along the table onto the rail, 4 mm past
   home (commanded 15 mm past: the stop takes the rest), and lets go: the catch springs up into the notch.

The drill stays put by its own weight and grip on the table (measured: no movement while the packs slide; the
packs ride on the table, not on the rail). ``--hold`` also has the left arm pinch the drill's handle (measured: the
pinch's first finger to touch shoves the drill a few mm, which made the rest miss).
``--no-button`` pinches the flat pack by its back half instead (the catch stays up): the pack must not come off.
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

SEAT = np.array([g.SEAT[0], g.SEAT[1], g.Z_C])
PACK_TOP_Z = g.Z_C + g.W / 2
PINCH_Z = PACK_TOP_Z - 0.020                       # fingertips 2 cm below the pack's upper side
FRONT_CX = g.SEAT[0] + g.BUTTON[0] / 2              # midway between the pack's lower face and the button's top


def tool_for_tip(tip, rot) -> np.ndarray:
    return np.asarray(tip, float) - PAD_TIP * rot[:, 2]


def hold_drill(ctx):
    """Left arm: fingers closing along y across the handle, low on it, fingertips level with its middle."""
    from yam_arena.demo import grip_rotation

    arm = ctx.left
    rot = grip_rotation(0.0)                        # pads close along y (the tool frame's x axis is world -y)
    hx = g.SEAT[0] + g.HANDLE_X[0] + 0.025
    hy = g.SEAT[1] + (g.HANDLE_Y[0] + g.HANDLE_Y[1]) / 2
    tip = np.array([hx, hy, g.Z_C - 0.006])
    yield from arm.move(tool_for_tip(tip + [0, 0, 0.08], rot), rot, 2.5, grip=1.0)
    yield from arm.move(tool_for_tip(tip, rot), None, 1.5)
    yield from arm.set_grip(0.0, 1.0)


def let_go(ctx, arm):
    yield from arm.set_grip(1.0, 0.6)
    yield from arm.move(arm.pos + [0, 0, 0.08], None, 1.0)
    yield from arm.go_home(2.0)


def pinch_front(arm, pack_y: float, button: bool = True):
    """Pads closing along x across the pack's height at its front end (on the button), or (``button=False``) by its
    back half, over the rail walls, well clear of the button. (Measured: a pinch 2 cm behind the button's centre
    still caught its edge and released the catch.)"""
    from yam_arena.demo import grip_rotation

    rot = grip_rotation(math.pi / 2)                # pads close along x
    y = pack_y + (g.BUTTON_Y if button else 0.030)
    cx = FRONT_CX if button else g.SEAT[0] + g.WALL_H / 2
    tip = np.array([cx, y, PINCH_Z])
    yield from arm.move(tool_for_tip(tip + [0, 0, 0.08], rot), rot, 2.5, grip=1.0)
    yield from arm.move(tool_for_tip(tip, rot), None, 1.5)
    yield from arm.set_grip(0.0, 1.2)
    yield from wait(0.5)
    return tip, rot


def pull_flat(ctx, button: bool = True):
    """Right arm: squeeze the button, slide the pack forward (-y) off the rail, then aside (+x), and let go."""
    arm = ctx.right
    tip, rot = yield from pinch_front(arm, g.SEAT[1], button)
    out = tip + [0, -(g.OFF + 0.05), 0]
    yield from arm.move(tool_for_tip(out, rot), None, 3.5)
    yield from arm.move(tool_for_tip(out + [0.10, 0, 0], rot), None, 2.5)
    yield from arm.set_grip(1.0, 0.6)
    yield from arm.move(arm.pos + [0, 0, 0.08], None, 1.0)


def push_charged(ctx):
    """Right arm: pinch the charged pack the same way (the squeeze holds its catch down, which does no harm: it
    springs up into the notch once the pack is home and let go) and slide it along the table onto the rail, 4 mm
    past home. (Measured: pinched by its back half, a pad climbed the 14 mm rail wall during the slide and the
    pack reached the rail 16 mm off line.)"""
    arm = ctx.right
    tip, rot = yield from pinch_front(arm, g.SEAT[1] + g.SPARE_DY)
    home = tip + [0, -g.SPARE_DY, 0]
    yield from arm.move(tool_for_tip(home + [0, -0.02, 0], rot), None, 5.0)
    yield from arm.move(tool_for_tip(home + [0, 0.015, 0], rot), None, 1.5)   # the stop takes it: the arm lags ~8 mm
    yield from wait(0.5)
    yield from let_go(ctx, arm)


def script(ctx, button: bool = True, hold: bool = False):
    if hold:
        yield from hold_drill(ctx)
    yield from pull_flat(ctx, button)
    yield from push_charged(ctx)
    if hold:
        yield from let_go(ctx, ctx.left)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE                                         # logging only
    f, c = S["flat"] or {}, S["charged"] or {}
    fmt = lambda d: " ".join(f"{k} {v * (1 if k == 'twist' else 1000):6.1f}" for k, v in d.items())  # noqa: E731
    d = ctx.states()["drill"]
    q = d.quat_xyzw
    yaw = np.degrees(np.arctan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)))
    return (f"DRILL {np.round((d.pos - SEAT) * 1000, 1)} yaw {yaw:5.2f} FLAT {fmt(f)} | CHARGED {fmt(c)} | "
            f"rel {S['released']} still {S['still']:.1f} outcome {ctx.task.outcome()}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-button", action="store_true", help="pull without pressing the button (must not come off)")
    ap.add_argument("--hold", action="store_true", help="left arm holds the drill's handle too")
    ap.add_argument("--log-every", type=int, default=20)
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, not a.no_button, a.hold), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=a.log_every)
