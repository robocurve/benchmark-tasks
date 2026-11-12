"""Scripted full-score demo of "Make the chainsaw safe to carry."

The left arm grips the hand guard's grip bar from above (the jaws straddle its thickness) and swings the guard forward
(toward the bar tip) along its arc until it snaps over its detent onto the brake stop, then lets go. The right arm
grips the bar cover across its width, 10 cm from its middle toward the closed end, slides it along the bar (from beyond the tip toward the housing)
until it stops 4 mm short of the sprocket cover, and lets go. Both arms go home. The plan is open loop: fixed
waypoints from geometry.py (the layout is fixed, object_jitter=0).

    python demo.py task.py demo_out [--mode MODE] [--max-steps 2400] [--no-record]

Modes: demo (1.0), no_brake (the cover only: 0.5), no_scabbard (the brake only: 0.25), half_scabbard (brake and a cover
slid only 44 percent of the way: 0.25), touch_chain (a gripper lands on the chain first, then everything else: 0.25),
settle (no motion: 0).
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
from demokit import run, wait  # noqa: E402

MODES = ("demo", "no_brake", "no_scabbard", "half_scabbard", "touch_chain", "settle")

HOVER = g.TOP + 0.100                     # tool-point height for travelling (higher is past the arm's joint limits here)
GUARD_GRIP_Z = g.TOP + 0.058              # the pads straddle the grip bar over most of its height
GUARD_SWING = -0.60                       # rad: a little past the brake stop's 0.58, the grip bar is pushed to it
SCAB_GRIP_Z = g.TOP + g.BAR_Z + 0.001     # the pads hold the cover's side walls, clear of the holder under it
SCAB_HOVER = g.TOP + 0.160                # above the cover's top (116 mm) with the pad tips clear of it
SCAB_YAW = -g.S * math.pi / 2             # jaws across the cover's width; this sign keeps the wrist away from its limit
SCAB_GRIP_OFF = 0.100                     # the pads hold the cover this far toward its closed end from its middle (the
                                          # wrist yaw has less range as the slide brings the hand near the arm's front)
SCAB_END_GAP = 0.004                      # the cover stops this far short of the sprocket cover
SCAB_SLIDE_S = 6.0


def grip_rotation(yaw):
    from yam_arena.demo import grip_rotation as gr

    return gr(yaw)


def guard_point(phi: float) -> np.ndarray:
    """World (x, y) of the hand guard's grip-bar centre when the guard is at angle ``phi`` (negative = forward)."""
    px, py = g.GUARD_PIVOT
    r = g.GUARD_LEN - g.GUARD_BAR_LEN / 2
    return np.array([g.SAW_X + px + r * math.cos(phi), g.SAW_Y + g.S * (py + r * math.sin(phi))])


def brake_on(arm):
    """Left arm: grip the guard's grip bar, swing the guard forward along its arc past the detent, let go, retreat."""
    x, y = guard_point(0.0)
    yield from arm.move([x, y, HOVER], grip_rotation(0.0), 2.4, grip=1.0)
    yield from arm.move([x, y, GUARD_GRIP_Z], grip_rotation(0.0), 1.6)
    yield from arm.set_grip(0.0, 0.8)
    steps = 12
    for k in range(1, steps + 1):
        phi = GUARD_SWING * k / steps
        x, y = guard_point(phi)
        yield from arm.move([x, y, GUARD_GRIP_Z], grip_rotation(g.S * phi), 0.22)
    yield from wait(0.4)
    yield from arm.set_grip(1.0, 0.8)
    yield from arm.move([arm.pos[0], arm.pos[1], HOVER], None, 1.0)
    yield from arm.go_home(2.5)


def scabbard_on(arm, fraction: float = 1.0):
    """Right arm: grip the cover across its width 10 cm from its middle toward the closed end and slide it along the bar toward the housing."""
    x0, y0 = g.SCAB_X, g.SCAB_CENTER_Y_START - g.S * SCAB_GRIP_OFF
    y_full = g.SAW_Y - g.S * (SCAB_END_GAP + g.SCAB_LEN / 2 + SCAB_GRIP_OFF)
    y1 = y0 + fraction * (y_full - y0)
    rot = grip_rotation(SCAB_YAW)
    yield from arm.move([x0, y0, SCAB_HOVER], rot, 2.8, grip=1.0)
    yield from arm.move([x0, y0, SCAB_GRIP_Z], rot, 1.6)
    yield from arm.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from arm.move([x0, y1, SCAB_GRIP_Z], rot, SCAB_SLIDE_S * max(fraction, 0.3))
    yield from wait(0.3)
    yield from arm.set_grip(1.0, 0.8)
    yield from arm.move([arm.pos[0], arm.pos[1], SCAB_HOVER], None, 1.0)
    yield from arm.go_home(2.5)


def touch_chain(arm):
    """Right arm: lower an open gripper onto the chain at the bar's tip, then go home."""
    x, y = g.SAW_X, g.SAW_Y - g.S * (g.BAR_LEN - 0.05)
    yield from arm.move([x, y, HOVER], grip_rotation(0.0), 2.4, grip=1.0)
    yield from arm.move([x, y, g.TOP + g.BAR_Z + 0.004], grip_rotation(0.0), 1.6)
    yield from wait(0.5)
    yield from arm.move([x, y, HOVER], None, 1.2)
    yield from arm.go_home(2.0)


def script(ctx, mode: str = "demo"):
    if mode == "settle":
        yield from wait(10.0)
        return
    guard_arm, cover_arm = getattr(ctx, g.GUARD_ARM), getattr(ctx, g.COVER_ARM)
    if mode == "touch_chain":
        yield from touch_chain(cover_arm)
    if mode in ("demo", "no_scabbard", "half_scabbard", "touch_chain"):
        yield from brake_on(guard_arm)
    if mode in ("demo", "no_brake", "touch_chain"):
        yield from scabbard_on(cover_arm)
    if mode == "half_scabbard":
        yield from scabbard_on(cover_arm, fraction=0.44)
    yield from wait(3.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    r = S["result"]
    sc = S.get("scab", [0, 0, 0])
    return (f"guard {math.degrees(S['angle']):6.1f} deg  brake {r.get('brake')}  covered {r.get('covered', 0):.2f} "
            f"aligned {r.get('aligned')}  clear {r.get('arms_clear')}  hold {r.get('hold_steps')}  "
            f"scab x {sc[0]:.3f} y {sc[1]:.3f} z {sc[2] - g.TOP:.4f} tilt {math.degrees(S.get('scab_tilt', 0)):.1f} "
            f"yaw {math.degrees(S.get('scab_yaw', 0)):.1f}  {S['why']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--mode", default="demo", choices=MODES)
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, a.mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=20)
