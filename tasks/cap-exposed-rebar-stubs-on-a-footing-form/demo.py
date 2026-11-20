"""Scripted full-score demo of "Make the rebar stubs safe for the crew to walk the formwork."

Each bare stub gets the loose cap that stands in front of it: the arm on that side pinches the cap's raised grip boss (the
fingers close along x), lifts it clear of the bar tips, carries it over the stub, lowers it until the flared mouth finds
the bar, presses it down until the bar is 30 mm up the bore, opens, and goes home. The middle stub is done by the left
arm. The plan is open loop: fixed waypoints from geometry.py (the layout is fixed, object_jitter=0).

    python demo.py task.py demo_out [--mode MODE] [--max-steps 2400] [--no-record]

Modes: demo (1.0), half_press (each cap pressed only until the bar is 14 mm up the bore, then let go: the fins hold it
partway and it does not count), release_above (each cap let go with its mouth 6 mm above the bar tip: it is not pressed
on),  one_cap (only the first stub: 0.25), two_caps (0.50), off_target (the same plan with the hand
shifted 40 mm along x, so every cap arrives 28 to 52 mm off its bar, beyond the 21 mm the flared mouth can catch), stay (all three capped, then the right arm
waits over the form: 0.75), settle (no motion: 0).
Writes one MP4 per camera, setup.png (the first top-camera frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, wait  # noqa: E402

MODES = ("demo", "one_cap", "two_caps", "off_target", "half_press", "release_above", "stay", "settle", "pick_test")

HOVER = g.TOP + 0.100                     # tool-point height for travelling over the loose caps
TIP_MM = float(os.environ.get("REBAR_TIP_MM", g.MOUTH_TO_PAD_TIP * 1000))   # grip height: pad tips above the mouth
GRIP_UP = TIP_MM / 1000 + 0.013           # the tool point sits this far above a held cap's mouth
CARRY_MOUTH = g.BAR_TOP + 0.010           # the held cap's mouth rides this high over the bar tips
FIND_MOUTH = g.BAR_TOP + 0.006            # the mouth hovers here before it is lowered onto the bar
PRESS_BELOW = 0.006                       # the commanded mouth goes this far below the seat: the push that opens the fins
# where the pinch leaves each cap along x relative to the tool point: closing the jaws pushes the cap along x until it
# meets the pad fixed to the hand, so it ends up between 0 and 17 mm to that side (the arm on the left of the table
# holds it toward -x, the right arm toward +x; 17 mm is half the opening left round the 60 mm boss), and which end of
# that range a given pick lands on varied from run to run. The plan shifts the tool point by half the range the other
# way, so the bore axis lands within 8 mm of the bar and the 21 mm of flare catches it. (metres, in pick order)
GRASP_DX = (0.0085, -0.0085, -0.0085)
YAW = {"left": -math.pi / 2, "right": math.pi / 2}   # jaws closing along x; the sign keeps each wrist off its limit


def grip_rotation(yaw):
    from yam_arena.demo import grip_rotation as gr

    return gr(yaw)


def side_for(y: float) -> str:
    return "left" if y >= 0.0 else "right"


def cap_one(arm, side, y_cap: float, y_stub: float, dx: float = 0.0, mouth_to: float | None = None,
            press_below: float = PRESS_BELOW, lift: float = 0.015, leave: bool = True):
    """Pinch the cap standing in front of a stub, carry it over, seat it. ``mouth_to`` is the world height the cap's mouth is
    taken down to before it is let go (None: the seat); ``press_below`` is how far under that the tool point is commanded."""
    rot = grip_rotation(YAW[side])
    mouth0 = g.CAP_START_Z
    yield from arm.move([g.PAD_X, y_cap, HOVER], rot, 2.4, grip=1.0)
    yield from arm.move([g.PAD_X, y_cap, mouth0 + GRIP_UP], rot, 1.6)
    yield from arm.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from arm.move([g.PAD_X, y_cap, CARRY_MOUTH + GRIP_UP], rot, 1.6)
    yield from arm.move([g.FORM_X - dx, y_stub, CARRY_MOUTH + GRIP_UP], rot, 2.4)
    yield from arm.move([g.FORM_X - dx, y_stub, FIND_MOUTH + GRIP_UP], rot, 1.0)
    seat = g.SEATED_Z if mouth_to is None else mouth_to
    yield from arm.move([g.FORM_X - dx, y_stub, seat - press_below + GRIP_UP], rot, 4.0)
    yield from wait(0.6)
    yield from arm.set_grip(1.0, 0.8)
    # lift 15 mm straight up off the plate, then centre the open fingers round the boss (the cap stayed against the fixed
    # pad, so one finger still touches it) before lifting away
    yield from arm.move([arm.pos[0], arm.pos[1], arm.pos[2] + lift], None, 0.6)
    yield from arm.move([g.FORM_X, y_stub, arm.pos[2]], None, 0.8)
    yield from arm.move([arm.pos[0], arm.pos[1], CARRY_MOUTH + GRIP_UP], None, 1.2)
    if leave:
        yield from arm.go_home(2.5)


def script(ctx, mode: str = "demo"):
    if mode == "settle":
        yield from wait(10.0)
        return
    if mode == "pick_test":                           # pick the right-hand cap, lift it clear and hold it: a grasp experiment
        arm, rot = ctx.right, grip_rotation(YAW["right"])
        y = g.CAP_START_Y[0]
        yield from arm.move([g.PAD_X, y, HOVER], rot, 2.4, grip=1.0)
        yield from arm.move([g.PAD_X, y, g.CAP_START_Z + GRIP_UP], rot, 1.6)
        yield from arm.set_grip(0.0, 0.8)
        yield from wait(0.5)
        yield from arm.move([g.PAD_X, y, CARRY_MOUTH + GRIP_UP], rot, 1.6)
        yield from wait(4.0)
        return
    jobs = list(zip(g.CAP_START_Y, [g.STUB_Y[i] for i in g.BARE]))
    if mode == "one_cap":
        jobs = jobs[:1]
    elif mode == "two_caps":
        jobs = jobs[:2]
    for k, (y_cap, y_stub) in enumerate(jobs):
        side = side_for(y_stub) if abs(y_stub) > 1e-6 else "left"
        arm = getattr(ctx, side)
        last = k == len(jobs) - 1
        dx = 0.040 if mode == "off_target" else GRASP_DX[k]
        mouth_to, press_below, lift = None, PRESS_BELOW, 0.015
        if mode == "half_press":                       # pressed only until the bar is 14 mm up the bore, no extra push
            mouth_to, press_below = g.BAR_TOP - 0.014, 0.0
        elif mode == "release_above":                  # let go with the mouth 6 mm above the bar tip: no press at all
            mouth_to, press_below, lift = g.BAR_TOP + 0.006, 0.0, 0.0     # no lift: that height is the arm's reach limit
        yield from cap_one(arm, side, y_cap, y_stub, dx=dx, mouth_to=mouth_to, press_below=press_below, lift=lift,
                           leave=not (mode == "stay" and last))
    yield from wait(4.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    r = S["result"]
    out = []
    for k, (p, q, jw) in S["caps"].items():
        if k.startswith("cap_loose"):
            tilt = math.degrees(ctx.task.logic.tilt(q))
            out.append(f"{k[9:]} x {p[0]:.3f} y {p[1]:+.3f} z {p[2]:.3f} tilt {tilt:4.1f} fins {jw[0] * 1000:.1f} {jw[1] * 1000:.1f}")
    t = S["tcps"]
    return (f"n {r.get('n_capped')} hold {r.get('hold_steps')} | " + " | ".join(out) +
            f" | L {t[0][0]:.3f} {t[0][1]:+.3f} {t[0][2]:.3f}  R {t[1][0]:.3f} {t[1][1]:+.3f} {t[1][2]:.3f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--mode", default="demo", choices=MODES)
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, a.mode), max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=10)
