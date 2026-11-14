"""Scripted full-score demo of "Make the rebar stubs safe for the crew to walk the formwork."

Each bare stub gets the loose cap that stands in front of it: the arm on that side pinches the cap's square head (the
fingers close along x), lifts it clear of the bar tips, carries it over the stub, lowers it until the flared mouth finds
the bar, presses it down until the bar is 30 mm up the bore, opens, and goes home. The middle stub is done by the left
arm. The plan is open loop: fixed waypoints from geometry.py (the layout is fixed, object_jitter=0).

    python demo.py task.py demo_out [--mode MODE] [--max-steps 2400] [--no-record]

Modes: demo (1.0), one_cap (only the first stub: 0.25), two_caps (0.50), off_target (the same plan with the hand
shifted 40 mm along x, so every cap arrives 28 to 52 mm off its bar, beyond the 21 mm the flared mouth can catch), stay (all three capped, then the right arm
waits over the form: 0.75), settle (no motion: 0).
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

MODES = ("demo", "one_cap", "two_caps", "off_target", "stay", "settle")

HOVER = g.TOP + 0.100                     # tool-point height for travelling over the loose caps
GRIP_UP = g.MOUTH_TO_PAD_TIP + 0.013      # the tool point sits this far above a held cap's mouth
CARRY_MOUTH = g.BAR_TOP + 0.010           # the held cap's mouth rides this high over the bar tips
FIND_MOUTH = g.BAR_TOP + 0.006            # the mouth hovers here before it is lowered onto the bar
PRESS_BELOW = 0.003                       # the commanded mouth goes this far below the seat (the arm tracks low)
# where the pinch leaves each cap along x relative to the tool point: closing the jaws pushes the cap along x until it
# meets the pad fixed to the hand, so it ends up between 0 and 12 mm to that side (the arm on the left of the table
# holds it toward -x, the right arm toward +x; 12 mm is half the opening left round the 70 mm head), and which end of
# that range a given pick lands on varied from run to run. The plan shifts the tool point by half the range the other
# way, so the bore axis lands within 6 mm of the bar and the 21 mm of flare catches it. (metres, in pick order)
GRASP_DX = (0.006, -0.006, -0.006)
YAW = {"left": -math.pi / 2, "right": math.pi / 2}   # jaws closing along x; the sign keeps each wrist off its limit


def grip_rotation(yaw):
    from yam_arena.demo import grip_rotation as gr

    return gr(yaw)


def side_for(y: float) -> str:
    return "left" if y >= 0.0 else "right"


def cap_one(arm, side, y_cap: float, y_stub: float, dx: float = 0.0, press_depth: float | None = None, leave: bool = True):
    """Pinch the cap standing in front of a stub, carry it over, seat it. ``press_depth`` is how far the mouth is pressed
    below where it first meets the bar tip (None: all the way to the seat)."""
    rot = grip_rotation(YAW[side])
    mouth0 = g.CAP_START_Z
    yield from arm.move([g.PAD_X, y_cap, HOVER], rot, 2.4, grip=1.0)
    yield from arm.move([g.PAD_X, y_cap, mouth0 + GRIP_UP], rot, 1.6)
    yield from arm.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from arm.move([g.PAD_X, y_cap, CARRY_MOUTH + GRIP_UP], rot, 1.6)
    yield from arm.move([g.FORM_X - dx, y_stub, CARRY_MOUTH + GRIP_UP], rot, 2.4)
    yield from arm.move([g.FORM_X - dx, y_stub, FIND_MOUTH + GRIP_UP], rot, 1.0)
    seat = g.SEATED_Z if press_depth is None else g.BAR_TOP - press_depth
    yield from arm.move([g.FORM_X - dx, y_stub, seat - PRESS_BELOW + GRIP_UP], rot, 4.0)
    yield from wait(0.6)
    yield from arm.set_grip(1.0, 0.8)
    # the cap stayed against the fixed pad, so the open fingers still touch its head: centre them round it before lifting
    yield from arm.move([g.FORM_X, y_stub, arm.pos[2]], None, 0.8)
    yield from arm.move([arm.pos[0], arm.pos[1], CARRY_MOUTH + GRIP_UP], None, 1.2)
    if leave:
        yield from arm.go_home(2.5)


def script(ctx, mode: str = "demo"):
    if mode == "settle":
        yield from wait(10.0)
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
        yield from cap_one(arm, side, y_cap, y_stub, dx=dx, press_depth=None, leave=not (mode == "stay" and last))
    yield from wait(4.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    r = S["result"]
    out = []
    for k, (p, q) in S["caps"].items():
        if k.startswith("cap_loose"):
            tilt = math.degrees(ctx.task.logic.tilt(q))
            out.append(f"{k[9:]} x {p[0]:.3f} y {p[1]:+.3f} z {p[2]:.3f} tilt {tilt:4.1f}")
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
