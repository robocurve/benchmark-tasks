"""Scripted full-score demo: the left arm first puts the knife in the tray, then the right arm opens the valve three
turns by its spinner knob, and both arms go home.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--naive]

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
from demokit import run, together, wait  # noqa: E402

KNOB_TCP = 0.17          # m above the table: the grasp point on the knob (fingertips 13 mm lower, above the rim)
TURN_S = 12.0            # seconds for the three turns


def knob_at(psi: float) -> np.ndarray:
    """The grasp point on the knob for a handwheel angle (world frame)."""
    x, y = g.knob_xy(psi)
    return np.array([x, y, g.TOP + KNOB_TCP])


def stow_knife(ctx):
    """Left arm: pinch the handle across its width, lift, lay the knife in the tray, let go."""
    from yam_arena.demo import grip_rotation

    L, down = ctx.left, grip_rotation(-math.pi / 2)     # fingers close along world x
    knife = ctx.states()["knife"].pos.copy()             # the handle's centre
    board_top = g.TOP + g.HINGE_Z + g.BOARD_T / 2
    grasp = np.array([knife[0], knife[1], board_top + 0.017])
    yield from L.move(grasp + [0, 0, 0.05], down, 2.0, grip=1.0)
    yield from L.move(grasp, down, 1.0)
    yield from L.set_grip(0.0, 0.8)
    yield from wait(0.3)
    yield from L.move(grasp + [0, 0, 0.06], down, 0.8)
    place = np.array([g.TRAY_XY[0], g.TRAY_XY[1] + 0.07, g.TOP + g.TRAY_FLOOR + 0.012 + 0.009])
    yield from L.move(place + [0, 0, 0.08], down, 2.0)
    yield from L.move(place, down, 1.0)
    yield from L.set_grip(1.0, 0.6)
    yield from wait(0.3)
    yield from L.move(place + [0, 0, 0.10], down, 0.8)


def open_valve(ctx, turns: float = 3.0):
    """Right arm: grip the knob and drive it round the wheel's axis; the knob spins freely, so the grip never turns.
    Closes the loop on the measured turns at the end."""
    from yam_arena import ik
    from yam_arena.demo import grip_rotation

    R, rot = ctx.right, grip_rotation(math.pi / 2)
    psi0 = ctx.states()["valve"].joints.get("wheel", 0.0)
    yield from R.move(knob_at(psi0) + [-0.02, 0, 0.04], rot, 2.0, grip=1.0)
    yield from R.move(knob_at(psi0), rot, 1.0)
    yield from R.set_grip(0.0, 0.6)
    yield from wait(0.3)
    total = turns * 2 * math.pi
    n = round(TURN_S * 20)
    for k in range(1, n + 1):
        R.aim(knob_at(psi0 + total * ik.smoothstep(k / n)), rot)
        yield
    cmd = psi0 + total
    for _ in range(60):
        err = (turns - ctx.task._STATE["turns"]) * 2 * math.pi
        if abs(err) < math.radians(3):
            break
        cmd += 0.5 * err
        R.aim(knob_at(cmd), rot)
        yield
    yield from R.set_grip(1.0, 0.5)
    yield from wait(0.2)
    yield from R.move(R.pos + [-0.05, 0, 0.04], rot, 0.8)
    yield from R.go_home(2.5)


def script(ctx):
    yield from stow_knife(ctx)
    yield from together(ctx.left.go_home(2.5), open_valve(ctx))


def naive(ctx):
    """What the instruction literally asks for, and nothing else: the knife slides onto the doll (scores 0)."""
    yield from open_valve(ctx)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    return (f"turns {S['turns']:.2f} tilt {math.degrees(S['tilt']):.1f} on_board {S['on_board']} held {S['held']} "
            f"safe {S['safe']} clear {S['clear']} dist {S['dist']:.3f} why {S['why']}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--naive", action="store_true", help="only open the valve, leaving the knife on the board")
    a = ap.parse_args()
    run(a.task, a.out, naive if a.naive else script, max_steps=a.max_steps, record=not a.no_record, log=log,
        log_every=40)
