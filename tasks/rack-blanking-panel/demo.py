"""Scripted full-score demo: both arms come in from the front and pinch the panel's top and bottom edges, one hand
either side of its middle, lift it out of the stand and carry it to the rack at U6's middle hole. They line the
clips up with the holes from the panel's measured pose, push both ends home together, let go and back off.

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--hole 18] [--keep-hold]

``--hole N`` fits it with its clips in hole N instead (18, U6's top hole, leaves it straddling U6 and U7: 0).
``--keep-hold`` fits it at U6 but never lets go (0.5). Writes one MP4 per camera, setup.png (the first top-camera
frame) and result.json to the output folder.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, together, turn, wait  # noqa: E402

GY = 0.10                                 # the hands pinch the panel this far either side of its middle
LIFT = 0.045                              # out of the stand's slots (26.5 mm deep)
STANDOFF = 0.03                           # the panel's back this far in front of the posts before pushing home


def forward(side: str) -> np.ndarray:
    """The gripper pointing forward (+x), fingers closing along z, so they pinch the panel's top and bottom edges."""
    from yam_arena.demo import grip_rotation

    return turn([0, 1, 0], -math.pi / 2) @ grip_rotation(math.pi / 2 if side == "left" else -math.pi / 2)


def panel_ends(ctx):
    """Where each clip is (world): left and right, from the panel's measured pose."""
    s = ctx.states()["panel"]
    R, p = s.rotation(), np.asarray(s.pos, float)
    return {k: p + R @ np.array([0.0, sy * g.PY, 0.0]) for k, sy in (("left", 1), ("right", -1))}


def script(ctx, hole: int = g.TARGET_HOLE, let_go: bool = True):
    L, R = ctx.left, ctx.right
    rl, rr = forward("left"), forward("right")
    x_mid = g.STAND_X - g.PANEL_T / 2                         # the tool point at the middle of the panel's thickness
    z0 = g.TOP + g.PANEL_START_Z

    # 1. in front of the panel, fingers open above and below it; slide in and pinch its edges; lift it out
    yield from together(L.move([x_mid - 0.07, GY, z0], rl, 2.5, grip=1.0), R.move([x_mid - 0.07, -GY, z0], rr, 2.5, grip=1.0))
    yield from together(L.move([x_mid, GY, z0], None, 1.0), R.move([x_mid, -GY, z0], None, 1.0))
    yield from together(L.set_grip(0.0, 0.8), R.set_grip(0.0, 0.8))
    yield from together(L.move(L.pos + [0, 0, LIFT], None, 1.0), R.move(R.pos + [0, 0, LIFT], None, 1.0))

    # 2. carry it to the rack: clips level with the hole, the panel's back STANDOFF in front of the posts
    zt = g.TOP + g.hole_z(hole)
    above = (L.pos[2] + R.pos[2]) / 2 - float(ctx.states()["panel"].pos[2])   # tool points above the panel's middle
    xt = g.RX - STANDOFF - g.PANEL_T / 2
    yield from together(L.move([xt, GY, zt + above], None, 2.5), R.move([xt, -GY, zt + above], None, 2.5))
    yield from wait(0.3)

    # 3. line the clips up with the holes from the panel's measured pose (the pinch lets it slip a little)
    for _ in range(4):
        e = panel_ends(ctx)
        dz = {k: zt - e[k][2] for k in e}
        dy = np.mean([g.PY - e["left"][1], -g.PY - e["right"][1]])
        if max(abs(v) for v in dz.values()) < 0.0004 and abs(dy) < 0.0004:
            break
        mid, roll = (dz["left"] + dz["right"]) / 2, (dz["left"] - dz["right"]) / 2 * GY / g.PY
        yield from together(L.move(L.pos + [0, dy, mid + roll], None, 0.4), R.move(R.pos + [0, dy, mid - roll], None, 0.4))
        yield from wait(0.2)

    # 4. push both ends home together, let go, back off
    push = STANDOFF + 0.002                                     # 2 mm past contact, against the wrist's give
    yield from together(L.move(L.pos + [push, 0, 0], None, 1.6), R.move(R.pos + [push, 0, 0], None, 1.6))
    yield from wait(0.3)
    if not let_go:
        yield from wait(8.0)
        return
    yield from together(L.set_grip(1.0, 0.6), R.set_grip(1.0, 0.6))
    yield from together(L.move(L.pos - [0.08, 0, 0], None, 1.0), R.move(R.pos - [0.08, 0, 0], None, 1.0))
    yield from together(L.go_home(2.0), R.go_home(2.0))
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    e = S.get("ends") or {}
    parts = [f"{k}: hole {v['hole']} gap {v['gap'] * 1000:5.1f} mm home {v['home']}" for k, v in e.items()]
    return " | ".join(parts) + f" | arms {S.get('clearance', 0) * 100:.1f} cm off, clear steps {S.get('clear_steps')}"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--hole", type=int, default=g.TARGET_HOLE, help="fit the clips in this hole (1-24)")
    ap.add_argument("--keep-hold", action="store_true", help="never let go")
    a = ap.parse_args()
    run(a.task, a.out, lambda ctx: script(ctx, hole=a.hole, let_go=not a.keep_hold), max_steps=a.max_steps,
        record=not a.no_record, log=log, log_every=10)
