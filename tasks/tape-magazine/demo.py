"""Scripted full-score demo. The slots are filled in order: AI0001L9, AI0002L9, AI0003L9, AI0004L9. Each cartridge
is taken from above by its top edge, carried to its slot, slid about 2 cm in, let go and pushed home with the closed
fingertips. AI0002L9 stands turned round: the right arm lifts it, turns it a quarter turn and stands it on the table;
the left arm takes it, turns it the other quarter, and loads it (a wrist turns only 120 degrees either way, so one
hand cannot turn it round).

    python demo.py task.py demo_out [--max-steps 2400] [--no-record] [--order 3,1,2,4] [--no-turn] [--keep-hold]
                                    [--open-hand]

``--order`` loads the cartridges into slots 1-4 in that order instead (3,1,2,4 is the tote's order: 0).
``--no-turn`` loads AI0002L9 without turning it round (0.25). ``--keep-hold`` keeps the last hand on the last
cartridge (0.75); with ``--open-hand`` that hand opens its fingers wide where it stands (0.75). Writes one MP4 per
camera, setup.png (the first top-camera frame) and result.json.
"""

from __future__ import annotations

import argparse
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import geometry as g  # noqa: E402
from demokit import run, turn, wait  # noqa: E402

PINCH = 0.020             # the tool point this far below a cartridge's top edge
LIFT = 0.06
AROUND = 0.40            # fingers open about 3.8 cm round a 2.15 cm cartridge, clear of its neighbours
PRE_X = 0.298            # the tool point before sliding in: the cartridge between the tote (behind) and the slot
HOUSING = 0.030           # the gripper's housing reaches this far ahead of the tool point (it meets the magazine's top)
PUSH_Z = 0.105            # pushing, the tool point this high: the fingertips on the label's upper half, the housing
                          # above the magazine's top


def down(yaw: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    return grip_rotation(yaw)


def pose(ctx, k: int):
    s = ctx.states()[f"tape_{k}"]
    R, p = s.rotation(), np.asarray(s.pos, float)
    centre = p + R @ np.array([g.TAPE_L / 2, 0.0, 0.0])
    return centre, math.atan2(R[1, 0], R[0, 0]), p[2]


def pick(ctx, arm, k: int, yaw: float):
    """Take cartridge k from above by the middle of its top edge, fingers either side of it, and lift it clear."""
    centre, _, z = pose(ctx, k)
    grasp = centre + [0.0, 0.0, g.TAPE_H - PINCH - (centre[2] - z)]
    yield from arm.move(grasp + [0, 0, 0.06], down(yaw), 1.6, grip=AROUND)
    yield from arm.move(grasp, None, 0.7)
    yield from arm.set_grip(0.0, 0.6)
    yield from arm.move(grasp + [0, 0, LIFT], None, 1.0)


def insert(ctx, arm, k: int, slot: int, stay: bool = False):
    """Carry the held cartridge in front of the slot, slide it in until the gripper's housing nears the magazine's
    top, let go, and push the label face's upper half with the closed fingertips until the cartridge is home."""
    ys = g.slot_y(slot)
    tz = g.TOP + g.MAG_FLOOR + g.TAPE_H - PINCH
    out = g.MAG_X - HOUSING - 0.004                         # the tool point with the housing just outside
    yield from arm.move([PRE_X, ys, tz + 0.06], down(0.0), 1.6)
    yield from arm.move([PRE_X, ys, tz + 0.004], None, 0.6)
    for _ in range(3):              # square it to the slot and line it up across, from its measured pose (it twists
        centre, yaw, z = pose(ctx, k)                       # a few degrees in the pinch, the wrist gives a little)
        err, dy = math.remainder(yaw, math.pi), ys - centre[1]
        dz = g.TOP + g.MAG_FLOOR + 0.004 - z                # its bottom 4 mm above the slot's floor (it slips down)
        if abs(err) < math.radians(0.5) and abs(dy) < 0.0005 and abs(dz) < 0.0005:
            break
        yield from arm.move(arm.pos + [0, dy, dz], turn([0, 0, 1], -err) @ arm.rot, 0.4)
        yield from wait(0.15)
    yield from arm.move([out, arm.pos[1], arm.pos[2]], None, 1.0)
    yield from arm.set_grip(AROUND, 0.4)
    yield from arm.move(arm.pos + [0, 0, 0.04], None, 0.4)
    near = pose(ctx, k)[0][0] - g.TAPE_L / 2
    yield from arm.move([near - 0.035, ys, arm.pos[2]], down(0.0), 0.6)           # back, behind its label face
    yield from arm.move([near - 0.035, ys, g.TOP + PUSH_Z], None, 0.6, grip=0.0)
    home = g.MAG_X + g.SLOT_IN
    for _ in range(4):                                      # push, measure, push the rest (the wrist gives a little)
        near = pose(ctx, k)[0][0] - g.TAPE_L / 2
        if near >= home - 0.001:
            break
        yield from arm.move([arm.pos[0] + (home - near) + 0.004, ys, g.TOP + PUSH_Z], None, 1.0 if near < home - 0.01
                            else 0.4)
        yield from wait(0.2)
    if not stay:
        yield from arm.move([arm.pos[0] - 0.05, ys, arm.pos[2] + 0.06], None, 0.6)


def turn_round(ctx, k: int):
    """Right arm: lift cartridge k, turn it a quarter turn and stand it on the table. Left arm: take it, turn it the
    other quarter turn (label to the arms), and load it."""
    R, L = ctx.right, ctx.left
    yield from pick(ctx, R, k, 0.0)
    stand = np.array([g.TURN_X, g.TURN_Y, g.TOP + g.TAPE_H - PINCH])
    yield from R.move(stand + [0, 0, 0.06], down(math.pi / 2), 2.0)
    yield from R.move(stand + [0, 0, 0.002], None, 0.8)
    yield from R.set_grip(AROUND, 0.5)
    yield from R.move(R.pos + [0, 0, 0.08], None, 0.6)
    yield from R.go_home(1.5)
    _, yaw, _ = pose(ctx, k)                                  # about -90 degrees: the label faces the robot's left
    yield from pick(ctx, L, k, yaw)
    yield from L.move(L.pos, down(0.0), 1.2)                  # the other quarter turn: label to the arms


def script(ctx, order=(1, 2, 3, 4), turn=True, let_go=True, open_hand=False):
    for slot, k in enumerate(order, start=1):
        bay = g.TOTE_ORDER.index(k)
        arm = ctx.left if g.BAY_Y[bay] > 0 else ctx.right
        if k == g.TURNED and turn:
            yield from turn_round(ctx, k)
            arm = ctx.left
        else:
            yield from pick(ctx, arm, k, 0.0)
        last_hold = slot == len(order) and not let_go
        yield from insert(ctx, arm, k, slot, stay=last_hold)
        if last_hold:                                       # the fingertips stay on the last label face
            if open_hand:
                yield from arm.set_grip(1.0, 0.6)           # opened wide, where it stands
            yield from wait(6.0)
            return
        yield from arm.go_home(1.5)
    yield from wait(2.0)


def log(ctx, step, info) -> str:
    S = ctx.task._STATE
    home = g.MAG_X + g.SLOT_IN
    parts = [f"{n}: slot {t['slot']} out {t['label_out']} home {t['home']} ({(home - t['near']) * 1000:.0f} mm short)"
             for n, t in S["tapes"].items()]
    return " | ".join(parts) + f" | arms {S['clearance'] * 100:.1f} cm off"


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--order", default="1,2,3,4", help="the cartridges for slots 1-4, e.g. 3,1,2,4")
    ap.add_argument("--no-turn", action="store_true", help="load AI0002L9 without turning it round")
    ap.add_argument("--keep-hold", action="store_true", help="keep a hand on the last cartridge")
    ap.add_argument("--open-hand", action="store_true", help="with --keep-hold, open that hand's fingers wide")
    a = ap.parse_args()
    order = tuple(int(v) for v in a.order.split(","))
    run(a.task, a.out, lambda ctx: script(ctx, order, turn=not a.no_turn, let_go=not a.keep_hold,
                                            open_hand=a.open_hand),
        max_steps=a.max_steps, record=not a.no_record, log=log, log_every=20)
