"""Near-successes and unintended strategies, scored by the offline checker. Each case: (label, {object: spec},
expected score, expected success); a spec is a ready state (Part.state(...)). Cases run in order and share the
whole-trial record, cleared only by a scene reset (the board back at its start by a jump of over 20 mm).

Offline there is no robot and no physics: the arms count as measured clear unless a case marks them otherwise (in the
simulator they are measured every step, and are None, no credit, until then). Cases are scored in lot A.
"""

import dataclasses
import math

import numpy as np

import lots
import parts
import task
from task import BOARD, BX, BY, SEAT_Z, STILL_STEPS

lots.use("A")


def _offline_clear(objects):
    b = objects.get("server_board")
    pts = getattr(b, "_offline_points", None)
    if pts is not None:
        return task._arms_clear(pts, task.board_boxes(b))
    return getattr(b, "_offline_clear", True) if b is not None else True


parts.__dict__.setdefault("_itx_task_grip", {})["offline_clear"] = _offline_clear


def _quat(yaw_deg=0.0, roll_deg=0.0, pitch_deg=0.0):
    r = (parts._rot((0, 0, 1), math.radians(yaw_deg)) @ parts._rot((1, 0, 0), math.radians(roll_deg))
         @ parts._rot((0, 1, 0), math.radians(pitch_deg)))
    return tuple(float(v) for v in parts._matrix_to_quat(r))


def board(dx=0.0, dy=0.0, dz=0.0, yaw=0.0, roll=0.0, pitch=0.0, vz=0.0, **mark):
    """The board dx, dy from home (m), dz above the standoff tops, turned and tilted (deg), moving down at -vz.
    Keyword marks set offline stand-ins (clear=, points=)."""
    st = BOARD.state(pos=(BX + dx, BY + dy, SEAT_Z + dz), quat_xyzw=_quat(yaw, roll, pitch), lin_vel=(0.0, 0.0, vz))
    for k, v in mark.items():
        object.__setattr__(st, f"_offline_{k}", v)
    return {"server_board": st}


def start(**mark):
    st = BOARD.state(lin_vel=(0.0, 0.0, 0.0))
    for k, v in mark.items():
        object.__setattr__(st, f"_offline_{k}", v)
    return {"server_board": st}


def held(label, state, want):
    """A state reached and held still 1 s, then one more step: credit needs the hold."""
    moving = dict(state)
    b = state["server_board"]
    moving["server_board"] = dataclasses.replace(b, pos=tuple(np.asarray(b.pos) + [0, 0, 0.001]))
    for k, v in vars(b).items():
        if k.startswith("_offline_"):
            object.__setattr__(moving["server_board"], k, v)
    out = [(f"{label}: still moving", moving, 0.0, False)]
    out += [(f"{label} (still, step {k + 1})", state, 0.0, False) for k in range(STILL_STEPS)]
    out.append((f"{label} (still 1 s)", state, want, False))
    out.append((f"{label} (still 1 s + 1 step)", state, want, want == 1.0))
    return out


def reset():
    """A scene reset: far away, then back at the start in one step."""
    return [("away before a reset", board(dx=-0.1, dz=0.1), 0.0, False),
            ("scene reset", start(), 0.0, False)]


def plate(gap):
    """A flat arm face 120 x 120 mm lying level `gap` above the seated board's heat sink: every corner is about 60 mm
    from the sink's top centre, only the face itself is near."""
    h = np.array([0.060, 0.060, 0.001])
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * h
    planes = np.array([[1, 0, 0, h[0]], [-1, 0, 0, h[0]], [0, 1, 0, h[1]], [0, -1, 0, h[1]], [0, 0, 1, h[2]],
                       [0, 0, -1, h[2]]], float)
    pts = task._face_points(corners, planes)
    top = SEAT_Z + task.PCB_T + task.SINK[2]
    centre = np.array([BX + task.SINK_AT[0], BY + task.SINK_AT[1], top + gap + h[2]])
    return [pts + centre]


SHORT = task.SHORT
CASES = [
    ("at the start", start(), 0.0, False),
    # full success needs the hold, the let-go and every condition
    *held("home, centred, let go", board(), 1.0),
    ("home but a fingertip still on it", board(clear=False), 0.0, False),
    ("home, arm state unknown", board(clear=None), 0.0, False),
    *reset(),
    *held("home, an arm face 4.5 mm above the heat sink (no corner near)", board(points=plate(0.0045)), 0.0),
    *reset(),
    *held("home, an arm face 6 mm above the heat sink", board(points=plate(0.006)), 1.0),
    *reset(),
    # partial levels
    *held("set down on the standoffs short of the shield, let go", board(dx=-SHORT), 0.25),
    *reset(),
    *held("D-sub shells 0.9 mm into their cut-outs (5.1 mm short of home)", board(dx=-0.0051), 0.25),
    *reset(),
    *held("D-sub shells 1.1 mm into their cut-outs (4.9 mm short of home)", board(dx=-0.0049), 0.5),
    *reset(),
    *held("flanges 0.31 mm off the shield", board(dx=-0.00031), 0.5),
    *reset(),
    *held("flanges 0.29 mm off the shield", board(dx=-0.00029), 1.0),
    *reset(),
    *held("home in depth, 0.49 mm to the side (beyond the D-sub cut-out play; scoring limit only)", board(dy=0.00049), 0.5),
    *reset(),
    *held("home in depth, 0.47 mm to the side", board(dy=0.00047), 1.0),
    *reset(),
    # not in its place
    *held("on the standoffs, turned 90 degrees (connectors to the side wall)", board(yaw=90.0), 0.0),
    *reset(),
    *held("on the standoffs, turned 6 degrees", board(dx=-SHORT, yaw=6.0), 0.0),
    *reset(),
    *held("tilted 2.5 degrees, one edge up", board(dx=-SHORT, dz=0.002, pitch=2.5), 0.0),
    *reset(),
    *held("resting 1.5 mm above the standoff tops (held up)", board(dx=-SHORT, dz=0.0015), 0.0),
    *reset(),
    *held("shifted 6 mm sideways: a standoff top no longer fully under the PCB", board(dx=-SHORT, dy=0.006), 0.0),
    *reset(),
    *held("left on the foam pad", start(), 0.0),
    # whole-trial zeros
    *reset(),
    ("dropped from above the standoffs", board(dz=0.02, vz=-0.7), 0.0, False),
    *held("dropped, then pushed home", board(), 0.0),
    *reset(),
    ("one corner on the chassis floor", board(dx=-0.02, dz=-0.004, roll=4.0), 0.0, False),
    *held("slipped off, then put home", board(), 0.0),
    *reset(),
    *held("after a reset, home again", board(), 1.0),
]
