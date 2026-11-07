"""On the station table lies a sheet of paper with a thin plastic stencil on it: three square holes in a row. A sponge
dauber stands in a holder beside it.

Task: print the stencil's three squares. One hand presses the stencil flat on the paper; the other takes the dauber and
dabs paint through each hole, once each, then lifts it clear. Dabbed without the stencil held flat, the paint bleeds
under its edges: the print smudges past the hole.

Run it:  inspect-robots run --embodiment yam_arena -E task=dab_paint_through_a_held_stencil.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, plate,  # noqa: E402
                     rotation)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the paper (fixed) and the stencil (loose): a thin sheet along y with three square holes ---------------------------
PAPER = (0.20, 0.30, 0.0004)
PAPER_AT = (0.30, 0.0)
PAPER_TOP = TOP + PAPER[2]
STENCIL = (0.10, 0.27, 0.002)  # x, y, thick
STENCIL_AT = (0.30, 0.005)
STENCIL_MASS = 0.012
HOLE = 0.030  # square
HOLES_Y = (-0.105, -0.067, -0.029)  # the holes' middles, world y (on the stencil as it first lies)

# -- the dauber: a square handle with a sponge pad under it; it stands in a holder ----------------------------------
PAD, PAD_H = 0.018, 0.010
HANDLE, HANDLE_H = 0.014, 0.075
DAUBER_MASS = 0.02
HOLDER_AT = (0.20, -0.12)
HOLDER_IN, HOLDER_H = 0.024, 0.012  # a square cup round the pad

# -- the paint: prints laid on the paper where the pad touches it --------------------------------------------------------
N_PRINTS = 8
TOUCH = 0.001  # the pad within this of the paper (through a hole: on the stencil it is 2 mm up): it prints
UP = 0.01  # lifted this far off the paper between dabs
HELD = 0.03  # held flat: the other gripper's TCP within this above the stencil
BLEED = 0.034  # a smudged print spreads this wide
HIDDEN = (PAPER_AT[0] + 0.08, PAPER_AT[1] + 0.13)  # where unused prints wait, under the paper
WHITE, CLEAR, DARK, WOOD, RED = (0.95, 0.95, 0.93), (0.55, 0.75, 0.85), (0.20, 0.21, 0.23), (0.80, 0.68, 0.45), \
    (0.80, 0.12, 0.15)

# -- scoring ----------------------------------------------------------------------------------------------------------
LIFTED = 0.01
SUCCESS_HOLD_S = 1.0


def _paper() -> Body:
    return Body.build("paper", pos=(PAPER_AT[0], PAPER_AT[1], TOP), mass=0.01, kinematic=True, color=WHITE,
                      shapes=[BoxShape((0.0, 0.0, PAPER[2] / 2), PAPER, WHITE)], friction=(0.4, 0.35))


def _stencil() -> Body:
    """A sheet with three square holes: built from strips round them."""
    sx, sy, t = STENCIL
    y0, y1 = -sy / 2, sy / 2
    z = t / 2
    shapes = [BoxShape((-(sx / 2 + HOLE / 2) / 2, 0.0, z), (sx / 2 - HOLE / 2, sy, t), CLEAR),  # in front of the holes
              BoxShape(((sx / 2 + HOLE / 2) / 2, 0.0, z), (sx / 2 - HOLE / 2, sy, t), CLEAR)]  # behind them
    edges = [y0] + [v for hy in HOLES_Y for v in (hy - STENCIL_AT[1] - HOLE / 2, hy - STENCIL_AT[1] + HOLE / 2)] + [y1]
    for a, b in zip(edges[0::2], edges[1::2]):  # the strips between the holes, across the middle band
        shapes.append(BoxShape((0.0, (a + b) / 2, z), (HOLE, b - a, t), CLEAR))
    return Body.build("stencil", pos=(STENCIL_AT[0], STENCIL_AT[1], PAPER_TOP + 0.0002), mass=STENCIL_MASS,
                      shapes=shapes, friction=(0.5, 0.45), color=CLEAR)


def _holder() -> Body:
    w, t = HOLDER_IN, 0.003
    shapes = [BoxShape((0.0, 0.0, 0.002), (w + 2 * t, w + 2 * t, 0.004), DARK)]
    for dx, dy, sx, sy in ((w / 2 + t / 2, 0.0, t, w + 2 * t), (-w / 2 - t / 2, 0.0, t, w + 2 * t),
                           (0.0, w / 2 + t / 2, w, t), (0.0, -w / 2 - t / 2, w, t)):
        shapes.append(BoxShape((dx, dy, HOLDER_H / 2), (sx, sy, HOLDER_H), DARK))
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.1, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


def _dauber() -> Body:
    """Origin at the middle of the pad's underside; it stands upright."""
    shapes = [BoxShape((0.0, 0.0, PAD_H / 2), (PAD, PAD, PAD_H), (0.95, 0.85, 0.40)),
              BoxShape((0.0, 0.0, PAD_H + HANDLE_H / 2), (HANDLE, HANDLE, HANDLE_H), WOOD),
              BoxShape((0.0, 0.0, 0.0006), (PAD - 0.002, PAD - 0.002, 0.0012), RED, collide=False)]  # paint on it
    return Body.build("dauber", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045), mass=DAUBER_MASS, shapes=shapes,
                      friction=(1.0, 0.9), color=WOOD, com=(0.0, 0.0, PAD_H + HANDLE_H / 2))


def _prints() -> list[Body]:
    out = []
    for k in range(N_PRINTS):
        out.append(Body.build(f"print_{k}", pos=(HIDDEN[0], HIDDEN[1], TOP + 0.0001), mass=0.001, kinematic=True,
                              color=RED, shapes=[BoxShape((0.0, 0.0, 0.0), (PAD, PAD, 0.0002), RED, collide=False)]))
        out.append(Body.build(f"smudge_{k}", pos=(HIDDEN[0], HIDDEN[1], TOP + 0.0001), mass=0.001, kinematic=True,
                              color=RED, shapes=[BoxShape((0.0, 0.0, 0.0), (BLEED, BLEED, 0.0002), RED, collide=False)]))
    return out


PAPER_B, STENCIL_B, HOLDER, DAUBER = _paper(), _stencil(), _holder(), _dauber()
PRINTS = _prints()
OBJECTS = [PAPER_B, STENCIL_B, HOLDER, DAUBER, *PRINTS]


# -- followed after every control step: the dabs ------------------------------------------------------------------------

def _fresh() -> dict:
    return {"prints": [], "down": False, "tcps": None, "pad": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("DAB_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _place(asset, origin, x: float, y: float) -> None:
    import torch

    pose = torch.tensor([[x + origin[0], y + origin[1], PAPER_TOP + 0.0002 + origin[2], 0.0, 0.0, 0.0, 1.0]],
                        dtype=torch.float32, device=asset.device)
    asset.write_root_pose_to_sim(pose)


def _held(s_pos, s_rot, pad, tcps) -> bool:
    """Some gripper other than the one dabbing presses on the stencil: its TCP within HELD above the sheet."""
    for p in tcps:
        if float(np.linalg.norm(p - pad)) < 0.05:
            continue  # the hand with the dauber
        v = s_rot.T @ (p - s_pos)
        if abs(v[0]) <= STENCIL[0] / 2 and abs(v[1]) <= STENCIL[1] / 2 and -0.005 <= v[2] <= HELD:
            return True
    return False


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    d = as_numpy(base.scene["dauber"].data.root_link_pose_w)[0]
    s = as_numpy(base.scene["stencil"].data.root_link_pose_w)[0]
    pad, s_pos, s_rot = d[:3] - origin, s[:3] - origin, rotation(s[3:7])
    _EP["pad"] = pad
    _EP["tcps"] = _tcps(base, origin)
    h = pad[2] - PAPER_TOP
    if h > UP:
        _EP["down"] = False
    elif h < TOUCH and not _EP["down"] and len(_EP["prints"]) < N_PRINTS:
        _EP["down"] = True  # a dab: the pad is on the paper (it can only be, through a hole)
        clean = _held(s_pos, s_rot, pad, _EP["tcps"])
        k = len(_EP["prints"])
        _place(base.scene[f"{'print' if clean else 'smudge'}_{k}"], origin, float(pad[0]), float(pad[1]))
        _EP["prints"].append((float(pad[0]), float(pad[1]), clean))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status()
            tl, tr = _EP["tcps"]
            print(f"[dab] t={_DEBUG['steps'] / 20:5.1f}s pad={np.round(pad * 1000).astype(int).tolist()} "
                  f"stencil={np.round(s_pos * 1000).astype(int).tolist()} prints={[(round(x * 1000), round(y * 1000), c) for x, y, c in _EP['prints']]} "
                  f"leftTCP-z={tl[2] - s_pos[2]:.3f} done={st['done']} smudged={st['smudged']} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status() -> dict:
    done = set()
    for x, y, clean in _EP["prints"]:
        for k, hy in enumerate(HOLES_Y):
            if clean and abs(x - STENCIL_AT[0]) <= HOLE / 2 and abs(y - hy) <= HOLE / 2:
                done.add(k)
    smudged = sum(1 for *_, c in _EP["prints"] if not c)
    pad = _EP["pad"]
    lifted = pad is not None and pad[2] - PAPER_TOP >= LIFTED
    return {"done": len(done), "smudged": smudged, "lifted": lifted, "prints": len(_EP["prints"])}


def _points(st) -> float:
    if st["done"] == len(HOLES_Y) and st["prints"] == len(HOLES_Y) and st["smudged"] == 0 and st["lifted"]:
        return 1.0
    if st["done"] >= 2 or (st["done"] + st["smudged"]) >= len(HOLES_Y):
        return 0.5
    return 0.25 if (st["done"] + st["smudged"]) >= 1 else 0.0


def score(objects) -> float:
    """1.0 with all three squares printed cleanly once each (each dabbed with the stencil held flat, inside its hole
    where the stencil first lay; exactly three prints in all), no smudged print, and the dauber lifted; 0.5 with two clean squares, or three prints with some
    smudged; 0.25 with at least one print; else 0.0."""
    return _points(_status())


def success(objects) -> bool:
    """Full score with the dauber and the stencil still."""
    return score(objects) >= 1.0 and objects["dauber"].resting() and objects["stencil"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the stencil's left end, fingers closed, and presses it flat on the paper. The
# right hand comes straight down on the dauber's handle, fingers closing across it, lifts it out of its holder and dabs
# it straight down through each hole in turn, right to left, then lifts it clear.

GRIP = grip_rotation(0.0)  # straight down: the left's fingers are closed; the right's close along y, across the handle
PRESS = 0.012
PRESS_Y = STENCIL_AT[1] + STENCIL[1] / 2 - 0.02  # near the stencil's left end
HANDLE_IN_HAND = 0.02  # the right TCP this far below the handle's top
DAB_DOWN = 0.005  # aim the pad this far into the paper (the arm gives: a firm dab)
LEAD_S, LEAD_STEPS = 3.0, 15


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.5, 0.0, 0.5), (0.8, 1.3), (0.4, 1.0), (-0.8, 0.0, 0.8), (-1.4, 0.0, 1.4), (0.0,)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _lead_in(arm: str, pos, rot, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


PAD_TCP = PAD_H + HANDLE_H - HANDLE_IN_HAND  # the right TCP above the pad's underside


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    press = np.array([STENCIL_AT[0], PRESS_Y, PAPER_TOP + STENCIL[2] + 0.013 - PRESS])
    out = _lead_in("left", press + [0.0, 0.0, 0.04 + PRESS], GRIP, "over the stencil's left end", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, PRESS], FINGER_CLOSED, "down onto it", GRIP),
            _w(0.8, press, FINGER_CLOSED, "press it flat on the paper", GRIP)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), press, FINGER_CLOSED, "hold it flat while the paint is dabbed", GRIP),
            _w(1.0, press + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), press + [0.0, 0.0, 0.05], FINGER_CLOSED,
                  "hold still: the trial ends once the full score has held 1 s", GRIP))
    return out, held_at


def _right(start: float) -> list[Waypoint]:
    pick = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP + 0.0045 + PAD_TCP])
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP, "over the dauber", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round its handle", GRIP),
            _w(0.8, pick, FINGER_CLOSED, "take it", GRIP),
            _w(1.0, pick + [0.0, 0.0, 0.03], FINGER_CLOSED, "lift it out of its holder", GRIP)]
    t = sum(w.duration for w in out)
    first = True
    for hy in HOLES_Y:
        above = np.array([STENCIL_AT[0], hy, PAPER_TOP + PAD_TCP + 0.025])
        down = np.array([STENCIL_AT[0], hy, PAPER_TOP + PAD_TCP - DAB_DOWN])
        wait = max(start - t, 0.3) + 0.5 if first else 0.0
        out += [_w(1.2 + wait, above, FINGER_CLOSED, "over the next hole", GRIP),
                _w(1.0, down, FINGER_CLOSED, "dab: down through the hole onto the paper", GRIP),
                _w(0.3, down, FINGER_CLOSED, "dab", GRIP),
                _w(0.6, above, FINGER_CLOSED, "up", GRIP)]
        first = False
    out.append(_w(1.0, out[-1].pos + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "lift it clear", GRIP))
    return out


def demo():
    _, held_at = _left(0.0, 0.0)
    right = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 3.0
    right.append(_w(3.0, right[-1].pos, FINGER_CLOSED, "hold still", GRIP))
    left, _ = _left(t_r, total)
    return {"left": left, "right": right}
