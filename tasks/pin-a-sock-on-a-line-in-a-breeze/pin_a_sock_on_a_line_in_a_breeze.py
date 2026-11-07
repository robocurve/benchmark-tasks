"""On the station table stands a short clothesline between two posts, with a long sock folded over it along its length;
a breeze swings the sock to and fro about the line. A wooden clothespin stands in a holder beside it.

Task: pin the sock to the line. One hand holds the sock still on the line, pinching it; the other takes the clothespin
by its handle (squeezing it opens the jaws), brings it down astride the line and the sock, and lets go: the pin's spring
shuts its jaws on them. Then the first hand lets go too, and the sock stays put.

Run it:  inspect-robots run --embodiment yam_arena -E task=pin_a_sock_on_a_line_in_a_breeze.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, hinge, on_reset,  # noqa: E402
                     on_step, rotation, set_glue_on, set_hinge_drive)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the line (fixed): a rod along y between two posts ---------------------------------------------------------------
LX, LZ, LR = 0.30, TOP + 0.06, 0.003  # its axis, and its radius
LINE_Y = (-0.17, 0.17)

# -- the sock: one body hung over the line, its fold resting on it; origin at the middle of the fold --------------------
SOCK_Y = (-0.12, 0.10)
FOLD_R = 0.0045
PANEL_T, PANEL_H = 0.003, 0.05  # each half hangs down beside the line
SOCK_MASS = 0.02
FOLD_Z = LZ + LR + FOLD_R
BREEZE, BREEZE_S = 0.05, 1.2  # N, s: a light, steady to-and-fro push across the sock's lower edge

# -- the clothespin: a fixed jaw with the handle block, and a moving jaw on a spring hinge (its fulcrum). Squeezing the
# handle opens the jaws: while a gripper holds the handle closed the task file drives them open; let go, the spring
# shuts them
JAW_L, HANDLE_L = 0.05, 0.035
PIN_W, PIN_T = 0.012, 0.005
HANDLE_X = 0.014  # the handle block, across
OPEN_DEG = 35.0
SPRING_K = 0.006  # N m per degree, toward shut
SQUEEZE_K = 0.004  # N m per degree, toward open, while held
HELD_AT = 0.02  # a TCP this close to the handle block's middle, its fingers closed, holds (squeezes) it ...
HELD_FINGERS = 0.15  # ... its fingers closed this far at least (of wide open): opening them lets go at once
PIN_MASS = 0.004  # each lever
HOLDER_AT = (0.22, -0.16)
SLOT_D = 0.020  # the holder's slot, deep
FULCRUM0 = np.array([HOLDER_AT[0], HOLDER_AT[1], TOP + 0.004 + JAW_L])
ANCHOR_AT = np.array([LX, 0.0, TOP - 0.05])  # the clip's hidden fixed block, under the table

WOOD, SOCK_C, METAL, DARK = (0.80, 0.68, 0.45), (0.30, 0.55, 0.30), (0.70, 0.71, 0.74), (0.20, 0.21, 0.23)

# -- scoring ----------------------------------------------------------------------------------------------------------
SOCK_HELD_AT = 0.03  # the other gripper's TCP this close to the sock (across it, and up or down from its fold) holds it
CLIP_WINDOW = 10  # steps after the pin is let go in which it can snap shut on the sock
LET_GO = 0.04
SUCCESS_HOLD_S = 1.0


def _line() -> Body:
    shapes = [CylShape((0.0, (LINE_Y[0] + LINE_Y[1]) / 2, LZ - TOP), LR, LINE_Y[1] - LINE_Y[0] + 0.01, METAL, axis="Y")]
    for y in (LINE_Y[0] - 0.005, LINE_Y[1] + 0.005):
        shapes.append(CylShape((0.0, y, (LZ - TOP + 0.01) / 2), 0.006, LZ - TOP + 0.01, DARK))
    return Body.build("line", pos=(LX, 0.0, TOP), mass=1.0, shapes=shapes, kinematic=True, friction=(0.1, 0.1),
                      slick=True, color=METAL)


def _sock() -> Body:
    length = SOCK_Y[1] - SOCK_Y[0]
    shapes = [CylShape((0.0, 0.0, 0.0), FOLD_R, length, SOCK_C, axis="Y")]
    for sx in (-1.0, 1.0):
        x = sx * (LR + PANEL_T / 2)
        shapes.append(BoxShape((x, 0.0, -(LR + FOLD_R) - PANEL_H / 2 + FOLD_R), (PANEL_T, length, PANEL_H), SOCK_C))
    return Body.build("sock", pos=(LX, (SOCK_Y[0] + SOCK_Y[1]) / 2, FOLD_Z), mass=SOCK_MASS, shapes=shapes,
                      friction=(0.6, 0.5), color=SOCK_C, com=(0.0, 0.0, -0.025))


def _lever(name: str, side: float) -> Body:
    """pin_a (side -1): the fixed jaw, on -x, with the handle block above the fulcrum; pin_b (side +1): the moving jaw.
    Origin at the fulcrum."""
    s = side
    shapes = [BoxShape((s * PIN_T / 2, 0.0, -JAW_L / 2), (PIN_T, PIN_W, JAW_L), WOOD)]
    if side < 0:
        shapes += [BoxShape((0.0, 0.0, 0.003 + HANDLE_L / 2), (HANDLE_X, PIN_W, HANDLE_L), WOOD),
                   CylShape((0.0, 0.0, 0.0), 0.004, PIN_W + 0.004, METAL, collide=False, axis="Y")]  # the spring
    return Body.build(name, pos=tuple(FULCRUM0.tolist()), mass=PIN_MASS, shapes=shapes, friction=(1.0, 0.9),
                      color=WOOD)


def _holder() -> Body:
    """A stand with a narrow slot that takes the fixed jaw (on -x of the fulcrum), open on +x so the other jaw can
    open."""
    t = 0.006
    w = PIN_T + 0.003  # the slot, across x
    sy = PIN_W + 0.004
    cx = -PIN_T / 2  # under the fixed jaw
    shapes = [BoxShape((0.0, 0.0, 0.002), (0.05, sy + 2 * t, 0.004), DARK),
              BoxShape((cx - w / 2 - t, 0.0, SLOT_D / 2), (2 * t, sy + 2 * t, SLOT_D), DARK),
              BoxShape((cx, sy / 2 + t / 2, SLOT_D / 2), (w, t, SLOT_D), DARK),
              BoxShape((cx, -sy / 2 - t / 2, SLOT_D / 2), (w, t, SLOT_D), DARK)]
    return Body.build("holder", pos=(HOLDER_AT[0], HOLDER_AT[1], TOP), mass=0.2, shapes=shapes, kinematic=True,
                      friction=(0.3, 0.25), color=DARK)


def _anchor() -> Body:
    """A small fixed block hidden under the table, out of everything's way: the clip fixes the sock and the pin to it
    (a joint to the line itself would stop the line and the sock colliding, even while off)."""
    return Body.build("anchor", pos=tuple(ANCHOR_AT.tolist()), mass=0.1, kinematic=True, color=DARK,
                      shapes=[BoxShape((0.0, 0.0, 0.0), (0.005, 0.005, 0.005), DARK, collide=False)])


LINE, SOCK, PIN_A, PIN_B, HOLDER = _line(), _sock(), _lever("pin_a", -1.0), _lever("pin_b", 1.0), _holder()
OBJECTS = [_anchor(), LINE, SOCK, HOLDER, PIN_A, PIN_B]
hinge("pin_spring", "pin_a", "pin_b", anchor=tuple(FULCRUM0.tolist()), axis="Y", limits_deg=(-OPEN_DEG, 0.0),
      spring=SPRING_K, damping=0.0003, target_deg=0.0)
# the clip: once the pin snaps shut on the held sock, the task file fixes both where they are (off till then)
SOCK_AT0 = np.array([LX, (SOCK_Y[0] + SOCK_Y[1]) / 2, FOLD_Z])
glue("clip_sock", "anchor", "sock", anchor=tuple(SOCK_AT0.tolist()))
glue("clip_pin", "anchor", "pin_a", anchor=tuple(FULCRUM0.tolist()))


# -- followed after every control step: the breeze, the squeeze on the pin's handle, the clip ---------------------------

def _fresh() -> dict:
    return {"t": 0.0, "tcps": None, "fingers": None, "held": False, "by": None, "since_let_go": None, "clipped": False,
            "clip_swing": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("PIN_DEBUG")), "steps": 0}
_ARM_IDS: dict[str, tuple[int, int]] = {}


def _quat_wxyz(m: np.ndarray) -> tuple[float, float, float, float]:
    w = math.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    if w > 1e-6:
        return w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)
    i = int(np.argmax(np.diag(m)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1.0 + m[i, i] - m[j, j] - m[k, k])) * 2
    q = [0.0, 0.0, 0.0, 0.0]
    q[0] = (m[k, j] - m[j, k]) / s
    q[1 + i] = s / 4
    q[1 + j] = (m[j, i] + m[i, j]) / s
    q[1 + k] = (m[k, i] + m[i, k]) / s
    return tuple(q)


def _glue_frames(name: str, pos0, rot0, pos1) -> None:
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS[name]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(np.asarray(rot0, float))))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(*[float(v) for v in pos1]))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    for name, at in (("clip_sock", SOCK_AT0), ("clip_pin", FULCRUM0)):
        _glue_frames(name, at - ANCHOR_AT, np.eye(3), np.zeros(3))
        set_glue_on(name, False)


def _hands(base, origin) -> tuple[list[np.ndarray], list[float]]:
    """Each gripper's TCP (left, right) and its finger joint (0 shut, FINGER_OPEN wide open)."""
    tcps, fingers = [], []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        if arm not in _ARM_IDS:
            names, joints = list(robot.body_names), list(robot.joint_names)
            _ARM_IDS[arm] = (next(i for i, n in enumerate(names) if n == "gripper" or n.endswith("/gripper")),
                             joints.index("joint7"))
        body_i, finger_j = _ARM_IDS[arm]
        g = as_numpy(robot.data.body_link_pose_w)[0][body_i]
        tcps.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
        fingers.append(float(as_numpy(robot.data.joint_pos)[0][finger_j]))
    return tcps, fingers


def _set_force(asset, f_world, rot, at) -> None:
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    pos = torch.tensor(np.asarray(at, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), positions=pos,
                                                           is_global=False)


def _poses(base, origin) -> dict:
    out = {}
    for n in ("sock", "pin_a", "pin_b"):
        p = as_numpy(base.scene[n].data.root_link_pose_w)[0]
        out[n] = (p[:3] - origin, rotation(p[3:7]))
    return out


def _handle(ps) -> np.ndarray:
    a_pos, a_rot = ps["pin_a"]
    return a_pos + a_rot @ np.array([0.0, 0.0, 0.003 + HANDLE_L / 2])


def _on_sock(p, ps) -> bool:
    """A TCP holding the sock: within SOCK_HELD_AT across it, between its lower edge and SOCK_HELD_AT above its fold,
    and along its length."""
    s_pos, s_rot = ps["sock"]
    v = s_rot.T @ (np.asarray(p, float) - s_pos)
    half = (SOCK_Y[1] - SOCK_Y[0]) / 2
    return abs(v[0]) < SOCK_HELD_AT and -PANEL_H < v[2] < SOCK_HELD_AT and abs(v[1]) < half + 0.01


def _clip(ps) -> None:
    """The pin has snapped shut on the held sock: fix both to the line where they are."""
    for name, body in (("clip_sock", "sock"), ("clip_pin", "pin_a")):
        pos, rot = ps[body]
        _glue_frames(name, pos - ANCHOR_AT, rot, np.zeros(3))
        set_glue_on(name, True)
    _EP["clipped"] = True
    _EP["clip_swing"] = _status(ps)["swing"]


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    _EP["t"] += 0.05
    ps = _poses(base, origin)
    s_pos, s_rot = ps["sock"]
    f = 0.0 if _EP["clipped"] else BREEZE * math.sin(2 * math.pi * _EP["t"] / BREEZE_S)
    _set_force(base.scene["sock"], [f, 0.0, 0.0], s_rot, at=(0.0, 0.0, -0.04))
    _EP["tcps"], _EP["fingers"] = _hands(base, origin)
    handle = _handle(ps)
    holding = [i for i, (p, fi) in enumerate(zip(_EP["tcps"], _EP["fingers"]))
               if float(np.linalg.norm(p - handle)) < HELD_AT and fi > FINGER_OPEN * HELD_FINGERS]
    held = bool(holding) and not _EP["clipped"]
    if held:
        _EP["by"], _EP["since_let_go"] = holding[0], None
    elif _EP["held"]:
        _EP["since_let_go"] = 0
    if held != _EP["held"]:
        _EP["held"] = held
        set_hinge_drive("pin_spring", target_deg=-OPEN_DEG if held else 0.0, spring=SQUEEZE_K if held else SPRING_K)
    if _EP["since_let_go"] is not None and not _EP["clipped"]:
        _EP["since_let_go"] += 1
        st = _status(ps)
        if st["on_line"] and st["sock_in"] and _on_sock(_EP["tcps"][1 - _EP["by"]], ps):
            _clip(ps)
        if _EP["since_let_go"] >= CLIP_WINDOW:
            _EP["since_let_go"] = None
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status(ps)
            tl, tr = _EP["tcps"]
            print(f"[pin] t={_DEBUG['steps'] / 20:5.1f}s sock={np.round(s_pos * 1000).astype(int).tolist()} "
                  f"swing={st['swing']:5.1f} fulcrum={np.round(ps['pin_a'][0] * 1000).astype(int).tolist()} "
                  f"open={st['open']:5.1f} held={_EP['held']} fingers={np.round(_EP['fingers'], 4).tolist()} "
                  f"on_line={st['on_line']} sock_in={st['sock_in']} hung={st['hung']} clipped={_EP['clipped']} "
                  f"let_go={st['let_go']} rightTCP-handle={np.round((tr - handle) * 1000).astype(int).tolist()} "
                  f"leftTCP-fold={np.round((tl - s_pos) * 1000).astype(int).tolist()} "
                  f"left_on_sock={_on_sock(tl, ps)} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status(ps) -> dict:
    s_pos, s_rot = ps["sock"]
    a_pos, a_rot = ps["pin_a"]
    b_pos, b_rot = ps["pin_b"]
    rel = a_rot.T @ b_rot
    open_deg = -math.degrees(math.atan2(rel[0, 2], rel[0, 0]))
    up = a_rot @ np.array([0.0, 0.0, 1.0])
    line_pt = np.array([LX, a_pos[1], LZ])
    v = a_rot.T @ (line_pt - a_pos)  # the line in the pin's frame
    on_line = (abs(v[0]) < 0.012 and -JAW_L + 0.003 < v[2] < -0.004 and up[2] > math.cos(math.radians(30.0))
               and LINE_Y[0] < a_pos[1] < LINE_Y[1])
    sock_in = SOCK_Y[0] + 0.006 < a_pos[1] < SOCK_Y[1] - 0.006 and abs(s_pos[0] - LX) < 0.006 \
        and abs(s_pos[2] - FOLD_Z) < 0.006
    swing = math.degrees(math.atan2(s_rot[0, 2], s_rot[2, 2]))
    hung = abs(s_pos[0] - LX) < 0.01 and abs(s_pos[2] - FOLD_Z) < 0.01 and abs(swing) < 20.0
    tcps = _EP["tcps"] or []
    let_go = all(float(np.linalg.norm(p - a_pos)) >= LET_GO + 0.02 and
                 float(np.linalg.norm((s_rot.T @ (p - s_pos))[[0, 2]])) >= LET_GO for p in tcps)
    return {"on_line": on_line, "sock_in": sock_in, "hung": hung, "let_go": let_go, "open": open_deg, "swing": swing,
            "clipped": _EP["clipped"]}


def _points(st) -> float:
    pinned = st["clipped"] and st["on_line"] and st["sock_in"] and st["hung"]
    if pinned and st["let_go"]:
        return 1.0
    if pinned or st["clipped"]:
        return 0.5
    return 0.25 if st["on_line"] else 0.0


def score(objects) -> float:
    """1.0 with the clothespin snapped shut on the line (the line between its jaws, the pin upright) over the sock
    while the other hand held the sock, the sock hanging straight on the line, and both hands let go; 0.5 with the pin
    so snapped on but a hand still on it or the sock; 0.25 with the pin on the line otherwise; else 0.0."""
    ps = {}
    for n in ("sock", "pin_a", "pin_b"):
        ps[n] = (np.asarray(objects[n].pos, float), rotation(objects[n].quat_xyzw))
    return _points(_status(ps))


def success(objects) -> bool:
    """Full score with the sock and the pin still (the breeze stops once the pin has clipped on): the trial ends once it
    has held 1 s."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("sock", "pin_a", "pin_b"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down astride the sock near its left end and pinches its two halves together round the
# line, holding it still in the breeze. The right hand comes straight down on the clothespin's handle, fingers closing across
# it (squeezing it opens the jaws), lifts it out of its holder, brings it over the line near the sock's right end and
# down over the line and the sock, and lets go: the spring closes the jaws on them. Then both hands rise.

GRIP_L = grip_rotation(-math.pi / 2)  # straight down, fingers closing along x: pinching the sock's two halves
GRIP_R = grip_rotation(math.pi / 2)  # straight down, fingers closing along x: across the pin's handle block
PINCH_Z = LZ - 0.012  # the left TCP: its fingers pinch the sock's halves together round the line
PRESS_Y = SOCK_Y[1] - 0.02
PIN_Y = SOCK_Y[1] - 0.17  # where the pin goes on: 15 cm right of where the left hand pinches
HANDLE_IN_HAND = 0.003 + HANDLE_L / 2  # the right TCP this far above the fulcrum: the handle's middle
PIN_IN_HAND = np.array([0.005, -0.006, 0.0])  # where the gripped pin hangs from the right TCP (measured): aimed against
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


def _left(release_at: float, total: float) -> tuple[list[Waypoint], float]:
    pinch = np.array([LX, PRESS_Y, PINCH_Z])
    out = _lead_in("left", pinch + [0.0, 0.0, 0.05], GRIP_L, "over the sock's left end", FINGER_OPEN)
    out += [_w(1.2, pinch, FINGER_OPEN, "down astride it", GRIP_L),
            _w(0.8, pinch, FINGER_CLOSED, "pinch it on the line: it holds still", GRIP_L)]
    held_at = sum(w.duration for w in out)
    out += [_w(max(release_at - held_at, 0.3), pinch, FINGER_CLOSED, "hold it while it is pinned", GRIP_L),
            _w(0.6, pinch, FINGER_OPEN, "let go", GRIP_L),
            _w(1.0, pinch + [0.0, 0.03, 0.07], FINGER_OPEN, "rise", GRIP_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), pinch + [0.0, 0.03, 0.07], FINGER_OPEN,
                  "hold still: the trial ends once the full score has held 1 s", GRIP_L))
    return out, held_at


def _right(start: float) -> tuple[list[Waypoint], float]:
    pick = FULCRUM0 + [0.0, 0.0, HANDLE_IN_HAND]
    x = LX - (LR + PANEL_T) - 0.003  # the fixed jaw 3 mm clear of the sock's near side; the other opens beyond it
    over = np.array([x, PIN_Y, LZ + 0.02 + JAW_L + HANDLE_IN_HAND]) - PIN_IN_HAND  # the fixed jaw's tip 2 cm above the line
    on = np.array([x, PIN_Y, LZ - 0.015 + JAW_L + HANDLE_IN_HAND]) - PIN_IN_HAND  # ... and 1.5 cm below it, astride it
    out = _lead_in("right", pick + [0.0, 0.0, 0.04], GRIP_R, "over the clothespin", FINGER_OPEN)
    out += [_w(1.0, pick, FINGER_OPEN, "down round its handles", GRIP_R),
            _w(0.8, pick, FINGER_CLOSED, "squeeze them: the jaws open", GRIP_R),
            _w(1.2, pick + [0.0, 0.0, 0.04], FINGER_CLOSED, "lift it out of its holder", GRIP_R)]
    t = sum(w.duration for w in out)
    out += [_w(max(start - t, 0.3) + 1.5, over, FINGER_CLOSED, "over the line by the sock's right end", GRIP_R),
            _w(1.2, on, FINGER_CLOSED, "down astride the line and the sock", GRIP_R),
            _w(0.6, on, FINGER_OPEN, "let go: the spring closes the jaws", GRIP_R)]
    pinned_at = sum(w.duration for w in out)
    out += [_w(1.0, on + [0.0, -0.03, 0.05], FINGER_OPEN, "rise clear", GRIP_R)]
    return out, pinned_at


def demo():
    _, held_at = _left(0.0, 0.0)
    right, pinned_at = _right(held_at)
    t_r = sum(w.duration for w in right)
    total = t_r + 4.0
    right.append(_w(4.0, right[-1].pos, FINGER_OPEN, "wait", GRIP_R))
    left, _ = _left(pinned_at + 0.5, total)
    return {"left": left, "right": right}
