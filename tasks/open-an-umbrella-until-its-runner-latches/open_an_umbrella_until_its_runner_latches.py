"""On the station table a stick umbrella, closed, rests on a fixed stand (two cradles under its shaft, one by the tip and
one by the handle), its tip to the right and its handle to the left, its shaft 14.5 cm up.

Task: open the umbrella: hold it by its handle and push the runner (the sleeve on the shaft) along the shaft toward
the tip, against the ribs' springs, until it latches at the top; then let go of the runner, and lift the open umbrella
up by its handle, tip raised. The umbrella lies loose in its stand: pushed on alone, the runner shoves the whole
umbrella along it, so one hand holds the handle while the other pushes the runner.

Run it:  inspect-robots run --embodiment yam_arena -E task=open_an_umbrella_until_its_runner_latches.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, on_reset, on_step, rotation,  # noqa: E402
                     set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the umbrella: its frame: origin at the tip (ferrule) on the shaft's axis, +y along the shaft toward the handle ----
UX, UY, UZ = 0.28, -0.33, TOP + 0.145  # the tip, in the world
SHAFT_R, SHAFT_L = 0.006, 0.40
HANDLE_R, HANDLE_L = 0.015, 0.10
NOTCH_Y = 0.030  # the ribs' top notch, along the shaft from the tip
UMBRELLA_MASS = 0.22

# -- the runner: a sleeve on the shaft; pushed toward the tip it opens the ribs; at the top it latches ----------------
RUNNER_R, RUNNER_L = 0.012, 0.030
RUNNER_Y = 0.300  # closed, along the shaft from the tip
TRAVEL = 0.120  # to the top
SPRING_K, SPRING_PRE = 40.0, 0.030  # N/m back toward closed: about 1.2 N to start it, 6 N at the top
LATCH = 0.95  # pushed this far along its travel, it latches

# -- the ribs and canopy: eight panels from the notch, closed along the shaft, open 60 degrees out --------------------
N_RIBS, RIB_L, RIB_W = 8, 0.140, 0.055
CLOSED_DEG, OPEN_DEG = 5.0, 60.0
NAVY, WOOD, STEEL, DARK, CRADLE_C = (0.10, 0.14, 0.32), (0.45, 0.28, 0.14), (0.75, 0.76, 0.78), (0.12, 0.12, 0.13), \
    (0.30, 0.30, 0.32)
CRADLE_Y = -0.19  # world y of the cradle under the shaft by the tip ...
POST_Y = UY + SHAFT_L + HANDLE_L - 0.005  # ... and of a narrow post under the handle's end (the rest of the handle is free)

# -- scoring ----------------------------------------------------------------------------------------------------------
LET_GO = 0.03
HALF = 0.5
HELD_UP = 0.12  # the umbrella's tip this far above where it lay: held up
BY_HANDLE = 0.04  # a TCP this close to the handle's axis (between its ends): the umbrella is held by its handle
SUCCESS_HOLD_S = 1.0


def _quat_from_matrix(m: np.ndarray) -> tuple[float, float, float, float]:
    """(x, y, z, w) of a rotation matrix, as plain floats."""
    return tuple(float(v) for v in _quat_raw(np.asarray(m, float)))


def _quat_raw(m: np.ndarray):
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s)
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        return (0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s)
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        return ((m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s)
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
    return ((m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s)


def _rib_pose(k: int, open_deg: float):
    """Rib k's middle and rotation in the umbrella's frame, opened ``open_deg`` from the shaft."""
    a, b = math.radians(open_deg), 2 * math.pi * k / N_RIBS
    d = np.array([math.sin(a) * math.cos(b), math.cos(a), math.sin(a) * math.sin(b)])  # along the rib
    w = np.array([-math.sin(b), 0.0, math.cos(b)])  # across it
    n = np.cross(w, d)
    return np.array([0.0, NOTCH_Y, 0.0]) + d * (RIB_L / 2), np.column_stack([w, d, n])


def _umbrella() -> Body:
    shapes = [CylShape((0.0, SHAFT_L / 2, 0.0), SHAFT_R, SHAFT_L, STEEL, axis="Y"),
              CylShape((0.0, 0.006, 0.0), 0.008, 0.012, DARK, axis="Y"),  # the tip
              CylShape((0.0, NOTCH_Y, 0.0), 0.011, 0.012, DARK, axis="Y"),  # the top notch
              CylShape((0.0, SHAFT_L + HANDLE_L / 2, 0.0), HANDLE_R, HANDLE_L, WOOD, axis="Y")]
    return Body.build("umbrella", pos=(UX, UY, UZ), mass=UMBRELLA_MASS, shapes=shapes, friction=(1.2, 1.0), color=NAVY,
                      com=(0.0, 0.30, 0.0))


def _runner() -> Body:
    return Body.build("runner", pos=(UX, UY + RUNNER_Y, UZ), mass=0.02, friction=(1.2, 1.0), color=DARK,
                      shapes=[CylShape((0.0, 0.0, 0.0), RUNNER_R, RUNNER_L, DARK, axis="Y")], ignore=("umbrella",))


def _ribs() -> list[Body]:
    out = []
    for k in range(N_RIBS):
        mid, rot = _rib_pose(k, CLOSED_DEG)
        shapes = [BoxShape((0.0, 0.0, 0.0), (RIB_W, RIB_L, 0.0015), NAVY, collide=False),
                  BoxShape((0.0, 0.0, 0.001), (0.003, RIB_L, 0.002), DARK, collide=False)]
        out.append(Body.build(f"rib_{k}", pos=tuple((np.array([UX, UY, UZ]) + mid).tolist()), mass=0.002, shapes=shapes,
                              color=NAVY, damping=(50.0, 50.0), start_xyzw=_quat_from_matrix(rot)))
    return out


def _cradles() -> Body:
    """A cradle under the shaft by the tip and a narrow post under the handle's end (nothing stops the umbrella sliding
    along them: slid toward the tip, its handle comes off the post)."""
    h = UZ - SHAFT_R - TOP
    hp = UZ - HANDLE_R - TOP
    shapes = [BoxShape((0.0, 0.0, h / 2), (0.05, 0.02, h), CRADLE_C),
              *[BoxShape((s * (SHAFT_R + 0.005), 0.0, h + 0.006), (0.006, 0.02, 0.012), CRADLE_C) for s in (-1.0, 1.0)],
              BoxShape((0.0, POST_Y - CRADLE_Y, hp / 2), (0.03, 0.008, hp), CRADLE_C)]
    return Body.build("cradles", pos=(UX, CRADLE_Y, TOP), mass=1.0, shapes=shapes, kinematic=True,
                      friction=(0.05, 0.05), slick=True, color=CRADLE_C)  # smooth: the umbrella slides along them


CRADLES, UMBRELLA, RUNNER = _cradles(), _umbrella(), _runner()
RIBS = _ribs()
OBJECTS = [CRADLES, UMBRELLA, RUNNER, *RIBS]
RIB_NAMES = [r.name for r in RIBS]
slider("shaft", "umbrella", "runner", anchor=RUNNER.pos, axis="Y", limits=(-TRAVEL, 0.0), stiffness=SPRING_K,
       damping=5.0, target=SPRING_PRE, max_force=200.0)  # 0 = closed; -TRAVEL = at the top


# -- followed after every control step: the runner, the latch, the ribs ---------------------------------------------------

def _fresh() -> dict:
    return {"latched": False, "best": 0.0, "tcps": None}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("UMB_DEBUG")), "steps": 0}


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


def _travel(u_pos, u_rot, r_pos) -> float:
    """How far the runner has been pushed from closed toward the tip (0 to 1)."""
    y = float((u_rot.T @ (np.asarray(r_pos, float) - np.asarray(u_pos, float)))[1])
    return max(0.0, min(1.0, (RUNNER_Y - y) / TRAVEL))


@on_step
def _follow(env) -> None:
    import torch

    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    u = as_numpy(base.scene["umbrella"].data.root_link_pose_w)[0]
    r = as_numpy(base.scene["runner"].data.root_link_pose_w)[0]
    u_pos, u_rot = u[:3] - origin, rotation(u[3:7])
    f = _travel(u_pos, u_rot, r[:3] - origin)
    _EP["tcps"] = _tcps(base, origin)
    _EP["best"] = max(_EP["best"], f)
    _EP["tip_up"] = float(u_pos[2] - UZ)  # the tip (the umbrella's origin) above where it lay
    if not _EP["latched"] and f >= LATCH:  # at the top: it latches
        set_slider_drive("shaft", target=-TRAVEL, stiffness=3000.0)
        _EP["latched"] = True
    # the ribs follow the runner (their stretchers): open in proportion to its travel
    angle = CLOSED_DEG + (OPEN_DEG - CLOSED_DEG) * (1.0 if _EP["latched"] else f)
    for k, name in enumerate(RIB_NAMES):
        mid, rot = _rib_pose(k, angle)
        pos = u_pos + u_rot @ mid + origin
        q = _quat_from_matrix(u_rot @ rot)
        asset = base.scene[name]
        pose = torch.tensor([*pos.tolist(), *q], dtype=torch.float32, device=asset.device).view(1, 7)
        asset.write_root_pose_to_sim(pose)
        asset.write_root_velocity_to_sim(torch.zeros(1, 6, dtype=torch.float32, device=asset.device))
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(r[:3] - origin, f, u_pos, u_rot)
            tl, tr = _EP["tcps"]
            print(f"[umb] t={_DEBUG['steps'] / 20:5.1f}s umbrella={np.round(u_pos * 1000).astype(int).tolist()} "
                  f"travel={f * 100:5.1f}% ribs={angle:5.1f}deg latched={_EP['latched']} let_go={st['let_go']} "
                  f"left TCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"right TCP-runner={np.round((tr - (r[:3] - origin)) * 1000).astype(int).tolist()} score={_points(st)}",
                  file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _to_handle(tcp, u_pos, u_rot) -> float:
    """How far a TCP is from the handle's axis, between the handle's ends."""
    p = u_rot.T @ (np.asarray(tcp, float) - np.asarray(u_pos, float))
    y = min(max(float(p[1]), SHAFT_L), SHAFT_L + HANDLE_L)
    return float(np.linalg.norm(p - np.array([0.0, y, 0.0])))


def _status_from(r_pos, f, u_pos, u_rot) -> dict:
    tcps = _EP["tcps"]
    let_go = tcps is None or all(float(np.linalg.norm(t - np.asarray(r_pos))) >= LET_GO for t in tcps)
    by_handle = tcps is not None and min(_to_handle(t, u_pos, u_rot) for t in tcps) <= BY_HANDLE
    return {"latched": _EP["latched"], "let_go": let_go, "best": _EP["best"], "by_handle": by_handle,
            "up": _EP.get("tip_up", 0.0) >= HELD_UP}


def _status(objects) -> dict:
    u, r = objects["umbrella"], objects["runner"]
    u_pos, u_rot = np.asarray(u.pos, float), rotation(u.quat_xyzw)
    return _status_from(r.pos, _travel(u_pos, u_rot, r.pos), u_pos, u_rot)


def _points(st) -> float:
    if st["latched"] and st["let_go"] and st["up"] and st["by_handle"]:
        return 1.0
    if st["latched"]:
        return 0.5
    return 0.25 if st["best"] >= HALF else 0.0


def score(objects) -> float:
    """1.0 with the runner pushed up until it latched (the umbrella open), no gripper on it (3 cm clear), and the open
    umbrella held up by its handle (its tip at least 12 cm above where it lay, a fingertip centre within 4 cm of the
    handle's axis); 0.5 with it latched; 0.25 if it was pushed at least half way up at some point; else 0.0."""
    return _points(_status(objects))


def success(objects) -> bool:
    """Full score with the umbrella still (held up)."""
    return score(objects) >= 1.0 and objects["umbrella"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes level from the front onto the umbrella's handle, fingers closing across it (top and bottom), and
# holds it. The right hand comes straight down on the runner, grips it and pushes it along the shaft toward the tip
# until it latches; the ribs open. Then the right hand lets go and draws back out of the way, and the left lifts the
# open umbrella up by its handle, tip raised, and holds it up.


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


GRIP_L = _ry(-math.pi / 2) @ grip_rotation(-math.pi / 2)  # level, pointing +x (from the front), closing up and down
LIFT, SHIFT, TILT = 0.10, 0.04, math.radians(35.0)  # the left lifts the umbrella this much, draws it this far left,
# and raises its tip this far
PARK_R = np.array([0.17, -0.22, TOP + 0.15])  # the right waits here, low and back, out of the open umbrella's way
GRIP_R = grip_rotation(math.pi / 2)  # straight down, fingers closing along x (across the runner)
IN_HAND = 0.003
PUSH_PAST = 0.040  # push this far past the top (the shaft's stop holds it; the arm gives against the spring)
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


def _lead_in(arm: str, pos, rot, label: str) -> list[Waypoint]:
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = np.array(READY, float), _good_ik(arm, pos, rot)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), FINGER_OPEN, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


HELD_AT = LEAD_S + 1.0 + 0.8


def _go_home(arm: str, pos, rot, finger) -> list[Waypoint]:
    """From a pose back to the arm's start, straight through joint space (the lead-in, reversed)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q0, q1 = _good_ik(arm, pos, rot), np.array(READY, float)
    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, "back to where it started", r))
    return out


def _left(lift_at: float, total: float) -> list[Waypoint]:
    grip = np.array([UX, UY + SHAFT_L + HANDLE_L / 2, UZ])
    out = _lead_in("left", grip + [-0.08, 0.0, 0.08], GRIP_L, "level, in front of and above the umbrella's handle")
    out += [_w(1.0, grip - [0.07, 0.0, 0.0], FINGER_OPEN, "down in front of the handle", GRIP_L),
            _w(1.0, grip, FINGER_OPEN, "onto the handle", GRIP_L),
            _w(0.8, grip, FINGER_CLOSED, "hold it", GRIP_L)]
    t = sum(w.duration for w in out)
    up = _rx(-TILT) @ GRIP_L  # the wrist rolled: the tip goes up
    out += [_w(max(lift_at - t, 0.5), grip, FINGER_CLOSED, "hold the umbrella in its cradles", GRIP_L),
            _w(1.2, grip + [0.0, SHIFT / 2, LIFT * 0.6], FINGER_CLOSED, "lift the open umbrella", GRIP_L),
            _w(1.5, grip + [0.0, SHIFT, LIFT], FINGER_CLOSED, "raise its tip", up)]
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), grip + [0.0, SHIFT, LIFT], FINGER_CLOSED,
                  "hold it up: the trial ends once the full score has held 1 s", up))
    return out


def _right() -> tuple[list[Waypoint], float]:
    start = np.array([UX, UY + RUNNER_Y, UZ + IN_HAND])
    top = start - [0.0, TRAVEL + PUSH_PAST, 0.0]
    out = _lead_in("right", start + [0.0, 0.0, 0.025], GRIP_R, "over the runner")
    out += [_w(1.0, start, FINGER_OPEN, "down round the runner", GRIP_R),
            _w(0.8, start, FINGER_CLOSED, "grip it", GRIP_R)]
    t = sum(w.duration for w in out)
    out += [_w(max(HELD_AT - t, 0.2) + 0.5, start, FINGER_CLOSED, "wait for the handle to be held", GRIP_R),
            _w(3.0, top, FINGER_CLOSED, "push it up the shaft: the ribs open", GRIP_R),
            _w(0.6, top, FINGER_CLOSED, "it latches", GRIP_R),
            _w(0.5, top, FINGER_OPEN, "let go", GRIP_R),
            _w(1.0, top + [0.0, 0.0, 0.04], FINGER_OPEN, "rise", GRIP_R)]
    out += [_w(1.0, top + [-0.05, 0.0, 0.05], FINGER_OPEN, "back and up, clear of the umbrella", GRIP_R),
            _w(1.5, PARK_R, FINGER_OPEN, "out of the way, low and back", GRIP_R)]
    lift_at = sum(w.duration for w in out)
    return out, lift_at


def demo():
    right, lift_at = _right()
    total = max(sum(w.duration for w in right), lift_at + 2.7) + 3.0
    right.append(_w(total - sum(w.duration for w in right), right[-1].pos, FINGER_OPEN, "wait at the start",
                    right[-1].yaw))
    return {"left": _left(lift_at, total), "right": right}
