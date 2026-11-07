"""On the station table a thin steel plate lies loose; near its right end a strong pot magnet (with a knob to hold it
by) clings to it.

Task: take the magnet off the plate: slide it off the plate's end (it holds on far too hard to be lifted straight
off) and carry it well clear of the plate; then show it still works: pick up the small steel nail lying on the table
with it and hold it up. A strong magnet let come back near the plate is pulled back and snaps onto it, which in real
life crushes fingers: that scores 0. Sliding the magnet drags the light plate along with it, so one hand presses the
plate down while the other slides the magnet off.

Run it:  inspect-robots run --embodiment yam_arena -E task=slide_a_strong_magnet_off_a_steel_plate_without_it_snapping_back.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import (Body, BoxShape, CylShape, allow_matrix_grips, as_numpy, glue, on_reset, on_step,  # noqa: E402
                     rotation, set_glue_on)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the steel plate: loose, along y; its frame: origin at the middle of its underside --------------------------------
PX, PY = 0.30, 0.07
PLATE = (0.09, 0.22, 0.005)  # across (x), long (y), thick
PLATE_MASS = 0.77
PLATE_RIGHT = PY - PLATE[1] / 2  # its right end

# -- the pot magnet: a steel cup with the magnet in it, a stud and a knob on top; frame: origin at its face's middle --
CUP_R, CUP_H = 0.016, 0.008
STUD_R, STUD_H = 0.006, 0.030
KNOB_R, KNOB_H = 0.009, 0.010
MAGNET_MASS = 0.08
MX0, MY0 = PX, PLATE_RIGHT + 0.03  # on the plate, 3 cm from its right end

# -- the pull, on the magnet and back on the plate: face on: F_MAX times the share of its face over the plate (it fades
# as it slides off the edge), less as it is lifted; beside the plate, F_SIDE / (1 + (gap / GAP0)^3) toward its nearest
# point, the gap measured from the magnet's rim: 1 N at 1 cm, 0.18 N at 2 cm (less than the table's friction on it) --
F_MAX, F_SIDE, GAP0 = 20.0, 3.0, 0.008
OFF = 0.02  # its rim this far from the plate: it is off
BACK = 0.004  # once off, back within this: it has snapped back
CLEAR = 0.05  # set down with its rim at least this far from the plate
STEEL, DARK, RED, GREY = (0.62, 0.63, 0.66), (0.16, 0.16, 0.18), (0.80, 0.15, 0.12), (0.45, 0.46, 0.48)

# -- a small steel nail on the table, right of the plate: brought within PICK of the magnet's face (once the magnet is
# off the plate), it jumps onto the face and sticks; its origin at the middle of its shank -----------------------------
NAIL_AT = np.array([0.30, -0.17])
NAIL_L, NAIL_R, HEAD_R, HEAD_T = 0.080, 0.0025, 0.0055, 0.002  # long: it sticks out either side of the magnet
NAIL_MASS = 0.012
PICK = 0.018

# -- scoring ----------------------------------------------------------------------------------------------------------
LET_GO = 0.03
SLID = 0.02
NAIL_UP = 0.03  # the nail held this far above the table, on the magnet: picked up
HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _plate() -> Body:
    return Body.build("plate", pos=(PX, PY, TOP + 0.0005), mass=PLATE_MASS, friction=(0.4, 0.35), color=GREY,
                      shapes=[BoxShape((0.0, 0.0, PLATE[2] / 2), PLATE, GREY)])


def _magnet() -> Body:
    shapes = [CylShape((0.0, 0.0, CUP_H / 2), CUP_R, CUP_H, RED),
              CylShape((0.0, 0.0, CUP_H + STUD_H / 2), STUD_R, STUD_H, STEEL),
              CylShape((0.0, 0.0, CUP_H + STUD_H + KNOB_H / 2), KNOB_R, KNOB_H, DARK)]
    return Body.build("magnet", pos=(MX0, MY0, TOP + 0.0005 + PLATE[2]), mass=MAGNET_MASS, shapes=shapes,
                      friction=(0.3, 0.25), color=RED, com=(0.0, 0.0, 0.006))


def _nail() -> Body:
    """Lying on the table along x, its head toward the arms."""
    shapes = [CylShape((0.0, 0.0, 0.0), NAIL_R, NAIL_L, STEEL, axis="X"),
              CylShape((-NAIL_L / 2, 0.0, 0.0), HEAD_R, HEAD_T, STEEL, axis="X")]
    return Body.build("nail", pos=(NAIL_AT[0], NAIL_AT[1], TOP + HEAD_R), mass=NAIL_MASS, shapes=shapes,
                      friction=(0.6, 0.5), color=STEEL)


PLATE_B, MAGNET = _plate(), _magnet()
OBJECTS = [PLATE_B, MAGNET, _nail()]
NAIL0 = np.array([NAIL_AT[0], NAIL_AT[1], TOP + HEAD_R])
glue("nail_on", "magnet", "nail", anchor=tuple(NAIL0.tolist()))  # off until the magnet picks the nail up


# -- followed after every control step: the pull, off and back --------------------------------------------------------------

def _fresh() -> dict:
    return {"off": False, "snapped": False, "best_slide": 0.0, "tcps": None, "gap": 0.0, "nail_on": False,
            "nail_up": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("MAG_DEBUG")), "steps": 0}


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


def _nail_frames(pos0, rot0) -> None:
    """Where the nail sits on the magnet when the joint is on: ``pos0``/``rot0`` in the magnet's frame."""
    import bodykit
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint(get_current_stage().GetPrimAtPath(bodykit._GLUE_PATHS["nail_on"]))
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos0]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_wxyz(np.asarray(rot0, float))))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    _nail_frames(NAIL0 - np.array([MX0, MY0, TOP + 0.0005 + PLATE[2]]), np.eye(3))
    set_glue_on("nail_on", False)


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _geometry(p_pos, p_rot, m_pos):
    """The magnet's face against the plate: (share of the face over the plate, its height above the plate's top, the
    gap from its rim to the plate, the unit vector toward the plate's nearest point, that point in the plate's frame)."""
    v = p_rot.T @ (np.asarray(m_pos, float) - np.asarray(p_pos, float))
    hx, hy = PLATE[0] / 2, PLATE[1] / 2
    over = 1.0
    for c, h in ((v[0], hx), (v[1], hy)):
        over *= max(0.0, min(h, c + CUP_R) - max(-h, c - CUP_R)) / (2 * CUP_R)
    up = max(0.0, float(v[2]) - PLATE[2])
    near = np.clip(v, [-hx, -hy, 0.0], [hx, hy, PLATE[2]])
    d = p_rot @ (near - v)
    flat = math.hypot(near[0] - v[0], near[1] - v[1])
    gap = math.hypot(max(0.0, flat - CUP_R), up)
    n = float(np.linalg.norm(d))
    toward = d / n if n > 1e-6 else -(p_rot @ np.array([0.0, 0.0, 1.0]))
    return over, up, gap, toward, near


def _gap(p_pos, p_rot, m_pos, m_rot) -> float:
    return _geometry(p_pos, p_rot, m_pos)[2]


def _set_force(asset, f_world, rot, at=None) -> None:
    """A force (world frame) on a body until the next step, at ``at`` (its own frame) or its origin."""
    import torch

    f = rot.T @ np.asarray(f_world, float)
    t = torch.tensor(f, dtype=torch.float32, device=asset.device).view(1, 1, 3)
    pos = None if at is None else torch.tensor(np.asarray(at, float), dtype=torch.float32, device=asset.device).view(1, 1, 3)
    asset.permanent_wrench_composer.set_forces_and_torques(forces=t, torques=torch.zeros_like(t), positions=pos,
                                                           is_global=False)


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    p = as_numpy(base.scene["plate"].data.root_link_pose_w)[0]
    m = as_numpy(base.scene["magnet"].data.root_link_pose_w)[0]
    p_pos, p_rot, m_pos, m_rot = p[:3] - origin, rotation(p[3:7]), m[:3] - origin, rotation(m[3:7])
    over, up, gap, toward, near = _geometry(p_pos, p_rot, m_pos)
    down = -(p_rot @ np.array([0.0, 0.0, 1.0]))
    f = F_MAX * over / (1.0 + (up / GAP0) ** 3) * down  # face on
    if over < 1.0:  # beside it, or partly off: toward its nearest point
        f = f + F_SIDE / (1.0 + (gap / GAP0) ** 3) * toward
    _set_force(base.scene["magnet"], f, m_rot)  # at its face
    _set_force(base.scene["plate"], -f, p_rot, at=near)  # and back on the plate where it pulls
    _EP["tcps"] = _tcps(base, origin)
    _EP["gap"] = gap
    slide = float(abs((p_rot.T @ (m_pos - p_pos))[1] - (MY0 - PY)))
    _EP["best_slide"] = max(_EP["best_slide"], slide)
    if gap >= OFF:
        _EP["off"] = True
    elif _EP["off"] and gap <= BACK:
        _EP["snapped"] = True
    n = as_numpy(base.scene["nail"].data.root_link_pose_w)[0]
    n_pos, n_rot = n[:3] - origin, rotation(n[3:7])
    _EP["nail_up"] = float(n_pos[2] - TOP)
    face = m_pos  # the middle of the magnet's face
    if not _EP["nail_on"] and _EP["off"] and float(np.linalg.norm(n_pos - face)) < PICK and m_rot[2, 2] > 0.8:
        # the nail jumps onto the face and sticks there, lying flat across it
        _nail_frames((0.0, 0.0, -NAIL_R - 0.0003), m_rot.T @ n_rot)
        set_glue_on("nail_on", True)
        _EP["nail_on"] = True
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 10 == 0:
            st = _status_from(m_pos, gap)
            tl, tr = _EP["tcps"]
            print(f"[mag] t={_DEBUG['steps'] / 20:5.1f}s plate={np.round(p_pos * 1000).astype(int).tolist()} "
                  f"magnet={np.round(m_pos * 1000).astype(int).tolist()} gap={gap * 1000:6.1f}mm pull={np.linalg.norm(f):5.2f}N "
                  f"off={_EP['off']} snapped={_EP['snapped']} let_go={st['let_go']} nail_on={_EP['nail_on']} "
                  f"nail_up={_EP['nail_up'] * 1000:5.1f}mm nail-face={np.round((n_pos - face) * 1000).astype(int).tolist()} "
                  f"left TCP={np.round(tl * 1000).astype(int).tolist()} "
                  f"right TCP-knob={np.round((tr - m_pos - [0.0, 0.0, CUP_H + STUD_H + KNOB_H / 2]) * 1000).astype(int).tolist()} "
                  f"score={_points(st)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def _status_from(m_pos, gap) -> dict:
    let_go = _EP["tcps"] is None or all(float(np.linalg.norm(t - (np.asarray(m_pos) + [0.0, 0.0, 0.02]))) >= LET_GO
                                        for t in _EP["tcps"])
    return {"gap": gap, "off": _EP["off"], "snapped": _EP["snapped"], "let_go": let_go,
            "slid": _EP["best_slide"] >= SLID, "nail": _EP["nail_on"] and _EP["nail_up"] >= NAIL_UP}


def _status(objects) -> dict:
    p, m = objects["plate"], objects["magnet"]
    gap = _gap(np.asarray(p.pos, float), rotation(p.quat_xyzw), np.asarray(m.pos, float), rotation(m.quat_xyzw))
    return _status_from(m.pos, gap)


def _points(st) -> float:
    if st["snapped"]:
        return 0.0
    if st["gap"] >= CLEAR and st["nail"]:
        return 1.0
    if st["gap"] >= OFF:
        return 0.5
    return 0.25 if st["slid"] else 0.0


def score(objects) -> float:
    """0.0 if the magnet, once off the plate, came back onto it (snapped back). Otherwise 1.0 with it at least 5 cm from
    the plate and the nail picked up on it (at least 3 cm off the table), the plate let go (both TCPs at least 3 cm from
    it) and the other arm back where it started; 0.5 with it off the plate (at least 2 cm from it at the end); 0.25 if
    it was slid at least 2 cm along the plate at some point; else 0.0."""
    pts = _points(_status(objects))
    if pts >= 1.0 and not (_plate_let_go(objects) and _other_hand_home(objects["magnet"].pos)):
        return 0.5  # off and the nail picked up, but the plate still held or the other arm not back
    return pts


_HOME: dict = {}


def _other_hand_home(m_pos) -> bool:
    """The arm not holding the magnet (the one whose TCP is farther from it) back where it started."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    k = int(np.argmax([float(np.linalg.norm(t - np.asarray(m_pos, float))) for t in tcps]))
    return float(np.linalg.norm(tcps[k] - _HOME["tcps"][k])) < HOME_WITHIN


def _plate_let_go(objects) -> bool:
    """Both grippers off the plate: each TCP at least LET_GO from the plate's outline."""
    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    p_pos, p_rot = np.asarray(objects["plate"].pos, float), rotation(objects["plate"].quat_xyzw)
    half = np.array(PLATE) / 2
    return all(float(np.linalg.norm(np.maximum(np.abs(p_rot.T @ (t - p_pos)) - half, 0.0))) >= LET_GO for t in tcps)


def success(objects) -> bool:
    """Full score with the magnet held still: the trial ends there."""
    return score(objects) >= 1.0 and objects["magnet"].resting()


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand comes straight down on the plate's left end, fingers closed, and presses it onto the table. The right
# hand comes straight down on the magnet's knob, grips it and slides the magnet along the plate and off its right end,
# and carries it on, clear of the plate. The left lets go of the plate and goes back to where it started. The right
# brings the magnet down over the nail on the table: the nail jumps onto its face; it lifts the magnet with the nail on
# it and turns it a quarter turn, showing the nail stuck there.

GRIP_L = grip_rotation(0.0)  # straight down, fingers closing along y (the left hand, pressing)
GRIP_R = grip_rotation(0.0)  # straight down, fingers closing along y (the pull is along y: a finger pushes it)
PRESS_AT = PLATE_RIGHT + PLATE[1] - 0.025  # the left hand presses here
PRESS = 0.016  # aim the fingertips this far into the plate (the arm gives: it presses it down)
KNOB_IN_HAND = 0.000  # the right TCP at the knob's middle
SET_AT = PLATE_RIGHT - CUP_R - 0.065  # the magnet set down here (its face's edge 6.5 cm from the plate)
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


def _knob_tcp(y: float, on_plate: bool) -> np.ndarray:
    base = TOP + 0.0005 + (PLATE[2] if on_plate else 0.0)
    return np.array([MX0, y, base + CUP_H + STUD_H + KNOB_H / 2 + KNOB_IN_HAND])


PRESSED_AT = LEAD_S + 1.0 + 1.0


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


def _rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _left(release_at: float, total: float) -> list[Waypoint]:
    press = np.array([PX, PRESS_AT, TOP + PLATE[2] + 0.013 - PRESS])
    out = _lead_in("left", press + [0.0, 0.0, 0.04], GRIP_L, "over the plate's left end", FINGER_CLOSED)
    out += [_w(1.0, press + [0.0, 0.0, 0.01], FINGER_CLOSED, "onto the plate", GRIP_L),
            _w(1.0, press, FINGER_CLOSED, "press it down", GRIP_L)]
    t = sum(w.duration for w in out)
    out.append(_w(max(release_at - t, 0.3), press, FINGER_CLOSED, "hold it down while the magnet is slid off", GRIP_L))
    out.append(_w(1.0, press + [0.0, 0.0, 0.05], FINGER_CLOSED, "rise", GRIP_L))
    out += _go_home("left", press + [0.0, 0.0, 0.05], GRIP_L, FINGER_CLOSED)
    t = sum(w.duration for w in out)
    out.append(_w(max(total - t, 0.5), out[-1].pos, FINGER_CLOSED, "wait at the start", out[-1].yaw))
    return out


FACE_TO_TCP = CUP_H + STUD_H + KNOB_H / 2 + KNOB_IN_HAND  # the right TCP this far above the magnet's face


def _right() -> tuple[list[Waypoint], float]:
    start = _knob_tcp(MY0, True)
    edge = _knob_tcp(PLATE_RIGHT - CUP_R - 0.004, True)  # just off the plate's end, still at the plate's height
    out = _lead_in("right", start + [0.0, 0.0, 0.04], GRIP_R, "over the magnet's knob", FINGER_OPEN)
    out += [_w(1.0, start, FINGER_OPEN, "down round the knob", GRIP_R),
            _w(0.8, start, FINGER_CLOSED, "grip it", GRIP_R)]
    t = sum(w.duration for w in out)
    over_nail = np.array([NAIL_AT[0], NAIL_AT[1], TOP + 0.03 + FACE_TO_TCP])
    on_nail = np.array([NAIL_AT[0], NAIL_AT[1], TOP + 2 * HEAD_R + 0.006 + FACE_TO_TCP])
    up = on_nail + [0.0, 0.0, 0.07]
    out += [_w(max(PRESSED_AT - t, 0.2) + 0.3, start, FINGER_CLOSED, "wait for the plate to be held", GRIP_R),
            _w(1.5, edge, FINGER_CLOSED, "slide it along the plate and off its end", GRIP_R),
            _w(1.5, _knob_tcp(SET_AT, False) + [0.0, 0.0, 0.02], FINGER_CLOSED, "carry it on, clear of the plate", GRIP_R)]
    clear_at = sum(w.duration for w in out)
    out += [_w(1.5, over_nail, FINGER_CLOSED, "over the steel nail", GRIP_R),
            _w(1.2, on_nail, FINGER_CLOSED, "down to it: the nail jumps onto the magnet", GRIP_R),
            _w(0.6, on_nail, FINGER_CLOSED, "down to it: the nail jumps onto the magnet", GRIP_R),
            _w(1.2, up, FINGER_CLOSED, "lift the magnet with the nail on it", GRIP_R),
            _w(2.0, up, FINGER_CLOSED, "turn it: the nail stays on", _rz(-math.pi / 2) @ GRIP_R),
            _w(1.0, up, FINGER_CLOSED, "hold it up: the trial ends once the full score has held 1 s",
               _rz(-math.pi / 2) @ GRIP_R)]
    return out, clear_at


def demo():
    right, clear_at = _right()
    t_r = sum(w.duration for w in right)
    total = max(t_r, clear_at + 1.0 + LEAD_S) + 3.0
    right.append(_w(total - t_r, right[-1].pos, FINGER_CLOSED, "hold it up", right[-1].yaw))
    return {"left": _left(clear_at, total), "right": right}
