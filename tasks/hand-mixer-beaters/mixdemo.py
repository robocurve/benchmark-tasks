"""The scripted demo for hand_mixer_beaters.py: plans each stage from the simulator's object poses.

Scripted demo, reads object poses from the simulator (mixer, beaters, spindle angles) and the arms' own joint
readings to plan its motions. It moves the parts only through the grippers, by contact. Stages:

0  left hand takes the mixer's handle (top strut) and holds it; right hand picks the collared beater by its stem
1  right hand lines the collared beater up with the larger socket, fins turned to the spindle's slots as read from
   the simulator, and pushes the stem in until the fins rest on the spindle
2  slot search: turns the beater a few degrees each way under a light push until the fins drop in
3  firm push past the spring clip, hold, let go, hand up
4-7 the same for the plain beater in the smaller socket
8  left hand lets go; both arms back to the ready pose

The simulated arms are soft, so each settles short of its target under load. The demo closes that gap from the joint
angles it reads (a bounded integral correction of the TCP pose), except during the light push of the slot search,
when the correction is held so the push stays light.
"""

from __future__ import annotations

import math
import os

import numpy as np

_PLAN_LOG = open(os.environ["MIX_DEBUG"] + ".plan", "w") if os.environ.get("MIX_DEBUG") else None
SAG_GAIN = 0.25
SAG_MAX = 0.30
UNWIND = ("let go",)
UNWIND_RATE = 0.8
FREEZE = ("search",)  # labels during which the correction is held (a light push)
FAIL = os.environ.get("MIX_FAIL", "")  # wrong on purpose: fixed, nobrace, swap, holding

N_STAGES = 9
OPEN_PICK = -0.020  # finger target: 40 mm opening
LET_GO = -0.010  # 20 mm: clear of the stem, not of the beater beside it
STEM_GRIP = 0.0  # fully closed on the 6 mm stem: 3 mm squeeze per finger, about 15 N
STRUT_GRIP = -(0.0125 - 0.006)  # 6 mm past the 25 mm strut's faces: about 30 N
TCP_ABOVE_STEM = 0.007
S_FAR, S_PRE, S_NEAR, S_ENTER, S_SEARCH = -0.130, -0.060, -0.008, 0.024, 0.0300
OVERPUSH = 0.006
SEARCH_DEG = (4.0, -4.0, 8.0, -8.0, 0.0)


class _Globals:
    """The task file's namespace, seen as a module (yam_arena's loader does not put it in sys.modules)."""

    def __init__(self, g: dict) -> None:
        self.__dict__ = g


_TASK: list = []


def bind(task_globals: dict) -> None:
    _TASK[:] = [_Globals(task_globals)]


def _m():
    return _TASK[0]


def _rot_axis(axis, deg: float) -> np.ndarray:
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    t = math.radians(deg)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def _grip(yaw: float) -> np.ndarray:
    from yam_arena.demo import grip_rotation

    while yaw > math.pi / 2:
        yaw -= math.pi
    while yaw < -math.pi / 2:
        yaw += math.pi
    return np.asarray(grip_rotation(yaw), float)


def _wp(dur, pos, finger, label, rot):
    from yam_arena.demo import Waypoint

    return Waypoint(float(dur), tuple(float(v) for v in pos), float(finger), label, np.asarray(rot, float))


def _park(arm: str, seconds: float, label: str = "ready"):
    from yam_arena import ik, station
    from yam_arena.constants import FINGER_OPEN, READY
    from yam_arena.demo import Waypoint

    p, r = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return Waypoint(seconds, tuple(np.asarray(p, float).tolist()), FINGER_OPEN, label, np.asarray(r, float))


def _pad(plans: dict) -> dict:
    tl = {a: sum(w.duration for w in p) for a, p in plans.items()}
    top = max(tl.values())
    for a, p in plans.items():
        if tl[a] < top - 1e-9:
            last = p[-1]
            p.append(type(last)(top - tl[a], last.pos, last.finger, last.label if last.label in ("brace", "hold beater")
                                else "wait", last.yaw))
    return plans


def demo():
    m = _m()
    if m.PROBE:
        return {arm: [_park(arm, 14.0, "look")] for arm in ("left", "right")}
    return {arm: [_park(arm, 0.3, "look")] for arm in ("left", "right")}


# -- what the demo reads -------------------------------------------------------------------------------------------------
def _poses(m) -> dict:
    """Mixer, beaters (position, rotation) and spindle angles: from the simulator, or the nominal layout (fixed)."""
    if FAIL == "fixed":
        lay = m.layout(None)
        out = {n: (np.asarray(v[0], float), m.bodykit.rotation(v[1])) for n, v in lay.items()
               if not n.startswith("_")}
        out["_spin"] = {k: 0.0 for k in m.SOCKETS}
        return out
    st = m._EP["st"]
    out = {n: (np.asarray(st[n].pos, float), st[n].rotation()) for n in ("mixer", "beater_collared", "beater_plain")}
    out["_spin"] = {k: m.spindle_deg(st["mixer"], st[f"spindle_{k}"]) for k in m.SOCKETS}
    return out


_S: dict = {}


BRACE_Z = 0.150  # the closed fingertips press on the back of the handle bar this high above the table
BRACE_IN = -0.004  # and stop this short of it: the hand holds its place, so it pushes back only when the mixer moves
FINGER_HALF = 0.019  # the closed hand's half-width across the closing direction (measured in run d5: it meets the handle 19 mm from its centre line)


def _strut(m, P):
    """(TCP behind the handle, TCP pressing on it, rotation): closed fingers, straight down, closing across the
    mixer's socket axis, so the fingers' straight side edges meet the handle bar's back (the fingers flare out toward
    their roots along the closing direction, so that face would reach into the handle)."""
    pm, rm = P["mixer"]
    yaw = math.atan2(rm[1, 0], rm[0, 0])
    tip_z = m.TOP + BRACE_Z
    press = pm + rm @ np.array([0.085 + FINGER_HALF - BRACE_IN, 0.0, 0.0])
    press[2] = tip_z + 0.006
    behind = press + rm[:, 0] * 0.05
    from yam_arena.demo import grip_rotation

    best = None
    for psi in (yaw, yaw + math.pi):  # either way round: take the one the arm reaches
        rot = np.asarray(grip_rotation(psi), float)
        err = _chain_err(None, [(behind + np.array([0.0, 0.0, 0.03]), rot), (behind, rot), (press, rot)], "left")
        if best is None or err < best[0]:
            best = (err, rot)
    return behind, press, best[1]


def _stage_pick(m, kind: str, first: bool) -> dict:
    P = _poses(m)
    pb, rb = P[f"beater_{kind}"]
    g = pb + rb @ np.array([m.GRIP_X, 0.0, 0.0])
    sock = dict(ORDER)[kind]
    rot = _plan_grasp(m, P, kind, sock, g)
    _S["grasp_rot"] = rot
    at = g + np.array([0.0, 0.0, TCP_ABOVE_STEM])
    up = at + np.array([0.0, 0.0, 0.08])
    lift = at + np.array([0.0, 0.0, 0.10])
    right = [_wp(3.0 if first else 2.5, up, OPEN_PICK, f"over the {kind} beater", rot),
             _wp(1.5, at, OPEN_PICK, "down round the stem", rot),
             _wp(0.8, at, STEM_GRIP, "close on the stem", rot),
             _wp(1.0, lift, STEM_GRIP, "lift", rot)]
    if first:
        behind, press, srot = _strut(m, P)
        _S["behind"] = behind
        if FAIL == "nobrace":
            left = [_park("left", 0.5, "ready")]
        else:
            from yam_arena.constants import FINGER_CLOSED

            left = [_wp(3.0, behind + np.array([0.0, 0.0, 0.03]), FINGER_CLOSED, "behind the handle", srot),
                    _wp(1.0, behind, FINGER_CLOSED, "down behind the handle", srot),
                    _wp(1.2, press, FINGER_CLOSED, "brace", srot)]
        _S["brace"] = left[-1]
    else:
        left = [_hold_left(0.5)]
    return _pad({"left": left, "right": right})


def _hold_left(seconds: float):
    w = _S["brace"]
    return type(w)(seconds, w.pos, w.finger, w.label, w.yaw)


def _targets(m, kind: str, sock: str) -> None:
    """Measure the grasp, then the TCP poses that put the stem tip at depth s on the socket axis, fins on the slots."""
    P = _poses(m)
    pb, rb = P[f"beater_{kind}"]
    if FAIL == "fixed":  # the nominal grasp of the nominal beater pose (no sensing), lifted 10 cm
        pb = pb + np.array([0.0, 0.0, 0.10])
        rg = _S["grasp_rot"]
        pg = pb + rb @ np.array([m.GRIP_X, 0.0, 0.0]) + np.array([0.0, 0.0, TCP_ABOVE_STEM])
        q0 = None
    else:
        pg, rg = m._EP["tcp"]["right"]
        q0 = m._EP["q"]["right"]
    r_t = rg.T @ rb
    p_t = rg.T @ (pb - pg)
    pm, rm = P["mixer"]
    f = pm + rm @ np.array([m.FACE_X, m.SOCK_Y[sock], m.Z_S])
    axis = rm[:, 0]
    best = None
    for k in range(4):
        r_b = rm @ m._rx(math.radians(P["_spin"][sock] + 90.0 * k))
        poses = _insert_poses(m, f, axis, r_b, r_t, p_t)
        err = _chain_err(q0, poses) + _lean_back(poses)
        if best is None or err < best[0]:
            best = (err, r_b)
    print(f"mixdemo: {kind} -> {sock}: fins chosen, IK error {best[0]:.1f}", flush=True)
    _S["ins"] = dict(f=f, axis=axis, r_b=best[1], r_t=r_t, p_t=p_t, kind=kind, sock=sock, step=0, rm=rm)


def _insert_poses(m, f, axis, r_b0, r_t, p_t) -> list:
    out = []
    for s, droll in ((S_PRE, 0.0), (S_ENTER, 0.0), (m.D_FULL + OVERPUSH, 0.0), (S_SEARCH, 8.0), (S_SEARCH, -8.0)):
        r_b = _rot_axis(axis, droll) @ r_b0
        r_g = r_b @ r_t.T
        out.append((f + axis * s - r_g @ p_t, r_g))
    return out


def _lean_back(poses) -> float:
    """A cost for a hand that leans back toward the robot at full depth: its wrist then sits beside the mixer and
    pushes it (seen in run look7). 100 per unit of the approach direction's -x component."""
    return 100.0 * max(0.0, -float(np.asarray(poses[2][1])[0, 2]))


def _chain_err(q0, poses, arm: str = "right") -> float:
    """Worst IK miss (mm + degrees) going through ``poses`` in order from joint angles ``q0`` (None: the ready pose),
    each in 8 steps along the straight line, as the scripted policy does."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    q = np.array(READY if q0 is None else q0, float)
    worst = 0.0
    for pos, rot in poses:
        p0, r0 = ik.tcp_pose(q, station.BASES[arm])
        for i in range(1, 5):
            a = i / 4
            q, pe, re = ik.solve(np.asarray(p0) + a * (np.asarray(pos) - np.asarray(p0)),
                                 ik.matrix_to_quat_xyzw(ik.slerp_matrix(np.asarray(r0), np.asarray(rot), a)), q,
                                 base_pos=station.BASES[arm], iters=60)
        worst = max(worst, 1000 * float(np.linalg.norm(pe)) + math.degrees(float(np.linalg.norm(re))))
    return worst


def _plan_grasp(m, P, kind: str, sock: str, g) -> np.ndarray:
    """The pick rotation (the hand's yaw, either way round) whose whole path, pick to search, the arm reaches best."""
    pb, rb = P[f"beater_{kind}"]
    yaw = math.atan2(rb[1, 0], rb[0, 0])
    pm, rm = P["mixer"]
    f = pm + rm @ np.array([m.FACE_X, m.SOCK_Y[sock], m.Z_S])
    best = None
    for psi in (yaw, yaw + math.pi):
        from yam_arena.demo import grip_rotation

        rg = np.asarray(grip_rotation(psi), float)
        at = g + np.array([0.0, 0.0, TCP_ABOVE_STEM])
        lift = at + np.array([0.0, 0.0, 0.10])
        r_t = rg.T @ rb
        p_t = rg.T @ (pb + np.array([0.0, 0.0, 0.10]) - lift)
        for k in range(4):
            r_b = rm @ m._rx(math.radians(P["_spin"][sock] + 90.0 * k))
            poses = _insert_poses(m, f, rm[:, 0], r_b, r_t, p_t)
            err = _chain_err(None, [(at, rg), (lift, rg)] + poses) + _lean_back(poses)
            if best is None or err < best[0]:
                best = (err, rg)
    print(f"mixdemo: {kind} grasp planned, IK error {best[0]:.1f}", flush=True)
    return best[1]


PREF_TILT = float(os.environ.get("MIX_PREF_TILT", "35.0"))  # degrees: the hand leans this far forward, at best


def hand_tilt(r_g: np.ndarray, axis) -> float:
    """How far the hand's approach leans from straight down, about the stem axis (degrees; + leans away from the
    robot, -x toward it)."""
    a = r_g[:, 2]
    away = np.cross(np.array([0.0, 0.0, 1.0]), np.asarray(axis, float))  # horizontal, square to the stem
    if away[0] < 0:
        away = -away
    return math.degrees(math.atan2(float(a @ away), float(-a[2])))


def choose_fins(m, rm, spin_deg: float, r_t) -> np.ndarray:
    """The beater rotation that puts its fins on the slots with the hand leaning nearest PREF_TILT forward (the right
    arm leans forward easily, back toward the robot hardly)."""
    best = None
    for k in range(-3, 4):
        r_b = rm @ m._rx(math.radians(spin_deg + 90.0 * k))
        tilt = hand_tilt(r_b @ r_t.T, rm[:, 0])
        if best is None or abs(tilt - PREF_TILT) < best[0]:
            best = (abs(tilt - PREF_TILT), r_b)
    return best[1]


def _tcp_at(s: float, droll: float = 0.0):
    t = _S["ins"]
    r_b = _rot_axis(t["axis"], droll) @ t["r_b"]
    r_g = r_b @ t["r_t"].T
    p_g = t["f"] + t["axis"] * s - r_g @ t["p_t"]
    return p_g, r_g


def _stage_approach(m, kind: str, sock: str) -> dict:
    _targets(m, kind, sock)
    # first draw the lifted beater back from the mixer (it was lying pointing at it) and up, then line it up well out
    t = _S["ins"]
    p0, r0 = (m._EP["tcp"]["right"] if FAIL != "fixed" else (None, None))
    if p0 is None:
        p0, r0 = _tcp_at(S_FAR)
        p0 = p0 + np.array([0.0, 0.0, 0.03])
    back = np.asarray(p0, float) - t["axis"] * 0.09 + np.array([0.0, 0.0, 0.03])
    right = [_wp(1.2, back, STEM_GRIP, "draw back from the mixer", r0)]
    p_far, r_far = _tcp_at(S_FAR)
    right.append(_wp(2.2, p_far + np.array([0.0, 0.0, 0.03]), STEM_GRIP, "line the stem up, well out", r_far))
    for dur, s, label in ((1.5, S_PRE, "line the stem up with the socket"), (1.5, S_NEAR, "to the socket"),
                          (0.6, S_NEAR, "to the socket")):
        p, r = _tcp_at(s)
        right.append(_wp(dur, p, STEM_GRIP, label, r))
    return _pad({"left": [_hold_left(0.5)], "right": right})


def _refine(m) -> dict:
    """Stem 8 mm short of the face: measure the beater in the hand again (it shifts in the fingers on the way) and the
    socket, keep the chosen fin alignment, and plan the last stretch in."""
    t = _S["ins"]
    if FAIL != "fixed":
        P = _poses(m)
        pb, rb = P[f"beater_{t['kind']}"]
        pg, rg = m._EP["tcp"]["right"]
        pm, rm = P["mixer"]
        f = pm + rm @ np.array([m.FACE_X, m.SOCK_Y[t["sock"]], m.Z_S])
        before = np.linalg.norm(np.cross(rm[:, 0], pb - f))
        # keep the fins where they were chosen: the same turn about the new axis
        t["r_b"] = rm @ (t["rm"].T @ t["r_b"]) if "rm" in t else t["r_b"]
        t.update(f=f, axis=rm[:, 0], r_t=rg.T @ rb, p_t=rg.T @ (pb - pg), rm=rm)
        print(f"mixdemo: refine {t['kind']}: tip {1000 * before:.1f} mm off the axis", flush=True)
    right = []
    for dur, s, label in ((0.8, S_NEAR, "to the socket"), (1.5, S_ENTER, "stem into the socket"),
                          (1.0, S_SEARCH, "search")):
        p, r = _tcp_at(s)
        right.append(_wp(dur, p, STEM_GRIP, label, r))
    return _pad({"left": [_hold_left(0.5)], "right": right})


def _depth(m) -> float:
    t = _S["ins"]
    st = m._EP["st"]
    return m.in_socket(st["mixer"], st[f"beater_{t['kind']}"], t["sock"])["depth"]


def _stage_search(m):
    """One step of the slot search; (plans, done)."""
    t = _S["ins"]
    if not t.get("refined"):
        t["refined"] = True
        return _refine(m), False
    if FAIL == "fixed" or _depth(m) >= m.FINS_S - 0.0005 or t["step"] >= len(SEARCH_DEG):
        return None, True
    d = SEARCH_DEG[t["step"]]
    t["step"] += 1
    p, r = _tcp_at(S_SEARCH, d)
    return _pad({"left": [_hold_left(0.6)], "right": [_wp(0.6, p, STEM_GRIP, "search", r)]}), False


def _stage_push(m, last: bool) -> dict:
    p_in, r = _tcp_at(m.D_FULL + OVERPUSH)
    right = [_wp(1.5, p_in, STEM_GRIP, "push past the clip", r), _wp(1.0, p_in, STEM_GRIP, "hold the push", r)]
    if FAIL == "holding" and last:
        right.append(_wp(4.0, p_in, STEM_GRIP, "hold beater", r))
        return _pad({"left": [_hold_left(0.5)], "right": right})
    p_out, _ = _tcp_at(m.D_FULL)
    right += [_wp(0.8, p_out, LET_GO, "let go", r), _wp(1.0, p_out + np.array([0, 0, 0.06]), LET_GO, "up", r)]
    return _pad({"left": [_hold_left(0.5)], "right": right})


def _stage_finish(m) -> dict:
    b = _S["brace"]
    behind = _S["behind"]
    if FAIL == "nobrace":
        left = [_park("left", 3.0)]
    else:
        left = [_wp(1.0, behind, b.finger, "let go", b.yaw),
                _wp(1.0, behind + np.array([0.0, 0.0, 0.03]), b.finger, "up", b.yaw), _park("left", 2.0)]
    right = [] if FAIL == "holding" else [_park("right", 3.0)]
    if FAIL == "holding":
        right = [_wp(4.0, _S["last_right"].pos, STEM_GRIP, "hold beater", _S["last_right"].yaw)]
    plans = _pad({"left": left + [_park("left", 2.5, "still")], "right": right + [type(left[0])(2.5, right[-1].pos,
                  right[-1].finger, "still", right[-1].yaw)]})
    return plans


ORDER = (("collared", "large"), ("plain", "small"))
if FAIL == "swap":
    ORDER = (("plain", "large"), ("collared", "small"))


def plan_stage(stage: int, module=None):
    """(plans or None, advance)."""
    m = _m()
    if not m._EP.get("st") or "tcp" not in m._EP:
        return None, False
    i, j = divmod(stage, 4)
    if stage >= 8:
        return _stage_finish(m), True
    kind, sock = ORDER[i]
    if j == 0:
        return _stage_pick(m, kind, i == 0), True
    if j == 1:
        return _stage_approach(m, kind, sock), True
    if j == 2:
        plans, done = _stage_search(m)
        return plans, done
    plans = _stage_push(m, i == 1)
    _S["last_right"] = plans["right"][-1]
    return plans, True


# -- the policy hooks ----------------------------------------------------------------------------------------------------
def install() -> None:
    try:
        from inspect_robots.types import Action, ActionChunk
        from inspect_robots_yam.packing import STATE_KEY
        from yam_arena import ik, scripted
        from yam_arena.demo import Waypoint
    except Exception:  # the task file also loads where yam_scripted is not installed (the checker)
        return
    cls = scripted.YamScriptedPolicy
    if getattr(cls, "_mixdemo", False):
        return
    plain_reset, plain_act = cls.reset, cls.act

    def tcp_jacobian(q, base):
        p0, r0 = ik.tcp_pose(q, base)
        jac = np.empty((6, 6))
        for k in range(6):
            qp = np.array(q, float)
            qp[k] += 1e-4
            p, r = ik.tcp_pose(qp, base)
            jac[:3, k] = (p - p0) / 1e-4
            w = (r - r0) / 1e-4 @ r0.T
            jac[3:, k] = 0.5 * np.array([w[2, 1] - w[1, 2], w[0, 2] - w[2, 0], w[1, 0] - w[0, 1]])
        return jac

    def log_rot(r):
        c = np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0)
        th = math.acos(c)
        v = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
        return 0.5 * v if th < 1e-7 else th / (2.0 * math.sin(th)) * v

    def _mine(self) -> bool:
        return self._task is not None and hasattr(self._task.module, "mixdemo_task")

    def reset(self, scene):
        plain_reset(self, scene)
        self._mix_stage = 0 if _mine(self) and getattr(self._task.module, "LOOKS", 0) else -1
        self._mix_off = np.zeros(14)
        _S.clear()

    def _extend(self, plan: dict, t: float) -> None:
        for arm, more in plan.items():
            have = self._plans[arm]
            gap = t + self._dt - have.duration
            last = have.waypoints[-1]
            if gap > 1e-6:
                have.waypoints.append(Waypoint(gap, last.pos, last.finger, last.label, last.yaw))
            have.waypoints.extend(more)
        self._end = max((p.duration for p in self._plans.values()), default=0.0)

    def act(self, observation):
        if not _mine(self):
            return plain_act(self, observation)
        m = self._task.module
        t = float(observation.state_time)
        if 0 <= self._mix_stage < m.LOOKS and t + 1e-6 >= self._end - 0.05:
            more, advance = m.plan_stage(self._mix_stage, m)
            if advance:
                self._mix_stage += 1
            if more:
                _extend(self, more, t)
        labels = {arm: plan.target(t)[3] for arm, plan in self._plans.items()}
        chunk = plain_act(self, observation)
        planned = np.asarray(chunk.actions[0].data, float).copy()
        measured = np.asarray(observation.state[STATE_KEY], float)
        fields = [f"{int(round(t * 20)):4d} st={self._mix_stage}"]
        for arm, idx in (("left", np.r_[0:6]), ("right", np.r_[7:13])):
            base = self._bases[arm]
            p_plan, r_plan = ik.tcp_pose(planned[idx], base)
            p_meas, r_meas = ik.tcp_pose(measured[idx], base)
            lab = labels.get(arm)
            if lab in UNWIND:
                self._mix_off[idx] *= UNWIND_RATE
            elif lab not in FREEZE:
                err = np.r_[p_plan - p_meas, log_rot(r_plan @ r_meas.T)]
                jac = tcp_jacobian(measured[idx], base)
                dq = jac.T @ np.linalg.solve(jac @ jac.T + 1e-4 * np.eye(6), err)
                self._mix_off[idx] = np.clip(self._mix_off[idx] + SAG_GAIN * dq, -SAG_MAX, SAG_MAX)
            if _PLAN_LOG is not None:
                fields.append(f"{arm}: plan=({p_plan[0]:.4f},{p_plan[1]:.4f},{p_plan[2]:.4f}) "
                              f"meas=({p_meas[0]:.4f},{p_meas[1]:.4f},{p_meas[2]:.4f}) "
                              f"grip={measured[idx[-1] + 1]:.4f} off={np.linalg.norm(self._mix_off[idx]):.3f} [{lab}]")
        if _PLAN_LOG is not None:
            _PLAN_LOG.write(" ".join(fields) + "\n")
            _PLAN_LOG.flush()
        out = planned + self._mix_off
        return ActionChunk(actions=[Action(data=out, meta=chunk.actions[0].meta)], control_hz=chunk.control_hz)

    cls.reset, cls.act = reset, act
    cls._mixdemo = True
