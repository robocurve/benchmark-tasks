"""Scripted full-score demo (and negative variants) for the recessed-trim task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the trim's, the legs' and the
housing's poses through ``trimkit.bench_state()`` (read-only, like an electrician looking at the springs and up into
the housing at the brackets); it never writes a pose, a joint state or a score. Its joint commands are rate-limited
below the CLI's per-step delta guard and clipped to the URDF joint limits.

Plan (both arms, side pinch: each hand's fingers close tangentially across one spring's two legs, 8 mm above the coil,
the hand approaching horizontally from outside the trim):
1. Left hand pinches spring A, right hand spring B, where they stand on the carton (their angle is read from the trim's
   pose); closing the fingers squeezes each spring's legs together against their stop (about 17 N per finger).
2. Lift the trim, carry it under the housing while turning it so that its springs face the brackets (the bracket angle
   is read from the housing's pose), and hold it with the leg tips 20 mm below the bracket tabs.
3. Line the squeezed leg tips up with the slots (closed loop on the measured tips) and raise the trim until the tips
   are 8 mm above the tabs.
4. Open the left hand (spring A's legs spread in its bracket), then the right hand (spring B), and take both hands out.
5. Push the flange up with both hands' closed fingertips (left and right of the robot-side half of the flange) until it
   meets the ceiling, hold briefly, take the hands away and go back to READY.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because the
joint PD sags under load (pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "src"))

import fastik  # noqa: E402
import layout as L  # noqa: E402

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
FINGER_OPEN = 0.04695  # m per finger at grip 1.0
COMP_GAIN = 0.08
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
ROT_CORR_MAX = 0.12
PINCH_ABOVE_COIL = 0.010  # pinch height on the legs (finger pads clear the flange by 2 mm)
PINCH_R = L.LEG_R + 0.006  # TCP radius: legs 6 mm in front of the TCP (7 mm behind the pad tips)
APPROACH_BACK = 0.012  # pre-pinch stand-off (radial): pad tips 8 mm outside the legs
APPROACH_UP = 0.040  # and above the pinch height (the hands come down there, then move in)
HOVER_BELOW_TAB = 0.020  # leg tips below the tab top while lining up
INSERT_ABOVE_TAB = 0.004  # leg tips above the tab top when the springs are let go (hooks 3 mm above it)
TIP_TOL_M = 0.0012  # tips within 1.2 mm of the slot centre (radial and tangential) before rising
RETRACT = 0.010  # hands move radially out after letting go (pad tips 6 mm outside the legs)
SIDESTEP = 0.080  # then (fingers half closed) sideways toward the robot until both fingers are past the flange rim
PUSH_BETA = 110.0  # push points (deg from the trim's spring A direction, either side)
PUSH_R = 0.087
PUSH_ELEV = 25.0
CARRY_Z = 0.10  # trim lifted this far off the carton before the carry


# extra IK seeds (joint grid) for poses the nearest-solution search does not reach from the current configuration
SEEDS = tuple(
    np.array([j1, j2, j3, j4, 0.0, 0.0])
    for j1 in (-0.6, 0.0, 0.6)
    for j2 in (0.6, 1.4, 2.2)
    for j3 in (0.6, 1.4, 2.2)
    for j4 in (-1.0, 0.0, 1.0)
)


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


class Stop(Exception):
    pass


def _rotvec(r):
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(r).as_rotvec()


def _rotm(v):
    from scipy.spatial.transform import Rotation

    return Rotation.from_rotvec(v).as_matrix()


class Arm:
    """One arm: smooth Cartesian moves (URDF IK), each starting where the last command left it."""

    def __init__(self, name, base_pos):
        from yam_arena.constants import READY

        self.name, self.base = name, np.asarray(base_pos, float)
        self.q = np.array(READY, float)
        self.grip = 1.0
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
        self.offset = np.zeros(6)
        self.measured = self.q.copy()
        self.measured_grip = 1.0
        self.comp = True
        self.label = ""
        self.worst_mm = 0.0
        _, _, lo, hi, _ = fastik._chain()
        self.lo, self.hi = lo, hi
        self.last_cmd = None

    @property
    def cmd(self):
        c = np.r_[np.clip(self.q + self.offset, self.lo, self.hi), self.grip]
        lim = np.r_[RATE * 0.05 * (self.hi - self.lo), RATE * 0.05]
        if self.last_cmd is not None:
            c = np.clip(c, self.last_cmd - lim, self.last_cmd + lim)
        return c

    def commit(self, c):
        self.last_cmd = np.asarray(c, float).copy()

    def set_comp(self, mode):
        """True: joint integral (free moves and carrying); 'hold': offsets frozen (in contact); False: offsets decay."""
        self.comp = mode

    def observe(self, q):
        self.measured = np.asarray(q[:6], float)
        self.measured_grip = float(q[6])
        if self.comp == "hold":
            pass
        elif self.comp:
            self.offset = np.clip(
                self.offset + COMP_GAIN * (self.q - self.measured), -0.25, 0.25
            )
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        part before the fingers open."""
        self.set_comp("hold")
        self.q = self.measured.copy()
        self.pos, self.rot = fastik.tcp_pose(self.q, self.base)

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
        """IK from the current joint angles; other seeds only if that fails, and then the solution nearest the current
        angles (no jump to another arm configuration in mid-move)."""
        from yam_arena.constants import READY

        q, pe, re = fastik.solve(pos, rot, self.q, base=self.base, iters=80)
        if pe < 2e-4 and re < 3e-3:
            return q, pe, re
        q, pe, re = fastik.solve(pos, rot, q, base=self.base, iters=200)
        best = (q, pe, re)
        if pe < 5e-4 and re < 5e-3:
            return best
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])) + (SEEDS if best[1] > 5e-4 else ()):
            q2, pe2, re2 = fastik.solve(pos, rot, s, base=self.base, iters=200)
            cost2 = pe2 + 0.1 * re2 + 0.002 * np.abs(q2 - self.q).sum()
            if cost2 < best[1] + 0.1 * best[2] + 0.002 * np.abs(best[0] - self.q).sum():
                best = (q2, pe2, re2)
        return best

    def set_pose(self, pos, rot):
        q, pe, re = self.solve(pos, rot)
        if pe > 0.004 or re > math.radians(4):
            raise Stop(
                f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg"
            )
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        from yam_arena import ik

        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(
                p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s)
            )
            yield

    def move_joint(self, pos, rot, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to the IK solution of (pos, rot): for transits whose straight Cartesian path
        leaves the workspace."""
        from yam_arena import ik

        self.label = label or self.label
        q1, pe, re = self.solve(pos, rot)
        if pe > 0.002 or re > math.radians(2):
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        q0, g0 = self.q.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            self.q = q0 + ik.smoothstep(k / n) * (q1 - q0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield


def wait(seconds):
    for _ in range(max(1, round(seconds * CONTROL_HZ))):
        yield


def both(*gens):
    """Run arm generators in parallel (one control step each per yield)."""
    gens = list(gens)
    while gens:
        alive = []
        for g in gens:
            try:
                next(g)
                alive.append(g)
            except StopIteration:
                pass
        gens = alive
        if gens:
            yield




class Ctx:
    def __init__(self, emb=None):
        from yam_arena import station

        self.emb = emb
        self.left = Arm("left", station.BASES["left"])
        self.right = Arm("right", station.BASES["right"])
        self.arms = {"left": self.left, "right": self.right}
        self.spring_of = {"left": "A", "right": "B"}
        self.log = []
        self.step = 0
        self.G = {}
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)

    def poses(self):
        import trimkit

        return trimkit.bench_state()

    def pose(self, name="trim"):
        return self.poses()[name]

    def phi(self):
        """Bracket rotation, read from the housing body's pose (an electrician sees the brackets in the can)."""
        r = self.pose("housing")[1]
        return math.degrees(math.atan2(r[1, 0], r[0, 0]))

    def detail(self):
        import trimkit

        return trimkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if getattr(self, "replay", None) is not None:  # open-loop replay of a recorded command trace
            a = np.asarray(self.replay[min(self.step, len(self.replay) - 1)], float)
            self.left.commit(a[:7])
            self.right.commit(a[7:])
            return Action(data=a.copy())
        cl, cr = self.left.cmd, self.right.cmd
        self.left.commit(cl)
        self.right.commit(cr)
        return Action(data=pack(cl, cr))

    def note(self, tag, **kw):
        d = self.detail()
        row = dict(at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"), **kw)
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def trim_yaw(r):
    return math.degrees(math.atan2(r[1, 0], r[0, 0]))


def pinch_rot(radial):
    """Horizontal approach toward -radial, fingers closing along the tangential direction."""
    radial = np.asarray(radial, float)
    radial = radial / np.linalg.norm(radial)
    z = -radial
    x = np.cross(np.array([0.0, 0.0, 1.0]), radial)  # tangential
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def pinch_target(tp, tr, spring, back=0.0):
    """TCP pose (world) of the pinch on ``spring`` for trim pose (tp, tr)."""
    c, rad, tan = L.spring_frame(spring)
    p_t = rad * (PINCH_R + back) + np.array([0.0, 0.0, L.COIL_Z + PINCH_ABOVE_COIL])
    return tp + tr @ p_t, pinch_rot(tr @ rad)


def best_flip(arm, pos, rot):
    """The fingers' closing axis sign (x and y flipped together) that needs the least wrist roll."""
    best = None
    for sgn in (1.0, -1.0):
        r = rot.copy()
        r[:, 0] *= sgn
        r[:, 1] *= sgn
        q, pe, re = arm.solve(pos, r)
        cost = abs(q[5]) + pe * 100 + re * 10
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


LOOK_EL = 35.0  # camera hand: approach 35 deg above horizontal, 16 cm from the target, from the robot's side
LOOK_AZ = 140.0  # deg (left hand; the right hand mirrors it)
LOOK_DIST = 0.16


def look_pose(arm, target):
    """A pose that points the hand (and its wrist camera) up at ``target`` from below, in front and to the side."""
    a = math.radians(LOOK_AZ if arm.name == "left" else -LOOK_AZ)
    e = math.radians(LOOK_EL)
    d = np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), -math.sin(e)])
    p = np.asarray(target, float) + d * LOOK_DIST
    z = -d
    x0 = np.cross(z, [0.0, 0.0, 1.0])
    x0 /= np.linalg.norm(x0)
    y0 = np.cross(z, x0)
    best = None
    for roll in range(0, 360, 30):
        r = math.radians(roll)
        x = math.cos(r) * x0 + math.sin(r) * y0
        rot = np.column_stack([x, np.cross(z, x), z])
        q, pe, re = arm.solve(p, rot)
        cost = pe * 1e3 + math.degrees(re) + 0.2 * abs(q[5])
        if best is None or cost < best[0]:
            best = (cost, rot)
    return p, best[1]


def capture_grasp(c):
    """Freeze the trim -> TCP transform of each hand (measured once, with the trim held still)."""
    tp, tr = c.pose()
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (tr.T @ (pt - tp), tr.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def trim_target(xy, z, yaw_deg):
    return np.array([xy[0], xy[1], z]), L.rz(yaw_deg)


def track(c, target_fn, duration, label, gain=0.05, rot_gain=None, until=None, max_s=None, extra=None):
    """Closed loop on the trim: each step command both hands to the TCP poses that put the trim at target_fn(s)
    through the frozen grasp transforms, plus slow integral corrections of the measured trim position / orientation
    error. ``extra(c)`` may return an additional (dp, dyaw_rad) correction target (the slot alignment)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        if extra is not None:
            dp, dyaw = extra(c)
            pt = pt + dp
            rt = L.rz(math.degrees(dyaw)) @ rt
        pm, rm = c.pose()
        err = pt - pm
        c.corr_p = np.clip(c.corr_p + gain * err, -0.015, 0.015)
        e_r = _rotvec(rt @ rm.T)
        c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * e_r, -ROT_CORR_MAX, ROT_CORR_MAX)
        rc = _rotm(c.corr_r)
        for name, arm in c.arms.items():
            gp, gr = c.G[name]
            arm.label = label
            arm.set_pose(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            break


def tips(c, spring):
    """Mid-point of the two leg tips of ``spring`` (world)."""
    ps = c.poses()
    out = []
    for side in "pm":
        lp, lr = ps[f"leg{spring}{side}"]
        out.append(lp + lr @ L.leg_tip_local())
    return (out[0] + out[1]) / 2


def slot_errors(c):
    """Per spring: (radial, tangential) offset (m) of its squeezed leg tips' mid-point from its bracket's slot centre."""
    phi = c.phi()
    out = {}
    for s in ("A", "B"):
        beta = L.bracket_angles(phi)[s]
        rad, tan = L.bracket_frame(beta)
        d = tips(c, s) - (L.H + L.slot_center(beta))
        out[s] = (float(d @ rad), float(d @ tan))
    return out


def slot_correction(c):
    """Trim correction that moves both tip pairs onto their slot centres: mean offset (xy) and the turn about the
    housing axis."""
    phi = c.phi()
    ta, tb = tips(c, "A"), tips(c, "B")
    sa = L.H + L.slot_center(L.bracket_angles(phi)["A"])
    sb = L.H + L.slot_center(L.bracket_angles(phi)["B"])
    dp = ((sa - ta) + (sb - tb)) / 2
    dp[2] = 0.0
    va, vs = (ta - tb)[:2], (sa - sb)[:2]
    dyaw = math.atan2(va[0] * vs[1] - va[1] * vs[0], va @ vs)
    return dp, dyaw


# --------------------------------------------------------------------------------------------------- the script
def pinch(c, squeeze_grip=0.0):
    """Both hands: approach the springs from outside the trim, close across their legs (squeezing them shut;
    ``squeeze_grip`` > 0 leaves the fingers that far open, for the light-pinch negative)."""
    tp, tr = c.pose()
    for arm in c.arms.values():
        arm.set_comp(True)
    tg = {}
    for n, arm in c.arms.items():
        p, r = pinch_target(tp, tr, c.spring_of[n])
        arm.label = f"{n} plan"
        r = best_flip(arm, p + (p - tp) * 0, r)
        pb, _ = pinch_target(tp, tr, c.spring_of[n], back=APPROACH_BACK)
        tg[n] = (p, pb, r)
    c.note("pinch plan", yaw=trim_yaw(tr))
    yield from both(*(arm.move(tg[n][1] + [0, 0, APPROACH_UP], tg[n][2], 3.0, 1.0, f"{n} over") for n, arm in c.arms.items()))
    yield from both(*(arm.move(tg[n][1], tg[n][2], 1.5, None, f"{n} stand-off") for n, arm in c.arms.items()))
    yield from wait(0.3)
    tp, tr = c.pose()
    for n in c.arms:
        p, _ = pinch_target(tp, tr, c.spring_of[n])
        tg[n] = (p, tg[n][1], tg[n][2])
    yield from both(*(arm.move(tg[n][0], tg[n][2], 1.5, None, f"{n} in") for n, arm in c.arms.items()))
    yield from wait(0.3)
    yield from both(*(arm.set_grip(squeeze_grip, 1.2, f"{n} squeeze") for n, arm in c.arms.items()))
    yield from wait(0.8)
    if squeeze_grip == 0.0:
        yield from even_squeeze(c)
    c.note("pinched", theta=c.detail().get("measure", {}).get("leg_theta_deg"))


def leg_theta_now(c, spring):
    th = c.detail().get("measure", {}).get("leg_theta_deg") or {}
    return th.get(spring + "p", 99.0), th.get(spring + "m", 99.0)


def even_squeeze(c, tries=4, ok_deg=0.8, step=0.0015):
    """If a spring's legs are not both shut (the hand sits off-centre, so one finger presses its leg harder), shift
    that hand 1.5 mm toward the more open leg and look again (measured leg angles)."""
    for _ in range(tries):
        moves = {}
        tp, tr = c.pose()
        for n, arm in c.arms.items():
            s = c.spring_of[n]
            tp_, tm_ = leg_theta_now(c, s)
            if max(tp_, tm_) <= ok_deg:
                continue
            _, _, tan = L.spring_frame(s)
            d = (tr @ tan) * (step if tp_ > tm_ else -step)
            moves[n] = arm.pos + d
        if not moves:
            return
        c.note("re-centre", moves={n: (moves[n] - c.arms[n].pos).round(4).tolist() for n in moves},
               theta=c.detail().get("measure", {}).get("leg_theta_deg"))
        yield from both(*(c.arms[n].move(moves[n], c.arms[n].rot, 0.6, None, f"{n} re-centre") for n in moves))
        yield from wait(0.5)


def lift_and_carry(c, turn=True, goal_yaw=None):
    tp, tr = c.pose()
    yield from both(*(arm.move(arm.pos + [0, 0, 0.03], arm.rot, 1.5, None, f"{n} lift") for n, arm in c.arms.items()))
    yield from wait(0.5)
    capture_grasp(c)
    tp, tr = c.pose()
    y0 = trim_yaw(tr)
    phi = c.phi() if turn else y0  # the no-turn negative keeps the trim's start yaw
    if goal_yaw is not None:  # the clock-to-the-left-right-line negative
        phi = goal_yaw
    c.goal_yaw = phi
    p0 = tp.copy()
    z1 = p0[2] + CARRY_Z
    zh = L.H[2] + (L.TAB_Z - HOVER_BELOW_TAB - L.LEG_L) - L.COIL_Z  # trim origin with tips 20 mm below the tabs
    c.note("carry", yaw0=y0, phi=phi)
    yield from track(c, lambda s: trim_target(p0[:2], p0[2] + s * (z1 - p0[2]), y0), 2.5, "lift off")
    yield from track(c, lambda s: trim_target(p0[:2] + s * (L.H[:2] - p0[:2]), z1 + s * (zh - z1), y0 + s * (phi - y0)),
                     6.0, "carry + turn")
    c.zh = zh
    c.note("hover", slot=slot_errors(c))


def align_insert(c, correct=True):
    phi = c.goal_yaw
    zh = c.zh
    # a leg that opened during the carry (> 0.8 deg) would not pass: re-centre that hand on its spring first
    th = c.detail().get("measure", {}).get("leg_theta_deg") or {}
    if th and max(th.values()) > 0.8:
        yield from even_squeeze(c)
        capture_grasp(c)
    corr = {"dp": np.zeros(3), "dyaw": 0.0}

    def extra(c):
        dp, dyaw = slot_correction(c)
        corr["dp"] = np.clip(corr["dp"] + 0.15 * dp, -0.01, 0.01)
        corr["dyaw"] = float(np.clip(corr["dyaw"] + 0.15 * dyaw, -0.05, 0.05))
        return corr["dp"], corr["dyaw"]

    def lined_up(c):
        e = slot_errors(c)
        return all(abs(v[0]) < TIP_TOL_M and abs(v[1]) < TIP_TOL_M for v in e.values())

    ex = extra if correct else None
    yield from track(c, lambda s: trim_target(L.H[:2], zh, phi), 1.0, "line up", extra=ex, until=lined_up if correct
                     else None, max_s=10.0)
    c.note("lined up", slot=slot_errors(c), theta=c.detail().get("measure", {}).get("leg_theta_deg"))
    zi = zh + HOVER_BELOW_TAB + INSERT_ABOVE_TAB
    yield from track(c, lambda s: trim_target(L.H[:2], zh + s * (zi - zh), phi), 3.0, "insert", extra=ex)
    yield from wait(0.5)
    c.note("inserted", slot=slot_errors(c))


def release(c):
    for arm in c.arms.values():
        arm.set_comp("hold")
    for n in ("left", "right"):
        arm = c.arms[n]
        yield from arm.set_grip(grip_of(0.030), 1.0, f"{n} let go")
        yield from wait(1.0)
        c.note(f"released {c.spring_of[n]}")
    for n, arm in c.arms.items():
        arm.set_comp(True)
    tp, tr = c.pose()
    outs = {}
    for n, arm in c.arms.items():
        _, rad, _ = L.spring_frame(c.spring_of[n])
        _, _, tan = L.spring_frame(c.spring_of[n])
        tw = tr @ tan
        tw = -tw if tw[0] > 0 else tw  # tangential direction toward the robot
        outs[n] = (arm.pos + (tr @ rad) * RETRACT, arm.pos + (tr @ rad) * RETRACT + tw * SIDESTEP)
    # out 10 mm (the pad tips clear the legs), then sideways 50 mm toward the robot (past the flange rim), then down
    yield from both(*(arm.move(outs[n][0], arm.rot, 1.0, None, f"{n} out") for n, arm in c.arms.items()))
    # fingers in to 12 mm a side (the spread legs are within 5 mm of the spring's centre at this height), so the
    # trailing finger also clears the flange rim
    yield from both(*(arm.set_grip(grip_of(0.012), 0.6, f"{n} fingers in") for n, arm in c.arms.items()))
    yield from both(*(arm.move(outs[n][1], arm.rot, 2.0, None, f"{n} aside") for n, arm in c.arms.items()))
    yield from both(*(arm.move(arm.pos - [0, 0, 0.06], arm.rot, 1.5, None, f"{n} below flange")
                      for n, arm in c.arms.items()))
    c.note("hands out")


PUSH_BETAS = (110.0, 105.0, 115.0, 120.0, 125.0, 100.0, 130.0)


def choose_push(c, tp, tr, dz_low=-0.080):
    """Per hand, the first push angle (PUSH_BETAS) whose fingertip pose is reachable from 80 mm below the flush height
    to 3 mm above it."""
    out = {}
    for n, arm in c.arms.items():
        for b in PUSH_BETAS:
            ok = True
            for dz in (dz_low, dz_low / 2, 0.003):
                tip, appr, rot = push_point(tp + np.array([0.0, 0.0, dz]), tr, n, b)
                tcp = tip - appr * 0.013 - np.array([0, 0, 0.016])
                q, pe, re = arm.solve(tcp, best_flip(arm, tcp, rot))
                if pe > 5e-4 or re > 0.01:
                    ok = False
                    break
            if ok:
                out[n] = b
                break
        else:
            raise Stop(f"{n}: no reachable push point under the flange")
    return out


def push_point(tp, tr, n, beta=PUSH_BETA):
    """Fingertip contact point under the flange and the approach (world) for hand n."""
    # left hand: spring A side (trim +90) turned toward the robot by (PUSH_BETA - 90); right: spring B side likewise
    a = math.radians(beta) if n == "left" else math.radians(-beta)
    rad_t = np.array([math.cos(a), math.sin(a), 0.0])
    rad = tr @ rad_t
    rad[2] = 0.0
    rad /= np.linalg.norm(rad)
    e = math.radians(PUSH_ELEV)
    appr = np.array([-rad[0] * math.cos(e), -rad[1] * math.cos(e), math.sin(e)])
    tip = tp + tr @ (rad_t * PUSH_R) + np.array([0.0, 0.0, -0.001])
    x = np.cross(np.array([0.0, 0.0, 1.0]), rad)
    x = x - appr * (x @ appr)
    x /= np.linalg.norm(x)
    rot = np.column_stack([x, np.cross(appr, x), appr])
    return tip, appr, rot


def push_flush(c):
    tp, tr = c.pose()
    flush = np.array([tp[0], tp[1], L.H[2] - L.FLANGE_T])
    betas = choose_push(c, flush, tr, dz_low=min(-0.01, tp[2] - flush[2] - 0.016))
    c.note("push plan", betas=betas)
    tg = {}
    for n, arm in c.arms.items():
        tip, appr, rot = push_point(tp, tr, n, betas[n])
        # the closed fingers' upper edge (13.7 mm from the tip centre, outward and up) meets the flange first: start
        # with the tip centre 16 mm under the flange's bottom face (that edge about 4 mm under it)
        tcp = tip - appr * 0.013 - np.array([0, 0, 0.016])
        rot = best_flip(arm, tcp, rot)
        tg[n] = (tcp, rot, appr)
    yield from both(*(arm.move_joint(tg[n][0] - [0, 0, 0.012], tg[n][1], 3.0, 0.0, f"{n} under flange")
                      for n, arm in c.arms.items()))
    yield from both(*(arm.move(tg[n][0], tg[n][1], 1.5, None, f"{n} to flange") for n, arm in c.arms.items()))
    c.note("under flange")
    # push up until the measured flange gap closes (closed loop on the trim pose), then 2 mm more
    for k in range(int(8.0 * CONTROL_HZ)):
        gaps = c.detail().get("measure", {})
        mg = gaps.get("max_gap_mm", 99.0)
        if mg is not None and mg < 0.8:
            break
        for n, arm in c.arms.items():
            arm.label = f"{n} push"
            arm.set_pose(arm.pos + np.array([0, 0, 0.0015]), arm.rot)
        yield
    c.note("flush", gaps=c.detail().get("measure"))
    yield from both(*(arm.move(arm.pos + [0, 0, 0.002], arm.rot, 0.5, None, f"{n} press") for n, arm in c.arms.items()))
    yield from wait(0.6)
    # the right hand comes away first and turns its wrist camera up at the trim (the end state seen from below) while
    # the left hand keeps the flange pressed; then the left hand comes away
    r = c.right
    yield from r.move(r.pos - [0, 0, 0.04], r.rot, 1.2, None, "right down")
    lp, lr = look_pose(r, L.H + np.array([0.0, 0.0, 0.0]))
    yield from r.move_joint(lp, lr, 3.0, 0.0, "right look")
    yield from c.left.move(c.left.pos - [0, 0, 0.04], c.left.rot, 1.5, None, "left down")
    c.note("hands off")
    yield from c.left.home(3.0)


def demo(c):
    yield from wait(0.5)
    yield from pinch(c)
    yield from lift_and_carry(c)
    yield from align_insert(c)
    yield from release(c)
    yield from push_flush(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def hold(c):
    """Both arms hold READY for the whole run limit (2000 steps): the full-length do-nothing check."""
    yield from wait(100.0)


def look(c):
    """Setup view (no task action): the right hand turns its wrist camera up into the empty housing at the brackets,
    as an electrician looks up before fitting the trim."""
    yield from wait(0.5)
    lp, lr = look_pose(c.right, L.H + np.array([0.0, 0.0, 0.03]))
    yield from c.right.move_joint(lp, lr, 4.0, 1.0, "right look")
    yield from wait(2.0)


def neg_clock_zero(c):
    """Heuristic check: turns the trim to the robot's left-right line (the brackets' nominal angle) instead of to the
    brackets, lines its centre up with the housing and raises it as in the demo (expected 0: the brackets are always
    3-5 deg off that line)."""
    yield from wait(0.5)
    yield from pinch(c)
    yield from lift_and_carry(c, goal_yaw=0.0)
    yield from align_insert(c, correct=False)
    yield from release(c)
    yield from push_flush(c)
    yield from wait(4.0)


def neg_no_turn(c):
    """Carries the trim up without turning it to the brackets (keeps its start yaw, at least 4 deg off), lines the
    trim's centre up with the housing and raises it as in the demo: the squeezed legs meet the tabs beside the slots
    (expected 0)."""
    yield from wait(0.5)
    yield from pinch(c)
    yield from lift_and_carry(c, turn=False)
    yield from align_insert(c, correct=False)
    yield from release(c)
    yield from push_flush(c)
    yield from wait(4.0)


def neg_light_pinch(c):
    """Pinches each spring with the fingers left 6 mm apart (legs only partly squeezed, about 10 deg open), then does
    everything else as the demo: the leg hooks cannot pass the slots (expected 0)."""
    yield from wait(0.5)
    yield from pinch(c, squeeze_grip=grip_of(0.006))
    yield from lift_and_carry(c)
    yield from align_insert(c)
    yield from release(c)
    yield from push_flush(c)
    yield from wait(4.0)


def neg_no_push(c):
    """Both springs hooked into their brackets as in the demo, hands taken away, flange never pushed up (expected
    0.5: both captured, not flush)."""
    yield from wait(0.5)
    yield from pinch(c)
    yield from lift_and_carry(c)
    yield from align_insert(c)
    yield from release(c)
    yield from both(c.left.home(3.0), c.right.home(3.0))
    yield from wait(4.0)


def carry_only(c):
    yield from wait(0.5)
    yield from pinch(c)
    yield from lift_and_carry(c)
    yield from align_insert(c)
    yield from wait(2.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in TRIM_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["TRIM_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "hold": hold,
    "carry-only": carry_only,
    "replay": replay,
    "look": look,
    "neg-clock-zero": neg_clock_zero,
    "neg-no-turn": neg_no_turn,
    "neg-light-pinch": neg_light_pinch,
    "neg-no-push": neg_no_push,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(
        str(path),
        fps=CONTROL_HZ,
        codec="libx264",
        quality=8,
        macro_block_size=8,
        ffmpeg_params=["-pix_fmt", "yuv420p"],
    )
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(
    task_path,
    out_dir,
    max_steps=3000,
    record=True,
    stage="demo",
    cam_width=320,
    cam_height=176,
    device="cpu",
    seed=None,
):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(
        task=str(Path(task_path).resolve()),
        object_jitter=0,
        control_hz=CONTROL_HZ,
        cam_width=cam_width,
        cam_height=cam_height,
        device=device,
    )
    stages = stage.split(",")
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=seed)
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import trimkit

    rt = trimkit.runtime()
    poses = rt.read()
    return dict(
        score=trimkit.STATE["score"],
        setup=rt.setup,
        phi=rt.phi,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import trimkit

    rows = []
    for i, r in enumerate(resets):
        want = trimkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("trim", "housing", "carton")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["trim"], want["trim"][1])).max())
        rows.append(
            dict(
                trial=i,
                stage=stages[i],
                setup=r["setup"],
                score_after_reset=r["score"],
                max_body_dev_from_authored_mm=round(dev, 4),
                max_rot_dev=round(rdev, 6),
                trial_score=results[i]["task_score"],
                trial_stop=results[i]["stop"],
            )
        )
    ok = all(
        x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 0.5
        for x in rows
    )
    return dict(
        summary=dict(
            passed=ok,
            trials=len(rows),
            max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows),
        ),
        rows=rows,
        reset_states=resets,
    )


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import trimkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    c = Ctx(emb)
    gen = STAGES[stage](c)
    frames = {}

    def snap(o):
        if record:
            for cam, img in o.images.items():
                frames.setdefault(cam, []).append(np.asarray(img))

    snap(obs)
    steps, ended, info, stop, success, res = 0, None, {}, None, False, None
    tracef = open(out / "trace.jsonl", "w")
    while steps < max_steps:
        if ended is None:
            try:
                next(gen)
            except StopIteration:
                ended = steps
            except Stop as e:
                stop, ended = str(e), steps
                print("STOP", stop, flush=True)
        res = emb.step(c.action())
        jp = np.asarray(res.observation.state[STATE_KEY], float).ravel()
        c.left.observe(jp[:7])
        c.right.observe(jp[7:14])
        steps += 1
        c.step = steps
        info = res.info
        snap(res.observation)
        d = trimkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=trimkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=trimkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
            trim_mm=np.round(c.pose()[0] * 1000, 2).tolist(),
            trim_rv=np.round(_rotvec(c.pose()[1]), 5).tolist(),
            tcp_mm={
                n: np.round(a.measured_tcp()[0] * 1000, 2).tolist()
                for n, a in c.arms.items()
            },
            tcp_cmd_mm={
                n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()
            },
            offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()},
        )
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get("measure", {})
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} cap {m.get('captured')} "
                f"gap {m.get('max_gap_mm')} tilt {m.get('tilt_deg')} z {m.get('trim_z_mm')} touch {m.get('touched')} "
                f"theta {m.get('leg_theta_deg')} robot_n {d.get('robot_n')} wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(
        stage=stage,
        task_score=score,
        success=success,
        steps=steps,
        stop=stop,
        termination=getattr(res, "termination_reason", None),
        setup=reset_state["setup"],
        scorer=dict(score=trimkit.STATE["score"], level=trimkit.STATE["level"]),
        final=trimkit.STATE["detail"],
        nonfinite=trimkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=trimkit.PHYSICS_DT,
        control_hz=CONTROL_HZ,
    )
    tracef.close()
    (out / "result.json").write_text(json.dumps(result, indent=2, default=float))
    if record and frames:
        import imageio

        for cam, fs in frames.items():
            _write_mp4(fs, out / f"demo_{cam}.mp4")
        top = next((k for k in frames if "top" in k), next(iter(frames), None))
        if top:
            imageio.imwrite(out / "setup.png", frames[top][0])
            imageio.imwrite(out / "final.png", frames[top][-1])
        for cam, fs in frames.items():
            imageio.imwrite(out / f"final_{cam}.png", fs[-1])
    print(
        "RESULT",
        json.dumps(
            {
                k: result[k]
                for k in ("stage", "task_score", "success", "steps", "stop", "setup")
            }
        ),
        flush=True,
    )
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=3000)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument(
        "--cam-size",
        default="320x176",
        help="camera WxH (evidence runs use the default)",
    )
    ap.add_argument(
        "--device",
        default="cpu",
        help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way",
    )
    ap.add_argument(
        "--seed",
        type=int,
        default=None,
        help="reset seed (seeded start state, as in inspect-robots runs)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    import os
    import traceback

    try:
        run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
        code = 0
    except Exception:  # noqa: BLE001  (Kit does not exit on an exception: report it and leave)
        traceback.print_exc()
        code = 1
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)
