"""Scripted full-score demo (and negative variants) for the jedec-flip task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed N]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the tray poses through
``flipkit.bench_state()`` (read-only, like an operator watching the trays' edges and chamfers); it never writes a pose,
a joint state or a score. Its joint commands are rate-limited below the CLI's per-step delta guard and clipped to the
URDF joint bounds.

Plan:
1. Both hands top-down on the empty tray's two end walls (inner finger in the end handling slot); lift it.
2. Carry it over the loaded tray and register it from the measured poses (edges flush, chamfers together), lower it
   until its walls sit over the loaded tray's stacking lip; let go.
3. Both hands come in along the stack's long axis and squeeze the two trays together at each end (fingers closing
   vertically, about 10 N per finger).
4. Lift 85 mm, roll the stack 180 deg about its long axis, set it back on the riser; let go, draw the hands out.
5. Top-down on the old tray's end walls (now on top, upside down); lift it off and lay it on the table aside.
6. Both arms back to READY.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
"""

from __future__ import annotations

import argparse
import json
import math
import os
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
ROT_CORR_MAX = 0.10


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
        tray before the fingers open."""
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
        sg = 1.0 if self.name == "left" else -1.0
        for s in (
            np.array(READY, float),
            np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0]),
            np.array([0.0, 2.3, 0.75, 1.5, 1.57 * sg, 1.6 * sg]),
            np.array([0.3 * sg, 2.3, 1.0, 1.0, 1.57 * sg, 1.6 * sg]),
        ):
            q2, pe2, re2 = fastik.solve(pos, rot, s, base=self.base, iters=200)
            cost2 = pe2 + 0.1 * re2 + 0.02 * np.abs(q2 - self.q).sum()
            if cost2 < best[1] + 0.1 * best[2] + 0.02 * np.abs(best[0] - self.q).sum():
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

    def move_j(self, pos, rot, duration, grip=None, label="", seed=None):
        """Joint-space move to the IK solution of (pos, rot) reached from ``seed`` (a joint posture of the wanted arm
        configuration family; for changes of arm posture)."""
        from yam_arena import ik

        self.label = label or self.label
        q_now = self.q.copy()
        q1, pe, re = fastik.solve(np.asarray(pos, float), np.asarray(rot, float),
                                  np.asarray(seed if seed is not None else q_now, float), base=self.base, iters=400)
        if pe > 0.004 or re > math.radians(4):
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        g0 = self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q_now + s * (q1 - q_now)
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


def both(a, b):
    """Run two arm generators in parallel (one control step each per yield)."""
    gens = [a, b]
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
        self.log = []
        self.step = 0
        self.G = {}
        self.corr_p, self.corr_r = np.zeros(3), np.zeros(3)

    def pose(self, name="A"):
        import flipkit

        return flipkit.bench_state()[name]

    def detail(self):
        import flipkit

        return flipkit.STATE["detail"]

    def action(self):
        from inspect_robots.types import Action
        from inspect_robots_yam.packing import pack

        if (
            getattr(self, "replay", None) is not None
        ):  # open-loop replay of a recorded command trace
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
        row = dict(
            at=tag, step=self.step, measure=d.get("measure"), level=d.get("level"), **kw
        )
        self.log.append(row)
        print("LOOK", json.dumps(row, default=float), flush=True)


def down_rot(radial_xy, arm):
    """Straight-down TCP, fingers closing along the given horizontal (radial) direction; sign = least joint-6 travel."""
    best = None
    for sgn in (1.0, -1.0):
        x = sgn * np.array([radial_xy[0], radial_xy[1], 0.0])
        x /= np.linalg.norm(x)
        z = np.array([0.0, 0.0, -1.0])
        r = np.column_stack([x, np.cross(z, x), z])
        q, pe, re = arm.solve(arm.pos, r)
        cost = abs(q[5]) / 2.094 + pe * 10
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def capture_grasp(c, body="A"):
    """Freeze the body -> TCP transform of each hand (measured once, with the body held still)."""
    pr, rr = c.pose(body)
    for n, arm in c.arms.items():
        pt, rt = arm.measured_tcp()
        c.G[n] = (rr.T @ (pt - pr), rr.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)
    c.body = body


def track(c, target_fn, duration, label, gain=0.05, rot_gain=None, free_z=False, until=None, max_s=None,
          deadband=0.0, correct=True):
    """Closed loop on the held body (c.body): each step command both hands to the TCP poses that put the body at
    target_fn(s) through the frozen grasp transforms, plus slow integral corrections of the measured body position /
    orientation error (arm sag, grasp slip). ``until(c)`` ends the move early (checked after the nominal duration, up to
    ``max_s``)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        if correct:
            pm, rm = c.pose(c.body)
            err = pt - pm
            if free_z:
                err[2] = 0.0
            if deadband > 0:
                err = np.where(np.abs(err) < deadband, 0.0, err)
            c.corr_p = np.clip(c.corr_p + gain * err, -0.010, 0.010)
            e_r = _rotvec(rt @ rm.T)
            if deadband > 0:
                e_r = np.where(np.abs(e_r) < deadband / 0.16, 0.0, e_r)
            c.corr_r = np.clip(c.corr_r + (gain if rot_gain is None else rot_gain) * e_r, -ROT_CORR_MAX, ROT_CORR_MAX)
        rc = _rotm(c.corr_r)
        for name, arm in c.arms.items():
            gp, gr = c.G[name]
            arm.label = label
            arm.set_pose(pt + c.corr_p + rc @ rt @ gp, rc @ rt @ gr)
        yield
        if until is not None and k >= n and until(c):
            break


def pose_path(p0, r0, p1, r1):
    from yam_arena import ik

    return lambda s: (p0 + s * (p1 - p0), ik.slerp_matrix(r0, r1, s))


# ------------------------------------------------------------------------------------------- task geometry
WALL_X = L.TRAY_L / 2 - (L.WALL_OUT_T + L.WALL_IN_T) / 2  # end-wall centre line (tray x)
WALL_HALF = (L.WALL_OUT_T + L.WALL_IN_T) / 2
WALL_OPEN = WALL_HALF + 0.0045  # pads 4.5 mm off the wall faces before closing (inner finger in the slot)
WALL_SQUEEZE = 0.0025  # 2.5 mm inside the wall faces: about 12.5 N per finger
PINCH_Y = 0.135  # stack pinch: TCP 135 mm either side of the arms' midline (pads 39 mm over each tray end)
PINCH_Z = (L.UNDER_RIB_Z[0] + L.STACK_DZ + L.TOP_RIB_Z[1]) / 2  # middle of the squeezed height (A frame)
PINCH_HALF = (L.STACK_DZ + L.TOP_RIB_Z[1] - L.UNDER_RIB_Z[0]) / 2  # 7.41 mm
PINCH_OPEN = PINCH_HALF + 0.010
PINCH_SIDE = 0.095  # start of the sideways approach: fingers clear of the tray (x)
PINCH_SQUEEZE = float(os.environ.get("JFLIP_PINCH_SQUEEZE", "0.002"))  # 2 mm: about 10 N per finger
STACK_TOP = L.STACK_DZ + L.TRAY_H + L.LIP_H  # B's lip top above A's frame origin (16.64 mm)
LIFT = 0.085  # stack raised 85 mm before the roll (the 136 mm wide stack sweeps a 69 mm radius)
ROLL_S = 7.0
HOVER_B = 0.020
NEST_ALIGN_TOL = 0.0003
NEST_YAW_TOL = 0.15
ASIDE = np.array([0.165, 0.0])
SIDES = (("left", 1.0), ("right", -1.0))  # left arm takes the tray's +x end (world +y), right the -x end


def chain_rot(close, app):
    x = np.asarray(close, float) / np.linalg.norm(close)
    z = np.asarray(app, float) / np.linalg.norm(app)
    return np.column_stack([x, np.cross(z, x), z])


def down_rot_at(arm, pos, along):
    """Straight-down TCP at ``pos`` closing along the horizontal direction ``along``; the finger sign the arm reaches
    best (least joint-6 travel)."""
    best = None
    for sgn in (1.0, -1.0):
        x = sgn * np.array([along[0], along[1], 0.0])
        r = chain_rot(x, (0.0, 0.0, -1.0))
        q, pe, re = arm.solve(pos, r)
        cost = abs(q[5]) / 2.094 + pe * 100 + re
        if best is None or cost < best[0]:
            best = (cost, r)
    return best[1]


def pinch_seed(sg):
    return [0.0, 2.3, 0.75, 1.5, 1.57 * sg, 1.6 * sg]


def pinch_rot(sg):
    """Horizontal approach along the stack's long axis (world -y for the left arm), fingers closing vertically
    (left: closing axis down, right: up, so a +180 deg roll about world y stays inside both wrists' joint-6 range)."""
    return chain_rot((0.0, 0.0, -sg), (0.0, -sg, 0.0))


def wall_tcp(pt, rt, s, tip_z):
    """TCP for a top-down grip on the tray's end wall at end s (+1 / -1), pad tips at tray-frame height tip_z."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    return pt + rt @ np.array([s * WALL_X, 0.0, tip_z]) + np.array([0.0, 0.0, PAD_TIP_PAST_TCP])


def grip_wall(c, body, tip_z, label):
    """Both hands top-down on the two end walls of tray ``body`` (inner finger in the end handling slot), closed."""
    pt, rt = c.pose(body)
    tg = {}
    for n, s in SIDES:
        p = wall_tcp(pt, rt, s, tip_z)
        tg[n] = (p, down_rot_at(c.arms[n], p + [0, 0, 0.05], rt[:, 0]))
    for arm in c.arms.values():
        arm.set_comp(True)
    yield from both(*(c.arms[n].move(tg[n][0] + [0, 0, 0.05], tg[n][1], 3.0, grip_of(WALL_OPEN), f"{n} over {label}")
                      for n in c.arms))
    yield from wait(0.4)
    pt, rt = c.pose(body)
    for n, s in SIDES:
        tg[n] = (wall_tcp(pt, rt, s, tip_z), tg[n][1])
    yield from both(*(c.arms[n].move(tg[n][0], tg[n][1], 2.0, None, f"{n} down to {label}") for n in c.arms))
    yield from wait(0.3)
    yield from both(*(c.arms[n].set_grip(grip_of(WALL_HALF - WALL_SQUEEZE), 0.8, f"{n} grip {label}") for n in c.arms))
    yield from wait(0.5)
    c.note(f"gripped {label}")


def pick_b(c):
    yield from grip_wall(c, "B", 0.0015, "empty tray")
    yield from both(*(arm.move(arm.pos + [0, 0, 0.060], arm.rot, 2.0, None, f"{n} lift B") for n, arm in c.arms.items()))
    yield from wait(0.5)
    capture_grasp(c, "B")
    c.note("lifted B")


def b_target(c, dz=0.0):
    """Stacked pose of B on A (measured A pose); for the blind-cover negative, on A's nominal (home) pose."""
    pa, ra = L.tray_pose(L.A_HOME, (0.0, 0.0, 0.0)) if getattr(c, "blind", False) else c.pose("A")
    return pa + ra @ np.array([0.0, 0.0, L.STACK_DZ + dz]), ra.copy()


def nest_err(c):
    pa, ra = c.pose("A")
    pb, rb = c.pose("B")
    return L.nest_error(pa, ra, pb, rb)


def place_b(c):
    """Carry B over A, register it by eye (edges flush, chamfers together) from the measured poses, lower it onto A's
    stacking lip, let go."""
    pb, rb = c.pose("B")
    p1, r1 = b_target(c, HOVER_B)
    mid = np.array([0.5 * (pb[0] + p1[0]), 0.5 * (pb[1] + p1[1]), max(pb[2], p1[2]) + 0.015])
    yield from track(c, pose_path(pb, rb, mid, r1), 2.5, "B up and over")
    yield from track(c, lambda s: (mid + s * (b_target(c, HOVER_B)[0] - mid), b_target(c)[1]), 2.5, "B over A")

    def aligned(c):
        e = nest_err(c)
        return abs(e[0]) < NEST_ALIGN_TOL and abs(e[1]) < NEST_ALIGN_TOL and abs(e[3]) < NEST_YAW_TOL

    if getattr(c, "blind", False):
        yield from track(c, lambda s: b_target(c, HOVER_B), 1.0, "hold B over nominal", gain=0.08)
    else:
        yield from track(c, lambda s: b_target(c, HOVER_B), 1.0, "align B", gain=0.08, until=aligned, max_s=6.0)
    e = nest_err(c)
    c.note("B aligned", err=[round(v * 1e3, 3) for v in e[:3]], yaw=round(e[3], 3))
    for arm in c.arms.values():
        arm.set_comp("hold")
    seat = [0]

    def seated(c):
        e = nest_err(c)
        seat[0] = seat[0] + 1 if abs(e[2]) < 0.0004 else 0
        return seat[0] >= 4

    dur = (HOVER_B + 0.0015) / 0.008
    yield from track(c, lambda s: b_target(c, HOVER_B - s * (HOVER_B + 0.0015)), dur, "lower B onto A", gain=0.04,
                     free_z=True, until=seated, max_s=dur + 2.0)
    yield from wait(0.3)
    e = nest_err(c)
    c.note("B on A", err=[round(v * 1e3, 3) for v in e[:3]], yaw=round(e[3], 3))
    yield from both(*(arm.set_grip(grip_of(WALL_OPEN), 0.8, f"{n} open") for n, arm in c.arms.items()))
    yield from both(*(arm.move(arm.pos + [0, 0, 0.045], arm.rot, 1.5, None, f"{n} up") for n, arm in c.arms.items()))
    for arm in c.arms.values():
        arm.set_comp(True)


def pinch_tcp(pa, ra, sg, z=PINCH_Z):
    t = (sg * PINCH_Y - pa[1] - ra[1, 2] * z) / ra[1, 0]
    return pa + ra @ np.array([t, 0.0, z])


def pinch_stack(c, z=PINCH_Z, half=PINCH_HALF):
    """Both hands come in along the stack's long axis and squeeze the two trays together at each end."""
    for arm in c.arms.values():
        arm.set_comp(True)
    yield from go_home(c, 2.0)  # change of arm posture from READY: a joint-space move from the top-down grip posture
    pa, ra = c.pose("A")  # would sweep the fingers through the stack
    tg = {n: (pinch_tcp(pa, ra, sg, z), pinch_rot(sg)) for n, sg in SIDES}
    side = np.array([-PINCH_SIDE, 0.0, 0.0])  # fingers slide in from the robot's side, pads above and below the end
    yield from both(*(c.arms[n].move_j(tg[n][0] + side + [0, 0, 0.040], tg[n][1], 3.5, grip_of(PINCH_OPEN),
                                       f"{n} beside stack end", seed=pinch_seed(sg)) for n, sg in SIDES))
    yield from both(*(c.arms[n].move(tg[n][0] + side, tg[n][1], 1.2, None, f"{n} level") for n, sg in SIDES))
    yield from wait(0.3)
    pa, ra = c.pose("A")
    tg = {n: (pinch_tcp(pa, ra, sg, z), pinch_rot(sg)) for n, sg in SIDES}
    yield from both(*(c.arms[n].move(tg[n][0], tg[n][1], 2.5, None, f"{n} onto the end") for n, sg in SIDES))
    yield from wait(0.3)
    yield from both(*(c.arms[n].set_grip(grip_of(half - PINCH_SQUEEZE), 1.0, f"{n} squeeze") for n in c.arms))
    yield from wait(0.6)
    capture_grasp(c, "A")
    c.note("pinched")


def roll_stack(c, angle=math.pi):
    """Lift the squeezed stack and roll it 180 deg about world y through both TCPs. The end grips sit at the edge of
    the YAM's wrist range (joint 5 at +-90 deg), so the hands follow the exact rolled grip orientation (no orientation
    correction from the measured stack pose; the stack is held rigidly between the four pads)."""
    from yam_arena import ik

    up = np.array([0.0, 0.0, LIFT])
    r0 = {n: pinch_rot(sg) for n, sg in SIDES}
    for arm in c.arms.values():
        arm.set_comp(True)
    yield from both(*(c.arms[n].move(c.arms[n].pos + up, r0[n], 3.0, None, "lift stack") for n in c.arms))
    p0 = {n: c.arms[n].pos.copy() for n in c.arms}
    ax = 0.5 * (p0["left"] + p0["right"])
    nst = round(ROLL_S * CONTROL_HZ)
    for k in range(1, nst + 1):
        rr = L.ry(ik.smoothstep(k / nst) * angle)
        for n, arm in c.arms.items():
            arm.label = "roll stack"
            arm.set_pose(ax + rr @ (p0[n] - ax), rr @ r0[n])
        yield
    yield from wait(0.5)
    c.note("rolled")


def set_down(c):
    """Lower the turned stack onto the riser (B's lip lands on it), let go and draw both hands out sideways."""
    pa, ra = c.pose("A")
    z_rest = L.RISER_TOP + STACK_TOP
    drop = float(pa[2] - z_rest)
    yield from both(*(arm.move(arm.pos - [0, 0, drop - 0.004], arm.rot, 3.5, None, "stack down")
                      for arm in c.arms.values()))
    for arm in c.arms.values():
        arm.set_comp("hold")
    landed = [0]

    def on_riser(c):
        landed[0] = landed[0] + 1 if c.pose("A")[0][2] - z_rest < 0.0005 else 0
        return landed[0] >= 4

    # lower at 4 mm/s until the stack rests on the riser (measured), then 0.5 s more (light press, arm compliance)
    p_now = {n: arm.pos.copy() for n, arm in c.arms.items()}
    for k in range(1, round(4.0 * CONTROL_HZ) + 1):
        for n, arm in c.arms.items():
            arm.label = "stack onto riser"
            arm.set_pose(p_now[n] - [0, 0, min(0.012, 0.004 * k / CONTROL_HZ)], arm.rot)
        yield
        if on_riser(c):
            break
    yield from wait(0.5)
    c.note("stack down", rest_mm=round((c.pose("A")[0][2] - z_rest) * 1e3, 3))
    yield from both(*(arm.set_grip(grip_of(PINCH_OPEN), 0.8, f"{n} open") for n, arm in c.arms.items()))
    yield from both(*(c.arms[n].move(c.arms[n].pos + [-PINCH_SIDE, 0.0, 0.0], c.arms[n].rot, 2.0, None, f"{n} out")
                      for n, sg in SIDES))
    for arm in c.arms.values():
        arm.set_comp(True)
    yield from both(*(arm.move(arm.pos + [0, 0, 0.05], arm.rot, 1.2, None, f"{n} up") for n, arm in c.arms.items()))


def lift_off_a(c):
    """Grip the old tray (now on top, upside down) by its end walls, lift it off and lay it on the table aside."""
    yield from grip_wall(c, "A", L.STACK_DZ - 0.002, "old tray")
    yield from both(*(arm.move(arm.pos + [0, 0, 0.030], arm.rot, 1.5, None, f"{n} lift A") for n, arm in c.arms.items()))
    yield from wait(0.3)
    capture_grasp(c, "A")
    pa, ra = c.pose("A")
    z_rest = L.TOP + (L.TRAY_H + L.LIP_H)  # upside down, it lands on its lip
    p_hover = np.array([ASIDE[0], pa[1], z_rest + 0.025])
    mid = np.array([0.5 * (pa[0] + ASIDE[0]), pa[1], max(pa[2], p_hover[2]) + 0.01])
    yield from track(c, pose_path(pa, ra, mid, ra), 2.0, "A up and back", gain=0.03)
    yield from track(c, pose_path(mid, ra, p_hover, ra), 2.0, "A over the table", gain=0.03)
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from track(c, lambda s: (p_hover - [0, 0, s * 0.0265], ra), 3.0, "A down", correct=False)
    yield from wait(0.3)
    c.note("A down")
    yield from both(*(arm.set_grip(grip_of(WALL_OPEN), 0.8, f"{n} open") for n, arm in c.arms.items()))
    yield from both(*(arm.move(arm.pos + [0, 0, 0.05], arm.rot, 1.5, None, f"{n} up") for n, arm in c.arms.items()))
    for arm in c.arms.values():
        arm.set_comp(True)


def go_home(c, duration=2.5):
    yield from both(c.left.home(duration), c.right.home(duration))


def demo(c):
    yield from wait(0.3)
    yield from pick_b(c)
    yield from place_b(c)
    yield from pinch_stack(c)
    yield from roll_stack(c)
    yield from set_down(c)
    yield from lift_off_a(c)
    yield from go_home(c)
    yield from wait(3.0)


def idle(c):
    yield from wait(4.0)


def snap(c):
    """A few steps only (high-resolution setup image)."""
    yield from wait(0.25)


def probe_roll(c):
    """M1 probe (needs JFLIP_DEV_PRESTACK=1: B starts nested on A): pinch, roll, set down, lift off."""
    yield from wait(0.3)
    yield from pinch_stack(c)
    yield from roll_stack(c)
    yield from set_down(c)
    yield from lift_off_a(c)
    yield from go_home(c)
    yield from wait(4.0)


def probe_place(c):
    yield from wait(0.3)
    yield from pick_b(c)
    yield from place_b(c)
    yield from go_home(c)
    yield from wait(2.0)


def neg_one_hand(c):
    """Development variant: the stack is squeezed and rolled by the left hand alone (right hand stays away). In the
    sim the rigid stack stayed closed in one hand (dev run, 23-28 N per finger); not used as a negative."""
    yield from wait(0.3)
    yield from pick_b(c)
    yield from place_b(c)
    pa, ra = c.pose("A")
    n, sg = "left", 1.0
    arm = c.left
    p, r = pinch_tcp(pa, ra, sg), pinch_rot(sg)
    side = np.array([-PINCH_SIDE, 0.0, 0.0])
    yield from go_home(c, 2.0)
    pa, ra = c.pose("A")
    p = pinch_tcp(pa, ra, sg)
    yield from arm.move_j(p + side + [0, 0, 0.040], r, 3.5, grip_of(PINCH_OPEN), "left beside stack end",
                          seed=pinch_seed(1.0))
    yield from arm.move(p + side, r, 1.2, None, "left level")
    yield from arm.move(p, r, 2.5, None, "left onto the end")
    yield from arm.set_grip(grip_of(PINCH_HALF - PINCH_SQUEEZE), 1.0, "left squeeze")
    yield from wait(0.5)
    pa, ra = c.pose("A")
    pt, rt = arm.measured_tcp()
    g = (ra.T @ (pt - pa), ra.T @ rt)
    ax = arm.pos.copy()
    from yam_arena import ik

    for k in range(1, 61):
        s = ik.smoothstep(k / 60)
        arm.set_pose(ax + [0, 0, s * LIFT], r)
        yield
    ax = arm.pos.copy()
    for k in range(1, round(ROLL_S * CONTROL_HZ) + 1):
        th = ik.smoothstep(k / (ROLL_S * CONTROL_HZ)) * math.pi
        arm.set_pose(ax, L.ry(th) @ r)
        yield
    c.note("one-hand rolled", grasp=np.round(g[0] * 1e3, 1).tolist())
    yield from wait(3.0)


def neg_blind_cover(c):
    """Core negative: the empty tray is carried and lowered onto where the loaded tray sits in the nominal layout
    (never looking at the loaded tray), then the rest of the procedure runs as in the demo."""
    yield from wait(0.3)
    yield from pick_b(c)
    c.blind = True
    yield from place_b(c)
    e = nest_err(c)
    c.note("B blind", err=[round(v * 1e3, 3) for v in e[:3]], yaw=round(e[3], 3))
    c.blind = False
    yield from pinch_stack(c)
    yield from roll_stack(c)
    yield from set_down(c)
    yield from lift_off_a(c)
    yield from go_home(c)
    yield from wait(3.0)


def neg_no_cover(c):
    """Core negative: the loaded tray is gripped at its ends and rolled over alone, without the empty tray stacked on
    it (no sandwich): the packages are not captive."""
    yield from wait(0.3)
    za = (L.UNDER_RIB_Z[0] + L.TOP_RIB_Z[1]) / 2
    yield from pinch_stack(c, z=za, half=(L.TOP_RIB_Z[1] - L.UNDER_RIB_Z[0]) / 2)
    yield from roll_stack(c)
    c.note("rolled without cover")
    yield from wait(3.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in JFLIP_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    rows = [json.loads(line) for line in open(os.environ["JFLIP_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "snap": snap,
    "replay": replay,
    "probe-roll": probe_roll,
    "probe-place": probe_place,
    "neg-one-hand": neg_one_hand,
    "neg-blind-cover": neg_blind_cover,
    "neg-no-cover": neg_no_cover,
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
    import flipkit

    rt = flipkit.runtime()
    poses = rt.read()
    flat = {"trayA": poses["A"], "trayB": poses["B"]}
    flat.update({n: pr for n, pr in zip(flipkit.CHIPS, poses["chips"])})
    return dict(
        score=flipkit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in flat.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in flat.items()},
    )


def reset_report(stages, resets, results):
    import flipkit

    rows = []
    for i, r in enumerate(resets):
        want = flipkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in want
        )
        rdev = max(float(np.abs(np.subtract(r["rots"][n], want[n][1])).max()) for n in want)
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
    import flipkit
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
        d = flipkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=flipkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=flipkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
            a_rv=np.round(_rotvec(c.pose("A")[1]), 5).tolist(),
            b_rv=np.round(_rotvec(c.pose("B")[1]), 5).tolist(),
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} lvl {d.get('level')} "
                f"nest {m.get('nest_mm')} yaw {m.get('nest_yaw_deg')} aup {m.get('a_up')} bup {m.get('b_up')} "
                f"capt {m.get('n_captive')} cell {m.get('n_cell')} out {m.get('out')} hand {m.get('hand_mm')} "
                f"grip {d.get('grip_n')} veto {d.get('veto')} wall {time.time() - t1:.0f}s",
                flush=True,
            )
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
        if steps % 20 == 0 and (out / "STOP").exists():
            stop = "STOP file"
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
        scorer=dict(score=flipkit.STATE["score"], level=flipkit.STATE["level"]),
        final=flipkit.STATE["detail"],
        nonfinite=flipkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=flipkit.PHYSICS_DT,
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
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
