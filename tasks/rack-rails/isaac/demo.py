"""Scripted full-score demo (and negative variants) for the rack-rails task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the rail members' and the
rack's poses through ``rackkit.bench_state()`` (read-only, like a technician watching the pegs, the holes and the U
numbers); it never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's
per-step delta guard and clipped to the URDF joint bounds.

Plan (top-down pinch grips across the rail's web, each hand next to its end bracket):
1. Read the target U: the free U directly above the installed 1U server (server pose).
2. Left hand grips the outer member next to the rear bracket, right hand the inner member next to the front bracket.
3. Lift the closed rail out of the cradle, carry it level to the rack's near side at the target U's height, outside the
   posts, then in between them (pegs still inside the flanges).
4. Rear end: align the rear pegs with the rear flange's holes of the target U (closed loop on the measured outer
   member pose), slide the rail toward the rear until the pegs are in, then push (both hands) until the catch drops in.
5. Front end: the left hand keeps the rear end where it is; the right hand draws the inner member out toward the front
   post, aligns the front pegs with the front flange's holes of the same U, slides them in and pushes until the front
   catch drops in.
6. Open both hands, lift them clear and go back to READY.
Both arms add the remaining joint error back to their commands (slow integral on their own joint readings), because
the joint PD sags under load (pattern reused from earlier demos).
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
SQUEEZE = 0.004  # fingers commanded 4 mm inside the web faces: about 20 N per finger (gripper 5 kN/m)
OPEN_CLEAR = 0.010  # 10 mm clearance a side before closing / after letting go
PAD_DEPTH = 0.018  # pad tips this far below the member's top
GRIP_Y = {"left": -0.070, "right": 0.070}  # grip centres from the bracket faces (member frames)
STAGE_DX = 0.060  # staging: rail this far outside the posts' plane (toward the robot)
ROT_CORR_MAX = 0.10
ALIGN_TOL = 0.0003
PRE_GAP = 0.0088  # peg tips 0.8 mm short of the flange: final alignment here
PARK = (0.180, 0.200, 0.900)  # parked TCP (left; right mirrored), top-down: clear of the rail ends in the overhead view
ENGAGE_GAP = 0.0072  # peg tips 0.8 mm into the holes, inside the chamfers
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
        self.offset_clip = 0.25

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
                self.offset + COMP_GAIN * (self.q - self.measured), -self.offset_clip, self.offset_clip
            )
        else:
            self.offset *= 0.85

    def relax(self):
        """Command the arm to where it is (keeping the sag offsets): releases the spring force it stores against the
        rail before the fingers open."""
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
        for s in (np.array(READY, float), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
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
        self.corr = {}

    def pose(self, name="outer"):
        import rackkit

        return rackkit.bench_state()[name]

    def detail(self):
        import rackkit

        return rackkit.STATE["detail"]

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


def grip_of(half_opening_m):
    return float(np.clip(half_opening_m / FINGER_OPEN, 0.0, 1.0))


WEB = {  # member: (x centre, half thickness, top z) in the member frame
    "outer": ((L.OUTER_X[0] + L.OUTER_X[1]) / 2, (L.OUTER_X[1] - L.OUTER_X[0]) / 2, L.OUTER_Z),
    "inner": ((L.INNER_X[0] + L.INNER_X[1]) / 2, (L.INNER_X[1] - L.INNER_X[0]) / 2, L.INNER_Z),
}
HAND_MEMBER = {"left": "outer", "right": "inner"}


def grip_rot(rm, hand, flip=False):
    """Straight-down TCP, fingers closing across the web; the wrist camera on the rail's inboard side."""
    x = rm[:, 0] * (-1.0 if hand == "left" else 1.0) * (-1.0 if flip else 1.0)
    x = np.array([x[0], x[1], 0.0])
    x /= np.linalg.norm(x)
    z = np.array([0.0, 0.0, -1.0])
    return np.column_stack([x, np.cross(z, x), z])


def grip_point(pm, rm, hand):
    xm, _, top = WEB[HAND_MEMBER[hand]]
    from yam_arena.constants import PAD_TIP_PAST_TCP

    return pm + rm @ np.array([xm, GRIP_Y[hand], top - PAD_DEPTH + PAD_TIP_PAST_TCP])


def rack_frame(c):
    """(hole column x, depth centre y, target U, U centre z) read from the rack's and the installed server's poses."""
    pr, _ = c.pose("rack")
    ps, _ = c.pose("server")
    u_server = int(round((ps[2] - L.U1_BOTTOM) / L.U)) + 1
    u = u_server + 1 + getattr(c, "u_shift", 0)
    return float(pr[0]), float(pr[1]), u, L.u_center(u)


def outer_at(c, gap, dx=0.0):
    """Outer member pose with its rear bracket face `gap` short of the rear flange, at the target U."""
    xh, y0, u, zc = rack_frame(c)
    return np.array([xh + dx, y0 + L.Y_F - gap, zc]), np.eye(3)


def inner_at(c, gap):
    xh, y0, u, zc = rack_frame(c)
    return np.array([xh, y0 - L.Y_F + gap, zc]), np.eye(3)


def end_err(c, end):
    import rackkit

    spec = rackkit.runtime().spec
    pm, rm = c.pose("outer" if end == "rear" else "inner")
    return L.end_state(end, pm, rm, spec)


def latched(c, end):
    import rackkit

    return bool(rackkit.runtime().latch.latched[end])


def capture(c, hand, ref):
    pr, rr = c.pose(ref)
    pt, rt = c.arms[hand].measured_tcp()
    c.G[(hand, ref)] = (rr.T @ (pt - pr), rr.T @ rt)


def track(c, ref, hands, target_fn, duration, label, gain=0.05, rot_gain=None, free=(False, False, False),
          until=None, max_s=None, use_rot=True):
    """Closed loop on one rail member: each step command the given hands to the TCP poses that put member ``ref`` at
    target_fn(s) through their frozen grasp transforms, plus slow integral corrections of the measured member position
    (and orientation) error (arm sag, grasp slip). ``free``: world axes whose error is not integrated (pushing against
    a stop). ``until(c)`` ends the move early (checked after the nominal duration, up to ``max_s``)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    key = (ref, tuple(hands))
    cp, cr = c.corr.get(key, (np.zeros(3), np.zeros(3)))
    for k in range(1, nmax + 1):
        pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
        pm, rm = c.pose(ref)
        err = pt - pm
        err = np.where(np.asarray(free), 0.0, err)
        cp = np.clip(cp + gain * err, -0.015, 0.015)
        if use_rot:
            e_r = _rotvec(rt @ rm.T)
            cr = np.clip(cr + (gain if rot_gain is None else rot_gain) * e_r, -ROT_CORR_MAX, ROT_CORR_MAX)
        rc = _rotm(cr)
        for h in hands:
            gp, gr = c.G[(h, ref)]
            arm = c.arms[h]
            arm.label = label
            arm.set_pose(pt + cp + rc @ rt @ gp, rc @ rt @ gr)
        c.corr[key] = (cp, cr)
        yield
        if until is not None and k >= n and until(c):
            break


def aligned(end, tol=ALIGN_TOL, n=4):
    count = [0]

    def ok(c):
        st = end_err(c, end)
        good = abs(st["axis_dx"]) < tol and abs(st["axis_dz"]) < tol and st["tilt_deg"] < 0.8
        count[0] = count[0] + 1 if good else 0
        return count[0] >= n

    return ok


# --------------------------------------------------------------------------------------------------- the script
def pick(c):
    for arm in c.arms.values():
        arm.set_comp(True)
    tg = {}
    for h, arm in c.arms.items():
        pm, rm = c.pose(HAND_MEMBER[h])
        p = grip_point(pm, rm, h)
        best = None
        for flip in (False, True):
            r = grip_rot(rm, h, flip)
            q, pe, re = arm.solve(p, r)
            cost = (abs(q[5]) > 1.95) + 10.0 * (pe > 0.002 or re > 0.03) + 0.01 * flip  # wrist camera inboard if possible
            if best is None or cost < best[0]:
                best = (cost, r)
        tg[h] = (p, best[1])
    half = {h: WEB[HAND_MEMBER[h]][1] for h in c.arms}
    yield from both(*(arm.move(tg[h][0] + [0, 0, 0.06], tg[h][1], 3.0, grip_of(half[h] + OPEN_CLEAR), f"{h} over grip")
                      for h, arm in c.arms.items()))
    yield from wait(0.4)
    for h in c.arms:  # re-read (the rail does not move, but the plan uses the measured pose)
        pm, rm = c.pose(HAND_MEMBER[h])
        tg[h] = (grip_point(pm, rm, h), tg[h][1])
    yield from both(*(arm.move(tg[h][0], tg[h][1], 2.0, None, f"{h} descend") for h, arm in c.arms.items()))
    yield from wait(0.4)
    yield from both(*(arm.set_grip(grip_of(half[h] - SQUEEZE), 1.0, f"{h} close") for h, arm in c.arms.items()))
    yield from wait(0.6)
    c.note("gripped")
    yield from both(*(arm.move(arm.pos + [0, 0, 0.045], arm.rot, 2.0, None, f"{h} lift")
                      for h, arm in c.arms.items()))
    yield from wait(0.6)
    capture(c, "left", "outer")
    capture(c, "right", "outer")
    capture(c, "right", "inner")
    c.note("lifted", target=rack_frame(c))


def carry(c):
    p0, r0 = c.pose("outer")
    ps, rs = outer_at(c, 0.020, -STAGE_DX)
    zmid = max(p0[2], ps[2]) + 0.010
    yield from track(c, "outer", ("left", "right"),
                     lambda s: (np.array([p0[0] + s * (ps[0] - p0[0]), p0[1] + s * (ps[1] - p0[1]),
                                          p0[2] + s * (zmid - p0[2])]),
                                L.rz(math.degrees(math.atan2(r0[1, 0], r0[0, 0])) * (1 - s))),
                     5.0, "carry")
    yield from track(c, "outer", ("left", "right"),
                     lambda s: (np.array([ps[0], ps[1], zmid + s * (ps[2] - zmid)]), rs), 1.5, "to U height")
    pi_, _ = outer_at(c, 0.020)
    yield from track(c, "outer", ("left", "right"),
                     lambda s: (ps + s * (pi_ - ps), rs), 3.0, "between the posts", gain=0.06)
    c.note("between posts")


def hand_slip(c, hand, ref):
    """Measured shift (m) of the hand along the rail relative to the member it holds, since the grasp was captured."""
    pr, rr = c.pose(ref)
    pt, _ = c.arms[hand].measured_tcp()
    return float((rr.T @ (pt - pr) - c.G[(hand, ref)][0])[1])


def push_home(c, end, ref, hand, target, start_gap, depths=(0.025, 0.035), steady=None, steady_lead=0.0):
    """Push the end home against its catch with ``hand``: command its member past the flange with the joint integral
    on (it winds up the joint torques, as a position-controlled arm pushing against a stop does), until the catch drops
    in. An attempt ends early if the hand slips more than 6 mm along the rail (no pushing a slipping grip into the
    post). ``steady`` (other hand, holding the other member) keeps that member on the hole axis (x, z closed loop on its
    measured pose) and leads it along the rail by ``steady_lead`` (m), so the rail stays square and the pegs do not
    bind. Sets c.pushed = True when the catch engaged."""
    from yam_arena import ik

    c.pushed = False
    xh, y0, u, zc = rack_frame(c)
    if steady is not None:
        s_hand, s_ref = steady
        capture(c, s_hand, s_ref)
        s_corr = np.zeros(3)
        c.arms[s_hand].set_comp(True)
        c.arms[s_hand].offset_clip = 0.35

    def steady_step(label, lead):
        nonlocal s_corr
        if steady is None:
            return
        pm, rm = c.pose(s_ref)
        want = np.array([xh, pm[1], zc])
        err = want - pm
        err[1] = 0.0
        s_corr = np.clip(s_corr + 0.05 * err, -0.010, 0.010)
        gp, gr = c.G[(s_hand, s_ref)]
        arm = c.arms[s_hand]
        arm.label = label
        arm.set_pose(want + s_corr + gp + L.outward(end) * lead, gr)

    arm = c.arms[hand]
    saved = {h: a.offset.copy() for h, a in c.arms.items()}
    arm.set_comp(True)
    arm.offset_clip = 0.35
    capture(c, hand, ref)
    gp, gr = c.G[(hand, ref)]
    g0 = start_gap
    try:
        for k, depth in enumerate(depths):
            pa, ra = target(g0)
            pd, _ = target(-depth)
            n = round(2.5 * CONTROL_HZ)
            slipped = False
            for i in range(1, round(6.0 * CONTROL_HZ) + 1):
                s = ik.smoothstep(min(1.0, i / n))
                arm.label = f"{end}: push to latch {k}"
                arm.set_pose(pa + s * (pd - pa) + gp, gr)
                steady_step(f"{end}: steady", s * steady_lead)
                yield
                if latched(c, end):
                    c.pushed = True
                    return
                if abs(hand_slip(c, hand, ref)) > 0.006:
                    slipped = True
                    c.note("slip", end=end, attempt=k)
                    break
            # ease off to where the end is (no stored push), re-grip position unchanged
            pm, _ = c.pose(ref)
            pb = np.array([pa[0], pm[1], pa[2]]) - L.outward(end) * 0.001
            for i in range(1, round(1.0 * CONTROL_HZ) + 1):
                arm.label = f"{end}: ease off {k}"
                arm.set_pose(pb + gp, gr)
                steady_step(f"{end}: steady", 0.0)
                yield
            if slipped:
                return
            g0 = float(L.outward(end) @ np.array([0.0, (y0 + (L.Y_F if end == "rear" else -L.Y_F)) - pm[1], 0.0]))
    finally:
        # the pushing arms go back to their pre-push sag offsets and are commanded to where they are, so they stop
        # pushing (once latched, the catch holds the end)
        for h, a in c.arms.items():
            a.offset_clip = 0.25
            if h == hand or (steady is not None and h == steady[0]):
                a.offset = saved[h]
                a.relax()


def insert_end(c, end, push=True, tries=3):
    """Align one end's pegs with its flange's holes at the target U (closed loop on the measured member pose), slide
    them in (the tapered noses guide; no integral then), and push the end home. If the pegs stop short of the catch zone
    (jammed on a hole edge) or the push does not latch, back out to the alignment point and try again."""
    rear = end == "rear"
    ref = "outer" if rear else "inner"
    hands = ("left", "right") if rear else ("right",)
    at = (lambda g: outer_at(c, g)) if rear else (lambda g: inner_at(c, g))
    use_rot = rear
    for t in range(tries):
        for h in hands:
            c.arms[h].set_comp(True)
        if rear:
            capture(c, "left", "outer")
            capture(c, "right", "outer")
        else:
            c.left.set_comp("hold")
            capture(c, "right", "inner")
        c.corr.pop((ref, hands), None)
        p0, _ = c.pose(ref)
        pb, rb = at(PRE_GAP)
        yield from track(c, ref, hands, lambda s, p0=p0, pb=pb: (p0 + s * (pb - p0), rb), 2.5 if t == 0 else 1.5,
                         f"{end}: align {t}", gain=0.06, use_rot=use_rot, until=aligned(end), max_s=8.0)
        st = end_err(c, end)
        c.note(f"{end} aligned", attempt=t, dx_mm=st["axis_dx"] * 1e3, dz_mm=st["axis_dz"] * 1e3, tilt=st["tilt_deg"])
        pe, _ = at(ENGAGE_GAP)
        yield from track(c, ref, hands, lambda s, pe=pe, pb=pb: (pb + s * (pe - pb), rb), 1.0, f"{end}: into the noses",
                         gain=0.06, use_rot=use_rot)
        for h in hands:
            c.arms[h].set_comp("hold")  # from here the peg noses guide the end; the arms' own compliance follows them
        pc, _ = at(0.002)
        yield from track(c, ref, hands, lambda s, pe=pe, pc=pc: (pe + s * (pc - pe), rb), 2.5, f"{end}: pegs in",
                         gain=0.05, free=(True, True, True), rot_gain=0.0, use_rot=use_rot)
        st = end_err(c, end)
        if st["gap"] > L.LATCH_ZONE + 0.0015:
            c.note(f"{end} jammed", attempt=t, gap_mm=st["gap"] * 1e3, dx_mm=st["axis_dx"] * 1e3, dz_mm=st["axis_dz"] * 1e3)
        elif not push:
            return
        else:
            if rear:
                yield from push_home(c, "rear", "outer", "left", at, st["gap"], steady=("right", "inner"),
                                     steady_lead=0.015)
            else:
                yield from push_home(c, "front", "inner", "right", at, st["gap"])
            if c.pushed:
                c.note(end, latched=True, attempt=t, st=end_err(c, end))
                return
        # back out to the alignment point (pegs out of the holes) and try again
        for h in hands:
            c.arms[h].set_comp(True)
        if rear:
            capture(c, "left", "outer")
            capture(c, "right", "outer")
        else:
            capture(c, "right", "inner")
        c.corr.pop((ref, hands), None)
        p1, _ = c.pose(ref)
        pz, _ = at(PRE_GAP + 0.002)
        yield from track(c, ref, hands, lambda s, p1=p1, pz=pz: (p1 + s * (pz - p1), rb), 1.5, f"{end}: back out {t}",
                         gain=0.0, free=(True, True, True), rot_gain=0.0, use_rot=use_rot)
    raise Stop(f"{end} end not latched after {tries} tries")


def rear_insert(c, push=True):
    yield from insert_end(c, "rear", push)


def front_insert(c, push=True):
    yield from insert_end(c, "front", push)


def let_go(c, hands=("left", "right")):
    """Relax both arms (command them to where they are: releases the push they store against the latched rail, so the
    hands do not spring into the posts when the fingers open), open, lift clear, back out, go home."""
    for h in hands:
        c.arms[h].relax()
    yield from wait(0.4)
    yield from both(*(c.arms[h].set_grip(grip_of(WEB[HAND_MEMBER[h]][1] + OPEN_CLEAR), 1.0, f"{h} open")
                      for h in hands))
    yield from wait(0.3)
    yield from both(*(c.arms[h].move(c.arms[h].pos + [0, 0, 0.024], c.arms[h].rot, 1.5, None, f"{h} up")
                      for h in hands))
    yield from both(*(c.arms[h].move(c.arms[h].pos + [-0.100, 0, -0.030], c.arms[h].rot, 1.5, None, f"{h} back")
                      for h in hands))
    for h in hands:
        c.arms[h].set_comp(True)
    # park the hands low near the arm bases (not READY, whose grippers hide the rail ends from the overhead camera)
    yield from both(*(c.arms[h].move(np.array([PARK[0], PARK[1] * (1 if h == "left" else -1), PARK[2]]), c.arms[h].rot,
                                     2.5, 0.0, f"{h} park") for h in hands))
    c.note("released")


def demo(c):
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from rear_insert(c)
    yield from front_insert(c)
    yield from let_go(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def carry_only(c):
    """Development: pick, carry between the posts, hold."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from wait(2.0)


def rear_only_dev(c):
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from rear_insert(c)
    yield from wait(2.0)


def neg_wrong_u(c):
    """Mounts the rail one U higher than the free U above the server (leaving a gap), both ends latched there: 0."""
    c.u_shift = 1
    yield from demo(c)


def neg_rear_only(c):
    """Rear end latched at the right U, front end never drawn out to its post; hands let go: 0.25."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from rear_insert(c)
    yield from let_go(c)
    yield from wait(4.0)


def neg_front_no_push(c):
    """Rear end latched at the right U; front pegs slid into the right U's holes but never pushed past the catch
    (stops 2 mm short); hands let go, the catch pushes the front end back to the edge of its zone: 0.5 (front pegged,
    not latched)."""
    yield from wait(0.5)
    yield from pick(c)
    yield from carry(c)
    yield from rear_insert(c)
    yield from front_insert(c, push=False)
    yield from let_go(c)
    yield from wait(4.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in RACK_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["RACK_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "carry-only": carry_only,
    "rear-only-dev": rear_only_dev,
    "replay": replay,
    "neg-wrong-u": neg_wrong_u,
    "neg-rear-only": neg_rear_only,
    "neg-front-no-push": neg_front_no_push,
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
    seeds=None,
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
    # per-trial seeds: --seeds a,b,... ('none' = unseeded named setup); otherwise --seed for every trial
    trial_seeds = [None if x in ("none", "") else int(x) for x in seeds.split(",")] if seeds else [seed] * len(stages)
    results, resets = [], []
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=trial_seeds[i])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import rackkit

    rt = rackkit.runtime()
    poses = rt.read()
    return dict(
        score=rackkit.STATE["score"],
        setup=rt.setup,
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import rackkit

    rows = []
    for i, r in enumerate(resets):
        want = rackkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("outer", "inner", "rack", "server")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["outer"], want["outer"][1])).max())
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
        x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 2.0  # rail settles in its cradle
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
    import rackkit
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
        d = rackkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=rackkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=rackkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm={f"{k[0]}:{'+'.join(k[1])}": np.round(v[0] * 1000, 3).tolist() for k, v in c.corr.items()},
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
            rr, ff = m.get("rear", {}), m.get("front", {})
            fmt = lambda e: {k: (round(e[k] * 1e3, 2) if k in ("gap", "axis_dx", "axis_dz") else e.get(k)) for k in ("gap", "axis_dx", "axis_dz", "u", "tilt_deg") if k in e}  # noqa: E731
            print(
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} rear {fmt(rr)} "
                f"front {fmt(ff)} latched {m.get('latched')} ext {m.get('extension_mm')} tip {m.get('tip_mm')} "
                f"grip {d.get('contacts')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=rackkit.STATE["score"], level=rackkit.STATE["level"]),
        final=rackkit.STATE["detail"],
        nonfinite=rackkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=rackkit.PHYSICS_DT,
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
    ap.add_argument(
        "--seeds",
        default=None,
        help="comma-separated reset seed per trial of a multi-stage run ('none' = unseeded named setup)",
    )
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed, a.seeds)
