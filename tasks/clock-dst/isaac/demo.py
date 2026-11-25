"""Scripted full-score demo (and negative variants) for the clock-dst task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the clock's pose, the knob
angle and the time shown through ``clockkit.bench_state()`` (read-only, like a person watching the hands and the
nail); it never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's per-step
delta guard and clipped to the URDF joint bounds.

PLAN_PLACEHOLDER
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
        ring before the fingers open."""
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

    def solve_global(self, pos, rot, n=80):
        """IK from many fixed pseudo-random seeds (for joint-space moves to a new arm configuration): the solution
        nearest the current angles."""
        rng = np.random.default_rng(7)
        best = None
        for _ in range(n):
            s = rng.uniform(self.lo, self.hi)
            q, pe, re = fastik.solve(pos, rot, s, base=self.base, iters=150)
            if pe < 3e-4 and re < 5e-3:
                cost = float(np.abs(q - self.q).sum())
                if best is None or cost < best[0]:
                    best = (cost, q)
        if best is None:
            raise Stop(f"{self.name} unreachable (global) at {self.label}")
        return best[1]

    def move_joint(self, pos, rot, duration, grip=None, label=""):
        """Joint-space move (smoothstep) to an IK solution of (pos, rot) found from many seeds."""
        from yam_arena import ik

        self.label = label or self.label
        q0, g0 = self.q.copy(), self.grip
        q1 = self.solve_global(pos, rot)
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.grip = g0 + s * (g1 - g0)
            self.pos, self.rot = fastik.tcp_pose(self.q, self.base)
            yield
        self.pos, self.rot = np.asarray(pos, float), np.asarray(rot, float)

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

    def pose(self, name="clock"):
        import clockkit

        return clockkit.bench_state()[name]

    def detail(self):
        import clockkit

        return clockkit.STATE["detail"]

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


# ------------------------------------------------------------------------------------------- clock-dst plan
# (this block is concatenated into demo.py between the shared infrastructure and the run loop; see build_demo.sh)
R_PINCH = (L.BEZEL_RI + L.CLOCK_R) / 2  # rim pinch: fingers straddle the 16 mm wide bezel ring, from the front
BEZEL_HALF = (L.CLOCK_R - L.BEZEL_RI) / 2
TIP_DEPTH = 0.007  # pad tips 7 mm behind the bezel front face (2 mm clear of the recessed glass)
TIP_PAST_TCP = 0.013
KZ_PINCH = L.BEZEL_Z[1] - TIP_DEPTH + TIP_PAST_TCP  # TCP z_K (in front of the bezel)
OPEN_BEZEL = BEZEL_HALF + 0.006  # 6 mm clearance a side before closing / after letting go
SQUEEZE_BEZEL = 0.0035  # fingers commanded 3.5 mm inside the ring faces: about 17 N per finger
OPEN_KNOB = L.KNOB_R + 0.0055
SQUEEZE_KNOB = 0.0025  # about 12 N per finger on the knob rim
KNOB_TIP_KZ = L.MOVE_Z[0] - 0.0020  # pad tips stop 2 mm short of the movement's back face (3 mm on the 5 mm knob)
ANG = {"left": 150.0, "right": 30.0, "top": 90.0}  # rim pinch points (deg from 3 o'clock toward 12): 10 and 2 o'clock
PULL = 0.13  # how far the clock comes off the wall before it is set down
REST_Z = L.CLOCK_R + 0.0003  # K origin height above the table when the clock stands on its bezel rim
TABLE_PRESS = 0.0010  # the holding hand keeps the rim pressed 1 mm "into" the table (the table carries the weight)
STROKE_DEG = 200.0  # knob turned per regrasp (the wrist roll allows 240 deg here)
TIME_TOL_MIN = 0.4  # the turning loop stops within 0.4 min of 10:10 (scoring needs 2)


def rz_m(deg):
    return L.rz(deg)


def rax(axis, ang):
    axis = np.asarray(axis, float) / np.linalg.norm(axis)
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(ang) * k + (1 - math.cos(ang)) * (k @ k)


def pinch_k(ang_deg, flip):
    """TCP pose in the clock frame K for a rim pinch at rim angle ang_deg: approach from the front (-z_K, toward the
    face), fingers closing radially (sign = flip) across the bezel ring, one finger outside the rim and one inside the
    bezel window in front of the glass."""
    a = math.radians(ang_deg)
    rd = np.array([math.cos(a), math.sin(a), 0.0])
    p = R_PINCH * rd + np.array([0.0, 0.0, KZ_PINCH])
    z = np.array([0.0, 0.0, -1.0])
    x = rd * (1.0 if flip > 0 else -1.0)
    return p, np.column_stack([x, np.cross(z, x), z])


def to_world(cp, cr, pk):
    p, r = pk
    return cp + cr @ p, cr @ r


def clock_pose(c):
    import clockkit

    return clockkit.bench_state()["clock"]


def bench(c):
    import clockkit

    return clockkit.bench_state()


def choose_flip(arm, poses_k_fn):
    """Finger orientation (+1 / -1 about the approach) for which every listed world pose is reachable with the least
    wrist-roll travel."""
    best = None
    for flip in (1.0, -1.0):
        q = arm.q.copy()
        worst, j6 = 0.0, 0.0
        for p, r in poses_k_fn(flip):
            q, pe, re = fastik.solve(p, r, q, base=arm.base, iters=120)
            if pe > 5e-4 or re > 0.01:
                q2 = None
                for s in (np.array([0.0, 1.0, 1.0, -0.6, 0.0, 0.0]), np.array([0.0, 1.6, 1.4, -1.2, 0.0, 0.0])):
                    qq, pe2, re2 = fastik.solve(p, r, s, base=arm.base, iters=200)
                    if pe2 < 5e-4 and re2 < 0.01:
                        q2 = qq
                        break
                if q2 is None:
                    worst = 1e9
                    break
                q = q2
            j6 = max(j6, abs(q[5]))
        if worst < 1e9 and (best is None or j6 < best[0]):
            best = (j6, flip)
    if best is None:
        raise Stop(f"{arm.name}: no reachable finger orientation")
    return best[1]


def capture(c, names):
    """Freeze the clock -> TCP transform of each named hand (measured, with the clock held still)."""
    cp, cr = clock_pose(c)
    for n in names:
        pt, rt = c.arms[n].measured_tcp()
        c.G[n] = (cr.T @ (pt - cp), cr.T @ rt)
    c.corr_p, c.corr_r = np.zeros(3), np.zeros(3)


def track(
    c,
    target_fn,
    duration,
    label,
    names=("left", "right"),
    gain=0.0,
    until=None,
    max_s=None,
    mask=(1, 1, 1),
    abort=None,
    kp=0.0,
    rot_lim=0.08,
):
    """Move the held clock along target_fn(s) (s 0..1 smoothstep): each step command the named hands to the TCP poses
    that put the clock there through the frozen grasp transforms. With gain > 0 a single slow integrator corrects the
    measured clock position / orientation error (rate-limited to 0.3 mm and 0.1 deg per step); the arms' own joint
    integrators are frozen meanwhile (two integrators in series made the loaded arms oscillate)."""
    from yam_arena import ik

    n = max(1, round(duration * CONTROL_HZ))
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    saved = {name: c.arms[name].comp for name in names}
    if gain > 0:
        for name in names:
            c.arms[name].set_comp("hold")
    try:
        for k in range(1, nmax + 1):
            pt, rt = target_fn(ik.smoothstep(min(1.0, k / n)))
            if gain > 0:
                pm, rm = clock_pose(c)
                dp = np.clip(gain * (pt - pm) * np.asarray(mask, float), -0.0003, 0.0003)
                dr = np.clip(gain * _rotvec(rt @ rm.T), -0.0017, 0.0017)
                c.corr_p = np.clip(c.corr_p + dp, -0.015, 0.015)
                c.corr_r = np.clip(c.corr_r + dr, -rot_lim, rot_lim)
            pfb = np.zeros(3)
            if kp > 0:  # proportional feedback on the measured clock position (no accumulation)
                pm, _ = clock_pose(c)
                pfb = np.clip(kp * (pt - pm) * np.asarray(mask, float), -0.003, 0.003)
            rc = _rotm(c.corr_r)
            for name in names:
                gp, gr = c.G[name]
                arm = c.arms[name]
                arm.label = label
                arm.set_pose(pt + c.corr_p + pfb + rc @ rt @ gp, rc @ rt @ gr)
            yield
            if abort is not None and abort(c):
                c.aborted = True
                break
            if until is not None and k >= n and until(c):
                break
    finally:
        if gain > 0:
            for name in names:
                c.arms[name].set_comp(saved[name])


def grasp_bezel(c, name, ang, flip, label, dur=2.0, joint=True):
    """Open hand in front of the rim point (40 mm out along the approach), then in, then close on the bezel."""
    arm = c.arms[name]
    cp, cr = clock_pose(c)
    p, r = to_world(cp, cr, pinch_k(ang, flip))
    mv = arm.move_joint if joint else arm.move
    yield from mv(pre_pose(arm, p, r, 0.040), r, dur, grip_of(OPEN_BEZEL), f"{name} to {label}")
    cp, cr = clock_pose(c)
    p, r = to_world(cp, cr, pinch_k(ang, flip))
    yield from arm.move(p, r, 1.2, None, f"{name} onto {label}")
    for _ in range(int(2.0 * CONTROL_HZ)):  # let the hand settle on the commanded pose (joint integrators) first
        if np.linalg.norm(arm.measured_tcp()[0] - arm.pos) < 0.0015:
            break
        yield
    yield from arm.set_grip(grip_of(BEZEL_HALF - SQUEEZE_BEZEL), 0.8, f"{name} close {label}")
    yield from wait(0.3)


def reachable(arm, p, r):
    q, pe, re = arm.solve(p, r)
    return pe < 0.002 and re < math.radians(2)


def retreat(arm, back, label, dur=1.0):
    """Back the hand off along its approach axis (or up, or between, whichever the arm reaches)."""
    a = arm.rot[:, 2]
    up = np.array([0.0, 0.0, 1.0])
    for d in (-a, (-a + up) / np.linalg.norm(-a + up), up, -a * 0.5):
        tgt = arm.pos + d * back
        if reachable(arm, tgt, arm.rot):
            yield from arm.move(tgt, arm.rot, dur, None, label)
            return
    raise Stop(f"{arm.name}: no retreat from {label}")


def pre_pose(arm, p, r, dist):
    """A reachable stand-off position `dist` out from a grasp pose: back along the approach, or from above."""
    a = r[:, 2]
    up = np.array([0.0, 0.0, 1.0])
    for d in (-a, (-a + up) / np.linalg.norm(-a + up), up):
        for f in (1.0, 0.6):
            tgt = p + d * dist * f
            try:
                arm.solve_global(tgt, r, n=30)
                return tgt
            except Stop:
                continue
    raise Stop(f"{arm.name}: no stand-off pose")


def release_bezel(c, name, label, back=0.045, open_s=0.6, relax=True):
    arm = c.arms[name]
    if relax:
        arm.relax()
    yield from arm.set_grip(grip_of(OPEN_BEZEL), open_s, f"{name} open {label}")
    # unloaded now: the joint integrators must re-adapt (offsets frozen while it carried the clock would lift the hand)
    arm.set_comp(True)
    yield from retreat(arm, back, f"{name} off {label}")


def read_start(c):
    b = bench(c)
    c.start_min = b["shown_min"]
    c.nail = np.asarray(b["nail_point"], float)
    c.note("start", shown=L.fmt(b["shown_min"]), nail_mm=np.round(c.nail * 1e3, 1).tolist())


def take_off(c):
    """Both hands pinch the bezel at the upper sides, lift 12 mm (nail shank down the slot into the entry hole), pull
    the clock off (head out through the hole), and stand it on its rim on the table."""
    for arm in c.arms.values():
        arm.set_comp(True)
    cp, cr = clock_pose(c)
    c.flip = {}
    for name in ("left", "right"):
        ang = ANG[name]
        c.flip[name] = choose_flip(
            c.arms[name], lambda f, ang=ang: [to_world(cp, cr, pinch_k(ang, f))]
        )
    yield from both(
        grasp_bezel(c, "left", ANG["left"], c.flip["left"], "rim", joint=False),
        grasp_bezel(c, "right", ANG["right"], c.flip["right"], "rim", joint=False),
    )
    capture(c, ("left", "right"))
    p0, r0 = clock_pose(c)
    c.note("gripped")
    # slide the clock up the wall until the entry hole is centred on the nail (closed loop on the measured clock pose:
    # the arms sag under its weight), then pull it straight off, slowly while the head passes the hole
    n = c.nail
    p1 = n - r0 @ np.array([L.KEY_HOLE_C[0], L.KEY_HOLE_C[1], 0.0])
    p1[0] = p0[0]

    def centred_n(k):
        cnt = [0]

        def ok(c):
            pm, rm = clock_pose(c)
            nk, _ = L.nail_in_k(pm, rm, n)
            good = abs(nk[1] - L.KEY_HOLE_C[1]) < 0.0005 and abs(nk[0]) < 0.0006
            cnt[0] = cnt[0] + 1 if good else 0
            return cnt[0] >= k

        return ok

    # slow lift (the arms lag under the clock's weight and overshoot if hurried), then settle on the hole centre
    yield from track(c, lambda s: (p0 + s * (p1 - p0), r0), 2.5, "lift off the nail")
    for attempt in range(4):
        yield from track(c, lambda s: (p1, r0), 0.5, "centre on the hole", kp=0.7, until=centred_n(8), max_s=6.0)
        pm, rm = clock_pose(c)
        c.note("over the hole", attempt=attempt, nail_k_mm=np.round(L.nail_in_k(pm, rm, n)[0] * 1e3, 2).tolist())
        # head out slowly with the arms' own sag compensation only; if the head catches the panel (the nail pushes
        # back, as felt in a real pull), go back in and centre again
        c.aborted = False
        yield from track(
            c,
            lambda s: (p1 + [-0.02 * s, 0, 0], r0),
            3.0,
            "head out",
            kp=0.7,
            mask=(0, 1, 1),
            abort=lambda c: nail_force(c) > 2.0,
        )
        if not c.aborted:
            break
        c.note("head caught", attempt=attempt, nail_n=nail_force(c))
        pcur = np.array([clock_pose(c)[0][0], p1[1], p1[2]])
        yield from track(c, lambda s, pcur=pcur: (pcur + s * (p1 - pcur), r0), 1.0, "back in", kp=0.7, mask=(0, 1, 1))
    pq = p1 + [-0.02, 0, 0]
    yield from track(c, lambda s: (pq + [-(PULL - 0.02) * s, 0, 0], r0), 2.2, "pull off")
    p2 = p1 + [-PULL, 0, 0]
    p3 = np.array([p2[0], p2[1], L.TOP + REST_Z - TABLE_PRESS])

    def resting(c):
        return clock_pose(c)[0][2] < L.TOP + REST_Z + 0.0015

    yield from track(c, lambda s: (p2 + s * (p3 - p2), r0), 2.0, "down to the table", until=resting, max_s=4.0)
    yield from wait(0.3)
    c.note("on table")


def face_yaw(cr):
    """Heading (deg) of the face normal in the horizontal plane (180 = facing the robot)."""
    return math.degrees(math.atan2(cr[1, 2], cr[0, 2]))


def quarter_turn(c, name, psi_target, label):
    """One hand (gripping the top of the rim) turns the clock standing on its rim about the vertical through its rim
    contact, until its face heads psi_target (deg, absolute: the turn is measured, not assumed)."""
    cp, cr = clock_pose(c)
    deg = (psi_target - face_yaw(cr) + 180.0) % 360.0 - 180.0
    pivot = cp + cr @ np.array([0.0, -L.CLOCK_R, (L.BEZEL_Z[0] + L.BEZEL_Z[1]) / 2])
    capture(c, (name,))
    cz = np.array(cp, float)

    def yaw(s):
        rr = rz_m(deg * s)
        return np.r_[pivot[:2], 0.0] + rr @ (cz - np.r_[pivot[:2], 0.0]), rr @ cr

    yield from track(c, yaw, 4.0, label, names=(name,))
    # the rim drags on the bench and the arm gives: finish the turn closed loop on the measured clock orientation
    pe, re = yaw(1.0)
    cnt = [0]

    def turned(c):
        _, rm = clock_pose(c)
        cnt[0] = cnt[0] + 1 if math.degrees(np.linalg.norm(_rotvec(re @ rm.T))) < 2.5 else 0
        return cnt[0] >= 5

    yield from track(c, lambda s: (pe, re), 0.3, f"{label}: finish", names=(name,), gain=0.06, mask=(0, 0, 0),
                     rot_lim=0.35, until=turned, max_s=5.0)
    yield from wait(0.3)


def turn_around(c):
    """The left hand moves its grip to the top of the rim (12 o'clock) while the right steadies the clock; the right lets
    go; the left turns the clock a quarter turn on its rim (back toward the right hand). Held at the top and standing on
    its rim, the clock cannot lean."""
    yield from release_bezel(c, "left", "rim")
    cp, cr = clock_pose(c)
    c.flip["left_top"] = choose_flip(c.left, lambda f: [to_world(cp, cr, pinch_k(ANG["top"], f))])
    yield from grasp_bezel(c, "left", ANG["top"], c.flip["left_top"], "top", dur=2.0)
    capture(c, ("left",))
    yield from release_bezel(c, "right", "rim")
    yield from c.right.home(1.2)
    yield from quarter_turn(c, "left", 85.0, "quarter turn")  # face heading 85 deg: the knob faces a bit forward
    cp, cr = clock_pose(c)
    c.note("turned", face_yaw=face_yaw(cr))


def knob_frame(c, roll_deg, pose=None):
    """TCP pose for the knob at a wrist roll (deg) about the approach (+z_K, from the back toward the face). ``pose``:
    a frozen clock pose (during a stroke the target must not follow the clock, or a push on the knob runs away)."""
    cp, cr = clock_pose(c) if pose is None else pose
    a = cr[:, 2].copy()
    if a[2] > 0.0:  # never point the hand upward (out of the YAM's reach): level it, the knob still between the pads
        a[2] = 0.0
        a /= np.linalg.norm(a)
    tip = cp + cr @ np.array([0.0, 0.0, KNOB_TIP_KZ])
    tcp = tip - a * TIP_PAST_TCP
    x0 = np.cross(a, [0.0, 0.0, 1.0])
    x0 /= np.linalg.norm(x0)
    x = rax(a, math.radians(roll_deg)) @ x0
    return tcp, np.column_stack([x, np.cross(a, x), a])


def plan_stroke(c, arm, turn):
    """(turn, start roll) of one knob stroke that the arm can follow continuously: from the stand-off pose at the start
    roll, in onto the knob, then through the whole roll range of the stroke on one IK branch (the reachable roll window
    depends on where the clock stands); shorter strokes if needed."""
    for t_ in (turn, 0.8 * turn, 0.6 * turn, 0.4 * turn, 0.25 * turn):
        for r0 in (t_ / 2.0, t_ / 2.0 + 20.0, t_ / 2.0 - 20.0, t_ / 2.0 + 40.0, t_ / 2.0 - 40.0, t_, 0.0):
            tcp, rot = knob_frame(c, r0)
            try:
                q = arm.solve_global(tcp - rot[:, 2] * 0.035, rot, n=25)
            except Stop:
                continue
            ok = True
            for k in range(11):
                roll = r0 - t_ * k / 10.0
                tcp, rot = knob_frame(c, roll)
                q, pe, re = fastik.solve(tcp, rot, q, base=arm.base, iters=100)
                if pe > 5e-4 or re > 0.01:
                    ok = False
                    break
            if ok:
                return t_, r0
    raise Stop(f"{arm.name}: no reachable knob stroke")


def minutes_left(c):
    target = getattr(c, "time_target", None)
    return (L.TARGET_MIN if target is None else target) - bench(c)["shown_min"]


def set_time(c):
    """Right hand: pinch the setting knob through the back opening and turn it (regrasping) until the hands show 10:10;
    the minute hand turns clockwise (seen from the front) when the knob turns anticlockwise seen from the back."""
    arm = c.right
    arm.set_comp(True)
    tries = 0
    while True:
        left = minutes_left(c)
        c.note("knob", minutes_to_go=round(left, 3))
        if abs(left) <= TIME_TOL_MIN or tries >= 8:
            break
        tries += 1
        turn, r0 = plan_stroke(c, arm, float(np.clip(left / 60.0 * 360.0, -STROKE_DEG, STROKE_DEG)))
        tcp, rot = knob_frame(c, r0)
        a = rot[:, 2]
        # joint-space move: the wrist unwinds up to 200 deg between strokes (a Cartesian slerp would turn the short way)
        yield from arm.move_joint(tcp - a * 0.035, rot, 2.4 if tries == 1 else 1.7, grip_of(OPEN_KNOB), "to knob")
        tcp, rot = knob_frame(c, r0)
        yield from arm.move(tcp, rot, 1.0, None, "onto knob")
        yield from arm.set_grip(grip_of(L.KNOB_R - SQUEEZE_KNOB), 0.6, "pinch knob")
        yield from wait(0.2)
        arm.set_comp("hold")
        n = max(8, int(abs(turn) / 60.0 * CONTROL_HZ))  # 60 deg/s
        frozen = clock_pose(c)
        q_start = bench(c)["knob_q"]
        for k in range(1, n + 1):
            from yam_arena import ik

            s = ik.smoothstep(k / n)
            tcp, rot = knob_frame(c, r0 - turn * s, frozen)
            arm.label = f"turn knob {tries}"
            try:
                arm.set_pose(tcp, rot)
            except Stop:  # the wrist ran out of reach mid-stroke: stop this stroke here, the loop re-plans the rest
                c.note("stroke cut short", at_roll=round(r0 - turn * s, 1))
                break
            yield
        yield from wait(0.3)
        c.note(
            "turned knob",
            stroke_deg=round(turn, 1),
            knob_deg=round(math.degrees(bench(c)["knob_q"] - q_start), 2),
        )
        yield from arm.set_grip(grip_of(OPEN_KNOB), 0.5, "release knob")
        yield from retreat(arm, 0.035, "off knob", 0.8)
        arm.set_comp(True)
    c.note("time set", shown=L.fmt(bench(c)["shown_min"]))


def turn_back(c):
    """The left turns the clock back to face the room; the right takes its 2 o'clock rim grip again."""
    yield from c.right.home(1.2)
    yield from quarter_turn(c, "left", 180.0, "quarter turn back")
    # Cartesian approach from READY (a joint-space move swept the hand through the clock)
    yield from grasp_bezel(c, "right", ANG["right"], c.flip["right"], "rim", dur=2.4, joint=False)
    # the clock is carried and hung with these two grips (top + 2 o'clock): a single rim grip cannot keep a clock
    # standing on its rim from tipping over, so the left does not change its grip on the bench


def hang(c):
    """Carry the clock level to the nail (seen on the wall), entry hole on the nail axis, push it flat to the wall
    (head through the hole), lower until the nail takes its weight, let go, hands away."""
    for arm in c.arms.values():
        arm.set_comp(True)
    capture(c, ("left", "right"))
    p0, r0 = clock_pose(c)
    rl = L.R_WALL
    n = c.nail
    hole = np.array([L.KEY_HOLE_C[0], L.KEY_HOLE_C[1], 0.0])
    # K origin when the entry hole is on the nail axis, back plane `gap` in front of the wall
    def at(gap, dz=0.0):
        return n - rl @ hole + np.array([-gap, 0.0, dz])

    off = np.asarray(getattr(c, "hang_offset", (0.0, 0.0, 0.0)), float)  # negatives only
    n = n + off
    pa = at(0.030)
    up = p0 + [0, 0, 0.03]
    yield from track(c, lambda s: (p0 + s * (up - p0), r0), 1.2, "lift from table")
    yield from track(c, lambda s: (up + s * (pa - up), _rotm(s * _rotvec(rl @ r0.T)) @ r0), 3.5, "carry to the nail")
    yield from track(c, lambda s: (pa, rl), 1.5, "line up", gain=0.06, until=lambda c: aligned(c, pa), max_s=4.0)
    c.note("lined up", err_mm=np.round((clock_pose(c)[0] - pa) * 1e3, 2).tolist())
    pb = at(-0.0004)
    for arm in c.arms.values():
        arm.set_comp("hold")
    yield from track(c, lambda s: (pa + s * (pb - pa), rl), 2.0, "onto the nail", gain=0.04, mask=(0, 1, 1))
    drop = L.HANG_SHANK_K[1] - L.KEY_HOLE_C[1]
    pc = pb + [0, 0, -(drop + 0.0015)]

    def loaded(c):
        return bench(c) is not None and nail_force(c) >= 0.7 * L.CLOCK_MASS * 9.81

    yield from track(c, lambda s: (pb + s * (pc - pb), rl), 2.0, "down on the nail", until=loaded, max_s=3.0)
    c.note("on nail", nail_n=nail_force(c))
    # press the clock flat to the wall (2 mm past flush, held by the arms' compliance) and a little further down
    pd = pc + [0.002, 0, -0.001]
    yield from track(c, lambda s: (pc + s * (pd - pc), rl), 0.6, "flat to the wall")
    # let go one hand at a time, gently: first the 2 o'clock hand (the top grip keeps the clock level on the nail), then
    # the top hand (its grip is above the nail, so opening it does not turn the clock)
    yield from release_bezel(c, "right", "rim", 0.05, open_s=1.2, relax=False)
    yield from wait(0.3)
    yield from release_bezel(c, "left", "top", 0.05, open_s=1.2, relax=False)
    yield from both(c.left.home(2.0), c.right.home(2.0))


def nail_force(c):
    import clockkit

    return float(clockkit.runtime().nail_n)


def aligned(c, target):
    pm, rm = clock_pose(c)
    return float(np.linalg.norm(pm - target)) < 0.0008


def demo(c):
    yield from wait(0.3)
    read_start(c)
    yield from take_off(c)
    yield from turn_around(c)
    yield from set_time(c)
    yield from turn_back(c)
    yield from hang(c)
    yield from wait(4.0)


def idle(c):
    yield from wait(8.0)


def neg_fixed60(c):
    """Turns the knob exactly one hour (the daylight-saving change) from the start, without reading the face for the
    clock's drift: hung level but the time is off by the drift (score from the time error: 0 to 0.5)."""
    yield from wait(0.3)
    read_start(c)
    c.time_target = c.start_min + 60.0
    yield from take_off(c)
    yield from turn_around(c)
    yield from set_time(c)
    yield from turn_back(c)
    yield from hang(c)
    yield from wait(4.0)


def neg_fixed50(c):
    """Face-blind fixed turn: always adds 50 minutes, whatever the clock shows (the shortcut a verifier tried): hung
    level, time off by the start's distance from 9:20 (0 at the default seed: 11 min off)."""
    yield from wait(0.3)
    read_start(c)
    c.time_target = c.start_min + 50.0
    yield from take_off(c)
    yield from turn_around(c)
    yield from set_time(c)
    yield from turn_back(c)
    yield from hang(c)
    yield from wait(4.0)


def neg_noset(c):
    """Takes the clock off, turns it round and back, hangs it back without touching the knob: 0."""
    yield from wait(0.3)
    read_start(c)
    yield from take_off(c)
    yield from turn_around(c)
    yield from turn_back(c)
    yield from hang(c)
    yield from wait(4.0)


def neg_miss(c):
    """Sets the time, but lines the keyhole up 5 mm to the side of the nail (as if hung blind from a wrong memory):
    the head does not enter the hole, the nail never carries the clock; the hands let go and it falls: 0."""
    yield from wait(0.3)
    read_start(c)
    c.hang_offset = (0.0, 0.005, 0.0)
    yield from take_off(c)
    yield from turn_around(c)
    yield from set_time(c)
    yield from turn_back(c)
    yield from hang(c)
    yield from wait(4.0)


def probe_takeoff(c):
    """Development: take the clock off and stand it on the table, then turn it and set the time."""
    yield from wait(0.3)
    read_start(c)
    yield from take_off(c)
    yield from turn_around(c)
    yield from set_time(c)
    yield from wait(2.0)


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in CLOCK_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["CLOCK_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "probe-takeoff": probe_takeoff,
    "neg-fixed60": neg_fixed60,
    "neg-fixed50": neg_fixed50,
    "neg-noset": neg_noset,
    "neg-miss": neg_miss,
    "replay": replay,
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
    import clockkit

    rt = clockkit.runtime()
    poses = rt.read()
    return dict(
        score=clockkit.STATE["score"],
        setup=rt.setup,
        knob_q=round(rt.dofs()["knob"], 6),
        bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
        rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()},
    )


def reset_report(stages, resets, results):
    import clockkit

    rows = []
    for i, r in enumerate(resets):
        want = clockkit.authored(
            tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"]
        )
        dev = max(
            float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000)))
            for n in ("clock", "nail")
        )
        rdev = float(np.abs(np.subtract(r["rots"]["clock"], want["clock"][1])).max())
        dev = max(dev, abs(r["knob_q"] - want["knob_q"]) * 1000)
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
    import clockkit
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
        if (out / "STOP").exists():  # operator stop (development): end the trial now
            stop = stop or "STOP file"
            break
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
        d = clockkit.STATE["detail"]
        row = dict(
            step=steps,
            score=info.get("score"),
            level=clockkit.STATE["level"],
            detail=d,
            labels=[c.left.label, c.right.label],
            nonfinite=clockkit.STATE["nonfinite"],
            cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(),
            q=np.round(jp, 4).tolist(),
            corr_mm=np.round(c.corr_p * 1000, 3).tolist(),
            corr_r=np.round(c.corr_r, 4).tolist(),
            clock_rv=np.round(_rotvec(c.pose()[1]), 5).tolist(),
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
                f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} "
                f"shown {m.get('shown_hm')} err {m.get('err_min')} tilt {m.get('tilt_deg')} flat {m.get('flat_deg')} "
                f"nail_k {m.get('nail_k_mm')} on_nail {m.get('on_nail')} nail_n {m.get('nail_n')} "
                f"robot_n {m.get('robot_n')} drop {d.get('dropped')} wall {time.time() - t1:.0f}s",
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
        scorer=dict(score=clockkit.STATE["score"], level=clockkit.STATE["level"]),
        final=clockkit.STATE["detail"],
        nonfinite=clockkit.STATE["nonfinite"],
        looks=c.log,
        wall_s=round(time.time() - t1, 1),
        wall_total_s=round(time.time() - t0, 1),
        worst_ik_mm=dict(
            left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)
        ),
        physics_dt=clockkit.PHYSICS_DT,
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
