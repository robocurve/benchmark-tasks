"""Scripted full-score demo (and negative variants) for the saw-depth task in the yam_arena embodiment.

    python demo.py task.py OUT [--max-steps N] [--no-record] [--stage demo|<variant>[,<variant>...]] [--seed S]

Acts ONLY through the 14 joint/gripper commands of ``YamArenaEmbodiment.step``. It reads the saw's link poses and
joint angles through ``sawkit.saw_state()`` (read-only, like an operator watching the blade against the board's edge
and the lever); it never writes a pose, a joint state or a score. Its joint commands are rate-limited below the CLI's
per-step delta guard and clipped to the URDF joint bounds.

Plan (both hands approach from the robot side, fingers closing up/down):
1. Left takes the rear handle's grip bar (lower finger under the bar, so the bar's load sits on the finger, not on
   friction); right pinches the depth lever's tab.
2. Left pre-loads the handle upward (joint feed-forward for the motor's weight on the handle, below the force that
   would lift the locked saw's rear); right rolls the lever up past its over-centre point: the clamp lets go and the
   left hand now carries the upper assembly. Right lets go of the lever (it stays released) and backs off.
3. Left pivots the motor about the depth pivot, closed loop on the measured blade, until the lowest tooth is 5 mm
   below the board's underside (the target follows from the board in front of the saw).
4. Right pinches the lever again and rolls it down onto its stop (locked); the clamp closes where the blade is.
5. Check: locked and the tooth 3.6 .. 6.4 mm below the underside, else release and repeat (up to 3 times).
6. Right lets go and parks its wrist camera in front of the blade (the tooth against the board's edge stays in view to
   the end); left opens, slides off the bar, backs off and goes home.
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
COMP_CLIP = 0.45
RATE = 0.8  # fraction of the CLI delta guard (0.05 x joint range per step) the script may use
JOINT_K = np.array([80.0, 80.0, 80.0, 10.0, 10.0, 10.0])  # arm joint PD stiffness (yam_arena constants)
# handle grasp (left)
BAR_HALF = L.GRIP_BAR[1] / 2
HANDLE_SQUEEZE = 0.003  # fingers 3 mm inside the bar faces: about 15 N per finger
HANDLE_OPEN_HALF = BAR_HALF + 0.012
HANDLE_RELEASE_HALF = 0.040  # wide open before backing off the bar (the lower finger stays inside the D loop)
HANDLE_TCP_V = 0.038  # TCP's v (saw frame) when gripping: pad tips 4 mm past the bar's far face
APPROACH_V = -0.075  # TCP's v before moving in
RETREAT_V = -0.140  # TCP's v after backing out (then up and home)
PRELOAD_N = 13.0  # upward pre-load on the locked handle (the locked saw's rear lifts at about 21 N)
# lever pinch (right)
TAB_HALF = L.LEVER_TAB_W / 2 + 0.002  # head of the tab
LEVER_SQUEEZE = 0.0025
LEVER_OPEN_HALF = TAB_HALF + 0.012
LEVER_TCP_V = -0.033
LEVER_PINCH_U = L.LEVER_LEN - 0.007  # along the tab from the bolt
PHI_OPEN_CMD = math.radians(62.0)
PHI_LOCK_CMD = math.radians(-3.0)
# depth
TARGET_BELOW = 0.0048  # aim: lowest tooth 4.8 mm below the board's underside (full score 3.2 .. 6.4 mm)
ACCEPT_TOL = 0.0011  # re-do if the locked blade is more than 1.1 mm off the aim
THETA_TOL = math.radians(0.04)


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
        self.ff = np.zeros(6)
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
        c = np.r_[np.clip(self.q + self.offset + self.ff, self.lo, self.hi), self.grip]
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
            self.offset = np.clip(self.offset + COMP_GAIN * (self.q - self.measured), -COMP_CLIP, COMP_CLIP)
        else:
            self.offset *= 0.85

    def feedforward(self, force_world, at_world=None):
        """Joint offsets that make the PD push with ``force_world`` (N) at the TCP (or ``at_world``): tau = J^T F,
        offset = tau / K."""
        p, _, axes = fastik.fk(self.q, self.base)
        at = p if at_world is None else np.asarray(at_world, float)
        tau = np.array([np.cross(a, at - o) @ np.asarray(force_world, float) for a, o in axes])
        return tau / JOINT_K

    def measured_tcp(self):
        return fastik.tcp_pose(self.measured, self.base)

    def solve(self, pos, rot):
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
            raise Stop(f"{self.name} unreachable at {self.label}: {pe * 1000:.1f} mm {math.degrees(re):.1f} deg")
        self.worst_mm = max(self.worst_mm, pe * 1000)
        self.q, self.pos, self.rot = q, np.asarray(pos, float), np.asarray(rot, float)

    def move(self, pos, rot, duration, grip=None, label=""):
        from yam_arena import ik

        self.label = label or self.label
        p0, r0, g0 = self.pos.copy(), self.rot.copy(), self.grip
        g1 = g0 if grip is None else grip
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.grip = g0 + s * (g1 - g0)
            self.set_pose(p0 + s * (np.asarray(pos) - p0), ik.slerp_matrix(r0, np.asarray(rot), s))
            yield

    def set_grip(self, grip, duration=0.8, label=""):
        yield from self.move(self.pos, self.rot, duration, grip, label)

    def home(self, duration=2.5, label=""):
        from yam_arena import ik
        from yam_arena.constants import READY

        self.label = label or f"{self.name} home"
        q0, q1 = self.q.copy(), np.array(READY, float)
        f0 = self.ff.copy()
        n = max(1, round(duration * CONTROL_HZ))
        for k in range(1, n + 1):
            s = ik.smoothstep(k / n)
            self.q = q0 + s * (q1 - q0)
            self.ff = f0 * (1 - s)
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
        self.corr = 0.0
        self.corr_x = 0.0
        self.tries = 0

    def state(self):
        import sawkit

        return sawkit.saw_state()

    def detail(self):
        import sawkit

        return sawkit.STATE["detail"]

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


# --------------------------------------------------------------------------------------------- grasp geometry
def handle_grasp_u(sign=1.0):
    """Left TCP pose for the grip bar in the U frame: approach +v, fingers along the bar's normal (u-w plane)."""
    g, d = L.grip_point_u()
    v = np.array([0.0, 1.0, 0.0])
    x = sign * np.cross(d, v)
    x /= np.linalg.norm(x)
    z = v
    r = np.column_stack([x, np.cross(z, x), z])
    p = g.copy()
    p[1] = HANDLE_TCP_V
    return p, r


def lever_grasp_k(sign=1.0):
    """Right TCP pose for the lever tab in the K frame: approach +v, fingers along the tab's normal (K w)."""
    x = np.array([0.0, 0.0, sign])
    z = np.array([0.0, 1.0, 0.0])
    r = np.column_stack([x, np.cross(z, x), z])
    p = np.array([LEVER_PINCH_U, LEVER_TCP_V - L.BOLT0[1], 0.0])
    return p, r


def flat_shoe(st):
    """The shoe's pose as it should be: flat on the board top, keeping its measured position and heading. Hand targets
    are computed from this, so a hand that sags sideways pushes a rocking saw back flat instead of following it."""
    sp, sr = st["shoe"]
    u = sr[:, 0]
    yaw = math.atan2(u[1], u[0]) - math.atan2(L.R_SAW[1, 0], L.R_SAW[0, 0])
    return np.array([sp[0], sp[1], L.board_top(st["spec"]["t"])]), L.rz(math.degrees(yaw)) @ L.R_SAW


def upper_world(st, theta=None):
    if theta is None:
        return st["upper"]
    sp, sr = flat_shoe(st)
    return L.upper_pose(sp, sr, theta)


def lever_world(st, phi, theta=None):
    up, ur = upper_world(st, theta)
    return L.lever_pose(up, ur, phi)


def choose_sign(arm, frame_fn, grasp_fn):
    """Finger-axis sign with the least joint-6 travel (the gripper is symmetric)."""
    best = None
    for s in (1.0, -1.0):
        gp, gr = grasp_fn(s)
        fp, fr = frame_fn()
        q, pe, re = arm.solve(fp + fr @ gp, fr @ gr)
        cost = abs(q[5] - arm.q[5]) + pe * 50 + re
        if best is None or cost < best[0]:
            best = (cost, s)
    return best[1]


def at_v(p, r, frame_rot, v):
    """Same pose moved along the saw's v axis to TCP v (saw frame) ``v`` (frame_rot: the body frame's rotation;
    the body's v axis is its second column)."""
    return p + frame_rot[:, 1] * v, r


# ------------------------------------------------------------------------------------------------ the script
def approach_handle(c):
    arm = c.left
    st = c.state()
    up, ur = st["upper"]
    c.hsign = 1.0  # wrist roll ~ +0.9..1.3 rad (the other sign sits on the joint-6 limit)
    gp, gr = handle_grasp_u(c.hsign)
    p_in = up + ur @ gp
    rot = ur @ gr
    p_out = p_in + ur[:, 1] * (APPROACH_V - HANDLE_TCP_V)
    yield from arm.move(p_out + [0, 0, 0.02], rot, 3.0, grip_of(HANDLE_OPEN_HALF), "left to handle")
    yield from arm.move(p_out, rot, 0.8, None, "left at handle")
    yield from arm.move(p_in, rot, 2.0, None, "left onto handle")


def approach_lever(c, phi=None):
    arm = c.right
    st = c.state()
    kp, kr = st["lever"] if phi is None else lever_world(st, phi)
    c.ksign = -1.0  # wrist roll +1.1..1.6 rad locked, +0.1..0.5 released: the 60 deg roll stays inside joint 6's range
    gp, gr = lever_grasp_k(c.ksign)
    p_in = kp + kr @ gp
    rot = kr @ gr
    p_out = p_in + kr[:, 1] * (APPROACH_V - LEVER_TCP_V)
    yield from arm.move(p_out, rot, 2.5, grip_of(LEVER_OPEN_HALF), "right to lever")
    yield from arm.move(p_in, rot, 1.5, None, "right onto lever")


def close_on(arm, half, squeeze, label):
    yield from arm.set_grip(grip_of(half - squeeze), 0.8, label)


def capture(c, name, body):
    """Freeze the body -> TCP transform of a hand (measured, held still)."""
    st = c.state()
    bp, br = st[body]
    pt, rt = c.arms[name].measured_tcp()
    c.G[name] = (body, br.T @ (pt - bp), br.T @ rt)


def hand_target(c, name, bp, br):
    _, gp, gr = c.G[name]
    return bp + br @ gp, br @ gr


def roll_lever(c, phi_cmd, duration, label, until=None):
    """Right hand carries the pinched lever to angle phi_cmd (rolling about the bolt), from the measured lever angle;
    the lever's frame follows the measured upper assembly."""
    from yam_arena import ik

    arm = c.right
    st = c.state()
    phi0 = st["phi"]
    n = max(1, round(duration * CONTROL_HZ))
    for k in range(1, n + 1):
        st = c.state()
        ph = phi0 + ik.smoothstep(k / n) * (phi_cmd - phi0)
        kp, kr = lever_world(st, ph, st["theta"])
        arm.label = label
        arm.set_pose(*hand_target(c, "right", kp, kr))
        yield
        if until is not None and until(c.state()):
            break


def left_follow(c, theta, label):
    """One control step: left hand at the grip for depth angle theta (+ the integral correction), on the flat-saw
    reference, plus a slow sideways correction against any rock of the shoe about its long axis (the shoe's v axis
    rising = the saw rolling toward the robot over the board's edge: push the handle away from the robot)."""
    st = c.state()
    up, ur = upper_world(st, theta + c.corr)
    roll = float(st["shoe"][1][2, 1])
    c.corr_x = float(np.clip(c.corr_x + 0.02 * roll, -0.015, 0.015))
    p, r = hand_target(c, "left", up, ur)
    c.left.label = label
    c.left.set_pose(p + np.array([c.corr_x, 0.0, 0.0]), r)


def hold_depth(c, theta_goal, seconds, label, gain=0.25, until=None, max_s=None):
    """Closed loop on the measured depth angle: the left hand's commanded angle is the goal plus an integral of the
    error (the arm sags under the motor's weight)."""
    n = round(seconds * CONTROL_HZ)
    nmax = n if max_s is None else max(n, round(max_s * CONTROL_HZ))
    for k in range(nmax):
        st = c.state()
        err = theta_goal - st["theta"]
        c.corr = float(np.clip(c.corr + gain * err, -0.35, 0.35))
        left_follow(c, theta_goal, label)
        yield
        if until is not None and k >= n and until(c.state()):
            break


def move_depth(c, theta_goal, label):
    """Left pivots the motor from the measured depth angle to theta_goal (smooth), then settles it closed-loop."""
    from yam_arena import ik

    st = c.state()
    th0 = st["theta"]
    dur = max(1.5, abs(theta_goal - th0) / math.radians(8.0))
    n = round(dur * CONTROL_HZ)
    for k in range(1, n + 1):
        th = th0 + ik.smoothstep(k / n) * (theta_goal - th0)
        st = c.state()
        err = th - st["theta"]
        c.corr = float(np.clip(c.corr + 0.15 * err, -0.35, 0.35))
        left_follow(c, th, label)
        yield
    cnt = [0]

    def settled(s):
        cnt[0] = cnt[0] + 1 if abs(s["theta"] - theta_goal) < THETA_TOL else 0
        return cnt[0] >= 8

    yield from hold_depth(c, theta_goal, 1.0, label + " (settle)", until=settled, max_s=6.0)


def aim_thickness(c):
    """The board thickness the depth is set for: the board in front of the saw (the negative neg-wrong-board aims at
    another stock instead)."""
    st = c.state()
    return st["spec"]["t"] if getattr(c, "aim_t", None) is None else c.aim_t


def goal_theta(c):
    return L.theta_of_depth(aim_thickness(c) + TARGET_BELOW)


def aim_error(c):
    """Lowest tooth below the aimed board's underside minus the aim (m): the blade read against the board's edge."""
    st = c.state()
    up, ur = st["upper"]
    return L.projection_below(up, ur) + (st["spec"]["t"] - aim_thickness(c)) - TARGET_BELOW


def proj_now(c):
    st = c.state()
    up, ur = st["upper"]
    return L.projection_below(up, ur)


def preload(c, force_n, seconds, label):
    """Ramp the left hand's upward push on the handle (joint feed-forward) while holding the grasp pose."""
    arm = c.left
    st = c.state()
    up, ur = st["upper"]
    g, _ = L.grip_point_u()
    at = up + ur @ g
    f0 = arm.ff.copy()
    f1 = arm.feedforward([0.0, 0.0, force_n], at)
    n = round(seconds * CONTROL_HZ)
    for k in range(1, n + 1):
        arm.ff = f0 + (k / n) * (f1 - f0)
        arm.label = label
        yield


def unlock(c, with_left_hold=True):
    """Right pinches and rolls the lever open; left keeps the grip pose for the current angle (closed loop)."""
    th_hold = c.state()["theta"]

    def right():
        yield from approach_lever(c)
        yield from close_on(c.right, TAB_HALF, LEVER_SQUEEZE, "right pinches lever")
        capture(c, "right", "lever")
        yield from roll_lever(c, PHI_OPEN_CMD, 1.6, "right releases lever")
        yield from wait(0.4)
        yield from c.right.set_grip(grip_of(LEVER_OPEN_HALF), 0.6, "right lets go")
        p, r = c.right.pos, c.right.rot
        st = c.state()
        yield from c.right.move(p + st["lever"][1][:, 1] * (APPROACH_V - LEVER_TCP_V), r, 1.0, None, "right off")

    def left():
        while True:
            st = c.state()
            err = th_hold - st["theta"]
            c.corr = float(np.clip(c.corr + 0.2 * err, -0.35, 0.35))
            left_follow(c, th_hold, "left holds depth")
            yield
            if c.right.label == "right off" and c.state()["phi"] > L.LEVER_PHI_MID:
                break

    yield from both(right(), left())
    c.note("released", phi_deg=math.degrees(c.state()["phi"]))


def lock(c, theta_goal):
    """Right pinches the released lever and rolls it down onto its stop while left holds the depth."""
    def right():
        yield from approach_lever(c)
        yield from close_on(c.right, TAB_HALF, LEVER_SQUEEZE, "right pinches lever")
        capture(c, "right", "lever")
        yield from roll_lever(c, PHI_LOCK_CMD, 1.6, "right locks lever")
        yield from wait(0.5)

    done = [False]

    def right_wrap():
        yield from right()
        done[0] = True

    def left():
        while not done[0]:
            st = c.state()
            err = theta_goal - st["theta"]
            c.corr = float(np.clip(c.corr + 0.2 * err, -0.35, 0.35))
            left_follow(c, theta_goal, "left holds depth")
            yield

    yield from both(right_wrap(), left())
    c.note("locked", proj_mm=proj_now(c) * 1e3, phi_deg=math.degrees(c.state()["phi"]))


def reachable_offset(arm, base_p, rot, direction, lengths):
    """The first (largest) of ``lengths`` for which base_p + direction * length is reachable with rot (URDF IK)."""
    for ln in lengths:
        q, pe, re = arm.solve(base_p + np.asarray(direction) * ln, rot)
        if pe < 0.0015 and re < math.radians(1.5):
            return ln
    return lengths[-1]


def retreat(arm, axis, v_from, label):
    """Back the hand out along the saw's v axis (as far as RETREAT_V allows, reach permitting), then up."""
    want = [RETREAT_V - v_from, -0.12 - v_from, -0.10 - v_from, APPROACH_V - v_from, -0.06 - v_from]
    ln = reachable_offset(arm, arm.pos, arm.rot, axis, [w for w in want if w < 0])
    yield from arm.move(arm.pos + axis * ln, arm.rot, 1.4, None, f"{label} off")
    up = reachable_offset(arm, arm.pos, arm.rot, [0.0, 0.0, 1.0], [0.06, 0.04, 0.02, 0.0])
    yield from arm.move(arm.pos + [0.0, 0.0, up], arm.rot, 1.0, None, f"{label} up")


def let_go_right(c, retreat_far=False):
    arm = c.right
    yield from arm.set_grip(grip_of(LEVER_OPEN_HALF), 0.6, "right lets go")
    st = c.state()
    if retreat_far:
        yield from retreat(arm, st["lever"][1][:, 1], LEVER_TCP_V, "right")
        return
    yield from arm.move(arm.pos + st["lever"][1][:, 1] * (APPROACH_V - LEVER_TCP_V), arm.rot, 1.2, None, "right off")


def let_go_left(c):
    arm = c.left
    arm.set_comp("hold")
    yield from preload(c, 0.0, 1.0, "left unloads")
    yield from arm.set_grip(grip_of(HANDLE_RELEASE_HALF), 0.8, "left lets go")
    st = c.state()
    arm.set_comp(True)
    yield from retreat(arm, upper_world(st, st["theta"])[1][:, 1], HANDLE_TCP_V, "left")


def grasp_both(c):
    def right():
        yield from approach_lever(c)

    yield from both(approach_handle(c), right())
    yield from both(close_on(c.left, BAR_HALF, HANDLE_SQUEEZE, "left grips handle"),
                    close_on(c.right, TAB_HALF, LEVER_SQUEEZE, "right pinches lever"))
    yield from wait(0.4)
    capture(c, "left", "upper")
    capture(c, "right", "lever")
    c.note("grasped")


def demo(c, skip_lock=False):
    yield from wait(0.5)
    yield from grasp_both(c)
    c.left.set_comp(True)
    yield from preload(c, PRELOAD_N, 1.0, "left pre-loads handle")
    st0 = c.state()
    c.note("preloaded", theta_deg=math.degrees(st0["theta"]))
    th_hold = st0["theta"]
    # release (right already pinches)

    def right_release():
        yield from roll_lever(c, PHI_OPEN_CMD, 1.6, "right releases lever")
        yield from wait(0.3)
        yield from let_go_right(c)

    def left_hold():
        for _ in range(round(3.5 * CONTROL_HZ)):
            st = c.state()
            c.corr = float(np.clip(c.corr + 0.2 * (th_hold - st["theta"]), -0.35, 0.35))
            left_follow(c, th_hold, "left holds depth")
            yield

    yield from both(right_release(), left_hold())
    c.note("released", phi_deg=math.degrees(c.state()["phi"]))
    # full load on the handle: feed-forward for the motor's weight (model), integral for the rest
    from scoring import tilt_deg  # noqa: F401

    th_goal = goal_theta(c)
    load = L.M_UPPER * 9.81 * (-L.upper_com_u()[0]) / (-L.grip_point_u()[0][0])
    yield from both(preload(c, load, 0.6, "left takes the motor's weight"), wait(0.1))
    for attempt in range(3):
        c.tries = attempt + 1
        yield from move_depth(c, th_goal, f"left sets depth (try {attempt + 1})")
        c.note("depth set", proj_mm=proj_now(c) * 1e3, theta_goal_deg=math.degrees(th_goal))
        if skip_lock:
            break
        yield from lock(c, th_goal)
        st = c.state()
        err = aim_error(c)
        if st["phi"] < L.LEVER_PHI_ENGAGE and abs(err) <= ACCEPT_TOL:
            break
        c.note("redo", proj_mm=proj_now(c) * 1e3, aim_err_mm=err * 1e3)
        # aim off by the shift the lock caused this time
        th_goal = goal_theta(c) + (th_goal - st["theta"])
        yield from roll_lever(c, PHI_OPEN_CMD, 1.6, "right releases lever")
        yield from wait(0.3)
        yield from let_go_right(c)
    yield from let_go_right(c)
    # the right wrist camera looks at the lowest tooth against the board's edge for the rest of the trial
    yield from park_camera(c)
    yield from let_go_left(c)
    c.note("hands off", proj_mm=proj_now(c) * 1e3)
    yield from c.left.home(2.5)
    yield from wait(4.0)


def neg_unlocked(c):
    """Negative: depth set correctly, lever never locked, hands off: the motor drops to full depth (0)."""
    yield from demo(c, skip_lock=True)


def neg_wrong_board(c):
    """Negative: depth set for the wrong stock (the next thinner board in the list, or the 2x6 if the board is the
    thinnest): a fixed-depth habit instead of reading the board (0)."""
    st = c.state()
    b = st["spec"]["board"]
    c.aim_t = L.BOARDS[b - 1][1] if b > 0 else L.BOARDS[3][1]
    yield from demo(c)


def neg_no_support(c):
    """Negative: the right hand releases the lever with nobody holding the handle (the motor drops to full depth),
    then locks it again and backs off (0)."""
    yield from wait(0.5)
    yield from approach_lever(c)
    yield from close_on(c.right, TAB_HALF, LEVER_SQUEEZE, "right pinches lever")
    capture(c, "right", "lever")
    yield from roll_lever(c, PHI_OPEN_CMD, 1.6, "right releases lever")
    yield from wait(1.5)
    c.note("dropped", theta_deg=math.degrees(c.state()["theta"]))
    yield from c.right.set_grip(grip_of(LEVER_OPEN_HALF), 0.6, "right lets go")
    st = c.state()
    p, r = c.right.pos, c.right.rot
    yield from c.right.move(p + st["lever"][1][:, 1] * (APPROACH_V - LEVER_TCP_V), r, 1.0, None, "right off")
    yield from approach_lever(c)
    yield from close_on(c.right, TAB_HALF, LEVER_SQUEEZE, "right pinches lever")
    capture(c, "right", "lever")
    yield from roll_lever(c, PHI_LOCK_CMD, 1.6, "right locks lever")
    yield from wait(0.5)
    yield from let_go_right(c)
    yield from c.right.home(2.5)
    yield from wait(3.0)


def look_pose(c):
    """Right TCP pose that parks the wrist camera about 17 cm in front of the blade, at the board's underside height,
    looking along +x at the lowest tooth against the board's edge face (fingers horizontal: upright image)."""
    st = c.state()
    up, ur = st["upper"]
    low = L.blade_low_point(up, ur)
    x = np.array([0.0, -1.0, 0.0])
    z = np.array([1.0, 0.0, 0.0])
    r = np.column_stack([x, np.cross(z, x), z])
    return np.array([low[0] - 0.17, low[1], L.UNDERSIDE_Z + 0.03]), r


def park_camera(c, label="right camera at the blade"):
    """Right hand to the look pose (from wherever it is: first back out and down in front of the saw)."""
    p, r = look_pose(c)
    arm = c.right
    mid = np.array([p[0], arm.pos[1] * 0.5 + p[1] * 0.5, max(arm.pos[2], p[2] + 0.06)])
    yield from arm.move(mid, r, 2.0, 0.0, label)
    yield from arm.move(p, r, 1.5, None, label)


def look(c):
    """Media only: the right wrist camera parked at the blade at the start state (the cue a carpenter sights)."""
    yield from wait(0.5)
    yield from park_camera(c)
    yield from wait(1.5)


def idle(c):
    for _ in range(100000):
        yield


def hold_still(c):
    """Development probes: both arms at READY for 8.5 s (+2 s after the end)."""
    for _ in range(170):
        yield


def replay(c):
    """Open-loop check: plays the 14-D commands recorded in a finished run's trace.jsonl (path in SAW_REPLAY), step
    by step, without reading anything: what a fixed trajectory scores on another start state."""
    import os

    rows = [json.loads(line) for line in open(os.environ["SAW_REPLAY"])]
    c.replay = [r["cmd"] for r in rows]
    for _ in range(len(c.replay) + 40):
        yield


STAGES = {
    "demo": demo,
    "idle": idle,
    "hold-still": hold_still,
    "replay": replay,
    "look": look,
    "neg-unlocked": neg_unlocked,
    "neg-wrong-board": neg_wrong_board,
    "neg-no-support": neg_no_support,
}


def _write_mp4(frames, path):
    import imageio

    w = imageio.get_writer(str(path), fps=CONTROL_HZ, codec="libx264", quality=8, macro_block_size=8,
                           ffmpeg_params=["-pix_fmt", "yuv420p"])
    for f in frames:
        w.append_data(np.asarray(f))
    w.close()


def run(task_path, out_dir, max_steps=2400, record=True, stage="demo", cam_width=320, cam_height=176, device="cpu",
        seed=None):
    from yam_arena.inspect_embodiment import yam_arena_embodiment

    t0 = time.time()
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    emb = yam_arena_embodiment(task=str(Path(task_path).resolve()), object_jitter=0, control_hz=CONTROL_HZ,
                               cam_width=cam_width, cam_height=cam_height, device=device)
    stages = stage.split(",")
    results, resets = [], []
    seeds = [None] if seed is None else [int(x) for x in str(seed).split(",")]
    for i, st in enumerate(stages):
        tdir = out if len(stages) == 1 else out / f"trial{i}_{st}"
        res, reset_state = run_trial(emb, tdir, max_steps, record, st, t0, seed=seeds[min(i, len(seeds) - 1)])
        results.append(res)
        resets.append(reset_state)
    if len(stages) > 1:
        rep = reset_report(stages, resets, results)
        (out / "reset_check.json").write_text(json.dumps(rep, indent=2, default=float))
        print("RESETCHECK", json.dumps(rep["summary"]), flush=True)
    return results


def reset_state_now():
    import sawkit

    rt = sawkit.runtime()
    poses = rt.read()
    th, ph = rt.dofs()
    return dict(score=sawkit.STATE["score"], setup=rt.setup, theta=th, phi=ph,
                bodies={n: np.round(p * 1000, 4).tolist() for n, (p, r) in poses.items()},
                rots={n: np.round(r, 6).tolist() for n, (p, r) in poses.items()})


def reset_report(stages, resets, results):
    import sawkit

    rows = []
    for i, r in enumerate(resets):
        want = sawkit.authored(tuple(r["setup"]) if isinstance(r["setup"], list) else r["setup"])
        dev = max(float(np.linalg.norm(np.subtract(r["bodies"][n], want[n][0] * 1000))) for n in ("shoe", "upper"))
        rows.append(dict(trial=i, stage=stages[i], setup=r["setup"], score_after_reset=r["score"],
                         max_body_dev_from_authored_mm=round(dev, 4),
                         theta_dev_deg=round(math.degrees(r["theta"] - want["theta"]), 4),
                         phi_deg=round(math.degrees(r["phi"]), 4), trial_score=results[i]["task_score"],
                         trial_stop=results[i]["stop"]))
    ok = all(x["score_after_reset"] == 0.0 and x["max_body_dev_from_authored_mm"] < 1.0 for x in rows)
    return dict(summary=dict(passed=ok, trials=len(rows),
                             max_body_dev_mm=max(x["max_body_dev_from_authored_mm"] for x in rows)),
                rows=rows, reset_states=resets)


def run_trial(emb, out, max_steps, record, stage, t0, seed=None):
    import sawkit
    from inspect_robots import Scene
    from inspect_robots_yam.packing import STATE_KEY

    out.mkdir(parents=True, exist_ok=True)
    t1 = time.time()
    obs = emb.reset(Scene(id="demo", instruction="demo"), seed=seed)
    print(f"BENCH reset {time.time() - t1:.0f} s", flush=True)
    reset_state = reset_state_now()
    print("START", json.dumps(sawkit.runtime().spec, default=float), flush=True)
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
        d = sawkit.STATE["detail"]
        st = c.state()
        row = dict(step=steps, score=info.get("score"), level=sawkit.STATE["level"], detail=d,
                   labels=[c.left.label, c.right.label], nonfinite=sawkit.STATE["nonfinite"],
                   cmd=np.round(np.r_[c.left.last_cmd, c.right.last_cmd], 4).tolist(), q=np.round(jp, 4).tolist(),
                   theta_deg=round(math.degrees(st["theta"]), 4), phi_deg=round(math.degrees(st["phi"]), 3),
                   corr_deg=round(math.degrees(c.corr), 4),
                   tcp_mm={n: np.round(a.measured_tcp()[0] * 1000, 2).tolist() for n, a in c.arms.items()},
                   tcp_cmd_mm={n: np.round(a.pos * 1000, 2).tolist() for n, a in c.arms.items()},
                   offs={n: np.round(a.offset, 3).tolist() for n, a in c.arms.items()})
        tracef.write(json.dumps(row, default=float) + "\n")
        if steps % 20 == 0:
            tracef.flush()
        if steps % 25 == 0:
            m = d.get("measure", {})
            print(f"step {steps} {c.left.label} | {c.right.label} score {info.get('score')} "
                  f"proj {m.get('proj_mm')} theta {m.get('theta_deg')} phi {m.get('phi_deg')} lock {m.get('locked')} "
                  f"flat {m.get('flat')} tilt {m.get('shoe_tilt_deg')} clear {m.get('clear_mm')} "
                  f"grip {d.get('grip_n')} veto {d.get('veto')} wall {time.time() - t1:.0f}s", flush=True)
        if res.terminated:
            success = res.termination_reason == "success"
            break
        if ended is not None and steps - ended >= 2 * CONTROL_HZ:
            break
    score = float(info.get("score", 0.0) or 0.0)
    result = dict(stage=stage, task_score=score, success=success, steps=steps, stop=stop,
                  termination=getattr(res, "termination_reason", None), setup=reset_state["setup"],
                  start=sawkit.runtime().spec, scorer=dict(score=sawkit.STATE["score"], level=sawkit.STATE["level"]),
                  final=sawkit.STATE["detail"], nonfinite=sawkit.STATE["nonfinite"], looks=c.log, tries=c.tries,
                  wall_s=round(time.time() - t1, 1), wall_total_s=round(time.time() - t0, 1),
                  worst_ik_mm=dict(left=round(c.left.worst_mm, 2), right=round(c.right.worst_mm, 2)),
                  physics_dt=sawkit.PHYSICS_DT, control_hz=CONTROL_HZ)
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
            imageio.imwrite(out / f"setup_{cam}.png", fs[0])
    print("RESULT", json.dumps({k: result[k] for k in ("stage", "task_score", "success", "steps", "stop", "setup")}),
          flush=True)
    return result, reset_state


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("task")
    ap.add_argument("out")
    ap.add_argument("--max-steps", type=int, default=2400)
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--stage", default="demo")
    ap.add_argument("--cam-size", default="320x176", help="camera WxH (evidence runs use the default)")
    ap.add_argument("--device", default="cpu", help="PhysX device (cpu, or cuda:0); rendering is on the GPU either way")
    ap.add_argument("--seed", default=None,
                    help="reset seed (seeded start state, as in inspect-robots runs); a comma list gives one per trial")
    a = ap.parse_args()
    cw, ch = (int(v) for v in a.cam_size.split("x"))
    run(a.task, a.out, a.max_steps, not a.no_record, a.stage, cw, ch, a.device, a.seed)
