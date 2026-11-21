"""The two-arm servo the aspirating demo runs on (the crux prototype's, on ourkit): each arm servoes a held object's
full pose every control step from its joint readings, with a capped integral that leads the arm's lag and sag.
Nothing here moves an object; it only works out each waypoint's TCP goal from the live states."""

from __future__ import annotations

import math

import numpy as np
import ourkit as k
from ourkit import demo as D
from yam_arena import ik, station
from yam_arena.constants import ARM_JOINTS
from yam_arena.demo import Waypoint

P = D.P
Grip = D.Grip
DT = 0.05
FEED = P.StateFeed()
SRV: dict = {}


def tcp_now(arm: str):
    """(position, rotation) of an arm's TCP now, from its joints (the same in Isaac and physcheck); None offline."""
    e = k.scene.env()
    if e is None:
        return None
    robot = e.unwrapped.scene[f"{arm}_robot"]
    names = list(robot.joint_names)
    j = k.scene._np(robot.data.joint_pos)[0]
    return ik.tcp_pose(np.array([j[names.index(n)] for n in ARM_JOINTS]), station.BASES[arm])


def expv(v) -> np.ndarray:
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    a = np.asarray(v) / th
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K


def logv(Rm) -> np.ndarray:
    c = float(np.clip((np.trace(Rm) - 1) / 2, -1, 1))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    return np.array([Rm[2, 1] - Rm[1, 2], Rm[0, 2] - Rm[2, 0], Rm[1, 0] - Rm[0, 1]]) / (2 * math.sin(th)) * th


def smooth(s):
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


class Servo:
    """One arm servoing a held object's full pose every control step. X = the TCP in the object frame (read from the
    arm's joints, low-passed, held while ``hold_x``); bias = the integrated object-pose error (the arm's lag and sag),
    capped, frozen while touching. Each ``Way`` asks for an object pose at its end; its TCP goal is re-solved each
    step of its move, then kept."""

    def __init__(self, arm, obj, xr0, xp0, gain, cap_p, cap_r):
        self.arm, self.obj, self.xr0, self.xp0 = arm, obj, xr0, xp0
        self.gain, self.cap_p, self.cap_r = gain, cap_p, cap_r
        self.ways, self.held_from, self.read_once = [], None, False
        self.reset()

    def reset(self, _st=None):
        self.xr, self.xp = self.xr0.copy(), self.xp0.copy()
        self.bp, self.br = np.zeros(3), np.zeros(3)
        self.steps, self.x_read = 0, False
        for w in self.ways:
            w.reset()

    def tcp(self, Ro, po, bias=True):
        Rt, pt = Ro @ self.xr, po + Ro @ self.xp
        if bias:
            Rt, pt = expv(self.br) @ Rt, pt + self.bp
        return Rt, pt

    def active(self, t):
        for w in self.ways:
            if w.t0 - 1e-9 <= t <= w.t1 + 1e-9:
                return w
        return None

    def update(self, states, t, hold_x=False, touching=False):
        self.steps += 1
        now = tcp_now(self.arm)
        if now is None or self.obj not in states:
            return
        tp, tr = now
        ob = states[self.obj]
        Ro, po = ob.rotation(), np.asarray(ob.pos, float)
        if self.held_from is not None and t >= self.held_from and not hold_x and not (self.read_once and self.x_read):
            xr, xp = Ro.T @ tr, Ro.T @ (tp - po)
            ok = np.linalg.norm(xp - self.xp0) < 0.025 and np.linalg.norm(logv(xr @ self.xr0.T)) < math.radians(20)
            SRV[f"{self.arm}_x_ok"] = bool(ok)
            if ok and self.read_once:
                self.xr, self.xp, self.x_read = xr, xp, True
            elif ok:
                self.xr = expv(0.3 * logv(xr @ self.xr.T)) @ self.xr
                self.xp = self.xp + 0.3 * (xp - self.xp)
        w = self.active(t)
        if w is None or not w.integrate or w.start is None or self.held_from is None or t < self.held_from or touching:
            return
        s = smooth((t - w.t0) / max(w.t1 - w.t0, 1e-6))
        (Rs, ps), (Re, pe) = w.start, w.end_pose(states)
        Rw, pw = self.tcp(expv(s * logv(Re @ Rs.T)) @ Rs, ps + s * (pe - ps), bias=False)
        self.bp = self.bp + self.gain * (pw - tp)
        self.br = self.br + self.gain * logv(Rw @ tr.T)
        n = np.linalg.norm(self.bp)
        if n > self.cap_p:
            self.bp *= self.cap_p / n
        n = np.linalg.norm(self.br)
        if n > self.cap_r:
            self.br *= self.cap_r / n


class Way:
    """One servoed move: ``fn(states) -> (R, p)`` is the object pose wanted at its end; ``nominal`` the same with the
    scene as planned (the planning fallback). ``integrate``: feed the servo's bias during this move."""

    def __init__(self, servo, fn, nominal, integrate=True):
        self.servo, self.fn, self.nominal, self.integrate = servo, fn, nominal, integrate
        self.t0 = self.t1 = None
        self.reset()

    def reset(self):
        self.goal, self.seen, self.start, self.last_end = None, -1, None, None

    def end_pose(self, states):
        if self.last_end is None or self.seen != self.servo.steps:
            self.last_end = self.fn(states)
        return self.last_end

    def resolve(self):
        sv = self.servo
        if not sv.steps or FEED.states is None:
            return sv.tcp(*self.nominal, bias=False)
        t = (sv.steps + 1) * DT
        if self.goal is not None and (self.seen == sv.steps or t > self.t1 + 1e-9):
            return self.goal
        if self.start is None:
            i = sv.ways.index(self)
            prev = sv.ways[i - 1] if i > 0 else None
            if prev is not None and prev.last_end is not None and abs(prev.t1 - self.t0) < 1e-6:
                self.start = prev.last_end
            else:
                ob = FEED.states[sv.obj]
                self.start = (ob.rotation(), np.asarray(ob.pos, float))
        self.last_end = self.fn(FEED.states)
        self.goal = sv.tcp(*self.last_end)
        self.seen = sv.steps
        return self.goal


class _WayPos:
    def __init__(self, way):
        self.way = way

    def resolve(self):
        return tuple(float(v) for v in self.way.resolve()[1])

    def __array__(self, dtype=None, copy=None):
        return np.asarray(self.resolve(), dtype=dtype or float)

    def __iter__(self):
        return iter(self.resolve())

    def __len__(self):
        return 3

    def __getitem__(self, i):
        return self.resolve()[i]


class _WayGrip(float):
    def __new__(cls, way, value=0.0):
        self = float.__new__(cls, value)
        self.way = way
        return self

    def __tiltgrip_rotation__(self):
        return self.way.resolve()[0]


def way_wp(servo, seconds, fn, nominal, finger, label, integrate=True) -> Waypoint:
    w = Way(servo, fn, nominal, integrate)
    servo.ways.append(w)
    yaw = float(Grip.from_matrix(servo.tcp(*nominal, bias=False)[0]))
    return Waypoint(seconds, _WayPos(w), finger, label, _WayGrip(w, yaw))
