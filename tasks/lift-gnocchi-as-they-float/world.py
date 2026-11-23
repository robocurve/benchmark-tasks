"""Offline test world for the gnocchi task (tests only; never imported by task.py).

A small kinematic world in place of Isaac Sim: the right hand follows the demo's plan (smoothstep between
waypoints, rotation slerped), a closed hand at the spoon's grip carries the spoon rigidly (set in the pinch by
M.HELD_OFFSET, as measured in the simulator), each gnocchi is a rigid body integrated at 240 Hz under gravity and
floatkit's water (penalty contact with the pan's floor and wall), the spoon's bowl lifts a piece that lies over it
when its floor comes up under the piece, and a piece in the bowl slides off the tip when the bowl is tipped more than
30 degrees. The left hand follows its own plan; while it is closed on the burner knob's bar, the knob turns with the
hand's yaw (and clicks into OFF within 8 degrees of it once let go). Every control step's states go through the task's
score().
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

M = load(os.path.join(HERE, "task.py")).module
import floatkit as fk  # noqa: E402
import yamkit as kit  # noqa: E402
from test_floatkit import Rigid  # noqa: E402

PHYS = 12  # physics steps per control step (240 Hz)
SLIDE_OFF = math.radians(30.0)


def quat_of(R) -> np.ndarray:
    return Rotation.from_matrix(R).as_quat()  # (x, y, z, w)


def st(pos, R=np.eye(3), vel=(0, 0, 0), spin=(0, 0, 0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat_of(R), lin_vel=np.asarray(vel, float), ang_vel=np.asarray(spin, float),
                       low=p - 0.01, high=p + 0.01)


def _smooth(s: float) -> float:
    s = min(max(s, 0.0), 1.0)
    return s * s * (3 - 2 * s)


class Player:
    """Plays a waypoint list like yam_arena's ArmPlan: smoothstep position, slerped rotation."""

    def __init__(self, plan, start_pos, start_R, start_finger):
        self.waypoints = plan
        self.p0, self.R0, self.f0 = np.asarray(start_pos, float), start_R, start_finger

    def target(self, t):
        prev_p, prev_R, prev_f = self.p0, self.R0, self.f0
        for w in self.waypoints:
            goal = np.asarray(w.pos, float)
            R = M.tilted_rot(w.yaw, getattr(w, "roll", 0.0), getattr(w, "pitch", 0.0))
            if t < w.duration:
                s = _smooth(t / w.duration)
                rot = Slerp([0, 1], Rotation.from_matrix([prev_R, R]))([s]).as_matrix()[0]
                return prev_p + s * (goal - prev_p), rot, prev_f + s * (w.finger - prev_f), w.label
            t -= w.duration
            prev_p, prev_R, prev_f = goal, R, w.finger
        return prev_p, prev_R, prev_f, "done"


class World:
    def __init__(self, seed=0, lag=0.0, drift=None, cook_override=None):
        M.setup_water(seed)
        if drift is not None:
            fk.set_drift(drift)
        if cook_override is not None:  # the world's truth differs from what the task file was told (blinding)
            for n, t in cook_override.items():
                fk.set_cook(n, fk.Cook(t, M.G_RHO_COOKED, M.COOK_RAMP))
        fk.reset()
        kit._EPISODE["count"] += 1
        kit._EPISODE["last"] = None
        self.lag = lag
        self.g = {}
        for i, n in enumerate(M.G_NAMES):
            q = Rotation.from_euler("z", M.G_YAW[i]).as_quat()
            self.g[n] = Rigid(fk._CFG["bodies"][n], M.G_START[i], q)
        self.carried = {}  # name -> local offset (pos in spoon frame, rot)
        self.landed = {}  # name -> resting pose out of the water
        self.spoon_o = np.asarray(M.SPOON_START, float)
        self.spoon_R = M.spoon_rot(M.REST_PSI, M.REST_TIP)
        self.knob = M.KNOB_START
        self.knob_grip = None  # (hand yaw, knob angle) when the left hand closed on the bar
        self.left = np.array([0.15, 0.30, M.TOP + 0.30])
        self.left_R = M.GRIP_DOWN
        self.left_f = FINGER_OPEN
        self.spoon_v = np.zeros(3)
        self.held = False
        self.hand = np.asarray(M.SPOON_START) + np.array([0.10, 0.05, 0.20])
        self.hand_R = M.tilted_rot(0.0, 0.0, 0.0)
        self.finger = FINGER_OPEN
        self.t = 0.0
        self.scores = []
        self.objs = None

    # -- one control step -----------------------------------------------------------------------------------------
    def step_left(self, goal, goal_R, finger):
        """The left hand and the burner knob (kinematic: the knob follows the hand's yaw while it grips the bar)."""
        self.left, self.left_R, self.left_f = np.asarray(goal, float), goal_R, finger
        bar = np.array([M.KNOB_C[0], M.KNOB_C[1], M.HOB_TOP + M.KNOB_TCP_UP])
        closed = finger / FINGER_OPEN < 0.5
        yaw = M.hand_angles(goal_R)[0]
        if closed and np.linalg.norm(self.left - bar) < 0.01:
            if self.knob_grip is None:
                self.knob_grip = (yaw, self.knob)
            y0, q0 = self.knob_grip
            d = (yaw - y0 + math.pi) % (2 * math.pi) - math.pi
            self.knob = min(max(q0 + d, -M.KNOB_MAX), 0.0)
        else:
            self.knob_grip = None
            if self.knob >= -M.KNOB_CLICK:
                self.knob = 0.0

    def step(self, goal, goal_R, finger):
        prev_o = self.spoon_o.copy()
        self.hand = self.hand + (np.asarray(goal) - self.hand) * (1.0 - self.lag)
        self.hand_R = goal_R
        self.finger = finger
        closed = finger / FINGER_OPEN < 0.5
        grip_world = self.spoon_o + self.spoon_R @ M.GRIP_LOCAL
        if closed and not self.held and np.linalg.norm(self.hand - grip_world) < 0.01:
            self.held = True
        if not closed:
            self.held = False
        if self.held:
            self.spoon_R = self.hand_R @ M._HAND_IN_SPOON.T
            self.spoon_o = self.hand - self.spoon_R @ M.GRIP_LOCAL + self.spoon_R @ M.HELD_OFFSET
        self.spoon_v = (self.spoon_o - prev_o) / M.DT
        # the water and the pieces in it
        spoon_q = quat_of(self.spoon_R)
        for _ in range(PHYS):
            free = {n: r for n, r in self.g.items() if n not in self.carried and n not in self.landed}
            tools = {"spoon": fk.tool_points_world("spoon", self.spoon_o, spoon_q, self.spoon_v, np.zeros(3))}
            out = fk.step({n: r.state() for n, r in free.items()}, 1 / 240, tools)
            for n, r in free.items():
                r.step(*out[n], M.VESSEL)
        self._bowl()
        self._land()
        self.t += M.DT
        return self.objects()

    def _bowl(self):
        R, o = self.spoon_R, self.spoon_o
        tip = math.asin(max(-1.0, min(1.0, (R @ np.array([0.0, 1.0, 0.0]))[2])))  # handle up = tip down
        for n, r in self.g.items():
            if n in self.landed:
                continue
            if n in self.carried:
                lp, lR = self.carried[n]
                r.p, r.q = o + R @ lp, quat_of(R @ lR)
                r.v, r.w = self.spoon_v.copy(), np.zeros(3)
                if tip > SLIDE_OFF:  # slides off the tip
                    del self.carried[n]
                    tip_pt = o + R @ np.array([0.0, -M.BOWL_L / 2 - 0.012, 0.0])
                    r.p = np.array([tip_pt[0], tip_pt[1], r.p[2]])
                continue
            q = R.T @ (r.p - o)
            if abs(q[0]) <= M.BOWL_W / 2 - 0.004 and abs(q[1]) <= M.BOWL_L / 2 - 0.004 and \
                    q[2] - M.G_SIZE[2] / 2 <= 0.002 and q[2] > -0.005 and self.spoon_v[2] >= 0:
                lR = R.T @ fk.quat_R(r.q)
                self.carried[n] = (np.array([q[0], q[1], M.G_SIZE[2] / 2]), lR)

    def _support(self, xy):
        if math.hypot(xy[0] - M.PLATE_C[0], xy[1] - M.PLATE_C[1]) <= M.PLATE_WELL_R:
            return M.PLATE_TOP
        if abs(xy[0] - M.HOB_C[0]) <= M.HOB_SIZE[0] / 2 and abs(xy[1] - M.HOB_C[1]) <= M.HOB_SIZE[1] / 2:
            return M.HOB_TOP
        return M.TOP

    def _land(self):
        for n, r in self.g.items():
            if n in self.carried or n in self.landed:
                continue
            inside = math.hypot(r.p[0] - M.PAN_C[0], r.p[1] - M.PAN_C[1]) < M.PAN_R
            if inside and r.p[2] < M.PAN_RIM_Z + 0.01:
                continue
            z = self._support(r.p[:2]) + M.G_SIZE[2] / 2
            yaw = Rotation.from_quat(r.q).as_euler("zyx")[0]
            self.landed[n] = True
            r.p, r.q, r.v, r.w = np.array([r.p[0], r.p[1], z]), Rotation.from_euler("z", yaw).as_quat(), \
                np.zeros(3), np.zeros(3)

    def objects(self):
        kit._OPENING["right_gripper"] = float(np.clip(self.finger / FINGER_OPEN, 0, 1))
        o = {n: ObjectState(pos=r.p.copy(), quat_xyzw=np.asarray(r.q, float).copy(), lin_vel=r.v.copy(),
                            ang_vel=r.w.copy(), low=r.p - 0.0125, high=r.p + 0.0125) for n, r in self.g.items()}
        o["spoon"] = st(self.spoon_o, self.spoon_R, self.spoon_v)
        o["right_gripper"] = st(self.hand, self.hand_R)
        kit._OPENING["left_gripper"] = float(np.clip(self.left_f / FINGER_OPEN, 0, 1))
        o["left_gripper"] = st(self.left, self.left_R)
        kit._JOINTS["burner/knob"] = kit.JointState(pos=float(self.knob), vel=0.0, latched=False)
        o["pan"] = st((M.PAN_C[0], M.PAN_C[1], M.HOB_TOP))
        o["plate"] = st((M.PLATE_C[0], M.PLATE_C[1], M.TOP))
        return o


def run_demo(seed=0, lag=0.0, steps=M.RUN_LIMIT_STEPS, cook_override=None):
    """Play the demo in the world; the world's cooking times can be overridden with cook_override."""
    w = World(seed=seed, lag=lag, cook_override=cook_override)
    plans = M.demo()
    plan = plans["right"]
    pl = Player(plan, w.hand, w.hand_R, FINGER_OPEN)
    left = [wp for wp in plans["left"] if not isinstance(wp, kit.Home)]
    t_left = sum(wp.duration for wp in plans["left"][:1] if isinstance(wp, kit.Home))
    pll = Player(left, w.left, w.left_R, FINGER_OPEN)
    for k in range(steps):
        goal, R, f, _ = pl.target(k * M.DT)
        if k * M.DT >= t_left:
            w.step_left(*pll.target(k * M.DT - t_left)[:3])
        objs = w.step(goal, R, f)
        kit._EPISODE["last"] = k
        w.scores.append(M.score(objs))
        w.objs = objs
    return w, plan
