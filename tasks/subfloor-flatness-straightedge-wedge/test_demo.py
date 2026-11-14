"""Offline end-to-end check of the scripted demo against the task's own scorer (no simulator).

    python -m pytest -q test_demo.py    (yam_arena importable: the instance's, or the offline stubs)

A small kinematic world stands in for Isaac Sim. Hands follow their goals with a lag and the pushing hand gives under
load (the demo does not know by how much). A closed hand at a part's grip picks it up (the straightedge needs both
hands); a released part drops onto what is under it. A closed fist low behind the wedge's heel pushes it in +x; the
wedge stops when its thickness under the straightedge's near edge fills the gap there, and the hand then presses on
it with F = overtravel / give. Through the 1:10 slope a push over 4 N would lift the straightedge's end (a spoiled
reading), and the world records it. The demo is open loop for the board FLATNESS_CASE fixes: its waypoints are
interpolated by yam_arena's ArmPlan and every step's states go through score().
"""
import math
import os
import sys
import ast

import numpy as np
import pytest
from yam_arena.demo import LEFT_BASE_POS, RIGHT_BASE_POS, ArmPlan
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
DT = 0.05
F_LIFT = 4.0  # N of push at which the wedge would lift the straightedge's end


def module(case):
    os.environ["FLATNESS_CASE"] = str(case)
    return load(os.path.join(HERE, "task.py")).module


M0 = module(1)
CASES = sorted(M0.CASES)


def st(pos, quat=(0, 0, 0, 1.0), vel=(0, 0, 0), half=0.01):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - half, high=p + half)


class World:
    def __init__(self, m, case, give=0.004, tau=0.12):
        d, ycm = m.CASES[case]
        self.m, self.give, self.tau = m, give, tau
        self.depths = m.strip_depths_mm(d, ycm)  # this world's board, whatever the module was told
        self.cmd = {s: np.array(p, float) for s, p in m.HOME_POS.items()}
        self.tcp = {s: v.copy() for s, v in self.cmd.items()}
        self.finger = {"left": m.FINGER_OPEN, "right": m.FINGER_OPEN}
        self.se = np.array(m.SE_START, float)
        self.wedge = np.array(m.W_START, float)
        self.puck = np.array(m.PUCK_START, float)
        self.held = {"left": None, "right": None}
        self.off = {}
        self.vel = {k: np.zeros(3) for k in ("se", "wedge", "puck")}
        self.force = {"left": 0.0, "right": 0.0}
        self.peak_force = 0.0
        self.lifted = False

    def floor(self, y0, y1):
        m = self.m
        ys = np.linspace(min(y0, y1), max(y0, y1), 13)
        i = np.floor((ys - m.PANEL_Y[0]) / m.STRIP_W).astype(int)
        ok = (i >= 0) & (i < m.N_STRIPS)
        z = np.where(ok, m.FLOOR - self.depths[np.clip(i, 0, m.N_STRIPS - 1)] / 1000.0, -np.inf)
        return float(z.max())

    def pad(self, s):
        return self.cmd[s][2] - self.m.PAD_TIP_PAST_TCP

    def step(self, goals):
        m = self.m
        prev = {"se": self.se.copy(), "wedge": self.wedge.copy(), "puck": self.puck.copy()}
        for s, (pos, f) in goals.items():
            self.cmd[s] = self.cmd[s] + (np.asarray(pos, float) - self.cmd[s]) * min(DT / self.tau, 1.0)
            # grip fractions: FINGER_OPEN is negative on the real arena (-0.04695), FINGER_CLOSED 0
            closing, was = f / m.FINGER_OPEN < 0.5, self.finger[s] / m.FINGER_OPEN < 0.5
            self.finger[s] = f
            c, pz = self.cmd[s], self.pad(s)
            if closing and not was and self.held[s] is None:
                se_top = self.se[2] + m.SE_H / 2
                g = self.wedge[:2] + [m.WEDGE_GRIP, 0.0]
                if abs(c[0] - self.se[0]) < 0.012 and abs(abs(c[1] - self.se[1]) - m.SE_GRIP_Y) < 0.02 and \
                        se_top - 0.04 < pz < se_top:
                    self.held[s] = "se"
                elif np.hypot(*(c[:2] - g)) < 0.012 and pz < self.wedge[2] + 0.01:
                    self.held[s] = "wedge"
                elif np.hypot(*(c[:2] - self.puck[:2])) < 0.008 and pz < self.puck[2] + m.PUCK_H:
                    self.held[s] = "puck"
                if self.held[s]:
                    self.off[s] = getattr(self, self.held[s]) - c
            elif not closing:
                self.held[s] = None
        both = [s for s in self.held if self.held[s] == "se"]
        if len(both) == 2:
            self.se = (self.cmd["left"] + self.off["left"] + self.cmd["right"] + self.off["right"]) / 2
        elif not both and self.se[0] > m.PANEL_X[0]:
            y = self.se[1]
            self.se[2] = max(self.floor(y - m.SE_L / 2, y + m.SE_L / 2), m.FLOOR) + m.SE_H / 2 + \
                (0.001 if self.lifted else 0.0)
        for s in ("left", "right"):
            self.tcp[s] = self.cmd[s].copy()
            self.force[s] = 0.0
            if self.held[s] == "wedge":
                self.wedge = self.cmd[s] + self.off[s]
            if self.held[s] == "puck":
                self.puck = self.cmd[s] + self.off[s]
        if "wedge" not in self.held.values():
            on = m.PANEL_X[0] <= self.wedge[0] and m.PANEL_Y[0] <= self.wedge[1] <= m.PANEL_Y[1]
            base = self.floor(self.wedge[1] - m.W_W / 2, self.wedge[1] + m.W_W / 2) if on else m.TOP
            self.wedge[2] = base
            for s in ("left", "right"):
                c = self.cmd[s]
                heel = self.wedge[0] - m.TIP_X
                closed = self.finger[s] / m.FINGER_OPEN < 0.5
                if closed and self.pad(s) < base + 0.01 and abs(c[1] - self.wedge[1]) < 0.015 and \
                        c[0] + m.FIST_BACK > heel and c[0] < heel + 0.01:
                    edge = self.se[0] - m.SE_T / 2
                    gap = (self.se[2] - m.SE_H / 2) - base
                    s_max = (gap - m.W_TIP) / m.W_SLOPE if gap >= m.W_TIP else -0.0005
                    front = c[0] + m.FIST_BACK
                    want = front + m.TIP_X  # the wedge's middle with its heel at the fist
                    stop = max(self.wedge[0], edge + s_max - m.TIP_X)
                    self.wedge[0] = max(self.wedge[0], min(want, stop))
                    over = front - (self.wedge[0] - m.TIP_X)  # how far the goal is past the heel
                    if over > 0:
                        F = over / self.give if self.give > 0 else 1e3
                        self.force[s] = F
                        self.peak_force = max(self.peak_force, F)
                        self.tcp[s][0] -= over  # the hand stops at the heel; the arm gives
                        if F > F_LIFT:
                            self.lifted = True
        if "puck" not in self.held.values():
            self.puck[2] = m.TOP + m.BOARD_T
        for k in prev:
            self.vel[k] = (getattr(self, k) - prev[k]) / DT

    def objects(self):
        m = self.m
        o = {"straightedge": st(self.se, vel=self.vel["se"], half=0.03),
             "wedge": st(self.wedge, vel=self.vel["wedge"]),
             "tag_puck": st(self.puck, vel=self.vel["puck"])}
        for s in ("left", "right"):
            o[f"{s}_gripper"] = st(self.tcp[s], quat=(1.0, 0.0, 0.0, 0.0), half=0.02)
            m.kit._OPENING[f"{s}_gripper"] = self.finger[s] / m.FINGER_OPEN
        return o


def run(case, steps=3000, **kw):
    """Play the open-loop demo planned for board `case` on that board."""
    m = module(case)
    m.kit._EPISODE["count"] += 1
    m.kit._EPISODE["last"] = None
    w = World(m, case, **kw)
    plans = m.demo()
    arms = {side: ArmPlan(p, LEFT_BASE_POS if side == "left" else RIGHT_BASE_POS) for side, p in plans.items()}
    score, end = 0.0, None
    for i in range(steps):
        m.kit._EPISODE["last"] = i
        o = w.objects()
        score = m.score(o)
        if end is None and m.success(o):
            end = i
        if end is not None and i > end + 30:
            break
        tg = {side: a.target(i * DT) for side, a in arms.items()}
        w.step({side: (v[0], v[2]) for side, v in tg.items()})
    return m, w, score, end


def pocket_of(m, w):
    return [k for k, (x, y) in m.POCKETS.items() if abs(w.puck[0] - x) < 0.01 and abs(w.puck[1] - y) < 0.01]


@pytest.mark.parametrize("case", CASES)
def test_demo_scores_one_on_every_board(case):
    m, w, score, end = run(case)
    assert pocket_of(m, w) == [m.VERDICT]
    assert score == 1.0 and end is not None, score
    assert end < m.RUN_LIMIT_STEPS * 0.55, end  # the slowest board (case 5) leaves the model about twice the demo's time
    assert not w.lifted and w.peak_force < F_LIFT, w.peak_force


@pytest.mark.parametrize("give", [0.002, 0.004, 0.006])
def test_arm_give(give):
    """The push ends 4 mm past the stop: 1 to 2 N against an arm that gives 2 to 4 mm/N, under the 4 N lift."""
    m, w, score, _ = run(3, give=give)
    assert score == 1.0 and not w.lifted, (give, w.peak_force)


def test_slow_arm_lag():
    m, w, score, _ = run(2, tau=0.25)
    assert score == 1.0


def test_wedge_lies_on_the_panel():
    """Laid down and at the stop, the wedge's center of mass is over the panel (its heel may overhang the front edge)."""
    m = module(2)
    a, b = m.W_TIP, float(m.wedge_thickness(m.W_L))
    com_from_tip = m.W_L * (a + 2 * b) / (3 * (a + b))  # a trapezoid's centroid, 9.9 cm back from the tip
    x_face = m.SE_X - m.SE_T / 2
    tips = [x_face - m.TIP_BACK] + [x_face + module(c).wedge_stop_under() for c in CASES]
    for tip in tips:
        assert tip - com_from_tip > m.PANEL_X[0] + 0.008, tip


def test_demo_is_open_loop():
    """demo() takes no states and nothing in task.py reaches the simulator's poses for it: no demoloop, no readouts."""
    src = open(os.path.join(HERE, "task.py")).read()
    assert "demoloop" not in src and "READOUTS" not in src
    tree = ast.parse(src)
    fn = {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
    assert not fn["demo"].args.args
    plans = (module(3).demo(), module(3).demo())
    assert [w.pos for w in plans[0]["right"]] == [w.pos for w in plans[1]["right"]]
