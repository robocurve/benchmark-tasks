"""The scripted bimanual demo as phases of Cartesian fingertip targets, solved to joint targets online.

Each phase is planned when it starts, from the state of the scene then (where the lip and the top tortilla are):
a scripted demo may read the simulation to plan its moves. A target is the fingertip point (the middle of the
pads' tips, 12.8 mm past the TCP), the approach direction (from the wrist toward the tips) and the closing axis.

Phases:
 1. the left hand pinches the bottom lip (it sticks out past the top lip), the right the curled top lip, 25 mm
    either side of the middle, each coming in on a diagonal so the wrists stay apart;
 2. they pull the lips apart and the right draws its lip outward: the zip peels open;
 3. the right keeps the top lip and folds the front film back over the bag; the left lets go of the bottom lip;
 4. the left hand, fingers closed, presses its tips on the top tortilla near its edge and slides it toward the arms
    until it hangs over the board's edge, then lifts off;
 5. it pinches the hanging edge from in front;
 6. draws the tortilla out, carries it hanging over the plate's near side and lays it down flat, moving forward;
 7. lets go; the right lays the film back down and lets go; both arms go back to the start.
"""

from __future__ import annotations

import math

import numpy as np

import fastik
import layout as L

TIP = 0.0128  # fingertips past the TCP
OPEN_PINCH = 0.85  # grip command (0 open .. 1 closed) for a 14 mm gap before a pinch
OPEN_LOW = 0.925  # a 7 mm gap, for the bottom lip's exposed strip
TRAVEL = 0.985  # a 1.4 mm gap: too narrow for film to slip between, too wide to press with
OPEN_TOP = 0.893  # a 10 mm gap, for the top lip: the lower pad passes between the lips
SEEDS = [np.array(s, float) for s in (L.READY, (0, 0.8, 1.4, -0.3, 0, 0), (0.3, 1.2, 0.9, -0.8, 0.5, 1.0),
                                      (-0.3, 1.2, 0.9, -0.8, -0.5, -1.0), (0, 1.5, 1.5, -1.2, 0, 1.5),
                                      (0, 1.5, 1.5, -1.2, 0, -1.5), (0.5, 0.6, 1.2, 0.2, -1.0, 0.5),
                                      (-0.5, 0.6, 1.2, 0.2, 1.0, -0.5))]
_rng = np.random.default_rng(0)
SEEDS += [_rng.uniform(fastik.LOWER, fastik.UPPER) for _ in range(60)]


def frame(approach, closing) -> np.ndarray:
    z = np.asarray(approach, float)
    z = z / np.linalg.norm(z)
    x = np.asarray(closing, float)
    x = x - x.dot(z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def tilted(deg: float, toward=(1.0, 0.0, 0.0), arm: str = "left") -> np.ndarray:
    """Approach along ``toward`` (horizontal) tipped ``deg`` down; closing axis in the vertical plane, up-ish for the
    left hand and down-ish for the right, so each wrist camera (on the TCP's -y side) is on the arm's outer side."""
    t = np.asarray(toward, float)
    t /= np.linalg.norm(t)
    a = math.radians(deg)
    app = np.r_[t[:2] * math.cos(a), -math.sin(a)]
    clo = np.r_[t[:2] * math.sin(a), math.cos(a)]
    return frame(app, clo if arm == "left" else -clo)


def smooth(f: float) -> float:
    return f * f * (3 - 2 * f)


def slerp(r0, r1, f):
    v = fastik._rotvec(r0.T @ r1)
    n = np.linalg.norm(v)
    if n < 1e-9:
        return r0.copy()
    k = v / n
    a = f * n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return r0 @ (np.eye(3) + math.sin(a) * K + (1 - math.cos(a)) * K @ K)


class Demo:
    """Call :meth:`next` once per control step: {arm: (joint targets q (6), grip 0..1, label)} or None when done."""

    def __init__(self, sense=None):
        self.sense = sense  # a function returning {"lip": ..., "tortilla": ...} from the simulation (or None)
        self.q = {a: np.array(L.READY, float) for a in L.BASES}
        self.tip = {}
        self.rot = {}
        for a in L.BASES:
            p, r = fastik.fk(self.q[a], L.BASES[a])
            self.tip[a], self.rot[a] = p + r[:, 2] * TIP, r
        self.grip = {a: 0.0 for a in L.BASES}
        self.phases = self._phases()
        self.cur = None
        self.k = 0
        self.label = ""
        self.worst = 0.0

    # -- motion primitives ---------------------------------------------------------------------------------------------
    def _ik(self, arm, tip, rot, seed=None):
        tcp = np.asarray(tip) - rot[:, 2] * TIP
        q, pe, re = fastik.solve(tcp, rot, self.q[arm] if seed is None else seed, L.BASES[arm], iters=60)
        return q, pe, re

    def _best_ik(self, arm, tip, rot):
        best = None
        for s in [self.q[arm]] + SEEDS:
            q, pe, re = self._ik(arm, tip, rot, seed=s)
            cost = pe + 0.05 * re + 0.002 * np.abs(q - self.q[arm]).sum()
            if best is None or cost < best[0]:
                best = (cost, q, pe, re)
        return best[1]

    def move(self, label, seconds, goals, grip=None):
        """Cartesian move: goals {arm: (tip, rot)}; grip {arm: target}. Returns a step generator."""
        n = max(1, round(seconds / L.DT))
        t0 = {a: self.tip[a].copy() for a in goals}
        r0 = {a: self.rot[a].copy() for a in goals}
        g0 = dict(self.grip)
        for k in range(1, n + 1):
            f = smooth(k / n)
            for a, (tip, rot) in goals.items():
                self.tip[a] = t0[a] + (np.asarray(tip) - t0[a]) * f
                self.rot[a] = slerp(r0[a], rot, f)
                q, pe, re = self._ik(a, self.tip[a], self.rot[a])
                self.worst = max(self.worst, pe)
                self.q[a] = q
            for a, g in (grip or {}).items():
                self.grip[a] = g0[a] + (g - g0[a]) * f
            yield label

    def joint_move(self, label, seconds, goals, grip=None):
        """Joint-space move to Cartesian goals {arm: (tip, rot)} (IK with many seeds), for big reconfigurations."""
        n = max(1, round(seconds / L.DT))
        q0 = {a: self.q[a].copy() for a in goals}
        q1 = {a: self._best_ik(a, *goals[a]) for a in goals}
        g0 = dict(self.grip)
        for k in range(1, n + 1):
            f = smooth(k / n)
            for a in goals:
                self.q[a] = q0[a] + (q1[a] - q0[a]) * f
                p, r = fastik.fk(self.q[a], L.BASES[a])
                self.tip[a], self.rot[a] = p + r[:, 2] * TIP, r
            for a, g in (grip or {}).items():
                self.grip[a] = g0[a] + (g - g0[a]) * f
            yield label

    def safe_move(self, label, seconds, goals, grip=None, height=0.12):
        """Up in a straight line to ``height`` above the board, a joint move up there to above each goal, then
        straight down onto it: big repositionings that cannot sweep through the bag."""
        top = L.BOARD_TOP + height
        up = {a: (np.r_[self.tip[a][:2], max(self.tip[a][2], top)], self.rot[a]) for a in goals}
        yield from self.move(label, 0.8, up)
        above = {a: (np.r_[np.asarray(t)[:2], max(np.asarray(t)[2], top)], r) for a, (t, r) in goals.items()}
        yield from self.joint_move(label, seconds, above, grip=grip)
        yield from self.move(label, 1.0, goals)

    def let_go(self, arm, label, open_to=0.0):
        """Open wide, draw straight back out along the fingers, then lift: a released lip or film is not dragged."""
        yield from self.move(label, 0.8, {}, grip={arm: open_to})
        r = self.rot[arm]
        yield from self.move(label, 1.0, {arm: (self.tip[arm] - r[:, 2] * 0.035, r)})
        yield from self.move(label, 0.8, {arm: (self.tip[arm] + [0, 0, 0.05], r)})

    def hold(self, label, seconds, grip=None):
        yield from self.move(label, seconds, {}, grip=grip)

    # -- the demo ------------------------------------------------------------------------------------------------------
    def _sense(self):
        return self.sense() if self.sense else {}

    def _phases(self):
        B, Z = L.BOARD_TOP, L.ZIP_X
        yield from self.hold("settle", 1.0)
        # 1-2: the left hand pinches the bottom lip, the right the curled top lip, either side of the middle, each
        # coming in on a diagonal so the wrists stay apart; they pull the lips apart and the zip peels open
        yield from self._open_zip()
        # 3: the right hand keeps the top lip and folds the front film back over the bag, uncovering the top
        # tortilla's front edge; the left lets go of the bottom lip
        r_fold = self.rot["right"]
        # lift it high near the middle: the front film stands up off the stack like a tent
        tent = np.array([L.ZIP_X + 0.010, -0.030, L.BOARD_TOP + 0.115])
        yield from self.move("lift the front film", 2.5, {"right": (tent, r_fold)})
        yield from self.let_go("left", "left hand lets go of the bottom lip")
        # 4: the left hand, fingers closed, presses its tips on the top tortilla near its edge and slides it toward
        # the arms until it hangs over the board's edge
        s = self._sense()
        near = s["near_edge"]() if "near_edge" in s else np.array([L.STACK_X - L.T_R, 0.01, L.STACK_TOP])
        r_press = tilted(55, arm="left")
        p_press = np.array([near[0] + 0.030, 0.035, L.STACK_TOP])
        if "uncovered" in s:  # a spot on the top tortilla's front band with no film over it, nearest the planned one
            p_press = s["uncovered"](p_press)
        y_drag = float(p_press[1])
        door = np.array([Z - 0.030, y_drag, L.STACK_TOP + 0.010])
        # fingers held a few millimetres apart on the way in, closed only over the spot
        yield from self.safe_move("line up with the mouth", 2.5, {"left": (door, r_press)}, grip={"left": TRAVEL})
        yield from self.move("over the top tortilla", 2.0, {"left": (p_press + [0, 0, 0.015], r_press)})
        yield from self.move("close the fingers", 0.5, {}, grip={"left": 1.0})
        yield from self.move("press on the top tortilla", 1.5, {"left": (p_press + [0, 0, -0.0025], r_press)})
        drag = 0.085
        yield from self.move("slide it out", 4.0, {"left": (p_press + [-drag, 0, -0.006], r_press)})
        yield from self.move("lift off", 1.0, {"left": (p_press + [-drag - 0.01, 0, 0.04], r_press)})
        # 5: pinch the hanging edge from in front
        s = self._sense()
        edge = s["front_edge"](y_drag) if "front_edge" in s else np.array([Z - drag + 0.005, y_drag, B - 0.005])
        r_pinch = tilted(35, arm="left")
        yield from self.safe_move("line up with the hanging edge", 2.5,
                                   {"left": (edge - r_pinch[:, 2] * 0.050, r_pinch)}, grip={"left": OPEN_PINCH})
        yield from self.move("slide onto the edge", 1.5, {"left": (edge + r_pinch[:, 2] * 0.026, r_pinch)})
        yield from self.hold("steady on the edge", 0.6)
        yield from self.move("pinch the tortilla", 0.8, {}, grip={"left": 1.0})
        # 6: draw it out, carry it hanging over the plate's near side, lay it down moving forward
        e = self.tip["left"].copy()
        yield from self.move("draw it out of the bag", 3.0, {"left": (e + [-0.10, 0.06, 0.10], tilted(15, arm="left"))})
        P = L.PLATE_AT
        # the tortilla leaves the pads past their tips, so it is laid out ahead of the hand: hang it over the plate's
        # far side and lay it down while drawing the hand back toward the near side
        yield from self.move("carry it over the plate", 3.0,
                             {"left": (np.array([P[0] - 0.095, P[1], L.TOP + 0.215]), tilted(20, arm="left"))})
        yield from self.move("over the plate's far side", 2.0,
                             {"left": (np.array([P[0] + 0.090, P[1], L.TOP + 0.215]), tilted(50, arm="left"))})
        yield from self.move("lay it down", 3.5,
                             {"left": (np.array([P[0] - 0.090, P[1], L.TOP + L.PLATE_WELL_Z + 0.014]), tilted(50, arm="left"))})
        yield from self.let_go("left", "let go of the tortilla")
        # 7: the right lays the film back down and lets go; both go back
        yield from self.move("lay the film back down", 2.0,
                             {"right": (np.array([L.ZIP_X - 0.030, -0.04, L.BOARD_TOP + 0.010]), self.rot["right"])})
        yield from self.let_go("right", "right lets go of the lip")
        home = {}
        for a in L.BASES:
            p, r = fastik.fk(np.array(L.READY, float), L.BASES[a])
            home[a] = (p + r[:, 2] * TIP, r)
        yield from self.joint_move("back to the start", 3.0, home)
        yield from self.hold("rest", 1.5)

    # -- pieces of the demo ---------------------------------------------------------------------------------------------
    def _open_zip(self):
        B, Z = L.BOARD_TOP, L.ZIP_X
        yaw = 0.8
        r = {"left": tilted(30, (1, -yaw, 0), arm="left"), "right": tilted(30, (1, yaw, 0), arm="right")}
        s = self._sense()
        top = s["lip"](-0.025) if "lip" in s else np.array([Z - 0.020, -0.025, B + 0.008])
        bot = s["bottom_lip"](0.025) if "bottom_lip" in s else np.array([Z - 0.030, 0.025, B + 0.001])
        at = {"right": (top + r["right"][:, 2] * 0.008, r["right"]),
              "left": (bot + r["left"][:, 2] * 0.010 - r["left"][:, 0] * 0.004, r["left"])}
        pre = {a_: (at[a_][0] - r[a_][:, 2] * 0.045 + [0, 0, 0.010], r[a_]) for a_ in at}
        yield from self.joint_move("reach for the lips", 3.0, pre, grip={"left": OPEN_LOW, "right": OPEN_TOP})
        yield from self.move("slide onto the lips", 1.5, at)
        yield from self.hold("steady", 0.6)
        yield from self.move("pinch the lips", 0.8, {}, grip={"left": 1.0, "right": 1.0})
        apart = {"right": (at["right"][0] + [-0.012, 0, 0.035], r["right"]),
                 "left": (at["left"][0] + [-0.004, 0, -0.018], r["left"])}
        yield from self.move("pull the lips apart", 2.0, apart)
        spread = {"right": (apart["right"][0] + [-0.008, -0.035, 0.030], r["right"]),
                  "left": (apart["left"][0] + [0.0, 0.0, -0.006], r["left"])}
        yield from self.move("peel the zip open along its length", 2.5, spread)

    def next(self):
        try:
            self.label = next(self.phases)
        except StopIteration:
            return None
        return {a: (self.q[a].copy(), self.grip[a], self.label) for a in L.BASES}
