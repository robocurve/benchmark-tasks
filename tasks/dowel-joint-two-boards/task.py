# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Join two loose boards end to end on three glued dowels.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475. The bench is a waxed laminate top (low friction).

- A pale birch board (18 x 80 x 150 mm, 123 g) lies at (0.36, -0.33), its long axis along y. Three 8 mm beech dowels
  are glued 20 mm out of its seam end (the +y end) at x -25, 0 and +25 mm, 9 mm above the bench.
- A dark walnut board (same size, 140 g) lies at (0.36, +0.33). Its seam end (the -y end) has three 9 mm square sockets
  22 mm deep at the same pitch and height (0.5 mm of clearance a side).
- Neither board is clamped, jigged or stopped. The seam is at y 0: the left arm stands at y +0.3, the right at y -0.3.
  Each board starts beside its own arm, 0.51 m from the other board's seam end; the pale board lies 0.73 m from the
  left arm's base and the dark board 0.73 m from the right arm's. A one-armed carry stops short: with a board in hand
  the arm cannot get its gripper past y -0.075 (left) or +0.075 (right) at x 0.36, and the seat needs y -0.135.

Goal: both boards lying flat on the bench with their seam ends touching, the three dowels in the three sockets, the
edges flush; both grippers let go.
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parent))

import linkkit as lk  # noqa: E402
import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"
SUCCESS_HOLD_S = 1.0
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001

T_B, W_B, L_B = 0.018, 0.080, 0.150  # board thickness, width, length (m)
DOWEL_R, DOWEL_LEN, DOWEL_X = 0.004, 0.020, (-0.025, 0.0, 0.025)
SOCKET, SOCKET_DEEP = 0.009, 0.022
BX = 0.36
A_Y0, B_Y0 = -0.33, 0.33  # board centres at the start

PALE, DARK, BEECH = (0.87, 0.74, 0.52), (0.36, 0.24, 0.16), (0.72, 0.50, 0.25)


def _pale():
    t = lk.Tree("pale", pos=(BX, A_Y0, TOP), color=PALE)
    shapes = [lk.box((W_B, L_B, T_B), (0.0, 0.0, T_B / 2), contact_offset=0.001)]
    for x in DOWEL_X:
        shapes.append(lk.cylinder(DOWEL_R, DOWEL_LEN - 0.002, (x, L_B / 2 + (DOWEL_LEN - 0.002) / 2, T_B / 2), axis="Y",
                                  color=BEECH, contact_offset=0.001))
        shapes.append(lk.cylinder(DOWEL_R - 0.0006, 0.002, (x, L_B / 2 + DOWEL_LEN - 0.001, T_B / 2), axis="Y", color=BEECH,
                                  contact_offset=0.001))
    t.link("board", shapes, mass=0.123, friction=(0.25, 0.25), contact_offset=0.001)
    return t.build()


def _dark():
    t = lk.Tree("dark", pos=(BX, B_Y0, TOP), color=DARK)
    sh = SOCKET / 2
    y_front, y_back = -L_B / 2, -L_B / 2 + SOCKET_DEEP
    shapes = [lk.box((W_B, L_B - SOCKET_DEEP, T_B), (0.0, (y_back + L_B / 2) / 2, T_B / 2), contact_offset=0.001)]
    edges = [-W_B / 2] + [v for x in DOWEL_X for v in (x - sh, x + sh)] + [W_B / 2]
    zl, zh = T_B / 2 - sh, T_B / 2 + sh
    ymid = (y_front + y_back) / 2
    for a, b in zip(edges[0::2], edges[1::2]):  # solid columns between the sockets
        shapes.append(lk.box((b - a, SOCKET_DEEP, T_B), ((a + b) / 2, ymid, T_B / 2), contact_offset=0.001))
    for x in DOWEL_X:  # floor and roof of each socket
        shapes.append(lk.box((SOCKET, SOCKET_DEEP, zl), (x, ymid, zl / 2), contact_offset=0.001))
        shapes.append(lk.box((SOCKET, SOCKET_DEEP, T_B - zh), (x, ymid, (zh + T_B) / 2), contact_offset=0.001))
    t.link("board", shapes, mass=0.140, friction=(0.25, 0.25), contact_offset=0.001)
    return t.build()


PALE_B, DARK_B = _pale(), _dark()
OBJECTS = [PALE_B, DARK_B]

# -- thresholds ------------------------------------------------------------------------------------------------------
TIPS_IN_M = 0.019  # seam gap under the dowel length: the tips are in the sockets
HALF_M = 0.010
SEATED_M = 0.0005
FLUSH_M = 0.001
YAW_TIPS_DEG = 2.0
CLEAR_M = 0.030
STILL_MS = 0.010
RELEASE_S = 1.0
BOARD_STILL = 0.005
EPS = 1e-6


def _inv(P):
    R = P[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ P[:3, 3]
    return out


def measure(PA, PB, tools=None, states=None) -> dict:
    """Seam measurements from the two board poses (4x4). Gaps are along the pale board's long axis."""
    PA, PB = np.asarray(PA, float), np.asarray(PB, float)
    iA = _inv(PA)
    gaps = []
    for x in (-W_B / 2, W_B / 2):
        p = iA @ (PB @ np.array([x, -L_B / 2, T_B / 2, 1.0]))
        gaps.append(float(p[1] - L_B / 2))
    rel = iA @ PB
    yaw = math.degrees(math.atan2(rel[1, 0], rel[0, 0]))
    m = {"gap_l": gaps[0], "gap_r": gaps[1], "dx": float(rel[0, 3]), "yaw": yaw,
         "tilt_a": math.degrees(math.acos(max(-1.0, min(1.0, float(PA[2, 2]))))),
         "tilt_b": math.degrees(math.acos(max(-1.0, min(1.0, float(PB[2, 2]))))),
         "pos_a": PA[:3, 3].copy(), "pos_b": PB[:3, 3].copy()}
    m["clear"], m["tcp_speed"], m["speed"] = {}, {}, 0.0
    if tools is not None and states is not None:
        for side, tool in tools.items():
            m["clear"][side] = min(lk.clearance(tool, states["pale"]), lk.clearance(tool, states["dark"]))
            m["tcp_speed"][side] = float(tool.speed)
        m["speed"] = max(states["pale"].max_speed(), states["dark"].max_speed())
    return m


class Record:
    """Whole-run record: a linkkit History fed every control step, cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.start: dict | None = None
        self.n = 0

    def reset(self) -> None:
        self.H.reset()
        self.last = self.start = None
        self.n = 0

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        if self.start is None:
            self.start = {"a": m["pos_a"].copy(), "b": m["pos_b"].copy()}
        self.last = m
        gmax, gmin = max(m["gap_l"], m["gap_r"]), min(m["gap_l"], m["gap_r"])
        ta = float(np.linalg.norm(m["pos_a"][:2] - self.start["a"][:2]))
        tb = float(np.linalg.norm(m["pos_b"][:2] - self.start["b"][:2]))
        H.trace("seam", (round(1000 * m["gap_l"], 1), round(1000 * m["gap_r"], 1), round(1000 * m["dx"], 1),
                         round(m["yaw"], 2)), every=0.5)
        H.peak("travel_a", ta)
        H.peak("travel_b", tb)
        H.ever("tips_in", gmax <= TIPS_IN_M and abs(m["dx"]) <= FLUSH_M and abs(m["yaw"]) <= YAW_TIPS_DEG)
        H.ever("half", gmax <= HALF_M and abs(m["yaw"]) <= YAW_TIPS_DEG)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < STILL_MS and m["speed"] < BOARD_STILL)


def seated(m) -> bool:
    return (-0.002 <= m["gap_l"] <= SEATED_M and -0.002 <= m["gap_r"] <= SEATED_M
            and abs(m["dx"]) <= FLUSH_M and m["tilt_a"] <= 3.0 and m["tilt_b"] <= 3.0)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if seated(m):
        if H.runs.get("released", 0.0) >= RELEASE_S - EPS:
            return 1.0, "seated, edges flush, both grippers 30 mm clear and still for 1 s"
        return 0.75, "seated at the end; the grippers are not yet 30 mm clear and still for 1 s"
    if "half" in H.evers and "tips_in" in H.evers and max(m["gap_l"], m["gap_r"]) <= HALF_M:
        return 0.5, "dowels more than half in, not seated at the end"
    if "tips_in" in H.evers:
        return 0.25, "the dowel tips entered the sockets square (within 1 mm sideways and 2 deg of yaw), not yet seated"
    return 0.0, "the dowel tips never entered the sockets square"


REC = Record()


@lk.on_reset
def _reset(tick):
    REC.reset()


@lk.on_step
def _track(tick):
    s = tick.states
    REC.update(measure(s["pale"].poses["board"], s["dark"].poses["board"], tick.tools, s))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Reads only what a camera could measure: the two boards' poses and the robot's own grippers.
VARIANTS = ("", "nothing", "left_only", "right_only", "one_end_first", "skew_3mm")
LOG: dict = {}
GRIP_OPEN, GRIP_CLOSED = 0.094, 0.0785
TCP_Z = TOP + 0.014
PRE = 0.100  # each board's centre this far from the seam before the insertion (seam faces 50 mm apart)
SEAT = L_B / 2  # centre distance from the seam when seated
PUSH = 0.0015
GRIP_OFF = 0.045  # the grip is this far from the board's centre toward its outer end


def rotz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose(x, y, deg=0.0, z=TOP):
    P = np.eye(4)
    P[:3, :3] = rotz(deg)
    P[:3, 3] = (x, y, z)
    return P


def yaw_of(P):
    return math.degrees(math.atan2(P[1, 0], P[0, 0]))


class Hand:
    """One arm carrying one board: the grasp transform is measured once, then the arm is commanded so the board's seen
    pose follows a target, with the seen error integrated into a small correction."""

    def __init__(self, ctx, side, name):
        self.ctx, self.arm, self.name = ctx, ctx.arm(side), name
        sgn = 1.0 if side == "right" else -1.0  # the wrist roll that keeps each arm inside its limits at the seam
        self.R0 = np.column_stack([[sgn, 0.0, 0.0], [0.0, -sgn, 0.0], [0.0, 0.0, -1.0]])
        self.G = np.eye(4)
        self.corr = np.zeros(2)
        self.cyaw = 0.0
        self.away = 1.0 if side == "left" else -1.0  # grip toward the board's outer end: the wrists need room at the seam
        self.log: list = []

    def seen(self):
        return self.ctx.observe().pose(self.name, "board")

    def tcp(self, D):
        T = D @ self.G
        p = T[:3, 3].copy()
        p[:2] += self.corr
        return p, rotz(self.cyaw) @ T[:3, :3]

    def pick(self):
        P = self.seen()
        at = P[:3, 3] + np.array([0.0, self.away * GRIP_OFF, 0.0])
        up = at + np.array([0.0, 0.0, TCP_Z - TOP + 0.07])
        yield from self.arm.move(up, self.R0, width=GRIP_OPEN, duration=2.5)
        down = at + np.array([0.0, 0.0, TCP_Z - TOP])
        yield from self.arm.move(down, self.R0, duration=1.2)
        yield from self.arm.grip(GRIP_CLOSED, 0.8)
        yield from self.arm.wait(0.4)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.arm.rot, self.arm.pos
        self.G = _inv(self.seen()) @ T

    def step_err(self, D, gain, clip=0.006):
        P = self.seen()
        e = D[:2, 3] - P[:2, 3]
        self.corr = np.clip(self.corr + gain * e, -clip, clip)
        self.cyaw = float(np.clip(self.cyaw + gain * (yaw_of(D) - yaw_of(P)), -6.0, 6.0))
        return 1000 * float(np.linalg.norm(e)), yaw_of(D) - yaw_of(P)

    def follow(self, Dfn, secs, width=None, gain=0.0):
        w = self.arm.width if width is None else width

        def at(s):
            D = Dfn(s)
            if gain:
                self.step_err(D, gain)
            p, r = self.tcp(D)
            return p, r, w

        yield from self.arm.follow(at, secs)

    def servo(self, Dfn, tag, tries=4, tol_mm=0.15, tol_deg=0.1):
        for k in range(tries):
            yield from self.arm.wait(0.3)
            e, a = self.step_err(Dfn(1.0), 0.8)
            self.log.append((tag, k, round(e, 2), round(a, 3)))
            if e <= tol_mm and abs(a) <= tol_deg:
                break
            p, r = self.tcp(Dfn(1.0))
            yield from self.arm.move(p, r, duration=0.5)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    hl, hr = Hand(ctx, "left", "dark"), Hand(ctx, "right", "pale")  # right arm: the pale board with the dowels
    LOG["hand_log"] = hl.log
    LOG["hand_log_r"] = hr.log
    yaw_cheat = 4.0 if variant == "one_end_first" else 0.0
    dx_cheat = 0.003 if variant == "skew_3mm" else 0.0

    def DA(off):  # pale: absolute, centre `off` before the seam
        return pose(BX, -off)

    def DB(off):  # dark: placed against the pale board as seen, centres 2 x `off` apart
        rel = np.eye(4)
        rel[:3, :3] = rotz(yaw_cheat)
        rel[:3, 3] = (dx_cheat, 2.0 * off, 0.0)
        return hr.seen() @ rel

    def release(h):
        yield from h.arm.grip(GRIP_OPEN, 0.6)
        yield from h.arm.move(h.arm.pos + np.array([0.0, 0.0, 0.07]), duration=1.2)

    def right():
        yield from hr.pick()
        yield from hr.follow(lambda s: pose(BX, A_Y0 + s * (-PRE - A_Y0)), 6.0, gain=0.15)
        yield from hr.servo(lambda s: DA(PRE), "A pre")
        LOG["right_ready"] = True
        while not LOG.get("left_ready"):
            yield from hr.arm.wait(0.1)
        yield from hr.follow(lambda s: DA(PRE - (PRE - SEAT) * s), 10.0, gain=0.1)
        yield from hr.follow(lambda s: DA(SEAT - PUSH * s), 1.0)
        yield from hr.arm.wait(1.5)
        LOG["right_done"] = True
        while not LOG.get("left_released"):
            yield from hr.arm.wait(0.1)
        yield from release(hr)

    def left():
        yield from hl.pick()
        yield from hl.follow(lambda s: pose(BX, B_Y0 + s * (PRE - B_Y0)), 6.0, gain=0.15)
        yield from hl.servo(lambda s: DB(PRE), "B pre")
        LOG["left_ready"] = True
        while not LOG.get("right_ready"):
            yield from hl.arm.wait(0.1)
        yield from hl.follow(lambda s: DB(PRE - (PRE - SEAT) * s), 10.0, gain=0.1)
        yield from hl.follow(lambda s: DB(SEAT - PUSH * s), 1.0)
        while not LOG.get("right_done"):
            yield from hl.arm.wait(0.1)
        LOG["left_released"] = True
        yield from release(hl)

    def left_alone():  # the left arm carries the dark board across the bench to the pale board's seam end and presses it on
        yield from hl.pick()
        yield from hl.follow(lambda s: pose(BX, B_Y0 + s * (A_Y0 + L_B - B_Y0)), 14.0, gain=0.1)
        yield from release(hl)

    def right_alone():  # the right arm carries the pale board across the bench to the dark board's seam end and presses it on
        yield from hr.pick()
        yield from hr.follow(lambda s: pose(BX, A_Y0 + s * (B_Y0 - L_B - A_Y0)), 14.0, gain=0.1)
        yield from release(hr)

    if variant == "left_only":
        yield from left_alone()
    elif variant == "right_only":
        yield from right_alone()
    else:
        yield from ctx.both(left(), right())
    yield from ctx.wait(3.0)


def demo_report() -> dict:
    sc, why = ladder(REC)
    H = REC.H
    m = REC.last or {}
    last = {k: (round(v, 4) if isinstance(v, float) else v) for k, v in m.items()
            if k not in ("pos_a", "pos_b", "clear", "tcp_speed")}
    if m:
        last["clear_mm"] = {s: round(1000 * v, 1) for s, v in m["clear"].items()}
    return {"score": sc, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 4) for k, v in H.peaks.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()}, "last": last,
            "log": {k: v for k, v in LOG.items()}, "trace": H.traces.get("seam", [])}
