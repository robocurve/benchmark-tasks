# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Assemble a rack-and-pinion parallel gripper: seat the loose finger rack so both fingers open evenly about the centre.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A grey housing (52 x 160 x 4 mm floor plate, two 12 mm outer walls, two 4 mm stop blocks) is fixed at (0.12, 0).
  A steel pinion (module 3 mm, 24 mm pitch circle, eight 3 mm pins, 12 mm tall) turns freely on a vertical shaft at
  its centre. Two white 72 mm racks (module 3, 9.42 mm tooth pitch, eight 2 mm teeth, 12 mm tall) run in the two
  channels, one each side of the pinion, along y. They move in opposite directions, so the two 6 mm finger plates (on
  a bridge over the pinion at each rack's leading end, 23 mm above the floor) open and close about the housing centre.
- The near rack (pitch line x 0.108) is already in its channel, 9.42 mm (one tooth) open. Each rack has a handle post
  (7 x 6 x 19.5 mm) over its tail tooth; a stop block at each rack's tail holds it closed.
- The far rack lies on a 4 mm feed tray at x 0.172, tail toward the left arm. Lowered into its channel (pitch line
  x 0.132) it meshes with the pinion at the pinion's present phase: with the near rack closed, only the closed position
  (and each whole tooth from it) lets its teeth drop between the pins; at any other position its teeth rest on the pins.

Goal: the far rack seated in its channel and meshed, the two fingers opening and closing evenly about the centre
(shown by opening the jaws 15 mm each), both grippers let go.
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

HX = 0.12  # housing and pinion centre, world x
R = 0.012  # pitch radius (module 3, 8 teeth)
PITCH = math.pi * 0.003  # 9.4248 mm tooth pitch
PIN_R = 0.0015
TOOTH_W = 0.002
H = 0.012  # rack and pinion height
Z0 = TOP + 0.004 + H / 2  # rack and pinion mid-height (floor plate top 4 mm above the table)
YC0 = 3 * PITCH  # near rack's closed centre is at -YC0, far rack's at +YC0 (28.27 mm)
RACK_X = 0.0  # rack frame x origin is on the pitch line

WHITE = (0.92, 0.92, 0.90)
STEEL = (0.62, 0.64, 0.68)
GREY = (0.45, 0.47, 0.50)
TRAY_GREY = (0.30, 0.32, 0.35)


def rack_shapes():
    """One rack in its own frame: origin on the pitch line at the body centre, +x toward the pinion's teeth side
    (teeth tips at +2 mm, roots at -3.75 mm), +y toward the leading (finger) end, z up from mid-height."""
    s = [lk.box((0.006, 0.072, H), (-0.00675, 0.0, 0.0))]  # back bar
    for j in range(-4, 4):  # eight teeth, gap centres at j x pitch
        s.append(lk.box((0.00575, TOOTH_W, H), (-0.000875, (j + 0.5) * PITCH, 0.0)))
    s.append(lk.box((0.006, 0.008, 0.019), (-0.00675, 0.038, 0.0035)))  # finger riser
    s.append(lk.box((0.0248, 0.008, 0.006), (0.002625, 0.038, 0.010)))  # bridge over the pinion, 11 mm above its top
    s.append(lk.box((0.010, 0.006, 0.023), (R, 0.039, 0.0185)))  # finger plate at the housing centre line
    s.append(lk.box((0.007, 0.006, 0.0195), (-0.0045, -3.5 * PITCH, 0.01625)))  # handle post bolted over the tail tooth, 0.5 mm above the pins
    return s


HANDLE_LOCAL = np.array([-0.0045, -3.5 * PITCH, 0.0185])  # grip point on a rack's centre post (rack frame)
PLATE_LOCAL = np.array([R, 0.039, 0.0185])

# -- housing and pinion ---------------------------------------------------------------------------------------------
_t = lk.Tree("gear", pos=(HX, 0.0, TOP), fixed=True, color=GREY)
_base = [lk.box((0.052, 0.160, 0.004), (0, 0, 0.002))]
for sx in (-1, 1):
    _base.append(lk.box((0.0035, 0.150, H), (sx * 0.02425, 0.0, 0.010)))  # outer walls, 0.75 mm off the racks
_base.append(lk.box((0.004, 0.004, H), (-0.01875, -(YC0 + 0.036 + 0.002 - 0.0009), 0.010)))  # stop at the near rack's tail
_base.append(lk.box((0.004, 0.004, H), (0.01875, YC0 + 0.036 + 0.002 - 0.0009, 0.010)))  # stop at the far rack's tail
_t.link("base", _base, mass=1.0, friction=(0.5, 0.4), contact_offset=0.001)
_pin = [lk.cylinder(0.007, H, (0, 0, 0))]
for k in range(8):
    a = math.radians(45.0 * k)
    _pin.append(lk.capsule(PIN_R, H - 2 * PIN_R, (R * math.cos(a), R * math.sin(a), 0.0), axis="Z"))
_t.link("pinion", _pin, mass=0.04, at=(0, 0, 0.010), color=STEEL, friction=(0.15, 0.1), contact_offset=0.001)
# the pinion's angle is minus (near rack opening) / R: the near rack starts one tooth open
_t.revolute("spin", "base", "pinion", anchor=(0, 0, 0.010), axis=(0, 0, 1), limits_deg=None, rest_deg=0.0,
            init_deg=-math.degrees(PITCH / R), stiffness=0.0, damping=0.0005, friction=0.0015)
GEAR = _t.build()

NEAR_X, FAR_X = HX - R, HX + R
FAR_START_X = HX + R + 0.040
FAR_START_DY = 0.030
START_OPEN_MM = PITCH * 1000.0

_t = lk.Tree("rackN", pos=(NEAR_X, -YC0 + PITCH, Z0), yaw_deg=0.0, color=WHITE)
_t.link("rack", rack_shapes(), mass=0.06, friction=(0.25, 0.2), contact_offset=0.001)
RACK_N = _t.build()
_t = lk.Tree("rackF", pos=(FAR_START_X, YC0 + FAR_START_DY, Z0 + 0.004), yaw_deg=180.0, color=WHITE)
_t.link("rack", rack_shapes(), mass=0.06, friction=(0.25, 0.2), contact_offset=0.001)
RACK_F = _t.build()
_t = lk.Tree("tray", pos=(HX + 0.052, 0.065, TOP), fixed=True, color=TRAY_GREY)
_t.link("tray", [lk.box((0.070, 0.115, 0.004), (0, 0, 0.002))], mass=0.3, friction=(0.5, 0.4), contact_offset=0.001)
TRAY = _t.build()

OBJECTS = [GEAR, RACK_N, RACK_F, TRAY]

# -- scoring ---------------------------------------------------------------------------------------------------------
SEAT_X_TOL, SEAT_Y_TOL = 0.002, 0.0035  # m
SEAT_Z_MAX = Z0 + 0.0015
SEAT_Z_MIN = Z0 - 0.0015
COUPLE_MM = 15.0  # opening travel in which pulling the near rack alone must have driven the far rack
FREE_FAR_M = 0.005  # no pad point of either gripper within this of the far rack while it is being driven
WINDOW = 4  # control steps of the coupling window
SEAT_TILT_DEG = 3.0
SYM_TOL_MM = 3.0  # finger-gap centre off the housing centre line (half the difference of the two openings)
OPEN_MM = 15.0
STEADY_S, RELEASE_S = 1.0, 1.0
RACK_STILL = 0.003  # m/s
CLEAR_M, TCP_STILL = 0.030, 0.005
EPS = 1e-6


def measure(states, tools) -> dict:
    n, f = states["rackN"].poses["rack"], states["rackF"].poses["rack"]
    sN = (n[1, 3] - (-YC0)) * 1000.0
    sF = (YC0 - f[1, 3]) * 1000.0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(f[2, 2])))))
    yaw = math.degrees(math.atan2(float(f[1, 0]), float(f[0, 0])))
    clear, clear_f = {}, {}
    for side, tool in tools.items():
        clear_f[side] = lk.clearance(tool, states["rackF"])
        clear[side] = min(lk.clearance(tool, states["rackN"]), lk.clearance(tool, states["rackF"]),
                          lk.clearance(tool, states["gear"]))
    return {
        "sN": float(sN), "sF": float(sF),
        "F": (float(f[0, 3]), float(f[1, 3]), float(f[2, 3])),
        "N_z": float(n[2, 3]), "N_x": float(n[0, 3]), "N_yaw": math.degrees(math.atan2(float(n[1, 0]), float(n[0, 0]))),
        "N_tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(n[2, 2]))))), "clear_F": clear_f,
        "spin": float(states["gear"].deg("spin")), "tilt": tilt, "yaw_err": abs(abs(yaw) - 180.0),
        "clear": clear, "tcp_speed": {s: float(t.speed) for s, t in tools.items()},
    }


def seated(m) -> bool:
    """The far rack lies in its channel on the floor plate, level and straight."""
    fx, fy, fz = m["F"]
    return (abs(fx - FAR_X) <= SEAT_X_TOL and SEAT_Z_MIN <= fz <= SEAT_Z_MAX
            and m["tilt"] <= SEAT_TILT_DEG and m["yaw_err"] <= 4.0 and -2.0 <= m["sF"] <= 60.0)


def near_seated(m) -> bool:
    """The near rack lies in its own channel on the floor plate, level and straight (not lifted out and set aside)."""
    return (abs(m["N_x"] - NEAR_X) <= SEAT_X_TOL and SEAT_Z_MIN <= m["N_z"] <= SEAT_Z_MAX
            and m["N_tilt"] <= SEAT_TILT_DEG and abs(m["N_yaw"]) <= 4.0 and -2.0 <= m["sN"] <= 60.0)


def offset_mm(m) -> float:
    """Where the middle of the two finger plates sits relative to the housing centre line."""
    return (m["sN"] - m["sF"]) / 2.0


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []
        self.win: list = []
        self.coupled = 0.0  # mm of opening travel in which both racks moved open together, both seated, no hand on the far rack

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace, self.win, self.coupled = None, 0, [], [], 0.0

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        prev = self.last
        self.last = m
        self.n += 1
        self.win.append(m)
        if len(self.win) > WINDOW:
            old = self.win.pop(0)
            dn, df = m["sN"] - old["sN"], m["sF"] - old["sF"]
            free = bool(m["clear_F"]) and min(m["clear_F"].values()) >= FREE_FAR_M
            if seated(m) and near_seated(m) and free:
                self.coupled += min(max(dn, 0.0), max(df, 0.0)) / (WINDOW - 1)
        if prev is not None and H.dt > 0:
            vs = max(abs(m["F"][1] - prev["F"][1]), abs((m["sN"] - prev["sN"]) * MM)) / H.dt
        else:
            vs = 1.0
        H.held("steady", vs < RACK_STILL)
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if not near_seated(m):
        return 0.0, "the near rack is not seated in its channel (lifted out, tilted or set aside)"
    if not seated(m):
        return 0.0, "the far rack is not seated in its channel"
    if rec.coupled < COUPLE_MM:
        return 0.0, f"the racks were not shown to drive each other: {rec.coupled:.1f} mm coupled opening of {COUPLE_MM:.0f}"
    if abs(offset_mm(m)) > SYM_TOL_MM:
        return 0.25, "both racks seated and coupled, but the fingers are not symmetric about the centre"
    if min(m["sN"], m["sF"]) < OPEN_MM:
        return 0.5, "seated and symmetric; the jaws are not open 15 mm each"
    ends = {"racks still 1 s": H.runs.get("steady", 0.0) >= STEADY_S - EPS,
            "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
    if all(ends.values()):
        return 1.0, "assembled, jaws open evenly, both grippers let go"
    return 0.75, "open evenly; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)


REC = Record()
LOG: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()


@lk.on_step
def _track(tick):
    m = measure(tick.states, tick.tools)
    REC.update(m)
    if REC.n % 10 == 0:
        REC.trace.append((round(tick.t, 1), round(m["sN"], 1), round(m["sF"], 1), [round(v * 1000, 1) for v in m["F"]],
                          round(m["tilt"], 1), [round(1000 * m["clear"][s]) for s in ("left", "right")], round(m["spin"], 1), round((m["N_x"] - NEAR_X) * 1000, 1), round(m["N_yaw"], 1)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
VARIANTS = ("", "nothing", "left_only", "right_only", "one_tooth", "near_out", "uncoupled")
CONTACT = 0.011  # pad contact centre past the TCP
GRIP_W = 0.006
HOVER = 0.045
APPROACH_W = 0.030  # pads open only 30 mm so the far pad clears the finger plates
SQUEEZE = 0.006


def grip_rot():
    return lk_demo.down(0.0)


import linkdemo as lk_demo  # noqa: E402


def pose_of(ctx, obj):
    return np.array(ctx.observe().pose(obj, "rack"), dtype=float)


def handle_world(A):
    return A[:3, :3] @ HANDLE_LOCAL + A[:3, 3]


def pick(ctx, arm, obj, fast=False):
    """Pinch a rack's handle post from above, pads closing along y; return the rack's pose in the TCP frame."""
    A = pose_of(ctx, obj)
    r = grip_rot()
    tcp = handle_world(A) + np.array([0.0, 0.0, CONTACT])
    yield from arm.move(tcp + [0, 0, HOVER], r, width=APPROACH_W, duration=1.0 if fast else 1.6)
    yield from arm.move(tcp, r, duration=1.0 if fast else 1.4)
    for _ in range(3):  # the arm lags its command by a few mm: correct from the seen tool point
        yield from arm.wait(0.4)
        seen = ctx.observe()
        want = handle_world(np.array(seen.pose(obj, "rack"), dtype=float)) + np.array([0.0, 0.0, CONTACT])
        err = want - np.asarray(seen.tools[arm.side].tcp, dtype=float)
        LOG.setdefault("align_" + obj, []).append([round(float(e) * 1000, 2) for e in err])
        if np.linalg.norm(err) < 0.0008:
            break
        yield from arm.move(arm.pos + err, r, duration=0.5)
    yield from arm.grip(GRIP_W - SQUEEZE, 0.8)
    yield from arm.wait(0.4)
    seen = ctx.observe()
    t = seen.tools[arm.side]
    LOG.setdefault("pick_" + obj, []).append([[round(float(x), 4) for x in t.tcp], round(float(t.opening), 4),
                                              [round(float(x), 4) for x in handle_world(pose_of(ctx, obj))],
                                              [[round(float(v), 4) for v in p] for p in t.pad_points()]])


def hold_transform(ctx, arm, obj):
    A = pose_of(ctx, obj)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = arm.rot, arm.pos
    return np.linalg.inv(T) @ A


def put_rack(ctx, arm, obj, G, A_des, seconds, corr=None):
    """Command the TCP so the held rack follows toward A_des (4x4), with an additive correction."""
    corr = np.zeros(3) if corr is None else corr
    T1 = A_des @ np.linalg.inv(G)
    p1, r1 = T1[:3, 3] + corr, T1[:3, :3]
    yield from arm.move(p1, r1, duration=seconds)


def far_goal(dy=0.0):
    A = np.eye(4)
    A[:3, :3] = lk.axis_rotation((0, 0, 1), math.pi)
    A[:3, 3] = (FAR_X, YC0 + dy, Z0)
    return A


def demo_script(ctx):
    v = VARIANT
    LOG.clear()
    LOG["variant"] = v
    yield from ctx.wait(0.5)
    if v == "nothing":
        yield from ctx.wait(6.0)
        return
    left, right = ctx.left, ctx.right
    st = {"n_closed": v == "left_only", "f_done": False, "l_ready": False, "r_ready": False}

    def pull_open(arm, sign, steps=9):
        for _ in range(steps):
            yield from arm.move(arm.pos + np.array([0.0, sign * 0.003, 0.0]), duration=0.4)

    def near():
        if v == "left_only":
            return
        if v == "near_out":  # cheat: lift the near rack out of its channel and set it down beside the housing
            yield from pick(ctx, right, "rackN")
            yield from right.move(right.pos + np.array([0.0, 0.0, 0.05]), duration=1.0)
            yield from right.move(right.pos + np.array([-0.048, 0.0116, 0.0]), duration=1.5)
            yield from right.move(right.pos + np.array([0.0, 0.0, -0.053]), duration=1.0)
            yield from right.grip(APPROACH_W, 0.5)
            yield from right.move(right.pos + np.array([0.0, 0.0, 0.08]), duration=1.0)
            st["n_closed"] = True
            return
        if v != "left_only":
            yield from pick(ctx, right, "rackN")
            N = pose_of(ctx, "rackN")
            yield from right.move(right.pos + np.array([0.0, -((N[1, 3] + YC0) + 0.0015), 0.0]), duration=2.0)
            An = pose_of(ctx, "rackN")
            LOG["N_closed_mm"] = round((An[1, 3] + YC0) * 1000, 2)
            yield from right.grip(APPROACH_W, 0.5)  # let go; the stop holds the rack closed
            st["n_closed"] = True
            yield from right.move(right.pos + np.array([0.0, 0.0, 0.07]), duration=1.0)
        if v == "uncoupled":  # cheat: each hand drags its own rack open at the same time
            while not st["l_ready"]:
                yield from right.wait(0.1)
            yield from pick(ctx, right, "rackN", fast=True)
            st["r_ready"] = True
            yield from pull_open(right, +1)
            yield from right.grip(APPROACH_W, 0.5)
            yield from right.move(right.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)
            return
        while not st["f_done"]:
            yield from right.wait(0.1)
        if v in ("", "one_tooth", "right_only"):  # open the jaws: pull the near rack out 18 mm
            yield from pick(ctx, right, "rackN", fast=True)
            LOG["pull_start"] = [round(float(right.pos[1]) * 1000, 1), round(float(ctx.observe().tools["right"].tcp[1]) * 1000, 1)]
            for k in range(9):
                yield from right.move(right.pos + np.array([0.0, 0.003, 0.0]), duration=0.4)
                LOG.setdefault("pull", []).append([round(float(right.pos[1]) * 1000, 1),
                                                   round(float(ctx.observe().tools["right"].tcp[1]) * 1000, 1),
                                                   round(float((pose_of(ctx, "rackN")[1, 3] + YC0) * 1000), 2),
                                                   round(float((YC0 - pose_of(ctx, "rackF")[1, 3]) * 1000), 2)])
            LOG["opened"] = [round(float((pose_of(ctx, "rackN")[1, 3] + YC0) * 1000), 2),
                             round(float((YC0 - pose_of(ctx, "rackF")[1, 3]) * 1000), 2)]
            yield from right.grip(APPROACH_W, 0.5)
            yield from right.move(right.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)

    def far():
        if v == "right_only":
            st["f_done"] = True
            return
        yield from pick(ctx, left, "rackF")
        G = hold_transform(ctx, left, "rackF")
        dy = {"one_tooth": -PITCH}.get(v, 0.0)
        goal = far_goal(dy)

        def servo(target, tag, tol=0.0005, tries=8):
            for _ in range(tries):
                err = target[:3] - pose_of(ctx, "rackF")[:3, 3]
                LOG.setdefault(tag, []).append([round(float(e) * 1000, 2) for e in err])
                if np.linalg.norm(err) < tol:
                    return
                yield from left.move(left.pos + err, duration=0.7)
                yield from left.wait(0.2)

        hover = goal.copy()
        hover[2, 3] += 0.020
        T1 = hover @ np.linalg.inv(G)
        yield from left.move(left.pos + np.array([0.0, 0.0, 0.05]), duration=1.0)
        while not st["n_closed"]:
            yield from left.wait(0.1)
        yield from left.move(T1[:3, 3], T1[:3, :3], duration=2.5)
        yield from servo(hover[:3, 3], "servo_hover")
        stage = goal[:3, 3] + np.array([0.0, 0.0, 0.014])  # tooth bottoms 2 mm over the pin tops
        yield from left.move(left.pos + stage - pose_of(ctx, "rackF")[:3, 3], duration=1.2)
        yield from left.wait(0.3)
        yield from servo(stage, "servo_stage", tol=0.0004)
        for dz in (0.0095, 0.005, 0.0005):  # lower in steps, holding the seen x and y on the goal
            tgt = goal[:3, 3] + np.array([0.0, 0.0, dz])
            yield from left.move(left.pos + np.array([0.0, 0.0, tgt[2] - pose_of(ctx, "rackF")[2, 3]]), duration=0.8)
            for _ in range(3):
                e = tgt - pose_of(ctx, "rackF")[:3, 3]
                LOG.setdefault("step_err", []).append([round(float(x) * 1000, 2) for x in e])
                if np.linalg.norm(e[:2]) < 0.0006:
                    break
                yield from left.move(left.pos + e * np.array([1.0, 1.0, 0.0]), duration=0.5)
        yield from left.wait(0.3)
        LOG["seat_err_mm"] = [round(float(x) * 1000, 2) for x in (pose_of(ctx, "rackF")[:3, 3] - goal[:3, 3])]
        yield from left.grip(APPROACH_W, 0.5)
        if v == "uncoupled":  # cheat: let the rack settle, take hold of it again; the right hand drags the near rack
            for _ in range(60):
                if pose_of(ctx, "rackF")[2, 3] < Z0 + 0.0008:
                    break
                yield from left.wait(0.1)
            yield from left.wait(0.3)
            yield from pick(ctx, left, "rackF", fast=True)
            st["l_ready"] = True
            while not st["r_ready"]:
                yield from left.wait(0.1)
            yield from pull_open(left, -1)
            yield from left.grip(APPROACH_W, 0.5)
            yield from left.move(left.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)
        else:
            yield from left.wait(0.5)
            yield from left.move(left.pos + np.array([0.0, 0.0, 0.05]), duration=0.6)
        if v == "near_out":  # cheat: with the near rack gone, drag the far rack open to the same travel by hand
            yield from left.move(left.pos + np.array([0.0, 0.0, -0.05]), duration=0.6)
            yield from pick(ctx, left, "rackF", fast=True)
            yield from pull_open(left, -1, 7)
            yield from left.grip(APPROACH_W, 0.5)
            yield from left.move(left.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)
        yield from left.move(left.pos + np.array([0.04, 0.09, 0.03]), duration=1.0)  # clear of the right arm
        st["f_done"] = True

    yield from ctx.both(near(), far())
    yield from ctx.wait(2.0)


def demo_report() -> dict:
    s, why = ladder(REC)
    m = REC.last or {}
    return {"score": s, "reason": why, "variant": VARIANT, "evers": dict(REC.H.evers),
            "runs": {k: round(x, 2) for k, x in REC.H.runs.items()}, "coupled_mm": round(REC.coupled, 2),
            "last": {k: (round(x, 3) if isinstance(x, float) else x) for k, x in m.items() if k not in ("clear", "tcp_speed")},
            "log": dict(LOG), "trace": REC.trace}
