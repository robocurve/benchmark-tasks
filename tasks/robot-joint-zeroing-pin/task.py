# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Zero an unpowered robot arm joint: hold the link on its zero mark and push the calibration pin home.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475 (TOP). The joint assembly sits on the table turned YAW deg about the vertical:
in its own frame x runs along the hinge line and pin axis, y along the link (zero = horizontal), z up.

- Base: a 6061 aluminium joint housing, bolted down (fixed), with a hinge 90 mm above the table and a pin guide block
  45 mm out along the link, behind the link.
- Link: a 10 x 20 x 160 mm aluminium arm (150 g with its 28 mm dia, 34 mm long actuator can on the end) hanging on a
  free hinge (motors off, friction 0.004 N m) that swings from -30 deg (resting on its stop, tip 25 mm above the
  table) up to +25 deg. Gravity torque at zero is about 0.12 N m. A 7 x 7 mm calibration window goes through the arm
  45 mm from the hinge; engraved zero marks sit on the arm and the housing.
- Pin: a 6 mm dia, 40 mm long steel calibration pin with a 16 mm dia knob in the guide, sprung back (0.8 N at full
  travel). Its tip waits 15 mm before the arm's near face; it can travel 28 mm. It passes the window only when the
  arm is at zero within about +-0.6 deg (0.5 mm of play each way at 45 mm). With the arm anywhere else the tip meets
  the arm or passes beside it. A pin pushed to 26 mm is held by an O-ring detent (the brake here).

Goal: the arm held on its zero mark, the pin pushed home through the window, both grippers let go.
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
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001

O_XY = (0.240, 0.000)  # hinge line, world
YAW = 75.0  # deg; the pin axis points along (cos, sin) toward the left-forward
ZREL = 0.090  # hinge height above the table
Y_PIN = 0.045  # window distance from the hinge along the link
LINK_LEN = 0.160
TH_MIN, TH_MAX = -30.0, 25.0
PIN_X0 = -0.020  # pin tip start (joint frame), 15 mm before the arm's near face at -0.005
PIN_TRAVEL = 0.028
PIN_THROUGH = 0.018  # tip 3 mm into the arm's thickness
PIN_HOME = 0.026  # tip 1 mm past the arm's far face; the O-ring detent holds
KNOB_REAR = -0.068  # knob rear face, joint frame x
ZERO_TOL = 1.0  # deg
STILL_V, STILL_W = 0.002, 2.0  # m/s, deg/s
PINCH_Y = 0.145  # the actuator can's centre: the hand grips its flat ends, clear of the pin guide


# ---- joint assembly ---------------------------------------------------------------------------------------------
t = lk.Tree("joint", pos=(O_XY[0], O_XY[1], TOP), yaw_deg=YAW, fixed=True, color=(0.30, 0.31, 0.34))
t.link("base", [
    lk.box((0.050, 0.024, ZREL + 0.020), (0.0, -0.038, (ZREL + 0.020) / 2), color=(0.30, 0.31, 0.34)),  # housing
    lk.box((0.120, 0.150, 0.008), (0.0, 0.0, 0.004), color=(0.20, 0.21, 0.23)),  # base plate
    lk.box((0.020, 0.035, ZREL - 0.012), (-0.021, Y_PIN, (ZREL - 0.012) / 2), color=(0.30, 0.31, 0.34)),  # guide post
    lk.box((0.020, 0.010, 0.024), (-0.021, Y_PIN - 0.0205, ZREL), color=(0.30, 0.31, 0.34)),  # guide cheeks
    lk.box((0.020, 0.010, 0.024), (-0.021, Y_PIN + 0.0205, ZREL), color=(0.30, 0.31, 0.34)),
    lk.box((0.0004, 0.004, 0.0004), (0.0252, -0.026, ZREL), color=(0.97, 0.97, 0.97)),  # housing zero mark
], mass=1.2, at=(0.0, 0.0, 0.0))

hw = 0.0035  # half window
arm = [
    lk.box((0.010, Y_PIN - hw + 0.020, 0.020), (0.0, (Y_PIN - hw - 0.020) / 2, 0.0), color=(0.74, 0.75, 0.78)),
    lk.box((0.010, LINK_LEN - Y_PIN - hw, 0.020), (0.0, (Y_PIN + hw + LINK_LEN) / 2, 0.0), color=(0.74, 0.75, 0.78)),
    lk.box((0.010, 2 * hw, 0.010 - hw), (0.0, Y_PIN, (hw + 0.010) / 2), color=(0.74, 0.75, 0.78)),
    lk.box((0.010, 2 * hw, 0.010 - hw), (0.0, Y_PIN, -(hw + 0.010) / 2), color=(0.74, 0.75, 0.78)),
    lk.cylinder(0.014, 0.034, (0.0, 0.145, 0.0), axis="X", color=(0.08, 0.08, 0.09)),  # actuator can
    lk.box((0.0004, 0.004, 0.0004), (0.0052, 0.0, 0.0), color=(0.97, 0.97, 0.97)),  # arm zero mark
]
t.link("arm", arm, mass=float(os.environ.get("MASS", "0.150")), at=(0.0, 0.0, ZREL), friction=(0.9, 0.8), contact_offset=0.0005)
t.link("plunger", [
    lk.cylinder(0.003, 0.040, (-0.020, 0.0, 0.0), axis="X", color=(0.80, 0.80, 0.82)),
    lk.cylinder(0.008, 0.008, (-0.044, 0.0, 0.0), axis="X", color=(0.85, 0.15, 0.12)),
], mass=0.013, at=(PIN_X0, Y_PIN, ZREL), friction=(0.3, 0.25), contact_offset=0.0005)
t.revolute("zero", "base", "arm", anchor=(0.0, 0.0, ZREL), axis=(1, 0, 0), limits_deg=(TH_MIN, TH_MAX), rest_deg=0,
           init_deg=TH_MIN, stiffness=0.0, damping=0.01, friction=0.004)
t.prismatic("pin", "base", "plunger", anchor=(PIN_X0, Y_PIN, ZREL), axis=(1, 0, 0), limits=(0.0, PIN_TRAVEL), rest=0.0,
            stiffness=30.0, damping=1.0, brake=lk.Brake(stiffness=4000.0, damping=30.0))
JOINT = t.build()

OBJECTS = [JOINT]


# ---- measurement ----------------------------------------------------------------------------------------------


def measure(s) -> dict:
    """th = arm angle above the zero mark (deg), qp = pin travel (mm)."""
    return {"th": float(s.deg("zero")), "qp": 1000 * float(s.q["pin"])}


def at_zero(m: dict) -> bool:
    return abs(m["th"]) <= ZERO_TOL


def record(H: lk.History, m: dict) -> None:
    """One control step of the whole-run record. m = measure() plus a_speed (m/s), a_spin (deg/s), clear and
    tcp_speed per side."""
    H.set("last", m)
    H.ever("through", m["qp"] >= 1000 * PIN_THROUGH and at_zero(m))
    H.ever("home", m["qp"] >= 1000 * PIN_HOME and at_zero(m))
    H.held("still", m["a_speed"] < STILL_V and m["a_spin"] < STILL_W)
    H.held("release", all(m["clear"][k] >= 0.030 and m["tcp_speed"][k] < 0.005 for k in ("left", "right")))


def ladder(H: lk.History) -> tuple[float, dict]:
    v, m = H.values, H.values.get("last")
    if m is None:
        return 0.0, {}
    end = m["qp"] >= 1000 * PIN_HOME and at_zero(m)
    info = {"through": "through" in H.evers, "home_ever": "home" in H.evers, "end": end,
            "still_s": round(H.runs.get("still", 0), 2), "release_s": round(H.runs.get("release", 0), 2)}
    if end and H.runs.get("still", 0) >= 1.0 and H.runs.get("release", 0) >= 1.0:
        return 1.0, info
    if end:
        return 0.75, info
    if "home" in H.evers:
        return 0.5, info
    if "through" in H.evers:
        return 0.25, info
    return 0.0, info


# ---- on_step ----------------------------------------------------------------------------------------------------

REC = lk.History()
LOG: dict = {}
ST = {"prev": None, "lock": False}


@lk.on_reset
def _reset(tick):
    ST["prev"] = None
    ST["lock"] = False
    LOG.clear()


@lk.on_step
def track(tick):
    s = tick.states["joint"]
    m = measure(s)
    A = s.poses["arm"]
    prev, dt = ST["prev"], max(REC.dt, 1e-3)
    tip = A[:3, 3] + A[:3, :3] @ np.array([0.0, LINK_LEN, 0.0])
    if prev is None:
        m["a_speed"], m["a_spin"] = 0.0, 0.0
    else:
        m["a_speed"] = float(np.linalg.norm(tip - prev[1])) / dt
        c = float(np.clip((np.trace(prev[0][:3, :3].T @ A[:3, :3]) - 1) / 2, -1, 1))
        m["a_spin"] = math.degrees(math.acos(c)) / dt
    ST["prev"] = (A.copy(), tip)
    tools = tick.tools
    m["clear"] = {k: lk.clearance(tools[k], s) for k in tools}
    m["tcp_speed"] = {k: tools[k].speed for k in tools}
    record(REC, m)
    if m["qp"] >= 1000 * PIN_HOME and not lk.braked("joint", "pin"):
        lk.brake("joint", "pin", True)  # the O-ring detent holds the pin home
    tl = tools["left"]
    pp = s.point("arm", (0.0, PINCH_Y, 0.0))
    REC.trace("d", tuple(round(1000 * float(x), 1) for x in (tl.tcp - pp)) + (round(1000 * tl.opening, 1),), every=0.5)
    REC.trace("m", (round(m["th"], 2), round(m["qp"], 1), round(m["a_spin"], 1)), every=0.5)


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


# ---- demo -------------------------------------------------------------------------------------------------------


def demo():
    return {}


def demo_report() -> dict:
    sc, info = ladder(REC)
    return {"score": sc, "info": info, "values": {k: v for k, v in REC.values.items() if k != "last"},
            "last": {k: (round(v, 3) if isinstance(v, float) else v) for k, v in REC.values.get("last", {}).items()
                     if k in ("th", "qp")},
            "trace": REC.traces.get("m", [])[-60:], "d": REC.traces.get("d", []), "log": LOG}


def _angle(ctx) -> float:
    return ctx.observe().angle_deg("joint", "zero")


def left_hold(ctx, target=0.0):
    """Pinch the arm near its middle and swing it up to the zero mark, then trim from the seen angle."""
    seen = ctx.observe()
    start = seen.angle_deg("joint", "zero")
    yield from ctx.pinch_pivot("left", "joint", "zero", target - start, local=(0.0, PINCH_Y, 0.0), width=0.034,
                               squeeze=float(os.environ.get("SQUEEZE", "0.0030")), follow=1.0, pivot_s=5.0)
    arm = ctx.left
    LOG["cmd_mm"] = [round(1000 * float(x), 1) for x in arm.pos]
    LOG["tcp_mm"] = [round(1000 * float(x), 1) for x in ctx.observe().tools["left"].tcp]
    LOG["pp_mm"] = [round(1000 * float(x), 1) for x in ctx.observe().point("joint", "arm", (0.0, PINCH_Y, 0.0))]
    for i in range(4):
        yield from arm.wait(0.4)
        err = target - _angle(ctx)
        LOG.setdefault("trim", []).append(round(err, 3))
        if abs(err) < 0.15 or abs(err) > 10:
            break
        hinge, axis = ctx.observe().axis("joint", "zero")
        yield from arm.arc(hinge, axis, err, point=(0.0, 0.0, arm.pivot["depth"]), follow=arm.pivot["follow"],
                           duration=max(0.8, abs(err) / 4.0))
    yield from arm.wait(0.5)
    LOG["held_deg"] = round(_angle(ctx), 3)


def right_push(ctx, depth=0.028, tilt=45.0):
    """Pinch the knob across its diameter with the hand pitched down 45 deg, push it along the pin axis, let go."""
    from linkdemo import grip_frame
    from yam_arena.constants import PAD_TIP_PAST_TCP

    arm = ctx.right
    seen = ctx.observe()
    B = seen.pose("joint", "base")[:3, :3]
    a, ly = B @ np.array([1.0, 0.0, 0.0]), B @ np.array([0.0, 1.0, 0.0])
    ap = math.cos(math.radians(tilt)) * a + np.array([0.0, 0.0, -math.sin(math.radians(tilt))])
    knob = seen.point("joint", "plunger", (-0.044, 0.0, 0.0))
    r = grip_frame(ap, ly)
    tcp0 = knob - ap * (PAD_TIP_PAST_TCP - 0.002)
    yield from arm.move(tcp0 - ap * 0.07, r, width=0.050, duration=3.0)
    yield from arm.move(tcp0, r, duration=1.5)
    yield from arm.grip(0.016 - float(os.environ.get("KSQ", "0.001")), 0.8)
    yield from arm.wait(0.3)
    yield from arm.move(tcp0 + a * depth, r, duration=2.5)
    yield from arm.wait(0.6)
    LOG["pin_mm"] = round(1000 * ctx.observe().angle_deg("joint", "pin"), 2)
    yield from arm.open(0.6)
    yield from arm.move(arm.pos - ap * 0.07, r, duration=1.5)


def demo_script(ctx):
    v = VARIANT
    LOG["variant"] = v
    yield from ctx.wait(1.0)
    if v == "nothing":
        yield from ctx.wait(8.0)
        return
    if v == "reach":
        from linkdemo import grip_frame, DOWN
        seen = ctx.observe()
        a = seen.pose("joint", "base")[:3, :3] @ np.array([1.0, 0.0, 0.0])
        g = seen.point("joint", "arm", (0.0, PINCH_Y, 0.0)) + np.array([0, 0, 0.0])
        r = grip_frame(DOWN, a)
        for dz in (0.1, 0.0, 0.0376):
            tgt = g + np.array([0, 0, dz]) + np.array([0, 0, 0.0]) - r @ np.array([0, 0, 0.011]) + np.array([0, 0.0, 0.0])
            yield from ctx.left.move(tgt, r, width=0.05, duration=2.5)
            LOG.setdefault("reach", []).append([[round(1000 * float(x), 1) for x in tgt], [round(1000 * float(x), 1) for x in ctx.observe().tools["left"].tcp]])
        yield from ctx.left.wait(1.0)
        return
    if v == "left_only":
        yield from left_hold(ctx)
        yield from ctx.left.wait(8.0)
        return
    if v == "right_only":
        yield from right_push(ctx)
        yield from ctx.right.wait(4.0)
        return
    if v == "pin_first":  # the pin is driven first, then the arm is raised against it
        yield from right_push(ctx)
        yield from left_hold(ctx)
        yield from ctx.left.wait(3.0)
        return
    yield from left_hold(ctx)
    if v == "drop":  # let the arm go before the pin
        yield from ctx.left.open(0.6)
        yield from ctx.left.retreat(0.08)
        yield from ctx.left.wait(4.0)
        return
    if v == "short_pin":  # the pin stopped 3 mm before its tip enters the window, then both let go
        yield from right_push(ctx, depth=0.015)
        yield from ctx.left.open(0.8)
        yield from ctx.left.wait(0.4)
        yield from ctx.left.retreat(0.04)
        yield from ctx.left.wait(3.5)
        return
    yield from right_push(ctx)
    yield from ctx.left.open(0.8)
    yield from ctx.left.wait(0.4)
    yield from ctx.left.retreat(0.04)
    yield from ctx.left.wait(3.5)
