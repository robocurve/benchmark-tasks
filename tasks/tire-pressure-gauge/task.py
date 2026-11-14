# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Read the pressure of a pneumatic wheel's tyre with a stick gauge pressed square onto the valve.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A 10 inch hand-truck wheel (black rubber tyre 260 mm across, 70 mm wide, 1.4 kg, steel rim) lies flat on the table,
  chocked and fixed. Its upward rim face is flat and carries a brass Schrader valve (barrel 8.5 mm across, 19 mm
  tall, a 4.5 mm bore) 55 mm from the axle, at (0.220, -0.100). In the bore sits the valve core pin (2.4 mm), 0.5 mm
  below the barrel's tip, on a 0.8 N/mm spring with 2.5 mm of travel; the valve is open (air can flow) only while the
  pin is pushed down 1.2 mm or more. The tyre is at 35 psi.
- A pencil tyre gauge (steel and brass, 124 mm long, 60 g) stands in its holder at (0.300, -0.300). Its chuck is a
  20 mm square head with a 12 mm square, 12 mm deep socket whose ceiling is the rubber gasket face, and a 2 mm square
  depressor pin standing 2 mm out of the ceiling. Its square 5 mm stick slides out of the top, 1 mm per psi, held where
  it stops by 0.5 N of friction. The seal is the gasket face seated on the barrel's tip (within 0.4 mm, gauge within
  3 deg of the valve's axis, barrel inside the socket within 2 mm); the depressor then pushes the core pin 1.5 mm
  down and the valve opens. The stick is drawn out only while the gasket is seated and the core is pushed down 1.2 mm
  or more, in proportion to the time held (full at 1.5 s). Valve open but gasket not seated is a leak (12 psi only).

Goal: tyre read (stick between 33 and 37 mm), the gauge back upright in its holder, both grippers let go.
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
VARIANT = os.environ.get("DEMO_VARIANT", "")
MM = 0.001
EPS = 1e-6

WHEEL_XY = (0.275, -0.100)
VALVE_XY = (0.220, -0.100)
HUB_Z = TOP + 0.070
HOLDER_XY = (0.300, -0.300)
BRASS = (0.80, 0.62, 0.25)
RED = (0.80, 0.12, 0.12)
BLACK = (0.10, 0.10, 0.10)
STEEL = (0.62, 0.64, 0.68)

CORE_TRAVEL = 0.0025
STEM_H = 0.019
HEAD_H = 0.012  # socket depth: the gasket face (ceiling) is this far above the chuck face
PIN_L = 0.002  # depressor pin length below the gasket face
TIP_Z = HUB_Z + STEM_H  # the barrel tip
BODY_TOP = 0.124
STICK_MAX = 0.070

# ---- wheel (fixed) with the valve stem and its sprung core pin --------------------------------------------------
_t = lk.Tree("wheel", pos=(WHEEL_XY[0], WHEEL_XY[1], TOP), yaw_deg=0, fixed=True, color=BLACK)
_t.link("tyre", [lk.cylinder(0.13, 0.07, (0, 0, 0.035))], mass=1.4, friction=(0.8, 0.8), contact_offset=0.001)
_vx = VALVE_XY[0] - WHEEL_XY[0]
_vy = VALVE_XY[1] - WHEEL_XY[1]
_W, _H = 0.0085, 0.0045  # barrel and bore (m)
_wall = (_W - _H) / 2
_tube = [
    lk.box((_wall, _W, STEM_H), (_W / 2 - _wall / 2, 0, STEM_H / 2)),
    lk.box((_wall, _W, STEM_H), (-_W / 2 + _wall / 2, 0, STEM_H / 2)),
    lk.box((_H, _wall, STEM_H), (0, _W / 2 - _wall / 2, STEM_H / 2)),
    lk.box((_H, _wall, STEM_H), (0, -_W / 2 + _wall / 2, STEM_H / 2)),
]
_t.link("stem", _tube, mass=0.01, at=(_vx, _vy, 0.070), color=BRASS, friction=(0.3, 0.25), contact_offset=0.0005)
_t.fixed_joint("weld", "tyre", "stem")
CORE_REST = 0.0005  # core pin top below the barrel tip
_t.link("core", [lk.box((0.0024, 0.0024, 0.012), (0, 0, STEM_H - CORE_REST - 0.006))], mass=0.003,
        at=(_vx, _vy, 0.070), color=RED, friction=(0.3, 0.25), contact_offset=0.0003)
_t.prismatic("core_slide", "stem", "core", anchor=(_vx, _vy, 0.070), axis=(0, 0, -1), limits=(0.0, CORE_TRAVEL),
             rest=0.0, stiffness=800.0, damping=2.0)
WHEEL = _t.build()

# ---- the gauge: chuck head with a square socket, square body, sliding stick ------------------------------------
_head = [
    lk.box((0.004, 0.020, HEAD_H), (0.008, 0, HEAD_H / 2)),
    lk.box((0.004, 0.020, HEAD_H), (-0.008, 0, HEAD_H / 2)),
    lk.box((0.012, 0.004, HEAD_H), (0, 0.008, HEAD_H / 2)),
    lk.box((0.012, 0.004, HEAD_H), (0, -0.008, HEAD_H / 2)),
    lk.box((0.020, 0.020, 0.004), (0, 0, HEAD_H + 0.002)),
    lk.box((0.002, 0.002, PIN_L), (0, 0, HEAD_H - PIN_L / 2)),
    lk.box((0.012, 0.012, BODY_TOP - HEAD_H - 0.004), (0, 0, (BODY_TOP + HEAD_H + 0.004) / 2)),
]
GAUGE_START = (HOLDER_XY[0], HOLDER_XY[1], TOP + 0.008)
_t = lk.Tree("gauge", pos=GAUGE_START, yaw_deg=0, color=STEEL)
_t.link("body", _head, mass=0.060, friction=(1.5, 1.4), contact_offset=0.0007)
_t.link("stick", [lk.box((0.005, 0.005, 0.080), (0, 0, -0.040), color=(0.9, 0.9, 0.2))], mass=0.006,
        at=(0, 0, BODY_TOP), friction=(0.5, 0.4), contact_offset=0.0005)
_t.prismatic("slide", "body", "stick", anchor=(0, 0, BODY_TOP), axis=(0, 0, 1), limits=(0.0, STICK_MAX), rest=0.0,
             stiffness=0.0, damping=2.0, friction=0.5)
GAUGE = _t.build()

# ---- holder (fixed): 25 mm square pocket, 18 mm deep, 8 mm floor ------------------------------------------------
_t = lk.Tree("holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP), yaw_deg=0, fixed=True, color=(0.45, 0.47, 0.5))
_t.link("holder", [
    lk.box((0.050, 0.050, 0.008), (0, 0, 0.004)),
    lk.box((0.0125, 0.050, 0.018), (0.0185, 0, 0.017)),
    lk.box((0.0125, 0.050, 0.018), (-0.0185, 0, 0.017)),
    lk.box((0.025, 0.0125, 0.018), (0, 0.0185, 0.017)),
    lk.box((0.025, 0.0125, 0.018), (0, -0.0185, 0.017)),
], mass=0.5, friction=(0.6, 0.5), contact_offset=0.001)
HOLDER = _t.build()

OBJECTS = [WHEEL, GAUGE, HOLDER]

# ---- scoring -------------------------------------------------------------------------------------------------------
SEAL_TILT_DEG = 3.0
SEAT_GAP_M = 0.0004  # gasket face to barrel tip
OPEN_CORE_M = 0.0012  # core pin down: valve open
SEAL_OFFSET_M = 0.0020
SEAL_HOLD_S = 1.5
LEAK_READING = 0.012
FULL_READING = 0.035
BAND = (0.033, 0.037)
HOME_XY_M = 0.006
HOME_Z_M = 0.004
HOME_TILT_DEG = 4.0
STILL_S = 1.0
GAUGE_STILL = 0.002  # m/s
CLEAR_M = 0.030
TCP_STILL = 0.010
RELEASE_S = 1.0


def measure(wheel, gauge, tools, prev=None, dt=0.05) -> dict:
    A = np.asarray(gauge.poses["body"], float)
    ax = A[:3, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(ax[2])))))
    tip = np.array([VALVE_XY[0], VALVE_XY[1], TIP_Z])
    d = tip - A[:3, 3]
    zloc = float(d @ ax)
    off = float(np.linalg.norm(d - zloc * ax))  # barrel axis offset from the socket axis
    gap = HEAD_H - zloc  # gasket face to barrel tip along the gauge axis
    core = float(wheel.q["core_slide"])
    clear = {s: float(lk.clearance(t, gauge)) for s, t in tools.items()}
    speed = 0.0 if prev is None else float(np.linalg.norm(A[:3, 3] - prev)) / dt
    hm = A[:3, 3] - np.array(GAUGE_START)
    return {"tilt": tilt, "off": off, "gap": gap, "core": core, "reading": float(gauge.q["slide"]),
            "pos": tuple(float(v) for v in A[:3, 3]), "speed": speed,
            "home_xy": float(np.linalg.norm(hm[:2])), "home_z": float(abs(hm[2])),
            "clear": clear, "tcp_speed": {s: float(t.speed) for s, t in tools.items()}}


def seated(m) -> bool:
    return m["tilt"] <= SEAL_TILT_DEG and m["off"] <= SEAL_OFFSET_M and m["gap"] <= SEAT_GAP_M and m["gap"] > -0.002


def opened(m) -> bool:
    return m["core"] >= OPEN_CORE_M


def sealed(m) -> bool:
    """Air flows into the gauge: gasket seated on the barrel tip AND the depressor holds the core pin open."""
    return seated(m) and opened(m)


def leaking(m) -> bool:
    return opened(m) and not seated(m)


def reading_ok(m) -> bool:
    return BAND[0] <= m["reading"] <= BAND[1]


def home(m) -> bool:
    return m["home_xy"] <= HOME_XY_M and m["home_z"] <= HOME_Z_M and m["tilt"] <= HOME_TILT_DEG


class Record:
    """Whole-run record: a linkkit History cleared only by the simulator reset, plus the latest sample."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0
        self.trace: list = []

    def reset(self):
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []

    def update(self, m, t=None):
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        run = H.held("sealed", sealed(m))
        H.ever("seal_hold", run >= SEAL_HOLD_S - EPS)
        H.peak("reading_max", m["reading"])
        H.peak("core_max", m["core"])
        H.held("home_still", home(m) and m["speed"] < GAUGE_STILL and reading_ok(m))
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)
        return run


def ladder(rec: Record):
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "seal_hold" not in H.evers:
        return 0.0, (f"no sealed press held {SEAL_HOLD_S} s (best {H.best_runs.get('sealed', 0.0):.2f} s, "
                     f"reading peak {1000 * H.peaks.get('reading_max', 0.0):.1f} mm)")
    if not reading_ok(m):
        return 0.4, f"sealed press held, but the stick ends at {1000 * m['reading']:.1f} mm (band 33 to 37)"
    ends = {
        "gauge home in its holder, still 1 s": H.runs.get("home_still", 0.0) >= STILL_S - EPS,
        "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
    }
    if all(ends.values()):
        return 1.0, "tyre read, gauge home in its holder, both grippers let go"
    return 0.7, "tyre read; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)


REC = Record()
_PREV: dict = {}
_DRV: dict = {"rest": None, "on": False}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()
    _DRV.update(rest=None, on=False)


@lk.on_step
def _track(tick):
    gauge, wheel = tick.states["gauge"], tick.states["wheel"]
    m = measure(wheel, gauge, tick.tools, _PREV.get("p"), (tick.t - _PREV["t"]) if _PREV else 0.05)
    _PREV.update(p=np.array(m["pos"]), t=tick.t)
    run = REC.update(m)
    # the gauge: while sealed the stick is drawn out to the tyre pressure over the hold; a leaking press only to 12 psi
    target = None
    if run > 0:
        target = FULL_READING * min(1.0, run / SEAL_HOLD_S)
    elif leaking(m):
        target = LEAK_READING
    drive = target is not None and m["reading"] < target - 0.0002
    if drive:
        if _DRV["rest"] is None or abs(_DRV["rest"] - target) > 1e-4 or not _DRV["on"]:
            lk.set_spring("gauge", "slide", 1000.0, 8.0)
            lk.set_rest("gauge", "slide", target + 0.0005)
            _DRV.update(rest=target, on=True)
    elif _DRV["on"]:
        lk.set_spring("gauge", "slide", 0.0, 2.0)
        _DRV.update(rest=None, on=False)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["tilt"], 1), round(1000 * m["core"], 2), round(1000 * m["gap"], 1),
                          round(1000 * m["reading"], 1), round(run, 2),
                          [round(1000 * v, 0) for v in m["clear"].values()]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
import linkdemo as ld  # noqa: E402

VARIANTS = ("", "nothing", "angled", "light", "quick", "offset", "beside", "left_only")
GRIP_OPEN, GRIP_CLOSED = 0.040, 0.003
GRASP_Z = 0.062  # TCP height above the chuck face along the gauge axis
PRESS_DEPTH = 0.010  # below the seated position (the gasket stops on the tip; the soft arm gives a few N)
LOG: dict = {}
ANGLE = 5.0
GX = float(os.environ.get("GX", "0")) * 0.001
HOVER = 0.022


def rot_y(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def gauge_pose(p, tilt_deg=0.0):
    A = np.eye(4)
    A[:3, :3] = rot_y(tilt_deg)
    A[:3, 3] = p
    return A


class Hand:
    def __init__(self, ctx):
        self.ctx, self.arm = ctx, ctx.right
        self.G = None
        self.dp = np.zeros(3)

    def seen(self):
        return self.ctx.observe().pose("gauge", "body")

    def tcp(self, A):
        T = A @ self.G
        return T[:3, 3] + self.dp, T[:3, :3]

    def follow(self, p0, t0, p1, t1, secs, width=None):
        w = self.arm.width if width is None else width
        p0, p1 = np.asarray(p0, float), np.asarray(p1, float)

        def at(s):
            e = ld.smooth(s)
            p, r = self.tcp(gauge_pose(p0 + e * (p1 - p0), t0 + e * (t1 - t0)))
            return p, r, w

        yield from self.arm.follow(at, secs)

    def servo(self, p, tilt, secs_each=0.4, tries=3, tol=0.0004):
        for _ in range(tries):
            yield from self.arm.wait(secs_each)
            A = self.seen()
            e = np.asarray(p) - A[:3, 3]
            LOG.setdefault("servo_mm", []).append(round(1000 * float(np.linalg.norm(e)), 2))
            if np.linalg.norm(e) < tol:
                break
            self.dp = self.dp + 0.8 * e
            pp, rr = self.tcp(gauge_pose(p, tilt))
            yield from self.arm.move(pp, rr, duration=0.4)


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    if variant == "left_only":
        # the left arm alone, no gauge: it reaches over the valve and presses its closed gripper on the core pin
        la = ctx.left
        RL = ld.grip_frame((1, 0, -1), (0, 1, 0))
        tipp = np.array([VALVE_XY[0], VALVE_XY[1], TIP_Z])
        yield from la.move(tipp + np.array([0, 0, 0.08]), RL, width=GRIP_CLOSED, duration=3.0)
        yield from la.move(tipp + np.array([0, 0, 0.004]), RL, duration=3.0)
        yield from la.move(tipp - np.array([0, 0, 0.006]), RL, duration=2.0)
        yield from ctx.wait(4.0)
        return
    arm = ctx.right
    hand = Hand(ctx)
    R0 = ld.grip_frame((1, 0, -1), (0, 1, 0))
    A0 = hand.seen()
    p_start = A0[:3, 3].copy()
    tcp0 = p_start + np.array([GX, 0, GRASP_Z])
    yield from arm.move(tcp0 - 0.07 * np.array([0.7071, 0, -0.7071]), R0, width=GRIP_OPEN, duration=2.0)
    yield from arm.move(tcp0, R0, duration=1.2)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = arm.rot, arm.pos
    hand.G = np.linalg.inv(A0) @ T
    LOG['open_after_grip_mm'] = round(1000 * float(ctx.observe().tools['right'].opening), 2)
    LOG['pads'] = [[round(1000 * float(v), 1) for v in q] for q in ctx.observe().tools['right'].pad_points()]
    LOG['gauge_axis_vs_tcp_mm'] = [round(1000 * float(v), 1) for v in (arm.pos - hand.seen()[:3, 3])]
    lift = p_start + np.array([0, 0, 0.090])
    yield from hand.follow(p_start, 0.0, lift, 0.0, 1.2)
    top = np.array([VALVE_XY[0], VALVE_XY[1], TIP_Z - HEAD_H])  # gauge origin with the gasket face on the tip
    if variant == "offset":
        top = top + np.array([0, 0.0017, 0])  # pin lands on the barrel wall, not in the bore
    if variant == "beside":
        top = np.array([VALVE_XY[0], VALVE_XY[1] - 0.040, HUB_Z])  # square, firm press on the rim face, off the valve
    tilt = ANGLE if variant == "angled" else 0.0
    hover = top + np.array([0, 0, HOVER])
    yield from hand.follow(lift, 0.0, hover, tilt, 4.0)
    depth = {"light": -0.0011}.get(variant, PRESS_DEPTH)
    hold = {"quick": 0.2}.get(variant, 3.0)
    fast = variant == "quick"
    mid = top + np.array([0, 0, 0.008])
    yield from hand.servo(hover, tilt, tries=5, tol=0.0003)
    yield from hand.follow(hover, tilt, mid, tilt, 3.0)
    yield from hand.servo(mid, tilt, tries=3, tol=0.0003)
    near = top + np.array([0, 0, 0.003])
    yield from hand.follow(mid, tilt, near, tilt, 1.5)
    press = top - np.array([0, 0, depth])
    LOG['slip0_mm'] = round(1000 * float(arm.pos[2] - hand.seen()[2, 3]), 2)
    yield from hand.follow(near, tilt, press, tilt, 0.3 if fast else 2.0)
    LOG['slip1_mm'] = round(1000 * float(arm.pos[2] - hand.seen()[2, 3]), 2)
    LOG["press_seen_z_mm"] = round(1000 * float(hand.seen()[2, 3] - top[2]), 2)
    yield from arm.wait(hold)
    yield from hand.follow(press, tilt, hover, tilt, 0.3 if fast else 1.5)
    yield from hand.follow(hover, tilt, hover + np.array([0, 0, 0.04]), 0.0, 1.5)
    yield from hand.follow(hover + np.array([0, 0, 0.04]), 0.0, lift, 0.0, 3.5)
    hov2 = p_start + np.array([0, 0, 0.030])
    yield from hand.follow(lift, 0.0, hov2, 0.0, 3.0)
    yield from hand.servo(hov2, 0.0, tries=5, tol=0.0003)
    above = p_start + np.array([0, 0, 0.004])
    yield from hand.follow(hov2, 0.0, above, 0.0, 2.5)
    yield from hand.servo(above, 0.0, tries=2, tol=0.0004)
    yield from hand.follow(above, 0.0, p_start + np.array([0, 0, 0.0005]), 0.0, 0.8)
    yield from arm.grip(GRIP_OPEN, 0.6)
    yield from arm.move(arm.pos - np.array([0.08, 0, 0]), duration=1.2)
    yield from arm.move(arm.pos + np.array([0, 0, 0.06]), duration=1.0)
    yield from ctx.wait(2.5)


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    return {"score": s, "reason": why, "variant": VARIANT,
            "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 5) for k, v in H.peaks.items()},
            "runs": {k: round(v, 2) for k, v in H.best_runs.items()},
            "last": {k: (round(v, 4) if isinstance(v, float) else v) for k, v in (REC.last or {}).items()
                     if k not in ("clear", "tcp_speed", "pos")},
            "log": dict(LOG), "trace": REC.trace}
