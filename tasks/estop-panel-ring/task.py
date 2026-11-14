# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""estop-panel-ring: mount an emergency-stop button in a panel hole, tab in its notch, then thread the ring on behind.

Station table; world frame in metres from the midpoint between the arm bases, x forward, y to the robot's left, z up.
- Panel: a 2 mm steel mounting plate (180 x 110 mm) standing on a bench frame, plane y = 0, front toward the left arm.
  A 22.5 mm round hole at (0.21, 0, TOP + 0.095) with a 5 mm anti-rotation notch above it (to 13.5 mm from the centre).
- Button: lying on its head: 40 mm mushroom head, 31 mm collar, 21.8 mm threaded barrel 13 mm long with one 2.8 mm
  anti-rotation tab, 70 g. It starts rolled 140 deg so the tab does not face the notch.
- Ring: 30 mm across-flats hex locking ring, 5 mm thick, 3-start thread of 12 mm lead (180 deg turns it 6 mm), waiting
  6 mm behind the panel. It threads only while the barrel is seated in the hole with the tab in the notch.
Goal: button flush in the panel with the tab in the notch, ring turned snug against the panel back, grippers let go.
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

HX, Y0, HZ = 0.24, -0.04, TOP + 0.12  # hole centre; the panel plane is y = Y0
PT = 0.002  # panel thickness; the plate spans y -1 to +1 mm
R_HOLE = 0.01125
NOTCH_W, NOTCH_TOP = 0.005, 0.0135
BARREL_R, BARREL_L = 0.0109, 0.013
TAB_W, TAB_L = 0.0028, 0.0019
TAB0_DEG = -100.0  # tab direction as presented: from straight up toward +x
LEAD = 0.012  # ring advance per turn
TRAVEL = 0.006  # ring front face from 6 mm behind the panel to touching it
RING_T, RING_Y0 = 0.005, -0.0095  # ring thickness, ring centre y at the start
SPIN_MAX_DEG = 185.0
STAND_H = 0.030
BTN_START = (0.26, 0.135, TOP + STAND_H + 0.0205)  # button origin (collar back face centre)

GREY, RED, BLACK, YELLOW = (0.62, 0.64, 0.68), (0.85, 0.08, 0.08), (0.10, 0.10, 0.10), (0.95, 0.80, 0.10)


def plate_boxes():
    half_w, top, bot = 0.09, 0.055, -0.055
    out = []
    h = 2 * R_HOLE / 30
    for i in range(30):  # 0.75 mm horizontal strips either side of the round hole (hole is the circumscribed staircase)
        z0 = -R_HOLE + i * h
        z1 = z0 + h
        zmin = 0.0 if z0 < 0 < z1 else min(abs(z0), abs(z1))
        w = math.sqrt(R_HOLE ** 2 - zmin ** 2)
        for sgn in (-1, 1):
            out.append(lk.box((half_w - w, PT, h), (sgn * (half_w + w) / 2, 0.0, (z0 + z1) / 2), contact_offset=0.001))
    hs = NOTCH_W / 2
    for sgn in (-1, 1):  # above the hole, either side of the notch
        out.append(lk.box((half_w - hs, PT, top - R_HOLE), (sgn * (half_w + hs) / 2, 0.0, (top + R_HOLE) / 2),
                          contact_offset=0.001))
    out.append(lk.box((NOTCH_W, PT, top - NOTCH_TOP), (0.0, 0.0, (top + NOTCH_TOP) / 2), contact_offset=0.001))
    out.append(lk.box((2 * half_w, PT, -bot - R_HOLE), (0.0, 0.0, (bot - R_HOLE) / 2), contact_offset=0.001))
    out.append(lk.box((2 * half_w, 0.060, 0.065), (0.0, 0.0, -0.0875), color=(0.35, 0.36, 0.40)))  # bench frame
    return out


def ring_boxes():
    out = []
    for k in range(6):
        a = math.radians(60 * k)
        r = 0.0131
        out.append(lk.box((0.0038, RING_T, 0.0174), (r * math.cos(a), 0.0, r * math.sin(a)),
                          rpy_deg=(0.0, -60.0 * k, 0.0), contact_offset=0.001))
    return out


t = lk.Tree("panel", pos=(HX, Y0, HZ), fixed=True, color=GREY)
t.link("plate", plate_boxes(), mass=2.0, friction=(0.3, 0.25))
t.link("hub", [lk.box((0.002, 0.001, 0.002), (0.0, 0.0, 0.0), contact_offset=0.0005)], mass=0.003,
       at=(0.0, RING_Y0, 0.015), color=YELLOW)
t.link("ring", ring_boxes(), mass=0.02, at=(0.0, RING_Y0, 0.0), color=YELLOW, friction=(0.9, 0.8))
t.prismatic("slide", "plate", "hub", anchor=(0.0, RING_Y0, 0.0), axis=(0, 1, 0), limits=(0.0, TRAVEL), rest=0.0,
            stiffness=30000.0, damping=300.0)
t.revolute("spin", "hub", "ring", anchor=(0.0, RING_Y0, 0.0), axis=(0, 1, 0), limits_deg=(0.0, 0.05), damping=0.003,
           friction=0.008)
PANEL = t.build()

a0 = math.radians(TAB0_DEG)
TAB_DIR0 = np.array([math.sin(a0), 0.0, math.cos(a0)])
BUTTON_SHAPES = [
    lk.cylinder(0.020, 0.022, (0.0, 0.015, 0.0), axis="Y", color=RED),
    lk.cylinder(0.0155, 0.004, (0.0, 0.002, 0.0), axis="Y", color=BLACK),
    lk.cylinder(BARREL_R, BARREL_L - 0.002, (0.0, -(BARREL_L - 0.002) / 2, 0.0), axis="Y", color=(0.20, 0.20, 0.22)),
    lk.cylinder(0.0098, 0.002, (0.0, -BARREL_L + 0.001, 0.0), axis="Y", color=(0.20, 0.20, 0.22)),  # chamfered tip
    # tab: r 10.0 to 12.9 mm from the axis, 2.8 mm wide, 1.9 mm long (inside the 2 mm panel thickness)
    lk.box((TAB_W, TAB_L, 0.0029), (0.01145 * TAB_DIR0[0], -TAB_L / 2, 0.01145 * TAB_DIR0[2]),
           rpy_deg=(0.0, TAB0_DEG, 0.0), color=YELLOW),
]
t = lk.Tree("button", pos=BTN_START, color=BLACK)
t.link("body", BUTTON_SHAPES, mass=0.070, friction=(1.0, 0.9), contact_offset=0.001)
BUTTON = t.build()

t = lk.Tree("chocks", pos=(BTN_START[0], BTN_START[1] + 0.015, TOP), fixed=True, color=(0.35, 0.36, 0.40))
t.link("blocks", [lk.box((0.047, 0.026, STAND_H), (0.0, 0.0, STAND_H / 2))]
       + [lk.box((0.004, 0.026, 0.004), (sgn * 0.0155, 0.0, STAND_H + 0.002)) for sgn in (-1, 1)], mass=0.3)
CHOCKS = t.build()

OBJECTS = [PANEL, BUTTON, CHOCKS]

# -- measurement and scoring ----------------------------------------------------------------------------------------
SEAT_Y = 0.0030  # collar back face within 2.0 mm of the panel front (a 2 mm feeler does not pass; the panel front is at y = +1 mm)
INSERT_Y = 0.006  # barrel at least 8 mm into the hole
LATERAL_M = 0.003
TAB_TOL_DEG = 12.0
TILT_SEAT_DEG, TILT_INSERT_DEG = 6.0, 8.0
ENGAGE_Y, ENGAGE_TILT = 0.0035, 8.0
RING_ON_DEG, RING_SNUG_DEG, RING_MAX_DEG = 90.0, 165.0, 185.0
CLEAR_M, TCP_STILL, STILL_SPEED, RING_STILL, RELEASE_S = 0.03, 0.01, 0.003, 0.005, 1.0
DROP_Z = TOP - 0.03
EPS = 1e-6


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def tab_angle(R) -> float:
    """Tab direction from straight up toward +x, deg, from the button's world rotation matrix."""
    d = np.asarray(R) @ TAB_DIR0
    return math.degrees(math.atan2(d[0], d[2]))


def measure(states, tools) -> dict:
    b, p = states["button"], states["panel"]
    P = b.point("body", (0.0, 0.0, 0.0))
    R = b.rotation("body")
    ax = R @ np.array([0.0, 1.0, 0.0])
    return {
        "y": float(P[1] - Y0), "dx": float(P[0] - HX), "dz": float(P[2] - HZ), "z_abs": float(P[2]),
        "tab_err": abs(wrap_deg(tab_angle(R))), "tilt": math.degrees(math.acos(max(-1.0, min(1.0, float(ax[1]))))),
        "spin": math.degrees(p.q["spin"]), "slide": float(p.q["slide"]),
        "btn_speed": float(b.max_speed()), "ring_speed": float(p.max_speed()),
        "clear": {s: min(lk.clearance(tools[s], b), lk.clearance(tools[s], p, ["ring"])) for s in ("left", "right")},
        "tcp_speed": {s: tools[s].speed for s in ("left", "right")},
    }


def aligned(m, tilt_tol) -> bool:
    return (abs(m["dx"]) <= LATERAL_M and abs(m["dz"]) <= LATERAL_M and m["tab_err"] <= TAB_TOL_DEG
            and m["tilt"] <= tilt_tol)


def inserted(m) -> bool:
    return m["y"] <= INSERT_Y and aligned(m, TILT_INSERT_DEG)


def seated(m) -> bool:
    return m["y"] <= SEAT_Y and aligned(m, TILT_SEAT_DEG)


def engaged(m) -> bool:
    return m["y"] <= ENGAGE_Y and aligned(m, ENGAGE_TILT)


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n = None, 0

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        H.ever("dropped", m["z_abs"] < DROP_Z)
        H.ever("inserted", inserted(m))
        H.ever("seated", seated(m))
        H.ever("ring_on", seated(m) and m["spin"] >= RING_ON_DEG)
        H.peak("spin_max", m["spin"])
        H.held("released", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL)
        H.held("still", m["btn_speed"] < STILL_SPEED and m["ring_speed"] < RING_STILL)


def final_ok(m) -> dict:
    return {
        "button seated flush with the tab in the notch": seated(m),
        "ring turned snug (165 to 185 deg)": RING_SNUG_DEG <= m["spin"] <= RING_MAX_DEG,
        "ring face within 0.5 mm of the panel": m["slide"] >= TRAVEL - 0.0005,
    }


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "dropped" in H.evers:
        return 0.0, f"gate: the button fell off the table at {H.evers['dropped']:.2f} s"
    ends = final_ok(m)
    if all(ends.values()):
        more = {"both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS,
                "button and ring still 1 s": H.runs.get("still", 0.0) >= RELEASE_S - EPS}
        if all(more.values()):
            return 1.0, "button seated with the tab in the notch, ring snug on it, both grippers let go"
        return 0.75, "ring snug, but not yet: " + ", ".join(k for k, ok in more.items() if not ok)
    if "ring_on" in H.evers:
        return 0.75, "the ring was turned 90 deg on the seated button, but the end state is not snug and seated"
    if "seated" in H.evers:
        return 0.5, "the button was seated flush with the tab in the notch, the ring never turned 90 deg on it"
    if "inserted" in H.evers:
        return 0.25, "the barrel went 8 mm into the hole with the tab in the notch, never fully seated"
    return 0.0, "the barrel never went 8 mm into the hole with the tab in the notch"


REC = Record()
_ENG = {"on": False}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _ENG["on"] = False


@lk.on_step
def _track(tick):
    m = measure(tick.states, tick.tools)
    REC.update(m)
    # thread: the ring turns only while the barrel is seated in the hole; its advance follows its angle
    on = engaged(m)
    if on != _ENG["on"]:
        _ENG["on"] = on
        hi = math.radians(SPIN_MAX_DEG) if on else max(math.radians(m["spin"]), 0.0) + 1e-3
        lk.set_limits("panel", "spin", 0.0, hi)
    lk.set_rest("panel", "slide", float(np.clip(LEAD * math.radians(m["spin"]) / (2 * math.pi), 0.0, TRAVEL)))
    if REC.n % 10 == 0:
        REC.H.trace("diag", (round(tick.t, 1), round(1000 * m["y"], 1), round(1000 * m["dx"], 1),
                             round(1000 * m["dz"], 1), round(m["tab_err"], 1), round(m["tilt"], 1),
                             round(m["spin"], 1), round(1000 * m["slide"], 2)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():
    return {}


LOG: dict = {}


def demo_report() -> dict:
    sc, why = ladder(REC)
    return {"variant": VARIANT, "score": sc, "why": why, "last": REC.last, "evers": REC.H.evers,
            "runs": REC.H.runs, "diag": REC.H.traces.get("diag", []), "log": LOG}


# -- scripted demo -------------------------------------------------------------------------------------------------
# Reads only what a camera could measure (object and link poses) and the robot's own grippers.
from linkdemo import grip_frame, rot_about, smooth  # noqa: E402

VARIANTS = ("", "nothing", "no_roll", "right_only", "drop_in_front", "left_only")
Y_AX = np.array([0.0, 1.0, 0.0])
HEAD_MID_Y = 0.0115  # grasp plane above the button origin: the centre of mass, so it does not swing in the pads
D_L = 0.0115  # pad contact centre past the TCP
HEAD_W = 0.0372  # pads across the 40 mm head, 1.4 mm squeeze each
RING_W = 0.0300 - 0.0006  # pads across the hex flats
V = VARIANT


def _btn(ctx):
    pose = ctx.observe().pose("button", "body")
    return np.asarray(pose[:3, 3], float), np.asarray(pose[:3, :3], float)


def seen_spin_deg(ctx) -> float:
    """Ring angle from the ring's pose, as a camera would read a painted mark on it."""
    R = np.asarray(ctx.observe().pose("panel", "ring"), float)[:3, :3]
    x = R @ np.array([1.0, 0.0, 0.0])
    return math.degrees(math.atan2(-x[2], x[0]))


def seen_ring_front(ctx) -> float:
    return float(np.asarray(ctx.observe().pose("panel", "ring"), float)[1, 3]) + RING_T / 2


PICK_C = 20.0  # deg: the pads' closing axis, turned about y from horizontal (clears the stand; reachable at the hole)
APP_L = np.array([0.0, -math.cos(math.radians(10)), -math.sin(math.radians(10))])
APP_R = np.array([math.sin(math.radians(20)) * math.cos(math.radians(25)),
                  math.cos(math.radians(20)) * math.cos(math.radians(25)), -math.sin(math.radians(25))])
D_R = 0.0115


X_AX = np.array([1.0, 0.0, 0.0])
Z_AX = np.array([0.0, 0.0, 1.0])


def level(ctx, arm, rounds=3):
    """Turn the held button until its axis is horizontal and along y (pitch and yaw read from its pose)."""
    for _ in range(rounds):
        P, R = _btn(ctx)
        ax = R @ Y_AX
        pitch = math.degrees(math.atan2(ax[2], ax[1]))
        yaw = math.degrees(math.atan2(ax[0], ax[1]))
        if abs(pitch) < 0.5 and abs(yaw) < 0.5:
            break
        centre = arm.pos + arm.rot @ np.array([0.0, 0.0, D_L])
        yield from arm.arc(centre, X_AX, -pitch, follow=1.0, point=(0.0, 0.0, D_L), duration=1.0)
        centre = arm.pos + arm.rot @ np.array([0.0, 0.0, D_L])
        yield from arm.arc(centre, Z_AX, yaw, follow=1.0, point=(0.0, 0.0, D_L), duration=1.0)
        yield from arm.wait(0.3)


def servo(ctx, arm, y_goal, step=0.004):
    """Advance toward the panel until the collar is at y_goal above the panel front, centring the barrel tip on the hole."""
    for _ in range(60):
        P, R = _btn(ctx)
        tip = P + R @ np.array([0.0, -BARREL_L, 0.0])
        rel_y = P[1] - (Y0 + 0.001)
        if rel_y - y_goal < 0.0003:
            break
        near = rel_y - BARREL_L < 0.012  # the tip is within 12 mm of the panel face
        ref = tip if near else P
        err = np.array([HX - ref[0], 0.0, HZ - ref[2]])
        if near and np.linalg.norm(err) > 0.0008:
            yield from arm.move(arm.pos + err, duration=0.4)
            yield from arm.wait(0.1)
            continue
        dy = min(step if near else 0.02, rel_y - y_goal)
        move = err.copy()
        move[1] = -dy - (0.0004 if rel_y - y_goal <= step else 0.0)
        yield from arm.move(arm.pos + move, duration=0.4 if near else 0.8)


def left_arm(ctx):
    arm = ctx.left
    if V == "right_only":
        return
    yield from ctx.wait(0.3)
    P, R = _btn(ctx)
    r0 = grip_frame(APP_L, rot_about(Y_AX, PICK_C) @ np.array([1.0, 0.0, 0.0]))
    contact = P + np.array([0.0, HEAD_MID_Y, 0.0])
    tcp = contact - r0 @ np.array([0.0, 0.0, D_L])
    yield from arm.move(tcp - APP_L * 0.07, r0, width=0.094, duration=1.6)
    yield from arm.move(tcp, r0, duration=1.0)
    yield from arm.grip(HEAD_W, 0.8)
    yield from arm.wait(0.4)
    P, R = _btn(ctx)
    rel = arm.pos - P
    mid = np.array([HX, Y0 + 0.07, HZ])
    yield from arm.move(arm.pos + [0.0, 0.0, 0.05], duration=0.8)
    yield from arm.move(mid + rel, duration=1.6)
    if V != "no_roll":  # roll the tab up to the notch, closed loop from the seen tab angle
        for _ in range(4):
            P, R = _btn(ctx)
            err = wrap_deg(tab_angle(R))
            if abs(err) < 1.0:
                break
            centre = arm.pos + arm.rot @ np.array([0.0, 0.0, D_L])
            yield from arm.arc(centre, Y_AX, -err, follow=1.0, point=(0.0, 0.0, D_L),
                               duration=max(0.8, abs(err) / 70.0))
            yield from arm.wait(0.25)
    LOG["tab_after_roll"] = round(wrap_deg(tab_angle(_btn(ctx)[1])), 2)
    yield from level(ctx, arm)
    yield from servo(ctx, arm, 0.045)
    P, _R = _btn(ctx)
    LOG["before_push_mm"] = [round(1000 * (P[0] - HX), 2), round(1000 * (P[1] - Y0), 1), round(1000 * (P[2] - HZ), 2)]
    if V == "drop_in_front":
        yield from arm.open(0.5)
        yield from arm.wait(2.0)
        return
    yield from servo(ctx, arm, 0.0012)
    yield from arm.wait(0.5)
    P, _R = _btn(ctx)
    LOG["after_push_y_mm"] = round(1000 * (P[1] - Y0), 2)
    LOG["t_pushed"] = ctx.emb._step if hasattr(ctx.emb, '_step') else None
    LOG["pushed"] = True
    if V != "left_only":
        while not LOG.get("ring_done"):
            yield
    yield from arm.open(0.8)
    yield from arm.move(arm.pos + [0.0, 0.07, 0.0], duration=1.2)
    yield from arm.move(arm.pos + [0.0, 0.0, 0.05], duration=1.0)


def right_arm(ctx):
    arm = ctx.right
    if V in ("left_only", "no_roll", "drop_in_front", "nothing"):
        return
    if V != "right_only":
        while not LOG.get("pushed"):
            yield
        yield from arm.wait(0.5)
    for stroke in range(4):
        th = seen_spin_deg(ctx)
        th = th if th > -90 else th + 360.0
        th = max(th, 0.0)
        if th >= 172.0:
            break
        c = np.array([HX, seen_ring_front(ctx) - RING_T / 2, HZ])  # ring mid-plane on the axis
        r1 = grip_frame(APP_R, rot_about(Y_AX, th) @ np.array([1.0, 0.0, 0.0]))
        tcp = c - r1 @ np.array([0.0, 0.0, D_R])
        yield from arm.move(tcp - APP_R * 0.06, r1, width=0.094, duration=1.6 if stroke == 0 else 1.0)
        yield from arm.move(tcp, r1, duration=0.8)
        yield from arm.grip(RING_W, 0.7)
        yield from arm.wait(0.3)
        deg = min(95.0, 184.0 - th)

        def at(s, deg=deg, r1=r1, c=c):
            a = smooth(s) * deg
            r = rot_about(Y_AX, a) @ r1
            return c + np.array([0.0, LEAD * a / 360.0, 0.0]) - r @ np.array([0.0, 0.0, D_R]), r, arm.width

        yield from arm.follow(at, 2.2)
        yield from arm.wait(0.2)
        yield from arm.open(0.5)
        yield from arm.move(arm.pos - APP_R * 0.04, duration=0.6)
    LOG["ring_final_deg"] = round(seen_spin_deg(ctx), 1)
    LOG["ring_done"] = True
    yield from arm.move(arm.pos - APP_R * 0.06, duration=0.8)


def demo_script(ctx):
    if V == "nothing":
        yield from ctx.wait(30.0)
        return
    yield from ctx.both(left_arm(ctx), right_arm(ctx))
    yield from ctx.wait(3.0)
