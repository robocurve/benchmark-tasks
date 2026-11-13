# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Secure a lab gas cylinder upright in its bench clamp (Safety and alignment).

Station table, x forward from the arm bases (arms at y = +-0.305), y to the robot's left, z up, metres.
- A 2 in x 12 in aluminium lecture bottle (0.6 kg, domed foot, 345 mm with its valve) rests on the bench leaning
  back 7.5 deg with its shoulder on the clamp's back plate; its rounded foot cannot stand on its own.
- The clamp: a 300 mm back plate bolted down, two 24 mm side cheeks (90 mm tall, 1.5 mm clear of the bottle), a cross
  bolt that slides in front of the bottle's shoulder 254-266 mm up (85 mm of travel, stops at a keeper block) and a
  cam lever (95 deg over centre) on the bolt guide; the lever presses a pad on the bolt and locks it once the bolt is
  home. An upright bottle clears the plate by 22.8 mm, so let go it falls forward onto the bolt or back to the plate.
"""

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

XC = 0.17  # bottle axis when upright (cradle centre line), m
LEAN_DEG = 7.5  # leaning back on the back plate
R_B = 0.025  # bottle radius
Z_ROD_TOP = 0.20  # height above the table at which the bottle's axis offset is read; the cheeks end here
BAR_Z = 0.26  # bolt centre height above the table
XF = -0.0325  # bolt centre, x relative to XC (2.5 mm clear of the upright bottle)
KNOB_Y = 0.076  # local y of the bolt knob's outer end
BOLT_TRAVEL = 0.085

GREEN = (0.10, 0.45, 0.25)
BRASS = (0.78, 0.60, 0.25)
STEEL = (0.55, 0.57, 0.60)
RED = (0.80, 0.15, 0.12)

# -- bottle: link origin at the foot sphere's centre; the tilt is built into the colliders, so the body frame starts
# upright and the bottle's axis is AXIS0 rotated by the body's pose
_TH = math.radians(LEAN_DEG)
AXIS0 = np.array([math.sin(_TH), 0.0, math.cos(_TH)])
PLATE_FRONT = 0.0478  # the back plate's face, from the cradle centre line; an upright bottle clears it by 22.8 mm
# the foot sits where the leaning bottle's rearmost point (its domed shoulder) is 0.5 mm off the plate
_x_s = XC + PLATE_FRONT - 0.0005 - (0.26 * math.sin(_TH) + R_B)
_t = lk.Tree("bottle", pos=(_x_s, 0.0, TOP + R_B), color=GREEN)
_t.link("body", [lk.capsule(R_B, 0.26, at=tuple(0.13 * AXIS0), axis="Z", rpy_deg=(0.0, LEAN_DEG, 0.0), color=GREEN),
                 lk.cylinder(0.011, 0.04, at=tuple(0.30 * AXIS0), rpy_deg=(0.0, LEAN_DEG, 0.0), color=BRASS)],
        mass=0.6, friction=(0.4, 0.3), contact_offset=0.001)
BOTTLE = _t.build()

# -- clamp: fixed back plate with side cheeks, bolt guide bridge and keeper arm; sliding bolt; cam lever
_t = lk.Tree("clamp", pos=(XC, 0.0, TOP), fixed=True, color=STEEL)
_t.link("post", [
    lk.box((0.020, 0.200, 0.300), (0.0578, 0.0, 0.150)),
    lk.cylinder(0.012, 0.09, (0.0, 0.0385, 0.155)),   # side cheeks: 1.5 mm clear of the bottle each side
    lk.cylinder(0.012, 0.09, (0.0, -0.0385, 0.155)),
    lk.box((0.0478, 0.012, 0.012), (0.0239, 0.0385, 0.206)),   # each cheek is tied back to the plate by an arm
    lk.box((0.0478, 0.012, 0.012), (0.0239, -0.0385, 0.206)),
    lk.box((0.088, 0.036, 0.015), (0.004, 0.054, 0.2795)),     # guide bridge over the bolt (left)
    lk.box((0.0875, 0.015, 0.015), (0.00375, -0.0615, 0.2795)),  # keeper arm (right)
    lk.box((0.012, 0.015, 0.040), (XF, -0.0615, 0.263)),        # keeper block
], mass=2.0, friction=(0.6, 0.5), contact_offset=0.001)
_t.link("bolt", [lk.box((0.010, 0.120, 0.012)),
                 lk.box((0.016, 0.016, 0.020), (0.0, 0.068, 0.0), color=RED)],
        mass=0.06, at=(XF, 0.095, BAR_Z), color=STEEL, friction=(0.4, 0.3), contact_offset=0.001)
_t.link("lever", [lk.box((0.008, 0.012, 0.050), (0.0, 0.0, 0.020)),
                  lk.box((0.012, 0.016, 0.012), (0.0, 0.0, 0.040), color=RED)],
        mass=0.02, at=(-0.034, 0.054, 0.2945), color=(0.15, 0.30, 0.75), friction=(0.5, 0.4), contact_offset=0.001)
_t.prismatic("slide", "post", "bolt", anchor=(XF, 0.095, BAR_Z), axis=(0, -1, 0), limits=(0.0, BOLT_TRAVEL), rest=0.0,
             stiffness=0.0, damping=4.0, friction=0.6, brake=lk.Brake(stiffness=3000.0, damping=200.0, engaged=False))
_t.revolute("cam", "post", "lever", anchor=(-0.034, 0.054, 0.2945), axis=(0, 1, 0), limits_deg=(-3, 100), rest_deg=0,
            stiffness=0.0, damping=0.004, friction=0.0)
CLAMP = _t.build()
lk.detent("clamp", "cam", [0.0, math.radians(95.0)], stiffness=0.02)

OBJECTS = [BOTTLE, CLAMP]

# -- scoring thresholds
UPRIGHT_DEG = 3.0
AXIS_TOL_MM = 4.0  # bottle axis at the cheeks' top height vs the cradle centre line
BOLT_HOME_MM = 80.0
LEVER_CLOSED_DEG = 85.0
TOPPLE_DEG = 40.0
STILL_S, STILL_V, STILL_W = 2.0, 0.003, 3.0  # s, m/s and deg/s
CLEAR_M, CLEAR_V, CLEAR_S = 0.030, 0.005, 1.0
EPS = 1e-6


def measure(bottle, clamp, tools, prev=None, dt=0.05) -> dict:
    """Everything the scorer reads, from the link states (bottle: pose of its link; clamp: joint values)."""
    S = bottle.poses["body"]
    a = S[:3, :3] @ AXIS0
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(a[2])))))
    h = (TOP + Z_ROD_TOP - S[2, 3]) / max(float(a[2]), EPS)
    p = S[:3, 3] + a * h
    err = 1000 * math.hypot(float(p[0]) - XC, float(p[1]))
    speed = spin = 0.0
    if prev is not None:
        speed = float(np.linalg.norm(S[:3, 3] - prev[0])) / dt
        spin = math.degrees(math.acos(max(-1.0, min(1.0, float(a @ prev[1]))))) / dt
    m = {"tilt": tilt, "lean": math.degrees(math.atan2(float(a[0]), float(a[2]))), "lean_y": math.degrees(math.atan2(float(a[1]), float(a[2]))), "foot_x": float(S[0, 3]), "foot_y": float(S[1, 3]), "err": err, "bolt": 1000 * float(clamp.q["slide"]),
         "lever": math.degrees(float(clamp.q["cam"])), "locked": bool(lk.braked("clamp", "slide")),
         "speed": speed, "spin": spin, "clear": {}, "tcp_speed": {}, "_pose": (S[:3, 3].copy(), a.copy())}
    for side, tool in tools.items():
        m["clear"][side] = min(lk.clearance(tool, bottle), lk.clearance(tool, clamp))
        m["tcp_speed"][side] = tool.speed
    return m


def upright(m) -> bool:
    return m["tilt"] <= UPRIGHT_DEG + EPS and m["err"] <= AXIS_TOL_MM + EPS


def bolt_home(m) -> bool:
    return m["bolt"] >= BOLT_HOME_MM - EPS


def secured(m) -> bool:
    return upright(m) and bolt_home(m) and m["lever"] >= LEVER_CLOSED_DEG - EPS and bool(m["locked"])


class Record:
    """Whole-run record (a linkkit History fed every control step, cleared only by the simulator reset)."""

    def __init__(self):
        self.H = lk.History()

    def update(self, m, t) -> None:
        H = self.H
        H.ever("toppled", m["tilt"] >= TOPPLE_DEG)
        if "lean" in m:
            H.trace("lean", (round(m["lean"], 2), round(1000 * (m["foot_x"] - XC), 1), round(m["bolt"], 1), round(m["lever"], 1), bool(m["locked"]), round(m["lean_y"], 2), round(1000 * m["foot_y"], 1)), every=0.5)
        H.ever("seat", upright(m) and bolt_home(m))
        H.ever("closed", secured(m))
        H.set("end", secured(m))
        H.held("still", m["speed"] < STILL_V and m["spin"] < STILL_W)
        far = all(c >= CLEAR_M for c in m["clear"].values()) and all(v < CLEAR_V for v in m["tcp_speed"].values())
        H.held("release", bool(m["clear"]) and far)

    def flags(self) -> dict:
        H = self.H
        return {"toppled": H.ever("toppled", False), "seat": H.ever("seat", False), "closed": H.ever("closed", False),
                "end": bool(H.values.get("end", False)), "still": H.runs.get("still", 0.0),
                "release": H.runs.get("release", 0.0)}


def ladder(rec: Record) -> tuple[float, str]:
    f = rec.flags()
    if f["toppled"]:
        return 0.0, "gate: the bottle toppled"
    if f["end"] and f["still"] >= STILL_S - EPS and f["release"] >= CLEAR_S - EPS:
        return 1.0, "secured, still, hands clear"
    if f["end"]:
        return 0.75, "secured at the end"
    if f["closed"]:
        return 0.5, "lever closed on a seated bottle"
    if f["seat"]:
        return 0.25, "bottle upright in the V with the bolt home"
    return 0.0, "nothing"


REC = Record()
_PREV: list = [None]


@lk.on_reset
def _reset(tick):
    _PREV[0] = None


@lk.on_step
def _track(tick):
    s = tick.states
    m = measure(s["bottle"], s["clamp"], tick.tools, _PREV[0], REC.H.dt or 0.05)
    _PREV[0] = m["_pose"]
    REC.update(m, tick.t)
    # the cam lever's pad locks the bolt once the bolt is home; raising the lever frees it
    if m["lever"] >= 80.0 and m["bolt"] >= BOLT_HOME_MM and not lk.braked("clamp", "slide"):
        lk.brake("clamp", "slide", True)
    elif m["lever"] < 20.0 and lk.braked("clamp", "slide"):
        lk.brake("clamp", "slide", False)


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
VARIANTS = ("", "nothing", "left_only", "right_only", "release_early", "lever_first")
LOG: dict = {}
HOLD_Z = 0.06  # holder pads' height above the table
HOLD_DEPTH = 0.0115  # pad contact centre past the TCP
HOLD_W = 0.034  # pad opening on the 50 mm bottle: pads commanded 16 mm short of the bottle so they grip hard
HOLD_PITCH = 20.0  # holder approach pitched down, deg (a level approach is out of the right arm's reach)
BOLT_HEAD, BOLT_PITCH = -60.0, 50.0  # pusher heading from +x (toward -y) and pitch down, deg
LEVER_HEAD, LEVER_PITCH = -20.0, 20.0


def _dir(head, pitch):
    h, p = math.radians(head), math.radians(pitch)
    return np.array([math.cos(h) * math.cos(p), math.sin(h) * math.cos(p), -math.sin(p)])


def demo_report() -> dict:
    return {"variant": VARIANT, "log": dict(LOG), "ladder": ladder(REC), "flags": REC.flags(),
            "lean_trace": REC.H.traces.get("lean", [])[40:]}


def _tilt(seen) -> float:
    a = seen.pose("bottle", "body")[:3, :3] @ AXIS0
    return math.degrees(math.acos(max(-1.0, min(1.0, float(a[2])))))


def _axis_at(z_above_table, pose):
    a = pose[:3, :3] @ AXIS0
    h = (TOP + z_above_table - pose[2, 3]) / a[2]
    return pose[:3, 3] + a * h


def demo_script(ctx):
    from linkdemo import grip_frame

    v = VARIANT
    LOG.clear()
    LOG["variant"] = v
    done = {"v": False}
    yield from ctx.wait(1.5)
    S = ctx.observe().pose("bottle", "body")
    LOG["tilt_start"] = round(_tilt(ctx.observe()), 2)

    def hold(side, hold_s=None, lean=True):
        """Holder: pads on the bottle from the front, push it upright, hold, then lean it onto the bolt and let go."""
        arm = ctx.arm(side)
        ap = _dir(0.0, HOLD_PITCH)
        r = grip_frame(ap, (0.0, 1.0, 0.0))
        c = _axis_at(HOLD_Z, S)
        tcp = c - r @ np.array([0.0, 0.0, HOLD_DEPTH])
        yield from arm.move(tcp - ap * 0.07, r, width=0.0939, duration=2.5)
        yield from arm.move(tcp, r, duration=1.5)
        yield from arm.grip(HOLD_W, 0.8)
        yield from arm.wait(0.4)
        tl = ctx.observe().tools[side]
        LOG["grip"] = {"opening_mm": round(1000 * tl.opening, 1), "tcp_err_mm": round(1000 * float(np.linalg.norm(tl.tcp - arm.pos)), 1),
                       "tcp": [round(float(x), 4) for x in tl.tcp], "bottle_c": [round(float(x), 4) for x in c]}
        for k in range(8):
            # drag the foot onto the cradle centre line, then pull the hand back until the axis is vertical (pads lag the foot)
            P = ctx.observe().pose("bottle", "body")
            ax = P[:3, :3] @ AXIS0
            lean = math.degrees(math.atan2(float(ax[0]), float(ax[2])))  # + = top toward the rods
            e_f = XC - float(P[0, 3])
            if abs(e_f) > 0.0015:
                d = e_f
            elif abs(lean) > 0.8:
                d = -(HOLD_Z - R_B) * math.tan(math.radians(lean))
            else:
                break
            tl = ctx.observe().tools[side]
            LOG[f"push{k}"] = [round(1000 * e_f, 2), round(lean, 2), round(1000 * d, 2), round(1000 * float(np.linalg.norm(tl.tcp - arm.pos)), 1)]
            yield from arm.move(arm.pos + [d, 0.0, 0.0], r, duration=1.2)
            yield from arm.wait(0.3)
        LOG["tilt_held"] = round(_tilt(ctx.observe()), 2)
        if hold_s is not None:
            yield from arm.wait(hold_s)
        else:
            while not done["v"]:
                yield from arm.wait(0.1)
        if lean:  # tip the bottle a little forward about the pads, so it settles onto the closed bolt, never back
            for k in range(6):
                P = ctx.observe().pose("bottle", "body")
                ax = P[:3, :3] @ AXIS0
                lean_now = math.degrees(math.atan2(float(ax[0]), float(ax[2])))
                LOG[f"lean{k}"] = round(lean_now, 2)
                if lean_now <= -0.4:
                    break
                c_now = _axis_at(HOLD_Z, P)
                yield from arm.arc(c_now, (0.0, 1.0, 0.0), -(lean_now + 0.7), point=(0.0, 0.0, HOLD_DEPTH), follow=1.0,
                                   duration=0.8)
                yield from arm.wait(0.4)
            yield from arm.wait(0.5)
        yield from arm.open(1.2)
        yield from arm.move(arm.pos - ap * 0.09, r, duration=1.2)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.04], r, duration=0.8)
        yield from arm.wait(0.5)

    def left(wait_s=9.0):
        """Operator: push the bolt home along its guide, then pull the cam lever over."""
        arm = ctx.left
        r = grip_frame(_dir(BOLT_HEAD, BOLT_PITCH), (0.0, 0.0, 1.0))
        yield from arm.grip(0.0, 0.3)
        yield from arm.wait(wait_s)
        knob = ctx.observe().point("clamp", "bolt", (0.0, KNOB_Y, 0.0))
        tcp0 = knob + np.array([0.0, 0.003, 0.0]) - r @ np.array([0.0, 0.0, 0.013])
        yield from arm.move(tcp0 + [0.0, 0.04, 0.0], r, duration=2.5)
        yield from arm.move(tcp0, r, duration=1.2)
        if v == "lever_first":
            yield from arm.move(arm.pos + [0.0, 0.06, 0.0], r, duration=0.8)
        else:
            yield from arm.move(tcp0 - [0.0, BOLT_TRAVEL + 0.0005, 0.0], r, duration=2.5)
            yield from arm.wait(0.4)
        LOG["bolt_mm"] = round(1000 * ctx.observe().angle_deg("clamp", "slide"), 2)
        yield from arm.move(arm.pos + [0.0, 0.04, 0.0], r, duration=0.8)
        # lever: fingers closed, pad on its front face near the tip, push it back and down about the hinge
        sn = ctx.observe()
        hinge, ax = sn.axis("clamp", "cam")
        contact = sn.point("clamp", "lever", (-0.004, 0.0, 0.036))
        ap2 = _dir(LEVER_HEAD, LEVER_PITCH)
        r2 = grip_frame(ap2, (0.0, 1.0, 0.0))
        tcp = contact - r2 @ np.array([0.0, 0.0, 0.013 + 0.003])
        yield from arm.move(tcp - ap2 * 0.03, r2, duration=2.0)
        yield from arm.move(tcp, r2, duration=1.0)
        yield from arm.arc(hinge, ax, 75.0, point=(0.0, 0.0, 0.013), follow=0.5, duration=3.0)
        yield from arm.wait(1.0)
        LOG["lever_deg"] = round(ctx.observe().angle_deg("clamp", "cam"), 2)
        done["v"] = True
        yield from arm.move(arm.pos - [0.09, 0.0, 0.0], arm.rot, duration=1.2)
        yield from arm.move(arm.pos + [0.0, 0.0, 0.04], arm.rot, duration=0.8)
        yield from arm.wait(0.5)

    if v == "nothing":
        yield from ctx.wait(10.0)
    elif v == "left_only":
        yield from left()
    elif v == "right_only":
        yield from hold("right", hold_s=12.0, lean=False)
    elif v == "release_early":
        yield from ctx.both(left(wait_s=17.0), hold("right", hold_s=0.0, lean=False))  # the bolt comes ~10 s after the let-go
    else:
        yield from ctx.both(left(), hold("right"))
    yield from ctx.wait(3.5)
    LOG["final_tilt"] = round(_tilt(ctx.observe()), 2)
