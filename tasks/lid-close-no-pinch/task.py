# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Lower the lid of a hand-carry tool chest by its side handle while the other hand holds the lid stay released.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A grey powder-coated steel hand-carry tool chest, 220 mm (x 0.14 to 0.36) by 460 mm (y -0.23 to 0.23) by 160 mm high,
  is fixed to the table. Its orange aluminium lid (1.3 kg, a 226 x 472 x 4 mm plate with 20 mm lips) is hinged along
  the far edge (x 0.36) and rests propped open at 25 deg on a ratchet lid stay.
- The stay is a pawl: the lid can be raised but cannot go down unless the stay's release knob (a spring-return pull
  knob on the left end wall) is held pulled out 6 mm or more. Released, the 1.3 kg lid swings shut by itself.
- The lid has a flat pull tab (the side handle) on its right side. The arm bases are 0.61 m apart and reach 0.35 m, so
  the left arm reaches the release knob (0.5 m or more from the right arm) and the right arm reaches the tab (0.5 m or
  more from the left arm). One hand must hold the release for the whole descent and the other must lower the lid by
  its tab.
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
OPTS = {"lower_s": 11.0, "squeeze": 0.004}
OPTS.update({k: float(v) for k, v in (kv.split("=") for kv in VARIANT.split(",") if "=" in kv)})

# -- geometry (m) -------------------------------------------------------------------------------------------------
HINGE_X = 0.36
BOX_H = 0.160
DEPTH = 0.226  # hinge line to the free edge
PLATE_T = 0.004
PLATE_HALF_W = 0.236
STAY_DEG = 25.0
LOW_DEG = 15.0
MID_DEG = 20.0
CLOSED_DEG = 0.6  # free edge within 2.4 mm of the rim
SPEED_LIMIT = 0.15  # m/s at the free edge, 2.5 mm per frame at 60 fps
CLEAR_M = 0.03
TCP_STILL = 0.01
LEAF_STILL = 0.005  # m/s at the free edge
WINDOW_S = 0.25  # speed = free-edge travel over this window / window
RELEASE_S = 1.0
RIM_MARGIN = 0.006
HINGE_ZONE = 0.025
RIM_ZONE = 0.040
TAB_T = 0.008
TAB_LOCAL = (-0.19, -0.261, 0.004)  # in the lid frame (origin on the hinge line, z up from the plate's underside)
Y_SPAN = (-PLATE_HALF_W, PLATE_HALF_W)  # the plate's footprint
APPROACH_TAB = (1.0, 0.3, -0.8)  # pads' approach to the tab with the lid shut (lid frame)
KNOB = (0.20, 0.260, BOX_H / 2)  # release knob head centre (relative to the table top)
KNOB_SIZE = (0.010, 0.060, 0.030)  # a flat pull tongue on the left end wall, 30 mm out of the wall at rest
KNOB_T = 0.010
KNOB_GRIP = (0.0, 0.012, 0.0)  # where the pads take it, in the knob frame
PULL_FREE = 0.006  # the pawl lets go when the knob is out 6 mm
PULL_HOLD = 0.004  # and catches again below 4 mm
PULL_MAX = 0.020
PULL_HOME = 0.002
PULL_DEMO = 0.014
KNOB_SQUEEZE = 0.007
APPROACH_KNOB = (0.0, 0.0, -1.0)  # straight down onto the tongue, pads closing along x
CONTACT_D = 0.011  # pad contact centre past the TCP
EPS = 1e-6

GREY = (0.55, 0.57, 0.60)
ORANGE = (0.95, 0.50, 0.10)
RED = (0.80, 0.15, 0.10)

t = lk.Tree("chest", pos=(0.0, 0.0, TOP), fixed=True, color=GREY)
W = 0.006
t.link("body", [
    lk.box((0.22, 0.46, W), (0.25, 0.0, W / 2)),
    lk.box((W, 0.46, BOX_H), (0.14 + W / 2, 0.0, BOX_H / 2)),
    lk.box((W, 0.46, BOX_H), (0.36 - W / 2, 0.0, BOX_H / 2)),
    lk.box((0.22, W, BOX_H), (0.25, 0.23 - W / 2, BOX_H / 2)),
    lk.box((0.22, W, BOX_H), (0.25, -0.23 + W / 2, BOX_H / 2)),
], mass=4.0, contact_offset=0.001)

yw = 2 * PLATE_HALF_W
t.link("lid", [
    lk.box((DEPTH, yw, PLATE_T), (-DEPTH / 2, 0.0, PLATE_T / 2)),
    lk.box((0.003, yw, 0.020), (-DEPTH + 0.0015, 0.0, -0.010)),
    lk.box((DEPTH, 0.003, 0.020), (-DEPTH / 2, 0.2345, -0.010)),
    lk.box((DEPTH, 0.003, 0.020), (-DEPTH / 2, -0.2345, -0.010)),
    lk.box((0.040, 0.050, TAB_T), (-0.19, -0.261, TAB_T / 2)),
], mass=1.3, at=(HINGE_X, 0.0, BOX_H), color=ORANGE, contact_offset=0.001)
t.revolute("hinge", "body", "lid", anchor=(HINGE_X, 0.0, BOX_H), axis=(0, 1, 0), limits_deg=(STAY_DEG, 95.0),
           init_deg=STAY_DEG, damping=0.02)
t.link("knob", [lk.box(KNOB_SIZE, (0.0, 0.0, 0.0))], mass=0.05, at=KNOB, color=RED, friction=1.5, contact_offset=0.001)
t.prismatic("pull", "body", "knob", anchor=KNOB, axis=(0, 1, 0), limits=(0.0, PULL_MAX), rest=0.0, init=0.0,
            stiffness=150.0, damping=2.0)
CHEST = t.build()
OBJECTS = [CHEST]

STAY: dict = {}


def _stay_reset():
    STAY.clear()
    STAY.update(lower=math.radians(STAY_DEG), free=False)


_stay_reset()


@lk.on_reset
def _reset(tick):
    REC.reset()
    _stay_reset()


# -- measurement ----------------------------------------------------------------------------------------------------


def pinch_zone(pts, pose) -> str | None:
    """Which pinch zone (if any) any of the world points ``pts`` is in: the gap between the lid plate and the rim
    (40 mm below the plate, over its footprint) or within 25 mm of the hinge line."""
    ylo, yhi = Y_SPAN
    R, p0 = pose[:3, :3], pose[:3, 3]
    for p in pts:
        x, y, z = R.T @ (np.asarray(p, dtype=float) - p0)
        if ylo - RIM_MARGIN <= y <= yhi + RIM_MARGIN:
            if math.hypot(x, z) < HINGE_ZONE:
                return "hinge"
            if -DEPTH - 0.01 <= x <= 0.02 and -RIM_ZONE <= z <= 0.0:
                return "rim"
    return None


def measure(chest, tools) -> dict:
    m = {"clear": {s: lk.clearance(tl, chest) for s, tl in tools.items()},
         "tcp_speed": {s: float(tl.speed) for s, tl in tools.items()}, "pinch": None,
         "q": chest.deg("hinge"), "pull": float(chest.q["pull"])}
    for tl in tools.values():
        z = pinch_zone(tl.pad_points(), chest.poses["lid"])
        if z:
            m["pinch"] = z
    return m


class Record:
    """Whole-run record: a linkkit History fed every control step and cleared only by the simulator reset."""

    def __init__(self):
        self.H = lk.History()
        self.last: dict | None = None
        self.n = 0
        self.trace: list = []
        self.qh: list = []

    def reset(self) -> None:
        self.H.reset()
        self.last, self.n, self.trace = None, 0, []
        self.qh = []

    def edge_speed(self, q: float) -> float:
        """Free-edge speed (m/s) as travel over the last 0.25 s, the way a 60 fps video reads it."""
        h = self.qh
        h.append((self.H.t, q))
        old = [e for e in h if e[0] <= self.H.t - WINDOW_S + 1e-9]
        if not old:
            return 0.0
        t0, q0 = old[-1]
        del h[:max(0, len(old) - 1)]
        return abs(math.radians(q - q0)) * DEPTH / max(self.H.t - t0, 1e-6)

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        q = m["q"]
        v = m["v"] = self.edge_speed(q)
        H.peak("v_peak", v)
        H.ever("fast", v > SPEED_LIMIT)
        if v > 0.08 and len(H.traces.get("hi_v", [])) < 25:
            H.traces.setdefault("hi_v", []).append((round(H.t, 2), round(q, 2), round(v, 3)))
        H.peak("pull_peak", m["pull"])
        H.ever("released", m["pull"] >= PULL_FREE)
        H.ever("mid", q < MID_DEG)
        H.ever("low", q < LOW_DEG)
        H.ever("pinched", m["pinch"] is not None)
        if m["pinch"]:
            H.values.setdefault("pinch_where", m["pinch"])
        H.held("clear", bool(m["clear"]) and min(m["clear"].values()) >= CLEAR_M
               and max(m["tcp_speed"].values()) < TCP_STILL and m["pull"] < PULL_HOME)
        H.held("still", v < LEAF_STILL)


def closed(m) -> bool:
    return m["q"] <= CLOSED_DEG


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "fast" in H.evers:
        return 0.0, f"gate: the lid's free edge moved faster than {SPEED_LIMIT} m/s (peak {H.peaks['v_peak']:.2f} m/s)"
    if "pinched" in H.evers:
        return 0.0, f"gate: a gripper pad was in a pinch zone: {H.values.get('pinch_where')}"
    if closed(m):
        ends = {"lid still 1 s": H.runs.get("still", 0.0) >= RELEASE_S - EPS,
                "both grippers 30 mm clear and still, knob home, 1 s": H.runs.get("clear", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "lid shut slowly, no pad in a pinch zone, both grippers let go"
        return 0.75, "lid shut; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if "low" in H.evers:
        return 0.5, "the lid was lowered below 15 deg but is not shut at the end"
    if "mid" in H.evers:
        return 0.25, "the lid was released from its stay and lowered below 20 deg, no further"
    return 0.0, "the lid never left its stay"


REC = Record()


@lk.on_step
def _track(tick):
    chest = tick.states["chest"]
    pull, ang = float(chest.q["pull"]), math.radians(chest.deg("hinge"))
    free = STAY["free"]
    if not free and pull >= PULL_FREE:  # the pawl lifts: the lid can go all the way down
        lk.set_limits("chest", "hinge", 0.0, math.radians(95.0))
        STAY.update(free=True)
    elif free and pull < PULL_HOLD:  # the knob let go: the pawl catches the lid where it is
        STAY.update(free=False, lower=min(max(ang, 0.0), math.radians(95.0)))
        lk.set_limits("chest", "hinge", STAY["lower"], math.radians(95.0))
    elif not free and ang > STAY["lower"] + 1e-3:  # raising the lid advances the ratchet
        STAY["lower"] = ang
        lk.set_limits("chest", "hinge", ang, math.radians(95.0))
    m = measure(chest, tick.tools)
    REC.update(m)
    if REC.n % 5 == 0:
        REC.trace.append((round(tick.t, 2), round(m["q"], 2), round(m["pull"] * 1000, 1), round(m["v"], 3),
                          [round(1000 * m["clear"][s]) for s in ("left", "right")], m["pinch"]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"ladder": ladder(REC), "peaks": REC.H.peaks, "evers": REC.H.evers, "values": REC.H.values,
            "runs": REC.H.runs, "hi_v": REC.H.traces.get("hi_v"), "phases": LOG, "trace": REC.trace[::3], "variant": VARIANT, "opts": OPTS}


# -- scripted demo ---------------------------------------------------------------------------------------------------
VARIANTS = ("", "nothing", "left_only", "right_only", "slam", "fast")
LOG: dict = {}


def demo_script(ctx):
    """Left arm: pinch the release knob, pull it out and hold it. Right arm: pinch the lid tab, lower the lid slowly once
    the knob is seen pulled out, then both let go and back out. Reads only observable poses."""
    from linkdemo import OPEN_WIDTH, grip_frame

    yield from ctx.wait(1.0)
    v = VARIANT.split(",")[0]
    sides = {"": ("left", "right"), "left_only": ("left",), "right_only": ("right",), "slam": ("left", "right"),
             "fast": ("left", "right"), "nothing": ()}[v]

    def lid_deg():
        return ctx.observe().angle_deg("chest", "hinge")

    def knob_out():
        return ctx.observe().angle_deg("chest", "pull")

    def left_script():
        arm = ctx.arm("left")

        def mark(name):
            LOG.setdefault("left", []).append((name, round(arm.worst_mm, 1), round(arm.worst_deg, 1)))
            arm.worst_mm = arm.worst_deg = 0.0

        seen = ctx.observe()
        app = np.array(APPROACH_KNOB) / np.linalg.norm(APPROACH_KNOB)
        r = grip_frame(app, (1.0, 0.0, 0.0))
        tcp = seen.point("chest", "knob", KNOB_GRIP) - r @ np.array([0.0, 0.0, CONTACT_D])
        yield from arm.move(tcp - app * 0.04, r, width=OPEN_WIDTH, duration=2.5)
        mark("hover")
        yield from arm.move(tcp, r, duration=1.5)
        mark("in")
        yield from arm.grip(KNOB_T - KNOB_SQUEEZE, 0.8)
        yield from arm.wait(0.5)
        yield from arm.move(tcp + np.array([0.0, PULL_DEMO, 0.0]), r, duration=1.5)
        mark("pull")
        for _ in range(60):  # hold the release until the lid is seen shut
            if lid_deg() <= 0.8:
                break
            yield from arm.wait(0.5)
        yield from arm.wait(1.0)
        mark("hold")
        yield from arm.open(0.8)
        mark("open")
        yield from arm.retreat(0.08, 1.2)
        mark("retreat")

    def right_script():
        arm = ctx.arm("right")

        def mark(name):
            LOG.setdefault("right", []).append((name, round(arm.worst_mm, 1), round(arm.worst_deg, 1)))
            arm.worst_mm = arm.worst_deg = 0.0

        seen = ctx.observe()
        hinge, axis = seen.axis("chest", "hinge")
        R_lid = seen.pose("chest", "lid")[:3, :3]
        app = R_lid @ (np.array(APPROACH_TAB) / np.linalg.norm(APPROACH_TAB))
        r = grip_frame(app, R_lid @ np.array([0.0, 0.0, 1.0]))
        tcp = seen.point("chest", "lid", TAB_LOCAL) - r @ np.array([0.0, 0.0, CONTACT_D])
        yield from arm.move(tcp - app * 0.04, r, width=OPEN_WIDTH, duration=2.5)
        mark("hover")
        yield from arm.move(tcp, r, duration=1.5)
        mark("in")
        yield from arm.grip(TAB_T - OPTS["squeeze"], 0.8)
        yield from arm.wait(0.5)
        for i in range(40):  # lower only once the release knob is seen pulled out
            if knob_out() >= PULL_FREE + 0.002 or (v == "right_only" and i >= 8):
                break
            yield from arm.wait(0.25)
        yield from arm.wait(0.5)
        mark("wait")
        if v == "slam":
            yield from arm.arc(hinge, axis, -5.0, point=(0.0, 0.0, CONTACT_D), follow=1.0, duration=3.0)
            yield from arm.open(0.3)
            yield from arm.wait(3.0)
            return
        dur = 0.5 if v == "fast" else OPTS["lower_s"]
        yield from arm.arc(hinge, axis, -STAY_DEG, point=(0.0, 0.0, CONTACT_D), follow=1.0, duration=dur)
        mark("lower")
        for _ in range(3):
            yield from arm.wait(0.4)
            err = lid_deg()
            if err <= 0.3 or err > 5.0:
                break
            yield from arm.arc(hinge, axis, -err, point=(0.0, 0.0, CONTACT_D), follow=1.0,
                               duration=max(1.0, err / 4.0))
        yield from arm.wait(0.5)
        mark("settle")
        yield from arm.open(0.8)
        mark("open")
        yield from arm.retreat(0.08, 1.2)
        mark("retreat")

    gens = {"left": left_script, "right": right_script}
    yield from ctx.both(*[gens[s]() for s in sides])
    yield from ctx.wait(3.0)
