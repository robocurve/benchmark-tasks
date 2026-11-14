# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Check that a seat-belt retractor locks on a sudden pull.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- A seat-belt emergency-locking retractor (steel frame about 90 x 70 x 110 mm, 0.5 kg, on a grey steel test plate
  fixed to the table) sits at x 0.55, y -0.12 with its webbing slot facing the robot, 60 mm above the table. 46 mm wide
  black polyester webbing runs out of the slot to a silver steel tongue plate (85 x 45 x 4 mm).
- A return spring draws the webbing back (about 0.1 N per cm of pull-out, plus damping). A webbing-speed lock catches
  the spool when the webbing is pulled out faster than 0.30 m/s (speed over 0.1 s); it lets go when the webbing is
  let go (no hand on the tongue) and the spring winds the webbing back in.

Goal: pull the webbing out 150 mm at a steady pace, jerk it so that it locks, then let go so it winds back in.
"""

from __future__ import annotations

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
DT = 0.05  # control step, 20 Hz

# -- the retractor -----------------------------------------------------------------------------------------------
BELT_POS = (0.55, -0.12, TOP)
SLOT_Z = 0.060  # webbing centre above the table
WEB_W, WEB_T = 0.046, 0.0012
TONGUE = (0.085, 0.045, 0.004)  # x (along the webbing), y (width), z
TONGUE_X = 0.100 + TONGUE[0] / 2  # tongue centre from the slot at rest (0.1 m of webbing outside)
TRAVEL = 0.30
SPRING_K, SPRING_REST, DAMP = 10.0, -0.02, 5.0  # N/m, m (the stop is at 0), N s/m
LOCK_K, LOCK_C = 20000.0, 200.0  # N/m, N s/m

BLACK, STEEL, PLATE = (0.08, 0.08, 0.09), (0.78, 0.79, 0.82), (0.50, 0.52, 0.55)

_t = lk.Tree("belt", pos=BELT_POS, yaw_deg=180.0, fixed=True, color=PLATE)
_t.link("frame", [lk.box((0.20, 0.16, 0.008), (-0.045, 0, 0.004)),
                  lk.box((0.090, 0.070, 0.110), (-0.045, 0, 0.063), color=(0.30, 0.31, 0.34))], mass=0.5)
_t.link("web", [lk.box((0.40, WEB_W, WEB_T), (-0.10, 0, 0), color=BLACK, contact_offset=0.001),
                lk.box(TONGUE, (TONGUE_X, 0, 0), color=STEEL, contact_offset=0.001)],
        mass=0.15, at=(0.0, 0.0, SLOT_Z))
_t.prismatic("pull", "frame", "web", anchor=(0.0, 0.0, SLOT_Z), axis=(1.0, 0.0, 0.0), limits=(0.0, TRAVEL),
             rest=SPRING_REST, stiffness=SPRING_K, damping=DAMP, brake=lk.Brake(stiffness=LOCK_K, damping=LOCK_C))
BELT = _t.build()
OBJECTS = [BELT]

# -- scoring constants (mm and m/s: every one is read from webbing marks and a video) -----------------------------
SLOW_E = 0.150  # pulled out at least this far ...
SLOW_V = 0.25  # ... at under this speed (speed over 0.1 s)
JERK_V = 0.30  # the lock catches above this speed
STOP_MM = 15.0  # after the jerk the webbing may run on at most this far
HOME_MM = 5.0  # wound back in to within this at the end
CLEAR_M, RELEASE_S, TCP_STILL = 0.030, 1.0, 0.005
EPS = 1e-6


def speed(es: list) -> float:
    """Pay-out speed (m/s) over the last 0.1 s from the last three extension samples (what marks and video give)."""
    return (es[-1] - es[-3]) / (2 * DT) if len(es) >= 3 else 0.0


class Record:
    """Whole-run record: a linkkit History updated every control step and cleared only by the simulator reset, so
    letting go and starting again never clears an early jerk. Pure Python."""

    def __init__(self):
        self.H = lk.History()
        self.es: list = []
        self.last: dict | None = None
        self.v = 0.0
        self.trace: list = []

    def reset(self) -> None:
        self.H.reset()
        self.es, self.last, self.v, self.trace = [], None, 0.0, []

    def update(self, m: dict, t: float | None = None) -> None:
        H = self.H
        if t is not None:
            H._tick(t)
        e = m["e"]
        self.es = (self.es + [e])[-3:]
        self.last, self.v = m, speed(self.es)
        H.peak("e_max", e)
        H.peak("v_max", self.v)
        if "jerk" not in H.evers:
            if self.v >= JERK_V:
                H.ever("jerk", True)
                H.set("e_jerk", e)
                if "slow_done" not in H.evers:
                    H.ever("early_jerk", True)  # locked before 150 mm had come out steadily
            elif "slow_done" not in H.evers:
                H.peak("v_slow", self.v)
                if e >= SLOW_E and H.peaks["v_slow"] < SLOW_V:
                    H.ever("slow_done", True)
        else:
            H.peak("e_after", e)
        clear = m["clear"]
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M and max(m["tcp_speed"].values()) < TCP_STILL)

    def lock_now(self) -> bool:
        return self.v >= JERK_V


def ladder(rec: Record) -> tuple[float, str]:
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    if "early_jerk" in H.evers:
        return 0.0, (f"gate: the webbing was jerked at {H.values['e_jerk'] * 1000:.0f} mm, before 150 mm had come out "
                     f"steadily, so it locked early")
    if "jerk" not in H.evers or "slow_done" not in H.evers:
        why = ("no steady 150 mm pull-out" if "slow_done" not in H.evers
               else "the webbing was never jerked fast enough to lock")
        return 0.0, why
    stopped = (H.peaks.get("e_after", 9) - H.values["e_jerk"]) * 1000 <= STOP_MM + EPS
    home = m["e"] * 1000 <= HOME_MM + EPS
    clear = H.runs.get("released", 0.0) >= RELEASE_S - EPS
    if not stopped:
        return 0.25, "steady pull then a jerk, but the webbing ran on more than 15 mm"
    if home and clear:
        return 1.0, "steady 150 mm pull-out, jerk locked the webbing, let go, wound back in, grippers clear"
    if home:
        return 0.75, "locked and wound back in; grippers not yet 30 mm clear and still 1 s"
    return 0.5, "steady pull then a jerk, the webbing stopped; not yet wound back in"


REC = Record()
LOCK = {"on": False, "free": 0}


@lk.on_reset
def _reset(tick):
    REC.reset()
    LOCK.update(on=False, free=0)


@lk.on_step
def _track(tick):
    s = tick.states["belt"]
    clear = {side: lk.clearance(tool, s) for side, tool in tick.tools.items()}
    REC.update({"e": float(s.q["pull"]), "clear": clear, "tcp_speed": {k: t.speed for k, t in tick.tools.items()}})
    # the lock: catches the spool on a fast pull-out, lets go once no hand holds the webbing
    held = any(t.opening < 0.060 and lk.clearance(t, s, ["web"]) < 0.004 for t in tick.tools.values())
    if not LOCK["on"] and REC.lock_now():
        lk.brake("belt", "pull", True)
        LOCK["on"], LOCK["free"] = True, 0
    elif LOCK["on"]:
        LOCK["free"] = 0 if held else LOCK["free"] + 1
        if LOCK["free"] >= 5:
            lk.brake("belt", "pull", False)
            LOCK["on"] = False
    if len(REC.trace) < 4000 and REC.last and int(round(tick.t / DT)) % 4 == 0:
        REC.trace.append((round(tick.t, 2), round(1000 * REC.last["e"], 1), round(REC.v, 3), int(LOCK["on"]),
                          int(held), [round(1000 * c, 0) for c in clear.values()]))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


# -- scripted demo ---------------------------------------------------------------------------------------------------
# Reads only what a camera could measure: the webbing link's pose (its pull-out from marks) and the robot's own
# grippers; never the lock state or the record.

VARIANTS = ("", "nothing", "slow_only", "jerk_only", "short_slow", "soft_pull")
LOG: dict = {}
SLOW_SPEED = 0.08  # m/s for the steady pull
JERK_M, JERK_S = 0.06, 0.10  # the jerk: 60 mm commanded in 0.1 s
TCP_DZ = 0.0095 + TONGUE[2] / 2  # TCP above the tongue's centre: pad tips 3.5 mm into its 4 mm edge
GRIP_OPEN, GRIP_CLOSED = 0.075, 0.043


def _pullout(ctx, p0) -> float:
    return float(np.linalg.norm(ctx.observe().pose("belt", "web")[:3, 3] - p0))


def demo_script(ctx):
    variant = VARIANT
    LOG.clear()
    LOG["variant"] = variant
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    from linkdemo import down

    arm = ctx.right
    pose0 = ctx.observe().pose("belt", "web")
    p0 = pose0[:3, 3].copy()
    d = pose0[:3, :3] @ np.array([1.0, 0.0, 0.0])  # pull-out direction (toward the robot)
    LOG["d"] = [round(float(v), 3) for v in d]
    tongue = pose0[:3, :3] @ np.array([TONGUE_X, 0.0, 0.0]) + p0
    LOG["tongue"] = [round(float(v), 3) for v in tongue]
    rot = down(0.0)
    top = tongue + np.array([0.0, 0.0, TCP_DZ])
    yield from arm.move(top + np.array([0.0, 0.0, 0.08]), rot, width=GRIP_OPEN, duration=2.0)
    yield from arm.move(top, rot, duration=1.2)
    yield from arm.grip(GRIP_CLOSED, 0.8)
    yield from arm.wait(0.4)
    pos = arm.pos.copy()

    def drive(speed, until):
        nonlocal pos
        while _pullout(ctx, p0) < until:
            pos = pos + d * speed * DT
            arm._solve(pos, rot, arm.width)
            yield

    def jerk():
        nonlocal pos
        n = round(JERK_S / DT)
        for _ in range(n):
            pos = pos + d * (JERK_M / n)
            arm._solve(pos, rot, arm.width)
            yield

    if variant == "jerk_only":
        yield from jerk()
    elif variant == "short_slow":
        yield from drive(SLOW_SPEED, 0.08)
        yield from arm.wait(0.5)
        yield from jerk()
    else:
        yield from drive(SLOW_SPEED, SLOW_E + 0.015)
        LOG["e_slow_mm"] = round(1000 * _pullout(ctx, p0), 1)
        yield from arm.wait(0.6)
        if variant == "soft_pull":
            yield from drive(0.20, SLOW_E + 0.075)
        elif variant != "slow_only":
            yield from jerk()
    LOG["e_end_pull_mm"] = round(1000 * _pullout(ctx, p0), 1)
    yield from arm.wait(0.4)
    yield from arm.grip(GRIP_OPEN, 0.5)  # let go: the lock frees and the spring winds the webbing in
    yield from arm.move(arm.pos + np.array([0.0, 0.0, 0.08]), duration=0.8)
    yield from arm.wait(0.5)
    yield from arm.move(arm.pos + np.array([0.0, 0.10, 0.0]), duration=1.0)  # clear of the tongue
    yield from ctx.wait(4.0)
    LOG["e_final_mm"] = round(1000 * _pullout(ctx, p0), 1)


def demo_report() -> dict:
    s, why = ladder(REC)
    H = REC.H
    return {"score": s, "reason": why, "variant": VARIANT, "evers": {k: round(v, 2) for k, v in H.evers.items()},
            "peaks": {k: round(v, 3) for k, v in H.peaks.items()},
            "values": {k: round(v, 4) for k, v in H.values.items()},
            "runs": {k: round(v, 2) for k, v in H.runs.items()}, "log": dict(LOG), "trace": REC.trace}
