# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Square a racked two-rail gantry beam by pushing both carriages onto their front end stops together.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- Two steel linear rails (12 x 8 mm section, 250 mm long, MGN12 pattern) lie along x on the table, 340 mm apart
  (y = +-0.170), from x 0.030 to x 0.280. A hard end-stop block (10 x 30 x 13 mm) ends each rail at x 0.280.
- A carriage block (45.4 x 27 x 13 mm, U-channel 12.5 mm over the 12 mm rail: 0.25 mm clearance a side) rides on each
  rail. A 20 x 20 x 400 mm aluminium beam lies across the two blocks, with a toolhead (40 x 50 x 40 mm) on its front.
  Beam, blocks and toolhead are 0.8 kg together; the belts are left out (loose).
- The beam starts racked: yawed -0.45 deg, so the left block is 2.7 mm ahead of the right one and both are cocked
  on their rails, the left front face 40 mm and the right 42.7 mm short of the stops.

Goal: both carriages against their front stops (a 0.05 mm feeler does not enter at either), the beam square; both
grippers let go.
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

# -- geometry (m) ------------------------------------------------------------------------------------------------
RAIL_Y = 0.170  # rails at y = +-RAIL_Y
RAIL_W, RAIL_H, RAIL_X0, STOP_X = 0.012, 0.008, 0.030, 0.280  # rail section, rear end, stop face
STOP_LEN, STOP_W, STOP_H = 0.010, 0.030, 0.013
BLOCK_L, BLOCK_W, BLOCK_H = 0.0454, 0.027, 0.013
CHANNEL_W = 0.0125  # clearance 0.25 mm a side on the 12 mm rail
WALL_W = (BLOCK_W - CHANNEL_W) / 2
BEAM_W, BEAM_H, BEAM_L = 0.020, 0.020, 0.400
RACK_DEG = -0.45  # initial beam yaw: the left (+y) end ahead by 2 x RAIL_Y x sin(0.45 deg) = 2.7 mm
GAP_L0 = 0.040  # left front face to its stop at the start
XC0 = STOP_X - BLOCK_L / 2 - GAP_L0 - RAIL_Y * math.sin(math.radians(-RACK_DEG))  # beam centre x

GREY, STEEL, ALU, ORANGE, RED = ((0.55, 0.57, 0.60), (0.62, 0.64, 0.68), (0.78, 0.80, 0.84), (0.95, 0.50, 0.10),
                                 (0.80, 0.10, 0.10))

_t = lk.Tree("rails", pos=(0.0, 0.0, TOP), fixed=True, color=STEEL)
_rail_shapes = []
for _s in (1, -1):
    _rail_shapes.append(lk.box((STOP_X - RAIL_X0, RAIL_W, RAIL_H), ((STOP_X + RAIL_X0) / 2, _s * RAIL_Y, RAIL_H / 2)))
    _rail_shapes.append(lk.box((STOP_LEN, STOP_W, STOP_H), (STOP_X + STOP_LEN / 2, _s * RAIL_Y, STOP_H / 2),
                               color=RED))
_t.link("rails", _rail_shapes, mass=2.0, friction=(0.2, 0.15), contact_offset=0.0004)
RAILS = _t.build()

_t = lk.Tree("gantry", pos=(XC0, 0.0, TOP), yaw_deg=RACK_DEG, color=ALU)
_shapes = [
    lk.box((BEAM_W, BEAM_L, BEAM_H), (0.0, 0.0, BLOCK_H + BEAM_H / 2)),  # 2020 extrusion on the two blocks
    lk.box((0.040, 0.050, 0.040), (0.030, 0.0, BLOCK_H + 0.020), color=ORANGE),  # toolhead on the front
]
for _s in (1, -1):
    _shapes.append(lk.box((BLOCK_L, BLOCK_W, BLOCK_H - RAIL_H), (0.0, _s * RAIL_Y, (RAIL_H + BLOCK_H) / 2), color=GREY))
    for _w in (1, -1):  # side walls, bottoms 1 mm above the table
        _shapes.append(lk.box((BLOCK_L, WALL_W, RAIL_H - 0.001),
                              (0.0, _s * RAIL_Y + _w * (CHANNEL_W / 2 + WALL_W / 2), 0.001 + (RAIL_H - 0.001) / 2),
                              color=GREY))
_t.link("beam", _shapes, mass=0.80, friction=(0.2, 0.15), contact_offset=0.0004)
GANTRY = _t.build()

OBJECTS = [RAILS, GANTRY]

# -- scoring -----------------------------------------------------------------------------------------------------
FRONT = BLOCK_L / 2
CORNERS = {"left": [(FRONT, RAIL_Y - BLOCK_W / 2, 0.0), (FRONT, RAIL_Y + BLOCK_W / 2, 0.0)],
           "right": [(FRONT, -RAIL_Y - BLOCK_W / 2, 0.0), (FRONT, -RAIL_Y + BLOCK_W / 2, 0.0)]}
BLOCK_TOP_LOCAL = {"left": (0.0, RAIL_Y, BLOCK_H), "right": (0.0, -RAIL_Y, BLOCK_H)}
MOVED_MM = 25.0  # each carriage came this far forward at some time
NEAR_MM = 5.0  # near the stop at the end
SQUARE_MM = 0.05  # the feeler gauge: worst gap over the front face at each stop
SEAT_MM = 1.0  # at the end each carriage block must sit on its rail: lifted less than this
STILL_S, RELEASE_S = 1.0, 1.0
STILL_SPEED = 0.003  # m/s
CLEAR_M, TCP_STILL = 0.030, 0.005
EPS = 1e-6


def measure_pose(P, prev=None, dt: float = 0.0) -> dict:
    """Worst front-face gap (mm) of each block to its stop, block heights above the rail (mm), beam speed (m/s)."""
    gaps, lift = {}, {}
    for side, pts in CORNERS.items():
        xs = [float((P @ np.array([*p, 1.0]))[0]) for p in pts]
        gaps[side] = (STOP_X - min(xs)) / MM  # the largest gap along the face is what a feeler finds
        top = P @ np.array([*BLOCK_TOP_LOCAL[side], 1.0])
        lift[side] = (float(top[2]) - TOP - BLOCK_H) / MM
    speed = 0.0
    if prev is not None and dt > 0.0:
        speed = float(np.linalg.norm(P[:3, 3] - prev[:3, 3])) / dt
    return {"gap": gaps, "lift": lift, "speed": speed, "yaw": math.degrees(math.atan2(P[1, 0], P[0, 0]))}


def measure(beam, tools=None, prev=None, dt: float = 0.0) -> dict:
    m = measure_pose(np.asarray(beam.poses["beam"], dtype=float), prev, dt)
    if tools is not None:
        m["clear"] = {s: lk.clearance(t, beam) for s, t in tools.items()}
        m["tcp_speed"] = {s: float(t.speed) for s, t in tools.items()}
    return m


class Record:
    """Whole-run record (a linkkit History updated every control step, cleared only by the simulator reset)."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0
        self.start: dict = {}
        self.trace: list = []

    def reset(self):
        self.H.reset()
        self.last, self.n, self.start, self.trace = None, 0, {}, []

    def update(self, m: dict, t: float | None = None):
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        for s in ("left", "right"):
            self.start.setdefault(s, m["gap"][s])
            H.peak(f"adv_{s}", self.start[s] - m["gap"][s])
            H.peak(f"lift_{s}", m["lift"][s])
        H.peak("speed", m["speed"])
        H.held("still", m["speed"] < STILL_SPEED)
        clear = m.get("clear")
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason: the rungs from the top (only the end state counts)."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    worst = max(m["gap"].values())
    if worst <= SQUARE_MM:
        ends = {"both carriage blocks seated on their rails": max(m["lift"].values()) < SEAT_MM,
                "beam still 1 s": H.runs.get("still", 0.0) >= STILL_S - EPS,
                "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
        if all(ends.values()):
            return 1.0, "both carriages on their stops, beam square, grippers let go"
        return 0.75, "squared at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
    if worst <= NEAR_MM:
        return 0.5, f"both carriages within {NEAR_MM} mm of their stops at the end (worst {worst:.2f} mm)"
    if H.peaks.get("adv_left", 0.0) >= MOVED_MM and H.peaks.get("adv_right", 0.0) >= MOVED_MM:
        return 0.25, "both carriages came 25 mm forward at some time, not at the stops at the end"
    return 0.0, (f"the carriages did not both come 25 mm forward (best {H.peaks.get('adv_left', 0):.1f}, "
                 f"{H.peaks.get('adv_right', 0):.1f} mm)")


REC = Record()
_PREV: dict = {}


@lk.on_reset
def _reset(tick):
    REC.reset()
    _PREV.clear()


@lk.on_step
def _track(tick):
    beam = tick.states["gantry"]
    P = np.array(beam.poses["beam"], dtype=float)
    dt = tick.t - _PREV["t"] if _PREV else 0.0
    m = measure(beam, tick.tools, _PREV.get("P"), dt)
    _PREV.update(P=P, t=tick.t)
    REC.update(m)
    if REC.n % 10 == 0:
        REC.trace.append((round(tick.t, 1), round(m["gap"]["left"], 2), round(m["gap"]["right"], 2),
                          round(m["yaw"], 3), round(1000 * m["speed"], 1)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"reason": ladder(REC)[1], "peaks": dict(REC.H.peaks), "last": REC.last, "trace": REC.trace}


# -- scripted demo -----------------------------------------------------------------------------------------------
# Reads only what a camera sees: the beam's pose (block front faces) and the grippers.
VARIANTS = ("", "nothing", "left_only", "right_only", "one_then_other")
GRIP_Z = float(os.environ.get("GRIP_Z", 0.036))  # TCP above the table: pads cover the upper beam (tips 13 mm below)
GRIP_OPEN, GRIP_SQUEEZE = 0.040, 0.0170
PUSH_SPEED = 0.008  # m/s
OVERSHOOT = 0.004  # commanded past the stop: the arm presses the carriage on (its compliance leaves 3 to 4 mm)
HOVER = 0.07


def variant_stalls():
    return VARIANT in ("left_only", "right_only", "one_then_other")


def _push(ctx, side, dist, duration):
    """One arm: pinch the beam above its carriage, push ``dist`` forward, hold, let go and back away."""
    import linkdemo as ld

    arm = ctx.arm(side)
    rot = ld.down(90.0)  # pads close along x
    s = 1.0 if side == "left" else -1.0
    p = ctx.observe().point("gantry", "beam", (0.0, s * RAIL_Y, 0.0))
    yield from arm.move(p + np.array([0.0, 0.0, HOVER + GRIP_Z]), rot, width=GRIP_OPEN, duration=1.5)
    grasp = np.array([p[0], p[1], TOP + GRIP_Z])
    yield from arm.move(grasp, rot, width=GRIP_OPEN, duration=1.2)
    yield from arm.grip(GRIP_SQUEEZE, 0.8)
    yield from arm.wait(0.3)
    yield from arm.move(grasp + np.array([dist, 0.0, 0.0]), rot, duration=duration)
    for _ in range(3):  # closed loop on the front face seen at the stop: press on by what is left
        yield from arm.wait(0.6)
        left = measure_pose(ctx.observe().pose("gantry", "beam"))["gap"][side] * MM
        if left <= SQUARE_MM * MM or variant_stalls():
            break
        yield from arm.move(arm.pos + np.array([left + OVERSHOOT, 0.0, 0.0]), rot, duration=max(0.8, left / PUSH_SPEED))
    yield from arm.wait(1.0)
    yield from arm.open(0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, HOVER]), rot, duration=1.0)


def demo_script(ctx):
    variant = VARIANT
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    gap = measure_pose(ctx.observe().pose("gantry", "beam"))["gap"]
    dist = {s: gap[s] * MM + OVERSHOOT for s in gap}
    dur = max(dist.values()) / PUSH_SPEED
    gens = {s: _push(ctx, s, dist[s], dur) for s in ("left", "right")}
    if variant == "left_only":
        yield from gens["left"]
    elif variant == "right_only":
        yield from gens["right"]
    elif variant == "one_then_other":
        yield from gens["left"]
        yield from gens["right"]
    else:
        yield from ctx.both(gens["left"], gens["right"])
    yield from ctx.wait(2.0)
