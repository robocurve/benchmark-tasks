# SPDX-License-Identifier: MIT
# Copyright (c) 2026 the task authors
"""Square a racked two-rail gantry beam by pushing both carriages onto their front end stops together.

Station table; world frame in metres from the midpoint between the two arm bases, x forward, y to the robot's left,
z up; the table top is at z 0.7475.

- Two steel linear rails (12 x 8 mm section, 250 mm long, MGN12 pattern) lie along x on the table, 340 mm apart
  (y = +-0.170), from x 0.030 to x 0.280. Each rail has a 2 mm high, 2 mm deep groove down both sides (the ball
  groove of the real rail, simplified): foot 12 x 3 mm, neck 8 x 2 mm, head 12 x 3 mm. A hard end-stop block
  (10 x 30 x 13 mm) ends each rail at x 0.280.
- A captured carriage block (MGN12H size, 45.4 x 27 x 13 mm) rides on each rail: its top plate sits on the rail head,
  its side walls (7.25 mm thick, bottoms 3 mm above the table as the real H1) wrap the 12 mm rail with 0.25 mm
  clearance a side, and a 1.5 mm lip on each wall sits in the rail groove with 0.25 mm above and below. The lips lie
  under the rail head, so the beam cannot be lifted more than 0.25 mm off its rails: it can only be slid.
  A 20 x 20 x 400 mm aluminium beam lies across the two blocks, with a toolhead (40 x 50 x 40 mm) on its front.
  Beam, blocks and toolhead are 0.8 kg together; the belts are left out (loose).
- The beam starts racked: yawed -0.45 deg, so the left block is 2.7 mm ahead of the right one and both are cocked
  on their rails, the left front face 40 mm and the right 42.7 mm short of the stops.

Goal: both carriages against their front stops (face to face: every front corner within 0.05 mm of the stop face),
both blocks seated on their rails, the beam square; both grippers let go. Credit needs the beam to have been slid
along the rails, not lifted off them.
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
FOOT_H, NECK_W, NECK_H = 0.003, 0.008, 0.002  # rail profile: foot z 0..3, neck (groove) z 3..5, head z 5..8
STOP_LEN, STOP_W, STOP_H = 0.010, 0.030, 0.013
BLOCK_L, BLOCK_W, BLOCK_H = 0.0454, 0.027, 0.013
CHANNEL_W = 0.0125  # clearance 0.25 mm a side on the 12 mm rail
WALL_W = (BLOCK_W - CHANNEL_W) / 2
WALL_Z0 = 0.003  # wall bottoms 3 mm above the table (H1 of the real MGN12H)
FIT = 0.00025  # clearance of every captured face
LIP_Z0, LIP_H = FOOT_H + FIT, NECK_H - 2 * FIT  # lip 1.5 mm high in the 2 mm groove
LIP_TIP = NECK_W / 2 + FIT  # lip tip 0.25 mm off the rail neck
BEAM_W, BEAM_H, BEAM_L = 0.020, 0.020, 0.400
RACK_DEG = -0.45  # initial beam yaw: the left (+y) end ahead by 2 x RAIL_Y x sin(0.45 deg) = 2.7 mm
GAP_L0 = 0.040  # left front face to its stop at the start
XC0 = STOP_X - BLOCK_L / 2 - GAP_L0 - RAIL_Y * math.sin(math.radians(-RACK_DEG))  # beam centre x

GREY, STEEL, ALU, ORANGE, RED = ((0.55, 0.57, 0.60), (0.62, 0.64, 0.68), (0.78, 0.80, 0.84), (0.95, 0.50, 0.10),
                                 (0.80, 0.10, 0.10))

_t = lk.Tree("rails", pos=(0.0, 0.0, TOP), fixed=True, color=STEEL)
_rail_shapes = []
_rail_len, _rail_cx = STOP_X - RAIL_X0, (STOP_X + RAIL_X0) / 2
_head_h = RAIL_H - FOOT_H - NECK_H
for _s in (1, -1):
    _y = _s * RAIL_Y
    _rail_shapes.append(lk.box((_rail_len, RAIL_W, FOOT_H), (_rail_cx, _y, FOOT_H / 2)))  # foot
    _rail_shapes.append(lk.box((_rail_len, NECK_W, NECK_H), (_rail_cx, _y, FOOT_H + NECK_H / 2)))  # neck (groove)
    _rail_shapes.append(lk.box((_rail_len, RAIL_W, _head_h), (_rail_cx, _y, RAIL_H - _head_h / 2)))  # head
    _rail_shapes.append(lk.box((STOP_LEN, STOP_W, STOP_H), (STOP_X + STOP_LEN / 2, _y, STOP_H / 2), color=RED))
_t.link("rails", _rail_shapes, mass=2.0, friction=(0.2, 0.15), contact_offset=0.0004)
RAILS = _t.build()

_t = lk.Tree("gantry", pos=(XC0, 0.0, TOP), yaw_deg=RACK_DEG, color=ALU)
_shapes = [
    lk.box((BEAM_W, BEAM_L, BEAM_H), (0.0, 0.0, BLOCK_H + BEAM_H / 2)),  # 2020 extrusion on the two blocks
    lk.box((0.040, 0.050, 0.040), (0.030, 0.0, BLOCK_H + 0.020), color=ORANGE),  # toolhead on the front
]
_lip_in, _lip_out = LIP_TIP, CHANNEL_W / 2 + 0.0005  # lip runs from its tip into the wall (0.5 mm overlap)
for _s in (1, -1):
    _shapes.append(lk.box((BLOCK_L, BLOCK_W, BLOCK_H - RAIL_H), (0.0, _s * RAIL_Y, (RAIL_H + BLOCK_H) / 2), color=GREY))
    for _w in (1, -1):
        # side wall: bottom 3 mm above the table, beside the rail foot and head
        _shapes.append(lk.box((BLOCK_L, WALL_W, RAIL_H - WALL_Z0),
                              (0.0, _s * RAIL_Y + _w * (CHANNEL_W / 2 + WALL_W / 2), (WALL_Z0 + RAIL_H) / 2),
                              color=GREY))
        # retaining lip under the rail head, in the rail groove: this is what stops the beam lifting off
        _shapes.append(lk.box((BLOCK_L, _lip_out - _lip_in, LIP_H),
                              (0.0, _s * RAIL_Y + _w * (_lip_in + _lip_out) / 2, LIP_Z0 + LIP_H / 2), color=GREY))
_t.link("beam", _shapes, mass=0.80, friction=(0.2, 0.15), contact_offset=0.0004)
GANTRY = _t.build()

OBJECTS = [RAILS, GANTRY]

# -- scoring -----------------------------------------------------------------------------------------------------
FRONT = BLOCK_L / 2
Y_RAIL = {"left": RAIL_Y, "right": -RAIL_Y}
FRONT_CORNERS = {s: [(FRONT, y - BLOCK_W / 2, BLOCK_H / 2), (FRONT, y + BLOCK_W / 2, BLOCK_H / 2)]
                 for s, y in Y_RAIL.items()}
TOP_CORNERS = {s: [(sx * FRONT, y + sy * BLOCK_W / 2, BLOCK_H) for sx in (1, -1) for sy in (1, -1)]
               for s, y in Y_RAIL.items()}
CENTRES = {s: (0.0, y, 0.0) for s, y in Y_RAIL.items()}
MOVED_MM = 25.0  # each carriage was slid this far forward while engaged
NEAR_MM = 5.0  # near the stop at the end
SQUARE_MM = 0.05  # the feeler gauge: no front corner farther than this from the stop face
PRESS_MM = 0.10  # a corner may sit this far past the stop face (contact compression); more is not face to face
SEAT_MM = 0.5  # at the end each carriage block sits on its rail: every top corner within this of its seated height
# Valid rail engagement at one step: the retaining lips allow 0.25 mm of lift and 0.25 mm of lateral play, and a
# yaw of 0.63 deg between the blocks' ends; the limits below are just outside what the real parts allow.
ENGAGE_LIFT_MM = 1.0  # any top corner higher or lower than this from its seated height: off the rail
ENGAGE_DY_MM = 0.6  # block centre off the rail centreline by more than this: off the rail
ENGAGE_YAW_DEG = 0.7  # block yawed on its rail by more than this
BEYOND_MM = 0.5  # a front corner this far past the stop face: through the stop, not on the rail
REAR_MM = (STOP_X - RAIL_X0 - BLOCK_L) / MM + 0.5  # a front corner this far from the stop: behind the rail's end
SLIDE_TOL_MM = 2.0  # the forward travel made while engaged must make up the whole displacement within this
STILL_S, RELEASE_S = 1.0, 1.0
STILL_SPEED = 0.003  # m/s
CLEAR_M, TCP_STILL = 0.030, 0.005
EPS = 1e-6
SIDES = ("left", "right")


def measure_pose(P, prev=None, dt: float = 0.0) -> dict:
    """Per carriage block: signed gaps of the two front corners to the stop face (mm, negative is past the stop),
    worst top-corner height error from the seated height (mm, absolute), lateral offset from the rail centreline
    (mm), mean front-face x (m); the beam yaw (deg) and speed (m/s)."""

    def world(p):
        return P @ np.array([*p, 1.0])

    gmax, gmin, lift, dy, xf = {}, {}, {}, {}, {}
    for side in SIDES:
        xs = [float(world(p)[0]) for p in FRONT_CORNERS[side]]
        gaps = [(STOP_X - x) / MM for x in xs]
        gmax[side], gmin[side] = max(gaps), min(gaps)  # largest gap = what a feeler finds; smallest = deepest past
        xf[side] = sum(xs) / len(xs)
        lift[side] = max(abs(float(world(p)[2]) - TOP - BLOCK_H) / MM for p in TOP_CORNERS[side])
        dy[side] = (float(world(CENTRES[side])[1]) - Y_RAIL[side]) / MM
    speed = 0.0
    if prev is not None and dt > 0.0:
        speed = float(np.linalg.norm(P[:3, 3] - prev[:3, 3])) / dt
    return {"gmax": gmax, "gmin": gmin, "lift": lift, "dy": dy, "xf": xf, "speed": speed,
            "yaw": math.degrees(math.atan2(P[1, 0], P[0, 0]))}


def measure(beam, tools=None, prev=None, dt: float = 0.0) -> dict:
    m = measure_pose(np.asarray(beam.poses["beam"], dtype=float), prev, dt)
    if tools is not None:
        m["clear"] = {s: lk.clearance(t, beam) for s, t in tools.items()}
        m["tcp_speed"] = {s: float(t.speed) for s, t in tools.items()}
    return m


def engaged(m: dict, side: str) -> bool:
    """True while this carriage is on its rail: seated height, on the centreline, not yawed off, between the rail's
    rear end and the stop face. A lifted, shifted, rotated or through-the-stop block is off the rail."""
    return (m["lift"][side] <= ENGAGE_LIFT_MM and abs(m["dy"][side]) <= ENGAGE_DY_MM
            and abs(m["yaw"]) <= ENGAGE_YAW_DEG and m["gmin"][side] >= -BEYOND_MM and m["gmax"][side] <= REAR_MM)


class Record:
    """Whole-run record (a linkkit History updated every control step, cleared only by the simulator reset).

    ``slid`` is the forward travel of each front face summed only over steps where that carriage was engaged at
    both ends of the step. A beam lifted off its rails and set down further on adds nothing to it."""

    def __init__(self):
        self.H = lk.History()
        self.last = None
        self.n = 0
        self.xf0: dict = {}
        self.slid = {s: 0.0 for s in SIDES}
        self.prev_ok = {s: False for s in SIDES}
        self.prev_x: dict = {}
        self.trace: list = []

    def reset(self):
        self.H.reset()
        self.last, self.n, self.xf0, self.trace = None, 0, {}, []
        self.slid = {s: 0.0 for s in SIDES}
        self.prev_ok = {s: False for s in SIDES}
        self.prev_x = {}

    def update(self, m: dict, t: float | None = None):
        H = self.H
        if t is not None:
            H._tick(t)
        self.n += 1
        self.last = m
        for s in SIDES:
            ok = engaged(m, s)
            self.xf0.setdefault(s, m["xf"][s])
            if ok and self.prev_ok[s]:
                self.slid[s] += (m["xf"][s] - self.prev_x[s]) / MM
            self.prev_ok[s], self.prev_x[s] = ok, m["xf"][s]
            H.peak(f"slid_{s}", self.slid[s])
            H.peak(f"lift_{s}", m["lift"][s])
            H.ever(f"off_rail_{s}", not ok)
        H.peak("speed", m["speed"])
        H.held("still", m["speed"] < STILL_SPEED)
        clear = m.get("clear")
        H.held("released", bool(clear) and min(clear.values()) >= CLEAR_M and max(m["tcp_speed"].values()) < TCP_STILL)


def ladder(rec: Record) -> tuple[float, str]:
    """Score and the reason: the rungs from the top (only the end state counts, and only travel made by sliding)."""
    H, m = rec.H, rec.last
    if m is None:
        return 0.0, "no steps yet"
    peaks = {s: H.peaks.get(f"slid_{s}", 0.0) for s in SIDES}
    moved = all(peaks[s] >= MOVED_MM for s in SIDES)
    on_rail = all(engaged(m, s) for s in SIDES)
    disp = {s: (m["xf"][s] - rec.xf0[s]) / MM for s in SIDES}
    slid_all = all(rec.slid[s] >= disp[s] - SLIDE_TOL_MM for s in SIDES)  # the whole way was slid, not carried
    gmax, gmin = max(m["gmax"].values()), min(m["gmin"].values())
    if on_rail and slid_all:
        if gmax <= SQUARE_MM and gmin >= -PRESS_MM and max(m["lift"].values()) <= SEAT_MM:
            ends = {"beam still 1 s": H.runs.get("still", 0.0) >= STILL_S - EPS,
                    "both grippers 30 mm clear and still 1 s": H.runs.get("released", 0.0) >= RELEASE_S - EPS}
            if all(ends.values()):
                return 1.0, "both carriages slid onto their stops face to face, beam square, grippers let go"
            return 0.75, "squared and seated at the end; not yet: " + ", ".join(k for k, ok in ends.items() if not ok)
        if gmax <= NEAR_MM and gmin >= -NEAR_MM:
            return 0.5, (f"on the rails within {NEAR_MM} mm of the stops at the end, not face to face and seated "
                         f"(gaps {gmin:.2f} to {gmax:.2f} mm, lift {max(m['lift'].values()):.2f} mm)")
    if moved:
        why = "both carriages were slid 25 mm forward while on their rails"
        if not on_rail:
            why += ", but at the end a carriage is off its rail"
        elif not slid_all:
            why += ", but part of the travel was not made on the rails"
        return 0.25, why + ", not squared at the end"
    return 0.0, (f"the carriages were not both slid 25 mm forward on their rails (best {peaks['left']:.1f}, "
                 f"{peaks['right']:.1f} mm)")


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
        REC.trace.append((round(tick.t, 1), round(m["gmax"]["left"], 2), round(m["gmax"]["right"], 2),
                          round(m["yaw"], 3), round(1000 * m["speed"], 1), round(max(m["lift"].values()), 2)))


def score(objects) -> float:
    return ladder(REC)[0]


def success(objects) -> bool:
    return ladder(REC)[0] >= 1.0


def distance(objects) -> float:
    return 1.0 - ladder(REC)[0]


def demo():  # yam_scripted plays this: both arms hold the ready pose (the closed-loop demo is demo_script)
    return {}


def demo_report() -> dict:
    return {"reason": ladder(REC)[1], "peaks": dict(REC.H.peaks), "slid": dict(REC.slid), "last": REC.last,
            "trace": REC.trace}


# -- scripted demo -----------------------------------------------------------------------------------------------
# Reads only what a camera sees: the beam's pose (block front faces) and the grippers.
VARIANTS = ("", "nothing", "left_only", "right_only", "one_then_other", "lift_carry")
GRIP_Z = float(os.environ.get("GRIP_Z", 0.036))  # TCP above the table: pads cover the upper beam (tips 13 mm below)
GRIP_OPEN, GRIP_SQUEEZE = 0.040, 0.0170
PUSH_SPEED = 0.008  # m/s
OVERSHOOT = 0.004  # commanded past the stop: the arm presses the carriage on (its compliance leaves 3 to 4 mm)
HOVER = 0.07
ENTRY = 0.02  # entry height above the grasp: pause here, then a slow final descent (the wrist lags while moving)
LIFT_CARRY = 0.04  # the lift_carry cheat raises the beam this far, then tries to carry it onto the stops


def variant_stalls():
    return VARIANT in ("left_only", "right_only", "one_then_other")


def _grasp(ctx, side):
    """Hover over the beam above its carriage, settle, descend slowly, pinch. Returns (arm, rot, grasp point)."""
    import linkdemo as ld

    arm = ctx.arm(side)
    rot = ld.down(90.0)  # pads close along x
    s = 1.0 if side == "left" else -1.0
    p = ctx.observe().point("gantry", "beam", (0.0, s * RAIL_Y, 0.0))
    grasp = np.array([p[0], p[1], TOP + GRIP_Z])
    yield from arm.move(p + np.array([0.0, 0.0, HOVER + GRIP_Z]), rot, width=GRIP_OPEN, duration=1.5)
    yield from arm.move(grasp + np.array([0.0, 0.0, ENTRY]), rot, width=GRIP_OPEN, duration=1.0)
    yield from arm.wait(0.6)
    yield from arm.move(grasp, rot, width=GRIP_OPEN, duration=2.0)
    yield from arm.wait(0.6)
    yield from arm.grip(GRIP_SQUEEZE, 0.8)
    yield from arm.wait(0.3)
    return arm, rot, grasp


def _push(ctx, side, dist, duration):
    """One arm: pinch the beam above its carriage, push ``dist`` forward (the carriage slides on its rail), hold,
    let go and back away."""
    arm, rot, grasp = yield from _grasp(ctx, side)
    yield from arm.move(grasp + np.array([dist, 0.0, 0.0]), rot, duration=duration)
    for _ in range(3):  # closed loop on the front face seen at the stop: press on by what is left
        yield from arm.wait(0.6)
        left = measure_pose(ctx.observe().pose("gantry", "beam"))["gmax"][side] * MM
        if left <= SQUARE_MM * MM or variant_stalls():
            break
        yield from arm.move(arm.pos + np.array([left + OVERSHOOT, 0.0, 0.0]), rot, duration=max(0.8, left / PUSH_SPEED))
    yield from arm.wait(1.0)
    yield from arm.open(0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, HOVER]), rot, duration=1.0)


def _lift_carry(ctx, side, dist, duration):
    """The cheat: pinch the beam, lift it clear of the rails, carry it forward, set it down, let go."""
    arm, rot, grasp = yield from _grasp(ctx, side)
    up = grasp + np.array([0.0, 0.0, LIFT_CARRY])
    yield from arm.move(up, rot, duration=1.5)
    yield from arm.move(up + np.array([dist, 0.0, 0.0]), rot, duration=duration)
    yield from arm.move(grasp + np.array([dist, 0.0, 0.0]), rot, duration=1.5)
    yield from arm.wait(1.0)
    yield from arm.open(0.6)
    yield from arm.move(arm.pos + np.array([0.0, 0.0, HOVER]), rot, duration=1.0)


def demo_script(ctx):
    variant = VARIANT
    yield from ctx.wait(1.0)
    if variant == "nothing":
        yield from ctx.wait(8.0)
        return
    gap = measure_pose(ctx.observe().pose("gantry", "beam"))["gmax"]
    dist = {s: gap[s] * MM + OVERSHOOT for s in gap}
    dur = max(dist.values()) / PUSH_SPEED
    mover = _lift_carry if variant == "lift_carry" else _push
    gens = {s: mover(ctx, s, dist[s], dur) for s in SIDES}
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
