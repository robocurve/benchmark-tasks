"""On the station table, in front of the left arm: a raised steel plate with a scrap trough along its far edge, a
loose blue workpiece standing on the plate, eight loose metal chips scattered around it, and a loose yellow bar to
use as a scraper.

The plate (fixed) is 280 mm (x) by 240 mm (y) by 15 mm tall, spanning x 0.12 to 0.40 and y 0.10 to 0.34. Beyond its
far edge (x 0.40) is a 40 mm wide scrap trough whose floor is the table top, 15 mm below the plate top. A red lip
10 mm thick and 30 mm tall (above the table) closes the trough's far side at x 0.44, and two 6 mm end walls of the
same height close its ends at y 0.10 and y 0.34.

On the plate, all loose: the yellow bar (aluminium, 20 mm (x) by 75 mm (y) by 20 mm tall, 81 g) lies along y at
x 0.15, y 0.22; the blue workpiece (acetal, 80 mm (x) by 60 mm (y) by 40 mm tall, 270 g) stands at x 0.30 and
y 0.15, 0.22 or 0.29; the eight chips (aluminium cubes, 15 mm, 9 g each) lie on a 25 mm grid of cells
(x 0.205 to 0.380, y 0.125 to 0.325), each at least 10 mm from the workpiece. The layout number picks the workpiece
spot and the chip cells.

The task: sweep every chip over the plate's far edge into the trough with the bar, never lifting a chip, and leave
the workpiece where it is.

Run it:  SWEEP_LAYOUT_SEED=2 inspect-robots run --embodiment yam_arena -E task=sweep_chips.py -E object_jitter=0 \\
           --instruction "..." --scorer task_score ...
(SWEEP_LAYOUT_SEED, default 0, picks the layout; the embodiment accepts no task-specific -E options and does not
pass the run seed to task files.)
Offline checks, no simulator:  python check_demo.py
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

# -- fixed: plate, trough lip, trough end walls -------------------------------------------------------------------
PLATE_H = 0.015
PT = TOP + PLATE_H  # plate top
PLATE_X = (0.12, 0.40)
PLATE_Y = (0.10, 0.34)
TROUGH_X = (0.40, 0.44)  # trough floor = the table top
LIP_T, LIP_H = 0.010, 0.030  # lip and end walls: height above the table (15 mm above the plate top)
WALL_T = 0.006
CY = sum(PLATE_Y) / 2
STEEL, LIP_C = (0.35, 0.36, 0.38), (0.55, 0.20, 0.15)

FIXED = [
    Box("plate", pos=(sum(PLATE_X) / 2, CY, TOP + PLATE_H / 2),
        size=(PLATE_X[1] - PLATE_X[0], PLATE_Y[1] - PLATE_Y[0], PLATE_H), color=STEEL),
    Box("lip", pos=(TROUGH_X[1] + LIP_T / 2, CY, TOP + LIP_H / 2),
        size=(LIP_T, PLATE_Y[1] - PLATE_Y[0] + 2 * WALL_T, LIP_H), color=LIP_C),
    Box("wall_near_y", pos=(sum(TROUGH_X) / 2, PLATE_Y[0] - WALL_T / 2, TOP + LIP_H / 2),
        size=(TROUGH_X[1] - TROUGH_X[0], WALL_T, LIP_H), color=LIP_C),
    Box("wall_far_y", pos=(sum(TROUGH_X) / 2, PLATE_Y[1] + WALL_T / 2, TOP + LIP_H / 2),
        size=(TROUGH_X[1] - TROUGH_X[0], WALL_T, LIP_H), color=LIP_C),
]

# -- loose: bar, workpiece, chips ---------------------------------------------------------------------------------
# The bar is 75 mm long: three 25 mm chip pitches, so a lane's ends can fall in the middle of the 10 mm gaps between
# chip rows. check_demo.py compares 70, 75, 80 and 100 mm; 100 mm cannot fit beside the middle workpiece spot.
BAR = (0.020, 0.075, 0.020)  # x, y, z: long along y
BAR_START = (0.15, CY)
BAR_MASS = 0.081  # aluminium, 2.7 g/cm^3
WP = (0.080, 0.060, 0.040)
WP_X = 0.30
WP_YS = (0.15, 0.22, 0.29)
WP_MASS = 0.27  # acetal, 1.41 g/cm^3
CHIP = 0.015
H = CHIP / 2
CHIP_MASS = 0.009  # aluminium
N_CHIPS = 8
CHIP_NAMES = [f"chip_{i}" for i in range(1, N_CHIPS + 1)]
# Kept exactly as the probe computed them (not rounded): the layout hash uses each cell's Python repr.
CELLS = [(0.205 + 0.025 * i, 0.125 + 0.025 * j) for i in range(8) for j in range(9)]
LAYOUT_ENV = "SWEEP_LAYOUT_SEED"
YELLOW, BLUE, ALU = (0.85, 0.65, 0.15), (0.20, 0.40, 0.75), (0.78, 0.79, 0.80)


def _h(*parts) -> str:
    return hashlib.sha256(":".join(map(str, parts)).encode()).hexdigest()


def gap_to_wp(x: float, y: float, wy: float) -> float:
    """Gap (m) between a chip footprint centred at (x, y) and the workpiece footprint at (WP_X, wy); < 0 overlaps."""
    dx = abs(x - WP_X) - (WP[0] + CHIP) / 2
    dy = abs(y - wy) - (WP[1] + CHIP) / 2
    return max(dx, dy)


def layout(seed: int) -> tuple[float, dict[str, tuple[float, float]]]:
    """(workpiece y, {chip: (x, y)}) for a layout number. The workpiece spot is WP_YS[int(SHA-256("sweep-wp:N")) % 3].
    The free cells are those at least 10 mm from the workpiece. For attempt = 0, 1, 2, ...: order the free cells by
    SHA-256("sweep-chip:N:attempt:(x, y)") and take the first eight; accept the first draw with at least two chips
    within 25 mm of a workpiece face and at least one in its shadow (between it and the trough, |y - wy| < 30 mm).
    Chips 1 to 8 take the eight cells in that order."""
    wy = WP_YS[int(_h("sweep-wp", seed), 16) % 3]
    free = [c for c in CELLS if gap_to_wp(*c, wy) >= 0.010]
    for attempt in range(1000):
        order = sorted(free, key=lambda c: _h("sweep-chip", seed, attempt, c))[:N_CHIPS]
        near = sum(gap_to_wp(*c, wy) <= 0.025 for c in order)
        shadow = sum(c[0] > WP_X + WP[0] / 2 and abs(c[1] - wy) < WP[1] / 2 for c in order)
        if near >= 2 and shadow >= 1:
            return wy, dict(zip(CHIP_NAMES, order, strict=True))
    raise RuntimeError(f"no layout for seed {seed}")


def _env_seed() -> int:
    raw = os.environ.get(LAYOUT_ENV, "").strip() or "0"
    seed = int(raw)
    if seed < 0:
        raise ValueError(f"{LAYOUT_ENV} must be a whole number >= 0, got {raw!r}")
    return seed


SEED = 0
WP_Y = WP_YS[0]
CHIPS: dict[str, tuple[float, float]] = {}
OBJECTS: list = []


def use_layout(seed: int) -> None:
    """Switch to layout ``seed``: SEED, WP_Y, CHIPS and OBJECTS change in place; the scoring state clears."""
    global SEED, WP_Y
    SEED = int(seed)
    WP_Y, chips = layout(SEED)
    CHIPS.clear()
    CHIPS.update(chips)
    OBJECTS[:] = [
        Box("bar", pos=(*BAR_START, PT + BAR[2] / 2), size=BAR, mass=BAR_MASS, color=YELLOW),
        Box("workpiece", pos=(WP_X, WP_Y, PT + WP[2] / 2), size=WP, mass=WP_MASS, color=BLUE),
        *(Box(n, pos=(x, y, PT + CHIP / 2), size=(CHIP,) * 3, mass=CHIP_MASS, color=ALU) for n, (x, y) in CHIPS.items()),
        *FIXED,
    ]
    _S.reset()
    _PLAN_CACHE.clear()


# -- scoring ---------------------------------------------------------------------------------------------------------
# A chip is credited when it ends in the trough and was never lifted and never moved except by the bar:
#   in the trough: its whole footprint (world-aligned bounding box, no tolerance) inside the trough (x 0.40-0.44,
#     y 0.10-0.34), still, and either its bottom at least LAYER_TOL below the plate top (fallen in: on the floor,
#     propped or leaning) or resting on another chip that is itself in the trough (within LAYER_TOL);
#   lifted (any time): its bottom more than LIFT_TOL above the plate top, unless it rests on a chip that is itself
#     down (on the plate, or riding on such a chip) or it is over the trough, its lip or its end walls. Chips can only reach the trough over
#     the plate edge without rising that high: the lip and end walls stand 15 mm above the plate top.
#   moved without the bar (any time): over the plate and moving (not resting()), while neither within NEAR_BAR of the
#     bar nor touching (within TOUCH) a moving chip that is itself bar-driven. Against pushing chips with the
#     fingertips; the chain lets a chip pushed by a chip the bar pushes keep its credit (the probe run had chains
#     up to 15 mm from the bar).
# Workpiece factor: 1.0 if it ends within 5 mm and 3 degrees of its start; 0.5 if within 15 mm; 0 if further, or if at
# any time it moved more than 15 mm, was lifted more than LIFT_TOL, tipped more than 5 degrees or left the plate.
# Score = credited chips / 8 x workpiece factor. The state clears when a trial's start is seen (every chip and the
# workpiece still, at their start places), since a task file gets no reset call.
LIFT_TOL = 0.004
LAYER_TOL = 0.003
NEAR_BAR = 0.010
TOUCH = 0.002
START_TOL = 0.003
WP_OK, WP_HALF = 0.005, 0.015
WP_YAW_OK = math.radians(3.0)
WP_TILT_MAX = 5.0  # degrees


def _yaw(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _yaw_off(q) -> float:
    """Turn about z from the start, folded to [0, 90] degrees: a box turned 180 degrees has the same footprint."""
    a = abs(_yaw(q)) % math.pi
    return min(a, math.pi - a)


def _tilt_deg(q) -> float:
    x, y, z, w = q
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


def _gap(a, b) -> float:
    """Distance (m) between two objects' world-aligned bounding boxes; 0 when they touch or overlap."""
    d = np.maximum(0.0, np.maximum(np.asarray(a.low) - np.asarray(b.high), np.asarray(b.low) - np.asarray(a.high)))
    return float(np.linalg.norm(d))


def _xy_overlap(a, b) -> bool:
    return bool(a.low[0] < b.high[0] and b.low[0] < a.high[0] and a.low[1] < b.high[1] and b.low[1] < a.high[1])


def _rests_on(a, b) -> bool:
    return bool(abs(a.low[2] - b.high[2]) <= LAYER_TOL and _xy_overlap(a, b))


def _over_plate(s) -> bool:
    return bool(PLATE_X[0] <= s.pos[0] <= PLATE_X[1] and PLATE_Y[0] <= s.pos[1] <= PLATE_Y[1])


def _over_trough(s) -> bool:
    """Over the trough, its lip or its end walls (where a chip pushed over the edge can fall or bounce)."""
    return bool(TROUGH_X[0] <= s.pos[0] <= TROUGH_X[1] + LIP_T
                and PLATE_Y[0] - WALL_T <= s.pos[1] <= PLATE_Y[1] + WALL_T)


def _footprint_in_trough(s) -> bool:
    return bool(TROUGH_X[0] <= s.low[0] and s.high[0] <= TROUGH_X[1]
                and PLATE_Y[0] <= s.low[1] and s.high[1] <= PLATE_Y[1])


def in_trough(objects) -> set[str]:
    """Chips in the trough: whole footprint inside, still, and either below the plate top by LAYER_TOL or more
    (fallen in) or resting on a chip that is itself in the trough."""
    cand = [n for n in CHIP_NAMES if _footprint_in_trough(objects[n]) and objects[n].resting()]
    got = {n for n in cand if objects[n].low[2] <= PT - LAYER_TOL}  # fallen below the plate top
    grew = True
    while grew:
        grew = False
        for n in cand:
            if n not in got and any(_rests_on(objects[n], objects[m]) for m in got):
                got.add(n)
                grew = True
    return got


def _down(objects) -> set[str]:
    """Chips not raised: bottom within LIFT_TOL of the plate top or lower, or resting on a chip that is down."""
    got = {n for n in CHIP_NAMES if objects[n].low[2] <= PT + LIFT_TOL}
    grew = True
    while grew:
        grew = False
        for n in CHIP_NAMES:
            if n not in got and any(_rests_on(objects[n], objects[m]) for m in got):
                got.add(n)
                grew = True
    return got


def bar_driven(objects) -> set[str]:
    """Moving chips within NEAR_BAR of the bar, and moving chips touching one of those (transitively)."""
    moving = [n for n in CHIP_NAMES if not objects[n].resting()]
    bar = objects["bar"]
    got = {n for n in moving if _gap(objects[n], bar) <= NEAR_BAR}
    grew = True
    while grew:
        grew = False
        for n in moving:
            if n not in got and any(_gap(objects[n], objects[m]) <= TOUCH for m in got):
                got.add(n)
                grew = True
    return got


class _State:
    """One trial's history: per chip the first step it was lifted / moved without the bar (None: never), and the
    workpiece's worst shift and tilt so far. (Plain class: task files are loaded without a sys.modules entry, which
    dataclasses need.)"""

    def __init__(self) -> None:
        self.step = 0
        self.last = None
        self.reset()

    def reset(self) -> None:
        self.lifted = dict.fromkeys(CHIP_NAMES)
        self.unbarred = dict.fromkeys(CHIP_NAMES)
        self.wp_max_shift = self.wp_max_tilt = 0.0
        self.wp_lifted = self.wp_left_plate = False


_S = _State()


def _wp_shift(w) -> float:
    return math.hypot(w.pos[0] - WP_X, w.pos[1] - WP_Y)


def at_start(objects) -> bool:
    """Every chip and the workpiece lying still at their start places (a trial's start)."""
    w = objects["workpiece"]
    if not (w.resting() and _wp_shift(w) < START_TOL and abs(w.low[2] - PT) < LAYER_TOL):
        return False
    return all(objects[n].resting() and math.hypot(objects[n].pos[0] - x, objects[n].pos[1] - y) < START_TOL
               and abs(objects[n].low[2] - PT) < LAYER_TOL for n, (x, y) in CHIPS.items())


def _update(objects) -> None:
    """Advance the trial's state by one control step (once per states dict: success() and score() share it)."""
    if objects is _S.last:
        return
    _S.last = objects
    _S.step += 1
    if at_start(objects):
        _S.reset()
        return
    down = _down(objects)
    driven = bar_driven(objects)
    for n in CHIP_NAMES:
        s = objects[n]
        if _S.lifted[n] is None and n not in down and not _over_trough(s):
            _S.lifted[n] = _S.step
        if _S.unbarred[n] is None and _over_plate(s) and not s.resting() and n not in driven:
            _S.unbarred[n] = _S.step
    w = objects["workpiece"]
    _S.wp_max_shift = max(_S.wp_max_shift, _wp_shift(w))
    _S.wp_max_tilt = max(_S.wp_max_tilt, _tilt_deg(w.quat_xyzw))
    _S.wp_lifted |= bool(w.low[2] > PT + LIFT_TOL)
    _S.wp_left_plate |= bool(w.low[2] < PT - LAYER_TOL)


def wp_factor(objects) -> float:
    _update(objects)
    w = objects["workpiece"]
    if (_S.wp_max_shift > WP_HALF or _S.wp_max_tilt > WP_TILT_MAX or _S.wp_lifted or _S.wp_left_plate):
        return 0.0
    if _wp_shift(w) <= WP_OK and _yaw_off(w.quat_xyzw) <= WP_YAW_OK:
        return 1.0
    return 0.5


def credited(objects) -> list[str]:
    _update(objects)
    trough = in_trough(objects)
    return [n for n in CHIP_NAMES if n in trough and _S.lifted[n] is None and _S.unbarred[n] is None]


def success(objects) -> bool:
    """All eight chips credited and the workpiece where it started (factor 1.0)."""
    return len(credited(objects)) == N_CHIPS and wp_factor(objects) == 1.0


def score(objects) -> float:
    """Credited chips / 8, times the workpiece factor (1.0, 0.5 or 0)."""
    result = len(credited(objects)) / N_CHIPS * wp_factor(objects)
    _log(objects)
    return result


def distance(objects) -> float:
    """Mean distance (m) from each chip not yet in the trough to the plate's far edge."""
    trough = in_trough(objects)
    return float(np.mean([0.0 if n in trough else max(0.0, TROUGH_X[0] - objects[n].pos[0]) for n in CHIP_NAMES]))


# -- demo plan: open loop, from the layout alone ------------------------------------------------------------------
# One arm (left) holds the bar by its middle from above, the fingers closing along x across its 20 mm width, and the
# bar long along y the whole time (no wrist turn). Three kinds of pass, each from the layout's start positions:
#   side lane: lower the bar behind every chip (x 0.15), sweep it along +x past the plate edge (front face 5 mm into
#     the trough), lift. Every lane misses the workpiece in y by at least the plan's margin m, and every chip is either
#     fully inside the lane (each end at least m beyond the chip) or at least m clear of it.
#   end push: the chips no lane can reach (behind the workpiece or in its shadow, in its y band) are pushed along y
#     out of the band with the bar's end face, one x row at a time (the bar centred on the row), to just beyond the
#     band on one side; landing 4 mm short of the nearest chip of the row. Done after the side lanes, so the landing
#     spot is clear.
#   re-sweep: one more side lane over the pushed chips.
# The plan never reads the simulator's poses.
L = BAR[1]
M_MAX, M_MIN, M_STEP = 0.005, 0.0025, 0.00025  # margin search, metres
WALL_MIN = 0.0025  # lane end to the trough end walls' inner faces (the bar's front enters the trough by 5 mm)
LAND_GAP = 0.010
SCAN = 0.00025
WP_HY = WP[1] / 2


def _u(y: float, s: int) -> float:
    return s * (y - WP_Y)


def _u_edge(s: int) -> float:
    return PLATE_Y[1] - WP_Y if s > 0 else WP_Y - PLATE_Y[0]


def _beside(x: float) -> bool:
    return abs(x - WP_X) < WP[0] / 2 + H


class Plan:
    """m: the chip and workpiece margin of every lane (m); lanes and resweep: (centre y, chips, u_lo, side);
    pushes: (row x, side, start centre y, end centre y, chips); wall: smallest lane-end to end-wall clearance (m)."""

    def __init__(self, m, lanes, pushes, resweep, wall) -> None:
        self.m, self.lanes, self.pushes, self.resweep, self.wall = m, lanes, pushes, resweep, wall

    def __repr__(self) -> str:
        return f"Plan(m={self.m}, lanes={self.lanes}, pushes={self.pushes}, resweep={self.resweep}, wall={self.wall})"


def _side_lanes(chips: dict, s: int, m: float, bar_len: float):
    present = {n: _u(y, s) for n, (_, y) in chips.items()}
    targets = {n for n, u in present.items() if u - H >= WP_HY + 2 * m - 1e-9}
    lanes, eps = [], 1e-9
    lo_min, lo_cap = WP_HY + m, _u_edge(s) - WALL_MIN - bar_len
    while targets:
        first = min(targets, key=present.get)
        lo_max = min(present[first] - H - m, lo_cap)
        best = None
        for k in range(int(round((lo_max - lo_min) / SCAN)) + 1 if lo_max >= lo_min - eps else 0):
            lo = lo_min + k * SCAN
            hi = lo + bar_len
            inside = {n for n, u in present.items() if u - H >= lo + m - eps and u + H <= hi - m + eps}
            if first not in inside or any(n not in inside and u - H < hi + m - eps and u + H > lo - m + eps
                                          for n, u in present.items()):
                continue
            clear = [lo - WP_HY, _u_edge(s) - hi]
            for n, u in present.items():
                if n in inside:
                    clear.append(min(u - H - lo, hi - u - H))
                elif abs(u) < 0.2:
                    clear.append(lo - (u + H) if u < lo else u - H - hi)
            key = (len(inside), round(min(clear), 6))
            if best is None or key > best[0]:
                best = (key, lo, inside)
        if best is None:
            return None
        _, lo, inside = best
        lanes.append((WP_Y + s * (lo + bar_len / 2), sorted(inside), lo, s))
        for n in inside:
            present.pop(n)
        targets -= inside
    return lanes


def plan_at(chips: dict, m: float, bar_len: float = L) -> Plan | None:
    lanes = []
    for s in (-1, +1):
        side = _side_lanes(chips, s, m, bar_len)
        if side is None:
            return None
        lanes += side
    taken = {n for lane in lanes for n in lane[1]}
    rest = {n: c for n, c in chips.items() if n not in taken}
    if any(_beside(x) for x, _ in rest.values()):
        return None
    wall = min([_u_edge(s) - (lo + bar_len) for _, _, lo, s in lanes] or [1.0])
    if not rest:
        return Plan(m, lanes, [], None, wall)
    best = None
    for s in (+1, -1):
        lo_r = WP_HY + m
        if lo_r + bar_len > _u_edge(s) - WALL_MIN + 1e-9:
            continue
        u_f = lo_r + m
        rows: dict[float, list] = {}
        for n, (x, _) in rest.items():
            rows.setdefault(round(x, 4), []).append(n)
        pushes, total = [], 0.0
        for x, names in sorted(rows.items()):
            us = [_u(rest[n][1], s) for n in names]
            if len(names) * CHIP > lo_r + bar_len - m - u_f + 1e-9:
                pushes = None
                break
            u0 = min(us) - H - LAND_GAP
            pushes.append((x, s, WP_Y + s * (u0 - bar_len / 2), WP_Y + s * (u_f - bar_len / 2), sorted(names)))
            total += u_f - u0
        if pushes is None:
            continue
        resweep = (WP_Y + s * (lo_r + bar_len / 2), sorted(rest), lo_r, s)
        w = min(wall, _u_edge(s) - (lo_r + bar_len))
        if best is None or total < best[0]:
            best = (total, Plan(m, lanes, pushes, resweep, w))
    return best[1] if best else None


def make_plan(chips: dict | None = None, bar_len: float = L) -> Plan | None:
    """The plan with the largest margin m (M_MAX down to M_MIN) for these chip positions (default: the layout's)."""
    chips = CHIPS if chips is None else chips
    k = 0
    while M_MAX - k * M_STEP >= M_MIN - 1e-12:
        p = plan_at(chips, M_MAX - k * M_STEP, bar_len)
        if p is not None:
            return p
        k += 1
    return None


# -- demo waypoints --------------------------------------------------------------------------------------------------
YAW_L = -math.pi / 2  # fingers close along x, across the bar's 20 mm width
PAD_TIP = 0.013  # PAD_TIP_PAST_TCP
GRIP_TIP_ABOVE_BAR_BOTTOM = 0.014  # pad tips 14 mm up the 20 mm bar: 6 mm of grip, tips 2 mm over the chip tops
TCP_ABOVE_BAR_BOTTOM = GRIP_TIP_ABOVE_BAR_BOTTOM + PAD_TIP
SWEEP_CLEAR = 0.003  # bar bottom above the plate while sweeping or pushing
TRAVEL_CLEAR = 0.060  # bar bottom above the plate when travelling: 20 mm over the workpiece top
FINGER_APPROACH = -0.025  # pads 50 mm apart: 15 mm clear of the bar
FINGER_SHUT = 0.0
SWEEP_SPEED = 0.05  # m/s, mean over the sweep
PUSH_SPEED = 0.03
TRAVEL_SPEED = 0.15
BAR_END_X = TROUGH_X[0] + 0.015 - BAR[0] / 2  # bar front face 15 mm past the plate edge
SETTLE_END_S = 3.0


def _z(clear: float) -> float:
    return PT + clear + TCP_ABOVE_BAR_BOTTOM


def demo_waypoints(plan: Plan | None = None) -> list[Waypoint]:
    plan = plan or make_plan()
    if plan is None:
        raise RuntimeError(f"layout {SEED}: no sweep plan found")
    bx, by = BAR_START
    w: list[Waypoint] = [
        Waypoint(2.0, (bx, by, _z(0.06)), FINGER_APPROACH, "to the bar", YAW_L),
        Waypoint(1.5, (bx, by, _z(0.0)), FINGER_APPROACH, "descend", YAW_L),
        Waypoint(0.8, (bx, by, _z(0.0)), FINGER_SHUT, "close", YAW_L),
        Waypoint(1.2, (bx, by, _z(TRAVEL_CLEAR)), FINGER_SHUT, "lift", YAW_L),
    ]
    at = [bx, by]

    def travel(x, y, label):
        d = math.hypot(x - at[0], y - at[1])
        w.append(Waypoint(max(1.0, d / TRAVEL_SPEED), (x, y, _z(TRAVEL_CLEAR)), FINGER_SHUT, label, YAW_L))
        at[:] = [x, y]

    def lane(yc, label):
        travel(bx, yc, f"{label}: to the lane")
        w.extend([
            Waypoint(1.0, (bx, yc, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: lower", YAW_L),
            Waypoint(0.5, (bx, yc, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: settle", YAW_L),
            Waypoint((BAR_END_X - bx) / SWEEP_SPEED, (BAR_END_X, yc, _z(SWEEP_CLEAR)), FINGER_SHUT,
                     f"{label}: sweep", YAW_L),
            Waypoint(1.0, (BAR_END_X, yc, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: hold at edge", YAW_L),
            Waypoint(1.0, (BAR_END_X, yc, _z(TRAVEL_CLEAR)), FINGER_SHUT, f"{label}: lift", YAW_L),
        ])
        at[:] = [BAR_END_X, yc]

    for i, (yc, *_rest) in enumerate(plan.lanes, 1):
        lane(yc, f"lane {i}")
    for i, (x, _s, y0, y1, _names) in enumerate(plan.pushes, 1):
        label = f"push {i}"
        travel(x, y0, f"{label}: to the row")
        w.extend([
            Waypoint(1.0, (x, y0, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: lower", YAW_L),
            Waypoint(0.5, (x, y0, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: settle", YAW_L),
            Waypoint(max(1.0, abs(y1 - y0) / PUSH_SPEED), (x, y1, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: push",
                     YAW_L),
            Waypoint(0.5, (x, y1, _z(SWEEP_CLEAR)), FINGER_SHUT, f"{label}: hold", YAW_L),
            Waypoint(1.0, (x, y1, _z(TRAVEL_CLEAR)), FINGER_SHUT, f"{label}: lift", YAW_L),
        ])
        at[:] = [x, y1]
    if plan.resweep is not None:
        lane(plan.resweep[0], "re-sweep")
    travel(bx, by, "back")
    w.append(Waypoint(SETTLE_END_S, (bx, by, _z(TRAVEL_CLEAR)), FINGER_SHUT, "hold (chips settle)", YAW_L))
    return w


def demo():
    _S.reset()
    _S.last = None
    _S.step = 0
    _PLAN_CACHE.clear()
    return {"left": demo_waypoints()}


# -- dev pose log (SWEEP_POSE_LOG=<dir>; unset: nothing is written) -----------------------------------------------
_PLAN_CACHE: dict = {}


def _r(v, k=4):
    return [round(float(a), k) for a in v]


def _log(objects) -> None:
    out = os.environ.get("SWEEP_POSE_LOG")
    if not out:
        return
    try:
        if "plan" not in _PLAN_CACHE:
            from yam_arena import station
            from yam_arena.demo import ArmPlan

            _PLAN_CACHE["plan"] = ArmPlan(demo_waypoints(), station.BASES["left"])
        t = _S.step / 20.0
        tcp, _, finger, label = _PLAN_CACHE["plan"].target(t)
        b, wp = objects["bar"], objects["workpiece"]
        trough, driven, down = in_trough(objects), bar_driven(objects), _down(objects)
        row = {
            "step": _S.step, "t": t, "label": label, "plan_tcp": _r(tcp), "plan_finger": round(float(finger), 4),
            "at_start": at_start(objects),
            "bar": {"pos": _r(b.pos), "low_z_mm_above_plate": round((b.low[2] - PT) * 1000, 2),
                    "yaw_deg": round(math.degrees(_yaw(b.quat_xyzw)), 2), "tilt_deg": round(_tilt_deg(b.quat_xyzw), 2),
                    "minus_plan_tcp_mm": _r((np.asarray(b.pos) - tcp) * 1000, 2)},
            "workpiece": {"pos": _r(wp.pos), "shift_mm": round(_wp_shift(wp) * 1000, 2),
                          "max_shift_mm": round(_S.wp_max_shift * 1000, 2),
                          "yaw_deg": round(math.degrees(_yaw(wp.quat_xyzw)), 2),
                          "tilt_deg": round(_tilt_deg(wp.quat_xyzw), 2), "factor": wp_factor(objects)},
            "chips": {n: {"x": round(float(objects[n].pos[0]), 4), "y": round(float(objects[n].pos[1]), 4),
                          "low_above_plate_mm": round((objects[n].low[2] - PT) * 1000, 2),
                          "tilt_deg": round(_tilt_deg(objects[n].quat_xyzw), 1),
                          "speed_mm_s": round(float(np.linalg.norm(objects[n].lin_vel)) * 1000, 1),
                          "to_bar_mm": round(_gap(objects[n], b) * 1000, 1),
                          "down": n in down, "driven": n in driven, "lifted_at": _S.lifted[n],
                          "unbarred_at": _S.unbarred[n], "in_trough": n in trough} for n in CHIP_NAMES},
        }
        p = Path(out).expanduser() / "sweep_poses.jsonl"
        p.parent.mkdir(parents=True, exist_ok=True)
        with p.open("a") as f:
            f.write(json.dumps(row) + "\n")
    except Exception as e:  # never break a run, but say so once
        if not _PLAN_CACHE.get("warned"):
            _PLAN_CACHE["warned"] = True
            print(f"sweep_chips: SWEEP_POSE_LOG failed: {e!r}")


use_layout(_env_seed())
__doc__ = f"{__doc__.rstrip()}\n\nLayout number {SEED}.\n"

if __name__ == "__main__":
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import check_demo

    check_demo.main()
