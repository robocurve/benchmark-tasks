"""On the station table: seven loose aluminium blocks, all 50 mm long and 20 mm tall, whose widths differ by a few
millimetres; a go/no-go gap gauge; a green tray and a red tray.

The blocks lie flat in two rows in front of the arms: four along y = +0.30 at x 0.16, 0.25, 0.34 and 0.43, three
along y = -0.30 at x 0.16, 0.25 and 0.34; each with its 50 mm length along x and its width along y. Their specified
width is 30.0 +- 3.0 mm. Which block starts in which of the seven places is shuffled by the layout number.

The gauge sits on the midline nearest the arms, centred at x 0.13: a dark grey steel base 100 mm (x) by 160 mm (y)
by 10 mm tall, with two gaps across it, each between a pair of anvils 10 mm (x) by 20 mm (y) by 15 mm tall. The
green GO gap, centred at y = +0.04, is 33.0 mm wide (the upper limit); the red NO-GO gap, centred at y = -0.04,
is 27.0 mm wide (the lower limit). A block is in tolerance when it drops into the GO gap and does not drop into
the NO-GO gap.

The two trays lie across the midline, open on top, floors 5 mm thick and walls 25 mm tall: the red reject tray
centred at x 0.24 and the green pass tray centred at x 0.35, each 80 mm (x) by 220 mm (y) inside.

Run it:  GAUGE_LAYOUT_SEED=3 inspect-robots run --embodiment yam_arena -E task=gauge_sort.py -E object_jitter=0 \\
           --instruction "..." --scorer task_score ...
(GAUGE_LAYOUT_SEED, default 0, picks the layout; the embodiment accepts no task-specific -E options and does not
pass the run seed to task files.)
Self-test, no simulator:  python gauge_sort.py
"""

from __future__ import annotations

import hashlib
import math
import os

import numpy as np

from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

# -- parts ---------------------------------------------------------------------------------------------------
NOMINAL, TOL = 0.0300, 0.0030
LOWER, UPPER = NOMINAL - TOL, NOMINAL + TOL
PART_LEN, PART_H = 0.050, 0.020
ALUMINIUM = (0.76, 0.77, 0.79)
DENSITY = 2700.0  # kg/m^3

# name -> width along y (m). Three in tolerance (28.5 to 30.5 mm), two over (>= 34.5 mm), two under (<= 25.5 mm):
# every part clears or misses each gap by at least 1.5 mm in width (0.75 mm a side), and the tightest cases at NO-GO
# and over GO (28.5, 25.5, 34.5 mm) are in the batch. Every in-tolerance part has at least 2.0 mm in GO (1.0 mm a
# side; GO_MARGIN); the widest, 30.5 mm, has 2.5 mm. At 31.5 mm (0.75 mm a side) the demo's block 8 jammed in GO;
# block 8 (31.0 mm) has since been dropped from the batch.
MARGIN = 0.0015
GO_MARGIN = 0.0020
WIDTHS = {
    "part_1": 0.0305,  # pass
    "part_2": 0.0345,  # over
    "part_3": 0.0285,  # pass
    "part_4": 0.0255,  # under
    "part_5": 0.0350,  # over
    "part_6": 0.0295,  # pass
    "part_7": 0.0250,  # under
}
# The seven start places (x, y): four in front of the left arm, three in front of the right, all at x >= 0.16
# (nearer the arm bases, a pick tipped blocks up on end).
SLOTS = ((0.16, +0.30), (0.25, +0.30), (0.34, +0.30), (0.43, +0.30), (0.16, -0.30), (0.25, -0.30), (0.34, -0.30))
LAYOUT_ENV = "GAUGE_LAYOUT_SEED"


def layout(seed: int) -> dict[str, tuple[float, float]]:
    """Block name -> its start place (x, y) for this layout number: the seven places ordered by SHA-256 of
    "gauge-layout:<seed>:<place index>", and handed to part_1 .. part_7 in that order. Pure and portable, so a
    real-world trial can rebuild the same layout (the README lists layouts 0 to 9)."""
    order = sorted(range(len(SLOTS)), key=lambda i: hashlib.sha256(f"gauge-layout:{seed}:{i}".encode()).hexdigest())
    return {name: SLOTS[i] for name, i in zip(WIDTHS, order, strict=True)}


def _env_seed() -> int:
    raw = os.environ.get(LAYOUT_ENV, "").strip() or "0"
    seed = int(raw)
    if seed < 0:
        raise ValueError(f"{LAYOUT_ENV} must be a whole number >= 0, got {raw!r}")
    return seed


# name -> (width, x, y) of its start, for the layout in use (use_layout fills it).
PARTS: dict[str, tuple[float, float, float]] = {}
LAYOUT_SEED = 0


def in_tolerance(width: float) -> bool:
    return LOWER <= width <= UPPER


PASS = {n for n, w in WIDTHS.items() if in_tolerance(w)}
REJECT = set(WIDTHS) - PASS


def _part(name: str) -> Box:
    w, x, y = PARTS[name]
    return Box(
        name,
        pos=(x, y, TOP + PART_H / 2),
        size=(PART_LEN, w, PART_H),
        mass=round(DENSITY * PART_LEN * w * PART_H, 4),
        color=ALUMINIUM,
    )


# -- gauge (fixed) ---------------------------------------------------------------------------------------------
GAUGE_X = 0.13
BASE_H, ANVIL = 0.010, (0.010, 0.020, 0.015)
GAUGE_BASE = (0.100, 0.160, BASE_H)  # wide enough that the GO anvils (out to |y| 76.5 mm) sit on it
GO_Y, NOGO_Y = +0.04, -0.04
GO_GAP, NOGO_GAP = UPPER, LOWER
BASE_TOP = TOP + BASE_H
ANVIL_TOP = BASE_TOP + ANVIL[2]
GAPS = {"go": (GO_Y, GO_GAP), "nogo": (NOGO_Y, NOGO_GAP)}  # gap -> (centre y, width)
GREEN, RED, STEEL = (0.18, 0.60, 0.25), (0.78, 0.16, 0.14), (0.25, 0.26, 0.28)


def _anvils(prefix: str, cy: float, gap: float, color) -> list[Box]:
    off = gap / 2 + ANVIL[1] / 2
    z = TOP + BASE_H + ANVIL[2] / 2
    return [
        Box(f"{prefix}_anvil_{side}", pos=(GAUGE_X, cy + s * off, z), size=ANVIL, color=color)
        for side, s in (("left", +1), ("right", -1))
    ]


GAUGE = [
    Box("gauge_base", pos=(GAUGE_X, 0.0, TOP + BASE_H / 2), size=GAUGE_BASE, color=STEEL),
    *_anvils("go", GO_Y, GO_GAP, GREEN),
    *_anvils("nogo", NOGO_Y, NOGO_GAP, RED),
]

# -- trays (fixed) ---------------------------------------------------------------------------------------------
TRAY_IN = (0.080, 0.220)  # inside, x by y
FLOOR_H, WALL_T, WALL_H = 0.005, 0.006, 0.025  # wall height above the table
FLOOR_TOP = TOP + FLOOR_H
RIM = TOP + WALL_H
TRAYS = {"reject_tray": (0.24, RED), "pass_tray": (0.35, GREEN)}


def _tray(name: str, cx: float, color) -> list[Box]:
    ix, iy = TRAY_IN
    ox, oy = ix + 2 * WALL_T, iy + 2 * WALL_T
    wz = TOP + WALL_H / 2
    return [
        Box(f"{name}_floor", pos=(cx, 0.0, TOP + FLOOR_H / 2), size=(ox, oy, FLOOR_H), color=color),
        Box(f"{name}_wall_near", pos=(cx - (ix + WALL_T) / 2, 0.0, wz), size=(WALL_T, oy, WALL_H), color=color),
        Box(f"{name}_wall_far", pos=(cx + (ix + WALL_T) / 2, 0.0, wz), size=(WALL_T, oy, WALL_H), color=color),
        Box(f"{name}_wall_left", pos=(cx, (iy + WALL_T) / 2, wz), size=(ix, WALL_T, WALL_H), color=color),
        Box(f"{name}_wall_right", pos=(cx, -(iy + WALL_T) / 2, wz), size=(ix, WALL_T, WALL_H), color=color),
    ]


FIXED = [*GAUGE, *(b for name, (cx, color) in TRAYS.items() for b in _tray(name, cx, color))]
OBJECTS: list[Box] = []
SIDE = {"left": +1.0, "right": -1.0}
ROWS: dict[str, tuple[str, ...]] = {}  # arm -> the blocks in front of it, nearest first


def use_layout(seed: int) -> None:
    """Put the blocks in layout ``seed``'s places: PARTS, OBJECTS and ROWS change in place; gauge checks clear."""
    global LAYOUT_SEED
    LAYOUT_SEED = int(seed)
    where = layout(LAYOUT_SEED)
    PARTS.clear()
    PARTS.update({n: (w, *where[n]) for n, w in WIDTHS.items()})
    OBJECTS[:] = [*(_part(n) for n in PARTS), *FIXED]
    ROWS.clear()
    for arm, side in SIDE.items():
        ROWS[arm] = tuple(sorted((n for n, (_, _, y) in PARTS.items() if y * side > 0), key=lambda n: PARTS[n][1]))
    _CHECKS.reset()


# -- scoring ---------------------------------------------------------------------------------------------------
# A block earns credit only if it was checked with the gauge, in order, and ends in the right tray:
#   GO check: for CHECK_HOLD_STEPS control steps in a row (0.3 s at 20 Hz) the block is seated in the GO gap, or
#     rests on the GO anvils (flat across them or wedged between them) without reaching the base. Each reading must
#     be the one the block's width allows (only a narrower block can seat; only a wider one can rest on top), so a
#     narrow block held above the gap does not read "stays out".
#   NO-GO check: the same over the NO-GO gap, counted only after the block has passed GO. A block that stays out of
#     GO needs no NO-GO check.
# The first valid reading at each gap stands. Checks clear when a trial starts (every block lying still on the
# table within START_TOL of its start place), since a task file gets no reset call.
# A block is in a tray when its whole footprint is inside the tray's walls, its bottom is on the tray floor or on
# top of another block (within LAYER_TOL), and it is at rest: held above the floor or lying on a rim does not count.
CHECK_HOLD_STEPS = 6  # 0.3 s at the 20 Hz control rate
LAYER_TOL = 0.003
START_TOL = 0.035  # covers the embodiment's default 3 cm object jitter
OVER_GAP, SEATED_LOW, SEATED_HIGH = 0.015, 0.003, 0.008  # see seated()
ON_ANVILS = 0.003  # resting on the anvils: lowest point at most this far above the anvil tops
ESCAPE_CAP = 0.25  # the most a trial earns with any out-of-tolerance part in the green tray
ESCAPE_COST = 2  # an escape costs two correct parts' credit
FALSE_REJECT_COST = 2  # so does a false reject: with 3 in tolerance and 4 not, every part in the red tray scores 0


def seated(state, gap: str) -> bool:
    """Has this block dropped into this gap: lying flat on the gauge base, between the anvils? A block resting on
    the anvils, or jammed tilted between them, has not. (A seated block stands 5 mm proud of the anvil tops.)"""
    gy = GAPS[gap][0]
    return bool(
        abs(state.pos[0] - GAUGE_X) < OVER_GAP
        and abs(state.pos[1] - gy) < OVER_GAP
        and state.low[2] < BASE_TOP + SEATED_LOW
        and state.high[2] < ANVIL_TOP + SEATED_HIGH
    )


def stays_out(state, gap: str) -> bool:
    """Is this block resting on this gap's anvils, flat across them or wedged between them, still and clear of the
    base?"""
    gy = GAPS[gap][0]
    return bool(
        abs(state.pos[0] - GAUGE_X) < OVER_GAP
        and abs(state.pos[1] - gy) < OVER_GAP
        and state.resting()
        and BASE_TOP + SEATED_LOW <= state.low[2] <= ANVIL_TOP + ON_ANVILS
        and state.high[2] <= ANVIL_TOP + PART_H + ON_ANVILS
    )


def gauge_reading(name: str, state, gap: str) -> bool | None:
    """True: seated in the gap; False: stays out on its anvils; None: no reading (elsewhere, moving, or a pose the
    block's width cannot take on its own, e.g. a narrow block held up on top of the gap)."""
    narrower = WIDTHS[name] < GAPS[gap][1]
    if narrower and seated(state, gap):
        return True
    if not narrower and stays_out(state, gap):
        return False
    return None


class _Checks:
    """Each block's gauge readings so far this trial."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.go: dict[str, bool | None] = dict.fromkeys(WIDTHS)
        self.nogo: dict[str, bool | None] = dict.fromkeys(WIDTHS)
        self.run: dict[tuple[str, str], tuple[bool | None, int]] = {}  # (name, gap) -> (reading, steps in a row)
        self.last = None  # the states dict last seen: success() and score() share one update per step

    def verdict(self, name: str) -> str | None:
        """The tray this block's valid check sequence calls for, or None if the sequence is incomplete."""
        go, nogo = self.go[name], self.nogo[name]
        if go is False:
            return "reject_tray"
        if go is True and nogo is not None:
            return "reject_tray" if nogo else "pass_tray"
        return None


_CHECKS = _Checks()


def reset_checks() -> None:
    _CHECKS.reset()


def _at_start(objects) -> bool:
    for name, (_, x, y) in PARTS.items():
        s = objects[name]
        if not (s.resting() and abs(s.low[2] - TOP) < LAYER_TOL and math.hypot(s.pos[0] - x, s.pos[1] - y) < START_TOL):
            return False
    return True


def _update(objects) -> None:
    """Advance the gauge checks by one control step (once per states dict)."""
    if objects is _CHECKS.last:
        return
    if _at_start(objects):
        _CHECKS.reset()
        _CHECKS.last = objects
        return
    _CHECKS.last = objects
    for name in PARTS:
        state = objects[name]
        for gap, done in (("go", _CHECKS.go), ("nogo", _CHECKS.nogo)):
            if done[name] is not None or (gap == "nogo" and _CHECKS.go[name] is not True):
                _CHECKS.run.pop((name, gap), None)
                continue
            reading = gauge_reading(name, state, gap)
            prev, n = _CHECKS.run.get((name, gap), (None, 0))
            n = n + 1 if reading is not None and reading == prev else (1 if reading is not None else 0)
            _CHECKS.run[name, gap] = (reading, n)
            if reading is not None and n >= CHECK_HOLD_STEPS:
                done[name] = reading


def tray_of(name: str, objects) -> str | None:
    """'pass_tray', 'reject_tray' or None for one part."""
    s = objects[name]
    if not s.resting():
        return None
    on_floor = abs(s.low[2] - FLOOR_TOP) <= LAYER_TOL
    on_block = any(
        other != name
        and abs(s.low[2] - objects[other].high[2]) <= LAYER_TOL
        and s.low[0] < objects[other].high[0] and objects[other].low[0] < s.high[0]
        and s.low[1] < objects[other].high[1] and objects[other].low[1] < s.high[1]
        for other in PARTS
    )
    if not (on_floor or on_block):
        return None
    for tray, (cx, _) in TRAYS.items():
        if (cx - TRAY_IN[0] / 2 <= s.low[0] and s.high[0] <= cx + TRAY_IN[0] / 2
                and -TRAY_IN[1] / 2 <= s.low[1] and s.high[1] <= TRAY_IN[1] / 2):
            return tray
    return None


def tally(objects) -> tuple[int, int, int]:
    """(credited: checked in order and in the right tray, false rejects: in-tolerance parts in the red tray,
    escapes: out-of-tolerance parts in the green tray). Penalties apply whether or not a part was checked."""
    _update(objects)
    correct = false_rejects = escapes = 0
    for name in PARTS:
        tray = tray_of(name, objects)
        want = "pass_tray" if name in PASS else "reject_tray"
        if tray == want and _CHECKS.verdict(name) == want:
            correct += 1
        elif tray == "reject_tray" and want == "pass_tray":
            false_rejects += 1
        elif tray == "pass_tray" and want == "reject_tray":
            escapes += 1
    return correct, false_rejects, escapes


def success(objects) -> bool:
    """Every part checked in order and in the right tray, at rest."""
    return tally(objects)[0] == len(PARTS)


def score(objects) -> float:
    """(credited - 2 x false rejects - 2 x escapes) / 7, at least 0; at most 0.25 if any out-of-tolerance part is in
    the green tray (an escape). A part left anywhere else counts for nothing either way. (It also hands the states
    to the scripted demo, which reads the gauge from them; see _see.)"""
    _see(objects)
    correct, false_rejects, escapes = tally(objects)
    _log_poses(objects)  # dev only (GAUGE_POSE_LOG); after tally, so it shows this step's checks
    result = max(0.0, (correct - FALSE_REJECT_COST * false_rejects - ESCAPE_COST * escapes) / len(PARTS))
    return min(result, ESCAPE_CAP) if escapes else result


def distance(objects) -> float:
    """Mean horizontal distance (m) from each part to the centre of the tray it belongs in."""
    d = []
    for name in PARTS:
        cx = TRAYS["pass_tray" if name in PASS else "reject_tray"][0]
        d.append(float(np.hypot(objects[name].pos[0] - cx, objects[name].pos[1])))
    return float(np.mean(d))


# -- demo ------------------------------------------------------------------------------------------------------
# The scripted demo gauges every block and sorts it by what the gauge did; it never reads WIDTHS or the
# PASS/REJECT sets. For each block, one arm:
#   1. picks it up and sets it square over the GO gap, lets go and backs off;
#   2. looks: if the block lies seated in the gap, it takes it out and sets it over the NO-GO gap the same way;
#      if not (lying on the anvils or jammed between them), it takes it to the red tray;
#   3. after a NO-GO test, looks again: seated in NO-GO goes to the red tray, anything else to the green tray.
# "Looks" is the simulator's pose of that block (see seated()), the way an inspector sees whether a part sits down
# in the gap; yam_scripted's observations carry only images and joints, so the task file's score() hook, which
# runs after every control step, keeps the latest object states for the demo (_see). The waypoints after each
# look are _Lazy: each works out its target the first time the plan reaches it, from those states.
#
# Grip: straight down, turned 90 degrees so the fingers close along x on the block's 50 mm ends (left arm -90,
# right arm +90 deg, inside joint6's +-120 deg). The fingers then stand clear of the anvils, which are only 10 mm
# long along x (gauge x +- 5 mm), and can reach down beside a block that has dropped into a gap; a grip across
# the width could not, with 0.75 to 4 mm between the block and each anvil and 6.4 mm thick finger tips.
# The arms take turns at the gauge; the gauge is one arm's from its carry in until it lifts the block out after
# the last test. Every step has a fixed duration whatever the gauge shows, so the schedule is fixed.
TRAVEL_Z = TOP + 0.14
GRIP_ABOVE_BOTTOM = 0.014  # TCP above a block's bottom: the pad tips 1 mm above it
DROP_ABOVE_FLOOR = 0.030  # a block's bottom above the tray floor (or the block under it) when let go
GAUGE_DROP = 0.005  # a block's bottom above the anvil tops when let go over a gap
FINGER_AJAR = -0.034  # each finger 34 mm from the TCP: pads 68 mm apart, 9 mm clear of a block's ends
# Approaching and descending onto a block (from the table or the gauge): pads 88 mm apart, 19 mm clear of its ends.
# With only 9 mm, a pick near the arm bases (x 0.12) tipped blocks up on end. Releases keep FINGER_AJAR: the trays
# are only 80 mm long inside.
FINGER_WIDE = -0.044
FINGER_SHUT = 0.0
YAW = {"left": -math.pi / 2, "right": +math.pi / 2}
SLOTS_Y = (0.045, 0.085)  # each arm's spots across a tray (|y|), inner first; a third and fourth go on top
PARK = (0.20, 0.25, TRAVEL_Z)  # (x, |y|, z): where an arm waits when it has nothing to carry
GAUGE_TURNOVER_S = 5.0  # one arm leaves the gauge this long before the other starts in: its delivery is over

# Durations (s). FETCH: to the block, descend, close, lift. INTO_GO: carry, lower, settle, let go, back off, hold for the check.
# TEST: to the block on the gauge, descend, close, lift, carry, lower, settle, let go, back off, hold for the check.
# OUT: to the block, descend, close, lift (the gauge is free after this), carry, lower, let go, back off.
# Settle: before each let-go over a gap the arm holds still for SETTLE_S, fingers shut, so the arm's own motion has
# stopped. It does not centre the block. In simulation the shut fingers hold every block off-centre along x, about
# 23 mm from the TCP (towards the arm base for the left arm, away from it for the right); as the fingers open to
# let go, the block slides those 23 mm back under the TCP, and its y over the gap is set during that slide, after
# the settle. In the layout-3 runs (logs/poses-r1) the release y was off by -0.8 to +1.4 mm for the left arm and
# -1.3 to -2.6 mm for the right; at -2.6 mm block 6 (29.5 mm, 1.75 mm a side in GO) caught an anvil edge, rolled
# 28 degrees and wedged. RELEASE_DY corrects for this.
SETTLE_S = 0.5
# RELEASE_DY: added to the gap's centre y for each arm's release over GO and NO-GO: minus the mean y offset of the
# block at release, per arm, over all its releases in logs/poses-r1 (left: 8, GO and NO-GO, mean +0.17 mm; right:
# 3, GO only, mean -1.80 mm). One fixed constant per arm, the same for every block and both gaps; the scoring does
# not use it.
RELEASE_DY = {"left": -0.00017, "right": +0.00180}
FETCH = (1.5, 1.5, 0.8, 1.2)
# CHECK_S: after backing off, the arm holds clear of the block while the gauge check is read (the block then sits
# untouched from the let-go through the next reach and descent, well over CHECK_HOLD_STEPS).
CHECK_S = 0.6
INTO_GO = (2.0, 2.0, SETTLE_S, 1.0, 1.0, CHECK_S)
TEST = (1.0, 1.5, 0.8, 1.2, 1.5, 2.0, SETTLE_S, 1.0, 1.0, CHECK_S)
OUT = (1.0, 1.5, 0.8, 1.2, 1.8, 1.2, 0.6, 1.0)
AT_GAUGE_S = sum(INTO_GO) + sum(TEST) + sum(OUT[:4])
assert GAUGE_TURNOVER_S > sum(OUT[4:])

_SEEN: dict = {}  # the latest object states, from score()
_SENSE = None  # offline checks put their own block states here: a function name -> ObjectState


def _see(objects) -> None:
    _SEEN.clear()
    _SEEN.update(objects)


# Development only: with GAUGE_POSE_LOG=<dir> set, each control step appends one JSON line to
# <dir>/gauge_poses.jsonl ("step" counts from the demo's reset): per block its pose, its GO and NO-GO check status
# (recorded reading, current raw reading, candidate streak), its tray result with the conditions behind it, and its
# distances to the gauge base, anvil tops and tray floor (mm). Unset (the default), nothing is written.
_POSE_LOG = {"step": 0}


def _mm(v: float) -> float:
    return round(float(v) * 1000.0, 2)


def _tray_detail(name: str, objects) -> dict:
    """The tray_of() conditions for one block, with margins in mm (negative = over the wall / outside)."""
    s = objects[name]
    under = [o for o in PARTS if o != name
             and s.low[0] < objects[o].high[0] and objects[o].low[0] < s.high[0]
             and s.low[1] < objects[o].high[1] and objects[o].low[1] < s.high[1]]
    d = {
        "tray": tray_of(name, objects),
        "above_floor_mm": _mm(s.low[2] - FLOOR_TOP),
        "on_floor": bool(abs(s.low[2] - FLOOR_TOP) <= LAYER_TOL),
        "on_block": [o for o in under if abs(s.low[2] - objects[o].high[2]) <= LAYER_TOL],
        "above_blocks_mm": {o: _mm(s.low[2] - objects[o].high[2]) for o in under},
    }
    for tray, (cx, _) in TRAYS.items():
        d[tray] = {  # inside margin to each wall (mm); all >= 0 means the footprint is inside
            "near": _mm(s.low[0] - (cx - TRAY_IN[0] / 2)), "far": _mm(cx + TRAY_IN[0] / 2 - s.high[0]),
            "right": _mm(s.low[1] + TRAY_IN[1] / 2), "left": _mm(TRAY_IN[1] / 2 - s.high[1]),
        }
    return d


def _gap_detail(name: str, s, gap: str) -> dict:
    gy = GAPS[gap][0]
    reading, streak = _CHECKS.run.get((name, gap), (None, 0))
    return {
        "recorded": (_CHECKS.go if gap == "go" else _CHECKS.nogo)[name],  # True seated, False stays out, None not yet
        "now": gauge_reading(name, s, gap),
        "seated": seated(s, gap), "stays_out": stays_out(s, gap),
        "streak": streak, "streak_reading": reading,
        "dx_mm": _mm(s.pos[0] - GAUGE_X), "dy_mm": _mm(s.pos[1] - gy),
    }


def _log_poses(objects) -> None:
    out_dir = os.environ.get("GAUGE_POSE_LOG")
    if not out_dir:
        return
    try:
        import json
        from pathlib import Path

        path = Path(out_dir).expanduser() / "gauge_poses.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        row = {"step": _POSE_LOG["step"], "at_start": _at_start(objects)}
        for n in PARTS:
            s = objects[n]
            want = "pass_tray" if n in PASS else "reject_tray"
            tray = _tray_detail(n, objects)
            row[n] = {
                "pos": [round(float(v), 4) for v in s.pos], "quat_xyzw": [round(float(v), 4) for v in s.quat_xyzw],
                "low_z": round(float(s.low[2]), 4), "high_z": round(float(s.high[2]), 4),
                "speed_mm_s": _mm(np.linalg.norm(s.lin_vel)), "spin_rad_s": round(float(np.linalg.norm(s.ang_vel)), 3),
                "resting": s.resting(),
                "low_above_base_mm": _mm(s.low[2] - BASE_TOP), "low_above_anvils_mm": _mm(s.low[2] - ANVIL_TOP),
                "high_above_anvils_mm": _mm(s.high[2] - ANVIL_TOP),
                "go": _gap_detail(n, s, "go"), "nogo": _gap_detail(n, s, "nogo"),
                "verdict": _CHECKS.verdict(n), "want": want, "tray": tray,
                "credited": tray["tray"] == want and _CHECKS.verdict(n) == want,
            }
        with path.open("a") as f:
            f.write(json.dumps(row) + "\n")
        _POSE_LOG["step"] += 1
    except Exception as e:  # never let the dev log break a run, but say so once
        if not _POSE_LOG.get("warned"):
            _POSE_LOG["warned"] = True
            print(f"gauge_sort: GAUGE_POSE_LOG write failed: {e!r}")


def _sense(name: str):
    return _SENSE(name) if _SENSE is not None else _SEEN[name]


class _Lazy(Waypoint):
    """A waypoint whose target (position, finger) comes from ``resolve()`` the first time the plan reaches it.
    ArmPlan reads a waypoint's position only once its segment has begun, so ``resolve`` sees the latest states."""

    def __init__(self, duration: float, label: str, yaw: float, resolve) -> None:  # noqa: D107
        self.duration, self.label, self.yaw = duration, label, yaw
        self._resolve, self._got = resolve, None

    def _value(self):
        if self._got is None:
            self._got = self._resolve()
        return self._got

    @property
    def pos(self):
        return self._value()[0]

    @property
    def finger(self):
        return self._value()[1]

    def __repr__(self) -> str:
        return f"_Lazy({self.label!r}, {'resolved' if self._got else 'pending'})"

    __eq__ = object.__eq__
    __hash__ = object.__hash__


class _Inspector:
    """One episode's gauge readings and tray spots, each decided once, when the plan first needs it."""

    def __init__(self) -> None:
        self.readings: dict[tuple[str, str], tuple[bool, tuple[float, float, float]]] = {}
        self.spots: dict[str, tuple[str, tuple[float, float, float]]] = {}
        self.used = {(arm, tray): 0 for arm in ROWS for tray in TRAYS}

    def read(self, name: str, gap: str) -> tuple[bool, tuple[float, float, float]]:
        """(seated in this gap?, the TCP position to grip the block where it now lies)."""
        if (name, gap) not in self.readings:
            s = _sense(name)
            grip = (float(s.pos[0]), float(s.pos[1]), max(float(s.low[2]), BASE_TOP) + GRIP_ABOVE_BOTTOM)
            self.readings[name, gap] = (seated(s, gap), grip)
        return self.readings[name, gap]

    def spot(self, arm: str, name: str, tray: str) -> tuple[float, float, float]:
        """The TCP position to let this block go in this tray: the arm's next free spot."""
        if name not in self.spots:
            k = self.used[arm, tray]
            self.used[arm, tray] += 1
            y = SIDE[arm] * SLOTS_Y[k % len(SLOTS_Y)]
            z = FLOOR_TOP + (k // len(SLOTS_Y)) * PART_H + DROP_ABOVE_FLOOR + GRIP_ABOVE_BOTTOM
            self.spots[name] = (tray, (TRAYS[tray][0], y, z))
        return self.spots[name][1]

    def verdict(self, name: str) -> str | None:
        """'pass_tray' or 'reject_tray' once decided, else None."""
        return self.spots[name][0] if name in self.spots else None


def _over_gap(gap: str, arm: str) -> tuple[float, float, float]:
    """The TCP position to let a block go over a gap: the gap's centre, corrected by the arm's RELEASE_DY."""
    return (GAUGE_X, GAPS[gap][0] + RELEASE_DY[arm], ANVIL_TOP + GAUGE_DROP + GRIP_ABOVE_BOTTOM)


def _up(p) -> tuple[float, float, float]:
    return (p[0], p[1], TRAVEL_Z)


def _move(grip, dest, settle: bool = False) -> list[tuple]:
    """(pos, finger) for TEST/OUT: take the block at ``grip`` and let it go at ``dest``; with ``settle``, hold
    still at ``dest`` with the fingers shut before letting go (TEST)."""
    return [
        (_up(grip), FINGER_WIDE),
        (grip, FINGER_WIDE),
        (grip, FINGER_SHUT),
        (_up(grip), FINGER_SHUT),
        (_up(dest), FINGER_SHUT),
        (dest, FINGER_SHUT),
        *([(dest, FINGER_SHUT)] if settle else []),
        (dest, FINGER_AJAR),
        (_up(dest), FINGER_AJAR),
    ]


def _block_steps(arm: str, name: str, start, insp: _Inspector) -> tuple[list, list, list]:
    """One block's waypoints: (fetch, at the gauge, deliver). ``start`` is where it lies (x, y)."""
    yaw = YAW[arm]
    pick = (start[0], start[1], TOP + GRIP_ABOVE_BOTTOM)
    go = _over_gap("go", arm)
    fetch = [
        Waypoint(d, p, f, f"{name}: {lab}", yaw)
        for d, (p, f), lab in zip(
            FETCH,
            [(_up(pick), FINGER_WIDE), (pick, FINGER_WIDE), (pick, FINGER_SHUT), (_up(pick), FINGER_SHUT)],
            ("to the block", "descend", "close", "lift"),
            strict=True,
        )
    ]
    into_go = [
        Waypoint(d, p, f, f"{name}: {lab}", yaw)
        for d, (p, f), lab in zip(
            INTO_GO,
            [(_up(go), FINGER_SHUT), (go, FINGER_SHUT), (go, FINGER_SHUT), (go, FINGER_AJAR), (_up(go), FINGER_AJAR),
             (_up(go), FINGER_AJAR)],
            ("carry to GO", "lower over GO", "settle over GO", "let go over GO", "back off", "hold for the GO check"),
            strict=True,
        )
    ]

    memo: dict[str, list] = {}
    park = (PARK[0], SIDE[arm] * PARK[1], PARK[2])

    def test_plan() -> list:
        """After GO: seated -> into NO-GO; not seated -> red tray (then park)."""
        if "test" not in memo:
            went_in, grip = insp.read(name, "go")
            if went_in:
                nogo = _over_gap("nogo", arm)
                memo["test"] = _move(grip, nogo, settle=True) + [(_up(nogo), FINGER_AJAR)]
            else:  # the same steps, so the schedule is fixed: the hold is over the tray
                dest = insp.spot(arm, name, "reject_tray")
                memo["test"] = _move(grip, dest, settle=True) + [(_up(dest), FINGER_AJAR)]
        return memo["test"]

    def out_plan() -> list:
        """After NO-GO: seated -> red tray, not seated -> green tray. A block already rejected at GO: park."""
        if "out" not in memo:
            went_in, _ = insp.read(name, "go")
            if not went_in:
                memo["out"] = [(park, FINGER_AJAR)] * len(OUT)
            else:
                fell_through, grip = insp.read(name, "nogo")
                dest = insp.spot(arm, name, "reject_tray" if fell_through else "pass_tray")
                memo["out"] = _move(grip, dest)
        return memo["out"]

    test_labels = ("to the block on the gauge", "descend", "close", "lift", "carry", "lower", "settle", "let go",
                   "back off", "hold for the check")
    out_labels = ("to the block", "descend", "close", "lift", "carry to the tray", "lower", "let go", "back off")
    test = [_Lazy(d, f"{name}: test: {lab}", yaw, lambda i=i: test_plan()[i])
            for i, (d, lab) in enumerate(zip(TEST, test_labels, strict=True))]
    out = [_Lazy(d, f"{name}: out: {lab}", yaw, lambda i=i: out_plan()[i])
           for i, (d, lab) in enumerate(zip(OUT, out_labels, strict=True))]
    return fetch, into_go + test + out[:4], out[4:]


def _hold(arm: str, duration: float, at) -> Waypoint:
    return Waypoint(duration, at, FINGER_SHUT, "wait for the gauge", YAW[arm])


def demo_plan(insp: _Inspector | None = None) -> tuple[dict[str, list], _Inspector, list[tuple[str, float, float]]]:
    """(waypoints per arm, the inspector they read, the gauge schedule as (block, from s, to s))."""
    insp = insp or _Inspector()
    plans: dict[str, list] = {arm: [] for arm in ROWS}
    clock = {arm: 0.0 for arm in ROWS}
    free_at = -GAUGE_TURNOVER_S
    schedule = []
    # left 1, right 5, left 2, right 6, left 3, right 7, left 4: the right row is one shorter
    turns = [(arm, i) for i in range(4) for arm in ("left", "right") if i < len(ROWS[arm])]
    for arm, i in turns:
        name = ROWS[arm][i]
        _, x, y = PARTS[name]  # where it lies; not its width
        fetch, at_gauge, deliver = _block_steps(arm, name, (x, y), insp)
        plans[arm] += fetch
        clock[arm] += sum(w.duration for w in fetch)
        start = max(clock[arm], free_at + GAUGE_TURNOVER_S)
        if start > clock[arm]:
            plans[arm].append(_hold(arm, start - clock[arm], fetch[-1].pos))
        clock[arm] = start + AT_GAUGE_S
        free_at = clock[arm]
        schedule.append((name, start, free_at))
        plans[arm] += at_gauge + deliver
        clock[arm] += sum(w.duration for w in deliver)
    for arm in plans:  # done: back off to the park pose, clear of the trays
        plans[arm].append(Waypoint(1.5, (PARK[0], SIDE[arm] * PARK[1], PARK[2]), FINGER_AJAR, "park", YAW[arm]))
    return plans, insp, schedule


def demo():
    _SEEN.clear()
    _CHECKS.reset()
    _POSE_LOG["step"] = 0
    return demo_plan()[0]


use_layout(_env_seed())
__doc__ = f"{__doc__.rstrip()}\n\nLayout number {LAYOUT_SEED}.\n"

if __name__ == "__main__":
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import check_demo

    check_demo.main()
