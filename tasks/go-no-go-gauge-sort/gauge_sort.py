"""On the station table: seven loose aluminium blocks, all 50 mm long and 20 mm tall, whose widths differ by a few
millimetres; a go/no-go gap gauge; a green tray and a red tray.

The blocks lie flat in two rows in front of the arms: four along y = +0.30 at x 0.16, 0.25, 0.34 and 0.43, three
along y = -0.30 at x 0.16, 0.25 and 0.34; each with its 50 mm length along x and its width along y. Their specified width is 30.0 +- 3.0 mm.

The gauge sits on the midline nearest the arms, centred at x 0.13: a dark grey steel base 100 mm (x) by 160 mm (y)
by 10 mm tall, with two gaps across it, each between a pair of anvils 10 mm (x) by 20 mm (y) by 15 mm tall. The
green GO gap, centred at y = +0.04, is 33.0 mm wide (the upper limit); the red NO-GO gap, centred at y = -0.04,
is 27.0 mm wide (the lower limit). A block is in tolerance when it drops into the GO gap and does not drop into
the NO-GO gap.

The two trays lie across the midline, open on top, floors 5 mm thick and walls 25 mm tall: the red reject tray
centred at x 0.24 and the green pass tray centred at x 0.35, each 80 mm (x) by 220 mm (y) inside.

Run it:  inspect-robots run --embodiment yam_arena -E task=gauge_sort.py -E object_jitter=0 \\
           --instruction "..." --scorer task_score ...
Self-test, no simulator:  python gauge_sort.py
"""

from __future__ import annotations

import math

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

# name -> (width along y in m, x, y) of its start. Three in tolerance (28.5 to 30.5 mm), two over (>= 34.5 mm), two
# under (<= 25.5 mm): every part clears or misses each gap by at least 1.5 mm in width (0.75 mm a side), and the
# tightest cases at NO-GO and over GO (28.5, 25.5, 34.5 mm) are in the batch. Every in-tolerance part has at least
# 2.0 mm in GO (1.0 mm a side; GO_MARGIN); the widest, 30.5 mm, has 2.5 mm. At 31.5 mm (0.75 mm a side) the demo's
# block 8 jammed in GO; block 8 (31.0 mm) has since been dropped from the batch.
MARGIN = 0.0015
GO_MARGIN = 0.0020
PARTS = {
    "part_1": (0.0305, 0.16, +0.30),  # pass
    "part_2": (0.0345, 0.25, +0.30),  # over
    "part_3": (0.0285, 0.34, +0.30),  # pass
    "part_4": (0.0255, 0.43, +0.30),  # under
    "part_5": (0.0350, 0.16, -0.30),  # over
    "part_6": (0.0295, 0.25, -0.30),  # pass
    "part_7": (0.0250, 0.34, -0.30),  # under
}


def in_tolerance(width: float) -> bool:
    return LOWER <= width <= UPPER


PASS = {n for n, (w, _, _) in PARTS.items() if in_tolerance(w)}
REJECT = set(PARTS) - PASS


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


OBJECTS = [
    *(_part(n) for n in PARTS),
    *GAUGE,
    *(b for name, (cx, color) in TRAYS.items() for b in _tray(name, cx, color)),
]


# -- scoring ---------------------------------------------------------------------------------------------------
# A part is in a tray when its centre is over the tray's inside, its lowest point is no more than 25 mm above
# the rim (a second layer of parts still counts; a part held over the tray does not), and it is at rest.
STACK_ALLOWANCE = 0.025
ESCAPE_CAP = 0.25  # the most a trial earns with any out-of-tolerance part in the green tray
ESCAPE_COST = 2  # an escape costs two correct parts' credit
FALSE_REJECT_COST = 2  # so does a false reject: with 3 in tolerance and 4 not, every part in the red tray scores 0


def tray_of(state) -> str | None:
    """'pass_tray', 'reject_tray' or None for one part's ObjectState."""
    if not state.resting() or state.low[2] > RIM + STACK_ALLOWANCE:
        return None
    x, y = state.pos[0], state.pos[1]
    for name, (cx, _) in TRAYS.items():
        if abs(x - cx) <= TRAY_IN[0] / 2 and abs(y) <= TRAY_IN[1] / 2:
            return name
    return None


def tally(objects) -> tuple[int, int, int]:
    """(correctly sorted, false rejects: in-tolerance parts in the red tray, escapes: out-of-tolerance parts in
    the green tray)."""
    correct = false_rejects = escapes = 0
    for name in PARTS:
        tray = tray_of(objects[name])
        want = "pass_tray" if name in PASS else "reject_tray"
        if tray == want:
            correct += 1
        elif tray == "reject_tray":
            false_rejects += 1
        elif tray == "pass_tray":
            escapes += 1
    return correct, false_rejects, escapes


def success(objects) -> bool:
    """Every part in the right tray and at rest."""
    return tally(objects)[0] == len(PARTS)


def score(objects) -> float:
    """(correct - 2 x false rejects - 2 x escapes) / 7, at least 0; at most 0.25 if any out-of-tolerance part is in the green
    tray (an escape). A part left on the table, on the gauge, on a rim or anywhere else counts for nothing either
    way. (It also hands the states to the scripted demo, which reads the gauge from them; see _see.)"""
    _see(objects)
    correct, false_rejects, escapes = tally(objects)
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
# The scripted demo gauges every block and sorts it by what the gauge did; it never reads PARTS' widths or the
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
SIDE = {"left": +1.0, "right": -1.0}
ROWS = {"left": ("part_1", "part_2", "part_3", "part_4"), "right": ("part_5", "part_6", "part_7")}
SLOTS_Y = (0.045, 0.085)  # each arm's spots across a tray (|y|), inner first; a third and fourth go on top
PARK = (0.20, 0.25, TRAVEL_Z)  # (x, |y|, z): where an arm waits when it has nothing to carry
GAUGE_TURNOVER_S = 5.0  # one arm leaves the gauge this long before the other starts in: its delivery is over
# seated(): centre within this of the gap's centre along x and y; lowest point within SEATED_LOW of the base;
# highest point less than SEATED_HIGH above the anvil tops (a seated block stands 5 mm proud of them).
OVER_GAP, SEATED_LOW, SEATED_HIGH = 0.015, 0.003, 0.008

# Durations (s). FETCH: to the block, descend, close, lift. INTO_GO: carry, lower, settle, let go, back off, look.
# TEST: to the block on the gauge, descend, close, lift, carry, lower, settle, let go, back off, look/park.
# OUT: to the block, descend, close, lift (the gauge is free after this), carry, lower, let go, back off.
# Settle: before each let-go over a gap the arm holds still, fingers shut. In simulation a carried block sways up
# to 7 mm across the gap while it is lowered and was still 2.5 mm off when let go, more than the 0.75 mm a side
# a 31.5 mm block had in GO; its edge caught an anvil and it jammed. The widest in-tolerance block is now 30.5 mm
# (1.25 mm a side).
SETTLE_S = 0.5
FETCH = (1.5, 1.5, 0.8, 1.2)
INTO_GO = (2.0, 2.0, SETTLE_S, 1.0, 1.0, 0.5)
TEST = (1.0, 1.5, 0.8, 1.2, 1.5, 2.0, SETTLE_S, 1.0, 1.0, 0.5)
OUT = (1.0, 1.5, 0.8, 1.2, 1.8, 1.2, 0.6, 1.0)
AT_GAUGE_S = sum(INTO_GO) + sum(TEST) + sum(OUT[:4])
assert GAUGE_TURNOVER_S > sum(OUT[4:])

_SEEN: dict = {}  # the latest object states, from score()
_SENSE = None  # offline checks put their own block states here: a function name -> ObjectState


def _see(objects) -> None:
    _SEEN.clear()
    _SEEN.update(objects)
    _log_poses(objects)


# Development only: with GAUGE_POSE_LOG=<dir> set, each control step appends the blocks' poses as one JSON line to
# <dir>/gauge_poses.jsonl ("step" counts from the demo's reset). Unset (the default), nothing is written.
_POSE_LOG = {"step": 0}


def _log_poses(objects) -> None:
    import os

    out_dir = os.environ.get("GAUGE_POSE_LOG")
    if not out_dir:
        return
    try:
        import json
        from pathlib import Path

        path = Path(out_dir).expanduser() / "gauge_poses.jsonl"
        path.parent.mkdir(parents=True, exist_ok=True)
        row = {"step": _POSE_LOG["step"]}
        for n in PARTS:
            s = objects[n]
            row[n] = {"pos": [round(float(v), 4) for v in s.pos], "quat_xyzw": [round(float(v), 4) for v in s.quat_xyzw],
                      "low_z": round(float(s.low[2]), 4), "high_z": round(float(s.high[2]), 4),
                      "resting": s.resting()}
        with path.open("a") as f:
            f.write(json.dumps(row) + "\n")
        _POSE_LOG["step"] += 1
    except Exception:  # never let the dev log break a run
        pass


def _sense(name: str):
    return _SENSE(name) if _SENSE is not None else _SEEN[name]


def seated(state, gap: str) -> bool:
    """Has this block dropped into this gap: lying flat on the gauge base, between the anvils? A block resting on
    the anvils, or jammed tilted between them, has not."""
    gy = GAPS[gap][0]
    return bool(
        abs(state.pos[0] - GAUGE_X) < OVER_GAP
        and abs(state.pos[1] - gy) < OVER_GAP
        and state.low[2] < BASE_TOP + SEATED_LOW
        and state.high[2] < ANVIL_TOP + SEATED_HIGH
    )


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


def _over_gap(gap: str) -> tuple[float, float, float]:
    return (GAUGE_X, GAPS[gap][0], ANVIL_TOP + GAUGE_DROP + GRIP_ABOVE_BOTTOM)


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
    go = _over_gap("go")
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
            ("carry to GO", "lower over GO", "settle over GO", "let go over GO", "back off", "look at GO"),
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
                nogo = _over_gap("nogo")
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
                   "back off", "look")
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
    _POSE_LOG["step"] = 0
    return demo_plan()[0]


if __name__ == "__main__":
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import check_demo

    check_demo.main()
