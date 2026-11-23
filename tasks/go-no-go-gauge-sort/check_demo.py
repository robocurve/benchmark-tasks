"""Offline checks for gauge_sort.py, no simulator (NumPy and SciPy, with yam_arena importable):

    python tasks/go-no-go-gauge-sort/check_demo.py        (or: python tasks/go-no-go-gauge-sort/gauge_sort.py)

0. The starting layouts (layout numbers 0 to 199): every block at x >= 0.16, in its own arm's row, with finger room
   between neighbours and clear of the gauge and the trays; every block visits every place.
1. Scoring rules on hand-built sequences of states (the gauge checks are stateful), the gauge's dimensions and the
   expected score of random sorting.
2. The gauge reading (seated / stays out / none) on blocks seated in a gap, lying on the anvils, jammed tilted,
   held, and over the other gap; and that a block held still just clear of the gauge (hovering 1 to 3 mm above the
   anvils or the base, flat or tilted as if wedged, where the gripper would have it before letting go) gives no
   reading. Release: the scorer reads the grippers through gauge_sort.grip_reader(), which offline is replaced by
   a model ({arm: (pad gap, TCP)}). A block supported on the gauge gives no reading while any gripper within 50 mm
   of it is not open by 55 mm (a closed gripper holding it seated; one arm holding it while the other is open and
   far), a far closed gripper is ignored, and with the reader unavailable (None, or the real reader with no
   simulator) nothing is credited and a warning is printed once.
3. The demo, replayed through yam_arena.demo.ArmPlan at 20 Hz for both arms, with the blocks' resting places
   tracked so the demo's gauge readings resolve: the physical scenario (a block drops into a gap exactly when it
   is narrower), every block staying out of GO, every block dropping through both gaps, and every block passing,
   which between them take each arm through every branch. Checked: joints inside the URDF limits, IK error under
   0.5 mm / 0.1 deg, the arms' link-3/4/5, gripper and TCP frame origins at least ARM_CLEARANCE apart, every
   block ending in the tray its gauge readings call for, the physical scenario scoring 1.0, every let-go over a
   gap following a hold of SETTLE_S at the release point with the fingers shut, and the run fitting in the run
   limit (6,000 steps). score() runs every replayed step, as in the simulator: in the physical scenario it must
   register every block's GO check, a NO-GO check exactly for the GO passers, and end at 1.0. The physical
   scenario runs for layouts 0 to 9 and the demo run's layout 3.
"""

from __future__ import annotations

import contextlib
import io
import sys
from itertools import product
from pathlib import Path

import numpy as np

from yam_arena import YAM_URDF, ik, station
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load
from yam_arena.urdf import forward_kinematics

HERE = Path(__file__).resolve().parent
g = load(HERE / "gauge_sort.py").module

HZ = 20.0
RUN_LIMIT_STEPS = 6000
ARM_CLEARANCE = 0.100  # m between the arms' link-3/4/5, gripper and TCP frame origins

# Offline grippers: {arm: (pad gap m, TCP xyz)}. By default both are wide open and parked, far from the gauge.
WIDE = 2 * -FINGER_OPEN  # 93.9 mm
OPEN_FAR = {arm: (WIDE, np.array([g.PARK[0], g.SIDE[arm] * g.PARK[1], g.PARK[2]])) for arm in ("left", "right")}
g.grip_reader = lambda: OPEN_FAR


@contextlib.contextmanager
def grippers(grips):
    """Within the block, grip_reader() returns ``grips`` (a dict, or None: unavailable)."""
    before = g.grip_reader
    g.grip_reader = lambda: grips
    try:
        yield
    finally:
        g.grip_reader = before


# -- small helpers -----------------------------------------------------------------------------------------------
def _state(pos, size, vel=(0.0, 0.0, 0.0), roll: float = 0.0) -> ObjectState:
    """A box at ``pos``, turned ``roll`` rad about x, with its world-aligned bounding box."""
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    q = np.array([np.sin(roll / 2), 0.0, 0.0, np.cos(roll / 2)])
    r = g._rot(q)
    corners = np.array([pos + r @ (half * s) for s in product((-1, 1), repeat=3)])
    return ObjectState(pos, q, np.asarray(vel, float), np.zeros(3), corners.min(0), corners.max(0))


def wedged_pose(gap: str, w: float, roll: float = np.radians(10.0)) -> tuple[np.ndarray, float]:
    """(centre, roll) of a block wider than the gap, jammed tilted: its low bottom corner against the -y anvil's inner
    face, its bottom face resting on the +y anvil's top inner edge (the pose seen in simulation, ~12 deg)."""
    gy, gw = g.GAPS[gap]
    assert gw / np.cos(roll) < w  # the bottom face reaches the far anvil
    low = np.array([gy - gw / 2, g.ANVIL_TOP - gw * np.tan(roll)])
    along, up = np.array([np.cos(roll), np.sin(roll)]), np.array([-np.sin(roll), np.cos(roll)])
    c = low + along * w / 2 + up * g.PART_H / 2
    return np.array([g.GAUGE_X, c[0], c[1]]), roll


def _world(where: dict, vel=None) -> dict:
    out = {}
    for name, (w, x, y) in g.PARTS.items():
        pos = where.get(name, (x, y, g.TOP + g.PART_H / 2))
        pos, roll = pos if len(pos) == 2 else (pos, 0.0)  # (x, y, z) or ((x, y, z), roll)
        out[name] = _state(pos, (g.PART_LEN, w, g.PART_H), (vel or {}).get(name, (0, 0, 0)), roll)
    return out


def _in(tray: str, k: int) -> tuple[float, float, float]:
    """The k-th spot (0..7) in a tray: one layer, then a second layer on top."""
    ys = (-0.085, -0.045, 0.045, 0.085)
    return (g.TRAYS[tray][0], ys[k % 4], g.FLOOR_TOP + g.PART_H / 2 + (k // 4) * g.PART_H)


# -- 0. starting layout ------------------------------------------------------------------------------------------
MIN_START_X = 0.16  # picks nearer the arm bases (x 0.12) tipped blocks up on end in simulation
PAD_T = 0.0064  # finger tip thickness


def _footprint(o) -> tuple[float, float, float, float]:
    return (o.pos[0] - o.size[0] / 2, o.pos[0] + o.size[0] / 2, o.pos[1] - o.size[1] / 2, o.pos[1] + o.size[1] / 2)


def check_layout(seeds=range(200)) -> None:
    """For every layout: every block starts at x >= MIN_START_X, in its own arm's row, apart from the others with
    room for the open fingers, and clear of the gauge and the trays. Across layouts, every block visits every
    place, and layout_of() is deterministic."""
    assert -FINGER_OPEN >= -g.FINGER_WIDE > -g.FINGER_AJAR  # wider than a release, inside the gripper's travel
    visits = {n: set() for n in g.WIDTHS}
    seen_layouts = set()
    for seed in seeds:
        g.use_layout(seed)
        assert g.layout(seed) == g.layout(seed)
        visits_now = tuple(g.PARTS[n][1:] for n in g.WIDTHS)
        seen_layouts.add(visits_now)
        assert sorted(visits_now) == sorted(g.SLOTS), seed  # a permutation of the seven places
        for n, (_, x, y) in g.PARTS.items():
            visits[n].add((x, y))
        blocks = {o.name: o for o in g.OBJECTS if o.name in g.PARTS}
        assert sum(len(r) for r in g.ROWS.values()) == 7
        for arm, row in g.ROWS.items():
            xs = [g.PARTS[n][1] for n in row]
            assert xs == sorted(xs), arm  # picked nearest first
            for n in row:
                _, x, y = g.PARTS[n]
                assert x >= MIN_START_X - 1e-9, n
                assert np.sign(y) == g.SIDE[arm], n  # in front of its own arm
            for a, b in zip(row, row[1:]):  # the open fingers reaching for one clear the next block's end
                gap = g.PARTS[b][1] - g.PARTS[a][1] - g.PART_LEN / 2
                assert gap > -g.FINGER_WIDE + PAD_T + 0.010, (a, b, gap)
        fixed = [o for o in g.OBJECTS if o.mass is None]
        for n, o in blocks.items():
            x0, x1, y0, y1 = _footprint(o)
            for f in fixed:  # clear of the gauge and trays by 20 mm
                fx0, fx1, fy0, fy1 = _footprint(f)
                assert x1 + 0.02 <= fx0 or fx1 + 0.02 <= x0 or y1 + 0.02 <= fy0 or fy1 + 0.02 <= y0, (n, f.name)
            for m, p in blocks.items():  # apart from each other
                if m < n:
                    px0, px1, py0, py1 = _footprint(p)
                    assert x1 < px0 or px1 < x0 or y1 < py0 or py1 < y0, (n, m)
    assert all(len(v) == 7 for v in visits.values()), visits
    g.use_layout(0)
    print(f"layout: {len(seeds)} layout numbers ({len(seen_layouts)} distinct), every block at x >= 0.16 in its own"
          " arm's row, finger room between blocks, clear of gauge/trays; every block visits all 7 places")


# -- 1. scoring --------------------------------------------------------------------------------------------------
def _on_gauge(name: str, gap: str) -> tuple[float, float, float]:
    """Where the block lies after being set on this gap: on the base if narrower, on the anvils otherwise."""
    gy, gw = g.GAPS[gap]
    bottom = g.BASE_TOP if g.WIDTHS[name] < gw else g.ANVIL_TOP
    return (g.GAUGE_X, gy, bottom + g.PART_H / 2)


def play(where: dict, checks=(), steps: int | None = None, nogo_first=(), only_go=(), extra=(),
         extra_grips=OPEN_FAR) -> tuple[float, bool]:
    """Score a trial: the start, then each block in ``checks`` set on GO for ``steps`` control steps and, if it
    drops into GO (and isn't in ``only_go``), on NO-GO; then ``extra`` states (name -> pos, each held for ``steps``);
    then the end state ``where``. A block in ``nogo_first`` is set on NO-GO before GO too. The grippers are open and
    far except during ``extra``, where they are ``extra_grips``. Returns (score, success) of the end state."""
    steps = g.CHECK_HOLD_STEPS if steps is None else steps
    g.score(_world({}))  # a trial starts: checks clear
    assert all(v is None for v in g._CHECKS.go.values())
    placed: dict = {}
    for name in checks:
        seq = []
        if name in nogo_first:
            seq.append("nogo")
        seq.append("go")
        if g.WIDTHS[name] < g.GO_GAP and name not in only_go:
            seq.append("nogo")
        for gap in seq:
            for _ in range(steps):
                g.score(_world(dict(placed, **{name: _on_gauge(name, gap)})))
        placed[name] = where.get(name, (g.PARTS[name][1], g.PARTS[name][2], g.TOP + g.PART_H / 2))
    with grippers(extra_grips):
        for name, pos in extra:
            for _ in range(steps):
                g.score(_world(dict(placed, **{name: pos})))
    end = _world(where)
    return g.score(end), g.success(end)


def check_scoring() -> float:
    task = load(HERE / "gauge_sort.py")
    assert set(task.free_objects) == set(g.PARTS) and len(task.objects) == 7 + 5 + 10
    assert len(g.PASS) == 3 and len(g.REJECT) == 4
    for name, w in g.WIDTHS.items():  # every part clears its nearest limit by >= 1.5 mm
        assert min(abs(w - g.LOWER), abs(w - g.UPPER)) >= g.MARGIN - 1e-9, name
    widths = sorted(g.WIDTHS.values())  # the tightest case of each kind is in the batch
    for limit, side, margin in ((g.LOWER, -1, g.MARGIN), (g.LOWER, +1, g.MARGIN), (g.UPPER, +1, g.MARGIN)):
        assert any(abs(w - (limit + side * margin)) < 1e-9 for w in widths), (limit, side)
    assert max(w for w in widths if g.in_tolerance(w)) <= g.UPPER - g.GO_MARGIN + 1e-9  # >= 1.0 mm a side in GO
    by = {o.name: o for o in g.OBJECTS}
    go = by["go_anvil_left"].pos[1] - by["go_anvil_right"].pos[1] - g.ANVIL[1]
    nogo = by["nogo_anvil_left"].pos[1] - by["nogo_anvil_right"].pos[1] - g.ANVIL[1]
    assert abs(go - g.UPPER) < 1e-9 and abs(nogo - g.LOWER) < 1e-9
    for o in g.OBJECTS:  # every anvil stands on the base
        if "anvil" in o.name:
            assert abs(o.pos[1]) + g.ANVIL[1] / 2 <= g.GAUGE_BASE[1] / 2 + 1e-9, o.name
    widest = max(widths)  # the widest part fits a tray spot clear of its neighbour
    assert g.SLOTS_Y[1] + widest / 2 < g.TRAY_IN[1] / 2
    assert g.SLOTS_Y[1] - g.SLOTS_Y[0] > widest + 0.004
    assert g.SLOTS_Y[0] - widest / 2 > 0.020  # and over 20 mm from the midline: the arms' spots never touch
    assert all(o.mass is None for o in g.OBJECTS if o.name not in g.PARTS)
    assert g.CHECK_HOLD_STEPS == round(0.3 * HZ)
    check_layout()

    for seed in (0, 3):
        g.use_layout(seed)
        _scoring_cases()
    g.use_layout(0)
    guess = _random_sorting()
    print(f"scoring: all cases pass (layouts 0 and 3); random sorting of checked blocks expects {guess:.3f}")
    return guess


def _scoring_cases() -> None:
    names = list(g.PARTS)
    start = _world({})
    assert g.score(start) == 0.0 and not g.success(start)
    pass_k = reject_k = 0
    sorted_ = {}
    for name in names:
        if name in g.PASS:
            sorted_[name] = _in("pass_tray", pass_k)
            pass_k += 1
        else:
            sorted_[name] = _in("reject_tray", reject_k)
            reject_k += 1
    one_pass, one_reject = sorted(g.PASS)[0], sorted(g.REJECT)[0]
    narrow_reject = next(n for n in sorted(g.REJECT) if g.WIDTHS[n] < g.LOWER)  # passes GO, drops through NO-GO
    wide_reject = next(n for n in sorted(g.REJECT) if g.WIDTHS[n] > g.UPPER)  # stays out of GO
    all_but = lambda n: [m for m in names if m != n]  # noqa: E731

    # every block checked and sorted: 1.0; sorted right without any gauge check: 0
    assert play(sorted_, names) == (1.0, True)
    assert play(sorted_) == (0.0, False)
    assert play(sorted_, all_but(one_pass)) == (6 / 7, False)
    assert play(sorted_, all_but(wide_reject)) == (6 / 7, False)
    # each check needs CHECK_HOLD_STEPS in a row: one step short counts for nothing
    assert play(sorted_, names, steps=g.CHECK_HOLD_STEPS - 1) == (0.0, False)
    # a GO passer needs a NO-GO check after GO: GO alone, or NO-GO only before GO, is not enough
    assert play(sorted_, names, only_go=(one_pass,)) == (6 / 7, False)
    assert play(sorted_, names, only_go=(narrow_reject,)) == (6 / 7, False)
    assert play(sorted_, [m for m in names if m != one_pass], extra=[(one_pass, _on_gauge(one_pass, "nogo"))]) == (
        6 / 7, False)
    assert play(sorted_, names, nogo_first=(one_pass,)) == (1.0, True)  # NO-GO again after GO: fine
    # a narrow block held on top of a gap (it would drop in) is no reading: no GO credit
    held_top = (g.GAUGE_X, g.GO_Y, g.ANVIL_TOP + g.PART_H / 2)
    assert play(sorted_, all_but(one_pass), extra=[(one_pass, held_top)]) == (6 / 7, False)
    # Held, never let go: a block kept still just above the anvils (or the base) for 3x the check time, as a
    # gripper still holding it would have it, then put in the right tray, earns nothing; the rest still count.
    # (Here the grippers read open and far: these fail for want of support alone.)
    long = 3 * g.CHECK_HOLD_STEPS
    hovers = {
        wide_reject: [("go", (g.GAUGE_X, g.GO_Y, g.ANVIL_TOP + 0.0015 + g.PART_H / 2))],  # 1.5 mm over GO anvils
        one_pass: [("go", (g.GAUGE_X, g.GO_Y, g.BASE_TOP + 0.0015 + g.PART_H / 2)),  # 1.5 mm over the base, in GO
                   ("nogo", (g.GAUGE_X, g.NOGO_Y, g.ANVIL_TOP + 0.0015 + g.PART_H / 2))],  # then over NO-GO
        narrow_reject: [("go", (g.GAUGE_X, g.GO_Y, g.BASE_TOP + 0.002 + g.PART_H / 2)),
                        ("nogo", (g.GAUGE_X, g.NOGO_Y, g.BASE_TOP + 0.002 + g.PART_H / 2))],
    }
    wedge_c, wedge_roll = wedged_pose("go", g.WIDTHS[wide_reject])
    tilted_hover = (wedge_c + (0, 0, 0.0015), wedge_roll)  # tilted as if wedged, 1.5 mm clear of both anvils
    for name, steps_ in hovers.items():
        got = play(sorted_, all_but(name), extra=[(name, pos) for _, pos in steps_ for _ in range(3)], steps=long)
        assert got == (6 / 7, False), (name, got)
        assert g._CHECKS.go[name] is None and g._CHECKS.nogo[name] is None, name
    got = play(sorted_, all_but(wide_reject), extra=[(wide_reject, tilted_hover)], steps=long)
    assert got == (6 / 7, False) and g._CHECKS.go[wide_reject] is None, got
    # the same block really let go onto the gauge is credited (the hovers above fail only for want of support)
    wedge_rest = (wedge_c, wedge_roll)
    assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wedge_rest)], steps=long) == (1.0, True)
    # Gaming, all at 0: every block hovered over its gaps and never let go, then sorted right; nothing gauged;
    # every check one step short; everything in one tray (above and below).
    every_hover = [(n, p) for n in names for _, p in hovers.get(
        n, [("go", (g.GAUGE_X, g.GO_Y, (g.BASE_TOP if g.WIDTHS[n] < g.GO_GAP else g.ANVIL_TOP) + 0.0015
                     + g.PART_H / 2))])]
    assert play(sorted_, extra=every_hover, steps=long) == (0.0, False)
    assert play(sorted_, extra=[(n, tilted_hover) for n in sorted(g.REJECT) if g.WIDTHS[n] > g.UPPER],
                steps=long) == (0.0, False)
    # Released: the scorer reads the grippers. A block resting exactly where a let-go block would (seated in GO and
    # NO-GO, or on the GO anvils) earns nothing while a closed gripper holds it there, even for 3x the check time.
    long_ = 3 * g.CHECK_HOLD_STEPS
    def holding(pos, gap=g.PART_LEN):  # noqa: E306 - one gripper at the block, closed on its ends; the other far
        return {"left": (gap, np.asarray(pos, float)), "right": OPEN_FAR["right"]}
    for name in (one_pass, narrow_reject):
        seq = [_on_gauge(name, "go"), _on_gauge(name, "nogo")]
        held = play(sorted_, all_but(name), extra=[(name, p) for p in seq], steps=long_, extra_grips=holding(seq[0]))
        assert held == (6 / 7, False) and g._CHECKS.go[name] is None, (name, held)
    wide_on = _on_gauge(wide_reject, "go")
    assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_,
                extra_grips=holding(wide_on)) == (6 / 7, False)
    # ... and is credited once that gripper is open past PART_LEN + GRIP_CLEAR (just over: yes; just under: no)
    thresh = g.PART_LEN + g.GRIP_CLEAR
    assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_,
                extra_grips=holding(wide_on, thresh + 1e-4)) == (1.0, True)
    assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_,
                extra_grips=holding(wide_on, thresh - 1e-4)) == (6 / 7, False)
    # two arms: one holds the block closed while the other is open and far: no credit; open near both: credited
    for closed_arm in ("left", "right"):
        other = "right" if closed_arm == "left" else "left"
        two = {closed_arm: (g.PART_LEN, np.asarray(wide_on)), other: OPEN_FAR[other]}
        assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_,
                    extra_grips=two) == (6 / 7, False), closed_arm
    above = np.asarray(wide_on) + (0, 0, 0.03)  # both TCPs 20 mm above the block, wide open
    both_open = {"left": (WIDE, above), "right": (g.FINGER_AJAR * -2, above)}
    assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_,
                extra_grips=both_open) == (1.0, True)
    # a closed gripper more than GRIP_NEAR away is ignored; one just inside it is not
    box_top = wide_on[2] + g.PART_H / 2
    for d, credited in ((g.GRIP_NEAR + 0.002, True), (g.GRIP_NEAR - 0.002, False)):
        far_closed = {"left": (0.0, np.array([wide_on[0], wide_on[1], box_top + d])), "right": OPEN_FAR["right"]}
        got = play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_, extra_grips=far_closed)
        assert got == ((1.0, True) if credited else (6 / 7, False)), (d, got)
    # Fail closed: with the reader unavailable for the whole trial, nothing is credited, and it says so once.
    g._GRIP["warned"] = False
    out = io.StringIO()
    with grippers(None), contextlib.redirect_stdout(out):
        assert play(sorted_, names) == (0.0, False)
        assert play(sorted_, all_but(wide_reject), extra=[(wide_reject, wide_on)], steps=long_, extra_grips=None) == (
            0.0, False)
    assert out.getvalue().count("WARNING: cannot read the grippers") == 1, out.getvalue()
    g._GRIP["warned"] = False
    # a reader that raises fails closed too
    def broken():  # noqa: E306
        raise RuntimeError("test")
    before, g.grip_reader = g.grip_reader, broken
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            assert play(sorted_, names) == (0.0, False)
    finally:
        g.grip_reader = before
        g._GRIP["warned"] = False
    # checks clear at the next trial's start
    play(sorted_, names)
    assert play(sorted_) == (0.0, False)

    # tray placement, every block checked
    moving = _world(sorted_, vel={one_pass: (0, 0, -0.2)})
    play(sorted_, names)
    assert not g.success(moving) and g.score(moving) == 6 / 7
    cx = g.TRAYS["pass_tray"][0]
    for label, pos in (
        ("held 10 cm above the tray", (cx, 0.045, g.RIM + 0.10)),
        ("held 5 mm above the floor", (cx, 0.045, g.FLOOR_TOP + 0.005 + g.PART_H / 2)),
        ("on the near wall's rim", (cx - g.TRAY_IN[0] / 2 - g.WALL_T / 2, 0.045, g.RIM + g.PART_H / 2)),
        ("straddling the near wall", (cx - g.TRAY_IN[0] / 2, 0.045, g.RIM + g.PART_H / 2)),
        ("on the side wall's rim", (cx, g.TRAY_IN[1] / 2 + g.WALL_T / 2, g.RIM + g.PART_H / 2)),
    ):
        got = play(dict(sorted_, **{one_pass: pos}), names)
        assert got == (6 / 7, False), (label, got)
    passes = sorted(g.PASS)  # a second layer counts
    stacked = dict(sorted_, **{passes[2]: _in("pass_tray", 4)})  # on top of passes[0]
    assert play(stacked, names) == (1.0, True)
    floating = dict(sorted_, **{passes[2]: _in("pass_tray", 7)})  # one layer up with nothing beneath: held
    assert play(floating, names) == (6 / 7, False)
    # one tray for all: 0 either way, checked or not
    all_pass = {n: _in("pass_tray", i) for i, n in enumerate(names)}
    all_reject = {n: _in("reject_tray", i) for i, n in enumerate(names)}
    for where in (all_pass, all_reject):
        assert play(where, names)[0] == 0.0 and play(where)[0] == 0.0
    # escapes and false rejects are penalised whether or not the block was checked
    one_escape = dict(sorted_, **{one_reject: _in("pass_tray", 4)})
    other_reject = sorted(g.REJECT)[1]
    two_escapes = dict(one_escape, **{other_reject: _in("pass_tray", 5)})
    one_false = dict(sorted_, **{one_pass: _in("reject_tray", 4)})
    assert play(one_escape, names)[0] == 0.25 and play(two_escapes, names)[0] == 1 / 7
    assert play(one_escape, all_but(one_reject))[0] == 0.25
    assert play(one_false, names)[0] == (6 - 2) / 7 and play(one_false, all_but(one_pass))[0] == 4 / 7
    # only the passes sorted 3/7, only the rejects 4/7
    assert play({n: sorted_[n] for n in g.PASS}, names)[0] == 3 / 7
    assert play({n: sorted_[n] for n in g.REJECT}, names)[0] == 4 / 7


def _random_sorting() -> float:
    """Every block checked, then each into either tray at random, all 2^7 ways equally likely."""
    names = list(g.PARTS)
    total = 0.0
    for pick in product(("pass_tray", "reject_tray"), repeat=len(names)):
        counts = {"pass_tray": 0, "reject_tray": 0}
        where = {}
        for n, t in zip(names, pick, strict=True):
            where[n] = _in(t, counts[t])
            counts[t] += 1
        total += play(where, names)[0]
    guess = total / 2 ** len(names)
    assert guess < 0.1, guess
    return guess


# -- 2. the gauge reading ----------------------------------------------------------------------------------------
def check_reading() -> None:
    def at(gap, w, bottom, dy=0.0, tall=g.PART_H):
        return _state((g.GAUGE_X, g.GAPS[gap][0] + dy, bottom + tall / 2), (g.PART_LEN, w, tall))

    def wedged(gap, w, lift=0.0):  # jammed tilted between the anvils; ``lift``: held that far above that pose
        c, roll = wedged_pose(gap, w)
        return _state(c + (0, 0, lift), (g.PART_LEN, w, g.PART_H), roll=roll)

    for gap, (_, gap_w) in g.GAPS.items():
        small, big = gap_w - g.MARGIN, gap_w + g.MARGIN
        assert g.seated(at(gap, small, g.BASE_TOP), gap)
        assert g.seated(at(gap, small, g.BASE_TOP, dy=0.0007), gap)  # off-centre by its clearance
        assert not g.seated(at(gap, big, g.ANVIL_TOP), gap)  # on the anvils
        assert g.stays_out(at(gap, big, g.ANVIL_TOP), gap)
        assert not g.seated(wedged(gap, big), gap)  # jammed tilted
        assert g.stays_out(wedged(gap, big), gap, big)
        assert not g.stays_out(wedged(gap, big), gap)  # a tilted pose needs the width to check contact
        assert not g.seated(wedged(gap, gap_w + 0.005), gap)
        assert g.stays_out(wedged(gap, gap_w + 0.005), gap, gap_w + 0.005)
        assert not g.seated(at(gap, small, g.BASE_TOP + 0.012), gap)  # still on its way down / held
        assert not g.stays_out(at(gap, big, g.ANVIL_TOP + 0.010), gap)  # held above the anvils
        other = "go" if gap == "nogo" else "nogo"
        assert not g.seated(at(gap, small, g.BASE_TOP), other)  # the other gap
        assert not g.stays_out(at(gap, big, g.ANVIL_TOP), other)
        # Held, not released: a block at rest just clear of the gauge, as a gripper still holding it would have it
        # (here the grippers read open and far, so only support decides). 1 to 3 mm of air under it: no reading. The old rule (3 mm
        # tolerances, no contact test) read every one of these.
        for air in (0.001, 0.0015, 0.002, 0.0029):
            assert not g.stays_out(at(gap, big, g.ANVIL_TOP + air), gap, big), (gap, air)  # flat, above the anvils
            assert not g.seated(at(gap, small, g.BASE_TOP + air), gap), (gap, air)  # above the base, in the gap
            assert not g.stays_out(wedged(gap, big, lift=air), gap, big), (gap, air)  # tilted, clear of both anvils
        # supported but not still (being set down or dragged across): no reading
        moving = (0.0, 0.03, 0.0)
        assert not g.seated(_state(at(gap, small, g.BASE_TOP).pos, (g.PART_LEN, small, g.PART_H), moving), gap)
        assert not g.stays_out(_state(at(gap, big, g.ANVIL_TOP).pos, (g.PART_LEN, big, g.PART_H), moving), gap, big)
        # level on the anvils but not spanning the gap (it would tip in): held
        assert not g.stays_out(at(gap, big, g.ANVIL_TOP, dy=0.012), gap, big)
        # tilted 5 deg on the base: not lying flat in the gap (held, or propped)
        tipped = _state(at(gap, small, g.BASE_TOP).pos, (g.PART_LEN, small, g.PART_H), roll=np.radians(5))
        assert not g.seated(tipped, gap)
    # gauge_reading: only the reading the block's width allows
    for name, w in g.WIDTHS.items():
        for gap, (_, gap_w) in g.GAPS.items():
            on_base, on_top = at(gap, w, g.BASE_TOP), at(gap, w, g.ANVIL_TOP)
            assert g.gauge_reading(name, on_base, gap, OPEN_FAR) is (True if w < gap_w else None), (name, gap)
            assert g.gauge_reading(name, on_top, gap, OPEN_FAR) is (None if w < gap_w else False), (name, gap)
            if w < gap_w:  # held partway down the gap: no reading
                assert g.gauge_reading(name, at(gap, w, g.BASE_TOP + 0.012), gap, OPEN_FAR) is None, (name, gap)
            # the same supported poses with a gripper closed on the block, or with no gripper reading: none
            for s in (on_base, on_top):
                closed = {"left": (g.PART_LEN, s.pos), "right": OPEN_FAR["right"]}
                assert g.gauge_reading(name, s, gap, closed) is None, (name, gap)
                assert g.gauge_reading(name, s, gap, None) is None, (name, gap)
    # the real reader, with no simulator running, gives None (fail closed) and says why
    assert g._sim_grip_reader() is None and g._GRIP["why"], g._GRIP["why"]
    print("gauge reading: seated / on the anvils / jammed / held / other gap all read right; readings follow width;"
          " blocks held 1-3 mm clear of the anvils or base (flat or tilted), moving, or tipped give no reading;"
          f" a closed gripper on the block or no gripper reading gives none (offline real reader: {g._GRIP['why']})")


# -- 3. the demo -------------------------------------------------------------------------------------------------
class Blocks:
    """Where each block rests, for the demo's gauge readings: carried by the fingers that close on it, dropped
    straight down when let go; over a gap, seated or left on the anvils as the scenario says."""

    def __init__(self, outcome) -> None:
        self.outcome = outcome  # (name, gap) -> seated?
        self.size = {n: np.array([g.PART_LEN, w, g.PART_H]) for n, (w, _, _) in g.PARTS.items()}
        self.pos = {n: np.array([x, y, g.TOP + g.PART_H / 2]) for n, (_, x, y) in g.PARTS.items()}
        self.held: dict[str, tuple[str, np.ndarray]] = {}  # name -> (arm, offset from the TCP)
        self.readings: dict[tuple[str, str], bool] = {}

    def state(self, name: str) -> ObjectState:
        return _state(self.pos[name], self.size[name])

    def update(self, arm: str, tcp: np.ndarray, grip: float, label: str) -> None:
        name = label.split(":")[0] if label.startswith("part_") else None
        held = next((n for n, (a, _) in self.held.items() if a == arm), None)
        touching = grip * -FINGER_OPEN < g.PART_LEN / 2  # finger inner faces reach the block's ends
        if held is None and touching and name is not None:
            self.held[name] = (arm, self.pos[name] - tcp)
        elif held is not None and not touching:
            del self.held[held]
            p = self.pos[held]
            rest = g.TOP
            for gap, (gy, _) in g.GAPS.items():
                if abs(p[0] - g.GAUGE_X) < 0.01 and abs(p[1] - gy) < 0.01:
                    # In simulation the block slides back under the TCP as the fingers open, landing off by about
                    # -RELEASE_DY (logs/poses-r1); the demo's RELEASE_DY cancels that. Model the slide, so a block
                    # lands where it would in the simulator, not where the TCP is.
                    p = p - (0.0, g.RELEASE_DY[arm], 0.0)
                    seat = self.outcome(held, gap)
                    self.readings[held, gap] = seat
                    rest = g.BASE_TOP if seat else g.ANVIL_TOP
                    break
            else:
                for cx, _ in g.TRAYS.values():
                    if abs(p[0] - cx) < g.TRAY_IN[0] / 2 and abs(p[1]) < g.TRAY_IN[1] / 2:
                        rest = g.FLOOR_TOP
                for other, q in self.pos.items():  # onto a block underneath
                    if other != held and other not in self.held and q[2] < p[2]:
                        if np.all(np.abs(p[:2] - q[:2]) < ((self.size[held] + self.size[other]) / 2)[:2]):
                            rest = max(rest, q[2] + g.PART_H / 2)
            self.pos[held] = np.array([p[0], p[1], rest + g.PART_H / 2])
        elif held is not None:
            self.pos[held] = tcp + self.held[held][1]


def settles(plans_wp, gauge_tests: int) -> list[str]:
    """Problems with the hold before each let-go over a gap (there are ``gauge_tests`` of them): the waypoint before
    it must keep the fingers shut at the release point for SETTLE_S, after the lowering to that same point."""
    problems, seen = [], 0
    for arm, wps in plans_wp.items():
        for i, w in enumerate(wps):
            p = np.asarray(w.pos, float)
            over_gap = any(abs(p[0] - g.GAUGE_X) < 1e-9 and abs(p[1] - gy - g.RELEASE_DY[arm]) < 1e-9
                           for gy, _ in g.GAPS.values())
            if not (w.finger == g.FINGER_AJAR and over_gap and "let go" in w.label):
                continue
            seen += 1
            hold, lower = wps[i - 1], wps[i - 2]
            ok = ("settle" in hold.label and hold.duration >= g.SETTLE_S - 1e-9 and hold.finger == g.FINGER_SHUT
                  and lower.finger == g.FINGER_SHUT and np.allclose(hold.pos, p) and np.allclose(lower.pos, p))
            if not ok:
                problems.append(f"{arm} {w.label!r}: no {g.SETTLE_S} s settle hold before it")
    if seen != gauge_tests:
        problems.append(f"{seen} let-gos over a gap in the plan, {gauge_tests} gauge tests")
    return problems


def run(outcome, title: str) -> dict:
    """Replay the demo through ArmPlan with these gauge outcomes."""
    blocks = Blocks(outcome)
    g._SENSE = blocks.state
    arms: dict = {}  # this step's {arm: (pad gap, TCP)}, from the plan's finger command and FK
    before, g.grip_reader = g.grip_reader, lambda: dict(arms)
    try:
        plans_wp, insp, schedule = g.demo_plan()
        plans = {arm: ArmPlan(wps, station.BASES[arm]) for arm, wps in plans_wp.items()}
        steps = round(max(p.duration for p in plans.values()) * HZ) + 5
        fixed = {o.name: _state(o.pos, o.size) for o in g.OBJECTS if o.mass is None}
        joints_ok, closest, closest_at = True, np.inf, None
        for k in range(steps):
            t = (k + 1) / HZ
            frames = {}
            for arm, plan in plans.items():
                action, label = plan.action7(t)
                q = action[:6]
                joints_ok &= bool(np.all(q >= ik.LOWER) and np.all(q <= ik.UPPER))
                base = np.asarray(station.BASES[arm], float)
                poses = forward_kinematics(YAM_URDF, dict(zip(ARM_JOINTS, q, strict=True)))
                tcp, _ = ik.tcp_pose(q, base)
                pts = [poses[l][:3, 3] + base for l in ("link3", "link4", "link5", "gripper")] + [tcp]
                frames[arm] = (np.array(pts), label)
                blocks.update(arm, tcp, float(action[6]), label)
                arms[arm] = (2 * float(action[6]) * -FINGER_OPEN, np.asarray(tcp, float).copy())
            g.score(fixed | {n: blocks.state(n) for n in g.PARTS})  # as the simulator does after every step
            (pl, ll), (pr, lr) = frames["left"], frames["right"]
            d = float(np.min(np.linalg.norm(pl[:, None] - pr[None], axis=2)))
            if d < closest:
                closest, closest_at = d, (t, ll, lr)
        problems = settles(plans_wp, len(blocks.readings))
        final = {n: blocks.state(n) for n in g.PARTS}
        for name in g.PARTS:  # every block where its gauge readings send it
            go = blocks.readings.get((name, "go"))
            nogo = blocks.readings.get((name, "nogo"))
            if go is None or (go and nogo is None) or (not go and nogo is not None):
                problems.append(f"{name}: gauge tests GO={go} NO-GO={nogo}")
                continue
            want = "pass_tray" if go and not nogo else "reject_tray"
            if g.tray_of(name, final) != want:
                problems.append(f"{name}: gauge says {want}, ends at {np.round(blocks.pos[name], 3)}")
        states = dict(final, **fixed)
        result, done = g.score(states), g.success(states)
        if outcome is physical:  # the scorer saw every check the gauge readings imply
            for name in g.PARTS:
                want_nogo = physical(name, "nogo") if physical(name, "go") else None
                if g._CHECKS.go[name] is not physical(name, "go") or g._CHECKS.nogo[name] is not want_nogo:
                    problems.append(f"{name}: scorer registered GO={g._CHECKS.go[name]} NO-GO={g._CHECKS.nogo[name]}")
        return {
            "title": title, "steps": steps, "closest_mm": closest * 1000, "closest_at": closest_at,
            "joints_ok": joints_ok, "ik": {a: (p.worst_ik_mm, p.worst_ik_deg) for a, p in plans.items()},
            "score": result, "success": done, "problems": problems,
            "schedule": schedule, "verdicts": {n: insp.verdict(n) for n in g.PARTS},
        }
    finally:
        g._SENSE = None
        g.grip_reader = before


def physical(name: str, gap: str) -> bool:
    """A block drops into a gap exactly when it is narrower."""
    return g.WIDTHS[name] < g.GAPS[gap][1]


def scenarios():
    yield "physical (widths vs gaps)", physical
    yield "every block stays out of GO", lambda n, gap: False
    yield "every block drops through both gaps", lambda n, gap: True
    yield "every block passes", lambda n, gap: gap == "go"


def _report(r) -> bool:
    ik_txt = ", ".join(f"{a} {mm:.3f} mm / {deg:.4f} deg" for a, (mm, deg) in r["ik"].items())
    print(f"- {r['title']}: {r['steps']} steps ({r['steps'] / HZ:.1f} s), score {r['score']:.3f}"
          f"{' success' if r['success'] else ''}; worst IK {ik_txt}; joints "
          f"{'inside' if r['joints_ok'] else 'OUTSIDE'} the limits; arms' wrist/TCP frames >= "
          f"{r['closest_mm']:.0f} mm apart (t={r['closest_at'][0]:.1f}: {r['closest_at'][1]!r} / {r['closest_at'][2]!r})")
    for p in r["problems"]:
        print("    ", p)
    return not (
        not r["problems"] and r["joints_ok"] and r["steps"] <= RUN_LIMIT_STEPS
        and all(mm < 0.5 and deg < 0.1 for mm, deg in r["ik"].values())
        and r["closest_mm"] >= ARM_CLEARANCE * 1000
    )


def check_demo(physical_seeds=range(10), all_scenarios_seed=3) -> dict:
    failed, results = False, {}
    for seed in sorted(set(physical_seeds) | {all_scenarios_seed}):
        g.use_layout(seed)
        for title, outcome in scenarios():
            if outcome is not physical and seed != all_scenarios_seed:
                continue
            r = run(outcome, f"layout {seed}, {title}")
            failed |= _report(r)
            if outcome is physical:
                failed |= not (r["score"] == 1.0 and r["success"])
                results[seed] = r
    r = results[all_scenarios_seed]
    print(f"layout {all_scenarios_seed} physical verdicts: {r['verdicts']}")
    print("gauge turns (block, from - to, s): " + ", ".join(f"{n} {a:.1f}-{b:.1f}" for n, a, b in r["schedule"]))
    g.use_layout(0)
    assert not failed, "demo check failed"
    return r


def main() -> None:
    check_scoring()
    check_reading()
    check_demo()
    print("gauge_sort offline checks passed")


if __name__ == "__main__":
    sys.exit(main())
