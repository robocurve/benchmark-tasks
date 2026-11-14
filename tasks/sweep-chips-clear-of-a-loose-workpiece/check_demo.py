"""Offline checks for sweep_chips.py, no simulator and no GPU (NumPy, with yam_arena importable):

    python tasks/sweep-chips-clear-of-a-loose-workpiece/check_demo.py

0. Layouts 0 to 9 (and a count over 0 to 199): every chip in a grid cell on the plate, apart from the others, at least
   10 mm from the workpiece, clear of the bar's start; at least two chips within 25 mm of a workpiece face and at
   least one in its shadow; and a sweep plan exists (solvable).
1. Bar length: the plan's smallest margin for 70, 75, 80 and 100 mm bars, layouts 0 to 9.
2. Scoring on hand-built sequences of states (the scoring is stateful).
3. The demo, replayed offline for layouts 0 to 9: through yam_arena.demo.ArmPlan at 20 Hz (IK error, joint limits,
   steps within the run limit), with a kinematic chip model (no physics: a chip touched by the bar's moving face is
   pushed ahead of it, chips push chips, a chip past the plate edge drops into the trough) and score() every step,
   as in the simulator. Checked: final score 1.0 and success held, the bar never lowered onto a chip, the bar and the
   fingers clear of the workpiece, the trough walls and lip, and the fingers clear of every chip.
"""

from __future__ import annotations

import math
import sys
import time
from pathlib import Path

import numpy as np

from yam_arena import ik, station
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.task_file import SUCCESS_HOLD_S, ObjectState, load

HERE = Path(__file__).resolve().parent
t = load(HERE / "sweep_chips.py").module

HZ = 20.0
DT = 1.0 / HZ
RUN_LIMIT_STEPS = 3000
SEEDS = range(10)
JOINTS = ["j1", "j2", "j3", "j4", "j5", "j6"]
Q0 = np.eye(4)[3]  # identity quaternion, xyzw
FAILS: list[str] = []


def check(cond: bool, what: str) -> bool:
    if not cond:
        FAILS.append(what)
        print(f"  FAIL: {what}")
    return bool(cond)


# -- states ----------------------------------------------------------------------------------------------------------
def _state(pos, size, vel=(0.0, 0.0, 0.0), quat=None, spin=(0.0, 0.0, 0.0)) -> ObjectState:
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    q = Q0 if quat is None else np.asarray(quat, float)
    if quat is not None:  # world-aligned box of the turned box
        x, y, z, w = q
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        half = np.abs(r) @ half
    return ObjectState(pos, q, np.asarray(vel, float), np.asarray(spin, float), pos - half, pos + half)


def _quat_z(deg: float):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _quat_x(deg: float):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _quat_y(deg: float):
    a = math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def world(chips: dict | None = None, bar=None, wp=None, vel: dict | None = None, wp_quat=None) -> dict:
    """States for one step. chips: name -> (x, y, bottom z) (default: start); bar, wp: centre (default: start)."""
    vel = vel or {}
    out = {}
    for n, (x, y) in t.CHIPS.items():
        cx, cy, low = (chips or {}).get(n, (x, y, t.PT))
        out[n] = _state((cx, cy, low + t.H), (t.CHIP,) * 3, vel.get(n, (0, 0, 0)))
    out["bar"] = _state(bar or (*t.BAR_START, t.PT + t.BAR[2] / 2), t.BAR, vel.get("bar", (0, 0, 0)))
    w = wp or (t.WP_X, t.WP_Y, t.PT + t.WP[2] / 2)
    out["workpiece"] = _state(w, t.WP, vel.get("workpiece", (0, 0, 0)), quat=wp_quat)
    return out


def run(seq) -> list[float]:
    """Score a sequence of states from a fresh trial; returns the score after every step."""
    t._S.reset()
    t._S.last = None
    return [t.score(s) for s in seq]


def _moves(start: dict, end: dict, steps: int, others: dict | None = None, bar=None, wp=None):
    """States moving chips linearly from start to end positions over ``steps`` steps (velocities set)."""
    seq = []
    for k in range(1, steps + 1):
        f = k / steps
        pos = {n: tuple(np.add(start[n], f * np.subtract(end[n], start[n]))) for n in start}
        v = {n: tuple(np.subtract(end[n], start[n]) / steps / DT) for n in start}
        seq.append(world({**(others or {}), **pos}, bar=bar, wp=wp, vel=v))
    return seq


def _start(n):
    x, y = t.CHIPS[n]
    return (x, y, t.PT)


# -- 0. layouts --------------------------------------------------------------------------------------------------
def check_layouts() -> None:
    print("0. layouts")
    for seed in SEEDS:
        t.use_layout(seed)
        ok = True
        cells = list(t.CHIPS.values())
        ok &= check(len(set(cells)) == t.N_CHIPS and all(c in t.CELLS for c in cells), f"layout {seed}: 8 distinct cells")
        for n, (x, y) in t.CHIPS.items():
            ok &= check(t.PLATE_X[0] + t.H <= x <= t.PLATE_X[1] - t.H and t.PLATE_Y[0] + t.H <= y <= t.PLATE_Y[1] - t.H,
                        f"layout {seed}: {n} on the plate")
            ok &= check(t.gap_to_wp(x, y, t.WP_Y) >= 0.010 - 1e-9, f"layout {seed}: {n} >= 10 mm from the workpiece")
            bx0 = t.BAR_START[0] + t.BAR[0] / 2
            ok &= check(x - t.H - bx0 >= 0.030, f"layout {seed}: {n} clear of the bar start")
        near = sum(t.gap_to_wp(x, y, t.WP_Y) <= 0.025 for x, y in cells)
        shadow = sum(x > t.WP_X + t.WP[0] / 2 and abs(y - t.WP_Y) < t.WP[1] / 2 for x, y in cells)
        ok &= check(near >= 2 and shadow >= 1, f"layout {seed}: >= 2 near, >= 1 in shadow")
        ok &= check(t.layout(seed) == t.layout(seed), f"layout {seed}: deterministic")
        p = t.make_plan()
        ok &= check(p is not None, f"layout {seed}: a sweep plan exists")
        if p:
            print(f"  layout {seed}: workpiece y {t.WP_Y:.2f}; {len(p.lanes)} side lanes, {len(p.pushes)} end push(es)"
                  f"{', 1 re-sweep' if p.resweep else ''}; margin {p.m * 1000:.2f} mm, end walls {p.wall * 1000:.1f} mm")
    solved, worst = 0, 1.0
    for seed in range(200):
        t.use_layout(seed)
        p = t.make_plan()
        if p:
            solved += 1
            worst = min(worst, p.m)
    print(f"  layouts 0-199: {solved} have a plan (smallest margin {worst * 1000:.2f} mm)")
    t.use_layout(0)


# -- 1. bar length -----------------------------------------------------------------------------------------------
def check_bar_length() -> None:
    print("1. bar length (smallest plan margin, mm; '-' = no plan)")
    best = {}
    for bar_len in (0.070, 0.075, 0.080, 0.100):
        row = []
        for seed in SEEDS:
            t.use_layout(seed)
            p = t.make_plan(bar_len=bar_len)
            row.append(None if p is None else p.m)
        best[bar_len] = min((m for m in row if m is not None), default=0.0) if all(row) else 0.0
        print(f"  {bar_len * 1000:.0f} mm: " + " ".join("-" if m is None else f"{m * 1000:.2f}" for m in row))
    check(max(best, key=best.get) == t.L, f"the task's {t.L * 1000:.0f} mm bar has the best worst-case margin")
    t.use_layout(0)


# -- 2. scoring --------------------------------------------------------------------------------------------------
def _in_trough_spots():
    """Eight trough spots, one layer: two rows across the 40 mm trough."""
    xs, ys = (0.4095, 0.4305), np.linspace(0.12, 0.32, 4)
    return [(x, y, t.TOP) for x in xs for y in ys]


def check_scoring() -> None:
    print("2. scoring")
    t.use_layout(2)
    names = t.CHIP_NAMES
    spots = dict(zip(names, _in_trough_spots(), strict=True))
    still_start = [world() for _ in range(20)]

    s = run(still_start)
    check(s[-1] == 0.0 and not t.success(still_start[-1]), "nothing done: 0")

    # picked by hand: each chip lifted 30 mm, carried over the trough and set down on its floor
    seq, where = [], {n: _start(n) for n in names}
    for n in names:
        x, y, _ = where[n]
        up = (x, y, t.PT + 0.030)
        over = (spots[n][0], spots[n][1], t.PT + 0.030)
        for a, b in ((where[n], up), (up, over), (over, spots[n])):
            seq += _moves({n: a}, {n: b}, 10, others={m: where[m] for m in names if m != n})
        where[n] = spots[n]
    seq += [world(where) for _ in range(10)]
    s = run(seq)
    check(s[-1] == 0.0 and all(t._S.lifted[n] for n in names), "every chip picked up by hand and put in the trough: 0")

    # lifted 3 mm (inside LIFT_TOL) then swept is fine; 5 mm is a lift
    n = "chip_7"
    for dz, want in ((0.003, None), (0.005, "lifted")):
        x, y, _ = _start(n)
        seq = _moves({n: _start(n)}, {n: (x, y, t.PT + dz)}, 5, bar=(x - 0.0175, y, t.PT + 0.013))
        run(seq)
        check((t._S.lifted[n] is not None) == (want == "lifted"), f"chip raised {dz * 1000:.0f} mm: lifted={want}")

    # pushed with the closed fingers, no bar (the bar left at its start): each chip slides on the plate to the edge
    # and drops into the trough
    seq, where = [], {n: _start(n) for n in names}
    for n in names:
        x, y, _ = where[n]
        edge = (t.PLATE_X[1] - t.H + 0.001, y, t.PT)
        seq += _moves({n: where[n]}, {n: edge}, max(2, int((edge[0] - x) / 0.05 / DT)),
                      others={m: where[m] for m in names if m != n})
        where[n] = (0.4205, y if t.PLATE_Y[0] + t.H <= y <= t.PLATE_Y[1] - t.H else t.CY, t.TOP)
        seq.append(world(where))
    seq += [world(where) for _ in range(10)]
    s = run(seq)
    check(len(t.in_trough(seq[-1])) == 8, "fingertip pushing: every chip ends in the trough")
    check(s[-1] == 0.0 and all(t._S.unbarred[n] for n in names), "pushed with closed fingers and no bar: 0")

    # chains: a chip pushed by a chip that the bar pushes keeps its credit; one moving 15 mm from the bar does not
    a, b = "chip_2", "chip_6"  # same x row in layout 2 (x 0.205): put them in contact for the test
    xa, ya, _ = _start(a)
    pa0, pb0 = (xa, ya, t.PT), (xa + t.CHIP, ya, t.PT)
    pa1, pb1 = (xa + 0.02, ya, t.PT), (xa + t.CHIP + 0.02, ya, t.PT)
    bar0 = np.array([xa - t.H - t.BAR[0] / 2, ya, t.PT + 0.013])
    seq = []
    for k in range(1, 11):
        f = k / 10
        v = (0.02 / 10 / DT, 0, 0)
        seq.append(world({a: tuple(np.add(pa0, f * np.subtract(pa1, pa0))), b: tuple(np.add(pb0, f * np.subtract(pb1, pb0)))},
                         bar=tuple(bar0 + (f * 0.02, 0, 0)), vel={a: v, b: v, "bar": v}))
    run(seq)
    check(t._S.unbarred[a] is None and t._S.unbarred[b] is None, "a chain (bar -> chip -> chip) keeps its credit")
    seq = [world({b: (xa + 0.0325 + 0.001 * k, ya, t.PT)}, bar=tuple(bar0), vel={b: (0.02, 0, 0)}) for k in range(5)]
    run(seq)
    check(t._S.unbarred[b] is not None, "a chip moving 15 mm from the bar, touching nothing driven: not credited")

    # riding on another chip is not a lift; held 5 mm above it is
    seq = [world({a: pa0, b: (xa, ya, t.PT + t.CHIP)}, bar=tuple(bar0)) for _ in range(3)]
    seq[0] = world({b: (xa, ya, t.PT + t.CHIP)}, bar=tuple(bar0))
    run(seq)
    check(t._S.lifted[b] is None, "a chip riding on another chip on the plate: not lifted")
    seq = [world({a: pa0, b: (xa, ya, t.PT + t.CHIP + 0.005)}, bar=tuple(bar0))]
    run(seq)
    check(t._S.lifted[b] is not None, "a chip held 5 mm above another chip: lifted")

    # reset: a trial's start clears the flags
    seq = [world({a: (xa, ya, t.PT + 0.03)}), world()]
    run(seq)
    check(t._S.lifted[a] is None, "flags clear at the next trial's start")

    # the demo's sweep (kinematic replay, layout 2) and variants of its end
    rep = replay(2, quiet=True)
    states = rep["states"]
    s = run(states)
    check(s[-1] == 1.0, f"chips swept by the bar, workpiece untouched: 1.0 (got {s[-1]})")
    final = states[-1]

    def with_end(extra_steps):
        return states + extra_steps

    def wp_at(dx=0.0, dy=0.0, dz=0.0, quat=None, vel=(0, 0, 0)):
        out = dict(final)
        w = (t.WP_X + dx, t.WP_Y + dy, t.PT + t.WP[2] / 2 + dz)
        out["workpiece"] = _state(w, t.WP, vel, quat=quat)
        return out

    for dx, want in ((0.004, 1.0), (0.010, 0.5), (0.020, 0.0)):
        s = run(with_end([wp_at(dx=dx) for _ in range(5)]))
        check(s[-1] == want, f"workpiece moved {dx * 1000:.0f} mm: {want} (got {s[-1]})")
    s = run(with_end([wp_at(quat=_quat_z(5)) for _ in range(5)]))
    check(s[-1] == 0.5, f"workpiece turned 5 degrees in place: 0.5 (got {s[-1]})")
    s = run(with_end([wp_at(dy=0.020)] * 3 + [wp_at() for _ in range(5)]))
    check(s[-1] == 0.0, f"workpiece pushed 20 mm aside and put back: 0 (got {s[-1]})")
    s = run(with_end([wp_at(dz=0.010)] * 3 + [wp_at() for _ in range(5)]))
    check(s[-1] == 0.0, f"workpiece lifted 10 mm and put back: 0 (got {s[-1]})")
    tipped = _quat_x(90)
    s = run(with_end([wp_at(dz=(t.WP[1] - t.WP[2]) / 2, quat=tipped) for _ in range(5)]))
    check(s[-1] == 0.0, f"workpiece tipped onto its side: 0 (got {s[-1]})")
    s = run(with_end([wp_at(dz=0.0005, quat=_quat_x(8)) for _ in range(3)] + [wp_at() for _ in range(5)]))
    check(s[-1] == 0.0, f"workpiece tipped 8 degrees and back: 0 (got {s[-1]})")
    s = run(with_end([wp_at(dx=0.12, dz=-t.PLATE_H) for _ in range(5)]))
    check(s[-1] == 0.0, f"workpiece in the trough: 0 (got {s[-1]})")

    # stacking in the trough, and what is not in the trough
    tr = sorted(t.in_trough(final))
    lo, hi = tr[0], tr[1]

    def moved(n, pos, vel=(0, 0, 0)):
        out = dict(final)
        out[n] = _state(pos, (t.CHIP,) * 3, vel)
        return out

    base = final[lo]
    on_top = (float(base.pos[0]), float(base.pos[1]), float(base.high[2]) + t.H)
    s = run(with_end([moved(hi, on_top) for _ in range(5)]))
    check(s[-1] == 1.0, f"a chip stacked in the trough on another chip: credited (got {s[-1]})")
    s = run(with_end([moved(hi, (on_top[0], on_top[1], on_top[2] + 0.005)) for _ in range(5)]))
    check(s[-1] == 7 / 8, f"a chip held 5 mm above a chip in the trough: not credited (got {s[-1]})")
    s = run(with_end([moved(hi, (t.TROUGH_X[0] + t.H - 0.003, 0.2, t.TOP + t.H)) for _ in range(5)]))
    check(s[-1] == 7 / 8, f"a chip straddling the plate edge: not credited (got {s[-1]})")
    hang = _state((0.405, 0.2, 0.0), (t.CHIP,) * 3, quat=_quat_y(22))  # hanging on the plate edge, tipped 22 deg
    hang = _state((0.405, 0.2, t.PT - 0.004 + (hang.high[2] - hang.low[2]) / 2), (t.CHIP,) * 3, quat=_quat_y(22))
    s = run(with_end([dict(final, **{hi: hang}) for _ in range(5)]))
    check(hang.low[0] < t.TROUGH_X[0] and hang.low[2] <= t.PT - t.LAYER_TOL and s[-1] == 7 / 8,
          f"a chip hanging on the plate edge at 22 degrees, centre x 0.405: not credited (got {s[-1]})")
    prop = _state((0.42, 0.2, 0.0), (t.CHIP,) * 3, quat=_quat_x(30))  # propped at 30 deg, bottom 5 mm off the floor
    prop = _state((0.42, 0.2, t.TOP + 0.005 + (prop.high[2] - prop.low[2]) / 2), (t.CHIP,) * 3, quat=_quat_x(30))
    s = run(with_end([dict(final, **{hi: prop}) for _ in range(5)]))
    check(s[-1] == 1.0, f"a chip propped 5 mm above the trough floor, footprint inside: credited (got {s[-1]})")
    s = run(with_end([moved(hi, (0.42, 0.2, t.TOP + t.H), vel=(0.02, 0, 0)) for _ in range(5)]))
    check(s[-1] == 7 / 8, f"a chip still moving in the trough: not credited (got {s[-1]})")
    s = run(with_end([moved(hi, (0.42, t.PLATE_Y[1] + 0.02, t.TOP + t.H)) for _ in range(5)]))
    check(s[-1] == 7 / 8, f"a chip on the table past the end wall: not credited (got {s[-1]})")
    print("  scoring cases: " + ("all as expected" if not FAILS else f"{len(FAILS)} failure(s) so far"))
    t.use_layout(0)


# -- 3. demo replay ----------------------------------------------------------------------------------------------
# Finger envelope from the URDF's collision meshes (tip_left/tip_right), relative to the pad tips: (height above the
# tip, thickness outward from the inner face, half-width across), closed on the bar.
FINGER_ZONES = ((0.000, 0.010, 0.0040, 0.0077), (0.010, 0.030, 0.0085, 0.0180), (0.030, 0.060, 0.0201, 0.0296))
TIP_PAST_TCP = 0.0128
FALL_X = 0.414  # where a chip pushed over the edge comes to rest (the probe run's chips: x 0.412 to 0.422)


def _boxes_fingers(tcp) -> list[tuple[np.ndarray, np.ndarray]]:
    """World boxes of both fingers closed on the bar (yaw -90: closing along x)."""
    out = []
    tip = tcp[2] - TIP_PAST_TCP
    for z0, z1, thick, half in FINGER_ZONES:
        for sgn in (-1, +1):
            inner = tcp[0] + sgn * t.BAR[0] / 2
            xa, xb = sorted((inner, inner + sgn * thick))
            out.append((np.array([xa, tcp[1] - half, tip + z0]), np.array([xb, tcp[1] + half, tip + z1])))
    return out


def _box_gap(a, b) -> float:
    d = np.maximum(0.0, np.maximum(a[0] - b[1], b[0] - a[1]))
    return float(np.linalg.norm(d))


def _fixed_box(name):
    o = next(o for o in t.FIXED if o.name == name)
    p, h = np.asarray(o.pos), np.asarray(o.size) / 2
    return (p - h, p + h)


def replay(seed: int, quiet: bool = False) -> dict:
    t.use_layout(seed)
    plan = t.make_plan()
    wps = t.demo_waypoints(plan)
    arm = ArmPlan(wps, station.BASES["left"])
    steps = int(math.ceil(arm.duration * HZ)) + 30  # and 1.5 s past the plan's end
    t_attach = sum(w.duration for w in wps[:3])
    bar_off = np.array([0.0, 0.0, -(t.TCP_ABOVE_BAR_BOTTOM - t.BAR[2] / 2)])
    chips = {n: np.array([x, y, t.PT]) for n, (x, y) in t.CHIPS.items()}  # centre x, y; bottom z
    bar = np.array([*t.BAR_START, t.PT + t.BAR[2] / 2])
    wp = (np.array([t.WP_X - t.WP[0] / 2, t.WP_Y - t.WP[1] / 2, t.PT]),
          np.array([t.WP_X + t.WP[0] / 2, t.WP_Y + t.WP[1] / 2, t.PT + t.WP[2]]))
    walls = [_fixed_box(n) for n in ("lip", "wall_near_y", "wall_far_y")]
    worst = np.full(6, np.inf)
    gaps = {"bar-workpiece": np.inf, "fingers-workpiece": np.inf, "bar-walls": np.inf, "fingers-walls": np.inf,
            "fingers-chips": np.inf}
    errors: list[str] = []
    states = []
    prev_chips = {n: c.copy() for n, c in chips.items()}
    prev_bar = bar.copy()

    def chip_box(c):
        return (np.array([c[0] - t.H, c[1] - t.H, c[2]]), np.array([c[0] + t.H, c[1] + t.H, c[2] + t.CHIP]))

    def bar_box(b):
        h = np.asarray(t.BAR) / 2
        return (b - h, b + h)

    def drop_into_trough(n):
        c = chips[n]
        c[0] = min(max(c[0], FALL_X), t.TROUGH_X[1] - t.H - 1e-4)
        z = t.TOP
        for m, o in chips.items():
            if m != n and o[2] < t.PT - 1e-6 and abs(o[0] - c[0]) < t.CHIP and abs(o[1] - c[1]) < t.CHIP:
                z = max(z, o[2] + t.CHIP)
        c[2] = z

    for k in range(steps):
        tt = (k + 1) / HZ
        a, _ = arm.action7(tt)
        worst = np.minimum(worst, np.degrees(np.minimum(a[:6] - ik.LOWER, ik.UPPER - a[:6])))
        tcp, _, _, label = arm.target(tt)
        tcp = np.asarray(tcp)
        if tt > t_attach:
            new_bar = tcp + bar_off
            sub = max(1, int(math.ceil(np.linalg.norm(new_bar - bar) / 0.0005)))
            for j in range(1, sub + 1):
                b_prev = bar.copy()
                bar = bar + (new_bar - bar) / (sub - j + 1)
                d = bar - b_prev
                lo_b, hi_b = bar_box(bar)
                horiz = np.abs(d[:2])
                axis = int(np.argmax(horiz)) if horiz.max() > 1e-12 else None
                for n, c in chips.items():
                    lo_c, hi_c = chip_box(c)
                    if not (np.all(lo_b < hi_c - 1e-9) and np.all(lo_c < hi_b - 1e-9)):
                        continue
                    if b_prev[2] - t.BAR[2] / 2 >= hi_c[2] - 1e-9 or axis is None:
                        errors.append(f"step {k}: bar lowered onto {n} ({label})")
                        continue
                    sgn = 1.0 if d[axis] > 0 else -1.0
                    c[axis] = (hi_b[axis] + t.H) if sgn > 0 else (lo_b[axis] - t.H)
                    # chips push chips along the same axis: walk them in push order, each one ahead of any it
                    # overlaps
                    for _ in range(t.N_CHIPS):
                        changed = False
                        order = sorted(chips, key=lambda m: sgn * chips[m][axis])
                        for i, p_ in enumerate(order):
                            cp = chips[p_]
                            for q_ in order[i + 1:]:
                                cq = chips[q_]
                                if abs(cp[2] - cq[2]) < 1e-6 and np.all(t.CHIP - np.abs(cp[:2] - cq[:2]) > 1e-9):
                                    cq[axis] = cp[axis] + sgn * t.CHIP
                                    changed = True
                        if not changed:
                            break
                for n, c in chips.items():
                    if c[2] >= t.PT - 1e-6:
                        if c[0] > t.PLATE_X[1]:
                            drop_into_trough(n)
                        elif not (t.PLATE_X[0] <= c[0] and t.PLATE_Y[0] <= c[1] <= t.PLATE_Y[1]):
                            errors.append(f"step {k}: {n} pushed off the plate at {c[:2].round(4)}")
            fingers = _boxes_fingers(tcp)
            bb = bar_box(bar)
            gaps["bar-workpiece"] = min(gaps["bar-workpiece"], _box_gap(bb, wp))
            gaps["fingers-workpiece"] = min(gaps["fingers-workpiece"], min(_box_gap(f, wp) for f in fingers))
            gaps["bar-walls"] = min(gaps["bar-walls"], min(_box_gap(bb, w) for w in walls))
            gaps["fingers-walls"] = min(gaps["fingers-walls"], min(_box_gap(f, w) for f in fingers for w in walls))
            gaps["fingers-chips"] = min(gaps["fingers-chips"],
                                        min(_box_gap(f, chip_box(c)) for f in fingers for c in chips.values()))
        vel = {n: tuple((c - prev_chips[n]) / DT) for n, c in chips.items()}
        vel["bar"] = tuple((bar - prev_bar) / DT)
        states.append(world({n: tuple(c) for n, c in chips.items()}, bar=tuple(bar), vel=vel))
        prev_chips = {n: c.copy() for n, c in chips.items()}
        prev_bar = bar.copy()

    t._S.reset()
    t._S.last = None
    scores, succ = [], []
    for s in states:  # as in the simulator: success() and score() on the same states each step
        succ.append(t.success(s))
        scores.append(t.score(s))
    # the simulator ends the episode once success has held SUCCESS_HOLD_S (1 s = 20 steps) in a row
    hold, run_len, ends = int(round(SUCCESS_HOLD_S * HZ)), 0, None
    for i, v in enumerate(succ):
        run_len = run_len + 1 if v else 0
        if run_len >= hold:
            ends = i
            break
    rep = dict(seed=seed, plan=plan, steps=steps, plan_steps=int(round(arm.duration * HZ)), scores=scores,
               success_from=ends, held=ends is not None and all(succ[ends:]),
               worst_ik_mm=arm.worst_ik_mm, worst_ik_deg=arm.worst_ik_deg, margins=worst, gaps=gaps, errors=errors,
               states=states, wps=wps, flags=(dict(t._S.lifted), dict(t._S.unbarred)))
    return rep


def _cold_ik(wps) -> tuple[float, float, float]:
    """Each waypoint solved on its own, warm-started from the previous: worst error (mm, deg), smallest margin."""
    q = np.array(__import__("yam_arena.constants", fromlist=["READY"]).READY, float)
    worst_mm = worst_deg = 0.0
    margin = np.inf
    for w in wps:
        q, pe, re = ik.solve(w.pos, ik.matrix_to_quat_xyzw(grip_rotation(w.yaw)), q, base_pos=station.BASES["left"],
                             iters=200)
        worst_mm, worst_deg = max(worst_mm, pe * 1000), max(worst_deg, math.degrees(re))
        margin = min(margin, float(np.degrees(np.minimum(q - ik.LOWER, ik.UPPER - q)).min()))
    return worst_mm, worst_deg, margin


def check_demo_replay() -> None:
    print("3. demo replay (ArmPlan at 20 Hz, kinematic chips, score() every step)")
    overall = np.full(6, np.inf)
    worst_gap = {}
    for seed in SEEDS:
        rep = replay(seed)
        p = rep["plan"]
        cmm, cdeg, cmargin = _cold_ik(rep["wps"])
        overall = np.minimum(overall, rep["margins"])
        for k, v in rep["gaps"].items():
            worst_gap[k] = min(worst_gap.get(k, np.inf), v)
        end = rep["scores"][-1]
        g = rep["gaps"]
        print(f"  layout {seed}: {rep['plan_steps']} plan steps; score {end:.3f}, episode ends (success held 1 s) at step "
              f"{rep['success_from']}; IK {rep['worst_ik_mm']:.3f} mm / {rep['worst_ik_deg']:.4f} deg "
              f"(waypoints {cmm:.3f} mm), min joint margin {rep['margins'].min():.1f} deg; gaps (mm): bar-wp "
              f"{g['bar-workpiece'] * 1000:.1f}, fingers-wp {g['fingers-workpiece'] * 1000:.1f}, bar-walls "
              f"{g['bar-walls'] * 1000:.1f}, fingers-walls {g['fingers-walls'] * 1000:.1f}, fingers-chips "
              f"{g['fingers-chips'] * 1000:.1f}")
        for e in rep["errors"][:5]:
            print(f"    {e}")
        check(not rep["errors"], f"layout {seed}: no bar landing on a chip, no chip off the plate")
        check(end == 1.0 and rep["held"], f"layout {seed}: final score 1.0 and success held")
        check(rep["success_from"] is not None and rep["success_from"] < RUN_LIMIT_STEPS,
              f"layout {seed}: success (held 1 s) within {RUN_LIMIT_STEPS} steps")
        check(rep["plan_steps"] <= RUN_LIMIT_STEPS, f"layout {seed}: plan within the run limit")
        check(rep["worst_ik_mm"] <= 0.5 and rep["worst_ik_deg"] <= 0.1 and cmm <= 0.5 and cdeg <= 0.1,
              f"layout {seed}: IK within 0.5 mm / 0.1 deg")
        check(rep["margins"].min() >= 2.0 and cmargin >= 2.0, f"layout {seed}: joints >= 2 deg inside the limits")
        check(g["bar-workpiece"] >= p.m - 1e-6 and g["fingers-workpiece"] >= 0.005,
              f"layout {seed}: bar >= the plan margin ({p.m * 1000:.2f} mm) and fingers >= 5 mm from the workpiece")
        check(g["bar-walls"] >= 0.002 and g["fingers-walls"] >= 0.001 and g["fingers-chips"] >= 0.001,
              f"layout {seed}: bar/fingers clear of the walls and lip, fingers clear of the chips")
    per_joint = {j: round(float(v), 1) for j, v in zip(JOINTS, overall, strict=True)}
    print(f"  all layouts: min margin to the joint limits per joint (deg) {per_joint}")
    print("  all layouts: smallest gaps (mm) " + ", ".join(f"{k} {v * 1000:.1f}" for k, v in worst_gap.items()))


def main() -> int:
    t0 = time.time()
    check_layouts()
    check_bar_length()
    check_scoring()
    check_demo_replay()
    t.use_layout(t._env_seed())
    print(f"\n{'ALL CHECKS PASSED' if not FAILS else f'{len(FAILS)} CHECK(S) FAILED'} ({time.time() - t0:.0f} s)")
    return 1 if FAILS else 0


if __name__ == "__main__":
    sys.exit(main())
