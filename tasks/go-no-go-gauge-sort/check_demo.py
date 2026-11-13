"""Offline checks for gauge_sort.py, no simulator (NumPy and SciPy, with yam_arena importable):

    python tasks/go-no-go-gauge-sort/check_demo.py        (or: python tasks/go-no-go-gauge-sort/gauge_sort.py)

0. The starting layout: every block at x >= 0.16, in its own arm's row, with finger room between neighbours and
   clear of the gauge and the trays.
1. Scoring rules on hand-built states, the gauge's dimensions and the expected score of random sorting.
2. The gauge reading (gauge_sort.seated) on blocks seated in a gap, lying on the anvils and jammed tilted.
3. The demo, replayed through yam_arena.demo.ArmPlan at 20 Hz for both arms, with the blocks' resting places
   tracked so the demo's gauge readings resolve: the physical scenario (a block drops into a gap exactly when it
   is narrower), every block staying out of GO, every block dropping through both gaps, and every block passing,
   which between them take each arm through every branch. Checked: joints inside the URDF limits, IK error under
   0.5 mm / 0.1 deg, the arms' link-3/4/5, gripper and TCP frame origins at least ARM_CLEARANCE apart, every
   block ending in the tray its gauge readings call for, the physical scenario scoring 1.0, every let-go over a
   gap following a hold of SETTLE_S at the release point with the fingers shut, and the run fitting in the run
   limit (6,000 steps).
"""

from __future__ import annotations

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


# -- small helpers -----------------------------------------------------------------------------------------------
def _state(pos, size, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    return ObjectState(pos, np.array([0, 0, 0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def _world(where: dict, vel=None) -> dict:
    out = {}
    for name, (w, x, y) in g.PARTS.items():
        pos = where.get(name, (x, y, g.TOP + g.PART_H / 2))
        out[name] = _state(pos, (g.PART_LEN, w, g.PART_H), (vel or {}).get(name, (0, 0, 0)))
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


def check_layout() -> None:
    """Every block starts at x >= MIN_START_X, in its own arm's row, apart from the others with room for the open
    fingers, and clear of the gauge and the trays."""
    assert -FINGER_OPEN >= -g.FINGER_WIDE > -g.FINGER_AJAR  # wider than a release, inside the gripper's travel
    blocks = {o.name: o for o in g.OBJECTS if o.name in g.PARTS}
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
    print("layout: every block at x >= 0.16, in its own arm's row, finger room between blocks, clear of gauge/trays")


# -- 1. scoring --------------------------------------------------------------------------------------------------
def check_scoring() -> float:
    task = load(HERE / "gauge_sort.py")
    assert set(task.free_objects) == set(g.PARTS) and len(task.objects) == 7 + 5 + 10
    assert len(g.PASS) == 3 and len(g.REJECT) == 4
    for name, (w, _, _) in g.PARTS.items():  # every part clears its nearest limit by >= 1.5 mm
        assert min(abs(w - g.LOWER), abs(w - g.UPPER)) >= g.MARGIN - 1e-9, name
    widths = sorted(w for w, _, _ in g.PARTS.values())  # the tightest case of each kind is in the batch
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
    widest = max(w for w, _, _ in g.PARTS.values())  # the widest part fits a tray spot clear of its neighbour
    assert g.SLOTS_Y[1] + widest / 2 < g.TRAY_IN[1] / 2
    assert g.SLOTS_Y[1] - g.SLOTS_Y[0] > widest + 0.004
    assert g.SLOTS_Y[0] - widest / 2 > 0.020  # and over 20 mm from the midline: the arms' spots never touch
    assert all(o.mass is None for o in g.OBJECTS if o.name not in g.PARTS)
    check_layout()

    start = _world({})
    assert g.score(start) == 0.0 and not g.success(start)
    pass_k = reject_k = 0
    sorted_ = {}
    for name in g.PARTS:
        if name in g.PASS:
            sorted_[name] = _in("pass_tray", pass_k)
            pass_k += 1
        else:
            sorted_[name] = _in("reject_tray", reject_k)
            reject_k += 1
    done = _world(sorted_)
    assert g.success(done) and g.score(done) == 1.0
    moving = _world(sorted_, vel={"part_1": (0, 0, -0.2)})  # still moving: not counted
    assert not g.success(moving) and g.score(moving) == 6 / 7
    held = dict(sorted_, part_1=(g.TRAYS["pass_tray"][0], 0.045, g.RIM + 0.10))  # held over the tray: not in
    assert g.score(_world(held)) == 6 / 7
    passes = sorted(g.PASS)  # a second layer counts
    stacked = dict(sorted_, **{passes[2]: _in("pass_tray", 4)})  # on top of passes[0]
    assert g.success(_world(stacked)) and g.score(_world(stacked)) == 1.0
    on_rim = dict(sorted_, part_2=(g.TRAYS["reject_tray"][0] - g.TRAY_IN[0] / 2 - g.WALL_T / 2, 0.0,
                                   g.RIM + g.PART_H / 2))  # on a wall top: not in
    assert g.score(_world(on_rim)) == 6 / 7
    all_pass = _world({n: _in("pass_tray", i) for i, n in enumerate(g.PARTS)})  # one tray for all: 0 either way
    all_reject = _world({n: _in("reject_tray", i) for i, n in enumerate(g.PARTS)})
    assert g.score(all_pass) == 0.0 and g.score(all_reject) == 0.0
    # one escape: 4/7 capped at 0.25; two escapes: 1/7; one false reject: 4/7
    one_escape = dict(sorted_, part_2=_in("pass_tray", 4))
    two_escapes = dict(one_escape, part_4=_in("pass_tray", 5))
    one_false = dict(sorted_, part_1=_in("reject_tray", 4))
    assert g.score(_world(one_escape)) == 0.25 and g.score(_world(two_escapes)) == 1 / 7
    assert g.score(_world(one_false)) == (6 - 2) / 7
    # an escape among a partial run: 3 right + 1 escape -> 1/7, under the cap
    partial = {n: sorted_[n] for n in ("part_1", "part_5", "part_6")}
    partial["part_2"] = _in("pass_tray", 4)
    assert g.score(_world(partial)) == 1 / 7
    assert g.score(_world({n: sorted_[n] for n in g.PASS})) == 3 / 7  # only the passes sorted
    assert g.score(_world({n: sorted_[n] for n in g.REJECT})) == 4 / 7  # only the rejects sorted
    # blind guessing: every part into either tray at random, all 2^7 ways equally likely
    names = list(g.PARTS)
    total = 0.0
    for pick in product(("pass_tray", "reject_tray"), repeat=len(names)):
        counts = {"pass_tray": 0, "reject_tray": 0}
        where = {}
        for n, t in zip(names, pick, strict=True):
            where[n] = _in(t, counts[t])
            counts[t] += 1
        total += g.score(_world(where))
    guess = total / 2 ** len(names)
    assert guess < 0.1, guess
    print(f"scoring: all cases pass; blind random sorting expects {guess:.3f}")
    return guess


# -- 2. the gauge reading ----------------------------------------------------------------------------------------
def check_reading() -> None:
    def at(gap, w, bottom, dy=0.0, tall=g.PART_H):
        return _state((g.GAUGE_X, g.GAPS[gap][0] + dy, bottom + tall / 2), (g.PART_LEN, w, tall))

    def wedged(gap, w):  # tipped about one anvil edge until its low corner meets the other anvil's face
        gap_w = g.GAPS[gap][1]
        th = np.arccos(gap_w / w)
        drop = w * np.sin(th)
        return at(gap, w, g.ANVIL_TOP - drop, dy=0.002, tall=drop + g.PART_H * np.cos(th))

    for gap, (_, gap_w) in g.GAPS.items():
        small, big = gap_w - g.MARGIN, gap_w + g.MARGIN
        assert g.seated(at(gap, small, g.BASE_TOP), gap)
        assert g.seated(at(gap, small, g.BASE_TOP, dy=0.0007), gap)  # off-centre by its clearance
        assert not g.seated(at(gap, big, g.ANVIL_TOP), gap)  # on the anvils
        assert not g.seated(wedged(gap, big), gap)  # jammed tilted
        assert not g.seated(wedged(gap, gap_w + 0.005), gap)
        assert not g.seated(at(gap, small, g.BASE_TOP + 0.012), gap)  # still on its way down / held
        assert not g.seated(at(gap, small, g.BASE_TOP), "go" if gap == "nogo" else "nogo")  # the other gap
    print("gauge reading: seated / on the anvils / jammed / held / other gap all read right")


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
            over_gap = any(abs(p[0] - g.GAUGE_X) < 1e-9 and abs(p[1] - gy) < 1e-9 for gy, _ in g.GAPS.values())
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
    try:
        plans_wp, insp, schedule = g.demo_plan()
        plans = {arm: ArmPlan(wps, station.BASES[arm]) for arm, wps in plans_wp.items()}
        steps = round(max(p.duration for p in plans.values()) * HZ) + 5
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
            if g.tray_of(final[name]) != want:
                problems.append(f"{name}: gauge says {want}, ends at {np.round(blocks.pos[name], 3)}")
        states = dict(final)
        states.update({o.name: _state(o.pos, o.size) for o in g.OBJECTS if o.mass is None})
        return {
            "title": title, "steps": steps, "closest_mm": closest * 1000, "closest_at": closest_at,
            "joints_ok": joints_ok, "ik": {a: (p.worst_ik_mm, p.worst_ik_deg) for a, p in plans.items()},
            "score": g.score(states), "success": g.success(states), "problems": problems,
            "schedule": schedule, "verdicts": {n: insp.verdict(n) for n in g.PARTS},
        }
    finally:
        g._SENSE = None


def physical(name: str, gap: str) -> bool:
    """A block drops into a gap exactly when it is narrower."""
    return g.PARTS[name][0] < g.GAPS[gap][1]


def scenarios():
    yield "physical (widths vs gaps)", physical
    yield "every block stays out of GO", lambda n, gap: False
    yield "every block drops through both gaps", lambda n, gap: True
    yield "every block passes", lambda n, gap: gap == "go"


def check_demo() -> dict:
    failed, physical_result = False, None
    for title, outcome in scenarios():
        r = run(outcome, title)
        ik_txt = ", ".join(f"{a} {mm:.3f} mm / {deg:.4f} deg" for a, (mm, deg) in r["ik"].items())
        print(f"- {title}: {r['steps']} steps ({r['steps'] / HZ:.1f} s), score {r['score']:.3f}"
              f"{' success' if r['success'] else ''}; worst IK {ik_txt}; joints "
              f"{'inside' if r['joints_ok'] else 'OUTSIDE'} the limits; arms' wrist/TCP frames >= "
              f"{r['closest_mm']:.0f} mm apart (t={r['closest_at'][0]:.1f}: {r['closest_at'][1]!r} / {r['closest_at'][2]!r})")
        for p in r["problems"]:
            print("    ", p)
        failed |= not (
            not r["problems"] and r["joints_ok"] and r["steps"] <= RUN_LIMIT_STEPS
            and all(mm < 0.5 and deg < 0.1 for mm, deg in r["ik"].values())
            and r["closest_mm"] >= ARM_CLEARANCE * 1000
        )
        if outcome is physical:
            physical_result = r
    r = physical_result
    assert r["score"] == 1.0 and r["success"], r
    print(f"physical scenario verdicts: {r['verdicts']}")
    print("gauge turns (block, from - to, s): " + ", ".join(f"{n} {a:.1f}-{b:.1f}" for n, a, b in r["schedule"]))
    assert not failed, "demo check failed"
    return r


def main() -> None:
    check_scoring()
    check_reading()
    check_demo()
    print("gauge_sort offline checks passed")


if __name__ == "__main__":
    sys.exit(main())
