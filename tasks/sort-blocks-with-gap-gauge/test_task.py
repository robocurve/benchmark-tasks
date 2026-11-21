"""Offline tests: no simulator. Run: PYTHONPATH=<yam-isaac-lab-arena> python test_task.py (or pytest)."""

import json
import math
import os
import pathlib
import types
import warnings

import numpy as np

from yam_arena import ik, station, task_file
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState

HERE = pathlib.Path(__file__).resolve().parent
FAR = np.array([0.0, 0.0, 2.0])  # a TCP nowhere near anything


def load(seed=0):
    os.environ["GAUGE_SORT_LAYOUT"] = str(seed)
    return task_file.load(HERE / "task.py").module


m = load()


def state(width, xy, bottom, yaw=0.0, roll=0.0, vel=0.0):
    r = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    r = r @ np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    half = np.array([m.BLOCK_LEN, width, m.BLOCK_H]) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half @ r.T
    pos = np.array([xy[0], xy[1], bottom - corners[:, 2].min()])
    pts = corners + pos
    return ObjectState(pos=pos, quat_xyzw=ik.matrix_to_quat_xyzw(r), lin_vel=np.array([vel, 0, 0]),
                       ang_vel=np.zeros(3), low=pts.min(0), high=pts.max(0))


def at_start_states(mod=m):
    return {n: state(w, mod.START[n], mod.TOP) for n, w in mod.BLOCKS.items()}


def weight(name):
    return m.block_mass(m.BLOCKS[name]) * m.G


def no_contacts():
    return {n: ({p: 0.0 for p in m._PARTS}, 0.0) for n in m.BLOCKS}


def contacts_for(name, gap, frac=1.0, robot=0.0, on="base"):
    out = no_contacts()
    parts = out[name][0]
    if on == "base":
        parts["gauge_base"] = frac * weight(name)
    else:
        anvils = m.GAUGE_PARTS[gap][1:]
        for p in anvils:
            parts[p] = frac * weight(name) / len(anvils)
    out[name] = (parts, robot)
    return out


def feed(states, grippers, contacts, steps, scorer=None):
    scorer = scorer or m.Scorer()
    m.read_grippers, m.read_contacts = (lambda: grippers), (lambda: contacts)
    for _ in range(steps):
        scorer(dict(states))  # a new dict each step, as the embodiment passes
    return scorer


GO, NOGO = (m.GAUGE_X, m.GAP_Y["go"]), (m.GAUGE_X, m.GAP_Y["nogo"])
OPEN_AWAY = [(FAR, 0.0939), (FAR, 0.0939)]


def with_block(name, st, base=None):
    states = dict(base or at_start_states())
    states[name] = st
    return states


# -- geometry, loading, task.json ---------------------------------------------------------------------------------------


def test_geometry():
    m.check_geometry()
    assert {k for k, w in m.WIDTHS.items() if m.in_tolerance(w)} == {"a", "b", "c"}
    for w in m.WIDTHS.values():
        for g in m.GAPS.values():
            side = abs(w - g) / 2
            assert side >= (0.0008 if w < g else 0.0003) - 1e-12  # drop-in clearance / stay-out overlap, a side
    widths = m.WIDTHS
    nearest = min(abs(widths[i] - widths[o]) for i in "abc" for o in "def")
    assert abs(nearest - 0.0022) < 1e-9  # 2.2 mm between the nearest in- and out-of-tolerance blocks


def test_loads_offline():
    t = task_file.load(HERE / "task.py")
    assert t.scene == "station" and len(t.free_objects) == 6 and t.demo is not None and t.score is not None


def test_task_json_limits():
    d = json.loads((HERE / "task.json").read_text())
    required = ["title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "reset_notes", "fidelity", "scores"]
    assert all(k in d for k in required), set(required) - set(d)
    limits = {"title": 120, "instruction": 1000, "context": 1000, "setup": 4000, "scoring": 4000, "run_limit": 200,
              "fidelity": 500, "scores": 300}
    for k, n in limits.items():
        assert len(d[k]) <= n, (k, len(d[k]), n)
    assert d["category"] == "Impact on jobs" and d["scores"].startswith(("PENDING-RUN", "Dream-1 run: "))


# -- support must be a measured contact, never a small distance ------------------------------------------------


def test_hovering_or_pressed_block_gives_no_reading():
    cases = [  # (block, gap, centre, surface it hovers over, contact part)
        ("block_a", "go", GO, m.BASE_TOP, "base"),  # narrow block above the base of GO
        ("block_f", "go", GO, m.ANVIL_TOP, "anvils"),  # oversize block above the GO anvil tops
        ("block_a", "nogo", NOGO, m.ANVIL_TOP, "anvils"),  # in-tolerance block above the NO-GO anvil tops
        ("block_d", "nogo", NOGO, m.BASE_TOP, "base"),
    ]
    for name, gap, xy, surface, on in cases:
        for lift in (0.0001, 0.0003, 0.0005):
            st = state(m.BLOCKS[name], xy, surface + lift)
            assert m.gauge_pose(st, gap, m.BLOCKS[name]) is not None  # the pose alone would pass: that is the trap
            states = with_block(name, st)
            tcp = st.pos + [0, 0, m.GRASP_DZ - m.BLOCK_H / 2]
            # held by a closed gripper (and the physics says the gauge carries nothing)
            s = feed(states, [(tcp, m.BLOCK_LEN), (FAR, 0.09)], contacts_for(name, gap, 0.0, 0.5, on), 20)
            assert s.checks[name].go is None and s.checks[name].nogo is None
            # gripper open and away, but the block rests on something else (zero gauge force)
            s = feed(states, OPEN_AWAY, contacts_for(name, gap, 0.0, 0.0, on), 20)
            assert s.checks[name].go is None and s.checks[name].nogo is None
            # resting on a finger: an open gripper near it, weight carried by a robot link
            s = feed(states, [(tcp, 0.09), (FAR, 0.09)], contacts_for(name, gap, 0.0, weight(name), on), 20)
            assert s.checks[name].go is None
        # touching the gauge while a robot link also presses on it
        st = state(m.BLOCKS[name], xy, surface)
        for robot in (0.03, 0.5, 5.0):
            s = feed(with_block(name, st), OPEN_AWAY, contacts_for(name, gap, 1.0, robot, on), 20)
            assert s.checks[name].go is None and s.checks[name].nogo is None
        # partial support (block partly carried by something else)
        s = feed(with_block(name, st), OPEN_AWAY, contacts_for(name, gap, 0.6, 0.0, on), 20)
        assert s.checks[name].go is None


def test_closed_gripper_near_block_blocks_reading():
    st = state(m.BLOCKS["block_a"], GO, m.BASE_TOP)
    tcp = st.pos + [0, 0, 0.03]
    s = feed(with_block("block_a", st), [(tcp, 0.064), (FAR, 0.09)], contacts_for("block_a", "go"), 20)
    assert s.checks["block_a"].go is None
    s = feed(with_block("block_a", st), [(tcp, 0.066), (FAR, 0.09)], contacts_for("block_a", "go"), 20)
    assert s.checks["block_a"].go == "in"


# -- genuine readings need 6 steps in a row -----------------------------------------------------------------------------


def test_readings_count_after_six_steps():
    seated = state(m.BLOCKS["block_a"], GO, m.BASE_TOP)
    s = feed(with_block("block_a", seated), OPEN_AWAY, contacts_for("block_a", "go"), 5)
    assert s.checks["block_a"].go is None
    feed(with_block("block_a", seated), OPEN_AWAY, contacts_for("block_a", "go"), 1, s)
    assert s.checks["block_a"].go == "in"

    out = state(m.BLOCKS["block_f"], GO, m.ANVIL_TOP)
    s = feed(with_block("block_f", out), OPEN_AWAY, contacts_for("block_f", "go", on="anvils"), 5)
    feed(with_block("block_f", out), OPEN_AWAY, {}, 1, s)  # one step without a reading breaks the streak
    feed(with_block("block_f", out), OPEN_AWAY, contacts_for("block_f", "go", on="anvils"), 5, s)
    assert s.checks["block_f"].go is None
    feed(with_block("block_f", out), OPEN_AWAY, contacts_for("block_f", "go", on="anvils"), 1, s)
    assert s.checks["block_f"].go == "out" and s.checks["block_f"].verdict() == "red"

    # wedged tilted in the GO gap, touching both anvils, clear of the base
    roll = math.acos(m.GAPS["go"] / m.BLOCKS["block_f"])
    wedged = state(m.BLOCKS["block_f"], (GO[0], GO[1] + 0.001), m.BASE_TOP + 0.0034, roll=roll)
    assert m.gauge_pose(wedged, "go", m.BLOCKS["block_f"]) == "out"
    s = feed(with_block("block_f", wedged), OPEN_AWAY, contacts_for("block_f", "go", on="anvils"), 6)
    assert s.checks["block_f"].go == "out"


def test_first_reading_stands_and_nogo_after_go():
    name = "block_b"
    s = feed(with_block(name, state(m.BLOCKS[name], NOGO, m.ANVIL_TOP)), OPEN_AWAY,
             contacts_for(name, "nogo", on="anvils"), 10)
    assert s.checks[name].nogo is None  # NO-GO before GO is ignored
    feed(with_block(name, state(m.BLOCKS[name], GO, m.BASE_TOP)), OPEN_AWAY, contacts_for(name, "go"), 6, s)
    assert s.checks[name].go == "in" and s.checks[name].nogo is None
    feed(with_block(name, state(m.BLOCKS[name], NOGO, m.ANVIL_TOP)), OPEN_AWAY,
         contacts_for(name, "nogo", on="anvils"), 6, s)
    assert s.checks[name].verdict() == "green"


def test_narrow_block_perched_gives_no_reading():
    name, w = "block_d", m.BLOCKS["block_d"]
    crosswise = state(w, GO, m.ANVIL_TOP, yaw=math.pi / 2)  # long axis across the gap, on the anvil tops
    tilted = state(w, (GO[0], GO[1] + 0.002), m.BASE_TOP + 0.004, roll=math.radians(10))  # looks wedged
    for st in (crosswise, tilted):
        s = feed(with_block(name, st), OPEN_AWAY, contacts_for(name, "go", on="anvils"), 20)
        assert s.checks[name].go is None
    # level at anvil-top height over NO-GO (it cannot span 36 mm, so something else holds it): pose says "out", the
    # width rule refuses it even with full force support
    narrow_on_nogo_anvils = state(w, NOGO, m.ANVIL_TOP)
    assert m.gauge_pose(narrow_on_nogo_anvils, "nogo", w) == "out"
    s = feed(with_block(name, narrow_on_nogo_anvils), OPEN_AWAY, contacts_for(name, "nogo", on="anvils"), 20)
    assert s.checks[name].nogo is None and s.checks[name].go is None


class _T:  # a tensor stand-in: what _np() expects from PhysX views
    def __init__(self, a):
        self.a = np.asarray(a)

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a


def test_friction_is_summed_per_pair_and_fails_closed():
    class View:
        def get_friction_data(self, dt):
            forces = [[0, 0, 0.3], [0, 0, 0.2], [1.0, 0, 0]]
            return _T(forces), None, _T([[2, 0], [0, 1]]), _T([[0, 0], [0, 2]])

    out = m._friction_matrix(View(), (2, 2, 3))
    assert np.allclose(out[0, 0], [0, 0, 0.5]) and np.allclose(out[1, 1], [1, 0, 0]) and not out[0, 1].any()

    n_filters = len(m._PARTS)

    class NoFriction:  # normal forces fine, friction data unavailable
        def get_contact_force_matrix(self, dt):
            return _T(np.zeros((6, n_filters, 3)))

        def get_friction_data(self, dt):
            raise RuntimeError("no friction data")

    cols = {"gauge": {p: [i] for i, p in enumerate(m._PARTS)}, "robot": []}
    m._SIM.update(env=object(), hz_ok=True, failed=False,
                  view=(None, NoFriction(), cols, list(range(6)), [list(range(n_filters))] * 6))
    m._WARNED.clear()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        assert m.sim_contacts() is None and m.sim_contacts() is None  # no reading at all, not normal-only
    assert len(caught) == 1 and "no friction data" in str(caught[0].message)
    m._SIM.update(env=None, view=None, failed=False)


def test_cheap_strategies_score_zero():
    os.environ["GAUGE_SORT_LAYOUT"] = "0"
    for variant in ("nothing", "no_checks", "all_green", "held_checks"):
        os.environ["GAUGE_SORT_VARIANT"] = variant
        v = task_file.load(HERE / "variants.py").module
        scorer, states, events, _ = replay(v)
        assert scorer._score == 0.0, (variant, scorer._score)
        checks = sum(c.go is not None or c.nogo is not None for c in scorer.checks.values())
        if variant != "nothing":
            assert all(e for e in events.values()), variant  # every block was moved
        assert checks == 0, variant
    del os.environ["GAUGE_SORT_VARIANT"]
    load(0)


def _stl_bounds(path):
    pts = np.array([[float(v) for v in line.split()[1:]] for line in path.read_text().splitlines()
                    if line.strip().startswith("vertex")])
    return pts.min(0), pts.max(0)


def test_build_files_match_task():
    b = HERE / "build"
    mm = 1000.0
    lo, hi = _stl_bounds(b / "gauge.stl")  # base bottom centre at the origin
    assert np.allclose(hi - lo, [m.BASE_SIZE[0] * mm, m.BASE_SIZE[1] * mm, (m.ANVIL_TOP - m.TOP) * mm], atol=1e-3)
    for k, w in m.WIDTHS.items():
        lo, hi = _stl_bounds(b / f"block_{k}.stl")
        assert np.allclose(hi - lo, [m.BLOCK_LEN * mm, w * mm, m.BLOCK_H * mm], atol=1e-3), k
    d = json.loads((b / "dimensions.json").read_text())
    assert np.allclose(d["base"], np.array(m.BASE_SIZE) * mm) and np.allclose(d["anvil"], np.array(m.ANVIL_SIZE) * mm)
    assert abs(d["anvil_top_above_base_bottom"] - (m.ANVIL_TOP - m.TOP) * mm) < 1e-6
    for gap in m.GAPS:
        assert abs(d["gaps"][gap]["width"] - m.GAPS[gap] * mm) < 1e-6
        assert abs(d["gaps"][gap]["centre_y"] - m.GAP_Y[gap] * mm) < 1e-6
        faces = sorted(d["gaps"][gap]["anvil_faces_y"])
        assert np.allclose(np.diff(faces), [m.ANVIL_SIZE[1] * mm, m.GAPS[gap] * mm, m.ANVIL_SIZE[1] * mm])
    assert d["widths"] == {k.upper(): round(w * mm, 4) for k, w in m.WIDTHS.items()}
    assert (d["block_length"], d["block_height"]) == (m.BLOCK_LEN * mm, m.BLOCK_H * mm)
    assert d["in_tolerance"] == ["A", "B", "C"]
    assert (d["drop_clearance_per_side_min"], d["stay_overlap_per_side_min"]) == (m.DROP_CLEAR * mm, m.STAY_OVERLAP * mm)


def test_plan_length_and_run_limit_margin():
    for seed in (0, 1, 2, 3):
        mod = load(seed)
        seconds = {round(sum(w.duration for w in wps), 1) for wps in mod.demo().values()}
        assert seconds == {229.8}, seconds
    steps = round(229.8 * mod.CONTROL_HZ)  # 4,596
    assert mod.RUN_LIMIT_STEPS / (steps + 2 * mod.CONTROL_HZ) >= 2.0  # at least twice the demo with its 2 s hold
    readme = (HERE / "README.md").read_text()
    assert "229.8 s" in readme and "4,596" in readme and "9,600" in readme
    assert json.loads((HERE / "task.json").read_text())["run_limit"].startswith("9,600 steps")
    load(0)


def test_deeply_tipped_oversize_block_reads_out():
    f, d = m.BLOCKS["block_f"], m.BLOCKS["block_d"]
    tipped = state(f, (GO[0], GO[1] + 0.0016), m.BASE_TOP + 0.0019, roll=math.radians(18.4))  # as in the wedge probe
    assert m.gauge_pose(tipped, "go", f) == "out"
    s = feed(with_block("block_f", tipped), OPEN_AWAY, contacts_for("block_f", "go", on="anvils"), 6)
    assert s.checks["block_f"].go == "out"
    leaning = state(d, (GO[0], GO[1] + 0.004), m.BASE_TOP, roll=math.radians(15))  # narrow block leaning in the gap
    s = feed(with_block("block_d", leaning), OPEN_AWAY, contacts_for("block_d", "go", on="anvils"), 20)
    assert s.checks["block_d"].go is None


def test_tipped_block_on_anvils_stays_out():
    w = m.BLOCKS["block_a"]
    for roll, dy in ((0.0, 0.0), (math.radians(2.5), 0.0015), (math.radians(14), 0.003), (math.radians(26), 0.004)):
        st = state(w, (NOGO[0], NOGO[1] + dy), m.ANVIL_TOP - (0.002 if roll else 0.0), roll=roll)
        assert m.gauge_pose(st, "nogo", w) == "out", (roll, dy)
    assert m.gauge_pose(state(w, NOGO, m.ANVIL_TOP + 0.003), "nogo", w) is None  # held 3 mm above the anvils
    assert m.gauge_pose(state(w, NOGO, m.ANVIL_TOP - 0.002, roll=math.radians(35)), "nogo", w) is None  # too far over


def test_reader_unavailable_fails_closed():
    seated = with_block("block_a", state(m.BLOCKS["block_a"], GO, m.BASE_TOP))

    def boom():
        raise RuntimeError("no physics view")

    cases = ((None, contacts_for("block_a", "go"), 1), (OPEN_AWAY, None, 1), (boom, None, 2), (None, boom, 2))
    for g, c, failing in cases:
        m._WARNED.clear()
        s = m.Scorer()
        m.read_grippers = boom if g is boom else (lambda g=g: g)
        m.read_contacts = boom if c is boom else (lambda c=c: c)
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            for _ in range(20):
                assert s(dict(seated)) == 0.0
        assert s.checks["block_a"].go is None
        assert len(caught) == failing  # one warning per failing reader, not one per step
    m._SIM.update(env=None, view=None, failed=False)
    assert m.sim_grippers() is None and m.sim_contacts() is None  # real readers without a simulation


# -- kinematic replay of the demo plan with idealised physics ------------------------------------------------------------


LAG = {"left": 0.0025, "right": -0.0025}  # larger than RELEASE_DY: the align stages must take up the rest


def replay(mod, hz=20):
    """Plays demo() kinematically: a block is carried while a closed gripper holds it and drops straight onto what is
    under it when the gripper opens. Fake readers report what an ideal simulator would. Returns (scorer, states, log)."""
    plans = {a: ArmPlan(w, station.BASES[a]) for a, w in mod.demo().items()}
    mod = getattr(mod, "T", mod)  # variants.py wraps task.py: the scorer and readers live there
    end = max(p.duration for p in plans.values())
    blocks = {n: {"pos": np.array([*mod.START[n], mod.TOP]), "held": None, "roll": 0.0} for n in mod.BLOCKS}
    events = {n: [] for n in mod.BLOCKS}
    log = {"tcp": [], "steps": 0}
    scorer = mod._SCORER
    scorer.reset()

    def surface(n, xy):
        w = mod.BLOCKS[n]
        for gap in mod.GAPS:
            if abs(xy[0] - mod.GAUGE_X) <= mod.X_TOL and abs(xy[1] - mod.GAP_Y[gap]) < mod.GAPS[gap] / 2:
                return (mod.BASE_TOP if mod.drops_in(w, gap) else mod.ANVIL_TOP), gap
        for tray in mod.TRAY_Y:
            lo, hi = mod.tray_inner(tray)
            if np.all(xy >= lo) and np.all(xy <= hi):
                return mod.TRAY_FLOOR_TOP, tray
        return mod.TOP, "table"

    grip = {}
    mod.read_grippers = lambda: [grip[a] for a in ("left", "right")]

    def contacts():
        out = {}
        for n, b in blocks.items():
            parts = {p: 0.0 for p in mod._PARTS}
            wt = mod.block_mass(mod.BLOCKS[n]) * mod.G
            where = events[n][-1] if events[n] else None
            if b["held"] is None and where in mod.GAPS:
                if abs(b["pos"][2] - mod.BASE_TOP) < 1e-9:
                    parts["gauge_base"] = wt
                else:
                    for p in mod.GAUGE_PARTS[where][1:]:
                        parts[p] = wt / (len(mod.GAUGE_PARTS[where]) - 1)
            out[n] = (parts, 2.0 if b["held"] else 0.0)
        return out

    mod.read_contacts = contacts
    mod.read_block = lambda n: blocks[n]["pos"] + [0, 0, mod.BLOCK_H / 2]  # what the demo sees: the block's centre
    for k in range(int(end * hz) + 2 * hz):
        t = k / hz
        tcps = {}
        for arm, plan in plans.items():
            pos, _, finger, _ = plan.target(t)
            opening = 2 * abs(finger)
            held = next((n for n, b in blocks.items() if b["held"] == arm), None)
            if held is None and opening <= mod.BLOCK_LEN:
                for n, b in blocks.items():
                    if b["held"] is None and np.hypot(*(b["pos"][:2] - pos[:2])) < 0.003 \
                            and abs(b["pos"][2] + mod.GRASP_DZ - pos[2]) < 0.003:
                        b["held"], held = arm, n
            if held is not None and opening > mod.BLOCK_LEN + 0.001:
                b = blocks[held]
                b["held"] = None
                z, where = surface(held, b["pos"][:2])
                b["pos"] = np.array([*b["pos"][:2], z])
                events[held].append(where)
                held = None
            if held is not None:  # over the gauge the arm lags its target by LAG across the slot
                lag = LAG[arm] if abs(pos[0] - mod.GAUGE_X) <= mod.X_TOL and abs(pos[1]) < 0.1 else 0.0
                blocks[held]["pos"] = pos - [0, lag, mod.GRASP_DZ]
                opening = mod.BLOCK_LEN
            grip[arm] = (pos, opening)
            tcps[arm] = pos
        log["tcp"].append((tcps["left"], tcps["right"]))
        states = {n: state(mod.BLOCKS[n], b["pos"][:2], b["pos"][2], vel=0.1 if b["held"] else 0.0)
                  for n, b in blocks.items()}
        mod.score(states)
        log["steps"] = k + 1
    return scorer, states, events, log


def check_plan(mod, events, log):
    mod.read_block = lambda n: None  # the nominal plan: no block poses to adjust to
    plan = mod.demo()
    for arm, wps in plan.items():
        for w in wps:
            x, y, z = w.pos
            assert 0.10 <= x <= 0.45 and abs(y) <= 0.35 and mod.TOP + 0.02 <= z <= mod.TOP + 0.12, (arm, w)
    zone = lambda p: abs(p[0] - mod.GAUGE_X) <= 0.08 and abs(p[1]) <= 0.13  # noqa: E731
    for left, right in log["tcp"]:
        assert not (zone(left) and zone(right)), "both arms in the gauge zone"
        assert np.linalg.norm(left - right) >= 0.12, "TCPs closer than 12 cm"
    for n, w in mod.BLOCKS.items():
        want = ["go", "nogo"] if mod.drops_in(w, "go") else ["go"]
        assert events[n] == want + ["green" if mod.in_tolerance(w) else "red"], (n, events[n])
    assert log["steps"] < mod.RUN_LIMIT_STEPS


def test_demo_plan_sorts_every_layout():
    global m
    for seed in (0, 1, 2, 7):
        mod = load(seed)
        scorer, states, events, log = replay(mod)
        check_plan(mod, events, log)
        assert scorer._score == 1.0, (seed, scorer._score, {n: (c.go, c.nogo) for n, c in scorer.checks.items()})
    m = load(0)


def test_demo_plan_ik():
    mod = load(3)
    for arm, wps in mod.demo().items():
        plan = ArmPlan(wps, station.BASES[arm])
        for k in range(int(plan.duration * 10) + 1):
            plan.action7(k / 10)
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 1.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- tray rules, escapes, reset -------------------------------------------------------------------------------------------


def final_sorted(mod=m):
    used, out = {"green": 0, "red": 0}, {}
    for n, w in mod.BLOCKS.items():
        tray = "green" if mod.in_tolerance(w) else "red"
        out[n] = state(w, (mod.TRAY_X, mod.TRAY_Y[tray] + mod.TRAY_SLOTS_Y[used[tray]]), mod.TRAY_FLOOR_TOP)
        used[tray] += 1
    return out


def test_correct_trays_without_checks_score_zero():
    s = feed(final_sorted(), OPEN_AWAY, {}, 50)
    assert s._score == 0.0


def test_escape_caps_score():
    scorer, states, _, _ = replay(m)
    assert scorer._score == 1.0
    escaped = dict(states)
    a = states["block_a"]
    escaped["block_d"] = state(m.BLOCKS["block_d"], a.pos[:2], a.high[2])  # D stacked on A in the green tray
    assert scorer(escaped) == 0.25


def test_tray_needs_release_rest_and_support():
    w, xy = m.BLOCKS["block_a"], (m.TRAY_X, m.TRAY_Y["green"])

    def placed(st, grippers=OPEN_AWAY, **others):
        states = with_block("block_a", st)
        states.update(others)
        return m.tray_contents(states, grippers)

    floor = state(w, xy, m.TRAY_FLOOR_TOP)
    assert placed(floor)["block_a"] == "green"
    assert "block_a" not in placed(state(w, xy, m.TRAY_FLOOR_TOP + 0.01))  # held above the floor
    assert "block_a" not in placed(state(w, xy, m.TRAY_FLOOR_TOP, vel=0.05))  # still moving
    (x0, _), _ = m.tray_inner("green")
    assert "block_a" not in placed(state(w, (x0, xy[1]), m.TRAY_FLOOR_TOP + m.TRAY_WALL_H))  # across the near wall
    assert "block_a" not in placed(state(w, (x0 + 0.02, xy[1]), m.TRAY_FLOOR_TOP))  # footprint over the wall
    tcp = floor.pos + [0, 0, m.GRASP_DZ - m.BLOCK_H / 2]
    assert "block_a" not in placed(floor, [(tcp, m.BLOCK_LEN), (FAR, 0.09)])  # gripped while touching the floor
    assert "block_a" not in placed(floor, None)  # no gripper reading: fail closed
    # stacks resolve bottom-up: on a block lying in the tray counts, on a block held above the floor does not
    b_floor = state(m.BLOCKS["block_b"], xy, m.TRAY_FLOOR_TOP)
    assert placed(state(w, xy, b_floor.high[2]), block_b=b_floor) == {"block_a": "green", "block_b": "green"}
    b_held = state(m.BLOCKS["block_b"], xy, m.TRAY_FLOOR_TOP + 0.01)  # held still, its gripper out of reach of A
    assert placed(state(w, xy, b_held.high[2]), block_b=b_held) == {}


def test_escape_cap_covers_held_over_and_rim():
    scorer, states, _, _ = replay(m)
    assert scorer._score == 1.0
    d, g = m.BLOCKS["block_d"], (m.TRAY_X, m.TRAY_Y["green"])
    (x0, _), _ = m.tray_inner("green")
    probes = [
        state(d, (g[0], g[1]), m.TOP + 0.08, vel=0.1),  # held over the green tray
        state(d, (x0 - 0.01, g[1]), m.TRAY_FLOOR_TOP + m.TRAY_WALL_H),  # lying on the near wall, one end in
        state(d, (x0 - 0.031, g[1]), m.TOP),  # on the table, touching the tray's outer face
    ]
    for st in probes:
        s = dict(states)
        s["block_d"] = st
        assert m.escapes(s) == ["block_d"] and m.score(s) == 0.25
    clear = dict(states)
    clear["block_d"] = state(d, (x0 - 0.04, g[1]), m.TOP)  # 4 mm clear of the tray: no escape
    assert m.escapes(clear) == []


class _Emb:
    def __init__(self, hz=20.0, module=None):
        self._env, self._t = object(), 0.0
        self.info = types.SimpleNamespace(control_hz=hz)
        self.task = types.SimpleNamespace(module=module)  # the SDK's TaskFile.module: the copy it loaded itself

    def reset(self, *a, **k):
        self._t = 0.0
        return "obs"

    def _task_step(self, *a, **k):
        return "step"


def stub_embodiment(mod, hz=20.0, own=None):
    ie = types.SimpleNamespace(YamArenaEmbodiment=type("YamArenaEmbodiment", (_Emb,), {}))
    mod._patch_embodiment(ie, mod._FINDER.gauge_sort_route)
    return ie.YamArenaEmbodiment(hz, own or mod)


def test_reset_clears_checks_between_trials():
    global m
    mod = load(0)
    emb = stub_embodiment(mod)
    _, states, _, _ = replay(mod)
    assert mod._SCORER._score == 1.0
    assert emb.reset() == "obs"  # the embodiment's reset, wrapped by task.py
    mod.read_grippers = mod.read_contacts = lambda: None
    shifted = {n: state(w, np.add(mod.START[n], (0.02, 0.0)), mod.TOP) for n, w in mod.BLOCKS.items()}
    assert not any(mod.at_start(n, shifted[n]) for n in mod.BLOCKS)  # the backup wipe cannot fire here
    assert mod.score(shifted) == 0.0
    assert all(c.go is None and c.nogo is None for c in mod._SCORER.checks.values())
    assert mod.score(dict(states)) == 0.0  # the old trial's sorted trays earn nothing now
    replay(mod)
    emb._t = 5.0
    assert emb._task_step() == "step"
    emb._t = 0.05  # episode clock went backwards without a reset call
    emb._task_step()
    assert all(c.go is None and c.nogo is None for c in mod._SCORER.checks.values())
    m = load(0)


def test_double_load_routes_to_the_embodiments_own_copy():
    global m
    m1, m2 = load(0), load(0)
    for own, other in ((m1, m2), (m2, m1)):  # embodiment loaded the first copy (a later load happened), or the second
        for mod in (m1, m2):
            mod._SIM.update(env=None, view=None, failed=False)
        emb = stub_embodiment(m1, own=own)  # patched by the first copy, as the meta-path hook would
        emb.reset()
        emb._task_step()
        assert own._SIM["env"] is emb._env and other._SIM["env"] is None
        assert own._sim_env("probe") is emb._env
    slow = stub_embodiment(m2, hz=10.0)
    slow._task_step()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        assert m2._sim_env("hz") is None  # wrong control rate: fail closed
    assert "control rate" in str(caught[0].message)
    m = load(0)


def test_broken_embodiment_never_raises():
    global m
    mod = load(0)
    emb = stub_embodiment(mod)
    del emb._t
    mod._WARNED.clear()
    with warnings.catch_warnings(record=True) as caught:
        warnings.simplefilter("always")
        assert emb._task_step() == "step" and emb._task_step() == "step"
        assert emb.reset() == "obs"
    assert len([w for w in caught if "embodiment" in str(w.message)]) == 1
    del emb._env
    emb._task_step()
    assert mod._SIM["env"] is None and mod.sim_grippers() is None  # no reading, no exception
    m = load(0)


def test_malformed_reader_values_give_no_reading():
    name = "block_a"
    seated = with_block(name, state(m.BLOCKS[name], GO, m.BASE_TOP))
    tcp = seated[name].pos + [0, 0, 0.02]
    bad_grippers = [
        [(np.full(3, np.nan), m.BLOCK_LEN), (FAR, 0.09)],  # a closed gripper on the block with a NaN position
        [(tcp, float("nan")), (FAR, 0.09)],
        [],  # no arms: must not pass "every nearby gripper is open" vacuously
        [(tcp, 0.09)],  # one arm missing
        [(np.zeros(2), 0.09), (FAR, 0.09)],  # wrong shape
        "garbage",
    ]
    for g in bad_grippers:
        s = feed(seated, g, contacts_for(name, "go"), 20)
        assert s.checks[name].go is None, g
    good = contacts_for(name, "go")
    nan = contacts_for(name, "go")
    nan[name][0]["gauge_base"] = float("nan")
    neg = contacts_for(name, "go", robot=-1.0)
    bad_contacts = [{}, {name: good[name]}, nan, neg, {n: 1.0 for n in m.BLOCKS}, [1, 2, 3]]
    for c in bad_contacts:
        s = feed(seated, OPEN_AWAY, c, 20)
        assert s.checks[name].go is None
    assert feed(seated, OPEN_AWAY, good, 6).checks[name].go == "in"  # the same scene with good readings


def test_wedge_probe_plan():
    global m
    os.environ["GAUGE_SORT_PROBE"] = "wedge"
    try:
        mod = load(0)
        plan = mod.demo()
    finally:
        del os.environ["GAUGE_SORT_PROBE"]
    (arm, wps), = plan.items()
    arm_plan = ArmPlan(wps, station.BASES[arm])
    for k in range(int(arm_plan.duration * 10) + 1):
        arm_plan.action7(k / 10)
    assert arm_plan.worst_ik_mm < 1.0 and wps[-1].label == "dwell wedged"
    assert abs(wps[-1].pos[1] - (mod.GAP_Y["go"] + mod.RELEASE_DY[arm] + mod.WEDGE_DY)) < 1e-9
    m = load(0)


def test_checks_reset_when_blocks_back_at_start():
    scorer, states, _, _ = replay(m)
    assert scorer._score == 1.0
    scorer(at_start_states())
    assert all(c.go is None and c.nogo is None for c in scorer.checks.values())
    assert scorer(dict(states)) == 0.0


if __name__ == "__main__":
    tests = [(k, v) for k, v in dict(globals()).items() if k.startswith("test_")]
    for k, fn in tests:
        fn()
        print("PASS", k)
    print(f"{len(tests)} passed")
