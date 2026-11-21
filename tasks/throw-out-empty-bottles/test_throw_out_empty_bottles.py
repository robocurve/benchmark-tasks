"""Checks that need no GPU: the task file loads, the bottles look identical, the bin holds them, the demo tests
every bottle and is reachable for every draw, every reset draws afresh (unless FULL fixes the draw), and the scorer
latches tipping and falling for the whole trial and credits only empty bottles resting on the bin floor, let go."""

import itertools
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from jointkit import PartState, rot_z  # noqa: E402
from yam_arena.task_file import load  # noqa: E402


def test_loads():
    t = load(HERE / "task.py")
    assert t.score and t.demo and t.success is None  # no early end: every bottle gets tested


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_bottles_look_identical():
    bottles = [o for o in task.OBJECTS if o.name in task.NAMES]
    assert len({(o.boxes, o.color) for o in bottles}) == 1


def test_bin_holds_four():
    assert task.IN_Y >= 2 * (2 * task.R) + 0.04 and task.IN_X >= 2 * (2 * task.R) + 0.01


def test_demo_tests_every_bottle_and_reaches():
    for full in itertools.combinations(range(4), 2):
        plan = task.plan(full)
        lifts = [w for ws in plan.values() for w in ws if w[3].startswith("lift it")]
        assert len(lifts) == 2 * 4, full
        for arm, ws in plan.items():
            p = ArmPlan([Waypoint(d, tuple(q), f, label, yaw) for d, q, f, label, yaw in ws], station.BASES[arm])
            for k in range(int(p.duration * 20) + 5):
                p.action7((k + 1) / 20)
            assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, (full, arm)


def fresh(monkeypatch, full=None):
    """A newly loaded copy of the task file, with FULL set to ``full`` (or unset)."""
    if full is None:
        monkeypatch.delenv("FULL", raising=False)
    else:
        monkeypatch.setenv("FULL", full)
    return load(HERE / "task.py").module


def bottle(t, x, y, lift=0.0, tilt_deg=0.0, speed=0.0):
    """A bottle's state: base centre at (x, y), ``lift`` m above the bin floor's top, tilted about x."""
    a = math.radians(tilt_deg)
    r = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]]) @ rot_z(0.3)
    pos = np.array([x, y, t.TOP + t.FLOOR + lift])
    if tilt_deg:  # resting on the rim: the base centre rises by R sin(tilt)
        pos[2] += t.R * math.sin(a)
    return PartState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.array([speed, 0.0, 0.0]),
                     ang_vel=np.zeros(3), low=pos - 0.1, high=pos + 0.1, R=r)


def scene(t, full, **over):
    """Full bottles standing in their places on the table, empty ones resting on the bin floor; ``over`` replaces
    some bottles' states."""
    out, slots = {}, list(t.SLOTS)
    for i, n in enumerate(t.NAMES):
        if i in full:
            s = bottle(t, t.BX, t.BY[i])
            out[n] = PartState(pos=s.pos - np.array([0.0, 0.0, t.FLOOR]), quat_xyzw=s.quat_xyzw, lin_vel=s.lin_vel,
                               ang_vel=s.ang_vel, low=s.low, high=s.high, R=s.R)
        else:
            out[n] = bottle(t, t.CX, slots.pop(0))
    out.update(over)
    return out


def test_every_reset_draws_afresh(monkeypatch):
    t = fresh(monkeypatch)
    draws = []
    for _ in range(40):
        coming = t._DRAW["next"]
        plan = t.plan()  # the demo plans for the coming trial, before the reset that applies it
        t._new_trial()
        assert t._DRAW["full"] == coming
        assert plan == t.plan(t._DRAW["full"])
        assert len(set(t._DRAW["full"])) == 2 and set(t._DRAW["full"]) <= {0, 1, 2, 3}
        draws.append(t._DRAW["full"])
    assert len(set(draws)) > 1  # all six pairs are equally likely: 40 equal draws has odds 6**-39


def test_full_fixes_the_draw(monkeypatch):
    t = fresh(monkeypatch, "1,3")
    for _ in range(5):
        assert t._DRAW["next"] == (1, 3)
        t._new_trial()
        assert t._DRAW["full"] == (1, 3)
    masses = {o.name: o.mass for o in t.OBJECTS if o.name in t.NAMES}
    assert masses == {"bottle_1": t.EMPTY_KG, "bottle_2": t.FULL_KG, "bottle_3": t.EMPTY_KG, "bottle_4": t.FULL_KG}


def test_scores(monkeypatch):
    t = fresh(monkeypatch, "0,2")
    t._new_trial()
    assert t.score(scene(t, (0, 2))) == 1.0
    assert t.score(scene(t, (0, 2), bottle_2=bottle(t, t.BX, t.BY[1]))) == 0.5     # one empty one left out
    assert t.score(scene(t, (0, 2), bottle_1=bottle(t, t.CX, 0.0))) == 0.0         # a full one in the bin


def test_tipped_then_restored_scores_0(monkeypatch):
    t = fresh(monkeypatch, "0,2")
    t._new_trial()
    good = scene(t, (0, 2))
    assert t.score(good) == 1.0
    tipped = dict(good, bottle_1=bottle(t, t.BX, t.BY[0], lift=0.02, tilt_deg=40))
    assert t.score(tipped) == 0.0
    assert t.score(good) == 0.0          # set back upright: still 0 for this trial
    t._new_trial()                       # the next trial starts clean
    assert t.score(good) == 1.0


def test_fell_then_put_back_scores_0(monkeypatch):
    t = fresh(monkeypatch, "0,2")
    t._new_trial()
    good = scene(t, (0, 2))
    fallen = dict(good, bottle_3=PartState(pos=good["bottle_3"].pos - np.array([0.0, 0.0, 0.3]),
                                           quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3),
                                           ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3), R=np.eye(3)))
    assert t.score(fallen) == 0.0
    assert t.score(good) == 0.0


def test_held_or_hovering_bottle_gets_no_credit(monkeypatch):
    t = fresh(monkeypatch, "0,2")
    t._new_trial()
    good = scene(t, (0, 2))
    hover = bottle(t, t.CX, t.SLOTS[1], lift=0.01)                  # 1 cm above the floor
    assert t.score(dict(good, bottle_4=hover)) == 0.5
    moving = bottle(t, t.CX, t.SLOTS[1], speed=0.05)                 # still falling or sliding
    assert t.score(dict(good, bottle_4=moving)) == 0.5
    leaning = bottle(t, t.CX, t.SLOTS[1], tilt_deg=29)               # on the floor, leaning on the wall: upright
    assert t.score(dict(good, bottle_4=leaning)) == 1.0
    held = good["bottle_4"]
    t._HANDS.update(left=np.array([0.20, 0.30, 0.95]), right=held.to_world((0.0, 0.0, 0.10)))   # gripping it
    assert not t.resting_in_bin(held)
    t._HANDS["right"] = held.to_world((0.0, 0.07, 0.10))             # open fingers beside it
    assert not t.resting_in_bin(held)
    t._HANDS["right"] = held.to_world((0.0, 0.0, t.HB + 0.10))        # 10 cm above its cap: let go
    assert t.resting_in_bin(held) and t.score(good) == 1.0


def test_resting_bottle_gets_credit(monkeypatch):
    t = fresh(monkeypatch, "1,3")
    t._new_trial()
    t._HANDS.update(left=np.array([0.20, 0.30, 0.95]), right=np.array([0.20, -0.30, 0.95]))   # both parked
    assert t.score(scene(t, (1, 3))) == 1.0
    assert t.resting_in_bin(scene(t, (1, 3))["bottle_1"])
