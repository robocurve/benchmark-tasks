"""Checks that need no GPU: the task file loads, the bottles look identical, the bin holds them, and the demo tests
every bottle and is reachable for every draw."""

import itertools
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
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
