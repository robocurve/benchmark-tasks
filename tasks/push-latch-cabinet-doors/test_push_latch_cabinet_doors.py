"""Checks that need no GPU: the task file loads, the latch thresholds are consistent, and the demo is reachable,
presses only the open doors, and lets the arms take turns."""

import itertools
import os
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402


def test_loads():
    t = load(HERE / "task.py")
    assert t.score and t.success and t.demo


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_latch_thresholds():
    # a door at rest must not count as pressed, and must re-arm after a press
    assert 0 < task.RELEASED < task.PRESS < task.math.radians(3.0)
    assert task.AJAR_ANGLE < 0 and task.POP_ANGLE < task.AJAR_ANGLE


def test_demo_presses_only_open_doors_and_reaches():
    for ajar in [c for n in (1, 2) for c in itertools.combinations(range(3), n)]:
        plan = task.plan(ajar)
        pressed = {w[1][1] for ws in plan.values() for w in ws if w[4] == "press it shut"}
        assert pressed == {task.UY[i] for i in ajar}, ajar
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm


def test_arms_take_turns():
    plan = task.plan((0, 2))
    t_left = sum(w[0] for w in plan["left"])
    assert plan["right"][0][4] == "wait for the left arm" and plan["right"][0][0] >= t_left
