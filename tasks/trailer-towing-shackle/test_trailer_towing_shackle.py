"""Checks that need no GPU: the task file loads, the lock geometry is consistent, and the demo is reachable."""

import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import task  # noqa: E402
from yam_arena import station  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import load  # noqa: E402


def test_loads():
    t = load(HERE / "task.py")
    assert t.success and t.score and t.demo
    assert {"shackle", "pin"} <= set(t.free_objects)


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_lock_inside_one_twist():
    # latched after 120 degrees, seated after half a turn, both within one twist of the wrist
    assert task.LOCK < math.pi
    assert abs(task.thread_height(0.0) - task.TRAVEL) < 1e-9
    assert task.thread_height(math.pi) == 0.0
    assert task.thread_height(task.LOCK) > task.SEATED  # the latch, not the seat, gives full credit at 120 degrees


def test_demo_has_one_twist():
    labels = [w.label for w in task.demo()["right"]]
    assert labels.count("pinch") == 1


def test_demo_reachable():
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            q, _ = p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm


def test_trailer_clear_of_the_work():
    # the deck and rails start behind the block, beyond the shackle stand and the pin holder
    assert task.DX0 > task.FACE and task.DX0 > task.XS + 0.05 and task.DX0 > task.HX + 0.05
    assert np.isfinite(task.AX)
