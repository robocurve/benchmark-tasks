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


def test_real_thread():
    # 5/8-11 UNC: 2.31 mm a turn; five full turns from the first catch to the collar seated
    assert abs(task.PITCH - 0.0254 / 11) < 1e-9
    assert abs(task.thread_height(0.0) - task.TRAVEL) < 1e-9
    assert task.thread_height(2 * math.pi * task.TURNS) == 0.0
    assert task.thread_height(2 * math.pi * (task.TURNS - 1)) > task.SEATED  # one turn short is not seated
    assert 0 < task.LEAD < task.EAR_T


def test_demo_turns_enough():
    # half turns with a regrip between them: enough to turn the pin five full turns
    labels = [w.label for w in task.demo()["right"]]
    assert labels.count("pinch") >= 2 * task.TURNS


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


def test_round_lug_hole():
    # no lug box reaches into the 19 mm hole around the pin's axis
    for b in task.BLOCK:
        if b.name.startswith("lug"):
            hx, hy = b.size[0] / 2, b.size[1] / 2
            c, s_ = math.cos(b.yaw), math.sin(b.yaw)
            # nearest point of the (yawed) box to the hole axis
            dx, dy = task.XH - b.pos[0], task.YH - b.pos[1]
            lx, ly = c * dx + s_ * dy, -s_ * dx + c * dy
            nx, ny = max(-hx, min(hx, lx)), max(-hy, min(hy, ly))
            assert math.hypot(lx - nx, ly - ny) >= task.HOLE / 2 * 0.98, b.name
