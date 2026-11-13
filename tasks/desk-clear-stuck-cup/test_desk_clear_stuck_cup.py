"""Checks that need no GPU: the task file loads, the layout is consistent, and the demo is reachable and the same
whichever cup is stuck."""

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
    assert t.success and t.score and t.demo
    assert set(task.NAMES) <= set(t.free_objects)


def test_text_is_ascii():
    for s in (task.__doc__, task.INSTRUCTION):
        s.encode("ascii")


def test_cups_identical():
    specs = [o for o in task.OBJECTS if o.name in task.NAMES]
    assert len({(o.boxes, o.mass, o.color, o.glue) for o in specs}) == 1


def test_cups_fit_the_tray_slots():
    slots = sorted(task.SLOTS)
    gaps = [b - a for a, b in zip(slots, slots[1:])]
    assert min(gaps) > 2 * task.R_OUT + 0.02                       # 2 cm between cups in the tray
    assert max(abs(s) for s in slots) + task.R_OUT < task.T_IN_Y / 2  # inside the walls
    assert task.CUP_X - task.R_OUT > task.TX + task.T_IN_X / 2 + task.T_WALL  # cups start clear of the tray


def test_demo_same_for_every_stuck_cup():
    plans = []
    for g in ("0", "1", "2"):
        os.environ["GLUED"] = g
        task._reset_state()
        plans.append([(w.duration, w.pos, w.finger) for arm in ("left", "right") for w in task.demo()[arm]])
    os.environ.pop("GLUED")
    assert plans[0] == plans[1] == plans[2]


def test_demo_reachable():
    for arm, ws in task.demo().items():
        p = ArmPlan(list(ws), station.BASES[arm])
        for k in range(int(p.duration * 20) + 5):
            p.action7((k + 1) / 20)
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, arm
