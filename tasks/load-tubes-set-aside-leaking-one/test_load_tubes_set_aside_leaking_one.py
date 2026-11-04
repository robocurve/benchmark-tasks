"""The task file without a simulator: python -m pytest -q test_load_tubes_set_aside_leaking_one.py

Needs ``yam_arena`` importable (it is in the simulator's environment; elsewhere put a clone of yam-isaac-lab-arena
on PYTHONPATH). Checks the scene loads, the demo is reachable, and the outcome table.
"""

import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import pytest  # noqa: E402
from yam_arena import station, task_file  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TASK = task_file.load(HERE / "load_tubes_set_aside_leaking_one.py")
M = TASK.module
UP = (0.0, 0.0, 0.0, 1.0)
HALF = M.TUBE_H / 2


def tube_at(x, y, quat=UP, z=None):
    z = M.TOP + HALF if z is None else z
    pos = np.array([x, y, z], dtype=float)
    r = np.array([M.TUBE_R, M.TUBE_R, HALF])
    return ObjectState(pos=pos, quat_xyzw=np.array(quat, dtype=float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - r, high=pos + r)


def start():
    return {f"tube{i}": tube_at(x, M.RACK_Y) for i, x in enumerate(M.SLOT_X)}


def load(objects, *tubes):
    for i in tubes:
        objects[f"tube{i}"] = tube_at(M.SLOT_X[i], M.TRAY_Y)
    return objects


def hold(objects, *tubes):
    for i in tubes:
        objects[f"tube{i}"] = tube_at(*M.HOLD_XY)
    return objects


def test_the_scene_loads():
    assert set(TASK.free_objects) == {"tube0", "tube1", "tube2", "tube3", "call_button"}
    assert "red button" in TASK.docs


def test_sorting_and_calling_scores_one():
    assert M.grade(hold(load(start(), *M.GOOD), M.LEAKING), True) == pytest.approx(1.0)


def test_doing_nothing_scores_zero():
    assert M.grade(start(), False) == 0.0


def test_pressing_the_button_alone_earns_nothing():
    assert M.grade(start(), True) == 0.0


def test_loading_the_leaking_tube_scores_zero():
    objects = load(start(), *M.GOOD, M.LEAKING)
    assert M.grade(objects, True) == 0.0


def test_holding_it_without_calling_is_partial():
    assert M.grade(hold(load(start(), *M.GOOD), M.LEAKING), False) == pytest.approx(0.6)


def test_leaving_it_in_the_rack_is_partial():
    assert M.grade(load(start(), *M.GOOD), False) == pytest.approx(0.45)
    assert M.grade(load(start(), *M.GOOD), True) == pytest.approx(0.45)


def test_holding_everything_back_is_not_sorting():
    assert M.grade(hold(start(), 0, 1, 2, 3), True) == pytest.approx(0.15)


def test_a_tube_on_its_side_in_a_slot_does_not_count():
    objects = start()
    objects["tube0"] = tube_at(M.SLOT_X[0], M.TRAY_Y, quat=(0.7071068, 0.0, 0.0, 0.7071068))
    assert not M.in_tray_slot(objects["tube0"])


def test_knocking_the_leaking_tube_away_scores_zero():
    objects = load(start(), 0, 1, 3)
    objects["tube2"] = tube_at(0.45, -0.20)
    assert M.grade(objects, False) == 0.0


def test_a_leaking_tube_leaning_in_a_slot_still_counts_as_loaded():
    lean = (0.2588, 0.0, 0.0, 0.9659)  # 30 degrees
    objects = load(start(), 0, 1, 3)
    objects["tube2"] = tube_at(M.SLOT_X[2], M.TRAY_Y, quat=lean)
    assert M.grade(objects, True) == 0.0


def test_the_button_stays_called_once_pressed():
    M.CALLED["yes"] = False
    from benchkit import tool_state

    base = {**start()}
    pressed = tool_state(M.BUTTON, M.BUTTON.pos, UP, sliders={"cap": -M.BUTTON_TRAVEL})
    M._watch({**base, "call_button": pressed})
    released = tool_state(M.BUTTON, M.BUTTON.pos, UP, sliders={"cap": 0.0})
    M._watch({**base, "call_button": released})
    assert M.CALLED["yes"]
    M._reset({})
    assert not M.CALLED["yes"]


def test_every_demo_waypoint_is_reachable():
    plan = M.demo()
    for arm, waypoints in plan.items():
        a = ArmPlan(list(waypoints), station.BASES[arm])
        for t in np.arange(0.0, a.duration + 0.5, 1 / 20):
            a.action7(t)
        assert a.worst_ik_mm < 0.5 and a.worst_ik_deg < 0.1
        assert a.duration * 20 + 60 < 2400


def test_task_json_matches_the_task_file():
    spec = json.loads((HERE / "task.json").read_text())
    assert spec["instruction"] == M.INSTRUCTION
    assert len(spec["fidelity"]) <= 500 and len(spec["scores"]) <= 300
    assert (HERE / "README.md").read_text().count(M.INSTRUCTION) == 2
