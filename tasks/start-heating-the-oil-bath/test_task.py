"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import os
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(str(HERE / "task.py"))
M = TASK.module

FAR = [np.array([0.12, 0.30, 1.0]), np.array([0.15, -0.30, 1.0])]
K_UP = M.HP_TOP + M.KEY_H / 2
K_DOWN = K_UP - 0.004
T_UP = M.TIP0
T_IN = M.TIP0 - M.P_TRAVEL
HOLD = int(M.SUCCESS_HOLD_S * M.HZ) + 1


@pytest.fixture(autouse=True)
def fresh():
    M._new_episode()
    yield


def step(key_z, tip_z, n=1, tcps=FAR):
    for _ in range(n):
        M.track(key_z, tip_z, tcps)
    return M.success({}), M.score({})


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_probe_is_out_of_the_oil_and_quiet():
    assert step(K_UP, T_UP, 30) == (False, 0.0)
    assert not M.EP["immersed"] and not M.EP["violated"] and M.TIP0 > M.OIL_TOP


def test_probe_then_heat_succeeds():
    step(K_UP, T_IN, 5)
    assert M.EP["immersed"]
    ok, sc = step(K_DOWN, T_IN, HOLD)
    assert ok and sc == 1.0 and not M.EP["violated"]


def test_heat_with_probe_out_is_a_violation():
    step(K_DOWN, T_UP)
    assert M.EP["violated"] and "out of the oil" in M.EP["why"]


def test_heat_first_then_probe_is_still_a_violation():
    step(K_DOWN, T_UP, 3)
    assert step(K_DOWN, T_IN, HOLD)[1] == 0.0


def test_probe_only_part_way_is_not_immersed():
    tip = M.OIL_TOP - 0.015
    step(K_UP, tip, 3)
    assert not M.EP["immersed"]
    step(K_DOWN, tip)
    assert M.EP["violated"]


def test_slight_press_does_not_start_heating():
    step(K_UP - 0.002, T_IN, 10)
    assert not M.EP["on"] and step(K_UP - 0.002, T_IN, HOLD)[1] == 0.0


def test_full_travel_reaches_the_manual_depth_with_clamp_slack():
    assert M.OIL_TOP - (T_IN + M.CLAMP_SLIP) >= M.IMMERSED_M
    assert T_IN > M.HP_TOP + M.D_FLOOR  # the tip stays off the dish's floor


def test_still_on_the_key_does_not_succeed():
    step(K_UP, T_IN, 2)
    key = np.array([*M.KEY_XY, M.HP_TOP + M.KEY_H])
    assert step(K_DOWN, T_IN, HOLD, tcps=[key, FAR[1]])[1] == 0.0


def test_probe_head_is_within_reach_height():
    assert M.TIP0 + M.PROBE_TOP + 0.013 - M.TOP < 0.20


def test_plans_order():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            labels = [w.label for w in M.demo()["left"]]
        finally:
            os.environ.pop("TASK_DEMO", None)
        p = labels.index("press it in")
        lo = labels.index("push it down through the clamp into the oil") if v != "naive" else None
        assert (lo is not None and lo < p) == (v == "demo")
