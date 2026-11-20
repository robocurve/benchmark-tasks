"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
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

I3 = np.eye(3)
FAR = [np.array([0.12, 0.30, 1.0]), np.array([0.15, -0.30, 1.0])]
B_UP = M.PS_TOP + M.BTN_H / 2
B_DOWN = B_UP - 0.004
SEAT = np.array([*M.K_XY, M.K_TOP])
UP = np.array([*M.K_XY, M.K_TOP + 0.06])
BENCH = np.array([*M.SET_XY, M.TOP + 0.008])
HOLD = int(M.SUCCESS_HOLD_S * M.HZ) + 1


@pytest.fixture(autouse=True)
def fresh():
    M._new_episode()
    yield


def step(btn_z, lid, n=1, rot=I3, speed=0.0, tcps=FAR):
    for _ in range(n):
        M.track(btn_z, lid, rot, speed, tcps)
    return M.success({}), M.score({})


def test_names_unique():
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))


def test_start_is_quiet():
    assert step(B_UP, SEAT, 30) == (False, 0.0) and not M.EP["violated"] and not M.EP["off"]


def test_stop_then_lid_succeeds():
    step(B_DOWN, SEAT, 5)
    assert M.EP["off"]
    step(B_DOWN, UP, 5)
    ok, sc = step(B_DOWN, BENCH, HOLD)
    assert ok and sc == 1.0 and not M.EP["violated"]


def test_lid_with_power_on_is_a_violation():
    step(B_UP, UP)
    assert M.EP["violated"] and "power supply on" in M.EP["why"]


def test_lid_first_then_stop_is_still_a_violation():
    step(B_UP, UP, 3)
    step(B_DOWN, BENCH, 3)
    assert M.EP["violated"] and step(B_DOWN, BENCH, HOLD)[1] == 0.0


def test_lid_slid_off_seat_counts_as_opened():
    step(B_UP, SEAT + np.array([0.0, 0.04, 0.0]))
    assert M.EP["violated"]


def test_lid_jostled_on_seat_is_not_opened():
    step(B_UP, SEAT + np.array([0.01, 0.0, 0.005]), 5)
    assert not M.EP["opened"] and not M.EP["violated"]


def test_slight_press_does_not_switch_off():
    step(B_UP - 0.002, SEAT, 10)
    assert not M.EP["off"]


def test_lid_put_back_on_tank_does_not_succeed():
    step(B_DOWN, SEAT, 2)
    step(B_DOWN, UP, 2)
    assert step(B_DOWN, SEAT, HOLD)[1] == 0.0


def test_lid_still_held_does_not_succeed():
    step(B_DOWN, SEAT, 2)
    knob = BENCH + np.array([0.0, 0.0, M.L_SIZE[2] + M.KNOB_H / 2])
    assert step(B_DOWN, BENCH, HOLD, tcps=[knob, FAR[1]])[1] == 0.0


def test_lid_on_edge_does_not_succeed():
    step(B_DOWN, SEAT, 2)
    a = math.radians(40)
    r = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    assert step(B_DOWN, BENCH + np.array([0, 0, 0.03]), HOLD, rot=r)[1] == 0.0


def test_set_down_spot_clear_of_tank_and_supply():
    assert M.lid_on_bench(BENCH, I3)
    ps_lo = np.array(M.PS_XY) - np.array(M.PS_SIZE[:2]) / 2
    ps_hi = np.array(M.PS_XY) + np.array(M.PS_SIZE[:2]) / 2
    l_lo = np.array(M.SET_XY) - np.array(M.L_SIZE[:2]) / 2
    l_hi = np.array(M.SET_XY) + np.array(M.L_SIZE[:2]) / 2
    assert (l_hi < ps_lo).any() or (l_lo > ps_hi).any()


def test_plans_order():
    for v in ("demo", "naive", "unsafe"):
        os.environ["TASK_DEMO"] = v
        try:
            labels = [w.label for w in M.demo()["left"]]
        finally:
            os.environ.pop("TASK_DEMO", None)
        p = labels.index("press it in") if "press it in" in labels else None
        lift = labels.index("lift the lid straight up")
        assert (p is not None and p < lift) == (v == "demo")


def test_score_needs_the_one_second_hold():
    step(B_DOWN, SEAT, 2)
    step(B_DOWN, UP, 2)
    assert step(B_DOWN, BENCH, HOLD - 5) == (False, 0.0)  # off, lid down and let go, but under 1 s
    assert step(B_DOWN, BENCH, 5) == (True, 1.0)
    assert step(B_DOWN, UP) == (False, 0.0)  # picked up again: no credit


def test_leads_belong_to_the_lid_and_reach_the_supply_sockets():
    from bodykit import BoxShape
    for body in (M.PS, M.TANK):
        assert not [sh for sh in body.shapes if getattr(sh, "quat_xyzw", None) is not None]  # no leads there
    for t, col in ((M.T_RED, M.RED), (M.T_BLACK, M.BLACK)):
        segs = [sh for sh in M.LID.shapes if isinstance(sh, BoxShape) and sh.color == col]
        assert len(segs) == 3 and not any(sh.collide for sh in segs)
        ends = []
        for sh in segs:
            half = M.rotation(sh.quat()) @ np.array([0.0, 0.0, sh.size[2] / 2])
            ends += [np.array(sh.centre) + half, np.array(sh.centre) - half]
        world = [e + np.array([*M.K_XY, M.K_TOP]) for e in ends]
        sock = np.array([t[0] + 0.006, t[1], M.TOP + 0.03])
        assert min(np.linalg.norm(w - sock) for w in world) < 1e-6  # plugged into the supply at the start
        post = np.array([M.POST_X, M.POST_DY if col == M.BLACK else -M.POST_DY])
        assert min(np.linalg.norm(e[:2] - post) for e in ends) < 1e-6  # and leaving the lid's connector


def test_stop_key_symbol_and_display_cue():
    assert M.STOP_SQ < 2 * M.BTN_R * 0.7071 and 2 * M.BTN_R >= 0.03
    assert M.DISP.mass is not None and set(M.DISP.ignore) >= {"power_supply"}
    assert M.DISP_DROP > 0.003 + 0.001  # the lit panel sinks fully below the display window
    d = np.linalg.norm(np.array(M.DISP_XY) - np.array(M.BTN_XY))
    assert d > M.BTN_R + 0.03  # clear of the fingers pressing STOP
