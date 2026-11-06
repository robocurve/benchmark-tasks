"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses
are reachable. Run in the yam_arena environment:  python -m pytest -q test_battery_into_bin.py"""

from __future__ import annotations

import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "battery_into_bin.py")
T = TASK.module


def _state(pos, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def _start(**moved):
    """Every object where it starts (centres), with ``moved`` objects at new centres."""
    out = {
        "picking_bin": _state((*T.BIN_XY, T.TOP + 0.05)),
        "battery": _state((*T.BATTERY_XY, T.TOP + 0.028)),
        **{b.name: _state(b.pos) for b in T.BLOCKS},
    }
    out.update({k: _state(v) if not isinstance(v, ObjectState) else v for k, v in moved.items()})
    return out


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"battery", "red_block", "blue_block", "green_block", "yellow_block"}


def test_start_scores_zero():
    assert T.score(_start()) == 0.0
    assert not T.success(_start())


def test_battery_upright_in_bin_succeeds():
    objects = _start(battery=(T.BIN_XY[0], T.BIN_XY[1], T.TOP + T.BIN_FLOOR + 0.027))
    assert T.success(objects) and T.score(objects) == 1.0


def test_battery_on_its_side_in_bin_succeeds():
    objects = _start(battery=(T.BIN_XY[0] + 0.05, T.BIN_XY[1] - 0.06, T.TOP + T.BIN_FLOOR + 0.009))
    assert T.success(objects)


def test_battery_held_inside_bin_half():
    objects = _start(battery=(T.BIN_XY[0], T.BIN_XY[1], T.TOP + 0.08))
    assert not T.success(objects) and T.score(objects) == 0.5


def test_battery_moving_on_floor_half():
    moving = _state((T.BIN_XY[0], T.BIN_XY[1], T.TOP + 0.03), vel=(0.05, 0.0, 0.0))
    assert T.score(_start(battery=moving)) == 0.5


def test_battery_lifted_elsewhere_quarter():
    assert T.score(_start(battery=(0.25, 0.0, T.TOP + 0.10))) == 0.25


def test_battery_on_rim_or_outside_is_not_in():
    rim = (T.BIN_XY[0], T.BIN_XY[1] + T.BIN_HALF[1] - 0.003, T.TOP + 0.03)
    beside = (T.BIN_XY[0], T.BIN_XY[1] + T.BIN_HALF[1] + 0.02, T.TOP + 0.028)
    for pos in (rim, beside):
        assert T.score(_start(battery=pos)) == 0.0


@pytest.mark.parametrize("block", ["red_block", "blue_block", "green_block", "yellow_block"])
def test_block_in_bin_scores_zero(block):
    objects = _start(battery=(T.BIN_XY[0], T.BIN_XY[1], T.TOP + 0.028),
                     **{block: (T.BIN_XY[0] + 0.06, T.BIN_XY[1], T.TOP + 0.022)})
    assert not T.success(objects) and T.score(objects) == 0.0


def test_object_off_table_scores_zero():
    objects = _start(battery=(T.BIN_XY[0], T.BIN_XY[1], T.TOP + 0.028), green_block=(0.4, 0.7, 0.3))
    assert T.score(objects) == 0.0
    assert T.score(_start(battery=(0.2, 0.0, 0.3))) == 0.0


def test_non_finite_scores_zero():
    assert T.score(_start(battery=(np.nan, 0.0, T.TOP))) == 0.0


def test_demo_is_reachable():
    plan = ArmPlan(T.demo()["right"], BASES["right"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 2.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 1.0, plan.worst_ik_deg
