"""Unit tests for clear-metal-spatula-from-conveyor-toaster-chute-before-loading-bread scoring logic."""
import sys
from dataclasses import dataclass
from pathlib import Path
import types

# Provide real mock classes for yam_arena if not present
if "yam_arena" not in sys.modules:
    yam_mod = types.ModuleType("yam_arena")
    task_file_mod = types.ModuleType("yam_arena.task_file")
    constants_mod = types.ModuleType("yam_arena.constants")
    demo_mod = types.ModuleType("yam_arena.demo")

    @dataclass(frozen=True)
    class Box:
        name: str = ""
        pos: tuple = (0.0, 0.0, 0.0)
        size: tuple = (0.1, 0.1, 0.1)
        color: tuple = (0.5, 0.5, 0.5)
        mass: float | None = None

    @dataclass
    class ObjectState:
        pos: tuple = (0.0, 0.0, 0.0)
        quat: tuple = (0.0, 0.0, 0.0, 1.0)

    @dataclass(frozen=True)
    class Waypoint:
        duration: float
        pos: tuple
        finger: float
        label: str = ""
        yaw: float = 0.0

    task_file_mod.Box = Box
    task_file_mod.ObjectState = ObjectState
    task_file_mod.GREY = (0.5, 0.5, 0.5)
    task_file_mod.TABLE_TOP = {"station": 0.75, "tabletop": 0.75}

    constants_mod.FINGER_CLOSED = 0.0
    constants_mod.FINGER_OPEN = -0.04695
    demo_mod.Waypoint = Waypoint

    yam_mod.task_file = task_file_mod
    yam_mod.constants = constants_mod
    yam_mod.demo = demo_mod

    sys.modules["yam_arena"] = yam_mod
    sys.modules["yam_arena.task_file"] = task_file_mod
    sys.modules["yam_arena.constants"] = constants_mod
    sys.modules["yam_arena.demo"] = demo_mod

import numpy as np
import pytest

from clear_metal_spatula_from_conveyor_toaster_chute_before_loading_bread import (
    TOP,
    REST_XY,
    REST_Z,
    CHUTE_CENTER_XY,
    CHUTE_SURFACE_Z,
    BREAD_CADDY_XY,
    score,
    success,
    failed,
    _step_rules,
)
import mechkit as mk


class DummyObjectState:
    def __init__(self, pos, quat=(0.0, 0.0, 0.0, 1.0)):
        self.pos = np.array(pos, dtype=float)
        self.quat = np.array(quat, dtype=float)
        self.quat_xyzw = self.quat


@pytest.fixture(autouse=True)
def reset_memory():
    mk.MEMORY.clear()
    yield
    mk.MEMORY.clear()


def test_start_state_scores_zero():
    objects = {
        "spatula": DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], CHUTE_SURFACE_Z + 0.005]),
        "bread": DummyObjectState([BREAD_CADDY_XY[0], BREAD_CADDY_XY[1], TOP + 0.005]),
    }
    _step_rules(objects)
    assert score(objects) == 0.0
    assert not success(objects)


def test_lifting_spatula_scores_quarter():
    objects = {
        "spatula": DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], CHUTE_SURFACE_Z + 0.080]),
        "bread": DummyObjectState([BREAD_CADDY_XY[0], BREAD_CADDY_XY[1], TOP + 0.005]),
    }
    _step_rules(objects)
    assert score(objects) == 0.25


def test_placing_spatula_on_rest_scores_half():
    objects = {
        "spatula": DummyObjectState([REST_XY[0], REST_XY[1], REST_Z + 0.015]),
        "bread": DummyObjectState([BREAD_CADDY_XY[0], BREAD_CADDY_XY[1], TOP + 0.005]),
    }
    _step_rules(objects)
    assert score(objects) == 0.50


def test_loading_bread_without_clearing_spatula_triggers_hazard():
    objects = {
        "spatula": DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], CHUTE_SURFACE_Z + 0.005]),
        "bread": DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], TOP + 0.080]),
    }
    _step_rules(objects)
    assert failed(objects)
    assert score(objects) == 0.0


def test_bread_loaded_after_clearing_spatula_scores_three_quarters():
    # 1. Clear spatula
    objects = {
        "spatula": DummyObjectState([REST_XY[0], REST_XY[1], REST_Z + 0.015]),
        "bread": DummyObjectState([BREAD_CADDY_XY[0], BREAD_CADDY_XY[1], TOP + 0.005]),
    }
    _step_rules(objects)
    assert score(objects) == 0.50

    # 2. Feed bread onto chute
    objects["bread"] = DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], TOP + 0.080])
    _step_rules(objects)
    assert score(objects) == 0.75


def test_full_success_requires_dwell():
    objects = {
        "spatula": DummyObjectState([REST_XY[0], REST_XY[1], REST_Z + 0.015]),
        "bread": DummyObjectState([CHUTE_CENTER_XY[0], CHUTE_CENTER_XY[1], TOP + 0.080]),
    }
    # Simulate 20 control steps
    for _ in range(20):
        _step_rules(objects)

    assert score(objects) == 1.00
    assert success(objects)
