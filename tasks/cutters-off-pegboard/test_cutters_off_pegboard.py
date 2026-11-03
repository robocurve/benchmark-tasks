"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses
are reachable. Run in the yam_arena environment:  python -m pytest -q test_cutters_off_pegboard.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "cutters_off_pegboard.py")
T = TASK.module

# Centres of the hanging tools, as measured in the simulator after they settle (see README.md).
HANGING = {
    "phillips_ph1": (0.455, -0.40, 0.904),
    "phillips_ph2": (0.455, -0.34, 0.901),
    "cutters": (0.465, -0.26, 0.885),
    "torx_t10": (0.454, -0.261, 1.103),
    "combination_pliers": (0.465, -0.18, 0.883),
    "long_nose_pliers": (0.466, -0.10, 0.906),
    "curved_long_nose_pliers": (0.466, -0.02, 0.906),
    "combination_wrench": (0.469, 0.047, 0.930),
    "groove_joint_pliers": (0.471, 0.117, 0.906),
    "claw_hammer": (0.465, 0.225, 0.942),
    "slotted_5_5mm": (0.455, 0.31, 0.900),
    "torx_t20": (0.455, 0.37, 0.901),
}
ON_TABLE = T.TOP + 0.013  # a tool lying on the table


def _state(pos, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def _scene(**moved):
    """Every tool where it hangs at the start, with ``moved`` tools at new centres (or states)."""
    out = {name: _state(pos) for name, pos in HANGING.items()}
    out.update({k: v if isinstance(v, ObjectState) else _state(v) for k, v in moved.items()})
    return out


ON_MAT = (*T.MAT_XY, T.TOP + T.MAT_T + 0.012)
FALLEN = (T.BOARD_FACE_X - 0.03, -0.18, ON_TABLE)


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set(HANGING)
    assert len(T.TOOLS) >= 10


def test_board_in_front_of_both_arms():
    assert T.BOARD_Y[0] < BASES["right"][1] and T.BOARD_Y[1] > BASES["left"][1]


def test_hanging_heights_match_the_scene():
    assert set(T.HANGING_Z) == set(HANGING)
    assert all(T.HANGING_Z[name] == pytest.approx(z, abs=0.002) for name, (_, _, z) in HANGING.items())


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0
    assert not T.success(_scene())


def test_cutters_on_mat_succeeds():
    objects = _scene(cutters=ON_MAT)
    assert T.success(objects) and T.score(objects) == 1.0


def test_cutters_on_mat_one_tool_fell_half():
    objects = _scene(cutters=ON_MAT, combination_pliers=FALLEN)
    assert not T.success(objects) and T.score(objects) == 0.5


def test_cutters_on_mat_two_tools_fell_zero():
    objects = _scene(cutters=ON_MAT, combination_pliers=FALLEN, phillips_ph2=(T.BOARD_FACE_X - 0.03, -0.34, ON_TABLE))
    assert T.score(objects) == 0.0


def test_cutters_held_off_board_quarter():
    objects = _scene(cutters=(T.OFF_BOARD_X - 0.01, -0.26, T.TOP + 0.20))
    assert T.score(objects) == 0.25


def test_cutters_on_table_off_mat_quarter():
    objects = _scene(cutters=(T.MAT_XY[0], T.MAT_XY[1] + T.MAT_HALF[1] + 0.03, ON_TABLE))
    assert T.score(objects) == 0.25


def test_cutters_moving_on_mat_not_success():
    objects = _scene(cutters=_state(ON_MAT, vel=(0.05, 0.0, 0.0)))
    assert not T.success(objects) and T.score(objects) == 0.25


def test_cutters_off_board_but_a_tool_fell_zero():
    objects = _scene(cutters=(T.OFF_BOARD_X - 0.01, -0.26, T.TOP + 0.20), long_nose_pliers=FALLEN)
    assert T.score(objects) == 0.0


def test_cutters_lifted_on_peg_zero():
    objects = _scene(cutters=(HANGING["cutters"][0], -0.26, 0.885 + 0.03))
    assert T.score(objects) == 0.0


def test_cutters_dropped_under_board_zero():
    objects = _scene(cutters=(T.BOARD_FACE_X - 0.02, -0.26, ON_TABLE))
    assert T.score(objects) == 0.0


def test_tool_off_table_zero():
    objects = _scene(cutters=ON_MAT, torx_t20=(0.3, 0.6, T.TOP - 0.3))
    assert T.score(objects) == 0.0


def test_non_finite_zero():
    objects = _scene(cutters=(math.nan, 0.0, 0.0))
    assert T.score(objects) == 0.0


def test_demo_lifts_before_pulling():
    """After closing on the jaws, the grip rises at least 2 cm before it moves toward the robot."""
    steps = T.demo()["right"]
    close = next(i for i, w in enumerate(steps) if w.label == "close")
    grip = np.asarray(steps[close].pos)
    lift = np.asarray(steps[close + 1].pos)
    pull = np.asarray(steps[close + 2].pos)
    assert lift[2] - grip[2] >= 0.02 and abs(lift[0] - grip[0]) < 1e-9
    assert pull[0] <= grip[0] - (T.BOARD_FACE_X - T.PEG_TIP_X) and pull[2] == pytest.approx(lift[2])


@pytest.mark.parametrize("arm", ["right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (plan.worst_ik_mm, plan.worst_ik_deg)
