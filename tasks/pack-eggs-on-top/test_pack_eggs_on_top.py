"""Offline checks (no GPU, no Isaac Sim): the scorer on hand-built end states, and the demo's reachability.

    pytest tasks/pack-eggs-on-top    (in an environment with yam_arena installed)
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("pack_eggs_on_top.py"))
T = TASK.module
CAN = (2 * T.CAN_R, 2 * T.CAN_R, T.CAN_H)


def state(pos, size, tilt_deg=0.0, vel=(0.0, 0.0, 0.0)):
    """An object at ``pos`` (its centre), tilted about x; the bounding box grows with the tilt."""
    pos, (sx, sy, sz) = np.asarray(pos, dtype=float), size
    a = math.radians(tilt_deg)
    half = np.array([sx, abs(sy * math.cos(a)) + abs(sz * math.sin(a)), abs(sy * math.sin(a)) + abs(sz * math.cos(a))])
    quat = np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])
    return ObjectState(pos, quat, np.asarray(vel, float), np.zeros(3), pos - half / 2, pos + half / 2)


def can_in_tote(x):
    return state((x, T.TOTE_Y, T.FLOOR + T.CAN_H / 2), CAN)


def can_at_start(name):
    return state((*T.CAN_START[name], T.TOP + T.CAN_H / 2), CAN)


def eggs_at(x, y, bottom, tilt_deg=0.0):
    """The carton with its lowest point at ``bottom``."""
    _, sy, sz = T.EGGS_SIZE
    a = math.radians(tilt_deg)
    return state((x, y, bottom + (abs(sy * math.sin(a)) + abs(sz * math.cos(a))) / 2), T.EGGS_SIZE, tilt_deg)


PACKED = {
    "can_1": can_in_tote(T.CAN_PLACE_X[0]),
    "can_2": can_in_tote(T.CAN_PLACE_X[1]),
    "eggs": eggs_at(T.TOTE_X, T.TOTE_Y, T.FLOOR + T.CAN_H),
}
START = {"can_1": can_at_start("can_1"), "can_2": can_at_start("can_2"), "eggs": eggs_at(*T.EGGS_START, T.TOP)}


def test_the_task_loads_on_the_station_with_three_loose_objects():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"can_1", "can_2", "eggs"}
    assert TASK.docs and "egg" in TASK.docs


def test_packed_scores_full_and_succeeds():
    assert T.score(PACKED) == 1.0
    assert T.success(PACKED)


def test_the_start_scores_nothing():
    assert T.score(START) == 0.0
    assert not T.success(START)


@pytest.mark.parametrize(
    ("objects", "expected"),
    [
        ({**START, "can_1": can_in_tote(T.CAN_PLACE_X[0])}, 0.2),
        ({**START, "can_1": can_in_tote(T.CAN_PLACE_X[0]), "can_2": can_in_tote(T.CAN_PLACE_X[1])}, 0.4),
        # the carton flat on the tote's floor, the cans still out: packed, but not on top
        ({**START, "eggs": eggs_at(T.TOTE_X, 0.0, T.FLOOR)}, 0.1),
        # the carton tipped on its side on top of the cans: no credit for it
        ({**PACKED, "eggs": eggs_at(T.TOTE_X, 0.0, T.FLOOR + T.CAN_H, tilt_deg=90)}, 0.4),
        # the cans lying on their sides in the tote, the carton flat on them: still on top
        ({"can_1": state((0.20, 0.0, T.FLOOR + T.CAN_R), CAN, tilt_deg=90),
          "can_2": state((0.28, 0.0, T.FLOOR + T.CAN_R), CAN, tilt_deg=90),
          "eggs": eggs_at(T.TOTE_X, 0.0, T.FLOOR + 2 * T.CAN_R)}, 1.0),
        # the carton on top but tilted 20 degrees (resting on one can and a wall): only "in the tote"
        ({**PACKED, "eggs": eggs_at(T.TOTE_X, 0.0, T.FLOOR + T.CAN_H, tilt_deg=20)}, 0.5),
        # the carton on the cans, but the cans outside the tote
        ({"can_1": state((0.20, 0.20, T.TOP + T.CAN_H / 2), CAN), "can_2": state((0.275, 0.20, T.TOP + T.CAN_H / 2), CAN),
          "eggs": eggs_at(0.2375, 0.20, T.TOP + T.CAN_H)}, 0.0),
    ],
)
def test_partial_credit(objects, expected):
    assert T.score(objects) == pytest.approx(expected)
    assert T.success(objects) == (expected == 1.0)


def test_a_can_on_the_eggs_scores_zero():
    eggs = eggs_at(T.TOTE_X, 0.0, T.FLOOR)
    on_eggs = state((T.TOTE_X, 0.0, T.FLOOR + T.EGGS_SIZE[2] + T.CAN_H / 2), CAN)
    assert T.score({"can_1": can_in_tote(T.CAN_PLACE_X[0]), "can_2": on_eggs, "eggs": eggs}) == 0.0


def test_success_waits_for_the_carton_to_settle():
    moving = {**PACKED, "eggs": state(PACKED["eggs"].pos, T.EGGS_SIZE, vel=(0.05, 0.0, 0.0))}
    assert not T.success(moving)


def test_the_demo_is_reachable_within_the_joint_limits():
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    for side, waypoints in plan.items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = round(arm.duration * 20) + 5
        actions = np.array([arm.action7((k + 1) / 20)[0] for k in range(steps)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER), side


def test_the_demo_fits_in_the_run_limit():
    longest = max(ArmPlan(w, station.BASES[s]).duration for s, w in TASK.demo().items())
    assert longest * 20 < 1800 * 0.5  # under half of the 1,800-step limit, with room for the settle and hold


def test_no_arm_moves_sideways_with_its_fingertips_below_the_stacked_carton_top():
    """After letting go, an arm that moves sideways must first lift its fingertips clear of the stack, or a finger
    catches the carton and drags it off the cans."""
    # What each arm has just let go of: the left a can on the tote floor, the right the carton on the cans.
    below = {"left": T.FLOOR + T.CAN_H, "right": T.FLOOR + T.CAN_H + T.EGGS_SIZE[2]}
    for side, waypoints in TASK.demo().items():
        released = False
        prev = None
        for w in waypoints:
            if w.label == "release":
                released = True
            elif released and prev is not None and np.hypot(w.pos[0] - prev[0], w.pos[1] - prev[1]) > 1e-6:
                assert prev[2] - PAD_TIP_PAST_TCP > below[side] + 0.005, f"{side}: {w.label} leaves too low"
            prev = w.pos


def test_the_right_arm_waits_until_the_left_has_parked():
    plan = TASK.demo()
    left_done = ArmPlan(plan["left"], station.BASES["left"]).duration
    assert plan["right"][0].duration >= left_done
