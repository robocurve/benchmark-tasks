"""Checks that need no simulator: the task file loads the way yam_arena loads it, the latch has the clearances the
task describes, and the score follows the panel's angle and the forcing rule."""

import json
import math
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parents[1]
TASK = task_file.load(HERE / "task.py")
T = TASK.module  # the module as yam_arena loaded it: these are the functions the embodiment calls


def objects(angle_deg: float = 0.0, travel_mm: float = 0.0) -> dict:
    """What ``success()`` and ``score()`` receive for a panel open ``angle_deg`` with the pin back ``travel_mm``."""
    from mechanism import MechanismState

    zero = np.zeros(3)
    joints = {"panel_hinge": math.radians(angle_deg), "plunger_slide": travel_mm / 1000}
    state = MechanismState(pos=zero, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=zero, ang_vel=zero, low=zero,
                           high=zero, joints=joints)
    return {"cabinet": state}


def test_loads_as_a_fixed_station_object():
    assert TASK.scene == "station"
    assert TASK.names == ("cabinet",)
    assert TASK.free_objects == ()  # fixed: never jittered
    assert TASK.success is not None and TASK.score is not None and TASK.score_over == "end"


def test_pin_fits_the_bore_with_0_6_mm_each_side():
    front, back, above, below = (s.bounds() for s in T.PANEL_SHAPES[-5:-1])
    bore_x = back[0][0] - front[1][0]
    bore_z = above[0][2] - below[1][2]
    assert bore_x == pytest.approx(0.0072, abs=1e-9) and bore_z == pytest.approx(0.0072, abs=1e-9)
    assert (bore_x / 2 - T.PIN_RADIUS) == pytest.approx(0.0006, abs=1e-9)
    assert (front[1][0] + back[0][0]) / 2 == pytest.approx(T.PIN_AXIS[0], abs=1e-9)


def test_pin_passes_4_mm_through_and_clears_at_12_mm():
    lug_low, lug_high = T.PANEL_SHAPES[-5].bounds()
    assert lug_high[1] - lug_low[1] == pytest.approx(0.008)  # the lug is 8 mm thick
    assert lug_low[1] - T.PIN_SPAN[0] == pytest.approx(0.004)  # the tip sticks out 4 mm past it
    assert T.PIN_SPAN[0] + 0.012 == pytest.approx(lug_high[1])  # 12 mm back, the tip is level with its face


def test_spring_pushes_0_3_N_at_rest_and_2_4_N_fully_pulled():
    slide = next(j for j in T.CABINET.joints if j.name == "plunger_slide")
    assert slide.stiffness * (0.0 - slide.target) == pytest.approx(0.3)
    assert slide.stiffness * (slide.upper - slide.target) == pytest.approx(2.4)
    assert slide.upper == pytest.approx(0.014)


def test_forcing_starts_a_third_of_a_degree_past_the_engaged_pin():
    play = max(d / 100 for d in range(300) if T.pin_overlap_mm(math.radians(d / 100), 0.0) == 0.0)
    forced = min(d / 100 for d in range(300) if T.pin_overlap_mm(math.radians(d / 100), 0.0) > T.FORCED_MM)
    assert play == pytest.approx(0.13, abs=0.01)
    assert forced == pytest.approx(0.36, abs=0.01)


def test_no_overlap_once_the_pin_is_back_or_the_strike_has_swung_clear():
    for deg in np.linspace(0, 65, 131):
        assert T.pin_overlap_mm(math.radians(deg), 0.0122) == 0.0
    for deg in np.linspace(7, 65, 59):
        assert T.pin_overlap_mm(math.radians(deg), 0.0) == 0.0


@pytest.mark.parametrize(
    "deg, expected",
    [(0.0, 0.0), (1.9, 0.0), (2.0, 0.25), (6.9, 0.25), (7.0, 0.5), (36.9, 0.5), (37.0, 1.0), (60.0, 1.0)],
)
def test_score_by_the_panel_angle(deg, expected):
    T._new_episode()
    assert T.score(objects(deg, 14.0)) == expected
    assert T.success(objects(deg, 14.0)) == (expected == 1.0)


def test_forcing_the_latch_scores_0_for_the_rest_of_the_run():
    T._new_episode()
    assert T.score(objects(1.0, 0.0)) == 0.0  # the strike 1 degree past the engaged pin
    assert T.episode()["forced"]
    assert T.score(objects(40.0, 14.0)) == 0.0
    assert not T.success(objects(40.0, 14.0))
    T._new_episode()
    assert T.score(objects(40.0, 14.0)) == 1.0


def test_pushing_the_panel_into_the_cabinet_is_forcing():
    T._new_episode()
    assert T.score(objects(-0.3, 0.0)) == 0.0 and T.episode()["forced"]
    T._new_episode()
    assert T.score(objects(-0.1, 0.0)) == 0.0 and not T.episode()["forced"]


def test_handle_follows_the_arc():
    assert np.allclose(T.handle_at(0.0), T.HANDLE)
    hx, hy = T.HINGE_AXIS
    radius = math.hypot(T.HANDLE[0] - hx, T.HANDLE[1] - hy)
    for deg in (10, 40, 65):
        p = T.handle_at(math.radians(deg))
        assert math.hypot(p[0] - hx, p[1] - hy) == pytest.approx(radius)
        assert p[0] < T.HANDLE[0]  # toward the arms


def test_task_json():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert meta.get(key), key
    assert meta["instruction"] == T.INSTRUCTION
    # the submission bot's limits (it rejected other pull requests over each of them)
    assert len(meta["fidelity"]) <= 500 and len(meta["scores"]) <= 300 and len(meta.get("context", "")) <= 500
    assert meta["category"] in ("Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relatability", "Societal relevance", "Other")
    assert meta["run_limit"].startswith("600 steps")
