"""Scoring and demo checks for open-drink-can that run without the simulator:
python -m pytest -q test_open_drink_can.py"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
from mechanism import MechanismState
from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import load

import open_drink_can as M

HERE = Path(__file__).resolve().parent


def can(tab_deg=0.0, panel_deg=0.0, catch_mm=0.0) -> dict:
    """The can with the tab's tail raised and the panel pushed in by these angles (degrees, positive)."""
    joints = {"tab_hinge": -math.radians(tab_deg), "panel_hinge": -math.radians(panel_deg), "catch_slide": catch_mm / 1000}
    z = np.zeros(3)
    return {"can": MechanismState(pos=np.array([*M.CAN_XY, M.TOP]), quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=z,
                                  ang_vel=z, low=z, high=z, joints=joints)}


def test_the_task_file_loads():
    task = load(HERE / "open_drink_can.py")
    assert [o.name for o in task.objects] == ["can"]


# -- scoring --------------------------------------------------------------------------------------------------------

def test_the_closed_can_scores_0():
    assert M.score(can()) == 0.0 and not M.success(can())


def test_opened_with_the_tab_succeeds_and_scores_0_9_until_it_has_held():
    s = can(tab_deg=22, panel_deg=82)
    assert M.success(s) and M.score(s) == 0.9


@pytest.mark.parametrize("tab, panel", [(10, 45), (40, 90)])
def test_the_thresholds(tab, panel):
    assert M.success(can(tab_deg=tab, panel_deg=panel))


def test_a_panel_poked_in_with_the_tab_flat_scores_0():
    assert M.score(can(tab_deg=0, panel_deg=80)) == 0.0


def test_the_seal_broken_but_not_open_scores_half():
    assert M.score(can(tab_deg=5, panel_deg=20)) == 0.5


def test_the_tab_raised_with_the_seal_holding_scores_a_quarter():
    assert M.score(can(tab_deg=15, panel_deg=2)) == 0.25


def test_a_barely_lifted_tab_scores_0():
    assert M.score(can(tab_deg=5, panel_deg=2)) == 0.0


# -- the can's geometry ---------------------------------------------------------------------------------------------

def test_the_can_is_a_real_12_oz_can():
    assert 2 * M.BODY_R == pytest.approx(66.0) and M.RIM_TOP == pytest.approx(122.7)
    assert M.RIM_TOP - M.LID_Z == pytest.approx(6.3)


def test_the_tail_stands_proud_of_the_tab_body_and_inside_the_rim():
    assert M.KNOB_Z - M.KNOB_R > M.LID_Z + M.TAB_LIFT + M.TAB_T
    assert M.KNOB_Z + M.KNOB_R < M.RIM_TOP


def test_the_fingers_round_the_tail_fit_inside_the_rim():
    # fingertips 26 mm wide (across, along y) and 6.3 mm thick, 10 mm apart coming down round the tail
    gap, thick, half_width = 10.0, 6.3, 13.0
    far_x = M.KNOB_X + gap / 2 + thick + 1000 * M.DEMO_AIM_SHIFT_MM / 1000
    assert math.hypot(far_x, half_width) < M.RIM_IN - 1.5


def test_the_near_finger_passes_over_the_tab_body():
    finger_bottom = M.KNOB_Z - M.KNOB_R + 0.3
    assert finger_bottom > M.LID_Z + M.TAB_LIFT + M.TAB_T + 0.3


def test_only_the_nub_reaches_down_to_the_panel():
    assert M.NOSE_LIFT > M.TAB_LIFT and M.NUB_X0 > M.PANEL_X0 and M.NUB_X1 < M.PANEL_X1
    assert M.BODY_X0 >= M.PANEL_X1  # the flat body stays off the panel


def test_the_catch_holds_the_panel_edge():
    assert 0 < M.CATCH_OVERLAP < M.CATCH_R and M.CATCH_TRAVEL > M.CATCH_OVERLAP / 1000


def test_the_opening_is_inside_the_lid():
    assert math.hypot(M.PANEL_X0 - M.OPEN_GAP, M.PANEL_HALF_W + M.OPEN_GAP) < M.RIM_IN


# -- the demo -------------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_pinches_the_tail_and_lifts_it_on_its_arc():
    left = M.demo()["left"]
    pinch = next(w for w in left if w.label == "pinch the tail")
    assert pinch.pos == pytest.approx(M.tail_tcp(0))
    lifts = [w for w in left if w.label.startswith("tail up")]
    assert len(lifts) == len(M.TAB_STEPS)
    # every aim on the tail's circle about the rivet, the tail rising and swinging in towards it
    radius = np.linalg.norm(M.KNOB_FROM_RIVET)
    for w in lifts:
        tail = np.array(w.pos) - np.array([0, 0, M.KNOB_GRIP_Z]) - M.RIVET_W
        assert np.linalg.norm(tail) == pytest.approx(radius, abs=1e-6)
    xs = [w.pos[0] for w in lifts]
    assert xs == sorted(xs, reverse=True) and lifts[0].pos[2] < max(w.pos[2] for w in lifts)


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 600 / 20


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
