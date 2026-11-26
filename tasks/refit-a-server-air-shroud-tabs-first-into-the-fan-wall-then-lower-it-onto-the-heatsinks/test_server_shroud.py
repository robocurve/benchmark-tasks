"""Pure-Python unit tests of the 2U server air-shroud task's figures, geometry, and scoring rules (no simulator)."""

from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR_HANDS = {
    "left": np.array([[0.0, 0.32, 1.15]]),
    "right": np.array([[0.0, -0.32, 1.15]]),
}
EYE = np.eye(3)


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene", "checks")}
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def shroud_at(m, x: float | None = None, y: float = 0.0, h_floor: float = 0.0, pitch_deg: float = 0.0, yaw_deg: float = 0.0):
    """Shroud pose (pos, rot): origin at (x, y, FLOOR_Z + h_floor), pitched nose-down (+y axis) and yawed (+z axis)."""
    if x is None:
        x = m.P.X_SEAT
    r_yaw = m.mk.axis_angle((0.0, 0.0, 1.0), math.radians(yaw_deg))
    r_pitch = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(pitch_deg))
    return np.array([x, y, m.FLOOR_Z + h_floor], dtype=float), r_yaw @ r_pitch


def put(m, shroud, hands=None, hand_hulls=None) -> None:
    m.mk.set_state(poses={"shroud": shroud})
    m.STATE["hands"] = FAR_HANDS if hands is None else hands
    m.STATE["hand_hulls"] = {} if hand_hulls is None else hand_hulls


def run_steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return float(m.score(None))


def test_scene_standards_and_instruction_invariants(m):
    assert m.SCENE == "station" and m.SCORE_OVER == "end"
    assert m.SUCCESS_HOLD_S == pytest.approx(3.0) and m.RUN_LIMIT_STEPS == 1000
    inst = m.INSTRUCTION
    low = inst.lower()
    # Goal-only neutral single sentence naming all scored objects and conditions
    assert "air shroud" in low and "motherboard bay" in low and "both front tabs" in low and "fan-wall slots" in low
    assert "processor heatsinks" in low and "memory modules" in low and "let go" in low
    for banned in ("tilt", "nose-down", "pitch", "first", "danger", "hazard", "unsafe"):
        assert banned not in low, banned

    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert sorted(bodies) == ["chassis", "shroud"]
    assert bodies["chassis"].kinematic and not bodies["chassis"].free
    assert bodies["shroud"].free and bodies["shroud"].mass == pytest.approx(m.P.SHROUD_MASS)
    assert bodies["chassis"].contact_offset == pytest.approx(0.0006)
    assert bodies["shroud"].contact_offset == pytest.approx(0.0006)
    assert len(joints) == 0  # pure rigid contact: zero hinges, slides, or welds

    # Standards & maker dimensions
    assert (m.P.CHASSIS_W, m.P.CHASSIS_H, m.P.CHASSIS_D) == pytest.approx((0.430, 0.0881, 0.400))
    assert (m.P.HS_L, m.P.HS_W, m.P.HS_H, m.P.HS_MASS) == pytest.approx((0.118, 0.078, 0.064, 0.5266))
    assert (m.P.DIMM_L, m.P.DIMM_H) == pytest.approx((0.13335, 0.03125))
    assert (m.P.FAN_W, m.P.FAN_D) == pytest.approx((0.080, 0.038))
    assert m.P.DUCT_CLEAR == pytest.approx(0.0025)
    assert m.P.SHROUD_L + m.P.TAB_L > m.P.BAY_OPEN_L + 0.003  # 170.0 mm > 165.5 mm


def test_idle_staged_start_scores_zero(m):
    put(m, m.mk.built_pose("shroud"))
    assert run_steps(m, 25) == 0.0
    assert m.STATE["zero"] is None
    assert m.tab_engagements() == {"left": 0.0, "right": 0.0}


def test_flat_drop_onto_fan_lip_or_rear_divider_scores_zero(m):
    # Case 1: flat drop over bay lands front tabs on top of the 83.0 mm bulkhead lip (h_floor = 83 - 72 = 11 mm)
    put(m, shroud_at(m, x=m.P.X_SEAT, h_floor=0.011, pitch_deg=0.0))
    assert m.tab_engagements() == {"left": 0.0, "right": 0.0}
    assert run_steps(m, 25) == 0.0

    # Case 2: shifted rearward (-x) so tabs miss the bulkhead (x_tip < 0.4730 m) and rear lands on the 52.0 mm divider
    m.begin(0)
    put(m, shroud_at(m, x=m.P.X_SEAT - 0.012, h_floor=0.052, pitch_deg=0.0))
    assert m.tab_engagements() == {"left": 0.0, "right": 0.0}
    assert run_steps(m, 25) == 0.0


def test_single_tab_tucked_scores_zero(m):
    # Yawed -2.2 deg and shifted back so left tab is ~6.6 mm in slot (>= 5 mm) while right tab is < 1 mm (< 5 mm)
    pose = shroud_at(m, x=m.P.X_BULKHEAD - m.P.SHROUD_L / 2.0 - 0.006, h_floor=0.0, yaw_deg=-2.2)
    put(m, pose)
    eng = m.tab_engagements()
    assert eng["left"] >= 0.005 and eng["right"] < 0.005
    assert run_steps(m, 25) == 0.0


def test_both_tabs_tucked_with_rear_unseated_scores_half(m):
    # Pivoted 6.0 deg nose-down around tucked tabs inside slots -> rear corners stand > 90 mm (> 79.5 mm)
    piv_loc = np.array([m.P.SHROUD_L / 2.0 + 0.0055, 0.0, m.P.TAB_Z_MID])
    piv_w = np.array([m.P.X_SEAT + m.P.SHROUD_L / 2.0 + 0.0055, 0.0, m.FLOOR_Z + m.P.TAB_Z_MID])
    r = m.mk.axis_angle((0.0, 1.0, 0.0), math.radians(6.0))
    put(m, (piv_w - r @ piv_loc, r))
    assert m.both_tabs_engaged() and not m.corners_seated()
    assert run_steps(m, 25) == 0.5


def test_full_score_requires_four_corners_seated_hands_clear_and_1s_hold(m):
    seated = shroud_at(m, x=m.P.X_SEAT, h_floor=0.0, pitch_deg=0.0)
    # Hand 18 mm above shroud roof (< 30 mm) -> stays at 0.5
    close_hand = {"left": np.array([[m.P.X_SEAT, m.P.GRASP_Y, m.FLOOR_Z + m.P.SHROUD_H + 0.018]]), "right": FAR_HANDS["right"]}
    put(m, seated, hands=close_hand)
    assert m.hands_clear() == pytest.approx(0.018, abs=1e-6)
    assert run_steps(m, m.P.HOLD_STEPS + 5) == 0.5

    # Hands >= 30 mm clear: 19 steps -> 0.5, 20th step (1.0 s) -> 1.0
    put(m, seated, hands=FAR_HANDS)
    assert m.both_tabs_engaged() and m.corners_seated() and m.hands_clear() >= m.P.HANDS_CLEAR
    assert run_steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert run_steps(m, 1) == 1.0 and m.success(None)


def test_hold_resets_when_shroud_drifts_over_2mm(m):
    seated = shroud_at(m, x=m.P.X_SEAT, h_floor=0.0)
    put(m, seated)
    assert run_steps(m, m.P.HOLD_STEPS - 4) == 0.5
    shifted = shroud_at(m, x=m.P.X_SEAT - 0.0024, h_floor=0.0)  # 2.4 mm > 2.0 mm drift cap, tabs still 5.1 mm in slots
    put(m, shifted)
    assert run_steps(m, 1) == 0.5 and m.STATE["hold"] == 1
    assert run_steps(m, m.P.HOLD_STEPS - 2) == 0.5
    assert run_steps(m, 1) == 1.0


@pytest.mark.parametrize(
    "foul_pt, expected_sub",
    [
        ((0.3915, 0.095, 0.7475 + 0.003 + 0.072), "heatsink left"),
        ((0.3915, -0.095, 0.7475 + 0.003 + 0.072), "heatsink right"),
        ((0.3915, 0.025, 0.7475 + 0.003 + 0.038), "dimm inner left"),
        ((0.3915, -0.176, 0.7475 + 0.003 + 0.038), "dimm outer right"),
    ],
)
def test_gripper_touching_heatsink_or_dimm_permanently_zeros_trial(m, foul_pt, expected_sub):
    put(m, m.mk.built_pose("shroud"), hands={"left": np.array([foul_pt]), "right": FAR_HANDS["right"]})
    assert run_steps(m, 1) == 0.0
    assert expected_sub in (m.STATE["zero"] or "")
    # Subsequent full seating remains 0.0 permanently
    put(m, shroud_at(m, x=m.P.X_SEAT, h_floor=0.0), hands=FAR_HANDS)
    assert run_steps(m, m.P.HOLD_STEPS + 5) == 0.0 and not m.success(None)


def test_dropping_shroud_outside_bay_permanently_zeros_trial(m):
    put(m, (np.array([0.30, 0.25, m.TOP + 0.010]), EYE))
    assert run_steps(m, 1) == 0.0
    assert m.STATE["zero"] == "the shroud fell outside the 2U server bay"
    put(m, shroud_at(m, x=m.P.X_SEAT, h_floor=0.0), hands=FAR_HANDS)
    assert run_steps(m, m.P.HOLD_STEPS + 5) == 0.0


def test_code_and_folder_hygiene():
    task_text = (HERE / "task.py").read_text()
    assert len(task_text) <= 15_000
    for fname in ("task.py", "scene.py", "parts.py", "demo.py"):
        fpath = HERE / fname
        if not fpath.is_file():
            continue
        code = fpath.read_text()
        for forbidden in ("weld(", "push(", ".push", "write_pose", "set_joint"):
            assert forbidden not in code, f"{fname} contains {forbidden!r}"
    readme = HERE / "README.md"
    if readme.is_file():
        rtext = readme.read_text()
        assert len(rtext) <= 10_000
        low = rtext.lower()
        assert "earlier review" not in low and "reviewer" not in low and "thank" not in low
    for path in HERE.iterdir():
        assert not path.is_symlink(), path.name
        if path.suffix in (".py", ".md", ".json"):
            txt = path.read_text()
            assert not re.search(r"TK-[A-Z0-9]{6}", txt) and not re.search(r"#\d{3,}", txt), path.name
