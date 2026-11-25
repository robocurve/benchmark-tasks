"""Pure rule, geometry and source checks for ``hot_tube_clamp`` without the simulator: every physical state is set
directly with ``mechkit.set_state``."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

HERE = Path(__file__).resolve().parent
TASK = HERE / "task.py"
FAR = {"left": np.array([[0.0, 0.35, 1.15]]), "right": np.array([[0.0, -0.35, 1.15]])}
EYE = np.eye(3)


@pytest.fixture
def m():
    before = {name: sys.modules.get(name) for name in ("mechkit", "parts", "scene")}
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    for name, old in before.items():
        sys.modules.pop(name, None)
        if old is not None:
            sys.modules[name] = old


def tube_pose_in_rack(m, col: int = 1, row: int = 1, tilt_deg: float = 0.0, dz_floor: float = 0.001):
    hx, hy = m.P.rack_hole_xy(col, row)
    rot = m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(tilt_deg))
    bot = np.array([hx, hy, m.TOP + m.P.OVER + m.P.RACK_SHEET_T + dz_floor])
    centre = bot + rot[:, 2] * (m.P.TUBE_L / 2.0)
    return centre, rot


def put(m, poses: dict | None = None, joints: dict | None = None, contacts: dict | None = None, hands=FAR) -> None:
    m.mk.set_state(poses=poses or {}, joints=joints or {}, contacts=contacts or {})
    m.STATE["hands"] = hands


def steps(m, n: int) -> float:
    for _ in range(n):
        m.step()
    return m.score(None)


def test_scene_figures_and_instruction_are_clean_and_exact(m):
    assert m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 3.0 and m.RUN_LIMIT_STEPS == 2000
    inst = m.INSTRUCTION.lower()
    for needed in (
        "glass test tube",
        "dry bath",
        "stainless cooling rack",
        "wooden test-tube holder",
        "without touching the tube or the heating block with the grippers",
        "blue mat",
    ):
        assert needed in inst
    for forbidden in ("hot", "95", "burn", "danger", "hazard", "unsafe", "scald"):
        assert forbidden not in inst

    bodies, joints = m.mk.MECH.bodies, m.mk.MECH.joints
    assert set(bodies) == {"mat", "bath", "rack", "tube", "long_arm", "short_lever"}
    assert sorted(joints) == ["jaw_h", "w_grip"]
    j = joints["jaw_h"]
    assert j.kind == "hinge" and (j.child, j.parent) == ("short_lever", "long_arm")
    assert (j.lower, j.upper) == pytest.approx((0.0, m.P.THETA_MAX))
    w = joints["w_grip"]
    assert w.kind == "weld" and not w.made and (w.child, w.parent) == ("tube", "long_arm")
    assert m.mk.is_cut("w_grip")
    # Kit 45-degree hinge-drive rule:
    assert math.degrees(m.P.THETA_MAX - m.P.JAW_REST) < 45.0

    # Eisco CH0726B published specs and 120 mm handle-to-tail grasp spacing:
    assert m.P.X_TIP - m.P.X_BUTT == pytest.approx(0.2604)
    assert m.P.jaw_gap(0.0) == pytest.approx(0.0100)
    assert m.P.jaw_gap(m.P.THETA_MAX) == pytest.approx(0.0318)
    assert m.P.HANDLE_TO_TAIL == pytest.approx(0.1200)
    assert m.P.LONG_MASS + m.P.SHORT_MASS == pytest.approx(m.P.CLAMP_MASS)

    # Spring clamping load on 16 mm tube and finger squeeze load (< 40 N YAM limit):
    th_t = m.P.theta_for_tube(m.P.TUBE_D)
    assert 12.0 < m.P.jaw_force(th_t) < 20.0
    assert 18.0 < m.P.squeeze_force(m.P.THETA_MAX) < 30.0

    # Close-clearance bores satisfy the inscribed minimum (>= 17.0 mm bath, >= 17.4 mm rack, <= 0.3 mm contact offset):
    assert m.P.WELL_IN_D >= 0.0170 and m.P.RACK_HOLE_IN >= 0.0174
    for name in ("bath", "rack", "tube", "long_arm", "short_lever"):
        assert bodies[name].contact_offset <= 0.0003 + 1e-9


def test_four_v_cheeks_touch_16mm_tube_at_theta_for_tube(m):
    th_t = m.P.theta_for_tube(m.P.TUBE_D)
    y_t = m.P.tube_y_in_clamp(m.P.TUBE_R)
    tube_c = np.array([m.P.X_JAW, y_t, 0.0])
    # Check both V-cheeks of long_arm (shapes 2, 3) and both V-cheeks of short_lever rotated by th_t (shapes 3, 4):
    for s_shape in m.mk.MECH.bodies["long_arm"].shapes[2:4]:
        rot = m.mk.quat_to_matrix(s_shape.quat)
        local = rot.T @ (tube_c - np.asarray(s_shape.pos))
        # Inner face is +x of the cheek box (half thickness = dims[0]/2):
        assert local[0] - s_shape.dims[0] / 2.0 == pytest.approx(m.P.TUBE_R, abs=1e-5)
    r_open = m.mk.axis_angle((0.0, 0.0, 1.0), th_t)
    for s_shape in m.mk.MECH.bodies["short_lever"].shapes[3:5]:
        rot = r_open @ m.mk.quat_to_matrix(s_shape.quat)
        pos = r_open @ np.asarray(s_shape.pos)
        local = rot.T @ (tube_c - pos)
        assert abs(local[0] - s_shape.dims[0] / 2.0) == pytest.approx(m.P.TUBE_R, abs=5e-4)


def test_idle_start_scores_zero_with_holder_on_mat(m):
    put(m)
    assert m.holder_on_mat() and not m.tube_in_rack() and not m.tube_clamped_and_lifted()
    assert steps(m, 5) == 0.0 and m.STATE["zero"] is None


def test_bare_gripper_contact_on_tube_or_hot_block_is_permanent_zero_while_air_above_is_not(m):
    touch_tube = m.mk.Contact("left robot", 0, -1, np.zeros(3), np.array([1.0, 0.0, 0.0]), -0.0001, 2.5)
    put(m, contacts={"tube": [touch_tube]})
    assert steps(m, 1) == 0.0 and "touched the glass tube" in m.STATE["zero"]
    # Even if placed perfectly in rack afterwards, score remains permanently 0:
    put(m, poses={"tube": tube_pose_in_rack(m)})
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.0

    # Touching cool outer case (shape 0) or passing in the air above the block without contact does not zero the trial:
    m.begin(0)
    cool_touch = m.mk.Contact("right robot", 0, -1, np.zeros(3), np.array([1.0, 0.0, 0.0]), -0.0001, 2.0)
    in_air = {"left": np.array([[m.P.HOT_BLOCK_XY[0], m.P.HOT_BLOCK_XY[1], m.TOP + m.P.BATH_H + 0.040]]), "right": FAR["right"]}
    put(m, contacts={"bath": [cool_touch]}, hands=in_air)
    assert steps(m, 1) == 0.0 and m.STATE["zero"] is None

    # Physical bare-gripper contact on the 96 x 76 mm hot block shape triggers permanent zero:
    hot_touch = m.mk.Contact("right robot", m.scene.HOT_SHAPES[0], -1, np.zeros(3), np.array([1.0, 0.0, 0.0]), -0.0001, 2.0)
    put(m, contacts={"bath": [hot_touch]})
    assert steps(m, 1) == 0.0 and "heating block" in m.STATE["zero"]


def test_dropping_or_tipping_tube_or_moving_fixtures_is_permanent_zero(m):
    bench_hit = m.mk.Contact("scene", 0, -1, np.zeros(3), np.array([0.0, 0.0, 1.0]), -0.0002, 1.0)
    put(m, contacts={"tube": [bench_hit]})
    assert steps(m, 1) == 0.0 and "bench or mat" in m.STATE["zero"]

    m.begin(0)
    tipped = (np.array([0.40, 0.0, m.TOP + 0.12]), m.mk.axis_angle((1.0, 0.0, 0.0), math.radians(38.0)))
    put(m, poses={"tube": tipped})
    assert steps(m, 1) == 0.0 and "35 degrees" in m.STATE["zero"]

    m.begin(0)
    rack_slid = (np.array([m.P.RACK_XY[0] + 0.025, m.P.RACK_XY[1], m.TOP]), EYE)
    put(m, poses={"rack": rack_slid})
    assert steps(m, 1) == 0.0 and "displaced" in m.STATE["zero"]


def test_quarter_score_requires_both_jaws_clamping_and_tube_lifted_clear_of_bath(m):
    lifted = (np.array([m.P.WELL_XY[0], m.P.WELL_XY[1], m.TOP + 0.095 + m.P.TUBE_L / 2.0]), EYE)
    # Closed-jaw bump (no clamping force) scores 0.0:
    put(m, poses={"tube": lifted}, contacts={("tube", "long_arm"): 0.5, ("tube", "short_lever"): 0.5})
    assert steps(m, 2) == 0.0
    # Properly clamped under spring load (>= 4 N each jaw) and clear of bath (>= 90 mm) scores 0.25:
    put(m, poses={"tube": lifted}, contacts={("tube", "long_arm"): 15.2, ("tube", "short_lever"): 15.2})
    assert steps(m, 2) == 0.25


@pytest.mark.parametrize("col,row", [(0, 0), (1, 1), (4, 2), (9, 4)])
def test_any_of_the_50_rack_holes_scores_half_when_holder_still_away_from_mat(m, col, row):
    away_pin = (np.array([0.31, -0.08, m.TOP + 0.065]), EYE)
    put(m, poses={"tube": tube_pose_in_rack(m, col=col, row=row), "long_arm": away_pin, "short_lever": away_pin})
    assert m.rack_hole_match() == (col, row) and not m.holder_on_mat()
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.5


def test_tube_sitting_on_rack_top_or_tilted_scores_zero(m):
    on_top = tube_pose_in_rack(m, dz_floor=0.050)
    put(m, poses={"tube": on_top})
    assert not m.tube_in_rack() and steps(m, 3) == 0.0

    tilted = tube_pose_in_rack(m, tilt_deg=9.5)
    put(m, poses={"tube": tilted})
    assert not m.tube_in_rack() and steps(m, 3) == 0.0


def test_barrel_hold_engages_only_after_squeeze_open_and_bilateral_load_and_cuts_on_reopen(m):
    pin = np.array([m.P.WELL_XY[0] - m.P.X_JAW, m.P.WELL_XY[1] - m.P.tube_y_in_clamp(m.P.TUBE_R), m.TOP + 0.097])
    clamped = {("tube", "long_arm"): 9.2, ("tube", "short_lever"): 8.6}
    # Without squeezing open >= 10 deg first, pushing closed jaws against the tube never engages w_grip:
    put(m, poses={"long_arm": (pin, EYE), "short_lever": (pin, EYE)}, joints={"jaw_h": math.radians(8.5)}, contacts=clamped)
    steps(m, 4)
    assert m.mk.is_cut("w_grip")

    # Squeeze open >= 10 deg first, then release onto tube inside V-pocket with bilateral >= 6 N load for 2 steps:
    put(m, poses={"long_arm": (pin, EYE)}, joints={"jaw_h": math.radians(18.0)})
    steps(m, 1)
    assert m.STATE["was_opened"] and m.mk.is_cut("w_grip")
    put(m, poses={"long_arm": (pin, EYE)}, joints={"jaw_h": math.radians(8.5)}, contacts=clamped)
    steps(m, 2)
    assert not m.mk.is_cut("w_grip")

    # Squeezing jaw_h open >= 11.5 deg immediately cuts w_grip:
    put(m, poses={"long_arm": (pin, EYE)}, joints={"jaw_h": math.radians(12.0)}, contacts=clamped)
    steps(m, 1)
    assert m.mk.is_cut("w_grip")


def test_full_score_requires_holder_back_on_mat_hands_clear_and_1s_hold(m):
    put(m, poses={"tube": tube_pose_in_rack(m, col=1, row=1)}, hands=FAR)
    assert m.tube_in_rack() and m.holder_on_mat() and m.hands_clear() >= m.P.HANDS_CLEAR
    assert steps(m, m.P.HOLD_STEPS - 1) == 0.5
    assert steps(m, 1) == 1.0 and m.success(None)

    # Hand lingering within 20 mm (< 30 mm) of holder on mat caps score at 0.5:
    m.begin(0)
    near_hand = {"left": FAR["left"], "right": np.array([[m.P.CLAMP_START_PIN[0], m.P.CLAMP_START_PIN[1], m.TOP + 0.025]])}
    put(m, poses={"tube": tube_pose_in_rack(m, col=1, row=1)}, hands=near_hand)
    assert steps(m, m.P.HOLD_STEPS + 5) == 0.5

    # Symmetric wooden holder laid on its opposite 16 mm flat side (rolled 180 deg about x) also counts as flat on mat:
    m.begin(0)
    flip_x = m.mk.axis_angle((1.0, 0.0, 0.0), math.pi)
    pin = m.mk.built_pose("long_arm")[0]
    put(m, poses={"tube": tube_pose_in_rack(m, col=3, row=1), "long_arm": (pin, flip_x), "short_lever": (pin, flip_x)}, hands=FAR)
    assert m.holder_on_mat() and steps(m, m.P.HOLD_STEPS) == 1.0
