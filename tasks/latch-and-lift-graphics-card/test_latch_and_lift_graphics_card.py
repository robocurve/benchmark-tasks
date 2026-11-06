"""Offline checks (no GPU): the task file loads, the scorer covers each outcome and remembers a pressed latch and a
forced card, the bond and the button are specified as the trap needs, and the demo presses before it lifts and stays
within reach. Run in the yam_arena environment:  python -m pytest -q test_latch_and_lift_graphics_card.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "latch_and_lift_graphics_card.py")
T = TASK.module

CARD_START = np.asarray(T.CARD.pos)
LATCH_START = np.asarray(T.LATCH.pos)
DOWN = np.array([0.0, 0.0, -T.PRESSED - 0.0005])


def _state(pos, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def _scene(card=None, latch=None):
    """The start, with the card or the latch replaced by a state or a centre."""
    out = {"card": _state(CARD_START), "latch": _state(LATCH_START)}
    for name, value in (("card", card), ("latch", latch)):
        if value is not None:
            out[name] = value if isinstance(value, ObjectState) else _state(value)
    return out


PRESSED = LATCH_START + DOWN
IN_TRAY = (*T.TRAY_XY, T.TRAY_FLOOR_Z + T.CARD_SIZE[1] / 2)  # lying flat on the foam
STANDING_IN_TRAY = (*T.TRAY_XY, T.TRAY_FLOOR_Z + T.CARD_SIZE[2] / 2)
RAISED = CARD_START + (0.0, 0.0, 0.006)  # up 6 mm, still between the rails
OUT = CARD_START + (0.0, 0.0, 0.05)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"card", "latch"}


def test_the_trap_is_specified():
    assert T.CARD.to == "mainboard" and not T.BOARD.free  # locked to the bench, through the fixed board
    assert T.CARD.hold == 60.0 and math.isinf(T.CARD.twist)
    assert T.LATCH.axis == "Z" and T.LATCH.lower == -T.LATCH_TRAVEL and T.LATCH.upper == 0.0
    assert T.PRESSED < T.LATCH_TRAVEL
    assert T.LATCH_RATE * T.LATCH_TRAVEL == pytest.approx(2.0)


def test_layout():
    assert T.CARD.pos[2] - T.CARD_SIZE[2] / 2 == pytest.approx(T.BOARD_TOP)  # the card's bottom edge on the board
    assert T.LATCH.pos[2] - T.LATCH_SIZE[2] / 2 - T.LATCH_TRAVEL > T.BOARD_TOP  # clear of the board when fully down
    assert T.LATCH_TOP - T.LATCH_TRAVEL > T.BOARD_TOP + T.HOUSING_H  # and still out of its housing
    for wall in T.HOUSING:
        assert not wall.free and wall.pos[2] + wall.size[2] / 2 == pytest.approx(T.BOARD_TOP + T.HOUSING_H)
    gap = T.CARD_U - T.CARD_SIZE[0] / 2 - (T.LATCH_U + T.LATCH_SIZE[0] / 2)
    assert 0.02 < gap < 0.06  # room for the pressing gripper, next to the card's near end
    for rail in T.RAILS:
        assert abs(rail.pos[0] - T.SLOT_LINE_X) - rail.size[1] / 2 == pytest.approx(T.CARD_SIZE[1] / 2 + T.RAIL_GAP)


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0
    assert not T.success(_scene())


def test_pressed_but_not_lifted_zero():
    assert T.score(_scene(latch=PRESSED)) == 0.0


def test_pressed_and_raised_quarter():
    assert T.score(_scene(card=RAISED, latch=PRESSED)) == 0.25


def test_pressed_then_released_still_counts():
    T.score(_scene(latch=PRESSED))
    assert T.score(_scene(card=RAISED)) == 0.25  # the spring has brought the latch back up


def test_pressed_less_than_three_mm_is_locked():
    shallow = LATCH_START + (0.0, 0.0, -0.002)
    assert T.score(_scene(card=RAISED, latch=shallow)) == 0.0
    assert T.STATE["broken"]


def test_raised_without_pressing_breaks_the_latch():
    assert T.score(_scene(card=RAISED)) == 0.0
    assert T.STATE["broken"]
    put_right = _scene(card=IN_TRAY, latch=PRESSED)
    assert T.score(put_right) == 0.0 and not T.success(put_right)
    T.reset_memory()
    assert T.score(put_right) == 1.0


def test_small_deflection_is_not_broken():
    assert T.score(_scene(card=CARD_START + (0.0, 0.0, 0.003))) == 0.0
    assert not T.STATE["broken"]


def test_out_of_slot_half():
    assert T.score(_scene(card=OUT, latch=PRESSED)) == 0.5
    assert T.score(_scene(card=(0.45, -0.10, T.BOARD_TOP + 0.01), latch=PRESSED)) == 0.5  # laid on the board


def test_in_tray_succeeds():
    for pose in (IN_TRAY, STANDING_IN_TRAY):
        T.reset_memory()
        objects = _scene(card=pose, latch=PRESSED)
        assert T.success(objects) and T.score(objects) == 1.0


def test_in_tray_still_moving_half():
    objects = _scene(card=_state(IN_TRAY, vel=(0.05, 0.0, 0.0)), latch=PRESSED)
    assert not T.success(objects) and T.score(objects) == 0.5


def test_held_above_the_tray_half():
    high = (*T.TRAY_XY, T.TRAY_FLOOR_Z + 0.10)
    assert T.score(_scene(card=high, latch=PRESSED)) == 0.5


def test_card_on_floor_zero():
    assert T.score(_scene(card=(0.2, 0.6, T.TOP - 0.5), latch=PRESSED)) == 0.0


def test_non_finite_zero():
    assert T.score(_scene(card=(math.nan, 0.0, 0.0))) == 0.0


def _at(steps, label):
    """When the waypoint ``label`` is reached (s from the start) and the waypoint."""
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_presses_before_it_lifts():
    plan = T.demo()
    pressed, press = _at(plan["right"], "press")
    held, _ = _at(plan["right"], "hold")
    closed, _ = _at(plan["left"], "close")
    lifted, lift = _at(plan["left"], "lift")
    assert pressed < closed < lifted <= held
    assert press.finger == T.FINGER_CLOSED  # pressing with the fingertips closed
    assert lift.pos[:2] == plan["left"][2].pos[:2] and lift.pos[2] - plan["left"][2].pos[2] > 0.03  # straight up


def test_demo_press_depth():
    tips = T.PRESS_Z - T.PAD_TIP_PAST_TCP
    assert T.LATCH_TOP - 0.02 < tips < T.LATCH_TOP - T.LATCH_TRAVEL  # past full travel: the arm must lean on it


def test_demo_grips_the_top_of_the_card():
    tips = T.GRIP_Z - T.PAD_TIP_PAST_TCP
    assert T.CARD_TOP - 0.03 < tips < T.CARD_TOP
    assert T.TRAVEL_Z - T.PAD_TIP_PAST_TCP + T.GRIP_DEPTH - T.CARD_SIZE[2] > T.TOP + T.TRAY_WALL_H  # clears the walls


def test_demo_keeps_the_arms_apart():
    """Each arm works in its own half of the table: the right arm on the latch, the left on the card and the tray."""
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < -0.15
    assert min(w.pos[1] for w in plan["left"]) > -0.03
    assert min(w.pos[1] for w in plan["left"]) - max(w.pos[1] for w in plan["right"]) > 0.15  # wrists 15 cm apart


def test_board_parts_turn_together():
    for box in (T.MAT, T.BOARD, *T.RAILS, *T.HOUSING, T.CARD, T.LATCH):
        assert box.yaw == T.TURN
    assert T.LATCH.pos[1] < T.CARD.pos[1] - T.CARD_SIZE[0] / 2  # the latch on the right of the card
    assert T.MAT.pos[1] + T.MAT_SIZE[0] / 2 < T.TRAY_XY[1] - T.TRAY_OUTER[1] / 2  # the tray clear of the mat


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["card", "latch"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The card's and the latch's meshes stay inside their colliders, so what the cameras see is what collides."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"card": T.CARD_SIZE, "latch": T.LATCH_SIZE}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.002)  # and fill them
