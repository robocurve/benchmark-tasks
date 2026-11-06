"""Offline checks (no GPU): the task file loads, the buttons and the layout are as specified, the hold timer and its
memory work, the scorer covers each outcome, and the demo tests each phone before it carries it and stays within the
right arm's reach. Run in the yam_arena environment:  python -m pytest -q test_phone_power_test_sort.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "phone_power_test_sort.py")
T = TASK.module
DT = 0.05  # 20 Hz
STEPS_TO_TEST = round(T.HOLD_S / DT)  # 50


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def _yaw(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _start():
    out = {}
    for phone, button in zip(T.PHONES, T.BUTTON_OBJECTS):
        out[phone.name] = _state(phone.pos)
        out[button.name] = _state(button.pos)
    return out


def _put(objects, i, pos, quat=(0.0, 0.0, 0.0, 1.0), depth=0.0, vel=(0.0, 0.0, 0.0)):
    """Phone i at ``pos`` turned by ``quat``, its button pressed in ``depth`` m."""
    rot = T._rotation(quat)
    objects[T.NAMES[i]] = _state(pos, quat, vel)
    local = np.array([T.BUTTON_DX, T.BUTTON_REST_Y + depth, 0.0])
    objects[T.BUTTONS[i]] = _state(np.asarray(pos) + rot @ local, quat, vel)
    return objects


def _press(i, steps, depth=0.002, objects=None):
    objects = objects or _start()
    _put(objects, i, T.PHONES[i].pos, depth=depth)
    for _ in range(steps):
        T.update(objects, DT)
    return objects


def _in(tray, layer=0):
    tx, ty = T.TRAYS[tray]
    return (tx, ty, T.TRAY_FLOOR_TOP + layer * T.PHONE_SIZE[2] + T.PHONE_SIZE[2] / 2)


def _sorted(objects=None, which=(0, 1, 2)):
    """The phones in ``which`` lying in their right trays (the dead ones stacked in the grey tray)."""
    objects = objects or _start()
    layer = 0
    for i in which:
        tray = T.right_tray(i)
        _put(objects, i, _in(tray, 0 if tray == "green" else layer))
        layer += tray == "grey"
    return objects


def _test_all():
    for i in range(3):
        _press(i, STEPS_TO_TEST)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {*T.NAMES, *T.BUTTONS}


def test_buttons_are_specified():
    for button, phone in zip(T.BUTTON_OBJECTS, T.NAMES):
        assert button.to == phone and button.axis == "Y" and not button.collide
        assert button.lower == 0.0 and button.upper == T.BUTTON_TRAVEL == 0.0025
    assert T.PRESSED < T.BUTTON_TRAVEL < T.BUTTON_SIZE[1]  # pressed fully, it is still 1 mm proud
    assert T.BUTTON_RATE * T.BUTTON_TRAVEL == pytest.approx(3.0)
    assert T.PHONE_MASS + T.BUTTON_OBJECTS[0].mass == pytest.approx(0.18)
    assert T.WORKING == 1 and T.NAMES[T.WORKING] == "phone_2"


def test_layout():
    for phone, block, button in zip(T.PHONES, T.BLOCKS, T.BUTTON_OBJECTS):
        assert phone.pos[2] - T.PHONE_SIZE[2] / 2 == pytest.approx(block.pos[2] + T.BLOCK_SIZE[2] / 2)  # on its block
        assert T.BLOCK_SIZE[1] < T.PHONE_SIZE[1] and T.BLOCK_SIZE[0] < T.PHONE_SIZE[0]  # its edges hang free
        inner = button.pos[1] + T.BUTTON_SIZE[1] / 2
        assert inner == pytest.approx(phone.pos[1] - T.PHONE_SIZE[1] / 2)  # the button rests against the right edge
        assert inner < block.pos[1] - T.BLOCK_SIZE[1] / 2 - 0.01  # 1 cm or more past the block's edge: it hangs free
    gaps = np.diff(sorted(T.PHONE_YS))
    assert np.all(gaps - T.PHONE_SIZE[1] - T.BUTTON_SIZE[1] > 0.04)  # room for a finger between the phones
    for tray in T.TRAYS.values():  # trays in front of the phones, clear of them
        assert tray[0] + T.TRAY_OUTER[0] / 2 < T.PHONE_X - T.PHONE_SIZE[0] / 2
    green, grey = T.TRAYS["green"], T.TRAYS["grey"]
    assert abs(green[1] - grey[1]) > T.TRAY_OUTER[1]  # side by side, apart
    names = [o.name for o in T.OBJECTS]
    assert len(names) == len(set(names))


def test_everything_in_the_right_half():
    for o in T.OBJECTS:
        assert o.pos[1] < -0.02, o.name


# -- the button and the hold timer -----------------------------------------------------------------------------------


def test_button_depth_in_the_phones_frame():
    objects = _start()
    assert T.button_depth(objects, 0) == pytest.approx(0.0, abs=1e-9)
    for quat in (_yaw(0.0), _yaw(math.pi / 2), _yaw(-2.0), (0.3, 0.1, 0.2, 0.927)):
        q = np.asarray(quat) / np.linalg.norm(quat)
        _put(objects, 0, (0.2, -0.1, T.TOP + 0.1), q, depth=0.002)
        assert T.button_depth(objects, 0) == pytest.approx(0.002)
        assert T.pressed(objects, 0)
    _put(objects, 0, (0.2, -0.1, T.TOP + 0.1), _yaw(1.0), depth=0.001)
    assert not T.pressed(objects, 0)


def test_hold_2_5_s_tests_the_phone():
    _press(1, STEPS_TO_TEST - 1)
    assert not T.STATE["tested"][1] and not T.lit(1)
    _press(1, 1)
    assert T.STATE["tested"][1] and T.lit(1)


def test_a_break_restarts_the_timer():
    objects = _press(1, STEPS_TO_TEST - 5)
    _put(objects, 1, T.PHONES[1].pos, depth=0.0)
    T.update(objects, DT)
    assert T.STATE["held"][1] == 0
    _press(1, STEPS_TO_TEST - 1)
    assert not T.STATE["tested"][1]


def test_a_shallow_press_never_tests():
    _press(1, 200, depth=0.001)
    assert not T.STATE["tested"][1]


def test_tested_is_remembered_after_release():
    objects = _press(1, STEPS_TO_TEST)
    _put(objects, 1, T.PHONES[1].pos, depth=0.0)
    for _ in range(10):
        T.update(objects, DT)
    assert T.STATE["tested"][1] and T.lit(1)


def test_dead_phones_never_light():
    _test_all()
    assert T.STATE["tested"] == [True] * 3
    assert [T.lit(i) for i in range(3)] == [False, True, False]


def test_timer_follows_the_control_rate():
    objects = _start()
    _put(objects, 0, T.PHONES[0].pos, depth=0.002)
    for _ in range(25):  # 2.5 s at 10 Hz
        T.update(objects, 0.1)
    assert T.STATE["tested"][0]


def test_reset_forgets():
    _test_all()
    T.update(_sorted(), DT)
    T.reset_memory()
    assert T.STATE["held"] == [0, 0, 0] and T.STATE["tested"] == [False] * 3 and T.STATE["early"] == [False] * 3


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    assert T.score(_start()) == 0.0
    assert not T.success(_start())


def test_all_tested_and_sorted_succeeds():
    _test_all()
    objects = _sorted()
    T.update(objects, DT)
    assert T.success(objects) and T.score(objects) == 1.0


def test_partial_credit_per_phone():
    _test_all()
    for which, expected in (((0,), 1 / 3), ((0, 2), 2 / 3), ((1,), 1 / 3), ((0, 1), 2 / 3)):
        objects = _sorted(which=which)
        assert T.score(objects) == pytest.approx(expected)
        assert not T.success(objects)


def test_untested_phones_score_nothing():
    objects = _sorted()
    T.update(objects, DT)
    assert T.score(objects) == 0.0
    assert T.STATE["early"] == [True] * 3


def test_tested_after_entering_a_tray_does_not_count():
    objects = _sorted()
    T.update(objects, DT)  # all three entered untested
    _test_all()  # then tested
    assert T.STATE["tested"] == [True] * 3
    assert T.score(_sorted()) == 0.0


def test_one_untested_phone_costs_its_third():
    for i in (0, 2):
        _press(i, STEPS_TO_TEST)
    objects = _sorted()
    T.update(objects, DT)
    assert T.score(objects) == pytest.approx(2 / 3)
    assert T.STATE["early"] == [False, True, False]


def test_short_press_does_not_count():
    for i in range(3):
        _press(i, STEPS_TO_TEST - 1)
    objects = _sorted()
    T.update(objects, DT)
    assert T.score(objects) == 0.0


def test_wrong_trays_score_nothing():
    _test_all()
    objects = _start()
    _put(objects, 0, _in("green"))
    _put(objects, 1, _in("grey"))
    _put(objects, 2, _in("green", 1))
    assert T.score(objects) == 0.0


def test_all_in_grey_after_testing_scores_two_thirds():
    _test_all()
    objects = _start()
    for layer, i in enumerate(range(3)):
        _put(objects, i, _in("grey", layer))
    assert T.score(objects) == pytest.approx(2 / 3)


def test_moving_phone_does_not_count_yet():
    _test_all()
    objects = _sorted()
    _put(objects, 1, _in("green"), vel=(0.05, 0.0, 0.0))
    assert T.score(objects) == pytest.approx(2 / 3) and not T.success(objects)


def test_held_above_the_tray_does_not_count():
    _test_all()
    objects = _sorted()
    tx, ty = T.TRAYS["green"]
    _put(objects, 1, (tx, ty, T.TRAY_FLOOR_TOP + 0.06))
    assert T.score(objects) == pytest.approx(2 / 3)


def test_tested_above_the_tray_then_lowered_counts():
    tx, ty = T.TRAYS["green"]
    objects = _start()
    _put(objects, 1, (tx, ty, T.TOP + T.ENTER_Z + 0.01), depth=0.002)  # held over the tray, above its walls
    for _ in range(STEPS_TO_TEST):
        T.update(objects, DT)
    _put(objects, 1, _in("green"))
    T.update(objects, DT)
    assert T.counted(objects, 1)


def test_phone_off_the_table_zeroes_everything():
    _test_all()
    objects = _sorted()
    _put(objects, 0, (0.3, -0.8, T.TOP - 0.5))
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_non_finite_zero():
    _test_all()
    objects = _sorted()
    _put(objects, 2, (math.nan, 0.0, 0.0))
    assert T.score(objects) == 0.0


def test_phone_left_on_its_block_scores_nothing():
    _test_all()
    objects = _sorted(which=(0, 1))
    assert T.score(objects) == pytest.approx(2 / 3)


# -- the demo --------------------------------------------------------------------------------------------------------


def _times(plan):
    t, out = 0.0, {}
    for w in plan:
        t += w.duration
        out[w.label] = (t, w)
    return out


def test_demo_tests_each_phone_before_carrying_it():
    plan = T.demo()["right"]
    at = _times(plan)
    for i in range(3):
        n = T.NAMES[i]
        squeezed, grip = at[f"squeeze {n}"]
        held, _ = at[f"hold {n}"]
        lifted, _ = at[f"lift {n}"]
        assert grip.finger == T.FINGER_CLOSED
        assert held - squeezed >= T.HOLD_S + 0.3  # squeezed on the block for 2.5 s and more
        assert lifted > held
        # the grip is on the button: across the phone at the button's place along it
        assert grip.pos[0] == pytest.approx(T.BUTTON_OBJECTS[i].pos[0])
        assert grip.pos[1] == pytest.approx(T.PHONE_YS[i] - T.BUTTON_SIZE[1] / 2)


def test_demo_sorts_by_the_working_phone():
    at = _times(T.demo()["right"])
    for i in range(3):
        tray = T.right_tray(i)
        _, lower = at[f"lower {T.NAMES[i]}"]
        assert lower.pos[0] - T.GRIP_DX == pytest.approx(T.TRAYS[tray][0])
        assert lower.pos[1] + T.SQUEEZE_SHIFT == pytest.approx(T.TRAYS[tray][1])
        assert abs(T.SQUEEZE_SHIFT) < 0.015


def test_demo_pads_clear_the_trays_and_blocks():
    pad_gap = -2 * T.FINGER_OPEN
    assert pad_gap > T.PHONE_SIZE[1] + T.BUTTON_SIZE[1] + 0.012  # 6 mm or more each side when open
    assert -2 * T.FINGER_RELEASE < T.TRAY_OUTER[1] - 2 * T.TRAY_WALL - 0.015  # inside the walls when let go
    assert -2 * T.FINGER_RELEASE > T.PHONE_SIZE[1] + T.BUTTON_SIZE[1] + 0.006  # and clear of the phone
    tips = T.GRIP_Z - PAD_TIP_PAST_TCP
    assert T.TOP + 0.01 < tips < T.PHONE_Z - T.PHONE_SIZE[2] / 2  # below the phone, above the table
    for w in T.demo()["right"]:
        if w.label.startswith("lower"):
            assert w.pos[2] - PAD_TIP_PAST_TCP > T.TRAY_FLOOR_TOP  # the pad tips stay off the floor


def test_demo_uses_the_right_arm_in_its_own_half():
    plan = T.demo()
    assert set(plan) == {"right"}  # the left arm stays at its ready pose, 60 cm away
    assert max(w.pos[1] for w in plan["right"]) < -0.03


def test_demo_fits_the_run_limit():
    seconds = sum(w.duration for w in T.demo()["right"])
    assert seconds * 20 + 20 * T.SUCCESS_HOLD_S < 900  # the demo's 900 steps


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    waypoints = T.demo().get(arm)
    if waypoints is None:
        pytest.skip("this arm stays at its ready pose")
    plan = ArmPlan(waypoints, BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes ------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["phone_1", "phone_2", "phone_3"])
def test_phone_meshes_fit_their_boxes_and_have_both_screens(name):
    """The phones' meshes stay inside their colliders, so what the cameras see is what collides and scores."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray(T.PHONE_SIZE) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.001)
    parts = {p.GetName() for p in stage.Traverse()}
    assert {"screen_on", "screen_off"} <= parts
