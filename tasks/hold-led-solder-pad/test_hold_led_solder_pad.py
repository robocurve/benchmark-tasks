"""Offline checks (no GPU): the task file loads, the layout (the LED on its joint, the iron in its stand, the arms in
their halves), every scoring outcome, the 2 s contact timer and the episode's memory, the failure rules (the tip on
the LED, in the no-touch zone, on the table or a finger, the LED out of its holes), the one-handed shortcut, and the
demo's order and reach. Run in the yam_arena environment: python -m pytest -q test_hold_led_solder_pad.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "hold_led_solder_pad.py")
T = TASK.module
DT = 0.05  # s: one control step at 20 Hz
STEPS_2S = round(T.SOLDER_S / DT)


def _quat_z(a: float) -> np.ndarray:
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


TURNED = _quat_z(T.TURN)


def _state(pos, quat=TURNED, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


LED0 = np.asarray(T.LED.pos)
IRON0 = np.asarray(T.IRON.pos)
LED_FLUSH = LED0 - T.LED_GAP * T.N


def _scene(led=None, iron=None):
    out = {"led": _state(LED0), "iron": _state(IRON0)}
    for name, value in (("led", led), ("iron", iron)):
        if value is not None:
            out[name] = value if isinstance(value, ObjectState) else _state(value)
    return out


def _iron_tip_at(point, offset=(0.0, 0.0, 0.0)):
    """The iron, turned as in its stand, with its tip's point at ``point`` (+ ``offset``)."""
    return _state(np.asarray(point) + np.asarray(offset) - T.TIP_LOCAL[1] * T.N)


def _finger_on_led(led_pos=LED_FLUSH):
    """Finger points: one on the LED's end face, one well away."""
    end = np.asarray(led_pos) + (T.LED_LEN / 2) * T.N
    return np.array([end + 0.0005 * T.N, end + 0.05 * T.N + (0, 0, 0.05)])


def _contact(steps=STEPS_2S, offset=(0.0, 0.0, 0.0), led=LED_FLUSH, fingers="on"):
    f = _finger_on_led(led) if isinstance(fingers, str) else fingers
    for _ in range(steps):
        T.observe(_scene(led=led, iron=_iron_tip_at(T.PADS[0], offset)), f, DT)


IN_STAND = _scene()


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"led", "iron"}


def test_frame():
    assert np.allclose(np.cross(T.U, T.N), T.Z)
    assert T.N[0] > 0 and T.N[1] > 0  # the components side faces forward and to the robot's left


def test_led_starts_slid_out_on_its_joint():
    assert T.led_out(_scene()) == pytest.approx(T.LED_GAP)
    assert T.LED.lower == pytest.approx(-(T.LED_GAP + 0.0005))  # the board stops it at flush, not the joint
    assert T.LED.upper + T.LED_GAP > T.LED_OUT  # the joint lets it out past where it counts as out of its holes
    assert T.LED.rate == 0.0 and T.LED.axis == "Y" and T.LED.yaw == T.TURN
    assert not T.led_fell(_scene())
    body, half, _ = T._led_body(_scene(led=LED_FLUSH))
    assert T.board_coords(body)[1] - half[1] == pytest.approx(T.BOARD_T / 2)  # flush: its back on the board


def test_pads_on_the_solder_side_behind_the_led():
    for p in T.PADS:
        b = T.board_coords(p)
        assert b[1] == pytest.approx(-T.BOARD_T / 2) and abs(b[0] - T.LED_U) == pytest.approx(T.PAD_PITCH / 2)
    assert np.linalg.norm(T.JOINTS[:, None] - T.PADS[None], axis=2).min() > T.ZONE + 0.01  # 1 cm free round them


def test_parts_clear_the_holder_and_the_board_edges():
    for c, h in T.BODIES:
        b = T.board_coords(c)
        assert c[2] - h[2] > T.JAW_TOP + 0.002
        assert abs(b[0]) + h[0] < T.BOARD_SIZE[0] / 2 and abs(b[2]) + h[2] < T.BOARD_SIZE[2] / 2


def test_iron_lies_in_its_stand():
    objects = _scene()
    assert T.in_stand(objects)
    assert np.linalg.norm(T.tip(objects) - T.PADS[0]) > 0.15  # far from the board
    handle_bottom = T.IRON_Z - T.HANDLE[2] / 2
    assert handle_bottom == pytest.approx(T.REST_TOP)
    assert T.IRON_Z - T.BARREL[2] / 2 == pytest.approx(T.COIL_FLOOR)
    mid = [c[1] for name, c, _ in T.IRON_PARTS]
    vol = [np.prod(s) for _, _, s in T.IRON_PARTS]
    com = float(np.dot(mid, vol) / sum(vol))
    assert max(T.REST_Y) < com < T.COIL_Y[0]  # the centre of mass between the rest and the spiral
    assert abs(com - T.GRIP_LY) < 0.002  # the demo grips over it


def test_arms_work_in_their_own_halves():
    plan = T.demo()
    assert min(w.pos[1] for w in plan["left"]) > 0.0
    assert max(w.pos[1] for w in plan["right"]) < -0.08
    _, press = _at(plan["left"], "hold it flush")
    _, hold = _at(plan["right"], "hold")
    assert np.linalg.norm(np.subtract(press.pos[:2], hold.pos[:2])) > 0.15


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    T.observe(_scene(), None, DT)
    assert T.score(_scene()) == 0.0 and not T.success(_scene()) and not T.STATE["failed"]


def test_two_seconds_solders():
    _contact(STEPS_2S - 1)
    assert not T.STATE["soldered"] and T.score(_scene(led=LED_FLUSH)) == 0.25
    _contact(1)
    assert T.STATE["soldered"]
    assert T.score(_scene(led=LED_FLUSH)) == 1.0 and T.success(_scene(led=LED_FLUSH))  # the iron back in its stand


def test_soldered_iron_elsewhere_half():
    _contact()
    held_up = _iron_tip_at(T.PADS[0], (0.0, 0.0, 0.05))
    assert T.score(_scene(led=LED_FLUSH, iron=held_up)) == 0.5
    moving = _state(IRON0, vel=(0.05, 0.0, 0.0))
    assert T.score(_scene(led=LED_FLUSH, iron=moving)) == 0.5 and not T.success(_scene(led=LED_FLUSH, iron=moving))


def test_contact_must_be_continuous():
    _contact(STEPS_2S - 2)
    T.observe(_scene(led=LED_FLUSH), _finger_on_led(), DT)  # the tip off for one step
    assert T.STATE["contact_s"] == 0.0
    _contact(STEPS_2S - 1)
    assert not T.STATE["soldered"]
    _contact(1)
    assert T.STATE["soldered"]


def test_either_pad():
    for _ in range(STEPS_2S):
        T.observe(_scene(led=LED_FLUSH, iron=_iron_tip_at(T.PADS[1])), _finger_on_led(), DT)
    assert T.STATE["soldered"]


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0019), (0.0, 0.0, -0.0019)])
def test_within_2_mm_touches(offset):
    _contact(offset=offset)
    assert T.STATE["soldered"]


@pytest.mark.parametrize("offset", [(0.0, 0.0, 0.0021), (0.0, 0.0, -0.0025), tuple(-0.003 * T.N)])
def test_beyond_2_mm_does_not(offset):
    _contact(offset=offset)
    assert not T.STATE["soldered"] and not T.STATE["touched"]


def test_led_not_flush_does_not_count():
    _contact(led=LED_FLUSH + 0.0012 * T.N)
    assert not T.STATE["touched"]
    T.reset_memory()
    _contact(led=LED_FLUSH + 0.0009 * T.N)
    assert T.STATE["soldered"]


def test_one_handed_shortcut_scores_zero():
    """The tip on the pad with the LED still slid out, or pushed flush and let go."""
    _contact(led=LED0, fingers=None)
    assert T.score(_scene()) == 0.0 and not T.STATE["failed"]
    T.reset_memory()
    _contact(led=LED_FLUSH, fingers=None)
    assert T.score(_scene(led=LED_FLUSH)) == 0.0


def test_finger_must_be_on_the_led():
    near = _finger_on_led() + 0.0034 * T.N  # 3.9 mm off the end face
    _contact(fingers=near)
    assert T.STATE["soldered"]
    T.reset_memory()
    _contact(fingers=_finger_on_led() + 0.004 * T.N)  # 4.5 mm off
    assert not T.STATE["touched"]


def test_soldering_is_remembered():
    _contact()
    T.observe(_scene(led=LED_FLUSH), None, DT)
    assert T.STATE["soldered"] and T.score(_scene(led=LED_FLUSH)) == 1.0


def test_tip_on_the_led_fails():
    end = LED_FLUSH + (T.LED_LEN / 2) * T.N
    T.observe(_scene(led=LED_FLUSH, iron=_iron_tip_at(end + 0.001 * T.N)), None, DT)
    assert T.STATE["failed"] == "the tip touched the LED"
    _contact()
    assert T.score(_scene(led=LED_FLUSH)) == 0.0


def test_tip_on_the_pad_is_not_the_led():
    """Through the 1.6 mm board, the tip on the pad is 1.6 mm from the LED's body: not a touch."""
    T.observe(_scene(led=LED_FLUSH, iron=_iron_tip_at(T.PADS[0], 0.0005 * T.N)), None, DT)
    assert not T.STATE["failed"]


@pytest.mark.parametrize("k", range(4))
def test_no_touch_zone(k):
    joint = T.JOINTS[k * 3 % len(T.JOINTS)]
    T.observe(_scene(iron=_iron_tip_at(joint, (0.0, 0.0, 0.009))), None, DT)
    assert T.STATE["failed"] == "the tip entered the no-touch zone"


def test_just_outside_the_zone():
    joint = T.JOINTS[0]
    T.observe(_scene(iron=_iron_tip_at(joint, (0.0, 0.0, -0.012))), None, DT)
    assert not T.STATE["failed"]


def test_tip_on_the_table_fails():
    p = np.array([0.2, -0.2, T.TOP + 0.001])
    assert T.score(_scene(iron=_iron_tip_at(p))) == 0.0 and T.STATE["failed"] == "the tip touched the table"


def test_tip_on_a_finger_fails():
    t = T.tip(_scene())
    T.observe(_scene(), np.array([t + (0.002, 0.0, 0.0)]), DT)
    assert T.STATE["failed"] == "the tip touched a gripper"


def test_gripping_the_handle_is_fine():
    grip = np.asarray(T.IRON.pos) + T.GRIP_LY * T.N
    T.observe(_scene(), np.array([grip + (0.0, 0.0, 0.01)]), DT)
    assert not T.STATE["failed"]


@pytest.mark.parametrize("led", [LED_FLUSH + 0.0101 * T.N, LED_FLUSH + 0.003 * T.U, LED_FLUSH + (0, 0, -0.05)])
def test_led_out_of_its_holes_fails(led):
    _contact()
    assert T.score(_scene(led=led)) == 0.0
    assert T.STATE["failed"] == "the LED came out of its holes"
    assert T.score(_scene(led=LED_FLUSH)) == 0.0  # remembered


def test_led_turned_fails():
    tilted = _state(LED_FLUSH, quat=np.array([math.sin(0.1), 0.0, 0.0, math.cos(0.1)]))  # 11.5 degrees
    assert T.led_fell({"led": tilted, "iron": _state(IRON0)})


def test_led_pulled_halfway_is_still_in():
    assert not T.led_fell(_scene(led=LED_FLUSH + 0.009 * T.N))


def test_iron_half_in_the_stand_is_not_in():
    out = _state(IRON0 - 0.105 * T.N)  # the tip 5 mm short of the spiral's mouth
    assert not T.in_stand(_scene(iron=out))
    reversed_ = _state(IRON0, quat=_quat_z(T.TURN + math.pi))  # tip toward the robot
    assert not T.in_stand(_scene(iron=reversed_))
    deep = _state(IRON0 + 0.008 * T.N)  # pushed 8 mm further in: still in, handle within 1.5 cm of the rest
    assert T.in_stand(_scene(iron=deep))


def test_non_finite_zero():
    _contact()
    assert T.score(_scene(iron=(math.nan, 0.0, 0.0))) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(steps, label):
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_holds_the_led_through_the_contact():
    plan = T.demo()
    flush, _ = _at(plan["left"], "push it flush")
    on, _ = _at(plan["right"], "tip on the pad")
    held, hold = _at(plan["right"], "hold")
    off, _ = _at(plan["right"], "tip off")
    released, _ = _at(plan["left"], "hold it flush")
    assert flush < on - 3.0 and released > off
    assert held - on == T.HOLD_S >= T.SOLDER_S + 0.5


def test_demo_tip_lands_on_the_pad():
    """Where the demo sends the TCP, the iron's tip meets the near pad (before the PRESS and the tuned AIM)."""
    _, poke = _at(T.demo()["right"], "tip on the pad")
    tip = np.asarray(poke.pos) - (0.0, 0.0, T.AIM[1]) - T.AIM[0] * T.U + (T.TIP_FROM_GRIP - T.PRESS) * T.N
    assert np.allclose(tip, T.PADS[0], atol=1e-9)
    assert abs(poke.yaw - math.pi / 4) < math.radians(6)  # fingers close across the iron (a small tuned trim)


def test_demo_takes_the_iron_out_and_back_along_the_spiral():
    plan = T.demo()["right"]
    _, grip = _at(plan, "close")
    _, out = _at(plan, "out of the spiral")
    _, back = _at(plan, "into the spiral")
    d = np.subtract(out.pos, grip.pos)
    assert np.allclose(d[:2], -T.WITHDRAW * T.N[:2]) and d[2] == pytest.approx(T.LIFT)
    tip_ly = T.IRON_SIZE[1] / 2 - T.WITHDRAW
    assert tip_ly < T.COIL_Y[0] - 0.02  # out past the spiral's mouth before it moves sideways
    assert np.allclose(back.pos[:2], grip.pos[:2])
    assert T.TRAVEL_Z - T.HANDLE[2] / 2 > T.COIL_FLOOR + 2 * T.COIL_R + T.WIRE + 0.01
    assert T.IRON_Z + T.LIFT - 0.0128 > T.REST_TOP + T.LIP_UP + 0.003  # the fingertips clear the rest's lips
    assert T.IRON_Z + T.LIFT + T.BARREL[2] / 2 < T.COIL_FLOOR + 2 * T.COIL_R - 0.004  # the barrel clears the spiral


def test_demo_left_finger_pushes_the_led_flush():
    _, press = _at(T.demo()["left"], "push it flush")
    _, down = _at(T.demo()["left"], "down beside it")
    b_press, b_down = T.board_coords(press.pos), T.board_coords(down.pos)
    assert b_press[1] - T.FACE_OFF < T.LED_END_N  # past the flush LED's end
    assert b_down[1] - T.FACE_OFF > T.LED_END_N + T.LED_GAP + 0.02  # comes down clear of the slid-out LED
    assert abs(b_press[0] - T.LED_U) < 1e-9
    assert press.pos[2] - 0.0128 > T.JAW_TOP  # the fingertips clear the holder


def test_shortcut_variant_keeps_the_left_arm_back():
    T.HOLD_LED = False
    try:
        plan = T.demo()
    finally:
        T.HOLD_LED = True
    assert len(plan["left"]) == 1


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


@pytest.mark.parametrize("name", ["led", "iron"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The free parts' meshes stay inside their boxes, so their centres are their boxes' centres."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"led": T.LED.size, "iron": T.IRON_SIZE}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)


def test_finger_cloud_from_the_urdf():
    import fingers

    for tip in fingers.TIPS:
        pts = fingers.cloud(tip)
        assert 200 < len(pts) < 20000
