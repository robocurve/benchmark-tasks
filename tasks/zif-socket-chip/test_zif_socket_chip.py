"""Offline checks (no GPU): the task file loads, the scorer covers each outcome and remembers when the lever closed,
the chip, the socket and the lever fit together as the trap needs, and the demo stays within reach with the arms
apart. Run in the yam_arena environment:  python -m pytest -q test_zif_socket_chip.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "zif_socket_chip.py")
T = TASK.module

SEATED_Z = T.SEAT_Z - T._bz0  # the chip's centre when its body's bottom is on the socket's top


def _yaw_quat(yaw, roll_x=0.0):
    """x y z w of a turn ``yaw`` about z after a turn ``roll_x`` about x."""
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(roll_x / 2), 0.0, 0.0, math.cos(roll_x / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.01, high=pos + 0.01)


def chip_at(xy=T.SOCKET_XY, z=SEATED_Z, yaw=0.0, tilt=0.0, vel=(0.0, 0.0, 0.0)):
    return _state((*xy, z), _yaw_quat(yaw, tilt), vel)


def lever_at(deg):
    return _state(T.LEVER.pos, _yaw_quat(0.0, math.radians(deg)))


START_CHIP = chip_at(T.CHIP_START_XY, T.CHIP.pos[2], T.CHIP_START_YAW)
SEATED = chip_at()
# The wrong way round: the solid end on the 2 mm boss, the other end on the socket's top.
# Its local -y end is then at the world's +y, raised: a negative turn about its own x.
_tilt = -math.atan2(T.BOSS[2], T.BOSS_Y + T._hl)
REVERSED = chip_at(z=SEATED_Z + T.BOSS[2] / 2, yaw=math.pi, tilt=_tilt)


def _scene(chip=START_CHIP, lever=0.0):
    return {"chip": chip, "lever": lever_at(lever)}


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"chip", "lever"}


def test_start_scores_zero():
    objects = _scene()
    assert T.score(objects) == 0.0 and not T.success(objects)
    assert T.notch_facing(objects) < -0.99  # the notch starts pointing away from the lever
    assert T.lever_angle(objects) == pytest.approx(0.0, abs=1e-6)


def test_lever_angle_reads_the_hinge():
    for deg in (0.0, 30.0, 85.0, 90.0):
        assert T.lever_angle(_scene(lever=deg)) == pytest.approx(deg, abs=1e-6)


def test_seated_open_half():
    objects = _scene(SEATED, 0.0)
    assert T.seated(objects) and T.score(objects) == 0.5 and not T.success(objects)


def test_seated_then_closed_full():
    T.score(_scene(SEATED, 0.0))
    T.score(_scene(SEATED, 60.0))
    objects = _scene(SEATED, 90.0)
    assert T.score(objects) == 1.0 and T.success(objects)
    assert T.score(_scene(SEATED, 86.0)) == 1.0  # within 5 degrees of closed


def test_seated_but_moving_is_not_success():
    T.score(_scene(SEATED, 0.0))
    objects = _scene(chip_at(vel=(0.05, 0.0, 0.0)), 90.0)
    assert T.score(objects) == 1.0 and not T.success(objects)


def test_reversed_quarter_either_lever():
    objects = _scene(REVERSED, 0.0)
    assert T.in_socket(objects) and not T.seated(objects)
    assert T.score(objects) == 0.25
    assert T.score(_scene(REVERSED, 90.0)) == 0.25


def test_reversed_ends_are_two_mm_apart():
    ends = T.body_ends_z(_scene(REVERSED))
    assert ends[1] - T.SEAT_Z > 0.0015  # this end (the solid one, at +y) sits on the boss
    assert abs(ends[0] - T.SEAT_Z) < 0.001


def test_closed_before_seating_quarter():
    high = chip_at(z=SEATED_Z + 0.002)  # pins in the slots, held 2 mm up
    T.score(_scene(high, 90.0))  # the lever closes on a chip not seated
    assert T.score(_scene(high, 90.0)) == 0.25
    assert T.score(_scene(SEATED, 90.0)) == 0.25  # pushed down after: the closed socket doesn't take it
    T.score(_scene(SEATED, 10.0))  # opened again, closed again over the seated chip
    assert T.score(_scene(SEATED, 90.0)) == 1.0


def test_lever_closed_with_no_chip_zero():
    assert T.score(_scene(START_CHIP, 90.0)) == 0.0


def test_lever_partly_closed_is_open():
    T.score(_scene(SEATED, 0.0))
    assert T.score(_scene(SEATED, 80.0)) == 0.5


def test_seat_tolerance():
    assert T.score(_scene(chip_at(z=SEATED_Z + 0.0004))) == 0.5
    assert T.score(_scene(chip_at(z=SEATED_Z + 0.0008))) == 0.0  # in the socket the right way, not down
    assert T.score(_scene(chip_at(xy=(T.SOCKET_XY[0] + 0.004, T.SOCKET_XY[1])))) == 0.0  # off the slots


def test_notch_sideways_is_not_seated():
    assert T.score(_scene(chip_at(yaw=math.pi / 2))) == 0.0


def test_chip_off_table_zero():
    T.score(_scene(SEATED, 0.0))
    assert T.score(_scene(chip_at(xy=(0.3, 0.7), z=T.TOP - 0.5), 90.0)) == 0.0


def test_non_finite_zero():
    assert T.score(_scene(chip_at(xy=(math.nan, 0.0)))) == 0.0


# -- the parts fit -----------------------------------------------------------------------------------------------------


def _part(parts, name):
    return next((c, s) for n, c, s in parts if n == name)


def _world_range(spec, name):
    c, s = _part(spec.parts, name)
    lo = np.asarray(spec.pos) + np.asarray(c) - np.asarray(s) / 2
    return lo, lo + np.asarray(s)


def test_pins_fit_their_slots_with_play():
    assert T.SLOT_W - T.PIN_W == pytest.approx(2 * 0.002)  # 2 mm each side across
    assert T.SLOT_LEN - T.PIN_LEN == pytest.approx(2 * 0.0025)
    assert T.SLOT_DEPTH > T.PIN_H  # the body, not the pins, carries the chip


def test_boss_fits_the_notch_wherever_the_play_puts_the_chip():
    lo, hi = _world_range(T.SOCKET, "boss")
    sx, sy = T.SOCKET_XY
    for dx in (-T.PLAY, 0.0, T.PLAY):
        for dy in (-T.PLAY_Y, 0.0, T.PLAY_Y):
            notch_x = (sx + dx - T.NOTCH[0] / 2, sx + dx + T.NOTCH[0] / 2)
            notch_y = (sy + dy + T._hl - T.NOTCH[1], sy + dy + T._hl)
            assert notch_x[0] < lo[0] and hi[0] < notch_x[1]
            assert notch_y[0] < lo[1] and hi[1] < notch_y[1]
    assert hi[2] - T.SEAT_Z == pytest.approx(0.002)


def test_reversed_pins_still_in_the_slots():
    assert T.SEAT_Z + T.BOSS[2] - T.PIN_H < T.SEAT_Z  # the high end's pins still reach into the slots


def test_lever_swings_clear_of_the_socket_and_board():
    sock_lo = np.asarray(T.SOCKET.pos) - np.asarray(T.SOCKET.size) / 2
    lever_x_hi = T.LEVER_X + T.LEVER_T / 2
    assert sock_lo[0] - lever_x_hi > 0.004  # 6 mm beside the socket's near side
    assert T.LEVER_PIVOT[2] - T.LEVER_T / 2 * math.sqrt(2) > T.BOARD_TOP + 0.003  # its corners clear the board
    assert T.LEVER.lower == 0.0 and T.LEVER.upper == 90.0 and T.LEVER.detents == (0.0, 90.0)
    assert T.LEVER.detent_for(44.0) == 0.0 and T.LEVER.detent_for(46.0) == 90.0


def test_chip_parts_inside_its_box():
    for name, c, s in T.CHIP_PARTS:
        assert np.all(np.abs(c) + np.asarray(s) / 2 <= np.asarray(T.CHIP_SIZE) / 2 + 1e-9), name


# -- the demo ------------------------------------------------------------------------------------------------------


def _at(steps, label):
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_turns_half_a_turn_through_the_middle():
    plan = T.demo()["right"]
    yaws = [w.yaw for w in plan]
    assert _at(plan, "seat")[1].yaw - _at(plan, "close")[1].yaw == pytest.approx(math.pi)
    assert all(abs(b - a) <= math.pi / 2 + 1e-9 for a, b in zip(yaws, yaws[1:]))  # no single half turn


def test_demo_grip_heights():
    tips = T.PICK_Z - T.PAD_TIP_PAST_TCP
    assert T.MAT_TOP + T.PIN_H < tips < T.MAT_TOP + T.PIN_H + T.CHIP_BODY[2]  # on the body, not the pins
    assert T.PLACE_Z - T.PAD_TIP_PAST_TCP > T.SEAT_Z + 0.001  # the pads clear the socket's top


def test_lever_pushed_after_the_chip_is_down():
    plan = T.demo()
    let_go, _ = _at(plan["right"], "let go")
    parked, _ = _at(plan["right"], "park")
    behind, _ = _at(plan["left"], "behind the lever")
    assert let_go < parked < behind - 1.0


def test_demo_keeps_the_arms_apart():
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < -0.02
    assert min(w.pos[1] for w in plan["left"]) > -0.03
    parked, _ = _at(plan["right"], "park")
    busy = [w for w in plan["left"] if w.label.startswith(("over the lever", "behind", "push", "hold"))]
    assert all(w.pos[1] - T.PARK_RIGHT[1] > 0.2 for w in busy)  # the right arm is parked 20 cm away


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


@pytest.mark.parametrize("name", ["chip", "lever"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The chip's and the lever's meshes stay inside their boxes, so what the cameras see is what is scored."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"chip": T.CHIP_SIZE, "lever": T.LEVER.size}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.002)  # and fill them


def test_looks_match_the_task_file():
    """make_looks.py repeats the task file's sizes: they must agree."""
    src = (HERE / "looks" / "make_looks.py").read_text()
    ns = {}
    for line in src.splitlines():
        if line.startswith(("CHIP_SIZE", "NOTCH", "PIN_X", "LEVER_SIZE", "SOCKET_W,", "SLOT_IN,", "END_Y,",
                            "BOSS_Y,", "BOARD_OFF,", "MAT_SIZE")):
            exec(line, ns)
    assert ns["CHIP_SIZE"] == pytest.approx(T.CHIP_SIZE) and ns["NOTCH"] == pytest.approx(T.NOTCH)
    assert ns["PIN_X"] == T.PIN_X and ns["LEVER_SIZE"] == pytest.approx(T.LEVER.size)
    assert ns["SOCKET_W"] == T.SOCKET_W and ns["SOCKET_TOP"] == T.SOCKET_TOP_H
    assert (ns["SLOT_IN"], ns["SLOT_OUT"], ns["SLOT_HALF"]) == pytest.approx((T.SLOT_IN, T.SLOT_OUT, T.SLOT_LEN / 2))
    assert (ns["END_Y"], ns["CAM_Y"]) == pytest.approx((T.END_Y, T.CAM_Y[1]))
    assert (ns["BOSS_Y"], *ns["BOSS"]) == pytest.approx((T.BOSS_Y, *T.BOSS))
    assert ns["MAT_SIZE"] == pytest.approx(T.MAT_SIZE) and ns["BOARD_SIZE"] == pytest.approx(T.BOARD_SIZE)
    assert ns["BOARD_OFF"] == pytest.approx((T.BOARD.pos[0] - T.SOCKET_XY[0], T.BOARD.pos[1] - T.SOCKET_XY[1]))
