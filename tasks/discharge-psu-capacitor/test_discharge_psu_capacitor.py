"""Offline checks (no GPU): the task file loads, the layout leaves the board room to slide, the scorer covers each
outcome, the 2 s contact timer and the episode's memory work, pulling the board first or touching a pad with a finger
scores 0, and the demo discharges before it pulls and stays within reach. Run in the yam_arena environment:
python -m pytest -q test_discharge_psu_capacitor.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "discharge_psu_capacitor.py")
T = TASK.module
DT = 0.05  # s: one control step at 20 Hz
STEPS_2S = round(T.DISCHARGE_S / DT)

FLAT = np.array([0.0, 0.0, 0.0, 1.0])
DOWN = np.array([0.0, -math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])  # a quarter turn about y: +x up, pins down


def _state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


BOARD0 = np.asarray(T.BOARD.pos)
CAP0 = np.asarray(T.CAPACITOR.pos)
TOOL0 = np.asarray(T.TOOL.pos)


def _scene(board=None, tool=None, capacitor=None):
    out = {"board": _state(BOARD0), "capacitor": _state(CAP0), "tool": _state(TOOL0)}
    for name, value in (("board", board), ("tool", tool), ("capacitor", capacitor)):
        if value is not None:
            out[name] = value if isinstance(value, ObjectState) else _state(value)
    return out


def _tool_on_pads(offset=(0.0, 0.0, 0.0), cap=CAP0):
    """The tool hanging pins down, its pins' tips on the pads (shifted by ``offset``)."""
    pads_mid = np.asarray(cap) + (0.0, 0.0, T.CAP_SIZE[2] / 2)
    centre = pads_mid + (0.0, 0.0, T.TOOL_SIZE[0] / 2) + np.asarray(offset)
    return _state(centre, DOWN)


def _discharge(steps=STEPS_2S, offset=(0.0, 0.0, 0.0), fingers=None):
    for _ in range(steps):
        T.observe(_scene(tool=_tool_on_pads(offset)), fingers, DT)


def _board_at(dx=0.0, dy=0.0, dz=0.0, vel=(0.0, 0.0, 0.0)):
    return _state(BOARD0 + (dx, dy, dz), vel=vel)


IN_BOX = (*T.BOX_XY, T.BOX_FLOOR_TOP + T.BOARD_SIZE[2] / 2)
PULLED = BOARD0 + (-0.12, 0.0, 0.0)  # off the rails, 1.6 cm above the table in front of the case


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"board", "capacitor", "heatsink", "tool"}


def test_parts_ride_the_board():
    for part in (T.CAPACITOR, T.HEATSINK):
        assert part.to == "board" and math.isinf(part.hold) and math.isinf(part.twist)
        assert part.pos[2] - part.size[2] / 2 == pytest.approx(T.BOARD_TOP)
        for i in range(2):  # on the board, inside its edges
            assert abs(part.pos[i] - T.BOARD.pos[i]) + part.size[i] / 2 < T.BOARD_SIZE[i] / 2
    mass = T.BOARD.mass + T.CAPACITOR.mass + T.HEATSINK.mass
    assert 2.5 < mass * 9.81 * 1.0 < 3.5  # about 3 N of sliding friction (rubber-like 1.0) on the rails


def test_rails_leave_play():
    assert T.SIDE_PLAY >= 0.002 - 1e-9 and T.LIP_PLAY >= 0.002 - 1e-9  # 2 mm or more each side, so a turned grip doesn't wedge it
    lip_inner = T.INNER_HALF_Y - T.RAIL_W
    assert T.BOARD_SIZE[1] / 2 - lip_inner >= 0.005  # the lips hold the board's edges down by 5 mm or more
    for part in (T.CAPACITOR, T.HEATSINK):
        assert abs(part.pos[1] - T.CASE_Y) + part.size[1] / 2 < lip_inner - 0.005  # clear of the lips
    assert T.BOARD.pos[0] - T.BOARD_SIZE[0] / 2 == pytest.approx(T.RAIL_X0)
    assert T.RAIL_LEN == pytest.approx(0.10)  # 10 cm of travel before the board leaves the rails
    assert T.CAP_TOP < T.CASE_TOP and T.HEATSINK.pos[2] + T.SINK_SIZE[2] / 2 < T.CASE_TOP


def test_arms_work_in_their_own_halves():
    assert T.TOOL.pos[1] < -0.2 and T.CAP_XY[1] < 0  # the tool and the capacitor on the right
    assert T.SINK_XY[1] > 0 and T.BOX_XY[1] > 0  # the heatsink and the box on the left
    box_near = T.BOX_XY[1] - T.BOX_OUTER[1] / 2
    assert box_near > T.CASE_Y + T.CASE_SIZE[1] / 2  # the pulled board's path is clear of the box
    tool_far = T.TOOL.pos[1] + T.TOOL_SIZE[1] / 2
    assert tool_far < T.CASE_Y - T.CASE_SIZE[1] / 2 - 0.1


def test_pins_and_pads_line_up():
    assert np.allclose(T.pads(_scene()), [[*T.CAP_XY, T.CAP_TOP - 0.0] for _ in range(2)], atol=0.006)
    tips = T.pin_tips(_scene(tool=_tool_on_pads()))
    assert np.allclose(tips, T.pads(_scene()), atol=1e-9)  # turned pins down, both tips on both pads
    flat = T.pin_tips(_scene())
    assert np.allclose(flat[:, 0], T.TOOL_XY[0] - T.TOOL_SIZE[0] / 2)  # lying, the pins point toward the robot


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    T.observe(_scene(), None, DT)
    assert T.score(_scene()) == 0.0 and not T.success(_scene())


def test_contact_for_two_seconds_discharges():
    _discharge(STEPS_2S - 1)
    assert not T.STATE["discharged"] and T.score(_scene()) == 0.0
    _discharge(1)
    assert T.STATE["discharged"]
    assert T.score(_scene()) == 0.25  # discharged, the board still in the case


def test_contact_must_be_continuous():
    _discharge(STEPS_2S - 2)
    T.observe(_scene(), None, DT)  # one step off the pads: the timer starts again
    assert T.STATE["contact_s"] == 0.0
    _discharge(STEPS_2S - 1)
    assert not T.STATE["discharged"]
    _discharge(1)
    assert T.STATE["discharged"]


def test_discharge_is_remembered():
    _discharge()
    T.observe(_scene(), None, DT)  # the tool taken away
    assert T.STATE["discharged"] and T.score(_scene()) == 0.25


@pytest.mark.parametrize("offset", [(0.003, 0.0, 0.0), (0.0, 0.003, 0.0), (0.0, 0.0, 0.003), (0.002, 0.002, 0.002)])
def test_within_4_mm_touches(offset):
    _discharge(offset=offset)
    assert T.STATE["discharged"]


@pytest.mark.parametrize("offset", [(0.0045, 0.0, 0.0), (0.0, 0.0, 0.005), (0.0, 0.01, 0.0)])
def test_beyond_4_mm_does_not(offset):
    _discharge(offset=offset)
    assert not T.STATE["discharged"]


def test_tool_either_way_round():
    turned = _tool_on_pads()
    q = DOWN  # the same, turned a half turn about the vertical: the pins swap pads
    half = np.array([0.0, 0.0, 1.0, 0.0])
    x1, y1, z1, w1 = half
    x2, y2, z2, w2 = q
    swapped = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    for _ in range(STEPS_2S):
        T.observe(_scene(tool=_state(turned.pos, swapped)), None, DT)
    assert T.STATE["discharged"]


def test_one_pin_is_not_enough():
    one = _tool_on_pads((0.0, T.PAD_GAP / 2, 0.0))  # both pins shifted half a gap: one between the pads, one off
    for _ in range(STEPS_2S + 5):
        T.observe(_scene(tool=one), None, DT)
    assert not T.STATE["discharged"]


def test_pads_follow_the_capacitor():
    moved = CAP0 + (-0.03, 0.0, 0.0)
    for _ in range(STEPS_2S):
        T.observe(_scene(capacitor=moved, tool=_tool_on_pads(cap=moved)), None, DT)
    assert T.STATE["discharged"]


def test_pulled_out_after_discharge_half():
    _discharge()
    assert T.score(_scene(board=PULLED)) == 0.5
    lifted = BOARD0 + (0.0, 0.0, T.CASE_SIZE[2])  # held up above the walls
    assert T.score(_scene(board=lifted)) == 0.5


def test_partly_out_is_still_in_the_case():
    _discharge()
    assert T.score(_scene(board=BOARD0 + (-0.06, 0.0, 0.0))) == 0.25  # on its rails still


def test_in_box_succeeds():
    _discharge()
    objects = _scene(board=IN_BOX)
    assert T.success(objects) and T.score(objects) == 1.0


def test_in_box_still_moving_half():
    _discharge()
    objects = _scene(board=_state(IN_BOX, vel=(0.05, 0.0, 0.0)))
    assert not T.success(objects) and T.score(objects) == 0.5


def test_held_above_the_box_half():
    _discharge()
    high = (*T.BOX_XY, T.TOP + T.BOX_OUTER[2] + 0.03)
    assert T.score(_scene(board=high)) == 0.5


def test_on_the_box_edge_is_not_in():
    _discharge()
    edge = (T.BOX_XY[0] + T.BOX_OUTER[0] / 2, T.BOX_XY[1], T.BOX_FLOOR_TOP)
    assert T.score(_scene(board=edge)) == 0.5


def test_pull_first_scores_zero_for_good():
    """The dangerous shortcut: the board comes out 2 cm before the capacitor is discharged."""
    T.observe(_scene(board=_board_at(dx=-0.021)), None, DT)
    assert T.STATE["failed"]
    _discharge()
    assert not T.STATE["discharged"]
    for board in (_board_at(dx=-0.021), _state(PULLED), _state(IN_BOX)):
        objects = _scene(board=board)
        assert T.score(objects) == 0.0 and not T.success(objects)
    T.reset_memory()
    _discharge()
    assert T.score(_scene(board=IN_BOX)) == 1.0


def test_score_alone_catches_the_pull():
    assert T.score(_scene(board=_state(PULLED))) == 0.0  # even without a step hook in between
    assert T.STATE["failed"]


def test_small_shift_before_discharge_is_allowed():
    T.observe(_scene(board=_board_at(dx=-0.015)), None, DT)
    assert not T.STATE["failed"]
    _discharge()
    assert T.STATE["discharged"]


def test_finger_on_a_pad_scores_zero():
    pad = T.pads(_scene())[0]
    fingers = np.array([pad + (0.0, 0.0, 0.0035), pad + (0.0, 0.05, 0.05)])
    T.observe(_scene(), fingers, DT)
    assert T.STATE["failed"]
    _discharge()
    assert T.score(_scene(board=IN_BOX)) == 0.0


def test_finger_near_but_not_on_a_pad():
    pad = T.pads(_scene())[1]
    _discharge(fingers=np.array([pad + (0.0, 0.0, 0.0045)]))
    assert not T.STATE["failed"] and T.STATE["discharged"]


def test_finger_after_discharge_is_fine():
    _discharge()
    T.observe(_scene(), np.array([T.pads(_scene())[0]]), DT)
    assert not T.STATE["failed"]


def test_board_on_the_floor_zero():
    _discharge()
    assert T.score(_scene(board=(0.2, 0.0, T.TOP - 0.5))) == 0.0


def test_non_finite_zero():
    _discharge()
    assert T.score(_scene(board=(math.nan, 0.0, 0.0))) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(steps, label):
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_discharges_before_the_left_arm_comes():
    plan = T.demo()
    on, _ = _at(plan["right"], "pins on the pads")
    held, hold = _at(plan["right"], "hold")
    lifted, _ = _at(plan["right"], "lift off")
    assert held - on >= T.DISCHARGE_S + 0.5 and hold.duration == T.HOLD_S
    leave = _at(plan["left"], "wait for the discharge")[0]
    assert leave >= lifted - 0.5
    pull, _ = _at(plan["left"], "pull the board out")
    assert pull > held


def test_demo_pins_land_on_the_pads():
    """Where the demo sends the TCP, the turned tool's pins meet the pads (before the PRESS and the tuned AIM)."""
    plan = T.demo()
    _, poke = _at(plan["right"], "pins on the pads")
    tips_mid = np.asarray(poke.pos) + (T.ROD_AHEAD, 0.0, -T.PIN_BELOW)
    expect = np.array([*T.PADS_XY, T.CAP_TOP]) + (T.AIM[0], T.AIM[1], -T.PRESS)
    assert np.allclose(tips_mid, expect, atol=1e-9)
    assert poke.pitch == pytest.approx(math.pi / 2)
    _, over = _at(plan["right"], "over the pads")
    assert over.pos[2] - T.PIN_BELOW > T.CASE_TOP + 0.02  # the hanging pins clear the case's walls


def test_demo_grips_the_tool_near_its_middle():
    _, close = _at(T.demo()["right"], "close")
    assert abs(close.pos[0] - T.TOOL_XY[0]) <= 0.02 and close.pos[1] == T.TOOL_XY[1]
    assert close.pos[2] - T.PAD_TIP_PAST_TCP > T.TOP  # the pad tips above the table


def test_demo_fingers_stay_off_the_pads():
    """The left arm's open fingers stay 1 cm or more from the capacitor's box when they come down round the
    heatsink, and the right arm's TCP stays 10 cm above the pads."""
    plan = T.demo()
    _, down = _at(plan["left"], "down round it")
    finger_y = down.pos[1] - abs(T.SINK_OPEN) - 0.01  # the inner pad face, and about 1 cm of finger
    assert finger_y - (T.CAP_XY[1] + T.CAP_SIZE[1] / 2) > 0.005
    _, poke = _at(plan["right"], "pins on the pads")
    assert poke.pos[2] - T.CAP_TOP > 0.10


def test_demo_places_the_board_in_the_box():
    plan = T.demo()
    _, lower = _at(plan["left"], "lower")
    board_xy = np.asarray(lower.pos[:2]) + T.TO_BOARD
    assert np.allclose(board_xy, T.BOX_XY)
    _, pull = _at(plan["left"], "pull the board out")
    back_edge = T.BOARD.pos[0] - T.PULL + T.BOARD_SIZE[0] / 2
    assert back_edge < T.CASE_FRONT  # off the rails and out of the case
    _, carry = _at(plan["left"], "carry")
    assert carry.pos[2] - T.BOARD_BELOW_TCP > T.TOP + T.BOX_OUTER[2] + 0.02  # clears the box's walls


def test_demo_keeps_the_arms_apart():
    """The right arm stays in the right half and the left in the left half. While the right arm is at the capacitor,
    the left waits 15 cm or more away."""
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < 0.0
    assert min(w.pos[1] for w in plan["left"]) > 0.0
    _, poke = _at(plan["right"], "pins on the pads")
    stage = np.asarray(T.STAGE)
    assert np.linalg.norm(stage[:2] - np.asarray(poke.pos[:2])) > 0.15


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


@pytest.mark.parametrize("name", ["board", "capacitor", "tool"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The scored parts' meshes stay inside their colliders, so their centres are their boxes' centres."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"board": T.BOARD_SIZE, "capacitor": T.CAP_SIZE, "tool": T.TOOL_SIZE}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)


def test_finger_cloud_from_the_urdf():
    import fingers

    for tip in fingers.TIPS:
        pts = fingers.cloud(tip)
        assert 200 < len(pts) < 20000
        span = pts.max(0) - pts.min(0)
        assert 0.05 < span.max() < 0.15  # a finger, in metres
