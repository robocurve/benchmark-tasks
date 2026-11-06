"""Offline checks (no GPU): the task file loads, the scorer covers each outcome and remembers a squeeze, the slide,
the lock and the buttons are specified as the task needs, and the demo squeezes before it pulls, keeps the arms
apart and stays within reach. Run in the yam_arena environment:  python -m pytest -q test_drill_battery_release.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "drill_battery_release.py")
T = TASK.module

DRILL0 = T.START["drill"].copy()
PACK0 = np.asarray(T.BATTERY.pos)
UNTURNED = (0.0, 0.0, 0.0, 1.0)


def _quat_z(angle, q=UNTURNED):
    """``q`` turned ``angle`` about the vertical (x, y, z, w)."""
    s, c = math.sin(angle / 2), math.cos(angle / 2)
    x, y, z, w = q
    return (c * x - s * y, c * y + s * x, c * z + s * w, c * w - s * z)


def _state(pos, quat=UNTURNED, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def _scene(pack=None, drill=None, pressed=(0.0, 0.0), pack_quat=UNTURNED):
    """The start, with the pack's centre (or state), the drill's centre (or state) and how far each button is in."""
    pack_state = pack if isinstance(pack, ObjectState) else _state(PACK0 if pack is None else pack, pack_quat)
    drill_state = drill if isinstance(drill, ObjectState) else _state(DRILL0 if drill is None else drill, T.DRILL_QUAT)
    turn = T._matrix(pack_state.quat_xyzw)
    out = {"drill": drill_state, "battery": pack_state}
    for name, s, depth in (("button_left", 1, pressed[0]), ("button_right", -1, pressed[1])):
        out[name] = _state(pack_state.pos + turn @ np.array([0.0, s * (T.BUTTON_OFFSET - depth), 0.0]),
                           pack_state.quat_xyzw)
    return out


SQUEEZE = (0.003, 0.003)
OUT = np.array([-1.0, 0.0, 0.0])
IN_TRAY = (*T.TRAY_XY, T.TRAY_FLOOR_Z + T.BATTERY_SIZE[2] / 2)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"drill", "battery", "button_left", "button_right"}


def test_the_mechanism_is_specified():
    assert T.BATTERY.to == "drill" and T.BATTERY.axis == "X" and T.BATTERY.friction == 10.0
    assert T.BATTERY.lower < -T.TRAVEL and T.BATTERY.upper == 0.0  # the slide is switched off before its stop
    for b in T.BUTTONS:
        assert b.to == "battery" and b.axis == "Y" and not b.collide
        assert b.upper - b.lower == pytest.approx(T.BUTTON_TRAVEL) == pytest.approx(0.003)
        assert b.rate * T.BUTTON_TRAVEL == pytest.approx(5.0)
        assert b.size[1] == pytest.approx(T.BUTTON_TRAVEL)  # flush with the pack when fully in
    left, right = T.BUTTONS
    assert left.lower < 0 and right.upper > 0  # each pressed toward the pack
    assert T.PRESSED < T.BUTTON_TRAVEL
    assert T.DRILL.mass == 1.2 and T.BATTERY.mass == 0.45
    assert T.DRILL.strip and T.DRILL.package == "powerTool_drill_a01"


def test_layout():
    pack_top_x = T.BATTERY.pos[0] + T.BATTERY_SIZE[0] / 2
    foot_x = T.on_drill((0.0, 0.0, T.FOOT_Z))[0]
    assert 0.001 < foot_x - pack_top_x < 0.004  # the pack just below the handle's foot, not touching it
    assert T.BATTERY.pos[2] - T.BATTERY_SIZE[2] / 2 == pytest.approx(T.TOP, abs=0.001)  # on the table
    assert T.on_drill((0.0, T.DRILL_SIDE, 0.1))[2] == pytest.approx(T.TOP, abs=0.001)  # the drill on its left side
    assert T.BATTERY_SIZE[2] / 2 == pytest.approx(T.DRILL_SIDE, abs=0.001)  # as thick as the drill
    across = T.BATTERY_SIZE[1] + 2 * T.BUTTON_SIZE[1]
    assert across < 0.094 - 0.015  # the open gripper (94 mm) clears the pack and its buttons by 7 mm or more a side
    for b in T.BUTTONS:
        assert abs(b.pos[1] - T.BATTERY.pos[1]) - b.size[1] / 2 == pytest.approx(T.BATTERY_SIZE[1] / 2)
    lo = np.asarray(T.TRAY_XY) - np.asarray(T.TRAY_OUTER) / 2
    assert lo[1] > T.BATTERY.pos[1] + T.BATTERY_SIZE[1] / 2 + 0.05  # the tray clear of the pack and the drill
    assert lo[1] > T.on_drill(T.DRILL_LOW)[1] + 0.01
    assert T.RING_XY[1] < 0 < T.BATTERY.pos[1]  # the right arm holds on its side, the left pulls on its side


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0
    assert not T.success(_scene())


def test_squeezed_but_not_slid_zero():
    assert T.score(_scene(pressed=SQUEEZE)) == 0.0
    assert T.STATE["squeezed"]


def test_one_button_is_not_a_squeeze():
    pack = PACK0 + 0.02 * OUT
    assert T.score(_scene(pack, pressed=(0.003, 0.0))) == 0.0
    assert T.score(_scene(pack, pressed=(0.0, 0.003))) == 0.0
    assert T.score(_scene(pack, pressed=(0.003, 0.001))) == 0.0  # under 1.5 mm is not pressed
    assert not T.STATE["squeezed"]


def test_squeezed_and_slid_quarter():
    assert T.score(_scene(PACK0 + 0.012 * OUT, pressed=SQUEEZE)) == 0.25


def test_squeeze_is_remembered():
    T.score(_scene(pressed=SQUEEZE))
    assert T.score(_scene(PACK0 + 0.03 * OUT)) == 0.25  # buttons let go, the pack still slid


def test_slid_under_a_centimetre_zero():
    assert T.score(_scene(PACK0 + 0.008 * OUT, pressed=SQUEEZE)) == 0.0


def test_dragging_the_drill_along_is_not_sliding():
    """Pulled with the drill loose, the drill comes along: the pack hasn't slid on the handle."""
    shift = 0.06 * OUT
    objects = _scene(PACK0 + shift, drill=DRILL0 + shift, pressed=SQUEEZE)
    assert T.slide(objects) == pytest.approx(0.0, abs=1e-9)
    assert T.score(objects) == 0.0


def test_turning_the_drill_with_its_pack_is_not_sliding():
    turn = 0.3
    q = _quat_z(turn, T.DRILL_QUAT)
    rz = T._matrix(_quat_z(turn))
    drill = _state(DRILL0, q)
    pack = _state(DRILL0 + rz @ (PACK0 - DRILL0), _quat_z(turn))
    objects = _scene(pack, drill=drill, pressed=SQUEEZE)
    assert abs(T.slide(objects)) < 1e-9 and T.separation(objects) < 1e-9
    assert T.score(objects) == 0.0


def test_off_the_drill_half():
    held_up = PACK0 + 0.07 * OUT + (0.0, 0.0, 0.08)
    assert T.score(_scene(held_up, pressed=SQUEEZE)) == 0.5
    on_table = PACK0 + 0.08 * OUT
    assert T.score(_scene(on_table)) == 0.5


def test_in_tray_succeeds():
    for quat in (UNTURNED, _quat_z(1.2)):
        T.reset_memory()
        T.score(_scene(pressed=SQUEEZE))
        objects = _scene(IN_TRAY, pack_quat=quat)
        assert T.success(objects) and T.score(objects) == 1.0


def test_in_tray_without_a_squeeze_zero():
    """Can't happen with the lock, but the scorer asks for the squeeze before any credit."""
    objects = _scene(IN_TRAY)
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_in_tray_still_moving_half():
    T.score(_scene(pressed=SQUEEZE))
    objects = _scene(_state(IN_TRAY, vel=(0.05, 0.0, 0.0)))
    assert not T.success(objects) and T.score(objects) == 0.5


def test_held_above_the_tray_half():
    T.score(_scene(pressed=SQUEEZE))
    assert T.score(_scene((*T.TRAY_XY, T.TRAY_FLOOR_Z + 0.10))) == 0.5


def test_in_tray_but_drill_moved_half():
    T.score(_scene(pressed=SQUEEZE))
    moved = _scene(IN_TRAY, drill=DRILL0 + (0.0, -0.11, 0.0))
    assert not T.success(moved) and T.score(moved) == 0.5
    kept = _scene(IN_TRAY, drill=DRILL0 + (0.0, -0.09, 0.0))
    assert T.success(kept) and T.score(kept) == 1.0


def test_drill_and_pack_in_the_tray_together_is_not_off():
    T.score(_scene(pressed=SQUEEZE))
    shift = np.array([*T.TRAY_XY, 0.0]) - np.array([*PACK0[:2], 0.0])
    objects = _scene(PACK0 + shift + (0.0, 0.0, 0.01), drill=DRILL0 + shift + (0.0, 0.0, 0.01))
    assert T.score(objects) == 0.0


def test_fallen_off_the_table_zero():
    T.score(_scene(pressed=SQUEEZE))
    assert T.score(_scene((0.2, 0.7, T.TOP - 0.5))) == 0.0
    assert T.score(_scene(IN_TRAY, drill=(0.3, -0.7, T.TOP - 0.5))) == 0.0


def test_non_finite_zero():
    assert T.score(_scene((math.nan, 0.0, 0.0), pressed=SQUEEZE)) == 0.0


# -- the demo ----------------------------------------------------------------------------------------------------------


def _at(steps, label):
    """When the waypoint ``label`` is reached (s from the start) and the waypoint."""
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_holds_and_squeezes_before_it_pulls():
    plan = T.demo()
    held, _ = _at(plan["right"], "hold the drill")
    squeezed, squeeze = _at(plan["left"], "squeeze both buttons")
    slid, pull = _at(plan["left"], "slide it off")
    lifted, _ = _at(plan["left"], "lift")
    let_go, _ = _at(plan["right"], "let go")
    assert held <= squeezed < slid - 2.0 and lifted <= let_go - 0.6
    assert squeeze.pos[:2] == pytest.approx(T.BATTERY.pos[:2]) and squeeze.yaw == 0.0  # closing across the buttons
    assert T.BATTERY.pos[0] - pull.pos[0] > T.TRAVEL + 0.02  # past the travel, with room for the arm's give
    assert pull.pos[1] == squeeze.pos[1]  # straight along the handle


def test_demo_pads_cover_the_buttons():
    tips = T.PACK_Z - T.PAD_TIP_PAST_TCP
    button_low = T.BUTTONS[0].pos[2] - T.BUTTON_SIZE[2] / 2
    assert T.TOP + 0.005 < tips < button_low - 0.01  # under the buttons, clear of the table
    ring_tips = T.HOLD_Z - T.PAD_TIP_PAST_TCP
    assert T.TOP + 0.005 < ring_tips < T.on_drill(T.RING_IN_DRILL)[2] - 0.01  # under the ring's axis
    pack_bottom = T.PLACE_Z - T._pack_below_tcp
    assert T.TRAY_FLOOR_Z < pack_bottom < T.TRAY_FLOOR_Z + 0.01
    assert T.CARRY_Z - T._pack_below_tcp > T.TOP + T.TRAY_WALL_H + 0.03  # the pack clears the tray's walls


def test_no_hold_variant_stays_up():
    T.HOLD = False
    try:
        right = T.demo()["right"]
        assert min(w.pos[2] for w in right) == T.TRAVEL_Z
    finally:
        T.HOLD = True


def test_demo_keeps_the_arms_apart():
    """Each arm in its own half, and the two TCPs 15 cm or more apart at every moment."""
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < 0.0
    assert min(w.pos[1] for w in plan["left"]) > 0.0
    arms = {arm: ArmPlan(plan[arm], BASES[arm]) for arm in ("left", "right")}
    duration = max(p.duration for p in arms.values())
    gap = min(np.linalg.norm(arms["left"].target(t)[0] - arms["right"].target(t)[0])
              for t in np.arange(0.0, duration, 0.05))
    assert gap > 0.15, gap


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


def test_the_pack_mesh_fits_its_box():
    """The pack's mesh stays inside its collider (and fills it), so what the cameras see is what collides."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS["battery"]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray(T.BATTERY_SIZE) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
    assert np.all(np.asarray(box.GetMax()) - np.asarray(box.GetMin()) >= 2 * half - 0.002)
