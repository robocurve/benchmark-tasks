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


def _quat_from_matrix(m):
    """x, y, z, w of a rotation matrix."""
    w = math.sqrt(max(1.0 + m[0, 0] + m[1, 1] + m[2, 2], 0.0)) / 2
    if w > 0.1:
        return ((m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w), w)
    x = math.sqrt(max(1.0 + m[0, 0] - m[1, 1] - m[2, 2], 0.0)) / 2  # a half turn about an axis: use the largest
    return (x, (m[0, 1] + m[1, 0]) / (4 * x), (m[0, 2] + m[2, 0]) / (4 * x), (m[2, 1] - m[1, 2]) / (4 * x))


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
IN_TRAY = (*T.TRAY_XY, T.TRAY_FLOOR_Z + T.BATTERY_SIZE[2] / 2)  # lying flat on the tray's floor, in its middle


def _let_go(released=True):
    """What the step hook records: no finger within 3 mm of the pack."""
    T.STATE["released"] = released


STEP_S = 1.0 / T.CONTROL_HZ
HOLD_STEPS = round(T.STILL_S * T.CONTROL_HZ)  # 20 control steps: 1 s at 20 Hz


def _hold(objects, steps=HOLD_STEPS):
    """What the step hook does each control step for ``steps`` steps in this state: time the full-success state."""
    for _ in range(steps):
        T.tick(objects, STEP_S)
    return objects


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
        _let_go()
        objects = _hold(_scene(IN_TRAY, pack_quat=quat))
        assert T.success(objects) and T.score(objects) == 1.0


def test_full_score_needs_one_second_without_a_break():
    """Placed, let go and still: 0.5 until the state has held 1 s (20 control steps) without a break, then 1.0. Any
    lapse (a finger back on the pack, the pack moving) starts the second over."""
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    placed = _scene(IN_TRAY)
    assert not T.success(placed) and T.score(placed) == 0.5  # placed, but not timed at all yet
    _hold(placed, HOLD_STEPS - 1)
    assert not T.success(placed) and T.score(placed) == 0.5
    _hold(placed, 1)
    assert T.success(placed) and T.score(placed) == 1.0
    _let_go(False)  # a finger touches the pack again
    _hold(placed, 1)
    _let_go(True)
    _hold(placed, HOLD_STEPS - 1)
    assert not T.success(placed) and T.score(placed) == 0.5
    _hold(_scene(_state(IN_TRAY, vel=(0.02, 0.0, 0.0))), 1)  # a nudge: it moves for one step
    _hold(placed, HOLD_STEPS - 1)
    assert T.score(placed) == 0.5
    _hold(placed, 1)
    assert T.score(placed) == 1.0
    T.reset_memory()
    assert T.STATE["placed_s"] == 0.0


def test_no_full_score_while_any_condition_is_missing():
    """Each full-success condition missing in turn, held far longer than 1 s: never 1.0."""
    tilt = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(0.4), -math.sin(0.4)], [0.0, math.sin(0.4), math.cos(0.4)]])
    wall_x = T.TRAY_XY[0] + T.TRAY_OUTER[0] / 2 - T.TRAY_WALL
    cases = {
        "not squeezed": (False, True, _scene(IN_TRAY)),
        "not let go": (True, False, _scene(IN_TRAY)),
        "held above the floor": (True, True, _scene((*IN_TRAY[:2], IN_TRAY[2] + 0.01))),
        "tilted": (True, True, _scene(_state(IN_TRAY, _quat_from_matrix(tilt)))),
        "over a wall": (True, True, _scene((wall_x, T.TRAY_XY[1], IN_TRAY[2]))),
        "moving": (True, True, _scene(_state(IN_TRAY, vel=(0.02, 0.0, 0.0)))),
        "drill moved": (True, True, _scene(IN_TRAY, drill=DRILL0 + (0.0, -0.11, 0.0))),
        "still on the drill": (True, True, _scene(PACK0 + 0.02 * OUT)),
    }
    for name, (squeezed, released, objects) in cases.items():
        T.reset_memory()
        if squeezed:
            T.score(_scene(pressed=SQUEEZE))
        _let_go(released)
        _hold(objects, 5 * HOLD_STEPS)
        assert not T.success(objects) and T.score(objects) < 1.0, name


def test_step_hook_times_the_second(monkeypatch):
    """work_the_lock times the full-success state with the run's own control step."""
    import sys
    import types

    import fingers

    T.score(_scene(pressed=SQUEEZE))
    objects = _scene(IN_TRAY)
    monkeypatch.setitem(sys.modules, "yam_arena.scene", types.SimpleNamespace(object_states=lambda env, n: objects))
    monkeypatch.setattr(T, "joint_enabled", lambda *a: True)
    monkeypatch.setattr(T, "set_joint", lambda *a: None)
    monkeypatch.setattr(fingers, "finger_points", lambda env: np.asarray([objects["battery"].pos + (0.0, 0.2, 0.0)]))
    env = types.SimpleNamespace(unwrapped=types.SimpleNamespace(step_dt=0.05))
    for k in range(HOLD_STEPS):
        assert T.score(objects) == 0.5, k
        T.work_the_lock(env)
    assert T.success(objects) and T.score(objects) == 1.0


def test_in_tray_without_a_squeeze_zero():
    """Can't happen with the lock, but the scorer asks for the squeeze before any credit."""
    _let_go()
    objects = _scene(IN_TRAY)
    assert T.score(objects) == 0.0 and not T.success(objects)


def test_in_tray_still_moving_half():
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    objects = _scene(_state(IN_TRAY, vel=(0.05, 0.0, 0.0)))
    assert not T.success(objects) and T.score(objects) == 0.5


def test_held_above_the_tray_half():
    T.score(_scene(pressed=SQUEEZE))
    assert T.score(_scene((*T.TRAY_XY, T.TRAY_FLOOR_Z + 0.10))) == 0.5
    _let_go()
    assert T.score(_scene((*T.TRAY_XY, T.TRAY_FLOOR_Z + 0.10))) == 0.5


def test_still_above_the_floor_is_not_full_credit():
    """The reviewers' exploit: a pack held still in the tray, off the floor, with or without the fingers on it."""
    T.score(_scene(pressed=SQUEEZE))
    for lift in (0.003, 0.01, 0.03, 0.06):
        for released in (False, True):
            _let_go(released)
            objects = _scene((*IN_TRAY[:2], IN_TRAY[2] + lift))
            assert objects["battery"].resting()
            assert not T.success(objects) and T.score(objects) == 0.5, (lift, released)


def test_in_tray_but_still_gripped_half():
    """Down on the floor in the tray, but a finger still touching the pack: it is not placed yet."""
    T.score(_scene(pressed=SQUEEZE))
    _let_go(False)
    objects = _scene(IN_TRAY)
    assert not T.success(objects) and T.score(objects) == 0.5
    _let_go(True)
    _hold(objects)
    assert T.success(objects) and T.score(objects) == 1.0


def test_no_finger_check_yet_is_not_released():
    """Before any step has run nothing counts as let go, so the offline default cannot score 1.0."""
    T.score(_scene(pressed=SQUEEZE))
    assert not T.STATE["released"]
    assert T.score(_scene(IN_TRAY)) == 0.5


def test_leaning_or_tilted_in_the_tray_half():
    """Corner on the floor and the pack leaning on a wall: not resting on the floor."""
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    tilt = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(0.4), -math.sin(0.4)], [0.0, math.sin(0.4), math.cos(0.4)]])
    q = _quat_from_matrix(tilt)
    pack = _state(IN_TRAY, q)
    z = T._pack_corners(pack)[:, 2].min()
    pack = _state((IN_TRAY[0], IN_TRAY[1], IN_TRAY[2] + (T.TRAY_FLOOR_Z - z)), q)
    objects = _scene(pack)
    assert abs(T._pack_corners(pack)[:, 2].min() - T.TRAY_FLOOR_Z) < 1e-9  # a corner on the floor
    assert not T.success(objects) and T.score(objects) == 0.5


def test_on_its_side_in_the_tray_succeeds():
    """A flat face down is resting whichever face it is: the pack on its 5 x 5.8 cm end."""
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    quat = _quat_from_matrix(np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]]))
    pack = _state((*T.TRAY_XY, T.TRAY_FLOOR_Z + T.BATTERY_SIZE[0] / 2), quat)
    assert T.success(_hold(_scene(pack)))


def test_on_a_wall_or_over_it_half():
    """Straddling or beyond the tray's walls, or in the open next to the tray: not inside them."""
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    wall_x = T.TRAY_XY[0] + T.TRAY_OUTER[0] / 2 - T.TRAY_WALL
    on_wall = _scene((wall_x - 0.02, T.TRAY_XY[1], T.TRAY_FLOOR_Z + T.BATTERY_SIZE[2] / 2))  # pack 5 cm along x
    assert not T.success(on_wall) and T.score(on_wall) == 0.5
    beside = _scene((T.TRAY_XY[0], T.TRAY_XY[1] + 0.12, T.TOP + T.BATTERY_SIZE[2] / 2))
    assert not T.success(beside) and T.score(beside) == 0.5
    edge = wall_x - T.BATTERY_SIZE[0] / 2  # the pack's far end just touching the inner face of the wall
    snug = _scene((edge - 0.0005, T.TRAY_XY[1], IN_TRAY[2]))
    assert T.success(_hold(snug))
    over = _scene((edge + 0.004, T.TRAY_XY[1], IN_TRAY[2]))
    assert not T.success(_hold(over))


def test_finger_gap():
    pack = _scene()["battery"]
    half_y = T.BATTERY_SIZE[1] / 2 + T.BUTTON_SIZE[1]
    on_button = pack.pos + (0.0, half_y, 0.0)
    inside = pack.pos + (0.01, 0.0, 0.0)
    near = pack.pos + (0.0, half_y + 0.0025, 0.0)
    clear = pack.pos + (0.0, half_y + 0.009, 0.0)
    objects = _scene()
    assert T.finger_gap(objects, [on_button]) == pytest.approx(0.0, abs=1e-9)
    assert T.finger_gap(objects, [inside]) == 0.0
    assert T.finger_gap(objects, [clear, near]) == pytest.approx(0.0025)
    assert T.finger_gap(objects, [clear]) == pytest.approx(0.009)
    assert T.finger_gap(objects, []) == math.inf and T.finger_gap(objects, None) == math.inf
    assert 0.0025 < T.RELEASED_GAP < 0.009


def test_step_hook_records_release(monkeypatch):
    """work_the_lock sets ``released`` from the finger points every step."""
    import sys
    import types

    import fingers

    objects = _scene()
    far = objects["battery"].pos + (0.0, 0.2, 0.0)
    touching = objects["battery"].pos + (0.0, T.BATTERY_SIZE[1] / 2 + T.BUTTON_SIZE[1], 0.0)
    # yam_arena.scene needs the simulator to import: stand in for it with the one function the hook uses
    monkeypatch.setitem(sys.modules, "yam_arena.scene", types.SimpleNamespace(object_states=lambda env, n: objects))
    monkeypatch.setattr(T, "joint_enabled", lambda *a: True)
    monkeypatch.setattr(T, "set_joint", lambda *a: None)
    for points, expected in (([far], True), ([far, touching], False), ([far], True)):
        monkeypatch.setattr(fingers, "finger_points", lambda env, points=points: np.asarray(points))
        T.work_the_lock(None)
        assert T.STATE["released"] is expected
    T.reset_memory()
    assert T.STATE["released"] is False


def test_in_tray_but_drill_moved_half():
    T.score(_scene(pressed=SQUEEZE))
    _let_go()
    moved = _scene(IN_TRAY, drill=DRILL0 + (0.0, -0.11, 0.0))
    assert not T.success(_hold(moved)) and T.score(moved) == 0.5
    kept = _hold(_scene(IN_TRAY, drill=DRILL0 + (0.0, -0.09, 0.0)))
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
