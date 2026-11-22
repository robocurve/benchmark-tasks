"""Offline checks of bar_clamp_boards.py: the layout, the clamp's mechanism (trigger, drive plate, brake, pad spring),
every scoring outcome, the 1 s hold, the finger check and the scripted demo. No GPU: run with
``python -m pytest -q test_bar_clamp_boards.py`` where ``yam_arena`` is installed."""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "bar_clamp_boards.py")
T = TASK.module
FLAT = (0.0, 0.0, 0.0, 1.0)
SPECS = {o.name: o for o in T.OBJECTS}


def _yaw(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rotz(v, a):
    c, s = math.cos(a), math.sin(a)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


def _state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _scene(o=None, travel=0.0, press=0.0, trigger=0.0, right=(0.0, 0.0), left=(0.0, 0.0), turn=0.0,
           left_turn=0.0, left_up=0.0):
    """The scene with the clamp's O at ``o`` (default: where it starts), turned ``turn`` about z, its bar moved
    ``travel`` through the handle (negative: closing), its pad pressed in ``press`` and its trigger squeezed
    ``trigger`` rad; the boards shifted by ``right`` / ``left`` (x, y) from where they start."""
    o = np.asarray(T.O_START if o is None else o, float)
    q = _yaw(turn)

    def at(p_from_o):
        return o + _rotz(np.asarray(p_from_o, float), turn)

    pivot = np.asarray(T.PIVOT)
    trig_c = pivot + _rotz(np.asarray(T.O_TO_TRIGGER) - pivot, trigger)
    objects = {
        "clamp": _state(at(T.O_TO_HANDLE), q),
        "pad": _state(at(np.asarray(T.PAD_CENTRE) + (0.0, -press, 0.0)), q),
        "bar": _state(at(np.asarray(T.O_TO_BAR) + (0.0, travel, 0.0)), q),
        "trigger": _state(at(trig_c), _qmul(q, _yaw(trigger))),
    }
    for name, (dx, dy), extra in (("board_right", right, 0.0), ("board_left", left, left_up)):
        p = np.asarray(SPECS[name].pos, float) + (dx, dy, extra)
        objects[name] = _state(p, _yaw(left_turn) if name == "board_left" else FLAT)
    return objects


def _clamped(press=0.001, **kw):
    """Clamped where the demo clamps: the pad pressed in ``press``, the right board pushed onto it, the left board
    onto the right one, the moving jaw on the left board's outer edge."""
    o = np.asarray(T.O_CLAMPED, float)
    face = o[1] - press  # the pad's face
    right_dy = face - (SPECS["board_right"].pos[1] - T.BOARD_SIZE[1] / 2)
    left_dy = right_dy - T.JOINT_GAP
    outer = face + 2 * T.BOARD_SIZE[1]
    travel = outer - (o[1] + T.OPEN)
    args = dict(o=o, travel=travel, press=press, right=(0.0, right_dy), left=(0.0, left_dy))
    args.update(kw)
    return _scene(**args)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def _run(objects, steps, fingers=None):
    """``steps`` control steps of the episode's memory with these objects (and these finger points); the score of
    the last one."""
    out = None
    for _ in range(steps):
        T.remember(objects, np.zeros((0, 3)) if fingers is None else fingers)
        out = (T.score(objects), T.success(objects))
    return out


# -- the task file and the layout ------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert {o.name for o in TASK.objects} >= {"clamp", "pad", "bar", "trigger", "board_right", "board_left"}
    assert TASK.demo is not None and TASK.success is not None and TASK.score is not None


def test_boards_lie_on_the_mat_edge_to_edge():
    r, l = SPECS["board_right"], SPECS["board_left"]
    assert l.pos[1] - r.pos[1] == pytest.approx(T.BOARD_SIZE[1] + T.JOINT_GAP)
    assert r.pos[2] - T.BOARD_SIZE[2] / 2 == pytest.approx(T.BASE + T.DROP)
    mat_x = (T.MAT_XY[0] - T.MAT_SIZE[1] / 2, T.MAT_XY[0] + T.MAT_SIZE[1] / 2)
    mat_y = (T.MAT_XY[1] - T.MAT_SIZE[0] / 2, T.MAT_XY[1] + T.MAT_SIZE[0] / 2)
    for b in (r, l):
        assert mat_x[0] < b.pos[0] - T.BOARD_SIZE[0] / 2 and b.pos[0] + T.BOARD_SIZE[0] / 2 < mat_x[1]
        assert mat_y[0] < b.pos[1] - T.BOARD_SIZE[1] / 2 and b.pos[1] + T.BOARD_SIZE[1] / 2 < mat_y[1]


def test_clamp_lies_flat_on_the_mat():
    for spec in (T.CLAMP, T.BAR_BODY):
        assert spec.pos[2] - spec.size[2] / 2 == pytest.approx(T.BASE + T.DROP)
    assert T.TRIGGER.pos[2] - T.TRIGGER.size[2] / 2 > T.BASE + 0.003  # the trigger rides clear of the mat
    assert T.GRIP[2][2] < T.CLAMP_T  # the rubber grip too


def test_clamp_starts_open_in_front_of_the_panel():
    """The jaws are open 14 cm (the clamp opens to 152 mm), wider than the 12.2 cm panel, their tips 2.2 cm in front
    of the boards' near ends, the fixed jaw 5 mm off the right board's edge and the moving jaw 13 mm off the left's."""
    s = _scene()
    assert T.OPEN <= 0.152 and T.OPEN > T.PANEL_W
    tip = T.O_START[0] + T.FIXED_JAW[1][0] + T.FIXED_JAW[2][0] / 2
    near = SPECS["board_right"].pos[0] - T.BOARD_SIZE[0] / 2
    assert near - tip == pytest.approx(0.022)
    fixed, moving = T.gaps(s)
    assert fixed == pytest.approx(0.005) and moving == pytest.approx(0.013)


def test_jaws_reach_the_boards_edges_once_slid_in():
    """Slid 8 cm forward, the pads lie beside the boards' edges, 1 cm or more inside their near ends, and the bar
    stays 4 mm in front of the boards."""
    s = _scene(o=np.asarray(T.O_START) + (T.SLIDE_IN, 0.0, 0.0))
    assert T.round_panel(s)
    bar_face = T.O_CLAMPED[0] + T.BAR[1][0] + T.BAR[2][0] / 2
    assert SPECS["board_right"].pos[0] - T.BOARD_SIZE[0] / 2 - bar_face == pytest.approx(T.BAR_CLEAR)


def test_throat_and_opening_are_the_mini_clamps():
    throat = T.FIXED_JAW[1][0] + T.FIXED_JAW[2][0] / 2 - (T.BAR[1][0] + T.BAR[2][0] / 2)
    assert throat == pytest.approx(0.062)  # 2-7/16 in
    assert T.MOVING_JAW[2][0] == T.FIXED_JAW[2][0]
    assert -T.BAR_BODY.lower + 0.002 == pytest.approx(T.OPEN)  # it closes to the pad (2 mm)


def test_bar_stays_in_the_housing():
    """The bar's rear end is inside or behind the housing at the start, and the moving jaw never reaches it."""
    rear = T.O_TO_BAR[1] - T.BAR_BODY.size[1] / 2
    assert rear <= T.HOUSING[1][1] + T.HOUSING[2][1] / 2


def test_masses_add_up():
    total = T.HANDLE_MASS + T.BAR_MASS + T.TRIGGER_MASS + T.PAD_MASS
    assert 0.25 <= total <= 0.4
    steel = 0.016 * 0.231 * 0.0048 * 7850
    assert steel == pytest.approx(0.139, abs=0.002) and T.BAR_MASS > steel


def test_board_mass_is_pine():
    assert T.BOARD_MASS == pytest.approx(np.prod(T.BOARD_SIZE) * 500, rel=0.01)


# -- the mechanism ---------------------------------------------------------------------------------------------------


def test_full_stroke_advances_the_bar_3_5_mm():
    assert T.DRIVE_ARM * math.sin(T.TRIGGER_MAX) == pytest.approx(0.0035, abs=0.0001)


def test_trigger_tip_stops_short_of_the_grip():
    """At the end of its stroke the trigger's tip (its back corner, 70 mm from the pivot) is still 1 mm or more in
    front of the grip."""
    corner = np.array([T.TRIGGER_LOW[0], T.TRIGGER_LOW[1], 0.0]) - T.PIVOT
    tip = np.asarray(T.PIVOT) + _rotz(corner, T.TRIGGER_MAX)
    grip_front = T.GRIP[1][1] + T.GRIP[2][1] / 2
    assert tip[1] - grip_front >= 0.001


def test_trigger_spring_holds_it_open_with_8_N_at_its_middle():
    preload = T.TRIGGER_SPRING * (0.0 - T.TRIGGER_REST)
    assert preload / 0.040 == pytest.approx(8.0, abs=0.1)
    full = T.TRIGGER_SPRING * (T.TRIGGER_MAX - T.TRIGGER_REST)
    assert full / 0.040 == pytest.approx(11.7, abs=0.2)


def test_squeezing_50_N_can_press_the_pad_but_a_light_touch_cannot():
    """A finger 36 mm from the pivot (the gripper pad's inner edge) pressing 50 N gives the bar well over 20 N; one
    pressing 9 N barely beats the trigger's spring."""
    def bar_force(finger):
        return (finger * 0.036 - T.TRIGGER_SPRING * (0.2 - T.TRIGGER_REST)) / (T.DRIVE_ARM * math.cos(0.2))
    assert bar_force(50.0) > 6 * T.FORCE_MIN
    assert bar_force(9.0) < 0


def test_drive_offset_follows_the_trigger():
    """The drive arm moves 9 mm x sin(angle) toward the fixed jaw as the trigger is squeezed."""
    assert T.drive_offset(_scene()) == pytest.approx(0.0, abs=1e-9)
    for a in (0.1, 0.25, T.TRIGGER_MAX):
        assert T.drive_offset(_scene(trigger=a)) == pytest.approx(-T.DRIVE_ARM * math.sin(a), abs=1e-4)
        assert T.trigger_angle(_scene(trigger=a)) == pytest.approx(a)


def test_drive_offset_is_measured_from_the_bar():
    """The bar moved shut by the drive keeps the offset where the drive left it: 0 against the stop."""
    a = T.TRIGGER_MAX
    s = _scene(trigger=a, travel=-T.DRIVE_ARM * math.sin(a))
    assert T.drive_offset(s) == pytest.approx(0.0, abs=1e-4)


def test_drive_plate_slips_back_with_the_trigger_but_grips_on_the_squeeze():
    """After a full squeeze (the bar 3.5 mm on, the drive at its stop, 0), the trigger springing open leaves the bar
    where it is: the drive plate slips back 3.5 mm along the bar and grips there for the next squeeze."""
    a = T.TRIGGER_MAX
    squeezed = _scene(travel=-T.DRIVE_ARM * math.sin(a), trigger=a)
    assert T.next_drive(squeezed, 0.0) == 0.0  # squeezed: the stop stays
    opened = _scene(travel=-T.DRIVE_ARM * math.sin(a), trigger=0.0)
    assert T.next_drive(opened, 0.0) == pytest.approx(T.DRIVE_ARM * math.sin(a), abs=1e-4)  # open: it follows
    half = _scene(travel=-T.DRIVE_ARM * math.sin(a), trigger=0.2)
    assert T.next_drive(half, T.DRIVE_ARM * math.sin(a)) == T.DRIVE_ARM * math.sin(a)  # squeezing again: it stays


def test_brake_follows_the_bar_shut_never_open():
    assert T.next_brake(_scene(travel=-0.01), 0.0) == pytest.approx(-0.01)
    assert T.next_brake(_scene(travel=-0.00005), 0.0) == 0.0  # under one tooth
    assert T.next_brake(_scene(travel=0.0), -0.01) == -0.01  # pulled open: the brake holds


def test_force_is_the_pads_compression_times_its_stiffness():
    assert T.clamping_force(_scene()) == pytest.approx(0.0, abs=1e-6)
    assert T.clamping_force(_scene(press=0.0005)) == pytest.approx(20.0)
    assert T.clamping_force(_scene(press=-0.001)) == 0.0  # pulled out: nothing pushes on it


def test_force_read_with_the_clamp_turned():
    s = _scene(turn=0.3, press=0.001)
    assert T.clamping_force(s) == pytest.approx(40.0)
    assert T.bar_travel(_scene(turn=0.3, travel=-0.02)) == pytest.approx(-0.02)


def test_pad_spring_fits_its_travel():
    assert T.PAD.lower == -T.PAD_TRAVEL and T.PAD_K * T.PAD_TRAVEL == pytest.approx(120.0)
    assert T.PAD_DAMPING == pytest.approx(2 * math.sqrt(T.PAD_K * T.PAD_MASS), rel=0.05)


def test_one_way_drive_is_registered_between_the_bar_and_the_trigger():
    assert (T.DRIVE.body0, T.DRIVE.body1, T.DRIVE.axis) == ("bar", "trigger", "Y")
    assert T.DRIVE.at == pytest.approx(tuple(a + b for a, b in zip(T.O_START, T.NOSE)))


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0 and not T.success(_scene())


def test_clamped_let_go_and_still_succeeds_after_one_second():
    s = _clamped()
    assert T.clamped(s) and T.aligned(s)
    assert _run(s, 10) == (0.75, False)  # not yet still for 0.5 s
    assert _run(s, 1) == (0.75, False)  # still: the second starts on this step
    assert _run(s, 19) == (0.75, False)  # 0.95 s
    assert _run(s, 1) == (1.0, True)  # 1.0 s


def test_moving_restarts_the_second():
    s = _clamped()
    _run(s, 25)
    nudged = _clamped(left=(0.001, -T.JOINT_GAP - 0.001))  # the left board 1 mm along: not still
    assert _run(nudged, 1) == (0.75, False)
    assert _run(s, 30) == (0.75, False)
    assert _run(s, 1) == (1.0, True)


@pytest.mark.parametrize("press,expected", [(0.0, 0.5), (0.0004, 0.5), (0.0005, 1.0), (0.002, 1.0)])
def test_minimum_force_20_N(press, expected):
    s = _clamped(press=press)
    assert _run(s, 40)[0] == expected


def test_finger_near_the_clamp_is_not_let_go():
    s = _clamped()
    near = np.asarray([T.point(s, "clamp", (T.GRIP[1][0], T.GRIP_BACK - 0.02, 0.0))])
    assert T.finger_distance(s, near) == pytest.approx(0.02)
    assert _run(s, 40, near) == (0.75, False)
    far = np.asarray([T.point(s, "clamp", (T.GRIP[1][0], T.GRIP_BACK - 0.031, 0.0))])
    assert _run(s, 21, far) == (1.0, True)


def test_finger_on_a_board_is_not_let_go():
    """A finger pressing a board (pushing the boards onto the pad instead of the clamp doing it) does not count."""
    s = _clamped()
    on_board = np.asarray([SPECS["board_left"].pos]) + (0.05, 0.0, T.BOARD_SIZE[2] / 2 + 0.001)
    assert T.finger_distance(s, on_board) < T.CLEAR
    assert _run(s, 40, on_board) == (0.75, False)


def test_finger_on_the_trigger_is_not_let_go():
    s = _clamped()
    on_trigger = np.asarray([T.point(s, "trigger", (-0.09, 0.006, 0.0))])
    assert T.finger_distance(s, on_trigger) < 0.002
    assert _run(s, 40, on_trigger) == (0.75, False)


def test_joint_open_is_not_closed():
    """The left board 1 mm off the right one, the moving jaw on it: the joint is open: 0.25."""
    s = _clamped()
    open_joint = _clamped(left=(0.0, s["board_left"].pos[1] - SPECS["board_left"].pos[1] + 0.001),
                          travel=T.bar_travel(s) + 0.001)
    assert T.joint_gap(open_joint) == pytest.approx(0.001)
    assert _run(open_joint, 40)[0] == 0.25


def test_joint_open_at_its_far_end():
    s = _clamped(left_turn=0.006)  # the left board turned: its far end 0.6 mm off
    assert T.joint_gap(s) > T.JOINT_SHUT
    assert not T.clamped(s)


def test_moving_jaw_off_the_board_half():
    s = _clamped(press=0.0)
    off = _clamped(press=0.0, travel=T.bar_travel(s) + 0.001)  # the moving jaw 1 mm off the left board
    assert T.gaps(off)[1] == pytest.approx(0.001)
    assert _run(off, 40)[0] == 0.25


def test_pressed_pad_with_the_moving_jaw_open_does_not_score_force():
    """The pad pressed in (by a hand pushing the boards, say) while the moving jaw is away: not clamped."""
    s = _clamped(press=0.002)
    off = _clamped(press=0.002, travel=T.bar_travel(s) + 0.002)
    assert not T.clamped(off) and T.score(off) == 0.25


def test_misaligned_boards_three_quarters():
    s = _clamped()
    shifted = _clamped(left=(0.003, s["board_left"].pos[1] - SPECS["board_left"].pos[1]))
    assert T.misalignment(shifted) == pytest.approx(0.003)
    assert T.clamped(shifted) and not T.aligned(shifted)
    assert _run(shifted, 40)[0] == 0.75


def test_board_ridden_up_three_quarters():
    s = _clamped()
    up = _clamped(left=(0.0, s["board_left"].pos[1] - SPECS["board_left"].pos[1]), left_up=0.0015)
    assert T.step_height(up) == pytest.approx(0.0015)
    assert _run(up, 40)[0] == 0.75


def test_round_the_panel_open_quarter():
    s = _scene(o=np.asarray(T.O_START) + (T.SLIDE_IN, 0.0, 0.0))
    assert T.score(s) == 0.25


def test_pads_off_the_boards_ends_zero():
    """Slid only 6 cm: the pad centres lie under 1 cm inside the boards' ends."""
    s = _scene(o=np.asarray(T.O_START) + (0.05, 0.0, 0.0))
    assert not T.round_panel(s) and T.score(s) == 0.0


def test_closed_beside_the_panel_zero():
    """The clamp closed on nothing beside the panel."""
    s = _scene(o=np.asarray(T.O_START) + (0.0, -0.2, 0.0), travel=-0.12)
    assert T.score(s) == 0.0


def test_jaws_inside_the_boards_zero():
    s = _clamped(travel=-0.1)
    assert T.gaps(s)[1] < T.GAP_FLOOR and T.score(s) == 0.0


@pytest.mark.parametrize("body", ["board_right", "board_left", "clamp", "bar", "trigger", "pad"])
def test_fallen_zero(body):
    s = _clamped()
    st = s[body]
    s[body] = _state(st.pos - (0.0, 0.0, 0.3), st.quat_xyzw)
    assert T.score(s) == 0.0


def test_non_finite_zero():
    s = _clamped()
    s["bar"] = _state((math.nan, 0.0, 0.0))
    assert T.score(s) == 0.0 and math.isinf(T.distance(s))


def test_distance_falls_as_the_clamp_closes():
    d0 = T.distance(_scene())
    d1 = T.distance(_scene(o=np.asarray(T.O_START) + (T.SLIDE_IN, 0.0, 0.0)))
    d2 = T.distance(_clamped(press=0.0))
    d3 = T.distance(_clamped(press=0.001))
    assert d0 > d1 > d2 > d3


def test_no_full_score_while_any_condition_is_missing():
    """With every other full-success condition held, 1.0 is not reached while one is missing."""
    cases = {
        "force": _clamped(press=0.0004),
        "joint": _clamped(left_turn=0.006),
        "aligned": _clamped(left=(0.004, _clamped()["board_left"].pos[1] - SPECS["board_left"].pos[1])),
    }
    for name, s in cases.items():
        T.reset_memory()
        assert _run(s, 60)[0] < 1.0, name
    T.reset_memory()
    s = _clamped()
    near = np.asarray([T.point(s, "trigger", (-0.09, 0.006, 0.0))])
    assert _run(s, 60, near)[0] < 1.0
    T.reset_memory()
    a, b = _clamped(), _clamped(left=(0.0006, _clamped()["board_left"].pos[1] - SPECS["board_left"].pos[1]))
    for k in range(60):
        T.remember(a if k % 2 else b, np.zeros((0, 3)))
        assert T.score(a if k % 2 else b) < 1.0


# -- the fingers -----------------------------------------------------------------------------------------------------


def test_finger_cloud_from_the_urdf():
    import fingers

    for tip in ("tip_left", "tip_right"):
        c = fingers.cloud(tip)
        assert c.shape[1] == 3 and len(c) > 50


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(plan, label):
    t = 0.0
    for w in plan:
        if w.label == label:
            return t + w.duration, w
        t += w.duration
    raise KeyError(label)


def test_demo_works_the_clamp_with_the_right_arm_and_films_with_the_left():
    """Only the right arm touches anything; the left arm holds its wrist camera over the clamp, its fingertips 3 cm or
    more above the panel and the clamp, and well away from the right arm's grip."""
    d = T.demo()
    assert set(d) == {"right", "left"}
    for w in d["left"]:
        x, y, z = w.pos
        assert z - 0.013 > T.BASE + max(T.BOARD_SIZE[2], T.CLAMP_T) + T.CLEAR
        assert y > T.O_CLAMPED[1] + T.GRIP[1][1] + 0.10


def test_demo_slides_the_clamp_to_where_it_clamps():
    plan = T.demo()["right"]
    _, w0 = _at(plan, "squeeze: hold the clamp")
    _, w1 = _at(plan, "slide the jaws round the panel")
    assert w1.pos[0] - w0.pos[0] == pytest.approx(T.SLIDE_IN)
    assert w0.pos[0] - T.TCP_X == pytest.approx(T.O_START[0])


def test_demo_fingers_go_round_the_grip_and_trigger():
    """The back finger stays on the grip's back (the TCP moves half the opening); open, the front finger clears the
    open trigger's front; squeezed, it is commanded past the trigger's full stroke (the trigger's stop holds it), the
    squeeze within the gripper's 94 mm."""
    full_front = 0.005 - 0.050 * math.sin(T.TRIGGER_MAX) + 0.006 * (math.cos(T.TRIGGER_MAX) - 1)
    assert abs(T.BACK_Y - T.GRIP_BACK) <= 0.001
    assert T._tcp_y(T.OPEN_W) - T.OPEN_W / 2 == pytest.approx(T.BACK_Y)
    assert T._tcp_y(T.SQUEEZE_W) - T.SQUEEZE_W / 2 == pytest.approx(T.BACK_Y)
    assert T.BACK_Y + T.OPEN_W > 0.005 + 0.015
    assert T.BACK_Y + T.SQUEEZE_W < full_front < T.BACK_Y + T.OPEN_W < 0.094 + T.BACK_Y
    assert T.TRIGGER_LOW[0] < T.TCP_X - 0.02 + 0.001 and T.TCP_X + 0.02 < T.PIVOT[0] - 0.025


def test_demo_lets_the_trigger_spring_fully_open_between_squeezes():
    plan = T.demo()["right"]
    oy = T.O_START[1]
    for w in plan:
        if w.label.startswith("let the trigger open"):
            assert w.pos[1] - oy == pytest.approx(T._tcp_y(T.OPEN_W))
        if w.label.startswith("squeeze ("):
            assert w.pos[1] - oy == pytest.approx(T._tcp_y(T.SQUEEZE_W))


def test_push_by_hand_check_never_touches_the_trigger(monkeypatch):
    """The check that the trigger is needed: after the slide the right arm lets go and the left arm pushes the moving
    jaw shut, 25 mm past the panel; both arms reach their waypoints."""
    monkeypatch.setenv("BAR_CLAMP_DEMO", "by_hand")
    d = T.demo()
    labels = [w.label for w in d["right"]]
    assert not any(lb.startswith("squeeze (") for lb in labels) and labels[-2] == "up"
    push = [w for w in d["left"] if w.label == "push the jaw shut"][0]
    jaw_face = push.pos[1] - T.FINGER_T - T.MOVING_JAW[2][1]  # the moving jaw's pad face, pushed there
    assert jaw_face == pytest.approx(T.O_CLAMPED[1] + T.PANEL_W - T.PUSH_PAST)
    for arm, plan in d.items():
        p = ArmPlan(plan, BASES[arm])
        t = 0.0
        while t < p.duration:
            p.action7(t)
            t += 0.05
        assert p.worst_ik_mm < 1.0 and p.worst_ik_deg < 1.0


def test_demo_pumps_full_strokes():
    """The squeeze that holds the clamp for the slide and every stroke after it are full strokes (3.5 mm of bar each):
    together they can drive the bar 14 mm or more, the moving jaw's 13 mm to the left board plus the joint's 2 mm, less
    the few millimetres the clamp's handle slides toward the right board as the back finger takes hold of it. Isaac
    runs of the demo close the joint and press 60 N or more on the second stroke after the slide."""
    assert T.STROKES >= 2
    assert (T.STROKES + 1) * 0.0035 >= 0.013 + T.JOINT_GAP - 0.005


def test_demo_order_and_length():
    plan = T.demo()["right"]
    labels = [w.label for w in plan]
    assert labels.index("slide the jaws round the panel") < labels.index("squeeze (1)") < labels.index("let go")
    total = sum(w.duration for w in plan)
    assert total * 20 + 40 < 1200  # within the run limit, with the 1 s hold and the settle


def test_demo_reachable():
    for arm, plan in T.demo().items():
        p = ArmPlan(plan, BASES[arm])
        t = 0.0
        while t < p.duration:
            p.action7(t)
            t += 0.05
        assert p.worst_ik_mm < 1.0 and p.worst_ik_deg < 1.0


def test_dressed_meshes_exist():
    for name, file in T.LOOKS.items():
        assert name in SPECS and (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["board_right", "board_left", "mat"])
def test_scored_meshes_fit_in_their_boxes(name):
    if name not in T.LOOKS:
        pytest.skip("no meshes")
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(
        stage.GetPseudoRoot()).ComputeAlignedRange()
    size = SPECS[name].size
    for lo, hi, s in zip(box.GetMin(), box.GetMax(), size):
        assert -s / 2 - 1e-4 <= lo and hi <= s / 2 + 1e-4
