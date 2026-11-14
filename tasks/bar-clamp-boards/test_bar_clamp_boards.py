"""Offline checks (no GPU): the task file loads, the layout leaves the clamp room to slide under the stack, the scorer
covers each outcome (clamped and let go, still held, off the middle third, boards misaligned, a jaw open, beside the
stack, fallen, blown up), the ratchet's travel and the finger check measure what they claim, and the demo puts the
clamp where it scores, keeps the arms apart and stays within reach. Run in the yam_arena environment:
python -m pytest -q test_bar_clamp_boards.py"""

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

FLAT = np.array([0.0, 0.0, 0.0, 1.0])
X, Y = T.STACK_XY
O_CLAMPED = np.array([X, T.BAR_Y_CLAMPED, T.UNDER])  # the clamp's O with its fixed jaw on the bottom board


def _state(pos, quat=FLAT, vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.02, high=pos + 0.02)


def _yaw(a):
    return np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)])


PRESS = 0.001  # m: the fixed jaw's pad pressed in 1 mm: 40 N


def _scene(shift=(0.0, 0.0, 0.0), press=PRESS, opening=None, top_shift=(0.0, 0.0), vel=(0.0, 0.0, 0.0), o=None):
    """Boards on the spacers, the clamp's O ``shift`` from where its pad face meets the bottom board with the pad
    pressed in ``press`` (m), and the sliding jaw's face ``opening`` above the O's height (by default on the top
    board's face)."""
    if o is None:
        o = O_CLAMPED + (0.0, 0.0, press) + np.asarray(shift, float)
    o = np.asarray(o, float)
    if opening is None:
        opening = T.UNDER + T.STACK_T - o[2]
    jaw = o + np.asarray(T.O_TO_JAW) + (0.0, 0.0, opening - T.JAW_OPEN)
    pad = o + np.asarray(T.PAD_CENTRE) - (0.0, 0.0, press)
    return {
        "board_bottom": _state((X, Y, T.UNDER + T.BOARD_SIZE[2] / 2)),
        "board_top": _state((X + top_shift[0], Y + top_shift[1], T.UNDER + 1.5 * T.BOARD_SIZE[2])),
        "clamp": _state(o + np.asarray(T.O_TO_CENTRE), vel=vel),
        "sliding_jaw": _state(jaw, vel=vel),
        "fixed_pad": _state(pad, vel=vel),
    }


def _start():
    return _scene(o=np.asarray(T.O_START), opening=T.JAW_OPEN, press=0.0)


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


def _let_go(clear=True):
    T.STATE["clear"] = clear
    T.STATE["still"] = True


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"board_bottom", "board_top", "clamp", "sliding_jaw", "fixed_pad"}


def test_spacers_under_the_ends_leave_the_middle_free():
    near, far = T.SPACERS
    gap = (far.pos[0] - far.size[0] / 2) - (near.pos[0] + near.size[0] / 2)
    assert gap > T.FIXED_JAW[2][0] + 0.05  # the fixed jaw passes between them with 2.5 cm or more each side
    for s in T.SPACERS:
        assert abs(s.pos[0] - X) + s.size[0] / 2 <= T.BOARD_SIZE[0] / 2  # under the boards


def test_clamp_fits_round_the_stack():
    """The fixed jaw clears the underside by 1 cm while the clamp stands, and the sliding jaw clears the top."""
    assert T.UNDER - T.O_START[2] > 0.009
    assert T.O_START[2] + T.JAW_OPEN - T.STACK_TOP > 0.01
    assert T.JAW_OPEN > T.STACK_T + 0.015
    tips = T.O_START[1] + T.CLAMP_LOW[1]
    assert tips - (Y + T.BOARD_SIZE[1] / 2) > 0.015  # the jaws start off the boards' edge


def test_start_scores_zero():
    s = _start()
    assert T.score(s) == 0.0 and not T.straddles(s)


def test_start_jaw_travel_zero():
    assert T.jaw_travel(_start()) == pytest.approx(0.0, abs=1e-9)
    assert T.jaw_travel(_scene(press=0.0)) == pytest.approx(T.STACK_T - T.JAW_OPEN, abs=1e-9)


# -- the scoring ladder ------------------------------------------------------------------------------------------------


def test_clamped_and_let_go_succeeds():
    s = _scene()
    _let_go()
    assert T.gaps(s) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert T.clamping_force(s) == pytest.approx(40.0)
    assert T.success(s) and T.score(s) == 1.0


def test_force_is_the_pads_compression_times_its_stiffness():
    for press in (0.0, 0.0005, 0.001, 0.0025):
        assert T.clamping_force(_scene(press=press)) == pytest.approx(T.PAD_K * press)
    assert T.pad_compression(_start()) == pytest.approx(0.0, abs=1e-9)


def test_pad_turned_with_the_clamp_still_measured():
    """The compression is along the clamp's own up axis: a clamp tilted 3 degrees reads the same."""
    s = _scene(press=0.001)
    a = math.radians(3.0)
    q = np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])
    c = np.asarray(s["clamp"].pos)
    for name in ("clamp", "fixed_pad"):
        rel = np.asarray(s[name].pos) - c
        rot = T._rot(q) @ rel
        s[name] = _state(c + rot, q)
    assert T.clamping_force(s) == pytest.approx(40.0)


@pytest.mark.parametrize("press,expected", [(0.0, 0.25), (0.0002, 0.25), (0.00049, 0.25), (0.00051, 1.0), (0.001, 1.0),
                                            (0.0025, 1.0)])
def test_minimum_force_20_N(press, expected):
    """Below 20 N (0.5 mm of the pad's travel) the clamp is round the stack but does not grip it."""
    _let_go()
    assert T.score(_scene(press=press)) == expected


def test_clamp_hanging_with_the_lower_pad_just_clear_does_not_score():
    """The reviewers' exploit: the clamp hangs from the top board, its lower pad 0.9 mm clear of the bottom board. The
    old gap-under-1-mm rule scored it 1.0. The pad is not pressed in, so no force: 0.25."""
    _let_go()
    s = _scene(shift=(0.0, 0.0, -0.0009), press=0.0)
    assert T.gaps(s)[0] == pytest.approx(0.0009)
    assert T.clamping_force(s) == 0.0 and T.hanging(s)
    assert T.score(s) == 0.25 and not T.success(s)


def test_clamp_just_touching_without_force_does_not_score():
    """Both pads on their boards, nothing pressing: a clamp that sits on the stack, not one that clamps it."""
    _let_go()
    s = _scene(press=0.0)
    assert T.gaps(s) == pytest.approx((0.0, 0.0), abs=1e-9)
    assert T.score(s) == 0.25


def test_pressed_pad_with_the_sliding_jaw_open_does_not_score():
    """The pad is pressed in, but the sliding jaw is 1 mm above the top board: the stack is not squeezed between the
    jaws (the force would lift it)."""
    _let_go()
    s = _scene(opening=T.STACK_T - PRESS + 0.001)
    assert T.clamping_force(s) == pytest.approx(40.0) and T.gaps(s)[1] == pytest.approx(0.001)
    assert T.score(s) == 0.25


def test_pressed_pad_off_the_bottom_board_does_not_score():
    """A pad compression with the pad's face 1 cm under the bottom board is not a contact."""
    _let_go()
    s = _scene(shift=(0.0, 0.0, -0.01), opening=T.STACK_T + 0.01 - PRESS, press=PRESS)
    assert T.gaps(s)[0] == pytest.approx(0.01 + 0.0, abs=1e-9) and T.score(s) == 0.25


def test_standing_on_the_mat_half():
    """Jaws closed on the stack, but the clamp tilted so its handle's end rests on the mat: not hanging."""
    s = _scene()
    a = math.radians(-12.0)  # the handle down
    q = np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)])
    s["clamp"] = _state(s["clamp"].pos, q)
    _let_go()
    assert not T.hanging(s) and T.score(s) <= 0.5


def test_hanging_clamp_is_off_the_mat():
    assert T.hanging(_scene())
    assert not T.hanging(_start())  # standing on its foot


def test_still_held_half():
    _let_go(False)
    s = _scene()
    assert not T.success(s) and T.score(s) == 0.5


def test_still_moving_half():
    """Not yet settled (the clamp or a board moved more than 0.5 mm in the last 0.5 s): 0.5."""
    _let_go()
    T.STATE["still"] = False
    assert not T.success(_scene()) and T.score(_scene()) == 0.5


def _history(poses):
    """The poses of successive control steps: each a (pos, quat) for the clamp, the bottom board and the top board."""
    return [tuple((np.array(p, float), np.array(q, float)) for p, q in step) for step in poses]


def _still_pose(dx=0.0, turn=0.0):
    q = (0.0, 0.0, math.sin(turn / 2), math.cos(turn / 2))
    return ((( dx, 0.0, 1.0), q), ((0.0, 0.0, 0.5), (0, 0, 0, 1)), ((0.0, 0.0, 0.52), (0, 0, 0, 1)))


def test_settled_means_not_moved_in_half_a_second():
    n = T.STILL_STEPS
    assert T.settled(_history([_still_pose()] * (n + 1)))
    assert not T.settled(_history([_still_pose()] * n))  # not 0.5 s of history yet
    assert not T.settled(_history([_still_pose(0.0)] * n + [_still_pose(0.001)]))  # 1 mm in 0.5 s
    assert T.settled(_history([_still_pose(0.0)] * n + [_still_pose(0.0004)]))  # 0.4 mm
    assert not T.settled(_history([_still_pose(0.0)] * n + [_still_pose(0.0, turn=0.02)]))  # turned 0.02 rad


def test_a_stack_reporting_a_residual_velocity_but_not_moving_is_still():
    """A clamp pressing the stack with tens of newtons reports about 1.9 cm/s from the solver while staying put (seen
    in the simulator). It counts as at rest: what is measured is where it is."""
    s = _scene(vel=(0.019, 0.0, 0.0))
    _let_go()
    assert T.score(s) == 1.0


@pytest.mark.parametrize("dx,expected", [(0.03, 1.0), (-0.03, 1.0), (0.04, 0.5), (-0.045, 0.5)])
def test_middle_third(dx, expected):
    _let_go()
    assert T.score(_scene(shift=(dx, 0.0, 0.0))) == expected


@pytest.mark.parametrize("shift,expected", [((0.002, 0.0), 1.0), ((0.0, 0.0025), 1.0), ((0.004, 0.0), 0.5),
                                            ((0.0, -0.0035), 0.5)])
def test_boards_aligned_within_3_mm(shift, expected):
    _let_go()
    assert T.score(_scene(top_shift=shift)) == expected


def test_turned_top_board_is_misaligned():
    s = _scene()
    s["board_top"] = _state(s["board_top"].pos, _yaw(math.radians(2.0)))  # corners 3.5 mm off
    _let_go()
    assert T.misalignment(s) > 0.003 and T.score(s) == 0.5


def test_sliding_jaw_open_quarter():
    _let_go()
    s = _scene(opening=T.STACK_T - PRESS + 0.0015)
    assert T.straddles(s) and not T.clamped(s) and T.score(s) == 0.25


def test_sliding_jaw_just_touching_counts():
    _let_go()
    s = _scene(opening=T.STACK_T - PRESS + 0.0004)
    assert T.clamped(s) and T.score(s) == 1.0
    s = _scene(opening=T.STACK_T - PRESS + 0.0006)  # 0.6 mm: off the board
    assert not T.clamped(s) and T.score(s) == 0.25


def test_jaws_wide_open_round_the_stack_quarter():
    _let_go()
    s = _scene(shift=(0.0, 0.0, -0.01), opening=T.JAW_OPEN, press=0.0)
    assert T.score(s) == 0.25


def test_pads_barely_on_the_boards_zero():
    """The clamp pulled back so the pads are less than 1 cm inside the boards' edge."""
    _let_go()
    s = _scene(shift=(0.0, 0.040, 0.0))
    assert not T.straddles(s) and T.score(s) == 0.0


def test_beside_the_stack_closed_zero():
    _let_go()
    s = _scene(o=np.asarray(T.O_START), opening=0.01, press=0.0)
    assert T.score(s) == 0.0


def test_jaws_in_the_boards_zero():
    _let_go()
    s = _scene(opening=T.STACK_T - 0.01)
    assert T.score(s) == 0.0


def test_pad_ends_inside_the_boards():
    """Both pad ends lie over the boards, 2.2 cm and more inside their edge, when the clamp is where it clamps."""
    s = _scene()
    for y in T.PAD_ENDS:
        assert T.on_board(s, "board_bottom", T.fixed_pad(s, y)) and T.on_board(s, "board_top", T.sliding_pad(s, y))


def test_tilted_clamp_touches_at_one_end():
    """A clamp tilted 1.5 degrees, its pads' centres 0.9 mm off the boards: one end of each pad touches."""
    s = _scene(press=0.0)
    a = math.radians(1.5)
    q = np.array([math.sin(-a / 2), 0.0, 0.0, math.cos(-a / 2)])  # the jaws' tips up
    for name in ("clamp", "sliding_jaw", "fixed_pad"):
        s[name] = _state(s[name].pos, q)
    f, g = T.gaps(s)
    assert f < 0.001 and g < 0.001


def test_clamp_turned_round_the_stack_still_measured():
    """A clamp tilted 1 degree about x still reads its gaps along the boards' normal."""
    s = _scene()
    tilt = np.array([math.sin(math.radians(0.5)), 0.0, 0.0, math.cos(math.radians(0.5))])
    for name in ("clamp", "sliding_jaw", "fixed_pad"):
        s[name] = _state(s[name].pos, tilt)
    f, g = T.gaps(s)
    assert abs(f) < 0.002 and abs(g) < 0.002


def test_fallen_zero():
    _let_go()
    s = _scene()
    s["board_top"] = _state((0.3, 0.0, T.TOP - 0.3))
    assert T.score(s) == 0.0
    s = _scene()
    s["clamp"] = _state((0.3, 0.0, T.TOP - 0.3))
    assert T.score(s) == 0.0
    s = _scene()
    s["fixed_pad"] = _state((0.3, 0.0, T.TOP - 0.3))
    assert T.score(s) == 0.0


def test_non_finite_zero():
    _let_go()
    s = _scene()
    s["sliding_jaw"] = _state((np.nan, 0.0, 0.0))
    assert T.score(s) == 0.0 and not T.success(s)
    assert T.distance(s) == math.inf


def test_distance_falls_as_the_clamp_closes():
    assert (T.distance(_start()) > T.distance(_scene(opening=T.JAW_OPEN, press=0.0))
            > T.distance(_scene(press=0.0)) > T.distance(_scene()))


def test_pad_spring_fits_its_travel():
    """The pad's 3 mm of travel reaches past 20 N and tops out at 120 N, and the jaw's own face is 3 mm under the
    pad's rest face, so the stack bottoms out on it at the end of the travel."""
    assert T.FORCE_MIN / T.PAD_K < T.PAD_TRAVEL and T.PAD_K * T.PAD_TRAVEL == pytest.approx(120.0)
    face = T.FIXED_JAW[1][2] + T.FIXED_JAW[2][2] / 2  # the jaw's top face, from O
    assert face == pytest.approx(-T.PAD_TRAVEL)
    assert T.PAD_CENTRE[2] + T.PAD_SIZE[2] / 2 == pytest.approx(0.0)


# -- the trigger -------------------------------------------------------------------------------------------------------


def test_stop_follows_the_jaw_down_never_up():
    s = _scene(press=0.0, opening=0.05)  # the jaw 2 mm... any travel
    travel = T.jaw_travel(s)
    assert T.next_stop(s, False, 0.0) == pytest.approx(travel)  # pushed past the stop: the stop follows
    assert T.next_stop(s, False, travel - 0.01) == pytest.approx(travel - 0.01)  # never moves back up
    assert T.next_stop(s, False, travel + T.TOOTH / 2) == pytest.approx(travel + T.TOOTH / 2)  # within a click: stays


def test_trigger_squeezes_only_when_the_jaw_is_on_the_stack_and_pressed():
    s = _scene(press=0.0)
    travel = T.jaw_travel(s)
    assert T.trigger_ready(s)
    assert T.next_stop(s, True, travel) == pytest.approx(travel - T.TAKEUP_STEP)  # the pad is not pressed in yet
    assert T.next_stop(s, False, travel) == pytest.approx(travel)  # nobody pressing
    open_jaw = _scene(press=0.0, opening=T.STACK_T + 0.001)  # the sliding jaw 1 mm off the top board
    assert not T.trigger_ready(open_jaw)
    assert T.next_stop(open_jaw, True, T.jaw_travel(open_jaw)) == pytest.approx(T.jaw_travel(open_jaw))
    beside = _scene(shift=(0.0, 0.040, 0.0), press=0.0)  # the clamp not round the stack
    assert T.next_stop(beside, True, T.jaw_travel(beside)) == pytest.approx(T.jaw_travel(beside))


def test_trigger_takes_up_slack_fast_then_builds_force_slowly():
    pressed = _scene(press=0.0005)  # pad pressed in 0.5 mm
    travel = T.jaw_travel(pressed)
    assert T.next_stop(pressed, True, travel) == pytest.approx(travel - T.SQUEEZE_STEP)
    slack = _scene(shift=(0.0, 0.0, -0.005), press=0.0)  # the pad 5 mm under the bottom board, the jaw on the top
    assert T.trigger_ready(slack)
    assert T.next_stop(slack, True, T.jaw_travel(slack)) == pytest.approx(T.jaw_travel(slack) - T.TAKEUP_STEP)


def test_trigger_stops_at_the_pads_travel():
    s = _scene(press=T.SQUEEZE_MAX + 0.0001)
    travel = T.jaw_travel(s)
    assert T.next_stop(s, True, travel) == pytest.approx(travel)


def test_squeeze_reaches_the_minimum_force_in_ten_steps():
    """Each squeezed step presses the stack 0.05 mm more (2 N): 20 N takes 10 steps, half a second."""
    assert T.FORCE_MIN / (T.PAD_K * T.SQUEEZE_STEP) == pytest.approx(10.0)
    assert T.PAD_K * T.SQUEEZE_MAX == pytest.approx(100.0)


def test_finger_pressing_the_jaw_only():
    s = _scene()
    top = np.asarray(s["sliding_jaw"].pos) + (0.0, 0.0, T.JAW_SIZE[2] / 2 + 0.002)
    assert T.finger_distance(s, [top], jaw_only=True) == pytest.approx(0.002)
    on_handle = O_CLAMPED + (0.015, T.GRIP_Y, PRESS)
    assert T.finger_distance(s, [on_handle], jaw_only=True) > T.PRESS_DIST  # a grip on the handle does not press


# -- the fingers -------------------------------------------------------------------------------------------------------


def test_finger_on_the_handle_holds():
    s = _scene()
    grip = O_CLAMPED + (0.015, T.GRIP_Y, PRESS)  # on the handle's side
    assert T.finger_distance(s, [grip]) == pytest.approx(0.0, abs=1e-9)


def test_finger_on_the_sliding_jaw_holds():
    s = _scene()
    top = np.asarray(s["sliding_jaw"].pos) + (0.0, 0.0, T.JAW_SIZE[2] / 2 + 0.003)
    assert T.finger_distance(s, [top]) == pytest.approx(0.003, abs=1e-9)


def test_finger_above_the_bar_is_clear():
    s = _scene()
    above = O_CLAMPED + (0.0, 0.0, PRESS + T.CLAMP_HIGH[2] + 0.02)
    assert T.finger_distance(s, [above]) == pytest.approx(0.02, abs=1e-9)
    assert T.finger_distance(s, []) == math.inf


def test_finger_cloud_from_the_urdf():
    import fingers

    for tip in fingers.TIPS:
        pts = fingers.cloud(tip)
        assert 200 < len(pts) < 20000
        assert 0.05 < (pts.max(0) - pts.min(0)).max() < 0.15


# -- the demo ----------------------------------------------------------------------------------------------------------


def _at(plan, label):
    t = 0.0
    for w in plan:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_slides_the_clamp_to_where_it_clamps():
    plan = T.demo()
    _, grip = _at(plan["left"], "grip")
    _, slid = _at(plan["left"], "slide the fixed jaw under the stack")
    assert grip.pos[1] - slid.pos[1] == pytest.approx(T.SLIDE)
    assert slid.pos[1] - T.GRIP_Y == pytest.approx(T.BAR_Y_CLAMPED)
    assert slid.pos[0] == pytest.approx(X + T.GRIP_DX)
    _, lift = _at(plan["left"], "lift against the bottom board")
    assert lift.pos[2] - slid.pos[2] > T.UNDER - T.O_START[2]  # lifted past the 1 cm to the board


def test_demo_pushes_past_the_stack_top():
    plan = T.demo()
    _, push = _at(plan["right"], "push the jaw down")
    assert push.pos[2] - T.PAD_TIP_PAST_TCP < T.STACK_TOP + T.JAW_SIZE[2]
    assert T.JAW_LOW[1] + 0.008 <= T.PUSH_FROM_BAR <= T.PAD_Y  # over the jaw's arm, the fingers on it
    assert push.pos[1] == pytest.approx(T.BAR_Y_CLAMPED + T.PUSH_FROM_BAR)


def test_demo_order():
    """The left arm lifts the clamp before the right arm pushes, and lets go while the right arm still holds."""
    plan = T.demo()
    lifted, _ = _at(plan["left"], "lift against the bottom board")
    let_go, _ = _at(plan["left"], "let go")
    down, _ = _at(plan["right"], "down to the jaw")
    pushed, _ = _at(plan["right"], "push the jaw down")
    held, _ = _at(plan["right"], "press the trigger, hold while the left arm lets go")
    assert lifted <= down - 0.8 and pushed < let_go - 1.0 and held >= let_go + 1.0


def test_demo_presses_the_trigger_long_enough_to_reach_the_minimum_force():
    """The right arm's fingers stay on the sliding jaw after the jaw is home: 10 steps (0.5 s) of squeeze give 20 N
    after the pad is up, and the slack takes about 1.5 s before that."""
    plan = T.demo()
    _, held = _at(plan["right"], "press the trigger, hold while the left arm lets go")
    assert held.duration >= 4.0 and T.PRESS_HOLD == held.duration


def test_demo_keeps_the_arms_apart():
    """Each arm in its own half, the grip and the push 14 cm or more apart."""
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < -0.05
    assert min(w.pos[1] for w in plan["left"]) > 0.02
    _, slid = _at(plan["left"], "slide the fixed jaw under the stack")
    _, push = _at(plan["right"], "push the jaw down")
    assert np.linalg.norm(np.asarray(slid.pos[:2]) - np.asarray(push.pos[:2])) > 0.14


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    assert T.LOOKS, "run looks/make_looks.py in Blender"
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["board_bottom", "board_top", "clamp", "sliding_jaw"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The scored parts' meshes stay inside their boxes, so their centres are their boxes' centres."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    size = {o.name: o.size for o in T.OBJECTS}[name]
    half = np.asarray(size) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
