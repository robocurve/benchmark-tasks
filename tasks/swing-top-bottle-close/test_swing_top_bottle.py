"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_swing_top_bottle.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "swing_top_bottle.py")
T = TASK.module
QUAT = np.array([0.0, 0.0, math.sin(T.BOTTLE_YAW / 2), math.cos(T.BOTTLE_YAW / 2)])


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _on_bottle(pos=None, contact=20.0):
    """The left gripper closed on the bottle's body from its left."""
    pos = T.BOTTLE_POS if pos is None else pos
    tcp = np.asarray(pos) + np.array([0.0, 0.0, T.HOLD_Z])
    return _gripper(tcp, contact=contact, opening=0.061, axis=(0.0, -1.0, 0.0))


def _on_paddle(bail, contact=20.0):
    k = T.linkage(bail, 0.0, 0.0, 0.0)
    return _gripper(T.to_world(k["paddle"]), contact=contact, opening=0.016, axis=T.BOTTLE_ROT @ k["push"])


def _on_knob(hinge_b, contact=20.0):
    return _gripper(T.to_world(np.asarray(hinge_b) + T.KNOB_GRIP * T._Z), contact=contact, opening=0.012)


HANG = T.linkage(*T.START_JOINTS.values())


def _stretch(bail):
    """The link's stretch (m) with the stopper seated on the mouth and the lever at ``bail``: the hook's distance from
    the seated hinge, beyond the link's unstretched length."""
    return float(np.linalg.norm(T.HINGE_SEATED - T.linkage(bail, 0.0, 0.0, 0.0)["hook"]) - T.LINK_LEN)


class Episode:
    def __init__(self):
        self.t = 0.0
        self.pos, self.quat = T.BOTTLE_POS.copy(), QUAT.copy()
        self.bail, self.link, self.seal, self.hinge = T.START_JOINTS.values()
        self.stopper_b, self.stopper_axis = HANG["hinge"].copy(), HANG["axis"].copy()
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        rot = T._rot(self.quat)
        root = ObjectState(pos=self.pos.copy(), quat_xyzw=self.quat.copy(), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                           low=self.pos.copy(), high=self.pos.copy())
        tilt = math.atan2(self.stopper_axis[0], self.stopper_axis[2])  # the stopper's axis in the bottle's x-z plane
        srot = rot @ T._ry(tilt)
        w = math.sqrt(max(0.0, 1 + np.trace(srot))) / 2
        squat = np.array([(srot[2, 1] - srot[1, 2]) / (4 * w), (srot[0, 2] - srot[2, 0]) / (4 * w),
                          (srot[1, 0] - srot[0, 1]) / (4 * w), w])
        return {"bottle": root, "bail": self.bail, "link": self.link, "seal": self.seal, "hinge": self.hinge,
                "stopper_pos": self.pos + rot @ self.stopper_b, "stopper_quat": squat,
                "left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t}

    def step(self, seconds=0.05, **changes):
        for k, v in changes.items():
            setattr(self, k, v)
        end = self.t + seconds
        while True:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            last = (T.score(objects), T.success(objects))
            if self.t >= end - 1e-9:
                return last

    def scores(self, seconds, **changes):
        """(score, success) at every control step over ``seconds``."""
        for k, v in changes.items():
            setattr(self, k, v)
        out, end = [], self.t + seconds
        while self.t < end - 1e-9:
            self.t = round(self.t + 0.05, 6)
            objects = self.objects()
            out.append((T.score(objects), T.success(objects)))
        return out

    def done(self):
        return T.steps_done(self.objects())

    def hold(self):
        self.step(left=_on_bottle())

    def stopper_on(self):
        """The stopper carried onto the mouth (the lever follows it up), let go: the link just taut."""
        self.step(right=_on_knob(self.stopper_b))
        self.step(bail=0.0, link=0.0, hinge=0.0, seal=0.0, stopper_b=T.HINGE_SEATED.copy(),
                  stopper_axis=np.array([0.0, 0.0, 1.0]), right=_on_knob(T.HINGE_SEATED))
        self.step(right=_home("right"))

    def push(self, to=None, held_by=None):
        """The lever pushed down from where it is to ``to`` (its stop), the link stretching as it goes."""
        to = T.CLOSED if to is None else to
        self.step(right=_on_paddle(self.bail))
        for a in np.linspace(self.bail, to, 10)[1:]:
            a = float(a)
            self.step(bail=a, seal=max(0.0, _stretch(a)), right=_on_paddle(a))

    def let_go(self):
        self.step(right=_home("right"))
        self.step(left=_home("left"))

    def full_course(self):
        self.hold()
        self.stopper_on()
        self.push()
        self.let_go()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert TASK.names == ("bottle",)


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_the_open_lever_on_its_stop_is_not_closed():
    e = Episode()
    assert e.objects()["bail"] >= T.PAST and not T.seated(e.objects())
    assert e.done()[2] is False


def test_bottle_held_is_a_step():
    e = Episode()
    e.hold()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_holding_the_paddle_is_not_holding_the_bottle():
    e = Episode()
    e.step(right=_on_paddle(e.bail))
    assert e.done()[0] is False


def test_holding_the_stopper_is_not_holding_the_bottle():
    e = Episode()
    e.step(right=_on_knob(e.stopper_b))
    assert e.done()[0] is False


def test_stopper_on_the_mouth_is_a_step():
    e = Episode()
    e.hold()
    e.stopper_on()
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.4


def test_stopper_beside_the_mouth_is_not_on():
    e = Episode()
    e.hold()
    e.stopper_on()
    e.step(stopper_b=T.HINGE_SEATED + np.array([0.004, 0.0, 0.0]))
    assert e.done()[1] is False


def test_stopper_cocked_on_the_mouth_is_not_on():
    e = Episode()
    e.hold()
    e.stopper_on()
    e.step(stopper_axis=np.array([math.sin(math.radians(10)), 0.0, math.cos(math.radians(10))]))
    assert e.done()[1] is False


def test_lever_pushed_over_centre_is_a_step():
    e = Episode()
    e.hold()
    e.stopper_on()
    e.push()
    assert e.done() == [True, True, True, False] and e.step()[0] == 0.6


def test_lever_pushed_short_of_centre_is_not_closed():
    e = Episode()
    e.hold()
    e.stopper_on()
    e.push(to=T.CENTRE - math.radians(3.0))
    assert e.done()[2] is False


def test_lever_pushed_over_with_the_bottle_let_go_still_crosses():
    """No hand rule in the score: the crossing counts with the stopper on; whether the bottle stays up is physics (the
    demo with the left hand idle fails in the simulator)."""
    e = Episode()
    e.hold()
    e.stopper_on()
    e.step(left=_home("left"))
    e.push()
    assert e.done()[2] is True


def test_stopper_pulled_on_with_the_lever_left_down_does_not_count():
    """The link stretched by pulling the stopper onto the mouth with the lever still on its stop: no crossing."""
    e = Episode()
    e.hold()
    e.step(right=_on_knob(e.stopper_b))
    e.step(stopper_b=T.HINGE_SEATED.copy(), stopper_axis=np.array([0.0, 0.0, 1.0]), seal=_stretch(T.CLOSED),
           right=_on_knob(T.HINGE_SEATED))
    e.let_go()
    assert e.done() == [True, True, False, False]


def test_lever_lifted_back_up_loses_the_step_until_pushed_over_again():
    e = Episode()
    e.full_course()
    e.step(bail=0.0, seal=0.0)
    assert e.done()[2:] == [False, False]
    e.step(bail=T.CLOSED, seal=max(0.0, _stretch(T.CLOSED)))  # pushed down again, from the start of the arc: a new crossing
    assert e.done()[2] is True


def test_still_holding_the_bottle_is_not_let_go():
    e = Episode()
    e.hold()
    e.stopper_on()
    e.push()
    e.step(right=_home("right"))
    assert e.done() == [True, True, True, False]


def test_a_gripper_beside_the_closure_is_not_let_go():
    e = Episode()
    e.full_course()
    e.step(right=_gripper(T.to_world(np.array([-0.06, 0.0, 0.21]))))  # open, fingertips just over the lever
    assert e.done()[3] is False


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True, True, True, True]
    assert e.step(1.1) == (1.0, True)


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode()
    e.full_course()  # every full-success condition holds from the last of these steps on
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step() == (1.0, True)  # held 1 s


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0


def _wobble(e, what):
    """``what`` moves back and forth every control step, a little more than the stillness tolerance from where the
    second began, every step still done (the stopper still within its seat's tolerances)."""
    k = round(e.t / 0.05)
    sign = 1.0 if k % 2 else -1.0
    if what == "bottle":
        e.pos = T.BOTTLE_POS + np.array([0.0025 * sign, 0.0, 0.0])
    elif what == "lever":
        e.bail = T.CLOSED - math.radians(1.25 + 1.25 * sign)
    elif what == "stopper":
        e.stopper_b = T.HINGE_SEATED + sign * np.array([0.002, 0.0, 0.0012])


def _knocked_then_righted(e):
    e.hold()
    e.stopper_on()
    a = math.radians(80.0)
    e.step(quat=np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)]))
    e.step(quat=QUAT.copy())
    e.push()
    e.let_go()


MISSING = {
    "bottle never held": lambda e: (e.stopper_on(), e.push(), e.let_go()),
    "stopper beside the mouth": lambda e: (e.full_course(),
                                           e.step(stopper_b=T.HINGE_SEATED + np.array([0.004, 0.0, 0.0]))),
    "lever short of centre": lambda e: (e.hold(), e.stopper_on(), e.push(to=T.CENTRE - math.radians(3.0)),
                                        e.let_go()),
    "stopper pulled on, lever never crossed": lambda e: (
        e.hold(), e.step(right=_on_knob(e.stopper_b)),
        e.step(stopper_b=T.HINGE_SEATED.copy(), stopper_axis=np.array([0.0, 0.0, 1.0]), seal=_stretch(T.CLOSED),
               right=_on_knob(T.HINGE_SEATED)), e.let_go()),
    "left hand still on the bottle": lambda e: (e.hold(), e.stopper_on(), e.push(), e.step(right=_home("right"))),
    "right hand still on the paddle": lambda e: (e.hold(), e.stopper_on(), e.push(), e.step(left=_home("left"))),
    "gripper within 3 cm of the closure": lambda e: (
        e.full_course(), e.step(right=_gripper(T.to_world(np.array([-0.06, 0.0, 0.21]))))),
    "bottle moved off its place": lambda e: (e.full_course(), e.step(pos=T.BOTTLE_POS + (0.03, 0.0, 0.0))),
    "bottle leaning": lambda e: (e.full_course(), e.step(quat=np.array([math.sin(math.radians(4.0)), 0.0, 0.0,
                                                                        math.cos(math.radians(4.0))]))),
    "bottle knocked over earlier": _knocked_then_righted,
    "bottle off the table": lambda e: (e.full_course(), e.step(pos=T.BOTTLE_POS - (0.0, 0.0, 0.2))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


@pytest.mark.parametrize("part", ["bottle", "lever", "stopper"])
def test_no_full_score_while_a_part_keeps_moving(part):
    e = Episode()
    e.full_course()
    history = []
    for _ in range(60):  # 3 s
        _wobble(e, part)
        history.append(e.step())
    assert e.done() == [True, True, True, True]
    assert all(s == 0.8 and not ok for s, ok in history), part


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.5) == (0.8, False)
    near = _gripper(T.to_world(np.array([-0.06, 0.0, 0.21])))
    assert e.step(right=near) == (0.6, False)  # a gripper back within 3 cm: not let go, timing stops
    assert e.step(right=_home("right")) == (0.8, False)  # clear again: the second starts over on this step
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 1.6 s since the conditions first held, but 0.95 s unbroken
    assert e.step() == (1.0, True)


def test_a_part_moving_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.9) == (0.8, False)
    assert e.step(bail=T.CLOSED - math.radians(3.0)) == (0.8, False)  # every step still done, the lever moved 3 deg
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 0.95 s still at the new angle
    assert e.step() == (1.0, True)


def test_slow_creep_within_the_tolerance_still_succeeds_but_beyond_it_restarts():
    e = Episode()
    e.full_course()
    for k in range(10):  # 1.5 mm in 0.5 s: within 2 mm of where the second began
        e.step(pos=T.BOTTLE_POS + np.array([0.00015 * (k + 1), 0.0, 0.0]))
    assert e.step(0.45) == (0.8, False)
    assert e.step(pos=T.BOTTLE_POS + np.array([0.0025, 0.0, 0.0])) == (0.8, False)  # 2.5 mm from the start: restarts
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_full_score_drops_when_a_part_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    assert e.step(bail=T.CLOSED - math.radians(5.0)) == (0.8, False)


def test_arms_not_home_still_scores_1():
    """The arms' return is not scored (the release is)."""
    e = Episode()
    e.full_course()
    assert e.step(1.1, left=_gripper(T._START_TCP["left"] + (0.0, 0.0, -0.1))) == (1.0, True)


def test_lever_still_moving_is_not_success():
    e = Episode()
    e.full_course()
    for k in range(25):  # a 3-degree wobble
        e.step(bail=T.CLOSED - math.radians(1.5 + 1.5 * math.sin(k)))
    assert e.step(bail=T.CLOSED) == (0.8, False)


def test_bottle_moved_from_its_place_loses_the_steps():
    e = Episode()
    e.full_course()
    e.step(pos=T.BOTTLE_POS + (0.03, 0.0, 0.0))
    assert e.done() == [False, False, False, False]


def test_bottle_leaning_loses_the_steps():
    e = Episode()
    e.full_course()
    a = math.radians(8.0)
    e.step(quat=np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)]))
    assert e.done() == [False, False, False, False]


def test_bottle_knocked_over_scores_zero_even_if_set_back_up():
    e = Episode()
    e.hold()
    e.stopper_on()
    a = math.radians(80.0)
    assert e.step(quat=np.array([math.sin(a / 2), 0.0, 0.0, math.cos(a / 2)]))[0] == 0.0
    e.step(quat=QUAT.copy())
    e.push()
    e.let_go()
    assert e.step(1.5) == (0.0, False)


def test_bottle_off_the_table_scores_zero():
    e = Episode()
    assert e.step(pos=T.BOTTLE_POS - (0.0, 0.0, 0.2))[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    e.hold()
    assert e.step(bail=float("nan"))[0] == 0.0


def test_linkage_solver_round_trips():
    for p in T.CARRY:
        b, l_, h = T.solve_linkage(p)
        k = T.linkage(b, l_, 0.0, h)
        assert np.linalg.norm(k["hinge"] - p) < 1e-6 and abs(k["axis"][0]) < 1e-6
    k = T.linkage(0.0, 0.0, 0.0, 0.0)
    assert np.linalg.norm(k["hinge"] - T.HINGE_SEATED) < 1e-6


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    left, right = (sum(w.duration for w in d[s]) for s in ("left", "right"))
    let_go = sum(w.duration for w in d["left"][: [w.label for w in d["left"]].index("let go")])
    backed_off = sum(w.duration for w in d["right"][:-1])
    assert let_go >= backed_off - 1e-9 and max(left, right) < 40.0
