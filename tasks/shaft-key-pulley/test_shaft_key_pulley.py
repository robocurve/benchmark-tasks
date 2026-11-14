"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and both arms reach every
pose of the demo. Run in the yam_arena environment:  python -m pytest -q test_shaft_key_pulley.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "shaft_key_pulley.py")
T = TASK.module

UP = T.UP_ANGLE
DOWN = np.array([0.0, 0.0, -1.0])


def _q(r):
    return np.array(T._matrix_xyzw(r))


def _gripper(tcp, contact=0.0, opening=0.09):
    tcp = np.asarray(tcp, float)
    return T.GripperState(back=tcp - T.PAD_BACK * DOWN, tips=tcp + T.PAD_TIP_PAST_TCP * DOWN, opening=opening,
                          contact=contact)


def _holding(point):
    """A gripper closed on whatever is at ``point`` (pressing, pads apart)."""
    return _gripper(np.asarray(point, float) + (0, 0, 0.005), contact=10.0, opening=0.012)


def _home(side):
    return _gripper(T._START_TCP[side])


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def key_seated_pose(angle=UP, lift=0.0, side=0.0, along=0.0):
    """The key in the shaft's keyway (``lift`` off its floor, ``side`` off its middle line, ``along`` the shaft)."""
    p = T.KEY_SEAT_P + lift * T.KEYWAY_DIR + side * T.KEYWAY_SIDE + np.array([along, 0.0, 0.0])
    return T.shaft_point(p, angle), T.shaft_rot(angle) @ T.KEY_SEAT_R


def pulley_pose(face, angle=UP, turn=0.0, drop=0.0):
    """The pulley coaxial with the shaft, its face at ``face`` along it, its keyway ``turn`` off the shaft's, and its
    axis ``drop`` below the shaft's (m)."""
    r = T.shaft_rot(angle) @ T.PULLEY_HOME_R
    r = T.axis_angle_matrix(r[:, 2], turn) @ r
    return T.shaft_point(np.array([face, 0.0, 0.0]), angle) - np.array([0.0, 0.0, drop]), r


class Episode:
    def __init__(self):
        self.t = 0.0
        self.angle = 0.0
        self.key, self.key_r = T.KEY_POS.copy(), T.KEY_R.copy()
        self.pulley, self.pulley_r = T.PULLEY_START_POS.copy(), T.PULLEY_START_R.copy()
        self.left, self.right = _home("left"), _home("right")
        self.wobble = 0.0

    def objects(self):
        jiggle = np.array([self.wobble * math.sin(40 * self.t), 0.0, 0.0])
        return {
            "shaft_stand": _state(np.array([*T.STAND_XY, T.TOP])),
            "key": _state(self.key + jiggle, _q(self.key_r)),
            "pulley": _state(self.pulley, _q(self.pulley_r)),
            "cradle": _state(T.CRADLE_POS),
            "shaft": T.PartPose(pos=T.AXIS_W.copy(), quat_xyzw=_q(T.shaft_rot(self.angle))),
            "shaft_angle": self.angle,
            "left_gripper": self.left,
            "right_gripper": self.right,
            "episode_time": self.t,
        }

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

    def crank_up(self):
        self.step(angle=UP, left=_holding(T.shaft_point([T.SLEEVE_X, 0.0, -T.CRANK_R], UP)))

    def key_in(self, **kw):
        pos, r = key_seated_pose(self.angle, **kw)
        self.step(key=pos, key_r=r, right=_holding(pos))
        self.step(right=_home("right"))

    def pulley_to(self, face, hold=False, **kw):
        pos, r = pulley_pose(face, self.angle, **kw)
        self.step(pulley=pos, pulley_r=r, right=_holding(pos + r @ np.array([T.PULLEY_R, 0.0, 0.01])))
        if not hold:
            self.step(right=_home("right"))

    def turn_shaft(self, angle, **changes):
        """The shaft turns to ``angle``, the key and the pulley with it (one control step)."""
        turn = T.shaft_rot(angle) @ T.shaft_rot(self.angle).T
        self.key, self.key_r = T.AXIS_W + turn @ (self.key - T.AXIS_W), turn @ self.key_r
        self.pulley, self.pulley_r = T.AXIS_W + turn @ (self.pulley - T.AXIS_W), turn @ self.pulley_r
        return self.step(angle=angle, **changes)

    def let_crank_down(self):
        """The left hand lets the crank down: the shaft turns back to rest, the key and the pulley with it."""
        self.turn_shaft(0.0, left=_home("left"))

    def full_course(self):
        self.crank_up()
        self.key_in()
        self.pulley_to(T.SHOULDER_X)
        self.let_crank_down()


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"key", "pulley"}


def test_start_scores_zero():
    e = Episode()
    assert e.step() == (0.0, False)
    assert e.done() == [False, False, False, False]


def test_key_in_the_keyway_is_a_step():
    e = Episode()
    e.crank_up()
    e.key_in()
    assert e.done() == [True, False, False, False] and e.step()[0] == 0.2


def test_key_within_the_keyways_play_counts():
    e = Episode()
    e.crank_up()
    e.key_in(side=0.0001, lift=0.0003, along=0.004)  # 0.1 mm a side, the hub keyway's 0.3 mm roof gap, overhanging
    assert e.done()[0] is True


def test_key_wedged_in_the_keyways_mouth_is_not_in():
    e = Episode()
    e.crank_up()
    e.key_in(side=0.0006, lift=0.0012)  # perched on the keyway's entry chamfer
    assert e.done()[0] is False


def test_key_still_held_in_the_keyway_is_not_in():
    e = Episode()
    e.crank_up()
    pos, r = key_seated_pose(UP)
    e.step(key=pos, key_r=r, right=_holding(pos))
    assert e.done()[0] is False


def test_key_resting_on_the_keyways_edge_is_not_in():
    e = Episode()
    e.crank_up()
    e.key_in(side=0.004, lift=0.004)
    assert e.done()[0] is False


def test_key_across_the_shaft_is_not_in():
    e = Episode()
    e.crank_up()
    pos, r = key_seated_pose(UP)
    e.step(key=pos, key_r=r @ T._rz(math.pi / 2))
    assert e.done()[0] is False


def test_key_hanging_off_the_shafts_end_is_not_in():
    e = Episode()
    e.crank_up()
    e.key_in(along=0.012)
    assert e.done()[0] is False


def test_key_on_the_keyways_closed_end_is_not_in():
    e = Episode()
    e.crank_up()
    e.key_in(along=-0.004)  # its end up on the plain seat past the keyway's end
    assert e.done()[0] is False


def test_pulley_started_over_the_key_is_a_step():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHAFT_END_X - 0.02)
    assert e.done() == [True, True, False, False] and e.step()[0] == 0.4


def test_pulley_at_the_shafts_end_is_not_started():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHAFT_END_X - 0.005)
    assert e.done()[1] is False


def test_pulley_off_the_shafts_axis_is_not_on():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHAFT_END_X - 0.02, drop=0.004)
    assert e.done()[1] is False


def test_pulley_home_over_the_key_is_a_step():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHOULDER_X + 0.001)
    assert e.done() == [True, True, True, False]  # the left hand still holds the crank
    assert e.step()[0] == 0.6


def test_pulley_short_of_the_shoulder_is_not_home():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHOULDER_X + 0.004)
    assert e.done()[:3] == [True, True, False]


def test_pulley_home_without_the_key_earns_nothing():
    e = Episode()
    e.crank_up()
    e.pulley_to(T.SHOULDER_X)
    e.let_crank_down()
    assert e.done() == [False, False, False, False] and e.step(1.5) == (0.0, False)


def test_pulley_home_with_its_keyway_away_from_the_key_is_not_home():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHOULDER_X, turn=math.radians(90))
    assert e.done()[2] is False


def test_key_lifted_against_the_pulleys_roof_with_the_keyway_sideways_still_counts():
    e = Episode()
    e.full_course()  # the crank let down: the keyway faces the robot, the key held in by the pulley
    pos, r = key_seated_pose(0.0, lift=0.0003)
    e.step(key=pos, key_r=r)
    assert e.done() == [True, True, True, True]


def test_full_course_succeeds_after_a_second_still():
    e = Episode()
    e.full_course()
    assert e.step(0.5) == (0.8, False)
    assert e.step(1.0) == (1.0, True)


def test_no_full_score_before_one_second_of_every_condition():
    e = Episode()
    e.full_course()  # every full-success condition holds from this step on
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step() == (1.0, True)  # held 1 s


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0


def _clear_right():
    return _gripper(T.shaft_point([T.SHOULDER_X + T.PULLEY_LEN + 0.06, 0.0, 0.0], 0.0))  # 6 cm past the hub's end


MISSING = {
    "key never fitted": lambda e: (e.crank_up(), e.pulley_to(T.SHOULDER_X), e.let_crank_down()),
    "key still held": lambda e: (e.full_course(), e.step(right=_holding(e.key))),
    "pulley short of the shoulder": lambda e: (e.crank_up(), e.key_in(), e.pulley_to(T.SHOULDER_X + 0.004),
                                               e.let_crank_down()),
    "pulley's keyway off the key": lambda e: (e.crank_up(), e.key_in(),
                                              e.pulley_to(T.SHOULDER_X, turn=math.radians(90)), e.let_crank_down()),
    "pulley still held": lambda e: (e.crank_up(), e.key_in(), e.pulley_to(T.SHOULDER_X, hold=True),
                                    e.let_crank_down()),
    "crank still held": lambda e: (e.crank_up(), e.key_in(), e.pulley_to(T.SHOULDER_X)),
    "gripper within 3 cm of the pulley": lambda e: (
        e.full_course(), e.step(right=_gripper(T.shaft_point([T.SHOULDER_X + T.PULLEY_LEN + 0.02, 0.0, 0.0], 0.0)))),
    "parts moving": lambda e: (e.full_course(), e.step(wobble=0.003)),
    "pulley off the table": lambda e: (e.full_course(), e.step(pulley=np.array([0.3, -0.4, T.TOP - 0.3]))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.5) == (0.8, False)
    near = T.shaft_point([T.SHOULDER_X + T.PULLEY_LEN + 0.02, 0.0, 0.0], 0.0)
    assert e.step(right=_gripper(near)) == (0.6, False)  # a gripper back within 3 cm: not released, timing stops
    assert e.step(right=_clear_right()) == (0.8, False)  # clear again: the second starts over on this step
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 1.6 s since the conditions first held, but 0.95 s unbroken
    assert e.step() == (1.0, True)


def test_the_shaft_turning_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.9) == (0.8, False)
    assert e.turn_shaft(math.radians(3)) == (0.8, False)  # every step still done, but the parts turned 3 degrees
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # 0.95 s still at the new angle
    assert e.step() == (1.0, True)


def test_slow_creep_within_the_tolerance_still_succeeds_but_beyond_it_restarts():
    e = Episode()
    e.full_course()
    for _ in range(10):  # 1.5 degrees in 0.5 s: within 2 degrees of where the second began
        e.turn_shaft(e.angle + math.radians(0.15))
    assert e.step(0.45) == (0.8, False)
    assert e.turn_shaft(e.angle + math.radians(1.0)) == (0.8, False)  # 2.5 degrees from the start: restarts
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_full_score_drops_when_a_part_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    assert e.turn_shaft(math.radians(5)) == (0.8, False)


def test_pulley_still_held_is_not_released():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHOULDER_X, hold=True)
    e.step(left=_home("left"))
    assert e.done() == [True, True, True, False]


def test_crank_still_held_is_not_released():
    e = Episode()
    e.crank_up()
    e.key_in()
    e.pulley_to(T.SHOULDER_X)
    assert e.done() == [True, True, True, False]
    assert e.step(1.5) == (0.6, False)


def test_open_gripper_resting_by_the_pulley_is_not_released():
    e = Episode()
    e.full_course()
    near = T.shaft_point([T.SHOULDER_X + T.PULLEY_LEN + 0.02, 0.0, 0.0], 0.0)  # 2 cm past the hub's end, not touching
    assert e.step(1.5, right=_gripper(near)) == (0.6, False)


def test_released_anywhere_clear_succeeds_without_going_home():
    e = Episode()
    e.full_course()
    clear = T.shaft_point([T.SHOULDER_X + T.PULLEY_LEN + 0.06, 0.0, 0.0], 0.0)  # 6 cm past the hub's end
    assert e.step(1.5, right=_gripper(clear), left=_gripper(T._START_TCP["left"] + (0.05, 0.0, -0.1))) == (1.0, True)


def test_parts_still_moving_is_not_success():
    e = Episode()
    e.full_course()
    assert e.step(1.5, wobble=0.003) == (0.8, False)


def test_key_off_the_table_scores_zero():
    e = Episode()
    e.crank_up()
    e.key_in()
    assert e.step(key=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


def test_pulley_off_the_table_scores_zero():
    e = Episode()
    assert e.step(pulley=np.array([0.3, -0.4, T.TOP - 0.3]))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_every_demo_pose_is_reachable(arm):
    """Static reach: each waypoint's pose has an IK solution (several seeds) within 1 mm and 1 degree, inside the
    joint limits."""
    lo, hi = np.array(ik.LOWER), np.array(ik.UPPER)
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(lo, hi) for _ in range(7)]
    for w in T.demo()[arm]:
        rot = w.rot if getattr(w, "rot", None) is not None else grip_rotation(w.yaw)
        q4 = ik.matrix_to_quat_xyzw(rot)
        best = (1.0, math.pi)
        for seed in seeds:
            _, pos_err, rot_err = ik.solve(np.asarray(w.pos, float), q4, seed, base_pos=BASES[arm], iters=150)
            if pos_err * 1000 + math.degrees(rot_err) < best[0] * 1000 + math.degrees(best[1]):
                best = (pos_err, rot_err)
            if best[0] < 0.001 and math.degrees(best[1]) < 1.0:
                break
        assert best[0] < 0.001 and math.degrees(best[1]) < 1.0, (w.label, best)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg
