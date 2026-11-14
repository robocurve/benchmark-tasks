"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_steam_pan_lids.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena import ik
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "steam_pan_lids.py")
T = TASK.module
HN, HS, TS = "lid_third_notched", "lid_third_solid", "lid_sixth_solid"


def _quat_z(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def _state(pos, quat):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos.copy(), high=pos.copy())


def _gripper(tcp, contact=0.0, opening=0.09, axis=(0.0, 0.0, -1.0)):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side])


def _seat(pan, lid_dz=0.0):
    """A lid's pose resting seated on ``pan``."""
    return np.array([*T.PAN_XY[pan], T.TOP + T.PAN_H + lid_dz])


class Episode:
    """The lids, the spoon and the grippers, stepped at 20 Hz through the task's own scorer."""

    def __init__(self):
        self.t = 0.0
        self.lids = {n: (np.array([*T.LID_XY[n.split("_", 1)[1]], T.TOP + 0.0002]), _quat_z(0.0)) for n in T.LID_NAMES}
        self.spoon = (np.array(T.SPOON_POS, float), np.array(T.SPOON_QUAT, float))
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        out = {n: _state(*self.lids[n]) for n in T.LID_NAMES}
        out["spoon"] = _state(*self.spoon)
        out.update({"left_gripper": self.left, "right_gripper": self.right, "episode_time": self.t})
        return out

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

    def put(self, name, pos, yaw=0.0, tilt=0.0):
        q = _quat_z(yaw)
        if tilt:
            qy = np.array([0.0, math.sin(tilt / 2), 0.0, math.cos(tilt / 2)])
            q = T_qmul(q, qy)
        self.lids[name] = (np.asarray(pos, float), q)

    def lid_on(self, name, pan, dz=0.0, **kw):
        self.put(name, _seat(pan, dz), **kw)

    def hold(self, name, side="left"):
        """``side``'s gripper closed on the handle of lid ``name`` (wherever it is now)."""
        pos = self.lids[name][0] + np.array([0.0, 0.0, 0.02])
        setattr(self, side, _gripper(pos, contact=15.0, opening=0.012, axis=(0.0, 0.0, -1.0)))

    def full_course(self):
        self.lid_on(HN, "third")
        self.lid_on(TS, "sixth")
        self.step()


def T_qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.names) == {"pan_third", "pan_sixth", *T.LID_NAMES, "spoon"}


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_the_asset_files_exist():
    for part in T.OBJECTS:
        assert pathlib.Path(part.usd_path).exists(), part.usd_path


def test_the_right_lids_are_full_success():
    e = Episode()
    e.full_course()
    assert e.done() == [True] * 4
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step() == (1.0, True)  # held 1 s


def test_success_is_timed_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0 and T.POINTS * len(T.STEPS) <= 0.8 + 1e-9


def test_a_lid_hovering_over_a_pan_in_the_gripper_is_not_covering():
    e = Episode()
    e.lid_on(HN, "third", dz=0.012)
    e.hold(HN)
    assert e.step()[0] == 0.0 and e.done() == [False] * 4


def test_a_lid_resting_on_the_pan_without_being_seated_covers_only():
    e = Episode()
    e.lid_on(HN, "third", dz=0.0045, tilt=math.radians(1.7))
    assert e.step()[0] == 0.2 and e.done() == [True, False, False, False]


def test_a_plain_lid_on_the_spoon_handle_is_covering_but_not_seated():
    """The spoon's handle holds the lid about 4 mm above the rim, tilted: the plain lid is not seated."""
    e = Episode()
    e.lid_on(HS, "third", dz=0.0036, tilt=math.radians(1.2))
    e.lid_on(TS, "sixth")
    e.step()
    assert e.done() == [True, False, True, True]
    assert all(s < 1.0 and not ok for s, ok in e.scores(3.0))


def test_the_notched_lid_the_wrong_way_round_is_not_seated():
    """Turned third a turn the notch is over the far end and the lid rests on the handle."""
    e = Episode()
    e.lid_on(HN, "third", dz=0.0045, tilt=math.radians(1.5), yaw=math.pi)
    e.lid_on(TS, "sixth")
    e.step()
    assert e.done()[1] is False
    assert all(s < 1.0 and not ok for s, ok in e.scores(3.0))


def test_the_notched_lid_the_right_way_round_is_seated_and_covers_the_opening_but_for_the_notch():
    e = Episode()
    e.lid_on(HN, "third")
    c = T.lid_on_pan(e.objects(), HN, "third")
    assert 0.968 < c["cover"] < 0.978  # 50 x 27 mm of the opening is under the notch
    assert T._seated(c)


def test_a_notch_over_the_opening_leaves_it_open():
    """The notched GN 1/3 lid pushed along so its notch end is over a GN 1/6 pan leaves part of the food open; centred, its
    notch hangs clear of the small pan and the big lid covers it."""
    e = Episode()
    e.lid_on(HN, "sixth")
    assert T.lid_on_pan(e.objects(), HN, "sixth")["cover"] >= T.COVER_FULL
    e.put(HN, _seat("sixth") + np.array([-0.08, 0.0, 0.0]))
    c = T.lid_on_pan(e.objects(), HN, "sixth")
    assert 0.9 < c["cover"] < T.COVER_FULL


def test_a_plain_third_lid_covers_the_sixth_pan_fully():
    e = Episode()
    e.lid_on(HN, "third")
    e.lid_on(HS, "sixth")
    e.step()
    assert e.done() == [True] * 4


def test_the_sixth_lid_does_not_cover_the_third_pan():
    e = Episode()
    e.lid_on(TS, "third")
    c = T.lid_on_pan(e.objects(), TS, "third")
    assert c["cover"] < 0.95
    assert e.step()[0] == 0.0


def test_one_lid_cannot_cover_both_pans():
    e = Episode()
    e.lid_on(HS, "third")  # on the handle, it covers the third pan but is not seated
    e.lids[HS] = (_seat("third") + np.array([0.0, 0.0, 0.0036]), _quat_z(0.0))
    assert e.step()[0] == 0.2


def test_the_spoon_moved_loses_the_second_step_and_success():
    e = Episode()
    e.full_course()
    e.step(spoon=(np.array(T.SPOON_POS) + np.array([0.03, 0.0, 0.0]), np.array(T.SPOON_QUAT)))
    assert e.done()[1] is False
    assert all(not ok for _, ok in e.scores(3.0))


def test_a_lid_a_little_off_centre_or_turned_is_not_seated():
    e = Episode()
    e.full_course()
    e.put(HN, _seat("third") + np.array([0.0, 0.008, 0.0]))
    assert e.step()[0] <= 0.6
    e.put(HN, _seat("third"), yaw=math.radians(3.5))
    assert e.step()[0] <= 0.6


MISSING = {
    "spoon pan lid not seated": lambda e: (e.full_course(), e.lid_on(HN, "third", dz=0.003), e.step()),
    "other pan lid not seated": lambda e: (e.full_course(), e.lid_on(TS, "sixth", dz=0.003), e.step()),
    "spoon pan uncovered": lambda e: (e.full_course(), e.put(HN, (0.3, 0.6, T.TOP)), e.step()),
    "other pan uncovered": lambda e: (e.full_course(), e.put(TS, (0.3, -0.5, T.TOP)), e.step()),
    "spoon out of place": lambda e: (e.full_course(), e.step(spoon=(np.array(T.SPOON_POS) + (0.03, 0, 0),
                                                                    np.array(T.SPOON_QUAT)))),
    "spoon turned": lambda e: (e.full_course(), e.step(spoon=(np.array(T.SPOON_POS), T_qmul(_quat_z(0.2), T.SPOON_QUAT)))),
    "lid still in a gripper": lambda e: (e.full_course(), e.hold(HN, "left"), e.step()),
    "other lid still in a gripper": lambda e: (e.full_course(), e.hold(TS, "right"), e.step()),
    "gripper within 3 cm of a lid": lambda e: (
        e.full_course(), e.step(right=_gripper(e.lids[TS][0] + np.array([0.0, 0.0, 0.045])))),
    "gripper within 3 cm of the spoon": lambda e: (
        e.full_course(), e.step(left=_gripper(np.array(T.SPOON_POS) + np.array([0.12, 0.0, 0.12])))),
    "lid off the table": lambda e: (e.full_course(), e.put(HS, (0.3, -0.07, T.TOP - 0.1)), e.step()),
    "spoon off the table": lambda e: (e.full_course(), e.step(spoon=(np.array(T.SPOON_POS) - (0, 0, 0.2),
                                                                     np.array(T.SPOON_QUAT)))),
}


@pytest.mark.parametrize("missing", sorted(MISSING))
def test_no_full_score_while_any_success_condition_is_missing(missing):
    e = Episode()
    MISSING[missing](e)
    history = e.scores(3.0)
    assert all(s < 1.0 and not ok for s, ok in history), missing
    assert max(s for s, _ in history) <= 0.8


@pytest.mark.parametrize("part", ["lid", "spoon"])
def test_no_full_score_while_a_part_keeps_moving(part):
    e = Episode()
    e.full_course()
    history = []
    for k in range(60):  # 3 s
        sign = 1.0 if k % 2 else -1.0
        if part == "lid":
            e.lid_on(HN, "third")
            e.lids[HN] = (e.lids[HN][0] + np.array([0.0, 0.0025 * sign, 0.0]), e.lids[HN][1])
        else:
            e.spoon = (np.array(T.SPOON_POS) + np.array([0.0025 * sign, 0.0, 0.0]), np.array(T.SPOON_QUAT))
        history.append(e.step())
    assert e.done() == [True] * 4
    assert all(s == 0.8 and not ok for s, ok in history), part


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.5) == (0.8, False)
    e.step(right=_gripper(e.lids[TS][0] + np.array([0.0, 0.0, 0.045])))  # a gripper too close, for one step
    e.step(right=_home("right"))
    assert e.step(T.STILL_S - 0.05) == (0.8, False)
    assert e.step() == (1.0, True)


def test_slow_creep_beyond_the_tolerance_restarts_the_second():
    e = Episode()
    e.full_course()
    for k in range(40):  # a lid creeping 0.15 mm a step: 3 mm in any second, more than the 2 mm tolerance
        e.lids[HN] = (_seat("third") + np.array([0.0, 0.00015 * k, 0.0]), e.lids[HN][1])
        assert e.step() [1] is False


def test_arms_not_home_still_scores_1():
    e = Episode()
    e.full_course()
    e.step(left=_gripper(np.array([0.3, 0.45, 1.0])), right=_gripper(np.array([0.3, -0.45, 1.0])))
    assert e.step(T.STILL_S) == (1.0, True)


def test_full_score_drops_when_a_lid_moves_after_success():
    e = Episode()
    e.full_course()
    assert e.step(T.STILL_S) == (1.0, True)
    e.lid_on(HN, "third", dz=0.01)
    assert e.step()[0] < 1.0


def test_not_finite_scores_zero():
    e = Episode()
    e.full_course()
    e.step(spoon=(np.array([float("nan"), 0.0, 0.8]), np.array(T.SPOON_QUAT)))
    assert e.step()[0] == 0.0


def test_the_spoon_thrown_out_of_its_pan_scores_zero():
    e = Episode()
    e.full_course()
    e.step(spoon=(np.array([0.3, 0.45, T.TOP + 0.01]), np.array(T.SPOON_QUAT)))
    assert e.step()[0] == 0.0


def test_the_spoon_rests_on_the_rim_and_the_floor():
    """Geometry of the start: the handle's underside meets the end rim's inner edge, the bowl sits on the floor."""
    q = T.SPOON_QUAT
    tilt = -math.atan2(2 * q[1] * q[3], 1 - 2 * q[1] ** 2)  # the handle rises toward +x by this much
    assert abs(math.degrees(tilt) - 19.7) < 1e-6
    bowl_x, bowl_z = T.SPOON_X, T.SPOON_Z - 0.0003
    edge_x, edge_z = T.PAN_LEN["third"] / 2 - T.FLANGE_W, T.PAN_H
    along = (edge_x - bowl_x) * math.cos(tilt) + (edge_z - bowl_z) * math.sin(tilt)
    across = -(edge_x - bowl_x) * math.sin(tilt) + (edge_z - bowl_z) * math.cos(tilt)  # the rim's edge below the axis
    assert 0.08 < along < 0.2 and abs(across + 0.0025) < 0.0004, (along, across)


def _tcp_waypoints_ok(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    return plan


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = _tcp_waypoints_ok(arm)
    assert plan.worst_ik_mm < 6.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 3.0, plan.worst_ik_deg


def test_each_arm_alone_cannot_do_both_jobs():
    """The GN 1/6 pan and its lid are out of the right arm's reach and the notched lid is out of the left arm's: both
    arms are needed."""
    rng = np.random.default_rng(0)
    seeds = [np.array([0, 1, 1, -0.6, 0, 0], float)] + [rng.uniform(ik.LOWER, ik.UPPER) for _ in range(8)]

    def reach(arm, xy):
        pos, rot = T._tcp_lid(xy, 0.0), T.GRIP_ROT
        q4 = ik.matrix_to_quat_xyzw(rot)
        for s in seeds:
            _, pe, re = ik.solve(pos, q4, s, base_pos=BASES[arm], iters=150)
            if pe < 0.001 and re < math.radians(1):
                return True
        return False

    assert reach("left", T.LID_XY["sixth_solid"]) and reach("right", T.LID_XY["third_notched"])
    assert not reach("right", T.LID_XY["sixth_solid"]) and not reach("right", T.PAN_XY["sixth"])
    assert not reach("left", T.LID_XY["third_notched"])


def test_demo_timing():
    d = T.demo()
    left, right = (sum(w.duration for w in d[s]) for s in ("left", "right"))
    assert max(left, right) < 60.0
