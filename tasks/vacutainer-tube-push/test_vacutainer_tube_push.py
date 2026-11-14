"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, and the demo's poses are
reachable. Run in the yam_arena environment:  python -m pytest -q test_vacutainer_tube_push.py"""

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "vacutainer_tube_push.py")
T = TASK.module


def _state(pos, rot):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos.copy(), quat_xyzw=np.asarray(T.ik.matrix_to_quat_xyzw(rot), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=pos.copy(), high=pos.copy())


def _gripper(tcp, axis, contact=0.0, opening=0.09):
    tcp, axis = np.asarray(tcp, float), np.asarray(axis, float)
    return T.GripperState(back=tcp - T.PAD_BACK * axis, tips=tcp + T.PAD_TIP_PAST_TCP * axis, opening=opening,
                          contact=contact)


def _home(side):
    return _gripper(T._START_TCP[side], T._START_ROT[side][:, 2])


def _on_tube(pos, rot):
    """The right gripper closed across a tube 30 mm from its closed end, pointing down."""
    grip = np.asarray(pos, float) - (T.TUBE_L - T.GRIP_FROM_BOTTOM) * np.asarray(rot, float)[:, 2]
    return _gripper(grip, np.array([0.0, 0.0, -1.0]), contact=20.0, opening=0.013)


def _on_holder():
    """The left gripper closed on the holder's body."""
    rot = T.left_rot()
    return _gripper(T.holder_point(T.LEFT_S), rot[:, 2], contact=20.0, opening=0.02)


def _on_needle(s, off=(0.0, 0.0), tilt=0.0, slide=0.0):
    """A tube's pose on the needle: its stopper's face ``s`` back from the entry point (the holder pushed ``slide``
    deeper), its axis ``off`` (x, z in the holder's frame) from the needle's and tilted ``tilt`` about x."""
    rot = T.HOLDER_R0 @ T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), tilt) @ T.axis_angle_matrix(
        np.array([1.0, 0.0, 0.0]), -math.pi / 2)
    pos = T.holder_point(s, slide) + T.HOLDER_R0 @ np.array([off[0], 0.0, off[1]])
    return pos, rot


class Episode:
    def __init__(self):
        self.t = 0.0
        self.tubes = {"tube_1": (T.tube_start(1), T.TUBE_LYING_R.copy()),
                      "tube_2": (T.tube_start(2), T.TUBE_LYING_R.copy())}
        self.slide, self.pivot = 0.0, 0.0
        self.left, self.right = _home("left"), _home("right")

    def objects(self):
        link = T.ENTRY + self.slide * T.NEEDLE_DIR
        rot = T.axis_angle_matrix(np.array([1.0, 0.0, 0.0]), self.pivot) @ T.HOLDER_R0
        out = {name: _state(*pose) for name, pose in self.tubes.items()}
        out.update({"training_arm": _state(T.ARM_POS, np.eye(3)), "tube_tray": _state(T.TRAY_POS, np.eye(3)),
                    "holder": _state(T.ENTRY, T.HOLDER_R0), "holder_link": _state(link, rot), "slide": self.slide,
                    "pivot": self.pivot, "left_gripper": self.left, "right_gripper": self.right,
                    "episode_time": self.t})
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

    def done(self):
        return T.steps_done(self.objects())

    def move(self, tube, pos, rot, seconds=0.05, held=True):
        self.tubes[tube] = (np.asarray(pos, float), np.asarray(rot, float))
        return self.step(seconds, right=_on_tube(pos, rot) if held else _home("right"))

    def fill(self, tube, seconds=T.FILL_S + 0.2, s=T.S_SEAT, **kw):
        """Tube ``tube`` carried in line with the holder, pushed onto the needle to ``s`` and held there."""
        self.move(tube, *_on_needle(0.120))
        self.move(tube, *_on_needle(0.060))
        return self.move(tube, *_on_needle(s, **kw), seconds=seconds)

    def back_to_tray(self, tube, k, let_go=True):
        self.move(tube, *_on_needle(0.150))
        pos = T.tube_start(k)
        self.move(tube, pos + (0, 0, 0.1), T.TUBE_LYING_R)
        r = self.move(tube, pos, T.TUBE_LYING_R)
        if let_go:
            r = self.step(right=_home("right"))
        return r

    def full_course(self, release_holder=True):
        self.step(left=_on_holder())
        self.fill("tube_1")
        self.back_to_tray("tube_1", 1)
        self.fill("tube_2")
        self.back_to_tray("tube_2", 2)
        if release_holder:
            self.step(left=_home("left"))


@pytest.fixture(autouse=True)
def fresh_episode():
    T._PROGRESS.__init__()


def test_loads():
    assert TASK.scene == "station"
    blood = tuple(f"{t}_blood_{i}" for t in ("tube_1", "tube_2") for i in range(1, 6))
    assert TASK.names == ("training_arm", "tube_tray", "holder", "tube_1", "tube_2", *blood)


def test_fill_time_is_the_draw_at_the_measured_rate():
    assert T.FILL_S == pytest.approx(6.0 / (5.0 / 7.15))  # 8.58 s
    assert T.DRAW_ML * T.BLOOD_PER_ML == pytest.approx(0.0609, abs=2e-4)  # a 61 mm column in a 11.2 mm bore
    # the full column ends inside the tube, 25 mm short of its inside bottom
    assert T.BLOOD_FROM + T.DRAW_ML * T.BLOOD_PER_ML < T.TUBE_L - 0.0015 - 0.020


@pytest.mark.parametrize("length_mm", [0.05, 1.0, 5.9, 6.0, 6.1, 11.9, 12.0, 17.5, 23.9, 24.0, 30.0, 47.9, 48.0,
                                       55.0, 60.9])
def test_blood_pieces_cover_the_column_exactly(length_mm):
    length = length_mm * T.MM
    starts = T.blood_pieces(length)
    spans = sorted((s, s + mm * T.MM) for mm, s in zip(T.BLOOD_PIECES, starts) if s is not None)
    assert spans and max(e for _, e in spans) == pytest.approx(length)  # the far end is the column's
    assert min(s for s, _ in spans) <= 1e-12  # it starts at the plug ...
    assert min(s for s, _ in spans) >= -6.0 * T.MM  # ... or less than 6 mm inside it (the plug is 9.5 mm long)
    reach = spans[0][1]
    for s, e in spans[1:]:
        assert s <= reach + 1e-12  # no gap
        reach = max(reach, e)
    assert all(s >= -1e-12 for s, _ in spans) or length_mm < 6.0  # only a short column reaches into the plug


def test_no_blood_shown_before_any_is_drawn():
    assert T.blood_pieces(0.0) == [None] * len(T.BLOOD_PIECES)
    e = Episode()
    e.step(1.0)
    assert all(p is None for p in T.blood_poses(e.objects(), "tube_1"))


def test_blood_grows_while_pierced_and_stays_when_pulled_off_early():
    e = Episode()
    e.fill("tube_1", seconds=T.FILL_S / 2)
    assert T._PROGRESS.drawn["tube_1"] == pytest.approx(3.0, abs=0.1)
    e.move("tube_1", *_on_needle(0.150))  # pulled off early: it keeps its blood (its vacuum keeps the rest)
    e.step(3.0)
    assert T._PROGRESS.drawn["tube_1"] == pytest.approx(3.0, abs=0.1)
    assert e.done() == [False] * 4  # partly filled is not filled
    assert T._PROGRESS.drawn["tube_2"] == 0.0


def test_a_filled_tube_shows_its_6_ml_inside_it_along_its_axis():
    e = Episode()
    e.fill("tube_1")
    assert T._PROGRESS.filled["tube_1"] and T._PROGRESS.drawn["tube_1"] == pytest.approx(T.DRAW_ML)
    pos, rot = e.tubes["tube_1"]
    shown = [(mm, w) for mm, w in zip(T.BLOOD_PIECES, T.blood_poses(e.objects(), "tube_1"), strict=True) if w]
    assert sum(mm for mm, _ in shown) * T.MM >= T.DRAW_ML * T.BLOOD_PER_ML  # 48 + 24 mm (overlapping)
    for mm, want in shown:
        d = want[0] - pos
        along = -float(d @ rot[:, 2])
        assert np.linalg.norm(d + along * rot[:, 2]) < 1e-9  # on the tube's axis
        assert T.BLOOD_FROM - 1e-9 <= along - mm * T.MM / 2
        assert along + mm * T.MM / 2 <= T.BLOOD_FROM + T.DRAW_ML * T.BLOOD_PER_ML + 1e-9
        assert np.allclose(want[1], e.objects()["tube_1"].quat_xyzw)


def test_start_scores_zero():
    e = Episode()
    assert e.step(2.0) == (0.0, False)
    assert e.done() == [False] * 4


def test_pierced_and_held_for_the_fill_time_fills_a_tube():
    e = Episode()
    assert e.fill("tube_1", seconds=T.FILL_S - 0.3)[0] == 0.0
    assert e.step(0.4)[0] == pytest.approx(0.2)
    assert e.done() == [True, False, False, False]


def test_on_the_needle_but_not_through_the_diaphragm_does_not_fill():
    e = Episode()
    assert e.fill("tube_1", s=T.PIERCED_S + 0.0008, seconds=10.0)[0] == 0.0


def test_off_axis_or_tilted_is_not_on_the_needle():
    e = Episode()
    assert e.fill("tube_1", off=(0.0015, 0.0), seconds=6.0)[0] == 0.0
    T._PROGRESS.__init__()
    e = Episode()
    assert e.fill("tube_1", tilt=math.radians(8.0), seconds=6.0)[0] == 0.0


def test_a_break_in_the_fill_starts_it_again():
    e = Episode()
    e.fill("tube_1", seconds=T.FILL_S - 1.0)
    e.move("tube_1", *_on_needle(0.050))  # backed off the needle (the sleeve pushes a let-go tube off)
    assert e.move("tube_1", *_on_needle(T.S_SEAT), seconds=T.FILL_S - 1.0)[0] == 0.0
    assert e.step(1.2)[0] == pytest.approx(0.2)


def test_a_filled_tube_laid_in_the_tray_and_let_go_is_a_step():
    e = Episode()
    e.fill("tube_1")
    assert e.back_to_tray("tube_1", 1, let_go=False)[0] == pytest.approx(0.2)  # still held
    assert e.step(right=_home("right"))[0] == pytest.approx(0.4)


def test_an_unfilled_tube_in_the_tray_scores_nothing():
    e = Episode()
    e.move("tube_1", T.tube_start(1) + (0, 0, 0.05), T.TUBE_LYING_R)
    assert e.move("tube_1", T.tube_start(1), T.TUBE_LYING_R, seconds=1.0)[0] == 0.0
    assert e.step(right=_home("right"))[0] == 0.0


def test_a_filled_tube_held_over_the_tray_or_standing_is_not_back():
    e = Episode()
    e.fill("tube_1")
    e.move("tube_1", T.tube_start(1) + (0, 0, 0.02), T.TUBE_LYING_R)
    tubes = dict(e.tubes)
    assert e.step(right=_home("right"))[0] == pytest.approx(0.2)  # 2 cm over the rests: not resting
    up = T.tube_start(1) + (0, 0, 0.06)
    e.tubes = {**tubes, "tube_1": (up, np.eye(3))}
    assert e.step()[0] == pytest.approx(0.2)  # standing on its end


def test_a_filled_tube_left_beside_the_tray_is_not_back():
    e = Episode()
    e.fill("tube_1")
    e.move("tube_1", T.tube_start(1) + (0.0, 0.25, 0.0), T.TUBE_LYING_R)
    assert e.step(right=_home("right"))[0] == pytest.approx(0.2)


def test_one_tube_filled_twice_is_not_both():
    e = Episode()
    e.fill("tube_1")
    e.move("tube_1", *_on_needle(0.060))
    assert e.fill("tube_1")[0] == pytest.approx(0.2)


def test_full_course_succeeds():
    e = Episode()
    e.full_course()
    assert e.done() == [True] * 4
    assert e.step(0.5) == (0.8, False)  # not yet still for 1 s
    assert e.step(0.6) == (1.0, True)


def test_success_needs_every_condition_held_for_one_second_in_the_task_file():
    assert T.SUCCESS_HOLD_S == 0.0 and T.STILL_S == 1.0
    e = Episode()
    e.full_course()  # the last step: the left hand lets go of the holder, every condition now holds
    assert e.step(T.STILL_S - 0.05) == (0.8, False)  # held 0.95 s
    assert e.step(0.05) == (1.0, True)  # held 1.0 s


def test_a_lapse_restarts_the_second():
    e = Episode()
    e.full_course()
    assert e.step(0.9) == (0.8, False)
    near = _gripper(T.tube_start(2) - (0.0, 0.02, -0.03), np.array([0.0, 0.0, -1.0]))
    assert e.step(right=near)[1] is False  # a gripper back near tube 2: not released
    assert e.step(right=_home("right"))[1] is False  # released again: the second starts over
    assert e.step(T.STILL_S - 0.1) == (0.8, False)  # 1.9 s since the conditions first held, 0.9 s unbroken
    assert e.step(0.1) == (1.0, True)


def test_a_part_that_moves_during_the_second_restarts_it():
    e = Episode()
    e.full_course()
    assert e.step(0.8) == (0.8, False)
    pos, rot = e.tubes["tube_1"]
    e.tubes["tube_1"] = (pos + (0.0025, 0.0, 0.0), rot)  # 2.5 mm: still in the tray, but it moved
    assert e.step(T.STILL_S) == (0.8, False)  # the second restarts at the first step at the new place: 0.95 s held
    assert e.step(0.05) == (1.0, True)


def test_holding_the_holder_at_the_end_is_not_success():
    e = Episode()
    e.full_course(release_holder=False)
    assert e.step(2.0) == (0.8, False)


def test_a_gripper_near_a_tube_at_the_end_is_not_success():
    e = Episode()
    e.full_course()
    near = _gripper(T.tube_start(2) - (0.0, 0.02, -0.03), np.array([0.0, 0.0, -1.0]))
    assert e.step(2.0, right=near) == (0.6, False)  # tube 2 is not let go: its tray step is not done either


def test_a_tube_still_moving_is_not_success():
    e = Episode()
    e.full_course()
    for i in range(30):
        pos, rot = e.tubes["tube_2"]
        e.tubes["tube_2"] = (pos + (0.003 * (-1) ** i, 0.0, 0.0), rot)  # rocking 3 mm
        assert e.step()[1] is False


def test_needle_pushed_in_5_mm_scores_zero_even_if_it_comes_back():
    e = Episode()
    e.slide = 0.004  # the holder pushed 4 mm deeper: still allowed
    e.fill("tube_1", slide=0.004)
    assert e.step(slide=0.004)[0] == pytest.approx(0.2)
    assert e.step(slide=0.0052)[0] == 0.0
    assert e.step(slide=0.0)[0] == 0.0
    e.back_to_tray("tube_1", 1)
    assert e.step(1.0)[0] == 0.0


def test_needle_pulled_out_or_levered_scores_zero():
    e = Episode()
    e.fill("tube_1")
    assert e.step(slide=-0.0055)[0] == 0.0
    T._PROGRESS.__init__()
    e = Episode()
    e.fill("tube_1")
    assert e.step(pivot=math.radians(5.5))[0] == 0.0


def test_tube_off_the_table_scores_zero():
    e = Episode()
    e.fill("tube_1")
    assert e.move("tube_1", (0.3, -0.7, T.TOP - 0.3), np.eye(3), held=False)[0] == 0.0


def test_not_finite_scores_zero():
    e = Episode()
    assert e.step(slide=float("nan"))[0] == 0.0


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_is_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 3.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 2.0, plan.worst_ik_deg


def test_demo_timing():
    d = T.demo()
    left = [w.label for w in d["left"]]
    right = [w.label for w in d["right"]]
    held = sum(w.duration for w in d["left"][: left.index("keep it steady")])
    first_push = sum(w.duration for w in d["right"][: right.index("into the holder")])
    let_go = sum(w.duration for w in d["left"][: left.index("let go")])
    done = sum(w.duration for w in d["right"][:-1])
    assert held <= first_push and let_go >= done - 1e-9
    fills = [w.duration for w in d["right"] if w.label == "hold it while it fills"]
    assert len(fills) == 2 and min(fills) > T.FILL_S
    assert max(sum(w.duration for w in d[s]) for s in ("left", "right")) < 2400 / 20 - 5.0
