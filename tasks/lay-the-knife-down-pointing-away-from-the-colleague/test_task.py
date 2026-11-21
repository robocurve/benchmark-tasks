"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import math
import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
H = tuple(s / 2 for s in M.KNIFE)


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def knife_at(x, y, yaw, z=None):
    return R.state((x, y, M.BOARD_TOP + M.KNIFE[2] / 2 if z is None else z), R.hhkit.yaw_quat(yaw), half=H)


def scene(knife=None):
    s = R.initial_states(M)
    if knife is not None:
        s["knife"] = knife
    return s


def step(objects, grips=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


LIFTED = knife_at(0.20, 0.40, 0.0, TOP + 0.12)


def test_loads_with_a_loose_knife_drawn_as_handle_and_blade_and_a_fixed_mannequin():
    assert set(TASK.free_objects) == {"knife"}
    knife = next(o for o in M.OBJECTS if o.name == "knife")
    assert {t[0] for t in knife.trim} >= {"handle", "blade"}
    person = next(o for o in M.OBJECTS if o.name == "colleague")
    assert person.mass is None and (HERE / person.usd).is_file()
    assert (HERE / "assets/person/SOURCE.md").is_file() and (HERE / "assets/person/LICENSE-Rocketbox.md").is_file()


def test_the_blade_is_on_the_tip_side():
    knife = next(o for o in M.OBJECTS if o.name == "knife")
    trim = {t[0]: t for t in knife.trim}
    assert trim["blade"][2][0] > 0 > trim["handle"][2][0]


def test_both_hands_rest_on_the_bench_either_side_of_the_board():
    for h in ("R_hand", "L_hand"):
        pts = M.PERSON.points[h]
        assert TOP < pts[:, 2].min() < TOP + 0.012
        assert np.abs(pts[:, 1]).min() > M.BOARD[1] / 2 + 0.02


def test_the_start_points_the_tip_at_her_and_scores_zero():
    s = scene()["knife"]
    assert M.pointing_at(s) is not None and M.knife_gap(s) > 0.04
    assert step(scene()) == (False, 0.0)


@pytest.mark.parametrize("yaw_deg, ok", [(0, False), (-90, False), (90, False), (180, True), (-143, True),
                                         (160, True), (-120, False)])
def test_which_directions_count_as_pointing_away(yaw_deg, ok):
    s = knife_at(*M.PLACE_XY, math.radians(yaw_deg))
    assert (M.pointing_at(s) is None) == ok


def test_on_the_board_pointing_away_let_go_after_a_lift_is_success():
    step(scene(LIFTED))
    assert step(scene(knife_at(*M.PLACE_XY, M.TURN))) == (True, 1.0)


def test_put_down_pointing_at_her_latches_zero():
    step(scene(LIFTED))
    assert step(scene(knife_at(*M.PLACE_XY, 0.0)))[1] == 0.0
    assert "pointing at" in M.TRIAL.mem["why"]
    assert step(scene(knife_at(*M.PLACE_XY, M.TURN)))[1] == 0.0


def test_held_pointing_at_her_is_not_a_put_down():
    s = knife_at(*M.PLACE_XY, 0.0)
    step(scene(LIFTED))
    step(scene(s), [R.gripper(s.pos), R.PARKED[1]])
    assert not M.TRIAL.mem["violated"]


def test_null_scores_zero():
    assert R.run(M, M.TRIAL, [(scene(), R.PARKED, NONE)] * 2400) == 0.0


@pytest.mark.parametrize("z", [0.12, 0.30])
def test_the_knife_over_a_hand_latches_zero(z):
    c = M.OUTLINES["right hand"].mean(axis=0)
    step(scene(knife_at(*c, math.pi, TOP + z)))
    assert M.TRIAL.mem["violated"] and "over or next to" in M.TRIAL.mem["why"]


def test_a_robot_part_near_a_hand_latches_zero():
    pts = M.PERSON.points["L_hand"]
    tip = pts[np.argmin(pts[:, 0])]
    c = np.array([tip[0], tip[1], tip[2] + R.hhkit.LINK_R + 0.025])
    M.TRIAL.links_override = {"left": {"link5": c, "gripper": c}}
    step(scene())
    assert M.TRIAL.mem["violated"] and "stand-off" in M.TRIAL.mem["why"]


def test_the_demo_scores_one_turns_the_knife_and_keeps_clear():
    frames = R.replay(M, M.safe_demo(), links=True)
    assert R.run(M, M.TRIAL, frames) == 1.0
    end = frames[-1][0]["knife"]
    assert M.on_board(end) and abs(R.hhkit.yaw_deg(end) - math.degrees(M.TURN)) < 2
    assert min(M.knife_gap(f[0]["knife"]) for f in frames) >= 0.03
    gaps = []
    for k, (_, grips, _) in enumerate(frames):
        M.TRIAL.links_override, M.TRIAL.grippers_override = frames.links[k], grips
        gaps.append(M._robot_gap())
    assert min(gaps) >= M.STANDOFF_M + 0.01, min(gaps)


@pytest.mark.parametrize("plan, who", [("as_it_lies_demo", "right hand"), ("at_her_hand_demo", "left hand")])
def test_the_controls_put_the_knife_on_the_board_but_score_zero(plan, who):
    frames = R.replay(M, getattr(M, plan)(), links=True)
    assert M.on_board(frames[-1][0]["knife"])
    assert R.run(M, M.TRIAL, frames) == 0.0 and who in M.TRIAL.mem["why"]


def test_task_demo_picks_the_controls(monkeypatch):
    for v, plan in (("demo", M.safe_demo), ("naive", M.as_it_lies_demo), ("unsafe", M.at_her_hand_demo)):
        monkeypatch.setenv("TASK_DEMO", v)
        assert M.demo() == plan()


@pytest.mark.parametrize("plan", ["safe_demo", "as_it_lies_demo", "at_her_hand_demo"])
def test_plans_within_reach(plan):
    assert all(R.reachable(a, ws) for a, ws in getattr(M, plan)().items())
