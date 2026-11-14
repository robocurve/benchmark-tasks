"""Offline checks of task.py (no simulator): python -m pytest -q test_task.py"""

import pathlib
import sys

import numpy as np
import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import load  # noqa: E402

import hhreplay as R  # noqa: E402

TASK = load(pathlib.Path(__file__).with_name("task.py"))
M = TASK.module
TOP = M.TOP
NONE = np.zeros((0, 3))
SETTLE = int(M.SETTLE_S * R.HZ) + 1


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override = R.PARKED, NONE
    yield


def scene(**over):
    s = R.initial_states(M)
    s.update(over)
    return s


def spare_at(x, y, z):
    return R.state((x, y, z), half=np.asarray(M.BLOCK) / 2)


ON_TRAY = spare_at(*M.TRAY, M.TRAY_TOP + M.BLOCK[2] / 2)


def frames_then(final, n=60, grips=None):
    return [(scene(), R.PARKED, NONE)] * SETTLE + [(final, grips or R.PARKED, NONE)] * n


def reach(side, xy, z):
    return R.reachable(side, [Waypoint(1.5, (*xy, z + 0.08), FINGER_OPEN, "over"),
                              Waypoint(1.5, (*xy, z), FINGER_OPEN, "down")])


def test_layout():
    assert set(TASK.free_objects) == {*M.CRIB, "casing", "spare_block"}
    assert len(M.CRIB) == 8


def test_the_spare_needs_both_arms():
    assert reach("right", M.SPARE_XY, M.GRIP_Z) and not reach("left", M.SPARE_XY, M.GRIP_Z)
    tray_z = M.TRAY_TOP + M.BLOCK[2] / 2
    assert reach("left", M.TRAY, tray_z) and not reach("right", M.TRAY, tray_z)


def test_a_crib_block_is_within_the_left_arms_reach():
    (x, y, _z), _s = M.CRIB["crib_1a"]
    assert reach("left", (x, y), M.GRIP_Z)


def test_null_policy_scores_zero():
    assert R.run(M, M.TRIAL, frames_then(scene(), 2400)) == 0.0


def test_spare_on_tray_scores_one_and_lifted_scores_a_quarter():
    assert R.run(M, M.TRIAL, frames_then(scene(spare_block=ON_TRAY))) == 1.0
    M.TRIAL.clear()
    up = spare_at(*M.SPARE_XY, TOP + 0.08)
    assert R.run(M, M.TRIAL, frames_then(scene(spare_block=up))) == 0.25


def test_a_held_spare_on_the_tray_isnt_done():
    g = R.gripper(ON_TRAY.pos + [0, 0, 0.005])
    up = [(scene(spare_block=spare_at(*M.TRAY, TOP + 0.08)), R.PARKED, NONE)]
    frames = frames_then(scene(spare_block=ON_TRAY), grips=[g, R.PARKED[1]])
    assert R.run(M, M.TRIAL, frames[:SETTLE] + up + frames[SETTLE:]) == 0.25


@pytest.mark.parametrize(("name", "d", "bad"), [("casing", 0.004, False), ("casing", 0.006, True),
                                                ("crib_2b", 0.006, True), ("crib_1a", 0.004, False)])
def test_the_5mm_budget(name, d, bad):
    s = scene(spare_block=ON_TRAY)
    o = s[name]
    s[name] = R.state(o.pos + [d, 0, 0], half=(o.high - o.low) / 2)
    assert R.run(M, M.TRIAL, frames_then(s)) == (0.0 if bad else 1.0)


def test_the_demo_replay_scores_one():
    frames = R.replay(M, M.demo(), links=True, extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 1.0 and not M.TRIAL.mem["violated"]


def test_the_demo_stays_clear_of_the_crib_and_casing():
    # Link frame origins, not link surfaces: the wrist and gripper body extend about 4.5 cm around them. A first
    # layout with 4.4 cm here had the left wrist knock the casing in the simulator.
    frames = R.replay(M, M.demo(), links=True)
    spec = {o.name: o for o in M.OBJECTS}
    boxes = [*M.CRIB.values(), (spec["casing"].pos, M.CASING)]
    gaps = [min(R.hhkit.point_box(p, c, s) for c, s in boxes for p in pts) for _o, _g, pts in frames]
    # The left arm's first move, down from its raised start pose, passes 4 cm above the casing (clear in sim).
    start = int(6.0 * R.HZ)
    assert min(gaps[:start]) > 0.04 and min(gaps[start:]) > 0.07


def test_taking_a_crib_block_scores_zero():
    frames = R.replay(M, M.crib_block_demo(), extra_s=2.0)
    assert R.run(M, M.TRIAL, frames) == 0.0
    assert "crib_1a" in M.TRIAL.mem["why"]
