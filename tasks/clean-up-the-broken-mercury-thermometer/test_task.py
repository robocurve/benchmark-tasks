"""Offline checks of task.py's scorer (no simulator): python -m pytest -q test_task.py

The beads' rolling cannot be replayed kinematically, so the scripted plans are checked in Isaac Sim (see task.json
"scores"); here the scorer is driven with hand-made object states."""

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
NONE = np.zeros((0, 3))


@pytest.fixture(autouse=True)
def fresh():
    M.TRIAL.clear()
    M.TRIAL.hz = R.HZ
    M.TRIAL.grippers_override, M.TRIAL.points_override, M.TRIAL.links_override = R.PARKED, NONE, None
    yield


def bead(i, xy=None, z=None, vel=(0.0, 0.0, 0.0)):
    r = M.BEADS[i][1]
    xy = M._bead_xy(i) if xy is None else np.asarray(xy, float)
    return R.state(np.array([xy[0], xy[1], M.TOP + r if z is None else z]), half=(r, r, r), vel=vel)


def box(name, xy, yaw=0.0):
    spec = next(o for o in M.OBJECTS if o.name == name)
    return R.state(np.array([xy[0], xy[1], M.TOP + spec.size[2] / 2]), R.hhkit.yaw_quat(yaw),
                   half=tuple(s / 2 for s in spec.size))


def gathered(n=5, glass=True, tool_away=True):
    st = R.initial_states(M)
    ys = np.linspace(-0.04, 0.04, 5)
    for i in range(n):
        st[f"bead{i}"] = bead(i, (0.19, M.TOWEL_XY[1] + ys[i]))
    if glass:
        st["glass_bulb"] = box("glass_bulb", (0.135, -0.085), 0.35)
        st["glass_stem"] = box("glass_stem", (0.135, -0.195), -0.40)
    if not tool_away:
        st["squeegee"] = box("squeegee", (0.20, -0.14))
    return st


def step(objects, grips=None, points=None):
    if grips is not None:
        M.TRIAL.grippers_override = grips
    if points is not None:
        M.TRIAL.points_override = points
    M.TRIAL.new_step()
    ok = M.success(objects)
    M.distance(objects)
    return ok, M.score(objects)


def test_loads():
    assert TASK.scene == "station"
    names = [o.name for o in M.OBJECTS]
    assert len(names) == len(set(names))
    assert set(TASK.free_objects) == set(M.BEAD_NAMES) | set(M.GLASS) | {"squeegee", "sponge"}


def test_beads_are_mercury_sized_and_heavy():
    for (_, r), o in zip(M.BEADS, [o for o in M.OBJECTS if o.name.startswith("bead")]):
        assert 0.0030 <= r <= 0.0050  # 6-10 mm across: visible at 320 x 176
        assert o.mass == pytest.approx(13530 * 4 / 3 * np.pi * r ** 3)


def test_start_scores_zero_and_no_bead_on_towel():
    assert step(R.initial_states(M)) == (False, 0.0)
    assert M.TRIAL.mem["on"] == 0 and not M.TRIAL.mem["violated"]


def test_all_gathered_is_full():
    assert step(gathered()) == (True, 1.0)


def test_squeegee_left_on_the_towel_is_not_full():
    assert step(gathered(tool_away=False))[1] == 0.5


def test_four_beads_is_half_and_glass_only_is_quarter():
    assert step(gathered(n=4))[1] == 0.5
    M.TRIAL.clear()
    assert step(gathered(n=0))[1] == 0.25


def test_beads_without_glass_is_quarter():
    assert step(gathered(glass=False))[1] == 0.25


def test_flicked_bead_latches_zero():
    st = R.initial_states(M)
    st["bead0"] = bead(0, vel=(-0.4, 0.0, 0.0))
    step(st)
    assert M.TRIAL.mem["violated"] and "flicked" in M.TRIAL.mem["why"]
    assert step(gathered()) == (False, 0.0)


def test_slow_push_is_fine():
    st = R.initial_states(M)
    st["bead0"] = bead(0, vel=(-0.03, 0.0, 0.0))
    step(st)
    assert not M.TRIAL.mem["violated"]


def test_bead_rolled_away_latches_zero():
    st = R.initial_states(M)
    st["bead2"] = bead(2, (M.SPILL[0] + 0.25, M.SPILL[1]))
    step(st)
    assert M.TRIAL.mem["violated"] and "rolled away" in M.TRIAL.mem["why"]


def test_sponge_on_a_bead_latches_zero():
    st = R.initial_states(M)
    st["sponge"] = box("sponge", M._bead_xy(1) + np.array([0.0, 0.03]))
    step(st)
    assert M.TRIAL.mem["violated"] and "sponge" in M.TRIAL.mem["why"]


def test_finger_on_a_bead_latches_zero():
    p = np.array([[*M._bead_xy(3), M.TOP + 2 * M.BEADS[3][1] + 0.0005]])
    step(R.initial_states(M), points=p)
    assert M.TRIAL.mem["violated"] and "pressed" in M.TRIAL.mem["why"]


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


def test_demo_stroke_is_slow_and_flick_is_fast():
    v = (M.STROKE[0] - M.STROKE[1]) / M.STROKE_S
    assert v < M.SLOW_MS / 4
    assert (M.STROKE[0] - M.STROKE[1]) / 0.4 > 2 * M.SLOW_MS
