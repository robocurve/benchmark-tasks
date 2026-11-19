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
        st["glass_bulb"] = box("glass_bulb", M.GLASS_DST[0], 0.35)
        st["glass_stem"] = box("glass_stem", M.GLASS_DST[1], -0.40)
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


def test_bead_off_the_side_of_the_mat_latches_zero():
    st = R.initial_states(M)
    st["bead4"] = bead(4, (M.SPILL[0], M.MAT_XY[1] + M.MAT[1] / 2 + 0.005))
    step(st)
    assert M.TRIAL.mem["violated"] and "rolled away" in M.TRIAL.mem["why"]


def test_sponge_on_a_bead_latches_zero():
    st = R.initial_states(M)
    st["sponge"] = box("sponge", M._bead_xy(1) + np.array([0.0, 0.03]))
    step(st)
    assert M.TRIAL.mem["violated"] and "sponge" in M.TRIAL.mem["why"]


def test_sponge_over_the_mat_in_the_air_latches_zero():
    st = R.initial_states(M)
    sp = box("sponge", (M.SPILL[0] + 0.08, M.SPILL[1]))
    st["sponge"] = R.state(sp.pos + np.array([0.0, 0.0, 0.10]), half=(sp.high - sp.low) / 2)
    step(st)
    assert M.TRIAL.mem["violated"] and M.TRIAL.mem["sponge_over_mat"]


def test_sponge_at_its_mark_is_clear_of_the_mat():
    spec = next(o for o in M.OBJECTS if o.name == "sponge")
    assert not M.on_mat(spec.pos)
    assert abs(spec.pos[1] - M.MAT_XY[1]) - spec.size[1] / 2 - M.MAT[1] / 2 > 0.02  # 2 cm clear of the mat's edge


def test_bead_moved_without_the_squeegee_latches_zero():
    st = R.initial_states(M)
    step(st)  # each bead's resting mark
    st["bead3"] = bead(3, M._bead_xy(3) + np.array([-0.02, 0.0]))  # pinched or nudged 2 cm, squeegee on its mark
    step(st)
    assert M.TRIAL.mem["violated"] and "other than the squeegee" in M.TRIAL.mem["why"]


def test_small_jiggle_without_the_squeegee_is_fine():
    st = R.initial_states(M)
    step(st)
    st["bead3"] = bead(3, M._bead_xy(3) + np.array([-0.008, 0.0]))
    step(st)
    assert not M.TRIAL.mem["violated"]


def test_bead_pushed_by_the_squeegee_is_fine():
    st = R.initial_states(M)
    step(st)
    xy = M._bead_xy(0) + np.array([-0.06, 0.0])
    st["bead0"] = bead(0, xy, vel=(-0.03, 0.0, 0.0))
    st["squeegee"] = box("squeegee", (xy[0] + 0.02, M.SPILL[1]))  # the blade 2 cm behind it
    step(st)
    assert not M.TRIAL.mem["violated"]
    st["squeegee"] = box("squeegee", M.SQ_XY)  # squeegee taken away; the bead stays where it was pushed
    step(st)
    assert not M.TRIAL.mem["violated"]


def test_squeegee_gap_is_seen_from_above():
    st = R.initial_states(M)
    sq = box("squeegee", (0.30, M.SPILL[1]))
    st["squeegee"] = R.state(sq.pos + np.array([0.0, 0.0, 0.15]), half=(sq.high - sq.low) / 2)  # lifted high
    assert M.squeegee_gap(st, (0.30, M.SPILL[1])) == 0.0
    assert M.squeegee_gap(st, (0.30 + M.SQ[0] / 2 + 0.03, M.SPILL[1])) == pytest.approx(0.03)


def test_mat_covers_spill_glass_and_towel_but_not_tools():
    for i in range(len(M.BEADS)):
        assert M.on_mat(M._bead_xy(i))
    for _, (_, xy, _) in M.GLASS.items():
        assert M.on_mat(xy)
    t = M.TOWEL_XY
    for sx in (-1, 1):
        for sy in (-1, 1):
            assert M.on_mat((t[0] + sx * M.TOWEL[0] / 2, t[1] + sy * M.TOWEL[1] / 2))
    assert not M.on_mat(M.SQ_XY)


def test_thermometer_is_drawn_with_bulb_scale_and_column():
    for o in M.OBJECTS:
        if o.name not in M.GLASS:
            continue
        names = [t[0] for t in o.trim]
        assert "scale_enamel" in names and "hg_column" in names and sum(n.startswith("tick") for n in names) >= 5
        assert ("bulb_hg" in names) == (o.name == "glass_bulb")
        assert not np.any(R.hhperson.local_centre(o))  # every part inside the collider (bulb only wider, symmetrically)


def test_task_json_within_submission_limits():
    import json
    d = json.loads((HERE / "task.json").read_text())
    assert len(d["title"]) <= 120 and len(d["fidelity"]) <= 500 and len(d["scores"]) <= 300 and len(d["context"]) <= 1000
    assert d["instruction"] == M.INSTRUCTION


def test_plans_reachable():
    for name in M.PLANS:
        for side, ws in M.PLANS[name]().items():
            assert R.reachable(side, ws), (name, side)


def test_demo_stroke_is_slow_and_flick_is_fast():
    v = (M.STROKE[0] - M.STROKE[1]) / M.STROKE_S
    assert v < M.SLOW_MS / 4
    assert (M.STROKE[0] - M.STROKE[1]) / 0.4 > 2 * M.SLOW_MS
