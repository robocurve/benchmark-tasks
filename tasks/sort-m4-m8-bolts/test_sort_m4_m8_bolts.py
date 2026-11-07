"""Offline checks (no GPU, no network): the task file loads, the scorer covers each outcome, the looks fit their
boxes, and the demo's poses are reachable. Run in the yam_arena environment:
python -m pytest -q test_sort_m4_m8_bolts.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "sort_m4_m8_bolts.py")
T = TASK.module
SPECS = {b.name: b for b in T.BOLTS}


def _state(pos, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.asarray(vel, float),
                       ang_vel=np.asarray(spin, float), low=pos - 0.01, high=pos + 0.01)


def _in(side, slot=0, name="m8_1"):
    """A centre lying on the floor of compartment ``side``, at grid spot ``slot``."""
    dx, dy = T.SLOTS[slot]
    return (T.ORG_XY[0] + dx, T.CELL_Y[side] + dy, T.TOP + T.FLOOR + SPECS[name].s / 2)


def _start(**moved):
    out = {o.name: _state(o.pos) for o in T.OBJECTS}
    out.update({k: v if isinstance(v, ObjectState) else _state(v) for k, v in moved.items()})
    return out


def _sorted(**moved):
    used = {"left": 0, "right": 0}
    placed = {}
    for name, side in T.TARGET.items():
        placed[name] = _in(side, used[side], name)
        used[side] += 1
    placed.update(moved)
    return _start(**placed)


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == set(T.TARGET)
    assert len(T.M8) == len(T.M4) == 4


def test_start_scores_zero():
    assert T.score(_start()) == 0.0
    assert not T.success(_start())


def test_all_sorted_succeeds():
    objects = _sorted()
    assert T.success(objects) and T.score(objects) == 1.0


def test_each_right_bolt_is_an_eighth():
    objects = _start(m8_1=_in("left", 0), m4_1=_in("right", 0, "m4_1"), m4_2=_in("right", 1, "m4_2"))
    assert T.score(objects) == pytest.approx(3 / 8)
    assert not T.success(objects)


def test_wrong_compartment_takes_an_eighth_off():
    objects = _start(m8_1=_in("left", 0), m8_2=_in("left", 1), m4_1=_in("left", 2, "m4_1"))
    assert T.score(objects) == pytest.approx(1 / 8)
    assert T.score(_sorted(m8_1=_in("right", 3))) == pytest.approx(6 / 8)


def test_all_in_one_compartment_scores_zero():
    objects = _start(**{n: _in("left", i % 4, n) for i, n in enumerate(T.TARGET)})
    assert T.score(objects) == 0.0
    objects = _start(**{n: _in("right", i % 4, n) for i, n in enumerate(T.TARGET)})
    assert T.score(objects) == 0.0


def test_swapped_scores_zero():
    swapped = {n: _in("right" if s == "left" else "left", i % 4, n) for i, (n, s) in enumerate(T.TARGET.items())}
    assert T.score(_start(**swapped)) == 0.0


def test_moving_bolt_does_not_count():
    moving = _state(_in("left", 0), vel=(0.05, 0.0, 0.0))
    assert T.score(_sorted(m8_1=moving)) == pytest.approx(7 / 8)
    spinning = _state(_in("left", 0), spin=(0.0, 0.0, 2.0))
    assert T.score(_sorted(m8_1=spinning)) == pytest.approx(7 / 8)


def test_held_above_walls_or_on_divider_is_not_in():
    above = (T.ORG_XY[0], T.CELL_Y["left"], T.TOP + T.ORG_H + 0.01)
    divider = (T.ORG_XY[0], T.ORG_XY[1], T.TOP + T.ORG_H + 0.006)
    outside = (T.ORG_XY[0] - T.CELL / 2 - 0.01, T.CELL_Y["left"], T.TOP + 0.007)
    for pos in (above, divider, outside):
        assert T.score(_sorted(m8_1=pos)) == pytest.approx(7 / 8)


def test_bolt_off_table_scores_zero():
    assert T.score(_sorted(m4_3=(0.4, 0.7, 0.3))) == 0.0
    assert T.score(_sorted(m4_3=(0.3, -0.2, T.TOP - 0.05))) == 0.0


def test_non_finite_scores_zero():
    assert T.score(_sorted(m8_2=(np.nan, 0.0, T.TOP))) == 0.0


def test_bolts_rest_on_the_mat_apart():
    names = list(T.LAYOUT)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            assert math.dist(T.LAYOUT[a][:2], T.LAYOUT[b][:2]) > 0.048, (a, b)
        x, y, _ = T.LAYOUT[a]
        assert abs(x - T.MAT_XY[0]) < T.MAT_SIZE[0] / 2 - 0.02 and abs(y - T.MAT_XY[1]) < T.MAT_SIZE[1] / 2 - 0.02


def test_slots_fit_the_compartment():
    """Each bolt let go at its spot lies inside the compartment, and the open fingers (24 mm gap, 7 mm thick) clear
    the walls and the bolt let go before it."""
    for dx, dy in T.SLOTS:
        for kind, s in T.SIZES.items():
            half_len, half_w = (s["k"] + s["l"]) / 2, s["e"] / 2
            assert abs(dx) + half_len < T.CELL / 2 - 0.003
            assert abs(dy) + half_w < T.CELL / 2 - 0.003
        assert abs(dy) + abs(T.OPEN_PLACE) + 0.007 < T.CELL / 2


def test_bolt_colliders_fill_the_box():
    for b in T.BOLTS:
        parts = dict((n, (c, s)) for n, c, s in b.parts())
        (hc, hs), (sc, ss) = parts["head"], parts["shank"]
        assert hc[0] - hs[0] / 2 == pytest.approx(-b.size[0] / 2)
        assert sc[0] + ss[0] / 2 == pytest.approx(b.size[0] / 2)
        assert hs[2] == pytest.approx(b.size[2]) and ss[1] < hs[1] <= b.size[1]


def _usd_bounds(path):
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(stage.GetPseudoRoot())
    r = box.ComputeAlignedRange()
    return np.array(r.GetMin()), np.array(r.GetMax())


@pytest.mark.parametrize("kind", ["m8", "m4"])
def test_bolt_look_fits_its_box(kind):
    pytest.importorskip("pxr")
    lo, hi = _usd_bounds(HERE / "looks" / f"{kind}.usdc")
    half = np.array(next(b for b in T.BOLTS if b.name.startswith(kind)).size) / 2
    assert np.all(lo >= -half - 1e-5) and np.all(hi <= half + 1e-5), (lo, hi, half)


def test_demo_is_reachable():
    demo = T.demo()
    assert set(demo) == {"right"}
    plan = ArmPlan(demo["right"], BASES["right"])
    t = 0.0
    while t < plan.duration + 0.5:
        plan.action7(t)
        t += 0.05
    assert plan.worst_ik_mm < 2.0, plan.worst_ik_mm
    assert plan.worst_ik_deg < 2.0, plan.worst_ik_deg
    assert plan.duration * 20 < 1400


def test_demo_stays_in_the_right_half_of_reach():
    """The right arm's TCP never goes past y = 0.08 (the left arm holds still at READY, 22 cm farther left)."""
    for w in T.demo()["right"]:
        assert w.pos[1] < 0.08
