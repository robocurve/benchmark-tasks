"""Checks of gear_train.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_gear_train.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("gear_train.py"))
M = TASK.module
GEARS = M.GEARS
THICK = 0.025


def gear(name, x, y, bottom=None, yaw_deg=0.0, tilt_deg=0.0, moving=False):
    """A gear centred at (x, y), its bottom at ``bottom`` (default: lying on the table), tilted about x."""
    a, b = math.radians(yaw_deg) / 2, math.radians(tilt_deg) / 2
    q = np.array([math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.sin(a) * math.cos(b),
                  math.cos(a) * math.cos(b)])
    r = M.DIAMETER[name] / 2
    low_z = M.TOP + 0.001 if bottom is None else bottom
    centre = np.array([x, y, low_z + THICK / 2])
    half = np.array([r, r, THICK / 2])
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=centre - half, high=centre + half)


def start():
    return {n: gear(n, *M.START[n]) for n in GEARS}


def seated_gear(name, **kw):
    return gear(name, *M.shaft(name), bottom=M.SEAT, **kw)


def layout(**where):
    o = start()
    o.update(where)
    return o


ALL_SEATED = {n: seated_gear(n) for n in GEARS}


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0 and not M.success(o)


def test_no_gear_starts_nearest_its_own_shaft():
    for n in GEARS:
        d = {s: np.linalg.norm(np.array(M.START[n]) - M.shaft(s)) for s in GEARS}
        assert min(d, key=d.get) != n


def test_all_seated_succeeds():
    assert M.success(ALL_SEATED) and M.score(ALL_SEATED) == 1.0


@pytest.mark.parametrize("name", GEARS)
def test_one_seated_gear_scores_0_1(name):
    assert M.score(layout(**{name: seated_gear(name)})) == pytest.approx(0.1)


def test_large_and_small_seated_without_the_medium_have_no_mesh():
    # the large and small shafts are too far apart to mesh: two gears down, no phase matched
    o = layout(large_gear=seated_gear("large_gear"), small_gear=seated_gear("small_gear"))
    assert M.score(o) == pytest.approx(0.2)


@pytest.mark.parametrize("pair", M.PAIRS)
def test_one_meshed_pair_scores_0_5(pair):
    assert M.score(layout(**{n: seated_gear(n) for n in pair})) == pytest.approx(0.5)


@pytest.mark.parametrize("name", GEARS)
@pytest.mark.parametrize("proud_mm, credit", [(-1.1, 0.0), (0.0, 0.1), (0.9, 0.1), (-0.9, 0.1), (1.1, 0.0),
                                              (8.0, 0.0), (19.0, 0.0)])
def test_height_on_the_shaft(name, proud_mm, credit):
    """Seated within 1 mm either way; stopped on teeth (7 to 10 mm proud) or resting on the tip (about 19 mm proud)
    scores nothing."""
    g = gear(name, *M.shaft(name), bottom=M.SEAT + proud_mm / 1000)
    assert M.score(layout(**{name: g})) == pytest.approx(credit)


@pytest.mark.parametrize("off_mm, credit", [(1.9, 0.1), (2.1, 0.0)])
def test_off_axis(off_mm, credit):
    x, y = M.shaft("medium_gear")
    g = gear("medium_gear", x + off_mm / 1000, y, bottom=M.SEAT)
    assert M.score(layout(medium_gear=g)) == pytest.approx(credit)


def test_a_gear_on_the_wrong_shaft_scores_nothing():
    o = layout(large_gear=gear("large_gear", *M.shaft("small_gear"), bottom=M.SEAT),
               small_gear=gear("small_gear", *M.shaft("large_gear"), bottom=M.SEAT))
    assert M.score(o) == 0


@pytest.mark.parametrize("tilt, credit", [(2.5, 0.1), (3.5, 0.0)])
def test_a_tilted_gear_is_not_seated(tilt, credit):
    assert M.score(layout(small_gear=seated_gear("small_gear", tilt_deg=tilt))) == pytest.approx(credit)


def test_yaw_does_not_matter():
    assert M.success({n: seated_gear(n, yaw_deg=37.0) for n in GEARS})


def test_success_needs_every_gear_still():
    o = dict(ALL_SEATED, small_gear=seated_gear("small_gear", moving=True))
    assert not M.success(o) and M.score(o) == 1.0


@pytest.mark.parametrize("jammed", GEARS)
def test_two_seated_one_jammed_is_not_success(jammed):
    o = dict(ALL_SEATED, **{jammed: gear(jammed, *M.shaft(jammed), bottom=M.SEAT + 0.008)})
    # jammed medium: two gears down, no mesh; jammed end gear: one mesh
    assert not M.success(o) and M.score(o) == pytest.approx(0.2 if jammed == "medium_gear" else 0.5)


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
    assert plan.duration < 2400 / 20
