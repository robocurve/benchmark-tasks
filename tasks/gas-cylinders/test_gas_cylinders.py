"""Checks of gas_cylinders.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_gas_cylinders.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("gas_cylinders.py"))
M = TASK.module
FT, R, H = M.FT, M.RADIUS, M.HEIGHT


def cyl(x, y, lift_mm=0.0, tilt_deg=0.0, moving=False):
    """A cylinder standing at (x, y) (or ``lift_mm`` above the table), tilted ``tilt_deg`` about x."""
    t = math.radians(tilt_deg)
    half = np.array([R, R * math.cos(t) + H / 2 * math.sin(t), H / 2 * math.cos(t) + R * math.sin(t)])
    centre = np.array([x, y, M.TOP + half[2] + lift_mm / 1000])
    return ObjectState(pos=centre, quat_xyzw=np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)]),
                       lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def start():
    return {n: cyl(x, M.ROW_Y) for n, x in M.START.items()}


def placed(**where):
    o = start()
    o.update({n: cyl(*xy) if isinstance(xy, tuple) else xy for n, xy in where.items()})
    return o


BEHIND = M.BEHIND
FAR = (0.20, 0.25)  # more than 20 ft from both fuel cylinders, no wall between
LOW_X = 0.18  # beside the row, past the low wall
BEHIND_LOW = (LOW_X, -0.26)


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0 and not M.success(o)


def test_behind_the_tall_wall_succeeds():
    o = placed(oxygen=BEHIND)
    assert all(M.behind_wall(o["oxygen"], o[f]) == "tall_wall" for f in M.FUELS)
    assert M.score(o) == 1.0 and M.success(o)


def test_twenty_feet_away_succeeds():
    o = placed(oxygen=FAR)
    assert all(M.apart_ft(o["oxygen"], o[f]) > 20 for f in M.FUELS)
    assert M.score(o) == 1.0 and M.success(o)


def test_moving_the_fuel_away_works_too():
    o = placed(acetylene_a=BEHIND, acetylene_b=(BEHIND[0], -0.08))
    assert M.score(o) == 1.0


def test_behind_the_low_wall_does_not_count():
    o = placed(oxygen=BEHIND_LOW)
    assert all(M.behind_wall(o["oxygen"], o[f]) is None for f in M.FUELS)
    assert M.score(o) == 0


@pytest.mark.parametrize("ft, ok", [(19.8, False), (20.2, True)])
def test_the_twenty_feet(ft, ok):
    fa = M.START["acetylene_a"]
    o = placed(acetylene_b=(fa, M.ROW_Y - 0.03), oxygen=(fa, M.ROW_Y + ft * FT + 2 * R))
    assert M.apart_ft(o["oxygen"], o["acetylene_a"]) == pytest.approx(ft)
    assert M.separated(o, "acetylene_a") is ok


def test_each_fuel_scores_half():
    fa = M.START["acetylene_a"]
    o = placed(oxygen=(fa, M.ROW_Y + 21 * FT + 2 * R), acetylene_b=(fa + 0.05, M.ROW_Y + 0.05))
    assert M.separated(o, "acetylene_a") and not M.separated(o, "acetylene_b")
    assert M.score(o) == 0.5 and not M.success(o)


@pytest.mark.parametrize("y, ok", [(-0.20 + M.END_MARGIN + 0.002, True), (-0.20 + M.END_MARGIN - 0.002, False),
                                   (-0.20 - 0.01, False)])
def test_a_line_grazing_the_walls_end_does_not_count(y, ok):
    o = placed(oxygen=(0.345, y), acetylene_a=(0.27, y))
    assert (M.behind_wall(o["oxygen"], o["acetylene_a"]) is not None) is ok


@pytest.mark.parametrize("y, ok", [(-0.04 - M.END_MARGIN - 0.002, True), (-0.04 - M.END_MARGIN + 0.002, False)])
def test_a_line_grazing_the_walls_other_end_does_not_count(y, ok):
    o = placed(oxygen=(0.345, y), acetylene_a=(0.27, y))
    assert (M.behind_wall(o["oxygen"], o["acetylene_a"]) is not None) is ok


def test_both_on_the_same_side_of_the_wall_does_not_count():
    o = placed(oxygen=BEHIND, acetylene_b=(BEHIND[0], -0.06))
    assert M.behind_wall(o["oxygen"], o["acetylene_b"]) is None and M.score(o) == 0.5


@pytest.mark.parametrize("name", ["oxygen", "acetylene_a"])
def test_any_cylinder_down_scores_zero(name):
    o = placed(oxygen=BEHIND)
    x, y = (float(v) for v in o[name].pos[:2])
    o[name] = cyl(x, y, tilt_deg=90)
    assert M.score(o) == 0 and not M.success(o)


@pytest.mark.parametrize("tilt, ok", [(8, True), (12, False)])
def test_upright_within_ten_degrees(tilt, ok):
    o = placed(oxygen=FAR)
    o["oxygen"] = cyl(*FAR, tilt_deg=tilt)
    assert (M.score(o) == 1.0) is ok


def test_held_in_the_air_scores_zero():
    o = placed(oxygen=FAR)
    o["oxygen"] = cyl(*FAR, lift_mm=5)
    assert M.score(o) == 0


def test_moving_is_not_yet_success():
    o = placed(oxygen=FAR)
    o["oxygen"] = cyl(*FAR, moving=True)
    assert M.score(o) == 1.0 and not M.success(o)


def test_the_walls_heights():
    assert M.WALLS["tall_wall"][1] >= M.BARRIER_FT * FT > M.WALLS["low_wall"][1]
    sizes = {b.name: b.size for b in M.WALL_BOXES}
    assert sizes["tall_wall"] == pytest.approx((M.WALL_T, 0.16, 6 * FT))
    assert sizes["low_wall"] == pytest.approx((0.16, M.WALL_T, 3 * FT))


def test_the_demo_ends_behind_the_tall_wall():
    o = placed(oxygen=M.BEHIND)
    assert M.success(o)
    assert BEHIND[0] - R > 0.30 + M.WALL_T / 2  # clear of the wall


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
