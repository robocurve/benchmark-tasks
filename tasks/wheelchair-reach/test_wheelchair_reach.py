"""Checks of wheelchair_reach.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_wheelchair_reach.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("wheelchair_reach.py"))
M = TASK.module
SIZE = {i.name: np.array(i.size) for i in M.ITEMS}
OTHER = {"first_aid_kit": "bandages", "bandages": "first_aid_kit"}


def item(name, y, bottom_m, tilt_deg=0.0, moving=False, x=None):
    """``name`` with its bottom ``bottom_m`` above the table, flat or turned ``tilt_deg`` about x."""
    r = math.radians(tilt_deg)
    half = SIZE[name] / 2
    up = abs(half[1] * math.sin(r)) + abs(half[2] * math.cos(r))
    centre = np.array([M.ITEM_X if x is None else x, y, M.TOP + bottom_m + up])
    ext = np.array([half[0], abs(half[1] * math.cos(r)) + abs(half[2] * math.sin(r)), up])
    return ObjectState(pos=centre, quat_xyzw=np.array([math.sin(r / 2), 0.0, 0.0, math.cos(r / 2)]),
                       lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - ext, high=centre + ext)


def on(ledge, name, dy=0.0, **kw):
    return item(name, M.LEDGE_Y[ledge][0] + dy, M.LEDGE_TOPS[ledge], **kw)


def start():
    return {"first_aid_kit": on("high_ledge", "first_aid_kit"), "bandages": on("low_ledge", "bandages")}


def middle():
    return {n: item(n, M.MIDDLE[n], M.LEDGE_TOPS["middle_ledge"]) for n in SIZE}


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0.0 and not M.success(o)


def test_both_start_wholly_out_of_range():
    """Kit 53 to 62 in, bandages 5 to 13 in: out whichever part of them is reached for."""
    o = start()
    assert M._span_in(o["first_aid_kit"])[0] > 48 and M._span_in(o["bandages"])[1] < 15


def test_both_on_the_middle_ledge_succeeds():
    o = middle()
    assert M.score(o) == 1.0 and M.success(o)


@pytest.mark.parametrize("moved", ["first_aid_kit", "bandages"])
def test_one_moved_scores_half(moved):
    o = start()
    o[moved] = on("middle_ledge", moved)
    assert M.score(o) == 0.5 and not M.success(o)


@pytest.mark.parametrize(("inches", "ok"), [(14.8, False), (15.2, True)])
def test_the_low_limit_is_at_the_bottom(inches, ok):
    assert M._in_range(item("bandages", -0.2, inches * M.INCH)) == ok


@pytest.mark.parametrize(("inches", "ok"), [(47.8, True), (48.2, False)])
def test_the_high_limit_is_at_the_top(inches, ok):
    top = inches * M.INCH
    assert M._in_range(item("bandages", -0.2, top - SIZE["bandages"][2])) == ok


def test_held_in_the_air_does_not_count():
    """At the middle ledge's height, in range, but off the ledge."""
    o = middle()
    o["bandages"] = item("bandages", 0.10, M.LEDGE_TOPS["middle_ledge"], x=0.20)
    assert M.score(o) == 0.5


@pytest.mark.parametrize(("lift_mm", "ok"), [(1.5, True), (3.0, False)])
def test_standing_on_the_ledge(lift_mm, ok):
    o = middle()
    o["bandages"] = item("bandages", M.MIDDLE["bandages"], M.LEDGE_TOPS["middle_ledge"] + lift_mm / 1000)
    assert M.score(o) == (1.0 if ok else 0.5)


@pytest.mark.parametrize(("dy", "ok"), [(0.0, True), (0.069, True), (0.075, False)])
def test_its_centre_over_the_ledge(dy, ok):
    """The middle ledge is 140 mm wide: 70 mm either side of its centre."""
    o = middle()
    o["bandages"] = on("middle_ledge", "bandages", dy=-dy)
    assert M._reachable(o, "bandages") == ok


def test_standing_on_the_other_item_is_supported():
    o = middle()
    kit = o["first_aid_kit"]
    o["bandages"] = item("bandages", float(kit.pos[1]), float(kit.high[2]) - M.TOP)
    assert M._supported(o["bandages"], kit)


def test_stacked_on_the_middle_ledge_the_top_one_is_too_high():
    o = middle()
    kit = o["first_aid_kit"]
    o["bandages"] = item("bandages", float(kit.pos[1]), float(kit.high[2]) - M.TOP)
    assert M.score(o) == 0.5


def test_on_the_table_is_too_low():
    o = middle()
    o["bandages"] = item("bandages", -0.2, 0.0, x=0.2)
    assert M.score(o) == 0.5


@pytest.mark.parametrize(("tilt", "ok"), [(0.0, True), (8.0, True), (12.0, False), (45.0, False), (90.0, True),
                                          (180.0, True)])
def test_any_face_counts_tipped_does_not(tilt, ok):
    o = middle()
    o["bandages"] = on("middle_ledge", "bandages", tilt_deg=tilt, dy=M.MIDDLE["bandages"] - M.LEDGE_Y["middle_ledge"][0])
    assert M.score(o) == (1.0 if ok else 0.5)


def test_moving_is_not_yet_success():
    o = middle()
    o["bandages"] = item("bandages", M.MIDDLE["bandages"], M.LEDGE_TOPS["middle_ledge"], moving=True)
    assert M.score(o) == 1.0 and not M.success(o)


def test_distance():
    o = start()
    kit_b, _ = M._span_in(o["first_aid_kit"])
    _, band_t = M._span_in(o["bandages"])
    kit_t = M._span_in(o["first_aid_kit"])[1]
    assert M.distance(o) == pytest.approx((kit_t - 48) + (15 - M._span_in(o["bandages"])[0]))
    assert M.distance(middle()) == 0.0


def test_no_ledge_is_above_another():
    spans = sorted((y - w / 2, y + w / 2) for y, w in M.LEDGE_Y.values())
    assert all(a[1] < b[0] for a, b in zip(spans, spans[1:]))


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["right"], station.BASES["right"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
