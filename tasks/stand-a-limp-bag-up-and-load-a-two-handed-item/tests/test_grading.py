"""Every score, and every way to a score that was not earned. NumPy only."""

import numpy as np
import pytest

from rdb_bag_load import bag_model as m
from rdb_bag_load import grading

UPRIGHT = {name: 0. for name in (*m.LEAN.values(), *m.FOLD.values())}
HANGING = {m.SWING[w]: np.radians(-176.) for w in m.HANDLES}
FAR = np.array(((.9, .9, .3), (.9, -.9, .3)))                   # both gripper centre points well away
STARTS = {"carton": (*m.CARTON_XY, m.CARTON[2] / 2), "pack": (*m.PACK_XY, m.PACK[2] / 2)}


def rot_x(angle):
    return m.rot_x(angle)


def rot_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array(((c, 0., s), (0., 1., 0.), (-s, 0., c)))


def scene(angles=None, *, inside=(), base=(0., 0., 0.), base_rot=None, shift=None, lift=0., swing=None, place=None):
    """Poses for a grader update. ``inside``: items standing in their places in the bag; the others stand where
    they start. ``place``: name -> (position, rotation) to override. ``lift``: height of the base board."""
    q = {**UPRIGHT, **HANGING, **(angles or {}), **(swing or {})}
    origin = np.array((*m.BAG_XY, 0.)) + np.asarray(base, dtype=float) + (0., 0., lift)
    frame = np.eye(3) if base_rot is None else np.asarray(base_rot)
    uppers, handles = {}, {}
    for wall in m.WALLS:
        pose = m.wall_pose(wall, q[m.LEAN[wall]], q[m.FOLD[wall]], q.get(m.SWING.get(wall), 0.))
        uppers[wall] = (origin + frame @ pose["upper"][0], frame @ pose["upper"][1])
        if wall in m.HANDLES:
            handles[wall] = (origin + frame @ pose["handle"][0], frame @ pose["handle"][1])
    items = {}
    for name, slot in (("carton", m.CARTON_SLOT), ("pack", m.PACK_SLOT)):
        if name in inside:
            items[name] = (origin + frame @ (np.asarray(slot) + (shift or {}).get(name, np.zeros(3))), frame)
        else:
            items[name] = (np.asarray(STARTS[name]), np.eye(3))
    items.update(place or {})
    return dict(base=(origin, frame), uppers=uppers, handles=handles, items=items)


def run(grade, state, ticks=1, tcp=FAR, touching=False):
    out = None
    for _ in range(ticks):
        out = grade.update(**state, tcp=tcp, touching=touching)
    return out


def lifted_grade(inside=("carton", "pack"), ticks=grading.HOLD_TICKS):
    grade = grading.BagGrade()
    up = {m.SWING[w]: 0. for w in m.HANDLES}                      # handles pulled straight up
    run(grade, scene(inside=inside))
    run(grade, scene(inside=inside, lift=.060, swing=up), ticks, touching=True)
    return grade


def test_start_scores_nothing():
    out = run(grading.BagGrade(), scene(m.START), 50)
    assert out["score"] == 0. and not out["upright"] and not out["carton"]["rests"]


def test_empty_bag_held_open_scores_nothing():
    assert run(grading.BagGrade(), scene(), 50)["score"] == 0.


def test_carton_alone():
    out = run(grading.BagGrade(), scene(inside=("carton",)), 50)
    assert out["score"] == .25 and out["upright"] and out["carton"]["rests"]


def test_pack_alone():
    assert run(grading.BagGrade(), scene(inside=("pack",)), 50)["score"] == .5


def test_both_without_the_lift():
    out = run(grading.BagGrade(), scene(inside=("carton", "pack")), 100)
    assert out["score"] == .75 and not out["lifted"]


def test_full_score_needs_two_seconds_hands_off():
    grade = lifted_grade()
    assert grade.lifted
    down = scene(inside=("carton", "pack"))
    assert run(grade, down, grading.HOLD_TICKS - 1)["score"] == .75
    assert run(grade, down)["score"] == 1.


def test_lift_must_last_two_seconds():
    grade = lifted_grade(ticks=grading.HOLD_TICKS - 1)
    assert not grade.lifted
    assert run(grade, scene(inside=("carton", "pack")), 100)["score"] == .75


def test_lift_needs_both_items_and_the_handles():
    assert not lifted_grade(inside=("carton",)).lifted
    grade = grading.BagGrade()                                    # carried some other way, handles hanging down
    run(grade, scene(inside=("carton", "pack"), lift=.060), 100, touching=True)
    assert not grade.lifted
    grade = grading.BagGrade()                                    # not high enough
    run(grade, scene(inside=("carton", "pack"), lift=.040, swing={m.SWING[w]: 0. for w in m.HANDLES}), 100, touching=True)
    assert not grade.lifted


def test_still_held_or_too_close_is_not_done():
    grade = lifted_grade()
    down = scene(inside=("carton", "pack"))
    assert run(grade, down, 100, touching=True)["score"] == .75
    grip = m.grip_centre(*down["handles"]["left"])
    assert run(grade, down, 100, tcp=np.array((grip + (0., .040, 0.), FAR[1])))["score"] == .75
    assert run(grade, down, 100, tcp=np.array((grip + (0., .060, 0.), FAR[1])))["score"] == 1.


@pytest.mark.parametrize("change", [
    dict(angles={m.LEAN["left"]: np.radians(40.), m.FOLD["left"]: np.radians(-80.)}),     # a wall folded in: rim at 139 mm
    dict(base=(.060, 0., 0.)),                                                           # off its marks
    dict(base_rot=rot_y(np.radians(12.))),                                               # tipped
    dict(lift=.010),                                                                     # not on the table
])
def test_bag_not_upright_scores_nothing(change):
    out = run(lifted_grade(), scene(inside=("carton", "pack"), **change), 100)
    assert out["score"] == 0. and not out["upright"]


def test_item_on_a_folded_wall_does_not_rest():
    out = run(grading.BagGrade(), scene(inside=("carton", "pack"), shift={"pack": np.array((0., 0., .006))}), 50)
    assert not out["pack"]["rests"] and out["score"] == .25


def test_item_over_the_edge_or_leaning_does_not_rest():
    grade = grading.BagGrade()
    assert not run(grade, scene(inside=("carton", "pack"), shift={"pack": np.array((.030, 0., 0.))}))["pack"]["rests"]
    state = scene(inside=("pack",))
    centre = np.array((*m.BAG_XY, 0.)) + m.CARTON_SLOT
    lean = rot_x(np.radians(15.))
    state["items"]["carton"] = (centre + (0., 0., .012), lean)
    assert not run(grade, state)["carton"]["rests"]


def test_item_off_the_table_is_zero_for_good():
    grade = grading.BagGrade()
    fallen = scene(inside=("carton",), place={"pack": (np.array((.5, -.8, -.3)), np.eye(3))})
    assert run(grade, fallen)["score"] == 0.
    assert run(grade, scene(inside=("carton", "pack")), 100)["score"] == 0.


def test_item_spilled_during_a_lift_is_zero_for_good():
    grade = grading.BagGrade()
    up = {m.SWING[w]: 0. for w in m.HANDLES}
    run(grade, scene(inside=("carton", "pack"), lift=.060, swing=up), 10, touching=True)
    run(grade, scene(inside=("carton",), lift=.060, swing=up), 1, touching=True)        # the pack is back on the table
    assert grade.spilled
    assert run(grade, scene(inside=("carton", "pack")), 100)["score"] == 0.


def test_taking_an_item_out_on_the_table_is_not_a_spill():
    grade = grading.BagGrade()
    run(grade, scene(inside=("carton", "pack")), 10)
    assert run(grade, scene(inside=("carton",)), 10)["score"] == .25 and not grade.spilled


def test_torn_seam_is_zero_for_good():
    grade = grading.BagGrade()
    torn = scene({m.LEAN["near"]: np.radians(-60.), m.FOLD["near"]: 0.}, inside=("carton", "pack"))   # an end wall pulled outward
    assert max(m.seam_lengths({m.UPPER[w]: torn["uppers"][w] for w in m.WALLS}).values()) > m.SEAM_TORN_M
    assert run(grade, torn)["torn"]
    assert run(grade, scene(inside=("carton", "pack")), 100)["score"] == 0.


def test_nonfinite_state_scores_nothing():
    state = scene(inside=("carton", "pack"))
    state["items"]["pack"] = (np.array((np.nan, 0., 0.)), np.eye(3))
    assert run(grading.BagGrade(), state)["score"] == 0.


def test_tiers_are_the_only_scores():
    seen = set()
    for inside in ((), ("carton",), ("pack",), ("carton", "pack")):
        seen.add(run(grading.BagGrade(), scene(inside=inside), 100)["score"])
    assert seen == {0., .25, .5, .75}

def test_contents_can_shift_during_the_lift_but_must_settle_at_the_end():
    grade = grading.BagGrade()
    both = ("carton", "pack")
    up = {m.SWING[w]: 0. for w in m.HANDLES}
    offset = {"pack": np.array((0., 0., .015))}
    moving = scene(inside=both, lift=.060, swing=up, shift=offset)
    held = run(grade, moving, grading.HOLD_TICKS, touching=True)
    assert held["lifted"] and not held["pack"]["rests"]
    assert run(grade, scene(inside=both, shift=offset), grading.HOLD_TICKS)["score"] < 1
    assert run(grade, scene(inside=both), grading.HOLD_TICKS)["score"] == 1
