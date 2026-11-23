"""The scoring rules, tier by tier, and the unintended strategies."""

import numpy as np
import pytest

from rdb_overfull_drawer import layout as L
from rdb_overfull_drawer.grading import HOLD_TICKS, DrawerGrade, clearance, pillow_in_drawer

FAR = [(.15, .30, .25), (.15, -.30, .25)]          # both grippers well away from the unit


def base(open_m, dx=0., dz=0.):
    return (L.drawer_x(open_m) + L.FRONT_T + L.DRAWER_INNER[0] / 2 + dx, 0., L.FLOOR_Z + .0005 + dz)


def tops(open_m, pressed=()):
    """Pad tops for a drawer at this opening: under the frame they are at its underside, outside at rest."""
    out = []
    for row in range(L.ROWS):
        rest = L.FLOOR_Z + L.REST_TOP[row]
        under = L.row_lead(row) > open_m or row in pressed
        out += [min(rest, L.RAIL_Z) if under else rest] * L.COLS
    return out


def final(open_m, *, ticks=HOLD_TICKS, tcp=FAR, touching=False, base_at=None, pad_tops=None, shift=0.):
    grade, result = DrawerGrade(), None
    for _ in range(ticks):
        result = grade.update(open_m, base_at or base(open_m), (0., 0., 1.), pad_tops or tops(open_m), tcp, touching, shift)
    return result


@pytest.mark.parametrize("open_mm, score", [(220, 0.), (214, 0.), (191, 0.), (190, .25), (163, .25), (112, .25),
                                            (91, .25), (90, .5), (30, .5), (6, .5), (5, 1.), (0, 1.)])
def test_tiers_by_opening(open_mm, score):
    assert final(open_mm / 1000.)["score"] == score


def test_thresholds_sit_between_the_places_where_the_pillow_catches():
    catch = [L.blocking_open(row) for row in (3, 2, 1)]
    assert catch[0] > .190 > catch[1] and catch[2] > .090          # 0.25 needs one part under, 0.5 all three
    assert L.OPEN_START - catch[0] < .010                           # the drawer is all but caught at the start


def test_shut_needs_two_seconds_with_hands_clear():
    assert final(0., ticks=HOLD_TICKS - 1)["score"] == .75
    assert final(0., ticks=HOLD_TICKS)["score"] == 1.
    assert final(0., touching=True)["score"] == .75                 # a hand still on the drawer
    near = [(L.FRAME_FRONT_X - .030, 0., .10), FAR[1]]
    assert clearance(near, 0.) < .05 and final(0., tcp=near)["score"] == .75
    above = [(L.FRAME_FRONT_X + .10, 0., L.FRAME_SIZE[2] + .04), FAR[1]]
    assert final(0., tcp=above)["score"] == .75                     # hovering 4 cm over the frame is not clear


def test_hold_restarts_when_interrupted():
    grade = DrawerGrade()
    for _ in range(HOLD_TICKS - 5):
        grade.update(0., base(0.), (0, 0, 1), tops(0.), FAR, False)
    grade.update(0., base(0.), (0, 0, 1), tops(0.), FAR, True)
    for _ in range(HOLD_TICKS - 1):
        result = grade.update(0., base(0.), (0, 0, 1), tops(0.), FAR, False)
    assert result["score"] == .75
    assert grade.update(0., base(0.), (0, 0, 1), tops(0.), FAR, False)["score"] == 1.


def test_end_state_counts_not_the_best_moment():
    grade = DrawerGrade()
    for _ in range(HOLD_TICKS - 1):
        grade.update(0., base(0.), (0, 0, 1), tops(0.), FAR, False)
    assert grade.update(.150, base(.150), (0, 0, 1), tops(.150), FAR, False)["score"] == .25     # pulled open again


def test_pillow_taken_out_scores_zero_even_with_the_drawer_shut():
    on_table = (.20, .40, .0005)
    assert not pillow_in_drawer(on_table, (0, 0, 1), 0.)
    assert final(0., base_at=on_table)["score"] == 0.
    assert final(0., base_at=base(0., dz=.05))["score"] == 0.          # lifted off the drawer floor
    assert final(0., base_at=base(0., dx=-.05))["score"] == 0.         # hanging out over the front
    grade = DrawerGrade()
    assert grade.update(0., base(0.), (.5, 0., .87), tops(0.), FAR, False)["score"] == 0.        # tipped up on edge


def test_pillow_forced_through_the_frame_is_not_shut():
    forced = tops(0.)
    forced[6] = L.RAIL_Z + .010                # a pad 10 mm inside the frame's top: contact gave way under a slam
    result = final(0., pad_tops=forced)
    assert not result["shut"] and result["score"] == .5


def test_unit_dragged_scores_zero():
    assert final(0., shift=.011)["score"] == 0. and final(0., shift=.009)["score"] == 1.


def test_nonfinite_state_scores_zero():
    grade = DrawerGrade()
    assert grade.update(float("nan"), base(0.), (0, 0, 1), tops(0.), FAR, False)["score"] == 0.


def test_untouched_scene_scores_zero():
    assert final(L.OPEN_START)["score"] == 0.
