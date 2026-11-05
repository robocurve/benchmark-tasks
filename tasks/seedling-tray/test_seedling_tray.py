"""Checks of seedling_tray.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_seedling_tray.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("seedling_tray.py"))
M = TASK.module
S1, S2, S3, S4 = M.SPARES
E1, E2, E3 = M.EMPTY


def plug_state(xy, bottom_z, tilt_deg=0.0, moving=False):
    """A plug standing over ``xy`` with its lowest point at ``bottom_z``, tilted ``tilt_deg`` about world x."""
    t = math.radians(tilt_deg)
    az = math.cos(t)
    centre_z = bottom_z + M.PLUG_H / 2 * abs(az) + M.PLUG_R * abs(math.sin(t))
    centre = np.array([xy[0], xy[1], centre_z])
    half = np.array([M.PLUG_R, M.PLUG_R, M.PLUG_H / 2])
    return ObjectState(pos=centre, quat_xyzw=np.array([math.sin(t / 2), 0.0, 0.0, math.cos(t / 2)]),
                       lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def fixed_state(box):
    centre, half = np.array(box.pos), np.array(box.size) / 2
    return ObjectState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def in_cell(cell, offset=(0.0, 0.0), **kw):
    return plug_state(M.CELLS[cell] + np.array(offset), M.FLOOR_TOP, **kw)


def scene(**plugs):
    """The start, except the plugs given as name=ObjectState."""
    out = {box.name: fixed_state(box) for box in M.TRAY}
    for name, cell in M.TRAY_PLUGS.items():
        out[name] = plugs.get(name, in_cell(cell))
    for name, x in zip(M.SPARES, M.SPARE_X, strict=True):
        out[name] = plugs.get(name, plug_state((x, M.SPARE_Y), M.TOP))
    return out


def filled(cells=M.EMPTY, **overrides):
    """spare_1, spare_2, ... in ``cells``, the rest as at the start; then the overrides."""
    return scene(**{**{s: in_cell(c) for s, c in zip(M.SPARES, cells, strict=False)}, **overrides})


def test_the_tray_has_twelve_cells_and_three_empty():
    assert len(M.CELLS) == 12 and len(M.TRAY_PLUGS) == 9 and len(M.EMPTY) == 3
    assert set(M.TRAY_PLUGS.values()) | set(M.EMPTY) == set(M.CELLS)
    assert len({c for c, _ in M.EMPTY}) == 3 and len({r for _, r in M.EMPTY}) == 3  # one per row, no shared column


def test_a_plug_fits_its_cell_with_13_mm_either_side():
    inside = M.PITCH - M.WALL_T
    assert (inside - 2 * M.PLUG_R) / 2 == pytest.approx(0.013)
    assert M.CELL_HALF == pytest.approx(inside / 2)


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_filling_all_three_is_success():
    assert M.success(filled()) and M.score(filled()) == 1.0


@pytest.mark.parametrize("n", [1, 2])
def test_each_filled_cell_scores_a_third(n):
    s = filled(M.EMPTY[:n])
    assert not M.success(s) and M.score(s) == pytest.approx(n / 3, abs=1e-6)


def test_any_spare_can_go_in_any_empty_cell():
    s = scene(**{S4: in_cell(E1), S2: in_cell(E2), S1: in_cell(E3)})
    assert M.score(s) == 1.0


@pytest.mark.parametrize("offset", [(0.0, 0.009), (0.0, -0.012), (0.012, 0.012), (-0.0129, 0.0)])
def test_a_plug_anywhere_on_its_cells_floor_counts(offset):
    # Review: with an 8 mm limit, a plug standing in its cell 9 or 12 mm off centre scored nothing for that cell.
    assert M.score(filled(**{S1: in_cell(E1, offset)})) == 1.0


def test_a_plug_outside_the_cells_walls_doesnt_count():
    assert not M._in_cell(in_cell(E1, (0.0, 0.03)), E1)


def test_a_plug_on_top_of_another_doesnt_count():
    s = filled(M.EMPTY[:2], **{S3: plug_state(M.CELLS[M.IN_TRAY[0]], M.FLOOR_TOP + M.PLUG_H)})
    assert M.score(s) == pytest.approx(2 / 3, abs=1e-6)


def test_moving_a_plug_already_in_the_tray_scores_zero():
    first = next(iter(M.TRAY_PLUGS))
    s = filled(**{first: in_cell(M.TRAY_PLUGS[first], (0.006, 0.0))})
    assert M.score(s) == 0.0
    s = filled(**{first: in_cell(M.TRAY_PLUGS[first], (0.004, 0.0))})  # under 5 mm: still untouched
    assert M.score(s) == 1.0


def test_swapping_a_tray_plug_into_an_empty_cell_scores_zero():
    first = next(iter(M.TRAY_PLUGS))
    assert M.score(filled(M.EMPTY[1:], **{first: in_cell(E1)})) == 0.0


@pytest.mark.parametrize("name", [S4, "plug_5"])
def test_a_knocked_over_plug_scores_zero(name):
    xy = M.CELLS[M.TRAY_PLUGS[name]] if name in M.TRAY_PLUGS else (M.SPARE_X[3], M.SPARE_Y)
    base = M.FLOOR_TOP if name in M.TRAY_PLUGS else M.TOP
    assert M.score(filled(**{name: plug_state(xy, base, tilt_deg=90)})) == 0.0


def test_a_plug_off_the_table_scores_zero():
    assert M.score(filled(**{S4: plug_state((0.3, 0.6), M.TOP - 0.3)})) == 0.0


def test_a_moving_plug_is_not_yet_counted():
    s = filled(**{S3: in_cell(E3, moving=True)})
    assert not M.success(s) and M.score(s) == pytest.approx(2 / 3, abs=1e-6)


def test_the_demo_fills_the_three_cells_and_leaves_the_fourth_spare():
    releases = [np.array(w.pos) for w in M.demo()["left"] if w.label == "release"]
    assert len(releases) == 3 and list(M.DEMO_FILL) == [S1, S2, S3]
    s = scene(**{s: plug_state(p[:2], p[2] - M.DROP - M.GRIP_ABOVE) for s, p in zip(M.SPARES, releases, strict=False)})
    assert M.success(s) and M.score(s) == 1.0


def test_the_fingers_come_down_only_part_open_and_clear_the_neighbours():
    apart = 0.094 * M.APPROACH_FINGER / FINGER_OPEN
    assert apart - 2 * M.PLUG_R >= 0.012  # at least 6 mm clear of the plug on each side
    assert M.PITCH - M.PLUG_R - (apart / 2 + 0.010) >= 0.012  # 10 mm fingers: 12 mm or more from the next plug
    for label in ("to the object", "descend", "steady"):
        assert all(w.finger == M.APPROACH_FINGER for w in M.demo()["left"] if w.label == label)


def test_the_pad_tips_stay_above_the_walls():
    lowest = min(w.pos[2] for w in M.demo()["left"] if w.label in ("set down", "hold", "release", "settle"))
    assert lowest - PAD_TIP_PAST_TCP - (M.FLOOR_TOP + M.WALL_H) >= 0.01


def test_a_carried_plug_clears_every_plug_top():
    assert M.TRAVEL_Z - M.GRIP_ABOVE - (M.FLOOR_TOP + M.PLUG_H) >= 0.02


def test_the_grip_centres_the_plug():
    from yam_arena.constants import GRIPPER

    squeeze = (2 * M.PLUG_R - 0.094 * M.GRIP_FINGER / FINGER_OPEN) / 2
    force = GRIPPER["stiffness"] * squeeze
    assert 0.002 <= squeeze and force < 0.5 * GRIPPER["effort"]
    assert 2 * force * 1.0 > 10 * M.PLUG_KG * 9.81


@pytest.mark.parametrize("arm", list(M.demo()))
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0


def test_the_demo_finishes_well_within_the_run_limit():
    assert sum(w.duration for w in M.demo()["left"]) < 2400 / 20 / 2
