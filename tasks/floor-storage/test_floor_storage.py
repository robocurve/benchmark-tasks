"""Checks of floor_storage.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_floor_storage.py
"""

import itertools
import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("floor_storage.py"))
M = TASK.module
NAMES = [b.name for b in M.BUNDLES]
FT = M.FT
HX, HY, HZ = (v / 2 for v in M.BUNDLE)


def _quat(yaw_deg=0.0, roll_deg=0.0):
    """Yaw about z after a roll about x, as xyzw."""
    a, b = math.radians(yaw_deg) / 2, math.radians(roll_deg) / 2
    return np.array([math.cos(a) * math.sin(b), math.sin(a) * math.sin(b), math.sin(a) * math.cos(b),
                     math.cos(a) * math.cos(b)])


def bundle(x, y, bottom=None, yaw_deg=0.0, roll_deg=0.0, moving=False):
    """A bundle centred at (x, y), its bottom at ``bottom`` (default: on the slab)."""
    q = _quat(yaw_deg, roll_deg)
    r = M._rotation(q)
    corners = np.array([r @ (np.array(s) * [HX, HY, HZ]) for s in itertools.product((-1, 1), repeat=3)])
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    z = (M.SLAB_TOP if bottom is None else bottom) - lo[2]
    centre = np.array([x, y, z])
    return ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.array([0.05, 0, 0]) if moving else np.zeros(3),
                       ang_vel=np.zeros(3), low=centre + lo, high=centre + hi)


def start():
    return {n: bundle(x, y) for n, (x, y) in M.START.items()}


def placed(**where):
    o = start()
    o.update(where)
    return o


LEGAL = {n: bundle(*M.PLACE[n]) for n in NAMES}
HOIST_FAR = M.HOIST_X[1]  # the hoistway's far edge (x)
HOIST_MID_Y = sum(M.HOIST_Y) / 2
WALL = M.SLAB_X[1]  # the outside walls' inner face (x)


def test_the_start_scores_zero():
    o = start()
    assert M.score(o) == 0 and not M.success(o)
    assert all(not M._stored(o, n) for n in NAMES)


def test_the_demo_layout_succeeds():
    assert M.score(LEGAL) == 1.0 and M.success(LEGAL)


@pytest.mark.parametrize("name", NAMES)
def test_each_bundle_scores_a_third(name):
    assert M.score(placed(**{name: LEGAL[name]})) == pytest.approx(1 / 3)


@pytest.mark.parametrize("ft, ok", [(5.8, False), (6.2, True), (8.0, True)])
def test_six_feet_from_the_hoistway(ft, ok):
    b = bundle(HOIST_FAR + ft * FT + HX, HOIST_MID_Y)
    assert M.hoistway_ft(b) == pytest.approx(ft)
    assert M.score(placed(lumber=b)) == pytest.approx(1 / 3 if ok else 0)


@pytest.mark.parametrize("ft, ok", [(5.8, False), (6.2, True)])
def test_six_feet_diagonally_from_the_hoistway_corner(ft, ok):
    d = ft * FT / math.sqrt(2)
    b = bundle(HOIST_FAR + d + HX, M.HOIST_Y[0] - d - HY)
    assert M.hoistway_ft(b) == pytest.approx(ft)
    assert M._stored(placed(lumber=b), "lumber") is ok


def test_turned_bundles_are_measured_by_their_corners():
    b = bundle(HOIST_FAR + 6.2 * FT + HX, HOIST_MID_Y, yaw_deg=30)  # its corners swing toward the hoistway
    assert M.hoistway_ft(b) < 6.0 and not M._stored(placed(lumber=b), "lumber")


def test_in_the_hoistway_does_not_count():
    b = bundle(sum(M.HOIST_X) / 2, HOIST_MID_Y, bottom=M.TOP + 0.001)
    assert M.hoistway_ft(b) == 0 and not M._stored(placed(lumber=b), "lumber")


@pytest.mark.parametrize("ft, ok", [(9.8, False), (10.2, True)])
def test_ten_feet_from_the_parapet(ft, ok):
    b = bundle(WALL - ft * FT - HX, 0.20)
    assert M.walls_in_force(placed(drywall=b), "drywall") == ["parapet"]
    assert M.wall_ft(b, "parapet") == pytest.approx(ft)
    assert M._stored(placed(drywall=b), "drywall") is ok


def test_against_the_full_wall_is_allowed():
    b = bundle(WALL - HX - 0.002, -0.15)
    assert M.walls_in_force(placed(drywall=b), "drywall") == ["parapet"] and M.wall_ft(b, "parapet") > 10
    assert M._stored(placed(drywall=b), "drywall")


def test_against_the_full_wall_but_near_the_parapets_end_is_not():
    b = bundle(WALL - HX - 0.002, -HY - 0.02)  # 20 mm (3.3 ft) along the wall from where the parapet starts
    assert M.wall_ft(b, "parapet") < 10 and not M._stored(placed(drywall=b), "drywall")


@pytest.mark.parametrize("below_mm, counts", [(0.0, True), (1.0, False)])
def test_the_full_wall_counts_once_the_material_reaches_its_top(below_mm, counts):
    wall_top = M.SLAB_TOP + M.WALL_HEIGHTS["full_wall"]
    b = bundle(WALL - HX - 0.002, -0.15, bottom=wall_top - below_mm / 1000 - 2 * HZ)
    assert ("full_wall" in M.walls_in_force(placed(drywall=b), "drywall")) is counts


def test_a_stack_taller_than_the_full_wall_is_not_stored_against_it():
    x, y = WALL - HX - 0.002, -0.15  # standing on their 30 mm side: 30 + 30 + 30 = 90 mm, above the 61 mm wall
    low = bundle(x, y, roll_deg=90)
    mid = bundle(x, y, bottom=float(low.high[2]), roll_deg=90)
    top = bundle(x, y, bottom=float(mid.high[2]), roll_deg=90)
    o = placed(lumber=low, insulation=mid, drywall=top)
    assert all(M._on_floor(o, n) for n in NAMES)
    assert M.stack_top(o, "lumber") == pytest.approx(float(top.high[2]))
    assert all("full_wall" in M.walls_in_force(o, n) for n in NAMES)
    assert M.score(o) == 0


def test_the_same_stack_away_from_the_walls_is_stored():
    x, y = M.PLACE["lumber"]
    low = bundle(x, y, roll_deg=90)
    mid = bundle(x, y, bottom=float(low.high[2]), roll_deg=90)
    top = bundle(x, y, bottom=float(mid.high[2]), roll_deg=90)
    assert M.score(placed(lumber=low, insulation=mid, drywall=top)) == 1.0


def test_resting_on_a_bundle_held_in_the_air_does_not_count():
    held = bundle(*M.PLACE["lumber"], bottom=M.SLAB_TOP + 0.010)
    on_it = bundle(*M.PLACE["lumber"], bottom=float(held.high[2]))
    o = placed(lumber=held, drywall=on_it)
    assert not M._on_floor(o, "drywall") and not M._stored(o, "drywall")


def test_a_centre_past_the_slabs_edge_does_not_count():
    b = bundle(M.SLAB_X[0] - 0.002, 0.20)  # bottom level with the slab, centre past its near edge
    assert not M._on_floor(placed(lumber=b), "lumber")


def test_stacked_on_another_bundle_counts():
    lower = LEGAL["lumber"]
    upper = bundle(*M.PLACE["lumber"], bottom=float(lower.high[2]))
    assert M._stored(placed(lumber=lower, drywall=upper), "drywall")


@pytest.mark.parametrize("where", [(0.05, 0.20), (0.45, 0.20), (0.30, 0.47)])
def test_off_the_slab_or_outside_the_building_does_not_count(where):
    b = bundle(*where, bottom=M.TOP)
    assert not M._stored(placed(lumber=b), "lumber")


def test_held_above_the_slab_does_not_count():
    b = bundle(*M.PLACE["lumber"], bottom=M.SLAB_TOP + 0.005)
    assert not M._stored(placed(lumber=b), "lumber")


@pytest.mark.parametrize("roll_deg, ok", [(90, True), (180, True), (8, True), (15, False), (45, False)])
def test_any_face_counts_tipped_does_not(roll_deg, ok):
    b = bundle(*M.PLACE["lumber"], roll_deg=roll_deg)
    assert M._stored(placed(lumber=b), "lumber") is ok


def test_moving_is_not_yet_success():
    o = dict(LEGAL, lumber=bundle(*M.PLACE["lumber"], moving=True))
    assert M.score(o) == 1.0 and not M.success(o)


def test_the_bundles_start_on_the_slab_and_in_violation():
    o = start()
    assert all(M._on_floor(o, n) for n in NAMES)
    assert M.hoistway_ft(o["lumber"]) < 6 and M.hoistway_ft(o["insulation"]) < 6
    assert M.wall_ft(o["drywall"], "parapet") < 10


def test_the_slab_pieces_leave_exactly_the_hoistway():
    area = sum(s.size[0] * s.size[1] for s in M.SLAB if s.name.startswith("slab"))
    full = (M.SLAB_X[1] - M.SLAB_X[0]) * (M.SLAB_Y[1] - M.SLAB_Y[0])
    hole = (M.HOIST_X[1] - M.HOIST_X[0]) * (M.HOIST_Y[1] - M.HOIST_Y[0])
    assert area == pytest.approx(full - hole)


@pytest.mark.parametrize("arm", ["left"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
