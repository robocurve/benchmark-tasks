"""Checks of pinwheel_pallet.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_pinwheel_pallet.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("pinwheel_pallet.py"))
M = TASK.module
HALF = np.array(M.CARTON_SIZE) / 2
NAMES = [c.name for c in M.CARTONS]
PINWHEEL = [((-1, 1), True), ((1, 1), False), ((1, -1), True), ((-1, -1), False)]  # the demo's
MIRROR = [((-1, 1), False), ((1, 1), True), ((1, -1), False), ((-1, -1), True)]


def _rot(yaw, roll=0.0):
    """The spin about the vertical times a roll about the carton's own 100 mm axis."""
    cz, sz, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    return np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]]) @ np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])


def carton(x, y, yaw_deg=0.0, bottom_z=None, roll_deg=0.0, moving=False):
    """A carton with its centre at x, y, turned ``yaw_deg``, rolled ``roll_deg`` about its 100 mm axis, its lowest
    corner at ``bottom_z`` (the pallet's top by default). A box's bounding box from its turned corners is exact."""
    bottom_z = M.PALLET_TOP if bottom_z is None else bottom_z
    psi, rho = math.radians(yaw_deg), math.radians(roll_deg)
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * HALF @ _rot(psi, rho).T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    sz, cz, sx, cx = math.sin(psi / 2), math.cos(psi / 2), math.sin(rho / 2), math.cos(rho / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([cz * sx, sz * sx, sz * cx, cz * cx]),
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=(corners + centre).min(axis=0),
        high=(corners + centre).max(axis=0),
    )


def in_corner(corner, along_x, inset_mm=4.0, yaw_deg=0.0, **kw):
    """A carton in ``corner``, its 100 mm side along x or y, turned ``yaw_deg`` more, its outermost point
    ``inset_mm`` inside both edges."""
    yaw = (0.0 if along_x else 90.0) + yaw_deg
    xy = (np.array([[i, j] for i in (-1, 1) for j in (-1, 1)]) * HALF[:2]) @ _rot(math.radians(yaw))[:2, :2].T
    (sx, sy), edge = corner, M.PALLET_SIDE / 2 - inset_mm / 1000
    x = M.PALLET_XY[0] + sx * edge - (sx * xy[:, 0]).max() * sx
    y = M.PALLET_XY[1] + sy * edge - (sy * xy[:, 1]).max() * sy
    return carton(x, y, yaw, **kw)


def at_start(name):
    return carton(*M.STARTS[name], bottom_z=M.TOP + 0.001)


def scene(*placed):
    """The first cartons placed as given, the rest still at their starts."""
    s = {n: at_start(n) for n in NAMES}
    s.update(zip(NAMES, placed))
    return s


def pinwheel(layout=PINWHEEL, **kw):
    return [in_corner(c, ax, **kw) for c, ax in layout]


def test_the_quaternion_matches_the_rotation():
    c = carton(0.3, 0.0, yaw_deg=25, roll_deg=3)
    assert np.allclose(M._rotation(c.quat_xyzw), _rot(math.radians(25), math.radians(3)))
    assert math.degrees(M._long_axis_yaw(c)) == pytest.approx(25, abs=1e-9)
    assert M._tilt_deg(c) == pytest.approx(3, abs=1e-9)


def test_the_start_scores_zero():
    s = scene()
    assert not M.success(s) and M.score(s) == 0.0


@pytest.mark.parametrize("layout", [PINWHEEL, MIRROR])
def test_a_pinwheel_either_way_round_is_success(layout):
    s = scene(*pinwheel(layout))
    assert M.success(s) and M.score(s) == 1.0


def test_the_demo_homes_make_a_pinwheel():
    s = scene(*(carton(*M._home(c, ax), 0.0 if ax else 90.0) for c, ax in PINWHEEL))
    assert M.success(s)


def test_the_demo_aims_fill_their_corners():
    """A carton that landed exactly where the demo aims it, AIM offset included, still fills its corner."""
    for c, ax in PINWHEEL:
        hx, hy = M._home(c, ax)
        dx, dy = M.AIM.get(c, (0.0, 0.0))
        s = scene(carton(hx + dx, hy + dy, 0.0 if ax else 90.0))
        assert M.score(s) == 0.25, c


def test_the_pinwheel_leaves_room_between_cartons():
    """Along each edge a 100 and a 50 mm carton, 4 mm inside both ends: 12 mm between them for a finger."""
    a, b = M.CARTON_SIZE[:2]
    assert 1000 * (M.PALLET_SIDE - a - b - 2 * M.INSET) == pytest.approx(12.0)


@pytest.mark.parametrize("inset_mm", [-M.OVERHANG_MM, M.FLUSH_MM])
def test_two_cartons_along_one_edge_would_overlap(inset_mm):
    """Why four filled corners force a pinwheel: two 100 mm sides along a 170 mm edge overlap by 200 - 170 = 30 mm,
    less what the two may stand past the edge (2 x 3 mm): at least 24 mm."""
    near, far = in_corner((-1, 1), True, inset_mm), in_corner((1, 1), True, inset_mm)
    overlap = near.high[0] - far.low[0]
    assert overlap >= 0.030 - 2 * M.OVERHANG_MM / 1000 - 1e-9


@pytest.mark.parametrize("filled", [0, 1, 2, 3, 4])
def test_each_filled_corner_scores_a_quarter(filled):
    assert M.score(scene(*pinwheel()[:filled])) == 0.25 * filled


@pytest.mark.parametrize(("inset_mm", "ok"), [(14.9, True), (15.5, False), (-2.9, True), (-3.5, False)])
def test_flush_within_15_mm_and_at_most_3_mm_past_the_edge(inset_mm, ok):
    s = scene(in_corner((1, -1), True, inset_mm))
    assert M.score(s) == (0.25 if ok else 0.0)


@pytest.mark.parametrize(("yaw_deg", "ok"), [(4.0, True), (-4.0, True), (6.0, False)])
def test_square_within_5_degrees(yaw_deg, ok):
    assert M.score(scene(in_corner((1, -1), True, yaw_deg=yaw_deg))) == (0.25 if ok else 0.0)


def _shifted(c, dx):
    d = np.array([dx, 0.0, 0.0])
    return ObjectState(pos=c.pos + d, quat_xyzw=c.quat_xyzw, lin_vel=c.lin_vel, ang_vel=c.ang_vel, low=c.low + d,
                       high=c.high + d)


@pytest.mark.parametrize("dx_mm", [0.0, 4.9, -4.9])
def test_a_tight_pinwheel_counts_centred_or_a_little_off(dx_mm):
    """Cartons touching, the layer centred: 10 mm inside every edge. Shifted, one side goes to 15 mm."""
    s = scene(*(_shifted(c, dx_mm / 1000) for c in pinwheel(inset_mm=10.0)))
    assert M.success(s)


def test_a_tilted_cartons_raised_corner_counts_for_flush():
    """Rolled 4 degrees, the 50 mm side reaches 1.3 mm further out than its footprint: 2 mm past becomes 3.3 mm."""
    assert M.score(scene(in_corner((1, -1), True, inset_mm=-2.0))) == 0.25
    assert M.score(scene(in_corner((1, -1), True, inset_mm=-2.0, roll_deg=4))) == 0.0


def test_upside_down_counts():
    s = scene(*pinwheel(roll_deg=180))
    assert M.success(s)


@pytest.mark.parametrize("roll_deg", [6, 90])
def test_tilted_or_on_its_side_does_not_count(roll_deg):
    """On its 100 x 40 side (rolled 90) four cartons would still need a pinwheel, but it is not lying flat."""
    assert M.score(scene(in_corner((1, -1), True, roll_deg=roll_deg))) == 0.0


def test_standing_on_its_end_does_not_count():
    """On end (50 x 40 footprint) four cartons fit in the corners any way round, so it must score 0."""
    r = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # 100 mm axis vertical
    q = np.array([0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)])
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * HALF @ r.T
    centre = np.array([M.PALLET_XY[0] + 0.06, M.PALLET_XY[1] - 0.06, M.PALLET_TOP + 0.05])
    c = ObjectState(pos=centre, quat_xyzw=q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                    low=(corners + centre).min(axis=0), high=(corners + centre).max(axis=0))
    assert np.allclose(M._rotation(q), r) and M.score(scene(c)) == 0.0


@pytest.mark.parametrize("bottom_mm, ok", [(0.0, True), (0.9, True), (1.1, False), (4.0, False), (40.0, False)])
def test_held_above_or_stacked_on_another_carton_does_not_count(bottom_mm, ok):
    c = in_corner((1, -1), True, bottom_z=M.PALLET_TOP + bottom_mm / 1000)
    assert M.score(scene(c)) == (0.25 if ok else 0.0)


def test_on_the_table_beside_the_pallet_does_not_count():
    assert M.score(scene(in_corner((1, -1), True, inset_mm=-60.0, bottom_z=M.TOP))) == 0.0


def test_a_moving_carton_is_not_yet_success():
    placed = pinwheel()
    placed[3] = in_corner((-1, -1), False, moving=True)
    s = scene(*placed)
    assert not M.success(s) and M.score(s) == 1.0


def test_distance_is_the_mean_corner_gap():
    s = scene(*pinwheel(inset_mm=4.0))
    assert M.distance(s) == pytest.approx(0.004 * math.sqrt(2), abs=1e-9)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
