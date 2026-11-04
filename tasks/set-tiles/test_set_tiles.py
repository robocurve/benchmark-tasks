"""Checks of set_tiles.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_set_tiles.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.constants import PAD_TIP_PAST_TCP
from yam_arena.demo import ArmPlan
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("set_tiles.py"))
M = TASK.module
W = M.TILE[0]
ROW_Y = M.EDGE_IN - W / 2  # a square tile's centre, touching the straightedge on the middle side
OUT_Y = M.EDGE_OUT + W / 2  # the same on the left-arm side


def _rot_x(t):
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def tile_state(x, y, bottom_z=None, turn_deg=0.0, tilt_deg=0.0, moving=False):
    """A tile with its centre over (x, y), turned ``turn_deg`` about the vertical, then tilted ``tilt_deg`` about world
    x, with its lowest point at ``bottom_z``. Its bounding box is built the way Arena 0.3.1 builds it: the 8 corners of
    the tile, turned with it."""
    bottom_z = M.TOP if bottom_z is None else bottom_z
    t, s = math.radians(tilt_deg), math.radians(turn_deg)
    rot = _rot_x(t) @ _rot_z(s)
    half = np.array(M.TILE) / 2
    corners = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)]) * half @ rot.T
    centre = np.array([x, y, bottom_z - corners[:, 2].min()])
    world = corners + centre
    st, ct, ss, cs = math.sin(t / 2), math.cos(t / 2), math.sin(s / 2), math.cos(s / 2)
    return ObjectState(
        pos=centre,
        quat_xyzw=np.array([st * cs, -st * ss, ct * ss, ct * cs]),  # the tilt about x times the turn about z
        lin_vel=np.array([0.05, 0.0, 0.0]) if moving else np.zeros(3),
        ang_vel=np.zeros(3),
        low=world.min(axis=0),
        high=world.max(axis=0),
    )


def edge_state():
    centre, half = np.array(M.EDGE.pos), np.array(M.EDGE.size) / 2
    return ObjectState(pos=centre, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=centre - half, high=centre + half)


def scene(**tiles):
    """The start (every tile in its starting line, turned as it starts), except the tiles given as name=ObjectState."""
    out = {"straightedge": edge_state()}
    for name, x, turn in zip(M.NAMES, M.SUPPLY_X, M.SUPPLY_TURN_DEG, strict=True):
        out[name] = tiles.get(name, tile_state(x, M.SUPPLY_Y, turn_deg=turn))
    return out


def row_xs(joints_mm=(3.0, 3.0, 3.0), first=0.37):
    """Centre x of four tiles in a row running toward the arms, with the given joints."""
    xs = [first]
    for j in joints_mm:
        xs.append(xs[-1] - W - j / 1000)
    return xs


def laid(joints_mm=(3.0, 3.0, 3.0), y=ROW_Y, **overrides):
    """All four tiles in a row against the straightedge with the given joints; then the overrides."""
    row = {name: tile_state(x, y) for name, x in zip(M.NAMES, row_xs(joints_mm), strict=True)}
    return scene(**{**row, **overrides})


def test_the_quaternion_matches_the_rotation():
    tile = tile_state(0.3, ROW_Y, turn_deg=30, tilt_deg=1)
    assert np.allclose(M._rot(tile.quat_xyzw), _rot_x(math.radians(1)) @ _rot_z(math.radians(30)))


def test_the_start_scores_zero():
    assert not M.success(scene()) and M.score(scene()) == 0.0


def test_a_full_row_is_success():
    assert M.success(laid()) and M.score(laid()) == 1.0


def test_a_full_row_on_the_other_side_of_the_straightedge_is_success_too():
    assert M.success(laid(y=OUT_Y)) and M.score(laid(y=OUT_Y)) == 1.0


@pytest.mark.parametrize(("joint_mm", "good"), [(1.9, False), (2.1, True), (3.0, True), (3.9, True), (4.1, False)])
def test_a_joint_counts_from_2_to_4_mm(joint_mm, good):
    row = laid(joints_mm=(3.0, joint_mm, 3.0))
    assert M.score(row) == pytest.approx(1.0 if good else 0.8)
    assert M.success(row) == good


def test_each_set_tile_and_good_joint_earns_its_share():
    one = scene(tile_1=tile_state(0.37, ROW_Y))
    assert M.score(one) == pytest.approx(0.1)
    xs = row_xs()
    two = scene(tile_1=tile_state(xs[0], ROW_Y), tile_2=tile_state(xs[1], ROW_Y))
    assert M.score(two) == pytest.approx(0.4)


@pytest.mark.parametrize(("gap_mm", "set_"), [(0.0, True), (0.9, True), (1.1, False)])
def test_a_tile_must_be_within_1_mm_of_the_straightedge(gap_mm, set_):
    xs = row_xs()
    row = laid(tile_2=tile_state(xs[1], ROW_Y - gap_mm / 1000))
    assert M.score(row) == pytest.approx(1.0 if set_ else 0.5)  # an unset tile also breaks its two joints


def _corner_out(turn_deg):
    """How much further a turned tile's corners reach than its sides do, square on (m)."""
    s = math.radians(turn_deg)
    return W / 2 * (math.cos(s) + math.sin(s)) - W / 2


def test_a_turned_tile_is_measured_at_its_corners():
    assert M._turn_deg(tile_state(0.3, ROW_Y, turn_deg=1.5)) == pytest.approx(1.5)
    assert M._turn_deg(tile_state(0.3, ROW_Y, turn_deg=88.5)) == pytest.approx(1.5)
    xs = row_xs(joints_mm=(3.0, 2.5, 3.0))
    turned = tile_state(xs[2], ROW_Y - _corner_out(1.5), turn_deg=1.5)  # a corner touching the straightedge
    assert M._set(turned)
    # Centre to centre, its joints would be 2.5 and 3 mm; at the narrowest point they are 0.77 mm less: 1.73 (too
    # tight) and 2.23 (good).
    assert M.score(laid(joints_mm=(3.0, 2.5, 3.0), tile_3=turned)) == pytest.approx(0.4 + 0.2 * 2)


def test_a_tile_turned_more_than_2_degrees_is_not_set():
    assert not M._set(tile_state(0.3, ROW_Y - _corner_out(3.0), turn_deg=3.0))


def test_a_tile_turned_90_degrees_is_still_square():
    xs = row_xs()
    assert M.score(laid(tile_2=tile_state(xs[1], ROW_Y, turn_deg=90))) == 1.0


def test_a_tile_off_the_end_of_the_straightedge_is_not_set():
    assert not M._set(tile_state(M.EDGE_X[1] + 0.02, ROW_Y))


def test_a_tilted_tile_is_not_set():
    xs = row_xs()
    tipped = tile_state(xs[3], ROW_Y - 0.0005, tilt_deg=3)
    assert M.score(laid(tile_4=tipped)) == pytest.approx(0.3 + 0.2 * 2)  # the end tile: one joint lost


def test_a_tile_resting_on_another_is_not_set():
    xs = row_xs()
    on_top = tile_state(xs[2], ROW_Y, bottom_z=M.TOP + M.TILE[2])
    assert M.score(laid(tile_4=on_top)) == pytest.approx(0.3 + 0.2 * 2)


def test_two_half_rows_earn_the_joints_of_one():
    # Two set tiles on each side, each pair with a good joint: one row is asked for, so one joint counts (review).
    xs = row_xs()
    split = laid(tile_3=tile_state(xs[2], OUT_Y), tile_4=tile_state(xs[3], OUT_Y))
    assert M.score(split) == pytest.approx(0.4 + 0.2)


def test_the_row_is_the_side_with_more_set_tiles():
    xs = row_xs()
    three_and_one = laid(tile_4=tile_state(xs[3], OUT_Y))
    assert M.score(three_and_one) == pytest.approx(0.4 + 0.2 * 2)


@pytest.mark.parametrize("tilt_deg", [89.9, 90.1])
def test_a_tile_on_its_edge_is_only_not_set(tilt_deg):
    # Three tiles set with two good joints, the fourth standing on its edge either side of 90 degrees (review).
    xs = row_xs()
    assert M.score(laid(tile_4=tile_state(xs[3], ROW_Y - 0.02, tilt_deg=tilt_deg))) == pytest.approx(0.3 + 0.2 * 2)


@pytest.mark.parametrize("override", [{"bottom_z": M.TOP - 0.3}, {"tilt_deg": 180.0}, {"tilt_deg": 136.0}])
def test_a_dropped_or_face_down_tile_scores_zero(override):
    xs = row_xs()
    assert M.score(laid(tile_1=tile_state(xs[0], ROW_Y, **override))) == 0.0


def test_a_moving_tile_is_not_yet_set():
    xs = row_xs()
    assert not M.success(laid(tile_2=tile_state(xs[1], ROW_Y, moving=True)))


def _landed(waypoints):
    """Where each tile ends up in an official run: let go at each release point, then shifted and turned as measured
    (DEMO_LANDING, plus DEMO_OFFICIAL_X along x)."""
    releases = [w for w in waypoints if w.label == "release"]
    out = {}
    for name, w in zip(M.NAMES, releases, strict=True):
        dx, dy, dturn = M.DEMO_LANDING[name]
        dx += M.DEMO_OFFICIAL_X[name]
        out[name] = tile_state(w.pos[0] + dx / 1000, w.pos[1] + dy / 1000, bottom_z=w.pos[2] - M.DROP - M.GRIP_ABOVE,
                               turn_deg=math.degrees(w.yaw) + dturn)
    return out


def test_the_demo_sets_the_tiles_down_where_they_score_full():
    tiles = _landed(M.demo()["left"])
    assert len(tiles) == 4
    assert all(abs(float(t.pos[2]) - (M.TOP + M.TILE[2] / 2)) < 1e-9 for t in tiles.values())
    assert all(M._turn_deg(t) < 1e-6 for t in tiles.values())
    assert M.success(scene(**tiles)) and M.score(scene(**tiles)) == 1.0
    joints = M._joints(scene(**tiles))
    assert joints == pytest.approx([M.DEMO_JOINT] * 3, abs=1e-6)  # the middle of the 2-4 mm window


def test_without_the_landing_correction_the_demo_scores_what_it_did_on_the_official_stack():
    # Let go square at the aimed spots, the tiles land where the 2026-11-03 traces found them: 0.6.
    plain = {name: tile_state(x + dx / 1000, M.EDGE_IN - W / 2 - M.DEMO_EDGE_GAP + dy / 1000, turn_deg=dturn)
             for (name, (dx, dy, dturn)), x in zip(M.DEMO_LANDING.items(),
                                                   (M.FIRST_X - i * (W + M.DEMO_JOINT) for i in range(4)), strict=True)}
    assert M.score(scene(**plain)) == pytest.approx(0.6)


def test_every_tile_starts_turned_more_than_the_2_degree_limit():
    for name in M.NAMES:
        assert M._turn_deg(scene()[name]) > M.MAX_TURN_DEG + 5


def test_the_turned_tiles_start_clear_of_each_other():
    reach = [W / 2 * (math.cos(math.radians(t)) + math.sin(math.radians(t))) for t in M.SUPPLY_TURN_DEG]
    for i in range(len(M.NAMES) - 1):
        assert abs(M.SUPPLY_X[i] - M.SUPPLY_X[i + 1]) - reach[i] - reach[i + 1] > 0.002


def test_the_demo_grips_each_tile_at_its_turn_and_sets_it_down_square():
    waypoints = M.demo()["left"]
    grips = [w.yaw for w in waypoints if w.label == "close"]
    releases = [w.yaw for w in waypoints if w.label == "release"]
    assert grips == pytest.approx([math.radians(t) for t in M.SUPPLY_TURN_DEG])
    # square, less the turn each tile was measured to pick up on the way down
    assert releases == pytest.approx([-math.radians(M.DEMO_LANDING[n][2]) for n in M.NAMES])


def test_the_demo_fingers_clear_the_straightedge():
    grips = [w.pos[2] for w in M.demo()["left"] if w.label in ("descend", "lower", "release")]
    tips = min(grips) - PAD_TIP_PAST_TCP
    assert tips >= M.TOP + M.EDGE.size[2] + 0.003


@pytest.mark.parametrize("arm", list(M.demo()))
def test_the_demo_is_within_reach(arm):
    plan = ArmPlan(M.demo()[arm], station.BASES[arm])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_the_official_x_shifts_recentre_the_joints_the_official_runs_measured():
    # a shift of -s moves a tile s mm out: its far-side joint narrows by s, its near-side joint widens by s
    def recentred(measured, before, after):
        d = {n: after[n] - before[n] for n in M.NAMES}
        return [measured[0] + d["tile_2"] - d["tile_1"], measured[1] + d["tile_3"] - d["tile_2"],
                measured[2] + d["tile_4"] - d["tile_3"]]

    none = {n: 0.0 for n in M.NAMES}
    first = {"tile_1": 0.0, "tile_2": -0.84, "tile_3": -0.97, "tile_4": -0.30}
    # mm, two official runs on 2026-11-03: with DEMO_LANDING alone, then with the first round of shifts
    assert recentred([3.84, 3.13, 2.33], none, first) == pytest.approx([3.0, 3.0, 3.0], abs=0.01)
    assert recentred([3.21, 2.38, 3.31], first, M.DEMO_OFFICIAL_X) == pytest.approx([3.0, 3.0, 3.0], abs=0.01)


def test_the_demo_aims_unshifted():
    assert M.DEMO_AIM_SHIFT_MM == 0.0
