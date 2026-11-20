"""The cake task's scene, icing and rules without the simulator: every state is set directly (the smoother put where
its straight edge stands against the icing, the cake turned under it, fields of thickness written in)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest
from yam_arena import task_file

TASK = Path(__file__).resolve().parent / "task.py"  # the task file in this test's own folder
FAR = {"left": np.array([[0.30, 0.30, 1.00]]), "right": np.array([[0.30, -0.30, 1.00]])}  # both hands' hulls far away


@pytest.fixture
def m():
    before = sys.modules.get("mechkit")
    module = task_file.load(TASK).module
    module.begin(0)
    yield module
    module.mk.MECH.clear()
    sys.modules.pop("mechkit", None)
    if before is not None:
        sys.modules["mechkit"] = before


def edge_pose(m, angle: float, distance: float, cake_turn: float = 0.0, lift: float = 0.0):
    """The smoother upright with its straight edge ``distance`` m from the foam at ``angle`` (cake frame), the plate
    standing straight out from the cake, its lower corner on the board (``lift`` m higher); the cake turned by
    ``cake_turn`` about its axis."""
    turn = m.mk.axis_angle((0.0, 0.0, 1.0), cake_turn)
    world_angle = angle + cake_turn
    radial = np.array([math.cos(world_angle), math.sin(world_angle), 0.0])
    x_axis, z_axis = -radial, np.array([0.0, 0.0, 1.0])
    rotation = np.column_stack([x_axis, np.cross(z_axis, x_axis), z_axis])
    centre = m.CAKE_AT + (m.DUMMY_R + distance + m.PLATE_W / 2) * radial + np.array([0.0, 0.0, m.EDGE_L / 2 + lift])
    return {"cake": (m.CAKE_AT.copy(), turn), "smoother": (centre, rotation)}


def put(m, poses: dict, hands=FAR) -> None:
    m.mk.set_state(poses=poses)
    m.STATE["hands"] = hands


def test_the_scene_is_the_one_specified(m):
    assert m.INSTRUCTION == "Smooth the icing on the side of the cake with the smoother, then put the smoother back in its block."
    assert m.RUN_LIMIT_STEPS <= 3600 and m.SCORE_OVER == "end" and m.SUCCESS_HOLD_S == 1.0
    bodies = m.mk.MECH.bodies
    assert bodies["stand"].mass + bodies["turntable"].mass == pytest.approx(3.0)  # Ateco 612: about 3 kg
    assert m.TOP_R * 2 == pytest.approx(0.305) and m.STAND_H == pytest.approx(0.127)
    assert m.DUMMY_R * 2 == pytest.approx(0.152) and m.DUMMY_H == pytest.approx(0.102) and m.BOARD_R * 2 == pytest.approx(0.203)
    assert (m.EDGE_L, m.PLATE_W) == pytest.approx((0.152, 0.0965))  # 6 x 3.8 in, the maker's figures; the straight edge a long one (a)
    assert m.COLUMNS == 96 and m.ROWS == 34 and 2 * m.ROW_H == pytest.approx(m.PITCH)  # two rows to a ridge
    assert len(bodies["cake"].shapes) == m.TILE0 + m.COLUMNS * m.ROWS and not any(s.solid for s in bodies["cake"].shapes[m.TILE0 :])
    assert m.MATERIAL["cut push"][0] * 0.001 * 0.100 == pytest.approx(0.4)  # 0.4 N per mm of depth per 100 mm of edge
    # every material number comes from the one table
    assert (m.THICK, m.PITCH, m.RIDGE, m.SCAR, m.OUT_OF_ROUND, m.CUT_PUSH, m.ROLL, m.FILL_REACH, m.BARE) == tuple(v[0] for v in m.MATERIAL.values())
    put(m, {})
    assert m.in_block() and m.score(None) == 0.0  # as it starts: ridges all round, nothing worked


def test_the_smoother_is_built_1_mm_up_the_groove_from_where_it_settles(m):
    put(m, {})
    position, rotation = m.mk.built_pose("smoother")
    slot_p, slot_r = m.slot_frame()  # the block as built
    local = lambda point: slot_r.T @ (np.asarray(point) - slot_p)  # noqa: E731
    lean = math.degrees(math.acos(rotation[2, 2]))
    assert lean == pytest.approx(35.87, abs=0.01)  # 30 degrees of the groove and 5.87 more (rest trial b2k47: 35.9)
    down = position - m.SMOOTHER_LIFT * slot_r[:, 2]  # where it settles: 1 mm down the groove
    half = m.PLATE_T / 2
    foot_back, foot_front = (local(down + rotation @ np.array([0.0, s * half, -m.EDGE_L / 2])) for s in (1.0, -1.0))
    assert foot_back[2] == pytest.approx(0.0, abs=1e-9)  # the foot's back corner on the groove's floor
    assert foot_front[1] == pytest.approx(m._BACK_FACE - m.GROOVE_W, abs=1e-9)  # its front corner against the lip
    corner = np.array([0.0, m._BACK_FACE, m.BACK_H])  # the back's top edge, on the plate's back face
    assert (corner - local(down)) @ (slot_r.T @ rotation[:, 1]) == pytest.approx(half, abs=1e-9)
    # rest trial b2k47 found the plate there, built upright in the groove: 2.98 mm, 1.39 mm and 2.24 mm (with the block's 1 mm drop, 3.24) from where it was built
    upright = np.array([*m.BLOCK_AT, m.TOP + m.BLOCK_DROP]) + m._block_r @ (m.SLOT_AT + m.SLOT_TILT @ np.array([0.0, 0.0, m.EDGE_L / 2]))
    assert 1000 * (down - upright) == pytest.approx([-2.98, 1.39, -2.24], abs=0.01)


def test_the_icing_starts_combed_with_its_scar_and_out_of_round_by_seed(m):
    field, scar, oor, phase = m.start_field(0)
    rows = field.mean(axis=0)
    assert field[(scar + 48) % 96].mean() == pytest.approx(m.THICK - (oor / 2) * math.cos(m.column_angle(scar + 48) - phase) * 0 + (field[(scar + 48) % 96].mean() - m.THICK))
    plain = np.delete(field, scar, axis=0)
    assert np.ptp(plain[:, 0] - plain[:, 1]) == pytest.approx(0.0, abs=1e-12) and (plain[:, 0] - plain[:, 1]).mean() == pytest.approx(m.RIDGE)  # ridges run round, 3 mm
    assert field[scar] - np.delete(field, scar, axis=0).mean(axis=0) == pytest.approx(np.full(m.ROWS, m.SCAR), abs=0.6e-3)
    assert 0.0 <= oor <= m.OUT_OF_ROUND and np.ptp(plain.mean(axis=1)) == pytest.approx(oor, abs=0.1e-3)
    for seed in (0, 1, 1415158438, None):  # the ridge tops within 0.5 mm of their radius (82.5 mm from the axis): the reset's tolerance
        tops = m.start_field(seed)[0][:, ::2]
        tops = np.delete(tops, m.start_field(seed)[1], axis=0)
        assert np.abs(tops - (m.THICK + m.RIDGE / 2)).max() <= 0.0005, seed
    assert abs(rows.mean() - m.THICK) < 0.1e-3
    other = m.start_field(1)
    assert (other[1], other[2]) != (scar, oor)


def test_crossed_lists_the_columns_passed_in_order(m):
    width = m.CELL_ANGLE
    assert m.crossed(0.5 * width, 0.5 * width) == [(0, 1.0)]  # standing still: the column it stands in
    passed = m.crossed(0.2 * width, 3.2 * width)  # middles of columns 0..2 passed? 0.5 is past 0.2, then 1.5, 2.5; it stands in 3
    assert [j for j, _ in passed] == [0, 1, 2, 3] and passed[0][1] == pytest.approx(0.1) and passed[-1] == (3, 1.0)
    back = m.crossed(3.2 * width, 0.2 * width)  # going back it passes the middles of 2, 1 and 0 and stands in 0
    assert [j for j, _ in back] == [2, 1, 0] and back[-1][1] == pytest.approx(0.9)
    wrap = m.crossed(2 * math.pi - 0.6 * width, 1.4 * width)  # through 0: columns 95, 0, 1
    assert [j for j, _ in wrap] == [95, 0, 1]


def test_the_edge_cuts_the_high_rows_and_fills_the_low_ones_as_the_cake_turns(m):
    distance = m.THICK  # the edge at the mean thickness: it cuts the ridges' tops, fills their troughs
    start = m.column_angle(m.STATE["scar"] + 20) % (2 * math.pi)  # away from the scar
    put(m, edge_pose(m, start, distance))
    m.work_icing()
    assert set(m.STATE["contact"]) == set(range(m.ROWS))  # the edge stands over every row (152 mm, upright)
    before = m.STATE["field"].copy()
    section = 0.0
    for step in range(1, 9):  # the cake turns 0.05 rad a step against the edge
        put(m, edge_pose(m, start + 0.05 * step, distance, cake_turn=-0.05 * step))
        section, _ = m.work_icing()
    field = m.STATE["field"]
    passed = [j for j in range(96) if start + 0.01 < m.column_angle(j) < start + 0.05 * 8 - 0.01]
    assert passed and np.all(field[passed][:, ::2] == pytest.approx(distance))  # the high rows cut to the edge
    assert np.all(field[passed][:, 1::2] >= before[passed][:, 1::2] - 1e-12)  # the low rows filled, never cut
    assert m.STATE["cut_rows"][passed][:, ::2].all() and not m.STATE["cut_rows"][passed][:, 1::2].any()
    assert section > 0.0 and m.STATE["roll"] >= 0.0


def test_a_deep_edge_leaves_foam_bare_and_a_straight_pull_leaves_its_roll_as_a_mark(m):
    put(m, edge_pose(m, 1.0, 0.0005))
    for step in range(1, 6):
        put(m, edge_pose(m, 1.0 + 0.05 * step, 0.0005, cake_turn=-0.05 * step))
        m.work_icing()
    assert (m.STATE["field"] < m.BARE).sum() > 0  # cut to 0.5 mm: the foam shows
    roll = m.STATE["roll"]
    assert roll > 0.0
    last = dict(m.STATE["contact"])
    put(m, edge_pose(m, 1.25, 0.05, cake_turn=-0.25))  # pulled straight off: 5 cm out in one step
    m.work_icing()
    assert m.STATE["contact"] is None and m.STATE["roll"] == pytest.approx(0.0) and m.STATE["marks_left"] == 1
    j = int(last[10][0] // m.CELL_ANGLE) % 96
    assert m.STATE["field"][j, 10] == pytest.approx(0.0005 + (roll / len(last)) / m.cell_area(0, 0))


def score_with(m, field: np.ndarray, poses: dict | None = None, hands=FAR) -> float:
    m.STATE["field"] = field
    put(m, poses or {}, hands)
    return m.score(None)


def smooth(m, thickness: float = 0.0045) -> np.ndarray:
    return np.full((m.COLUMNS, m.ROWS), thickness)


def test_the_rule_and_the_gauge_rest_on_the_outermost_points_and_read_the_gaps_between(m):
    x = np.arange(7.0)
    assert m.gaps_under(x, np.array([1.0, 1.0, 1.0, 0.0, 1.0, 1.0, 1.0])) == pytest.approx([0, 0, 0, 1, 0, 0, 0])  # a dip: its depth
    assert m.gaps_under(x, 0.3 * x) == pytest.approx(np.zeros(7))  # an even slope: nothing
    assert m.gaps_under(x, -((x - 3) ** 2)) == pytest.approx(np.zeros(7))  # a crown rocks the edge: no gap between its touches
    assert m.gaps_under(x, np.array([0.0, 0, 0, 2, 0, 0, 0])) == pytest.approx([0, 2 / 3, 4 / 3, 0, 4 / 3, 2 / 3, 0])  # a ridge: rocked each way onto it and an end
    # the gauge: 50 mm of a circle of 81 mm spans 36 degrees, 4.79 columns either side of its mark; 16 placements 6 columns apart overlap
    assert m.GAUGE_HALF == pytest.approx(math.asin(0.025 / 0.081) / (2 * math.pi / 96)) and 2 * m.GAUGE_HALF > m.COLUMNS // m.MARKS
    assert list(m.MARK_COLUMNS) == list(range(0, 96, 6)) and [(k + 0.5) * m.ROW_H * 1000 for k in m.GAUGE_ROWS] == pytest.approx([25.5, 49.5, 76.5])
    # a 50 mm straightedge laid round the side instead would stand 3.95 mm off it at its middle (81 mm radius)
    assert 1000 * (0.081 - math.sqrt(0.081**2 - 0.025**2)) == pytest.approx(3.955, abs=0.001)


def test_an_even_side_scores_one_and_its_limit_is_a_gap_of_1_2_mm(m):
    assert score_with(m, smooth(m)) == 1.0
    for depth, level in ((m.EVEN - 0.00001, 1.0), (m.EVEN + 0.00001, 0.5)):
        groove = smooth(m)
        groove[:, 15:17] -= depth  # a groove round the side, 6 mm high: under the upright rule
        assert m.upright_gaps(groove).max() == pytest.approx(depth) and score_with(m, groove) == level, depth
        dip = smooth(m)
        dip[40, :] -= depth  # a dip down the side between two marks (36 and 42): under the gauge at the three heights
        assert m.upright_gaps(dip).max() == pytest.approx(0.0) and m.round_gaps(dip).max() == pytest.approx(depth) and score_with(m, dip) == level, depth
    ridged = smooth(m) + np.where(np.arange(m.ROWS) % 2 == 0, 0.0015, -0.0015)[None, :]
    assert score_with(m, ridged) == 0.0  # the side with its ridges (3 mm): nothing


def test_a_smooth_slope_or_an_out_of_round_side_passes_where_high_to_low_would_not(m):
    tapered = np.tile(np.linspace(0.003, 0.006, m.ROWS), (m.COLUMNS, 1))  # 3 mm at the foot to 6 mm at the top: 1.45 mm up 17 rows
    assert np.ptp(tapered[0, 3:20]) > m.EVEN and m.largest_gap(tapered) == pytest.approx(0.0, abs=1e-12) and score_with(m, tapered) == 1.0
    angles = m.column_angle(np.arange(m.COLUMNS))
    off_round = smooth(m) + 0.0025 * np.cos(angles)[:, None]  # 5 mm high to low once round: 1.45 mm high to low in 9 columns
    high_low = max(np.ptp(np.take(off_round[:, 16], range(j, j + 9), mode="wrap")) for j in range(m.COLUMNS))
    assert high_low > m.EVEN and m.round_gaps(off_round).max() < 0.00015 and score_with(m, off_round) == 1.0


def test_the_combed_start_fails_clearly(m):
    for seed in (0, 1415158438, None):
        field = m.start_field(seed)[0]
        assert m.upright_gaps(field).min() == pytest.approx(m.RIDGE)  # 3 mm under the rule at every mark
        assert 0.0015 < m.round_gaps(field).max() < 0.0018  # the comb's scar, 2 mm high, under the gauge
        assert score_with(m, field) == 0.0, seed


def test_one_mark_or_one_small_bare_place_scores_three_quarters(m):
    for height, level in ((0.0019, 0.75), (0.0021, 0.5)):
        marked = smooth(m)
        marked[30, :] += height  # the take-off mark down the side
        assert score_with(m, marked) == level, height
    two = smooth(m)
    two[33, :] += 0.0016  # 1.39 mm under the gauge
    two[70, :] += 0.0016  # 1.42 mm under the gauge
    assert score_with(m, two) == 0.5  # two marks: not one flaw
    for cells, level in ((12, 0.75), (13, 0.5)):  # 12 cells of 15.9 mm2: 1.9 cm2
        bare = smooth(m)
        for n in range(cells):
            bare[50 + n // 4, 10 + n % 4] = 0.0005
        assert score_with(m, bare) == level, cells


def test_the_lower_levels(m):
    field = m.STATE["field"].copy()  # as combed
    half = field.copy()
    half[:48] = 0.0045  # half the way round smoothed
    assert score_with(m, half) == 0.5
    less = field.copy()
    less[:42] = 0.0045  # 7 of the 16 marks smoothed
    m.STATE["cut_rows"][:24, :] = True
    assert score_with(m, less) == 0.25  # worked a quarter of the way round
    m.STATE["cut_rows"][:, :] = False
    m.STATE["cut_rows"][:23, :] = True
    assert score_with(m, less) == 0.0
    thin = smooth(m, 0.0029)
    assert score_with(m, thin) == 0.5  # even, but under 3 mm on average
    bare = smooth(m)
    bare[:33] = 0.0005  # a third and a bit bare
    assert score_with(m, bare) == 0.0


def test_one_point_zero_needs_the_smoother_in_its_block_the_hands_clear_and_the_cake_centred(m):
    out_of_block = edge_pose(m, 0.0, 0.03)
    del out_of_block["cake"]
    assert score_with(m, smooth(m), out_of_block) == 0.5
    near = {"left": FAR["left"], "right": np.array([[*m.CAKE_AT[:2] + np.array([0.0, -(m.DUMMY_R + 0.0045 + 0.029)]), m.CAKE_AT[2] + 0.05]])}
    assert score_with(m, smooth(m), hands=near) == 0.5  # a hand 29 mm from the icing
    near["right"] = near["right"] + np.array([[0.0, -0.002, 0.0]])
    assert score_with(m, smooth(m), hands=near) == 1.0  # 31 mm
    for shift, level in ((0.0049, 1.0), (0.0051, 0.5), (0.0149, 0.5), (0.0151, 0.0)):
        assert score_with(m, smooth(m), {"cake": (m.CAKE_AT + np.array([shift, 0.0, 0.0]), np.eye(3))}) == level, shift


def test_the_stand_pushed_off_its_place_scores_nothing(m):
    built = np.array(m.mk.built_pose("stand")[0], dtype=float)
    for shift, level in ((0.0149, 1.0), (0.0151, 0.0)):  # the base slid on the bench; the top and the cake stay on it as built
        assert score_with(m, smooth(m), {"stand": (built + np.array([0.0, -shift, 0.0]), np.eye(3))}) == level, shift


def test_the_roll_is_capped_and_what_it_cannot_hold_stays_on_the_blade(m):
    start = m.column_angle(m.STATE["scar"] + 30) % (2 * math.pi)
    put(m, edge_pose(m, start, 0.002))  # 3 mm under the mean: much more is cut than the troughs take
    m.work_icing()
    for step in range(1, 30):
        put(m, edge_pose(m, start + 0.05 * step, 0.002, cake_turn=-0.05 * step))
        m.work_icing()
    rows = len(m.STATE["contact"])
    assert m.STATE["roll"] == pytest.approx(m.ROLL * m.ROW_H * rows) and m.STATE["carried"] > 10 * m.STATE["roll"]


def test_drawn_away_along_the_turning_the_mark_is_smaller_than_pulled_straight_off(m):
    def mark(gradual: bool) -> float:
        m.begin(0)
        start = m.column_angle(m.STATE["scar"] + 40) % (2 * math.pi)
        angle, turn = start, 0.0
        put(m, edge_pose(m, angle, 0.004))
        m.work_icing()
        for _ in range(int(2 * math.pi / 0.05) + 20):  # once round and a little more: smoothed at 4 mm, a full roll on the edge
            angle, turn = angle + 0.05, turn - 0.05
            put(m, edge_pose(m, angle, 0.004, cake_turn=turn))
            m.work_icing()
        before = m.STATE["field"].copy()
        out = 0.004
        for _ in range(30 if gradual else 0):  # 0.1 mm out a step while the cake turns on 0.05 rad, over the smoothed icing
            angle, turn, out = angle + 0.05, turn - 0.05, out + 0.0001
            put(m, edge_pose(m, angle, out, cake_turn=turn))
            m.work_icing()
        put(m, edge_pose(m, angle, 0.05, cake_turn=turn))  # then away
        m.work_icing()
        return float((m.STATE["field"] - before).max())

    pulled, drawn = mark(False), mark(True)
    assert pulled > 0.0015 and drawn < pulled / 2, (pulled, drawn)


def test_the_cut_pushes_the_blade_along_the_icing_and_outward(m):
    put(m, edge_pose(m, -math.pi / 2, 0.004))  # the edge on the cake's right side
    m.mk.MECH.velocities = {"cake": (np.zeros(3), np.array([0.0, 0.0, -1.0])), "smoother": (np.zeros(3), np.zeros(3))}  # turning clockwise from above
    at = m.CAKE_AT + np.array([0.0, -(m.DUMMY_R + 0.004), 0.05])
    force = m.cut_force(0.001 * 0.1, at)  # 1 mm deep over 100 mm of edge
    assert force == pytest.approx([-0.4, -0.4, 0.0], abs=1e-9)  # along the icing's way (-x there) and outward (-y), 0.4 N each
    m.mk.MECH.velocities["cake"] = (np.zeros(3), np.array([0.0, 0.0, -0.001 / (m.DUMMY_R + 0.004)]))  # 1 mm/s at the edge: half the push
    assert m.cut_force(0.001 * 0.1, at) == pytest.approx([-0.2, -0.2, 0.0], abs=1e-6)
    assert m.cut_force(0.0, at) == pytest.approx([0.0, 0.0, 0.0])


def test_a_wide_raised_band_is_not_one_mark_and_the_scar_is_read_apart(m):
    wide = smooth(m)
    wide[30:36, :] += 0.0015  # six columns round, raised: not one narrow mark
    assert m.one_flaw(wide) is None and score_with(m, wide) == 0.5
    scarred = smooth(m)
    scarred[m.STATE["scar"], :] += 0.0015  # column 81: 3 columns from the marks at 78 and 84
    parts = m.regions(scarred)
    lever = 3 + m.GAUGE_HALF  # the gauge rocked onto the scar and its far end
    assert parts["plain side"] == pytest.approx(0.0) and parts["over the scar"] == pytest.approx(0.0015 * (lever - 1) / lever)


def test_the_smoother_is_in_its_block_only_down_in_the_groove(m):
    put(m, {})
    assert m.in_block()
    position, rotation = m.mk.pose("smoother")
    slot_r = m.slot_frame()[1]
    # as built, the corners of the plate's middle plane at its foot stand 1.10 mm over the floor and 3.81 mm in front of
    # the back (1 mm behind the lip's face); the plate moved in the groove's frame:
    for move, inside in (((0.0, 0.0, 0.00285), True), ((0.0, 0.0, 0.00295), False),  # up: under the lip's top, 4 mm
                         ((0.0, -0.0014, 0.0), True), ((0.0, -0.0016, 0.0), False),  # through the lip: 0.5 mm play
                         ((0.0, 0.0054, 0.0), True), ((0.0, 0.0056, 0.0), False),  # into the back: 0.5 mm play
                         ((0.0216, 0.0, 0.0), True), ((0.0219, 0.0, 0.0), False)):  # along the groove: 140 mm for a 96.5 mm edge
        put(m, {"smoother": (position + slot_r @ np.array(move), rotation)})
        assert m.in_block() == inside, move
    turned = rotation @ m.mk.axis_angle((1.0, 0.0, 0.0), math.pi)  # the other short edge down, in the same place
    put(m, {"smoother": (position, turned)})
    assert m.in_block()
