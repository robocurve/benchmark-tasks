"""Offline checks of the mint-box task: the scoring rule, the counting, the lid's snap, the shapes and both layouts.

    python -m pytest -q test_scoring.py        (needs yam_arena importable; no simulator is started)
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))


def load(lot: str):
    from yam_arena import task_file

    os.environ["MINT_LOT"] = lot
    sys.modules.pop("task", None)
    return task_file.load(str(HERE / "task.py")).module


@pytest.fixture(params=["A", "B"])
def T(request):
    return load(request.param)


@pytest.fixture
def A():
    return load("A")


# -- the scoring rule ------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("in_dish, spilled, shut, down, want", [
    (2, 0, True, True, 1.0),
    (2, 0, False, True, 0.5),         # lid left open
    (2, 0, True, False, 0.5),         # box still held, or left in the dish
    (2, 0, False, False, 0.5),
    (1, 0, True, True, 0.5),
    (3, 0, True, True, 0.5),
    (1, 0, False, True, 0.25),
    (3, 0, True, False, 0.25),
    (0, 0, True, True, 0.0),          # nothing done
    (4, 0, True, True, 0.0),
    (11, 0, True, True, 0.0),         # the box tipped out
    (2, 1, True, True, 0.25),         # two in the dish, one on the table
    (2, 3, False, False, 0.25),
    (1, 1, True, True, 0.0),
    (3, 1, True, True, 0.0),
    (0, 2, True, True, 0.0),
])
def test_score_ladder(A, in_dish, spilled, shut, down, want):
    assert A.score_from(in_dish, spilled, shut, down) == want


def test_every_outcome_is_scored(A):
    seen = set()
    for n in range(0, A.COUNT + 1):
        for spilled in range(0, A.COUNT + 1 - n):
            for shut in (True, False):
                for down in (True, False):
                    seen.add(A.score_from(n, spilled, shut, down))
    assert seen == {0.0, 0.25, 0.5, 1.0}


def test_score_never_rises_with_a_spill_or_an_open_lid(A):
    for n in range(0, 6):
        clean = A.score_from(n, 0, True, True)
        assert A.score_from(n, 1, True, True) <= clean
        assert A.score_from(n, 0, False, True) <= clean
        assert A.score_from(n, 0, True, False) <= clean


def test_start_scores_nothing(T):
    pos, quat = T.box_start()
    mints = [T.mint_start(k)[0] for k in range(T.COUNT)]
    assert T.tally(mints, pos, quat) == (0, T.COUNT, 0)
    assert T.score_from(0, 0, True, True) == 0.0
    assert T.score(None) == 0.0 and not T.success(None)


# -- counting ----------------------------------------------------------------------------------------------------------
def test_tally_sorts_mints_by_where_they_are(T):
    K = T.K
    pos, quat = T.box_start()
    dish = np.array([T.DISH_XY[0], T.DISH_XY[1], T.TOP + K.DISH_FLOOR + K.MINT_D / 2])
    inside = [T.mint_start(k)[0] for k in range(5)]
    in_dish = [dish, dish + [0.03, -0.04, 0.0], dish + [K.DISH_IN / 2 - 0.001, 0.0, 0.0]]
    spilled = [dish + [K.DISH_IN / 2 + 0.004, 0.0, 0.0],              # on the rim
               dish + [0.0, 0.0, K.DISH_H],                           # in the air above the dish
               np.array([0.40, -0.20, T.TOP + K.MINT_D / 2]),         # on the table
               np.array([0.40, -0.20, 0.10])]                         # on the floor
    assert T.tally(inside + in_dish + spilled, pos, quat) == (3, 5, 4)


def test_a_mint_in_a_box_held_over_the_dish_is_in_the_box(T):
    K = T.K
    quat = K._quat(K.lying_rot(0.3))
    pos = np.array([T.DISH_XY[0], T.DISH_XY[1], T.TOP + 0.012])
    assert T.tally([K.place(pos, quat, (0.0, 0.0, 0.01))], pos, quat) == (0, 1, 0)


def test_box_down_needs_table_rest_and_clear_grippers(A):
    assert A.box_down(0.0005, 0.0, 0.0, 0.05, False)
    assert not A.box_down(0.0005, 0.0, 0.0, 0.029, False)        # a gripper within 3 cm
    assert not A.box_down(0.004, 0.0, 0.0, 0.05, False)          # not on the table
    assert not A.box_down(0.0005, 0.03, 0.0, 0.05, False)        # still moving
    assert not A.box_down(0.0005, 0.0, 0.6, 0.05, False)
    assert not A.box_down(0.0005, 0.0, 0.0, 0.05, True)          # left in the dish


# -- the lid -----------------------------------------------------------------------------------------------------------
def test_lid_angle_round_trip(A):
    K = A.K
    pos, quat = A.box_start()
    for deg in (0.0, 5.0, 30.0, 90.0, K.LID_MAX):
        _, fq = K.flap_pose(pos, quat, deg)
        assert K.lid_angle(quat, fq) == pytest.approx(deg, abs=1e-6)


def test_open_flap_stands_out_from_the_lid_end(A):
    K = A.K
    pos, quat = A.box_start()
    fp, fq = K.flap_pose(pos, quat, 90.0)
    tip = K.to_local(K.place(fp, fq, (K.FLAP_L + K.LIP, 0.0, 0.0)), pos, quat)
    assert tip[2] == pytest.approx(K.H / 2 + K.FLAP_L + K.LIP) and tip[0] == pytest.approx(K.HINGE[0])


def test_snap_holds_the_lid_shut_and_open(A):
    K = A.K
    assert K.lid_torque(0.0) == pytest.approx(-K.T_SHUT)
    assert K.lid_torque(K.OVER - K.RAMP) == pytest.approx(-K.T_SHUT)
    assert K.lid_torque(K.OVER + K.RAMP) == pytest.approx(K.T_OPEN)
    assert K.lid_torque(K.LID_MAX) == pytest.approx(K.T_OPEN)
    angles = np.linspace(0.0, K.LID_MAX, 221)
    torque = [K.lid_torque(a) for a in angles]
    assert all(b >= a for a, b in zip(torque, torque[1:]))          # one changeover, no second catch
    assert K.SHUT_AT < K.OVER - K.RAMP                               # "shut" lies inside the range that is held shut


def test_a_fingertip_can_work_the_snap(A):
    K = A.K
    lever = K.FLAP_L + K.LIP / 2
    assert 0.3 < K.T_SHUT / lever < 1.5        # N at the lip to open it: a real resistance, well under the 3 N a fingertip gives
    assert K.T_OPEN / (K.FLAP_L / 2) < 1.5     # N at the middle of the flap to start closing it
    # the lip still overhangs the side when the flap has passed the changeover, so a finger drawn along the side gets it there
    over = K.OVER + K.RAMP
    assert K.HINGE[0] + (K.FLAP_L + K.LIP) * math.cos(math.radians(over)) > K.W / 2 + 0.001


# -- the shapes ----------------------------------------------------------------------------------------------------------
def test_shell_is_inside_its_outline_and_has_one_opening(A):
    K = A.K
    half = np.array([K.W, K.T, K.H]) / 2
    for _, centre, size in K.box_boards():
        assert np.all(np.abs(centre) + np.array(size) / 2 <= half + 1e-9)

    def solid(p):
        return any(np.all(np.abs(np.asarray(p) - c) <= np.array(s) / 2 + 1e-9) for _, c, s in K.box_boards())

    o = K.opening()
    assert not solid(o)
    assert not solid(o + [K.OPEN_W / 2 - 0.0005, K.OPEN_T / 2 - 0.0005, 0.0])
    assert solid(o + [0.0, K.OPEN_T / 2 + 0.0005, 0.0]) and solid(o - [K.OPEN_W / 2 + 0.0005, 0.0, 0.0])
    # the rest of the top end is closed: sample it
    z = K.H / 2 - K.FLAP_T - K.TOP_T / 2
    for x in np.linspace(-K.W / 2 + 0.0015, K.W / 2 - 0.0015, 40):
        for y in np.linspace(-K.T / 2 + 0.0015, K.T / 2 - 0.0015, 12):
            inside_opening = abs(x - o[0]) < K.OPEN_W / 2 and abs(y) < K.OPEN_T / 2
            assert solid((x, y, z)) != inside_opening


def test_opening_passes_one_mint_at_a_time(A):
    K = A.K
    assert K.MINT_D < K.OPEN_T < 2 * K.MINT_D and K.MINT_D < K.OPEN_W < 2 * K.MINT_D
    assert K.T - 2 * K.WALL >= 2 * K.MINT_D    # two layers of mints fit in the box's thickness


def test_closed_flap_covers_the_opening(A):
    K = A.K
    (_, centre, size), = K.flap_boards()
    lo = np.array(K.HINGE) + centre - np.array(size) / 2
    hi = np.array(K.HINGE) + centre + np.array(size) / 2
    o = K.opening()
    assert lo[0] <= o[0] - K.OPEN_W / 2 and hi[0] >= o[0] + K.OPEN_W / 2
    assert lo[1] <= -K.OPEN_T / 2 and hi[1] >= K.OPEN_T / 2
    assert hi[2] == pytest.approx(K.H / 2) and hi[0] == pytest.approx(K.W / 2 + K.LIP)


def test_mint_is_as_dense_as_sugar(A):
    K = A.K
    r = K.MINT_D / 2
    volume = math.pi * r * r * (K.MINT_L - 2 * r) + 4 / 3 * math.pi * r ** 3
    assert K.MINT_MASS / volume == pytest.approx(1590.0, rel=0.08)      # sucrose: 1.59 g/cm3


def test_mints_start_inside_the_box_without_touching(T):
    K = T.K
    grid = K.mint_grid(T.COUNT)
    inner = np.array([K.W / 2 - K.WALL, K.T / 2 - K.WALL, K.H / 2 - K.WALL])
    for p, rot in grid:
        ends = [p + rot[:, 0] * s * (K.MINT_L / 2 - K.MINT_D / 2) for s in (-1, 1)]
        for e in ends:
            assert np.all(np.abs(e) + K.MINT_D / 2 <= inner + 1e-9)
            assert e[2] + K.MINT_D / 2 < K.H / 2 - K.FLAP_T - K.TOP_T
    for i, (p, _) in enumerate(grid):
        for q, _ in grid[i + 1:]:
            d = np.abs(p - q)
            assert d[0] >= K.MINT_D - 1e-9 or d[2] >= K.MINT_L - 1e-9
    pos, quat = T.box_start()
    for k in range(T.COUNT):
        assert K.in_box(T.mint_start(k)[0], pos, quat)
        assert not K.in_dish(T.mint_start(k)[0], T.DISH_XY, T.TOP)


def test_dish_holds_a_mint_and_its_boards_join(A):
    K = A.K
    boards = K.dish_boards((0.2, 0.1))
    assert len(boards) == 5
    top = max(c[2] + s[2] / 2 for _, c, s in boards)
    assert top == pytest.approx(K.DISH_H)
    assert K.in_dish((0.2, 0.1, 0.008), (0.2, 0.1))
    assert not K.in_dish((0.2 + K.DISH_IN / 2 + 0.001, 0.1, 0.008), (0.2, 0.1))
    assert not K.in_dish((0.2, 0.1, K.DISH_H + 0.001), (0.2, 0.1))


# -- the layouts ---------------------------------------------------------------------------------------------------------
def test_layouts_differ_and_the_box_is_clear_of_the_dish(T):
    K = T.K
    pos, quat = T.box_start()
    corners = [K.place(pos, quat, (sx * K.W / 2, sy * K.T / 2, sz * K.H / 2)) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    assert not any(K.over_dish(c, T.DISH_XY, margin=0.02) for c in corners)
    assert min(c[2] for c in corners) - T.TOP == pytest.approx(T.GAP)
    assert K._rotation(quat)[2, 1] == pytest.approx(1.0)          # lying on a big face
    assert K._rotation(quat)[0, 2] < -0.9                         # lid end toward the robot
    assert len(T.OBJECTS) == 5 + 2 + T.COUNT
    assert len({o.name for o in T.OBJECTS}) == len(T.OBJECTS)


def test_the_two_layouts_are_not_the_same():
    a, b = load("A"), load("B")
    assert a.BOX_XY != b.BOX_XY and a.DISH_XY != b.DISH_XY and a.COUNT != b.COUNT and a.BOX_YAW != b.BOX_YAW
    assert a.INSTRUCTION == b.INSTRUCTION and a.WANT == b.WANT == 2


# -- the demo's geometry -------------------------------------------------------------------------------------------------
def test_demo_holds_the_opening_over_the_dish(T):
    import demo as D

    K = T.K
    pos, quat = T.box_start()
    # at the grasp the held box's axes are the lying box's
    assert np.allclose(D.left_rot(T, 0.0) @ D.HELD, K._rotation(quat), atol=1e-9)
    for lean, roll in ((0.0, 0.0), (35.0, 0.0), (60.0, 15.0)):
        tool, rot = D.pour_tool(T, lean, roll)
        box_rot = rot @ D.HELD
        centre = tool + rot @ np.array([0.0, -D.L_CLOSE * D.GRIP_Z, D.GRIP_UP - K.T / 2])
        mouth = centre + box_rot @ K.opening()
        assert np.allclose(mouth, [T.DISH_XY[0], T.DISH_XY[1], T.TOP + D.DROP_H], atol=1e-9)
        if lean:
            assert box_rot[2, 2] < 0.0            # the lid end is below the bottom end
    assert D.DROP_H > K.DISH_H + 0.015            # the opening is held clear above the rim


def test_demo_strokes_parse(A):
    import demo as D

    for plan in [*D.STROKES.values(), *D.CONTROL.values()]:
        for token in plan.split(","):
            assert token[0] in "LRSTW" and all(float(v) >= 0 for v in token[1:].split(":"))
        assert plan.split(",")[-1].startswith("L0:")           # every plan ends with the box level again
    assert set(D.STROKES) == {"A", "B"}


# -- task.json -----------------------------------------------------------------------------------------------------------
def test_task_json(A):
    path = HERE / "task.json"
    if not path.exists():
        pytest.skip("task.json not written yet")
    j = json.loads(path.read_text())
    for key in ("title", "category", "instruction", "setup", "scoring", "run_limit", "fidelity", "scores"):
        assert j.get(key), key
    assert "media" not in j
    assert j["instruction"] == A.INSTRUCTION
    assert len(j["title"]) <= 120 and len(j["fidelity"]) <= 500 and len(j.get("context", "")) <= 1000 and len(j["scores"]) <= 300
    assert j["category"] == "Societal relatability"
