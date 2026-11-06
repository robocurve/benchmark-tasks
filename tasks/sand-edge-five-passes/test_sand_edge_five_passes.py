"""Offline checks (no GPU): the task file loads, the layout (the workpiece in the jaws' gap, the arms' halves), the
pass counter, every scoring outcome and the episode's memory, the two ways to score 0 (the workpiece slipped, the
block dug in tilted), the looks fit their boxes, and the demo sweeps five full strokes within reach. Run in the
yam_arena environment: python -m pytest -q test_sand_edge_five_passes.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "sand_edge_five_passes.py")
T = TASK.module
DT = 0.05  # s: one control step at 20 Hz

FLAT = np.array([0.0, 0.0, 0.0, 1.0])


def _tilt(deg, axis=(1.0, 0.0, 0.0)):
    a = math.radians(deg) / 2
    return np.array([*(math.sin(a) * np.asarray(axis)), math.cos(a)])


def _state(pos, quat=FLAT):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - 0.02, high=pos + 0.02)


PIECE0 = np.asarray(T.PIECE_START)
Y_RIGHT, Y_LEFT = T.PIECE_Y - T.HALF_EDGE, T.PIECE_Y + T.HALF_EDGE


def _on_edge(y, gap=0.0, quat=FLAT):
    """The block on the edge, its centre at ``y``: turned about the middle of its sanded face, which stays ``gap``
    above the edge's top (upside down, its other face rests there)."""
    from yam_arena.urdf import quat_xyzw_to_matrix

    up = quat_xyzw_to_matrix(np.asarray(quat, float)) @ np.array([0, 0, T.BLOCK_SIZE[2] / 2])
    return _state((T.EDGE_X, y, T.EDGE_TOP + gap + abs(up[2])), quat)


def _scene(block=None, piece=None):
    return {"sanding_block": block if block is not None else _state((*T.BLOCK_XY, T.BLOCK.pos[2])),
            "workpiece": piece if piece is not None else _state(PIECE0)}


def _stroke(y0, y1, steps=40, gap=0.0, quat=FLAT, piece=None):
    for y in np.linspace(y0, y1, steps):
        T.observe(_scene(_on_edge(y, gap, quat), piece), DT)


A, B = Y_RIGHT - 0.01, Y_LEFT + 0.01  # the demo's turning points, 1 cm past each end


@pytest.fixture(autouse=True)
def fresh_episode():
    T.reset_memory()


# -- the scene -------------------------------------------------------------------------------------------------------


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"workpiece", "sanding_block"}


def test_workpiece_stands_in_the_jaws_gap():
    gap_world = (T.FIXED_FACE + (T.SLIDING_FACE + T.JAW_SHIFT)) / 2 * T.VISE_SCALE + T.VISE_XY[0]
    assert gap_world == pytest.approx(T.EDGE_X)
    gap = (T.SLIDING_FACE + T.JAW_SHIFT - T.FIXED_FACE) * T.VISE_SCALE
    assert gap - T.PIECE_T == pytest.approx(2 * T.JAW_PLAY) and T.JAW_PLAY >= 0.0015 - 1e-9
    assert T.EDGE_TOP - T.JAW_TOP == pytest.approx(0.03)
    bottom = T.PIECE_START[2] - T.PIECE_SIZE[2] / 2
    screw_top = T.TOP + 0.0963 * T.VISE_SCALE  # the vise's screw, under the gap, measured in its file
    assert bottom - screw_top >= 0.009 and bottom < T.TOP + 0.145 * T.VISE_SCALE  # 1 cm over the screw, below the plates


def test_arms_work_in_their_own_halves():
    assert T.PIECE_Y + T.HALF_EDGE < -0.1 and T.BLOCK_XY[1] < -0.2  # the right arm's side
    for prop in (T.SQUARE, T.GLASSES):
        assert prop.pos[1] > 0.1  # on the left, out of the right arm's way


def test_block_fits_the_fingers():
    assert T.BLOCK_SIZE[0] < 0.094 - 0.015  # gripped across its 7 cm, the fingers open 9.4 cm
    assert T.BLOCK_SIZE[1] / 2 > T.END_ZONE + T.OVERRUN  # at a turning point the block still covers the edge


def test_looks_fit_their_boxes():
    for file, size in (("workpiece.usdc", T.PIECE_SIZE), ("block.usdc", T.BLOCK_SIZE)):
        assert (HERE / "looks" / file).is_file()
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom

    for file, size in (("workpiece.usdc", T.PIECE_SIZE), ("block.usdc", T.BLOCK_SIZE)):
        stage = Usd.Stage.Open(str(HERE / "looks" / file))
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
        box = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
        half = np.asarray(size) / 2 + 1e-6
        assert np.all(np.abs(np.asarray(box.GetMin())) <= half) and np.all(np.abs(np.asarray(box.GetMax())) <= half)


# -- contact geometry ------------------------------------------------------------------------------------------------


def test_face_on_edge():
    assert T.face_on_edge(_scene(_on_edge(T.PIECE_Y)))
    assert T.face_on_edge(_scene(_on_edge(T.PIECE_Y, gap=0.0009)))
    assert not T.face_on_edge(_scene(_on_edge(T.PIECE_Y, gap=0.0015)))
    assert T.face_on_edge(_scene(_on_edge(Y_RIGHT - 0.04)))  # 1 cm of the block still over the edge
    assert not T.face_on_edge(_scene(_on_edge(Y_RIGHT - 0.06)))  # past the end: off the edge
    assert not T.face_on_edge(_scene()) and not T.block_on_edge(_scene())  # lying on the table


def test_upside_down_block_is_not_its_face():
    over = _on_edge(T.PIECE_Y, quat=_tilt(180))
    assert T.block_on_edge(_scene(over)) and not T.face_on_edge(_scene(over))
    assert T.tilt_deg(_scene(over)) == pytest.approx(180)


def test_end_zones():
    assert T.end_zone(_scene(_on_edge(Y_RIGHT + 0.0099))) == "a"
    assert T.end_zone(_scene(_on_edge(Y_RIGHT + 0.011))) == ""
    assert T.end_zone(_scene(_on_edge(Y_LEFT - 0.0099))) == "b"
    assert T.end_zone(_scene(_on_edge(A))) == "a" and T.end_zone(_scene(_on_edge(B))) == "b"


# -- passes and scoring ----------------------------------------------------------------------------------------------


def test_start_scores_zero():
    T.observe(_scene(), DT)
    assert T.score(_scene()) == 0.0 and not T.success(_scene())


def test_each_full_stroke_is_a_pass():
    for i in range(5):
        _stroke(A, B) if i % 2 == 0 else _stroke(B, A)
        assert T.STATE["passes"] == i + 1
        assert T.score(_scene()) == pytest.approx(0.2 * (i + 1))
    assert T.success(_scene())


def test_more_than_five_still_one():
    for i in range(7):
        _stroke(A, B) if i % 2 == 0 else _stroke(B, A)
    assert T.score(_scene()) == 1.0 and T.success(_scene())


def test_partial_stroke_does_not_count():
    _stroke(A, Y_LEFT - 0.02)  # stops 2 cm short of the far end
    _stroke(Y_LEFT - 0.02, A)
    assert T.STATE["passes"] == 0


def test_lifting_mid_stroke_breaks_it():
    _stroke(A, T.PIECE_Y)
    T.observe(_scene(_on_edge(T.PIECE_Y, gap=0.005)), DT)  # lifted 5 mm for one step
    _stroke(T.PIECE_Y, B)
    assert T.STATE["passes"] == 0
    _stroke(B, A)  # a whole stroke back counts
    assert T.STATE["passes"] == 1


def test_tilted_block_does_not_count():
    _stroke(A, B, quat=_tilt(12))  # rocked 12 degrees across the edge
    assert T.STATE["passes"] == 0
    _stroke(B, A, quat=_tilt(8))  # 8 degrees is flat enough
    assert T.STATE["passes"] == 1


def test_stroke_started_mid_edge_does_not_count():
    _stroke(T.PIECE_Y, B)
    assert T.STATE["passes"] == 0


def test_slipped_workpiece_scores_zero_for_good():
    _stroke(A, B)
    _stroke(B, A)
    pushed = _state(PIECE0 + (0.0, 0.0, -0.005))  # pressed 5 mm down into the vise
    T.observe(_scene(piece=pushed), DT)
    assert T.STATE["failed"] and T.score(_scene(piece=pushed)) == 0.0
    for i in range(5):  # passes after that don't help, even with the piece back
        _stroke(A, B) if i % 2 == 0 else _stroke(B, A)
    assert T.score(_scene()) == 0.0 and not T.success(_scene())


def test_small_slip_is_allowed():
    nudged = _state(PIECE0 + (0.0, 0.004, -0.002))
    _stroke(A, B, gap=-0.002, piece=nudged)  # the block follows the edge 2 mm down
    assert T.STATE["passes"] == 1 and not T.STATE["failed"]


def test_turned_workpiece_slipped():
    turned = _state(PIECE0, _tilt(3.0, (1.0, 0.0, 0.0)))  # in its plane: its edge's ends move 5 mm or more
    assert T.piece_moved(_scene(piece=turned))


def test_steep_press_for_half_a_second_scores_zero():
    steep = _on_edge(T.PIECE_Y, quat=_tilt(30))
    steps = round(T.GOUGE_S / DT)
    for _ in range(steps - 1):
        T.observe(_scene(steep), DT)
    assert not T.STATE["failed"]
    T.observe(_scene(_on_edge(T.PIECE_Y)), DT)  # flat again: the timer starts over
    for _ in range(steps - 1):
        T.observe(_scene(steep), DT)
    assert not T.STATE["failed"]
    T.observe(_scene(steep), DT)
    assert T.STATE["failed"] and T.score(_scene()) == 0.0


def test_steep_block_off_the_edge_is_fine():
    held = _state((T.EDGE_X, T.PIECE_Y, T.EDGE_TOP + 0.05), _tilt(40))
    for _ in range(20):
        T.observe(_scene(held), DT)
    assert not T.STATE["failed"]


def test_reset_forgets():
    _stroke(A, B)
    T.STATE["failed"] = "x"
    T.reset_memory()
    assert T.STATE == {"passes": 0, "from": "", "gouge_s": 0.0, "failed": ""}


def test_not_finite_scores_zero():
    bad = _state((np.nan, 0.0, 0.0))
    assert T.score(_scene(bad)) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _plan():
    return ArmPlan(T.demo()["right"], BASES["right"])


def test_demo_only_right_arm():
    assert set(T.demo()) == {"right"}  # the left arm holds READY on its side


def test_demo_strokes_turn_past_both_ends():
    strokes = [w for w in T.demo()["right"] if w.label.startswith("stroke")]
    assert len(strokes) == 5
    for w in strokes:
        y = w.pos[1]
        assert y <= Y_RIGHT - 0.01 + 1e-9 or y >= Y_LEFT + 0.01 - 1e-9
        assert w.pos[0] == pytest.approx(T.EDGE_X) and w.pos[2] < T.EDGE_TOP + T.FACE_BELOW_TCP


def test_demo_within_reach():
    """Every pose of the demo solves within 2 mm and 2 degrees, the sweep's low TCP included."""
    plan = _plan()
    for t in np.arange(0.0, plan.duration + 0.05, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (plan.worst_ik_mm, plan.worst_ik_deg)


def test_workpiece_hold():
    assert T.PIECE.hold == pytest.approx(30.0) and math.isinf(T.PIECE.twist)  # slips under a 30 N push
    assert T.PIECE_FRICTION[0] < 1.0  # sandpaper on plywood, not the bench's rubber-like friction
