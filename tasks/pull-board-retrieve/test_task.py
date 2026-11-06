"""Offline checks for pull-board-retrieve (NumPy and pytest only; no Isaac Sim).

Run from this folder with the yam-isaac-lab-arena package importable (installed, as on the Brev VM, or a clone
named by YAM_ARENA):

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA"]))

from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import Box, Cylinder, ObjectState, load  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = TASK.table_top
HZ = 20.0  # the rate the README's runs use
GRIPPER_MAX = 2 * abs(FINGER_OPEN) + 0.0001  # pads 94 mm apart fully open (constants.py, tests/test_constants.py)
MARGIN = 0.01
IK_STARTS = (
    [0.0, 1.0, 1.0, -0.6, 0.0, 0.0],
    [0.0, 2.0, 2.0, -0.5, 0.0, 0.0],
    [0.0, 2.5, 1.5, 0.0, 0.0, 0.0],
    [0.0, 1.5, 0.5, 0.5, 0.0, 0.0],
    [0.0, 3.0, 0.3, 0.0, 0.0, 0.0],
    [-0.8, 1.5, 1.0, 0.0, 0.0, 0.0],
    [0.8, 1.5, 1.0, 0.0, 0.0, 0.0],
)


# -- helpers -----------------------------------------------------------------------------------------------------------


def state(pos, size, vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, dtype=float), np.asarray(size, dtype=float) / 2
    return ObjectState(pos, np.array([0.0, 0.0, 0.0, 1.0]), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


JAR_UP = (2 * M.JAR_R, 2 * M.JAR_R, M.JAR_H)  # upright bounding box
JAR_LYING = (M.JAR_H, 2 * M.JAR_R, 2 * M.JAR_R)


def jar(x, y, bottom, size=JAR_UP, vel=(0.0, 0.0, 0.0)):
    return state((x, y, bottom + size[2] / 2), size, vel)


def scene(jar_state, board_near_x=M.BOARD_NEAR_X):
    board = state((board_near_x + M.BOARD_LEN / 2, 0.0, TOP + M.BOARD_H / 2), M.BOARD.size)
    objs = {"board": board, "spice_jar": jar_state}
    for b in M.BASKET:
        objs[b.name] = state(b.pos, b.size)
    return objs


def bbox(obj):
    if isinstance(obj, Box):
        half = np.asarray(obj.size) / 2
    elif isinstance(obj, Cylinder):
        half = np.array([obj.radius, obj.radius, obj.height / 2])
    else:
        raise TypeError(obj)
    assert obj.yaw == 0.0
    return np.asarray(obj.pos) - half, np.asarray(obj.pos) + half


def best_ik(target, rot_xyzw, side, iters=300):
    """Smallest (position mm, rotation deg) error over several starting configurations."""
    errs = []
    for q0 in IK_STARTS:
        _, pe, re = ik.solve(target, rot_xyzw, q0, base_pos=station.BASES[side], iters=iters)
        errs.append((pe * 1000, float(np.degrees(re))))
    return min(errs, key=lambda e: e[0] + e[1])


def down(yaw=0.0):
    return ik.matrix_to_quat_xyzw(grip_rotation(yaw))


# -- 1. the file loads ------------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.free_objects == ("board", "spice_jar")
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")


# -- 2. the demo is reachable -----------------------------------------------------------------------------------------


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz():
    plan = TASK.demo()
    assert set(plan) == {"left"}
    arm = ArmPlan(plan["left"], station.BASES["left"])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, (arm.worst_ik_mm, arm.worst_ik_deg)
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)
    assert arm.duration * HZ < 0.5 * 2400  # the demo needs under half the run limit


def test_the_demo_grips_the_board_without_the_pads_touching_the_table_and_the_strokes_are_gentle():
    pts = TASK.demo()["left"]
    board_grips = [w for w in pts if "board" in w.label and w.pos[2] == M.BOARD_GRIP_Z]
    assert board_grips, "the demo never grips the board"
    assert M.BOARD_GRIP_Z - 0.013 > TOP + 0.003  # pad tips (13 mm past the TCP) stay 3 mm or more above the table
    assert M.BOARD_GRIP_Z - 0.013 < TOP + M.BOARD_H - 0.01  # and at least 1 cm of pad overlaps the board
    for w in pts:
        if "slide" in w.label:
            dist = M.GRIP_X - w.pos[0]
            # smoothstep: peak speed 1.5 d / T, peak acceleration 6 d / T^2
            assert 1.5 * dist / w.duration < 0.08 and 6 * dist / w.duration**2 < 0.06


def test_the_demo_picks_the_jar_where_the_two_strokes_leave_it():
    # the pick point is where the strokes leave the jar: the plan, or within 2 cm of it as measured in sim
    assert M.JAR_END_X == pytest.approx(M.JAR_X0 - (M.GRIP_X - M.PULL1_TO) - (M.GRIP_X - M.PULL2_TO), abs=0.02)
    assert M.JAR_END_X <= 0.32  # README: keep it at most 0.32
    first = M.JAR_X0 - (M.GRIP_X - M.PULL1_TO)
    assert first > M.REACH_X  # one stroke is not enough: the jar is still outside the 0.4 zone
    assert M.JAR_END_X <= M.REACH_X - 0.05
    # the board is still under the second grip point, and the jar clears the fingers there
    near_after_1 = M.BOARD_NEAR_X - (M.GRIP_X - M.PULL1_TO)
    assert near_after_1 + 0.02 < M.GRIP_X < near_after_1 + M.BOARD_LEN
    assert first - M.JAR_R - M.GRIP_X > 0.05
    # the board's near end stays on the table, clear of the camera stand's foot (x < -0.18)
    near_after_2 = near_after_1 - (M.GRIP_X - M.PULL2_TO)
    assert near_after_2 > -0.17


# -- 3. scoring of the goal and partial credit ------------------------------------------------------------------------


def test_a_jar_resting_in_the_basket_succeeds_and_scores_one():
    bx, by = M.BASKET_XY
    upright = scene(jar(bx, by, M.FLOOR_TOP), board_near_x=-0.14)
    lying = scene(jar(bx + 0.02, by - 0.03, M.FLOOR_TOP, size=JAR_LYING), board_near_x=-0.14)
    for objs in (upright, lying):
        assert TASK.success(objs) and TASK.score(objs) == 1.0
    assert TASK.distance(upright) == pytest.approx(0.0, abs=1e-9)


@pytest.mark.parametrize(
    "where, objs",
    [
        ("on the board after both strokes", scene(jar(M.JAR_END_X, 0.0, TOP + M.BOARD_H), board_near_x=-0.14)),
        ("knocked off the board onto the table", scene(jar(0.25, 0.10, TOP, size=JAR_LYING), board_near_x=-0.14)),
        ("held in the gripper above the table", scene(jar(0.30, 0.10, TOP + 0.09), board_near_x=-0.14)),
        ("dropped in the basket, still moving", scene(jar(*M.BASKET_XY, M.FLOOR_TOP, vel=(0, 0, -0.1)), -0.14)),
        ("on the basket's rim", scene(jar(M.BASKET_XY[0] + 0.073, M.BASKET_XY[1], M.RIM), board_near_x=-0.14)),
        ("held inside the basket, 3 cm above its floor", scene(jar(*M.BASKET_XY, M.FLOOR_TOP + 0.03), -0.14)),
    ],
)
def test_a_retrieved_jar_not_in_the_basket_scores_0_4(where, objs):
    assert not TASK.success(objs), where
    assert TASK.score(objs) == 0.4, where


# -- 4. construct validity: shortcuts and failures score 0 -------------------------------------------------------------


@pytest.mark.parametrize(
    "where, objs",
    [
        ("untouched start", scene(jar(M.JAR_X0, 0.0, TOP + M.BOARD_H))),
        ("one stroke only", scene(jar(M.JAR_X0 - 0.30, 0.0, TOP + M.BOARD_H), board_near_x=M.BOARD_NEAR_X - 0.30)),
        ("board pulled, jar left behind on the table", scene(jar(0.80, 0.0, TOP), board_near_x=-0.14)),
        ("board pushed away", scene(jar(0.84, 0.0, TOP + M.BOARD_H), board_near_x=0.40)),
        ("jar off the table's far edge", scene(jar(0.95, 0.0, TOP - 0.30, size=JAR_LYING))),
        ("jar flung off the near edge (x < reach, on the floor)", scene(jar(-0.25, 0.0, 0.0), board_near_x=-0.14)),
        ("jar off the side of the table, still falling", scene(jar(0.30, 0.70, TOP - 0.05), board_near_x=-0.14)),
        ("jar on the floor under the basket", scene(jar(*M.BASKET_XY, 0.0), board_near_x=-0.14)),
    ],
)
def test_shortcuts_and_failures_score_zero(where, objs):
    assert not TASK.success(objs), where
    assert TASK.score(objs) == 0.0, where


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_jar_is_out_of_reach_straight_down_and_in_any_orientation(side):
    """The core rule: neither arm can grasp the jar where it starts, so the board has to come first."""
    target = (M.JAR_X0, 0.0, M.JAR_GRIP_Z)
    for yaw in (0.0, np.pi / 4, np.pi / 2, -np.pi / 2):
        pe, re = best_ik(target, down(yaw), side)
        assert pe > 50.0, (side, yaw, pe, re)  # straight down: more than 5 cm short (demo tolerance is 0.5 mm)
    pe, _ = best_ik(target, None, side)
    assert pe > 30.0, (side, pe)  # any gripper orientation: the TCP still ends more than 3 cm short of the centre
    # nor does it reach the jar's near face (centre minus radius) once the 13 mm of pad past the TCP is counted
    face = (M.JAR_X0 - M.JAR_R - 0.013, 0.0, M.JAR_GRIP_Z)
    pe, _ = best_ik(face, None, side)
    assert pe > 5.0, (side, pe)


@pytest.mark.parametrize("side", ["left", "right"])
def test_either_arm_reaches_the_boards_near_end_straight_down(side):
    pe, re = best_ik((M.GRIP_X, 0.0, M.BOARD_GRIP_Z), down(), side)
    assert pe < 0.5 and re < 0.1


# -- 5. layout: no overlaps, loose objects supported, grasped objects fit the gripper -----------------------------------


def test_objects_do_not_overlap_at_the_start():
    boxes = {o.name: bbox(o) for o in TASK.objects}
    names = list(boxes)
    for i, a in enumerate(names):
        for b in names[i + 1 :]:
            (alo, ahi), (blo, bhi) = boxes[a], boxes[b]
            overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
            assert not np.all(overlap > 1e-9), (a, b, overlap)


def test_loose_objects_rest_on_a_support_and_everything_is_on_the_table():
    blo, bhi = bbox(M.BOARD)
    jlo, jhi = bbox(M.JAR)
    assert blo[2] - TOP == pytest.approx(M.REST_GAP)
    assert jlo[2] - bhi[2] == pytest.approx(M.REST_GAP)
    assert np.all(jlo[:2] >= blo[:2] + 0.005) and np.all(jhi[:2] <= bhi[:2] - 0.005)  # the jar stands on the board
    for o in TASK.objects:
        lo, hi = bbox(o)
        assert M.TABLE_X[0] < lo[0] and hi[0] < M.TABLE_X[1] and M.TABLE_Y[0] < lo[1] and hi[1] < M.TABLE_Y[1], o.name
        assert lo[2] >= TOP - 1e-9, o.name


def test_grasped_objects_fit_the_open_gripper_with_a_centimetre_to_spare():
    assert GRIPPER_MAX == pytest.approx(0.0939, abs=1e-3)
    assert M.BOARD_W <= GRIPPER_MAX - MARGIN  # gripped across its width (fingers close along world y at yaw 0)
    assert 2 * M.JAR_R <= GRIPPER_MAX - MARGIN
    # open fingers (94 mm between the pads) fit inside the basket's 140 mm when setting the jar down
    assert GRIPPER_MAX + 2 * 0.012 < M.BASKET_INNER


def test_the_start_is_scored_zero_and_the_basket_lies_inside_the_reach_zone():
    start = scene(jar(M.JAR_X0, 0.0, TOP + M.BOARD_H))
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    bx, by = M.BASKET_XY
    assert bx + M.BASKET_INNER / 2 <= M.REACH_X and abs(by) + M.BASKET_INNER / 2 <= M.REACH_Y
    # the board's path (|y| <= 3.2 cm) stays clear of the basket
    assert by - M.OUTER / 2 > M.BOARD_W / 2 + 0.10


# -- task.json --------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_names_no_one():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    assert "media" not in meta
    assert meta["category"] in {"Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relevance", "Other"}
    text = json.dumps(meta).lower() + (HERE / "README.md").read_text().lower() + TASK.docs.lower()
    for word in ("robocurve_sprint", "knowledge base", "kairos"):
        assert word not in text, word
