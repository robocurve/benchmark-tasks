"""CPU checks: shapes, scoring outcomes and shortcuts, and the scripted plan. No GPU or Isaac Sim needed."""

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from geometry import spacer_points, tile_points, width_at  # noqa: E402
from yam_arena import analysis, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan, Waypoint  # noqa: E402
from yam_arena.station import BASES  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TASK = task_file.load(HERE / "task.py")
M = TASK.module
CLEAR = {"tcp": {"left": [0.20, 0.25, 0.90], "right": [0.20, -0.25, 0.90]}}
ASSIGN = {"spacer_left_inner": "left", "spacer_left_outer": "far", "spacer_right_inner": "right",
          "spacer_right_outer": "near"}  # fmt: skip


def quat(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]


def state(pos, q=(0, 0, 0, 1), moving=False):
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(q, float),
                       lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                       low=np.zeros(3), high=np.zeros(3))  # fmt: skip


def start():
    objs = {n: state([*xy, M.BOARD_TOP + M.TILE_SIZE[2] / 2]) for n, xy in M.TILE_START_XY.items()}
    objs.update({n: state([p[0], p[1], M.SLOT_FLOOR + M.SPACER_SIZE[2] / 2]) for n, p in M.SPACER_START.items()})
    return objs


def seated(joint, dz=0.0, yaw_extra=0.0, along=0.0):
    x, y = M.joint_spot(joint)
    axis = M.JOINTS[joint]["axis"]
    yaw = (0.0 if axis == 0 else np.pi / 2) + yaw_extra
    p = np.array([x, y, M.BOARD_TOP + M.SPACER_SIZE[2] / 2 + dz])
    p[axis] += along
    return state(p, quat([0, 0, 1], yaw))


def finished():
    objs = start()
    objs.update({s: seated(j) for s, j in ASSIGN.items()})
    return objs


# -- shapes -------------------------------------------------------------------------------------------------


def test_tile_hull_has_eased_top_edges_and_true_size():
    pts = tile_points(M.TILE_SIZE, M.TILE_CHAMFER)
    assert np.allclose(pts.max(0) - pts.min(0), M.TILE_SIZE)
    top, bottom = M.TILE_SIZE[2] / 2, -M.TILE_SIZE[2] / 2
    assert width_at(pts, 0, top) == pytest.approx(M.TILE_SIZE[0] - 2 * M.TILE_CHAMFER)
    assert width_at(pts, 0, bottom) == pytest.approx(M.TILE_SIZE[0])
    # Two eased edges open the 3 mm joint to 4.6 mm at the glaze.
    assert M.JOINT + 2 * M.TILE_CHAMFER == pytest.approx(0.0046)


def test_spacer_hull_narrows_to_its_edges():
    pts = spacer_points(M.SPACER_SIZE, M.SPACER_CHAMFER)
    assert np.allclose(pts.max(0) - pts.min(0), M.SPACER_SIZE)
    assert width_at(pts, 1, -M.SPACER_SIZE[2] / 2) == pytest.approx(M.SPACER_SIZE[1] - 2 * M.SPACER_CHAMFER)
    assert width_at(pts, 1, 0.0) == pytest.approx(M.SPACER_SIZE[1])


def test_spacer_fits_its_joint_and_slot():
    assert M.SPACER_SIZE[1] == pytest.approx(M.JOINT)
    assert M.SLOT_WIDTH > M.SPACER_SIZE[1]
    assert M.SPACER_SIZE[2] > M.TILE_SIZE[2] + 0.010  # stands proud of the glaze for the fingers


# -- layout ------------------------------------------------------------------------------------------------


def test_starting_layout():
    names = [o.name for o in TASK.objects]
    assert len(names) == len(set(names))
    assert set(M.TILES) | set(M.SPACERS) == set(TASK.free_objects)
    xs = sorted({round(float(xy[0]), 4) for xy in M.TILE_START_XY.values()})
    assert xs[1] - xs[0] - M.TILE_SIZE[0] == pytest.approx(M.JOINT)
    for spec in M.JOINTS.values():
        assert spec["span"][1] - spec["span"][0] == pytest.approx(M.TILE_SIZE[0])


def test_start_scores_zero():
    r = M.measure(start(), CLEAR)
    assert r["partial_score"] == 0.0 and not r["success_now"] and r["layout_ok"]


# -- outcomes ----------------------------------------------------------------------------------------------


def test_full_score_needs_all_four_layout_release_and_rest():
    r = M.measure(finished(), CLEAR)
    assert r["success_now"] and r["joints_filled"] == ["far", "left", "near", "right"]
    assert r["partial_score"] == pytest.approx(0.8)


@pytest.mark.parametrize("n", [0, 1, 2, 3])
def test_partial_credit_per_seated_spacer(n):
    objs = start()
    for s in list(ASSIGN)[:n]:
        objs[s] = seated(ASSIGN[s])
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == pytest.approx(0.2 * n) and not r["success_now"]


def test_gripper_still_on_a_spacer_is_not_success():
    x, y = M.joint_spot("right")
    tcp = {"tcp": {"left": CLEAR["tcp"]["left"], "right": [x, y, M.SEAT_Z]}}
    r = M.measure(finished(), tcp)
    assert not r["success_now"] and r["partial_score"] == pytest.approx(0.8)


def test_no_gripper_positions_is_not_success():
    assert not M.measure(finished(), {})["success_now"]


def test_moving_objects_are_not_success():
    objs = finished()
    objs["tile_near_left"] = state(objs["tile_near_left"].pos, moving=True)
    assert not M.measure(objs, CLEAR)["success_now"]


@pytest.mark.parametrize("shift", [0.0012, 0.003])
def test_moved_tile_scores_zero(shift):
    objs = finished()
    p = objs["tile_far_right"].pos.copy()
    p[1] -= shift  # pushed away from the far joint, as a forced-in spacer would
    objs["tile_far_right"] = state(p)
    r = M.measure(objs, CLEAR)
    assert not r["layout_ok"] and r["partial_score"] == 0.0 and not r["success_now"]


def test_small_tile_shift_is_tolerated():
    objs = finished()
    p = objs["tile_far_right"].pos.copy()
    p[1] -= 0.0006
    objs["tile_far_right"] = state(p)
    assert M.measure(objs, CLEAR)["success_now"]


def test_turned_tile_scores_zero():
    objs = finished()
    objs["tile_near_right"] = state(objs["tile_near_right"].pos, quat([0, 0, 1], np.radians(1.5)))
    assert M.measure(objs, CLEAR)["partial_score"] == 0.0


def test_tile_riding_on_a_spacer_scores_zero():
    objs = finished()
    p = objs["tile_far_left"].pos.copy()
    p[2] += 0.0015
    objs["tile_far_left"] = state(p, quat([1, 0, 0], np.radians(1.0)))
    assert M.measure(objs, CLEAR)["partial_score"] == 0.0


# -- what does not count as a seated spacer ---------------------------------------------------------------


def test_spacer_only_started_into_the_joint():
    objs = finished()
    objs["spacer_left_outer"] = seated("far", dz=0.003)  # resting on the eased edges, not on the board
    r = M.measure(objs, CLEAR)
    assert "far" not in r["joints_filled"] and r["partial_score"] == pytest.approx(0.6)


def test_spacer_lying_flat_over_a_joint():
    objs = finished()
    x, y = M.joint_spot("near")
    objs["spacer_right_outer"] = state([x, y, M.BOARD_TOP + M.TILE_SIZE[2] + 0.0015], quat([1, 0, 0], np.pi / 2))
    assert "near" not in M.measure(objs, CLEAR)["joints_filled"]


def test_spacer_at_the_crossing_counts_for_no_joint():
    objs = finished()
    objs["spacer_left_inner"] = state([*M.PATCH_XY, M.BOARD_TOP + M.SPACER_SIZE[2] / 2])
    assert "left" not in M.measure(objs, CLEAR)["joints_filled"]


def test_spacer_hanging_out_of_the_patch_edge():
    objs = finished()
    objs["spacer_right_inner"] = seated("right", along=-0.040)  # 2.5 mm past the outer tile edge
    assert "right" not in M.measure(objs, CLEAR)["joints_filled"]


def test_spacer_turned_across_the_joint():
    objs = finished()
    objs["spacer_left_outer"] = seated("far", yaw_extra=np.radians(30))
    assert "far" not in M.measure(objs, CLEAR)["joints_filled"]


def test_spacer_on_its_end_still_counts():
    objs = finished()
    x, y = M.joint_spot("near")
    q = quat([0, 1, 0], np.pi / 2)  # its 25 mm length vertical, thickness still across the joint
    objs["spacer_right_outer"] = state([x, y, M.BOARD_TOP + M.SPACER_SIZE[0] / 2], q)
    assert "near" in M.measure(objs, CLEAR)["joints_filled"]


def test_far_and_near_joints_are_told_apart():
    objs = start()
    objs["spacer_right_outer"] = seated("near")
    objs["spacer_left_outer"] = seated("near", along=0.030)
    r = M.measure(objs, CLEAR)
    assert r["joints_filled"] == ["near"] and r["partial_score"] == pytest.approx(0.2)


def test_spacer_left_in_its_slot_or_dropped_on_the_board():
    objs = start()
    objs["spacer_left_inner"] = state([0.14, 0.12, M.BOARD_TOP + M.SPACER_SIZE[2] / 2])  # upright, off the patch
    assert M.measure(objs, CLEAR)["joints_filled"] == []


# -- the scripted plan ----------------------------------------------------------------------------------


def test_demo_plan_is_reachable_and_inside_the_limit():
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    assert end < 110.0
    for k in range(int(end * 20) + 1):
        for plan in plans.values():
            plan.action7(k / 20)
    for plan in plans.values():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5


def _arm_hulls(plan, arm, t):
    q7, _ = plan.action7(t)
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(q7[:6])}
    q["joint7"] = q["joint8"] = float(q7[6] * FINGER_OPEN)
    return analysis.world_hulls(analysis.fk_link_poses_world(q, BASES[arm]))


def _arm_gap(left, right, margin=0.05):
    """hull_set_distance, run only on link pairs whose bounding boxes are within ``margin``: inf if none are."""

    def boxes(hulls):
        out = {}
        for name, hull in hulls:
            lo, hi = hull.vertices.min(0), hull.vertices.max(0)
            box = out.get(name)
            out[name] = (lo, hi) if box is None else (np.minimum(box[0], lo), np.maximum(box[1], hi))
        return out

    bl, br = boxes(left), boxes(right)
    near = [(a, b) for a in bl for b in br if np.maximum(br[b][0] - bl[a][1], bl[a][0] - br[b][1]).max() <= margin]
    if not near:
        return np.inf, None
    links_l, links_r = {a for a, _ in near}, {b for _, b in near}
    return analysis.hull_set_distance([h for h in left if h[0] in links_l], [h for h in right if h[0] in links_r])


def test_demo_arms_never_meet():
    """The arms' collision hulls stay at least 2 cm apart all through the plan. (With both grippers over the
    side joints at once, the two wrists overlap: that is why the plan takes the joints one at a time.)"""
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    for k in range(int(end * 10) + 1):
        gap, pair = _arm_gap(*(_arm_hulls(plans[a], a, k / 10) for a in ("left", "right")))
        assert gap > 0.02, (k / 10, gap, pair)


def test_hull_check_sees_both_arms_over_the_side_joints():
    hulls = []
    for arm, y, yaw in (("left", 0.0515, -np.pi / 2), ("right", -0.0515, np.pi / 2)):
        plan = ArmPlan([Waypoint(1.0, (0.27, y, 0.80), M.FINGER_APPROACH, "", yaw)], BASES[arm])
        for k in range(21):
            plan.action7(k / 20)
        hulls.append(_arm_hulls(plan, arm, 1.0))
    assert _arm_gap(*hulls)[0] < 0.0


def test_demo_seats_each_spacer_on_the_board():
    assert M.SEAT_Z - M.GRIP_DROP == pytest.approx(M.BOARD_TOP - M.PRESS)
    assert M.SEAT_Z - 0.013 > M.BOARD_TOP + M.TILE_SIZE[2] + 0.003  # pad tips stay 3 mm over the glaze
    assert M.GRASP_Z - 0.013 > M.BLOCK_TOP  # and over the block while gripping
