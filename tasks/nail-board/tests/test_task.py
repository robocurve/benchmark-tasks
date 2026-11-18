"""CPU checks: the layout, every scoring outcome and shortcut, and the scripted plan. No GPU or Isaac Sim."""

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

from yam_arena import analysis, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

TASK = task_file.load(HERE / "task.py")
M = TASK.module
CLEAR = {"tcp": {"left": [0.20, 0.26, 0.92], "right": [0.20, -0.26, 0.92]}}
TOTE_CENTRE = (M.TOTE_XY[0], M.TOTE_XY[1], M.TOTE_FLOOR + M.SECTION[1] / 2)
ALONG_Y = np.pi / 2  # a board in the tote lies along y


def quat(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]


def compose(*quats):
    r = np.eye(3)
    for q in quats:
        r = quat_xyzw_to_matrix(np.asarray(q)) @ r
    from yam_arena.ik import matrix_to_quat_xyzw

    return matrix_to_quat_xyzw(r)


def board(name, centre, yaw=0.0, roll=0.0, pitch=0.0, moving=False):
    """An ObjectState whose reported position is what the embodiment reports: the nailed board's includes its nail."""
    q = compose(quat([1, 0, 0], roll), quat([0, 1, 0], pitch), quat([0, 0, 1], yaw))
    rot = quat_xyzw_to_matrix(q)
    pos = np.asarray(centre, dtype=float) + (rot[:, 2] * M.NAILED_POS_DZ if name == M.NAILED else 0.0)
    return ObjectState(pos=pos, quat_xyzw=q, lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                       low=np.zeros(3), high=np.zeros(3))  # fmt: skip


def on_deck(name, y, x=None, layer=0, **kw):
    return board(name, (M.STACK_X if x is None else x, y, M.DECK_TOP + (layer + 0.5) * M.SECTION[1]), **kw)


def in_slot(name, slot, **kw):
    y, layer = M.STACK_SLOTS[slot]
    return on_deck(name, y, layer=layer, **kw)


def start():
    return {n: board(n, (x, y, M.TOP + M.SECTION[1] / 2), yaw) for n, (_, x, y, yaw) in M.BOARDS.items()}


def finished():
    objs = start()
    for slot, name in enumerate(M.CLEAN):
        objs[name] = in_slot(name, slot)
    objs[M.NAILED] = board(M.NAILED, TOTE_CENTRE, ALONG_Y)
    return objs


# -- the layout -----------------------------------------------------------------------------------------------


def footprint(name, state):
    return M._corners(name, state)[[0, 2, 6, 4], :2]  # the bottom corners, in polygon order


def separated(a, b):
    """Separating-axis test of two convex plan footprints."""
    for poly in (a, b):
        for i in range(len(poly)):
            edge = poly[(i + 1) % len(poly)] - poly[i]
            axis = np.array([-edge[1], edge[0]])
            pa, pb = a @ axis, b @ axis
            if pa.max() < pb.min() or pb.max() < pa.min():
                return True
    return False


def test_boards_start_apart_and_off_the_pallet_and_tote():
    objs = start()
    names = list(M.BOARDS)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            assert separated(footprint(a, objs[a]), footprint(b, objs[b])), (a, b)
    for name in names:
        r = M.board_report(name, objs[name])
        assert not r["touching_deck"] and not r["over_deck"] and not r["in_tote"], name
        assert all(abs(c[1]) <= 0.40 and 0.05 <= c[0] <= 0.55 for c in r["corners"]), name  # on the table, off the bases
    # the nail stands inside the board's footprint
    assert abs(M.NAIL_X) + M.NAIL_HEAD_DIAMETER / 2 < M.BOARDS[M.NAILED][0] / 2
    assert abs(M.NAIL_Y) + M.NAIL_HEAD_DIAMETER / 2 < M.SECTION[0] / 2


def test_objects_are_fixed_or_loose_as_described():
    free = {o.name for o in TASK.objects if o.free}
    assert free == set(M.BOARDS)
    assert {o.name for o in TASK.objects if not o.free} >= {"pallet_slat0", "tote_floor", "tote_label"}


def test_nailed_centre_is_recovered_from_the_reported_position():
    centre = np.array([0.3, -0.1, M.TOP + 0.019])
    s = board(M.NAILED, centre, yaw=0.4, roll=0.3)
    assert np.allclose(M._frame(M.NAILED, s)[0], centre, atol=1e-9)
    assert not np.allclose(s.pos, centre)


# -- scoring --------------------------------------------------------------------------------------------------


def test_start_scores_zero():
    m = M.measure(start(), CLEAR)
    assert m["partial_score"] == 0.0 and m["stacked"] == [] and not m["nailed_in_tote"] and not m["success_now"]


def test_full_score():
    m = M.measure(finished(), CLEAR)
    assert sorted(m["stacked"]) == sorted(M.CLEAN) and m["nailed_in_tote"] and not m["nailed_on_pallet"]
    assert m["hands_clear"] and m["still"] and m["success_now"]
    assert m["partial_score"] == pytest.approx(0.8)
    assert M.success(finished()) is False  # no gripper positions given: the final release is not shown
    M._robot.update(CLEAR)
    try:
        assert M.success(finished()) is True
    finally:
        M._robot.clear()


@pytest.mark.parametrize("n", [0, 1, 3, 5])
def test_partial_credit_per_stacked_board(n):
    objs = start()
    for slot, name in list(enumerate(M.CLEAN))[:n]:
        objs[name] = in_slot(name, slot)
    assert M.score(objs) == pytest.approx(0.1 * n)
    objs[M.NAILED] = board(M.NAILED, TOTE_CENTRE, ALONG_Y)
    assert M.score(objs) == pytest.approx(0.1 * n + 0.3)


@pytest.mark.parametrize("how", ["flat", "nail_down", "leaning_on_the_edge", "on_top_of_the_stack"])
def test_nailed_board_on_the_pallet_scores_zero_however_neat(how):
    objs = finished()
    if how == "flat":
        objs[M.NAILED] = on_deck(M.NAILED, 0.0, x=M.PALLET_XY[0] + 0.03)
    elif how == "nail_down":
        objs[M.NAILED] = on_deck(M.NAILED, 0.0, roll=np.pi)
    elif how == "leaning_on_the_edge":  # one end up on the deck's edge, the other on the table
        length = M.BOARDS[M.NAILED][0]
        pitch = np.arcsin((M.DECK_TOP - M.TOP) / length)  # +x end down on the table, -x end up on the deck
        objs[M.NAILED] = board(M.NAILED, (M.DECK_X[1] + 0.02, 0.0, (M.DECK_TOP + M.TOP) / 2 + M.SECTION[1] / 2), pitch=pitch)
    else:
        objs[M.NAILED] = on_deck(M.NAILED, 0.0, layer=2)
    m = M.measure(objs, CLEAR)
    assert m["nailed_on_pallet"] and m["partial_score"] == 0.0 and not m["success_now"], how


def test_nailed_board_held_in_the_air_over_the_pallet_is_not_on_it():
    objs = finished()
    objs[M.NAILED] = board(M.NAILED, (M.STACK_X, 0.0, M.DECK_TOP + 0.15))
    m = M.measure(objs, CLEAR)
    assert not m["nailed_on_pallet"] and not m["nailed_in_tote"] and m["partial_score"] == pytest.approx(0.5)


def test_nailed_board_left_on_the_table_keeps_the_stacking_credit_only():
    objs = finished()
    objs[M.NAILED] = start()[M.NAILED]
    m = M.measure(objs, CLEAR)
    assert m["partial_score"] == pytest.approx(0.5) and not m["success_now"]


def test_nailed_board_sticking_out_of_the_tote_does_not_count():
    objs = finished()
    length = M.BOARDS[M.NAILED][0]
    # standing on end in the tote: its top is above the rim
    objs[M.NAILED] = board(M.NAILED, (M.TOTE_XY[0], M.TOTE_XY[1], M.TOTE_FLOOR + length / 2), pitch=np.pi / 2)
    assert not M.measure(objs, CLEAR)["nailed_in_tote"]
    # lying across the rim (along x, the tote's short way)
    objs[M.NAILED] = board(M.NAILED, (M.TOTE_XY[0], M.TOTE_XY[1], M.TOTE_RIM + M.SECTION[1] / 2))
    assert not M.measure(objs, CLEAR)["nailed_in_tote"]
    # on its side on the floor, leaning on a wall: counts
    objs[M.NAILED] = board(M.NAILED, (M.TOTE_XY[0] + 0.02, M.TOTE_XY[1], M.TOTE_FLOOR + M.SECTION[0] / 2), yaw=ALONG_Y, roll=np.radians(80))
    assert M.measure(objs, CLEAR)["nailed_in_tote"]
    # flat on the floor, turned a little: counts
    objs[M.NAILED] = board(M.NAILED, TOTE_CENTRE, yaw=ALONG_Y + 0.15)
    assert M.measure(objs, CLEAR)["nailed_in_tote"]


def test_clean_board_in_the_tote_does_not_count_as_stacked():
    objs = finished()
    objs["offcut_a"] = board("offcut_a", TOTE_CENTRE, ALONG_Y)
    objs[M.NAILED] = start()[M.NAILED]
    assert M.score(objs) == pytest.approx(0.4)


def test_board_on_top_of_another_counts_as_stacked():
    objs = finished()
    objs["offcut_b"] = on_deck("offcut_b", 0.0, layer=2)  # a third layer, on the two bridging boards
    assert "offcut_b" in M.measure(objs, CLEAR)["stacked"]
    objs["offcut_b"] = on_deck("offcut_b", 0.0, layer=3)  # one layer too high: in the air
    assert "offcut_b" not in M.measure(objs, CLEAR)["stacked"]


def test_tilted_hovering_or_overhanging_boards_do_not_count():
    objs = finished()
    objs["offcut_b"] = in_slot("offcut_b", 1, roll=np.radians(15))
    assert "offcut_b" not in M.measure(objs, CLEAR)["stacked"], "tilted 15 degrees"
    objs["offcut_b"] = board("offcut_b", (M.STACK_X, 0.0, M.DECK_TOP + 0.10 + M.SECTION[1] / 2))
    assert "offcut_b" not in M.measure(objs, CLEAR)["stacked"], "held 10 cm above the deck"
    objs["offcut_b"] = on_deck("offcut_b", M.DECK_Y[1] + 0.01)
    assert "offcut_b" not in M.measure(objs, CLEAR)["stacked"], "centre past the deck's edge"
    objs["offcut_b"] = in_slot("offcut_b", 1, roll=np.radians(5))
    assert "offcut_b" in M.measure(objs, CLEAR)["stacked"], "5 degrees is flat enough"


def test_gripper_still_on_a_board_is_not_success():
    objs = finished()
    held = {"tcp": {"left": [M.STACK_X, M.STACK_SLOTS[0][0], M.DECK_TOP + M.SECTION[1] + 0.01], "right": CLEAR["tcp"]["right"]}}
    m = M.measure(objs, held)
    assert not m["hands_clear"] and not m["success_now"] and m["partial_score"] == pytest.approx(0.8)


def test_moving_boards_are_not_success():
    objs = finished()
    objs["offcut_a"] = in_slot("offcut_a", 0, moving=True)
    assert not M.measure(objs, CLEAR)["success_now"]


def test_distance_falls_to_zero_when_finished():
    assert M.distance(start()) > 0.5
    assert M.distance(finished()) == 0.0


# -- the scripted plan -----------------------------------------------------------------------------------------


def test_demo_plan_is_reachable_and_inside_the_limit():
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    assert end < 150.0
    for k in range(int(end * 20) + 1):
        for plan in plans.values():
            plan.action7(k / 20)
    for plan in plans.values():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5


def test_demo_puts_every_board_where_the_scorer_wants_it():
    """The plan's release points, read as final board poses, make the scorer's full score."""
    objs = start()
    for arm, jobs in M.JOBS.items():
        for name, where in jobs:
            if where == "tote":
                objs[name] = board(name, TOTE_CENTRE, ALONG_Y)
            else:
                objs[name] = in_slot(name, where)
    assert {n for jobs in M.JOBS.values() for n, _ in jobs} == set(M.BOARDS)
    assert sorted(w for jobs in M.JOBS.values() for _, w in jobs if w != "tote") == list(range(len(M.STACK_SLOTS)))
    assert M.measure(objs, CLEAR)["success_now"]
    # boards in the same layer do not overlap, and each top board rests on two lower ones
    by_layer = {}
    for n, where in ((n, w) for jobs in M.JOBS.values() for n, w in jobs if w != "tote"):
        by_layer.setdefault(M.STACK_SLOTS[where][1], []).append(n)
    for names in by_layer.values():
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                assert separated(footprint(names[i], objs[names[i]]), footprint(names[j], objs[names[j]]))
    for n in by_layer[1]:
        under = [m for m in by_layer[0] if not separated(footprint(n, objs[n]), footprint(m, objs[m]))]
        assert len(under) == 2, n
    # the top layer is laid after the boards under it, in the plan's alternating order
    turn = {}
    order = ["left", "right"] * 3
    counters = {"left": 0, "right": 0}
    for k, arm in enumerate(order):
        name, where = M.JOBS[arm][counters[arm]]
        counters[arm] += 1
        turn[where] = k
    for slot, (y, layer) in enumerate(M.STACK_SLOTS):
        if layer == 1:
            for lower, (ly, ll) in enumerate(M.STACK_SLOTS):
                if ll == 0 and abs(ly - y) < M.SECTION[0]:
                    assert turn[lower] < turn[slot], (slot, lower)


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
    """The arms' collision hulls stay at least 2 cm apart all through the plan (they take turns over the
    pallet and the tote)."""
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    for k in range(int(end * 10) + 1):
        gap, pair = _arm_gap(*(_arm_hulls(plans[a], a, k / 10) for a in ("left", "right")))
        assert gap > 0.02, (k / 10, gap, pair)
