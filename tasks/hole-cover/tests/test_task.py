"""CPU checks: the cover's shape, every scoring outcome and shortcut, and the scripted plan. No GPU needed."""

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import geometry as G  # noqa: E402
from yam_arena import analysis, task_file  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TASK = task_file.load(HERE / "task.py")
M = TASK.module
CLEAR = {"tcp": {"left": [0.15, 0.30, 0.95], "right": [0.15, -0.30, 0.95]}}
TOTE_SPOT = [*M.TOTE_XY, M.TOTE_FLOOR_TOP + M.WRENCH_SIZE[2] / 2]
DECK_SPOT = [0.42, 0.30, M.DECK_TOP + M.WRENCH_SIZE[2] / 2]  # the wrench just laid on the deck


def quat(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]


def state(pos, q=(0, 0, 0, 1), moving=False):
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(q, float),
                       lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                       low=np.zeros(3), high=np.zeros(3))  # fmt: skip


def seated_cover(dx=0.0, dy=0.0, dz=0.0, q=(0, 0, 0, 1)):
    return state([M.OPENING_XY[0] + dx, M.OPENING_XY[1] + dy, M.COVER_SEATED_Z + dz], q)


def parked_cover():
    return state([*M.PARK_COVER_XY, M.COVER_SEATED_Z + G.CLEAT_SIZE[2]])


def start():
    return {"cover": seated_cover(), "wrench": state([*M.OPENING_XY, M.TOP + M.WRENCH_SIZE[2] / 2])}


def finished():
    return {"cover": seated_cover(), "wrench": state(TOTE_SPOT)}


# -- shapes -------------------------------------------------------------------------------------------------


def test_cover_frame_is_its_bounding_box_centre():
    lo, hi = G.cover_bounds_local()
    assert np.allclose((lo + hi) / 2, 0.0, atol=1e-9)
    assert np.allclose(hi - lo, M.COVER_SIZE)


def test_cover_mass_matches_its_wood():
    plywood, softwood = 600.0, 500.0  # kg/m3
    plate_and_cleats = (np.prod(G.PLATE_SIZE) + 2 * np.prod(G.CLEAT_SIZE)) * plywood
    handle = np.prod(G.HANDLE_SIZE) * softwood
    assert M.COVER_MASS == pytest.approx(plate_and_cleats + handle, abs=0.1)


def test_cleats_locate_in_the_opening_with_clearance():
    outer = G.CLEAT_Y + G.CLEAT_SIZE[1] / 2
    assert 0.004 <= M.OPENING_HALF - outer <= 0.008  # 6 mm a side
    assert 0.003 <= M.OPENING_HALF - G.CLEAT_SIZE[0] / 2 <= 0.008  # 5 mm an end
    assert M.OPENING_HALF - outer < M.OUTLINE_TOLERANCE  # a seated cover is always on its outline
    # the chamfered feet: a cover set down up to play + chamfer off centre still finds the opening
    feet = G.chamfered_bottom_points(G.CLEAT_SIZE, G.CLEAT_CHAMFER)
    assert feet[:, 2].min() == pytest.approx(-G.CLEAT_SIZE[2] / 2)
    foot = feet[np.isclose(feet[:, 2], -G.CLEAT_SIZE[2] / 2)]
    assert foot[:, 1].max() - foot[:, 1].min() == pytest.approx(G.CLEAT_SIZE[1] - 2 * G.CLEAT_CHAMFER)
    assert (M.OPENING_HALF - outer) + G.CLEAT_CHAMFER >= 0.012  # 14 mm of lateral error is taken up
    parts = dict(G.cover_part_points())
    assert set(parts) == {"plate", "handle", "cleat_left", "cleat_right"}
    assert len(parts["cleat_left"]) == 12 and len(parts["plate"]) == 8


def test_handle_grip_tolerates_a_shifted_cover():
    assert 2 * abs(M.FINGER_HANDLE) - G.HANDLE_SIZE[1] >= 0.040  # 20 mm to spare each side
    assert G.PLATE_SIZE[0] / 2 - M.OPENING_HALF >= 0.03  # the plate overlaps the deck 40 mm all round
    assert M.COVER_SEATED_Z + G.CLEAT_BOTTOM_Z > M.TOP + 0.05  # the cleats hang well short of the subfloor


def test_opening_depth_and_cover_do_not_leave_the_wrench_reachable():
    assert M.DECK_HEIGHT == pytest.approx(0.080)
    assert M.WRENCH_START[2] < M.DECK_TOP - 0.06
    assert G.PLATE_SIZE[0] > 2 * M.OPENING_HALF + 0.06


def test_starting_layout():
    names = [o.name for o in TASK.objects]
    assert len(names) == len(set(names))
    assert set(M.FREE) == set(TASK.free_objects)
    assert M.DECK_X[0] <= M.OPENING_XY[0] - M.OPENING_HALF and M.OPENING_XY[0] + M.OPENING_HALF <= M.DECK_X[1]
    assert M.DECK_Y[0] <= -M.OPENING_HALF and M.OPENING_HALF <= M.DECK_Y[1]
    # the parked cover lies wholly on the deck and clear of the marked outline
    px, py = M.PARK_COVER_XY
    half = G.PLATE_SIZE[0] / 2
    assert M.DECK_X[0] <= px - half and px + half <= M.DECK_X[1]
    assert py - half >= M.OPENING_XY[1] + half + 0.005 and py + half <= M.DECK_Y[1]
    # the tote stands clear of the deck
    assert M.TOTE_XY[1] + M.TOTE_INNER[1] / 2 + M.TOTE_WALL < M.DECK_Y[0] - 0.02


# -- outcomes ----------------------------------------------------------------------------------------------


def test_start_scores_zero():
    r = M.measure(start(), CLEAR)
    assert r["partial_score"] == 0.0 and not r["success_now"]
    assert r["cover"]["seated"] and r["wrench"]["in_opening"] and not r["wrench"]["in_tote"]


def test_full_score():
    r = M.measure(finished(), CLEAR)
    assert r["success_now"] and r["partial_score"] == 1.0


def test_hole_covered_wrench_left_on_the_deck_scores_half():
    objs = {"cover": seated_cover(), "wrench": state(DECK_SPOT)}
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == pytest.approx(0.5) and not r["success_now"]


def test_wrench_put_away_hole_left_open_scores_a_quarter():
    objs = {"cover": parked_cover(), "wrench": state(TOTE_SPOT)}
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == pytest.approx(0.25) and not r["success_now"]


def test_wrench_out_and_hole_open_scores_zero():
    objs = {"cover": parked_cover(), "wrench": state(DECK_SPOT)}
    assert M.measure(objs, CLEAR)["partial_score"] == 0.0


def test_cover_lifted_and_put_back_without_the_wrench_scores_zero():
    assert M.measure(start(), CLEAR)["partial_score"] == 0.0


def test_gripper_still_on_the_handle_is_not_success():
    tcp = {"tcp": {"left": [*M.OPENING_XY, M.COVER_GRIP_Z], "right": CLEAR["tcp"]["right"]}}
    r = M.measure(finished(), tcp)
    assert not r["success_now"] and r["partial_score"] == 1.0


def test_no_gripper_positions_is_not_success():
    assert not M.measure(finished(), {})["success_now"]


def test_moving_objects_are_not_success():
    objs = finished()
    objs["wrench"] = state(TOTE_SPOT, moving=True)
    assert not M.measure(objs, CLEAR)["success_now"]


# -- what does not count as a seated cover ---------------------------------------------------------------


def test_cover_resting_on_a_cleat_beside_the_opening():
    objs = finished()
    objs["cover"] = seated_cover(dy=0.030, dz=G.CLEAT_SIZE[2])  # one cleat on the deck, the plate 18 mm up
    r = M.measure(objs, CLEAR)
    assert not r["cover"]["seated"] and r["partial_score"] == pytest.approx(0.25)


def test_cover_shifted_past_the_outline_tolerance():
    objs = finished()
    objs["cover"] = seated_cover(dx=0.018)
    assert not M.measure(objs, CLEAR)["cover"]["seated"]


def test_cover_within_the_outline_tolerance_counts():
    objs = finished()
    objs["cover"] = seated_cover(dx=0.005, dy=-0.006, q=quat([0, 0, 1], np.radians(1.5)))
    assert M.measure(objs, CLEAR)["success_now"]


def test_cover_turned_a_quarter_turn_counts():
    objs = finished()
    objs["cover"] = seated_cover(q=quat([0, 0, 1], np.pi / 2))
    assert M.measure(objs, CLEAR)["success_now"]


def test_cover_turned_a_little_fails_the_outline():
    objs = finished()
    objs["cover"] = seated_cover(q=quat([0, 0, 1], np.radians(8)))  # corners move about 20 mm
    assert not M.measure(objs, CLEAR)["cover"]["seated"]


def test_cover_tilted_on_something_fails_flatness():
    objs = finished()
    tilt = np.arctan2(0.012, G.PLATE_SIZE[0])  # one edge 12 mm up
    objs["cover"] = seated_cover(dz=0.006, q=quat([1, 0, 0], tilt))
    assert not M.measure(objs, CLEAR)["cover"]["seated"]


def test_cover_floating_over_the_opening_on_its_cleats_fails():
    objs = finished()
    objs["cover"] = seated_cover(dz=G.CLEAT_SIZE[2])  # as if the cleats sat on the deck's edges
    assert not M.measure(objs, CLEAR)["cover"]["seated"]


def test_wrench_on_the_tote_rim_or_beside_it_is_not_in():
    objs = finished()
    objs["wrench"] = state([M.TOTE_XY[0], M.TOTE_XY[1] + M.TOTE_INNER[1] / 2, M.TOTE_RIM + M.WRENCH_SIZE[2] / 2])
    assert not M.measure(objs, CLEAR)["wrench"]["in_tote"]
    objs["wrench"] = state([M.TOTE_XY[0], M.TOTE_XY[1] - 0.15, M.TOP + M.WRENCH_SIZE[2] / 2])
    assert not M.measure(objs, CLEAR)["wrench"]["in_tote"]


def test_wrench_standing_in_the_tote_counts():
    objs = finished()
    objs["wrench"] = state([*M.TOTE_XY, M.TOTE_FLOOR_TOP + M.WRENCH_SIZE[0] / 2], quat([0, 1, 0], np.pi / 2))
    assert M.measure(objs, CLEAR)["success_now"]


# -- the scripted plan ----------------------------------------------------------------------------------


def test_demo_plan_is_reachable_and_inside_the_limit():
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    assert end < 60.0  # the limit is 150 s
    for k in range(int(end * 20) + 1):
        for plan in plans.values():
            plan.action7(k / 20)
    for arm, plan in plans.items():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_demo_carries_the_cover_clear_of_the_deck():
    assert M.COVER_TRAVEL_Z - M.GRIP_TO_CLEAT_FEET >= M.DECK_TOP + 0.025
    assert M.COVER_GRIP_Z - 0.013 > M.DECK_TOP + G.PLATE_SIZE[2] + 0.005  # pad tips over the plate when gripping
    assert M.WRENCH_TRAVEL_Z - 0.0135 > M.DECK_TOP + 0.04  # the carried wrench over the deck
    assert M.TOTE_DROP_Z - 0.0135 > M.TOTE_RIM


def _arm_hulls(plan, arm, t, links=None):
    q7, _ = plan.action7(t)
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(q7[:6])}
    q["joint7"] = q["joint8"] = float(q7[6] * FINGER_OPEN)
    return analysis.world_hulls(analysis.fk_link_poses_world(q, BASES[arm]), links)


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
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    for k in range(int(end * 10) + 1):
        gap, pair = _arm_gap(*(_arm_hulls(plans[a], a, k / 10) for a in ("left", "right")))
        assert gap > 0.02, (k / 10, gap, pair)


FIXED_BOXES = [(o.name, np.asarray(o.pos), np.asarray(o.size)) for o in TASK.objects if not o.free]


def _nearest_fixed(hulls, exclude=()):
    best, where = np.inf, None
    for name, pos, size in FIXED_BOXES:
        if name in exclude:
            continue
        d, link = analysis.hulls_to_box(hulls, pos, np.eye(3), size)
        if d < best:
            best, where = d, (name, link)
    return best, where


def test_demo_arms_keep_off_the_deck_and_tote():
    """Upper links stay 1 cm off every fixed box; the gripper and fingers 3 mm (they go down into the opening)."""
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    upper = ["base", "link1", "link2", "link3", "link4", "link5"]
    hand = ["gripper", "tip_left", "tip_right"]
    for k in range(int(end * 10) + 1):
        for arm, plan in plans.items():
            d, where = _nearest_fixed(_arm_hulls(plan, arm, k / 10, upper))
            assert d > 0.01, (arm, k / 10, d, where)
            d, where = _nearest_fixed(_arm_hulls(plan, arm, k / 10, hand))
            assert d > 0.003, (arm, k / 10, d, where)


def test_gripper_fits_down_the_opening_at_the_wrench():
    plan = ArmPlan(M.demo()["right"], BASES["right"])
    t = next(t for t, w in _times(plan) if w.label == "grip the wrench")
    hulls = _arm_hulls(plan, "right", t, ["gripper", "tip_left", "tip_right"])
    low = np.concatenate([h.vertices[h.vertices[:, 2] < M.DECK_TOP] for _, h in hulls])
    assert len(low) > 0 and low[:, 2].min() < M.TOP + 0.02  # the pads reach the subfloor
    assert np.all(np.abs(low[:, :2] - M.OPENING_XY) < M.OPENING_HALF - 0.004)  # inside the opening with 4 mm to spare


def test_demo_arms_keep_off_the_parked_cover():
    """While the cover stands beside the opening, no upper link comes within 1 cm of it."""
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    left = plans["left"]
    parked_from = next(t for t, w in _times(left) if w.label == "lift clear")
    parked_to = next(t for t, w in _times(left) if w.label == "grip the handle" and t > parked_from)
    pos = np.array([*M.PARK_COVER_XY, M.COVER_SEATED_Z + G.CLEAT_SIZE[2]])
    upper = ["base", "link1", "link2", "link3", "link4", "link5"]
    for k in range(int(parked_from * 10), int(parked_to * 10) + 1):
        for arm, plan in plans.items():
            hulls = _arm_hulls(plan, arm, k / 10, upper)
            for _, centre, size in G.COVER_PARTS:
                d, link = analysis.hulls_to_box(hulls, pos + np.asarray(centre), np.eye(3), np.asarray(size))
                assert d > 0.01, (arm, k / 10, d, link)


def _times(plan):
    """(end time, waypoint) pairs of a plan."""
    t = 0.0
    for w in plan.waypoints:
        t += w.duration
        yield t, w
