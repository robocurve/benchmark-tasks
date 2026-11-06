"""Offline checks for frame-on-ledge (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest test_task.py -q

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

for _candidate in (os.environ.get("YAM_ARENA_DIR"), "~/yam-isaac-lab-arena"):
    if _candidate and Path(_candidate).expanduser().is_dir():
        sys.path.insert(0, str(Path(_candidate).expanduser()))
        break

from yam_arena import ik, station  # noqa: E402
from yam_arena.analysis import box_signed_distance, link_hulls, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics  # noqa: E402

HZ = 20.0
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN
MAX_GRIP = 0.084  # 94 mm less a 1 cm margin
GRIP_OFFSET = 0.015  # a held part can sit this far off centre between the pads (12-20 mm measured in sim)
RUN_LIMIT_STEPS = 1200
CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)


# -- helpers ---------------------------------------------------------------------------------------------------


def roty(deg):
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), math.radians(deg))


def rotz(deg):
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), math.radians(deg))


def box_state(pos, size, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    r = np.eye(3) if r is None else np.asarray(r)
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    world = (CORNERS * half) @ r.T + pos
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def frame_state(pos, r=None, vel=(0, 0, 0)):
    return box_state(pos, T.FRAME_SIZE, r, vel)


def frame_low_at(xy, z_low, r):
    """The frame turned by ``r`` with its centre over ``xy`` and its lowest corner at ``z_low``."""
    half_h = float(np.abs((CORNERS * np.asarray(T.FRAME_SIZE) / 2) @ np.asarray(r).T)[:, 2].max())
    return frame_state((xy[0], xy[1], z_low + half_h), r)


def scene(frame=None) -> dict:
    out = {o.name: box_state(o.pos, o.size, rotz(math.degrees(o.yaw))) for o in TASK.objects}
    if frame is not None:
        out["frame"] = frame
    return out


def leaning(overlap=T.GOAL_OVERLAP, y=T.LEDGE_Y):
    c, r = T.lean_pose(overlap, y)
    return frame_state(c, r)


def fixed_boxes():
    return [o for o in TASK.objects if not o.free]


def surface_points(center, r, size, n=7) -> np.ndarray:
    g = np.linspace(-0.5, 0.5, n)
    pts = []
    for axis in range(3):
        for sign in (-0.5, 0.5):
            for a in g:
                for b in g:
                    p = np.zeros(3)
                    p[axis] = sign
                    p[(axis + 1) % 3], p[(axis + 2) % 3] = a, b
                    pts.append(p)
    return np.asarray(pts) * np.asarray(size) @ np.asarray(r).T + np.asarray(center)


def box_box_clearance(c1, r1, s1, c2, r2, s2) -> float:
    a = box_signed_distance(surface_points(c1, r1, s1), np.asarray(c2), np.asarray(r2), np.asarray(s2)).min()
    b = box_signed_distance(surface_points(c2, r2, s2), np.asarray(c1), np.asarray(r1), np.asarray(s1)).min()
    return float(min(a, b))


def hull_box_clearance(hulls, center, r, size) -> float:
    center, r, size = np.asarray(center), np.asarray(r), np.asarray(size)
    world = (CORNERS * size / 2) @ r.T + center
    lo, hi = world.min(0), world.max(0)
    pts = surface_points(center, r, size, n=9)
    best = np.inf
    for _, h in hulls:
        vlo, vhi = h.vertices.min(0), h.vertices.max(0)
        if np.maximum(lo - vhi, vlo - hi).max() > 0.03:
            continue
        best = min(best, box_signed_distance(h.vertices, center, r, size).min())
        best = min(best, (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1).min())
    return float(best)


def _arm_hulls(side, joints, finger):
    from yam_arena import YAM_URDF

    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})


@pytest.fixture(scope="module")
def played():
    out = {}
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = []
        for k in range(round(arm.duration * HZ) + 5):
            t = (k + 1) / HZ
            pos, r, finger, label = arm.target(t)
            action, _ = arm.action7(t)
            steps.append((t, action[:6].copy(), finger, pos, r, label))
        out[side] = (arm, steps)
    return out


def _first(steps, label):
    return next(s[0] for s in steps if s[5] == label)


def _last(steps, label):
    return max(s[0] for s in steps if s[5] == label)


# -- loading -----------------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert TASK.free_objects == ("frame",)
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream", "institut"):
        assert word not in TASK.docs.lower()
    text = (Path(__file__).resolve().parent / "task.json").read_text().lower()
    readme = (Path(__file__).resolve().parent / "README.md").read_text().lower()
    for word in ("university", "team ", "laboratory", "institut"):
        assert word not in text and word not in readme, word


# -- geometry ----------------------------------------------------------------------------------------------------


def test_the_frame_fits_the_gripper_across_its_width_only():
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    assert T.FRAME_W <= MAX_GRIP
    assert (GRIPPER_MAX_GAP - T.FRAME_W) / 2 == pytest.approx(0.007)  # 7 mm each side on the way down
    assert T.FRAME_H > GRIPPER_MAX_GAP  # its height cannot be gripped


def test_the_shelf_is_too_narrow_to_stand_on_so_a_frame_must_lean():
    # Standing upright with its front face against the lip, the frame's centre of mass is over the shelf's back
    # edge at best: it can only stay up by leaning back onto the wall.
    assert T.SEAT_D <= T.FRAME_T / 2
    # The demo's release: bottom 5 mm onto the shelf, centre of mass 5 mm behind the edge, 5 mm clear of the lip.
    com_x = T.PLACE_X
    assert com_x - T.SEAT_EDGE == pytest.approx(T.FRAME_T / 2 - T.GOAL_OVERLAP)
    assert com_x - T.SEAT_EDGE >= 0.005 - 1e-9
    assert (T.PLACE_X - T.FRAME_T / 2) - T.LIP_BACK == pytest.approx(0.005)
    # Any release from 0 to 10 mm onto the shelf (+-5 mm in x) tips back and ends leaning 12-17 deg, inside 3-20.
    for overlap in np.linspace(0.0005, T.FRAME_T / 2 - 0.0005, 20):
        a = T.lean_angle_deg(overlap)
        assert 11.0 < a < 18.0, (overlap, a)
        st = leaning(overlap)
        assert T.displayed(st), overlap
        assert st.high[0] == pytest.approx(T.WALL_FACE, abs=1e-6)  # the top back edge is on the wall
        assert st.high[2] < TOP + T.WALL_H - 0.01  # and below the wall's top
    assert T.lean_angle_deg(T.GOAL_OVERLAP) == pytest.approx(14.7, abs=0.1)


def test_a_frame_perched_on_the_lip_is_told_apart_from_one_on_the_shelf():
    # Leaning back from the lip's top edge, the frame's lowest corner stays at least 8 mm above the shelf.
    worst_dip = T.FRAME_T * math.sin(math.radians(T.LEAN_MAX_DEG))
    assert T.LIP_TOP - worst_dip > T.SEAT_TOP + T.FOOT_ABOVE
    # On the shelf edge the lowest corner dips at most 5 mm under the shelf top, inside the 8 mm allowance.
    for overlap in (0.0005, 0.005, 0.0095):
        assert T.SEAT_TOP - leaning(overlap).low[2] < T.FOOT_BELOW


def test_the_height_budget_fits_joint4():
    assert T.TRAVEL_Z - TOP <= 0.14
    assert T.TRAVEL_Z - T.GRIP_ABOVE_BOTTOM >= T.LIP_TOP + 0.010 - 1e-9  # frame bottom 1 cm over the lip
    assert T.TRAVEL_Z - T.GRIP_ABOVE_BOTTOM >= TOP + T.SLOT_STRIP[2] + 0.010
    lifted = T.SEAT_TOP + T.RELEASE_GAP + T.GRIP_ABOVE_BOTTOM + T.LIFT_AFTER_RELEASE
    assert lifted - TOP <= 0.145


# -- starting layout ---------------------------------------------------------------------------------------------


def _bounds(o):
    st = box_state(o.pos, o.size, rotz(math.degrees(o.yaw)))
    return st.low, st.high


def test_nothing_overlaps_at_the_start():
    objects = list(TASK.objects)
    for i, a in enumerate(objects):
        lo_a, hi_a = _bounds(a)
        for b in objects[i + 1 :]:
            if {a.name, b.name} == {"ledge_rail", "ledge_lip"}:
                continue  # the lip sits on the rail
            lo_b, hi_b = _bounds(b)
            gap = np.maximum(lo_b - hi_a, lo_a - hi_b).max()
            assert gap > 0.0029, (a.name, b.name, gap)
    lo, hi = _bounds(T.LIP)
    rlo, rhi = _bounds(T.RAIL)
    assert lo[2] == pytest.approx(rhi[2]) and lo[0] == pytest.approx(rlo[0])


def test_everything_stands_on_the_table_and_the_frame_starts_upright_in_the_slot():
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for o in TASK.objects:
        lo, hi = _bounds(o)
        assert np.all(lo[:2] > tlo) and np.all(hi[:2] < thi), o.name
        if o.free:
            assert 0.0 <= lo[2] - TOP <= 0.002 and o.yaw == 0.0
        elif o.name != "ledge_lip":
            assert lo[2] == pytest.approx(TOP, abs=1e-9), o.name
    flo, fhi = _bounds(T.FRAME)
    for strip in (T.SLOT_FRONT, T.SLOT_BACK):
        lo, hi = _bounds(strip)
        assert min(abs(lo[0] - fhi[0]), abs(flo[0] - hi[0])) == pytest.approx(T.SLOT_CLEAR)
        assert lo[1] < flo[1] + 0.02 and hi[1] > fhi[1] - 0.02  # the strips span most of the frame's width


def test_the_start_scores_zero_and_is_far_from_the_goal():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert TASK.distance(start) > 0.2


def test_the_ready_pose_and_the_parked_arms_leave_the_frame_and_ledge_visible():
    """The ready-pose wrist and the parked grippers are at least 6 cm (in plan) from the frame in the slot and from
    the ledge, so the top camera sees both in the first and the last frame."""
    targets = [_bounds(T.FRAME), _bounds(T.RAIL), _bounds(T.WALL)]
    for side in ("left", "right"):
        ready, _ = ik.tcp_pose(np.array(READY), np.array(station.BASES[side]))
        park = np.array(T.PARK[side])
        for p in (ready, park):
            for lo, hi in targets:
                d = np.maximum(np.maximum(lo[:2] - p[:2], p[:2] - hi[:2]), 0.0)
                assert np.linalg.norm(d) > 0.06, (side, p, lo, hi)


# -- scoring -----------------------------------------------------------------------------------------------------


def test_the_demo_end_state_scores_one():
    goal = scene(leaning())
    assert TASK.success(goal) and TASK.score(goal) == 1.0
    assert TASK.distance(goal) == pytest.approx(0.0, abs=1e-9)


def test_the_lean_scores_one_anywhere_along_the_ledge_with_small_turns():
    for overlap in (0.001, 0.005, 0.009):
        for dy in (-0.08, -GRIP_OFFSET, 0.0, GRIP_OFFSET, 0.08):
            c, r = T.lean_pose(overlap, T.LEDGE_Y + dy)
            for yaw in (-8.0, 0.0, 8.0):
                rr = rotz(yaw) @ r
                st = frame_state(c, rr)
                # a turned frame touches the wall with one top corner: shift it back onto the wall
                st = frame_state(c + np.array([T.WALL_FACE - st.high[0], 0, 0]), rr)
                assert TASK.score(scene(st)) == 1.0, (overlap, dy, yaw)


def test_upright_on_the_shelf_not_touching_the_wall_scores_0_4():
    st = frame_low_at((T.LIP_BACK + T.FRAME_T / 2, T.LEDGE_Y), T.SEAT_TOP, np.eye(3))
    assert TASK.score(scene(st)) == 0.4 and not TASK.success(scene(st))


def test_tipped_forward_onto_the_lip_scores_0_4():
    # foot on the shelf, face resting on the lip's top edge, top toward the arms
    for tilt in (8.0, 25.0, 40.0):
        st = frame_low_at((T.LIP_BACK + 0.012, T.LEDGE_Y), T.SEAT_TOP, roty(-tilt))
        assert T.back_tilt_deg(st) == pytest.approx(-tilt)
        assert TASK.score(scene(st)) == 0.4, tilt


def test_lying_flat_on_the_ledge_scores_0_4():
    # on its back across the lip top, long side along the ledge
    r = rotz(90.0) @ roty(90.0)
    st = frame_low_at((T.RAIL_FRONT + 0.03, T.LEDGE_Y), T.LIP_TOP, r)
    assert abs(T.side_tilt_deg(st)) > 80  # face up, top pointing along the ledge
    assert TASK.score(scene(st)) == 0.4


def test_leaning_more_than_20_deg_or_perched_on_the_lip_scores_0_4():
    c, r = T.lean_pose(T.GOAL_OVERLAP)
    steep = frame_low_at((c[0] + 0.01, c[1]), T.SEAT_TOP - 0.004, roty(24.0))
    assert TASK.score(scene(steep)) == 0.4
    # perched on the lip's top edge, leaning back onto the wall
    a = 19.0
    pivot = np.array([T.LIP_BACK, T.LEDGE_Y, T.LIP_TOP])
    rr = roty(a)
    perched = frame_state(pivot + rr @ np.array([T.FRAME_T / 2 - 0.004, 0, T.FRAME_H / 2]), rr)
    assert TASK.score(scene(perched)) == 0.4


def test_shortcuts_and_misses_score_zero():
    # left in the slot
    assert TASK.score(scene()) == 0.0
    # tipped forward over the lip onto the table in front of the ledge
    fallen = frame_low_at((T.RAIL_FRONT - 0.07, T.LEDGE_Y), TOP, roty(-90.0))
    assert TASK.score(scene(fallen)) == 0.0
    # dropped into the gap behind the shelf, leaning on the wall from the table
    pivot = np.array([T.WALL_FACE - 0.045, T.LEDGE_Y, TOP])
    r = roty(10.0)
    gap = frame_state(pivot + r @ np.array([-T.FRAME_T / 2, 0, T.FRAME_H / 2]), r)
    assert gap.low[2] < T.SEAT_TOP - 0.015
    assert TASK.score(scene(gap)) == 0.0
    # standing on the table in front of the ledge, leaning on the lip
    front = frame_low_at((T.RAIL_FRONT - 0.02, T.LEDGE_Y), TOP, roty(5.0))
    assert TASK.score(scene(front)) == 0.0
    # lying on top of the wall
    top = frame_low_at((T.WALL_FACE + T.WALL_T / 2, T.LEDGE_Y), TOP + T.WALL_H, rotz(90.0) @ roty(90.0))
    assert TASK.score(scene(top)) == 0.0
    # off the end of the ledge
    c, r = T.lean_pose(T.GOAL_OVERLAP, T.LEDGE_Y + T.LEDGE_LEN / 2 + 0.01)
    assert TASK.score(scene(frame_state(c, r))) == 0.0
    # in the lean pose but still moving (held in the air or falling)
    c, r = T.lean_pose(T.GOAL_OVERLAP)
    assert TASK.score(scene(frame_state(c, r, vel=(0.0, 0.0, -0.05)))) == 0.0
    # held above the ledge
    held = frame_low_at((T.PLACE_X, T.LEDGE_Y), T.LIP_TOP + 0.02, np.eye(3))
    assert TASK.score(scene(held)) == 0.0


def test_a_frame_turned_edge_on_or_side_tilted_does_not_get_full_score():
    c, r = T.lean_pose(T.GOAL_OVERLAP)
    turned = frame_state(c, rotz(30.0) @ r)
    assert TASK.score(scene(turned)) < 1.0
    side = frame_state(c, axis_angle_matrix(np.array([1.0, 0, 0]), math.radians(15.0)) @ r)
    assert TASK.score(scene(side)) < 1.0


# -- the demo ------------------------------------------------------------------------------------------------------


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(played):
    durations = set()
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side
        durations.add(round(arm.duration, 6))
        assert arm.duration * HZ + 100 < RUN_LIMIT_STEPS
    assert len(durations) == 1


def test_the_demo_follows_the_gpu_round_rules():
    for side, wps in TASK.demo().items():
        last, prev = wps[-1], wps[-2]
        assert last.label == "hold still" and last.duration >= 1.5, side
        assert tuple(last.pos) == tuple(prev.pos) and last.finger == prev.finger
        assert max(w.pos[2] for w in wps) <= TOP + 0.145
    wps = TASK.demo()["left"]
    for a, b in zip(wps, wps[1:], strict=False):
        if b.label == "lower onto the ledge":
            assert (a.pos[2] - b.pos[2]) / b.duration <= 0.035
        if b.label == "release":
            assert a.label == "steady" and a.pos == b.pos
        if a.label == "release":
            assert b.label == "lift off" and b.pos[:2] == a.pos[:2]
            assert b.pos[2] - a.pos[2] >= 0.03 - 1e-9


def test_the_release_puts_the_frame_bottom_3_mm_over_the_shelf_behind_the_lip(played):
    _, steps = played["left"]
    k = next(i for i, s in enumerate(steps) if s[5] == "release")
    pos = steps[k][3]
    bottom = pos[2] - T.GRIP_ABOVE_BOTTOM
    assert bottom - T.SEAT_TOP == pytest.approx(T.RELEASE_GAP, abs=1e-4)
    assert pos[0] == pytest.approx(T.PLACE_X) and pos[1] == pytest.approx(T.PLACE_Y)


def test_the_arms_clear_the_fixtures_and_the_frame(played):
    """Every other control step: no arm link within 5 mm of a fixed box, or of the frame when not holding it
    (in the slot before the grip, in its settled lean from 0.6 s after the release), finger tips above the table,
    and the carried frame, 15 mm off centre either way between the pads, at least 3 mm clear of every fixed box
    until it is let go. Fingers are taken wide open while the frame is held (either pad can sit at its open stop)."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    fixed = [(o.name, np.asarray(o.pos), np.eye(3), np.asarray(o.size)) for o in fixed_boxes()]
    lean = leaning()
    lean_box = ("frame", lean.pos, T._rot(lean), np.asarray(T.FRAME_SIZE))
    slot_box = ("frame", np.asarray(T.FRAME.pos), np.eye(3), np.asarray(T.FRAME_SIZE))
    worst_arm, worst_frame = np.inf, np.inf
    for side, (_, steps) in played.items():
        close = _first(steps, "close") if side == "left" else np.inf
        release = _first(steps, "release") if side == "left" else np.inf
        for k, (t, joints, finger, pos, r, label) in enumerate(steps):
            if k % 2:
                continue
            holding = close + 0.8 <= t < release
            hulls = _arm_hulls(side, joints, FINGER_OPEN if holding else finger)
            assert min(h.vertices[:, 2].min() for _, h in hulls) > TOP + 0.005, (side, label)
            boxes = list(fixed)
            if t < close:
                boxes.append(slot_box)
            elif t >= release + 0.6:
                boxes.append(lean_box)
            for name, c, rr, s in boxes:
                worst_arm = min(worst_arm, hull_box_clearance(hulls, c, rr, s))
            if holding:
                item_r = r @ grip_rotation(T.YAW).T
                centre = np.asarray(pos) + np.array([0.0, 0.0, T.FRAME_H / 2 - T.GRIP_ABOVE_BOTTOM])
                for shift in (-GRIP_OFFSET, 0.0, GRIP_OFFSET):
                    c = centre + shift * item_r[:, 1]
                    for _, bc, br, bs in fixed:
                        worst_frame = min(worst_frame, box_box_clearance(c, item_r, T.FRAME_SIZE, bc, br, bs))
    assert worst_arm > 0.005, worst_arm
    assert worst_frame > 0.0029, worst_frame  # 3 mm over the shelf at the release

