"""CPU checks: the two bodies' shapes, the layout, every scoring outcome and shortcut, and the scripted plan."""

import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))

import geometry as G  # noqa: E402
from yam_arena import analysis, task_file  # noqa: E402
from yam_arena.analysis import Hull  # noqa: E402
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP, TCP_OFFSET  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.station import BASES  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

TASK = task_file.load(HERE / "task.py")
M = TASK.module
CLEAR = {"tcp": {"left": [0.15, 0.30, 0.95], "right": [0.15, -0.30, 0.95]}}
UPPER = ["base", "link1", "link2", "link3", "link4", "link5"]
MOVING = UPPER[1:]  # the base never moves; it stands beside the pencil tick
HAND = ["gripper", "tip_left", "tip_right"]


def quat(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]


def rz(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def state(pos, q=(0, 0, 0, 1), moving=False):
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(q, float),
                       lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                       low=np.zeros(3), high=np.zeros(3))  # fmt: skip


def scene(toolbox_pos, toolbox_q=(0, 0, 0, 1), ext_pos=None, ext_q=(0, 0, 0, 1), moving=False):
    ext = M.EXT_START if ext_pos is None else ext_pos
    return {"toolbox": state(toolbox_pos, toolbox_q, moving), "extinguisher": state(ext, ext_q)}


def start():
    return scene(M.START_CENTRE, quat([0, 0, 1], M.TOOLBOX_YAW))


def placed(dx=0.0, dy=0.0, dz=0.0, q=(0, 0, 0, 1), **kwargs):
    """The toolbox resting where the demonstration sets it down, end-on to the wall."""
    return scene(M.PLACE - [0, 0, M.SPAWN_GAP] + np.array([dx, dy, dz]), q, **kwargs)


# -- shapes -------------------------------------------------------------------------------------------------


def test_frames_are_bounding_box_centres():
    lo, hi = G.toolbox_bounds_local()
    assert np.allclose((lo + hi) / 2, 0.0, atol=1e-9) and np.allclose(hi - lo, M.TOOLBOX_SIZE)
    lo, hi = G.extinguisher_bounds_local()
    assert np.allclose((lo + hi) / 2, 0.0, atol=1e-9) and np.allclose(hi - lo, G.EXT_SIZE)
    assert G.EXT_AXIS_LOCAL[2] == pytest.approx(lo[2])  # the axis point is on its base


def test_real_sizes_and_masses():
    assert 0.30 <= G.BOX_BODY[0] <= 0.35 and 0.16 <= G.BOX_BODY[1] <= 0.18 and 0.13 <= G.TOOLBOX_HEIGHT <= 0.15
    assert G.EXT_RADIUS * 2 == pytest.approx(0.083) and 0.35 <= G.EXT_SIZE[2] <= 0.40  # a 2.5 lb unit: 3.25 x 15 in
    assert 0.8 <= M.TOOLBOX_MASS <= 1.5 and 1.9 <= M.EXT_MASS <= 2.4


def test_handle_leaves_room_for_the_fingers():
    pads_bottom = G.BAR_TOP_Z - M.GRIP_BELOW_TOP - PAD_TIP_PAST_TCP
    assert pads_bottom <= G.BAR_BOTTOM_Z  # the pads cover the bar's full height
    assert pads_bottom - G.LID_Z >= 0.010  # and stop 10 mm or more above the lid
    assert 2 * abs(M.FINGER_APPROACH) >= G.BAR[1] + 0.020  # 10 mm clear each side of the bar on the way down
    assert 2 * G.POST_X - G.POST[0] >= 0.040 + 0.040  # the 40 mm pads fit between the posts with 2 cm each side


def test_extinguisher_sits_in_its_bracket():
    axis = M.EXT_START + G.EXT_AXIS_LOCAL
    assert axis[2] == pytest.approx(M.SADDLE_Z + M.SPAWN_GAP)
    assert M.WALL_X - M.PLATE[0] - (axis[0] + G.EXT_RADIUS) == pytest.approx(0.001)  # 1 mm off the back plate
    lip_inside = M.WALL_X - M.PLATE[0] - M.SADDLE[0] + M.LIP[0]
    assert 0.001 <= axis[0] - G.EXT_RADIUS - lip_inside <= 0.005  # the lip stops its base sliding off
    assert M.STRAP_Z - M.SADDLE_Z < G.EXT_BODY_H - M.STRAP_H  # the strap is round the straight body
    top = M.EXT_START[2] + G.EXT_SIZE[2] / 2
    assert M.TOP + 0.45 < top < M.TOP + M.WALL_HEIGHT - 0.10  # in front of the wall, well below its top


# -- layout -------------------------------------------------------------------------------------------------


def test_starting_layout():
    names = [o.name for o in TASK.objects]
    assert len(names) == len(set(names))
    assert set(M.FREE) == set(TASK.free_objects)
    corners = M.toolbox_corners(start()["toolbox"])
    assert np.abs(corners[:, 1]).max() < M.STRIP_HALF - 0.01  # wholly in the strip
    assert corners[:, 0].max() < M.WALL_X - M.PLATE[0] - M.SADDLE[0] - 0.08  # 8 cm short of the bracket
    assert corners[:, 0].min() > 0.10  # clear of the arm bases


def test_strip_covers_the_extinguisher_and_its_bracket():
    fixed = [o for o in TASK.objects if o.name.startswith(("bracket_", "strap_"))]
    for o in fixed:
        half = np.abs(rz(o.yaw)[:2, :2]) @ np.asarray(o.size[:2]) / 2
        assert abs(o.pos[1]) + half[1] < M.STRIP_HALF - 0.09, o.name
    for name, pts in G.extinguisher_part_points():
        assert np.abs(pts[:, 1] + M.EXT_START[1]).max() < M.STRIP_HALF - 0.14, name
    assert M.STRIP_HALF == pytest.approx(G.EXT_RADIUS + 0.150)  # the extinguisher's width plus 15 cm each side


def test_pencil_ticks_mark_the_strip_edges_faintly():
    ticks = [o for o in TASK.objects if o.name.startswith("tick_")]
    assert len(ticks) == 2
    for o in ticks:
        assert abs(o.pos[1]) == pytest.approx(M.STRIP_HALF)  # centred on the strip's edge
        assert o.pos[0] + o.size[0] / 2 == pytest.approx(M.WALL_X)  # at the wall's foot
        assert o.size[0] <= 0.02 and o.size[1] <= 0.002 and o.size[2] <= 0.0005  # a short, thin pencil line
        assert max(o.color) - min(o.color) < 0.05 and max(o.color) < 0.5  # grey, not a colour
    assert not [o for o in TASK.objects if o.name.startswith("tape")]  # nothing else marks the strip


def test_wall_either_side_of_the_strip_takes_the_toolbox():
    for target in M.TARGETS:
        corners = M.toolbox_corners(state(target - [0, 0, M.SPAWN_GAP]))
        assert np.abs(corners[:, 1]).min() > M.STRIP_HALF + 0.03  # 3 cm clear of the strip
        assert np.abs(corners[:, 1]).max() < M.WALL_WIDTH / 2 - 0.05  # in front of the wall
    assert -0.65 + 0.1 < M.TARGETS[1][1] - G.TOOLBOX_SIZE[1] / 2


# -- outcomes ----------------------------------------------------------------------------------------------


def test_start_scores_zero():
    r = M.measure(start(), CLEAR)
    assert r["partial_score"] == 0.0 and not r["success_now"] and r["toolbox"]["in_strip"]


def test_full_score_either_side():
    r = M.measure(placed(), CLEAR)
    assert r["success_now"] and r["partial_score"] == 1.0
    r = M.measure(scene(M.TARGETS[0] - [0, 0, M.SPAWN_GAP]), CLEAR)
    assert r["success_now"] and r["partial_score"] == 1.0
    assert M.distance(placed()) == 0.0


def test_broadside_to_the_wall_beside_the_strip_also_counts():
    y = -(M.STRIP_HALF + G.TOOLBOX_SIZE[0] / 2 + 0.01)
    objs = scene([M.WALL_X - 0.02 - G.TOOLBOX_SIZE[1] / 2, y, M.TOP + G.TOOLBOX_HEIGHT / 2], quat([0, 0, 1], np.pi / 2))
    assert M.measure(objs, CLEAR)["success_now"]


def test_against_the_wall_in_front_of_the_extinguisher_scores_zero():
    """The obvious spot: straight ahead, pushed up to the bracket."""
    x = M.WALL_X - M.PLATE[0] - M.SADDLE[0] - 0.005 - G.TOOLBOX_SIZE[0] / 2
    r = M.measure(scene([x, 0.0, M.TOP + G.TOOLBOX_HEIGHT / 2]), CLEAR)
    assert r["partial_score"] == 0.0 and r["toolbox"]["in_strip"]


def test_straddling_the_strip_edge_scores_zero():
    y = -(M.STRIP_HALF + G.TOOLBOX_SIZE[1] / 2 - 0.01)  # 1 cm over the strip's edge
    r = M.measure(placed(dy=y - M.PLACE[1]), CLEAR)
    assert r["partial_score"] == 0.0 and r["toolbox"]["strip_overlap_m"] == pytest.approx(0.01)
    assert M.measure(placed(dy=y - M.PLACE[1] - 0.011), CLEAR)["partial_score"] == 1.0


def test_clear_of_the_strip_but_out_on_the_table_scores_half():
    r = M.measure(placed(dx=-0.06), CLEAR)  # 7.5 cm off the wall
    assert r["partial_score"] == 0.5 and not r["success_now"] and not r["toolbox"]["against_wall"]
    assert M.measure(placed(dx=-0.034), CLEAR)["partial_score"] == 1.0  # 4.9 cm off still counts


def test_past_the_end_of_the_wall_is_not_against_it():
    y = -(M.WALL_WIDTH / 2 + G.TOOLBOX_SIZE[1] / 2 + 0.06)  # 6 cm beyond the wall's end, at the wall's line
    r = M.measure(placed(dy=y - M.PLACE[1]), CLEAR)
    assert r["partial_score"] == 0.5 and not r["toolbox"]["against_wall"]
    y = -(M.WALL_WIDTH / 2 + G.TOOLBOX_SIZE[1] / 2 - 0.03)  # overlapping the wall's end by 3 cm
    assert M.measure(placed(dy=y - M.PLACE[1]), CLEAR)["partial_score"] == 1.0


def test_on_its_side_against_the_wall_scores_half():
    q = quat([1, 0, 0], np.pi / 2)
    z = M.TOP + G.TOOLBOX_SIZE[1] / 2
    r = M.measure(scene([M.PLACE[0], M.PLACE[1] - 0.03, z], q), CLEAR)
    assert r["partial_score"] == 0.5 and not r["toolbox"]["upright"] and r["toolbox"]["set_down"]
    assert M.measure(placed(q=quat([1, 0, 0], np.radians(8))), CLEAR)["toolbox"]["upright"]


def test_still_held_scores_zero():
    grip = M._grip_tcp(M.PLACE)
    r = M.measure(placed(), {"tcp": {"left": CLEAR["tcp"]["left"], "right": grip.tolist()}})
    assert r["partial_score"] == 0.0 and not r["success_now"] and not r["hands_clear"]
    assert M.measure(placed(), {})["partial_score"] == 0.0  # no gripper positions: not released


def test_held_up_scores_zero():
    r = M.measure(placed(dz=0.03), CLEAR)
    assert r["partial_score"] == 0.0 and not r["toolbox"]["set_down"]


def test_off_the_table_scores_zero():
    r = M.measure(scene([0.30, -0.75, M.TOP - 0.10]), CLEAR)
    assert r["partial_score"] == 0.0 and r["toolbox"]["fallen"]


def test_extinguisher_knocked_scores_zero():
    assert M.measure(placed(ext_pos=M.EXT_START + [-0.015, 0, 0]), CLEAR)["partial_score"] == 0.0
    assert M.measure(placed(ext_pos=M.EXT_START + [0, 0, 0.02]), CLEAR)["partial_score"] == 0.0  # lifted out
    assert M.measure(placed(ext_q=quat([0, 1, 0], np.radians(6))), CLEAR)["partial_score"] == 0.0
    r = M.measure(placed(ext_pos=M.EXT_START + [0.001, 0.002, -0.0005]), CLEAR)  # settling in its play
    assert r["success_now"]
    off_strip = scene([0.10, -0.40, M.TOP + G.TOOLBOX_HEIGHT / 2], ext_pos=M.EXT_START + [-0.03, 0, 0])
    assert M.measure(off_strip, CLEAR)["partial_score"] == 0.0  # a knocked extinguisher voids the half score too


def test_moving_is_not_success():
    r = M.measure(placed(moving=True), CLEAR)
    assert r["partial_score"] == 1.0 and not r["success_now"]


# -- the scripted plan ----------------------------------------------------------------------------------


def _plans():
    return {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}


def _q(plan, t):
    q7, _ = plan.action7(t)
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(q7[:6])}
    q["joint7"] = q["joint8"] = float(q7[6] * FINGER_OPEN)
    return q


def _poses(plan, arm, t):
    return analysis.fk_link_poses_world(_q(plan, t), BASES[arm])


def _arm_hulls(plan, arm, t, links=None):
    return analysis.world_hulls(_poses(plan, arm, t), links)


def _times(plan):
    """(end time, waypoint) pairs of a plan."""
    t = 0.0
    for w in plan.waypoints:
        t += w.duration
        yield t, w


def _end_of(plan, label):
    return next(t for t, w in _times(plan) if w.label == label)


def _box_hull(centre, rot, size) -> Hull:
    half = np.asarray(size, dtype=float) / 2
    verts = np.asarray(centre) + G.box_corners(size) @ rot.T
    normals = np.concatenate([rot.T, -rot.T])  # rows: the box's +x, +y, +z, -x, -y, -z in the world
    d = normals @ np.asarray(centre) + np.r_[half, half]
    return Hull(vertices=verts, planes=np.c_[normals, d])


def _toolbox_hulls(centre, rot, skip=()):
    return [(n, _box_hull(centre + rot @ c, rot, s)) for n, c, s in G.toolbox_parts() if n not in skip]


FIXED = [(o.name, _box_hull(np.asarray(o.pos), rz(o.yaw), o.size)) for o in TASK.objects if not o.free]
EXTINGUISHER = [("extinguisher", _box_hull(M.EXT_START, np.eye(3), G.EXT_SIZE))]  # its bounding box


def test_demo_plan_is_reachable_and_inside_the_limit():
    plans = _plans()
    end = max(p.duration for p in plans.values())
    assert end + M.SUCCESS_HOLD_S < 0.5 * 2400 / 20  # well inside the 120 s limit
    for k in range(int(end * 20) + 1):
        for plan in plans.values():
            plan.action7(k / 20)
    for arm, plan in plans.items():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_demo_puts_it_beside_the_strip_against_the_wall():
    r = M.measure(placed(), CLEAR)
    assert r["toolbox"]["gap_to_wall_m"] == pytest.approx(0.015)
    assert not r["toolbox"]["in_strip"]
    park = np.asarray(M.PARK["right"])
    assert M._box_distance(park, placed()["toolbox"], G.TOOLBOX_SIZE, (0, 0, 0)) > 0.05  # parked well past the 3 cm release check


def test_demo_arms_never_meet():
    plans = _plans()
    end = max(p.duration for p in plans.values())
    for k in range(int(end * 10) + 1):
        gap, pair = analysis.hull_set_distance(*(_arm_hulls(plans[a], a, k / 10) for a in ("left", "right")))
        assert gap > 0.03, (k / 10, gap, pair)


def test_demo_arms_keep_off_the_wall_bracket_and_extinguisher():
    """The moving links stay 2 cm off everything fixed and the extinguisher; the gripper and fingers 1 cm."""
    plans = _plans()
    end = max(p.duration for p in plans.values())
    for k in range(int(end * 10) + 1):
        for arm, plan in plans.items():
            poses = _poses(plan, arm, k / 10)
            for links, margin in ((MOVING, 0.02), (HAND, 0.01)):
                gap, pair = analysis.hull_set_distance(analysis.world_hulls(poses, links), FIXED + EXTINGUISHER)
                assert gap > margin, (arm, k / 10, gap, pair)


def _carried_pose(plan, t, grip_pose):
    """The toolbox's pose at ``t`` if it stays where it was gripped in the hand (no slip, no sag)."""
    g = _poses(plan, "right", t)["gripper"]
    rel = np.linalg.inv(grip_pose) @ np.r_[np.c_[rz(M.TOOLBOX_YAW), M.START_CENTRE], [[0, 0, 0, 1]]]
    box = g @ rel
    return box[:3, 3], box[:3, :3]


def test_carried_toolbox_clears_the_arm_the_wall_and_the_extinguisher():
    plans = _plans()
    plan = plans["right"]
    t0, t1 = _end_of(plan, "grip the handle"), _end_of(plan, "let it settle")
    lowering = _end_of(plan, "turn it end-on to the wall")
    grip_pose = _poses(plan, "right", t0)["gripper"]
    tcp = grip_pose[:3, :3] @ np.asarray(TCP_OFFSET) + grip_pose[:3, 3]
    assert np.allclose(tcp, M._grip_tcp(M.START_CENTRE), atol=0.001)
    others = [h for h in FIXED if not h[0].startswith(("wall", "tick"))] + EXTINGUISHER
    wall = [h for h in FIXED if h[0] == "wall"]
    for k in range(int(t0 * 20), int(t1 * 20) + 1):
        t = k / 20
        centre, rot = _carried_pose(plan, t, grip_pose)
        box = _toolbox_hulls(centre, rot)
        gap, pair = analysis.hull_set_distance(box, others)
        assert gap > 0.05, (t, gap, pair)  # 5 cm or more from the bracket and the extinguisher
        gap, pair = analysis.hull_set_distance(box, wall)
        assert gap > 0.010, (t, gap, pair)
        gap, pair = analysis.hull_set_distance(box, _arm_hulls(plan, "right", t, UPPER))
        assert gap > 0.02, (t, gap, pair)
        gap, pair = analysis.hull_set_distance(box, _arm_hulls(plans["left"], "left", t))
        assert gap > 0.05, (t, gap, pair)
        if _end_of(plan, "lift the toolbox") <= t <= lowering:
            assert min(h.vertices[:, 2].min() for _, h in box) > M.TOP + 0.03, t  # carried clear of the table
    centre, rot = _carried_pose(plan, t1, grip_pose)
    assert np.allclose(centre, M.PLACE - [0, 0, M.SPAWN_GAP + M.PRESS], atol=0.002)
    assert np.allclose(rot, np.eye(3), atol=0.002)


def test_gripper_clears_the_toolbox_but_for_the_bar():
    """On the way down to the handle and after letting go, nothing comes within 3 mm of the toolbox; while
    holding it, only the finger pads touch it, on the bar."""
    plan = _plans()["right"]
    at_start = (M.START_CENTRE, rz(M.TOOLBOX_YAW))
    at_place = (M.PLACE - [0, 0, M.SPAWN_GAP], np.eye(3))
    windows = [
        (0.0, _end_of(plan, "down round the bar"), at_start, 0.003, ()),
        (_end_of(plan, "down round the bar"), _end_of(plan, "grip the handle"), at_start, 0.003, ("bar",)),
        (_end_of(plan, "let go"), plan.duration, at_place, 0.003, ()),
    ]
    for a, b, (centre, rot), margin, skip in windows:
        for k in range(int(a * 20), int(b * 20) + 1):
            hulls = _arm_hulls(plan, "right", k / 20)
            gap, pair = analysis.hull_set_distance(_toolbox_hulls(centre, rot, skip), hulls)
            assert gap > margin, (k / 20, gap, pair)
    hulls = _arm_hulls(plan, "right", _end_of(plan, "grip the handle"), UPPER + ["gripper"])
    gap, pair = analysis.hull_set_distance(_toolbox_hulls(*at_start), hulls)
    assert gap > 0.03, (gap, pair)  # the palm is well above the bar
