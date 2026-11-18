"""CPU checks: the rack's shape and its keyholes, every scoring outcome and shortcut, and the scripted plan."""

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
from yam_arena.urdf import quat_xyzw_to_matrix  # noqa: E402

TASK = task_file.load(HERE / "task.py")
M = TASK.module
CLEAR = {"tcp": {"left": [0.15, 0.30, 0.95], "right": [0.15, -0.30, 0.95]}}


def quat(axis, angle):
    axis = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.r_[axis * np.sin(angle / 2), np.cos(angle / 2)]


def state(pos, q=(0, 0, 0, 1), moving=False):
    return ObjectState(pos=np.asarray(pos, float), quat_xyzw=np.asarray(q, float),
                       lin_vel=np.array([0.05 if moving else 0.0, 0, 0]), ang_vel=np.zeros(3),
                       low=np.zeros(3), high=np.zeros(3))  # fmt: skip


def hung(dx=0.0, dy=0.0, dz=0.0, q=(0, 0, 0, 1)):
    return {"rack": state(M.HANG_CENTRE + np.array([dx, dy, dz]), q)}


def start():
    return {"rack": state(M.START_CENTRE)}


# -- shapes -------------------------------------------------------------------------------------------------


def test_rack_frame_is_its_bounding_box_centre():
    lo, hi = G.rack_bounds_local()
    assert np.allclose((lo + hi) / 2, 0.0, atol=1e-9)
    assert np.allclose(hi - lo, M.RACK_SIZE)


def test_rack_mass_matches_its_materials():
    oak, zinc = 700.0, 2700.0  # kg/m3: oak, cast aluminium
    wood = sum(np.prod(s) for n, _, s in G.rack_parts() if not n.startswith(("plate", "arm", "tip")))
    hooks = sum(np.prod(s) for n, _, s in G.rack_parts() if n.startswith(("plate", "arm", "tip")))
    assert M.RACK_MASS == pytest.approx(wood * oak + hooks * zinc, abs=0.05)


def test_keyhole_passes_the_head_and_holds_it_by_the_throat():
    assert G.SCREW_HEAD_D < G.KEYHOLE_ENTRY  # the head goes through the entry
    assert G.SCREW_SHANK_D < G.THROAT < G.SCREW_HEAD_D  # the shank runs in the throat, the head cannot
    assert G.SCREW_HEAD_D - G.THROAT >= 0.002  # at least 1 mm of head over the lip each side
    assert G.SCREW_PROUD - G.SCREW_HEAD_THICKNESS >= G.LIP  # the lip fits under the head: the rack sits flush
    assert G.SCREW_PROUD - G.SCREW_HEAD_THICKNESS - G.LIP <= M.FLUSH_TOLERANCE - 0.001  # its play is inside the flush check
    assert G.BACK_LAYER - G.LIP >= G.SCREW_HEAD_THICKNESS  # the head fits in the channel behind the lip
    assert G.SCREW_PROUD < G.BACK_LAYER  # the head's top never reaches the front block
    assert G.REST_Z > G.ENTRY_TOP_Z + G.SCREW_SHANK_D  # the head is well up the channel when the rack rests on it


def test_back_pieces_leave_exactly_the_keyholes_open():
    """Probe the back layer: a point is covered by some piece everywhere except in the slots."""
    pieces = [(np.asarray(c), np.asarray(s)) for n, c, s in G.rack_parts()
              if n.startswith(("back_", "below_", "above_", "lip_"))]  # fmt: skip

    def covered(y, z, depth):  # depth below the back surface, m
        p = np.array([G.BACK_X - depth, y, z])
        return any(np.all(np.abs(p - c) <= s / 2 + 1e-9) for c, s in pieces)

    for ky in G.KEYHOLE_Y:
        assert not covered(ky, G.KEYHOLE_ENTRY_Z, 0.001) and not covered(ky, G.KEYHOLE_ENTRY_Z, 0.007)  # entry pocket
        assert not covered(ky, G.ENTRY_TOP_Z + 0.006, 0.001)  # the throat at the surface
        assert not covered(ky + 0.004, G.ENTRY_TOP_Z + 0.006, 0.005)  # the channel behind the lip is open
        assert covered(ky + 0.004, G.ENTRY_TOP_Z + 0.006, 0.001)  # but the lip is there at the surface
        assert covered(ky, G.CHANNEL_TOP_Z + 0.002, 0.001)  # solid above the channel
        assert covered(ky, G.ENTRY_BOTTOM_Z - 0.002, 0.001)  # solid below the entry
        assert covered(ky + 0.008, G.KEYHOLE_ENTRY_Z, 0.001)  # solid beside the entry
    assert covered(0.0, 0.0, 0.001) and covered(0.19, 0.03, 0.007)


def test_rack_stands_upright_at_the_start():
    """It stands on its bottom edge in front of the wall: the real rack's centre of mass (oak rail, light cast
    hooks) is inside its 25 mm base, and so is the simulated one's."""
    pts = np.concatenate([G.box_corners(s, c) for _, c, s in G.rack_parts()]) + M.START_CENTRE
    assert pts[:, 2].min() == pytest.approx(M.TOP + M.SPAWN_GAP, abs=1e-6)
    assert pts[:, 0].max() <= M.WALL_X - 0.04  # well clear of the wall and its screws
    assert pts[:, 2].max() < M.SCREW_Z - G.SCREW_HEAD_D  # and below the screws
    oak, aluminium = 700.0, 2700.0
    wood = [(np.prod(s) * oak, c[0]) for n, c, s in G.rack_parts() if not n.startswith(("plate", "arm", "tip"))]
    hooks = [(np.prod(s) * aluminium, c[0]) for n, c, s in G.rack_parts() if n.startswith(("plate", "arm", "tip"))]
    com_x = sum(m * x for m, x in wood + hooks) / sum(m for m, _ in wood + hooks)
    assert G.FRONT_X + 0.003 < com_x < G.BACK_X - 0.003  # inside the base with 3 mm to spare either way


def test_keyhole_lead_ins():
    pts = dict(G.rack_part_points())
    for name in ("below_entry_0", "lip_1_1"):
        assert len(pts[name]) == 10  # a box with one edge chamfered
    below = pts["below_entry_0"]
    assert below[:, 0].max() == pytest.approx(G.BACK_X)
    top_back = below[np.isclose(below[:, 0], G.BACK_X) & np.isclose(below[:, 2], G.ENTRY_BOTTOM_Z)]
    assert len(top_back) == 0  # the top-back edge is gone
    assert np.isclose(below[:, 2].max(), G.ENTRY_BOTTOM_Z)  # the top face is still there, inset by the chamfer


def test_starting_layout():
    names = [o.name for o in TASK.objects]
    assert len(names) == len(set(names))
    assert set(M.FREE) == set(TASK.free_objects)
    assert M.HANG_CENTRE[2] - G.RAIL_HEIGHT / 2 > M.TOP + 0.03  # the hung rack is well off the table
    assert M.HANG_TOP_Z < M.TOP + M.WALL_HEIGHT - 0.05  # and well below the wall section's top
    assert abs(M.MARK_Y[0]) > G.RAIL_LENGTH / 2 + 0.02  # the pencil ticks sit beside the rack, not under it


# -- outcomes ----------------------------------------------------------------------------------------------


def test_start_scores_zero():
    r = M.measure(start(), CLEAR)
    assert r["partial_score"] == 0.0 and not r["success_now"] and r["rack"]["hanging_on"] == 0


def test_full_score():
    r = M.measure(hung(), CLEAR)
    assert r["success_now"] and r["partial_score"] == 1.0 and r["rack"]["hanging_on"] == 2


def test_gripper_still_on_a_hook_is_not_success():
    tcp = {"tcp": {"left": M._hang_tcp("left").tolist(), "right": CLEAR["tcp"]["right"]}}
    r = M.measure(hung(), tcp)
    assert not r["success_now"] and r["partial_score"] == 1.0


def test_no_gripper_positions_is_not_success():
    assert not M.measure(hung(), {})["success_now"]


def test_moving_rack_is_not_success():
    objs = {"rack": state(M.HANG_CENTRE, moving=True)}
    assert not M.measure(objs, CLEAR)["success_now"]


def test_rack_with_play_in_the_throats_still_counts():
    r = M.measure(hung(dy=0.0009, dx=-0.0006), CLEAR)
    assert r["success_now"]


# -- what does not count as hung ---------------------------------------------------------------------------


def test_heads_in_the_entries_only_scores_zero():
    """Pushed on over the heads but not lowered: it would fall if let go."""
    objs = hung(dz=G.REST_Z - G.KEYHOLE_ENTRY_Z)
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == 0.0 and all(k["in_entry"] for k in r["rack"]["keyholes"])


def test_lowered_only_part_way_scores_zero():
    """Shanks in the throats but the rack still held 4 mm short of resting on them."""
    objs = hung(dz=0.004)
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == 0.0 and all(not k["in_throat"] for k in r["rack"]["keyholes"])
    assert M.measure(hung(dz=0.001), CLEAR)["success_now"]  # 1 mm short still counts: the play of a real slot


def test_hanging_on_one_keyhole_scores_half():
    """One shank in its throat, the other end dropped: the rack swings down about the screw."""
    tilt = np.radians(6.0)
    pivot = np.array([M.WALL_X - G.LIP / 2, M.SCREW_Y[1], M.SCREW_Z])  # the left screw
    local = np.array([G.BACK_X - G.LIP / 2, G.KEYHOLE_Y[1], G.REST_Z])
    q = quat([1, 0, 0], tilt)
    rot = quat_xyzw_to_matrix(q)
    objs = {"rack": state(pivot - rot @ local, q)}
    r = M.measure(objs, CLEAR)
    assert r["rack"]["keyholes"][1]["in_throat"] and not r["rack"]["keyholes"][0]["in_throat"]
    assert r["partial_score"] == pytest.approx(0.5) and not r["success_now"]


def test_rack_standing_against_the_wall_under_the_screws_scores_zero():
    objs = {"rack": state([M.WALL_X - G.BACK_X, 0.0, M.TOP + G.RAIL_HEIGHT / 2])}
    r = M.measure(objs, CLEAR)
    assert r["partial_score"] == 0.0 and not r["rack"]["off_table"]


def test_rack_off_the_wall_or_turned_scores_zero():
    assert M.measure(hung(dx=-0.006), CLEAR)["partial_score"] == 0.0  # 6 mm off the wall: the shanks leave the lips
    assert M.measure(hung(q=quat([0, 0, 1], np.radians(3))), CLEAR)["partial_score"] == 0.0  # turned: one keyhole misses


def test_rack_hung_but_tilted_fails_level():
    r = M.measure(hung(q=quat([1, 0, 0], np.radians(1.5))), CLEAR)
    assert not r["rack"]["level"] and not r["rack"]["hung"]


def test_rack_on_the_table_in_front_of_the_wall_scores_zero():
    objs = {"rack": state([0.30, 0.0, M.TOP + G.RACK_SIZE[2] / 2], quat([1, 0, 0], np.pi / 2))}
    assert M.measure(objs, CLEAR)["partial_score"] == 0.0


# -- the scripted plan ----------------------------------------------------------------------------------


def test_demo_plan_is_reachable_and_inside_the_limit():
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    assert end < 60.0  # the limit is 160 s
    for k in range(int(end * 20) + 1):
        for plan in plans.values():
            plan.action7(k / 20)
    for arm, plan in plans.items():
        assert plan.worst_ik_mm < 1.0 and plan.worst_ik_deg < 0.5, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


def test_demo_grips_the_tips_and_carries_the_rack_clear():
    for arm in ("left", "right"):
        g = M._tip_grip_point(arm)
        assert G.TIP_X < g[0] < G.TIP_X + G.HOOK_TIP[0]  # over the tip, not the arm
        assert abs(g[1]) == pytest.approx(0.160)
        pads_bottom = g[2] - 0.013
        assert pads_bottom > G.HOOK_ARM_Z + G.HOOK_ARM[2] / 2 + 0.002  # the pads stop short of the arm
        assert g[2] - pads_bottom >= 0.010
    assert 2 * abs(M.FINGER_APPROACH) >= G.HOOK_TIP[0] + 0.016  # 8 mm clear each side of the tip on approach
    assert G.TIP_CENTRE_X + abs(M.FINGER_APPROACH) < G.FRONT_X - 0.010  # and the open pad stays off the rail
    entry = M._entry_tcp("left")
    assert entry[2] - (G.TIP_TOP_Z + M.GRIP_ABOVE_TIP) - G.RAIL_HEIGHT / 2 >= M.TOP + M.LIFT_CLEAR - 0.001


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


def test_demo_arms_keep_off_the_wall():
    """Upper links stay 1 cm off the wall section, its screws and marks; the gripper and fingers 5 mm."""
    plans = {arm: ArmPlan(w, BASES[arm]) for arm, w in M.demo().items()}
    end = max(p.duration for p in plans.values())
    upper = ["base", "link1", "link2", "link3", "link4", "link5"]
    hand = ["gripper", "tip_left", "tip_right"]
    for k in range(int(end * 10) + 1):
        for arm, plan in plans.items():
            d, where = _nearest_fixed(_arm_hulls(plan, arm, k / 10, upper))
            assert d > 0.01, (arm, k / 10, d, where)
            d, where = _nearest_fixed(_arm_hulls(plan, arm, k / 10, hand))
            assert d > 0.005, (arm, k / 10, d, where)


def test_gripper_clears_the_rack_while_holding_a_tip():
    """With the pads on a tip, nothing but the finger links comes within 5 mm of any other part of the rack,
    both at the start and when it hangs."""
    for arm in ("left", "right"):
        plan = ArmPlan(M.demo()[arm], BASES[arm])
        poses = {"grip the tip": (M.START_CENTRE, np.eye(3)), "let it settle": (M.HANG_CENTRE, np.eye(3))}
        for t, w in _times(plan):
            if w.label not in poses:
                continue
            centre, rot = poses[w.label]
            hulls = _arm_hulls(plan, arm, t, ["base", "link1", "link2", "link3", "link4", "link5", "gripper"])
            tip_name = f"tip_{M.GRIP_HOOKS[arm]}"
            for name, c, s in G.rack_parts():
                if name == tip_name:
                    continue
                d, link = analysis.hulls_to_box(hulls, centre + rot @ np.asarray(c), rot, np.asarray(s))
                assert d > 0.005, (arm, w.label, name, d, link)


def _times(plan):
    """(end time, waypoint) pairs of a plan."""
    t = 0.0
    for w in plan.waypoints:
        t += w.duration
        yield t, w
