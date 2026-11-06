"""Offline checks for the grade-three-robot-attempts task (numpy + pytest only, no Isaac Sim).

Run with the yam-isaac-lab-arena clone (commit bd62771) importable, e.g.
    YAM_ARENA_DIR=~/yam-isaac-lab-arena pytest -q test_task.py
"""

import itertools
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import pytest

for _var in ("YAM_ARENA_DIR", "YAM_ARENA_PATH"):
    if os.environ.get(_var):
        sys.path.insert(0, os.path.expanduser(os.environ[_var]))

from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena.analysis import box_signed_distance, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY  # noqa: E402
from yam_arena.demo import ArmPlan  # noqa: E402
from yam_arena.task_file import Box, Cylinder, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics, quat_xyzw_to_matrix  # noqa: E402

TASK = load(Path(__file__).with_name("task.py"))
M = TASK.module
META = json.loads(Path(__file__).with_name("task.json").read_text())
HZ = 20
RUN_LIMIT_STEPS = 2400
PAD_HALF_WIDTH = 0.020  # finger pad extent across the closing direction (tip meshes)
PAD_OUTER = 0.057  # outer face of an open finger pad from the TCP along the closing direction
GRIP_OFFSET = 0.018  # measured in sim: a 40-45 mm part is held 18-19 mm off the TCP, toward -y
GREENS = [n for n in M.TOKEN_NAMES if M.TOKEN_COLOR[n] == "green"]
REDS = [n for n in M.TOKEN_NAMES if M.TOKEN_COLOR[n] == "red"]
Z = np.array([0.0, 0.0, 1.0])
SCORES = {0.0, 1 / 3, 2 / 3, 1.0}


# -- helpers ----------------------------------------------------------------------------------------------


def quat(yaw=0.0, tilt_deg=0.0):
    r = axis_angle_matrix(Z, yaw) @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(tilt_deg))
    return np.asarray(ik.matrix_to_quat_xyzw(r), dtype=float), r


def state(pos, half, vel=(0.0, 0.0, 0.0), yaw=0.0, tilt_deg=0.0):
    """An ObjectState as the simulator reports it: world-aligned bounds of the turned body."""
    pos = np.asarray(pos, dtype=float)
    q, r = quat(yaw, tilt_deg)
    ext = np.abs(r) @ np.asarray(half, dtype=float)
    return ObjectState(pos, q, np.asarray(vel, dtype=float), np.zeros(3), pos - ext, pos + ext)


CUBE_HALF = (M.CUBE / 2,) * 3
TOKEN_HALF = (M.TOKEN_R, M.TOKEN_R, M.TOKEN_H / 2)


def cube_state(pos, yaw=0.0, **kw):
    return state(pos, CUBE_HALF, yaw=yaw, **kw)


def token_state(pos, **kw):
    return state(pos, TOKEN_HALF, **kw)


def fixed_states():
    out = {}
    for o in M.OBJECTS:
        if not o.free:
            p, h = np.asarray(o.pos), np.asarray(o.size) / 2
            out[o.name] = ObjectState(p, np.array([0, 0, 0, 1.0]), np.zeros(3), np.zeros(3), p - h, p + h)
    return out


def start_state():
    objects = fixed_states()
    for b in M.BAYS:
        objects[M.CUBE_OF[b]] = cube_state(M.CUBE_START[b], yaw=M.CUBE_PLACEMENT[b][1])
    for o in M.TOKENS:
        objects[o.name] = token_state((o.pos[0], o.pos[1], o.pos[2] - M.SPAWN_GAP))
    return objects


def on_pad_pos(bay, dx=0.0, dy=0.0):
    return (M.PAD_X + dx, M.BAY_Y[bay] + dy, M.CARD_TOP + M.TOKEN_H / 2)


def flag(objects, name, bay, dx=0.0, dy=0.0):
    objects[name] = token_state(on_pad_pos(bay, dx, dy))


def flagged(colours):
    """The start state with a token of colour ``colours[bay]`` on each pad (a list of colours puts several,
    side by side; None leaves the pad empty)."""
    objects = start_state()
    left = {"green": list(GREENS), "red": list(REDS)}
    for b in M.BAYS:
        want = colours.get(b)
        if want is None:
            continue
        want = [want] if isinstance(want, str) else want
        for k, c in enumerate(want):
            flag(objects, left[c].pop(0), b, dy=(-0.025, 0.025)[k] if len(want) > 1 else 0.0)
    return objects


def goal_state():
    return flagged(dict(M.VERDICT))


def aabb(o):
    p = np.asarray(o.pos, dtype=float)
    if isinstance(o, Box):
        _, r = quat(o.yaw)
        h = np.abs(r) @ (np.asarray(o.size) / 2)
    else:
        h = np.array([o.radius, o.radius, o.height / 2])
    return p - h, p + h


# -- 1. the file loads ------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_hooks_and_docs():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.free_objects == ("cube1", "cube2", "cube3", *M.TOKEN_NAMES)
    assert TASK.docs.startswith("On the station table")
    assert all(isinstance(o, (Box, Cylinder)) for o in TASK.objects)  # primitives only, no library assets
    assert len(GREENS) == 3 and len(REDS) == 3


def test_the_verdicts_are_one_success_and_two_failures():
    assert M.VERDICT == {1: "red", 2: "green", 3: "red"}


# -- 2. the demo is reachable -----------------------------------------------------------------------------


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_demo_is_reachable_within_the_joint_limits(side):
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    arm = ArmPlan(plan[side], station.BASES[side])
    steps = round(arm.duration * HZ) + 5
    actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER)


def test_the_demo_fits_the_run_limit_and_both_arms_end_together():
    plan = TASK.demo()
    durations = [sum(w.duration for w in plan[s]) for s in ("left", "right")]
    assert durations[0] == pytest.approx(durations[1])
    assert durations[0] * HZ + 100 < RUN_LIMIT_STEPS / 2


def test_the_demo_ends_with_a_still_hold_of_at_least_1_5_s():
    for wps in TASK.demo().values():
        last = wps[-1]
        assert last.duration >= 1.5 and last.pos == wps[-2].pos and last.finger == wps[-2].finger


def test_the_demo_puts_the_verdict_colour_on_every_pad():
    plan = TASK.demo()
    placed = {}
    for side, wps in plan.items():
        for i, w in enumerate(wps):
            if w.label.startswith("over token"):
                name = w.label.split()[-1]
                assert w.pos[:2] == pytest.approx(next(o.pos[:2] for o in M.TOKENS if o.name == name))
                carry = next(v for v in wps[i:] if v.label.startswith("carry over pad"))
                bay = int(carry.label[-1])
                drop = next(v for v in wps[i:] if v.label == "let go")
                assert drop.pos[:2] == pytest.approx((M.PAD_X, M.BAY_Y[bay] + M.PLACE_AIM_Y))
                placed[bay] = M.TOKEN_COLOR[name]
    assert placed == M.VERDICT


def sample(plan, side, dt=0.05):
    arm = ArmPlan(plan[side], station.BASES[side])
    return [arm.target(t) for t in np.arange(0.0, arm.duration + dt, dt)]


def test_the_tcps_stay_apart_and_never_pass_beyond_the_pads():
    plan = TASK.demo()
    left, right = sample(plan, "left"), sample(plan, "right")
    assert min(np.linalg.norm(a[0] - b[0]) for a, b in zip(left, right, strict=True)) > 0.18
    cube_near_edge = min(M.CUBE_START[b][0] for b in M.BAYS) - 0.035  # the turned cube reaches 3.4 cm out
    for side in ("left", "right"):
        for wp in plan[side]:
            assert wp.pos[0] + PAD_HALF_WIDTH < cube_near_edge - 0.03, wp


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})


def _played(side, dt=0.1):
    """(time, joints, finger) at every dt of one arm's demo, with the IK the scripted policy runs."""
    arm = ArmPlan(TASK.demo()[side], station.BASES[side])
    out = []
    for t in np.arange(dt, arm.duration + dt, dt):
        a, _ = arm.action7(t)
        out.append((t, a[:6].copy(), FINGER_OPEN * a[6]))
    return out


@pytest.mark.parametrize("side", ["left", "right"])
def test_no_arm_link_comes_within_4_cm_of_a_cube_during_the_demo(side):
    start = start_state()
    for _, q, finger in _played(side):
        hulls = _arm_hulls(side, q, finger)
        for b in M.BAYS:
            s = start[M.CUBE_OF[b]]
            r = quat_xyzw_to_matrix(s.quat_xyzw)
            gap = min(box_signed_distance(h.vertices, s.pos, r, np.array([M.CUBE] * 3)).min() for _, h in hulls)
            assert gap > 0.04, (side, b, gap)


def test_open_pads_at_each_pick_clear_the_neighbouring_tokens():
    ys = sorted(o.pos[1] for o in M.TOKENS)
    for a, b in zip(ys, ys[1:]):
        assert (b - a) - PAD_OUTER - M.TOKEN_R >= 0.02  # the open finger's outer face to the next token
    for o in M.TOKENS:  # open pads (57 mm out along y) and the pad width (20 mm along x) stay off the flag pads
        assert o.pos[0] + PAD_HALF_WIDTH < M.PAD_X - M.PAD / 2 - 0.02


def test_the_pads_rise_straight_up_after_each_release():
    for wps in TASK.demo().values():
        for i, w in enumerate(wps):
            if w.label != "let go":
                continue
            rise = wps[i + 1]
            assert w.finger == FINGER_OPEN and rise.label == "rise"
            assert rise.pos[:2] == w.pos[:2] and rise.pos[2] - w.pos[2] >= 0.03
            assert w.pos[2] - PAD_TIP_PAST_TCP > M.CARD_TOP + 0.010  # pad tips stay off the flag pad


def test_the_grip_heights_keep_the_pad_tips_off_the_table_and_hold_the_upper_part():
    assert M.GRASP_Z - PAD_TIP_PAST_TCP > M.TOP + 0.010
    assert M.GRIP_BELOW_TOP + PAD_TIP_PAST_TCP >= 0.025  # a 2.5 cm band held well on the GPU
    assert M.RELEASE_Z - (M.TOKEN_H - M.GRIP_BELOW_TOP) == pytest.approx(M.CARD_TOP + M.RELEASE_GAP)
    carried_bottom = M.TRAVEL_Z - (M.TOKEN_H - M.GRIP_BELOW_TOP)
    assert carried_bottom > M.TOP + M.TOKEN_H + 0.03  # a carried token passes 3 cm over the row


# -- camera: the first and last frames show every scored object --------------------------------------------


def _shadow_contains(hull_vertices, cam, point):
    """Whether the hull hides ``point`` from the camera: the point lies inside the 2-D convex hull of the hull's
    vertices projected along camera rays onto the horizontal plane through the point."""
    v = hull_vertices[hull_vertices[:, 2] > point[2] + 1e-6]
    if len(v) < 3:
        return False
    t = (v[:, 2] - point[2]) / (cam[2] - v[:, 2])
    hit = v[:, :2] + (v[:, :2] - cam[:2]) * t[:, None]
    pts = sorted(map(tuple, hit))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    poly = lower[:-1] + upper[:-1]
    q = point[:2]
    return all(cross(poly[i], poly[(i + 1) % len(poly)], q) >= 0 for i in range(len(poly)))


def _scored_points(with_tokens):
    pts = []
    for b in M.BAYS:
        x, y, z = M.CUBE_START[b]
        pts += [(x + dx, y + dy, z + M.CUBE / 2) for dx in (-0.025, 0.0, 0.025) for dy in (-0.025, 0.0, 0.025)]
        cx, cy = M.BAY_X, M.BAY_Y[b]
        pts += [(cx + dx, cy + dy, M.CARD_TOP) for dx in (-0.075, 0.0, 0.075) for dy in (-0.075, 0.0, 0.075)]
        pts += [(M.PAD_X + dx, M.BAY_Y[b] + dy, M.CARD_TOP + M.TOKEN_H) for dx in (-0.03, 0.0, 0.03)
                for dy in (-0.03, 0.0, 0.03)]
    if with_tokens:
        pts += [(o.pos[0], o.pos[1], o.pos[2] + M.TOKEN_H / 2) for o in M.TOKENS]
    return pts


def test_the_ready_pose_hides_no_bay_pad_or_token_in_the_first_frame():
    cam = np.asarray(station.top_camera().pos)
    for side in ("left", "right"):
        for _, h in _arm_hulls(side, np.array(READY), FINGER_OPEN):
            for p in _scored_points(with_tokens=True):
                assert not _shadow_contains(h.vertices, cam, np.asarray(p)), (side, p)


def test_the_parked_arms_hide_no_bay_or_pad_in_the_last_frame():
    cam = np.asarray(station.top_camera().pos)
    for side in ("left", "right"):
        _, q, _ = _played(side)[-1]
        for _, h in _arm_hulls(side, q, FINGER_CLOSED):
            for p in _scored_points(with_tokens=False):
                assert not _shadow_contains(h.vertices, cam, np.asarray(p)), (side, p)


def project(cam, lens, p):
    """Pixel (u, v) of world point p in an OpenGL-convention camera (looks along -z, +y up)."""
    r = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    c = r.T @ (np.asarray(p) - np.asarray(cam.pos))
    return np.array([lens.cx + lens.fx * c[0] / -c[2], lens.cy - lens.fy * c[1] / -c[2]])


def test_a_failed_cube_sits_well_over_ten_pixels_off_its_cross_in_the_320x176_top_view():
    cam = station.top_camera()
    lens = cam.lens.resized(320, 176)
    for b in M.BAYS:
        z = M.CUBE_START[b][2] + M.CUBE / 2  # compare at the cube's top face, where the camera sees it
        centre = project(cam, lens, (M.BAY_X, M.BAY_Y[b], z))
        cube = project(cam, lens, (*M.CUBE_START[b][:2], z))
        uv = np.array([project(cam, lens, (M.BAY_X + dx, M.BAY_Y[b] + dy, M.CARD_TOP))
                       for dx in (-M.BAY / 2, M.BAY / 2) for dy in (-M.BAY / 2, M.BAY / 2)])
        assert uv[:, 0].min() >= 0 and uv[:, 0].max() <= 320 and uv[:, 1].min() >= 0 and uv[:, 1].max() <= 176
        off = np.linalg.norm(cube - centre)
        if M.VERDICT[b] == "red":
            assert off >= 10.0, (b, off)
        else:
            assert off <= 1.0
        strip = np.linalg.norm(project(cam, lens, (M.BAY_X, M.BAY_Y[b] + M.STRIP / 2, M.CARD_TOP))
                               - project(cam, lens, (M.BAY_X, M.BAY_Y[b] - M.STRIP / 2, M.CARD_TOP)))
        assert strip >= 3.5  # the cross's 2 cm arms are about 4 px wide


# -- 3. goal state and partial credit ---------------------------------------------------------------------


def test_the_goal_state_succeeds_with_full_score_and_zero_distance():
    goal = goal_state()
    assert TASK.success(goal) and TASK.score(goal) == 1.0 and TASK.distance(goal) == 0.0


def test_the_goal_holds_with_the_measured_grip_offset():
    objects = start_state()
    left = {"green": list(GREENS), "red": list(REDS)}
    for b, c in M.VERDICT.items():
        flag(objects, left[c].pop(0), b, dy=M.PLACE_AIM_Y - GRIP_OFFSET - 0.010)  # aim, grip offset, release shift
    assert TASK.success(objects) and TASK.score(objects) == 1.0
    assert M.PAD / 2 - (GRIP_OFFSET + 0.010) >= 0.020  # the centre is still 2 cm inside the pad's edge


def test_the_start_state_scores_zero():
    s = start_state()
    assert not TASK.success(s) and TASK.score(s) == 0.0 and TASK.distance(s) > 0.15


@pytest.mark.parametrize("k", range(4))
def test_each_right_flag_is_worth_a_third(k):
    objects = flagged({b: M.VERDICT[b] for b in M.BAYS[:k]})
    assert TASK.score(objects) == pytest.approx(k / 3)
    assert TASK.success(objects) == (k == 3)


def test_two_right_flags_and_an_empty_pad_score_two_thirds():
    for empty in M.BAYS:
        objects = flagged({b: M.VERDICT[b] for b in M.BAYS if b != empty})
        assert TASK.score(objects) == pytest.approx(2 / 3) and not TASK.success(objects)


def test_a_wrong_flag_costs_two_thirds():
    objects = flagged({1: "red", 2: "green", 3: "green"})
    assert TASK.score(objects) == 0.0  # 2/3 - 2/3
    objects = flagged({1: "red", 2: "red"})
    assert TASK.score(objects) == 0.0  # 1/3 - 2/3, floored
    objects = flagged({1: "red", 2: "green", 3: ["red", "green"]})
    assert TASK.score(objects) == 0.0  # both colours on one pad count as a wrong flag


def test_two_tokens_of_the_right_colour_on_a_pad_still_count_once():
    objects = flagged({1: ["red", "red"], 2: "green", 3: "red"})
    assert TASK.score(objects) == 1.0 and TASK.success(objects)


def test_a_token_still_moving_held_or_stacked_does_not_flag_a_pad():
    objects = goal_state()
    g = next(n for n in GREENS if M.on_pad(objects[n], 2))
    objects[g] = token_state(objects[g].pos, vel=(0.0, 0.0, -0.05))  # falling onto the pad
    assert TASK.score(objects) == pytest.approx(2 / 3)
    objects[g] = token_state((M.PAD_X, 0.0, M.TRAVEL_Z - M.GRIP_BELOW_TOP))  # still in the gripper over the pad
    assert TASK.score(objects) == pytest.approx(2 / 3)
    r = next(n for n in REDS if M.on_pad(objects[n], 1))
    objects[g] = token_state((M.PAD_X, M.BAY_Y[1], M.CARD_TOP + 1.5 * M.TOKEN_H))  # stacked on bay 1's red token
    assert M.on_pad(objects[r], 1) and TASK.score(objects) == pytest.approx(2 / 3)


def test_a_token_lying_on_its_side_on_the_right_pad_counts():
    objects = goal_state()
    g = next(n for n in GREENS if M.on_pad(objects[n], 2))
    objects[g] = state((M.PAD_X, 0.0, M.CARD_TOP + M.TOKEN_R), TOKEN_HALF, tilt_deg=90.0)
    assert TASK.score(objects) == 1.0


def test_a_token_off_the_pad_edge_or_on_the_bay_card_does_not_count():
    objects = goal_state()
    g = next(n for n in GREENS if M.on_pad(objects[n], 2))
    objects[g] = token_state(on_pad_pos(2, dx=M.PAD / 2 + 0.005))
    assert TASK.score(objects) == pytest.approx(2 / 3)
    objects[g] = token_state((M.BAY_X + 0.045, 0.045, M.CARD_TOP + M.TOKEN_H / 2))  # on bay 2's card itself
    assert TASK.score(objects) == pytest.approx(2 / 3)


def test_a_token_knocked_off_the_table_does_not_matter():
    objects = goal_state()
    objects["token_g3"] = token_state((0.3, 0.75, M.TOKEN_H / 2))  # a spare token
    assert TASK.score(objects) == 1.0 and TASK.success(objects)


def test_every_combination_of_flags_scores_on_a_third_grid():
    for combo in itertools.product((None, "green", "red", "both"), repeat=3):
        objects = flagged({b: (["green", "red"] if c == "both" else c) for b, c in zip(M.BAYS, combo, strict=True)})
        s = TASK.score(objects)
        assert any(s == pytest.approx(v) for v in SCORES), (combo, s)
        assert TASK.success(objects) == (s == 1.0)


# -- 4. construct validity --------------------------------------------------------------------------------


@pytest.mark.parametrize("colour", ["green", "red"])
def test_flagging_every_bay_the_same_colour_scores_zero(colour):
    assert TASK.score(flagged({b: colour for b in M.BAYS})) == 0.0


def test_hedging_with_both_colours_on_every_pad_scores_zero():
    objects = start_state()
    for b, (g, r) in zip(M.BAYS, zip(GREENS, REDS, strict=True), strict=True):
        flag(objects, g, b, dy=-0.025)
        flag(objects, r, b, dy=0.025)
    assert TASK.score(objects) == 0.0


def test_the_opposite_verdicts_score_zero():
    flip = {"green": "red", "red": "green"}
    assert TASK.score(flagged({b: flip[c] for b, c in M.VERDICT.items()})) == 0.0


def test_putting_the_nearest_token_on_each_pad_scores_zero():
    def nearest(bay, taken):
        return min((n for n in M.TOKEN_NAMES if n not in taken),
                   key=lambda n: abs(next(o.pos[1] for o in M.TOKENS if o.name == n) - M.BAY_Y[bay]))

    objects, taken = start_state(), set()
    for b in M.BAYS:
        n = nearest(b, taken)
        taken.add(n)
        flag(objects, n, b)
    assert TASK.score(objects) == 0.0


def test_random_flags_average_well_below_the_true_verdicts():
    scores = [TASK.score(flagged(dict(zip(M.BAYS, c, strict=True))))
              for c in itertools.product(("green", "red"), repeat=3)]
    assert np.mean(scores) <= 0.2 and max(scores) == 1.0  # 1 of the 8 colourings is the truth


@pytest.mark.parametrize("how", ["fixed_onto_the_cross", "pushed_aside", "lifted", "tipped", "off_the_table"])
def test_moving_any_cube_scores_zero_even_with_matching_flags(how):
    objects = goal_state()
    x, y, z = M.CUBE_START[1]
    if how == "fixed_onto_the_cross":  # the failed attempt 'repaired', then flagged green as a success
        objects["cube1"] = cube_state((M.BAY_X, M.BAY_Y[1], z))
        objects = {**objects, **{k: v for k, v in flagged({1: "green", 2: "green", 3: "red"}).items()
                                 if k.startswith("token")}}
    elif how == "pushed_aside":
        objects["cube1"] = cube_state((x + 0.008, y + 0.008, z))  # 11 mm
    elif how == "lifted":
        objects["cube2"] = cube_state((M.BAY_X, 0.0, z + 0.05), yaw=M.CUBE_PLACEMENT[2][1])
    elif how == "tipped":
        objects["cube3"] = cube_state(M.CUBE_START[3], tilt_deg=15.0)
    else:
        objects["cube2"] = cube_state((0.3, 0.75, M.CUBE / 2))
    assert TASK.score(objects) == 0.0 and not TASK.success(objects)


def test_a_cube_nudged_under_a_centimetre_or_spun_in_place_is_still_untouched():
    objects = goal_state()
    x, y, z = M.CUBE_START[1]
    objects["cube1"] = cube_state((x + 0.006, y - 0.006, z))  # 8.5 mm
    objects["cube3"] = cube_state(M.CUBE_START[3], yaw=0.4)
    assert TASK.score(objects) == 1.0


def test_the_failures_are_unambiguous_and_the_success_is_not_square_on():
    for b in M.BAYS:
        (dx, dy), yaw = M.CUBE_PLACEMENT[b]
        if M.VERDICT[b] == "red":
            assert math.hypot(dx, dy) >= 0.03 * 2  # over 6 cm from the cross centre
            # the cube stands wholly on a grey corner, at least 1 cm from either arm of the cross
            assert min(abs(dx), abs(dy)) - M.CUBE / 2 - M.STRIP / 2 >= 0.010 - 1e-9
            # and wholly on the card
            assert max(abs(dx), abs(dy)) + M.CUBE / 2 <= M.BAY / 2 - 0.005
        else:
            assert math.hypot(dx, dy) <= M.COVER and abs(math.degrees(yaw)) >= 20


def test_neither_names_nor_token_positions_give_the_verdicts_away():
    for n in TASK.names:
        assert "fail" not in n and "success" not in n and "ok" not in n.split("_")
    assert {M.TOKEN_COLOR[n] for n in M.TOKEN_NAMES if next(o.pos[1] for o in M.TOKENS if o.name == n) > 0} \
        == {"green", "red"}
    assert {M.TOKEN_COLOR[n] for n in M.TOKEN_NAMES if next(o.pos[1] for o in M.TOKENS if o.name == n) < 0} \
        == {"green", "red"}
    cubes = [o for o in M.OBJECTS if o.name.startswith("cube")]
    assert len({o.color for o in cubes}) == 1 and len({o.size for o in cubes}) == 1
    pads = [o for o in M.OBJECTS if o.name.startswith("pad")]
    assert len({(o.color, o.size, o.pos[0]) for o in pads}) == 1


def test_the_instruction_defines_success_and_names_the_token_colours():
    words = META["instruction"].lower()
    for w in ("green", "red", "centre of the cross", "pad", "cube"):
        assert w in words
    for w in ("bay 1", "bay 2", "bay 3", "left", "right", "middle"):
        assert w not in words


# -- 5. start layout and gripper --------------------------------------------------------------------------


def test_no_two_objects_overlap_at_the_start():
    for a, b in itertools.combinations(M.OBJECTS, 2):
        (alo, ahi), (blo, bhi) = aabb(a), aabb(b)
        overlap = np.minimum(ahi, bhi) - np.maximum(alo, blo)
        assert overlap.min() <= 1e-9, (a.name, b.name)


def test_each_bay_card_tiles_its_16_cm_square_exactly():
    for b in M.BAYS:
        tiles = [o for o in M.OBJECTS if o.name.startswith(f"bay{b}_")]
        assert sum(o.size[0] * o.size[1] for o in tiles) == pytest.approx(M.BAY**2)
        lo = np.min([aabb(o)[0] for o in tiles], axis=0)
        hi = np.max([aabb(o)[1] for o in tiles], axis=0)
        assert hi[:2] - lo[:2] == pytest.approx([M.BAY, M.BAY])
        assert {round(o.pos[2] + o.size[2] / 2, 9) for o in tiles} == {round(M.CARD_TOP, 9)}  # flush tops


def test_loose_objects_rest_on_a_support():
    for o in M.CUBES:
        lo, hi = aabb(o)
        assert lo[2] - M.CARD_TOP == pytest.approx(M.SPAWN_GAP)
        b = int(o.name[-1])
        assert abs(o.pos[0] - M.BAY_X) + (hi[0] - lo[0]) / 2 <= M.BAY / 2
        assert abs(o.pos[1] - M.BAY_Y[b]) + (hi[1] - lo[1]) / 2 <= M.BAY / 2
    for o in M.TOKENS:
        assert aabb(o)[0][2] - M.TOP == pytest.approx(M.SPAWN_GAP)


def test_cards_and_tokens_are_apart():
    for b in M.BAYS:
        assert M.PAD_X + M.PAD / 2 + 0.015 <= M.BAY_X - M.BAY / 2  # pad and card do not touch
    for b1, b2 in zip(M.BAYS, M.BAYS[1:]):
        assert abs(M.BAY_Y[b1] - M.BAY_Y[b2]) - M.BAY >= 0.03


def test_every_token_fits_the_gripper_with_a_centimetre_to_spare():
    assert M.GRIPPER_MAX_OPENING == pytest.approx(-2 * FINGER_OPEN, abs=1e-3)  # 94 mm pad gap
    assert 2 * M.TOKEN_R <= M.GRIPPER_MAX_OPENING - 0.010
    assert M.TOKEN_H / (2 * M.TOKEN_R) <= 1.3  # squat
    assert math.isclose(M.TOKEN_MASS, 0.030) and math.isclose(M.CUBE_MASS, 0.075)


def test_the_task_json_has_every_field_and_no_media():
    for k in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
              "fidelity", "scores"):
        assert META.get(k), k
    # scores is filled after the Dream-1 run and media by the upload script
    assert META["scores"] == "not yet run" or META["scores"].startswith("Dream-1 run: "), META["scores"]
    assert META["category"] == "Impact on AI timelines"
