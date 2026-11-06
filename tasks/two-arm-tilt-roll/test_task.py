"""Offline checks for two-arm-tilt-roll (NumPy and pytest only; no Isaac Sim).

Run from this folder with the yam-isaac-lab-arena package importable (installed, as on the Brev VM, or a clone
named by YAM_ARENA):

    YAM_ARENA=~/yam-isaac-lab-arena python -m pytest -q test_task.py
"""

import json
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET
from functools import lru_cache
from pathlib import Path

import numpy as np
import pytest

if os.environ.get("YAM_ARENA"):
    sys.path.insert(0, os.path.expanduser(os.environ["YAM_ARENA"]))

import yam_arena  # noqa: E402
from yam_arena import ik, station  # noqa: E402
from yam_arena.constants import ARM_JOINTS, FINGER_OPEN, GRIPPER, GRIPPY_FRICTION, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import Box, ObjectState, Sphere, load  # noqa: E402
from yam_arena.urdf import forward_kinematics, quat_xyzw_to_matrix, rpy_matrix  # noqa: E402

HERE = Path(__file__).resolve().parent
TASK = load(HERE / "task.py")
M = TASK.module
TOP = TASK.table_top
HZ = 20.0  # the rate the README's runs use
GRIPPER_MAX = 2 * abs(FINGER_OPEN)  # pads 94 mm apart fully open
MARGIN = 0.01
ARM_URDF = Path(yam_arena.__file__).resolve().parent / "assets" / "yam" / "yam_arm.urdf"  # links with collision meshes
FIXED = [o for o in TASK.objects if not o.free]


# -- helpers -----------------------------------------------------------------------------------------------------------


def quat_x(angle):
    """XYZW quaternion of a rotation about world x (positive: the board's left end, +y, goes up)."""
    return np.array([np.sin(angle / 2), 0.0, 0.0, np.cos(angle / 2)])


def state(pos, half, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos, half = np.asarray(pos, float), np.asarray(half, float)
    return ObjectState(pos, np.asarray(quat, float), np.asarray(vel, float), np.zeros(3), pos - half, pos + half)


def board_on_line(y1, top1, y2, top2):
    """The board held so its top face passes through (y1, top1) and (y2, top2), heights above the table."""
    phi = np.arctan((top1 - top2) / (y1 - y2))
    yc = M.BOARD.pos[1]
    top_c = top2 + (top1 - top2) * (yc - y2) / (y1 - y2)
    zc = TOP + top_c - np.cos(phi) * M.BOARD_T / 2
    half = (
        M.BOARD_W / 2,
        abs(np.cos(phi)) * M.BOARD_LEN / 2 + abs(np.sin(phi)) * M.BOARD_T / 2,
        abs(np.sin(phi)) * M.BOARD_LEN / 2 + abs(np.cos(phi)) * M.BOARD_T / 2,
    )
    return state((M.X_BOARD, yc, zc), half, quat_x(phi)), phi


def board_flat():
    return state((M.X_BOARD, M.BOARD.pos[1], TOP + M.BOARD_T / 2), np.asarray(M.BOARD.size) / 2)


def board_pivoting_on(end_y, grip_y, grip_top):
    """One end's bottom edge on the table, the board's top face at grip_top above the table at grip_y."""
    # top face at the resting end is T / cos(phi) above the table; solve by fixed-point iteration
    top_end = M.BOARD_T
    for _ in range(20):
        board, phi = board_on_line(end_y, top_end, grip_y, grip_top)
        top_end = M.BOARD_T / np.cos(phi)
    return board, phi


def ball_on(board, y, vel=(0.0, 0.0, 0.0)):
    """The ball resting on the board's top face at world y (on the centre line)."""
    rot = quat_xyzw_to_matrix(board.quat_xyzw)
    local_y = (y - board.pos[1]) / rot[1, 1]
    centre = board.pos + rot @ np.array([0.0, local_y, M.BOARD_T / 2 + M.BALL_R])
    return state(centre, [M.BALL_R] * 3, vel=vel)


def ball_at(x, y, bottom, vel=(0.0, 0.0, 0.0)):
    return state((x, y, bottom + M.BALL_R), [M.BALL_R] * 3, vel=vel)


def scene(ball, board):
    objs = {"board": board, "ball": ball}
    for o in FIXED:
        objs[o.name] = state(o.pos, np.asarray(o.size) / 2)
    return objs


def tilt_pose():
    return board_on_line(M.Y_LEFT_GRIP, M.TILT_LEFT, M.Y_RIGHT_GRIP, M.LIFT_RIGHT)[0]


def lift_pose():
    return board_on_line(M.Y_LEFT_GRIP, M.LIFT_LEFT, M.Y_RIGHT_GRIP, M.LIFT_RIGHT)[0]


def bbox(obj):
    if isinstance(obj, Box):
        half = np.asarray(obj.size) / 2
    elif isinstance(obj, Sphere):
        half = np.full(3, obj.radius)
    else:
        raise TypeError(obj)
    assert obj.yaw == 0.0
    return np.asarray(obj.pos) - half, np.asarray(obj.pos) + half


def _stl(path):
    data = path.read_bytes()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        return np.array(re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", data), float)
    n = struct.unpack("<I", data[80:84])[0]
    rec = np.frombuffer(data[84 : 84 + 50 * n], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return np.unique(rec["v"].reshape(-1, 3).astype(float), axis=0)


@lru_cache(maxsize=1)
def collision_points():
    """Every link's collision-mesh vertices in its own frame, from yam_arm.urdf."""
    root = ET.parse(ARM_URDF).getroot()
    out = {}
    for link in root.findall("link"):
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array(o.get("xyz").split(), float)
            rpy = np.array(o.get("rpy").split(), float)
            mesh = ARM_URDF.parent / col.find("geometry/mesh").get("filename")
            pts.append(_stl(mesh) @ rpy_matrix(rpy).T + xyz)
        if pts:
            out[link.get("name")] = np.vstack(pts)
    return out


def links_in_world(q6, finger, base):
    joints = {n: float(v) for n, v in zip(ARM_JOINTS, q6, strict=True)}
    joints["joint7"] = joints["joint8"] = float(finger)
    fk = forward_kinematics(ARM_URDF, joints)
    return {name: p @ fk[name][:3, :3].T + fk[name][:3, 3] + base for name, p in collision_points().items()}


@lru_cache(maxsize=1)
def finger_profile(finger=-M.BOARD_W / 2):
    """Finger and gripper geometry around the TCP for a straight-down grip at yaw 0 (fingers close along world y),
    with the pads ``-finger`` from the centre: vertices relative to the TCP, in metres."""
    base = np.zeros(3)
    target = np.array([0.30, 0.0, 0.03])
    q, pe, re_ = ik.solve(target, ik.matrix_to_quat_xyzw(grip_rotation(0.0)), np.array([0, 1, 1, -0.6, 0, 0.0]),
                          base_pos=base, iters=400)
    assert pe < 1e-4 and re_ < 1e-3
    world = links_in_world(q, finger, base)
    return {k: v - target for k, v in world.items()}


def band(points, z0, z1):
    return points[(points[:, 2] >= z0) & (points[:, 2] < z1)]


# -- 1. the file loads ------------------------------------------------------------------------------------------------


def test_the_task_file_loads_with_its_objects_and_hooks():
    assert TASK.scene == "station" and TASK.score_over == "end"
    assert TASK.free_objects == ("board", "ball")
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    assert set(TASK.demo()) == {"left", "right"}  # a two-arm demo


# -- 2. the demo is reachable -----------------------------------------------------------------------------------------


@pytest.mark.parametrize("plan_fn", ["demo", "one_arm_demo"])
def test_every_arm_plan_is_reachable_within_the_joint_limits_at_20_hz(plan_fn):
    plan = getattr(M, plan_fn)()
    for side, waypoints in plan.items():
        arm = ArmPlan(waypoints, station.BASES[side])
        steps = round(arm.duration * HZ) + 5
        actions = np.array([arm.action7((k + 1) / HZ)[0] for k in range(steps)])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, (side, arm.worst_ik_mm, arm.worst_ik_deg)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER), side
        assert arm.duration * HZ < 0.5 * 1200  # under half the 1,200-step run limit


def test_both_arms_follow_the_same_clock_and_end_with_a_still_hold():
    plan = TASK.demo()
    left, right = plan["left"], plan["right"]
    assert [w.duration for w in left] == [w.duration for w in right]  # every phase starts together
    for pts in (left, right):
        assert pts[-1].pos == pts[-2].pos and pts[-1].finger == pts[-2].finger
        assert pts[-1].duration >= 1.5 + 2.0  # 2 s for the ball to drop and settle, then the 1.5 s still hold
        assert all(w.yaw == M.GRIP_YAW for w in pts)


def test_the_grips_sit_in_the_open_gaps_and_the_pads_clear_the_table():
    tips = M.GRIP_Z - PAD_TIP_PAST_TCP
    assert tips - TOP >= 0.003 - 1e-9  # pad tips at least 3 mm above the table when gripping
    assert M.GRIP_Z - PAD_TIP_PAST_TCP < TOP + M.BOARD_T - 0.01  # at least 1 cm of pad on the board's side
    prof = finger_profile()
    half_along_board = max(np.abs(prof[k][:, 0]).max() for k in ("tip_left", "tip_right", "gripper"))
    assert M.GRIP_HALF == pytest.approx(half_along_board, abs=0.001)  # 34 mm, from the meshes
    # each gripper fits its gap with 3 cm to spare: the left one beyond the slot, the right one between slot and rails
    assert M.Y_LEFT_GRIP - M.GRIP_HALF - M.SLOT_Y[1] >= 0.03 - 1e-9
    assert M.Y_LEFT_GRIP + 0.03 <= M.Y_BOARD_LEFT  # and 3.5 cm in from the board's left end
    assert M.Y_RIGHT_GRIP - M.GRIP_HALF - M.SLOT_Y[0] <= -0.03 + 1e-9
    assert M.Y_RIGHT_GRIP - M.GRIP_HALF - M.RAIL_Y[1] >= 0.03 - 1e-9
    # the ball starts clear of the right gripper's fingers and body (8.3 cm from its centre) with 2.5 cm to spare
    assert M.Y_RIGHT_GRIP - M.Y_BALL0 >= 0.083 + 0.025 - 1e-9


def test_the_lift_parks_the_ball_against_the_right_hand_and_the_tilt_sends_it_down_toward_the_cup():
    span = M.Y_LEFT_GRIP - M.Y_RIGHT_GRIP
    lift_slope = (M.LIFT_LEFT - M.LIFT_RIGHT) / span  # + : the board rises to the left
    tilt_slope = (M.TILT_LEFT - M.LIFT_RIGHT) / span
    assert lift_slope < -0.005  # lifted: the right part slopes up toward the cup, the ball rolls back to the hand
    assert 0.02 <= tilt_slope <= 0.04  # tipped: 2-4 % down toward the cup (1.1-2.3 deg)
    # where the ball parks (its centre 8.3 cm from the right grip) the rails still hold it
    parked = M.Y_RIGHT_GRIP - 0.083
    assert M.RAIL_Y[0] + M.BALL_R < parked < M.RAIL_Y[1]
    # the board's right end stays at least 2.5 cm above the cup's rim from the end of the lift to the end of the plan
    for left, right in ((M.LIFT_LEFT, M.LIFT_RIGHT), (M.TILT_LEFT, M.LIFT_RIGHT)):
        bottom = M.board_top_at(M.Y_BOARD_RIGHT, left, right) - M.BOARD_T
        assert bottom >= M.CUP_H + 0.025, (left, right, bottom)


def test_the_slot_and_rails_stay_above_the_board_while_it_is_held():
    plan = TASK.demo()
    left = ArmPlan(plan["left"], station.BASES["left"])
    right = ArmPlan(plan["right"], station.BASES["right"])
    labels = [w.label for w in plan["left"]]
    held_from = sum(w.duration for w in plan["left"][: labels.index("grip the board") + 1])
    for k in range(round(held_from * HZ), round(left.duration * HZ) + 1):
        t = k / HZ
        zl, zr = left.target(t)[0][2] - TOP, right.target(t)[0][2] - TOP  # TCP = board top face at each grip
        for y in M.SLOT_Y:
            assert M.board_top_at(y, zl, zr) <= M.SLOT_H - 0.015, (t, y)
        for y in M.RAIL_Y:
            assert M.board_top_at(y, zl, zr) <= M.RAIL_H - 0.015, (t, y)  # the ball stays between the rails


def test_a_ball_rolling_off_the_tipped_board_clears_the_cups_near_wall():
    """Slow roll over the board's end: a rolling sphere leaves the edge when cos(theta) = 10/17. Allow 1.5 cm of
    arm sag under load (earlier simulation runs measured 0.5-1 cm with 0.37 kg in the gripper)."""
    edge_top = M.board_top_at(M.Y_BOARD_RIGHT, M.TILT_LEFT, M.LIFT_RIGHT) - 0.015
    theta = np.arccos(10 / 17)
    ahead, centre_z = M.BALL_R * np.sin(theta), edge_top + M.BALL_R * np.cos(theta)
    wall_inner = M.CUP_GAP + M.CUP_WALL  # distance past the board's end
    dx = ahead - wall_inner
    assert dx > 0  # the ball's centre is already past the wall: it falls into the cup, not back
    lowest_over_wall = centre_z - np.sqrt(M.BALL_R**2 - dx**2)
    assert lowest_over_wall >= M.CUP_H + 0.005, lowest_over_wall


def test_the_arms_keep_clear_of_the_slot_rails_and_cup_through_the_demo():
    """Every arm link's URDF collision mesh stays at least 1.5 cm from every fixed box (sampled every 0.25 s)."""
    for side, waypoints in TASK.demo().items():
        arm = ArmPlan(waypoints, station.BASES[side])
        base = np.asarray(station.BASES[side])
        worst = np.inf
        for k in range(round(arm.duration * HZ)):
            action, _ = arm.action7((k + 1) / HZ)
            if k % 5:
                continue
            for name, pts in links_in_world(action[:6], FINGER_OPEN * action[6], base).items():
                if name == "base":
                    continue
                for o in FIXED:
                    gap = np.max(np.abs(pts - np.asarray(o.pos)) - np.asarray(o.size) / 2, axis=1).min()
                    worst = min(worst, gap)
        assert worst >= 0.015, (side, worst)


# -- 3. scoring: the goal and every partial-credit branch ------------------------------------------------------------


def test_a_ball_resting_in_the_cup_succeeds_and_scores_one():
    cx, cy = M.CUP_XY
    for board in (tilt_pose(), board_flat()):  # still held, or already set back down
        objs = scene(ball_at(cx, cy, TOP), board)
        assert TASK.success(objs) and TASK.score(objs) == 1.0
    assert TASK.distance(scene(ball_at(cx, cy, TOP), tilt_pose())) == pytest.approx(0.0, abs=1e-9)
    corner = scene(ball_at(cx - 0.02, cy + 0.02, TOP), tilt_pose())  # rolled into a corner
    assert TASK.success(corner) and TASK.score(corner) == 1.0


def test_a_ball_still_rolling_in_the_cup_scores_0_8():
    objs = scene(ball_at(*M.CUP_XY, TOP, vel=(0.0, -0.1, 0.0)), tilt_pose())
    assert not TASK.success(objs) and TASK.score(objs) == 0.8


def test_a_ball_on_the_lifted_board_scores_0_3_plus_its_progress_toward_the_cup():
    board = tilt_pose()
    parked = scene(ball_on(board, M.Y_RIGHT_GRIP - 0.083), board)
    assert TASK.score(parked) == pytest.approx(0.3)  # farther from the cup than at the start: no progress yet
    start = scene(ball_on(lift_pose(), M.Y_BALL0), lift_pose())
    assert TASK.score(start) == pytest.approx(0.3, abs=0.005)
    at_end = scene(ball_on(board, M.Y_BOARD_RIGHT + 0.01), board)
    d = np.hypot(0.0, M.Y_BOARD_RIGHT + 0.01 - M.CUP_XY[1])
    assert TASK.score(at_end) == pytest.approx(0.3 + 0.3 * (1 - d / M.BALL_START_DIST), abs=0.005)
    assert 0.4 < TASK.score(at_end) < 0.6
    for objs in (parked, start, at_end):
        assert not TASK.success(objs)


# -- 4. construct validity: one-arm strategies and other shortcuts score 0 -------------------------------------------


def one_arm_right():
    """The right arm alone lifts its gap 10.5 cm: the board pivots on its left end; the ball rolls back to the hand."""
    board, _ = board_pivoting_on(M.Y_BOARD_LEFT, M.Y_RIGHT_GRIP, M.LIFT_RIGHT)
    return scene(ball_on(board, M.Y_RIGHT_GRIP - 0.083), board)


def one_arm_left():
    """The left arm alone lifts its end 14 cm: the board pivots on its right end; the ball rolls down against the
    cup's near wall and stops there, still on the board's end."""
    board, _ = board_pivoting_on(M.Y_BOARD_RIGHT, M.Y_LEFT_GRIP, 0.14)
    return scene(ball_on(board, M.Y_CUP_NEAR + M.BALL_R + 0.001), board)


@pytest.mark.parametrize(
    "where, objs",
    [
        ("untouched start", scene(ball_on(board_flat(), M.Y_BALL0), board_flat())),
        ("right arm alone lifts its gap", one_arm_right()),
        ("left arm alone lifts its end", one_arm_left()),
        ("ball rolled to the board's right end on the flat board", scene(ball_on(board_flat(), M.Y_BOARD_RIGHT + 0.04),
                                                                          board_flat())),
        ("ball dropped off the end onto the table", scene(ball_at(M.X_BOARD, M.Y_CUP_NEAR + 0.05, TOP), board_flat())),
        ("ball lying on the cup's near wall", scene(ball_at(M.X_BOARD, M.Y_CUP_NEAR - 0.004, TOP + M.CUP_H),
                                                    tilt_pose())),
        ("ball on top of the guide slot", scene(ball_at(M.X_BOARD - 0.05, 0.0, TOP + M.SLOT_H), tilt_pose())),
        ("ball above the cup, still falling", scene(ball_at(*M.CUP_XY, TOP + 0.03, vel=(0, 0, -0.5)), tilt_pose())),
        ("ball beside the cup on the table", scene(ball_at(M.X_BOARD + 0.15, M.CUP_XY[1], TOP), tilt_pose())),
        ("ball off the table", scene(ball_at(M.X_BOARD, -0.70, TOP - 0.30), tilt_pose())),
        ("board dropped off the table, ball in the cup", scene(ball_at(*M.CUP_XY, TOP),
                                                               state((0.3, 0.75, 0.2), (0.04, 0.42, 0.01)))),
    ],
)
def test_one_arm_outcomes_and_other_shortcuts_score_zero(where, objs):
    assert not TASK.success(objs), where
    assert TASK.score(objs) == 0.0, where


def test_the_ball_cannot_be_picked_up_or_pushed_into_the_cup():
    assert 2 * M.BALL_R > GRIPPER_MAX + 0.005  # 100 mm ball, 94 mm between the open pads
    # lying on the flat board or on the table, the ball's centre is below the cup's rim: pushing it sideways into
    # the wall cannot lift it over (the contact is above its equator)
    assert M.BOARD_T + M.BALL_R <= M.CUP_H - 0.005
    assert M.CUP_H > M.BALL_R


@pytest.mark.parametrize("lift", [0.02, 0.06, 0.105, 0.15, 0.20])
def test_with_the_left_end_on_the_table_the_balls_side_of_the_board_slopes_back_toward_the_right_hand(lift):
    """Right arm alone: the board pivots on its left end. Its right part then rises past the grip, so the ball rolls
    to the hand, never toward the cup."""
    board, phi = board_pivoting_on(M.Y_BOARD_LEFT, M.Y_RIGHT_GRIP, lift)
    assert phi < 0  # left end low
    slope = np.tan(phi)
    top_grip, top_end = lift, lift - slope * (M.Y_RIGHT_GRIP - M.Y_BOARD_RIGHT)
    assert top_end > top_grip


@pytest.mark.parametrize("lift", [0.06, 0.14, 0.20, 0.30])
def test_with_the_right_end_on_the_table_the_ball_reaches_the_cup_wall_below_its_rim(lift):
    """Left arm alone: the board pivots on its right end, so the ball's centre at that end is at most one board
    thickness plus one radius above the table, below the rim, and it rolls off downhill (it cannot rise)."""
    board, phi = board_pivoting_on(M.Y_BOARD_RIGHT, M.Y_LEFT_GRIP, lift)
    assert phi > 0
    # top face at the right end, measured at the board's end face: within T / cos(phi) of the table
    assert M.BOARD_T / np.cos(phi) + M.BALL_R <= M.CUP_H - 0.005


def test_no_single_gripper_can_take_the_board_inside_the_slot_or_the_rails():
    """Inside the slot and the rails the board can only be pinched if a finger fits between the board and the wall.
    Measured on the URDF collision meshes, with the pads on the board's sides (80 mm apart)."""
    prof = finger_profile()
    tip = prof["tip_left"] if prof["tip_left"][:, 1].mean() > 0 else prof["tip_right"]
    pad_face = band(tip, -PAD_TIP_PAST_TCP, 0.0)[:, 1].min()
    assert pad_face == pytest.approx(M.BOARD_W / 2, abs=0.002)
    thick_low = band(tip, 0.0, 0.040)[:, 1].max() - pad_face  # just above the pad: about 10 mm
    thick_high = band(tip, 0.040, 0.060)[:, 1].max() - pad_face  # higher up: about 21 mm
    assert thick_low >= M.SLOT_GAP + 0.003, thick_low
    assert thick_high >= M.RAIL_GAP + 0.005, thick_high
    # those finger bands are below the slot's and rails' tops whenever the pads can reach the board on the table
    assert M.SLOT_H > M.BOARD_T + 0.010 + 0.040 and M.RAIL_H > M.BOARD_T + 0.010 + 0.060
    # nor can a gripper close around the board and a wall together
    assert M.BOARD_W + 2 * (M.SLOT_GAP + M.SLOT_T) > GRIPPER_MAX + MARGIN
    assert M.BOARD_W + 2 * (M.RAIL_GAP + M.RAIL_T) > GRIPPER_MAX + MARGIN


def test_one_gripper_cannot_hold_the_board_off_the_table_by_either_gap():
    """Statics. Held by one gap alone, the board's weight makes a moment about the fingers' closing axis that the
    two pads must resist by friction alone. Upper bound on what they hold: 2 pads x static friction 1.2 x the
    gripper's 50 N force limit x the 10.2 mm corner radius of a pad's contact patch (15.6 mm along the board,
    13 mm of bite). The board's moment is at least 1.4 times that."""
    prof = finger_profile()
    tip = prof["tip_left"]
    pad = band(tip, -PAD_TIP_PAST_TCP, 0.0)
    corner = np.hypot(np.abs(pad[:, 0]).max(), PAD_TIP_PAST_TCP / 2)
    hold = 2 * GRIPPY_FRICTION[0] * GRIPPER["effort"] * corner
    assert hold == pytest.approx(1.22, abs=0.05)
    g, yc = 9.81, M.BOARD.pos[1]
    ball_moment = M.BALL.mass * g * (M.Y_RIGHT_GRIP - M.Y_BALL0)  # the ball sits on the other side of the right grip
    right = M.BOARD_MASS * g * abs(M.Y_RIGHT_GRIP - yc) - ball_moment
    left = M.BOARD_MASS * g * abs(M.Y_LEFT_GRIP - yc) + M.BALL.mass * g * (M.Y_LEFT_GRIP - M.Y_BALL0)
    assert right >= 1.4 * hold, (right, hold)
    assert left >= 2.5 * hold, (left, hold)


# -- 5. layout: no overlaps, loose objects supported, grasped object fits, everything on the table and in view -------


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
    assert blo[2] - TOP == pytest.approx(M.REST_GAP)
    assert M.BALL.pos[2] - M.BALL_R - bhi[2] == pytest.approx(M.REST_GAP)
    assert blo[1] + 0.05 < M.BALL.pos[1] < bhi[1] - 0.04  # the ball's contact point is on the board, 4 cm from its end
    for o in TASK.objects:
        lo, hi = bbox(o)
        assert M.TABLE_X[0] < lo[0] and hi[0] < M.TABLE_X[1] and M.TABLE_Y[0] < lo[1] and hi[1] < M.TABLE_Y[1], o.name
        assert lo[2] >= TOP - 1e-9, o.name
    # the board starts centred in the slot and between the rails, and its right end short of the cup's near wall
    assert M.SLOT_GAP > 0 and M.RAIL_GAP > 0 and M.Y_BOARD_RIGHT - M.Y_CUP_NEAR == pytest.approx(M.CUP_GAP)
    # the ball (100 mm) fits between the rails (104 mm) and inside the cup (140 mm)
    assert M.BOARD_W + 2 * M.RAIL_GAP > 2 * M.BALL_R and M.CUP_INNER >= 2 * M.BALL_R + 0.03 - 1e-9


def test_the_board_fits_the_open_gripper_with_a_centimetre_to_spare():
    assert GRIPPER_MAX == pytest.approx(0.0939, abs=1e-3)
    assert M.BOARD_W <= GRIPPER_MAX - MARGIN  # gripped across its 80 mm width (fingers along world x at yaw 90 deg)
    assert M.GRIP_YAW == pytest.approx(np.pi / 2)


def _project(cam, p):
    rot = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    pc = rot.T @ (np.asarray(p) - np.asarray(cam.pos))  # OpenGL camera: looks along -z, y up
    lens = cam.lens
    return lens.cx + lens.fx * pc[0] / -pc[2], lens.cy - lens.fy * pc[1] / -pc[2], -pc[2]


def _blocked(a, b, lo, hi):
    """Does the segment a -> b pass through the box [lo, hi]? (slab test)"""
    d = b - a
    t0, t1 = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-12:
            if not lo[i] <= a[i] <= hi[i]:
                return False
            continue
        ta, tb = sorted(((lo[i] - a[i]) / d[i], (hi[i] - a[i]) / d[i]))
        t0, t1 = max(t0, ta), min(t1, tb)
        if t0 > t1:
            return False
    return t1 > 1e-6 and t0 < 1 - 1e-6


@pytest.mark.parametrize(
    "what, point",
    [
        ("top of the ball at the start", (M.X_BOARD, M.Y_BALL0, TOP + M.BOARD_T + 2 * M.BALL_R)),
        ("centre of the ball at the start", (M.X_BOARD, M.Y_BALL0, TOP + M.BOARD_T + M.BALL_R + 0.02)),
        ("top of a ball in the cup", (*M.CUP_XY, TOP + 2 * M.BALL_R)),
        ("the cup's floor centre", (*M.CUP_XY, TOP + 0.005)),
    ],
)
def test_the_top_camera_sees_the_ball_and_the_cup(what, point):
    cam = station.top_camera()
    u, v, depth = _project(cam, point)
    assert depth > 0 and 10 <= u <= cam.lens.width - 10 and 10 <= v <= cam.lens.height - 10, (what, u, v)
    a, b = np.asarray(cam.pos, float), np.asarray(point, float)
    for o in FIXED:
        lo, hi = bbox(o)
        assert not _blocked(a, b, lo, hi), (what, o.name)


# -- task.json --------------------------------------------------------------------------------------------------------


def test_task_json_has_every_field_and_names_no_one():
    meta = json.loads((HERE / "task.json").read_text())
    for key in ("title", "category", "keywords", "instruction", "context", "setup", "scoring", "run_limit",
                "fidelity", "scores"):
        assert meta.get(key), key
    assert meta.get("media", "M-").startswith("M-")  # media is written by the upload script
    assert meta["category"] in {"Impact on jobs", "Impact on AI timelines", "Safety and alignment",
                                "Societal relevance", "Other"}
    text = json.dumps(meta).lower() + (HERE / "README.md").read_text().lower() + TASK.docs.lower()
    for word in ("robocurve_sprint", "knowledge base", "kairos"):
        assert word not in text, word
