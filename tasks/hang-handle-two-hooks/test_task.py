"""Offline checks for hang-handle-two-hooks (NumPy and pytest only, no Isaac Sim).

    YAM_ARENA_DIR=/path/to/yam-isaac-lab-arena python -m pytest test_task.py -q

YAM_ARENA_DIR is a checkout of the arena (tested at bd62771); it is not needed when yam_arena is installed.
"""

import itertools
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

from yam_arena import YAM_URDF, ik, station  # noqa: E402
from yam_arena.analysis import box_signed_distance, link_hulls, world_hulls  # noqa: E402
from yam_arena.constants import FINGER_OPEN, READY  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics, quat_xyzw_to_matrix  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN (arena tests/test_constants.py)
RUN_LIMIT_STEPS = 2400
JOINT4_TRAVEL_LIMIT = 0.14  # TCP height over the table past which joint4 can run out (arena stack_cubes.py)
Z = np.array([0.0, 0.0, 1.0])
RACK = {b.name: b for b in T.RACK}
Y_IN, Y_OUT = T.HOOK_YS  # inner hook (nearer the midline), outer hook


# -- geometry helpers ------------------------------------------------------------------------------------------------


def rz(yaw: float) -> np.ndarray:
    return axis_angle_matrix(Z, yaw)


def box_state(pos, size, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState as the simulator reports it: world-aligned bounds of the turned box."""
    r = np.eye(3) if r is None else r
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half
    world = corners @ r.T + pos
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def axis_frame(direction, roll_deg=0.0) -> np.ndarray:
    """A rotation whose body x is ``direction`` and whose body z is as near world up as it can be, then rolled
    ``roll_deg`` about body x."""
    x = np.asarray(direction, float)
    x = x / np.linalg.norm(x)
    up = Z if abs(x[2]) < 0.99 else np.array([1.0, 0.0, 0.0])
    y = np.cross(up, x)
    y /= np.linalg.norm(y)
    r = np.c_[x, y, np.cross(x, y)]
    return r @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll_deg))


def handle(pos, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    return box_state(pos, T.BAR_SIZE, r, vel)


def handle_through(p, q, at=0.0) -> ObjectState:
    """A handle whose centre-line runs from ``p`` toward ``q``, centred ``at`` metres from ``p`` along that line."""
    p, q = np.asarray(p, float), np.asarray(q, float)
    d = (q - p) / np.linalg.norm(q - p)
    return handle(p + at * d, axis_frame(d))


def hung(dx=0.0, dy=0.0, yaw_deg=90.0, roll_deg=0.0, dz=0.0, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """The handle along y across both hooks, resting on the hook arms (yaw 90 deg from its start along x)."""
    r = rz(math.radians(yaw_deg)) @ axis_angle_matrix(np.array([1.0, 0.0, 0.0]), math.radians(roll_deg))
    half_h = float(np.abs(r @ np.diag(np.asarray(T.BAR_SIZE) / 2)).sum(axis=1)[2])
    return handle((T.GOAL_X + dx, T.RACK_Y + dy, T.ARM_TOP + half_h + dz), r, vel)


def scene(h=None) -> dict:
    """The fixed rack where it is, and the handle (at its start unless given)."""
    out = {b.name: box_state(b.pos, b.size, rz(b.yaw)) for b in T.RACK}
    out["handle"] = h if h is not None else handle(T.BAR.pos, rz(T.BAR.yaw))
    return out


def obb_gap(c1, r1, s1, c2, r2, s2) -> float:
    """Separating-axis lower bound on the distance between two oriented boxes (m); negative when they overlap."""
    axes = [r1[:, i] for i in range(3)] + [r2[:, i] for i in range(3)]
    axes += [np.cross(r1[:, i], r2[:, j]) for i in range(3) for j in range(3)]
    d = np.asarray(c2, float) - np.asarray(c1, float)
    best = -np.inf
    for a in axes:
        n = np.linalg.norm(a)
        if n < 1e-9:
            continue
        a = a / n
        e1 = sum(abs(float(np.dot(r1[:, i], a))) * s1[i] / 2 for i in range(3))
        e2 = sum(abs(float(np.dot(r2[:, i], a))) * s2[i] / 2 for i in range(3))
        best = max(best, abs(float(np.dot(d, a))) - e1 - e2)
    return float(best)


def rack_gap(c, r, size) -> tuple[float, str]:
    """The smallest gap from a box to any rack part, and that part's name."""
    return min((obb_gap(c, r, size, b.pos, rz(b.yaw), b.size), b.name) for b in T.RACK)


# -- loading and description ---------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert TASK.free_objects == ("handle",) and len(TASK.objects) == 6
    assert all(o.mass is None for o in T.RACK)  # the rack is fixed
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "institute", "team", "lab "):
        assert word not in TASK.docs.lower()


def test_the_handle_is_a_loose_square_bar_that_fits_the_gripper_with_room_to_spare():
    assert T.BAR.size == (0.320, 0.022, 0.022) and T.BAR.mass == pytest.approx(0.100) and T.BAR.yaw == 0.0
    assert T.BAR_W + 0.010 <= GRIPPER_MAX_GAP - 0.010  # 1 cm margin under the 84 mm usable grip
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    approach_gap = -2 * T._finger(T.APPROACH_CLEARANCE)
    assert approach_gap == pytest.approx(0.042) and approach_gap < GRIPPER_MAX_GAP


# -- layout ----------------------------------------------------------------------------------------------------------


def test_the_rack_is_a_board_on_the_table_with_two_j_hooks_against_it():
    board = scene()["back_board"]
    assert board.low[2] == pytest.approx(TOP) and board.low[0] == pytest.approx(T.WALL_X)
    assert board.high[2] - TOP == pytest.approx(0.145)
    for tag, y in T.HOOKS.items():
        arm, lip = scene()[f"hook_{tag}_arm"], scene()[f"hook_{tag}_lip"]
        assert arm.high[0] == pytest.approx(T.WALL_X) and arm.low[0] == pytest.approx(T.LIP_IN_X)  # board to lip
        assert arm.high[2] == pytest.approx(TOP + 0.080) and lip.high[2] == pytest.approx(TOP + 0.095)
        assert lip.high[0] == pytest.approx(T.LIP_IN_X) and lip.low[2] == pytest.approx(arm.low[2])
        assert arm.pos[1] == lip.pos[1] == y and arm.high[1] - arm.low[1] == pytest.approx(0.020)
    assert Y_IN - Y_OUT == pytest.approx(0.200)
    assert T.LIP_IN_X < board.low[0] and T.SLOT == pytest.approx(0.087)
    # Rack parts touch but never overlap each other.
    for a, b in itertools.combinations(T.RACK, 2):
        assert obb_gap(a.pos, rz(a.yaw), a.size, b.pos, rz(b.yaw), b.size) >= -1e-9, (a.name, b.name)


def test_the_slot_leaves_twenty_millimetres_each_side_of_the_hung_handle():
    assert T.LIP_GAP >= 0.020 and T.WALL_GAP >= 0.020
    assert T.GOAL_X - T.BAR_W / 2 - T.LIP_GAP == pytest.approx(T.LIP_IN_X)
    assert T.GOAL_X + T.BAR_W / 2 + T.WALL_GAP == pytest.approx(T.WALL_X)
    # The hung handle reaches 50 mm past each hook's outer edge (30 mm needed).
    g = hung()
    assert T.end_past_hook(g, Y_IN) == pytest.approx(0.050) and T.end_past_hook(g, Y_OUT) == pytest.approx(0.050)


def test_the_handle_starts_on_the_table_clear_of_the_rack():
    start = scene()["handle"]
    assert 0.0 < start.low[2] - TOP <= 0.002  # spawned just above the table, settles onto it
    gap, part = rack_gap(start.pos, rz(0.0), T.BAR_SIZE)
    assert gap >= 0.025, (part, gap)  # room for a finger between the handle and the rack
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for st in scene().values():
        assert np.all(st.low[:2] > tlo + 0.05) and np.all(st.high[:2] < thi - 0.05)


def test_the_start_and_doing_nothing_score_zero():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert TASK.distance(start) > 0.3


# -- the demo --------------------------------------------------------------------------------------------------------


@pytest.fixture(scope="module")
def played():
    """The right arm's plan, and per control step (time, joints, finger, TCP target pos, rotation, label)."""
    plan = TASK.demo()
    assert set(plan) == {"right"}  # the left arm stays in its ready pose
    arm = ArmPlan(plan["right"], station.BASES["right"])
    steps = []
    for k in range(round(arm.duration * HZ) + 5):
        t = (k + 1) / HZ
        pos, r, finger, label = arm.target(t)
        action, _ = arm.action7(t)
        steps.append((t, action[:6].copy(), finger, pos, r, label))
    return arm, steps


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(played):
    arm, steps = played
    joints = np.array([s[1] for s in steps])
    assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1
    assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER)
    assert np.all(joints[:, 3] - ik.LOWER[3] > 0.1)  # joint4 keeps 0.1 rad off its limit
    assert arm.duration * HZ + 100 < RUN_LIMIT_STEPS / 4


def test_the_carry_height_stays_under_the_joint4_limit_and_over_the_lips():
    assert T.CARRY_Z - TOP <= JOINT4_TRAVEL_LIMIT
    assert max(w.pos[2] for w in TASK.demo()["right"]) - TOP <= JOINT4_TRAVEL_LIMIT
    carried_underside = T.CARRY_Z - T.GRIP_BELOW
    assert carried_underside - (T.ARM_TOP + T.LIP_H) >= 0.015


def test_the_demo_lowers_slowly_lets_go_low_rises_before_moving_away_and_ends_still():
    w = {x.label: x for x in TASK.demo()["right"]}
    labels = [x.label for x in TASK.demo()["right"]]
    lower = w["lower"]
    assert (T.CARRY_Z - lower.pos[2]) / lower.duration <= 0.035  # at most 3.5 cm/s into the slots
    assert lower.pos[2] - T.GRIP_BELOW - T.ARM_TOP == pytest.approx(T.RELEASE_GAP)
    rise = w["rise clear"]
    assert rise.pos[:2] == w["release"].pos[:2] and rise.pos[2] - w["release"].pos[2] >= 0.03 - 1e-9
    assert labels.index("rise clear") == labels.index("release") + 1
    last, before = TASK.demo()["right"][-1], TASK.demo()["right"][-2]
    assert last.label == "hold still" and last.duration >= 1.5
    assert last.pos == before.pos and last.finger == before.finger and last.yaw == before.yaw
    # A quarter turn, done before the carry.
    assert w["turn"].yaw - w["lift"].yaw == pytest.approx(math.pi / 2)
    assert w["turn"].pos == w["lift"].pos and w["carry"].yaw == w["turn"].yaw


def _arm_hulls(side, joints, finger):
    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})  # the base is bolted to the table


def _hulls_to_rack(hulls) -> tuple[float, str]:
    best = (np.inf, "")
    for b in T.RACK:
        for link, h in hulls:
            s = float(box_signed_distance(h.vertices, np.asarray(b.pos), rz(b.yaw), np.asarray(b.size)).min())
            best = min(best, (s, f"{link}/{b.name}"))
    return best


def test_neither_arm_comes_within_five_millimetres_of_the_rack(played):
    """At every control step the right arm's links (fingers, gripper body, wrist) stay 5 mm off every rack part,
    and its finger tips stay above the table; the left arm stays in its ready pose, clear of everything."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    _, steps = played
    worst = (np.inf, "")
    for _, joints, finger, _, _, label in steps:
        hulls = _arm_hulls("right", joints, finger)
        low = min(h.vertices[:, 2].min() for _, h in hulls)
        assert low > TOP + 0.002, (label, low - TOP)
        gap = _hulls_to_rack(hulls)
        worst = min(worst, (gap[0], f"{label}: {gap[1]}"))
    assert worst[0] > 0.005, worst
    left = _arm_hulls("left", READY, FINGER_OPEN)
    assert _hulls_to_rack(left)[0] > 0.05


def _carried(pos, r, offset=0.0, yaw_err_deg=0.0):
    """The handle in the grip at a TCP pose: centred under the pads, ``offset`` along the closing direction, its
    long axis across the closing direction, turned ``yaw_err_deg`` in the pads (slip)."""
    closing = r[:, 0].copy()
    closing[2] = 0.0
    closing /= np.linalg.norm(closing)
    axis = np.cross(Z, closing)
    rot = rz(math.radians(yaw_err_deg)) @ np.c_[axis, np.cross(Z, axis), Z]
    centre = np.asarray(pos) + offset * closing + (T.BAR_W / 2 - T.GRIP_BELOW) * Z
    return centre, rot


def test_the_fingers_close_along_y_at_the_pick_and_along_x_at_the_rack():
    assert abs(grip_rotation(T.PICK_YAW)[:, 0] @ np.array([0.0, 1.0, 0.0])) == pytest.approx(1.0)
    assert abs(grip_rotation(T.PLACE_YAW)[:, 0] @ np.array([1.0, 0.0, 0.0])) == pytest.approx(1.0)
    c, r = _carried((*T.BAR_START, T.GRIP_Z), grip_rotation(T.PICK_YAW))
    assert np.allclose(c, T.BAR.pos[:2] + (TOP + T.BAR_W / 2,), atol=1e-9)
    assert abs(r[:, 0] @ np.array([1.0, 0.0, 0.0])) == pytest.approx(1.0)  # gripped across, lying along x


CARRIED = ("lift", "turn", "carry", "lower")


@pytest.mark.parametrize("offset", [0.0, -0.015, 0.015])
@pytest.mark.parametrize("yaw_err", [0.0, -5.0, 5.0])
def test_the_carried_handle_never_touches_the_rack(played, offset, yaw_err):
    """From the lift to the release the handle in the grip (centred, or 15 mm off along the closing direction,
    turned 5 deg in the pads either way) stays at least 2 mm clear of every rack part (the closest is the end of
    the lowering, 3 mm above the hook arms). It also clears the left arm in its ready pose by 2 cm."""
    _, steps = played
    left = _arm_hulls("left", READY, FINGER_OPEN)
    worst = (np.inf, "")
    for _, _, _, pos, r, label in steps:
        if label not in CARRIED:
            continue
        c, rot = _carried(pos, r, offset, yaw_err)
        gap, part = rack_gap(c, rot, T.BAR_SIZE)
        worst = min(worst, (gap, f"{label}: {part}"))
        for _, h in left:
            assert box_signed_distance(h.vertices, c, rot, np.asarray(T.BAR_SIZE)).min() > 0.02
    assert worst[0] > 0.002, worst


def test_the_handle_is_lowered_into_both_slots_at_once_from_above_the_lips(played):
    """At the end of the carry the handle is over both slots, along y, its underside above the lip tops."""
    _, steps = played
    pos, r = next((s[3], s[4]) for s in steps if s[5] == "lower")
    c, rot = _carried(pos, r)
    assert c[2] - T.BAR_W / 2 >= T.ARM_TOP + T.LIP_H + 0.014
    assert abs(rot[1, 0]) == pytest.approx(1.0, abs=1e-3)
    assert c[0] == pytest.approx(T.GOAL_X, abs=1e-3) and c[1] == pytest.approx(T.RACK_Y, abs=1e-3)


def _demo_result(offset=0.0, pick_err=(0.0, 0.0), place_err=(0.0, 0.0), turn_loss_deg=0.0) -> ObjectState:
    """Where the demo leaves the handle under a simple grip model: closing the fingers pulls it ``offset`` off the
    TCP along the closing direction (and the release shifts it again; both fold into ``offset``), the TCP is off
    by ``pick_err`` at the grip and ``place_err`` at the release, and the turn comes up ``turn_loss_deg`` short."""
    start = np.asarray(T.BAR_START)
    closing = np.array([-math.sin(T.PICK_YAW), math.cos(T.PICK_YAW)])
    along = np.array([closing[1], -closing[0]])
    tcp_pick = start + np.asarray(pick_err)
    held = offset * closing + float(np.dot(start - tcp_pick, along)) * along  # handle centre minus TCP
    turn = T.PLACE_YAW - T.PICK_YAW - math.radians(turn_loss_deg)
    c, s = math.cos(turn), math.sin(turn)
    final = np.array([T.GOAL_X, T.RACK_Y]) + np.asarray(place_err) + np.array([[c, -s], [s, c]]) @ held
    return hung(final[0] - T.GOAL_X, final[1] - T.RACK_Y, yaw_deg=math.degrees(T.BAR.yaw + turn))


def test_the_demo_plan_hangs_the_handle_with_a_centred_grip():
    end = scene(_demo_result())
    assert TASK.score(end) == 1.0 and TASK.success(end)


def test_the_demo_still_hangs_it_with_an_off_centre_grip_wrist_error_and_turn_loss():
    """Every combination of a 15 mm grip offset either way (the GPU rounds measured under 1 mm on 16-25 mm bars,
    plus 6-10 mm from opening the pads), 5 mm of wrist error in x and y at the grip and at the release, and 5 deg
    of turn lost either way (about 4 deg was measured)."""
    errs = [(dx, dy) for dx in (-0.005, 0.005) for dy in (-0.005, 0.005)]
    for offset, pick, place, loss in itertools.product((-0.015, 0.0, 0.015), errs, errs, (-5.0, 0.0, 5.0)):
        end = scene(_demo_result(offset, pick, place, loss))
        assert TASK.score(end) == 1.0, (offset, pick, place, loss)


# -- scoring: full credit ------------------------------------------------------------------------------------------


def test_the_goal_succeeds_scores_one_and_is_at_zero_distance():
    g = scene(hung())
    assert TASK.score(g) == 1.0 and TASK.success(g)
    assert TASK.distance(g) == pytest.approx(0.0, abs=1e-9)
    assert T.hooks_seated(g) == 2


@pytest.mark.parametrize("yaw", [90.0, -90.0, 86.0, 94.0, 270.0])
def test_hung_either_end_first_or_a_few_degrees_skew_still_counts(yaw):
    assert TASK.score(scene(hung(yaw_deg=yaw))) == 1.0


@pytest.mark.parametrize("roll", [0.0, 45.0, 90.0, 30.0])
def test_a_handle_rolled_about_its_own_axis_still_rests_in_the_hooks(roll):
    assert TASK.score(scene(hung(roll_deg=roll))) == 1.0


@pytest.mark.parametrize("dx", [-0.029, 0.0, 0.034])
def test_anywhere_across_the_slot_counts(dx):
    """Against the lip or against the board: both rest in the hook."""
    assert TASK.score(scene(hung(dx=dx))) == 1.0


def test_sliding_along_the_hooks_is_allowed_while_both_ends_overhang_thirty_millimetres():
    assert TASK.score(scene(hung(dy=0.019))) == 1.0
    assert TASK.score(scene(hung(dy=-0.019))) == 1.0


# -- scoring: one hook only ----------------------------------------------------------------------------------------


@pytest.mark.parametrize("seated_y, other_y", [(Y_IN, Y_OUT), (Y_OUT, Y_IN)])
def test_tilted_onto_the_other_hooks_lip_scores_partial(seated_y, other_y):
    """Seated in one hook, the other end resting on top of the other hook's lip: a tilted placement."""
    p = (T.GOAL_X, seated_y, T.ARM_TOP + T.BAR_W / 2)
    q = (T.LIP_IN_X - T.LIP_T / 2, other_y, T.ARM_TOP + T.LIP_H + T.BAR_W / 2)
    h = handle_through(p, q, at=0.100)
    st = scene(h)
    assert T.seated(h, seated_y) and not T.seated(h, other_y)
    assert TASK.score(st) == pytest.approx(0.3) and not TASK.success(st)


@pytest.mark.parametrize("seated_y, toward", [(Y_OUT, +1.0), (Y_IN, -1.0)])
def test_one_end_in_a_hook_and_the_other_down_on_the_table_scores_partial(seated_y, toward):
    """Seated in one hook, sloping down past the front of the other hook's lip to the table."""
    p = np.array([T.GOAL_X, seated_y, T.ARM_TOP + T.BAR_W / 2])
    q = np.array([0.20, seated_y + toward * 0.27, TOP + T.BAR_W / 2])
    h = handle_through(p, q, at=0.090)
    other = Y_IN if seated_y == Y_OUT else Y_OUT
    point, _ = T.at_hook(h, other)
    assert point[0] < T.LIP_IN_X - T.LIP_T - T.BAR_W / 2  # in front of the other lip, not through it
    assert TASK.score(scene(h)) == pytest.approx(0.3)


def test_a_handle_slid_until_one_end_barely_reaches_a_hook_scores_partial():
    """Hung level, but slid along y so that one end reaches only 20 mm past a hook: that hook does not count."""
    h = hung(dy=-0.030)
    assert T.end_past_hook(h, Y_OUT) >= 0.030 and T.end_past_hook(h, Y_IN) < 0.030
    assert TASK.score(scene(h)) == pytest.approx(0.3)


# -- scoring: shortcuts and failures that score 0 -------------------------------------------------------------------


def test_resting_on_top_of_both_lips_scores_zero():
    h = handle((T.LIP_IN_X - T.LIP_T / 2, T.RACK_Y, T.ARM_TOP + T.LIP_H + T.BAR_W / 2), rz(math.pi / 2))
    assert T.hooks_seated(scene(h)) == 0 and TASK.score(scene(h)) == 0.0


def test_resting_on_the_board_top_scores_zero():
    h = handle((T.WALL_X + T.BOARD_T / 2, T.RACK_Y, TOP + T.BOARD_H + T.BAR_W / 2), rz(math.pi / 2))
    assert TASK.score(scene(h)) == 0.0


def test_leaning_on_the_board_above_the_hooks_scores_zero():
    """Foot on the table, the top leaning on the board's face above the hooks, along the board (crossing both
    hook planes but high above the outer hook and under the inner one) or straight at it."""
    along = handle_through((0.333, -0.02, TOP + T.BAR_W / 2), (0.333, -0.32, TOP + 0.135), at=0.16)
    point, under = T.at_hook(along, Y_OUT)
    assert T.LIP_IN_X < point[0] < T.WALL_X and under > T.ARM_TOP + 0.02
    assert TASK.score(scene(along)) == 0.0
    straight = handle_through((0.10, T.RACK_Y, TOP + T.BAR_W / 2), (T.WALL_X - 0.011, T.RACK_Y, TOP + 0.14), at=0.16)
    assert TASK.score(scene(straight)) == 0.0


def test_lying_on_the_table_in_front_of_or_under_the_hooks_scores_zero():
    in_front = handle((0.20, T.RACK_Y, TOP + T.BAR_W / 2), rz(math.pi / 2))
    under = handle((T.GOAL_X, T.RACK_Y, TOP + T.BAR_W / 2), rz(math.pi / 2))  # in the slots, but on the table
    assert TASK.score(scene(in_front)) == 0.0 and TASK.score(scene(under)) == 0.0


def test_held_above_the_hooks_or_still_moving_scores_zero():
    assert TASK.score(scene(hung(dz=0.030))) == 0.0  # held 3 cm up, not resting on the arms
    assert TASK.score(scene(hung(vel=(0.0, 0.0, -0.05)))) == 0.0  # still settling
    assert TASK.score(scene(hung(dz=0.004))) == 1.0  # within the 5 mm seat tolerance


def test_crossing_the_rack_instead_of_lying_along_it_scores_zero():
    """Pointing at the board (along x), or more than 60 deg off y, the handle does not lie across the hooks."""
    for yaw in (0.0, 25.0, 155.0):
        h = handle((T.GOAL_X - 0.10, Y_IN, T.ARM_TOP + T.BAR_W / 2), rz(math.radians(yaw)))
        assert T.at_hook(h, Y_OUT) is None and TASK.score(scene(h)) == 0.0


def test_dropped_on_the_floor_or_left_at_the_start_scores_zero():
    floor = handle((0.30, -0.80, station.FLOOR_Z + T.BAR_W / 2), rz(math.pi / 2))
    assert TASK.score(scene(floor)) == 0.0 and TASK.score(scene()) == 0.0


def test_score_stays_in_range_and_success_matches_full_score_over_random_states():
    rng = np.random.default_rng(0)
    seen = set()
    for _ in range(2000):
        pos = np.array([rng.uniform(0.15, 0.40), rng.uniform(-0.40, 0.10), rng.uniform(TOP, TOP + 0.15)])
        if rng.random() < 0.5:  # near the goal, to reach every branch
            pos = T.GOAL_POS + rng.normal(0.0, [0.02, 0.03, 0.006])
        d = np.array([rng.normal(0, 0.3), 1.0, rng.normal(0, 0.1)]) if rng.random() < 0.7 else rng.normal(size=3)
        st = scene(handle(pos, axis_frame(d, rng.uniform(0, 90))))
        s = TASK.score(st)
        seen.add(s)
        assert s in (0.0, 0.3, 1.0) and TASK.success(st) == (s == 1.0)
        assert TASK.distance(st) >= 0.0
    assert seen == {0.0, 0.3, 1.0}


# -- the top camera ------------------------------------------------------------------------------------------------

CAM = station.top_camera("calibrated")


def _project(points):
    """World points -> (pixel (u, v) at 640 x 360, depth) in the calibrated top camera (OpenGL: looks along -z)."""
    rot = quat_xyzw_to_matrix(np.asarray(CAM.rot_xyzw))
    c = (np.asarray(points) - np.asarray(CAM.pos)) @ rot
    depth = -c[:, 2]
    lens = CAM.lens
    return np.c_[lens.cx + lens.fx * c[:, 0] / depth, lens.cy - lens.fy * c[:, 1] / depth], depth


def _hull2d(p):
    pts = sorted(map(tuple, np.round(p, 6)))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return np.array(lower[:-1] + upper[:-1])


def _inside(poly, pts):
    ok = np.ones(len(pts), bool)
    for i in range(len(poly)):
        a, b = poly[i], poly[(i + 1) % len(poly)]
        ok &= (b[0] - a[0]) * (pts[:, 1] - a[1]) - (b[1] - a[1]) * (pts[:, 0] - a[0]) >= 0
    return ok


def _surface_points(st: ObjectState, size, n=7):
    r = T._rotation(st)
    g = np.linspace(-0.5, 0.5, n)
    local = np.array([[a, b, c] for a in g for b in g for c in g])
    local = local[np.abs(local).max(axis=1) >= 0.5] * np.asarray(size)
    return local @ r.T + np.asarray(st.pos)


def _hidden_fraction(points, hulls) -> float:
    uv, depth = _project(points)
    hidden = np.zeros(len(points), bool)
    for _, h in hulls:
        huv, hdepth = _project(h.vertices)
        hidden |= _inside(_hull2d(huv), uv) & (hdepth.min() < depth)
    return float(hidden.mean())


def _scored(handle_state):
    items = [(handle_state, T.BAR_SIZE)]
    items += [(box_state(b.pos, b.size), b.size) for b in T.RACK if b.name.startswith("hook_")]
    return items


def test_the_top_camera_sees_the_handle_and_both_hooks_in_the_first_and_last_frames(played):
    """First frame: both arms in the ready pose. Last frame: the right arm parked, the handle hung. Every surface
    point of the handle and of both hooks projects inside the image and none is behind an arm link."""
    _, steps = played
    ready = _arm_hulls("left", READY, FINGER_OPEN) + _arm_hulls("right", READY, FINGER_OPEN)
    parked = _arm_hulls("left", READY, FINGER_OPEN) + _arm_hulls("right", steps[-1][1], steps[-1][2])
    for frame, hulls, h in (("first", ready, scene()["handle"]), ("last", parked, hung())):
        for st, size in _scored(h):
            pts = _surface_points(st, size)
            uv, _ = _project(pts)
            assert np.all(uv[:, 0] > 10) and np.all(uv[:, 0] < CAM.lens.width - 10), frame
            assert np.all(uv[:, 1] > 10) and np.all(uv[:, 1] < CAM.lens.height - 10), frame
            assert _hidden_fraction(pts, hulls) == 0.0, frame


def test_seated_and_resting_on_the_lips_differ_by_more_than_a_centimetre():
    """Not a camera claim: the scorer's 5 mm seat tolerance sits well inside the 15 mm lip height."""
    assert T.LIP_H - T.SEAT_TOL >= 0.010 - 1e-12
