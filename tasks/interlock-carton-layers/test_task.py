"""Offline checks for interlock-carton-layers (NumPy and pytest only, no Isaac Sim).

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
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import ArmPlan, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState, load  # noqa: E402
from yam_arena.urdf import axis_angle_matrix, forward_kinematics  # noqa: E402

HZ = 20.0  # the rate the task is run at (-E control_hz=20)
TASK_PATH = Path(__file__).resolve().parent / "task.py"
TASK = load(TASK_PATH)
T = TASK.module
TOP = TABLE_TOP["station"]
GRIPPER_MAX_GAP = 0.094  # pads 94 mm apart at FINGER_OPEN (arena tests/test_constants.py)
MAX_GRIP = 0.084  # the widest grip allowed: 94 mm less a 1 cm margin
GRIP_OFFSET = 0.015  # a held carton can sit this far off centre between the pads (measured 12-20 mm in sim)
RUN_LIMIT_STEPS = 2400
L, W, H = T.CARTON_SIZE


# -- helpers ---------------------------------------------------------------------------------------------------


def rot(yaw=0.0, tilt_deg=0.0, tilt_axis=(1.0, 0.0, 0.0)) -> np.ndarray:
    """Turn ``yaw`` about the vertical after tipping ``tilt_deg`` about a body axis."""
    tip = axis_angle_matrix(np.asarray(tilt_axis, float), math.radians(tilt_deg))
    return axis_angle_matrix(np.array([0.0, 0.0, 1.0]), yaw) @ tip


CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)


def box_state(pos, size, r=None, vel=(0.0, 0.0, 0.0)) -> ObjectState:
    """An ObjectState as the simulator reports it: world-aligned bounds of the turned box."""
    r = np.eye(3) if r is None else r
    pos, half = np.asarray(pos, float), np.asarray(size, float) / 2
    world = (CORNERS * half) @ r.T + pos
    return ObjectState(pos, ik.matrix_to_quat_xyzw(r), np.asarray(vel, float), np.zeros(3), world.min(0), world.max(0))


def carton(xy, z_low, yaw_deg=0.0, tilt=0.0, tilt_axis=(1.0, 0.0, 0.0), vel=(0, 0, 0)) -> ObjectState:
    """A carton with its centre over ``xy`` and its lowest corner at ``z_low``; yaw 0 = long side along x."""
    r = rot(math.radians(yaw_deg), tilt, tilt_axis)
    half_h = float(np.abs((CORNERS * np.asarray(T.CARTON_SIZE) / 2) @ r.T)[:, 2].max())
    return box_state((xy[0], xy[1], z_low + half_h), T.CARTON_SIZE, r, vel)


def bottom(xy, yaw_deg=90.0, **kw):
    return carton(xy, T.BOARD_TOP, yaw_deg, **kw)


def second(xy, yaw_deg=0.0, **kw):
    return carton(xy, T.BOARD_TOP + H, yaw_deg, **kw)


def scene(**cartons) -> dict:
    """Every object's state: the board where it is, cartons at their start unless given."""
    out = {}
    for o in TASK.objects:
        out[o.name] = box_state(o.pos, o.size, rot(o.yaw))
    out.update(cartons)
    return out


BX, BY = T.BOARD_CENTER
P = T.PITCH / 2


def goal(dx=0.0, dy=0.0):
    """The demo's pattern: first layer across the arms, second layer pointing away from them."""
    return scene(
        carton_1=bottom((BX + P + dx, BY + dy)),
        carton_3=bottom((BX - P + dx, BY + dy)),
        carton_2=second((BX + dx, BY + P + dy)),
        carton_4=second((BX + dx, BY - P + dy)),
    )


def fixed_boxes():
    return [o for o in TASK.objects if not o.free]


def surface_points(center, r, size, n=7) -> np.ndarray:
    """Points on the six faces of an oriented box (an n x n grid per face)."""
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
    """Approximate clearance between two oriented boxes (m; negative = overlap), from surface samples."""
    a = box_signed_distance(surface_points(c1, r1, s1), np.asarray(c2), np.asarray(r2), np.asarray(s2)).min()
    b = box_signed_distance(surface_points(c2, r2, s2), np.asarray(c1), np.asarray(r1), np.asarray(s1)).min()
    return float(min(a, b))


def hull_box_clearance(hulls, center, r, size) -> float:
    """Clearance between arm hulls and an oriented box: hull vertices in the box, and box samples in the hulls."""
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
        inside = (pts @ h.planes[:, :3].T - h.planes[:, 3]).max(axis=1)
        best = min(best, inside.min())
    return float(best)


# -- the demo, played once at 20 Hz per arm ----------------------------------------------------------------------


@pytest.fixture(scope="module")
def played():
    """Per arm: the plan, and per control step (time, joints, finger, TCP target pos, TCP target rotation, label)."""
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


def _events(played):
    """Per carton: the arm, and the times it is gripped ('close' starts), released ('release' starts) and the
    arm is clear ('lift off' ends)."""
    out = {}
    for arm, name in T.DEMO:
        out[name] = {"arm": arm}
    for side, (_, steps) in played.items():
        names = [n for a, n in T.DEMO if a == side]
        labels = [s[5] for s in steps]
        times = [s[0] for s in steps]
        idx = 0
        for k in range(1, len(steps)):
            if labels[k] == "close" and labels[k - 1] != "close":
                out[names[idx]]["close"] = times[k]
            if labels[k] == "release" and labels[k - 1] != "release":
                out[names[idx]]["release"] = times[k]
            if labels[k - 1] == "lift off" and labels[k] != "lift off":
                out[names[idx]]["clear"] = times[k]
                idx += 1
    return out


def _placed_state(name):
    """Where the demo leaves ``name``: centred under the TCP at release, at the place yaw, on its support."""
    gx, gy, layer, heading = T.GOAL[name]
    return carton((gx, gy), T.BOARD_TOP + layer * H, heading)


# -- loading and description -------------------------------------------------------------------------------------


def test_the_task_file_loads_with_hooks_and_a_neutral_docstring():
    assert TASK.scene == "station" and TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert set(TASK.free_objects) == {"carton_1", "carton_2", "carton_3", "carton_4"}
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.docs.startswith("On the station table")
    for word in ("university", "team", "lab ", "dream"):
        assert word not in TASK.docs.lower()
    assert [o.name for o in fixed_boxes()] == ["pallet_board"]


# -- geometry ----------------------------------------------------------------------------------------------------


def test_every_carton_fits_the_gripper_across_its_width_only():
    assert W <= MAX_GRIP
    assert L > GRIPPER_MAX_GAP and H < W < L  # only the width can be gripped; a carton on its side stands taller
    assert -FINGER_OPEN * 2 == pytest.approx(GRIPPER_MAX_GAP, abs=5e-4)
    # Approaching with the fingers fully open leaves 9.5 mm on each side of the carton.
    assert (GRIPPER_MAX_GAP - W) / 2 == pytest.approx(0.0095)


def test_two_cartons_make_a_square_layer_and_the_board_holds_it_with_room_for_a_grip_offset():
    layer = 2 * W + T.LAYER_GAP
    assert layer == pytest.approx(L)
    margin = (T.BOARD_SIZE[0] - layer) / 2
    assert margin >= 2 * GRIP_OFFSET - 1e-9  # 3 cm of board around the nominal pattern
    assert T.BOARD_SIZE[0] == T.BOARD_SIZE[1]
    # Two cartons cannot lie end to end on the board, and one layer holds at most three side by side.
    assert 2 * L > T.BOARD_SIZE[0] and 4 * W > T.BOARD_SIZE[0]


def test_the_stack_is_low():
    assert T.BOARD_TOP + 2 * H - TOP <= 0.115
    assert T.TRAVEL_Z - TOP <= 0.14


# -- starting layout ---------------------------------------------------------------------------------------------


def _bounds(o):
    st = box_state(o.pos, o.size, rot(o.yaw))
    return st.low, st.high


def test_nothing_overlaps_at_the_start_and_the_cartons_have_finger_room():
    objects = list(TASK.objects)
    for i, a in enumerate(objects):
        lo_a, hi_a = _bounds(a)
        for b in objects[i + 1 :]:
            lo_b, hi_b = _bounds(b)
            gap = np.maximum(lo_b - hi_a, lo_a - hi_b)[:2].max()
            assert gap > 0.03, (a.name, b.name, gap)


def test_everything_stands_on_the_table_inside_its_edges():
    tlo = np.array(station.TABLE_CENTER[:2]) - np.array(station.TABLE_SIZE[:2]) / 2
    thi = np.array(station.TABLE_CENTER[:2]) + np.array(station.TABLE_SIZE[:2]) / 2
    for o in TASK.objects:
        lo, hi = _bounds(o)
        assert np.all(lo[:2] > tlo) and np.all(hi[:2] < thi), o.name
        if o.free:
            assert 0.0 <= lo[2] - TOP <= 0.002, o.name  # spawned just above the table, settles onto it
            assert o.yaw == 0.0  # all four start the same way round
        else:
            assert lo[2] == pytest.approx(TOP, abs=1e-9), o.name


def test_the_start_scores_zero_and_is_far_from_the_goal():
    start = scene()
    assert TASK.score(start) == 0.0 and not TASK.success(start)
    assert TASK.distance(start) > 0.5


# -- the demo -------------------------------------------------------------------------------------------------------


def test_the_demo_is_reachable_within_the_joint_limits_at_20_hz(played):
    assert set(played) == {"left", "right"}
    durations = set()
    for side, (arm, steps) in played.items():
        joints = np.array([s[1] for s in steps])
        assert arm.worst_ik_mm < 0.5 and arm.worst_ik_deg < 0.1, side
        assert np.all(joints >= ik.LOWER) and np.all(joints <= ik.UPPER), side
        durations.add(round(arm.duration, 6))
        assert arm.duration * HZ + 100 < RUN_LIMIT_STEPS / 2
    assert len(durations) == 1  # both arms finish together


def test_the_demo_ends_with_both_arms_holding_still_for_at_least_1_5_s():
    for side, wps in TASK.demo().items():
        last = wps[-1]
        assert last.label == "hold still" and last.duration >= 1.5, side
        prev = wps[-2]
        assert tuple(last.pos) == tuple(prev.pos) and last.finger == prev.finger and last.yaw == prev.yaw


def test_travel_lowering_and_lift_off_follow_the_gpu_round_rules():
    for side, wps in TASK.demo().items():
        for a, b in zip(wps, wps[1:], strict=False):
            if b.label == "lower onto the stack":
                drop = a.pos[2] - b.pos[2]
                assert drop / b.duration <= 0.035, side  # average lowering speed
            if a.label == "release":
                assert b.label == "lift off" and b.pos[:2] == a.pos[:2]
                assert b.pos[2] - a.pos[2] >= 0.03 - 1e-9  # straight up 3 cm before any sideways move
        assert max(w.pos[2] for w in wps) <= TOP + 0.14 + 1e-9


def _arm_hulls(side, joints, finger):
    from yam_arena import YAM_URDF

    q = {f"joint{i + 1}": float(v) for i, v in enumerate(joints)}
    q.update(joint7=finger, joint8=finger)
    poses = forward_kinematics(YAM_URDF, q)
    base = np.eye(4)
    base[:3, 3] = station.BASES[side]
    return world_hulls({k: base @ v for k, v in poses.items() if k != "base"})  # the base is bolted to the table


def _others(name_carried, t, ev):
    """Every carton except ``name_carried`` at time t: at its start until gripped, then at its goal once released."""
    out = []
    for name, spec in T.CARTONS.items():
        if name == name_carried:
            continue
        e = ev[name]
        if t < e["close"]:
            out.append((name, np.asarray(spec.pos), np.eye(3), np.asarray(T.CARTON_SIZE)))
        elif t >= e["release"] + 0.6:
            st = _placed_state(name)
            out.append((name, st.pos, ik_rot(st), np.asarray(T.CARTON_SIZE)))
        # in between, the carton is in a gripper: the other arm's business
    return out


def ik_rot(state):
    return T._rot(state)


def test_the_arm_and_the_carried_carton_clear_the_board_and_the_other_cartons(played):
    """At every control step: no arm link within 5 mm of the board or a carton it is not holding, with the
    fingers taken as wide open while a carton is held (either pad can sit at its open stop when the grip is off
    centre); finger tips above the table; and the carried carton, 15 mm off centre either way between the pads,
    at least 3 mm clear of the board and every other carton until it is let go."""
    assert set(link_hulls()) >= {"gripper", "tip_left", "tip_right"}
    ev = _events(played)
    board = fixed_boxes()[0]
    worst_arm, worst_carton = {}, {}
    for side, (_, steps) in played.items():
        mine = [n for a, n in T.DEMO if a == side]
        worst_arm[side], worst_carton[side] = np.inf, np.inf
        for k, (t, joints, finger, pos, r, label) in enumerate(steps):
            if k % 2:
                continue
            held = next((n for n in mine if ev[n]["close"] <= t < ev[n]["clear"]), None)
            hull_finger = FINGER_OPEN if held and t >= ev[held]["close"] + 0.8 and t < ev[held]["release"] else finger
            hulls = _arm_hulls(side, joints, hull_finger)
            low = min(h.vertices[:, 2].min() for _, h in hulls)
            assert low > TOP + 0.005, (side, label)
            boxes = [("pallet_board", np.asarray(board.pos), np.eye(3), np.asarray(board.size))]
            boxes += _others(held, t, ev)
            if held is None or t >= ev[held]["close"] + 0.8:
                # the item being gripped is excluded; the board and every other carton count
                for _, c, rr, s in boxes:
                    worst_arm[side] = min(worst_arm[side], hull_box_clearance(hulls, c, rr, s))
            if held and ev[held]["close"] + 0.8 <= t < ev[held]["release"]:
                item_r = r @ grip_rotation(T.PICK_YAW).T
                centre = np.asarray(pos) - np.array([0.0, 0.0, H / 2 - T.GRIP_BELOW_TOP])
                across = item_r[:, 1]  # the fingers close along the carton's width
                for shift in (-GRIP_OFFSET, 0.0, GRIP_OFFSET):
                    c = centre + shift * across
                    for _, bc, br, bs in boxes:
                        worst_carton[side] = min(worst_carton[side], box_box_clearance(c, item_r, T.CARTON_SIZE, bc, br, bs))
    for side in played:
        assert worst_arm[side] > 0.005, (side, worst_arm[side])
        assert worst_carton[side] > 0.003, (side, worst_carton[side])


def test_the_two_arms_stay_apart(played):
    """Bounding boxes of the two grippers (and finger tips) never come within 5 cm of each other."""
    (_, left), (_, right) = played["left"], played["right"]
    links = {"gripper", "tip_left", "tip_right"}
    worst = np.inf
    for k in range(0, min(len(left), len(right)), 2):
        boxes = []
        for side, steps in (("left", left), ("right", right)):
            hulls = [h for n, h in _arm_hulls(side, steps[k][1], steps[k][2]) if n in links]
            v = np.vstack([h.vertices for h in hulls])
            boxes.append((v.min(0), v.max(0)))
        (alo, ahi), (blo, bhi) = boxes
        worst = min(worst, np.maximum(blo - ahi, alo - bhi).max())
    assert worst > 0.05, worst


def test_the_demo_releases_each_carton_flat_just_above_its_slot(played):
    ev = _events(played)
    for name, e in ev.items():
        _, steps = played[e["arm"]]
        k = next(i for i, s in enumerate(steps) if s[0] >= e["release"])
        _, _, _, pos, r, _ = steps[k]
        gx, gy, layer, heading = T.GOAL[name]
        support = T.BOARD_TOP + layer * H
        bottom_z = pos[2] + T.GRIP_BELOW_TOP - H
        assert 0.0 < bottom_z - support <= 0.01, name
        assert pos[0] == pytest.approx(gx) and pos[1] == pytest.approx(gy)
        item_r = r @ grip_rotation(T.PICK_YAW).T
        assert math.degrees(math.atan2(item_r[1, 0], item_r[0, 0])) % 180.0 == pytest.approx(heading, abs=0.5)
    # Released in order, each into a free board.
    order = [n for _, n in T.DEMO]
    for a, b in zip(order, order[1:], strict=False):
        assert ev[a]["clear"] <= ev[b]["release"] - 2.0


def test_the_demo_end_state_scores_one_even_with_each_carton_15_mm_off():
    final = scene(**{n: _placed_state(n) for n in T.NAMES})
    assert TASK.success(final) and TASK.score(final) == 1.0
    assert TASK.distance(final) == pytest.approx(0.0, abs=1e-9)
    rng = np.random.default_rng(1)
    for _ in range(200):
        moved = {}
        for n in T.NAMES:
            st = _placed_state(n)
            dx, dy = rng.uniform(-GRIP_OFFSET, GRIP_OFFSET, size=2)
            yaw = T.GOAL[n][3] + rng.uniform(-4, 4)  # two cartons 4 deg off each way: 8 deg apart at most
            moved[n] = carton((st.pos[0] + dx, st.pos[1] + dy), T.BOARD_TOP + T.GOAL[n][2] * H, yaw)
        # skip draws where two cartons of a layer would overlap (not physical)
        lay0 = [moved["carton_1"], moved["carton_3"]]
        lay1 = [moved["carton_2"], moved["carton_4"]]
        if any(T.length_over(a, b) > 0 for a, b in (lay0, lay1)):
            continue
        assert TASK.score(scene(**moved)) == 1.0


# -- scoring ---------------------------------------------------------------------------------------------------------


def test_the_goal_succeeds_and_scores_one():
    assert TASK.success(goal()) and TASK.score(goal()) == 1.0


def test_the_pattern_turned_the_other_way_or_pushed_tight_also_scores_one():
    turned = scene(
        carton_1=bottom((BX, BY + P), yaw_deg=0.0),
        carton_3=bottom((BX, BY - P), yaw_deg=0.0),
        carton_2=second((BX + P, BY), yaw_deg=90.0),
        carton_4=second((BX - P, BY), yaw_deg=90.0),
    )
    assert TASK.success(turned) and TASK.score(turned) == 1.0
    assert TASK.distance(turned) == pytest.approx(0.0, abs=1e-9)
    tight = W / 2 + 0.0005
    pushed = scene(
        carton_1=bottom((BX + tight, BY)),
        carton_3=bottom((BX - tight, BY)),
        carton_2=second((BX, BY + tight)),
        carton_4=second((BX, BY - tight)),
    )
    assert TASK.success(pushed) and TASK.score(pushed) == 1.0
    flipped = scene(
        carton_1=bottom((BX + P, BY), yaw_deg=270.0, tilt=180.0),
        carton_3=bottom((BX - P, BY), yaw_deg=-90.0),
        carton_2=second((BX, BY + P), yaw_deg=180.0, tilt=4.0),
        carton_4=second((BX + 0.01, BY - P), yaw_deg=8.0),
    )
    assert TASK.score(flipped) == 1.0


@pytest.mark.parametrize(
    "layout, expected",
    [
        # single layer: two cartons on the board, two left on the table
        (dict(carton_1=bottom((BX + P, BY)), carton_3=bottom((BX - P, BY))), 0.4),
        # one carton on the board
        (dict(carton_1=bottom((BX + P, BY))), 0.2),
        # column stack: second layer the same way round as the first
        (
            dict(
                carton_1=bottom((BX + P, BY)),
                carton_3=bottom((BX - P, BY)),
                carton_2=second((BX + P, BY), yaw_deg=90.0),
                carton_4=second((BX - P, BY), yaw_deg=90.0),
            ),
            0.6,
        ),
        # one carton interlocked, the other column-stacked
        (
            dict(
                carton_1=bottom((BX + P, BY)),
                carton_3=bottom((BX - P, BY)),
                carton_2=second((BX, BY + P)),
                carton_4=second((BX - P, BY), yaw_deg=90.0),
            ),
            0.8,
        ),
        # one interlocked carton, the fourth left on the table
        (dict(carton_1=bottom((BX + P, BY)), carton_3=bottom((BX - P, BY)), carton_2=second((BX, BY + P))), 0.7),
        # the same way round as the first layer, on the far carton only
        (
            dict(
                carton_1=bottom((BX + P, BY)),
                carton_3=bottom((BX - P, BY)),
                carton_2=second((BX + P, BY), yaw_deg=90.0),
            ),
            0.5,
        ),
        # turned only 45 deg: stacked, not interlocked
        (
            dict(carton_1=bottom((BX + P, BY)), carton_3=bottom((BX - P, BY)), carton_2=second((BX, BY), yaw_deg=45.0)),
            0.5,
        ),
        # a second-layer carton over the board's edge
        (
            dict(
                carton_1=bottom((BX + P, BY)),
                carton_3=bottom((BX - P, BY)),
                carton_2=second((BX, BY + P + 0.05)),
            ),
            0.4,
        ),
    ],
)
def test_every_partial_credit_branch_returns_its_stated_value(layout, expected):
    objects = scene(**layout)
    assert TASK.score(objects) == pytest.approx(expected)
    assert not TASK.success(objects)


def test_the_turn_tolerance_is_ten_degrees():
    def with_top(yaw):
        return scene(
            carton_1=bottom((BX + P, BY)),
            carton_3=bottom((BX - P, BY)),
            carton_2=second((BX, BY + P), yaw_deg=yaw),
            carton_4=second((BX, BY - P)),
        )

    assert TASK.score(with_top(9.0)) == 1.0 and TASK.score(with_top(-9.0)) == 1.0
    assert TASK.score(with_top(12.0)) == pytest.approx(0.8)


def test_the_joint_must_be_bridged_by_a_fifth_of_the_footprint():
    # The near first-layer carton pushed 3 cm toward the board's near edge; a second-layer carton slid toward the
    # far edge rests on less and less of it, and once under a fifth of its length is on it, it is only stacked.
    def slid(dx):
        return scene(
            carton_1=bottom((BX + P, BY)),
            carton_3=bottom((BX - P - 0.03, BY)),
            carton_2=second((BX + dx, BY + P)),
            carton_4=second((BX, BY - P)),
        )

    def near_share(dx):
        objects = slid(dx)
        return T.length_over(objects["carton_2"], objects["carton_3"])

    assert near_share(0.0) >= T.BRIDGE_MIN and TASK.score(slid(0.0)) == 1.0
    assert near_share(0.035) < T.BRIDGE_MIN and TASK.score(slid(0.035)) == pytest.approx(0.8)
    assert T.classify(slid(0.035))["carton_2"] == "stacked"


def test_a_carton_still_moving_or_held_does_not_count():
    moving = goal()
    moving["carton_4"] = second((BX, BY - P), vel=(0.0, 0.0, -0.05))
    assert TASK.score(moving) == pytest.approx(0.7) and not TASK.success(moving)
    held = goal()
    held["carton_4"] = carton((BX, BY - P), T.BOARD_TOP + 2 * H + 0.03)
    assert TASK.score(held) == pytest.approx(0.7) and not TASK.success(held)


# -- construct validity: shortcuts do not earn full credit ------------------------------------------------------------


def test_a_single_layer_never_scores_more_than_0_4():
    three = scene(
        carton_1=bottom((BX - 0.08, BY), yaw_deg=90.0),
        carton_2=bottom((BX, BY), yaw_deg=90.0),
        carton_3=bottom((BX + 0.08, BY), yaw_deg=90.0),
    )
    assert TASK.score(three) == pytest.approx(0.4)
    four_off_board = scene(
        carton_1=bottom((BX + P, BY)),
        carton_3=bottom((BX - P, BY)),
        carton_2=carton((0.25, 0.30), TOP),
        carton_4=carton((0.25, -0.30), TOP, yaw_deg=90.0),
    )
    assert TASK.score(four_off_board) == pytest.approx(0.4)


def test_a_column_stack_scores_at_most_0_6_whichever_way_round():
    for yaw in (0.0, 90.0):
        col = (P, 0.0) if yaw == 90.0 else (0.0, P)
        objects = scene(
            carton_1=bottom((BX + col[0], BY + col[1]), yaw_deg=yaw),
            carton_3=bottom((BX - col[0], BY - col[1]), yaw_deg=yaw),
            carton_2=second((BX + col[0], BY + col[1]), yaw_deg=yaw),
            carton_4=second((BX - col[0], BY - col[1]), yaw_deg=yaw),
        )
        assert TASK.score(objects) == pytest.approx(0.6), yaw
        assert not TASK.success(objects)


def test_a_tower_or_a_bridge_of_two_cartons_is_not_the_pattern():
    tower = scene(
        carton_1=bottom((BX, BY)),
        carton_2=second((BX, BY), yaw_deg=0.0),
        carton_3=carton((BX, BY), T.BOARD_TOP + 2 * H, 90.0),
        carton_4=carton((BX, BY), T.BOARD_TOP + 3 * H, 0.0),
    )
    # one first-layer carton, and a turned carton on it that bridges nothing: 0.2 + 0.1; the two above count nothing
    assert TASK.score(tower) == pytest.approx(0.3)


def test_a_tipped_carton_or_one_off_the_table_zeroes_the_score():
    tipped = goal()
    tipped["carton_4"] = carton((BX, BY - P - 0.02), T.BOARD_TOP, 0.0, tilt=35.0)
    assert TASK.score(tipped) == 0.0 and not TASK.success(tipped)
    on_its_side = goal()
    on_its_side["carton_4"] = carton((0.25, -0.30), TOP, 0.0, tilt=90.0)
    assert TASK.score(on_its_side) == 0.0
    on_end = goal()
    on_end["carton_4"] = carton((0.25, -0.30), TOP, 0.0, tilt=90.0, tilt_axis=(0, 1, 0))
    assert TASK.score(on_end) == 0.0
    fallen = goal()
    fallen["carton_4"] = carton((0.25, -0.70), 0.0)
    assert TASK.score(fallen) == 0.0
    leaning = goal()
    leaning["carton_2"] = carton((BX, BY + P), T.BOARD_TOP + H, 0.0, tilt=15.0)
    assert TASK.score(leaning) == pytest.approx(0.7)  # leaning 10-20 deg: no credit for it, but no collapse


def test_score_stays_in_range_and_success_matches_a_full_score_over_random_states():
    rng = np.random.default_rng(0)
    values = set()
    for _ in range(400):
        objects = scene(
            **{
                n: carton(
                    rng.uniform((0.16, -0.09), (0.34, 0.09)),
                    T.BOARD_TOP + H * rng.integers(0, 3),
                    rng.choice([0.0, 90.0]) + rng.uniform(-12, 12),
                    tilt=rng.choice([0.0, 0.0, 0.0, rng.uniform(0, 25)]),
                )
                for n in T.NAMES
            }
        )
        s = TASK.score(objects)
        values.add(s)
        assert 0.0 <= s <= 1.0
        assert TASK.success(objects) == (s == 1.0)
    assert len(values) > 3
