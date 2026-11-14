"""Offline checks (no GPU): the task file loads, the bag's walls leave the cavity the board needs, the scorer covers
each outcome (with the bag lying straight, turned and tilted, held, hanging over the crate's wall, not released), the
contact monitor's release rule, and the demo's grips, depths and reach. Run in the yam_arena environment:
python -m pytest -q test_board_into_antistatic_bag.py"""

from __future__ import annotations

import math
import pathlib

import numpy as np
import pytest
from yam_arena.demo import FINGER_OPEN, ArmPlan
from yam_arena.station import BASES
from yam_arena.task_file import ObjectState, load

HERE = pathlib.Path(__file__).resolve().parent
TASK = load(HERE / "board_into_antistatic_bag.py")
T = TASK.module

BAG0 = np.asarray(T.BAG.pos)
BOARD0 = np.asarray(T.BOARD.pos)
L = T.BAG_SIZE[1]
MOUTH_LOCAL = -L / 2


def _quat(yaw=0.0, roll=0.0):
    """xyzw of a turn ``yaw`` about z after a tilt ``roll`` about x."""
    qz = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    qx = np.array([math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


@pytest.fixture(autouse=True)
def grippers_away(monkeypatch):
    """By default no gripper touches the bag or the board (the contact monitor needs the simulator)."""
    monkeypatch.setattr(T.contact, "released", lambda: True)


def _state(pos, quat=(0.0, 0.0, 0.0, 1.0), vel=(0.0, 0.0, 0.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=pos - 0.05, high=pos + 0.05)


def _scene(bag_pos=BAG0, bag_quat=(0, 0, 0, 1), board_local=None, board_pos=BOARD0, bag_vel=(0, 0, 0),
           board_yaw=0.0, board_vel=(0, 0, 0)):
    """The bag where given, and the board at ``board_local`` (its centre in the bag's frame, lying in it) or at
    ``board_pos`` in the world."""
    rot = T._rot(bag_quat)
    if board_local is not None:
        board_pos = np.asarray(bag_pos) + rot @ np.asarray(board_local, float)
        qb = _quat_mul(np.asarray(bag_quat, float), _quat(board_yaw))
    else:
        qb = _quat(board_yaw)
    return {"bag": _state(bag_pos, bag_quat, bag_vel), "board": _state(board_pos, qb, board_vel)}


def _quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def _local(rear_past_mouth, x=0.0, z=None):
    """The board's centre in the bag's frame with its near end this far past the mouth, lying on the floor."""
    if z is None:
        z = -T.CAVITY[2] / 2 + T.BOARD_SIZE[2] / 2
    return (x, MOUTH_LOCAL + rear_past_mouth + T.BOARD_SIZE[1] / 2, z)


FULL = _local(0.004)  # pushed in, near end 4 mm past the mouth
ON_FLOOR_Z = T.CRATE_FLOOR + T.BAG_SIZE[2] / 2  # the bag's centre when it lies flat on the crate's floor
IN_CRATE = np.array([*T.CRATE_XY, ON_FLOOR_Z])
POSES = {
    "straight": (0.0, 0.0),
    "turned": (2.0, 0.0),  # turned 115 degrees about the vertical
    "upside down": (0.7, math.pi),  # flipped over
    "on edge": (-1.0, math.pi / 2),
}
LYING = {  # the poses a bag can rest in on the crate's floor: flat, either face up, turned a little
    "straight": (0.0, 0.0),
    "turned": (0.3, 0.0),
    "upside down": (0.4, math.pi),
}


def test_loads():
    assert TASK.scene == "station"
    assert set(TASK.free_objects) == {"bag", "board"}


def test_bag_walls_leave_the_cavity():
    cw, cd, ch = T.CAVITY
    assert (cw, cd, ch) == pytest.approx((0.073, 0.148, 0.008))
    assert ch - T.BOARD_SIZE[2] >= 0.004 - 1e-9  # 2 mm of play above and below the board
    assert cw - T.BOARD_SIZE[0] >= 0.008 - 1e-9  # 4 mm each side
    assert 0 < cd - T.BOARD_SIZE[1] < T.NEAR_END  # fully in means the far edge is within 1 cm of the end
    half = np.asarray(T.BAG_SIZE) / 2
    for name, centre, size in T.BAG.parts():
        lo, hi = np.asarray(centre) - np.asarray(size) / 2, np.asarray(centre) + np.asarray(size) / 2
        assert np.all(lo >= -half - 1e-9) and np.all(hi <= half + 1e-9), name  # inside the bag's outside
        # nothing in the cavity
        cav_lo = np.array([-cw / 2, MOUTH_LOCAL, -ch / 2])
        cav_hi = np.array([cw / 2, MOUTH_LOCAL + cd, ch / 2])
        overlap = np.minimum(hi, cav_hi) - np.maximum(lo, cav_lo)
        assert np.any(overlap <= 1e-9), name


def test_notch_lets_the_closed_fingers_through():
    nw, nd = T.BAG.notch
    assert nw >= 0.0126 + 2 * 0.008  # closed pads are 12.6 mm across, 8 mm of play each side
    pads_front = T.PUSH_S + T.PAD_TIP_HALF  # along the line from the mouth
    assert 0 < pads_front < nd  # the pads stop inside the notch
    assert T.PUSH_Z - T.PAD_TIP_PAST_TCP > T.BAG_FLOOR  # the pad tips ride above the floor


def test_friction_drags_an_unheld_bag():
    """Pushing the board along the bag's floor pulls the bag harder than the mat holds it."""
    drag = 1.0 * T.BOARD_MASS  # the board on the bag's inside: yam_arena's dynamic friction
    hold = T.MAT_FRICTION * (T.BAG_MASS + T.BOARD_MASS)
    assert drag > 1.2 * hold


def test_start_scores_zero():
    assert T.score(_scene()) == 0.0
    assert not T.success(_scene())


@pytest.mark.parametrize("pose", POSES)
def test_ladder_with_the_bag_turned(pose):
    yaw, roll = POSES[pose]
    q = _quat(yaw, roll)
    on_mat = BAG0 + (0.05, 0.02, 0.07)  # held up, so a bag on edge keeps the board above the table
    assert T.score(_scene(on_mat, q, _local(-0.08))) == 0.0  # the centre 1 cm short of the mouth
    assert T.score(_scene(on_mat, q, _local(-0.06))) == 0.25  # the centre 1 cm past the mouth
    assert T.score(_scene(on_mat, q, _local(-0.002))) == 0.25  # near end 2 mm out
    assert T.score(_scene(on_mat, q, FULL)) == 0.5
    assert T.score(_scene(IN_CRATE + (0, 0, 0.07), q, FULL)) == 0.5  # held above the crate's floor
    assert T.score(_scene(IN_CRATE + (0, 0, 0.07), q, _local(-0.03))) == 0.25  # over the crate, the board half in


@pytest.mark.parametrize("pose", LYING)
def test_full_score_with_the_bag_lying_in_the_crate(pose):
    yaw, roll = LYING[pose]
    scene = _scene(IN_CRATE, _quat(yaw, roll), FULL)
    assert T.bag_in_crate(scene)
    assert T.success(scene) and T.score(scene) == 1.0


def test_bag_on_edge_in_the_crate_is_not_full():
    scene = _scene(IN_CRATE + (0, 0, 0.02), _quat(-1.0, math.pi / 2), FULL)  # on its long edge, in the crate
    assert not T.bag_in_crate(scene) and T.score(scene) == 0.5


def test_far_edge_against_the_end():
    deep = _local(T.CAVITY[1] - T.BOARD_SIZE[1])
    assert T.score(_scene(IN_CRATE, board_local=deep)) == 1.0


def test_board_beside_or_on_the_bag_is_not_in():
    assert T.score(_scene(board_local=(T.CAVITY[0] / 2 + 0.01, 0.0, 0.0))) == 0.0  # next to it
    on_top = (0.0, 0.0, T.BAG_SIZE[2] / 2 + T.BOARD_SIZE[2] / 2)
    assert T.score(_scene(IN_CRATE, board_local=on_top)) == 0.0
    under = (0.0, 0.0, -T.BAG_SIZE[2] / 2 - T.BOARD_SIZE[2] / 2)
    assert T.score(_scene(IN_CRATE, board_local=under)) == 0.0


def test_board_turned_too_far_is_not_fully_in():
    assert T.score(_scene(board_local=FULL, board_yaw=0.3)) < 0.5


def test_bag_carried_or_beside_the_crate():
    held = _scene(IN_CRATE + (0, 0, 0.10), board_local=FULL)  # above the rim
    assert T.score(held) == 0.5
    moving = _scene(IN_CRATE, board_local=FULL, bag_vel=(0.05, 0.0, 0.0))
    assert not T.success(moving) and T.score(moving) == 0.5
    board_moving = _scene(IN_CRATE, board_local=FULL, board_vel=(0.05, 0.0, 0.0))
    assert not T.success(board_moving) and T.score(board_moving) == 0.5
    beside = _scene(IN_CRATE + (0.0, 0.14, 0.0), board_local=FULL)  # past the crate's end
    assert T.score(beside) == 0.5


# -- what the reviewers named: the centre and the speed are not enough ---------------------------------------------


def test_filled_bag_still_held_in_the_crate_is_not_released(monkeypatch):
    """The bag lies in the crate and is still, but a gripper still touches it: 0.5, not 1.0."""
    scene = _scene(IN_CRATE, board_local=FULL)
    assert T.score(scene) == 1.0
    monkeypatch.setattr(T.contact, "released", lambda: False)
    assert not T.success(scene) and T.score(scene) == 0.5


def test_bag_held_still_above_the_crate_floor_is_not_resting_on_it():
    """Held still 2 cm up, 1 cm up, 4 mm up: inside the walls and below the rim, but not on the floor."""
    for up in (0.02, 0.01, 0.004):
        scene = _scene(IN_CRATE + (0, 0, up), board_local=FULL)
        assert not T.bag_in_crate(scene) and T.score(scene) == 0.5, up
    assert T.score(_scene(IN_CRATE + (0, 0, 0.002), board_local=FULL)) == 1.0  # 2 mm up is on the floor


def test_bag_hanging_over_the_crate_wall_is_not_inside():
    """Its centre is over the crate's floor (the old test passed up to 11.2 cm along y), but its end is over the
    wall."""
    for dy in (0.05, 0.08, 0.11):
        scene = _scene(IN_CRATE + (0.0, dy, 0.0), board_local=FULL)
        assert not T.bag_in_crate(scene) and T.score(scene) == 0.5, dy
    wide = _scene(IN_CRATE + (0.06, 0.0, 0.0), board_local=FULL)  # 4 cm over a long wall
    assert not T.bag_in_crate(wide) and T.score(wide) == 0.5
    assert T.score(_scene(IN_CRATE + (0.0, 0.04, 0.0), board_local=FULL)) == 1.0  # 4 cm along y still fits


def test_bag_leaning_on_the_wall_is_not_flat_on_the_floor():
    leaning = _scene(IN_CRATE + (0, 0, 0.03), _quat(0.0, 0.5), FULL)  # tilted 29 degrees
    assert not T.bag_in_crate(leaning) and T.score(leaning) == 0.5


def test_bag_on_the_rim_is_not_inside():
    scene = _scene((T.CRATE_XY[0], T.CRATE_XY[1], T.CRATE_RIM + T.BAG_SIZE[2] / 2), board_local=FULL)
    assert not T.bag_in_crate(scene) and T.score(scene) == 0.5


def test_fully_bagged_board_elsewhere_keeps_half():
    for pos in (BAG0 + (0.0, 0.0, 0.05), np.array([0.2, -0.1, T.TOP + 0.01]), IN_CRATE + (0.0, 0.14, 0.0)):
        assert T.score(_scene(pos, board_local=FULL)) == 0.5


# -- the contact monitor's release rule -------------------------------------------------------------------------


@pytest.fixture
def monitor(monkeypatch):
    """A working contact monitor with 24 substeps (0.1 s) read, and the real ``released``."""
    monkeypatch.undo()  # the autouse fixture's stub
    mon = {"view": object(), "stage": None, "n": 240, "last": None, "dt": 1 / 240, "polls": 240, "err": None}
    monkeypatch.setattr(T.contact, "_MON", mon)
    return mon


def test_released_needs_a_working_monitor(monitor):
    assert T.contact.released()
    monitor["err"] = "boom"
    assert not T.contact.released()  # fails closed
    monitor.update(err=None, view=None)
    assert not T.contact.released()
    monitor.update(view=object(), polls=0)
    assert not T.contact.released()


def test_released_after_a_touch_waits_out_the_window(monitor):
    monitor["last"] = 240  # touched in the latest substep
    assert not T.contact.released()
    monitor["n"] = 240 + int(T.contact.RELEASE_S * 240) - 1
    assert not T.contact.released()
    monitor["n"] = 240 + int(T.contact.RELEASE_S * 240)
    assert T.contact.released()


def test_the_monitor_reads_every_robot_link_and_both_bodies():
    assert T.contact.HELD == ("bag", "board")
    assert T.contact.TOUCH_SEP <= 0.001  # contact, not "near"


# -- the rest ----------------------------------------------------------------------------------------------------------


def test_fell_off_the_table():
    assert T.score(_scene(board_local=FULL, bag_pos=(0.3, 0.6, T.TOP - 0.5))) == 0.0
    assert T.score(_scene(board_pos=(0.3, -0.6, T.TOP - 0.5))) == 0.0


def test_non_finite_zero():
    assert T.score(_scene(board_pos=(math.nan, 0.0, 0.0))) == 0.0


# -- the demo --------------------------------------------------------------------------------------------------------


def _at(steps, label):
    """When the waypoint ``label`` is reached (s from the start) and the waypoint."""
    t = 0.0
    for w in steps:
        t += w.duration
        if w.label == label:
            return t, w
    raise KeyError(label)


def test_demo_order():
    plan = T.demo()
    held, _ = _at(plan["left"], "hold the bag")
    started, _ = _at(plan["right"], "lift to the mouth")
    pushed, _ = _at(plan["right"], "push it in")
    away, _ = _at(plan["right"], "away")
    lifted, _ = _at(plan["left"], "lift")
    assert held < started and pushed < away <= T.RIGHT_DONE < lifted


def test_demo_ends_with_both_hands_letting_go_in_the_crate():
    plan = T.demo()
    for arm in ("left", "right"):
        assert plan[arm][-1].finger == FINGER_OPEN, arm  # both hands open at the end
    open_at, _ = _at(plan["left"], "let go")
    lifted, _ = _at(plan["left"], "up")
    assert open_at < lifted
    let_go = next(w for w in plan["left"] if w.label == "let go")
    assert let_go.pos[2] - T.BAG_BELOW_TCP < T.CRATE_RIM  # the bag is let go below the crate's rim
    assert plan["left"][-1].pos[2] > T.CRATE_RIM + T.BAG_SIZE[2]  # the left hand ends clear of the bag
    # the right hand ends well away from the bag, which lies in the crate
    right_end = np.asarray(plan["right"][-1].pos[:2])
    assert np.hypot(*(right_end - np.asarray(T.CRATE_XY))) > 0.2


def test_demo_lays_the_bag_inside_the_crate_walls():
    """Lowered along y over the crate's centre, the bag's whole outline is 4 cm or more inside the walls."""
    along_y = np.array([T.BAG_SIZE[0], T.BAG_SIZE[1]]) / 2
    # the left hand holds the closed end, so the bag's centre is 6.4 cm toward -y from the hand: over the crate's
    centre = np.asarray(T.PLACE_XY) - np.array([0.0, T.HOLD_S - T.BAG_SIZE[1] / 2])
    assert centre == pytest.approx(np.asarray(T.CRATE_XY))
    assert np.all(np.abs(centre - np.asarray(T.CRATE_XY)) + along_y <= np.asarray(T.CRATE_INNER) / 2 - 0.04 + 1e-9)


def test_demo_board_rides_mid_cavity():
    bottom = T.MAT_TOP + T.LIFT
    assert bottom - T.BAG_FLOOR == pytest.approx((T.CAVITY[2] - T.BOARD_SIZE[2]) / 2)
    assert T.INSERT_S + T.PAD_HALF < 0  # the pads stop short of the mouth
    pushed_in = T.PUSH_S + T.PAD_TIP_HALF  # the near end past the mouth after the push
    assert 0.003 < pushed_in < T.CAVITY[1] - T.BOARD_SIZE[1]


def test_demo_grips_fit():
    assert T.BOARD_SIZE[0] < 0.094 - 0.02 and T.BAG_SIZE[0] < 0.094 - 0.015  # both fit the open fingers
    assert T.PLACE_Z - T.BAG_BELOW_TCP > T.TOP + 0.01
    assert T.CARRY_Z - T.BAG_BELOW_TCP > T.CRATE_RIM + 0.03


def test_demo_keeps_the_arms_apart():
    """Each arm works in its own half, and while the left arm holds the bag the right arm's TCP stays 13 cm or more
    from it (its wrist camera points away from the left arm, the left arm's toward the right one)."""
    plan = T.demo()
    assert max(w.pos[1] for w in plan["right"]) < -0.015
    assert min(w.pos[1] for w in plan["left"]) > 0.09
    hold = np.asarray(T.on_line(T.HOLD_S))
    assert min(np.hypot(*(np.asarray(w.pos[:2]) - hold)) for w in plan["right"]) > 0.13
    assert math.degrees(T.GRIP_YAW) == pytest.approx(-60.0)


@pytest.mark.parametrize("arm", ["left", "right"])
def test_demo_reachable(arm):
    plan = ArmPlan(T.demo()[arm], BASES[arm])
    for k in range(int(plan.duration * 20) + 1):
        plan.action7(k / 20)
    assert plan.worst_ik_mm < 2.0 and plan.worst_ik_deg < 2.0, (arm, plan.worst_ik_mm, plan.worst_ik_deg)


# -- the meshes --------------------------------------------------------------------------------------------------------


def test_every_dressed_box_exists_and_its_mesh_is_there():
    names = {o.name for o in T.OBJECTS}
    for name, file in T.LOOKS.items():
        assert name in names
        assert (HERE / "looks" / file).is_file()


@pytest.mark.parametrize("name", ["bag", "board"])
def test_scored_meshes_fit_in_their_boxes(name):
    """The bag's and the board's meshes stay inside their boxes, so the centre the scorer reads is the box's."""
    pytest.importorskip("pxr.Usd")
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(HERE / "looks" / T.LOOKS[name]))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    box = cache.ComputeWorldBound(stage.GetDefaultPrim()).ComputeAlignedRange()
    half = np.asarray({"bag": T.BAG_SIZE, "board": T.BOARD_SIZE}[name]) / 2
    assert np.all(np.asarray(box.GetMax()) <= half + 1e-5)
    assert np.all(np.asarray(box.GetMin()) >= -half - 1e-5)
