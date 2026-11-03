"""Checks of counterweight_overhang.py's scoring and demo, without the simulator. Needs yam_arena (the official stack):

    python -m pytest -q test_counterweight_overhang.py
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import station
from yam_arena.demo import ArmPlan
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.task_file import ObjectState, load

TASK = load(Path(__file__).with_name("counterweight_overhang.py"))
M = TASK.module


def _rot_x(t):
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])


def _rot_y(t):
    return np.array([[math.cos(t), 0.0, math.sin(t)], [0.0, 1.0, 0.0], [-math.sin(t), 0.0, math.cos(t)]])


def _rot_z(s):
    return np.array([[math.cos(s), -math.sin(s), 0.0], [math.sin(s), math.cos(s), 0.0], [0.0, 0.0, 1.0]])


def _state(centre, half, rot, vel=(0.0, 0.0, 0.0)):
    """An object's state, its bounding box built the way Arena 0.3.1 builds it: the turned corners of its local box."""
    corners = np.array([[i * half[0], j * half[1], k * half[2]] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    world = corners @ rot.T + np.asarray(centre, dtype=float)
    return ObjectState(pos=np.asarray(centre, dtype=float), quat_xyzw=np.asarray(matrix_to_quat_xyzw(rot)),
                       lin_vel=np.asarray(vel, dtype=float), ang_vel=np.zeros(3),
                       low=world.min(axis=0), high=world.max(axis=0))


def plank(tilt_deg=0.0, yaw_deg=0.0, centre_xy=None):
    """The plank on the ledge, turned tilt_deg about the ledge edge (far end down); or, level, centred at centre_xy
    and turned yaw_deg about the vertical."""
    half = (M.PLANK_W / 2, M.PLANK_LEN / 2, M.PLANK_T / 2)
    if centre_xy is not None:
        return _state((*centre_xy, M.TOP + M.LEDGE_H + M.PLANK_T / 2), half, _rot_z(math.radians(yaw_deg)))
    rot = _rot_x(math.radians(tilt_deg))
    pivot = np.array([M.ROW_X, M.EDGE_Y, M.TOP + M.LEDGE_H])
    centre = pivot + rot @ np.array([0.0, M.PLANK_Y - M.EDGE_Y, M.PLANK_T / 2])
    return _state(centre, half, rot)


def tin(x, y, base_z, tilt_deg=0.0, spin_deg=0.0, vz=0.0):
    """A tin whose base is centred at (x, y), its lowest point at base_z, spun on its axis, then tilted about y."""
    t = math.radians(tilt_deg)
    rot = _rot_y(t) @ _rot_z(math.radians(spin_deg))
    centre = np.array([x, y, base_z + M.TIN_R * math.sin(t)]) + rot[:, 2] * M.TIN_H / 2
    return _state(centre, (M.TIN_R, M.TIN_R, M.TIN_H / 2), rot, (0.0, 0.0, vz))


def block(x, y, base_z, vz=0.0):
    return _state((x, y, base_z + M.BLOCK_SIZE / 2), (M.BLOCK_SIZE / 2,) * 3, np.eye(3), (0.0, 0.0, vz))


def tin_on_plank(from_far_end=0.045, side=0.0, **kw):
    return tin(M.ROW_X + side, M.PLANK_FAR_Y + from_far_end, M.PLANK_TOP, **kw)


def block_on_plank(y=M.BLOCK_SPOT_Y):
    return block(M.ROW_X, y, M.PLANK_TOP)


def scene(plank_state=None, tin_state=None, block_state=None):
    return {"plank": plank_state or plank(), "tin": tin_state or tin(*M.TIN_XY, M.TOP),
            "block": block_state or block(*M.BLOCK_XY, M.TOP)}


def delivered():
    return scene(tin_state=tin_on_plank(), block_state=block_on_plank())


def step(objects):
    """One control step as the simulator scores it: success(), then score(), on the same state."""
    return M.success(objects), M.score(objects)


@pytest.fixture(autouse=True)
def new_trial():
    """Each test starts a trial the way the simulator does: every object at its start, which clears the fall flag."""
    assert step(scene()) == (False, 0.0)


def test_the_unloaded_plank_holds_itself_up():
    assert M._balance(scene()) == pytest.approx(0.04)


@pytest.mark.parametrize("from_far_end", np.linspace(M.ZONE_NEAR, M.ZONE_FAR, 6))
def test_the_tin_alone_tips_the_plank_from_every_spot_in_the_zone(from_far_end):
    """Its centre of mass lands at least 3 cm past the edge: far beyond what sim noise could flip."""
    assert M._balance(scene(tin_state=tin_on_plank(from_far_end))) < -0.03


@pytest.mark.parametrize("from_far_end", np.linspace(M.ZONE_NEAR, M.ZONE_FAR, 6))
def test_the_demo_block_holds_the_tin_anywhere_in_the_zone_with_room_to_spare(from_far_end):
    assert M._balance(scene(tin_state=tin_on_plank(from_far_end), block_state=block_on_plank())) > 0.05


def test_the_block_behind_and_the_tin_on_the_far_end_succeeds():
    objects = delivered()
    assert step(objects) == (True, 1.0)
    assert M.distance(objects) < 0.01


def test_a_tin_on_the_far_end_of_a_propped_plank_is_not_success():
    """The plank shows level only because a gripper props it, or still holds the tin: no block on the plank."""
    assert step(scene(tin_state=tin_on_plank())) == (False, 0.0)


def test_a_plank_turned_past_the_stop_scores_0_with_no_block():
    """Found in review: turned 16 degrees and pulled toward the arms, the plank's left end passes beside the stop and
    more of it sits on the ledge, so the tin alone balances on it."""
    turned = plank(yaw_deg=16.0, centre_xy=(0.225, 0.355))
    rot = _rot_z(math.radians(16.0))
    base = np.array([0.225, 0.355, 0.0]) - rot[:, 1] * (M.PLANK_LEN / 2 - 0.075)  # 7.5 cm from its far end
    objects = scene(turned, tin_state=tin(base[0], base[1], M.PLANK_TOP))
    assert M._balance(objects) >= M.BALANCE_MARGIN  # it really would balance
    assert step(objects) == (False, 0.0)
    assert step(scene(turned, block_state=block(0.225, 0.42, M.PLANK_TOP))) == (False, 0.0)  # no half credit either


def test_a_plank_pushed_back_against_the_stop_is_still_in_place():
    pushed = plank(centre_xy=(M.ROW_X, M.PLANK_Y + 0.003))
    tin_state = tin(M.ROW_X, M.PLANK_FAR_Y + 0.003 + 0.045, M.PLANK_TOP)
    assert step(scene(pushed, tin_state=tin_state, block_state=block_on_plank())) == (True, 1.0)


def test_a_tin_or_block_held_just_above_the_plank_does_not_rest_on_it():
    held_tin = tin(M.ROW_X, M.PLANK_FAR_Y + 0.045, M.PLANK_TOP + 0.005)
    assert step(scene(tin_state=held_tin, block_state=block_on_plank())) == (False, 0.5)
    assert step(scene(block_state=block(M.ROW_X, M.BLOCK_SPOT_Y, M.PLANK_TOP + 0.005))) == (False, 0.0)


def test_tipping_then_rebuilding_still_scores_0():
    step(scene(plank(18.0), tin_state=tin_on_plank()))  # tin first: the far end lands on the table
    # The robot lifts the tin off (the plank levels by itself), sets both back near their starts, then does it right
    step(scene(tin_state=tin(M.TIN_XY[0], M.TIN_XY[1] - 0.02, M.TOP), block_state=block(*M.BLOCK_XY, M.TOP)))
    step(scene(tin_state=tin(M.TIN_XY[0], M.TIN_XY[1] - 0.02, M.TOP), block_state=block_on_plank()))
    assert step(delivered()) == (False, 0.0)


def test_the_fall_flag_clears_when_the_next_trial_starts():
    step(scene(plank(18.0), tin_state=tin_on_plank()))
    assert step(delivered()) == (False, 0.0)
    step(scene())  # the reset puts every object back
    assert step(delivered()) == (True, 1.0)


def test_putting_only_the_tin_back_does_not_clear_the_fall_flag():
    step(scene(plank(18.0), tin_state=tin_on_plank()))
    step(scene(block_state=block(M.BLOCK_XY[0], M.BLOCK_XY[1] + 0.03, M.TOP)))  # tin back at its start, block not
    assert step(delivered()) == (False, 0.0)


def test_a_tin_put_back_2_mm_off_its_mark_after_a_tip_leaves_the_flag_set():
    """Found in review: with a loose start check, tin first, tip, tin back on its mark, then the right order scored 1.0."""
    step(scene(plank(18.0), tin_state=tin_on_plank()))
    step(scene(tin_state=tin(M.TIN_XY[0] + 0.002, M.TIN_XY[1], M.TOP)))
    assert step(delivered()) == (False, 0.0)


def test_the_block_far_enough_back_earns_half_credit_and_too_near_the_edge_none():
    assert step(scene(block_state=block_on_plank())) == (False, 0.5)
    assert step(scene(block_state=block_on_plank(M.EDGE_Y + 0.04))) == (False, 0.0)


@pytest.mark.parametrize(("from_far_end", "side", "inside"), [
    (M.ZONE_NEAR + 0.002, 0.0, True),
    (M.ZONE_FAR - 0.002, 0.0, True),
    (0.045, M.ZONE_SIDE - 0.002, True),
    (M.ZONE_NEAR - 0.005, 0.0, False),  # hanging past the end
    (M.ZONE_FAR + 0.005, 0.0, False),
    (0.045, M.ZONE_SIDE + 0.005, False),
])
def test_the_target_zone_edges(from_far_end, side, inside):
    objects = scene(tin_state=tin_on_plank(from_far_end, side), block_state=block_on_plank())
    assert step(objects) == ((True, 1.0) if inside else (False, 0.5))


@pytest.mark.parametrize("fall", ["tin on its side", "tin dropped", "block dropped"])
def test_a_tin_knocked_over_or_anything_dropped_scores_0_for_the_trial(fall):
    if fall == "tin on its side":
        objects = scene(tin_state=tin(0.16, 0.40, M.TOP, tilt_deg=90.0))
    elif fall == "tin dropped":
        objects = scene(tin_state=tin(0.16, 0.40, M.TOP + 0.03, vz=-1.2))
    else:
        objects = scene(block_state=block(0.16, 0.25, M.TOP + 0.03, vz=-1.2))
    assert step(objects) == (False, 0.0)
    assert step(delivered()) == (False, 0.0)


def test_a_slight_lean_is_not_a_fall_but_a_tip_is():
    assert step(scene(plank(2.0))) == (False, 0.0)  # not level, so no credit, but not a fall
    assert step(delivered()) == (True, 1.0)
    step(scene(plank(4.0)))
    assert step(delivered()) == (False, 0.0)


def test_a_tin_spinning_on_its_axis_is_scored_from_its_axis():
    """Its bounding box swings out as it spins (by 41% of its radius at 45 degrees); its base centre doesn't move."""
    objects = scene(tin_state=tin_on_plank(spin_deg=45.0), block_state=block_on_plank())
    assert step(objects) == (True, 1.0)


def test_the_stop_blocks_the_plank_and_the_loose_objects_start_clear_of_the_ledge():
    assert 0.0 < M.STOP_Y[0] - (M.PLANK_Y + M.PLANK_LEN / 2) <= 0.005
    ledge_near_x = M.ROW_X - M.LEDGE_DEPTH / 2
    assert ledge_near_x - (M.TIN_XY[0] + M.TIN_R) > 0.03
    assert ledge_near_x - (M.BLOCK_XY[0] + M.BLOCK_SIZE / 2) > 0.03


def test_the_demo_is_within_reach():
    plan = ArmPlan(M.demo()["left"], station.BASES["left"])
    for t in np.arange(0.0, plan.duration, 0.05):
        plan.action7(t)
    assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (plan.worst_ik_mm, plan.worst_ik_deg)
