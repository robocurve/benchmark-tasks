"""Offline checks of task.py with yam_arena's own loader and IK (no Isaac Sim, no physics).

These check the task file's layout arithmetic, its scoring of hand-built object states, and that the scripted demo
stays inside the arms' reach and joint limits. They do not show that the demo succeeds in the simulator: only an
actual yam_scripted run in Isaac Lab-Arena does that.

    pytest -q test_sliding_door.py        # with yam_arena importable (the walkthrough's Arena environment)
"""

import math
from pathlib import Path

import numpy as np
import pytest

from yam_arena import ik, station
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import ArmPlan, Waypoint
from yam_arena.task_file import TABLE_TOP, ObjectState, load

TASK = load(Path(__file__).resolve().with_name("task.py"))
M = TASK.module
CONTROL_HZ = 20.0  # the rate the submission runs at (-E control_hz=20)
RUN_LIMIT_STEPS = 1200
SCRIPTED_HOLD_S = 3.0  # yam_scripted holds the last pose this long before asking to stop


def state(pos, size, vel=(0.0, 0.0, 0.0), lean_deg=0.0):
    """An object at ``pos`` (bounding-box centre), leaning ``lean_deg`` about the world x axis."""
    pos, half = np.asarray(pos, dtype=float), np.asarray(size, dtype=float) / 2
    a = math.radians(lean_deg) / 2
    quat = np.array([math.sin(a), 0.0, 0.0, math.cos(a)])
    return ObjectState(pos, quat, np.asarray(vel, dtype=float), np.zeros(3), pos - half, pos + half)


def door_at(y, z_lift=0.0, x=M.TRACK_X, **kw):
    return state((x, y, M.FLOOR + M.DOOR_SIZE[2] / 2 + z_lift), M.DOOR_SIZE, **kw)


def block_at(x, y, z_bottom, **kw):
    return state((x, y, z_bottom + M.BLOCK_SIZE[2] / 2), M.BLOCK_SIZE, **kw)


CLOSED = M.DOOR_START_Y - M.SPAWN_GAP  # settled against the right end stop
BLOCK_IN_TRACK = block_at(M.TRACK_X, M.BLOCK_START_Y + M.SPAWN_GAP, M.FLOOR)
BLOCK_ON_TABLE = block_at(0.16, 0.40, TABLE_TOP["station"])
FULL = M.DOOR_END_Y  # where the demo sets the door down


def outcome(door, block):
    objects = {"door": door, "block": block}
    return M.score(objects), M.success(objects)


def test_the_task_file_loads_on_the_station_with_two_loose_parts():
    assert (TASK.scene, TASK.table_top) == ("station", TABLE_TOP["station"])
    assert TASK.free_objects == ("door", "block")
    fixed = {"track_base", "track_lip_far", "track_lip_near", "left_stop", "right_stop"}
    assert set(TASK.names) == fixed | {"door", "block"}
    assert all(callable(f) for f in (TASK.success, TASK.distance, TASK.score, TASK.demo))
    assert TASK.score_over == "end" and TASK.success_hold_s == 1.0
    assert TASK.docs.startswith("On the station table")


def test_the_parts_fit_the_channel_and_the_block_really_stops_the_door():
    clearance = (M.GAP - M.DOOR_SIZE[0]) / 2
    assert 0.001 <= clearance <= 0.002 and (M.GAP - M.BLOCK_SIZE[0]) / 2 == pytest.approx(clearance)
    assert M.BLOCK_SIZE[2] - M.LIP_H >= 0.025  # enough sticks up to pinch above the lips
    # With the block against the left end stop, the door's left edge can get no further than the block.
    furthest_left_edge = M.TRACK_Y + M.STOP_IN - M.BLOCK_SIZE[1]
    assert M.TRACK_Y + M.STOP_IN - furthest_left_edge >= M.FULL_OPEN_GAP + 0.05  # never "fully open" by forcing
    assert furthest_left_edge - M.DOOR_START_LEFT >= M.PARTIAL_TRAVEL  # forcing still earns the partial credit
    # The demo's end position counts as fully open.
    assert M.DOOR_END_Y + M.DOOR_SIZE[1] / 2 >= M.TRACK_Y + M.STOP_IN - M.FULL_OPEN_GAP


def test_the_starting_layout_scores_zero():
    assert outcome(door_at(CLOSED), BLOCK_IN_TRACK) == (0.0, False)
    assert M.distance({"door": door_at(CLOSED), "block": BLOCK_IN_TRACK}) == pytest.approx(0.40, abs=1e-3)


def test_forcing_the_door_against_the_block_only_earns_partial_credit():
    against_block = M.TRACK_Y + M.STOP_IN - M.BLOCK_SIZE[1] - M.DOOR_SIZE[1] / 2
    assert outcome(door_at(against_block), BLOCK_IN_TRACK) == (0.25, False)


def test_clearing_the_track_is_worth_more_than_forcing():
    assert outcome(door_at(CLOSED), BLOCK_ON_TABLE) == (0.5, False)


def test_fully_open_with_the_block_on_the_table_and_everything_still_is_full_score():
    assert outcome(door_at(FULL), BLOCK_ON_TABLE) == (1.0, True)


def test_a_dropped_or_held_up_block_or_unsettled_parts_cap_a_fully_open_door_at_half():
    on_floor = block_at(0.16, 0.40, TABLE_TOP["station"] - 0.70)
    held_up = block_at(0.16, 0.40, TABLE_TOP["station"] + 0.10)
    assert outcome(door_at(FULL), on_floor) == (0.5, False)
    assert outcome(door_at(FULL), held_up) == (0.5, False)
    assert outcome(door_at(CLOSED), held_up) == (0.0, False)  # held out of the track is not yet "cleared"
    assert outcome(door_at(FULL, vel=(0.0, 0.05, 0.0)), BLOCK_ON_TABLE) == (0.5, False)


def test_a_door_out_of_its_track_scores_zero_whatever_else_happened():
    lifted_out = door_at(FULL, z_lift=0.03)
    beside_track = state((M.TRACK_X - 0.06, FULL, M.TOP + M.DOOR_SIZE[2] / 2), M.DOOR_SIZE)
    leaning = door_at(FULL, lean_deg=15.0)
    for door in (lifted_out, beside_track, leaning):
        assert outcome(door, BLOCK_ON_TABLE) == (0.0, False)
    # carried a few millimetres off the channel floor, still between the lips, the door still counts as seated
    assert outcome(door_at(FULL, z_lift=0.003), BLOCK_ON_TABLE) == (1.0, True)


def test_a_block_put_back_in_the_doors_path_does_not_count_as_cleared():
    on_the_lips = block_at(M.TRACK_X, M.BLOCK_START_Y, M.FLOOR + M.LIP_H)
    assert outcome(door_at(CLOSED), on_the_lips) == (0.0, False)


def test_a_block_moved_into_the_vacated_end_of_the_track_is_accepted():
    # Documented choice: the goal is the door fully open with nothing dropped; where on the table the block ends
    # up is not scored, including in the part of the track the door has left.
    behind_the_door = block_at(M.TRACK_X, M.TRACK_Y - M.STOP_IN + M.BLOCK_SIZE[1] / 2, M.FLOOR)
    assert outcome(door_at(FULL), behind_the_door) == (1.0, True)


@pytest.fixture(scope="module")
def plans():
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    return {arm: ArmPlan(list(waypoints), station.BASES[arm]) for arm, waypoints in plan.items()}


def test_the_demo_is_reachable_within_the_joint_limits(plans):
    for arm, plan in plans.items():
        steps = range(round(plan.duration * CONTROL_HZ) + 5)
        actions = np.array([plan.action7((k + 1) / CONTROL_HZ)[0] for k in steps])
        assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, arm  # the bounds yam_arena's own example meets
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER), arm


def segments(waypoints):
    """{label: (start, end)} in seconds from the start of the demo, for each label's first segment."""
    t, out = 0.0, {}
    for w in waypoints:
        out.setdefault(w.label, (t, t + w.duration))
        t += w.duration
    return out


def test_the_left_arm_clears_the_track_then_the_right_arm_slides_and_the_left_arm_finishes(plans):
    plan = TASK.demo()
    left, right = segments(plan["left"]), segments(plan["right"])
    # The right hand comes down to the door only once the left arm has carried the block clear of the track ...
    assert left["carry"][1] <= right["lower around the top edge"][0] + 1e-6
    # ... and the left arm comes to the door only once the right arm has let go and is back up.
    assert right["retreat"][1] <= left["above the door's left end"][0] + 1e-6
    assert left["release"][1] <= left["pinch the top edge"][0] + 1e-6
    assert (max(p.duration for p in plans.values()) + SCRIPTED_HOLD_S) * CONTROL_HZ < RUN_LIMIT_STEPS


def test_the_right_arm_does_most_of_the_slide_but_cannot_reach_the_end_alone():
    start, end = M.DOOR_START_Y + M.DOOR_GRIP_DY, M.DOOR_END_Y + M.DOOR_GRIP_DY
    assert start < M.HANDOFF_Y < end
    assert M.HANDOFF_Y - start > end - M.HANDOFF_Y  # the right arm's share of the travel is the larger
    # Pulling on to the end position, the right arm's IK misses by centimetres: the left arm has to finish.
    yaw = M.RIGHT_DOOR_YAW
    alone = ArmPlan([
        Waypoint(2.0, (M.TRACK_X, start, M.TRAVEL_Z), FINGER_OPEN, "above", yaw),
        Waypoint(1.5, (M.TRACK_X, start, M.DOOR_GRIP_Z), FINGER_OPEN, "lower", yaw),
        Waypoint(6.0, (M.TRACK_X, end, M.DOOR_GRIP_Z + M.DOOR_LIFT), FINGER_OPEN, "slide", yaw),
    ], station.BASES["right"])
    for k in range(round(alone.duration * CONTROL_HZ) + 5):
        alone.action7((k + 1) / CONTROL_HZ)
    assert alone.worst_ik_mm > 10


def test_the_door_is_pinched_by_its_leading_end_and_carried_inside_the_lips():
    # Pinched left of its centre (the end that leads when it slides left), with the pads inside the door's length.
    assert 0 < M.DOOR_GRIP_DY < M.DOOR_SIZE[1] / 2 - 0.01
    # Even if the arm lifted the full commanded height, the door's bottom edge would stay below the lips' top.
    assert M.DOOR_LIFT < M.LIP_H - 0.004


def test_the_forcing_control_leaves_the_block_and_pushes_the_door_into_it(monkeypatch):
    # DEMO_FORCE=1: the demo without clearing the track. The left arm never goes to the block; the right arm slides the
    # door to the handover point and the left arm pulls it on, past the point where the block stops it.
    monkeypatch.setenv("DEMO_FORCE", "1")
    plan = TASK.demo()
    assert set(plan) == {"left", "right"}
    for w in plan["left"]:  # down at the track only at the door's top edge, never beyond where the block stops it
        if w.pos[2] < M.TRAVEL_Z - 1e-9:
            assert w.pos[0] == M.TRACK_X and w.pos[2] >= M.DOOR_GRIP_Z - 1e-9, w.label
            assert w.pos[1] <= M.BLOCKED_PINCH_Y + M.FORCE_PAST + 1e-9, w.label
    push = [w for w in plan["left"] if w.label == "keep pushing"]
    assert len(push) == 1 and push[0].pos[1] > M.BLOCKED_PINCH_Y
    left, right = segments(plan["left"]), segments(plan["right"])
    assert right["retreat"][1] <= left["above the door's left end"][0] + 1e-6
    # Where the block stops the door, its left edge is 8 cm short of the end stop: no more than 0.25 is possible.
    blocked = door_at(M.BLOCKED_PINCH_Y - M.DOOR_GRIP_DY)  # door centre with its left edge against the block
    assert abs(float(blocked.high[1]) - (M.BLOCK_START_Y - M.BLOCK_SIZE[1] / 2)) < 1e-9
    assert outcome(blocked, BLOCK_IN_TRACK) == (0.25, False)
    for arm, waypoints in plan.items():
        p = ArmPlan(list(waypoints), station.BASES[arm])
        actions = np.array([p.action7((k + 1) / CONTROL_HZ)[0] for k in range(round(p.duration * CONTROL_HZ) + 5)])
        assert p.worst_ik_mm < 0.5 and p.worst_ik_deg < 0.1, (arm, p.worst_ik_mm, p.worst_ik_deg)
        assert np.all(actions[:, :6] >= ik.LOWER) and np.all(actions[:, :6] <= ik.UPPER), arm
        assert (p.duration + SCRIPTED_HOLD_S) * CONTROL_HZ < RUN_LIMIT_STEPS
