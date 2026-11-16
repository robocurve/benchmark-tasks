"""Regression tests for the linked transfer score and unobstructed finish."""

import math

import numpy as np

import reacher_trigger_retrieve as task
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan
from yam_arena.urdf import quat_xyzw_to_matrix


def finish(remote_x=task.MAT_X, tool_x=.140, trigger_deg=0):
    return task.kit.fake_states(
        q={"trigger_stroke": math.radians(trigger_deg)},
        moved={
            task.remote: ((remote_x, task.MAT_Y, task.TOP+.01018), (0, 0, 0, 1)),
            task.body: ((tool_x, -.390, task.TOP+.010), task.mk.yaw_quat(task.RIGHT_YAW)),
        },
    )


def test_direct_remote_shortcut_scores_zero(monkeypatch):
    monkeypatch.setattr(task.mk, "MEMORY", {})
    assert task.score(task.kit.fake_states()) == 0.0
    assert task.score(finish()) == 0.0


def test_linked_transfer_placement_and_release(monkeypatch):
    monkeypatch.setattr(task.mk, "MEMORY", {"linked_travel_m": .130})
    assert task.score(finish(remote_x=.385)) == .4
    assert task.score(finish(tool_x=.230)) == .8
    assert task.score(finish(trigger_deg=-15)) == .8
    assert task.score(finish()) == 1.0


def top_camera(points):
    cam = station.top_camera("calibrated")
    lens = cam.lens.resized(1280, 704)
    rot = quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw))
    coords = (np.asarray(points)-cam.pos) @ rot
    depth = -coords[:, 2]
    return np.c_[lens.cx+lens.fx*coords[:, 0]/depth, lens.cy-lens.fy*coords[:, 1]/depth]


def test_parked_arms_leave_scored_regions_unoccluded():
    """Check the projected clearance of the parked gripper, wrist and approach axis."""
    states = finish()
    trigger, _ = task.kit.pose(states, task.trigger)
    jaw = task.kit.point(states, task.body, (.435, 0, 0))
    target, _ = task.kit.pose(states, task.remote)
    pixels = top_camera([trigger, jaw, target])
    assert np.all((pixels > 20) & (pixels < [1260, 684]))
    for name, base in station.BASES.items():
        q = np.asarray(READY)
        if name in task.demo():
            plan = ArmPlan(task.demo()[name], base)
            q = plan.action7(plan.duration + .1)[0][:6]
        tcp, rotation = ik.tcp_pose(q, base)
        arm_positions = [tcp - d * rotation[:, 2] for d in (0, .05, .10, .15)]
        arm_pixels = top_camera(arm_positions)
        assert np.min(np.linalg.norm(arm_pixels[:, None, :] - pixels, axis=2)) >= 36
