# SPDX-License-Identifier: MIT
"""Checks for the bevel policy's deadlines, physical grasp path, and guide geometry."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


def test_all_controller_phases_have_deadlines():
    source = Path(__file__).parents[1] / "bevel_angle_transfer" / "feedback_policy.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    limits = next(ast.literal_eval(node.value) for node in tree.body
                  if isinstance(node, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == "PHASE_LIMITS" for t in node.targets))
    transitions = {node.args[0].value for node in ast.walk(tree)
                   if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                   and node.func.attr == "_transition"}
    assert transitions | {"pick_stock"} == limits.keys()
    assert all(0 < seconds <= 30 for seconds in limits.values())


@pytest.fixture
def policy():
    pytest.importorskip("inspect_robots_yam.config")
    pytest.importorskip("yam_arena")
    from bevel_angle_transfer import feedback_policy
    return feedback_policy


def test_draw_path_changes_with_measured_blade_angle(policy):
    spec = policy.spec
    stock = np.asarray(spec.GAUGE_TRANSFER_POS)
    pivot = stock + spec.GAUGE_PIVOT_LOCAL
    for angle in (spec.REFERENCE_ANGLE, spec.REFERENCE_ANGLE + math.radians(3.)):
        rotation = policy._yaw_rotation(angle)
        start, end = policy._draw_endpoints(pivot, rotation, np.asarray(spec.BOARD_CENTER), np.eye(3))
        assert start[0] == pytest.approx(.357)
        assert end[0] == pytest.approx(.223)
        assert (start[1] - end[1]) / (start[0] - end[0]) == pytest.approx(math.tan(angle))
        working_edge = pivot + rotation @ spec.BLADE_LOCAL_EDGE[0]
        assert (start - working_edge[:2]) @ rotation[:2, 1] == pytest.approx(-.0025)


def test_continuous_right_tab_grasp_and_release_ik(policy):
    from yam_arena import ik, station
    from yam_arena.constants import READY
    from yam_arena.demo import ArmPlan

    base = station.BASES["right"]
    q = np.asarray(READY)
    initial_position, initial_rotation = ik.tcp_pose(q, base)
    spec = policy.spec
    blade = np.asarray(spec.REFERENCE_PIVOT)
    start_position, start_rotation = policy._tab_grasp(blade, policy._yaw_rotation(spec.GAUGE_INITIAL_ANGLE))
    hover = start_position - .026 * start_rotation[:, 2]

    def solve(position, rotation):
        nonlocal q
        q, position_error, angle_error = ik.solve(
            position, ik.matrix_to_quat_xyzw(rotation), q, base_pos=base, iters=40,
        )
        assert position_error < .001
        assert angle_error < math.radians(1.)

    for fraction in np.linspace(0., 1., 31):
        solve(initial_position + fraction * (hover - initial_position),
              ik.slerp_matrix(initial_rotation, start_rotation, fraction))
    for fraction in np.linspace(0., 1., 15):
        solve(hover + fraction * (start_position - hover), start_rotation)
    for yaw in np.linspace(spec.GAUGE_INITIAL_ANGLE, spec.REFERENCE_ANGLE, 31):
        position, rotation = policy._tab_grasp(blade, policy._yaw_rotation(yaw))
        solve(position, rotation)
    for fraction in np.linspace(0., 1., 15):
        solve(position - .038 * fraction * rotation[:, 2], rotation)

    pickup = ArmPlan(spec.pickup_waypoints()["right"], base, q_start=q)
    for t in np.linspace(0., pickup.duration, 80):
        pickup.action7(t)
    assert pickup.worst_ik_mm < 1.
    assert pickup.worst_ik_deg < 1.



@pytest.mark.parametrize(("normal_force", "lead_mm", "lead_deg", "expected_phase"), [
    (1.6, 1.9, .6, "release_tab"),
    (4., 1.9, .6, "unload_tab"),
    (1.6, 5., .6, "unload_tab"),
    (1.6, 1.9, 2., "unload_tab"),
])
def test_reference_release_requires_stable_physical_capture(
    policy, monkeypatch, normal_force, lead_mm, lead_deg, expected_phase,
):
    from inspect_robots.types import Observation
    from inspect_robots_yam.packing import STATE_KEY, pack

    spec = policy.spec
    controller = policy.make_demo_policy()
    controller.reset(None)
    controller._transition("fit_reference", 0.)
    # Tracking alone is insufficient to leave this phase, even at the desired angle.
    def track(arm, *args, **kwargs):
        meta = args[-1]
        if arm == "right":
            meta.update(right_lead_mm=lead_mm, right_lead_deg=lead_deg)
        return np.r_[np.zeros(6), 0.]

    monkeypatch.setattr(controller, "_track", track)
    tools = {
        "stock_pose": [*spec.GAUGE_CAPTURE_POS, 0., 0., 0., 1.],
        "blade_pose": [*spec.REFERENCE_PIVOT, 0., 0., math.sin(spec.REFERENCE_ANGLE / 2),
                       math.cos(spec.REFERENCE_ANGLE / 2)],
        "marker_pose": [.18, -.19, .8175, 0., 0., 0., 1.],
        "board_pose": [*spec.BOARD_CENTER, 0., 0., 0., 1.],
        "marker_board_force": [0., 0., 0.],
        "reference_stock_force": [0., 0., 0.],
        "stock_board_force": [0., 0., 0.],
        "reference_blade_force": normal_force * policy._yaw_rotation(spec.REFERENCE_ANGLE)[:, 1],
        "gauge_angle": spec.REFERENCE_ANGLE,
        "gauge_velocity": 0.,
        "nib_contact": False,
        "capture_valid": False,
        "released": False,
        "transfer_valid": False,
    }
    joints = pack(*controller._last)
    controller.act(Observation(state={STATE_KEY: joints}, state_time=0., extra={"demo_tools": tools}))
    assert controller._phase == "fit_reference"
    tools["capture_valid"] = True
    tools["gauge_velocity"] = math.radians(4.)
    for step in range(1, 7):
        tools["gauge_angle"] += math.radians(.1 if step % 2 else -.1)
        controller.act(Observation(state={STATE_KEY: joints}, state_time=.05 * step,
                                   extra={"demo_tools": tools}))
    assert controller._phase == "fit_reference"

    # A stable measured angle can settle despite the instantaneous solver velocity.
    for step in range(7, 11):
        controller.act(Observation(state={STATE_KEY: joints}, state_time=.05 * step,
                                   extra={"demo_tools": tools}))
    assert controller._phase == "fit_reference"

    action = controller.act(Observation(state={STATE_KEY: joints}, state_time=.55,
                                        extra={"demo_tools": tools})).actions[0]
    assert controller._phase == expected_phase
    assert action.data.shape == (14,)



def test_left_stock_path_reserves_wrist_margin(policy):
    from yam_arena import ik, station
    from yam_arena.demo import grip_rotation

    spec = policy.spec
    base = station.BASES["left"]
    pickup = spec.pickup_waypoints()["left"]
    plan = policy._TiltedStockPickup(pickup, base)
    for t in np.linspace(0., plan.duration, 70):
        plan.action7(t)
    assert plan.worst_ik_mm < 1.

    grasp_offset = np.asarray(pickup[1].pos) - spec.GAUGE_INITIAL_POS
    capture, transfer = policy.CAPTURE, policy.TRANSFER
    high, inset = policy.STOCK_TRAVEL, policy.STOCK_DESCENT_CLEARANCE
    withdraw, floor = policy.STOCK_WITHDRAW_CLEARANCE, policy.STOCK_FLOOR_PRELOAD
    stock_path = (
        capture + high + inset,
        capture + floor + inset,
        capture + policy.REFERENCE_PRELOAD,
        capture + high,
        capture + high + withdraw,
        transfer + high + inset,
        transfer + floor,
        transfer + policy.BOARD_PRELOAD,
    )
    q = plan.q
    prev_pos = np.asarray(pickup[-1].pos)
    # Exercise 5° of tool yaw correction, comparable with the measured servo
    # lead during reference seating. Preserve at least 5° of further wrist travel.
    tilt = policy.axis_angle_matrix(np.array([1., 0., 0.]), policy.STOCK_PICKUP_TILT)
    rotation = ik.matrix_to_quat_xyzw(policy._yaw_rotation(math.radians(-5.)) @ tilt @ grip_rotation(pickup[1].yaw))
    for stock_pos in stock_path:
        goal = stock_pos + grasp_offset
        for fraction in np.linspace(0., 1., 21):
            q, position_error, angle_error = ik.solve(
                prev_pos + fraction * (goal - prev_pos), rotation, q, base_pos=base, iters=40,
            )
            assert position_error < .001
            assert angle_error < math.radians(1.)
            assert min(ik.UPPER[5] - q[5], q[5] - ik.LOWER[5]) > math.radians(5.)
        prev_pos = goal



def test_unload_rotates_around_pivot_without_losing_vertical_support(policy, monkeypatch):
    controller = policy.make_demo_policy()
    controller.reset(None)
    pivot = np.array([.208, .18, .7665])
    position = pivot + [.115, .015, .03]
    rotation = np.eye(3)
    controller._servo["right"] = policy._ServoState(
        np.zeros(6), position.copy(), rotation, np.zeros(3), rotation,
    )
    monkeypatch.setattr(controller, "_right_tcp", lambda pos, rot, grip, seed, meta: np.r_[seed, grip])
    meta = {}
    command = controller._unload_tab(pivot, 8., .05, meta)
    state = controller._servo["right"]
    assert command[-1] == 0.
    assert meta["tab_unload_deg"] == pytest.approx(.05)
    np.testing.assert_allclose(state.command_pos, pivot + state.command_rot @ (position - pivot))
    assert state.command_pos[2] == pytest.approx(position[2])
    assert state.command_pos[1] > position[1]



def test_withdrawal_requires_fixture_clearance_before_lifting(policy):
    clear = np.zeros(3)
    touching = np.array([.019, .007, 0.])
    assert policy._withdrawn(.0018, math.radians(1.7), clear, clear)
    assert not policy._withdrawn(.0018, math.radians(1.7), clear, touching)
    assert not policy._withdrawn(.0018, math.radians(1.7), touching)
    assert not policy._withdrawn(.0021, math.radians(1.7), clear)
    assert not policy._withdrawn(.0018, math.radians(2.1), clear)


def test_released_stock_holds_measured_joints_then_lifts_vertically(policy, monkeypatch):
    from inspect_robots.types import Observation
    from inspect_robots_yam.packing import STATE_KEY, pack

    controller = policy.make_demo_policy()
    controller.reset(None)
    controller._transition("release_tab", 0.)
    controller._servo["left"] = object()
    tracked = []

    def track(arm, q, tool, rotation, goal, goal_rot, dt, meta, **kwargs):
        tracked.append((arm, goal.copy(), goal_rot.copy(), kwargs["press"]))
        return np.r_[q, 0.]

    monkeypatch.setattr(controller, "_track", track)
    spec = policy.spec
    stock = np.asarray(spec.GAUGE_CAPTURE_POS) + [.0003, .002, .0001]
    stock_rot = policy._yaw_rotation(math.radians(.8))
    identity = [0., 0., 0., 1.]
    tools = {
        "stock_pose": [*stock, *policy.ik.matrix_to_quat_xyzw(stock_rot)],
        "blade_pose": [*spec.REFERENCE_PIVOT, *identity],
        "marker_pose": [.18, -.19, .8175, *identity],
        "board_pose": [*spec.BOARD_CENTER, *identity],
        "marker_board_force": [0., 0., 0.],
        "reference_stock_force": [0., 0., 0.],
        "reference_blade_force": [0., 0., 0.],
        "stock_board_force": [0., 0., 0.],
        "gauge_angle": spec.REFERENCE_ANGLE,
        "gauge_velocity": 0.,
        "nib_contact": False,
        "capture_valid": True,
        "released": True,
        "transfer_valid": False,
    }
    measured_left = np.r_[np.asarray(policy.READY) + .01, .25]
    right = np.r_[policy.READY, 1.]

    def observe(t):
        state = {STATE_KEY: pack(measured_left, right)}
        obs = Observation(state=state, state_time=t, extra={"demo_tools": tools})
        return controller.act(obs).actions[0].data[:7]

    held = observe(0.)
    np.testing.assert_array_equal(held, np.r_[measured_left[:6], 0.])
    measured_left[0] += .005
    tools["released"] = False
    np.testing.assert_array_equal(observe(.05), held)
    assert not tracked

    tools["released"] = True
    controller._transition("park_right", .05)
    controller._right_park = SimpleNamespace(duration=0., action7=lambda t: (right, "park"))
    np.testing.assert_array_equal(observe(.10), np.r_[measured_left[:6], 0.])
    assert controller._phase == "unseat_reference"
    assert "left" not in controller._servo

    observe(.15)
    ((arm, goal, goal_rot, pressing),) = tracked
    assert arm == "left" and pressing is False
    np.testing.assert_allclose(goal, stock + [0., 0., .05])
    np.testing.assert_allclose(goal_rot, stock_rot)



def test_left_pickup_tilts_during_hover_and_keeps_balanced_waypoints(policy):
    from yam_arena.demo import ArmPlan
    from yam_arena.urdf import rotation_angle_deg

    controller = policy.make_demo_policy()
    controller.reset(None)
    plan = controller._left_pick
    baseline = ArmPlan(plan.waypoints, plan.base_pos)
    hover = plan.waypoints[0].duration
    for t, tilt in ((0., 0.), (.5 * hover, 11.), (hover, 22.), (plan.duration, 22.)):
        position, rotation, finger, label = plan.target(t)
        flat_position, flat_rotation, flat_finger, flat_label = baseline.target(t)
        np.testing.assert_array_equal(position, flat_position)
        assert (finger, label) == (flat_finger, flat_label)
        assert rotation_angle_deg(rotation, flat_rotation) == pytest.approx(tilt, abs=1e-5)
        np.testing.assert_allclose(rotation @ flat_rotation.T @ [1., 0., 0.], [1., 0., 0.], atol=1e-12)

    position, rotation, _, _ = plan.target(hover)
    assert position[1] == pytest.approx(.27)
    assert rotation[1, 2] < 0.  # The wrist leans toward +Y, away from the marking arm.
    grasp, _, _, _ = plan.target(hover + plan.waypoints[1].duration)
    np.testing.assert_allclose(grasp, [.18, .27, .7825])


def test_marker_uses_refreshed_path_and_stops_after_clear_retraction(policy, monkeypatch):
    from inspect_robots.types import Observation
    from inspect_robots_yam.packing import STATE_KEY, pack

    spec = policy.spec
    controller = policy.make_demo_policy()
    controller.reset(None)
    goals = {}

    def track(arm, q, tool, rotation, goal, *args, **kwargs):
        goals[arm] = goal.copy()
        return np.r_[np.zeros(6), 0.]

    monkeypatch.setattr(controller, "_track", track)
    pivot = np.asarray(spec.GAUGE_TRANSFER_POS) + spec.GAUGE_PIVOT_LOCAL
    angle = spec.REFERENCE_ANGLE
    tools = {
        "stock_pose": [*spec.GAUGE_TRANSFER_POS, 0., 0., 0., 1.],
        "blade_pose": [*pivot, 0., 0., math.sin(angle / 2), math.cos(angle / 2)],
        "marker_pose": [.18, -.19, .86, 0., 0., 0., 1.],
        "board_pose": [*spec.BOARD_CENTER, 0., 0., 0., 1.],
        "marker_board_force": [0., 0., 0.],
        "reference_stock_force": [0., 0., 0.],
        "reference_blade_force": [0., 0., 0.],
        "stock_board_force": [1., 0., 0.],
        "gauge_angle": angle,
        "gauge_velocity": 0.,
        "nib_contact": False,
        "capture_valid": True,
        "released": True,
        "transfer_valid": True,
    }
    joints = pack(*controller._last)

    def observe(t):
        observation = Observation(state={STATE_KEY: joints}, state_time=t, extra={"demo_tools": tools})
        return controller.act(observation).actions[0]

    def act(phase, t):
        controller._transition(phase, t)
        observe(t)
        return np.array([controller._draw_start, controller._draw_end])

    approach = act("carry_marker", 0.)
    tools["blade_pose"][1] -= .001
    touch = act("touch_board", .05)
    np.testing.assert_allclose(touch - approach, [[0., -.001], [0., -.001]], atol=1e-12)
    tools["blade_pose"][1] -= .001
    np.testing.assert_array_equal(act("draw", .10), touch)

    controller._transition("finish", .10)
    tools["marker_pose"] = [*touch[1], .8155, 0., 0., 0., 1.]
    tools["nib_contact"] = True
    tools["marker_board_force"] = [0., 0., .3]
    for step in range(3, 6):
        observe(.05 * step)
    assert controller._phase == "lift"

    tools["marker_pose"][2] += .010
    tools["nib_contact"] = False
    tools["marker_board_force"] = [0., 0., 0.]
    for step in range(6, 19):
        assert not observe(.05 * step).meta.get("request_stop")
    assert controller._phase == "clear_marker"
    assert not observe(.95).meta.get("request_stop")
    np.testing.assert_array_equal(goals["right"], policy.MARKER_CLEAR_GOAL)
    np.testing.assert_array_equal(goals["left"], policy.TRANSFER + policy.BOARD_PRELOAD)

    tools["marker_pose"][:3] = policy.MARKER_CLEAR_GOAL
    tools["marker_pose"][3:] = policy.ik.matrix_to_quat_xyzw(policy.MARKER_CLEAR_TILT)
    for step in range(20, 26):
        assert not observe(.05 * step).meta.get("request_stop")
    action = observe(1.30)
    assert action.meta["stop_reason"] == "demo_motion_complete"
    np.testing.assert_array_equal(goals["left"], policy.TRANSFER + policy.BOARD_PRELOAD)


def test_right_marker_withdrawal_has_continuous_ik_and_joint_margin(policy):
    from yam_arena import ik, station

    # Joint and tool poses measured after lifting the marker in the frozen demo.
    q = np.array([.836577, 2.036352, 1.756924, -1.417658, -.178991, .807640])
    marker = np.array([.221984, -.026075, .829428])
    marker_rot = policy.rotation_xyzw([.01362729, .00084997, .00501151, .99989414])
    base = station.BASES["right"]
    tcp_pos, tcp_rot = ik.tcp_pose(q, base)
    grasp_pos = marker_rot.T @ (tcp_pos - marker)
    grasp_rot = marker_rot.T @ tcp_rot
    yaw = math.atan2(marker_rot[1, 0], marker_rot[0, 0])
    final_marker_rot = policy.MARKER_CLEAR_TILT @ policy._yaw_rotation(yaw)
    motion = policy.MARKER_CLEAR_GOAL - marker
    for fraction in np.linspace(0., 1., 31):
        rotation = ik.slerp_matrix(marker_rot, final_marker_rot, fraction)
        target_pos = marker + fraction * motion + rotation @ grasp_pos
        target_rot = ik.matrix_to_quat_xyzw(rotation @ grasp_rot)
        q, position_error, angle_error = ik.solve(target_pos, target_rot, q, base_pos=base, iters=40)
        assert position_error < .001
        assert angle_error < math.radians(1.)
        assert np.min(np.minimum(ik.UPPER - q, q - ik.LOWER)) > math.radians(5.)
