# SPDX-License-Identifier: MIT
"""Scripted joint-action demonstration with demo-only measured tool feedback.

The simulator provides tool root poses and a filtered nib contact signal for
this policy. The controller sends arm joint and gripper targets. Grasping,
contact, and scoring are handled by the simulator.
"""
from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
import json
import math
from pathlib import Path

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk, Observation
from inspect_robots_yam.config import action_box
from inspect_robots_yam.packing import STATE_KEY, pack, split
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import ArmPlan, grip_rotation
from yam_arena.urdf import axis_angle_matrix

from . import task_spec as spec
from .contact import rotation_xyzw

CAPTURE = np.asarray(spec.GAUGE_CAPTURE_POS)
TRANSFER = np.asarray(spec.GAUGE_TRANSFER_POS)
STOCK_ROTATION = np.eye(3)
STOCK_TRAVEL = np.array([0., 0., .050])
STOCK_DESCENT_CLEARANCE = np.array([-.006, 0., 0.])
STOCK_WITHDRAW_CLEARANCE = np.array([-.006, .006, 0.])
STOCK_FLOOR_PRELOAD = np.array([0., 0., -.0005])
REFERENCE_PRELOAD = np.array([.0002, 0., -.0005])
BOARD_PRELOAD = np.array([.0006, 0., -.0005])
STOCK_PICKUP_TILT = math.radians(-22.)
MARKER_HOVER_Z = .840
MARKER_CLEAR_GOAL = np.array([.18, -.19, .90])
MARKER_CLEAR_TILT = axis_angle_matrix(np.array([0., 1., 0.]), math.radians(-10.))
FINAL_TRAVEL_SPEED = .045
CONTACT_FORCE = .10
STROKE_S = 10.
MAX_POSITION_LEAD = .030
MAX_ROTATION_LEAD = math.radians(18.)
MAX_RUN_S = 115.
PHASE_LIMITS = {
    "pick_stock": 7., "carry_to_reference": 15., "lower_reference": 12., "seat_reference": 12.,
    "approach_tab": 5., "close_tab": 3., "fit_reference": 25., "unload_tab": 10.,
    "release_tab": 3., "retract_tab": 5., "park_right": 6.,
    "unseat_reference": 12., "lift_stock": 12.,
    "transfer_stock": 22., "lower_board": 12., "seat_board": 12., "pick_marker": 8.,
    "carry_marker": 20., "touch_board": 8., "draw": 15., "finish": 6., "lift": 7.,
    "clear_marker": 9.,
}


def _yaw_rotation(yaw):
    return axis_angle_matrix(np.array([0., 0., 1.]), yaw)


def _tab_grasp(blade, blade_rot):
    """Approach the far tab with the jaw-closing axis along the blade."""
    yaw = math.atan2(blade_rot[1, 0], blade_rot[0, 0])
    rotation = axis_angle_matrix(blade_rot[:, 0], math.radians(40.)) @ grip_rotation(yaw + math.pi / 2)
    tab = blade[:3] + blade_rot @ np.asarray(spec.BLADE_GRASP_LOCAL)
    return tab - .016 * rotation[:, 2], rotation


class _TiltedStockPickup(ArmPlan):
    """Lean the left wrist away from the marking arm around the jaw closing axis."""

    def target(self, t):
        position, rotation, finger, label = super().target(t)
        tilt = STOCK_PICKUP_TILT * ik.smoothstep(t / self.waypoints[0].duration)
        return position, axis_angle_matrix(np.array([1., 0., 0.]), tilt) @ rotation, finger, label


def _draw_endpoints(blade, blade_rot, board, board_rot):
    """Intersect a measured guide offset with the two inset board cross sections."""
    edge = blade[:3] + np.asarray(spec.BLADE_LOCAL_EDGE) @ blade_rot.T
    edge -= .0025 * blade_rot[:, 1]
    local_edge = (edge - board[:3]) @ board_rot
    direction = local_edge[1] - local_edge[0]
    if direction[0] < .1:
        raise ValueError("measured guide does not span the board")
    x_edge = spec.BOARD_SIZE[0] / 2. - .003
    points = []
    for x in (x_edge, -x_edge):
        point = local_edge[0] + ((x - local_edge[0, 0]) / direction[0]) * direction
        if abs(point[1]) > spec.BOARD_SIZE[1] / 2. - spec.MARKER_NIB_RADIUS:
            raise ValueError("measured guide exits the long edge of the board")
        points.append((board[:3] + board_rot @ point)[:2])
    return points



def _withdrawn(position_error, angle_error, *fixture_forces):
    return (position_error < .002 and angle_error < math.radians(2.)
            and all(np.linalg.norm(force) < .02 for force in fixture_forces))


def _bounded(vector, maximum):
    length = float(np.linalg.norm(vector))
    return vector * min(1., maximum / max(length, 1e-12))


def _rotation_angle(rotation):
    """Angle of a rotation matrix in radians."""
    return math.acos(float(np.clip((np.trace(rotation) - 1.) / 2., -1., 1.)))


def _array(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(f"expected finite observation with shape {shape}")
    return result


@dataclass
class _ServoState:
    command_q: np.ndarray
    command_pos: np.ndarray
    command_rot: np.ndarray
    grasp_pos: np.ndarray
    grasp_rot: np.ndarray
    saturated_s: float = 0.
    drift_s: float = 0.


class BevelAngleTransferDemo(PolicyBase):
    """Capture an angle with the passive gauge, then mark alongside its measured blade."""

    def __init__(self, mode: str = "demo", debug_path: str | None = None) -> None:
        if mode not in ("demo", "hover", "mechanism"):
            raise ValueError("mode must be demo, hover, or mechanism")
        self.mode = mode
        self._debug_path = Path(debug_path) if debug_path else None
        self._dt = 1. / 20.
        self._bases = station.BASES
        low, high = np.r_[ik.LOWER, 0.], np.r_[ik.UPPER, 1.]
        self.info = PolicyInfo(
            name="bevel_angle_transfer_demo", action_space=action_box(pack(low, low), pack(high, high))
        )

    def bind(self, embodiment_info) -> None:
        if embodiment_info.control_hz:
            self._dt = 1. / float(embodiment_info.control_hz)

    def reset(self, scene) -> None:
        pickup = spec.pickup_waypoints()
        self._left_pick = _TiltedStockPickup(pickup["left"], self._bases["left"])
        self._right_pick = None
        self._right_waypoints = pickup["right"]
        self._last = (np.r_[READY, 1.], np.r_[READY, 1.])
        self._phase, self._phase_start = "pick_stock", None
        self._start = self._last_time = self._last_gauge_angle = None
        self._hold = self._stroke_s = self._contact_gap = 0.
        self._marker_rot = np.eye(3)
        self._pressure_bias = self._force_filtered = 0.
        self._servo: dict[str, _ServoState] = {}
        self._reference_hold = None
        self._debug_observation = {}

    def _unload_tab(self, pivot, force, dt, meta):
        state = self._servo["right"]
        # Relieve the tangential fence load while preserving vertical support.
        angle = math.radians(min(1., .3 * max(0., force - .35))) * dt
        outward = _yaw_rotation(angle)
        pos = pivot + outward @ (state.command_pos - pivot)
        rot = outward @ state.command_rot
        command = self._right_tcp(pos, rot, 0., state.command_q, meta)
        state.command_q, state.command_pos, state.command_rot = command[:6], pos, rot
        meta["tab_unload_deg"] = math.degrees(angle)
        return command


    def _right_tcp(self, pos, rot, grip, seed, meta):
        q, position_error, angle_error = ik.solve(
            pos, ik.matrix_to_quat_xyzw(rot), seed, base_pos=self._bases["right"], iters=40
        )
        meta["right_ik_error_mm"] = position_error * 1000.
        meta["right_ik_error_deg"] = math.degrees(angle_error)
        if position_error > .005 or angle_error > math.radians(5.):
            meta["feedback_fault"] = "right_tab_ik_unreachable"
        return np.r_[q, grip]


    def _transition(self, phase, now):
        self._phase, self._phase_start, self._hold = phase, now, 0.

    def _stable(self, condition, duration, dt):
        self._hold = self._hold + dt if condition else 0.
        return self._hold >= duration - 1e-9

    def _track(self, arm, q, tool_pose, tool_rot, goal_pos, goal_rot, dt, meta, *,
               press=False, feedforward=None, max_speed=.020):
        base = self._bases[arm]
        tcp_pos, tcp_rot = ik.tcp_pose(q, base)
        grasp_pos = tcp_rot.T @ (tool_pose[:3] - tcp_pos)
        grasp_rot = tcp_rot.T @ tool_rot
        state = self._servo.get(arm)
        if state is None:
            # Retain the last pickup setpoint and its load-bearing lead.
            command_q = self._last[0 if arm == "left" else 1][:6].copy()
            command_pos, command_rot = ik.tcp_pose(command_q, base)
            state = self._servo[arm] = _ServoState(
                command_q, command_pos, command_rot, grasp_pos, grasp_rot
            )

        error = goal_pos - tool_pose[:3]
        rot_error = goal_rot @ tool_rot.T
        error_angle = _rotation_angle(rot_error)
        correction = error if feedforward is None else error + feedforward
        velocity = _bounded(correction, max_speed)
        if press:
            velocity[2] = np.clip(velocity[2], -.010, .010)
        fraction = min(.5 * dt, math.radians(10.) * dt / max(error_angle, 1e-12))
        command_rot = ik.slerp_matrix(np.eye(3), rot_error, fraction) @ state.command_rot
        # Rotate about the commanded tool root, so yaw correction does not
        # sweep a held tool sideways about the gripper.
        tool_command = state.command_pos + state.command_rot @ grasp_pos + dt * velocity
        lead = tool_command - command_rot @ grasp_pos - tcp_pos
        rotation_lead = _rotation_angle(command_rot @ tcp_rot.T)
        position_lead = float(np.linalg.norm(lead))
        saturated = position_lead > MAX_POSITION_LEAD or rotation_lead > MAX_ROTATION_LEAD
        state.saturated_s = state.saturated_s + dt if saturated else 0.
        grasp_drift = float(np.linalg.norm(grasp_pos - state.grasp_pos))
        grasp_angle = _rotation_angle(grasp_rot @ state.grasp_rot.T)
        slipped = grasp_drift > .010 or grasp_angle > math.radians(20.)
        state.drift_s = state.drift_s + dt if slipped else 0.
        if state.saturated_s >= 2. - 1e-9:
            meta["feedback_fault"] = f"{arm}_servo_lead_saturated"
        elif state.drift_s >= .4 - 1e-9:
            meta["feedback_fault"] = f"{arm}_grasp_drift"

        # Clamp lead in tool space and blend rotation by the same fraction
        # to preserve the pivot while backing away from the lead limit.
        blend = min(1., MAX_POSITION_LEAD / max(position_lead, 1e-12),
                    MAX_ROTATION_LEAD / max(rotation_lead, 1e-12))
        command_rot = ik.slerp_matrix(tcp_rot, command_rot, blend)
        tool_command = tool_pose[:3] + blend * (tool_command - tool_pose[:3])
        lead = _bounded(tool_command - command_rot @ grasp_pos - tcp_pos, MAX_POSITION_LEAD)
        if press:
            # Allow a small normal lead into the surface without windup.
            lead[2] = np.clip(lead[2], -.004, .006)
        command_pos = tcp_pos + lead
        target, pos_error, angular_residual = ik.solve(
            command_pos, ik.matrix_to_quat_xyzw(command_rot), state.command_q,
            base_pos=base, iters=32, tol=5e-5,
        )
        state.command_q, state.command_pos, state.command_rot = target, command_pos, command_rot
        meta.update({
            arm + "_ik_error_mm": pos_error * 1000.,
            arm + "_ik_error_deg": math.degrees(angular_residual),
            arm + "_tool_error_mm": (error * 1000.).tolist(),
            arm + "_tool_error_deg": math.degrees(error_angle),
            arm + "_lead_mm": position_lead * 1000.,
            arm + "_lead_deg": math.degrees(rotation_lead),
            arm + "_grasp_drift_mm": grasp_drift * 1000.,
            arm + "_grasp_drift_deg": math.degrees(grasp_angle),
        })
        return np.r_[target, 0.]

    def _chunk(self, meta):
        data = pack(*self._last)
        if self._debug_path is not None:
            record = {**self._debug_observation, **meta, "commanded_joint_state": data.tolist()}
            with self._debug_path.open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, allow_nan=False) + "\n")
        return ActionChunk(actions=[Action(data=data, meta=meta)], control_hz=1. / self._dt)

    def _stop(self, reason, meta):
        return self._chunk({**meta, "request_stop": True, "stop_reason": reason})

    def act(self, observation: Observation) -> ActionChunk:
        self._debug_observation = {}
        try:
            tools = observation.extra["demo_tools"]
            if not isinstance(tools, Mapping):
                raise ValueError("demo_tools must be a mapping")
            stock, blade, marker, board = (
                _array(tools[name + "_pose"], (7,)) for name in ("stock", "blade", "marker", "board")
            )
            stock_rot, blade_rot, marker_rot, board_rot = (
                rotation_xyzw(pose[3:]) for pose in (stock, blade, marker, board)
            )
            board_force = _array(tools["marker_board_force"], (3,))
            blade_force = _array(tools["reference_blade_force"], (3,))
            stock_reference_force = _array(tools["reference_stock_force"], (3,))
            stock_board_force = _array(tools["stock_board_force"], (3,))
            gauge_angle, gauge_velocity = _array([tools["gauge_angle"], tools["gauge_velocity"]], (2,))
            nib_contact, captured, released, transferred = (
                tools[name] for name in ("nib_contact", "capture_valid", "released", "transfer_valid")
            )
            if not all(isinstance(flag, (bool, np.bool_)) for flag in (nib_contact, captured, released, transferred)):
                raise ValueError("demo contact and transfer flags must be booleans")
            left, right = split(_array(observation.state[STATE_KEY], (14,)))
            now = float(observation.state_time)
            if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
                raise ValueError("invalid simulation time")
        except (KeyError, ValueError, TypeError) as exc:
            return self._stop("demo_feedback_unavailable", {"phase": self._phase, "stop_detail": str(exc)})
        if self._debug_path is not None:
            self._debug_observation = {
                "state_time": now, "measured_joint_state": pack(left, right).tolist(),
                "stock_pose": stock.tolist(), "blade_pose": blade.tolist(), "marker_pose": marker.tolist(),
                "board_pose": board.tolist(), "marker_board_force": board_force.tolist(),
                "reference_stock_force": stock_reference_force.tolist(),
                "reference_blade_force": blade_force.tolist(),
                "stock_board_force": stock_board_force.tolist(),
            }

        if self._start is None:
            self._start = self._phase_start = now
        dt = self._dt if self._last_time is None else min(now - self._last_time, self._dt)
        pose_speed = (math.inf if dt <= 0. or self._last_gauge_angle is None else
                      abs(math.remainder(gauge_angle - self._last_gauge_angle, 2. * math.pi)) / dt)
        self._last_gauge_angle = gauge_angle
        self._last_time = now
        phase = self._phase
        elapsed = now - self._phase_start
        meta = {
            "phase": phase, "phase_s": elapsed, "nib_contact": bool(nib_contact),
            "gauge_angle_deg": math.degrees(gauge_angle), "gauge_velocity_deg_s": math.degrees(gauge_velocity),
            "gauge_pose_speed_deg_s": math.degrees(pose_speed) if math.isfinite(pose_speed) else None,
            "capture_valid": bool(captured), "released": bool(released), "transfer_valid": bool(transferred),
        }
        if now - self._start > MAX_RUN_S or elapsed > PHASE_LIMITS[phase]:
            return self._stop("demo_timeout_" + phase, meta)

        command_left, command_right = self._last
        withdrawing_tab = phase in ("release_tab", "retract_tab", "park_right")
        if withdrawing_tab and released and self._reference_hold is None:
            # Once the blade has been released, stop integrating stock preload
            # against the reference while the right hand withdraws and parks.
            self._reference_hold = np.r_[left[:6], 0.]
        if phase == "pick_stock":
            command_left, meta["waypoint"] = self._left_pick.action7(elapsed + self._dt)
            if elapsed >= self._left_pick.duration and stock[2] > .790:
                self._transition("carry_to_reference", now)
        elif withdrawing_tab and self._reference_hold is not None:
            command_left = self._reference_hold
        else:
            stock_goal = {
                "carry_to_reference": CAPTURE + STOCK_TRAVEL + STOCK_DESCENT_CLEARANCE,
                "lift_stock": CAPTURE + STOCK_TRAVEL + STOCK_WITHDRAW_CLEARANCE,
                "transfer_stock": TRANSFER + STOCK_TRAVEL + STOCK_DESCENT_CLEARANCE,
            }.get(phase)
            stock_goal_rot = STOCK_ROTATION
            if phase == "unseat_reference":
                stock_goal = self._reference_lift_goal
                stock_goal_rot = self._reference_lift_rot
            pressing = stock_goal is None
            if pressing:
                at_reference = phase in ("lower_reference", "seat_reference", "approach_tab", "close_tab", "fit_reference", "unload_tab",
                                         "release_tab", "retract_tab", "park_right")
                stock_goal = CAPTURE + REFERENCE_PRELOAD if at_reference else TRANSFER + BOARD_PRELOAD
                if phase in ("lower_reference", "lower_board"):
                    # Reference fitting may slide into place before capture. At
                    # the board descend directly into seating, without dragging
                    # the captured gauge sideways after touching the table.
                    datum = CAPTURE if at_reference else TRANSFER
                    inset = STOCK_DESCENT_CLEARANCE if at_reference else np.zeros(3)
                    stock_goal = datum + inset + STOCK_FLOOR_PRELOAD
            command_left = self._track(
                "left", left[:6], stock, stock_rot, stock_goal, stock_goal_rot, dt, meta, press=pressing,
            )
            if "feedback_fault" in meta:
                return self._stop(meta["feedback_fault"], meta)
            stock_error = float(np.linalg.norm(stock[:3] - stock_goal))
            stock_angle = _rotation_angle(stock_goal_rot @ stock_rot.T)
            aligned = stock_angle <= math.radians(1.)
            meta["stock_error_mm"] = stock_error * 1000.
            meta["stock_angle_deg"] = math.degrees(stock_angle)
            if phase == "carry_to_reference" and self._stable(stock_error < .002 and aligned, .15, dt):
                self._transition("lower_reference", now)
            elif phase == "lower_reference" and self._stable(stock_error < .002 and aligned, .15, dt):
                self._transition("seat_reference", now)
            elif phase == "seat_reference" and self._stable(stock_error < .002 and aligned, .30, dt):
                self._tab_pos, self._tab_rot = _tab_grasp(blade, blade_rot)
                self._tab_start_pos, self._tab_start_rot = ik.tcp_pose(right[:6], self._bases["right"])
                self._transition("approach_tab", now)
            elif phase == "unseat_reference":
                clear = _withdrawn(stock_error, stock_angle, stock_reference_force, blade_force)
                if self._stable(clear, .20, dt):
                    self._transition("lift_stock", now)
            elif phase == "lift_stock" and self._stable(stock_error < .003, .15, dt):
                self._transition("transfer_stock", now)
            elif phase == "transfer_stock" and self._stable(stock_error < .003 and aligned, .15, dt):
                self._transition("lower_board", now)
            elif phase == "lower_board" and self._stable(stock_error < .002 and aligned, .15, dt):
                self._transition("seat_board", now)
            elif phase == "seat_board" and self._stable(stock_error < .002 and aligned, .30, dt):
                if self.mode == "mechanism":
                    self._last = command_left, command_right
                    return self._stop("demo_transfer_complete", meta)
                self._right_pick = ArmPlan(self._right_waypoints, self._bases["right"], q_start=right[:6])
                self._transition("pick_marker", now)

        if phase == "approach_tab":
            fraction = ik.smoothstep(elapsed / 2.5)
            hover = self._tab_pos - .026 * self._tab_rot[:, 2]
            pos = self._tab_start_pos + fraction * (hover - self._tab_start_pos)
            rot = ik.slerp_matrix(self._tab_start_rot, self._tab_rot, fraction)
            command_right = self._right_tcp(pos, rot, 1., command_right[:6], meta)
            if elapsed >= 2.8:
                self._transition("close_tab", now)
        elif phase == "close_tab":
            approach = ik.smoothstep(elapsed / .95)
            grip = 1. - ik.smoothstep((elapsed - 1.05) / .65)
            pos = self._tab_pos - .026 * (1. - approach) * self._tab_rot[:, 2]
            command_right = self._right_tcp(pos, self._tab_rot, grip, command_right[:6], meta)
            if elapsed >= 2.:
                self._transition("fit_reference", now)
        elif phase == "fit_reference":
            # Rotate the hand around the stock's measured hinge center. A small
            # overtravel into the reference develops a physical contact preload.
            pivot = stock[:3] + stock_rot @ np.asarray(spec.GAUGE_PIVOT_LOCAL)
            desired = _yaw_rotation(spec.REFERENCE_ANGLE - math.radians(.2))
            command_right = self._track(
                "right", right[:6], blade, blade_rot, pivot, desired, dt, meta, press=True,
            )
            settled = (
                captured
                and abs(gauge_angle - spec.REFERENCE_ANGLE) <= math.radians(.6)
                and pose_speed <= math.radians(1.)
                and abs(stock[1] - CAPTURE[1]) <= .0007
            )
            meta["fit_settled"] = bool(settled)
            if self._stable(settled, .25, dt):
                reference_normal = _yaw_rotation(spec.REFERENCE_ANGLE)[:, 1]
                normal_force = float(abs(blade_force @ reference_normal))
                light_fit = normal_force <= 2. and meta["right_lead_mm"] <= 3. and meta["right_lead_deg"] <= 1.5
                meta["reference_blade_normal_force_n"] = normal_force
                if light_fit:
                    self._transition("release_tab", now)
                else:
                    self._transition("unload_tab", now)
        elif phase == "unload_tab":
            angle_error = abs(gauge_angle - spec.REFERENCE_ANGLE)
            if not captured or angle_error > math.radians(2.):
                return self._stop("demo_capture_lost_during_unload", meta)
            pivot = stock[:3] + stock_rot @ np.asarray(spec.GAUGE_PIVOT_LOCAL)
            reference_normal = _yaw_rotation(spec.REFERENCE_ANGLE)[:, 1]
            normal_force = float(abs(blade_force @ reference_normal))
            command_right = self._unload_tab(pivot, normal_force, dt, meta)
            meta["reference_blade_normal_force_n"] = normal_force
            settled = (
                normal_force <= .5
                and pose_speed <= math.radians(1.)
                and abs(stock[1] - CAPTURE[1]) <= .0007
            )
            if self._stable(settled, .25, dt):
                self._transition("release_tab", now)
        elif phase == "release_tab":
            command_right = np.r_[command_right[:6], ik.smoothstep(elapsed / .55)]
            if elapsed >= .75:
                self._retract_pos, self._retract_rot = ik.tcp_pose(right[:6], self._bases["right"])
                self._servo.pop("right", None)
                self._transition("retract_tab", now)
        elif phase == "retract_tab":
            fraction = ik.smoothstep(elapsed / 1.5)
            pos = self._retract_pos - .038 * fraction * self._retract_rot[:, 2]
            command_right = self._right_tcp(pos, self._retract_rot, 1., command_right[:6], meta)
            if elapsed >= 1.8 and released:
                self._right_park = ArmPlan(self._right_waypoints[:1], self._bases["right"], q_start=right[:6])
                self._transition("park_right", now)
        elif phase == "park_right":
            command_right, meta["waypoint"] = self._right_park.action7(elapsed + self._dt)
            park_error = float(np.max(np.abs(command_right[:6] - right[:6])))
            if elapsed >= self._right_park.duration and park_error < math.radians(3.) and released:
                self._reference_lift_goal = stock[:3] + STOCK_TRAVEL
                self._reference_lift_rot = stock_rot
                self._servo.pop("left", None)
                command_left = np.r_[left[:6], 0.]
                self._transition("unseat_reference", now)

        if phase == "pick_marker":
            command_right, meta["waypoint"] = self._right_pick.action7(elapsed + self._dt)
            if elapsed >= self._right_pick.duration and marker[2] > .840:
                relative = board_rot.T @ marker_rot
                yaw = math.atan2(relative[1, 0], relative[0, 0])
                self._marker_rot = board_rot @ _yaw_rotation(yaw)
                self._transition("carry_marker", now)
        elif phase in ("carry_marker", "touch_board", "draw", "finish", "lift"):
            if phase in ("carry_marker", "touch_board"):
                # The seated gauge can still settle while the marker approaches.
                # Keep the last measured path fixed once the stroke begins.
                try:
                    self._draw_start, self._draw_end = _draw_endpoints(blade, blade_rot, board, board_rot)
                except ValueError as exc:
                    return self._stop("demo_invalid_blade_path", {**meta, "stop_detail": str(exc)})
            board_normal = board_rot[:, 2]
            nib = marker[:3] + marker_rot @ np.asarray(spec.MARKER_NIB_LOCAL)
            clearance = float((nib - board[:3]) @ board_normal - spec.BOARD_SIZE[2]/2 - spec.MARKER_NIB_RADIUS)
            force = float(abs(board_force @ board_normal))
            self._force_filtered += .2 * (force - self._force_filtered)
            meta.update(nib_clearance_mm=clearance * 1000., marker_board_force_n=force)
            touching_phase = phase in ("touch_board", "draw", "finish") and self.mode == "demo"
            # Favor geometric clearance first, then compensate gently for
            # force error. Never run this controller in the hover control.
            if touching_phase:
                self._pressure_bias = float(np.clip(
                    self._pressure_bias + dt * .0005 * (self._force_filtered - CONTACT_FORCE),
                    -.00075, .00075,
                ))
            offset = self._marker_rot @ np.asarray(spec.MARKER_NIB_LOCAL)
            surface_z = board[2] + spec.BOARD_SIZE[2] / (2. * board_normal[2])
            contact_z = surface_z + spec.MARKER_NIB_RADIUS - offset[2]
            z = (contact_z - .0002 + self._pressure_bias if touching_phase else
                 contact_z + .015 if self.mode == "hover" and phase in ("touch_board", "draw", "finish") else
                 MARKER_HOVER_Z)
            xy = self._draw_start
            feedforward = None
            if phase == "draw":
                if nib_contact or self.mode == "hover":
                    # Pause stroke progress if measured contact momentarily breaks.
                    self._stroke_s += dt
                    self._contact_gap = 0.
                    feedforward = np.r_[(self._draw_end - self._draw_start) / STROKE_S, 0.]
                else:
                    self._contact_gap += dt
                    if self._contact_gap > .6:
                        return self._stop("demo_lost_nib_contact", meta)
                fraction = min(1., self._stroke_s / STROKE_S)
                xy = self._draw_start + fraction * (self._draw_end - self._draw_start)
                meta["stroke_progress"] = fraction
            elif phase in ("finish", "lift"):
                xy = self._draw_end
            goal = np.r_[xy - offset[:2], z]
            command_right = self._track(
                "right", right[:6], marker, marker_rot, goal, self._marker_rot, dt, meta,
                press=touching_phase, feedforward=feedforward,
            )
            if "feedback_fault" in meta:
                return self._stop(meta["feedback_fault"], meta)
            at_xy = float(np.linalg.norm(nib[:2] - xy)) < .0015
            marker_error = float(np.linalg.norm(marker[:3] - goal))
            meta["marker_error_mm"] = marker_error * 1000.
            if phase == "carry_marker" and self._stable(marker_error < .002, .15, dt):
                self._transition("touch_board", now)
            elif phase == "touch_board":
                ready_to_draw = nib_contact if self.mode == "demo" else abs(clearance - .015) < .002
                if self._stable(at_xy and ready_to_draw, .15, dt):
                    self._transition("draw", now)
            elif phase == "draw" and self._stroke_s >= STROKE_S - 1e-9:
                self._transition("finish", now)
            elif phase == "finish" and self._stable(at_xy and (nib_contact or self.mode == "hover"), .15, dt):
                self._transition("lift", now)
            elif phase == "lift" and self._stable(not nib_contact and clearance >= .005, .65, dt):
                self._transition("clear_marker", now)
        elif phase == "clear_marker":
            # The pitch adjustment adds wrist travel at the holder hover.
            marker_goal_rot = MARKER_CLEAR_TILT @ self._marker_rot
            command_right = self._track(
                "right", right[:6], marker, marker_rot, MARKER_CLEAR_GOAL, marker_goal_rot, dt, meta,
                max_speed=FINAL_TRAVEL_SPEED,
            )
            if "feedback_fault" in meta:
                return self._stop(meta["feedback_fault"], meta)
            marker_error = float(np.linalg.norm(marker[:3] - MARKER_CLEAR_GOAL))
            marker_angle = _rotation_angle(marker_goal_rot @ marker_rot.T)
            marker_clear = not nib_contact and marker_error < .004 and marker_angle < math.radians(2.)
            meta["marker_error_mm"] = marker_error * 1000.
            if self._stable(marker_clear and stock_error < .004 and aligned, .35, dt):
                self._last = command_left, command_right
                return self._stop("demo_motion_complete", meta)

        if "feedback_fault" in meta:
            return self._stop(meta["feedback_fault"], meta)
        self._last = command_left, command_right
        return self._chunk(meta)


def make_demo_policy(**kwargs) -> BevelAngleTransferDemo:
    return BevelAngleTransferDemo(**kwargs)
