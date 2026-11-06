# SPDX-License-Identifier: MIT
"""Scripted joint-action demonstration with demo-only measured tool feedback.

The simulator provides tool root poses and a filtered nib contact signal for
this policy. Only arm joint and gripper targets are commanded; grasping,
contact, and scoring are handled by the same physics as any other policy.
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
from yam_arena.demo import ArmPlan
from yam_arena.urdf import axis_angle_matrix

from . import task_spec as spec
from .contact import guide_metrics, rotation_xyzw

SQUARE_HOVER = np.array([.2078, .090, .8155])
SQUARE_SEATED = np.array([.2078, .090, .7650])
DRAW_START = np.array([.357, -.030])
DRAW_END = np.array([.223, -.030])
MARKER_HOVER_Z = .840
CONTACT_FORCE = .30
STROKE_S = 8.
MAX_POSITION_LEAD = .030
MAX_ROTATION_LEAD = math.radians(18.)
MAX_RUN_S = 100.0
PHASE_LIMITS = {
    "pick_square": 7., "carry_square": 20., "seat_square": 10.,
    "pick_marker": 7., "carry_marker": 20., "touch_board": 8.,
    "draw": 15., "finish": 6., "lift": 7., "retreat_marker": 20., "lift_square": 10., "retreat_square": 20.,
}


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


class SquareCutLineDemo(PolicyBase):
    """Pick, seat, mark, and lift with independent feedback on each held tool."""

    def __init__(self, mode: str = "demo", debug_path: str | None = None) -> None:
        if mode not in ("demo", "hover"):
            raise ValueError("mode must be demo or hover")
        self.mode = mode
        self._debug_path = Path(debug_path) if debug_path else None
        self._dt = 1. / 20.
        self._bases = station.BASES
        low, high = np.r_[ik.LOWER, 0.], np.r_[ik.UPPER, 1.]
        self.info = PolicyInfo(
            name="square_cut_line_demo", action_space=action_box(pack(low, low), pack(high, high))
        )

    def bind(self, embodiment_info) -> None:
        if embodiment_info.control_hz:
            self._dt = 1. / float(embodiment_info.control_hz)

    def reset(self, scene) -> None:
        pickup = spec.pickup_waypoints()
        # Retain the proven approach, finger closure, and lift for each tool.
        self._left_pick = ArmPlan(pickup["left"], self._bases["left"])
        self._right_pick = None
        self._right_waypoints = pickup["right"]
        self._last = (np.r_[READY, 1.], np.r_[READY, 1.])
        self._phase, self._phase_start = "pick_square", None
        self._start = self._last_time = None
        self._hold = self._stroke_s = self._contact_gap = 0.
        self._marker_rot = np.eye(3)
        self._pressure_bias = self._force_filtered = 0.
        self._servo: dict[str, _ServoState] = {}
        self._debug_observation = {}

    def _transition(self, phase, now):
        self._phase, self._phase_start, self._hold = phase, now, 0.

    def _stable(self, condition, duration, dt):
        self._hold = self._hold + dt if condition else 0.
        return self._hold >= duration - 1e-9

    def _track(self, arm, q, tool_pose, tool_rot, goal_pos, goal_rot, dt, meta, *,
               press=False, feedforward=None):
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
        velocity = _bounded(correction, .020)
        if press:
            velocity[2] = np.clip(velocity[2], -.010, .010)
        fraction = min(.5 * dt, math.radians(10.) * dt / max(error_angle, 1e-12))
        command_rot = ik.slerp_matrix(np.eye(3), rot_error, fraction) @ state.command_rot
        # Rotate about the commanded tool root, so yaw correction does not
        # sweep the far end of the square sideways about the gripper.
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
            square, marker, board = (
                _array(tools[name + "_pose"], (7,)) for name in ("square", "marker", "board")
            )
            square_rot, marker_rot, board_rot = (
                rotation_xyzw(pose[3:]) for pose in (square, marker, board)
            )
            board_force = _array(tools["marker_board_force"], (3,))
            if not isinstance(tools["nib_contact"], (bool, np.bool_)):
                raise ValueError("nib_contact must be a boolean")
            nib_contact = bool(tools["nib_contact"])
            left, right = split(_array(observation.state[STATE_KEY], (14,)))
            now = float(observation.state_time)
            if not math.isfinite(now) or (self._last_time is not None and now < self._last_time):
                raise ValueError("invalid simulation time")
        except (KeyError, ValueError, TypeError) as exc:
            return self._stop("demo_feedback_unavailable", {"phase": self._phase, "stop_detail": str(exc)})
        if self._debug_path is not None:
            self._debug_observation = {
                "state_time": now, "measured_joint_state": pack(left, right).tolist(),
                "square_pose": square.tolist(), "marker_pose": marker.tolist(),
                "board_pose": board.tolist(), "marker_board_force": board_force.tolist(),
            }

        if self._start is None:
            self._start = self._phase_start = now
        dt = self._dt if self._last_time is None else min(now - self._last_time, self._dt)
        self._last_time = now
        phase = self._phase
        elapsed = now - self._phase_start
        meta = {"phase": phase, "phase_s": elapsed, "nib_contact": nib_contact}
        if now - self._start > MAX_RUN_S or elapsed > PHASE_LIMITS[phase]:
            return self._stop("demo_timeout_" + phase, meta)

        command_left, command_right = self._last
        if phase == "pick_square":
            command_left, label = self._left_pick.action7(elapsed + self._dt)
            meta["waypoint"] = label
            if elapsed >= self._left_pick.duration and square[2] > .800:
                self._transition("carry_square", now)
        else:
            square_goal = (SQUARE_HOVER if phase in ("carry_square", "lift_square") else
                           np.array([.18, .27, .8155]) if phase == "retreat_square" else SQUARE_SEATED)
            command_left = self._track(
                "left", left[:6], square, square_rot, square_goal, board_rot, dt, meta,
                press=(phase not in ("carry_square", "lift_square", "retreat_square")),
            )
            if "feedback_fault" in meta:
                return self._stop(meta["feedback_fault"], meta)
            square_error = float(np.linalg.norm(square[:3] - square_goal))
            square_angle = _rotation_angle(board_rot @ square_rot.T)
            aligned = square_angle <= math.radians(1.)
            meta["square_error_mm"] = square_error * 1000.
            meta["square_angle_deg"] = math.degrees(square_angle)
            if phase == "carry_square" and self._stable(square_error < .002 and aligned, .15, dt):
                self._transition("seat_square", now)
            elif phase == "seat_square":
                # Check the guide across the whole line, not just at the start.
                seated = all(
                    guide_metrics([*xy, spec.BOARD_TOP], square, board, np.zeros(3), spec)["guide_used"]
                    for xy in (DRAW_START, DRAW_END)
                )
                meta["square_seated"] = seated
                if self._stable(seated and square_error < .002 and aligned, .30, dt):
                    self._right_pick = ArmPlan(
                        self._right_waypoints, self._bases["right"], q_start=right[:6]
                    )
                    self._transition("pick_marker", now)

        if phase == "pick_marker":
            command_right, label = self._right_pick.action7(elapsed + self._dt)
            meta["waypoint"] = label
            if elapsed >= self._right_pick.duration and marker[2] > .840:
                # Keep the observed yaw of the cylindrical marker. Correct only
                # its tilt, avoiding an unnecessary twist of the wrist.
                relative = board_rot.T @ marker_rot
                yaw = math.atan2(relative[1, 0], relative[0, 0])
                self._marker_rot = board_rot @ axis_angle_matrix(np.array([0., 0., 1.]), yaw)
                self._transition("carry_marker", now)
        elif phase in ("carry_marker", "touch_board", "draw", "finish", "lift", "retreat_marker", "lift_square", "retreat_square"):
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
                    self._pressure_bias + dt * .001 * (self._force_filtered - CONTACT_FORCE),
                    -.002, .001,
                ))
            offset = self._marker_rot @ np.asarray(spec.MARKER_NIB_LOCAL)
            surface_z = board[2] + spec.BOARD_SIZE[2] / (2. * board_normal[2])
            contact_z = surface_z + spec.MARKER_NIB_RADIUS - offset[2]
            z = (contact_z - .0005 + self._pressure_bias if touching_phase else
                 contact_z + .015 if self.mode == "hover" and phase in ("touch_board", "draw", "finish") else
                 MARKER_HOVER_Z)
            xy = DRAW_START
            feedforward = None
            if phase == "draw":
                if nib_contact or self.mode == "hover":
                    # Pause stroke progress if measured contact momentarily breaks.
                    self._stroke_s += dt
                    self._contact_gap = 0.
                    feedforward = np.r_[(DRAW_END - DRAW_START) / STROKE_S, 0.]
                else:
                    self._contact_gap += dt
                    if self._contact_gap > .6:
                        return self._stop("demo_lost_nib_contact", meta)
                fraction = min(1., self._stroke_s / STROKE_S)
                xy = DRAW_START + fraction * (DRAW_END - DRAW_START)
                meta["stroke_progress"] = fraction
            elif phase in ("finish", "lift"):
                xy = DRAW_END
            elif phase in ("retreat_marker", "lift_square", "retreat_square"):
                xy = np.array([.18, -.19])
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
            elif phase == "finish" and self._stable(at_xy and (nib_contact or self.mode == "hover"), .80, dt):
                self._transition("lift", now)
            elif phase == "lift" and self._stable(not nib_contact and clearance >= .005, .65, dt):
                self._transition("retreat_marker", now)
            elif phase == "retreat_marker" and self._stable(marker_error < .003, .20, dt):
                self._transition("lift_square", now)
            elif phase == "lift_square" and self._stable(square_error < .002, .15, dt):
                self._transition("retreat_square", now)
            elif phase == "retreat_square" and self._stable(square_error < .003, .20, dt):
                self._last = command_left, command_right
                return self._stop("demo_motion_complete", meta)

        self._last = command_left, command_right
        return self._chunk(meta)


def make_demo_policy(**kwargs) -> SquareCutLineDemo:
    return SquareCutLineDemo(**kwargs)
