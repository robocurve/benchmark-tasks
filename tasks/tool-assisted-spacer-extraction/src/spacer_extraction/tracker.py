"""Observable progress tracking for the tool-assisted spacer task.

The tracker consumes only Arena ``ObjectState`` values. The task-local runtime
adapter is responsible for resetting one tracker per episode, calling
``update`` once after each action, and supplying timestamps and verified gripper
release state. The stock task-file API does not provide those reset/release
signals by itself.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np


TOP = 0.7475
CHANNEL_X0 = 0.330
CHANNEL_X1 = 0.370
CHANNEL_Y = 0.100
SPACER_HALF_X = 0.014
SPACER_HALF_Y = 0.012
SPACER_HALF_Z = 0.012
PUSHER_HALF_X = 0.130
PUSHER_HALF_Y = 0.008
PUSHER_HALF_Z = 0.008
CONTACT_SEPARATION_M = 0.002001
MOTION_EPSILON_M = 1e-9
PUSHER_START = np.array([0.270, 0.340, TOP + PUSHER_HALF_Z])
SPACER_START = np.array([0.351, CHANNEL_Y, TOP + SPACER_HALF_Z])
NEST = np.array([0.350, 0.260, TOP + SPACER_HALF_Z])
TABLE_X_BOUNDS = (-0.190, 0.910)
TABLE_Y_BOUNDS = (-0.650, 0.650)


@dataclass(frozen=True)
class SpacerScoreSnapshot:
    mediated_push: bool = False
    verified_push_m: float = 0.0
    ejected: bool = False
    recovered: bool = False
    tool_parked: bool = False
    stable: bool = False
    stable_s: float = 0.0
    released: bool = False
    failure: str | None = None
    score: float = 0.0
    success_ready: bool = False


def _unit(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    return v / norm if norm > 1e-12 else np.array([1.0, 0.0, 0.0])


def _rotate(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector by XYZW quaternion."""
    q = _unit(np.asarray(q, dtype=float))
    qv, qw = q[:3], q[3]
    return v + 2.0 * np.cross(qv, np.cross(qv, v) + qw * v)


def _axis_errors(q: np.ndarray) -> tuple[float, float]:
    x_axis = _rotate(np.asarray(q, dtype=float), np.array([1.0, 0.0, 0.0]))
    y_axis = _rotate(np.asarray(q, dtype=float), np.array([0.0, 1.0, 0.0]))
    z_axis = _rotate(np.asarray(q, dtype=float), np.array([0.0, 0.0, 1.0]))
    yaw_deg = float(np.degrees(np.arccos(np.clip(abs(x_axis[0]), 0.0, 1.0))))
    tilt_deg = float(np.degrees(np.arccos(np.clip(max(abs(y_axis[2]), abs(z_axis[2])), 0.0, 1.0))))
    return yaw_deg, tilt_deg


def _rotation_error_deg(q: np.ndarray) -> float:
    q = _unit(np.asarray(q, dtype=float))
    # Both parts have square transverse sections: either end and any long
    # face may lead/rest on the table without changing the capability.
    yaw, tilt = _axis_errors(q)
    return max(yaw, tilt)


def _forward_axis(q: np.ndarray) -> np.ndarray:
    axis = _rotate(np.asarray(q), np.array([1.0, 0.0, 0.0]))
    return axis if axis[0] >= 0 else -axis


def _boxes_near_contact(tool, spacer) -> bool:
    """Reject separated oriented boxes using all 15 separating-axis candidates.

    Project each box onto both sets of face normals and their nine edge cross
    products. A 2 mm allowance matches the leading-face contact proxy; overlap
    of world-axis bounds alone cannot prove contact for a long rotated bar.
    """
    tool_axes = np.column_stack([_rotate(tool.quat_xyzw, axis) for axis in np.eye(3)])
    spacer_axes = np.column_stack([_rotate(spacer.quat_xyzw, axis) for axis in np.eye(3)])
    tool_half = np.array([PUSHER_HALF_X, PUSHER_HALF_Y, PUSHER_HALF_Z])
    spacer_half = np.array([SPACER_HALF_X, SPACER_HALF_Y, SPACER_HALF_Z])
    displacement = np.asarray(spacer.pos) - np.asarray(tool.pos)
    candidates = [*tool_axes.T, *spacer_axes.T]
    candidates.extend(np.cross(tool_axis, spacer_axis)
                      for tool_axis in tool_axes.T for spacer_axis in spacer_axes.T)
    for candidate in candidates:
        length = float(np.linalg.norm(candidate))
        if length < 1e-10:
            continue  # Parallel edges have no usable separating normal.
        normal = candidate / length
        tool_radius = float(np.abs(tool_axes.T @ normal) @ tool_half)
        spacer_radius = float(np.abs(spacer_axes.T @ normal) @ spacer_half)
        gap = abs(float(displacement @ normal)) - tool_radius - spacer_radius
        if gap > CONTACT_SEPARATION_M:
            return False
    return True


def _near_pose(state, target: np.ndarray, tolerance: tuple[float, float, float]) -> bool:
    return bool(np.all(np.abs(state.pos - target) <= np.asarray(tolerance)))


def _still(state, *, speed: float = 0.01, spin: float = 0.5) -> bool:
    return state.resting(max_speed=speed, max_spin=spin)


class SpacerTrialTracker:
    """Accumulate verified pusher-mediated ejection, recovery, and release dwell."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.previous_tool: np.ndarray | None = None
        self.previous_spacer: np.ndarray | None = None
        self.previous_tool_geometry: dict[str, np.ndarray] | None = None
        self.previous_spacer_geometry: dict[str, np.ndarray] | None = None
        self.previous_time: float | None = None
        self.verified_push_m = 0.0
        self.push_event = False
        self.valid_exit = False
        self.failure: str | None = None
        self.stable_since: float | None = None
        self.last = SpacerScoreSnapshot()

    @staticmethod
    def _inside_table(state) -> bool:
        return bool(
            state.low[0] >= TABLE_X_BOUNDS[0]
            and state.high[0] <= TABLE_X_BOUNDS[1]
            and state.low[1] >= TABLE_Y_BOUNDS[0]
            and state.high[1] <= TABLE_Y_BOUNDS[1]
            and state.low[2] >= TOP - 0.01
        )

    @staticmethod
    def _contact_overlap(tool, spacer) -> tuple[float, float]:
        overlap_y = min(float(tool.high[1]), float(spacer.high[1])) - max(
            float(tool.low[1]), float(spacer.low[1])
        )
        overlap_z = min(float(tool.high[2]), float(spacer.high[2])) - max(
            float(tool.low[2]), float(spacer.low[2])
        )
        return overlap_y, overlap_z

    @staticmethod
    def _aligned_contact(tool, spacer) -> bool:
        if abs(float(spacer.pos[2]) - (TOP + SPACER_HALF_Z)) > 0.004:
            return False
        if abs(float(tool.pos[2]) - (TOP + PUSHER_HALF_Z)) > 0.005:
            return False
        if abs(float(spacer.pos[1]) - CHANNEL_Y) > 0.010:
            return False
        tool_yaw, tool_tilt = _axis_errors(tool.quat_xyzw)
        spacer_yaw, spacer_tilt = _axis_errors(spacer.quat_xyzw)
        if max(tool_yaw, spacer_yaw) > 10.0 or max(tool_tilt, spacer_tilt) > 5.0:
            return False
        tool_axis = _forward_axis(tool.quat_xyzw)
        spacer_axis = _forward_axis(spacer.quat_xyzw)
        tip = tool.pos + PUSHER_HALF_X * tool_axis
        spacer_rear_face = spacer.pos - SPACER_HALF_X * spacer_axis
        axial_gap = float(np.dot(spacer_rear_face - tip, tool_axis))
        overlap_y, overlap_z = SpacerTrialTracker._contact_overlap(tool, spacer)
        return bool(
            -0.004 <= axial_gap <= CONTACT_SEPARATION_M
            and overlap_y > 0.008 and overlap_z > 0.006
            and _boxes_near_contact(tool, spacer)
        )

    @staticmethod
    def _geometry(state) -> dict[str, np.ndarray]:
        return {name: np.asarray(getattr(state, name), dtype=float).copy()
                for name in ("pos", "quat_xyzw", "low", "high")}

    @staticmethod
    def _valid_state(name: str, state) -> None:
        for field, shape in (("pos", (3,)), ("quat_xyzw", (4,)), ("lin_vel", (3,)),
                             ("ang_vel", (3,)), ("low", (3,)), ("high", (3,))):
            value = np.asarray(getattr(state, field, None), dtype=float)
            if value.shape != shape or not np.all(np.isfinite(value)):
                raise ValueError(f"{name}.{field} must be a finite vector of shape {shape}")
        quat = np.asarray(state.quat_xyzw, dtype=float)
        norm = float(np.linalg.norm(quat))
        if not np.isfinite(norm) or norm <= 1e-12:
            raise ValueError(f"{name}.quat_xyzw must have finite nonzero norm")
        if np.any(np.asarray(state.low, dtype=float) > np.asarray(state.high, dtype=float)):
            raise ValueError(f"{name} bounds are inverted")

    def update(
        self,
        objects: Mapping[str, object],
        *,
        time_s: float | None = None,
        released: bool = False,
    ) -> SpacerScoreSnapshot:
        """Advance once per control frame using the wrapper-provided time and release proof."""
        tool = objects["pusher"]
        spacer = objects["spacer"]
        self._valid_state("pusher", tool)
        self._valid_state("spacer", spacer)
        if not self._inside_table(tool) or not self._inside_table(spacer):
            self.failure = "required_object_left_table"

        if time_s is None:
            raise ValueError("time_s is required for 20 Hz frame validation")
        now = float(time_s)
        if not np.isfinite(now):
            raise ValueError("time_s must be finite")
        if self.previous_time is None:
            if abs(now) > 1e-7:
                raise ValueError("the first evaluator frame must be at reset time zero")
        elif abs((now - self.previous_time) - 0.05) > 1e-7:
            raise ValueError("evaluator frames must advance at exactly 20 Hz")
        if self.previous_tool is not None and self.previous_spacer is not None:
            assert self.previous_tool_geometry is not None and self.previous_spacer_geometry is not None
            current_contact = self._aligned_contact(tool, spacer)
            previous_contact = self._aligned_contact(
                _ArrayState(self.previous_tool_geometry), _ArrayState(self.previous_spacer_geometry)
            )
            d_tool = float(tool.pos[0] - self.previous_tool[0])
            d_spacer = float(spacer.pos[0] - self.previous_spacer[0])
            if d_spacer < 0:
                self.verified_push_m = max(0.0, self.verified_push_m + d_spacer)
            if current_contact and previous_contact:
                # Count net paired progress regardless of execution speed. The
                # negative-spacer subtraction above prevents oscillation from
                # accumulating a stroke without real forward displacement.
                if d_tool > MOTION_EPSILON_M and d_spacer > MOTION_EPSILON_M:
                    self.verified_push_m += min(d_tool, d_spacer)

        fully_clear = float(spacer.low[0]) >= CHANNEL_X1
        remains_on_axis = abs(float(spacer.pos[1]) - CHANNEL_Y) <= 0.010
        if self.previous_spacer_geometry is not None:
            crossed_exit = float(self.previous_spacer_geometry["low"][0]) < CHANNEL_X1 and fully_clear
            if crossed_exit and not self.valid_exit:
                if current_contact and previous_contact and self.verified_push_m >= 0.025 and remains_on_axis:
                    self.valid_exit = True
                else:
                    self.failure = "extraction_without_verified_pusher_contact"
        self.push_event = self.push_event or bool(
            self.valid_exit and self.verified_push_m >= 0.040 and fully_clear and remains_on_axis
            and self._aligned_contact(tool, spacer)
        )
        ejected_now = bool(self.push_event and fully_clear and remains_on_axis)

        recovered = self.push_event and _near_pose(spacer, NEST, (0.008, 0.008, 0.004)) and _rotation_error_deg(
            spacer.quat_xyzw
        ) <= 10.0 and _still(spacer, speed=0.015, spin=0.75)
        tool_parked = _near_pose(tool, PUSHER_START, (0.012, 0.012, 0.010)) and _rotation_error_deg(
            tool.quat_xyzw
        ) <= 10.0
        stable = bool(recovered and tool_parked and released and _still(tool, speed=0.01, spin=0.5))

        if stable and now is not None:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None
        stable_s = 0.0 if self.stable_since is None else max(0.0, now - self.stable_since)
        full = stable and stable_s >= 1.0 and self.failure is None
        partial = 0.50 if recovered else 0.25 if ejected_now else 0.0
        score = 0.0 if self.failure is not None else 1.0 if full else partial

        self.last = SpacerScoreSnapshot(
            mediated_push=self.push_event,
            verified_push_m=self.verified_push_m,
            ejected=ejected_now,
            recovered=recovered,
            tool_parked=tool_parked,
            stable=stable,
            stable_s=stable_s,
            released=bool(released),
            failure=self.failure,
            score=score,
            success_ready=full,
        )
        self.previous_tool = np.asarray(tool.pos, dtype=float).copy()
        self.previous_spacer = np.asarray(spacer.pos, dtype=float).copy()
        self.previous_tool_geometry = self._geometry(tool)
        self.previous_spacer_geometry = self._geometry(spacer)
        self.previous_time = now
        return self.last


class _ArrayState:
    """Minimal state view for validating contact at the preceding frame."""

    def __init__(self, geometry: dict[str, np.ndarray]) -> None:
        for name, value in geometry.items():
            setattr(self, name, value)
