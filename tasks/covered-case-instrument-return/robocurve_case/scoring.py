"""Private, simulator-independent scoring contract. Never expose snapshots to policy.

Units are metres, seconds and radians. Quaternions are wxyz. This evaluates
simulator state sampled at the 20 Hz control rate, not visual understanding.
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from itertools import permutations, product
from typing import Mapping

import numpy as np


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(frozen=True)
class Thresholds:
    containment_tolerance: float = 0.0015
    support_tolerance: float = 0.002
    orientation_tolerance_rad: float = np.deg2rad(10.0)
    linear_speed_max: float = 0.015
    angular_speed_max: float = 0.10
    release_distance_min: float = 0.008
    gripper_near_distance: float = 0.060
    release_opening_min: float = 0.045
    stability_hold_s: float = 1.0
    channel_hold_s: float = 0.20
    pickup_height: float = 0.020
    expulsion_margin: float = 0.005
    occupied_xy_overlap_min: float = 0.010
    occupied_intrusion_depth: float = 0.003
    workspace_tolerance: float = 0.010
    max_sample_dt: float = 0.10


DEFAULT_THRESHOLDS = Thresholds()


@dataclass(frozen=True)
class Pose:
    position: Vec3
    quaternion_wxyz: Quat = (1.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class ModuleState:
    pose: Pose
    size: Vec3
    linear_velocity: Vec3 = (0.0, 0.0, 0.0)
    angular_velocity: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class CradleState:
    pose: Pose
    interior_min: Vec3
    interior_max: Vec3
    support_z: float = 0.0
    linear_velocity: Vec3 = (0.0, 0.0, 0.0)
    angular_velocity: Vec3 = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class GripperState:
    # Minimum surface distance over ALL robot fingers/palm collision shapes.
    # Penetration is negative; TCP or finger-centre distances are insufficient.
    min_distance_to_intake: float
    opening: float


@dataclass(frozen=True)
class Snapshot:
    modules: Mapping[str, ModuleState]
    cradles: Mapping[str, CradleState]
    channel_open: Mapping[str, bool]
    grippers: tuple[GripperState, GripperState]
    mechanism_intact: bool
    workspace_min: Vec3
    workspace_max: Vec3
    dt: float
    # Adapter reports forbidden-solid crossing, missing lids, derailment, etc.
    # These are observed violations, never policy-supplied declarations.
    geometry_violations: tuple[str, ...] = ()
    # Covered task only: each lid is released, stationary and seated in the
    # matching physical rim. None for tasks without loose lids.
    lid_seated: Mapping[str, bool] | None = None


@dataclass(frozen=True)
class ScoreResult:
    score: float
    success_ready: bool
    stable_s: float
    stage: str
    violations: tuple[str, ...]
    termination_reason: str | None = None

    @property
    def success(self) -> bool:
        return self.score == 1.0 and self.termination_reason == "success"


def _vec(value, n: int) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (n,) or not np.isfinite(result).all():
        raise ValueError(f"Expected {n} finite coordinates")
    return result


def rotation(pose: Pose) -> np.ndarray:
    q = _vec(pose.quaternion_wxyz, 4)
    norm = np.linalg.norm(q)
    if abs(norm - 1.0) > 1e-3:
        raise ValueError("Quaternion must have unit norm (wxyz)")
    w, x, y, z = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def world_corners(module: ModuleState) -> np.ndarray:
    local = np.array(list(product((-0.5, 0.5), repeat=3))) * _vec(module.size, 3)
    return local @ rotation(module.pose).T + _vec(module.pose.position, 3)


def local_corners(module: ModuleState, cradle: CradleState) -> np.ndarray:
    return (world_corners(module) - _vec(cradle.pose.position, 3)) @ rotation(cradle.pose)


@lru_cache(maxsize=32)
def _box_symmetries(size: Vec3) -> tuple[np.ndarray, ...]:
    """Proper rotations of an unmarked box; only exactly equal axes may swap.

    These re-label the same solid, rather than relaxing placement tolerances.
    A cube has 24, a square prism 8, and a distinct-sided box 4 symmetries.
    Marked/keyed objects require a different, explicitly restricted contract.
    """
    result = []
    for permutation in permutations(range(3)):
        if any(size[i] != size[permutation[i]] for i in range(3)):
            continue
        for signs in product((-1, 1), repeat=3):
            symmetry = np.eye(3)[:, permutation] * signs
            if np.linalg.det(symmetry) > 0:
                symmetry.setflags(write=False)
                result.append(symmetry)
    return tuple(result)


def is_placed(module: ModuleState, cradle: CradleState,
              thresholds: Thresholds = DEFAULT_THRESHOLDS) -> bool:
    """Containment and four-corner support, modulo unmarked-box symmetry."""
    corners = local_corners(module, cradle)
    low, high = _vec(cradle.interior_min, 3), _vec(cradle.interior_max, 3)
    contained = np.all(corners >= low - thresholds.containment_tolerance) and np.all(
        corners <= high + thresholds.containment_tolerance)
    if not contained:
        return False
    cradle_rotation = rotation(cradle.pose)
    relative = cradle_rotation.T @ rotation(module.pose)
    centre = (_vec(module.pose.position, 3) - _vec(cradle.pose.position, 3)) @ cradle_rotation
    # For each equivalent body frame, this is an actual four-corner box face.
    # Never replace face support with a single lowest corner or edge contact.
    bottom = np.array([(x, y, -.5) for x, y in product((-.5, .5), repeat=2)]) * module.size
    for symmetry in _box_symmetries(tuple(module.size)):
        equivalent = relative @ symmetry
        angle = np.arccos(np.clip((np.trace(equivalent)-1)/2, -1, 1))
        if angle <= thresholds.orientation_tolerance_rad:
            heights = (bottom @ equivalent.T + centre)[:, 2]
            if np.all(np.abs(heights - cradle.support_z) <= thresholds.support_tolerance):
                return True
    return False


def completely_outside_cradle(module: ModuleState, cradle: CradleState,
                               margin: float) -> bool:
    """Strict OBB separation from the margin-expanded cradle-local volume.

    Use all 15 SAT axes, including module normals and edge cross-products.
    A positive separating gap above roundoff means fully expelled; touching
    or overlapping the expanded volume does not. Box symmetries relabel the
    same geometry, so no placement-orientation requirement belongs here.
    """
    if not np.isfinite(margin) or margin < 0:
        raise ValueError("Expulsion margin must be finite and nonnegative")
    low, high = _vec(cradle.interior_min, 3), _vec(cradle.interior_max, 3)
    frame = rotation(cradle.pose)
    relative = frame.T @ rotation(module.pose)
    centre = ((_vec(module.pose.position, 3) - _vec(cradle.pose.position, 3)) @ frame
              - (low + high) / 2)
    axes = np.concatenate((np.eye(3), relative.T,
        np.cross(np.eye(3)[:, None, :], relative.T[None, :, :]).reshape(-1, 3)))
    lengths = np.linalg.norm(axes, axis=1)
    axes = axes[lengths > 1e-10] / lengths[lengths > 1e-10, None]
    radius = (np.abs(axes) @ ((high - low) / 2 + margin)
              + np.abs(axes @ relative) @ (_vec(module.size, 3) / 2))
    return bool(np.any(np.abs(axes @ centre) - radius > 1e-12))


def enters_occupied_cradle(module: ModuleState, cradle: CradleState,
                           thresholds: Thresholds = DEFAULT_THRESHOLDS) -> bool:
    """Detect meaningful incoming-object entry into an initially occupied bay.

    Require at least 10 mm overlap in both horizontal axes and 3 mm below the
    cavity top. This catches a 40 mm cube stacked on an existing 40 mm cube
    inside a 44 mm-high cavity; passing over the case cannot trigger it.
    The adapter checks this after each 20 Hz control step.
    """
    corners = local_corners(module, cradle)
    low, high = _vec(cradle.interior_min, 3), _vec(cradle.interior_max, 3)
    lo, hi = corners.min(axis=0), corners.max(axis=0)
    overlap_xy = np.minimum(hi[:2], high[:2]) - np.maximum(lo[:2], low[:2])
    return bool(np.all(overlap_xy >= thresholds.occupied_xy_overlap_min)
                and lo[2] <= high[2] - thresholds.occupied_intrusion_depth
                and hi[2] >= low[2])


class ReturnScorer:
    """One instance per episode stream; call reset before update.

    Occupancy is private ground truth. Score aggregation is END, never BEST.
    update() cannot return 1; finalize('success') requires success_ready.
    """

    def __init__(self, initial_occupancy: Mapping[str, str | None],
                 intake_id: str = "incoming", *, max_steps: int = 2400,
                 thresholds: Thresholds = DEFAULT_THRESHOLDS,
                 covered_search: bool = False):
        self.occupancy = dict(initial_occupancy)
        empty = [key for key, value in self.occupancy.items() if value is None]
        stored = [value for value in self.occupancy.values() if value is not None]
        if len(self.occupancy) not in (2, 3) or len(empty) != 1:
            raise ValueError("Core layouts require 2 or 3 cradles and exactly one empty cradle")
        if len(set(stored)) != len(stored) or intake_id in stored:
            raise ValueError("Each stored module must have its own unique cradle")
        if not isinstance(max_steps, int) or max_steps <= 0:
            raise ValueError("max_steps must be a positive integer")
        self.target = empty[0]
        self.covered_search = covered_search
        self._case_order = tuple(self.occupancy)
        if covered_search and len(self._case_order) != 3:
            raise ValueError("Covered search requires exactly three cases")
        self.intake_id = intake_id
        self.max_steps = max_steps
        self.t = thresholds
        values = tuple(vars(thresholds).values())
        if not np.isfinite(values).all() or any(value <= 0 for value in values):
            raise ValueError("All scoring thresholds must be finite and positive")
        if thresholds.release_distance_min >= thresholds.gripper_near_distance:
            raise ValueError("Near-gripper range must exceed the release distance")
        self._initialized = False

    def _validate(self, snapshot: Snapshot, *, initial: bool = False) -> None:
        if set(snapshot.cradles) != set(self.occupancy) or set(snapshot.channel_open) != set(self.occupancy):
            raise ValueError("Snapshot cradle/channel IDs must match occupancy")
        expected = {self.intake_id} | {v for v in self.occupancy.values() if v is not None}
        if set(snapshot.modules) != expected:
            raise ValueError("Snapshot must contain exactly the incoming and stored modules")
        if len(snapshot.grippers) != 2:
            raise ValueError("Both robot grippers are required")
        if self.covered_search and (snapshot.lid_seated is None or
                                    set(snapshot.lid_seated) != set(self.occupancy) or
                                    any(not isinstance(v, bool) for v in snapshot.lid_seated.values())):
            raise ValueError("Covered search requires one physical seated flag per lid")
        if not np.isfinite(snapshot.dt) or snapshot.dt < 0 or snapshot.dt > self.t.max_sample_dt:
            raise ValueError("dt must be finite and within [0, max_sample_dt]")
        if not initial and snapshot.dt <= 0:
            raise ValueError("update requires positive elapsed simulation time")
        if not isinstance(snapshot.mechanism_intact, bool) or any(
            not isinstance(value, bool) for value in snapshot.channel_open.values()
        ):
            raise ValueError("Mechanism/channel flags must be explicit booleans")
        low, high = _vec(snapshot.workspace_min, 3), _vec(snapshot.workspace_max, 3)
        if np.any(low >= high):
            raise ValueError("Invalid workspace bounds")
        for module in snapshot.modules.values():
            world_corners(module)
            if np.any(_vec(module.size, 3) <= 0):
                raise ValueError("Module sizes must be positive")
            _vec(module.linear_velocity, 3)
            _vec(module.angular_velocity, 3)
        for cradle in snapshot.cradles.values():
            _vec(cradle.pose.position, 3)
            rotation(cradle.pose)
            if np.any(_vec(cradle.interior_min, 3) >= _vec(cradle.interior_max, 3)):
                raise ValueError("Invalid cradle interior")
            if not np.isfinite(cradle.support_z):
                raise ValueError("Invalid support plane")
            if not cradle.interior_min[2] <= cradle.support_z < cradle.interior_max[2]:
                raise ValueError("Support plane must lie within the cradle interior")
            _vec(cradle.linear_velocity, 3)
            _vec(cradle.angular_velocity, 3)
        for gripper in snapshot.grippers:
            if not np.isfinite([gripper.min_distance_to_intake, gripper.opening]).all() or gripper.opening < 0:
                raise ValueError("Invalid gripper geometry")

    def reset(self, snapshot: Snapshot) -> ScoreResult:
        self._validate(snapshot, initial=True)
        if not snapshot.mechanism_intact or snapshot.geometry_violations:
            raise ValueError("Initial mechanisms must be intact")
        if any(snapshot.channel_open.values()):
            raise ValueError("Core layouts must start with all channels closed")
        if self.covered_search and not all(snapshot.lid_seated.values()):
            raise ValueError("Covered search must start with all three lids seated")
        for key, module_id in self.occupancy.items():
            if module_id is not None and not is_placed(snapshot.modules[module_id], snapshot.cradles[key], self.t):
                raise ValueError("Every stored module must start supported in its own cradle")
        incoming = snapshot.modules[self.intake_id]
        if any(is_placed(incoming, cradle, self.t) for cradle in snapshot.cradles.values()):
            raise ValueError("Incoming module must start on a separate intake pad")
        if np.any(world_corners(incoming) < np.array(snapshot.workspace_min)) or np.any(
            world_corners(incoming) > np.array(snapshot.workspace_max)
        ):
            raise ValueError("Incoming module must start fully inside the workspace")
        self._initial_bottom = float(world_corners(incoming)[:, 2].min())
        self._sizes = {key: tuple(value.size) for key, value in snapshot.modules.items()}
        self._cradle_bounds = {key: (tuple(value.interior_min), tuple(value.interior_max), value.support_z)
                               for key, value in snapshot.cradles.items()}
        self._workspace = (tuple(snapshot.workspace_min), tuple(snapshot.workspace_max))
        self._violations: set[str] = set()
        self._open_s = self._stable_s = 0.0
        self._was_stable = False
        self._opened = False
        self._case_open_s = {key: 0.0 for key in self._case_order}
        self._case_seen_open = {key: False for key in self._case_order}
        self._steps = 0
        self._finalized = False
        self._initialized = True
        self.result = ScoreResult(0.0, False, 0.0, "initial", ())
        return self.result

    def update(self, snapshot: Snapshot) -> ScoreResult:
        if not self._initialized or self._finalized:
            raise RuntimeError("reset is required before a new episode")
        self._validate(snapshot)
        self._steps += 1
        for key, value in snapshot.modules.items():
            if tuple(value.size) != self._sizes[key]:
                raise ValueError("Module dimensions cannot change within an episode")
        if (tuple(snapshot.workspace_min), tuple(snapshot.workspace_max)) != self._workspace:
            raise ValueError("Workspace cannot change within an episode")
        for key, value in snapshot.cradles.items():
            if (tuple(value.interior_min), tuple(value.interior_max), value.support_z) != self._cradle_bounds[key]:
                raise ValueError("Cradle interior cannot change within an episode")
        if not snapshot.mechanism_intact:
            self._violations.add("mechanism_not_intact")
        self._violations.update(snapshot.geometry_violations)
        all_stored_placed = True
        for key, module_id in self.occupancy.items():
            if module_id is None:
                continue
            module, cradle = snapshot.modules[module_id], snapshot.cradles[key]
            if completely_outside_cradle(module, cradle, self.t.expulsion_margin):
                self._violations.add(f"stored_module_expelled:{module_id}")
            all_stored_placed &= is_placed(module, cradle, self.t)
        incoming, target = snapshot.modules[self.intake_id], snapshot.cradles[self.target]
        for key, module_id in self.occupancy.items():
            if module_id is not None and enters_occupied_cradle(incoming, snapshot.cradles[key], self.t):
                self._violations.add(f"incoming_entered_occupied:{key}")
        corners = world_corners(incoming)
        if np.any(corners < np.array(snapshot.workspace_min)-self.t.workspace_tolerance) or np.any(
            corners > np.array(snapshot.workspace_max)+self.t.workspace_tolerance
        ):
            self._violations.add("incoming_outside_workspace")
        self._open_s = self._open_s + snapshot.dt if snapshot.channel_open[self.target] else 0.0
        open_now = self._open_s + 1e-9 >= self.t.channel_hold_s
        self._opened |= open_now
        if self.covered_search:
            for key in self._case_order:
                self._case_open_s[key] = (self._case_open_s[key] + snapshot.dt
                                          if snapshot.channel_open[key] else 0.0)
                if self._case_open_s[key] + 1e-9 >= self.t.channel_hold_s:
                    self._case_seen_open[key] = True
        placed = is_placed(incoming, target, self.t)
        released = all(
            g.min_distance_to_intake >= self.t.release_distance_min
            and (g.min_distance_to_intake >= self.t.gripper_near_distance
                 or g.opening >= self.t.release_opening_min)
            for g in snapshot.grippers
        )
        cradle_velocity_at_module = np.array(target.linear_velocity) + np.cross(
            np.array(target.angular_velocity), np.array(incoming.pose.position)-np.array(target.pose.position))
        slow = (np.linalg.norm(np.array(incoming.linear_velocity)-cradle_velocity_at_module)
                <= self.t.linear_speed_max and
                np.linalg.norm(np.array(incoming.angular_velocity)-np.array(target.angular_velocity))
                <= self.t.angular_speed_max)
        lids_final = bool(not self.covered_search or all(snapshot.lid_seated.values()))
        valid = bool(self._opened and placed and released and slow and all_stored_placed
                     and lids_final
                     and not self._violations and self._steps <= self.max_steps)
        self._stable_s = self._stable_s + snapshot.dt if valid and self._was_stable else 0.0
        self._was_stable = valid
        ready = valid and self._stable_s + 1e-9 >= self.t.stability_hold_s
        lifted = float(corners[:, 2].min()) >= self._initial_bottom + self.t.pickup_height
        if self._violations:
            score, stage = 0.0, "disqualified"
        elif self.covered_search and self._opened and placed and released and all_stored_placed:
            score, stage = 0.75, "covered_placed_pending_lids"
        elif self.covered_search and sum(self._case_seen_open.values()) >= 2:
            score, stage = 0.5, "covered_two_cases_opened"
        elif self.covered_search and any(self._case_seen_open.values()):
            score, stage = 0.25, "covered_case_opened"
        elif self.covered_search:
            score, stage = 0.0, "no_progress"
        elif self._opened and placed:
            score, stage = 0.75, "placed_pending_success"
        elif open_now and lifted:
            score, stage = 0.5, "picked_up"
        elif open_now:
            score, stage = 0.25, "channel_open"
        else:
            score, stage = 0.0, "no_progress"
        self.result = ScoreResult(score, ready, self._stable_s, stage, tuple(sorted(self._violations)))
        return self.result

    def finalize(self, termination_reason: str) -> ScoreResult:
        if not self._initialized or self._finalized:
            raise RuntimeError("Episode is not active")
        if termination_reason == "success" and not self.result.success_ready:
            raise ValueError("Cannot terminate successfully without a continuous valid hold")
        self._finalized = True
        self.result = ScoreResult(
            1.0 if termination_reason == "success" else min(self.result.score, 0.75),
            self.result.success_ready, self.result.stable_s,
            "success" if termination_reason == "success" else self.result.stage,
            self.result.violations, termination_reason,
        )
        return self.result
