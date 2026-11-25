"""What a yam_pair task file imports. Pure Python: importing it needs no Isaac Sim.

    from yam_pair.task import Box, Cylinder, Sphere, Asset, UsdBody, Shape, Layout, TABLE_TOP, Waypoint

See CONTRACT.md for the whole task-file contract.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, READY
from yam_arena.demo import Waypoint, grip_rotation, pick_place_waypoints
from yam_arena.task_file import Asset, Box, Cylinder, ObjectState, Sphere

from yam_pair.layout import SIDES, STATIONS, TABLE_TOP, Layout

__all__ = [
    "FINGER_CLOSED",
    "FINGER_OPEN",
    "READY",
    "SIDES",
    "STATIONS",
    "TABLE_TOP",
    "Asset",
    "BodyState",
    "Box",
    "Cylinder",
    "Layout",
    "ObjectState",
    "PairState",
    "Shape",
    "Sphere",
    "UsdBody",
    "Waypoint",
    "grip_rotation",
    "pick_place_waypoints",
]


@dataclass(frozen=True)
class UsdBody(Box):
    """A body from ``usd`` (relative to the task file's folder). ``pos``/``rot_xyzw`` place the file's origin (world).
    ``size`` is only nominal. With ``mass=None`` a rigid body is kinematic; an articulation keeps the fixed base its
    file declares. Same fields as scenekit's UsdBody."""

    usd: str = ""
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    articulation: bool = False
    joint_pos: tuple[tuple[str, float], ...] = ()
    joint_damping: float = 0.03
    drives: tuple[tuple, ...] = ()  # (joint name or regex, stiffness, damping[, effort limit])
    friction: tuple[float, float] = (0.8, 0.7)  # static, dynamic
    friction_min: bool = False
    solver_iters: int = 16
    torsional_patch: float | None = None

    def _check(self) -> None:  # the file is checked by the loader, relative to the task's folder
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if not self.usd:
            raise ValueError(f"{self.name}: usd= is required")


@dataclass(frozen=True)
class Shape(Box):
    """A fixed box (``size``) or capsule (``radius``, ``height`` of its cylinder, along ``axis``), posed with
    ``rot_xyzw`` (world). It collides unless ``collide`` is False. Same fields as scenekit's Shape."""

    kind: str = "box"
    radius: float = 0.01
    height: float = 0.02
    axis: str = "Z"
    rot_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    collide: bool = True

    def _check(self) -> None:
        if self.kind not in ("box", "capsule"):
            raise ValueError(f"{self.name}: kind must be 'box' or 'capsule'")
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Shape is fixed (no mass)")


@dataclass(frozen=True)
class BodyState(ObjectState):
    """A UsdBody's origin pose, velocities and (articulations) joint positions."""

    joints: dict = field(default_factory=dict)


@dataclass
class PairState:
    """Both stations after a control step, for score()/success()/distance()/report(). See CONTRACT.md §2."""

    step: int
    t: float
    contact_n: float
    contact_peak_n: float
    contact_by_arm: dict[str, float]
    arms: dict[str, np.ndarray]
    layout: Layout
    env: Any = None

    def tcp(self, station: str, side: str) -> np.ndarray:
        """World position of the arm's tool point (between the finger pads), from its measured joints (URDF FK)."""
        from yam_arena import ik
        from yam_arena.station import BASES

        q = np.asarray(self.arms[station], float)
        joints = q[0:6] if side == "left" else q[7:13]
        pos, _ = ik.tcp_pose(joints, BASES[side])
        return self.layout.to_world(station, pos)
