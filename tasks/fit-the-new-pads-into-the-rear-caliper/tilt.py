"""Tilted grips for yam_scripted demos. Copy this file next to your task file and import it there:

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import tilt  # noqa: E402

    tilt.Waypoint(1.5, (x, y, z), FINGER_CLOSED, "tip forward", yaw=0.0, tilt=45, heading=tilt.away(arm, (x, y)))

``tilt.Waypoint`` is the stock :class:`yam_arena.demo.Waypoint` with two more fields: ``tilt`` (degrees) leans the
gripper's approach axis off straight down, and ``heading`` (rad, world frame, 0 = +x away from the arms, pi/2 = +y)
is the horizontal direction the fingertips swing toward. ``yaw`` keeps its stock meaning (the grip turned about the
vertical before tilting). The lean is a rotation about the TCP, so a held object pivots about the point between the
pads. With ``tilt=0`` it is exactly a stock Waypoint. Mix both kinds freely in one plan: ArmPlan slerps between them.

How it works, and what it changes: the stock ``ArmPlan.target`` builds every waypoint's rotation by calling
``yam_arena.demo.grip_rotation(w.yaw)``. A tilted waypoint stores its full rotation on its ``yaw`` value (a float
subclass that still equals the yaw), and importing this module wraps ``grip_rotation`` so that it returns that
stored rotation when there is one and calls the original otherwise. This is a runtime patch of the stock package,
inside the task's own process only: nothing is changed on disk, plain floats behave exactly as before, and the
patch is checked at import (it raises if a future yam_arena no longer builds rotations this way).

Reach (offline IK, YAM URDF): leaning the fingertips *away* from the arm's base works to about 45 degrees almost
everywhere and to 60 at most points; leaning *toward* the base mostly fails past 20 degrees. Lower TCP heights
reach better. Check every plan with :func:`check_plan` (offline, no Isaac) before a GPU run.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
import yam_arena.demo as _demo
from yam_arena.urdf import axis_angle_matrix

# Arm bases on the station (world x, y); the tabletop scene's are at y +-0.30.
BASES = {"left": (0.0, 0.305), "right": (0.0, -0.305)}


class _TiltedYaw(float):
    """A yaw (rad) that also carries the full TCP rotation the tilted waypoint asks for."""

    rotation: np.ndarray

    def __new__(cls, yaw: float, rotation: np.ndarray):
        obj = super().__new__(cls, yaw)
        obj.rotation = np.array(rotation, dtype=float)
        return obj

    def __reduce__(self):
        return (_TiltedYaw, (float(self), self.rotation))


def rotation(yaw: float = 0.0, tilt: float = 0.0, heading: float = 0.0) -> np.ndarray:
    """TCP rotation: the stock straight-down grip turned ``yaw`` about the vertical, then its approach axis leaned
    ``tilt`` degrees from straight down toward the horizontal direction ``heading`` (rad, world)."""
    down = _ORIGINAL(float(yaw))
    if tilt == 0.0:
        return down
    toward = np.array([math.cos(heading), math.sin(heading), 0.0])
    axis = np.cross([0.0, 0.0, -1.0], toward)  # turns -z (the approach) toward `toward`
    return axis_angle_matrix(axis, math.radians(tilt)) @ down


def away(arm: str, xy) -> float:
    """Heading (rad) from ``arm``'s base to the point ``xy``: tilt toward it to lean the fingertips away from that
    base, the reachable direction."""
    bx, by = BASES[arm]
    return math.atan2(xy[1] - by, xy[0] - bx)


@dataclass
class Waypoint(_demo.Waypoint):
    tilt: float = 0.0  # degrees off straight down
    heading: float = 0.0  # rad, world: where the fingertips lean (0 = +x, away from the arms)

    def __post_init__(self) -> None:
        if not -90.0 <= self.tilt <= 90.0:
            raise ValueError(f"tilt must be within +-90 degrees, got {self.tilt!r}")
        yaw = float(self.yaw)
        self.yaw = yaw if self.tilt == 0.0 else _TiltedYaw(yaw, rotation(yaw, self.tilt, self.heading))


# -- the patch: grip_rotation honours a rotation carried on the yaw ---------------------------------------------

_ORIGINAL = getattr(_demo.grip_rotation, "_tilt_original", _demo.grip_rotation)


def _grip_rotation(yaw: float = 0.0) -> np.ndarray:
    rot = getattr(yaw, "rotation", None)
    return rot.copy() if rot is not None else _ORIGINAL(yaw)


_grip_rotation._tilt_original = _ORIGINAL  # type: ignore[attr-defined]
_demo.grip_rotation = _grip_rotation


def _self_check() -> None:
    """Fail loudly at import if the stock ArmPlan no longer takes its rotations from grip_rotation(w.yaw)."""
    w = Waypoint(1.0, (0.3, 0.3, 0.9), 0.0, "check", tilt=30.0)
    plan = _demo.ArmPlan([w], BASES["left"] + (0.75,))
    _, rot, _, _ = plan.target(10.0)
    if not np.allclose(rot, w.yaw.rotation, atol=1e-9):
        raise RuntimeError("tilt.py: yam_arena.demo.ArmPlan ignores tilted waypoints on this stack; do not use it")


_self_check()


def tilt_of(quat_xyzw) -> float:
    """Degrees an object's local +z axis leans off world vertical (for score() or a trace)."""
    x, y, z, w = (float(v) for v in quat_xyzw)
    zz = 1.0 - 2.0 * (x * x + y * y)  # world z of the object's local z axis
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def check_plan(plan: dict, bases: dict | None = None, dt: float = 0.05) -> dict[str, tuple[float, float]]:
    """Play a demo() plan through the stock ArmPlan IK offline; worst (position mm, rotation deg) error per arm.
    Keep both small (a few mm, a few degrees); a large error means that pose is out of reach."""
    from yam_arena import station

    bases = bases or station.BASES  # the station's; pass the tabletop's for SCENE = "tabletop"
    out = {}
    for arm, waypoints in plan.items():
        base = bases[arm]
        p = _demo.ArmPlan(list(waypoints), base)
        for k in range(int(p.duration / dt) + 2):
            p.action7(k * dt)
        out[arm] = (round(p.worst_ik_mm, 1), round(p.worst_ik_deg, 1))
    return out
