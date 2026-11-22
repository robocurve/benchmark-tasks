"""Pure geometry checks for the IV task's aseptic field.

Once a protective cap is off, the exposed sterile interface (the bag's open port mouth, or the giving set's spike) must not
touch anything nonsterile before the spike is locked in the port: any gripper surface, the work tray, or any collidable
surface of the presentation rack, the IV support or the bag's outside. Nonsterile surfaces other than the grippers and the
tray are given as oriented boxes and cylinders, generated from the bodies' own collision shapes (``obstacles_from_body``).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


def point_segment_distance(point, a, b) -> float:
    point, a, b = (np.asarray(v, float) for v in (point, a, b))
    ab = b - a
    if float(ab @ ab) < 1e-12:
        return float(np.linalg.norm(point - a))
    t = float(np.clip(((point - a) @ ab) / (ab @ ab), 0.0, 1.0))
    return float(np.linalg.norm(point - (a + t * ab)))


@dataclass(frozen=True)
class BoxObstacle:
    """A nonsterile box: world centre, world-from-box rotation, half extents (m)."""

    name: str
    center: np.ndarray
    rot: np.ndarray
    half: np.ndarray

    def distance(self, point) -> float:
        """Signed distance from a point to the box surface (negative inside)."""
        q = self.rot.T @ (np.asarray(point, float) - self.center)
        d = np.abs(q) - self.half
        return float(np.linalg.norm(np.maximum(d, 0.0)) + min(float(d.max()), 0.0))


@dataclass(frozen=True)
class CylinderObstacle:
    """A nonsterile solid cylinder: world centre, unit axis, radius and half length (m)."""

    name: str
    center: np.ndarray
    axis: np.ndarray
    radius: float
    half_length: float

    def distance(self, point) -> float:
        v = np.asarray(point, float) - self.center
        t = float(v @ self.axis)
        dr = float(np.linalg.norm(v - t * self.axis)) - self.radius
        dz = abs(t) - self.half_length
        return float(np.hypot(max(dr, 0.0), max(dz, 0.0)) + min(max(dr, dz), 0.0))


def segment_distance(obstacle, a, b, step: float = 0.0005) -> float:
    """Distance from a segment to an obstacle, sampled every ``step`` m along it (ends included)."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    n = max(2, int(np.ceil(float(np.linalg.norm(b - a)) / step)) + 1)
    return min(obstacle.distance(a + (b - a) * k / (n - 1)) for k in range(n))


def obstacles_from_body(body, pos, rot, skip=lambda shape: False) -> list:
    """Every collidable box and cylinder of a bodykit Body, placed at world ``pos`` with world-from-body ``rot``."""
    from bodykit import BoxShape, CylShape, rotation

    out = []
    pos, rot = np.asarray(pos, float), np.asarray(rot, float)
    for k, s in enumerate(body.shapes):
        if not s.collide or skip(s):
            continue
        centre = pos + rot @ np.asarray(s.centre, float)
        name = f"{body.name}[{k}]"
        if isinstance(s, BoxShape):
            out.append(BoxObstacle(name, centre, rot @ rotation(s.quat()), np.asarray(s.size, float) / 2))
        elif isinstance(s, CylShape):
            axis = rot @ np.eye(3)["XYZ".index(s.axis)]
            out.append(CylinderObstacle(name, centre, axis, float(s.radius), float(s.height) / 2))
    return out


@dataclass
class AsepsisMonitor:
    """Latch contamination after either sterile interface becomes exposed."""

    contaminated: bool = False
    tray_clearance: float = 0.0005
    clearance: float = 0.0005  # a sterile surface this close to a nonsterile obstacle counts as touching it
    cause: str | None = field(default=None)

    def _latch(self, cause: str) -> None:
        self.contaminated, self.cause = True, cause

    def observe(
        self,
        *,
        port_open: bool,
        spike_exposed: bool,
        spike_locked: bool,
        port_center,
        port_radius: float,
        spike_base,
        spike_tip,
        spike_radius: float,
        gripper_surfaces,
        tray_top: float,
        gripper_contact: bool = False,
        obstacles=(),
    ) -> None:
        if self.contaminated:
            return
        if gripper_contact:
            self._latch("gripper")
            return
        port_center = np.asarray(port_center, float)
        spike_base, spike_tip = np.asarray(spike_base, float), np.asarray(spike_tip, float)
        if port_open and not spike_locked:
            if port_center[2] - port_radius <= tray_top + self.tray_clearance:
                self._latch("port on tray")
                return
            # Each tuple is the measured centre and conservative collision radius of one physical palm/finger link.
            # Expanding the sterile interface by that radius tests surface-to-surface contact, not TCP distance.
            if any(np.linalg.norm(np.asarray(c, float) - port_center) <= port_radius + float(r)
                   for c, r in gripper_surfaces):
                self._latch("port on gripper")
                return
            for ob in obstacles:
                if ob.distance(port_center) <= port_radius + self.clearance:
                    self._latch(f"port on {ob.name}")
                    return
        if spike_exposed and not spike_locked:
            if min(spike_base[2], spike_tip[2]) - spike_radius <= tray_top + self.tray_clearance:
                self._latch("spike on tray")
                return
            if any(point_segment_distance(c, spike_base, spike_tip) <= spike_radius + float(r)
                   for c, r in gripper_surfaces):
                self._latch("spike on gripper")
                return
            for ob in obstacles:
                if segment_distance(ob, spike_base, spike_tip) <= spike_radius + self.clearance:
                    self._latch(f"spike on {ob.name}")
                    return
