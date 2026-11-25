"""Taut ribbon paths supported by the outside of fixed ellipsoids."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Ellipsoid:
    name: str
    centre: tuple[float, float, float]
    radii: tuple[float, float, float]
    color: tuple[float, float, float]

    def mesh(self):
        """Triangulate the same convex surface used by the routing constraints."""
        from scipy.spatial import ConvexHull
        from bodykit import MeshShape

        theta, phi = np.meshgrid(np.arange(20) * (2 * np.pi / 20), np.arange(1, 13) * (np.pi / 13))
        points = np.column_stack(((np.sin(phi) * np.cos(theta)).ravel(),
                                  (np.sin(phi) * np.sin(theta)).ravel(), np.cos(phi).ravel()))
        points = np.concatenate([points, [[0, 0, 1], [0, 0, -1]]])
        faces = ConvexHull(points).simplices.copy()
        triangles = points[faces]
        normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        inward = (normal * triangles[:, 0]).sum(axis=1) < 0
        faces[inward] = faces[inward, ::-1]
        normals = points / self.radii
        normals /= np.linalg.norm(normals, axis=1, keepdims=True)
        return MeshShape(np.asarray(self.centre) + points * self.radii, faces, normals=normals,
                         color=self.color, approximation="convexHull")


@dataclass
class Route:
    points: np.ndarray
    surfaces: tuple[Ellipsoid, ...]
    axis: int
    sign: int

    @property
    def length(self) -> float:
        return float(np.linalg.norm(np.diff(self.points, axis=0), axis=1).sum())

    @property
    def pull_direction(self) -> np.ndarray:
        direction = self.points[-2] - self.points[-1]
        return direction / max(float(np.linalg.norm(direction)), 1e-9)

    def near(self, surface: Ellipsoid, clearance: float) -> np.ndarray:
        scaled = (self.points - surface.centre) / surface.radii
        distance = (np.linalg.norm(scaled, axis=1) - 1) * min(surface.radii)
        return np.abs(distance) <= clearance

    def near_length(self, surface: Ellipsoid, clearance: float) -> float:
        near = self.near(surface, clearance)
        segments = np.diff(self.points, axis=0)[near[:-1] & near[1:]]
        return float(np.linalg.norm(segments, axis=1).sum())

    def ribbon(self, width: float, slot_normal: np.ndarray) -> np.ndarray:
        """Lay the ribbon flat against supporting surfaces and turn it into the tongue's slot."""
        tangent = np.gradient(self.points, axis=0)
        tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-9)
        normals = np.zeros_like(tangent)
        normals[:, self.axis] = self.sign
        if abs(normals[0] @ tangent[0]) > 0.85:
            normals[:] = [0.0, 1.0, 0.0]
        last = normals.copy()
        for surface in self.surfaces:
            near = self.near(surface, 0.012)
            local = self.points[near] - surface.centre
            gradient = local / np.square(surface.radii)
            normals[near] = gradient / np.maximum(np.linalg.norm(gradient, axis=1, keepdims=True), 1e-9)
        for offset in range(1, min(7, len(normals))):
            frac = offset / 7
            normals[-offset] = slot_normal * (1 - frac) + normals[-offset] * frac
        across = np.cross(tangent, normals)
        weak = np.linalg.norm(across, axis=1) < 0.1
        across[weak] = np.cross(tangent[weak], last[weak])
        across /= np.maximum(np.linalg.norm(across, axis=1, keepdims=True), 1e-9)
        # A ribbon's edge labels can be swapped whenever needed to avoid a 180-degree visual twist.
        flips = np.einsum("ij,ij->i", across[1:], across[:-1]) < 0
        across[1:] *= np.cumprod(np.where(flips, -1, 1))[:, None]
        delta = 0.5 * width * across
        return np.stack((self.points - delta, self.points + delta), axis=1).reshape(-1, 3)


def taut_route(
    start: np.ndarray,
    end: np.ndarray,
    surfaces: tuple[Ellipsoid, ...],
    axis: int,
    sign: int,
    clearance: float = 0.004,
    samples: int = 45,
) -> Route:
    """Find the shortest path in the plane of the chord and preferred routing direction.

    Each obstacle cuts an ellipse into the plane. The belt is kept on the selected side,
    and supporting tangents bridge the concave hull of the sampled envelope.
    """
    fractions = np.linspace(0.0, 1.0, samples)
    chord = start + (end - start) * fractions[:, None]
    heights = chord[:, axis] * sign
    envelope = heights.copy()
    other_axes = [i for i in range(3) if i != axis]
    active = []
    for surface in surfaces:
        radii = np.asarray(surface.radii) + clearance
        scaled = (chord - surface.centre) / radii
        delta = scaled[-1] - scaled[0]
        closest_t = np.clip(-np.dot(scaled[0], delta) / max(float(delta @ delta), 1e-12), 0.0, 1.0)
        closest = scaled[0] + closest_t * delta
        if closest @ closest >= 1.0:
            continue
        active.append(surface)
        cross_section = 1.0 - np.square(scaled[:, other_axes]).sum(axis=1)
        border = sign * surface.centre[axis] + radii[axis] * np.sqrt(np.maximum(0.0, cross_section))
        envelope = np.maximum(envelope, np.where(cross_section >= 0, border, -np.inf))
    if not active:
        return Route(chord, (), axis, sign)
    envelope[[0, -1]] = heights[[0, -1]]
    hull = []
    for i, height in enumerate(envelope):
        while len(hull) >= 2:
            a, b = hull[-2:]
            if (envelope[b] - envelope[a]) * (i - b) > (height - envelope[b]) * (b - a):
                break
            hull.pop()
        hull.append(i)
    routed = chord.copy()
    routed[:, axis] = sign * np.interp(fractions, fractions[hull], envelope[hull])
    return Route(routed, tuple(active), axis, sign)
