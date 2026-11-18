"""Finds the card in the top camera's image, for the scripted demo. NumPy only (the camera model comes from yam_arena).

The card is the only yellow thing on the table. Its pixels are picked by colour, back-projected onto the card's plane
with the station's calibrated top camera (the same intrinsics and pose the real station is calibrated with), and
weighted by the table area each pixel covers. The weighted centroid gives the card's centre and the principal axis
its long edge, so its yaw. No simulator object state is read.
"""

from __future__ import annotations

import math

import numpy as np

from . import geometry as g

ROI_X = (0.06, 0.28)          # table area searched: the stand and its surroundings, in front of the punch's throat
ROI_Y = (-0.15, 0.10)
MIN_AREA = 0.5 * g.CARD_X * g.CARD_Y  # fewer yellow square metres than half a card: not found


def quat_xyzw_to_matrix(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TopCamera:
    """Pinhole camera, OpenGL convention (looks along its -z, image up is its +y), pose in the station frame."""

    def __init__(self, pos, rot_xyzw, fx, fy, cx, cy):
        self.pos = np.asarray(pos, float)
        self.R = quat_xyzw_to_matrix(rot_xyzw)
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)

    @classmethod
    def station(cls, width: int, height: int, variant: str = "calibrated") -> TopCamera:
        from yam_arena import station

        cam = station.top_camera(variant)
        lens = cam.lens.resized(width, height)
        return cls(cam.pos, cam.rot_xyzw, lens.fx, lens.fy, lens.cx, lens.cy)

    def project(self, xyz):
        """World points (N, 3) -> pixel coordinates (N, 2) as (u, v), continuous (pixel i spans i..i+1)."""
        p = (np.asarray(xyz, float) - self.pos) @ self.R  # camera frame
        depth = -p[:, 2]
        return np.stack([self.cx + self.fx * p[:, 0] / depth, self.cy - self.fy * p[:, 1] / depth], axis=1)

    def to_plane(self, u, v, z):
        """Pixel coordinates -> (x, y) on the horizontal plane at height z."""
        u, v = np.asarray(u, float), np.asarray(v, float)
        d = np.stack([(u - self.cx) / self.fx, -(v - self.cy) / self.fy, -np.ones_like(u)])
        d = self.R @ d
        s = (z - self.pos[2]) / d[2]
        return np.stack([self.pos[0] + s * d[0], self.pos[1] + s * d[1]], axis=1)


def yellow_mask(rgb) -> np.ndarray:
    """Card-yellow pixels: hue 48-66 deg and strongly saturated. The card renders at hue 56-57 deg, saturation about
    0.7. The beige table in the arms' shadows renders at hue 35-39 deg, saturation under 0.5, so it is left out. The mat
    is green, the stand grey, the punch black."""
    c = np.asarray(rgb, float)[..., :3] / 255.0
    r, gr, b = c[..., 0], c[..., 1], c[..., 2]
    mx, mn = c.max(axis=-1), c.min(axis=-1)
    chroma = mx - mn
    hue = np.degrees(np.arctan2(math.sqrt(3) * (gr - b), 2 * r - gr - b)) % 360.0
    return (hue >= 48) & (hue <= 66) & (mx > 0.30) & (chroma > 0.55 * np.maximum(mx, 1e-6)) & (r >= b)


def find_card(rgb, camera: TopCamera, z: float = g.BED + g.T):
    """(x, y, yaw, yellow area m^2) of the card seen in ``rgb``, or None when no card-sized yellow patch is seen."""
    v, u = np.nonzero(yellow_mask(rgb))
    if u.size == 0:
        return None
    u, v = u + 0.5, v + 0.5
    xy = camera.to_plane(u, v, z)
    du = camera.to_plane(u + 1.0, v, z) - xy  # each pixel's footprint on the plane
    dv = camera.to_plane(u, v + 1.0, z) - xy
    area = np.abs(du[:, 0] * dv[:, 1] - du[:, 1] * dv[:, 0])
    keep = (xy[:, 0] > ROI_X[0]) & (xy[:, 0] < ROI_X[1]) & (xy[:, 1] > ROI_Y[0]) & (xy[:, 1] < ROI_Y[1])
    xy, area = xy[keep], area[keep]
    total = float(area.sum())
    if total < MIN_AREA:
        return None
    c = (xy * area[:, None]).sum(axis=0) / total
    d = xy - c
    cov = (d * area[:, None]).T @ d / total
    evals, evecs = np.linalg.eigh(cov)
    long_axis = evecs[:, int(np.argmax(evals))]  # along the card's 148 mm side (its local y)
    yaw = math.atan2(long_axis[1], long_axis[0]) - math.pi / 2
    yaw = (yaw + math.pi / 2) % math.pi - math.pi / 2  # the axis has no sign: keep yaw in -90..90 deg
    return float(c[0]), float(c[1]), float(yaw), total
