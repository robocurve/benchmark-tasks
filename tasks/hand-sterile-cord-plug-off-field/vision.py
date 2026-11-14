"""vision: camera-based estimates for scripted demos (robot-observable information only).

Reviewers reject demos that read object poses from the simulator ("Demo uses simulator-internal object state").
A demo may use the camera images (``ctx.obs.images``), the arms' own poses (``arm.tcp()``) and fixed layout a robot
would know from its setup. This module turns the top camera image into world (x, y) estimates on horizontal planes.

Symlink it into the task folder next to cablekit.py (``ln -s ../../cablekit/vision.py``) and ``import vision``.

API (all positions in the task's world frame, metres; images are HxWx3 uint8 RGB):

    cam = vision.top_camera(img.shape)          # calibrated station top camera, intrinsics resized to the image
    cam.project(xyz) -> (u, v)                  # world point -> pixel (for debug overlays)
    cam.to_plane(mask, z) -> (N, 2) xy          # every True pixel back-projected onto the plane at height z
    vision.top_image(ctx)                       # ctx.obs.images["top_cam"] as int16 RGB (safe for differences)
    vision.color_mask(img, rgb, tol)            # pixels within ``tol`` (RGB distance) of a target colour
    vision.hsv_mask(img, hue_deg, hue_tol, min_sat, min_val)
                                                # hue/saturation mask: robust to shading and to thin-cable blending
    vision.blob_xy(img, mask, z, min_px=8, stat="mean") -> xy | None
                                                # centre of a coloured blob on the plane at height z
    vision.trace_cable(img, mask, z, anchor_xy, ...) -> Polyline | None
                                                # ordered bead chain of a thin coloured cable from a known end
    poly.length, poly.points                    # visible length from the anchor; beads (N, 2), ~2 cm apart
    poly.at(s) -> (xy, heading) | None          # point s m along the cable from the anchor, heading towards anchor
    poly.snap(xy)                               # nearest cable centre (grasp points: beads cut tight bends)

Limits (read before relying on it):
- Resolution: recorded runs render 320x176 (``cabledemo.py`` without ``--no-record``); ~3-6 mm per pixel on the
  table (coarser on the far side). A 6 mm cable is about one pixel wide: use hue/saturation masks, not exact RGB.
  ``--no-record`` debug runs get 64x36 images, which are useless for vision; test the vision path with recording.
- H.264 videos (demo/*.mp4) subsample colour: thin cables lose their colour there. Debug on raw frames
  (np.save ctx.obs.images["top_cam"]), not on the mp4s.
- Occlusion: the top camera sits behind and above the arms, so an arm hides the table beyond it (further from the
  robot) and anything under it. Move the arms clear (low and to the side, out of the line of sight) and wait a few
  steps before looking; re-observe after each move; keep the last good estimate if a new one jumps.
- Heights: back-projection needs the plane height of what you see. Use the object's top face for boxes (side faces
  bias the estimate towards the robot: mask the brightest face, or calibrate a constant offset once offline against
  the sim, then hard-code it) and the cable's centre height for cables lying on the table.
- Calibrate offline only: comparing estimates with ctx.states() in a scratch or CRAWLER_DEBUG-style log is fine;
  the demo's motions must not read ctx.states().
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

_CAMS: dict = {}


@dataclass
class TopCamera:
    """Pinhole model of the station's top camera (OpenGL convention: looks along -z, +y up), at a given image size."""

    fx: float
    fy: float
    cx: float
    cy: float
    rot: np.ndarray  # camera -> world rotation
    pos: np.ndarray  # camera centre, world

    def project(self, xyz) -> tuple[float, float]:
        """World point -> pixel (u right, v down; pixel centres at index + 0.5)."""
        pc = self.rot.T @ (np.asarray(xyz, dtype=float) - self.pos)
        return self.cx + self.fx * pc[0] / -pc[2] - 0.5, self.cy - self.fy * pc[1] / -pc[2] - 0.5

    def rays(self, us, vs) -> np.ndarray:
        """World directions of the rays through pixels (column ``us``, row ``vs``)."""
        us, vs = np.asarray(us, dtype=float), np.asarray(vs, dtype=float)
        d = np.stack([(us + 0.5 - self.cx) / self.fx, -(vs + 0.5 - self.cy) / self.fy, -np.ones(us.shape)], axis=-1)
        return d @ self.rot.T

    def to_plane(self, mask, z: float) -> np.ndarray:
        """(N, 2) world xy where the True pixels of ``mask`` meet the horizontal plane at height ``z``."""
        v, u = np.nonzero(mask)
        if len(u) == 0:
            return np.zeros((0, 2))
        d = self.rays(u, v)
        t = (z - self.pos[2]) / d[:, 2]
        return self.pos[:2] + t[:, None] * d[:, :2]


def top_camera(shape, variant: str = "calibrated") -> TopCamera:
    """The station top camera (``yam_arena.station.top_camera``; ``variant`` as ``-E top_cam=``), with its 640x360
    intrinsics resized to an image of ``shape`` (H, W, ...) as the embodiment resizes the render."""
    key = (int(shape[0]), int(shape[1]), variant)
    if key not in _CAMS:
        from yam_arena import station
        from yam_arena.urdf import quat_xyzw_to_matrix

        cam = station.top_camera(variant)
        lens = cam.lens.resized(key[1], key[0])
        _CAMS[key] = TopCamera(lens.fx, lens.fy, lens.cx, lens.cy,
                               quat_xyzw_to_matrix(np.asarray(cam.rot_xyzw, dtype=float)), np.asarray(cam.pos, float))
    return _CAMS[key]


def top_image(ctx, cam: str = "top_cam") -> np.ndarray:
    """The latest image of ``cam`` as int16 RGB (H, W, 3)."""
    return np.asarray(ctx.obs.images[cam])[..., :3].astype(np.int16)


def color_mask(img, rgb, tol: float = 40.0) -> np.ndarray:
    """Pixels within ``tol`` (Euclidean, 0-255 RGB) of ``rgb`` (0-255 or 0-1 floats). For large, evenly lit parts."""
    rgb = np.asarray(rgb, dtype=float)
    if rgb.max() <= 1.0:
        rgb = rgb * 255.0
    return np.linalg.norm(np.asarray(img, dtype=float)[..., :3] - rgb, axis=-1) <= tol


def hsv_mask(img, hue_deg: float, hue_tol: float = 25.0, min_sat: float = 0.2, min_val: float = 40.0,
             max_val: float = 256.0) -> np.ndarray:
    """Pixels whose hue is within ``hue_tol`` deg of ``hue_deg`` (0 red, 30 orange, 120 green, 220 blue), with
    saturation (max-min)/max >= ``min_sat`` and value max in [min_val, max_val). Thin cables blend with the table,
    so use a low ``min_sat`` for them (~0.15) and a high one (~0.7) for saturated parts on the oak table (hue ~40,
    saturation ~0.3-0.45)."""
    f = np.asarray(img, dtype=float)[..., :3]
    r, g, b = f[..., 0], f[..., 1], f[..., 2]
    mx, mn = f.max(axis=-1), f.min(axis=-1)
    c = mx - mn + 1e-9
    h = np.where(mx == r, ((g - b) / c) % 6, np.where(mx == g, (b - r) / c + 2, (r - g) / c + 4)) * 60.0
    dh = np.abs((h - hue_deg + 180.0) % 360.0 - 180.0)
    sat = (mx - mn) / (mx + 1e-9)
    return (dh <= hue_tol) & (sat >= min_sat) & (mx >= min_val) & (mx < max_val)


def blob_xy(img, mask, z: float, min_px: int = 8, stat: str = "mean"):
    """Centre (x, y) of the masked pixels on the plane at height ``z`` (``stat`` mean or median); None if fewer than
    ``min_px`` pixels are visible."""
    pts = top_camera(img.shape).to_plane(mask, z)
    if len(pts) < min_px:
        return None
    return np.median(pts, axis=0) if stat == "median" else pts.mean(axis=0)


@dataclass
class Polyline:
    """An ordered bead chain along a cable, from its anchor end outwards."""

    points: np.ndarray  # (N, 2), points[0] = the anchor
    start: float = 0.0  # arc length along the cable at points[0] (negative: the anchor lies before the cable start)
    raw: np.ndarray | None = None  # (M, 2) all cable pixels on the plane (for snap())

    def snap(self, xy, radius: float = 0.02, blur: float = 0.008):
        """The cable's centre nearest ``xy``: the mean of the cable pixels within ``blur`` of the nearest one (within
        ``radius``). Beads are ring means, so they cut the inside of tight bends by up to ~1 cm; snap a grasp point
        before closing a gripper on it. Returns ``xy`` unchanged if no cable pixel is that near."""
        if self.raw is None or len(self.raw) == 0:
            return np.asarray(xy, dtype=float)
        d = np.linalg.norm(self.raw - np.asarray(xy)[:2], axis=1)
        k = int(np.argmin(d))
        if d[k] > radius:
            return np.asarray(xy, dtype=float)
        near = self.raw[np.linalg.norm(self.raw - self.raw[k], axis=1) <= blur]
        return near.mean(axis=0)

    @property
    def length(self) -> float:
        """Visible cable length from the cable start (points[0] at arc length ``start``)."""
        return float(np.linalg.norm(np.diff(self.points, axis=0), axis=1).sum()) + self.start

    def at(self, s: float, heading_span: int = 2, snap: bool = True):
        """(xy, heading) of the point ``s`` m along the cable from its start; heading (rad) points back towards the
        anchor, averaged over ``heading_span`` beads; xy snapped onto the cable's pixels (see snap()). None if the
        visible chain is shorter than ``s``."""
        p = self.points
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        cum = np.r_[0.0, np.cumsum(seg)] + self.start
        if len(seg) == 0 or s > cum[-1]:
            return None
        i = int(np.clip(np.searchsorted(cum, s) - 1, 0, len(seg) - 1))
        t = (s - cum[i]) / max(seg[i], 1e-9)
        xy = p[i] + t * (p[i + 1] - p[i])
        j = min(i + 1, len(p) - 1)
        dv = p[max(j - heading_span, 0)] - p[j]
        return (self.snap(xy) if snap else xy), math.atan2(dv[1], dv[0])


def trace_cable(img, mask, z: float, anchor_xy, *, start_ring=(0.035, 0.07), exclude: float = 0.035,
                rings=((0.015, 0.03), (0.03, 0.06), (0.06, 0.10)), cone_deg: float = 60.0, max_beads: int = 80,
                start: float = 0.0):
    """Ordered bead chain of a thin cable seen in ``mask`` (on the plane at height ``z``), walked from ``anchor_xy``
    (a known end: an object it is fixed to, a plug blob, a clamp). The first bead is the mean of the cable points in
    ``start_ring`` around the anchor (pixels nearer than ``exclude`` are ignored: the anchor body hides them); each
    next bead is the mean of the points 1.5-3 cm ahead of the last, within ``cone_deg`` of the chain's direction;
    the wider rings bridge gaps where an arm hides the cable. Stops at the first gap no ring bridges, so check
    ``.length`` before using far points. ``start`` = arc length at the anchor (e.g. -(distance from a body's centre
    to where the cable leaves it)). None if fewer than 3 beads."""
    pts = top_camera(img.shape).to_plane(mask, z)
    a = np.asarray(anchor_xy, dtype=float)[:2]
    pts = pts[np.linalg.norm(pts - a, axis=1) > exclude]
    raw = pts
    beads, cur, d = [a], a, None
    cos = math.cos(math.radians(cone_deg))
    for _ in range(max_beads):
        v = pts - cur
        r = np.linalg.norm(v, axis=1)
        nxt = None
        for lo, hi in ((start_ring,) if d is None else rings):
            m = (r >= lo) & (r < hi)
            if d is not None:
                m &= (v @ d) > cos * r
            if m.sum() >= 2:
                nxt = pts[m].mean(axis=0)
                break
        if nxt is None:
            break
        step = nxt - cur
        d = step / (np.linalg.norm(step) + 1e-12)
        pts = pts[r >= 0.012]
        beads.append(nxt)
        cur = nxt
    return Polyline(np.array(beads), start, raw) if len(beads) >= 3 else None
