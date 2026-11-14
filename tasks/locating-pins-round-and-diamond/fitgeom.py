"""Does the bracket go down over a set of locked pins? Pure geometry, NumPy only, in the fixture's frame.

Each bracket hole is the free region of a 16-sided ring (layout.hole_polygon: flats tangent to the true 8.2 mm
radius circle) turned with the bracket. A pin's head fits in a hole when every point of the head's footprint (a
circle, or the diamond's two lands and relieved body, as layout.pin_shapes draws them) is on the free side of every flat. The
support of a convex footprint in a direction is exact, so no sampling is needed. ``violation`` is the largest
overlap (m) of any head with any flat, negative when everything clears.
"""

import math

import numpy as np

import layout as L

NORMALS, R_IN = L.hole_polygon(L.BR_HOLE_R)


def diamond_vertices() -> np.ndarray:
    """Corners of the diamond head's footprint at its full radius (local x = wide axis): the two lands and the
    relieved body, the boxes layout.pin_shapes draws."""
    ly, by = L.DIAMOND_LAND, L.DIAMOND_FLATS / 2
    lx = math.sqrt(L.HEAD_R ** 2 - ly * ly)
    pts = [(sx * lx, sy * ly) for sx in (-1, 1) for sy in (-1, 1)]
    pts += [(sx * L.DIAMOND_BODY_X, sy * by) for sx in (-1, 1) for sy in (-1, 1)]
    return np.array(pts)


DIAMOND_V = diamond_vertices()


def support(kind: str, yaw: np.ndarray, directions: np.ndarray) -> np.ndarray:
    """Support of a head's footprint (about its axis) along ``directions`` (..., 2), the pin turned ``yaw`` (rad,
    broadcast with the leading dims)."""
    if kind == "round":
        return np.full(directions.shape[:-1], L.HEAD_R)
    c, s = np.cos(yaw), np.sin(yaw)
    # direction in the pin's frame: rotate by -yaw
    dx = directions[..., 0] * c + directions[..., 1] * s
    dy = -directions[..., 0] * s + directions[..., 1] * c
    return np.max(dx[..., None] * DIAMOND_V[:, 0] + dy[..., None] * DIAMOND_V[:, 1], axis=-1)


def violation(dx, dy, theta, pins, hole_y=L.BR_HOLE_Y, normals=NORMALS, r_in=R_IN) -> np.ndarray:
    """Largest overlap (m) of the pins with the bracket's holes. The bracket's centre is (dx, dy) from the fixture's
    centre, turned ``theta`` (rad); ``pins``: a list of (kind, y of the pin's axis, yaw) with y from the fixture's
    centre (the holes: +-0.050) and x 0. Arrays broadcast."""
    dx, dy, theta = np.broadcast_arrays(np.asarray(dx, float), np.asarray(dy, float), np.asarray(theta, float))
    c, s = np.cos(theta), np.sin(theta)
    worst = np.full(dx.shape, -np.inf)
    for kind, py, yaw in pins:
        # the bracket's hole nearest this pin
        sign = 1.0 if py > 0 else -1.0
        hx = dx + c * 0.0 - s * (sign * hole_y)
        hy = dy + s * 0.0 + c * (sign * hole_y)
        # flat normals turned with the bracket
        nx = normals[:, 0] * c[..., None] - normals[:, 1] * s[..., None]
        ny = normals[:, 0] * s[..., None] + normals[:, 1] * c[..., None]
        d = np.stack([nx, ny], axis=-1)  # (..., N, 2)
        sup = support(kind, np.broadcast_to(np.asarray(yaw, float), dx.shape)[..., None] * np.ones(len(normals)), d)
        centre = nx * (0.0 - hx)[..., None] + ny * (py - hy)[..., None]  # n . (pin - hole)
        worst = np.maximum(worst, np.max(centre + sup - r_in, axis=-1))
    return worst


def best_pose(pins, span=0.0008, step=0.00001, turn_deg=0.8, turn_step=0.02):
    """Grid search for the pose with the least violation: (violation, dx, dy, theta_deg)."""
    xs = np.arange(-span, span + 1e-12, step)
    ys = np.arange(-span, span + 1e-12, step)
    ts = np.radians(np.arange(-turn_deg, turn_deg + 1e-9, turn_step))
    best = (np.inf, 0.0, 0.0, 0.0)
    for t in ts:
        v = violation(xs[:, None], ys[None, :], t, pins)
        i = np.unravel_index(np.argmin(v), v.shape)
        if v[i] < best[0]:
            best = (float(v[i]), float(xs[i[0]]), float(ys[i[1]]), math.degrees(t))
    return best


def feasible_window(pins, span=0.0008, step=0.00002, turn_deg=0.4, turn_step=0.02):
    """Extent (mm) of the poses with violation <= 0: dict of dx, dy ranges and turn range (deg), or None."""
    xs = np.arange(-span, span + 1e-12, step)
    ys = np.arange(-span, span + 1e-12, step)
    lo = np.array([np.inf, np.inf, np.inf])
    hi = -lo
    n = 0
    for t in np.arange(-turn_deg, turn_deg + 1e-9, turn_step):
        v = violation(xs[:, None], ys[None, :], math.radians(t), pins)
        ok = v <= 0
        if ok.any():
            n += int(ok.sum())
            ix, iy = np.nonzero(ok)
            lo = np.minimum(lo, [xs[ix].min(), ys[iy].min(), t])
            hi = np.maximum(hi, [xs[ix].max(), ys[iy].max(), t])
    if n == 0:
        return None
    return {"dx_mm": (lo[0] * 1000, hi[0] * 1000), "dy_mm": (lo[1] * 1000, hi[1] * 1000), "turn_deg": (lo[2], hi[2])}


def diamond_limit_deg(round_hole="A", pose=None):
    """The largest turn (deg) of the diamond from square (wide axis along x) at which the bracket, centred on the
    round pin, still clears it. Searches the bracket's pose near the located one."""
    sign = 1.0 if round_hole == "A" else -1.0
    out = 0.0
    for deg in np.arange(0.0, 90.01, 0.5):
        pins = [("round", sign * L.HOLE_SPACING / 2, 0.0),
                ("diamond", -sign * L.HOLE_SPACING / 2, math.radians(deg))]
        v = violation(0.0, -sign * L.SPACING_ERROR / 2, 0.0, pins)
        if v <= 0:
            out = float(deg)
        else:
            break
    return out
