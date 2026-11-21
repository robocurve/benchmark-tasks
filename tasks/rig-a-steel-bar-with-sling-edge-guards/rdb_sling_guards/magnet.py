"""The guards' magnetic pull on the bar, evaluated at every 240 Hz physics substep (the window cleaner package's way:
rdb_window_wall/magnet.py). Each guard carries four NdFeB discs, one pair in each flange; each disc pulls straight
toward the nearest point of the bar's surface with F0 / (1 + g / g0)^2 at a gap g, zero beyond MAG_RANGE. Nothing
attaches a guard to the bar: the result is a force and a torque on each guard, and contact with the bar does the rest.
"""

import numpy as np

from . import layout as L

BAR_LO = np.array((L.BAR_X[0], -L.HALF, L.Z_BOT))
BAR_HI = np.array((L.BAR_X[1], L.HALF, L.Z_TOP))


def guard_magnets_local():
    """The four discs' faces in a guard's own frame (origin at the inside corner's middle, x along the edge, the two
    flanges along -y (the 'face' flange, normal +z) and -z (the 'side' flange, normal +y) as the guard sits on a
    top-left edge)."""
    out = []
    for dx, out_from_corner in L.MAG_POS:
        out.append(np.array((dx, -out_from_corner, L.MAG_RECESS)))       # face flange: inside face at z = 0
        out.append(np.array((dx, L.MAG_RECESS, -out_from_corner)))       # side flange: inside face at y = 0
    return out


MAGNETS = guard_magnets_local()


def pull(points, f0=None):
    """Pull (N) on each disc toward the bar's surface, and each disc's gap (m)."""
    f0 = L.MAGNET_PULL_N if f0 is None else f0
    p = np.asarray(points, dtype=float)
    q = np.clip(p, BAR_LO, BAR_HI)
    d = p - q
    gap = np.linalg.norm(d, axis=1)
    direction = -d / np.maximum(gap, 1e-9)[:, None]
    inside = gap < 1e-7
    if inside.any():                       # on or in the surface: along the nearest face's inward normal
        lo, hi = p - BAR_LO, BAR_HI - p
        for i in np.nonzero(inside)[0]:
            k = int(np.argmin(np.concatenate((lo[i], hi[i]))))
            n = np.zeros(3)
            n[k % 3] = 1. if k < 3 else -1.
            direction[i] = n
    size = np.where(gap < L.MAG_RANGE, f0 / (1. + gap / L.MAG_G0) ** 2, 0.)
    return direction * size[:, None], gap


def on_guard(position, rotation, com_local, f0=None):
    """Total force and torque about the centre of mass (world frame) on one guard at ``position`` with rotation
    matrix ``rotation``; and its discs' largest gap."""
    pts = np.array([position + rotation @ m for m in MAGNETS])
    force, gap = pull(pts, f0)
    com = position + rotation @ com_local
    return force.sum(axis=0), np.cross(pts - com, force).sum(axis=0), float(gap.max())
