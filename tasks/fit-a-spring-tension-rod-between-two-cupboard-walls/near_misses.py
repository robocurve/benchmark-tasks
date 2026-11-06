"""Near-successes and unintended strategies, scored offline by factory/check.py. Rod states come from the part's own
kinematics (ROD.state), so the bounding box and the caps move with the slide and any tilt."""

import math

from task import BAND_Z0, BAND_Z1, CAP, FREE, GAP, ROD, TOP, WALL_CX, WALL_H, WALL_X0, WALL_X1

FIT_Q = FREE - GAP  # 20 mm squeezed when spanning the gap


def qx(deg):
    """Rolled about x: the +y end (tube cap) goes up for positive angles."""
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def qz(deg):
    a = math.radians(deg) / 2
    return (0.0, 0.0, math.sin(a), math.cos(a))


def _face_span(a):
    """Cap face to cap face (m) along a rod turned `a` rad (tilted or skewed) whose cap corners just touch the walls."""
    return (GAP - 2 * (CAP[0] / 2) * math.sin(a)) / math.cos(a)


def _turned(deg, z, x, quat, along, vel=None):
    """A fitted rod turned `deg` so that its cap corners touch both walls. `along(a)` gives the axis direction."""
    a = math.radians(deg)
    q = FREE - _face_span(a) if deg else FREE - GAP
    d = along(a)
    o = [-(q / 2) * c for c in d]  # the axis midpoint sits at (x, 0, z); the body origin is q/2 back along the axis
    return {"rod": ROD.state(joints={"slide": q}, pos=(x + o[0], o[1], TOP + z + o[2]),
                             quat_xyzw=quat(deg) if deg else None, lin_vel=vel)}


def fitted(z, x=WALL_CX, tilt=0.0, vel=None):
    """Spanning the gap, axis midpoint at height z, the +y end raised by `tilt` degrees."""
    return _turned(tilt, z, x, qx, lambda a: (0.0, math.cos(a), math.sin(a)), vel)


def skewed(deg, z=None):
    """Fitted level but turned `deg` about the vertical (the +y cap further back)."""
    return _turned(deg, MID if z is None else z, WALL_CX, qz, lambda a: (-math.sin(a), math.cos(a), 0.0))


def _solve(gap_between_centres):
    """Angle (deg) at which the two cap centres are `gap_between_centres` apart across the axis' turn."""
    lo, hi = 0.0, 0.5
    for _ in range(80):
        a = (lo + hi) / 2
        off = (_face_span(a) - CAP[1]) * math.sin(a)
        lo, hi = (a, hi) if off < gap_between_centres else (lo, a)
    return math.degrees((lo + hi) / 2)


MID = (BAND_Z0 + BAND_Z1) / 2


def short(gap):
    """Level in the band, tube cap on its wall, the bar's cap `gap` m short of the other wall."""
    return {"rod": ROD.state(joints={"slide": FIT_Q + gap}, pos=(WALL_CX, -FIT_Q / 2, TOP + MID))}


def tilt_for(dz):
    return _solve(dz)


def skew_for(dx):
    return _solve(dx)


CASES = [
    ("fitted level in the middle of the band", fitted(MID), 1.0, True),
    ("fitted at the bottom of the band", fitted(BAND_Z0), 1.0, True),
    ("fitted at the top of the band", fitted(BAND_Z1), 1.0, True),
    ("fitted 1 mm below the band", fitted(BAND_Z0 - 0.001), 0.0, False),
    ("fitted 1 mm above the band", fitted(BAND_Z1 + 0.001), 0.0, False),
    ("fitted low, 40 mm up", fitted(0.040), 0.0, False),
    ("fitted, ends 10 mm apart in height (limit)", fitted(MID, tilt=tilt_for(0.010)), 1.0, True),
    ("fitted, ends 11 mm apart", fitted(MID, tilt=tilt_for(0.011)), 0.0, False),
    ("fitted, ends 30 mm apart (limit)", fitted(MID, tilt=tilt_for(0.030)), 0.0, False),
    ("fitted, ends 31 mm apart", fitted(MID, tilt=tilt_for(0.031)), 0.0, False),
    ("fitted level, 30 mm up (limit)", fitted(0.030), 0.0, False),
    ("fitted level, 29 mm up", fitted(0.029), 0.0, False),
    ("fitted, skewed 10 deg about the vertical", skewed(10), 0.0, False),
    ("fitted, caps 9 mm apart front to back", skewed(skew_for(0.009)), 1.0, True),
    ("fitted, caps 11 mm apart front to back", skewed(skew_for(0.011)), 0.0, False),
    ("fitted but still sliding down", fitted(MID, vel=(0.0, 0.0, -0.05)), 0.0, False),
    ("spanning the gap at the walls' front edge, caps half off", fitted(MID, x=WALL_X0), 0.0, False),
    ("spanning the gap, caps flush with the front edge", fitted(MID, x=WALL_X0 + 0.015), 1.0, True),
    ("spanning the gap at the back edge, caps flush", fitted(MID, x=WALL_X1 - 0.015), 1.0, True),
    ("one cap 0.3 mm short of its wall (contact limit)", short(0.0003), 1.0, True),
    ("one cap 0.5 mm short of its wall (not pressing)", short(0.0005), 0.0, False),
    ("one cap 1.5 mm short (a gap only a hand or the other cap's friction could hold)", short(0.0015), 0.0, False),
    ("one cap 3 mm short of its wall (not touching)", short(0.003), 0.0, False),
    ("lying on the table between the walls, unsqueezed",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + 0.0125), quat_xyzw=qz(25))}, 0.0, False),
    ("lying across the wall tops",
     {"rod": ROD.state(pos=(WALL_CX, 0.0, TOP + WALL_H + 0.015))}, 0.0, False),
    ("still on its risers", {}, 0.0, False),
    ("squeezed on the risers", {"rod": ROD.state(joints={"slide": 0.03})}, 0.0, False),
]
