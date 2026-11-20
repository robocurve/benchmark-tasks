"""Compound props built from cuboids: open boxes, lidded boxes, inverted cups, pocket trays, machine housings.

All take the table top ``z0`` they stand on; with ``mass`` the prop is one free rigid body, without it a fixed
collider. Sizes in metres, x forward, y left.
"""

from __future__ import annotations

from rta.spec import TABLE_TOP, Compound, Part

TOP = TABLE_TOP["station"]
GREY = (0.62, 0.62, 0.60)


def open_box(name, xy, size, height, *, wall=0.01, color=GREY, mass=None, z0=TOP) -> Compound:
    """Floor plate plus four walls of total height ``height``; the floor's top is at ``z0 + wall``."""
    sx, sy = size
    dz = height / 2 - wall / 2
    return Compound(
        name, pos=(xy[0], xy[1], z0 + wall / 2), color=color, mass=mass, size=(sx, sy, wall),
        parts=(
            Part((wall, sy, height), (+(sx - wall) / 2, 0.0, dz)),
            Part((wall, sy, height), (-(sx - wall) / 2, 0.0, dz)),
            Part((sx - 2 * wall, wall, height), (0.0, +(sy - wall) / 2, dz)),
            Part((sx - 2 * wall, wall, height), (0.0, -(sy - wall) / 2, dz)),
        ),
    )


def lid(name, xy, size, *, plate=0.01, knob=0.03, z_bottom, color=GREY, mass=0.08) -> Compound:
    """A flat lid with a cube knob on top, resting with its underside at ``z_bottom``."""
    return Compound(name, pos=(xy[0], xy[1], z_bottom + plate / 2), color=color, mass=mass, size=(size[0], size[1], plate),
                    parts=(Part((knob, knob, knob), (0.0, 0.0, plate / 2 + knob / 2)),))


def cup(name, xy, *, size=0.07, height=0.05, wall=0.006, knob=0.025, knob_h=None, color=GREY, mass=0.06, z0=TOP) -> Compound:
    """An upside-down open box (covers what is under it) with a knob on top to lift it by (``knob`` wide,
    ``knob_h`` tall; a cube by default)."""
    h = height - wall
    kh = knob if knob_h is None else knob_h
    return Compound(
        name, pos=(xy[0], xy[1], z0 + height - wall / 2), color=color, mass=mass, size=(size, size, wall),
        parts=(
            Part((wall, size, h), (+(size - wall) / 2, 0.0, -height / 2)),
            Part((wall, size, h), (-(size - wall) / 2, 0.0, -height / 2)),
            Part((size - 2 * wall, wall, h), (0.0, +(size - wall) / 2, -height / 2)),
            Part((size - 2 * wall, wall, h), (0.0, -(size - wall) / 2, -height / 2)),
            Part((knob, knob, kh), (0.0, 0.0, wall / 2 + kh / 2)),
        ),
    )


def cup_rest_z(height=0.05, wall=0.006, z0=TOP) -> float:
    return z0 + height - wall / 2


def pocket_tray(name, xy, pockets, *, pocket=(0.03, 0.03), depth=0.02, wall=0.006, base=0.006, color=GREY, z0=TOP) -> Compound:
    """A fixed base plate with a rim of walls around each pocket; pockets at ``(dx, dy)`` from ``xy``.
    The pocket floor is at ``z0 + base``."""
    px, py = pocket
    xs = [dx for dx, _ in pockets]
    ys = [dy for _, dy in pockets]
    sx = max(xs) - min(xs) + px + 2 * wall
    sy = max(ys) - min(ys) + py + 2 * wall
    cx = (max(xs) + min(xs)) / 2
    cy = (max(ys) + min(ys)) / 2
    parts = []
    for dx, dy in pockets:
        ox, oy, oz = dx - cx, dy - cy, base / 2 + depth / 2
        parts += [
            Part((wall, py + 2 * wall, depth), (ox + (px + wall) / 2, oy, oz)),
            Part((wall, py + 2 * wall, depth), (ox - (px + wall) / 2, oy, oz)),
            Part((px, wall, depth), (ox, oy + (py + wall) / 2, oz)),
            Part((px, wall, depth), (ox, oy - (py + wall) / 2, oz)),
        ]
    return Compound(name, pos=(xy[0] + cx, xy[1] + cy, z0 + base / 2), color=color, size=(sx, sy, base), parts=tuple(parts))


def housing(name, xy, *, size=(0.12, 0.12), height=0.14, wall=0.01, color=(0.36, 0.44, 0.52), z0=TOP) -> Compound:
    """A fixed machine housing: roof, two side walls and the wall facing the arms; open on the far (+x) side."""
    sx, sy = size
    h = height - wall
    return Compound(
        name, pos=(xy[0], xy[1], z0 + height - wall / 2), color=color, size=(sx, sy, wall),
        parts=(
            Part((sx, wall, h), (0.0, +(sy - wall) / 2, -height / 2)),
            Part((sx, wall, h), (0.0, -(sy - wall) / 2, -height / 2)),
            Part((wall, sy - 2 * wall, h), (-(sx - wall) / 2, 0.0, -height / 2)),
        ),
    )


def inside(o, xy, half, *, floor_z, below=0.015, above=0.06) -> bool:
    """An object's centre over a footprint (half sizes) with its bottom near ``floor_z``."""
    return bool(abs(o.pos[0] - xy[0]) < half[0] and abs(o.pos[1] - xy[1]) < half[1] and floor_z - below < o.low[2] < floor_z + above)


def lifted(o, rest_low_z, dz=0.03) -> bool:
    return bool(o.low[2] > rest_low_z + dz)


def tilt_deg(o) -> float:
    """Angle between the object's z axis and the vertical."""
    import math

    x, y, z, w = (float(v) for v in o.quat_xyzw)
    zz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))
