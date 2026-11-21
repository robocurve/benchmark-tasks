"""USD bodies of the hidden-leak inspection (original simplified geometry). Import after Isaac Sim starts.

    housing  kinematic, origin at the station origin on the table top: the housing's side, the wall, the cover strip,
             the far end and and the floor of the gap.
    leak     kinematic, visual only: the green coolant trace, placed on the hidden face at reset (one of five places).
    mirror   the Ullman HTC-2LT lighted inspection mirror, a free tool: grip along its +x, telescoped shaft, round head
             hanging from its rim swivel at 45 degrees, built-in LED. Origin at the grip's centre.
    marker   the paint marker, a free tool standing tip down in a steel pen cup (part of the housing body). Origin at the barrel's centre.
    dot_i    kinematic, visual only: paint dots, moved to where the marker's tip touched the cover strip.
"""

from __future__ import annotations

import math
from pathlib import Path

from . import design as d
from .usd_bodies import Body

GREY, DARK, BLACK, STEEL = (0.55, 0.57, 0.60), (0.30, 0.31, 0.33), (0.04, 0.04, 0.04), (0.62, 0.64, 0.67)
PALE, WHITE, PAINT = (0.80, 0.81, 0.83), (0.95, 0.95, 0.95), (0.95, 0.85, 0.05)


def housing() -> Path:
    b = Body("housing", mass=10.0, kinematic=True, contact_offset=0.002)
    x0, x1 = d.GAP_X
    xc, lx = (x0 + x1) / 2, x1 - x0
    hz = d.HEIGHT
    colours = {"side": GREY, "wall": DARK, "strip": GREY, "far_end": GREY, "floor": DARK}
    for name, (centre, size) in d.housing_boxes().items():  # the hidden face is the side's +y face
        b.box(centre, size, colours[name])
    b.box((xc, d.FACE_Y + 0.0001, hz / 2), (lx - 0.004, 0.0002, hz - 0.010), PALE, collide=False)  # painted face
    # steel pen cup: floor and four walls around an 18 mm bore
    hx, hy = (float(v) for v in d.HOLDER_XY)
    w, h, bore = d.HOLDER[0], d.HOLDER[2], d.HOLDER_BORE
    wall = (w - bore) / 2
    z0 = d.HOLDER_STAND
    b.box((hx, hy, z0 / 2), (w, w, z0), GREY)  # stand
    b.box((hx, hy, z0 + d.HOLDER_FLOOR / 2), (w, w, d.HOLDER_FLOOR), STEEL)
    for s in (-1, 1):
        b.box((hx + s * (bore + wall) / 2, hy, z0 + h / 2), (wall, w, h), STEEL)
        b.box((hx, hy + s * (bore + wall) / 2, z0 + h / 2), (bore, wall, h), STEEL)
    return b.save()


def leak() -> Path:
    """The trace: a thin film just proud of the face, its origin on the face."""
    b = Body("leak", mass=0.01, kinematic=True)
    lw, lh = d.LEAK_SIZE
    b.box((0.0, 0.0006, 0.0), (lw, 0.0008, lh), d.GREEN, collide=False)  # 22 x 50 mm: the stencil's window
    return b.save()


def mirror() -> Path:
    gl, gd = d.GRIP
    sl, sd = d.SHAFT
    hx, dz = d.HEAD_REACH, -d.HEAD_DROP
    # mass: the grip end is heavier; centre of mass about 30 mm ahead of the grip's centre [estimate]
    b = Body("mirror", mass=d.TOOL_MASS, friction=(0.9, 0.8), com=(0.03, 0.0, -0.004),
             inertia=(2.1e-5, 8.5e-4, 8.5e-4), contact_offset=0.001,
             torsional_patch=0.004)  # a pad's contact patch: point contacts alone let the held tool pivot
    b.box((0.0, 0.0, 0.0), (gl, gd, gd), BLACK)  # vinyl grip, modelled square in section so it sits flat (a round one rolls under the closing fingers)
    b.cylinder((gl / 2 + sl / 2, 0.0, 0.0), sd / 2, sl, STEEL, axis="X")  # telescoped shaft
    b.box((hx, 0.0, -sd / 2 - 0.002), (0.008, 0.006, 0.006), STEEL)  # rim swivel
    # the head: a 60.3 mm disc, its plane vertical and turned 45 degrees. The glass (1 mm) faces the robot and the
    # hidden face (-n); behind it, the 7 mm black plastic battery case is matte, so a wrong-facing head shows no reflection
    nx, ny = math.cos(d.HEAD_YAW), math.sin(d.HEAD_YAW)
    glass, back = 0.001, d.HEAD_T - 0.001
    off = d.HEAD_T / 2 - glass / 2
    b.mirror_disc((hx - nx * off, -ny * off, dz), d.HEAD_D / 2, glass, yaw=d.HEAD_YAW)
    off = d.HEAD_T / 2 - back / 2
    b.mirror_disc((hx + nx * off, ny * off, dz), d.HEAD_D / 2 - 0.0005, back, yaw=d.HEAD_YAW, color=DARK)
    # the HTC-2LT's built-in LED: at the head's rim on the far side, lighting the face; hidden from direct view
    b.light((hx + 0.024, -0.014, dz + 0.030), d.LED_INTENSITY)
    return b.save()


def marker() -> Path:
    length, dia = d.MARKER
    tl, td = d.MARKER_TIP
    b = Body("marker", mass=d.MARKER_MASS, friction=(0.9, 0.8), contact_offset=0.001, torsional_patch=0.004,
             com=(0.0, 0.0, 0.0), inertia=(3.5e-5, 3.5e-5, 7e-7))
    b.cylinder((0.0, 0.0, 0.0), dia / 2, length, WHITE)  # barrel (and cap end)
    b.cylinder((0.0, 0.0, -length / 2 - tl / 2), td / 2, tl, PAINT)  # felt tip, tip down
    return b.save()


def dot(i: int):
    def build() -> Path:
        b = Body(f"dot_{i}", mass=0.001, kinematic=True)
        b.cylinder((0.0, 0.0, 0.0002), d.DOT_D / 2, 0.0004, PAINT, collide=False)
        return b.save()
    return build


BUILDERS = {"housing": housing, "leak": leak, "mirror": mirror, "marker": marker,
            **{f"dot_{i}": dot(i) for i in range(d.MARK_DOTS)}}
