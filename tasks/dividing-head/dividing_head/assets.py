"""USD bodies of the dividing head (original simplified geometry). Import after Isaac Sim starts.

Bodies and frames (origins in design.start_poses):
    head    kinematic: housing, index plate (pocket rings as raised lands, the band between the circles, the zero
            line), the workpiece pointer. Origin at (CRANK_XY, table top).
    rotor   the crank: hub and arm, visual only (the gripper works the knob). Origin on the crank axis at the plate face.
    slide   the plunger's radial setting, riding in the arm's slot; visual only. Origin on the pin axis at the face.
    pin     the plunger pin; only its bottom 8 mm collides (with the pockets). Same origin as the slide when seated.
    knob    the yellow knob sleeve, spinning freely on the pin; the part a gripper holds. Origin at its centre.
    work    kinematic: spindle nose and the workpiece disc with its red reference line. Origin on the spindle axis at
            the disc's bottom face.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from . import design as d
from .usd_bodies import Body

HOUSING, PLATE, STEEL, DARK = (0.12, 0.20, 0.42), (0.42, 0.44, 0.47), (0.55, 0.57, 0.60), (0.12, 0.12, 0.13)
BLACK, RED, ORANGE, YELLOW = (0.02, 0.02, 0.02), (0.88, 0.06, 0.06), (0.98, 0.42, 0.02), (1.00, 0.85, 0.05)
GREEN, ALU, WHITE = (0.05, 0.75, 0.20), (0.68, 0.70, 0.74), (0.95, 0.95, 0.95)


def _pol(r: float, a_deg: float) -> tuple[float, float]:
    a = math.radians(a_deg)
    return r * math.cos(a), r * math.sin(a)


def _annulus_segment(b: Body, r0: float, r1: float, a0: float, a1: float, z0: float, z1: float, color,
                     collide=True):
    """A convex piece of an annulus between angles a0 < a1 (deg): its chords lie inside the arcs."""
    am = (a0 + a1) / 2
    pts = [_pol(r0, a0), _pol(r1, a0), _pol(r1, am), _pol(r1, a1), _pol(r0, a1)]
    b.prism_z(pts, z0, z1, color, collide=collide)


def _ring_lands(b: Body, n: int, r: float, z0: float, z1: float):
    """The raised lands between the n pockets of one circle: each pocket is a square POCKET_W gap."""
    h = d.POCKET_W / 2
    r0, r1 = r - h, r + h
    for k in range(n):
        a = math.radians(d.hole_deg(n, k))  # pocket k
        c = math.radians(d.hole_deg(n, k + 1))  # pocket k + 1 (clockwise)
        ua, na = np.array([math.cos(a), math.sin(a)]), np.array([-math.sin(a), math.cos(a)])
        uc, nc = np.array([math.cos(c), math.sin(c)]), np.array([-math.sin(c), math.cos(c)])
        # the land runs from pocket k's clockwise wall to pocket k+1's counter-clockwise wall
        pa = [rr * ua - h * na for rr in (r0, r1)]
        pc = [rr * uc + h * nc for rr in (r0, r1)]
        mid = math.radians((d.hole_deg(n, k) + d.hole_deg(n, k + 1)) / 2)
        um = np.array([math.cos(mid), math.sin(mid)])
        pts = [tuple(pa[0]), tuple(pc[0]), tuple(pc[1]), tuple(r1 * um), tuple(pa[1])]
        b.prism_z(pts, z0, z1, PLATE)


def head() -> Path:
    b = Body("head", mass=5.0, friction=d.FRICTION["head"], kinematic=True)
    x0, x1 = d.HOUSE_X
    y0, y1 = d.HOUSE_Y
    b.box(((x0 + x1) / 2, (y0 + y1) / 2, d.PLATE_Z0 / 2), (x1 - x0, y1 - y0, d.PLATE_Z0), HOUSING)
    base_top = d.PLATE_Z0 + d.PLATE_BASE_T
    b.cylinder((0.0, 0.0, (d.PLATE_Z0 + base_top) / 2), d.PLATE_R, d.PLATE_BASE_T, PLATE)
    z0, z1 = base_top, d.PLATE_TOP
    h = d.POCKET_W / 2
    r20, r21 = d.CIRCLES[d.INNER_N], d.CIRCLES[d.OUTER_N]
    b.cylinder((0.0, 0.0, (z0 + z1) / 2), r20 - h, z1 - z0, PLATE)  # inside the 20-hole circle
    step = 360.0 / d.BAND_SEGMENTS
    for i in range(d.BAND_SEGMENTS):  # between the circles, and outside the 21-hole circle
        a0 = d.ZERO_DEG + i * step
        _annulus_segment(b, r20 + h, r21 - h, a0, a0 + step, z0, z1, PLATE)
        _annulus_segment(b, r21 + h, d.PLATE_R, a0, a0 + step, z0, z1, PLATE)
    for n, r in d.CIRCLES.items():
        _ring_lands(b, n, r, z0, z1)
        for k in range(n):  # dark pocket floors; hole 0 red
            x, y = _pol(r, d.hole_deg(n, k))
            b.cylinder((x, y, base_top + 0.0003), h - 0.0003, 0.0006, RED if k == 0 else BLACK, collide=False)
    # red zero line on the plate face (the turn counter's fixed mark), clear of the pockets
    for ra, rb in ((0.008 + d.HUB_R, r20 - h - 0.002), (r20 + h + 0.001, r21 - h - 0.001), (r21 + h + 0.001,
                                                                                          d.PLATE_R - 0.001)):
        rm = (ra + rb) / 2
        x, y = _pol(rm, d.ZERO_DEG)
        b.box((x, y, z1 + 0.0004), (rb - ra, 0.004, 0.0008), RED, yaw=math.radians(d.ZERO_DEG), collide=False)
    # workpiece pointer: a post and a green needle just outside the disc rim
    sx, sy = d.SPINDLE_XY - d.CRANK_XY
    a = math.radians(d.POINTER_DEG)
    ca, sa = math.cos(a), math.sin(a)
    rp = d.WORK_R + 0.010
    top = d.WORK_Z0 + d.WORK_T
    b.box((sx + rp * ca, sy + rp * sa, (d.PLATE_Z0 + top) / 2), (0.008, 0.008, top - d.PLATE_Z0), DARK, yaw=a)
    tip, base, hw = d.WORK_R + 0.001, d.WORK_R + 0.016, 0.006
    pts = [(sx + tip * ca, sy + tip * sa), (sx + base * ca + hw * sa, sy + base * sa - hw * ca),
           (sx + base * ca - hw * sa, sy + base * sa + hw * ca)]  # counter-clockwise: face up
    b.prism_z(pts, top, top + 0.003, GREEN, collide=False)
    return b.save()


def rotor() -> Path:
    b = Body("rotor", mass=d.MASS["rotor"], com=(0.02, 0.0, 0.01), inertia=d.INERTIA["rotor"])
    a0, a1 = d.ARM_Z[0] - d.PLATE_TOP, d.ARM_Z[1] - d.PLATE_TOP
    b.cylinder((0.0, 0.0, a1 / 2 + 0.001), d.HUB_R, a1 + 0.002, DARK, collide=False)  # hub
    reach = d.CIRCLES[d.OUTER_N] + 0.012
    b.box(((reach - 0.010) / 2, 0.0, (a0 + a1) / 2), (reach + 0.010, 0.018, a1 - a0), ORANGE, collide=False)
    r20, r21 = d.CIRCLES[d.INNER_N], d.CIRCLES[d.OUTER_N]
    b.box(((r20 + r21) / 2, 0.0, a1 + 0.0003), (r21 - r20 + 0.008, 0.007, 0.0006), DARK, collide=False)  # slot
    b.box((0.006, 0.0, a1 + 0.0004), (0.010, 0.004, 0.0008), WHITE, collide=False)  # arm stripe on the hub
    return b.save()


def slide() -> Path:
    b = Body("slide", mass=d.MASS["slide"], com=(0.0, 0.0, 0.014), inertia=d.INERTIA["slide"])
    a1 = d.ARM_Z[1] - d.PLATE_TOP
    b.box((0.0, 0.0, a1 + 0.002), (0.014, 0.020, 0.004), STEEL, collide=False)
    return b.save()


def pin() -> Path:
    b = Body("pin", mass=d.MASS["pin"], friction=d.FRICTION["pin"], com=(0.0, 0.0, 0.006), inertia=d.INERTIA["pin"])
    b.cylinder((0.0, 0.0, 0.0), d.PIN_R, 2 * d.PIN_DEPTH, STEEL)  # the part that enters a pocket
    top = d.KNOB_Z0 - d.PLATE_TOP
    b.cylinder((0.0, 0.0, (d.PIN_DEPTH + top) / 2), 0.0025, top - d.PIN_DEPTH, STEEL, collide=False)
    return b.save()


def knob() -> Path:
    b = Body("knob", mass=d.MASS["knob"], friction=d.FRICTION["knob"], com=(0.0, 0.0, 0.0), inertia=d.INERTIA["knob"])
    b.cylinder((0.0, 0.0, 0.0), d.KNOB_R, d.KNOB_H, YELLOW)
    return b.save()


def work() -> Path:
    b = Body("work", mass=1.0, friction=d.FRICTION["work"], kinematic=True)
    b.cylinder((0.0, 0.0, (d.NOSE_Z[0] - d.WORK_Z0) / 2), d.NOSE_R, d.WORK_Z0 - d.NOSE_Z[0], DARK)
    b.cylinder((0.0, 0.0, d.WORK_T / 2), d.WORK_R, d.WORK_T, ALU)
    length = d.WORK_R - 0.006
    b.box((0.005 + length / 2, 0.0, d.WORK_T + 0.0004), (length, 0.004, 0.0008), RED, collide=False)
    b.cylinder((0.0, 0.0, d.WORK_T + 0.0005), 0.006, 0.001, DARK, collide=False)
    return b.save()


BUILDERS = {"head": head, "rotor": rotor, "slide": slide, "pin": pin, "knob": knob, "work": work}
