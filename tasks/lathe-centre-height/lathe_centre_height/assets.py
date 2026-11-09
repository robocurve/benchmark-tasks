"""USD bodies of the lathe set-up (original simplified geometry). Import after Isaac Sim starts.

Bodies and frames (origins in design.start_poses):
    lathe     kinematic, origin at the station origin on the table top: bed, headstock, chuck and the 8 mm bar,
              the cross-feed crank's pedestal, and the slotted block the rule stands in.
    carriage  the cross slide (visual only), origin at the cutting edge's start position.
    tool      tool bit and holder; only the cutting edge strip and the bit body collide. Origin at the cutting edge's
              front, at its height.
    xcrank / zcrank   crank hub and arm (visual only), origin on the crank axis at the hub top.
    xknob / zknob     the crank knobs a gripper holds (spinning freely on the arms), origin at the knob centre.
    rule      the steel rule, a free plate, origin at its centre.
"""

from __future__ import annotations

from pathlib import Path

from . import design as d
from .usd_bodies import Body

CAST, STEEL, DARK, BLACK = (0.30, 0.34, 0.36), (0.62, 0.64, 0.67), (0.14, 0.14, 0.15), (0.03, 0.03, 0.03)
ORANGE, YELLOW, BLUE, RED = (0.98, 0.42, 0.02), (1.00, 0.85, 0.05), (0.10, 0.30, 0.80), (0.88, 0.06, 0.06)
RULE_SILVER = (0.80, 0.82, 0.85)


def _span(a):
    return (a[0] + a[1]) / 2, a[1] - a[0]


def lathe() -> Path:
    b = Body("lathe", mass=20.0, friction=d.FRICTION["lathe"], kinematic=True, contact_offset=0.0005)
    (bx, bw), (by, bl), (bz, bh) = _span(d.BED_X), _span(d.BED_Y), _span(d.BED_Z)
    b.box((bx, by, bz), (bw, bl, bh), CAST)
    (hx, hw), (hy, hl), (hz, hh) = _span(d.HEAD_X), _span(d.HEAD_Y), _span(d.HEAD_Z)
    b.box((hx, hy, hz), (hw, hl, hh), CAST)
    cy, cl = _span(d.CHUCK_Y)
    b.cylinder((d.BAR_X, cy, d.HC), d.CHUCK_R, cl, DARK, axis="Y")
    by0, bl0 = _span(d.BAR_Y)
    b.cylinder((d.BAR_X, by0, d.HC), d.BAR_R, bl0, STEEL, axis="Y")
    # cross-feed crank pedestal (visual: the crank turns above it)
    b.cylinder((d.CROSS_AXIS[0], d.CROSS_AXIS[1], d.CROSS_HUB_Z / 2), 0.012, d.CROSS_HUB_Z, CAST, collide=False)
    # cross slide way under the carriage (visual)
    b.box((d.TIP_X0 - 0.045, d.CONTACT_Y, (d.BED_Z[1] + d.HC - 0.035) / 2), (0.080, 0.050, d.HC - 0.035 - d.BED_Z[1]),
          CAST, collide=False)
    return b.save()  # the rule's slotted stand is in the task file (plain fixed boxes)


def carriage() -> Path:
    b = Body("carriage", mass=d.MASS["carriage"], com=(-0.05, 0.0, -0.04), inertia=d.INERTIA["carriage"])
    b.box((-0.050, 0.0, -d.HOLDER[2] - 0.004), (0.060, 0.045, 0.006), DARK, collide=False)
    return b.save()


def tool() -> Path:
    b = Body("tool", mass=d.MASS["tool"], friction=d.FRICTION["tool"], com=(-0.03, 0.0, -0.01),
             inertia=d.INERTIA["tool"], contact_offset=0.0003, solver_iters=(32, 8))
    # cutting edge: a thin strip centred on the tool's origin (its height is the edge height the scorer reads; a strip
    # hanging below the origin put the rule's vertical point 0.25 mm high in the 2026-11-08 run)
    b.box((-0.0005, 0.0, 0.0), (0.001, d.BIT[1], d.EDGE_T), STEEL)
    # bit body below and behind the edge (front relief), then the holder (visual)
    lx, w, h = d.BIT
    b.box((-d.RELIEF - (lx - d.RELIEF) / 2, 0.0, -d.EDGE_T - (h - d.EDGE_T) / 2), (lx - d.RELIEF, w, h - d.EDGE_T),
          STEEL)
    hx, hy, hz = d.HOLDER
    b.box((-lx - hx / 2 + 0.006, 0.0, d.HOLDER_TOP_DZ - hz / 2), (hx, hy, hz), BLACK, collide=False)
    return b.save()


def _crank(name: str) -> Path:
    b = Body(name, mass=d.MASS[name], com=(d.CRANK_ARM_R / 2, 0.0, d.ARM_T / 2), inertia=d.INERTIA[name])
    b.cylinder((0.0, 0.0, d.ARM_T / 2), 0.009, d.ARM_T, DARK, collide=False)
    b.box((d.CRANK_ARM_R / 2, 0.0, d.ARM_T / 2), (d.CRANK_ARM_R + 0.012, 0.012, d.ARM_T), ORANGE, collide=False)
    return b.save()


def _knob(name: str) -> Path:
    b = Body(name, mass=d.MASS[name], friction=d.FRICTION["knob"], com=(0.0, 0.0, 0.0), inertia=d.INERTIA[name])
    b.cylinder((0.0, 0.0, 0.0), d.KNOB_R, d.KNOB_H, YELLOW)
    return b.save()


def rule() -> Path:
    t, w, length = d.RULE
    b = Body("rule", mass=d.RULE_MASS, friction=d.FRICTION["rule"], com=(0.0, 0.0, 0.0),
             inertia=(d.RULE_MASS * (w * w + length * length) / 12, d.RULE_MASS * (t * t + length * length) / 12,
                      d.RULE_MASS * (t * t + w * w) / 12), contact_offset=0.001, solver_iters=(32, 8),
             torsional_patch=0.004)  # a fingertip pad's contact patch: point contacts alone let the rule spin
    b.box((0.0, 0.0, 0.0), (t, w, length), RULE_SILVER)
    # black graduation marks on both faces, every 10 mm (visual), and a red top end for the cameras
    for k in range(1, 15):
        z = -length / 2 + k * 0.010
        for s in (-1, 1):
            b.box((s * (t / 2 + 0.00005), -w / 2 + 0.002, z), (0.0001, 0.004, 0.0006), BLACK, collide=False)
    for s in (-1, 1):
        b.box((s * (t / 2 + 0.00005), 0.0, length / 2 - 0.004), (0.0001, w, 0.008), RED, collide=False)
    return b.save()


BUILDERS = {"lathe": lathe, "carriage": carriage, "tool": tool, "xcrank": lambda: _crank("xcrank"),
            "zcrank": lambda: _crank("zcrank"), "xknob": lambda: _knob("xknob"), "zknob": lambda: _knob("zknob"),
            "rule": rule}
