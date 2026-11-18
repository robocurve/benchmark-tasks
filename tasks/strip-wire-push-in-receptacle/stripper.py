"""The wire stripper: a Klein 11055 (Klein-Kurve, 10-18 AWG solid), modelled as two halves on a sprung pivot.

Tool frame: origin at the pivot, +x along the jaws, the tool lying in its xy plane, +z up. Each half is authored as it lies
with the jaws closed, then the moving half ("upper", jaw on +y, handle on -y) is turned about z to where its spring rests,
OPEN_DEG + PRELOAD_DEG, past the open stop. Its joint runs from -(OPEN_DEG + PRELOAD_DEG), shut, to -PRELOAD_DEG, open,
where it starts.

- Overall 181 mm (jaw tip +50 mm, handle ends -131 mm), 154 g (maker's listing: 7-1/8 in, 5.4 oz).
- Stripping holes on the jaws' meeting line, y = 0, for 10, 12, 14, 16 and 18 AWG solid. The real holes are the
  conductor's diameter. In simulation the jaws' edges are U-notched, 4.4 mm across at each hole and 1 mm between,
  so a closing jaw slides a wire into the nearest hole without pinching its jacket; across each hole a 1 mm blade stands
  out to that gauge's conductor radius (plus 0.3 mm): it bites the jacket and holds the cut slug.
"""

from __future__ import annotations

import math

import numpy as np

import mechkit as m

OPEN_DEG = 22.0  # the moving half's swing from shut to fully open
LENGTH, MASS = 0.181, 0.154
JAW_TIP, HANDLE_END = 0.050, -0.131
JAW_X0 = 0.006  # where the jaws start, in front of the pivot
JAW_W, JAW_T = 0.007, 0.005  # each jaw's width (y) beside the meeting line, and thickness (z)
HANDLE_W, HANDLE_T = 0.012, 0.014  # a handle's width in the tool's plane and its thickness
HANDLE_Y = 0.008  # handle centrelines this far either side of the axis with the jaws shut
HANDLE_X0 = -0.020  # where the handles start behind the pivot
HOLE = 0.0044  # every hole's notch, 4.4 mm across: clear round a 2.69 mm insulated 14 AWG wire
GAP = 0.001  # the jaws' collider gap between holes
NOTCH_FLAT, NOTCH_RAMP = 0.0012, 0.0014  # each hole's U-notch: its flat bottom's half-width, then its 50-degree sides
BLADE_T, BLADE_CLEAR, BLADE_GAP = 0.0010, 0.0006, 0.0002  # the blades' thickness, their holes' clearance over the conductor, their gap on the line
# AWG -> (x of the hole's centre from the pivot, the real hole's diameter = the conductor's)
HOLES = {10: (0.014, 0.00259), 12: (0.020, 0.00205), 14: (0.026, 0.00163), 16: (0.032, 0.00129), 18: (0.038, 0.00102)}
PRELOAD_DEG = 40.0  # the spring rests this far past the open stop, so it holds the handles open with a preload
SPRING = 0.8  # N m/rad: 0.56 N m (about 5 N at the handle ends) at the open stop, 0.87 N m (7.5 N) shut
FRAME_INERTIA = (2.0e-5, 2.2e-4, 2.4e-4)  # a 77 g, 18 cm slender half: about its long axis, then across

LOOKS = {
    "steel": m.Look(color=m.rgb(0.62, 0.63, 0.66), roughness=0.35, metallic=0.9),
    "blue": m.Look(color=m.rgb(0.10, 0.25, 0.62), roughness=0.6),
    "yellow": m.Look(color=m.rgb(0.95, 0.75, 0.10), roughness=0.6),
}


def _jaw_meshes(side: float) -> list[m.Mesh]:
    """One jaw's collider meshes with the jaws shut; `side` +1 for the jaw on +y, -1 for -y. Each hole is a U-notch in the
    jaw's edge: flat-bottomed, HOLE/2 from the line for NOTCH_FLAT either side of the hole's centre, then rising to GAP/2
    over NOTCH_RAMP, so a closing jaw guides a wire into the nearest hole without pinching its jacket. Across each hole's
    window a 1 mm blade stands out to the conductor's radius (plus clearance): it bites the jacket and holds the cut slug,
    and the copper slides past it."""
    xs = sorted(x for x, _ in HOLES.values())
    hi, lo, w, f, r = HOLE / 2, GAP / 2, JAW_W, NOTCH_FLAT, NOTCH_RAMP
    outlines = [[(JAW_X0, lo), (xs[0] - f - r, lo), (xs[0] - f, hi), (xs[0] - f, w), (JAW_X0, w)]]
    for a, b in zip(xs, xs[1:]):
        outlines.append([(a + f, hi), (a + f + r, lo), (b - f - r, lo), (b - f, hi), (b - f, w), (a + f, w)])
    outlines.append([(xs[-1] + f, hi), (xs[-1] + f + r, lo), (JAW_TIP, lo), (JAW_TIP, w), (xs[-1] + f, w)])
    meshes = []
    for o in outlines:
        pts = [(x, side * y) for x, y in o]
        if side < 0:
            pts = pts[::-1]  # keep the outline counter-clockwise
        meshes.append(m.prism(pts, -JAW_T / 2, JAW_T / 2))
    for x, d in HOLES.values():
        meshes.append(m.box((2 * f, w - hi, JAW_T), (x, side * (hi + w) / 2, 0.0)))  # the notch's flat bottom
        rb = blade_r(d)
        meshes.append(m.box((2 * (f + r), hi - rb, BLADE_T), (x, side * (rb + hi) / 2, 0.0)))  # the blade
    return meshes


def blade_r(hole_d: float) -> float:
    """Half the side of a blade's square hole in simulation: the real hole's diameter plus BLADE_CLEAR, halved."""
    return (hole_d + BLADE_CLEAR) / 2


def _half(side: float, look: str, prefix: str):
    """One half with the jaws shut: its jaw on `side` of the line, its handle on the other side. Returns (parts, collider meshes)."""
    parts, cols = [], []
    for k, mesh in enumerate(_jaw_meshes(side)):
        cols.append((f"{prefix}_jaw{k}", mesh))
    jaw = m.box((JAW_TIP - JAW_X0, JAW_W, JAW_T), ((JAW_TIP + JAW_X0) / 2, side * JAW_W / 2, 0.0))
    parts.append((f"{prefix}_jaw", jaw, "steel"))
    hub = m.cylinder(0.011, JAW_T, (0, 0, 0))
    parts.append((f"{prefix}_hub", hub, "steel"))
    neck = m.box((abs(HANDLE_X0) + JAW_X0, 0.010, JAW_T), ((HANDLE_X0 + JAW_X0) / 2, -side * 0.004, 0.0))
    parts.append((f"{prefix}_neck", neck, "steel"))
    cols.append((f"{prefix}_neck_c", neck))
    handle = m.box((HANDLE_X0 - HANDLE_END, HANDLE_W, HANDLE_T), ((HANDLE_X0 + HANDLE_END) / 2, -side * HANDLE_Y, 0.0))
    parts.append((f"{prefix}_handle", handle, look))
    cols.append((f"{prefix}_handle_c", handle))
    return parts, cols


TORSION = (0.015, 0.012)  # torsional patch radius and its minimum (m) on the handles, so that a pinch resists the tool turning about it


def _handle_hull(name: str, mesh: m.Mesh) -> str:
    """A handle's collider: its hull, with a torsional patch (mechkit gives one only to free parts, not to mechanisms)."""
    text = m.col_hull(name, mesh)
    extra = (f"\n        float physxCollision:torsionalPatchRadius = {TORSION[0]}"
             f"\n        float physxCollision:minTorsionalPatchRadius = {TORSION[1]}")
    return text.replace('uniform token physics:approximation = "convexHull"', 'uniform token physics:approximation = "convexHull"' + extra)


def _hull(name: str, mesh: m.Mesh) -> str:
    return _handle_hull(name, mesh) if "handle" in name else m.col_hull(name, mesh)


def _turned(items, deg: float):
    return [(n, mesh.turned(deg, "z"), *rest) for n, mesh, *rest in items]


def make(name: str, pos, yaw_deg: float = 0.0) -> m.Part:
    """The stripper lying flat, its pivot at `pos`, jaws open, turned `yaw_deg` from +x."""
    fparts, fcols = _half(-1.0, "blue", "lower")
    uparts, ucols = _half(+1.0, "yellow", "upper")
    rest = OPEN_DEG + PRELOAD_DEG
    uparts, ucols = _turned(uparts, rest), _turned(ucols, rest)
    upper = m.Link("upper", "hinge", "Z", (0.0, 0.0, 0.0), -rest, -PRELOAD_DEG, tuple(uparts), tuple(_hull(n, mesh) for n, mesh in ucols),
                   mass=MASS / 2, spring=SPRING, damping=0.02, friction=0.01, gravity=True, inertia=FRAME_INERTIA)
    return m.mechanism(name, pos, [upper], LOOKS, frame_parts=fparts, frame_colliders=[_hull(n, mesh) for n, mesh in fcols],
                       yaw_deg=yaw_deg, start=[("upper", math.radians(-PRELOAD_DEG))], free_mass=MASS / 2, frame_inertia=FRAME_INERTIA, friction=(1.2, 1.0))


OPEN_J, SHUT_J = -PRELOAD_DEG, -(OPEN_DEG + PRELOAD_DEG)  # the moving half's joint, open and shut (deg)


def handle_point(x_tool: float, which: str, joint_deg: float) -> np.ndarray:
    """A handle's centreline point (tool frame, z 0) where it crosses tool x = `x_tool`, with the moving half's joint at `joint_deg` (OPEN_J open, SHUT_J shut)."""
    if which == "lower":
        return np.array([x_tool, HANDLE_Y, 0.0])
    a = math.radians(OPEN_DEG + PRELOAD_DEG + joint_deg)
    # the upper handle's centreline, shut: (p, -HANDLE_Y); turned by a: x' = p cos a + HANDLE_Y sin a. Solve x' = x_tool for p.
    p = (x_tool - HANDLE_Y * math.sin(a)) / math.cos(a)
    return np.array([x_tool, p * math.sin(a) - HANDLE_Y * math.cos(a), 0.0])


def hole_world(awg: int, pivot: np.ndarray, rot: np.ndarray) -> np.ndarray:
    """Where a hole's centre is in the world, given the lower half's pose."""
    return pivot + rot @ np.array([HOLES[awg][0], 0.0, 0.0])
