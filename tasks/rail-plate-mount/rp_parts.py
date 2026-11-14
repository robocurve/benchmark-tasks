"""The rail-plate-mount task's own parts: a steel mounting plate with two tapped M5 holes, a loose slotted TS35 DIN
rail, two loose M5 hexagon socket head cap screws and a screwdriver with a 4 mm hexagon blade. All geometry is our own
simplified design, drawn from the TS35 rail standard (EN 60715), the screw's head standard (ISO 4762) and the key sizes
of ISO 2936; no manufacturer CAD is used.

yam_arena task files support single primitives and library assets only, so this module is the task's own minimal
mechanism (the same approach as end-bracket-mount, copied so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` that also names which part it is.
- :func:`install` wraps ``yam_arena.scene.task_object`` and ``object_states`` the moment that module is imported.

The parts (dimensions in mm in comments, metres in code):

- plate (fixed): galvanised steel mounting plate, 200 across x 300 along, 2 mm sheet on folded 15 mm edges (as
  mounting plates lie on the bench), two tapped M5 holes 200 mm apart on its centreline.
- rail (free): TS35 x 7.5, 1 mm sheet (web and lips 2 mm in the simulation, against tunnelling), 250 long, lying web-down; slots 6.2 x 15 at
  25 mm pitch along the web's centreline.
- screws (free): M5 x 6 hexagon socket head cap screws (head 8.5 diameter, 5.0 high; socket 4 across flats, 2.5
  deep), zinc-plated steel.
- driver (free): a stubby screwdriver with a 4 mm hexagon blade.

The thread. A real thread (0.8 mm pitch) is far too fine for PhysX, and a loose screw can't carry a permanent thread
joint, so each screw exists twice:
- the loose screw (kind "screw"), an ordinary rigid body. Its 0.6 mm dog point (a reduced tip, r 1.9) drops into a
  hole (4.4 mm square, the tapped hole's core), its shank (r 2.45) sits on the hole's edge: it stands in the hole,
  centred, like a real screw before it catches.
- its threaded twin (kind "thread"): an articulation fixed to the world, parked out of sight. Bodies: anchor -> thread
  slider (coupled to the screw's turning by a PhysX mimic joint: 0.8 mm per turn) -> clamp slider (a stiff spring: the
  clamp under the head) -> the screw (turning, with constant thread friction). Its shank is thinner (r 2.0) so it
  passes through the hole.
When the loose screw stands in a hole (dog point within ENGAGE_R of its axis, seated, upright within ENGAGE_TILT) and
is turned clockwise by START_ANGLE, :func:`update_threads` swaps it for its twin at the same place and angle: it has
caught the thread. Turned back out BACKOUT_TURNS past where it caught, it is swapped back. The swap runs once per
control step from the patched ``object_states`` (which yam_arena calls every step to score).

Frames: plate: origin at the centre of its top surface. Rail: centre of its web's underside (x across, y along, z
up). Screw: the centre of its tip, its axis +z, two socket flats facing +-y. Thread twin: the anchor sits where the tip
was when the screw caught, turned to the screw's angle then. Driver: its blade tip, the shaft along +z, two of the
hexagon's flats facing +-y.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import inspect
import math
import sys
import tempfile
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from yam_arena import task_file

MM = 1e-3

# -- mounting plate -------------------------------------------------------------------------------------------
PLATE_ACROSS = 200 * MM  # x
PLATE_ALONG = 300 * MM  # y
PLATE_SHEET = 2.0 * MM
PLATE_FLANGE = 15.0 * MM  # the plate's top above the table (its folded edges)
HOLE_Y = (-100 * MM, 100 * MM)  # the two tapped M5 holes on the centreline
HOLE_HALF = 2.2 * MM  # the hole (sim: the tapped hole's core, 4.13 mm for M5) as an octagon 4.4 mm across flats ...
HOLE_SINK_HALF = 2.75 * MM  # ... with a 90 degree countersink at its top, 5.5 mm across (as tapped holes are made): it
# guides a tip up to ~0.85 mm off centre into the hole (without it, 0.3 mm; demo runs 11-12)
HOLE_CUT = 4.0 * MM  # the plate's square cut-out round each hole, filled with the eight countersunk wall pieces

# -- rail (TS35 x 7.5, EN 60715; slot pattern our choice) ----------------------------------------------------
RAIL_LENGTH = 250 * MM
RAIL_HEIGHT = 7.5 * MM
SHEET = 1.0 * MM  # the side walls
WEB = 2.0 * MM  # simulation: a real web is 1 mm (thickened: under a tightening screw head the 1 mm web let the head
# sink into it, bench run 4)
LIP_THICK = 2.0 * MM  # simulation: real lips are 1 mm (thickened so contacts don't tunnel at 240 Hz)
LIP_EDGE = 17.5 * MM
LIP_INNER = 13.5 * MM
SLOT_HALF_W = 3.1 * MM  # slots 6.2 wide ...
SLOT_LEN = 15.0 * MM  # ... 15 long ...
SLOT_PITCH = 25.0 * MM  # ... at 25 mm pitch, the first 12.5 mm from the rail's end (centre)
SLOTS = tuple(-RAIL_LENGTH / 2 + 12.5 * MM + k * SLOT_PITCH for k in range(10))
RAIL_MASS = 0.095  # kg: 25 cm of 1 mm steel TS35, less the slots

# -- screw (M5 x 6 hexagon socket head cap screw, head per ISO 4762) ---------------------------------------------------------------
SCREW_LEN = 6.0 * MM  # under the head to the tip
DOG_LEN, DOG_R = 0.6 * MM, 1.9 * MM  # the tip's reduced start (sim: what centres it in the hole)
SHANK_R = 2.45 * MM  # loose screw: the thread's outside (sits on the hole's edge)
THREAD_SHANK_R = 2.0 * MM  # threaded twin: passes through the hole
SEAT_DEPTH = DOG_LEN + (HOLE_SINK_HALF - SHANK_R)  # a loose screw standing in a hole: its tip this far below the
# plate's top (its shank on the countersink, 0.3 mm down)
HEAD_R, HEAD_H = 4.25 * MM, 5.0 * MM  # socket head (ISO 4762 M5: 8.5 diameter, 5 high) ...
SOCKET_AF, SOCKET_DEPTH = 4.0 * MM, 2.5 * MM  # ... with a hexagon socket 4 across flats, 2.5 deep (the standard's minimum)
SOCKET_CHAMFER = 0.25 * MM  # its 45 degree entry chamfer (a key within ~0.3 mm finds its way in; at 0.5 mm, a key
# turning the seated screw rode up the chamfer and skipped over the flats, demo run 38)
HEAD_BEARING = True  # simulation: the head bears on BEARING_N small balls on a ring (see screw_shapes)
BEARING_N, BEARING_RING, BEARING_R = 12, 3.7 * MM, 0.4 * MM
PITCH = 0.8 * MM  # M5 coarse thread
SCREW_MASS = 0.0027  # kg (steel, 345 mm3)
SCREW_INERTIA = 5.0e-7  # kg m^2 about each axis, the loose screw (simulation: its real ~3e-8 spins it up to
# thousands of rad/s in one 240 Hz step under any contact torque; the mass stays real)

# -- screw holder (our design: a steel block the two screws stand in head up, as a fitter's screw holder, and the
# screwdriver blade down) -----------------------------------------------------------------------------------------
HOLDER_SIZE = (90 * MM, 24 * MM, 30 * MM)  # x, y, z
HOLDER_HOLES = (-32 * MM, -12 * MM)  # the screws' holes along x, on its centreline (20 mm apart: fingers closing
# along the block clear the other screw's head) ...
HOLDER_HOLE_HALF = 2.8 * MM  # ... 5.6 mm square: the shank (r 2.45) drops in, the head (r 4.75) rests on a collar
HOLDER_BOSS_HALF, HOLDER_BOSS_H = 3.5 * MM, 3.0 * MM  # a 7 x 7 x 3 mm collar round each screw hole: the head rests on
# it, 3 mm above the block, so fingers can take the head over its whole height (on a flat block they caught only its
# top edge and it tipped over, pick bench run 5)
HOLDER_DRIVER = 28 * MM  # the screwdriver's hole (x), 6.8 mm square: its shaft (r 3) in it, its collar on the block
# (40 mm from the nearer screw: its handle, r 13, clears the fingers taking that screw, which widen to 19 mm above
# the pads; at 29 mm they landed on the handle, demo run 2)
HOLDER_DRIVER_HALF = 3.4 * MM
HOLDER_MASS = 0.45  # kg (steel)

# -- thread twin ----------------------------------------------------------------------------------------------
CLAMP_SPRING = 3.0e5  # N/m: the clamp under the head (the joint's elasticity)
CLAMP_TRAVEL = 0.5 * MM  # hard stop (150 N): the screw can't be turned further
CONTACT_CLAMP = 2.0  # N: the head counts as touching (the clamp spring takes up this much) ...
CONTACT_STEPS = 10  # ... for this many control steps in a row (0.5 s)
TIGHT_PAST = 0.25  # "tight" in the simulation: turned this far past the head's first contact ...
TIGHT_CLAMP = 5.0  # N ... with the head still pressing at least this hard. (The spring's force alone isn't a usable
# measure: it relaxes after tightening as the head settles into the contact, and reaches only 12-15 N on the loose rail
# against 57 N on the fixed plate, bench run 10; the turns are exact.) The
# stand-in for a real screw tightened to its torque; a real trial checks with a torque screwdriver.
FREE_TORQUE = 0.03  # Nm: thread friction (constant)
THREAD_RANGE = 12.0 * MM  # the thread slider's limits either way (more than the screw's whole length)
SCREW_ARMATURE = 2.0e-4  # kg m^2: about a screwdriver's and a hand's share (as in end-bracket-mount; the bare
# screw's ~3e-8 is too light for 240 Hz steps against the clamp spring through the thread)
LINK_ARMATURE = 0.001
LINK_MASS = 0.0002
ANCHOR_MASS = 0.001

# -- catching the thread ------------------------------------------------------------------------------------
ENGAGE_R = 1.2 * MM  # the tip within this of the hole's axis (inside the countersink, which ends 2.75 mm out; held by
# the soft-wristed hand and turned, the tip wanders up to ~1.4 mm, demo run 15) ...
ENGAGE_R_HYST = 0.4 * MM  # ... (plus this once it is being turned) ...
ENGAGE_DZ = 0.3 * MM  # ... its tip in the hole's countersink: from ENGAGE_DZ below its resting depth up to ...
ENGAGE_MIN_DEPTH = 0.2 * MM  # ... this below the plate's top (a real thread centres itself in the countersink as it
# catches; held by the hand, a screw stands up to 0.5 mm off centre and 0.5 mm high there, demo run 13) ...
ENGAGE_TILT = math.radians(3.0)  # ... upright within this ...
START_ANGLE = math.radians(20.0)  # ... and turned clockwise this far while so
BACKOUT_TURNS = 0.25  # turned out this far past where it caught: loose again
PARK = (-1.0, 0.0, 0.05)  # parked bodies (x, z; y per screw): behind the station, out of the cameras' view

# -- screwdriver: stubby, with a 4 mm hexagon blade (ISO 2936 key size) for the M5 socket ---------------------------
KEY_AF = 3.9 * MM  # simulation: 0.05 mm play each side in the 4.0 socket (real: ~0.02) so PhysX contacts don't jam
# (3.8 turned 5.7 degrees loose in the socket and skipped under load, run 38)
KEY_TIP_CHAMFER = 0.3 * MM
BLADE_LENGTH = 10.0 * MM  # the hexagon part, then the round shaft
SHAFT_R = 3.0 * MM
SHAFT_TOP = 25.0 * MM
COLLAR_R, COLLAR_TOP = 8.0 * MM, 31.0 * MM
HANDLE_R = 13.0 * MM
HANDLE_TOP = 85.0 * MM
DRIVER_MASS = 0.035

# -- materials and looks ------------------------------------------------------------------------------------
MATERIALS = {  # (static friction, dynamic friction, combine mode)
    "steel": (0.30, 0.30, "average"),
    "screw": (0.20, 0.20, "average"),  # blade in the slot, head on the rail
    "handle": (0.90, 0.80, "average"),
}
LOOKS = {  # sRGB, roughness, metallic
    "steel": ((0.72, 0.74, 0.76), 0.35, 0.8),
    "plate": ((0.78, 0.79, 0.80), 0.45, 0.7),
    "zinc": ((0.80, 0.80, 0.78), 0.3, 0.9),
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
    "handle": ((0.85, 0.12, 0.10), 0.6, 0.0),
    "black": ((0.08, 0.08, 0.09), 0.7, 0.0),
}
CONTACT_OFFSET = 0.5 * MM


# -- shapes -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One piece of a part, in its body's frame: a box (``center``, ``size``), a prism (a 2-D ``profile``
    extruded over ``span``: (x, z) along y for "xz", (x, y) along z for "xy", (y, z) along x for "yz"; convex unless
    visual-only), or a visual cylinder (``center``, ``radius``, ``length`` along ``axis``)."""

    body: str
    look: str
    material: str | None  # None: visual only
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float] | None = None
    profile: tuple[tuple[float, float], ...] | None = None
    span: tuple[float, float] = (0.0, 0.0)
    plane: str = "xz"
    radius: float = 0.0
    length: float = 0.0
    axis: str = "Z"
    visible: bool = True
    hull: tuple | None = None  # a collider: the convex hull of these (x, y, z) points

    def lift(self, p, s):
        if self.plane == "xz":
            return (p[0], s, p[1])
        if self.plane == "yz":
            return (s, p[0], p[1])
        return (p[0], p[1], s)

    def points(self):
        if self.hull is not None:
            return list(self.hull)
        if self.profile is not None:
            return [self.lift(p, s) for p in self.profile for s in self.span]
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h), "S": (r, r, r)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _prism(body, look, material, profile, s0, s1, plane="xz", visible=True) -> Shape:
    return Shape(body, look, material, profile=tuple((float(a), float(b)) for a, b in profile), span=(s0, s1),
                 plane=plane, visible=visible)


def _ngon(r, n, start=0.0, a0=0.0, a1=2 * math.pi):
    """Points on a circle (counter-clockwise) from a0 to a1; the full circle without repeating the start."""
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    k = n if full else n + 1
    return [(r * math.cos(a0 + start + (a1 - a0) * i / n), r * math.sin(a0 + start + (a1 - a0) * i / n))
            for i in range(k)]


def plate_shapes() -> list[Shape]:
    """The sheet with its two square holes, and the folded edges it stands on (all collide)."""
    ax, al, h, t = PLATE_ACROSS / 2, PLATE_ALONG / 2, HOLE_HALF, PLATE_SHEET
    cut = HOLE_CUT
    out = [_box("plate", "plate", "steel", -ax, -cut, -al, al, -t, 0.0),
           _box("plate", "plate", "steel", cut, ax, -al, al, -t, 0.0)]
    ys = [-al] + [v for y in HOLE_Y for v in (y - cut, y + cut)] + [al]
    for y0, y1 in zip(ys[::2], ys[1::2]):
        out.append(_box("plate", "plate", "steel", -cut, cut, y0, y1, -t, 0.0))
    for y in HOLE_Y:  # each hole: eight convex wall pieces, octagonal, countersunk at the top
        out += hole_pieces(0.0, y, t)
    f = PLATE_FLANGE
    for s in (1, -1):  # the folded edges, 2 mm thick, down to the table
        out.append(_box("plate", "plate", "steel", *sorted((s * ax, s * (ax - t))), -al, al, -f, -t))
        out.append(_box("plate", "plate", "steel", -ax + t, ax - t, *sorted((s * al, s * (al - t))), -f, -t))
    return out


def hole_pieces(cx: float, cy: float, t: float) -> list[Shape]:
    """The wall round a hole in a sheet (top at z 0, ``t`` thick) inside a square cut-out of half-width HOLE_CUT: eight
    convex pieces, one per side of an octagon (apothem HOLE_HALF), countersunk 45 degrees to HOLE_SINK_HALF at the
    top."""
    h, hs, cut = HOLE_HALF, HOLE_SINK_HALF, HOLE_CUT
    sink = hs - h  # the countersink's depth (45 degrees)
    r_v = h / math.cos(math.pi / 8)  # the octagon's vertices
    out = []
    for k in range(8):
        a0, a1 = (k - 0.5) * math.pi / 4, (k + 0.5) * math.pi / 4
        inner = [(r_v * math.cos(a), r_v * math.sin(a)) for a in (a0, a1)]
        ring = []  # the cut-out's square boundary between the two rays, with its corner if one lies between
        for a in (a0, a1):
            c, s_ = math.cos(a), math.sin(a)
            m = cut / max(abs(c), abs(s_))
            ring.append((m * c, m * s_))
        for sx in (1, -1):
            for sy in (1, -1):
                ac = math.atan2(sy, sx)
                if any(a0 < ac + n * 2 * math.pi < a1 for n in (-1, 0, 1)):
                    ring.append((sx * cut, sy * cut))
        pts = []
        for x, y in ring:
            pts += [(x, y, -t), (x, y, 0.0)]
        for x, y in inner:
            pts += [(x, y, -t), (x, y, -sink), (x * hs / h, y * hs / h, 0.0)]
        out.append(Shape("plate", "plate", "steel", hull=tuple((cx + x, cy + y, z) for x, y, z in pts), visible=False))
    # the looks: the plate's top over the cut-out (but the hole), a dark hole, and the bright countersink
    for x0, x1, y0, y1 in ((-cut, cut, -cut, -hs), (-cut, cut, hs, cut), (-cut, -hs, -hs, hs), (hs, cut, -hs, hs)):
        out.append(_box("plate", "plate", None, x0 + cx, x1 + cx, y0 + cy, y1 + cy, -t, 0.0))
    out.append(Shape("plate", "hole", None, (cx, cy, -t / 2 - 0.05 * MM), radius=h * 0.98, length=t))
    out.append(Shape("plate", "zinc", None, (cx, cy, -sink / 2), radius=(h + hs) / 2, length=sink))
    return out


def rail_shapes() -> list[Shape]:
    L = RAIL_LENGTH / 2
    out = []
    for s in (1, -1):  # the web either side of the slots
        out.append(_box("rail", "steel", "steel", *sorted((s * SLOT_HALF_W, s * LIP_INNER)), -L, L, 0.0, WEB))
    edges = [-L] + [v for y in SLOTS for v in (y - SLOT_LEN / 2, y + SLOT_LEN / 2)] + [L]
    for y0, y1 in zip(edges[::2], edges[1::2]):  # the web between the slots
        out.append(_box("rail", "steel", "steel", -SLOT_HALF_W, SLOT_HALF_W, y0, y1, 0.0, WEB))
    for s in (1, -1):
        out.append(_box("rail", "steel", "steel", *sorted((s * (LIP_INNER - SHEET), s * LIP_INNER)), -L, L,
                        WEB, RAIL_HEIGHT))
        out.append(_box("rail", "steel", "steel", *sorted((s * LIP_INNER, s * LIP_EDGE)), -L, L,
                        RAIL_HEIGHT - LIP_THICK, RAIL_HEIGHT))
    return out


def _hex(af: float, start: float = 0.0):
    """A hexagon's corners (across flats ``af``), counter-clockwise, the first at angle ``start``: with start 0 a
    corner points along +x and two flats face +-y."""
    rc = af / 2 / math.cos(math.pi / 6)
    return [(rc * math.cos(start + k * math.pi / 3), rc * math.sin(start + k * math.pi / 3)) for k in range(6)]


def screw_shapes(body: str = "screw", shank_r: float = SHANK_R) -> list[Shape]:
    """Dog point, shank and a cylindrical head with a hexagon socket (two flats facing +-y). The head below the socket
    is one solid; round the socket it collides as six convex pieces, each between one of the socket's flats and the
    head's rim, with the 45 degree entry chamfer at the top (a key within ~0.5 mm drops in)."""
    out = [_prism(body, "zinc", "screw", _ngon(DOG_R, 12), 0.0, DOG_LEN, plane="xy"),
           _prism(body, "zinc", "screw", _ngon(shank_r, 16), DOG_LEN, SCREW_LEN, plane="xy")]
    z0, z1 = SCREW_LEN, SCREW_LEN + HEAD_H
    zs = z1 - SOCKET_DEPTH
    if HEAD_BEARING:
        # the head's underside bears on small balls on a ring outside the rail's slot: a flat underside straddling
        # the slot rests on the web pieces' edges, where PhysX may push sideways and let the head sink (bench 4-5)
        for k in range(BEARING_N):
            a = 2 * math.pi * (k + 0.5) / BEARING_N
            out.append(Shape(body, "zinc", "screw", (BEARING_RING * math.cos(a), BEARING_RING * math.sin(a),
                                                     z0 + BEARING_R), radius=BEARING_R, axis="S", visible=False))
        out.append(_prism(body, "zinc", None, _ngon(HEAD_R, 24), z0, zs, plane="xy"))  # the look
        out.append(_prism(body, "zinc", "screw", _ngon(HEAD_R, 24), z0 + 0.3 * MM, zs, plane="xy", visible=False))
    else:
        out.append(_prism(body, "zinc", "screw", _ngon(HEAD_R, 24), z0, zs, plane="xy"))
    out.append(_prism(body, "black", None, _hex(SOCKET_AF), zs, zs + 0.05 * MM, plane="xy"))  # the socket's floor
    inner = _hex(SOCKET_AF)
    mouth = _hex(SOCKET_AF + 2 * SOCKET_CHAMFER)
    c = SOCKET_CHAMFER
    for k in range(6):
        a0 = math.atan2(inner[k][1], inner[k][0])
        a1 = a0 + math.pi / 3
        rim = _ngon(HEAD_R, 4, a0=a0, a1=a1)  # the rim over this flat's sector
        sector = [inner[k]] + rim + [inner[(k + 1) % 6]]
        sector = [(x, y) for x, y in sector]
        out.append(_prism(body, "zinc", None, sector, zs, z1, plane="xy"))  # the look (without the chamfer)
        pts = ([(x, y, zs) for x, y in sector] + [(x, y, z1) for x, y in rim]
               + [(x, y, z1 - c) for x, y in (inner[k], inner[(k + 1) % 6])]
               + [(x, y, z1) for x, y in (mouth[k], mouth[(k + 1) % 6])])
        out.append(Shape(body, "zinc", "screw", hull=tuple(pts), visible=False))
    return out


def holder_shapes() -> list[Shape]:
    """A block (origin: the centre of its top face) with three square holes through it: two screw seats with collars,
    and the screwdriver's."""
    hx, hy, hz = (v / 2 for v in HOLDER_SIZE)
    holes = [(x, HOLDER_HOLE_HALF) for x in HOLDER_HOLES] + [(HOLDER_DRIVER, HOLDER_DRIVER_HALF)]
    hmax = max(h for _, h in holes)
    out = [_box("holder", "steel", "steel", -hx, hx, -hy, -hmax, -2 * hz, 0.0),
           _box("holder", "steel", "steel", -hx, hx, hmax, hy, -2 * hz, 0.0)]
    edges = [-hx] + [v for x, h in holes for v in (x - h, x + h)] + [hx]
    for x0, x1 in zip(edges[::2], edges[1::2]):  # between the holes, across the middle strip
        out.append(_box("holder", "steel", "steel", x0, x1, -hmax, hmax, -2 * hz, 0.0))
    for x, h in holes:  # a narrower hole than the strip: fill the strip's rest beside it
        if h < hmax:
            out += [_box("holder", "steel", "steel", x - h, x + h, -hmax, -h, -2 * hz, 0.0),
                    _box("holder", "steel", "steel", x - h, x + h, h, hmax, -2 * hz, 0.0)]
        out.append(Shape("holder", "hole", None, (x, 0.0, -hz - 0.05 * MM), radius=h * 0.98, length=2 * hz))
    b, bh, h = HOLDER_BOSS_HALF, HOLDER_BOSS_H, HOLDER_HOLE_HALF
    for x in HOLDER_HOLES:
        out += [_box("holder", "steel", "steel", x - b, x + b, -b, -h, 0.0, bh),
                _box("holder", "steel", "steel", x - b, x + b, h, b, 0.0, bh),
                _box("holder", "steel", "steel", x - b, x - h, -h, h, 0.0, bh),
                _box("holder", "steel", "steel", x + h, x + b, -h, h, 0.0, bh)]
    return out


def thread_shapes() -> list[Shape]:
    return screw_shapes("screw", THREAD_SHANK_R)


def driver_shapes() -> list[Shape]:
    """A hexagon blade (the key; flats facing +-y like the socket's) with a chamfered tip, a round shaft, a collar and
    the handle."""
    key = _hex(KEY_AF)
    tip = _hex(KEY_AF - 2 * KEY_TIP_CHAMFER)
    c = KEY_TIP_CHAMFER
    out = [Shape("driver", "steel", "steel", hull=tuple([(x, y, 0.0) for x, y in tip] + [(x, y, c) for x, y in key]),
                 visible=False),
           _prism("driver", "steel", None, tip, 0.0, c, plane="xy"),
           _prism("driver", "steel", "steel", key, c, BLADE_LENGTH, plane="xy")]
    shaft = _prism("driver", "steel", "steel", _ngon(SHAFT_R, 12), BLADE_LENGTH, SHAFT_TOP, plane="xy")
    collar = _prism("driver", "black", "handle", _ngon(COLLAR_R, 16), SHAFT_TOP, COLLAR_TOP, plane="xy")
    handle = _prism("driver", "handle", "handle", _ngon(HANDLE_R, 16), COLLAR_TOP, HANDLE_TOP - 4 * MM, plane="xy")
    cap = _prism("driver", "black", "handle", _ngon(HANDLE_R - 2 * MM, 16), HANDLE_TOP - 4 * MM, HANDLE_TOP,
                 plane="xy")
    return out + [shaft, collar, handle, cap]


SHAPES = {"plate": plate_shapes, "rail": rail_shapes, "screw": screw_shapes, "thread": thread_shapes,
          "driver": driver_shapes, "holder": holder_shapes}
BODIES = {"plate": ("plate",), "rail": ("rail",), "screw": ("screw",), "driver": ("driver",), "holder": ("holder",),
          "thread": ("anchor", "carrier", "clamp", "screw")}
MASSES = {"rail": RAIL_MASS, "screw": SCREW_MASS, "driver": DRIVER_MASS, "holder": HOLDER_MASS,
          "anchor": ANCHOR_MASS, "carrier": LINK_MASS, "clamp": LINK_MASS}
THREAD_JOINTS = ("thread", "clamp", "spin")


@lru_cache(maxsize=None)
def bounds(kind: str):
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for sh in SHAPES[kind]():
        for p in sh.points():
            for i in range(3):
                lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    return tuple(lo), tuple(hi)


@lru_cache(maxsize=None)
def local_center(kind: str):
    lo, hi = bounds(kind)
    return tuple((a + b) / 2 for a, b in zip(lo, hi, strict=True))


def _triangulate(poly):
    """Ear-clipping triangulation of a simple counter-clockwise polygon."""
    idx = list(range(len(poly)))
    tris = []

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def inside(p, a, b, c):
        return cross(a, b, p) >= 0 and cross(b, c, p) >= 0 and cross(c, a, p) >= 0

    guard = 0
    while len(idx) > 3 and guard < 100000:
        guard += 1
        for k in range(len(idx)):
            i0, i1, i2 = idx[k - 1], idx[k], idx[(k + 1) % len(idx)]
            a, b, c = poly[i0], poly[i1], poly[i2]
            if cross(a, b, c) <= 1e-14:
                continue
            if any(inside(poly[j], a, b, c) for j in idx if j not in (i0, i1, i2)):
                continue
            tris.append((i0, i1, i2))
            idx.pop(k)
            break
        else:
            raise ValueError("polygon is not simple or not counter-clockwise")
    tris.append(tuple(idx))
    return tris


def polygon_area(poly) -> float:
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, list(poly[1:]) + [poly[0]]))


# -- task-file objects --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin, ``roll``/``pitch`` turns
    about the world x/y axes applied before ``yaw``."""

    kind: str = "rail"
    roll: float = 0.0
    pitch: float = 0.0

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name, kind, pos, yaw=0.0, roll=0.0, pitch=0.0, fixed=False) -> Part:
    lo, hi = bounds(kind)
    mass = None if fixed or kind in ("plate", "thread") else MASSES[kind]
    color = {"plate": LOOKS["plate"][0], "rail": LOOKS["steel"][0], "screw": LOOKS["zinc"][0],
             "thread": LOOKS["zinc"][0], "driver": LOOKS["handle"][0], "holder": LOOKS["steel"][0]}[kind]
    size = tuple(b - a for a, b in zip(lo, hi, strict=True))
    return Part(name=name, pos=tuple(pos), size=size, mass=mass, color=color, yaw=yaw, kind=kind, roll=roll,
                pitch=pitch)


def twin_name(screw: str) -> str:
    return f"{screw}_thread"


def screw_parts(name: str, pos, park_y: float, yaw=0.0, roll=0.0, pitch=0.0) -> list[Part]:
    """A loose screw and its parked threaded twin."""
    return [part(name, "screw", pos, yaw=yaw, roll=roll, pitch=pitch),
            part(twin_name(name), "thread", (PARK[0], park_y, PARK[2]))]


def part_quat_xyzw(spec) -> tuple[float, float, float, float]:
    """Roll about x, then pitch about y, then yaw about z (all world axes)."""
    def q(axis, a):
        s = math.sin(a / 2)
        return (s * (axis == 0), s * (axis == 1), s * (axis == 2), math.cos(a / 2))

    return _qmul(q(2, spec.yaw), _qmul(q(1, spec.pitch), q(0, spec.roll)))


def _qmul(p, r):
    px, py, pz, pw = p
    rx, ry, rz, rw = r
    return (pw * rx + px * rw + py * rz - pz * ry, pw * ry - px * rz + py * rw + pz * rx,
            pw * rz + px * ry - py * rx + pz * rw, pw * rw - px * rx - py * ry - pz * rz)


@dataclass(frozen=True)
class ScrewState(task_file.ObjectState):
    """A screw's ``ObjectState`` (of whichever body is live) plus its thread: ``engaged`` (caught in a hole's
    thread), ``hole`` (the plate's hole index, -1 if loose), ``turns`` (turned in since it caught), ``clamp_force``
    (N, the spring under its head)."""

    engaged: bool = False
    hole: int = -1
    turns: float = 0.0
    clamp_force: float = 0.0
    past_contact: float = 0.0  # turns since the head first touched (0 before)

    def tight(self) -> bool:
        return bool(self.engaged and self.past_contact >= TIGHT_PAST and self.clamp_force >= TIGHT_CLAMP)


# -- USD (after the simulator has started) ------------------------------------------------------------------

_USD_DIR: Path | None = None


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _apply(prim, schema: str, attrs: dict) -> None:
    from pxr import Sdf

    prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float,
             str: Sdf.ValueTypeNames.Token}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _prism_mesh(sh: Shape):
    prof = list(sh.profile)
    if polygon_area(prof) < 0:
        prof = prof[::-1]
    n = len(prof)
    s0, s1 = sh.span
    pts = [sh.lift(p, s0) for p in prof] + [sh.lift(p, s1) for p in prof]
    counts, indices = [], []
    for a, b, c in _triangulate(prof):
        counts += [3, 3]
        indices += [a, b, c, n + c, n + b, n + a]
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        indices += [i, n + i, n + j, j]
    return pts, counts, indices


def write_usd(kind: str, path: Path, fixed: bool = False) -> Path:
    """One part as a USD file: bodies, colliders, looks, materials and (for a thread twin) its joints."""
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, "/Part").GetPrim()
    stage.SetDefaultPrim(root)

    looks, mats = {}, {}
    for name, (rgb, rough, metal) in LOOKS.items():
        mat = UsdShade.Material.Define(stage, f"/Part/Looks/{name}")
        shader = UsdShade.Shader.Define(stage, f"/Part/Looks/{name}/shader")
        shader.CreateIdAttr("UsdPreviewSurface")
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*map(_srgb_to_linear, rgb)))
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        looks[name] = mat
    for name, (static, dynamic, combine) in MATERIALS.items():
        mat = UsdShade.Material.Define(stage, f"/Part/PhysicsMaterials/{name}")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(static)
        api.CreateDynamicFrictionAttr(dynamic)
        api.CreateRestitutionAttr(0.0)
        _apply(mat.GetPrim(), "PhysxMaterialAPI", {"physxMaterial:frictionCombineMode": combine})
        mats[name] = mat

    jointed = kind == "thread"
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        prim = UsdGeom.Xform.Define(stage, path_b).GetPrim() if jointed else root
        body_paths[body] = path_b
        if fixed:
            continue
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(MASSES[body])
        if kind == "screw":
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(SCREW_INERTIA, SCREW_INERTIA, SCREW_INERTIA))
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 5.5 * MM))  # near the head
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:solverPositionIterationCount": 32,
            "physxRigidBody:solverVelocityIterationCount": 4,
            "physxRigidBody:maxDepenetrationVelocity": 0.3,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        if sh.hull is not None:
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/hull_{i}")
            pts = list(sh.hull)
            n = len(pts)
            # a fan over the points: only their convex hull matters (it is cooked from the points; invisible)
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * (n - 2)))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray([v for j in range(1, n - 1) for v in (0, j, j + 1)]))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        elif sh.profile is not None:
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/prism_{i}")
            pts, counts, indices = _prism_mesh(sh)
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            geom.CreateDoubleSidedAttr(True)
        elif sh.size is not None:
            geom = UsdGeom.Cube.Define(stage, f"{parent}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*sh.center))
            geom.AddScaleOp().Set(Gf.Vec3f(*sh.size))
        elif sh.axis == "S":
            geom = UsdGeom.Sphere.Define(stage, f"{parent}/sphere_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*sh.center))
        else:
            geom = UsdGeom.Cylinder.Define(stage, f"{parent}/cyl_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.CreateHeightAttr(sh.length)
            geom.CreateAxisAttr(sh.axis)
            geom.AddTranslateOp().Set(Gf.Vec3d(*sh.center))
        prim = geom.GetPrim()
        if not sh.visible:
            UsdGeom.Imageable(prim).MakeInvisible()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(looks[sh.look])
        if sh.material is None:
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        if sh.profile is not None or sh.hull is not None:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI(prim).Bind(mats[sh.material], UsdShade.Tokens.strongerThanDescendants, "physics")
        _apply(prim, "PhysxCollisionAPI", {"physxCollision:contactOffset": CONTACT_OFFSET,
                                           "physxCollision:restOffset": 0.0})

    if jointed:
        # the articulation root on the anchor body (not /Part): Isaac Lab's fix_root_link needs a rigid body there
        anchor = stage.GetPrimAtPath(body_paths["anchor"])
        UsdPhysics.ArticulationRootAPI.Apply(anchor)
        _apply(anchor, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        ident = Gf.Quatf(1, 0, 0, 0)
        turns = THREAD_RANGE / PITCH
        joints = {  # name: (type, parent, child, lower, upper)
            "thread": ("pri", "anchor", "carrier", -THREAD_RANGE, THREAD_RANGE),
            "clamp": ("pri", "carrier", "clamp", 0.0, CLAMP_TRAVEL),
            "spin": ("rev", "clamp", "screw", -math.degrees(2 * math.pi * turns), math.degrees(2 * math.pi * turns)),
        }
        for name, (jt, parent, child, lo, hi) in joints.items():
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(
                stage, f"/Part/joints/{name}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr("Z")
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr(ident)
            joint.CreateLocalRot1Attr(ident)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if jt == "rev" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(CLAMP_SPRING if name == "clamp" else 0.0)
            drive.CreateDampingAttr(20.0 if name == "clamp" else 0.0)
            drive.CreateTargetPositionAttr(0.0)
            _apply(joint.GetPrim(), "PhysxJointAPI",
                   {"physxJoint:armature": SCREW_ARMATURE if name == "spin" else LINK_ARMATURE})
        # the thread: spin (deg) + gearing * thread (m) = 0. Clockwise seen from above is negative about +z, and
        # turns the screw down: one turn in (-360 deg) moves the thread slider -PITCH
        spin = stage.GetPrimAtPath("/Part/joints/spin")
        _apply(spin, "PhysxMimicJointAPI:rotZ", {"physxMimicJoint:rotZ:gearing": float(-360.0 / (PITCH / 1.0)),
                                                 "physxMimicJoint:rotZ:offset": 0.0})
        spin.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets([Sdf.Path("/Part/joints/thread")])
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str, fixed: bool = False) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="rail_plate_parts_"))
    path = _USD_DIR / f"{kind}{'_fixed' if fixed else ''}.usda"
    if not path.exists():
        write_usd(kind, path, fixed)
    return str(path)


_THREADS: dict[str, str] = {}  # loose screw name -> its twin's name
_HOLES: list[tuple[float, float, float]] = []  # the plates' holes: (x, y, the plate top's z), env frame


def arena_object(spec: Part):
    """The Arena object for a part: a plate a fixed collider, a thread twin a world-fixed articulation, the rest free
    rigid bodies."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    class _Thread(Object):
        """A thread twin: the clamp spring and the thread friction, written to PhysX by implicit actuators."""

        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "thread": ImplicitActuatorCfg(joint_names_expr=["thread"], stiffness=0.0, damping=0.0,
                                              armature=LINK_ARMATURE),
                "clamp": ImplicitActuatorCfg(joint_names_expr=["clamp"], stiffness=CLAMP_SPRING, damping=20.0,
                                             armature=LINK_ARMATURE),
                "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=0.0, damping=0.0,
                                            armature=SCREW_ARMATURE, friction=FREE_TORQUE,
                                            dynamic_friction=FREE_TORQUE),
            }
            cfg.init_state.joint_pos = {"thread": 0.0, "clamp": 0.0, "spin": 0.0}
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=part_quat_xyzw(spec))
    if spec.kind == "plate":
        for y in HOLE_Y:  # on the plate's centreline (its local y), turned by its yaw
            _HOLES.append((spec.pos[0] - math.sin(spec.yaw) * y, spec.pos[1] + math.cos(spec.yaw) * y, spec.pos[2]))
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file("plate", fixed=True))
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    if spec.kind == "thread":
        screw = spec.name[: -len("_thread")]
        _THREADS[screw] = spec.name
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file("thread"))
        cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=4,
            fix_root_link=True)
        return _Thread(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)
    fixed = spec.mass is None
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind, fixed))
    return Object(name=spec.name, object_type=ObjectType.BASE if fixed else ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=pose)


# -- the thread swap ------------------------------------------------------------------------------------------


def _rot(quat_xyzw):
    import numpy as np

    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _yaw_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


@dataclass
class _Track:
    engaged: bool = False
    hole: int = -1
    turned: float = 0.0  # clockwise angle while seated in a hole (rad, positive)
    contact: float | None = None  # turns at which the head first touched (None: not touching yet)
    pressing: int = 0  # control steps in a row with the clamp at CONTACT_CLAMP or more
    pressing_from: float = 0.0  # turns when that run began
    last_yaw: float | None = None
    swaps: int = 0


_TRACK: dict[str, _Track] = {}
_EPISODE_STEP: list = [None]


def _np(x):
    import numpy as np

    return (x.torch if hasattr(x, "torch") else x)[0].detach().cpu().numpy().astype(np.float64)


def _tensor(base, rows):
    import torch

    return torch.tensor(rows, dtype=torch.float32, device=base.device)


def _set_rigid(base, name, pos_env, quat_xyzw):
    obj = base.scene.rigid_objects[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = [float(v) for v in (pos_env + origin)]
    obj.write_root_pose_to_sim_index(root_pose=_tensor(base, [p + list(quat_xyzw)]), env_ids=[0])
    obj.write_root_velocity_to_sim_index(root_velocity=_tensor(base, [[0.0] * 6]), env_ids=[0])


def _set_twin(base, name, pos_env, quat_xyzw, spin_vel: float = 0.0):
    art = base.scene.articulations[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = [float(v) for v in (pos_env + origin)]
    art.write_root_pose_to_sim_index(root_pose=_tensor(base, [p + list(quat_xyzw)]), env_ids=[0])
    names = list(art.joint_names)
    pos = [[0.0] * len(names)]
    vel = [[0.0] * len(names)]
    vel[0][names.index("spin")] = spin_vel
    vel[0][names.index("thread")] = spin_vel / (2 * math.pi) * PITCH  # consistent with the mimic coupling
    art.write_joint_position_to_sim_index(position=_tensor(base, pos), env_ids=[0])
    art.write_joint_velocity_to_sim_index(velocity=_tensor(base, vel), env_ids=[0])


def twin_mechanism(base, twin: str) -> dict:
    art = base.scene.articulations[twin]
    q = _np(art.data.joint_pos)
    names = list(art.joint_names)
    thread, clamp = float(q[names.index("thread")]), float(q[names.index("clamp")])
    return {"turns": -thread / PITCH, "clamp": clamp, "clamp_force": CLAMP_SPRING * max(0.0, clamp)}


def _twin_screw_pose(base, twin: str):
    """The twin's screw body pose (env frame position, xyzw quaternion)."""
    art = base.scene.articulations[twin]
    i = list(art.body_names).index("screw")
    pos = (art.data.body_pos_w.torch if hasattr(art.data.body_pos_w, "torch") else art.data.body_pos_w)[0, i]
    quat = (art.data.body_quat_w.torch if hasattr(art.data.body_quat_w, "torch") else art.data.body_quat_w)[0, i]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    return pos.detach().cpu().numpy().astype(float) - origin, quat.detach().cpu().numpy().astype(float)


def park_pos(base, name: str):
    """Where a part waits out of sight: the twin's spawn place (for a loose screw: 5 cm beside its twin's)."""
    import numpy as np

    if name in _THREADS.values():
        return np.array(_PARKED[name])
    twin = _THREADS[name]
    p = np.array(_PARKED[twin])
    return p + np.array([0.0, 0.05, -PARK[2] + 0.01])


_PARKED: dict[str, tuple] = {}


def update_threads(base) -> None:
    """Catch or release each screw's thread (once per control step, see the module docstring)."""
    import numpy as np

    step = getattr(base, "episode_length_buf", None)
    step = None if step is None else int(step[0])
    if step == 0 or (_EPISODE_STEP[0] is not None and step is not None and step < _EPISODE_STEP[0]):
        _TRACK.clear()  # a new episode: everything starts loose
    _EPISODE_STEP[0] = step
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for screw, twin in _THREADS.items():
        if twin not in _PARKED:
            _PARKED[twin] = tuple(_np(base.scene.articulations[twin].data.root_pose_w)[:3] - origin)
        tr = _TRACK.setdefault(screw, _Track())
        if not tr.engaged:
            pose = _np(base.scene.rigid_objects[screw].data.root_pose_w)
            tip, rot = pose[:3] - origin, _rot(pose[3:])
            tilt = math.acos(min(1.0, max(-1.0, rot[2, 2])))
            seated = None
            r_ok = ENGAGE_R + (ENGAGE_R_HYST if tr.turned > 0.0 else 0.0)  # once it is being turned, a little more
            for i, (hx, hy, hz) in enumerate(_HOLES):
                if (math.hypot(tip[0] - hx, tip[1] - hy) < r_ok
                        and hz - SEAT_DEPTH - ENGAGE_DZ < tip[2] < hz - ENGAGE_MIN_DEPTH
                        and tilt < ENGAGE_TILT):
                    seated = i
            if seated is None:
                tr.turned, tr.last_yaw = 0.0, None
                continue
            yaw = math.atan2(rot[1, 0], rot[0, 0])
            if tr.last_yaw is not None:
                d = (yaw - tr.last_yaw + math.pi) % (2 * math.pi) - math.pi
                tr.turned = max(0.0, tr.turned - d)  # clockwise (negative about +z) counts
            tr.last_yaw = yaw
            if tr.turned < START_ANGLE:
                continue
            # caught: the twin takes the screw's place, centred on the hole and upright, at the screw's angle
            hx, hy, _ = _HOLES[seated]
            _set_twin(base, twin, np.array([hx, hy, tip[2]]), _yaw_quat(yaw))  # at rest: the thread just caught
            _set_rigid(base, screw, park_pos(base, screw), (0.0, 0.0, 0.0, 1.0))
            tr.engaged, tr.hole, tr.turned, tr.last_yaw = True, seated, 0.0, None
            tr.swaps += 1
        else:
            m = twin_mechanism(base, twin)
            if m["clamp_force"] >= CONTACT_CLAMP:
                if tr.pressing == 0:
                    tr.pressing_from = m["turns"]
                tr.pressing += 1
            else:
                tr.pressing = 0
            # the head touches once the clamp has held for CONTACT_STEPS in a row (a key lifted out of the socket
            # pulls the screw up for a step or two, which read as contact half a turn in, demo run 35)
            if tr.contact is None and tr.pressing >= CONTACT_STEPS:
                tr.contact = tr.pressing_from
            elif tr.contact is not None and m["turns"] < tr.contact - 0.05 and m["clamp_force"] < CONTACT_CLAMP:
                tr.contact = None  # loosened: the head is off again
            if m["turns"] > -BACKOUT_TURNS:
                continue
            # backed out past where it caught: loose again, where the twin's screw is
            pos, quat = _twin_screw_pose(base, twin)
            _set_rigid(base, screw, pos + np.array([0.0, 0.0, 0.05 * MM]), quat)
            _set_twin(base, twin, np.array(_PARKED[twin]), (0.0, 0.0, 0.0, 1.0))
            tr.engaged, tr.hole, tr.contact, tr.pressing = False, -1, None, 0
            tr.swaps += 1


def _screw_state(base, screw: str, rigid_state) -> ScrewState:
    import numpy as np

    tr = _TRACK.get(screw, _Track())
    if not tr.engaged:
        return ScrewState(pos=rigid_state.pos, quat_xyzw=rigid_state.quat_xyzw, lin_vel=rigid_state.lin_vel,
                          ang_vel=rigid_state.ang_vel, low=rigid_state.low, high=rigid_state.high)
    twin = _THREADS[screw]
    pos, quat = _twin_screw_pose(base, twin)
    rot = _rot(quat)
    lo, hi = bounds("screw")
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ rot.T + pos
    art = base.scene.articulations[twin]
    i = list(art.body_names).index("screw")
    lin = (art.data.body_lin_vel_w.torch if hasattr(art.data.body_lin_vel_w, "torch") else art.data.body_lin_vel_w)
    ang = (art.data.body_ang_vel_w.torch if hasattr(art.data.body_ang_vel_w, "torch") else art.data.body_ang_vel_w)
    m = twin_mechanism(base, twin)
    return ScrewState(pos=rot @ np.array(local_center("screw")) + pos, quat_xyzw=quat,
                      lin_vel=lin[0, i].detach().cpu().numpy().astype(float),
                      ang_vel=ang[0, i].detach().cpu().numpy().astype(float), low=corners.min(axis=0),
                      high=corners.max(axis=0), engaged=True, hole=tr.hole, turns=m["turns"],
                      clamp_force=m["clamp_force"],
                      past_contact=0.0 if tr.contact is None else m["turns"] - tr.contact)


def _twin_state(base, twin: str):
    import numpy as np

    pose = _np(base.scene.articulations[twin].data.root_pose_w)
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    p = pose[:3] - origin
    z = np.zeros(3)
    return task_file.ObjectState(pos=p, quat_xyzw=pose[3:], lin_vel=z, ang_vel=z, low=p, high=p)


# -- the hook into yam_arena.scene --------------------------------------------------------------------------


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "rp_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_rp_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        base = env.unwrapped
        update_threads(base)
        twins = {n for n in names if n in _THREADS.values()}
        out = original_states(env, [n for n in names if n not in twins])
        for n in names:
            if n in twins:
                out[n] = _twin_state(base, n)
            elif n in _THREADS:
                out[n] = _screw_state(base, n, out[n])
        return {n: out[n] for n in names}

    task_object._spawns_rp_parts = True
    task_object.__doc__ = original.__doc__
    object_states.__doc__ = original_states.__doc__
    scene_module.task_object = task_object
    scene_module.object_states = object_states


class _SceneHook(importlib.abc.MetaPathFinder):
    """Patches ``yam_arena.scene`` right after it is first imported (after the simulator has started)."""

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        spec = importlib.machinery.PathFinder.find_spec(fullname, path)
        if spec is None or spec.loader is None:
            return None
        exec_module = spec.loader.exec_module

        def exec_and_patch(module):
            exec_module(module)
            _patch(module)

        spec.loader.exec_module = exec_and_patch
        return spec


def install() -> None:
    """Make yam_arena spawn :class:`Part` objects. Safe to call more than once."""
    if "yam_arena.scene" in sys.modules:
        _patch(sys.modules["yam_arena.scene"])
    elif not any(isinstance(f, _SceneHook) for f in sys.meta_path):
        sys.meta_path.insert(0, _SceneHook())
