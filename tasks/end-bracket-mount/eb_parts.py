"""The task's own parts: a loose TS35 DIN rail with a row of terminal blocks mounted on it, a screw-fixed end
bracket with a working clamping screw, and a flat-blade screwdriver. All geometry is our own simplified design,
drawn from the TS35 rail standard (EN 60715) and the screw's standard (DIN 84); no manufacturer CAD is used.

yam_arena task files support single primitives and library assets only, and yam_arena has no extension point for
multi-shape or jointed parts. This module is the task's own minimal mechanism, kept in the task folder (the same
approach as the terminal-block-mount task, copied rather than imported so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` (so the stock loader accepts it and the scorer gets an
  ``ObjectState``) that also names which part it is and how it is turned.
- :func:`install` wraps ``yam_arena.scene.task_object`` and ``object_states`` the moment that module is imported (it
  imports Isaac Sim, so it is never imported here before the simulator starts). Every :class:`Part` is spawned from
  a USD file this module writes; every other object goes through yam_arena unchanged.

The parts (dimensions in mm in comments, metres in code):

- rail (one free rigid body): steel TS35 x 7.5 top-hat rail lying web-down, 1 mm sheet but 2 mm lips (thickened so
  contacts cannot tunnel through them at 240 Hz), lip outer edges at +-17.5 from the centreline. A row of ROW_BLOCKS
  beige terminal blocks (our own design of a generic 2.5 mm2 screw-clamp block, 5.2 thick, 52 across, 40 tall) is
  mounted on it as part of the same body. No end brackets.
- bracket (an articulation of four bodies): a screw-fixed end bracket of our own design (8 wide along the rail, 56
  across, 46 tall), its outline coded below as coordinates.
- driver: a stubby flat-blade screwdriver for M4 slotted screws (blade 0.8 x 4.0, 25 mm shaft, 26 mm handle).

The bracket mechanism (our design, a common principle for screw-fixed end brackets):
- Fixed hook on the -x side: reaches 1.5 under that lip with 0.8 vertical play. The bracket goes on by hooking this
  side under its lip and tilting the bracket down over the other lip (or by sliding it on from a free rail end).
- Clamping jaw on the +x side, driven by the M4 clamping screw through a wedge: turning the screw clockwise (seen
  from above) moves the jaw inward under the +x lip and up against its underside, clamping the lip between the jaw
  and the bracket's seat. Screw rotation and jaw travel are coupled by a PhysX mimic joint (no scripted motion).
- The jaw sits on a stiff spring (the compliance of the plastic housing and the wedge): its compression is the
  clamp. The screw turns against a constant friction (FREE_TORQUE). The scorer calls the bracket tight when the screw
  has turned TIGHT_TURNS (0.25 turn past the jaw's first contact) and the jaw presses on the lip: the simulation's
  stand-in for the rated 0.5 Nm tightening torque. (A friction rising with the clamp would need the screw joint's friction rewritten
  to PhysX while it turns, and each rewrite kicks a turning screw: the screwdriver jumps out of the slot.)

Frames: the rail's origin is the centre of its web's underside (x across the rail, y along it, z up). The bracket's
origin is on the rail's centreline at the plane of the lip tops when seated (x across the rail, y through its 8 mm
thickness, z up); the screw's axis is its z axis. The driver's origin is its blade tip, its shaft along +z.
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

# -- rail (TS35 x 7.5, EN 60715) ----------------------------------------------------------------------------
RAIL_LENGTH = 240 * MM
RAIL_HEIGHT = 7.5 * MM  # top of the lips above the table
SHEET = 1.0 * MM  # web and side walls
LIP_THICK = 2.0 * MM  # simulation: a real rail's lips are 1 mm (thickened so contacts don't tunnel at 240 Hz)
LIP_EDGE = 17.5 * MM  # lips' outer edges, from the centreline
LIP_INNER = 13.5 * MM  # outer faces of the side walls, where the lips start
ROW_BLOCKS = 4  # beige terminal blocks mounted side by side on the rail: the terminal strip the bracket closes off
BLOCK_MASS = 0.0075  # kg per block (estimated: a polyamide housing with a small metal clamp and two screws)
RAIL_MASS = 0.10 + ROW_BLOCKS * BLOCK_MASS  # kg: 24 cm of 1 mm steel TS35 rail plus the mounted blocks

# -- the mounted terminal block (beige; our own simplified design of a generic 2.5 mm2 screw-clamp feed-through
# block: two wire entries, two screws in wells on top, a marker groove between them) -------------------------------
# Section coordinates (across, up) in mm in the drawing frame: across 0..52 (the rail's centreline at 26), up from the
# block's base, which rests BLOCK_SEATED above the lip tops. Symmetric about the centreline.
_BLOCK_ACROSS0 = 26.0
_BLOCK_OUTLINE = (
    (8.5, 0.0), (43.5, 0.0), (43.5, 5.0), (52.0, 8.0), (52.0, 15.0), (44.0, 19.0), (44.0, 25.0), (50.0, 30.0),
    (50.0, 35.0), (45.0, 40.0), (40.0, 40.0), (40.0, 33.0), (36.0, 33.0), (36.0, 40.0), (30.0, 40.0), (30.0, 37.0),
    (22.0, 37.0), (22.0, 40.0), (16.0, 40.0), (16.0, 33.0), (12.0, 33.0), (12.0, 40.0), (7.0, 40.0), (2.0, 35.0),
    (2.0, 30.0), (8.0, 25.0), (8.0, 19.0), (0.0, 15.0), (0.0, 8.0), (8.5, 5.0),
)
_BLOCK_CORE = (  # convex collision pieces on the centreline (the rest: _BLOCK_HALF_HULLS and their mirror images)
    ((8.5, 0.0), (43.5, 0.0), (43.5, 33.0), (8.5, 33.0)),  # core, under the screw wells
    ((16.0, 33.0), (36.0, 33.0), (36.0, 37.0), (16.0, 37.0)),  # between the wells, under the marker groove
)
_BLOCK_HALF_HULLS = (  # the +across side
    ((43.5, 5.0), (52.0, 8.0), (52.0, 15.0), (44.0, 19.0), (43.5, 19.0)),  # lower wire-entry shoulder
    ((40.0, 33.0), (43.5, 25.0), (44.0, 25.0), (50.0, 30.0), (50.0, 35.0), (45.0, 40.0), (40.0, 40.0)),  # upper
    ((30.0, 37.0), (36.0, 37.0), (36.0, 40.0), (30.0, 40.0)),  # beside the marker groove
)
_BLOCK_SCREWS = (14.0, 38.0)  # across: the two clamping screws, in wells 4 mm wide ...
_BLOCK_WELL_FLOOR = 33.0  # ... with this floor
_BLOCK_FEET = (  # the two latch feet beside the lips, looks only (the block is part of the rail body)
    ((5.2, -6.5), (8.5, -6.5), (8.5, 5.0), (5.2, 6.2)),
    ((43.5, -6.5), (46.8, -6.5), (46.8, 6.2), (43.5, 5.0)),
)
BLOCK_WIDTH = 5.2 * MM  # along the rail
BLOCK_SEATED = 0.3 * MM  # the mounted block's base above the lip tops
# The row of beige blocks along the rail: the rightmost is centred on the rail's middle, the others to its left (+y).
BEIGE_Y = (-BLOCK_WIDTH / 2, -BLOCK_WIDTH / 2 + ROW_BLOCKS * BLOCK_WIDTH)
BEIGE_FACE_Y = BEIGE_Y[0]  # the row's free face, toward the rail's right end (-y): the bracket goes against it
HOLD_Y = 70.0 * MM  # where the left arm holds the rail (left of the block, clear of the right hand's work)

# -- end bracket: our own simplified design -------------------------------------------------------------------------
# Drawing frame: across 0..56, up 0..46 (feet at 0), 8.0 thick, symmetric about the centreline at across 28. What
# fixes it: the seat (the underside that rests on the lip tops) at up 7.0, across 10.3..45.7, 0.2 mm wider than the
# TS35 lips each side; skirts straddling the lips down to 0.5 mm above the table, their inner faces chamfered so the
# bracket can be set down over a lip; a 10 mm wide screw recess in the middle of the top for the M4 screw head, its
# floor 32 mm above the seat. The rest (straight sides with a waist, chamfered top corners) is plain styling.
# Bracket frame: x = across - 28, z = up - 7.0.
_BR_ACROSS0, _BR_UP0 = 28.0, 7.0
_BRACKET_OUTLINE = (
    (0.0, 0.0), (4.0, 0.0), (10.3, 7.0), (45.7, 7.0), (52.0, 0.0), (56.0, 0.0), (56.0, 15.0), (51.0, 20.0),
    (51.0, 33.0), (54.0, 36.0), (54.0, 40.0), (48.0, 46.0), (33.0, 46.0), (33.0, 39.0), (23.0, 39.0), (23.0, 46.0),
    (8.0, 46.0), (2.0, 40.0), (2.0, 36.0), (5.0, 33.0), (5.0, 20.0), (0.0, 15.0),
)
# Collision: convex pieces covering the left half (mirrored for the right). Every piece is at least 1 mm thick so
# PhysX does not inflate it.
_BR_HALF_HULLS = (
    ((0.0, 0.0), (4.0, 0.0), (10.3, 7.0), (10.3, 15.0), (0.0, 15.0)),  # skirt
    ((0.0, 15.0), (10.3, 15.0), (10.3, 20.0), (5.0, 20.0)),  # lower shoulder
    ((5.0, 20.0), (10.3, 20.0), (10.3, 33.0), (5.0, 33.0)),  # waist
    ((5.0, 33.0), (10.3, 33.0), (10.3, 36.0), (2.0, 36.0)),  # upper shoulder
    ((2.0, 36.0), (23.0, 36.0), (23.0, 46.0), (8.0, 46.0), (2.0, 40.0)),  # top corner, up to the recess
    ((10.3, 7.0), (28.0, 7.0), (28.0, 36.0), (10.3, 36.0)),  # body, to the centreline
    ((23.0, 36.0), (28.0, 36.0), (28.0, 39.0), (23.0, 39.0)),  # recess floor
)
BRACKET_WIDTH = 8.0 * MM  # along the rail
BRACKET_SEAT_HALF = (45.7 - 10.3) / 2 * MM  # the seat's half width (17.7): 0.2 mm wider than the lips each side
BRACKET_MASS = 0.014  # kg, all four bodies together (estimated: a cored polyamide moulding of this size with a steel
# jaw and screw; solid it would be about 18 g)
RECESS_HALF = 5.0 * MM  # screw recess: 10 wide across, open on both faces, floor at z 32.0
RECESS_FLOOR = (39.0 - _BR_UP0) * MM

# -- end bracket: the assumed mechanism ------------------------------------------------------------------------
# Hook (on the body, -x side): its top HOOK_PLAY below the lip's underside, its tip 1.5 inside the lip edge.
HOOK_REACH = 1.5 * MM
HOOK_PLAY = 0.8 * MM  # the clamp takes up the play, so the hook needs none; 0.8 mm leaves a usable window
# (2.8 mm between hook top and seat over a 2 mm lip) for tilting the bracket on
# Screw: M4 slotted cheese head (DIN 84: head 7.0 dia, 2.6 high, slot 1.2 wide, 1.1 deep), sitting on the recess floor.
SCREW_HEAD_R = 3.5 * MM
SCREW_HEAD_H = 2.6 * MM
SLOT_WIDTH = 1.2 * MM
SLOT_DEPTH = 1.1 * MM
SCREW_PITCH = 0.7 * MM  # M4 coarse thread, mm per turn
SLOT_CHAMFER = 0.3 * MM  # the slot's top edges chamfered 45 degrees (simulation: see bracket_shapes)
CAP_X, CAP_Y = 3.0 * MM, 2.3 * MM  # the head's chamfered top cap either side of the slot: half its length along the
# slot and its reach across it (corners at 3.78 mm from the axis; see bracket_shapes)
# Wedge: the jaw moves inward and up along a line JAW_ANGLE above the horizontal, JAW_LEAD per screw turn.
JAW_ANGLE = math.radians(20.0)
JAW_LEAD = 1.3 * MM  # along its line, per turn: 1.22 inward, 0.44 up
JAW_GAP = 0.6 * MM  # retracted: its top this far below the lip's underside ...
JAW_CLEAR = 0.3 * MM  # ... and its tip this far outside the lip edge, so the bracket can be set down over the lip
JAW_TRAVEL = 2.5 * JAW_LEAD  # 2.5 turns from fully retracted to the end stop
TURNS_TO_CONTACT = JAW_GAP / math.sin(JAW_ANGLE) / JAW_LEAD  # 1.35: the jaw touches the lip's underside
# The jaw's spring (housing and wedge compliance) and the screw's friction. Tuned on the bench: see README.
JAW_SPRING = 1.0e5  # N/m along the jaw's line
JAW_SPRING_TRAVEL = 1.5 * MM
REAL_TIGHT_TORQUE = 0.5  # Nm: the bracket's rated tightening torque (what a real trial checks; usual for an M4
# clamping screw in a plastic end bracket)
TIGHT_TURNS = 1.6  # "tight" in the simulation: the screw turned this far from backed out (0.25 turn past the jaw's
# first contact at 1.37) ...
TIGHT_CLAMP = 2.0  # N ... with the jaw pressing on the lip at least this hard. (The jaw's force itself isn't a usable
# measure: it creeps into the lip in PhysX and sagged from ~22 N to 8-11 N between runs; the turns are exact)
FREE_TORQUE = 0.03  # Nm: the screw's (constant) friction: thread friction in the plastic
SCREW_MASS = 0.0020
JAW_MASS = 0.0010
CARRIER_MASS = 0.0002
LINK_ARMATURE = 0.001  # kg: the jaw's and carrier's joint armature (more lets the jaw creep into the lip under load)
SCREW_ARMATURE = 2.0e-4  # kg m^2: about a screwdriver's and a hand's share engaged in the slot; the bare 2 g
# screw (~5e-9) is too light for 240 Hz steps against the jaw's spring through the thread


# -- screwdriver: a stubby flat-blade screwdriver for M4 slotted screws (0.8 x 4.0 mm blade, 25 mm shaft) ----------
# A stubby, as used in cramped panels: gripped coaxially on its handle, a standard 100 mm screwdriver puts the hand
# 21-23 cm above the table at this screw, beyond the YAM's reach pointing down over the rail (12-16 cm there).
BLADE_THICK = 0.8 * MM
BLADE_WIDTH = 4.0 * MM
BLADE_LENGTH = 6.0 * MM
SHAFT_R = 2.0 * MM
SHAFT_TOP = 25.0 * MM
COLLAR_R, COLLAR_TOP = 8.0 * MM, 31.0 * MM
HANDLE_R = 13.0 * MM
HANDLE_TOP = 85.0 * MM  # handle 31..81 mm, cap to 85 mm
DRIVER_MASS = 0.030

# -- materials and looks ------------------------------------------------------------------------------------
# (static friction, dynamic friction, combine mode). PhysX uses the combine mode of higher priority
# (average < min < multiply < max).
MATERIALS = {
    "steel": (0.30, 0.30, "average"),
    "polyamide": (0.35, 0.35, "min"),  # the bracket's housing on steel
    "screw": (0.20, 0.20, "average"),  # blade in the slot: oiled steel on steel
    "handle": (0.90, 0.80, "average"),  # the screwdriver's two-component handle (rubber-like grip zone)
}
LOOKS = {  # sRGB, roughness, metallic
    "steel": ((0.72, 0.74, 0.76), 0.35, 0.8),
    "zinc": ((0.80, 0.80, 0.78), 0.3, 0.9),
    "beige": ((0.86, 0.80, 0.66), 0.5, 0.0),
    "bracket": ((0.70, 0.62, 0.46), 0.5, 0.0),  # dark beige polyamide
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
    "handle": ((0.85, 0.12, 0.10), 0.6, 0.0),
    "black": ((0.08, 0.08, 0.09), 0.7, 0.0),
}
CONTACT_OFFSET = 0.5 * MM


# -- shapes -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One piece of a part, in its body's frame: a box (``center``, ``size``), a prism (a 2-D ``profile``
    extruded over ``span``: profile (x, z) along y when ``plane`` is "xz", (x, y) along z for "xy", (y, z) along x for
    "yz"; convex
    unless visual-only), or a visual cylinder (``center``, ``radius``, ``length`` along ``axis``)."""

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
    visible: bool = True  # False: a collider hidden behind a separate look

    def lift(self, p: tuple[float, float], s: float) -> tuple[float, float, float]:
        """A profile point at extrusion coordinate ``s`` in the body frame."""
        if self.plane == "xz":
            return (p[0], s, p[1])
        if self.plane == "yz":
            return (s, p[0], p[1])
        return (p[0], p[1], s)

    def points(self) -> list[tuple[float, float, float]]:
        """Corner points (for bounds)."""
        if self.profile is not None:
            return [self.lift(p, s) for p in self.profile for s in self.span]
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]

    def moved(self, dx: float = 0.0, dy: float = 0.0, dz: float = 0.0) -> "Shape":
        if self.profile is not None:
            if self.plane == "xz":
                prof, span = tuple((x + dx, z + dz) for x, z in self.profile), (self.span[0] + dy, self.span[1] + dy)
            elif self.plane == "yz":
                prof, span = tuple((y + dy, z + dz) for y, z in self.profile), (self.span[0] + dx, self.span[1] + dx)
            else:
                prof, span = tuple((x + dx, y + dy) for x, y in self.profile), (self.span[0] + dz, self.span[1] + dz)
            return Shape(self.body, self.look, self.material, profile=prof, span=span, plane=self.plane,
                         visible=self.visible)
        c = (self.center[0] + dx, self.center[1] + dy, self.center[2] + dz)
        return Shape(self.body, self.look, self.material, c, self.size, None, self.span, self.plane, self.radius,
                     self.length, self.axis, self.visible)


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _prism(body, look, material, profile, s0, s1, plane="xz", visible=True) -> Shape:
    return Shape(body, look, material, profile=tuple((float(a), float(b)) for a, b in profile), span=(s0, s1),
                 plane=plane, visible=visible)


def _ngon(r: float, n: int, start: float = 0.0, cx: float = 0.0, cy: float = 0.0, a0: float = 0.0,
          a1: float = 2 * math.pi) -> list[tuple[float, float]]:
    """Points on a circle (counter-clockwise) from angle a0 to a1; the full circle without repeating the start."""
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    k = n if full else n + 1
    return [(cx + r * math.cos(a0 + start + (a1 - a0) * i / n), cy + r * math.sin(a0 + start + (a1 - a0) * i / n))
            for i in range(k)]


def _blk(pts, mirror=False):
    """Block drawing (across, up) mm to the block frame (x, z) m (origin on its base, on the rail's centreline);
    ``mirror`` reflects across the centreline."""
    out = [(((a - _BLOCK_ACROSS0) * (-1 if mirror else 1)) * MM, u * MM) for a, u in pts]
    return out[::-1] if mirror else out


def _br(pts, mirror=False):
    """Bracket drawing (across, up) mm to the bracket frame (x, z) m; ``mirror`` reflects across the centreline."""
    out = [(((a - _BR_ACROSS0) * (-1 if mirror else 1)) * MM, (u - _BR_UP0) * MM) for a, u in pts]
    return out[::-1] if mirror else out


def beige_block_shapes(body: str, y: float, dz: float) -> list[Shape]:
    """One mounted beige block centred at ``y`` along the rail, ``dz`` = its base's height in the body frame. The
    housing collides; screws and feet are looks."""
    w = BLOCK_WIDTH / 2
    out = [_prism(body, "beige", None, _blk(_BLOCK_OUTLINE), y - w, y + w)]
    hulls = [_blk(h) for h in _BLOCK_CORE] + [_blk(h, m) for h in _BLOCK_HALF_HULLS for m in (False, True)]
    out += [_prism(body, "beige", "polyamide", h, y - w, y + w, visible=False) for h in hulls]
    for foot in _BLOCK_FEET:
        out.append(_prism(body, "beige", None, _blk(foot), y - w + 0.15 * MM, y + w - 0.15 * MM))
    for a in _BLOCK_SCREWS:
        x = (a - _BLOCK_ACROSS0) * MM
        out.append(Shape(body, "zinc", None, (x, y, (_BLOCK_WELL_FLOOR + 1.0) * MM), radius=1.9 * MM,
                         length=2.0 * MM))
    return [sh.moved(dz=dz) for sh in out]


def rail_shapes() -> list[Shape]:
    L = RAIL_LENGTH / 2
    out = [_box("rail", "steel", "steel", -LIP_INNER, LIP_INNER, -L, L, 0.0, SHEET)]
    for s in (1, -1):
        out.append(_box("rail", "steel", "steel", *sorted((s * (LIP_INNER - SHEET), s * LIP_INNER)), -L, L,
                        SHEET, RAIL_HEIGHT))
        out.append(_box("rail", "steel", "steel", *sorted((s * LIP_INNER, s * LIP_EDGE)), -L, L,
                        RAIL_HEIGHT - LIP_THICK, RAIL_HEIGHT))
    for i in range(ROW_BLOCKS):  # the terminal strip: blocks side by side, touching
        out += beige_block_shapes("rail", BEIGE_Y[0] + (i + 0.5) * BLOCK_WIDTH, RAIL_HEIGHT + BLOCK_SEATED)
    return out


def _hook_profile():
    """The fixed hook under the -x lip, (x, z) in the bracket frame, counter-clockwise."""
    lip_under = -LIP_THICK / MM
    top = lip_under - HOOK_PLAY / MM
    tip = -(LIP_EDGE - HOOK_REACH) / MM
    root = -22.5  # inside the skirt's inner chamfer
    return [(p[0] * MM, p[1] * MM) for p in ((root, top - 2.2), (tip - 1.0, top - 2.2), (tip, top - 1.2), (tip, top),
                                             (root, top))]


def _jaw_profile():
    """The clamping jaw under the +x lip, retracted, (x, z) in the bracket frame, counter-clockwise."""
    lip_under = -LIP_THICK / MM
    top = lip_under - JAW_GAP / MM
    tip = (LIP_EDGE + JAW_CLEAR) / MM
    back = 19.6  # clear of the skirt's inner chamfer (x 20.0 at the jaw's top), which it moves away from
    return [(p[0] * MM, p[1] * MM) for p in ((tip, top - 0.8), (tip + 0.8, top - 2.2), (back, top - 2.2),
                                             (back, top), (tip, top))]


def bracket_shapes() -> list[Shape]:
    w = BRACKET_WIDTH / 2
    out = [_prism("body", "bracket", None, _br(_BRACKET_OUTLINE), -w, w)]
    for mirror in (False, True):
        out += [_prism("body", "bracket", "polyamide", _br(h, mirror), -w, w, visible=False)
                for h in _BR_HALF_HULLS]
    # guide faces inside each skirt, 0.2 mm outside the lip edges (the seat is 0.2 wider than the lips each side) and
    # reaching just below the lips: they keep the bracket square on the rail. Without them a twist pushes the
    # skirts' 45 degree chamfers against the lips' top corners and the light bracket climbs them.
    for s in (1, -1):
        out.append(_box("body", "bracket", "polyamide", *sorted((s * BRACKET_SEAT_HALF, s * (BRACKET_SEAT_HALF + 1.2 * MM))),
                        -w + 0.5 * MM, w - 0.5 * MM, -LIP_THICK - 0.2 * MM, 0.0, visible=False))
    # the hook (-x) and the jaw (+x, its own link), as wide as the bracket less 0.5 mm each side
    out.append(_prism("body", "bracket", "polyamide", _hook_profile(), -w + 0.5 * MM, w - 0.5 * MM))
    out.append(_prism("jaw", "zinc", "steel", _jaw_profile(), -w + 1.0 * MM, w - 1.0 * MM))
    # the screw: a cheese head 0.2 mm above the recess floor (its bearing friction is in the screw
    # joint's friction, not a contact), its slot across the rail (along x) to start
    z0, z1 = RECESS_FLOOR + 0.2 * MM, RECESS_FLOOR + 0.2 * MM + SCREW_HEAD_H
    zs = z1 - SLOT_DEPTH
    out.append(_prism("screw", "zinc", "screw", _ngon(SCREW_HEAD_R, 16), z0, zs, plane="xy"))
    for s in (1, -1):  # the two half-heads either side of the slot
        a0, a1 = (0.0, math.pi) if s > 0 else (math.pi, 2 * math.pi)
        half = [p for p in _ngon(SCREW_HEAD_R, 8, a0=a0, a1=a1) if s * p[1] >= -1e-12]
        half = [(x, s * max(abs(y), SLOT_WIDTH / 2)) for x, y in half]
        # the round half-head is the look. It collides as the same half-disc up to SLOT_CHAMFER below the top, and
        # above that as a thin cap whose slot edge is chamfered 45 degrees (a slot's edges are rounded off on a real
        # screw; it lets a blade within +-0.5 mm drop in, for PhysX's ~0.3 mm contact error). Everything stays within
        # the head's 3.5 mm radius but the cap's corners (CAP_X, CAP_Y: 3.78 mm), inside the bracket's 4.0 mm
        # half-thickness, so the turning head never touches the neighbouring terminal block
        out.append(_prism("screw", "zinc", None, half, zs, z1, plane="xy"))
        w, c = SLOT_WIDTH / 2, SLOT_CHAMFER
        out.append(_prism("screw", "zinc", "screw", half, zs, z1 - c, plane="xy", visible=False))
        cap = [(w, z1 - c), (CAP_Y, z1 - c), (CAP_Y, z1), (w + c, z1)]
        cap = [(s * y, z) for y, z in cap]
        out.append(_prism("screw", "zinc", "screw", cap, -CAP_X, CAP_X, plane="yz", visible=False))
    out.append(Shape("screw", "zinc", None, (0.0, 0.0, (RECESS_FLOOR - 12 * MM)), radius=2.0 * MM, length=24 * MM))
    return out


def driver_shapes() -> list[Shape]:
    b = _box("driver", "steel", "steel", -BLADE_WIDTH / 2, BLADE_WIDTH / 2, -BLADE_THICK / 2, BLADE_THICK / 2,
             0.0, BLADE_LENGTH)
    # the blade widens from 0.8 mm to the shaft over 4 mm (a flattened shaft end, as on a real driver)
    taper = _prism("driver", "steel", "steel", [(-BLADE_WIDTH / 2, -BLADE_THICK / 2), (BLADE_WIDTH / 2, -BLADE_THICK / 2),
                                                (BLADE_WIDTH / 2, BLADE_THICK / 2), (-BLADE_WIDTH / 2, BLADE_THICK / 2)],
                   BLADE_LENGTH, BLADE_LENGTH + 4 * MM, plane="xy")
    shaft = _prism("driver", "steel", "steel", _ngon(SHAFT_R, 10), BLADE_LENGTH + 4 * MM, SHAFT_TOP, plane="xy")
    collar = _prism("driver", "black", "handle", _ngon(COLLAR_R, 16), SHAFT_TOP, COLLAR_TOP, plane="xy")
    handle = _prism("driver", "handle", "handle", _ngon(HANDLE_R, 16), COLLAR_TOP, HANDLE_TOP - 4 * MM, plane="xy")
    cap = _prism("driver", "black", "handle", _ngon(HANDLE_R - 2 * MM, 16), HANDLE_TOP - 4 * MM, HANDLE_TOP,
                 plane="xy")
    return [b, taper, shaft, collar, handle, cap]


SHAPES = {"rail": rail_shapes, "bracket": bracket_shapes, "driver": driver_shapes}
BODIES = {"rail": ("rail",), "bracket": ("body", "screw", "carrier", "jaw"), "driver": ("driver",)}
MASSES = {"rail": RAIL_MASS, "driver": DRIVER_MASS, "screw": SCREW_MASS, "jaw": JAW_MASS, "carrier": CARRIER_MASS,
          "body": BRACKET_MASS - SCREW_MASS - JAW_MASS - CARRIER_MASS}


@lru_cache(maxsize=None)
def bounds(kind: str) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """(low, high) corners of a part's geometry in its own frame."""
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for sh in SHAPES[kind]():
        for p in sh.points():
            for i in range(3):
                lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    return tuple(lo), tuple(hi)


@lru_cache(maxsize=None)
def local_center(kind: str) -> tuple[float, float, float]:
    """The centre of a part's bounding box in its own frame: ``ObjectState.pos`` is this point."""
    lo, hi = bounds(kind)
    return tuple((a + b) / 2 for a, b in zip(lo, hi, strict=True))


def jaw_direction() -> tuple[float, float, float]:
    """Unit vector of the jaw's travel in the bracket frame: inward (-x) and up."""
    return (-math.cos(JAW_ANGLE), 0.0, math.sin(JAW_ANGLE))


def _triangulate(poly) -> list[tuple[int, int, int]]:
    """Ear-clipping triangulation of a simple counter-clockwise polygon [(a, b), ...]."""
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


def is_convex(poly) -> bool:
    """True if the polygon (either winding) is convex."""
    n, sign = len(poly), 0
    for i in range(n):
        a, b, c = poly[i - 1], poly[i], poly[(i + 1) % n]
        cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cr) < 1e-14:
            continue
        s = 1 if cr > 0 else -1
        if sign and s != sign:
            return False
        sign = s
    return True


# -- task-file objects --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin (see the module docstring),
    ``size`` its bounding box, ``roll`` a turn about the world x axis applied before ``yaw``. Spawned by
    :func:`arena_object` once :func:`install` has run. ``screw_turns``: a bracket's screw starts this many turns in
    from fully backed out (0: jaw retracted)."""

    kind: str = "bracket"
    roll: float = 0.0
    pitch: float = 0.0
    screw_turns: float = 0.0

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name: str, kind: str, pos, yaw: float = 0.0, roll: float = 0.0, pitch: float = 0.0, fixed: bool = False,
         screw_turns: float = 0.0) -> Part:
    lo, hi = bounds(kind)
    mass = None if fixed else sum(MASSES[b] for b in BODIES[kind])
    color = {"rail": LOOKS["steel"][0], "bracket": LOOKS["bracket"][0], "driver": LOOKS["handle"][0]}[kind]
    size = tuple(b - a for a, b in zip(lo, hi, strict=True))
    return Part(name=name, pos=tuple(pos), size=size, mass=mass, color=color, yaw=yaw, kind=kind, roll=roll,
                pitch=pitch, screw_turns=screw_turns)


def part_quat_xyzw(spec: Part) -> tuple[float, float, float, float]:
    """The part's orientation: roll about x, then pitch about y, then yaw about z (all world axes)."""
    def q(axis, a):
        s = math.sin(a / 2)
        return (s * (axis == 0), s * (axis == 1), s * (axis == 2), math.cos(a / 2))

    def mul(p, r):  # p * r (apply r first)
        px, py, pz, pw = p
        rx, ry, rz, rw = r
        return (pw * rx + px * rw + py * rz - pz * ry, pw * ry - px * rz + py * rw + pz * rx,
                pw * rz + px * ry - py * rx + pz * rw, pw * rw - px * rx - py * ry - pz * rz)

    return mul(q(2, spec.yaw), mul(q(1, spec.pitch), q(0, spec.roll)))


@dataclass(frozen=True)
class BracketState(task_file.ObjectState):
    """A bracket's ``ObjectState`` plus its mechanism: how far the screw has been turned in (turns, from fully backed
    out), how far the jaw has moved along its line (m), and the jaw spring's compression (m) and force (N): the
    clamp."""

    screw_turns: float = 0.0
    jaw_travel: float = 0.0
    jaw_compression: float = 0.0
    clamp_force: float = 0.0


# -- USD (after the simulator has started) ------------------------------------------------------------------

_USD_DIR: Path | None = None
_JOINTED: dict[str, str] = {}  # articulated parts spawned so far: name -> kind


def _srgb_to_linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _apply(prim, schema: str, attrs: dict) -> None:
    """Apply a PhysX API schema by name and author its attributes (the PhysxSchema module isn't importable)."""
    from pxr import Sdf

    prim.AddAppliedSchema(schema)
    types = {bool: Sdf.ValueTypeNames.Bool, int: Sdf.ValueTypeNames.Int, float: Sdf.ValueTypeNames.Float,
             str: Sdf.ValueTypeNames.Token}
    for name, value in attrs.items():
        prim.CreateAttribute(name, types[type(value)]).Set(value)


def _prism_mesh(sh: Shape):
    """(points, face counts, face indices) of a prism: the profile at both ends of its span, caps and sides."""
    prof = list(sh.profile)
    if polygon_area(prof) < 0:
        prof = prof[::-1]
    n = len(prof)
    s0, s1 = sh.span
    pts = [sh.lift(p, s0) for p in prof] + [sh.lift(p, s1) for p in prof]
    # Winding only matters for looks, and the meshes are double-sided; colliders are convex hulls of the points.
    counts, indices = [], []
    for a, b, c in _triangulate(prof):
        counts += [3, 3]
        indices += [a, b, c, n + c, n + b, n + a]
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        indices += [i, n + i, n + j, j]
    return pts, counts, indices


def _quat_about_y(angle: float):
    from pxr import Gf

    return Gf.Quatf(math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)


def write_usd(kind: str, path: Path, fixed: bool = False) -> Path:
    """Write one part as a USD file: its bodies, colliders, looks, materials and (for a bracket) its joints."""
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

    jointed = kind == "bracket"
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        prim = UsdGeom.Xform.Define(stage, path_b).GetPrim() if jointed else root
        body_paths[body] = path_b
        if fixed:
            continue  # a fixed collider: no rigid body
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(MASSES[body])
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:solverPositionIterationCount": 32,
            "physxRigidBody:solverVelocityIterationCount": 4,
            "physxRigidBody:maxDepenetrationVelocity": 0.3,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        if sh.profile is not None:
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
        if sh.profile is not None:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI(prim).Bind(mats[sh.material], UsdShade.Tokens.strongerThanDescendants, "physics")
        _apply(prim, "PhysxCollisionAPI", {"physxCollision:contactOffset": CONTACT_OFFSET,
                                           "physxCollision:restOffset": 0.0})

    if jointed:
        UsdPhysics.ArticulationRootAPI.Apply(root)
        _apply(root, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        up = math.degrees
        rot_jaw = _quat_about_y(math.pi + JAW_ANGLE)  # joint x axis -> the jaw's line (inward and up)
        ident = Gf.Quatf(1, 0, 0, 0)
        turns_max = JAW_TRAVEL / JAW_LEAD
        joints = {  # name: (type, parent, child, axis, rot, lower, upper)
            "screw": ("rev", "body", "screw", "Z", ident, -up(2 * math.pi * turns_max), 0.0),
            "carrier": ("pri", "body", "carrier", "X", rot_jaw, 0.0, JAW_TRAVEL),
            "jaw": ("pri", "carrier", "jaw", "X", rot_jaw, -JAW_SPRING_TRAVEL, 0.0),
        }
        for name, (jt, parent, child, axis, rot, lo, hi) in joints.items():
            jp = f"/Part/joints/{name}"
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(stage, jp)
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr(axis)
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            joint.CreateLocalPos0Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLocalRot0Attr(rot)
            joint.CreateLocalRot1Attr(rot)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if jt == "rev" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(JAW_SPRING if name == "jaw" else 0.0)
            drive.CreateDampingAttr(20.0 if name == "jaw" else 0.0)
            drive.CreateTargetPositionAttr(0.0)
            _apply(joint.GetPrim(), "PhysxJointAPI",
                   {"physxJoint:armature": SCREW_ARMATURE if name == "screw" else LINK_ARMATURE})
        # the thread/wedge: screw angle (deg) + gearing * carrier travel (m) = 0, i.e. one turn in (-360 deg)
        # moves the jaw JAW_LEAD along its line
        screw = stage.GetPrimAtPath("/Part/joints/screw")
        _apply(screw, "PhysxMimicJointAPI:rotZ", {"physxMimicJoint:rotZ:gearing": float(360.0 / (JAW_LEAD / 1.0)),
                                                  "physxMimicJoint:rotZ:offset": 0.0})
        screw.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets([Sdf.Path("/Part/joints/carrier")])
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str, fixed: bool = False) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="end_bracket_parts_"))
    path = _USD_DIR / f"{kind}{'_fixed' if fixed else ''}.usda"
    if not path.exists():
        write_usd(kind, path, fixed)
    return str(path)


def arena_object(spec: Part):
    """The Arena object for a part: a bracket an articulation, a rail or driver with a mass a free rigid body,
    without one a fixed collider."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    turns = spec.screw_turns

    class _Bracket(Object):
        """A bracket: the jaw's spring and the screw's friction, written to PhysX by implicit actuators."""

        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "screw": ImplicitActuatorCfg(joint_names_expr=["screw"], stiffness=0.0, damping=0.0,
                                             armature=SCREW_ARMATURE, friction=FREE_TORQUE,
                                             # `friction` is only the static part (holding a still screw); without the
                                             # dynamic part a turning screw coasts freely once the blade lets go
                                             dynamic_friction=FREE_TORQUE),
                "carrier": ImplicitActuatorCfg(joint_names_expr=["carrier"], stiffness=0.0, damping=0.0,
                                               armature=LINK_ARMATURE),
                "jaw": ImplicitActuatorCfg(joint_names_expr=["jaw"], stiffness=JAW_SPRING, damping=20.0,
                                           armature=LINK_ARMATURE),
            }
            cfg.init_state.joint_pos = {"screw": -2 * math.pi * turns, "carrier": turns * JAW_LEAD, "jaw": 0.0}
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos), rotation_xyzw=part_quat_xyzw(spec))
    fixed = spec.mass is None
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind, fixed and spec.kind != "bracket"))
    if spec.kind == "bracket":  # the links overlap inside the housing (jaw in the skirt, screw in its bore)
        cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False,
                                                                         solver_position_iteration_count=64,
                                                                         solver_velocity_iteration_count=4)
    if spec.kind == "bracket":
        _JOINTED[spec.name] = spec.kind
        return _Bracket(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)
    return Object(name=spec.name, object_type=ObjectType.BASE if fixed else ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=pose)


# -- the hook into yam_arena.scene --------------------------------------------------------------------------

_MOTION: dict = {}  # name -> (control step, root position, rotation, velocities over the step that ended there)


def _rot(quat_xyzw):
    import numpy as np

    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _motion(base, name: str, pos, rot):
    """A jointed part's velocities (m/s, rad/s) from its root's pose change over the last control step, or
    (None, None) on an episode's first step (PhysX's velocities of a light articulation on stiff springs read high)."""
    import numpy as np

    counter = getattr(base, "common_step_counter", None)
    episode = getattr(base, "episode_length_buf", None)
    if counter is None or episode is None or int(episode[0]) == 0:
        _MOTION.pop(name, None)
        return None, None
    last = _MOTION.get(name)
    if last is not None and last[0] == counter:
        return last[3], last[4]
    if last is None or last[0] != counter - 1:
        _MOTION[name] = (counter, pos, rot, None, None)
        return None, None
    dt = float(base.step_dt)
    lin = (pos - last[1]) / dt
    d = rot @ last[2].T
    angle = math.acos(min(1.0, max(-1.0, (np.trace(d) - 1.0) / 2.0)))
    axis = np.array([d[2, 1] - d[1, 2], d[0, 2] - d[2, 0], d[1, 0] - d[0, 1]])
    n = np.linalg.norm(axis)
    ang = axis / n * angle / dt if n > 1e-12 else np.zeros(3)
    _MOTION[name] = (counter, pos, rot, lin, ang)
    return lin, ang


def mechanism(q, names) -> dict:
    """The bracket mechanism's state from its joint positions."""
    screw = float(q[names.index("screw")])
    carrier = float(q[names.index("carrier")])
    jaw = float(q[names.index("jaw")])
    comp = max(0.0, -jaw)
    force = JAW_SPRING * comp
    return {"screw_turns": -screw / (2 * math.pi), "jaw_travel": carrier + jaw, "jaw_compression": comp,
            "clamp_force": force}


def _bracket_state(base, name: str) -> BracketState:
    import numpy as np

    art = base.scene.articulations[name]
    data = art.data
    pose = data.root_pose_w.torch[0].detach().cpu().numpy().astype(np.float64)
    origin = base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)
    pos, quat = pose[:3] - origin, pose[3:]
    rot = _rot(quat)
    lo, hi = bounds("bracket")
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ rot.T + pos
    q = data.joint_pos.torch[0].detach().cpu().numpy().astype(np.float64)
    mech = mechanism(q, list(art.joint_names))
    lin_vel, ang_vel = _motion(base, name, pos, rot)
    if lin_vel is None:
        lin_vel = data.root_lin_vel_w.torch[0].detach().cpu().numpy().astype(np.float64)
        ang_vel = data.root_ang_vel_w.torch[0].detach().cpu().numpy().astype(np.float64)
    return BracketState(pos=rot @ np.array(local_center("bracket")) + pos, quat_xyzw=quat, lin_vel=lin_vel,
                        ang_vel=ang_vel, low=corners.min(axis=0), high=corners.max(axis=0), **mech)


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "eb_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_eb_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        # Arena's geometry queries don't cover articulations, so a bracket's state is built here.
        base = env.unwrapped
        jointed = {n for n in names if n in _JOINTED and n in base.scene.articulations}
        out = original_states(env, [n for n in names if n not in jointed])
        out.update({n: _bracket_state(base, n) for n in jointed})
        return {n: out[n] for n in names}

    task_object._spawns_eb_parts = True
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
