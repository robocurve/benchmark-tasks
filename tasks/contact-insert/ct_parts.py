"""The contact-insert task's own parts: a sealed 4-way connector housing with a sprung locking finger (lance) in each
cavity, a crimped size-16 socket contact on a short wire, a secondary lock (a wedgelock: a face plate with four wedge
blades, pushed into the housing's mating face) in a small holder, and a pinout card. All geometry is our own simplified design (sized like common sealed 4-way connectors with
size-16 contacts); no manufacturer CAD or product names are used.

yam_arena task files support single primitives and library assets only, so this module is the task's own minimal
mechanism (the approach of our earlier tasks, copied so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` that also names which part it is.
- :func:`install` wraps ``yam_arena.scene.task_object``, ``object_states`` and ``make_env`` when that module is
  imported.

Parts (mm in comments, metres in code). Every part is built in its own frame:

- housing (a free articulation): 24 (x) x 40 (y) x 24 (z), lying on the table; its frame's origin at the centre of
  its rear face (the wire side), y along the cavities toward the mating face, z up. Four square cavities (3.8 mm
  across, 10 mm apart) at (x, z) = (-5, 5) #1, (5, 5) #2, (5, -5) #3, (-5, -5) #4 (numbered as seen from the rear),
  each with a lead-in funnel at the rear (6 mm -> 3.8 mm over 1.5 mm) and a step 16 mm in (to 2.7 mm across, the
  contact's shoulder stops on it). In each cavity, on its outer side (+x for #2/#3, -x for #1/#4), a locking finger
  (prismatic joint along x, spring) reaches 0.6 mm into the cavity at 12.8-14.1 mm deep; a contact's shoulder pushes
  it aside (the click) and it springs back behind the shoulder: pulled back, the shoulder meets its flat face.
  Beside each column of cavities, on both sides, a channel (SLOT_X from the column's axis, the column's full height)
  runs from the mating face (y = H_L) back to the fingers: the secondary lock's wedge blades slide in along it from
  the mating face and end beside the fingers, where only a closed finger leaves room for them (as on a DEUTSCH DT
  plug, whose wedgelock goes into the mating face after the contacts and backs up their locking fingers).
- contact (a free articulation): a socket contact, crimp end at the frame's origin, pointing along -x: nose (r 1.2),
  shoulder (r 1.7, 2 mm long, 9 mm behind the nose's tip... see CONTACT_PROFILE), neck, crimp barrel (r 1.6, 6 mm);
  then the wire along +x: WIRE_LINKS links of WIRE_LINK, insulation r 1.3, joined by two bending joints and a
  twisting joint each (springs): pushed far behind the crimp, the wire bends sideways.
- lock (a free articulation): the secondary lock, a wedgelock: a 26 x 22 mm face plate (6 mm thick) with four wedge
  blades (0.8 mm thick, 19 mm wide, 22 mm long) along its -z, each on a stiff spring (prismatic along z): pressed
  onto a finger that a half-seated contact holds open, a blade gives; loaded LOCK_FORCE or more, it was forced (the
  housing damaged). In the housing its +z points along the cavities (+y), the plate on the mating face.
- holder (fixed): a block the secondary lock stands in, blades down in its pocket, the plate on its rim.
- card (fixed): the pinout card: the 4 cavities as seen from the rear, cavity 3 marked.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import inspect
import math
import sys
import tempfile
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

from yam_arena import task_file

MM = 1e-3

# -- housing ----------------------------------------------------------------------------------------------------------
H_W, H_L, H_H = 24 * MM, 40 * MM, 24 * MM  # x, y, z
H_SHELL = 2 * MM
PITCH = 10 * MM
CAVITIES = {1: (-5 * MM, 5 * MM), 2: (5 * MM, 5 * MM), 3: (5 * MM, -5 * MM), 4: (-5 * MM, -5 * MM)}  # (x, z)
TARGET_CAVITY = 3
BORE = 1.95 * MM  # half the cavity's width (a square hole: the round contact runs on its four walls; at 1.9 the
# finger's bevel pushed the contact aside into the far wall instead of opening, bench 9)
FUNNEL = 3.0 * MM  # the rear lead-in: half-width at the rear face ...
FUNNEL_L = 1.5 * MM  # ... down to BORE over this
Y_STEP = 22 * MM  # the step to the pin exit (the fingers 18 mm in: the crimp barrel is inside the cavity, guiding the
# contact, when the shoulder reaches them; 6 mm further out the contact cocked on the fingers, benches 26-27)
EXIT = 1.4 * MM  # the pin exit's half-width (the body r 1.0 passes with 0.4 mm play, the shoulder r 1.9 stops; at 1.15
# the body bound in the 17 mm exit like a drawer, bench 28)
STEP_L = 1.0 * MM
Y_LAT = (18.0 * MM, 19.8 * MM)  # (its flat face 0.26 mm short of a seated contact's shoulder: at 14.1 it rested on the
# shoulder's edge, 0.09 mm open, and the tug pulled the contact past it, bench 21)  # the locking finger's tip, along the cavity (its flat face at the larger y)
LAT_IN = 1.05 * MM  # the finger's inner face, from the cavity's axis, at rest (0.9 into the cavity: the shoulder lifts it
# 0.85 mm; at 0.45 the parts' ~0.3 mm solver slop let the shoulder slip past it closed, benches 12-15)
LAT_OUT = 2.3 * MM  # its outer face
LAT_HALF_Z = 0.8 * MM
LAT_UNDERCUT = 0.2 * MM  # its front face undercut (the inner edge this much further forward): pulled, the shoulder
# presses it shut (square, a tilted contact wedged it 0.5 mm open under ~12 N of tug, bench 23)
LAT_CHAMFER = 0.9 * MM  # its rear edge bevelled (0.9 mm deep over LAT_BEVEL_L along the cavity, 30 degrees: the
LAT_BEVEL_L = 1.56 * MM  # shoulder's ramp slides it out; at 45 degrees and 15 N/mm the contact cocked and jammed at ~6 N,
# bench 20)
LAT_TRAVEL = 1.0 * MM
LAT_K = 4.0e3  # N/m (the shoulder lifts it 0.85 mm: ~3.4 N on the finger; at 6 N/mm the robot's push through the
# gripped wire stalled just short of the click in some runs, demo 6)
LAT_D = 20.0
LAT_ARMATURE = 0.02  # kg (with LATCH_MASS: a 1 g finger with no armature jittered 0.8 mm, bench 10)
Y_SLOT = (17.8 * MM, 20.0 * MM)  # the fingers' window along the cavities (where the lock's blades end, beside them)
SLOT_X = (2.55 * MM, 3.65 * MM)  # the slot beside each column, from the column's axis outward

# -- contact and wire --------------------------------------------------------------------------------------------------
# (x from, x to, radius from, radius to) along the contact, crimp end at 0, nose toward -x
CONTACT_PROFILE = (
    (-21 * MM, -20 * MM, 0.7 * MM, 1.0 * MM),  # nose
    (-20 * MM, -12.5 * MM, 1.0 * MM, 1.0 * MM),  # socket body
    (-12.5 * MM, -11 * MM, 1.0 * MM, 1.9 * MM),  # shoulder ramp (31 degrees)
    (-11 * MM, -9 * MM, 1.9 * MM, 1.9 * MM),  # shoulder
    (-9 * MM, -6 * MM, 1.0 * MM, 1.0 * MM),  # neck (the finger drops in here)
    (-6 * MM, 0.0, 1.85 * MM, 1.85 * MM),  # crimp barrel (guides the contact in the cavity with the shoulder: at r 1.6
    # the contact pivoted on its shoulder, cocked and jammed on the finger, bench 19)
)
SHOULDER_FRONT_X = -11 * MM
SHOULDER_REAR_X = -9 * MM
NOSE_X = -21 * MM
SHOULDER_R = 1.9 * MM
Y_SEAT_SHOULDER = Y_STEP + (BORE - SHOULDER_R) / (BORE - EXIT) * STEP_L  # where the shoulder stops (22.09)
Y_SEATED_CRIMP = Y_SEAT_SHOULDER + SHOULDER_FRONT_X  # the crimp end's depth when seated (11.09)
WIRE_LINK = 10 * MM
WIRE_LINKS = 14
WIRE_R = 1.3 * MM  # 1.0 mm2 automotive wire, thin wall (Ø 2.6)
WIRE_EI = 8.0e-5  # N m^2 (our estimate for a stranded 1.0 mm2 PVC wire)
WIRE_GJ = 5.0e-5
WIRE_D = 2.0e-4
WIRE_BEND_LIMIT = 60.0  # deg per joint
CONTACT_MASS = 0.01  # kg (simulation; a real one ~1 g)
WIRE_MASS = 0.0015  # per 10 mm link (simulation; real 1.0 mm2 wire ~0.15 g; at 0.5 g a 15 N push blew the chain up,
GIMBAL_MASS = 0.0005  # bench 22)

# -- secondary lock ---------------------------------------------------------------------------------------------------
LOCK_BAR = (26 * MM, 22 * MM, 6 * MM)  # the wedgelock's face plate (x, y, z: z out of the mating face); 1 mm wider than
# the housing on each side, so that the fingers holding it by its side edges pass beside the housing
PRONG_X = (-8.1 * MM, -1.9 * MM, 1.9 * MM, 8.1 * MM)  # the four blades' centres (5 +- 3.1 mm: beside each finger)
LOCK_PRONG = (0.8 * MM, 16 * MM, 22 * MM)  # a blade: thick (x), wide across both rows' fingers (y), long (z): its tip
# 18 mm from the rear face, beside the fingers, with the plate on the mating face
CHANNEL_MOUTH = (3.0 * MM, 0.6 * MM)  # the channels' lead-in at the mating face: this long, each side this much wider
LOCK_K = 1.0e4  # N/m: a prong's give
LOCK_D = 5.0
LOCK_TRAVEL = 3.0 * MM
LOCK_FORCE = 20.0  # N: a prong loaded this hard: the lock forced over an open finger (housing damaged, latched)
LOCK_MASS, PRONG_MASS = 0.02, 0.002
Y_LOCK = H_L  # the lock's plate (its underside) on the mating face when in
HOUSING_MASS = 0.2  # kg (simulation; a real one ~10 g: heavier, the fingers and contacts solve steadily, bench 19)
LATCH_MASS = 0.02  # (simulation)
LINK_ARMATURE = 1.0e-4
HOLDER_SIZE = (24 * MM, 34 * MM, 26 * MM)  # (narrower than the plate across x: the tilted pads' lower ends pass beside
# it; 34 mm wide, they landed on its rim and the hand never closed on the plate, demo p1)
HOLDER_POCKET = (9.0 * MM, 10.0 * MM, 24 * MM)  # half x, half y, depth: the blades stand in it, the plate on the rim
REST_H = 20 * MM  # the lead's rest: two blocks this tall along the wire (frame: the contact's crimp end above it, x along
REST_A = (-24 * MM, -2 * MM)  # the wire): the contact lies on the first, the wire on the second, and over the gap
REST_B = (36 * MM, 80 * MM)  # between them (right behind the crimp) a gripper's fingers close round the wire
REST_W = 20 * MM  # the prongs (19 mm) stand in a 20 mm deep slot
CARD_SIZE = (50 * MM, 36 * MM, 1 * MM)

# -- scoring and the tug ----------------------------------------------------------------------------------------------
IN_DEPTH = 5 * MM  # the contact counts as in a cavity with its nose this far in, on the cavity's axis (radial
IN_RADIAL = 1.0 * MM  # within this) and pointing in (within IN_ANGLE)
IN_ANGLE = 20.0
SEATED_TOL = 0.3 * MM  # seated: the crimp end within this of its seated depth
LATCH_CLOSED = 0.1 * MM  # the finger at most this far open
LOCK_IN_TOL = 0.3 * MM  # the secondary lock fully in: its plate within this of the mating face ...
LOCK_Z_PLAY = 2.2 * MM  # ... and its centre within this up or down (the blades' play in the channels: 16 mm blades in the
# 20 mm columns; released, the lock settled 0.7 mm low, demo p10)
SNAP_FORCE = 3.0  # N: once home, the wedgelock is held onto the mating face with this (its snap; see update)
LOCK_FORCE_S = 0.1  # (0.25: the contact blew out of a forced cavity first, bench 33)
FORCE_DEPTH = Y_LAT[1] + LOCK_PRONG[2] - H_L + 0.2 * MM  # the plate this near the mating face: the blades' tips reach the
# fingers (2.0 mm). Pressed this far
FORCE_OPEN = 0.3 * MM  # while a finger is this far open, or a contact half in (its shoulder at the fingers, not past):
# forced (latched; the prongs pushed an open finger shut, and the contact back, rather than stopping, bench 32)
HELD_PLAY = 0.1 * MM  # (simulation) a cavity's fingers, once snapped in behind a seated shoulder, are held to this opening
# by the task (their joint limits): the solver's ~0.3 mm slop between millimetre parts let a 30 N tug wedge them open
# (benches 21-30); on a real connector the undercut fingers hold
TUG_FORCE = 30.0  # N, the tug of a real trial (a hand holding the housing, the other tugging the wire)
TUG_FORCE_SIM = 8.0  # N, the tug applied in simulation: it pulls out a contact the fingers don't hold (only the seal's
# friction, a few N); a locked contact counts as holding 30 N (the lance holds >100 N on real connectors; between the
# simulated millimetre parts the solver let even held fingers be dragged open by 30 N, benches 21-31)
TUG_RAMP_S = 1.0
TUG_S = 1.5
TUG_MAX_MOVE = 1.0 * MM  # (the fingers sit 0.29 mm behind a seated shoulder; with the solver's slop a locked contact
# moved 0.51 mm, bench 29)
TUG_BODY = "contact"  # the tug pulls on the contact (along the wire; through the light wire links it blew up, bench 22)
HANDS_OFF_WIRE = 25 * MM  # no fingertip this near the contact or the first WIRE_NEAR of wire (for the tug) ...
WIRE_NEAR = 60 * MM
HANDS_OFF_S = 1.0
HANDS_OFF_ALL = 20 * MM  # ... and for the end: none this near the housing, the contact, the wire or the lock
TUG_INVALID_MOVE = 0.5 * MM  # (a passed test is redone if the contact moves more than this since: at 0.3 mm the
# contact settling back onto its step when the other hand took the housing reset it, demo p9)
ROBOT_TUG_NEAR = 8 * MM  # the robot's own tug: every fingertip of one hand this near the crimp barrel or the first
# WIRE_NEAR of wire (the hand closed on it) ...
ROBOT_TUG_PULL = 0.15 * MM  # ... while the clicked contact in cavity 3 is drawn back at least this far from its seated
# depth (its shoulder off the step, onto the fingers 0.26 mm behind: the wire pulled taut, the housing held) ...
ROBOT_TUG_S = 0.5  # ... for this long
MARK_BEHIND = 15 * MM  # the wire's reference mark (a 2 mm white band, its centre this far behind the crimp end): seated,
# it lies MARK_BEHIND - Y_SEATED_CRIMP (3.9 mm) outside the rear face, measurable with a ruler
MARK_TOL = 1.0 * MM  # the mark within this of its seated position (with the retention test: in place of a concealed
# 0.3 mm depth check)

# -- materials and looks ---------------------------------------------------------------------------------------------
MATERIALS = {  # (static friction, dynamic friction, combine mode)
    "pa": (0.25, 0.20, "average"),  # glass-filled nylon (the cavities, the fingers, the lock)
    "shell": (0.70, 0.60, "max"),  # the housing's textured outside (gripped; at 0.25 the hand's grip slid 10 mm, demo 1)
    "seal": (0.60, 0.50, "average"),  # the rear funnel's rubber seal
    "brass": (0.15, 0.12, "average"),  # the contact (plated, lubricated)
    "pvc": (0.50, 0.45, "average"),
    "steel": (0.40, 0.35, "average"),
}
LOOKS = {  # sRGB, roughness, metallic
    "housing": ((0.18, 0.19, 0.20), 0.6, 0.0),
    "seal": ((0.10, 0.10, 0.11), 0.8, 0.0),
    "latch": ((0.22, 0.23, 0.24), 0.6, 0.0),
    "gold": ((0.85, 0.68, 0.30), 0.3, 0.9),
    "tin": ((0.75, 0.76, 0.78), 0.35, 0.8),
    "wire": ((0.80, 0.12, 0.10), 0.5, 0.0),
    "lock": ((0.95, 0.50, 0.08), 0.5, 0.0),
    "white": ((0.93, 0.93, 0.90), 0.8, 0.0),
    "mark": ((0.85, 0.10, 0.10), 0.6, 0.0),
    "grey": ((0.45, 0.46, 0.48), 0.7, 0.0),
    "steel": ((0.72, 0.74, 0.76), 0.35, 0.8),
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
}
CONTACT_OFFSET = 0.5 * MM  # (0.1: a contact at 130 mm/s jumped through the 1.3 mm finger, bench 5)


# -- shapes -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One piece of a part, in the PART's frame: a box (``center``, ``size``), a capsule (``center``, ``radius``,
    ``length`` along ``axis``) or a convex ``hull`` of points (a collider; drawn only if ``visible``)."""

    body: str
    look: str
    material: str | None  # None: visual only
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float] | None = None
    radius: float = 0.0
    length: float = 0.0
    axis: str = "X"
    capsule: bool = False
    visible: bool = True
    hull: tuple | None = None

    def points(self):
        if self.hull is not None:
            return list(self.hull)
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2 + (self.radius if self.capsule else 0.0)
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _hull(body, look, material, pts, visible=True) -> Shape:
    return Shape(body, look, material, hull=tuple(tuple(float(v) for v in p) for p in pts), visible=visible)


def _frustum_x(body, look, material, x0, x1, r0, r1, n=16, visible=True) -> Shape:
    """A round section along x (r0 at x0, r1 at x1) as an n-gon hull."""
    pts = []
    for x, r in ((x0, r0), (x1, r1)):
        for k in range(n):
            a = 2 * math.pi * (k + 0.5) / n
            pts.append((x, r * math.cos(a), r * math.sin(a)))
    return _hull(body, look, material, pts, visible)


# -- the housing ------------------------------------------------------------------------------------------------------


def _cell_walls(xc, zc, y0, y1, h0, h1, look="housing", material="pa", window=False, channel=False):
    """The four walls of one cavity's 10 x 10 mm cell between y0 and y1, the square hole's half-width going from h0
    to h1 (a funnel if they differ). ``window``: the fingers' section: on both sides the wall is open between BORE
    and SLOT_X[1] at the fingers' height (their window and the lock's slot behind it), kept above and below them
    up to SLOT_X[0] (open full height, the shoulder slid sideways into it and jammed, bench 17). ``channel``: the x
    walls leave SLOT_X[0]-SLOT_X[1] open over the cell's full height (the lock's blades slide along it)."""
    c = PITCH / 2
    out = []
    if abs(h0 - h1) < 1e-12:  # a straight section: boxes
        h = h0
        for side in (-1.0, 1.0):
            a = SLOT_X[1] if (window or channel) else h
            out.append(_box("base", look, material, *sorted((xc + side * a, xc + side * c)), y0, y1, zc - c, zc + c))
            if channel:
                out.append(_box("base", look, material, *sorted((xc + side * h, xc + side * SLOT_X[0])), y0, y1,
                                zc - c, zc + c))
            out.append(_box("base", look, material, xc - h, xc + h, y0, y1, *sorted((zc + side * h, zc + side * c))))
            if window:
                xa, xb = sorted((xc + side * h, xc + side * SLOT_X[0]))
                gap = LAT_HALF_Z + 0.1 * MM
                out.append(_box("base", look, material, xa, xb, y0, y1, zc - c, zc - gap))
                out.append(_box("base", look, material, xa, xb, y0, y1, zc + gap, zc + c))
        return out
    for side in (-1.0, 1.0):  # x walls (full cell height)
        p = []
        outer = SLOT_X[0] if channel else c
        for y, h in ((y0, h0), (y1, h1)):
            for z in (zc - c, zc + c):
                p += [(xc + side * h, y, z), (xc + side * outer, y, z)]
        out.append(_hull("base", look, material, p))
        if channel:
            out.append(_box("base", look, material, *sorted((xc + side * SLOT_X[1], xc + side * c)), y0, y1,
                            zc - c, zc + c))
    for side in (-1.0, 1.0):  # z walls (between the x walls)
        p = []
        for y, h in ((y0, h0), (y1, h1)):
            for x in (xc - h, xc + h):
                p += [(x, y, zc + side * h), (x, y, zc + side * c)]
        out.append(_hull("base", look, material, p))
    return out


def _channel_mouth(xc, zc, y0, y1, look="housing", material="pa"):
    """The cell's walls at the mating face, like ``_cell_walls(..., EXIT, EXIT, channel=True)`` but with each blade
    channel widening by CHANNEL_MOUTH[1] a side toward the face (a lead-in for the wedgelock's blades)."""
    c = PITCH / 2
    w = CHANNEL_MOUTH[1]
    out = []
    for side in (-1.0, 1.0):
        out.append(_box("base", look, material, xc - EXIT, xc + EXIT, y0, y1, *sorted((zc + side * EXIT, zc + side * c))))
        inner, outer = [], []
        for y, d in ((y0, 0.0), (y1, w)):
            for z in (zc - c, zc + c):
                inner += [(xc + side * EXIT, y, z), (xc + side * (SLOT_X[0] - d), y, z)]
                outer += [(xc + side * (SLOT_X[1] + d), y, z), (xc + side * c, y, z)]
        out += [_hull("base", look, material, inner), _hull("base", look, material, outer)]
    return out


def slot_ranges():
    """The lock's four prong slots across the housing (x from, x to): beside each column of cavities, both sides."""
    out = []
    for xc in (-5 * MM, 5 * MM):
        for d in (-1.0, 1.0):
            out.append(tuple(sorted((xc + d * SLOT_X[0], xc + d * SLOT_X[1]))))
    return sorted(out)


def housing_shapes() -> list[Shape]:
    out = []
    hx, hz = H_W / 2, H_H / 2
    inner = PITCH  # the cells span +-10 mm
    # shell: sides, bottom, top
    out.append(_box("base", "housing", "shell", -hx, -inner, 0.0, H_L, -hz, hz))
    out.append(_box("base", "housing", "shell", inner, hx, 0.0, H_L, -hz, hz))
    out.append(_box("base", "housing", "shell", -inner, inner, 0.0, H_L, -hz, -inner))
    out.append(_box("base", "housing", "pa", -inner, inner, 0.0, H_L, inner, hz))
    for c, (xc, zc) in CAVITIES.items():
        out += _cell_walls(xc, zc, 0.0, FUNNEL_L, FUNNEL, BORE, look="seal", material="seal")
        out += _cell_walls(xc, zc, FUNNEL_L, Y_SLOT[0], BORE, BORE)
        out += _cell_walls(xc, zc, Y_SLOT[0], Y_SLOT[1], BORE, BORE, window=True)
        out += _cell_walls(xc, zc, Y_SLOT[1], Y_STEP, BORE, BORE, channel=True)  # the blades' channels, from the
        out += _cell_walls(xc, zc, Y_STEP, Y_STEP + STEP_L, BORE, EXIT, channel=True)  # mating face back to the
        out += _cell_walls(xc, zc, Y_STEP + STEP_L, H_L - CHANNEL_MOUTH[0], EXIT, EXIT, channel=True)  # fingers' window
        out += _channel_mouth(xc, zc, H_L - CHANNEL_MOUTH[0], H_L)
        # the cavity's number, moulded on the rear face (dots: 1-4)
        sx, sz = (1.0 if xc > 0 else -1.0), (1.0 if zc > 0 else -1.0)
        for k in range(c):
            out.append(_box("base", "white", None, xc - sx * 3.6 * MM - 0.35 * MM, xc - sx * 3.6 * MM + 0.35 * MM,
                            -0.1 * MM, 0.0, zc + sz * (3.9 - 1.1 * k) * MM - 0.35 * MM,
                            zc + sz * (3.9 - 1.1 * k) * MM + 0.35 * MM))
        for side in FINGERS:
            out += latch_shapes(c, side)
    return out


FINGERS = ("o", "i")  # each cavity's two locking fingers: on its outer side (away from the housing's middle), inner


def finger_dir(c: int, side: str) -> float:
    """+1 if the finger opens toward +x, -1 toward -x."""
    sx = 1.0 if CAVITIES[c][0] > 0 else -1.0
    return sx if side == "o" else -sx


def latch_frame(c: int, side: str):
    xc, zc = CAVITIES[c]
    return (xc + finger_dir(c, side) * (LAT_IN + LAT_OUT) / 2, (Y_LAT[0] + Y_LAT[1]) / 2, zc)


def latch_shapes(c: int, side: str) -> list[Shape]:
    """A locking finger of cavity ``c`` (housing frame): its tip, bevelled at the rear, undercut at the front."""
    xc, zc = CAVITIES[c]
    d = finger_dir(c, side)
    y0, y1 = Y_LAT
    pts = []
    for z in (zc - LAT_HALF_Z, zc + LAT_HALF_Z):
        pts += [(xc + d * (LAT_IN + LAT_CHAMFER), y0, z), (xc + d * LAT_IN, y0 + LAT_BEVEL_L, z),
                (xc + d * LAT_IN, y1, z), (xc + d * LAT_OUT, y1 - LAT_UNDERCUT, z), (xc + d * LAT_OUT, y0, z)]
    return [_hull(f"latch{c}{side}", "latch", "pa", pts)]


# -- the contact and its wire ------------------------------------------------------------------------------------------


def contact_bodies():
    out = [("contact", (0.0, 0.0, 0.0))]
    for i in range(WIRE_LINKS):
        x = i * WIRE_LINK
        out += [(f"ga{i}", (x, 0.0, 0.0)), (f"gb{i}", (x, 0.0, 0.0)), (f"w{i}", (x, 0.0, 0.0))]
    return out


def contact_shapes() -> list[Shape]:
    out = []
    for x0, x1, r0, r1 in CONTACT_PROFILE:
        look = "tin" if x0 >= -6 * MM else "gold"
        out.append(_frustum_x("contact", look, "brass", x0, x1, r0, r1))
    for i in range(WIRE_LINKS):
        out.append(Shape(f"w{i}", "wire", "pvc", ((i + 0.5) * WIRE_LINK, 0.0, 0.0), radius=WIRE_R,
                         length=WIRE_LINK, axis="X", capsule=True))
    i = int(MARK_BEHIND // WIRE_LINK)  # the reference mark (visual only), on the link it lies on
    out.append(Shape(f"w{i}", "white", None, (MARK_BEHIND, 0.0, 0.0), radius=WIRE_R + 0.05 * MM, length=2 * MM,
                     axis="X"))
    return out


def contact_joints():
    out, parent = {}, "contact"
    k_b = math.radians(1.0) * WIRE_EI / WIRE_LINK
    k_t = math.radians(1.0) * WIRE_GJ / WIRE_LINK
    for i in range(WIRE_LINKS):
        lim = WIRE_BEND_LIMIT
        out[f"bz{i}"] = ("rev", parent, f"ga{i}", "Z", -lim, lim, k_b, WIRE_D, LINK_ARMATURE)
        out[f"by{i}"] = ("rev", f"ga{i}", f"gb{i}", "Y", -lim, lim, k_b, WIRE_D, LINK_ARMATURE)
        out[f"tw{i}"] = ("rev", f"gb{i}", f"w{i}", "X", -720.0, 720.0, k_t, WIRE_D, LINK_ARMATURE)
        parent = f"w{i}"
    return out


def contact_masses():
    out = {"contact": (CONTACT_MASS, (-10 * MM, 0.0, 0.0), (1e-7, 5e-7, 5e-7))}
    for i in range(WIRE_LINKS):
        m, r = WIRE_MASS, WIRE_R
        i_t = m * (WIRE_LINK ** 2 / 12 + r ** 2 / 4)
        out[f"w{i}"] = (m, ((i + 0.5) * WIRE_LINK, 0.0, 0.0), (m * r ** 2 / 2, i_t, i_t))
        out[f"ga{i}"] = (GIMBAL_MASS, (i * WIRE_LINK, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
        out[f"gb{i}"] = (GIMBAL_MASS, (i * WIRE_LINK, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
    return out


# -- the secondary lock, its holder, the card -------------------------------------------------------------------------


def lock_bodies():
    return [("bar", (0.0, 0.0, 0.0))] + [(f"p{k}", (x, 0.0, 0.0)) for k, x in enumerate(PRONG_X)]


def lock_shapes() -> list[Shape]:
    """Frame: the plate's underside centre (in the housing: on the mating face, its +z along the cavities); the
    blades along -z."""
    bx, by, bz = LOCK_BAR
    px, py, pz = LOCK_PRONG
    out = [_box("bar", "lock", "shell", -bx / 2, bx / 2, -by / 2, by / 2, 0.0, bz)]  # (textured: gripped by its edges)
    for k, x in enumerate(PRONG_X):
        out.append(_box(f"p{k}", "lock", "pa", x - px / 2, x + px / 2, -py / 2, py / 2, -pz, -0.2 * MM))
    return out


def lock_joints():
    return {f"gp{k}": ("pri", "bar", f"p{k}", "Z", 0.0, LOCK_TRAVEL, LOCK_K, LOCK_D, LINK_ARMATURE)
            for k in range(len(PRONG_X))}


def lock_masses():
    px, py, pz = LOCK_PRONG
    out = {"bar": (LOCK_MASS, (0.0, 0.0, LOCK_BAR[2] / 2), (2e-6, 2e-6, 2e-6))}
    for k, x in enumerate(PRONG_X):
        out[f"p{k}"] = (PRONG_MASS, (x, 0.0, -pz / 2), (1e-7, 1e-7, 1e-8))
    return out


def holder_shapes() -> list[Shape]:
    """A block (origin: the centre of its top face) with a pocket the lock's blades stand in (the plate on its rim)."""
    hx, hy, hz = HOLDER_SIZE[0] / 2, HOLDER_SIZE[1] / 2, HOLDER_SIZE[2]
    px, py, pd = HOLDER_POCKET
    out = [_box("holder", "grey", "steel", -hx, hx, -hy, -py, -hz, 0.0),
           _box("holder", "grey", "steel", -hx, hx, py, hy, -hz, 0.0),
           _box("holder", "grey", "steel", -hx, -px, -py, py, -hz, 0.0),
           _box("holder", "grey", "steel", px, hx, -py, py, -hz, 0.0),
           _box("holder", "grey", "steel", -px, px, -py, py, -hz, -pd)]
    return out


def rest_shapes() -> list[Shape]:
    """The lead's rest (origin: under the crimp end on the blocks' top, x along the wire): two grey blocks."""
    hw = REST_W / 2
    return [_box("rest", "grey", "steel", *REST_A, -hw, hw, -REST_H, 0.0),
            _box("rest", "grey", "steel", *REST_B, -hw, hw, -REST_H, 0.0)]


def card_shapes() -> list[Shape]:
    """The pinout card (origin: its underside's centre): white, the four cavities as seen from the rear, #3 red."""
    cx, cy, cz = CARD_SIZE[0] / 2, CARD_SIZE[1] / 2, CARD_SIZE[2]
    out = [_box("card", "white", "pa", -cx, cx, -cy, cy, 0.0, cz)]
    for c, (xc, zc) in CAVITIES.items():
        x, y = 1.6 * xc, 1.6 * zc  # (the card lies flat: the housing's rear view, x right, z away from the arms)
        look = "mark" if c == TARGET_CAVITY else "grey"
        s = 3.5 * MM if c == TARGET_CAVITY else 2.5 * MM
        out.append(_box("card", look, None, x - s, x + s, y - s, y + s, cz, cz + 0.2 * MM))
    return out


# -- bookkeeping -------------------------------------------------------------------------------------------------------

HOUSING_BODIES = ("base",) + tuple(f"latch{c}{s}" for c in CAVITIES for s in FINGERS)
SHAPES = {"housing": housing_shapes, "contact": contact_shapes, "lock": lock_shapes, "holder": holder_shapes,
          "card": card_shapes, "rest": rest_shapes}
BODIES = {"housing": HOUSING_BODIES, "contact": tuple(b for b, _ in contact_bodies()),
          "lock": tuple(b for b, _ in lock_bodies()), "holder": ("holder",), "card": ("card",), "rest": ("rest",)}
ARTICULATED = ("housing", "contact", "lock")
FIXED = ("holder", "card", "rest")


def body_frames(kind: str) -> dict:
    if kind == "housing":
        return {"base": (0.0, 0.0, 0.0), **{f"latch{c}{s}": latch_frame(c, s) for c in CAVITIES for s in FINGERS}}
    if kind == "contact":
        return dict(contact_bodies())
    if kind == "lock":
        return dict(lock_bodies())
    return {}


def part_joints(kind: str) -> dict:
    if kind == "housing":
        out = {}
        for c in CAVITIES:
            for s in FINGERS:
                lo, hi = (0.0, LAT_TRAVEL) if finger_dir(c, s) > 0 else (-LAT_TRAVEL, 0.0)
                out[f"lat{c}{s}"] = ("pri", "base", f"latch{c}{s}", "X", lo, hi, LAT_K, LAT_D, LAT_ARMATURE)
        return out
    if kind == "contact":
        return contact_joints()
    if kind == "lock":
        return lock_joints()
    return {}


def part_masses(kind: str) -> dict:
    if kind == "housing":
        i_h = HOUSING_MASS * (H_L ** 2 + H_W ** 2) / 12
        out = {"base": (HOUSING_MASS, (0.0, H_L / 2, 0.0), (i_h, HOUSING_MASS * (H_W ** 2 + H_H ** 2) / 12, i_h))}
        for c in CAVITIES:
            for s in FINGERS:
                out[f"latch{c}{s}"] = (LATCH_MASS, latch_frame(c, s), (LATCH_MASS * 1e-5,) * 3)
        return out
    if kind == "contact":
        return contact_masses()
    if kind == "lock":
        return lock_masses()
    return {}


@lru_cache(maxsize=None)
def bounds(kind: str):
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for sh in SHAPES[kind]():
        for p in sh.points():
            for i in range(3):
                lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    return tuple(lo), tuple(hi)


# -- task-file objects ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin, ``yaw`` its turn about z."""

    kind: str = "card"

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name, kind, pos, yaw=0.0) -> Part:
    lo, hi = bounds(kind)
    size = tuple(max(b - a, 1e-3) for a, b in zip(lo, hi, strict=True))
    color = {"housing": LOOKS["housing"][0], "contact": LOOKS["wire"][0], "lock": LOOKS["lock"][0],
             "holder": LOOKS["grey"][0], "card": LOOKS["white"][0], "rest": LOOKS["grey"][0]}[kind]
    mass = None if kind in FIXED else 0.05
    return Part(name=name, pos=tuple(pos), size=size, mass=mass, color=color, yaw=yaw, kind=kind)


@dataclass(frozen=True)
class HousingState(task_file.ObjectState):
    """The housing and what is in it: which cavity holds the contact (0: none), how deep (the crimp end's depth, m),
    whether it is seated, its finger closed (locked), the secondary lock fully in, the latches and the tug."""

    cavity: int = 0
    crimp_depth: float = -1.0
    nose_depth: float = -1.0
    seated: bool = False
    latch_open: tuple = (0.0, 0.0, 0.0, 0.0)  # m, per cavity 1-4
    locked: bool = False
    lock_in: bool = False
    lock_depth: float = 1.0  # the lock bar's underside above the housing's top (m)
    lock_force: float = 0.0  # the larger prong's load (N)
    forced: bool = False  # latched: the lock forced over an open finger
    wrong_locked: bool = False  # latched: the contact locked in a wrong cavity
    tug: str = "none"  # none | running | passed | failed
    tug_moved: float = 0.0
    hands_off_steps: int = 0
    hand_dist: float = 1.0
    mark_ok: bool = False  # the wire's reference mark within MARK_TOL of its seated position (a contact in a cavity)
    robot_tug: bool = False  # latched: the robot's own tug on the clicked contact in cavity 3 (ROBOT_TUG_*)


# -- USD (after the simulator has started) ---------------------------------------------------------------------------

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


def _hull_faces(pts):
    import numpy as np

    p = np.asarray(pts, float)
    n = len(p)
    c = p.mean(axis=0)
    faces = []
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                if np.linalg.norm(nrm) < 1e-14:
                    continue
                d = (p - p[i]) @ nrm
                if np.all(d <= 1e-12):
                    faces.append((i, j, k) if (c - p[i]) @ nrm < 0 else (i, k, j))
                elif np.all(d >= -1e-12):
                    faces.append((i, k, j) if (c - p[i]) @ nrm > 0 else (i, j, k))
    return faces


def write_usd(kind: str, path: Path) -> Path:
    """One part as a USD file: bodies (frames translated, not turned), colliders, looks, materials, joints."""
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

    jointed = kind in ARTICULATED
    frames = body_frames(kind)
    masses = part_masses(kind)
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        xf = UsdGeom.Xform.Define(stage, path_b) if jointed else UsdGeom.Xform(root)
        if jointed and any(frames.get(body, (0.0, 0.0, 0.0))):
            xf.AddTranslateOp().Set(Gf.Vec3d(*frames[body]))
        body_paths[body] = path_b
        if kind in FIXED:
            continue
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        m, com, inertia = masses[body]
        off = frames.get(body, (0.0, 0.0, 0.0))
        mass_api.CreateMassAttr(m)
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*(c - o for c, o in zip(com, off, strict=True))))
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:solverPositionIterationCount": 64,
            "physxRigidBody:solverVelocityIterationCount": 4,
            "physxRigidBody:maxDepenetrationVelocity": 0.2,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        off = frames.get(sh.body, (0.0, 0.0, 0.0))

        def loc(p, off=off):
            return tuple(a - b for a, b in zip(p, off, strict=True))

        if sh.hull is not None:
            pts = [loc(p) for p in sh.hull]
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/hull_{i}")
            faces = _hull_faces(pts)
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray([v for f in faces for v in f]))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        elif sh.size is not None:
            geom = UsdGeom.Cube.Define(stage, f"{parent}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
            geom.AddScaleOp().Set(Gf.Vec3f(*sh.size))
        else:
            geom = (UsdGeom.Capsule if sh.capsule else UsdGeom.Cylinder).Define(stage, f"{parent}/round_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.CreateHeightAttr(sh.length)
            geom.CreateAxisAttr(sh.axis)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
        prim = geom.GetPrim()
        if not sh.visible:
            UsdGeom.Imageable(prim).MakeInvisible()
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(looks[sh.look])
        if sh.material is None:
            continue
        UsdPhysics.CollisionAPI.Apply(prim)
        if sh.hull is not None:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI(prim).Bind(mats[sh.material], UsdShade.Tokens.strongerThanDescendants, "physics")
        _apply(prim, "PhysxCollisionAPI", {"physxCollision:contactOffset": CONTACT_OFFSET,
                                           "physxCollision:restOffset": 0.0})

    if jointed:
        root_body = BODIES[kind][0]
        rb = stage.GetPrimAtPath(body_paths[root_body])
        UsdPhysics.ArticulationRootAPI.Apply(rb)
        _apply(rb, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        for name, (jt, parent, child, ax, lo, hi, k, d, arm) in part_joints(kind).items():
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(
                stage, f"/Part/joints/{name}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr(ax)
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            fp, fc = frames.get(parent, (0.0, 0.0, 0.0)), frames.get(child, (0.0, 0.0, 0.0))
            joint.CreateLocalPos0Attr(Gf.Vec3f(*(c - p for c, p in zip(fc, fp, strict=True))))
            joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if jt == "rev" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k)
            drive.CreateDampingAttr(d)
            drive.CreateTargetPositionAttr(0.0)
            _apply(joint.GetPrim(), "PhysxJointAPI", {"physxJoint:armature": arm})
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="contact_insert_parts_"))
    path = _USD_DIR / f"{kind}.usda"
    if not path.exists():
        write_usd(kind, path)
    return str(path)


_NAMES: dict = {}  # kind -> object name
_POSES: dict = {}  # kind -> (x, y, z, yaw) of the part's origin as spawned (env frame)


def arena_object(spec: Part):
    """The Arena object for a part: holder and card fixed, housing, contact and lock free articulations."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    actuators = {
        "housing": lambda: {"lat": ImplicitActuatorCfg(joint_names_expr=["lat[1-4][oi]"], stiffness=LAT_K, damping=LAT_D,
                                                       armature=LAT_ARMATURE)},
        "contact": lambda: {
            "bend": ImplicitActuatorCfg(joint_names_expr=["b[yz][0-9]+"], stiffness=WIRE_EI / WIRE_LINK,
                                        damping=WIRE_D, armature=LINK_ARMATURE),
            "twist": ImplicitActuatorCfg(joint_names_expr=["tw[0-9]+"], stiffness=WIRE_GJ / WIRE_LINK,
                                         damping=WIRE_D, armature=LINK_ARMATURE)},
        "lock": lambda: {"give": ImplicitActuatorCfg(joint_names_expr=["gp[0-3]"], stiffness=LOCK_K, damping=LOCK_D,
                                                     armature=LINK_ARMATURE)},
    }

    class _Jointed(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = actuators[spec.kind]()
            cfg.init_state.joint_pos = {".*": 0.0}
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos),
                rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)))
    _NAMES[spec.kind] = spec.name
    _POSES[spec.kind] = (*spec.pos, spec.yaw)
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind))
    if spec.kind in FIXED:
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=4,
        fix_root_link=False)
    return _Jointed(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)


# -- simulation state helpers ----------------------------------------------------------------------------------------


def _rot(quat_xyzw):
    import numpy as np

    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _quat_xyzw(m):
    import numpy as np

    t = m[0][0] + m[1][1] + m[2][2]
    if t > 0:
        s4 = math.sqrt(t + 1.0) * 2
        w, x, y, z = 0.25 * s4, (m[2][1] - m[1][2]) / s4, (m[0][2] - m[2][0]) / s4, (m[1][0] - m[0][1]) / s4
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s4 = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        w, x, y, z = (m[2][1] - m[1][2]) / s4, 0.25 * s4, (m[0][1] + m[1][0]) / s4, (m[0][2] + m[2][0]) / s4
    elif m[1][1] > m[2][2]:
        s4 = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        w, x, y, z = (m[0][2] - m[2][0]) / s4, (m[0][1] + m[1][0]) / s4, 0.25 * s4, (m[1][2] + m[2][1]) / s4
    else:
        s4 = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
        w, x, y, z = (m[1][0] - m[0][1]) / s4, (m[0][2] + m[2][0]) / s4, (m[1][2] + m[2][1]) / s4, 0.25 * s4
    return np.array([x, y, z, w])


def _t(x):
    return x.torch if hasattr(x, "torch") else x


def _np(x):
    import numpy as np

    return _t(x)[0].detach().cpu().numpy().astype(np.float64)


def body_poses(base, name):
    art = base.scene.articulations[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = _t(art.data.body_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    quat = _t(art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
    return {b: (pos[i], _rot(quat[i])) for i, b in enumerate(art.body_names)}


def body_vels(base, name):
    art = base.scene.articulations[name]
    lin = _t(art.data.body_lin_vel_w)[0].detach().cpu().numpy().astype(float)
    ang = _t(art.data.body_ang_vel_w)[0].detach().cpu().numpy().astype(float)
    return {b: (lin[i], ang[i]) for i, b in enumerate(art.body_names)}


def joints(base, name) -> dict:
    art = base.scene.articulations[name]
    q = _np(art.data.joint_pos)
    return {n: float(q[i]) for i, n in enumerate(art.joint_names)}


def geometry(base) -> dict:
    """Everything the scoring needs, in the housing's frame: the contact's crimp end and nose, its axis, the cavity it
    is in (0: none), the fingers' opening, the lock's depth and prong loads."""
    import numpy as np

    hp = body_poses(base, _NAMES["housing"])
    h_p, h_r = hp["base"]
    cp = body_poses(base, _NAMES["contact"])
    c_p, c_r = cp["contact"]
    crimp = h_r.T @ (c_p - h_p)
    nose = h_r.T @ (c_p + c_r @ np.array([NOSE_X, 0.0, 0.0]) - h_p)
    axis = h_r.T @ c_r[:, 0]  # toward the wire: -y when in a cavity
    cavity, radial = 0, None
    for c, (xc, zc) in CAVITIES.items():
        r_n = math.hypot(nose[0] - xc, nose[2] - zc)
        r_c = math.hypot(crimp[0] - xc, crimp[2] - zc)
        if max(r_n, r_c) < IN_RADIAL and -axis[1] > math.cos(math.radians(IN_ANGLE)) and nose[1] >= IN_DEPTH:
            cavity, radial = c, max(r_n, r_c)
    q = joints(base, _NAMES["housing"])
    latch_open = tuple(max(abs(q[f"lat{c}{s}"]) for s in FINGERS) for c in CAVITIES)  # the wider-open finger
    lock = {"depth": 1.0, "force": 0.0, "aligned": False, "over": False}
    if "lock" in _NAMES:
        lp = body_poses(base, _NAMES["lock"])
        l_p, l_r = lp["bar"]
        lb = h_r.T @ (l_p - h_p)  # the plate's underside centre, housing frame
        out = h_r.T @ l_r[:, 2]  # the lock's +z: along the cavities (+y) when in
        lq = joints(base, _NAMES["lock"])
        lock["depth"] = float(lb[1] - Y_LOCK)  # the plate's underside out from the mating face
        lock["aligned"] = bool(abs(lb[0]) < 1.0 * MM and abs(lb[2]) < LOCK_Z_PLAY and out[1] > 0.99)
        lock["over"] = bool(abs(lb[0]) < 3.0 * MM and abs(lb[2]) < LOCK_Z_PLAY + 2.0 * MM and out[1] > 0.9)  # (forced)
        lock["force"] = LOCK_K * max(lq.values())
    return {"h_p": h_p, "h_r": h_r, "crimp": crimp, "nose": nose, "axis": axis, "cavity": cavity, "radial": radial,
            "latch_open": latch_open, "lock": lock, "contact_poses": cp}


# -- fingers (for "hands off") ---------------------------------------------------------------------------------------

_FINGER_PTS: dict = {}


def finger_points(body: str):
    """Points on a gripper finger's collision meshes (thinned to <= 400), in its body's frame (yam_arena's URDF)."""
    import struct
    import xml.etree.ElementTree as ET

    import numpy as np

    if body in _FINGER_PTS:
        return _FINGER_PTS[body]
    import yam_arena

    root = Path(yam_arena.__file__).resolve().parent / "assets" / "yam"
    pts = []
    links = [lk for f in ("yam_arm.urdf", "yam.urdf") if (root / f).exists()
             for lk in ET.parse(root / f).getroot().iter("link")
             if lk.get("name") == body and lk.find("collision") is not None]
    for link in links[:1]:
        for col in link.iter("collision"):
            org = col.find("origin")
            xyz = np.array([float(v) for v in (org.get("xyz", "0 0 0") if org is not None else "0 0 0").split()])
            mesh = col.find("geometry/mesh")
            if mesh is None:
                continue
            data = (root / mesh.get("filename")).read_bytes()
            if data[:5] == b"solid":
                v = [[float(x) for x in ln.split()[1:4]] for ln in data.decode(errors="ignore").splitlines()
                     if ln.strip().startswith("vertex")]
            else:
                n = struct.unpack("<I", data[80:84])[0]
                v = [struct.unpack("<3f", data[84 + 50 * i + 12 + 12 * k: 84 + 50 * i + 24 + 12 * k])
                     for i in range(n) for k in range(3)]
            pts.append(np.asarray(v, dtype=float) + xyz)
    allp = np.unique(np.round(np.concatenate(pts), 5), axis=0) if pts else np.zeros((1, 3))
    if len(allp) > 400:
        allp = allp[np.linspace(0, len(allp) - 1, 400).astype(int)]
    _FINGER_PTS[body] = allp
    return allp


def _finger_cloud(base):
    import numpy as np

    pts = []
    for art_name, art in base.scene.articulations.items():
        if art_name in _NAMES.values():
            continue
        names = [b for b in art.body_names if b.startswith("tip")]
        if not names:
            continue
        poses_r = body_poses(base, art_name)
        for b in names:
            p, r = poses_r[b]
            pts.append(finger_points(b) @ r.T + p)
    return np.concatenate(pts) if pts else np.zeros((0, 3))


def hand_distances(base, geo) -> dict:
    """The nearest fingertip point to (a) the contact and the first WIRE_NEAR of wire, (b) everything (the housing,
    the contact, that wire, the lock)."""
    import numpy as np

    fingers = _finger_cloud(base)
    if not len(fingers):
        return {"wire": 1.0, "all": 1.0}
    cp = geo["contact_poses"]
    wire = [cp["contact"][0] + cp["contact"][1] @ np.array([x, 0.0, 0.0]) for x in (NOSE_X, -10 * MM, 0.0)]
    for i in range(WIRE_LINKS):
        if i * WIRE_LINK > WIRE_NEAR:
            break
        p, r = cp[f"w{i}"]
        wire.append(p + r @ np.array([0.5 * WIRE_LINK, 0.0, 0.0]))
    wire = np.array(wire)
    h_p, h_r = geo["h_p"], geo["h_r"]
    house = np.array([h_p + h_r @ np.array([x, y, z]) for x in (-H_W / 2, 0.0, H_W / 2)
                      for y in (0.0, H_L / 2, H_L) for z in (-H_H / 2, 0.0, H_H / 2)])
    others = [house]
    if "lock" in _NAMES:
        lp = body_poses(base, _NAMES["lock"])
        l_p, l_r = lp["bar"]
        others.append(np.array([l_p + l_r @ np.array([x, 0.0, z]) for x in (-LOCK_BAR[0] / 2, 0.0, LOCK_BAR[0] / 2)
                                for z in (0.0, LOCK_BAR[2])]))

    def dist(targets):
        return float(np.linalg.norm(targets[:, None, :] - fingers[None, :, :], axis=2).min())

    d_wire = dist(wire)
    return {"wire": d_wire, "all": min(d_wire, *(dist(t) for t in others))}


def wire_gripped(base, geo) -> bool:
    """A hand closed on the wire: every fingertip of one arm within ROBOT_TUG_NEAR of the crimp barrel or the first
    WIRE_NEAR of wire (sampled every 2 mm)."""
    import numpy as np

    cp = geo["contact_poses"]
    pts = [cp["contact"][0] + cp["contact"][1] @ np.array([x, 0.0, 0.0]) for x in (-6 * MM, -3 * MM, 0.0)]
    for k in range(1, int(WIRE_NEAR / (2 * MM)) + 1):
        dist = k * 2 * MM
        i = min(int(dist // WIRE_LINK), WIRE_LINKS - 1)
        p, r = cp[f"w{i}"]
        pts.append(p + r @ np.array([dist - i * WIRE_LINK, 0.0, 0.0]))
    pts = np.array(pts)
    for art_name, art in base.scene.articulations.items():
        if art_name in _NAMES.values():
            continue
        names = [b for b in art.body_names if b.startswith("tip")]
        if len(names) < 2:
            continue
        poses_r = body_poses(base, art_name)
        near = []
        for b in names:
            p, r = poses_r[b]
            f = finger_points(b) @ r.T + p
            near.append(float(np.linalg.norm(pts[:, None, :] - f[None, :, :], axis=2).min()) < ROBOT_TUG_NEAR)
        if all(near):
            return True
    return False


# -- the per-step update ---------------------------------------------------------------------------------------------


@dataclass
class _Track:
    forced_steps: int = 0
    forced_steps_open: int = 0
    forced: bool = False
    held: set = field(default_factory=set)  # cavities whose fingers the task holds shut
    wrong_locked: bool = False
    off_wire: int = 0
    off_all: int = 0
    hand_dist: float = 1.0
    tug: str = "none"
    tug_step: int = 0
    tug_y0: float = 0.0
    tug_moved: float = 0.0
    tug_crimp: float = 0.0
    tug_cavity: int = 0
    robot_tug: bool = False
    robot_tug_steps: int = 0
    tug_force: float = 0.0
    snapped: bool = False  # the wedgelock went home and snapped in: the task holds it on the face (SNAP_FORCE)
    last: dict = field(default_factory=dict)  # body -> (pos, rot) at the previous control step
    speeds: dict = field(default_factory=dict)  # part -> (lin m/s, ang rad/s) from the pose change
    log: list = field(default_factory=list)


_TRACK = _Track()
HANDS_ON = [False]  # a bench's "virtual hand" is on the parts (counts as hands on)
_EPISODE_STEP: list = [None]


def _set_tug(base, force: float, geo=None) -> None:
    """The tug: ``force`` on the contact out of the housing along the cavity (a hand pulling the wire), the same back on
    the housing (the other hand holding it); 0 ends it."""
    import numpy as np
    import torch

    _TRACK.tug_force = force if geo is not None else 0.0
    snap = _TRACK.snapped and "lock" in _NAMES and geo is not None
    kinds = ("contact", "housing") + (("lock",) if "lock" in _NAMES else ())
    for kind in kinds:
        art = base.scene.articulations[_NAMES[kind]]
        n = len(art.body_names)
        forces = torch.zeros((1, n, 3), dtype=torch.float32, device=base.device)
        f = np.zeros(3)
        if force > 0 and geo is not None and kind != "lock":
            out = geo["h_r"] @ [0.0, -1.0, 0.0]  # out of the cavity (housing -y)
            f += (1.0 if kind == "contact" else -1.0) * force * out
        if snap and kind != "contact":  # the snap: the lock pressed onto the face, the housing pressed back
            into = geo["h_r"] @ [0.0, -1.0, 0.0]
            f += (1.0 if kind == "lock" else -1.0) * SNAP_FORCE * into
        if np.any(f):
            body = list(art.body_names).index({"contact": TUG_BODY, "housing": "base", "lock": "bar"}[kind])
            forces[0, body] = torch.tensor(f.tolist(), dtype=torch.float32, device=base.device)
        art.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torch.zeros_like(forces),
                                                                   body_ids=list(range(n)), is_global=True)


def _hold_fingers(base, c: int | None, held: bool) -> None:
    """Cavity ``c``'s two fingers held shut (limits 0..HELD_PLAY) or free again (0..LAT_TRAVEL); ``c`` None: all."""
    import torch

    art = base.scene.articulations[_NAMES["housing"]]
    names = list(art.joint_names)
    cav = list(CAVITIES) if c is None else [c]
    ids, lims = [], []
    for cc in cav:
        for side in FINGERS:
            d = finger_dir(cc, side)
            reach = HELD_PLAY if held else LAT_TRAVEL
            ids.append(names.index(f"lat{cc}{side}"))
            lims.append([0.0, reach] if d > 0 else [-reach, 0.0])
    art.write_joint_position_limit_to_sim_index(limits=torch.tensor([lims], dtype=torch.float32, device=base.device),
                                                joint_ids=ids, env_ids=[0], warn_limit_violation=False)


def _speeds(tr, poses: dict, dt: float) -> None:
    """Each tracked body's speeds from its pose change per control step (PhysX's own velocities of parts squeezed in
    contacts jitter: cord-clamp demo 19)."""
    import numpy as np

    for key, (p, r) in poses.items():
        if key in tr.last:
            p0, r0 = tr.last[key]
            dr = r @ r0.T
            ang = math.acos(max(-1.0, min(1.0, (float(np.trace(dr)) - 1.0) / 2.0)))
            tr.speeds[key] = (float(np.linalg.norm(p - p0)) / dt, ang / dt)
        tr.last[key] = (np.array(p, dtype=float), np.array(r, dtype=float))


def update(base) -> None:
    """Once per control step: the latches (lock forced, contact locked in a wrong cavity), hands off and the tug."""
    global _TRACK
    step = getattr(base, "episode_length_buf", None)
    step = None if step is None else int(step[0])
    if step == 0 or (_EPISODE_STEP[0] is not None and step is not None and step < _EPISODE_STEP[0]):
        if _TRACK.tug == "running":
            _set_tug(base, 0.0)
        if _TRACK.held:
            _hold_fingers(base, None, False)
        _TRACK = _Track()
    elif step is not None and step == _EPISODE_STEP[0]:
        return  # already updated this step (yam_arena and a demo may both ask for the states)
    _EPISODE_STEP[0] = step
    tr = _TRACK
    dt = float(base.cfg.sim.dt * base.cfg.decimation)

    def n(seconds):
        return max(1, round(seconds / dt))

    geo = geometry(base)
    poses = {"housing": (geo["h_p"], geo["h_r"]), "contact": geo["contact_poses"]["contact"]}
    if "lock" in _NAMES:
        poses["lock"] = body_poses(base, _NAMES["lock"])["bar"]
    _speeds(tr, poses, dt)
    # forcing the lock over an open finger (latched)
    if geo["lock"]["force"] >= LOCK_FORCE:
        tr.forced_steps += 1
        if tr.forced_steps >= n(LOCK_FORCE_S):
            tr.forced = True
    else:
        tr.forced_steps = 0
    seated = geo["cavity"] and abs(geo["crimp"][1] - Y_SEATED_CRIMP) < SEATED_TOL
    locked = bool(seated and geo["latch_open"][geo["cavity"] - 1] < LATCH_CLOSED)
    if locked and geo["cavity"] != TARGET_CAVITY:
        tr.wrong_locked = True
    d = hand_distances(base, geo)
    tr.hand_dist = d["all"]
    # forced: the lock pressed in to the fingers' height over an open finger (latched)
    lock = geo["lock"]
    half = bool(geo["cavity"] and not seated and geo["crimp"][1] > Y_LAT[0] + SHOULDER_FRONT_X - 0.5 * MM)  # shoulder at
    # or under the fingers, not past them
    if lock["over"] and lock["depth"] < FORCE_DEPTH and (half or max(geo["latch_open"]) > FORCE_OPEN):
        tr.forced_steps_open += 1
        if tr.forced_steps_open >= n(LOCK_FORCE_S):
            tr.forced = True
    else:
        tr.forced_steps_open = 0
    # the wedgelock home over closed fingers: it snaps in (latched; the task then holds it on the face, see SNAP_FORCE)
    if (not tr.snapped and not tr.forced and lock["aligned"] and lock["depth"] < LOCK_IN_TOL
            and max(geo["latch_open"]) < LATCH_CLOSED):
        tr.snapped = True
        tr.log.append((step, "snapped", lock["depth"]))
    if tr.snapped and tr.tug != "running":
        _set_tug(base, 0.0, geo)  # (refreshes the snap's forces as the housing moves; a running tug does it below)
    # a seated contact whose fingers have snapped in: held (see HELD_PLAY)
    if locked and geo["cavity"] not in tr.held:
        _hold_fingers(base, geo["cavity"], True)
        tr.held.add(geo["cavity"])
    tr.off_wire = tr.off_wire + 1 if d["wire"] > HANDS_OFF_WIRE and not HANDS_ON[0] else 0
    tr.off_all = tr.off_all + 1 if d["all"] > HANDS_OFF_ALL and not HANDS_ON[0] else 0
    crimp_y = float(geo["crimp"][1])
    # the robot's own tug: a hand closed on the wire draws the clicked contact back onto its fingers (latched)
    pulled = Y_SEATED_CRIMP - TUG_MAX_MOVE < crimp_y <= Y_SEATED_CRIMP - ROBOT_TUG_PULL
    if (not tr.robot_tug and not HANDS_ON[0] and tr.tug != "running" and geo["cavity"] == TARGET_CAVITY
            and TARGET_CAVITY in tr.held and pulled and wire_gripped(base, geo)):
        tr.robot_tug_steps += 1
        if tr.robot_tug_steps >= n(ROBOT_TUG_S):
            tr.robot_tug = True
            tr.log.append((step, "robot_tug", Y_SEATED_CRIMP - crimp_y))
    else:
        tr.robot_tug_steps = 0
    if tr.tug in ("passed", "failed") and (abs(crimp_y - tr.tug_crimp) > TUG_INVALID_MOVE
                                           or geo["cavity"] != tr.tug_cavity):
        tr.tug = "none"
    if tr.tug == "running":
        if tr.off_wire == 0:
            _set_tug(base, 0.0, geo)
            tr.tug = "none"
        else:
            tr.tug_step += 1
            tr.tug_moved = max(tr.tug_moved, tr.tug_y0 - crimp_y)
            if tr.tug_moved >= TUG_MAX_MOVE:
                _set_tug(base, 0.0, geo)
                tr.tug = "failed"
            elif tr.tug_step >= n(TUG_S):
                _set_tug(base, 0.0, geo)
                tr.tug = "passed" if geo["cavity"] in tr.held else "failed"  # the fingers snapped in behind the shoulder
            else:
                _set_tug(base, TUG_FORCE_SIM * min(1.0, (tr.tug_step + 1) / n(TUG_RAMP_S)), geo)
            if tr.tug != "running":
                tr.tug_crimp, tr.tug_cavity = crimp_y, geo["cavity"]
                tr.log.append((step, tr.tug, tr.tug_moved))
    elif tr.tug == "none" and tr.off_wire >= n(HANDS_OFF_S) and geo["cavity"] and crimp_y > Y_SEATED_CRIMP - 2 * MM:
        tr.tug, tr.tug_step, tr.tug_y0, tr.tug_moved = "running", 0, crimp_y, 0.0
        _set_tug(base, TUG_FORCE_SIM / n(TUG_RAMP_S), geo)


def reset_parts(base) -> None:
    """Joints, velocities and targets of the jointed parts back to their start; no tug."""
    import torch

    global _TRACK
    for kind in ARTICULATED:
        if kind not in _NAMES:
            continue
        art = base.scene.articulations[_NAMES[kind]]
        k = len(art.joint_names)
        zero = torch.zeros((1, k), dtype=torch.float32, device=base.device)
        art.write_joint_position_to_sim_index(position=zero, env_ids=[0])
        art.write_joint_velocity_to_sim_index(velocity=zero, env_ids=[0])
        art.set_joint_position_target_index(target=zero, joint_ids=list(range(k)))
        art.permanent_wrench_composer.reset()
    if "housing" in _NAMES:
        _hold_fingers(base, None, False)
    _TRACK = _Track()
    _EPISODE_STEP[0] = None


def housing_state(base) -> HousingState:
    import numpy as np

    tr = _TRACK
    geo = geometry(base)
    h_p, h_r = geo["h_p"], geo["h_r"]
    lo, hi = bounds("housing")
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ h_r.T + h_p
    seated = bool(geo["cavity"] and abs(geo["crimp"][1] - Y_SEATED_CRIMP) < SEATED_TOL)
    locked = bool(seated and geo["latch_open"][geo["cavity"] - 1] < LATCH_CLOSED)
    mark_ok = bool(geo["cavity"] and abs(geo["crimp"][1] - Y_SEATED_CRIMP) <= MARK_TOL)
    lock = geo["lock"]
    lock_in = bool(lock["aligned"] and lock["depth"] < LOCK_IN_TOL and max(geo["latch_open"]) < LATCH_CLOSED)
    sp = [tr.speeds.get(k, (0.0, 0.0)) for k in ("housing", "contact", "lock")]
    q = np.array([0.0, 0.0, 0.0, 1.0])
    return HousingState(pos=h_p.copy(), quat_xyzw=q, lin_vel=np.array([max(s[0] for s in sp), 0.0, 0.0]),
                        ang_vel=np.array([max(s[1] for s in sp), 0.0, 0.0]), low=corners.min(axis=0),
                        high=corners.max(axis=0), cavity=int(geo["cavity"]), crimp_depth=float(geo["crimp"][1]),
                        nose_depth=float(geo["nose"][1]), seated=seated, latch_open=geo["latch_open"],
                        locked=locked, lock_in=lock_in,
                        lock_depth=float(lock["depth"]), lock_force=float(lock["force"]), forced=tr.forced,
                        wrong_locked=tr.wrong_locked, tug=tr.tug, tug_moved=tr.tug_moved, hands_off_steps=tr.off_all,
                        hand_dist=tr.hand_dist, mark_ok=mark_ok, robot_tug=tr.robot_tug)


def part_state(base, kind) -> task_file.ObjectState:
    """A jointed part's root as a plain object state (velocities from the pose change per control step)."""
    import numpy as np

    poses = body_poses(base, _NAMES[kind])
    root = BODIES[kind][0]
    p, r = poses[root]
    pts = np.array([pp for pp, _ in poses.values()])
    s = _TRACK.speeds.get(kind, (0.0, 0.0))
    return task_file.ObjectState(pos=p.copy(), quat_xyzw=_quat_xyzw(r),
                                 lin_vel=np.array([s[0], 0.0, 0.0]), ang_vel=np.array([s[1], 0.0, 0.0]),
                                 low=pts.min(axis=0), high=pts.max(axis=0))


def fixed_state(kind) -> task_file.ObjectState:
    """A fixed part's state from its spawn pose (yam_arena's own object_states reads poses through the arena world,
    which put a pose just written to a free articulation back to its spawn pose: debug runs, 2026-11-18)."""
    import numpy as np

    x, y, z, yaw = _POSES[kind]
    c, s_ = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
    lo, hi = bounds(kind)
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ rot.T + np.array([x, y, z])
    return task_file.ObjectState(pos=corners.mean(axis=0), quat_xyzw=_quat_xyzw(rot), lin_vel=np.zeros(3),
                                 ang_vel=np.zeros(3), low=corners.min(axis=0), high=corners.max(axis=0))


# -- the hook into yam_arena.scene -----------------------------------------------------------------------------------


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "ct_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_ct_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        base = env.unwrapped
        update(base)
        ours = set(_NAMES.values())
        others = [n for n in names if n not in ours]
        out = original_states(env, others) if others else {}
        kinds = {v: k for k, v in _NAMES.items()}
        for n in names:
            if n in kinds and kinds[n] in FIXED:
                out[n] = fixed_state(kinds[n])
            elif n == _NAMES.get("housing"):
                out[n] = housing_state(base)
            elif n in (_NAMES.get("contact"), _NAMES.get("lock")):
                kind = "contact" if n == _NAMES.get("contact") else "lock"
                out[n] = part_state(base, kind)
        return {n: out[n] for n in names}

    original_make_env = getattr(scene_module, "make_env", None)

    def make_env(*args, **kwargs):
        """yam_arena's env, its reset also putting the jointed parts' joints back (yam_arena resets the roots)."""
        env = original_make_env(*args, **kwargs)
        reset = env.reset

        def reset_with_parts(*a, **k):
            out = reset(*a, **k)
            reset_parts(env.unwrapped)
            return out

        env.reset = reset_with_parts
        return env

    if callable(original_make_env):
        scene_module.make_env = make_env
    task_object._spawns_ct_parts = True
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
