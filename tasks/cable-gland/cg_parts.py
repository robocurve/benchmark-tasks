"""The cable-gland task's own parts: a lidless polycarbonate enclosure with an M20 cable gland fitted in its near wall
(polyamide body, seal insert, domed cap nut, starting loose) and a round PVC-sheathed control cable whose far end is
held in a cable clamp on the table. All geometry is our own simplified design (metric thread M20 x 1.5, ISO 261 /
EN 60423; gland of the general EN 62444 kind; dimensions ours); no manufacturer CAD is used.

yam_arena task files support single primitives and library assets only, so this module is the task's own minimal
mechanism (the approach of our earlier tasks, copied so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` that also names which part it is.
- :func:`install` wraps ``yam_arena.scene.task_object`` and ``object_states`` the moment that module is imported.

Parts (mm in comments, metres in code). The gland frame: origin on the gland's axis in the plane of the wall's
outer face, +x along the axis into the box (the arms look at the cap nut), z up.

- box (fixed): grey polycarbonate enclosure, 120 deep (x) x 160 wide (y) x 60 tall, 3 mm walls, lid off; a 21 mm
  square cut-out in its near wall for the gland (the gland's flange covers it).
- gland (world-fixed articulation): body (hex flange 22 across flats, M20 thread, locknut inside the wall, bore
  13.2 mm); the cap nut (hex 24 across flats, 14 long, dome with a 13 mm mouth and a lead-in) on the thread: a
  prismatic "thread" joint (nut travel, 1.5 mm per turn) and the nut's "spin" coupled to it by a PhysX mimic joint;
  the seal insert: six pads round the bore on a "seal" ring dragged round by the nut through a slipping clutch
  (a drive capped at SEAL_CLUTCH, see _seal_follow), each pad driven inward
  by the nut's travel through the cap's cone (mimic, SEAL_RATIO mm radially per mm of travel) and sitting on a
  radial spring (the rubber's give); a 45 degree lead-in at their front. The pads squeeze the cable by contact, so
  grip and pull resistance come out of the physics.
- cable (articulation, its clamp end fixed): 8 mm round PVC sheath, links of LINK_LEN joined by two bending
  joints and a twisting joint each (springs: bending stiffness EI and torsion GJ of a small control cable), then
  the prepared end: the three cores (sheath stripped 75 mm) taped together, a thinner and softer bundle. Its laid
  curve is built into the USD (the springs rest in it) and its sheath takes a set (SET_TAU, SET_BAND).

Per control step (from the patched ``object_states``, before scoring) :func:`update` follows the seal (when it first
grips the cable all round, how far the nut has turned since, whether the sheath was crushed) and runs the pull test: once
no fingertip has been near the gland or the cable by it for 1 s, it pulls the cable outward with 30 N (ramped up in
1 s, held 0.5 s) and records how far the sheath end moved; it stops early once it moved 2 mm.
"""

from __future__ import annotations

import importlib.abc
import importlib.machinery
import inspect
import math
import os
import sys
import tempfile
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

from yam_arena import task_file

MM = 1e-3

# -- enclosure ------------------------------------------------------------------------------------------------
BOX_DEPTH = 120 * MM  # x, from the near wall's outer face
BOX_WIDTH = 160 * MM  # y
BOX_HEIGHT = 60 * MM  # z, from the table (80 mm left the wall's top edge 40 mm above the axis: the hand turning the
# nut met it, demo runs 3, 7, 8)
BOX_WALL = 3.0 * MM
AXIS_H = 35 * MM  # the gland's axis above the table
WALL_CUT = 10.5 * MM  # half the square cut-out for the gland (a 20.5 mm round hole in a real box)

# -- gland (M20 x 1.5) ----------------------------------------------------------------------------------------
PITCH = 1.5 * MM
FLANGE = (-8.0 * MM, 0.0)  # x span of the body's hex flange (22 across flats) outside the wall
FLANGE_AF = 22 * MM
BODY_THREAD_R = 10.0 * MM  # the M20 thread (visual; inside the nut)
BODY_THREAD_X0 = -27.0 * MM
LOCKNUT = (BOX_WALL, BOX_WALL + 6.0 * MM)  # inside the wall
LOCKNUT_AF = 24 * MM
GLAND_INNER_X = LOCKNUT[1]  # the gland's inner end (the locknut's face, inside the box): +9 mm
BORE_R = 6.6 * MM  # the body's bore behind the seal (guides the cable)
BORE_X = (-15.0 * MM, GLAND_INNER_X)
NUT_AF = 24 * MM
NUT_HEX_LEN = 14 * MM
NUT_DOME_LEN = 6 * MM
NUT_X0 = -30.6 * MM  # the nut's hex starts here (wall side at NUT_X0 + 14) with the nut at its start: on the
# thread's first turn (loose)
MOUTH_R = 6.5 * MM  # the cap's mouth (13 mm) ...
MOUTH_LEAD_R = 9.5 * MM  # ... with a 45 degree lead-in from 19 mm
NUT_INNER_R = 10.0 * MM  # the hex's inside (over the body's thread)
SEAL_X = (-27.0 * MM, -15.0 * MM)  # the seal pads' axial span (fixed: the cap's cone squeezes them radially)
SEAL_OPEN_R = 7.265 * MM  # the pads' inner radius with the nut at its start (14.5 mm open): first contact on the 8 mm
# cable 3 turns from the thread's start, as measured on a real gland (with 6.5 mm it came after 2.15 turns)
PAD_THICK = 2.0 * MM
PAD_LEAD = 1.5 * MM  # the pads' lead-in (45 degrees) at the seal's front
PAD_ARC = math.radians(50.0)  # each of 6 pads (10 degree gaps)
SEAL_N = 6
SEAL_RATIO = 0.6  # radial closing per axial nut travel (the cap's cone)
NUT_TRAVEL = 8.5 * MM  # the thread joint's range [0, NUT_TRAVEL]: at the end the nut sits on the flange (5.67 turns:
# 2.67 past first contact, beyond the measured damage mark at 1.75)
PAD_K = 8.0e4  # N/m: each pad's radial spring (the rubber's give; at 40 N/mm the cable only held 30 N from ~0.45 turn
# past first contact, leaving no room below the lower mark, demo run 32)
PAD_D = 0.3  # N s/m
PAD_TRAVEL = 1.5 * MM  # how far a pad can be pushed back (hard stop)
NUT_FRICTION = 0.08  # Nm: thread friction (static and moving; holds the nut: at 0.02 the twisted cable and the seal
# pads unscrewed every nut back to its start once let go, bench run 1; at 0.15 the hand could no longer turn it to the
# crush mark, demo run 12)
SEAL_CLUTCH = 0.03  # Nm: the seal turns with the nut only up to this torque (the cap's cone slipping on the seal):
# held, the cable stops the seal; unheld, the seal twists it. (Coupled rigidly by a mimic joint, a held cable was wound
# up ~150 degrees inside the seal and, let go, set nut and cable ringing, bench run 2.)
SEAL_K, SEAL_D = 2.0, 0.01  # Nm/rad, Nm s/rad: the drive that makes the seal follow the nut (capped at SEAL_CLUTCH)
NUT_ARMATURE = 2.0e-4  # kg m^2 on the nut's spin (a hand's share, as in our screw tasks)
NUT_MASS = 0.010
LINK_ARMATURE = 1.0e-4
SMALL_MASS = 0.001

# -- cable (3 x 1.5 mm2 control cable, PVC sheath) -------------------------------------------------------------
CABLE_R = 4.0 * MM
LINK_LEN = 20 * MM
SHEATH_LINKS = int(os.environ.get("CG_SHEATH_LINKS", "21"))  # 420 mm of sheath from the clamp (shorter: pulled
# tight before the sheath was through, demo runs 21, 25; the bench: shorter)
CORE_LINKS = 3
CORE_LEN = 25 * MM  # 75 mm of stripped cores
CORE_R = 3.2 * MM  # three 3 mm cores taped together
SHEATH_MASS = 0.002  # kg per 20 mm link (~100 g/m)
CORE_MASS = 0.0012
GIMBAL_MASS = 0.0002
CLAMP_OUT = 12 * MM  # the cable leaves the clamp block here (cable frame x)
CLAMP_SIZE = (24 * MM, 22 * MM, 18 * MM)
CLAMP_H = CABLE_R  # the cable's centre at the clamp above the table: the clamp sits on the table. (Tried for a twist
# rule: a clamp on a 120 mm stand, the cable then rising behind the hand jammed against the gripper, demo runs 21-22.)
POST = 20 * MM
FOOT = (60 * MM, 60 * MM, 5 * MM)
EI = 6.0e-3  # N m^2: bending stiffness of a small PVC control cable (our estimate)
GJ = 4.0e-3  # N m^2: torsion
EI_CORES = 1.5e-3
BEND_D = 3.0e-3  # Nm s/rad (about a third of critical on a link with its joint armature; 2e-4 let twist ring, bench 2)
TWIST_D = 3.0e-3
SET_TAU = 1.5  # s: the cable's set creeps: each bending spring's rest angle follows the joint's angle with this time
# constant (PVC takes a set). With a fully elastic memory of the laid curve, the cable sprang 47 mm back out of the
# open gland whenever the hand let go to take it again (demo run 2). Twist stays elastic.
SET_BAND = math.radians(6.0)  # ... but only the part of a bend beyond this elastic band creeps (a laid-down cable comes
# to rest; creeping all of it, the cable never stopped moving, demo run 13; at 6 degrees the bowed cable sprang 30-35
# mm back out of the open gland whenever the hand let go, demo run 23; at 1 degree gravity's bend crept too). The core
# bundle doesn't take a set (creeping, it drooped onto the box's floor and jammed the cable, demo run 24).
BEND_LIMIT = 45.0  # deg per link (the minimum bending radius, ~4 x the diameter)
TWIST_LIMIT = 720.0  # deg per link (never reached)

# -- scoring constants (used by the task) ----------------------------------------------------------------------
SHEATH_PAST = 5.0 * MM  # the sheath's end at least this past the gland's inner end
CONTACT_DEFL = 0.02 * MM  # the seal grips the cable all round: every pad pushed back at least this ... (the mean let
# a cable resting on the lower pads count as contact half a turn early, demo run 14)
CONTACT_S = 0.25  # ... for this long in a row
SNUG_TURNS = (1.0, 1.5)  # the marks: nut turns past the seal's first contact (snug between them; past the upper one:
# over-tightened, the seal overstressed but the sheath intact), measured on a real M20 polyamide gland with an 8 mm
# H05VV-F 3G1.5 cable (three glands, matching results): H = 1.0 (the first 1/8-turn step holding a 3.06 kg weight),
# T = 1.5 (3 Nm reached, torque wrench)
CRUSH_TURNS = 1.75  # turned this far past first contact, the seal crushes the sheath (for good): measured, the first
# 1/8-turn step at which the seal cage's teeth marked the sheath
PULL_FORCE = 30.0  # N
PULL_MAX_MOVE = 2.0 * MM
PULL_RAMP_S = 1.0  # the pull ramps up over this (as a hand on a spring scale; ramped in 0.5 s, a slipping cable was
# yanked 28 mm out before the 2 mm stop could act, demo run 16) ...
PULL_S = 1.5  # ... and lasts this long in all
HANDS_OFF_DIST = 20 * MM  # no fingertip within this of the nut or the cable's last 15 cm before the gland
HANDS_OFF_S = 1.0  # for this long
HANDS_ZONE = 150 * MM  # cable links within this of the gland's origin count for "hands off"
PULL_INVALID_TURNS = 0.02  # a pull result stops counting once the nut turned this far ...
PULL_INVALID_MOVE = 1.0 * MM  # ... or the sheath moved this far since

# -- materials and looks ------------------------------------------------------------------------------------
MATERIALS = {  # (static friction, dynamic friction, combine mode)
    "plastic": (0.40, 0.35, "average"),  # polycarbonate box, polyamide gland
    "rubber": (0.80, 0.75, "max"),  # the seal pads (max: the pads' friction governs pad-on-sheath)
    "pvc": (0.50, 0.45, "average"),  # cable sheath
}
LOOKS = {  # sRGB, roughness, metallic
    "box": ((0.62, 0.64, 0.66), 0.5, 0.0),
    "gland": ((0.20, 0.21, 0.23), 0.55, 0.0),  # black-grey polyamide
    "seal": ((0.05, 0.05, 0.05), 0.9, 0.0),
    "sheath": ((0.55, 0.56, 0.57), 0.6, 0.0),  # grey PVC
    "core_bn": ((0.45, 0.25, 0.12), 0.5, 0.0),
    "core_bu": ((0.15, 0.30, 0.70), 0.5, 0.0),
    "core_gy": ((0.30, 0.65, 0.20), 0.5, 0.0),  # green-yellow (drawn green)
    "tape": ((0.08, 0.08, 0.08), 0.8, 0.0),
    "clamp": ((0.15, 0.15, 0.16), 0.6, 0.0),
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
}
CONTACT_OFFSET = 0.5 * MM


# -- shapes -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One piece of a part, in the part's frame: a box (``center``, ``size``), a prism (a 2-D ``profile`` (y, z)
    extruded along x over ``span``), a cylinder or capsule (``center``, ``radius``, ``length`` along ``axis``), or a
    convex ``hull`` of points (a collider; drawn only if ``visible``)."""

    body: str
    look: str
    material: str | None  # None: visual only
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float] | None = None
    profile: tuple[tuple[float, float], ...] | None = None
    span: tuple[float, float] = (0.0, 0.0)
    radius: float = 0.0
    length: float = 0.0
    axis: str = "X"
    capsule: bool = False
    visible: bool = True
    hull: tuple | None = None

    def points(self):
        if self.hull is not None:
            return list(self.hull)
        if self.profile is not None:
            return [(s, p[0], p[1]) for p in self.profile for s in self.span]
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2 + (self.radius if self.capsule else 0.0)
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _hexagon(af: float, start: float = 0.0):
    """A hexagon's corners (y, z), ``af`` across flats, a flat facing +y when ``start`` = 0."""
    r = af / 2 / math.cos(math.pi / 6)
    return [(r * math.cos(start + math.pi / 6 + k * math.pi / 3), r * math.sin(start + math.pi / 6 + k * math.pi / 3))
            for k in range(6)]


def _ring_pieces(body, look, material, x0, x1, r_in0, r_in1, r_out, n=12, visible=False, a0=0.0):
    """A ring round the x axis as n convex pieces: inner radius r_in0 at x0 to r_in1 at x1 (a cone), outer r_out."""
    out = []
    for k in range(n):
        ta, tb = a0 + 2 * math.pi * k / n, a0 + 2 * math.pi * (k + 1) / n
        pts = []
        for x, ri in ((x0, r_in0), (x1, r_in1)):
            for t in (ta, tb):
                # the inner edge pulled in a little at the chord's middle so the facets don't leave a gap
                pts += [(x, ri * math.cos(t), ri * math.sin(t)), (x, r_out * math.cos(t), r_out * math.sin(t))]
        out.append(Shape(body, look, material, hull=tuple(pts), visible=visible))
    return out


def _hex_ring(body, look, material, x0, x1, af, r_in, visible=False, start=0.0):
    """A hexagon prism with a round hole (r_in) as six convex pieces (one per side)."""
    corners = _hexagon(af, start)
    out = []
    for k in range(6):
        (ya, za), (yb, zb) = corners[k], corners[(k + 1) % 6]
        ta, tb = math.atan2(za, ya), math.atan2(zb, yb)
        if tb < ta:
            tb += 2 * math.pi
        inner = [(r_in * math.cos(ta + (tb - ta) * i / 3), r_in * math.sin(ta + (tb - ta) * i / 3)) for i in range(4)]
        prof = [(ya, za), (yb, zb)] + inner[::-1]
        out.append(Shape(body, look, material, hull=tuple((x, y, z) for x in (x0, x1) for y, z in prof),
                         visible=visible))
    return out


# -- the enclosure ---------------------------------------------------------------------------------------------


def box_shapes() -> list[Shape]:
    """The box in the gland frame: near wall (with the cut-out), side walls, far wall, bottom (all collide)."""
    d, w, t, c = BOX_DEPTH, BOX_WIDTH / 2, BOX_WALL, WALL_CUT
    z0, z1 = -AXIS_H, BOX_HEIGHT - AXIS_H
    return [
        # near wall round the cut-out
        _box("box", "box", "plastic", 0.0, t, -w, w, c, z1),
        _box("box", "box", "plastic", 0.0, t, -w, w, z0, -c),
        _box("box", "box", "plastic", 0.0, t, -w, -c, -c, c),
        _box("box", "box", "plastic", 0.0, t, c, w, -c, c),
        _box("box", "box", "plastic", d - t, d, -w, w, z0, z1),  # far wall
        _box("box", "box", "plastic", t, d - t, -w, -w + t, z0, z1),  # right wall (-y)
        _box("box", "box", "plastic", t, d - t, w - t, w, z0, z1),  # left wall
        _box("box", "box", "plastic", t, d - t, -w + t, w - t, z0, z0 + t),  # bottom
    ]


# -- the gland ----------------------------------------------------------------------------------------------------


def pad_angle(i: int) -> float:
    """Pad i's direction round the axis (rad, from +y toward +z)."""
    return 2 * math.pi * i / SEAL_N


def gland_shapes() -> list[Shape]:
    out = []
    # body (fixed root): flange, thread (visual), the part through the wall, locknut, and the bore behind the seal
    out.append(Shape("base", "gland", None, profile=tuple(_hexagon(FLANGE_AF)), span=FLANGE))
    out.append(Shape("base", "gland", None, ((BODY_THREAD_X0 + FLANGE[0]) / 2, 0.0, 0.0), radius=BODY_THREAD_R,
                     length=FLANGE[0] - BODY_THREAD_X0))
    out.append(Shape("base", "gland", None, (BOX_WALL / 2, 0.0, 0.0), radius=BODY_THREAD_R, length=BOX_WALL))
    out.append(Shape("base", "gland", None, profile=tuple(_hexagon(LOCKNUT_AF)), span=LOCKNUT))
    out.append(Shape("base", "hole", None, ((BORE_X[0] + BORE_X[1]) / 2, 0.0, 0.0), radius=BORE_R + 0.3 * MM,
                     length=BORE_X[1] - BORE_X[0] + 0.4 * MM))  # dark bore (drawn just proud of the body's ends)
    out += _ring_pieces("base", "gland", "plastic", BORE_X[0], BORE_X[1], BORE_R, BORE_R, BODY_THREAD_R - 1.0 * MM)
    # (a short cone in front of the bore so nothing catches on its edge when the pads are open)
    out += _ring_pieces("base", "gland", "plastic", BORE_X[0] - 1.5 * MM, BORE_X[0], BORE_R + 1.5 * MM, BORE_R,
                        BODY_THREAD_R - 1.0 * MM)
    # the cap nut (its own body; its frame = the gland frame at the start): hex, dome, mouth with lead-in
    hx0, hx1 = NUT_X0, NUT_X0 + NUT_HEX_LEN
    dx0 = NUT_X0 - NUT_DOME_LEN
    out.append(Shape("nut", "gland", None, profile=tuple(_hexagon(NUT_AF)), span=(hx0, hx1)))
    out += _hex_ring("nut", "gland", "plastic", hx0, hx1, NUT_AF, NUT_INNER_R)
    # dome: three stepped visual discs and a collider ring from the hex's inscribed circle down to the mouth
    r_hex = NUT_AF / 2
    for k in range(3):
        f0, f1 = k / 3, (k + 1) / 3
        r = r_hex - (r_hex - (MOUTH_LEAD_R + 1.0 * MM)) * (f0 + f1) / 2 * (0.4 + 0.6 * f1)
        out.append(Shape("nut", "gland", None, (hx0 - NUT_DOME_LEN * (f0 + f1) / 2, 0.0, 0.0), radius=r,
                         length=NUT_DOME_LEN / 3))
    out.append(Shape("nut", "hole", None, (dx0 + 0.1 * MM, 0.0, 0.0), radius=MOUTH_R, length=0.4 * MM))
    lead = MOUTH_LEAD_R - MOUTH_R
    out += _ring_pieces("nut", "gland", "plastic", dx0, dx0 + lead, MOUTH_LEAD_R, MOUTH_R, r_hex - 0.5 * MM)
    out += _ring_pieces("nut", "gland", "plastic", dx0 + lead, hx0, MOUTH_R, MOUTH_R, r_hex - 0.5 * MM)
    # seal pads (each its own body; drawn at the start, the nut loose)
    for i in range(SEAL_N):
        a = pad_angle(i)
        pts = []
        # (x, inner radius) along the pad: a 45 degree lead-in at its front so an off-centre sheath slides in (square,
        # its rounded end caught on the pads' front faces, demo runs 23-28)
        x0, x1 = SEAL_X
        for x, r_in in ((x0, SEAL_OPEN_R + PAD_LEAD), (x0 + PAD_LEAD, SEAL_OPEN_R), (x1, SEAL_OPEN_R)):
            for j in range(5):
                t = a - PAD_ARC / 2 + PAD_ARC * j / 4
                pts.append((x, r_in * math.cos(t), r_in * math.sin(t)))
        for x in (x0, x1):
            for j in range(5):
                t = a - PAD_ARC / 2 + PAD_ARC * j / 4
                r = SEAL_OPEN_R + PAD_THICK
                pts.append((x, r * math.cos(t), r * math.sin(t)))
        out.append(Shape(f"pad{i}", "seal", "rubber", hull=tuple(pts), visible=True))
    return out


GLAND_BODIES = ("base", "carrier", "nut", "seal", *(f"drv{i}" for i in range(SEAL_N)),
                *(f"pad{i}" for i in range(SEAL_N)))


def gland_joints():
    """name: (type, parent, child, axis, frame turn about x (rad), lower, upper, stiffness, damping, armature).
    Every gland body's frame is the gland frame (prismatic limits in m, revolute in deg)."""
    turns = NUT_TRAVEL / PITCH + 0.5
    out = {
        "thread": ("pri", "base", "carrier", "X", 0.0, 0.0, NUT_TRAVEL, 0.0, 0.0, LINK_ARMATURE),
        "spin": ("rev", "carrier", "nut", "X", 0.0, -5.0, 360.0 * turns, 0.0, 0.0, NUT_ARMATURE),
        "seal": ("rev", "base", "seal", "X", 0.0, -360.0 * turns, 360.0 * turns, 0.0, 0.0, LINK_ARMATURE),
    }
    for i in range(SEAL_N):
        a = pad_angle(i)
        # radial joints along the frame's y axis turned by a about x: + = outward
        out[f"d{i}"] = ("pri", "seal", f"drv{i}", "Y", a, -SEAL_RATIO * NUT_TRAVEL, 0.0, 0.0, 0.0, LINK_ARMATURE)
        out[f"p{i}"] = ("pri", f"drv{i}", f"pad{i}", "Y", a, 0.0, PAD_TRAVEL, PAD_K, PAD_D, LINK_ARMATURE)
    return out


# mimic couplings: joint + gearing * thread = 0 (gearing per metre of the nut's travel). Tightening (the nut moving
# +x, toward the wall) turns it positively about +x: clockwise as seen from the arms (a right-hand thread).
MIMICS = {"spin": -360.0 / PITCH, **{f"d{i}": SEAL_RATIO for i in range(SEAL_N)}}  # (the seal: a clutch, see update)


def gland_masses():
    """body: (mass, centre of mass, diagonal inertia)."""
    pad_r = SEAL_OPEN_R + PAD_THICK / 2
    out = {"base": (0.05, (0.0, 0.0, 0.0), (1e-5, 1e-5, 1e-5)),
           "carrier": (SMALL_MASS, (0.0, 0.0, 0.0), (1e-7, 1e-7, 1e-7)),
           "nut": (NUT_MASS, (NUT_X0 + 4 * MM, 0.0, 0.0), (1.5e-6, 1.0e-6, 1.0e-6)),
           "seal": (SMALL_MASS, (sum(SEAL_X) / 2, 0.0, 0.0), (1e-7, 1e-7, 1e-7))}
    for i in range(SEAL_N):
        a = pad_angle(i)
        c = (sum(SEAL_X) / 2, pad_r * math.cos(a), pad_r * math.sin(a))
        out[f"drv{i}"] = (SMALL_MASS, c, (1e-7, 1e-7, 1e-7))
        out[f"pad{i}"] = (0.002, c, (1e-7, 1e-7, 1e-7))
    return out


# -- the cable ------------------------------------------------------------------------------------------------------


def cable_links():
    """(name, start x, length, radius, is core) of each link, in the cable frame (the clamp at the origin, the
    cable laid straight along +x)."""
    out, x = [], CLAMP_OUT
    for i in range(SHEATH_LINKS):
        out.append((f"link{i}", x, LINK_LEN, CABLE_R, False))
        x += LINK_LEN
    for i in range(CORE_LINKS):
        out.append((f"core{i}", x, CORE_LEN, CORE_R, True))
        x += CORE_LEN
    return out


SHEATH_END_X = CLAMP_OUT + SHEATH_LINKS * LINK_LEN  # cable frame, straight


CABLE_HEADINGS: list = []  # the laid-out cable: per link (yaw, pitch) in degrees relative to the clamp (pitch + = down),
# or a yaw alone; set by the task before spawning (empty: straight). The curve is built into the USD (joint angle 0 =
# as laid), so the bending springs rest in the laid shape (a cable's set) and nothing is written at reset (curved
# starting joint angles written at reset made PhysX produce NaNs, debug runs 1-2).


def _yaw_pitch(i: int):
    if not CABLE_HEADINGS:
        return 0.0, 0.0
    h = CABLE_HEADINGS[i]
    return (math.radians(h[0]), math.radians(h[1])) if isinstance(h, (tuple, list)) else (math.radians(h), 0.0)


def _rot_yp(yaw: float, pitch: float):
    """Rz(yaw) Ry(pitch) as nested lists (pitch + turns the link's x axis down)."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]


def cable_frames() -> dict:
    """{body: ((x, y, z), rotation as nested lists)}: each body's frame in the cable frame (the clamp at the origin),
    as laid."""
    out = {"clamp": ((0.0, 0.0, 0.0), _rot_yp(0.0, 0.0))}
    x, y, z = CLAMP_OUT, 0.0, 0.0
    for i, (name, _, length, _, _) in enumerate(cable_links()):
        r = _rot_yp(*_yaw_pitch(i))
        for b in (f"ga{i}", f"gb{i}", name):
            out[b] = ((x, y, z), r)
        x, y, z = x + length * r[0][0], y + length * r[1][0], z + length * r[2][0]
    return out


def laid_points(origin, yaw: float, headings):
    """Each link's start point, the sheath's end and the cores' tip (env frame, 3-D) for a cable laid with
    ``headings`` from a clamp at ``origin`` turned ``yaw``."""
    saved = list(CABLE_HEADINGS)
    CABLE_HEADINGS[:] = list(headings)
    try:
        frames = cable_frames()
    finally:
        CABLE_HEADINGS[:] = saved
    c, s_ = math.cos(yaw), math.sin(yaw)

    def world(p):
        return (origin[0] + c * p[0] - s_ * p[1], origin[1] + s_ * p[0] + c * p[1], origin[2] + p[2])

    links = cable_links()
    pts = [world(frames[name][0]) for name, *_ in links]
    last = frames[links[-1][0]]
    tip = world(tuple(last[0][k] + links[-1][2] * last[1][k][0] for k in range(3)))
    sheath = world(frames[links[SHEATH_LINKS][0]][0])
    return pts, sheath, tip


def cable_bodies():
    """Bodies with their frame's position (cable frame, as laid)."""
    return [(b, p) for b, (p, _) in cable_frames().items()]


def cable_joints():
    """name: (type, parent, child, axis, frame turn, lower, upper, stiffness, damping, armature) (revolute; at the
    child's frame origin). Bend about z (sideways), then about y (up/down), then twist about the link's axis."""
    out, parent = {}, "clamp"
    for i, (name, _, length, _, core) in enumerate(cable_links()):
        ei = EI_CORES if core else EI
        k_b = math.radians(1.0) * ei / length  # Nm per degree (USD drive stiffness on angular joints is per degree)
        k_t = math.radians(1.0) * (GJ if not core else EI_CORES) / length
        pre = "c" if core else ""
        out[f"{pre}bz{i}"] = ("rev", parent, f"ga{i}", "Z", 0.0, -BEND_LIMIT, BEND_LIMIT, k_b, BEND_D, LINK_ARMATURE)
        out[f"{pre}by{i}"] = ("rev", f"ga{i}", f"gb{i}", "Y", 0.0, -BEND_LIMIT, BEND_LIMIT, k_b, BEND_D,
                              LINK_ARMATURE)
        out[f"{pre}tw{i}"] = ("rev", f"gb{i}", name, "X", 0.0, -TWIST_LIMIT, TWIST_LIMIT, k_t, TWIST_D,
                              LINK_ARMATURE)
        parent = name
    return out


def cable_shapes() -> list[Shape]:
    """The clamp block, the sheath links (capsules), the core links (a taped bundle: one capsule collider, three
    coloured cores drawn), and a black ring marking the sheath's cut end."""
    sx, sy, sz = CLAMP_SIZE
    if CLAMP_H > 20 * MM:  # on a stand: a post down to the table and its foot
        out = [_box("clamp", "clamp", "plastic", CLAMP_OUT - sx, CLAMP_OUT, -sy / 2, sy / 2, -sz / 2, sz / 2),
               _box("clamp", "clamp", "plastic", CLAMP_OUT - sx / 2 - POST / 2, CLAMP_OUT - sx / 2 + POST / 2,
                    -POST / 2, POST / 2, -CLAMP_H + FOOT[2], -sz / 2),
               _box("clamp", "clamp", "plastic", CLAMP_OUT - sx / 2 - FOOT[0] / 2, CLAMP_OUT - sx / 2 + FOOT[0] / 2,
                    -FOOT[1] / 2, FOOT[1] / 2, -CLAMP_H, -CLAMP_H + FOOT[2])]
    else:  # on the table
        out = [_box("clamp", "clamp", "plastic", CLAMP_OUT - sx, CLAMP_OUT, -sy / 2, sy / 2, -CLAMP_H,
                    -CLAMP_H + sz)]
    for name, _, length, r, core in cable_links():  # in each link's own frame (its start, x along it)
        if not core:
            out.append(Shape(name, "sheath", "pvc", (length / 2, 0.0, 0.0), radius=r, length=length, capsule=True))
            continue
        out.append(Shape(name, "tape", "pvc", (length / 2, 0.0, 0.0), radius=r, length=length, capsule=True,
                         visible=False))
        for k, look in enumerate(("core_bn", "core_bu", "core_gy")):
            t = 2 * math.pi * k / 3 + math.pi / 2
            out.append(Shape(name, look, None, (length / 2, 1.6 * MM * math.cos(t), 1.6 * MM * math.sin(t)),
                             radius=1.5 * MM, length=length + 2.0 * MM))
    return out


def cable_masses():
    out = {"clamp": (0.05, (0.0, 0.0, 0.0), (1e-5, 1e-5, 1e-5))}
    for i, (name, x, length, r, core) in enumerate(cable_links()):
        m = CORE_MASS if core else SHEATH_MASS
        i_t = m * (length ** 2 / 12 + r ** 2 / 4)
        out[name] = (m, (length / 2, 0.0, 0.0), (m * r ** 2 / 2, i_t, i_t))  # (com in the body's frame)
        out[f"ga{i}"] = (GIMBAL_MASS, (0.0, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
        out[f"gb{i}"] = (GIMBAL_MASS, (0.0, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
    return out


# -- bookkeeping --------------------------------------------------------------------------------------------------

SHAPES = {"box": box_shapes, "gland": gland_shapes, "cable": cable_shapes}
BODIES = {"box": ("box",), "gland": GLAND_BODIES, "cable": tuple(b for b, _ in cable_bodies())}


@lru_cache(maxsize=None)
def bounds(kind: str):
    lo, hi = [math.inf] * 3, [-math.inf] * 3
    for sh in SHAPES[kind]():
        for p in sh.points():
            for i in range(3):
                lo[i], hi[i] = min(lo[i], p[i]), max(hi[i], p[i])
    return tuple(lo), tuple(hi)


# -- task-file objects --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin (the gland frame's origin for
    box and gland; the clamp for the cable), ``yaw`` its turn about z."""

    kind: str = "box"

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name, kind, pos, yaw=0.0) -> Part:
    lo, hi = bounds(kind)
    size = tuple(b - a for a, b in zip(lo, hi, strict=True))
    color = {"box": LOOKS["box"][0], "gland": LOOKS["gland"][0], "cable": LOOKS["sheath"][0]}[kind]
    # the box is fixed; the gland and the cable are articulations with fixed roots (a mass marks them as moving
    # for yam_arena's checks; their USD sets the real masses)
    return Part(name=name, pos=tuple(pos), size=size, mass=None if kind == "box" else 0.1, color=color, yaw=yaw,
                kind=kind)


@dataclass(frozen=True)
class GlandState(task_file.ObjectState):
    """The gland: the nut's turns since the start, the seal (first contact, turns past it, mean pad deflection),
    whether the sheath was ever crushed, and the latest pull test."""

    turns: float = 0.0
    contact_turns: float | None = None
    past_contact: float = 0.0  # turns past first contact (0 before contact)
    pad_deflection: float = 0.0  # m, mean
    crushed: bool = False
    hands_off_steps: int = 0
    pull: str = "none"  # none | running | passed | failed
    pull_moved: float = 0.0  # m
    pull_sheath_x: float = -1.0  # the sheath end before the latest pull test (gland frame x, m)


@dataclass(frozen=True)
class CableState(task_file.ObjectState):
    """The cable: where the sheath's cut end is along the gland's axis (gland frame x, m), how far off the axis,
    and the twist (deg) of the cable just outside the cap nut against the clamp."""

    through: bool = False  # the cable's centreline crosses the wall inside the gland's bore
    sheath_x: float = -1.0
    sheath_off_axis: float = 1.0
    twist_deg: float = 0.0  # summed twist joints, clamp to the mouth (diagnostic)
    roll_deg: float = 0.0  # the sheath's roll just outside the mouth (the twist rule)
    tip_x: float = -1.0


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


def _polygon_area(poly) -> float:
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, list(poly[1:]) + [poly[0]]))


def _hull_faces(pts):
    """Triangles of the convex hull of a small point set (brute force; for drawing a hull)."""
    import numpy as np

    p = np.asarray(pts, float)
    n = len(p)
    c = p.mean(axis=0)
    faces = []
    for i in range(n):
        for j in range(i + 1, n):
            for k in range(j + 1, n):
                nrm = np.cross(p[j] - p[i], p[k] - p[i])
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                d = (p - p[i]) @ nrm
                if np.all(d <= 1e-9):
                    faces.append((i, j, k) if (c - p[i]) @ nrm < 0 else (i, k, j))
                elif np.all(d >= -1e-9):
                    faces.append((i, k, j) if (c - p[i]) @ nrm > 0 else (i, j, k))
    return faces


def _quatf(m):
    """A rotation matrix (nested lists) as a Gf.Quatf (w, x, y, z)."""
    from pxr import Gf

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
    return Gf.Quatf(w, x, y, z)


def _quat_x(a: float):
    from pxr import Gf

    return Gf.Quatf(math.cos(a / 2), math.sin(a / 2), 0.0, 0.0)


def write_usd(kind: str, path: Path) -> Path:
    """One part as a USD file: bodies, colliders, looks, materials and (gland, cable) joints."""
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

    jointed = kind in ("gland", "cable")
    cframes = cable_frames() if kind == "cable" else {}
    frames = {b: p for b, (p, _) in cframes.items()}
    masses = gland_masses() if kind == "gland" else cable_masses() if kind == "cable" else {}
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        xf = UsdGeom.Xform.Define(stage, path_b) if jointed else UsdGeom.Xform(root)
        if jointed and any(frames.get(body, (0.0, 0.0, 0.0))):
            xf.AddTranslateOp().Set(Gf.Vec3d(*frames[body]))
        if body in cframes:
            xf.AddOrientOp().Set(_quatf(cframes[body][1]))
        body_paths[body] = path_b
        if not jointed:
            continue  # the box: a fixed collider
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m, com, inertia = masses[body]
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.CreateMassAttr(m)
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:maxDepenetrationVelocity": 0.3,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        # gland shapes are written in the part's frame (bodies may be offset); cable shapes in their body's own frame
        off = (0.0, 0.0, 0.0) if kind == "cable" else frames.get(sh.body, (0.0, 0.0, 0.0))

        def loc(p, off=off):
            return tuple(a - b for a, b in zip(p, off, strict=True))

        if sh.hull is not None:
            pts = [loc(p) for p in sh.hull]
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/hull_{i}")
            faces = _hull_faces(pts) if sh.visible else [(0, j, j + 1) for j in range(1, len(pts) - 1)]
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(faces)))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray([v for f in faces for v in f]))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        elif sh.profile is not None:
            geom = UsdGeom.Mesh.Define(stage, f"{parent}/prism_{i}")
            prof = list(sh.profile)
            if _polygon_area(prof) < 0:
                prof = prof[::-1]
            n = len(prof)
            s0, s1 = sh.span
            pts = [loc((s0, y, z)) for y, z in prof] + [loc((s1, y, z)) for y, z in prof]
            counts, idx = [n, n], list(range(n))[::-1] + [n + k for k in range(n)]
            for k in range(n):
                counts.append(4)
                idx += [k, (k + 1) % n, n + (k + 1) % n, n + k]
            geom.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in pts]))
            geom.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
            geom.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
            geom.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
            geom.CreateDoubleSidedAttr(True)
        elif sh.size is not None:
            geom = UsdGeom.Cube.Define(stage, f"{parent}/box_{i}")
            geom.CreateSizeAttr(1.0)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
            geom.AddScaleOp().Set(Gf.Vec3f(*sh.size))
        elif sh.capsule:
            geom = UsdGeom.Capsule.Define(stage, f"{parent}/capsule_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.CreateHeightAttr(sh.length)
            geom.CreateAxisAttr(sh.axis)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
        else:
            geom = UsdGeom.Cylinder.Define(stage, f"{parent}/cyl_{i}")
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
        if sh.profile is not None or sh.hull is not None:
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
        UsdShade.MaterialBindingAPI(prim).Bind(mats[sh.material], UsdShade.Tokens.strongerThanDescendants, "physics")
        _apply(prim, "PhysxCollisionAPI", {"physxCollision:contactOffset": CONTACT_OFFSET,
                                           "physxCollision:restOffset": 0.0})

    if jointed:
        root_body = "base" if kind == "gland" else "clamp"
        rb = stage.GetPrimAtPath(body_paths[root_body])
        UsdPhysics.ArticulationRootAPI.Apply(rb)
        _apply(rb, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        joints = gland_joints() if kind == "gland" else cable_joints()
        for name, (jt, parent, child, ax, turn, lo, hi, k, d, arm) in joints.items():
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(
                stage, f"/Part/joints/{name}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr(ax)
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            fp, fc = frames.get(parent, (0.0, 0.0, 0.0)), frames.get(child, (0.0, 0.0, 0.0))
            if kind == "cable":  # the child's frame in the parent's (both as laid)
                rp_, rc_ = cframes[parent][1], cframes[child][1]
                dv = [c - p for c, p in zip(fc, fp, strict=True)]
                local = [sum(rp_[k][i] * dv[k] for k in range(3)) for i in range(3)]  # Rp^T dv
                rel = [[sum(rp_[k][i] * rc_[k][j] for k in range(3)) for j in range(3)] for i in range(3)]  # Rp^T Rc
                joint.CreateLocalPos0Attr(Gf.Vec3f(*local))
                joint.CreateLocalRot0Attr(_quatf(rel))
                joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            else:
                joint.CreateLocalPos0Attr(Gf.Vec3f(*(c - p for c, p in zip(fc, fp, strict=True))))
                joint.CreateLocalRot0Attr(_quat_x(turn))
                joint.CreateLocalRot1Attr(_quat_x(turn))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular" if jt == "rev" else "linear")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(k)
            drive.CreateDampingAttr(d)
            drive.CreateTargetPositionAttr(0.0)
            _apply(joint.GetPrim(), "PhysxJointAPI", {"physxJoint:armature": arm})
        if kind == "gland":
            for name, gearing in MIMICS.items():
                prim = stage.GetPrimAtPath(f"/Part/joints/{name}")
                _apply(prim, "PhysxMimicJointAPI:rotX", {"physxMimicJoint:rotX:gearing": float(gearing),
                                                         "physxMimicJoint:rotX:offset": 0.0})
                prim.CreateRelationship("physxMimicJoint:rotX:referenceJoint").SetTargets(
                    [Sdf.Path("/Part/joints/thread")])
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="cable_gland_parts_"))
    path = _USD_DIR / f"{kind}.usda"
    if not path.exists():
        write_usd(kind, path)
    return str(path)


_GLANDS: dict[str, tuple] = {}  # gland name -> (x, y, z, yaw) of its frame, env frame
_CABLES: set = set()  # cable names; a gland "gland<suffix>" goes with the cable "cable<suffix>"


def cable_of(gland: str) -> str:
    return "cable" + gland[len("gland"):]


def arena_object(spec: Part):
    """The Arena object for a part: the box a fixed collider, the gland and the cable articulations with fixed
    roots."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    class _Gland(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "thread": ImplicitActuatorCfg(joint_names_expr=["thread"], stiffness=0.0, damping=0.0,
                                              armature=LINK_ARMATURE),
                "spin": ImplicitActuatorCfg(joint_names_expr=["spin"], stiffness=0.0, damping=0.0,
                                            armature=NUT_ARMATURE, friction=NUT_FRICTION,
                                            dynamic_friction=NUT_FRICTION),
                "seal": ImplicitActuatorCfg(joint_names_expr=["seal"], stiffness=SEAL_K, damping=SEAL_D,
                                            armature=LINK_ARMATURE, joint_effort_limit=SEAL_CLUTCH),
                "drv": ImplicitActuatorCfg(joint_names_expr=["d[0-9]"], stiffness=0.0, damping=0.0,
                                           armature=LINK_ARMATURE),
                "pads": ImplicitActuatorCfg(joint_names_expr=["p[0-9]"], stiffness=PAD_K, damping=PAD_D,
                                            armature=LINK_ARMATURE),
            }
            cfg.init_state.joint_pos = {".*": 0.0}
            return cfg

    class _Cable(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            k_b = EI / LINK_LEN
            k_t = GJ / LINK_LEN
            k_c = EI_CORES / CORE_LEN
            cfg.actuators = {
                "bend": ImplicitActuatorCfg(joint_names_expr=["b[yz][0-9]+"], stiffness=k_b, damping=BEND_D,
                                            armature=LINK_ARMATURE),
                "twist": ImplicitActuatorCfg(joint_names_expr=["tw[0-9]+"], stiffness=k_t, damping=TWIST_D,
                                             armature=LINK_ARMATURE),
                "cores": ImplicitActuatorCfg(joint_names_expr=["c(b[yz]|tw)[0-9]+"], stiffness=k_c, damping=BEND_D,
                                             armature=LINK_ARMATURE),
            }
            cfg.init_state.joint_pos = {".*": 0.0}  # as laid (the curve is in the USD)
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos),
                rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)))
    if spec.kind == "box":
        cfg = sim_utils.UsdFileCfg(usd_path=usd_file("box"))
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    if spec.kind == "gland":
        _GLANDS[spec.name] = (*spec.pos, spec.yaw)
    else:
        _CABLES.add(spec.name)
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind))
    cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
        enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=4,
        fix_root_link=True)
    cls = _Gland if spec.kind == "gland" else _Cable
    return cls(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)


# -- simulation state helpers -------------------------------------------------------------------------------


def _rot(quat_xyzw):
    import numpy as np

    x, y, z, w = quat_xyzw
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _t(x):
    return x.torch if hasattr(x, "torch") else x


def _np(x):
    import numpy as np

    return _t(x)[0].detach().cpu().numpy().astype(np.float64)


def gland_frame(gland: str):
    """(origin, rotation) of the gland frame in the env frame."""
    import numpy as np

    x, y, z, yaw = _GLANDS[gland]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def body_poses(base, name):
    """{body: (pos env frame, rotation)} of an articulation."""
    art = base.scene.articulations[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = _t(art.data.body_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    quat = _t(art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
    return {b: (pos[i], _rot(quat[i])) for i, b in enumerate(art.body_names)}


def joints(base, name) -> dict:
    art = base.scene.articulations[name]
    q = _np(art.data.joint_pos)
    return {n: float(q[i]) for i, n in enumerate(art.joint_names)}


def cable_geometry(base, gland: str):
    """In the gland frame: each link's start point and axis, the sheath's end point, the cores' tip, and the
    twist (deg) summed from the clamp to the link just outside the cap's mouth."""
    import numpy as np

    o, R = gland_frame(gland)
    poses = body_poses(base, cable_of(gland))
    q = joints(base, cable_of(gland))
    links = cable_links()
    starts, axes, ups = [], [], []
    for name, _, _, _, _ in links:
        p, r = poses[name]
        starts.append(R.T @ (p - o))
        axes.append(R.T @ r[:, 0])
        ups.append(R.T @ r[:, 2])
    starts, axes = np.array(starts), np.array(axes)
    last_sheath = SHEATH_LINKS - 1
    sheath_end = starts[last_sheath] + LINK_LEN * axes[last_sheath]
    tip = starts[-1] + CORE_LEN * axes[-1]
    mouth_x = NUT_X0 - NUT_DOME_LEN
    twist, outside = 0.0, 0
    for i, (name, _, _, _, core) in enumerate(links):
        if starts[i][0] + (CORE_LEN if core else LINK_LEN) > mouth_x - 5 * MM:
            break
        outside = i
        twist += q[f"{'c' if core else ''}tw{i}"]
    # where the centreline crosses the wall's outer plane (x = 0): through the gland if within the bore
    through = False
    for i, (_, _, length, _, _) in enumerate(links):
        a, b = starts[i], starts[i] + length * axes[i]
        if (a[0] - 0.0) * (b[0] - 0.0) <= 0.0 and abs(b[0] - a[0]) > 1e-9:
            c = a + (b - a) * (0.0 - a[0]) / (b[0] - a[0])
            through = through or math.hypot(c[1], c[2]) < BORE_R
    # the roll: how far a line drawn along the top of the sheath (the cable lay flat at the start, every link's z up)
    # has turned about the cable's axis, on the link just outside the cap's mouth: the angle from "up" (projected
    # square to the axis) to the link's z axis, about the axis. This is what a person checks on a real gland; the
    # summed twist joints miss the part a curved cable takes up by bending and lifting its loops (demo runs 15-16).
    a, u = axes[outside], np.asarray(ups[outside])
    ref = np.array([0.0, 0.0, 1.0]) - a[2] * a
    ref /= max(np.linalg.norm(ref), 1e-9)
    roll = math.atan2(float(np.dot(a, np.cross(ref, u))), float(np.dot(ref, u)))
    return {"starts": starts, "axes": axes, "sheath_end": sheath_end, "tip": tip, "twist_deg": math.degrees(twist),
            "roll_deg": math.degrees(roll), "outside": outside, "through": through}


def gland_mechanism(base, gland: str) -> dict:
    q = joints(base, gland)
    s = q["thread"]
    defl = [max(0.0, q[f"p{i}"]) for i in range(SEAL_N)]
    return {"travel": s, "turns": s / PITCH, "deflection": sum(defl) / SEAL_N, "max_deflection": max(defl),
            "min_deflection": min(defl)}


# -- fingers (for "hands off") ----------------------------------------------------------------------------------

_FINGER_PTS: dict = {}


def finger_points(body: str):
    """Points on a gripper finger's collision meshes (vertices, thinned to <= 400), in its body's frame, read from
    yam_arena's robot description."""
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
            if data[:5] == b"solid":  # ASCII STL
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


def hand_distance(base, gland: str, geo) -> float:
    """The nearest robot fingertip point to the cap nut or to the cable within HANDS_ZONE of the gland (m)."""
    import numpy as np

    o, R = gland_frame(gland)
    # targets in the env frame: points round the nut's outside, points along the cable's axis near the gland
    nut_pts = [(x, r * math.cos(t), r * math.sin(t)) for x in (NUT_X0 - NUT_DOME_LEN, NUT_X0, NUT_X0 + NUT_HEX_LEN)
               for r in (NUT_AF / 2,) for t in np.linspace(0, 2 * math.pi, 12, endpoint=False)]
    s = gland_mechanism(base, gland)["travel"]
    nut_env = np.array([R @ (np.array(p) + (s, 0.0, 0.0)) + o for p in nut_pts])
    cab = []
    for p, a in zip(geo["starts"], geo["axes"], strict=True):
        if np.linalg.norm(p) < HANDS_ZONE:
            for f in (0.0, 0.5):
                cab.append(R @ (p + f * LINK_LEN * a) + o)
    targets = [(nut_env, NUT_AF / 2 * 0.0)]
    if cab:
        targets.append((np.array(cab), CABLE_R))
    best = float("inf")
    for art_name, art in base.scene.articulations.items():
        if art_name in _GLANDS or art_name in _CABLES:
            continue
        names = [b for b in art.body_names if b.startswith("tip")]
        if not names:
            continue
        poses = body_poses(base, art_name)
        for b in names:
            p, r = poses[b]
            pts = finger_points(b) @ r.T + p
            for tg, rad in targets:
                d = np.linalg.norm(tg[:, None, :] - pts[None, :, :], axis=2).min() - rad
                best = min(best, float(d))
    return best


# -- the per-step update: seal contact, crushing, pull test -----------------------------------------------------


@dataclass
class _Track:
    contact: float | None = None
    touching: int = 0
    touching_from: float = 0.0
    crushed: bool = False
    hands_off: int = 0
    pull: str = "none"
    pull_step: int = 0
    pull_x0: float = 0.0
    pull_moved: float = 0.0
    pull_turns: float = 0.0
    pull_sheath: float = 0.0
    pull_body: int = -1
    hand_dist: float = float("inf")
    last_end: object = None  # the sheath end (gland frame) and twist at the last control step ...
    last_twist: float = 0.0
    end_vel: object = None  # ... and their rates since (m/s, rad/s)
    twist_rate: float = 0.0
    log: list = field(default_factory=list)


_TRACKS: dict[str, _Track] = {}
HANDS_ON: set = set()  # glands a bench's "virtual hand" is working on (count as hands on)
_EPISODE_STEP: list = [None]


def _set_pull(base, gland: str, force: float, body: int) -> None:
    import torch

    art = base.scene.articulations[cable_of(gland)]
    _, R = gland_frame(gland)
    f = (R @ [-force, 0.0, 0.0]).tolist()  # outward along the gland's axis
    n = len(art.body_names)
    forces = torch.zeros((1, n, 3), dtype=torch.float32, device=base.device)
    if force > 0 and body >= 0:
        forces[0, body] = torch.tensor(f, dtype=torch.float32, device=base.device)
    torques = torch.zeros_like(forces)
    art.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torques,
                                                               body_ids=list(range(n)), is_global=True)


def update(base) -> None:
    """Once per control step: follow each gland's seal and run its pull test (see the module docstring)."""
    step = getattr(base, "episode_length_buf", None)
    step = None if step is None else int(step[0])
    if step == 0 or (_EPISODE_STEP[0] is not None and step is not None and step < _EPISODE_STEP[0]):
        for gland, tr in _TRACKS.items():  # a new episode
            if tr.pull == "running":
                _set_pull(base, gland, 0.0, -1)
        _TRACKS.clear()
        _SET.clear()
        _SEAL.clear()
    elif step is not None and step == _EPISODE_STEP[0] and _TRACKS:
        return  # already updated this step (yam_arena and a demo may both ask for the states)
    _EPISODE_STEP[0] = step
    for gland in _GLANDS:
        _update_one(base, gland, step)


def _update_one(base, gland: str, step) -> None:
    tr = _TRACKS.setdefault(gland, _Track())
    dt = float(base.cfg.sim.dt * base.cfg.decimation)  # the control period
    _seal_follow(base, gland)
    _cable_set(base, cable_of(gland), dt)

    def n(seconds):
        return max(1, round(seconds / dt))

    m = gland_mechanism(base, gland)
    geo = cable_geometry(base, gland)
    # the seal: first contact (sustained), and how far the nut has turned since
    if m["min_deflection"] >= CONTACT_DEFL:
        if tr.touching == 0:
            tr.touching_from = m["turns"]
        tr.touching += 1
    else:
        tr.touching = 0
    if tr.contact is None and tr.touching >= n(CONTACT_S):
        tr.contact = tr.touching_from
    elif tr.contact is not None and m["turns"] < tr.contact - 0.05 and m["min_deflection"] < CONTACT_DEFL:
        tr.contact = None  # loosened off the cable again
    past = 0.0 if tr.contact is None else m["turns"] - tr.contact
    if tr.contact is not None and past >= CRUSH_TURNS:
        tr.crushed = True  # past the upper mark: the sheath is damaged for good
    # hands off?
    tr.hand_dist = hand_distance(base, gland, geo)
    tr.hands_off = tr.hands_off + 1 if tr.hand_dist > HANDS_OFF_DIST and gland not in HANDS_ON else 0
    sheath_x = float(geo["sheath_end"][0])
    import numpy as np

    end = np.asarray(geo["sheath_end"], dtype=float)
    if tr.last_end is not None:
        tr.end_vel = (end - tr.last_end) / dt
        tr.twist_rate = math.radians(geo["twist_deg"] - tr.last_twist) / dt
    tr.last_end, tr.last_twist = end, geo["twist_deg"]
    # a finished pull result stops counting once the nut or the cable moved
    if tr.pull in ("passed", "failed") and (abs(m["turns"] - tr.pull_turns) > PULL_INVALID_TURNS
                                            or abs(sheath_x - tr.pull_sheath) > PULL_INVALID_MOVE):
        tr.pull = "none"
    if tr.pull == "running":
        if tr.hands_off == 0:  # a hand came back: abandon the test
            _set_pull(base, gland, 0.0, -1)
            tr.pull = "none"
        else:
            tr.pull_step += 1
            tr.pull_moved = max(tr.pull_moved, tr.pull_x0 - sheath_x)
            if tr.pull_moved >= PULL_MAX_MOVE:
                _set_pull(base, gland, 0.0, -1)
                tr.pull = "failed"
            elif tr.pull_step >= n(PULL_S):
                _set_pull(base, gland, 0.0, -1)
                tr.pull = "passed"
            else:
                _set_pull(base, gland, PULL_FORCE * min(1.0, (tr.pull_step + 1) / n(PULL_RAMP_S)), tr.pull_body)
            if tr.pull != "running":
                tr.pull_turns, tr.pull_sheath = m["turns"], sheath_x
                tr.log.append((step, tr.pull, tr.pull_moved, m["turns"], past))
    elif (tr.pull == "none" and tr.hands_off >= n(HANDS_OFF_S) and tr.contact is not None
          and sheath_x > SEAL_X[1]):
        # start a pull test: on the link just outside the mouth
        art = base.scene.articulations[cable_of(gland)]
        name = cable_links()[geo["outside"]][0]
        tr.pull_body = list(art.body_names).index(name)
        tr.pull, tr.pull_step, tr.pull_x0, tr.pull_moved = "running", 0, sheath_x, 0.0
        _set_pull(base, gland, PULL_FORCE / n(PULL_RAMP_S), tr.pull_body)


_SET: dict = {}  # cable name -> (bending joint ids, their current rest angles)


def _cable_set(base, cable: str, dt: float) -> None:
    """Creep each bending spring's rest angle toward the joint's angle (SET_TAU)."""
    import numpy as np
    import torch

    art = base.scene.articulations[cable]
    names = list(art.joint_names)
    if cable not in _SET:
        ids = [i for i, n in enumerate(names) if "b" in n and "tw" not in n]  # bz*, by*, cbz*, cby*
        _SET[cable] = (ids, np.zeros(len(ids)))
    ids, rest = _SET[cable]
    q = _np(art.data.joint_pos)[ids]
    dev = q - rest
    rest += np.sign(dev) * np.maximum(np.abs(dev) - SET_BAND, 0.0) * min(1.0, dt / SET_TAU)
    art.set_joint_position_target_index(
        target=torch.tensor([rest.tolist()], dtype=torch.float32, device=base.device), joint_ids=ids)


_SEAL: dict = {}  # gland -> [the seal drive's target, the nut's angle last step]


def _seal_follow(base, gland: str) -> None:
    """The seal as a friction clutch on the nut: its drive target moves with the nut's turning, but never more than
    SEAL_CLUTCH / SEAL_K from the seal itself, so the drive (capped at SEAL_CLUTCH) drags the seal while the nut turns
    and only holds it once the nut stops. (A target at the nut's angle kept twisting the cable after the hands let
    go, demo run 13.)"""
    import torch

    art = base.scene.articulations[gland]
    names = list(art.joint_names)
    q = _np(art.data.joint_pos)
    seal, spin = float(q[names.index("seal")]), float(q[names.index("spin")])
    tgt, last = _SEAL.get(gland, [seal, spin])
    lag = SEAL_CLUTCH / SEAL_K
    tgt = seal + min(lag, max(-lag, tgt + (spin - last) - seal))
    _SEAL[gland] = [tgt, spin]
    art.set_joint_position_target_index(
        target=torch.tensor([[tgt]], dtype=torch.float32, device=base.device), joint_ids=[names.index("seal")])


def reset_parts(base) -> None:
    """Every gland and cable back to its start: joints, velocities and drive targets at zero (as built), no pull,
    the seal and set trackers cleared."""
    import torch

    for name in [*_GLANDS, *_CABLES]:
        art = base.scene.articulations[name]
        n = len(art.joint_names)
        zero = torch.zeros((1, n), dtype=torch.float32, device=base.device)
        art.write_joint_position_to_sim_index(position=zero, env_ids=[0])
        art.write_joint_velocity_to_sim_index(velocity=zero, env_ids=[0])
        art.set_joint_position_target_index(target=zero, joint_ids=list(range(n)))
        art.permanent_wrench_composer.reset()
    _TRACKS.clear()
    _SET.clear()
    _SEAL.clear()
    _EPISODE_STEP[0] = None


def gland_state(base, gland: str) -> GlandState:
    import numpy as np

    tr = _TRACKS.get(gland, _Track())
    m = gland_mechanism(base, gland)
    poses = body_poses(base, gland)
    p, r = poses["nut"]
    lo, hi = bounds("gland")
    o, R = gland_frame(gland)
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ R.T + o
    art = base.scene.articulations[gland]
    i = list(art.body_names).index("nut")
    ang = _t(art.data.body_ang_vel_w)[0, i].detach().cpu().numpy().astype(float)
    quat = _t(art.data.body_quat_w)[0, i].detach().cpu().numpy().astype(float)
    past = 0.0 if tr.contact is None else m["turns"] - tr.contact
    return GlandState(pos=p, quat_xyzw=quat, lin_vel=np.zeros(3), ang_vel=ang, low=corners.min(axis=0),
                      high=corners.max(axis=0), turns=m["turns"], contact_turns=tr.contact, past_contact=past,
                      pad_deflection=m["deflection"], crushed=tr.crushed, hands_off_steps=tr.hands_off,
                      pull=tr.pull, pull_moved=tr.pull_moved, pull_sheath_x=tr.pull_x0)


def cable_state(base, cable: str) -> CableState:
    import numpy as np

    gland = "gland" + cable[len("cable"):]
    geo = cable_geometry(base, gland)
    o, R = gland_frame(gland)
    art = base.scene.articulations[cable]
    names = list(art.body_names)
    i = names.index(f"link{SHEATH_LINKS - 1}")
    lin = _t(art.data.body_lin_vel_w)[0].detach().cpu().numpy().astype(float)
    ang = _t(art.data.body_ang_vel_w)[0].detach().cpu().numpy().astype(float)
    quat = _t(art.data.body_quat_w)[0, i].detach().cpu().numpy().astype(float)
    pts = np.array([R @ p + o for p in geo["starts"]])
    end = geo["sheath_end"]
    # its motion: the sheath end's displacement and the twist's change over the last control step (instantaneous link
    # velocities jitter at ~4 cm/s in the seal's grip with nothing moving, so the cable never counted as still,
    # demo runs 15-16)
    tr = _TRACKS.get(gland)
    v = R @ tr.end_vel if tr is not None and tr.end_vel is not None else lin[i]
    w = R @ np.array([tr.twist_rate, 0.0, 0.0]) if tr is not None and tr.end_vel is not None else ang[i]
    return CableState(pos=R @ end + o, quat_xyzw=quat, lin_vel=v, ang_vel=w, low=pts.min(axis=0),
                      high=pts.max(axis=0), through=bool(geo["through"]), sheath_x=float(end[0]), sheath_off_axis=float(math.hypot(end[1], end[2])),
                      twist_deg=geo["twist_deg"], roll_deg=geo["roll_deg"], tip_x=float(geo["tip"][0]))


# -- the hook into yam_arena.scene --------------------------------------------------------------------------


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "cg_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_cg_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        base = env.unwrapped
        update(base)
        out = original_states(env, [n for n in names if n not in _GLANDS and n not in _CABLES])
        for n in names:
            if n in _GLANDS:
                out[n] = gland_state(base, n)
            elif n in _CABLES:
                out[n] = cable_state(base, n)
        return {n: out[n] for n in names}

    original_make_env = getattr(scene_module, "make_env", None)

    def make_env(*args, **kwargs):
        """yam_arena's env, its reset also putting the gland and the cable back (yam_arena resets the robot and the
        free objects, not these jointed parts: the next episode started with the nut still turned, demo run 12)."""
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
    task_object._spawns_cg_parts = True
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
