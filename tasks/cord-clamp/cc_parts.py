"""The cord-clamp task's own parts: a lidless terminal box with a cord clamp (strain relief) inside its cable entry,
a control cable whose cores are already connected to the box's terminal block, and a hex-key screwdriver in a
holder. All geometry is our own simplified design (screws after ISO 4762 M4, key after ISO 2936, cable sized as a
3 x 1.5 mm2 control cable); no manufacturer CAD is used.

yam_arena task files support single primitives and library assets only, so this module is the task's own minimal
mechanism (the approach of our earlier tasks, copied so this folder stands alone):

- :class:`Part` is a :class:`yam_arena.task_file.Box` that also names which part it is.
- :func:`install` wraps ``yam_arena.scene.task_object``, ``object_states`` and ``make_env`` when that module is
  imported.

Parts (mm in comments, metres in code). The box frame: origin at the centre of the box's footprint on the table,
x forward (away from the arms), y left, z up. The cable enters through the box's right side wall (-y).

- box (fixed): cast-aluminium-grey terminal box, 150 (x) x 120 (y) x 60 tall, 4 mm walls and floor, lid off; a
  12 x 12 mm entry in its right wall (cable centre 18 mm above the table); inside, 26 mm from that wall, the clamp's
  saddle (36 x 10 mm, its top 14 mm up: the cable lies on it) and, near the far side, a terminal block.
- clamp (world-fixed articulation, its frame on the saddle's centre at table height): two M4 x 16 socket cap screws
  (head 7 mm across, 4 high, 3 mm hex socket), 22 mm apart along x, already started in the saddle's tapped holes.
  Each: a prismatic "thread" joint coupled to the screw's turning by a PhysX mimic joint (0.7 mm per turn), a
  compliant "clamp" joint under the head (the cable's and the bar's give), the screw's "spin" (joint friction).
- bar (free): the clamp bar, steel, 32 x 8 x 3 mm, two 4.6 mm holes on the screws' shanks; it rests on the cable.
  The screw heads press it down by contact; it presses the cable onto the saddle.
- cable (articulation, its root fixed at the terminal block): the cores first (connected, taped together into one
  bundle, r 3.2, a little slack), then the 8 mm PVC sheath running out through the entry and lying on the table
  outside. Links of LINK_LEN joined by two bending joints and a twisting joint each (springs); the laid shape is
  built into the USD; the sheath's bends take a set (SET_TAU, SET_BAND).
- driver (free): a stubby screwdriver with a 3 mm hexagon blade; holder (free, heavy): a block it stands in.

Per control step (from the patched ``object_states``, before scoring) :func:`update` follows each screw (when its
head first presses, its turns since, crushing), whether the bar pressed while the cores were under it, and runs the
pull test (hands off 1 s: the cable outside the box pulled outward, 30 N; the cable at the clamp must move < 1 mm).
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

# -- terminal box ---------------------------------------------------------------------------------------------------
BOX_X, BOX_Y, BOX_Z = 150 * MM, 155 * MM, 60 * MM  # (120 wide with the clamp 22 mm inside the entry: holding the
# cable at the entry, that hand met the screwdriver hand over the box, demo runs 56-61)
WALL = 4.0 * MM
CABLE_Z = 18 * MM  # the cable's centre at the entry and over the saddle, above the table
ENTRY_HALF = 6.0 * MM  # the entry: 12 x 12 mm
CLAMP_INSIDE = 57 * MM  # the clamp's centre line this far inside the right wall's inner face
SADDLE_Y = -BOX_Y / 2 + WALL + CLAMP_INSIDE  # the clamp's centre line (box frame y): 61 mm inside the wall's outside
SADDLE_HALF = (18 * MM, 5 * MM)  # x, y
CABLE_R = 4.0 * MM
SADDLE_TOP = CABLE_Z - CABLE_R  # 14 mm
TERMINAL_Y = 55.5 * MM  # the terminal block's face (the cores end here)
TERMINAL_SIZE = (40 * MM, 14 * MM, 22 * MM)

# -- clamp: screws, bar ---------------------------------------------------------------------------------------------
SCREW_DX = 11 * MM  # the screws at x = +-SCREW_DX (clamp frame)
PITCH = 0.7 * MM  # M4 coarse
SHANK_R = 2.0 * MM
SCREW_LEN = 16 * MM
HEAD_R = 3.5 * MM
HEAD_H = 4.0 * MM
SHANK_FREE = 3.5 * MM  # the screw's travel down from the start that its shank collider allows for
SOCKET_AF = 3.0 * MM
SOCKET_DEPTH = 2.0 * MM
SOCKET_CHAMFER = 0.3 * MM
BEARING_N, BEARING_RING, BEARING_R = 8, 3.0 * MM, 0.4 * MM  # the head bears on small balls (a flat underside over the
# bar's hole edges sank in, as in rail-plate-mount)
BAR_SIZE = (32 * MM, 8 * MM, 3 * MM)
BAR_HOLE = 2.3 * MM  # hole radius (the shank's 2.0 slides in it)
BAR_MASS = 0.06  # kg (simulation: a real 32 x 8 x 3 mm steel bar is ~6 g; that light, squeezed between the screws and
# the cable it sank 1.4 mm into the cable instead of loading the heads' give, bench 8)
BAR_Z0 = CABLE_Z + CABLE_R  # the bar's underside at the start: resting on the sheath (22 mm)
HEAD_GAP = 1.0 * MM  # the heads' undersides this far above the bar's top at the start (about 1.5 turns to contact;
# 2 mm made the demo slow, demo run 3; 1 mm jammed in bench 3 only while the cores were still stiff)
HEAD_Z0 = BAR_Z0 + BAR_SIZE[2] + HEAD_GAP  # 26 mm
THREAD_RANGE = 8.0 * MM  # the thread slider's travel down from the start (and 2 mm up)
CLAMP_K = 6.0e4  # N/m: the give under each head (the PVC sheath's and the bar's compliance; at 40 N/mm with a
# smooth bar the cable slipped under 30 N, bench 4; at 150 N/mm a quarter turn needed ~26 N, more than the robot's
# grip on the key gives, demo runs 8-11 stalled at ~16 N)
CLAMP_D = 20.0
CLAMP_TRAVEL = 2.5 * MM  # hard stop: the screw can't be turned further
FREE_TORQUE = 0.01  # PhysX joint friction coefficient on the spin (scales with the joint's load, like a thread's
# torque with its clamp load; at 0.03 the key's grip slipped on the handle once a head pressed ~6 N, demo run 8)
SCREW_ARMATURE = 2.0e-4  # kg m^2: a screwdriver's and a hand's share (as in rail-plate-mount)
LINK_ARMATURE = 1.0e-4
SMALL_MASS = 0.001

# -- cable (3 x 1.5 mm2 control cable, PVC sheath) -------------------------------------------------------------------
LINK_LEN = 20 * MM
CORE_LINKS = 5  # the connected cores: 80 mm from the terminal to the sheath's cut end, laid in a bow (slack)
CORE_LEN = 16 * MM
CORE_R = 3.2 * MM
SHEATH_LINKS = 16  # 320 mm of sheath
SHEATH_MASS = 0.002
CORE_MASS = 0.0012
GIMBAL_MASS = 0.0002
EI = 6.0e-3  # N m^2 (our estimate)
GJ = 4.0e-3
EI_CORES = 9.0e-4  # the connected cores: copper, bowed: pushed in, the bow pushes the cable back out unless it is held
# or clamped (let go after a push to 18-25 mm past the bar's middle, held 0.5-20 s, it settles at 3-5 mm, short of
# the goal's 9: benches 30-31; at 6e-4 held 20 s it stayed at 10, at 3e-4 11.5)
BEND_D = 3.0e-3
TWIST_D = 3.0e-3
BEND_LIMIT = 45.0  # deg per link
TWIST_LIMIT = 720.0
SET_TAU = 1.5  # s: the cable's set creeps (each bending spring's rest angle follows the joint's angle) ...
SET_BAND = math.radians(6.0)  # ... beyond this elastic band (as in cable-gland)
SET_CORES = False  # the connected cores' bow stays elastic (True: it takes a set like the sheath's, and a cable let go
# stays where it was pushed, bench 20)

# -- screwdriver and holder -----------------------------------------------------------------------------------------
KEY_AF = 2.9 * MM  # simulation: 0.05 mm play each side in the 3.0 socket
KEY_TIP_CHAMFER = 0.25 * MM
BLADE_LENGTH = 9.0 * MM
SHAFT_R = 2.5 * MM
SHAFT_TOP = 24.0 * MM
COLLAR_R = 6.0 * MM
COLLAR_TOP = 28.0 * MM
HANDLE_R = 12.0 * MM
HANDLE_TOP = 80.0 * MM
DRIVER_MASS = 0.03
HOLDER_SIZE = (40 * MM, 40 * MM, 30 * MM)
HOLDER_HOLE_HALF = 2.9 * MM  # the shaft (r 2.5) in it, the collar on the block
HOLDER_MASS = 0.4

# -- scoring constants (used by the task) ----------------------------------------------------------------------------
SHEATH_PAST = 5.0 * MM  # the sheath's cut end at least this past the bar (its edge on the terminals' side)
CONTACT_CLAMP = 2.0  # N: a head counts as pressing (its give takes up this much) ...
CONTACT_S = 0.5  # ... for this long in a row
SNUG_TURNS = (0.25, 1.75)  # a screw is snug turned this far past its head's first contact (0.25 to under 1.75) ...
CRUSH_TURNS = 2.0  # ... and crushes the cable from this far (latched)
CORES_CLAMP = 3.0  # N: either head pressing this hard while the sheath's end isn't past the bar: the cores clamped
TILT_MAX_DEG = 2.0  # the bar parallel: tilted less than this
IN_TURNS_MAX = 0.25  # tightened in turns: the two screws' turns past their first contact differ by less than this
PULL_FORCE = 30.0
PULL_MAX_MOVE = 1.0 * MM  # at the clamp
PULL_RAMP_S = 1.0
PULL_S = 1.5
HANDS_OFF_DIST = 60 * MM  # no fingertip or screwdriver tip this near the heads / bar / cable (a key lifted between the
# screws is still "on": with 20 mm the pull test started mid-tightening, demo run 4)
HANDS_OFF_S = 1.0
HANDS_ZONE = 80 * MM  # cable links within this of the clamp count for "hands off"
PULL_INVALID_TURNS = 0.02
PULL_INVALID_MOVE = 1.0 * MM

# -- materials and looks ---------------------------------------------------------------------------------------------
MATERIALS = {  # (static friction, dynamic friction, combine mode)
    "alu": (0.40, 0.35, "average"),
    "steel": (0.40, 0.35, "average"),
    "ribbed": (0.80, 0.70, "max"),  # the bar's underside and the saddle's top are ribbed (as on real cord clamps; smooth,
    # the cable slipped 1.04 mm under 30 N, bench 5)
    "screw": (0.20, 0.20, "average"),
    "pvc": (0.50, 0.45, "average"),
    "handle": (0.90, 0.80, "max"),  # a rubbery screwdriver handle: grips the fingers at its own friction
}
LOOKS = {  # sRGB, roughness, metallic
    "box": ((0.70, 0.71, 0.72), 0.45, 0.6),  # cast aluminium
    "terminal": ((0.15, 0.35, 0.65), 0.6, 0.0),  # a blue terminal block
    "steel": ((0.72, 0.74, 0.76), 0.35, 0.8),
    "zinc": ((0.80, 0.80, 0.78), 0.3, 0.9),
    "black": ((0.08, 0.08, 0.09), 0.7, 0.0),
    "sheath": ((0.55, 0.56, 0.57), 0.6, 0.0),
    "core_bn": ((0.45, 0.25, 0.12), 0.5, 0.0),
    "core_bu": ((0.15, 0.30, 0.70), 0.5, 0.0),
    "core_gy": ((0.30, 0.65, 0.20), 0.5, 0.0),
    "tape": ((0.08, 0.08, 0.08), 0.8, 0.0),
    "handle": ((0.85, 0.12, 0.10), 0.6, 0.0),
    "hole": ((0.03, 0.03, 0.035), 0.9, 0.0),
}
CONTACT_OFFSET = 0.5 * MM


# -- shapes -------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Shape:
    """One piece of a part, in its body's frame: a box (``center``, ``size``), a prism (a 2-D ``profile`` (x, y)
    extruded along z over ``span``), a cylinder/capsule/sphere (``center``, ``radius``, ``length`` along ``axis``;
    axis "S" = sphere) or a convex ``hull`` of points (a collider; drawn only if ``visible``)."""

    body: str
    look: str
    material: str | None  # None: visual only
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)
    size: tuple[float, float, float] | None = None
    profile: tuple[tuple[float, float], ...] | None = None
    span: tuple[float, float] = (0.0, 0.0)
    radius: float = 0.0
    length: float = 0.0
    axis: str = "Z"
    capsule: bool = False
    visible: bool = True
    hull: tuple | None = None

    def points(self):
        if self.hull is not None:
            return list(self.hull)
        if self.profile is not None:
            return [(p[0], p[1], s) for p in self.profile for s in self.span]
        if self.size is not None:
            return [tuple(c + s * h / 2 for c, h, s in zip(self.center, self.size, sg, strict=True))
                    for sg in ((-1, -1, -1), (1, 1, 1))]
        r, h = self.radius, self.length / 2 + (self.radius if self.capsule else 0.0)
        ext = {"X": (h, r, r), "Y": (r, h, r), "Z": (r, r, h), "S": (r, r, r)}[self.axis]
        return [tuple(c + s * e for c, e in zip(self.center, ext, strict=True)) for s in (-1, 1)]


def _box(body, look, material, x0, x1, y0, y1, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0),
                 visible=visible)


def _prism(body, look, material, profile, z0, z1, visible=True) -> Shape:
    return Shape(body, look, material, profile=tuple((float(a), float(b)) for a, b in profile), span=(z0, z1),
                 visible=visible)


def _ngon(r, n, a0=0.0, a1=2 * math.pi):
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    k = n if full else n + 1
    return [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(k)]


def _hex(af: float, start: float = 0.0):
    """A hexagon's corners (across flats ``af``), the first at angle ``start``: with start 0 a corner points along +x
    and two flats face +-y."""
    rc = af / 2 / math.cos(math.pi / 6)
    return [(rc * math.cos(start + k * math.pi / 3), rc * math.sin(start + k * math.pi / 3)) for k in range(6)]


# -- the terminal box --------------------------------------------------------------------------------------------------


def box_shapes() -> list[Shape]:
    """Floor, walls (the right one with the entry), the clamp's saddle and the terminal block (all collide)."""
    hx, hy, t, e = BOX_X / 2, BOX_Y / 2, WALL, ENTRY_HALF
    out = [
        _box("box", "box", "alu", -hx, hx, -hy, hy, 0.0, t),  # floor
        _box("box", "box", "alu", -hx, -hx + t, -hy, hy, t, BOX_Z),  # near wall (toward the arms)
        _box("box", "box", "alu", hx - t, hx, -hy, hy, t, BOX_Z),  # far wall
        _box("box", "box", "alu", -hx + t, hx - t, hy - t, hy, t, BOX_Z),  # left wall
        # the right wall round the entry
        _box("box", "box", "alu", -hx + t, -e, -hy, -hy + t, t, BOX_Z),
        _box("box", "box", "alu", e, hx - t, -hy, -hy + t, t, BOX_Z),
        _box("box", "box", "alu", -e, e, -hy, -hy + t, t, CABLE_Z - e),
        _box("box", "box", "alu", -e, e, -hy, -hy + t, CABLE_Z + e, BOX_Z),
        # the clamp's saddle (the screws thread into it)
        _box("box", "box", "ribbed", -SADDLE_HALF[0], SADDLE_HALF[0], SADDLE_Y - SADDLE_HALF[1],
             SADDLE_Y + SADDLE_HALF[1], t, SADDLE_TOP),
        # the terminal block (blue), the cores end at its face
        _box("box", "terminal", "alu", -TERMINAL_SIZE[0] / 2, TERMINAL_SIZE[0] / 2, TERMINAL_Y,
             TERMINAL_Y + TERMINAL_SIZE[1], t, t + TERMINAL_SIZE[2]),
    ]
    for dx in (-SCREW_DX, SCREW_DX):  # the tapped holes (drawn)
        out.append(Shape("box", "hole", None, (dx, SADDLE_Y, SADDLE_TOP + 0.05 * MM), radius=SHANK_R,
                         length=0.1 * MM))
    for k in (-1, 0, 1):  # the terminals' screws (drawn)
        out.append(Shape("box", "zinc", None, (k * 12 * MM, TERMINAL_Y + TERMINAL_SIZE[1] / 2,
                                               t + TERMINAL_SIZE[2] + 0.5 * MM), radius=2.5 * MM, length=1.0 * MM))
    return out


# -- the clamp: two threaded screws (an articulation) and the loose bar -----------------------------------------------

SCREWS = ("s0", "s1")  # at x = -SCREW_DX, +SCREW_DX


def screw_local_shapes(body: str) -> list[Shape]:
    """A screw in its own frame (origin: the head's underside on its axis, +z up): shank, bearing balls, the head and
    its hexagon socket (six convex pieces between the socket's flats and the head's rim, with an entry chamfer)."""
    # the shank: drawn its whole length; it collides only above the saddle (it guides the bar). Inside the saddle's
    # tapped hole it doesn't (the saddle is solid in the simulation: a full-length shank collider was pushed out of
    # it, 112 N on the bar at the start, preview 1)
    above = HEAD_Z0 - SADDLE_TOP - SHANK_FREE  # the shank's length above the saddle when turned SHANK_FREE in
    out = [_prism(body, "zinc", None, _ngon(SHANK_R, 16), -SCREW_LEN, 0.0),
           _prism(body, "zinc", "screw", _ngon(SHANK_R, 16), -above, 0.0, visible=False)]
    for k in range(BEARING_N):
        a = 2 * math.pi * (k + 0.5) / BEARING_N
        out.append(Shape(body, "zinc", "screw", (BEARING_RING * math.cos(a), BEARING_RING * math.sin(a), BEARING_R),
                         radius=BEARING_R, axis="S", visible=False))
    zs = HEAD_H - SOCKET_DEPTH
    out.append(_prism(body, "black", None, _ngon(HEAD_R, 24), 0.0, zs))  # the look (black oxide)
    out.append(_prism(body, "black", "screw", _ngon(HEAD_R, 24), 0.3 * MM, zs, visible=False))
    out.append(_prism(body, "hole", None, _hex(SOCKET_AF), zs, zs + 0.05 * MM))  # the socket's floor
    inner, mouth, c = _hex(SOCKET_AF), _hex(SOCKET_AF + 2 * SOCKET_CHAMFER), SOCKET_CHAMFER
    for k in range(6):
        a0 = math.atan2(inner[k][1], inner[k][0])
        rim = _ngon(HEAD_R, 4, a0=a0, a1=a0 + math.pi / 3)
        sector = [inner[k]] + rim + [inner[(k + 1) % 6]]
        out.append(_prism(body, "black", None, sector, zs, HEAD_H))
        pts = ([(x, y, zs) for x, y in sector] + [(x, y, HEAD_H) for x, y in rim]
               + [(x, y, HEAD_H - c) for x, y in (inner[k], inner[(k + 1) % 6])]
               + [(x, y, HEAD_H) for x, y in (mouth[k], mouth[(k + 1) % 6])])
        out.append(Shape(body, "black", "screw", hull=tuple(pts), visible=False))
    return out


def clamp_frames() -> dict:
    """{body: (x, y, z)} frames in the clamp frame (the saddle's centre at table height)."""
    out = {"base": (0.0, 0.0, 0.0)}
    for s, dx in zip(SCREWS, (-SCREW_DX, SCREW_DX), strict=True):
        for b in (f"{s}_carrier", f"{s}_clamp", s):
            out[b] = (dx, 0.0, HEAD_Z0)
    return out


def clamp_shapes() -> list[Shape]:
    """Each screw's shapes, moved to its frame (the writer subtracts the body frame again)."""
    frames = clamp_frames()
    out = []
    for s in SCREWS:
        fx, fy, fz = frames[s]
        for sh in screw_local_shapes(s):
            if sh.hull is not None:
                out.append(Shape(sh.body, sh.look, sh.material, hull=tuple((x + fx, y + fy, z + fz)
                                                                           for x, y, z in sh.hull),
                                 visible=sh.visible))
            elif sh.profile is not None:
                out.append(Shape(sh.body, sh.look, sh.material,
                                 profile=tuple((x + fx, y + fy) for x, y in sh.profile),
                                 span=(sh.span[0] + fz, sh.span[1] + fz), visible=sh.visible))
            else:
                cx, cy, cz = sh.center
                out.append(Shape(sh.body, sh.look, sh.material, (cx + fx, cy + fy, cz + fz), radius=sh.radius,
                                 axis=sh.axis, visible=sh.visible))
    return out


CLAMP_BODIES = ("base", *(f"{s}_{p}" for s in SCREWS for p in ("carrier", "clamp")), *SCREWS)


def clamp_joints():
    """name: (type, parent, child, axis, lower, upper, stiffness, damping, armature) (prismatic in m, revolute in
    deg). Thread: down is negative; clamp: the head pushed up against its give is positive."""
    turns = (THREAD_RANGE + 2 * MM) / PITCH + 1
    out = {}
    for s in SCREWS:
        out[f"{s}_thread"] = ("pri", "base", f"{s}_carrier", "Z", -THREAD_RANGE, 2.0 * MM, 0.0, 0.0, LINK_ARMATURE)
        out[f"{s}_give"] = ("pri", f"{s}_carrier", f"{s}_clamp", "Z", 0.0, CLAMP_TRAVEL, CLAMP_K, CLAMP_D,
                            LINK_ARMATURE)
        out[f"{s}_spin"] = ("rev", f"{s}_clamp", s, "Z", -360.0 * turns, 360.0 * turns, 0.0, 0.0, SCREW_ARMATURE)
    return out


# spin (deg) + gearing * thread (m) = 0: clockwise seen from above (negative about +z) drives the screw down
MIMICS = {f"{s}_spin": (f"{s}_thread", -360.0 / PITCH) for s in SCREWS}


def clamp_masses():
    out = {"base": (0.05, (0.0, 0.0, 0.0), (1e-5, 1e-5, 1e-5))}
    frames = clamp_frames()
    for s in SCREWS:
        f = frames[s]
        out[f"{s}_carrier"] = (SMALL_MASS, f, (1e-7, 1e-7, 1e-7))
        out[f"{s}_clamp"] = (SMALL_MASS, f, (1e-7, 1e-7, 1e-7))
        out[s] = (0.003, (f[0], f[1], f[2] - 2 * MM), (5e-7, 5e-7, 5e-7))
    return out


def bar_shapes() -> list[Shape]:
    """The bar (origin: the centre of its underside): a plate with a hole for each screw, as convex pieces round the
    holes (it slides on the shanks)."""
    hx, hy, hz = BAR_SIZE[0] / 2, BAR_SIZE[1] / 2, BAR_SIZE[2]
    h = BAR_HOLE
    out = [_box("bar", "steel", "ribbed", -SCREW_DX + h, SCREW_DX - h, -hy, hy, 0.0, hz)]  # (ribbed underneath)
    for dx in (-SCREW_DX, SCREW_DX):
        sgn = 1 if dx > 0 else -1
        out.append(_box("bar", "steel", "steel", min(dx + sgn * h, sgn * hx), max(dx + sgn * h, sgn * hx), -hy, hy,
                        0.0, hz))
        out += [_box("bar", "steel", "steel", dx - h, dx + h, -hy, -h, 0.0, hz),
                _box("bar", "steel", "steel", dx - h, dx + h, h, hy, 0.0, hz)]
        out.append(Shape("bar", "hole", None, (dx, 0.0, hz / 2), radius=h * 0.98, length=hz + 0.1 * MM))
    return out


# -- the cable (root at the terminal block) ----------------------------------------------------------------------------


def cable_links():
    """(name, start, length, radius, is core) from the terminal: the cores, then the sheath (cable frame: the
    terminal at the origin, the cable laid straight along +x)."""
    out, x = [], 0.0
    for i in range(CORE_LINKS):
        out.append((f"core{i}", x, CORE_LEN, CORE_R, True))
        x += CORE_LEN
    for i in range(SHEATH_LINKS):
        out.append((f"link{i}", x, LINK_LEN, CABLE_R, False))
        x += LINK_LEN
    return out


CABLE_HEADINGS: list = []  # per link (yaw, pitch) in degrees relative to the root (pitch + = down), set by the task


def _rot_yp(yaw: float, pitch: float):
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    return [[cy * cp, -sy, cy * sp], [sy * cp, cy, sy * sp], [-sp, 0.0, cp]]


def _yaw_pitch(i: int):
    if not CABLE_HEADINGS:
        return 0.0, 0.0
    h = CABLE_HEADINGS[i]
    return (math.radians(h[0]), math.radians(h[1])) if isinstance(h, (tuple, list)) else (math.radians(h), 0.0)


def cable_frames() -> dict:
    """{body: ((x, y, z), rotation)}: each body's frame in the cable frame, as laid."""
    out = {"root": ((0.0, 0.0, 0.0), _rot_yp(0.0, 0.0))}
    x, y, z = 0.0, 0.0, 0.0
    for i, (name, _, length, _, _) in enumerate(cable_links()):
        r = _rot_yp(*_yaw_pitch(i))
        for b in (f"ga{i}", f"gb{i}", name):
            out[b] = ((x, y, z), r)
        x, y, z = x + length * r[0][0], y + length * r[1][0], z + length * r[2][0]
    return out


def laid_points(origin, yaw: float, headings):
    """Each link's start point, the sheath's cut end and the cable's free end (env frame) for a cable laid with
    ``headings`` from a root at ``origin`` turned ``yaw``."""
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
    end = world(tuple(last[0][k] + links[-1][2] * last[1][k][0] for k in range(3)))
    return pts, world(frames[links[CORE_LINKS][0]][0]), end


def cable_bodies():
    return [(b, p) for b, (p, _) in cable_frames().items()]


def cable_joints():
    out, parent = {}, "root"
    for i, (name, _, length, _, core) in enumerate(cable_links()):
        ei = EI_CORES if core else EI
        k_b = math.radians(1.0) * ei / length
        k_t = math.radians(1.0) * (EI_CORES if core else GJ) / length
        pre = "c" if core else ""
        out[f"{pre}bz{i}"] = ("rev", parent, f"ga{i}", "Z", -BEND_LIMIT, BEND_LIMIT, k_b, BEND_D, LINK_ARMATURE)
        out[f"{pre}by{i}"] = ("rev", f"ga{i}", f"gb{i}", "Y", -BEND_LIMIT, BEND_LIMIT, k_b, BEND_D, LINK_ARMATURE)
        out[f"{pre}tw{i}"] = ("rev", f"gb{i}", name, "X", -TWIST_LIMIT, TWIST_LIMIT, k_t, TWIST_D, LINK_ARMATURE)
        parent = name
    return out


def cable_shapes() -> list[Shape]:
    """In each link's own frame: the cores (one capsule collider, three coloured cores drawn, a terminal ferrule at
    the root), the sheath links (capsules)."""
    out = [Shape("root", "zinc", None, (0.0, 0.0, 0.0), radius=2.0 * MM, length=6 * MM, axis="X")]
    for name, _, length, r, core in cable_links():
        if not core:
            out.append(Shape(name, "sheath", "pvc", (length / 2, 0.0, 0.0), radius=r, length=length, axis="X",
                             capsule=True))
            continue
        out.append(Shape(name, "tape", "pvc", (length / 2, 0.0, 0.0), radius=r, length=length, axis="X",
                         capsule=True, visible=False))
        for k, look in enumerate(("core_bn", "core_bu", "core_gy")):
            t = 2 * math.pi * k / 3 + math.pi / 2
            out.append(Shape(name, look, None, (length / 2, 1.6 * MM * math.cos(t), 1.6 * MM * math.sin(t)),
                             radius=1.5 * MM, length=length + 2.0 * MM, axis="X"))
    return out


def cable_masses():
    out = {"root": (0.01, (0.0, 0.0, 0.0), (1e-6, 1e-6, 1e-6))}
    for i, (name, _, length, r, core) in enumerate(cable_links()):
        m = CORE_MASS if core else SHEATH_MASS
        i_t = m * (length ** 2 / 12 + r ** 2 / 4)
        out[name] = (m, (length / 2, 0.0, 0.0), (m * r ** 2 / 2, i_t, i_t))
        out[f"ga{i}"] = (GIMBAL_MASS, (0.0, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
        out[f"gb{i}"] = (GIMBAL_MASS, (0.0, 0.0, 0.0), (1e-8, 1e-8, 1e-8))
    return out


# -- screwdriver and holder --------------------------------------------------------------------------------------------


def driver_shapes() -> list[Shape]:
    """Blade tip at the origin, the shaft along +z: hexagon blade (flats facing +-y) with a chamfered tip, round
    shaft, collar, red handle."""
    key, tip, c = _hex(KEY_AF), _hex(KEY_AF - 2 * KEY_TIP_CHAMFER), KEY_TIP_CHAMFER
    return [Shape("driver", "steel", "steel", hull=tuple([(x, y, 0.0) for x, y in tip] + [(x, y, c) for x, y in key]),
                  visible=False),
            _prism("driver", "steel", None, tip, 0.0, c),
            _prism("driver", "steel", "steel", key, c, BLADE_LENGTH),
            _prism("driver", "steel", "steel", _ngon(SHAFT_R, 12), BLADE_LENGTH, SHAFT_TOP),
            _prism("driver", "black", "handle", _ngon(COLLAR_R, 16), SHAFT_TOP, COLLAR_TOP),
            _prism("driver", "handle", "handle", _ngon(HANDLE_R, 16), COLLAR_TOP, HANDLE_TOP - 4 * MM),
            _prism("driver", "black", "handle", _ngon(HANDLE_R - 2 * MM, 16), HANDLE_TOP - 4 * MM, HANDLE_TOP)]


def holder_shapes() -> list[Shape]:
    """A block (origin: the centre of its top face) with a square hole for the screwdriver's shaft."""
    hx, hy, hz = HOLDER_SIZE[0] / 2, HOLDER_SIZE[1] / 2, HOLDER_SIZE[2]
    h = HOLDER_HOLE_HALF
    return [_box("holder", "steel", "steel", -hx, hx, -hy, -h, -hz, 0.0),
            _box("holder", "steel", "steel", -hx, hx, h, hy, -hz, 0.0),
            _box("holder", "steel", "steel", -hx, -h, -h, h, -hz, 0.0),
            _box("holder", "steel", "steel", h, hx, -h, h, -hz, 0.0),
            Shape("holder", "hole", None, (0.0, 0.0, -hz / 2 - 0.05 * MM), radius=h * 0.98, length=hz)]


# -- bookkeeping -------------------------------------------------------------------------------------------------------

SHAPES = {"box": box_shapes, "clamp": clamp_shapes, "bar": bar_shapes, "cable": cable_shapes,
          "driver": driver_shapes, "holder": holder_shapes}
BODIES = {"box": ("box",), "clamp": CLAMP_BODIES, "bar": ("bar",), "cable": tuple(b for b, _ in cable_bodies()),
          "driver": ("driver",), "holder": ("holder",)}
FREE_MASSES = {"bar": BAR_MASS, "driver": DRIVER_MASS, "holder": HOLDER_MASS}


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


# -- task-file objects ---------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Part(task_file.Box):
    """One of this module's parts as a task-file object: ``pos`` is the part's origin, ``yaw`` its turn about z."""

    kind: str = "box"

    def _check(self) -> None:
        super()._check()
        if self.kind not in SHAPES:
            raise ValueError(f"{self.name}: unknown part kind {self.kind!r}")


def part(name, kind, pos, yaw=0.0) -> Part:
    lo, hi = bounds(kind)
    size = tuple(max(b - a, 1e-3) for a, b in zip(lo, hi, strict=True))
    color = {"box": LOOKS["box"][0], "clamp": LOOKS["black"][0], "bar": LOOKS["steel"][0],
             "cable": LOOKS["sheath"][0], "driver": LOOKS["handle"][0], "holder": LOOKS["steel"][0]}[kind]
    # the box is fixed; the clamp and the cable are articulations with fixed roots (a mass marks them as moving for
    # yam_arena's checks; their USD sets the real masses); bar, driver and holder are free rigid bodies
    mass = None if kind == "box" else FREE_MASSES.get(kind, 0.1)
    return Part(name=name, pos=tuple(pos), size=size, mass=mass, color=color, yaw=yaw, kind=kind)


@dataclass(frozen=True)
class ClampState(task_file.ObjectState):
    """The clamp: per screw (s0, s1) its turns since the start, turns past its head's first contact (0 before), the
    force under its head (N); the latches (crushed, cores clamped); the latest pull test."""

    turns: tuple = (0.0, 0.0)
    contact_turns: tuple = (None, None)
    past_contact: tuple = (0.0, 0.0)
    clamp_force: tuple = (0.0, 0.0)
    crushed: bool = False
    cores_clamped: bool = False
    hands_off_steps: int = 0
    pull: str = "none"  # none | running | passed | failed
    pull_moved: float = 0.0  # m
    pull_sheath_y: float = -1.0  # the sheath's cut end before the latest pull test (clamp frame y, m)


@dataclass(frozen=True)
class BarState(task_file.ObjectState):
    """The bar: its tilt (deg, its long axis against the horizontal)."""

    tilt_deg: float = 0.0


@dataclass(frozen=True)
class CableState(task_file.ObjectState):
    """The cable: the sheath's cut end along the clamp (clamp frame y, m; + toward the terminals), its centre's height
    and the cable's speed at the clamp (m/s, over the last control step)."""

    sheath_y: float = -1.0
    clamp_y_speed: float = 0.0


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


def _polygon_area(poly) -> float:
    return 0.5 * sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(poly, list(poly[1:]) + [poly[0]]))


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
                if np.linalg.norm(nrm) < 1e-12:
                    continue
                d = (p - p[i]) @ nrm
                if np.all(d <= 1e-9):
                    faces.append((i, j, k) if (c - p[i]) @ nrm < 0 else (i, k, j))
                elif np.all(d >= -1e-9):
                    faces.append((i, k, j) if (c - p[i]) @ nrm > 0 else (i, j, k))
    return faces


def _quatf(m):
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


def write_usd(kind: str, path: Path) -> Path:
    """One part as a USD file: bodies, colliders, looks, materials and (clamp, cable) joints."""
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

    jointed = kind in ("clamp", "cable")
    cframes = cable_frames() if kind == "cable" else {}
    frames = {b: p for b, (p, _) in cframes.items()} if kind == "cable" else clamp_frames() if kind == "clamp" else {}
    masses = clamp_masses() if kind == "clamp" else cable_masses() if kind == "cable" else {}
    body_paths = {}
    for body in BODIES[kind]:
        path_b = f"/Part/{body}" if jointed else "/Part"
        xf = UsdGeom.Xform.Define(stage, path_b) if jointed else UsdGeom.Xform(root)
        if jointed and any(frames.get(body, (0.0, 0.0, 0.0))):
            xf.AddTranslateOp().Set(Gf.Vec3d(*frames[body]))
        if body in cframes:
            xf.AddOrientOp().Set(_quatf(cframes[body][1]))
        body_paths[body] = path_b
        if kind == "box":
            continue  # a fixed collider
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if jointed:
            m, com, inertia = masses[body]
            mass_api.CreateMassAttr(m)
            # (clamp masses are given in the clamp frame: their centre relative to the body's frame)
            off = frames.get(body, (0.0, 0.0, 0.0)) if kind == "clamp" else (0.0, 0.0, 0.0)
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(*(c - o for c, o in zip(com, off, strict=True))))
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(*inertia))
        else:
            mass_api.CreateMassAttr(FREE_MASSES[kind])
        _apply(prim, "PhysxRigidBodyAPI", {
            "physxRigidBody:solverPositionIterationCount": 64,
            "physxRigidBody:solverVelocityIterationCount": 4,
            "physxRigidBody:maxDepenetrationVelocity": 0.3,
            "physxRigidBody:enableCCD": False,
        })

    for i, sh in enumerate(SHAPES[kind]()):
        parent = body_paths[sh.body]
        # clamp shapes are written in the part's frame (bodies offset); cable shapes in their body's own frame
        off = frames.get(sh.body, (0.0, 0.0, 0.0)) if kind == "clamp" else (0.0, 0.0, 0.0)

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
            z0, z1 = sh.span
            pts = [loc((x, y, z0)) for x, y in prof] + [loc((x, y, z1)) for x, y in prof]
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
        elif sh.axis == "S":
            geom = UsdGeom.Sphere.Define(stage, f"{parent}/sphere_{i}")
            geom.CreateRadiusAttr(sh.radius)
            geom.AddTranslateOp().Set(Gf.Vec3d(*loc(sh.center)))
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
        root_body = "base" if kind == "clamp" else "root"
        rb = stage.GetPrimAtPath(body_paths[root_body])
        UsdPhysics.ArticulationRootAPI.Apply(rb)
        _apply(rb, "PhysxArticulationAPI", {
            "physxArticulation:enabledSelfCollisions": False,
            "physxArticulation:solverPositionIterationCount": 64,
            "physxArticulation:solverVelocityIterationCount": 4,
        })
        if kind == "clamp":
            joints = {n: (t, p, c, ax, 0.0, lo, hi, k, d, a) for n, (t, p, c, ax, lo, hi, k, d, a)
                      in clamp_joints().items()}
        else:
            joints = {n: (t, p, c, ax, 0.0, lo, hi, k, d, a) for n, (t, p, c, ax, lo, hi, k, d, a)
                      in cable_joints().items()}
        for name, (jt, parent, child, ax, _turn, lo, hi, k, d, arm) in joints.items():
            joint = (UsdPhysics.RevoluteJoint if jt == "rev" else UsdPhysics.PrismaticJoint).Define(
                stage, f"/Part/joints/{name}")
            joint.CreateBody0Rel().SetTargets([Sdf.Path(body_paths[parent])])
            joint.CreateBody1Rel().SetTargets([Sdf.Path(body_paths[child])])
            joint.CreateAxisAttr(ax)
            joint.CreateLowerLimitAttr(lo)
            joint.CreateUpperLimitAttr(hi)
            fp, fc = frames.get(parent, (0.0, 0.0, 0.0)), frames.get(child, (0.0, 0.0, 0.0))
            if kind == "cable":
                rp_, rc_ = cframes[parent][1], cframes[child][1]
                dv = [c - p for c, p in zip(fc, fp, strict=True)]
                local = [sum(rp_[kk][ii] * dv[kk] for kk in range(3)) for ii in range(3)]
                rel = [[sum(rp_[kk][ii] * rc_[kk][jj] for kk in range(3)) for jj in range(3)] for ii in range(3)]
                joint.CreateLocalPos0Attr(Gf.Vec3f(*local))
                joint.CreateLocalRot0Attr(_quatf(rel))
            else:
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
        if kind == "clamp":
            for name, (ref, gearing) in MIMICS.items():
                prim = stage.GetPrimAtPath(f"/Part/joints/{name}")
                _apply(prim, "PhysxMimicJointAPI:rotZ", {"physxMimicJoint:rotZ:gearing": float(gearing),
                                                         "physxMimicJoint:rotZ:offset": 0.0})
                prim.CreateRelationship("physxMimicJoint:rotZ:referenceJoint").SetTargets(
                    [Sdf.Path(f"/Part/joints/{ref}")])
    stage.GetRootLayer().Save()
    return path


def usd_file(kind: str) -> str:
    global _USD_DIR
    if _USD_DIR is None:
        _USD_DIR = Path(tempfile.mkdtemp(prefix="cord_clamp_parts_"))
    path = _USD_DIR / f"{kind}.usda"
    if not path.exists():
        write_usd(kind, path)
    return str(path)


_NAMES: dict = {}  # kind -> object name
_POSES: dict = {}  # kind -> (x, y, z, yaw) of the part's origin, env frame


def arena_object(spec: Part):
    """The Arena object for a part: the box a fixed collider, the clamp and the cable articulations with fixed roots,
    the rest free rigid bodies."""
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    class _Clamp(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "thread": ImplicitActuatorCfg(joint_names_expr=["s[01]_thread"], stiffness=0.0, damping=0.0,
                                              armature=LINK_ARMATURE),
                "give": ImplicitActuatorCfg(joint_names_expr=["s[01]_give"], stiffness=CLAMP_K, damping=CLAMP_D,
                                            armature=LINK_ARMATURE),
                "spin": ImplicitActuatorCfg(joint_names_expr=["s[01]_spin"], stiffness=0.0, damping=0.0,
                                            armature=SCREW_ARMATURE, friction=FREE_TORQUE,
                                            dynamic_friction=FREE_TORQUE),
            }
            cfg.init_state.joint_pos = {".*": 0.0}
            return cfg

    class _Cable(Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {
                "bend": ImplicitActuatorCfg(joint_names_expr=["b[yz][0-9]+"], stiffness=EI / LINK_LEN,
                                            damping=BEND_D, armature=LINK_ARMATURE),
                "twist": ImplicitActuatorCfg(joint_names_expr=["tw[0-9]+"], stiffness=GJ / LINK_LEN, damping=TWIST_D,
                                             armature=LINK_ARMATURE),
                "cores": ImplicitActuatorCfg(joint_names_expr=["c(b[yz]|tw)[0-9]+"], stiffness=EI_CORES / CORE_LEN,
                                             damping=BEND_D, armature=LINK_ARMATURE),
            }
            cfg.init_state.joint_pos = {".*": 0.0}  # as laid (the curve is in the USD)
            return cfg

    pose = Pose(position_xyz=tuple(spec.pos),
                rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2)))
    _NAMES[spec.kind] = spec.name
    _POSES[spec.kind] = (*spec.pos, spec.yaw)
    cfg = sim_utils.UsdFileCfg(usd_path=usd_file(spec.kind))
    if spec.kind == "box":
        return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg, initial_pose=pose)
    if spec.kind in ("clamp", "cable"):
        cfg.articulation_props = sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=64, solver_velocity_iteration_count=4,
            fix_root_link=True)
        cls = _Clamp if spec.kind == "clamp" else _Cable
        return cls(name=spec.name, object_type=ObjectType.ARTICULATION, spawner_cfg=cfg, initial_pose=pose)
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg, initial_pose=pose)


# -- simulation state helpers ----------------------------------------------------------------------------------------


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


def clamp_frame():
    """(origin, rotation) of the clamp frame in the env frame."""
    import numpy as np

    x, y, z, yaw = _POSES["clamp"]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([x, y, z]), np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def body_poses(base, name):
    art = base.scene.articulations[name]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = _t(art.data.body_pos_w)[0].detach().cpu().numpy().astype(float) - origin
    quat = _t(art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
    return {b: (pos[i], _rot(quat[i])) for i, b in enumerate(art.body_names)}


def rigid_pose(base, name):
    """(origin env frame, rotation) of a free rigid part."""
    pose = _np(base.scene.rigid_objects[name].data.root_pose_w)
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    return pose[:3] - origin, _rot(pose[3:])


def joints(base, name) -> dict:
    art = base.scene.articulations[name]
    q = _np(art.data.joint_pos)
    return {n: float(q[i]) for i, n in enumerate(art.joint_names)}


def cable_geometry(base):
    """In the clamp frame: each link's start point and axis, the sheath's cut end (where the cores end) and the
    index of the first link outside the box's entry wall."""
    import numpy as np

    o, R = clamp_frame()
    poses = body_poses(base, _NAMES["cable"])
    starts, axes = [], []
    for name, *_ in cable_links():
        p, r = poses[name]
        starts.append(R.T @ (p - o))
        axes.append(R.T @ r[:, 0])
    starts, axes = np.array(starts), np.array(axes)
    sheath_end = starts[CORE_LINKS]
    wall_out = -BOX_Y / 2 - SADDLE_Y  # the right wall's outer face (clamp frame y)
    outside = next((i for i in range(CORE_LINKS, len(starts)) if starts[i][1] < wall_out - 15 * MM), len(starts) - 1)
    return {"starts": starts, "axes": axes, "sheath_end": sheath_end, "outside": outside}


def screw_mechanism(base) -> dict:
    """Per screw: turns in since the start and the force under its head (N)."""
    q = joints(base, _NAMES["clamp"])
    return {s: {"turns": -q[f"{s}_thread"] / PITCH, "force": CLAMP_K * max(0.0, q[f"{s}_give"])} for s in SCREWS}


def bar_tilt_deg(base) -> float:
    """The bar's tilt as its ends follow the screw heads: the heads' height difference (the screws' advance down
    their threads, PITCH per turn) over their spacing. On a real clamp the give is in the sheath under the bar's
    middle, so the bar's ends sit at the heads; in the simulation the give is a spring under each head, which lets
    the bar body level out between two pressing heads (both 0.3 and 1.45 turns past contact: the bar body 0.5
    degrees, bench run 14), so the body's own tilt is not used."""
    m = screw_mechanism(base)
    dz = PITCH * abs(m[SCREWS[0]]["turns"] - m[SCREWS[1]]["turns"])
    return math.degrees(math.atan2(dz, 2 * SCREW_DX))


def bar_body_tilt_deg(base) -> float:
    """The simulated bar body's own tilt about the cable's axis (for logs)."""
    _, r = rigid_pose(base, _NAMES["bar"])
    return math.degrees(math.asin(min(1.0, abs(float(r[2, 0])))))


# -- fingers (for "hands off") ---------------------------------------------------------------------------------------

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


def hand_distance(base, geo) -> float:
    """The nearest robot fingertip point to the screws' heads, the bar, or the cable within HANDS_ZONE of the clamp
    (m). The screwdriver held in a hand counts as the hand (its blade tip)."""
    import numpy as np

    o, R = clamp_frame()
    targets = []
    poses = body_poses(base, _NAMES["clamp"])
    for s in SCREWS:
        p, _ = poses[s]
        targets.append(p + (0.0, 0.0, HEAD_H / 2))
    bp, br = rigid_pose(base, _NAMES["bar"])
    for dx in (-BAR_SIZE[0] / 2, 0.0, BAR_SIZE[0] / 2):
        targets.append(bp + br @ np.array([dx, 0.0, BAR_SIZE[2]]))
    for p, a in zip(geo["starts"], geo["axes"], strict=True):
        if np.linalg.norm(p[:2]) < HANDS_ZONE:
            targets.append(R @ (p + 0.5 * LINK_LEN * a) + o)
    targets = np.array(targets)
    best = float("inf")
    if "driver" in _NAMES:  # a screwdriver's blade near the clamp counts as a hand on it
        dp, dr = rigid_pose(base, _NAMES["driver"])
        best = min(best, float(np.linalg.norm(targets - dp[None, :], axis=1).min()))
    for art_name, art in base.scene.articulations.items():
        if art_name in _NAMES.values():
            continue
        names = [b for b in art.body_names if b.startswith("tip")]
        if not names:
            continue
        poses_r = body_poses(base, art_name)
        for b in names:
            p, r = poses_r[b]
            pts = finger_points(b) @ r.T + p
            best = min(best, float(np.linalg.norm(targets[:, None, :] - pts[None, :, :], axis=2).min()))
    return best


# -- the per-step update ---------------------------------------------------------------------------------------------


@dataclass
class _Screw:
    contact: float | None = None  # turns at which its head first pressed
    pressing: int = 0
    pressing_from: float = 0.0


@dataclass
class _Track:
    screws: dict = field(default_factory=lambda: {s: _Screw() for s in SCREWS})
    crushed: bool = False
    cores_clamped: bool = False
    cores_steps: int = 0
    hands_off: int = 0
    hand_dist: float = float("inf")
    pull: str = "none"
    pull_step: int = 0
    pull_y0: float = 0.0
    pull_moved: float = 0.0
    pull_turns: tuple = (0.0, 0.0)
    pull_sheath: float = 0.0
    pull_body: int = -1
    last_y: float | None = None
    y_speed: float = 0.0
    bar_last: tuple | None = None  # (pos, rotation matrix) at the previous control step
    bar_v: tuple = (0.0, 0.0, 0.0)  # m/s and rad/s from the pose change per control step (PhysX's own velocities
    bar_w: tuple = (0.0, 0.0, 0.0)  # of the bar squeezed between heads and cable jitter at ~12 mm/s, ~2 rad/s, demo 19)
    log: list = field(default_factory=list)


_TRACK = _Track()
HANDS_ON = [False]  # a bench's "virtual hand" is on the clamp or the cable (counts as hands on)
_SET: dict = {}
_EPISODE_STEP: list = [None]


def _set_pull(base, force: float, body: int) -> None:
    import torch

    art = base.scene.articulations[_NAMES["cable"]]
    _, R = clamp_frame()
    n = len(art.body_names)
    forces = torch.zeros((1, n, 3), dtype=torch.float32, device=base.device)
    if force > 0 and body >= 0:
        forces[0, body] = torch.tensor((R @ [0.0, -force, 0.0]).tolist(), dtype=torch.float32, device=base.device)
    art.permanent_wrench_composer.set_forces_and_torques_index(forces=forces, torques=torch.zeros_like(forces),
                                                               body_ids=list(range(n)), is_global=True)


def _cable_set(base, dt: float) -> None:
    """Creep each of the sheath's bending springs' rest angle toward the joint's angle beyond SET_BAND (SET_TAU)."""
    import numpy as np
    import torch

    art = base.scene.articulations[_NAMES["cable"]]
    names = list(art.joint_names)
    if "ids" not in _SET:
        _SET["ids"] = [i for i, n in enumerate(names) if "b" in n and "tw" not in n
                       and (SET_CORES or not n.startswith("c"))]  # the sheath (and the cores, if SET_CORES)
        _SET["rest"] = np.zeros(len(_SET["ids"]))
    ids, rest = _SET["ids"], _SET["rest"]
    q = _np(art.data.joint_pos)[ids]
    dev = q - rest
    rest += np.sign(dev) * np.maximum(np.abs(dev) - SET_BAND, 0.0) * min(1.0, dt / SET_TAU)
    art.set_joint_position_target_index(
        target=torch.tensor([rest.tolist()], dtype=torch.float32, device=base.device), joint_ids=ids)


def update(base) -> None:
    """Once per control step: follow the screws, the latches and the pull test (see the module docstring)."""
    global _TRACK
    step = getattr(base, "episode_length_buf", None)
    step = None if step is None else int(step[0])
    if step == 0 or (_EPISODE_STEP[0] is not None and step is not None and step < _EPISODE_STEP[0]):
        if _TRACK.pull == "running":
            _set_pull(base, 0.0, -1)
        _TRACK = _Track()
        _SET.clear()
    elif step is not None and step == _EPISODE_STEP[0]:
        return  # already updated this step (yam_arena and a demo may both ask for the states)
    _EPISODE_STEP[0] = step
    tr = _TRACK
    dt = float(base.cfg.sim.dt * base.cfg.decimation)

    def n(seconds):
        return max(1, round(seconds / dt))

    _cable_set(base, dt)
    m = screw_mechanism(base)
    geo = cable_geometry(base)
    sheath_y = float(geo["sheath_end"][1])
    if tr.last_y is not None:
        tr.y_speed = (sheath_y - tr.last_y) / dt
    tr.last_y = sheath_y
    if "bar" in _NAMES:
        import numpy as np

        bp, br = rigid_pose(base, _NAMES["bar"])
        if tr.bar_last is not None:
            p0, r0 = tr.bar_last
            dr = br @ r0.T
            ang = math.acos(max(-1.0, min(1.0, (float(np.trace(dr)) - 1.0) / 2.0)))
            axis = np.array([dr[2, 1] - dr[1, 2], dr[0, 2] - dr[2, 0], dr[1, 0] - dr[0, 1]])
            n_ax = float(np.linalg.norm(axis))
            tr.bar_v = tuple((bp - p0) / dt)
            tr.bar_w = tuple(axis / n_ax * ang / dt) if n_ax > 1e-12 else (0.0, 0.0, 0.0)
        tr.bar_last = (np.array(bp, dtype=float), np.array(br, dtype=float))
    for s in SCREWS:
        sc, ms = tr.screws[s], m[s]
        if ms["force"] >= CONTACT_CLAMP:
            if sc.pressing == 0:
                sc.pressing_from = ms["turns"]
            sc.pressing += 1
        else:
            sc.pressing = 0
        if sc.contact is None and sc.pressing >= n(CONTACT_S):
            sc.contact = sc.pressing_from
        elif sc.contact is not None and ms["turns"] < sc.contact - 0.05 and ms["force"] < CONTACT_CLAMP:
            sc.contact = None  # loosened off again
        if sc.contact is not None and ms["turns"] - sc.contact >= CRUSH_TURNS:
            tr.crushed = True
    # the bar pressing while the sheath's end isn't past it: the cores clamped (latched)
    if max(m[s]["force"] for s in SCREWS) >= CORES_CLAMP and sheath_y < BAR_SIZE[1] / 2:
        tr.cores_steps += 1
        if tr.cores_steps >= n(0.25):
            tr.cores_clamped = True
    else:
        tr.cores_steps = 0
    tr.hand_dist = hand_distance(base, geo)
    tr.hands_off = tr.hands_off + 1 if tr.hand_dist > HANDS_OFF_DIST and not HANDS_ON[0] else 0
    turns = tuple(m[s]["turns"] for s in SCREWS)
    if tr.pull in ("passed", "failed") and (max(abs(a - b) for a, b in zip(turns, tr.pull_turns, strict=True))
                                            > PULL_INVALID_TURNS or abs(sheath_y - tr.pull_sheath) > PULL_INVALID_MOVE):
        tr.pull = "none"
    pressing = any(tr.screws[s].contact is not None for s in SCREWS)  # the bar clamps (a head pressed 0.5 s)
    if tr.pull == "running":
        if tr.hands_off == 0:
            _set_pull(base, 0.0, -1)
            tr.pull = "none"
        else:
            tr.pull_step += 1
            tr.pull_moved = max(tr.pull_moved, tr.pull_y0 - sheath_y)
            if tr.pull_moved >= PULL_MAX_MOVE:
                _set_pull(base, 0.0, -1)
                tr.pull = "failed"
            elif tr.pull_step >= n(PULL_S):
                _set_pull(base, 0.0, -1)
                tr.pull = "passed"
            else:
                _set_pull(base, PULL_FORCE * min(1.0, (tr.pull_step + 1) / n(PULL_RAMP_S)), tr.pull_body)
            if tr.pull != "running":
                tr.pull_turns, tr.pull_sheath = turns, sheath_y
                tr.log.append((step, tr.pull, tr.pull_moved, turns))
    elif tr.pull == "none" and tr.hands_off >= n(HANDS_OFF_S) and pressing:
        art = base.scene.articulations[_NAMES["cable"]]
        tr.pull_body = list(art.body_names).index(cable_links()[geo["outside"]][0])
        tr.pull, tr.pull_step, tr.pull_y0, tr.pull_moved = "running", 0, sheath_y, 0.0
        _set_pull(base, PULL_FORCE / n(PULL_RAMP_S), tr.pull_body)


def reset_parts(base) -> None:
    """The clamp and the cable back to their start (joints, velocities, targets at zero, as built); no pull."""
    import torch

    global _TRACK
    for kind in ("clamp", "cable"):
        if kind not in _NAMES:
            continue
        art = base.scene.articulations[_NAMES[kind]]
        k = len(art.joint_names)
        zero = torch.zeros((1, k), dtype=torch.float32, device=base.device)
        art.write_joint_position_to_sim_index(position=zero, env_ids=[0])
        art.write_joint_velocity_to_sim_index(velocity=zero, env_ids=[0])
        art.set_joint_position_target_index(target=zero, joint_ids=list(range(k)))
        art.permanent_wrench_composer.reset()
    _TRACK = _Track()
    _SET.clear()
    _EPISODE_STEP[0] = None


def clamp_state(base) -> ClampState:
    import numpy as np

    tr = _TRACK
    m = screw_mechanism(base)
    o, R = clamp_frame()
    lo, hi = bounds("clamp")
    corners = np.array([[(lo, hi)[(i >> k) & 1][k] for k in range(3)] for i in range(8)]) @ R.T + o
    art = base.scene.articulations[_NAMES["clamp"]]
    ang = _t(art.data.body_ang_vel_w)[0].detach().cpu().numpy().astype(float)
    spin = max(float(np.linalg.norm(ang[list(art.body_names).index(s)])) for s in SCREWS)
    past = tuple((m[s]["turns"] - tr.screws[s].contact) if tr.screws[s].contact is not None else 0.0 for s in SCREWS)
    return ClampState(pos=o.copy(), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3),
                      ang_vel=np.array([0.0, 0.0, spin]), low=corners.min(axis=0), high=corners.max(axis=0),
                      turns=tuple(m[s]["turns"] for s in SCREWS),
                      contact_turns=tuple(tr.screws[s].contact for s in SCREWS), past_contact=past,
                      clamp_force=tuple(m[s]["force"] for s in SCREWS), crushed=tr.crushed,
                      cores_clamped=tr.cores_clamped, hands_off_steps=tr.hands_off, pull=tr.pull,
                      pull_moved=tr.pull_moved, pull_sheath_y=tr.pull_y0)


def cable_state(base) -> CableState:
    import numpy as np

    tr = _TRACK
    geo = cable_geometry(base)
    o, R = clamp_frame()
    pts = np.array([R @ p + o for p in geo["starts"]])
    end = geo["sheath_end"]
    v = R @ np.array([0.0, tr.y_speed, 0.0])
    return CableState(pos=R @ end + o, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=v, ang_vel=np.zeros(3),
                      low=pts.min(axis=0), high=pts.max(axis=0), sheath_y=float(end[1]),
                      clamp_y_speed=float(tr.y_speed))


def bar_state(base, rigid_state) -> BarState:
    import numpy as np

    tr = _TRACK  # velocities from the pose change per control step (see _Track.bar_v)
    return BarState(pos=rigid_state.pos, quat_xyzw=rigid_state.quat_xyzw, lin_vel=np.array(tr.bar_v),
                    ang_vel=np.array(tr.bar_w), low=rigid_state.low, high=rigid_state.high,
                    tilt_deg=bar_tilt_deg(base))


# -- the hook into yam_arena.scene -----------------------------------------------------------------------------------


def _patch(scene_module) -> None:
    original = getattr(scene_module, "task_object", None)
    original_states = getattr(scene_module, "object_states", None)
    for fn, params in ((original, ["spec"]), (original_states, ["env", "names"])):
        if not callable(fn) or list(inspect.signature(fn).parameters)[: len(params)] != params:
            raise RuntimeError(
                "cc_parts: yam_arena.scene no longer has task_object(spec) and object_states(env, names); "
                "this task's parts hook needs updating for this yam_arena version"
            )
    if getattr(original, "_spawns_cc_parts", False):
        return

    def task_object(spec):
        return arena_object(spec) if isinstance(spec, Part) else original(spec)

    def object_states(env, names):
        base = env.unwrapped
        update(base)
        ours = {_NAMES.get("clamp"), _NAMES.get("cable")}
        out = original_states(env, [n for n in names if n not in ours])
        for n in names:
            if n == _NAMES.get("clamp"):
                out[n] = clamp_state(base)
            elif n == _NAMES.get("cable"):
                out[n] = cable_state(base)
            elif n == _NAMES.get("bar"):
                out[n] = bar_state(base, out[n])
        return {n: out[n] for n in names}

    original_make_env = getattr(scene_module, "make_env", None)

    def make_env(*args, **kwargs):
        """yam_arena's env, its reset also putting the clamp and the cable back (yam_arena resets the robot and the
        free objects only)."""
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
    task_object._spawns_cc_parts = True
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
