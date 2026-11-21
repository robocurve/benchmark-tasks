"""totekit: an attached-lid tote (a flip-top container: a tub with two lids hinged on its long sides, whose free edges
meet along an S-shaped seam traced from a product photo: each lid has one full tooth and a half tooth), for yam_arena task files.

    import pathlib, sys
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
    import totekit as K

The tote is one articulation: the tub is the root (a free body on the table) and each lid is a link on a revolute joint
about the tub's x axis (its long axis), at the top outer edge of a long wall. Every part is made of cuboid colliders, except the three
diagonal sections of each lid's plate, which are convex prisms (trapezoids in plan view) so that the seam's diagonals are true straight
lines: the lids' overlap is real boxes: the lid tops are flush across the seam, and under the top surface the wider lid of each straight
run has a thin overhang (an upper layer) lying over a thin tongue of the other lid. Whether a lid closed first blocks the second is
the geometry's doing (measured, see the README). The model has no snap or latch; a shut lid is held by its weight.

Joint angles (rad), measured on each lid as the angle it is open by (theta, 0 closed and flat on the rim, 90 standing
up, THETA_START = 180 flat outward (the lid lying on its support), THETA_STOP = 265 the hinge stop, the lid hanging almost straight
down against the outside of the long wall, as in the manufacturer's video); the simulator's own joint angle is -theta for the
+y lid and +theta for the -y lid. :func:`lid_angles` returns thetas.

In the product the open lids do not stay flat: they swing to about 270 degrees and hang. A trial therefore starts with both lids
lying flat at THETA_START, each on a fixed support beside the tote (:func:`support_specs`).

``on_step`` and ``on_reset`` register callbacks ``fn(env)``. Pure Python at import time; Isaac Sim is reached only when
the scene is built. The hooks are installed through ``sys.meta_path`` the way yam_arena's own modules need (the task
file is read before the simulator starts).
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena.task_file import Box, ObjectState

__all__ = ["Tote", "ToteState", "finger_points", "install", "lid_angles", "lid_point", "on_reset", "on_step"]

# -- the Akro-Mils 39085 attached lid container, outside dimensions as on the retailer listings the team used (m) -------------------------
LENGTH, WIDTH, HEIGHT = 0.5461, 0.381, 0.2286     # 21.5 x 15 x 9 in; some listings give the width as 15.25 in, the model uses 15
# -- Geometry and masses, read from product photos: see README -----------------------------------------------------
WALL = 0.004                    # wall thickness
FLOOR = 0.008                   # floor thickness
LID_T = 0.004                   # lid plate thickness
HINGE_OUT = 0.004               # the hinge axis sits this far outside the rim band, so a hanging lid clears it
LID_LEN = LENGTH - 0.008        # the lid's length along the hinge
SEAM_GAP = 0.002                # the visible gap between the two lids' top surfaces (flush tops)
LAP_OV = 0.010                  # the lap under the flush top: the tongue of the lower-lapping lid reaches this far under the overhang of the other
LAP_CLEAR = 0.002               # the overlying lid's full-thickness plate stops this far short of the tongue's tip
OVER_W0 = 0.0018                # the overhang (upper layer) spans w 1.8 mm to the 4 mm top: 2.2 mm thick, flush with the top
TONGUE_T = 0.0012               # the tongue spans w 0 to 1.2 mm, 0.6 mm clear of the overhang's underside
SEAM_OFFSET = 0.030             # From product photos (+-1.5 cm): the seam line is this far from the tote's centre line, toward +y in
                                # the high runs and toward -y in the low runs (6 cm between the levels). The hang limit is
                                # HINGE_Y + SEAM_OFFSET < HEIGHT - 3 mm: an overlying lid's top edge hangs this far below its hinge
# The seam along x (tote frame), traced from a retailer product photo of the grey 39085 (a rectified top view of the closed lids, see README), read to about
# +-1.5 cm: end run R0 (high, y +SEAM_OFFSET), a diagonal, a long run R1 (low), the short middle diagonal, a long run R2 (high), a
# diagonal, end run R3 (low). Point-symmetric about the tote's centre. The short jogs the photo shows at the seal blocks are not modelled.
# Breakpoints in x (m): the end of R0, the start of R1, the end of R1, the start of R2, the end of R2, the start of R3.
SEAM_X = (-0.215, -0.158, -0.030, 0.030, 0.158, 0.215)
# each diagonal is one straight-edged piece of plain plate: a convex prism (a trapezoid in plan view, 4 mm thick) whose edge runs in a
# straight line between the neighbouring runs' edges (about 45 degrees; the lids keep a constant 2 mm gap, measured along y)
BLOCK = (0.020, 0.025, 0.008)   # raised seal block at each lid's seam end (x, u, height); the hole is a gap in it
TOTE_MASS, LID_MASS = 1.9, 0.4  # kg
LID_DAMPING = 0.15              # N m s / rad: viscous hinge friction
THETA_STOP = math.radians(265.0)   # The hinge stop. The video shows an open lid hanging nearly straight down (about 270
                                   # degrees from closed); 265 keeps the plate off the rim band and the tapered wall
THETA_START = math.radians(180.0)  # the trial's start: each lid lies flat, outward, on a fixed support (not a product stop)
# fixed supports beside the tote (designed for the task, not part of the product); the lid's near end overhangs by SUPPORT_OVERHANG
SUPPORT_OVERHANG = 0.100        # m: the lid's near end (x -LID_LEN/2) sticks out this far past the support, for the lower gripper pad
SUPPORT_U0, SUPPORT_U1 = 0.120, 0.250  # the support lies under the lid between these distances from the hinge axis: beyond the bosses (u up to
                                        # 9.2 cm) and the embossed logos (u up to about 11.5 cm), and 2.5 cm or more beyond the lid's outer edge
SUPPORT_X1 = 0.215              # the support's far end, tote-frame x (2.4 cm short of the far end's seal block, clear of its lug)
SUPPORT_TOP = HEIGHT - LID_T    # the support top is the plate's underside at 180 degrees: 4 mm below the rim plane
SUPPORT_GAP = 0.0               # (tote-frame) the support top equals the plate underside; no gap

# -- product-photo details, all read from product photos (see README) ----------------------------------
A_TOP = 0.006                   # the wall's outer face is this far in from the rim band's outer face (the band stands proud)
A_BASE = 0.025                  # the wall's outer face at the base is this far in from the rim outline (nestable taper)
RIM_H, RIM_T = 0.035, 0.012     # rim band: height and horizontal thickness
ZT = HEIGHT - RIM_H             # the band's underside; the tapered wall runs from z 0 to here
PHI = math.atan((A_BASE - A_TOP) / ZT)      # wall lean from vertical (about 5.6 degrees)
FOOT_H, FOOT_W = 0.006, 0.012   # recessed foot: the tub stands on a ring this tall and wide, the floor sits above it
SLOT_H, SLOT_GAP = 0.020, 0.004     # slots under the rim band: height, and the wall left between slot top and band
SLOT_W_LONG, SLOT_W_END = 0.060, 0.050
RIB_W, RIB_PROUD = 0.014, 0.003     # visual ribs between the recessed panels
LUG = (0.050, 0.030, 0.007)         # lid stacking lug (x, u, height) at each outer corner, u from the hinge side
LUG_U0, LUG_X0 = 0.050, 0.200       # lug starts this far from the hinge axis and this far from the lid's middle (|x|)
HOLE = 0.006                        # square hole through each seal block (a gap between boxes, not round)
KNUCKLES, KNUCKLE_LEN, KNUCKLE_R = 7, 0.040, 0.006     # visual hinge knuckles per side
BOSS_R, BOSS_H, BOSS_U, BOSS_X = 0.012, 0.003, 0.080, 0.140   # two bosses on each lid's top, 3 mm proud, modelled as 2.13 cm squares
BOSS_SQ = BOSS_R * math.sqrt(math.pi)   # side of the square with the area of the 2.4 cm round boss


def inset(z: float) -> float:
    """How far the tapered wall's outer face is in from the rim outline at height z (z in 0..ZT)."""
    return A_BASE + (A_TOP - A_BASE) * z / ZT


def wall_piece(kind: str, sgn: int, z0: float, z1: float, c0: float, c1: float, thick: float, offset: float):
    """A tilted box on the tapered wall: ``kind`` 'long' (the wall at y = sgn * ..., c along x) or 'end' (at x = sgn * ..., c
    along y), between wall heights z0 and z1 measured on the outer face, c0..c1 along the wall. ``offset`` is the box's
    distance from the outer face along the outward normal (negative = inside the wall) of its middle plane. Returns
    (centre, size, quat_wxyz)."""
    zm = (z0 + z1) / 2
    half = (WIDTH if kind == "long" else LENGTH) / 2 - inset(zm)      # outer face's horizontal half-extent at zm
    h = half + offset * math.cos(PHI)
    z = zm - offset * math.sin(PHI)
    cc, ln, hz = (c0 + c1) / 2, c1 - c0, (z1 - z0) / math.cos(PHI)
    if kind == "long":
        al = -sgn * PHI / 2
        return (cc, sgn * h, z), (ln, thick, hz), (math.cos(al), math.sin(al), 0.0, 0.0)
    be = sgn * PHI / 2
    return (sgn * h, cc, z), (thick, ln, hz), (math.cos(be), 0.0, math.sin(be), 0.0)


def _segments(lo, hi, gaps):
    out, a = [], lo
    for g0, g1 in sorted(gaps):
        if g0 > a:
            out.append((a, g0))
        a = g1
    if hi > a:
        out.append((a, hi))
    return out


def panel_ribs(kind: str):
    """Rib and slot centres along a wall: 5 panels on a long wall, 2 on an end wall."""
    zm = ZT / 2
    n = 5 if kind == "long" else 2
    half = (LENGTH if kind == "long" else WIDTH) / 2 - inset(zm) - 0.016
    ribs = [-half + 2 * half * i / n for i in range(n + 1)]
    return ribs, [(a + b) / 2 for a, b in zip(ribs, ribs[1:])]


def tub_parts() -> list:
    """Every part of the tub as (name, centre, size, quat_wxyz or None, collide). Collision: tapered walls (with real slot
    openings under the rim band), rim band, foot ring, floor. Visual only: the ribs between the recessed panels."""
    parts = []
    zs0, zs1 = ZT - SLOT_GAP - SLOT_H, ZT - SLOT_GAP
    for kind in ("long", "end"):
        for sgn in (1, -1):
            tag = f"{kind}_{'pn'[sgn < 0]}"
            half = lambda z: (LENGTH if kind == "long" else WIDTH) / 2 - inset(z) - (0.0 if kind == "long" else 0.002)
            _, centres = panel_ribs(kind)
            sw = SLOT_W_LONG if kind == "long" else SLOT_W_END
            slots = [(c - sw / 2, c + sw / 2) for c in (centres[1], centres[-2]) ] if kind == "long" else [(c - sw / 2, c + sw / 2) for c in centres]
            for nm, (z0, z1, gaps) in {"lo": (0.0, zs0, []), "mid": (zs0, zs1, slots), "top": (zs1, ZT, [])}.items():
                zm = (z0 + z1) / 2
                for si, (c0, c1) in enumerate(_segments(-half(zm), half(zm), gaps)):
                    ctr, size, q = wall_piece(kind, sgn, z0, z1, c0, c1, WALL, -WALL / 2)
                    parts.append((f"wall_{tag}_{nm}{si}", ctr, size, q, True))
            ribs, _ = panel_ribs(kind)
            for ri, c in enumerate(ribs):
                if kind == "end" and ri in (0, len(ribs) - 1):
                    continue                                         # the corner ribs belong to the long walls
                ctr, size, q = wall_piece(kind, sgn, 0.010, ZT - 0.001, c - RIB_W / 2, c + RIB_W / 2, RIB_PROUD, RIB_PROUD / 2)
                parts.append((f"rib_{tag}_{ri}", ctr, size, q, False))
    zb = HEIGHT - RIM_H / 2                                          # rim band: a ring from z ZT to HEIGHT, outline = the catalogue size
    for sgn, tag in ((1, "pos"), (-1, "neg")):
        parts.append((f"rim_{tag}", (0.0, sgn * (WIDTH / 2 - RIM_T / 2), zb), (LENGTH, RIM_T, RIM_H), None, True))
        parts.append((f"rim_end_{tag}", (sgn * (LENGTH / 2 - RIM_T / 2), 0.0, zb), (RIM_T, WIDTH - 2 * RIM_T, RIM_H), None, True))
    lx, wy = LENGTH - 2 * A_BASE, WIDTH - 2 * A_BASE                 # foot ring (outline = the base footprint) and the floor above it
    for sgn, tag in ((1, "pos"), (-1, "neg")):
        parts.append((f"foot_{tag}", (0.0, sgn * (wy / 2 - FOOT_W / 2), FOOT_H / 2), (lx, FOOT_W, FOOT_H), None, True))
        parts.append((f"foot_end_{tag}", (sgn * (lx / 2 - FOOT_W / 2), 0.0, FOOT_H / 2), (FOOT_W, wy - 2 * FOOT_W, FOOT_H), None, True))
    parts.append(("floor", (0.0, 0.0, FOOT_H + FLOOR / 2), (lx, wy, FLOOR), None, True))
    return parts


HINGE_Y = WIDTH / 2 + HINGE_OUT                 # |y| of the hinge axes


def _run_list():
    """(x0, x1, seam centre y, wider lid side) of the four straight runs. Where the seam is high (+y) the -y lid is the wider one."""
    xs = (-LID_LEN / 2,) + SEAM_X + (LID_LEN / 2,)
    spans = [(xs[0], xs[1]), (xs[2], xs[3]), (xs[4], xs[5]), (xs[6], xs[7])]
    levels = [+1, -1, +1, -1]
    return [(a, b, lv * SEAM_OFFSET, -lv) for (a, b), lv in zip(spans, levels)]


# (x0, x1, seam centre y, the lid side that is the wider one there)
RUNS = _run_list()
DIAGONALS = [(SEAM_X[0], SEAM_X[1], 0, 1), (SEAM_X[2], SEAM_X[3], 1, 2), (SEAM_X[4], SEAM_X[5], 2, 3)]   # (x0, x1, run before, run after)


def centre_y(x: float) -> float:
    """The seam's centre line (tote-frame y) at tote-frame ``x`` in the straight runs (the diagonals in between are plain plate)."""
    for x0, x1, yc, _ in RUNS:
        if x0 <= x <= x1:
            return yc
    raise ValueError("x is on a diagonal or off the lid")


def _edge(side: int, yc: float, wide: int):
    """(top-surface edge u, role) of the lid at ``side`` in a straight run with seam line yc and wider lid ``wide``: distance from
    the hinge axis to the lid's top-surface edge. The wider lid ("over") ends exactly at the seam line with an overhang; the other
    lid ("tongue") stops SEAM_GAP short of it and has a thin tongue reaching LAP_OV under the overhang."""
    d = HINGE_Y - side * yc                                 # hinge axis to the seam line
    return (d, "over") if wide == side else (d - SEAM_GAP, "tongue")


def lid_zones(side: int) -> list[dict]:
    """The lid at ``side``'s free-edge profile along x: zones with x0, x1, the top-surface edge u_end (distance from the hinge axis),
    and the role in a straight run ('over' or 'tongue'; None on a diagonal). Each of the three diagonals is one zone (``diag`` True)
    of plain plate whose edge is the straight line from u_a at x0 to u_b at x1 (the neighbouring runs' edges; the lids keep a constant 2 mm
    gap there); its u_end is the larger of the two, the zone's farthest reach."""
    ends = [_edge(side, yc, wide) for _, _, yc, wide in RUNS]
    zones = [dict(x0=x0, x1=x1, u_end=ends[i][0], role=ends[i][1], run=i) for i, (x0, x1, _, _) in enumerate(RUNS)]
    for x0, x1, i0, i1 in DIAGONALS:
        u0, u1 = ends[i0][0], ends[i1][0]
        zones.append(dict(x0=x0, x1=x1, u_a=u0, u_b=u1, u_end=max(u0, u1), role=None, run=None, diag=True))
    return zones


def diag_edge_u(z: dict, x: float) -> float:
    """The plate's free-edge u at ``x`` inside a diagonal zone (a straight line from u_a at x0 to u_b at x1)."""
    return z["u_a"] + (z["u_b"] - z["u_a"]) * (x - z["x0"]) / (z["x1"] - z["x0"])


def lid_outline(side: int) -> list:
    """The top surface's free edge of the lid at ``side`` as a polyline of (x, y) points in the tote frame, from the lid's near end to its
    far end (the plate's seam-side outline: straight runs and the straight diagonals between them)."""
    pts = []
    for z in sorted(lid_zones(side), key=lambda z: z["x0"]):
        ua, ub = (z["u_a"], z["u_b"]) if z.get("diag") else (z["u_end"], z["u_end"])
        pts += [(z["x0"], side * (HINGE_Y - ua)), (z["x1"], side * (HINGE_Y - ub))]
    return pts


@dataclass(frozen=True)
class Tote(Box):
    """The tote. ``pos`` is the middle of its footprint on the table (z the table top), ``quat`` (xyzw) its
    orientation; ``mass`` is the tub's (the lids add 2 x LID_MASS)."""

    quat: tuple = (0.0, 0.0, 0.0, 1.0)
    colour: tuple = (0.14, 0.34, 0.70)          # mid blue, one solid colour for tub and lids
    lid_colour: tuple = (0.14, 0.34, 0.70)
    friction: tuple = (0.45, 0.35)
    meta: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        if not self.name.isidentifier():
            raise ValueError(f"object name {self.name!r} must be a Python identifier")
        if self.mass is None or self.mass <= 0:
            raise ValueError(f"{self.name}: a Tote must be loose (give it a mass)")


@dataclass(frozen=True)
class ToteState(ObjectState):
    """The tote as the simulator has it; ``joints`` maps ``lid_pos``/``lid_neg`` to their opening angles theta (rad),
    ``joint_vel`` to their angular speeds (rad/s)."""

    joints: dict = field(default_factory=dict)
    joint_vel: dict = field(default_factory=dict)


def _rotation(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# Measuring points: the middle of the long straight run in which the lid is the wider one: R1 (the low run, x -9.4 cm) for the +y lid,
# R2 (the high run, x +9.4 cm) for the -y lid.
GAUGE_X = {+1: (RUNS[1][0] + RUNS[1][1]) / 2, -1: (RUNS[2][0] + RUNS[2][1]) / 2}


def gauge_u(side: int) -> float:
    """Distance from the hinge axis to the lid's free (seam-side) plate edge at its gauge point GAUGE_X[side]."""
    return next(z["u_end"] for z in lid_zones(side) if z["x0"] <= GAUGE_X[side] <= z["x1"])


def edge_height(side: int, theta: float) -> float:
    """Height (m) of the lid's free plate edge (its lower corner) above the rim plane at its gauge point."""
    return gauge_u(side) * math.sin(theta)


def lid_point(side: int, theta: float, s: float, offset: float = 0.0, x: float = 0.0) -> np.ndarray:
    """A point on a lid's wall-facing face (its underside when open past 90 degrees), in the tote's frame (origin the
    middle of its footprint, z up): ``s`` metres out from the hinge axis along the lid, ``offset`` metres off that face
    (positive is away from the lid, on the side of the wall/ground), at ``x`` along the hinge."""
    d = np.array([-side * math.cos(theta), math.sin(theta)])            # (y, z) along the lid from the hinge
    n = np.array([side * math.sin(theta), math.cos(theta)])             # normal of the face turned toward the wall/ground
    yz = np.array([side * HINGE_Y, HEIGHT]) + s * d + offset * n
    return np.array([x, yz[0], yz[1]])


def lid_parts(side: int) -> list:
    """Every collision box of the lid at ``side`` as (tag, u0, u1, w0, w1, x0, x1): u from the hinge axis along the lid, w off the
    plate's lower face (up when the lid is shut), x along the hinge. Plate, overhangs, tongues, seal blocks (with their holes), lugs and bosses."""
    out = []
    zones = lid_zones(side)
    for zi, z in enumerate(zones):
        if z.get("diag"):                                   # a diagonal: the prism's bounding box (the real piece is in diag_prisms)
            out.append((f"edge_{zi}", 0.0, z["u_end"], 0.0, LID_T, z["x0"], z["x1"]))
        elif z["role"] == "over":                           # full plate, then an overhang (upper layer, flush top) over the other's tongue
            uf = z["u_end"] - LAP_OV - LAP_CLEAR
            out.append((f"edge_{zi}", 0.0, uf, 0.0, LID_T, z["x0"], z["x1"]))
            out.append((f"over_{zi}", uf, z["u_end"], OVER_W0, LID_T, z["x0"], z["x1"]))
        elif z["role"] == "tongue":                         # full plate, then a thin lower tongue reaching under the other lid's overhang
            out.append((f"edge_{zi}", 0.0, z["u_end"], 0.0, LID_T, z["x0"], z["x1"]))
            out.append((f"tongue_{zi}", z["u_end"], z["u_end"] + SEAM_GAP + LAP_OV, 0.0, TONGUE_T, z["x0"], z["x1"]))
        else:
            out.append((f"edge_{zi}", 0.0, z["u_end"], 0.0, LID_T, z["x0"], z["x1"]))
    for e in (1, -1):                                       # raised seal block at each seam end, with a square hole through it
        xa, xb = sorted((e * (LID_LEN / 2 - 0.030), e * (LID_LEN / 2 - 0.010)))
        ub = next(z["u_end"] for z in zones if z["x0"] <= (xa + xb) / 2 <= z["x1"]) - 0.025     # 2.5 cm inside the local plate edge
        ua = ub - BLOCK[1]
        xc, uc, h = (xa + xb) / 2, (ua + ub) / 2, HOLE / 2
        t, zt_ = f"block_{'ew'[e < 0]}", LID_T + BLOCK[2]
        out += [(t + "_a", ua, ub, LID_T, zt_, xa, xc - h), (t + "_b", ua, ub, LID_T, zt_, xc + h, xb),
                (t + "_c", ua, uc - h, LID_T, zt_, xc - h, xc + h), (t + "_d", uc + h, ub, LID_T, zt_, xc - h, xc + h)]
    for e in (1, -1):                                       # stacking lug at each outer corner, on the hinge side
        out.append((f"lug_{'ew'[e < 0]}", LUG_U0, LUG_U0 + LUG[1], LID_T, LID_T + LUG[2], *sorted((e * LUG_X0, e * (LUG_X0 + LUG[0])))))
    for e in (1, -1):                                       # two bosses on the top face (square boxes of the round boss's area)
        bh = BOSS_SQ / 2
        out.append((f"boss_{'ew'[e < 0]}", BOSS_U - bh, BOSS_U + bh, LID_T, LID_T + BOSS_H, e * BOSS_X - bh, e * BOSS_X + bh))
    return out


def diag_prisms(side: int) -> dict:
    """The lid's diagonal plate pieces as {tag: eight vertices (x, u, w)}: a convex prism 0 to LID_T thick whose plan view is the
    trapezoid x0..x1, u from 0 (the hinge axis) to the straight free edge. The tag is that of the matching bounding box in lid_parts."""
    out = {}
    for zi, z in enumerate(lid_zones(side)):
        if z.get("diag"):
            quad = [(z["x0"], 0.0), (z["x1"], 0.0), (z["x1"], z["u_b"]), (z["x0"], z["u_a"])]
            out[f"edge_{zi}"] = [(x, u, w) for w in (0.0, LID_T) for x, u in quad]
    return out


def lid_box_world(side: int, theta: float, part):
    """A lid box (see lid_parts) posed at opening angle ``theta``: (centre, 3x3 rotation, half sizes), tote frame (origin the middle of
    the footprint on the table, z up). Used by the offline clearance tests."""
    _, u0, u1, w0, w1, x0, x1 = part
    a = -side * theta                                       # the simulator's joint angle about +x
    rot = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    local = np.array([(x0 + x1) / 2, -side * (u0 + u1) / 2, (w0 + w1) / 2])
    centre = np.array([0.0, side * HINGE_Y, HEIGHT]) + rot @ local
    return centre, rot, np.array([x1 - x0, u1 - u0, w1 - w0]) / 2


def support_specs() -> list:
    """The two fixed supports as (side, x0, x1, y0, y1, top): the lid at ``side`` lies on its flat top surface between SUPPORT_U0 and
    SUPPORT_U1 from its hinge axis, from SUPPORT_OVERHANG short of its near end to SUPPORT_X1. Tote frame; the top is a height above
    the table. Each is a flat-topped block standing on the table, fixed there."""
    x0 = -LID_LEN / 2 + SUPPORT_OVERHANG
    out = []
    for side in (+1, -1):
        ya, yb = sorted((side * (HINGE_Y + SUPPORT_U0), side * (HINGE_Y + SUPPORT_U1)))
        out.append((side, x0, SUPPORT_X1, ya, yb, SUPPORT_TOP))
    return out


def max_hang_reach() -> float:
    """The farthest any lid box reaches from the hinge axis (u), which must stay under HEIGHT for the lid to hang clear of the table."""
    return max(p[2] for side in (+1, -1) for p in lid_parts(side))


# -- authoring helpers --------------------------------------------------------------------------------------------------

def _material(sim_utils, sf, df):
    try:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df, friction_combine_mode="average")
    except TypeError:
        return sim_utils.RigidBodyMaterialCfg(static_friction=sf, dynamic_friction=df)


class _Author:
    def __init__(self, prim_path, translation, orientation):
        import isaaclab.sim as sim_utils
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import Gf, UsdGeom

        self.sim_utils, self.Gf, self.UsdGeom = sim_utils, Gf, UsdGeom
        self.stage = get_current_stage()
        self.path = prim_path
        double = UsdGeom.XformOp.PrecisionDouble
        root = UsdGeom.Xform.Define(self.stage, prim_path)
        root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation or (0.0, 0.0, 0.0))))
        x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
        root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
        self.root = root.GetPrim()
        self.looks, self.mats = {}, {}

    def look(self, color, rough=0.6) -> str:
        from yam_arena.station import linear_rgb

        key = (tuple(color), rough)
        if key not in self.looks:
            path = f"{self.path}/Looks/v{len(self.looks)}"
            vis = self.sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=rough)
            vis.func(path, vis)
            self.looks[key] = path
        return self.looks[key]

    def mat(self, key, sf, df) -> str:
        if key not in self.mats:
            path = f"{self.path}/Looks/phys_{key}"
            pm = _material(self.sim_utils, sf, df)
            pm.func(path, pm)
            self.mats[key] = path
        return self.mats[key]

    def body(self, path, pos, mass, ang_damp=0.1):
        from pxr import PhysxSchema, UsdPhysics

        xf = self.UsdGeom.Xform.Define(self.stage, path)
        if path != self.path and pos is not None:
            xf.AddTranslateOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Vec3d(*pos))
            xf.AddOrientOp(precision=self.UsdGeom.XformOp.PrecisionDouble).Set(self.Gf.Quatd(1, 0, 0, 0))
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(float(mass))
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
        rb.CreateMaxDepenetrationVelocityAttr(0.3)
        rb.CreateLinearDampingAttr(0.05)
        rb.CreateAngularDampingAttr(ang_damp)
        return prim

    def box(self, path, centre, size, look, mat, quat_wxyz=None, collide=True):
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics

        cube = self.UsdGeom.Cube.Define(self.stage, path)
        cube.CreateSizeAttr(1.0)
        cube.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        if quat_wxyz is not None:
            cube.AddOrientOp().Set(self.Gf.Quatf(*(float(v) for v in quat_wxyz)))
        cube.AddScaleOp().Set(self.Gf.Vec3f(*(float(v) for v in size)))
        prim = cube.GetPrim()
        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        if not collide:
            return prim
        UsdPhysics.CollisionAPI.Apply(prim)
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0008)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim

    def prism(self, path, verts, faces, look, mat):
        """A convex mesh collider (convex hull) with flat-shaded outward faces; ``verts`` are body-frame points, ``faces`` index lists."""
        from isaaclab.sim.utils import bind_physics_material, bind_visual_material
        from pxr import PhysxSchema, UsdPhysics, Vt

        pts = np.array(verts, float)
        centre = pts.mean(0)
        counts, idx, normals = [], [], []
        for f in faces:
            f = list(f)
            n = np.cross(pts[f[1]] - pts[f[0]], pts[f[2]] - pts[f[0]])
            if n @ (pts[f].mean(0) - centre) < 0:
                f.reverse()
                n = -n
            n = n / np.linalg.norm(n)
            counts.append(len(f))
            idx += f
            normals += [self.Gf.Vec3f(*(float(v) for v in n))] * len(f)
        mesh = self.UsdGeom.Mesh.Define(self.stage, path)
        mesh.CreatePointsAttr(Vt.Vec3fArray([self.Gf.Vec3f(*(float(v) for v in p)) for p in pts]))
        mesh.CreateFaceVertexCountsAttr(counts)
        mesh.CreateFaceVertexIndicesAttr(idx)
        mesh.CreateNormalsAttr(Vt.Vec3fArray(normals))
        mesh.SetNormalsInterpolation("faceVarying")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(False)
        prim = mesh.GetPrim()
        bind_visual_material(str(prim.GetPath()), look, stage=self.stage)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        cp = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        cp.CreateContactOffsetAttr(0.0008)
        cp.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), mat, stage=self.stage)
        return prim

    def cylinder(self, path, centre, radius, height, axis, look):
        """A visual-only cylinder (no collider)."""
        from isaaclab.sim.utils import bind_visual_material

        cyl = self.UsdGeom.Cylinder.Define(self.stage, path)
        cyl.CreateAxisAttr(axis)
        cyl.CreateRadiusAttr(float(radius))
        cyl.CreateHeightAttr(float(height))
        cyl.AddTranslateOp().Set(self.Gf.Vec3d(*(float(v) for v in centre)))
        bind_visual_material(str(cyl.GetPrim().GetPath()), look, stage=self.stage)
        return cyl.GetPrim()


# -- spawning ------------------------------------------------------------------------------------------------------------

_SPECS: dict = {}
SIDES = {"lid_pos": +1, "lid_neg": -1}       # joint name -> side (+1: hinged at +y)


def _spawn_tote(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from pxr import Gf, PhysxSchema, Sdf, UsdPhysics

    spec: Tote = _SPECS[cfg.kit_name]
    a = _Author(prim_path, translation, orientation)
    grey, lid_grey = a.look(spec.colour), a.look(spec.lid_colour)
    m_tub, m_lid = a.mat("tub", *spec.friction), a.mat("lid", 0.4, 0.3)

    tub = f"{prim_path}/tub"
    a.body(tub, (0.0, 0.0, 0.0), spec.mass, ang_damp=0.05)
    for nm, centre, size, quat, collide in tub_parts():
        a.box(f"{tub}/{nm}", centre, size, grey, m_tub, quat_wxyz=quat, collide=collide)
    stage = a.stage
    for jname, side in SIDES.items():
        body = f"{prim_path}/{jname}"
        a.body(body, (0.0, side * HINGE_Y, HEIGHT), LID_MASS, ang_damp=0.05)
        # lid frame: origin on the hinge axis at the plate's lower face; the closed plate runs toward the centre line
        # (local y = -side * u) with u the distance from the hinge axis. Boxes are (tag, u0, u1, w0, w1, x0, x1).
        prisms = diag_prisms(side)
        for tag, u0, u1, w0, w1, x0, x1 in lid_parts(side):
            if tag in prisms:                                   # a diagonal: a convex prism in the lid frame (local y = -side * u)
                v = [(x, -side * u, w) for x, u, w in prisms[tag]]
                a.prism(f"{body}/{tag}", v, [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4), (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)], lid_grey, m_lid)
                continue
            a.box(f"{body}/{tag}", ((x0 + x1) / 2, -side * (u0 + u1) / 2, (w0 + w1) / 2), (x1 - x0, u1 - u0, w1 - w0), lid_grey, m_lid)
        for k in range(KNUCKLES):                               # visual hinge knuckles centred on the axis
            kx = -(LID_LEN / 2 - KNUCKLE_LEN / 2 - 0.004) + k * (LID_LEN - KNUCKLE_LEN - 0.008) / (KNUCKLES - 1)
            a.cylinder(f"{body}/knuckle_{k}", (kx, 0.0, 0.0), KNUCKLE_R, KNUCKLE_LEN, "X", lid_grey)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{body}/{jname}")
        j.CreateAxisAttr("X")
        j.CreateBody0Rel().SetTargets([Sdf.Path(tub)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(body)])
        j.CreateLocalPos0Attr(Gf.Vec3f(0.0, side * HINGE_Y, HEIGHT))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        lo, hi = (-math.degrees(THETA_STOP), 0.0) if side > 0 else (0.0, math.degrees(THETA_STOP))
        j.CreateLowerLimitAttr(float(lo))
        j.CreateUpperLimitAttr(float(hi))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(float(LID_DAMPING) * math.radians(1.0))   # per degree
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(tub))
    art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(tub))
    art.CreateEnabledSelfCollisionsAttr(True)             # the two lids must collide with each other
    art.CreateSolverPositionIterationCountAttr(32)
    art.CreateSolverVelocityIterationCountAttr(8)
    return a.root


def _tote_object(spec: Tote):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class ToteSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_tote)
        kit_name: str = ""

    class KitObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {"hinges": ImplicitActuatorCfg(joint_names_expr=list(SIDES), stiffness=0.0, damping=LID_DAMPING)}
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            cfg = self._add_initial_pose_to_cfg(cfg)
            cfg.init_state.joint_pos = {"lid_pos": -THETA_START, "lid_neg": THETA_START}
            return cfg

    _SPECS[spec.name] = spec
    return KitObject(name=spec.name, object_type=ObjectType.ARTICULATION,
                     spawner_cfg=ToteSpawnerCfg(kit_name=spec.name),
                     initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),
                                       rotation_xyzw=tuple(float(v) for v in spec.quat)))


# -- run time ------------------------------------------------------------------------------------------------------------

def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    if hasattr(x, "numpy") and not hasattr(x, "detach"):
        return np.asarray(x.numpy(), dtype=np.float64)
    return _t(x).detach().cpu().numpy().astype(np.float64)


def lid_angles(environment, name: str) -> tuple[dict, dict]:
    """Opening angle theta (rad) and angular speed (rad/s) of each lid."""
    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    q = _np(asset.data.joint_pos)[0]
    qd = _np(asset.data.joint_vel)[0]
    th = {j: -SIDES[j] * float(q[names.index(j)]) for j in SIDES}        # +y lid: sim angle is -theta; -y lid: +theta
    om = {j: abs(float(qd[names.index(j)])) for j in SIDES}
    return th, om


def _write_pose(environment, name, pos, quat_xyzw) -> None:
    import torch

    base = environment.unwrapped
    asset = base.scene[name]
    origin = _np(base.scene.env_origins[0])
    dev = _t(asset.data.root_link_pose_w).device
    pose = torch.tensor([[*(np.asarray(pos, float) + origin), *quat_xyzw]], dtype=torch.float32, device=dev)
    vel = torch.zeros((1, 6), dtype=torch.float32, device=dev)
    for stem, kw in (("write_root_pose_to_sim", {"root_pose": pose}), ("write_root_velocity_to_sim", {"root_velocity": vel})):
        fn = getattr(asset, f"{stem}_index", None)
        if fn is not None:
            fn(env_ids=[0], **kw)
        else:
            getattr(asset, stem)(*kw.values())


def _set_open(environment, name: str, spec) -> None:
    """Both lids lying flat at THETA_START (on their supports), at rest, the tote back on its mark (writing joint positions moves an
    articulation's root, so the root pose is written again after them)."""
    import torch

    asset = environment.unwrapped.scene[name]
    names = list(asset.joint_names)
    ids = [names.index(j) for j in SIDES]
    dev = _t(asset.data.joint_pos).device
    pos = torch.tensor([[-SIDES[j] * THETA_START for j in SIDES]], dtype=torch.float32, device=dev)
    fn = getattr(asset, "write_joint_state_to_sim_index", None)
    if fn is not None:
        fn(position=pos, velocity=torch.zeros_like(pos), joint_ids=ids, env_ids=[0])
    else:
        asset.write_joint_state_to_sim(pos, torch.zeros_like(pos), joint_ids=ids)
    tfn = getattr(asset, "set_joint_position_target_index", None)
    if tfn is not None:
        tfn(target=pos, joint_ids=ids, env_ids=[0])
    else:
        asset.set_joint_position_target(pos, joint_ids=ids)
    _write_pose(environment, name, spec.pos, spec.quat)


SETTLE_STEPS = 10
_SETTLE: dict = {}


def _kit_state(environment, spec) -> ToteState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _np(base.scene.env_origins[0])
    pose = _np(asset.data.root_link_pose_w)[0]
    pos, quat = pose[:3] - origin, pose[3:7]
    vel = _np(asset.data.root_vel_w)[0] if hasattr(asset.data, "root_vel_w") else np.zeros(6)
    th, om = lid_angles(environment, spec.name)
    corners = np.array([[x, y, z] for x in (-LENGTH / 2, LENGTH / 2) for y in (-WIDTH / 2, WIDTH / 2) for z in (0.0, HEIGHT)])
    pts = corners @ _rotation(quat).T + pos
    return ToteState(pos=pos, quat_xyzw=quat, lin_vel=vel[:3], ang_vel=vel[3:6], low=pts.min(0), high=pts.max(0),
                     joints=th, joint_vel=om)


def finger_points(environment) -> list:
    """The grasp point of each gripper (world, relative to the env origin), from the arms' joint angles."""
    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS

    base = environment.unwrapped
    pts = []
    for arm in ("left", "right"):
        robot = base.scene[f"{arm}_robot"]
        names = list(robot.joint_names)
        q = _np(robot.data.joint_pos)[0]
        qa = [float(q[names.index(j)]) for j in ARM_JOINTS]
        pts.append(ik.tcp_pose(np.array(qa), station.BASES[arm])[0])
    return pts


# -- hooking yam_arena ------------------------------------------------------------------------------------------------

_STEP: list = []
_RESET: list = []
_ENV: dict = {"env": None}


def on_step(fn):
    _STEP.append(fn)
    return fn


def on_reset(fn):
    _RESET.append(fn)
    return fn


def _patch_scene(scene) -> None:
    if getattr(scene, "_totekit_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Tote):
            return _tote_object(spec)
        return stock_object(spec)

    def object_states(environment, names):
        mine = {n for n in names if isinstance(_SPECS.get(n), Tote)}
        plain = [n for n in names if n not in mine]
        out = stock_states(environment, plain) if plain else {}
        for n in mine:
            out[n] = _kit_state(environment, _SPECS[n])
        return {n: out[n] for n in names}

    scene.task_object = task_object
    scene.object_states = object_states
    scene._totekit_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_totekit_patched", False):
        return
    stock_step, stock_reset = cls._task_step, cls.reset

    def _task_step(self, *a, **k):
        _ENV["env"] = self._env
        for name, left in list(_SETTLE.items()):                       # just reset: hold it open where it stands
            if left > 0:
                _SETTLE[name] = left - 1
                _set_open(self._env, name, _SPECS[name])
        for fn in _STEP:
            fn(self._env)
        return stock_step(self, *a, **k)

    def reset(self, *a, **k):
        obs = stock_reset(self, *a, **k)
        _ENV["env"] = self._env
        for name, spec in _SPECS.items():
            _set_open(self._env, name, spec)
            _SETTLE[name] = SETTLE_STEPS
        try:                                         # the observation above was drawn before the lids were opened: draw it again
            base = self._env.unwrapped
            for _ in range(3):
                for name, spec in _SPECS.items():
                    _set_open(self._env, name, spec)
                base.sim.step(render=True)
            base.scene.update(dt=base.physics_dt)
            obs = self._observation()
        except Exception as exc:                     # keep the stock observation
            print(f"totekit: could not refresh the reset observation: {exc!r}", file=sys.stderr)
        for fn in _RESET:
            fn(self._env)
        return obs

    cls._task_step, cls.reset = _task_step, reset
    cls._totekit_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    targets = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}

    def find_spec(self, fullname, path, target=None):
        if fullname not in self.targets:
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        loader, patch = spec.loader, self.targets[fullname]
        stock_exec = loader.exec_module

        def exec_module(module):
            stock_exec(module)
            patch(module)

        loader.exec_module = exec_module
        return spec


def install() -> None:
    """Hook yam_arena (now for modules already imported, otherwise when they are). Idempotent."""
    for name, patch in _Finder.targets.items():
        if name in sys.modules:
            patch(sys.modules[name])
    if not any(isinstance(f, _Finder) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


install()
