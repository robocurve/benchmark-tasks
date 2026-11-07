"""cutter: a desktop CO2 laser cutter, its sizes and its collider boxes, shared by the task file and the Blender script.

Pure Python (no NumPy, no yam_arena), so Blender can import it. The same file is in the laser-scrap-sheet-lift and
open-laser-cutter-lid tasks.

The machine, 52 cm wide, 40 cm deep and 22 cm tall, no brand: a dark body whose front wall is only 7 cm tall, two side
walls and a back block (the tube and the exhaust) the full height, an aluminium honeycomb bed inside with its top
14.5 cm up, and the laser head on its gantry parked at the back. The lid is a hood: a top panel with a tinted window
and a 15 cm front flap that closes the front above the low wall, a handle bar along the flap's lower edge, hinged at
the front edge of the back block. A gas strut on each side holds it open at 80 degrees.

Coordinates: the machine's own frame has u forward (station x) from the front face of its body, v to the robot's left
(station y) from its centre line, w up from the table. :func:`at` turns a machine point into the station frame for a
machine whose front face is at ``xf`` and centre line at ``yc``.
"""

from __future__ import annotations

import math

TOP = 0.7475  # the station's table top

# -- the body ----------------------------------------------------------------------------------------------------------
W, D, H = 0.52, 0.40, 0.22  # width (v), depth (u), height (w)
WALL = 0.015  # walls and lid panels
FRONT_H = 0.07  # the front wall's top: low, so a hand pinching a sheet on the bed passes over it
BACK_D = 0.07  # the back block (u from D - BACK_D to D): the laser tube and the exhaust, full height
HINGE_U = D - BACK_D  # the lid's hinge: across the top of the back block's front face
HINGE_W = H

# -- the bed: aluminium honeycomb on a support, its top 14.5 cm up ---------------------------------------------------
BED_TOP_W = 0.145
BED_U = (0.05, 0.31)  # its near and far edges (a 4 cm gap behind the front wall, for fingers under a sheet)
BED_V = W / 2 - WALL - 0.004  # it spans v from -BED_V to +BED_V

# -- the gantry, parked at the back, and the laser head on it ---------------------------------------------------------
RAIL_W = (0.185, 0.20)  # the side rails' bottom and top
RAIL_V = (W / 2 - WALL - 0.016, W / 2 - WALL - 0.001)  # each side rail's inner and outer |v|
BEAM_U = (0.285, 0.31)  # the beam across, parked at the back
BEAM_W = (0.175, 0.20)
HEAD_V = (-0.205, -0.165)  # the head, parked at the right end of the beam
HEAD_U = (0.270, 0.325)
HEAD_W = (0.150, 0.200)

# -- the lid, closed (its own pieces are given in the lid box's frame by lid_pieces) ----------------------------------
LID_GAP = 0.002  # the closed lid sits this far above the walls
FLAP_W = FRONT_H + LID_GAP  # the flap's lower edge
HANDLE_W = 0.14  # the handle bar's centre height when closed
HANDLE_T = 0.016  # its square section
HANDLE_STANDOFF = 0.018  # the gap between the bar and the flap, for fingers
HANDLE_V = 0.21  # the bar runs from -HANDLE_V to +HANDLE_V, a post at each end
HANDLE_U = -(HANDLE_STANDOFF + HANDLE_T / 2)  # the bar's centre in front of the body
LID_U = (HANDLE_U - HANDLE_T / 2, HINGE_U - LID_GAP)  # the closed lid's extent along u, handle included
LID_SIZE = (LID_U[1] - LID_U[0], W, H - FLAP_W)
LID_CENTRE = ((LID_U[0] + LID_U[1]) / 2, 0.0, (FLAP_W + H) / 2)  # machine frame
OPEN_DEG = 80.0  # the open stop, where the struts hold it


def at(xf: float, yc: float, u: float, v: float, w: float):
    """A machine point in the station frame."""
    return (xf + u, yc + v, TOP + w)


def _box(lo, hi):
    return tuple((a + b) / 2 for a, b in zip(lo, hi)), tuple(b - a for a, b in zip(lo, hi))


def body_boxes():
    """The fixed parts as (name, centre, size) in the machine frame: walls, back block, bed, gantry and head."""
    s = W / 2
    parts = [
        ("cutter_front", (0.0, -s, 0.0), (WALL, s, FRONT_H)),
        ("cutter_right", (WALL, -s, 0.0), (HINGE_U, -s + WALL, H - WALL - LID_GAP)),
        ("cutter_left", (WALL, s - WALL, 0.0), (HINGE_U, s, H - WALL - LID_GAP)),
        ("cutter_back", (HINGE_U, -s, 0.0), (D, s, H)),
        ("cutter_bed", (BED_U[0], -BED_V, 0.0), (BED_U[1], BED_V, BED_TOP_W)),
        ("cutter_rail_r", (BED_U[0] + 0.04, -RAIL_V[1], RAIL_W[0]), (HINGE_U, -RAIL_V[0], RAIL_W[1])),
        ("cutter_rail_l", (BED_U[0] + 0.04, RAIL_V[0], RAIL_W[0]), (HINGE_U, RAIL_V[1], RAIL_W[1])),
        ("cutter_beam", (BEAM_U[0], -RAIL_V[0], BEAM_W[0]), (BEAM_U[1], RAIL_V[0], BEAM_W[1])),
        ("cutter_head", (HEAD_U[0], HEAD_V[0], HEAD_W[0]), (HEAD_U[1], HEAD_V[1], HEAD_W[1])),
    ]
    return [(name, *_box(lo, hi)) for name, lo, hi in parts]


def placed_body(xf: float, yc: float):
    """:func:`body_boxes` in the station frame."""
    return [(name, (xf + c[0], yc + c[1], TOP + c[2]), size) for name, c, size in body_boxes()]


def lid_pieces():
    """The lid's collider pieces as (centre, size) in the closed lid box's frame (centred on LID_CENTRE): the top
    panel, the front flap, the handle bar and its two posts."""
    cu, _, cw = LID_CENTRE
    s = W / 2
    parts = [
        ((0.0, -s, H - WALL), (LID_U[1], s, H)),  # top panel (the window is in it)
        ((0.0, -s, FLAP_W), (WALL, s, H - WALL)),  # front flap
        ((HANDLE_U - HANDLE_T / 2, -HANDLE_V, HANDLE_W - HANDLE_T / 2),
         (HANDLE_U + HANDLE_T / 2, HANDLE_V, HANDLE_W + HANDLE_T / 2)),  # handle bar
        ((HANDLE_U + HANDLE_T / 2, -HANDLE_V, HANDLE_W - HANDLE_T / 2),
         (0.0, -HANDLE_V + HANDLE_T, HANDLE_W + HANDLE_T / 2)),  # right post
        ((HANDLE_U + HANDLE_T / 2, HANDLE_V - HANDLE_T, HANDLE_W - HANDLE_T / 2),
         (0.0, HANDLE_V, HANDLE_W + HANDLE_T / 2)),  # left post
    ]
    out = []
    for lo, hi in parts:
        c, size = _box(lo, hi)
        out.append(((c[0] - cu, c[1], c[2] - cw), size))
    return out


def lid_volume_centre():
    """The lid's centre of mass for an even density, machine frame (u, w)."""
    tot = su = sw = 0.0
    for (cu, _, cw), (a, b, c) in lid_pieces():
        vol = a * b * c
        tot += vol
        su += vol * (cu + LID_CENTRE[0])
        sw += vol * (cw + LID_CENTRE[2])
    return su / tot, sw / tot


def turn(u: float, w: float, deg: float):
    """The machine point (u, w) on the lid, turned ``deg`` open about the hinge (its front edge going up)."""
    a = math.radians(deg)
    du, dw = u - HINGE_U, w - HINGE_W
    return (HINGE_U + du * math.cos(a) + dw * math.sin(a), HINGE_W - du * math.sin(a) + dw * math.cos(a))


def handle_point(deg: float):
    """The handle bar's centre line (u, w) with the lid open ``deg`` degrees."""
    return turn(HANDLE_U, HANDLE_W, deg)


def open_lid_pieces(deg: float = OPEN_DEG):
    """The lid's pieces turned ``deg`` open, as (centre, size, pitch) in the machine frame, pitch in radians about +v
    (the box turned like the lid). For a lid that stays open."""
    out = []
    for (cu, cv, cw), size in lid_pieces():
        u, w = turn(cu + LID_CENTRE[0], cw + LID_CENTRE[2], deg)
        out.append(((u, cv, w), size, math.radians(deg)))
    return out


def open_lid_bounds(deg: float = OPEN_DEG):
    """The open lid's axis-aligned bounds in the machine frame, (lo, hi)."""
    lo, hi = [1e9] * 3, [-1e9] * 3
    a = math.radians(deg)
    for (u, v, w), (su, sv, sw), _ in open_lid_pieces(deg):
        for du in (-su / 2, su / 2):
            for dw in (-sw / 2, sw / 2):
                pu = u + du * math.cos(a) + dw * math.sin(a)
                pw = w - du * math.sin(a) + dw * math.cos(a)
                for i, p in ((0, pu), (2, pw)):
                    lo[i], hi[i] = min(lo[i], p), max(hi[i], p)
        lo[1], hi[1] = min(lo[1], v - sv / 2), max(hi[1], v + sv / 2)
    return tuple(lo), tuple(hi)
