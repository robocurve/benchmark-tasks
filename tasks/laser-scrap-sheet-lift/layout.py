"""layout: the sizes and places of the scene, shared by the task file and the Blender script (looks/make_looks.py).

Pure Python (no NumPy, no yam_arena), so Blender can import it. Metres, in the station frame: x forward from the arm
bases, y to the robot's left, z up from the floor. The sheet's own frame has u along x and v along y, from its
centre. The laser cutter's sizes are in ``cutter.py``.
"""

from __future__ import annotations

import math

import cutter as C

TOP = C.TOP  # the station's table top (yam_arena.task_file.TABLE_TOP["station"])

# -- the laser cutter: its front face 29.5 cm out, its left side on the robot's centre line, lid open ---------------
XF = 0.295
YC = -C.W / 2  # the machine's centre line: its left side wall's outer face at y = 0
BED_TOP = TOP + C.BED_TOP_W
BED_X = (XF + C.BED_U[0], XF + C.BED_U[1])  # the honeycomb's near and far edges
BED_Y = (YC - C.BED_V, YC + C.BED_V)

# -- the sheet: 5 mm clear acrylic, its four cut parts still in their holes -------------------------------------------
OVERHANG = 0.03  # m: the sheet's near edge sticks out past the honeycomb's near edge by this much
SHEET_SIZE = (0.22, 0.20, 0.005)  # x (depth), y (width), thickness
SHEET_EDGE_X = BED_X[0] - OVERHANG  # the sheet's near edge
SHEET_Y = -0.15  # its centre line: its left edge 2 cm clear of the gantry's left rail
ACRYLIC = 1190.0  # kg/m^3
PLAY = 0.0005  # m between each part and its hole, all round (the cut's width)
SHEET_POS = (SHEET_EDGE_X + SHEET_SIZE[0] / 2, SHEET_Y, BED_TOP + SHEET_SIZE[2] / 2)
FAR_STRIP = 0.012  # m of sheet left between the far edge and the far side of the two far holes
DISC_R = 0.03
DISC_R_ = DISC_R
RECT = (0.05, 0.03)  # long side, short side
TRI = 0.04  # equilateral, side

# The parts in the sheet's frame: (u, v) of each one's bounding-box centre. The disc and the 3 x 5 cm rectangle were
# cut close to the far edge (1.2 cm of sheet behind them), the other two in the middle. The near 6 cm of the sheet is
# uncut, for the fingers.
_FAR = SHEET_SIZE[0] / 2 - FAR_STRIP - PLAY  # u of the far side of the far parts
PART_UV = {
    "disc": (_FAR - DISC_R_, -0.045),
    "triangle": (-0.025, -0.045),
    "rect_long": (-0.020, 0.040),  # 5 cm along x
    "rect_wide": (_FAR - 0.015, 0.050),  # 5 cm along y
}
PART_SIZE = {
    "disc": (2 * DISC_R, 2 * DISC_R),
    "triangle": (TRI * math.sqrt(3) / 2, TRI),  # its point toward +x
    "rect_long": (RECT[0], RECT[1]),
    "rect_wide": (RECT[1], RECT[0]),
}
DISC_SIDES = 32  # the disc's collider: a 32-sided prism around the 6 cm circle
HOLE_SIDES = 24  # the disc's hole: 24 sides 0.5 mm outside the circle


def triangle_outline(cx: float = 0.0, cy: float = 0.0, side: float = TRI, grow: float = 0.0):
    """The triangle's corners, counter-clockwise, point toward +x, around its bounding box's centre (cx, cy), every
    side moved ``grow`` outward."""
    h = side * math.sqrt(3) / 2
    pts = [(h / 2, 0.0), (-h / 2, side / 2), (-h / 2, -side / 2)]
    if grow:
        # the incentre is at x = -h/2 + h/3 from the box centre; move each corner away from it by grow / sin(30 deg)
        ix = -h / 2 + h / 3
        out = []
        for x, y in pts:
            dx, dy = x - ix, y
            d = math.hypot(dx, dy)
            out.append((x + dx / d * 2 * grow, y + dy / d * 2 * grow))
        pts = out
    return [(cx + x, cy + y) for x, y in pts]


def rect_outline(cx: float, cy: float, sx: float, sy: float, grow: float = 0.0):
    hx, hy = sx / 2 + grow, sy / 2 + grow
    return [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]


def circle_outline(cx: float, cy: float, apothem: float, n: int):
    r = apothem / math.cos(math.pi / n)
    return [(cx + r * math.cos(2 * math.pi * (k + 0.5) / n), cy + r * math.sin(2 * math.pi * (k + 0.5) / n))
            for k in range(n)]


def hole_outlines():
    """Each hole in the sheet's frame (convex, counter-clockwise): the part's outline grown by the play."""
    out = {}
    for name, (u, v) in PART_UV.items():
        if name == "disc":
            out[name] = circle_outline(u, v, DISC_R + PLAY, HOLE_SIDES)
        elif name == "triangle":
            out[name] = triangle_outline(u, v, grow=PLAY)
        else:
            out[name] = rect_outline(u, v, *PART_SIZE[name], grow=PLAY)
    return out


def part_area(name: str) -> float:
    if name == "disc":
        return math.pi * DISC_R ** 2
    if name == "triangle":
        return TRI * TRI * math.sqrt(3) / 4
    return RECT[0] * RECT[1]


def part_mass(name: str) -> float:
    return part_area(name) * SHEET_SIZE[2] * ACRYLIC


def sheet_mass() -> float:
    holes = sum(part_area(n) for n in PART_UV) + 2 * PLAY * 0.6  # the parts and roughly 0.6 m of cut, 0.5 mm wide
    return (SHEET_SIZE[0] * SHEET_SIZE[1] - holes) * SHEET_SIZE[2] * ACRYLIC


def part_pos(name: str):
    u, v = PART_UV[name]
    return (SHEET_POS[0] + u, SHEET_POS[1] + v, BED_TOP + SHEET_SIZE[2] / 2)


# -- the scrap bin: an open black plastic bin, fixed, left of the machine -------------------------------------------
BIN_OUTSIDE = (0.30, 0.234, 0.10)  # x, y, height
BIN_WALL = 0.006
BIN_FLOOR = 0.006
BIN_XY = (SHEET_POS[0], YC + C.W / 2 + 0.01 + BIN_OUTSIDE[1] / 2)  # 1 cm left of the machine
BIN_RIM = TOP + BIN_OUTSIDE[2]
BIN_INSIDE = (BIN_OUTSIDE[0] - 2 * BIN_WALL, BIN_OUTSIDE[1] - 2 * BIN_WALL)


def bin_parts():
    """The bin's floor and walls as (name, centre, size), fixed boxes."""
    (bx, by), (sx, sy, h), w, f = BIN_XY, BIN_OUTSIDE, BIN_WALL, BIN_FLOOR
    zc = TOP + f + (h - f) / 2
    return [
        ("bin_floor", (bx, by, TOP + f / 2), (sx, sy, f)),
        ("bin_near", (bx - sx / 2 + w / 2, by, zc), (w, sy, h - f)),
        ("bin_far", (bx + sx / 2 - w / 2, by, zc), (w, sy, h - f)),
        ("bin_right", (bx, by - sy / 2 + w / 2, zc), (sx - 2 * w, w, h - f)),
        ("bin_left", (bx, by + sy / 2 - w / 2, zc), (sx - 2 * w, w, h - f)),
    ]


# -- the lid, standing open at its stop: fixed --------------------------------------------------------------------------
_lo, _hi = C.open_lid_bounds()
LID_OPEN_CENTRE = tuple((a + b) / 2 for a, b in zip(_lo, _hi))  # machine frame
LID_OPEN_SIZE = tuple(b - a for a, b in zip(_lo, _hi))
LID_OPEN_POS = C.at(XF, YC, *LID_OPEN_CENTRE)


def open_lid_pieces():
    """The open lid's collider pieces as (centre, size, yaw, pitch) in its box's frame."""
    return [(tuple(c - o for c, o in zip(cent, LID_OPEN_CENTRE)), size, 0.0, pitch)
            for cent, size, pitch in C.open_lid_pieces()]
