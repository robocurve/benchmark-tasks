"""Every dimension, mass and threshold of the pry-board-off-joists task, in one place (metres, kilograms, radians, N).

World frame (yam_arena's): x forward from the line through the arm bases, y to the robot's left, z up; the arm bases
stand at y = +-0.305. The station table's top is the work surface: ``TOP``. On it, in front of the robot:

- the frame (fixed), a piece of floor or deck framing: two 2x4 SPF joists (38 x 89 mm actual) standing on edge,
  612 mm long, running away from the robot, 16 in (406 mm) apart on centre, and a 2x4 rim joist across their back
  ends, held to the table's back edge by two 6 in C-clamps;
- the board (loose, but nailed): a 600 mm length of 1x4 pine (19 x 89 mm actual) lying flat across the joists' tops,
  centred between them (its ends overhang each joist by 78 mm), held by one 3d box nail (1-1/4 in) into each joist,
  driven through its centre line;
- the pry bar (loose): a Vaughan B215 Original SuperBar, 381 mm long, lying on the table in front of the frame.

The board's own frame (used by the scorer): origin at its centre, x across its width (away from the robot), y along
its length (to the robot's left), z up out of its top face. At rest on its nails it is the world's axes.
"""

from __future__ import annotations

import math

MM = 0.001
INCH = 25.4 * MM
TABLE = 0.7475  # yam_arena.task_file.TABLE_TOP["station"]
TOP = TABLE  # the work surface: the station table's top
TABLE_X = (-0.19, 0.91)  # the station table's extent (yam_arena.station TABLE_CENTER, TABLE_SIZE)
TABLE_Y = (-0.65, 0.65)
CONTROL_HZ = 20.0

# -- colours (sRGB) --------------------------------------------------------------------------------------------------
SPF = (0.83, 0.70, 0.49)  # kiln-dried SPF 2x4s
PINE = (0.93, 0.85, 0.67)  # the 1x4 board, paler common pine
NAIL_HEAD = (0.30, 0.31, 0.33)  # bright steel heads, darkened by the wood around them
CLAMP_BLUE = (0.10, 0.22, 0.45)  # the C-clamps' painted cast-iron frames
CLAMP_STEEL = (0.62, 0.63, 0.66)  # their screws and swivel pads
BAR_STEEL = (0.20, 0.36, 0.62)  # blue powder-coated steel
BAR_BRIGHT = (0.70, 0.71, 0.73)  # the ground (bare steel) blade and claw
HIDDEN = SPF  # the nails' carrier inside the joists: never seen

# -- the frame (fixed) -----------------------------------------------------------------------------------------------
JOIST_W = 1.5 * INCH  # 38.1 mm: a 2x4 on edge
JOIST_H = 3.5 * INCH  # 88.9 mm
JOIST_X = (0.250, 0.8619)  # front (robot side) end, and back end against the rim joist
JOIST_L = JOIST_X[1] - JOIST_X[0]  # 612 mm
JOIST_Y = (-8.0 * INCH, 8.0 * INCH)  # centre lines: 16 in on centre, either side of the middle (right, left)
JOIST_TOP = TOP + JOIST_H
# The rim joist: a third 2x4 on edge across the joists' back ends (face-nailed into them, as floor and deck framing
# ends), flush with their outer faces: 406.4 + 38.1 = 444.5 mm long, its back face 10 mm in from the table's back edge.
RIM_X = (JOIST_X[1], JOIST_X[1] + JOIST_W)  # 861.9 to 900 mm
RIM_Y = (JOIST_Y[0] - JOIST_W / 2, JOIST_Y[1] + JOIST_W / 2)  # the joists' outer faces: -222.25 to +222.25 mm
# Two 6 in C-clamps hold the rim to the table's back edge (screw pad on the rim's top edge, fixed jaw under the table
# top), 120 mm either side of the middle: nothing is screwed into the robot's table. Drawn as the parts above the
# table: the frame's spine beyond the table's edge, its arm over the rim, the screw and its swivel pad.
CLAMP_Y = (-0.120, 0.120)
CLAMP_ARM_W = 22.0 * MM
# -- the board (loose; nailed) ---------------------------------------------------------------------------------------
BOARD_T = 0.75 * INCH  # 19.05 mm: a 1x4
BOARD_W = 3.5 * INCH  # 88.9 mm
BOARD_L = 0.600
BOARD_X = 0.400  # its centre: midway along the joists
BOARD_Y = 0.0
PINE_DENSITY = 450.0  # kg/m3, kiln-dried common pine
BOARD_MASS = round(BOARD_L * BOARD_W * BOARD_T * PINE_DENSITY, 3)  # 0.457 kg
BOARD_FRICTION = (0.5, 0.4)
# Dressed (S4S) lumber has its four long edges eased to about a 1/8 in radius; drawn as a 3 x 3 mm step along each.
# Where the board's bottom edge meets a joist, that ease leaves the small V where a flat bar's blade is started.
EASE = 3.0 * MM
SPAWN_CLEAR = 0.5 * MM  # above the joists at spawn, so nothing starts in contact

# -- the nails -------------------------------------------------------------------------------------------------------
# 3d box nails: 1-1/4 in (31.75 mm), 0.076 in (1.93 mm) shank, 7/32 in (5.6 mm) flat head, bright steel. One into each
# joist, square through the board's face on its centre line, head flush.
NAIL_LEN = 1.25 * INCH
NAIL_D = 0.076 * INCH
NAIL_HEAD_D = 7.0 / 32.0 * INCH
NAILS = {"left": JOIST_Y[1], "right": JOIST_Y[0]}  # name -> y in the world (into that joist's centre line)
NAIL_DY = JOIST_Y[1] - BOARD_Y  # 203.2 mm from the board's centre
EMBED = NAIL_LEN - BOARD_T  # 12.7 mm into the joist
# Withdrawal resistance: proportional to the length still in the joist, PULL_FULL with the whole EMBED in. The Wood
# Handbook's formula for a smooth nail freshly driven into side grain, p = 54.12 G^2.5 D L (N, mm), with SPF's
# specific gravity G = 0.42, gives 54.12 x 0.114 x 1.93 x 12.7 = 151 N. The real trial's nails should be checked with a
# spring scale (README).
PULL_FULL = 150.0  # N per nail
NAIL_K = 75_000.0  # N/m: the elastic give of one nail before it starts to slide (2 mm at full load)
OUT_AT = EMBED

# The nails as the simulator holds them (task.py): the board is glued to a carrier link inside the frame, which
# slides straight up (LIFT) and tips the board end to end about a line across it through its middle, on the joists'
# tops (TIP, about x). One
# spring each, sized so that the two nails are two independent springs: LIFT_K = 2 k, TIP_K = 2 k a^2 (a = NAIL_DY).
# Their targets follow each nail's slide every control step (task.py, ``_nails``).
LIFT_K = 2.0 * NAIL_K
TIP_K = 2.0 * NAIL_K * NAIL_DY ** 2
LIFT_DAMPING = 400.0  # N s/m (about 0.7 of critical for the board and carrier on LIFT_K)
TIP_DAMPING = 12.0  # N m s/rad
LIFT_RANGE = (-1.0 * MM, 30.0 * MM)
TIP_RANGE = (-0.12, 0.12)
CARRIER_MASS = 0.05  # each of the carrier's two links
CARRIER_Z = 25.0 * MM  # the carrier's hidden 6 mm boxes, inside each joist, this far above the table

# -- the pry bar (loose) ---------------------------------------------------------------------------------------------
# Vaughan B215 Original SuperBar (current part VN45001). The manufacturer's VGSS001 sell sheet specifies
# 15 in / 381 mm and 24 oz. Stock size, flare, slots and bend dimensions below are explicit photo-based
# approximations, not manufacturer drawings; see README. The blade is
# bent 15 deg off the shaft, away from the claw's curl: laid flat on the work with the shaft rising behind it, the bend
# is the heel the bar rocks on.
# Drawn in its own frame: x along the bar from the blade's tip (x = 0) to the claw, y across its width, z through its
# thickness; the claw curls toward -z. Each segment: (length, thickness, angle from +x toward -z in degrees, colour).
BAR_MAKE = "Vaughan"
BAR_MODEL = "B215 Original SuperBar"
BAR_PART_NUMBER = "VN45001"
BAR_LENGTH = 15.0 * INCH
BAR_MASS = 24.0 * 0.028349523125  # kg: published tool weight, not the website's shipping weight
BAR_W = 32.0 * MM  # approximate flat shaft stock, retained from the original simulation
BAR_END_W = 45.0 * MM  # approximate flared blade and rocker-head width from manufacturer photos
BLADE_BEND = 15.0  # deg: from the tip, the blade falls toward -z (the claw's side) to meet the shaft
BLADE_LEN = 48.0 * MM
_BAR_PROFILE = (
    (16.0 * MM, 1.6 * MM, BLADE_BEND, BAR_BRIGHT),  # the blade's edge
    (16.0 * MM, 2.8 * MM, BLADE_BEND, BAR_BRIGHT),
    (16.0 * MM, 4.2 * MM, BLADE_BEND, BAR_BRIGHT),
    (290.0 * MM, 6.0 * MM, 0.0, BAR_STEEL),  # the shaft
    (20.0 * MM, 6.0 * MM, 45.0, BAR_STEEL),  # the claw's curve
    (20.0 * MM, 6.0 * MM, 100.0, BAR_STEEL),
    (18.0 * MM, 5.0 * MM, 150.0, BAR_BRIGHT),
    (14.0 * MM, 3.5 * MM, 172.0, BAR_BRIGHT),  # the claw's tip
)


def _profile_span(segments) -> float:
    """End-to-end extent along the tool, including the bevel and the folded rocker head."""
    xs = []
    x = 0.0
    for i, (length, thick, angle, _color) in enumerate(segments):
        a = math.radians(angle)
        up = -(segments[3][1] - thick) / 2 if i < 3 else 0.0
        xs.extend(x + math.cos(a) * u + math.sin(a) * (up + v)
                  for u in (0.0, length) for v in (-thick / 2, thick / 2))
        x += math.cos(a) * length
    return max(xs) - min(xs)


# Size the straight shaft so the complete folded tool spans the published 381 mm. Summing segment lengths
# would measure the curved path, rather than the overall length. Blade and heel geometry are independent of this.
_SHAFT_EXTENSION = BAR_LENGTH - _profile_span(_BAR_PROFILE)
BAR_SEGMENTS = tuple((length + (_SHAFT_EXTENSION if i == 3 else 0.0), thick, angle, color)
                     for i, (length, thick, angle, color) in enumerate(_BAR_PROFILE))
STEEL_DENSITY = 7850.0
BAR_FRICTION = (0.9, 0.8)  # powder-coated steel on the grippers' rubber pads
# Where it lies: on the table in front of the frame, parallel to the board, its blade to the robot's left and its
# claw to the right, curled down: it rests on the claw and the blade's heel, the shaft sloping up toward the claw.
BAR_START_X = 0.120  # the bar's centre line, across the table
BAR_START_Y_TIP = 0.180  # the blade's tip
BAR_YAW = -math.pi / 2  # the bar's x (tip to claw) along the world's -y


SHAFT_T = BAR_SEGMENTS[3][1]
BLADE_SEGMENTS = 3  # the first three: ground thinner on the top (+z) face only, the underside flush with the shaft's


def bar_point(name: str) -> tuple:
    """A point on the bar in its own frame: "tip" (the middle of the blade's edge), "heel" (the underside of the bend
    where the blade meets the shaft: what the bar rocks on) or "grip" (the shaft's middle, BAR_GRIP_X from the tip,
    where the demo holds it)."""
    a0 = math.radians(BAR_SEGMENTS[0][2])
    if name == "tip":
        lift = -(SHAFT_T - BAR_SEGMENTS[0][1]) / 2  # the edge's centre, below the bar's centre line
        return (math.sin(a0) * lift, 0.0, math.cos(a0) * lift)
    x = z = 0.0
    for length, _t, ang, _c in BAR_SEGMENTS[:BLADE_SEGMENTS]:
        a = math.radians(ang)
        x, z = x + math.cos(a) * length, z - math.sin(a) * length
    if name == "heel":
        return (x, 0.0, z - SHAFT_T / 2)
    if name == "grip":
        return (BAR_GRIP_X, 0.0, z)
    raise KeyError(name)


BAR_GRIP_X = 0.200  # where the demo holds the plain shaft, measured from the blade tip

# -- scoring ----------------------------------------------------------------------------------------------------------
BAR_AT_NAIL_Y = 0.060  # the bar levers at a nail when its blade is within 6 cm of that joist's centre line, side to side
BAR_AT_NAIL_Z = 25.0 * MM  # ...its edge under the board's footprint and no more than this over the joists' tops
LOOSE = 6.0 * MM  # a nail counts as loosened when the board's underside is this far off the joist's top at it
FLAT_TILT = math.radians(20.0)  # laid flat: its face within 20 deg of facing straight up or down
ON_TABLE = 10.0 * MM  # laid on the table: its lowest point within 10 mm of the table's top...
ON_TABLE_TOP = BOARD_W + 15.0 * MM  # ...and its highest point within this of it (not propped on a joist)
CLEAR = 0.03  # both grippers' fingers at least 3 cm from the board at the end
STILL_V = 0.01  # m/s
SUCCESS_HOLD_S = 1.0


def joist_boxes() -> dict:
    """Each joist as an axis-aligned box in the world: name -> (low, high)."""
    out = {}
    for name, yc in (("right", JOIST_Y[0]), ("left", JOIST_Y[1])):
        out[name] = ((JOIST_X[0], yc - JOIST_W / 2, TOP), (JOIST_X[1], yc + JOIST_W / 2, JOIST_TOP))
    return out


def frame_shapes() -> list:
    """(centre, size, colour) boxes of the fixed frame in the world: the two joists, the rim joist across their back
    ends and the two C-clamps that hold the rim to the table's back edge."""
    out = []
    for lo, hi in joist_boxes().values():
        out.append((tuple((a + b) / 2 for a, b in zip(lo, hi)), tuple(b - a for a, b in zip(lo, hi)), SPF))
    out.append((((RIM_X[0] + RIM_X[1]) / 2, 0.0, TOP + JOIST_H / 2), (JOIST_W, RIM_Y[1] - RIM_Y[0], JOIST_H), SPF))
    edge = TABLE_X[1]
    top = JOIST_TOP
    for y in CLAMP_Y:
        w = CLAMP_ARM_W
        # the spine, just beyond the table's back edge, from under the table top up past the rim
        out.append(((edge + 0.010, y, TOP + (top + 0.040 - TOP - 0.060) / 2 + 0.030 - 0.030),
                    (0.016, w, top + 0.040 - (TOP - 0.060)), CLAMP_BLUE))
        # the arm over the rim, the screw down through it and the swivel pad on the rim's top edge
        out.append((((RIM_X[0] + 0.010 + edge + 0.018) / 2, y, top + 0.032), (edge + 0.018 - RIM_X[0] - 0.010, w, 0.016),
                    CLAMP_BLUE))
        out.append(((RIM_X[0] + 0.020, y, top + 0.014), (0.010, 0.010, 0.022), CLAMP_STEEL))
        out.append(((RIM_X[0] + 0.020, y, top + 0.002), (0.020, 0.020, 0.004), CLAMP_STEEL))
    return out


def board_start_centre() -> tuple:
    return (BOARD_X, BOARD_Y, JOIST_TOP + SPAWN_CLEAR + BOARD_T / 2)


def board_local_nail(name: str) -> tuple:
    """A nail's point on the board's underside, in the board's frame."""
    return (0.0, NAILS[name] - BOARD_Y, -BOARD_T / 2)


def bar_frame_rotation():
    """The bar's own frame (as drawn in BAR_SEGMENTS) in its assembly's base frame: the assembly is drawn lying at
    its resting tilt, turned about the bar's width by -tilt."""
    t, _ = bar_rest()
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))


def bar_shapes() -> list:
    """Solid boxes forming the B215, including two open nail notches and a closed keyhole.

    Slots are actual gaps between collision pieces. Their stepped outlines and the flared shoulders approximate
    the photographed forging; no black decals or collision boxes fill the holes.
    """
    out = []
    x = z = 0.0
    for k, (length, thick, ang, colour) in enumerate(BAR_SEGMENTS):
        a = math.radians(ang)
        dx, dz = math.cos(a), -math.sin(a)
        # the blade is ground on its top face: its underside runs on from the shaft's (its centre lowered along the
        # segment's +z normal, (sin a, cos a) in x, z)
        up = -(BAR_SEGMENTS[3][1] - thick) / 2 if k < 3 else 0.0
        # Each interval is (start, end, outside width, central opening width).
        width = BAR_W if k == 3 else BAR_END_W
        bands = [(0.0, length, width, 0.0)]
        if k == 0:  # V-shaped nail notch at the flat blade's end
            bands = [(0.0, 4 * MM, width, 8 * MM), (4 * MM, 8 * MM, width, 4 * MM),
                     (8 * MM, length, width, 0.0)]
        elif k == 3:  # flared shoulder, closed keyhole nail slot, then the unbroken gripping shaft
            bands = [(0.0, 20 * MM, BAR_END_W, 0.0), (20 * MM, 28 * MM, BAR_END_W, 10 * MM),
                     (28 * MM, 38 * MM, BAR_END_W, 4 * MM), (38 * MM, 40 * MM, BAR_END_W, 0.0),
                     (40 * MM, 55 * MM, (BAR_END_W + BAR_W) / 2, 0.0),
                     (55 * MM, length, BAR_W, 0.0)]
        elif k == len(BAR_SEGMENTS) - 1:  # open notch in the curved nail claw
            bands = [(0.0, length - 8 * MM, width, 0.0),
                     (length - 8 * MM, length - 4 * MM, width, 4 * MM),
                     (length - 4 * MM, length, width, 8 * MM)]
        for start, end, outside, gap in bands:
            mid = (start + end) / 2
            strips = [(0.0, outside)] if not gap else [(-(outside + gap) / 4, (outside - gap) / 2),
                                                                    ((outside + gap) / 4, (outside - gap) / 2)]
            for y, strip_width in strips:
                out.append(((x + dx * mid + math.sin(a) * up, y, z + dz * mid + math.cos(a) * up),
                            (end - start, strip_width, thick), a, colour))
        x, z = x + dx * length, z + dz * length
    return out


def bar_corners_local() -> list:
    """Every corner of every bar segment in the bar's frame (x, z only: the bar is flat across y)."""
    pts = []
    for (cx, _cy, cz), (lx, _ly, t), a, _c in bar_shapes():
        ca, sa = math.cos(a), math.sin(a)
        for u in (-lx / 2, lx / 2):
            for v in (-t / 2, t / 2):
                pts.append((cx + u * ca + v * sa, cz - u * sa + v * ca))
    return pts


def bar_rest() -> tuple:
    """How the bar lies on the table, curl down: (tilt about its width in rad, z of its frame's origin above the
    table). It rests on the blade's edge and the claw's lowest corner; the tilt raises the claw end."""
    pts = bar_corners_local()
    tip = min((p for p in pts if p[0] < 2 * MM), key=lambda p: p[1])
    best = None
    for t in [i * 0.0005 for i in range(-400, 401)]:
        c, s = math.cos(t), math.sin(t)
        zs = [p[0] * s + p[1] * c for p in pts]  # turned about y by -t: the +x end raised for t > 0
        low = min(zs)
        tip_z = tip[0] * s + tip[1] * c
        far = min(z for p, z in zip(pts, zs) if p[0] > 0.2)
        err = abs(tip_z - low) + abs(far - low)
        if best is None or err < best[0]:
            best = (err, t, -low)
    return best[1], best[2]
