"""Every dimension and threshold of the task in one place, and the shapes as plain rows. NumPy only.

Real parts, real size. Sources: a 12 mm boring bar shank (150 mm long, 0.13 kg of steel), a Weldon-style flat on the
shank for a set screw, a 12.4 mm holder bore (0.4 mm over the shank), M6 x 1.0 set screws, a 3 mm hex T-handle key.
The 4-diameter overhang rule of thumb for a steel bar is the one machinists quote (Sandvik Coromant and Kennametal
guides give 4 x D for steel bars, 7 x D for carbide). Every other size (the block, the chuck, the 22 mm bore, the
rack, the key stand) is our estimate of an ordinary bench lathe set-up, NOT measured on a machine.

Frames, millimetres unless named. World: x forward from the line through the two arm bases, y to the left, z up; the
bench top is TOP. The lathe's axis runs along y at x = X_AX and z = Z_AX over the bench; the chuck is at +y, the
tool post (holder) near y 0, and the operator side is the robot's side (-x).

BAR frame (the loose ``bar``): origin on the shank's axis at its middle, local x along the shank toward the cutting
end, z the flat's normal, y = z cross x (so the cutting insert, at local +y, is 90 deg round from the flat). Spawned
with yaw +90 deg: local x is world +y (cutting end toward the chuck), the flat faces up, the insert faces -x (the
operator side), which is the one right way round. DEMO_VARIANT=flat_down builds the shapes rolled 180 deg about
the shank (FLAT_N and INSERT_D below say where the flat and the insert are in the bar frame).
KEY frame (the loose ``key``): origin at the hex tip, z up along the shaft, crossbar along local x.
"""

from __future__ import annotations

import math
import os

import numpy as np

TOP = 0.7475  # the station bench top
MM = 0.001
CONTROL_HZ = 20.0
DT = 1.0 / CONTROL_HZ

STEEL = (0.62, 0.64, 0.68)
DARK = (0.22, 0.23, 0.26)
GREY = (0.45, 0.47, 0.50)
GOLD = (0.88, 0.72, 0.20)
BLACK = (0.08, 0.08, 0.09)
WOOD = (0.55, 0.42, 0.28)
RED = (0.72, 0.16, 0.14)

VARIANT = os.environ.get("DEMO_VARIANT", "")
ROLL_DEG = 180.0 if VARIANT == "flat_down" else 0.0

# -- the lathe ------------------------------------------------------------------------------------------------------
X_AX = 250.0  # world x of the lathe axis
Z_AX = 45.0  # height of the axis over the bench
OPERATOR = np.array([-1.0, 0.0, 0.0])  # from the axis toward the operator side (the robot's side)
CHUCK_DIR = np.array([0.0, 1.0, 0.0])

# the holder block (tool post side): a steel block with a through-bore on the axis
HOLD_Y = (-20.0, 40.0)  # the back face (toward the operator end) and the FACE toward the chuck
Y_BACK, Y_FACE = HOLD_Y
HOLD_W = 50.0  # across x
HOLD_H = 70.0  # top of the block
BORE_R = 6.2  # 12.4 mm bore
RING_R_OUT = 10.0  # the bore's wall: 3.8 mm of steel, 20 flats
RING_N = 20
FUNNEL = ((37.0, 38.5, 7.2), (38.5, 40.0, 8.5))  # (y0, y1, bore radius) the chamfer, as two steps at the face
SCREW_Y = (-5.0, 25.0)  # the two set screws (x = X_AX): 30 mm apart, both over the bar
SLAB_Z = (Z_AX + RING_R_OUT, HOLD_H)  # the block over the bore: 55 to 70
SOCKET = 5.2  # a 3 mm hex key fits a 5.2 mm square socket hole in the top face (1 mm a side)
SOCKET_STEPS = ((55.0, 64.0, SOCKET), (64.0, 67.0, 7.6), (67.0, 70.0, 10.0))  # the socket and its two-step mouth

# the workpiece and chuck (fixed)
WP_FACE = 240.0  # the workpiece's face toward the tool post
WP_R = 25.0
WP_LEN = 70.0
HOLE_R = 11.0  # the 22 mm blind bore
HOLE_DEPTH = 40.0
CHUCK_Y = (WP_FACE + WP_LEN, WP_FACE + WP_LEN + 45.0)
CHUCK_R = 40.0

# the rack (fixed): two blades the bar lies on
RACK_Y = (70.0, 150.0)
RACK_X = (3.0, 9.0)  # each blade from this far off the axis, on both sides
BAR_R = 6.0
RACK_TOP = Z_AX - math.sqrt(BAR_R ** 2 - RACK_X[0] ** 2)  # the bar's axis is then at Z_AX

# -- the bar ----------------------------------------------------------------------------------------------------------
BAR_L = 150.0
BAR_MASS = 0.130
BAR_FRICTION = 0.3
FLAT_Z = 5.2  # the flat's plane: 0.8 mm under the top of the round
SHANK_N = 8  # the round shank as 8 rectangles turned 22.5 deg apart
SHANK_T = 3.2
INSERT_X = (70.0, 75.0)  # from the bar's middle toward the cutting end: the insert sits in the cutting end, flush with the tip face
INSERT_Y = (4.0, 7.0)  # and 1 mm proud of the shank's side (local +y)
INSERT_Z = 4.0
BAR_START_Y = 130.0  # the bar's middle at spawn: the plain end 55, the cutting end 205
BAR_YAW = math.pi / 2

# -- the key ----------------------------------------------------------------------------------------------------------
KEY_MASS = 0.060
KEY_SHAFT_R = 1.6
KEY_SHAFT_LEN = 60.0
KEY_BAR_R = 4.0
KEY_BAR_LEN = 90.0
KEY_BAR_Z = KEY_SHAFT_LEN + KEY_BAR_R  # the crossbar's axis over the tip
KEY_XY = (190.0, -120.0)  # the stand
STAND_SIZE = 30.0
STAND_FLOOR = 4.0  # the key's tip rests here, in its hole
STAND_TOP = 28.0
STAND_HOLE = 3.8
STAND_MOUTH = ((28.0, 29.5, 8.0), (29.5, 31.0, 12.0))
KEY_FRICTION = 0.4

# -- the set screws: the software thread model -----------------------------------------------------------------------
PITCH = 1.0  # mm per turn (M6 x 1.0)
BACKOFF = 0.2  # a screw's tip starts this far over the top of a seated round bar
TIGHT_PAST = 0.4  # turns past contact with the bar at which a screw is TIGHT
TIGHT_KEEP = 0.2  # a tight screw stays tight down to this many turns past contact
CAP_PAST = 0.5  # the screw bottoms out this far past contact (a tight screw cannot be turned further)
ENGAGE_XY = 1.5  # mm: the key's tip from a socket's axis
ENGAGE_TILT = 6.0  # deg: the key's shaft from the vertical
ENGAGE_DEPTH = 7.0  # mm: the tip this far under the top face ("pushed down")
FLAT_OK_DEG = 10.0  # flat facing up within this: the screw bears on the flat, else on the round
BAR_UNDER_XY = 4.0  # the bar's axis within this of a screw's line, and covering it by 3 mm
BAR_UNDER_ENDS = 3.0
WELD_RELEASE_AT = TIGHT_KEEP

# -- scoring ----------------------------------------------------------------------------------------------------------
S_BAND = (45.0, 48.0)  # the stick-out that scores 1.0 (reach the 40 mm depth with 5 mm to spare ... 4 x 12 mm)
FLAT_UP_DEG = 10.0
INSERT_DEG = 15.0
TOWARD_DEG = 30.0  # the cutting end within this of +y: "toward the chuck"
IN_BORE_MM = 1.5  # the bar's axis within this of the bore's at both faces
CLEAR_M = 0.030
WIN_STEPS = 20
STILL_MM = 1.0
OFF_TABLE_M = 0.05


def m(v):
    return tuple(float(x) * MM for x in v)


def rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


# where the flat and the insert are in the bar frame (rolled by the variant)
FLAT_N = rx(math.radians(ROLL_DEG)) @ np.array([0.0, 0.0, 1.0])
INSERT_D = rx(math.radians(ROLL_DEG)) @ np.array([0.0, 1.0, 0.0])


def world(p_mm):
    """A lathe-frame point (mm: x, y over the bench, z over the bench) in the world (m)."""
    return np.array([p_mm[0] * MM, p_mm[1] * MM, TOP + p_mm[2] * MM])


# -- shapes: ("box", centre, size, (yaw, pitch, roll) rad, colour) and ("cyl", centre, radius, height, axis, colour) --
def ring_y(y0, y1, r_in, r_out, n, colour, cx=0.0, cz=0.0):
    """A ring of n boxes round an axis along y (inner faces at r_in), from y0 to y1, about (cx, cz)."""
    out = []
    rc = (r_in + r_out) / 2
    tang = 2 * r_out * math.tan(math.pi / n) + 0.4
    for k in range(n):
        phi = 2 * math.pi * k / n
        out.append(("box", (cx + rc * math.cos(phi), (y0 + y1) / 2, cz + rc * math.sin(phi)),
                    (r_out - r_in, y1 - y0, tang), (0.0, -phi, 0.0), colour))
    return out


def holder_rows():
    """The holder block in the world's x (centre X_AX), y and z over the bench, all in mm (x is absolute)."""
    rows = []
    x0, x1 = X_AX - HOLD_W / 2, X_AX + HOLD_W / 2
    yc, yl = (Y_BACK + Y_FACE) / 2, Y_FACE - Y_BACK
    rows.append(("box", (X_AX, yc, (Z_AX - RING_R_OUT) / 2), (HOLD_W, yl, Z_AX - RING_R_OUT), (0, 0, 0), STEEL))
    zc, zl = Z_AX, 2 * RING_R_OUT
    for sx in (-1.0, 1.0):
        w = HOLD_W / 2 - RING_R_OUT
        rows.append(("box", (X_AX + sx * (RING_R_OUT + w / 2), yc, zc), (w, yl, zl), (0, 0, 0), STEEL))
    y_main = FUNNEL[0][0]
    rows += ring_y(Y_BACK, y_main, BORE_R, RING_R_OUT, RING_N, STEEL, X_AX, Z_AX)
    for y0, y1, r in FUNNEL:
        rows += ring_y(y0, y1, r, RING_R_OUT, RING_N, STEEL, X_AX, Z_AX)
    return rows, (x0, x1)


def socket_plates():
    """(z0, z1, hole size) of the layers of the top slab; each layer has a square hole at each screw."""
    return SOCKET_STEPS


def workpiece_rows():
    rows = ring_y(WP_FACE, WP_FACE + HOLE_DEPTH, HOLE_R, WP_R, 16, GREY, X_AX, Z_AX)
    rows.append(("cyl", (X_AX, WP_FACE + HOLE_DEPTH + (WP_LEN - HOLE_DEPTH) / 2, Z_AX), WP_R, WP_LEN - HOLE_DEPTH, "Y",
                 GREY))
    rows.append(("cyl", (X_AX, (CHUCK_Y[0] + CHUCK_Y[1]) / 2, Z_AX), CHUCK_R, CHUCK_Y[1] - CHUCK_Y[0], "Y", DARK))
    rows.append(("cyl", (X_AX, CHUCK_Y[1] + 30.0, Z_AX), 30.0, 60.0, "Y", DARK))
    return rows


def rack_rows():
    ycen, ylen = (RACK_Y[0] + RACK_Y[1]) / 2, RACK_Y[1] - RACK_Y[0]
    rows = []
    for sx in (-1.0, 1.0):
        xc = X_AX + sx * (RACK_X[0] + RACK_X[1]) / 2
        rows.append(("box", (xc, ycen, RACK_TOP / 2), (RACK_X[1] - RACK_X[0], ylen, RACK_TOP), (0, 0, 0), WOOD))
    return rows


def stand_rows():
    """The key stand's boxes, in mm about the stand's own origin (middle of its bottom face)."""
    return [("box", (0.0, 0.0, STAND_FLOOR / 2), (STAND_SIZE, STAND_SIZE, STAND_FLOOR), (0, 0, 0), WOOD)]


def bar_rows(roll_deg: float = ROLL_DEG):
    """The bar's shapes in its own frame (mm): 8 rectangles for the round shank (the three that reach over the flat
    are cut to it), a box filling the flat's shoulders, the insert."""
    hw = math.sqrt(BAR_R ** 2 - (SHANK_T / 2) ** 2)
    rows = []
    roll = math.radians(roll_deg)
    for k in range(SHANK_N):
        a = math.pi * k / SHANK_N
        w_lo, w_up = -hw, hw
        s, c = math.sin(a), abs(math.cos(a))
        if s > 1e-9:
            w_up = min(hw, (FLAT_Z - SHANK_T / 2 * c) / s)
        mid, half = (w_up + w_lo) / 2, (w_up - w_lo) / 2
        yz = np.array([mid * math.cos(a), mid * math.sin(a)])
        yz = np.array([[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]]) @ yz
        rows.append(("box", (0.0, float(yz[0]), float(yz[1])), (BAR_L, 2 * half, SHANK_T), (0.0, 0.0, a + roll),
                     STEEL))
    # the flat's shoulders: a box from z 3.5 up to the flat, as wide as the flat's chord (the circle's own width there)
    half_chord = math.sqrt(BAR_R ** 2 - FLAT_Z ** 2)
    zc = (3.5 + FLAT_Z) / 2
    yz = np.array([[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]]) @ np.array([0.0, zc])
    rows.append(("box", (0.0, float(yz[0]), float(yz[1])), (BAR_L, 2 * half_chord, FLAT_Z - 3.5), (0.0, 0.0, roll),
                 STEEL))
    ix = (INSERT_X[0] + INSERT_X[1]) / 2  # (GPU runs 2 to 4: a stray - BAR_L / 2 put it at the middle, where it stopped on the funnel's 7.2 mm step)
    iy = (INSERT_Y[0] + INSERT_Y[1]) / 2
    yz = np.array([[math.cos(roll), -math.sin(roll)], [math.sin(roll), math.cos(roll)]]) @ np.array([iy, 0.0])
    rows.append(("box", (ix, float(yz[0]), float(yz[1])),
                 (INSERT_X[1] - INSERT_X[0], INSERT_Y[1] - INSERT_Y[0], INSERT_Z), (0.0, 0.0, roll), GOLD))
    return rows


def key_rows():
    return [("cyl", (0.0, 0.0, KEY_SHAFT_LEN / 2), KEY_SHAFT_R, KEY_SHAFT_LEN, "Z", STEEL),
            ("cyl", (0.0, 0.0, KEY_BAR_Z), KEY_BAR_R, KEY_BAR_LEN, "X", RED)]


# -- thread geometry (pure) --------------------------------------------------------------------------------------------
Z_SCREW0 = Z_AX + BAR_R + BACKOFF  # a screw's tip at its start (z over the bench, mm)
HOLDER_TOP = HOLD_H


def stickout_mm(tip_y_m: float) -> float:
    """How far the bar's cutting-end face is past the holder's chuck-side face along the axis (mm)."""
    return tip_y_m / MM - Y_FACE

# GPU run 6: pushed by the closed pad block, the bar stopped 4.9 mm short of where the commanded tip position put
# it (arm compliance under the push). The demo aims that much further and the offline model stops that much short.
GPU_PUSH_LAG = 4.9
