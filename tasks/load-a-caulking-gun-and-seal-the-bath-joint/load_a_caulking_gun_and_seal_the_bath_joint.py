"""On the station table lies an empty caulking gun, the kind used to seal baths and sinks: a red steel skeleton frame
with a pistol grip and a hinged trigger, an open half-round cradle for a sealant cartridge and a front yoke with a
notch for the cartridge's nozzle. It lies level along the table, nozzle end toward the left, on its grip and on a
wooden rest block under its front. Its smooth steel plunger rod comes out through the back of the frame and ends in a
hook; the rod is part way in. On top of the frame, behind the cradle, is the gun's thumb release lever. To the left of
the gun, along the table's left edge, is the white rim of a bath against a tiled wall; the joint where the rim meets
the tiles is not sealed yet. In front of the gun, on a cardboard tray, lies a new 310 ml cartridge of grey sanitary
silicone, its nozzle already cut.

Task: load the caulking gun, seal the joint between the bath and the tiles, and put the gun down.

How the gun works here (the mechanism of a real smooth-rod gun):
- The trigger is a hinged lever with a return spring. Squeezed toward the grip, its short upper arm pushes the gripping
  plate forward, and the plate carries the rod forward with it (10:1 at the middle of the trigger, about 2.5 mm of rod
  per full squeeze). Let go, the springs return the trigger and the plate, which slides back along the rod while the
  rod stays where it is.
- The locking plate behind the frame lets the rod slide in freely but holds it against coming back out, unless the
  thumb release lever is pressed down.
- The two plates' friction grip on the smooth rod is modelled by one-way stops on the slides (the plate on the rod, the
  rod in the frame) that this file keeps just behind the plate and behind the rod.
- The rod's push plate pushes the cartridge's piston. The silicone resists the piston with 150 N (its flow pressure):
  far more than a push on the rod's hook can give, easily given through the trigger's leverage.
- Flow approximation: the sealant pushed out (the piston's advance times the bore's area) leaves the nozzle's tip a
  quarter second later and is laid along the tip's path; it counts where the bead it makes lies in the joint, touching
  both the rim and the tiles.

Run it:  inspect-robots run --embodiment yam_arena -E task=load_a_caulking_gun_and_seal_the_bath_joint.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, rotation, set_slider_drive, slider)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the gun's frame: origin on the rod's axis at the cradle's back plate (inner face), +y toward the nozzle ---------
GX, GY = 0.300, 0.020
AXIS_Z = TOP + 0.090  # the axis this high with the grip's foot on the table
GUN0 = np.array([GX, GY, AXIS_Z])
R_TROUGH = 0.028  # the cradle's strips sit on this radius (their inner faces 1.5 mm inside it)
YOKE_Y0, YOKE_T = 0.236, 0.004  # the front yoke's inner face and thickness
GUN_MASS = 0.36
HANDLE_Y = (-0.074, -0.046)  # the pistol grip's back and front faces (its back flush with the housing's)
FOOT_Z = -0.090
# -- the trigger: hinged about x below the rod; squeezing turns it negative (its foot back, its upper arm forward) --
PIVOT_T = np.array([0.0, -0.016, -0.030])  # gun frame
TRIG_TILT = 18.0  # its blade leans forward this much at rest
TRIG_MAX = 36.0  # full squeeze (its stop)
NUB = (-0.005 * math.sin(math.radians(18.0)), 0.005 * math.cos(math.radians(18.0)))  # upper arm: 5 mm above the pivot
TRIG_MASS = 0.070
TRIG_SPRING, TRIG_TARGET = 0.004, 25.0  # N m per degree; the target past the rest stop gives the preload
BAR_R, BAR_DZ = 0.006, -0.036  # the trigger's rounded finger rest: a 12 mm round bar 36 mm below the pivot
_TILT = math.radians(TRIG_TILT)
BAR_DY = (-BAR_DZ) * math.tan(_TILT) + 0.004 / math.cos(_TILT) + 0.0052 - BAR_R  # its front 5.2 mm ahead of the blade
# -- the gripping (drive) plate: on the rod behind the back plate, pushed forward by the trigger's upper arm ---------
PLATE_Y = -0.0138  # its middle at rest (back face 0.25 mm in front of the trigger's upper arm)
PLATE_TRAVEL = 0.0036
PLATE_K, PLATE_PRE = 1000.0, 0.005  # its return spring: N/m, and its preload (5 N at rest)
PLATE_MASS = 0.020
CLUTCH_REST, CLUTCH_GAP = 0.00005, 0.00005  # the plate is back at rest within 0.05 mm; its stop sits 0.05 mm behind it
# -- the plunger rod: its origin at the push plate's front face, which sits S (m) in front of the back plate --------
ROD_BACK = 0.300  # from the plate's face back to the rod's end, where the hook starts
S0 = 0.080  # where the plate starts (the rod part way in)
S_MAX = 0.226
HOOK_GRIP = (0.0, -ROD_BACK - 0.018, 0.040)  # the hand's TCP on the hook's top bar, rod frame
ROD_MASS = 0.10
PUSH_R = 0.020  # the push plate's radius
# -- the locking plate (the ratchet against coming back out) -----------------------------------------------------------
LOCK_D, LOCK_GAP = 12.0, 0.0001  # N s/m of slide friction; the locking plate's stop sits 0.1 mm behind the rod
# -- the release lever: hinged about x behind the back plate, its pad pressed down -----------------------------------
PIVOT = np.array([GX, GY - 0.030, AXIS_Z + 0.034])
RELEASE_DEG = -6.0  # pressed at least this far: the rod is free
# -- the cartridge: its origin at the middle of its tube, nozzle along +y ----------------------------------------------
TUBE_R, TUBE_L = 0.0245, 0.205  # 49 mm across, 205 mm long (a 310 ml C20 cartridge)
BORE_R = 0.023  # 46 mm bore
A_BORE = math.pi * BORE_R ** 2  # 1662 mm^2: 1.66 ml per mm of piston travel
SHOULDER = 0.006
NOZZLE = ((0.009, 0.030), (0.007, 0.030), (0.0052, 0.030), (0.0035, 0.030))  # cut for a 7 mm opening
TIP = TUBE_L / 2 + SHOULDER + sum(n[1] for n in NOZZLE)  # the nozzle's tip, from the middle
SEAT_Y = YOKE_Y0 - TUBE_L / 2 - SHOULDER  # the middle, seated (shoulder against the yoke)
SEAT_Z = -(R_TROUGH - 0.0015 - TUBE_R)  # the cartridge's axis a little below the gun's, lying in the cradle
TUBE_MASS = 0.355
SOLID_FROM = 0.045  # the tube's collision starts this far in from its back end (behind it: the piston's travel)
PISTON_IN, PISTON_T = 0.004, 0.008  # the piston's back face 4 mm in from the tube's end; 8 mm thick
PISTON_Y = -TUBE_L / 2 + PISTON_IN + PISTON_T / 2  # its middle, tube frame
PISTON_MASS = 0.015
PISTON_TRAVEL = SOLID_FROM - PISTON_IN - PISTON_T - 0.002
SEAL_F = 150.0  # N: what it takes to move the piston (the silicone's flow pressure, about 0.09 MPa)
SEAL_K = 1.0e6  # N/m: below SEAL_F the piston is held where it is (the task file moves the hold along as it yields)
TRAY = np.array([0.170, 0.175])
TUBE0 = np.array([TRAY[0], TRAY[1], TOP + 0.036])
# -- the bath rim and the tiled wall ---------------------------------------------------------------------------------
BX0, BX1 = 0.140, 0.480
RIM_Y0, WALL_Y = 0.390, 0.432  # the rim's front face; the wall's face (the joint runs along it)
RIM_Z = TOP + 0.100
JOINT_X0, JOINT_X1 = 0.185, 0.377
N_BIN = int(round((JOINT_X1 - JOINT_X0) * 1000))  # 1 mm lengths of the joint
# -- the sealant and the bead --------------------------------------------------------------------------------------------
BEAD_A = math.pi * 0.003 ** 2  # a 6 mm bead: 28.3 mm^2, 0.0283 ml per mm (310 ml = 11 m)
A_MIN = math.pi * 0.002 ** 2  # a length counts as sealed with a bead at least 4 mm across (12.6 mm^2)
TAU = 0.25  # s: the sealant leaves the nozzle this much behind the piston (a first-order lag)
TOUCH = 0.002  # the bead touches a surface if the tip is within its radius plus this
SEG_LEN, SEG_STEPS, SEG_V = 0.005, 6, 0.4e-6  # a segment is laid every 5 mm of tip travel, 0.3 s, or 0.4 ml
FIL_R, FIL_L, N_FIL = 0.0065, 0.010, 64  # the laid bead in the corner: rounds sunk to show its section
N_BLOB, BLOB_R = 12, 0.004  # sealant laid anywhere else
COVER_FULL, GAP_FULL, COVER_HALF = 0.95, 2, 0.50  # full: 95% covered, no gap over 2 mm; half: 50% covered
HIDE = (0.300, 0.410, TOP + 0.050)  # inside the rim, where the unlaid bead waits
# -- put down -------------------------------------------------------------------------------------------------------------
CLEAR = 0.06  # each gripper's TCP at least this far from the gun's frame, trigger and rod
DOWN_Z, DOWN_TILT = 0.020, 25.0  # the grip's foot within 2 cm of the table, the gun tilted at most 25 degrees

RED, RED_DARK, ZINC, STEEL, RUBBER = (0.72, 0.10, 0.08), (0.50, 0.07, 0.06), (0.80, 0.82, 0.84), (0.62, 0.64, 0.67), \
    (0.07, 0.07, 0.07)
WHITE, LABEL, LABEL2, NOZ = (0.95, 0.95, 0.93), (0.10, 0.34, 0.70), (0.98, 0.80, 0.15), (0.90, 0.90, 0.86)
ACRYLIC, TILE, GROUT = (0.97, 0.97, 0.96), (0.86, 0.91, 0.93), (0.58, 0.60, 0.60)
SILICONE = (0.42, 0.44, 0.47)  # grey sanitary silicone
WOOD, WOOD_DARK, CARD, CARD_DARK = (0.74, 0.57, 0.36), (0.56, 0.41, 0.24), (0.68, 0.52, 0.33), (0.52, 0.38, 0.22)

SUCCESS_HOLD_S = 1.0


def _qx(deg: float) -> tuple[float, float, float, float]:
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


def _strip(a_deg: float, y0: float, y1: float, width: float, color, collide=True) -> BoxShape:
    """One flat strip of the half-round cradle, ``a_deg`` round from the bottom (toward +x)."""
    a = math.radians(a_deg)
    rc = R_TROUGH
    c = (rc * math.sin(a), (y0 + y1) / 2, -rc * math.cos(a))
    return BoxShape(c, (width, y1 - y0, 0.003), color, collide=collide,
                    quat_xyzw=(0.0, -math.sin(a / 2), 0.0, math.cos(a / 2)))  # tangent: rising toward +x for a > 0


def _gun() -> Body:
    s = []
    w = 2 * R_TROUGH * math.tan(math.radians(33.4 / 2))
    for a in (-33.4, 0.0, 33.4):  # the cradle: three strips, up to 50 degrees round each side
        s.append(_strip(a, 0.000, YOKE_Y0, w, RED))
    for y in (0.060, 0.120, 0.180):  # pressed stiffening ribs
        s.append(_strip(0.0, y - 0.004, y + 0.004, 0.012, RED_DARK, collide=False))
    # the back plate (the rod passes through a hole in it)
    s.append(BoxShape((0.0, -0.00125, 0.010), (0.064, 0.0025, 0.044), RED))  # (open below, over the trigger)
    # the drive housing behind it: two side cheeks, a rear wall and a floor over the grip
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.012, -0.03625, -0.010), (0.003, 0.0675, 0.052), RED))
    s.append(BoxShape((0.0, -0.072, -0.010), (0.027, 0.004, 0.052), RED))
    s.append(BoxShape((0.0, -0.059, -0.0345), (0.027, 0.026, 0.003), RED_DARK))
    s.append(CylShape((0.0, -0.078, 0.0), 0.008, 0.008, RED_DARK, axis="Y"))  # the rod's bearing boss
    s.append(BoxShape((0.0, -0.018, 0.022), (0.030, 0.026, 0.004), RED_DARK))  # the release lever's bracket
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.012, -0.024, 0.019), (0.003, 0.016, 0.010), RED_DARK))
    # the gripping plate's return spring round the rod (drawn only)
    s.append(CylShape((0.0, -0.0075, 0.0), 0.0065, 0.009, STEEL, collide=False, axis="Y"))
    # the front yoke, with a V notch the nozzle drops into
    yc = YOKE_Y0 + YOKE_T / 2
    s.append(BoxShape((0.0, yc, -0.0215), (0.064, YOKE_T, 0.021), RED))
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.0215, yc, 0.000), (0.021, YOKE_T, 0.022), RED))
        s.append(BoxShape((sx * 0.0240, yc, 0.0215), (0.016, YOKE_T, 0.021), RED))
    # the pistol grip, black rubber over steel, under the housing's back
    hy = (HANDLE_Y[0] + HANDLE_Y[1]) / 2
    s.append(BoxShape((0.0, hy, -0.060), (0.024, HANDLE_Y[1] - HANDLE_Y[0], 0.048), RUBBER))
    s.append(BoxShape((0.0, hy, FOOT_Z + 0.003), (0.032, 0.032, 0.006), RUBBER))
    s.append(BoxShape((0.0, HANDLE_Y[0] - 0.005, FOOT_Z + 0.00725), (0.024, 0.010, 0.0145), RUBBER))  # its flared heel
    for z in (-0.045, -0.058, -0.071):
        s.append(BoxShape((0.0, HANDLE_Y[0] - 0.0006, z), (0.020, 0.0012, 0.004), (0.15, 0.15, 0.15), collide=False))
    return Body.build("gun", pos=tuple(GUN0.tolist()), mass=GUN_MASS, shapes=s, friction=(1.3, 1.2), color=RED,
                      com=(0.0, 0.050, -0.030), solver_iterations=64)


def _trigger() -> Body:
    """The trigger: pressed steel, hinged below the rod. Its blade leans forward; above the pivot a short arm (a 4 mm
    round nub, 5 mm from the pivot) pushes the gripping plate."""
    tilt = math.radians(TRIG_TILT)
    d = np.array([0.0, math.sin(tilt), -math.cos(tilt)])  # down the blade at rest
    s = [CylShape((0.0, 0.0, 0.0), 0.004, 0.014, RED_DARK, collide=False, axis="X"),  # the pivot boss
         CylShape((0.0, NUB[0], NUB[1]), 0.002, 0.012, STEEL, axis="X"),  # the upper arm's nub
         BoxShape((0.0, NUB[0] / 2, NUB[1] / 2), (0.010, 0.004, 0.006), RED_DARK, collide=False)]
    c = d * 0.0295
    s.append(BoxShape(tuple(c.tolist()), (0.014, 0.008, 0.050), RED, quat_xyzw=_qx(TRIG_TILT)))  # the blade
    s.append(CylShape((0.0, BAR_DY, BAR_DZ), BAR_R, 0.014, RED_DARK, axis="X"))  # the rounded finger rest
    for t in (0.024, 0.044):  # finger grooves (drawn)
        p = d * t + np.array([0.0, math.cos(tilt), math.sin(tilt)]) * 0.0041
        s.append(BoxShape(tuple(p.tolist()), (0.012, 0.0008, 0.0025), RED_DARK, collide=False, quat_xyzw=_qx(TRIG_TILT)))
    return Body.build("trigger", pos=tuple((GUN0 + PIVOT_T).tolist()), mass=TRIG_MASS, shapes=s, friction=(1.0, 0.9),
                      color=RED, com=(0.0, 0.008, -0.025), solver_iterations=64)


def _plate() -> Body:
    """The gripping plate: a steel plate with a hole for the rod, between the housing's cheeks."""
    s = [BoxShape((0.0, 0.0, -0.008), (0.018, 0.003, 0.040), STEEL)]
    return Body.build("plate", pos=(GX, GY + PLATE_Y, AXIS_Z), mass=PLATE_MASS, shapes=s, friction=(0.6, 0.5),
                      color=STEEL, solver_iterations=64)


def _release() -> Body:
    """The thumb release lever: a pressed-steel lever pivoted behind the back plate, its ribbed pad forward."""
    s = [CylShape((0.0, 0.0, 0.0), 0.004, 0.034, ZINC, axis="X"),
         BoxShape((0.0, 0.014, 0.004), (0.030, 0.024, 0.004), ZINC)]
    for y in (0.008, 0.014, 0.020):
        s.append(BoxShape((0.0, y, 0.0063), (0.026, 0.0015, 0.0006), STEEL, collide=False))
    return Body.build("release", pos=tuple(PIVOT.tolist()), mass=0.015, shapes=s, friction=(1.0, 0.9), color=ZINC)


def _rod() -> Body:
    """The plunger rod: the push plate, the rod and the D-shaped hook at its back end."""
    rb = -ROD_BACK
    s = [CylShape((0.0, -0.0025, 0.0), PUSH_R, 0.005, ZINC, axis="Y"),
         CylShape((0.0, -0.007, 0.0), 0.008, 0.004, STEEL, axis="Y"),
         CylShape((0.0, rb / 2, 0.0), 0.004, -rb, ZINC, axis="Y"),
         BoxShape((0.0, rb - 0.022, 0.000), (0.010, 0.044, 0.008), ZINC),  # the hook's bottom bar
         BoxShape((0.0, rb - 0.040, 0.018), (0.010, 0.008, 0.044), ZINC),  # its back
         BoxShape((0.0, rb - 0.022, 0.036), (0.010, 0.044, 0.008), ZINC),  # its top bar (taken hold of)
         BoxShape((0.0, rb - 0.003, 0.028), (0.010, 0.006, 0.010), ZINC)]  # its lip
    s0 = _S_START
    return Body.build("rod", pos=(GX, GY + s0, AXIS_Z), mass=ROD_MASS, shapes=s, friction=(1.2, 1.1), color=ZINC,
                      com=(0.0, -0.11, 0.0), solver_iterations=64)


def _tube() -> Body:
    """A 310 ml cartridge of grey silicone: the white tube, its label, the shoulder and the cut nozzle. Its back
    45 mm (where the piston travels) is drawn only; the rest collides."""
    solid = TUBE_L - SOLID_FROM
    s = [CylShape((0.0, (TUBE_L / 2 - solid / 2), 0.0), TUBE_R, solid, WHITE, axis="Y"),
         CylShape((0.0, -TUBE_L / 2 + SOLID_FROM / 2, 0.0), TUBE_R, SOLID_FROM - 0.0005, WHITE, collide=False,
                  axis="Y"),
         CylShape((0.0, 0.010, 0.0), TUBE_R + 0.0004, 0.120, LABEL, collide=False, axis="Y"),
         CylShape((0.0, 0.030, 0.0), TUBE_R + 0.0007, 0.020, LABEL2, collide=False, axis="Y"),
         CylShape((0.0, -0.010, 0.0), TUBE_R + 0.0007, 0.014, SILICONE, collide=False, axis="Y"),
         CylShape((0.0, TUBE_L / 2 + SHOULDER / 2, 0.0), 0.016, SHOULDER, WHITE, axis="Y")]
    y = TUBE_L / 2 + SHOULDER
    for r, ln in NOZZLE:
        s.append(CylShape((0.0, y + ln / 2, 0.0), r, ln, NOZ, axis="Y"))
        y += ln
    return Body.build("tube", pos=tuple(_TUBE_START.tolist()), mass=TUBE_MASS, shapes=s, friction=(1.0, 0.9),
                      color=WHITE, solver_iterations=64)


def _piston() -> Body:
    """The cartridge's polyethylene piston, inside the back of the tube."""
    s = [CylShape((0.0, 0.0, 0.0), BORE_R - 0.0002, PISTON_T, (0.80, 0.80, 0.78), axis="Y"),
         CylShape((0.0, -PISTON_T / 2 - 0.0002, 0.0), 0.012, 0.0004, (0.70, 0.70, 0.68), collide=False, axis="Y")]
    return Body.build("piston", pos=tuple((_TUBE_START + [0.0, PISTON_Y, 0.0]).tolist()), mass=PISTON_MASS,
                      shapes=s, friction=(0.6, 0.5), color=WHITE, solver_iterations=64,
                      ignore=("gun", "trigger", "plate", "release"))  # inside the tube: only the push plate reaches it


def _tray() -> Body:
    """A cardboard tray with two notched saddles the cartridge lies on."""
    s = [BoxShape((0.0, 0.0, 0.002), (0.100, 0.340, 0.004), CARD)]
    for sx in (-1.0, 1.0):
        s.append(BoxShape((sx * 0.049, 0.0, 0.0125), (0.002, 0.340, 0.025), CARD_DARK))
    for sy in (-0.150, 0.150):
        s.append(BoxShape((0.0, sy * 1.12, 0.0125), (0.100, 0.002, 0.025), CARD_DARK))
    for y in (-0.060, 0.060):
        for sx in (-1.0, 1.0):
            s.append(BoxShape((sx * 0.026, y, 0.010), (0.024, 0.016, 0.012), CARD_DARK))
    return Body.build("tray", pos=(TRAY[0], TRAY[1] + 0.005, TOP), mass=1.0, kinematic=True, shapes=s, color=CARD)


def _rest() -> Body:
    h = AXIS_Z - R_TROUGH - 0.0015 - TOP
    s = [BoxShape((0.0, 0.0, h / 2), (0.060, 0.040, h), WOOD),
         BoxShape((0.0, 0.0201, h / 2), (0.056, 0.0004, h - 0.006), WOOD_DARK, collide=False)]
    return Body.build("rest", pos=(GX, YOKE_Y0 - 0.026 + GY - 0.020, TOP), mass=1.0, kinematic=True, shapes=s,
                      color=WOOD, friction=(0.8, 0.7))


def _bath() -> Body:
    """The white acrylic rim of a bath along the table's left edge, against a tiled wall."""
    cx = (BX0 + BX1) / 2
    lx = BX1 - BX0
    h = RIM_Z - TOP
    s = [BoxShape((0.0, (RIM_Y0 + WALL_Y) / 2, h / 2), (lx, WALL_Y - RIM_Y0, h), ACRYLIC),
         BoxShape((0.0, RIM_Y0 + 0.004, h - 0.004), (lx, 0.008, 0.008), (0.93, 0.93, 0.92), collide=False)]
    wall_h = 0.300
    s.append(BoxShape((0.0, WALL_Y + 0.010, wall_h / 2), (lx + 0.04, 0.020, wall_h), TILE))
    for k in range(1, 4):  # grout lines between 75 mm tiles, on the wall's face
        z = h + 0.075 * k
        s.append(BoxShape((0.0, WALL_Y - 0.0004, z), (lx + 0.04, 0.0008, 0.003), GROUT, collide=False))
    for k in range(6):
        x = -lx / 2 - 0.02 + 0.075 * (k + 0.5) + 0.0175
        s.append(BoxShape((x, WALL_Y - 0.0004, (h + wall_h) / 2), (0.003, 0.0008, wall_h - h), GROUT, collide=False))
    return Body.build("bath", pos=(cx, 0.0, TOP), mass=10.0, kinematic=True, shapes=s, color=ACRYLIC,
                      friction=(0.25, 0.2), slick=True)  # glossy acrylic and glazed tile


def _beads() -> list[Body]:
    fil = [Body.build(f"fil{k}", pos=HIDE, mass=0.001, kinematic=True, color=SILICONE,
                      shapes=[CylShape((0.0, 0.0, 0.0), FIL_R, FIL_L, SILICONE, collide=False, axis="X")])
           for k in range(N_FIL)]
    blob = [Body.build(f"blob{k}", pos=HIDE, mass=0.001, kinematic=True, color=SILICONE,
                       shapes=[SphereShape((0.0, 0.0, 0.0), BLOB_R, SILICONE, collide=False)])
            for k in range(N_BLOB)]
    return fil + blob


_TUBE_START = TUBE0.copy()  # where the cartridge starts
_S_START = S0  # where the rod starts

hinge("release_pivot", "gun", "release", anchor=tuple(PIVOT.tolist()), axis="X", limits_deg=(-12.0, 1.0),
      spring=0.003, damping=2e-4, target_deg=0.0)
hinge("trigger_pivot", "gun", "trigger", anchor=tuple((GUN0 + PIVOT_T).tolist()), axis="X",
      limits_deg=(-TRIG_MAX, 0.0), spring=TRIG_SPRING, damping=2e-4, target_deg=TRIG_TARGET)
slider("plate_slide", "gun", "plate", anchor=(GX, GY + PLATE_Y, AXIS_Z), axis="Y", limits=(0.0, PLATE_TRAVEL),
       stiffness=PLATE_K, damping=2.0, target=-PLATE_PRE, max_force=60.0)
slider("rod_slide", "gun", "rod", anchor=(GX, GY + _S_START, AXIS_Z), axis="Y", limits=(-_S_START, S_MAX - _S_START),
       stiffness=0.0, damping=LOCK_D, target=0.0, max_force=400.0)
slider("piston_slide", "tube", "piston", anchor=tuple((_TUBE_START + [0.0, PISTON_Y, 0.0]).tolist()), axis="Y",
       limits=(0.0, PISTON_TRAVEL), stiffness=SEAL_K, damping=200.0, target=0.0, max_force=SEAL_F)

OBJECTS = [_bath(), _rest(), _tray(), _gun(), _release(), _trigger(), _plate(), _rod(), _tube(), _piston()] + _beads()


# -- the gripping plate's grip on the rod: a slide between them whose stop the task file sets ------------------------
# The rod may run forward through the plate freely, but not back past the stop: pushed forward, the plate carries the
# rod; let go, it slides back along the rod. While the plate is back at rest the stop is kept just behind it.

slider("clutch", "plate", "rod", anchor=(GX, GY + PLATE_Y, AXIS_Z), axis="Y", limits=(-1.0, 1.0), stiffness=0.0,
       damping=0.0, target=0.0, max_force=1.0)
_CLUTCH: dict = {}


def _stop(name: str, lo: float) -> None:
    """Move a slide's lower stop (m from where it started)."""
    if name not in _CLUTCH:
        from isaaclab.sim.utils.stage import get_current_stage
        from pxr import UsdPhysics

        prim = get_current_stage().GetPrimAtPath(bodykit._SLIDER_PATHS[name])
        _CLUTCH[name] = UsdPhysics.PrismaticJoint(prim).GetLowerLimitAttr()
    _CLUTCH[name].Set(float(lo))


def _clutch_stop(lo: float) -> None:
    _stop("clutch", lo)


# -- the bead's look: a round of radius FIL_R sunk along the corner's bisector shows a section of the corner ---------

def _visible_table() -> tuple[np.ndarray, np.ndarray]:
    r = FIL_R
    g = np.linspace(0.0, 2.0 * r, 400)
    du = g[1] - g[0]
    uu, vv = np.meshgrid(g + du / 2, g + du / 2)
    ds = np.linspace(-0.6 * r, 0.70 * r, 120)
    area = np.array([float(((uu + d) ** 2 + (vv + d) ** 2 <= r * r).sum()) * du * du for d in ds])
    return ds, area


_VIS_D, _VIS_A = _visible_table()


def _sink_for(section: float) -> float:
    """How far into the corner (along its bisector) a round's centre goes so the visible section is ``section``."""
    a = np.clip(section, _VIS_A[-1], _VIS_A[0])
    return float(np.interp(a, _VIS_A[::-1], _VIS_D[::-1]))


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"seal": 0.0, "lock": None, "free": False, "clutch": False, "stop": None, "pprev": 0.0, "adv_max": 0.0, "v_in": 0.0,
            "v_out": 0.0, "v_press": 0.0, "seg": None, "sec": np.zeros(N_BIN), "fil": 0, "blob": 0, "tcps": None,
            "s": {}, "stray": 0.0, "laid": 0.0}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("CAULK_DEBUG")), "steps": 0}


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    if "clutch" in _CLUTCH:
        _clutch_stop(-1.0)
        _stop("rod_slide", 0.0)


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _tcps(base, origin) -> list[np.ndarray]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _fingers(base) -> list[float]:
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        names = list(robot.joint_names)
        q = as_numpy(robot.data.joint_pos)[0]
        out.append(float(q[names.index("joint7")]) if "joint7" in names else float("nan"))
    return out


def _place(base, origin, name, pos) -> None:
    import torch

    asset = base.scene[name]
    pose = torch.tensor([[*(np.asarray(pos, float) + origin).tolist(), 0.0, 0.0, 0.0, 1.0]], dtype=torch.float32,
                        device=asset.device)
    asset.write_root_pose_to_sim(pose)


def hinge_deg(r0, r1) -> float:
    rel = r0.T @ r1
    return math.degrees(math.atan2(rel[2, 1], rel[1, 1]))


def _corner(p, r: float) -> bool:
    """A bead of radius r laid at p lies in the corner, touching both the rim's top and the tiles."""
    return (BX0 <= p[0] <= BX1 and -0.002 <= WALL_Y - p[1] <= r + TOUCH and -0.002 <= p[2] - RIM_Z <= r + TOUCH)


def _lay(base, origin, p0, p1, vol) -> None:
    """Lay ``vol`` (m^3) of sealant along the tip's path p0 -> p1."""
    ln = float(np.linalg.norm(p1 - p0))
    sec = vol / max(ln, 0.006)
    r = math.sqrt(sec / math.pi)
    if _corner(p0, r) and _corner(p1, r):
        _EP["laid"] += vol
        xa, xb = sorted((float(p0[0]), float(p1[0])))
        lo, hi = xa - r, xb + r  # a round bead runs its radius past the tip's path at each end
        per_m = vol / (hi - lo)
        edges = JOINT_X0 + np.arange(N_BIN + 1) * 0.001
        overlap = np.clip(np.minimum(edges[1:], hi) - np.maximum(edges[:-1], lo), 0.0, None)
        _EP["sec"] += overlap * per_m  # m^3 laid on each 1 mm length of the joint
        n = max(1, int(math.ceil((xb - xa + 0.002) / (FIL_L * 0.8))))
        d = _sink_for(sec)
        for k in range(n):
            x = xa + (xb - xa) * (k + 0.5) / n
            name = f"fil{min(_EP['fil'], N_FIL - 1)}"
            _EP["fil"] += 1
            _place(base, origin, name, (x, WALL_Y + d, RIM_Z - d))
    else:
        _EP["stray"] += vol
        if _DEBUG["on"]:
            print(f"[caulk-stray] v={vol * 1e6:.3f}ml r={r * 1000:.1f} p0=({p0[0] * 1000:.0f},{(WALL_Y - p0[1]) * 1000:.1f},"
                  f"{(p0[2] - RIM_Z) * 1000:.1f}) p1=({p1[0] * 1000:.0f},{(WALL_Y - p1[1]) * 1000:.1f},"
                  f"{(p1[2] - RIM_Z) * 1000:.1f})", file=sys.stderr, flush=True)
        name = f"blob{min(_EP['blob'], N_BLOB - 1)}"
        _EP["blob"] += 1
        on_rim = BX0 <= p1[0] <= BX1 and RIM_Y0 <= p1[1] <= WALL_Y and p1[2] >= RIM_Z
        floor = (RIM_Z if on_rim else TOP) + BLOB_R * 0.6  # it drops onto whatever is under the tip
        _place(base, origin, name, (float(p1[0]), float(p1[1]), min(float(p1[2]), floor)))


def bead_stats() -> tuple[float, int]:
    """(the fraction of the joint's length sealed, the longest unsealed stretch in mm, ends included)."""
    ok = (_EP["sec"] / 0.001) >= A_MIN
    gap = run = 0
    for v in ok:
        run = 0 if v else run + 1
        gap = max(gap, run)
    return float(ok.mean()), int(gap)


def _box_dist(p_local, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p_local, p_local - hi), 0.0)))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    dt = float(getattr(base, "step_dt", 0.05))
    origin = as_numpy(base.scene.env_origins[0])
    pg, rg = _pose(base, origin, "gun")
    _, rl = _pose(base, origin, "release")
    _, rtr = _pose(base, origin, "trigger")
    pr, _ = _pose(base, origin, "rod")
    pp, _ = _pose(base, origin, "plate")
    pt, rt = _pose(base, origin, "tube")
    pps, _ = _pose(base, origin, "piston")
    lever = hinge_deg(rg, rl)
    trig = hinge_deg(rg, rtr)
    ptr_g = rg.T @ (_pose(base, origin, "trigger")[0] - pg)
    s = float((rg.T @ (pr - pg))[1])  # the push plate's face, in front of the back plate
    plate = float((rg.T @ (pp - pg))[1]) - PLATE_Y  # the gripping plate, forward of its rest
    free = lever <= RELEASE_DEG
    rel = s - _S_START  # the rod slide's own coordinate
    # the locking plate: none while the lever is held down; otherwise a stop just behind the rod, following it in
    if free:
        if not _EP["free"]:
            _stop("rod_slide", -_S_START)
    elif _EP["free"] or _EP["lock"] is None or rel - LOCK_GAP > _EP["lock"] + 1e-5:
        _EP["lock"] = rel - LOCK_GAP
        _stop("rod_slide", _EP["lock"])
    _EP["free"] = free
    # the gripping plate: its stop on the rod is kept just behind it except while it is pushed forward (then it carries
    # the rod); none while the release lever is down
    if free:
        if _EP["stop"] is not None:
            _EP["stop"] = None
            _clutch_stop(-1.0)
    else:
        here = rel - plate - CLUTCH_GAP
        if _EP["stop"] is None or ((plate < CLUTCH_REST or plate < _EP["pprev"] - 1e-5) and here > _EP["stop"] + 1e-5):
            _EP["stop"] = here
            _clutch_stop(here)
    _EP["clutch"] = _EP["stop"] is not None and rel - plate - _EP["stop"] < CLUTCH_GAP + 1e-4
    _EP["pprev"] = plate
    # the piston: sealant pushed out of the cartridge, leaving the nozzle a moment later
    adv = float((rt.T @ (pps - pt))[1]) - PISTON_Y
    if adv - _EP["seal"] > SEAL_F / SEAL_K:  # pushed past what the sealant holds: it yields, the hold follows
        _EP["seal"] = adv - SEAL_F / SEAL_K
        set_slider_drive("piston_slide", target=_EP["seal"])
    flowed = _EP["seal"]  # how far the piston has moved through the silicone (its elastic give below 150 N excluded)
    if flowed > _EP["adv_max"]:
        dv = (flowed - _EP["adv_max"]) * A_BORE
        _EP["adv_max"] = flowed
        _EP["v_in"] += dv
        _EP["v_press"] += dv
    out = _EP["v_press"] * (1.0 - math.exp(-dt / TAU))
    _EP["v_press"] -= out
    _EP["v_out"] += out
    tip = pt + rt @ np.array([0.0, TIP, 0.0])
    seg = _EP["seg"]
    if seg is None or (seg["v"] < 1e-10 and out < 1e-10):
        _EP["seg"] = seg = {"p0": tip.copy(), "v": 0.0, "n": 0}
    else:
        seg["v"] += out
        seg["n"] += 1
        if (np.linalg.norm(tip - seg["p0"]) >= SEG_LEN or seg["n"] >= SEG_STEPS or seg["v"] >= SEG_V) \
                and seg["v"] > 1e-10:
            _lay(base, origin, seg["p0"], tip.copy(), seg["v"])
            _EP["seg"] = {"p0": tip.copy(), "v": 0.0, "n": 0}
    # the gun in its own frame: the cartridge, and the plate against its piston
    v = rg.T @ (pt - pg)
    along = float((rg.T @ rt)[:, 1][1])
    seated = (abs(v[0]) <= 0.008 and SEAT_Z - 0.010 <= v[2] <= SEAT_Z + 0.008 and SEAT_Y - 0.030 <= v[1] <= SEAT_Y + 0.006
              and along >= math.cos(math.radians(8.0)))
    piston_back = float((rg.T @ (pps - pg))[1]) - PISTON_T / 2
    loaded = seated and s >= piston_back - 0.002
    # put down: the grip's foot on (or near) the table, upright enough, both grippers clear of the gun
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rg[2, 2])))))
    foot = pg + rg @ np.array([0.0, (HANDLE_Y[0] + HANDLE_Y[1]) / 2, FOOT_Z])
    tcps = _tcps(base, origin)
    _EP["tcps"] = tcps
    lo_g, hi_g = OBJECTS_BY["gun"].local_bounds()
    lo_t, hi_t = OBJECTS_BY["trigger"].local_bounds()
    lo_r, hi_r = OBJECTS_BY["rod"].local_bounds()
    clear = True
    for t in tcps:
        dg = _box_dist(rg.T @ (t - pg), lo_g, hi_g)
        dtr = _box_dist(rtr.T @ (t - (pg + rg @ PIVOT_T)), lo_t, hi_t)
        drd = _box_dist(rg.T @ (t - pr), lo_r, hi_r)
        clear = clear and min(dg, dtr, drd) >= CLEAR
    down = float(foot[2]) - TOP <= DOWN_Z and tilt <= DOWN_TILT
    cov, gap = bead_stats()
    _EP["s"] = {"seated": seated, "loaded": loaded, "down": down, "clear": clear, "cov": cov, "gap": gap}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 100 == 0:
            ok = (_EP["sec"] / 0.001) >= A_MIN
            runs, k = [], 0
            while k < N_BIN:
                if not ok[k]:
                    j = k
                    while j < N_BIN and not ok[j]:
                        j += 1
                    runs.append((round(JOINT_X0 * 1000) + k, j - k))
                    k = j
                else:
                    k += 1
            print(f"[caulk-gaps] {runs} sec(mm2) min/mean={(_EP['sec'] / 0.001).min() * 1e6:.1f}/"
                  f"{(_EP['sec'] / 0.001).mean() * 1e6:.1f}", file=sys.stderr, flush=True)
        if _DEBUG["steps"] % 5 == 0:
            tl, trc = tcps
            fl, fr = _fingers(base)
            print(f"[caulk] t={_DEBUG['steps'] * dt:5.1f}s lever={lever:5.1f} free={free} trig={trig:6.1f}"
                  f" toff={np.linalg.norm(ptr_g - PIVOT_T) * 1000:4.2f} plate={plate * 1000:5.2f} clutch={_EP['clutch']} stop={(_EP['stop'] or 0) * 1000:6.2f} rp={(rel - plate) * 1000:6.2f} s={s * 1000:6.1f} lock={((_EP['lock'] or 0) + _S_START) * 1000:6.1f}"
                  f" adv={adv * 1000:6.2f} vin={_EP['v_in'] * 1e6:5.2f}ml vout={_EP['v_out'] * 1e6:5.2f}ml"
                  f" laid={_EP['laid'] * 1e6:5.2f} stray={_EP['stray'] * 1e6:5.2f}"
                  f" tube=({v[0] * 1000:.0f},{v[1] * 1000:.0f},{v[2] * 1000:.0f}) seated={seated} loaded={loaded}"
                  f" tip_c=({(tip[0]) * 1000:.0f},{(WALL_Y - tip[1]) * 1000:.1f},{(tip[2] - RIM_Z) * 1000:.1f})"
                  f" cov={cov:.2f} gap={gap} fil={_EP['fil']} blob={_EP['blob']}"
                  f" gun={np.round((pg - [0, 0, TOP]) * 1000).astype(int).tolist()} tilt={tilt:4.1f}"
                  f" pry=({math.degrees(math.asin(max(-1, min(1, rg[2, 1])))):.1f},{math.degrees(math.atan2(-rg[2, 0], rg[2, 2])):.1f},{math.degrees(math.atan2(rg[1, 0], rg[0, 0])):.1f})"
                  f" foot={(foot[2] - TOP) * 1000:.0f} down={down} clear={clear}"
                  f" L={np.round((tl - [0, 0, TOP]) * 1000).astype(int).tolist()}"
                  f" R={np.round((trc - [0, 0, TOP]) * 1000).astype(int).tolist()} fing=({fl:.4f},{fr:.4f})"
                  f" Rg={np.round(rg.T @ (trc - pg) * 1000).astype(int).tolist()} score={score(None)}",
                  file=sys.stderr, flush=True)


OBJECTS_BY = {o.name: o for o in OBJECTS}


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """At the end. Sealed: at least 95% of the joint's 192 mm carries a bead at least 4 mm across (12.6 mm^2 of
    sealant per mm) touching both the rim and the tiles, with no unsealed stretch longer than 2 mm (the joint's ends
    included); only sealant the gun dispensed counts. 1.0: sealed, and the gun put down (its grip's foot within 2 cm of
    the table, tilted at most 25 degrees) with both grippers clear of it (each TCP at least 6 cm from its frame,
    trigger and rod); 0.75: sealed; 0.5: at least half of the joint sealed; 0.25: the gun loaded (the cartridge seated
    in its cradle nozzle first, the push plate against its piston); else 0.0."""
    s = _EP["s"]
    cov, gap = s.get("cov", 0.0), s.get("gap", N_BIN)
    if cov >= COVER_FULL and gap <= GAP_FULL:
        return 1.0 if (s.get("down") and s.get("clear")) else 0.75
    if cov >= COVER_HALF:
        return 0.5
    if s.get("loaded"):
        return 0.25
    return 0.0


def success(objects) -> bool:
    """Full score with the gun and its moving parts still: the trial ends there."""
    return (score(objects) >= 1.0 and all(objects[n].resting() for n in
                                          ("gun", "rod", "trigger", "plate", "release", "tube", "piston")))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand presses the release lever down with closed fingers; the right takes the rod's hook and pulls the rod
# all the way back. The left lets go of the lever, takes the cartridge off its tray, carries it over the gun and lowers
# it into the cradle, nozzle through the yoke. The right pushes the rod back in until its plate meets the piston. The
# left takes the yoke across its width; the right lets go of the hook and closes across the grip and the trigger. Both
# lift the gun, lean it nose down, put the nozzle's tip into the corner at the joint's right end and lay the bead in
# four strokes: the right squeezes until the silicone comes, then on to the trigger's stop while both hands draw the
# nozzle along the joint, and lets the trigger come back before the next stroke. They lift the nozzle out, set the gun
# back on its grip and rest block, let go and move clear; the left ends with its wrist camera over the sealed joint.

LEAD_S, LEAD_STEPS = 3.0, 15
OVER = 0.035
PRESS_DZ = 0.005  # the closed fingertips aimed this far below the lever pad's top
PAD = np.array([GX, GY - 0.015, AXIS_Z + 0.034 + 0.006])  # the pad's top, at rest
HOOK_FINGER = -0.012
TUBE_FINGER = -0.0305
TUBE_DX = 0.017  # the cartridge sits this far toward -x of the left TCP once taken hold of (measured)
YOKE_FINGER = -0.044  # open round the yoke's 64 mm width
YOKE_HOLD = -0.026  # closed on it 6 mm short of its width: about 30 N a side, and stiff along the closing axis
EAR = np.array([-0.005, YOKE_Y0 + YOKE_T / 2, 0.008])  # the left's TCP across the yoke's width, gun frame (it lands
# about 6 mm toward +x: aimed against it)
AIM_DZ = 0.006  # aimed this much higher: the arms sag with the gun
# the right's grip across the pistol grip and the trigger: a side grip from the robot's side, pitched down
GRIP_TH = float(os.environ.get("CAULK_TH", "25"))  # degrees below level
GRIP_DZ = -0.036  # the fingers 36 mm below the trigger's pivot
GRIP_IN = 0.012  # the parts this far past the TCP toward the fingertips


def trig_front(phi_deg: float, dz: float = BAR_DZ) -> float:
    """The front of the trigger's finger rest (gun-frame y), the trigger turned phi_deg."""
    f = math.radians(phi_deg)
    return float(PIVOT_T[1] + BAR_DY * math.cos(f) - BAR_DZ * math.sin(f) + BAR_R)


SPAN_REST = trig_front(0.0, GRIP_DZ) - HANDLE_Y[0]  # grip's back face to the trigger's front face, at rest
SPAN_FULL = trig_front(-TRIG_MAX, GRIP_DZ) - HANDLE_Y[0]  # the same, trigger fully squeezed
HELD = SPAN_REST - 0.008  # holding the gun: the fingers aimed 4 mm into the grip and the trigger (about 20 N; the
# trigger takes up its slack, then the silicone holds it: no sealant comes below about 26 N)


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    side = [(0.0, 1.3, 1.0, -0.8, 0.0, 0.0), (0.7, 1.9, 1.7, -1.3, 1.2, 1.5), (-0.7, 1.9, 1.7, -1.3, -1.2, -1.5),
            (0.0, 1.9, 1.7, -0.8, 0.0, 1.5), (0.0, 1.9, 1.7, -0.8, 0.0, -1.5), (0.4, 2.5, 1.7, 0.0, 1.2, 0.0),
            (-0.4, 2.5, 1.7, 0.0, -1.2, 0.0), (0.0, 1.3, 1.0, 0.0, 0.0, 1.5), (0.0, 1.3, 1.0, 0.0, 0.0, -1.5),
            (0.7, 1.3, 1.0, -0.8, 1.5, 0.0), (-0.7, 1.3, 1.0, -0.8, -1.5, 0.0)]
    seeds = list(itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4),
                                   (-0.4, 0.4))) + side
    for seed in seeds:
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed),
                             base_pos=station.BASES[arm], iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def _w(t, pos, finger, label, rot) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in pos), finger, label, rot)


def _side(th_deg: float) -> np.ndarray:
    """A grip from the robot's side (approach +x), pitched down th_deg, the fingers closing along the gun's axis."""
    t = math.radians(th_deg)
    z = np.array([math.cos(t), 0.0, -math.sin(t)])
    x = np.array([0.0, -1.0, 0.0])  # (the other way round has no IK solution)
    return np.column_stack([x, np.cross(z, x), z])


RL_ROT = grip_rotation(0.0)  # the left pressing the lever (closed fingers side by side along y)
RH_ROT = grip_rotation(math.pi / 2)  # the right on the hook: fingers closing across it, along x
LT_ROT = grip_rotation(-math.pi / 2)  # the left across the cartridge, closing along x
LY_ROT = grip_rotation(-math.pi / 2)  # the left across the yoke's width, closing along x (holds the gun's roll)
RG_ROT = _side(GRIP_TH)  # the right across the grip and the trigger


def _grip_tcp(span: float) -> np.ndarray:
    """The right's TCP (gun frame) across the grip and the trigger, the fingers ``span`` apart."""
    approach = RG_ROT[:, 2]
    part = np.array([0.0, HANDLE_Y[0] + span / 2, PIVOT_T[2] + GRIP_DZ])
    return part - approach * GRIP_IN


def _span_cmd(span: float) -> float:
    return -span / 2


def _sync(plan, label_l, label_r) -> None:
    L, R = plan["left"], plan["right"]
    tl, tr = sum(w.duration for w in L), sum(w.duration for w in R)
    if tl > tr:
        last = R[-1]
        R.append(_w(tl - tr, last.pos, last.finger, label_r, last.yaw))
    elif tr > tl:
        last = L[-1]
        L.append(_w(tr - tl, last.pos, last.finger, label_l, last.yaw))


def _stage(gun_o: np.ndarray) -> np.ndarray:
    """Where the right waits, fingers open, before reaching in under the gun's frame: back along its approach."""
    return gun_o + _grip_tcp(SPAN_REST) - RG_ROT[:, 2] * 0.12 + [0.0, -0.02, 0.03]


def _take_gun(plan, gun_o: np.ndarray, start_right_q=None) -> None:
    """The left takes the yoke's ear and the right closes across the grip and the trigger, the gun at ``gun_o``."""
    L, R = plan["left"], plan["right"]
    ear0 = gun_o + EAR
    L.append(_w(1.4, ear0 + [0, 0, OVER], YOKE_FINGER, "over the gun's front yoke", LY_ROT))
    L.append(_w(1.0, ear0, YOKE_FINGER, "down round the yoke's ear", LY_ROT))
    L.append(_w(0.6, ear0, YOKE_HOLD, "take hold of the yoke", LY_ROT))
    g_open = gun_o + _grip_tcp(SPAN_REST)
    wide = _span_cmd(SPAN_REST + 0.012)
    R.append(_w(1.0, _stage(gun_o), wide, "toward the gun's grip", RG_ROT))
    R.append(_w(1.4, g_open - RG_ROT[:, 2] * 0.05, wide, "toward the gun's grip", RG_ROT))
    R.append(_w(1.2, g_open, wide, "the fingers round the grip and the trigger", RG_ROT))
    R.append(_w(0.8, g_open, _span_cmd(HELD), "close on the grip and the trigger", RG_ROT))
    _sync(plan, "hold the yoke", "hold the grip")


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    L, R = plan["left"], plan["right"]

    def hook_at(s):
        return GUN0 + np.array([HOOK_GRIP[0], s + HOOK_GRIP[1], HOOK_GRIP[2]])

    # -- the left presses the lever; the right takes the hook and pulls the rod back -------------------------------
    press = PAD + [0.0, 0.0, 0.013 - PRESS_DZ - 0.006]
    near = press + [0.0, 0.040, 0.025]  # (straight over the lever is out of the left's reach)
    L += _joint_path("left", np.array(READY, float), _good_ik("left", near, RL_ROT), "over the release lever",
                     FINGER_CLOSED)
    L.append(_w(1.0, press + [0, 0, 0.012], FINGER_CLOSED, "down onto the release lever", RL_ROT))
    L.append(_w(0.8, press, FINGER_CLOSED, "press the release lever down and hold the gun", RL_ROT))
    h0 = hook_at(S0)
    R += _joint_path("right", np.array(READY, float), _good_ik("right", h0 + [0, 0, OVER], RH_ROT),
                     "over the rod's hook", HOOK_FINGER)
    R.append(_w(1.0, h0 + [0, 0, OVER], HOOK_FINGER, "over the rod's hook", RH_ROT))
    R.append(_w(0.8, h0, HOOK_FINGER, "down round the hook's top bar", RH_ROT))
    R.append(_w(0.5, h0, FINGER_CLOSED, "take hold of the hook", RH_ROT))
    h1 = hook_at(-0.012)
    for k in range(1, 5):
        R.append(_w(0.6, h0 + (h1 - h0) * k / 4, FINGER_CLOSED, "pull the rod back, all the way", RH_ROT))
    R.append(_w(0.5, h1, FINGER_CLOSED, "pull the rod back, all the way", RH_ROT))
    t_pull_end = sum(w.duration for w in R)
    t_left = sum(w.duration for w in L)
    L.append(_w(t_pull_end - t_left, press, FINGER_CLOSED, "keep the lever pressed", RL_ROT))
    L.append(_w(0.6, press + [0, 0.040, 0.025], FINGER_CLOSED, "let go of the lever", RL_ROT))

    # -- the left lays the cartridge in the cradle ---------------------------------------------------------------
    grab = TUBE0.copy()
    L.append(_w(1.4, grab + [0, 0, OVER + 0.04], TUBE_FINGER, "over the cartridge", LT_ROT))
    L.append(_w(1.0, grab + [0, 0, 0.012], TUBE_FINGER, "down round the cartridge", LT_ROT))
    L.append(_w(0.6, grab, TUBE_FINGER, "down round the cartridge", LT_ROT))
    L.append(_w(0.6, grab, FINGER_CLOSED, "take hold of the cartridge", LT_ROT))
    up = grab + [0, 0, 0.13]
    L.append(_w(1.0, up, FINGER_CLOSED, "lift it off the tray", LT_ROT))
    drop_y = GY + SEAT_Y - 0.006
    over = np.array([GX + TUBE_DX, drop_y, up[2]])
    L.append(_w(1.6, over, FINGER_CLOSED, "carry it over the gun's cradle", LT_ROT))
    seat = np.array([GX + TUBE_DX, drop_y, AXIS_Z + SEAT_Z + 0.003 + 0.004])
    L.append(_w(1.2, seat + [0, 0, 0.03], FINGER_CLOSED, "lower it into the cradle, nozzle through the yoke", LT_ROT))
    L.append(_w(0.8, seat, FINGER_CLOSED, "lower it into the cradle, nozzle through the yoke", LT_ROT))
    L.append(_w(0.6, seat, TUBE_FINGER, "let go of it", LT_ROT))
    L.append(_w(0.8, seat + [0, 0, 0.07], TUBE_FINGER, "rise clear", LT_ROT))
    t_laid = sum(w.duration for w in L)
    t_right = sum(w.duration for w in R)
    R.append(_w(t_laid - t_right, h1, FINGER_CLOSED, "hold the hook while the cartridge goes in", RH_ROT))
    # -- the right pushes the rod back in until the plate meets the piston ---------------------------------------
    s_piston = SEAT_Y - TUBE_L / 2 + PISTON_IN
    h2 = hook_at(s_piston + 0.018)  # (the arm lags about 1 cm)
    for k in range(1, 4):
        R.append(_w(0.6, h1 + (h2 - h1) * k / 3, FINGER_CLOSED, "push the rod in until the plate meets the piston",
                    RH_ROT))
    R.append(_w(0.6, h2, FINGER_CLOSED, "push the rod in until the plate meets the piston", RH_ROT))
    R.append(_w(0.5, h2, HOOK_FINGER, "let go of the hook", RH_ROT))
    R.append(_w(0.6, h2 + [0, 0, OVER], HOOK_FINGER, "rise clear", RH_ROT))
    R += _joint_path("right", _good_ik("right", h2 + [0, 0, OVER], RH_ROT), _good_ik("right", _stage(GUN0), RG_ROT),
                     "toward the gun's grip", _span_cmd(SPAN_REST + 0.012))
    _sync(plan, "wait", "wait")
    _take_gun(plan, GUN0)
    _seal(plan)
    return plan


def _rx(deg: float) -> np.ndarray:
    a = math.radians(deg)
    return np.array([[1.0, 0.0, 0.0], [0.0, math.cos(a), -math.sin(a)], [0.0, math.sin(a), math.cos(a)]])


def _move(plan, g0, g1, t, label, n, span0, span1=None, rdx0=0.0, rdx1=None) -> None:
    """Both hands move the gun from pose g0 to g1 (each (origin, pitch in degrees, nose down < 0)); the right's
    fingers go from span0 to span1 apart (trigger squeezed or let go), its TCP shifted rdx along x (it leads the gun
    when the gun is drawn along x)."""
    L, R = plan["left"], plan["right"]
    span1 = span0 if span1 is None else span1
    rdx1 = rdx0 if rdx1 is None else rdx1
    for k in range(1, n + 1):
        f = k / n
        o = np.asarray(g0[0]) + (np.asarray(g1[0]) - np.asarray(g0[0])) * f
        rp = _rx(g0[1] + (g1[1] - g0[1]) * f)
        sp = span0 + (span1 - span0) * f
        rdx = rdx0 + (rdx1 - rdx0) * f
        L.append(_w(t / n, o + rp @ EAR + [0, 0, AIM_DZ], YOKE_HOLD, label, rp @ LY_ROT))
        R.append(_w(t / n, o + rp @ _grip_tcp(max(sp, SPAN_FULL)) + [rdx, 0, AIM_DZ], _span_cmd(sp), label,
                    rp @ RG_ROT))


PITCH = -10.0  # sealing, the gun leans nose down this much (its nozzle clears the rim's front edge)
RDX = -0.006  # the right's TCP this far behind while the gun is drawn along the joint


def _seal(plan) -> None:
    L, R = plan["left"], plan["right"]
    tip_local = np.array([0.0, SEAT_Y + TIP, SEAT_Z])
    rest = (GUN0, 0.0)
    lift = (GUN0 + [0, 0, 0.045], 0.0)
    held = HELD
    _move(plan, rest, lift, 1.2, "lift the gun off its rest", 3, held)
    # the tip into the corner at the joint's right end, resting on the rim against the tiles
    tip_in = np.array([JOINT_X0 - 0.012, WALL_Y + 0.003, RIM_Z + 0.0035 - 0.020])  # (aimed 2 cm into the rim and
    # 3 mm into the tiles: the arms give, the gun leans less than planned, the nozzle rides in the corner)
    rp = _rx(PITCH)

    def at(tip):
        return (np.asarray(tip) - rp @ tip_local, PITCH)

    start = at(tip_in)
    above = (start[0] + [0, -0.03, 0.03], PITCH)
    _move(plan, lift, above, 2.0, "carry it to the joint's right end", 5, held)
    _move(plan, above, start, 1.0, "the nozzle's tip into the joint", 3, held)
    full = SPAN_FULL - 0.012  # (the fingers aimed past the trigger's stop: they press it there)
    near = SPAN_FULL - 0.009  # (the trigger reaches its stop just as the stroke ends: the fingers lag about 1 cm)
    back = SPAN_REST - 0.006  # (letting the trigger come back most of the way, the grip still held)
    x_end = JOINT_X1 + 0.035  # (past the joint's end: the arms lag about 1.5 cm)
    n_strokes = 4
    stops = [tip_in[0] + (x_end - tip_in[0]) * k / n_strokes for k in range(n_strokes + 1)]
    prime = SPAN_REST - 0.018  # (the fingers pressing the trigger past its slack: the silicone starts to come)
    for k in range(n_strokes):
        a0 = at(np.array([stops[k] - (0.012 if k else 0.0), tip_in[1], tip_in[2]]))  # (each starts on the last's end)
        a1 = at(np.array([stops[k + 1], tip_in[1], tip_in[2]]))
        if k:
            prev = at(np.array([stops[k], tip_in[1], tip_in[2]]))
            _move(plan, prev, a0, 0.9, "let the trigger come back", 3, full, back, RDX, 0.0)
        _move(plan, a0, a0, 1.2, "squeeze the trigger: the silicone comes", 4, back if k else held, prime)
        _move(plan, a0, a1, 2.8, "squeeze the trigger while drawing the nozzle along the joint", 8, prime, near,
              0.0, RDX)
        _move(plan, a1, a1, 0.4, "squeeze the trigger to its stop", 2, near, full, RDX)
    end = at(np.array([x_end, tip_in[1], tip_in[2]]))
    out = (end[0] + [0.0, -0.025, 0.015], PITCH)
    _move(plan, end, out, 0.8, "lift the nozzle out of the joint", 2, full, held, RDX, 0.0)
    _move(plan, out, lift, 2.2, "carry the gun back over its rest", 5, held)
    down = (GUN0 + [0, 0, -0.004], 0.0)
    _move(plan, lift, down, 1.2, "set it down on its grip and rest block", 3, held)
    o = down[0]
    L.append(_w(0.6, o + EAR, YOKE_FINGER, "let go of the yoke", LY_ROT))
    R.append(_w(0.6, o + _grip_tcp(SPAN_REST), _span_cmd(SPAN_REST + 0.010), "let go of the grip", RG_ROT))
    L.append(_w(0.8, GUN0 + EAR + [0, 0, OVER + 0.03], YOKE_FINGER, "rise clear", LY_ROT))
    wide = _span_cmd(SPAN_REST + 0.012)
    R.append(_w(0.8, GUN0 + _grip_tcp(SPAN_REST) - RG_ROT[:, 2] * 0.05, wide, "back out", RG_ROT))
    R.append(_w(1.0, _stage(GUN0), wide, "back out", RG_ROT))
    from yam_arena.constants import READY

    look = np.array([(JOINT_X0 + JOINT_X1) / 2, WALL_Y - 0.10, TOP + 0.19])  # the wrist camera over the joint
    L += _joint_path("left", _good_ik("left", GUN0 + EAR + [0, 0, OVER + 0.03], LY_ROT),
                     _good_ik("left", look, grip_rotation(0.0)), "look over the sealed joint", FINGER_CLOSED)
    R += _joint_path("right", _good_ik("right", _stage(GUN0), RG_ROT), np.array(READY, float), "back out of the way",
                     wide)
    _sync(plan, "wait", "wait")
    for arm in ("left", "right"):
        last = plan[arm][-1]
        plan[arm].append(_w(3.0, last.pos, last.finger, "hold still: the trial ends once the full score has held 1 s",
                            last.yaw))
