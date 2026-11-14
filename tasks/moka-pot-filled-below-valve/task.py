"""A 3-cup aluminum stovetop moka pot, taken apart: its empty water chamber (the base, with the brass safety valve on
its side) already stands on the cold burner grate of a gas stove, its upper chamber stands upright on the counter to
the left of the stove, a glass carafe of water stands on the counter to the right, and the funnel basket, already
filled with ground coffee, stands in an espresso cup behind it. Making moka coffee means filling the base with water up
to just below the safety valve, never over it (the valve could then not vent the steam if the funnel clogs) and not
far under it (weak, under-extracted coffee), dropping the filled basket into the base, and screwing the upper chamber
onto the base until it is snug, so steam cannot escape at the gasket and the pot is safe to heat.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pourkit as pk  # noqa: E402  (the water: volumes, pours, spills)
import twistkit as tk  # noqa: E402  (regrasp twist plans and the signed wrist-roll grip accounting reused below)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted grips)
import guardkit as gk  # noqa: E402  (gripper contact: holding and letting go)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the moka pot ready to brew."
D = math.radians

# Real size throughout (a 3-cup moka pot is 15.9 cm tall assembled).
# The stove: a steel hob plate with one burner and a four-finger cast-iron grate, centered on the pot.
BX, BY = 0.27, 0.05  # toward the left arm: it can lift its open hand over the whole upper chamber here (see twist)
PLATE = (0.14, 0.14, 0.004)
GRATE_H = 0.014
FLOOR_Z = TOP + PLATE[2] + GRATE_H  # the grate fingers' top: the pot's base stands here

# The base (in a fixed holder on the grate, in the sim and the real build; in a kitchen the other hand steadies it):
# an aluminum water chamber 8.2 cm across at the foot, 7.2 cm tall, tapering to a threaded collar 6.6 cm across.
# Heights are in its own frame (origin on its foot, on its axis). Inside it steps in three times (cast wall); the
# water is modeled in these three sections.
B_FOOT_R, B_COLLAR_R, RIM = 0.041, 0.033, 0.072
B_FLOOR = 0.004
SECTIONS = ((0.036, 0.020), (0.033, 0.020), (0.031, 0.028))  # (inside radius, height) from the floor up
COLLAR_H = 0.012  # the thread on the collar's outside, under the upper chamber's skirt when screwed on
VALVE_Z = 0.050  # the safety valve's center; its bore inside the wall is 8 mm across
VALVE_BORE_R = 0.004
TOP_LINE_Z = VALVE_Z - VALVE_BORE_R  # 4.6 cm: water above the bore's bottom edge stands in the valve


def ml_below(z):
    """mL the base holds up to height z (its own frame)."""
    v, z0 = 0.0, B_FLOOR
    for r, h in SECTIONS:
        dz = min(max(z - z0, 0.0), h)
        v += math.pi * r * r * dz * 1e6
        z0 += h
    return v


VALVE_ML = ml_below(TOP_LINE_Z)  # 155.8 mL: up to the valve bore's bottom edge
LOW_ML = ml_below(TOP_LINE_Z - 0.010)  # 122.5 mL: 1 cm below it (a fifth less water: weak, under-extracted coffee)
DRY_ML = 30.0  # under this nothing worth brewing went in (do-nothing included)

# The upper chamber (free, 0.20 kg): a 7.0 cm wide aluminum pot with a lid, knob, spout and a black handle, 8.7 cm
# tall, its skirt (female thread) dropping 1.2 cm over the base's collar. Frame origin: the center of its bottom
# face, which rests on the base's rim when it is on.
T_R, T_TOP_R, T_H = 0.035, 0.040, 0.087
SKIRT_IN, SKIRT_OUT = 0.036, 0.0425  # the skirt (female thread) round the base's collar
RIB_R = 0.0385  # its ribs' centers (2.5 mm round): inner edge 3.6 cm off the axis, the collar's collider is 3.4 cm
GRIP_DZ = 0.025  # grip height on its body, above its bottom face
T_WIDTH = 2 * T_R
HANDLE_ANG = D(45)  # the handle points 45 degrees off the fingers' closing axis at the start (fingers clear of it)

# The carafe (free): a 4.9 cm glass carafe 13.5 cm tall holding 175 mL. Pouring all of it in brings the base to 175
# mL, 19 mL over the valve: the pour has to stop.
JUG_XY = (0.23, -0.12)
JUG_R, JUG_H, JUG_IN, JUG_BASE = 0.0245, 0.135, 0.022, 0.004
JUG0 = 175.0
JUG_POS = (*JUG_XY, TOP + JUG_H / 2)
FOOT = 0.001
TOP_XY = (0.24, 0.19)

# The funnel basket (free, 40 g: 25 g of stainless steel and 15 g of ground coffee), a stainless replacement funnel for
# a 3-cup pot: a flange 6.5 cm across and 1.5 mm thick, a cup 6.1 cm across at the top tapering to 4.6 cm at its
# perforated floor 2.0 cm down, and a stem 1.0 cm across and 3.8 cm long. Frame origin: the center of the flange's
# bottom face, which rests on the base's rim when it is in (the upper chamber then sits on the flange, 1.5 mm higher).
# The coffee is level with the flange.
K_FLANGE_R, K_FLANGE_T = 0.0325, 0.0015
K_TOP_R, K_FLOOR_R, K_DEPTH = 0.0305, 0.023, 0.020
K_STEM_R, K_STEM_L = 0.005, 0.038
# It starts standing in a ceramic espresso cup (5.4 cm across, 5.0 cm tall, 4.2 cm across inside its mouth): the
# basket's floor rests on the cup's rim and its stem hangs inside, the way it is set down to be filled.
CUP_XY, CUP_R, CUP_IN, CUP_H = (0.36, -0.07), 0.027, 0.021, 0.050
BASKET_POS = (*CUP_XY, TOP + CUP_H + K_DEPTH + 0.0005)
# The base's wall inside the collar, as 24 posts round the 6.2 cm opening (the cup and stem go down inside them, the
# flange rests on their tops at the rim): center radius and post radius
WALL_R, POST_R, N_POSTS = 0.0325, 0.0016, 24
# The basket and the base's mouth are bare metal: they get METAL_MU (static, dynamic), a handbook dry value for steel
# on aluminum (the stainless basket on the aluminum base), instead of yam_arena's rubber-like default (1.2, 1.0) for
# things a gripper picks up. With the rubber value a basket dropped 14 degrees crooked wedged on the square-edged posts
# that model the mouth, 7 mm up (sim runs); with this one it slides down flat.
METAL_MU = (0.6, 0.47)

ALU = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=1.0)
DARK_ALU = kit.Look((0.45, 0.46, 0.48), 0.5, metallic=1.0)
BRASS = kit.Look((0.86, 0.66, 0.22), 0.3, metallic=1.0)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.5)
IRON = kit.Look((0.12, 0.12, 0.13), 0.7)
COFFEE = kit.Look((0.20, 0.12, 0.07), 0.9)
CERAMIC = kit.Look((0.93, 0.92, 0.89), 0.25)
STEEL = kit.Look((0.55, 0.56, 0.58), 0.35, metallic=1.0)
GLASS_LOOK = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.28)
WATER_RGB = (0.25, 0.52, 0.95)


def _stove():
    t, g = PLATE[2], GRATE_H
    meshes = [kit.box((0, 0, t / 2), PLATE, STEEL),
              kit.cylinder((0, 0, t + 0.003), 0.028, 0.006, BLACK)]  # the burner cap
    cols = [kit.Collider((0, 0, t / 2), size=PLATE)]
    for a in (45, 135, 225, 315):  # grate fingers, diagonal so the pot's valve side (-x) stays in view
        c, s = math.cos(D(a)), math.sin(D(a))
        r0, r1 = 0.022, 0.066
        # a finger along its own radial direction, as four blocks (yamkit boxes are axis-aligned)
        for k in range(4):
            rr = r0 + (k + 0.5) * (r1 - r0) / 4
            p = (rr * c, rr * s, t + g / 2)
            meshes.append(kit.box(p, (0.012, 0.012, g), IRON))
            cols.append(kit.Collider(p, size=(0.012, 0.012, g)))
    ring = [(0.066 * math.cos(u), 0.066 * math.sin(u), t + g - 0.003) for u in np.linspace(0, 2 * math.pi, 41)]
    meshes.append(kit.band(ring, 0.006, 0.006, IRON, normal=(0, 0, 1)))
    return kit.Prop("stove", pos=(BX, BY, TOP), size=(PLATE[0], PLATE[1], t + g),
                    meshes=tuple(meshes), colliders=tuple(cols))


def _base():
    z1 = B_FLOOR + SECTIONS[0][1]
    z2 = z1 + SECTIONS[1][1]
    outer = [(0.0, 0.0), (B_FOOT_R, 0.0), (B_FOOT_R, 0.006), (0.036, RIM - COLLAR_H - 0.002),
             (B_COLLAR_R, RIM - COLLAR_H), (B_COLLAR_R, RIM),
             (SECTIONS[2][0], RIM), (SECTIONS[2][0], z2), (SECTIONS[1][0], z2), (SECTIONS[1][0], z1),
             (SECTIONS[0][0], z1), (SECTIONS[0][0], B_FLOOR), (0.0, B_FLOOR)]
    xr = 0.041 - 0.005 * (VALVE_Z - 0.006) / (RIM - COLLAR_H - 0.008)  # outer radius at the valve
    meshes = [kit.lathe(outer, ALU),
              # thread ridges on the collar (seen while the upper chamber is off)
              *[kit.band([(B_COLLAR_R * math.cos(u), B_COLLAR_R * math.sin(u), RIM - 0.003 - 0.003 * k)
                          for u in np.linspace(0, 2 * math.pi, 33)], 0.0012, 0.0008, DARK_ALU, normal=(0, 0, 1))
                for k in range(3)],
              # the safety valve on the side toward the robot (-x): a brass hex body and a pin
              kit.cylinder((-xr - 0.004, 0, VALVE_Z), 0.0065, 0.008, BRASS, axis="x", n=6),
              kit.cylinder((-xr - 0.009, 0, VALVE_Z), 0.002, 0.004, BRASS, axis="x"),
              # its bore inside the wall, a brass ring you see when you look into the base
              kit.cylinder((-SECTIONS[2][0] + 0.0004, 0, VALVE_Z), VALVE_BORE_R, 0.001, BRASS, axis="x")]
    # hollow, so the basket's cup and stem go down inside: a solid bottom 1 cm thick (the stem's end stops 2.5 mm
    # over it), then the wall as posts up to the rim (the base is fixed, nothing else reaches inside)
    cols = [kit.Collider((0, 0, 0.005), radius=0.039, height=0.010)]
    for k in range(N_POSTS):
        u = 2 * math.pi * k / N_POSTS
        cols.append(kit.Collider((WALL_R * math.cos(u), WALL_R * math.sin(u), (0.010 + RIM) / 2), radius=POST_R,
                                 height=RIM - 0.010))
    return kit.Prop("base", pos=(BX, BY, FLOOR_Z), size=(2 * B_FOOT_R, 2 * B_FOOT_R, RIM),
                    meshes=tuple(meshes), colliders=tuple(cols), friction=METAL_MU)


def _top():
    prof = [(0.0, 0.0), (SKIRT_IN, 0.0), (SKIRT_IN, -COLLAR_H), (SKIRT_OUT, -COLLAR_H), (SKIRT_OUT, 0.0),
            (T_R, 0.006), (T_R, 0.040), (0.039, 0.046),
            (T_TOP_R, T_H - 0.004), (0.036, T_H), (0.0, T_H)]
    c, s = math.cos(HANDLE_ANG), math.sin(HANDLE_ANG)
    meshes = [kit.lathe(prof, ALU),
              kit.cylinder((0, 0, T_H + 0.005), 0.008, 0.010, BLACK),  # lid knob
              kit.box((-c * 0.044, -s * 0.044, T_H - 0.008), (0.012, 0.012, 0.010), ALU)]  # spout (opposite the handle)
    cols = [kit.Collider((0, 0, 0.020), radius=T_R, height=0.040),
            kit.Collider((0, 0, (0.040 + T_H) / 2), radius=0.039, height=T_H - 0.040),
            kit.Collider((0, 0, T_H + 0.005), radius=0.008, height=0.010)]
    # the skirt's inside: eight round ribs hidden in it, 2 mm clear of the base's collar, so a top set on straight
    # drops over the collar and stays centered on it, as the female thread does
    for k in range(8):
        u = 2 * math.pi * k / 8
        cols.append(kit.Collider((RIB_R * math.cos(u), RIB_R * math.sin(u), -COLLAR_H / 2), radius=0.0025,
                                 height=COLLAR_H))
    for k in range(3):  # the handle: three short blocks stepping out along its direction
        r = 0.046 + 0.010 * k
        p = (c * r, s * r, 0.060 - 0.012 * k)
        meshes.append(kit.box(p, (0.012, 0.012, 0.022), BLACK))
        cols.append(kit.Collider(p, size=(0.012, 0.012, 0.022)))
    return kit.Prop("top", pos=(*TOP_XY, TOP + COLLAR_H + 0.0005), size=(0.11, 0.11, T_H + COLLAR_H + 0.01),
                    mass=0.20, meshes=tuple(meshes), colliders=tuple(cols))


def _jug():
    z0 = -JUG_H / 2
    shell = kit.lathe([(0.0, z0), (JUG_R, z0), (JUG_R, -z0), (JUG_IN, -z0), (JUG_IN, z0 + JUG_BASE),
                       (0.0, z0 + JUG_BASE)], GLASS_LOOK)
    return kit.Prop("jug", pos=JUG_POS, size=(2 * JUG_R, 2 * JUG_R, JUG_H), mass=0.20, meshes=(shell,),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=JUG_R, height=JUG_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


def _basket():
    f, d = K_FLANGE_T, K_DEPTH
    prof = [(0.0, -d - K_STEM_L), (K_STEM_R, -d - K_STEM_L), (K_STEM_R, -d), (K_FLOOR_R, -d), (K_TOP_R, 0.0),
            (K_FLANGE_R, 0.0), (K_FLANGE_R, f), (0.029, f), (0.029, f - 0.001), (0.0, f - 0.001)]
    meshes = [kit.lathe(prof, ALU),
              kit.cylinder((0, 0, f - 0.0008), 0.0288, 0.0012, COFFEE)]  # the ground coffee, level with the flange
    # the tapered cup as two straight bands (its mean width over each), so a gripper's pads bear on a flat face
    cols = [kit.Collider((0, 0, f / 2), radius=K_FLANGE_R, height=f),
            kit.Collider((0, 0, -0.007), radius=0.028, height=0.014),
            kit.Collider((0, 0, -0.017), radius=0.0245, height=0.006),
            kit.Collider((0, 0, -d - K_STEM_L / 2), radius=K_STEM_R, height=K_STEM_L)]
    return kit.Prop("basket", pos=BASKET_POS, size=(2 * K_FLANGE_R, 2 * K_FLANGE_R, d + K_STEM_L + f), mass=0.04,
                    meshes=tuple(meshes), colliders=tuple(cols), friction=METAL_MU)


def _cup():
    h = CUP_H
    prof = [(0.0, 0.0), (0.022, 0.0), (CUP_R, 0.006), (CUP_R, h), (CUP_IN, h), (CUP_IN, 0.008), (0.0, 0.008)]
    cols = [kit.Collider((0, 0, 0.004), radius=CUP_R, height=0.008)]
    for k in range(12):  # its wall as posts, so the stem hangs inside
        u = 2 * math.pi * k / 12
        cols.append(kit.Collider((0.024 * math.cos(u), 0.024 * math.sin(u), (0.008 + h) / 2), radius=0.003,
                                 height=h - 0.008))
    return kit.Prop("cup", pos=(*CUP_XY, TOP), size=(2 * CUP_R, 2 * CUP_R, h), meshes=(kit.lathe(prof, CERAMIC),),
                    colliders=tuple(cols))


OBJECTS = [_stove(), _base(), _top(), _jug(), _basket(), _cup()]


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw(q):
    r = _rot(q)
    return math.atan2(r[1, 0], r[0, 0])


def _rim(objects):
    b = objects["base"]
    return np.asarray(b.pos, float) + _rot(b.quat_xyzw) @ np.array([0.0, 0.0, RIM])


# On the base: the upper chamber's bottom face within 4 mm of the base's rim height, its axis within 4 mm of the base's
# (the skirt's 1.5 mm clearance over the collar plus the thread's 2.5 mm play when it has not caught) and its axis
# within 4 degrees of the base's (more, it has gone on crooked: cross-threaded).
ON_DZ, ON_DXY, ON_TILT = 0.004, 0.004, 4.0


def top_on(objects):
    t, b = objects["top"], objects["base"]
    d = np.asarray(t.pos, float) - _rim(objects)
    at, ab = _rot(t.quat_xyzw)[:, 2], _rot(b.quat_xyzw)[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(at @ ab)))))
    return abs(d[2]) < ON_DZ and math.hypot(d[0], d[1]) < ON_DXY and tilt < ON_TILT


def _lets_water_in(objects):
    """The base's mouth is open unless the upper chamber sits over it (within 3 cm above the rim and 5 cm across), or
    the basket is in it or just over it: a stream then lands on the grounds and runs off over the rim (spilled)."""
    rim = _rim(objects)
    d = np.asarray(objects["top"].pos, float) - rim
    k = np.asarray(objects["basket"].pos, float) - rim
    return not ((math.hypot(d[0], d[1]) < 0.05 and -0.005 < d[2] < 0.03)
                or (math.hypot(k[0], k[1]) < 0.035 and -0.01 < k[2] < 0.04))


# The basket is in: its flange's bottom within 2 mm of the rim (resting on it, not propped on something), its axis
# within 4 mm of the base's (the modeled cup, a band 5.6 cm across, can sit 2.9 mm off center in the 6.2 cm opening)
# and within 5 degrees of it.
SEAT_DZ, SEAT_DXY, SEAT_TILT = 0.002, 0.004, 5.0
COFFEE_TIP = 45.0  # degrees: tipped further, the grounds slide out of a 2 cm deep cup filled to its brim


def _tilt_deg(q, ref=(0.0, 0.0, 1.0)):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(q)[:, 2] @ np.asarray(ref, float))))))


def basket_seated(objects):
    k, b = objects["basket"], objects["base"]
    d = np.asarray(k.pos, float) - _rim(objects)
    return (abs(d[2]) < SEAT_DZ and math.hypot(d[0], d[1]) < SEAT_DXY
            and _tilt_deg(k.quat_xyzw, _rot(b.quat_xyzw)[:, 2]) < SEAT_TILT)


pk.setup([pk.Vessel("base", sections=SECTIONS, floor=B_FLOOR, fill={}, accepts=_lets_water_in),
          pk.Vessel("jug", sections=((JUG_IN, JUG_H - JUG_BASE),), floor=-JUG_H / 2 + JUG_BASE, fill={"water": JUG0})],
         liquids={"water": WATER_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(TOP)

# Scoring
SEAT = 450.0  # degrees of screwing from sitting on the collar to the gasket meeting the filter plate (a 3-cup pot's
#               1.25 turns, see README): snug is reaching this seat by the gripper's own turning
STOP = SEAT + 90.0  # degrees: the thread's hard stop, a quarter turn of gasket squeeze past the seat (model choice);
#                    the upper chamber turns no further clockwise on the base, so it cannot be over-turned
SPILL_MAX = 5.0  # mL, a teaspoon on the stove
UPRIGHT = 10.0  # degrees
GRIPPERS = ("left_gripper", "right_gripper")
HOLD_MIN = 0.45  # the pads squeeze the 7 cm body to a reading of 0.55-0.65 in the sim (0.74 nominal)
DEBT_MAX = D(720)  # counterclockwise wrist roll kept as debt against later credit (see _account)


# Contact is judged with the gripper's own shape (palm and both fingers at their present opening, guardkit boxes fitted
# to the simulator's collision meshes) against these shapes in each object's frame: within 2 mm counts as touching
# (the solver's resting gap plus the box fit), so a hand hovering near a part is not on it.
TOP_SHAPE = (gk.Ring(0.0, T_R, 0.0, 0.040), gk.Ring(0.0, T_TOP_R, 0.040, T_H),  # the lower and upper body
             gk.Ring(SKIRT_IN, SKIRT_OUT, -COLLAR_H, 0.0),  # the skirt
             gk.Seg((0.0, 0.0, T_H), (0.0, 0.0, T_H + 0.010), 0.008),  # the lid knob
             *[gk.Block((math.cos(HANDLE_ANG) * (0.046 + 0.010 * k), math.sin(HANDLE_ANG) * (0.046 + 0.010 * k),
                         0.060 - 0.012 * k), (0.012, 0.012, 0.022)) for k in range(3)])  # the handle
JUG_SHAPE = gk.Ring(0.0, JUG_R, -JUG_H / 2, JUG_H / 2)


def top_held_by(objects, g):
    """Fingers closed on the upper chamber: the gripper touching it (guardkit contact) with an opening between
    HOLD_MIN and its 7 cm width + 3 %, i.e. stopped by its body (an empty closed hand reads 0, an open one 1)."""
    if g not in objects:
        return False
    t = objects["top"]
    r = _rot(t.quat_xyzw)
    along = float((np.asarray(objects[g].pos, float) - np.asarray(t.pos, float)) @ r[:, 2])
    # the body is 7 cm across up to 4 cm up and 8 cm above that: the fingers' stop follows the width at the grip's
    # height, so a grip anywhere on the chamber's side counts
    want = (T_WIDTH if along <= 0.040 else 2 * T_TOP_R) / (2 * abs(FINGER_OPEN))
    op = kit.opening(objects, g)
    return HOLD_MIN <= op <= want + tk.HOLD_OVER and bool(gk.touching(objects, "top", TOP_SHAPE, grippers=(g,)))


def top_held(objects):
    return any(top_held_by(objects, g) for g in GRIPPERS)


# --- What happened during the run (success() and score() run after every control step; pourkit has already moved the
# water). yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.on = False  # the upper chamber was on the thread (engaged) at the last step
        self.rel = 0.0  # its yaw relative to the base at the last step
        self.screwed = 0.0  # rad screwed on (clockwise seen from above) with a gripper's own roll, since it went on
        self.turned = 0.0  # rad it has turned clockwise on the base since it went on, however (the thread's position)
        self.unheld = 0.0  # rad it turned on the base without that credit (knocked, swiped, dragged)
        self.reached = False  # a gripper's own turning has brought it down to the seat since it went on
        self.budget = 0.0  # signed wrist roll not yet matched by the top (twistkit's roll budget)
        self.yaw_last = {}
        self.was_held = False
        self.under = False  # the basket was in the base when the upper chamber last went on
        self.tipped = 0  # consecutive steps the basket has been tipped past COFFEE_TIP
        self.coffee_lost = None  # where the grounds fell out (latched), else None


_H = _History()


def _engaged(objects):
    """The skirt still round the collar: the top's bottom face less than the skirt's 1.2 cm above the rim, its axis
    within 6 mm and 8 degrees of the base's. A top that sat on the base stays on the thread while this holds; lifted
    past it (or pushed off), it is off and the count starts again."""
    t, b = objects["top"], objects["base"]
    d = np.asarray(t.pos, float) - _rim(objects)
    at, ab = _rot(t.quat_xyzw)[:, 2], _rot(b.quat_xyzw)[:, 2]
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(at @ ab)))))
    return -ON_DZ < d[2] < COLLAR_H and math.hypot(d[0], d[1]) < 0.006 and tilt < 8.0


def _account(objects):
    """Screwing the upper chamber on: once it has sat on the base, and while its skirt stays round the collar, its
    turning relative to the base counts, clockwise seen from above positive. As in twistkit, clockwise turning is
    credited only while a gripper holds it (at either end of the step) and only from a budget that holding gripper's
    own wrist roll earns (1.25 x its roll, signed, at most 15 degrees of credit carried over, counterclockwise roll
    kept as debt): another arm swiping it while a hand rests on it, or a hand wiggling back and forth, earns nothing.
    Counterclockwise turning always counts against it, held or not (dragged back, it is that much looser)."""
    on = top_on(objects)
    eng = _engaged(objects) and (on or _H.on)
    rel = _yaw(objects["top"].quat_xyzw) - _yaw(objects["base"].quat_xyzw)
    yaws = {g: tk._grip_yaw(objects[g]) for g in GRIPPERS if g in objects}
    holder = next((g for g in GRIPPERS if top_held_by(objects, g)), None)
    if eng and not _H.on:  # it just went on: was the basket already in under it?
        _H.under = basket_seated(objects) and _H.coffee_lost is None
    if eng and _H.on:
        du = -tk._wrap(rel - _H.rel)  # clockwise seen from above = tightening
        g = holder or _H.was_held
        credit = 0.0
        if not (_H.under and basket_seated(objects)):
            g = None  # screwing counts only with the basket in under it, put in before the top went on
        if g and g in yaws and g in _H.yaw_last:
            _H.budget += tk.ROLL_GAIN * -tk._wrap(yaws[g] - _H.yaw_last[g])
            if du * _H.budget > 0:
                credit = math.copysign(min(abs(du), abs(_H.budget)), du)
                _H.budget -= credit
            # Unused clockwise roll carries over at most 15 degrees (twistkit's cap), but counterclockwise roll is
            # kept in full as debt: twistkit clips both, so a closed hand wiggling +-20 degrees while another arm
            # swipes the top round earned about half the swipe; with the debt kept, net zero roll earns nothing.
            _H.budget = max(-DEBT_MAX, min(_H.budget, tk.ROLL_BUDGET_MAX))
        else:
            _H.budget = 0.0
        if du < 0:
            credit = du  # loosening counts whoever does it
        _H.screwed += credit
        _H.unheld += abs(du - credit)
        _H.turned += du
    elif not eng:
        _H.screwed, _H.budget, _H.turned, _H.reached = 0.0, 0.0, 0.0, False
    _H.reached = _H.reached or (eng and _H.screwed >= math.radians(SEAT))
    # past the seat (the stop is then within 90 degrees): the stop's relative yaw, handed to the physics-step hook,
    # which also holds the top on its seat from the first step it is down on the base there
    seated = eng and _H.turned > math.radians(SEAT)
    _STOP["yaw"] = rel - (math.radians(STOP) - _H.turned) if seated else None
    _STOP["seat"] = seated and (on or _STOP["seat"])
    _H.on, _H.rel = eng, rel
    _H.was_held = holder
    _H.yaw_last = yaws


def _watch_coffee(objects):
    """The grounds fall out if the basket is tipped past 45 degrees for two steps running (one step is a physics pop);
    latched, with where they fell."""
    k = objects["basket"]
    _H.tipped = _H.tipped + 1 if _tilt_deg(k.quat_xyzw) > COFFEE_TIP else 0
    if _H.tipped >= 2 and _H.coffee_lost is None:
        _H.coffee_lost = (float(k.pos[0]), float(k.pos[1]))


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
        _clear_stop()
    _watch_coffee(objects)
    _account(objects)


# --- The thread's stop and seat. There is no modeled thread (the upper chamber is a free body on the collar), so the
# sim applies what the thread does once the top is screwed down to the seat (the gasket on the filter plate): before
# every physics step (240 Hz) a hook holds the top where it sat down on the seat, on the base's axis and level with
# it, free only to turn about that axis (a seated thread lets it turn, not lift, tip or slide), and turns it back to
# the hard stop if it has gone past it clockwise, taking away its clockwise spin there. Turned back counterclockwise
# off the seat, the hold lets go. The scorer (20 Hz) says when the top is seated and where the stop is.
_STOP = {"yaw": None, "seat": False, "pose": None}  # the stop's relative yaw (rad) past the seat, else None; whether
#   the top is seated; and its pose (position, quaternion) on the seat, taken by the hook when it first sees it seated


def _clear_stop() -> None:
    _STOP.update(yaw=None, seat=False, pose=None)


def _hold_at_stop(base) -> None:
    if not _STOP["seat"]:
        _STOP["pose"] = None
    if _STOP["yaw"] is None:
        return
    import torch

    t = lambda a: getattr(a, "torch", a)  # noqa: E731
    top, bas = base.scene["top"], base.scene["base"]
    pose = t(top.data.root_link_pose_w)[0].clone()  # (x, y, z, then the quaternion x, y, z, w)
    q = t(top.data.root_link_quat_w)[0].detach().cpu().numpy().astype(np.float64)
    qb = t(bas.data.root_link_quat_w)[0].detach().cpu().numpy().astype(np.float64)
    if _STOP["seat"] and _STOP["pose"] is None:  # it has just come down on the seat: there the thread holds it
        _STOP["pose"] = (pose[:3].clone(), q.copy())
    past = -tk._wrap(_yaw(q) - _yaw(qb) - _STOP["yaw"])  # clockwise past the stop (rad)
    vel = t(top.data.root_com_vel_w)[0].clone()  # (vx, vy, vz, wx, wy, wz)
    if _STOP["pose"] is not None:  # on the seat: back where it sat, turned only about the vertical (and not past the
        #                            stop), with no sliding, lifting or tipping
        pos0, q0 = _STOP["pose"]
        h = tk._wrap(_yaw(q) - _yaw(q0) + max(past, 0.0)) / 2
        pose[:3] = pos0
        pose[3:7] = torch.tensor(kit._quat_mul(np.array([0.0, 0.0, math.sin(h), math.cos(h)]), q0),
                                 dtype=pose.dtype, device=pose.device)
        top.write_root_pose_to_sim_index(root_pose=pose[None])
        vel[0:5] = 0.0
    elif past > 0.0:  # past the seat but not down on the base (crooked): only turned back to the stop
        h = past / 2
        pose[3:7] = torch.tensor(kit._quat_mul(np.array([0.0, 0.0, math.sin(h), math.cos(h)]), q),
                                 dtype=pose.dtype, device=pose.device)
        top.write_root_pose_to_sim_index(root_pose=pose[None])
    if past > -0.01:  # at the stop: no clockwise spin (clockwise seen from above is -z)
        vel[5] = max(float(vel[5]), 0.0)
    top.write_root_velocity_to_sim_index(root_velocity=vel[None])


def _hook_physics() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_moka_stop", False):
        return
    cls._moka_stop = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None:
            return
        isc = env.unwrapped.scene
        if getattr(isc, "_moka_stop", False):
            return
        write = isc.write_data_to_sim

        def write_data_to_sim(*wa, **wk):
            try:
                _hold_at_stop(env.unwrapped)
            except Exception as e:  # noqa: BLE001
                if not _STOP.get("failed"):
                    print(f"moka: thread stop off ({e!r})", file=sys.stderr, flush=True)
                _STOP["failed"] = True
            return write(*wa, **wk)

        isc.write_data_to_sim, isc._moka_stop = write_data_to_sim, True
        base, reset = env.unwrapped, env.unwrapped.reset

        def reset_and_clear(*ra, **rk):  # a new trial: no seat or stop left over from the last one
            _clear_stop()
            return reset(*ra, **rk)

        base.reset = reset_and_clear

    cls._ensure_sim = _ensure_sim


_hook_physics()


def basket_ok(objects):
    """The basket in the base with its coffee still in it."""
    return basket_seated(objects) and _H.coffee_lost is None


def screwed_deg():
    return math.degrees(_H.screwed)


def screwed_snug(objects):
    """On the base and screwed down to the seat (450 degrees, 1.25 turns, the gasket on the filter plate) by a gripper's
    own roll, with the basket in under it before it went on, and still at or past the seat: turned back
    counterclockwise off the seat (by anything) it is loose again, while a few degrees of give at the stop (the
    fingers letting go of a top they pressed against it, the next stroke turning it back) leave it snug. The thread's stop (_hold_at_stop) keeps it from turning more than a
    quarter turn past the seat."""
    return top_on(objects) and _H.under and _H.reached and math.degrees(_H.turned) >= SEAT


def _jug_held(objects):
    """A gripper still touching the carafe (guardkit contact, within 2 mm)."""
    return bool(gk.touching(objects, "jug", JUG_SHAPE))


def _put_away(objects):
    """The carafe upright, let go and off the stove plate; nothing holding the pot (the base is held in a fixed holder
    on the grate, so it cannot be knocked over)."""
    j = objects["jug"]
    jug_ok = (pk.tilt(objects, "jug") < UPRIGHT and not _jug_held(objects)
              and not (abs(j.pos[0] - BX) < PLATE[0] / 2 and abs(j.pos[1] - BY) < PLATE[1] / 2))
    return jug_ok and not top_held(objects) and not _hand_on_top(objects)


def _hand_on_top(objects):
    """A gripper (palm or either finger) still touching the upper chamber anywhere: its body, skirt, lid knob or
    handle (guardkit contact, within 2 mm)."""
    return bool(gk.touching(objects, "top", TOP_SHAPE))


# The grounds, once fallen out, are drawn for the cameras: a disc of bare steel over the coffee in the basket and a
# brown patch where they fell (Isaac Lab visualization markers, drawn after pourkit's water; no physics).
_CV = {"markers": None, "failed": False}


def _coffee_markers():
    if _CV["markers"] is None and not _CV["failed"]:
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            def disc(look):
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in look.color),
                                                  roughness=look.roughness, metallic=look.metallic)
                return sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat)
            _CV["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/World/Visuals/moka_coffee", markers={"bare": disc(ALU), "grounds": disc(COFFEE)}))
        except Exception as e:  # noqa: BLE001  (visuals are optional: scoring never depends on them)
            print(f"moka: no coffee visuals ({e!r})", file=sys.stderr, flush=True)
            _CV["failed"] = True
    return _CV["markers"]


def _draw_coffee(objects, origin):
    lost = _H.coffee_lost if _H.episode == kit.episode() else None
    if lost is None and _CV["markers"] is None:
        return  # nothing to draw yet (markers are made on the first spill, so the first frames are untouched)
    m = _coffee_markers()
    if m is None:
        return
    hide = ((0.0, 0.0, -10.0), (0, 0, 0, 1), (1e-5, 1e-5, 1e-5))
    if lost is None:
        items = [hide, hide]
    else:
        k = objects["basket"]
        top = np.asarray(k.pos, float) + _rot(k.quat_xyzw) @ np.array([0.0, 0.0, K_FLANGE_T - 0.0001])
        on_plate = abs(lost[0] - BX) < PLATE[0] / 2 and abs(lost[1] - BY) < PLATE[1] / 2
        z = TOP + (PLATE[2] if on_plate else 0.0) + 0.0005
        items = [(tuple(top), tuple(float(q) for q in k.quat_xyzw), (0.0292, 0.0292, 0.0006)),
                 ((lost[0], lost[1], z), (0, 0, 0, 1), (0.035, 0.028, 0.001))]
    m.visualize(translations=np.array([np.asarray(p, float) + origin for p, _, _ in items]),
                orientations=np.array([q for _, q, _ in items], float),
                scales=np.array([sc for _, _, sc in items], float), marker_indices=np.array([0, 1]))


_pk_draw = pk._draw


def _draw_all(objects, origin):
    _pk_draw(objects, origin)
    try:
        _draw_coffee(objects, origin)
    except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
        if not _CV["failed"]:
            print(f"moka: coffee visuals off ({e!r})", file=sys.stderr, flush=True)
        _CV["failed"] = True


pk._draw = _draw_all


def _job(objects):
    water = pk.volume(objects, "base")
    if water > VALVE_ML or water < DRY_ML:
        return 0.0
    basket, snug = basket_ok(objects), screwed_snug(objects)
    if water < LOW_ML:  # weak coffee: some credit only if the pot is otherwise put together
        return 0.25 if basket and snug else 0.0
    if not basket:
        return 0.25
    if not snug:
        return 0.5
    if pk.spilled(objects) > SPILL_MAX or not _put_away(objects):
        return 0.75
    return 1.0


def success(objects):
    """The job done; where the arms end up is not scored. Called first each step, so it also records the history."""
    _update(objects)
    return _job(objects) == 1.0


def score(objects):
    """0 with more than 155.8 mL in the base at the end (water above the bottom of the safety valve's bore) or under 30
    mL (nothing to brew; includes doing nothing), and with 30 to 122.5 mL unless the pot is otherwise ready (then
    0.25: weak coffee). With the base filled between the lines: 0.25 if the basket is not in it with its coffee
    (flange within 2 mm of the rim, axis within 4 mm and 5 degrees; tipped past 45 degrees on the way, the grounds
    fell out); 0.5 if the upper chamber is not screwed snug over it (on the base, put on after the basket went in and
    turned clockwise to the seat, 450 degrees, by a gripper's own wrist roll since; a hard stop 90 degrees past the
    seat keeps it from turning further); 0.75 with more than 5 mL spilled, the carafe not set down upright, let go
    and off the stove, or a gripper still touching the upper chamber. 1.0 otherwise. Possible scores: 0, 0.25, 0.5,
    0.75, 1."""
    return _job(objects)


# --- Demo
YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.065
LIP_ABOVE = 0.05
POUR_PITCH = 1.40  # rad: tuned in the sim so the base ends between the lines (see README)
TWIST_DEG = 630.0  # up to 7 strokes of 90 degrees; the pads slip a little (about 86 degrees a stroke), so the run
# ends in success after the sixth, with the top screwed about 514 degrees
STROKE = 90.0  # a 90-degree stroke keeps the handle 45 degrees off the fingers at every regrasp


def pour_plan(pitch=POUR_PITCH):
    sx, sy, _ = JUG_POS
    gz = TOP + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("jug", JUG_POS, toward=(1, 0, 0))
    over = (BX - 0.008, BY, FLOOR_Z + RIM + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the carafe"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the carafe"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=0.85, to_pitch=pitch, above=0.05, steps=8,
                    duration=6.0, hold=2.0, back=3.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away")]
    return plan


# The basket: gripped straight down across its cup (tool point 7.5 mm under the flange, the pad tips level with the
# cup's floor), carried over the base and let go with the flange 3.2 cm over the rim, the cup's floor 1.2 cm over it
# and its stem already inside; the tapered cup centers itself as it drops.
K_GRIP_DZ = -0.0075
K_RELEASE = 0.032
# The fingers close along the carry from the cup to the base (toward -x and +y, 45 degrees off each): a pinched basket
# swings about the line through the two pads, so a carry across that line tipped it 13 degrees in the grip (sim runs).
K_YAW = math.pi / 4
# In the grip the basket sits 1.5 cm off the tool point along the fingers' line: the first finger to touch it pushes it
# along until its stem meets the espresso cup's wall, and the two pads then hold it wherever it is. Opened all at once,
# the fingers flick it back toward the tool point and it lands crooked on the rim (sim runs). So the tool point is
# carried over the base's axis and the grip is eased in two steps: the basket slides back between the pads to the axis,
# still held, and then drops in level.


def basket_plan():
    kx, ky, kz = BASKET_POS
    gz = kz + K_GRIP_DZ
    rz = FLOOR_Z + RIM + K_RELEASE + K_GRIP_DZ
    travel = TOP + 0.20
    y = K_YAW
    return [Waypoint(2.0, (kx, ky, travel), FINGER_OPEN, "over the basket", y),
            Waypoint(1.5, (kx, ky, gz + 0.03), FINGER_OPEN, "down to it", y),
            Waypoint(1.0, (kx, ky, gz), FINGER_OPEN, "around the cup", y),
            Waypoint(1.0, (kx, ky, gz), FINGER_CLOSED, "grip the basket", y),
            Waypoint(1.5, (kx, ky, rz + 0.06), FINGER_CLOSED, "lift it out of the cup", y),
            Waypoint(3.5, (BX, BY, rz + 0.06), FINGER_CLOSED, "over the base", y),
            Waypoint(1.0, (BX, BY, rz + 0.06), FINGER_CLOSED, "let it stop swinging", y),
            Waypoint(2.0, (BX, BY, rz), FINGER_CLOSED, "stem into the base", y),
            Waypoint(0.8, (BX, BY, rz), FINGER_CLOSED, "steady", y),
            Waypoint(1.0, (BX, BY, rz), 0.35 * FINGER_OPEN, "ease the grip", y),
            Waypoint(1.0, (BX, BY, rz), 0.45 * FINGER_OPEN, "let it slide over the axis", y),
            Waypoint(0.6, (BX, BY, rz), FINGER_OPEN, "let it drop in", y),
            Waypoint(1.0, (BX, BY, rz + 0.06), FINGER_OPEN, "up off it", y)]


def right_plan():
    """Pour, set the carafe down, then put the basket in; returns the plan and when the right hand is clear of the
    base (s)."""
    plan = pour_plan() + basket_plan()
    clear = sum(w.duration for w in plan)
    return plan + [kit.Home(2.0), kit.Home(1.5, label="hold at home")], clear


# Measured in the sim (demo runs): with a straight-down grip over the base the left arm's tool point settles about
# (-2.3, 0.9) mm off its command, and a held top's axis sits (1.5, -9.7) mm off the tool point at wrist yaw 0 (turning
# with the wrist). Commands are offset so the top's axis, not the tool point, is over the base and the wrist turns
# about that axis.
ARM_ERR = np.array((-0.0023, 0.0009))
PAD_OFF = np.array((0.0015, -0.0097))
PRESS = 0.002  # m: set the top down this far into the rim so it settles


def _about_axis(yaw):
    """Tool point xy that puts a held top's axis on the base's axis at wrist yaw `yaw` (rad)."""
    c, s = math.cos(yaw), math.sin(yaw)
    off = np.array((c * PAD_OFF[0] - s * PAD_OFF[1], s * PAD_OFF[0] + c * PAD_OFF[1]))
    return tuple(np.array((BX, BY)) - off - ARM_ERR)


# The open hand rolls back above the whole upper chamber: a top that turns more than 45 degrees per stroke brings its
# handle under the fingers' path back, so they rise until the pad tips clear the handle (7.1 cm up), the spout (8.4 cm)
# and the lid (8.7 cm), with 1 cm to spare.
LIFT_OVER = T_H + 0.010 + PAD_TIP_PAST_TCP - GRIP_DZ  # 8.5 cm over the grip height


def _centered():
    """Tool point xy that puts the open hand's fingers evenly round the top's axis (each 4.7 cm out, 7 mm clear of
    its 4.0 cm upper body). At the grip offset (see _about_axis) one open finger is only 3.7 cm out, under that
    upper body: lifted from there it hooks the top and tips it off the collar (runs 5 and 6)."""
    return tuple(np.array((BX, BY)) - ARM_ERR)


def twist(z, degrees, start_yaw, stroke=STROKE, step=15.0, step_s=0.3, grip_s=0.7, clear=0.03):
    """Regrasp loop like twistkit's twist_waypoints (grip, roll `stroke` degrees clockwise in `step` steps, let go,
    lift the open hand over the top, roll back, come down round it again, repeat), but turning about the top's axis
    rather than the tool point (see _about_axis). The open hand moves up and down only centered on the top."""
    label = "screw the top on"
    y0 = math.radians(start_yaw)
    c = _centered()
    zo = z + LIFT_OVER
    out = [Waypoint(1.2, (*c, z + clear), FINGER_OPEN, f"{label}: over the top", y0),
           Waypoint(1.2, (*c, z), FINGER_OPEN, f"{label}: around it", y0)]
    left = degrees
    while left > 1e-6:
        sdeg = min(stroke, left)
        n = max(1, math.ceil(sdeg / step))
        out.append(Waypoint(0.6, (*_about_axis(y0), z), FINGER_OPEN, f"{label}: to the grip offset", y0))
        out.append(Waypoint(grip_s, (*_about_axis(y0), z), FINGER_CLOSED, f"{label}: grip", y0))
        for k in range(1, n + 1):
            yk = y0 - math.radians(sdeg * k / n)
            out.append(Waypoint(step_s, (*_about_axis(yk), z), FINGER_CLOSED, f"{label}: turn", yk))
        yend = y0 - math.radians(sdeg)
        out.append(Waypoint(grip_s, (*_about_axis(yend), z), FINGER_OPEN, f"{label}: let go", yend))
        out.append(Waypoint(0.6, (*c, z), FINGER_OPEN, f"{label}: center the open hand", yend))
        left -= sdeg
        if left > 1e-6:  # up over the handle, roll back there, and down round the body again
            out.append(Waypoint(1.0, (*c, zo), FINGER_OPEN, f"{label}: lift over", yend))
            for k in range(1, n + 1):
                yk = yend + math.radians(sdeg * k / n)
                out.append(Waypoint(step_s * 0.8, (*c, zo), FINGER_OPEN, f"{label}: roll back", yk))
            out.append(Waypoint(1.0, (*c, z), FINGER_OPEN, f"{label}: back down", y0))
    # up over the whole top before going home: from 3 cm up, the way home swept a finger through the handle and
    # tipped the top off the thread (run 8)
    out.append(Waypoint(1.2, (*c, zo), FINGER_OPEN, f"{label}: lift clear", yend))
    return out


def left_plan(wait):
    """Pick the upper chamber up by its body and hold it until `wait` s (the carafe is away), set it on the base,
    let go, then screw it on clockwise with 90-degree regrasp strokes and go home."""
    tx, ty = TOP_XY
    gz = TOP + COLLAR_H + 0.0005 + GRIP_DZ
    cx, cy = _about_axis(0.0)
    seat_gz = FLOOR_Z + RIM + K_FLANGE_T + GRIP_DZ - PRESS  # the top sits on the basket's flange
    plan = [Waypoint(2.0, (tx, ty, gz + 0.10), FINGER_OPEN, "over the upper chamber", 0.0),
            Waypoint(1.0, (tx, ty, gz + 0.10), FINGER_OPEN, "steady", 0.0),
            Waypoint(2.0, (tx, ty, gz), FINGER_OPEN, "around it", 0.0),
            Waypoint(0.8, (tx, ty, gz), FINGER_CLOSED, "grip", 0.0),
            Waypoint(1.2, (tx, ty, seat_gz + 0.03), FINGER_CLOSED, "lift", 0.0)]
    # held 2 cm further back and to the left than where it stood: nearer, the right arm reaching over the base with the
    # basket brushed the held chamber and was pushed 3 cm off the base's axis (sim runs)
    plan.append(Waypoint(max(0.5, wait - sum(w.duration for w in plan)), (tx - 0.02, ty + 0.02, seat_gz + 0.03),
                         FINGER_CLOSED, "hold it aside while the base is filled", 0.0))
    plan += [Waypoint(2.0, (cx, cy, seat_gz + 0.03), FINGER_CLOSED, "over the base", 0.0),
             Waypoint(1.5, (cx, cy, seat_gz + 0.016), FINGER_CLOSED, "lower to the collar", 0.0),
             Waypoint(1.0, (cx, cy, seat_gz + 0.016), FINGER_CLOSED, "settle", 0.0),
             Waypoint(1.5, (cx, cy, seat_gz), FINGER_CLOSED, "down over the collar", 0.0),
             Waypoint(0.8, (cx, cy, seat_gz), FINGER_OPEN, "let go", 0.0)]
    lo, hi = tk.yaw_range("left", (BX, BY, seat_gz))
    start = math.floor((hi - 10) / 90.0) * 90.0  # a multiple of 90: the fingers land where they first gripped
    if start - STROKE < lo:
        raise ValueError(f"left wrist range {lo:.0f}..{hi:.0f} is too small for a {STROKE:.0f}-degree stroke")
    plan += twist(seat_gz + PRESS, TWIST_DEG, start)
    plan += [kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The right arm pours from the carafe into the base, tipping it about its lip 5 cm over the rim, sets it back
    down, then lifts the basket out of its cup and drops it into the base. The left arm then sets the upper chamber on
    the base and screws it on with 90-degree wrist strokes (up to seven); once it is snug and let go the run ends."""
    right, clear = right_plan()
    return {"right": right, "left": left_plan(clear)}
