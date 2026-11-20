"""At a bedside (the floor of a 28 x 30 in plastic drip pan on the station table stands in for the floor beside the bed;
the pan catches the water of a failed trial): a Bard 154002 2000 mL urinary drainage bag hangs by its hook from a
catheter-bag floor holder, its front toward the robot, holding 600 mL of clear tap water standing in for urine (in the
sim and the real trial). Its drain tube is folded up in front of the bag with the twist-valve spout pushed up into its
sleeve, the valve closed. A blue absorbent underpad lies on the floor under the bag; a clean, empty 1000 mL graduated
measuring jug with a handle (the graduate) stands beside it on the robot's right.

The job is a nursing assistant's routine, "Measure and record food and liquid intake or urinary and fecal output"
(O*NET 31-1131.00), done the way the CDC's catheter guideline requires (III.B.3): empty the bag into a separate clean
container, avoid splashing, and never let the drain spout touch the container. As nurses do it: set the graduate on the
underpad under the drain; one hand takes the drain tube out of its sleeve and aims the spout into the graduate's mouth
without letting the tube touch it; the other hand opens the valve; the aiming hand follows the stream as its arc
shortens while the bag empties; the valve is closed and the spout is pushed back up into its sleeve. The real failures
are kept possible: the spout or the tube touches the graduate, the stream misses the mouth (an angled spout throws it
forward, and the throw changes as the bag empties), the tube is dropped and hangs down draining onto the floor, the
valve is left open, the graduate is knocked over, the spout is left out of its sleeve or only half in.

The bag and tube are one yamkit articulation on the holder, its parts colliding with each other: the bag swings on its
hook (a pull on the tube moves it), the drain tube is two links (an 8.5 cm run off the outlet, then 11 cm) on hinges with
a sideways hinge at the outlet, and at its end a twist-valve collar (22 mm long, 12 mm across), with the spout (15 mm
long, 8 mm across) past it, turns on the tube between its closed and open stops. Nothing pulls the tube or the collar anywhere: the tube's hinges carry only damping,
the collar's turn damping and its seal's friction (it stays where it is left), and the spout stays in its sleeve by
contact and friction. The water is a lumped flow model: the bag is a pillow-shaped volume
(its section grows over the bottom 8 cm, so 600 mL stands 10 cm deep); it drains through the spout as an orifice
(Torricelli, Cd 0.61) with the head from the bag's level down to the spout, scaled by how far the valve is open; the
stream leaves the spout along its axis at sqrt(2 g h) and falls ballistically into the graduate's mouth or onto the
floor (counted as spilled); a stream that runs into either arm (the robot's own collision hulls) splashes off it onto
the floor and is spilled too. The graduate is a liquidkit vessel (its level, its slosh when moved, and a spill over its
rim if tipped or jolted).

A touch is real contact with the surface the real trial coats with lipstick: any point of the drain tube's surface
(the run out of the bag, the folded-up tube, the valve collar and the spout past it) within 0.5 mm of the
graduate's rim (its top edge, the pour spout's lip included), inside wall or inside floor, checked every control step
and along the motion between steps, from the start of the trial to its end; and, as lipstick rides on a hand, a robot
link that has touched that surface and then touches the tube. A stream that runs into a hand or arm for more than 1 s
in all scores 0. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import liquidkit as lk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPY_FRICTION  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = ("Empty the drainage bag into the graduate without letting the drain tube or its spout touch the "
               "graduate, then close the drain and push the spout back into its sleeve.")
ML = 1e-6  # m^3 per mL
G = lk.G

# -- the drip pan (spill containment) -----------------------------------------------------------------------------------
# Oatey 34169 plastic washing machine pan (plain, no ribs): 28 x 30 in inside, 2.5 in deep, 0.09 in minimum thickness
# (Oatey's spec sheet for its 28 x 30 in pans), modeled 30 x 32 in outside with its sloped walls and lip (a retailer
# lists 29.75 x 31.75 in overall for the same pan with ribs). It comes with a 1 in PVC drain adapter, left off, and its
# drain hole is taped shut. It stands on the station table under the whole scene: a dropped tube, a missed stream or a
# knocked-over graduate drains into it, not onto the table. Its floor is the scene's floor (the floor beside the bed):
# every height below is from it. Each sloped wall and its lip are modeled as the solid band between the inside floor's
# edge and the outside edge, the full depth high (more than the real wall: the arms are checked against the larger
# shape).
PAN_IN = (0.7112, 0.7620)  # inside floor, x (away from the robot) by y
PAN_OUT = (0.7620, 0.8128)  # outside, with the lip
PAN_H, PAN_T = 0.0635, 0.0023  # depth; floor thickness (the 0.09 in minimum)
PAN_X0, PAN_Y0 = 0.065, -0.420  # its overall near right corner, in front of the arm bases (test_task.py: clearance)
FLOOR = TOP + PAN_T
PAN_RGB = (0.93, 0.93, 0.91)

# -- looks --------------------------------------------------------------------------------------------------------------
VINYL = kit.Look((0.90, 0.92, 0.93), 0.3)  # the bag's clear vinyl (drawn as its back panel and seams)
VINYL_EDGE = kit.Look((0.78, 0.80, 0.82), 0.4)
PRINT = kit.Look((0.35, 0.45, 0.70), 0.5)  # the bag's printed graduation panel
TUBE = kit.Look((0.86, 0.90, 0.92), 0.25)  # clear PVC drain tube
VALVE = kit.Look((0.20, 0.45, 0.80), 0.4)  # the blue twist-valve collar
STEEL = kit.Look((0.75, 0.76, 0.78), 0.3, metallic=0.8)
PP = kit.Look((0.93, 0.93, 0.90), 0.35, opacity=0.6)  # the graduate's transparent polypropylene, drawn see-through
WATER_RGB = (0.66, 0.83, 0.96)  # clear tap water: light, slightly blue-tinted
WATER_OPACITY = 0.55  # drawn semi-transparent (RTX fractional cutout opacity)
JET_OPACITY = 1.0  # the thin jet drawn solid (a few pixels wide: partial opacity would only speckle it)
PAD_RGB = (0.55, 0.70, 0.88)  # the blue absorbent underpad on the floor

# -- the holder and the bag ---------------------------------------------------------------------------------------------
# Catheter bag floor holder for drainage bags: 50 cm tall, 21 cm deep, 300 g, plastic-coated metal (Essential Aids). Its
# base sits on the floor (the pan's floor) behind the bag; the bag's hanger hangs on the hook at the top, 480 mm up.
STAND_X = 0.450
BAG_Y = -0.040
STAND_POS = (STAND_X, BAG_Y, FLOOR)  # the holder's base, middle, on the floor: the articulation's fixed base frame
HOOK = (-0.030, 0.0, 0.480)  # the hook, in the base frame
# Bard 154002 urinary drainage bag, 2000 mL, vinyl, single hook (BD product page; retailer listings: twist valve
# drain tube). Inside modeled as a box 18 cm wide, 4.5 cm thick, 25 cm tall (2.03 L), hanging 2 cm below the hook on
# its hanger. (The maker lists the capacity; the model dimensions are checked against the trial bag at setup.)
STRAP = 0.020
BAG_W, BAG_T, BAG_H = 0.180, 0.045, 0.250
BAG_AREA = BAG_W * BAG_T  # m^2: the full section (81 cm^2) above the bottom's bulge
# A filled vinyl bag is a pillow: welded flat along its bottom seam, it bulges to its full thickness a few cm up. The
# liquid's section grows as sqrt(height) over the bottom BAG_BULGE_H, then stays BAG_AREA: 600 mL stands 10 cm deep.
BAG_BULGE_H = 0.080  # m
_BULGE_V = BAG_AREA * 2.0 / 3.0 * BAG_BULGE_H  # m^3 held in the bulge (432 mL)


def bag_depth(volume_m3: float) -> float:
    """Height of the urine's surface above the bag's bottom seam for this volume."""
    v = max(volume_m3, 0.0)
    if v <= _BULGE_V:
        return BAG_BULGE_H * (v / _BULGE_V) ** (2.0 / 3.0)
    return BAG_BULGE_H + (v - _BULGE_V) / BAG_AREA


def bag_thickness(z: float) -> float:
    """The filled bag's inside thickness at height z above its bottom seam."""
    return BAG_T * math.sqrt(min(max(z, 0.0), BAG_BULGE_H) / BAG_BULGE_H)


BAG_SHELL_KG = 0.070  # the empty bag and its 48 in inlet tubing
START_ML = 600.0  # 30 % of the bag: emptied at the end of a shift, well before it is half full
BAG_KG = BAG_SHELL_KG + START_ML * ML * 1000.0  # the bag's body mass (the liquid's weight is not changed as it drains)
BAG_SWING = (-0.02, 0.40)  # rad: the holder's frame stops it swinging back; it can swing out toward the robot
PORT = (HOOK[0], 0.0, HOOK[2] - STRAP - BAG_H)  # the outlet, at the middle of the bag's bottom (base frame)

# -- the drain's specification -------------------------------------------------------------------------------------------
# Every size of the drain tube, valve collar, spout and sleeve is set here, and everything else (the sleeve's colliders,
# the pad's height, the "in its sleeve" thresholds, the pull-out and push-in moves, the tests) is derived from these.
# Clear PVC drain tube 9 mm across, ending in a twist-valve collar 22 mm long and 12 mm across; past the collar a spout
# 15 mm long and 8 mm across. The sleeve is 30 mm deep from its mouth to its closed top. Fully home, the spout's tip
# reaches the closed top, so 15 mm of the collar is inside the sleeve and 7 mm shows below its mouth (15 + 15 = 30).
TUBE_R = 0.0045
LA = 0.085  # the tube's run out from the outlet before it folds up: the folded tube stands 6.25 cm proud of the bag's
# front, held there by the holder tab (room for the gripper's 10 cm palm beside the bag; a real tube folds closer)
LB = 0.110  # the rest of the tube
LV = 0.022  # the twist-valve collar's length
COLLAR_R = 0.006
SPOUT_LEN = 0.015  # the spout, past the collar
SPOUT_R = 0.004
TIP_L = LV + SPOUT_LEN  # the spout's tip from the collar's tube end, along the axis
SLEEVE_DEPTH = 0.030  # m from the sleeve's mouth to its closed top
COLLAR_HOME_IN = SLEEVE_DEPTH - SPOUT_LEN  # 15 mm of collar inside the sleeve when the spout is fully home ...
COLLAR_SHOWING = LV - COLLAR_HOME_IN  # ... and 7 mm showing below the mouth
TUBE_KG = (0.004, 0.008, 0.010, 0.010)  # sideways hinge, stub, tube, collar (raised from ~2 g for the solver)
STOW_A = math.pi / 2  # the stub folded out toward the robot, horizontal ...
STOW_B = math.pi / 2  # ... and the tube folded up in front of the bag: the spout in its sleeve, pointing up
TUBE_DAMP = 0.30  # N*m*s/rad on the tube's hinges: the tube's own internal damping (it resists speed, never position)
VALVE_OPEN = math.pi / 2  # rad: the collar's travel, closed stop to open stop (a quarter turn)
VALVE_OPEN_Q = -VALVE_OPEN  # the collar's joint at its open stop: it opens turning clockwise seen from the spout (the
# direction the right wrist can turn it at the draining pose)
VALVE_CLOSED_DEG = 10.0  # the valve counts as closed within 10 degrees of its closed stop (no flow there)
VALVE_DAMP = 0.01  # N*m*s/rad on the collar's turn
VALVE_FRICTION = 0.03  # N*m: the collar's seal friction, the joint's static friction effort (in this Isaac Lab
# version joint friction is an effort, N*m on a hinge; it has no preferred position). It stays where it is left: with
# damping alone (a sim run of an earlier version, task.py 31b81c30832f) the collar turned itself open while the tube
# was pulled out and swung up, then crept shut in ~28 s once let go. The valve hand's pinch (3 mm inside each side on
# the 5000 N/m fingers, ~15 N) can turn ~0.18 N*m, six times this.
OUTLET_R = 0.003  # m: the spout's 6 mm bore
OUTLET_A = math.pi * OUTLET_R ** 2
CD = lk.ORIFICE_CD  # 0.61, a sharp-edged orifice


class Spring:
    """A plain linear spring on a joint, toward `target` with stiffness `k` (and damping `damp`), everywhere: the
    sleeve's springy wall pad. No zone, no capture: it only presses on what is in the sleeve."""

    def __init__(self, target: float, k: float, damp: float):
        self.target, self.k, self.damp = target, k, damp

    def drive(self, key, q, qd, J, mem):
        return self.target, self.k, 0.0, self.damp


def _below(p, d) -> tuple:
    return (p[0], p[1], p[2] - d)


def _cyl(center, r, h, look) -> kit.Mesh:
    return kit.cylinder(center, r, h, look)


_HOOK_TO_BAG = STRAP + BAG_H / 2
_BAG_C = (0.0, 0.0, -_HOOK_TO_BAG)  # the bag body's middle, in the bag link's frame (origin at the hook)
_PORT_L = (0.0, 0.0, -(STRAP + BAG_H))  # the outlet in the bag link's frame
# -- the drain spout's sleeve ---------------------------------------------------------------------------------------------
# Drainage bags park the drain spout in a sleeve on the bag ("Put the drain spout back into its sleeve at the bottom of
# the bag", patient guides; Bard's patent US4417892: "The free end of the outlet is received in a protective housing
# heat sealed to one face of the urine drainage bag"). Modeled as a short sleeve on the holder tab in front of the bag,
# open at the bottom and closed at the top, that the folded-up tube's spout is pushed up into (sizes in the drain's
# specification above): 30 mm deep, a bore 1 mm wider than the collar, a flared mouth, and a springy pad in its
# bag-side wall that presses the collar against the far wall. Fully home the spout's tip is at the closed top, the spout
# is inside the upper 15 mm and the collar fills the lower 15 mm, its last 7 mm showing below the mouth. The spout
# stays in by friction alone (the pad's push and the walls' grip on the collar), plastic on plastic; nothing pulls it
# in, and the robot must push it all the way up (the scored "in its holder"). The trial bag's sleeve is checked against
# these sizes at setup.
SLEEVE_X = -LA  # the sleeve's axis in the bag link's frame: over the end of the tube's run, ...
SLEEVE_TOP = _PORT_L[2] + LB + TIP_L  # ... closed at the height of the stowed spout's tip
SLEEVE_MOUTH = SLEEVE_TOP - SLEEVE_DEPTH
SLEEVE_R = COLLAR_R + 0.0005  # its bore's radius: 13 mm across, 1 mm wider than the collar
SLEEVE_FLARE = (0.004, 0.0025)  # its lowest 4 mm flares out to the mouth, this much wider all round (18 mm across)
SLEEVE_WALL = 0.0015
SLEEVE_PANELS = 16
PLASTIC_MU = (0.30, 0.25)  # static, dynamic: dry PVC on plastic (kinetic 0.23-0.30 measured, arXiv:1902.03573)
PAD_FACE = 0.0045  # m from the axis: the pad's face at rest, clear of the 8 mm spout (the collar, against the far
# wall, pushes it back 1 mm)
PAD_SIZE = (0.003, 0.006, 0.010)  # thickness, width, height
PAD_LEAD = SLEEVE_R - PAD_FACE  # m: its top and bottom edges are chamfered back to the bore's wall over PAD_RAMP of
PAD_RAMP = 0.004  # its height (27 degrees off the wall), so the collar's shoulder coming up (or its end coming down)
# meets a slope, not a step, and pushes the pad back
PAD_Z = SLEEVE_MOUTH + (SLEEVE_FLARE[0] + COLLAR_HOME_IN) / 2  # its middle: on the collar's part in the straight bore
PAD_TRAVEL = 0.004  # m it can be pushed back into the wall
PAD_K, PAD_PRELOAD = 200.0, 0.006  # N/m and m: about 1.2 N pressing at rest (a light spring)
PAD_DAMP = 1.0  # N*s/m
PAD_KG = 0.005
_PAD_HALF = math.radians(25.0)  # the wall is open this far either side of the pad's direction (+x, toward the bag)


def _sleeve_colliders() -> list:
    """The sleeve's wall (convex panels round its axis, with a window for the pad), its flared mouth and its top, in the
    bag link's frame."""
    out, n = [], SLEEVE_PANELS
    k = 1.0 / math.cos(math.pi / n)
    z0, z1 = SLEEVE_MOUTH + SLEEVE_FLARE[0], SLEEVE_TOP  # the straight bore, above the flare
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        mid = (a0 + a1) / 2
        if abs(math.atan2(math.sin(mid), math.cos(mid))) < _PAD_HALF:
            continue  # the pad's window
        for (r_lo, r_hi, za, zb) in ((SLEEVE_R, SLEEVE_R, z0, z1),
                                     (SLEEVE_R + SLEEVE_FLARE[1], SLEEVE_R, z0 - SLEEVE_FLARE[0], z0)):
            pts = []
            for a in (a0, a1):
                for (r, z) in ((r_lo, za), (r_hi, zb), (r_hi + SLEEVE_WALL, zb), (r_lo + SLEEVE_WALL, za)):
                    pts.append((SLEEVE_X + r * k * math.cos(a), r * k * math.sin(a), z))
            pts = np.array([pts[0], pts[1], pts[5], pts[4], pts[3], pts[2], pts[6], pts[7]])
            c = pts.mean(0)
            out.append(kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in p - c) for p in pts),
                                    faces=_HEX_FACES, material=PLASTIC_MU))
    out.append(kit.Collider((SLEEVE_X, 0.0, SLEEVE_TOP + SLEEVE_WALL / 2), radius=SLEEVE_R + SLEEVE_WALL,
                            height=SLEEVE_WALL, material=PLASTIC_MU))  # its closed top
    return out


_HEX_FACES = ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2), (2, 6, 7),
              (2, 7, 3), (3, 7, 4), (3, 4, 0))
_SLEEVE_MESH = kit.lathe([(0.0, SLEEVE_TOP + SLEEVE_WALL), (SLEEVE_R + SLEEVE_WALL, SLEEVE_TOP + SLEEVE_WALL),
                          (SLEEVE_R + SLEEVE_WALL, SLEEVE_MOUTH + SLEEVE_FLARE[0]),
                          (SLEEVE_R + SLEEVE_FLARE[1] + SLEEVE_WALL, SLEEVE_MOUTH),
                          (SLEEVE_R + SLEEVE_FLARE[1], SLEEVE_MOUTH),
                          (SLEEVE_R, SLEEVE_MOUTH + SLEEVE_FLARE[0]), (SLEEVE_R, SLEEVE_TOP), (0.0, SLEEVE_TOP)],
                         kit.Look((0.93, 0.93, 0.92), 0.4), center=(SLEEVE_X, 0.0, 0.0), n=24)
BAG_LINK = kit.Link(
    "bag", "revolute", "Y", anchor=HOOK, lower=BAG_SWING[0], upper=BAG_SWING[1], start=0.0, mass=BAG_KG, damping=0.3,
    meshes=(kit.box((BAG_T / 2 - 0.0005, 0.0, _BAG_C[2]), (0.001, BAG_W, BAG_H), VINYL),  # back panel
            kit.box((-BAG_T / 2 + 0.0004, 0.0, _BAG_C[2] + 0.06), (0.0008, 0.10, 0.08), PRINT),  # front label
            kit.box((0.0, BAG_W / 2, _BAG_C[2]), (BAG_T, 0.004, BAG_H), VINYL_EDGE),  # side seams
            kit.box((0.0, -BAG_W / 2, _BAG_C[2]), (BAG_T, 0.004, BAG_H), VINYL_EDGE),
            kit.box((0.0, 0.0, -STRAP - 0.002), (BAG_T, BAG_W, 0.004), VINYL_EDGE),  # top seam
            kit.box((0.0, 0.0, -STRAP - BAG_H + 0.002), (BAG_T, BAG_W, 0.004), VINYL_EDGE),  # bottom seam
            kit.box((0.0, 0.0, -STRAP / 2), (0.003, 0.012, STRAP), VINYL_EDGE),  # the hanger
            _cyl(_below(_PORT_L, -0.004), 0.006, 0.008, TUBE),  # the outlet boss
            # the holder tab out from the bag's front, carrying the spout's sleeve at its end
            kit.box((-(BAG_T / 2 + LA - SLEEVE_R) / 2, 0.0, SLEEVE_TOP - SLEEVE_DEPTH / 2),
                    (LA - SLEEVE_R - BAG_T / 2, 0.010, 0.003), VINYL_EDGE),
            _SLEEVE_MESH),
    colliders=(kit.Collider(_BAG_C, size=(BAG_T, BAG_W, BAG_H)),
               *([] if os.environ.get("DRAINBAG_NO_SLEEVE") else _sleeve_colliders())))
# DRAINBAG_NO_SLEEVE=1 (a check run only): the sleeve and its pad without colliders, to show that only their contact
# holds the spout up
# The tube's hinges carry only damping: nothing pulls the tube toward any pose; in its sleeve the sleeve holds it.
TUBE_X = kit.Link("tube_x", "revolute", "X", anchor=PORT, lower=-0.5, upper=0.5, start=0.0, mass=TUBE_KG[0],
                  parent="bag", damping=TUBE_DAMP)
TUBE_A = kit.Link("tube_a", "revolute", "Y", anchor=PORT, lower=-0.6, upper=2.6, start=STOW_A, mass=TUBE_KG[1],
                  parent="tube_x", damping=TUBE_DAMP,
                  meshes=(_cyl((0.0, 0.0, -LA / 2), TUBE_R, LA, TUBE),),
                  colliders=(kit.Collider((0.0, 0.0, -LA / 2), radius=TUBE_R, height=LA),))
TUBE_B = kit.Link("tube_b", "revolute", "Y", anchor=_below(PORT, LA), lower=-2.6, upper=2.6, start=STOW_B,
                  mass=TUBE_KG[2], parent="tube_a", damping=TUBE_DAMP,
                  meshes=(_cyl((0.0, 0.0, -LB / 2), TUBE_R, LB, TUBE),),
                  # the tube end to end, and its knee where the run folds up (a sphere: no gap on the bend's outside)
                  colliders=(kit.Collider((0.0, 0.0, -LB / 2), radius=TUBE_R, height=LB),
                             kit.Collider((0.0, 0.0, 0.0), radius=TUBE_R, sphere=True)))
VALVE_LINK = kit.Link("valve", "revolute", "Z", anchor=_below(PORT, LA + LB), lower=min(0.0, VALVE_OPEN_Q),
                      upper=max(0.0, VALVE_OPEN_Q), start=0.0,
                      mass=TUBE_KG[3], parent="tube_b", damping=VALVE_DAMP, friction=VALVE_FRICTION,
                      meshes=(_cyl((0.0, 0.0, -LV / 2), COLLAR_R, LV, VALVE), _cyl((0.0, 0.0, -LV - SPOUT_LEN / 2),
                                                                                    SPOUT_R, SPOUT_LEN, VALVE)),
                      colliders=(kit.Collider((0.0, 0.0, -LV / 2), radius=COLLAR_R, height=LV),
                                 kit.Collider((0.0, 0.0, -LV - SPOUT_LEN / 2), radius=SPOUT_R, height=SPOUT_LEN)))


def _pad_collider():
    """The pad: a block in the sleeve's bag-side wall, its face toward the axis, chamfered top and bottom (a 6-sided
    prism across y), in the pad link's frame."""
    x0, x1, h, w = PAD_FACE, PAD_FACE + PAD_SIZE[0], PAD_SIZE[2] / 2, PAD_SIZE[1] / 2
    prof = [(x1, -h), (x0 + PAD_LEAD, -h), (x0, -h + PAD_RAMP), (x0, h - PAD_RAMP), (x0 + PAD_LEAD, h), (x1, h)]
    pts = [(x, y, z) for y in (-w, w) for (x, z) in prof]
    n = len(prof)
    faces = [(0, k, k + 1) for k in range(1, n - 1)] + [(n, n + k + 1, n + k) for k in range(1, n - 1)]
    for k in range(n):
        a, b = k, (k + 1) % n
        faces += [(a, b, n + b), (a, n + b, n + a)]
    c = np.mean(pts, axis=0)
    return kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in np.subtract(p, c)) for p in pts),
                        faces=tuple(faces), material=PLASTIC_MU)


# the sleeve's springy pad: a slide in its bag-side wall, pushed toward the axis by a light spring, against a stop
PAD_LINK = kit.Link("pad", "prismatic", "X", anchor=(HOOK[0] + SLEEVE_X, 0.0, HOOK[2] + PAD_Z), lower=0.0,
                    upper=PAD_TRAVEL, start=COLLAR_R - PAD_FACE, mass=PAD_KG, parent="bag", damping=PAD_DAMP,
                    meshes=(kit.box((PAD_FACE + PAD_SIZE[0] / 2, 0.0, 0.0), PAD_SIZE, kit.Look((0.93, 0.93, 0.92), 0.4)),),
                    colliders=() if os.environ.get("DRAINBAG_NO_SLEEVE") else
                    (_pad_collider(),),
                    law=Spring(-PAD_PRELOAD, PAD_K, PAD_DAMP))
_STAND_MESHES = (kit.box((0.0, 0.0, 0.004), (0.210, 0.220, 0.008), STEEL),  # the base frame on the floor
                 _cyl((0.020, 0.090, 0.245), 0.005, 0.490, STEEL), _cyl((0.020, -0.090, 0.245), 0.005, 0.490, STEEL),
                 kit.box((0.020, 0.0, 0.488), (0.010, 0.190, 0.010), STEEL),  # the top bar
                 kit.box(((HOOK[0] + 0.020) / 2, 0.0, 0.486), (0.050, 0.006, 0.006), STEEL))  # the hook arm
BAG = kit.Mechanism(
    "drainbag", pos=STAND_POS, size=(0.21, 0.22, 0.50), meshes=_STAND_MESHES,
    colliders=(kit.Collider((0.0, 0.0, 0.004), size=(0.210, 0.220, 0.008)),
               kit.Collider((0.020, 0.0, 0.245), size=(0.010, 0.190, 0.490))),
    links=(BAG_LINK, TUBE_X, TUBE_A, TUBE_B, VALVE_LINK, PAD_LINK),
    self_collide=True, no_collide=(("bag", "tube_a"),),  # the run starts inside the bag's outlet
    # every other collider grips with the competition environment's own object material, bound in the USD so the
    # sleeve's plastic keeps its own (the spawner would otherwise bind the scene's material over all of them)
    material=(GRIPPY_FRICTION[0], GRIPPY_FRICTION[1], "average"), contact_offset=0.002)

# -- the graduate --------------------------------------------------------------------------------------------------------
# Eisco CH0357D 1000 mL short-form measuring jug, transparent polypropylene, with a handle and a pour spout (maker's
# listing: 5.75 in tall, 5 in across the top, 4.4 in across the bottom, 6.75 in from spout to handle edge). Modeled as a
# frustum of those outside sizes with a 2 mm wall and a 3 mm floor, its handle on the robot's right; the small pour
# spout is left out. The wall, floor and mass are model values, checked against the trial jug at setup.
GR_B, GR_T, GR_H = 0.0539, 0.0615, 0.143  # inside radius at the floor and at the rim, inside depth
GR_FLOOR, GR_WALL = 0.003, 0.002
GR_KG = 0.136
GR_RIM_Z = GR_FLOOR + GR_H  # the rim's height above the floor
GR_START = (0.230, -0.258)  # on the robot's right beside the underpad, its handle toward the right
GR_SIDE = -1.0  # the handle sticks out toward -y
HANDLE_ARM = (0.012, 0.040, 0.008)  # the handle's top arm (x across it, y out from the rim, z): 4 cm out (the
# maker's 6.75 in from spout to handle edge less its 5 in top); the handle's sizes are checked at setup
HANDLE_ARM_C = (0.0, GR_SIDE * (GR_T + HANDLE_ARM[1] / 2 - 0.002), GR_RIM_Z - 0.010)
HANDLE_BAR = (0.012, 0.008, 0.090)
HANDLE_BAR_C = (0.0, GR_SIDE * (GR_T + HANDLE_ARM[1] - 0.006), GR_RIM_Z - 0.010 - HANDLE_BAR[2] / 2)
HANDLE_GRIP = (0.0, GR_SIDE * (GR_T + 0.024), HANDLE_ARM_C[2])  # where the hand takes it: the top arm, 2 cm out
# from the rim's outside, so the fingers stay clear of the rim (test_task.py checks the arms against it)


GR_PANELS = 48  # the flared wall's colliders: one convex panel per 7.5 degrees, as the drawn wall (lathe, n=48)
# The graduate's wall in its (radius, height) half-plane: inside from (GR_B, floor) up to (GR_T, rim), GR_WALL thick.
# Each panel's inside face is a flat chord whose middle lies on that line (its corners stand 0.14 mm further out at the
# rim), so the colliders are the drawn wall to within 0.14 mm.
WALL_SECTION = ((GR_B, GR_FLOOR), (GR_T, GR_RIM_Z), (GR_T + GR_WALL, GR_RIM_Z), (GR_B + GR_WALL, GR_FLOOR))
FLOOR_SECTION = ((0.0, 0.0), (GR_B + GR_WALL, 0.0), (GR_B + GR_WALL, GR_FLOOR), (0.0, GR_FLOOR))


def wall_panels(n: int = GR_PANELS) -> list:
    """The flared wall as n convex hull colliders (8 corners each), joined edge to edge round the axis."""
    out, k = [], 1.0 / math.cos(math.pi / n)
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        pts = []
        for (r, z) in WALL_SECTION:  # each section corner at both edge angles of the panel
            pts.append((r * k * math.cos(a0), r * k * math.sin(a0), z))
        for (r, z) in WALL_SECTION:
            pts.append((r * k * math.cos(a1), r * k * math.sin(a1), z))
        pts = np.array([pts[0], pts[1], pts[5], pts[4], pts[3], pts[2], pts[6], pts[7]])
        c = pts.mean(0)
        out.append(kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in p - c) for p in pts),
                                faces=_HEX_FACES))
    return out


GRADUATE = kit.Prop(
    "graduate", pos=(GR_START[0], GR_START[1], FLOOR), mass=GR_KG,
    size=(2 * (GR_T + 0.008), 2 * (GR_T + HANDLE_ARM[1]), GR_RIM_Z),
    meshes=(kit.lathe([(0.0, 0.0), (GR_B + GR_WALL, 0.0), (GR_T + GR_WALL, GR_RIM_Z), (GR_T, GR_RIM_Z),
                       (GR_B, GR_FLOOR), (0.0, GR_FLOOR)], PP, n=GR_PANELS),
            kit.box(HANDLE_ARM_C, HANDLE_ARM, PP), kit.box(HANDLE_BAR_C, HANDLE_BAR, PP)),
    colliders=(kit.Collider((0.0, 0.0, GR_FLOOR / 2), radius=GR_B + GR_WALL, height=GR_FLOOR),
               kit.Collider(HANDLE_ARM_C, size=HANDLE_ARM), kit.Collider(HANDLE_BAR_C, size=HANDLE_BAR))
    + tuple(wall_panels()))

# Medline disposable underpad, 23 x 36 in, folded in half both ways: 29 x 46 cm, 1 mm, on the floor
UNDERPAD = Box("underpad", pos=(0.285, BAG_Y + 0.085, FLOOR + 0.0006), size=(0.29, 0.46, 0.001), color=PAD_RGB)


PAN_WALL = ((PAN_OUT[0] - PAN_IN[0]) / 2, (PAN_OUT[1] - PAN_IN[1]) / 2)  # the band each wall and its lip take, x and y


def pan_inside() -> tuple:
    """(x0, y0, x1, y1) of the pan's inside floor."""
    return (PAN_X0 + PAN_WALL[0], PAN_Y0 + PAN_WALL[1], PAN_X0 + PAN_OUT[0] - PAN_WALL[0],
            PAN_Y0 + PAN_OUT[1] - PAN_WALL[1])


def _pan() -> list:
    """The drip pan: its floor and four walls (fixed)."""
    x0, y0, x1, y1 = PAN_X0, PAN_Y0, PAN_X0 + PAN_OUT[0], PAN_Y0 + PAN_OUT[1]
    cx, cy, zc = (x0 + x1) / 2, (y0 + y1) / 2, TOP + PAN_H / 2
    wx, wy = PAN_WALL
    return [Box("pan_floor", pos=(cx, cy, TOP + PAN_T / 2), size=(PAN_OUT[0], PAN_OUT[1], PAN_T), color=PAN_RGB),
            Box("pan_near", pos=(x0 + wx / 2, cy, zc), size=(wx, PAN_OUT[1], PAN_H), color=PAN_RGB),
            Box("pan_far", pos=(x1 - wx / 2, cy, zc), size=(wx, PAN_OUT[1], PAN_H), color=PAN_RGB),
            Box("pan_right", pos=(cx, y0 + wy / 2, zc), size=(PAN_OUT[0] - 2 * wx, wy, PAN_H), color=PAN_RGB),
            Box("pan_left", pos=(cx, y1 - wy / 2, zc), size=(PAN_OUT[0] - 2 * wx, wy, PAN_H), color=PAN_RGB)]


PAN = _pan()
OBJECTS = [*PAN, UNDERPAD, BAG, GRADUATE]

# -- the scored thresholds -----------------------------------------------------------------------------------------------
FULL_ML = 570.0  # in the graduate for full credit: 95 % of the 600 mL (the bag drains completely when open and aimed)
HALF_ML = 300.0  # half the bag
TRACE_ML = 100.0
CLEAN_ML = 5.0  # mL spilled for full credit: a few splashes on the pad
SPILL_MAX_ML = 20.0  # mL: more spilled scores 0 (a puddle on the floor)
TOUCH_TOL = 0.0005  # m: any point of the drain tube's surface within 0.5 mm of the graduate is a touch
HAND_HIT_S = 1.0  # s: a stream running into a hand or arm for longer than this in all scores 0
MOUTH_MARGIN = 0.002  # m: a stream landing this close to the rim is taken to hit it and splash out
UPRIGHT_DEG = 10.0  # the graduate's axis within 10 degrees of vertical at the end
RELEASE_MARGIN = 0.010  # m: every tool point and pad tip this far from the spout, the tube and the graduate
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
TOUCH_CAP = 0.25  # a touch caps the trial at 0.25
GRIPPERS = ("left_gripper", "right_gripper")


# -- geometry ------------------------------------------------------------------------------------------------------------
def rot_axis(axis: str, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    if axis == "X":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "Y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def axis_rot(u, a: float) -> np.ndarray:
    """Rotation by a about unit axis u (Rodrigues)."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def fk(swing=0.0, side=0.0, a=STOW_A, b=STOW_B, v=0.0) -> dict:
    """World pose (origin, rotation) of every link's frame for these joint angles: the bag on its hook, the sideways
    hinge, the stub, the tube and the valve collar; plus 'tip' (the spout's end, a point) and 'collar' (its middle)."""
    base = np.asarray(STAND_POS, float)
    hook = base + np.asarray(HOOK, float)
    Rb = rot_axis("Y", swing)
    port = hook + Rb @ (np.asarray(PORT, float) - np.asarray(HOOK, float))
    Rx = Rb @ rot_axis("X", side)
    Ra = Rx @ rot_axis("Y", a)
    pb = port + Ra @ np.array([0.0, 0.0, -LA])
    Rbt = Ra @ rot_axis("Y", b)
    pv = pb + Rbt @ np.array([0.0, 0.0, -LB])
    Rv = Rbt @ rot_axis("Z", v)
    return {"bag": (hook, Rb), "tube_x": (port, Rx), "tube_a": (port, Ra), "tube_b": (pb, Rbt), "valve": (pv, Rv),
            "tip": pv + Rv @ np.array([0.0, 0.0, -TIP_L]), "collar": pv + Rv @ np.array([0.0, 0.0, -LV / 2])}


def _R(q) -> np.ndarray:
    return lk.rot(q)


def _state_pose(objects, key) -> tuple:
    s = objects[key]
    return np.asarray(s.pos, float), _R(np.asarray(s.quat_xyzw, float))


def spout(objects) -> tuple:
    """(spout end, its outward axis, the valve frame's origin, rotation) from the valve collar's state."""
    p, R = _state_pose(objects, "drainbag/valve")
    d = R @ np.array([0.0, 0.0, -1.0])
    return p + TIP_L * d, d, p, R


def bag_level(objects, volume_m3: float) -> float:
    """World height of the urine's surface in the bag: the bottom's middle plus the lumped depth (the bag hangs within
    a few degrees of vertical)."""
    p, R = _state_pose(objects, "drainbag/bag")
    floor = p + R @ np.asarray(_PORT_L, float)
    return float(floor[2]) + bag_depth(volume_m3)


def joint_angle(key: str, default: float) -> float:
    try:
        return float(kit.joint(None, f"drainbag/{key}").pos)
    except KeyError:
        return default


def valve_open_fraction(v: float) -> float:
    """How far the valve is open: 0 within VALVE_CLOSED_DEG of its closed stop, 1 at its open stop."""
    lo = math.radians(VALVE_CLOSED_DEG)
    return float(min(max((v / VALVE_OPEN_Q * VALVE_OPEN - lo) / (VALVE_OPEN - lo), 0.0), 1.0))


IN_SLEEVE_TOP = 0.004  # m: "fully home" is the spout's tip within this of the sleeve's closed top ...
COLLAR_MIN_IN = COLLAR_HOME_IN - IN_SLEEVE_TOP  # ... with at least this much collar inside the sleeve (11 of its
# 22 mm; at most 11 mm showing below the mouth)
COAXIAL = SLEEVE_R - COLLAR_R + 0.0005  # m: the collar's axis within this of the sleeve's, where it is inside


def holder_readout(objects) -> tuple:
    """(spout tip's height below the sleeve's closed top, mm of collar inside the sleeve, the collar's axis' largest
    offset from the sleeve's axis over its inside part, the collar pointing up) in the bag link's frame."""
    bp, bR = _state_pose(objects, "drainbag/bag")
    vp, vR = _state_pose(objects, "drainbag/valve")

    def at(z):  # a point on the valve's axis, z along it (0 the collar's tube end, -LV its spout end, -TIP_L the tip)
        return bR.T @ (vp + vR @ np.array([0.0, 0.0, z]) - bp)

    end, shoulder, tip = at(0.0), at(-LV), at(-TIP_L)
    up = float(shoulder[2] - end[2]) / LV  # the axis' rise per length, 1 straight up
    inside = min(max(float(shoulder[2]) - SLEEVE_MOUTH, 0.0), LV * max(up, 0.0)) / max(up, 1e-9)  # along the axis
    mouth = at(-LV + inside)  # where the collar passes the mouth (its shoulder if it is not in)
    radial = max(math.hypot(float(e[0]) - SLEEVE_X, float(e[1])) for e in (shoulder, mouth))
    return SLEEVE_TOP - float(tip[2]), inside, radial, up > 0.0


def in_holder(objects) -> bool:
    """The spout is in its sleeve, fully home: its tip within IN_SLEEVE_TOP of the sleeve's closed top (and not past
    it), at least COLLAR_MIN_IN of the valve collar inside the sleeve, and the collar coaxial in the bore over its
    inside part, pointing up. Derived from the drain's specification: fully home the tip is at the top with
    COLLAR_HOME_IN of collar inside. A geometric readout of where the collar is; what keeps it there is the sleeve's
    contact and friction."""
    below_top, inside, radial, up = holder_readout(objects)
    return bool(up and -0.0005 <= below_top <= IN_SLEEVE_TOP and inside >= COLLAR_MIN_IN - 1e-9 and radial <= COAXIAL)


def graduate_frame(objects) -> tuple:
    """(origin at the inside floor's middle, rotation) of the graduate's liquidkit frame."""
    p, R = _state_pose(objects, "graduate")
    return p + R @ np.array([0.0, 0.0, GR_FLOOR]), R


def _polygon_distance(r, z, poly) -> np.ndarray:
    """Signed distance (negative inside) from points (r, z) to a convex polygon in the (radius, height) half-plane."""
    P = np.stack([r, z], -1)
    V = np.asarray(poly, float)
    best = np.full(len(P), np.inf)
    inside = np.ones(len(P), bool)
    area = 0.0
    for i in range(len(V)):
        a, b = V[i], V[(i + 1) % len(V)]
        area += a[0] * b[1] - b[0] * a[1]
    s = 1.0 if area > 0 else -1.0  # counter-clockwise: inside is to the left of every edge
    for i in range(len(V)):
        a, b = V[i], V[(i + 1) % len(V)]
        e = b - a
        t = np.clip(((P - a) @ e) / float(e @ e), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(P - (a + t[:, None] * e), axis=1))
        inside &= s * (e[0] * (P[:, 1] - a[1]) - e[1] * (P[:, 0] - a[0])) >= 0.0
    return np.where(inside, -best, best)


def _box_distance(local, center, size) -> np.ndarray:
    """Signed distance from points (graduate frame) to an axis-aligned box of the graduate (its handle)."""
    q = np.abs(np.asarray(local, float) - np.asarray(center, float)) - np.asarray(size, float) / 2
    out = np.linalg.norm(np.maximum(q, 0.0), axis=1)
    return out + np.minimum(q.max(1), 0.0)


def graduate_distance(local) -> np.ndarray:
    """Signed distance from points in the graduate's prop frame (origin at the middle of its base) to its colliders:
    the flared wall with its rim, the floor, and the handle. The wall and floor are solids of revolution, so the
    distance is the one in the (radius, height) half-plane."""
    P = np.atleast_2d(np.asarray(local, float))
    rho = np.hypot(P[:, 0], P[:, 1])
    d = np.minimum(_polygon_distance(rho, P[:, 2], WALL_SECTION), _polygon_distance(rho, P[:, 2], FLOOR_SECTION))
    for c, s in ((HANDLE_ARM_C, HANDLE_ARM), (HANDLE_BAR_C, HANDLE_BAR)):
        d = np.minimum(d, _box_distance(P, c, s))
    return d


# The real trial's touch check: the graduate's rim (its top edge), inside walls and floor carry a thin coat of lipstick,
# and the drain tube is wiped with a white towel after the trial (any red on it is a touch). The outside wall and the
# handle stay bare, as the hands carry the graduate there. So the scored touch is the tube on that coated surface, here
# its (radius, height) outline: the inside wall, the rim's top and the inside floor.
COATED = (((GR_B, GR_FLOOR), (GR_T, GR_RIM_Z)), ((GR_T, GR_RIM_Z), (GR_T + GR_WALL, GR_RIM_Z)),
          ((0.0, GR_FLOOR), (GR_B, GR_FLOOR)))
# The jug's small pour spout is left out of the model: the real trial coats its lip with the rim, and the sim counts
# the round rim there as coated rim (a tube resting where the spout is touches the rim in both).


def coated_distance(local) -> np.ndarray:
    """Distance from points in the graduate's prop frame to its coated surface (rim top, inside wall, inside floor)."""
    P = np.atleast_2d(np.asarray(local, float))
    rho, z = np.hypot(P[:, 0], P[:, 1]), P[:, 2]
    best = np.full(len(P), np.inf)
    for (r0, z0), (r1, z1) in COATED:
        dr, dz = r1 - r0, z1 - z0
        t = np.clip(((rho - r0) * dr + (z - z0) * dz) / (dr * dr + dz * dz), 0.0, 1.0)
        best = np.minimum(best, np.hypot(rho - r0 - t * dr, z - z0 - t * dz))
    return best


def _coated_samples(step=0.001) -> np.ndarray:
    """Points over the coated surface (prop frame), about 1 mm apart, for the hands' contact with it."""
    out = []
    for z in np.arange(GR_FLOOR, GR_RIM_Z + 1e-9, step):  # the inside wall
        r = GR_B + (GR_T - GR_B) * (z - GR_FLOOR) / GR_H
        out.append(_ring(r, z, int(math.ceil(2 * math.pi * r / step))))
    for r in np.arange(GR_T, GR_T + GR_WALL + 1e-9, step / 2):  # the rim's top
        out.append(_ring(r, GR_RIM_Z, int(math.ceil(2 * math.pi * r / step))))
    for r in np.arange(0.0, GR_B, step):  # the inside floor
        out.append(_ring(r, GR_FLOOR, max(int(math.ceil(2 * math.pi * r / step)), 1)))
    return np.concatenate(out)


def _ring(r, z, n):
    a = np.linspace(0.0, 2 * math.pi, n, endpoint=False)
    return np.stack([r * np.cos(a), r * np.sin(a), np.full(n, z)], 1)


def _cylinder_samples(r, z0, z1, n_around, caps=True) -> np.ndarray:
    """Points on a cylinder's side (radius r, from z0 to z1 along its axis) and, with caps, on its two end discs."""
    out = [_ring(r, z, n_around) for z in np.linspace(z0, z1, int(math.ceil(abs(z1 - z0) / 0.0004)) + 1)]
    if caps:
        for z in (z0, z1):
            for rr in np.arange(0.0, r, 0.0005):
                out.append(_ring(rr, z, max(int(math.ceil(2 * math.pi * rr / 0.0005)), 1)))
    return np.concatenate(out)


def _sphere_samples(r, spacing=0.0004) -> np.ndarray:
    """Points evenly over a sphere of radius r (a Fibonacci lattice)."""
    n = int(math.ceil(4 * math.pi * r * r / (spacing * spacing)))
    k = np.arange(n) + 0.5
    z = 1.0 - 2.0 * k / n
    a = math.pi * (1.0 + math.sqrt(5.0)) * k
    rr = np.sqrt(1.0 - z * z)
    return r * np.stack([rr * np.cos(a), rr * np.sin(a), z], 1)


def _surface_samples() -> dict:
    """Points on the whole drain tube's surface, each in its own link's frame, on that link's colliders: the run out of
    the bag (tube_a), the folded-up tube with its knee (tube_b), and the valve collar with the spout past it. Every
    point of that surface lies within 0.4 mm of a sample (test_task.py), so a real contact reads as a gap under 0.4 mm,
    inside the 0.5 mm touch band."""
    return {"drainbag/tube_a": _cylinder_samples(TUBE_R, 0.0, -LA, 48),
            "drainbag/tube_b": np.concatenate([_cylinder_samples(TUBE_R, 0.0, -LB, 48), _sphere_samples(TUBE_R)]),
            "drainbag/valve": np.concatenate([_cylinder_samples(COLLAR_R, 0.0, -LV, 64),
                                              _cylinder_samples(SPOUT_R, -LV, -TIP_L, 48)])}


TUBE_SAMPLES = _surface_samples()
NEAR_RHO, NEAR_Z = GR_T + 0.04, GR_RIM_Z  # a box round the graduate (with its handle) in its frame: samples outside it
# by more than the touch band cannot be touching


def tube_surface(objects) -> np.ndarray:
    """The drain tube's surface samples in the world frame (from each link's state)."""
    out = []
    for key, pts in TUBE_SAMPLES.items():
        p, R = _state_pose(objects, key)
        out.append(pts @ R.T + p)
    return np.concatenate(out)


def tube_local(objects) -> np.ndarray:
    """The drain tube's surface samples in the graduate's prop frame."""
    p, R = _state_pose(objects, "graduate")
    return (tube_surface(objects) - p) @ R


def _outside_bound(local) -> np.ndarray:
    """A lower bound on each point's distance to the graduate (its distance to a box that holds it)."""
    rho = np.hypot(local[:, 0], local[:, 1])
    dz = np.maximum(np.maximum(local[:, 2] - NEAR_Z, -local[:, 2]), 0.0)
    return np.hypot(np.maximum(rho - NEAR_RHO, 0.0), dz)


def tube_gap(objects) -> float:
    """Smallest gap (m) between the drain tube's surface and the graduate's coated surface (rim top, inside wall,
    inside floor)."""
    local = tube_local(objects)
    near = _outside_bound(local) <= 0.01
    return float(coated_distance(local[near]).min()) if near.any() else float(_outside_bound(local).min())


def touches(objects) -> bool:
    """The drain tube's surface within TOUCH_TOL of the graduate's rim, inside wall or inside floor."""
    return tube_gap(objects) <= TOUCH_TOL


COATED_SAMPLES = None  # filled on first use (_coated_samples)


def hull_contacts(hulls, pts, tol=TOUCH_TOL) -> set:
    """Names of the robot hulls within tol of any of the points (world frame), by the hull's own face planes."""
    out = set()
    if not hulls or len(pts) == 0:
        return out
    lo, hi = pts.min(0) - tol, pts.max(0) + tol
    for name, h in hulls:
        v = h.vertices
        if np.any(v.max(0) < lo) or np.any(v.min(0) > hi):
            continue
        near = np.all((pts >= v.min(0) - tol) & (pts <= v.max(0) + tol), axis=1)
        if near.any() and float((pts[near] @ h.planes[:, :3].T - h.planes[:, 3]).max(1).min()) <= tol:
            out.add(name)
    return out


def touched_between(prev_local, now_local) -> bool:
    """Whether the drain tube came within TOUCH_TOL of the graduate's coated surface while moving from one control step
    to the next (both in the graduate's frame; the relative motion is taken as straight over the step). A gap can
    shrink no faster than the samples move, so only samples that come close are checked, and only steps that come
    close are subdivided."""
    move = np.linalg.norm(now_local - prev_local, axis=1)
    keep = np.minimum(_outside_bound(prev_local), _outside_bound(now_local)) - move / 2 <= TOUCH_TOL
    if not keep.any():
        return False
    a, b, move = prev_local[keep], now_local[keep], move[keep]
    d0, d1 = coated_distance(a), coated_distance(b)
    if min(d0.min(), d1.min()) <= TOUCH_TOL:
        return True
    near = np.minimum(d0, d1) - move / 2 <= TOUCH_TOL
    if not near.any():
        return False
    a, b = a[near], b[near]
    n = int(min(math.ceil(float(move[near].max()) / 0.0002), 400))
    for k in range(1, n):
        if coated_distance(a + (b - a) * (k / n)).min() <= TOUCH_TOL:
            return True
    return False


def graduate_depth(volume_m3: float) -> float:
    """Depth of this much water in the upright graduate (the frustum's exact volume solved for the height), for the
    drawing: the drawn level rises smoothly, not in the liquidkit sample layers' 5.6 mm steps."""
    def vol(h):
        r = GR_B + (GR_T - GR_B) * h / GR_H
        return math.pi * h / 3.0 * (GR_B * GR_B + GR_B * r + r * r)
    v = max(float(volume_m3), 0.0)
    if v >= vol(GR_H):
        return GR_H
    lo, hi = 0.0, GR_H
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if vol(mid) < v else (lo, mid)
    return (lo + hi) / 2


def on_underpad(objects) -> bool:
    """The graduate stands on the barrier: the middle of its base over the underpad."""
    p, _R = _state_pose(objects, "graduate")
    return (abs(float(p[0]) - UNDERPAD.pos[0]) <= UNDERPAD.size[0] / 2
            and abs(float(p[1]) - UNDERPAD.pos[1]) <= UNDERPAD.size[1] / 2)


def tilt_deg(R) -> float:
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))


def arc_points(p0, v0, z_end: float, n: int = 14) -> list:
    """Points along the ballistic stream from p0 (velocity v0) down to height z_end."""
    p0, v0 = np.asarray(p0, float), np.asarray(v0, float)
    a, b, c = -G / 2, float(v0[2]), float(p0[2]) - z_end
    disc = b * b - 4 * a * c
    t_end = (-b - math.sqrt(disc)) / (2 * a) if disc >= 0 else 0.0
    ts = np.linspace(0.0, max(t_end, 0.0), n + 1)
    return [p0 + v0 * t + np.array([0.0, 0.0, -G / 2 * t * t]) for t in ts]


def land(p0, v0, z: float):
    """Where a stream leaving p0 with velocity v0 crosses height z on its way down (None if it never does)."""
    a, b, c = -G / 2, float(v0[2]), float(p0[2]) - z
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / (2 * a)
    if t < 0:
        return None
    return np.asarray(p0, float)[:2] + np.asarray(v0, float)[:2] * t


# -- one trial's record --------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.bag = START_ML * ML  # m^3 in the bag
        self.world = lk.World(vessels={"graduate": lk.Vessel("graduate", lk.frustum(GR_B, GR_T, GR_H))}, table_z=FLOOR)
        self.stream_spill = 0.0  # m^3 that missed the graduate
        self.hand_ml = 0.0  # mL of it that ran into an arm (part of stream_spill)
        self.hand_steps = 0  # control steps in which the stream ran into a hand or arm
        self.hand_check_off = False  # the arms' hulls were missing at some step in the sim
        self.red = set()  # robot links that touched the graduate's coated surface (they carry its lipstick)
        self.carried = False  # a touch made by such a link touching the drain tube
        self.hulls, self.in_sim = None, False  # the arms' hulls this step, and whether this is the simulator
        self.local = None  # the spout's surface samples in the graduate's frame at the last step (the touch sweep)
        self.stream = None  # the stream's arc (points from the spout to where it lands) while it flows, for the drawing
        self.stream_frac = 0.0  # how far the valve was open for that stream (the drawn stream's thickness)
        self.stream_caught = False  # whether that stream went into the graduate's mouth
        self.touched = False
        self.good = 0
        self.last = None
        self.a, self.b, self.v = STOW_A, STOW_B, 0.0

    def graduate_ml(self) -> float:
        return self.world.vessels["graduate"].volume / ML

    def spilled_ml(self) -> float:
        return (self.stream_spill + self.world.spilled) / ML

    def flow(self, objects, dt: float, hulls=None) -> None:
        """The drain for one control step: orifice flow from the bag's level down to the spout, then the stream's
        landing (in the graduate's mouth, else on the floor; a stream that runs into an arm first splashes off it onto
        the floor), then the graduate's own liquid (slosh, and a spill over its rim if it is tipped or jolted)."""
        self.stream = None
        frac = valve_open_fraction(self.v)
        tip, d, _pv, _Rv = spout(objects)
        head = bag_level(objects, self.bag) - float(tip[2])
        o, R = graduate_frame(objects)
        if frac > 0.0 and head > 0.0 and self.bag > 0.0:
            speed = math.sqrt(2 * G * head)
            q = min(self.bag, CD * OUTLET_A * frac * speed * dt)
            self.bag -= q
            mouth = o + R @ np.array([0.0, 0.0, GR_H])
            caught = False
            if tilt_deg(R) <= 30.0:
                if float((tip - mouth) @ R[:, 2]) <= 0.0:  # the spout below the mouth's plane: inside it or outside it
                    loc = R.T @ (tip - o)
                    inside = 0.0 <= float(loc[2]) <= GR_H and math.hypot(loc[0], loc[1]) <= GR_T - MOUTH_MARGIN
                    up = land(tip, speed * d, float(mouth[2]))  # a jet thrown up out of the mouth comes down here
                    caught = inside and (up is None or float(np.linalg.norm(up - mouth[:2])) <= GR_T - MOUTH_MARGIN)
                else:  # where the arc crosses the mouth's own plane (tilted with the graduate) on its way down
                    hit = lk.ballistic(tip, speed * d, mouth, R[:, 2])
                    if hit is not None:
                        loc = R.T @ (hit[1] - o)
                        caught = math.hypot(float(loc[0]), float(loc[1])) <= GR_T - MOUTH_MARGIN
            if caught:  # the arc down to the graduate's surface, else to the floor
                z_end = float((o + R @ np.array([0.0, 0.0, graduate_depth(self.world.vessels["graduate"].volume)]))[2])
            else:
                z_end = FLOOR
            z_end = min(z_end, float(tip[2]))
            path = arc_points(tip, speed * d, z_end, n=STREAM_CHECK_POINTS)
            # checked from one collar radius out of the mouth: fingers holding the collar beside its mouth are not in
            # the water's way (it leaves through the bore), a hand in front of the mouth is
            free = [k for k, pt in enumerate(path) if float(np.linalg.norm(pt - tip)) >= COLLAR_R]
            blocked = None
            if hulls and free:
                blocked = lk.stream_blocked(path[free[0]:], hulls, lk.math_jet_radius(q / dt, speed))
            if blocked is not None:  # it runs into a hand or arm: it splashes off onto the floor
                caught = False
                path = path[:free[0] + max(blocked[0], 1) + 1]
                self.hand_ml += q / ML
                self.hand_steps += 1
            if caught:
                g = self.world.vessels["graduate"]
                room = max(g.capacity - g.volume, 0.0)
                g.volume += min(q, room)
                self.stream_spill += max(q - room, 0.0)
            else:
                self.stream_spill += q
            # the drawing: the stream's arc from the spout to where it ends
            keep = np.linspace(0, len(path) - 1, min(STREAM_SEGMENTS + 1, len(path)))
            self.stream = [path[int(round(k))] for k in keep]
            self.stream_frac = frac
            self.stream_caught = caught
        self.world.step({"graduate": (o, np.asarray(objects["graduate"].quat_xyzw, float))}, dt, hulls)

    def step(self, objects, hulls=None, in_sim=False) -> None:
        self.hulls, self.in_sim = hulls, in_sim
        self.a, self.b = joint_angle("tube_a", STOW_A), joint_angle("tube_b", STOW_B)
        self.v = joint_angle("valve", 0.0)
        if in_sim and hulls is None:
            self.hand_check_off = True  # the stream and lipstick-on-a-hand checks could not run: no full credit
        self.flow(objects, DT, hulls)
        local = tube_local(objects)
        if self.local is not None and len(self.local) == len(local):
            hit = touched_between(self.local, local)
        else:
            hit = touches(objects)
        if hit:
            self.touched = True
        self.local = local
        if hulls:  # lipstick carried by a hand: a link that touched the coated surface, then the tube
            global COATED_SAMPLES
            if COATED_SAMPLES is None:
                COATED_SAMPLES = _coated_samples()
            gp, gR = _state_pose(objects, "graduate")
            self.red |= hull_contacts(hulls, COATED_SAMPLES @ gR.T + gp)
            if self.red and hull_contacts([(n, h) for n, h in hulls if n in self.red], tube_surface(objects)):
                self.touched = self.carried = True
        gp, _ = _state_pose(objects, "graduate")
        vp, _ = _state_pose(objects, "drainbag/valve")
        now = np.concatenate([gp, vp])
        still = self.last is not None and float(np.abs(now - self.last).max()) <= STILL_STEP
        self.last = now
        self.good = self.good + 1 if (self.done_state(objects) and still) else 0

    def released(self, objects) -> bool:
        """Both hands clear of the spout, the tube and the graduate: every tool point and pad tip RELEASE_MARGIN away
        and, in the sim, no part of either arm (its collision hulls: palm, fingers, wrist, forearm) within
        RELEASE_MARGIN of any part of the drain tube, so nothing but the sleeve holds the spout up."""
        spout_parts = (gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, -LV), COLLAR_R),
                       gk.Seg((0.0, 0.0, -LV), (0.0, 0.0, -TIP_L), SPOUT_R))
        tube_parts = (gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, -LB), TUBE_R),)
        run_parts = (gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, -LA), TUBE_R),)
        clear = bool(gk.released_clear(objects, "drainbag/valve", spout_parts, RELEASE_MARGIN)
                     and gk.released_clear(objects, "drainbag/tube_b", tube_parts, RELEASE_MARGIN)
                     and gk.released_clear(objects, "drainbag/tube_a", run_parts, RELEASE_MARGIN)
                     and gk.released_clear(objects, "graduate", None, RELEASE_MARGIN))
        if clear and self.in_sim:
            clear = bool(self.hulls) and not hull_contacts(self.hulls, tube_surface(objects), tol=RELEASE_MARGIN)
        return clear

    def done_state(self, objects) -> bool:
        _o, R = graduate_frame(objects)
        return (self.graduate_ml() >= FULL_ML and self.spilled_ml() <= CLEAN_ML and not self.touched
                and valve_open_fraction(self.v) == 0.0 and in_holder(objects) and tilt_deg(R) <= UPRIGHT_DEG
                and on_underpad(objects) and self.released(objects))

    def tier(self, objects) -> float:
        g, s = self.graduate_ml(), self.spilled_ml()
        if s > SPILL_MAX_ML or self.hand_steps * DT > HAND_HIT_S:
            return 0.0
        _o, R = graduate_frame(objects)
        upright = tilt_deg(R) <= UPRIGHT_DEG
        closed = valve_open_fraction(self.v) == 0.0
        if g >= FULL_ML and closed and upright:
            out = 0.75
            if self.good >= HOLD_STEPS and not self.hand_check_off:
                out = 1.0
        elif (g >= FULL_ML) or (g >= HALF_ML and closed and upright):
            out = 0.5
        elif g >= TRACE_ML:
            out = 0.25
        else:
            out = 0.0
        return min(out, TOUCH_CAP) if self.touched else out


_L = Ledger()
_HULLS = {"now": None, "failed": 0, "sim": False}  # the arms' collision hulls at this control step (set by the simulator hook)
_DEBUG = int(os.environ.get("DRAINBAG_DEBUG", "0") or 0)


def _update(objects):
    """Advance the trial's record (and the urine) once per control step; success() and score() share it."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.step(objects, _HULLS.get("now"), _HULLS.get("sim", False))
    if _DEBUG and n is not None and n % _DEBUG == 0:  # run log only (DRAINBAG_DEBUG=<every n steps>)
        tip, _d, _p, _R_ = spout(objects)
        print(f"drain step {n}: bag {_L.bag / ML:.1f} mL, graduate {_L.graduate_ml():.1f} mL, spilled "
              f"{_L.spilled_ml():.2f} mL, valve {math.degrees(_L.v):.0f} deg, stub {math.degrees(_L.a):.0f} tube "
              f"{math.degrees(_L.b):.0f} deg, in its sleeve {in_holder(objects)}, spout z {float(tip[2]) - FLOOR:.3f}, touched "
              f"{_L.touched} (via a hand {_L.carried}, red links {sorted(_L.red)}), gap {1000 * tube_gap(objects):.1f} "
              f"mm, stream on a hand {_L.hand_ml:.2f} mL, hand check off "
              f"{_L.hand_check_off}, on underpad {on_underpad(objects)}, tier "
              f"{_L.tier(objects)}", flush=True)
        if _DEBUG and not _VIS.get("friction_logged"):  # run log only: the sleeve's friction in the stage, once
            _VIS["friction_logged"] = True
            _log_sleeve_friction()
        if _DEBUG:  # run log only: the spout in its sleeve, the pad, the hull counts
            tip_b = _state_pose(objects, "drainbag/bag")[1].T @ (tip - _state_pose(objects, "drainbag/bag")[0])
            _below, _inside, _radial, _up = holder_readout(objects)  # (DRAINBAG_DEBUG)
            print(f"  sleeve: spout tip {1000 * (float(tip_b[2]) - SLEEVE_TOP):+.1f} mm from its top, "
                  f"{1000 * math.hypot(float(tip_b[0]) - SLEEVE_X, float(tip_b[1])):.1f} mm off its axis, collar "
                  f"{1000 * _inside:.1f} mm inside ({1000 * _radial:.1f} mm off axis), pad "
                  f"{1000 * joint_angle('pad', 0.0):.2f} mm back, arm hulls {_HULLS.get('counts')}", flush=True)
        if _DEBUG and "left_gripper" in objects and "right_gripper" in objects:  # run log only
            col = _p + _R_ @ np.array([0.0, 0.0, -LV / 2])  # (DRAINBAG_DEBUG)
            grip_l = _p + _R_ @ np.array([0.0, 0.0, TUBE_GRIP - LV / 2])  # (DRAINBAG_DEBUG)
            _go, Rg = graduate_frame(objects)  # (inside the DRAINBAG_DEBUG guard above)
            print(f"  left hand - tube grip mm {np.round((objects['left_gripper'].pos - grip_l) * 1000, 1)}, right hand - "
                  f"collar mm {np.round((objects['right_gripper'].pos - col) * 1000, 1)}, openings "
                  f"{kit.opening(objects, 'left_gripper'):.2f}/{kit.opening(objects, 'right_gripper'):.2f}, bag swing "
                  f"{math.degrees(joint_angle('bag', 0.0)):.1f} deg, graduate tilt {tilt_deg(Rg):.1f} deg", flush=True)


def _log_sleeve_friction() -> None:
    """Run log only: the physics material bound to one of the sleeve's colliders and to the valve collar, as the stage
    has them (checks that the sleeve's own plastic friction is the one in force)."""
    try:
        import omni.usd
        from pxr import UsdShade

        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not (path.endswith("/bag/C1") or path.endswith(f"/bag/C{len(_sleeve_colliders())}")
                    or path.endswith("/pad/C0") or path.endswith("/valve/C0")):
                continue
            mat, _rel = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose="physics")
            if not mat:
                sys.stderr.write(f"drainbag: friction at {path}: no physics material bound" + "\n")
                continue
            mp = mat.GetPrim()
            vals = {k: mp.GetAttribute(k).Get() for k in ("physics:staticFriction", "physics:dynamicFriction",
                                                         "physxMaterial:frictionCombineMode") if mp.HasAttribute(k)}
            sys.stderr.write(f"drainbag: friction at {path} ({prim.GetTypeName()}): {mp.GetPath()} {vals}" + "\n")
    except Exception as e:  # noqa: BLE001  (a log line only)
        sys.stderr.write(f"drainbag: friction check failed ({e!r})" + "\n")


def graduate_ml(objects=None) -> float:
    """mL of urine in the graduate."""
    return _L.graduate_ml()


def spilled_ml(objects=None) -> float:
    """mL that landed outside the graduate (on the underpad or the floor)."""
    return _L.spilled_ml()


def success(objects):
    """At least 570 mL in the graduate, at most 5 mL spilled, no part of the drain tube ever touched the graduate's
    coated surface, the valve closed, the spout all the way up in its sleeve, the graduate upright on the underpad, both
    hands clear and everything still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0 when more than 20 mL is spilled, or when the stream has run into a hand or arm for more than 1 s in all.
    Otherwise: 1.0 for at least 570 mL in the graduate with the valve closed, the graduate upright on the underpad, at
    most 5 mL spilled, the spout all the way up in its sleeve and both hands clear, still, for 1 s (and, in the sim, the
    arms' hulls read at every step); 0.75 for at least 570 mL with the valve closed and the graduate upright; 0.5 for
    at least 570 mL otherwise (valve left open, or the graduate tipped after), or at least 300 mL with the valve closed
    and the graduate upright; 0.25 for at least 100 mL. A touch by any part of the drain tube at any time caps the trial
    at 0.25. Doing nothing scores 0."""
    _update(objects)
    return _L.tier(objects)


# -- drawing the water (Isaac Lab markers; never affects the score) ------------------------------------------------------
# Created when the env is built (so the renderer has compiled the material long before the first frame) and drawn in
# its start state on every scene reset, before the settle steps and the reset renders: frame 0 already shows the full
# bag. After that it is redrawn every control step from the trial's record.
_VIS = {"markers": None, "failed": False}
SURFACE_T = 0.002
STREAM_R = 0.004  # m: the drawn stream's radius with the valve fully open (the 6 mm bore's jet, a little widened so it
STREAM_R_MIN = 0.0015  # shows on the 320 px camera); thinner while the valve is only partly open, never below this
STREAM_SEGMENTS = 24
STREAM_CHECK_POINTS = 160  # points along the stream's arc checked against the arms' hulls (about 2 mm apart)
STREAM_INTO = 0.004  # m: the drawn stream starts this far inside the spout and ends this far below the surface it meets
BAG_SLICES = 12
_HIDDEN = (np.array([0.0, 0.0, -1.0]), (1.0, 0.0, 0.0, 0.0), (1e-4, 1e-4, 1e-4))  # under the floor, for an empty draw


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        try:  # partial opacity in RTX real-time (otherwise opacity < 1 is ignored and the water draws opaque)
            import carb
            carb.settings.get_settings().set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
        except Exception:  # noqa: BLE001
            pass
        lin = tuple(kit._linear(c) for c in WATER_RGB)
        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=WATER_OPACITY)
        jet = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=JET_OPACITY)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/water",
            markers={"disc": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat),
                     "block": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=mat),
                     "jet": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=jet)}))
    except Exception as e:  # noqa: BLE001  (visuals are optional: scoring never depends on them)
        sys.stderr.write(f"drainbag: no visuals ({e!r})\n")
        _VIS["failed"] = True
    return _VIS["markers"]


def _wxyz(R) -> tuple:
    x, y, z, w = lk.quat(R)
    return (w, x, y, z)


def _seg(p, q, r):
    """(center, quat wxyz, scale) of a unit cylinder drawn from p to q with radius r."""
    v = np.asarray(q, float) - np.asarray(p, float)
    n = float(np.linalg.norm(v))
    if n < 1e-5:
        return None
    zc = v / n
    ref = np.array([1.0, 0.0, 0.0]) if abs(zc[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    xc = np.cross(ref, zc)
    xc /= np.linalg.norm(xc)
    Rs = np.stack([xc, np.cross(zc, xc), zc], 1)
    return (np.asarray(p) + np.asarray(q)) / 2, _wxyz(Rs), (r, r, n)


def stream_path(points) -> list:
    """The drawn stream's center line: the arc, started a little inside the spout and run a little below the surface
    it lands on, so it meets both without a gap."""
    pts = [np.asarray(p, float) for p in points]
    if len(pts) < 2:
        return pts
    d0, d1 = pts[1] - pts[0], pts[-1] - pts[-2]
    n0, n1 = float(np.linalg.norm(d0)), float(np.linalg.norm(d1))
    if n0 > 1e-6:
        pts[0] = pts[0] - d0 / n0 * STREAM_INTO
    if n1 > 1e-6:
        pts[-1] = pts[-1] + d1 / n1 * STREAM_INTO
    return pts


def inside_path(points, o, R) -> list:
    """A caught stream's drawn path inside the graduate: the ballistic arc until it meets the inside wall (a fast jet
    reaches the near wall before the surface), then down that wall to the end height (it runs down the wall)."""
    pts = [np.asarray(p, float) for p in points]
    if len(pts) < 2:
        return pts
    loc = [R.T @ (p - o) for p in pts]

    def out_of(lp, margin=0.001 + STREAM_R):  # the drawn tube's surface stays inside the wall
        h = min(max(float(lp[2]), 0.0), GR_H)
        return float(lp[2]) < GR_H and math.hypot(lp[0], lp[1]) > GR_B + (GR_T - GR_B) * h / GR_H - margin

    for k in range(1, len(loc)):
        if out_of(loc[k]) and not out_of(loc[k - 1]):
            lo, hi = loc[k - 1], loc[k]
            for _ in range(30):
                mid = (lo + hi) / 2
                lo, hi = (lo, mid) if out_of(mid) else (mid, hi)
            hit = lo
            ang = math.atan2(hit[1], hit[0])
            z_end = float(loc[-1][2])
            run = []
            for h in np.linspace(float(hit[2]), z_end, 6)[1:]:
                rr = GR_B + (GR_T - GR_B) * min(max(h, 0.0), GR_H) / GR_H - 0.001 - STREAM_R
                run.append(np.array([rr * math.cos(ang), rr * math.sin(ang), h]))
            return [o + R @ lp for lp in loc[:k]] + [o + R @ hit] + [o + R @ lp for lp in run]
    return pts


def drawing(objects, L=None, segments: bool = True) -> list:
    """(prototype index, center, quat wxyz, scale) of what is drawn: the water in the bag (slices from its bottom seam
    to its level, each as thick as the filled bag there, so the bag shows full and visibly empties), the surface in the
    graduate (at the frustum's exact level for its volume) and, only where the stream's tube mesh is unavailable, the
    stream as end-to-end cylinders along stream_line()."""
    L = _L if L is None else L
    out = []
    p, R = _state_pose(objects, "drainbag/bag")
    depth = bag_depth(L.bag)
    if depth > 0.001:
        floor = p + R @ np.asarray(_PORT_L, float)
        h = depth / BAG_SLICES
        for k in range(BAG_SLICES):
            zc = (k + 0.5) * h
            t = max(bag_thickness(zc) - 0.004, 0.003)
            out.append((1, floor + R @ np.array([0.0, 0.0, zc]), _wxyz(R), (t, BAG_W - 0.010, h * 1.02)))
    o, Rg = graduate_frame(objects)
    vol = L.world.vessels["graduate"].volume
    if vol > 0.0:
        if tilt_deg(Rg) <= 15.0:  # upright: a level disc at the exact depth, just inside the wall
            hd = graduate_depth(vol)
            r = GR_B + (GR_T - GR_B) * hd / GR_H - 0.001 - hd * math.tan(math.radians(tilt_deg(Rg)))
            out.append((0, o + Rg @ np.array([0.0, 0.0, hd]), (1.0, 0.0, 0.0, 0.0), (r, r, SURFACE_T)))
        else:
            surf = L.world.vessels["graduate"].surface(o, np.asarray(objects["graduate"].quat_xyzw, float))
            if surf is not None:  # tipped: the free surface's outline drawn as the ellipse that spans it
                c, n, pts = surf
                _u, _sv, vt = np.linalg.svd(pts - c, full_matrices=False)
                e1 = vt[0] - (vt[0] @ n) * n
                e1 /= max(np.linalg.norm(e1), 1e-9)
                Rs = np.stack([e1, np.cross(n, e1), n], 1)
                ext = np.abs((pts - c) @ Rs[:, :2]).max(0)
                out.append((0, c, _wxyz(Rs), (float(ext[0]), float(ext[1]), SURFACE_T)))
    line = stream_line(objects, L) if segments else None
    if line is not None:
        pts, r = line
        for p0, p1 in zip(pts[:-1], pts[1:]):
            seg = _seg(p0, p1, r)
            if seg is not None:
                out.append((2, *seg))
    return out


def stream_line(objects, L=None):
    """(center line points, radius) of the drawn stream, or None while nothing flows."""
    L = _L if L is None else L
    if L.stream is None:
        return None
    r = max(STREAM_R * math.sqrt(max(L.stream_frac, 0.0)), STREAM_R_MIN)
    pts = L.stream
    if getattr(L, "stream_caught", False):
        o, Rg = graduate_frame(objects)
        pts = inside_path(pts, o, Rg)
    return stream_path(pts), r


JET_RINGS, JET_SIDES = 40, 12


def jet_tube(pts, r) -> np.ndarray:
    """Vertices (JET_RINGS x JET_SIDES) of a smooth tube of radius r along the polyline pts, resampled evenly by length
    (fixed topology, so the mesh only moves its points)."""
    pts = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    u = np.linspace(0.0, s[-1], JET_RINGS)
    c = np.stack([np.interp(u, s, pts[:, k]) for k in range(3)], 1)
    t = np.gradient(c, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    side = np.cross(t[0], [0.0, 0.0, 1.0])
    side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-6 else np.array([0.0, 1.0, 0.0])
    ang = np.linspace(0.0, 2 * math.pi, JET_SIDES, endpoint=False)
    out = np.zeros((JET_RINGS, JET_SIDES, 3))
    for k in range(JET_RINGS):
        a = side - (side @ t[k]) * t[k]
        a /= max(np.linalg.norm(a), 1e-9)
        b = np.cross(t[k], a)
        out[k] = c[k] + r * (np.outer(np.cos(ang), a) + np.outer(np.sin(ang), b))
    return out.reshape(-1, 3)


def _jet():
    """The stream's tube mesh (one smooth surface, no segment seams), made with the markers; None if it cannot be."""
    if "jet" in _VIS:
        return _VIS["jet"]
    _VIS["jet"] = None
    try:
        import isaaclab.sim as sim_utils
        import omni.usd
        from pxr import Gf, UsdGeom, UsdShade, Vt

        stage = omni.usd.get_context().get_stage()
        path, mat_path = "/World/Visuals/water_jet", "/World/Visuals/water_jet_material"
        lin = tuple(kit._linear(c) for c in WATER_RGB)
        cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=JET_OPACITY)
        cfg.func(mat_path, cfg)
        mesh = UsdGeom.Mesh.Define(stage, path)
        n, m = JET_RINGS, JET_SIDES
        idx = []
        for k in range(n - 1):
            for j in range(m):
                j2 = (j + 1) % m
                idx += [k * m + j, k * m + j2, (k + 1) * m + j2, (k + 1) * m + j]
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * ((n - 1) * m)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -1.0)] * (n * m)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, 1.0)] * (n * m)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(mat_path)))
        _VIS["jet"] = mesh
        _VIS["jet_api"] = (Gf, Vt)
    except Exception as e:  # noqa: BLE001  (fall back to drawing the stream as marker segments)
        sys.stderr.write(f"drainbag: no stream mesh, segments instead ({e!r})\n")
    return _VIS["jet"]


def _draw_jet(mesh, line, origin) -> None:
    Gf, Vt = _VIS["jet_api"]
    if line is None:  # nothing flows: the tube shrinks to a hidden point under the floor
        pts = np.tile(np.array([0.0, 0.0, -1.0]) + origin, (JET_RINGS * JET_SIDES, 1))
        nrm = np.tile([0.0, 0.0, 1.0], (JET_RINGS * JET_SIDES, 1))
    else:
        c, r = line
        pts = jet_tube(c, r)
        nrm = (pts - jet_tube(c, 0.0)) / max(r, 1e-9)
        pts = pts + origin
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(nrm.astype(np.float32)))


def _draw(objects, origin, L=None) -> None:
    m = _markers()
    if m is None:
        return
    mesh = _jet()
    items = drawing(objects, L, segments=mesh is None) or [(0, *_HIDDEN)]
    if mesh is not None:
        _draw_jet(mesh, stream_line(objects, L), origin)
    m.visualize(translations=np.array([np.asarray(c, float) + origin for _, c, _, _ in items], float),
                orientations=np.array([q for _, _, q, _ in items], float),
                scales=np.array([s for _, _, _, s in items], float),
                marker_indices=np.array([i for i, _, _, _ in items], int))


def _start_objects(base) -> dict:
    """The bag's and the graduate's poses straight from the sim, read without yamkit's per-step joint laws (which must
    not act before the first control step), as states for drawing()."""
    from types import SimpleNamespace

    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    origin = np_(base.scene.env_origins)[0]
    art = base.scene[BAG.name]
    i = list(art.body_names).index(BAG_LINK.name)
    bag = SimpleNamespace(pos=np_(art.data.body_pos_w)[0][i] - origin, quat_xyzw=np_(art.data.body_quat_w)[0][i])
    gp = np_(base.arena_world.get_pose_w(GRADUATE.name))[0]
    return {"drainbag/bag": bag, "graduate": SimpleNamespace(pos=gp[:3] - origin, quat_xyzw=gp[3:])}


def spring_laws(art) -> None:
    """The sleeve's pad spring on from the first physics step after a reset. The scene's settle runs before any control
    step, and yamkit applies its laws only at control steps (its reset leaves them limp): without this the pad would
    not press on the collar during the settle, and the spout would slide out of its sleeve before the trial starts."""
    names = list(art.joint_names)
    j = names.index(f"{PAD_LINK.name}_joint")
    kit._put_joint(art, j, PAD_LINK.start)
    law = PAD_LINK.law
    kit._set_drive(art, j, target=law.target, stiffness=law.k, effort=0.0, damping=law.damp)


def _spring_after_resets(base) -> None:
    """Wrap the scene's reset (again) so the springs go on after yamkit's own reset hook, which yamkit installs at the
    first control step and which leaves every law-held joint limp: the springs then hold through every later settle."""
    if getattr(base, "_drainbag_springs_outer", False):
        return
    base._drainbag_springs_outer = True
    inner = base.reset

    def reset(*a, **k):
        out = inner(*a, **k)
        try:
            spring_laws(base.scene[BAG.name])
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"drainbag: sleeve pad not sprung at reset ({e!r})\n")
        return out

    base.reset = reset


def _prime(env) -> None:
    """At env build: create the markers now and draw the start state after every scene reset."""
    base = env.unwrapped
    if getattr(base, "_drainbag_vis", False) or _markers() is None:
        return
    _jet()
    base._drainbag_vis = True
    inner = base.reset

    def reset(*a, **k):
        out = inner(*a, **k)
        try:
            spring_laws(base.scene[BAG.name])
        except Exception as e:  # noqa: BLE001
            sys.stderr.write(f"drainbag: sleeve pad not sprung at reset ({e!r})\n")
        if not _VIS["failed"]:
            try:
                _draw(_start_objects(base), base.scene.env_origins[0].detach().cpu().numpy(), Ledger())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                sys.stderr.write(f"drainbag: start drawing off ({e!r})\n")
        return out

    base.reset = reset


# -- the simulator hook: the drain steps once per control step, after yamkit's object_states ------------------------------
def _patch(scene) -> None:
    if getattr(scene, "_drainbag", False):
        return
    scene._drainbag = True
    inner = scene.object_states

    def object_states(env, names):
        out = inner(env, names)
        if getattr(env.unwrapped, "_yamkit_reset", False):
            _spring_after_resets(env.unwrapped)
        if all(n in out for n in ("drainbag/bag", "drainbag/valve", "graduate")):
            _HULLS["now"], _HULLS["sim"] = None, True
            try:  # the arms' own collision hulls at their live poses (loaded from the competition environment)
                hulls = lk.arm_hulls(env.unwrapped)
                per_arm = {r: sum(1 for n, _h in hulls if n.startswith(r + "/")) for r in ("left_robot", "right_robot")}
                _HULLS["counts"] = per_arm
                if all(per_arm.values()):  # both arms, or the check counts as off
                    _HULLS["now"] = hulls
                else:
                    _HULLS["error"] = f"hulls per arm {per_arm}"
            except Exception as e:  # noqa: BLE001  (tried again every step; reported every 100 failed steps)
                _HULLS["error"] = repr(e)
            if _HULLS["now"] is None:
                _HULLS["failed"] += 1
                if _HULLS["failed"] % 100 == 1:
                    sys.stderr.write(f"drainbag: HAND CHECK OFF: no arm hulls for the stream check "
                                     f"({_HULLS.get('error')}), {_HULLS['failed']} steps\n")
            elif not _HULLS.get("reported"):
                _HULLS["reported"] = True
                sys.stderr.write(f"drainbag: hand check on: arm hulls {_HULLS['counts']}\n")
            _update(out)
            if not _VIS["failed"]:
                try:
                    _draw(out, env.unwrapped.scene.env_origins[0].detach().cpu().numpy())
                except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                    sys.stderr.write(f"drainbag: visuals off ({e!r})\n")
                    _VIS["failed"] = True
        return out

    scene.object_states = object_states
    make = getattr(scene, "make_env", None)
    if make is not None:
        def make_env(*a, **k):
            env = make(*a, **k)
            try:
                _prime(env)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"drainbag: visuals not primed ({e!r})\n")
            return env

        scene.make_env = make_env


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also adds the drain step after yamkit's patch
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------------
# The way a nursing assistant empties the bag, with the two hands split the same way. Right hand: takes the graduate by
# its handle's top arm (from above), carries it onto the underpad beneath the drain and lets go. Left hand (the aiming
# hand): holds the drain tube 8.5 cm up from the collar, pulls the spout straight down out of its sleeve, leans the
# tube out clear of the sleeve, swings it out and down until the spout points into the graduate's mouth without
# touching it, and holds it there. Right hand (the valve hand): takes the valve collar and twists it open, lets go (the
# collar stays where it is turned) and stands by. As the bag empties, its level and so the stream's speed drop and the
# stream's arc shortens; the aiming hand follows it, moving the spout so the stream keeps landing on the same spot in
# the mouth. When the bag is empty the valve hand takes the collar again and twists it closed; the aiming hand swings
# the tube back up, stands it under the sleeve and pushes the spout straight up into it until its end meets the
# sleeve's top; both hands let go and back away. (The right hand cannot hold the tube fingers-across in front of the
# sleeve: a global IK search finds no pose within 35 mm; the left one can.)
X_YAW = -math.pi / 2  # top-down fingers closing across world x (turned this way: the wrist stays inside its limits)
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open (guardkit's gripper model)
DRAIN_A = math.radians(129.0)  # the run out of the outlet raised 39 degrees above horizontal, out toward the robot
# (with the spout 15 mm past the collar, 36 degrees left its tip low enough that a full bag's stream, missing the
# graduate, would carry past the pan's near wall)
DRAIN_PHI = math.radians(60.0)  # the spout's axis 60 degrees from straight down, toward the robot
PULL_CLEAR = 0.006  # m: pulled out, the spout's tip comes this far below the sleeve's mouth
PULL_A = STOW_A - math.asin((SLEEVE_DEPTH + PULL_CLEAR) / LA)  # the stub lowered 25 degrees: the spout's tip comes
# down out of its sleeve
PUSH_PAST = 0.002  # m: once the spout is in, the hand eases back to aim this far past the sleeve's closed top, and
# lets go there (a light push up, so easing off never pulls the spout back down)
PUSH_DEEP = 0.025  # m: pushing the spout in, the hand aims this far past the top: the arm gives under the pad's push
# (in sim runs, aiming 2 mm past stopped 11 mm short with the collar on the pad's chamfer (task.py 31b81c30832f), and
# with the 30 mm sleeve aiming 15 mm past stopped 9 mm short, the shoulder on a 45-degree chamfer (42bca433c847)), so
# it aims deeper to push through; the top stops the spout
PUSH_HOLD_S = 1.5  # s: the hand keeps pushing this long before easing off
RAISE_PHI = math.pi - 0.62  # the tube already leans out toward the robot as the stub rises: the palm keeps off the bag
LEAN_PHI = math.pi - 0.75  # below the sleeve the tube first leans out this far, so it rises clear of the sleeve
TUBE_GRIP = 0.085  # m: the aiming hand holds the tube this far up from the collar's middle (room for the valve hand)
COLLAR_GRIP_UP = 0.004  # m: the valve hand pinches the collar this far toward the tube from its middle
APPROACH = 0.070  # m: a hand comes in to the tube along its fingers from this far back
CARRY_DZ = 0.040  # m: the graduate is carried this far above the floor
SLIDE_OFF = 0.030  # m: the open fingers slide this far out along the handle's arm (past its end) before lifting
# Grips: the fingers are aimed 2 mm inside each side of the part they hold (3 mm on the valve collar, a firmer pinch to
# turn it against its seal's friction).
TUBE_FINGER = None  # set below (needs finger_for)
TWIST_OPEN_PAST = math.radians(5.0)  # the valve hand turns this far past the open stop (it slips there; more is out of reach)
TWIST_CLOSE_PAST = math.radians(2.0)  # ... and 2 deg past the closed stop when closing, pressing the collar onto it
# (the stop holds it; further is out of the right wrist's reach)
# Margins: the aim is planned for the valve 75 % open, and the valve is held open long enough for 97 % of the bag to
# pass even at 60 % open, in case the collar turns a little in the pads during the twist.
PLAN_OPEN_FRAC = 0.75
HOLD_OPEN_FRAC = 0.6
AIM_STEPS = 6  # waypoints of the aiming hand's following move
AIM_TIP_RIM = (0.035, 0.060)  # m: the spout's end stays this high above the rim while it follows the stream


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


TUBE_FINGER = finger_for(TUBE_R - 0.002)
GRASP_FINGER = finger_for(COLLAR_R - 0.003)
WIDE = finger_for(0.011)  # 5 mm each side of the collar: the far pad must fit between it and the bag's front
TUBE_WIDE = finger_for(0.0095)


def hand_angles(m) -> tuple:
    """(yaw, roll, pitch) of a kit.Tilted waypoint whose hand is turned by m from the straight-down grip, written as
    pitch(y) @ roll(x) @ yaw(z)."""
    m = np.asarray(m, float)
    r = math.asin(max(-1.0, min(1.0, -m[1, 2])))
    if math.cos(r) < 1e-6:
        # gimbal lock (roll +-90 degrees, which every untwisted tube grip is): only pitch -+ yaw is defined, so take
        # yaw 0 and put the whole turn in the pitch (atan2(0, 0) would drop both, and the hand would never roll)
        s = 1.0 if r > 0 else -1.0
        return 0.0, r, math.atan2(s * m[0, 1], m[0, 0])
    return math.atan2(m[1, 0], m[1, 1]), r, math.atan2(m[0, 2], m[2, 2])


def tilted(duration: float, pos, finger: float, label: str, m) -> kit.Tilted:
    yaw, r, p = hand_angles(m)
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=yaw, roll=r, pitch=p)


def tube_angles(a: float, phi: float) -> tuple:
    """(stub, tube) joint angles that put the stub at `a` and the tube's (and spout's) axis `phi` from straight down
    toward the robot (pi: straight up)."""
    return a, phi - a


def hand_turn(phi: float, twist: float, side: float) -> np.ndarray:
    """A hand's turn from the straight-down grip when it holds the tube (or collar) of a tube pointing `phi`: fingers
    pointing along `side` * y (square to the plane the tube swings in: -1 the left hand's, pointing right; +1 the right
    hand's, pointing left), closing across the tube in that plane, then turned `twist` about the tube's axis
    (negative turns the collar toward its open stop). Swinging the tube is then a roll of the wrist about the
    fingers' direction."""
    d = np.array([-math.sin(phi), 0.0, -math.cos(phi)])  # the spout's outward axis
    c = -np.array([math.cos(phi), 0.0, -math.sin(phi)])  # the closing direction, in the swing plane square to d
    a = np.array([0.0, side, 0.0])
    m = np.stack([np.cross(c, -a), c, -a], 1)  # takes the straight-down grip's x, y (closing), z (back up the arm)
    return axis_rot(-d, twist) @ m


def collar_turn(phi: float, twist: float) -> np.ndarray:
    """The valve hand's (right hand's) turn on the collar."""
    return hand_turn(phi, twist, 1.0)


def tube_turn(phi: float) -> np.ndarray:
    """The aiming hand's (left hand's) turn on the tube."""
    return hand_turn(phi, 0.0, -1.0)


def _axis(a: float, phi: float) -> tuple:
    f = fk(a=tube_angles(a, phi)[0], b=tube_angles(a, phi)[1])
    d = (f["tip"] - f["collar"]) / np.linalg.norm(f["tip"] - f["collar"])
    return f, d


def tube_wp(duration, a, phi, finger, label, back=0.0) -> kit.Tilted:
    """An aiming-hand waypoint on the tube (TUBE_GRIP up from the collar) with the tube at (stub a, axis phi), the hand
    `back` m behind it along its fingers."""
    f, d = _axis(a, phi)
    m = tube_turn(phi)
    return tilted(duration, f["collar"] - TUBE_GRIP * d + back * m[:, 2], finger, label, m)


def collar_wp(duration, a, phi, twist, finger, label, back=0.0) -> kit.Tilted:
    """A valve-hand waypoint on the collar with the tube at (stub a, axis phi), the hand `back` m behind it along its
    fingers."""
    f, d = _axis(a, phi)
    m = collar_turn(phi, twist)
    return tilted(duration, f["collar"] - COLLAR_GRIP_UP * d + back * m[:, 2], finger, label, m)


# -- the stream the aiming hand follows (the scorer's own drain model, run ahead offline) ---------------------------------
def tip_head(a: float, phi: float, volume_m3: float) -> tuple:
    """(spout end, outward axis, head) for the tube at (a, phi) and this much urine in the (unswung) bag."""
    f, d = _axis(a, phi)
    floor = float(f["bag"][0][2]) - STRAP - BAG_H
    return f["tip"], d, floor + bag_depth(volume_m3) - float(f["tip"][2])


def landing(a: float, phi: float, volume_m3: float):
    """Where the stream crosses the rim's plane for the tube at (a, phi) and this much in the bag (None: no flow)."""
    tip, d, head = tip_head(a, phi, volume_m3)
    if head <= 0.0:
        return None
    return land(tip, math.sqrt(2 * G * head) * d, FLOOR + GR_RIM_Z)


def aim_point() -> np.ndarray:
    """Where the stream is kept landing (on the rim's plane): where the full bag's stream lands at the draining pose."""
    return np.asarray(landing(DRAIN_A, DRAIN_PHI, START_ML * ML), float)


def graduate_place() -> tuple:
    """The graduate is set down centered between the spout's end at the draining pose and the aim point: the spout is
    over the mouth and the stream lands inside it."""
    f, _d = _axis(DRAIN_A, DRAIN_PHI)
    c = (f["tip"][:2] + aim_point()) / 2
    return float(c[0]), float(c[1])


def drain_seconds(fill_ml: float = START_ML, upto: float = 0.995, frac: float = 1.0) -> float:
    """How long the valve, `frac` open, takes to pass `upto` of the bag at the demo's draining pose (offline
    integration)."""
    v, t = fill_ml * ML, 0.0
    while v > (1 - upto) * fill_ml * ML and t < 600:
        _tip, _d, head = tip_head(DRAIN_A, DRAIN_PHI, v)
        if head <= 0:
            break
        v -= CD * OUTLET_A * frac * math.sqrt(2 * G * head) * DT
        t += DT
    return t


def predicted_volume(t: float, frac: float = PLAN_OPEN_FRAC) -> float:
    """m^3 left in the bag t s after the valve opens `frac` (at the draining pose)."""
    v = START_ML * ML
    for _ in range(int(round(t / DT))):
        _tip, _d, head = tip_head(DRAIN_A, DRAIN_PHI, v)
        if head <= 0:
            break
        v = max(v - CD * OUTLET_A * frac * math.sqrt(2 * G * head) * DT, 0.0)
    return v


def aim_pose(volume_m3: float, prev=(DRAIN_A, DRAIN_PHI)) -> tuple:
    """The tube pose (stub a, axis phi) near `prev` that lands the stream for this bag volume closest to the mouth's
    aim point, with the spout's end AIM_TIP_RIM above the rim."""
    goal = aim_point()
    best, out = math.inf, prev
    for da in np.radians(np.arange(-6.0, 6.01, 0.5)):
        for dp in np.radians(np.arange(-6.0, 6.01, 0.5)):
            a, phi = prev[0] + da, prev[1] + dp
            tip, _d, head = tip_head(a, phi, volume_m3)
            rise = float(tip[2]) - (FLOOR + GR_RIM_Z)
            if not AIM_TIP_RIM[0] <= rise <= AIM_TIP_RIM[1] or head <= 0:
                continue
            hit = landing(a, phi, volume_m3)
            cost = float(np.linalg.norm(hit - goal)) + 0.002 * math.hypot(da, dp)
            if cost < best:
                best, out = cost, (a, phi)
    return out


# -- the timeline ---------------------------------------------------------------------------------------------------------
GRADUATE_DONE_S = 2.0 + 1.5 + 0.8 + 1.2 + 3.0 + 1.5 + 0.8 + 1.2 + 1.5  # the graduate is down and the right hand clear
LEFT_DONE_S = GRADUATE_DONE_S  # the left hand starts on the tube once the graduate is down
T_TUBE_IN = GRADUATE_DONE_S + 0.5  # the aiming hand moves in to the tube
T_AIMED = T_TUBE_IN + 1.5 + 0.8 + 1.5 + 2 * 1.2 + 4 * 1.0 + 1.0  # the spout is over the mouth, steady
VALVE_IN_S, VALVE_GRIP_S, TWIST_S, LET_GO_S, BACK_S = 1.5, 0.8, 4.0, 0.8, 1.5
T_OPENING = T_AIMED + VALVE_IN_S + VALVE_GRIP_S  # the twist starts
T_FLOW = T_OPENING + TWIST_S / 4  # the valve passes 10 degrees about here: flow starts
T_FOLLOW = T_OPENING + TWIST_S + LET_GO_S + BACK_S  # the valve hand is clear: the aiming hand starts to follow
HOLD_S = math.ceil(drain_seconds(upto=0.97, frac=HOLD_OPEN_FRAC)) + 2.0
T_CLOSING = T_FLOW + HOLD_S  # the valve hand comes back in to close


def aim_schedule() -> list:
    """(time s, stub a, axis phi) of the aiming hand's following waypoints, from T_FOLLOW to T_CLOSING: each pose lands
    the stream the plan predicts for that moment (PLAN_OPEN_FRAC open) on the aim point."""
    out, prev = [], (DRAIN_A, DRAIN_PHI)
    for k in range(1, AIM_STEPS + 1):
        t = T_FOLLOW + (T_CLOSING - T_FOLLOW) * k / AIM_STEPS
        v = predicted_volume(min(t, T_CLOSING - 2.0) - T_FLOW)
        if v < 0.03 * START_ML * ML:  # the last trickle falls where it falls: keep the pose
            out.append((t, *prev))
            continue
        prev = aim_pose(v, prev)
        out.append((t, *prev))
    return out


def swing_path(a0, phi0, a1, phi1, n: int) -> list:
    return [(a0 + (a1 - a0) * k / n, phi0 + (phi1 - phi0) * k / n) for k in range(1, n + 1)]


def graduate_moves() -> list:
    gx, gy = GR_START
    px, py = graduate_place()
    hx, hy, hz = HANDLE_GRIP
    z = FLOOR + hz
    # a firm squeeze (fingers commanded shut): the graduate hangs off the pinch, and a light one lets it turn about the
    # closing axis and tip
    open_, hold = finger_for(0.016), FINGER_CLOSED
    from yam_arena.demo import Waypoint

    def w(d, pos, f, label):
        return Waypoint(float(d), tuple(float(v) for v in pos), float(f), label, X_YAW)

    return [w(2.0, (gx + hx, gy + hy, z + 0.08), open_, "over the graduate's handle"),
            w(1.5, (gx + hx, gy + hy, z), open_, "around the handle's top arm"),
            w(0.8, (gx + hx, gy + hy, z), hold, "grip"),
            w(1.2, (gx + hx, gy + hy, z + CARRY_DZ), hold, "lift"),
            w(3.0, (px + hx, py + hy, z + CARRY_DZ), hold, "carry it beneath the bag"),
            w(1.5, (px + hx, py + hy, z + 0.001), hold, "set it on the underpad"),
            w(0.8, (px + hx, py + hy, z + 0.001), open_, "let go"),
            w(1.2, (px + hx, py + hy + GR_SIDE * SLIDE_OFF, z + 0.001), open_, "slide off the end of the handle's arm"),
            w(1.5, (px + hx - 0.06, py + hy + GR_SIDE * (SLIDE_OFF + 0.04), z + 0.06), open_,
              "lift clear, back toward the robot")]


def right_plan() -> list:
    """The right hand: the graduate onto the underpad, then the valve: open, stand by, close."""
    plan = graduate_moves()
    a0, p0 = DRAIN_A, DRAIN_PHI
    _t, a1, p1 = aim_schedule()[-1]
    t_open = -VALVE_OPEN - TWIST_OPEN_PAST
    plan.append(collar_wp(max(T_AIMED - GRADUATE_DONE_S, 1.0), a0, p0, 0.0, WIDE,
                          "beside the valve collar, waiting for the spout to be aimed", back=APPROACH))
    plan += [collar_wp(VALVE_IN_S, a0, p0, 0.0, WIDE, "around the collar"),
             collar_wp(VALVE_GRIP_S, a0, p0, 0.0, GRASP_FINGER, "grip the collar"),
             collar_wp(TWIST_S, a0, p0, t_open, GRASP_FINGER, "twist the valve open"),
             collar_wp(LET_GO_S, a0, p0, t_open, WIDE, "let go: the valve stays open"),
             collar_wp(BACK_S, a0, p0, t_open, WIDE, "back away from the collar", back=APPROACH),
             collar_wp(max(T_CLOSING - T_FOLLOW - VALVE_IN_S, 1.0), a1, p1, t_open, WIDE,
                       "stand by while the bag empties", back=APPROACH),
             collar_wp(VALVE_IN_S, a1, p1, t_open, WIDE, "back around the collar"),
             collar_wp(VALVE_GRIP_S, a1, p1, t_open, GRASP_FINGER, "grip the collar"),
             collar_wp(TWIST_S, a1, p1, TWIST_CLOSE_PAST, GRASP_FINGER, "twist the valve closed"),
             collar_wp(LET_GO_S, a1, p1, TWIST_CLOSE_PAST, WIDE, "let go"),
             collar_wp(BACK_S, a1, p1, TWIST_CLOSE_PAST, WIDE, "back away from the collar", back=APPROACH)]
    from yam_arena.demo import Waypoint
    plan.append(Waypoint(7.0, (0.20, -0.22, TOP + 0.20), WIDE, "out of the way", X_YAW))
    plan.append(Waypoint(6.0, (0.20, -0.22, TOP + 0.20), WIDE, "wait", X_YAW))
    return plan


T_CLOSED = T_CLOSING + VALVE_IN_S + VALVE_GRIP_S + TWIST_S + LET_GO_S + BACK_S  # the valve hand is clear again


def sleeve_phi(a: float) -> float:
    """The tube's angle that keeps the spout's end straight under the sleeve's axis for the stub at `a` (the knee
    swings on the stub's arc, so the tube leans a little to keep the spout in line with the sleeve)."""
    goal = STAND_POS[0] + HOOK[0] + SLEEVE_X

    def err(phi):
        return float(fk(a=a, b=phi - a)["tip"][0]) - goal

    lo, hi = math.pi - 0.3, math.pi + 0.3
    e_lo = err(lo)
    for _ in range(50):
        mid = (lo + hi) / 2
        if (err(mid) > 0) == (e_lo > 0):
            lo, e_lo = mid, err(mid)
        else:
            hi = mid
    return (lo + hi) / 2


def sleeve_path(a0: float, a1: float, n: int) -> list:
    """(stub, tube) poses from a0 to a1 with the spout kept in line with its sleeve."""
    return [(a0 + (a1 - a0) * k / n, sleeve_phi(a0 + (a1 - a0) * k / n)) for k in range(1, n + 1)]


def left_tube_poses() -> list:
    """The left hand's plan as (duration, stub angle, tube angle, finger, label, back) on the tube: the tube out of its
    sleeve, the spout aimed and following the stream, then back into its sleeve, pushed all the way up."""
    push_a, deep_a = STOW_A + PUSH_PAST / LA, STOW_A + PUSH_DEEP / LA
    out = [(7.0, STOW_A, math.pi, TUBE_WIDE, "behind the tube in its sleeve", APPROACH),
           (max(T_TUBE_IN - 7.0, 0.5), STOW_A, math.pi, TUBE_WIDE, "wait for the graduate to be set down", APPROACH),
           (1.5, STOW_A, math.pi, TUBE_WIDE, "around the tube", 0.0),
           (0.8, STOW_A, math.pi, TUBE_FINGER, "grip the tube", 0.0)]
    for i, (a, phi) in enumerate(sleeve_path(STOW_A, PULL_A, 3)):
        out.append((0.6, a, phi, TUBE_FINGER, f"pull the spout straight down out of its sleeve {i + 1}", 0.0))
    out.append((1.0, PULL_A, LEAN_PHI, TUBE_FINGER, "lean the tube out, clear of the sleeve", 0.0))
    for i, (a, phi) in enumerate(swing_path(PULL_A, LEAN_PHI, DRAIN_A, RAISE_PHI, 2)):
        out.append((1.2, a, phi, TUBE_FINGER, f"raise the stub {i + 1}", 0.0))
    for i, (a, phi) in enumerate(swing_path(DRAIN_A, RAISE_PHI, DRAIN_A, DRAIN_PHI, 4)):
        out.append((1.0, a, phi, TUBE_FINGER, f"swing the spout down over the mouth {i + 1}", 0.0))
    out.append((1.0, DRAIN_A, DRAIN_PHI, TUBE_FINGER, "steady over the mouth", 0.0))
    out.append((T_FOLLOW - T_AIMED, DRAIN_A, DRAIN_PHI, TUBE_FINGER, "hold the aim while the valve is opened", 0.0))
    t_prev = T_FOLLOW
    for i, (t, a, phi) in enumerate(aim_schedule()):
        out.append((t - t_prev, a, phi, TUBE_FINGER, f"follow the stream as it shortens {i + 1}", 0.0))
        t_prev = t
    _t, a1, p1 = aim_schedule()[-1]
    out.append((T_CLOSED - t_prev + 0.5, a1, p1, TUBE_FINGER, "hold the aim while the valve is closed", 0.0))
    for i, (a, phi) in enumerate(swing_path(a1, p1, DRAIN_A, RAISE_PHI, 4)):
        out.append((1.0, a, phi, TUBE_FINGER, f"swing the tube back up {i + 1}", 0.0))
    for i, (a, phi) in enumerate(swing_path(DRAIN_A, RAISE_PHI, PULL_A, LEAN_PHI, 2)):
        out.append((1.2, a, phi, TUBE_FINGER, f"lower the stub {i + 1}", 0.0))
    out.append((1.0, PULL_A, sleeve_phi(PULL_A), TUBE_FINGER, "stand the tube up under the sleeve", 0.0))
    for i, (a, phi) in enumerate(sleeve_path(PULL_A, deep_a, 4)):
        out.append((0.6, a, phi, TUBE_FINGER, f"push the spout straight up into its sleeve {i + 1}", 0.0))
    out += [(PUSH_HOLD_S, deep_a, sleeve_phi(deep_a), TUBE_FINGER, "keep pushing it up against the sleeve's top", 0.0),
            (0.8, push_a, sleeve_phi(push_a), TUBE_FINGER, "ease off", 0.0),
            (0.8, push_a, sleeve_phi(push_a), TUBE_WIDE, "let go", 0.0),
            (1.5, STOW_A, math.pi, TUBE_WIDE, "back away from the tube", APPROACH),
            (4.0, STOW_A, math.pi, TUBE_WIDE, "wait", APPROACH)]
    return out


def left_plan() -> list:
    """The left hand's waypoints (see left_tube_poses)."""
    return [tube_wp(d, a, phi, f, label, back=back) for d, a, phi, f, label, back in left_tube_poses()]


def demo():
    """Right hand sets the graduate on the underpad beneath the drain; left hand pulls the spout out of its sleeve and
    aims it into the graduate; right hand opens the valve; the left hand follows the shortening stream; the right hand
    closes the valve; the left hand pushes the spout back up into its sleeve."""
    return {"left": left_plan(), "right": right_plan()}
