"""Six fresh potato gnocchi lie on the floor of a sauté pan of simmering water on an induction hob. Each one floats up
when it is cooked through, at its own time. The job is a restaurant cook's: lift each one out with a slotted spoon
after it floats up and within 45 seconds, without pinching it, and put it on the plate; a gnocchi taken out before it
floats is raw in the middle and must not be served.

There is no fluid in the simulator. floatkit (beside this file) adds the water's forces to each gnocchi before every
physics step: buoyancy from its submerged volume, drag, a slow simmer drift and the push of the moving spoon. A
gnocchi's density drops from 1,080 to 950 kg/m3 at its cooking time, so it rises. PhysX does every contact. Every
threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402
import floatkit as fk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "The gnocchi are cooking. Take them out onto the plate when they're done, then turn off the burner."

# -- looks ----------------------------------------------------------------------------------------------------------
STEEL = kit.Look((0.78, 0.79, 0.80), 0.25, metallic=0.9)
GLASS_BLACK = kit.Look((0.05, 0.05, 0.06), 0.15)
NYLON = kit.Look((0.10, 0.10, 0.11), 0.5)  # black nylon spoon
GRIP = kit.Look((0.14, 0.14, 0.15), 0.8)  # its soft handle
CERAMIC = kit.Look((0.96, 0.95, 0.92), 0.25)
DOUGH = kit.Look((0.93, 0.84, 0.58), 0.7)  # potato gnocchi
WOOD = kit.Look((0.66, 0.50, 0.33), 0.7)
# The water, drawn only (no collider; floatkit gives its forces): see-through, so the pieces on the floor and the
# floaters at the surface both show, as in a real pan. If the renderer shows it opaque, set the opacity to 0 (the
# floaters show only 1 mm above an opaque surface).
WATER = kit.Look((0.62, 0.78, 0.90), 0.05, opacity=0.25)

# -- layout (world frame: x away from the arms, y to the robot's left, z up; m) -------------------------------------
# Induction hob: a slim portable single-zone hob, black glass top, 36 cm deep (x) x 28 cm wide (y) x 4 cm tall, fixed,
# with its rotary control knob on the top in the near-left corner (below).
HOB_C, HOB_SIZE = (0.300, 0.020), (0.36, 0.28, 0.04)
HOB_TOP = TOP + HOB_SIZE[2]
# Pan: a 24 cm stainless sauté pan (inside 24 cm across, 7 cm deep, 5 mm base), fixed on the hob (with 2.4 L of
# water it weighs about 3.5 kg). Long handle pointing away from the robot (+x).
PAN_C = HOB_C
PAN_R = 0.120  # inside radius
PAN_FLOOR_T = 0.005
PAN_DEPTH = 0.070
PAN_FLOOR_Z = HOB_TOP + PAN_FLOOR_T
PAN_RIM_Z = PAN_FLOOR_Z + PAN_DEPTH  # 14.0 cm above the table
WATER_L = 2.4
SURFACE_Z0 = PAN_FLOOR_Z + WATER_L * 1e-3 / (math.pi * PAN_R ** 2)  # 5.3 cm of water, 1.7 cm under the rim

# Gnocchi: fresh potato gnocchi, ridged pillows 2.3 cm long, 1.5 cm across and 1.1 cm thick: a flat core rounded all
# round by half the thickness (two side-by-side capsules and the block between them as colliders). Raw dough density
# 1,080 kg/m3 (potato gnocchi dough is 1.05-1.1 g/cm3), so 3.0 g each; cooked through, 950: they sink raw and float when
# done. Start on the pan floor in a ring 4.5 cm out from its center, on the half of the pan nearer the right arm (from
# its near-left round to its far-right).
G_SIZE = (0.023, 0.015, 0.011)  # length, width, thickness
G_SHAPE = fk.Body("gnocchi", size=G_SIZE, mass=1.0, shape="pillow")
G_CORE_A, G_CORE_B, G_R = fk.pillow_dims(G_SIZE)  # half-length and half-width of the flat core, rounding radius
G_RHO_RAW = 1080.0
G_VOLUME = G_SHAPE.volume  # 2.75 cm3
G_MASS = G_RHO_RAW * G_VOLUME  # 3.0 g
G_RHO_COOKED = 950.0
G_NAMES = tuple(f"g{i}" for i in range(1, 7))
G_RING = 0.045
G_ANGLES = (120, 160, 200, 240, 280, 320)  # degrees around the pan's center, from +x toward +y
G_FRICTION = (0.4, 0.3)  # wet cooked dough is slippery (static, dynamic); PhysX averages it with the other surface
G_START = tuple((PAN_C[0] + G_RING * math.cos(math.radians(a)), PAN_C[1] + G_RING * math.sin(math.radians(a)),
                 PAN_FLOOR_Z + G_SIZE[2] / 2) for a in G_ANGLES)
G_YAW = tuple(math.radians(a + 90) for a in G_ANGLES)  # long side along the ring
# When they float: fresh potato gnocchi dropped into water at a low boil float up 1 1/2 to 2 minutes later and are
# lifted out with a slotted spoon as they come up (America's Test Kitchen, Potato Gnocchi). The cook drops them in two
# at a time, 85, 57 and 30 s before the trial starts, so the two of a pair float up a few seconds apart, often while the
# previous piece is still being lifted out. Each piece floats FLOAT_MIN to FLOAT_MAX s after it goes in.
FLOAT_MIN, FLOAT_MAX = 90.0, 120.0  # s: 1 1/2 to 2 minutes
DROP_AT = (-85.0, -57.0, -30.0)  # s, relative to the trial's start: when each pair goes in
RISE_S_EST = 2.5  # s: a piece takes about this long to come up from the floor once it has turned buoyant
# The canonical run (seed 0): G1 and G2 go in first, then G3 and G4, then G5 and G6; they float up these many seconds
# after going in, all within 1 1/2 to 2 minutes: at about 10.5, 21.5, 38.5, 49.5, 65.5 and 78.5 s into the trial. The
# second of each pair leaves the floor just after the first has been lifted out of the water (a piece rising beside the
# bowl as it slides would be caught raw or shoved aside), floats up while the first is on its way to the plate, and
# floats 5-6 s before the spoon gets to it.
FLOAT_S0 = (95.5, 106.5, 95.5, 106.5, 95.5, 108.5)
COOK_AT = tuple(DROP_AT[i // 2] + FLOAT_S0[i] - RISE_S_EST for i in range(6))  # s: G1..G6 turn buoyant
COOK_RAMP = 2.0  # s over which the density change is ramped in

# Slotted spoon: a nylon slotted spoon 32 cm long: bowl 8 x 6 cm, 1.2 cm deep at the back, its sides 8 mm and its front
# edge flush with its floor (so food slides off the tip), with nine 3 x 25 mm slots; a 24 cm
# handle rising 30 degrees from the bowl's plane, a 1.5 cm-wide neck for its first 6 cm, then a soft grip 2.8 cm wide
# and 1.8 cm thick; 80 g. The spoon's frame: origin at the middle of the bowl's inside floor, +z up when the bowl is
# level, +y toward the handle (the bowl's tip at -y).
BOWL_L, BOWL_W, BOWL_D, BOWL_T = 0.080, 0.060, 0.012, 0.003
SIDE_H = 0.008  # the bowl's sides are lower than its back (12 mm); its front edge has no lip
CRANK = math.radians(30.0)
HANDLE_START = np.array([0.0, BOWL_L / 2, BOWL_D])
HANDLE_DIR = np.array([0.0, math.cos(CRANK), math.sin(CRANK)])
HANDLE_LEN = 0.240
NECK_LEN, NECK_W, NECK_T = 0.060, 0.015, 0.006
GRIP_W, GRIP_T = 0.028, 0.018
GRIP_S = 0.090  # the demo holds the grip 9 cm along the handle (3 cm into the soft grip), choked up for reach
GRASP_UP = 0.0  # m: the hand holds the grip on its center line (lying on the counter it is 2.4 cm up: the fingertips
# stay 1.1 cm off the counter)
SPOON_MASS = 0.080


def handle_point(s: float) -> np.ndarray:
    """The handle's center line, s m from where it leaves the bowl (spoon frame)."""
    return HANDLE_START + s * HANDLE_DIR


HANDLE_UP = np.array([0.0, -math.sin(CRANK), math.cos(CRANK)])  # the handle's top-face normal (spoon frame)
GRIP_LOCAL = handle_point(GRIP_S) + GRASP_UP * HANDLE_UP

# The spoon lies on the counter right of the hob the way a spoon is put down: on the back of its bowl and the far end of
# its handle, nothing under it. The handle lies nearly flat, pointing away from the robot (+x), and the 30-degree crank
# tips the bowl up (REST_TIP, worked out from the colliders below).
REST_PSI = -math.pi / 2  # the spoon's turn at rest: handle toward +x
SPOON_XY = (0.310, -0.280)  # the bowl's floor center at rest (x, y)

# Plate: a white ceramic plate 18 cm across, its flat well 14 cm across and 6 mm up, its rim rising to 2 cm; fixed.
PLATE_C = (0.165, -0.255)
PLATE_R, PLATE_WELL_R = 0.090, 0.070
PLATE_TOP = TOP + 0.006  # the well's surface
PLATE_RIM_Z = TOP + 0.020

# -- thresholds (each with its reason in the README and task.json) -----------------------------------------------------
SURFACED_TOL = 0.003  # m: a piece's top within 3 mm of the surface (a cooked gnocchi floats with about 1 mm showing)
WINDOW_S = 45.0  # s: lifted out within 45 s of surfacing (left longer, it goes soft and falls apart)
ON_PLATE_UP = 0.020  # m: on the plate = lowest point at most 2 cm above the well (one piece leaning on another is fine)
STILL_SPEED, STILL_SPIN = 0.01, 0.5  # m/s, rad/s: at rest
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 1 s on the plate and still
LADDER = {6: 1.0, 5: 0.75, 4: 0.5, 3: 0.5, 2: 0.25, 1: 0.25, 0: 0.0}
HELD_CAP = 0.5  # a hand still on the spoon at the end caps the score
BURNER_CAP = 0.5  # the burner not turned off at the end caps the score
GRIPPERS = ("left_gripper", "right_gripper")


# -- seeds: the canonical layout is seed 0; other seeds shuffle the cooking times and the simmer drift ---------------
def trial_seed() -> int:
    """GNOCCHI_SEED=<n> repeats a run (0 is the canonical schedule). Unset, each run draws its own seed and prints it, so
    the order, the times and the place of each rise are not known in advance (a memorized schedule can't score)."""
    v = os.environ.get("GNOCCHI_SEED")
    if v is not None:
        try:
            return int(v)
        except ValueError:
            pass
    seed = int.from_bytes(os.urandom(4), "little") % 1_000_000 + 1
    print(f"lift-gnocchi: seed {seed} (GNOCCHI_SEED={seed} repeats this run)", file=sys.stderr, flush=True)
    return seed


def trial_cook_times(seed: int) -> tuple:
    """When G1..G6 turn buoyant (s into the trial). Seed 0: COOK_AT. Other seeds: the pieces are shared out over the
    three drops at random, and each floats up a random time between 1 1/2 and 2 minutes after its drop, so the times,
    the order and the place of each rise change from run to run and pieces come up together."""
    if seed == 0:
        return COOK_AT
    rng = np.random.default_rng(seed)
    slot = rng.permutation(6)
    t = [DROP_AT[slot[i] // 2] + rng.uniform(FLOAT_MIN, FLOAT_MAX) - RISE_S_EST for i in range(6)]
    return tuple(float(v) for v in t)


SIMMER_RMS = 0.003  # m/s: the surface drift of a gentle simmer (a few millimeters a second; not a rolling boil)

# -- objects ----------------------------------------------------------------------------------------------------------
WALL_POST = 0.005


def _wall_posts(r_in, z_lo, z_hi, post=WALL_POST):
    """A ring wall as upright cylinder posts standing just outside radius `r_in` (colliders are convex)."""
    rc = r_in + post
    n = int(math.ceil(2 * math.pi * rc / (2 * post)))
    return [kit.Collider((rc * math.cos(2 * math.pi * k / n), rc * math.sin(2 * math.pi * k / n), (z_lo + z_hi) / 2),
                         radius=post, height=z_hi - z_lo) for k in range(n)]


HOB = Box("hob", pos=(HOB_C[0], HOB_C[1], TOP + HOB_SIZE[2] / 2), size=HOB_SIZE, color=(0.05, 0.05, 0.06))
_pan_cols = [kit.Collider((0.0, 0.0, PAN_FLOOR_T / 2), size=(0.17, 0.17, PAN_FLOOR_T))]
_pan_cols += _wall_posts(PAN_R, PAN_FLOOR_T, PAN_FLOOR_T + PAN_DEPTH)
_PH = (PAN_R + 0.002 + 0.10, 0.0, PAN_FLOOR_T + PAN_DEPTH - 0.010)  # the long handle, +x
PAN = kit.Prop(
    "pan", pos=(PAN_C[0], PAN_C[1], HOB_TOP), size=(0.25 + 0.20, 0.25, PAN_FLOOR_T + PAN_DEPTH), mass=None,
    meshes=(kit.lathe([(0.0, 0.0), (PAN_R + 0.002, 0.0), (PAN_R + 0.002, PAN_FLOOR_T + PAN_DEPTH),
                       (PAN_R, PAN_FLOOR_T + PAN_DEPTH), (PAN_R, PAN_FLOOR_T), (0.0, PAN_FLOOR_T)], STEEL),
            kit.box(_PH, (0.20, 0.025, 0.012), STEEL),
            kit.cylinder((0.0, 0.0, PAN_FLOOR_T + (SURFACE_Z0 - PAN_FLOOR_Z) / 2), PAN_R - 0.0005,
                         SURFACE_Z0 - PAN_FLOOR_Z, WATER)),
    colliders=tuple(_pan_cols) + (kit.Collider(_PH, size=(0.20, 0.025, 0.012)),))


RIDGE = kit.Look((0.78, 0.66, 0.40), 0.8)


def _capsule_mesh(center, r, half, look):
    """A capsule along x (a lathe: hemispherical ends, straight middle 2 * half long)."""
    prof = [(r * math.sin(a), -half - r * math.cos(a)) for a in np.linspace(0.0, math.pi / 2, 7)]
    prof += [(r * math.cos(a), half + r * math.sin(a)) for a in np.linspace(0.0, math.pi / 2, 7)]
    return kit.lathe([(float(x), float(z)) for x, z in prof], look, center=center, axis="x", n=20)


def _gnocchi(i: int) -> kit.Prop:
    """A ridged pillow: two capsules side by side and the block between them (a flat core rounded all round)."""
    a, b, r = G_CORE_A, G_CORE_B, G_R
    meshes = (_capsule_mesh((0.0, b, 0.0), r, a, DOUGH), _capsule_mesh((0.0, -b, 0.0), r, a, DOUGH),
              kit.box((0, 0, 0), (2 * a, 2 * b, 2 * r), DOUGH))
    ridges = tuple(kit.box((x, 0.0, r - 0.0003), (0.0012, 2 * b + 1.2 * r, 0.0008), RIDGE)
                   for x in (-0.0075, -0.0045, -0.0015, 0.0015, 0.0045, 0.0075))
    cols = (kit.Collider((0.0, b, 0.0), radius=r, height=2 * a, axis="X", capsule=True),
            kit.Collider((0.0, -b, 0.0), radius=r, height=2 * a, axis="X", capsule=True),
            kit.Collider((0.0, 0.0, 0.0), size=(2 * a, 2 * b, 2 * r)))
    return kit.Prop(G_NAMES[i], pos=G_START[i], yaw=G_YAW[i], size=G_SIZE, mass=G_MASS, friction=G_FRICTION,
                    meshes=meshes + ridges, colliders=cols)


GNOCCHI = [_gnocchi(i) for i in range(6)]


def _handle_parts():
    """The handle as short pieces (colliders are boxes in the spoon's frame): the neck in 4 steps, the grip in 5,
    each stepped up to follow the 30-degree rise."""
    out, meshes = [], []
    for a, b, w, t, n, look in ((0.0, NECK_LEN, NECK_W, NECK_T, 4, NYLON), (NECK_LEN, HANDLE_LEN, GRIP_W, GRIP_T, 5,
                                                                             GRIP)):
        d = (b - a) / n
        for k in range(n):
            p, q = handle_point(a + k * d), handle_point(a + (k + 1) * d)
            c = tuple(float(v) for v in (p + q) / 2)
            out.append(kit.Collider(c, size=(w, abs(q[1] - p[1]), abs(q[2] - p[2]) + t)))
        meshes.append(kit.cylinder(tuple(handle_point((a + b) / 2)), w / 2, b - a, look, axis=HANDLE_DIR))
    return tuple(out), tuple(meshes)


_HCOLS, _HMESH = _handle_parts()
_BOWL_WALLS = (((0.0, BOWL_L / 2, BOWL_D / 2), (BOWL_W, 0.002, BOWL_D)),  # the back, where the handle starts
               ((-BOWL_W / 2, 0.0, SIDE_H / 2), (0.002, BOWL_L, SIDE_H)),
               ((BOWL_W / 2, 0.0, SIDE_H / 2), (0.002, BOWL_L, SIDE_H)))
_BOWL_COLS = (kit.Collider((0.0, 0.0, -BOWL_T / 2), size=(BOWL_W, BOWL_L, BOWL_T)),) + tuple(
    kit.Collider(c, size=z) for c, z in _BOWL_WALLS)
def _box_corners(cols) -> np.ndarray:
    return np.array([np.asarray(c.center, float) + np.array([sx, sy, sz]) * np.asarray(c.size, float) / 2
                     for c in cols for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _rest_pose() -> tuple:
    """(tip, height of the spoon's origin over the counter) for the spoon lying on the counter: tipped about its own x
    (negative = bowl tip up) until the back of the bowl and the end of the handle touch the counter together."""
    P = _box_corners(_BOWL_COLS + _HCOLS)
    bowl = P[:, 1] <= BOWL_L / 2 + 1e-6
    end = P[:, 1] >= handle_point(HANDLE_LEN - 0.03)[1]

    def low(t, sel):
        return float((P[sel, 1] * math.sin(t) + P[sel, 2] * math.cos(t)).min())

    lo, hi = -1.2, 0.0  # f(lo) > 0 (handle end below the bowl), f(hi) < 0
    for _ in range(60):
        t = (lo + hi) / 2
        if low(t, bowl) - low(t, end) > 0:
            lo = t
        else:
            hi = t
    t = (lo + hi) / 2
    return t, -float((P[:, 1] * math.sin(t) + P[:, 2] * math.cos(t)).min())


REST_TIP, _REST_UP = _rest_pose()  # about -31 degrees: the bowl's tip up, the handle flat
SPOON_START = (float(SPOON_XY[0]), float(SPOON_XY[1]), float(TOP + _REST_UP + 0.0005))
SPOON = kit.Prop(
    "spoon", pos=SPOON_START, yaw=REST_PSI, tilt_xyzw=(float(math.sin(REST_TIP / 2)), 0.0, 0.0,
                                                       float(math.cos(REST_TIP / 2))),
    size=(BOWL_W, BOWL_L + HANDLE_LEN, 0.15), mass=SPOON_MASS,
    meshes=(kit.box((0, 0, -BOWL_T / 2), (BOWL_W, BOWL_L, BOWL_T), NYLON),)
    + tuple(kit.box(c, z, NYLON) for c, z in _BOWL_WALLS) + _HMESH,
    colliders=_BOWL_COLS + _HCOLS)

_plate_cols = [kit.Collider((0.0, 0.0, 0.003), radius=PLATE_R, height=0.006)]
_plate_cols += _wall_posts(PLATE_WELL_R + 0.004, 0.006, 0.014, post=0.004)
PLATE = kit.Prop(
    "plate", pos=(PLATE_C[0], PLATE_C[1], TOP), size=(2 * PLATE_R, 2 * PLATE_R, 0.020), mass=None,
    meshes=(kit.lathe([(0.0, 0.0), (0.05, 0.0), (PLATE_R, 0.014), (PLATE_R, 0.020), (PLATE_WELL_R, 0.008),
                       (PLATE_WELL_R - 0.002, 0.006), (0.0, 0.006)], CERAMIC),),
    colliders=tuple(_plate_cols))

# Burner knob: the hob's rotary control, a black range knob 4.2 cm across and 2.6 cm tall (an 8 mm skirt and a grip bar
# 3.6 cm long, 9 mm thick and 1.8 cm tall across its top, with a white pointer at one end of the bar), on the hob's top
# in its near-left corner. Like a range's infinite switch it turns clockwise from OFF (pointer toward the robot) through
# its settings to HIGH at 300 degrees, with about 0.1 N m of friction all the way and a click into OFF (about 0.15 N m
# to turn it out again); OFF is also its stop. It starts 90 degrees clockwise of OFF, the low-medium setting that keeps
# the water at a gentle simmer. Its joint angle: 0 at OFF, negative clockwise (seen from above).
KNOB_C = (0.145, 0.125)
KNOB_R, KNOB_SKIRT_H = 0.021, 0.008
KNOB_BAR = (0.036, 0.009, 0.018)  # length, thickness, height (above the skirt)
KNOB_MASS = 0.025
KNOB_START = -math.radians(90.0)
KNOB_MAX = math.radians(300.0)
KNOB_FRICTION = 0.10  # N m, anywhere off the click
KNOB_HOLD = 2.0  # N m/rad: the friction acts as a spring to where the knob was left, slipping past KNOB_FRICTION
KNOB_CLICK = math.radians(8.0)  # within 8 degrees of OFF it clicks into OFF
KNOB_CLICK_TORQUE = 0.15  # N m to turn it out of the click
KNOB_LOOK = kit.Look((0.06, 0.06, 0.07), 0.4)
MARK = kit.Look((0.95, 0.95, 0.95), 0.4)
MARK_HIGH = kit.Look((0.85, 0.20, 0.15), 0.5)


class RangeKnob:
    """The knob's feel, as a yamkit law (it sets the joint's PhysX drive every control step; the drive is implicit, so
    it stays stable on a 25 g knob): inside the click it is pulled to OFF; elsewhere it holds where it was left, giving
    way once turned with more than KNOB_FRICTION (the spring's anchor slides along behind it)."""

    def __init__(self, friction, hold, click, click_torque, damp=0.004):
        self.friction, self.hold, self.click, self.click_torque, self.damp = friction, hold, click, click_torque, damp

    def fresh(self, lk):
        return {"anchor": lk.start, "off": False}

    def drive(self, key, q, qd, J, mem):
        if q >= -self.click:
            mem["anchor"], mem["off"] = q, True
            return 0.0, self.click_torque / self.click, 0.0, self.damp
        mem["off"] = False
        slip = self.friction / self.hold
        a = min(max(mem.get("anchor", q), q - slip), q + slip)
        mem["anchor"] = a
        return a, self.hold, 0.0, self.damp


def _burner() -> kit.Mechanism:
    """The knob on the hob's top, with the dial printed on the glass round it: a white OFF mark toward the robot and a
    dot for each setting, every 30 degrees clockwise, the last (HIGH) red."""
    marks = [kit.box((-0.029, 0.0, 0.0003), (0.006, 0.0025, 0.0006), MARK)]
    for k in range(1, 11):
        a = math.pi - math.radians(30.0 * k)  # where the pointer is at that setting
        marks.append(kit.box((0.029 * math.cos(a), 0.029 * math.sin(a), 0.0003), (0.0025, 0.0025, 0.0006),
                             MARK_HIGH if k == 10 else MARK))
    top = KNOB_SKIRT_H + KNOB_BAR[2]
    knob = kit.Link(
        "knob", "revolute", "Z", (0.0, 0.0, 0.0), lower=-KNOB_MAX, upper=0.0, start=KNOB_START, mass=KNOB_MASS,
        meshes=(kit.cylinder((0.0, 0.0, KNOB_SKIRT_H / 2), KNOB_R, KNOB_SKIRT_H, KNOB_LOOK),
                kit.box((0.0, 0.0, KNOB_SKIRT_H + KNOB_BAR[2] / 2), KNOB_BAR, KNOB_LOOK),
                kit.box((-0.012, 0.0, top + 0.0003), (0.010, 0.002, 0.0006), MARK)),
        colliders=(kit.Collider((0.0, 0.0, 0.001 + (KNOB_SKIRT_H - 0.001) / 2), radius=KNOB_R,
                                height=KNOB_SKIRT_H - 0.001),
                   kit.Collider((0.0, 0.0, KNOB_SKIRT_H + KNOB_BAR[2] / 2), size=KNOB_BAR)),
        law=RangeKnob(KNOB_FRICTION, KNOB_HOLD, KNOB_CLICK, KNOB_CLICK_TORQUE))
    return kit.Mechanism("burner", pos=(KNOB_C[0], KNOB_C[1], HOB_TOP), size=(0.07, 0.07, 0.03), meshes=tuple(marks),
                         links=(knob,))


BURNER = _burner()
OBJECTS = [HOB, PAN, PLATE, BURNER, SPOON] + GNOCCHI

# -- the water ----------------------------------------------------------------------------------------------------------
_gx, _gy = np.meshgrid(np.linspace(-0.025, 0.025, 4), np.linspace(-0.035, 0.035, 5))
SPOON_WATER_PTS = tuple((float(x), float(y), 0.004) for x, y in zip(_gx.ravel(), _gy.ravel()))
VESSEL = fk.Vessel("pan", center=PAN_C, radius=PAN_R, floor_z=PAN_FLOOR_Z, liquid_m3=WATER_L * 1e-3)


def setup_water(seed: int) -> None:
    times = trial_cook_times(seed)
    fk.setup(VESSEL,
             [fk.Body(n, size=G_SIZE, mass=G_MASS, cd=0.9, cook=fk.Cook(times[i], G_RHO_COOKED, COOK_RAMP),
                      shape="pillow")
              for i, n in enumerate(G_NAMES)],
             tools=[fk.Tool("spoon", points=SPOON_WATER_PTS, radius=0.022)],
             drift=fk.Drift(rms=SIMMER_RMS, seed=seed))


setup_water(trial_seed())

# -- scorer helpers -----------------------------------------------------------------------------------------------------
G_PARTS = (gk.Seg((-G_CORE_A, G_CORE_B, 0.0), (G_CORE_A, G_CORE_B, 0.0), G_R),
           gk.Seg((-G_CORE_A, -G_CORE_B, 0.0), (G_CORE_A, -G_CORE_B, 0.0), G_R))
SPOON_PARTS = (gk.Block((0.0, 0.0, BOWL_D / 2 - BOWL_T / 2), (BOWL_W, BOWL_L, BOWL_D + BOWL_T)),
               gk.Seg(tuple(handle_point(0.0)), tuple(handle_point(NECK_LEN)), NECK_W / 2),
               gk.Seg(tuple(handle_point(NECK_LEN)), tuple(handle_point(HANDLE_LEN)), GRIP_W / 2))
def piece_z(objects, name: str) -> tuple:
    """(lowest, highest) z of a gnocchi (its flat core's farthest corner plus the rounding radius)."""
    s = objects[name]
    R = fk.quat_R(s.quat_xyzw)
    h = G_CORE_A * abs(R[2, 0]) + G_CORE_B * abs(R[2, 1]) + G_R
    z = float(np.asarray(s.pos, float)[2])
    return z - h, z + h


def in_pan(objects, name: str) -> bool:
    p = np.asarray(objects[name].pos, float)
    return bool(math.hypot(p[0] - PAN_C[0], p[1] - PAN_C[1]) < PAN_R and p[2] < PAN_RIM_Z)


def in_water(objects, name: str) -> bool:
    return in_pan(objects, name) and piece_z(objects, name)[0] < fk.surface_z()


def in_spoon(objects, name: str) -> bool:
    """The piece's center lies over the spoon's bowl (in the spoon's frame), within 3 cm above its floor."""
    s = objects["spoon"]
    q = fk.quat_R(s.quat_xyzw).T @ (np.asarray(objects[name].pos, float) - np.asarray(s.pos, float))
    return bool(abs(q[0]) <= BOWL_W / 2 and abs(q[1]) <= BOWL_L / 2 and -0.005 <= q[2] <= 0.030)


def on_plate(objects, name: str) -> bool:
    """On the plate, not in the spoon: center over the 14 cm well, lowest point at most 2 cm above it, at rest."""
    p = np.asarray(objects[name].pos, float)
    lo, _ = piece_z(objects, name)
    return bool(math.hypot(p[0] - PLATE_C[0], p[1] - PLATE_C[1]) <= PLATE_WELL_R
                and PLATE_TOP - 0.002 <= lo <= PLATE_TOP + ON_PLATE_UP
                and objects[name].resting(max_speed=STILL_SPEED, max_spin=STILL_SPIN)
                and not in_spoon(objects, name))


def pinched(objects, name: str) -> bool:
    """A gripper's own shape (palm, fingers, pads) touches the piece: contact, not nearness (guardkit, 2 mm)."""
    return bool(gk.touching(objects, name, G_PARTS, grippers=GRIPPERS))


def hand_on_spoon(objects) -> bool:
    return bool(gk.touching(objects, "spoon", SPOON_PARTS, grippers=GRIPPERS))


def knob_angle(objects) -> float:
    """The burner knob's angle (rad): 0 at OFF, negative clockwise."""
    return float(kit.joint(objects, "burner/knob").pos)


def burner_off(objects) -> bool:
    """The knob's pointer within 8 degrees of OFF (inside its click)."""
    return knob_angle(objects) >= -KNOB_CLICK


def surfaced_now(objects, name: str) -> bool:
    """Cooked through and floating on its own: its top within 3 mm of the surface, still in the water."""
    lo, hi = piece_z(objects, name)
    zs = fk.surface_z()
    return bool(fk.state().cooked.get(name) and in_pan(objects, name) and lo < zs and abs(hi - zs) <= SURFACED_TOL
                and not in_spoon(objects, name))


# -- one trial's record ------------------------------------------------------------------------------------------------
class Piece:
    def __init__(self):
        self.t_s = None  # first time it floated up on its own (s)
        self.t_o = None  # the last time it left the water (s)
        self.was_in = True
        self.crushed = False  # latched
        self.plated = 0  # consecutive steps on the plate and still


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.t = 0.0
        self.p = {n: Piece() for n in G_NAMES}


_L = Ledger()


def _update(objects):
    """Advance the trial's record once per control step (success() and score() share it)."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.t = (n if n is not None else _L.n) * DT
    _L.n += 1
    for name in G_NAMES:
        r = _L.p[name]
        if r.t_s is None and surfaced_now(objects, name):
            r.t_s = _L.t
        wet = in_water(objects, name)
        if r.was_in and not wet:
            r.t_o = _L.t
        r.was_in = wet
        if pinched(objects, name):
            r.crushed = True
        r.plated = r.plated + 1 if on_plate(objects, name) else 0


def good(name: str) -> bool:
    """Lifted out within 45 s after it floated up, on the plate (still for 1 s), never pinched."""
    r = _L.p[name]
    return bool(r.t_s is not None and r.t_o is not None and r.t_s <= r.t_o <= r.t_s + WINDOW_S
                and not r.was_in and r.plated >= HOLD_STEPS and not r.crushed)


def raw_served(objects, name: str) -> bool:
    """Out of the pan at the end, and it left the water before it had floated up (raw in the middle)."""
    r = _L.p[name]
    out = not in_pan(objects, name)
    return bool(out and (r.t_s is None or (r.t_o is not None and r.t_o < r.t_s)))


def tier(objects) -> float:
    if any(raw_served(objects, n) for n in G_NAMES):
        return 0.0
    s = LADDER[sum(good(n) for n in G_NAMES)]
    if hand_on_spoon(objects):
        s = min(s, HELD_CAP)
    if not burner_off(objects):
        s = min(s, BURNER_CAP)
    return s


def success(objects):
    """All six good, no raw piece out of the pan, the spoon let go, the burner off."""
    _update(objects)
    return tier(objects) >= 1.0


def score(objects):
    """0 if any gnocchi that had not floated up when it left the water ends outside the pan (a raw one served).
    Otherwise by the number of good pieces (lifted out between floating up and 45 s later, on the plate and still for
    1 s, never touched by a gripper): 6 is 1.0, 5 is 0.75, 3-4 is 0.5, 1-2 is 0.25, 0 is 0. A hand still on the spoon at
    the end caps the score at 0.5, and so does the burner's knob not at OFF at the end. Doing nothing scores 0."""
    _update(objects)
    return tier(objects)


# == scripted demo: open loop for the canonical seed (GNOCCHI_SEED=0) ===============================================
# The right hand holds the spoon's grip from above and to its right, the fingers closing across the grip's 2.8 cm width
# and pointing down and to the robot's left, square to the handle. On an approximate YAM model a straight-down hand
# tops out about 17 cm above the table over the pan; turned like this it reaches 24-30 cm there.
# Real yam_arena convention (STEP 0, Nov 13): yaw 0 closes the fingers along world y, yaw +-90 along world x. The
# hand's frame below is written by its world closing axis (column 0) and approach (column 2), then converted.
# yam_arena's grip_rotation(0): gripper x (the fingers' closing axis) -> world -y, y -> -x, z (approach) -> -z; and
# grip_rotation(yaw) = Rz(yaw) @ GRIP_DOWN.
GRIP_DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_HAND_IN_SPOON = np.column_stack([(1.0, 0.0, 0.0),  # closing across the grip's width (spoon x)
                                  (0.0, -math.cos(CRANK), -math.sin(CRANK)),
                                  (0.0, math.sin(CRANK), -math.cos(CRANK))])  # approach: square to the handle, down


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def spoon_rot(psi: float = 0.0, tip: float = 0.0) -> np.ndarray:
    """The spoon's world rotation: turned `psi` about the vertical (0: handle toward +y, -pi/2: toward +x), then its
    tip dipped `tip` (rad, positive = tip down) about its own x."""
    return rz(psi) @ rx(tip)


def hand_angles(R_hand: np.ndarray) -> tuple:
    """(yaw, roll, pitch) of a kit.Tilted waypoint whose rotation is R_hand: pitch(y) @ roll(x) @ grip(yaw)."""
    m = R_hand @ GRIP_DOWN.T
    r = math.asin(max(-1.0, min(1.0, -m[1, 2])))
    return math.atan2(m[1, 0], m[1, 1]), r, math.atan2(m[0, 2], m[2, 2])


def tilted_rot(yaw: float, roll: float, pitch: float) -> np.ndarray:
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return Ry @ Rx @ rz(yaw) @ GRIP_DOWN


def hold(duration, origin, psi, tip, finger, label):
    """A waypoint for the hand holding the spoon so its bowl's floor center is at `origin` (world)."""
    R = spoon_rot(psi, tip)
    tcp = np.asarray(origin, float) + R @ GRIP_LOCAL
    yaw, roll, pitch = hand_angles(R @ _HAND_IN_SPOON)
    return kit.Tilted(float(duration), tuple(float(v) for v in tcp), finger, label, yaw=yaw, roll=roll, pitch=pitch)


SCOOP_DEPTH = 0.023  # m: the bowl's floor center goes 2.3 cm under the surface (a floating gnocchi reaches 1.4 cm down)
SCOOP_TIP = math.radians(13.0)  # the bowl's tip 13 degrees down while it slides under a piece: the tip's lower edge
# stays 3 mm over the raw pieces on the floor, the sides' tops stay under a floater from the bowl's middle forward (so
# a piece that drifted off the bowl's center line is not steered out over a side), and the handle clears the rim
SLIDE_SPEED = 0.03  # m/s: slow, so the water it pushes moves the piece gently
SLIDE_PAST = 0.022  # m: the bowl slides on until the piece's start point is this far behind the bowl's middle, toward
# the handle (the water the bowl pushes carries a floater about half as far as the bowl slides, so it ends near the
# middle, not at the open tip)
RISE_S = 0.5  # s: at the slide's end the bowl rises at once, leveling, to the surface (a pushed floater keeps
# drifting toward the open tip at about 1.5 cm/s after the bowl stops; the bowl's floor must catch it first)
TURN_RATE = math.radians(90.0)  # rad/s: the wrist turns the handle at up to this
LIFT_SPEED = 0.025  # m/s for the first 2 cm out of the water, so it drains through the slots
LIFT_TIP = 0.0  # under the piece, the bowl comes level before the lift, so the piece does not ride toward its open tip
# (tipped further up, the handle and the fingers come down too near the water)
CARRY_SPEED = 0.35  # m/s
DUMP_TIP = math.radians(45.0)  # tip down this much over the plate: the piece slides off the bowl's tip (wet dough on
# nylon: friction about 0.8 averaged in the sim, so it slides past 39 degrees)
DUMP_PSI = -math.pi / 2  # over the plate the handle points away from the robot, so the hand stays off the pan; the
# bowl's tip (and the piece sliding off it) points toward the robot
DUMP_START_UP = 0.025  # m: the tip's lower edge this far over the well when the bowl comes down level (clears pieces)
DUMP_TIP_UP = 0.003  # m: ... and this far over it from 36 degrees on, so a piece steps off with no drop
DUMP_TIP_S = 1.2  # s: tipping from level to 45 degrees about the tip's edge (slow, so the piece leaves slowly)
_TIP_EDGE = np.array([0.0, -BOWL_L / 2, -BOWL_T])  # the bowl's front lower edge (spoon frame)
HELD_OFFSET = np.array([0.011, 0.005, 0.0])  # m, spoon frame: in the simulator the held bowl sits this far off the
# pose the hand is sent to (toward the spoon's +x and toward the handle; measured at the end of all six slides of one
# run: 0.9-1.4 cm and 0-0.8 cm, the spoon's set in the pinch), so the scoops send the hand this much the other way
AIM = (0.0, 1.0, 1.0, 1.0, 1.0, 1.0)  # share of HELD_OFFSET cancelled for G1..G6: G1's scoop (handle at -120 deg) is at
# the right arm's reach edge (shifted, joint 4 comes within 5 deg of its stop and the drain lift misses by 7 mm); its
# piece ends 1.7 cm off the bowl's center line without it, inside the bowl
HOVER_ABOVE = 0.020  # m: the bowl's floor this far over the water while waiting
RIM_CLEAR = 0.008  # m: the handle's underside and the fingers at least this far over the rim where they cross it
TIPS_OVER_WATER = 0.020  # m: the fingertips at least this far over the water inside the pan
PSIS = tuple(math.radians(a) for a in range(-130, 50, 10))  # handle directions the right hand can hold over the pan
SCOOP_PSI = (-120, -100, -70, -30, 0, 20)  # degrees, tried first for G1..G6: the bowl comes in from the pan's middle
# with its tip toward the wall, the handle turned toward the right arm (checked with the real IK)
# Where the tip's edge stops over the well (each piece slides 2-3 cm on toward the robot): the first three a row near the
# plate's middle, the last three a row 3.5 cm farther out, so the bowl never comes down over a piece already plated.
PLATE_SPOTS = tuple((PLATE_C[0] + dx, PLATE_C[1] + dy) for dx in (-0.005, 0.030) for dy in (0.0, 0.024, -0.024))
HOME_ORIGIN = (PAN_C[0] - 0.040, PAN_C[1] - 0.050, SURFACE_Z0 + HOVER_ABOVE)  # waiting: the near-right of the pan


def spoon_clear(origin, psi: float, tip: float, surface: float = SURFACE_Z0) -> bool:
    """The spoon held so: its bowl inside the pan (5 mm off the wall), the handle's underside and the hand at least
    RIM_CLEAR over the rim where they are over the wall, and the hand's fingertips above the water."""
    R = spoon_rot(psi, tip)
    o = np.asarray(origin, float)
    c = np.asarray(PAN_C, float)
    corners = [o + R @ np.array([sx * BOWL_W / 2, sy * BOWL_L / 2, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]
    if o[2] < PAN_RIM_Z and any(np.hypot(*(p[:2] - c)) > PAN_R - 0.005 for p in corners):
        return False
    for s in np.linspace(0.0, HANDLE_LEN, 49):
        p = o + R @ handle_point(s)
        r = float(np.hypot(*(p[:2] - c)))
        half_t = (NECK_T if s < NECK_LEN else GRIP_T) / 2
        if PAN_R - 0.015 <= r <= PAN_R + 0.020 and p[2] - half_t < PAN_RIM_Z + RIM_CLEAR:
            return False
    tcp = o + R @ GRIP_LOCAL
    a = R @ _HAND_IN_SPOON[:, 2]
    tips = tcp + 0.013 * a  # the pad tips, 13 mm past the tool point along the approach
    r = float(np.hypot(*(tips[:2] - c)))
    if r <= PAN_R + 0.030 and tips[2] < (PAN_RIM_Z + RIM_CLEAR if r > PAN_R - 0.030 else surface + TIPS_OVER_WATER):
        return False
    return True


OFFSETS = tuple((dx, dy) for dy in (0.0, -0.012, 0.012, -0.024, 0.024) for dx in (0.0, -0.012, 0.012))


def plan_scoop(p_xy, first: float | None = None) -> tuple | None:
    """(psi, start origin, end origin, offset) for a piece floating at p_xy: the bowl comes in under it tip first (its
    sides would catch the piece's underside if it came in sideways) from 2.2 cm short of it, and ends with the piece
    inside the bowl (at the bowl's center if it can, else up to 2.4 cm toward its ends or 1.2 cm toward its sides),
    with every point of the move clear (spoon_clear). For each spot in the bowl, the first handle direction that works,
    `first` (rad) and then PSIS in order. None if no move works there."""
    p = np.asarray(p_xy, float)
    z = SURFACE_Z0 - SCOOP_DEPTH
    for dx, dy in OFFSETS:
        for psi in ((first,) if first is not None else ()) + PSIS:
            Rz = rz(psi)
            u = Rz @ np.array([0.0, 1.0, 0.0])  # toward the handle
            # the piece is aimed at (dx, dy + SLIDE_PAST) in the bowl; it is carried on to about (dx, dy)
            end = np.r_[p - (Rz @ np.array([dx, dy + SLIDE_PAST, 0.0]))[:2], z]
            if not spoon_clear(end, psi, SCOOP_TIP) or not spoon_clear(end, psi, LIFT_TIP) or \
                    not spoon_clear(end + np.array([0, 0, SCOOP_DEPTH + 0.02]), psi, LIFT_TIP):
                continue
            start = end + np.r_[u[:2] * (BOWL_L / 2 + dy + SLIDE_PAST + G_SIZE[0] / 2 + 0.022), 0.0]  # tip 2.2 cm short
            above = start + np.array([0, 0, SCOOP_DEPTH + HOVER_ABOVE])
            if all(spoon_clear(q, psi, SCOOP_TIP) for q in (start, (start + end) / 2, above)):
                return psi, start, end, (dx, dy)
    return None


DEMO_SEED = 0  # the demo is scripted for this seed's cooking times (run it with GNOCCHI_SEED=0)
SCOOP_AFTER = 2.5  # s after a piece's cooking time: the bowl starts toward it (it floats up about 2 s after cooking)


def aimed(origin, psi: float, tip: float, share: float = 1.0) -> np.ndarray:
    """The bowl origin to send the hand to so the held bowl ends at `origin` (cancels `share` of HELD_OFFSET)."""
    return np.asarray(origin, float) - share * (spoon_rot(psi, tip) @ HELD_OFFSET)


def _lift_and_plate(n: str, end, psi: float, spot: int, psi_next: float = 0.0, share: float = 0.0,
                    last: bool = False) -> list:
    """From the bowl under piece `n` at `end`: level the bowl as it lifts the piece, lift it out slowly (draining), carry
    it to its spot on the plate, tip it off and come back over the pan's near-right side, the handle already turned
    to `psi_next` (the next piece's handle direction). The wrist turns the handle at up to 90 deg/s."""
    c = FINGER_CLOSED
    at_surface = np.asarray(end, float) + np.array([0.0, 0.0, SCOOP_DEPTH])
    drain = at_surface + np.array([0.0, 0.0, 0.020])
    clear = np.r_[np.asarray(HOME_ORIGIN)[:2], PAN_RIM_Z + 0.025]
    sx, sy = PLATE_SPOTS[spot % len(PLATE_SPOTS)]

    def at_edge(tip, up):  # the bowl's origin when tipped `tip` with its tip's lower edge `up` over the spot
        return np.array([sx, sy, PLATE_TOP + up]) - spoon_rot(DUMP_PSI, tip) @ _TIP_EDGE

    over = at_edge(0.0, DUMP_START_UP)
    high = np.r_[over[:2], PAN_RIM_Z + 0.025]
    home_high = np.r_[np.asarray(HOME_ORIGIN)[:2], PAN_RIM_Z + 0.025]
    steps = [hold(RISE_S, aimed(at_surface, psi, LIFT_TIP, share), psi, LIFT_TIP, c,
                  f"{n}: level the bowl and lift it to the surface"),
             hold(0.020 / LIFT_SPEED, aimed(drain, psi, LIFT_TIP, share), psi, LIFT_TIP, c,
                  f"{n}: lift 2 cm slowly, draining"),
             hold(float(np.linalg.norm(clear - drain)) / 0.2 + 0.3 + abs(psi - DUMP_PSI) / TURN_RATE, clear, DUMP_PSI,
                  0.0, c, f"{n}: to the near side and up over the rim, turning the handle"),
             hold(float(np.linalg.norm(high - clear)) / CARRY_SPEED + 0.3, high, DUMP_PSI, 0.0, c,
                  f"{n}: carry to the plate"),
             hold(0.5, over, DUMP_PSI, 0.0, c, f"{n}: down over its spot")]
    # tip it about the tip's edge (the edge stays put, coming down to 3 mm over the well by 36 degrees)
    for k in range(1, 6):  # in 9-degree steps (the edge strays under 0.5 mm between them)
        a = DUMP_TIP * k / 5
        up = DUMP_TIP_UP + (DUMP_START_UP - DUMP_TIP_UP) * max(0.0, 1.0 - a / math.radians(36.0))
        steps.append(hold(DUMP_TIP_S / 5, at_edge(a, up), DUMP_PSI, a, c, f"{n}: tip it off ({math.degrees(a):.0f} deg)"))
    lifted = at_edge(DUMP_TIP, DUMP_TIP_UP + 0.030)
    out = steps + [hold(0.3, at_edge(DUMP_TIP, DUMP_TIP_UP), DUMP_PSI, DUMP_TIP, c, f"{n}: let it slide off"),
                   hold(0.4, lifted, DUMP_PSI, DUMP_TIP, c, f"{n}: up from the plate, still tipped"),
                   hold(0.7, high, DUMP_PSI, 0.0, c, f"{n}: level, up from the plate")]
    if last:  # the spoon goes straight back to the counter (demo())
        return out + [hold(0.01, high, DUMP_PSI, 0.0, c, f"{n}: done")]
    return out + [hold(float(np.linalg.norm(home_high - high)) / CARRY_SPEED + 0.3 + abs(psi_next - DUMP_PSI) / TURN_RATE,
                       home_high, psi_next, 0.0, c, "back over the pan")]


def scoop_schedule() -> list:
    """[(start time s, piece, (x, y))]: each piece in the order it cooks at DEMO_SEED, taken where it starts on the
    pan floor (it floats straight up), no earlier than SCOOP_AFTER after its cooking time."""
    times = trial_cook_times(DEMO_SEED)
    return [(times[i] + SCOOP_AFTER, G_NAMES[i], G_START[i][:2]) for i in sorted(range(6), key=lambda k: times[k])]


def demo():
    """Open loop, scripted for GNOCCHI_SEED=0 (its cooking times are fixed constants of this file); it reads no object
    state at run time. The right hand takes the spoon by its grip, lifts it and holds the bowl over the pan's near
    side. For each piece, in cooking order and once it has floated up, it lowers the bowl 2.3 cm under the water, its tip
    2.2 cm short of where the piece rose, tip first and 13 degrees down, slides it under at up to 3 cm/s, levels the
    bowl, lifts it out at 2.5 cm/s for the first 2 cm so the water drains, carries it to the plate and tips it off onto
    its own spot. Then it lays the spoon back down on the counter where it was, lets go and backs off. Once the last
    piece is off the spoon, the left hand takes the burner's knob by its bar and turns it back to OFF."""
    s0 = np.asarray(SPOON_START, float)
    c, o = FINGER_CLOSED, FINGER_OPEN
    P0 = REST_PSI
    approach = spoon_rot(P0, REST_TIP) @ _HAND_IN_SPOON[:, 2]
    g = s0 + spoon_rot(P0, REST_TIP) @ GRIP_LOCAL
    back = g - 0.08 * approach

    def at_grip(d, pos, finger, label):
        w = hold(d, s0, P0, REST_TIP, finger, label)
        return kit.Tilted(float(d), tuple(float(v) for v in pos), finger, label, yaw=w.yaw, roll=w.roll, pitch=w.pitch)

    home_high = np.r_[np.asarray(HOME_ORIGIN)[:2], PAN_RIM_Z + 0.025]
    plan = [at_grip(2.0, back, o, "over the grip"),
            at_grip(1.5, g, o, "around the grip"),
            at_grip(1.0, g, c, "grip"),
            hold(1.0, s0 + np.array([0.0, 0.0, SPOON_LIFT]), P0, REST_TIP, c, "lift the spoon off the counter"),
            hold(1.2, s0 + np.array([0.0, 0.0, SPOON_LEVEL_Z]), P0, 0.0, c, "level the bowl"),
            hold(1.8, home_high, 0.0, 0.0, c, "over the pan"),
            hold(1.0, HOME_ORIGIN, 0.0, 0.0, c, "wait over the pan")]
    t = sum(w.duration for w in plan)
    sched = scoop_schedule()
    scoops = [plan_scoop(xy, math.radians(SCOOP_PSI[G_NAMES.index(n)])) for _, n, xy in sched]
    psi_home = 0.0  # the handle's direction while over the pan's near side
    t_plated = 0.0  # when the last piece has slid off the spoon
    for spot, ((t_go, n, xy), (psi, start, end, _)) in enumerate(zip(sched, scoops)):
        if t_go > t:
            plan.append(hold(max(t_go - t, 1.0), HOME_ORIGIN, psi_home, 0.0, c, f"wait for {n} to float up"))
        above = start + np.array([0.0, 0.0, SCOOP_DEPTH + HOVER_ABOVE])
        slide = float(np.linalg.norm(end - start))
        a = AIM[G_NAMES.index(n)]
        psi_next = scoops[spot + 1][0] if spot + 1 < len(scoops) else 0.0
        steps = [hold(0.6 + abs(psi - psi_home) / TURN_RATE, aimed(above, psi, SCOOP_TIP, a), psi, SCOOP_TIP, c,
                      f"{n}: over the start"),
                 hold((SCOOP_DEPTH + HOVER_ABOVE) / 0.05, aimed(start, psi, SCOOP_TIP, a), psi, SCOOP_TIP, c,
                      f"{n}: bowl under the water"),
                 # smoothstep peaks at 1.5x the mean speed: this keeps the peak at SLIDE_SPEED
                 hold(1.5 * slide / SLIDE_SPEED, aimed(end, psi, SCOOP_TIP, a), psi, SCOOP_TIP, c,
                      f"{n}: slide under")]
        steps += _lift_and_plate(n, end, psi, spot, psi_next, a, last=spot == len(sched) - 1)
        psi_home = psi_next
        t_plated = sum(w.duration for w in plan) + sum(w.duration for w in steps[:-3])
        plan += steps
        t = sum(w.duration for w in plan)
    plan += [hold(2.0, s0 + np.array([0.0, 0.0, SPOON_LEVEL_Z]), P0, 0.0, c, "over its place on the counter"),
             hold(1.2, s0 + np.array([0.0, 0.0, SPOON_LIFT]), P0, REST_TIP, c, "tip it as it lies"),
             hold(1.0, s0 + np.array([0.0, 0.0, 0.002]), P0, REST_TIP, c, "down on the counter"),
             hold(0.8, s0 + np.array([0.0, 0.0, 0.002]), P0, REST_TIP, o, "let go"),
             at_grip(1.2, back, o, "back off")]
    return {"right": plan, "left": knob_plan(t_plated)}


SPOON_LIFT = 0.05  # m: the spoon is lifted (or set down) this far above where it lies before its bowl is turned level
SPOON_LEVEL_Z = PAN_RIM_Z + 0.030 - TOP  # m over the counter: the bowl's origin while it is turned level beside the hob
KNOB_TCP_UP = KNOB_SKIRT_H + 0.016  # m over the hob: the left hand's tool point on the knob's bar (fingertips 3 mm over
# the skirt, the pads on the bar's upper 1.5 cm)
KNOB_YAW0 = -math.pi / 2  # the hand's yaw on the bar at the start (fingers closing across it); it turns with the knob
KNOB_PAST = math.radians(3.0)  # the turn is aimed 3 degrees past OFF, into the stop, so the knob clicks in


def knob_plan(t_start: float) -> list:
    """The left hand: waits where it starts until `t_start`, then comes down over the knob with its fingers across the
    bar, grips it, turns it counterclockwise (seen from above) back to OFF in three steps, lets go and goes home."""
    o, c = FINGER_OPEN, FINGER_CLOSED
    x, y = KNOB_C
    z = HOB_TOP + KNOB_TCP_UP

    def at(d, h, q, f, label):
        return kit.Tilted(float(d), (float(x), float(y), float(z + h)), f, label, yaw=float(KNOB_YAW0 + q - KNOB_START))

    plan = [kit.Home(float(t_start), label="wait"),
            at(2.0, 0.08, KNOB_START, o, "over the knob"),
            at(1.2, 0.0, KNOB_START, o, "fingers round the knob's bar"),
            at(1.0, 0.0, KNOB_START, c, "grip the bar")]
    for k in range(1, 4):
        q = KNOB_START + (KNOB_PAST - KNOB_START) * k / 3
        plan.append(at(0.7, 0.0, q, c, f"turn it toward OFF ({math.degrees(q):.0f} deg)"))
    plan += [at(0.3, 0.0, KNOB_PAST, c, "hold at OFF"),
             at(0.8, 0.0, 0.0, o, "let go of the knob"),
             at(0.8, 0.08, 0.0, o, "up from the knob"),
             kit.Home(2.0)]
    return plan
