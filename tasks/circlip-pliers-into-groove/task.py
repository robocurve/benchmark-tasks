"""Fit an external retaining ring (circlip, DIN 471 for a 40 mm shaft) into its groove with circlip pliers.

A loose shaft-and-gear assembly stands on end on the bench, not clamped: a 40 mm steel shaft with a spur gear against
a shoulder, and just above the gear the groove for the retaining ring that keeps the gear on. The yellow zinc-plated
spring-steel ring lies on the shaft's top end face and the circlip pliers (90-degree bent tips, external type:
squeezing the handles spreads the tips) lie on a rubber rest block.

The job is a mechanic's: steady the shaft (a part this size is held in one hand, not put in a vise), put the pliers'
tips in the ring's two lug holes, spread the ring just enough to clear the shaft, slide it down to the groove, let it
close into the groove, turn it in the groove to check it is seated, and put the pliers down. A spring-steel ring
spread past the maker's maximum installation diameter takes a permanent set: it sits loose in its groove and can walk
out under load. A ring let go short of the groove grips the shaft in the wrong place, and one let go tilted lands
half in the groove (cocked).

The ring is modeled by ringkit (see ringkit.py): its axial position, turn and opening are joints of the shaft
assembly, driven each step from the pliers' tips; the shaft assembly and the pliers are loose bodies on the bench.
Closed onto the shaft the ring grips it, so turning it with the pliers puts a torque on the shaft (GRIP_N, MU below).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import ringkit as rk  # noqa: E402  (the ring on the shaft; imports yamkit)
import yamkit as kit  # noqa: E402  (spawns the modeled props and the moving parts; nothing on disk changes)
import guardkit as gk  # noqa: E402  (hand contact and the debounced twist latch)
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.station import TABLE_CENTER, TABLE_SIZE  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the retaining ring into the groove on the shaft with the circlip pliers, then put the pliers down."

# --- The shaft assembly and its ring (m). DIN 471 external retaining ring for a 40 mm shaft.
SX, SY = 0.36, -0.06  # where the shaft stands on the bench
END_H = 0.149  # the shaft's top end face above the bench (the shaft is 14.9 cm long, standing on its lower end)
SHAFT_D = 0.040
FREE_ID = 0.0365  # the ring's inner diameter at rest (DIN 471, d3 for a 40 mm shaft)
GROOVE_D = 0.0375  # groove bottom diameter (DIN 471, d2)
GROOVE_W = 0.00185  # groove width (DIN 471, m)
RING_T = 0.00175  # ring thickness (DIN 471, s)
RING_W = 0.0045  # the ring's radial section
GEAR_FACE = 0.03903  # the gear's top face, m below the shaft's end face: the groove's lower wall sits on it
GEAR_T = 0.016  # the gear's thickness
SHOULDER_D, SHOULDER_T = 0.046, 0.006  # the shaft's shoulder the gear sits on
GEAR_Z = END_H - GEAR_FACE  # the gear's top face above the bench (11.0 cm)
SHOULDER_Z = GEAR_Z - GEAR_T  # the shoulder's top face (9.4 cm): plain 40 mm shaft below it, where a hand holds it
GROOVE_AT = GEAR_FACE - GROOVE_W / 2  # the groove's middle, below the end face
LUG_GAP = 0.012  # the ring's lug holes, center to center, at rest
LUG_OUT = 0.0058  # the lug holes' centers beyond the ring's inner edge
SHAFT_MASS = 1.6  # kg: 14.9 cm of 40 mm steel bar (1.45 kg) plus the gear (0.15 kg)
# What the ring pushes on the shaft (estimates, see ringkit's reaction loads). Closed in its groove the ring grips the
# groove bottom with about 270 N all round: the maker's detaching speed for this ring (Seeger A40: 14,000 rpm) is where
# its 6.03 g, spun at about 21 mm radius, just lifts off it, and m * w^2 * r = 0.00603 * 1466^2 * 0.021 = 272 N.
# With mu 0.12 (lightly oiled steel on steel) it takes 0.12 * 270 N * 18.75 mm = 0.61 N*m to turn it in the groove:
# 25 N at the lug holes (24.55 mm out), which the ring passes to the shaft. Spread clear of the shaft it pushes nothing.
GRIP_N = 270.0
MU = 0.12

# --- Scoring thresholds (each with its reason).
LIMIT_ID = 0.0410  # m: the largest bore the ring may be spread to while fitting it, 1 mm (2.5%) over the shaft, the
#                   usual maximum installation spread for a 40 mm DIN 471 ring; past it the spring steel takes a set
CLEAR = 0.0002  # m (diametral): the spread ring must clear the 40 mm shaft by 0.2 mm to slide along it
TILT_LIMIT = 4.0  # deg: let go tilted more than this, a 49 mm-wide ring's far side sits 3.4 mm off the groove plane,
#                   nearly twice the 1.85 mm groove width, so that side closes onto the shaft beside the groove
TWIST_STEPS = 2  # control steps in a row (0.1 s) past TWIST_LIMIT for the twist to count: a one-step PhysX pop of the
#                 pliers doesn't count
TWIST_LIMIT = 10.0  # deg: ends twisted apart along the shaft by more than this (2.1 mm across the 12 mm lug spacing)
#                    counts as a ring bent out of its plane (a modeled stand-in; a real trial checks it lies flat)
ON_SHAFT = 0.005  # m: the ring's mid-plane this far past the end face is on the shaft's 40 mm land (past the 1.5 mm
#                  chamfer with room to spare): spread over the shaft end without being scrapped
CHECK_DEG = 15.0  # deg: turning the ring this far in its groove (5 mm at its rim) is the check that it is free all
#                   round in the groove, i.e. seated, not caught on the shaft or cocked
UPRIGHT_DEG = 10.0  # deg: the shaft's axis within this of vertical is standing on its end (knocked over it lies at 90)
DOWN_Z = 0.05  # m: the pliers' grip point within 5 cm of the bench top (on the 3 cm rest block or the bench) = down
DOWN_LOW = 0.01  # m: and no more than 1 cm below the bench top (the handles are 1.2 cm tall): lower is on the floor
BASE_LOW = 0.01  # m: the shaft's lower end no more than 1 cm off the bench top: standing on the bench, not lifted
TABLE_X = (TABLE_CENTER[0] - TABLE_SIZE[0] / 2, TABLE_CENTER[0] + TABLE_SIZE[0] / 2)  # the bench's footprint
TABLE_Y = (TABLE_CENTER[1] - TABLE_SIZE[1] / 2, TABLE_CENTER[1] + TABLE_SIZE[1] / 2)
STILL = 0.001  # m per control step: the pliers not moving

BRIGHT = kit.Look((0.80, 0.80, 0.81), 0.25, metallic=0.9)
GEAR = kit.Look((0.42, 0.44, 0.47), 0.5, metallic=0.5)  # hobbed steel, darker than the ground shaft
RING_LOOK = kit.Look((0.95, 0.74, 0.08), 0.45, metallic=0.2)  # yellow zinc-chromate plating, a common ring finish;
#                                                             bright so the ring shows against the steel at 320 x 176
GRIP = kit.Look((0.80, 0.10, 0.08), 0.6)
JAW = kit.Look((0.25, 0.26, 0.28), 0.4, metallic=0.7)
RUBBER = kit.Look((0.12, 0.12, 0.12), 0.9)

RING = rk.Ring("clip", end=(0.0, 0.0, END_H), axis="-Z", shaft_d=SHAFT_D, free_id=FREE_ID, thickness=RING_T,
               width=RING_W, length=END_H - 0.05,
               grooves=(rk.Groove(at=GROOVE_AT, width=GROOVE_W, diameter=GROOVE_D),), stop=GEAR_FACE,
               limit_id=LIMIT_ID, clear=CLEAR, lug_gap=LUG_GAP, lug_out=LUG_OUT, lug_angle=math.pi,
               tilt_limit=TILT_LIMIT, grip_n=GRIP_N, mu=MU, look=RING_LOOK, hide=END_H + 0.02)  # placeholder put away inside the bench


def _shaft():
    """The loose shaft assembly, in its own frame: origin at the center of the shaft's lower end (on the bench), z up
    the shaft. A free body of 1.6 kg: nothing holds it but the bench under it and whatever hand steadies it."""
    g0, g1 = GEAR_Z, GEAR_Z + GROOVE_W  # the groove, just above the gear
    meshes = [kit.lathe([(0.0, 0.0), (SHAFT_D / 2 - 0.001, 0.0), (SHAFT_D / 2, 0.001),
                         (SHAFT_D / 2, SHOULDER_Z - SHOULDER_T), (SHOULDER_D / 2, SHOULDER_Z - SHOULDER_T),
                         (SHOULDER_D / 2, SHOULDER_Z), (SHAFT_D / 2, SHOULDER_Z), (SHAFT_D / 2, g0),
                         (GROOVE_D / 2, g0), (GROOVE_D / 2, g1), (SHAFT_D / 2, g1),
                         (SHAFT_D / 2, END_H - 0.0015), (SHAFT_D / 2 - 0.0015, END_H), (0.0, END_H)], BRIGHT, n=48)]
    meshes.append(kit.cylinder((0, 0, SHOULDER_Z + GEAR_T / 2), 0.026, GEAR_T, GEAR, n=48))  # the gear's body
    for k in range(30):  # its teeth
        a = 2 * math.pi * k / 30
        meshes.append(kit.cylinder((0.0262 * math.cos(a), 0.0262 * math.sin(a), SHOULDER_Z + GEAR_T / 2), 0.0019,
                                   GEAR_T - 0.0005, GEAR, n=10))
    colliders = [kit.Collider((0, 0, END_H / 2), radius=SHAFT_D / 2, height=END_H),
                 kit.Collider((0, 0, SHOULDER_Z - SHOULDER_T / 2), radius=SHOULDER_D / 2, height=SHOULDER_T),
                 kit.Collider((0, 0, SHOULDER_Z + GEAR_T / 2), radius=0.028, height=GEAR_T)]
    return rk.free_tool(kit.Mechanism("shaft", pos=(SX, SY, TOP + 0.0005), size=(0.056, 0.056, END_H),
                                      mass=SHAFT_MASS, meshes=tuple(meshes), colliders=tuple(colliders),
                                      links=rk.links(RING, mass=0.002)), com=(0.0, 0.0, 0.072))


# --- The circlip pliers (m), in their own frame: origin at the grip point between the handles at mid-height, x toward
# the tips. The base half carries handle A (+y) and the jaw that crosses to tip A; the other half (link "jaw") slides
# along y: squeezing the handles together by q spreads the tips apart by q (handle gap 40 mm - q, tip gap 12 mm + q).
TIP_X = 0.075  # the tips' distance from the grip point
TIP_LEN = 0.017  # the bent tips reach this far below the jaws' underside
TIP_END = -0.004 - TIP_LEN  # z of the tip ends in the pliers' frame
HANDLE_GAP = 0.040  # outer faces of the handles at rest
SQUEEZE_MAX = 0.018  # the handles' stop
PLIERS_MASS, JAW_MASS = 0.08, 0.05
SPRING_K, SPRING_PRE = 100.0, 1.0  # N/m and N: the handles' return spring (1 N at rest, 2.8 N fully squeezed)


def _jaw_path(sign):
    """A jaw's center line from its handle (y = sign * 15 mm) across to its tip (y = -sign * 6 mm)."""
    return [(0.030, sign * 0.015, 0.0), (0.045, sign * 0.012, 0.0), (0.060, -sign * 0.002, 0.0),
            (TIP_X, -sign * 0.006, 0.0)]


def _pliers():
    hz = 0.012  # the handles' height
    tip_a = (TIP_X, -0.006, -0.004 - TIP_LEN / 2)
    tip_b = (TIP_X, 0.006, -0.004 - TIP_LEN / 2)
    base_meshes = (kit.box((-0.005, 0.015, 0.0), (0.080, 0.010, hz), GRIP),
                   kit.band([(x, y, z - 0.0022) for x, y, z in _jaw_path(1)], 0.0044, 0.005, JAW, normal=(0, 0, 1)),
                   kit.cylinder(tip_a, 0.0013, TIP_LEN, JAW),
                   kit.cylinder((0.0525, 0.0045, 0.0), 0.0035, 0.010, BRIGHT))  # the pivot rivet
    jaw_meshes = (kit.box((-0.005, -0.015, 0.0), (0.080, 0.010, hz), GRIP),
                  kit.band([(x, y, z + 0.0022) for x, y, z in _jaw_path(-1)], 0.0044, 0.005, JAW, normal=(0, 0, 1)),
                  kit.cylinder(tip_b, 0.0013, TIP_LEN, JAW))
    jaw = kit.Link("jaw", "prismatic", "Y", (0.0, 0.0, 0.0), lower=0.0, upper=SQUEEZE_MAX, start=0.0, mass=JAW_MASS,
                   meshes=jaw_meshes, colliders=(kit.Collider((-0.005, -0.015, 0.0), size=(0.080, 0.010, hz)),
                                                 kit.Collider(tip_b, radius=0.0013, height=TIP_LEN)),
                   damping=0.0, law=rk.HandleSpring(rest=-SPRING_PRE / SPRING_K, k=SPRING_K, damp=1.0))
    return rk.free_tool(kit.Mechanism(
        "pliers", pos=PLIERS_POS, size=(0.125, 0.04, 0.03), mass=PLIERS_MASS, meshes=base_meshes,
        colliders=(kit.Collider((-0.005, 0.015, 0.0), size=(0.080, 0.010, hz)),
                   kit.Collider(tip_a, radius=0.0013, height=TIP_LEN)),
        links=(jaw,)), com=(0.0, 0.0, 0.0))  # weight at the grip point, so the pliers don't pitch in the gripper


REST = (0.256, -0.22, 0.015)  # the rubber rest block's center (8 x 3 x 3 cm)
PLIERS_POS = (REST[0] + 0.005, REST[1], TOP + 0.030 + 0.006)  # lying on the block, tips hanging past its end
TIPS = rk.TipPair(("pliers", (TIP_X, -0.006, TIP_END)), ("pliers/jaw", (TIP_X, 0.006, TIP_END)), catch=0.004)
KEY = rk.attach("shaft", RING, TIPS)

OBJECTS = [_shaft(), _pliers(),
           kit.Prop("rest", pos=(REST[0], REST[1], TOP + REST[2]), size=(0.08, 0.03, 0.03),
                    meshes=(kit.box((0, 0, 0), (0.08, 0.03, 0.03), RUBBER),),
                    colliders=(kit.Collider((0, 0, 0), size=(0.08, 0.03, 0.03)),))]

# The pliers' outline for hand contact, in the base half's frame: both handles (the jaw half's handle slides up to
# 18 mm toward the other, so its box spans that travel), the crossed jaws and the bent tips.
PLIERS_PARTS = (gk.Block((-0.005, 0.015, 0.0), (0.080, 0.010, 0.012)),
                gk.Block((-0.005, -0.006, 0.0), (0.080, 0.028, 0.012)),
                gk.Block((0.055, 0.0, -0.002), (0.045, 0.034, 0.005)),
                gk.Block((TIP_X, 0.0, -0.004 - TIP_LEN / 2), (0.003, 0.034, TIP_LEN)))


# --- What happened during the run. success() and score() are called after every control step; ringkit keeps the
# ring's history, and the twist latch and the pliers' last position are kept here (all reset with the scene).
TWISTED = gk.Latch(steps=TWIST_STEPS)


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.pliers = None  # the pliers' grip point at the last step
        self.pliers_still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    p = np.asarray(objects["pliers"].pos, float)
    _H.pliers_still = _H.pliers is not None and np.linalg.norm(p - _H.pliers) < STILL
    _H.pliers = p


def _watch(objects):
    """Once per control step (success() and score() both call it; the latch ignores repeats within a step)."""
    TWISTED.feed(objects, abs(rk.state(KEY).twist) > TWIST_LIMIT)


def _pliers_down(objects):
    """Put down: out of the ring, no gripper touching them, lying still over the bench with the grip point from 1 cm
    below to 5 cm above its top (not dropped on the floor, not left hanging on the shaft)."""
    x, y, z = (float(v) for v in objects["pliers"].pos)
    return (not rk.engaged(objects, KEY) and not gk.touching(objects, "pliers", PLIERS_PARTS) and _H.pliers_still
            and -DOWN_LOW <= z - TOP < DOWN_Z and TABLE_X[0] < x < TABLE_X[1] and TABLE_Y[0] < y < TABLE_Y[1])


def _shaft_standing(objects):
    """The shaft assembly standing on its end on the bench: axis within 10 degrees of vertical, its lower end within
    1 cm of the bench top, over the bench (not knocked over, not on the floor)."""
    o = objects["shaft"]
    axis = rk._rotm(o.quat_xyzw) @ np.array([0.0, 0.0, 1.0])
    x, y, z = (float(v) for v in o.pos)
    return (axis[2] >= math.cos(math.radians(UPRIGHT_DEG)) and abs(z - TOP) <= BASE_LOW
            and TABLE_X[0] < x < TABLE_X[1] and TABLE_Y[0] < y < TABLE_Y[1])


def _closed_in_groove(objects):
    """Seated for the 0.5 and 0.75 rungs: in the groove, closed onto its bottom and not cocked, with or without the
    pliers' tips still in the lug holes, as long as they are not spreading it (a ring the tips hold open is not
    closed). 1.0 also needs the pliers out of it."""
    g, grip = rk.groove(objects, KEY), rk.state(KEY).grip
    bottom = RING.grooves[g].diameter / 2 + RING.seat_tol if g is not None else 0.0
    return (g is not None and not rk.cocked(objects, KEY) and rk.inner_d(objects, KEY) / 2 <= bottom
            and (grip is None or grip.r is None or grip.r <= bottom))


def _score(objects):
    if rk.overspread(objects, KEY) or TWISTED.on:
        return 0.0  # the ring has taken a set or been bent: scrap, whatever happened next
    if not _closed_in_groove(objects):
        # on the shaft (spread over the end within the limit) but short of the groove, cocked or held open
        return 0.25 if rk.position(objects, KEY) >= ON_SHAFT else 0.0
    if rk.turned_deg(objects, KEY) < CHECK_DEG - 1e-6:
        return 0.5  # seated, but never turned in its groove to check it
    if not (_pliers_down(objects) and _shaft_standing(objects)):
        return 0.75  # seated and checked; the pliers still in hand or not on the bench, or the shaft knocked over
    return 1.0


def success(objects):
    """The ring seated and checked, the pliers put down and the shaft left standing. Called first each step, so it
    also records the history."""
    _update(objects)
    _watch(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the ring was ever spread past 41.0 mm bore or its ends twisted more than 10 degrees out of plane for 2
    control steps in a row (scrap; irreversible). Otherwise 0.25 with the ring on the shaft (its mid-plane 5 mm or
    more past the end face) but not seated; 0.5 seated (in the groove, closed onto its 37.5 mm bottom, not cocked;
    the pliers' tips may still be in it, not spreading it); 0.75 seated and turned at least 15 degrees net in the
    groove; 1.0 that plus the pliers put down (out of the ring, no gripper touching them, still, on the bench) and the
    shaft standing upright on the bench.
    Doing nothing scores 0."""
    _watch(objects)
    return _score(objects)


# --- Demo. Finger targets are half the gap between the pads (FINGER_OPEN = 94 mm open).
def _finger(gap):
    return -gap / 2


F_HOLD = _finger(0.037)  # a light grip on the handles (3 mm squeeze: tips 15 mm apart)
F_SPREAD = _finger(0.0262)  # squeezed to spread the ring to about 40.6 mm bore (handles' spring takes ~1 mm)
F_SHAFT = _finger(0.034)  # the left hand closed on the 40 mm shaft (the pads stop on it; the rest is grip force)
LUG_R = FREE_ID / 2 + LUG_OUT  # the lug holes' distance from the shaft's axis
HOLD_Z = 0.036  # the left hand's tool point above the bench: the hand clears the bench by 1.2 cm and the shoulder by 6 mm
SIDE = dict(yaw=-math.pi / 2, roll=-1.2)  # the left hand pointing along -y, 21 degrees down, fingers closing along x


def demo():
    """Left arm: hold the shaft from the side, low on its plain part, while the right arm works. Right arm: pick the
    pliers off the rest, put the tips in the ring's lug holes, squeeze to spread it to about 40.6 mm, slide it down
    onto the gear face, let it close into the groove, turn it 15 degrees each way in the groove, lift the pliers off and
    lay them back on the rest. The left hand lets go once the tips are out, and both arms go home (not scored)."""
    px, py, pz = PLIERS_POS[0], PLIERS_POS[1], PLIERS_POS[2]
    ex, ey = SX - LUG_R - TIP_X, SY  # grip point with the tips over the lug holes
    ez = TOP + END_H + 0.0003 - TIP_END  # tips 1.45 mm into the holes of the ring lying on the end face
    sz = TOP + END_H - (GEAR_FACE - 0.0005) - TIP_END  # tips 0.5 mm above the gear face: the ring's far face there

    def orbit(deg, z, finger, label, t):  # the grip point turned about the shaft's axis, the wrist with it
        a = math.radians(deg)
        dx, dy = ex - SX, ey - SY
        return Waypoint(t, (SX + dx * math.cos(a) - dy * math.sin(a), SY + dx * math.sin(a) + dy * math.cos(a), z),
                        finger, label, a)

    work = [Waypoint(1.5, (px, py, pz + 0.08), FINGER_OPEN, "over the pliers"),
            Waypoint(1.5, (px, py, pz), FINGER_OPEN, "around the handles"),
            Waypoint(1.0, (px, py, pz), F_HOLD, "grip the handles"),
            Waypoint(1.5, (px, py, ez + 0.03), F_HOLD, "lift the pliers"),
            Waypoint(2.0, (ex, ey, ez + 0.02), F_HOLD, "over the ring's lug holes"),
            Waypoint(1.5, (ex, ey, ez), F_HOLD, "tips into the lug holes"),
            Waypoint(1.5, (ex, ey, ez), F_SPREAD, "spread the ring"),
            Waypoint(4.0, (ex, ey, sz), F_SPREAD, "slide it down onto the gear"),
            Waypoint(1.0, (ex, ey, sz), F_HOLD, "let it close into the groove"),
            orbit(8, sz, F_HOLD, "turn it in the groove", 1.0), orbit(15, sz, F_HOLD, "turn it in the groove", 1.0),
            orbit(0, sz, F_HOLD, "turn it back", 1.5), orbit(-15, sz, F_HOLD, "turn it the other way", 1.5),
            orbit(0, sz, F_HOLD, "turn it back", 1.5),
            Waypoint(1.0, (ex, ey, sz + 0.03), F_HOLD, "lift the tips out"),
            Waypoint(1.5, (ex, ey, ez + 0.03), F_HOLD, "up off the shaft")]
    put_away = [Waypoint(2.0, (px, py, pz + 0.08), F_HOLD, "over the rest"),
                Waypoint(1.5, (px, py, pz + 0.002), F_HOLD, "lay the pliers on the rest"),
                Waypoint(0.8, (px, py, pz + 0.002), FINGER_OPEN, "let go"),
                Waypoint(1.0, (px, py, pz + 0.08), FINGER_OPEN, "back away"),
                kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    right = work + put_away
    hz = TOP + HOLD_Z
    left = [kit.Tilted(2.0, (SX, SY + 0.12, hz + 0.04), FINGER_OPEN, "beside the shaft", **SIDE),
            kit.Tilted(1.5, (SX, SY + 0.12, hz), FINGER_OPEN, "level with the shaft's plain part", **SIDE),
            kit.Tilted(1.5, (SX, SY, hz), FINGER_OPEN, "fingers either side of the shaft", **SIDE),
            kit.Tilted(1.0, (SX, SY, hz), F_SHAFT, "hold the shaft", **SIDE)]
    t_out = sum(w.duration for w in work)  # the tips are up off the shaft
    left += [kit.Tilted(t_out - sum(w.duration for w in left), (SX, SY, hz), F_SHAFT,
                        "steady it while the ring goes on", **SIDE),
             kit.Tilted(1.0, (SX, SY, hz), FINGER_OPEN, "let go of the shaft", **SIDE),
             kit.Tilted(1.5, (SX, SY + 0.12, hz), FINGER_OPEN, "hand back off the shaft", **SIDE),
             kit.Home(2.0)]
    t_left, t_right = sum(w.duration for w in left), sum(w.duration for w in right)
    left.append(kit.Home(max(t_right - t_left, 1.5), label="hold at home"))
    return {"left": left, "right": right}
