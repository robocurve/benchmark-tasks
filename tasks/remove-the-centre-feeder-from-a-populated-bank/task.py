"""The front of a desktop pick-and-place machine (an Opulo LumenPnP) on a bench. Its front frame rail, a black 20 x 20 mm
aluminium extrusion, runs straight away from the robot; the machine's bed, frame and parked gantry are to the robot's
right, the front left leg and side frame nearer the robot. Three black 8 mm tape feeders sit side by side on the rail in
its first three slots, each hooked over the rail and clipped to the yellow slot blade on the rail's left face, each
with a white 7 inch component reel hanging from it on the left and its tape threaded into the feeder. The units are
identical and touch-close: about 0.8 mm between feeder bodies and 2.8 mm between reels. To the left of the machine lies
a blue service mat. Nobody is in the scene.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
import tilt  # noqa: E402
import observation  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

# Millimetre-scale catches and a 0.8 mm packing gap need a contact envelope
# smaller than that gap (the generic kit uses 5 mm). Rest offsets remain zero.
parts.SOLVER_ITERATIONS = (64, 8)
parts.MAX_DEPENETRATION = 0.10
lots.SETTLE_S = 2.0  # arms at READY; no policy actions or scorer bypass during a trial
PACKED_CONTACT = 0.0002

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6
D = math.radians

# -- colours ------------------------------------------------------------------------------------------------------------
RAIL_BLACK = (0.12, 0.12, 0.13)
FEEDER_BLACK = (0.07, 0.07, 0.08)
YELLOW = (0.95, 0.72, 0.08)
REEL_WHITE = (0.90, 0.91, 0.92)
TAPE = (0.20, 0.17, 0.14)
MAT_BLUE = (0.16, 0.33, 0.62)
LABEL = (0.95, 0.80, 0.25)


def _qy(a: float) -> tuple[float, float, float, float]:
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


# -- geometry, in a feeder's own frame (metres) ---------------------------------------------------------------------------
# u: inward, from the reel side toward the machine's bed; w: across, along the rail (the slot pitch); z: up. Origin:
# the rail's centre line at the slot centre. From the maker's CAD: the LumenPnP front frame rail is a 20 x 20 mm
# V-slot extrusion whose centre line is 110 mm above the bench (machine on its leg extensions); the front slot blades
# stand 28 mm out from its outer face, 45 mm tall; slots sit at a 15.8 mm pitch; the 8 mm feeder frame is 164.8 mm
# long and 82.8 mm tall (feeder repository commit c9869fe), 15 mm wide, reaching 70 mm inward from the rail centre to
# just short of the bed; modelled 73 mm tall, without the bulge under the drive gear. The hook, clip and lever shapes
# below are a simplified copy sized from the maker's photos.
PITCH = 0.0158  # slot pitch (maker CAD: 25 front slots on two blades, 103 to 498 mm along the rail)
W2 = 0.0075  # half the feeder width
Z_R = 0.110  # rail centre height above the bench

RAIL_CORE = ((-0.010, 0.010), (-0.010, 0.006))  # (u range, z range)
RAIL_TOP = [((-0.010, -0.0025), (0.006, 0.010)), ((0.0025, 0.010), (0.006, 0.010))]  # the top-face slot, 5 x 4 mm
BLADE = ((-0.038, -0.010), (-0.035, 0.010))  # slot blade on the rail's outer face (u range, z range)
PEG = ((-0.0435, -0.038), (-0.016, -0.010), 0.0025)  # alignment peg on the blade, one per slot

BODY = [((-0.052, -0.001), (0.01005, 0.040)), ((-0.001, 0.070), (0.015, 0.040)),  # over the blade and rail
        ((-0.095, -0.052), (-0.033, 0.030))]  # the frame's outer part, in front of the blade
CLIP = ((-0.052, -0.046), (-0.030, 0.01005))
PRONG = ((-0.046, -0.0395), (-0.030, 0.01005), (0.0026, 0.0075))  # two prongs, a close fit either side of the peg
BEAR = ((-0.0395, -0.03805), (-0.026, -0.022), (0.0026, 0.0075))  # the pads that bear on the blade's face
BRIDGE = ((-0.046, -0.0395), (-0.0099, 0.01005), 0.0026)
STEM = ((-0.0010, 0.0010), (0.0083, 0.015))
NOSE = ((0.0, 0.0083), 0.0020)  # round hook nose in the rail's top slot; the feeder turns about it
NOSE_W = 0.014  # the nose runs along the slot for nearly the feeder's width
SPOOL_ARM = ((-0.186, -0.095), (-0.019, -0.011))
HUB = (-0.186, -0.015)
REEL_R, REEL_W = 0.089, 0.013
PIVOT = (-0.0440, -0.0293)  # release lever pivot, outboard of the teeth so a lift on a tooth turns it
TOOTH = ((-0.0376, -0.03755), 0.0025, (0.0034, 0.0070))  # one tooth either side of the peg (w and -w)
LEVER_SIDE = ((-0.0435, -0.0395), (-0.037, -0.029))  # the lever's two side bars, beside the blade's face
LEVER_CROSS = [((-0.050, -0.0435), (-0.032, -0.029)), ((-0.050, -0.047), (-0.032, -0.019))]  # crossbar and thumb tab

BODY_MASS = 0.13  # kg: printed frame, two gear motors, board and spool arm (estimate)
REEL_MASS = 0.13  # kg: 7 inch reel with paper tape, part used (estimate)
LEVER_MASS = 0.025
# Obtained by phone from a technician who works with this product (Nov 2026).
# The call reports catch pull, not its lever arm. Use the longest pivot-to-tab
# radius as a conservative model mapping: resisting torque / any tab moment arm
# is >= CATCH_RELEASE_PULL at the closed stop and increases during opening.
CATCH_RELEASE_PULL = 8.0  # N on a new feeder; lower as it wears in
CATCH_PULL_ARM = math.hypot(LEVER_CROSS[1][0][0] - PIVOT[0], LEVER_CROSS[1][1][1] - PIVOT[1])
LEVER_PRELOAD = -0.15  # rad, engineering assumption beyond the closed stop
LEVER_SPRING = CATCH_RELEASE_PULL * CATCH_PULL_ARM / abs(LEVER_PRELOAD)  # N m/rad
REEL_RIM_DRAG = 0.30  # N tangentially at the reel rim; phone figure
REEL_HUB_FRICTION = REEL_RIM_DRAG * REEL_R  # N m = 0.0267 at 89 mm rim
REEL_DRAG = 0.002  # N m s/rad, additional viscous drag (estimate)
OBSERVED_REEL_TURN_DEG = (5.0, 15.0)  # phone-reported slight turn during removal; not a score gate
# Threaded tape proxy, not physically validated. Reel and stopped feeder travel together.
# Rim drag is implemented as equal static/dynamic hub friction, with parent reaction.
# Winding takes up WIND_PLAY then meets an ideal angular stop representing taut tape.
# Unwinding produces slack; no finite tape breaking/slip load or snag collider exists.
TAPE_R = 0.070  # m, estimated tape takeoff radius on a part-used 7 inch reel
TAPE_TENSION = REEL_HUB_FRICTION / TAPE_R  # 0.381 N equivalent hub scale, not applied tape load
WIND_PLAY = 0.02  # rad, estimated winding take-up before taut-tape angular stop
REEL_SLACK = 3.0  # rad, numerical unwind bound, not a measured slack capacity
# Printed feeder surfaces: 0.38 static is the top of the 0.12-0.38 range measured between pairs of printed PLA parts
# (Santana, Alves and Netto, Proc. IMechE Part L 235(6), 2021; printed PETG pairs 0.13-0.21); 0.30 sliding is an estimate.
FEEDER_FRICTION = (0.38, 0.30)


def _box(ur, zr, wr, color, collide=True, quat=None, friction=None):
    if isinstance(wr, (int, float)):
        wr = (-wr, wr)
    size = (ur[1] - ur[0], wr[1] - wr[0], zr[1] - zr[0])
    at = ((ur[0] + ur[1]) / 2, (wr[0] + wr[1]) / 2, (zr[0] + zr[1]) / 2)
    return parts.box(size, at=at, color=color, quat=quat, collide=collide, friction=friction)


def _feeder_shapes():
    out = [_box(u, z, W2, FEEDER_BLACK) for u, z in BODY]
    out.append(_box(*CLIP, W2, FEEDER_BLACK))
    for s in (1, -1):
        w = PRONG[2] if s > 0 else (-PRONG[2][1], -PRONG[2][0])
        out.append(_box(PRONG[0], PRONG[1], w, FEEDER_BLACK))
        out.append(_box(BEAR[0], BEAR[1], w, FEEDER_BLACK))
    out.append(_box(BRIDGE[0], BRIDGE[1], BRIDGE[2], FEEDER_BLACK))
    out.append(_box(STEM[0], STEM[1], NOSE_W / 2, FEEDER_BLACK))
    out.append(parts.cylinder(NOSE[1], NOSE_W, at=(NOSE[0][0], 0.0, NOSE[0][1]), color=FEEDER_BLACK, axis="y"))
    out.append(_box(*SPOOL_ARM, 0.0025, FEEDER_BLACK))
    # looks only: the threaded tape from the top of the reel into the feeder, and a label strip on each side
    a = math.atan2(0.030 - 0.074, -0.095 - (-0.150))
    out.append(parts.box((0.071, 0.008, 0.0006), at=(-0.1225, 0.0, 0.052), color=TAPE, quat=_qy(-a), collide=False))
    for s in (1, -1):
        out.append(_box((-0.010, 0.050), (0.022, 0.032), (s * W2 - 0.0002, s * W2 + 0.0002) if s > 0 else
                        (-W2 - 0.0002, -W2 + 0.0002), LABEL, collide=False))
    return out


def _reel_shapes():
    out = [parts.cylinder(REEL_R, REEL_W, at=(HUB[0], 0.0, HUB[1]), color=REEL_WHITE, axis="y")]
    for s in (1, -1):  # looks only: the wound tape seen through the faces, the hub, and a spoke so its turning shows
        out.append(parts.cylinder(0.070, 0.0004, at=(HUB[0], s * (REEL_W / 2 + 0.0002), HUB[1]), color=TAPE, axis="y",
                                  collide=False))
        out.append(parts.cylinder(0.018, 0.0006, at=(HUB[0], s * (REEL_W / 2 + 0.0005), HUB[1]), color=REEL_WHITE,
                                  axis="y", collide=False))
        out.append(parts.cylinder(0.006, 0.0008, at=(HUB[0], s * (REEL_W / 2 + 0.0008), HUB[1]), color=YELLOW, axis="y",
                                  collide=False))
        out.append(parts.box((0.050, 0.0008, 0.006), at=(HUB[0] + 0.044, s * (REEL_W / 2 + 0.0006), HUB[1]),
                             color=REEL_WHITE, collide=False))
    return out


def _lever_shapes():
    out = []
    for s in (1, -1):
        w = TOOTH[2] if s > 0 else (-TOOTH[2][1], -TOOTH[2][0])
        out.append(parts.cylinder(TOOTH[1], w[1] - w[0], at=(TOOTH[0][0], (w[0] + w[1]) / 2, TOOTH[0][1]), color=YELLOW,
                                  axis="y"))
        out.append(_box(*LEVER_SIDE, w, YELLOW))
    out += [_box(u, z, TOOTH[2][1], YELLOW) for u, z in LEVER_CROSS]
    return out


# -- layout -------------------------------------------------------------------------------------------------------------
# The machine's own frame (maker CAD, mm): x_m along the front rail from the machine's left outer face, y_m inward
# (front to back), z_m up from the bottom of the frame, whose feet stand 100 mm lower on the bench.
X_L, Y_R = 0.150, -0.070  # world x of the machine's left outer face; world y of the front rail's centre line
SLOT_X = [103.0 + PITCH * 1000 * (k + 0.5) for k in range(25)]  # front slot centres, x_m
FIRST = 0  # the bank fills slots 1 to 3; the middle feeder is in slot 2
X0 = X_L + SLOT_X[FIRST + 1] / 1000  # world x of the middle slot
YAW = -math.pi / 2  # feeder frame u -> world -y, w -> world +x
NAMES = ("feeder_front", "feeder_middle", "feeder_back")  # nearest the robot first
MIDDLE = "feeder_middle"
NEIGHBOURS = ("feeder_front", "feeder_back")


def _m(xr, yr, zr):
    """World (centre, size) of a box given in the machine frame (mm ranges)."""
    xs = (X_L + xr[0] / 1000, X_L + xr[1] / 1000)
    ys = (Y_R - (yr[1] - 10) / 1000, Y_R - (yr[0] - 10) / 1000)
    zs = (TOP + 0.100 + zr[0] / 1000, TOP + 0.100 + zr[1] / 1000)
    return [(a + b) / 2 for a, b in (xs, ys, zs)], [b - a for a, b in (xs, ys, zs)]


ALU = (0.13, 0.13, 0.14)
PRINT = (0.10, 0.10, 0.11)
BED = (0.08, 0.08, 0.09)
STEEL = (0.62, 0.64, 0.67)
# (name, x_m, y_m, z_m, colour); positions from the maker's assembly, the gantry parked at the back of its travel
MACHINE = [
    ("left lower side rail", (0, 20), (-80, 520), (20, 40), ALU), ("right lower side rail", (580, 600), (-80, 520), (20, 40), ALU),
    ("left upper side rail", (0, 20), (-80, 520), (90, 110), ALU), ("right upper side rail", (580, 600), (-80, 520), (90, 110), ALU),
    ("left y guide", (4, 16), (-56, 494), (110, 118), STEEL), ("right y guide", (584, 596), (-56, 494), (110, 118), STEEL),
    ("back frame rail", (0, 600), (450, 470), (0, 20), ALU),
    ("front left leg", (-10, 30), (-104, 0), (-50, 120), PRINT), ("front left foot", (-10, 20), (-85, -29), (-100, -50), PRINT),
    ("front right leg", (570, 610), (-104, 0), (-50, 120), PRINT), ("front right foot", (580, 610), (-85, -29), (-100, -50), PRINT),
    ("back left leg", (-12, 31), (500, 568), (-100, 120), PRINT), ("back right leg", (568, 612), (500, 568), (-100, 120), PRINT),
    ("bed", (20, 580), (85, 450), (40, 43), BED), ("bottom camera", (101, 139), (121, 159), (4, 34), PRINT),
    ("control box", (-65, 0), (30, 244), (-80, 40), PRINT),
    ("gantry beam", (-65, 640), (420, 440), (125, 165), ALU), ("left gantry carriage", (-12, 27), (384, 496), (85, 135), PRINT),
    ("right gantry carriage", (573, 612), (384, 496), (85, 135), PRINT), ("toolhead", (399, 489), (395, 455), (88, 240), PRINT),
]


def _machine_shapes():
    out = []
    for _, xr, yr, zr, col in MACHINE:
        c, size = _m(xr, yr, zr)
        out.append(parts.box(tuple(size), at=tuple(c), color=col))
    return out


def _rail_shapes():
    lo, hi = -SLOT_X[FIRST + 1] / 1000, (600.0 - SLOT_X[FIRST + 1]) / 1000  # the rail's ends, in w
    out = [_box(RAIL_CORE[0], RAIL_CORE[1], (lo, hi), RAIL_BLACK)]
    for u, z in RAIL_TOP:
        out.append(_box(u, z, (lo, hi), RAIL_BLACK))
    for s in (1, -1):  # looks only: the V grooves on the two side faces
        out.append(_box((s * 0.0101 - 0.0001, s * 0.0101 + 0.0001), (-0.003, 0.003), (lo, hi), (0.05, 0.05, 0.05),
                        collide=False))
    b0, b1 = (103.0 - SLOT_X[FIRST + 1]) / 1000, (498.0 - SLOT_X[FIRST + 1]) / 1000
    out.append(_box(BLADE[0], BLADE[1], (b0, b1), YELLOW))
    for k in range(25):
        w = (SLOT_X[k] - SLOT_X[FIRST + 1]) / 1000
        out.append(_box(PEG[0], PEG[1], (w - PEG[2], w + PEG[2]), YELLOW))
        out.append(_box((BLADE[0][0] - 0.0003, BLADE[0][0]), (0.000, 0.006), (w - 0.003, w + 0.003), (0.15, 0.15, 0.15),
                        collide=False))  # looks only: a slot number plate
    return out


def _feeder(name, k):
    lever = parts.Hinge("lever", shapes=tuple(_lever_shapes()), pivot=(PIVOT[0], 0.0, PIVOT[1]), axis=(0.0, 1.0, 0.0),
                        lower=0.0, upper=0.6, mass=LEVER_MASS, spring=LEVER_SPRING, damping=0.002, max_effort=1.0,
                        target=LEVER_PRELOAD)
    reel = parts.Hinge("reel", shapes=tuple(_reel_shapes()), pivot=(HUB[0], 0.0, HUB[1]), axis=(0.0, 1.0, 0.0),
                       lower=-WIND_PLAY, upper=REEL_SLACK, mass=REEL_MASS, spring=0.0, damping=REEL_DRAG, max_effort=1.0,
                       friction=REEL_HUB_FRICTION, dynamic_friction=REEL_HUB_FRICTION)
    return parts.Part(name, pos=(X0 + k * PITCH, Y_R, TOP + Z_R), yaw=YAW, mass=BODY_MASS,
                      shapes=tuple(_feeder_shapes()), joints=(lever, reel), friction=FEEDER_FRICTION, contact_offset=PACKED_CONTACT,
                      link_friction=(("lever", (0.2, 0.15)),))


FEEDERS = {n: _feeder(n, k) for n, k in zip(NAMES, (-1, 0, 1))}
RAIL = parts.Part("feeder_rail", pos=(X0, Y_R, TOP + Z_R), yaw=YAW, shapes=tuple(_rail_shapes()), friction=(0.3, 0.25),
                  contact_offset=PACKED_CONTACT)
MACHINE_PART = parts.Part("pick_and_place_machine", pos=(0.0, 0.0, 0.0), shapes=tuple(_machine_shapes()),
                          friction=(0.5, 0.4), contact_offset=PACKED_CONTACT)
MAT_SIZE = (0.300, 0.420, 0.003)
MAT_A = (0.350, 0.425)
MAT = parts.Part("service_mat", pos=(*MAT_A, TOP), mass=0.5,
                 shapes=(parts.box(MAT_SIZE, at=(0.0, 0.0, MAT_SIZE[2] / 2), color=MAT_BLUE),), friction=(0.9, 0.8),
                 contact_offset=PACKED_CONTACT)
OBJECTS = [MACHINE_PART, RAIL, *FEEDERS.values(), MAT]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"service_mat": (*MAT_A, TOP)}),
                    lots.Lot("B", place={"service_mat": (MAT_A[0] - 0.030, MAT_A[1], TOP)}))

# -- scoring constants --------------------------------------------------------------------------------------------------
NB_MOVE, NB_TURN = 0.002, D(3.0)  # neighbour limits at the end of the trial
# At any moment, a neighbour is knocked loose when its hook leaves the rail's top slot or its catch lets go, or (a
# backstop for sliding or turning that the simplified clip does not stop) its origin moves or its body turns past these:
NB_LOOSE_MOVE, NB_LOOSE_TURN = 0.005, D(10.0)
SLOT_HALF, RAIL_TOP_Z = RAIL_TOP[1][0][0], RAIL_TOP[1][1][1]  # top slot half width (2.5 mm); rail top face (z 10 mm)
BLADE_FACE, BLADE_BOTTOM = BLADE[0][0], BLADE[1][0]  # slot blade's front face (u -38 mm) and bottom face (z -35 mm)
NOSE_ENDS = tuple((NOSE[0][0], s * NOSE_W / 2, NOSE[0][1]) for s in (-1, 1))  # hook nose axis, both ends
TEETH = tuple((TOOTH[0][0], s * (TOOTH[2][0] + TOOTH[2][1]) / 2, TOOTH[0][1]) for s in (1, -1))  # lever tooth centres
CLEAR = 0.005  # middle feeder clear of the rail, slot blade and pegs
HAND_CLEAR = 0.005  # released: no gripper or fingertip within this of the middle feeder
FLAT = D(20.0)  # lying on a face: its across axis within this of vertical
STILL_V, STILL_W, HOLD_S = 0.005, 0.05, 1.0
DROP_V = 1.2  # m/s: any feeder this fast has been dropped (a free fall from about 7 cm)
SUCCESS_HOLD_S = 1.0


# -- geometry helpers ---------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(np.asarray(q, float))


def _surface_points(shape, step=0.004) -> np.ndarray:
    """Points on a shape's surface in its part (or link) frame."""
    if shape.kind == "cylinder":  # all cylinders here lie along the frame's y axis
        r, h = shape.size[0], shape.size[1]
        n = max(12, int(2 * math.pi * r / step))
        a = np.linspace(0, 2 * math.pi, n, endpoint=False)
        ring = np.c_[r * np.cos(a), np.zeros(n), r * np.sin(a)]  # axis y
        pts = [ring + (0, y, 0) for y in (-h / 2, h / 2)]
        pts.append(np.array([[0.0, -h / 2, 0.0], [0.0, h / 2, 0.0]]))
        loc = np.vstack(pts)
    else:
        half = np.asarray(shape.size, float) / 2
        axes = [np.linspace(-hh, hh, max(2, int(2 * hh / step) + 1)) for hh in half]
        g = np.array(np.meshgrid(*axes, indexing="ij")).reshape(3, -1).T
        loc = g[np.any(np.isclose(np.abs(g), half, atol=1e-9), axis=1)]
    return loc @ shape.R.T + np.asarray(shape.at, float)


_PTS = {}


LINKS = {"lever": (_lever_shapes, PIVOT), "reel": (_reel_shapes, HUB)}


def _link_pose(st, name):
    if name not in getattr(st, "links", {}):
        raise ValueError(name + " link pose unknown")
    lp, lq = (np.asarray(a, float) for a in st.links[name])
    if lp.shape != (3,) or lq.shape != (4,) or not np.isfinite(lp).all() or not np.isfinite(lq).all():
        raise ValueError(name + " link pose malformed")
    return lp, _rot(lq)


def feeder_points(st) -> np.ndarray:
    """World points on every collider of a feeder (body, lever and reel), from its reported pose and link poses."""
    if "base" not in _PTS:
        _PTS["base"] = np.vstack([_surface_points(s) for s in _feeder_shapes() if s.collide])
        for name, (fn, piv) in LINKS.items():  # relative to the hinge (the link frame origin)
            _PTS[name] = np.vstack([_surface_points(s) for s in fn() if s.collide]) - np.array([piv[0], 0.0, piv[1]])
    p, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    out = [_PTS["base"] @ r.T + p]
    for name in LINKS:
        lp, lr = _link_pose(st, name)
        out.append(_PTS[name] @ lr.T + lp)
    return np.vstack(out)


def _rail_boxes():
    if "rail" not in _PTS:
        st = RAIL.state()
        _PTS["rail"] = [(c, r, s) for _, c, r, s in RAIL.world_boxes({"base": (np.asarray(st.pos), _rot(st.quat_xyzw))})]
    return _PTS["rail"]


def rail_gap(pts) -> float:
    """Least distance (m) from any point to the rail, its slot blade and its pegs."""
    best = math.inf
    for c, r, s in _rail_boxes():
        loc = (pts - c) @ r
        q = np.abs(loc) - np.asarray(s, float) / 2
        d = np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
        best = min(best, float(d.min()))
    return best


def _shape_frames(st):
    """(shape, world centre, world rotation) for every collider of a feeder: body, lever and reel shapes."""
    p, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    out = [(s, p + r @ np.asarray(s.at, float), r @ s.R) for s in _feeder_shapes() if s.collide]
    for name, (fn, piv) in LINKS.items():
        lp, lr = _link_pose(st, name)
        pv = np.array([piv[0], 0.0, piv[1]])
        out += [(s, lp + lr @ (np.asarray(s.at, float) - pv), lr @ s.R) for s in fn() if s.collide]
    return out


def _shape_distance(pts, shape, c, r) -> np.ndarray:
    """Exact signed distance (m) from points to one box, or to one solid cylinder along its frame's y axis."""
    loc = (pts - c) @ r
    if shape.kind == "cylinder":
        rad, h = shape.size[0], shape.size[1]
        q = np.c_[np.hypot(loc[:, 0], loc[:, 2]) - rad, np.abs(loc[:, 1]) - h / 2]
    else:
        q = np.abs(loc) - np.asarray(shape.size, float) / 2
    return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)


def hand_gap(clouds, st) -> float:
    """Least distance from any face-sampled gripper or fingertip surface point to any collider of the feeder, computed
    exactly against each box and cylinder (reel faces included). Capped at 30 mm."""
    if not clouds:
        raise ValueError("no hand geometry")
    frames = _shape_frames(st)
    best = 0.03
    for c in clouds:
        c = np.asarray(c, float)
        if c.ndim != 2 or c.shape[1] != 3 or not len(c) or not np.isfinite(c).all():
            raise ValueError("hand geometry malformed")
        for shape, cen, rot in frames:
            reach = float(np.linalg.norm(shape.half)) + 0.03
            near = c[np.linalg.norm(c - cen, axis=1) <= reach]
            if len(near):
                best = min(best, float(_shape_distance(near, shape, cen, rot).min()))
    return best


def _rail_frame(pts) -> np.ndarray:
    """World points in the rail's frame (u out from the rail centre line toward the bed, w along the rail, z up)."""
    st = RAIL.state()
    return (np.asarray(pts, float) - np.asarray(st.pos, float)) @ _rot(st.quat_xyzw)


def hooked(st) -> bool:
    """The hook nose is in the rail's top slot: at both ends of the nose, its axis lies within the slot's width and its
    lowest point below the rail's top face."""
    p, r = np.asarray(st.pos, float), _rot(st.quat_xyzw)
    c = _rail_frame([p + r @ np.asarray(e) for e in NOSE_ENDS])
    return bool(np.all(np.abs(c[:, 0]) <= SLOT_HALF + EPS) and np.all(c[:, 2] - NOSE[1] <= RAIL_TOP_Z + EPS))


def latched(st) -> bool:
    """The catch holds: each of the lever's two teeth is still under the slot blade (its centre at or below the
    blade's bottom face, and its inboard edge reaching in past the blade's front face)."""
    lp, lr = _link_pose(st, "lever")
    pv = np.array([PIVOT[0], 0.0, PIVOT[1]])
    c = _rail_frame([lp + lr @ (np.asarray(t) - pv) for t in TEETH])
    return bool(np.all(c[:, 2] <= BLADE_BOTTOM + EPS) and np.all(c[:, 0] + TOOTH[1] > BLADE_FACE + EPS))


def flat(st) -> bool:
    across = _rot(st.quat_xyzw)[:, 1]  # the feeder's w axis
    return bool(abs(across[2]) >= math.cos(FLAT) - EPS)


def on_mat(st, mat) -> bool:
    """Every sampled point of the middle feeder lies over the mat, and its lowest point within 5 mm of the mat top."""
    pts = feeder_points(st)
    mp, mr = np.asarray(mat.pos, float), _rot(mat.quat_xyzw)
    loc = (pts - mp) @ mr
    inside = np.all(np.abs(loc[:, :2]) <= np.array(MAT_SIZE[:2]) / 2 + EPS)
    low = float(loc[:, 2].min())
    return bool(inside and -0.001 <= low <= MAT_SIZE[2] + 0.005 + EPS)


def on_bench(st, mat) -> bool:
    """Resting on the bench or the mat: its lowest point within 5 mm of the bench top or of the mat top."""
    pts = feeder_points(st)
    low = float(pts[:, 2].min())
    mp = np.asarray(mat.pos, float)
    return bool(TOP - 0.001 <= low <= TOP + 0.005 + EPS or
                mp[2] + MAT_SIZE[2] - 0.001 <= low <= mp[2] + MAT_SIZE[2] + 0.005 + EPS)


# -- the trial record ---------------------------------------------------------------------------------------------------
def _fresh():
    return dict(known=False, steps=0, t=None, start=None, zero=False, why="", gap=math.inf, hand=0.0, since=None,
                held=0.0, prev=None)


_rec = _fresh()


def _turn(qa, qb) -> float:
    r = _rot(qa).T @ _rot(qb)
    return math.acos(max(-1.0, min(1.0, (np.trace(r) - 1) / 2)))


def observe(objects, clouds, t) -> None:
    """One control step. An unknown or malformed observation scores 0 for as long as it lasts (fail closed)."""
    try:
        sts = {n: objects[n] for n in (*NAMES, MAT.name)}
        for n, st in sts.items():
            arr = np.r_[st.pos, st.quat_xyzw, st.lin_vel, st.ang_vel]
            if arr.shape != (13,) or not np.isfinite(arr).all() or np.linalg.norm(st.quat_xyzw) <= EPS:
                raise ValueError("state unknown: " + n)
        if _rec["t"] is not None and not t > _rec["t"]:
            raise ValueError("time did not advance")
        if _rec["t"] is not None and t - _rec["t"] > 0.05 + EPS:
            _rec.update(since=None, held=0.0)  # a missing 20 Hz sample interrupts the continuous hold
        if _rec["start"] is None:
            _rec["start"] = {n: (np.asarray(sts[n].pos, float).copy(), np.asarray(sts[n].quat_xyzw, float).copy())
                             for n in NAMES}
        m = sts[MIDDLE]
        pts = feeder_points(m)
        gap = rail_gap(pts)
        hand = hand_gap(clouds, m)
        if not _rec["zero"]:
            for n in NEIGHBOURS:
                if not hooked(sts[n]):
                    _rec.update(zero=True, why=f"{n} was knocked loose: its hook left the rail's slot")
                elif not latched(sts[n]):
                    _rec.update(zero=True, why=f"{n} was knocked loose: its catch let go")
                elif np.linalg.norm(np.asarray(sts[n].pos) - _rec["start"][n][0]) > NB_LOOSE_MOVE + EPS or \
                        _turn(_rec["start"][n][1], sts[n].quat_xyzw) > NB_LOOSE_TURN + EPS:
                    _rec.update(zero=True, why=f"{n} was knocked loose: moved over 5 mm or turned over 10 degrees")
            for n in NAMES:
                if np.linalg.norm(sts[n].lin_vel) > DROP_V:
                    _rec.update(zero=True, why=f"{n} was dropped")
                if float(np.asarray(sts[n].pos)[2]) < TOP - 0.02:
                    _rec.update(zero=True, why=f"{n} left the bench")
        rest = (gap >= CLEAR - EPS and hand >= HAND_CLEAR - EPS and flat(m) and on_bench(m, sts[MAT.name]) and
                m.resting(STILL_V, STILL_W) and neighbours_seated(sts))
        if rest and _rec["since"] is None:
            _rec["since"] = t
        if not rest:
            _rec["since"] = None
        _rec.update(known=True, steps=_rec["steps"] + 1, t=t, gap=gap, hand=hand,
                    held=0.0 if _rec["since"] is None else t - _rec["since"])
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, since=None, held=0.0, why="unknown observation: " + str(e))


def neighbours_seated(objects) -> bool:
    """Both outer feeders within 2 mm and 3 degrees of where they started."""
    for n in NEIGHBOURS:
        p0, q0 = _rec["start"][n]
        st = objects[n]
        if np.linalg.norm(np.asarray(st.pos, float) - p0) > NB_MOVE + EPS or _turn(q0, st.quat_xyzw) > NB_TURN + EPS:
            return False
    return True


def score(objects):
    """0: unknown observation; at any moment a neighbour knocked loose (its hook out of the rail's slot, its catch let
    go, or its origin moved over 5 mm or its body turned over 10 degrees) or any feeder dropped or off the bench; at the end, a neighbour more than 2 mm or 3 degrees from its start,
    or the middle feeder within 5 mm of the rail, its slot blade or pegs.
    0.5: the middle feeder 5 mm or more clear of all of them (held, moving, or not lying on a side).
    0.75: also released (no hand within 5 mm), lying on a side on the bench or mat, still for 1 s.
    1.0: also entirely over the mat."""
    try:
        if not _rec["known"] or _rec["steps"] == 0 or _rec["zero"] or _rec["start"] is None:
            return 0.0
        if not neighbours_seated(objects):
            return 0.0
        m = objects[MIDDLE]
        pts = feeder_points(m)
        if rail_gap(pts) < CLEAR - EPS:
            return 0.0
        value = 0.5
        if _rec["held"] >= HOLD_S - EPS and _rec["hand"] >= HAND_CLEAR - EPS and flat(m) and \
                on_bench(m, objects[MAT.name]) and m.resting(STILL_V, STILL_W):
            value = 0.75
            if on_mat(m, objects[MAT.name]):
                value = 1.0
        return value
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) == 1.0


@parts.on_reset
def _reset(sim):
    _rec.clear()
    _rec.update(_fresh())
    observation.reset()


@parts.on_step
def _step(sim):
    objects = sim.states()
    try:
        clouds, _ = observation.hand_state(sim)
    except (ValueError, AttributeError, KeyError, IndexError):
        _rec.update(known=False, since=None, held=0.0, why="hand state unknown")
        return
    observe(objects, clouds, float(sim.t))
    if os.environ.get("TRACE_OUT") or os.environ.get("FEEDER_TRACE"):
        row = dict(t=float(sim.t), lot=lots.current().name, known=_rec["known"], zero=_rec["zero"], why=_rec["why"],
                   gap=_rec["gap"], hand=_rec["hand"], held=_rec["held"], score=score(objects))
        for n in (*NAMES, MAT.name):
            st = objects[n]
            row[n] = dict(pos=np.asarray(st.pos).round(5).tolist(), quat=np.asarray(st.quat_xyzw).round(5).tolist(),
                          v=float(np.linalg.norm(st.lin_vel)), joints={k: float(v) for k, v in st.joints.items()})
            if n in NEIGHBOURS:
                try:
                    row[n].update(hooked=hooked(st), latched=latched(st))
                except (ValueError, KeyError):
                    pass
        try:  # debugging only: total incoming reel joint wrench, not isolated tape tension
            base = sim.env.unwrapped
            for n in NAMES:
                asset = base.scene[n]
                view = getattr(asset, "root_physx_view", None) or getattr(asset, "root_view", None)
                arr = view.get_link_incoming_joint_force()
                f = np.asarray(arr.numpy() if hasattr(arr, "numpy") else arr.detach().cpu().numpy(), float)
                f = f.reshape(f.shape[0], -1, 6)[0]  # per link: force, then torque about the link origin (the hub)
                i = asset.body_names.index("reel")
                row[n]["reel_wrench"] = np.round(f[i], 5).tolist()
                row[n]["reel_vel"] = float(parts._np(asset.data.joint_vel)[0][asset.joint_names.index("reel")])
                lever_q = float(objects[n].joints["lever"])
                row[n]["lever_spring_torque_Nm"] = LEVER_SPRING * (lever_q - LEVER_PRELOAD)
                row[n]["catch_tab_force_lower_bound_N"] = row[n]["lever_spring_torque_Nm"] / CATCH_PULL_ARM
        except Exception as e:  # noqa: BLE001  (debug logging only)
            row["reel_wrench_error"] = repr(e)[:200]
        try:  # debugging only: where each hand's TCP is
            from yam_arena.constants import TCP_OFFSET

            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            for arm in ("left_robot", "right_robot"):
                robot = base.scene[arm]
                pose = parts._np(robot.data.body_link_pose_w)[0, robot.body_names.index("gripper")]
                tcp = pose[:3] - origin + parts._quat_to_matrix(pose[3:]) @ np.array(TCP_OFFSET)
                q = parts._np(robot.data.joint_pos)[0]
                row[arm] = dict(tcp=np.round(tcp, 4).tolist(), finger=float(q[robot.joint_names.index("joint7")]))
        except (AttributeError, KeyError, IndexError, ValueError, ImportError):
            pass
        with pathlib.Path(os.environ.get("TRACE_OUT") or os.environ["FEEDER_TRACE"]).with_name("feeder.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- demo: open loop from the published layout --------------------------------------------------------------------------
def world(u, z, k=0):
    """World point of feeder-frame (u, z) in slot k (-1 front, 0 middle, 1 back)."""
    return (X0 + k * PITCH, Y_R - u, TOP + Z_R + z)


GRIP_YAW = math.pi / 2  # fingers close along world x, across the feeders
PINCH = -0.0030  # finger command: 4.5 mm past each side of the 15 mm feeder body


def _w(d, p, finger, label, tilt_deg=0.0, heading=0.0, yaw=GRIP_YAW):
    return tilt.Waypoint(d, tuple(float(v) for v in p), finger, label, yaw, tilt=tilt_deg, heading=heading)


def demo():
    lot = lots.current()
    mx, my, _ = lot.pos(MAT.name)
    inner_top = TOP + Z_R + BODY[1][1][1]  # top of the feeders' inner part
    outer_top = TOP + Z_R + BODY[2][1][1]  # top of the feeders' outer part
    # right hand: one open finger presses the middle feeder's inner end down, the other finger is in free space
    px, py = X0 + 0.0545, Y_R - 0.060  # 4.5 mm past the finger spacing: the arm lags about that far here
    right = [
        _w(2.5, (px, py, TOP + 0.195), FINGER_OPEN, "above the middle feeder's inner end"),
        _w(1.5, (px, py, inner_top + 0.0129 + 0.004), FINGER_OPEN, "finger on the inner end"),
        _w(3.0, (px, py, inner_top + 0.0129 - 0.045), FINGER_OPEN, "press it down"),
        _w(5.2, (px, py, inner_top + 0.0129 - 0.045), FINGER_OPEN, "hold it tipped"),
        _w(1.5, (px, py, TOP + 0.195), FINGER_OPEN, "lift away"),
        _w(2.0, (0.20, -0.30, TOP + 0.22), FINGER_OPEN, "aside"),
        _w(20.0, (0.20, -0.30, TOP + 0.22), FINGER_OPEN, "wait"),
    ]
    # left hand: pinch the middle feeder's outer end where it now stands proud, lift the hook out, move it out a
    # little, level it, slide it out along its own plane past the neighbours' reels, then lay it on the mat
    gy = Y_R + 0.078
    gz = outer_top + 0.004 + 0.0129
    level = dict(tilt_deg=15.0, heading=-math.pi / 2)
    out_y = gy + 0.365
    lay = (mx + 0.060, out_y + 0.040)
    away = tilt.away("left", lay)

    def lw(*a, **k):
        return _w(*a, yaw=-GRIP_YAW, **k)

    left = [
        lw(2.5, (X0, gy, TOP + 0.178), FINGER_OPEN, "above the feeders' outer ends"),
        lw(4.5, (X0, gy, TOP + 0.178), FINGER_OPEN, "wait for the tip"),
        lw(1.5, (X0, gy, gz), FINGER_OPEN, "fingers either side of the proud outer end"),
        lw(1.0, (X0, gy, gz), PINCH, "pinch the feeder body"),
        lw(0.7, (X0, gy, gz), PINCH, "settle"),
        lw(2.0, (X0, gy, gz + 0.040), PINCH, "lift the hook out", tilt_deg=5.0, heading=-math.pi / 2),
        lw(1.5, (X0, gy, gz + 0.040), PINCH, "hold while the other hand lifts", tilt_deg=5.0, heading=-math.pi / 2),
        lw(1.5, (X0, gy + 0.020, gz + 0.040), PINCH, "out a little", tilt_deg=5.0, heading=-math.pi / 2),
        lw(2.0, (X0, gy + 0.020, gz + 0.029), PINCH, "level it", **level),
        lw(3.0, (X0, gy + 0.170, gz + 0.029), PINCH, "slide it out", **level),
        lw(3.0, (X0, out_y, gz + 0.029), PINCH, "clear of the bank"),
        lw(1.5, (*lay, gz + 0.004), PINCH, "over the mat"),
        lw(2.0, (*lay, gz - 0.010), PINCH, "lower"),
        lw(2.0, (*lay, gz - 0.010), PINCH, "lean it", tilt_deg=35.0, heading=away),
        lw(1.0, (*lay, gz - 0.010), FINGER_OPEN, "let go", tilt_deg=35.0, heading=away),
        lw(1.5, (*lay, gz + 0.030), FINGER_OPEN, "up", tilt_deg=20.0, heading=away),
        lw(4.0, (*lay, gz + 0.030), FINGER_OPEN, "wait", tilt_deg=20.0, heading=away),
    ]
    return {"left": left, "right": right}
