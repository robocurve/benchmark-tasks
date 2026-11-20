"""Mounting a wafer for e-beam evaporation and loading it into the evaporator's load lock (after a Lesker PVD SOP).

An e-beam evaporator deposits from sources below the samples, so a wafer travels face down, clipped to a sample plate
that a ring holder carries. The load lock has just been vented; its door, which slides up, is shut and held by a
twist lock, and its light tower turns from blue (venting) to green. Inside, on its holder,
lies the ring holder. On a loading stand beside it lies the sample plate (stainless, 200 mm across, for wafers up to 6
inch), face up, with a 100 mm wafer on it, pattern up and about 5 mm off centre. Round the wafer, in the plate's 4 inch
ring of tapped holes, stand eight swing clips, all parked open with their screws loose. The wafer's primary flat faces
one of them, and one clip has a screw too long for its hole: its head stands 8 mm proud, and tightened, its tip would
stick out of the plate's back and jam the transfer. A screwdriver stands in a holder.

The job: once the light is green, turn the twist lock and lift the door until it latches open, take the ring holder out and set it on the plate; centre the wafer; swing
in clips that can hold its round edge, enough of them and not the one with the long screw, and tighten their screws
(300 to 420 degrees: tighter cracks the wafer); put the screwdriver back; turn the plate and ring over together
(samples face down, the ring now carrying the plate) and set them on the load lock's holder; pull the door down shut
and lock it.
The door may be open 2 minutes in all: an open load lock takes on air and water that slow its pump-down and load the
main chamber.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py -E object_jitter=0 -E control_hz=20 \\
           --instruction "<see task.json>" --scorer task_score ...
"""

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import labkit  # noqa: E402
from labkit import Joint, on_reset, on_step, part  # noqa: E402
import yam_arena.demo as _demo  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DT = 0.05  # one control step at 20 Hz

SI_BACK = (0.58, 0.59, 0.62)
DIE = (0.20, 0.22, 0.42)
STREET = (0.62, 0.60, 0.48)
STAINLESS = (0.60, 0.61, 0.64)
ALU = (0.80, 0.81, 0.83)
STAND_C = (0.42, 0.43, 0.46)
STEEL = (0.72, 0.73, 0.75)
DARK = (0.20, 0.21, 0.23)
HOLE = (0.12, 0.12, 0.13)
CHAMBER = (0.55, 0.57, 0.60)
VIEWPORT = (0.82, 0.90, 0.92)
RED = (0.75, 0.15, 0.12)
LAMP_BLUE = (0.15, 0.40, 1.00)
LAMP_GREEN = (0.10, 0.90, 0.20)
labkit.GLASS_COLORS.update({VIEWPORT})
X = np.array([1.0, 0.0, 0.0])
Y = np.array([0.0, 1.0, 0.0])
Z = np.array([0.0, 0.0, 1.0])

FONT = {  # 5 x 7, rows top to bottom
    "V": ["10001", "10001", "10001", "10001", "10001", "01010", "00100"],
    "E": ["11111", "10000", "10000", "11110", "10000", "10000", "11111"],
    "N": ["10001", "11001", "10101", "10011", "10001", "10001", "10001"],
    "T": ["11111", "00100", "00100", "00100", "00100", "00100", "00100"],
    "P": ["11110", "10001", "10001", "11110", "10000", "10000", "10000"],
    "U": ["10001", "10001", "10001", "10001", "10001", "10001", "01110"],
    "M": ["10001", "11011", "10101", "10101", "10001", "10001", "10001"],
    " ": ["00000"] * 7,
}


def Rz(a):
    return axis_angle_matrix(Z, a)


def _boxes_mesh(boxes, colour):
    """One drawn-only mesh (no collision) from many boxes: (centre, size[, yaw]) tuples."""
    pts, tris = [], []
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6), (0, 2, 6),
             (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    for box in boxes:
        c, h = np.asarray(box[0], float), np.asarray(box[1], float) / 2
        rot = Rz(box[2]) if len(box) > 2 else np.eye(3)
        b = len(pts)
        pts += [c + rot @ (h * [sx, sy, sz]) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        tris += [(b + i, b + j, b + k) for i, j, k in faces]
    return labkit.Mesh(points=np.array(pts), tris=np.array(tris), color=colour, collide="none")


def _glyphs(text, pix):
    w = len(text) * 6 * pix - pix
    out = []
    for n, ch in enumerate(text):
        for r, row in enumerate(FONT[ch]):
            for c, bit in enumerate(row):
                if bit == "1":
                    out.append((-w / 2 + (n * 6 + c + 0.5) * pix, (3 - r) * pix))
    return out


def _text(line, pix, cx, cy, z):
    """Letters printed on a flat face, read from the robots' side: the line runs along -y, letters' tops towards +x."""
    return [((cx + v, cy - u, z), (pix * 0.96, pix * 0.96, 0.0006)) for u, v in _glyphs(line, pix)]


def _prism(outline, z0, z1, colour):
    n = len(outline)
    pts = [(*p, z1) for p in outline] + [(*p, z0) for p in outline] + [(0.0, 0.0, z1), (0.0, 0.0, z0)]
    ct, cb = 2 * n, 2 * n + 1
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(ct, i, j), (cb, n + j, n + i), (i, n + i, n + j), (i, n + j, j)]
    return labkit.Mesh(points=np.array(pts, float), tris=np.array(tris), color=colour, collide="none")


def _disc(r, n=96):
    return np.array([(r * math.cos(2 * math.pi * i / n), r * math.sin(2 * math.pi * i / n)) for i in range(n)])


def _arc_boxes(r0, r1, z0, z1, degs, width, colour):
    """Boxes along a ring between radii r0 and r1, one per angle in degs (tangential size ``width``)."""
    out = []
    for d in degs:
        a = math.radians(d)
        rm = (r0 + r1) / 2
        out.append(((rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2), (r1 - r0, width, z1 - z0), colour, a))
    return out


def _on(deg, r):
    a = math.radians(deg)
    return np.array([r * math.cos(a), r * math.sin(a)])


# -- the wafer (loose): 100 mm silicon, drawn 1 mm thick, pattern up, with its 32.5 mm primary flat. Frame: origin at the
# mid-plane centre, +z out of the pattern side, the flat's middle on +x. ------------------------------------------------
W_R, W_T = 0.0508, 0.0010
PRI_L = 0.0325
FLAT_D = math.sqrt(W_R ** 2 - (PRI_L / 2) ** 2)  # the flat's distance from the centre
EDGE_EXCL = 0.005
STRIP_IN = 0.001  # the wafer's collision strips end this far inside its outline


def _outline():
    pts = [p for p in _disc(W_R, 144) if p[0] <= FLAT_D + 1e-9]
    pts += [(FLAT_D, PRI_L / 2), (FLAT_D, -PRI_L / 2)]
    pts.sort(key=lambda p: math.atan2(p[1], p[0]))
    return np.array(pts, float)


def wafer_inside(p):
    """How far the point p (wafer frame, x and y) lies inside the wafer's outline (negative: outside)."""
    return min(W_R - math.hypot(p[0], p[1]), FLAT_D - p[0])


def _pattern():
    r = W_R - EDGE_EXCL
    out = [_prism(_disc(r), W_T / 2, W_T / 2 + 0.00008, DIE)]
    lines = []
    for k in range(-4, 5):
        u = 0.010 * k + 0.005
        half = math.sqrt(max(r * r - u * u, 0.0)) - 0.0005
        if half > 0.002:
            lines += [((u, 0.0, W_T / 2 + 0.00012), (0.0006, 2 * half, 0.00004)),
                      ((0.0, u, W_T / 2 + 0.00012), (2 * half, 0.0006, 0.00004))]
    out.append(_boxes_mesh(lines, STREET))
    return out


def _strips():
    """The wafer collides as 16 thin strips across its centre, each end 1 mm inside the outline (inside the flat too):
    a thin convex mesh sinks through thin supports, and strip faces give the clips a contact patch."""
    out = []
    for k in range(16):
        a = math.pi * k / 16
        d = np.array([math.cos(a), math.sin(a)])
        ends = []
        for s in (1, -1):
            t = W_R
            while wafer_inside(s * t * d) < STRIP_IN:
                t -= 0.0002
            ends.append(t)
        mid = (ends[0] - ends[1]) / 2
        out.append(((mid * d[0], mid * d[1], 0.0), (ends[0] + ends[1], 0.010, W_T * 0.9), SI_BACK, a))
    return out


# -- the sample plate (loose): stainless, 200 mm across, 1.5 mm thick, face up on the loading stand, with rings of tapped
# holes for 2, 3, 4 and 6 inch wafers. In the 4 inch ring (59.5 mm out) stand eight swing clips every 45 degrees. Each is
# a 13 mm steel tab on its screw: parked along the rim (joint 0) it clears the wafer; swung in 90 degrees clockwise to its
# stop, its tip lies 4.3 mm over a round edge (1.6 mm over the flat). Nothing else holds the wafer: an untightened tab
# stands 0.35 mm over it, so the wafer slides out from under loose clips. Each clip's screw (7 mm cheese head, its slot
# 2.2 mm wide, 2.5 mm deep) turns on the clip: clockwise tightens it. The screw is M3 x 0.5: each turn lowers the tab
# 0.5 mm (the clip rides on a short slide, its shank), so the tab meets the wafer after 252 degrees and presses it from
# then on through the tab's spring (TAB_K), and the screw gets harder to turn as it presses (SCREW_T0, SCREW_T1). A boss under
# the plate sits in the stand's socket. Frame: origin at the centre of the plate's back face, +z out of its sample face.
PL_R, PL_T = 0.100, 0.0015
CLIP_P, CLIP_L, CLIP_W, CLIP_T = 0.0595, 0.013, 0.004, 0.0010
TAB_GAP_REAL = 0.00035  # the real start: the screw's head 0.35 mm over its tab (the README's feeler gauge); the tab
# meets the wafer after 252 degrees
TAB_GAP = 0.0008  # drawn here: the sim's 0.5 mm contact band (labkit CONTACT_OFFSET) plus 0.3 mm, so a swinging tab
# cannot catch the wafer; the screw closes this drawn gap over the same first 252 degrees (see screw_depth)
TAB_BOT = PL_T + W_T + TAB_GAP
TAB_Z = TAB_BOT + CLIP_T / 2
TAB_TOP = TAB_BOT + CLIP_T
KNOB_AT, KNOB_R, KNOB_H = 0.007, 0.0020, 0.005
HEAD_R, HEAD_H, SLOT, RIDGE = 0.0035, 0.0025, 0.0032, (0.0012, 0.0060, 0.0025)  # slot 3.2 mm: the sim's 0.5 mm contact margin
# on both faces leaves the 0.8 mm blade about 0.7 mm of play either side
CLIP_DEG = (0, 45, 90, 135, 180, 225, 270, 315)
FLAT_AT = 90.0  # the wafer's flat faces the 90 degree clip
LONG_AT = 225  # the clip whose screw is too long
LONG_UP = 0.008  # its head stands this far proud
# each screw's slot direction (plate frame, degrees, joint 0, clip parked); the clips are swung in by -90 degrees
SLOT_DEG = {0: 145.0, 45: 20.0, 90: 160.0, 135: 145.0, 180: 75.0, 225: 60.0, 270: 145.0, 315: 110.0}
W_OFF = 0.005 * np.array([math.cos(math.radians(202.5)), math.sin(math.radians(202.5))])  # the wafer starts off centre
SCREW_TIGHT = math.radians(300.0)  # tightened: turned at least 300 degrees clockwise
SCREW_MAX = math.radians(420.0)  # past this, a clip over the wafer cracks it
LONG_TURN = math.radians(90.0)
PITCH = 0.0005  # M3 x 0.5: the tab's travel per turn of its screw (m)
SA_TOUCH = 2 * math.pi * TAB_GAP_REAL / PITCH  # 252 degrees: where the real tab meets the wafer


SIM_GAP = TAB_GAP + 0.1 * W_T  # tab underside to the wafer's colliders: the strips' tops lie 0.1 x W_T under the
# drawn top (strip thickness 0.9 x W_T, resting on the plate), measured in r6: contact began at 325 degrees


def screw_depth(sa):
    """How far (m) a screw turned sa (radians, clockwise) drives its tab down: the gap to the wafer's colliders
    (SIM_GAP) closes over the first 252 degrees, then 0.5 mm a turn as the real M3 x 0.5 screw, so contact is at 252
    degrees and every angle past it clamps as on the real clip (0.07 mm at 300 degrees, 0.23 at 420)."""
    sa = max(float(sa), 0.0)
    return SIM_GAP * min(sa, SA_TOUCH) / SA_TOUCH + PITCH * max(sa - SA_TOUCH, 0.0) / (2 * math.pi)
SLIDE_K = 200000.0  # a swinging clip's slide: a 7 N push on its knob moves the tab 35 um
TAB_K = 20000.0  # the clip tab's spring against the wafer (N/m): 300 degrees presses about 1.3 N, 420 degrees about 4.7 N
SCREW_T0 = 0.005  # a loose screw's running torque in its tapped hole (N m, estimate)
SCREW_T1 = 350.0  # extra torque per metre of clamp (N m/m): about 0.03 N m at 300 degrees, 0.09 N m at 420 (estimates)
# PhysX's joint friction is not a torque: measured on these screws (a spring drive ramped against values 1e-4 to 1e-2,
# static and dynamic set alike) the running resistance is 8.0 N m per unit, linear (0.0084 N m at 0.001, 0.080 at 0.01)
FRICTION_NM = 8.0


def clip_frame(k):
    """Screw (pivot), parked direction and inward direction of clip k, in the plate frame (x, y)."""
    f = math.radians(CLIP_DEG[k])
    P = CLIP_P * np.array([math.cos(f), math.sin(f)])
    return P, np.array([math.sin(f), -math.cos(f)]), -np.array([math.cos(f), math.sin(f)])


def clip_dir(k, th):
    """The tab's direction when clip k is swung in by th (clockwise, radians)."""
    _, t, n = clip_frame(k)
    return math.cos(th) * t + math.sin(th) * n


def clip_tip(k, th):
    return clip_frame(k)[0] + CLIP_L * clip_dir(k, th)


def _clip_joints(k):
    P, t, _ = clip_frame(k)
    yaw = math.atan2(t[1], t[0])
    c = P + t * CLIP_L / 2
    kb = P + t * KNOB_AT
    # the clip's slide: the screw's shank, down through the tab into the plate's tapped hole (the part does not collide
    # with itself); the screw drives it down (see _press_clips), the tab's spring (TAB_K) is its stiffness
    lift = Joint(f"clipz{k}", kind="slide", axis="Z", anchor=(P[0], P[1], TAB_Z), lower=-0.0012, upper=0.0, initial=0.0,
                 stiffness=TAB_K, damping=20.0, target=0.0, mass=0.001,
                 cylinders=[((P[0], P[1], (PL_T - 0.001 + TAB_BOT) / 2), 0.0012, TAB_BOT - PL_T + 0.001, STEEL)])
    clip = Joint(f"clip{k}", kind="hinge", axis="Z", anchor=(P[0], P[1], TAB_Z), lower=-math.pi / 2, upper=0.0,
                 initial=0.0, friction=0.002, damping=0.001, mass=0.002, parent=f"clipz{k}",
                 boxes=[((c[0], c[1], TAB_Z), (CLIP_L + 0.002, CLIP_W, CLIP_T), STEEL, yaw)],
                 cylinders=[((kb[0], kb[1], TAB_TOP + KNOB_H / 2), KNOB_R, KNOB_H, DARK)])
    up = LONG_UP if CLIP_DEG[k] == LONG_AT else 0.0
    s = math.radians(SLOT_DEG[CLIP_DEG[k]])
    nrm = np.array([-math.sin(s), math.cos(s)])  # across the slot
    hz = TAB_TOP + up + HEAD_H / 2
    rz = TAB_TOP + up + HEAD_H + RIDGE[2] / 2
    cyl = [((P[0], P[1], hz), HEAD_R, HEAD_H, STEEL)]
    if up:
        cyl.append(((P[0], P[1], TAB_TOP + up / 2), 0.0019, up, STEEL))
    screw = Joint(f"screw{k}", kind="hinge", axis="Z", anchor=(P[0], P[1], TAB_Z), lower=-10.0, upper=0.05,
                  initial=0.0, friction=SCREW_T0 / FRICTION_NM, damping=0.0005, mass=0.002, parent=f"clip{k}", cylinders=cyl,
                  boxes=[((P[0] + e * (SLOT / 2 + RIDGE[0] / 2) * nrm[0], P[1] + e * (SLOT / 2 + RIDGE[0] / 2) * nrm[1], rz),
                          RIDGE, STEEL, s - math.pi / 2) for e in (-1, 1)]
                  # a white paint mark on one end of one ridge, so the screw's turns can be counted from above
                  + [((P[0] + m * (SLOT / 2 + RIDGE[0] / 2) * nrm[0] + (RIDGE[1] / 2 - 0.0007) * math.cos(s),
                       P[1] + m * (SLOT / 2 + RIDGE[0] / 2) * nrm[1] + (RIDGE[1] / 2 - 0.0007) * math.sin(s),
                       rz + RIDGE[2] / 2 + 0.00015), (0.0013, 0.0013, 0.0003), (0.95, 0.95, 0.95), s) for m in (1,)])
    return [lift, clip, screw]


_holes = [((*_on(d, r), PL_T + 0.00005), (0.0042, 0.0042, 0.0001), math.radians(45 + d))
          for r, n, off in ((0.0340, 6, 30.0), (0.0470, 6, 0.0), (0.0850, 8, 22.5)) for d in
          (off + 360.0 * i / n for i in range(n))]
ST_XY, ST_H = (0.18, 0.0), 0.130
BOSS_R, BOSS_H = 0.020, 0.003
PLATE_START = np.array([*ST_XY, TOP + ST_H])
PLATE = part(
    "sample_plate", pos=tuple(PLATE_START), mass=0.36, friction=0.9,
    cylinders=[((0.0, 0.0, PL_T / 2), PL_R, PL_T, STAINLESS), ((0.0, 0.0, -BOSS_H / 2), BOSS_R, BOSS_H, STAINLESS),
               *[((*clip_frame(k)[0], (PL_T + TAB_BOT) / 2), 0.0022, TAB_BOT - PL_T, STEEL) for k in range(8)]],
    meshes=[_boxes_mesh(_holes, HOLE)],
    joints=[j for k in range(8) for j in _clip_joints(k)],
)
WAFER_START = PLATE_START + [*W_OFF, PL_T + W_T / 2]
WAFER = part("wafer", pos=tuple(WAFER_START), yaw=math.radians(FLAT_AT), mass=0.010, friction=0.6,
             boxes=_strips(), meshes=[_prism(_outline(), -W_T / 2, W_T / 2, SI_BACK), *_pattern()])
STAND = part("loading_stand", pos=(*ST_XY, TOP), fixed=True, friction=0.9,
             boxes=[((0.0, 0.0, (ST_H - 0.009) / 2), (0.050, 0.050, ST_H - 0.009), STAND_C),
                    ((0.0, 0.0, ST_H - BOSS_H - 0.003), (0.120, 0.120, 0.006), STAND_C),
                    *[((0.0285 * math.cos(a), 0.0285 * math.sin(a), ST_H - BOSS_H / 2), (0.016, 0.016, BOSS_H),
                       STAND_C, a) for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]])

# -- the ring holder (loose): aluminium, 220 mm across, 180 mm inside, 4 mm thick, with a 1.5 mm recess under its rim
# (its skirt, 101.5 mm in radius) into which the sample plate fits. It starts on the load lock's holder. Frame: origin
# at the centre of its underside (the face that rests on the plate), +z towards its top. -----------------------------
RG_IN, RG_OUT, RG_T, SK_IN = 0.090, 0.110, 0.004, 0.1015
_segs = [5.0 + 10.0 * i for i in range(36)]
RING_BOXES = (_arc_boxes(RG_IN, RG_OUT, 0.0, RG_T, _segs, 0.0200, ALU)
              + _arc_boxes(SK_IN, RG_OUT, -PL_T, 0.0, _segs, 0.0205, ALU))

# -- the load lock (fixed): a closed chamber, 0.76 m wide, its front at x = 0.36 m, with a viewport on top. Its front
# opening (0.75 x 0.22 m, 100 to 320 mm above the bench) is closed by a door that slides up. Inside, on a centre shaft,
# its holder: two arcs (front and back) that carry the ring holder 170 mm above the bench, between low fences. ----------
LL_X0, LL_X1, LL_Y, LL_Z = 0.360, 0.640, 0.380, 0.350
OPEN_Z0, OPEN_Z1 = 0.100, 0.320
HOLD_XY, HOLD_Z = (0.485, 0.0), 0.170
WALL = 0.006
RING_START = np.array([*HOLD_XY, TOP + HOLD_Z])
RING = part("ring_holder", pos=tuple(RING_START), mass=0.14, friction=0.9, boxes=RING_BOXES)
_arcs = [-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0]
_cx = LL_X0 + (LL_X1 - LL_X0) / 2
_hold = [((HOLD_XY[0] - _cx, 0.0, (HOLD_Z - 0.030) / 2), (0.030, 0.030, HOLD_Z - 0.030), STEEL),
         ((HOLD_XY[0] - _cx, 0.0, HOLD_Z - 0.027), (0.200, 0.024, 0.006), STEEL)]
for side in (0.0, 180.0):
    for d in _arcs:
        a = math.radians(side + d)
        p = _on(side + d, 0.093)
        h = HOLD_Z - 0.024
        _hold.append(((HOLD_XY[0] - _cx + p[0], p[1], h + 0.012), (0.012, 0.0175, 0.024), STEEL, a))
    for d in (-15.0, 0.0, 15.0):
        a = math.radians(side + d)
        p = _on(side + d, 0.1155)
        _hold.append(((HOLD_XY[0] - _cx + p[0], p[1], (HOLD_Z + 0.004) / 2), (0.004, 0.018, HOLD_Z + 0.004), STEEL, a))
_ll_w = LL_X1 - LL_X0
LL_BOXES = [
    ((-_ll_w / 2 + WALL / 2, 0.0, OPEN_Z0 / 2), (WALL, 2 * LL_Y, OPEN_Z0), CHAMBER),  # front, under the opening
    ((-_ll_w / 2 + WALL / 2, 0.0, (OPEN_Z1 + LL_Z) / 2), (WALL, 2 * LL_Y, LL_Z - OPEN_Z1), CHAMBER),  # over it
    *[((WALL / 2, e * (LL_Y - WALL / 2), LL_Z / 2), (_ll_w, WALL, LL_Z), CHAMBER) for e in (-1, 1)],  # sides
    ((_ll_w / 2 - WALL / 2, 0.0, LL_Z / 2), (WALL, 2 * LL_Y, LL_Z), CHAMBER),  # back
]
LOADLOCK = part("load_lock", pos=(_cx, 0.0, TOP), fixed=True, friction=0.8,
                boxes=[*LL_BOXES, *_hold,
                       ((0.0, 0.0, LL_Z - WALL / 2), (_ll_w - 2 * WALL, 2 * LL_Y - 2 * WALL, WALL), VIEWPORT)])

# the door (fixed frame, a slide joint up): an aluminium plate 6 mm thick in front of the opening, with a 12 mm round
# handle bar 25 mm in front of it. It starts shut, the load lock already vented. Lifted to the top, a latch holds it
# open; pulled down off the latch, it slides down onto its seat under its own weight.
DOOR_X, DOOR_Z0, DOOR_Z1 = LL_X0 - 0.005, 0.095, 0.325
HANDLE_X, HANDLE_Z, HANDLE_W = DOOR_X - 0.025, 0.200, 0.012
DOOR_UP = 0.250  # fully open: its bottom edge 25 mm over the opening
DOOR_M = 0.25  # kg
LATCH_K = 400.0  # the top latch's hold (N/m): a pull of about 8 N takes the door off it
LATCH_ON, LATCH_OFF = 0.015, 0.025  # it latches within 15 mm of the top, and lets go once pulled 25 mm down
# the door's twist lock: a black T-lever on a boss on the door's front, right of the handle, 150 mm up when the door is
# shut. Upright (0 degrees) it is locked: its catch holds the door on its seat. A quarter turn clockwise (seen from the
# robot) lays it level and frees the door. It rides up and down with the door.
LOCK_Y, LOCK_H, LOCK_DX = -0.20, 0.150, -0.019  # its bar's centre: across, height (door shut), in front of the door
LOCK_FREE = math.radians(80.0)  # turned past this, the door is free
LOCK_SHUT = math.radians(10.0)  # turned back within this, it is locked
DOOR = part(
    "door", pos=(DOOR_X, 0.0, TOP), fixed=True, friction=1.0,
    boxes=[((0.0, 0.0, 0.003), (0.004, 0.004, 0.006), DARK)],
    joints=[Joint("door", kind="slide", axis="Z", anchor=(0.0, 0.0, DOOR_Z0), lower=0.0, upper=DOOR_UP + 0.005,
                  initial=0.0, damping=6.0, mass=DOOR_M,
                  boxes=[((0.0, 0.0, (DOOR_Z0 + DOOR_Z1) / 2), (0.006, 2 * LL_Y, DOOR_Z1 - DOOR_Z0), ALU),
                         *[((-0.0145, e * 0.064, HANDLE_Z), (0.023, 0.010, HANDLE_W), DARK) for e in (-1, 1)]],
                  cylinders=[((HANDLE_X - DOOR_X, 0.0, HANDLE_Z), HANDLE_W / 2, 0.140, DARK, "Y"),
                             ((-0.009, LOCK_Y, LOCK_H), 0.010, 0.012, DARK, "X")]),
            Joint("lock", kind="hinge", axis="X", anchor=(-0.015, LOCK_Y, LOCK_H), lower=0.0, upper=math.pi / 2,
                  initial=0.0, friction=0.02, damping=0.002, mass=0.02, parent="door",
                  cylinders=[((-0.016, LOCK_Y, LOCK_H), 0.007, 0.004, DARK, "X")],
                  boxes=[((LOCK_DX, LOCK_Y, LOCK_H), (0.008, 0.012, 0.050), DARK)])],
)

# -- the screwdriver (loose) in its holder (fixed): a stubby one, a 0.8 x 5 mm blade on a 5 mm square shaft 52 mm long
# under a 12 mm square handle 14 mm long. Frame: origin at the blade's tip, +z up the shaft, the blade's width along y. It stands
# blade down in its holder, a block with a 6.5 mm square socket 12 mm deep (snug: the screwdriver stands in it), the tip 48 mm above the bench. ---------------
BLADE = (0.0008, 0.0050, 0.0040)
SH_W, SH_L, H_W, H_L = 0.0050, 0.052, 0.012, 0.014
SD_XY = (0.120, -0.170)
SOCK_Z, SOCK_D, SOCK_W = 0.060, 0.012, 0.0065  # the holder's top above the bench, the socket's depth and width
SD_START = np.array([*SD_XY, TOP + SOCK_Z - SOCK_D])
SCREWDRIVER = part(
    "screwdriver", pos=tuple(SD_START), mass=0.035, friction=1.0,
    boxes=[((0.0, 0.0, BLADE[2] / 2), BLADE, STEEL),
           ((0.0, 0.0, BLADE[2] + (SH_L - BLADE[2]) / 2), (SH_W, SH_W, SH_L - BLADE[2]), STEEL),
           ((0.0, 0.0, SH_L + H_L / 2), (H_W, H_W, H_L), RED)],
)
_hb = (0.030 - SOCK_W) / 2  # the socket's walls
HOLDER = part(
    "sd_holder", pos=(*SD_XY, TOP), fixed=True, friction=0.6,
    boxes=[((0.0, 0.0, (SOCK_Z - SOCK_D) / 2), (0.030, 0.030, SOCK_Z - SOCK_D), DARK),
           *[((e * (SOCK_W + _hb) / 2, 0.0, SOCK_Z - SOCK_D / 2), (_hb, 0.030, SOCK_D), DARK) for e in (-1, 1)],
           *[((0.0, e * (SOCK_W + _hb) / 2, SOCK_Z - SOCK_D / 2), (SOCK_W, _hb, SOCK_D), DARK) for e in (-1, 1)]],
)

# -- the load lock's light tower (fixed), on its top near the front left corner: red (error: call the staff), orange
# (warning: tell the staff before going on), green (a process has finished) and blue (a process is running: keep the
# door shut), top to bottom. Each lamp is a dim coloured band; a lit lamp's bright lens slides 14 mm out of the band
# towards the robot. Here blue is lit while the load lock finishes venting, then green, 8 s after the start. -----------
TOWER_XY = (LL_X0 + 0.040, 0.30)
VENT_S = 8.0  # the venting finishes (light blue to green) this long after the start
LENS_IN = 0.022  # a lens slides this far into its band when off
SEG_H, SEG_R = 0.028, 0.016
_dim = {"blue": (0.06, 0.12, 0.40), "green": (0.07, 0.30, 0.10), "orange": (0.42, 0.24, 0.05), "red": (0.38, 0.08, 0.07)}
_seg_z = {c: 0.010 + SEG_H / 2 + 0.030 * k for k, c in enumerate(("blue", "green", "orange", "red"))}


def _lens(c, color, lit):
    z = _seg_z[c]
    return Joint(f"lens_{c}", kind="slide", axis="X", anchor=(0.0, 0.0, z), lower=0.0, upper=LENS_IN,
                 initial=0.0 if lit else LENS_IN, target=0.0 if lit else LENS_IN, stiffness=3000.0, damping=5.0,
                 mass=0.005, boxes=[((-0.022, 0.0, z), (0.016, 0.026, 0.024), color)])


TOWER = part("light_tower", pos=(*TOWER_XY, TOP + LL_Z), fixed=True, friction=0.8,
             cylinders=[((0.0, 0.0, 0.005), 0.010, 0.010, DARK),
                        *[((0.0, 0.0, z), SEG_R, SEG_H, _dim[c]) for c, z in _seg_z.items()],
                        ((0.0, 0.0, _seg_z["red"] + SEG_H / 2 + 0.004), SEG_R, 0.008, DARK)],
             joints=[_lens("blue", LAMP_BLUE, True), _lens("green", LAMP_GREEN, False)])

OBJECTS = [STAND, PLATE, WAFER, RING, LOADLOCK, DOOR, SCREWDRIVER, HOLDER, TOWER]
PLANCHECK_BOXES = [
    *[(f"ll{i}", (_cx + c[0], c[1], TOP + c[2]), s) for i, (c, s, _) in enumerate(LL_BOXES)],
    ("viewport", (_cx, 0.0, TOP + LL_Z - WALL / 2), (_ll_w, 2 * LL_Y, WALL)),
    ("holder_back", (HOLD_XY[0] + 0.093, 0.0, TOP + HOLD_Z / 2), (0.014, 0.060, HOLD_Z)),
    ("stand_plate", (*ST_XY, TOP + ST_H - BOSS_H - 0.003), (0.120, 0.120, 0.006)),
    ("plate_ring_on_stand", (*ST_XY, TOP + ST_H + 0.004), (0.150, 0.150, 0.008)),  # until the stack is lifted
    ("sd_holder", (*SD_XY, TOP + SOCK_Z / 2), (0.030, 0.030, SOCK_Z)),
    ("door_up", (DOOR_X, 0.0, TOP + (DOOR_Z0 + DOOR_Z1) / 2 + DOOR_UP), (0.006, 2 * LL_Y, DOOR_Z1 - DOOR_Z0)),
]

# -- scoring ----------------------------------------------------------------------------------------------------------
DOOR_SHUT = 0.003  # the door counts as shut within 3 mm of its seat
CLIP_IN = math.radians(70.0)
BITE = 0.0020  # a clip's tip at least 2 mm inside the wafer's outline holds it
PRESS_MIN = 0.00003  # and its tab presses the wafer: the larger of its spring deflection and tab/wafer overlap is at
#   least 0.03 mm, with spring deflection at least 0.005 mm; the overlap includes contact compliance and is not a
#   measured clamp force
GAP_MAX = 150.0
CENTRED = 0.002
SEAT_XY, SEAT_Z, SEAT_DEG, PLATE_ON = 0.004, 0.003, 5.0, 0.003
TOUCH = 0.004
RELEASE = 0.08  # every positive score: all six jaw and pad points this far (m) from every task surface
OPEN_MAX = 120.0  # the load lock's door may be open (over 3 mm off its seat) this long in all (s)
SETTLE_STEPS, SETTLE_TOL = 20, 0.001  # all moving task parts: every material point under 1 mm over the last 21 samples
_state = {}
_hist = []


@on_reset
def _reset_state(env):
    _state.clear()
    _state.update(step=0, viol=set(), tight={}, mode=None, settled=False, far=True, gap=360.0, centred=False,
                  stack=False, door=0.0, sd_in=False, opened=False, open_s=0.0, green=False, lock=0.0)
    _hist.clear()
    _door_drive(env, "locked")
    _light(env, False)
    import torch

    plate = env.unwrapped.scene["sample_plate"]
    ids = plate.find_joints("clip.*_joint")[0]
    plate.write_joint_stiffness_to_sim(torch.zeros((1, len(ids)), device=plate.device), joint_ids=ids)
    _press_clips(env, [0.0] * 8, [0.0] * 8, [0.0] * 8)


def _press_clips(env, z, torque, swing):
    """Drive each clip's slide to z (m, negative: down, from its screw's turns) and set its screw's turning resistance
    (N m, from how hard the tab presses)."""
    import torch

    plate = env.unwrapped.scene["sample_plate"]
    zi = plate.find_joints([f"clipz{k}_joint" for k in range(8)], preserve_order=True)[0]
    zt = torch.tensor([z], dtype=torch.float32, device=plate.device)
    plate.set_joint_position_target(zt, joint_ids=zi)
    # the slide is held stiff (SLIDE_K) while its clip is out or swinging, so a finger pressing the clip's knob as it
    # swings cannot push the tab onto the wafer; once the clip is swung in (CLIP_IN) the tab's own spring (TAB_K) presses
    k = [TAB_K if a >= CLIP_IN else SLIDE_K for a in swing]
    plate.write_joint_stiffness_to_sim(torch.tensor([k], dtype=torch.float32, device=plate.device), joint_ids=zi)
    si = plate.find_joints([f"screw{k}_joint" for k in range(8)], preserve_order=True)[0]
    f = torch.tensor([torque], dtype=torch.float32, device=plate.device) / FRICTION_NM  # N m to PhysX friction
    plate.write_joint_friction_coefficient_to_sim_index(joint_friction_coeff=f, joint_dynamic_friction_coeff=f,
                                                        joint_ids=si)


def press(p, k, w=None):
    """How far (m) clip k's tab presses into the wafer: the slide held up short of its screw (the tab's spring) plus,
    when the wafer is under the tab (``w``), how far the wafer itself has given way (its contact deflection). Both are
    camera-visible in principle: the gap under the screw's head and the wafer's height. Zero without real contact."""
    sa = -float(p.joints.get(f"screw{k}", 0.0))
    spring = max(0.0, float(p.joints.get(f"clipz{k}", 0.0)) + screw_depth(sa))
    if w is None:
        return spring
    if spring < 0.000005:  # the slide is not held up at all: the tab touches nothing
        return 0.0
    strip_top = float(in_plate(p, w.pos)[2]) + 0.45 * W_T  # the wafer colliders' top face, where the wafer now sits
    tab_bot = TAB_BOT + float(p.joints.get(f"clipz{k}", 0.0))
    return max(spring, strip_top - tab_bot)


def _light(env, green):
    """The light tower: the blue lens out (venting) or the green one (done)."""
    import torch

    tw = env.unwrapped.scene["light_tower"]
    for c, up in (("blue", not green), ("green", green)):
        ids = tw.find_joints(f"lens_{c}_joint")[0]  # the sim names each joint <name>_joint
        tw.set_joint_position_target(torch.full((1, len(ids)), 0.0 if up else LENS_IN, device=tw.device),
                                     joint_ids=ids)


LOCK_K = 20000.0  # the twist lock's hold on the shut door (N/m)


def _clamp_clip(env, k, th):
    """A tightened screw clamps its clip: from then on the clip holds its angle (a stiff spring)."""
    import torch

    plate = env.unwrapped.scene["sample_plate"]
    ids = plate.find_joints(f"clip{k}_joint")[0]
    plate.set_joint_position_target(torch.full((1, 1), -th, device=plate.device), joint_ids=ids)
    plate.write_joint_stiffness_to_sim(torch.full((1, 1), 50.0, device=plate.device), joint_ids=ids)


def _door_drive(env, mode):
    """What holds the door: "locked" (the twist lock keeps it on its seat), "latched" (its top latch holds it up: a
    spring to the top; pulled down off it, it is free) or "free" (it slides down under its own weight)."""
    import torch

    if _state.get("mode") == mode:
        return
    door = env.unwrapped.scene["door"]
    ids = door.find_joints("door_joint")[0]  # the slide (the twist lock is the door's other joint)
    target, k = {"locked": (0.0, LOCK_K), "latched": (DOOR_UP, LATCH_K), "free": (0.0, 0.0)}[mode]
    door.set_joint_position_target(torch.full((1, 1), target, device=door.device), joint_ids=ids)
    door.write_joint_stiffness_to_sim(torch.full((1, 1), k, device=door.device), joint_ids=ids)
    _state["mode"] = mode


def _tips(env):
    """Both hands' pad midpoints (the runtime's own pad datum) and their two jaw points (world, env frame), each jaw
    from its own finger joint (joint7 and joint8 separately)."""
    from yam_arena import runtime as _rt

    b = env.unwrapped
    o = b.scene.env_origins[0].cpu().numpy()
    out = []
    for n in ("left_robot", "right_robot"):
        robot = b.scene[n]
        mid = _rt.pad_tips_world(robot) - o
        g = _rt.link_poses(robot)["gripper"]
        ids = robot.find_joints(["joint7", "joint8"], preserve_order=True)[0]
        q7, q8 = (abs(float(v)) for v in robot.data.joint_pos.torch[0, ids].cpu().numpy())
        out += [mid, mid - q7 * g[:3, 0], mid + q8 * g[:3, 0]]
    return out


def _seg_dist(x, a, b):
    """Distance of point x from segment a-b."""
    x, a, b = (np.asarray(v, float) for v in (x, a, b))
    ab = b - a
    t = float(np.clip(np.dot(x - a, ab) / max(float(np.dot(ab, ab)), 1e-12), 0.0, 1.0))
    return float(np.linalg.norm(x - (a + t * ab)))


def _clearance(p, w, r, d, s, tips) -> float:
    """Smallest distance (m) from any of the six hand points to any task surface: the plate with its clips and screws,
    the wafer, the ring holder (the carrier), the screwdriver, the door with its handle and twist lock, and the fixed
    apparatus (the load lock, the stand, the screwdriver holder and the light tower). Bodies that move or turn are
    bounded by spheres and segments around their current poses (conservative: never larger than the true clearance)."""
    door = float(d.joints.get("door", 0.0))
    hp, kp = handle_pos(door), np.array([DOOR_X + LOCK_DX, LOCK_Y, TOP + LOCK_H + door])
    sd_top = np.asarray(s.pos, float) + s.rotation() @ np.array([0.0, 0.0, SH_L + H_L])
    spheres = [(np.asarray(p.pos, float), PL_R + 0.006),  # the plate, clips, screws and knobs to the tabs' reach
               (np.asarray(w.pos, float), W_R),
               (np.asarray(r.pos, float), RG_OUT),
               (hp, 0.025), (kp, 0.030)]  # the door's handle and its twist lock's bar
    boxes = [((DOOR_X, 0.0, TOP + (DOOR_Z0 + DOOR_Z1) / 2 + door), (0.004, LL_Y, (DOOR_Z1 - DOOR_Z0) / 2)),
             (((LL_X0 + LL_X1) / 2, 0.0, TOP + LL_Z / 2), ((LL_X1 - LL_X0) / 2, LL_Y, LL_Z / 2)),
             ((*ST_XY, TOP + ST_H / 2), (0.080, 0.080, ST_H / 2)),
             ((*SD_XY, TOP + SOCK_Z / 2), (0.020, 0.020, SOCK_Z / 2)),
             ((*TOWER_XY, TOP + LL_Z + 0.045), (0.016, 0.016, 0.055))]
    best = 9.0
    for t in tips:
        t = np.asarray(t, float)
        for c, rad in spheres:
            best = min(best, float(np.linalg.norm(t - c)) - rad)
        best = min(best, _seg_dist(t, np.asarray(s.pos, float), sd_top) - 0.009)  # the screwdriver, tip to handle
        for c, h in boxes:
            best = min(best, float(np.linalg.norm(np.maximum(np.abs(t - np.asarray(c, float)) - np.asarray(h, float),
                                                             0.0))))
    return best


def _still_points(p, w, r, d, s) -> np.ndarray:
    """Material points on every moving task part (world, m): centre and two rim points of the plate, the wafer and the
    ring holder (so a turn at a fixed centre counts as movement), each clip's tab tip with its slide's height and a
    slot point on each screw (so swings, screw turns and tab heights count), the screwdriver's tip and handle end, the
    door's handle, and a bar point on the twist lock turning with its angle."""
    Rp, Rw, Rr, Rs = p.rotation(), w.rotation(), r.rotation(), s.rotation()
    pp, pw, pr, ps = (np.asarray(v.pos, float) for v in (p, w, r, s))
    door = float(d.joints.get("door", 0.0))
    lock = float(d.joints.get("lock", 0.0))
    kp = np.array([DOOR_X + LOCK_DX, LOCK_Y, TOP + LOCK_H + door])
    pts = [pp, pp + Rp @ np.array([PL_R, 0.0, 0.0]), pp + Rp @ np.array([0.0, PL_R, 0.0]),
           pw, pw + Rw @ np.array([W_R, 0.0, 0.0]), pw + Rw @ np.array([0.0, W_R, 0.0]),
           pr, pr + Rr @ np.array([RG_OUT, 0.0, 0.0]), pr + Rr @ np.array([0.0, RG_OUT, 0.0]),
           ps, ps + Rs @ np.array([0.0, 0.0, SH_L + H_L]),
           handle_pos(door), kp + 0.02 * np.array([0.0, math.cos(lock), math.sin(lock)])]
    for k in range(8):
        th = -float(p.joints.get(f"clip{k}", 0.0))
        sa = -float(p.joints.get(f"screw{k}", 0.0))
        z = TAB_Z + float(p.joints.get(f"clipz{k}", 0.0))
        tip = clip_tip(k, th)
        pts.append(pp + Rp @ np.array([tip[0], tip[1], z]))
        P = clip_frame(k)[0]
        pts.append(pp + Rp @ np.array([P[0] + 0.003 * math.cos(sa), P[1] + 0.003 * math.sin(sa), z]))
    return np.asarray(pts, float)


def tilt_deg(R, down=False):
    c = float(R[2, 2]) * (-1.0 if down else 1.0)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def in_plate(p, x):
    """The point x (world) in the plate's frame."""
    return p.rotation().T @ (np.asarray(x, float) - np.asarray(p.pos, float))


def wafer_seated(p, w):
    q = in_plate(p, w.pos)
    return math.hypot(q[0], q[1]) <= 0.008 and PL_T - 0.0005 <= q[2] <= PL_T + W_T + 0.003


def holding_clips(p, w):
    """Plate-frame angles of the clips that hold the wafer: swung in past 70 degrees, screw tightened after it was
    swung in (and not swung since), its tab pressing (PRESS_MIN), tip at least 2 mm inside the wafer's outline."""
    Rp, Rw = p.rotation(), w.rotation()
    out = []
    for k in range(8):
        th = -float(p.joints.get(f"clip{k}", 0.0))
        lock = _state["tight"].get(k)
        if th < CLIP_IN or lock is None or abs(th - lock) > math.radians(10.0) or press(p, k, w) < PRESS_MIN:
            continue
        tip = clip_tip(k, th)
        world = np.asarray(p.pos, float) + Rp @ np.array([tip[0], tip[1], TAB_Z])
        q = Rw.T @ (world - np.asarray(w.pos, float))
        if wafer_inside(q) >= BITE:
            out.append(CLIP_DEG[k])
    return out


def largest_gap(degs):
    if not degs:
        return 360.0
    s = sorted(degs)
    return max((s[(i + 1) % len(s)] - s[i]) % 360 or 360.0 for i in range(len(s)))


def stack_loaded(p, r):
    """The ring holder face down on the load lock's holder, carrying the plate face down and centred on it."""
    rp, pp = np.asarray(r.pos, float), np.asarray(p.pos, float)
    ring_ok = (tilt_deg(r.rotation(), down=True) <= SEAT_DEG
               and math.hypot(rp[0] - HOLD_XY[0], rp[1] - HOLD_XY[1]) <= SEAT_XY
               and abs(rp[2] - (TOP + HOLD_Z + RG_T)) <= SEAT_Z)
    plate_ok = (tilt_deg(p.rotation(), down=True) <= SEAT_DEG
                and math.hypot(pp[0] - rp[0], pp[1] - rp[1]) <= PLATE_ON
                and abs(pp[2] - (TOP + HOLD_Z + RG_T + PL_T)) <= SEAT_Z)
    return ring_ok and plate_ok


def handle_pos(door):
    return np.array([HANDLE_X, 0.0, TOP + HANDLE_Z + door])


def sd_inside(s):
    x = np.asarray(s.pos, float)
    return LL_X0 < x[0] < LL_X1 and abs(x[1]) < LL_Y and x[2] < TOP + LL_Z


@on_step
def _watch(env):
    """Latch the violations (the long screw tightened, the wafer off its seat on the plate, the plate or ring dropped, a
    fingertip at the wafer's pattern, a screw too tight, the door open too long or opened on a blue light), work the
    light, the door's lock and latch, and follow the clips' screws and the end state."""
    S = labkit._SPECS
    st = _state
    st["step"] += 1
    p, w, r, d, s = (labkit._part_state(env, S[k]) for k in
                     ("sample_plate", "wafer", "ring_holder", "door", "screwdriver"))
    door = float(d.joints.get("door", 0.0))
    lock = float(d.joints.get("lock", 0.0))
    st["door"], st["lock"] = door, lock
    if not st["green"] and st["step"] * DT >= VENT_S:
        st["green"] = True
        _light(env, True)
    if door > DOOR_SHUT and not st["green"]:
        st["viol"].add("the door lifted while the light was blue")
    if door > DOOR_SHUT:
        st["open_s"] += DT
        if st["open_s"] > OPEN_MAX:
            st["viol"].add("the load lock left open too long")
    if lock < LOCK_FREE and door <= 0.005:
        _door_drive(env, "locked")
    elif door >= DOOR_UP - LATCH_ON:
        _door_drive(env, "latched")
        st["opened"] = True
    elif st["mode"] == "locked" or door < DOOR_UP - LATCH_OFF:
        _door_drive(env, "free")
    tips = _tips(env)
    # screws: each drives its clip's tab down 0.5 mm a turn; the tab's press sets the screw's resistance
    zs, ts = [], []
    for k in range(8):
        sa = max(0.0, -float(p.joints.get(f"screw{k}", 0.0)))
        zs.append(-min(screw_depth(sa), 0.0012))
        ts.append(SCREW_T0 + SCREW_T1 * press(p, k, w))
    _press_clips(env, zs, ts, [-float(p.joints.get(f"clip{k}", 0.0)) for k in range(8)])
    st["press"] = [press(p, k, w) for k in range(8)]
    for k in range(8):
        sa = -float(p.joints.get(f"screw{k}", 0.0))
        if CLIP_DEG[k] == LONG_AT:
            if sa >= LONG_TURN:
                st["viol"].add("the long screw tightened")
            continue
        th = -float(p.joints.get(f"clip{k}", 0.0))
        if sa >= SCREW_TIGHT and k not in st["tight"] and th >= CLIP_IN:
            st["tight"][k] = th
            _clamp_clip(env, k, th)
        if sa > SCREW_MAX and th >= CLIP_IN:
            st["viol"].add("a screw too tight: the wafer cracked")
    # the wafer, the plate and the ring
    if not wafer_seated(p, w):
        st["viol"].add("wafer off its seat on the plate")
    if min(float(p.pos[2]), float(r.pos[2])) < TOP + 0.10:
        st["viol"].add("plate or ring dropped")
    Rw, pw = w.rotation(), np.asarray(w.pos, float)
    for tip in tips:
        q = Rw.T @ (np.asarray(tip, float) - pw)
        if abs(q[2]) < TOUCH and math.hypot(q[0], q[1]) < W_R - EDGE_EXCL:
            st["viol"].add("fingertip at the wafer's pattern")
    q = in_plate(p, w.pos)
    st["centred"] = math.hypot(q[0], q[1]) <= CENTRED
    st["gap"] = largest_gap(holding_clips(p, w))
    st["stack"] = stack_loaded(p, r)
    st["sd_in"] = sd_inside(s)
    st["far"] = _clearance(p, w, r, d, s, tips) >= RELEASE
    _hist.append(_still_points(p, w, r, d, s))
    del _hist[:-(SETTLE_STEPS + 1)]
    if len(_hist) > SETTLE_STEPS:
        h = np.asarray(_hist)  # samples x points x 3: every point's movement between any two samples under 1 mm
        st["settled"] = float(np.linalg.norm(h[:, None] - h[None, :], axis=3).max()) < SETTLE_TOL
    else:
        st["settled"] = False


def score(objects):
    """0 after any violation (the long screw turned over 90 degrees, a clip's screw over 420 degrees, the door open over
    120 s in all, the wafer off its seat on the plate, the plate or ring dropped, a fingertip within 4 mm of the wafer's
    pattern), if the screwdriver is in the load lock, or unless both hands are released (all six jaw and pad points
    80 mm or more from every task surface: the plate with its clips and screws, the wafer, the ring holder, the
    screwdriver, the door with its handle and lock, and the fixed apparatus) and every moving task part, the clips,
    screws, screwdriver, door and lock included, moved less than 1 mm over the last 21 samples at 20 Hz (1.0 s).
    Otherwise: 0.25 if the wafer is centred
    on the plate (within 2 mm) and held by clips (swung in past 70 degrees, then their screws tightened 300 degrees, tabs
    pressing the wafer, tips
    at least 2 mm inside the wafer's outline) that leave no gap over 150 degrees round it; 0.5 if also the ring holder
    lies face down on the load lock's holder (within 5 degrees and 4 mm) carrying the plate face down (centred on it
    within 3 mm); 0.75 if also the door is shut (within 3 mm of its seat); 1.0 if also the door is locked (its twist
    lock turned back within 10 degrees)."""
    st = _state
    if st.get("viol") or st.get("sd_in"):
        return 0.0
    if not (st.get("settled", False) and st.get("far", False)):
        return 0.0
    if not st.get("centred") or st.get("gap", 360.0) >= GAP_MAX:
        return 0.0
    if not st.get("stack"):
        return 0.25
    if st.get("door", 1.0) > DOOR_SHUT:
        return 0.5
    return 1.0 if st.get("lock", 1.0) <= LOCK_SHUT else 0.75


SUCCESS_HOLD_S = 2.0


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    p = np.asarray(objects["sample_plate"].pos, float)
    return float(math.hypot(p[0] - HOLD_XY[0], p[1] - HOLD_XY[1]))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# Right: pushes the wafer to the plate's centre by its edge, then lifts the load lock's door until it latches. Both: take
# the ring holder off the load lock's holder and set it on the plate; right shuts the door again. Left swings in the 0 and
# 135 degree clips, right the 270 degree one (the 90 degree one faces the flat; the 225 degree one has the long screw).
# Right takes the screwdriver and tightens the three screws, stroke by stroke, stopping once a screw has turned about
# 300 to 360 degrees (counted from its slot), then puts it back. Right opens the door; both pinch the plate and ring by
# their side edges, lift them, roll them over together, carry them into the load lock and set them on its holder. Right
# pulls the door down off its latch; both move away. Aims are trimmed only by what a camera sees: the wafer's, plate's,
# ring's and screwdriver tip's positions, and the screw's slot and the blade between strokes.
from yam_arena import ik as _ik  # noqa: E402
from yam_arena.constants import READY as _READY  # noqa: E402
from yam_arena.constants import RIGHT_BASE_POS as _RIGHT_BASE  # noqa: E402


def _ik_err(pe, re):
    return pe * 1000.0 + math.degrees(re)


FREE_MOVES = ("wrist turned", "hand turned", "up and clear")


def _endpoints(plan):
    """Joint angles for every waypoint's end, chained: warm-started from the previous one, and, where that solve misses,
    the closest of several seeded solves that reach it (so the wrist turns the way the waypoints mean)."""
    rng = np.random.default_rng(7)
    seeds = [np.array(_READY, float)] + [np.array(_READY, float) + rng.normal(0.0, 0.8, 6) for _ in range(10)]
    q_prev, out, switch, forced = plan.q.copy(), [], set(), set()
    for i, w in enumerate(plan.waypoints):
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, q_prev, base_pos=plan.base_pos, iters=300)
        if any(m in w.label for m in FREE_MOVES):
            forced.add(i)  # a free-space move: always a joint-space blend
        if _ik_err(pe, re) > 1.5:
            switch.add(i)
            best = (_ik_err(pe, re), 0.0, q)
            for s in [q_prev + rng.normal(0.0, 0.3, 6) for _ in range(4)] + seeds:
                qs, ps, rs = _ik.solve(pos, quat, s, base_pos=plan.base_pos, iters=200)
                e, d = _ik_err(ps, rs), float(np.max(np.abs(qs - q_prev)))
                if (e < 1.0 and (best[0] >= 1.0 or d < best[1])) or (best[0] >= 1.0 and e < best[0]):
                    best = (e, d, qs)
            q = best[2]
        elif (float(np.max(np.abs(q[3:5] - q_prev[3:5]))) > math.radians(70.0)
              or float(np.max(np.abs(q - q_prev))) > math.radians(100.0)):
            switch.add(i)  # reached, but by turning the wrist over
        out.append(q)
        q_prev = q
    # then backwards: where the arm changes its wrist's branch between two waypoints, move the change earlier if the
    # earlier waypoint can be reached in the later branch (so it happens in free space, before an approach)
    for i in range(len(out) - 1, 0, -1):
        if i not in switch or i in forced:
            continue
        w = plan.waypoints[i - 1]
        pos, quat = np.asarray(w.pos, float), _ik.matrix_to_quat_xyzw(_demo.grip_rotation(w.yaw))
        q, pe, re = _ik.solve(pos, quat, out[i], base_pos=plan.base_pos, iters=300)
        if (_ik_err(pe, re) < 1.5 and float(np.max(np.abs(q - out[i])))
                < float(np.max(np.abs(out[i - 1] - out[i]))) - math.radians(10.0)):
            out[i - 1] = q
            switch.discard(i)
            switch.add(i - 1)
    return out, switch | forced


def _seg(plan, t):
    acc = 0.0
    for i, w in enumerate(plan.waypoints):
        if t < acc + w.duration:
            return i, _ik.smoothstep((t - acc) / w.duration)
        acc += w.duration
    return len(plan.waypoints) - 1, 1.0


def _action7(self, t):
    """[j1..j6, grip] at time t: IK warm-started from the previous step, or, where that misses, from the joint-space
    blend of the segment's end solutions. It also sets the plan clock the trims run on."""
    _demo._plan_clock = float(t)
    if getattr(self, "_qends", None) is None:
        self._qends, self._switch = _endpoints(self)
    pos, rot, finger, label = self.target(t)
    quat = _ik.matrix_to_quat_xyzw(rot)
    i, s = _seg(self, t)
    q0 = self._qends[i - 1] if i > 0 else self._q_start
    seed = q0 + s * (self._qends[i] - q0)
    if i in self._switch:
        # the wrist changes branch on this (free-space) segment: follow the joint-space blend
        self.q = seed
        grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
        return np.r_[seed, grip], label
    q, pe, re = _ik.solve(pos, quat, self.q, base_pos=self.base_pos, iters=80)
    if _ik_err(pe, re) > 2.0:
        q2, p2, r2 = _ik.solve(pos, quat, seed, base_pos=self.base_pos, iters=80)
        if _ik_err(p2, r2) < _ik_err(pe, re) - 1.0:
            q, pe, re = q2, p2, r2
    self.q = q
    self.worst_ik_mm = max(self.worst_ik_mm, pe * 1000)
    self.worst_ik_deg = max(self.worst_ik_deg, float(np.degrees(re)))
    grip = float(np.clip(finger / FINGER_OPEN, 0.0, 1.0)) if FINGER_OPEN != FINGER_CLOSED else 1.0
    return np.r_[q, grip], label


_prev_init = _demo.ArmPlan.__init__
if not getattr(_prev_init, "_eb2", False):
    def _init(self, *a, _prev=_prev_init, **k):
        _prev(self, *a, **k)
        self._q_start, self._qends = self.q.copy(), None

    _init._eb2 = True
    _demo.ArmPlan.__init__ = _init
    _action7._clock = True
    _demo.ArmPlan.action7 = _action7


class Orient(float):
    """A Waypoint ``yaw`` that carries a full TCP rotation."""

    def __new__(cls, rot):
        obj = float.__new__(cls, 0.0)
        obj.rot = np.asarray(rot, float)
        return obj


_prev_grip = _demo.grip_rotation
if not getattr(_prev_grip, "_rot", False):
    def _grip(yaw=0.0):
        return yaw.rot if hasattr(yaw, "rot") else _prev_grip(yaw)

    _grip._rot = True
    _demo.grip_rotation = _grip

TIP_ABOVE_TCP = 0.013
LEFT_PARK = (0.20, 0.22, TOP + 0.22)
RIGHT_PARK = (0.20, -0.22, TOP + 0.22)
OPEN_W = -0.014
PINCH = -0.0012  # squeezing 5.5 mm: plate and ring, or the ring's rim and skirt
SD_OPEN, SD_HOLD = -0.030, -0.0012  # round the 5 mm shaft (wide open past the 12 mm handle)
BAR_OPEN, BAR_HOLD = -0.016, -0.0045  # round the 12 mm handle bar
SD_GRIP = 0.018  # the fingertips' ends this far above the blade's tip
CLIPS_USED = {"left": (0, 3), "right": (6,)}  # 0, 135 and 270 degrees
STROKE_TOP, STROKE, STROKES = 145.0, 190.0, 3  # hand yaw at the start of each tightening stroke, and how far it turns
HAND_HI, HAND_LO = 168.0, -52.0  # how far the wrist (joint6, +-120 degrees) turns the hand at the screws (yaw, degrees)
J6_SIGN, J6_REACH = -1.0, math.radians(120.0)  # joint6 turns against the hand's yaw, to its limit
REACH_SLACK = math.radians(4.0)  # a start up to this far past the wrist's limit still lines the blade up well enough
SCREW_AIM, STROKE_PLAY = 355.0, 40.0  # aim each screw at 355 degrees; a stroke turns the screw ~28 degrees less than the hand
FLIP = {"right": True, "left": False}
ROLL_SIGN = -1.0
LIFT_Z = 0.27  # the stack's centre this high (above the bench) while it is rolled over
STACK_IN = 0.092  # pad tips this far out from the centre when pinching the plate and ring
RING_IN = 0.104  # and when pinching the ring alone (its rim and skirt)
PUSH_DIR = -W_OFF / np.linalg.norm(W_OFF)

# trims: each a 3-vector added to the TCPs of _Dyn waypoints with its key; integrated in windows from seen positions
_ct = {"trim": {}, "wins": [], "last": {}, "looks": [], "turn": {}, "ang": {}, "freeze": [], "vis": {},
       "strokes": [], "engaged": {}, "arm": np.zeros(3), "want": None, "in_stroke": False, "aligns": [],
       "yawfix": {}, "c6": {}}


@on_reset
def _reset_ct(env):
    _ct["trim"] = {}
    _ct["last"] = {}
    _ct["turn"] = {}
    _ct["ang"] = {}


def _wrap180(d):
    return (d + 90.0) % 180.0 - 90.0


@on_step
def _look_at_slot(env):
    """Between strokes, with the blade lifted just over the screw, look at the slot and at the blade (both visible from
    above) and note how far the blade must turn to line up with the slot again (degrees, -90 to 90)."""
    t = getattr(_demo, "_plan_clock", 0.0)
    for t0, t1, k, n in _ct["looks"]:
        if t0 <= t < t1:
            p = labkit._part_state(env, labkit._SPECS["sample_plate"])
            Rp = p.rotation()
            slot = (math.degrees(math.atan2(Rp[1, 0], Rp[0, 0])) + SLOT_DEG[CLIP_DEG[k]]
                    + math.degrees(float(p.joints.get(f"clip{k}", 0.0)) + float(p.joints.get(f"screw{k}", 0.0))))
            Rs = labkit._part_state(env, labkit._SPECS["screwdriver"]).rotation()
            blade = math.degrees(math.atan2(Rs[1, 1], Rs[0, 1]))
            _ct["turn"][(k, n)] = _wrap180(slot - blade)
            # Count the screw's turns from what is seen: a white mark on its head shows which way it faces relative
            # to its clip (worked out here from the hinges: the same direction a camera sees). A stroke turns it less
            # than a full turn, so the turn since the last look is the change in that direction, read in -20 to
            # 340 degrees.
            vis = (-math.degrees(float(p.joints.get(f"screw{k}", 0.0)))) % 360.0
            prev_vis = _ct["vis"].get((k, n - 1), 0.0)  # loose and untouched at the start, its mark at 0
            prev_ang = _ct["ang"].get((k, n - 1), 0.0)
            d = (vis - prev_vis) % 360.0
            if d > 340.0:
                d -= 360.0
            _ct["vis"][(k, n)], _ct["ang"][(k, n)] = vis, prev_ang + d


def _hand_turn(a):
    """How far a stroke turns the hand (degrees) on a screw counted at ``a`` degrees: none once it is tight, else
    enough to bring it near SCREW_AIM."""
    if a >= math.degrees(SCREW_TIGHT):
        return 0.0
    return max(0.0, min(STROKE, SCREW_AIM - a + STROKE_PLAY))


class _SlotYaw(float):
    """A Waypoint ``yaw`` for a later stroke: the planned hand yaw shifted so that the blade lines up with the slot as
    last seen (by a multiple of 180 degrees, kept near the stroke's planned start), and the stroke shortened so that the
    screw ends near SCREW_AIM (none at all once it is tight)."""

    def __new__(cls, k, frac, start, end, prev=None):  # k: (clip, look); frac: how far through the stroke
        obj = float.__new__(cls, 0.0)
        obj.k, obj.frac, obj.start, obj.end, obj.prev = k, frac, start, end, prev
        return obj

    def reach(self):
        """The hand yaws (lo, hi) the wrist reaches over this screw: worked out when the demo is planned, from the arm's
        kinematics and joint6's limits (no sensor reading)."""
        c = _ct["c6"].get(self.k[0])
        if c is None:
            return math.radians(HAND_LO), math.radians(HAND_HI)
        a, b = (J6_REACH - c) / J6_SIGN, (-J6_REACH - c) / J6_SIGN
        return min(a, b), max(a, b)

    def s0(self):
        """The hand's yaw at the start of this stroke: turned to line the blade up with the slot, by whichever multiple
        of 180 degrees starts it highest within the wrist's reach, for the longest stroke."""
        d = _ct["turn"].get(self.k)
        if d is None:
            return self.start
        s = self.prev_end() + math.radians(d)
        return s + math.pi * math.floor((self.reach()[1] + REACH_SLACK - s) / math.pi)

    def turn(self):
        """How far this stroke turns the hand (radians), no further than the wrist reaches."""
        a = _ct["ang"].get(self.k)
        t = math.radians(STROKE) if a is None else math.radians(_hand_turn(a))
        return max(0.0, min(t, self.s0() - self.reach()[0]))

    def prev_end(self):
        if self.prev is None:
            return self.end
        return self.prev.s0() - self.prev.turn() + _ct["yawfix"].get(self.prev.k, 0.0)

    def off(self):
        return self.s0() - self.start

    @property
    def rot(self):
        return _prev_grip(self.start + self.off() - self.frac * self.turn() + _ct["yawfix"].get(self.k, 0.0))


def _seen(env, name):
    return np.asarray(labkit._part_state(env, labkit._SPECS[name]).pos, float)


@on_step
def _ctrack(env):
    t = getattr(_demo, "_plan_clock", 0.0)
    _ct["in_stroke"] = False
    for t0, t1, kn in _ct["strokes"]:  # during a stroke: did the blade's tip drop into the slot (seen low)?
        if t0 <= t < t1:
            _ct["in_stroke"] = True
            if _seen(env, "screwdriver")[2] < PLATE_START[2] + TAB_TOP + HEAD_H + 0.0015:
                _ct["engaged"][kn] = True
    # what the cameras and the arm's joints tell about the screwdriver in the hand
    b = env.unwrapped
    pst = labkit._part_state(env, labkit._SPECS["sample_plate"])
    _ct["plate"] = (np.asarray(pst.pos, float), pst.rotation())
    sd = labkit._part_state(env, labkit._SPECS["screwdriver"])
    Rs = sd.rotation()
    _ct["b_now"] = math.atan2(Rs[1, 1], Rs[0, 1])
    for t0, t1, k, kn in _ct["aligns"]:  # blade lifted over the slot: turn the hand until blade and slot line up (seen)
        if t0 <= t < t1 and sd.pos[2] > PLATE_START[2] + TAB_TOP + HEAD_H + 0.0015:
            slot = (math.degrees(math.atan2(pst.rotation()[1, 0], pst.rotation()[0, 0])) + SLOT_DEG[CLIP_DEG[k]]
                    + math.degrees(float(pst.joints.get(f"clip{k}", 0.0)) + float(pst.joints.get(f"screw{k}", 0.0))))
            e = (slot - math.degrees(_ct["b_now"]) + 90.0) % 180.0 - 90.0
            if kn not in _ct["yawfix"]:  # first sight of this screw's slot: turn the hand straight to it (the plate
                _ct["yawfix"][kn] = math.radians(e)  # may have been dragged far by earlier strokes), then refine
            else:
                f = _ct["yawfix"][kn] + math.radians(max(-0.8, min(0.8, 0.04 * e)))
                _ct["yawfix"][kn] = max(-math.radians(90.0), min(math.radians(90.0), f))
    w = _ct["want"]
    still = w is not None and _ct.get("want_last") is not None and np.max(np.abs(w - _ct["want_last"])) < 0.0005
    _ct["want_last"] = None if w is None else np.array(w)
    tip_last, _ct["tip_last"] = _ct.get("tip_last"), np.asarray(sd.pos, float)
    settled = tip_last is not None and np.max(np.abs(np.asarray(sd.pos, float) - tip_last)) < 0.0001
    if still and settled and t - _ct.get("want_t", -1.0) < 0.1 and \
            sd.pos[2] > PLATE_START[2] + TAB_TOP + HEAD_H + 0.004:  # held still, the tip settled, clear of the screw
        # where the hand was sent (the plan's own command) against where the blade's tip is seen: how far the tip
        # sits from the hand's centre, the arm's own lag included
        om = np.asarray(w, float)[:2] - np.asarray(sd.pos, float)[:2]
        if _ct.get("b0") is None:
            _ct["b0"] = _ct["b_now"]
        a = _ct["b0"] - _ct["b_now"]
        c, s_ = math.cos(a), math.sin(a)
        om = np.array([c * om[0] - s_ * om[1], s_ * om[0] + c * om[1]])
        _ct["o_ref"] = om if _ct.get("o_ref") is None else 0.7 * _ct["o_ref"] + 0.3 * om
    if t - _ct.get("want_t", -1.0) < 0.1 and sd.pos[2] < PLATE_START[2] + TAB_TOP + HEAD_H + 0.0015:
        # The blade is down in a slot (seen low), so its tip is held there. With the wrist turned far out the arm drifts
        # off its commanded place, which leans the screwdriver (seen, the shaft's direction): move the hand back over
        # the tip until the shaft stands upright again (applied all through the screwdriver's work, so the hand never
        # jumps).
        lean = np.array([*Rs[:2, 2], 0.0])  # the shaft's axis, tip to handle, seen from above
        _ct["arm"] = np.clip(_ct["arm"] + np.clip(-0.005 * lean, -0.0002, 0.0002), -0.006, 0.006)
    if any(f0 <= t < f1 for f0, f1 in _ct["freeze"]):  # the hand is feeling for a slot: hold the trims
        return
    for t0, t1, key, fn, watch in _ct["wins"]:
        if not t0 <= t < t1:
            continue
        p = _seen(env, watch)
        last = _ct["last"].get(key)
        _ct["last"][key] = p
        if last is None or np.max(np.abs(p - last)) > 0.0005:
            continue
        if key == "sd" and p[2] < PLATE_START[2] + TAB_TOP + HEAD_H + 0.0015:
            continue  # the blade's tip has dropped into a slot (seen low): leave the hand where it is
        e = np.asarray(fn(env), float)
        e[2] = 0.0
        tr = _ct["trim"].get(key, np.zeros(3))
        g, step = (0.05, 0.0001) if key == "sd" else (0.2, 0.0003)  # the held screwdriver lags: a gentler gain
        if key == "push":  # the wafer sticks, then slides: push gently and stop within 0.3 mm, so it is not overshot
            if float(np.linalg.norm(e)) < 0.0003:
                continue
            g, step = 0.1, 0.0002
        _ct["trim"][key] = np.clip(tr + np.clip(g * e, -step, step), -0.012, 0.012)


def _screw_seen(env, P):
    """Where a screw is now (world x, y), from the plate's pose as seen: the plate turns a little on its stand under
    the screwdriver's twist, so its screws are not where they started."""
    p = labkit._part_state(env, labkit._SPECS["sample_plate"])
    return (np.asarray(p.pos, float) + p.rotation() @ np.array([P[0], P[1], 0.0]))[:2]


class _Dyn:
    """A TCP plus the live trim ``key``; during a screwdriver stroke, plus the arm's own correction (see _ctrack).
    Given a screw ``P`` (plate frame), the hand is placed over that screw as now seen, offset by where the screwdriver's
    tip sits in the hand: the shaft is not held exactly on the hand's turning axis, so as the hand turns, the hand
    moves round the tip and the tip stays over the screw."""

    def __init__(self, tcp, key, P=None):
        self.tcp, self.key, self.P = np.asarray(tcp, float), key, P

    def __array__(self, dtype=None, copy=None):
        p = self.tcp + _ct["trim"].get(self.key, np.zeros(3))
        if self.P is not None and _ct.get("o_ref") is not None and _ct.get("plate") is not None:
            pos, R = _ct["plate"]
            sxy = (pos + R @ np.array([self.P[0], self.P[1], 0.0]))[:2]
            a = _ct["b_now"] - _ct["b0"]
            c, s_ = math.cos(a), math.sin(a)
            o = np.array([c * _ct["o_ref"][0] - s_ * _ct["o_ref"][1], s_ * _ct["o_ref"][0] + c * _ct["o_ref"][1]])
            p = np.array([*(sxy + o), self.tcp[2]])
        if self.key == "sd":
            _ct["want"] = p  # the planner evaluates the current waypoint last
            _ct["want_t"] = getattr(_demo, "_plan_clock", 0.0)
            p = p + _ct["arm"]
        return np.asarray(p, dtype=dtype or float)


def _down(p, yaw):
    """TCP and rotation for straight-down fingertips ending at point p, the fingers closing along Rz(yaw) y."""
    return tuple(np.asarray(p, float) + [0.0, 0.0, TIP_ABOVE_TCP]), Orient(_prev_grip(yaw))


def _grasp(side, flip):
    a = Y if side == "right" else -Y
    c0 = -Z if flip else Z
    return np.column_stack([c0, np.cross(a, c0), a])


def _pinch(side, c, R, r_tip, out=0.0):
    """TCP and rotation for ``side``'s hand pinching an edge at its side, the grip line's centre at ``c``, the stack's
    attitude R (rolled about y); pad tips ``r_tip`` out from the centre; ``out`` backs the hand out sideways."""
    Rh = R @ _grasp(side, FLIP[side])
    a = Rh[:, 2]
    tip = np.asarray(c, float) - a * (r_tip + out)
    return tuple(tip - a * TIP_ABOVE_TCP), Orient(Rh)


def _bar(z, pitch=80.0, back=0.0):
    """TCP and rotation for the right hand round the door's handle bar at height z (above the bench), pitched down
    towards the door; ``back`` backs it off along its approach."""
    R = axis_angle_matrix(Y, math.radians(-pitch)) @ _prev_grip(math.pi / 2)
    a = R[:, 2]
    c = np.array([HANDLE_X, 0.0, TOP + z])
    return tuple(c - a * (0.007 + back)), Orient(R)


def _pw(q):
    """A plate-frame point (x, y, z above the plate's face) on the stand, in the world."""
    return np.array([PLATE_START[0] + q[0], PLATE_START[1] + q[1], PLATE_START[2] + PL_T + q[2]])


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class _Plan:
    """Two arms' waypoints, built in phases that start together; the clock is the phases' running time."""

    def __init__(self):
        self.L, self.R = [], []
        self.t = 0.0

    def hold(self, plan, dur, label="hold"):
        w = plan[-1]
        plan.append(Waypoint(dur, w.pos, w.finger, label, w.yaw))

    def phase(self, right=(), left=()):
        """Each list: (duration, pos, finger, label, yaw) tuples. The shorter one is padded with a hold."""
        tr, tl = sum(w[0] for w in right), sum(w[0] for w in left)
        dur = max(tr, tl)
        for plan, ws, tot in ((self.R, right, tr), (self.L, left, tl)):
            for w in ws:
                plan.append(Waypoint(*w))
            if dur - tot > 1e-9:
                self.hold(plan, dur - tot)
        t0 = self.t
        self.t += dur
        return t0

    def window(self, t0, t1, key, fn, watch):
        _ct["wins"].append((t0, t1, key, fn, watch))


def demo():
    _ct["trim"], _ct["wins"], _ct["looks"], _ct["freeze"] = {}, [], [], []
    _ct["vis"], _ct["strokes"], _ct["engaged"], _ct["c6"] = {}, [], {}, {}
    _ct["arm"], _ct["want"], _ct["in_stroke"] = np.zeros(3), None, False
    _ct["o_ref"], _ct["b0"], _ct["plate"], _ct["b_now"] = None, None, None, 0.0
    _ct["aligns"], _ct["yawfix"] = [], {}
    pl = _Plan()
    face = PL_T  # the plate's face, plate frame

    # -- 0. right pushes the wafer to the plate's centre by its edge; left waits --------------------------------------------
    u = PUSH_DIR
    pyaw = _wrap(math.atan2(u[1], u[0]) + math.pi / 2)  # fingers closing along the push: its 16 mm face leads
    tipz = -0.0002  # fingertips aimed 0.2 mm into the plate's face: they ride about 0.4 mm over it
    w0 = W_OFF - u * (W_R - STRIP_IN + 0.004)  # the fingertips' centre touching the wafer's edge
    w1 = -u * (W_R - STRIP_IN + 0.004)  # and where it is when the wafer is centred
    p_start = _down(_pw((*(w0 - u * 0.012), tipz)), pyaw)
    p_touch = _down(_pw((*(w0 - u * 0.001), tipz)), pyaw)
    p_near = _down(_pw((*(w1 - u * 0.006), tipz)), pyaw)  # 6 mm short: the trim pushes the rest
    push = [(2.5, tuple(np.asarray(p_start[0]) + [0, 0, 0.03]), FINGER_CLOSED, "above the plate, beside the wafer's "
             "edge", p_start[1]),
            (1.2, p_start[0], FINGER_CLOSED, "fingertips down on the plate, outside the wafer", p_start[1]),
            (1.0, p_touch[0], FINGER_CLOSED, "against the wafer's edge", p_start[1]),
            (3.0, p_near[0], FINGER_CLOSED, "push the wafer towards the plate's centre", p_start[1])]
    pl.phase(right=push, left=[(2.5, LEFT_PARK, FINGER_CLOSED, "wait, up and clear", 0.0)])
    seen_off = lambda env: (u * float(np.dot(u, (_seen(env, "sample_plate") - _seen(env, "wafer"))[:2]))).tolist() + [0]
    pl.phase(right=[(3.0, _Dyn(p_near[0], "push"), FINGER_CLOSED, "hold still, then push on until the wafer is "
                     "centred", p_start[1])])
    pl.window(pl.t - 3.0, pl.t, "push", lambda env: np.array([*seen_off(env)[:2], 0.0]), "wafer")
    back = tuple(np.asarray(p_near[0]) - [*(u * 0.010), 0] + [0, 0, 0.0])
    pl.phase(right=[(0.8, _Dyn(back, "push"), FINGER_CLOSED, "back off the wafer's edge", p_start[1]),
                    (1.0, _Dyn(tuple(np.asarray(back) + [0, 0, 0.04]), "push"), FINGER_CLOSED, "up", p_start[1]),
                    (1.5, (back[0], back[1], TOP + 0.25), FINGER_CLOSED, "up high, clear of the plate", p_start[1])])
    b2, bo = _bar(HANDLE_Z + DOOR_UP - 0.006)  # the latched door sags about 6 mm
    b3, _ = _bar(HANDLE_Z + DOOR_UP - 0.006, back=0.06)

    d2, _ = _bar(HANDLE_Z + 0.001)
    top = _bar(HANDLE_Z + DOOR_UP + 0.008)[0]  # against its top stop
    kp = np.array([DOOR_X + LOCK_DX, LOCK_Y, TOP + LOCK_H])  # the twist lock's T-bar (door shut), seen by the cameras

    def front(p, roll):
        """TCP and rotation for the right hand reaching straight forward (+x), fingertips at p, its fingers closing
        across an upright bar, turned clockwise (seen from behind) by ``roll``."""
        R = axis_angle_matrix(X, roll) @ axis_angle_matrix(Y, -math.pi / 2) @ _prev_grip(0.0)
        return tuple(np.asarray(p, float) - R[:, 2] * TIP_ABOVE_TCP), Orient(R)

    def lock_turn(r0, r1, what):
        """Right takes the twist lock's T-bar from the front and turns it from r0 to r1 (degrees)."""
        t0, o0 = front(kp, math.radians(r0))
        t1, o1 = front(kp, math.radians(r1))
        return [(2.5, tuple(np.asarray(t0) + [-0.06, 0, 0.12]), BAR_OPEN, "high in front of the door's twist lock, "
                 "hand turned", o0),  # high: the wrist turns well clear of the plate
                (1.2, tuple(np.asarray(t0) - [0.05, 0, 0]), BAR_OPEN, "in front of the twist lock", o0),
                (1.2, t0, BAR_OPEN, "fingers round its bar", o0),
                (0.5, t0, BAR_HOLD, "hold it", o0),
                (2.0, t1, BAR_HOLD, what, o1),
                (0.5, t1, BAR_OPEN, "let go", o1),
                (1.0, tuple(np.asarray(t1) - [0.05, 0, 0]), BAR_OPEN, "back off", o1)]

    def door_open(t0):
        """Right lifts the door until it latches; it starts at t0 and lifts no sooner than 1 s after the light turns
        green (the light is seen from above)."""
        wait = max(0.3, VENT_S + 1.0 - (t0 + 2.5 + 2.0 + 1.2 + 0.5))
        return [(2.5, _bar(HANDLE_Z, back=0.10)[0], BAR_OPEN, "back from the shut door's handle, hand turned", bo),
                 (2.0, _bar(HANDLE_Z, back=0.06)[0], BAR_OPEN, "in front of the handle", bo),
                 (1.2, d2, BAR_OPEN, "round the handle", bo),
                 (wait, d2, BAR_OPEN, "wait for the green light", bo),
                 (0.5, d2, BAR_HOLD, "hold it", bo),
                 (3.0, top, BAR_HOLD, "lift the door until it latches open", bo),
                 (0.6, top, BAR_HOLD, "hold it against its top stop", bo),
                 (0.5, top, BAR_OPEN, "let go: the latch holds it up", bo),
                 (1.0, _bar(HANDLE_Z + DOOR_UP, back=0.07)[0], BAR_OPEN, "back off", bo),
                 # straight back, well clear of the raised door, before the wrist turns over in free space
                 (1.0, _bar(HANDLE_Z + DOOR_UP, back=0.16)[0], BAR_OPEN, "back away from the raised door", bo)]

    door_shut = [(2.5, _bar(HANDLE_Z + DOOR_UP - 0.006, back=0.10)[0], BAR_OPEN, "back from the raised door, hand turned",
                  bo),
                 (1.5, b3, BAR_OPEN, "in front of the raised door's handle", bo),
                 (1.2, b2, BAR_OPEN, "round the handle", bo),
                 (0.5, b2, BAR_HOLD, "hold it", bo),
                 (2.5, _bar(HANDLE_Z - 0.010)[0], BAR_HOLD, "pull the door down off its latch, onto its seat", bo),
                 (0.6, _bar(HANDLE_Z - 0.006)[0], BAR_OPEN, "let go: it rests shut", bo),
                 (1.0, _bar(HANDLE_Z, back=0.07)[0], BAR_OPEN, "back off", bo),
                 (2.0, RIGHT_PARK, FINGER_CLOSED, "away", 0.0)]

    # -- 1. right turns the twist lock open, then lifts the door until it latches open; left waits ---------------------
    unlock = lock_turn(0.0, 90.0, "turn it a quarter turn clockwise: unlocked")
    do = unlock + door_open(pl.t + sum(w[0] for w in unlock))
    pl.phase(right=do, left=[(sum(w[0] for w in do), LEFT_PARK, FINGER_CLOSED, "wait", 0.0)])

    # -- 2. both take the ring holder out and set it on the plate -------------------------------------------------------
    rc = RING_START + [0, 0, (RG_T - PL_T) / 2]  # its grip line: between the rim's top and the skirt's underside
    over = np.array([*ST_XY, PLATE_START[2] + PL_T + (RG_T - PL_T) / 2])  # the same, the ring resting on the plate
    I = np.eye(3)

    def both(dur, c, finger, label, R=I, r_tip=RING_IN, out=0.0, key=None):
        rt, ro = _pinch("right", c, R, r_tip, out)
        lt, lo = _pinch("left", c, R, r_tip, out)
        if key:
            rt, lt = _Dyn(rt, key), _Dyn(lt, key)
        return [(dur, rt, finger, label, ro)], [(dur, lt, finger, label, lo)]

    def ph(*a, **k):
        r, l_ = both(*a, **k)
        return pl.phase(right=r, left=l_)

    def ph_off_door(dur, c, finger, label, r_tip=RING_IN, out=0.0):
        """Like ph, for the move just after the right hand backs away from the raised door. The right's wrist turns
        over there (a joint-space move), so it first turns over towards a point well back from the door, fingertips at
        (0.12, -0.04) and 35 cm up: its fingers then swing no nearer the door than about 12 cm (checked on the arm's
        kinematics), where turning straight to c would sweep them into it. Then it goes on to c in a straight line."""
        via = np.array([0.12, -0.04 + r_tip, TOP + 0.35])
        rv, rvo = _pinch("right", via, I, r_tip, out)
        rt, ro = _pinch("right", c, I, r_tip, out)
        lt, lo = _pinch("left", c, I, r_tip, out)
        return pl.phase(right=[(2.5, rv, finger, "back from the door, wrist turned", rvo),
                               (dur - 1.0, rt, finger, "on to " + label.split(",")[0], ro)],
                        left=[(dur + 1.5, lt, finger, label, lo)])

    ph_off_door(3.0, np.array([0.24, 0.0, TOP + 0.25]), OPEN_W, "high over the stand, wrist turned", out=0.0)
    ph(2.0, rc + [-0.225, 0, 0.02], OPEN_W, "in front of the open door", out=0.02)
    ph(2.5, rc + [-0.03, 0, 0.0], OPEN_W, "into the load lock, beside the ring holder", out=0.02)
    ph(2.0, rc, OPEN_W, "level with its rim", out=0.02)
    ph(1.5, rc, OPEN_W, "fingers over its rim and under its skirt")
    ph(0.8, rc, PINCH, "pinch it")
    ph(1.2, rc + [0, 0, 0.012], PINCH, "lift it off the holder")
    hi = PLATE_START[2] + 0.075
    ph(3.0, np.array([*ST_XY, rc[2] + 0.012]) + [0.05, 0, 0], PINCH, "out of the load lock")
    ph(2.0, np.array([*ST_XY, over[2] + 0.008]), PINCH, "over the plate")
    tw = pl.t
    ph(2.0, np.array([*ST_XY, over[2] + 0.004]), PINCH, "hold still, its skirt seen round the plate", key="ring")
    pl.window(tw, pl.t, "ring", lambda env: _seen(env, "sample_plate") - _seen(env, "ring_holder"), "ring_holder")
    ph(1.2, over + [0, 0, 0.0005], PINCH, "set it on the plate", key="ring")
    ph(0.6, over + [0, 0, 0.0005], OPEN_W, "let go", key="ring")
    ph(1.2, over, OPEN_W, "back out sideways", out=0.012, key="ring")
    ph(1.2, over + [0, 0, hi - over[2]], OPEN_W, "up", out=0.012, key="ring")
    # right shuts the door again while the plate is worked on (the load lock stays open as little as possible)
    pl.phase(right=door_shut, left=[(2.0, LEFT_PARK, FINGER_CLOSED, "up and clear", 0.0)])

    # -- 3. clips: left 0 and 135 degrees, right 270 ------------------------------------------------------------------
    knob_z = face + (TAB_TOP - PL_T) + 0.0008  # the fingertips just over the tab, low on the knob

    def knob(k, th, back=None):
        """The fingertips' point behind clip k's knob (against its swing) at swing angle th: the closed fingertips are 16
        mm wide across and 8 mm thick along their closing axis, which lies along the clip's parked direction."""
        if back is None:  # pressing: the fingertips' face 1 mm into the knob
            back = KNOB_R - 0.001 + 0.0078 * abs(math.cos(th)) + 0.004 * abs(math.sin(th))
        P, t, n = clip_frame(k)
        kp = P + KNOB_AT * clip_dir(k, th)
        d = -math.sin(th) * t + math.cos(th) * n
        return _pw((*(kp - back * d), knob_z - face))

    def swing(k, label):
        yaw = _wrap(math.radians(CLIP_DEG[k]))
        if abs(yaw) > math.pi / 2:
            yaw -= math.copysign(math.pi, yaw)
        clear = KNOB_R + 0.0015 + 0.0078
        s = [_down(knob(k, 0.0, clear) + [0, 0, 0.03], yaw), _down(knob(k, 0.0, clear), yaw)]
        s += [_down(knob(k, math.radians(a)), yaw) for a in (25.0, 50.0, 75.0)]
        P, t, n = clip_frame(k)
        k90 = P + KNOB_AT * n
        s.append(_down(_pw((*(k90 + t * (KNOB_R + 0.004 - 0.003)), knob_z - face)), yaw))
        return [(1.5, s[0][0], FINGER_CLOSED, f"above the {label} clip, outside its knob", s[0][1]),
                (1.0, s[1][0], FINGER_CLOSED, "fingertips down beside the knob", s[1][1]),
                (0.8, s[2][0], FINGER_CLOSED, "swing the clip in", s[2][1]),
                (0.8, s[3][0], FINGER_CLOSED, "swing the clip in", s[3][1]),
                (0.8, s[4][0], FINGER_CLOSED, "swing the clip in", s[4][1]),
                (1.0, s[5][0], FINGER_CLOSED, "against its stop: the tip over the wafer's edge", s[5][1]),
                (0.8, tuple(np.asarray(s[5][0]) + [0, 0, 0.03]), FINGER_CLOSED, "up", s[5][1])]

    lp = [(2.0, LEFT_PARK, FINGER_CLOSED, "up and clear", 0.0)] + swing(0, "0 degree") + swing(3, "135 degree")
    rp = [(2.0 + sum(w[0] for w in swing(0, "") + swing(3, "")), RIGHT_PARK, FINGER_CLOSED, "wait, out of the left hand's way", 0.0)]
    rp += swing(6, "270 degree")
    pl.phase(right=rp, left=lp + [(1.5, LEFT_PARK, FINGER_CLOSED, "aside, out of the right hand's way", 0.0)])

    # -- 4. right: the screwdriver, three screws ----------------------------------------------------------------------
    def sd_tcp(tip, z):
        return (tip[0], tip[1], z + SD_GRIP + TIP_ABOVE_TCP)

    z0 = SD_START[2]
    up = z0 + 0.04  # its tip clear of the socket
    yo = Orient(_prev_grip(0.0))
    pl.phase(right=[(2.5, sd_tcp(SD_XY, z0 + 0.04), SD_OPEN, "above the screwdriver's shaft", yo),
                    (1.2, sd_tcp(SD_XY, z0), SD_OPEN, "fingers round its shaft", yo),
                    (0.8, sd_tcp(SD_XY, z0), SD_HOLD, "hold it", yo),
                    (1.8, sd_tcp(SD_XY, up), SD_HOLD, "lift it out of its holder", yo),
                    (1.5, sd_tcp(SD_XY, TOP + 0.17), SD_HOLD, "up, its tip well over the plate and ring", yo)])
    head = PLATE_START[2] + TAB_TOP + HEAD_H  # a screw head's top (world z)
    FEEL_R, FEEL_N, FEEL_DT = 0.0006, 6, 0.2

    def feel(seq, xy, key, yaw, P=None):
        """Pressing lightly, keep the blade's tip (seen) centred on the screw, so that a tip resting on the slot's
        edges slides over until it drops in."""
        seq.append((FEEL_DT * (FEEL_N + 1), _Dyn(sd_tcp(xy, head - 0.0005), key, P), SD_HOLD,
                    "press, the tip centred on the screw", yaw))
    def now(seq):
        return pl.t + sum(w[0] for w in seq)

    for k in (6, 0, 3):  # 270, 0, 135 degrees
        P = clip_frame(k)[0]
        xy = _pw((*P, 0.0))[:2]
        # how far the hand can turn over this screw: from the arm's kinematics (joint6 = c - hand yaw, within +-120)
        ym = math.radians(STROKE_TOP - STROKE / 2)
        qk, _, _ = _ik.solve(sd_tcp(xy, head + 0.008), _ik.matrix_to_quat_xyzw(_prev_grip(ym)), np.array(_READY, float),
                             base_pos=_RIGHT_BASE, iters=300)
        _ct["c6"][k] = float(qk[5]) - J6_SIGN * ym
        key = "sd"  # one trim for all three screws: it follows how the screwdriver sits in the hand
        n_strokes = STROKES + 1  # one spare, should a try miss the slot; once the screw is tight, a stroke does not turn
        y0 = math.radians(STROKE_TOP)
        yaws = [Orient(_prev_grip(y0 - math.radians(STROKE) * i / 6)) for i in range(7)]
        mid = Orient(_prev_grip(math.radians(STROKE_TOP - STROKE / 2)))
        # held still here (no screw offset yet), the tip seen against where the hand was sent gives the tip's offset as
        # the arm now holds it, before the hand is placed over this screw by it
        seq = [(1.5, _Dyn(sd_tcp(xy, head + 0.008), key), SD_HOLD, f"over the {CLIP_DEG[k]} degree clip's screw", mid),
               (1.0, _Dyn(sd_tcp(xy, head + 0.008), key), SD_HOLD, "turn the blade to the slot", yaws[0])]
        tw0 = pl.t + 2.5
        # the slot's direction depends on how far the clip swung in: line the blade up with it as seen, as on later strokes
        yaws = [_SlotYaw((k, -1), i / 6, y0, y0 - math.radians(STROKE)) for i in range(7)]
        ta = now(seq)
        seq += [(2.0, _Dyn(sd_tcp(xy, head + 0.005), key, P), SD_HOLD, "hold still, the blade's tip seen over the slot",
                 yaws[0])]
        _ct["aligns"].append((ta, now(seq), k, (k, -1)))
        seq.append((1.0, _Dyn(sd_tcp(xy, head + 0.0002), key, P), SD_HOLD, "blade into the slot", yaws[0]))
        feel(seq, xy, key, yaws[0], P)
        f0 = now(seq)  # from here until the blade is lifted out, the tip is held by the screw: hold the trim
        y_end = y0 - math.radians(STROKE)
        ys = yaws
        for stroke in range(n_strokes):
            ts = now(seq) - 1.4  # from the press before the stroke
            seq += [(0.6, _Dyn(sd_tcp(xy, head - 0.0010), key, P), SD_HOLD, "turn it clockwise", ys[i])
                    for i in range(1, 7)]
            _ct["strokes"].append((ts, now(seq), (k, stroke)))
            seq.append((0.6, _Dyn(sd_tcp(xy, head + 0.008), key, P), SD_HOLD, "lift the blade out", ys[6]))
            _ct["freeze"].append((f0, now(seq)))
            if stroke < n_strokes - 1:
                t_look = pl.t + sum(w[0] for w in seq)
                _ct["looks"].append((t_look, t_look + 0.6, k, stroke))
                seq.append((0.6, _Dyn(sd_tcp(xy, head + 0.008), key, P), SD_HOLD, "look at the slot", ys[6]))
                prev = ys[0] if isinstance(ys[0], _SlotYaw) else None
                ys = [_SlotYaw((k, stroke), i / 6, y0, y_end, prev) for i in range(7)]
                seq += [(0.8, _Dyn(sd_tcp(xy, head + 0.008), key, P), SD_HOLD, "turn back", ys[i]) for i in (4, 2, 0)]
                ta = now(seq)
                seq.append((1.2, _Dyn(sd_tcp(xy, head + 0.008), key, P), SD_HOLD, "hold still over the slot, blade "
                            "turned to it", ys[0]))
                seq.append((0.8, _Dyn(sd_tcp(xy, head + 0.0002), key, P), SD_HOLD, "blade back in the slot", ys[0]))
                _ct["aligns"].append((ta, now(seq), k, (k, stroke)))
                feel(seq, xy, key, ys[0], P)
                f0 = now(seq)
        seq.append((1.2, _Dyn(sd_tcp(xy, head + 0.008), key, P), SD_HOLD, "up", ys[6]))
        pl.phase(right=seq)
        pl.window(tw0 + 0.6, pl.t - 1.2, key, lambda env, P=P: np.array([*(_screw_seen(env, P)
                                                                            - _seen(env, "screwdriver")[:2]), 0.0]),
                  "screwdriver")
    pl.phase(right=[(1.5, sd_tcp((0.17, -0.10), head + 0.03), SD_HOLD, "up, off the plate", yo),
                    (2.0, sd_tcp(SD_XY, TOP + 0.17), SD_HOLD, "high over its holder", yo),
                    (1.5, sd_tcp(SD_XY, up), SD_HOLD, "down over its holder", yo)])
    tw = pl.t
    near = z0 + SOCK_D + 0.003  # its tip 3 mm over the snug socket
    pl.phase(right=[(2.0, _Dyn(sd_tcp(SD_XY, up), "sdh"), SD_HOLD, "hold still, its tip seen over the socket", yo),
                    (1.2, _Dyn(sd_tcp(SD_XY, near), "sdh"), SD_HOLD, "down, its tip just over the socket", yo),
                    (1.2, _Dyn(sd_tcp(SD_XY, near), "sdh"), SD_HOLD, "hold still, lined up with the socket (seen)",
                     yo),
                    # slowly: a quick move down drags the tip a millimetre or more sideways, onto the socket's rim
                    (3.0, _Dyn(sd_tcp(SD_XY, z0 + 0.001), "sdh"), SD_HOLD, "slowly into its holder", yo),
                    (0.6, _Dyn(sd_tcp(SD_XY, z0 + 0.001), "sdh"), SD_OPEN, "let go", yo),
                    (1.2, _Dyn(sd_tcp(SD_XY, z0 + 0.06), "sdh"), SD_OPEN, "up", yo),
                    (1.5, (0.20, -0.20, TOP + 0.23), FINGER_CLOSED, "high, beside the stand", yo)],
             left=[(9.2, LEFT_PARK, FINGER_CLOSED, "wait", 0.0),
                   (1.5, (0.20, 0.20, TOP + 0.23), FINGER_CLOSED, "high, beside the stand", 0.0)])
    seen_sock = lambda env: np.array([*(np.asarray(SD_XY) - _seen(env, "screwdriver")[:2]), 0.0])
    pl.window(tw + 0.8, tw + 2.0, "sdh", seen_sock, "screwdriver")
    pl.window(tw + 3.4, tw + 4.4, "sdh", seen_sock, "screwdriver")  # the arm drifts on the way down: trim again

    # -- 5. right opens the door again; both roll the plate and ring over together and set them on the load lock's holder
    do = door_open(pl.t)
    pl.phase(right=do, left=[(sum(w[0] for w in do), (0.20, 0.20, TOP + 0.23), FINGER_CLOSED, "wait", 0.0)])
    c0 = np.array([*ST_XY, PLATE_START[2] + (PL_T + RG_T) / 2])
    lift = np.array([*ST_XY, TOP + LIFT_Z])
    ph_off_door(3.0, c0 + [0, 0, 0.12], OPEN_W, "high beside the plate and ring, wrist turned", r_tip=STACK_IN,
                out=0.03)
    ph(2.0, c0 + [0, 0, 0.03], OPEN_W, "beside the plate's and ring's side edges, above", r_tip=STACK_IN, out=0.03)
    ph(1.5, c0, OPEN_W, "level with them", r_tip=STACK_IN, out=0.03)
    ph(1.5, c0, OPEN_W, "fingers over the ring and under the plate", r_tip=STACK_IN)
    ph(0.8, c0, PINCH, "pinch them together", r_tip=STACK_IN)
    ph(1.5, c0 + [0, 0, 0.03], PINCH, "lift them off the stand", r_tip=STACK_IN)
    ph(2.5, lift, PINCH, "up, clear of everything", r_tip=STACK_IN)
    for k in range(1, 9):
        ph(0.8, lift, PINCH, "roll them over together: samples face down", axis_angle_matrix(Y, ROLL_SIGN * math.pi * k / 8),
           r_tip=STACK_IN)
    Rf = axis_angle_matrix(Y, ROLL_SIGN * math.pi)
    seat = np.array([*HOLD_XY, TOP + HOLD_Z + (PL_T + RG_T) / 2])
    above = seat + [0, 0, 0.026]  # the clips' knobs and the long screw hang below the ring: clear of its fences
    ph(2.0, np.array([*ST_XY, above[2]]) + [0.06, 0, 0], PINCH, "down to the door's opening", Rf, r_tip=STACK_IN)
    ph(3.0, above, PINCH, "into the load lock, over its holder", Rf, r_tip=STACK_IN)
    tw = pl.t
    ph(2.0, above, PINCH, "hold still, seen over the holder", Rf, r_tip=STACK_IN, key="stack")
    pl.window(tw, pl.t, "stack", lambda env: np.array([*HOLD_XY, 0.0]) - _seen(env, "ring_holder"), "ring_holder")
    ph(1.5, seat + [0, 0, 0.0015], PINCH, "set the ring holder on the holder", Rf, r_tip=STACK_IN, key="stack")
    ph(0.6, seat + [0, 0, 0.0005], OPEN_W, "let go", Rf, r_tip=STACK_IN, key="stack")
    ph(1.2, seat, OPEN_W, "back out sideways", Rf, r_tip=STACK_IN, out=0.03, key="stack")
    ph(2.5, seat + [-0.17, 0, 0.0], OPEN_W, "out of the load lock", Rf, r_tip=STACK_IN, out=0.03)

    # -- 6. right pulls the door down off its latch onto its seat; both away ---------------------------------------------------------
    pl.phase(right=[(1.5, _pinch("right", seat + [-0.245, 0.0, 0.08], Rf, STACK_IN, 0.0)[0], OPEN_W, "up and out",
                     _pinch("right", seat, Rf, STACK_IN)[1]),
                    *door_shut[:-1],
                    *lock_turn(90.0, 0.0, "turn it back a quarter turn: locked"),
                    door_shut[-1]],
             left=[(1.5, _pinch("left", seat + [-0.245, 0.0, 0.08], Rf, STACK_IN, 0.0)[0], OPEN_W, "up and out",
                    _pinch("left", seat, Rf, STACK_IN)[1]),
                   (3.0, LEFT_PARK, FINGER_CLOSED, "up and clear", 0.0)])
    pl.phase(right=[(2.5, RIGHT_PARK, FINGER_CLOSED, "rest", 0.0)], left=[(2.5, LEFT_PARK, FINGER_CLOSED, "rest", 0.0)])
    return {"left": pl.L, "right": pl.R}


@on_step
def _debug(env):
    """EBEAM_DEBUG=1: the scorer's state every 5 s (stderr)."""
    if not os.environ.get("EBEAM_DEBUG"):
        return
    st = _state
    p = labkit._part_state(env, labkit._SPECS["sample_plate"])
    if st.get("step", 0) % 20 == 0 and 40 <= st["step"] * DT <= 110:  # the wafer's offset on the plate, each second
        q = in_plate(p, labkit._part_state(env, labkit._SPECS["wafer"]).pos)
        print(f"[ebeam-w] t={st['step'] * DT:.0f}s wafer offset ({q[0] * 1000:.2f}, {q[1] * 1000:.2f}) mm",
              file=sys.stderr, flush=True)
    if st.get("step", 0) % 100:
        return
    sc = " ".join(f"{CLIP_DEG[k]}:{math.degrees(-float(p.joints.get(f'screw{k}', 0.0))):.0f}deg/"
                  f"{press(p, k, labkit._part_state(env, labkit._SPECS['wafer'])) * 1000:.3f}mm/z{float(p.joints.get(f'clipz{k}', 0.0)) * 1000:.3f}" for k in (6, 0, 3))
    wq = in_plate(p, labkit._part_state(env, labkit._SPECS["wafer"]).pos)
    print(f"[ebeam] t={st['step'] * DT:.0f}s wafer_z={(wq[2] - PL_T) * 1000:.3f}mm viol={sorted(st['viol'])} settled={st.get('settled')} "
          f"centred={st.get('centred')} gap={st.get('gap', 0):.0f} stack={st.get('stack')} door={st.get('door', 0):.3f} "
          f"lock={st.get('lock', 0):.2f} far={st.get('far')} sd_in={st.get('sd_in')} tight={sorted(st['tight'])} "
          f"screws {sc}", file=sys.stderr, flush=True)
