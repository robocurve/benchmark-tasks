"""On the station table: the custom fabrication cassette, with its front panel and handle toward the robot's right.
It holds 100 sheets of A4 copy paper, dropped in
loosely: fanned along their length, a little skewed, and short of the feed edge. The two width guides (linked, so
they move together) are still set for Letter, and the rear length guide for Legal. The near width and rear guides
have blue squeeze-to-release latches; the far width guide follows the near one.
"""

from __future__ import annotations

import math
import os
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import parts  # noqa: E402  (before lots.py, so its reset hooks run first)
import lots  # noqa: E402
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

TRAY_COL = (0.20, 0.21, 0.23)  # dark grey cassette body
GUIDE_COL = (0.62, 0.64, 0.66)  # light grey guides
LATCH_COL = (0.10, 0.42, 0.85)  # blue squeeze latches
PAPER = ((0.97, 0.97, 0.95), (0.90, 0.91, 0.89), (0.97, 0.97, 0.95), (0.90, 0.91, 0.89))

# -- the paper ---------------------------------------------------------------------------------------------------------
A4 = (0.210, 0.297)  # m: x (across the tray) by y (along it)
N_BUNDLES = 4
SHEETS = 25  # per bundle
T = 0.0025  # 25 sheets of 80 g/m2 copy paper, about 0.1 mm each
BUNDLE_MASS = 0.125  # 25 x 5.0 g (A4 at 80 g/m2)

# -- the cassette (fixed on the table) -------------------------------------------------------------------------------
XC = 0.27  # centre line of the paper path (x)
FLOOR_T = 0.003
PLINTH_H = 0.035  # underfloor shafts, opposed racks, pinion and pawls in fabrication revision D0
DECK = TOP + PLINTH_H
FLOOR = DECK + FLOOR_T  # where the paper rests
YF = -0.160  # the feed edge: inside face of the front panel; the stack's leading edge must sit against it
WIDE = 0.218  # the width guides' widest setting (Letter/Legal 215.9 mm plus 1 mm a side)
LEGAL = 0.356  # the length guide starts set for Legal (355.6 mm)
GUIDE_H = 0.036  # guide faces end 36 mm above deck (33 mm above the paper floor)
PAD_T, PLATE_T = 0.003, 0.004
PAD_TRAVEL = 0.005
PAD_K = 1500.0  # N/m: each guide face is a sprung pad
LEVER_UP = 0.17  # rad: a latch lever squeezed shut
UNLOCK = 0.10  # rad: the latch lets the guide slide once its lever is squeezed this far
LEVER_K = 0.4  # N m/rad design rate; actual squeeze depends on contact radius and linkage friction
LOCK_K, LOCK_C = 5.0e4, 60.0  # a latched guide's hold on its rack (N/m, N s/m)
FREE_C = 30.0  # N s/m numerical viscous proxy; hardware split bush is dry friction, not a measured match
LOCK_PITCH = 0.0005  # physical pawl rack pitch; capture to the nearest phased tooth
LOCK_WINDOW = 0.00005  # half the 0.10 mm available tooth-clearance window; no automatic crest centring
W_SLIDE = (-0.070, 0.030)  # y span of the width guides
WN0 = XC - WIDE / 2  # near width guide's pad face (x) at the start
WF0 = XC + WIDE / 2
LF0 = YF + LEGAL  # length guide's pad face (y) at the start
L_SPAN = (XC - 0.070, XC + 0.070)
TAB = 0.030  # latch width
TAB_YC = -0.020  # near width guide's latch centre (y)


def _b(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(tuple(hi - lo), tuple((lo + hi) / 2), color)


Z_PAD = (FLOOR + 0.0005, DECK + GUIDE_H)
Z_POST = (DECK + GUIDE_H, DECK + 0.062)
Z_FOOT = (DECK + 0.028, DECK + 0.032)
Z_LEVER = (DECK + GUIDE_H + 0.0005, DECK + 0.062)


def _cartridge(face, normal_axis, toward_paper, span):
    """FC1 carrier: 5 mm stops, pin bores, relieved latch seat and a direct scale.

    N points toward paper; S runs along the pad. Internal springs/bush drag and
    pawl tooth forces remain declared joint-law approximations, not hidden solid overlap.
    """
    along = 1 - normal_axis
    lo_s, hi_s = span
    pins = (lo_s + 0.014, hi_s - 0.014)
    pin_z = DECK + 0.018
    def block(n0, n1, s0, s1, z0, z1, color):
        lo, hi = np.zeros(3), np.zeros(3)
        lo[normal_axis], hi[normal_axis] = sorted((face + toward_paper * n0, face + toward_paper * n1))
        lo[along], hi[along] = s0, s1
        lo[2], hi[2] = z0, z1
        return _b(lo, hi, color)
    fixed = []
    # Backplate N=-22..-18 with two square collision-clearance bores around
    # the drawing's round Ø3.10 mm bores. Solid stock never overlaps pad/pins.
    latched = normal_axis == 1 or toward_paper == 1
    centre = (lo_s + hi_s) / 2
    cuts_s = [lo_s, pins[0]-.00155, pins[0]+.00155, pins[1]-.00155, pins[1]+.00155, hi_s]
    if latched:
        cuts_s += [centre-.031, centre+.031]
    cuts_s = sorted(cuts_s)
    cuts_z = [Z_PAD[0], pin_z-.00155, pin_z+.00155, DECK+.0275, Z_PAD[1]]
    for s0, s1 in zip(cuts_s, cuts_s[1:]):
        for z0, z1 in zip(cuts_z, cuts_z[1:]):
            if (any(abs((s0+s1)/2-p) < .00155 for p in pins)
                    and abs((z0+z1)/2-pin_z) < .00155):
                continue
            if latched and s0 >= centre-.031-EPS and s1 <= centre+.031+EPS and z0 >= DECK+.0275-EPS:
                continue
            fixed.append(block(-0.022, -0.018, s0, s1, z0, z1, GUIDE_COL))
    for s0, s1 in ((lo_s, lo_s + 0.008), (hi_s - 0.008, hi_s)):
        fixed.append(block(-0.030, -0.0085, s0, s1, Z_PAD[0], Z_PAD[1], GUIDE_COL))
    for s in (lo_s+.004, hi_s-.004):
        fixed.append(block(-.033,-.008,s-.0015,s+.0015,DECK+.0165,DECK+.0195,GUIDE_COL))
    moving = []
    for s in pins:
        at = np.zeros(3)
        at[normal_axis] = face - toward_paper * 0.013
        at[along], at[2] = s, pin_z
        moving.append(parts.cylinder(0.0015, 0.020, at=tuple(at), axis="xy"[normal_axis],
                                     color=(0.68, 0.69, 0.71)))
    # Rear-stop pointer travels with a pin beside (not through) a supported
    # fixed scale. Zero N=-24.5 mm; full compression N=-29.5 mm.
    s = pins[0]
    fixed.append(block(-0.030, -0.024, lo_s, s - 0.0045,
                       Z_PAD[1], Z_PAD[1] + 0.0005, GUIDE_COL))
    fixed.append(block(-0.030, -0.024, s - 0.0095, s - 0.0045,
                       Z_PAD[1] + 0.0005, Z_PAD[1] + 0.0006, (0.96, 0.96, 0.94)))
    for i in range(11):
        n = -0.0245 - i * 0.0005
        color = (0.85, 0.12, 0.10) if i == 4 else (0.12, 0.12, 0.12)
        fixed.append(block(n - 0.00006, n + 0.00006, s - 0.009, s - 0.0045,
                           Z_PAD[1] + 0.0006, Z_PAD[1] + 0.0007, color))
    moving.append(block(-0.0247, -0.0243, s - 0.004, s + 0.004,
                        DECK+.015, Z_PAD[1] + 0.0008, (0.88, 0.12, 0.10)))
    moving.append(block(-.0263,-.0247,s-.0019,s+.0019,DECK+.0161,DECK+.0199,GUIDE_COL))
    for n in (-.029,-.025):
        at = np.zeros(3)
        at[normal_axis], at[along], at[2] = face+toward_paper*n, lo_s+.002, DECK+.0373
        fixed.append(parts.cylinder(.0019,.0016,at=tuple(at),axis="z",color=GUIDE_COL))
    return tuple(fixed), tuple(moving)


def _bridge(face, normal_axis, toward_paper, centre):
    """D06 L bridge, including clearance for the vertical latch tie rod."""
    along = 1 - normal_axis
    def block(n0, n1, s0, s1, z0, z1):
        lo, hi = np.zeros(3), np.zeros(3)
        lo[normal_axis], hi[normal_axis] = sorted((face+toward_paper*n0, face+toward_paper*n1))
        lo[along], hi[along] = centre+s0, centre+s1
        lo[2], hi[2] = DECK+z0, DECK+z1
        return _b(lo, hi, GUIDE_COL)
    # Lowered foot clears the crank root. The 6 x 4 mm pocket clears the
    # actual rod eye, rather than only its 2 mm threaded stem.
    foot = []
    for z0,z1,smax in ((.028,.030,.0255),(.030,.032,.0195)):
        foot += [block(a,b,-.0195,smax,z0,z1) for a,b in ((-.030,-.028),(-.022,-.0085))]
        foot += [block(-.028,-.022,a,b,z0,z1) for a,b in ((-.0195,-.002),(.002,smax))]
    foot += [block(-.022,-.018,s-.002,s+.002,.0275,.028) for s in (-.010,.010)]
    return (*foot, block(-.015,-.011,-.015,.015,.032,.044))


def _latch_hardware(face, normal_axis, toward_paper, centre):
    """FC1 exposed stock; small fasteners/coil use conservative solid envelopes.

    Bearing bores and stop slots are not contact-solved. The guide law and hinge
    limits represent their function; all gripper-accessible surfaces remain covered.
    """
    along = 1-normal_axis
    def xyz(n,s,z):
        out = np.zeros(3)
        out[normal_axis],out[along],out[2] = face+toward_paper*n/1000,centre+s/1000,DECK+z/1000
        return out
    def block(n0,n1,s0,s1,z0,z1,color=GUIDE_COL):
        a,b = xyz(n0,s0,z0),xyz(n1,s1,z1)
        return _b(np.minimum(a,b),np.maximum(a,b),color)
    def cyl(n,s,z,r,length,axis):
        return parts.cylinder(r/1000,length/1000,at=tuple(xyz(n,s,z)),axis=axis,color=GUIDE_COL)
    sa = "xy"[along]
    fixed = [block(-23,-15,16,19.5,32,40),block(-23,-15,-19.5,-16,32,44),
             block(-13,-9,19.5,25.5,30,44),cyl(-19,3.5,36,1.5,53,sa),
             cyl(-19,-21.75,36,3.5,2.5,sa),cyl(-19,27.25,36,3.5,2.5,sa),
             cyl(-19,22.5,36,4.6,4,sa),cyl(-19,22.5,36,3,5,sa),
             cyl(-15,22.5,40,.6,8,"xy"[normal_axis]),
             block(-27.1,-22.9,-1.2,1.2,30,39.1),block(-26.1,-23.9,-1.1,1.1,0,31.1)]
    moving = [block(-21,-17,-15,15,38,62,LATCH_COL),
              block(-27,-17,-15,-2,34,38,LATCH_COL),block(-27,-17,2,15,34,38,LATCH_COL),
              block(-22,-17,-2,2,34,38,LATCH_COL),block(-24.6,-17,14.5,24.5,56,62),
              cyl(-23,22.5,49,.6,26,"z"),cyl(-19,-16,42,1,8,sa),cyl(-25,0,36,1,9,sa)]
    moving += [cyl(-19,25.3,z,1.9,1.6,sa) for z in (57.5,60.5)]
    return tuple(fixed),tuple(moving)

# near width guide (moves +x), its latch on the outside
_NEAR_PAD = _b((WN0 - PAD_T, W_SLIDE[0], Z_PAD[0]), (WN0, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_NEAR_FRAME, _NEAR_PINS = _cartridge(WN0, 0, +1, W_SLIDE)
_NEAR_POST = (
    _b((WN0-.011,TAB_YC-TAB/2,Z_POST[0]),(WN0-.0085,TAB_YC+TAB/2,Z_POST[1]),LATCH_COL),
    _b((WN0-.0085,TAB_YC-TAB/2,Z_POST[0]+.0005),(WN0-.003,TAB_YC+TAB/2,Z_POST[1]),LATCH_COL),
)
_NEAR_BRIDGE = _bridge(WN0, 0, +1, TAB_YC)
_NEAR_HARDWARE, _NEAR_LEVER = _latch_hardware(WN0,0,+1,TAB_YC)
# far width guide (moves -x), active tracking proxy for the physical opposed rack
_FAR_PAD = _b((WF0, W_SLIDE[0], Z_PAD[0]), (WF0 + PAD_T, W_SLIDE[1], Z_PAD[1]), GUIDE_COL)
_FAR_FRAME, _FAR_PINS = _cartridge(WF0, 0, -1, W_SLIDE)
# length guide (moves -y), its latch on the back
_LEN_PAD = _b((L_SPAN[0], LF0, Z_PAD[0]), (L_SPAN[1], LF0 + PAD_T, Z_PAD[1]), GUIDE_COL)
_LEN_FRAME, _LEN_PINS = _cartridge(LF0, 1, -1, L_SPAN)
_LEN_POST = (
    _b((XC-TAB/2,LF0+.0085,Z_POST[0]),(XC+TAB/2,LF0+.011,Z_POST[1]),LATCH_COL),
    _b((XC-TAB/2,LF0+.003,Z_POST[0]+.0005),(XC+TAB/2,LF0+.0085,Z_POST[1]),LATCH_COL),
)
_LEN_BRIDGE = _bridge(LF0, 1, -1, XC)
_LEN_HARDWARE, _LEN_LEVER = _latch_hardware(LF0,1,-1,XC)

# cassette body: 304 x 417 floor, 292 x 396 mm inside the walls.
X0, X1 = XC - 0.152, XC + 0.152
Y0, Y1 = YF - 0.012, LF0 + 0.049


def _slotted_floor():
    # FC1 cutouts in fabrication U,V millimetres. Service holes are plugged
    # flush during trials; shaft/rod slots remain open as on the physical floor.
    holes = [(-136,-62,138,142),(-2,2,229,383)]
    holes += [(a,b,v,v+5) for a,b in ((-140,-58),(58,140)) for v in (97.5,177.5)]
    holes += [(u,u+5,201,390) for u in (-62.5,57.5)]
    rects = [(X0,X1,Y0,Y1)]
    for a,b,c,d in holes:
        a,b,c,d = XC+a/1000,XC+b/1000,YF+c/1000,YF+d/1000
        out = []
        for x0,x1,y0,y1 in rects:
            i0,i1,j0,j1 = max(x0,a),min(x1,b),max(y0,c),min(y1,d)
            if i0>=i1 or j0>=j1:
                out.append((x0,x1,y0,y1))
                continue
            for r in ((x0,i0,y0,y1),(i1,x1,y0,y1),(i0,i1,y0,j0),(i0,i1,j1,y1)):
                if r[1]-r[0]>EPS and r[3]-r[2]>EPS:
                    out.append(r)
        rects = out
    return tuple(_b((a,c,DECK),(b,d,FLOOR),TRAY_COL) for a,b,c,d in rects)


BODY = (
    *_slotted_floor(),
    _b((X0, Y0, DECK), (X1, YF, DECK + 0.065), TRAY_COL),  # front panel, 65 mm above deck
    _b((XC - 0.050, Y0 - 0.010, DECK + 0.040), (XC + 0.050, Y0, DECK + 0.050), (0.12, 0.12, 0.13)),
    _b((X0, YF, DECK), (X0 + 0.006, Y1, DECK + 0.030), TRAY_COL),
    _b((X1 - 0.006, YF, DECK), (X1, Y1, DECK + 0.030), TRAY_COL),
    _b((X0, Y1 - 0.009, DECK), (X1, Y1, DECK + 0.030), TRAY_COL),
    # Enclosed 35 mm plinth: actual underfloor space, not a solid 38 mm slab.
    _b((X0, Y0, TOP), (X0 + 0.006, Y1, DECK), TRAY_COL),
    _b((X1 - 0.006, Y0, TOP), (X1, Y1, DECK), TRAY_COL),
    _b((X0, Y0, TOP), (X1, Y0 + 0.006, DECK), TRAY_COL),
    _b((X0, Y1 - 0.006, TOP), (X1, Y1, DECK), TRAY_COL),
)


def _origin(*shapes):
    return np.mean([s.at for s in shapes], axis=0)


# Moving masses are design estimates from the cut stock and hardware, not weighings.
WN = parts.Slider("width_near", shapes=(*_NEAR_FRAME, *_NEAR_POST, *_NEAR_BRIDGE, *_NEAR_HARDWARE), axis=(1.0, 0.0, 0.0), travel=0.070,
                  mass=0.110, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
WF = parts.Slider("width_far", shapes=_FAR_FRAME, axis=(-1.0, 0.0, 0.0), travel=0.070,
                  mass=0.070, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
LG = parts.Slider("length", shapes=(*_LEN_FRAME, *_LEN_POST, *_LEN_BRIDGE, *_LEN_HARDWARE), axis=(0.0, -1.0, 0.0), travel=0.150,
                  mass=0.120, spring=LOCK_K, damping=LOCK_C, max_effort=400.0)
WN_PAD = parts.Slider("pad_near", shapes=(_NEAR_PAD, *_NEAR_PINS), axis=(-1.0, 0.0, 0.0), travel=PAD_TRAVEL, mass=0.013,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="width_near")
WF_PAD = parts.Slider("pad_far", shapes=(_FAR_PAD, *_FAR_PINS), axis=(1.0, 0.0, 0.0), travel=PAD_TRAVEL, mass=0.013,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="width_far")
LG_PAD = parts.Slider("pad_length", shapes=(_LEN_PAD, *_LEN_PINS), axis=(0.0, 1.0, 0.0), travel=PAD_TRAVEL, mass=0.016,
                      spring=PAD_K, damping=1.0, max_effort=50.0, parent="length")
WN_LEVER = parts.Hinge("latch_width", shapes=_NEAR_LEVER, pivot=(WN0 - 0.019, TAB_YC, DECK + GUIDE_H),
                       axis=(0.0, 1.0, 0.0), lower=0.0, upper=LEVER_UP, mass=0.007, spring=LEVER_K, damping=0.003,
                       max_effort=5.0, parent="width_near")
LG_LEVER = parts.Hinge("latch_length", shapes=_LEN_LEVER, pivot=(XC, LF0 + 0.019, DECK + GUIDE_H),
                       axis=(1.0, 0.0, 0.0), lower=0.0, upper=LEVER_UP, mass=0.007, spring=LEVER_K, damping=0.003,
                       max_effort=5.0, parent="length")
CASSETTE = parts.Part("cassette", pos=(0.0, 0.0, 0.0), shapes=BODY,  # shapes given in world coordinates
                     
                      joints=(WN, WF, LG, WN_PAD, WF_PAD, LG_PAD, WN_LEVER, LG_LEVER), friction=(0.35, 0.30))


def _bundle(i):
    return parts.Part(f"paper_{i + 1}", pos=(XC, YF + A4[1] / 2, FLOOR + T / 2 + i * (T + 0.0002)), mass=BUNDLE_MASS,
                      shapes=(parts.box((A4[0], A4[1], T), (0.0, 0.0, 0.0), PAPER[i]),), friction=(0.45, 0.35))


BUNDLES = [_bundle(i) for i in range(N_BUNDLES)]
NAMES = [b.name for b in BUNDLES]
OBJECTS = [CASSETTE, *BUNDLES]


def _lot_place(d, dx):
    return {b.name: (XC + dx[i], YF + d[i] + A4[1] / 2, b.pos[2]) for i, b in enumerate(BUNDLES)}


# Lot A (the screenshot): bottom sheets nearest the feed edge, each bundle 17 mm further back toward the top (3 to
# 54 mm short of the edge). Lot B: staggered the other way (top sheets nearest, bottom 54 mm short), skewed the other way.
FAN_A, FAN_B = (0.003, 0.020, 0.037, 0.054), (0.054, 0.037, 0.020, 0.003)
DX_A = (0.0008, -0.0006, 0.0005, -0.0008)
YAW_A = tuple(math.radians(a) for a in (0.4, -0.5, 0.7, -0.3))
LOTS = lots.declare(
    OBJECTS,
    lots.Lot("A", place=_lot_place(FAN_A, DX_A), yaw={n: a for n, a in zip(NAMES, YAW_A)}),
    lots.Lot("B", place=_lot_place(FAN_B, tuple(-v for v in DX_A)), yaw={n: -a for n, a in zip(NAMES, YAW_A)}),
)
# the declared poses are lot A's, so check.py and the screenshot agree
for _b_ in BUNDLES:
    object.__setattr__(_b_, "pos", LOTS[0].place[_b_.name])
    object.__setattr__(_b_, "yaw", LOTS[0].yaw[_b_.name])

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
CLEAR = 0.005  # let go: every gripper face more than 5 mm from the paper, the guides and their latches
HOLD_S = 1.0
SNUG = 0.001  # a guide counts as against the stack within 1 mm
SQUEEZE = 0.0020  # >2 mm (nominal 3 N) marks excessive squeeze; not a measured paper-bowing law
SQUARE_DEG = 1.0
FLAT_DEG = 2.0
FACE_STEP = 0.003
CONTACT_TOL = 0.00025  # numerical surface-contact allowance, not permission for gross interpenetration
SUPPORT_TOL = 0.001  # every rigid 25-sheet layer must be supported within 1 mm
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
LOCKABLE = (("width_near", "latch_width"), ("length", "latch_length"))


# -- geometry ----------------------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _finite_vector(value, size):
    v = np.asarray(value, float)
    return v.shape == (size,) and bool(np.isfinite(v).all())


def _valid_pose(pos, quat):
    return (_finite_vector(pos, 3) and _finite_vector(quat, 4)
            and abs(float(np.linalg.norm(quat)) - 1.0) <= 0.001)


def _states_valid(objects):
    """Unknown state cannot certify any tier, even when cached link poses look finite."""
    try:
        for name in ("cassette", *NAMES):
            obj = objects[name]
            if not (_valid_pose(obj.pos, obj.quat_xyzw) and _finite_vector(obj.lin_vel, 3)
                    and _finite_vector(obj.ang_vel, 3)):
                return False
        p = objects["cassette"]
        for joint in CASSETTE.joints:
            if not np.isfinite(float(p.joints[joint.name])):
                return False
            pos, quat = p.links[joint.name]
            if not _valid_pose(pos, quat):
                return False
    except (KeyError, AttributeError, ValueError, TypeError, OverflowError):
        return False
    return True


def _link_poses(p) -> dict:
    out = {"base": (np.asarray(p.pos, float), _rot(p.quat_xyzw))}
    for name, (pos, qx) in getattr(p, "links", {}).items():
        out[name] = (np.asarray(pos, float), _rot(qx))
    return out


def _bundle_box(s):
    return np.asarray(s.pos, float), _rot(s.quat_xyzw), np.array((A4[0], A4[1], T))


def _corners(s):
    c, r, size = _bundle_box(s)
    k = np.array([[a, b, z] for a in (-0.5, 0.5) for b in (-0.5, 0.5) for z in (-0.5, 0.5)])
    return c + (k * size) @ r.T


def bundle_yaw(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.atan2(r[1, 0], r[0, 0])


def bundle_tilt(s) -> float:
    r = _rot(s.quat_xyzw)
    return math.acos(max(-1.0, min(1.0, r[2, 2])))


def pad_faces(p) -> dict:
    """The stack-side face of each guide's pad, world: near x, far x, length y."""
    poses = _link_poses(p)
    out = {}
    for link, shape, axis, sign in (("pad_near", _NEAR_PAD, 0, +1), ("pad_far", _FAR_PAD, 0, -1),
                                    ("pad_length", _LEN_PAD, 1, -1)):
        pos, r = poses[link]
        j = next(j for j in CASSETTE.joints if j.name == link)
        centre = pos + r @ (np.asarray(shape.at) - j.origin)
        out[link] = float(centre[axis] + sign * shape.half[axis])
    return out


def _clip_polygon(poly, axis, bound, less):
    """Clip a convex paper footprint to the actual finite guide-contact span."""
    out = []
    for a, b in zip(poly, poly[1:] + poly[:1]):
        ia = a[axis] <= bound if less else a[axis] >= bound
        ib = b[axis] <= bound if less else b[axis] >= bound
        if ia:
            out.append(a)
        if ia != ib:
            out.append(a + (b - a) * ((bound - a[axis]) / (b[axis] - a[axis])))
    return out


def _contact_gaps(objects, faces):
    out = {"near": [], "far": [], "rear": []}
    corners = np.array([[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]]) * A4
    for name in NAMES:
        s = objects[name]
        poly = list(np.asarray(s.pos)[:2] + corners @ _rot(s.quat_xyzw)[:2, :2].T)
        width = _clip_polygon(_clip_polygon(poly, 1, W_SLIDE[0], False), 1, W_SLIDE[1], True)
        length = _clip_polygon(_clip_polygon(poly, 0, L_SPAN[0], False), 0, L_SPAN[1], True)
        if width:
            out["near"].append(min(v[0] for v in width) - faces["pad_near"])
            out["far"].append(faces["pad_far"] - max(v[0] for v in width))
        if length:
            out["rear"].append(faces["pad_length"] - max(v[1] for v in length))
    return out


def _supported_stack(objects):
    layers = sorted((objects[n] for n in NAMES), key=lambda s: float(s.pos[2]))
    bottom_face = np.array([[x, y, -T / 2] for x, y in
                            ((-A4[0] / 2, -A4[1] / 2), (A4[0] / 2, -A4[1] / 2),
                             (A4[0] / 2, A4[1] / 2), (-A4[0] / 2, A4[1] / 2))])
    for i, s in enumerate(layers):
        bottom = np.asarray(s.pos) + bottom_face @ _rot(s.quat_xyzw).T
        if i:
            below = layers[i - 1]
            rb = _rot(below.quat_xyzw)
            at = np.array([s.pos[0], s.pos[1], below.pos[2]])
            local = rb.T @ (at - below.pos)
            if np.any(np.abs(local[:2]) > np.asarray(A4) / 2 + CONTACT_TOL):
                return False
            overlap = list((bottom - below.pos) @ rb)
            for axis in (0, 1):
                overlap = _clip_polygon(overlap, axis, -A4[axis] / 2, False)
                overlap = _clip_polygon(overlap, axis, A4[axis] / 2, True)
            if not overlap:
                return False
            separation = min(v[2] for v in overlap) - T / 2
        else:
            separation = float(bottom[:, 2].min() - FLOOR)
        if not -CONTACT_TOL - EPS <= separation <= SUPPORT_TOL + EPS:
            return False
    return True


def stack_report(objects) -> dict:
    p = objects["cassette"]
    faces = pad_faces(p)
    corners = np.concatenate([_corners(objects[n]) for n in NAMES])
    contact = _contact_gaps(objects, faces)
    report = dict(
        near_gap=float(max(contact["near"], default=math.inf)),
        far_gap=float(max(contact["far"], default=math.inf)),
        rear_gap=float(min(contact["rear"], default=math.inf)),
        front=[float(_corners(objects[n])[:, 1].min() - YF) for n in NAMES],
        yaw=[math.degrees(bundle_yaw(objects[n])) for n in NAMES],
        tilt=[math.degrees(bundle_tilt(objects[n])) for n in NAMES],
        top=float(corners[:, 2].max() - FLOOR),
        pressed={k: float(p.joints[k]) for k in ("pad_near", "pad_far", "pad_length")},
        latches={k: float(p.joints[k]) for k in ("latch_width", "latch_length")},
    )
    report["flat"] = bool(max(report["tilt"]) <= FLAT_DEG + EPS and report["top"] <= N_BUNDLES * T + 0.0015)
    report["squeezed"] = bool(max(report["pressed"].values()) > SQUEEZE + EPS)
    report["square"] = bool(max(abs(a) for a in report["yaw"]) <= SQUARE_DEG + EPS
                            and max(report["front"]) <= SNUG + EPS)
    report["length_snug"] = bool(report["rear_gap"] <= SNUG + EPS and report["square"])
    report["width_snug"] = bool(len(contact["near"]) == N_BUNDLES and len(contact["far"]) == N_BUNDLES
                                 and report["near_gap"] <= SNUG + EPS and report["far_gap"] <= SNUG + EPS)
    report["latched"] = bool(max(report["latches"].values()) < UNLOCK - EPS)
    report["contained"] = bool(corners[:, 0].min() >= X0 + 0.006 - CONTACT_TOL - EPS
                               and corners[:, 0].max() <= X1 - 0.006 + CONTACT_TOL + EPS
                               and corners[:, 1].min() >= YF - CONTACT_TOL - EPS
                               and corners[:, 1].max() <= Y1 - 0.009 + CONTACT_TOL + EPS
                               and corners[:, 2].min() >= FLOOR - CONTACT_TOL - EPS)
    report["supported"] = _supported_stack(objects)
    report["penetrating"] = bool(min((g for values in contact.values() for g in values), default=0.0)
                                 < -CONTACT_TOL - EPS)
    return report


# -- the per-trial record -------------------------------------------------------------------------------------------
_rec = parts.__dict__.setdefault("paper_cassette_record", {})


def fresh() -> None:
    _rec.update(blind=False, gap=None, since=None, held=False, prev=None, t=None, free={g: False for g, _ in LOCKABLE},
                n=0, qprev={g: None for g, _ in LOCKABLE})


if not _rec:
    fresh()


# -- hands and the release check (gripper hull faces sampled every 3 mm, corners included) ----------------------------
def _face_points(hull, step=FACE_STEP) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            for s_ in range(m + 1):
                for t_ in range(m + 1 - s_):
                    pts.append((c + (a - c) * (s_ / m) + (b - c) * (t_ / m))[None, :])
    pts = np.concatenate(pts)
    _, keep = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(keep)]])


_FACES: dict = {}


def _link_faces(name):
    if name not in _FACES:
        from yam_arena import analysis

        _FACES[name] = [_face_points(h) for h in analysis.link_hulls().get(name, [])]
    return _FACES[name]


def scored_boxes(objects):
    """(centre, rotation, size) of every scored thing: the paper bundles and every moving part of the guides."""
    p = objects["cassette"]
    poses = {k: v for k, v in _link_poses(p).items() if k != "base"}
    out = [(c, r, s) for _, c, r, s in CASSETTE.world_boxes(poses)]
    out += [_bundle_box(objects[n]) for n in NAMES]
    return out


def hand_gap(points, objects) -> float:
    """Closest any sampled gripper face point comes to a scored box (m; negative inside)."""
    from yam_arena import analysis

    return float(min(analysis.box_signed_distance(points, c, r, s).min() for c, r, s in scored_boxes(objects)))


def _hand_points(sim):
    """World points on both arms' gripper and finger-tip hull faces, or None unless all six links were read."""
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    pts, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                faces = _link_faces(n)
                if faces:
                    found += 1
                rot, pos = _rot(poses[i, 3:]), poses[i, :3] - origin
                pts += [f @ rot.T + pos for f in faces]
    if found != 2 * len(_HAND_LINKS) or not pts:
        return None
    pts = np.concatenate(pts)
    return pts if np.isfinite(pts).all() else None


# -- latches: squeezed or unseated pawls free a guide; seated pawls hold it -------------------------------------------
def _write_lock(sim, guide: str, locked: bool, at: float | None = None) -> None:
    import torch

    base = sim.env.unwrapped
    asset = base.scene["cassette"]
    ids = asset.find_joints([guide], preserve_order=True)[0]
    k = LOCK_K if locked else 0.0
    c = LOCK_C if locked else FREE_C
    if locked and at is not None:
        captured = LOCK_PITCH * math.floor(at / LOCK_PITCH + 0.5)
        sim.set_target("cassette", guide, captured)
    asset.write_joint_stiffness_to_sim_index(stiffness=torch.tensor([[k]], dtype=torch.float32, device=base.device),
                                             joint_ids=ids)
    asset.write_joint_damping_to_sim_index(damping=torch.tensor([[c]], dtype=torch.float32, device=base.device),
                                           joint_ids=ids)


def _tooth_target(at):
    centre = LOCK_PITCH * math.floor(at / LOCK_PITCH + 0.5)
    return centre if abs(at - centre) <= LOCK_WINDOW + EPS else None


def _crossed_tooth(before, after):
    """The first tooth centre a free carriage passed between two samples, or None. A pawl riding the rack catches
    the first tooth it passes; the check runs at the control rate, so a fast slide can step over a window."""
    if before is None or not np.isfinite([before, after]).all() or abs(after - before) <= EPS:
        return None
    k = math.floor(before / LOCK_PITCH) if after < before else math.ceil(before / LOCK_PITCH)
    centre = LOCK_PITCH * k
    return centre if min(before, after) - EPS <= centre <= max(before, after) + EPS else None


@parts.on_reset
def _new_trial(sim) -> None:
    fresh()
    for g, _ in LOCKABLE:
        _write_lock(sim, g, True, 0.0)
    for j in CASSETTE.joints:
        sim.set_target("cassette", j.name, 0.0)


def _still_vector(objects):
    p = objects["cassette"]
    v = [objects[n].pos for n in NAMES] + [np.array([p.joints[k] for k in ("width_near", "width_far", "length",
                                                                           "pad_near", "pad_far", "pad_length")]),
         0.026 * np.array([p.joints[k] for k in ("latch_width", "latch_length")])]
    return np.concatenate(v)


@parts.on_step
def _step(sim) -> None:
    objects = sim.states()
    if not _states_valid(objects) or not np.isfinite(float(sim.t)):
        _rec.update(blind=True, gap=None, since=None, held=False, prev=None, qprev={g: None for g, _ in LOCKABLE})
        return
    p = objects["cassette"]
    q = p.joints
    for g, lever in LOCKABLE:
        # A released square pawl can rest on a tooth crest. It stays free until
        # a small carriage slide reaches an engagement window, or passes a tooth
        # between two samples (it then holds at that tooth); no magic snap.
        released = q[lever] < UNLOCK - EPS
        tooth = _tooth_target(q[g])
        if tooth is None and released:
            tooth = _crossed_tooth(_rec["qprev"].get(g), q[g])
        _rec["qprev"][g] = q[g] if released else None  # only motion with the pawl down can catch a tooth
        free = (q[lever] >= UNLOCK - EPS
                or (_rec["free"][g] and tooth is None))
        if free != _rec["free"][g]:
            _write_lock(sim, g, not free, q[g] if tooth is None else tooth)
            _rec["free"][g] = free
    sim.set_target("cassette", "width_far", q["width_near"])  # one-way active PD tracking, not reciprocal gearing
    t = float(sim.t)
    vec = _still_vector(objects)
    speed = None
    if _rec["prev"] is not None and t > _rec["prev"][0] + EPS:
        d = np.abs(vec - _rec["prev"][1])
        speed = float(d.max() / (t - _rec["prev"][0]))
    _rec["prev"] = (t, vec)
    _rec["t"] = t
    try:
        pts = _hand_points(sim)
    except Exception as e:  # noqa: BLE001  the arms could not be read: no release can be confirmed
        print(f"[hands] unreadable: {e!r}", file=sys.stderr, flush=True)
        pts = None
    if pts is None:
        _rec.update(blind=True, gap=None, since=None, held=False)
    else:
        _rec["gap"] = hand_gap(pts, objects)
        if not np.isfinite(_rec["gap"]):
            _rec.update(blind=True, gap=None, since=None, held=False)
            return
        still = speed is not None and speed < 0.005 and all(objects[n].resting(0.005, 0.1) for n in NAMES)
        if _rec["gap"] > CLEAR + EPS and still and not any(_rec["free"].values()):
            if _rec["since"] is None:
                _rec["since"] = t
            _rec["held"] = t - _rec["since"] >= HOLD_S - EPS
        else:
            _rec.update(since=None, held=False)
    _rec["n"] += 1
    if os.environ.get("TRACE_OUT") and _rec["n"] % 5 == 0:
        row = dict(t=t, gap=_rec["gap"], since=_rec["since"], held=_rec["held"], blind=_rec["blind"], free=dict(_rec["free"]),
                   joints={k: float(v) for k, v in q.items()}, score=score(objects))
        row["paper_states"] = {name: {key: np.asarray(getattr(objects[name], key)).tolist()
                                       for key in ("pos", "quat_xyzw", "lin_vel", "ang_vel")} for name in NAMES}
        row["link_poses"] = {name: [np.asarray(pos).tolist(), np.asarray(quat).tolist()]
                             for name, (pos, quat) in p.links.items()}
        row.update({k: v for k, v in stack_report(objects).items()})
        try:  # diagnostics only: gripper and fingertip link poses (world, env origin removed)
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            row["hands"] = {}
            for arm in ("left_robot", "right_robot"):
                asset = base.scene[arm]
                poses = parts._np(asset.data.body_link_pose_w)[0]
                for i, n in enumerate(asset.body_names):
                    if n in _HAND_LINKS:
                        row["hands"][f"{arm}/{n}"] = [(poses[i, :3] - origin).tolist(), poses[i, 3:].tolist()]
        except Exception as e:  # noqa: BLE001
            row["hands"] = repr(e)
        with pathlib.Path(os.environ["TRACE_OUT"]).with_name("cassette.jsonl").open("a") as f:
            f.write(json.dumps(row) + "\n")


# -- scoring -----------------------------------------------------------------------------------------------------------
def score(objects):
    """End score: finite readable state, both pawls seated, levers released, hands >5 mm clear and still for 1 s.
    All four layers must be contained, supported, flat and not penetrating or squeezed >2 mm. Then 0.5 for rear
    contact plus square/feed alignment and independently 0.5 for both finite-span width-guide contacts."""
    try:
        if (_rec["blind"] or not _rec["held"] or any(_rec["free"].values()) or _rec["gap"] is None
                or not np.isfinite(_rec["gap"]) or _rec["gap"] <= CLEAR + EPS
                or _rec["t"] is None or _rec["since"] is None
                or not np.isfinite([_rec["t"], _rec["since"]]).all()
                or _rec["t"] - _rec["since"] < HOLD_S - EPS or not _states_valid(objects)):
            return 0.0
        r = stack_report(objects)
    except (KeyError, ValueError, TypeError):
        return 0.0
    if (r["squeezed"] or not r["flat"] or not r["latched"] or not r["contained"]
            or not r["supported"] or r["penetrating"]):
        return 0.0
    return 0.5 * r["length_snug"] + 0.5 * r["width_snug"]


def success(objects):
    return score(objects) == 1.0


def distance(objects):
    try:
        if not _states_valid(objects):
            return 1.0
        r = stack_report(objects)
    except (KeyError, ValueError, TypeError):
        return 1.0
    return float(max(r["rear_gap"], 0.0) + max(r["near_gap"], 0.0) + max(r["far_gap"], 0.0))


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Open loop from the published layout: the guides' visible start settings and the paper's printed size (A4). The
# width guides go first: squeezing the loose bundles between their flat pads takes out their skew while nothing yet
# pins them along the tray. Then the length guide pushes the squared bundles up against the feed edge. (Pushing first
# wedges a skewed bundle corner-to-corner between the feed face and the rear pad, and its skew can then no longer be
# taken out by the width guides.)
# Each latch is pinched shut (fingers either side of the blue latch), the guide slid with it, and the latch let go.
# The arm is soft (about 150-190 N/m), so a guide is pressed only as hard as the hand is commanded past the edge.
# Each guide is first pushed well past the stack's edge (squaring the stack, which takes a few newtons to slide the
# lowest sheets), then eased back to still slightly pressing the stack, and let go there. Once the lever is let up,
# the hand draws back slowly; the pad's spring eases the guide out behind it until the released pawl drops into the
# next tooth instead of staying on a crest (the arm is much softer than the pad, so the guide moves far less).
# The rear latch is pinched only firmly enough to hold its lever on the stop (about 10 N per finger across the
# 16 mm latch-and-post span), not crushed: its push is the hardest one, the soft wrist tips the gripper about 1 deg
# per newton of push, and a crushing pinch drove the fingertip into the thin lever until it passed through.
PUSH_L, EASE_L = 0.045, 0.005
PUSH_W, EASE_W = 0.025, 0.0085
SETTLE = 0.004  # outward draw after the lever is let up: several tooth pitches


def _grip(gap_m: float) -> float:
    return FINGER_OPEN * gap_m / 0.094


PINCH_Z = DECK + 0.060  # tips 13 mm lower, across the top of the raised latch


def demo():
    open_g, shut_g = _grip(0.032), _grip(0.006)
    # near width guide: latch spans x WN0-0.021 .. WN0-0.003; it must travel (WIDE - A4 width) / 2
    wx0 = WN0 - 0.012
    wx1 = wx0 + (WIDE - A4[0]) / 2 + PUSH_W
    wx2 = wx0 + (WIDE - A4[0]) / 2 + EASE_W
    yaw = math.pi / 2  # fingers close along x
    right = [
        Waypoint(1.5, (0.20, -0.22, TOP + 0.14), FINGER_OPEN, "aside", yaw),
        Waypoint(1.5, (wx0, TAB_YC, PINCH_Z + 0.03), open_g, "over the width latch", yaw),
        Waypoint(1.0, (wx0, TAB_YC, PINCH_Z), open_g, "down round the latch", yaw),
        Waypoint(0.8, (wx0, TAB_YC, PINCH_Z), shut_g, "squeeze the latch", yaw),
        Waypoint(2.0, (wx1, TAB_YC, PINCH_Z), shut_g, "close the guides on the paper", yaw),
        Waypoint(1.0, (wx1, TAB_YC, PINCH_Z), shut_g, "hold the stack square", yaw),
        Waypoint(0.8, (wx2, TAB_YC, PINCH_Z), shut_g, "ease back to pressing lightly", yaw),
        Waypoint(0.5, (wx2, TAB_YC, PINCH_Z), shut_g, "settle", yaw),
        Waypoint(0.6, (wx2, TAB_YC, PINCH_Z), _grip(0.016), "release the pawl toward its teeth", yaw),
        Waypoint(1.2, (wx2 - SETTLE, TAB_YC, PINCH_Z), _grip(0.016), "settle the pawl into a tooth", yaw),
        Waypoint(0.8, (wx2 - SETTLE, TAB_YC, PINCH_Z), open_g, "let the latch go", yaw),
        Waypoint(1.0, (wx2 - SETTLE, TAB_YC, PINCH_Z + 0.05), open_g, "up", yaw),
        Waypoint(1.5, (0.20, -0.24, TOP + 0.16), FINGER_OPEN, "park", 0.0),
    ]
    t_up = sum(w.duration for w in right[:-1])  # the right hand is up and clear of the width latch
    # length guide: latch spans y LF0+0.003 .. LF0+0.021 at the start; it must travel LEGAL - A4 length
    ly0 = LF0 + 0.012
    ly1 = ly0 - (LEGAL - A4[1]) - PUSH_L
    ly2 = ly0 - (LEGAL - A4[1]) - EASE_L
    pinch_l = _grip(0.012)  # lighter pinch for the rear latch (see above)
    left = [
        Waypoint(1.5, (0.20, 0.24, TOP + 0.16), FINGER_OPEN, "aside", 0.0),
        Waypoint(t_up + 0.5 - 3.0, (0.20, 0.24, TOP + 0.16), FINGER_OPEN, "wait", 0.0),
        Waypoint(1.5, (XC, ly0 + 0.01, TOP + 0.12), FINGER_OPEN, "over the length latch", 0.0),
        Waypoint(1.2, (XC, ly0, PINCH_Z + 0.03), open_g, "open over it", 0.0),
        Waypoint(1.0, (XC, ly0, PINCH_Z), open_g, "down round the latch", 0.0),
        Waypoint(0.8, (XC, ly0, PINCH_Z), pinch_l, "squeeze the latch", 0.0),
        Waypoint(3.0, (XC, ly1, PINCH_Z), pinch_l, "push the guide and paper up to the feed edge", 0.0),
        Waypoint(2.0, (XC, ly1, PINCH_Z), pinch_l, "hold the stack square", 0.0),  # skewed groups square slowly
        Waypoint(1.0, (XC, ly2, PINCH_Z), pinch_l, "ease the guide back to pressing lightly", 0.0),
        Waypoint(0.6, (XC, ly2, PINCH_Z), pinch_l, "settle", 0.0),
        Waypoint(0.6, (XC, ly2, PINCH_Z), _grip(0.016), "release the pawl toward its teeth", 0.0),
        Waypoint(1.2, (XC, ly2 + SETTLE, PINCH_Z), _grip(0.016), "settle the pawl into a tooth", 0.0),
        Waypoint(0.8, (XC, ly2 + SETTLE, PINCH_Z), open_g, "let the latch go", 0.0),
        Waypoint(1.0, (XC, ly2 + SETTLE, PINCH_Z + 0.05), open_g, "up", 0.0),
        Waypoint(1.5, (0.20, 0.24, TOP + 0.16), FINGER_OPEN, "park", 0.0),
    ]
    return {"left": left, "right": right}
