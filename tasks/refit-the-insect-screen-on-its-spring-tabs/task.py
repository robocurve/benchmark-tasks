"""A cropped section of a small window is fixed on the bench in front of the robot: the wall below it with its window
stool (the inside sill board), the two jambs, and the head. The screen sits in two channels: a shallow one along the
sill, behind a 4 mm lip, and a deeper one in the head, behind a lip that covers the top 6 mm of the seated screen.
A stop frames the opening from behind, with the glass beyond. The aluminium insect screen stands on the stool,
leaning back with its top against the head's front face, mesh toward the robot. Two bought corner tension springs
(flat bowed steel leaves, slipped into the frame's top corners) bow up out of its top rail, and two pull tabs stand out
of its room-side face, one on each stile. Where the screen stands along the stool changes between trials.
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
import tilt  # noqa: E402  (the fingers lean in toward the screen, under the wrist)
from yam_arena.demo import FINGER_OPEN, Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
EPS = 1e-6

ALU = (0.78, 0.79, 0.80)  # mill-finish aluminium screen frame
MESH = (0.22, 0.23, 0.25)  # charcoal fibreglass mesh
TAB_C = (0.05, 0.05, 0.05)  # black vinyl pull tabs
SPRING_C = (0.72, 0.73, 0.75)  # zinc-plated steel tension springs
WHITE = (0.93, 0.93, 0.91)  # painted wood window parts
STOP_C = (0.70, 0.62, 0.50)  # the stop and the two lips, a contrasting wood tone so they read from the room
GLASS = (0.62, 0.74, 0.80)
WALL = (0.86, 0.84, 0.78)


def _f(v):
    """Plain Python floats: a position of NumPy floats crashes the simulator at reset."""
    return tuple(float(x) for x in v)


# -- the screen (part frame: origin at the middle of its bottom edge, mid-thickness; x toward the window) -------------
W, H = 0.406, 0.356  # approximately 16 x 14 in outside size
T, FACE = 0.0111, 0.01905  # 7/16 x 3/4 in roll-formed frame profile
SCREEN_KG = 0.22  # uncalibrated assembly estimate; 1.524 m outside perimeter plus mesh/corners/springs/tabs
FACE_X = -T / 2  # the room-side face
# Corner tension springs (Prime-Line PL 7776, 2-3/4 x 1/4 in heat-treated steel): each slips into one top corner of the
# frame and bows up along the top edge toward the middle. The leaf's flexing is modelled as a pad at its crown that
# slides down into the rail against a linear spring (q > 0 is pressed in; q = SPRING_CROWN is pressed flat).
SPRING_LEN = 0.06985  # 2-3/4 in, from the corner inward
SPRING_WIDTH = 0.00635  # 1/4 in, across the rail's thickness
# Measured Nov 23 on one package-checked PL 7776: digital calipers, ruler, kitchen scale (1 g display).
# Free bench bow 7 mm; strip 0.27 mm; 161 g (1.58 N) to press its center flat. See fabrication/MEASUREMENTS.md.
SPRING_CROWN = 0.007  # measured free bow, adopted as model crown/travel above the rail; installed fit must be checked
SPRING_STRIP_THICKNESS = 0.00027  # m, measured on that one spring; provenance, not a separate flexible-strip model
SPRING_FLAT_N = 1.58  # N, measured free-part flat force on a kitchen scale (rounded from 161 g)
SPRING_PAD = 0.020  # length of the crown that the head channel's ceiling bears on (modelled as a flat pad)
SPRING_Y = W / 2 - SPRING_LEN / 2  # crown centre, 34.9 mm in from each side edge
SPRING_K = 400.0  # N/m, model stiffness; measured free-part endpoint secant is 226 N/m
SPRING_DAMPING = 1.0  # N s/m, uncalibrated damping of the linear crown proxy
SPRING_EFFORT_LIMIT = 20.0  # N solver drive cap; above the 2.84905 N maximum elastic drive
FRAME_WINDOW_FRICTION = (0.6, 0.5)  # static/dynamic contact coefficients, uncalibrated estimates
SPRING_KG = 0.005  # uncalibrated 5 g moving-mass proxy for solver stability
GRAVITY = 9.81  # m/s^2, both the proxy-weight offset and static bearing-load estimate
SPRING_PROXY_OFFSET_N = SPRING_KG * GRAVITY  # 0.04905 N; compensates artificial proxy weight upright
# Conservative linear-model assumption: 400 N/m is 1.77 times the measured free-part secant
# (1.58 N / 7 mm = 225.7 N/m). With the separate artificial-weight correction, net upright
# resistance is 400*q N, pointwise above that assumed linear secant throughout 0..7 mm.
# No actual installed preload was measured; do not manufacture one from the bench endpoint.
# This linear envelope is NOT a proven installed loading/unloading upper bound or uncertainty
# margin. Enrol the actual frame/springs; reject/remodel if their resistance exceeds the model.
SPRING_PRELOAD_N = 0.0  # installed preload unmeasured; nominal model has none
SPRING_DRIVE_OFFSET_N = SPRING_PROXY_OFFSET_N + SPRING_PRELOAD_N
SPRING_TARGET = -SPRING_DRIVE_OFFSET_N / SPRING_K
# Pull tabs (Prime-Line PL 14621, flat vinyl tongue): maker's line art gives 1-1/16 in overall height, 15/16 in
# spline foot width, 5/8 in tongue width, 1/4 in foot depth and 3/32 in base thickness. Installed on a vertical
# stile, the tongue projects toward the room. This is a rigid approximation; its bend/compliance is not calibrated.
TAB_PROUD, TAB_THK, TAB_TALL = 0.0270, 0.00238, 0.0159  # nominal proud projection x, thickness y, tongue tip width z
TAB_BASE_TALL, TAB_FOOT_W = 0.0238, 0.00635  # foot width along the stile z, foot depth across stile y
TAB_FRICTION = (1.0, 0.9)  # vinyl (estimate)
TAB_Y = W / 2 - 0.015  # in the stile's spline groove, 15 mm in from the side edge
TAB_ZS = {1: 0.140, -1: 0.050}  # centre heights above the bottom edge: left stile 140 mm, right 50 mm. Two pinches at
# one height act as a hinge about the line between them; staggered they hold the screen's angle. At 60 mm apart the
# screen still turned 3-10 degrees in the grips at the 22 degree approach, and at 45 mm it turned to about 30 degrees and
# fell, so the stagger is 90 mm. The left tab cannot go higher: at 150 mm and above the left forearm meets the head
# while the screen is turned up (factory check); the right tab is lowered instead.
TAB_Z = TAB_ZS[1]
TAB_XC = FACE_X - TAB_PROUD / 2


def _screen_shapes():
    out = []
    for s in (-1, 1):  # stiles
        out.append(parts.box((T, FACE, H), at=(0.0, s * (W / 2 - FACE / 2), H / 2), color=ALU))
    for z in (FACE / 2, H - FACE / 2):  # rails
        out.append(parts.box((T, W - 2 * FACE, FACE), at=(0.0, 0.0, z), color=ALU))
    out.append(parts.box((0.001, W - 2 * FACE, H - 2 * FACE), at=(0.0, 0.0, H / 2), color=MESH))
    for s in (-1, 1):  # spline foot and tapered flat tongue (rigid approximation of the bought vinyl part)
        out.append(parts.box((TAB_THK, TAB_FOOT_W, TAB_BASE_TALL),
                             at=(FACE_X - TAB_THK / 2, s * TAB_Y, TAB_ZS[s]), color=TAB_C, friction=TAB_FRICTION))
        points = [(x, s * TAB_Y + y, TAB_ZS[s] + z)
                  for x, half_z in ((FACE_X - TAB_THK, TAB_BASE_TALL / 2), (FACE_X - TAB_PROUD, TAB_TALL / 2))
                  for y in (-TAB_THK / 2, TAB_THK / 2) for z in (-half_z, half_z)]
        out.append(parts.hull(points, ((0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1),
                                      (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)), color=TAB_C,
                              friction=TAB_FRICTION))
    return out


def _spring(side: int) -> parts.Slider:
    """One corner tension spring: its crown is pushed down toward the top rail against the leaf's stiffness (q > 0 is
    pressed in, SPRING_CROWN pressed flat). A separate 0.04905 N correction compensates artificial crown weight upright;
    installed preload remains unmeasured and the nominal model has none."""
    shapes = (parts.box((SPRING_WIDTH, SPRING_PAD, SPRING_CROWN), at=(0.0, side * SPRING_Y, H + SPRING_CROWN / 2),
                        color=SPRING_C),)
    return parts.Slider("spring_l" if side > 0 else "spring_r", shapes=shapes, axis=(0.0, 0.0, -1.0),
                        travel=SPRING_CROWN, mass=SPRING_KG, spring=SPRING_K, damping=SPRING_DAMPING, max_effort=SPRING_EFFORT_LIMIT,
                        target=SPRING_TARGET)


# -- the window section (world) --------------------------------------------------------------------------------------
X_STOP = 0.400  # the stop's room-side face: the screen's back face seats here
PLAY = 0.003  # side play between the screen and each jamb
DEPTH = 0.045  # jamb depth from the wall face to the seated screen's room-side face
X_WALL = X_STOP - T - DEPTH  # room-side face of the jambs and head
STOOL = 0.060  # the stool projects this far in front of the wall
X_STOOL = X_WALL - STOOL
Z_SILL = TOP + 0.070  # stool top = the sill channel's floor
Y_JAMB = W / 2 + PLAY  # the window frame's side faces, where the screen's sides sit (3 mm play)
REVEAL = 0.040  # the interior jambs (the reveal in front of the window frame) stand this far out from the screen's sides
Y_REVEAL = W / 2 + REVEAL
JAMB_W = 0.040
X_BACK = X_STOP + 0.040  # back of the cropped section
STOP_LAP = 0.010  # the stop covers the screen's edge by this much on every side
STOP_T = 0.012
CLR = 0.0015  # sill channel width = screen thickness + 1.5 mm
X_LIPB = X_STOP - T - CLR  # back face of the sill lip
HEAD_W = 0.0159  # head channel 5/8 in wide: room to tip the screen's top into it (a 5/8 in router cut or strips)
X_LIPH = X_STOP - HEAD_W  # back face of the head lip, 4.8 mm in front of the seated screen's room-side face
LIP_T = 0.006  # sill lip, front to back
SILL_LIP_H = 0.004  # sill lip height above the channel floor
HEAD_LAP = 0.006  # the head lip covers the seated screen's top 6 mm
GAP_TOP = 0.006  # head channel's ceiling above the seated screen's top edge: the springs are pressed 1 mm when seated
Z_HL = Z_SILL + H - HEAD_LAP  # underside of the head lip (and of the head in front of the channel)
Z_HC = Z_SILL + H + GAP_TOP  # head channel's ceiling
Z_HTOP = Z_HC + 0.030
# Two straight inspection/probe ports through only the room-side head, ending at the channel.
# Neither the bearing ceiling nor the stop is cut. A <=13 mm wide, >=80 mm long 0.10 mm
# blade reaches each leaf apex horizontally; no bend or blind turn through the frame slot.
PROBE_PORT_W, PROBE_PORT_H = 0.016, 0.003
WIN_ORIGIN = np.array([X_STOP, 0.0, Z_SILL])


def _box(lo, hi, color):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return parts.box(_f(hi - lo), at=_f((lo + hi) / 2 - WIN_ORIGIN), color=color)


def window_boxes():
    """Every fixed box of the window section, world frame: (lo, hi, colour)."""
    yo = Y_REVEAL + JAMB_W
    out = [((X_STOOL, -0.37, TOP), (X_BACK, 0.37, Z_SILL - 0.003), WALL),  # wall below, under the stool board
           ((X_STOOL, -0.37, Z_SILL - 0.003), (X_BACK, 0.37, Z_SILL), WHITE),  # stool board and channel floor
           ((X_LIPB - LIP_T, -Y_REVEAL, Z_SILL), (X_LIPB, Y_REVEAL, Z_SILL + SILL_LIP_H), STOP_C),  # sill lip
           ((X_STOP, -Y_JAMB, Z_SILL), (X_STOP + STOP_T, Y_JAMB, Z_SILL + STOP_LAP), STOP_C),  # sill stop
           ((X_WALL, -yo, Z_HL), (X_LIPH, yo, Z_HL + 0.010), STOP_C),  # head lip
           ((X_WALL, -yo, Z_HL + 0.010), (X_LIPH, yo, Z_HTOP), WHITE),  # head, room side of the channel
           ((X_LIPH, -yo, Z_HC), (X_BACK, yo, Z_HTOP), WHITE),  # head over the channel
           ((X_STOP, -Y_JAMB, Z_SILL + H - STOP_LAP), (X_STOP + STOP_T, Y_JAMB, Z_HC), STOP_C),  # head stop
           ((X_STOP + 0.024, -Y_JAMB, Z_SILL), (X_STOP + 0.030, Y_JAMB, Z_HC), GLASS)]  # glass
    for s in (-1, 1):
        def jb(xl, xh, yl, yh, zl, zh, c=WHITE):
            ys = sorted((s * yl, s * yh))
            out.append(((xl, ys[0], zl), (xh, ys[1], zh), c))
        jb(X_WALL, X_LIPB, Y_REVEAL, yo, Z_SILL, Z_HL)  # interior jamb (the reveal)
        jb(X_LIPB, X_BACK, Y_JAMB, yo, Z_SILL, Z_HC)  # window frame's side, beside the seated screen
        jb(X_STOP, X_STOP + STOP_T, W / 2 - STOP_LAP, Y_JAMB, Z_SILL, Z_HC, STOP_C)  # side stop
    # Subtract the two probe passages from head/lip boxes on the room side of the channel.
    for side in (-1, 1):
        cutlo = np.array([X_WALL, side * SPRING_Y - PROBE_PORT_W / 2, Z_HC - PROBE_PORT_H])
        cuthi = np.array([X_LIPH, side * SPRING_Y + PROBE_PORT_W / 2, Z_HC])
        split = []
        for lo, hi, colour in out:
            lo, hi = np.array(lo, float), np.array(hi, float)
            il, ih = np.maximum(lo, cutlo), np.minimum(hi, cuthi)
            if np.any(ih - il <= 1e-9):
                split.append((tuple(lo), tuple(hi), colour))
                continue
            # Disjoint slabs partition the remainder of this box around its rectangular hole.
            remlo, remhi = lo.copy(), hi.copy()
            for axis in range(3):
                if il[axis] > remlo[axis] + 1e-9:
                    end = remhi.copy(); end[axis] = il[axis]
                    split.append((tuple(remlo), tuple(end), colour)); remlo[axis] = il[axis]
                if ih[axis] < remhi[axis] - 1e-9:
                    start = remlo.copy(); start[axis] = ih[axis]
                    split.append((tuple(start), tuple(remhi), colour)); remhi[axis] = ih[axis]
        out = split
    return out


WINDOW = parts.Part("window", pos=_f(WIN_ORIGIN), shapes=[_box(lo, hi, c) for lo, hi, c in window_boxes()],
                    friction=FRAME_WINDOW_FRICTION, contact_offset=0.002)

# -- start: the screen stands on the stool, leaning back against the head's front face --------------------------------
LEAN = math.radians(6.0)


def _ry(a) -> np.ndarray:
    return parts._rot((0.0, 1.0, 0.0), a)


def lean_origin(y=0.0, lean=LEAN) -> np.ndarray:
    """The screen's origin (bottom-edge middle) when it stands on the stool on its back bottom edge, leaning back
    ``lean`` with its top back edge against the head's front face, centred on ``y``."""
    r = _ry(lean)
    bottom_back = np.array([X_WALL - H * math.sin(lean) - 0.0008, y, Z_SILL + 0.0005])
    return bottom_back - r @ np.array([T / 2, 0.0, 0.0])


def _quat(m) -> tuple:
    return tuple(float(v) for v in parts._matrix_to_quat(m))


START_A = lean_origin(0.0)
SCREEN = parts.Part("screen", pos=_f(START_A), quat=_quat(_ry(LEAN)), mass=SCREEN_KG - 2 * SPRING_KG,
                    shapes=_screen_shapes(), joints=[_spring(+1), _spring(-1)], friction=FRAME_WINDOW_FRICTION,
                    contact_offset=0.002)
SEAT = np.array([X_STOP - T / 2, 0.0, Z_SILL])  # the seated screen's origin
SPRINGS = ("spring_l", "spring_r")

OBJECTS = [WINDOW, SCREEN]
LOTS = lots.declare(OBJECTS, lots.Lot("A", place={"screen": _f(START_A)}),
                    lots.Lot("B", place={"screen": _f(lean_origin(0.025))}),
                    lots.Lot("C", place={"screen": _f(lean_origin(-0.025))}))
SUCCESS_HOLD_S = 1.5  # the 1 s hold is enforced in score() itself; this only keeps a run going past success


# -- scoring geometry ----------------------------------------------------------------------------------------------
def _rot(q) -> np.ndarray:
    return parts._quat_to_matrix(q)


def _local(st, p) -> np.ndarray:
    """A screen-frame point in the world."""
    return np.asarray(st.pos, float) + _rot(st.quat_xyzw) @ np.asarray(p, float)


CORNERS = [(T / 2, s * W / 2, z) for s in (-1, 1) for z in (0.0, H)]  # back-face corners
TOP_CORNERS = [(x, s * W / 2, H) for x in (-T / 2, T / 2) for s in (-1, 1)]
FLUSH_TOL = 0.0015  # every back corner within 1.5 mm of the stop's face (the sill channel's own play)
SILL_TOL = 0.0015  # bottom corners within 1.5 mm above the channel floor (1 mm below allowed for contact)
TILT_TOL = 2.0  # degrees from upright, square to the opening
TOP_IN = 0.003  # captured at the top: the top edge at least 3 mm up behind the head lip
TOP_TILT = 15.0  # degrees from upright allowed for the 0.5 tier
BEARING_TOL = 0.0001  # m, unchanged 0.10 mm leaf-apex/ceiling allowance; straight room-side feeler
JOINT_TOL = 0.0001  # m, solver joint-limit allowance
LINK_TOL = 0.00025  # m, maximum link position disagreement with joint kinematics
LINK_ROT_TOL = 0.002  # rad, maximum link orientation disagreement


def spring_points(st):
    """Four top-face corners per crown, from the actual reported moving-link poses."""
    out = []
    for j in SPRINGS:
        pos, quat = st.links[j]
        r = _rot(quat)
        out.append(np.array([np.asarray(pos, float) + r @ np.array([x, y, SPRING_CROWN / 2])
                             for x in (-SPRING_WIDTH / 2, SPRING_WIDTH / 2)
                             for y in (-SPRING_PAD / 2, SPRING_PAD / 2)]))
    return np.asarray(out)


def validate_springs(st):
    """Missing/nonfinite, out-of-travel or detached link states invalidate the trial."""
    expected = SCREEN.link_poses(st.pos, _rot(st.quat_xyzw), st.joints)
    for j in SPRINGS:
        q = float(st.joints[j])
        pos, quat = st.links[j]
        if not np.isfinite([q, *pos, *quat]).all() or abs(np.linalg.norm(quat) - 1.0) > 0.001:
            raise ValueError("nonfinite/invalid spring link")
        if not -JOINT_TOL <= q <= SPRING_CROWN + JOINT_TOL:
            raise ValueError("spring outside travel")
        ep, er = expected[j]
        angle = math.acos(float(np.clip((np.trace(er.T @ _rot(quat)) - 1.0) / 2.0, -1.0, 1.0)))
        if np.linalg.norm(np.asarray(pos) - ep) > LINK_TOL or angle > LINK_ROT_TOL:
            raise ValueError("spring link disagrees with joint")


def spring_apices(st):
    """Two width-edge points on each leaf's apex line, rather than fictional flat-pad corners.
    A bowed leaf contacts at its longitudinal apex; a straight blade centered there tests this line.
    The pad collider remains a declared approximation of that leaf, not measured leaf geometry."""
    return np.array([[np.asarray(st.links[j][0], float) + _rot(st.links[j][1]) @
                      np.array([x, 0.0, SPRING_CROWN / 2])
                      for x in (-SPRING_WIDTH / 2, SPRING_WIDTH / 2)] for j in SPRINGS])


def spring_axial_resistance(st):
    """Calculated static force along the slider, not a contact sensor or ceiling-normal force."""
    upz = float(_rot(st.quat_xyzw)[2, 2])
    return np.array([SPRING_K * float(st.joints[j]) + SPRING_DRIVE_OFFSET_N
                     - SPRING_KG * GRAVITY * upz for j in SPRINGS])


def springs_bearing(st):
    """Both leaf apex lines meet the ceiling within 0.10 mm with positive static axial resistance.
    At least one width-edge point touches per leaf, as a real bowed leaf need not bear across
    its entire width when tilted. All apex points remain inside the channel; penetration guards
    still inspect the complete pad. Straight feeler refusal at the apex is the physical test."""
    pts = spring_apices(st)
    gaps = Z_HC - pts[:, :, 2]
    return (bool(np.all(np.min(gaps, axis=1) <= BEARING_TOL + EPS))
            and bool(np.all(gaps >= -BEARING_TOL - EPS))
            and bool(np.all(pts[:, :, 0] >= X_LIPH - EPS))
            and bool(np.all(pts[:, :, 0] <= X_STOP + EPS))
            and bool(np.all(np.abs(pts[:, :, 1]) <= Y_JAMB + EPS))
            and min(float(st.joints[j]) for j in SPRINGS) > EPS
            and float(spring_axial_resistance(st).min()) > EPS)



def seat_measures(st):
    """(worst back-corner distance from the stop's face, m; bottom corners' height range above the channel floor, m;
    tilt from upright, deg; worst side-edge overrun past a jamb face, m)."""
    back = [float(_local(st, c)[0] - X_STOP) for c in CORNERS]
    bottom = [float(_local(st, c)[2] - Z_SILL) for c in CORNERS if c[2] == 0.0]
    up = _rot(st.quat_xyzw)[:, 2]
    tl = math.degrees(math.acos(max(-1.0, min(1.0, float(up[2])))))
    side = max(abs(float(_local(st, (x, s * W / 2, z))[1])) - Y_JAMB for x in (-T / 2, T / 2) for s in (-1, 1)
               for z in (0.0, H))
    return max(abs(b) for b in back), (min(bottom), max(bottom)), tl, side


def faces_room(st) -> bool:
    """Mesh, spline and pull tabs toward the room (the screen not turned round)."""
    return float(_rot(st.quat_xyzw)[0, 0]) > 0.0


def seated(st) -> bool:
    flush, (blo, bhi), tl, side = seat_measures(st)
    return (flush <= FLUSH_TOL + EPS and blo >= -0.001 - EPS and bhi <= SILL_TOL + EPS and tl <= TILT_TOL + EPS
            and side <= 0.0005 + EPS and faces_room(st))


def top_rise(st) -> float:
    """How far the lowest of the four top corners stands above the head lip's underside (m; negative below it)."""
    return min(float(_local(st, c)[2]) for c in TOP_CORNERS) - Z_HL


def top_captured(st) -> bool:
    """The top edge is up in the head channel, behind the lip, at least 3 mm along its whole length."""
    _, _, tl, side = seat_measures(st)
    behind = min(float(_local(st, c)[0]) for c in TOP_CORNERS) >= X_LIPH - 0.0005 - EPS
    return (top_rise(st) >= TOP_IN - EPS and behind and tl <= TOP_TILT + EPS and side <= 0.0005 + EPS
            and faces_room(st))


def geometry_valid(st) -> bool:
    """Reject impossible end states that put a rail through the floor, stop or channel ceiling (0.5 mm contact
    allowance at the stop/ceiling; 1 mm at the floor, as in seated()). Crown penetration is rejected wherever it lies under the solid head."""
    top = np.array([_local(st, c) for c in TOP_CORNERS])
    back = np.array([_local(st, c) for c in CORNERS])
    crowns = spring_points(st)
    under_head = (crowns[:, :, 0] >= X_WALL) & (crowns[:, :, 0] <= X_STOP)
    if np.any(under_head & (crowns[:, :, 2] > Z_HC + BEARING_TOL + EPS)):
        return False
    return (bool(np.all(top[:, 2] <= Z_HC + 0.0005 + EPS))
            and bool(np.all(back[:, 0] <= X_STOP + 0.0005 + EPS))
            and bool(np.all(back[:, 2] >= Z_SILL - 0.001 - EPS)))


def tier(objects) -> float:
    """End-state tier before the release and stillness rules: 1.0 seated in both channels against the stop; 0.5 the
    top captured in the head channel but the screen not seated; 0 otherwise."""
    st = objects["screen"]
    validate_springs(st)
    if not geometry_valid(st):
        return 0.0
    if seated(st) and springs_bearing(st):
        return 1.0
    if top_captured(st):
        return 0.5
    return 0.0


# -- trial record ----------------------------------------------------------------------------------------------------
CLEAR = 0.005
STILL_M, HOLD_S = 0.001, 1.0
DROP_V, DROP_GAP = 0.5, 0.020  # falling faster than 0.5 m/s with every hand point 2 cm or more away: dropped
_rec = parts.__dict__.setdefault("insect_screen_record", {})


def reset_record() -> None:
    _rec.clear()
    _rec.update(steps=0, known=True, reason="", gap=None, since=None, held=False, snap=None, t=0.0, tier=0.0,
                dropped=False, last_z=None, last_t=None)


if not _rec:
    reset_record()


def _snap(objects) -> np.ndarray:
    st = objects["screen"]
    # Four back corners reconstructed from the four visible inner frame/mesh intersections
    # and the known rigid profile. Crown pad corners have no physical landmark counterparts.
    return np.array([_local(st, c) for c in CORNERS])


def _movement(a, b):
    return float(np.linalg.norm(a - b, axis=1).max())


def observe(objects, gap, t) -> None:
    """One control step. ``gap``: closest sampled hand point to the screen (m), or None when the hands could not be
    read, which ends all credit for the trial."""
    _rec["steps"] += 1
    try:
        st = objects["screen"]
        vals = [*st.pos, *st.quat_xyzw, *(st.joints[j] for j in SPRINGS)]
        if not all(math.isfinite(float(v)) for v in vals):
            raise ValueError("screen state unknown")
        if gap is None or not math.isfinite(gap):
            raise ValueError("hand state unknown")
        z = float(st.pos[2])
        if _rec["last_z"] is not None and t > _rec["last_t"]:
            vz = (z - _rec["last_z"]) / (t - _rec["last_t"])
            if vz < -DROP_V and gap >= DROP_GAP - EPS:
                _rec["dropped"] = True
        _rec["last_z"], _rec["last_t"] = z, float(t)
        tr = tier(objects)
        snap = _snap(objects)
    except (KeyError, ValueError, TypeError, AttributeError, IndexError) as e:
        _rec.update(known=False, reason=str(e), since=None, held=False)
        return
    _rec["t"] = float(t)
    _rec["gap"] = float(gap)
    if tr <= 0.0 or gap <= CLEAR + EPS:
        _rec.update(since=None, held=False, snap=None, tier=tr)
    elif _rec["since"] is None or tr != _rec["tier"] or _movement(snap, _rec["snap"]) > STILL_M + EPS:
        _rec.update(since=float(t), held=False, snap=snap, tier=tr)
    else:
        _rec["held"] = float(t) - _rec["since"] >= HOLD_S - EPS


def score(objects):
    """0 for the whole trial if the hands or the screen could not be read at any step, or if the screen was ever
    dropped. Otherwise, with every hand point more than 5 mm from the screen and all four frame back corners moving at most 1 mm for
    1 s: 1.0 seated in both channels against the stop, 0.5 only its top captured in the head channel, 0 otherwise."""
    try:
        if not _rec["known"] or _rec["dropped"] or _rec["steps"] == 0 or not _rec["held"]:
            return 0.0
        if _rec["gap"] is None or _rec["gap"] <= CLEAR + EPS:
            return 0.0
        tr = tier(objects)
        if tr != _rec["tier"]:
            return 0.0
        if _movement(_snap(objects), _rec["snap"]) > STILL_M + EPS:
            return 0.0
        return tr
    except (KeyError, ValueError, TypeError, AttributeError, IndexError):
        return 0.0


def success(objects):
    return score(objects) >= 1.0 - 1e-9


def distance(objects):
    try:
        st = objects["screen"]
        return float(np.linalg.norm(np.asarray(st.pos, float) - SEAT))
    except (KeyError, ValueError, TypeError, AttributeError):
        return 1.0


# -- hands: hull faces sampled every 3 mm ----------------------------------------------------------------------------
_HAND_LINKS = ("gripper", "tip_left", "tip_right")
_FACES: dict = {}


def _hull_face_points(hull, step=0.003) -> np.ndarray:
    pts = [hull.vertices]
    for n, d in zip(hull.planes[:, :3], hull.planes[:, 3]):
        on = hull.vertices[np.abs(hull.vertices @ n - d) < 1e-6]
        if len(on) < 3:
            continue
        c = on.mean(axis=0)
        e1 = on[0] - c
        if np.linalg.norm(e1) < 1e-9:
            continue
        e1 = e1 / np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        ring = on[np.argsort(np.arctan2((on - c) @ e2, (on - c) @ e1))]
        for i in range(len(ring)):
            a, b = ring[i], ring[(i + 1) % len(ring)]
            m = max(1, int(math.ceil(max(np.linalg.norm(a - c), np.linalg.norm(b - c), np.linalg.norm(a - b)) / step)))
            ii, jj = np.meshgrid(np.arange(m + 1), np.arange(m + 1), indexing="ij")
            keep = (ii + jj) <= m
            s_, t_ = ii[keep][:, None] / m, jj[keep][:, None] / m
            pts.append(c + (a - c) * s_ + (b - c) * t_)
    pts = np.concatenate(pts)
    _, idx = np.unique(np.round(pts / (step / 2)).astype(np.int64), axis=0, return_index=True)
    return np.concatenate([hull.vertices, pts[np.sort(idx)]])


def link_points(name) -> np.ndarray:
    if name not in _FACES:
        from yam_arena import analysis

        hs = analysis.link_hulls().get(name, [])
        if not hs:
            raise ValueError(f"no collision hull for {name}")
        _FACES[name] = np.concatenate([_hull_face_points(h) for h in hs])
    return _FACES[name]


def screen_boxes(st):
    poses = {n: (np.asarray(p, float), _rot(q)) for n, (p, q) in st.links.items()}
    poses["base"] = (np.asarray(st.pos, float), _rot(st.quat_xyzw))
    return [(c, r, s) for _, c, r, s in SCREEN.world_boxes(poses)]


def hand_gap(objects, pts) -> float:
    """Closest hand point to any part of the screen, tabs and spring crowns included (m; negative inside)."""
    from yam_arena.analysis import box_signed_distance

    st = objects["screen"]
    near = np.linalg.norm(pts - np.asarray(st.pos, float), axis=1) < 0.70
    pts = pts[near]
    if len(pts) == 0:
        return 1.0
    return min(float(box_signed_distance(pts, c, r, s).min()) for c, r, s in screen_boxes(st))


def _hand_points(sim):
    base = sim.env.unwrapped
    origin = parts._np(base.scene.env_origins[0])
    out, found = [], 0
    for arm in ("left_robot", "right_robot"):
        asset = base.scene[arm]
        poses = parts._np(asset.data.body_link_pose_w)[0]
        for i, n in enumerate(asset.body_names):
            if n in _HAND_LINKS:
                out.append(link_points(n) @ _rot(poses[i, 3:]).T + (poses[i, :3] - origin))
                found += 1
    if found != 2 * len(_HAND_LINKS):
        return None
    pts = np.concatenate(out)
    return pts if np.isfinite(pts).all() else None


# -- simulator hooks -------------------------------------------------------------------------------------------------
@parts.on_reset
def _new_trial(sim) -> None:
    # Arena initializes/reset buffers to zero; apply this constant declared spring rest target after reset.
    # This is not a state/score trigger and never writes an object pose or disables gravity.
    for joint in SPRINGS:
        sim.set_target("screen", joint, SPRING_TARGET)
    reset_record()
    _rec["hook_t"] = None


@parts.on_step
def _step(sim) -> None:
    if _rec.get("hook_t") == sim.t:
        return
    _rec["hook_t"] = sim.t
    try:
        objects = sim.states()
    except Exception:  # noqa: BLE001  unknown state: no credit
        objects = {}
    g = None
    try:
        pts = _hand_points(sim)
        g = hand_gap(objects, pts) if pts is not None else None
    except Exception:  # noqa: BLE001
        g = None
    observe(objects, g, float(sim.t))
    log = str(pathlib.Path(os.environ["TRACE_OUT"]).with_name("screen.jsonl")) if os.environ.get("TRACE_OUT") else None
    if log and _rec["steps"] % 5 == 0 and objects:
        try:
            st = objects["screen"]
            flush, (blo, bhi), tl, side = seat_measures(st)
            up = _rot(st.quat_xyzw)[:, 2]
            row = dict(t=round(float(sim.t), 3), lot=lots.current().name, score=score(objects), tier=tier(objects),
                       pos=np.round(np.asarray(st.pos), 4).tolist(), lean=round(math.degrees(math.atan2(up[0], up[2])), 2),
                       flush=round(flush, 4), bottom=[round(blo, 4), round(bhi, 4)], tilt=round(tl, 2),
                       side=round(side, 4), top=round(top_rise(st), 4),
                       springs=[round(float(st.joints[j]), 4) for j in SPRINGS],
                       crown_gap_mm=np.round(1000 * (Z_HC - spring_points(st)[:, :, 2]), 5).tolist(),
                       bearing=springs_bearing(st),
                       spring_N=[round(SPRING_K * max(0.0, float(st.joints[j])), 2) for j in SPRINGS],
                       spring_drive_N=[round(SPRING_K * float(st.joints[j]) + SPRING_DRIVE_OFFSET_N, 5)
                                       for j in SPRINGS],
                       spring_axial_static_N=np.round(spring_axial_resistance(st), 5).tolist(),
                       gap=_rec["gap"], held=_rec["held"], dropped=_rec["dropped"], known=_rec["known"],
                       reason=_rec["reason"])
            base = sim.env.unwrapped
            origin = parts._np(base.scene.env_origins[0])
            spring_asset = base.scene["screen"]
            spring_ids = spring_asset.find_joints(list(SPRINGS), preserve_order=True)[0]
            row["spring_targets_m"] = np.round(parts._np(spring_asset.data.joint_pos_target)[0, spring_ids], 9).tolist()
            for arm in ("left_robot", "right_robot"):
                a_ = base.scene[arm]
                poses = parts._np(a_.data.body_link_pose_w)[0]
                qj = parts._np(a_.data.joint_pos)[0]
                names = list(a_.joint_names)
                gi = list(a_.body_names).index("gripper")
                tcp = poses[gi, :3] - origin + _rot(poses[gi, 3:]) @ np.array([0.0, 0.0, 0.134])
                row[arm[0]] = dict(tcp=np.round(tcp, 4).tolist(),
                                   f=[round(float(qj[names.index(n)]), 4) for n in ("joint7", "joint8") if n in names])
            with open(log, "a") as f:
                f.write(json.dumps(row) + "\n")
        except Exception as e:  # noqa: BLE001
            with open(log, "a") as f:
                f.write(json.dumps({"err": str(e)}) + "\n")


# -- demo: open loop from the published layout --------------------------------------------------------------------
LEAN_IN = 46.0  # fingertip lean toward the screen; lowers the wrist/forearm clear of the head at the high lift
GRIP_DX, GRIP_DZ = -0.007, 0.000  # TCP from the tab's centre, screen frame (room side, up)
GRIP_YAW = {1: 0.0, -1: 0.0}  # grip turned about the vertical (rad), per side
GRIP_HEADING = {1: 0.0, -1: 0.0}  # direction the fingertips lean (rad, world; 0 = toward the window), per side
F_OPEN, F_PINCH, F_LET = -0.024, 0.0, -0.017
TILT_IN = 22.0  # degrees the screen leans back (top toward the window) to pass its top under the head lip. Kept: at
# 18-20 degrees the band of heights in which the top passes under the head lip while the bottom clears the stool
# shrinks from about 11.7 mm to 4-8 mm, less than the arms' measured 5-9 mm sag, and the lean only changes the
# screen's weight moment about the grips by about 5 % (0.103 -> 0.098 N m); the tab stagger is changed instead.
CEIL_MARGIN = 0.0005  # the planned path keeps the frame's top this far under the head channel's ceiling
# Diagnostic traces show loaded TCPs below/short of their commands. Offsets compensate at the hands, while
# screen_path itself keeps its top-back corner at the stop and below the ceiling; no object pose is written.
# With the deeper flat-tab grip and fully closed stock command, 13 mm final lift compensation leaves the bottom
# only 2.0-2.1 mm high at the swing (lot A). At 25 mm, with a 46-degree wrist for forearm clearance, lot A clears
# the 4 mm lip and seats. The toward-stop command eases to 1.5 mm as the top rises to limit sliding friction,
# then returns to 6.5 mm while lowering and 3 mm before release. These are controller offsets, not score tolerances.
# v28 lot B: low bottom corner 3.4 mm at swing, below the unchanged 4 mm lip.
# Test 30 mm lift compensation rather than 25 mm, retaining the 2 s two-arm high completion.
SAG = {"carry": np.array([0.0, 0.0, 0.004]), "reach": np.array([0.0075, 0.0, 0.004]),
       "push0": np.array([0.0075, 0.0, 0.006]), "push": np.array([0.0015, 0.0, 0.030]),
       "seat": np.array([0.0065, 0.0, 0.004]), "let": np.array([0.003, 0.0, 0.004])}
SLIP = math.radians(3.0)  # the screen turns this much further back in the pinches while leaned: in the first trace
# it leaned 3.8-6.7 degrees past the planned 22 by "top just short of the jambs" (lots A, B), and its top then met the
# glass and was turned back past upright in the grips; the hands are turned this much less over the approach
REACH_ARM = {1: np.zeros(3), -1: np.zeros(3)}  # extra aim toward the window per arm from "top in" on (from a trace)
PIVOT_X = X_STOP  # the top-back corner is held against the stop while the screen turns up
APPROACH_DZ = -0.022  # top-back corner below the seated top edge while the top goes in under the head lip (3 mm
# lower than planned before, so that with the hands turned SLIP less the springs' crowns still pass under the lip)


def lift_at(deg: float) -> float:
    """The highest the top-back corner may be planned above the seated top edge at lean ``deg`` with the frame's
    room-side top corner (T sin(lean) higher) still CEIL_MARGIN under the ceiling (m)."""
    return GAP_TOP - CEIL_MARGIN - T * math.sin(math.radians(deg))


# (lean, deg; height of the top-back corner above the seated top edge, m) while the top turns up into the channel,
# pivoting on its back corner against the stop, and pressing the springs up to 6.5 mm of their 7 mm. The top is kept
# low while the screen still leans far back, because the room-side face below the top then slopes out toward the head
# lip: planned this way the path still clears the lip with the top-back corner 3 mm short of the stop (plan_clearance
# with PIVOT_X moved), where the draft's 12.6 mm head channel and schedule already touched it at 0 mm.
RISE = ((22.0, -0.0100), (18.0, -0.0075), (14.0, -0.0045), (11.0, -0.0015), (8.0, 0.0015), (6.0, 0.0035),
        (4.0, lift_at(4.0)))
# leans (deg) while the bottom swings in over the sill lip, the top held as high as the ceiling allows: the bottom
# then stands 5.3-5.5 mm above the channel floor, 1.3 mm or more over the 4 mm lip
SWING = (3.0, 1.5, 0.0)
LIFT = lift_at(0.0)  # 5.5 mm: the top's height above its seat as the upright screen goes back against the stop


def origin_from_top_back(p, lean) -> np.ndarray:
    """The screen's origin when its top-back corner is at ``p`` (world), leaning ``lean`` rad."""
    return np.asarray(p, float) - _ry(lean) @ np.array([T / 2, 0.0, H])


def grip_pose(origin, lean, side, dx=0.0, dz=0.0, dy=0.0, slip=0.0):
    """TCP position and rotation for the pinch on one pull tab with the screen's origin at ``origin``, leaning ``lean``
    (rad, top toward the window). dx, dz: extra offset in the screen frame; dy: toward the side edge. ``slip`` (rad):
    the hand is turned this much less than the screen, for the screen turning that much in the pinch."""
    r = _ry(lean)
    p = np.asarray(origin, float) + r @ np.array([TAB_XC + GRIP_DX + dx, side * (TAB_Y + dy), TAB_ZS[side] + GRIP_DZ + dz])
    rot = _ry(lean - slip) @ tilt.rotation(GRIP_YAW[side], LEAN_IN, GRIP_HEADING[side])
    return p, rot


def screen_path(start):
    """The screen's poses (origin, lean rad, seconds to reach it, label) from the pinch to the seat."""
    y0 = float(start[1])  # the lot's sideways offset, taken out while backing away from the head
    seat_top = Z_SILL + H
    a_in = PIVOT_X
    tin = math.radians(TILT_IN)
    out = [(start + np.array([-0.010, 0.0, 0.020]), LEAN, 1.5, "lift it off the stool, still leaning"),
           (start + np.array([-0.060, -y0, 0.040]), LEAN, 1.5, "back from the head, centred on the opening")]
    # centred while still nearly upright: in lot B the sideways move while leaned far back let the screen turn
    # 29 degrees back in the pinches (lot A, with no sideways move, 23.7)
    out.append((origin_from_top_back((X_WALL - 0.060, 0.0, seat_top + APPROACH_DZ), tin), tin, 1.6,
                "lean it back further, top level with the gap under the head"))
    out.append((origin_from_top_back((a_in, 0.0, seat_top + APPROACH_DZ), tin), tin, 2.6,
                "top in under the head, back against the stop"))  # one move: each stop on the way turned the screen
    out.append((origin_from_top_back((a_in, 0.0, seat_top + APPROACH_DZ), tin), tin, 0.8, "settle"))
    push_from = len(out)
    for deg, dz in RISE:
        a = math.radians(deg)
        out.append((origin_from_top_back((a_in, 0.0, seat_top + dz), a), a, 0.9,
                    "turn it up into the head channel, pressing the springs"))
    for deg in SWING:
        a = math.radians(deg)
        out.append((origin_from_top_back((a_in, 0.0, seat_top + lift_at(deg)), a), a, 0.7,
                    "swing the bottom in over the sill lip"))
    out.append((origin_from_top_back((X_STOP, 0.0, seat_top + LIFT), 0.0), 0.0, 2.0, "both complete the high swing against the stop"))
    out.append((origin_from_top_back((X_STOP, 0.0, seat_top - 0.001), 0.0), 0.0, 1.2,
                "down into the sill channel"))
    phases = ["carry"] * (push_from - 2) + ["reach"] * 2 + ["push"] * (len(out) - push_from - 1) + ["seat"]
    return [(o, a, d, lab, ph) for (o, a, d, lab), ph in zip(out, phases)]


def _rest_pose(arm):
    from yam_arena import ik, station
    from yam_arena.demo import READY

    pos, rot = ik.tcp_pose(np.array(READY, float), station.BASES[arm])
    return tuple(float(v) for v in pos), tilt._TiltedYaw(0.0, rot)


def Wp(d, p, f, lab, rot):
    return Waypoint(float(d), _f(p), float(f), lab, tilt._TiltedYaw(0.0, rot))


def demo():
    lot = lots.current()
    start = np.asarray(lot.pos("screen"), float)
    up = np.array([0.0, 0.0, 1.0])
    path = screen_path(start)
    plans = {}
    for arm, side in (("left", 1), ("right", -1)):
        g0, r0 = grip_pose(start, LEAN, side, dx=-0.030)
        g1, _ = grip_pose(start, LEAN, side, dy=0.0)
        plan = [
            Wp(1.5, g0 + 0.06 * up, F_OPEN, "above the pull tab, in front of it", r0),
            Wp(1.2, g0, F_OPEN, "down beside the tab", r0),
            Wp(1.0, g1, F_OPEN, "fingers either side of it", r0),
            Wp(0.8, g1, F_PINCH, "pinch the tab", r0),
            Wp(0.4, g1, F_PINCH, "settle", r0),
        ]
        k = 0
        for i, (o, a, d, lab, ph) in enumerate(path):
            if ph == "push":  # from the start of the rise to the swing: ramp the offset up over the rise
                u = min(1.0, k / max(1, len(RISE) - 1))
                off = SAG["push0"] + u * (SAG["push"] - SAG["push0"])
                slip = SLIP * (1.0 - u)
                k += 1
            else:
                off = SAG[ph]
                slip = SLIP if i >= 2 else 0.0  # from "lean it back further" on
            if ph in ("reach", "push", "seat"):
                off = off + REACH_ARM[side]
            g, r = grip_pose(o, a, side, slip=slip)
            command = g + off
            finger = F_PINCH
            if arm == "left" and i == 4 + len(RISE):
                brace_command, brace_rotation = command.copy(), r.copy()
            if arm == "left" and 4 + len(RISE) < i < len(path) - 2:
                # Left holds through the right swing; both finish the high rotation before lowering.
                command, r, finger = brace_command, brace_rotation, F_PINCH
                lab = "left holds at raised position while right swings bottom"
            plan.append(Wp(d, command, finger, lab, r))
        g7, r7 = grip_pose(path[-1][0], 0.0, side)
        g7 = g7 + SAG["let"]
        g8, _ = grip_pose(SEAT, 0.0, side, dx=-0.040)
        plan += [
            Wp(0.6, g7, F_PINCH, "ease the press on the stop", r7),
            Wp(0.8, g7, F_LET, "let the tab go", r7),
            Wp(0.5, g7, F_LET, "settle", r7),
            Wp(1.2, g8, F_LET, "draw back off the tab", r7),
            Wp(1.2, g8 + 0.10 * up + np.array([-0.06, 0.0, 0.0]), F_LET, "up and away", r7),
        ]
        rest, ryaw = _rest_pose(arm)
        plan += [Waypoint(2.0, rest, F_LET, "back to the rest pose", ryaw),
                 Waypoint(4.0, rest, F_LET, "wait", ryaw)]
        plan[:0] = [Waypoint(2.0, rest, FINGER_OPEN, "to the rest pose", ryaw),
                    Waypoint(0.5, rest, FINGER_OPEN, "settle at the rest pose", ryaw),
                    Waypoint(1.0, _f(np.asarray(rest) + np.array([-0.100, 0.0, 0.0])), FINGER_OPEN,
                             "draw back clear of the screen before approaching", ryaw)]
        plans[arm] = plan
    return plans


def plan_clearance(start=None, step=0.0005):
    """Offline: the smallest gap (m) between the screen's frame and tabs and the window, stop excepted, along the
    demo's screen path, sampled finely between waypoints, with the pose where it occurs; and the most either spring is
    pressed on the way (m, from its crown's height under the head channel's ceiling; at most SPRING_CROWN, when it is
    flat). Negative gap means the path runs into the window. The springs' crowns are meant to touch the ceiling and are
    checked by the second number, not the first."""
    start = START_A if start is None else np.asarray(start, float)
    path = [(o, a) for o, a, _, _, _ in screen_path(start)]  # from the first lift (the start touches the head)
    boxes = [(np.asarray(lo, float), np.asarray(hi, float)) for lo, hi, _ in window_boxes()
             if abs(lo[0] - X_STOP) > 1e-9]  # the stop is touched on purpose while the screen turns up
    pts = []
    for sh in _screen_shapes():
        c, hf = np.asarray(sh.at), np.asarray(sh.size) / 2
        g = [np.arange(-h, h + 1e-9, max(step * 4, 2 * h / 6)) if h > 0.001 else np.array([-h, h]) for h in hf]
        X, Y, Z = np.meshgrid(*g, indexing="ij")
        q = np.stack([X.ravel(), Y.ravel(), Z.ravel()], 1)
        edge = (np.isclose(np.abs(q), hf, atol=1e-9)).any(axis=1)
        pts.append(c + q[edge])
    crowns = np.array([[x, s * SPRING_Y + d, H + SPRING_CROWN] for s in (-1, 1) for x in (-SPRING_WIDTH / 2,
                       SPRING_WIDTH / 2) for d in (-SPRING_PAD / 2, SPRING_PAD / 2)])
    pts = np.concatenate(pts)
    worst, pressed = (1.0, None), 0.0
    for (o0, a0), (o1, a1) in zip(path[:-2], path[1:-2]):  # the last two poses touch the stop and the floor
        n = max(2, int(np.linalg.norm(o1 - o0) / 0.002) + int(abs(a1 - a0) / math.radians(0.5)) + 1)
        for u in np.linspace(0.0, 1.0, n):
            o, a = o0 + u * (o1 - o0), a0 + u * (a1 - a0)
            r = _ry(a)
            w = pts @ r.T + o
            tips = crowns @ r.T + o
            inside = (tips[:, 0] >= X_LIPH) & (tips[:, 0] <= X_STOP)  # crown corners under the channel's ceiling
            if inside.any():
                pressed = max(pressed, float((tips[inside, 2] - Z_HC).max()))
            for lo, hi in boxes:
                d = np.maximum(lo - w, w - hi).max(axis=1)  # >0 outside (box metric, conservative inside)
                m = float(d.min())
                if m < worst[0]:
                    worst = (m, (np.round(o, 4).tolist(), round(math.degrees(a), 2), np.round(lo, 4).tolist()))
    return worst, pressed
