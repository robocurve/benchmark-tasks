"""On the station table: the drive-roller bay of a small roller conveyor deck, two dark anodised side rails 236 mm
apart (inside faces at y +-0.118) running along x from 0.215 to 0.385, each with an open-top drop-in slot at x 0.30
(13 mm wide, a V seat at the bottom for an 11 mm hex shaft). In front of the bay, on two grey V blocks at x 0.165, lies
a 24 V motorized drive roller (38 mm steel tube, 225 mm long, 11 mm hex stub at each end, 0.70 kg) along y, its hex
stubs flat-up. As in a real motorized roller, the shaft (both hex stubs and the motor and gearbox fixed on it) is one
body and the tube turns on it, against the gearbox's drag. Its black 6 mm power cable leaves the left (+y) stub along
the axis (it is welded to the shaft) and runs in a short loop to a
connector anchored at the driver card on the left rail's outer face. On the left rail's outer face, a guide pocket at
x 0.30 waits for the stainless anti-rotation torque plate (34 x 70 x 4 mm, hex notch at the bottom), which stands in a
slotted rack at (0.355, 0.215).

Requires -E object_jitter=0: fixed layout.

Run it:
    inspect-robots run --embodiment yam_arena -E task=ext_robot_amr_payload_hitch_conveyor_modules_23.py \\
        -E object_jitter=0 --instruction "Seat the roller's hex ends point-up in the frame slots without pulling
        its cable, then drop the torque plate over the hex." --policy yam_scripted --scorer task_score
"""

from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import belimits  # noqa: E402, F401
import closedloop  # noqa: E402, F401
import mechkit  # noqa: E402
import ropekit  # noqa: E402
from closedloop import grip_rot, placeholder_demo  # noqa: E402
from mechkit import Joint, Mech, box, cyl, extrude, tube  # noqa: E402
from ropekit import EndBody, Plastic, Rope, fillet  # noqa: E402

from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
from yam_arena.urdf import axis_angle_matrix  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 8.0
SCORE_OVER = "end"

# Offline testing override for the hands-clear check (stays None in live simulator runs).
FORCE_RELEASED: list = [None]

STEEL = (0.74, 0.76, 0.80)  # zinc-plated roller tube and shafts
STAINLESS = (0.90, 0.91, 0.93)  # SUS304 torque plate
FRAME = (0.16, 0.20, 0.30)  # dark blue anodised aluminium rails and guide
STAGE = (0.42, 0.43, 0.46)  # grey printed V blocks and plate rack
CABLE = (0.06, 0.06, 0.07)
CARD = (0.10, 0.45, 0.20)  # green driver card housing

# ---- conveyor frame (fixed) ---------------------------------------------------------------------------------------
FX = 0.30  # slot centre x
RAIL_IN, RAIL_T = 0.118, 0.006  # inside face |y| and rail thickness: between-frame width 236 mm
RAIL_OUT = RAIL_IN + RAIL_T
RAIL_X0, RAIL_X1 = 0.215, 0.385
RAIL_H = 0.075
SEAT_H = 0.050  # hex axis height above the table when seated
SEAT_Z = TOP + SEAT_H
HEX_AF = 0.011  # hex across flats
HEX_R = HEX_AF / math.sqrt(3.0)  # across corners / 2 = 6.35 mm
SLOT_HALF = 0.0065  # 13 mm slot: 1 mm per side on the hex flats
LEAD_HALF, LEAD_D = 0.011, 0.005  # entry chamfer at the rail top
TAN30 = math.tan(math.radians(30.0))

# ---- roller: shaft (rope end body) and tube (a separate free Mech turning on it) ------------------------------------
TUBE_R, TUBE_L = 0.019, 0.225  # 38 mm tube; 236 mm between frames less the 11 mm (5.5 mm a side) end gap
TUBE_WALL = 0.0012
STUB_L = 0.0275  # each hex stub, tube end to shaft end
ROLLER_L = TUBE_L + 2 * STUB_L  # 0.280 overall; shaft ends at y +-0.140 when centred
ROLLER_HALF = ROLLER_L / 2
SHAFT_R = 0.005  # round shaft between the stubs, inside the tube
KEY_L, KEY_W, KEY_H = 0.100, 0.024, 0.012  # flat key on the shaft: the motor and gearbox housing's flange
KEY_CLEAR = 0.0005  # per side, in the sleeve's slot: about +-2.4 deg of free play (gearbox backlash)
KEY_PLAY_DEG = 2.4  # free turn of the shaft in the sleeve either side of centre
KEY_AXIAL = 0.002  # axial play of the sleeve (and so the tube) on the key, each way
CAP_BORE = SHAFT_R + 0.0015  # tube end-cap bore round the shaft
SHAFT_MASS = 0.40  # shaft, motor and gearbox: everything fixed to the shaft
SLEEVE_MASS = 0.05  # gearbox output carrier the tube is driven by
TUBE_MASS = 0.25  # tube shell 245 g (1.2 mm wall) plus end caps
ROLLER_MASS = SHAFT_MASS + SLEEVE_MASS + TUBE_MASS
TUBE_DRAG = 0.10  # N m to turn the tube on its shaft (gearbox back-drive drag, design value)
TUBE_DAMP = 0.1  # N m s/rad viscous drag on the same joint (design value)

# ---- cradle and staging -------------------------------------------------------------------------------------------
X0 = 0.165  # roller axis x on the V blocks
V_APEX = 0.012  # V bottom above the table
Z0 = TOP + V_APEX + TUBE_R * math.sqrt(2.0) + 0.0003  # roller axis on two 90 deg V blocks, 0.3 mm above contact
VBLOCK_Y = 0.075
TIP0 = (X0, ROLLER_HALF, Z0)  # left shaft end, where the cable leaves the roller

# ---- torque plate ---------------------------------------------------------------------------------------------------
PL_T, PL_W = 0.004, 0.034
PL_BOT, PL_TOP = -0.010, 0.060  # plate edges relative to its origin (the hex axis point of its notch)
PL_LEAD_HALF, PL_LEAD_D = 0.009, 0.005  # notch mouth flared to 18 mm over its lowest 5 mm: a lead-in onto the hex
PL_GRIP = 0.042  # TCP height in the plate frame (18 mm below its top)
PL_MASS = 0.072  # SUS304, 7.93 g/cm3 x (34 x 70 - notch) x 4 mm, checked in check.py
Y_PLATE = RAIL_OUT + 0.003 + PL_T / 2  # pocket 10 mm deep for a 4 mm plate: 3 mm a side
POCKET_Y1 = RAIL_OUT + 0.010
LEDGE_TOP = TOP + 0.028  # guide floor: 12 mm under a seated plate's bottom
GUIDE_TOP = TOP + 0.070
RACK = (0.355, 0.215)
RACK_FLOOR = 0.004
PLATE0 = (RACK[0], RACK[1], TOP + RACK_FLOOR - PL_BOT)  # plate origin standing in the rack

# ---- cable -----------------------------------------------------------------------------------------------------------
CABLE_R = 0.003
ANCHOR = (0.235, 0.170)
ZC = TOP + CABLE_R + 0.0025  # cable axis lying on the table (the start connector is 10 mm tall)
# A limp PVC power lead: each 20 mm joint is elastic only to 3 deg (0.01 N m/rad), then keeps the bend it is given, so
# the lead stays where the roller's move leaves it instead of springing back to its laid shape and shoving the shaft
# along its axis or turning it back after clocking.
CABLE_SET = Plastic(yield_angle=math.radians(3.0), stiffness=0.01, damping=0.001, segment=0.02,
                    label="limp 6 mm PVC power lead")


def _ry(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), float(a))


def _rx(a: float) -> np.ndarray:
    return axis_angle_matrix(np.array([1.0, 0.0, 0.0]), float(a))


def _rail(side: float) -> mechkit.Shape:
    """A side rail with its drop-in hex slot: an x-z outline extruded along y (outline points are (z, x))."""
    a = SEAT_H - HEX_R  # V apex: the hex's lower corner when seated
    v = SEAT_H - HEX_R + SLOT_HALF * TAN30  # V meets the slot walls
    hx = (RAIL_X1 - RAIL_X0) / 2
    xz = [(-hx, 0.0), (hx, 0.0), (hx, RAIL_H), (LEAD_HALF, RAIL_H), (SLOT_HALF, RAIL_H - LEAD_D), (SLOT_HALF, v),
          (0.0, a), (-SLOT_HALF, v), (-SLOT_HALF, RAIL_H - LEAD_D), (-LEAD_HALF, RAIL_H), (-hx, RAIL_H)]
    cx = (RAIL_X0 + RAIL_X1) / 2
    return extrude((cx, side * (RAIL_IN + RAIL_T / 2), 0.0), [(z, x + cx - FX) for x, z in xz], RAIL_T, FRAME,
                   axis="y")


def _vblock(y: float) -> list:
    """A 90 deg V block (60 x 16 x 28 mm, V bottom 12 mm up) built from native boxes: one 6 mm slab under each V face,
    two outer pillars and a base. Native box colliders, so the tube (a hollow mesh) rests on plain box faces."""
    h, hx, t, d = 0.028, 0.030, 0.006, 0.016
    top_half = h - V_APEX
    L = top_half * math.sqrt(2.0) + 0.003  # face length, run 3 mm past the apex into the opposite slab
    s = [box((X0, y, 0.004), (2 * hx, d, 0.008), STAGE)]
    for sx in (-1.0, 1.0):
        s.append(box((X0 + sx * (top_half + hx) / 2, y, h / 2), (hx - top_half, d, h), STAGE))
        n = np.array([-sx, 1.0]) / math.sqrt(2.0)  # V face normal (x, z), up into the V
        mid = np.array([sx * top_half / 2, (V_APEX + h) / 2]) - np.array([sx, 1.0]) / math.sqrt(2.0) * 0.0015
        c = mid - n * t / 2
        q = (math.cos(-sx * math.pi / 8), 0.0, math.sin(-sx * math.pi / 8), 0.0)  # -sx * 45 deg about y
        s.append(box((X0 + float(c[0]), y, float(c[1])), (L, d, t), STAGE, tilt=q))
    return s


def _frame_shapes() -> list:
    s = [_rail(+1.0), _rail(-1.0)]
    # Cross ties front and back, below the roller
    for x in (RAIL_X0 + 0.006, RAIL_X1 - 0.006):
        s.append(box((x, 0.0, 0.010), (0.012, 2 * RAIL_IN, 0.020), FRAME))
    # Torque plate guide on the left rail's outer face: two posts with front lips, and a floor ledge
    gy0, gy1 = RAIL_OUT, POCKET_Y1 + 0.004
    zc, zh = (LEDGE_TOP - TOP + GUIDE_TOP - TOP) / 2, GUIDE_TOP - LEDGE_TOP
    for sx in (-1.0, 1.0):
        s.append(box((FX + sx * 0.024, (gy0 + gy1) / 2, zc), (0.008, gy1 - gy0, zh), FRAME))
        s.append(box((FX + sx * 0.017, POCKET_Y1 + 0.002, zc), (0.006, 0.004, zh), FRAME))
    s.append(box((FX, (gy0 + gy1) / 2, (LEDGE_TOP - TOP) - 0.004), (0.056, gy1 - gy0, 0.008), FRAME))
    return s


def _staging_shapes() -> list:
    s = [*_vblock(+VBLOCK_Y), *_vblock(-VBLOCK_Y)]
    rx, ry = RACK
    s.append(box((rx, ry, RACK_FLOOR / 2), (0.052, 0.030, RACK_FLOOR), STAGE))
    for sy in (-1.0, 1.0):  # side walls 1 mm off each plate face, 30 mm tall
        s.append(box((rx, ry + sy * (PL_T / 2 + 0.001 + 0.003), 0.015), (0.052, 0.006, 0.030), STAGE))
    for sx in (-1.0, 1.0):  # end walls 3 mm off each plate edge
        s.append(box((rx + sx * (PL_W / 2 + 0.003 + 0.003), ry, 0.015), (0.006, 2 * (PL_T / 2 + 0.001), 0.030), STAGE))
    return s


def _cable_path(end_y: float = TIP0[1]):
    ax, ay = ANCHOR
    pts = [(ax, ay, ZC), (ax, 0.215, ZC), (X0, 0.215, ZC), (X0, 0.166, Z0), (X0, end_y, Z0)]
    return fillet(pts, 0.02)


def _shaft_body() -> EndBody:
    """The roller's shaft, everything fixed to it, and the cable's end body. In the end frame: origin at the left shaft
    end, +x along the roller (world -y), z up. The hex prisms put a corner along the frame's y (world x), so the stubs
    start flat-up, 30 deg from point-up. Between the stubs a 10 mm round shaft runs through the tube, carrying at its
    middle a flat key (the motor and gearbox housing's flange) that the tube's drive sleeve rides on."""
    rod_l = ROLLER_HALF - KEY_L / 2 - STUB_L  # round shaft from each stub to the key
    shapes = [
        cyl((STUB_L / 2, 0.0, 0.0), HEX_R, STUB_L, STEEL, axis="x", sides=6),
        cyl((STUB_L + rod_l / 2, 0.0, 0.0), SHAFT_R, rod_l, STEEL, axis="x", sides=12),
        box((ROLLER_HALF, 0.0, 0.0), (KEY_L, KEY_W, KEY_H), STEEL),
        cyl((ROLLER_L - STUB_L - rod_l / 2, 0.0, 0.0), SHAFT_R, rod_l, STEEL, axis="x", sides=12),
        cyl((ROLLER_L - STUB_L / 2, 0.0, 0.0), HEX_R, STUB_L, STEEL, axis="x", sides=6),
    ]
    return EndBody(shapes, mass=SHAFT_MASS, friction=1.0)


def _sleeve_shapes() -> list:
    """The tube's drive sleeve round the shaft's key (Mech frame at the roller centre, axis along y): a slot 1 mm a
    side over the key, closed at both ends by plates whose 12 mm window passes the round shaft but not the key, so the
    sleeve (and the tube on it) stays on the shaft with 2 mm of axial play and turns with the shaft within +-5 deg."""
    sw, sh, t = KEY_W / 2 + KEY_CLEAR, KEY_H / 2 + KEY_CLEAR, 0.002
    ln = KEY_L + 2 * KEY_AXIAL
    s = [box((0.0, 0.0, sz * (sh + t / 2)), (2 * (sw + t), ln, t), STEEL) for sz in (-1.0, 1.0)]
    s += [box((sx * (sw + t / 2), 0.0, 0.0), (t, ln, 2 * sh), STEEL) for sx in (-1.0, 1.0)]
    win, pt, half = SHAFT_R + 0.001, 0.003, 0.0123  # plates stay inside the tube bore (corners at 17.4 mm)
    for sy in (-1.0, 1.0):
        y = sy * (ln / 2 + pt / 2)
        s += [box((0.0, y, sz * (win + (half - win) / 2)), (2 * half, pt, half - win), STEEL) for sz in (-1.0, 1.0)]
        s += [box((sx * (win + (half - win) / 2), y, 0.0), (half - win, pt, 2 * win), STEEL) for sx in (-1.0, 1.0)]
    return s


def _tube_shapes() -> list:
    """The steel tube shell and its two end caps, on the tube hinge (frame at the roller centre)."""
    s = [tube((0.0, 0.0, 0.0), TUBE_R - TUBE_WALL, TUBE_R, TUBE_L, STEEL, axis="y")]
    for sy in (-1.0, 1.0):
        s.append(tube((0.0, sy * (TUBE_L / 2 - 0.0035), 0.0), CAP_BORE, TUBE_R - TUBE_WALL, 0.005, STEEL, axis="y"))
    return s


def _connector() -> EndBody:
    return EndBody((box((0.011, 0.0, 0.0), (0.022, 0.014, 0.010), CABLE),), mass=0.012)


def _build_cable() -> Rope:
    """The equal-chord segments cut the path's corners, so the bare tip overshoots the path end: move the path end
    until the tip (where the roller's left shaft end is welded) sits at y = +0.140, the roller centred on y = 0."""
    end_y = TIP0[1]
    for _ in range(6):
        lay = ropekit.segment_layout(_cable_path(end_y), 0.02)
        err = float(lay.knots[-1][1]) - TIP0[1]
        if abs(err) < 2e-5:
            break
        end_y -= err
    return Rope("cable", path=_cable_path(end_y), segment=0.02, radius=CABLE_R, rope_mass=0.012, max_bend=1.2,
                plastic=CABLE_SET, color=CABLE, collision_sides=8, anchor_start=True, start_body=_connector(),
                end_body=_shaft_body())


def _plate_shapes() -> list:
    flank = HEX_R - SLOT_HALF * TAN30  # where the notch flanks meet its walls (1 mm off the hex side flats)
    xz = [(-PL_W / 2, PL_BOT), (-PL_LEAD_HALF, PL_BOT), (-SLOT_HALF, PL_BOT + PL_LEAD_D), (-SLOT_HALF, flank),
          (0.0, HEX_R), (SLOT_HALF, flank), (SLOT_HALF, PL_BOT + PL_LEAD_D), (PL_LEAD_HALF, PL_BOT), (PL_W / 2, PL_BOT),
          (PL_W / 2, PL_TOP), (-PL_W / 2, PL_TOP)]
    return [extrude((0.0, 0.0, 0.0), [(z, x) for x, z in xz], PL_T, STAINLESS, axis="y")]


CABLE_ROPE = _build_cable()
CABLE_LEN = CABLE_ROPE.total_length
FRAME_MECH = Mech("conveyor_frame", pos=(0.0, 0.0, TOP), mass=None, shapes=_frame_shapes())
STAGING = Mech("staging", pos=(0.0, 0.0, TOP), mass=None, shapes=_staging_shapes())
CARD_BOX = Mech("driver_card", pos=(0.0, 0.0, TOP), mass=None,
                shapes=[box((ANCHOR[0], (RAIL_OUT + 0.146) / 2, 0.016), (0.060, 0.146 - RAIL_OUT, 0.032), CARD)])
ROLLER_TUBE = Mech("roller_tube", pos=(X0, 0.0, Z0), mass=SLEEVE_MASS, shapes=_sleeve_shapes(), friction=1.0,
                   joints=[Joint("tube", "hinge", tuple(_tube_shapes()), axis="y", low=-3.0, high=3.0,
                                 damping=TUBE_DAMP, mass=TUBE_MASS, hold=TUBE_DRAG, friction=1.0)])
PLATE = Mech("torque_plate", pos=PLATE0, mass=PL_MASS, shapes=_plate_shapes(), friction=0.8)
OBJECTS = [FRAME_MECH, STAGING, CARD_BOX, CABLE_ROPE, ROLLER_TUBE, PLATE]

# ---- scoring --------------------------------------------------------------------------------------------------------
SEAT_TOL_Z = 0.0015  # both stub axes within 1.5 mm of the seat height
SEAT_TOL_X = 0.002  # and within 2 mm of the slot centre line
STUB_ON_RAIL = 0.002  # each hex stub covers at least 2 mm of its rail's 6 mm thickness
CLOCK_TOL_DEG = 5.0  # a hex corner within 5 deg of straight up
PLATE_TOL_Z = 0.0015
PLATE_TOL_X = 0.003
TUG_RATIO = 0.95  # cable straight-line span over 95 % of its length counts as pulled
TUG_STEPS = 5  # held for 5 control steps (0.25 s at 20 Hz)
CATCH_Z = TOP + 0.11  # any cable segment between the rails or in the plate pocket below this is caught


def _quat_matrix(q_xyzw) -> np.ndarray:
    return mechkit._quat_xyzw_matrix(np.asarray(q_xyzw, dtype=float))


def roller_pose(objects):
    """(rotation, left shaft-end point) of the roller's shaft (the cable's end body), never the tube."""
    o, q = objects["cable"].end_poses["end"]
    return _quat_matrix(q), np.asarray(o, dtype=float)


def stub_axes(objects):
    R, o = roller_pose(objects)
    a = R[:, 0]
    return o + a * (STUB_L / 2), o + a * (ROLLER_L - STUB_L / 2)


def clock_error_deg(objects) -> float:
    """Degrees from the nearest hex corner to straight up, measured about the roller's own axis (0 is point-up)."""
    R, _ = roller_pose(objects)
    return clock_error_from_R(R)


def clock_error_from_R(R) -> float:
    """clock_error_deg for a shaft rotation (frame x along the shaft, frame y along a hex corner)."""
    a, c = R[:, 0], R[:, 1]  # roller axis and the direction of one hex corner
    u = np.array([0.0, 0.0, 1.0]) - a[2] * a
    if np.linalg.norm(u) < 1e-6:
        return 90.0
    u /= np.linalg.norm(u)
    phi = math.degrees(math.atan2(float(np.cross(u, c) @ a), float(u @ c)))
    return abs(((phi + 30.0) % 60.0) - 30.0)


def stub_spans(objects):
    """(inner, outer) |y| of each hex stub along the roller axis: the left stub (from the cable end) first."""
    R, o = roller_pose(objects)
    a = R[:, 0]
    ys = [(float((o + a * u)[1]), float((o + a * v)[1])) for u, v in ((STUB_L, 0.0), (ROLLER_L - STUB_L, ROLLER_L))]
    return ys


def roller_seated(objects) -> bool:
    R, _ = roller_pose(objects)
    if abs(float(R[2, 0])) > math.sin(math.radians(3.0)) or abs(float(R[1, 0])) < math.cos(math.radians(3.0)):
        return False  # axis must lie along y within 3 deg
    left, right = stub_axes(objects)
    spans = stub_spans(objects)
    if left[1] < right[1]:
        left, right = right, left
        spans = spans[::-1]
    ok = True
    for p, (y_in, y_out), side in ((left, spans[0], 1.0), (right, spans[1], -1.0)):
        ok &= abs(p[0] - FX) <= SEAT_TOL_X and abs(p[2] - SEAT_Z) <= SEAT_TOL_Z
        # the hex stub lies across its own rail: at least 2 mm of the 6 mm rail under the hex (a seated shaft can sit
        # up to 7.5 mm off centre: the tube ends 5.5 mm off the rails plus the sleeve's 2 mm axial play)
        overlap = min(side * y_out, RAIL_OUT) - max(side * y_in, RAIL_IN)
        ok &= overlap >= STUB_ON_RAIL
    return bool(ok)


def plate_seated(objects) -> bool:
    s = objects["torque_plate"]
    R = _quat_matrix(s.quat_xyzw)
    o = np.asarray(getattr(s, "origin", s.pos), dtype=float)
    upright = float(R[2, 2]) >= math.cos(math.radians(10.0))
    return bool(upright and abs(o[0] - FX) <= PLATE_TOL_X and RAIL_OUT < o[1] < POCKET_Y1
                and abs(o[2] - SEAT_Z) <= PLATE_TOL_Z)


def cable_span(objects) -> float:
    _, o = roller_pose(objects)
    s = objects["cable"]
    a, _ = s.end_poses["start"]
    return float(np.linalg.norm(o - np.asarray(a, dtype=float)))


def cable_caught(objects) -> bool:
    """A cable segment between the rails, in a slot or in the torque plate's pocket (under the plate's path).

    The cable's attachment point on the shaft end (within 5 mm of it) is skipped: it is fixed to the shaft, so a roller
    seated up to 5.5 mm toward -y (which the tube ends allow) must not count its own attachment as a caught cable."""
    _, o = roller_pose(objects)
    for p in np.asarray(objects["cable"].points, dtype=float):
        if np.linalg.norm(p - o) < 0.005:
            continue
        if RAIL_X0 - 0.01 < p[0] < RAIL_X1 + 0.01 and p[1] < POCKET_Y1 and p[2] < CATCH_Z:
            return True
    return False


_TUG = {"streak": 0, "tugged": False, "last": None}


@mechkit.on_reset
def _clear_tug(env=None):
    _TUG.update(streak=0, tugged=False, last=None)


def _track_tug(objects) -> bool:
    """Latch a pull on the cable: its straight span over 95 % of its length for 5 control steps in a row. score() and
    success() both see each step's states, so a step is counted once."""
    _, o = roller_pose(objects)
    key = (id(objects), tuple(np.round(o, 7)))
    if key != _TUG["last"]:
        _TUG["last"] = key
        if cable_span(objects) > TUG_RATIO * CABLE_LEN:
            _TUG["streak"] += 1
            if _TUG["streak"] >= TUG_STEPS:
                _TUG["tugged"] = True
        else:
            _TUG["streak"] = 0
    return _TUG["tugged"]


def hands_clear(objects) -> bool:
    if FORCE_RELEASED[0] is not None:
        return bool(FORCE_RELEASED[0])
    return bool(mechkit.released(objects["torque_plate"], clearance=0.03)
                and mechkit.released(objects["cable"], clearance=0.03)
                and mechkit.released(objects["roller_tube"], clearance=0.03))


def evaluate(objects) -> dict:
    tugged = _track_tug(objects)
    caught = cable_caught(objects)
    seated = roller_seated(objects)
    clocked = seated and clock_error_deg(objects) <= CLOCK_TOL_DEG
    plate = clocked and plate_seated(objects)
    clear = plate and hands_clear(objects)
    if tugged or caught:
        sc = 0.0
    else:
        sc = 0.25 * seated + 0.25 * clocked + 0.25 * plate + 0.25 * clear
    return {"score": sc, "tugged": tugged, "caught": caught, "seated": seated, "clocked": clocked,
            "plate": plate, "clear": clear}


def score(objects) -> float:
    """0 if the cable was ever pulled straight (span over 95 % of its length for 0.25 s) or ends caught between the
    rails, in a slot or in the plate pocket. Otherwise 0.25 roller seated in both slots, +0.25 hex point-up within
    5 deg, +0.25 torque plate seated over the hex, +0.25 both hands 3 cm clear."""
    return float(evaluate(objects)["score"])


def success(objects) -> bool:
    return score(objects) == 1.0 and objects["torque_plate"].resting() and objects["cable"].resting()


# ---- arm keep-out (checked offline by sweep.py) ---------------------------------------------------------------------
# The driver card, its connector and the cable's run beside the left rail: no arm part goes down onto them.
CABLE_KEEP_OUT = {"x": (0.195, 0.275), "y": (0.142, 0.245)}
ARM_KEEP_OUT = [
    {"name": "driver card and cable run", "x": CABLE_KEEP_OUT["x"], "y": CABLE_KEEP_OUT["y"], "z_top": TOP + 0.08},
]

# ---- scripted demo ----------------------------------------------------------------------------------------------------
# The right hand turns the shaft by its right hex stub while the tube lies on its V blocks, whose friction (about
# 0.18 N m) holds the tube against the 0.10 N m drag, so the shaft turns inside the tube. The left hand then carries
# the roller by its tube, centres the shaft between the rails on its live pose, seats it and fits the plate.
YAW_X = -math.pi / 2  # pads close along world x, across the roller lying along y
R_YAW_X = math.pi / 2  # right hand: pads along x (yaw -90 misses IK by up to 12 mm here)
CLOCK_TURN = math.radians(-30.0)  # pads across two side flats of the flat-up hex
CARRY_Z = TOP + 0.125  # roller axis while carried: tube bottom 3 cm over the rail tops
G_LEVEL = grip_rot(yaw=YAW_X)
G_PLATE = grip_rot()  # pads close along y on the plate's two 34 x 70 mm faces (its 4 mm edges let it squirt out)
G_R_LEVEL = grip_rot(yaw=R_YAW_X)
G_R_TURNED = _ry(CLOCK_TURN) @ G_R_LEVEL
GRASP_OPEN = 0.65 * FINGER_OPEN  # about 61 mm over the 38 mm tube
STUB_OPEN = 0.40 * FINGER_OPEN  # about 35 mm over the 11 mm hex (12.7 mm across corners)
STUB_Y = -(ROLLER_HALF - 0.006)  # right pinch centre: pads (+-7.8 mm) 14 mm on the hex; 28 mm finger body 7.5 mm off the tube end

demo = placeholder_demo


def _trim_pose(arm, target, rot_goal, steps: int = 20, gain: float = 0.5):
    """Servo the measured tool pose (encoder forward kinematics) onto ``target`` and ``rot_goal``: cancels the wrist
    sag a 0.7 kg roller puts on the arm, including its roll about the roller axis. No-op offline."""
    target = np.asarray(target, dtype=float)
    for _ in range(steps):
        dp = target - arm.measured_pos
        R_err = rot_goal @ arm.measured_rot.T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1.0) / 2.0)))
        if np.linalg.norm(dp) < 0.0015 and ang < math.radians(0.7):
            break
        cmd_pos = arm.pos + gain * np.clip(dp, -0.004, 0.004)
        w = np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])
        if np.linalg.norm(w) > 1e-9 and ang > 1e-6:
            dR = axis_angle_matrix(w / np.linalg.norm(w), min(gain * ang, math.radians(1.5)))
            cmd_rot = dR @ arm.rot
        else:
            cmd_rot = arm.rot
        arm._set_cartesian(cmd_pos, cmd_rot, arm.finger)
        yield


def _offset(arm, nominal):
    """Commanded minus nominal position after a trim, carried into the next moves."""
    return np.asarray(arm.pos, dtype=float) - np.asarray(nominal, dtype=float)


def _shaft_live(ctx):
    """(rotation, left shaft-end point) of the shaft from the simulator (the cable's end pose), or None offline, where
    dry_run hands the cable over as a plain ObjectState with no end poses."""
    try:
        o, q = ctx.obj("cable").end_poses["end"]
        x, y, z, w = (float(v) for v in q)
        Rm = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return Rm, np.asarray(o, dtype=float)
    except Exception:  # noqa: BLE001
        return None


def _tube_live(ctx):
    """Centre of the tube (its Mech position) from the simulator, or the laid centre if the read fails."""
    try:
        p = np.asarray(ctx.obj("roller_tube").pos, dtype=float)
        if p.shape == (3,) and np.all(np.isfinite(p)):
            return p
    except Exception:  # noqa: BLE001
        pass
    return np.array([X0, 0.0, Z0])


def corner_alpha(R) -> float:
    """Signed angle (rad, about world +y) of a hex corner from straight up, picked in [-45, +15) deg so a flat-up hex
    gives -30 deg. Pads closing along x with the wrist turned by this angle sit across two side flats."""
    a, c = R[:, 0], R[:, 1]
    best = None
    for k in range(6):
        ck = axis_angle_matrix(a, k * math.pi / 3.0) @ c
        al = math.atan2(float(ck[0]), float(ck[2]))
        if -math.radians(45.0) <= al < math.radians(15.0) and (best is None or abs(al + math.radians(15.0)) <
                                                               abs(best + math.radians(15.0))):
            best = al
    return float(best if best is not None else CLOCK_TURN)


def _clock_shaft(R, ctx):
    """Right hand: pinch the right hex stub across two side flats, the wrist turned about the roller axis by the hex's
    live corner angle, then turn the wrist about the stub's own axis so a corner points up. The shaft turns inside
    the tube, which its V blocks hold still; the shaft's key drags the tube's sleeve round on the tube's hinge against
    its 0.10 N m drag, so the tube and shaft turn relative to each other. When the
    pinch opens, the shaft settles back onto the sleeve through the key's free play (about 2.4 deg), so each turn goes
    past point-up by a spring-back allowance that starts at the key play and is corrected from the angle read after
    the stroke. Up to 4 strokes, until a corner is within 1 deg of up. Offline: one nominal 30 deg stroke."""
    nominal_R, nominal_o = CABLE_ROPE.end_frame("end")
    live = _shaft_live(ctx)
    back = math.radians(KEY_PLAY_DEG)  # spring-back allowance, learned per stroke
    prev_alpha = None
    for stroke in range(4):
        Rm, o = live if live is not None else (np.asarray(nominal_R), np.asarray(nominal_o))
        alpha = corner_alpha(Rm) if live is not None else CLOCK_TURN
        if stroke > 0 and abs(alpha) < math.radians(1.0):
            break
        if prev_alpha is not None:  # residual on the same side as the last start: it fell short, so allow more
            same = (alpha < 0) == (prev_alpha < 0)
            back = min(math.radians(8.0), max(0.0, back + (0.8 if same else -0.8) * abs(alpha)))
        prev_alpha = alpha
        over = -math.copysign(back, alpha) if live is not None else 0.0
        grasp = o + Rm[:, 0] * (ROLLER_L - 0.006)  # right pinch centre: pads 14 mm on the hex, body 7.5 mm off the tube
        g_turned = _ry(alpha) @ G_R_LEVEL
        approach = g_turned[:, 2]
        for attempt in range(2):
            pre = grasp - 0.08 * approach
            yield from R.move(pre, 2.0 if attempt == 0 and stroke == 0 else 1.0, STUB_OPEN, rot=g_turned)
            yield from R.trim_to_measured(pre)
            off = _offset(R, pre)
            yield from R.move(grasp + off, 1.2, rot=g_turned)
            yield from R.trim_to_measured(grasp, max_duration=0.6)  # cancel lag before closing on the 11 mm hex
            off = _offset(R, grasp)
            yield from R.grip(FINGER_CLOSED, 0.8)
            yield from R.hold(0.4)
            # Isaac's pads read about 4 mm under the part, so an 11 mm hex reads near 7 mm: a 3.5 mm floor still tells
            # a hex from fingers closed on nothing. The second close goes ahead whatever it reads.
            if attempt == 1 or R.holding(0.0035):
                break
            yield from R.grip(STUB_OPEN, 0.5)
            yield from R.move(pre + off, 0.8, rot=g_turned)
        turn_s = 1.0 + 3.0 * abs(alpha - over) / math.radians(30.0)
        g_end = _ry(over) @ G_R_LEVEL
        yield from R.move(grasp + off, turn_s, rot=g_end)  # turn the hex about its own axis to point-up, plus allowance
        yield from R.hold(0.5)
        yield from R.grip(STUB_OPEN, 0.8)
        yield from R.hold(0.5)
        yield from R.move(grasp + off + np.array([0.0, 0.0, 0.09]), 1.2, rot=g_end)
        yield from R.hold(0.3)
        live = _shaft_live(ctx)
        if live is None:
            break
        print(f"RCDIAG clock stroke {stroke}: started {math.degrees(alpha):.1f} deg, allowance "
              f"{math.degrees(over):.1f} deg, corner {math.degrees(corner_alpha(live[0])):.1f} deg after the turn",
              flush=True)
    yield from R.home(2.5)
    return True


SLOT_PINCH_Y = -0.1375  # right pad centre in the slot: pads 10 mm on the hex past the rail's outer face
SLOT_LEAN = math.radians(25.0)  # hand leaned outboard about x: the fingers clear the rail top (25 mm above the axis)
SLOT_OPEN = 0.30 * FINGER_OPEN  # about 28 mm over the hex's 12.7 mm across corners: open fingers stay off the rail top
SLOT_CLOCK_TOL = math.radians(1.0)  # clock in the slot until a corner reads within 1 deg of up (scorer allows 5)


def _clock_in_slot(R, ctx):
    """Right hand, after the roller is seated and the left hand has let go: read the shaft's angle; if a hex corner is
    not within 1 deg of up, pinch the right hex stub across two side flats where it stands out past its rail (pads
    centred 13.5 mm outboard of the rail, the hand leaned 25 deg outboard about x so the fingers clear the rail top; a
    lean about x keeps the pads flush on the side flats), turn the wrist about the stub's own axis
    so a corner points up, open and re-read. The shaft is turned directly: the tube, free between the rails, follows it
    or not through its 0.10 N m drag. The 120 deg V seat takes the point-down lower corner and holds it once there.
    Before this step the cable (it cannot twist) and the tube's drag can turn the shaft back while the left hand carries
    the roller by its tube, which is what left the hex unclocked in Isaac. Up to 4 strokes. Offline: one nominal
    30 deg stroke at the seat, for the IK check."""
    live = _shaft_live(ctx)
    if live is not None:
        print(f"RCDIAG slot clock start: corner {math.degrees(corner_alpha(live[0])):.1f} deg", flush=True)
        if abs(corner_alpha(live[0])) < SLOT_CLOCK_TOL:
            return True
    nominal_R = np.asarray(CABLE_ROPE.end_frame("end")[0])
    nominal_o = np.array([FX, ROLLER_HALF, SEAT_Z])
    back = 0.0  # spring-back allowance, learned per stroke (the V seat holds a point-down hex, so it starts at 0)
    prev_alpha = None
    for stroke in range(4):
        Rm, o = live if live is not None else (nominal_R, nominal_o)
        alpha = corner_alpha(Rm) if live is not None else CLOCK_TURN
        if stroke > 0 and abs(alpha) < SLOT_CLOCK_TOL:
            break
        if prev_alpha is not None:
            same = (alpha < 0) == (prev_alpha < 0)
            back = min(math.radians(6.0), max(0.0, back + (0.8 if same else -0.8) * abs(alpha)))
        prev_alpha = alpha
        over = -math.copysign(back, alpha) if live is not None else 0.0
        a = Rm[:, 0]
        end_y = float((o + a * ROLLER_L)[1])
        y_c = min(SLOT_PINCH_Y, end_y + 0.006)
        s = (y_c - float(o[1])) / float(a[1]) if abs(float(a[1])) > 0.5 else ROLLER_L - 0.006
        grasp = o + a * s
        g_lean = _rx(SLOT_LEAN) @ G_R_LEVEL
        g_turned = _ry(alpha) @ g_lean
        approach = g_turned[:, 2]
        pre = grasp - 0.07 * approach
        yield from R.move(pre, 2.2 if stroke == 0 else 1.0, SLOT_OPEN, rot=g_turned)
        yield from R.trim_to_measured(pre)
        off = _offset(R, pre)
        yield from R.move(grasp + off, 1.2, rot=g_turned)
        yield from R.trim_to_measured(grasp, max_duration=0.6)
        off = _offset(R, grasp)
        yield from R.grip(FINGER_CLOSED, 0.8)
        yield from R.hold(0.4)
        turn_to = np.array([grasp[0], grasp[1], SEAT_Z]) + off  # a perched flat-down hex drops 1 mm into the V
        turn_s = 1.0 + 3.0 * abs(alpha - over) / math.radians(30.0)
        g_end = _ry(over) @ g_lean
        yield from R.move(turn_to, turn_s, rot=g_end)
        yield from R.hold(0.5)
        yield from R.grip(SLOT_OPEN, 0.8)
        yield from R.hold(0.4)
        yield from R.move(turn_to - 0.07 * g_end[:, 2], 1.0, rot=g_end)  # back out along the leaned fingers
        yield from R.hold(0.4)
        live = _shaft_live(ctx)
        if live is None:
            break
        print(f"RCDIAG slot clock stroke {stroke}: started {math.degrees(alpha):.1f} deg, allowance "
              f"{math.degrees(over):.1f} deg, corner {math.degrees(corner_alpha(live[0])):.1f} deg after the turn, "
              f"off point-up {clock_error_from_R(live[0]):.1f} deg", flush=True)
    yield from R.home(2.5)
    return True


def _grasp_roller(L, ctx):
    grasp = _tube_live(ctx)  # the tube's live centre: pads across the tube at its axis height
    for attempt in range(2):
        pre = grasp + np.array([0.0, 0.0, 0.09])
        yield from L.move(pre, 2.0 if attempt == 0 else 1.2, GRASP_OPEN, rot=G_LEVEL)
        yield from L.trim_to_measured(pre)
        off = _offset(L, pre)
        yield from L.move(grasp + off, 1.4, rot=G_LEVEL)
        yield from L.grip(FINGER_CLOSED, 0.8)
        yield from L.hold(0.4)
        if L.holding(0.030):
            return True
        yield from L.grip(GRASP_OPEN, 0.5)
        yield from L.move(pre + off, 0.8, rot=G_LEVEL)
        grasp = _tube_live(ctx)
    return False


def _shaft_centre(ctx):
    """Live midpoint of the shaft (between its two ends), or None offline."""
    live = _shaft_live(ctx)
    if live is None:
        return None
    Rm, o = live
    return o + Rm[:, 0] * ROLLER_HALF


def _centre_on_shaft(L, ctx, target):
    """Hold the roller just over the slots and move the hand until the shaft's live midpoint is over the slot line and
    centred between the rails (the tube can sit up to 2 mm along the shaft and the hand lags under the 0.7 kg load).
    Up to 3 corrections, each capped at 8 mm. Returns the corrected hand target."""
    target = np.asarray(target, dtype=float).copy()
    for k in range(3):
        c = _shaft_centre(ctx)
        if c is None:
            break
        err = np.array([float(c[0]) - FX, float(c[1]), 0.0])
        print(f"RCDIAG centring {k}: shaft centre {np.round(c, 4).tolist()}, error {np.round(err * 1000, 1).tolist()} mm",
              flush=True)
        if np.linalg.norm(err) < 0.0012:
            break
        target = target - np.clip(err, -0.008, 0.008)
        yield from _trim_pose(L, target, G_LEVEL, steps=20)
        yield from L.hold(0.3)
    return target


def _seat_roller(L, ctx):
    # Lift the roller by its tube straight up off the V blocks and carry it over the rails, hex point-up.
    yield from L.move((X0, 0.0, Z0 + 0.04), 1.2, rot=G_LEVEL)
    yield from L.move((X0 + 0.03, 0.0, CARRY_Z), 1.8, rot=G_LEVEL)
    above = np.array([FX, 0.0, CARRY_Z])
    yield from L.move(above, 2.2, rot=G_LEVEL)
    yield from _trim_pose(L, above, G_LEVEL, steps=30)
    off = _offset(L, above)
    rot = L.rot.copy()
    entry = np.array([FX, 0.0, SEAT_Z + 0.028])  # hex corners just above the rail tops
    yield from L.move(entry + off, 1.6, rot=rot)
    yield from _trim_pose(L, entry, G_LEVEL, steps=24)
    yield from L.hold(0.4)  # let the hanging shaft settle before reading where it is
    entry = yield from _centre_on_shaft(L, ctx, entry)
    off = _offset(L, entry)
    rot = L.rot.copy()
    down = np.array([entry[0], entry[1], SEAT_Z + 0.008])
    yield from L.move(down + off, 1.6, rot=rot)
    yield from L.press_until(down + off, max_depth=0.016, err_mm=4.0, min_depth=0.006, speed=0.008, rot=rot)
    yield from L.grip(FINGER_OPEN, 1.2)
    yield from L.hold(0.4)
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.11]), 1.6, rot=L.rot.copy())


def _plate_live(ctx):
    """(origin, rotation) of the torque plate from the simulator (origin: the notch apex), or None offline, where the
    reader hands back the plate's start pose in its rack."""
    if getattr(ctx, "offline", False):
        return None
    try:
        s = ctx.obj("torque_plate")
        o = np.asarray(getattr(s, "origin", s.pos), dtype=float)
        x, y, z, w = (float(v) for v in s.quat_xyzw)
        Rm = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return o, Rm
    except Exception:  # noqa: BLE001
        return None


def _plate_tilt(R) -> float:
    """Lean of the plate in its own plane (rad, about world +y): the pinch across its faces lets it swing that way."""
    return math.atan2(float(R[0, 2]), float(R[2, 2]))


def _plate_mean(ctx, n: int = 10):
    """The plate's live pose averaged over n control steps (half a second): a plate hanging in a pinch keeps swinging
    in Isaac, and a single read caught it at either end of a 2.5 mm swing. Returns (origin, rotation) or None."""
    reads = []
    for _ in range(n):
        live = _plate_live(ctx)
        if live is None:
            return None
        reads.append(live)
        yield
    o = np.mean([r[0] for r in reads], axis=0)
    t = float(np.mean([_plate_tilt(r[1]) for r in reads]))
    return o, _ry(t)  # the mean lean only: the pinch holds the plate's other two angles


def _align_plate(L, ctx, hand, rot, z_origin, tries: int = 4):
    """Servo the held plate's live pose (not the hand's) until it stands upright over the pocket: turn the wrist about
    the pinch axis by the plate's lean, then shift the hand by the notch's x and y error. In Isaac a plate pinched across
    its faces swung about 30 deg in the pads and wedged diagonally on a guide post. Returns the corrected hand target
    and rotation. Gated: skipped offline and when the read plate is not in the hand."""
    hand = np.asarray(hand, dtype=float).copy()
    for k in range(tries):
        live = yield from _plate_mean(ctx)
        if live is None:
            break
        o, Rp = live
        if np.linalg.norm(o - (L.measured_pos - np.array([0.0, 0.0, PL_GRIP]))) > 0.05:
            print(f"RCDIAG plate not in the hand: notch {np.round(o, 4).tolist()}", flush=True)
            break
        tilt = _plate_tilt(Rp)
        err = np.array([float(o[0]) - FX, float(o[1]) - Y_PLATE, 0.0])
        print(f"RCDIAG plate align {k}: notch {np.round(o, 4).tolist()} (want z {z_origin:.4f}), error "
              f"{np.round(err * 1000, 1).tolist()} mm, lean {math.degrees(tilt):.1f} deg", flush=True)
        if np.linalg.norm(err) < 0.0015 and abs(tilt) < math.radians(1.5):
            break
        rot = _ry(-0.8 * float(np.clip(tilt, -math.radians(20.0), math.radians(20.0)))) @ rot
        hand = hand - 0.5 * np.clip(err, -0.012, 0.012)  # damped: at 0.7 the error grew from 2 to 5 mm in the replay
        yield from _trim_pose(L, hand, rot, steps=24)
        yield from L.hold(0.4)
    return hand, rot


def _grip_plate(L, origin, Rp):
    """Pinch the plate across its two faces 18 mm below its top, the wrist leaned with the plate."""
    tilt = _plate_tilt(Rp)
    rot = _ry(tilt) @ G_PLATE
    grasp = np.asarray(origin, dtype=float) + Rp[:, 2] * PL_GRIP
    for attempt in range(2):
        pre = grasp + np.array([0.0, 0.0, 0.07])
        yield from L.move(pre, 2.2 if attempt == 0 else 1.0, FINGER_OPEN, rot=rot)
        yield from L.trim_to_measured(pre)
        off = _offset(L, pre)
        yield from L.move(grasp + off, 1.2, rot=rot)
        yield from L.trim_to_measured(grasp, max_duration=0.6)
        off = _offset(L, grasp)
        yield from L.grip(FINGER_CLOSED, 0.7)
        yield from L.hold(0.4)
        if attempt == 1 or L.holding(0.001):  # a 4 mm plate may read near 0 in Isaac: the second close goes ahead
            break
        yield from L.grip(FINGER_OPEN, 0.5)
        yield from L.move(pre + off, 0.8, rot=rot)
    return grasp, off


def _place_plate(L, ctx):
    """Carry the plate over its guide, square and centre it on its live pose, lower it 24 mm into the guide with its
    notch 6 mm over the hex, re-centre, then open and let it drop onto the point-up hex (the guide holds it upright).
    Reads the plate's pose after the drop; if it is not down on the hex, picks it up where it lies and tries again."""
    origin, Rp = np.array(PLATE0, dtype=float), np.eye(3)
    for attempt in range(3):
        grasp, off = yield from _grip_plate(L, origin, Rp)
        lift = grasp + np.array([0.0, 0.0, 0.09])
        yield from L.move(lift + off, 1.2, rot=L.rot.copy())
        z_over = TOP + 0.105  # notch 3.5 cm over the hex, plate bottom 2.5 cm over the guide tops
        over = np.array([FX, Y_PLATE, z_over + PL_GRIP])
        yield from L.move(over + off, 2.4, rot=G_PLATE)
        yield from _trim_pose(L, over, G_PLATE, steps=24)
        yield from L.hold(0.6)  # let the hanging plate stop swinging before reading it
        over, rot = yield from _align_plate(L, ctx, over, G_PLATE, z_over)
        z_mouth = GUIDE_TOP - PL_BOT + 0.006  # plate bottom 6 mm over the guide tops
        hand = np.array([over[0], over[1], z_mouth + PL_GRIP])
        # Lower in short servoed stages, each carrying the trim offset (a long open move dropped it and the plate
        # drifted 11 mm onto the hex top in the replay), re-centring the plate on its live pose at each stage.
        for z_o, tries in ((z_mouth, 3), (SEAT_Z + 0.022, 2), (SEAT_Z + 0.014, 2), (SEAT_Z + 0.006, 1)):
            hand = np.array([hand[0], hand[1], z_o + PL_GRIP])
            off = _offset(L, hand)
            off[2] = 0.0
            yield from L.move(hand + off, 0.8, rot=rot)
            yield from _trim_pose(L, hand, rot, steps=16)
            yield from L.hold(0.3)
            hand, rot = yield from _align_plate(L, ctx, hand, rot, z_o, tries=tries)
        yield from L.grip(0.45 * FINGER_OPEN, 0.8)  # open no wider than needed: pads stay clear of the guide posts
        yield from L.hold(0.8)
        yield from L.move(L.pos + np.array([0.0, 0.0, 0.05]), 1.0, rot=L.rot.copy())
        yield from L.hold(0.4)
        live = _plate_live(ctx)
        if live is None:
            break
        origin, Rp = live
        down = bool(float(Rp[2, 2]) >= math.cos(math.radians(8.0)) and abs(float(origin[0]) - FX) <= 0.002
                    and abs(float(origin[2]) - SEAT_Z) <= 0.001)  # tighter than the scorer's 10 deg, 3 mm, 1.5 mm
        print(f"RCDIAG plate drop {attempt}: notch {np.round(origin, 4).tolist()}, lean "
              f"{math.degrees(_plate_tilt(Rp)):.1f} deg, seated {down}", flush=True)
        if down:
            break
        if abs(_plate_tilt(Rp)) > math.radians(45.0) or float(Rp[2, 2]) < 0.5:
            break  # fallen over: no safe regrip
    yield from L.move(L.pos + np.array([0.0, 0.0, 0.05]), 1.0, rot=L.rot.copy())


def demo_script(ctx):
    """The right arm turns the shaft inside the tube by its hex stub while the tube lies on its V blocks, then parks;
    the left arm then grips the tube, seats the roller centred on the shaft's live pose, and fits the plate.
    Reads the fixed layout (object_jitter=0), joint encoders, gripper openings and, from the simulator, the shaft's
    pose (the cable's end pose: where its stubs are and the hex's angle) and the tube's position."""
    L, R = ctx.left, ctx.right
    live = _shaft_live(ctx)
    if live is not None:
        print(f"RCDIAG start: shaft end {np.round(live[1], 4).tolist()}, corner "
              f"{math.degrees(corner_alpha(live[0])):.1f} deg, tube {np.round(_tube_live(ctx), 4).tolist()}", flush=True)
    yield from _clock_shaft(R, ctx)
    ok = yield from _grasp_roller(L, ctx)
    if not ok:
        print(f"RCDIAG tube grasp missed: gap {L.gap() * 1000:.1f} mm", flush=True)
        yield from L.home()
        return
    yield from _seat_roller(L, ctx)
    live = _shaft_live(ctx)
    if live is not None:
        print(f"RCDIAG seated: shaft end {np.round(live[1], 4).tolist()}, corner "
              f"{math.degrees(corner_alpha(live[0])):.1f} deg", flush=True)
    # The left hand waits over the plate rack while the right hand clocks the seated shaft in its slot; the plate goes
    # on only after the shaft reads point-up.
    yield from L.move((RACK[0], RACK[1], TOP + 0.20), 1.6, FINGER_OPEN, rot=G_PLATE)
    yield from _clock_in_slot(R, ctx)
    yield from _place_plate(L, ctx)
    yield from L.home(3.0)
    yield from ctx.wait(1.0)
