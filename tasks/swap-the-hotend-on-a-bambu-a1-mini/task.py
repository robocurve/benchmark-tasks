"""On the station table: a Bambu Lab A1 mini 3D printer (powered off and cold; the toolhead's front cover and the
silicone sock already off, filament cut and unloaded), its toolhead parked with its part-cooling ducts' undersides 20 cm
above the table. The installed hotend sits as on the A1: its finned heatsink (1.55 cm wide, 2.0 cm front to back,
1.65 cm tall) in a recess under the toolhead's body, resting on the hotend heating assembly with its magnet block
against the assembly's mounting plate; its 6.5 mm tube in the heating assembly's groove; its nozzle 3.5 mm below the
ducts on either side. The buckle is a bar across the front of the tube, locked: its left end sits behind a catch. To
open it, push it 2.5 mm to the right (its right end rides on a slide), then swing its left end out to the front; to
lock it, swing it shut while it is still pushed right and push it back left. In front of the printer, a printed hotend
holder (a block with a slotted boss at each end): the new 0.2 mm hotend stands nozzle-down in the right slot, the left
slot is empty.

The job (Bambu's A1-series hotend replacement): push the buckle right to unlock it and swing its left side out, take
the old hotend out by its heatsink and stand it in the holder's empty slot, slide the new hotend into the recess until
it sits flush against the mounting plate, then swing the buckle shut and lock it (its left end back behind the catch).

Scored from part poses, the buckle's track and hinge positions, and finger-pad points; see score().
"""

from __future__ import annotations

import math
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

import kitchenkit as kk  # noqa: E402

SCENE = "station"
SCORE_OVER = "end"
TOP = TABLE_TOP[SCENE]

INSTRUCTION = ("Change the 3D printer's hotend: unlock the buckle and swing it open, take the old hotend out and stand "
               "it in the empty slot of the holder, then put the new hotend in until it sits flush and lock the "
               "buckle.")

MM = 0.001
WHITE, BED_C, FRAME_C = (0.86, 0.87, 0.89), (0.13, 0.13, 0.14), (0.24, 0.25, 0.27)
HEAD_C, GRILLE_C, DUCT_C = (0.82, 0.83, 0.85), (0.18, 0.18, 0.20), (0.36, 0.37, 0.40)
HEATER_C, PLATE_C, BUCKLE_C = (0.74, 0.75, 0.78), (0.15, 0.15, 0.16), (0.78, 0.79, 0.82)
SINK_OLD_C, SINK_NEW_C, FIN_C, TUBE_C, TAG_C = (0.12, 0.12, 0.13), (0.78, 0.79, 0.82), (0.45, 0.46, 0.50), \
    (0.70, 0.71, 0.74), (0.20, 0.70, 0.30)
STAND_C = (0.95, 0.55, 0.15)
C = (0.001, 0.0)  # (contact offset, rest offset) for the small parts
RIGID = {"solver_position_iteration_count": 32, "solver_velocity_iteration_count": 4}

# -- the toolhead frame "TH": origin on the installed hotend's tube axis, level with the heatsink's underside (the
#    heating assembly's top); +x away from the robot, +y to the robot's left, +z up. The part-cooling ducts' undersides
#    are 2.46 cm below it, 20 cm above the table.
TH = np.array([0.375, 0.000, TOP + 0.2246])

# -- the hotend (Bambu Hotend - A1 Series: 47.3 mm long; proportions measured on Bambu's photograph). Frame "H": as
#    TH when seated. Heatsink 15.5 (y) x 20 (x) x 16.5 (z) mm, its front 11.5 mm in front of the tube's axis; a magnet
#    block on its back; a 4.4 mm inlet stub on top; the 6.5 mm tube 25.3 mm long and the nozzle 2.8 mm below it.
SINK_X0, SINK_X1, SINK_W, SINK_H = -0.0115, 0.0085, 0.0155, 0.0165
MAG = dict(center=(0.00925, 0.0, 0.008), size=(0.0015, 0.006, 0.006))
STUB_R, STUB_H = 0.0022, 0.0027
TUBE_R, TUBE_L = 0.00325, 0.0253
NOZ_R, NOZ_H, TIP_R, TIP_H = 0.0016, 0.0015, 0.0008, 0.0013
TIP_Z = -(TUBE_L + NOZ_H + TIP_H)  # the nozzle's end, H frame (-28.1 mm)
HOTEND_MASS = 0.018  # kg (Bambu Hotend - A1 Series, listed item weight about 18 g)
SINK_C0 = ((SINK_X0 + SINK_X1) / 2, 0.0, SINK_H / 2)
SINK_SIZE = (SINK_X1 - SINK_X0, SINK_W, SINK_H)


def _hotend_geoms(sink_color, tag: bool):
    g = [kk.Geom("box", SINK_C0, SINK_SIZE, color=sink_color, contact=C),
         kk.Geom("box", MAG["center"], MAG["size"], color=PLATE_C, contact=C),
         kk.Geom("cyl", (0.0, 0.0, SINK_H + STUB_H / 2), (STUB_R, STUB_H), color=TUBE_C, contact=C),
         kk.Geom("cyl", (0.0, 0.0, -TUBE_L / 2), (TUBE_R, TUBE_L), color=TUBE_C, contact=C),
         kk.Geom("cyl", (0.0, 0.0, -TUBE_L - NOZ_H / 2), (NOZ_R, NOZ_H), color=TUBE_C, contact=C),
         kk.Geom("cyl", (0.0, 0.0, -TUBE_L - NOZ_H - TIP_H / 2), (TIP_R, TIP_H), color=TUBE_C, contact=C)]
    for k in range(8):  # fin edges on the front and both sides (visual)
        z = 0.0012 + 0.002 * k
        g.append(kk.Geom("box", (SINK_X0 - 0.0002, 0.0, z), (0.0006, SINK_W - 0.002, 0.0008), color=FIN_C,
                         collide=False))
        for s in (1, -1):
            g.append(kk.Geom("box", (SINK_C0[0], s * (SINK_W / 2 + 0.0002), z), (SINK_SIZE[0] - 0.002, 0.0006, 0.0008),
                             color=FIN_C, collide=False))
    if tag:  # a strip of green tape on the new hotend's heatsink
        g.append(kk.Geom("box", (SINK_X0 - 0.0006, 0.0, 0.0115), (0.0006, 0.010, 0.004), color=TAG_C, collide=False))
    return g


# -- the printer: base with bed, Z column, X arm and carriage, and the toolhead (one 5.5 kg body, loose on the table)
P0 = np.array([TH[0] + 0.040, TH[1], TOP])  # printer frame: on the table under the base's centre

# toolhead boxes, TH frame in mm: (x0, x1, y0, y1, z0, z1, colour[, collide])
HEAD_FRONT = -3.0  # the toolhead body's front, 8.5 mm behind the heatsink's front (front cover off)
RECESS_L, RECESS_R = 12.75, -9.75  # the recess's side walls (5 mm and 2 mm beside the heatsink)
PLATE_X = 10.0  # the heating assembly's mounting plate (the magnet seats against it)
CEIL, INLET_TOP = 17.5, 20.5  # the recess's ceiling (1 mm over the heatsink) and the inlet slot's top (1.3 mm over
#   the inlet stub)
HEATER_Z0 = -20.9  # the heating assembly's underside
GROOVE = 3.6  # half-width of the heating assembly's groove (tube radius 3.25 + 0.35)
DUCT_Y, DUCT_X0, DUCT_Z = 10.5, -3.0, (-24.6, -4.0)  # duct fronts behind the buckle bar's hinge
TOOLHEAD_BOXES = [
    (HEAD_FRONT, 30.0, -27.0, 27.0, INLET_TOP, 75.0, HEAD_C),  # body above the inlet
    (2.6, 30.0, -27.0, 27.0, CEIL, INLET_TOP, HEAD_C),  # ceiling behind the inlet slot
    (HEAD_FRONT, 2.6, 2.6, 27.0, CEIL, INLET_TOP, HEAD_C),  # ceiling beside the inlet slot
    (HEAD_FRONT, 2.6, -27.0, -2.6, CEIL, INLET_TOP, HEAD_C),
    (HEAD_FRONT, 30.0, RECESS_L, 27.0, 0.0, CEIL, HEAD_C),  # the recess's side walls
    (HEAD_FRONT, 30.0, -27.0, RECESS_R, 0.0, CEIL, HEAD_C),
    (PLATE_X, 30.0, RECESS_R, RECESS_L, HEATER_Z0, CEIL, PLATE_C),  # the heating assembly's mounting plate
    (HEAD_FRONT, 30.0, 7.75, 27.0, -4.0, 0.0, HEAD_C),  # body beside the heating assembly
    (HEAD_FRONT, 30.0, -27.0, -7.75, -4.0, 0.0, HEAD_C),
    (-3.25, PLATE_X, GROOVE, 7.75, HEATER_Z0, 0.0, HEATER_C),  # heating assembly: the groove's cheeks and back
    (-3.25, PLATE_X, -7.75, -GROOVE, HEATER_Z0, 0.0, HEATER_C),
    (GROOVE, PLATE_X, -GROOVE, GROOVE, HEATER_Z0, 0.0, HEATER_C),
    (DUCT_X0, 30.0, DUCT_Y, 27.0, DUCT_Z[0], DUCT_Z[1], DUCT_C),  # part-cooling ducts
    (DUCT_X0, 30.0, -27.0, -DUCT_Y, DUCT_Z[0], DUCT_Z[1], DUCT_C),
    (-8.4, -6.4, 6.0, DUCT_Y, -18.5, -15.5, BUCKLE_C),  # the buckle's catch (a hook on the left duct)
    (-6.4, DUCT_X0, 8.5, DUCT_Y, -18.5, -15.5, BUCKLE_C),  # the hook's root (beside the bar's end)
    (HEAD_FRONT - 0.3, HEAD_FRONT, -20.0, 20.0, 28.0, 60.0, GRILLE_C, False),  # hotend fan grille (visual)
    (5.0, 25.0, -27.6, -27.0, 30.0, 55.0, GRILLE_C, False),  # filament cutter lever (visual, right side)
]


def _printer_geoms():
    th = TH - P0  # the toolhead origin in the printer frame
    g = [kk.Geom("box", (0.0, 0.0, 0.035), (0.28, 0.24, 0.070), color=WHITE),
         kk.Geom("box", (-0.030, 0.0, 0.073), (0.18, 0.18, 0.006), color=BED_C),
         kk.Geom("box", (-0.141, -0.060, 0.040), (0.002, 0.070, 0.040), color=GRILLE_C, collide=False),  # screen
         kk.Geom("box", (0.080, -0.145, 0.225), (0.060, 0.050, 0.310), color=FRAME_C),  # Z column (right rear)
         kk.Geom("box", (th[0] + 0.070, -0.015, th[2] + 0.090), (0.030, 0.250, 0.030), color=FRAME_C),  # X arm
         kk.Geom("box", (th[0] + 0.040, 0.0, th[2] + 0.090), (0.025, 0.040, 0.030), color=FRAME_C)]  # carriage
    for b in TOOLHEAD_BOXES:
        x0, x1, y0, y1, z0, z1, col = b[:7]
        collide = b[7] if len(b) > 7 else True
        c = th + MM * np.array([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
        g.append(kk.Geom("box", tuple(c), (MM * (x1 - x0), MM * (y1 - y0), MM * (z1 - z0)), color=col,
                         contact=C if collide else None, collide=collide))
    return g


PRINTER = kk.part("printer", tuple(P0), _printer_geoms(), mass=5.5, color=WHITE, friction=(0.8, 0.7), rigid=RIGID)

# -- the buckle: a bar across the front of the tube, on a vertical hinge at its right end; the hinge rides on a
#    slide (the "track") along y with 2.5 mm of travel to the right. The slide is damped, so it stays where it is
#    pushed. Locked, the bar's left end sits behind the catch on the left duct. Pushing the buckle 2.5 mm to the right
#    frees that end, and the bar's left end then swings out to the front (hinge angle positive); a light spring takes
#    it to its 95-degree stop. Swung shut while still pushed right, its end passes the catch; pushed back left, the end
#    goes behind the catch (locked; the spring then rests it on the catch at about 3 degrees). Closed, the bar lies
#    0.35 mm in front of a seated hotend's tube; over a hotend that is not seated it cannot close, so its end cannot get
#    behind the catch. A tab on the bar is what a gripper takes.
HINGE = np.array([-4.6, -7.5, -17.0]) * MM  # hinge axis (TH frame) with the track at its locked end
BAR_LEN = 0.015
TRACK = 0.0025  # m of travel to the right
TRACK_D = 150.0  # N s/m: the track is a damped slide (it stays where it is pushed: 1.5 N moves it 1 cm/s)
TAB = dict(center=(-0.005, 0.0105, 0.0), size=(0.008, 0.005, 0.005))  # buckle frame (origin on the hinge axis)
HINGE_K = 3e-5  # N m/deg: a light spring that swings the bar open once unlocked and holds it at its 95-degree stop
BUCKLE_MASS, TRACK_MASS = 0.020, 0.050  # kg: heavier than the real steel buckle, for the joint solver
RIGID_B = {"solver_position_iteration_count": 64, "solver_velocity_iteration_count": 8}
BUCKLE_POS = TH + HINGE
SLIDE = kk.part("buckle_track", tuple(BUCKLE_POS), [
    kk.Geom("box", (-0.0004, -0.0003, 0.0), (0.0012, 0.0012, 0.0012), color=BUCKLE_C, contact=C),
    # the track's carriage inside the heating assembly's mounting plate (out of every other part's reach; the
    # track does not collide with the printer it is jointed to): it gives the slide a solid's inertia
    kk.Geom("box", (0.0226, 0.0055, 0.0), (0.012, 0.012, 0.012), color=PLATE_C, contact=C)],
    mass=TRACK_MASS, color=BUCKLE_C, rigid=RIGID_B)
kk.joint("buckle_track_slide", "prismatic", "buckle_track", tuple(BUCKLE_POS), "Y", parent="printer",
         limits=(-TRACK, 0.0), damping=TRACK_D)
BUCKLE = kk.part("buckle", tuple(BUCKLE_POS), [
    kk.Geom("box", (0.0, BAR_LEN / 2, 0.0), (0.002, BAR_LEN, 0.003), color=BUCKLE_C, contact=C),
    kk.Geom("box", TAB["center"], TAB["size"], color=BUCKLE_C, contact=C),
    kk.Geom("cyl", (0.0, 0.0, 0.0), (0.0013, 0.006), color=BUCKLE_C, collide=False)],
    mass=BUCKLE_MASS, color=BUCKLE_C, friction=(0.6, 0.5), rigid=RIGID_B)
kk.joint("buckle_hinge", "revolute", "buckle", tuple(BUCKLE_POS), "Z", parent="buckle_track", limits=(0.0, 95.0),
         stiffness=HINGE_K, damping=1e-3, target=120.0)

# -- the hotends ----------------------------------------------------------------------------------------------------
SEAT = np.zeros(3)  # hotend origin (TH frame) when seated: magnet against the plate, heatsink on the heating assembly
OLD = kk.part("hotend_old", tuple(TH + SEAT), _hotend_geoms(SINK_OLD_C, False), mass=HOTEND_MASS, color=SINK_OLD_C,
              friction=(0.8, 0.7), contact=C, rigid=RIGID)

# -- the printed two-slot holder (frame: on the table under its centre): a block with a 1.2 x 1.0 x 3 cm boss under
#    each slot; the slot is a 7.5 mm square hole down the boss for the tube; the heatsink rests on the boss's top.
STAND_POS = np.array([0.250, 0.030, TOP])
SLOT_Y = {"new": -0.065, "empty": 0.065}  # holder frame y of the two slots: the new hotend right, the empty one left
STAND_H, STAND_X, STAND_Y = 0.100, 0.040, 0.160  # 10 cm tall to the boss tops
BOSS_X, BOSS_Y, BOSS_H, SLOT_W = 0.012, 0.010, 0.030, 0.0075


def _stand_geoms():
    hb = STAND_H - BOSS_H
    g = [kk.Geom("box", (0.0, 0.0, hb / 2), (STAND_X, STAND_Y, hb), color=STAND_C)]
    zc = hb + BOSS_H / 2
    wx, wy = (BOSS_X - SLOT_W) / 2, (BOSS_Y - SLOT_W) / 2
    for y in SLOT_Y.values():
        for s_ in (1, -1):
            g.append(kk.Geom("box", (s_ * (SLOT_W / 2 + wx / 2), y, zc), (wx, BOSS_Y, BOSS_H), color=STAND_C))
            g.append(kk.Geom("box", (0.0, y + s_ * (SLOT_W / 2 + wy / 2), zc), (SLOT_W, wy, BOSS_H), color=STAND_C))
    return g


STAND = kk.part("stand", tuple(STAND_POS), _stand_geoms(), mass=0.200, color=STAND_C, friction=(0.7, 0.6), contact=C)
NEW_IN_STAND = STAND_POS + np.array([0.0, SLOT_Y["new"], STAND_H])  # heatsink resting on the boss's top
NEW = kk.part("hotend_new", tuple(NEW_IN_STAND), _hotend_geoms(SINK_NEW_C, True), mass=HOTEND_MASS, color=SINK_NEW_C,
              friction=(0.8, 0.7), contact=C, rigid=RIGID)

OBJECTS = [PRINTER, SLIDE, BUCKLE, OLD, NEW, STAND]

# -- scoring -------------------------------------------------------------------------------------------------------
SEAT_TOL = 0.0010  # m: the new hotend within 1 mm of its seat in each axis (flush on the plate, on the assembly)
SEAT_TILT = 3.0  # deg
LOCK_DEG = 4.0  # deg: the bar within 4 degrees of shut (locked, it rests on the catch at about 3)
LATCH_MM = 0.6  # mm: and the track within 0.6 mm of its locked end (the bar's end behind the catch)
SLOT_TOL, SLOT_TILT = 0.003, 8.0  # the old hotend standing in a slot: axis within 3 mm of the slot, upright to 8 deg
RELEASE = 0.010  # m: at the end no finger pad within 1 cm of the new hotend or the buckle
HOLD_STEPS = 20  # 1 s at 20 Hz: the finished state, hands off and still
BED_TOP = 0.076  # printer frame z of the bed's top
_S: dict = {}


def _reset() -> None:
    _S.clear()
    _S.update(step=None, result=(0.0, False), last={}, dropped={"hotend_old": False, "hotend_new": False},
              falls={k: kk.FallWatch(0.02, 10) for k in ("hotend_old", "hotend_new")}, hold=0,
              still=kk.StillWatch(0.0015, HOLD_STEPS), max_open=0.0, min_track=0.0)


_reset()
kk.RESET_CALLBACKS.append(_reset)


def _rel(pose_parent, pose_child):
    (pp, Rp), (pc, Rc) = pose_parent, pose_child
    return Rp.T @ (pc - pp), Rp.T @ Rc


def _sink_box(name):
    """The heatsink box of a hotend as (center, size, pose) for the finger checks."""
    return SINK_C0, SINK_SIZE, kk.pose_of(name)


def buckle_state():
    """(hinge angle in degrees, 0 shut and positive open; track position in mm, 0 locked end, negative pushed right)."""
    pr = kk.pose_of("printer", P0)
    ps, _ = _rel(pr, kk.pose_of("buckle_track", BUCKLE_POS))
    track = 1000.0 * float(ps[1] - (BUCKLE_POS - P0)[1])
    _, Rb = _rel(pr, kk.pose_of("buckle", BUCKLE_POS))
    return math.degrees(math.atan2(Rb[1, 0], Rb[0, 0])), track


def _update(objects):
    st = kk.STATE.step
    if _S["step"] == st and _S["step"] is not None:
        return _S["result"]
    _S["step"] = st
    pr = kk.pose_of("printer", P0)
    th_pose = (pr[0] + pr[1] @ (TH - P0), pr[1])  # toolhead frame (TH) in the world
    ang, track = buckle_state()
    _S["max_open"] = max(_S["max_open"], ang)
    _S["min_track"] = min(_S["min_track"], track)
    shut = ang < LOCK_DEG
    latched = shut and track > -LATCH_MM
    sp = kk.pose_of("stand", STAND_POS)
    res = {}
    for name in ("hotend_old", "hotend_new"):
        p, R = kk.pose_of(name)
        pl, Rl = _rel(th_pose, (p, R))
        tilt = kk.tilt_deg(R)
        err = pl - SEAT
        seated = bool(np.all(np.abs(err) <= SEAT_TOL) and tilt <= SEAT_TILT)
        in_recess = (abs(pl[1]) < 0.010 and -0.012 < pl[0] < 0.004 and -0.004 < pl[2] < 0.004)
        ps, Rs = _rel(sp, (p, R))
        slot = None
        for k, y in SLOT_Y.items():
            if (abs(ps[0]) <= SLOT_TOL and abs(ps[1] - y) <= SLOT_TOL and abs(ps[2] - STAND_H) <= 0.003
                    and tilt <= SLOT_TILT):
                slot = k
        handled = any(kk.near(arm, *_sink_box(name), 0.015) for arm in kk.ARMS)
        # a drop: falls 2 cm in 0.5 s with no finger near, or its nozzle tip reaches the bed or the table
        tip = p + R @ np.array([0.0, 0.0, TIP_Z])
        pp = pr[1].T @ (p - pr[0])
        over_bed = abs(pp[0] + 0.030) < 0.09 and abs(pp[1]) < 0.09
        on_bed = over_bed and (pr[1].T @ (tip - pr[0]))[2] < BED_TOP + 0.004
        on_table = slot is None and tip[2] < TOP + 0.004 and not over_bed
        if _S["falls"][name].update(p[2], handled or in_recess or slot is not None) or on_bed or on_table:
            _S["dropped"][name] = True
        res[name] = dict(local=pl, tilt=tilt, seated=seated, in_recess=in_recess, slot=slot)
    old, new = res["hotend_old"], res["hotend_new"]
    removed = old["slot"] is not None and not _S["dropped"]["hotend_old"]
    seated = new["seated"]
    lock_ok = seated and latched
    # hands off: no finger pad within 1 cm of the new hotend's heatsink or the buckle's bar and tab
    bp = kk.pose_of("buckle", BUCKLE_POS)
    hands_off = not any(kk.near(arm, *_sink_box("hotend_new"), RELEASE) or
                        kk.near(arm, TAB["center"], TAB["size"], bp, RELEASE) or
                        kk.near(arm, (0.0, BAR_LEN / 2, 0.0), (0.002, BAR_LEN, 0.003), bp, RELEASE)
                        for arm in kk.ARMS)
    still = _S["still"].update(kk.pose_of("hotend_new")[0], bp[0])
    finished = lock_ok and hands_off and still
    _S["hold"] = _S["hold"] + 1 if finished else 0
    veto = _S["dropped"]["hotend_new"]
    sc = 0.0 if veto else (0.2 * removed + 0.4 * seated + 0.3 * lock_ok + 0.1 * (_S["hold"] >= HOLD_STEPS))
    done = (not veto) and removed and _S["hold"] >= HOLD_STEPS
    _S["last"] = dict(buckle_deg=round(ang, 1), track_mm=round(track, 2), latched=bool(latched),
                      buckle_max_open_deg=round(_S["max_open"], 1), track_min_mm=round(_S["min_track"], 2),
                      old_slot=old["slot"], old_dropped=_S["dropped"]["hotend_old"],
                      new_dropped=_S["dropped"]["hotend_new"],
                      new_local_mm=[round(1000 * float(v), 2) for v in new["local"]],
                      old_local_mm=[round(1000 * float(v), 1) for v in old["local"]],
                      new_tilt=round(new["tilt"], 1), new_seated=bool(seated), new_in_recess=bool(new["in_recess"]),
                      hands_off=bool(hands_off), hold_steps=_S["hold"],
                      printer_shift_mm=round(1000 * float(np.linalg.norm((pr[0] - P0)[:2])), 1))
    _S["result"] = (round(sc, 3), bool(done))
    return _S["result"]


def success(objects) -> bool:
    """The old hotend stands in a holder slot, the new hotend is seated and the buckle shut and locked, both hands
    off and everything still for 1 s, and the new hotend was never dropped."""
    return _update(objects)[1]


def score(objects) -> float:
    """0.2 old hotend standing in a holder slot (axis within 3 mm of the slot, upright to 8 deg, not dropped on the way)
    + 0.4 new hotend seated (within 1 mm of its seat in each axis: flush against the mounting plate, on the heating
    assembly, centred; tilt <= 3 deg) + 0.3 buckle shut (within 4 deg) and locked (track within 0.6 mm of its locked
    end, so the bar's end is behind the catch) with the new hotend seated + 0.1 that finished state held 1 s with no
    finger pad within 1 cm of the new hotend or the buckle. 0 if the new hotend is dropped (falls 2 cm with no finger
    near it, or its nozzle reaches the bed or the table)."""
    return _update(objects)[0]


def report() -> dict:
    return dict(_S.get("last", {}))
