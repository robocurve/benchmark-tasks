"""Put one eye drop into a care-training manikin's right eye, then put the bottle back on the tray.

An adult-size care-training head lies back on a gel positioning pad, face up and tilted toward the robot (head tilted back
60 degrees, as a patient looking up). Its right lower lid is soft: drawn toward the cheek, it opens the lower-lid
pocket. A 15 mL dropper bottle of artificial tears lies on the medication tray, cap off. A drop leaves the bottle only
when its walls are squeezed hard enough for long enough with the tip down (dropkit); squeezing harder or longer gives
more drops. The head sits on a sprung neck, so a push on the face moves it (a strike).

Scoring (see score()): latched 0 for the bottle tip touching the skin, a hand touching either eye, a strike or the
bottle dropped on the table; else 0.25 for a drop in the right eye; 0.5 for the first drop in the eye, at most two
drops, each released from no more than 3 cm above where it landed; 0.75 for exactly one drop, landed in the lower-lid
pocket; 1.0 for that plus the bottle back on the tray, let go and still, and the lid let go, for 1 s.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import mechkit as mk  # noqa: E402
import guardkit as gk  # noqa: E402
import dropkit as dk  # noqa: E402
import manikin as mn  # noqa: E402
from yam_arena.constants import FINGER_OPEN, GRIPPER, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Put one drop of the eye medicine in the manikin's right eye, then put the bottle back on the tray."

# -- looks ----------------------------------------------------------------------------------------------------------
SKIN = kit.Look((0.87, 0.72, 0.62), 0.6)
LID = kit.Look((0.86, 0.62, 0.58), 0.5)
EYE = kit.Look((0.93, 0.93, 0.92), 0.15)
FOAM = kit.Look((0.55, 0.70, 0.85), 0.9)  # blue gel positioning pad
STAND = kit.Look((0.80, 0.80, 0.82), 0.5)
TRAY_LOOK = kit.Look((0.96, 0.96, 0.95), 0.4)
LDPE = kit.Look((0.95, 0.95, 0.93), 0.35, opacity=0.9)
CAP_LOOK = kit.Look((0.98, 0.98, 0.98), 0.4)
REST_LOOK = kit.Look((0.92, 0.92, 0.90), 0.9)

# -- the head: frames -------------------------------------------------------------------------------------------------
# Head frame (manikin.py): x out of the face, y the patient's left, z toward the crown, origin mid-skull. The head is
# tilted back HEAD_TILT from upright, facing the robot (-x): the face looks up and toward the arms. The patient's
# left is then the robot's right (-y), so the right eye is on the robot's left of the nose.
HEAD_TILT = math.radians(60.0)
F_W = np.array([-math.cos(HEAD_TILT), 0.0, math.sin(HEAD_TILT)])  # out of the face
L_W = np.array([0.0, -1.0, 0.0])  # the patient's left
U_W = np.array([math.sin(HEAD_TILT), 0.0, math.cos(HEAD_TILT)])  # toward the crown
R_H = np.column_stack([F_W, L_W, U_W])  # head frame -> world
EYE_W = np.array([0.250, 0.072, TOP + 0.189])  # the right eye opening's center (world, at rest)
HEAD_C = EYE_W - R_H @ mn.eye_center(-1)  # head frame origin in the world
PIV_H = np.array([-0.035, 0.0, -0.125])  # neck pivot (both hinges), head frame: below the back of the jaw
PIV_W = HEAD_C + R_H @ PIV_H


def _quat(R) -> tuple:
    """(x, y, z, w) of a rotation matrix."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


Q_H = _quat(R_H)


def _rot(q) -> np.ndarray:
    return gk._rot(q)


def _l(p) -> tuple:
    """A head-frame point in the head link's frame (origin at the neck pivot)."""
    return tuple(float(v) for v in np.asarray(p, float) - PIV_H)


# -- the head: neck springs ---------------------------------------------------------------------------------------------
# Two hinges through the pivot: "tilt" (side to side, about the head's x axis) and "head" (nodding, about its y axis).
# Spring rates are set at the eye: a push straight into the face at the right eye moves it 1.5 mm per N, sideways
# 2.0 mm per N. A strike is the eye pushed STRIKE_MM out of place for STRIKE_STEPS steps (about 4 N into the face), or
# JAB_MM in one step (about 6 N). The real neck is set with a luggage scale at the eye (README).
_R = mn.eye_center(-1) - PIV_H  # from the pivot to the right eye (head frame)
NOD_MM_PER_N, TILT_MM_PER_N = 1.5, 2.0
K_NOD = abs(_R[2]) * math.hypot(_R[0], _R[2]) / (NOD_MM_PER_N / 1000)  # N*m/rad, 16.3
K_TILT = abs(_R[2]) * math.hypot(_R[1], _R[2]) / (TILT_MM_PER_N / 1000)  # N*m/rad, 9.64
STRIKE_MM, JAB_MM, STRIKE_STEPS = 6.0, 9.0, 2

# -- the head: shapes -------------------------------------------------------------------------------------------------
_HEAD_PTS, _HEAD_COUNTS, _HEAD_IDX = mn.mesh()
HEAD_MESH = kit.Mesh(tuple(map(tuple, np.round(_HEAD_PTS - PIV_H, 6))), tuple(_HEAD_COUNTS), tuple(_HEAD_IDX), SKIN)
EYE_MESHES = tuple(kit.lathe([(0.0, -0.0005), (mn.EYE_HALF_W * 0.9, 0.0), (0.0, 0.0005)], EYE,
                             center=_l(mn.eye_center(s) + np.array([0.0005, 0, 0])), axis="x", n=24)
                   for s in (-1, 1))  # the white of each eye opening, a flat disc just proud of the eyeball
# Colliders: boxes inside the skin (checked 0.4 mm to 11 mm under it), so a part touching the skin is stopped close
# to where it is seen. The bottle tip's contact is scored from the skin's own shape (mn.skin_sdf), not these.
HEAD_BOXES = (((-0.058, -0.044, -0.038), (0.040, 0.044, 0.072)),  # skull
              ((0.050, -0.045, -0.035), (0.066, 0.045, 0.030)),  # face under the eyes
              ((0.000, -0.030, -0.085), (0.060, 0.030, -0.040)),  # jaw
              ((0.080, -0.006, -0.030), (0.096, 0.006, -0.002)),  # nose
              ((0.066, -0.040, 0.024), (0.078, 0.040, 0.034)),  # brow
              ((0.066, -0.050, -0.035), (0.080, -0.020, -0.012)),  # right cheek, under the lid pad
              ((0.066, 0.020, -0.035), (0.080, 0.050, -0.012)))  # left cheek


def _col(lo, hi, shift=PIV_H):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return kit.Collider(tuple(float(v) for v in (lo + hi) / 2 - shift), size=tuple(float(v) for v in hi - lo))


HEAD_COLLIDERS = tuple(_col(lo, hi) for lo, hi in HEAD_BOXES)
NECK_H = (0.000, 0.0, -0.110)  # the neck stub's top (head frame), under the jaw
NECK_MESH = kit.cylinder(_l((-0.015, 0.0, -0.140)), 0.045, 0.070, SKIN, axis="z")
NECK_COLLIDER = kit.Collider(_l((-0.015, 0.0, -0.140)), radius=0.035, height=0.050)

# The right lower lid: its margin and a 2.5 x 2.0 cm patch of cheek skin below it move together on one slide straight
# down the face (head -z), travel LID_TRAVEL, sprung back closed at LID_K. A finger pressed on the patch and drawn toward the
# chin pulls it by friction, as on a real face.
LID_PAD_H = np.array([0.0877, -mn.EYE_Y, mn.EYE_Z - mn.EYE_HALF_H - 2 * mn.LID_R - 0.010])  # pad center on the skin
LID_TRAVEL = 0.008
LID_K = 100.0  # N/m: 0.1 N per mm, 0.8 N at the full 8 mm
LID_LET_GO = 0.001  # m: the lid back within 1 mm of closed counts as let go
PAD_SIZE = (0.004, 0.025, 0.020)  # head x (thickness), y, z
_LID_PTS, _LID_COUNTS, _LID_IDX = mn.lid_mesh()
LID_MESHES = (kit.Mesh(tuple(map(tuple, np.round(_LID_PTS - LID_PAD_H, 6))), tuple(_LID_COUNTS), tuple(_LID_IDX), LID),
              kit.box((-PAD_SIZE[0] / 2 + 0.0005, 0.0, 0.0), PAD_SIZE, SKIN))
LID_COLLIDER = kit.Collider((-PAD_SIZE[0] / 2, 0.0, 0.0), size=PAD_SIZE)

MANIKIN = kit.Mechanism(
    "manikin", pos=tuple(PIV_W), size=(0.26, 0.16, 0.26), tilt_xyzw=Q_H,
    meshes=(kit.box((0.0, 0.0, -0.015), (0.03, 0.03, 0.02), STAND),),  # the neck's spring-hinge mount
    colliders=(kit.Collider((0.0, 0.0, -0.015), size=(0.03, 0.03, 0.02)),),
    links=(
        kit.Link("tilt", "revolute", "X", anchor=(0.0, 0.0, 0.0), lower=-0.25, upper=0.25, start=0.0, mass=0.15,
                 meshes=(NECK_MESH,), colliders=(NECK_COLLIDER,), floating=True,
                 law=mk.Spring(rest=0.0, k=K_TILT, damp=0.3)),
        kit.Link("head", "revolute", "Y", anchor=(0.0, 0.0, 0.0), lower=-0.25, upper=0.25, start=0.0, mass=1.0,
                 meshes=(HEAD_MESH,) + EYE_MESHES, colliders=HEAD_COLLIDERS, floating=True, parent="tilt",
                 law=mk.Spring(rest=0.0, k=K_NOD, damp=0.4)),
        kit.Link("lid", "prismatic", "Z", anchor=_l(LID_PAD_H), lower=-LID_TRAVEL, upper=0.0, start=0.0, mass=0.005,
                 meshes=LID_MESHES, colliders=(LID_COLLIDER,), floating=True, parent="head",
                 law=mk.Spring(rest=0.0, k=LID_K, damp=0.5)),
    ))

# Gel head pad: the back of the head rests on a blue gel positioning pad 1 cm thick (world-aligned, fixed). In the sim
# the head floats 3 mm above it on its sprung neck so the pad never props the head against a push.
HEAD_LOW_Z = float(np.min((HEAD_C + _HEAD_PTS @ R_H.T)[:, 2]))
RING_TOP = HEAD_LOW_Z - 0.003
OCC_XY = (HEAD_C + R_H @ np.array([-0.06, 0.0, -0.02]))[:2]  # under the back of the skull
STAND = kit.Prop("headrest", pos=(float(OCC_XY[0]), float(OCC_XY[1]), TOP), size=(0.20, 0.18, max(RING_TOP - TOP, 0.01)),
                 meshes=(kit.box((0.0, 0.0, (RING_TOP - TOP) / 2), (0.20, 0.18, RING_TOP - TOP), FOAM),),
                 colliders=(kit.Collider((0.0, 0.0, (RING_TOP - TOP) / 2), size=(0.20, 0.18, RING_TOP - TOP)),))

# -- tray, bottle rest, bottle, cap -----------------------------------------------------------------------------------
TRAY_C, TRAY_SIZE = (0.250, -0.130), (0.220, 0.140, 0.010)  # white ABS medication tray, fixed
TRAY_TOP = TOP + TRAY_SIZE[2]
TRAY = kit.Prop("tray", pos=(TRAY_C[0], TRAY_C[1], TOP), size=TRAY_SIZE,
                meshes=(kit.box((0, 0, TRAY_SIZE[2] / 2), TRAY_SIZE, TRAY_LOOK),),
                colliders=(kit.Collider((0, 0, TRAY_SIZE[2] / 2), size=TRAY_SIZE),))
REST_SIZE = (0.080, 0.050, 0.010)  # white foam bottle rest with a 2.6 cm groove 4 mm deep, on the tray
REST_FLOOR = TRAY_TOP + REST_SIZE[2] - 0.004  # the groove's floor
REST = kit.Prop("bottle_rest", pos=(TRAY_C[0], TRAY_C[1], TRAY_TOP), size=REST_SIZE,
                meshes=(kit.box((0, 0, (REST_SIZE[2] - 0.004) / 2), (REST_SIZE[0], REST_SIZE[1], REST_SIZE[2] - 0.004),
                                REST_LOOK),
                        kit.box((0, 0.019, REST_SIZE[2] - 0.002), (REST_SIZE[0], 0.012, 0.004), REST_LOOK),
                        kit.box((0, -0.019, REST_SIZE[2] - 0.002), (REST_SIZE[0], 0.012, 0.004), REST_LOOK)),
                colliders=(kit.Collider((0, 0, (REST_SIZE[2] - 0.004) / 2),
                                        size=(REST_SIZE[0], REST_SIZE[1], REST_SIZE[2] - 0.004)),))

# Bottle: 15 mL LDPE eye-drop bottle, 2.5 cm across, 6.5 cm long without its cap, holding 10 mL; 17 g.
# Its frame: origin mid-body, +x toward the tip; the squeeze pads face +-y. Body x -0.030..0.020, nozzle to 0.035.
B_R = 0.0125
B_BODY = (-0.030, 0.020)
B_TIP = 0.035
TIP_R = 0.0015  # the nozzle's rounded end
GRIP_X = -0.005  # the demo holds the body here: 4.0 cm back from the tip
WALL_K = 2000.0  # N/m per wall (2 N per mm)
PAD_T, PAD_TRAVEL = 0.003, 0.005
PAD_X, PAD_W = (-0.028, 0.018), 0.015  # pads run the body's length, 1.5 cm wide
BOTTLE_START = (TRAY_C[0], TRAY_C[1], REST_FLOOR + B_R)
_pad_c = ((PAD_X[0] + PAD_X[1]) / 2, 0.0, 0.0)
_PAD_SIZE = {"Y": (PAD_X[1] - PAD_X[0], PAD_T, PAD_W), "Z": (PAD_X[1] - PAD_X[0], PAD_W, PAD_T)}
_CORE = B_R - PAD_T - PAD_TRAVEL  # the rigid core's half-width under the pads
BOTTLE_PROFILE = [(0.0, B_BODY[0]), (B_R, B_BODY[0]), (B_R, B_BODY[1]), (0.0085, 0.025), (0.005, 0.028),
                  (0.0035, 0.033), (TIP_R, B_TIP - 0.0003), (0.0, B_TIP)]
# The round wall gives under fingers anywhere; the model has four sprung pads in two opposite pairs (90 degrees apart),
# so a grasp across either pair squeezes it.
PADS = (("l", "Y", 1), ("r", "Y", -1), ("t", "Z", 1), ("b", "Z", -1))


class _PadSpring:
    """The wall's spring, with the pad's travel opened only once the spring acts (the first control step). The arena
    lets the scene settle before its first step, with no spring yet: an open pad under the lying bottle would sag to
    its stop and spring back on the first step, throwing the bottle off its rest."""

    sets_limits = True

    def __init__(self, rest, k, damp, lo, hi):
        self.rest, self.k, self.damp, self.lo, self.hi = rest, k, damp, lo, hi

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = (self.lo, self.hi)
        return self.rest, self.k, 0.0, self.damp


def _pad(side, axis, sg):
    anchor = (0.0, sg * (B_R - PAD_T / 2), 0.0) if axis == "Y" else (0.0, 0.0, sg * (B_R - PAD_T / 2))
    lo, hi = (-PAD_TRAVEL, 0.0) if sg > 0 else (0.0, PAD_TRAVEL)
    return kit.Link(f"pad_{side}", "prismatic", axis, anchor=anchor, lower=-1e-4, upper=1e-4, start=0.0, mass=0.0025,
                    meshes=(kit.box(_pad_c, _PAD_SIZE[axis], LDPE),),
                    colliders=(kit.Collider(_pad_c, size=_PAD_SIZE[axis]),),
                    law=_PadSpring(rest=0.0, k=WALL_K, damp=2.0, lo=lo, hi=hi))


BOTTLE = kit.Mechanism(
    "bottle", pos=BOTTLE_START, size=(B_TIP - B_BODY[0], 2 * B_R, 2 * B_R), mass=0.007,
    meshes=(kit.lathe(BOTTLE_PROFILE, LDPE, axis="x", n=28),),
    colliders=(kit.Collider((sum(B_BODY) / 2, 0.0, 0.0), size=(B_BODY[1] - B_BODY[0], 2 * _CORE, 2 * _CORE)),
               kit.Collider((0.0275, 0.0, 0.0), radius=0.004, height=0.015, axis="X")),
    links=tuple(_pad(*p) for p in PADS))
BOTTLE_SPEC = dk.Bottle("bottle", pads=((("pad_l", -1), ("pad_r", 1)), (("pad_t", -1), ("pad_b", 1))),
                        tip=(B_TIP - TIP_R, 0.0, 0.0), axis=(1, 0, 0),
                        wall_k=WALL_K, open_force=4.0, flow=50.0, drop_ul=40.0, tip_down_deg=30.0)
CAP = kit.Prop("cap", pos=(TRAY_C[0] + 0.060, TRAY_C[1] + 0.045, TRAY_TOP), size=(0.016, 0.016, 0.018),
               meshes=(kit.lathe([(0.0, 0.0), (0.008, 0.0), (0.008, 0.018), (0.0, 0.018)], CAP_LOOK, n=24),),
               colliders=(kit.Collider((0, 0, 0.009), radius=0.008, height=0.018),))

OBJECTS = [MANIKIN, STAND, TRAY, REST, BOTTLE, CAP]
GRIPPERS = ("left_gripper", "right_gripper")

# -- thresholds (each with its reason in the README and task.json) ----------------------------------------------------
TIP_TOUCH = 0.0005  # m: the tip's surface within 0.5 mm of the skin is a touch (the solver's resting gap)
MAX_TIP_H = 0.030  # m: a drop released from more than 3 cm above where it lands does not earn 0.5 or more
POCKET_MIN = mn.POCKET_MIN  # m: lid drawn down at least 3 mm for a pocket
ON_TABLE_Z = TOP + 0.018  # bottle center below this, off the tray: it fell on the table
FALL_V = 0.5  # m/s: the bottle, held by no hand, dropping faster than this is falling (a free fall of 1.3 cm)
FALL_STEPS = 5  # steps (0.25 s): how far back a fall counts toward landing on the table
STILL_MM = 0.5  # mm per step: the bottle at rest
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
TRAY_LO = (TRAY_C[0] - TRAY_SIZE[0] / 2, TRAY_C[1] - TRAY_SIZE[1] / 2)
TRAY_HI = (TRAY_C[0] + TRAY_SIZE[0] / 2, TRAY_C[1] + TRAY_SIZE[1] / 2)
BOTTLE_PARTS = (gk.Seg((B_BODY[0], 0, 0), (B_BODY[1], 0, 0), B_R), gk.Seg((B_BODY[1], 0, 0), (B_TIP, 0, 0), 0.006))
EYE_PARTS = tuple(gk.Block(_l(mn.eye_center(s) - np.array([0.002, 0, 0])), (0.008, 2 * mn.EYE_HALF_W, 2 * mn.EYE_HALF_H + 0.002))
                  for s in (-1, 1))


# -- readouts ---------------------------------------------------------------------------------------------------------
def head_pose(objects):
    """(position, rotation) taking head-frame points to the world, as the head is now."""
    s = objects["manikin/head"]
    R = _rot(s.quat_xyzw)
    return np.asarray(s.pos, float) - R @ PIV_H, R


def to_head(objects, p) -> np.ndarray:
    c, R = head_pose(objects)
    return R.T @ (np.asarray(p, float) - c)


def to_world(objects, p) -> np.ndarray:
    c, R = head_pose(objects)
    return c + R @ np.asarray(p, float)


def lid_pulled(objects) -> float:
    """How far the right lower lid is drawn down (m, 0 = closed)."""
    return max(0.0, -kit.joint(objects, "manikin/lid").pos)


def eye_shift_mm(objects) -> float:
    """How far the right eye has been pushed from where it starts (mm)."""
    return 1000.0 * float(np.linalg.norm(to_world(objects, mn.eye_center(-1)) - EYE_W))


def tip_touches_skin(objects) -> bool:
    """The nozzle (its rounded end and the last 7 mm of its cone) reaches the skin: eye, lids, lashes or face."""
    c, R = head_pose(objects)
    s = objects["bottle"]
    Rb = _rot(s.quat_xyzw)
    pull = lid_pulled(objects)
    for x, r in ((B_TIP - TIP_R, TIP_R), (0.031, 0.004), (0.028, 0.005)):
        p_h = R.T @ (np.asarray(s.pos, float) + Rb @ np.array([x, 0.0, 0.0]) - c)
        if float(mn.skin_sdf(p_h, pull)) <= r + TIP_TOUCH:
            return True
    return False


def _in_hand(objects, p) -> bool:
    """Point p (world) inside either gripper's own shape (palm and fingers as they are open now)."""
    for g in GRIPPERS:
        if g not in objects:
            continue
        o, cg, Rg, _, _ = gk._hand_frames(objects, g, "bottle")
        q = Rg.T @ (np.asarray(p, float) - cg)
        for b in gk.hand_boxes(o):
            if np.all(np.abs(q - np.asarray(b.center)) <= np.asarray(b.size) / 2):
                return True
    return False


def land(objects, tip_w):
    """Where a drop leaving the tip lands, falling straight down: ('hand' | 'pocket' | 'eye' | 'face' | 'tray' |
    'table', landing point, info with the tip's height above the landing point)."""
    tip_w = np.asarray(tip_w, float)
    c, R = head_pose(objects)
    pull = lid_pulled(objects)
    hit_h = mn.ray_hit(R.T @ (tip_w - c), R.T @ np.array([0.0, 0.0, -1.0]), lambda p: mn.skin_sdf(p, pull))
    hit_w = None if hit_h is None else c + R @ hit_h
    floor_z = TRAY_TOP if (TRAY_LO[0] <= tip_w[0] <= TRAY_HI[0] and TRAY_LO[1] <= tip_w[1] <= TRAY_HI[1]) else TOP
    end_z = hit_w[2] if hit_w is not None else floor_z
    for z in np.arange(tip_w[2] - 0.002, end_z, -0.002):  # a hand under the tip catches it first
        if _in_hand(objects, (tip_w[0], tip_w[1], z)):
            p = np.array([tip_w[0], tip_w[1], z])
            return "hand", p, {"height": float(tip_w[2] - z), "pulled": pull}
    if hit_w is not None:
        return mn.zone(hit_h, pull), hit_w, {"height": float(tip_w[2] - hit_w[2]), "pulled": pull}
    p = np.array([tip_w[0], tip_w[1], floor_z])
    return ("tray" if floor_z > TOP else "table"), p, {"height": float(tip_w[2] - floor_z), "pulled": pull}


dk.setup(BOTTLE_SPEC, land)


def bottle_on_tray(objects) -> bool:
    p = np.asarray(objects["bottle"].pos, float)
    return (TRAY_LO[0] <= p[0] <= TRAY_HI[0] and TRAY_LO[1] <= p[1] <= TRAY_HI[1]
            and TRAY_TOP <= p[2] <= TRAY_TOP + 0.040)


def bottle_released(objects) -> bool:
    """Neither hand's palm or fingers in contact with the bottle (a contact check)."""
    return not gk.touching(objects, "bottle", BOTTLE_PARTS)


def hand_on_eye(objects) -> bool:
    """A hand's palm or fingers in contact with either eye opening."""
    return bool(gk.touching(objects, "manikin/head", EYE_PARTS))


# -- the trial's record -------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key, self.objs = None, None
        self.touched = self.poked = self.strike = self.fell = False
        self.over = 0  # consecutive steps with the eye pushed STRIKE_MM or more
        self.last = None  # the bottle's position last step
        self.drop_v = []  # m/s: the bottle's drop speed over the last FALL_STEPS steps (0 while a hand holds it)
        self.hold = 0  # consecutive steps with the end state met


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
    dk.update(objects)
    _L.touched |= tip_touches_skin(objects)
    _L.poked |= hand_on_eye(objects)
    shift = eye_shift_mm(objects)
    _L.over = _L.over + 1 if shift >= STRIKE_MM else 0
    _L.strike |= _L.over >= STRIKE_STEPS or shift >= JAB_MM
    p = np.asarray(objects["bottle"].pos, float)
    v = 0.0 if _L.last is None or not bottle_released(objects) else float(_L.last[2] - p[2]) / DT
    _L.drop_v = (_L.drop_v + [v])[-FALL_STEPS:]
    # dropped on the table: off the tray just after falling free; a bottle laid on the table by hand can be put back
    _L.fell |= p[2] < ON_TABLE_Z and not bottle_on_tray(objects) and max(_L.drop_v) >= FALL_V
    still = _L.last is not None and 1000.0 * float(np.linalg.norm(p - _L.last)) <= STILL_MM
    _L.last = p
    done = (bottle_on_tray(objects) and bottle_released(objects) and still and lid_pulled(objects) <= LID_LET_GO)
    _L.hold = _L.hold + 1 if done else 0


def failed() -> bool:
    return _L.touched or _L.poked or _L.strike or _L.fell


def tier() -> float:
    if failed():
        return 0.0
    drops = dk.drops()
    if not any(d.zone in ("eye", "pocket") for d in drops):
        return 0.0  # no drop reached the right eye (doing nothing lands here)
    first = drops[0]
    t = 0.25
    if (len(drops) <= 2 and first.zone in ("eye", "pocket")
            and all(d.info.get("height", 1.0) <= MAX_TIP_H for d in drops)):
        t = 0.5
    if t == 0.5 and len(drops) == 1 and first.zone == "pocket":
        t = 0.75
    if t == 0.75 and _L.hold >= HOLD_STEPS:
        t = 1.0
    return t


def success(objects):
    """One drop in the lower-lid pocket, nothing touched, the bottle back on the tray and let go, the lid let go."""
    _update(objects)
    return tier() == 1.0


def score(objects):
    """0 if the bottle tip ever touched the skin (eye, lids, lashes or face), a hand touched either eye, the head was
    pushed (a strike) or the bottle fell on the table; 0 if no drop landed in the right eye (doing nothing scores 0).
    0.25: a drop landed in the right eye. 0.5: the first drop landed in the right eye, at most 2 drops were released,
    each from no more than 3 cm above where it landed. 0.75: exactly one drop, in the lower-lid pocket (lid drawn down
    at least 3 mm). 1.0: that, and the bottle lies on the tray, let go and still, with the lid let go, for 1 s."""
    _update(objects)
    return tier()


# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
_GRIP0 = np.asarray(grip_rotation(0.0), float)  # the arena's own straight-down grip (fingers close along its x)


def _decompose(R) -> tuple:
    """(yaw, roll, pitch) with kit.Tilted's rotation ry(pitch) @ rx(roll) @ grip_rotation(yaw) equal to R (columns:
    the fingers' closing axis, the hand's width axis, the approach)."""
    A = np.asarray(R, float) @ _GRIP0.T  # grip_rotation(yaw) = rz(yaw) @ grip_rotation(0), so A = ry @ rx @ rz(yaw)
    roll = math.asin(max(-1.0, min(1.0, -A[1, 2])))
    yaw = math.atan2(A[1, 0], A[1, 1])
    pitch = math.atan2(A[0, 2], A[2, 2])
    return yaw, roll, pitch


def _frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - (np.asarray(close, float) @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


# The finger constants below are magnitudes (m from the tool point); the arena's finger joint runs the sign of
# FINGER_OPEN (negative on the real arm), so every waypoint carries F_SIGN * magnitude.
F_SIGN = math.copysign(1.0, FINGER_OPEN)


def _tw(duration, pos, finger, label, R):
    yaw, roll, pitch = _decompose(R)
    return kit.Tilted(duration, tuple(float(v) for v in pos), F_SIGN * finger, label, yaw=yaw, roll=roll,
                      pitch=pitch)


# Where the drop is aimed: POCKET_AIM into the strip the lid uncovers when drawn down (head frame), and the tip
# TIP_ABOVE straight above it (world). The bottle is held as it was taken off the rest (the fingers close along -y
# there), tip down BOTTLE_LEAN from vertical, its tip pointing toward the crown and a little toward the nose
# (LEAN_DIR), turned BOTTLE_TURN about its own axis: of the poses the right arm reaches with joint 6 inside its 120
# degree limit, this one keeps the bottle hand 2.8 cm from the fist and the bottle 1.2 cm from it (the arms' collision
# meshes) and the bottle 5.6 mm from the skin.
LID_PULL = 0.007
POCKET_AIM = 0.0025  # m: the drop aimed this far below the eye opening, inside the strip once the lid is drawn 3 mm
POCKET_H = mn.ray_hit(np.array([0.2, -mn.EYE_Y, mn.EYE_Z - mn.EYE_HALF_H - POCKET_AIM]), np.array([-1.0, 0, 0]),
                      mn.head_sdf)
TIP_ABOVE = 0.020
LIFT_HIGH = 0.09  # m: the bottle hand rises this far above the drop pose before it swings back over the face
CARRY_UP = 0.14  # m: the bottle is lifted this high above its rest before it swings over the head (clear of its side)
BOTTLE_LEAN = math.radians(35.0)
LEAN_DIR = math.radians(-75.0)  # the body leans toward the temple (+y) at 0, toward the crown (+x) at 90 degrees
BOTTLE_TURN = math.radians(340.0)  # the bottle turned about its own axis (it is round; this sets the hand's roll)
# Each finger's position servo (GRIPPER stiffness, N/m) and the wall pad's spring act in series, so a finger commanded
# c past the wall moves the pad c * K_F / (K_F + WALL_K) (5/7 of c).
K_F = float(GRIPPER["stiffness"])
FINGER_HOLD = B_R - 0.0005 * (K_F + WALL_K) / K_F  # each pad pushed 0.5 mm: 1 N on the wall, below the 4 N opening
# The squeeze: a short, firm press that takes the walls to their 5 mm stop (10 N, 300 uL/s), for about 0.2 s
# above the opening force. That makes one drop with the most room for error in an open-loop press: the stop fixes the
# force, so only the press's length counts, and one 40 uL drop needs 3 to 5 control steps of it.
FINGER_SQUEEZE = B_R - PAD_TRAVEL - 0.003  # 3 mm past the walls' stop
FINGER_OPEN_WIDE = 0.025
# The lid hand: a closed fist comes in at a slant, from the chin and temple side, so that it stays clear of the bottle
# hand above the eye. It stops along that slant FIST_TOUCH from the patch, where the fingertips' collision meshes just
# meet it, then presses FIST_DEPTH straight into the skin along the patch's normal (all of the contact on the patch,
# clear of the head's colliders, 10 mm from the eye), so the patch is dragged by friction when the fist moves toward
# the chin. (Pressing on along the slant pushed the patch against its upper stop and nodded the head 4 degrees.)
FIST_LOW = -0.011  # m: the fist aimed 11 mm above the patch's center, so all of its contact is on the patch
LID_FROM_SIDE = 0.8  # the fist's approach: into the skin at the patch, leaning this much toward -y (from the temple)
LID_FROM_CHIN = 1.2  # ... and this much toward the crown (from the chin)
FIST_TOUCH = -0.0075  # m: the tool point along the approach, from the patch, where the fingertips first meet it
FIST_DEPTH = 0.007  # m: then pressed this far into the skin along its normal (5 mm gripped the patch only 3 mm)
_e = 1e-4
PAD_NORMAL_H = np.array([float(mn.head_sdf(LID_PAD_H + _e * v) - mn.head_sdf(LID_PAD_H - _e * v)) for v in np.eye(3)])
PAD_NORMAL_H /= np.linalg.norm(PAD_NORMAL_H)  # the skin's outward normal at the cheek patch (head frame)
SQUEEZE_S = 0.1  # s held at FINGER_SQUEEZE, between a 0.1 s press and a 0.1 s release (about 56 uL in all)


def _tray_grasp() -> np.ndarray:
    """The hand frame of the grasp on the rest: straight down, the fingers closing across the bottle along -y (with
    +y, joint 6 would need -145 degrees, past its -120 limit)."""
    return _frame((0.0, 0.0, -1.0), (0.0, -1.0, 0.0))


GRASP_B = np.array([GRIP_X, 0.0, 0.004])  # the tool point in the bottle's frame, as grasped on the rest


def _bottle_hand_at_eye():
    """The hand's pose with the bottle tip-down TIP_ABOVE over the pocket: the bottle's frame there (its x along the tip
    direction, turned BOTTLE_TURN about it) times the grasp, which holds the bottle as it was taken off the rest."""
    target_w = HEAD_C + R_H @ POCKET_H
    tip_w = target_w + np.array([0.0, 0.0, TIP_ABOVE])
    d = np.array([-math.sin(BOTTLE_LEAN) * math.sin(LEAN_DIR), -math.sin(BOTTLE_LEAN) * math.cos(LEAN_DIR),
                  -math.cos(BOTTLE_LEAN)])  # the tip's direction
    u = np.array([1.0, 0.0, 0.0]) - d[0] * d
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    b2 = math.cos(BOTTLE_TURN) * u + math.sin(BOTTLE_TURN) * v
    B = np.column_stack([d, b2, np.cross(d, b2)])
    R = B @ _tray_grasp()
    grip_w = tip_w - (B_TIP - TIP_R) * d + B @ GRASP_B
    return grip_w, R, d, R[:, 2]


def demo():
    """Right hand: pick the bottle off its rest, turn it tip-down 2 cm above the right lower-lid pocket, squeeze one
    drop, put it back. Left hand: a closed fist on the cheek patch below the right eye, pressed in and drawn 7 mm
    toward the chin, held while the drop falls, then lifted off."""
    # right arm (the bottle)
    R_tray = _tray_grasp()
    g0 = np.array(BOTTLE_START) + np.array([GRIP_X, 0.0, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    grip_w, R_eye, d, a = _bottle_hand_at_eye()
    right = [
        _tw(2.5, g0 + 0.08 * up, FINGER_OPEN_WIDE, "over the bottle", R_tray),
        _tw(1.5, g0 + 0.004 * up, FINGER_OPEN_WIDE, "down to the bottle", R_tray),
        _tw(1.0, g0 + 0.004 * up, FINGER_HOLD, "hold the bottle (no squeeze)", R_tray),
        _tw(1.5, g0 + 0.08 * up, FINGER_HOLD, "lift", R_tray),
        _tw(1.5, g0 + CARRY_UP * up, FINGER_HOLD, "lift high", R_tray),
        _tw(3.0, grip_w + 0.08 * up, FINGER_HOLD, "turn tip-down above the eye", R_eye),
        _tw(1.5, grip_w + 0.02 * up, FINGER_HOLD, "tip 4 cm above the pocket", R_eye),
        _tw(1.0, grip_w, FINGER_HOLD, "tip 2 cm above the pocket", R_eye),
        _tw(2.0, grip_w, FINGER_HOLD, "wait for the lid", R_eye),
        _tw(0.1, grip_w, FINGER_SQUEEZE, "squeeze", R_eye),
        _tw(SQUEEZE_S, grip_w, FINGER_SQUEEZE, "hold the squeeze: one drop", R_eye),
        _tw(0.1, grip_w, FINGER_HOLD, "ease off", R_eye),
        _tw(1.5, grip_w + 0.04 * up, FINGER_HOLD, "lift clear", R_eye),
        _tw(1.0, grip_w + LIFT_HIGH * up - 0.03 * a, FINGER_HOLD, "up and back, clear of the face", R_eye),
        _tw(2.5, g0 + 0.08 * up, FINGER_HOLD, "back over the rest", R_tray),
        _tw(1.5, g0 + 0.006 * up, FINGER_HOLD, "lay it in the rest", R_tray),
        _tw(1.0, g0 + 0.006 * up, FINGER_OPEN_WIDE, "let go", R_tray),
        _tw(1.5, g0 + 0.10 * up, FINGER_OPEN_WIDE, "hand away", R_tray),
    ]
    # left arm (the lid): fist pressed on the cheek patch, approach from the chin and temple side
    pad_w = HEAD_C + R_H @ LID_PAD_H - FIST_LOW * U_W
    a_l = -(R_H @ PAD_NORMAL_H) + np.array([0.0, -LID_FROM_SIDE, 0.0]) + LID_FROM_CHIN * U_W
    a_l /= np.linalg.norm(a_l)
    R_l = _frame(a_l, L_W)  # closing axis across the face
    tcp = lambda p: p - PAD_TIP_PAST_TCP * a_l  # noqa: E731  (the fingertips, not the tool point, on the pad)
    n_w = R_H @ PAD_NORMAL_H  # the skin's outward normal at the patch (world)
    touch = tcp(pad_w + FIST_TOUCH * a_l)
    pressed = touch - FIST_DEPTH * n_w
    down = -U_W * LID_PULL
    t_right_squeeze = sum(w.duration for w in right[:[w.label for w in right].index("squeeze")])
    left = [
        _tw(4.0, tcp(pad_w - 0.06 * a_l + 0.04 * up), 0.0, "fist up beside the face", R_l),
        _tw(2.5, tcp(pad_w - 0.03 * a_l), 0.0, "fist 3 cm off the cheek patch", R_l),
        _tw(1.5, touch, 0.0, "touch the cheek patch", R_l),
        _tw(0.5, pressed, 0.0, "press on the cheek patch", R_l),
        _tw(1.5, pressed + down, 0.0, "draw the lid 7 mm toward the chin", R_l),
    ]
    t_left = sum(w.duration for w in left)
    left.append(_tw(max(0.5, t_right_squeeze + 0.1 + SQUEEZE_S + 0.1 + 1.0 - t_left), pressed + down, 0.0,
                    "hold the lid while the drop falls", R_l))
    left += [_tw(1.0, pressed + down + 0.03 * n_w, 0.0, "lift off: the lid springs back", R_l),
             _tw(2.0, tcp(pad_w - 0.08 * a_l + 0.06 * up), 0.0, "hand away", R_l)]
    return {"right": right, "left": left}
