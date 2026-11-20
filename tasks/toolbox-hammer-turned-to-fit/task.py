"""Close a toolbox propped open by a hammer (Stanley STST1-75520 class: plastic box, hinged lid, two plastic latches).

On the workbench in front of the arms stands the toolbox, long side along y, front face towards the robot (x 27 cm),
taped down, its back 5.4 cm from the wall: the lid opens to about 50 deg, where its back edge meets the wall, and falls
shut if let go. A 29 cm claw hammer lies with its head on the tote tray (a small stop block just behind it) and its
handle straight forward over the front rim, the end 8 cm past the face; the lid has been let down onto the handle and
stands about 2.7 cm open at the front. The hammer is longer than the box is deep inside (23.4 cm), so it fits under the
lid only turned toward the box's long side. Each latch is a stiff plastic part hinged on the body below the
seam, with a hook at its top that must pass over a lip on the lid's front edge; within 15 deg of shut a hinge spring
(the snap) pulls it shut. Latch, hook, lip, lid and hammer are all colliding bodies.

Then the box is padlocked: its padlock eye is a tab on the lid's front edge over a matching tab on the body, each with a
13 mm square hole (a bolt-on fixture, see README). A Master Lock No. 5 (51 mm laminated padlock) stands open on the
bench; its shackle (3/8 in, 9.5 mm) springs 22 mm up when open,
turns freely only when fully up, goes down only in line, and catches (stays locked) when pushed home. Padlocked = the
shackle home with its free leg down through both tabs' holes.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Close the toolbox, fasten both latches and padlock it, then let go."

BLACK = (0.07, 0.07, 0.08)
DARK = (0.16, 0.16, 0.17)
YELLOW = (0.96, 0.78, 0.05)
CHROME = (0.72, 0.73, 0.75)
WALL_COL = (0.82, 0.80, 0.76)
WOOD = (0.72, 0.55, 0.33)
STEEL = (0.35, 0.36, 0.38)

# -- box (fixed): x 0.27..0.524, y -0.241..0.241, rim (lid seam) 14.5 cm up --------------------------------------
BX0, BX1 = 0.27, 0.524
BY = 0.241
DEPTH = BX1 - BX0  # 0.254
SEAM = TOP + 0.145
WALL = 0.010
LID_H = 0.070  # lid shell height: lid top 21.5 cm above the bench when shut
HINGE = np.array([BX1, 0.0, SEAM])  # back bottom edge of the lid, axis along y
TRAY_Z = SEAM - 0.050  # tote tray floor, 5 cm below the rim
LID_MASS = 0.60
LID_DAMP = 0.02  # N m s / rad
WALL_GAP = 0.054  # room wall this far behind the box: the lid's back edge meets it at about 50 deg
LID0_DEG = 7.5  # spawn: lid resting on the hammer handle

# -- latches: one rigid plate + hook on a hinge on the body below the seam; lip on the lid's front edge -------------
LATCH_Y = (0.13, -0.13)  # left latch (+y, in front of the left arm), right latch
LIP_D, LIP_H, LIP_W = 0.006, 0.006, 0.050  # lip: sticks out 6 mm from the lid face, 6 mm tall
LIP_Z0 = SEAM + 0.004  # lip bottom (lid shut)
LIP_TOP = LIP_Z0 + LIP_H
GRIP_Y = (-0.225, -0.16)  # the lid's front finger grip (same section as the lips), where the lid is lifted
LAT_T, LAT_W = 0.006, 0.045  # latch plate thickness (x), width (y)
PIVOT_X = BX0 - LIP_D - 0.001 - LAT_T / 2  # plate centre plane: 1 mm in front of the lip
HOOK_D, HOOK_H = 0.0055, 0.006  # hook: from the plate's inner face towards the box (stops 1.5 mm short of the lid)
HOOK_Z0 = LIP_TOP + 0.001  # hook underside, latch shut: 1 mm above the lip of a shut lid
LAT_TOP = HOOK_Z0 + HOOK_H
PIVOT_Z = SEAM - 0.035
LAT_L = LAT_TOP - PIVOT_Z  # 52 mm pivot to top
LAT_MASS, HOOK_MASS = 0.025, 0.006
OPEN_DEG = 30.0  # open stop
SNAP_DEG = 15.0  # within this of shut the snap spring pulls the latch shut
SNAP_K = 0.02  # N m / deg
SNAP_TARGET = 10.0  # deg past shut (the latch stops on its hinge limit at 0.3 deg)
LATCH_DEG = 3.0  # latched: within this of shut

# -- hammer: claw hammer, 29 cm; handle and head welded -----------------------------------------------------
HEAD_C = np.array([0.45, -0.04])  # head centre (x, y) on the tray floor, behind the handle
END_XY = np.array([0.19, -0.04])  # handle end: 8 cm out past the front face, the handle straight across the rim
HANDLE_W, HANDLE_T = 0.030, 0.024  # handle section (across, thickness)
HEAD_L, HEAD_A, HEAD_T = 0.110, 0.028, 0.030  # head: length across the handle, along it, height
HANDLE_MASS, HEAD_MASS = 0.12, 0.45
HANDLE_BACK = 0.02  # handle runs this far past the head centre
PEG_X = 0.470  # reset stop: a 1 cm block on the tray floor just behind the head's middle (x of its front face)


def _rot_pt(dx: float, dz: float, open_deg: float) -> tuple[float, float]:
    """Latch-frame offset (dx towards the box, dz up, latch shut) from the pivot -> world (x, z) offset when the latch
    is open by open_deg (its top swung out towards the robot)."""
    a = math.radians(open_deg)
    return dx * math.cos(a) - dz * math.sin(a), dx * math.sin(a) + dz * math.cos(a)


def _about_hinge(p, deg: float) -> tuple[float, float, float]:
    """World point p carried with the lid opened by deg about the back hinge."""
    a = math.radians(deg)
    dx, dz = p[0] - HINGE[0], p[2] - HINGE[2]
    return (float(HINGE[0] + dx * math.cos(a) + dz * math.sin(a)), float(p[1]),
            float(HINGE[2] - dx * math.sin(a) + dz * math.cos(a)))


def _quat(yaw: float, pitch: float) -> tuple[float, float, float, float]:
    """xyzw of a turn `pitch` about +y followed by `yaw` about +z (world)."""
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    # q = q_yaw * q_pitch, (w, x, y, z)
    w, x, y, z = cy * cp, -sy * sp, cy * sp, sy * cp
    return (float(x), float(y), float(z), float(w))


def _rotm(q_xyzw) -> np.ndarray:
    x, y, z, w = q_xyzw
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


# objects ------------------------------------------------------------------------------------------------------------
ROT: dict = {}  # spawn rotation (xyzw) of the bodies that do not spawn upright


def _box_parts() -> list[Box]:
    xc = (BX0 + BX1) / 2
    h = SEAM - TOP
    return [
        Box("box_front", pos=(BX0 + WALL / 2, 0.0, TOP + h / 2), size=(WALL, 2 * BY, h), color=BLACK),
        Box("box_back", pos=(BX1 - WALL / 2, 0.0, TOP + h / 2), size=(WALL, 2 * BY, h), color=BLACK),
        Box("box_end_l", pos=(xc, BY - WALL / 2, TOP + h / 2), size=(DEPTH - 2 * WALL, WALL, h), color=BLACK),
        Box("box_end_r", pos=(xc, -BY + WALL / 2, TOP + h / 2), size=(DEPTH - 2 * WALL, WALL, h), color=BLACK),
        Box("tray", pos=(xc, 0.0, TOP + (TRAY_Z - TOP) / 2), size=(DEPTH - 2 * WALL, 2 * BY - 2 * WALL, TRAY_Z - TOP),
            color=DARK),
        Box("head_stop", pos=(PEG_X + 0.005, float(HEAD_C[1]), TRAY_Z + 0.005), size=(0.010, 0.010, 0.010), color=DARK),
        Box("room_wall", pos=(BX1 + WALL_GAP + 0.01, 0.0, TOP + 0.30), size=(0.02, 1.2, 0.60), color=WALL_COL),
    ]


LID = Box("lid", pos=_about_hinge((BX0 + DEPTH / 2, 0.0, SEAM + LID_H / 2), LID0_DEG), size=(DEPTH, 2 * BY, LID_H),
          mass=LID_MASS, color=BLACK)
ROT["lid"] = _quat(0.0, math.radians(LID0_DEG))
LIPS = []
for _y, _n in zip(LATCH_Y, ("l", "r")):
    LIPS.append(Box(f"lip_{_n}", pos=_about_hinge((BX0 - LIP_D / 2, _y, LIP_Z0 + LIP_H / 2), LID0_DEG),
                    size=(LIP_D, LIP_W, LIP_H), mass=0.01, color=BLACK))
LIPS.append(Box("lid_grip", pos=_about_hinge((BX0 - LIP_D / 2, sum(GRIP_Y) / 2, LIP_Z0 + LIP_H / 2), LID0_DEG),
                size=(LIP_D, GRIP_Y[1] - GRIP_Y[0], LIP_H), mass=0.01, color=BLACK))
for _l in LIPS:
    ROT[_l.name] = ROT["lid"]

LATCHES, HOOKS = [], []
for _y, _n in zip(LATCH_Y, ("l", "r")):
    _px, _pz = _rot_pt(0.0, LAT_L / 2, OPEN_DEG)
    LATCHES.append(Box(f"latch_{_n}", pos=(PIVOT_X + _px, _y, PIVOT_Z + _pz), size=(LAT_T, LAT_W, LAT_L),
                       mass=LAT_MASS, color=YELLOW))
    _hx, _hz = _rot_pt(LAT_T / 2 + HOOK_D / 2, LAT_L - HOOK_H / 2, OPEN_DEG)
    HOOKS.append(Box(f"hook_{_n}", pos=(PIVOT_X + _hx, _y, PIVOT_Z + _hz), size=(HOOK_D, LAT_W, HOOK_H),
                     mass=HOOK_MASS, color=YELLOW))
    ROT[f"latch_{_n}"] = ROT[f"hook_{_n}"] = _quat(0.0, -math.radians(OPEN_DEG))

# hammer: head flat on the tray floor, handle rising to its end on the front rim
_hz = TRAY_Z + HEAD_T / 2
_d = END_XY - HEAD_C
HAM_YAW = math.atan2(_d[1], _d[0])
# the handle's underside rests on the rim's top: the lowest elevation at which no point of its underside dips into
# the front wall (it crosses the rim at a slant, so one edge meets the rim before the centreline does)
_L_h = float(np.linalg.norm(_d))


def _rim_clear(elev: float) -> float:
    """Lowest height (m) of the handle's underside above the rim, over the part that lies over the front wall."""
    u = np.array([math.cos(HAM_YAW), math.sin(HAM_YAW)])
    acr = np.array([-u[1], u[0]])
    best = 1.0
    for t in np.linspace(0.0, _L_h, 400):
        for w in (-HANDLE_W / 2, 0.0, HANDLE_W / 2):
            xy = HEAD_C + t * u + w * acr
            if BX0 <= xy[0] <= BX0 + WALL:
                z = _hz + t * math.tan(elev) - HANDLE_T / 2 / math.cos(elev)
                best = min(best, z - SEAM)
    return best


def _solve_elev() -> float:
    e = 0.0
    while _rim_clear(e) < 0.001:
        e += math.radians(0.1)
    return e


def _floor_clear(elev: float) -> float:
    """Lowest corner (m) of the head and of the handle's back end above the tray floor."""
    lo = []
    for along, half_a, half_t in ((0.0, HEAD_A / 2, HEAD_T / 2), (-HANDLE_BACK, 0.0, HANDLE_T / 2)):
        for sa in (-1, 1):
            for st in (-1, 1):
                da = along + sa * half_a
                lo.append(_hz + da * math.sin(elev) + st * half_t * math.cos(elev))
    return min(lo) - TRAY_Z


_elev = _solve_elev()
for _ in range(5):  # lift the hammer clear of the floor, then re-solve its rest on the rim
    _hz += max(0.0, 0.001 - _floor_clear(_elev))
    _elev = _solve_elev()
HAM_PITCH = -_elev  # + about y dips +x; the handle rises towards its end
_ez = _hz + _L_h * math.tan(_elev)
_u = np.array([math.cos(HAM_YAW) * math.cos(HAM_PITCH), math.sin(HAM_YAW) * math.cos(HAM_PITCH), -math.sin(HAM_PITCH)])
_start = np.array([HEAD_C[0], HEAD_C[1], _hz]) - HANDLE_BACK * _u
_end = np.array([END_XY[0], END_XY[1], _ez])
HANDLE_L = float(np.linalg.norm(_end - _start))
HANDLE = Box("hammer_handle", pos=tuple(float(c) for c in (_start + _end) / 2), size=(HANDLE_L, HANDLE_W, HANDLE_T), mass=HANDLE_MASS,
             color=WOOD)
HEAD = Box("hammer_head", pos=(float(HEAD_C[0]), float(HEAD_C[1]), float(_hz + 0.002)), size=(HEAD_A, HEAD_L, HEAD_T), mass=HEAD_MASS,
           color=STEEL)
ROT["hammer_handle"] = ROT["hammer_head"] = _quat(HAM_YAW, HAM_PITCH)
HAMMER = [HANDLE, HEAD]


# -- padlock eye: a tab on the body and a tab on the lid, each with a square hole; shut, the holes line up --------
TAB_T = 0.003  # tab thickness
TAB_L = 0.034  # how far the tabs stick out from the front face
TAB_HW = 0.020  # tab half-width (y)
HOLE = 0.013  # square hole (13 mm: the fixture's tabs, see README)
HOLE_X = BX0 - 0.022  # hole centre x (y 0)
BTAB_Z = (SEAM - 0.005, SEAM - 0.002)  # body tab bottom, top
LTAB_Z = (SEAM + 0.001, SEAM + 0.004)  # lid tab bottom, top (lid shut)


def _tab_pieces(prefix: str, z0: float, z1: float) -> list[tuple[str, tuple, tuple]]:
    """A tab with a square hole as four boxes: (name, centre, size)."""
    zc, t = (z0 + z1) / 2, z1 - z0
    xf0, xf1 = BX0 - TAB_L, HOLE_X - HOLE / 2  # front bar (towards the robot)
    xb0, xb1 = HOLE_X + HOLE / 2, BX0  # back bar (against the box)
    return [
        (f"{prefix}_front", ((xf0 + xf1) / 2, 0.0, zc), (xf1 - xf0, 2 * TAB_HW, t)),
        (f"{prefix}_back", ((xb0 + xb1) / 2, 0.0, zc), (xb1 - xb0, 2 * TAB_HW, t)),
        (f"{prefix}_left", (HOLE_X, (HOLE / 2 + TAB_HW) / 2, zc), (HOLE - 0.001, TAB_HW - HOLE / 2, t)),
        (f"{prefix}_right", (HOLE_X, -(HOLE / 2 + TAB_HW) / 2, zc), (HOLE - 0.001, TAB_HW - HOLE / 2, t)),
    ]


LTABS = []
for _n, _c, _s in _tab_pieces("ltab", *LTAB_Z):
    LTABS.append(Box(_n, pos=_about_hinge(_c, LID0_DEG), size=_s, mass=0.005, color=BLACK))
    ROT[_n] = ROT["lid"]
_BT = _tab_pieces("btab", *BTAB_Z)
BTABS = [Box(n, pos=c, size=sz, color=BLACK) for n, c, sz in _BT[:3]]

# -- padlock: 50 mm laminated-steel padlock, open (shackle up), standing on the bench ------------------------------
PL_W, PL_T, PL_H = 0.051, 0.032, 0.041  # Master Lock No. 5 body: width (x, along the shackle's plane), thickness (y), height
SH_D = 0.0095  # shackle bar section (3/8 in)
SH_S = 0.0335  # shackle leg spacing (centre to centre): 24 mm (15/16 in) between the legs
SH_CLR = 0.025  # (1 in) height inside the shackle, locked (body top to the bar's underside)
PL_OPEN = 0.022  # the shackle stands this far up when open
PL_MASS, SH_MASS = 0.30, 0.06
PL_XY = (0.15, 0.10)  # body centre on the bench at the start
PL_SPRING = 150.0  # N/m: the shackle's opening spring (about 3 N to push it home from fully open)
PL_HOLD_K = 4000.0  # N/m: locked, or held up while the shackle is turned out of line
PL_ALIGN = 6.0  # deg: the free leg is over its hole in the body within this
PL_CLICK = 0.0025  # m: pushed this far home (and in line), the lock's pawl catches
PL_TURN_DAMP = 0.005  # N m s / deg: the open shackle turns stiffly in its bore, not freely
PL_TURN_K = 5.0  # N m / deg: holds the shackle in line once it is below fully open
STEEL_PL = (0.62, 0.63, 0.65)
BRASS = (0.78, 0.62, 0.28)


def _padlock(xc: float, yc: float, zb: float, open_: float) -> list[Box]:
    """Padlock with body bottom at zb, shackle raised by open_ (0 = locked); the long leg is the -x leg, the free
    (short) leg the +x one, both in the body's x-z plane."""
    top = zb + PL_H
    xl, xf = xc - SH_S / 2, xc + SH_S / 2
    lz0 = top + open_  # the legs' bottoms
    bar_z = lz0 + SH_CLR + SH_D / 2
    plunger_len = PL_OPEN + 0.006
    return [
        Box("pl_body", pos=(xc, yc, zb + PL_H / 2), size=(PL_W, PL_T, PL_H), mass=PL_MASS, color=STEEL_PL),
        Box("pl_plunger", pos=(xl, yc, lz0 - plunger_len / 2), size=(SH_D * 0.9, SH_D * 0.9, plunger_len),
            mass=0.005, color=CHROME),
        Box("sh_long", pos=(xl, yc, lz0 + (SH_CLR - 0.0005) / 2), size=(SH_D, SH_D, SH_CLR - 0.0005),
            mass=SH_MASS * 0.35, color=CHROME),
        Box("sh_bar", pos=(xc, yc, bar_z), size=(SH_S + SH_D, SH_D, SH_D), mass=SH_MASS * 0.3, color=CHROME),
        Box("sh_free", pos=(xf, yc, lz0 + (SH_CLR - 0.0005) / 2), size=(SH_D, SH_D, SH_CLR - 0.0005),
            mass=SH_MASS * 0.35, color=CHROME),
    ]


PADLOCK = _padlock(PL_XY[0], PL_XY[1], TOP, PL_OPEN)
PL_BODY, PL_PLUNGER, SH_LONG, SH_BAR, SH_FREE = PADLOCK


class PadlockEye(Box):
    """The body tab's right bar (fixed); its spawner also authors all the joints."""


EYE = PadlockEye(_BT[3][0], pos=_BT[3][1], size=_BT[3][2], color=BLACK)

OBJECTS = [*_box_parts(), LID, *LIPS, *LTABS, *LATCHES, *HOOKS, *HAMMER, *PADLOCK, *BTABS, EYE]

TWIN_EXTRA = [
    {"name": "lid_hinge", "kind": "joint", "real": "the lid's back hinge", "note": "revolute 0-85 deg; the room wall "
     "behind the box stops the lid at about 50 deg"},
    {"name": "latches", "kind": "joint", "real": "one-piece plastic snap latches",
     "note": "colliding plate+hook on a hinge (open stop 30 deg); hinge spring pulls it shut within 15 deg"},
    {"name": "hammer", "kind": "weld", "real": "29 cm claw hammer", "note": "handle and head boxes welded"},
    {"name": "padlock_eye", "kind": "weld", "real": "bolt-on padlock eye tabs (3 mm aluminium)", "note": "lid tab welded to the lid over a "
     "fixed body tab, 13 mm square holes"},
    {"name": "padlock", "kind": "joint", "real": "Master Lock No. 5 laminated padlock", "note": "body -slide-> plunger -turn-> "
     "shackle; opening spring ~3 N; per-step catch: in line and within 2.5 mm of home it locks for good"},
]

# -- Isaac side: joints authored when the padlock eye (the last object) is spawned --------------------------------
_P: dict = {}
_OBJ = {o.name: o for o in OBJECTS}


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    q0 = Gf.Quatf(1.0, 0.0, 0.0, 0.0)

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def path(o):
        return Sdf.Path(f"{parent}/{o.name}")

    def local(o, p_world):
        """World point -> body o's frame (bodies spawned turned by ROT)."""
        d = np.asarray(p_world, float) - np.array(o.pos)
        q = ROT.get(o.name)
        return tuple(d) if q is None else tuple(_rotm(q).T @ d)

    def joint(kind, name, b0, b1, p_world, axis="Y", lo=None, hi=None, p1_world=None):
        j = kind.Define(stage, f"{parent}/{EYE.name}/{name}")
        p_world = np.asarray(p_world, float)
        if b0 is not None:
            j.CreateBody0Rel().SetTargets([path(b0)])
            j.CreateLocalPos0Attr(v(local(b0, p_world)))
        else:
            j.CreateLocalPos0Attr(v(W + p_world))
        j.CreateBody1Rel().SetTargets([path(b1)])
        j.CreateLocalPos1Attr(v(local(b1, p_world if p1_world is None else np.asarray(p1_world, float))))
        j.CreateLocalRot0Attr(q0)
        j.CreateLocalRot1Attr(q0)
        if kind is not UsdPhysics.FixedJoint:
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        return d

    deg = math.pi / 180.0
    # lid: hinge to the world along the back edge; the joint frames are written in body frames, so it reads LID0_DEG
    # at the spawn (0 = shut, + opens)
    j = joint(UsdPhysics.RevoluteJoint, "hinge_lid", None, LID, HINGE, lo=0.0, hi=85.0)
    drive(j, "angular", 0.0, LID_DAMP * deg)
    for lip in LIPS:
        joint(UsdPhysics.FixedJoint, f"weld_{lip.name}", LID, lip, lip.pos)
    for lat, hook, y in zip(LATCHES, HOOKS, LATCH_Y):
        j = joint(UsdPhysics.RevoluteJoint, f"hinge_{lat.name}", None, lat, (PIVOT_X, y, PIVOT_Z), lo=-OPEN_DEG,
                  hi=0.3)
        drive(j, "angular", 0.0, 0.0005)
        _P[f"hinge_{lat.name}"] = str(j.GetPath())
        joint(UsdPhysics.FixedJoint, f"weld_{hook.name}", lat, hook, hook.pos)
    joint(UsdPhysics.FixedJoint, "weld_hammer", HANDLE, HEAD, HEAD.pos)
    for t in LTABS:
        joint(UsdPhysics.FixedJoint, f"weld_{t.name}", LID, t, t.pos)
    # padlock: body -(slide, z)-> plunger -(turn about the long leg)-> shackle (long leg, bar, free leg welded)
    xl, yl, top = SH_LONG.pos[0], SH_LONG.pos[1], PL_BODY.pos[2] + PL_H / 2
    j = joint(UsdPhysics.PrismaticJoint, "slide_shackle", PL_BODY, PL_PLUNGER, (xl, yl, top), axis="Z", lo=0.0,
              hi=PL_OPEN + 0.002, p1_world=(xl, yl, top + PL_OPEN))
    drive(j, "linear", PL_SPRING, 2.0, PL_OPEN)
    _P["slide_shackle"] = str(j.GetPath())
    j = joint(UsdPhysics.RevoluteJoint, "turn_shackle", PL_PLUNGER, SH_LONG, (xl, yl, top + PL_OPEN), axis="Z",
              lo=-180.0, hi=180.0)
    drive(j, "angular", 0.0, PL_TURN_DAMP)
    _P["turn_shackle"] = str(j.GetPath())
    joint(UsdPhysics.FixedJoint, "weld_sh_bar", SH_LONG, SH_BAR, SH_BAR.pos)
    joint(UsdPhysics.FixedJoint, "weld_sh_free", SH_LONG, SH_FREE, SH_FREE.pos)

    def deco(prim, name, center, size, color):
        g = UsdGeom.Cube.Define(stage, f"{prim}/{name}")
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))

    # yellow organiser lids on the lid top (visual, in the lid's frame)
    for s in (1, -1):
        deco(f"{parent}/lid", f"organiser_{'l' if s > 0 else 'r'}", (0.0, s * 0.15, LID_H / 2 + 0.002),
             (0.20, 0.12, 0.004), YELLOW)
    if os.environ.get("TOOLBOX_DEBUG"):
        print(f"[toolbox] joints authored under {parent}; env origin {W}", flush=True)


def _spawn_eye(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cuboid
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cuboid(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _eye_object(spec: PadlockEye):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class EyeCfg(sim_utils.CuboidCfg):
        func: object = clone(_spawn_eye)

    cfg = EyeCfg(size=tuple(spec.size), collision_props=CollisionPropertiesCfg(),
                 physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                 dynamic_friction=GRIPPY_FRICTION[1]),
                 visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.BASE, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, 0.0, 1.0)))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_toolbox_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, PadlockEye):
            return _eye_object(spec)
        q = ROT.get(spec.name)
        if q is None:
            return orig(spec)
        # Box specs only carry a yaw: spawn the lid, latches and hammer turned as ROT says instead
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: q
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._toolbox_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _toolbox = True

    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None or spec.loader is None:
            return None
        orig_exec = spec.loader.exec_module

        def exec_module(module):
            orig_exec(module)
            _patch_scene(module)

        spec.loader.exec_module = exec_module
        return spec


def _install() -> None:
    if "yam_arena.scene" in sys.modules:
        _patch_scene(sys.modules["yam_arena.scene"])
    elif not any(getattr(f, "_toolbox", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode: the snap springs run every control step --------------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.snap = [None, None]  # last snap-spring state written (True = pulling shut)
        self.pl_mode = None  # last padlock drive state written: "open" | "held_up" | "no_turn" | "locked"
        self.locked = False  # the padlock has caught (stays locked: there is no key in the task)


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _reset_joints() -> None:
    if not _P:  # not spawned yet (first reset builds the scene; importing pxr before the app starts breaks it)
        return
    try:
        st = _stage()
    except Exception:  # noqa: BLE001  (offline)
        return
    if st is None:
        return
    for lat in LATCHES:
        d = st.GetPrimAtPath(_P.get(f"hinge_{lat.name}", ""))
        if d and d.IsValid():
            d.GetAttribute("drive:angular:physics:stiffness").Set(0.0)
    d = st.GetPrimAtPath(_P.get("slide_shackle", ""))
    if d and d.IsValid():
        d.GetAttribute("drive:linear:physics:stiffness").Set(PL_SPRING)
        d.GetAttribute("drive:linear:physics:targetPosition").Set(PL_OPEN)
    d = st.GetPrimAtPath(_P.get("turn_shackle", ""))
    if d and d.IsValid():
        d.GetAttribute("drive:angular:physics:stiffness").Set(0.0)
        d.GetAttribute("drive:angular:physics:targetPosition").Set(0.0)


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    _reset_joints()




def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_toolbox_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._toolbox_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_toolbox_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                _latch_step(object_states(self._env, [o.name for o in LATCHES]))
                _padlock_step(object_states(self._env, [PL_BODY.name, SH_LONG.name]))
            except Exception as e:  # noqa: BLE001
                print(f"[toolbox] latch step failed: {e!r}", flush=True)
            return out

        _task_step._toolbox_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- geometry readouts -------------------------------------------------------------------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


def _pitch(q_xyzw) -> float:
    """Rotation about world +y (rad) of a body that only turns about y."""
    x, y, z, w = (float(c) for c in q_xyzw)
    return 2.0 * math.atan2(y, w)


def lid_angle(objects) -> float:
    o = objects[LID.name]
    return _pitch(o.quat_xyzw) if _finite(o.quat_xyzw) else float("nan")


def lid_gap(objects, i: int = 0) -> float:
    """How far (m) the lid's front edge stands above shut (the lid is one rigid part: the same at both latches)."""
    a = lid_angle(objects)
    return DEPTH * math.sin(a) if math.isfinite(a) else float("nan")


def latch_open(objects, i: int) -> float:
    """How far (deg) latch i stands open: 0 shut against the box, OPEN_DEG at its stop."""
    o = objects[LATCHES[i].name]
    return -math.degrees(_pitch(o.quat_xyzw)) if _finite(o.quat_xyzw) else float("nan")


def _set_snap(i: int, on: bool) -> None:
    """The plastic snap: within SNAP_DEG of shut a hinge spring pulls the latch shut; beyond it, nothing."""
    if _EP.snap[i] == on:
        return
    _EP.snap[i] = on
    d = _stage().GetPrimAtPath(_P[f"hinge_{LATCHES[i].name}"])
    d.GetAttribute("drive:angular:physics:stiffness").Set(SNAP_K if on else 0.0)
    d.GetAttribute("drive:angular:physics:targetPosition").Set(SNAP_TARGET)


def _latch_step(objects) -> None:
    for i in range(2):
        a = latch_open(objects, i)
        if math.isfinite(a):
            _set_snap(i, a < SNAP_DEG)


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def shackle_state(objects) -> tuple[float, float]:
    """(how far the shackle stands up from home, m; how far it is turned out of line, deg), in the body's frame."""
    b, s = objects[PL_BODY.name], objects[SH_LONG.name]
    if not all(_finite(a) for a in (b.pos, b.quat_xyzw, s.pos, s.quat_xyzw)):
        return float("nan"), float("nan")
    d = _rotm(b.quat_xyzw).T @ (np.asarray(s.pos, float) - np.asarray(b.pos, float))
    z = float(d[2] - (PL_H / 2 + (SH_CLR - 0.0005) / 2))
    bx, by, bz, bw = (float(c) for c in b.quat_xyzw)
    qr = _qmul((-bx, -by, -bz, bw), tuple(float(c) for c in s.quat_xyzw))
    turn = math.degrees(2.0 * math.atan2(qr[2], qr[3]))
    turn = (turn + 180.0) % 360.0 - 180.0
    return z, turn


def _set_pl(mode: str) -> None:
    if _EP.pl_mode == mode:
        return
    _EP.pl_mode = mode
    st = _stage()
    sl = st.GetPrimAtPath(_P["slide_shackle"])
    tu = st.GetPrimAtPath(_P["turn_shackle"])
    k, tgt = {"open": (PL_SPRING, PL_OPEN), "held_up": (PL_HOLD_K, PL_OPEN), "no_turn": (PL_SPRING, PL_OPEN),
              "locked": (PL_HOLD_K, 0.0)}[mode]
    sl.GetAttribute("drive:linear:physics:stiffness").Set(k)
    sl.GetAttribute("drive:linear:physics:targetPosition").Set(tgt)
    tu.GetAttribute("drive:angular:physics:stiffness").Set(0.0 if mode in ("open", "held_up") else PL_TURN_K)
    tu.GetAttribute("drive:angular:physics:targetPosition").Set(0.0)


def _padlock_step(objects) -> None:
    """The lock's mechanism: open, the shackle turns freely only when fully up and goes down only when in line; pushed
    home in line it catches and stays locked."""
    z, turn = shackle_state(objects)
    if not (math.isfinite(z) and math.isfinite(turn)):
        return
    if not _EP.locked and z < PL_CLICK and abs(turn) < PL_ALIGN:
        _EP.locked = True
    if _EP.locked:
        _set_pl("locked")
    elif z < PL_OPEN - 0.003:
        _set_pl("no_turn")  # below fully open: the long leg's notch stops it turning
    elif abs(turn) >= PL_ALIGN:
        _set_pl("held_up")  # swung out of line: the free leg can't go down
    else:
        _set_pl("open")


def _corners(o_spec, state) -> np.ndarray | None:
    if not (_finite(state.pos) and _finite(state.quat_xyzw)):
        return None
    h = np.array(o_spec.size) / 2
    signs = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], dtype=float)
    return np.asarray(state.pos, float) + (signs * h) @ _rotm(state.quat_xyzw).T


IN_LO = np.array([BX0 + WALL - 0.003, -BY + WALL - 0.003, TOP])
IN_HI = np.array([BX1 - WALL + 0.003, BY - WALL + 0.003, SEAM + 0.002])


def hammer_in(objects) -> bool:
    """Every corner of the hammer's handle and head inside the box's walls and below its rim."""
    for spec in HAMMER:
        c = _corners(spec, objects[spec.name])
        if c is None or not (np.all(c >= IN_LO) and np.all(c <= IN_HI)):
            return False
    return True


# -- grippers ----------------------------------------------------------------------------------------------------------
def _robots():
    env = _ENV["env"]
    if env is None:
        return None, None
    base = env.unwrapped
    return base, base.scene.env_origins[0].detach().cpu().numpy().astype(np.float64)


def gripper_points() -> list[np.ndarray]:
    base, origin = _robots()
    if base is None:
        return []
    from yam_arena.runtime import link_poses, pad_tips_world, tcp_world

    pts = []
    for k in ("left_robot", "right_robot"):
        rob = base.scene[k]
        t = np.asarray(tcp_world(rob), dtype=np.float64) - origin
        tip = np.asarray(pad_tips_world(rob), dtype=np.float64) - origin
        pts += [tip, t]
        lp = link_poses(rob)
        for n in ("tip_left", "tip_right"):
            if n in lp:
                pts.append(np.asarray(lp[n][:3, 3], dtype=np.float64) - origin)
    return pts


CLEAR = 0.05
BOX_LO = np.array([min(PIVOT_X - LAT_T / 2 - LAT_L * math.sin(math.radians(OPEN_DEG)), HOLE_X - SH_S / 2 - PL_W / 2),
                   -BY, TOP])  # the box, its open latches and the padlock hanging from its eye
BOX_HI = np.array([BX1, BY, SEAM + LID_H])


def released(objects) -> bool:
    pts = gripper_points()
    if not pts:
        return False
    for p in pts:
        if not _finite(p):
            return False
        if float(np.linalg.norm(np.maximum(np.maximum(BOX_LO - p, p - BOX_HI), 0.0))) < CLEAR:
            return False
    return True


SHUT_GAP = 0.002


def shut(objects, i: int = 0) -> bool:
    g = lid_gap(objects, i)
    return bool(math.isfinite(g) and g <= SHUT_GAP)


def latched(objects, i: int) -> bool:
    """Latch i shut against the box (within LATCH_DEG) with the lid down (its hook over the lip)."""
    a = latch_open(objects, i)
    return bool(math.isfinite(a) and a <= LATCH_DEG and shut(objects, i))


def padlocked(objects) -> bool:
    """The padlock is shut (shackle home and in line, so it has caught) with its free leg down through both tabs'
    holes."""
    z, turn = shackle_state(objects)
    if not (math.isfinite(z) and z <= 0.002 and abs(turn) < PL_ALIGN):
        return False
    f = objects[SH_FREE.name]
    if not _finite(f.pos):
        return False
    x, y, zc = (float(c) for c in f.pos)
    half = (SH_CLR - 0.0005) / 2
    in_hole = abs(x - HOLE_X) <= HOLE / 2 and abs(y) <= HOLE / 2
    return bool(in_hole and zc - half < BTAB_Z[0] and zc + half > LTAB_Z[0] + 0.001)  # through both tabs' holes


def score(objects) -> float:
    """0 unless the hammer lies wholly inside the box below its rim and both grippers are 5 cm clear of the box and
    the lock; then 1.0 both latches latched (latch shut, hook over the lip, lid within 2 mm of shut) and padlocked,
    0.5 both latched but not padlocked, otherwise 0."""
    if not hammer_in(objects) or not released(objects):
        return 0.0
    if not all(latched(objects, i) for i in range(2)):
        return 0.0
    return 1.0 if padlocked(objects) else 0.5


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    g = lid_gap(objects)
    if not math.isfinite(g):
        return 1.0
    return float(max(0.0, g) + 0.01 * (2 - sum(latched(objects, i) for i in range(2))) + 0.05 * (not hammer_in(objects))
                 + 0.02 * (not padlocked(objects)))


def progress(objects) -> dict:
    return {"latched_l": float(latched(objects, 0)), "latched_r": float(latched(objects, 1)),
            "gap_mm": lid_gap(objects) * 1000, "lid_deg": math.degrees(lid_angle(objects)),
            "latch_l_deg": latch_open(objects, 0), "latch_r_deg": latch_open(objects, 1),
            "hammer_in": float(hammer_in(objects)), "released": float(released(objects)),
            "padlocked": float(padlocked(objects)), "shackle_up_mm": shackle_state(objects)[0] * 1000,
            "shackle_turn_deg": shackle_state(objects)[1]}


VARIATION: dict = {}
