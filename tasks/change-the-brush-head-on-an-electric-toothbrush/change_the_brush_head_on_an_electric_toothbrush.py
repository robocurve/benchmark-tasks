"""On a dark granite bathroom counter lies an electric toothbrush on its back, its worn brush head still on its steel
shaft. To its right lies a new brush head, shut in its reclosable clear PET clamshell, and a small pedal bin stands
near the front right.

Task: replace the worn brush head with the new one from its pack, put the worn one in the bin, and finish with both
arms back where they started.

What the physics asks for: two press studs hold the clamshell's lid shut until it is turned about its hinge with
0.18 N m (3.2 N at the studs); the whole pack weighs 12 g, so it slides or lifts with its lid unless its tray is held.
Each brush head is a push fit on the steel shaft that slides only under about 3 N, while the 140 g toothbrush slides
along the counter at about 0.7 N.

Run it:  inspect-robots run --embodiment yam_arena -E task=change_the_brush_head_on_an_electric_toothbrush.py \\
           --instruction "..." --scorer task_score ...
"""

from __future__ import annotations

import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import bodykit  # noqa: E402,F401
from bodykit import (Body, BoxShape, CylShape, SphereShape, allow_matrix_grips, as_numpy, hinge,  # noqa: E402
                     on_reset, on_step, plate, rotation, set_slide_target, set_thread_on, thread)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, TCP_OFFSET  # noqa: E402,F401
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]

# -- the counter ------------------------------------------------------------------------------------------------------
CX0, CX1, CY0, CY1, CT_H = 0.170, 0.480, -0.250, 0.200, 0.010
CT = TOP + CT_H
# -- the toothbrush, lying on its back along x, its shaft forward (+x) -------------------------------------------------
B_LEN, B_W, B_H = 0.170, 0.026, 0.026
AXIS_Z = CT + B_H / 2  # the toothbrush's axis (and its shaft's)
BX, BY = 0.280, -0.005  # the handle's middle
SHAFT_BASE = np.array([BX + B_LEN / 2, BY, AXIS_Z])  # where a head's base sits when it is all the way on
SHAFT_L, SHAFT_R = 0.024, 0.0022
# -- the brush heads: origin at the base of the head (on the shaft's base when on), neck along +x ----------------------
NECK_R, NECK_L = 0.0055, 0.052
COLLAR_R, COLLAR_L = 0.0080, 0.012
HEAD_R = 0.0095
HEAD_HX = COLLAR_L + NECK_L + HEAD_R - 0.002  # the round head's middle, from the head's base
HEAD_MASS = 0.005
# -- the clamshell: a clear PET tray (the base) and its lid, hinged along the tray's right-hand long side -----------
PX0, PY = 0.190, -0.085  # the tray's front (near) end and its middle line
TRAY_L, TRAY_W, PET = 0.130, 0.064, 0.002  # outside length (x) and width (y); walls modelled 2 mm thick
RIM = 0.012  # the tray's rim (and the shut lid's underside) above the counter
THUMB_L = 0.030  # the moulded thumb rest at the tray's front, not covered by the lid
LID_X0, LID_L = PX0 + THUMB_L, TRAY_L - THUMB_L  # the lid covers the tray behind the thumb rest
HINGE = np.array([LID_X0, PY - TRAY_W / 2, CT + RIM])  # the living hinge's front end; it runs along x
BUB_X0, BUB_X1, BUB_HW, BUB_TOP = LID_X0, PX0 + TRAY_L - 0.006, 0.0145, CT + 0.048  # the lid's bubble over the head
BUB_WALL = 0.004  # its walls, modelled 4 mm thick so a squeezing gripper can't push through them
STUD_DX = (0.012, LID_L - 0.012)  # the two press studs, along the lid's free (left-hand) edge
STUD_IN = 0.008  # how far in from that edge
NEW0 = np.array([PX0 + THUMB_L + 0.004, PY, AXIS_Z])  # the new head's base as it lies in the tray (neck along +x)
TRAY_MASS, LID_MASS = 0.006, 0.006
# -- the bin ----------------------------------------------------------------------------------------------------------
BINX, BINY, BIN_R, BIN_H = 0.415, -0.190, 0.045, 0.085
# -- the rules -------------------------------------------------------------------------------------------------------
K_FIT, SLIP, D_FIT = 3000.0, 0.001, 3.0  # the push fit: about 3 N to slide a head along the shaft
OFF = SHAFT_L + 0.004  # a head pulled this far out is off the shaft
CAPTURE_R, CAPTURE_DEG = 0.010, 15.0
ON_DEPTH = 0.003  # a head within this of the shaft's base is all the way on
STUD_HOLD = 0.18  # N m about the hinge the press studs hold the lid shut with (3.2 N at the studs, 5.6 cm out)
STUD_K, STUD_TARGET = 0.15, -1.0  # the studs' detent: a stiff spring pressing the lid shut, capped at STUD_HOLD
POP_DEG = 1.5  # opened this far (1.5 mm at the studs) the studs are out of their sockets: the lid is free
SNAP_DEG = 0.8  # pushed back shut to within this, the studs catch again
HINGE_K, HINGE_TARGET, HINGE_D = 4.0e-5, 125.0, 7.0e-5  # the moulded-open living hinge springs a free lid open
LID_OPEN_MAX = 120.0  # how far the lid opens: over past upright, its bubble clear of the counter
OPEN_DEG = 60.0  # the pack counts as open with its studs out and its lid at least this far open

QUARTZ, QUARTZ_V = (0.20, 0.21, 0.23), (0.36, 0.37, 0.39)  # dark granite: the white toothbrush shows
WHITE_PL, BLUE, BLUE_D, GREY = (0.96, 0.96, 0.96), (0.15, 0.45, 0.85), (0.08, 0.25, 0.55), (0.62, 0.64, 0.67)
STEEL = (0.78, 0.80, 0.82)
BRISTLE_NEW, BRISTLE_TIP, BRISTLE_OLD = (0.97, 0.97, 0.97), (0.20, 0.55, 0.92), (0.86, 0.82, 0.66)
PET_EDGE, PET_FROST, CARD, CARD_W = (0.80, 0.90, 0.97), (0.62, 0.72, 0.80), (0.14, 0.38, 0.78), (0.93, 0.95, 0.97)
BIN_C, BIN_D = (0.95, 0.95, 0.94), (0.55, 0.56, 0.58)

HOME_WITHIN = 0.05
SUCCESS_HOLD_S = 1.0


def _counter() -> Body:
    shapes = [BoxShape((0.0, 0.0, CT_H / 2), (CX1 - CX0, CY1 - CY0, CT_H), QUARTZ)]
    rng = np.random.default_rng(5)
    for _ in range(14):  # paler veins in the granite
        x0, y0 = rng.uniform(-0.14, 0.14), rng.uniform(-0.2, 0.2)
        a = rng.uniform(0, math.pi)
        shapes.append(BoxShape((x0, y0, CT_H + 0.0002), (rng.uniform(0.04, 0.10), 0.0015, 0.0004), QUARTZ_V,
                               collide=False, yaw=a))
    return Body.build("counter", pos=((CX0 + CX1) / 2, (CY0 + CY1) / 2, TOP), mass=1.0, kinematic=True, color=QUARTZ,
                      shapes=shapes, friction=(0.5, 0.4))


def _brush() -> Body:
    """The handle: a white body with a flat back (it lies still), a blue band and power button, and the steel shaft."""
    shapes = [BoxShape((0.0, 0.0, 0.0), (B_LEN - 0.010, B_W - 0.006, B_H), WHITE_PL),
              BoxShape((0.0, 0.0, 0.0), (B_LEN - 0.010, B_W, B_H - 0.006), WHITE_PL)]
    for s in (-1.0, 1.0):  # rounded long edges
        for t in (-1.0, 1.0):
            shapes.append(CylShape((0.0, s * (B_W / 2 - 0.003), t * (B_H / 2 - 0.003)), 0.003, B_LEN - 0.010,
                                   WHITE_PL, axis="X"))
    shapes.append(CylShape((-B_LEN / 2 + 0.005, 0.0, 0.0), B_W / 2 - 0.001, 0.010, GREY, axis="X"))  # the base cap
    shapes.append(CylShape((B_LEN / 2 - 0.004, 0.0, 0.0), 0.0095, 0.008, WHITE_PL, axis="X"))  # the tapered top
    shapes.append(CylShape((B_LEN / 2 - 0.016, 0.0, 0.0), B_W / 2 + 0.0004, 0.006, BLUE, axis="X", collide=False))
    shapes.append(BoxShape((0.030, 0.0, B_H / 2 + 0.0006), (0.016, 0.010, 0.0016), BLUE, collide=False))  # button
    shapes.append(CylShape((-0.002, 0.0, B_H / 2 + 0.0004), 0.0018, 0.0008, (0.25, 0.85, 0.35), collide=False))
    shapes.append(CylShape((B_LEN / 2 + SHAFT_L / 2, 0.0, 0.0), SHAFT_R, SHAFT_L, STEEL, axis="X", collide=False))
    return Body.build("brush", pos=(BX, BY, AXIS_Z), mass=0.14, shapes=shapes, color=WHITE_PL, friction=(0.5, 0.4))


def _head(name: str, pos, worn: bool) -> Body:
    """A round oscillating brush head: a collar over the shaft, a neck, and a round head with bristles up."""
    shapes = [CylShape((COLLAR_L / 2, 0.0, 0.0), COLLAR_R, COLLAR_L, BLUE if not worn else BLUE_D, axis="X"),
              CylShape((COLLAR_L + NECK_L / 2, 0.0, 0.0), NECK_R, NECK_L, WHITE_PL, axis="X")]
    hx = HEAD_HX
    shapes.append(CylShape((hx, 0.0, 0.0), HEAD_R, 0.008, WHITE_PL))  # the round head, its bristles up
    bristle = BRISTLE_OLD if worn else BRISTLE_NEW
    shapes.append(CylShape((hx, 0.0, 0.0085), HEAD_R - 0.0015, 0.009, bristle))
    if worn:  # splayed, worn tufts
        for k in range(8):
            a = k * math.pi / 4
            shapes.append(SphereShape((hx + 0.008 * math.cos(a), 0.008 * math.sin(a), 0.010), 0.0028, bristle,
                                      collide=False))
    else:
        shapes.append(CylShape((hx, 0.0, 0.0133), HEAD_R - 0.0030, 0.0008, BRISTLE_TIP, collide=False))
        for k in range(8):
            a = k * math.pi / 4
            shapes.append(CylShape((hx + 0.0068 * math.cos(a), 0.0068 * math.sin(a), 0.0132), 0.0012, 0.001,
                                   BRISTLE_TIP, collide=False))
    ring = BLUE if not worn else (0.5, 0.5, 0.5)
    shapes.append(CylShape((COLLAR_L + 0.001, 0.0, 0.0), COLLAR_R + 0.0003, 0.002, ring, axis="X", collide=False))
    return Body.build(name, pos=tuple(np.asarray(pos, float).tolist()), mass=HEAD_MASS, shapes=shapes,
                      color=WHITE_PL, friction=(1.2, 1.1), ignore=("brush",), damping=(0.0, 0.1), solver_iterations=32)


def _edge_box(lo, hi, color, collide=False) -> BoxShape:
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return BoxShape(tuple(((lo + hi) / 2).tolist()), tuple((hi - lo).tolist()), color, collide=collide)


def _tray() -> Body:
    """The clamshell's base: a clear PET tray with a raised thumb rest at its front, two cradles under the head and a
    printed card insert. Origin: the front end's bottom, on the middle line. The clear walls collide but are drawn
    only along their edges."""
    L, hw, t, h = TRAY_L, TRAY_W / 2, PET, RIM
    shapes = [_edge_box((0.0, -hw, 0.0), (L, hw, t), None, True)]  # the floor
    for s in (-1.0, 1.0):  # the long walls
        shapes.append(_edge_box((0.0, s * hw - (t if s > 0 else 0.0), 0.0), (L, s * hw + (0.0 if s > 0 else t), h),
                                None, True))
    shapes.append(_edge_box((L - t, -hw, 0.0), (L, hw, h), None, True))  # the back wall
    shapes.append(_edge_box((0.0, -hw, 0.0), (THUMB_L, hw, h), None, True))  # the thumb rest (solid front)
    # what shows of it: the rim, the bottom edges and the thumb rest's frosted, ribbed top
    for s in (-1.0, 1.0):
        shapes.append(_edge_box((0.0, s * hw - 0.0012, h - 0.0012), (L, s * hw + 0.0012, h + 0.0002), PET_EDGE))
        shapes.append(_edge_box((0.0, s * hw - 0.0010, 0.0), (L, s * hw + 0.0010, 0.0015), PET_EDGE))
    for x in (0.0, L):
        shapes.append(_edge_box((x - 0.0012, -hw, h - 0.0012), (x + 0.0012, hw, h + 0.0002), PET_EDGE))
        shapes.append(_edge_box((x - 0.0010, -hw, 0.0), (x + 0.0010, hw, 0.0015), PET_EDGE))
    shapes.append(_edge_box((0.0015, -hw + 0.0015, h - 0.0004), (THUMB_L - 0.001, hw - 0.0015, h + 0.0001), PET_FROST))
    for k in range(4):  # grip ribs on the thumb rest
        x = 0.005 + k * 0.005
        shapes.append(_edge_box((x - 0.0006, -0.018, h), (x + 0.0006, 0.018, h + 0.0006), PET_EDGE))
    for dx in STUD_DX:  # the press studs' sockets in the tray's rim, under the lid's studs
        shapes.append(CylShape((THUMB_L + dx, hw - STUD_IN, h - 0.0004), 0.0042, 0.0006, PET_FROST, collide=False))
    # the cradles: under the collar and under the round head, holding the head level
    hx0 = NEW0[0] - PX0
    collar_bottom = AXIS_Z - COLLAR_R - CT
    head_bottom = AXIS_Z - 0.004 - CT
    shapes.append(_edge_box((hx0 + 0.001, -0.007, t), (hx0 + COLLAR_L - 0.001, 0.007, collar_bottom), PET_FROST, True))
    shapes.append(_edge_box((hx0 + HEAD_HX - 0.006, -0.008, t), (hx0 + HEAD_HX + 0.006, 0.008, head_bottom), PET_FROST,
                            True))
    # the printed card insert lying on the floor (a picture only)
    shapes.append(_edge_box((THUMB_L + 0.002, -hw + 0.004, t), (L - 0.004, hw - 0.004, t + 0.0004), CARD))
    shapes.append(_edge_box((THUMB_L + 0.004, -hw + 0.006, t + 0.0004), (L - 0.006, -hw + 0.014, t + 0.0007), CARD_W))
    shapes.append(_edge_box((L - 0.020, hw - 0.016, t + 0.0004), (L - 0.008, hw - 0.006, t + 0.0007), CARD_W))
    return Body.build("tray", pos=(PX0, PY, CT), mass=TRAY_MASS, shapes=shapes, color=PET_EDGE, friction=(0.6, 0.5),
                      solver_iterations=32, damping=(0.0, 0.1))


def _lid() -> Body:
    """The clamshell's lid: a clear PET cover on the tray's rim with a raised bubble over the head, hinged along the
    tray's right-hand long side; two press studs along its left-hand edge. Origin: the hinge's front end. Clear walls
    are drawn along their edges only."""
    t, L, W = PET, LID_L, TRAY_W
    yc, bw, top, w = TRAY_W / 2, BUB_HW, BUB_TOP - (CT + RIM), BUB_WALL  # the bubble's middle line, half-width, top
    x0, x1 = BUB_X0 - LID_X0, BUB_X1 - LID_X0
    shapes = [_edge_box((0.0, 0.0, 0.0), (L, yc - bw, t), None, True),  # the cover on either side of the bubble
              _edge_box((0.0, yc + bw, 0.0), (L, W, t), None, True),
              _edge_box((x1, yc - bw, 0.0), (L, yc + bw, t), None, True)]  # and behind it
    for s in (-1.0, 1.0):  # the bubble's long walls
        lo, hi = (yc + bw - w, yc + bw) if s > 0 else (yc - bw, yc - bw + w)
        shapes.append(_edge_box((x0, lo, 0.0), (x1, hi, top), None, True))
    for x in (x0, x1 - t):  # its end walls
        shapes.append(_edge_box((x, yc - bw, 0.0), (x + t, yc + bw, top), None, True))
    shapes.append(_edge_box((x0, yc - bw, top - 0.003), (x1, yc + bw, top), None, True))  # its roof
    e = 0.0011  # what shows: the cover's outline, the bubble's edges, a sheen on its roof, the studs, the hinge line
    for y in (0.0, W):
        shapes.append(_edge_box((0.0, y - e, 0.0), (L, y + e, t), PET_EDGE))
    for x in (0.0, L):
        shapes.append(_edge_box((x - e, 0.0, 0.0), (x + e, W, t), PET_EDGE))
    for s in (-1.0, 1.0):
        for z in (t, top):
            shapes.append(_edge_box((x0, yc + s * bw - e, z - e), (x1, yc + s * bw + e, z + e), PET_EDGE))
        for x in (x0, x1):
            shapes.append(_edge_box((x - e, yc + s * bw - e, 0.0), (x + e, yc + s * bw + e, top), PET_EDGE))
    for x in (x0, x1):
        shapes.append(_edge_box((x - e, yc - bw, top - e), (x + e, yc + bw, top + e), PET_EDGE))
    shapes.append(_edge_box((x0 + 0.012, yc - 0.004, top), (x1 - 0.020, yc - 0.0025, top + 0.0003), (0.95, 0.98, 1.0)))
    for dx in STUD_DX:  # the press studs (male buttons) in the cover, over the tray's sockets
        shapes.append(CylShape((dx, W - STUD_IN, t + 0.0012), 0.0035, 0.0024, PET_EDGE, collide=False))
    shapes.append(CylShape((L / 2, 0.0, 0.0), 0.0016, L, PET_FROST, axis="X", collide=False))  # the living hinge
    return Body.build("lid", pos=tuple(HINGE.tolist()), mass=LID_MASS, shapes=shapes, color=PET_EDGE,
                      friction=(0.9, 0.8), solver_iterations=32, damping=(0.0, 0.1))


def _bin() -> Body:
    shapes = []
    n = 14
    for k in range(n):
        a = 2 * math.pi * k / n
        nrm = np.array([math.cos(a), math.sin(a), 0.0])
        m = (BIN_R - 0.0015) * nrm
        w = 2 * BIN_R * math.tan(math.pi / n) + 0.0004
        shapes.append(plate(m, m + [0.0, 0.0, BIN_H], w, 0.003, nrm, BIN_C))
    shapes.append(CylShape((0.0, 0.0, 0.002), BIN_R - 0.002, 0.004, BIN_D))
    shapes.append(CylShape((0.0, 0.0, BIN_H - 0.002), BIN_R + 0.002, 0.004, BIN_D, collide=False))
    return Body.build("bin", pos=(BINX, BINY, CT), mass=0.5, kinematic=True, color=BIN_C, shapes=shapes)


SHAFT_LOCAL = (B_LEN / 2, 0.0, 0.0)  # the shaft's base in the handle's frame
thread("old_on", "brush", "old_head", anchor0=SHAFT_LOCAL, axis0=(1.0, 0.0, 0.0),
       anchor1=(0.0, 0.0, 0.0), axis1=(1.0, 0.0, 0.0), slide_limits=(-0.001, 0.10), turn_damping=0.5,
       slide_stiffness=K_FIT, slide_damping=D_FIT, slide_max_force=40.0, always_on=True)
thread("new_on", "brush", "new_head", anchor0=SHAFT_LOCAL, axis0=(1.0, 0.0, 0.0),
       anchor1=(0.0, 0.0, 0.0), axis1=(1.0, 0.0, 0.0), slide_limits=(-0.001, 0.10), turn_damping=0.5,
       slide_stiffness=K_FIT, slide_damping=D_FIT, slide_max_force=40.0, play=0.001, tilt_deg=3.0)
# the lid on the tray: shut, the studs' detent presses it onto its stop (0 degrees) with at most STUD_HOLD
hinge("lid_hinge", "tray", "lid", anchor=tuple(HINGE.tolist()), axis="X", limits_deg=(0.0, LID_OPEN_MAX), spring=STUD_K,
      damping=1.0e-4, target_deg=STUD_TARGET)

_author_threads_plain = bodykit._author_threads
_author_hinges_plain = bodykit._author_hinges


def _author_threads(stage, prim_path: str, spec) -> None:
    """bodykit's threads, then: the worn head's push fit (on from the start) holds it where it sits, not 10 cm out."""
    _author_threads_plain(stage, prim_path, spec)
    if spec.name == "old_head":
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/old_on")
        UsdPhysics.DriveAPI(prim, "transX").GetTargetPositionAttr().Set(0.0)


def _author_hinges(stage, prim_path: str, spec) -> None:
    """bodykit's hinges, then: the lid hinge's drive is capped at what the press studs hold (STUD_HOLD)."""
    _author_hinges_plain(stage, prim_path, spec)
    if spec.name == "lid":
        from pxr import UsdPhysics

        prim = stage.GetPrimAtPath(f"{prim_path}/lid_hinge")
        UsdPhysics.DriveAPI(prim, "angular").GetMaxForceAttr().Set(float(STUD_HOLD))


bodykit._author_threads = _author_threads
bodykit._author_hinges = _author_hinges

OBJECTS = [_counter(), _bin(), _brush(), _head("old_head", SHAFT_BASE, True), _tray(), _lid(),
           _head("new_head", NEW0, False)]


# -- followed after every control step --------------------------------------------------------------------------------

def _fresh() -> dict:
    return {"on": {"old": True, "new": False}, "hold": {"old": 0.0, "new": 0.0}, "studs": True, "lid_deg": 0.0,
            "tcps": None, "s": {}}


_EP = _fresh()
_DEBUG = {"on": bool(os.environ.get("BRUSH_DEBUG")), "steps": 0}


def _lid_drive(target: float, spring: float, damping: float) -> None:
    bodykit.set_hinge_drive("lid_hinge", target_deg=target, spring=spring, damping=damping)


@on_reset
def _new_episode() -> None:
    _EP.clear()
    _EP.update(_fresh())
    _DEBUG["steps"] = 0
    set_slide_target("old_on", 0.0)
    set_thread_on("old_on", True)
    set_thread_on("new_on", False)
    _lid_drive(STUD_TARGET, STUD_K, 1.0e-4)


def _tcps(base, origin):
    out = []
    for arm in ("left_robot", "right_robot"):
        robot = base.scene[arm]
        i = next(j for j, n in enumerate(robot.body_names) if n == "gripper" or n.endswith("/gripper"))
        g = as_numpy(robot.data.body_link_pose_w)[0][i]
        out.append(g[:3] - origin + rotation(g[3:7]) @ np.array(TCP_OFFSET))
    return out


def _pose(base, origin, name):
    p = as_numpy(base.scene[name].data.root_link_pose_w)[0]
    return p[:3] - origin, rotation(p[3:7])


def _on_shaft(pb, rb, ph, rh):
    """(distance from the shaft's axis, how far out from its base along it, angle to it in degrees) for a head."""
    base_w = pb + rb @ (SHAFT_BASE - np.array([BX, BY, AXIS_Z]))
    q = rb.T @ (ph - base_w)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, float(rb[:, 0] @ rh[:, 0])))))
    return math.hypot(q[1], q[2]), float(q[0]), angle


def _lid_angle(rt, rl) -> float:
    """How far the lid is turned open about the hinge (degrees, 0 shut), from the two bodies' poses."""
    r = rt.T @ rl  # a turn about the hinge's line (x): its left-hand edge up and over to the right
    return math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))


@on_step
def _follow(env) -> None:
    base = env.unwrapped
    origin = as_numpy(base.scene.env_origins[0])
    pb, rb = _pose(base, origin, "brush")
    po, ro = _pose(base, origin, "old_head")
    pn, rn = _pose(base, origin, "new_head")
    pt, rt = _pose(base, origin, "tray")
    pl, rl = _pose(base, origin, "lid")
    off_o, d_o, a_o = _on_shaft(pb, rb, po, ro)
    off_n, d_n, a_n = _on_shaft(pb, rb, pn, rn)
    lid = _lid_angle(rt, rl)
    _EP["lid_deg"] = lid
    if _EP["studs"] and lid > POP_DEG:  # lifted out of the studs' sockets: the lid is free, its hinge springs it open
        _EP["studs"] = False
        _lid_drive(HINGE_TARGET, HINGE_K, HINGE_D)
    elif not _EP["studs"] and lid < SNAP_DEG:  # pushed back shut: the studs catch again
        _EP["studs"] = True
        _lid_drive(STUD_TARGET, STUD_K, 1.0e-4)
    shaft = {"old": (d_o, off_o, a_o), "new": (d_n, off_n, a_n)}
    for name, other in (("old", "new"), ("new", "old")):
        d, off, ang = shaft[name]
        if _EP["on"][name]:
            if d > OFF:  # pulled off the shaft's end: free
                set_thread_on(f"{name}_on", False)
                _EP["on"][name] = False
            elif abs(d - _EP["hold"][name]) > SLIP:  # pushed or pulled along it: it slides, and stays where left
                _EP["hold"][name] = d
                set_slide_target(f"{name}_on", d)
        elif (not _EP["on"][other] and off <= CAPTURE_R and 0.0 <= d <= SHAFT_L + 0.002
              and ang <= CAPTURE_DEG):  # its collar pushed over the free shaft's end: it catches
            set_slide_target(f"{name}_on", d)
            set_thread_on(f"{name}_on", True)
            _EP["on"][name], _EP["hold"][name] = True, d
    old_in_bin = math.hypot(po[0] - BINX, po[1] - BINY) <= BIN_R - 0.005 and CT < po[2] < CT + BIN_H
    _EP["tcps"] = _tcps(base, origin)
    _EP["s"] = {"old_off": not _EP["on"]["old"], "old_in_bin": old_in_bin,
                "new_seated": _EP["on"]["new"] and d_n <= ON_DEPTH,
                "pack_open": (not _EP["studs"]) and lid >= OPEN_DEG}
    if _DEBUG["on"]:
        _DEBUG["steps"] += 1
        if _DEBUG["steps"] % 5 == 0 or (_DEBUG["steps"] < 200 and (lid > 0.05 or os.environ.get("BRUSH_FAST"))):
            mm = lambda v: np.round((np.asarray(v) - [0, 0, CT]) * 1000).astype(int).tolist()  # noqa: E731
            tl, tr = _EP["tcps"]
            fj = [float(as_numpy(base.scene[a].data.joint_pos)[0][base.scene[a].joint_names.index("joint7")]) * 1000
                  for a in ("left_robot", "right_robot")]
            print(f"[brush] t={_DEBUG['steps'] / 20:5.2f}s fingers={[round(v, 1) for v in fj]} lid={lid:6.2f}"
                  f" studs={_EP['studs']} tray={mm(pt)} tray_tilt={math.degrees(math.acos(min(1.0, rt[2, 2]))):.1f}"
                  f" brush={mm(pb)} old={mm(po)} d_old={d_o * 1000:.1f} on={_EP['on']['old']} new={mm(pn)}"
                  f" off_new={off_n * 1000:.1f} d_new={d_n * 1000:.1f} a_new={a_n:.1f} on_new={_EP['on']['new']}"
                  f" bin={old_in_bin} L={mm(tl)} R={mm(tr)} score={score(None)}", file=sys.stderr, flush=True)


# -- scoring --------------------------------------------------------------------------------------------------------

def score(objects) -> float:
    """1.0: the new head all the way on the shaft, the worn head in the bin and both arms back where they started;
    0.75: the same with an arm not back; 0.5: the new head all the way on the shaft; 0.25: the pack open (studs out,
    lid at least 60 degrees open) and the worn head off the shaft; else 0.0. All judged at the end, on live states."""
    s = _EP["s"]
    if not s:
        return 0.0
    if s["new_seated"]:
        if s["old_in_bin"]:
            return 1.0 if _arms_home() else 0.75
        return 0.5
    return 0.25 if s["pack_open"] and s["old_off"] else 0.0


_HOME: dict = {}


def _arms_home() -> bool:
    """Both arms back where they started (each TCP within HOME_WITHIN of its start)."""
    from yam_arena import ik, station
    from yam_arena.constants import READY

    tcps = _EP.get("tcps")
    if tcps is None:
        return False
    if "tcps" not in _HOME:
        _HOME["tcps"] = [np.asarray(ik.tcp_pose(np.array(READY, float), station.BASES[a])[0], float)
                         for a in ("left", "right")]
    return all(float(np.linalg.norm(t - h)) < HOME_WITHIN for t, h in zip(tcps, _HOME["tcps"]))


def success(objects) -> bool:
    """Full score with everything that moved at rest: the trial ends once that has held for SUCCESS_HOLD_S."""
    return score(objects) >= 1.0 and all(objects[n].resting() for n in ("brush", "new_head", "old_head", "tray", "lid"))


# -- scripted demo ----------------------------------------------------------------------------------------------------
# The left hand holds the clamshell's tray by its thumb rest while the right takes the lid's bubble and pulls it over
# toward the hinge: the studs let go and the moulded hinge swings the lid open. The left takes the toothbrush's handle
# and the right pulls the worn head straight off the shaft and drops it in the bin. The left lets go and stands aside
# while the right takes the new head out of the tray (the two wrists would meet over it), then holds the handle again
# while the right lines the new head up in front of the shaft and pushes it straight on, all the way. Both let go and
# go back to where they started.

LEFT_GRIP_X = BX - 0.030  # the left holds the handle here, near its middle (a shove moves it, not turns it)
HANDLE_FINGER = -0.020  # pre-open only a little wider than the handle
NECK_FINGER = -0.012
HANDLE_DEPTH = 0.004
NECK_DEPTH = 0.004  # the neck's axis this far past the TCP: held deep in the jaw (the tips just clear the counter)
NECK_GRIP = COLLAR_L + 0.026  # where the right takes a head's neck, from its base
OVER = 0.060
PULL_PAST = 0.020  # the arm lags under the push fit's 3 N: aim this far past
LEAD_S, LEAD_STEPS = 3.0, 15

PITCH = math.radians(25.0)  # the right's grip pitched forward (its wrist further back) to reach the shaft's end
LEFT_PITCH = math.radians(35.0)  # the left's more, so its wrist stays well clear of the right's
LID_GRIP = (PX0 + TRAY_L - 0.035, PY, BUB_TOP + 0.002)  # the right's TCP on the lid's bubble, 3.6 cm over the hinge
LID_FINGER = -0.024  # 4.8 cm open round the 2.9 cm bubble
LID_HOLD = -(BUB_HW - 0.003)  # a gentle grip on the bubble (about 15 N)
LID_PULL = np.array([0.0, -0.032, 0.004])  # pulled over toward the hinge (about 5 N there pops the studs; arms give)
TRAY_PULL = np.array([0.0, 0.012, 0.0])  # while the left leans the tray the other way
ASIDE = np.array([-0.020, 0.070, 0.090])  # where the left waits while the right takes the new head out
TRAY_GRIP = (PX0 + 0.012, PY, CT + RIM + 0.009)  # the left's TCP across the tray's front, over the thumb rest
TRAY_FINGER = -0.039  # 7.8 cm open round the 6.4 cm tray
TRAY_HOLD = -0.029  # closed to 3 mm inside its sides: a gentle grip (about 15 N) that doesn't squirt the light tray out


def _ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rx(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot(arm: str) -> np.ndarray:
    """Grips closing along y, pitched forward (fingers ahead of the wrist)."""
    return _ry(-(LEFT_PITCH if arm == "left" else PITCH)) @ grip_rotation(0.0)


ROT_TRAY = _ry(math.radians(-35.0)) @ grip_rotation(0.0)  # left: closing along y, its wrist pitched back
ROT_LID = _ry(math.radians(10.0)) @ grip_rotation(0.0)  # right: closing along y, its wrist toward the hinge


def _tcp_for(point, depth: float, rot: np.ndarray) -> np.ndarray:
    return np.asarray(point, float) - depth * rot[:, 2]


def _good_ik(arm: str, pos, rot) -> np.ndarray:
    import itertools

    from yam_arena import ik, station

    best = None
    for seed in itertools.product((-0.7, 0.0, 0.7), (1.3, 1.9), (1.0, 1.7), (-1.3, -0.8), (-0.4, 0.0, 0.4), (-0.4, 0.4)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.array(seed), base_pos=station.BASES[arm],
                             iters=200)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} grip at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, label: str, finger) -> list[Waypoint]:
    from yam_arena import ik, station

    out = []
    for k in range(1, LEAD_STEPS + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / LEAD_STEPS, station.BASES[arm])
        out.append(Waypoint(LEAD_S / LEAD_STEPS, tuple(float(v) for v in p), finger, label, r))
    return out


def demo():
    from yam_arena.constants import READY

    plan = {"left": [], "right": []}
    cur = {}
    up = np.array([0.0, 0.0, OVER])

    def go(arm, t, pos, finger, label, rot=None, n=1):
        p0, _, r0 = cur[arm]
        rot = r0 if rot is None else rot
        p1 = np.asarray(pos, float)
        for j in range(1, n + 1):
            plan[arm].append(Waypoint(t / n, tuple(float(v) for v in p0 + (p1 - p0) * j / n), finger, label, rot))
        cur[arm] = (p1, finger, rot)

    def wait(arm, t, label="wait"):
        p, f, r = cur[arm]
        if t > 0:
            plan[arm].append(Waypoint(t, tuple(float(v) for v in p), f, label, r))

    def dur(arm):
        return sum(w.duration for w in plan[arm])

    def neck(head_base):
        return _tcp_for(np.asarray(head_base, float) + [NECK_GRIP, 0.0, 0.0], NECK_DEPTH, _rot("right"))

    # 1. the left holds the clamshell's tray by its front, the right takes the lid's bubble near the hinge; pulled
    #    apart, the lid turns back over its hinge and the studs let go
    tray = np.asarray(TRAY_GRIP, float)
    plan["left"] += _joint_path("left", np.array(READY, float), _good_ik("left", tray + up, ROT_TRAY),
                                "over the clamshell's front", TRAY_FINGER)
    cur["left"] = (tray + up, TRAY_FINGER, ROT_TRAY)
    go("left", 1.2, tray, TRAY_FINGER, "down round the tray's front")
    go("left", 0.6, tray, TRAY_HOLD, "hold the tray")
    lid = np.asarray(LID_GRIP, float)
    plan["right"] += _joint_path("right", np.array(READY, float), _good_ik("right", lid + up, ROT_LID),
                                 "over the lid's bubble", LID_FINGER)
    cur["right"] = (lid + up, LID_FINGER, ROT_LID)
    wait("right", dur("left") - dur("right"))
    go("right", 1.0, lid, LID_FINGER, "down round the bubble")
    go("right", 0.6, lid, LID_HOLD, "take the lid by its bubble")
    wait("left", dur("right") - dur("left"), "hold the tray")
    go("left", 1.2, tray + TRAY_PULL, TRAY_HOLD, "lean the tray to the left", n=3)
    go("right", 1.2, lid + LID_PULL, LID_HOLD, "pull the lid over toward its hinge: the studs let go", n=3)
    go("right", 0.25, lid + LID_PULL, FINGER_OPEN, "let the lid go: its hinge swings it open")
    go("right", 0.4, lid + LID_PULL + [0.0, 0.0, 0.080], FINGER_OPEN, "rise out of the lid's way")
    wait("left", dur("right") - dur("left") - 1.1, "keep holding the tray")
    go("left", 0.6, tray, TRAY_HOLD, "set the tray back down where it was")
    go("left", 0.4, tray, TRAY_FINGER, "let go of the tray")
    go("left", 0.6, tray + up, TRAY_FINGER, "rise")

    # 2. the left takes the toothbrush's handle and holds it
    hold = _tcp_for((LEFT_GRIP_X, BY, AXIS_Z), HANDLE_DEPTH, _rot("left"))
    go("left", 1.8, hold + up, HANDLE_FINGER, "over the toothbrush's handle", rot=_rot("left"), n=3)
    go("left", 1.2, hold, HANDLE_FINGER, "down round the handle")
    go("left", 0.6, hold, FINGER_CLOSED, "hold the handle")

    # 3. the right pulls the worn head off and drops it in the bin
    old_grip = neck(SHAFT_BASE)
    wait("right", 1.0, "let the lid swing open")
    go("right", 2.0, old_grip + up + [0.0, 0.0, 0.03], NECK_FINGER, "over the worn head", rot=_rot("right"), n=3)
    wait("right", dur("left") - dur("right"))
    go("right", 1.2, old_grip, NECK_FINGER, "down round its neck")
    go("right", 0.6, old_grip, FINGER_CLOSED, "take the worn head")
    go("right", 1.6, neck(SHAFT_BASE + [OFF + PULL_PAST, 0.0, 0.0]), FINGER_CLOSED, "pull it straight off the shaft",
       n=4)
    go("right", 1.0, neck(SHAFT_BASE + [OFF + PULL_PAST, 0.0, 0.110]), FINGER_CLOSED, "lift it clear")
    over_bin = np.array([BINX - NECK_GRIP, BINY, CT + BIN_H + 0.040])
    go("right", 1.6, neck(over_bin), FINGER_CLOSED, "carry it over the bin", n=3)
    go("right", 0.5, neck(over_bin), NECK_FINGER, "drop it in")

    # 4. the left lets go of the handle and stands aside while the right takes the new head out of the open tray
    #    (the two wrists would meet over it); the left takes the handle again and the right pushes the head on
    wait("left", dur("right") - dur("left"), "hold the handle")
    go("left", 0.4, hold, HANDLE_FINGER, "let go of the handle for now")
    go("left", 0.8, hold + ASIDE, HANDLE_FINGER, "stand aside")
    new_grip = neck(NEW0)
    go("right", 1.8, new_grip + up + [0.0, 0.0, 0.070], NECK_FINGER, "over the new head in the open tray", n=3)
    go("right", 0.6, new_grip + up, NECK_FINGER, "lower")
    go("right", 1.0, new_grip, NECK_FINGER, "down round its neck")
    go("right", 0.6, new_grip, FINGER_CLOSED, "take the new head")
    go("right", 0.8, new_grip + [0.0, 0.0, 0.045], FINGER_CLOSED, "lift it out of the tray")
    front = SHAFT_BASE + [SHAFT_L + 0.012, 0.0, 0.0]
    go("right", 1.8, neck(front) + [0.0, 0.0, 0.030], FINGER_CLOSED, "carry it in front of the shaft", n=4)
    wait("left", dur("right") - dur("left") - 1.8, "stand aside")
    go("left", 1.0, hold + up, HANDLE_FINGER, "back over the handle")
    go("left", 0.8, hold, HANDLE_FINGER, "down round the handle")
    go("left", 0.5, hold, FINGER_CLOSED, "hold the handle again")
    wait("right", dur("left") - dur("right"), "wait for the handle to be held")
    go("right", 0.8, neck(front), FINGER_CLOSED, "line it up with the shaft")
    go("right", 2.0, neck(SHAFT_BASE - [PULL_PAST, 0.0, 0.0]), FINGER_CLOSED, "push it straight on, all the way",
       n=5)
    wait("right", 0.4)
    go("right", 0.5, neck(SHAFT_BASE - [PULL_PAST, 0.0, 0.0]), NECK_FINGER, "let go")
    go("right", 0.8, neck(SHAFT_BASE) + up, NECK_FINGER, "rise clear")

    # 5. the left lets go; both arms go home and hold still
    wait("left", dur("right") - dur("left"), "hold the handle")
    go("left", 0.5, hold, HANDLE_FINGER, "let go of the handle")
    go("left", 0.8, hold + up, HANDLE_FINGER, "rise clear")
    wait("right", 1.3)
    for arm in ("left", "right"):
        p, f, r = cur[arm]
        plan[arm] += _joint_path(arm, _good_ik(arm, p, r), np.array(READY, float), "back to where it started", f)
        last = plan[arm][-1]
        plan[arm].append(Waypoint(3.0, last.pos, f,
                                  "hold still: the trial ends once the full score has held 1 s with both arms home",
                                  last.yaw))
    return plan
