"""On the station table: a car front seat (cushion, backrest and head restraint) with its three-point seat belt as
installed in a car. The belt's webbing comes out of an emergency-locking retractor (ELR) at the foot of the door
pillar, runs up to the shoulder guide (D-ring) on the pillar and down again, through the sliding latch plate (the
tongue), to its anchor at the sill beside the cushion; with the belt unfastened the tongue hangs on the webbing beside
the pillar. The buckle stands on its short stalk at the inboard side of the cushion. The pillar is on the robot's
left, as for a left-hand-drive driver's seat seen from the front. Task (instruction): fasten the seat belt.

How it is done: the tongue is drawn across the seat and pushed straight into the buckle until it clicks. An ELR that is
sensitive to webbing withdrawal locks when the webbing accelerates out of it; FMVSS 209 (49 CFR 571.209), S4.3(j)(2)
(iii): such a retractor "shall not lock before the webbing payout extends to the minimum limit of 51 mm ... when the
retractor is subjected to an acceleration no greater than 0.3 g". A locked belt stops dead; it has to be let back a
little and drawn again, smoothly.

The hand skill: draw the tongue 48 cm across the seat smoothly (no jerk that locks the retractor) against the
retractor's pull, carry it blade down, and push the 3 mm blade straight into the buckle's 4.5 mm slot on a stalk that
gives, past the latch's spring, until it catches; then let go.

World frame (m): x forward from the arm bases, y to the robot's left, z up; the table top is at z TOP (0.7475);
heights below are above the table.
- seat: fixed. Cushion 45 cm wide (y -0.43 to +0.02), its top 4.5 cm above the table, from x 0.06 to 0.40 (the front
  15 cm of a real cushion is left off to clear the arm bases), side bolsters 2 cm higher; backrest 9 cm thick, its
  front at x 0.40, 62 cm tall; head restraint on two posts.
- pillar: fixed, outboard of the seat (y +0.09 to +0.15, x 0.33 to 0.47), 68 cm tall, the retractor housing at its foot,
  the shoulder guide on a bracket at (0.245, 0.03, 0.60); a sill beside the cushion carries the lap anchor at
  (0.245, 0.03, 0.045).
- belt (one articulation): the 48 mm webbing is two straight runs kept taut by the retractor, from the shoulder guide
  to the tongue and from the tongue to the lap anchor. Each run turns freely about its ends (universal joints; the
  lap run's end on the anchor by a ball joint) and lengthens on slides (the shoulder run on one, the lap run on five in
  series that share its length), each pulled back by the retractor spring (4 N/m, preloaded: 2.0-3.5 N along each run
  over the draw). The tongue (40 g) turns freely about its webbing slot. Start: the tongue hangs blade down on the
  vertical webbing at (0.245, 0.03), its slot about 14 cm above the table, its faces toward +-y.
- tongue: plastic head 50 x 13 x 28 mm (x, y, z) with the webbing slot at its top; steel blade 22 mm wide, 3 mm
  thick, 24 mm long with a 12 x 11 mm latch window whose lower edge is 6 mm above the tip.
- buckle (articulation): head 53 x 32 x 50 mm on a stalk that gives (two hinges at its root, 3 N m/rad each,
  +-20 deg) at (0.23, -0.448), its mouth 9.5 cm above the table: a slot 26 x 4.5 mm, 30 mm deep, under a funnel 6 mm
  deep that is 6 mm wider each side (3 mm at the ends). The latch: a sprung pawl (2500 N/m, ramped on top, flat
  underneath) sticks 2.75 mm into the slot, its underside 15 mm below the mouth; the blade's tip pushes it back
  (about 6 N) and it springs into the blade's window once the tongue's head is within 3 mm of the mouth; the webbing's
  pull then holds the window's lower edge against it.

Retractor lock (the inertia reel): every control step (50 ms) the webbing payout (the two runs' lengths less their
start) and its rate are read; if the rate rises by more than LOCK_G (0.7 g: above FMVSS 209's 0.3 g no-lock limit;
makers do not publish the lock point) over a step, from rest or while paying out, the retractor locks: no more webbing
comes out (each slide's outward limit is set where it is) until 5 mm has gone back in, as on a real reel.

Score (end of the run; "latched": the pawl back within 0.8 mm of its rest with the tongue's head within 3.5 mm of the
mouth, so the window sits on the pawl; "let go": every finger point 2 cm or more from the tongue):
- 0.25: the blade's tip has been 10 mm or more inside the mouth, in line with the slot, at some time.
- 0.4: latched at the end, but still held (or moving).
- 0.6: latched, let go and still (under 2 cm/s) for 1 s, but the retractor locked at some time.
- 1.0 (success): latched, let go and still for 1 s, and the retractor never locked.
"""

from __future__ import annotations

import itertools
import math
import os
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from bodykit import Body, BoxShape, CylShape, Link, Linkage, allow_matrix_grips, as_numpy, on_reset, on_step, plate  # noqa: E402
from bodykit import rotation  # noqa: E402
from strapkit import Loop, Part, Strap  # noqa: E402
from yam_arena.demo import Waypoint, grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

allow_matrix_grips()

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
_E = os.environ.get
G = 9.81

# -- layout (x, y, height above the table) -------------------------------------------------------------------------
WY = 0.03  # the webbing's line beside the pillar (the pillar is on the robot's left, +y)
OUT = 1.0  # the outboard direction (toward the pillar) along y
WX = 0.245
DRING = (WX, WY, 0.60)  # shoulder guide
ANCHOR = (WX, WY, 0.045)  # lap anchor
SLOT0 = (WX, WY, 0.150)  # the tongue's webbing slot at the start
CUSH_TOP = 0.045
CUSH_Y = (-0.43, 0.02)
CUSH_X = (0.06, 0.40)
BACK_X = (0.40, 0.49)
BUCKLE = (0.23, -0.448)  # the buckle head's slot centre (x, y)
MOUTH = 0.095  # the buckle mouth's height
ROOT_Z = 0.020  # the stalk's root (in the seat bight)

# webbing and retractor
WEB_W, WEB_T = 0.048, 0.0012
LP_SH = 0.40  # shoulder strip pieces (2 overlap: cover 0.40 to 0.80 m)
LP_LAP = 0.10  # lap strip pieces (6 overlap: cover 0.105 to 0.58 m)
N_LAP = 5
K_RET = float(_E("BELT_K", "4.0"))  # N/m per run
F_PRE = float(_E("BELT_PRE", "2.0"))  # N at the start
D_SLIDE = float(_E("BELT_DAMP", "0.4"))  # N s/m per slide
LOCK_G = float(_E("BELT_LOCK_G", "0.7"))  # webbing acceleration that locks the reel (g)
UNLOCK_BACK = 0.005  # m of webbing let back that frees it
TONGUE_KG = 0.040
SAG = float(_E("BELT_SAG", "0.025"))  # m of extra preload on the shoulder run for the webbing parts' own weight

# tongue (frame at its slot; blade down, faces toward +-y, width along x)
HEAD = (0.050, 0.013, 0.028)
BLADE_W, BLADE_T, BLADE_L = 0.022, 0.003, 0.024
WIN_W = 0.012
WIN_Z = (-0.044, -0.033)  # the window's lower and upper edges below the slot
TIP_Z = -0.050

# buckle
SLOT_X, SLOT_Y, SLOT_D = 0.026, 0.0045, 0.030
HEAD_LO = 0.025  # head bottom above the root
K_PAWL = float(_E("BELT_KPAWL", "2500"))
PAWL_IN = 0.00275  # how far the pawl sticks into the slot
PAWL_BOT = 0.015  # its flat underside below the mouth
STALK_K = 3.0  # N m/rad
FUN_W, FUN_D = 0.006, 0.006  # the mouth's funnel: 6 mm wider each side, 6 mm deep

WEB, DARK, GREY, STEEL, RED, SEAT, TRIM = ((0.11, 0.11, 0.12), (0.08, 0.08, 0.09), (0.35, 0.35, 0.37),
                                           (0.72, 0.73, 0.75), (0.80, 0.10, 0.10), (0.50, 0.50, 0.53),
                                           (0.55, 0.53, 0.50))


def _v(p) -> tuple:
    return tuple(float(x) for x in p)


def _w(p) -> tuple:
    return (float(p[0]), float(p[1]), float(TOP + p[2]))


# -- the seat, pillar and sill (fixed) --------------------------------------------------------------------------------
def _seat() -> Body:
    cy = (CUSH_Y[0] + CUSH_Y[1]) / 2
    wy = CUSH_Y[1] - CUSH_Y[0]
    cx = (CUSH_X[0] + CUSH_X[1]) / 2
    dx = CUSH_X[1] - CUSH_X[0]
    sh = [BoxShape((cx, cy, CUSH_TOP / 2), (dx, wy, CUSH_TOP), SEAT)]
    for y in (CUSH_Y[0] + 0.025, CUSH_Y[1] - 0.025):  # side bolsters
        sh.append(BoxShape((cx, y, CUSH_TOP + 0.01), (dx, 0.05, 0.02), SEAT))
    bx = (BACK_X[0] + BACK_X[1]) / 2
    sh.append(BoxShape((bx, cy, CUSH_TOP + 0.2875), (BACK_X[1] - BACK_X[0], wy, 0.575), SEAT))
    for y in (CUSH_Y[0] + 0.03, CUSH_Y[1] - 0.03):
        sh.append(BoxShape((bx - 0.005, y, CUSH_TOP + 0.30), (0.10, 0.06, 0.52), SEAT))
    for y in (cy - 0.08, cy + 0.08):
        sh.append(CylShape((bx, y, 0.66), 0.006, 0.08, STEEL))
    sh.append(BoxShape((bx, cy, 0.76), (0.08, 0.26, 0.15), SEAT))
    return Body.build("seat", pos=(0.0, 0.0, TOP), mass=20.0, color=SEAT, fixed=True, friction=(0.8, 0.7), shapes=sh)


def _pillar() -> Body:
    o = OUT
    sh = [BoxShape((0.40, WY + o * 0.09, 0.34), (0.14, 0.06, 0.68), TRIM),  # B-pillar trim
          BoxShape((0.40, WY + o * 0.09, 0.06), (0.12, 0.065, 0.12), DARK),  # retractor housing at its foot
          BoxShape((0.33, WY + o * 0.035, 0.02), (0.30, 0.05, 0.04), TRIM),  # sill beside the cushion
          BoxShape((WX, WY + o * 0.015, 0.0425), (0.03, 0.022, 0.005), STEEL),  # lap anchor plate and bolt
          CylShape((WX, WY + o * 0.008, 0.0425), 0.006, 0.006, STEEL, axis="Y"),
          BoxShape(((WX + 0.33) / 2, WY + o * 0.03, 0.615), (0.36 - WX, 0.06, 0.02), TRIM),  # shoulder guide bracket
          BoxShape((WX, WY + o * 0.003, 0.61), (0.06, 0.008, 0.026), DARK, collide=False),  # the guide loop
          BoxShape((0.40, WY + o * 0.058, 0.23), (0.07, 0.004, 0.33), DARK, collide=False)]  # webbing run in the trim
    return Body.build("pillar", pos=(0.0, 0.0, TOP), mass=5.0, color=TRIM, fixed=True, friction=(0.6, 0.5), shapes=sh)


# -- the belt ----------------------------------------------------------------------------------------------------------
def _tongue_shapes() -> list:
    hz = -HEAD[2] / 2 + 0.002
    sh = [BoxShape((0.0, 0.0, hz), HEAD, DARK),
          BoxShape((0.0, 0.0, 0.0), (0.036, 0.0135, 0.004), GREY, collide=False)]  # the slot's lips
    side = (BLADE_W - WIN_W) / 2
    top = -HEAD[2] + 0.002  # the head's bottom
    zc = (top + WIN_Z[1]) / 2
    sh.append(BoxShape((0.0, 0.0, zc), (BLADE_W, BLADE_T, top - WIN_Z[1] + 0.001), STEEL))  # above the window
    for s in (1, -1):
        sh.append(BoxShape((s * (WIN_W + side) / 2, 0.0, (WIN_Z[0] + WIN_Z[1]) / 2),
                           (side, BLADE_T, WIN_Z[1] - WIN_Z[0]), STEEL))
    sh.append(BoxShape((0.0, 0.0, (TIP_Z + WIN_Z[0]) / 2), (BLADE_W, BLADE_T, WIN_Z[0] - TIP_Z), STEEL))  # tip bar
    return sh


def _strip(z0, length, dy) -> BoxShape:
    return BoxShape((0.0, dy, z0 + length / 2), (WEB_W, WEB_T, abs(length)), WEB, collide=False)


def _belt() -> Strap:
    d = _w(DRING)
    s0 = _w(SLOT0)
    a = _w(ANCHOR)
    d_sh = DRING[2] - SLOT0[2]
    d_lap = SLOT0[2] - ANCHOR[2]
    tiny = (2e-6, 2e-6, 2e-6)
    parts = [Part("guide", (BoxShape((0.0, 0.0, 0.0), (0.02, 0.006, 0.006), DARK, collide=False),), 0.1, d),
             Part("sh_x", (), 0.004, d, parent="guide", joint="revolute", axis="X", anchor=d, damping=0.001,
                  armature=1e-4, inertia=tiny),
             Part("sh_y", (_strip(0.0, -LP_SH, 0.0),), 0.004, d, parent="sh_x", joint="revolute", axis="Y", anchor=d,
                  damping=0.001, armature=1e-4, inertia=tiny),
             Part("sh_s", (_strip(0.0, LP_SH, 0.0004),), 0.004, s0, parent="sh_y", joint="prismatic", axis="Z",
                  anchor=s0, limits=(-0.40, 0.02), stiffness=K_RET, damping=D_SLIDE, armature=0.01, inertia=tiny),
             Part("tg_x", (), 0.002, s0, parent="sh_s", joint="revolute", axis="X", anchor=s0, damping=0.0005,
                  armature=1e-5, inertia=(1e-7, 1e-7, 1e-7)),
             Part("tg_y", (), 0.002, s0, parent="tg_x", joint="revolute", axis="Y", anchor=s0, damping=0.0005,
                  armature=1e-5, inertia=(1e-7, 1e-7, 1e-7)),
             Part("tongue", tuple(_tongue_shapes()), TONGUE_KG, s0, parent="tg_y", joint="revolute", axis="Z",
                  anchor=s0, damping=0.0005, armature=1e-5, material=(0.5, 0.4)),
             Part("lp_x", (), 0.002, s0, parent="tongue", joint="revolute", axis="X", anchor=s0, damping=0.001,
                  armature=1e-4, inertia=tiny),
             Part("lp_y", (_strip(0.0, -LP_LAP, -0.0004),), 0.002, s0, parent="lp_x", joint="revolute", axis="Y",
                  anchor=s0, damping=0.001, armature=1e-4, inertia=tiny)]
    prev = "lp_y"
    for i in range(N_LAP):
        name = f"lp_{i + 1}"
        parts.append(Part(name, (_strip(0.0, LP_LAP, 0.0002 * (i % 2)),), 0.002, a, parent=prev,
                          joint="prismatic", axis="Z", anchor=a, limits=(-0.16, 0.02), stiffness=K_RET,
                          damping=D_SLIDE, armature=0.01, inertia=tiny))
        prev = name
    assert d_sh > 0 and d_lap >= LP_LAP
    return Strap.build("belt", parts=parts, loops=[Loop(prev, None, kind="spherical")], self_collision=False,
                       solver_iterations=48, velocity_iterations=8, contact_offset=0.002)


SLIDES = ["sh_s"] + [f"lp_{i + 1}" for i in range(N_LAP)]
SLIDE_LIMITS = {"sh_s": (-0.40, 0.02), **{f"lp_{i + 1}": (-0.16, 0.02) for i in range(N_LAP)}}


# -- the buckle --------------------------------------------------------------------------------------------------------
def _buckle() -> Linkage:
    r = np.array([BUCKLE[0], BUCKLE[1], TOP + ROOT_Z])
    zm = MOUTH - ROOT_Z  # the mouth above the root
    hx0, hx1 = -0.032, 0.021  # head extent along x (the release button at -x)
    hy = 0.016
    sx, sy = SLOT_X / 2, SLOT_Y / 2
    base = [BoxShape((0.0, 0.0, -0.005), (0.04, 0.03, 0.01), DARK)]
    stalk = [BoxShape((0.0, 0.0, HEAD_LO / 2), (0.030, 0.012, HEAD_LO), DARK)]
    head = []
    z_lo = HEAD_LO
    r2 = math.sqrt(0.5)
    # the long walls either side of the slot: their inner FUN_W stop FUN_D under the mouth, where 45 deg funnel faces
    # lead into the 4.5 mm slot; the end walls likewise with a 3 mm funnel
    for s in (1, -1):
        head.append(BoxShape((0.0, s * (sy + FUN_W / 2), (z_lo + zm - FUN_D) / 2), (SLOT_X, FUN_W, zm - FUN_D - z_lo),
                             DARK))
        head.append(BoxShape(((hx0 + hx1) / 2, s * (sy + FUN_W + hy) / 2, (z_lo + zm) / 2),
                             (hx1 - hx0, hy - sy - FUN_W, zm - z_lo), DARK))
        head.append(plate((0.0, s * (sy + 0.0007), zm - FUN_D - 0.0007), (0.0, s * (sy + FUN_W + 0.0007), zm - 0.0007),
                          SLOT_X, 0.002, (0.0, -s * r2, r2), GREY))
    for s, ex in ((1, hx1), (-1, hx0)):
        span = abs(ex) - sx
        head.append(BoxShape((s * (sx + 0.003 + abs(ex)) / 2, 0.0, (z_lo + zm) / 2),
                             (span - 0.003, 2 * (sy + FUN_W), zm - z_lo), DARK))
        head.append(BoxShape((s * (sx + 0.0015), 0.0, (z_lo + zm - 0.003) / 2), (0.003, 2 * (sy + FUN_W), zm - 0.003 - z_lo),
                             DARK))
        head.append(plate((s * (sx + 0.0007), 0.0, zm - 0.0037), (s * (sx + 0.0037), 0.0, zm - 0.0007),
                          2 * (sy + FUN_W), 0.002, (-s * r2, 0.0, r2), GREY))
    head.append(BoxShape((0.0, 0.0, (z_lo + zm - SLOT_D) / 2), (SLOT_X, SLOT_Y, zm - SLOT_D - z_lo), DARK))
    head.append(BoxShape(((hx0 - sx) / 2 - 0.001, 0.0, zm + 0.002), (-sx - hx0 - 0.006, 0.020, 0.004), RED,
                         collide=False))  # release button
    head.append(BoxShape((0.0, 0.0, zm + 0.0002), (SLOT_X + 0.008, 0.0012, 0.0004), (0.02, 0.02, 0.02),
                         collide=False))  # the dark mouth seen from above
    pz = zm - PAWL_BOT
    pawl = [BoxShape((0.0, sy - PAWL_IN + 0.003, pz + 0.0015), (0.008, 0.006, 0.003), STEEL),
            BoxShape((0.0, sy - PAWL_IN + 0.00275, pz + 0.003), (0.008, 0.0039, 0.0039), STEEL,
                     quat_xyzw=(math.sin(math.pi / 8), 0.0, 0.0, math.cos(math.pi / 8)))]
    o = _v(r)
    links = [Link("st_x", tuple(stalk), mass=0.01, anchor=o, axis="X", stiffness=STALK_K, spring_damping=0.03,
                  limits=(-20.0, 20.0)),
             Link("st_y", tuple(head), mass=0.06, anchor=o, axis="Y", parent="st_x", stiffness=STALK_K,
                  spring_damping=0.03, limits=(-20.0, 20.0)),
             Link("pawl", tuple(pawl), mass=0.003, anchor=_v(r + [0.0, sy, pz]), axis="Y", parent="st_y", slide=True,
                  limits=(0.0, 0.004), stiffness=K_PAWL, spring_damping=2.0)]
    return Linkage.build("buckle", pos=o, mass=0.1, color=DARK, floating=False, friction=(0.3, 0.25),
                         shapes=base, links=links, contact_offset=0.001, solver_iterations=48)


SEAT_BODY = _seat()
PILLAR = _pillar()
BELT = _belt()
BUCKLE_LK = _buckle()
OBJECTS = [SEAT_BODY, PILLAR, BELT, BUCKLE_LK]

# -- tracking ----------------------------------------------------------------------------------------------------------
_DEBUG = _E("BELT_DEBUG") == "1"
_DEBUG_EVERY = int(_E("BELT_DEBUG_EVERY", "5"))
CLEAR = 0.020
DT = 0.05
_EP: dict = {}
_IDS: dict = {}
# points on the tongue (its frame): head corners and blade
_TS = np.array([[x, y, z] for x in (-0.025, 0.0, 0.025) for y in (-0.0065, 0.0065) for z in (0.002, -0.012, -0.026)]
               + [[x, 0.0, z] for x in (-0.011, 0.0, 0.011) for z in (-0.03, -0.04, -0.05)])


@on_reset
def _reset() -> None:
    _EP.update(tick=0, step=0, payout=0.0, rate=0.0, rate_prev=0.0, acc=0.0, peak_acc=0.0, locked=False, lock_at=0.0, locks=0,
               entered=False, latched=False, depth=None, inside=1.0, pawl=0.0, near=1.0, still=False, tcp={},
               gaps={}, limits_set=False, slot=np.array(_w(SLOT0)))


_reset()


def _finger_points() -> dict:
    import struct
    import xml.etree.ElementTree as ET

    from yam_arena import YAM_URDF
    from yam_arena.urdf import rpy_matrix

    root = pathlib.Path(YAM_URDF).parent
    out = {}
    for link in ET.parse(YAM_URDF).getroot().iter("link"):
        name = link.get("name")
        if name not in ("tip_left", "tip_right"):
            continue
        pts = []
        for col in link.findall("collision"):
            o = col.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = np.array([float(v) for v in (o.get("rpy") or "0 0 0").split()]) if o is not None else np.zeros(3)
            b = (root / col.find("geometry").find("mesh").get("filename")).read_bytes()
            n = struct.unpack("<I", b[80:84])[0]
            tri = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
            v = tri["v"].reshape(-1, 3).astype(float)
            pts.append(np.unique(np.round(v, 4), axis=0) @ rpy_matrix(rpy).T + xyz)
        out[name] = np.concatenate(pts)
    return out


@on_step
def _track(env) -> None:
    try:
        _track_inner(env)
    except Exception:
        import traceback

        print("BELT_TRACEBACK\n" + traceback.format_exc(), flush=True)
        raise


def _set_limits(asset, lows: dict) -> None:
    import torch

    ids = [_IDS["j"][n] for n in SLIDES]
    lim = torch.tensor([[[lows[n], SLIDE_LIMITS[n][1]] for n in SLIDES]], dtype=torch.float32, device=asset.device)
    asset.write_joint_position_limit_to_sim_index(limits=lim, joint_ids=ids, warn_limit_violation=False)


def _track_inner(env) -> None:
    import torch
    from scipy.spatial import cKDTree

    from yam_arena import ik, station
    from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS

    _EP["tick"] += 1
    base = getattr(env, "unwrapped", env)
    origin = as_numpy(base.scene.env_origins[0])
    if "fingers" not in _IDS:
        _IDS["fingers"] = _finger_points()
    fingers = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        if side not in _IDS:
            _IDS[side] = list(robot.find_joints(ARM_JOINTS, preserve_order=True)[0])
            _IDS[side + "_f"] = list(robot.find_joints(FINGER_JOINTS, preserve_order=True)[0])
            _IDS[side + "_tips"] = [robot.body_names.index(n) for n in ("tip_left", "tip_right")]
        q = as_numpy(robot.data.joint_pos)[0]
        _EP["tcp"][side] = np.asarray(ik.tcp_pose(q[_IDS[side]], station.BASES[side])[0], float)
        _EP["gaps"][side] = float(-q[_IDS[side + "_f"]].sum())
        poses = as_numpy(robot.data.body_link_pose_w)[0]
        fingers[side] = np.concatenate([_IDS["fingers"][n] @ rotation(poses[i][3:7]).T + poses[i][:3] - origin
                                        for n, i in zip(("tip_left", "tip_right"), _IDS[side + "_tips"])])
    belt = base.scene["belt"]
    buck = base.scene["buckle"]
    if "j" not in _IDS:
        jn, bn = list(belt.joint_names), list(belt.body_names)
        _IDS["j"] = {n: jn.index(f"{n}_joint") for n in SLIDES}
        _IDS["tongue"] = bn.index("tongue")
        _IDS["head"] = list(buck.body_names).index("st_y")
        _IDS["pawl"] = list(buck.joint_names).index("pawl_turn")
    # the retractor spring's preload: each slide is pulled back toward a rest point beyond its start
    if not _EP["limits_set"]:
        ids = [_IDS["j"][n] for n in SLIDES]
        tgt = [F_PRE / K_RET + (TONGUE_KG * G / K_RET + SAG if n == "sh_s" else 0.0) for n in SLIDES]
        belt.set_joint_position_target_index(target=torch.tensor([tgt], dtype=torch.float32, device=belt.device),
                                             joint_ids=ids)
        _set_limits(belt, {n: SLIDE_LIMITS[n][0] for n in SLIDES})
        _EP["limits_set"] = True
    q = as_numpy(belt.data.joint_pos)[0]
    qd = as_numpy(belt.data.joint_vel)[0]
    payout = -sum(q[_IDS["j"][n]] for n in SLIDES)
    rate = -sum(qd[_IDS["j"][n]] for n in SLIDES)
    acc = (rate - _EP["rate"]) / DT if _EP["tick"] > 1 else 0.0
    _EP.update(payout=payout, rate=rate, acc=acc, rate_prev=_EP["rate"])
    if _EP["tick"] > 5:
        _EP["peak_acc"] = max(_EP["peak_acc"], acc)
    # a lock: the webbing accelerating outward faster than LOCK_G (from rest or while paying out; not the rebound of
    # a belt that has just whipped back onto its stop)
    if (not _EP["locked"] and _EP["tick"] > 5 and rate > 0.0 and _EP["rate_prev"] > -0.05
            and acc > LOCK_G * G):
        _EP.update(locked=True, lock_at=payout, locks=_EP["locks"] + 1)
        _set_limits(belt, {n: float(q[_IDS["j"][n]]) - 0.0005 for n in SLIDES})
    elif _EP["locked"] and payout <= _EP["lock_at"] - UNLOCK_BACK:
        _EP["locked"] = False
        _set_limits(belt, {n: SLIDE_LIMITS[n][0] for n in SLIDES})
    bp = as_numpy(belt.data.body_link_pose_w)[0]
    bv = as_numpy(belt.data.body_link_vel_w)[0] if hasattr(belt.data, "body_link_vel_w") else None
    tp = bp[_IDS["tongue"]]
    t_pos, t_rot = tp[:3] - origin, rotation(tp[3:7])
    hp = as_numpy(buck.data.body_link_pose_w)[0][_IDS["head"]]
    h_pos, h_rot = hp[:3] - origin, rotation(hp[3:7])
    # the tongue's slot and blade tip in the buckle head's frame (origin: the stalk root, mouth at zm)
    zm = MOUTH - ROOT_Z
    slot_h = h_rot.T @ (t_pos - h_pos)
    tip_h = h_rot.T @ (t_pos + t_rot @ np.array([0.0, 0.0, TIP_Z]) - h_pos)
    pawl = float(as_numpy(buck.data.joint_pos)[0][_IDS["pawl"]])
    in_line = abs(tip_h[0]) < 0.006 and abs(tip_h[1]) < 0.004
    depth = zm - tip_h[2]  # the blade tip's depth inside the mouth
    if in_line and depth >= 0.010:
        _EP["entered"] = True
    # latched: the head's bottom within 3.5 mm of the mouth (the window's lower edge below the pawl) and the pawl back
    head_gap = slot_h[2] - HEAD[2] + 0.002 - zm
    latched = bool(in_line and head_gap <= 0.0035 and pawl <= 0.0008)
    pts = t_pos + _TS @ t_rot.T
    tree = cKDTree(pts)
    near = min(float(tree.query(fp, k=1)[0].min()) for fp in fingers.values())
    tv = float(np.linalg.norm(bv[_IDS["tongue"]][:3])) if bv is not None else 0.0
    _EP["q"] = [float(q[_IDS["j"][n]]) for n in SLIDES]
    _EP["tip_h"], _EP["head"] = tip_h, h_pos
    _EP["guide"] = bp[list(belt.body_names).index("guide")][:3] - origin
    _EP.update(latched=latched, depth=depth, head_gap=head_gap, pawl=pawl, near=near, still=tv < 0.02,
               slot=t_pos, tv=tv, tilt=math.degrees(math.acos(max(-1.0, min(1.0, float(t_rot[2, 2]))))))


def _tally(o) -> dict:
    done = bool(_EP["latched"] and _EP["near"] >= CLEAR and _EP["still"])
    return dict(done=done)


def _debug(t) -> None:
    _EP["step"] += 1
    if (_EP["step"] - 1) % _DEBUG_EVERY:
        return
    s = _EP["slot"]
    print(f"DBG step={_EP['step']} payout={_EP['payout'] * 1000:.0f}mm rate={_EP['rate']:.3f} "
          f"acc={_EP['acc'] / G:+.2f}g peak={_EP['peak_acc'] / G:.2f}g locked={int(_EP['locked'])} locks={_EP['locks']} "
          f"slot=({s[0]:.3f},{s[1]:.3f},{(s[2] - TOP) * 1000:.1f}mm) tilt={_EP.get('tilt', 0):.0f} "
          f"depth={(_EP['depth'] or 0) * 1000:.1f}mm headgap={_EP.get('head_gap', 0) * 1000:.1f}mm "
          f"pawl={_EP['pawl'] * 1000:.2f}mm entered={int(_EP['entered'])} latched={int(_EP['latched'])} "
          f"tip_h={np.round(np.array(_EP.get('tip_h', np.zeros(3))) * 1000, 1).tolist()} head={np.round(_EP.get('head', np.zeros(3)), 3).tolist()} "
          f"near={_EP['near'] * 1000:.0f}mm tv={_EP.get('tv', 0):.3f} done={int(t['done'])} | "
          + " ".join(f"{k} tcp=({v[0]:.3f},{v[1]:.3f},{(v[2] - TOP) * 1000:.1f}mm) gap={_EP['gaps'][k] * 1000:.0f}"
                     for k, v in _EP["tcp"].items()), flush=True)


def score(objects) -> float:
    t = _tally(objects)
    if _DEBUG:
        _debug(t)
    best = 0.25 if _EP["entered"] else 0.0
    if _EP["latched"]:
        best = 0.4
        if t["done"]:
            best = 0.6 if _EP["locks"] else 1.0
    return best


def success(objects) -> bool:
    return bool(_tally(objects)["done"] and not _EP["locks"])


# -- scripted demo -----------------------------------------------------------------------------------------------------
# Open loop, from the known layout, right arm only (the left arm stays at its ready pose). The right gripper comes
# down over the hanging tongue, its fingers closing across the head's 13 mm thickness (along y), and pinches it; it
# draws the tongue across the seat to above the buckle in 5 s on a smooth (minimum-jerk) path, so the webbing never
# accelerates out of the retractor at more than about 0.2 g, with the blade down. Under the webbing's pull the arm
# stops short of where it is sent (about 2.4 cm toward the pillar) and the tongue turns a little in the grip, so the hand
# is sent past the buckle and turned 4 deg to bring the blade over the mouth; it lowers the blade into the mouth and
# pushes 10 mm past the latch point, holds, lets go, rises and goes back to the ready pose.
# BELT_JERK=1 (the wrong way): the same, but the first 15 cm of the draw is a jerk in 0.1 s, as when a belt is
# snatched: the retractor locks, the webbing stops dead and pulls the tongue out of the fingers, and it swings back to
# the pillar; the rest of the demo goes on with an empty hand.
OPEN_C = -0.047
JERK = _E("BELT_JERK") == "1"
R_Y = grip_rotation(0.0)  # straight down, the fingers closing along y
P = {k: float(_E("BELT_" + k.upper(), v)) for k, v in dict(
    open="-0.018", pinch="-0.0045", grip_dz="-0.010", draw_s="5.0", push="0.010", lift="0.050",
    aim_dx="0.004", aim_dy="-0.023", over_dz="0.016", roll="-4.0").items()}


def _good_ik(arm: str, pos, rot):
    from yam_arena import ik, station

    best = None
    b = station.BASES[arm]
    j1 = math.atan2(float(pos[1]) - b[1], float(pos[0]) - b[0])
    for sd in itertools.product((j1,), (0.8, 1.3), (0.6, 1.2), (-0.8, 0.8), (0.0,), (-1.8, -0.9, 0.0, 0.9, 1.8)):
        q, pe, re = ik.solve(np.asarray(pos, float), ik.matrix_to_quat_xyzw(rot), np.asarray(sd, float),
                             base_pos=station.BASES[arm], iters=150)
        if pe < 0.001 and re < math.radians(0.5):
            room = float(np.minimum(q - ik.LOWER, ik.UPPER - q).min())
            if best is None or room > best[1]:
                best = (q, room)
                if room > 0.3:
                    break
    if best is None:
        raise ValueError(f"no IK solution for the {arm} arm at {pos}")
    return best[0]


def _joint_path(arm: str, q0, q1, finger: float, label: str, seconds: float, steps: int = 15) -> list:
    from yam_arena import ik, station

    out = []
    for k in range(1, steps + 1):
        p, r = ik.tcp_pose(q0 + (q1 - q0) * k / steps, station.BASES[arm])
        out.append(Waypoint(seconds / steps, tuple(float(v) for v in p), finger, label, r))
    return out


def _lead_in(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, np.array(READY, float), _good_ik(arm, pos, rot), finger, label, seconds)


def _lead_out(arm: str, pos, rot, finger: float, label: str, seconds: float = 3.0) -> list:
    from yam_arena.constants import READY

    return _joint_path(arm, _good_ik(arm, pos, rot), np.array(READY, float), finger, label, seconds)


def _wp(t, p, finger, label, rot=R_Y) -> Waypoint:
    return Waypoint(t, tuple(float(v) for v in p), finger, label, rot)


def _ready(arm: str):
    from yam_arena import ik, station
    from yam_arena.constants import READY

    return ik.tcp_pose(np.array(READY, float), station.BASES[arm])


def _dur(wps) -> float:
    return float(sum(w.duration for w in wps))


def _roll(deg: float) -> np.ndarray:
    """The straight-down grip turned ``deg`` about the world x axis (the tongue's tip toward +y for deg > 0)."""
    t = math.radians(deg)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, math.cos(t), -math.sin(t)], [0.0, math.sin(t), math.cos(t)]])
    return rx @ R_Y


def _minjerk(p0, p1, seconds, finger, label, n=40, roll=(0.0, 0.0)) -> list:
    out = []
    for k in range(1, n + 1):
        s = k / n
        f = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        out.append(_wp(seconds / n, p0 + (p1 - p0) * f, finger, label, _roll(roll[0] + (roll[1] - roll[0]) * f)))
    return out


def _right() -> list:
    g = np.array(_w(SLOT0)) + [0.0, 0.0, P["grip_dz"]]  # TCP holding the tongue's head
    above = g + [0.0, 0.0, 0.02]
    # aimed past the buckle along +y (and x): under the webbing's pull the arm stops about that much short
    bx, by = BUCKLE[0] + P["aim_dx"], BUCKLE[1] + P["aim_dy"]
    over = np.array([bx, by, TOP + MOUTH - TIP_Z + P["over_dz"] + P["grip_dz"]])  # blade tip just over the mouth
    seat = np.array([bx, by, TOP + MOUTH + HEAD[2] - 0.002 + P["grip_dz"] - P["push"]])
    mid = np.array([(g[0] + over[0]) / 2, (g[1] + over[1]) / 2, max(g[2], over[2]) + 0.01])
    out = _lead_in("right", above, R_Y, P["open"], "over the hanging tongue, fingers open")
    out += [_wp(1.5, g, P["open"], "fingers either side of the tongue's head"),
            _wp(1.0, g, P["pinch"], "pinch the head"),
            _wp(0.5, g, P["pinch"], "settle")]
    start = g
    if JERK:
        d = mid - g
        j = g + d / np.linalg.norm(d) * 0.15
        out += [_wp(0.1, j, P["pinch"], "snatch the belt out")]
        start = j
    out += _minjerk(start, mid, P["draw_s"] / 2 if not JERK else P["draw_s"] / 2.5, P["pinch"],
                    "draw the tongue across the seat", 20)
    rl = P["roll"]  # the hand turns so the tongue's blade, pulled askew by the webbing, points straight down
    out += _minjerk(mid, over, P["draw_s"] / 2, P["pinch"], "to over the buckle", 20, roll=(0.0, rl))
    out += [_wp(0.5, over, P["pinch"], "settle over the mouth", _roll(rl))]
    out += _minjerk(over, seat, 2.0, P["pinch"], "push the blade into the buckle until it latches", 10, roll=(rl, rl))
    out += [_wp(1.0, seat, P["pinch"], "hold", _roll(rl)),
            _wp(1.0, seat, P["open"], "let go", _roll(rl)),
            _wp(1.2, seat + [0.0, 0.04, P["lift"]], P["open"], "rise clear")]
    out += _lead_out("right", seat + [0.0, 0.04, P["lift"]], R_Y, OPEN_C, "back to the ready pose")
    return out


def demo():
    right = _right()
    rp, rq = _ready("right")
    right += [_wp(3.0, rp, OPEN_C, "stay clear", rq)]
    lp, lq = _ready("left")
    left = [_wp(_dur(right), lp, OPEN_C, "stay at the ready pose", lq)]
    return {"left": left, "right": right}
