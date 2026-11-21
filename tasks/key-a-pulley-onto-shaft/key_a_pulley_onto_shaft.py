"""Fitting a keyed pulley to a shaft on an assembly bench and locking it with its set screw.

A 20 mm steel shaft is clamped horizontally on a support block, its axis 40 mm above the bench and pointing at the
robot, with a shoulder collar at the back. By the shoulder the top of the shaft has a keyseat for a 6 x 6 x 25 mm
round-ended parallel key (DIN 6885 form A), which lies loose on a mat. A 60 mm aluminium pulley (12 mm wide rim,
36 mm boss) is started on the free end of the shaft; its bore has a 6.05 mm keyway (6 mm D10), turned 45 deg away
from the key's line. An M6 x 1.0 socket set screw sits in the boss over the keyway, its point flush with the keyway
roof, 0.3 mm clear of the key. A stubby 3 mm hex driver (20 mm square handle) stands in a rack on the bench.

The job: press the key into the keyseat (an interference fit: it needs about 6 N), turn the pulley so its keyway
lines up with the key, slide it over the key up to the shoulder, put the hex driver in the set screw and tighten it
onto the key to at least 0.4 N*m, take the driver out and put it down. The bench then pulls the pulley off the
shoulder with 20 N for 1 s; a seated screw holds it.

Mechanism parts come from mechkit.py (in this folder), which adds joints to the stock runtime in memory. Every joint
is authored when the scene is built; the ones that act later (key weld, hex engagement, clamp hold, pull) start
disabled and are only switched on.

Run it:  inspect-robots run --embodiment yam_arena -E task=tasks/key-a-pulley-onto-shaft/key_a_pulley_onto_shaft.py \\
           -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 --max-steps 1800 \\
           --instruction "<the instruction in task.json>" --scorer task_score ...
"""

import math
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import mechkit as mk  # noqa: E402

from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]

STEEL = (0.62, 0.63, 0.66)
KEYC = (0.30, 0.30, 0.32)
ALU = (0.80, 0.80, 0.82)
DARK = (0.14, 0.14, 0.15)
BLOCK = (0.20, 0.30, 0.55)
SCREW = (0.10, 0.10, 0.11)
MAT = (0.35, 0.50, 0.38)
HANDLE = (0.85, 0.35, 0.10)
RACK = (0.45, 0.33, 0.20)


def _rz(deg):
    return mk.mat_to_quat(mk.axis_angle((0, 0, 1), math.radians(deg)))


# -- shaft (axis along x, y = 0) ---------------------------------------------------------------------------------
AXZ = TOP + 0.040  # shaft centreline
R_SHAFT = 0.010
X_END = 0.205  # free end
X_SH = 0.335  # shoulder face
X_CL = 0.375  # end over the support block
KEY_L, KEY_W, KEY_H = 0.025, 0.006, 0.006  # 6 x 6 x 25 mm, round ends
KEY_XC = X_SH - 0.015  # keyseat centre
SEAT_DEPTH = 0.0035  # keyseat depth t1 for a 6 x 6 key
SLOT_L, SLOT_HW = 0.0256, 0.00305  # keyseat 6.10 mm wide in sim (0.10 mm clearance), 0.6 mm longer than the key
SLOT0, SLOT1 = KEY_XC - SLOT_L / 2, KEY_XC + SLOT_L / 2
SLABS = [(-0.0099, -0.0085), (-0.0085, -0.007), (-0.007, -0.005), (-0.005, 0.005), (0.005, 0.007), (0.007, 0.0085),
         (0.0085, 0.0099)]  # stacked slabs inside the 20 mm circle
SHAFT_SLICK = []  # keyseat wall boxes (steel on steel)


def _shaft_boxes():
    out = []
    for x0, x1, keyed in ((X_END, SLOT0, False), (SLOT0, SLOT1, True), (SLOT1, X_CL, False)):
        L, xc = x1 - x0, (x0 + x1) / 2
        for a, b in SLABS:
            w = 2 * math.sqrt(R_SHAFT ** 2 - max(a * a, b * b))
            zc = AXZ + (a + b) / 2
            if keyed and a >= 0.0085:
                continue  # the keyseat opens the top
            if keyed and a >= 0.005:  # split around the slot
                hw = w / 2
                side = hw - SLOT_HW
                for s in (1, -1):
                    SHAFT_SLICK.append(len(out))
                    out.append(mk.box((L, side, b - a), at=(xc, s * (SLOT_HW + side / 2), zc), color=STEEL))
                continue
            out.append(mk.box((L, w, b - a), at=(xc, 0, zc), color=STEEL))
    # shoulder collar, 30 mm, 8 mm long
    side = 2 * 0.015 / math.sqrt(2) * 1.08
    for i in range(3):
        out.append(mk.box((0.008, side, side), at=(X_SH + 0.0045, 0, AXZ), color=STEEL,
                          quat=mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 2 * i / 3))))
    return out


kit = mk.Kit()
kit.part("shaft", (0, 0, 0), _shaft_boxes(), mass=None, color=STEEL)

# Interference fit: the keyseat floor is a pad that the key pushes 1 mm down against 6 N of Coulomb friction.
SEAT_TOP0 = AXZ + R_SHAFT - SEAT_DEPTH + 0.001  # pad top at the start (the key enters 1 mm below the walls' tops)
SEAT_TRAVEL = 0.001
PRESS_N = 6.0  # interference: the key needs this much push to go home
kit.part("seat_pad", (KEY_XC, 0, SEAT_TOP0 - 0.00075), [mk.box((SLOT_L - 0.0002, 2 * SLOT_HW - 0.0001, 0.0015))],
         mass=0.02, color=STEEL)
kit.slider("seat_press", "shaft", "seat_pad", axis=(0, 0, -1), lower=0.0, upper=SEAT_TRAVEL, friction=PRESS_N,
           damping=1.0)

# The key: loose on the bench mat, long side along x. Round ends (12-sided), 1 mm chamfers along the bottom edges.
KEY_START = (0.270, -0.130, TOP + 0.001 + KEY_H / 2)
_c45 = mk.mat_to_quat(mk.axis_angle((1, 0, 0), math.pi / 4))
_KS = KEY_L - KEY_W  # straight length between the end centres
_KEY = [mk.box((_KS, KEY_W, KEY_H - 0.001), at=(0, 0, 0.0005)), mk.box((_KS, KEY_W - 0.002, 0.001), at=(0, 0, -0.0025)),
        *[mk.box((_KS, 0.001414, 0.001414), at=(0, s * (KEY_W / 2 - 0.001), -0.002), quat=_c45) for s in (1, -1)]]
for _sx in (1, -1):
    _KEY += [mk.box((KEY_W, KEY_W * math.tan(math.radians(15.0)), KEY_H - 0.001), at=(_sx * _KS / 2, 0, 0.0005),
                    quat=_rz(30.0 * i)) for i in range(6)]
kit.part("key", KEY_START, _KEY, mass=0.02, color=KEYC)
KEY_SEATED = np.array([KEY_XC, 0.0, AXZ + R_SHAFT - SEAT_DEPTH + KEY_H / 2])  # bottom 6.5 mm, top 12.5 mm above the axis
_T_seat = np.eye(4)
_T_seat[:3, 3] = KEY_SEATED
kit.catch("key_seat", "key", _T_seat)  # the interference fit holds it once fully in

# -- pulley ---------------------------------------------------------------------------------------------------
HUB_L, BOSS_L = 0.036, 0.024  # boss toward the robot (-x), rim at the shoulder side
R_BORE, R_KEYWAY, R_BOSS, R_RIM = 0.01001, 0.0128, 0.018, 0.030  # keyway roof 0.3 mm above the seated key
KW_HW = 0.003025  # keyway half-width: 6.05 mm for the 6.00 mm key
HUB_X0 = X_END - 0.004  # hub's front face at the start
TRAVEL = (X_SH - HUB_L) - HUB_X0  # to the shoulder
SCREW_DX = 0.012  # set screw, from the hub's front face
HUB_START_DEG = 45.0
HOLE_HX = 0.0031  # the tapped hole is a 6.2 mm pocket through the boss
HUB_SLICK = []  # keyway wall boxes (aluminium on steel)


def _ring(x0, x1, ri, ro, n, color, skip_deg=0.0, hole=None, roof=False):
    """A ring of n boxes round the shaft axis. roof: the boxes beside the top one start further out, so that the
    keyway under them is a clean rectangle up to its roof at `ri` (their inner faces would cut its corners)."""
    out = []
    w = 2 * ro * math.tan(math.pi / n)
    for i in range(n):
        a = 2 * math.pi * i / n
        if skip_deg and abs((math.degrees(a) + 180) % 360 - 180) < skip_deg:
            continue
        rin = ri
        if roof and i != 0:  # keep clear of the keyway's top corners (0.3 mm beyond its walls, 0.1 mm above its roof)
            rin = max(ri, max((s * (KW_HW + 0.0003)) * -math.sin(a) + (ri + 0.0001) * math.cos(a) for s in (1, -1)))
        r = (rin + ro) / 2
        spans = [(x0, x1)]
        if hole is not None and i == 0:
            spans = [(x0, hole - HOLE_HX), (hole + HOLE_HX, x1)]
        for xa, xb in spans:
            out.append(mk.box((xb - xa, w, ro - rin), at=((xa + xb) / 2, -r * math.sin(a), r * math.cos(a)), color=color,
                              quat=mk.mat_to_quat(mk.axis_angle((1, 0, 0), a))))
    return out


def _hub_boxes():
    b = _ring(0, HUB_L, R_BORE, R_KEYWAY, 24, ALU, skip_deg=20.0)  # bore, keyway at the top (phase 0)
    for s in (1, -1):  # keyway walls
        HUB_SLICK.append(len(b))
        b.append(mk.box((HUB_L, 0.002, 0.0032), at=(HUB_L / 2, s * (KW_HW + 0.001), 0.0112), color=ALU))
    b += _ring(0, BOSS_L, R_KEYWAY, R_BOSS, 16, ALU, hole=SCREW_DX, roof=True)
    b += _ring(BOSS_L, HUB_L, R_KEYWAY, R_RIM, 20, ALU, roof=True)
    b.append(mk.box((0.001, 0.004, 0.003), at=(-0.0005, 0, R_BOSS - 0.0015), color=DARK))  # keyway mark
    return b


# The pulley is one body on a cylindrical joint about the shaft axis: it slides 0..TRAVEL and turns freely. mechkit
# declares it as a slider (so the readouts and the self-test work) and the joint is re-authored as a D6 (x free
# within the stroke, rotation about x free, the rest locked) right after mechkit builds the scene; see _author.
_R45 = mk.axis_angle((1, 0, 0), math.radians(HUB_START_DEG))
HUB_Q0 = mk.mat_to_quat(_R45)
HUB_P0 = np.array([HUB_X0, 0, AXZ])
kit.part("hub", tuple(HUB_P0), _hub_boxes(), mass=0.15, color=ALU, quat=HUB_Q0)
kit.slider("hub_slide", "shaft", "hub", axis=(1, 0, 0), lower=0.0, upper=TRAVEL)
SLIDE_FRICTION_N = 2.0  # a slip fit
TURN_FRICTION_NM = 0.01

# -- set screw: M6 x 1.0, 5 mm long, 3 mm hex socket, flat point, in the boss over the keyway --------------------
PITCH = 0.001  # m per turn
SCREW_L, SCREW_AF = 0.005, 0.0042
SCREW_TOP_R = R_KEYWAY + SCREW_L  # its point starts flush with the keyway roof, its top 0.2 mm below the boss surface
SCREW_GAP = (SCREW_TOP_R - SCREW_L) - (R_SHAFT - SEAT_DEPTH + KEY_H)  # point to key top at the start: 0.3 mm
SOCKET_D = 0.003
X_MIN, X_MAX = -0.0003, 0.0015  # thread travel of the point from its start (m)
S_MIN, S_MAX = -20.0, 170.0  # deg
SCREW_LOCAL = np.array([SCREW_DX, 0.0, SCREW_TOP_R])  # top-face centre, in the hub frame
SCREW_AT = tuple(HUB_P0 + _R45 @ SCREW_LOCAL)
SCREW_AXIS = tuple(_R45 @ np.array([0.0, 0.0, -1.0]))  # into the hub: + turn is clockwise seen from outside
kit.part("screw", SCREW_AT, [mk.box((SCREW_AF, SCREW_AF, SCREW_L), at=(0, 0, -SCREW_L / 2), color=SCREW, quat=_rz(45.0 * i))
                             for i in range(2)], mass=0.02, color=SCREW, quat=HUB_Q0)
# Declared as a hinge for the readouts; re-authored at scene build as a D6: turn about the screw axis plus travel
# along it, the travel driven to PITCH x turns (the thread).
kit.hinge("screw_turn", "hub", "screw", anchor=SCREW_AT, axis=SCREW_AXIS, lower_deg=S_MIN, upper_deg=S_MAX)

K_THREAD, THREAD_N = 1.0e5, 0.6  # N/m, N: the drive that carries the screw along its thread, and the most it pushes
K_CLAMP = 4.0e6  # N/m: clamp force per metre the thread is run past contact (screw, key and boss in series)
DEADBAND = 0.00001  # m the point must lag its thread before that counts as contact
C_THREAD = 0.2 * 0.006  # m: tightening torque = 0.2 x d x clamp force (nut factor 0.2, d = 6 mm)
T_RUN = 0.02  # N*m running torque of the free thread
K_ROT = 6.0  # N*m/rad: torsional give of screw and key before the thread slips
T_SEAT = 0.4  # N*m: the specified seating torque
MU_HOLD = 0.3  # the pulley's hold on the shaft per newton of clamp force (two faces at 0.15)
K_HOLD = 5.0e5  # N/m: stiffness of that hold before it slips
PULL_N, PULL_S, PULL_SLIP = 20.0, 1.0, 0.0002  # the bench's proof pull: 20 N for 1 s, the pulley may give 0.2 mm
K_PULL = 400.0  # N/m: the pull is a long soft spring
PULL_STOP = 0.003  # a pulley that lets go is pulled this far off the shoulder

# -- hex driver: 3 mm hex tip, 8 mm shank, 20 mm square handle; stands in a rack ------------------------------------
BIT_L, BIT_AF = 0.0029, 0.003
SHANK_W, SHANK_L = 0.008, 0.017
GRIP_W, GRIP_H = 0.020, 0.026
TOOL_H = BIT_L + SHANK_L + GRIP_H
RACK_XY = (0.260, 0.130)
RACK_W, RACK_H, RACK_HOLE = 0.036, 0.022, 0.012
_bs = BIT_AF / math.sqrt(3.0)
_TOOL = [*[mk.box((BIT_AF, _bs, BIT_L), at=(0, 0, BIT_L / 2), color=DARK, quat=_rz(60.0 * i)) for i in range(3)],
         mk.box((SHANK_W, SHANK_W, SHANK_L), at=(0, 0, BIT_L + SHANK_L / 2), color=STEEL),
         mk.box((GRIP_W, GRIP_W, GRIP_H), at=(0, 0, BIT_L + SHANK_L + GRIP_H / 2), color=HANDLE)]
TOOL_START = (RACK_XY[0], RACK_XY[1], TOP + RACK_H - BIT_L - SHANK_L)  # frame: the hex tip's end, z up the driver
kit.part("tool", TOOL_START, _TOOL, mass=0.06, color=HANDLE)
_rs = (RACK_W - RACK_HOLE) / 2
kit.part("rack", (RACK_XY[0], RACK_XY[1], TOP),
         [*[mk.box((RACK_W, _rs, RACK_H), at=(0, s * (RACK_HOLE + _rs) / 2, RACK_H / 2)) for s in (1, -1)],
          *[mk.box((_rs, RACK_HOLE, RACK_H), at=(s * (RACK_HOLE + _rs) / 2, 0, RACK_H / 2)) for s in (1, -1)]],
         mass=None, color=RACK)
ENGAGE_MM, ENGAGE_TILT, ENGAGE_PHASE, ENGAGE_DEPTH = 0.0005, 4.0, 10.0, 0.001  # hex finds the socket within these
BIT_OUT = 0.0003  # the hex has left the socket once its end is within 0.3 mm of the screw top

_STICK_M, _STICK_DEG = 0.0005, 0.5
_author0 = mk._author_all
_X = {}  # extra joint paths by name


def _d6(stage, path, b0, b1, lp0, lr0, lp1, lr1, locked=(), limits=None, drives=None, enabled=True):
    from pxr import UsdPhysics

    j = UsdPhysics.Joint.Define(stage, path)
    if b0:
        j.CreateBody0Rel().SetTargets(b0)
    j.CreateBody1Rel().SetTargets(b1)
    j.CreateLocalPos0Attr().Set(lp0)
    j.CreateLocalRot0Attr().Set(lr0)
    j.CreateLocalPos1Attr().Set(lp1)
    j.CreateLocalRot1Attr().Set(lr1)
    j.CreateCollisionEnabledAttr().Set(False)
    j.CreateExcludeFromArticulationAttr().Set(True)
    j.CreateJointEnabledAttr().Set(bool(enabled))
    prim = j.GetPrim()
    for dof in locked:
        lim = UsdPhysics.LimitAPI.Apply(prim, dof)
        lim.CreateLowAttr().Set(1.0)
        lim.CreateHighAttr().Set(-1.0)  # low > high: locked
    for dof, (lo, hi) in (limits or {}).items():
        lim = UsdPhysics.LimitAPI.Apply(prim, dof)
        lim.CreateLowAttr().Set(float(lo))
        lim.CreateHighAttr().Set(float(hi))
    for dof, (k, d, target, *cap) in (drives or {}).items():
        dr = UsdPhysics.DriveAPI.Apply(prim, dof)
        dr.CreateTypeAttr().Set("force")
        if cap:
            dr.CreateMaxForceAttr().Set(float(cap[0]))
        dr.CreateStiffnessAttr().Set(float(k))
        dr.CreateDampingAttr().Set(float(d))
        dr.CreateTargetPositionAttr().Set(float(target))
        dr.CreateTargetVelocityAttr().Set(0.0)
    return j


def _frames(stage, path):
    from pxr import UsdPhysics

    old = UsdPhysics.Joint(stage.GetPrimAtPath(path))
    return (old.GetBody0Rel().GetTargets(), old.GetBody1Rel().GetTargets(), old.GetLocalPos0Attr().Get(),
            old.GetLocalRot0Attr().Get(), old.GetLocalPos1Attr().Get(), old.GetLocalRot1Attr().Get())


def _slick(stage, prim_path, mu):
    """Bare-metal friction on one box (the rest of the scene keeps the stock rubber-like 1.2)."""
    from pxr import Sdf, Usd, UsdPhysics

    n = 0
    for p in Usd.PrimRange(stage.GetPrimAtPath(prim_path)):
        if p.HasAPI(UsdPhysics.MaterialAPI):
            n += 1
            m = UsdPhysics.MaterialAPI(p)
            m.CreateStaticFrictionAttr().Set(float(mu))
            m.CreateDynamicFrictionAttr().Set(float(mu))
            p.AddAppliedSchema("PhysxMaterialAPI")
            p.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set("min")
    return n


def _author(stage, env_path):
    """Scene build only: the pulley's and the screw's joints become D6 joints, and the joints that act later are
    authored disabled (hex engagement at the six socket positions, the clamp's hold, the proof pull)."""
    _author0(stage, env_path)
    hub_path = mk._SIM["joint_paths"].get((kit.id, "hub_slide"))
    screw_path = mk._SIM["joint_paths"].get((kit.id, "screw_turn"))
    paths = mk._SIM["paths"]
    if not hub_path or not screw_path or "tool" not in paths:
        return
    from pxr import Gf, Sdf, UsdPhysics

    # pulley: slide 0..TRAVEL and turn, with a small friction drive on each
    b0, b1, lp0, lr0, lp1, lr1 = _frames(stage, hub_path)
    stage.RemovePrim(hub_path)
    _d6(stage, hub_path, b0, b1, lp0, lr0, lp1, lr1, locked=("transY", "transZ", "rotY", "rotZ"),
        limits={"transX": (0.0, TRAVEL)},
        drives={"transX": (SLIDE_FRICTION_N / _STICK_M, 2.0, 0.0),
                "rotX": (TURN_FRICTION_NM / math.radians(_STICK_DEG) * math.pi / 180, 0.0005, 0.0)})
    root = hub_path.rsplit("/", 1)[0]
    # the clamp's hold on the shaft and the bench's proof pull: same frames, every axis free, one spring each
    _X["hold"] = f"{root}/hub_hold"
    _d6(stage, _X["hold"], b0, b1, lp0, lr0, lp1, lr1, drives={"transX": (K_HOLD, 300.0, 0.0)}, enabled=False)
    _X["pull"] = f"{root}/hub_pull"
    _d6(stage, _X["pull"], b0, b1, lp0, lr0, lp1, lr1,
        drives={"transX": (K_PULL, 20.0, TRAVEL - PULL_N / K_PULL)}, enabled=False)
    # set screw: turn about its axis, travel along it on the thread drive
    s0, s1, sp0, sr0, sp1, sr1 = _frames(stage, screw_path)
    stage.RemovePrim(screw_path)
    _d6(stage, screw_path, s0, s1, sp0, sr0, sp1, sr1, locked=("transY", "transZ", "rotY", "rotZ"),
        limits={"transX": (X_MIN, X_MAX), "rotX": (S_MIN, S_MAX)},
        drives={"transX": (K_THREAD, 50.0, 0.0, THREAD_N), "rotX": (K_ROT * math.pi / 180, 0.01 * math.pi / 180, 0.0)})
    # hex engagement: the driver slides along the screw axis and turns with the screw, at each 60 deg position
    rj = mk._frame_with_x((0, 0, 1))

    def gq(r):
        x, y, z, w = mk.mat_to_quat(r)
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    for k in range(6):
        _X[f"bit{k}"] = f"{root}/hex_{k}"
        j = UsdPhysics.PrismaticJoint.Define(stage, _X[f"bit{k}"])
        j.CreateBody0Rel().SetTargets([Sdf.Path(paths["screw"])])
        j.CreateBody1Rel().SetTargets([Sdf.Path(paths["tool"])])
        j.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot0Attr().Set(gq(rj))
        j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        j.CreateLocalRot1Attr().Set(gq(mk.axis_angle((0, 0, 1), -math.radians(60.0 * k)) @ rj))
        j.CreateAxisAttr().Set("X")
        j.CreateLowerLimitAttr().Set(-0.004)
        j.CreateUpperLimitAttr().Set(0.003)
        j.CreateCollisionEnabledAttr().Set(False)
        j.CreateExcludeFromArticulationAttr().Set(True)
        j.CreateJointEnabledAttr().Set(False)
    for name, path in _X.items():
        mk._SIM["catch_paths"][(kit.id, f"x_{name}")] = path  # switched off again at every reset
    # the hex tip passes into the socket: the driver and the screw do not collide
    UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(paths["tool"])).CreateFilteredPairsRel().AddTarget(
        Sdf.Path(paths["screw"]))
    for name in ("key", "hub", "screw"):  # stiff contacts between the small parts that carry the fit
        stage.GetPrimAtPath(paths[name]).CreateAttribute(
            "physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(64)
    n = sum(_slick(stage, f"{paths['hub']}/box{i}", 0.15) for i in HUB_SLICK)
    n += sum(_slick(stage, f"{paths['shaft']}/box{i}", 0.15) for i in SHAFT_SLICK)
    print(f"[keypulley] pulley and screw joints re-authored as D6; hex, hold and pull joints authored disabled; "
          f"{n} keyway and keyseat walls at friction 0.15", flush=True)


mk._author_all = _author


def _attr(path, name, value):
    stage = mk._SIM["stage"]
    if path and stage is not None:
        stage.GetPrimAtPath(path).GetAttribute(name).Set(value)


def _target(path, dof, value):
    _attr(path, f"drive:{dof}:physics:targetPosition", float(value))


def _enable(name, on):
    _attr(_X.get(name), "physics:jointEnabled", bool(on))


def _unwrap(key, p, start):
    prev = mk.MEMORY.get(key, start)
    v = prev + (p - prev + 180.0) % 360.0 - 180.0
    mk.MEMORY[key] = v
    return v


def _stick(key, q, lim, joint, dof, scale=1.0, lo=-math.inf, hi=math.inf):
    """Coulomb friction on a D6 axis: the drive's anchor follows the joint once it has slipped past `lim`."""
    M = mk.MEMORY
    s = M.get(key, 0.0)
    new = min(max(min(max(s, q - lim), q + lim), lo), hi)
    if new != s or key not in M:
        M[key] = new
        _target(mk._SIM["joint_paths"].get((kit.id, joint)), dof, new * scale)
    return new


FIXTURES = [
    Box("v_block", pos=(0.360, 0.0, (TOP + AXZ - R_SHAFT) / 2), size=(0.030, 0.050, AXZ - R_SHAFT - TOP), color=BLOCK),
    Box("bench_mat", pos=(0.260, -0.130, TOP + 0.0005), size=(0.080, 0.080, 0.001), color=MAT),
]
OBJECTS = [*kit.objects(), *FIXTURES]

# -- state ------------------------------------------------------------------------------------------------------


KEY_PAD_GAP_MAX = 0.00008  # m: max positive gap between key underside and seat_pad top for bearing contact
KEY_PAD_PEN_MAX = 0.0005  # m: max solver interpenetration allowed on the press pad
KEY_PRESS_CAUSAL = 0.0007  # m: downward pad travel that must occur while the key bears on the pad
POINT_KEY_GAP_MAX = 0.00004  # m: max positive gap between set-screw flat point and seated key top
POINT_KEY_PEN_MAX = 0.00025  # m: max solver interpenetration on the seated key top
POINT_KEY_MARGIN = 0.0005  # m: inset from key top perimeter for square bearing contact


def _key_cock(q) -> float:
    """Angle of the key's long side from the shaft axis."""
    x = mk.quat_to_mat(q)[:, 0]
    return math.degrees(math.acos(min(1.0, abs(float(x[0])))))


def key_near_seat(objects, tol_xy=0.0006, tol_z=0.00015, tol_deg=3.0) -> bool:
    p, q = kit.pose(objects, "key")
    d = p - KEY_SEATED
    return abs(float(d[0])) <= tol_xy and abs(float(d[1])) <= tol_xy and abs(float(d[2])) <= tol_z and _key_cock(q) <= tol_deg


def key_press(objects) -> float:
    """How far the keyseat pad has been pushed down (m), from the height of its own centre."""
    return float(SEAT_TOP0 - 0.00075 - kit.pose(objects, "seat_pad")[0][2])


def key_pad_gap(objects) -> float:
    """Vertical gap (m) from the key's underside to the top of the keyseat press pad (+ means hovering)."""
    kp, _ = kit.pose(objects, "key")
    pp, _ = kit.pose(objects, "seat_pad")
    return float(kp[2]) - KEY_H / 2 - (float(pp[2]) + 0.00075)


def key_bears_on_pad(objects, tol_x=0.0012, tol_y=0.0008, tol_deg=5.0, gap_max=0.00010) -> bool:
    """True when the key is aligned in the keyseat and its underside bears on the press pad."""
    p, q = kit.pose(objects, "key")
    d = p - KEY_SEATED
    gap = key_pad_gap(objects)
    return (abs(float(d[0])) <= tol_x and abs(float(d[1])) <= tol_y and _key_cock(q) <= tol_deg
            and -KEY_PAD_PEN_MAX <= gap <= gap_max)


def screw_point_on_key(objects):
    """(horizontal excess past key flat top m, vertical gap from screw flat point to key top m)."""
    point = kit.point(objects, "screw", (0.0, 0.0, -SCREW_L))
    rel = mk._inv(kit._pose_T(objects, "key")) @ np.array([point[0], point[1], point[2], 1.0])
    dx_end = max(0.0, abs(float(rel[0])) - _KS / 2)
    radial_excess = math.hypot(dx_end, float(rel[1])) - (KEY_W / 2 - POINT_KEY_MARGIN)
    return radial_excess, float(rel[2]) - KEY_H / 2


def screw_bears_on_key(objects) -> bool:
    """True when the key is genuinely seated and the set screw's flat point bears on the key's top face."""
    if not (bool(mk.MEMORY.get("seated")) and key_press(objects) >= SEAT_TRAVEL - 0.00015
            and -KEY_PAD_PEN_MAX <= key_pad_gap(objects) <= 0.00010
            and key_near_seat(objects, 0.0008, 0.00025, 3.0)):
        return False
    if abs(hub_phase(objects)) > 5.0:
        return False
    radial_excess, gap = screw_point_on_key(objects)
    return radial_excess <= 0.0 and -POINT_KEY_PEN_MAX <= gap <= POINT_KEY_GAP_MAX


def key_in_slot(objects) -> bool:
    p, _ = kit.pose(objects, "key")
    return abs(float(p[1])) < 0.004 and SLOT0 - 0.004 < float(p[0]) < SLOT1 + 0.004 and float(p[2]) < AXZ + 0.016


def hub_lead(objects) -> float:
    """x of the pulley's rear face (the one that meets the key first)."""
    return HUB_X0 + HUB_L + kit.q(objects, "hub_slide")


def hub_home(objects) -> bool:
    return kit.q(objects, "hub_slide") >= TRAVEL - 0.0015


def hub_phase(objects) -> float:
    """Angle (deg) of the keyway from the top of the bore, about the shaft axis."""
    _, q = kit.pose(objects, "hub")
    r = mk.quat_to_mat(q)
    return math.degrees(math.atan2(float(r[2, 1]), float(r[1, 1])))


def screw_deg(objects) -> float:
    """Turn of the set screw in its thread (deg, + tightens)."""
    return kit.q_deg(objects, "screw_turn")


def screw_advance(objects) -> float:
    """How far the screw's point has moved in from its start (m), measured between the screw and the pulley."""
    j = kit.joints_["screw_turn"]
    m = kit._joint_motion(objects, j)
    return float(np.dot(m[:3, :3] @ j.anchor + m[:3, 3] - j.anchor, j.axis))


def bit_in_socket(objects):
    """(end of the hex below the screw top m, sideways offset m, tilt deg, turn of the driver on the screw deg)."""
    rel = mk._inv(kit._pose_T(objects, "screw")) @ kit._pose_T(objects, "tool")
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(rel[2, 2])))))
    return (-float(rel[2, 3]), math.hypot(float(rel[0, 3]), float(rel[1, 3])), tilt,
            math.degrees(math.atan2(float(rel[1, 0]), float(rel[0, 0]))))


def torque_for(clamp_n: float) -> float:
    """Torque (N*m) it takes to turn the screw further in against `clamp_n` newtons of clamp force."""
    return T_RUN + C_THREAD * clamp_n if clamp_n > 0 else 0.0


_TEST_TCPS = None


def _released(st, margin) -> bool:
    tcps = mk.tcps() if _TEST_TCPS is None else _TEST_TCPS
    lo, hi = np.asarray(st.low) - margin, np.asarray(st.high) + margin
    return not any(bool(np.all(np.asarray(t) >= lo) and np.all(np.asarray(t) <= hi)) for t in tcps)


def tool_down(objects, margin=0.010) -> bool:
    """The hex driver is out of the screw, let go, at rest and still on the bench."""
    t = objects["tool"]
    return (mk.MEMORY.get("bit") is None and _released(t, margin) and t.resting(max_speed=0.01, max_spin=0.5)
            and float(t.low[2]) > TOP - 0.020)


def _mechanism_rule(objects):
    M = mk.MEMORY
    # pulley slip-fit friction (sliding and turning)
    slide = kit.q(objects, "hub_slide")
    rot = _unwrap("phase_unwrapped", hub_phase(objects), HUB_START_DEG) - HUB_START_DEG
    _stick("stick_slide", slide, _STICK_M, "hub_slide", "transX")
    _stick("stick_rot", rot, _STICK_DEG, "hub_slide", "rotX")

    # key seating requires both live bearing contact on the pad and pad travel driven while in key contact
    press = key_press(objects)
    prev_press = M.get("pad_press_prev", 0.0)
    dp = press - prev_press
    if key_bears_on_pad(objects) and dp > 0.0:
        M["key_driven_press"] = min(press, M.get("key_driven_press", 0.0) + dp)
    elif dp < 0.0:
        M["key_driven_press"] = max(0.0, min(press, M.get("key_driven_press", 0.0) + dp))
    M["pad_press_prev"] = press
    if (not M.get("seated") and press >= SEAT_TRAVEL - 0.0001 and key_near_seat(objects)
            and -KEY_PAD_PEN_MAX <= key_pad_gap(objects) <= KEY_PAD_GAP_MAX
            and M.get("key_driven_press", 0.0) >= KEY_PRESS_CAUSAL):
        M["seated"] = True
        kit.engage("key_seat")
    kp, kq = kit.pose(objects, "key")
    lead = hub_lead(objects)
    kmin = float(kp[0]) - KEY_L / 2
    if key_in_slot(objects) and not M.get("seated") and lead > kmin + 0.002:
        M["jam"] = True  # the pulley forced over a key that is not home
    if key_in_slot(objects) and _key_cock(kq) > 5.0 and lead > kmin:
        M["jam"] = True
    if float(kp[2]) < TOP - 0.020:
        M["dropped"] = True

    # hex engagement
    depth, off, tilt, turn = bit_in_socket(objects)
    k = int(round(turn / 60.0)) % 6
    err = (turn - 60.0 * k + 180.0) % 360.0 - 180.0
    if M.get("bit") is None:
        if (ENGAGE_DEPTH <= depth <= SOCKET_D + 0.0005 and off <= ENGAGE_MM and tilt <= ENGAGE_TILT
                and abs(err) <= ENGAGE_PHASE):
            M["bit"] = k
            M["used"] = True
            M["pull"] = None  # a screw that is touched again is proved again
            _enable(f"bit{k}", True)
    elif depth < BIT_OUT or off > ENGAGE_MM * 3:
        _enable(f"bit{M['bit']}", False)
        M["bit"] = None

    # set screw. Thread friction: the screw turns once the torque on it passes T_RUN + C_THREAD x clamp force; the
    # point follows the turned angle at PITCH (the drive pushes it with at most THREAD_N, so it stops on whatever
    # it meets); clamp force builds only while the flat point genuinely bears on the seated key's top face.
    theta = _unwrap("screw_unwrapped", screw_deg(objects), 0.0)
    force = M.get("clamp_N", 0.0)
    lim = math.degrees((T_RUN + C_THREAD * force) / K_ROT)
    s = _stick("screw_stick", theta, lim, "screw_turn", "rotX", lo=S_MIN, hi=S_MAX)
    x_t = min(max(PITCH * s / 360.0, X_MIN), X_MAX)
    if M.get("thread_target") != x_t:
        M["thread_target"] = x_t
        _target(mk._SIM["joint_paths"].get((kit.id, "screw_turn")), "transX", x_t)
    adv = screw_advance(objects)
    lag = x_t - adv
    if not screw_bears_on_key(objects):
        M["contact_x"], M["touch_n"], M["free_n"] = None, 0, 0
    elif M.get("contact_x") is None:
        M["touch_n"] = M.get("touch_n", 0) + 1 if lag > DEADBAND else 0
        if M["touch_n"] >= 2:  # bearing on the seated key for two steps running
            M["contact_x"], M["free_n"] = adv, 0
    else:
        M["contact_x"] = max(M["contact_x"], adv)
        M["free_n"] = M.get("free_n", 0) + 1 if lag < DEADBAND / 2 else 0
        if x_t < M["contact_x"] or M["free_n"] >= 4:  # backed off, or thread unloaded
            M["contact_x"], M["touch_n"] = None, 0
    force = 0.0 if M.get("contact_x") is None else K_CLAMP * max(0.0, x_t - M["contact_x"] - DEADBAND)
    M["clamp_N"] = force
    M["torque_Nm"] = torque_for(force)

    # the clamp's hold on the shaft: Coulomb, MU_HOLD x clamp force, anchored where the pulley is when it bites
    hold = MU_HOLD * force
    speed = float(np.linalg.norm(objects["hub"].lin_vel))
    if M.get("hold_on"):
        if hold < 1.0 or abs(slide - M["hold_at"]) > hold / K_HOLD:
            M["hold_on"] = False  # let go, or broke loose
            _enable("hold", False)
    elif hold >= 1.0 and speed < 0.002 and not M.get("pulling"):
        M["hold_on"], M["hold_at"] = True, slide
        _target(_X.get("hold"), "transX", slide)
        _enable("hold", True)

    # the bench's proof pull, once the driver has been used and put down and both grippers are clear
    clear = _released(objects["hub"], 0.020) and tool_down(objects, 0.020)
    if M.get("pulling"):
        M["pull_n"] += 1
        M["pull_max"] = max(M["pull_max"], M["pull_q0"] - slide)
        done = M["pull_n"] >= round(PULL_S * 20)
        if not clear or done or M["pull_max"] > PULL_STOP:
            M["pulling"] = False
            _enable("pull", False)
            if clear:
                M["pull"] = "held" if done and M["pull_max"] <= PULL_SLIP else "moved"
    elif (M.get("seated") and M.get("used") and M.get("pull") is None and clear and hub_home(objects)
          and speed < 0.002):
        M["ready_n"] = M.get("ready_n", 0) + 1
        if M["ready_n"] >= 10:  # 0.5 s of quiet first
            M.update(pulling=True, pull_n=0, pull_max=0.0, pull_q0=slide, ready_n=0)
            _enable("pull", True)
    else:
        M["ready_n"] = 0

    if os.environ.get("KEYPULLEY_DEBUG") and mk._EP["step"] % int(os.environ.get("KEYPULLEY_EVERY", "20")) == 0:
        tp, _ = kit.pose(objects, "tool")
        print(f"[keypulley] step {mk._EP['step']} tcp {[np.round(t, 4).tolist() for t in mk.tcps()]} key {np.round(kp, 4)} "
              f"cock {_key_cock(kq):.1f} press {key_press(objects) * 1000:.2f} mm slide {slide * 1000:.2f} mm "
              f"phase {hub_phase(objects):.2f} | screw {theta:.1f} deg slip {s:.1f} adv {screw_advance(objects) * 1e6:.0f} um "
              f"clamp {force:.0f} N torque {M['torque_Nm']:.3f} | tool {np.round(tp, 4)} depth {depth * 1000:.2f} off "
              f"{off * 1000:.2f} mm tilt {tilt:.1f} turn {turn:.1f} | "
              f"{ {k_: v for k_, v in M.items() if k_ in ('seated', 'jam', 'dropped', 'bit', 'used', 'hold_on', 'pulling', 'pull', 'pull_max')} }"
              f" score {score(objects)}", flush=True)


kit.on_step(_mechanism_rule)

SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"

# -- scoring ---------------------------------------------------------------------------------------------------


def failed(objects) -> bool:
    """Latched: the key fell off the bench, or the pulley was forced over a key that was not fully home."""
    M = mk.MEMORY
    return bool(M.get("jam") or M.get("dropped"))


def key_ok(objects) -> bool:
    return (bool(mk.MEMORY.get("seated")) and key_press(objects) >= SEAT_TRAVEL - 0.00015
            and -KEY_PAD_PEN_MAX <= key_pad_gap(objects) <= 0.00010
            and key_near_seat(objects, 0.0008, 0.00025, 3.0)
            and _released(objects["key"], 0.010))


def hub_ok(objects) -> bool:
    return (key_ok(objects) and hub_home(objects) and _released(objects["hub"], 0.010)
            and objects["hub"].resting(max_speed=0.01, max_spin=0.2))


def screw_ok(objects) -> bool:
    """The set screw was tightened with the hex driver onto the seated key to at least T_SEAT."""
    return (hub_ok(objects) and bool(mk.MEMORY.get("used")) and screw_bears_on_key(objects)
            and mk.MEMORY.get("torque_Nm", 0.0) >= T_SEAT)


def lock_ok(objects) -> bool:
    """The driver is out and put down, and the pulley stayed put through the 20 N proof pull."""
    return screw_ok(objects) and tool_down(objects) and mk.MEMORY.get("pull") == "held"


def score(objects) -> float:
    """0 if failed(). Each step counts only with the ones before it: key fully seated and let go 0.25; pulley over
    the key against the shoulder, let go and still 0.25; set screw seated to T_SEAT 0.25; driver out and put down
    and the pulley held through the proof pull 0.25."""
    if failed(objects):
        return 0.0
    s = 0.0
    if key_ok(objects):
        s += 0.25
        if hub_ok(objects):
            s += 0.25
            if screw_ok(objects):
                s += 0.25
                if lock_ok(objects):
                    s += 0.25
    return round(s, 4)


def success(objects) -> bool:
    return score(objects) >= 1.0 - 1e-9


def distance(objects) -> float:
    p, _ = kit.pose(objects, "key")
    short = max(0.0, T_SEAT - mk.MEMORY.get("torque_Nm", 0.0))
    return float(np.linalg.norm(p - KEY_SEATED) + (TRAVEL - kit.q(objects, "hub_slide")) + short / 10.0)


# -- the scripted demo ------------------------------------------------------------------------------------------
# Right arm: picks the key off the mat, lowers it into the keyseat, presses it home and lets go.
# Left arm: drags the rim round with closed fingertips until the keyway is near the top, pushes the boss so the
# pulley rides over the key's round end (which lines the keyway up) to the shoulder, takes the hex driver from its
# rack, sets its tip in the set screw, turns it clockwise in one wrist stroke until the screw is seated, lifts the
# driver out and stands it back in the rack.

TIP = 0.013
PARK_L = (0.18, 0.24, TOP + 0.09)
PARK_R = (0.18, -0.26, TOP + 0.09)
KEY_OPEN = -0.011
KEY_FIX = (0.0012, 0.0010)  # where the key sits in the closed grip (measured in sim), corrected at the keyseat
TOOL_OPEN = -0.018  # pads 36 mm apart round the 20 mm handle
TOOL_FIX = (-0.0005, 0.0004)  # where the hex tip sits under the closed grip (measured in sim), corrected at the screw
RACK_FIX = (0.003, -0.002)  # the same for the shank over the rack hole on the way back
YAW_IN, YAW_TIGHT, YAW_SNUG = math.radians(60.0), math.radians(-108.0), math.radians(-55.0)
DRAG = (30.0, -30.0)  # deg of rim the fingertips drag over


def _dur(plan):
    return sum(w.duration for w in plan)


def _wait(plan, until, label="wait"):
    t = _dur(plan)
    if until > t:
        last = plan[-1]
        plan.append(Waypoint(until - t, last.pos, last.finger, label, last.yaw))
    return plan


def _key_plan(hover_only=False):
    kx, ky, kz = KEY_START
    grip_z = kz + TIP  # pad tips at the key's mid-height
    hi = AXZ + 0.050
    plan = [Waypoint(2.0, (kx, ky, TOP + 0.06), KEY_OPEN, "over the key", 0.0),
            Waypoint(1.5, (kx, ky, grip_z), KEY_OPEN, "down round the key", 0.0),
            Waypoint(0.8, (kx, ky, grip_z), FINGER_CLOSED, "pinch the key", 0.0),
            Waypoint(1.5, (kx, ky, hi), FINGER_CLOSED, "lift", 0.0)]
    kb = KEY_H / 2 + TIP  # TCP above the key's underside while it is pinched
    z_in = SEAT_TOP0 + 0.0006 + kb  # underside 0.6 mm over the pad: the chamfers find the slot
    z_home = KEY_SEATED[2] - KEY_H / 2 - 0.028 + kb  # 28 mm past home: the arm gives 3 mm per newton, so about 9 N
    px, py = KEY_XC + KEY_FIX[0], KEY_FIX[1]
    if hover_only:  # hold 0.35 mm over KEY_SEATED above a pre-depressed pad, then release without pressing
        z_hover = KEY_SEATED[2] + 0.00035 - KEY_H / 2 + kb
        return plan + [Waypoint(2.5, (px, py, z_in + 0.02), FINGER_CLOSED, "over the keyseat", 0.0),
                       Waypoint(1.5, (px, py, z_hover), FINGER_CLOSED, "hover 0.35 mm above seat", 0.0),
                       Waypoint(1.2, (px, py, z_hover), FINGER_CLOSED, "hold hover", 0.0),
                       Waypoint(0.6, (px, py, z_hover), KEY_OPEN, "drop onto pre-depressed pad", 0.0),
                       Waypoint(1.0, (px, py, hi), KEY_OPEN, "up", 0.0),
                       Waypoint(2.0, PARK_R, FINGER_CLOSED, "park", 0.0)]
    return plan + [Waypoint(2.5, (px, py, z_in + 0.02), FINGER_CLOSED, "over the keyseat", 0.0),
                   Waypoint(1.2, (px, py, z_in), FINGER_CLOSED, "down to the keyseat", 0.0),
                   Waypoint(2.0, (px, py, z_home), FINGER_CLOSED, "press the key home", 0.0),
                   Waypoint(0.8, (px, py, z_home), FINGER_CLOSED, "hold", 0.0),
                   Waypoint(0.6, (px, py, z_home), KEY_OPEN, "let go", 0.0),
                   Waypoint(1.0, (px, py, hi), KEY_OPEN, "up", 0.0),
                   Waypoint(2.0, PARK_R, FINGER_CLOSED, "park", 0.0)]


def _predepress_pad_plan():
    """Left arm uses the 3 mm hex driver tip to push seat_pad down 1 mm before the key enters the slot."""
    rx, ry = RACK_XY
    bx, by = rx + RACK_FIX[0], ry + RACK_FIX[1]
    px, py = KEY_XC + TOOL_FIX[0], TOOL_FIX[1]
    g_z = TOP + RACK_H + 0.001 + TIP
    c_z = AXZ + R_BOSS + 0.015 + (g_z - TOOL_START[2])
    pad_seated_z = AXZ + R_SHAFT - SEAT_DEPTH
    press_z = pad_seated_z - 0.022 + (g_z - TOOL_START[2])  # 22 mm overdrive pushes the 6 N pad all 1 mm home
    return [Waypoint(1.6, (rx, ry, g_z + 0.045), TOOL_OPEN, "over the hex driver", 0.0),
            Waypoint(1.2, (rx, ry, g_z), TOOL_OPEN, "down round its handle", 0.0),
            Waypoint(0.7, (rx, ry, g_z), FINGER_CLOSED, "grip the handle", 0.0),
            Waypoint(1.2, (rx, ry, c_z), FINGER_CLOSED, "lift it out of the rack", 0.0),
            Waypoint(2.2, (px, py, c_z), FINGER_CLOSED, "over the keyseat", 0.0),
            Waypoint(1.2, (px, py, pad_seated_z + 0.003 + (g_z - TOOL_START[2])), FINGER_CLOSED, "into the slot", 0.0),
            Waypoint(1.8, (px, py, press_z), FINGER_CLOSED, "pre-depress the pad 1 mm", 0.0),
            Waypoint(0.6, (px, py, press_z), FINGER_CLOSED, "hold pad down", 0.0),
            Waypoint(1.0, (px, py, c_z), FINGER_CLOSED, "lift out of the slot", 0.0),
            Waypoint(2.2, (bx, by, c_z), FINGER_CLOSED, "back over the rack", 0.0),
            Waypoint(1.8, (bx, by, g_z + 0.001), FINGER_CLOSED, "stand it in the rack", 0.0),
            Waypoint(0.6, (bx, by, g_z + 0.001), TOOL_OPEN, "let go", 0.0),
            Waypoint(1.0, (bx, by, g_z + 0.045), TOOL_OPEN, "up", 0.0),
            Waypoint(2.0, PARK_L, FINGER_CLOSED, "park", 0.0)]


def _rim_point(phi_deg, press=0.002):
    """TCP with the closed pad tips `press` into the top of the rim at angle phi (deg, + toward -y)."""
    f = math.radians(phi_deg)
    r = R_RIM - press
    return (HUB_X0 + BOSS_L + (HUB_L - BOSS_L) / 2, -r * math.sin(f), AXZ + r * math.cos(f) + TIP)


def _hub_plan(tighten="seat", has_key=True, stop_short=0.0):
    a0, a1 = DRAG if has_key else (30.0, -40.0)  # without a key to cam against, drag the keyway all the way to top
    plan = [Waypoint(2.0, _rim_point(a0, -0.012), FINGER_CLOSED, "over the rim", 0.0),
            Waypoint(1.2, _rim_point(a0, 0.005), FINGER_CLOSED, "fingertips onto the rim", 0.0)]
    n = 8
    for i in range(1, n + 1):
        plan.append(Waypoint(0.45, _rim_point(a0 + (a1 - a0) * i / n, 0.005), FINGER_CLOSED, "drag the rim round", 0.0))
    end = plan[-1].pos
    pz = AXZ + R_BOSS + 0.001 + TIP  # pad tips 1 mm above the boss
    zp = AXZ + R_SHAFT + 0.0015 + TIP  # pad tips just over the shaft
    plan += [Waypoint(0.6, (end[0], end[1], end[2] + 0.012), FINGER_CLOSED, "lift off the rim", 0.0)]
    # closed fingertips against the boss's front face. Pushing drags the pulley a couple of degrees past the key
    # and it stops on the key's round end; the fingertips (they grip the face) then lean it back the other way
    # (the arm gives 6 mm per newton sideways, so 12 mm of command is about 2 N) while pushing, and the round end
    # turns the keyway into line.
    f0 = HUB_X0 - 0.025
    target_slide = TRAVEL - stop_short
    f1 = (HUB_X0 + TRAVEL + 0.032 - 0.0095) if stop_short == 0.0 else (HUB_X0 + target_slide + 0.0116 - 0.0095)
    fk = KEY_XC - KEY_L / 2 - HUB_L - 0.0075  # TCP where the pulley's rear face meets the key
    plan += [Waypoint(1.2, (f0, 0.0, pz + 0.03), FINGER_CLOSED, "in front of the pulley", 0.0),
             Waypoint(1.0, (f0, 0.0, zp), FINGER_CLOSED, "down", 0.0)]
    if has_key:
        mid_x = min(fk + 0.046, f1)
        plan += [Waypoint(2.6, (fk + 0.008, 0.0, zp), FINGER_CLOSED, "push the pulley along the shaft to the key", 0.0),
                 Waypoint(1.5, (min(fk + 0.028, f1), -0.012, zp), FINGER_CLOSED, "lean the keyway onto the key", 0.0),
                 Waypoint(2.0, (mid_x, 0.0, zp), FINGER_CLOSED, "push on over the key", 0.0)]
    plan += [Waypoint(2.2 if not has_key else 1.5, (f1, 0.0, zp), FINGER_CLOSED, "push pulley along shaft", 0.0),
             Waypoint(1.0, (f1, 0.0, zp), FINGER_CLOSED, "hold", 0.0),
             Waypoint(0.6, (f1 - 0.012, 0.0, zp), FINGER_CLOSED, "back off", 0.0),
             Waypoint(0.8, (f1 - 0.012, 0.0, pz + 0.03), FINGER_CLOSED, "up", 0.0)]
    if tighten is None:
        return plan + [Waypoint(2.0, PARK_L, FINGER_CLOSED, "park", 0.0)]
    rx, ry = RACK_XY
    bx, by = rx + RACK_FIX[0], ry + RACK_FIX[1]
    sx, sy = HUB_X0 + target_slide + SCREW_DX + TOOL_FIX[0], TOOL_FIX[1]
    g_z = TOP + RACK_H + 0.001 + TIP  # pad tips 1 mm above the rack: the whole handle between the pads
    c_z = AXZ + R_BOSS + 0.004 + (g_z - TOOL_START[2])  # carry: the hex tip 4 mm over the boss
    p_z = AXZ + R_BOSS + 0.0005 + (g_z - TOOL_START[2])  # the hex tip just over the tapped hole
    e_z = AXZ + R_BOSS + (g_z - TOOL_START[2]) - BIT_L - 0.001  # shank on the boss, pressing 1 mm
    yaw1 = YAW_TIGHT if tighten == "seat" else YAW_SNUG
    seek = []
    if not has_key:  # sweep y slightly at socket depth so residual open-loop phase still engages the hex socket
        seek = [Waypoint(0.8, (sx, sy - 0.0015, e_z), FINGER_CLOSED, "seek socket -y", YAW_IN),
                Waypoint(1.2, (sx, sy + 0.0015, e_z), FINGER_CLOSED, "seek socket +y", YAW_IN),
                Waypoint(0.6, (sx, sy, e_z), FINGER_CLOSED, "centre on socket", YAW_IN)]
    elif stop_short > 0.0:  # sweep x slightly at socket depth so mid-shaft stop position engages the hex socket
        seek = [Waypoint(0.8, (sx - 0.0025, sy, e_z), FINGER_CLOSED, "seek socket -x", YAW_IN),
                Waypoint(1.2, (sx + 0.0025, sy, e_z), FINGER_CLOSED, "seek socket +x", YAW_IN),
                Waypoint(0.6, (sx, sy, e_z), FINGER_CLOSED, "centre on socket", YAW_IN)]
    plan += [Waypoint(1.6, (rx, ry, g_z + 0.045), TOOL_OPEN, "over the hex driver", 0.0),
             Waypoint(1.2, (rx, ry, g_z), TOOL_OPEN, "down round its handle", 0.0),
             Waypoint(0.7, (rx, ry, g_z), FINGER_CLOSED, "grip the handle", 0.0),
             Waypoint(1.2, (rx, ry, c_z), FINGER_CLOSED, "lift it out of the rack", 0.0),
             Waypoint(2.4, (sx, sy, c_z), FINGER_CLOSED, "over the set screw", YAW_IN),
             Waypoint(1.0, (sx, sy, p_z), FINGER_CLOSED, "down to the tapped hole", YAW_IN),
             Waypoint(0.6, (sx, sy, p_z), FINGER_CLOSED, "steady", YAW_IN),
             Waypoint(1.5, (sx, sy, e_z), FINGER_CLOSED, "hex tip into the socket", YAW_IN),
             *seek,
             Waypoint(0.4, (sx, sy, e_z), FINGER_CLOSED, "settle", YAW_IN),
             Waypoint(6.0, (sx, sy, e_z), FINGER_CLOSED, "turn the screw clockwise onto the key", yaw1),
             Waypoint(0.8, (sx, sy, e_z), FINGER_CLOSED, "hold the torque", yaw1),
             Waypoint(1.0, (sx, sy, c_z), FINGER_CLOSED, "lift the driver out", yaw1),
             Waypoint(2.4, (bx, by, c_z), FINGER_CLOSED, "back over the rack", 0.0),
             Waypoint(0.5, (bx, by, c_z), FINGER_CLOSED, "steady", 0.0),
             Waypoint(2.0, (bx, by, g_z + 0.001), FINGER_CLOSED, "stand it in the rack", 0.0),
             Waypoint(0.6, (bx, by, g_z + 0.001), TOOL_OPEN, "let go", 0.0),
             Waypoint(1.0, (bx, by, g_z + 0.045), TOOL_OPEN, "up", 0.0),
             Waypoint(2.0, PARK_L, FINGER_CLOSED, "park", 0.0)]
    return plan


def demo():
    # DEMO_VARIANT selects a wrong-path check (see README); unset runs the full-score demo
    v = os.environ.get("DEMO_VARIANT", "")
    idle_r = [Waypoint(2.0, PARK_R, FINGER_CLOSED, "wait", 0.0)]
    if v == "no_key":  # pulley aligned, slid home and the screw turned full stroke with no key in the shaft
        return {"left": _hub_plan(has_key=False), "right": idle_r}
    if v == "hover_key":  # pad pre-depressed with hex tip, then key hovered/dropped without pressing
        left = _predepress_pad_plan()
        start_r = [Waypoint(1.0, PARK_R, FINGER_CLOSED, "wait", 0.0)]
        return {"left": left, "right": _wait(start_r, _dur(left) - 4.0) + _key_plan(hover_only=True)}
    right = _key_plan()
    start = [Waypoint(1.0, PARK_L, FINGER_CLOSED, "wait", 0.0)]
    if v == "skip_screw":  # key and pulley fitted, the set screw never touched
        return {"left": _wait(start, _dur(right) + 0.3) + _hub_plan(tighten=None), "right": right}
    if v == "snug":  # the screw run down until it touches the key, not torqued
        return {"left": _wait(start, _dur(right) + 0.3) + _hub_plan(tighten="snug"), "right": right}
    if v == "off_key_screw":  # pulley stopped 8.5 mm short of shoulder so screw turns axially ahead of the key
        return {"left": _wait(start, _dur(right) + 0.3) + _hub_plan(stop_short=0.0085), "right": right}
    return {"left": _wait(start, _dur(right) + 0.3) + _hub_plan(), "right": right}


# -- offline self-test ------------------------------------------------------------------------------------------


def self_test():
    global _TEST_TCPS
    up = mk.mat_to_quat(np.eye(3))
    seated = (KEY_SEATED.copy(), up)
    in_slot = (KEY_SEATED + np.array([0, 0, SEAT_TRAVEL]), up)
    home = {"hub_slide": TRAVEL}
    turned = {**home, "screw_turn": math.radians(110.0)}
    on_bench = (np.array([0.20, 0.20, TOP + GRIP_W / 2]), mk.mat_to_quat(mk.axis_angle((0, 1, 0), math.pi / 2)))

    def st(q=None, key=seated, tool=None, in_screw=False, screw_adv=None):
        q = dict(q or {})
        if key is seated:
            q.setdefault("seat_press", SEAT_TRAVEL)
        hub = (np.array([HUB_X0 + q.get("hub_slide", 0.0), 0, AXZ]), up)  # keyway at the top
        moved = {"key": key, "hub": hub}
        if in_screw:  # the driver standing in the set screw, shank on the boss
            moved["tool"] = (hub[0] + np.array([SCREW_DX, 0, R_BOSS - BIT_L]), up)
        elif tool is not None:
            moved["tool"] = tool
        out = kit.fake_states(q=q, moved=moved)
        adv = screw_adv
        if adv is None and "screw_turn" in q:
            adv = min(SCREW_GAP, max(0.0, PITCH * math.degrees(q["screw_turn"]) / 360.0))
        if adv:
            sp, sq = kit.pose(out, "screw")
            moved["screw"] = (sp - mk.quat_to_mat(sq)[:, 2] * adv, sq)
            out = kit.fake_states(q=q, moved=moved)
        return out

    seat_mem = {"seated": True, "key_driven_press": SEAT_TRAVEL}
    tight = {**seat_mem, "used": True, "torque_Nm": 0.45, "clamp_N": 358.0}
    snug = {**seat_mem, "used": True, "torque_Nm": 0.10, "clamp_N": 67.0}
    key_tcp = [KEY_SEATED + np.array([0, 0, KEY_H / 2 + 0.006])]
    hub_tcp = [np.array([HUB_X0 + TRAVEL - 0.005, 0, AXZ + R_SHAFT + 0.005])]
    tool_tcp = [np.array([HUB_X0 + TRAVEL + SCREW_DX, 0, AXZ + R_BOSS + SHANK_L + 0.014])]
    cases = {  # name: (states, tcps, MEMORY, score)
        "start": (kit.fake_states(), [], {}, 0.0),
        "key in the slot, not pressed": (st(key=in_slot), [], {}, 0.0),
        "key hovering over pre-depressed pad": (st({"seat_press": SEAT_TRAVEL},
                                                   key=(KEY_SEATED + np.array([0, 0, 0.00039]), up)), [], seat_mem, 0.0),
        "key seated, held down": (st(), key_tcp, seat_mem, 0.0),
        "key seated, let go": (st(), [], seat_mem, 0.25),
        "key seated, pulley half way": (st({"hub_slide": TRAVEL / 2}), [], seat_mem, 0.25),
        "pulley home, still pushed": (st(home), hub_tcp, seat_mem, 0.25),
        "pulley home, let go": (st(home), [], seat_mem, 0.5),
        "screw turned, touching, under torque": (st(turned), [], {**snug, "pull": "held"}, 0.5),
        "screw turned, 0.30 mm above key": (st(turned, screw_adv=0.0), [], {**tight, "pull": "held"}, 0.5),
        "screw turned onto key without hex driver": (st(turned), [], {**seat_mem, "torque_Nm": 0.45, "clamp_N": 358.0}, 0.5),
        "screw seated, driver still in it and held": (st(turned, in_screw=True), tool_tcp, {**tight, "bit": 0}, 0.75),
        "screw seated, driver left in the screw": (st(turned, in_screw=True), [], {**tight, "bit": 0}, 0.75),
        "screw seated, driver out, still in the hand": (st(turned, tool=(np.array([0.25, 0.1, TOP + 0.08]), up)),
                                                         [np.array([0.25, 0.1, TOP + 0.11])], tight, 0.75),
        "screw seated, driver down, no pull yet": (st(turned), [], tight, 0.75),
        "screw seated, driver down, pull moved it": (st(turned), [], {**tight, "pull": "moved"}, 0.75),
        "screw seated, driver down, pull held": (st(turned), [], {**tight, "pull": "held"}, 1.0),
        "same, driver lying on the bench": (st(turned, tool=on_bench), [], {**tight, "pull": "held"}, 1.0),
        "pulled off the shoulder": (st({**turned, "hub_slide": TRAVEL - 0.003}), [], {**snug, "pull": "moved"}, 0.25),
        "pulley home and screw turned, no key": (st(turned, key=(np.array(KEY_START), up)), [],
                                                 {"used": True, "pull": "held"}, 0.0),
        "forced over an unseated key": (st(turned), [], {**tight, "pull": "held", "jam": True}, 0.0),
        "key dropped": (st(turned), [], {**tight, "pull": "held", "dropped": True}, 0.0),
    }
    try:
        for name, (s_, tcps, mem, want) in cases.items():
            mk.new_episode()
            _TEST_TCPS = tcps
            mk.MEMORY.update(mem)
            got = score(s_)
            assert abs(got - want) < 1e-9, (name, got, want)
        # B1 live rule check: pre-depressed pad + hovering key (+0.39 mm) or unpressed drop must not weld/seat
        mk.new_episode()
        _TEST_TCPS = []
        s_hover = st({"seat_press": SEAT_TRAVEL}, key=(KEY_SEATED + np.array([0, 0, 0.00039]), up))
        _mechanism_rule(s_hover)
        assert not mk.MEMORY.get("seated") and score(s_hover) == 0.0
        _mechanism_rule(st({"seat_press": SEAT_TRAVEL}, key=seated))
        assert not mk.MEMORY.get("seated") and score(st({"seat_press": SEAT_TRAVEL}, key=seated)) == 0.0
        # B2 live rule check: screw turned 145 deg with zero axial advance (0.30 mm off key) gives 0 clamp
        mk.new_episode()
        _TEST_TCPS = []
        mk.MEMORY.update(seat_mem)
        s_lag = st({**home, "screw_turn": math.radians(145.0)}, screw_adv=0.0)
        for _ in range(3):
            _mechanism_rule(s_lag)
        assert mk.MEMORY["clamp_N"] == 0.0 and not mk.MEMORY.get("hold_on") and abs(score(s_lag) - 0.5) < 1e-9
        mk.new_episode()
        _TEST_TCPS = []
        s0 = kit.fake_states()
        assert key_in_slot(st(key=in_slot)) and not key_in_slot(s0)
        assert abs(hub_phase(s0) - HUB_START_DEG) < 1e-6 and abs(hub_phase(st(home))) < 1e-6
        assert abs(screw_deg(st(turned)) - 110.0) < 1e-6 and abs(screw_deg(s0)) < 1e-6
        assert abs(screw_advance(st(turned)) - SCREW_GAP) < 1e-9 and abs(screw_advance(st(turned, screw_adv=0.0))) < 1e-9
        assert abs(hub_lead(st(home)) - X_SH) < 1e-9
        assert abs(key_press(kit.fake_states(q={"seat_press": 0.0007})) - 0.0007) < 1e-9 and abs(key_press(s0)) < 1e-9
        # fits: key to keyway 0.05 mm, key to keyseat 0.10 mm, 0.3 mm over the seated key, screw point flush with that
        assert abs(2 * KW_HW - KEY_W - 0.00005) < 1e-9 and abs(2 * SLOT_HW - KEY_W - 0.0001) < 1e-9
        assert abs(R_KEYWAY - (KEY_SEATED[2] + KEY_H / 2 - AXZ) - 0.0003) < 1e-9 and abs(SCREW_GAP - 0.0003) < 1e-9
        assert KEY_SEATED[2] + KEY_H / 2 + SEAT_TRAVEL - AXZ > R_KEYWAY  # an unseated key does not clear the keyway
        # thread: the specified torque is reached within the travel and holds more than the proof pull
        f_seat = (T_SEAT - T_RUN) / C_THREAD
        turn_seat = 360.0 * (SCREW_GAP + DEADBAND + f_seat / K_CLAMP) / PITCH
        assert turn_seat < S_MAX - 25.0 and MU_HOLD * f_seat > 2 * PULL_N and PULL_N / K_HOLD < PULL_SLIP / 2
        assert abs(torque_for(f_seat) - T_SEAT) < 1e-9 and torque_for(0.0) == 0.0
        # the driver: in the rack it is not in the screw; standing in the screw its hex is 2.6 mm deep and square on
        d, off, tilt, turn = bit_in_socket(st(home, in_screw=True))
        assert abs(d - (BIT_L - (R_BOSS - SCREW_TOP_R))) < 1e-9 and off < 1e-9 and tilt < 1e-6 and abs(turn) < 1e-6
        assert ENGAGE_DEPTH < d < SOCKET_D and bit_in_socket(st(home))[1] > 0.05
        assert tool_down(st(home)) and tool_down(st(home, tool=on_bench))
    finally:
        mk.new_episode()
        _TEST_TCPS = None
    return len(cases)


if __name__ == "__main__":
    from yam_arena import ik, station
    from yam_arena.demo import ArmPlan
    from yam_arena.task_file import load

    print(f"scoring: {self_test()} cases pass")
    task = load(__file__)
    hz = 20.0
    for variant in ("", "no_key", "skip_screw", "snug", "hover_key", "off_key_screw"):
        os.environ["DEMO_VARIANT"] = variant
        for arm, waypoints in task.demo().items():
            plan = ArmPlan(list(waypoints), station.BASES[arm])
            q = np.array([plan.action7((k + 1) / hz)[0] for k in range(round(plan.duration * hz) + 5)])
            assert plan.worst_ik_mm < 0.5 and plan.worst_ik_deg < 0.1, (variant, arm, plan.worst_ik_mm, plan.worst_ik_deg)
            assert np.all(q[:, :6] >= ik.LOWER) and np.all(q[:, :6] <= ik.UPPER), (variant, arm)
            if variant == "":
                print(f"{arm}: {plan.duration:.1f} s plan, worst IK {plan.worst_ik_mm:.3f} mm / {plan.worst_ik_deg:.3f} deg")
    os.environ.pop("DEMO_VARIANT", None)
