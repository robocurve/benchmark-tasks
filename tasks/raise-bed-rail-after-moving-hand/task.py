"""Bedside on the station table: a care-training bed rig (70 x 110 cm platform 18 cm high, 8 cm foam mattress, near
long edge 30 cm in front of the arm bases) with a half-length side rail on its near side, and a supine adult care
manikin (upper body), head to the robot's right.

The rail (70 cm long, 16 cm tall molded panel with a round top tube on end posts, 23 cm overall) hangs latched in its low position, its top tube 3 cm below
the mattress top. It rides on two parallel pivot arms hinged to the bed frame: lifting it swings it out and up 24 cm
to its top stop (tube top 22.6 cm over the mattress), where it ends outside the mattress edge again. A gas spring
on one pivot carries 91% of its 4.9 kg weight (lifting takes up to about 4 N at the tube). A spring plunger locks the foot-end pivot at the low and top stops (two holes); it is withdrawn only
while its red pull knob, on the bed frame just beyond the rail's foot end, is held pulled out (12 mm or more). So one
hand holds the knob out while the other lifts the rail off its low stop; between the stops the pin rides on the
boss face, and at the top it snaps in.

The manikin's left arm lies on the mattress along her side, elbow bent 90 deg, and her forearm and hand stick out over
the near edge, straight across the lowered rail. Raising the rail with them there traps them (scores 0). Her
forearm has to be swung in first, about the elbow, until it lies on the mattress along her side.

Run it:  inspect-robots run --embodiment yam_arena -E task=task.py --scorer task_score --instruction "<INSTRUCTION>"
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import math
import os
import sys

import numpy as np
from yam_arena.task_file import TABLE_TOP, Box, Cylinder, Sphere

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SCORE_OVER = "end"

INSTRUCTION = "Raise the near side rail on the resident's bed, with nothing of hers caught in it."

FRAME_COL = (0.55, 0.57, 0.60)  # powder-coated steel bed frame
MATT_COL = (0.20, 0.36, 0.62)  # blue vinyl mattress cover
RAIL_COL = (0.86, 0.87, 0.88)  # light grey molded rail panel
TUBE_COL = (0.70, 0.72, 0.75)
KNOB_COL = (0.85, 0.10, 0.08)
SKIN = (0.93, 0.76, 0.64)  # manikin vinyl
GOWN = (0.62, 0.80, 0.78)  # patient gown

# -- bed rig (fixed): platform and mattress ------------------------------------------------------------------------
BED_X0, BED_X1 = 0.30, 1.00
BED_Y0, BED_Y1 = -0.55, 0.55
DECK_H = 0.18
MATT_T = 0.08
DECK_TOP = TOP + DECK_H
M = DECK_TOP + MATT_T  # mattress top, 26 cm above the table
EDGE = BED_X0  # near edge of the mattress / frame face

# -- side rail: panel + top tube, two pivot arms (parallel links) ----------------------------------------------------
RAIL_L = 0.70
RAIL_YC = -0.10  # spans y -0.45 .. +0.25
RAIL_X = 0.275  # panel centre plane (inner face 0.285, 1.5 cm outside the mattress edge) at the low and high stops
PANEL_T, PANEL_H = 0.020, 0.160
TUBE_GAP = 0.040  # the top tube stands on two end posts, 4 cm clear of the panel (a hand-hold)
TUBE_R = 0.016
TUBE_Z0 = M - 0.030  # top tube centre, rail low
PANEL_Z0 = TUBE_Z0 - TUBE_R - TUBE_GAP - PANEL_H / 2
RAISE_SPEC = 0.24
LINK_HALF = 65.0  # deg: arm points 65 deg below the outward horizontal at the low stop, 65 above at the high stop
LINK_L = RAISE_SPEC / (2 * math.sin(math.radians(LINK_HALF)))  # pivot arm length (13.2 cm)
TOP_DEG = 2 * LINK_HALF  # joint travel low -> high (130 deg)
PIVOT_X = RAIL_X - PANEL_T / 2 + LINK_L * math.cos(math.radians(LINK_HALF))  # arms hinge on the panel's outer face; pivots just inside the frame face
PIVOT_Z = DECK_TOP - 0.005
PIVOT_Y = (RAIL_YC - 0.26, RAIL_YC + 0.26)  # foot-end arm (with the plunger) is the second
_c, _s = math.cos(math.radians(LINK_HALF)), math.sin(math.radians(LINK_HALF))
LINK_END0 = np.array([PIVOT_X - LINK_L * _c, PIVOT_Z - LINK_L * _s])  # (x, z) of the arm ends, rail low
RAISE = 2 * LINK_L * _s  # 0.240 m

PANEL_MASS, TUBE_MASS, LINK_MASS = 2.60, 1.10, 0.60  # steel-framed rail: 4.9 kg moving with both arm pairs
# rail angle a (deg): 0 latched low, TOP_DEG at the top stop
LOW_WIN = 3.0  # within this of the low stop the rail counts as down
TOP_WIN = 3.0
LIFTED_DEG = 20.0  # "lifted off its low stop": about 3 cm up
LOCK_K = 1.5  # N m / deg: plunger engaged (a 20 N push on the tube deflects it about 1 deg)
LOCK_C = 0.02
CB_K = 0.04  # N m / deg drive standing in for the gas spring on the head-end pivot (set point kept < 200 deg) ...
CB_FRAC = 0.91  # ... which carries this share of the rail's weight torque at every angle (as split-siderail gas springs do)
CB_WL = 9.81 * LINK_L * (PANEL_MASS + TUBE_MASS + LINK_MASS)  # two arm pairs, each at half length  # rail weight torque, arms level (N m)


def _cb_target(a_deg: float) -> float:
    """Counterbalance drive set point for rail angle a (deg): K x (set - a) = CB_FRAC x weight torque at a."""
    return a_deg + CB_FRAC * CB_WL * math.cos(math.radians(a_deg - LINK_HALF)) / CB_K


LINK_C = 0.002  # pivot friction (damping)

# -- release plunger: pull knob on the frame face beyond the rail's foot end -----------------------------------------
KNOB_Y = 0.37
KNOB_Z = TOP + DECK_H - 0.025  # 2.5 cm below the deck top
KNOB_R, KNOB_L = 0.014, 0.045
KNOB_X0 = EDGE - 0.022 - KNOB_L / 2  # centre: front face 6.7 cm out from the frame
KNOB_TRAVEL = 0.025
KNOB_PULLED = 0.012  # plunger clear of the arm: pulled at least this far
KNOB_K = 150.0  # N/m return spring (3 N at 2 cm)
KNOB_MASS = 0.03

# -- manikin: fixed torso, head, far arm; near (left) arm jointed --------------------------------------------
TORSO_X0, TORSO_X1 = 0.51, 0.84
TORSO_Y0, TORSO_Y1 = -0.38, 0.20
TORSO_H = 0.17
HEAD_C = (0.675, -0.48, M + 0.095)
HEAD_R = 0.095
SHOULDER = np.array([0.45, -0.24, M + 0.030])  # left shoulder joint (axis along the upper arm, y)
UA_LEN, UA_R = 0.28, 0.030  # upper arm: rounded (cylinder), 6 cm thick
ELBOW = np.array([0.45, SHOULDER[1] + UA_LEN, M + 0.030])  # y 0.04
FA_LEN, FA_R = 0.26, 0.027  # forearm (elbow pivot to wrist): rounded (cylinder), 5.4 cm thick
HAND_LEN, HAND_W, HAND_H = 0.17, 0.085, 0.030
FA_BACK = 0.02  # forearm box starts 2 cm behind the elbow pivot
UA_MASS, FA_MASS, HAND_MASS = 0.60, 0.45, 0.20
ELBOW_LIM = (-88.0, 40.0)  # deg: e < 0 straightens the elbow (forearm turns towards her feet, +y); 0 = 90 deg bent
SHOULDER_LIM = (-15.0, 60.0)  # deg about +y: < 0 lets the forearm droop over the edge, > 0 lifts it
JOINT_C = 0.002  # N m s / deg: light joint friction
ELBOW_C = 0.02  # N m s / deg elbow damping
ELBOW_K, ELBOW_SLIP = 0.25, 1.5  # elbow friction: holds up to about 0.4 N m (K x SLIP), then slips and stays put


def _fa_centre():
    return (ELBOW[0] + FA_BACK - FA_LEN / 2 - FA_BACK / 2, ELBOW[1], ELBOW[2])


WRIST_X0 = ELBOW[0] - FA_LEN

# spawn rotations (xyzw) for bodies that are not upright / axis-aligned
ROT = {
    "rail_tube": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # cylinder along y
    "release_knob": (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)),  # cylinder along x
    "manikin_upper_arm": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # along y
    "manikin_arm_r": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # along y
    "manikin_torso_l": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # along y
    "manikin_torso_r": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # along y
    "manikin_neck": (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)),  # along y
    "manikin_forearm": (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)),  # along x (out over the edge)
}


def _bed() -> list:
    xc, yc = (BED_X0 + BED_X1) / 2, (BED_Y0 + BED_Y1) / 2
    lx, ly = BED_X1 - BED_X0, BED_Y1 - BED_Y0
    return [
        Box("bed_frame", pos=(xc, yc, TOP + DECK_H / 2), size=(lx, ly, DECK_H), color=FRAME_COL),
        Box("mattress", pos=(xc, yc, DECK_TOP + MATT_T / 2), size=(lx, ly, MATT_T), color=MATT_COL),
    ]


def _manikin() -> list:
    tx, ty = (TORSO_X0 + TORSO_X1) / 2, (TORSO_Y0 + TORSO_Y1) / 2
    parts = [
        # rounded trunk: two flank rolls along her length with a block between them (flat back, rounded sides)
        Cylinder("manikin_torso_l", pos=(TORSO_X0 + TORSO_H / 2, ty, M + TORSO_H / 2), radius=TORSO_H / 2,
                 height=TORSO_Y1 - TORSO_Y0, color=GOWN),
        Cylinder("manikin_torso_r", pos=(TORSO_X1 - TORSO_H / 2, ty, M + TORSO_H / 2), radius=TORSO_H / 2,
                 height=TORSO_Y1 - TORSO_Y0, color=GOWN),
        Box("manikin_torso", pos=(tx, ty, M + TORSO_H / 2), size=(TORSO_X1 - TORSO_X0 - TORSO_H, TORSO_Y1 - TORSO_Y0,
                                                                 TORSO_H), color=GOWN),
        Sphere("manikin_head", pos=HEAD_C, radius=HEAD_R, color=SKIN),
        Cylinder("manikin_neck", pos=(0.675, -0.405, M + 0.06), radius=0.05, height=0.07, color=SKIN),
        Cylinder("manikin_arm_r", pos=(0.90, -0.06, M + 0.030), radius=UA_R, height=0.56, color=SKIN),
    ]
    return parts


UPPER_ARM = Cylinder("manikin_upper_arm", pos=(SHOULDER[0], SHOULDER[1] + UA_LEN / 2, SHOULDER[2]), radius=UA_R,
                     height=UA_LEN,
                mass=UA_MASS, color=GOWN)
FOREARM = Cylinder("manikin_forearm", pos=_fa_centre(), radius=FA_R, height=FA_LEN + FA_BACK, mass=FA_MASS, color=SKIN)
HAND = Box("manikin_hand", pos=(WRIST_X0 - HAND_LEN / 2, ELBOW[1], ELBOW[2] - 0.010), size=(HAND_LEN, HAND_W, HAND_H),
           mass=HAND_MASS, color=SKIN)

PANEL0 = np.array([RAIL_X, RAIL_YC, PANEL_Z0])
TUBE0 = np.array([RAIL_X, RAIL_YC, TUBE_Z0])
PANEL = Box("rail_panel", pos=tuple(PANEL0), size=(PANEL_T, RAIL_L, PANEL_H), mass=PANEL_MASS, color=RAIL_COL)
TUBE = Cylinder("rail_tube", pos=tuple(TUBE0), radius=TUBE_R, height=RAIL_L, mass=TUBE_MASS, color=TUBE_COL)
KNOB0 = np.array([KNOB_X0, KNOB_Y, KNOB_Z])


class ReleaseKnob(Cylinder):
    """The plunger's pull knob; its spawner also authors all the rig's joints and the pivot arms."""


KNOB = ReleaseKnob("release_knob", pos=tuple(KNOB0), radius=KNOB_R, height=KNOB_L, mass=KNOB_MASS, color=KNOB_COL)

def _pyfloats(o):
    """Positions as plain Python floats (numpy float64 turns the reset pose tensor into float64, which Warp refuses)."""
    import dataclasses

    return dataclasses.replace(o, pos=tuple(float(v) for v in o.pos))


UPPER_ARM, FOREARM, HAND, PANEL, TUBE, KNOB = (_pyfloats(o) for o in (UPPER_ARM, FOREARM, HAND, PANEL, TUBE, KNOB))
OBJECTS = [_pyfloats(o) for o in (*_bed(), *_manikin(), UPPER_ARM, FOREARM, HAND, PANEL, TUBE, KNOB)]

TWIN_EXTRA = [
    {"name": "rail_pivot_arms", "kind": "joint", "real": "half rail's two steel pivot arms (13.2 cm centres), "
     "parallel-link", "note": "rail travels 130 deg of arm swing: out and up 24 cm"},
    {"name": "counterbalance", "kind": "joint", "real": "gas spring on the head-end pivot arm",
     "note": "carries 91% of the rail's weight torque (6.4 N m with the arms level) at every angle"},
    {"name": "plunger", "kind": "joint", "real": "spring plunger with red pull knob, engages a hole in the foot-end pivot boss at the low "
     "stop and one at the top stop", "note": "withdrawn while the knob is pulled 12 mm or more; 150 N/m return spring"},
    {"name": "manikin_left_arm", "kind": "joint", "real": "care manikin's jointed left arm",
     "note": "shoulder rotation about the upper arm (-15..60 deg), elbow hinge (2..130 deg bend) with friction "
     "(holds about 0.4 N m), wrist fixed"},
]

# -- Isaac side ------------------------------------------------------------------------------------------------------
_P: dict = {}


def _rotmat_xyzw(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _link_centre(i: int) -> np.ndarray:
    return np.array([(PIVOT_X + LINK_END0[0]) / 2, PIVOT_Y[i], (PIVOT_Z + LINK_END0[1]) / 2])


def _author_joints(stage, parent: str) -> None:
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    w = UsdGeom.Xformable(stage.GetPrimAtPath(parent)).ComputeLocalToWorldTransform(0).ExtractTranslation()
    W = np.array([w[0], w[1], w[2]], dtype=float)
    q_id = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    bodies = {o.name: o for o in OBJECTS}

    def v(p):
        return Gf.Vec3f(*(float(x) for x in p))

    def frame(name, p_world):
        """(prim path, local position, local rotation) of world point p in body `name`'s spawn frame."""
        if name in bodies:
            o = bodies[name]
            pos = np.array(o.pos, float)
            q = ROT.get(name, (0.0, 0.0, 0.0, 1.0))
        else:  # a pivot arm authored below: identity rotation at its centre
            pos = _link_centre(int(name[-1]) - 1)
            q = (0.0, 0.0, 0.0, 1.0)
        R = _rotmat_xyzw(q)
        loc = R.T @ (np.asarray(p_world, float) - pos)
        qi = Gf.Quatf(float(q[3]), -float(q[0]), -float(q[1]), -float(q[2]))  # inverse: joint frame = world axes
        return Sdf.Path(f"{parent}/{name}"), v(loc), qi

    def joint(kind, jname, b0, b1, p_world, axis="Y", lo=None, hi=None):
        j = kind.Define(stage, f"{parent}/release_knob/{jname}")
        if b0 is None:
            j.CreateLocalPos0Attr(v(W + np.asarray(p_world, float)))
            j.CreateLocalRot0Attr(q_id)
        else:
            path, lp, lr = frame(b0, p_world)
            j.CreateBody0Rel().SetTargets([path])
            j.CreateLocalPos0Attr(lp)
            j.CreateLocalRot0Attr(lr)
        path, lp, lr = frame(b1, p_world)
        j.CreateBody1Rel().SetTargets([path])
        j.CreateLocalPos1Attr(lp)
        j.CreateLocalRot1Attr(lr)
        if kind not in (UsdPhysics.FixedJoint, UsdPhysics.Joint):
            j.CreateAxisAttr(axis)
            if lo is not None:
                j.CreateLowerLimitAttr(lo)
                j.CreateUpperLimitAttr(hi)
        j.CreateExcludeFromArticulationAttr(True)
        _P[jname] = str(j.GetPath())
        return j

    def drive(j, kind, k, c, target=0.0):
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(k)
        d.CreateDampingAttr(c)
        d.CreateTargetPositionAttr(target)
        return d

    def visual(prim, name, centre, size, color, rot_y_deg=0.0):
        g = UsdGeom.Cube.Define(stage, f"{prim}/{name}")
        g.CreateSizeAttr(1.0)
        g.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        x = UsdGeom.Xformable(g)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, centre)))
        if rot_y_deg:
            x.AddRotateYOp().Set(float(rot_y_deg))
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        return g

    # pivot arms: rigid bodies without colliders (they sit beside the frame), steel flat bar
    for i in range(2):
        c = _link_centre(i)
        p = f"{parent}/rail_link{i + 1}"
        xf = UsdGeom.Xform.Define(stage, p)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, W + c)))
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        m = UsdPhysics.MassAPI.Apply(xf.GetPrim())
        m.CreateMassAttr(LINK_MASS)
        _i = float(max(5e-4, LINK_MASS * (LINK_L + 0.02) ** 2 / 12))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(_i, _i, _i))
        m.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
        # bar along the arm: from pivot to end, pointing 65 deg below the outward horizontal (-x)
        visual(p, "bar", (0, 0, 0), (LINK_L + 0.02, 0.012, 0.025), (0.35, 0.36, 0.38), rot_y_deg=-LINK_HALF)
    for i in range(2):
        visual(f"{parent}/{BEDF}", f"pivot_plate{i + 1}", (EDGE - 0.004 - BED_XC, PIVOT_Y[i], PIVOT_Z - TOP - DECK_H / 2),
               (0.008, 0.05, 0.05), (0.30, 0.31, 0.33))
    # rail: tube welded on the panel; arms hinged to the frame and to the panel (a parallelogram)
    joint(UsdPhysics.FixedJoint, "weld_tube", PANEL.name, TUBE.name, TUBE0)
    for i in range(2):
        piv = np.array([PIVOT_X, PIVOT_Y[i], PIVOT_Z])
        end = np.array([LINK_END0[0], PIVOT_Y[i], LINK_END0[1]])
        j = joint(UsdPhysics.RevoluteJoint, f"pivot{i + 1}", None, f"rail_link{i + 1}", piv, lo=-1.0,
                  hi=TOP_DEG + 1.0)
        if i == 1:  # foot-end arm: the plunger (engaged at spawn, rail latched low)
            drive(j, "angular", LOCK_K, LOCK_C, 0.0)
        else:  # head-end arm: counterbalance spring
            drive(j, "angular", CB_K, LINK_C, _cb_target(0.0))
        joint(UsdPhysics.RevoluteJoint, f"link_end{i + 1}", f"rail_link{i + 1}", PANEL.name, end)
    # each pivot arm is one of a parallel pair in the real rail; the pairs keep the panel upright. Here a 6-dof joint
    # to the frame stands in for the second arms: it locks the panel's rotation and its sideways (y) slide only.
    j = joint(UsdPhysics.Joint, "panel_upright", None, PANEL.name, PANEL0)
    for dof in ("rotX", "rotY", "rotZ", "transY"):
        lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), dof)
        lim.CreateLowAttr(1.0)
        lim.CreateHighAttr(-1.0)
    # plunger knob: slides out along -x against its spring
    j = joint(UsdPhysics.PrismaticJoint, "knob_slide", None, KNOB.name, KNOB0, axis="X", lo=-KNOB_TRAVEL, hi=0.0)
    drive(j, "linear", KNOB_K, 5.0, 0.0)
    # tube posts (visual, on the tube body: tube local z runs along -y, local y is up)
    for sgn in (-1, 1):
        visual(f"{parent}/{TUBE.name}", f"post{'ab'[sgn > 0]}", (0.0, -(TUBE_R + TUBE_GAP) / 2, -sgn * (RAIL_L / 2 - 0.02)),
               (0.016, TUBE_R + TUBE_GAP, 0.016), (0.70, 0.72, 0.75))
    # plunger body on the frame face (visual; scored bodies carry no visual extras, so their bounding boxes are true)
    visual(f"{parent}/{BEDF}", "plunger_body", (EDGE - 0.011 - BED_XC, KNOB_Y, KNOB_Z - TOP - DECK_H / 2),
           (0.022, 0.016, 0.016), (0.7, 0.7, 0.72))
    # manikin arm: shoulder rotation about the upper arm (y), elbow hinge (z in the upper arm), wrist welded
    j = joint(UsdPhysics.RevoluteJoint, "shoulder", None, UPPER_ARM.name, SHOULDER, axis="Y", lo=SHOULDER_LIM[0],
              hi=SHOULDER_LIM[1])
    drive(j, "angular", 0.0, JOINT_C)
    j = joint(UsdPhysics.RevoluteJoint, "elbow", UPPER_ARM.name, FOREARM.name, ELBOW, axis="Z", lo=ELBOW_LIM[0],
              hi=ELBOW_LIM[1])
    drive(j, "angular", ELBOW_K, ELBOW_C)
    joint(UsdPhysics.FixedJoint, "wrist", FOREARM.name, HAND.name, (WRIST_X0, ELBOW[1], ELBOW[2]))
    if os.environ.get("RAIL_DEBUG"):
        print(f"[rail] joints authored under {parent}; env origin {W}", flush=True)


BEDF = "bed_frame"
BED_XC = (BED_X0 + BED_X1) / 2


def _spawn_knob(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    from isaaclab.sim.spawners.shapes import spawn_cylinder
    from isaaclab.sim.utils.stage import get_current_stage

    prim = spawn_cylinder(prim_path, cfg, translation, orientation, **kwargs)
    _author_joints(get_current_stage(), str(prim_path).rsplit("/", 1)[0])
    return prim


def _knob_object(spec: ReleaseKnob):
    import isaaclab.sim as sim_utils
    from isaaclab.sim.schemas import CollisionPropertiesCfg, MassPropertiesCfg, RigidBodyPropertiesCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose
    from yam_arena.constants import GRIPPY_FRICTION
    from yam_arena.station import linear_rgb

    _register_module()

    @configclass
    class KnobCfg(sim_utils.CylinderCfg):
        func: object = clone(_spawn_knob)

    cfg = KnobCfg(radius=spec.radius, height=spec.height, rigid_props=RigidBodyPropertiesCfg(),
                  mass_props=MassPropertiesCfg(mass=spec.mass), collision_props=CollisionPropertiesCfg(),
                  physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=GRIPPY_FRICTION[0],
                                                                  dynamic_friction=GRIPPY_FRICTION[1]),
                  visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(spec.color), roughness=0.4))
    return Object(name=spec.name, object_type=ObjectType.RIGID, spawner_cfg=cfg,
                  initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=ROT[spec.name]))


def _register_module():
    if __name__ not in sys.modules:  # configclass needs this module registered (task files load unregistered)
        import types

        mod = types.ModuleType(__name__)
        mod.__dict__.update(globals())
        sys.modules[__name__] = mod


def _patch_scene(scene) -> None:
    if getattr(scene, "_bedrail_patched", False):
        return
    orig = scene.task_object

    def task_object(spec):
        if isinstance(spec, ReleaseKnob):
            return _knob_object(spec)
        q = ROT.get(spec.name)
        if q is None:
            return orig(spec)
        yaw_q = scene._yaw_xyzw
        scene._yaw_xyzw = lambda _yaw: q
        try:
            return orig(spec)
        finally:
            scene._yaw_xyzw = yaw_q

    scene.task_object = task_object
    scene._bedrail_patched = True


class _Finder(importlib.abc.MetaPathFinder):
    _bedrail = True

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
    elif not any(getattr(f, "_bedrail", False) for f in sys.meta_path):
        sys.meta_path.insert(0, _Finder())


_install()

# -- episode: plunger logic each control step, latched gates --------------------------------------------------------
_ENV: dict = {"env": None}


class _Episode:
    def __init__(self):
        self.lock = 0.0  # plunger: the stop angle (deg, 0 or TOP_DEG) whose hole its pin is in, or None (out / on the boss face)
        self.pinched = False  # rail off its low stop while her forearm or hand was over the edge in its path
        self.lifted_arm = False  # forearm/hand raised more than 30 cm above the mattress
        self.overbent = False
        self.yanked = False  # her forearm swung or her hand moved too fast (rough handling)
        self.limb_hist = []  # (t, elbow deg, hand centre) over the last few control steps
        self.t = 0.0
        self.wrenched = False  # rail forced off its pivot arms (a real rail would be damaged)
        self.elbow_set = 0.0
        self.cb_set = _cb_target(0.0)
        self.why = ""
        self.last = None


_EP = _Episode()


def _stage():
    from isaaclab.sim.utils.stage import get_current_stage

    return get_current_stage()


def _set_lock(state) -> None:
    if _EP.lock == state:
        return
    _EP.lock = state
    try:
        d = _stage().GetPrimAtPath(_P["pivot2"])
        d.GetAttribute("drive:angular:physics:stiffness").Set(0.0 if state is None else LOCK_K)
        d.GetAttribute("drive:angular:physics:damping").Set(LINK_C if state is None else LOCK_C)
        d.GetAttribute("drive:angular:physics:targetPosition").Set(0.0 if state is None else float(state))
    except Exception as e:  # noqa: BLE001
        print(f"[rail] lock switch failed: {e!r}", flush=True)


def reset_episode() -> None:
    global _EP
    _EP = _Episode()
    if not _P:
        return
    try:
        if _stage() is None:
            return
    except Exception:  # noqa: BLE001  (offline)
        return
    _EP.lock = None
    _set_lock(0.0)
    try:
        _stage().GetPrimAtPath(_P["elbow"]).GetAttribute("drive:angular:physics:targetPosition").Set(0.0)
    except Exception:  # noqa: BLE001
        pass


def _install_hooks() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline)
        return
    cls = ie.YamArenaEmbodiment
    if not getattr(cls.reset, "_bedrail_hooked", False):
        orig = cls.reset

        def reset(self, *a, **k):
            reset_episode()
            return orig(self, *a, **k)

        reset._bedrail_hooked = True
        cls.reset = reset
    if not getattr(cls._task_step, "_bedrail_hooked", False):
        base = cls._task_step

        def _task_step(self, *a, **k):
            _ENV["env"] = self._env
            out = base(self, *a, **k)
            try:
                from yam_arena.scene import object_states

                st = object_states(self._env, [o.name for o in OBJECTS if o.free])
                _plunger_step(st)
                _elbow_friction(st)
                _update(st)
                _limb_speed(st, float(getattr(getattr(self._env, "unwrapped", self._env), "step_dt", 0.05)))
            except Exception as e:  # noqa: BLE001
                print(f"[rail] step hook failed: {e!r}", flush=True)
            return out

        _task_step._bedrail_hooked = True
        cls._task_step = _task_step


_install_hooks()


# -- geometry readouts (positions and bounding boxes only: no orientation needed) ------------------------------------
def _finite(a) -> bool:
    return a is not None and bool(np.isfinite(np.asarray(a, dtype=float)).all())


# the panel centre travels on a circle of radius LINK_L about this point (parallelogram: the panel never turns)
_ARC_C = np.array([PANEL0[0] + LINK_L * _c, PANEL0[2] + LINK_L * _s])


def rail_angle(objects) -> float:
    """Pivot-arm swing (deg) from the low stop: 0 latched low, 130 at the top stop. From the panel's height alone
    (monotonic over the travel), so a few mm of play across the bed does not shift it."""
    p = objects[PANEL.name].pos
    if not _finite(p):
        return float("nan")
    s = max(-1.0, min(1.0, (p[2] - _ARC_C[1]) / LINK_L))
    return math.degrees(math.asin(s)) + LINK_HALF


def rail_on_pivots(objects) -> bool:
    """The panel is still where its two pivot arms hold it (on its arc, within 1.5 cm): not wrenched off them."""
    p = objects[PANEL.name].pos
    if not _finite(p):
        return False
    r = math.hypot(p[0] - _ARC_C[0], p[2] - _ARC_C[1])
    return bool(abs(r - LINK_L) <= 0.015 and abs(p[1] - RAIL_YC) <= 0.015)


def rail_rise(objects) -> float:
    p = objects[PANEL.name].pos
    return float(p[2] - PANEL0[2]) if _finite(p) else float("nan")


def knob_pulled(objects) -> float:
    p = objects[KNOB.name].pos
    return float(KNOB0[0] - p[0]) if _finite(p) else 0.0


def _counterbalance(a) -> None:
    """Gas-spring counterbalance: keeps the drive set point so it carries CB_FRAC of the rail's weight torque."""
    t = _cb_target(a)
    if abs(t - _EP.cb_set) < 0.2:
        return
    _EP.cb_set = t
    try:
        _stage().GetPrimAtPath(_P["pivot1"]).GetAttribute("drive:angular:physics:targetPosition").Set(float(t))
    except Exception as e:  # noqa: BLE001
        print(f"[rail] counterbalance failed: {e!r}", flush=True)


def _elbow_friction(objects) -> None:
    """Stick-slip friction in the manikin elbow: a spring to a set point that is dragged along once the forearm is
    pushed more than ELBOW_SLIP from it, so a limp arm stays where it is put."""
    e = elbow_angle(objects)
    if not math.isfinite(e):
        return
    t = _EP.elbow_set
    if e > t + ELBOW_SLIP:
        t = e - ELBOW_SLIP
    elif e < t - ELBOW_SLIP:
        t = e + ELBOW_SLIP
    else:
        return
    _EP.elbow_set = t
    try:
        _stage().GetPrimAtPath(_P["elbow"]).GetAttribute("drive:angular:physics:targetPosition").Set(float(t))
    except Exception as e:  # noqa: BLE001
        print(f"[rail] elbow friction failed: {e!r}", flush=True)


def _plunger_step(objects) -> None:
    a = rail_angle(objects)
    if not math.isfinite(a):
        return
    _counterbalance(a)
    if knob_pulled(objects) >= KNOB_PULLED:
        _set_lock(None)
    elif _EP.lock is not None:
        pass  # the pin is in its hole: it stays engaged, however hard the rail is pushed, until the knob is pulled
    elif a <= LOW_WIN:  # knob let go: the pin rides on the boss face until a stop's hole lines up, then drops in
        _set_lock(0.0)
    elif a >= TOP_DEG - TOP_WIN:
        _set_lock(TOP_DEG)


ARM_PARTS = (FOREARM.name, HAND.name)
LINE_X = EDGE  # the rail line: the mattress edge, 1.5 cm inside the raised rail's inner face
PATH_TOP = TUBE_Z0 + RAISE + TUBE_R + 0.02  # the rail's swept space reaches this high
LIFT_MAX = 0.30
REST_TOL = 0.025  # resting: underside no more than this above the mattress top


def arm_inside(objects) -> bool:
    """Forearm and hand rest on the mattress inside the rail line: their bounding boxes wholly at x >= the mattress
    edge, their undersides between 1 cm below and 2.5 cm above the mattress top (lying on it, not hanging off it and
    not held up)."""
    for n in ARM_PARTS:
        o = objects[n]
        if not (_finite(o.low) and _finite(o.high)):
            return False
        if o.low[0] < LINE_X or o.low[2] < M - 0.01 or o.low[2] > M + REST_TOL:
            return False
    return True


def arm_in_path(objects) -> bool:
    """Some part of her forearm or hand is over the edge, in the space the rail sweeps through."""
    for n in ARM_PARTS:
        o = objects[n]
        if not (_finite(o.low) and _finite(o.high)):
            return True
        if o.low[0] < LINE_X and o.low[2] < PATH_TOP:
            return True
    return False


def elbow_angle(objects) -> float:
    """Forearm direction about the elbow (deg): 0 sticking straight out over the edge (90 deg bend), -90 straight
    along her side (elbow straight), + towards her head (more bent)."""
    p = objects[FOREARM.name].pos
    if not _finite(p):
        return float("nan")
    d = np.asarray(p[:2]) - ELBOW[:2]
    return -math.degrees(math.atan2(d[1], -d[0]))


def _update(objects) -> None:
    if objects is _EP.last:
        return
    _EP.last = objects
    a = rail_angle(objects)
    if not _EP.pinched and (not math.isfinite(a) or a > LOW_WIN) and arm_in_path(objects):
        _EP.pinched, _EP.why = True, f"rail at {a:.0f} deg with the arm in its path"
    for n in ARM_PARTS:
        h = objects[n].high
        if not _EP.lifted_arm and (not _finite(h) or h[2] > M + LIFT_MAX):
            _EP.lifted_arm, _EP.why = True, f"{n} lifted over 30 cm"
    if not _EP.wrenched and not rail_on_pivots(objects):
        _EP.wrenched, _EP.why = True, "rail forced off its pivot arms"
    e = elbow_angle(objects)
    if not _EP.overbent and (not math.isfinite(e) or e < ELBOW_LIM[0] - 5 or e > ELBOW_LIM[1] + 5):
        _EP.overbent, _EP.why = True, f"elbow at {e:.0f} deg"


ELBOW_RATE_MAX = 90.0  # deg/s: faster swings of her forearm count as rough handling
HAND_SPEED_MAX = 0.5  # m/s
SPEED_WIN = 0.15  # s: averaged over this window (ignores single-step contact jitter)


def _limb_speed(objects, dt: float) -> None:
    _EP.t += dt
    e = elbow_angle(objects)
    h = np.asarray(objects[HAND.name].pos, float)
    if not (math.isfinite(e) and _finite(h)):
        return
    _EP.limb_hist.append((_EP.t, e, h))
    while len(_EP.limb_hist) > 2 and _EP.limb_hist[1][0] <= _EP.t - SPEED_WIN:
        _EP.limb_hist.pop(0)
    t0, e0, h0 = _EP.limb_hist[0]
    if _EP.t - t0 < SPEED_WIN - 1e-6 or _EP.yanked:
        return
    w = abs(e - e0) / (_EP.t - t0)
    v = float(np.linalg.norm(h - h0)) / (_EP.t - t0)
    if w > ELBOW_RATE_MAX or v > HAND_SPEED_MAX:
        _EP.yanked, _EP.why = True, f"her arm moved too fast ({w:.0f} deg/s, {v:.2f} m/s)"


def rail_lifted(objects) -> bool:
    a = rail_angle(objects)
    return bool(math.isfinite(a) and a >= LIFTED_DEG)


def rail_latched_top(objects) -> bool:
    """At the top stop with the plunger engaged (knob back in, so the rail cannot drop)."""
    a = rail_angle(objects)
    return bool(math.isfinite(a) and a >= TOP_DEG - TOP_WIN and knob_pulled(objects) < 0.005)


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


CLEAR = 0.10
CLEAR_OF = (PANEL.name, TUBE.name, KNOB.name, FOREARM.name, HAND.name, UPPER_ARM.name)


def released(objects) -> bool:
    """Every gripper point at least 10 cm from the rail, the knob and her arm (their bounding boxes)."""
    pts = gripper_points()
    if not pts:
        return False
    for n in CLEAR_OF:
        o = objects[n]
        if not (_finite(o.low) and _finite(o.high)):
            return False
        lo, hi = np.asarray(o.low), np.asarray(o.high)
        for p in pts:
            if not _finite(p) or float(np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0))) < CLEAR:
                return False
    return True


def gated(objects) -> bool:
    _update(objects)
    return bool(_EP.pinched or _EP.lifted_arm or _EP.overbent or _EP.wrenched or _EP.yanked)


def score(objects) -> float:
    """0 if the rail ever left its low stop with her forearm or hand over the edge in its path, the arm was lifted more
    than 30 cm above the mattress or bent past its elbow stops, the rail was wrenched off its pivot arms, or the arm is
    not lying inside the rail line.
    Else 0.25 arm inside; 0.5 rail lifted off its low stop (20 deg of arm swing, about 3 cm); 1.0 rail latched at its
    top stop (knob back in) with grippers 10 cm clear of rail, knob and arm."""
    if gated(objects) or not arm_inside(objects):
        return 0.0
    if rail_latched_top(objects) and released(objects):
        return 1.0
    if rail_lifted(objects):
        return 0.5
    return 0.25


def success(objects) -> bool:
    return bool(score(objects) == 1.0)


def distance(objects) -> float:
    if gated(objects):
        return 1.0
    a = rail_angle(objects)
    e = elbow_angle(objects)
    d_arm = 0.0 if arm_inside(objects) else abs((e if math.isfinite(e) else 0.0) - ELBOW_LIM[0]) / 90.0
    d_rail = max(0.0, TOP_DEG - (a if math.isfinite(a) else 0.0)) / TOP_DEG
    return float(0.5 * d_arm + 0.5 * d_rail)


def progress(objects) -> dict:
    _update(objects)
    return {"arm_inside": float(arm_inside(objects)), "never_pinched": float(not _EP.pinched),
            "knob_pulled_mm": knob_pulled(objects) * 1000, "rail_deg": rail_angle(objects),
            "rail_lifted": float(rail_lifted(objects)), "latched_top": float(rail_latched_top(objects)),
            "released": float(released(objects)), "elbow_deg": elbow_angle(objects),
            "rail_on_pivots": float(rail_on_pivots(objects))}


VARIATION: dict = {}

