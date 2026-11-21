"""On the station table: a piece of light blue painted drywall (30 cm wide, 25 cm tall) stands upright on a wooden
foot, its face towards the robot, 32 cm out, from the midline to 30 cm to the left. A black marker cross (4 cm
strokes) is drawn on it 12 cm left of the midline and 13 cm above the table. 12 cm closer to the robot, in line with the cross, a
white plastic adhesive hook (76 mm tall, 23 mm wide) stands in a slot in a small block: its foam adhesive strip on the
back faces the wall and its hook faces the robot; the strip's liner is already off. Stick the hook on the wall centred
on the cross and level: the strip grabs as soon as it touches, so it can't be slid into place afterwards. Then press
it firmly against the wall for 30 seconds and let go.
"""

# This task needs more than yam_arena's task-file format: a fixed mechanism with a joint (the wall panel on a stiff
# spring, which measures how hard the hook is pressed against it) and a free object made of several shapes (the hook
# with its strip), joined by a bond the task switches on when the strip touches. All of it is built here, from the task
# file, by hooking yam_arena when it is imported; nothing in yam_arena is modified.
#
# The mechanism code (Shape, Link, Joint, Mechanism, MechanismState and the hooks that build and reset it) is adapted
# from robocurve/benchmark-tasks pull request #223 ("Open a panel held shut by a spring plunger",
# tasks/spring-plunger-panel/mechanism.py), published under the MIT License:
#
#   MIT License
#
#   Copyright (c) 2026 the authors of this task
#
#   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
#   documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
#   rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
#   permit persons to whom the Software is furnished to do so, subject to the following conditions:
#
#   The above copyright notice and this permission notice shall be included in all copies or substantial portions of
#   the Software.
#
#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
#   WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
#   OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
#   OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#
# Changes from that file: shapes may be turned about the mechanism's y axis and may be spheres; a joint's frame may be
# turned about y; the top camera can be recorded; and the hook, the bond, the grippers and the scoring below.

import atexit
import importlib.abc
import importlib.util
import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
from yam_arena import demo as yam_demo
from yam_arena import ik, station
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, READY, SHOULDER, WRIST
from yam_arena.demo import Waypoint
from yam_arena.task_file import TABLE_TOP, Box, ObjectState
from yam_arena.urdf import axis_angle_matrix

SCENE = "station"
TOP = TABLE_TOP[SCENE]
MM = 0.001

# -- the wall and the hook: sizes ------------------------------------------------------------------------------------
# The hook: a 3M Command Medium Utility Hook (17001), white plastic, 76 mm tall, 23 mm wide, 25 mm deep, rated to hold
# 3 lb (1.36 kg) once its strip has bonded (3M's 17001 listing; 3M UK 17001-6: "76 mm" x "23 mm" x "25 mm").
# Its shape below is an estimate from product photos: a flat back plate, and a hook arm out from its lower part.
PLATE_H, PLATE_W, PLATE_T = 76.0 * MM, 23.0 * MM, 4.5 * MM
# The adhesive strip on its back (a Command medium strip): foam, 1.6 mm thick, 64 x 19 mm [estimate], centred on the
# plate, its liner already removed.
STRIP_H, STRIP_W, STRIP_T = 64.0 * MM, 19.0 * MM, 1.6 * MM
ARM_W, ARM_T = 8.0 * MM, 5.0 * MM  # the hook arm's width (y) and thickness [estimate]
ARM_OUT, ARM_Z0 = 19.0 * MM, 6.0 * MM  # how far it reaches out (25 mm deep in all), and how high above the bottom
LIP_H = 12.0 * MM  # its upturned lip
REAL_HOOK_KG = 0.008  # [estimate]
# WHY heavier than real: an 8 g body squeezed between finger pads makes the solver give (as for every small part here).
HOOK_KG = 0.03

# The wall: a 30 x 25 cm piece of 1/2 in (12.7 mm) drywall painted light blue, standing on a wooden foot, its face towards the
# robot (-x). Its face is at WALL_X; the marker cross marks where the plate's middle goes.
WALL_X = 0.32
WALL_W, WALL_H, WALL_T = 0.30, 0.25, 12.7 * MM
WALL_Y = 0.15  # the panel's middle (it spans y 0 to 0.30)
MARK = np.array([WALL_X, 0.12, TOP + 0.13])  # the cross's centre, on the face
# A marker cross, 40 mm strokes 3 mm wide. WHY not a thin pencil line: at the policy's 320 px the top camera sees about
# 5 mm a pixel, and a 1 mm line 2 cm long was nearly invisible (official sim, 2026-11-20).
MARK_LEN, MARK_LINE = 40.0 * MM, 3.0 * MM
# WHY the panel is on a stiff spring: so the joint's travel measures how hard the hook is pressed against it, as a force
# gauge behind the wall would. At 40,000 N/m the panel gives 0.25 mm under 10 N [sim].
WALL_K = 40000.0
WALL_TRAVEL = 2.0 * MM

# The bond. 3M's instructions for its Command hooks: press the hook "firmly for 30 seconds", then "Wait 1 hour for
# adhesive to build" before hanging anything (3M Command large hook sheet 17036; 17001-6 listing: wait 1 hour).
# Pressure-sensitive foam grabs on contact, so a hook can't be slid once it touches. Modelled so: the strip bonds to the
# wall (a joint switched on at their pose of the moment) when it lies flat on the wall within TOUCH_MM and is pushed
# with at least TACK_N; after that the task counts the seconds it is pressed with at least FIRM_N.
TOUCH_MM = 0.6  # three of the strip's four corners this close to the wall's face
TACK_N = 1.0
FIRM_N = 10.0  # "firmly": no published figure; a firm thumb press is 10-30 N [estimate]
PRESS_S = 30.0
# A step of the press counts only if the force goes through the hook: nothing but the hook may push on the panel. WHY:
# the gauge feels a push anywhere on the panel, so a hand pressing beside a lightly tacked hook would otherwise count
# (robocurve review of TK-EWQ42S, 2026-11-20). The panel's contacts are read from the simulator each step; whatever
# touches it other than the hook must push with less than OTHER_MAX_N.
OTHER_MAX_N = 0.5

# Where the hook starts: standing in a 5 mm deep slot in a small block, plate upright, strip towards the wall.
SLOT_DEPTH, SLOT_GAP = 5.0 * MM, 5.5 * MM
HOLDER_XY = (0.20, 0.12)

# WHY a blue wall: the white hook vanished against a light grey one on camera (official sim, 2026-11-20).
WHITE, GREY, DARK, WOOD, FOAM = (0.96, 0.96, 0.95), (0.45, 0.62, 0.80), (0.05, 0.05, 0.06), (0.55, 0.38, 0.22), \
    (0.85, 0.85, 0.90)
TAN = (0.70, 0.55, 0.35)
MATERIALS = {"default": (1.2, 1.0), "paint": (0.8, 0.6)}


# -- the mechanism kit (adapted from robocurve#223, MIT) ---------------------------------------------------------------
Color = tuple[float, float, float]
AXES = ("X", "Y", "Z")


@dataclass(frozen=True)
class Shape:
    """One collider, which is also what is drawn, in the mechanism's frame (m). ``kind`` "box": ``size`` is its x, y,
    z edges; "cylinder" and "capsule": (radius, length) along ``axis``; "sphere": (radius,). ``turn`` turns it about
    the mechanism's y axis (rad) around its own centre, after ``tilt`` turns a box about its own z."""

    kind: str
    center: tuple[float, float, float]
    size: tuple[float, ...]
    color: Color = (0.7, 0.7, 0.7)
    axis: str = "Z"
    material: str = "default"
    contact_offset: float = 0.002
    turn: float = 0.0
    tilt: float = 0.0  # a box only: turned about its own z axis (rad) before ``turn``

    def bounds(self) -> tuple[np.ndarray, np.ndarray]:
        c = np.asarray(self.center, float)
        if self.kind == "box":
            half = np.asarray(self.size, float) / 2
            if self.turn or self.tilt:  # bound it by its half-diagonal
                half = np.full(3, float(np.linalg.norm(half)))
        elif self.kind == "sphere":
            half = np.full(3, self.size[0])
        else:
            radius, length = self.size
            half = np.full(3, radius)
            half[AXES.index(self.axis)] = length / 2 + (radius if self.kind == "capsule" else 0.0)
        return c - half, c + half


@dataclass(frozen=True)
class Link:
    name: str
    mass: float
    shapes: tuple[Shape, ...]


@dataclass(frozen=True)
class Joint:
    """A joint from the base to ``child`` through ``anchor`` along ``axis`` of a frame turned ``turn`` (rad) about the
    mechanism's y axis. "revolute" (rad) or "prismatic" (m); a spring of ``stiffness`` pulls it to ``target``, which
    may lie beyond a stop to preload it there."""

    name: str
    kind: str
    child: str
    anchor: tuple[float, float, float]
    axis: str
    lower: float
    upper: float
    stiffness: float = 0.0
    damping: float = 0.0
    friction: float = 0.0
    target: float = 0.0
    turn: float = 0.0


@dataclass(frozen=True)
class Mechanism(Box):
    """A fixed mechanism, its frame at ``pos``; a ``Box`` only so yam_arena's checks accept it (no mass: fixed)."""

    base: Link | None = None
    links: tuple[Link, ...] = ()
    joints: tuple[Joint, ...] = ()
    materials: dict = field(default_factory=dict, compare=False, hash=False)

    def _check(self) -> None:
        super()._check()
        if self.base is None or self.mass is not None:
            raise ValueError(f"{self.name}: a Mechanism needs a base link and no mass (it is fixed)")
        names = [self.base.name] + [link.name for link in self.links]
        if len(set(names)) != len(names):
            raise ValueError(f"{self.name}: link names must be unique, got {names}")
        for link in (self.base, *self.links):
            if not link.shapes or not link.mass > 0:
                raise ValueError(f"{self.name}.{link.name}: a link needs shapes and a positive mass")
            for shape in link.shapes:
                if shape.kind not in ("box", "cylinder", "capsule", "sphere") or shape.axis not in AXES:
                    raise ValueError(f"{self.name}.{link.name}: unknown shape {shape.kind!r} or axis {shape.axis!r}")
                if shape.material not in self.materials:
                    raise ValueError(f"{self.name}.{link.name}: no friction given for material {shape.material!r}")
        for joint in self.joints:
            if joint.child not in names[1:] or joint.kind not in ("revolute", "prismatic") or joint.axis not in AXES:
                raise ValueError(f"{self.name}: bad joint {joint.name!r}")
            if not joint.lower < joint.upper:
                raise ValueError(f"{self.name}.{joint.name}: lower must be below upper")
        if sorted(j.child for j in self.joints) != sorted(names[1:]):
            raise ValueError(f"{self.name}: every moving link needs exactly one joint to the base")

    def all_links(self) -> tuple[Link, ...]:
        return (self.base, *self.links)


def mechanism(name: str, pos, base: Link, links, joints, materials: dict, yaw: float = 0.0,
              color: Color = (0.7, 0.7, 0.7)) -> Mechanism:
    lows, highs = zip(*(s.bounds() for link in (base, *links) for s in link.shapes))
    size = tuple((np.max(highs, axis=0) - np.min(lows, axis=0)).tolist())
    return Mechanism(name=name, pos=tuple(pos), size=size, yaw=yaw, color=color, base=base, links=tuple(links),
                     joints=tuple(joints), materials=dict(materials))


@dataclass(frozen=True)
class MechanismState(ObjectState):
    """The base's pose (``pos`` is its frame's origin) and each joint's position (m or rad, 0 at rest)."""

    joints: dict = field(default_factory=dict)


def _about_y(angle: float) -> tuple[float, float, float, float]:
    """wxyz quaternion turning ``angle`` (rad) about y."""
    return (math.cos(angle / 2), 0.0, math.sin(angle / 2), 0.0)


def _shape_quat(shape: "Shape") -> tuple[float, float, float, float]:
    """wxyz: the shape's tilt about its own z, then its turn about y."""
    w1, x1, y1, z1 = _about_y(shape.turn)
    w2, x2, y2, z2 = (math.cos(shape.tilt / 2), 0.0, 0.0, math.sin(shape.tilt / 2))
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


_SPECS: dict[str, Mechanism] = {}


def _spawn(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """Author the mechanism under ``prim_path``: an Xform (the articulation root) holding one rigid body per link,
    a fixed joint from the world to the base, and the joints. Bodies and shapes are in the mechanism's frame."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    spec: Mechanism = _SPECS[cfg.mechanism_name]
    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    root = UsdGeom.Xform.Define(stage, prim_path)
    root.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    root.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))

    physics: dict[str, str] = {}
    for name, (static, dynamic) in spec.materials.items():
        path = f"{prim_path}/Looks/physics_{name}"
        material = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
        material.func(path, material)
        physics[name] = path
    looks: dict[Color, str] = {}

    def look(color: Color) -> str:
        if color not in looks:
            path = f"{prim_path}/Looks/color_{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.4, metallic=0.3)
            surface.func(path, surface)
            looks[color] = path
        return looks[color]

    def collider(prim, shape: Shape) -> None:
        UsdPhysics.CollisionAPI.Apply(prim)
        physx = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physx.CreateContactOffsetAttr(shape.contact_offset)
        physx.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(prim.GetPath()), physics[shape.material], stage=stage)
        bind_visual_material(str(prim.GetPath()), look(shape.color), stage=stage)

    bodies = {}
    for link in spec.all_links():
        path = f"{prim_path}/{link.name}"
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp(precision=double).Set(Gf.Vec3d(0.0, 0.0, 0.0))
        body.AddOrientOp(precision=double).Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(link.mass)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateMaxDepenetrationVelocityAttr(1.0)
        for i, shape in enumerate(link.shapes):
            _author_shape(stage, f"{path}/{shape.kind}_{i}", shape, collider)
        if (spec.name, link.name) == (WALL_NAME, "panel"):
            # WHY: the task reads the panel's contacts to tell a press through the hook from one beside it
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.0)
            _PANEL_AT[0] = path
        bodies[link.name] = Sdf.Path(path)

    base_prim = stage.GetPrimAtPath(bodies[spec.base.name])
    to_world = UsdGeom.Xformable(base_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{prim_path}/{spec.base.name}/fixed_to_world")
    fixed.CreateBody1Rel().SetTargets([bodies[spec.base.name]])
    fixed.CreateLocalPos0Attr(Gf.Vec3f(to_world.ExtractTranslation()))
    fixed.CreateLocalRot0Attr(Gf.Quatf(to_world.ExtractRotationQuat()))
    fixed.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    fixed.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))

    for joint in spec.joints:
        path = f"{prim_path}/{joint.child}/{joint.name}"
        revolute = joint.kind == "revolute"
        usd_joint = (UsdPhysics.RevoluteJoint if revolute else UsdPhysics.PrismaticJoint).Define(stage, path)
        usd_joint.CreateAxisAttr(joint.axis)
        usd_joint.CreateBody0Rel().SetTargets([bodies[spec.base.name]])
        usd_joint.CreateBody1Rel().SetTargets([bodies[joint.child]])
        frame = Gf.Quatf(*_about_y(joint.turn))
        usd_joint.CreateLocalPos0Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot0Attr(frame)
        usd_joint.CreateLocalPos1Attr(Gf.Vec3f(*joint.anchor))
        usd_joint.CreateLocalRot1Attr(frame)
        unit = math.degrees(1.0) if revolute else 1.0
        usd_joint.CreateLowerLimitAttr(joint.lower * unit)
        usd_joint.CreateUpperLimitAttr(joint.upper * unit)
        drive = UsdPhysics.DriveAPI.Apply(usd_joint.GetPrim(), "angular" if revolute else "linear")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(joint.stiffness / unit)
        drive.CreateDampingAttr(joint.damping / unit)
        drive.CreateTargetPositionAttr(joint.target * unit)

    # The moving links collide with one another but not with the base, which they rest against or slide in.
    UsdPhysics.FilteredPairsAPI.Apply(base_prim).CreateFilteredPairsRel().SetTargets(
        [bodies[link.name] for link in spec.links])
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(root.GetPrim())
    articulation.CreateEnabledSelfCollisionsAttr(True)
    articulation.CreateSolverPositionIterationCountAttr(64)
    articulation.CreateSolverVelocityIterationCountAttr(4)
    return root.GetPrim()


def _author_shape(stage, path: str, shape: Shape, collider) -> None:
    from pxr import Gf, UsdGeom

    if shape.kind == "box":
        geom = UsdGeom.Cube.Define(stage, path)
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
        if shape.turn or shape.tilt:
            geom.AddOrientOp().Set(Gf.Quatf(*_shape_quat(shape)))
        geom.AddScaleOp().Set(Gf.Vec3f(*shape.size))
    elif shape.kind == "sphere":
        geom = UsdGeom.Sphere.Define(stage, path)
        geom.CreateRadiusAttr(shape.size[0])
        geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
    else:
        geom = (UsdGeom.Cylinder if shape.kind == "cylinder" else UsdGeom.Capsule).Define(stage, path)
        geom.CreateAxisAttr(shape.axis)
        geom.CreateRadiusAttr(shape.size[0])
        geom.CreateHeightAttr(shape.size[1])
        geom.AddTranslateOp().Set(Gf.Vec3d(*shape.center))
    collider(geom.GetPrim(), shape)


def _arena_object(spec: Mechanism):
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    _SPECS[spec.name] = spec

    @configclass
    class MechanismSpawnerCfg(SpawnerCfg):
        # WHY: configclass looks the class's module up in sys.modules, and yam_arena loads task files without
        # registering them there ("'NoneType' object has no attribute '__dict__'", official sim, 2026-11-20).
        __module__ = "yam_arena.scene"
        func: object = clone(_spawn)
        mechanism_name: str = ""

    class MechanismObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {
                joint.name: ImplicitActuatorCfg(joint_names_expr=[joint.name], stiffness=joint.stiffness,
                                                damping=joint.damping, friction=joint.friction)
                for joint in spec.joints
            }
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return MechanismObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=MechanismSpawnerCfg(mechanism_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos),
                          rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2), math.cos(spec.yaw / 2))),
    )


def _torch(x):
    return getattr(x, "torch", x)  # Isaac Lab 3.0 wraps some buffers


def _numpy(x) -> np.ndarray:
    x = _torch(x)
    if hasattr(x, "detach"):  # a torch tensor
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x.numpy() if hasattr(x, "numpy") else x, dtype=np.float64)  # a warp array (the contact views)


def _joint_ids(asset, spec: Mechanism) -> list[int]:
    names = list(asset.joint_names)
    return [names.index(joint.name) for joint in spec.joints]


def _mechanism_state(environment, spec: Mechanism) -> MechanismState:
    base = environment.unwrapped
    asset = base.scene[spec.name]
    origin = _numpy(base.scene.env_origins[0])
    pose = _numpy(asset.data.root_link_pose_w)[0]
    q = _numpy(asset.data.joint_pos)[0]
    joints = {joint.name: float(q[i]) for joint, i in zip(spec.joints, _joint_ids(asset, spec), strict=True)}
    lows, highs = zip(*(s.bounds() for link in spec.all_links() for s in link.shapes))
    return MechanismState(pos=pose[:3] - origin, quat_xyzw=pose[3:7], lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                          low=pose[:3] - origin + np.min(lows, axis=0), high=pose[:3] - origin + np.max(highs, axis=0),
                          joints=joints)


def _rest(environment, spec: Mechanism) -> None:
    """Every joint back to 0 and still, and each spring's target set (a target beyond a stop preloads it)."""
    import torch

    asset = environment.unwrapped.scene[spec.name]
    ids = _joint_ids(asset, spec)
    device = _torch(asset.data.joint_pos).device
    zeros = torch.zeros((1, len(ids)), dtype=torch.float32, device=device)
    targets = torch.tensor([[joint.target for joint in spec.joints]], dtype=torch.float32, device=device)
    if hasattr(asset, "write_joint_state_to_sim_index"):  # Isaac Lab 3.0
        asset.write_joint_state_to_sim_index(position=zeros, velocity=zeros, joint_ids=ids)
        asset.set_joint_position_target_index(target=targets, joint_ids=ids)
        asset.set_joint_velocity_target_index(target=zeros, joint_ids=ids)
    else:  # Isaac Lab 2.x
        asset.write_joint_state_to_sim(zeros, zeros, joint_ids=ids)
        asset.set_joint_position_target(targets, joint_ids=ids)
        asset.set_joint_velocity_target(zeros, joint_ids=ids)
    asset.write_data_to_sim()




# -- the wall as a mechanism, and the holder ----------------------------------------------------------------------------
WALL_NAME = "wall"
_PANEL_AT = [""]  # the panel's prim path, once spawned


def _wall() -> Mechanism:
    """Mechanism frame: origin on the table under the middle of the wall's face, x into the wall."""
    foot = Shape("box", (0.045, 0.0, 0.01), (0.09, WALL_W, 0.02), WOOD)
    brace = Shape("box", (WALL_T + 0.01, 0.0, 0.06), (0.02, WALL_W, 0.08), WOOD)
    base = Link("foot", 2.0, (foot, brace))
    mark_y, mark_z = MARK[1] - WALL_Y, MARK[2] - TOP
    panel = (
        Shape("box", (WALL_T / 2, 0.0, 0.02 + WALL_H / 2), (WALL_T, WALL_W, WALL_H), GREY, material="paint"),
        # the marker cross, 0.1 mm proud of the face
        Shape("box", (-0.05 * MM, mark_y, mark_z), (0.1 * MM, MARK_LEN, MARK_LINE), DARK, material="paint"),
        Shape("box", (-0.05 * MM, mark_y, mark_z), (0.1 * MM, MARK_LINE, MARK_LEN), DARK, material="paint"),
    )
    links = [Link("panel", 0.75, panel)]
    joints = [Joint("panel_give", "prismatic", "panel", (0.0, 0.0, 0.02 + WALL_H / 2), "X", 0.0, WALL_TRAVEL,
                    stiffness=WALL_K, damping=50.0, target=0.0)]
    return mechanism(WALL_NAME, (WALL_X, WALL_Y, TOP), base, links, joints, MATERIALS, color=GREY)


WALL = _wall()
# The panel's face sits 2 cm up on the foot: WALL_H tall from TOP + 0.02.

# -- the hook: one free body made of several boxes ----------------------------------------------------------------------
# Its frame: origin at the middle of the strip's sticky face, x out of that face (towards the wall when it is on it),
# z up the plate.
PLATE_FRONT = -(STRIP_T + PLATE_T)  # the plate's front face (x)
ARM_TOP = -PLATE_H / 2 + ARM_Z0 + ARM_T


def hook_boxes() -> list[tuple[str, tuple, tuple, tuple]]:
    """(name, centre, size, colour) of each box of the hook, in its frame."""
    return [
        ("strip", (-STRIP_T / 2, 0.0, 0.0), (STRIP_T, STRIP_W, STRIP_H), FOAM),
        ("plate", (-STRIP_T - PLATE_T / 2, 0.0, 0.0), (PLATE_T, PLATE_W, PLATE_H), WHITE),
        ("arm", (PLATE_FRONT - ARM_OUT / 2, 0.0, -PLATE_H / 2 + ARM_Z0 + ARM_T / 2), (ARM_OUT, ARM_W, ARM_T), WHITE),
        ("lip", (PLATE_FRONT - ARM_OUT + 2.5 * MM, 0.0, ARM_TOP + LIP_H / 2), (5.0 * MM, ARM_W, LIP_H), WHITE),
    ]


def strip_corners() -> np.ndarray:
    """The sticky face's four corners, hook frame."""
    return np.array([[0.0, sy * STRIP_W / 2, sz * STRIP_H / 2] for sy in (-1, 1) for sz in (-1, 1)])


@dataclass(frozen=True)
class CompoundBody(Box):
    """A free body made of boxes (hook_boxes()); ``pos`` is its frame's origin, ``size`` its rough extent."""


SLOT_X = HOLDER_XY[0]  # the slot's middle
HOOK_START = (SLOT_X + PLATE_T / 2 + STRIP_T, HOLDER_XY[1], TOP + PLATE_H / 2 + 0.0005)
HOOK = CompoundBody("hook", HOOK_START, color=WHITE, mass=HOOK_KG, size=(0.026, PLATE_W, PLATE_H))
_block_x = (SLOT_GAP + 0.012) / 2  # two 12 mm blocks either side of the slot
HOLDER = [
    Box("holder_front", (SLOT_X - _block_x, HOLDER_XY[1], TOP + SLOT_DEPTH / 2), size=(0.012, 0.04, SLOT_DEPTH), color=TAN),
    Box("holder_back", (SLOT_X + _block_x, HOLDER_XY[1], TOP + SLOT_DEPTH / 2), size=(0.012, 0.04, SLOT_DEPTH), color=TAN),
]
OBJECTS = [WALL, *HOLDER, HOOK]
BOND_JOINT = "hook_bond"


def _spawn_hook(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    """The hook as one rigid body with a collider per box, and its bond to the wall panel: a fixed joint, switched off
    until the strip touches (the wall is spawned first)."""
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    double = UsdGeom.XformOp.PrecisionDouble
    xf = UsdGeom.Xform.Define(stage, prim_path)
    xf.AddTranslateOp(precision=double).Set(Gf.Vec3d(*(translation if translation is not None else (0, 0, 0))))
    x, y, z, w = orientation if orientation is not None else (0.0, 0.0, 0.0, 1.0)
    xf.AddOrientOp(precision=double).Set(Gf.Quatd(w, x, y, z))
    prim = xf.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(HOOK_KG)
    body = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    body.CreateSolverPositionIterationCountAttr(32)
    body.CreateSolverVelocityIterationCountAttr(4)
    body.CreateMaxDepenetrationVelocityAttr(1.0)
    materials = {}
    # WHY the strip's friction is high: pressure-sensitive foam is tacky; it can't be slid along the wall [sim, estimate]
    for kind, (static, dynamic) in (("plastic", (0.6, 0.5)), ("foam", (2.0, 2.0))):
        materials[kind] = f"{prim_path}/physics_{kind}"
        cfg_m = sim_utils.RigidBodyMaterialCfg(static_friction=static, dynamic_friction=dynamic)
        cfg_m.func(materials[kind], cfg_m)
    looks = {}
    for name, centre, size, color in hook_boxes():
        geom = UsdGeom.Cube.Define(stage, f"{prim_path}/{name}")
        geom.CreateSizeAttr(1.0)
        geom.AddTranslateOp().Set(Gf.Vec3d(*centre))
        geom.AddScaleOp().Set(Gf.Vec3f(*size))
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        physx = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
        physx.CreateContactOffsetAttr(0.002)
        physx.CreateRestOffsetAttr(0.0)
        bind_physics_material(str(geom.GetPath()), materials["foam" if name == "strip" else "plastic"], stage=stage)
        if color not in looks:
            looks[color] = f"{prim_path}/look_{len(looks)}"
            surface = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            surface.func(looks[color], surface)
        bind_visual_material(str(geom.GetPath()), looks[color], stage=stage)
    parent = prim_path.rsplit("/", 1)[0]
    panel = f"{parent}/{WALL.name}/panel"
    if not stage.GetPrimAtPath(panel).IsValid():
        raise RuntimeError(f"hang_command_hook: {panel} doesn't exist when the hook's bond is made")
    joint = UsdPhysics.FixedJoint.Define(stage, f"{parent}/{BOND_JOINT}")
    joint.CreateBody0Rel().SetTargets([Sdf.Path(panel)])
    joint.CreateBody1Rel().SetTargets([Sdf.Path(prim_path)])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.CreateJointEnabledAttr(False)
    # WHY: by default bodies joined by a joint don't collide; the strip has to meet the wall before it bonds
    joint.CreateCollisionEnabledAttr(True)
    _BOND_AT[0] = f"{parent}/{BOND_JOINT}"
    _HOOK_AT[0] = prim_path
    return prim


_BOND_AT = [""]
_HOOK_AT = [""]


def _hook_spawner():
    from isaaclab.sim.utils import clone

    return clone(_spawn_hook)


def _body_state(environment, name: str) -> ObjectState:
    base = environment.unwrapped
    obj = base.scene[name]
    origin = _numpy(base.scene.env_origins[0])
    pose = _numpy(obj.data.root_link_pose_w)[0]
    lin = _numpy(getattr(obj.data, "root_link_lin_vel_w", obj.data.root_lin_vel_w))[0]
    ang = _numpy(getattr(obj.data, "root_link_ang_vel_w", obj.data.root_ang_vel_w))[0]
    p = pose[:3] - origin
    half = np.full(3, PLATE_H / 2)
    return ObjectState(pos=p, quat_xyzw=pose[3:7], lin_vel=lin, ang_vel=ang, low=p - half, high=p + half)


# -- the grippers, for the hands-off check (as in the accepted flashlight tasks) ---------------------------------------
@dataclass(frozen=True)
class Pads:
    """One gripper after a control step (world frame, m): the span along its pointing axis from the pads' back to
    their tips, and points across the end of each of its two fingertips."""

    back: np.ndarray
    tips: np.ndarray
    fingertips: tuple = ()


PAD_BACK = 0.025
GRIPPERS = ("left_gripper", "right_gripper")
FINGERTIP_ENDS = {
    "tip_left": [(x, -0.04454, -0.09016) for x in (0.01799, 0.02399, 0.02999)],
    "tip_right": [(x, 0.04454, -0.09016) for x in (-0.02999, -0.02399, -0.01799)],
}


def _gripper_states(env) -> dict:
    from yam_arena import runtime

    base = env.unwrapped
    origin = _numpy(base.scene.env_origins[0])
    out = {}
    for side in ("left", "right"):
        robot = base.scene[f"{side}_robot"]
        poses = runtime.link_poses(robot)
        axis = poses["gripper"][:3, 2]
        tcp = runtime.tcp_world(robot) - origin
        ends = tuple(poses[link][:3, :3] @ np.array(p) + poses[link][:3, 3] - origin
                     for link, points in FINGERTIP_ENDS.items() for p in points)
        out[f"{side}_gripper"] = Pads(back=tcp - PAD_BACK * axis, tips=tcp + PAD_TIP_PAST_TCP * axis, fingertips=ends)
    return out



def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])



# -- the bond: the strip grabs the wall when it touches it flat under pressure, and the task times the firm press ------
def _quat_to_wxyz(r: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = ik.matrix_to_quat_xyzw(r)
    return float(w), float(x), float(y), float(z)


def wall_force(objects) -> float:
    """How hard the wall is being pushed (N): its spring's travel times its stiffness."""
    return max(0.0, objects[WALL.name].joints["panel_give"]) * WALL_K


def panel_pose(objects) -> tuple[np.ndarray, np.ndarray]:
    """The panel link's frame (world): the mechanism's frame moved by the panel's travel along its x."""
    w = objects[WALL.name]
    r = _rot(w.quat_xyzw)
    return np.asarray(w.pos, float) + r @ np.array([objects[WALL.name].joints["panel_give"], 0.0, 0.0]), r


def corner_gaps_mm(objects) -> np.ndarray:
    """Each corner of the strip's sticky face: how far in front of the wall's face it is (mm; 0 or less: touching)."""
    h = objects[HOOK.name]
    corners = np.asarray(h.pos, float) + strip_corners() @ _rot(h.quat_xyzw).T
    p, r = panel_pose(objects)
    return -((corners - p) @ r[:, 0]) * 1000


class _PanelContacts:
    """The panel's contact forces from the simulator: in all, and from the hook alone. The difference is what anything
    else (a finger, a wrist, the other arm) pushes on the panel with."""

    def __init__(self) -> None:
        self.view, self.failed = None, False

    def other_n(self, env) -> float:
        """The force (N) on the panel from everything but the hook, at the last physics step; inf if it can't be
        read, so that a press can't count unseen."""
        if self.failed or not (_PANEL_AT[0] and _HOOK_AT[0]):
            return math.inf
        try:
            if self.view is None:
                from isaaclab_physx.physics import PhysxManager

                self.view = PhysxManager.get_physics_sim_view().create_rigid_contact_view(
                    [_PANEL_AT[0]], filter_patterns=[[_HOOK_AT[0]]], max_contact_data_count=256)
            dt = env.unwrapped.physics_dt
            net = _numpy(self.view.get_net_contact_forces(dt=dt)).reshape(-1)[:3]
            hook = _numpy(self.view.get_contact_force_matrix(dt=dt)).reshape(-1)[:3]
            return float(np.linalg.norm(net - hook))
        except Exception as error:  # noqa: BLE001
            print(f"hang_command_hook: can't read the panel's contacts ({error!r}); no press will count")
            self.failed = True
            return math.inf


_CONTACTS = _PanelContacts()


@dataclass(frozen=True)
class BondState:
    bonded: bool = False
    firm_s: float = 0.0  # seconds pressed with at least FIRM_N through the hook, since it bonded
    force_n: float = 0.0  # the panel's spring force
    other_n: float = 0.0  # the force on the panel from anything but the hook


class _Bond:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.bonded, self.firm_steps = False, 0
        if _BOND_AT[0]:
            _set_bond(None)

    def update(self, env, objects) -> BondState:
        force = wall_force(objects)
        other = float(objects.get("panel_other_n", 0.0))
        if not self.bonded:
            touching = int(np.sum(corner_gaps_mm(objects) <= TOUCH_MM))
            if touching >= 3 and force >= TACK_N:
                p, r = panel_pose(objects)
                h = objects[HOOK.name]
                _set_bond((r.T @ (np.asarray(h.pos, float) - p), r.T @ _rot(h.quat_xyzw)))
                self.bonded = True
        elif force >= FIRM_N and other < OTHER_MAX_N:
            self.firm_steps += 1
        return BondState(self.bonded, self.firm_steps * env.unwrapped.step_dt, force, other)


def _set_bond(rel) -> None:
    """Switch the bond on with the hook held at ``rel`` = (position, rotation) in the panel's frame, or off (None)."""
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, UsdPhysics

    prim = get_current_stage().GetPrimAtPath(_BOND_AT[0])
    if not prim.IsValid():
        return
    joint = UsdPhysics.Joint(prim)
    if rel is None:
        joint.GetJointEnabledAttr().Set(False)
        return
    pos, r = rel
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in pos]))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(*_quat_to_wxyz(r)))
    joint.GetLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.GetLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    joint.GetJointEnabledAttr().Set(True)


_BOND = _Bond()


# -- hooking yam_arena -------------------------------------------------------------------------------------------------
def _patch_scene(scene) -> None:
    if getattr(scene, "_hook_patched", False):
        return
    stock_object, stock_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Mechanism):
            return _arena_object(spec)
        obj = stock_object(spec)
        if isinstance(spec, CompoundBody):
            # WHY both: Arena builds the object's RigidObjectCfg (a copy of the spawner config) when it is made
            func = _hook_spawner()
            obj.spawner_cfg.func = func
            if getattr(getattr(obj, "object_cfg", None), "spawn", None) is not None:
                obj.object_cfg.spawn.func = func
        return obj

    def object_states(env, names):
        others = [n for n in names if n not in _SPECS and n != HOOK.name]
        out = stock_states(env, others) if others else {}
        out.update({n: _mechanism_state(env, _SPECS[n]) for n in names if n in _SPECS})
        if HOOK.name in names:
            out[HOOK.name] = _body_state(env, HOOK.name)
        out.update(_gripper_states(env))
        if WALL.name in out and HOOK.name in out:
            out["panel_other_n"] = _CONTACTS.other_n(env)
            out["bond"] = _BOND.update(env, out)
        return out

    scene.task_object, scene.object_states = task_object, object_states
    scene._hook_patched = True


def _patch_embodiment(module) -> None:
    cls = module.YamArenaEmbodiment
    if getattr(cls, "_hook_patched", False):
        return
    reset, step, close = cls.reset, cls.step, cls.close

    def patched_reset(self, scene, *, seed=None):
        observation = reset(self, scene, seed=seed)
        _BOND.reset()
        specs = [o for o in (self.task.objects if self.task is not None else ()) if isinstance(o, Mechanism)]
        if specs:
            for spec in specs:
                _rest(self._env, _SPECS.get(spec.name, spec))
            base = self._base
            for _ in range(max(1, round(module.SETTLE_S / base.step_dt))):
                self._env.step(self._ready)
            for _ in range(module.RESET_RENDERS):
                base.sim.render()
            self._t = 0.0
            observation = self._observation()
        _RECORDER.start(self)
        return observation

    def patched_step(self, action):
        result = step(self, action)
        _RECORDER.add(self)
        return result

    def patched_close(self):
        _RECORDER.close()
        return close(self)

    cls.reset, cls.step, cls.close = patched_reset, patched_step, patched_close
    cls._hook_patched = True


VIDEO_DIR = "HOOK_VIDEO"  # environment variable: record the top camera at full size into this folder
CAMERAS = ("top_cam",)


class _Recorder:
    """With ``HOOK_VIDEO=<folder>`` set, each episode's top camera goes to <folder>/top_cam.mp4 (H.264) at its full
    render size (640 x 360) and the control rate, from the frame at the reset; its first frame also to setup.png."""

    def __init__(self) -> None:
        self.writers, self._exit_hook = {}, False

    def _frame(self, embodiment, cam: str) -> np.ndarray:
        out = _torch(embodiment._base.scene[cam].data.output["rgb"])
        return np.ascontiguousarray(out[0, ..., :3].cpu().numpy(), dtype=np.uint8)

    def start(self, embodiment) -> None:
        self.close()
        folder = os.environ.get(VIDEO_DIR)
        if not folder:
            return
        import imageio.v2 as imageio
        import imageio_ffmpeg

        folder = os.path.abspath(os.path.expanduser(folder))
        os.makedirs(folder, exist_ok=True)
        hz = round(1.0 / embodiment._base.step_dt, 6)
        for cam in CAMERAS:
            frame = self._frame(embodiment, cam)
            if cam == "top_cam":
                imageio.imwrite(os.path.join(folder, "setup.png"), frame)
            h, w = frame.shape[:2]
            writer = imageio_ffmpeg.write_frames(
                os.path.join(folder, f"{cam}.mp4"), (w, h), pix_fmt_in="rgb24", pix_fmt_out="yuv420p", fps=hz,
                quality=None, codec="libx264", macro_block_size=8, output_params=["-crf", "18", "-movflags", "+faststart"])
            writer.send(None)
            writer.send(frame)
            self.writers[cam] = writer
        if not self._exit_hook:
            atexit.register(self.close)
            self._exit_hook = True

    def add(self, embodiment) -> None:
        for cam, writer in self.writers.items():
            writer.send(self._frame(embodiment, cam))

    def close(self) -> None:
        writers, self.writers = self.writers, {}
        for writer in writers.values():
            writer.close()


_RECORDER = _Recorder()


_PATCHES = {"yam_arena.scene": _patch_scene, "yam_arena.inspect_embodiment": _patch_embodiment}


class _AfterImport(importlib.abc.MetaPathFinder):
    """Patches the modules in ``_PATCHES`` right after each is first imported (they need Isaac Sim, which starts
    later)."""

    def find_spec(self, fullname, path, target=None):
        if fullname not in _PATCHES:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None:
                break
        else:
            return None
        if spec.loader is None:
            return None
        run, patch = spec.loader.exec_module, _PATCHES[fullname]

        def exec_module(module):
            run(module)
            patch(module)

        spec.loader.exec_module = exec_module
        return spec


for _name, _patch in _PATCHES.items():
    if _name in sys.modules:
        _patch(sys.modules[_name])
if not any(isinstance(f, _AfterImport) for f in sys.meta_path):
    sys.meta_path.insert(0, _AfterImport())




# -- scoring: from the bond (on or off, and how long it was pressed firmly), the hook's pose on the wall and the grippers
ON_MARK_MM, LEVEL_DEG = 5.0, 3.0  # full credit: the plate's middle within 5 mm of the cross, level within 3 degrees
NEAR_MM, NEAR_DEG = 15.0, 8.0  # 0.5: within 15 mm and 8 degrees
CLEAR = 0.01  # hands off: no point of either gripper's pads or fingertips within 1 cm of the hook
COMPLETE_SCORE = 0.9  # score() for a finished hook; 1.0 comes only from success() holding 1 s, which ends the trial


@dataclass(frozen=True)
class Placement:
    dy_mm: float  # the plate's middle from the cross, along the wall (+: to the robot's left)
    dz_mm: float  # and up
    tilt_deg: float  # the plate's turn in the wall's plane from upright

    @property
    def off_mm(self) -> float:
        return math.hypot(self.dy_mm, self.dz_mm)


def placement(objects) -> Placement:
    h = objects[HOOK.name]
    r = _rot(h.quat_xyzw)
    p, rw = panel_pose(objects)
    face = rw.T @ (np.asarray(h.pos, float) - p)  # wall frame: y along it, z up
    mark = rw.T @ (MARK - np.asarray(objects[WALL.name].pos, float))
    up = rw.T @ r[:, 2]
    return Placement(dy_mm=(face[1] - mark[1]) * 1000, dz_mm=(face[2] - mark[2]) * 1000,
                     tilt_deg=math.degrees(math.atan2(up[1], up[2])))


def _near_hook(objects, point) -> bool:
    h = objects[HOOK.name]
    local = _rot(h.quat_xyzw).T @ (np.asarray(point, float) - np.asarray(h.pos, float))
    lo = np.array([PLATE_FRONT - ARM_OUT, -PLATE_W / 2, -PLATE_H / 2]) - CLEAR
    hi = np.array([0.0, PLATE_W / 2, PLATE_H / 2]) + CLEAR
    return bool(np.all(local >= lo) and np.all(local <= hi))


def _points(pads: Pads):
    along = [pads.back + s * (pads.tips - pads.back) for s in np.linspace(0.0, 1.0, 11)]
    return (*along, *(np.asarray(q, float) for q in pads.fingertips))


def hands_off(objects) -> bool:
    pads = [objects.get(g) for g in GRIPPERS]
    if any(p is None for p in pads):
        return False
    return not any(_near_hook(objects, q) for p in pads for q in _points(p))


def grade(objects) -> float:
    """1.0: stuck on the wall with the plate's middle within 5 mm of the cross and level within 3 degrees, pressed with
    at least 10 N for 30 s in all, and both hands clear. 0.75: stuck on the mark and level, but pressed firmly for less
    than 30 s or a hand still on it. 0.5: stuck within 15 mm and 8 degrees. 0.25: stuck anywhere else on the wall. 0:
    not stuck to the wall."""
    bond = objects.get("bond", BondState())
    if not bond.bonded:
        return 0.0
    s = placement(objects)
    if s.off_mm <= ON_MARK_MM and abs(s.tilt_deg) <= LEVEL_DEG:
        return 1.0 if bond.firm_s >= PRESS_S - 1e-9 and hands_off(objects) else 0.75
    if s.off_mm <= NEAR_MM and abs(s.tilt_deg) <= NEAR_DEG:
        return 0.5
    return 0.25


def _debug(objects, g) -> None:
    """With HOOK_DEBUG=path set, one line per control step: the bond, the press and the hook's place on the wall."""
    path = os.environ.get("HOOK_DEBUG")
    if not path:
        return
    bond = objects.get("bond", BondState())
    s = placement(objects)
    h = objects[HOOK.name]

    def tips(name):
        pads = objects.get(name)
        return "?" if pads is None else "(%.0f,%.0f,%.0f)" % tuple(np.asarray(pads.tips, float) * 1000)

    with open(path, "a") as f:
        f.write(f"grade={g} bonded={bond.bonded} force_n={bond.force_n:.2f} other_n={bond.other_n:.2f} "
                f"firm_s={bond.firm_s:.2f} "
                f"gaps_mm={[round(float(v), 2) for v in corner_gaps_mm(objects)]} dy_mm={s.dy_mm:.2f} "
                f"dz_mm={s.dz_mm:.2f} tilt_deg={s.tilt_deg:.2f} hands_off={hands_off(objects)} "
                f"hook=({h.pos[0] * 1000:.1f},{h.pos[1] * 1000:.1f},{h.pos[2] * 1000:.1f}) "
                f"left_tips={tips('left_gripper')} right_tips={tips('right_gripper')}\n")


def score(objects) -> float:
    g = grade(objects)
    _debug(objects, g)
    return min(g, COMPLETE_SCORE)


def success(objects) -> bool:
    return grade(objects) == 1.0


# -- the scripted demo -------------------------------------------------------------------------------------------------
# The left hand, tipped 20 degrees below level and pointing at the wall, pinches the hook's plate by its two side edges
# near its fingertips (the strip just ahead of them), lifts it out of its slot, carries it to 1 cm in front of the
# cross, moves it straight in until the strip touches, presses it with 15 N for 34 s, opens and backs away.
PITCH = math.radians(70.0)  # the hand's approach turned from straight down towards the wall
# WHY not level (90): pointing straight at the wall, the arm can't reach below 14 cm (IK off by 7-36 mm, checked
# 2026-11-20); tipped to 60-75 degrees it reaches everywhere it needs to.
TIP_CLEAR = 2.0 * MM  # the strip's face this far ahead of the most forward point of the fingertips
SQUEEZE = 0.003
PINCH = -(PLATE_W / 2 - SQUEEZE)
PRESS_N = 15.0
JOINT_K = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
WEIGHT = np.array([0.0, 0.0, -HOOK_KG * 9.81])
PARK_LEFT = (0.20, 0.30, TOP + 0.15)


def _allow_matrix_grips() -> None:
    """Let a Waypoint's ``yaw`` be a full 3x3 TCP rotation instead of an angle (yam_arena.demo builds each waypoint's
    rotation with grip_rotation(yaw), which this wraps)."""
    if getattr(yam_demo, "_matrix_grips", False):
        return
    plain = yam_demo.grip_rotation

    def grip_rotation(yaw=0.0):
        return np.asarray(yaw, float) if np.ndim(yaw) == 2 else plain(yaw)

    yam_demo.grip_rotation = grip_rotation
    yam_demo._matrix_grips = True


_allow_matrix_grips()


def _rot_y(angle: float) -> np.ndarray:
    return axis_angle_matrix(np.array([0.0, 1.0, 0.0]), angle)


# Straight down with the fingers closing along y, then tipped about y so the hand points at the wall (+x).
GRIP = _rot_y(-PITCH) @ yam_demo.grip_rotation(0.0)


def compliance(arm: str, tcp, rot: np.ndarray) -> np.ndarray:
    """The arm's 3x3 compliance at its TCP (m/N) with the hand at ``tcp`` turned ``rot``: J K^-1 J^T, from the arm's
    joint gains (80 N m/rad at the shoulder joints, 10 at the wrist)."""
    base = np.asarray(station.BASES[arm], float)
    q, _, _ = ik.solve(np.asarray(tcp, float), ik.matrix_to_quat_xyzw(rot), np.array(READY, float), base_pos=base,
                       iters=200)
    jac = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        jac[:, i] = (ik.tcp_pose(q + dq, base)[0] - ik.tcp_pose(q - dq, base)[0]) / 2e-6
    return jac @ np.linalg.inv(JOINT_K) @ jac.T


def aim(arm: str, tcp, rot: np.ndarray, force_on_hand=(0.0, 0.0, 0.0)):
    """Where to aim the TCP so that, meeting ``force_on_hand`` (N, world), it settles at ``tcp``. WHY: the arm's joints
    are soft position servos, so a hand that meets a force settles C F from where it is aimed."""
    tcp = np.asarray(tcp, float)
    return tuple(float(v) for v in tcp - compliance(arm, tcp, rot) @ np.asarray(force_on_hand, float))


def tcp_for(origin) -> np.ndarray:
    """The TCP that holds the hook (its frame upright, sticky face towards +x) with that face's middle at ``origin``:
    the fingertips' ends level with the plate's middle and TIP_CLEAR behind the face at their most forward point."""
    a = GRIP[:, 2]  # the hand's approach: towards the wall and a little down
    b = np.cross(a, np.array([0.0, 1.0, 0.0]))
    b /= np.linalg.norm(b)  # across the pads' width, in the x-z plane
    reach = PAD_TIP_PAST_TCP * a[0] + 0.013 * abs(b[0])  # the fingertips' most forward point, ahead of the TCP (x)
    tips_mid = np.asarray(origin, float) + np.array([-(TIP_CLEAR + reach - PAD_TIP_PAST_TCP * a[0]), 0.0, 0.0])
    return tips_mid - PAD_TIP_PAST_TCP * a


def _w(duration, tcp, finger, label, force=(0.0, 0.0, 0.0)) -> Waypoint:
    return Waypoint(duration, aim("left", tcp, GRIP, force), finger, label, GRIP)


def _plan(mode: str = "demo"):
    """The demo, or a wrong way: HOOK_DEMO=brief (pressed 5 s, not 30), HOOK_DEMO=off (stuck 12 mm to the right of the
    cross), HOOK_DEMO=beside (tacked on the mark, let go, then the wall pressed 45 mm beside the hook for 34 s)."""
    if mode == "beside":
        return _plan_beside()
    start = np.asarray(HOOK_START, float)
    target = MARK + (np.array([0.0, -0.012, 0.0]) if mode == "off" else 0.0)
    pre = target + np.array([-0.010, 0.0, 0.0])
    touch = target + np.array([0.0005, 0.0, 0.0])  # aimed 0.5 mm into the wall: the face meets it on the way
    press_s = 5.0 if mode == "brief" else PRESS_S + 4.0
    press = WEIGHT + PRESS_N * np.array([-1.0, 0.0, 0.0])  # the wall pushes back towards the robot
    up = np.array([0.0, 0.0, 0.05])
    return [
        _w(2.0, tcp_for(start) + up, FINGER_OPEN, "over the hook in its slot"),
        _w(1.5, tcp_for(start), FINGER_OPEN, "down either side of its plate"),
        _w(1.2, tcp_for(start), PINCH, "pinch the plate's edges"),
        _w(0.5, tcp_for(start), PINCH, "hold"),
        _w(1.2, tcp_for(start) + up, PINCH, "lift it out of the slot", WEIGHT),
        _w(2.5, tcp_for(pre), PINCH, "carry it to 1 cm in front of the cross", WEIGHT),
        _w(1.0, tcp_for(pre), PINCH, "line it up, level", WEIGHT),
        _w(1.5, tcp_for(touch), PINCH, "straight in until the strip touches", WEIGHT),
        _w(2.0, tcp_for(touch), PINCH, "press firmly", press),
        _w(press_s, tcp_for(touch), PINCH, "keep pressing", press),
        _w(1.0, tcp_for(touch), PINCH, "ease off"),
        _w(0.6, tcp_for(touch), FINGER_OPEN, "let go"),
        _w(1.2, tcp_for(touch) - 0.06 * GRIP[:, 2], FINGER_OPEN, "back the fingers away"),
        Waypoint(1.5, PARK_LEFT, FINGER_OPEN, "clear", 0.0),
        Waypoint(6.0, PARK_LEFT, FINGER_OPEN, "stay", 0.0),
    ]


BESIDE = MARK + np.array([0.0, -0.045, 0.0])  # on the wall, 45 mm to the robot's right of the cross
LIGHT_N = 3.0  # the beside probe's light push on the hook: enough to bond (TACK_N), well under firm (FIRM_N)


def _plan_beside():
    """A wrong way: the strip tacked on the mark with a light touch, the hook let go, then the closed fingertips pressed
    against the bare wall 45 mm beside it with 15 N for 34 s. The wall's gauge feels the press, but it isn't through
    the hook."""
    steps = _plan("demo")
    touch_at = [w.label for w in steps].index("straight in until the strip touches")
    tip_in = np.array([TIP_CLEAR + 0.0005, 0.0, 0.0])  # the fingertips aimed 0.5 mm into the wall
    press = PRESS_N * np.array([-1.0, 0.0, 0.0])
    touch = tcp_for(MARK + np.array([0.0005, 0.0, 0.0]))
    return steps[:touch_at + 1] + [
        _w(1.0, touch, PINCH, "tack it on with a light push", WEIGHT + LIGHT_N * np.array([-1.0, 0.0, 0.0])),
        _w(0.6, touch, FINGER_OPEN, "let go"),
        _w(1.2, touch - 0.06 * GRIP[:, 2], FINGER_OPEN, "back the fingers away"),
        _w(0.8, touch - 0.06 * GRIP[:, 2], FINGER_CLOSED, "close the fingers"),
        _w(1.5, tcp_for(BESIDE + tip_in) - 0.03 * GRIP[:, 2], FINGER_CLOSED, "to 3 cm in front of the wall beside it"),
        _w(1.5, tcp_for(BESIDE + tip_in), FINGER_CLOSED, "fingertips onto the bare wall"),
        _w(2.0, tcp_for(BESIDE + tip_in), FINGER_CLOSED, "press the wall firmly", press),
        _w(PRESS_S + 4.0, tcp_for(BESIDE + tip_in), FINGER_CLOSED, "keep pressing the wall", press),
        _w(1.0, tcp_for(BESIDE + tip_in), FINGER_CLOSED, "ease off"),
        _w(1.2, tcp_for(BESIDE + tip_in) - 0.06 * GRIP[:, 2], FINGER_CLOSED, "back away"),
        Waypoint(1.5, PARK_LEFT, FINGER_OPEN, "clear", 0.0),
        Waypoint(6.0, PARK_LEFT, FINGER_OPEN, "stay", 0.0),
    ]


def _idle():
    return [Waypoint(2.0, (0.20, -0.30, TOP + 0.15), FINGER_OPEN, "clear", 0.0),
            Waypoint(60.0, (0.20, -0.30, TOP + 0.15), FINGER_OPEN, "stay", 0.0)]


def demo():
    mode = os.environ.get("HOOK_DEMO", "demo")
    return {"left": _plan(mode), "right": _idle()}
