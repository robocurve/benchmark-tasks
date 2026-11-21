"""hingekit: spring-loaded hinges for partkit parts. Written from scratch for this kit (MIT); it needs partkit.py next
to it and does not modify it.

    from hingekit import Hinge, hinged
    BOX = hinged("box", pos=(0.32, 0.14, TOP), mass=25.0, boxes=[...the fixed body...], hinges=[
        Hinge("flap", pivot=(0.04, 0.0, 0.205), axis="Y", lower=0.0, upper=100.0, rate=0.05, mass=0.10,
              boxes=[((-0.01, 0.0, 0.205), (0.10, 0.12, 0.01), WHITE)])])

A Hinge is a rigid leaf (boxes given in the part frame, in the closed position) on a revolute joint through pivot
along the part's x, y or z axis. It turns between lower and upper (degrees; 0 is the position it was drawn in) and a
torsion spring of rate N m/rad pulls it back to 0. The part becomes an articulation, as with partkit sliders, so the
leaf does not collide with its own part: the limits are its stops. In score(), objects[name].sliders[hinge_name] is
the hinge angle in degrees.
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass

import partkit
from partkit import Part

__all__ = ["Hinge", "HingedPart", "hinged"]


@dataclass(frozen=True)
class Hinge:
    name: str
    pivot: tuple
    boxes: tuple
    axis: str = "Y"
    lower: float = 0.0        # degrees
    upper: float = 90.0       # degrees
    rate: float = 0.05        # torsion spring toward 0, N m/rad
    damping: float = 0.005    # N m s/rad
    mass: float = 0.10
    friction: float | None = None


@dataclass(frozen=True)
class HingedPart(Part):
    hinges: tuple = ()


def hinged(name, pos, boxes, mass, hinges, **kw) -> HingedPart:
    """Build a part with hinged leaves; the other arguments are those of partkit.part()."""
    hinges = tuple(dataclasses.replace(h, boxes=tuple(h.boxes), pivot=tuple(h.pivot)) for h in hinges)
    every = list(boxes) + [b for h in hinges for b in h.boxes]
    p = partkit.part(name, pos, every, mass, **kw)          # for the bounding size
    vals = {f.name: getattr(p, f.name) for f in dataclasses.fields(p)}
    vals["boxes"] = tuple(boxes)
    return HingedPart(**vals, hinges=hinges)


_orig_spawn, _orig_object, _orig_state = partkit._spawn_part, partkit._arena_object, partkit._part_state


def _spawn_part(prim_path: str, cfg, translation=None, orientation=None, **kwargs):
    prim = _orig_spawn(prim_path, cfg, translation=translation, orientation=orientation, **kwargs)
    spec = partkit._SPECS[cfg.part_name]
    hinges = getattr(spec, "hinges", ())
    if not hinges:
        return prim
    import isaaclab.sim as sim_utils
    from isaaclab.sim.utils import bind_physics_material, bind_visual_material
    from isaaclab.sim.utils.stage import get_current_stage
    from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics

    from yam_arena.station import linear_rgb

    stage = get_current_stage()
    main = f"{prim_path}/body"
    for h in hinges:
        path = f"{prim_path}/{h.name}"
        xform = UsdGeom.Xform.Define(stage, path)
        xform.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*h.pivot))
        xform.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(1, 0, 0, 0))
        leaf = xform.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(leaf)
        UsdPhysics.MassAPI.Apply(leaf).CreateMassAttr(h.mass)
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(leaf)
        rb.CreateMaxDepenetrationVelocityAttr(1.0)
        rb.CreateSolverPositionIterationCountAttr(32)
        material = f"{prim_path}/Looks/physics"
        if h.friction is not None:
            material = f"{prim_path}/Looks/physics_{h.name}"
            pm = sim_utils.RigidBodyMaterialCfg(static_friction=h.friction, dynamic_friction=h.friction)
            pm.func(material, pm)
        for i, (centre, size, color, rot) in enumerate(map(partkit._box, h.boxes)):
            cube = UsdGeom.Cube.Define(stage, f"{path}/box_{i}")
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*[float(c - p) for c, p in zip(centre, h.pivot)]))
            if rot != (1.0, 0.0, 0.0, 0.0):
                cube.AddOrientOp().Set(Gf.Quatf(*rot))
            cube.AddScaleOp().Set(Gf.Vec3f(*size))
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            cp = PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            cp.CreateContactOffsetAttr(0.0005)
            cp.CreateRestOffsetAttr(0.0)
            look = f"{prim_path}/Looks/{h.name}_{i}"
            vis = sim_utils.PreviewSurfaceCfg(diffuse_color=linear_rgb(color), roughness=0.5)
            vis.func(look, vis)
            bind_physics_material(str(cube.GetPrim().GetPath()), material, stage=stage)
            bind_visual_material(str(cube.GetPrim().GetPath()), look, stage=stage)
        j = UsdPhysics.RevoluteJoint.Define(stage, f"{path}/{h.name}_hinge")
        j.CreateAxisAttr(h.axis.upper())
        j.CreateBody0Rel().SetTargets([Sdf.Path(main)])
        j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
        j.CreateLocalPos0Attr(Gf.Vec3f(*h.pivot))
        j.CreateLocalRot0Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        j.CreateLocalRot1Attr(Gf.Quatf(1, 0, 0, 0))
        j.CreateLowerLimitAttr(float(h.lower))               # USD revolute limits are in degrees
        j.CreateUpperLimitAttr(float(h.upper))
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(h.rate * math.pi / 180.0)  # USD angular drives are per degree
        drive.CreateDampingAttr(h.damping * math.pi / 180.0)
        drive.CreateTargetPositionAttr(0.0)
    if not spec.sliders:                                     # partkit only makes an articulation for sliders
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(main))
        art = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(main))
        art.CreateEnabledSelfCollisionsAttr(False)
        art.CreateSolverPositionIterationCountAttr(32)
        art.CreateSolverVelocityIterationCountAttr(4)
    return prim


def _arena_object(spec):
    if not getattr(spec, "hinges", ()):
        return _orig_object(spec)
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg
    from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.utils.pose import Pose

    @configclass
    class HingedSpawnerCfg(SpawnerCfg):
        func: object = clone(_spawn_part)
        part_name: str = ""

    class HingedObject(Object):
        def _generate_articulation_cfg(self):
            actuators = {s.name: ImplicitActuatorCfg(joint_names_expr=[f"{s.name}_slide"], stiffness=s.rate,
                                                     damping=s.damping) for s in spec.sliders}
            actuators.update({h.name: ImplicitActuatorCfg(joint_names_expr=[f"{h.name}_hinge"], stiffness=h.rate,
                                                          damping=h.damping) for h in spec.hinges})
            cfg = ArticulationCfg(prim_path=self.prim_path, spawn=self.spawner_cfg, actuators=actuators)
            return self._add_initial_pose_to_cfg(cfg)

    return HingedObject(
        name=spec.name,
        object_type=ObjectType.ARTICULATION,
        spawner_cfg=HingedSpawnerCfg(part_name=spec.name),
        initial_pose=Pose(position_xyz=tuple(spec.pos), rotation_xyzw=(0.0, 0.0, math.sin(spec.yaw / 2),
                                                                       math.cos(spec.yaw / 2))),
    )


def _part_state(environment, spec):
    st = _orig_state(environment, spec)
    hinges = getattr(spec, "hinges", ())
    if not hinges:
        return st
    asset = environment.unwrapped.scene[spec.name]
    names = list(asset.joint_names)
    q = partkit._np(asset.data.joint_pos)[0]
    angles = dict(st.sliders)
    angles.update({h.name: math.degrees(float(q[names.index(f"{h.name}_hinge")])) for h in hinges})
    return dataclasses.replace(st, sliders=angles)


partkit._spawn_part, partkit._arena_object, partkit._part_state = _spawn_part, _arena_object, _part_state
