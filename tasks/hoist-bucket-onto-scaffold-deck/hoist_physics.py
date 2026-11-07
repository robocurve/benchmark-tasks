"""Spawn-time-only USD authoring; no simulation state writes during episodes."""
import math
from types import SimpleNamespace
import numpy as np
import yam_parts as parts
import yam_joints as joints
from yam_arena.ik import matrix_to_quat_xyzw
from hoist_geometry import (HoistRope, PITCH, RADIUS, LINK_MASS, ARMATURE,
                            capsule_inertia, compound_inertia)

SPECS = {}
_CFG = _SPAWN = None


def rope_object(s):
    global _CFG, _SPAWN
    import isaaclab.sim as sim
    from isaaclab.sim.utils import clone
    from isaaclab.utils import configclass
    from isaaclab.managers import EventTermCfg, SceneEntityCfg
    from isaaclab_arena.assets.object import Object
    from isaaclab_arena.assets.object_type import ObjectType
    from isaaclab_arena.terms.events import reset_articulation_pose_and_joints
    from isaaclab_arena.utils.pose import Pose
    if _CFG is None:
        # Must live in an imported helper, never in the unregistered task module.
        @configclass
        class HoistSpawnCfg(sim.SpawnerCfg):
            rope_name: str = ''
        _CFG, _SPAWN = HoistSpawnCfg, clone(spawn_rope)

    class BucketRopeObject(Object):
        def _build_reset_event(self):
            return EventTermCfg(func=reset_articulation_pose_and_joints, mode='reset',
                params={'asset_cfg': SceneEntityCfg(self.name),
                        'pose': self.get_initial_pose(), 'velocity': None})
    SPECS[s.name] = s
    # Arena builds an articulation with actuators={}; only USD passive drives.
    return BucketRopeObject(name=s.name, object_type=ObjectType.ARTICULATION,
        spawner_cfg=_CFG(func=_SPAWN, rope_name=s.name),
        initial_pose=Pose(position_xyz=s.pos, rotation_xyzw=(0, 0, 0, 1)))


def spawn_rope(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics
    from yam_arena.station import linear_rgb
    s = SPECS[cfg.rope_name]
    stage = kwargs.get('stage') or sim.get_current_stage()
    root = UsdGeom.Xform.Define(stage, prim_path)
    translation = kwargs.get('translation', args[0] if args else None)
    orientation = kwargs.get('orientation', args[1] if len(args) > 1 else None)
    if translation is not None:
        root.AddTranslateOp().Set(Gf.Vec3d(*map(float, translation)))
    if orientation is not None:
        x, y, z, w = map(float, orientation)
        root.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
    v, centres, rotations = s.layout()
    bp = f'{prim_path}/bucket'
    parts._SPECS[bp] = s.bucket_part()
    try:
        bucket = parts._spawn_part(bp, SimpleNamespace(part_name=bp),
                                  translation=(0, 0, 0), stage=stage)
    finally:
        del parts._SPECS[bp]
    com, inertia = compound_inertia(s.bucket_part())
    mass = UsdPhysics.MassAPI.Apply(bucket)
    mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(*map(float, com)))
    mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float, np.diag(inertia))))
    assert np.max(np.abs(inertia-np.diag(np.diag(inertia)))) < 1.e-12
    # The heavy bucket, at the container origin, is the floating articulation root.
    UsdPhysics.ArticulationRootAPI.Apply(bucket)
    art = PhysxSchema.PhysxArticulationAPI.Apply(bucket)
    art.CreateEnabledSelfCollisionsAttr().Set(True)
    art.CreateSolverPositionIterationCountAttr().Set(32)
    art.CreateSolverVelocityIterationCountAttr().Set(4)
    matpath, vispath = f'{prim_path}/rope_material', f'{prim_path}/rope_look'
    mat = sim.RigidBodyMaterialCfg(static_friction=.8, dynamic_friction=.8, restitution=0.)
    mat.func(matpath, mat)
    look = sim.PreviewSurfaceCfg(diffuse_color=linear_rgb((.20, .16, .09)), roughness=.8)
    look.func(vispath, look)
    for i, (c, R) in enumerate(zip(centres, rotations)):
        path = f'{prim_path}/seg_{i:02d}'
        body = UsdGeom.Xform.Define(stage, path)
        body.AddTranslateOp().Set(Gf.Vec3d(*map(float, c)))
        x, y, z, w = map(float, matrix_to_quat_xyzw(R))
        body.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w, Gf.Vec3d(x, y, z)))
        prim = body.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr().Set(LINK_MASS)
        mass.CreateCenterOfMassAttr().Set(Gf.Vec3f(0))
        mass.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(*map(float, capsule_inertia())))
        geom = UsdGeom.Capsule.Define(stage, path+'/capsule')
        geom.CreateAxisAttr().Set('X')
        geom.CreateRadiusAttr().Set(RADIUS)
        geom.CreateHeightAttr().Set(PITCH-2*RADIUS)
        geom.CreateVisibilityAttr().Set('invisible')
        UsdPhysics.CollisionAPI.Apply(geom.GetPrim())
        collision = PhysxSchema.PhysxCollisionAPI.Apply(geom.GetPrim())
        collision.CreateContactOffsetAttr().Set(.002)
        collision.CreateRestOffsetAttr().Set(0.)
        sim.bind_physics_material(path+'/capsule', matpath, stage=stage)
        visual = UsdGeom.Capsule.Define(stage, path+'/visual')
        visual.CreateAxisAttr().Set('X')
        visual.CreateRadiusAttr().Set(RADIUS)
        visual.CreateHeightAttr().Set(PITCH+1.8*RADIUS-2*RADIUS)
        sim.bind_visual_material(path+'/visual', vispath, stage=stage)
    # Per-body recipe only; no changes to the harness physics configuration.
    for name in s.body_names():
        rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f'{prim_path}/{name}'))
        rigid.CreateLinearDampingAttr().Set(.5)
        rigid.CreateAngularDampingAttr().Set(2.)
        rigid.CreateMaxDepenetrationVelocityAttr().Set(1.)
        rigid.CreateSolverPositionIterationCountAttr().Set(32)
        rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    UsdGeom.Xform.Define(stage, f'{prim_path}/joints')
    for i in range(s.segments):
        parent = 'bucket' if i == 0 else f'seg_{i-1:02d}'
        child = f'seg_{i:02d}'
        name = 'bail_swivel' if i == 0 else f'bend_{i:02d}'
        joint = UsdPhysics.Joint.Define(stage, f'{prim_path}/joints/{name}')
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateArmatureAttr().Set(ARMATURE)
        Rframe = rotations[max(0, i-1)]
        for j, body in enumerate((parent, child)):
            c, R = (np.zeros(3), np.eye(3)) if body == 'bucket' else (
                centres[int(body[-2:])], rotations[int(body[-2:])])
            getattr(joint, f'CreateBody{j}Rel')().SetTargets([f'{prim_path}/{body}'])
            getattr(joint, f'CreateLocalPos{j}Attr')().Set(Gf.Vec3f(*map(float, R.T@(v[i]-c))))
            x, y, z, w = map(float, matrix_to_quat_xyzw(R.T@Rframe))
            getattr(joint, f'CreateLocalRot{j}Attr')().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
        joint.CreateCollisionEnabledAttr().Set(False)
        for axis in ('transX', 'transY', 'transZ', 'rotX', 'rotY', 'rotZ'):
            limit = UsdPhysics.LimitAPI.Apply(joint.GetPrim(), axis)
            if i == 0 and axis == 'rotX':
                lo, hi = -float('inf'), float('inf')  # free bail twist
            elif axis in ('rotY', 'rotZ'):
                lo, hi = (-120., 120.) if i == 0 else (-60., 60.)
            else:
                lo, hi = 1., -1.  # locked translation and rope twist
            limit.CreateLowAttr().Set(lo)
            limit.CreateHighAttr().Set(hi)
        for axis in ('rotY', 'rotZ'):
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), axis)
            drive.CreateTypeAttr().Set('force')
            drive.CreateStiffnessAttr().Set(.020*math.pi/180)
            drive.CreateDampingAttr().Set(.002*math.pi/180)
            drive.CreateTargetPositionAttr().Set(0.)
            drive.CreateTargetVelocityAttr().Set(0.)
            drive.CreateMaxForceAttr().Set(1.e6)
    return root.GetPrim()


# The joints-kit spawner remains byte-identical on disk. This spawn-time
# wrapper applies the proven per-body iteration counts only to these wheels.
_kit_spawn = joints._spawn_mechanism


def spawn_wheel(prim_path, cfg, *args, **kwargs):
    import isaaclab.sim as sim
    from pxr import PhysxSchema
    root = _kit_spawn(prim_path, cfg, *args, **kwargs)
    if cfg.mech_name == 'hoist_a':
        stage = kwargs.get('stage') or sim.get_current_stage()
        for name in ('base', 'sheave'):
            rigid = PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(f'{prim_path}/{name}'))
            rigid.CreateSolverPositionIterationCountAttr().Set(32)
            rigid.CreateSolverVelocityIterationCountAttr().Set(4)
    return root


joints._spawn_mechanism = spawn_wheel
