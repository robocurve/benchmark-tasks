"""Narrow steel finger extensions with silicone contact faces.

Each extension is 40 x 24 x 3 mm and leaves the original finger bodies 23 mm
behind the working point, giving access to the thin elastic without the finger bodies pushing it away.
They are rigid robot hardware in both demo and Dream runs, never grasp joints.
"""
import numpy as np
from scipy.spatial.transform import Rotation


def configure_kinematics():
    from yam_arena import constants, ik, station
    station.BASES.update({side:(0.,y,.75) for side,y in [("left",.305),("right",-.305)]})
    constants.TCP_OFFSET=(0.,0.,.170)
    ik.TCP_OFFSET=constants.TCP_OFFSET
    import sys
    if 'yam_arena.runtime' in sys.modules:sys.modules['yam_arena.runtime'].TCP_OFFSET=constants.TCP_OFFSET


def install():
    configure_kinematics()
    from yam_arena import embodiment, robot
    from yam_arena.constants import GRIPPER
    from isaaclab.sim.utils import clone
    GRIPPER.update(stiffness=5000., effort=40.)
    original = embodiment.yam_arm_cfg
    if getattr(original, '_task_pad_covers', False):
        return

    def covered_config(*args, **kwargs):
        config = original(*args, **kwargs)
        config.spawn.articulation_props.solver_position_iteration_count=64
        config.spawn.articulation_props.solver_velocity_iteration_count=16
        original_spawn = config.spawn.func

        @clone
        def spawn(path, cfg, **spawn_kwargs):
            prim = original_spawn(path, cfg, **spawn_kwargs)
            from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
            import isaaclab.sim as sim
            from isaaclab.sim.utils import bind_physics_material, bind_visual_material
            from isaaclab.sim.utils.stage import get_current_stage
            stage = get_current_stage()
            paths = robot.asset_link_paths()
            # The 3 mm contact faces touch at a 6 mm nominal jaw gap. The pinned
            # model lacks a cover-to-cover closing stop, so represent that ordinary
            # physical stop with the two prismatic joint limits.
            stops=[]
            for joint_prim in stage.Traverse():
                if str(joint_prim.GetPath()).startswith(path+'/') and joint_prim.GetName() in ('joint7','joint8') and joint_prim.IsA(UsdPhysics.PrismaticJoint):
                    UsdPhysics.PrismaticJoint(joint_prim).CreateUpperLimitAttr(-.003)
                    stops.append(joint_prim.GetName())
            if sorted(stops)!=['joint7','joint8']:
                raise RuntimeError('Could not apply the two physical pad-cover closing stops.')
            physics_path = path + '/SiliconeCoverPhysics'
            material = sim.RigidBodyMaterialCfg(static_friction=1.2, dynamic_friction=1.0, restitution=0.)
            material.func(physics_path, material)
            visual_path = path + '/SiliconeCoverVisual'
            visual = sim.PreviewSurfaceCfg(diffuse_color=(.15, .15, .15), roughness=.85)
            visual.func(visual_path, visual)
            rotation = Rotation.from_euler('xyz', [-np.pi, 0., np.pi/2])
            q = rotation.inv().as_quat()
            for name, sign in [('tip_left', 1.), ('tip_right', -1.)]:
                # joint7 opens toward negative gripper X; its cover protrudes
                # toward positive X. joint8 is the mirror image.
                # Nominal closed-link origins from the pinned YAM URDF.
                origin = np.array([sign*.0445119, -sign*.0239931, .0566829])
                centre = rotation.inv().apply(np.array([sign*.0015, 0., .152])-origin)
                PhysxSchema.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(path+paths[name])).CreateEnableSpeculativeCCDAttr(True)
                shape = UsdGeom.Cube.Define(stage, path+paths[name]+'/precision_finger_extension')
                shape.CreateSizeAttr(1.)
                shape.AddTranslateOp().Set(Gf.Vec3d(*centre))
                shape.AddOrientOp().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))
                shape.AddScaleOp().Set(Gf.Vec3f(.003, .024, .040))
                UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
                collision = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
                collision.CreateContactOffsetAttr(.002)
                collision.CreateRestOffsetAttr(0.)
                collision.CreateTorsionalPatchRadiusAttr(.006)
                collision.CreateMinTorsionalPatchRadiusAttr(.004)
                bind_physics_material(str(shape.GetPath()), physics_path, stage=stage)
                bind_visual_material(str(shape.GetPath()), visual_path, stage=stage)
            return prim

        config.spawn.make_uninstanceable = True
        config.spawn.func = spawn
        return config

    covered_config._task_pad_covers = True
    embodiment.yam_arm_cfg = covered_config
