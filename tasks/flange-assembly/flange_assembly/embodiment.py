"""Real-CAD flange geometry for the YAM task-file environment."""
from pathlib import Path
from yam_arena.inspect_embodiment import YamArenaEmbodiment

_DEMO_STATES = {}


def demo_states():
    """Measured simulator poses for the scripted oracle; not a Dream input."""
    return _DEMO_STATES


def spawn_pin(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """A single rigid pin with a short tapered lead on its shank."""
    import math
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema
    stage=omni.usd.get_context().get_stage()
    root=UsdGeom.Xform.Define(stage,prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0,0,0))))
    q=orientation or (1,0,0,0)
    root.AddOrientOp().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:]))))
    p=root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(p)
    UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(.035)
    n=24
    shaft_radius = .007
    tip_radius = .004
    points=[Gf.Vec3f(r*math.cos(2*math.pi*i/n),r*math.sin(2*math.pi*i/n),z)
            for z,r in ((-.025,tip_radius),(-.015,shaft_radius),(.025,shaft_radius)) for i in range(n)]
    faces=[tuple(reversed(range(n))),tuple(2*n+i for i in range(n))]
    faces += [(j*n+i,j*n+(i+1)%n,(j+1)*n+(i+1)%n,(j+1)*n+i)
              for j in range(2) for i in range(n)]
    shaft=UsdGeom.Mesh.Define(stage,f'{prim_path}/shaft')
    shaft.CreatePointsAttr(points)
    shaft.CreateFaceVertexCountsAttr([len(face) for face in faces])
    shaft.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    shaft.CreateSubdivisionSchemeAttr('none')
    shaft.CreateDisplayColorAttr([Gf.Vec3f(.65,.48,.12)])
    UsdPhysics.CollisionAPI.Apply(shaft.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(shaft.GetPrim()).CreateApproximationAttr('convexHull')
    shaft_contact = PhysxSchema.PhysxCollisionAPI.Apply(shaft.GetPrim())
    shaft_contact.CreateContactOffsetAttr().Set(.0005)
    shaft_contact.CreateRestOffsetAttr().Set(0.)
    head=UsdGeom.Cylinder.Define(stage,f'{prim_path}/head')
    head.CreateRadiusAttr(.010); head.CreateHeightAttr(.01)
    head.AddTranslateOp().Set(Gf.Vec3d(0,0,.03))
    head.CreateDisplayColorAttr([Gf.Vec3f(.65,.48,.12)])
    UsdPhysics.CollisionAPI.Apply(head.GetPrim())
    head_contact = PhysxSchema.PhysxCollisionAPI.Apply(head.GetPrim())
    head_contact.CreateContactOffsetAttr().Set(.0005)
    head_contact.CreateRestOffsetAttr().Set(0.)
    return p


def spawn_holder(prim_path,cfg,translation=None,orientation=None,**kwargs):
    """Fixed circular rim support, open in the middle for the lower neck."""
    import math
    import omni.usd
    from pxr import Gf,UsdGeom,UsdPhysics,UsdShade
    stage=omni.usd.get_context().get_stage()
    root=UsdGeom.Xform.Define(stage,prim_path)
    root.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0,0,0))))
    q=orientation or (1,0,0,0)
    root.AddOrientOp().Set(Gf.Quatf(float(q[0]),Gf.Vec3f(*map(float,q[1:]))))
    material=UsdShade.Material.Define(stage,f'{prim_path}/support_material')
    props=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    props.CreateStaticFrictionAttr(1.5); props.CreateDynamicFrictionAttr(1.2)
    n=32
    # Each wedge is a convex collider. Top is 60 mm above the table;
    # a 67 mm inner radius leaves the neck and bolt holes free.
    for i in range(n):
        a,b=2*math.pi*i/n,2*math.pi*(i+1)/n
        pts=[]
        for z in (-.03,.03):
            for r,theta in ((.067,a),(.085,a),(.085,b),(.067,b)):
                pts.append(Gf.Vec3f(r*math.cos(theta),r*math.sin(theta),z))
        m=UsdGeom.Mesh.Define(stage,f'{prim_path}/sector_{i}')
        m.CreatePointsAttr(pts)
        faces=[(0,3,2,1),(4,5,6,7),(0,1,5,4),(1,2,6,5),(2,3,7,6),(3,0,4,7)]
        m.CreateFaceVertexCountsAttr([4]*len(faces))
        m.CreateFaceVertexIndicesAttr([v for f in faces for v in f])
        m.CreateSubdivisionSchemeAttr('none')
        m.CreateDisplayColorAttr([Gf.Vec3f(.16,.19,.22)])
        UsdPhysics.CollisionAPI.Apply(m.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
        UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(material,materialPurpose='physics')
    return root.GetPrim()


def spawn_pipe(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Horizontal rigid fixture representing the fixed pipe."""
    import omni.usd
    from pxr import Gf, UsdGeom, UsdPhysics
    stage = omni.usd.get_context().get_stage()
    pipe = UsdGeom.Cylinder.Define(stage, prim_path)
    pipe.CreateAxisAttr('X')
    pipe.CreateRadiusAttr(.027)
    pipe.CreateHeightAttr(.1)
    pipe.AddTranslateOp().Set(Gf.Vec3d(*(translation or (0, 0, 0))))
    pipe.CreateDisplayColorAttr([Gf.Vec3f(.4, .46, .52)])
    UsdPhysics.CollisionAPI.Apply(pipe.GetPrim())
    return pipe.GetPrim()


class FlangeEmbodiment(YamArenaEmbodiment):
    def reset(self, scene, *, seed=None):
        observation = super().reset(scene, seed=seed)
        self._publish_demo_states()
        return observation

    def _publish_demo_states(self):
        global _DEMO_STATES
        from yam_arena.scene import object_states
        _DEMO_STATES = object_states(self._env, ('lower_flange', 'upper_flange',
                                                'pin_1', 'pin_2'))
        return _DEMO_STATES

    def _ensure_sim(self,goal):
        import os,atexit
        from yam_arena import inspect_embodiment as base
        if base._APP is None:
            from isaaclab.app import AppLauncher
            os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
            base._APP=AppLauncher(headless=self.headless,device=self.device,enable_cameras=True).app
            atexit.register(base._close_app_at_exit)
        from yam_arena import scene
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.assets.object_type import ObjectType
        from isaaclab_arena.utils.pose import Pose
        import isaaclab.sim as sim_utils
        path=Path(__file__).resolve().parent.parent/'assets'/'flange.usda'
        original=scene.task_object
        def make_object(spec):
            if spec.name in ('lower_flange','upper_flange'):
                rotation=(0.,2**-.5,0.,2**-.5) if spec.name=='lower_flange' else (0.,-2**-.5,0.,2**-.5)
                return Object(name=spec.name,usd_path=str(path.with_name('flange_fixed.usda') if spec.name=='lower_flange' else path.with_name('flange_with_stub.usda')),object_type=ObjectType.BASE if spec.name=='lower_flange' else ObjectType.RIGID,
                              initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=rotation),
                              spawn_cfg_addon={'physics_material':scene._grippy(),
                                  **({'mass_props':{'':[sim_utils.MassCfg(mass=spec.mass)]}} if spec.mass else {}),
                                  'mass_props_create_if_missing':True,
                                  'collision_props':sim_utils.PhysxCollisionPropertiesCfg(contact_offset=.0005, rest_offset=0.)})
            obj=original(spec)
            if spec.name=='fixed_pipe':
                obj.spawner_cfg.func=spawn_pipe
                obj.object_cfg=obj._init_object_cfg()
            if spec.name.startswith('pin_') and not spec.name.startswith('pin_support_'):
                obj.spawner_cfg.func=spawn_pin
                obj.initial_pose=Pose(position_xyz=tuple(spec.pos),rotation_xyzw=(0.,-2**-.5,0.,2**-.5))
                obj.object_cfg=obj._init_object_cfg()
                obj._pose_event_cfg=obj._build_reset_event()
            return obj
        scene.task_object=make_object
        try: return super()._ensure_sim(goal)
        finally: scene.task_object=original

    def _task_step(self):
        result=super()._task_step()
        states=self._publish_demo_states()
        if int(self._t*20)%20==0:
            print('ASSEMBLY_STATE',round(self._t,2),{k:{'p':[round(float(x),4) for x in v.pos],
                    'q':[round(float(x),3) for x in v.quat_xyzw],
                    'v':[round(float(x),4) for x in v.lin_vel],
                    'w':[round(float(x),3) for x in v.ang_vel]} for k,v in states.items()},flush=True)
        return result


def make_embodiment(**kwargs): return FlangeEmbodiment(**kwargs)
