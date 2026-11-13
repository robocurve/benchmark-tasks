"""Passive articulated props on the official bimanual YAM station."""
from dataclasses import dataclass
from pathlib import Path
import atexit,json,math,os
import numpy as np
import snack_base as common
from snack_base import Compound

@dataclass(frozen=True)
class Articulated(Compound):
    links: tuple = ()
    joints: tuple = ()
    fixed_base: bool = False
    self_collisions: bool = False
    initial_joints: tuple = ()
    contact_pairs: tuple = ()

    def _check(self):
        super()._check()
        names={'root'}|{a['name'] for a in self.links}
        if len(names)!=len(self.links)+1: raise ValueError('Repeated link name')
        for j in self.joints:
            if j['body0'] not in names or j['body1'] not in names: raise ValueError('Unknown joint body')
            if j['type'] not in ['prismatic','revolute']: raise ValueError('Unsupported joint')

def _write_usd(spec,directory):
    from pxr import Gf,Usd,UsdGeom,UsdPhysics,PhysxSchema,UsdShade
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/(spec.name+'.usda')
    stage=Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage,1.)
    root=UsdGeom.Xform.Define(stage,'/prop').GetPrim()
    stage.SetDefaultPrim(root)
    UsdPhysics.ArticulationRootAPI.Apply(root)
    solver=PhysxSchema.PhysxArticulationAPI.Apply(root)
    solver.CreateEnabledSelfCollisionsAttr(spec.self_collisions)
    solver.CreateSolverPositionIterationCountAttr(64)
    solver.CreateSolverVelocityIterationCountAttr(8)
    def body(name,parts,mass,pos,quat=(0.,0.,0.,1.)):
        prim=UsdGeom.Xform.Define(stage,'/prop/'+name)
        prim.AddTranslateOp().Set(Gf.Vec3d(*map(float,pos)))
        prim.AddOrientOp().Set(Gf.Quatf(float(quat[3]),Gf.Vec3f(*map(float,quat[:3]))))
        UsdPhysics.RigidBodyAPI.Apply(prim.GetPrim()).CreateRigidBodyEnabledAttr(True)
        UsdPhysics.MassAPI.Apply(prim.GetPrim()).CreateMassAttr(float(mass))
        PhysxSchema.PhysxContactReportAPI.Apply(prim.GetPrim()).CreateThresholdAttr(0.)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim.GetPrim()).CreateEnableCCDAttr(True)
        for i,p in enumerate(parts):
            if p.get('shape')=='mesh':
                cube=UsdGeom.Mesh.Define(stage,f'/prop/{name}/p{i}')
                cube.CreatePointsAttr([Gf.Vec3f(*vertex) for vertex in p['vertices']])
                cube.CreateFaceVertexCountsAttr([len(face) for face in p['faces']])
                cube.CreateFaceVertexIndicesAttr([index for face in p['faces'] for index in face])
                cube.CreateSubdivisionSchemeAttr('none')
                cube.CreateDoubleSidedAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(cube.GetPrim()).CreateApproximationAttr('convexHull')
            elif p.get('shape')=='sphere':
                cube=UsdGeom.Sphere.Define(stage,f'/prop/{name}/p{i}')
                cube.CreateRadiusAttr(float(p['radius']))
            elif p.get('shape')=='cylinder':
                cube=UsdGeom.Cylinder.Define(stage,f'/prop/{name}/p{i}')
                cube.CreateRadiusAttr(float(p['radius']))
                cube.CreateHeightAttr(float(p['height']))
                cube.CreateAxisAttr(p.get('axis','Z'))
            else:
                cube=UsdGeom.Cube.Define(stage,f'/prop/{name}/p{i}')
                cube.CreateSizeAttr(1.)
            cube.AddTranslateOp().Set(Gf.Vec3d(*map(float,p['pos'])))
            angle=float(p.get('rot_z',0.))
            cube.AddOrientOp().Set(Gf.Quatf(math.cos(angle/2),Gf.Vec3f(0,0,math.sin(angle/2))))
            if p.get('shape') not in ('sphere','cylinder','mesh'):cube.AddScaleOp().Set(Gf.Vec3f(*map(float,p['size'])))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*p.get('color',spec.color))])
            if p.get('visual_only',False): continue
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim()).CreateCollisionEnabledAttr(True)
            contact=PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            contact.CreateContactOffsetAttr(float(p.get("contact_offset", .00005)))
            contact.CreateRestOffsetAttr(0.)
            material=UsdShade.Material.Define(stage,f'/prop/Materials/{name}_{i}')
            mat=UsdPhysics.MaterialAPI.Apply(material.GetPrim())
            friction=float(p.get('friction',.4))
            mat.CreateStaticFrictionAttr(friction)
            mat.CreateDynamicFrictionAttr(.8*friction)
            mat.CreateRestitutionAttr(0.)
            UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(material,UsdShade.Tokens.weakerThanDescendants,'physics')
    body('root',spec.parts,spec.mass or 1.,(0,0,0))
    for link in spec.links:
        body(link['name'],link['parts'],link['mass'],link.get('pos',(0,0,0)),link.get('quat_xyzw',(0,0,0,1)))
    if spec.contact_pairs:
        from itertools import combinations
        active={frozenset(pair) for pair in spec.contact_pairs}
        for left,right in combinations(['root']+[link['name'] for link in spec.links],2):
            if frozenset((left,right)) not in active:
                prim=stage.GetPrimAtPath('/prop/'+left)
                rel=UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel()
                rel.AddTarget('/prop/'+right)
    if spec.fixed_base:
        fixed=UsdPhysics.FixedJoint.Define(stage,'/prop/world_fix')
        fixed.CreateBody1Rel().SetTargets(['/prop/root'])
        fixed.CreateLocalPos0Attr(Gf.Vec3f(*spec.pos))
        fixed.CreateLocalRot0Attr(Gf.Quatf(math.cos(spec.yaw/2),Gf.Vec3f(0,0,math.sin(spec.yaw/2))))
    for j in spec.joints:
        cls=UsdPhysics.PrismaticJoint if j['type']=='prismatic' else UsdPhysics.RevoluteJoint
        joint=cls.Define(stage,'/prop/'+j['name'])
        joint.CreateBody0Rel().SetTargets(['/prop/'+j['body0']])
        joint.CreateBody1Rel().SetTargets(['/prop/'+j['body1']])
        joint.CreateAxisAttr(j.get('axis','Y'))
        joint.CreateLocalPos0Attr(Gf.Vec3f(*j.get('anchor0',(0,0,0))))
        joint.CreateLocalPos1Attr(Gf.Vec3f(*j.get('anchor1',(0,0,0))))
        if not j.get('continuous',False):
            joint.CreateLowerLimitAttr(j['lower'])
            joint.CreateUpperLimitAttr(j['upper'])
        drive=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'linear' if j['type']=='prismatic' else 'angular')
        drive.CreateTypeAttr('force')
        drive.CreateTargetPositionAttr(j.get('target',0.))
        drive.CreateStiffnessAttr(j.get('stiffness',100.))
        drive.CreateDampingAttr(j.get('damping',1.))
        drive.CreateMaxForceAttr(j.get('max_force',3.))
    stage.GetRootLayer().Save()
    return path

def _install():
    from yam_arena import scene
    common._install_spawn()
    if getattr(scene.task_object,'_articulated',False): return
    original=scene.task_object
    original_states=scene.object_states
    specs={}
    def spawn(spec):
        if not isinstance(spec,Articulated): return original(spec)
        specs[spec.name]=spec
        from isaaclab_arena.assets.object import Object
        from isaaclab_arena.utils.pose import Pose
        path=_write_usd(spec,Path(os.environ.get('YAM_GEOMETRY_DIR','generated_geometry')))
        q=(0.,0.,math.sin(spec.yaw/2),math.cos(spec.yaw/2))
        obj=Object(name=spec.name,usd_path=str(path.resolve()),object_type=scene.ObjectType.ARTICULATION,
                      initial_pose=Pose(position_xyz=tuple(float(v) for v in spec.pos),rotation_xyzw=q))
        if spec.initial_joints:
            obj.object_cfg.init_state.joint_pos=dict(spec.initial_joints)
        return obj
    spawn._industrial=True
    spawn._articulated=True
    scene.task_object=spawn
    def object_states(env,names):
        # Arena's geometry helper does not handle articulations. Build exact bounds
        # from the simulated link poses and the authored primitive corners.
        from yam_arena.task_file import ObjectState
        from yam_arena.urdf import quat_xyzw_to_matrix
        out=original_states(env,[n for n in names if n not in specs])
        base=env.unwrapped
        origin=base.scene.env_origins[0].detach().cpu().numpy()
        signs=np.array([[x,y,z] for x in [-1,1] for y in [-1,1] for z in [-1,1]])
        for name in names:
            if name not in specs: continue
            spec=specs[name];art=base.scene[name]
            poses=art.data.body_link_pose_w.torch[0].detach().cpu().numpy()
            by_name={n:i for i,n in enumerate(art.body_names)}
            pieces={'root':spec.parts,**{link['name']:link['parts'] for link in spec.links}}
            points=[]
            for link,parts in pieces.items():
                pose=poses[by_name[link]]
                rot=quat_xyzw_to_matrix(pose[3:])
                for part in parts:
                    vertices=signs*np.array(part['size'])/2+np.array(part['pos'])
                    points.extend(vertices@rot.T+pose[:3]-origin)
            points=np.asarray(points)
            low=points.min(axis=0);high=points.max(axis=0)
            root=poses[by_name['root']]
            data=art.data
            lin=getattr(data,'root_link_lin_vel_w',None)
            ang=getattr(data,'root_link_ang_vel_w',None)
            if lin is None or ang is None:
                vel=data.body_link_vel_w.torch[0,by_name['root']].detach().cpu().numpy()
                linear,angular=vel[:3],vel[3:]
            else:
                linear=lin.torch[0].detach().cpu().numpy();angular=ang.torch[0].detach().cpu().numpy()
            out[name]=ObjectState((low+high)/2,root[3:].astype(float),linear.astype(float),angular.astype(float),low,high)
        return out
    scene.object_states=object_states

def embodiment(**kwargs):
    from yam_arena import inspect_embodiment as core
    class ArticulatedYam(core.YamArenaEmbodiment):
        def _ensure_sim(self,goal):
            if core._APP is None:
                from isaaclab.app import AppLauncher
                os.environ.setdefault('OMNI_KIT_ACCEPT_EULA','YES')
                opts=dict(headless=self.headless,device=self.device,enable_cameras=True)
                if os.environ.get('YAM_LIMIT_CPU_THREADS'):
                    count=int(os.environ['YAM_LIMIT_CPU_THREADS'])
                    if not 1<=count<=8:raise ValueError('Invalid CPU thread bound')
                    opts['limit_cpu_threads']=count
                    opts['kit_args']=f'--/plugins/carb.tasking.plugin/threadCount={count} --/plugins/omni.tbb.globalcontrol/maxThreadCount={count} --/persistent/physics/numThreads={count}'
                if not self.headless:
                    opts['visualizer']=['kit'];os.environ['DISPLAY']=core.window_display(os.environ)
                core._APP=AppLauncher(**opts).app
                atexit.register(core._close_app_at_exit)
            _install()
            result=super()._ensure_sim(goal)
            if self.task and not getattr(self._env,'_packaging_reset_installed',False):
                reset_environment=self._env.reset
                def restore_fixture(*args,**kwargs):
                    result=reset_environment(*args,**kwargs)
                    # The default YAM scene resets the robots. Restore the passive
                    # fixture coordinates once before its normal settling interval.
                    for spec in self.task.module.OBJECTS:
                        if not getattr(spec,'initial_joints',()):continue
                        art=self._base.scene[spec.name]
                        positions=art.data.joint_pos.torch.clone()
                        for joint,value in spec.initial_joints:
                            positions[:,art.joint_names.index(joint)]=value
                        art.write_joint_state_to_sim(positions,self._torch.zeros_like(positions))
                    return result
                self._env.reset=restore_fixture
                self._env._packaging_reset_installed=True
            return result

        def reset(self,scene,*,seed=None):
            self._telemetry=None
            obs=super().reset(scene,seed=seed)
            if self.task and hasattr(self.task.module,'reset_history'): self.task.module.reset_history()
            if os.environ.get('YAM_TELEMETRY'):
                self._telemetry=Path(os.environ['YAM_TELEMETRY'])
                self._telemetry.parent.mkdir(parents=True,exist_ok=True);self._telemetry.write_text('')
            return obs

        def _task_step(self):
            from yam_arena.scene import object_states
            states=object_states(self._env,self.task.names)
            joints={}
            for name,art in self._base.scene.articulations.items():
                joints[name]=dict(names=list(art.joint_names),q=art.data.joint_pos.torch[0].detach().cpu().numpy().tolist())
            self.task.module._expert_states=states
            self.task.module._expert_joints=joints
            if hasattr(self.task.module,'observe'): self.task.module.observe(states,self._t,self._base)
            result=super()._task_step()
            if self._telemetry:
                row=dict(t=float(self._t),joints=joints,objects={n:dict(pos=s.pos.tolist(),quat=s.quat_xyzw.tolist(),lin_vel=s.lin_vel.tolist(),ang_vel=s.ang_vel.tolist(),low=s.low.tolist(),high=s.high.tolist()) for n,s in states.items()},info=result.info)
                if hasattr(self.task.module,'diagnostics'):row['task_diagnostics']=self.task.module.diagnostics()
                with self._telemetry.open('a') as f: f.write(json.dumps(row,default=lambda v:v.item() if isinstance(v,np.generic) else str(v))+'\n')
            return result
    return ArticulatedYam(**kwargs)



