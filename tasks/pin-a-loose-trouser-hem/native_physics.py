"""Native contact bodies and passive material joints; never attaches a hand.

Scene construction is declarative in geometry.py. Every manipulated object has
collision geometry, mass and force-limited joints. Only reset writes body poses.
"""
from dataclasses import dataclass
import numpy as np
from yam_arena.task_file import Box
from geometry import BODIES, JOINTS

@dataclass(frozen=True)
class Materials(Box): pass

def spawn(prim_path,cfg,translation=None,orientation=None,**kwargs):
 from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema
 import isaaclab.sim as sim
 from isaaclab.sim.utils import bind_physics_material,bind_visual_material
 from isaaclab.sim.utils.stage import get_current_stage
 stage=get_current_stage();path=str(prim_path);root=path.rsplit('/',1)[0]
 index=cfg.body_index;spec=BODIES[index]
 UsdGeom.Xform.Define(stage,root)
 body=UsdGeom.Xform.Define(stage,path)
 body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))
 x,y,z,w=orientation;body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w,x,y,z))
 prim=body.GetPrim();rigid=UsdPhysics.RigidBodyAPI.Apply(prim)
 if spec.get('fixed'):rigid.CreateKinematicEnabledAttr(True)
 mass=UsdPhysics.MassAPI.Apply(prim);mass.CreateMassAttr(spec['mass'])
 if spec.get('inertia'):
  mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*spec['inertia']));mass.CreatePrincipalAxesAttr(Gf.Quatf(1.,Gf.Vec3f(0.,0.,0.)))
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim);rb.CreateEnableCCDAttr(not spec.get('fixed',False))
 if spec.get('external_force'):
  force=PhysxSchema.PhysxForceAPI.Apply(prim);force.CreateForceEnabledAttr(True);force.CreateWorldFrameEnabledAttr(True);force.CreateModeAttr('force');force.CreateForceAttr(Gf.Vec3f(0,0,0));force.CreateTorqueAttr(Gf.Vec3f(0,0,0))
 rb.CreateSolverPositionIterationCountAttr(64);rb.CreateSolverVelocityIterationCountAttr(16)
 rb.CreateLinearDampingAttr(spec.get('linear_damping',.25));rb.CreateAngularDampingAttr(spec.get('angular_damping',.5))
 rb.CreateMaxLinearVelocityAttr(spec.get('max_linear_velocity',1.));rb.CreateMaxAngularVelocityAttr(spec.get('max_angular_velocity',30.));rb.CreateMaxDepenetrationVelocityAttr(.2);rb.CreateSleepThresholdAttr(0.)
 # Material reaction comes from the PhysX solver; no kinematic grasp constraint.
 mat=path+'/physics'
 c=sim.RigidBodyMaterialCfg(static_friction=spec.get('friction',.7),dynamic_friction=spec.get('friction',.7)*.8,restitution=0.);c.func(mat,c)
 for k,shape in enumerate(spec['shapes']):
  label=path+'/shape_'+str(k);kind=shape['kind']
  if kind=='box':g=UsdGeom.Cube.Define(stage,label);g.CreateSizeAttr(1.);scale=shape['size']
  elif kind=='sphere':g=UsdGeom.Sphere.Define(stage,label);g.CreateRadiusAttr(1.);scale=shape['radii']
  elif kind in ('cylinder','capsule'):
   cls=UsdGeom.Cylinder if kind=='cylinder' else UsdGeom.Capsule
   g=cls.Define(stage,label);g.CreateRadiusAttr(shape['radius']);g.CreateHeightAttr(shape['length']);g.CreateAxisAttr('Z');scale=(1,1,1)
  elif kind=='mesh':
   from pxr import Vt
   g=UsdGeom.Mesh.Define(stage,label);g.CreatePointsAttr(Vt.Vec3fArray(np.asarray(shape['points'],dtype=np.float32).tolist()));g.CreateFaceVertexCountsAttr([len(f) for f in shape['faces']]);g.CreateFaceVertexIndicesAttr([i for f in shape['faces'] for i in f]);scale=(1,1,1)
  else:raise ValueError(kind)
  g.AddTranslateOp().Set(Gf.Vec3d(*shape.get('pos',(0,0,0))))
  q=shape.get('quat',(0,0,0,1));g.AddOrientOp().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
  g.AddScaleOp().Set(Gf.Vec3f(*scale))
  if shape.get('collision',True):
   UsdPhysics.CollisionAPI.Apply(g.GetPrim())
   if kind=='mesh':UsdPhysics.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr('convexHull')
   collision=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim());collision.CreateContactOffsetAttr(spec.get('contact_offset',.00025));collision.CreateRestOffsetAttr(0.)
   if spec.get('torsional_patch_radius'):
    collision.CreateTorsionalPatchRadiusAttr(spec['torsional_patch_radius']);collision.CreateMinTorsionalPatchRadiusAttr(spec.get('min_torsional_patch_radius',0.))
   bind_physics_material(label,mat,stage=stage)
  visual=label+'/material';v=sim.PreviewSurfaceCfg(diffuse_color=tuple(map(float,shape.get('color',spec.get('color',(.5,.5,.5))))),roughness=.6);v.func(visual,v);bind_visual_material(label,visual,stage=stage)
 adjacent=[Sdf.Path(root+'/'+j['b']) if j['a']==spec['name'] else Sdf.Path(root+'/'+j['a']) for j in JOINTS if spec['name'] in (j['a'],j['b']) and j.get('enabled',True) and not j.get('collision',False)]
 adjacent.extend(Sdf.Path(root+'/'+name) for name in spec.get('exclude',[]))
 if adjacent:UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(adjacent)
 if index==len(BODIES)-1:
  for specj in JOINTS:
   j=UsdPhysics.Joint.Define(stage,root+'/joint_'+specj['name'])
   j.CreateBody0Rel().SetTargets([Sdf.Path(root+'/'+specj['a'])]);j.CreateBody1Rel().SetTargets([Sdf.Path(root+'/'+specj['b'])])
   for n in (0,1):
    getattr(j,'CreateLocalPos%dAttr'%n)(Gf.Vec3f(*specj.get('p%d'%n,(0,0,0))))
    q=specj.get('q%d'%n,(0,0,0,1));getattr(j,'CreateLocalRot%dAttr'%n)().Set(Gf.Quatf(q[3],Gf.Vec3f(*q[:3])))
   j.CreateCollisionEnabledAttr(specj.get('collision',False));j.CreateJointEnabledAttr(specj.get('enabled',True))
   if 'break_force' in specj:j.CreateBreakForceAttr(specj['break_force'])
   if 'break_torque' in specj:j.CreateBreakTorqueAttr(specj['break_torque'])
   for axis in ('transX','transY','transZ','rotX','rotY','rotZ'):
    cfgj=specj.get('axes',{}).get(axis,{})
    lo,hi=cfgj.get('limits',(1,-1) if axis.startswith('trans') else (-180,180))
    lim=UsdPhysics.LimitAPI.Apply(j.GetPrim(),axis);lim.CreateLowAttr(lo);lim.CreateHighAttr(hi)
    if 'stiffness' in cfgj:
     # Geometry coefficients are SI torque/radian (and torque*s/radian).
     # USD angular drives instead use torque/degree; limits/targets stay degrees.
     units=np.pi/180 if axis.startswith('rot') else 1.
     d=UsdPhysics.DriveAPI.Apply(j.GetPrim(),axis);d.CreateTypeAttr('force');d.CreateStiffnessAttr(cfgj['stiffness']*units);d.CreateDampingAttr(cfgj.get('damping',.001)*units);d.CreateTargetPositionAttr(cfgj.get('target',0.));d.CreateMaxForceAttr(cfgj.get('max_force',1e6))
  for scene in [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]:PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableCCDAttr(True)
 return prim

def reset(env,env_ids=None):
 import torch
 base=env.unwrapped if hasattr(env,'unwrapped') else env;asset=base.scene['materials'];origin=base.scene.env_origins
 ids=torch.arange(len(origin),device=origin.device,dtype=torch.long) if env_ids is None else env_ids
 poses=torch.as_tensor([b['pos']+b.get('quat',[0,0,0,1]) for b in BODIES],device=origin.device,dtype=origin.dtype)[None].repeat(len(ids),1,1)
 poses[:,:,:3]+=origin[ids,None,:];asset.write_body_link_pose_to_sim_index(body_poses=poses,env_ids=ids)
 asset.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),len(BODIES),6),device=origin.device),env_ids=ids)

def asset(spec):
 from isaaclab.assets import RigidObjectCfg,RigidObjectCollectionCfg
 from isaaclab.managers import EventTermCfg
 from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
 from isaaclab.sim.utils import clone
 from isaaclab.utils import configclass
 from isaaclab_arena.assets.asset import Asset
 @configclass
 class Spawn(SpawnerCfg):
  func:object=clone(spawn)
  body_index:int=0
 class A(Asset):
  def __init__(self):
   super().__init__('materials',tags=['physx','contact'])
   self.object_cfg=RigidObjectCollectionCfg(rigid_objects={b['name']:RigidObjectCfg(prim_path='{ENV_REGEX_NS}/materials/'+b['name'],spawn=Spawn(body_index=i),init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(b['pos']),rot=tuple(b.get('quat',[0,0,0,1])))) for i,b in enumerate(BODIES)})
  def get_object_cfg(self):return self.name,self.object_cfg
  def get_event_cfg(self):return self.name,EventTermCfg(func=reset,mode='reset')
 return A()

def install():
 from yam_arena import scene
 old=scene.task_object;old_states=scene.object_states
 scene.task_object=lambda spec:asset(spec) if isinstance(spec,Materials) else old(spec)
 scene.object_states=lambda env,objects:old_states(env,[o for o in objects if o!='materials'])
install()
