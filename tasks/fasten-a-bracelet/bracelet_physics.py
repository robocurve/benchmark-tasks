"""Native, freely moving bracelet bodies. No gripper attachment or pose tracking.

Short rigid leather panels have passive joints. The two metal clasp bodies are
hollow/solid collision geometry. Only the clasp's own spring catch is reduced:
a disabled inter-clasp joint engages after the pin physically enters the socket.
"""
from dataclasses import dataclass
from yam_arena.task_file import Box
import numpy as np
from bracelet_geometry import N,STEP,POSITIONS,QUATS,ANCHOR0,ANCHOR1,JOINT_ROT0,JOINT_ROT1
@dataclass(frozen=True)
class Bracelet(Box):pass

def spawn(prim_path,cfg,translation=None,orientation=None,**kwargs):
 from pxr import Gf,Sdf,UsdGeom,UsdPhysics,PhysxSchema
 import isaaclab.sim as sim
 from isaaclab.sim.utils import bind_physics_material,bind_visual_material
 from isaaclab.sim.utils.stage import get_current_stage
 stage=get_current_stage();path=str(prim_path);root=path.rsplit('/',1)[0];i=cfg.link_index
 UsdGeom.Xform.Define(stage,root)
 body=UsdGeom.Xform.Define(stage,path)
 body.AddTranslateOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translation))
 x,y,z,w=orientation;body.AddOrientOp(precision=UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(w,x,y,z))
 prim=body.GetPrim();UsdPhysics.RigidBodyAPI.Apply(prim)
 UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(.006 if i in (0,N-1) else .00010)
 rb=PhysxSchema.PhysxRigidBodyAPI.Apply(prim);rb.CreateEnableCCDAttr(True)
 rb.CreateSolverPositionIterationCountAttr(128);rb.CreateSolverVelocityIterationCountAttr(32)
 rb.CreateLinearDampingAttr(.35);rb.CreateAngularDampingAttr(1.)
 rb.CreateMaxDepenetrationVelocityAttr(.5);rb.CreateSleepThresholdAttr(0.)
 mat=root+'/physics'
 if not stage.GetPrimAtPath(mat).IsValid():
  c=sim.RigidBodyMaterialCfg(static_friction=.8,dynamic_friction=.65,restitution=0.);c.func(mat,c)
 visual=root+('/metal' if i in (0,N-1) else '/leather')
 if not stage.GetPrimAtPath(visual).IsValid():
  c=sim.PreviewSurfaceCfg(diffuse_color=(.65,.68,.72) if i in (0,N-1) else (.035,.07,.15),roughness=.55);c.func(visual,c)
 def box(label,pos,size):
  g=UsdGeom.Cube.Define(stage,path+'/'+label);g.CreateSizeAttr(1.)
  g.AddTranslateOp().Set(Gf.Vec3d(*pos));g.AddScaleOp().Set(Gf.Vec3f(*size))
  UsdPhysics.CollisionAPI.Apply(g.GetPrim());c=PhysxSchema.PhysxCollisionAPI.Apply(g.GetPrim())
  c.CreateContactOffsetAttr(.00015);c.CreateRestOffsetAttr(0.)
  bind_physics_material(str(g.GetPath()),mat,stage=stage);bind_visual_material(str(g.GetPath()),visual,stage=stage)
 if i==0:
  # 4 mm square entrance; 12 x 7 mm outer grip body, 10 mm long. Back stop is solid.
  for sign in (-1,1):
   box('side_'+str(sign).replace('-','n'),(0,sign*.004,0),(.010,.004,.007))
   box('top_'+str(sign).replace('-','n'),(0,0,sign*.00275),(.010,.004,.0015))
  box('back',(-.0045,0,0),(.001,.004,.004))
 elif i==N-1:
  box('body',(0,0,0),(.010,.012,.007))
  box('pin',(-.008,0,0),(.006,.0022,.0022))
  box('stop_collar',(-.0065,0,0),(.003,.005,.005))
 else:box('band',(0,0,0),(STEP,.010,.002))
 adjacent=[Sdf.Path(root+f'/link_{j:03d}') for j in (i-1,i+1) if 0<=j<N]
 UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(adjacent)
 if i:
  joint=UsdPhysics.Joint.Define(stage,root+f'/seam_{i:03d}')
  joint.CreateBody0Rel().SetTargets([Sdf.Path(root+f'/link_{i-1:03d}')]);joint.CreateBody1Rel().SetTargets([Sdf.Path(path)])
  joint.CreateLocalPos0Attr(Gf.Vec3f(*ANCHOR0[i-1]));joint.CreateLocalPos1Attr(Gf.Vec3f(*ANCHOR1[i-1]))
  for attr,q in [(joint.CreateLocalRot0Attr,JOINT_ROT0[i-1]),(joint.CreateLocalRot1Attr,JOINT_ROT1[i-1])]:
   attr().Set(Gf.Quatf(float(q[3]),Gf.Vec3f(*q[:3])))
  joint.CreateCollisionEnabledAttr(False)
  for axis in ('transX','transY','transZ'):
   a=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);a.CreateLowAttr(1.);a.CreateHighAttr(-1.)
  for axis,limit in [('rotX',90),('rotY',170),('rotZ',45)]:
   a=UsdPhysics.LimitAPI.Apply(joint.GetPrim(),axis);a.CreateLowAttr(-limit);a.CreateHighAttr(limit)
   d=UsdPhysics.DriveAPI.Apply(joint.GetPrim(),axis);d.CreateTypeAttr('force')
   d.CreateStiffnessAttr(.000002);d.CreateDampingAttr(.0000008);d.CreateTargetPositionAttr(0.)
 if i==N-1:
  # The spring catch connects clasp to clasp, never clasp to robot.
  j=UsdPhysics.SphericalJoint.Define(stage,root+'/spring_catch')
  j.CreateAxisAttr('X');j.CreateConeAngle0LimitAttr(8.);j.CreateConeAngle1LimitAttr(8.)
  j.CreateBody0Rel().SetTargets([Sdf.Path(root+'/link_000')]);j.CreateBody1Rel().SetTargets([Sdf.Path(path)])
  j.CreateLocalPos0Attr(Gf.Vec3f(.005,0,0));j.CreateLocalPos1Attr(Gf.Vec3f(-.0065,0,0))
  j.CreateCollisionEnabledAttr(False);j.CreateJointEnabledAttr(False)
  j.CreateBreakForceAttr(30.);j.CreateBreakTorqueAttr(.20)
 if i==0:
  for scene in [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]:
   PhysxSchema.PhysxSceneAPI.Apply(scene).CreateEnableCCDAttr(True)
 return prim

def reset(env,env_ids=None):
 import torch
 base=env.unwrapped if hasattr(env,'unwrapped') else env
 asset=base.scene['bracelet'];origin=base.scene.env_origins
 ids=torch.arange(len(origin),device=origin.device,dtype=torch.long) if env_ids is None else env_ids
 poses=torch.as_tensor(np.c_[POSITIONS,QUATS],device=origin.device,dtype=origin.dtype)[None].repeat(len(ids),1,1)
 poses[:,:,:3]+=origin[ids,None,:]
 asset.write_body_link_pose_to_sim_index(body_poses=poses,env_ids=ids)
 asset.write_body_link_velocity_to_sim_index(body_velocities=torch.zeros((len(ids),N,6),device=origin.device),env_ids=ids)

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
  link_index:int=0
 class A(Asset):
  def __init__(self):
   super().__init__('bracelet',tags=['bracelet','physx'])
   self.object_cfg=RigidObjectCollectionCfg(rigid_objects={f'link_{i:03d}':RigidObjectCfg(prim_path='{ENV_REGEX_NS}/bracelet/'+f'link_{i:03d}',spawn=Spawn(link_index=i),init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(p),rot=tuple(QUATS[i]))) for i,p in enumerate(POSITIONS)})
  def get_object_cfg(self):return self.name,self.object_cfg
  def get_event_cfg(self):return self.name,EventTermCfg(func=reset,mode='reset')
 return A()

def install():
 from yam_arena import scene
 old=scene.task_object;old_states=scene.object_states
 def convert(spec):return asset(spec) if isinstance(spec,Bracelet) else old(spec)
 def states(env,objects):
  return old_states(env,[o for o in objects if o!='bracelet'])
 scene.task_object=convert;scene.object_states=states
install()
