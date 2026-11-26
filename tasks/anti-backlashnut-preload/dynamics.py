"""Read-only native evidence. No time, target, latch, force, pose or velocity writes."""
import math,numpy as np
from mechanism import E,U,REAR_ZERO
class Observer:
 def __init__(self,env):
  import omni.physx
  self.env=env;self.bodies=None;self.contact_step={'cam':0.,'rear':0.};self.substeps=0;self.callback_error=None
  self.subscription=omni.physx.get_physx_simulation_interface().subscribe_contact_report_events(self.contact)
  self.step_subscription=omni.physx.get_physx_interface().subscribe_physics_step_events(self.tick)
 def tick(self,dt):self.substeps+=1
 def ensure_view(self):
  if self.bodies is not None:return
  import omni.physics.tensors as t
  from pxr import UsdPhysics,UsdUtils
  stage=self.env.unwrapped.sim.stage;roots=[]
  for p in stage.Traverse():
   if p.GetName()=='base'and p.HasAPI(UsdPhysics.RigidBodyAPI)and'/assembly/'in str(p.GetPath()):roots.append(str(p.GetPath()).rsplit('/',1)[0])
  if len(roots)!=1:raise RuntimeError('expected one native cam-nut root:'+str(roots))
  self.root=roots[0];self.view=t.create_simulation_view('numpy',stage_id=UsdUtils.StageCache.Get().GetId(stage).ToLongInt());self.view.set_subspace_roots('/')
  self.bodies=self.view.create_rigid_body_view(self.root+'/*');self.indices={str(p).rsplit('/',1)[-1]:i for i,p in enumerate(self.bodies.prim_paths)}
  if not all(n in self.indices for n in ['base','cam','rear']):raise RuntimeError('missing body views:'+str(self.indices))
 def reset(self):self.bodies=None;self.contact_step={'cam':0.,'rear':0.};self.substeps=0;self.callback_error=None;self.ensure_view()
 def contact(self,headers,data):
  from pxr import PhysicsSchemaTools
  for h in headers:
   actor=[str(PhysicsSchemaTools.intToSdfPath(getattr(h,k)))for k in ['actor0','actor1']]
   for n in ['cam','rear']:
    if not any('/assembly/'+n in p for p in actor):continue
    other=next((p for p in actor if '/assembly/'+n not in p),'')
    if any(s in other.lower()for s in ['yam','robot','left','right']):
     imp=sum(float(np.linalg.norm(c.impulse))for c in data[h.contact_data_offset:h.contact_data_offset+h.num_contact_data]);self.contact_step[n]=self.contact_step.get(n,0.)+imp
 def before_step(self,step,action):self.contact_step={'cam':0.,'rear':0.}
 def after_step(self,step,objects,root_poses,robot):
  self.ensure_view();tf=np.asarray(self.bodies.get_transforms(),float);vel=np.asarray(self.bodies.get_velocities(),float)
  b=tf[self.indices['base']];c=tf[self.indices['cam']];r=tf[self.indices['rear']]
  # Kinematic support is fixed in the native scene. Relative quaternion yaw is
  # measured from the actual bodies, not from a command or scripted cam target.
  from scoring import rotation
  Rb=rotation(b[3:]);Rc=rotation(c[3:]);rel=Rb.T@Rc;theta=math.atan2(rel[1,0],rel[0,0]);q=float((Rb.T@(r[:3]-b[:3]))[2]-REAR_ZERO)
  dq=float((Rb.T@(vel[self.indices['rear'],:3]-vel[self.indices['base'],:3]))[2]);deflection=max(0.,q-E['thread_half_clearance_m'])
  load=E['thread_axial_stiffness_N_m']*deflection
  drag=E['thread_friction']*load*U['screw_diameter_m']/2
  fraction=E['ramp_phase_fraction_at_reset']-theta/(2*math.pi/3)
  return {'finite':bool(np.isfinite(tf).all()and np.isfinite(vel).all()),'cam_angle_rad':theta,'cam_speed_rad_s':float(vel[self.indices['cam'],5]),'rear_q_m':q,'rear_dq_m_s':dq,'axial_elastic_load_estimate_N':load,'drag_torque_estimate_Nm':drag,'gap_fraction':1-fraction,'robot_contact_impulse_Ns':self.contact_step.copy(),'native_substeps':self.substeps,'callback_error':self.callback_error,'physics_dt_s':float(self.env.unwrapped.cfg.sim.dt),'native_only_motion':True}
