import numpy as np
from native_model import NativeModel
from geometry import HAIRS,SEGMENTS,SNAGS_PER_LOCK
class Model(NativeModel):
 def __init__(self):super().__init__();self.comb_touched_locks=set();self.comb_loosened_locks=set();self.last_comb_contact={};self.hand_contacted_trapped_locks=set();self.previous_broken=set();self.grasp_state={}
 def bind_native(self,base):
  super().bind_native(base)
  from isaaclab.app.settings_manager import get_settings_manager
  get_settings_manager().set_bool('/physics/disableContactProcessing',False)
  from pxr import UsdUtils
  import omni.physics.tensors as tensors
  cache=UsdUtils.StageCache.Get();stage_id=cache.GetId(self.stage).ToLongInt()
  if stage_id<0:cache.Insert(self.stage);stage_id=cache.GetId(self.stage).ToLongInt()
  self.contact_sim=tensors.create_simulation_view('torch',stage_id);self.contact_sim.set_subspace_roots('/')
  roots={p.GetName():str(p.GetPath()) for p in self.stage.Traverse() if '/materials/' in str(p.GetPath()) and p.GetName() in self.index}
  hair_paths=[roots[f'hair_{s}_{i}'] for s in range(HAIRS) for i in range(SEGMENTS)]
  self.contact_view=self.contact_sim.create_rigid_contact_view([roots['comb']],filter_patterns=[hair_paths],max_contact_data_count=4096)
  if self.contact_view.sensor_count!=1 or self.contact_view.filter_count!=len(hair_paths):raise RuntimeError('Incomplete native comb contact matrix')
  paths=list(self.contact_view.filter_paths)
  if paths and isinstance(paths[0],(list,tuple)):paths=paths[0]
  self.contact_order=np.asarray([paths.index(p) for p in hair_paths]);self.contact_dt=float(base.physics_dt)
  from pxr import UsdPhysics,PhysxSchema
  hand_paths=[]
  for prim in self.stage.Traverse():
   path=str(prim.GetPath())
   if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in ('gripper','tip_left','tip_right') and any(arm in path for arm in ('LeftArm','RightArm')):
    PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr(0.);hand_paths.append(path)
  if len(hand_paths)!=6:raise RuntimeError('Expected both grippers and four fingers')
  self.hand_contact_view=self.contact_sim.create_rigid_contact_view(hand_paths,filter_patterns=[hair_paths for _ in hand_paths],max_contact_data_count=4096)
  if self.hand_contact_view.sensor_count!=6 or self.hand_contact_view.filter_count!=len(hair_paths):raise RuntimeError('Incomplete native hand/hair contact matrix')
  paths=list(self.hand_contact_view.filter_paths)
  if paths and isinstance(paths[0],(list,tuple)):paths=paths[0]
  self.hand_contact_order=np.asarray([paths.index(p) for p in hair_paths])
  from pxr import PhysicsSchemaTools
  from omni.physx import get_physx_simulation_interface
  def contact(headers,data):
   for h in headers:
    names=[str(PhysicsSchemaTools.intToSdfPath(v)).rsplit('/',1)[-1] for v in [h.actor0,h.actor1]]
    if 'comb' not in names or not h.num_contact_data:continue
    other=names[1-names.index('comb')]
    if not other.startswith('hair_'):continue
    strand=int(other.split('_')[1])
    impulse=sum(float(np.linalg.norm(data[i].impulse)) for i in range(h.contact_data_offset,h.contact_data_offset+h.num_contact_data))
    if impulse<=0.:continue
    self.last_comb_contact[strand]=self.time
    if strand in self.comb_touched_locks:continue
    self.comb_touched_locks.add(strand);self.events.append(dict(event='comb_hair_contact',lock=strand,body=other,impulse_ns=impulse,time=self.time,source='native per-substep contact report'))
  self.contact_subscription=get_physx_simulation_interface().subscribe_contact_report_events(contact)
  from finite_pinch import FinitePinch
  self.pinch=FinitePinch(self,['comb','brush'],{'comb':.006,'brush':.0105},.002,.013,(-.011,.013))
 def record_tool_contacts(self):
  value=self.contact_view.get_contact_force_matrix(dt=self.contact_dt)
  forces=getattr(value,'torch',value).detach().cpu().numpy()[0,self.contact_order]
  magnitudes=np.linalg.norm(forces,axis=1).reshape(HAIRS,SEGMENTS)
  for strand in range(HAIRS):
   index=int(np.argmax(magnitudes[strand]));force=float(magnitudes[strand,index])
   if force>1e-6:
    self.last_comb_contact[strand]=self.time
    if strand in self.comb_touched_locks:continue
    self.comb_touched_locks.add(strand);self.events.append(dict(event='comb_hair_contact',lock=strand,body=f'hair_{strand}_{index}',impulse_ns=force*self.contact_dt,force_n=force,time=self.time,source='native filtered contact-force matrix'))
 def record_direct_hand_contacts(self):
  value=self.hand_contact_view.get_contact_force_matrix(dt=self.contact_dt)
  forces=getattr(value,'torch',value).detach().cpu().numpy()[:,self.hand_contact_order]
  magnitudes=np.linalg.norm(forces,axis=2).reshape(6,HAIRS,SEGMENTS).max(axis=(0,2))
  for strand,force in enumerate(magnitudes):
   # Catch direct handling before a snag breaks, including its break step.
   trapped=any(f'hair_snag_{strand}_{i+1}' not in self.previous_broken for i in range(SNAGS_PER_LOCK))
   if trapped and force>1e-6 and strand not in self.hand_contacted_trapped_locks:
    self.hand_contacted_trapped_locks.add(strand)
    self.events.append(dict(event='hand_contacted_trapped_hair',lock=strand,force_n=float(force),time=self.time,source='native filtered hand/hair contact-force matrix'))
 def configure_station(self):
  from pad_covers import install
  install()
 def material(self,hands,dt):
  self.record_tool_contacts();self.record_direct_hand_contacts()
  for name in self.broken-self.previous_broken:
   if not name.startswith('hair_snag_'):continue
   strand=int(name.split('_')[2]);last=self.last_comb_contact.get(strand,-1e9)
   if self.time-last<=.25 and strand not in self.hand_contacted_trapped_locks:
    self.comb_loosened_locks.add(strand);self.events.append(dict(event='comb_loosened_snag',lock=strand,joint=name,time=self.time,last_comb_contact=last))
  self.previous_broken=set(self.broken);self.pinch.update(hands);self.failed |= any(n.startswith('pin_') for n in self.broken)
 def measure(self):
  hair=np.array([self.p(f'hair_{s}_{i}') for s in range(HAIRS) for i in range(SEGMENTS)])
  local=(hair-self.p('brush'))@self.orientation('brush')
  trapped=(abs(local[:,0])<.029)&(abs(local[:,1])<.050)&(local[:,2]>-.010)&(local[:,2]<.033)
  # Count a strand as cleared only when its entire centreline leaves the
  # expanded bristle envelope. This corresponds to weighing removed locks.
  clear=~trapped.reshape(HAIRS,SEGMENTS).any(axis=1)
  qualified=clear & np.array([s in self.comb_loosened_locks and s not in self.hand_contacted_trapped_locks for s in range(HAIRS)])
  return dict(removed_fraction=float(clear.mean()),comb_cleared_fraction=float(qualified.mean()),comb_touched_locks=len(self.comb_touched_locks),comb_loosened_locks=len(self.comb_loosened_locks),directly_handled_trapped_locks=len(self.hand_contacted_trapped_locks),remaining_contact_snags=HAIRS*SNAGS_PER_LOCK-sum(n.startswith('hair_snag_') for n in self.broken),intact=not self.failed,released_seconds=self.release,failed=self.failed)
 def reset_valid(self):
  q=self.measure();return q['removed_fraction']==0. and q['remaining_contact_snags']==HAIRS*SNAGS_PER_LOCK and not self.failed
 def score(self):
  self.failed |= self.integrity_failed()
  if self.failed:return 0.
  x=self.measure()['comb_cleared_fraction']
  if x>=.9 and self.release>=3:return 1.
  return .5 if x>=.5 else (.25 if x>=.25 else 0.)
 def integrity_failed(self):return super().integrity_failed() or any(n.startswith('pin_') for n in self.broken)

 def draw(self,d):
  from hair_visual import draw
  draw(self,d)
 def state(self):return dict(super().state(),comb_touched_locks=sorted(self.comb_touched_locks),comb_loosened_locks=sorted(self.comb_loosened_locks),last_comb_contact=self.last_comb_contact,hand_contacted_trapped_locks=sorted(self.hand_contacted_trapped_locks),previous_broken=sorted(self.previous_broken),grasp_state=self.grasp_state)
 def restore(self,state):
  s=dict(state);self.comb_touched_locks=set(s.pop('comb_touched_locks'));self.comb_loosened_locks=set(s.pop('comb_loosened_locks'));self.last_comb_contact={int(k):v for k,v in s.pop('last_comb_contact').items()};self.hand_contacted_trapped_locks=set(s.pop('hand_contacted_trapped_locks'));self.previous_broken=set(s.pop('previous_broken'));super().restore(s)
