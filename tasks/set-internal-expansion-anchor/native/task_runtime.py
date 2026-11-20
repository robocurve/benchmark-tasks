"""Scene, sleeve material law, trace recording and outcome score of the anchor-setting task. No body pose writes."""
from pathlib import Path
import os,sys,json,hashlib,math
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent;sys.path.insert(0,str(HERE))
import native_geometry as ng
from engineering_parameters import PassivePlasticRoot,hinge_parameters
GROOT=Path(os.environ.get('ROBOCURVE_GEOMETRY_ROOT',str(HERE.parent/'geometry')))
GFILE=GROOT/'geometry.json';G=json.load(open(GFILE));GSHA=hashlib.sha256(GFILE.read_bytes()).hexdigest()
TRACE=Path(os.environ.get('ROBOCURVE_TRACE_DIR',str(HERE/'trace')))
from yam_arena import station
station.BASES={s:tuple(p)for s,p in G['robot_bases_m'].items()}
OBJECTS=ng.objects(G);SCENE='station';SCORE_OVER='end';SUCCESS_HOLD_S=.05;CONTROL_HZ=20.
_COMMAND=None;CONTROL=0;LAST_ACTUAL=None;PHASE_PROVIDER=lambda:'passive_reset'
MATERIAL={};MATERIAL_LOGGED_STEP=None;LAST_MATERIAL={};CONTACT_STEP=None;MATERIAL_DRIVES={};CONTACT_STEP=None
BLOW_PEAK=0.   # largest hammer-on-punch contact force (N) over the physics steps since the last control step
from contact_diagnostics import Diagnostics
CONTACTS=Diagnostics(sys.modules[__name__])
from primary_score import PrimaryScore
PRIMARY=PrimaryScore()

def arr(x):
 x=getattr(x,'torch',x)
 if hasattr(x,'detach'):return x.detach().cpu().numpy().copy()
 if hasattr(x,'numpy'):return x.numpy().copy()
 return np.array(x)
def save(name,obj):
 TRACE.mkdir(parents=True,exist_ok=True);(TRACE/name).write_text(json.dumps(obj,allow_nan=False,indent=2)+'\n')
def poses(base):
 return {p['name']:arr(base.scene[p['name']].root_view.get_transforms())[0].tolist()for p in G['parts']}

def material_update(base,t):
 global MATERIAL_LOGGED_STEP,LAST_MATERIAL
 if MATERIAL_LOGGED_STEP is not None and t<=MATERIAL_LOGGED_STEP:return
 from pxr import UsdPhysics
 ps=poses(base);parent=Rotation.from_quat(ps['anchor_ring'][3:]).as_matrix();rows={}
 for j in G['joints']:
  name=j['child'];R=parent.T@Rotation.from_quat(ps[name][3:]).as_matrix()
  angle=float(np.dot(Rotation.from_matrix(R).as_rotvec(),j['axes_child'][0]))
  law=MATERIAL[j['name']];r=law.observe(angle,t)
  path=base.scene.env_prim_paths[0]+'/dropin_material_joints/'+j['name']
  drive=UsdPhysics.DriveAPI(base.scene.stage.GetPrimAtPath(path),'angular')
  # Only resisting material reststate changes. No angular/body pose/velocity target
  # is selected from task phases or commanded object expansion.
  target=math.degrees(r['plastic_angle']);drive.GetTargetPositionAttr().Set(target)
  r.update(USD_target_plastic_angle_deg=float(drive.GetTargetPositionAttr().Get()),
           joint=path)
  rows[name]=r
 MATERIAL_LOGGED_STEP=t;LAST_MATERIAL=rows
 TRACE.mkdir(parents=True,exist_ok=True)
 with(TRACE/'material.jsonl').open('a')as f:f.write(json.dumps(dict(actual_physics_step=t,parts=rows),allow_nan=False)+'\n')

@ng.pk.on_substep
def substep(env):
 global CONTACT_STEP,BLOW_PEAK
 base=env.unwrapped
 if not MATERIAL:return
 step=int(base._sim_step_counter);material_update(base,float(step))
 if CONTACT_STEP==step:return
 CONTACT_STEP=step
 if CONTACTS.views:
  sample={}
  for key in ['hammer_punch','punch_wedge','punch_ring']:
   v=CONTACTS.views.get(key)
   if v is None:sample[key]={'unknown':CONTACTS.errors.get(key,'unavailable')}
   else:
    try:
     normal,point,direction,separation,counts,starts=[arr(x)for x in v.get_contact_data(dt=float(base.physics_dt))];entries=[]
     for ix in np.ndindex(counts.shape):
      for j in range(int(starts[ix]),int(starts[ix])+int(counts[ix])):entries.append(dict(normal_force_N=float(np.asarray(normal[j]).reshape(-1)[0]),point_world_m=point[j].tolist(),normal_world=direction[j].tolist(),separation_m=np.asarray(separation[j]).tolist()))
     sample[key]=dict(counts=counts.tolist(),entries=entries)
    except Exception as e:sample[key]={'unknown':type(e).__name__+':'+str(e)}
  BLOW_PEAK=max(BLOW_PEAK,sum(abs(x['normal_force_N'])for x in sample['hammer_punch'].get('entries',[])))
  with(TRACE/'impact-contacts.jsonl').open('a')as f:f.write(json.dumps(dict(actual_physics_step=step,physics_dt_s=float(base.physics_dt),pairs=sample),allow_nan=False)+'\n')

@ng.pk.on_reset
def reset(env):
 global CONTROL,_COMMAND,LAST_ACTUAL,MATERIAL_LOGGED_STEP,LAST_MATERIAL,CONTACT_STEP,BLOW_PEAK
 BLOW_PEAK=0.;CONTROL=0;_COMMAND=None;LAST_ACTUAL=None;CONTACTS.reset();PRIMARY.reset();MATERIAL.clear();MATERIAL_LOGGED_STEP=None;LAST_MATERIAL={};CONTACT_STEP=None
 hp=hinge_parameters()
 for j in G['joints']:MATERIAL[j['name']]=PassivePlasticRoot(hp['stiffness_Nm_per_rad'],hp['yield_moment_Nm'])


def inventory(base):
 from pxr import Usd,UsdPhysics,PhysxSchema
 root=base.scene.env_prim_paths[0];shapes=[];joints=[];robots=[]
 for prim in base.scene.stage.Traverse(Usd.TraverseInstanceProxies()):
  p=str(prim.GetPath())
  if prim.HasAPI(UsdPhysics.CollisionAPI):
   if any(p.startswith(root+'/'+n['name']+'/')for n in G['parts']):shapes.append(dict(path=p,source=prim.GetCustomDataByKey('source_geometry_name')))
   elif'/LeftArm/'in p or'/RightArm/'in p:robots.append(p)
  if prim.IsA(UsdPhysics.Joint)and'/dropin_material_joints/'in p:
   j=UsdPhysics.Joint(prim);d=UsdPhysics.DriveAPI(prim,'angular')
   joints.append(dict(path=p,bodies=[str(x)for rel in[j.GetBody0Rel(),j.GetBody1Rel()]for x in rel.GetTargets()],stiffness_Nm_per_degree=d.GetStiffnessAttr().Get(),max_force_Nm=d.GetMaxForceAttr().Get()))
 if len(shapes)!=sum(len(p['geoms'])for p in G['parts'])or len(joints)!=4:raise RuntimeError('Actualinventory mismatch')
 bodies={}
 for p in G['parts']:
  v=base.scene[p['name']].root_view;m=float(arr(v.get_masses())[0])
  if not np.isclose(m,p['mass_kg'],rtol=1e-5):raise RuntimeError('Actualmass mismatch '+p['name'])
  bodies[p['name']]=dict(mass_kg=m,shape_count=int(v.max_shapes),fixed=p['fixed'])
 save('NATIVE-INVENTORY.json',dict(passed=True,rigid_only=True,geometry_sha256=GSHA,shapes=shapes,joints=joints,bodies=bodies,robot_colliders=robots,physics_dt=float(base.physics_dt),control_dt=float(base.step_dt),object_pose_writes_after_reset=0,external_task_wrenches=0,formal_score=None))
 base.scene.stage.Export(str(TRACE/'authored.usda'))

@ng.pk.on_step
def observe(env):
 global CONTROL,LAST_ACTUAL,BLOW_PEAK
 if _COMMAND is None:return
 base=env.unwrapped;CONTROL+=1
 if CONTROL==1:inventory(base)
 ps=poses(base);bodies={};robots={}
 for p in G['parts']:
  v=base.scene[p['name']].root_view;vel=arr(v.get_velocities())[0]
  bodies[p['name']]=dict(pose_xyzw=ps[p['name']],native_velocity=vel.tolist())
 for s in ['left','right']:
  a=base.scene[s+'_robot'];v=a.root_view
  robots[s]=dict(joint_names=list(a.backend_joint_names),joint_positions=arr(v.get_dof_positions())[0].tolist(),body_names=list(a.backend_body_names),body_link_pose_w=arr(v.get_link_transforms())[0].tolist())
 if not all(np.isfinite(b['pose_xyzw']).all()and np.isfinite(b['native_velocity']).all()for b in bodies.values()):raise RuntimeError('Nonfiniteactualbody')
 ring=np.array(ps['anchor_ring']);R=Rotation.from_quat(ring[3:]).as_matrix()
 relative={n:(R.T@(np.array(p[:3])-ring[:3])).tolist()for n,p in ps.items()if n in ['setting_wedge','punch']}
 row=dict(control=CONTROL,time_s=CONTROL/20,physics_step=int(base._sim_step_counter),geometry_sha256=GSHA,phase=PHASE_PROVIDER(),command14=_COMMAND,rigid_bodies=bodies,robots=robots,material=LAST_MATERIAL,relative_to_ring=relative,formal_score=None,score=0.,object_pose_writes_after_reset=0,external_task_wrenches=0,contacts=CONTACTS.observe(base))
 row['hammer_punch_peak_N']=float(BLOW_PEAK);BLOW_PEAK=0.
 row['primary']=PRIMARY.observe(row,G);row['score']=PRIMARY.value
 LAST_ACTUAL=row;TRACE.mkdir(parents=True,exist_ok=True)
 with(TRACE/'observations.jsonl').open('a')as f:f.write(json.dumps(row,allow_nan=False)+'\n')
 if CONTROL%20==0:print('DROPIN_ACTUAL '+json.dumps(dict(control=CONTROL,phase=row['phase'])),flush=True)

_prevscene=ng.pk._Finder.targets['yam_arena.scene']
def patch_scene(mod):
 _prevscene(mod)
 if getattr(mod,'_dropin_timing',False):return
 old=mod.env_cfg_callback
 def cb(control_hz=None):
  fn=old(control_hz)
  def configure(cfg):
   cfg=fn(cfg);cfg.sim.dt=1/960;cfg.decimation=48;cfg.sim.render_interval=48;return cfg
  return configure
 mod.env_cfg_callback=cb;mod._dropin_timing=True
ng.pk._Finder.targets['yam_arena.scene']=patch_scene
_previe=ng.pk._Finder.targets['yam_arena.inspect_embodiment']
def patch_inspect(mod):
 _previe(mod);cls=mod.YamArenaEmbodiment
 if getattr(cls,'_dropin_command',False):return
 old=cls.step
 def step(self,action):
  global _COMMAND
  _COMMAND=np.asarray(action.data,float).reshape(-1).tolist();return old(self,action)
 cls.step=step;cls._dropin_command=True
ng.pk._Finder.targets['yam_arena.inspect_embodiment']=patch_inspect
for name,patch in [('yam_arena.scene',patch_scene),('yam_arena.inspect_embodiment',patch_inspect)]:
 if name in sys.modules:patch(sys.modules[name])

def score(states):return PRIMARY.value
def success(states):return PRIMARY.value>=1.
