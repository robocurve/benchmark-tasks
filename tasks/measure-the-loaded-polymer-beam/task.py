"""A materials bench holds a full lever-operated test stand with bending rollers, a loading nose and force gauge,
a digital deflection indicator under the rollers' midpoint, a white polymer bar and two specimen cradles.
A flat display beside the stand shows live force in N and centre deflection in mm. The stand's spring-loaded
lever sits on the right of its head and ends in a black knob; pulling the knob down lowers the loading nose.
"""
from __future__ import annotations
import json,math,os,pathlib,sys
from functools import lru_cache
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parent))
import parts,lots,tilt
from yam_arena.task_file import TABLE_TOP
from yam_arena.station import TABLE_CENTER,TABLE_SIZE
from yam_arena.demo import Waypoint,FINGER_OPEN,grip_rotation
SCENE='station'; TOP=TABLE_TOP[SCENE]; SCORE_OVER='end'; SUCCESS_HOLD_S=.1
GREY=(.58,.60,.63);SILVER=(.78,.79,.80);WHITE=(.94,.94,.90);BLUE=(.08,.22,.52);BLACK=(.05,.05,.05)
X=.32; SPAN=.150; SUPPORT_Z=TOP+.1261; BEAM_Z=SUPPORT_Z+.005
# Two cradles side by side on the left of the bench: far (layout A) and near (layout B) the arm bases.
RACK_A=(.20,.130,TOP+.035);RACK_B=(.11,.130,TOP+.035)
# Native stand's published overall dimensions; detailed carriage/lever geometry is estimated.
stand=[parts.box((.1621,.0991,.020),(.03955,0,.010),BLUE),
 parts.cylinder(.009,.3356,(.092,0,.1878),SILVER),
 parts.box((.0701,.0828,.0185),(0,0,.02925),GREY),
 parts.box((.0572,.2116,.0393),(0,0,.05815),SILVER)]
rollers=[]
for y,label in [(-.075,'right'),(.075,'left')]:
 stand.append(parts.box((.033,.012,.0433),(0,y,.09945),GREY))
 rollers.append(parts.Hinge('roller_'+label,(parts.cylinder(.005,.0514,(0,y,.1211),SILVER,'x'),),
  (0,y,.1211),(1,0,0),-math.pi,math.pi,mass=.032,spring=0.,damping=.002,max_effort=1.))
# Gauge/upper anvil/carriage on the genuine load axis. Nose transmits contact to the beam.
nose=[parts.cylinder(.005,.0514,(0,0,.1471),SILVER,'x'),
 parts.box((.033,.014,.0395),(0,0,.17185),GREY),
 parts.cylinder(.004,.010,(0,0,.1966),SILVER),
 parts.box((.035,.0711,.145),(0,0,.2741),BLUE),parts.box((.001,.045,.025),(-.018,0,.290),BLACK),
 parts.box((.070,.030,.038),(.054,0,.302),GREY)]
# Native spring-loaded lever on the side of the repositionable head, scaled from the maker drawing: hub 32 mm in front
# of the column, 105 mm lever with a ball-type knob, resting 40 degrees above horizontal toward the operator.
HUB=(.060,-.070,.2425);LEVER_L=.105;LEVER_REST=math.radians(40.)
# Digital indicator (drawn only): body on the lower fixture, plunger up to the bar's underside at mid-span.
stand+=[parts.box((.020,.056,.036),(0,0,.0958),BLACK,collide=False),parts.cylinder(.0025,.0118,(0,0,.1197),SILVER,collide=False)]
stand.append(parts.box((.060,.060,.075),(.075,0,.2425),GREY))
def lever_point(s,phi=0.):
 b=LEVER_REST-phi;return (HUB[0]-s*math.cos(b),HUB[1],HUB[2]+s*math.sin(b))
lever=[parts.cylinder(.012,.046,(HUB[0],-.051,HUB[2]),BLACK,'y')]
for i in range(9):lever.append(parts.box((.009,.008,.009),lever_point(.016+.0095*i),SILVER))
lever.append(parts.cylinder(.013,.028,lever_point(LEVER_L),BLACK,'y'))
MACHINE=parts.Part('test_stand',pos=(X,0,TOP),shapes=stand,joints=(
 parts.Hinge('lever',tuple(lever),HUB,(0,-1,0),0.,.038/.018,mass=.10,spring=.3,damping=.05,max_effort=8.),
 parts.Slider('nose',tuple(nose),(0,0,-1),.038,mass=.65,spring=0.,damping=.8,max_effort=150.),*rollers),
 couplings=(('nose','lever',.018),),friction=(.20,.20))
# Equivalent centre mode. End patches stay in one root body; no geometric beam FEM is claimed.
def specimen(name,thick,pos):
 k=48*600e6*(.025*thick**3/12)/SPAN**3
 return parts.Part(name,pos=pos,mass=930*.1*.025*thick,shapes=[
  parts.box((.025,.050,thick),(0,-.075,0),WHITE),parts.box((.025,.050,thick),(0,.075,0),WHITE),
  *[parts.box((.0002,.004,.004),(-.0126,y,0),BLACK,collide=False) for y in (-.075,.075)]],
  joints=(parts.Slider('bend',(parts.box((.025,.099,thick),(0,0,0),WHITE),parts.box((.0002,.004,.004),(-.0126,0,0),BLACK,collide=False),parts.box((.0002,.004,.004),(.0126,0,0),BLACK,collide=False)),(0,0,-1),.0045,
  mass=930*.1*.025*thick,spring=k,damping=4.,max_effort=100.),),friction=(.20,.20))
BEAM=specimen('polymer_beam',.010,RACK_A)
# Both layouts use this one 10 mm specimen; layout B starts it on the near cradle.
RACKS=[]
for label,pos in [('far',RACK_A),('near',RACK_B)]:
 shapes=[parts.box((.070,.220,.010),(0,0,-.030),BLUE)]
 for y in (-.075,.075):shapes.append(parts.box((.040,.025,.020),(0,y,-.015),GREY))
 RACKS.append(parts.Part(label+'_rack',pos=pos,shapes=shapes,friction=(.20,.20)))
# The digital indicator below the bar's centre is drawn on the lower fixture but has no contact; geometry() gives
# its ideal reading (centre relative to both support patches). The small black face marks are witness marks.
OBJECTS=[MACHINE,BEAM,*RACKS]
LOTS=lots.declare(OBJECTS,lots.Lot('A',place={'polymer_beam':RACK_A}),lots.Lot('B',place={'polymer_beam':RACK_B}))
DEMO_TOUCHES=set()

def stamp(s):return tuple(float(v) for v in np.r_[s.pos,s.quat_xyzw,s.lin_vel,s.ang_vel,*s.joints.values()])
def pose(s):return parts._quat_to_matrix(s.quat_xyzw)
def geometry(s):
 r=pose(s);p=np.asarray(s.pos)
 ends=np.array([p+r@np.array((0,y,0.)) for y in (-.075,.075)])
 centre=s.links['bend'][0]
 # Centre relative to the end/support chord removes all rigid translation/contact compression.
 delta=float((ends[:,2].mean()-centre[2])*1000.)
 upright=float(r[2,2])>=math.cos(math.radians(2.))
 yaw=abs(math.atan2(r[1,0],r[0,0]));yaw=min(yaw,math.pi-yaw)  # the bar is symmetric end for end
 placed=upright and yaw<=math.radians(2.)+1e-9 and np.linalg.norm(p[:2]-np.array((X,0.)))<=.002000001 and max(abs(ends[:,2]-(SUPPORT_Z+.005)))<=.001
 return dict(placed=bool(placed),delta=delta,ends=ends,centre=np.asarray(centre),yaw=math.atan2(r[1,0],r[0,0]))

@lru_cache(maxsize=1)
def hand_mesh():
 from yam_arena import analysis
 result={}
 for name in ('link1','link2','link3','link4','link5','gripper','tip_left','tip_right'):
  cloud=[]
  for h in analysis.link_hulls().get(name,[]):
   cloud.extend(h.vertices)
   for pl in np.unique(h.planes.round(7),axis=0):
    face=h.vertices[np.abs(h.vertices@pl[:3]-pl[3])<2e-6]
    if len(face)<3:continue
    c=face.mean(0);u=face[0]-c;u/=np.linalg.norm(u);v=np.cross(pl[:3],u)
    face=face[np.argsort(np.arctan2((face-c)@v,(face-c)@u))]
    for a,b in zip(face,np.roll(face,-1,axis=0)):
     n=max(1,math.ceil(max(np.linalg.norm(a-c),np.linalg.norm(b-c),np.linalg.norm(a-b))/.003))
     for i in range(n+1):
      for j in range(n-i+1):cloud.append(c+(a-c)*i/n+(b-c)*j/n)
  if not cloud:raise ValueError('missing hand mesh')
  cloud=np.asarray(cloud);_,keep=np.unique(np.floor(cloud/.002).astype(int),axis=0,return_index=True);result[name]=cloud[keep]
 return result

def hand_state(sim):
 base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);clouds=[]
 for arm in ('left_robot','right_robot'):
  robot=base.scene[arm];poses=parts._np(robot.data.body_link_pose_w)[0]
  for name,points in hand_mesh().items():
   p=poses[robot.body_names.index(name)];clouds.append(points@parts._quat_to_matrix(p[3:]).T+p[:3]-origin)
 return clouds

def gap(clouds,spec,s):
 if not clouds:return -math.inf
 best=math.inf
 poses={'base':(s.pos,pose(s)),**{n:(p,parts._quat_to_matrix(q)) for n,(p,q) in s.links.items()}}
 for _,c,r,size in spec.world_boxes(poses):
  for cloud in clouds:best=min(best,float(np.linalg.norm(np.maximum(np.abs((cloud-c)@r)-size/2,0),axis=1).min()))
 return best

def arr(x):
 if hasattr(x,'torch'):x=x.torch
 return x.detach().cpu().numpy() if hasattr(x,'detach') else np.asarray(x.numpy())
_views=None

def forces(sim):
 global _views
 if _views is None:
  from isaaclab_physx.physics import PhysxManager
  root=parts.ROOT_PATHS;sv=PhysxManager.get_physics_sim_view()
  # Nose-to-centre force and root-to-two original rollers are independent contact measurements.
  from yam_arena.robot import asset_link_paths
  links=asset_link_paths();envpath=sim.env.unwrapped.scene.env_prim_paths[0]
  handpaths=[envpath+'/'+arm+links[name] for arm in ('LeftArm','RightArm') for name in hand_mesh()]
  _views=[sv.create_rigid_contact_view([root['polymer_beam']+'/bend'],filter_patterns=[[root['test_stand']+'/nose']],max_contact_data_count=128),
   sv.create_rigid_contact_view([root['polymer_beam']+'/base'],filter_patterns=[[root['test_stand']+'/roller_left']],max_contact_data_count=128),
   sv.create_rigid_contact_view([root['polymer_beam']+'/base'],filter_patterns=[[root['test_stand']+'/roller_right']],max_contact_data_count=128),
   sv.create_rigid_contact_view([root['polymer_beam']+'/base',root['polymer_beam']+'/bend'],filter_patterns=[handpaths,handpaths],max_contact_data_count=256)]
 vals=[arr(v.get_contact_force_matrix(dt=sim.env.unwrapped.sim.get_physics_dt())).reshape(-1,3) for v in _views]
 if any(not np.isfinite(x).all() for x in vals):raise ValueError('unknown contacts')
 return abs(float(vals[0][:,2].sum())),abs(float(vals[1][:,2].sum())),abs(float(vals[2][:,2].sum())),float(np.linalg.norm(vals[3],axis=1).max())

# Visible seven-segment instrument repeater (same quantized values supplied to demo feedback).
# The repeater lies flat to the right of the stand, clear of both cradles.
PANEL_DX,PANEL_DY=.19,-.055
DISPLAY={'force':0.,'delta':0.,'known':False};_digits={};_display_stage=None;_display_values=None
SEGMENTS={'0':'abcdef','1':'bc','2':'abdeg','3':'abcdg','4':'bcfg','5':'acdfg','6':'acdefg','7':'abc','8':'abcdefg','9':'abcdfg','-':'g',' ':'','?':'abg'}

def render_displays(force,delta):
 global _display_stage,_display_values
 values=(f'{force:05.2f}',f'{delta:05.2f}')
 if _display_values==values and _digits:return
 _display_values=values
 from pxr import Gf,UsdGeom
 from isaaclab.sim.utils import get_current_stage
 stage=get_current_stage()
 if not _digits:
  _display_stage=stage;_digits.clear()
  panel=UsdGeom.Cube.Define(stage,'/World/BeamDisplay/panel');panel.CreateSizeAttr(1.)
  panel.AddTranslateOp().Set(Gf.Vec3d(.205+PANEL_DX,-.100+PANEL_DY,TOP+.015));panel.AddScaleOp().Set(Gf.Vec3f(.190,.090,.006))
  panel.CreateDisplayColorAttr().Set([Gf.Vec3f(.015,.025,.020)])
  # Large repeater labels are integral to the instrument faceplate.
  glyphs={'N':['10001','11001','10101','10011','10001','10001','10001'],
          'M':['10001','11011','10101','10101','10001','10001','10001']}
  for label,y in [('N',-.08),('MM',-.12)]:
   for c,ch in enumerate(label):
    for j,line in enumerate(glyphs[ch]):
     for i,pixel in enumerate(line):
      if pixel=='1':
       g=UsdGeom.Cube.Define(stage,f'/World/BeamDisplay/unit{int(-y*100)}c{c}p{i}r{j}');g.CreateSizeAttr(1.)
       g.AddTranslateOp().Set(Gf.Vec3d(.275+PANEL_DX+c*.013+i*.002,y+PANEL_DY+.007-j*.002,TOP+.020));g.AddScaleOp().Set(Gf.Vec3f(.0017,.0017,.001))
       g.CreateDisplayColorAttr().Set([Gf.Vec3f(.95,.95,.95)])
  # Six large digits per channel.
  for row in range(2):
   for digit in range(6):
    for seg,(dx,dy,sx,sy) in {'a':(0,.012,.012,.002),'b':(.007,.006,.002,.010),'c':(.007,-.006,.002,.010),'d':(0,-.012,.012,.002),'e':(-.007,-.006,.002,.010),'f':(-.007,.006,.002,.010),'g':(0,0,.012,.002),'.':(.010,-.013,.002,.002)}.items():
     path=f'/World/BeamDisplay/r{row}d{digit}{seg if seg!="." else "dot"}'
     g=UsdGeom.Cube.Define(stage,path);g.CreateSizeAttr(1.);g.AddTranslateOp().Set(Gf.Vec3d(.14+PANEL_DX+digit*.020+dx,-.08+PANEL_DY-row*.04+dy,TOP+.020));g.AddScaleOp().Set(Gf.Vec3f(sx,sy,.001))
     g.CreateDisplayColorAttr().Set([Gf.Vec3f(.05,.9,.35)]);_digits[(row,digit,seg)]=g
 for row,value in enumerate(values):
  for i,char in enumerate(value.rjust(6)[:6]):
   for seg in 'abcdefg.':
    g=_digits[row,i,seg];g.CreateVisibilityAttr().Set('inherited' if seg in ('.' if char=='.' else SEGMENTS.get(char,'')) else 'invisible')

_state={}
def reset_state():
 global _state
 _state=dict(known=False,invalid=False,t=0.,placed_hold=0.,reading_hold=0.,records=[],unloaded=False,engaged=False,slip=None,final_hold=0.,certificate=None,history=[],reason='',previous_delta=None,zero=None,previous_placed=False,previous_reading=False,previous_final=False,seated_once=False,last_pose=None)
reset_state()
@parts.on_reset
def reset(sim):
 global _views,_phi,_unload_extra
 reset_state();_views=None;DISPLAY.update(force=0.,delta=0.,known=False);_phi=0.;_unload_extra=0
 from isaaclab.app.settings_manager import get_settings_manager
 get_settings_manager().set_bool('/physics/disableContactProcessing',False)
 sim.set_target('test_stand','lever',-1.)

@parts.on_step
def step(sim):
 st=_state;states=sim.states();s=states['polymer_beam'];dt=float(sim.t)-st['t'];m=geometry(s)
 try:
  clouds=hand_state(sim);f,support_left,support_right,arm_force=forces(sim);support=support_left+support_right;both_supports=support_left>.02 and support_right>.02;clear=gap(clouds,BEAM,s)
  known=0<dt<=.101 and np.isfinite(np.r_[s.pos,s.quat_xyzw,m['delta'],f,support,clear]).all()
 except Exception as e:
  known=False;f=0.;support=0.;support_left=0.;support_right=0.;both_supports=False;arm_force=0.;clear=-1.;st['reason']=str(e)
 st['known']=bool(known)
 if not known:st['invalid']=True
 if st['zero'] is None and known:st['zero']=m['delta']
 delta=m['delta']-(st['zero'] or 0.);DISPLAY.update(force=round(f/.02)*.02,delta=round(delta/.01)*.01,known=bool(known))
 if known:render_displays(DISPLAY['force'],DISPLAY['delta'])
 # Stillness is judged as a camera would: root translation and rotation between consecutive 20 Hz poses.
 r_now=pose(s);last=st['last_pose'];st['last_pose']=(np.asarray(s.pos,float).copy(),r_now)
 if last is None or dt<=0:still=False
 else:
  turn=math.acos(float(np.clip((np.trace(last[1].T@r_now)-1)/2,-1,1)))
  still=bool(np.linalg.norm(np.asarray(s.pos)-last[0])/dt<.002 and turn/dt<.02)
 def invalid(reason):st['invalid']=True;st['reason']=reason
 if s.low[2]<TOP-.015 or np.any(s.low[:2]<np.asarray(TABLE_CENTER[:2])-np.asarray(TABLE_SIZE[:2])/2-1e-6) or np.any(s.high[:2]>np.asarray(TABLE_CENTER[:2])+np.asarray(TABLE_SIZE[:2])/2+1e-6):invalid('off table')
 if delta>4.000001:invalid('overbend >4mm exclusion limit')
 if np.linalg.norm(states['test_stand'].pos-np.asarray(MACHINE.pos))>.005000001:invalid('apparatus moved >5mm')
 placed_ok=m['placed'] and both_supports and arm_force<=.5 and clear>.010 and still
 st['placed_hold']=st['placed_hold']+dt if placed_ok and st['previous_placed'] else 0.
 st['previous_placed']=bool(placed_ok)
 if st['placed_hold']>=1.-1e-8:st['seated_once']=True
 if f>50.000001:invalid('nose load exceeds50N protocol limit')
 if f>2. and clear<=.005:invalid('arm loading on beam')
 if f>2. and (not m['placed'] or not both_supports):invalid('lost support under load')
 if (st['seated_once'] or (m['placed'] and both_supports)) and (len(st['records'])<2 or f>2.) and arm_force>.5 and (f>2. or support>2.):invalid('native arm contact loads specimen')
 if st['engaged'] and (len(st['records'])<2 or f>2.):
  if np.max(np.linalg.norm(m['ends'][:,:2]-st['slip'],axis=1))>.001000001:invalid('support slip >1mm during measurement sequence')
 if f>2. and m['placed'] and both_supports:
  if not st['engaged']:st['engaged']=True;st['slip']=m['ends'][:,:2].copy()
  if np.max(np.linalg.norm(m['ends'][:,:2]-st['slip'],axis=1))>.001000001:invalid('support slip >1mm')
  if clear<=.005:invalid('arm loading on beam')
  if not m['placed']:invalid('lost support')
 if st['engaged'] and ((f>2 and not both_supports) or (len(st['records'])<2 and (not m['placed'] or not both_supports))):invalid('lost support during measurement sequence')
 if st['records'] and len(st['records'])==1 and abs(delta)<=.200001 and f<2.:st['unloaded']=True
 target=1. if not st['records'] else (2. if len(st['records'])==1 and st['unloaded'] else None)
 quiet=st['previous_delta'] is not None and abs(delta-st['previous_delta'])<=.03
 valid=known and not st['invalid'] and target is not None and abs(delta-target)<=.200001 and f>2 and both_supports and clear>.010 and arm_force<=.5 and m['placed'] and quiet and still
 st['reading_hold']=st['reading_hold']+dt if valid and st['previous_reading'] else 0.
 st['previous_reading']=bool(valid)
 st['history'].append(dict(t=float(sim.t),force_N=DISPLAY['force'],delta_mm=DISPLAY['delta'],valid=bool(valid)))
 if st['reading_hold']>=1.-1e-8:
  # The synchronized display record itself is retained, so a5% transcription condition is inapplicable.
  st['records'].append(dict(t=float(sim.t),force_N=DISPLAY['force'],delta_mm=DISPLAY['delta'],samples=st['history'][-21:]));st['reading_hold']=0.
  capture=pathlib.Path(os.environ.get('BEAM_RECORD_DIR',str(pathlib.Path(__file__).with_name('captures'))));capture.mkdir(exist_ok=True,parents=True)
  capture.joinpath('records-'+str(os.getpid())+'-'+lots.current().name+'.json').write_text(json.dumps(st['records'],indent=2))
 p=np.asarray(s.pos);rack=min((RACK_A,RACK_B),key=lambda c:float(np.linalg.norm(p[:2]-np.asarray(c[:2]))))
 racked=np.linalg.norm(p[:2]-np.asarray(rack[:2]))<=.005000001 and abs(p[2]-rack[2])<=.001 and float(pose(s)[2,2])>=math.cos(math.radians(2.))
 final=known and not st['invalid'] and len(st['records'])==2 and racked and abs(delta)<=.200001 and f<2 and clear>=.030 and still and quiet
 st['final_hold']=st['final_hold']+dt if final and st['previous_final'] else 0.
 st['previous_final']=bool(final)
 st.update(t=float(sim.t),previous_delta=delta,certificate=stamp(s))
 if round(sim.t*20)%20==0:
  telemetry=dict(t=float(sim.t),lot=lots.current().name,known=st['known'],invalid=st['invalid'],reason=st['reason'],
   delta=delta,force=f,support_left=support_left,support_right=support_right,arm_force=arm_force,clear=clear,
   placed_hold=st['placed_hold'],reading_hold=st['reading_hold'],final_hold=st['final_hold'],records=st['records'],
   score=score(states),pos=s.pos.tolist(),machine={k:float(v) for k,v in states['test_stand'].joints.items()})
  try:pathlib.Path(os.environ.get('BEAM_STATUS_PATH',str(pathlib.Path(__file__).with_name('runtime-status.json')))).write_text(json.dumps(telemetry))
  except OSError:pass

 if os.environ.get('TRACE_OUT'):
  row={k:v for k,v in st.items() if k not in ('history','certificate','slip','last_pose')};row.update(lot=lots.current().name,phi=_phi,diag=dict(placed=bool(m['placed']),still=still,solver_rest=bool(s.resting(.002,.02)),lin=float(np.linalg.norm(s.lin_vel)),ang=float(np.linalg.norm(s.ang_vel)),both=bool(both_supports),quiet=bool(quiet),valid=bool(valid)),delta=delta,force=f,support=support,support_left=support_left,support_right=support_right,arm_force=arm_force,clear=clear,pos=s.pos.tolist(),joints={k:float(v) for k,v in s.joints.items()},machine={k:float(v) for k,v in states['test_stand'].joints.items()},score=score(states))
  from yam_arena.constants import TCP_OFFSET
  base=sim.env.unwrapped;origin=parts._np(base.scene.env_origins[0]);row['hands']={}
  for arm in ('left_robot','right_robot'):
   robot=base.scene[arm];poses=parts._np(robot.data.body_link_pose_w)[0];pp=poses[robot.body_names.index('gripper')];row['hands'][arm]=dict(tcp=(pp[:3]-origin+parts._quat_to_matrix(pp[3:])@np.array(TCP_OFFSET)).tolist(),q=parts._np(robot.data.joint_pos)[0].tolist(),effort=arr(robot.root_view.get_dof_projected_joint_forces()).reshape(-1).tolist())
  with pathlib.Path(os.environ['TRACE_OUT']).with_name('beam.jsonl').open('a') as h:h.write(json.dumps(row)+'\n')

def score(objects):
 try:
  st=_state;s=objects['polymer_beam']
  if not st['known'] or st['invalid'] or st['certificate']!=stamp(s):return 0.
  if st['final_hold']>=2.-1e-8:return 1.
  if len(st['records'])==2:return .75
  if len(st['records'])==1:return .5
  return .25 if st['placed_hold']>=1.-1e-8 else 0.
 except (KeyError,ValueError,AttributeError,TypeError):return 0.
def success(objects):return score(objects)==1.

# A demo controller may read only these quantized displayed numbers during the loading stages.
# No joint angle, material parameter, lot id, exact specimen pose or scorer record is read.
_phi=0.;_unload_extra=0
_original_target=None
def knob_target(phi):
 """Tool point for a commanded lever rotation phi (rad) from rest: the declared drawing geometry places the knob
 centre 7 mm inside the pad tips, so the fingertips stop short of the lever rod."""
 x,y,z=lever_point(LEVER_L,phi);return np.array((X+x,y,TOP+z))-.006*knob_rotation(phi)[:,2]
def knob_rotation(phi):
 # Pads close along the knob's arc tangent; the approach follows the lever line so the shaft stays between them.
 # The commanded angle leads the lever by the arm's compliance under load; 0.6 of it keeps the pads near the arc.
 return tilt.rotation(math.pi/2,50.+.6*math.degrees(phi),0.)

def install_feedback():
 global _original_target
 from yam_arena.demo import ArmPlan
 if getattr(ArmPlan,'_beam_feedback',False):return
 _original_target=ArmPlan.target
 def target(self,t):
  global _phi,_unload_extra
  p,r,f,label=_original_target(self,t)
  if label.startswith('read display'):
   goal=float(label.split()[-1]);_unload_extra=0
   if DISPLAY['known']:
    d,F=DISPLAY['delta'],DISPLAY['force']
    if F<1. and d<.3:step=.006 if _phi<.28 else .002
    else:
     e=goal-d;step=0. if abs(e)<.04 else float(np.clip(.006*e,-.004,.004))
    _phi=float(np.clip(_phi+step,0.,.95))
   p,r=knob_target(_phi),knob_rotation(_phi)
  elif label=='unload between readings':
   if DISPLAY['known'] and (DISPLAY['force']>=.5 or DISPLAY['delta']>=.1):_phi=max(0.,_phi-.004)
   elif _unload_extra<15:_phi=max(0.,_phi-.004);_unload_extra+=1
   p,r=knob_target(_phi),knob_rotation(_phi)
  elif label=='unload native lever':
   _phi=max(0.,_phi-.01);p,r=knob_target(_phi),knob_rotation(_phi)
  return p,r,f,label
 ArmPlan.target=target;ArmPlan._beam_feedback=True
install_feedback()


# Each trial starts with the specimen exactly on its declared cradle. A loose articulation moved only by its root
# leaves its sprung centre link behind for a step, so the root and joints are written together, settled, rewritten.
_lots_apply=lots._apply
def _seat_specimen(emb):
 import torch
 base=emb._base;asset=base.scene['polymer_beam'];x,y,z=lots.current().pos('polymer_beam');n=len(asset.joint_names)
 pose=torch.tensor([[x,y,z,0.,0.,0.,1.]],dtype=torch.float32,device=base.device);pose[:,:3]+=base.scene.env_origins[0][:3].to(pose.dtype)
 asset.write_root_pose_to_sim_index(root_pose=pose)
 asset.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1,6),device=base.device))
 asset.write_joint_position_to_sim_index(position=torch.zeros((1,n),device=base.device))
 asset.write_joint_velocity_to_sim_index(velocity=torch.zeros((1,n),device=base.device))
 try:
  from isaaclab_physx.physics import PhysxManager
  PhysxManager.get_physics_sim_view().update_articulations_kinematic()
 except Exception:pass
def _apply_exact(emb):
 base=emb._base
 for _ in range(8):
  _seat_specimen(emb)
  for _ in range(2):emb._env.step(emb._ready)
 _seat_specimen(emb)
 for _ in range(max(1,round(.25/base.step_dt))):emb._env.step(emb._ready)
 for _ in range(lots.RENDERS):base.sim.render()
 return True
lots._apply=_apply_exact

def demo():
 # The trial's declared cradle (far in A, near in B) gives the pickup point; the specimen passes behind the load axis.
 rack=np.asarray(lots.current().pos('polymer_beam'));end=np.array((X,0,BEAM_Z))
 pick=rack+np.array((0,.090,.008));put=end+np.array((0,.090,.010));grip=-.0055
 lane_x=max(float(pick[0]),.20)  # carry lane left of the stand, clear of the arm's own base
 away=np.array((.20,.23,TOP+.15));rightaway=np.array((.25,-.30,TOP+.22))
 def w(t,p,f,label,angle=34.):
  return tilt.Waypoint(t,tuple(float(v) for v in p),f,label,-math.pi/2,tilt=angle,heading=-math.pi/2)
 # The right hand straddles the native knob from the operator side; its pads close along the knob's arc so the
 # press and the unload are normal pad forces. Loading tracks the nominal lever arc using displayed values only.
 K0=knob_target(0.);appr=knob_rotation(0.)[:,2];pre=K0-.05*appr;knob_grip=-.010
 def rw(t,p,f,label):return tilt.Waypoint(t,tuple(float(v) for v in p),f,label,math.pi/2,tilt=50.,heading=0.)
 loading=[rw(3.,pre,FINGER_OPEN,'reach behind native knob'),rw(2.,K0,FINGER_OPEN,'straddle native knob'),
  rw(1.5,K0,knob_grip,'close pads on native knob'),rw(12.,K0,knob_grip,'read display 1'),
  rw(4.,K0,knob_grip,'unload between readings'),rw(12.,K0,knob_grip,'read display 2'),
  rw(5.,K0,knob_grip,'unload native lever'),rw(1.,K0,knob_grip,'lever back at rest'),
  rw(1.5,K0,FINGER_OPEN,'release native knob'),rw(2.,pre,FINGER_OPEN,'withdraw from native knob')]
 left=[w(2,pick+(0,0,.07),FINGER_OPEN,'above specimen',30.),w(2,pick,FINGER_OPEN,'pinch on cradle',30.),w(1,pick,grip,'grasp25mm body',30.),
  w(3,(pick[0],pick[1],put[2]+.003),grip,'lift'),w(3,(lane_x,.290,put[2]+.003),grip,'carry clear of stand to left'),
  w(5,(X+.045,.290,put[2]+.003),grip,'carry beside full stand'),w(5,(X+.045,put[1],put[2]+.003),grip,'align behind load axis'),
  w(3,put+(0,0,.003),grip,'slide forward under retracted nose'),w(1,put+(0,0,.003),FINGER_OPEN,'release specimen'),
  w(2,away,FINGER_OPEN,'clear specimen'),w(2,away,FINGER_OPEN,'settle on native supports')]
 start=sum(p.duration for p in left)
 right=[Waypoint(start,tuple(rightaway),FINGER_OPEN,'wait clear')]+loading
 left+=[w(sum(p.duration for p in loading)+2,away,FINGER_OPEN,'wait away')]
 right +=[Waypoint(2.,tuple(rightaway),FINGER_OPEN,'clear lever')]
 left +=[w(2,put+(0,0,.003),FINGER_OPEN,'approach unloaded specimen'),w(2,put,FINGER_OPEN,'full height pinch',30.),
  w(1,put,grip,'grasp unloaded specimen',30.),w(2,put+(0,0,.003),grip,'lift from rollers'),
  w(3,(X+.045,put[1],put[2]+.003),grip,'withdraw behind nose'),w(4,(X+.045,.290,put[2]+.003),grip,'clear column to left'),w(4,(lane_x,.290,put[2]+.003),grip,'carry back along cradle row'),w(3,pick+(0,0,.07),grip,'return to cradle'),
  w(2,pick+(0,0,.006),grip,'lower to cradle'),w(1,pick+(0,0,.006),FINGER_OPEN,'release'),w(2,away,FINGER_OPEN,'hands clear'),w(3,away,FINGER_OPEN,'released still hold')]
 return dict(left=left,right=right)
