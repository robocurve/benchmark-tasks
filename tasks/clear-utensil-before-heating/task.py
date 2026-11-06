"""Station scene with a wooden spatula, a pan and an electric hotplate with a rotary control."""
import os,pathlib,sys,numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).parent))
from yam_arena.task_file import TABLE_TOP,Cylinder
from yam_arena.demo import Waypoint,pick_place_waypoints
from yam_arena.constants import FINGER_OPEN,FINGER_CLOSED
from scipy.spatial.transform import Rotation
import scenekit,geometry,monitor,scoring,handgeom,steptime,meshclearance,cooked_support
from scenekit import UsdBody,on_step,on_reset
geometry.build()
SCENE='station';TOP=TABLE_TOP[SCENE];SCORE_OVER='end';SUCCESS_HOLD_S=1.
CENTER=monitor.CENTER;KNOB_XY=monitor.KNOB_XY;PAN_XY=(.36,-.27)
PAN_TARGET=(CENTER[0]+.01131370849898476,CENTER[1]-.01131370849898476)
BURNER_Z=TOP+geometry.BURNER_TOP
RADIUS=geometry.BURNER_RADIUS
WOOD_YAW=-np.pi/4
WOOD_ROT=tuple(float(x) for x in (Rotation.from_euler('z',-45,degrees=True)*Rotation.from_euler('y',-3.684740314612854,degrees=True)).as_quat())
WOOD_POS=(.20137253836959798,.15862746163040203,.7548288631872321)
if os.environ.get('TASK_SAFE_INITIAL')=='1':WOOD_POS=(.23,.42,TOP+.001);WOOD_ROT=tuple(float(x) for x in Rotation.from_euler('z',-45,degrees=True).as_quat())
OBJECTS=[UsdBody('wood',pos=WOOD_POS,rot_xyzw=WOOD_ROT,usd='spatula.usda',size=(.2963,.0692,.0164),mass=.05154,friction=(.8,.6),torsional_patch=.008,solver_iters=32),
 UsdBody('pan',pos=(*PAN_XY,TOP+.002),rot_xyzw=(0.,0.,-.7071067811865476,.7071067811865476),usd='pan.usda',size=(.16,.284,.06),mass=.4,friction=(.9,.8),torsional_patch=.012,solver_iters=32),
 UsdBody('hotplate',pos=(*CENTER,TOP),usd='hotplate.usda',size=(.36,.25,geometry.BURNER_TOP)),
 UsdBody('knob',pos=(*KNOB_XY,TOP+.004),usd='knob.usda',size=(.078,.078,.041),articulation=True,joint_pos=(('turn',0.),),joint_damping=.015),
 Cylinder('indicator',radius=.007,height=.002,pos=(float(KNOB_XY[0]-.035),float(KNOB_XY[1]-.035),TOP+.005),color=(.12,.015,.01))]
V=monitor.V;F=monitor.F;H=monitor.H

def surfaces(objects):
 wood=objects['wood'];pan=objects['pan'];knob=objects['knob'];q=knob.joints['turn']
 w=V@wood.rotation().T+wood.pos;p=geometry.PAN_TRIANGLES@pan.rotation().T+pan.pos
 c,s=np.cos(-q),np.sin(-q);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
 k=np.concatenate([geometry.KNOB_BASE_TRIANGLES,geometry.KNOB_MOVING_TRIANGLES@r.T])+np.array([*KNOB_XY,TOP+.004])
 return w,w[F],p,k

def other_support_clear(objects,wt,pt,kt):
 pan=objects['pan'];q=objects['knob'].joints['turn'];c,s=np.cos(-q),np.sin(-q);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
 woodpoint=[wt[0,0]]
 supports=[(pt,geometry.PAN_COMPONENT_POINTS@pan.rotation().T+pan.pos),
  (geometry.HOTPLATE_TRIANGLES+np.r_[CENTER,TOP],geometry.HOTPLATE_COMPONENT_POINTS+np.r_[CENTER,TOP]),
  (kt,np.concatenate([geometry.KNOB_BASE_COMPONENT_POINTS,geometry.KNOB_MOVING_COMPONENT_POINTS@r.T])+np.r_[KNOB_XY,TOP+.004])]
 return all(meshclearance.surfaces_separated(wt,surface,.002,woodpoint,points) for surface,points in supports)

_condition_key=None;_condition_hands=None;_condition_result=None

def conditions(objects):
 global _condition_key,_condition_hands,_condition_result
 hands=handgeom._hand_surfaces()
 key=b''.join(np.r_[objects[name].pos,objects[name].quat_xyzw,objects[name].lin_vel,objects[name].ang_vel].tobytes() for name in ('wood','pan'))+np.array([objects['knob'].joints['turn']]).tobytes()
 if _condition_hands is hands and _condition_key==key:return _condition_result
 result=_condition_values(objects,hands)
 _condition_key=key;_condition_hands=hands;_condition_result=result
 return result

def _condition_values(objects,hands):
 wood=objects['wood'];pan=objects['pan'];w,wt,pt,kt=surfaces(objects)
 panpoints=geometry.PAN_COMPONENT_POINTS@pan.rotation().T+pan.pos
 q=objects['knob'].joints['turn'];c,s=np.cos(-q),np.sin(-q);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
 knobpoints=np.concatenate([geometry.KNOB_BASE_COMPONENT_POINTS,geometry.KNOB_MOVING_COMPONENT_POINTS@r.T])+np.r_[KNOB_XY,TOP+.004]
 low=w.min(0);high=w.max(0);support_min=(cooked_support.vertices()@wood.rotation().T+wood.pos)[:,2].min();velocity=wood.lin_vel+np.cross(wood.ang_vel,w-wood.pos)
 # Natural support is judged against the original solid worktable, no marked parking rectangle.
 cold=(low[0]>=-.19 and high[0]<=.91 and low[1]>=-.65 and high[1]<=.65
   and TOP-.00005<=support_min<=TOP+.002 and wood.rotation()[2,2]>=np.cos(np.radians(5))
   and scoring.surface_disk_distance(wt,CENTER)>=RADIUS+.05 and np.linalg.norm(velocity,axis=1).max()<=.01
   and handgeom._clear(hands,[wt]) and other_support_clear(objects,wt,pt,kt))
 stablepan=(np.linalg.norm(pan.pos[:2]-CENTER)<=.02 and pan.rotation()[2,2]>=np.cos(np.radians(5))
   and -.00005<=pan.pos[2]-BURNER_Z<=.005 and np.linalg.norm(pan.lin_vel)<=.01 and np.linalg.norm(pan.ang_vel)<=np.radians(5)
   and handgeom._clear(hands,[pt],component_points=[panpoints]))
 on=monitor.is_energized(q)
 # Wood/pan predicates already include hand clearance. The complete goal
 # needs the knob clear as well; do not repeat all surface checks or evaluate
 # this completion-only condition while either placement is invalid.
 clear=cold and stablepan and handgeom._clear(hands,[kt],component_points=[knobpoints])
 return bool(cold),bool(stablepan),bool(on),bool(clear)

def success(objects):
 steptime.tick()
 if monitor.monitor_error is not None:raise RuntimeError(monitor.monitor_error)
 wood,pan,on,clear=conditions(objects)
 monitor.trial.wood_s=monitor.trial.wood_s+.05 if wood else 0.
 monitor.trial.pan_s=monitor.trial.pan_s+.05 if pan else 0.
 return not monitor.trial.failed and wood and pan and on and clear

def score(objects):
 if monitor.trial.failed:return 0.
 wood,pan,_,_=conditions(objects)
 return .5 if wood and pan and monitor.trial.wood_s>=1-1e-8 and monitor.trial.pan_s>=1-1e-8 else (.25 if wood and monitor.trial.wood_s>=1-1e-8 else 0.)

@on_reset
def reset(environment):
 path=os.environ.get('TASK_STEP_FILE')
 if path and pathlib.Path(path+'.worst30').exists():print('PREVIOUS_TRIAL_WORST30_S',pathlib.Path(path+'.worst30').read_text().strip(),flush=True)
 cooked_support.prepare(environment);handgeom.prepare(environment);steptime.reset()

@on_step
def diagnose(environment):
 base=environment.unwrapped
 if base.common_step_counter%20 and not (os.environ.get('TASK_DEMO')=='closed_low' and base.common_step_counter>=620 and base.common_step_counter%2==0):return
 from yam_arena.scene import object_states
 o=object_states(environment,['wood','pan','knob']);w,_,_,_=surfaces(o)
 print('STATE',base.common_step_counter,'wood_pose',o['wood'].pos.tolist(),'wood_quat',o['wood'].quat_xyzw.tolist(),'wood_edge_penetration_mm',float((RADIUS-scoring.surface_disk_distance(w[F],CENTER))*1000),'wood_bounds',w.min(0).tolist(),w.max(0).tolist(),'pan_pose',o['pan'].pos.tolist(),'pan_quat',o['pan'].quat_xyzw.tolist(),'pan_centre_error_mm',float(np.linalg.norm(o['pan'].pos[:2]-CENTER)*1000),'pan_linear_speed_mm_s',float(np.linalg.norm(o['pan'].lin_vel)*1000),'pan_angular_speed_deg_s',float(np.degrees(np.linalg.norm(o['pan'].ang_vel))),'pan_height_gap_mm',float((o['pan'].pos[2]-BURNER_Z)*1000),'pan_tilt_deg',float(np.degrees(np.arccos(np.clip(o['pan'].rotation()[2,2],-1,1)))),'knob_deg',np.degrees(o['knob'].joints['turn']),'conditions',conditions(o),'failed',monitor.trial.failed,monitor.trial.reason,'support_gap_mm',float(((cooked_support.vertices()@o['wood'].rotation().T+o['wood'].pos)[:,2].min()-TOP)*1000),'wood_heat_contact',scoring.mesh_touches_contact_disk(w[F],CENTER,RADIUS,BURNER_Z,.002),'physics_samples',monitor.trial.physics_steps,'completed',monitor.completed_physics_samples,'monitor_error',monitor.monitor_error,'gripper_apertures_m',handgeom.apertures(),flush=True)

def turn_on():
 angle=45 if os.environ.get('TASK_DEMO')=='closed_low' else 75
 end=FINGER_CLOSED if os.environ.get('TASK_DEMO')=='closed_low' else FINGER_OPEN
 z=TOP+.004+.036
 return [Waypoint(1.5,(*KNOB_XY,TOP+.13),FINGER_OPEN),Waypoint(1.5,(*KNOB_XY,z),FINGER_OPEN),Waypoint(1,(*KNOB_XY,z),FINGER_CLOSED),Waypoint(2,(*KNOB_XY,z),FINGER_CLOSED,yaw=-np.radians(angle)),Waypoint(1,(*KNOB_XY,z),FINGER_OPEN,yaw=-np.radians(angle)),Waypoint(.6,(*KNOB_XY,TOP+.12),FINGER_OPEN,yaw=-np.radians(angle)),Waypoint(.6,(.13,.08,TOP+.17),end,yaw=-np.radians(angle))]

def demo():
 yaw=WOOD_YAW;pick=(.14844916121319057,.21155083878680944,TOP+.018);place=(.17696699141100894,.47303300858899106,TOP+.018)
 left=[Waypoint(1.5,(pick[0],pick[1],TOP+.12),FINGER_OPEN,yaw=yaw),Waypoint(1.5,pick,FINGER_OPEN,yaw=yaw),Waypoint(.8,pick,FINGER_CLOSED,yaw=yaw),Waypoint(1.5,(pick[0],pick[1],TOP+.12),FINGER_CLOSED,yaw=yaw),Waypoint(1,(pick[0],pick[1],TOP+.12),FINGER_CLOSED,yaw=yaw),Waypoint(2,(place[0],place[1],TOP+.12),FINGER_CLOSED,yaw=yaw),Waypoint(1.5,place,FINGER_CLOSED,yaw=yaw),Waypoint(.8,place,FINGER_OPEN,yaw=yaw),Waypoint(1.5,(place[0],place[1],TOP+.12),FINGER_OPEN,yaw=yaw),Waypoint(2.6,(.13,.10,TOP+.15),FINGER_OPEN,yaw=0.)]
 if os.environ.get('TASK_SAFE_INITIAL')=='1' or os.environ.get('TASK_LEAVE_WOOD')=='1':left=[Waypoint(14.7,(.13,.10,TOP+.15),FINGER_OPEN)]
 closed_low=os.environ.get('TASK_DEMO')=='closed_low'
 end=FINGER_CLOSED if closed_low else FINGER_OPEN
 left += [Waypoint(12,(.13,.10,TOP+.15),FINGER_OPEN)]+turn_on()+[Waypoint(3,(.13,.08,TOP+.17),end,yaw=-np.radians(45 if closed_low else 75))]
 pickpan=(PAN_XY[0]-.151,PAN_XY[1],TOP+.055);placepan=(PAN_TARGET[0]+.009,PAN_TARGET[1]+.010-.151,BURNER_Z+.055)
 right=[Waypoint(14.7,(.20,-.35,TOP+.12),FINGER_OPEN)]+pick_place_waypoints(pickpan,placepan,travel_z=TOP+.12,yaw=0.,place_yaw=np.pi/2)+[Waypoint(2,(.20,-.40,TOP+.13),FINGER_OPEN),Waypoint(1,(.20,-.40,TOP+.13),end),Waypoint(14,(.20,-.40,TOP+.13),end)]
 if os.environ.get('TASK_DEMO')=='failure':
  z=TOP+.004+.036
  off=[Waypoint(1.5,(*KNOB_XY,TOP+.13),FINGER_OPEN,yaw=-np.radians(75)),Waypoint(1.5,(*KNOB_XY,z),FINGER_OPEN,yaw=-np.radians(75)),Waypoint(1,(*KNOB_XY,z),FINGER_CLOSED,yaw=-np.radians(75)),Waypoint(2,(*KNOB_XY,z),FINGER_CLOSED,yaw=0.),Waypoint(1,(*KNOB_XY,z),FINGER_OPEN),Waypoint(1.5,(*KNOB_XY,TOP+.13),FINGER_OPEN)]
  prefix=turn_on()+off
  left=prefix+left
  right=[Waypoint(sum(w.duration for w in prefix),(.20,-.35,TOP+.12),FINGER_OPEN)]+right
 return {'left':left,'right':right}
