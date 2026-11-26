"""Thread a household key onto a split spring key ring."""
from pathlib import Path
import sys,math
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
from scipy.spatial.transform import Rotation
import geometry as G
from scenekit import UsdBody,body_link_poses,on_reset,on_step
SCENE='station'
SCORE_OVER='end'
SUCCESS_HOLD_S=2.
INSTRUCTION=('Thread the brass house key fully onto the steel split ring. Use the orange spring-return splitter to open the end if needed, feed the key around the spiral, then release the key and splitter. Keep both grippers clear of the key and ring; you may keep holding the crank. The key must hang retained on both overlapping turns with the splitter released.')
OBJECTS=[
 UsdBody('ring',usd='assets/ring.usda',pos=tuple(G.CENTER),rot_xyzw=tuple(Rotation.from_matrix(G.W).as_quat()),size=(.15,.15,.23),articulation=True,joint_targets=(('retract_slide',.080),),
  joint_stiffness={'flex.*':G.SPRING_K,'splitter_slide':35.,'retract_slide':5.,'turn|knob_spin.*':0.},joint_damping={'flex.*':G.SPRING_DAMPING,'splitter_slide':1.,'retract_slide':1.,'turn':G.TURN_DAMPING,'knob_spin.*':.000002},joint_friction=0.,solver_iters=64,solver_velocity_iters=8,friction=(.25,.18)),
 UsdBody('key',usd='assets/key.usda',pos=tuple(G.KEY_START),rot_xyzw=tuple(Rotation.from_matrix(G.key_rotation(G.KEY_START_ANGLE)).as_quat()),size=(.049,.002,.018),mass=G.KEY_MASS,friction=(.8,.65),torsional_patch=.004,solver_iters=32),
 UsdBody('stand',usd='assets/stand.usda',pos=tuple(G.KEY_START),size=(.08,.06,.24),friction=(.6,.45))]
STATE={}
_passage=None
_baseline=None

HOLE_NORMALS=np.stack([np.cos((np.arange(32)+.5)*2*math.pi/32),np.sin((np.arange(32)+.5)*2*math.pi/32)],1)
# A released rim contact can have small solver overlap. The wire and brass
# each contribute one contact offset to the allowed final fit. The separately
# tracked tip traversal keeps its tighter tolerance.
RIM_CONTACT_TOLERANCE=2*G.CONTACT_OFFSET

def capsule_slab_support(a,b,half,radius):
 """Exact capsule support against the 32 authored hole-wall halfspaces.

 Inside the plate slab a linear projection has its maximum at an endpoint.
 Outside a face, write the centre distance as radius*x. Each wall support is
 radius*(intercept+slope*x+sqrt(1-x*x)), whose maximum is at the clipped
 stationary point x=slope/sqrt(1+slope*slope). No inscribed-circle or
 oblique-radius upper bound is used to decide whether the capsule fits.
 """
 a=np.asarray(a,float);b=np.asarray(b,float);d=b-a;values=[]
 if abs(d[1])<1e-12:
  gap=max(abs(a[1])-half,0.)
  return max(np.max(HOLE_NORMALS@a[[0,2]]),np.max(HOLE_NORMALS@b[[0,2]]))+math.sqrt(max(0.,radius**2-gap**2)) if gap<=radius else -float('inf')
 lo,hi=sorted(((-half-a[1])/d[1],(half-a[1])/d[1]));lo=max(0.,lo);hi=min(1.,hi)
 if lo<=hi:
  values += [np.max(HOLE_NORMALS@(a+t*d)[[0,2]])+radius for t in [lo,hi]]
 for sign in [-1.,1.]:
  ylo,yhi=sorted((sign*a[1],sign*b[1]));xlo=max(0.,(ylo-half)/radius);xhi=min(1.,(yhi-half)/radius)
  if xlo>xhi:continue
  v=(a+((sign*half-a[1])/d[1])*d)[[0,2]]/radius;k=sign*d[[0,2]]/d[1]
  intercept=HOLE_NORMALS@v;slope=HOLE_NORMALS@k;x=np.clip(slope/np.sqrt(1+slope*slope),xlo,xhi)
  values.append(radius*np.max(intercept+slope*x+np.sqrt(np.maximum(0,1-x*x))))
 return max(values,default=-float('inf'))


def crossings(segments,key_pos,key_rot):
 """Full-thickness crossings of the curved capsule centreline.

 Every connected capsule portion through the plate must fit the hole.
 Exact radial support includes the sphere caps, both faces, and every curved
 span between them; a tip ending within the plate is not a through crossing.
 """
 local=[(key_rot.T@(a-key_pos),key_rot.T@(b-key_pos))for a,b in segments]
 half=G.KEY_THICKNESS/2;expanded=half+G.WIRE_R;hole=G.HOLE_R*math.cos(math.pi/32)
 material=G.span_angles() if len(segments)==len(G.span_angles()) else None
 clipped={}
 def slab_piece(j):
  if j in clipped:return clipped[j]
  a,b=local[j];d=b-a
  if abs(d[1])<1e-12:
   part=(a,b)if abs(a[1])<=expanded else None
  else:
   t0,t1=sorted(((-expanded-a[1])/d[1],(expanded-a[1])/d[1]));lo=max(0.,t0);hi=min(1.,t1)
   part=(a+lo*d,a+hi*d)if lo<=hi else None
  clipped[j]=part;return part
 def slab_clearance(i):
  lo=hi=i
  while lo>0 and abs(local[lo][0][1])<=expanded+1e-8 and slab_piece(lo-1)is not None:lo-=1
  while hi+1<len(local)and abs(local[hi][1][1])<=expanded+1e-8 and slab_piece(hi+1)is not None:hi+=1
  points=np.array([p for j in range(lo,hi+1)for p in slab_piece(j)])
  if points[:,1].min()>-half or points[:,1].max()<half:return None
  return hole-max(capsule_slab_support(local[j][0],local[j][1],half,G.WIRE_R)for j in range(lo,hi+1))
 raw=[]
 for i,(a,b) in enumerate(local):
  d=b-a;length=float(np.linalg.norm(d))
  if length<1e-12 or abs(d[1])<1e-10:continue
  u=-a[1]/d[1]
  if not -1e-8<=u<=1+1e-8:continue
  u=float(np.clip(u,0,1));p=a+u*d;rr=float(np.linalg.norm(p[[0,2]]));cosine=abs(d[1])/length
  clearance=hole-rr-G.WIRE_R/cosine
  # Diagnostic radial upper bound only; actual polygon/slab support decides.
  slab=slab_clearance(i)
  if slab is None or slab < -RIM_CONTACT_TOLERANCE:continue
  arc=(1-u)*material[i,0]+u*material[i,1] if material is not None else (i+u)*G.END_ANGLE/len(segments)
  raw.append({'segment':i,'u':u,'arc_rad':float(arc),
   'radius_m':rr,'tube_clearance_m':float(clearance),'slab_clearance_m':float(slab),
   'sign':1 if d[1]>0 else -1,'point_local_m':p.tolist()})
 groups=[]
 for h in raw:
  if groups and h['segment']-groups[-1][-1]['segment']<=1 and np.linalg.norm(np.array(h['point_local_m'])-groups[-1][-1]['point_local_m'])<.0001:groups[-1].append(h)
  else:groups.append([h])
 out=[]
 for group in groups:
  if len({h['sign']for h in group})==1:out.append(max(group,key=lambda h:h['slab_clearance_m']))
 return out

class TipPassage:
 """Observable signed free-tip passage; reverse passages cancel earlier passes.

 A marked end must travel through both key faces. The same end may pass twice. The swept tube must fit
 the actual 32-sided aperture throughout that traversal. Mid-wire penetration
 cannot create a passage. This reads geometry only for scoring/diagnostics.
 """
 def __init__(self):
  self.previous=None;self.entry=[None,None];self.last_side=[None,None]
  self.events=[0,0];self.net=[0,0];self.history=[]
 def update(self,local_tips):
  points=np.asarray(local_tips,float);half=G.KEY_THICKNESS/2;face=half+G.WIRE_R
  hole=G.HOLE_R*math.cos(math.pi/32)
  for i,p in enumerate(points):
   side=1 if p[1]>=face else -1 if p[1]<=-face else None
   if self.previous is None:
    self.last_side[i]=side;continue
   old=self.previous[i]
   intersects=min(old[1],p[1])<=face and max(old[1],p[1])>=-face
   fits=capsule_slab_support(old,p,half,G.WIRE_R)<=hole+.00005
   if intersects:
    if not fits:self.entry[i]=None;self.last_side[i]=None
    elif self.entry[i] is None:self.entry[i]=self.last_side[i]
   if side is not None:
    if self.entry[i] is not None and side!=self.entry[i] and fits:
     self.events[i]+=1;self.net[i]+=side
     self.history.append({'tip':'green' if i==0 else 'red','passage_number':self.events[i],'exit_side':side})
    self.entry[i]=None;self.last_side[i]=side
  self.previous=points.copy()
  return list(self.net)

def visible_measurements(local_points,endpoints):
 """Front silhouette caliper diameters, edge tip clearance and visible breaks."""
 points=np.asarray(local_points)
 diameters=np.ptp(points[:,:2],axis=0)+2*G.WIRE_R
 # Match each free end to the material point one full turn away, then subtract
 # two wire radii. The paired points are marked in the inspection overlays.
 spans=G.span_angles();surface_gaps=[]
 for tip,angle in [(points[0],2*math.pi),(points[-1],G.END_ANGLE-2*math.pi)]:
  j=next(j for j,(a,b) in enumerate(spans) if a-1e-10<=angle<=b+1e-10)
  a,b=spans[j];u=float(np.clip((angle-a)/(b-a),0,1));other=(1-u)*points[2*j]+u*points[2*j+1]
  surface_gaps.append(max(0.,float(np.linalg.norm(tip-other)-2*G.WIRE_R)))
 breaks=[max(0.,float(np.linalg.norm(a[1]-b[0])-2*G.WIRE_R)) for a,b in zip(endpoints[:-1],endpoints[1:])]
 return {'diameters_m':diameters.tolist(),'roundness_m':float(abs(diameters[0]-diameters[1])),
  'tip_surface_gaps_m':surface_gaps,'visible_break_m':max(breaks,default=0.)}

def visible_integrity(measurement,baseline):
 change=np.abs(np.asarray(measurement['diameters_m'])-np.asarray(baseline['diameters_m']))
 round_change=abs(measurement['roundness_m']-baseline['roundness_m'])
 return bool(np.isfinite(change).all() and max(change)<=G.MAX_DIAMETER_CHANGE
  and round_change<=G.MAX_ROUNDNESS_CHANGE and max(measurement['tip_surface_gaps_m'])<=G.MAX_RESIDUAL_TIP_GAP
  and measurement['visible_break_m']<=G.MAX_VISIBLE_BREAK)

def evaluate(hits,end_distance,speed,spin,clear,closed,integrity,key_height,splitter,ring_spin=0.,passages=(0,0),inspection_ready=False):
 count=len(hits);separation=abs(hits[1]['arc_rad']-hits[0]['arc_rad']) if count==2 else 0.
 same=count==2 and hits[0]['sign']==hits[1]['sign'] and math.radians(290)<=separation<=math.radians(430)
 winding=int(passages[1]-passages[0])
 qualified=bool(count in (1,2) and abs(winding)==count and all(h["sign"]==(1 if winding>0 else -1) for h in hits))
 inspected=bool(inspection_ready and clear and closed and splitter<.0007 and abs(ring_spin)<=.03 and speed<=.015 and spin<=.35)
 full=bool(same and qualified and inspected and end_distance>=.012 and speed<=.015 and spin<=.35 and clear and closed and integrity and key_height>G.TABLE_Z+.10 and splitter<.0007 and abs(ring_spin)<=.03)
 partial=(.75 if same and end_distance>=.012 else .5 if count==1 and end_distance>=.010 else .25 if count==1 else 0.) if integrity and qualified and inspected else 0.
 return {'crossings':hits,'captured_strands':count,'two_turns':same,'strand_material_separation_deg':math.degrees(separation),'free_end_clearance_m':float(end_distance),'key_speed_m_s':float(speed),'key_spin_rad_s':float(spin),'ring_spin_rad_s':float(ring_spin),'grippers_clear':bool(clear),'ring_closed':bool(closed),'integrity':bool(integrity),'complete':full,'partial_score':partial,
  'tip_passage_net':list(passages),'passage_winding':winding,'passage_qualified':bool(qualified),'unloaded_inspection_ready':inspected}

@on_reset
def reset_monitor(env):
 global _passage,_baseline
 STATE.clear();_passage=TipPassage();_baseline=None

@on_step
def monitor(env):
 global _passage,_baseline
 poses=body_link_poses(env,'ring');kp=body_link_poses(env,'key')['key'];kr=G.rotation(kp[3:]);pts=G.ring_points();segments=[];gaps=[];endpoints=[]
 rotor=poses['rotor'];rr=G.rotation(rotor[3:])
 for i in range(G.N):
  p=poses[f'wire{i:02d}'];r=G.rotation(p[3:]);aa,bb=(pts[i+1],pts[i]) if i<G.CLAMP_INDEX else (pts[i],pts[i+1]);xx=(bb-aa)/np.linalg.norm(bb-aa);yy=np.cross([0,0,1.],xx);yy/=np.linalg.norm(yy);authored_r=np.stack([xx,yy,np.cross(xx,yy)],1)
  curve=(G.segment_curve(i)-aa)@authored_r@r.T+p[:3]
  endpoints.append((curve[0],curve[-1]))
  segments.extend(list(zip(curve[:-1],curve[1:])))
 for i in range(G.N-1):gaps.append(float(np.linalg.norm(endpoints[i][1]-endpoints[i+1][0])))
 gaps.append(float(np.linalg.norm(endpoints[G.CLAMP_INDEX][0]-(rotor[:3]+rr@pts[G.CLAMP_INDEX]))))
 ring=env.unwrapped.scene['ring'];q=ring.data.joint_pos[0].detach().cpu().numpy();dq=ring.data.joint_vel[0].detach().cpu().numpy()
 slider=float(q[ring.joint_names.index('splitter_slide')]);retract=float(q[ring.joint_names.index('retract_slide')]);flex=[float(q[j]) for j,n in enumerate(ring.joint_names) if n.startswith('flex')]
 local_points=(np.array([point for span in segments for point in span])-rotor[:3])@rr
 radii=np.linalg.norm(local_points[:,:2],axis=1)
 key=env.unwrapped.scene['key'];vel=key.data.root_link_vel_w[0].detach().cpu().numpy()
 finite=bool(np.isfinite(kp).all() and np.isfinite(q).all() and np.isfinite(dq).all() and np.isfinite(vel).all())
 measurement=visible_measurements(local_points,endpoints)
 if _baseline is None:_baseline=measurement.copy()
 integrity=finite and visible_integrity(measurement,_baseline)
 if _passage is None:_passage=TipPassage()
 local_tips=[kr.T@(segments[0][0]-kp[:3]),kr.T@(segments[-1][1]-kp[:3])]
 passages=_passage.update(local_tips)
 hits=crossings(segments,kp[:3],kr)
 end_distance=min(np.linalg.norm(kp[:3]-segments[0][0]),np.linalg.norm(kp[:3]-segments[-1][1]))
 ring_span=float(np.ptp(local_points[:,2]))
 closed=max(measurement['tip_surface_gaps_m'])<=G.MAX_RESIDUAL_TIP_GAP
 speed=np.linalg.norm(vel[:3]);spin=np.linalg.norm(vel[3:])
 tcp={}
 for side in ('left','right'):
  gp=body_link_poses(env,f'{side}_robot')['gripper'];tcp[side]=gp[:3]+G.rotation(gp[3:])@np.array([0,0,.134])
 # Conservative bounds cover every authored gripper and finger collision mesh.
 bounds={'gripper':(np.array([-.05101,-.03251,-.01051]),np.array([.05101,.03251,.06351])),
  'tip_left':(np.array([-.01002,-.06830,-.09018]),np.array([.05801,.00330,.00370])),
  'tip_right':(np.array([-.05801,-.00330,-.09018]),np.array([.01002,.06830,.00370]))}
 key_ball=kp[:3]+kr@np.array([.018,0,0]);ring_radius=float(np.linalg.norm(local_points,axis=1).max()+G.WIRE_R);hand_key=[];hand_ring=[]
 for side in ('left','right'):
  for name,p in body_link_poses(env,f'{side}_robot').items():
   if name not in bounds:continue
   lo,hi=bounds[name];r=G.rotation(p[3:])
   for point,radius,out in [(key_ball,.027,hand_key),(rotor[:3],ring_radius,hand_ring)]:
    local=r.T@(point-p[:3]);delta=np.maximum(np.maximum(lo-local,local-hi),0.)
    out.append(float(np.linalg.norm(delta)-radius))
 clear=min(hand_key)>=G.MIN_HAND_CLEARANCE and min(hand_ring)>=G.MIN_HAND_CLEARANCE
 inspection=bool(clear and slider<.0007 and retract>-.005 and abs(float(dq[ring.joint_names.index('turn')]))<=.03 and speed<=.015 and spin<=.35)
 STATE.update(evaluate(hits,end_distance,speed,spin,clear,closed,integrity,kp[2],slider,float(dq[ring.joint_names.index('turn')]),passages,inspection))
 STATE.update(ring_bound_radius_m=ring_radius,hand_key_clearance_m=min(hand_key),hand_ring_clearance_m=min(hand_ring),finite_physics=finite,key_world_m=kp[:3].tolist(),key_quat_xyzw=kp[3:].tolist(),key_grip_world_m=(kp[:3]+kr@G.GRIP).tolist(),splitter_m=slider,splitter_retract_m=retract,
  visible_measurements=measurement,visible_baseline=_baseline,tip_passage_events=list(_passage.events),tip_passage_history=list(_passage.history),marked_tip_key_local_m=[p.tolist() for p in local_tips],
  max_joint_gap_m=max(gaps),max_joint_angle_deg=float(np.degrees(max(abs(np.array(flex))))),ring_vertical_span_m=ring_span,
  turn_deg=float(np.degrees(q[ring.joint_names.index('turn')])),knob_world_m=poses['knob'][:3].tolist(),
  tip_world_m=segments[-1][1].tolist(),tcp_world_m={k:p.tolist() for k,p in tcp.items()},flex_joint_radians=flex)

def success(objects):return bool(STATE.get('complete',False))
def score(objects):return float(STATE.get('partial_score',0.))
