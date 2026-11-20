"""Outcome score from actual poses, contacts, material state and robot links; never reads reference phases or commands."""
import numpy as np
from scipy.spatial.transform import Rotation

class PrimaryScore:
 DT=.05   # control step (s)
 def __init__(self):self.reset()
 def reset(self):self.last=-1;self.setting_witness=None;self.withdraw_streak=0;self.value=0.;self.latest={};self.release_streak=0;self.prev={};self.prev_down=None;self.hammered=0.
 def observe(self,row,G):
  control=row['control']
  if not isinstance(control,int)or control<=self.last:raise ValueError('Fresh unique actualscoretime required')
  if self.last>=0 and control!=self.last+1:raise ValueError('Actual scoregap')
  self.last=control;parts={p['name']:p for p in G['parts']};b=row['rigid_bodies'];pos={};R={}
  for n,d in b.items():
   a=np.array(d['pose_xyzw'],float)
   if a.shape!=(7,)or not np.isfinite(a).all()or np.linalg.norm(a[3:])<1e-8:raise ValueError('Invalidactualscorebody')
   pos[n]=a[:3];R[n]=Rotation.from_quat(a[3:]).as_matrix()
  origin=np.array(G['frame_origin_m']);axis=R['anchor_ring'][:,2];top=pos['anchor_ring']+axis*.0396875
  shoulder=pos['punch']+R['punch'][:,2]*.0414875
  gap=float(np.dot(shoulder-top,axis));lateral=float(np.linalg.norm((shoulder-top)-gap*axis))
  align=float(np.arccos(np.clip(np.dot(axis,R['punch'][:,2]),-1,1)))
  anchor_shift=np.linalg.norm(pos['anchor_ring']-origin);anchor_upright=float(np.arccos(np.clip(axis[2],-1,1)))
  wedge=(R['anchor_ring'].T@(pos['setting_wedge']-pos['anchor_ring']))[2]
  # Bodies use full common modelorigin; actualconegeometry startsz5mm.
  actual_down=-float(wedge)
  # Plug travel counts as hammer-driven when it happens in a control step in which the hammer bore on the punch
  # with 10 N or more (largest contact force over that step's physics steps).
  peak=row.get('hammer_punch_peak_N')
  if self.prev_down is not None and peak is not None and peak>=10.:self.hammered+=max(0.,actual_down-self.prev_down)
  self.prev_down=actual_down
  contacts=row.get('contacts',{});pairs=contacts.get('pairs',{});unavailable=contacts.get('unavailable_views',{})
  wall=[]
  for k in range(4):
   c=pairs.get(f'petal{k}_concrete')
   wall.append(None if c is None or'unknown'in c else int(np.array(c['counts']).sum())>0)
  current_anchor_seated=anchor_shift<.0008 and anchor_upright<np.deg2rad(2)
  current_material_good=all(np.isfinite(x.get('plastic_angle',np.nan))for x in row.get('material',{}).values())and len(row.get('material',{}))==4
  # Visible shoulderflush is the primary settingwitness; actual wedge/material/wall
  # readback prevent awarding a geometric proxy to a physically unset/broken mechanism.
  witness=(current_anchor_seated and-.0002<=gap<=.00015 and lateral<.0004 and align<np.deg2rad(2)
            and actual_down>=.0014 and self.hammered>=.0007 and all(x is True for x in wall)and current_material_good)
  if witness:self.setting_witness={'control':control,'gap_m':gap,'actual_wedge_down_m':actual_down}
  if self.setting_witness and not current_anchor_seated:self.setting_witness=None
  pverts=np.vstack([s['vertices']for s in parts['punch']['geoms']]);world=pverts@R['punch'].T+pos['punch'];zmin=float(((world-top)@axis).min())
  low=world.min(0);high=world.max(0);hc=np.array(G['frame_origin_m']);rad=.00635
  sideclear=float(max(low[0]-hc[0]-rad,hc[0]-rad-high[0],low[1]-hc[1]-rad,hc[1]-rad-high[1]))
  out=zmin>.01 or sideclear>.01
  retained=(self.setting_witness is not None and current_anchor_seated and all(x is True for x in wall))
  # At rest means the body's actual pose changed by less than 5 mm/s and .1 rad/s over the last control step.
  # The simulator's own velocity readback is recorded but not used: for the jammed sleeve and plug it stays
  # at several rad/s while their poses do not change at all.
  speed={}
  for n in ['anchor_ring','setting_wedge','punch','hammer']:
   if n in self.prev:
    p0,R0=self.prev[n];speed[n]=(float(np.linalg.norm(pos[n]-p0))/self.DT,float(Rotation.from_matrix(R[n]@R0.T).magnitude())/self.DT)
   self.prev[n]=(pos[n].copy(),R[n].copy())
  rest=lambda names:all(n in speed and speed[n][0]<.005 and speed[n][1]<.1 for n in names)
  velocity_known_stable=rest(['anchor_ring','setting_wedge'])
  self.withdraw_streak=self.withdraw_streak+1 if retained and out and velocity_known_stable else 0
  released,release_reason=self.released_tools_and_hands(row,G,parts,pos,R)
  tools_at_rest=rest(['punch','hammer'])
  self.release_streak=self.release_streak+1 if retained and out and released and velocity_known_stable and tools_at_rest else 0
  self.value=1. if self.release_streak>=21 else .75 if self.withdraw_streak>=11 else .5 if retained else 0.
  self.latest=dict(primary_score=self.value,setting_witness=self.setting_witness,shoulder_gap_m=gap,actual_wedge_down_m=actual_down,hammer_driven_plug_travel_m=self.hammered,wall_contacts=wall,
    punch_min_axial_clearance_m=zmin,punch_wholebody_lateral_separation_m=sideclear,anchor_origin_shift_m=float(anchor_shift),stable_fresh_streak=self.withdraw_streak,
    anchor_at_rest=bool(velocity_known_stable),tools_at_rest=bool(tools_at_rest),pose_speeds_m_s_rad_s=speed,contacts_unknown=unavailable,tool_release_clearance=released,tool_release_reason=release_reason,release_stable_fresh_streak=self.release_streak,
    limit='Outcome stages 0/.5/.75/1 from actual poses, contacts and robot links; rest is judged from pose change per control step.')
  return self.latest

 def released_tools_and_hands(self,row,G,parts,pos,R):
  # Ordinaryrectangularacceptancezones match the realtwoVblocks andhammercradle.
  # Stablegeometriczones do not use controllerphase or desiredpose.
  p=pos['punch'];h=pos['hammer'];ax=R['punch'][:,2]
  if not(.17<p[0]<.20 and .23<p[1]<.27 and .780<p[2]<.800 and abs(ax[0])>.995):return False,'PunchnotinactualhorizontalVrestzone'
  if np.linalg.norm(h-np.array(parts['hammer']['pos']))>.010 or np.linalg.norm(Rotation.from_matrix(R['hammer']).as_rotvec())>np.deg2rad(5):return False,'Hammernotinactualoriginalcradlezone'
  from yam_arena.analysis import link_hulls
  from scipy.spatial.transform import Rotation as Rot
  target=[]
  for name in ['punch','hammer','anchor_ring']:
   for sh in parts[name]['geoms']:
    v=np.array(sh['vertices'])@R[name].T+pos[name];target.append((v.min(0),v.max(0)))
  low=np.array([x[0]for x in target]);high=np.array([x[1]for x in target]);count=0
  for side in ['left','right']:
   rob=row.get('robots',{}).get(side,{})
   names=rob.get('body_names',[]);poses=rob.get('body_link_pose_w',[])
   if len(names)!=9 or set(names)!=set(link_hulls())or np.array(poses).shape!=(9,7)or not np.isfinite(poses).all():return False,'Missingactualcomplete9linkrobotposes'
   for name,pose in zip(names,poses):
    if name not in link_hulls():return False,'Unknownactualrobotbody'
    rr=Rot.from_quat(pose[3:]).as_matrix()
    for hull in link_hulls()[name]:
     w=hull.vertices@rr.T+pose[:3];count+=1
     # AABBpositiveaxisgap is a sufficient conservative fullgeometryseparation.
     if np.any(np.max(np.maximum(low-w.max(0),w.min(0)-high),axis=1)<.020):return False,'Completebodygeometrynot20mmclear'
  if count!=630:return False,'Incompleteactual630hullcoverage'
  return True,'Bothreleasedinordinarysupportzonesand630actualrobotpieces20mmclear'
