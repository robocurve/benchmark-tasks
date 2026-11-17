"""Ending-state mechanical assembly, with actual contact-based release and stability."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
ACTIVE=None

def neck_clearance(relative, axis):
 """Conservative shaft fit in the actual32-sided neck bore, in metres.

 Check both ends of the10mm neck, coupling eccentricity and tilt. The shaft's
 oblique circular section is an ellipse; independent origin/angle bounds alone
 would accept a shaft that crosses the neck wall. A circular outer envelope is
 conservative for the32-sided shaft collider.
 """
 if axis[1]<=.1:return -np.inf
 angles=(np.arange(32)+.5)*2*np.pi/32
 normals=np.column_stack([np.cos(angles),np.sin(angles)])
 extent=.0044*np.sqrt(1+(normals@axis[[0,2]]/axis[1])**2)
 apothem=.0046*np.cos(np.pi/32)
 return min(float(np.min(apothem-(normals@(relative+axis*((y-relative[1])/axis[1]))[[0,2]]+extent))) for y in (.014,.024))

CONTACT_ALLOWANCE=.00005  # Numerical surface tolerance, not a human measurement.

def plate_neck_clearance(cup,hole):
 """Native32-sided neck inside the48-sided chamfer/bore, over its actual length.

 Clip each longitudinal neck edge at both plate faces and the chamfer break.
 Each resulting region is convex, so endpoint half-space checks suffice.
 """
 rc=Rotation.from_quat(cup[3:]);angles=np.arange(32)*2*np.pi/32
 ends=np.array([np.column_stack([.0083*np.cos(angles),np.full(32,y),.0083*np.sin(angles)]) for y in (.014,.024)])
 world=rc.apply(ends.reshape(-1,3)).reshape(2,32,3)+cup[:3]-hole
 direction=world[1]-world[0];dy=direction[:,1]
 if np.any(dy<=1e-7):return -np.inf
 normals=np.column_stack([np.cos((np.arange(48)+.5)*2*np.pi/48),np.sin((np.arange(48)+.5)*2*np.pi/48)])
 clearances=[]
 for low,high in [(-.002,-.001),(-.001,.002)]:
  start=np.maximum(0,(low-world[0,:,1])/dy);end=np.minimum(1,(high-world[0,:,1])/dy);valid=start<=end
  for t in (start,end):
   points=(world[0]+t[:,None]*direction)[valid]
   if not len(points):continue
   radius=np.where(points[:,1]<-.001,.009+(-.001-points[:,1]),.009)
   clearances.append(float(np.min(radius[:,None]*np.cos(np.pi/48)-points[:,[0,2]]@normals.T)))
 return min(clearances) if clearances else -np.inf

class Outcome:
 def __init__(self,paths,geometry,contacts):
  self.paths=paths;self.g=geometry;self.c=contacts;self.value=0.;self.history=deque(maxlen=int(3*geometry['physics_hz'])+1);self.detail={};self.unsafe=False
  self.cup=paths.index('/World/envs/env_0/vac_cup');self.core=paths.index('/World/envs/env_0/vac_core');self.barbs=[paths.index('/World/envs/env_0/vac_barb'+str(i)) for i in range(4)]
 def update(self,p,robot,contacts,environment):
  if not np.isfinite(p).all():raise RuntimeError('Invalid scorer state')
  # Only robot-to-robot and original table are prohibited, not intended workpiece contact.
  self.unsafe |= bool(np.sum(environment[:,:18]) or np.sum(environment[:,-1]))
  cup=p[self.cup];core=p[self.core];rc=Rotation.from_quat(cup[3:]);rr=Rotation.from_quat(core[3:]);relative=rc.inv().apply(core[:3]-cup[:3])
  tilt=float(np.rad2deg(np.arccos(np.clip(rc.as_matrix()[1,1],-1,1))))
  coretilt=float(np.rad2deg(np.arccos(np.clip(np.dot(rc.as_matrix()[:,1],rr.as_matrix()[:,1]),-1,1))))
  target=np.asarray(self.g['cup_target']);delta=cup[:3]-target
  hole=np.asarray(self.g.get('hole_center',[target[0],0.,target[2]]));axis=rc.as_matrix()[:,1]
  intersection=cup[:3]+axis*((hole[1]-cup[1])/axis[1]) if abs(axis[1])>.1 else np.full(3,999.)
  neck_error=float(np.linalg.norm((intersection-hole)[[0,2]]))
  radii=[];spring_errors=[]
  for i,index in enumerate(self.barbs):
   v=rc.inv().apply(p[index,:3]-cup[:3]);direction=np.array([np.cos(i*np.pi/2),0,np.sin(i*np.pi/2)])
   radial=float(v@direction)
   corner=self.g.get('head_retention_corner_mm')
   radius=float(np.hypot(corner[0]/1000+radial,corner[1]/1000)) if corner is not None else .0105+radial
   radii.append(radius);spring_errors.append(float(np.linalg.norm(v-radial*direction)))
  angles=np.arange(32)*2*np.pi/32
  shoulder=rc.apply(np.column_stack([.011*np.cos(angles),np.full(32,.0214),.011*np.sin(angles)]))+cup[:3]
  shoulder_before_plate=bool(np.max(shoulder[:,1]-hole[1])<=-.002+CONTACT_ALLOWANCE)
  corners=[]
  for index in self.barbs:
   # Child orientation carries each sector's original in-body angular layout.
   i=self.barbs.index(index);a=i*np.pi/2
   base=np.sqrt(.0105**2-.0035**2)
   points=np.array([[base,.026,-.0035],[base,.026,.0035]])
   rotation=np.array([[np.cos(a),0,-np.sin(a)],[0,1,0],[np.sin(a),0,np.cos(a)]])
   corners.extend(Rotation.from_quat(p[index,3:]).apply(points@rotation.T)+p[index,:3])
  corners=np.asarray(corners)-hole
  head_behind_plate=bool(np.min(corners[:,1])>=.002-CONTACT_ALLOWANCE and np.min(np.linalg.norm(corners[:,[0,2]],axis=1))>.009)
  plate_clearance=plate_neck_clearance(cup,hole)
  cup_ok=bool(shoulder_before_plate and head_behind_plate and plate_clearance>=-CONTACT_ALLOWANCE)
  bore_clearance=neck_clearance(relative,(rc.inv()*rr).apply([0.,1.,0.]))
  axis=(rc.inv()*rr).apply([0.,1.,0.])
  # Cap lower face must meet the head tips; the rib's rear face must have
  # passed the inner lip. Actual shaft fit rejects offset/tilted wall crossing.
  cap_gap=float((relative-axis*.0009)[1]-.030)
  rib_angles=np.arange(24)*2*np.pi/24
  core_rim=relative+(rc.inv()*rr).apply(np.column_stack([.0053*np.cos(rib_angles),np.full(24,-.0055),.0053*np.sin(rib_angles)]))
  rib_passed=bool(np.max(core_rim[:,1])<=.026+CONTACT_ALLOWANCE)
  lip_overlaps=[]
  for i,index in enumerate(self.barbs):
   direction=np.array([np.cos(i*np.pi/2),0,np.sin(i*np.pi/2)])
   child=rc.inv().apply(p[index,:3]-cup[:3]);inner=.0045+float(child@direction)
   lip_overlaps.append(float(np.max(core_rim@direction)-inner))
  core_ok=bool(-CONTACT_ALLOWANCE<=cap_gap<.00075 and rib_passed and min(lip_overlaps)>1e-9 and bore_clearance>=-CONTACT_ALLOWANCE)
  released=bool(np.sum(robot)==0)
  # Actual plate support: body contacts filter positions follow all moving paths then fixed plate.
  supported=bool(np.sum(contacts[[self.cup]+self.barbs,len(self.paths):-1])>0)
  core_supported=bool(np.sum(contacts[self.core,[self.cup]+self.barbs])>0)
  self.history.append((p.copy(),cup_ok and core_ok and released and supported and core_supported))
  hold=len(self.history)==self.history.maxlen and all(x[1] for x in self.history)
  drift=999.
  if hold:
   hist=np.array([x[0][[self.cup,self.core]] for x in self.history]);drift=float(np.linalg.norm(hist[:,:,:3]-hist[0,:,:3],axis=-1).max())
   angular=0.
   for j in range(2):
    axes=Rotation.from_quat(hist[:,j,3:]).apply(np.tile([0.,1.,0.],(len(hist),1)))
    angular=max(angular,float(np.arccos(np.clip(axes@axes[0],-1,1)).max()))
   # Motion within the installed fit is permitted. Neither drift nor native
   # locked-axis error is a human outcome condition; integrity is audited separately.
  self.value=0. if self.unsafe else 1. if hold else .75 if cup_ok and core_ok else .5 if cup_ok else 0.
  self.detail=dict(score=self.value,cup_seated=cup_ok,core_seated=core_ok,released=released,cup_supported=supported,core_supported=core_supported,stable_3s=bool(hold),unsafe=self.unsafe,cup_error_mm=(delta*1000).tolist(),neck_axis_error_mm=neck_error*1000,cup_tilt_deg=tilt,core_tilt_deg=coretilt,core_relative_mm=(relative*1000).tolist(),neck_clearance_mm=float(bore_clearance*1000) if np.isfinite(bore_clearance) else None,plate_neck_clearance_mm=float(plate_clearance*1000) if np.isfinite(plate_clearance) else None,shoulder_before_plate=shoulder_before_plate,head_behind_plate=head_behind_plate,cap_gap_mm=cap_gap*1000,rib_passed=rib_passed,lip_overlap_mm=(np.array(lip_overlaps)*1000).tolist(),head_radii_mm=(np.array(radii)*1000).tolist(),spring_orthogonal_error_mm=max(spring_errors)*1000,held_drift_mm=drift*1000)
