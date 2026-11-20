"""Reference-only check that a held tool is still in the hand; never controls an object pose.

A tool rocking a few degrees between the pads during a blow, or a punch sliding a millimetre or two as the wedge
drives down, is ordinary and is only recorded. The reference stops when the tool has really left the grasp: the
grasp point has moved more than 15 mm along the tool or the tool has turned more than 30 degrees in the hand for
three controls running, or a pad has had no contact with it for six controls running.
"""
import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.constants import TCP_OFFSET

class MaterialGraspGuard:
 SLIP_M=.015;TURN_RAD=float(np.deg2rad(30));SLIP_CONTROLS=3;CONTACT_CONTROLS=6
 def __init__(self):self.anchors={};self.last={};self.out={};self.open={}
 def observe(self,row,side,name,pads_touching):
  r=row['robots'][side];bn=r['body_names'];poses=np.array(r['body_link_pose_w'],float)
  if len(bn)!=9 or poses.shape!=(9,7)or not np.isfinite(poses).all():raise ValueError('Invalidactualfullrobotpose')
  hand=poses[bn.index('gripper')];obj=np.array(row['rigid_bodies'][name]['pose_xyzw'],float)
  if obj.shape!=(7,)or not np.isfinite(obj).all():raise ValueError('Invalidactualheldbodypose')
  Rh=Rotation.from_quat(hand[3:]).as_matrix();Ro=Rotation.from_quat(obj[3:]).as_matrix();tcp=hand[:3]+Rh@np.array(TCP_OFFSET)
  # Tool-frame coordinates of the actual TCP and the actual relative orientation, not targets.
  x=Ro.T@(tcp-obj[:3]);rel=Rh.T@Ro;key=(side,name);c=row['control']
  if key not in self.anchors:self.anchors[key]=(x.copy(),rel.copy(),c)
  x0,R0,c0=self.anchors[key]
  previous=self.last.get(key)
  if previous is not None and c<=previous:raise ValueError('Graspguardrequiresfreshactualrow')
  self.last[key]=c;shift=float(np.linalg.norm(x-x0));angle=float(Rotation.from_matrix(rel@R0.T).magnitude())
  self.out[key]=self.out.get(key,0)+1 if shift>self.SLIP_M or angle>self.TURN_RAD else 0
  self.open[key]=0 if pads_touching else self.open.get(key,0)+1
  out=dict(actual_control=c,side=side,object=name,anchor_actual_control=c0,TCP_in_object_m=x.tolist(),material_point_drift_m=shift,
           held_rotation_drift_rad=angle,pads_touching=bool(pads_touching),limits_m_rad=[self.SLIP_M,self.TURN_RAD],no_object_pose_or_force_write=True)
  if self.out[key]>=self.SLIP_CONTROLS:raise GraspFailure('Held tool left the grasp',out)
  if self.open[key]>=self.CONTACT_CONTROLS:raise GraspFailure('Held tool lost pad contact',out)
  return out
class GraspFailure(RuntimeError):
 def __init__(self,reason,measurement):self.measurement=measurement;super().__init__(reason)
