"""Disclosed clock-only action replay. No scene, FK, physics or object feedback."""
import ast,hashlib,json,math
from pathlib import Path
import numpy as np


class Clock:
 def __init__(self,folder=None):
  folder=Path(folder) if folder is not None else Path(__file__).parent
  raw=(folder/'clock_actions.json').read_bytes();meta_raw=(folder/'clock_metadata.json').read_bytes()
  namespace={}
  for node in ast.parse((folder/'clock_manifest.py').read_text()).body:
   if not isinstance(node,ast.Assign) or len(node.targets)!=1 or not isinstance(node.targets[0],ast.Name) or node.targets[0].id not in ('DATA_SHA256','METADATA_SHA256'):raise ValueError('invalid clock manifest')
   key=node.targets[0].id
   if key in namespace:raise ValueError('duplicate clock manifest key')
   value=ast.literal_eval(node.value)
   if not isinstance(value,str) or len(value)!=64 or any(c not in '0123456789abcdef' for c in value):raise ValueError('invalid clock manifest digest')
   namespace[key]=value
  if set(namespace)!= {'DATA_SHA256','METADATA_SHA256'}:raise ValueError('incomplete clock manifest')
  if hashlib.sha256(raw).hexdigest()!=namespace['DATA_SHA256'] or hashlib.sha256(meta_raw).hexdigest()!=namespace['METADATA_SHA256']:raise ValueError('frozen clock dataset hash mismatch')
  self.meta=json.loads(meta_raw);self.actions=np.asarray(json.loads(raw),dtype=float);n=self.meta['action_count'];self.total=self.meta['total_steps']
  if not 1<=n<=self.total<=2400 or self.meta['control_hz']!=20 or self.actions.shape!=(n,14) or not np.isfinite(self.actions).all():raise ValueError('invalid clock action protocol')
  low=np.asarray(self.meta['action_lower']);high=np.asarray(self.meta['action_upper'])
  if low.shape!=(14,) or high.shape!=(14,) or not np.isfinite([low,high]).all() or np.any(self.actions<low-1e-6) or np.any(self.actions>high+1e-6):raise ValueError('clock actions outside original bounds')
  self.actions.setflags(write=False);self.reset()
 def reset(self):self.index=None
 def command(self,state_time):
  if isinstance(state_time,(bool,np.bool_)) or not isinstance(state_time,(int,float,np.integer,np.floating)) or not math.isfinite(state_time) or state_time<0:raise ValueError('finite clock required')
  i=round(float(state_time)*20)
  if abs(i/20-float(state_time))>1e-6 or not 0<=i<self.total:raise ValueError('clock outside bounded20Hz dataset')
  if self.index is None and i!=0:raise ValueError('fresh time-zero reset required')
  if self.index is not None and i not in (self.index,self.index+1):raise ValueError('clock skipped or reversed')
  self.index=i;return self.actions[min(i,len(self.actions)-1)].copy(),i==self.total-1


def register_policy():
 from inspect_robots.policy import PolicyBase,PolicyInfo
 from inspect_robots.registry import register
 from inspect_robots_yam.config import action_box
 from inspect_robots.types import Action,ActionChunk
 class Recorded(PolicyBase):
  def __init__(self):
   from gripper_waypoints import gripper_waypoints
   self.clock=Clock()
   self.gripper_targets=gripper_waypoints(self.clock.actions[:,[6,13]])
   self.info=PolicyInfo(name='ethernet_recorded',action_space=action_box(np.array(self.clock.meta['action_lower']),np.array(self.clock.meta['action_upper'])))
  def bind(self,info):
   if float(info.control_hz)!=20:raise ValueError('clock requires20Hz')
  def reset(self,scene):self.clock.reset()
  def act(self,observation):
   command,stop=self.clock.command(observation.state_time)
   index=min(self.clock.index,len(self.gripper_targets)-1)
   command[[6,13]]=self.gripper_targets[index]
   return ActionChunk(actions=[Action(data=command,meta={'request_stop':True} if stop else {})],control_hz=20.)
 register('policy','ethernet_recorded')(Recorded)
 return 'ethernet_recorded'
