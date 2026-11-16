"""Read-only current motor state for auditing a future contact-bias proposal.

No zero substitute for unavailable effort data. This records current values,
not a bound on every future native substep. The standard YAM action already
uses gravity_compensation_forces; contact bias must not add gravity twice.
"""
import numpy as np
from yam_arena.constants import SHOULDER,WRIST
K=np.array([SHOULDER['stiffness']]*3+[WRIST['stiffness']]*3)
D=np.array([SHOULDER['damping']]*3+[WRIST['damping']]*3)
E=np.array([SHOULDER['effort']]*3+[WRIST['effort']]*3)
def array(v):
 v=v.torch if hasattr(v,'torch')else v
 if hasattr(v,'detach'):return v.detach().cpu().numpy()
 if hasattr(v,'numpy'):return v.numpy()
 return np.asarray(v)
def selected(data,name,ids,n,*,floating=False):
 a=np.asarray(array(getattr(data,name)),float)
 if floating and a.shape==(1,n+6):a=a[:,6:]
 if a.shape!=(1,n)or not np.isfinite(a).all():raise ValueError('invalid '+name)
 return a[0,np.asarray(ids,dtype=int)].copy()
def read_current(base,env,time):
 out={'requested_control_time_s':float(time),'scope':'current measured state/parameters only; not a future-substep effort guarantee','sides':{},'verified':True}
 for side in ('left','right'):
  asset=base.scene[side+'_robot'];ids,_=env._ids[side+'_robot'];data=asset.data;n=int(asset.num_joints)
  r={'verified':False}
  try:
   r.update(q_actual=selected(data,'joint_pos',ids,n).tolist(),joint_velocity_rad_s=selected(data,'joint_vel',ids,n).tolist(),own_gravity_compensation_Nm=selected(data,'gravity_compensation_forces',ids,n,floating=True).tolist(),stiffness_Nm_per_rad=selected(data,'joint_stiffness',ids,n).tolist(),damping_Nm_s_per_rad=selected(data,'joint_damping',ids,n).tolist(),effort_limits_Nm=selected(data,'joint_effort_limits',ids,n).tolist())
   if not(np.allclose(r['stiffness_Nm_per_rad'],K,rtol=0,atol=1e-4)and np.allclose(r['damping_Nm_s_per_rad'],D,rtol=0,atol=1e-4)and np.allclose(r['effort_limits_Nm'],E,rtol=0,atol=1e-4)):raise ValueError('native motor parameters differ from referenced YAM constants')
   r['verified']=True
  except (AttributeError,ValueError,RuntimeError,TypeError)as error:r['unavailable_reason']=str(error)
  try:r['applied_torque_Nm']=selected(data,'applied_torque',ids,n).tolist()
  except (AttributeError,ValueError,RuntimeError,TypeError):r['applied_torque_Nm']=None
  out['sides'][side]=r;out['verified'] &=r['verified']
 return out
