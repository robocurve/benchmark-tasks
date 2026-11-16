"""Read-only fingertip normal forces filtered to the twelve carton panels.

Column ordering is not asserted. Only the sum over the entire named panel
family is interpreted; vectors are forces on the sensor fingertips, not on the
carton. This reader never applies force. Controllers may use its verified measurements to gate contact acquisition.
"""
import numpy as np
TARGETS=('north','east','south','west','north_top','east_top','south_top','west_top','north_bottom','east_bottom','south_bottom','west_bottom')
SIDES=('left','right')
def add_sensors(embodiment):
 from isaaclab.sensors import ContactSensorCfg
 from isaaclab_arena.utils.configclass import make_configclass
 from yam_arena.robot import asset_link_paths
 paths=asset_link_paths();a,b=paths['tip_left'],paths['tip_right']
 if a.rsplit('/',1)[0]!=b.rsplit('/',1)[0]:raise RuntimeError('unverified fingertip hierarchy')
 cfg=embodiment.scene_config;fields=[(n,type(getattr(cfg,n)),getattr(cfg,n))for n in cfg.__class__.__annotations__]
 for side,root in [('left','LeftArm'),('right','RightArm')]:
  sensor=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/'+root+a.rsplit('/',1)[0]+'/tip_.*',update_period=0.,track_pose=False,filter_prim_paths_expr=['{ENV_REGEX_NS}/'+n for n in TARGETS])
  fields.append((side+'_carton_contacts',ContactSensorCfg,sensor))
 embodiment.scene_config=make_configclass('CartonTurnoverContactCfg',fields)()
def summarize(force,names):
 force=np.asarray(force,float)
 if force.shape!=(1,2,len(TARGETS),3)or len(names)!=2 or set(names)!={'tip_left','tip_right'}:raise ValueError('carton contact shape/body mismatch')
 if not np.isfinite(force).all():raise ValueError('nonfinite carton contact force')
 net=force[0].sum(axis=(0,1))
 return dict(body_names=list(names),configured_target_family=list(TARGETS),individual_filter_column_order_native_verified=False,normal_force_matrix_w=force[0].tolist(),family_normal_force_on_fingertips_w_N=net.tolist(),family_normal_force_norm_N=float(np.linalg.norm(net)),family_sum_of_normal_force_magnitudes_N=float(np.linalg.norm(force[0],axis=-1).sum()))
def native_array(value):
 value=value.torch if hasattr(value,'torch') else value
 if hasattr(value,'detach'):return value.detach().cpu().numpy()
 if hasattr(value,'numpy'):return value.numpy()
 return np.asarray(value)
class Reader:
 def __init__(self,base):
  self.sensors={s:base.scene[s+'_carton_contacts']for s in SIDES};self.physics_dt=float(base.sim.get_physics_dt());self.last_clocks=None;self.last_requested=None;self.sample_id=0
  if not np.isfinite(self.physics_dt)or not 0<self.physics_dt<=1/120:raise ValueError('unsupported native sensor timestep')
 def read(self,time,phase='post_step'):
  time=float(time)
  if not np.isfinite(time)or time<0:raise ValueError('invalid force measurement request time')
  result={};clocks={};updates={};tol=max(2e-6,self.physics_dt*.02)
  for side,sensor in self.sensors.items():
   result[side]=summarize(native_array(sensor.data.force_matrix_w),list(sensor.body_names))
   for field,dest in [('_timestamp',clocks),('_timestamp_last_update',updates)]:
    if not hasattr(sensor,field):raise ValueError('native sensor clock contract unavailable: '+field)
    value=np.asarray(native_array(getattr(sensor,field)),float)
    if value.shape!=(1,)or not np.isfinite(value).all()or value[0]<0:raise ValueError('invalid native sensor clock')
    dest[side]=float(value[0])
   if abs(clocks[side]-updates[side])>tol:raise ValueError('native contact buffer was not refreshed to sensor time')
  if abs(clocks['left']-clocks['right'])>tol:raise ValueError('left/right force clocks disagree')
  if self.last_clocks is not None:
   requested_delta=time-self.last_requested
   if requested_delta<-tol:raise ValueError('force request clock moved backwards')
   deltas=[clocks[s]-self.last_clocks[s]for s in SIDES]
   if any(dt<-tol or abs(dt-requested_delta)>tol for dt in deltas):raise ValueError('stale/reset/unsynchronised native force sample')
   new=all(dt>tol for dt in deltas)
  else:new=True
  if new:self.sample_id+=1
  self.last_clocks=clocks.copy();self.last_requested=time
  return dict(requested_control_time_s=time,sensor_clock_s=clocks,sensor_update_time_s=updates,sample_id=self.sample_id,clock_verified=True,native_clock_fields=['_timestamp','_timestamp_last_update'],measurement_phase=phase,force_sample_interval='latest native sensor substep, not control-interval impulse',sides=result)
