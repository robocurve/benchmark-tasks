"""Read-only filtered opposing normal-force measurement for roll fingertips."""
import numpy as np
from config import CELLS
SENSOR='right_roll_contacts'
TARGETS=['roll',*[f'tape_{i}' for i in range(CELLS)]]

def add_sensor(embodiment):
 from isaaclab.sensors import ContactSensorCfg
 from isaaclab_arena.utils.configclass import make_configclass
 from yam_arena.robot import asset_link_paths
 paths=asset_link_paths();a,b=paths['tip_left'],paths['tip_right']
 if a.rsplit('/',1)[0]!=b.rsplit('/',1)[0]:raise RuntimeError('unverified fingertip hierarchy')
 cfg=embodiment.scene_config;fields=[(n,type(getattr(cfg,n)),getattr(cfg,n)) for n in cfg.__class__.__annotations__]
 sensor=ContactSensorCfg(prim_path='{ENV_REGEX_NS}/RightArm'+a.rsplit('/',1)[0]+'/tip_.*',update_period=0.,track_pose=False,filter_prim_paths_expr=['{ENV_REGEX_NS}/'+name for name in TARGETS])
 embodiment.scene_config=make_configclass('PackingRollContactSceneCfg',fields+[(SENSOR,ContactSensorCfg,sensor)])()

class Reader:
 def __init__(self,base,env):self.sensor=base.scene[SENSOR];self.env=env;self.latest=None
 def __call__(self,side):
  if side!='right':raise ValueError('only right roll fingertip binding exists')
  value=self.sensor.data.force_matrix_w
  value=value.torch if hasattr(value,'torch') else value
  force=value.detach().cpu().numpy() if hasattr(value,'detach') else np.asarray(value)
  names=list(self.sensor.body_names)
  if force.shape!=(1,len(names),len(TARGETS),3) or set(names)!={'tip_left','tip_right'}:raise ValueError('native roll contact shape/body mismatch')
  if not np.isfinite(force).all():raise ValueError('nonfinite native roll normal force')
  # Reading data forces the sensor's current substep buffer update. Both
  # fingertips and every filtered target share this same native readback.
  self.latest=dict(measurement_phase='pre_action',individual_filter_column_order_native_verified=False,time_s=float(self.env._t),body_names=names,target_names=TARGETS,normal_force_matrix_w=force[0].tolist())
  return self.latest
